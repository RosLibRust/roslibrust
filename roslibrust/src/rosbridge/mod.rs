// Subscriber is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod subscriber;
pub use subscriber::*;

// Publisher is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod publisher;
pub use publisher::*;

// Client is a transparent module, we directly expose internal types
// Module exists only to organize source code
mod client;
pub use client::*;

// Tests are fully private module
#[cfg(test)]
mod integration_tests;

/// Communication primitives for the rosbridge_suite protocol
mod comm;

use crate::{RosMessageType, RosServiceType};
use anyhow::anyhow;
use comm::RosBridgeComm;
use dashmap::DashMap;
use futures_util::stream::{SplitSink, SplitStream, StreamExt};
use log::*;
use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde_json::Value;
use std::collections::HashMap;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::net::TcpStream;
use tokio::sync::RwLock;
use tokio::time::Duration;
use tokio_tungstenite::*;
use tungstenite::Message;

/// For now starting with a central error type, may break this up more in future
#[derive(thiserror::Error, Debug)]
pub enum RosLibRustError {
    #[error("Not currently connected to ros master / bridge")]
    Disconnected,
    // TODO we probably want to eliminate tungstenite from this and hide our
    // underlying websocket implementation from the API
    // currently we "technically" break the API when we change tungstenite verisons
    #[error("Websocket communication error: {0}")]
    CommFailure(tokio_tungstenite::tungstenite::Error),
    #[error("Operation timed out: {0}")]
    Timeout(#[from] tokio::time::error::Elapsed),
    #[error("Failed to parse message from JSON: {0}")]
    InvalidMessage(#[from] serde_json::Error),
    // Generic catch-all error type for not-yet-handled errors
    // TODO ultimately this type will be removed from API of library
    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}

/// Provides an implementation tranlating the underlying websocket error into our error type
impl From<tokio_tungstenite::tungstenite::Error> for RosLibRustError {
    fn from(e: tokio_tungstenite::tungstenite::Error) -> Self {
        // TODO we probably want to expand this type and do some matching here
        RosLibRustError::CommFailure(e)
    }
}

/// Generic result type used as standard throughout library.
/// Note: many functions which currently return this will be updated to provide specific error
/// types in the future instead of the generic error here.
type RosLibRustResult<T> = Result<T, RosLibRustError>;

/// Used for type erasure of message type so that we can store arbitrary handles
type Callback = Box<dyn Fn(&str) -> () + Send + Sync>;

/// Type erasure of callback for a service
/// Internally this will covert the input string to the Request type
/// Send that converted type into the user's callback
/// Get the result of the user's callback and then serialize that so it can be transmitted
// TODO reconsider use of serde_json::Value here vs. tungstenite::Message vs. &str
// Not quite sure what type we want to erase to?
// I can make a good argument for &str because that should be generic even if we switch
// backends - Carter 2022-10-6
type ServiceCallback = Box<
    dyn Fn(&str) -> Result<serde_json::Value, Box<dyn std::error::Error + Send + Sync>>
        + Send
        + Sync,
>;

/// The handle returned to the caller of advertise_service this struct represents the lifetime
/// of the service, and dropping this struct automatically unadvertises and removes the service.
/// No interaction with this struct is expected beyond managing its lifetime.
pub struct ServiceHandle {
    /// Reference back to the client this service originated so we can notify it when we are dropped
    client: ClientHandle,
    /// Topic that the service is served on, this is used to uniquely identify it, only one service
    /// may exist for a given topic (per client, we can't control it on the ROS side)
    topic: String,
}

/// Service handles automatically unadvertise their service when dropped.
impl Drop for ServiceHandle {
    fn drop(&mut self) {
        self.client.unadvertise_service(&self.topic);
    }
}

/// Our underlying communication socket type (maybe move to comm?)
type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

/// We split our underlying socket into two halves with seperate locks on read and write.
/// This is the read half.
type Reader = SplitStream<Socket>;

/// We split our underlying socket into two halves with seperate locks on read and write.
/// This is the write half.
type Writer = SplitSink<Socket, Message>;

/// Topics have a fundamental queue *per subscriber* this is te queue type used for each subscriber.
type MessageQueue<T> = deadqueue::limited::Queue<T>;

// TODO queue size should be configurable for subscribers
const QUEUE_SIZE: usize = 1_000;

/// Internal tracking structure used to maintain information about each subscribtion our client has
/// with rosbridge.
struct Subscription {
    /// Map of "subscriber id" -> callback
    /// Subscriber ids are randomly generated
    /// There will be one callback per subscriber to the topic.
    // Note: don't need dashmap here as the subscription is already inside a dashmap
    pub handles: HashMap<uuid::Uuid, Callback>,
    /// Name of ros type (package_name/message_name), used for re-subscribes
    pub topic_type: String,
}

struct PublisherHandle {
    #[allow(dead_code)]
    topic: String,
    #[allow(dead_code)]
    msg_type: String,
}

/// Builder options for creating a client
#[derive(Clone)]
pub struct ClientHandleOptions {
    url: String,
    timeout: Option<Duration>,
}

impl ClientHandleOptions {
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    pub fn new<S: Into<String>>(url: S) -> ClientHandleOptions {
        ClientHandleOptions {
            url: url.into(),
            timeout: None,
        }
    }

    /// Configures a default timeout for all operations.
    /// Underlying communication implementations may define their own timeouts, this options does
    /// not affect those timeouts, but adds an additional on top to prempt any operations.
    pub fn timeout<T: Into<Duration>>(mut self, duration: T) -> ClientHandleOptions {
        self.timeout = Some(duration.into());
        self
    }
}

/// The ClientHandle is the fundamental object through which users of this library are expected to interact with it.
///
/// Creating a new ClientHandle will create an underlying connection to rosbridge and spawn an async connection task,
/// which is responsible for contiuously managing that connection and attempts to re-establish the connection if it goes down.
///
/// ClientHandle is clone and multiple handles can be clone()'d from the origional and passed throughout your application.
/// ```no_run
/// # // TODO figure out how to de-duplicate code here with this message definition...
/// # mod std_msgs {
/// # #[allow(non_snake_case)]
/// # #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
/// # pub struct Header {
/// #     pub r#seq: u32,
/// #     pub r#stamp: ::roslibrust::integral_types::Time,
/// #     pub r#frame_id: std::string::String,
/// # }
/// # impl ::roslibrust::RosMessageType for Header {
/// #     const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
/// # }
/// # impl Header {}
/// # }
/// # #[tokio::main]
/// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
///   // Create a new client
///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
///   // Create a copy of the handle (does not create a seperate connection)
///   let mut handle2 = handle.clone();
///   tokio::spawn(async move {
///     let subscription = handle.subscribe::<std_msgs::Header>("/topic").await.unwrap();
///   });
///   tokio::spawn(async move{
///     let subscription = handle2.subscribe::<std_msgs::Header>("/topic").await.unwrap();
///   });
///   # Ok(())
/// # }
/// // Both tasks subscribe to the same topic, but since the use the same underlying client only one subscription is made to rosbridge
/// // Both subscribers will recieve a copy of each message recieved on the topic
/// ```
#[derive(Clone)]
pub struct ClientHandle {
    inner: Arc<RwLock<Client>>,
    is_disconnected: Arc<AtomicBool>,
}

impl ClientHandle {
    /// Creates a new client handle with configurable options.
    ///
    /// Use this method if you need more control than [ClientHandle::new] provides.
    /// Like [ClientHandle::new] this function does not resolve until the connection is established for the first time.
    /// This function respects the [ClientHandleOptions] timeout and will return with an error if a connection is not
    /// established within the timeout.
    async fn new_with_options(opts: ClientHandleOptions) -> RosLibRustResult<Self> {
        let inner = Arc::new(RwLock::new(timeout(opts.timeout, Client::new(opts)).await?));
        let inner_weak = Arc::downgrade(&inner);

        // We connect when we create Client
        let is_disconnected = Arc::new(AtomicBool::new(false));

        // Spawn the spin task
        // The internal stubborn spin task continues to try to reconnect on failure
        let _ = tokio::task::spawn(stubborn_spin(inner_weak, is_disconnected.clone()));

        Ok(ClientHandle {
            inner,
            is_disconnected,
        })
    }

    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// When awaited will not resolve until connection is succesfully made.
    pub async fn new<S: Into<String>>(url: S) -> RosLibRustResult<Self> {
        Self::new_with_options(ClientHandleOptions::new(url)).await
    }

    fn check_for_disconnect(&self) -> RosLibRustResult<()> {
        match self.is_disconnected.load(Ordering::Relaxed) {
            false => Ok(()),
            true => Err(RosLibRustError::Disconnected),
        }
    }

    // Internal implementation of subscribe
    async fn _subscribe<Msg>(&self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        // Lookup / create a subscription entry for tracking
        let client = self.inner.read().await;
        let mut cbs = client
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(Subscription {
                handles: HashMap::new(),
                topic_type: Msg::ROS_TYPE_NAME.to_string(),
            });

        // TODO Possible bug here? We send a subscribe message each time even if already subscribed
        // Send subscribe message to rosbridge to initiate it sending us messages
        let mut stream = client.writer.write().await;
        stream.subscribe(&topic_name, &Msg::ROS_TYPE_NAME).await?;

        // Create a new watch channel for this topic
        let queue = Arc::new(MessageQueue::new(QUEUE_SIZE));

        // Move the tx into a callback that takes raw string data
        // This allows us to store the callbacks generic on type, Msg conversion is embedded here
        let topic_name_copy = topic_name.to_string();
        let queue_copy = queue.clone();
        let send_cb = Box::new(move |data: &str| {
            let converted = match serde_json::from_str::<Msg>(data) {
                Err(e) => {
                    // TODO makes sense for callback to return Result<>, instead of this handling
                    // Should do better error propogation
                    error!(
                        "Failed to deserialize ros message: {:?}. Message will be skipped!",
                        e
                    );
                    return;
                }
                Ok(t) => t,
            };

            match queue_copy.try_push(converted) {
                Ok(()) => {
                    // Msg queued succesfully
                }
                Err(msg) => {
                    info!(
                        "Queue on topic {} is full attempting to drop oldest messgae",
                        &topic_name_copy
                    );
                    let _dropped = queue_copy.try_pop();
                    // Retry pushing into queue
                    match queue_copy.try_push(msg) {
                        Ok(()) => {
                            trace!("Msg was queued succesfully after dropping front");
                        }
                        Err(msg) => {
                            // We don't expect to see this, the only way this should be possible
                            // would be if due to a race condition a message was inserted into queue
                            // between the try_pop and try_push.
                            // This closure should be the only place where push occurs, so this is not
                            // expected
                            error!(
                                "Msg was dropped during recieve because queue could not be emptied: {:?}", msg
                            );
                        }
                    }
                }
            }
        });

        // Create subscriber
        let sub = Subscriber::new(self.clone(), queue, topic_name.to_string());

        // Store callback in map under the subscriber's id
        cbs.handles.insert(*sub.get_id(), send_cb);

        Ok(sub)
    }

    /// Subscribe to a given topic expecting msgs of provided type.
    /// ```no_run
    /// # // TODO figure out how to de-duplicate code here with this message definition...
    /// # mod std_msgs {
    /// # #[allow(non_snake_case)]
    /// # #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    /// # pub struct Header {
    /// #     pub r#seq: u32,
    /// #     pub r#stamp: ::roslibrust::integral_types::Time,
    /// #     pub r#frame_id: std::string::String,
    /// # }
    /// # impl ::roslibrust::RosMessageType for Header {
    /// #     const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
    /// # }
    /// # impl Header {}
    /// # }
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    ///   // Create a new client
    ///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
    ///   // Subscribe using ::<T> style
    ///   let subscriber1 = handle.subscribe::<std_msgs::Header>("/topic").await?;
    ///   // Subscribe using explicit type style
    ///   let subscriber2: roslibrust::Subscriber<std_msgs::Header> = handle.subscribe::<std_msgs::Header>("/topic").await?;
    ///   # Ok(())
    /// # }
    /// ```
    /// This function returns after a subscribe message has been sent to rosbridge, it will
    /// return immediately with an error if call while currently disconnected.
    ///
    /// It does not error if subscribed type does not match the topic type or check this in anyway.
    /// If a type different that what is expected on the topic is published the deserialization of that message will fail,
    /// and the returned subscriber will simply not recieve that message.
    /// Roslibrust will log an error which can be used to detect this situtation.
    /// This can be useful to subscribe to the same topic with multiple different types and whichever
    /// types succesfully deserialize the message will recieve a message.
    ///
    /// ```no_run
    /// # // TODO figure out how to de-duplicate code here with this message definition...
    /// # mod ros1 {
    /// # pub mod std_msgs {
    /// # #[allow(non_snake_case)]
    /// # #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    /// # pub struct Header {
    /// #   pub stamp: f64,
    /// # }
    /// # impl ::roslibrust::RosMessageType for Header {
    /// #     const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
    /// # }
    /// # }
    /// # }
    /// # mod ros2 {
    /// # pub mod std_msgs {
    /// # #[allow(non_snake_case)]
    /// # #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    /// # pub struct Header {
    /// #   pub stamp: f64,
    /// # }
    /// # impl ::roslibrust::RosMessageType for Header {
    /// #     const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
    /// # }
    /// # }
    /// # }
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    ///   // Create a new client
    ///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
    ///   // Subscribe to the same topic with two different types
    ///   let ros1_subscriber = handle.subscribe::<ros1::std_msgs::Header>("/topic").await?;
    ///   let ros2_subscriber = handle.subscribe::<ros2::std_msgs::Header>("/topic").await?;
    ///   // Await both subscribers and get a result back from whichever succeeds at deserializing
    ///   let time = tokio::select!(
    ///     r1_msg = ros1_subscriber.next() => r1_msg.stamp,
    ///     r2_msg = ros2_subscriber.next() => r2_msg.stamp,
    ///   );
    /// # Ok(())
    /// # }
    /// ```
    pub async fn subscribe<Msg>(&mut self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        self.check_for_disconnect()?;
        timeout(
            self.inner.read().await.opts.timeout,
            self._subscribe(topic_name),
        )
        .await
    }

    /// Publishes a message
    pub(crate) async fn publish<T>(&self, topic: &str, msg: T) -> RosLibRustResult<()>
    where
        T: RosMessageType,
    {
        self.check_for_disconnect()?;
        let client = self.inner.read().await;
        let mut stream = client.writer.write().await;
        debug!("Publish got write lock on comm");
        stream.publish(topic, msg).await?;
        Ok(())
    }

    pub async fn advertise<T>(&self, topic: &str) -> RosLibRustResult<Publisher<T>>
    where
        T: RosMessageType,
    {
        self.check_for_disconnect()?;
        let client = self.inner.read().await;
        if client.publishers.contains_key(topic) {
            // TODO if we ever remove this restriction we should still check types match
            return Err(RosLibRustError::Unexpected(anyhow!(
                "Attempted to create two publisher to same topic, this is not supported"
            )));
        } else {
            client.publishers.insert(
                topic.to_string(),
                PublisherHandle {
                    topic: topic.to_string(),
                    msg_type: T::ROS_TYPE_NAME.to_string(),
                },
            );
        }

        {
            let mut stream = client.writer.write().await;
            debug!("Advertise got lock on comm");
            stream.advertise::<T>(topic).await?;
        }
        Ok(Publisher::new(topic.to_string(), self.clone()))
    }

    /// Calls a ros service and returns the response
    ///
    /// Service calls can fail if communication is interrupted.
    /// This method is currently unaffected by the clients Timeout configuration.
    ///
    /// Roadmap:
    ///   - Provide better error information when a service call fails
    ///   - Integrate with ClientHandle's timeout better
    pub async fn call_service<Req: RosMessageType, Res: RosMessageType>(
        &self,
        service: &str,
        req: Req,
    ) -> RosLibRustResult<Res> {
        self.check_for_disconnect()?;
        let (tx, rx) = tokio::sync::oneshot::channel();
        let rand_string: String = thread_rng()
            .sample_iter(&Alphanumeric)
            .take(30)
            .map(char::from)
            .collect();
        let client = self.inner.read().await;
        {
            if client
                .service_calls
                .insert(rand_string.clone(), tx)
                .is_some()
            {
                error!("ID collision encountered in call_service");
            }
        }
        {
            let mut comm = client.writer.write().await;
            timeout(
                client.opts.timeout,
                comm.call_service(service, &rand_string, req),
            )
            .await?;
        }

        // Having to do manual timeout logic here because of error types
        let recv = if let Some(timeout) = client.opts.timeout {
            tokio::time::timeout(timeout, rx).await?
        } else {
            rx.await
        };

        let msg = match recv {
            Ok(msg) => msg,
            Err(e) =>
            // TODO remove panic! here, this could result from dropping communication, need to handle disconnect better
            panic!("The sender end of a service channel was dropped while rx was being awaited, this should not be possible: {}", e),
        };
        Ok(serde_json::from_value(msg)?)
    }

    // TODO: reveal this documentation when support for service_unadvertise is actually done
    // Advertises a service and returns a handle to the service server
    // Service will be active until the handle is dropped!
    pub async fn advertise_service<T: RosServiceType>(
        &mut self,
        topic: &str,
        server: fn(
            T::Request,
        )
            -> Result<T::Response, Box<dyn std::error::Error + 'static + Send + Sync>>,
    ) -> RosLibRustResult<ServiceHandle> {
        self.check_for_disconnect()?;
        {
            let client = self.inner.read().await;
            let mut writer = client.writer.write().await;
            writer.advertise_service(topic, T::ROS_SERVICE_NAME).await?;

            // We need to do type erasure and hide the request by wrapping their closure in a generic closure
            let erased_closure = move |message: &str| -> Result<
                serde_json::Value,
                Box<dyn std::error::Error + Send + Sync>,
            > {
                // Type erase the incoming type
                let parsed_msg = serde_json::from_str(message)?;
                let response = server(parsed_msg)?;
                // Type erase the outgoing type
                let response_string = serde_json::json!(response);
                Ok(response_string)
            };

            let res = client
                .services
                .insert(topic.to_string(), Box::new(erased_closure));
            if let Some(_previous_server) = res {
                warn!("Re-registering a server for a pre-existing topic? Are you sure you want to do this");
                unimplemented!()
            }
        } // Drop client lock here so we can clone without creating an issue

        Ok(ServiceHandle {
            client: self.clone(),
            topic: topic.to_string(),
        })
    }

    /// Internal method for removing a service, this is expected to be automatically called
    /// by dropping the relevant service handle. Intentionally not async as a result.
    fn unadvertise_service(&self, topic: &str) {
        let copy = self.inner.clone();
        let topic = topic.to_string();
        tokio::spawn(async move {
            let client = copy.read().await;
            let entry = client.services.remove(&topic);
            // Since this is called by drop we can't really propagate and error and instead simply have to log
            if entry.is_none() {
                error!(
                    "Unadvertise service was called on topic `{topic}` however no service was found.\
                This likely indicates and error with the roslibrust crate."
                );
            }

            // Regardless of whether we found an entry we should still send he unadvertise_service message to rosbridge
            let mut writer = client.writer.write().await;
            let res = writer.unadvertise_service(&topic).await;
            if let Err(e) = res {
                error!("Failed to send unadvertise_service message when service handle was dropped for `{topic}`: {e}");
            }
        });
    }

    // This function is not async specifically so it can be called from drop
    // same reason why it doesn't return anything
    // Called automatically when Publisher is dropped
    fn unadvertise(&self, topic_name: &str) {
        let copy = self.clone();
        let topic_name_copy = topic_name.to_string();
        tokio::spawn(async move {
            // Remove publisher from our records
            let client = copy.inner.read().await;
            client.publishers.remove(&topic_name_copy);

            // Send unadvertise message
            {
                debug!("Unadvertise waiting for comm lock");
                let mut comm = client.writer.write().await;
                debug!("Unadvertise got comm lock");
                if let Err(e) = comm.unadvertise(&topic_name_copy).await {
                    error!("Failed to send unadvertise in comm layer: {:?}", e);
                }
            }
        });
    }

    /// This function removes the entry for a subscriber in from the client, and if it is the last
    /// subscriber for a given topic then dispatches an unsubscribe message to the master/bridge
    fn unsubscribe(&self, topic_name: &str, id: &uuid::Uuid) -> RosLibRustResult<()> {
        // Copy so we can move into closure
        let client = self.clone();
        let topic_name = topic_name.to_string();
        let id = id.clone();
        // Actually send the unsubscribe message in a task so subscriber::Drop can call this function
        tokio::spawn(async move {
            // Identify the subscription entry for the subscriber
            let client = client.inner.read().await;
            let mut subscription = match client.subscriptions.get_mut(&topic_name) {
                Some(subscription) => subscription,
                None => {
                    error!("Topic not found in subscriptions upon dropping. This should be impossible and indicates a bug in the roslibrust crate. Topic: {topic_name} UUID: {id:?}");
                    return;
                }
            };
            if subscription.value_mut().handles.remove(&id).is_none() {
                error!("Subscriber id {id:?} was not found in handles list for topic {topic_name:?} while unsubscribing");
                return;
            }

            if subscription.handles.is_empty() {
                // This is the last subscriber for that topic and we need to unsubscribe now
                let mut stream = client.writer.write().await;
                match stream.unsubscribe(&topic_name).await {
                    Ok(_) => {}
                    Err(e) => error!(
                        "Failed to send unsubscribe while dropping subscriber: {:?}",
                        e
                    ),
                }
            }
        });
        Ok(())
    }
}

/// A client connection to the rosbridge_server that allows for publishing and subscribing to topics
struct Client {
    // TODO replace Socket with trait RosBridgeComm to allow mocking
    reader: RwLock<Reader>,
    writer: RwLock<Writer>,
    // Stores a record of the publishers we've handed out
    publishers: DashMap<String, PublisherHandle>,
    subscriptions: DashMap<String, Subscription>,
    services: DashMap<String, ServiceCallback>,
    // Contains any outstanding service calls we're waiting for a response on
    // Map key will be a uniquely generated id for each call
    service_calls: DashMap<String, tokio::sync::oneshot::Sender<Value>>,
    opts: ClientHandleOptions,
}

impl Client {
    // internal implementation of new
    async fn new(opts: ClientHandleOptions) -> RosLibRustResult<Self> {
        let (writer, reader) = stubborn_connect(&opts.url).await;
        let client = Self {
            reader: RwLock::new(reader),
            writer: RwLock::new(writer),
            publishers: DashMap::new(),
            services: DashMap::new(),
            subscriptions: DashMap::new(),
            service_calls: DashMap::new(),
            opts,
        };

        Ok(client)
    }

    async fn handle_message(&self, msg: Message) -> RosLibRustResult<()> {
        match msg {
            Message::Text(text) => {
                debug!("got message: {}", text);
                // TODO better error handling here serde_json::Error not send
                let parsed: serde_json::Value = serde_json::from_str(text.as_str()).unwrap();
                let parsed_object = parsed
                    .as_object()
                    .expect("Recieved non-object json response");
                let op = parsed_object
                    .get("op")
                    .expect("Op field not present on returned object.")
                    .as_str()
                    .expect("Op field was not of string type.");
                let op = comm::Ops::from_str(op)?;
                match op {
                    comm::Ops::Publish => {
                        trace!("handling publish for {:?}", &parsed);
                        self.handle_publish(parsed).await;
                    }
                    comm::Ops::ServiceResponse => {
                        trace!("handling service response for {:?}", &parsed);
                        self.handle_response(parsed).await;
                    }
                    comm::Ops::CallService => {
                        trace!("handling call_service for {:?}", &parsed);
                        self.handle_service(parsed).await;
                    }
                    _ => {
                        warn!("Unhandled op type {}", op)
                    }
                }
            }
            Message::Close(close) => {
                // TODO how should we respond to this?
                // How do we represent connection status via our API well?
                panic!("Close requested from server: {:?}", close);
            }
            Message::Ping(ping) => {
                debug!("Ping received: {:?}", ping);
            }
            Message::Pong(pong) => {
                debug!("Pong received {:?}", pong);
            }
            _ => {
                panic!("Non-text response received");
            }
        }

        Ok(())
    }

    async fn handle_response(&self, data: Value) {
        // TODO lots of error handling!
        let id = data.get("id").unwrap().as_str().unwrap();
        let (_id, call) = self.service_calls.remove(id).unwrap();
        let res = data.get("values").unwrap();
        call.send(res.clone()).unwrap();
    }

    /// Response handler for receiving a service call looks up if we have a service
    /// registered for the incoming topic and if so dispatches to the callback
    async fn handle_service(&self, data: Value) {
        // Unwrap is okay, field is fully required and strictly type
        let topic = data.get("service").unwrap().as_str().unwrap();
        // Unwrap is okay, field is strictly typed to string
        let id = data.get("id").map(|id| id.as_str().unwrap().to_string());

        // Lookup if we have a service for the message
        let callback = self.services.get(topic);
        let callback = match callback {
            Some(callback) => callback,
            _ => panic!("Received call_service for unadvertised service!"),
        };
        // TODO likely bugs here remove this unwrap. Unclear what we are expected to get for empty service
        let request = data.get("args").unwrap().to_string();
        let mut writer = self.writer.write().await;
        match callback(&request) {
            Ok(res) => {
                // TODO unwrap here is probably bad... Failure to write means disconnected?
                writer.service_response(topic, id, true, res).await.unwrap();
            }
            Err(e) => {
                error!("A service callback on topic {:?} failed with {:?} sending response false in service_response", data.get("service"), e);
                writer
                    .service_response(topic, id, false, serde_json::json!(format!("{e}")))
                    .await
                    .unwrap();
            }
        };

        // Now we need to send the service_response back
    }

    async fn spin_once(&self) -> RosLibRustResult<()> {
        let read = {
            let mut stream = self.reader.write().await;
            match stream.next().await {
                Some(Ok(msg)) => msg,
                Some(Err(e)) => {
                    return Err(e.into());
                }
                None => {
                    return Err(RosLibRustError::Unexpected(anyhow!(
                        "Wtf does none mean here?"
                    )));
                }
            }
        };
        debug!("Got message: {:?}", read);
        self.handle_message(read).await
    }

    /// Response handler for received publish messages
    /// Converts the return message to the subscribed type and calls any callbacks
    /// Panics if publish is received for unexpected topic
    async fn handle_publish(&self, data: Value) {
        // TODO lots of error handling!
        let callbacks = self
            .subscriptions
            .get(data.get("topic").unwrap().as_str().unwrap());
        let callbacks = match callbacks {
            Some(callbacks) => callbacks,
            _ => panic!("Received publish message for unsubscribed topic!"), // TODO probably shouldn't be a panic?
        };
        for (_id, callback) in &callbacks.handles {
            callback(
                // TODO possible bug here if "msg" isn't defined remove this unwrap
                serde_json::to_string(data.get("msg").unwrap())
                    .unwrap()
                    .as_str(),
            )
        }
    }

    async fn reconnect(&mut self) -> RosLibRustResult<()> {
        // Reconnect stream
        let (writer, reader) = stubborn_connect(&self.opts.url).await;
        self.reader = RwLock::new(reader);
        self.writer = RwLock::new(writer);

        // TODO re-advertise!
        // Resend rosbridge our subscription requests to re-establish inflight subscriptions
        // Clone here is dumb, but required due to async
        let mut subs: Vec<(String, String)> = vec![];
        {
            for sub in self.subscriptions.iter() {
                subs.push((sub.key().clone(), sub.value().topic_type.clone()))
            }
        }
        let mut stream = self.writer.write().await;
        for (topic, topic_type) in &subs {
            stream.subscribe(topic, topic_type).await?;
        }

        Ok(())
    }
}

/// Wraps spin in retry logic to handle reconnections automagically
async fn stubborn_spin(
    client: std::sync::Weak<RwLock<Client>>,
    is_disconnected: Arc<AtomicBool>,
) -> RosLibRustResult<()> {
    debug!("Starting stubborn_spin");
    while let Some(client) = client.upgrade() {
        const SPIN_DURATION: Duration = Duration::from_millis(10);

        match tokio::time::timeout(SPIN_DURATION, client.read().await.spin_once()).await {
            Ok(Ok(())) => {}
            Ok(Err(err)) => {
                is_disconnected.store(true, Ordering::Relaxed);
                warn!("Spin failed with error: {err}, attempting to reconnect");
                client.write().await.reconnect().await?;
                is_disconnected.store(false, Ordering::Relaxed);
            }
            Err(_) => {
                // Time out occurred, so we'll check on our weak pointer again
            }
        }
    }

    Ok(())
}

/// Implementation of timeout that is a no-op if timeout is 0 or unconfigured
/// Only works on functions that already return our result type
// This might not be needed but reading tokio::timeout docs I couldn't confirm this
async fn timeout<F, T>(timeout: Option<Duration>, future: F) -> RosLibRustResult<T>
where
    F: futures::Future<Output = RosLibRustResult<T>>,
{
    if let Some(t) = timeout {
        tokio::time::timeout(t, future).await?
    } else {
        future.await
    }
}

/// Connects to websocket at specified URL, retries indefinitely
async fn stubborn_connect(url: &str) -> (Writer, Reader) {
    loop {
        match connect(&url).await {
            Err(e) => {
                warn!("Failed to reconnect: {:?}", e);
                // TODO configurable rate?
                tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
                continue;
            }
            Ok(stream) => {
                let (writer, reader) = stream.split();
                return (writer, reader);
            }
        }
    }
}

/// Basic connection attempt and error wrapping
async fn connect(url: &str) -> RosLibRustResult<Socket> {
    let attempt = tokio_tungstenite::connect_async(url).await;
    match attempt {
        Ok((stream, _response)) => Ok(stream),
        Err(e) => Err(e.into()),
    }
}
