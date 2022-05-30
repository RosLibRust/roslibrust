// Contains tools for generating message definitions from ROS1 .msg and .srv files
pub mod message_gen;
// Utilities functions primarily for working with ros env vars and package structures
pub mod util;
// Communication primitives for the rosbridge_suite protocol
pub mod comm;
// Contains generated messages created by message_gen from the test_msgs directory
// TODO look into restricting visibility of this... #[cfg(test)] and pub(crate) not working as needed
pub mod test_msgs;

use comm::RosBridgeComm;
use futures::StreamExt;
use log::*;
use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde::de::DeserializeOwned;
use serde::Serialize;
use serde_json::Value;
use simple_error::bail;
use std::collections::HashMap;
use std::error::Error;
use std::fmt::Debug;
use std::sync::Arc;
use tokio::net::TcpStream;
use tokio::sync::RwLock;
use tokio::time::Duration;
use tokio_tungstenite::*;
use tungstenite::Message;

/// Used for type erasure of message type so that we can store arbitrary handles
type Callback = Box<dyn Fn(&str) -> () + Send + Sync>;
type Stream = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

// TODO someday make this more specific / easier to handle
type RosBridgeError = Box<dyn Error>;
type RosBridgeResult<T> = Result<T, RosBridgeError>;

struct Subscription {
    pub handles: Vec<Callback>,
    /// Name of ros type (package_name/message_name), used for re-subscribes
    pub topic_type: String,
}

/// Fundamental traits for message types this works with
/// This trait will be satisfied for any types generate with this crate's message_gen
pub trait RosMessageType:
    'static + DeserializeOwned + Default + Send + Serialize + Sync + Clone + Debug
{
    const ROS_TYPE_NAME: &'static str;
}

// TODO: Potential options to add
//  * stubborn reconnect interval
//  * Automatic header Seq / Stamp setting
#[derive(Clone)]
pub struct ClientOptions {
    url: String,
    timeout: Option<Duration>,
}

impl ClientOptions {
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// URL is required to be set
    pub fn new<S: Into<String>>(url: S) -> ClientOptions {
        ClientOptions {
            url: url.into(),
            timeout: None,
        }
    }

    /// Configures a default timeout for all operations
    pub fn timeout<T: Into<Duration>>(mut self, duration: T) -> ClientOptions {
        self.timeout = Some(duration.into());
        self
    }
}

// Fundamental structure for a client
#[derive(Clone)]
pub struct Client {
    // TODO split stream into read and write halves
    // TODO replace Stream with trait RosBridgeComm to allow mocking
    comm: Arc<RwLock<Stream>>,
    subscriptions: Arc<RwLock<HashMap<String, Subscription>>>,
    // Contains any outstanding service calls we're waiting for a response on
    // Map key will be a uniquely generated id for each call
    service_calls: Arc<RwLock<HashMap<String, tokio::sync::oneshot::Sender<Value>>>>,
    opts: Arc<ClientOptions>,
}

impl Client {
    // TODO apparently rosbridge doesn't support this!
    // TODO get them to! and then implement!
    // /// Advertises a service and returns a handle to the service server
    // /// Serivce will be active until the handle is dropped!
    // pub async fn advertise_service(
    //     &mut self,
    //     topic: &str,
    // ) -> Result<ServiceServer, Box<dyn Error>> {
    //     unimplemented!()
    // }

    async fn call_service<Req: RosMessageType, Res: RosMessageType>(
        &mut self,
        service: &str,
        req: Req,
    ) -> RosBridgeResult<Res> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        let rand_string: String = thread_rng()
            .sample_iter(&Alphanumeric)
            .take(30)
            .map(char::from)
            .collect();
        {
            let mut service_calls = self.service_calls.write().await;
            if service_calls.insert(rand_string.clone(), tx).is_some() {
                error!("ID collision encounted in call_service");
            }
        }
        {
            let mut comm = self.comm.write().await;
            comm.call_service(service, &rand_string, req).await?;
        }
        let msg = rx.await?;
        Ok(serde_json::from_value(msg)?)
    }

    /// Implementation of timeout that is a no-op if timeout is 0 or unconfigured
    /// Only works on functions that already return our result type
    // This might not be needed but reading tokio::timeout docs I couldn't confirm this
    async fn timeout<F, T>(timeout: Option<Duration>, future: F) -> Result<T, Box<dyn Error>>
    where
        F: futures::Future<Output = Result<T, Box<dyn Error>>>,
    {
        if let Some(t) = timeout {
            tokio::time::timeout(t, future).await?
        } else {
            future.await
        }
    }

    // internal implementation of new
    async fn _new(opts: ClientOptions) -> Result<Client, Box<dyn Error>> {
        let client = Client {
            comm: Arc::new(RwLock::new(
                Client::stubborn_connect(opts.url.clone()).await,
            )),
            subscriptions: Arc::new(RwLock::new(HashMap::new())),
            service_calls: Arc::new(RwLock::new(HashMap::new())),
            opts: Arc::new(opts),
        };

        // Spawn the spin task
        let client_copy = client.clone();
        // Okay we can't do this yet, because somehow doing this breaks Send
        // I don't understand exactly why quite yet, but it does
        let _jh = tokio::task::spawn(client_copy.stubborn_spin());

        Ok(client)
    }

    pub async fn new_with_options(opts: ClientOptions) -> Result<Client, Box<dyn Error>> {
        // TODO clone here is dumb but was most ergonomic option I found
        Client::timeout(opts.timeout, Client::_new(opts)).await
    }

    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// When awaited will not resolve until connection is completed
    // TODO better error handling
    pub async fn new<S: Into<String>>(url: S) -> Result<Client, Box<dyn Error>> {
        Client::new_with_options(ClientOptions::new(url)).await
    }

    // Internal implementation of subscribe
    async fn _subscribe<Msg>(
        &mut self,
        topic_name: &str,
    ) -> Result<tokio::sync::watch::Receiver<Msg>, Box<dyn std::error::Error>>
    where
        Msg: RosMessageType,
    {
        let mut lock = self.subscriptions.write().await;
        debug!("Subscribe got write lock");
        let cbs = lock.entry(topic_name.to_string()).or_insert(Subscription {
            handles: vec![],
            topic_type: Msg::ROS_TYPE_NAME.to_string(),
        });

        // TODO single place to define rosbridge operations
        // Send subscribe message to rosbridge to initiate it sending us messages
        Client::send_subscribe_request(
            &mut self.comm,
            &topic_name.to_string(),
            &Msg::ROS_TYPE_NAME.to_string(),
        )
        .await?;

        // TODO we have spsc, so watch is kinda overkill? Buffer of 1 is nice...
        // Note: Type erasure with callback is forcing generating a new channel per subscription
        // instead of allowing us to re-use the subscription.
        // I can't figure out how to store rx's with different types...
        // Trait objects can't
        // Create a new watch channel for this topic
        let (tx, rx) = tokio::sync::watch::channel(Msg::default());

        // Move the tx into a callback that takes raw string data
        // This allows us to store the callbacks generic on type, Msg conversion is embedded here
        let send_cb = Box::new(move |data: &str| {
            let converted = serde_json::from_str(data);
            // TODO makes sense for callback to return Result<>, instead of this handling
            if converted.is_err() {
                error!(
                    "Failed to deserialize ros message: {:?}. Message will be skipped!",
                    converted.err().unwrap()
                );
                return;
            }
            let converted = converted.unwrap();

            tx.send(converted).unwrap();
        });

        // Store callback
        cbs.handles.push(send_cb);

        // Return a subscription handle
        Ok(rx)
    }

    /// Subscribe to a given topic expecting msgs of provided type
    pub async fn subscribe<Msg>(
        &mut self,
        topic_name: &str,
    ) -> Result<tokio::sync::watch::Receiver<Msg>, Box<dyn std::error::Error>>
    where
        Msg: RosMessageType,
    {
        Client::timeout(self.opts.timeout, self._subscribe(topic_name)).await
    }

    pub async fn unsubscribe(&mut self, topic_name: &str) -> RosBridgeResult<()> {
        self.subscriptions.write().await.remove(topic_name);
        let mut stream = self.comm.write().await;
        stream.unsubscribe(topic_name).await?;
        Ok(())
    }

    /// Core read loop, receives messages from rosbridge and dispatches them.
    // Creating a client spawns a task which calls stubborn spin, which in turn calls this funtion
    async fn spin(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Start spin");
        loop {
            let read = {
                let mut stream = self.comm.write().await;
                // Okay this is what I want to do, but I can't because this actually block publishes
                // We have to instead settle for some periodic bullshit.
                // TODO figure out how to properly duplex the websocket...
                // We need to split the channel into seperate halves with indpendently blocking read / write halves
                // stream.next().await
                match tokio::time::timeout(tokio::time::Duration::from_millis(1), stream.next())
                    .await
                {
                    Ok(read) => read,
                    Err(_) => {
                        // Timed out, release mutex so other tasks can use stream
                        continue;
                    }
                }
            };
            let read = match read {
                Some(r) => r?,
                None => {
                    // TODO can we do better than using SimpleError? Define our own?
                    bail!("Wtf does none mean here?");
                }
            };
            debug!("Got message: {:?}", read);
            match read {
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
                    match op {
                        "publish" => {
                            self.handle_publish(parsed).await;
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
        }
    }

    /// Wraps spin in retry logic to handle reconnections automagically
    async fn stubborn_spin(mut self) -> Result<(), Box<dyn Error + Send>> {
        debug!("Starting stubborn_spin");
        loop {
            {
                let res = self.spin().await;
                if res.is_err() {
                    warn!(
                        "Spin failed with error: {}, attempting to reconnect",
                        res.err().unwrap()
                    );
                } else {
                    panic!("Spin should not exit cleanly?");
                }
            }
            // Reconnect stream
            self.comm = Arc::new(RwLock::new(
                Client::stubborn_connect(self.opts.url.to_string()).await,
            ));

            // Resend rosbridge our subscription requests to re-establish inflight subscriptions
            // Clone here is dumb, but required due to async
            let mut subs: Vec<(String, String)> = vec![];
            {
                let lock = self.subscriptions.read();
                let lock = lock.await;
                for (k, v) in lock.iter() {
                    subs.push((k.clone(), v.topic_type.clone()))
                }
            }
            for (topic, topic_type) in &subs {
                // TODO bubble error up
                Client::send_subscribe_request(&mut self.comm, topic, topic_type)
                    .await
                    .unwrap();
            }
        }
    }

    /// Response handler for received publish messages
    /// Converts the return message to the subscribed type and calls any callbacks
    /// Panics if publish is received for unexpected topic
    async fn handle_publish(&mut self, data: Value) {
        // TODO lots of error handling!
        let lock = self.subscriptions.read().await;
        let callbacks = lock.get(data.get("topic").unwrap().as_str().unwrap());
        let callbacks = match callbacks {
            Some(callbacks) => callbacks,
            _ => panic!("Received publish message for unsubscribed topic!"),
        };
        for callback in &callbacks.handles {
            callback(
                serde_json::to_string(data.get("msg").unwrap())
                    .unwrap()
                    .as_str(),
            )
        }
    }

    /// Connects to websocket at specified URL, retries indefinitely
    async fn stubborn_connect(url: String) -> Stream {
        loop {
            // TODO had a failure on my system:
            /*
            thread 'main' panicked at 'Could not determine the UTC offset on this system.
            Possible causes are that the time crate does not implement "local_offset_at" on your system, or that you are running in a multi-threaded environment and the time crate is returning "None" from "local_offset_at" to avoid unsafe behaviour.
            See the time crate's documentation for more information.
             */
            // Need to figure out how to not panic if this occurs
            // Seems potentially to be coming from SimpleLogger?
            // 100% is coming from SimpleLogger used in examples wtf...
            // Only origin from this line is the fact that this next line logs internally
            let connection_attempt = tokio_tungstenite::connect_async(&url).await;
            if connection_attempt.is_err() {
                warn!(
                    "Failed to reconnect: {:?}",
                    connection_attempt.err().unwrap()
                );
                // TODO configurable rate?
                tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
                continue;
            }
            let (stream, _) = connection_attempt.unwrap();
            return stream;
        }
    }

    /// Sends a subscribe request packet to rosbridge
    // Note: doesn't borrow self only stream so we can iterate subscriptions and call simultaneously
    async fn send_subscribe_request(
        stream: &Arc<RwLock<Stream>>,
        topic: &String,
        msg_type: &String,
    ) -> Result<(), Box<dyn Error>> {
        let mut stream = stream.write().await;
        stream.subscribe(topic, msg_type).await
    }

    /// Publishes a message
    // TODO publish shouldn't be a function directly on client, instead client needs to return a Publisher, which mananges lifetime of advertise / unadvertise
    pub async fn publish<T>(
        &mut self,
        topic: &str,
        msg: T,
    ) -> Result<(), Box<dyn std::error::Error>>
    where
        T: RosMessageType,
    {
        let mut stream = self.comm.write().await;
        debug!("Publish got write lock on comm");
        stream.publish(topic, msg).await?;
        Ok(())
    }

    pub async fn advertise<T>(&mut self, topic: &str) -> Result<(), Box<dyn std::error::Error>>
    where
        T: RosMessageType,
    {
        let mut stream = self.comm.write().await;
        stream.advertise::<T>(topic).await?;
        Ok(())
    }
}

// TODO all tests in this mod require a running rosbridge_server at localhost:9090
// How to set that up before this module runs?
// How to test against both rosbridge_1 and rosbridge_2 automagically?
#[cfg(test)]
mod general_usage {
    use crate::test_msgs::NodeInfo;
    #[allow(dead_code)]
    pub const LOCAL_WS: &str = "ws://localhost:9090";

    /// Ensures that associate constants are generated on the test_msgs correctly
    /// requires test_msgs gen_code to have been generated.
    /// Compliation is passing for this test
    //TODO may move this code somewhere else
    #[test]
    fn test_associated_contants() {
        let _ = NodeInfo::STATUS_UNINITIALIZED;
    }
}

#[cfg(test)]
#[cfg(feature = "running_bridge")]
mod integration_tests {
    use crate::general_usage::LOCAL_WS;
    use crate::test_msgs::Header;
    use crate::{Client, ClientOptions};
    use tokio::time::timeout;
    // On my laptop test was ~90% reliable at 10ms
    const TIMEOUT: Duration = Duration::from_millis(100);
    use tokio::time::Duration;

    /**
    This test does a round trip publish subscribe for real
    Requires a running local rosbridge
    TODO figure out how to automate setting up the needed environment for this
    */
    #[tokio::test]
    async fn self_publish() {
        // TODO figure out better logging for tests
        simple_logger::SimpleLogger::new()
            .with_level(log::LevelFilter::Debug)
            // TODO had to remove timestamps here to prevent a panic on my laptop with
            // "Could not determine UTC offset on this system"
            // need to investigate futher at some point
            .without_timestamps()
            .init()
            .unwrap();

        const TOPIC: &str = "self_publish";
        // 100ms allowance for connecting so tests still fails
        let mut client = timeout(TIMEOUT, Client::new(LOCAL_WS))
            .await
            .expect("Failed to create client in time")
            .unwrap();

        timeout(TIMEOUT, client.advertise::<Header>(TOPIC))
            .await
            .expect("Failed to advertise in time")
            .unwrap();
        let mut rx = timeout(TIMEOUT, client.subscribe::<Header>(TOPIC))
            .await
            .expect("Failed to subscribe in time")
            .unwrap();

        // Delay here to allow subscribe to complete before publishing
        // Test is flaky without it
        tokio::time::sleep(TIMEOUT).await;

        let msg_out = Header {
            seq: 666,
            stamp: Default::default(),
            frame_id: "self_publish".to_string(),
        };

        timeout(TIMEOUT, client.publish(TOPIC, msg_out.clone()))
            .await
            .expect("Failed to publish in time")
            .unwrap();

        timeout(TIMEOUT, rx.changed())
            .await
            .expect("Failed to receive in time")
            .unwrap();

        let msg_in = rx.borrow().clone();
        assert_eq!(msg_in, msg_out);
    }

    #[tokio::test]
    async fn timeouts_new() {
        // Intentionally a port where there won't be a server at
        let opts = ClientOptions::new("ws://localhost:666").timeout(TIMEOUT);
        assert!(Client::new_with_options(opts).await.is_err());
        // Impossibly short to actually work
        let opts = ClientOptions::new(LOCAL_WS).timeout(Duration::from_nanos(1));
        assert!(Client::new_with_options(opts).await.is_err());
        // Doesn't timeout if given enough time
        let opts = ClientOptions::new(LOCAL_WS).timeout(TIMEOUT);
        assert!(Client::new_with_options(opts).await.is_ok());
    }

    /// This test doesn't actually do much, but instead confirms the internal structure of the lib is multi-threaded correclty
    /// The whole goal here is to catch send / sync complier errors
    #[tokio::test]
    async fn parrallel_construnction() {
        let mut client = Client::new(LOCAL_WS)
            .await
            .expect("Failed to construct client");

        let mut client_1 = client.clone();
        tokio::task::spawn(async move {
            let _ = client_1
                .advertise::<Header>("parrallel_1")
                .await
                .expect("Failed to advertise _1");
        });

        tokio::task::spawn(async move {
            let _ = client
                .subscribe::<Header>("parrallel_1")
                .await
                .expect("Failed to subscribe _1");
        });
    }
}
