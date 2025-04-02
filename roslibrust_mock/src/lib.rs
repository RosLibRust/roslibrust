//! A mock implementation of roslibrust's generic traits useful for testing ROS behaviors.
//!
//! It is not recommended to depend on this crate directly, but instead access it via roslibrust with the `mock` feature enabled.
//!
//! ```
//! // Normally accessed as roslibrust::{Result, TopicProvider, Publish}
//! use roslibrust_common::{Result, TopicProvider, Publish};
//! // Normally you'd use generated types from roslibrust::codegen
//! use roslibrust_test::ros1::*;
//!
//! async fn my_ros_thing(ros: impl TopicProvider) -> Result<()> {
//!     let my_publisher = ros.advertise::<std_msgs::String>("my_topic").await?;
//!     my_publisher.publish(&std_msgs::String { data: "Hello, world!".to_string() }).await?;
//!     Ok(())
//! }
//!
//! #[tokio::test]
//! async fn test_my_ros_thing() {
//!     // Create a mock ros instance with new
//!     let ros = roslibrust::mock::MockRos::new();
//!     // Use it like ros:
//!     let test_sub = ros.subscribe::<std_msgs::String>("my_topic").await?;
//!     // Kick off our object under test
//!     tokio::spawn(my_ros_thing(ros));
//!     // Assert we got the message we expected
//!     assert_eq!(test_sub.next().await.unwrap().unwrap().data, "Hello, world!");
//! }
//! ```
use std::collections::BTreeMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

use roslibrust_common::*;

use tokio::sync::broadcast as Channel;
use tokio::sync::RwLock;

use log::*;

type TypeErasedCallback = Arc<
    dyn Fn(Vec<u8>) -> std::result::Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
        + Send
        + Sync,
>;

// This is the type that will be returned from an async service function after we have type erased it
type TypeErasedServiceFuture = Pin<
    Box<
        dyn Future<Output = std::result::Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>>
            + Send,
    >,
>;

// This is the type that we will store to store an async service function
type TypeErasedAsyncCallback = Arc<dyn Fn(Vec<u8>) -> TypeErasedServiceFuture + Send + Sync>;
/// A mock ROS implementation that can be substituted for any roslibrust backend in unit tests.
///
/// Implements [TopicProvider] and [ServiceProvider] to provide basic ros functionality.
#[derive(Clone)]
pub struct MockRos {
    // We could probably achieve some fancier type erasure than actually serializing the data
    // but this ends up being pretty simple
    topics: Arc<RwLock<BTreeMap<String, (Channel::Sender<Vec<u8>>, Channel::Receiver<Vec<u8>>)>>>,
    services: Arc<RwLock<BTreeMap<String, TypeErasedCallback>>>,
    async_services: Arc<RwLock<BTreeMap<String, TypeErasedAsyncCallback>>>,
}

impl MockRos {
    pub fn new() -> Self {
        Self {
            topics: Arc::new(RwLock::new(BTreeMap::new())),
            services: Arc::new(RwLock::new(BTreeMap::new())),
            async_services: Arc::new(RwLock::new(BTreeMap::new())),
        }
    }
}

// This is a very basic mocking of sending and receiving messages over topics
// It does not implement automatic shutdown of topics on dropping
impl TopicProvider for MockRos {
    type Publisher<T: RosMessageType> = MockPublisher<T>;
    type Subscriber<T: RosMessageType> = MockSubscriber<T>;

    async fn advertise<T: RosMessageType>(&self, topic: &str) -> Result<Self::Publisher<T>> {
        // Check if we already have this channel
        {
            let topics = self.topics.read().await;
            if let Some((sender, _)) = topics.get(topic) {
                debug!("Issued new publisher to existing topic {}", topic);
                return Ok(MockPublisher {
                    sender: sender.clone(),
                    _marker: Default::default(),
                });
            }
        } // Drop read lock here
          // Create a new channel
        let tx_rx = Channel::channel(10);
        let tx_copy = tx_rx.0.clone();
        let mut topics = self.topics.write().await;
        topics.insert(topic.to_string(), tx_rx);
        debug!("Created new publisher and channel for topic {}", topic);
        Ok(MockPublisher {
            sender: tx_copy,
            _marker: Default::default(),
        })
    }

    async fn subscribe<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> roslibrust_common::Result<Self::Subscriber<T>> {
        // Check if we already have this channel
        {
            let topics = self.topics.read().await;
            if let Some((_, receiver)) = topics.get(topic) {
                debug!("Issued new subscriber to existing topic {}", topic);
                return Ok(MockSubscriber {
                    receiver: receiver.resubscribe(),
                    _marker: Default::default(),
                });
            }
        } // Drop read lock here
          // Create a new channel
        let tx_rx = Channel::channel(10);
        let rx_copy = tx_rx.1.resubscribe();
        let mut topics = self.topics.write().await;
        topics.insert(topic.to_string(), tx_rx);
        debug!("Created new subscriber and channel for topic {}", topic);
        Ok(MockSubscriber {
            receiver: rx_copy,
            _marker: Default::default(),
        })
    }
}

enum Callback {
    Sync(TypeErasedCallback),
    Async(TypeErasedAsyncCallback),
}

/// The handle type returned by calling [MockRos::service_client].
/// Represents a ROS service connection and allows the service to be called multiple times.
pub struct MockServiceClient<T: RosServiceType> {
    callback: Callback,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosServiceType> Service<T> for MockServiceClient<T> {
    async fn call(&self, request: &T::Request) -> roslibrust_common::Result<T::Response> {
        let data =
            bincode::serialize(request).map_err(|e| Error::SerializationError(e.to_string()))?;

        let response = match &self.callback {
            Callback::Sync(callback) => (callback)(data),
            Callback::Async(callback) => (callback)(data).await,
        }
        .map_err(|e| Error::ServerError(e.to_string()))?;

        let response = bincode::deserialize(&response[..])
            .map_err(|e| Error::SerializationError(e.to_string()))?;
        Ok(response)
    }
}

impl ServiceProvider for MockRos {
    type ServiceClient<T: RosServiceType> = MockServiceClient<T>;
    type ServiceServer = ();

    async fn call_service<T: RosServiceType>(
        &self,
        topic: &str,
        request: T::Request,
    ) -> roslibrust_common::Result<T::Response> {
        let client = self.service_client::<T>(topic).await?;
        client.call(&request).await
    }

    async fn service_client<T: RosServiceType + 'static>(
        &self,
        topic: &str,
    ) -> roslibrust_common::Result<Self::ServiceClient<T>> {
        // Look for service in our list of services
        let services = self.services.read().await;
        if let Some(callback) = services.get(topic) {
            return Ok(MockServiceClient {
                callback: Callback::Sync(callback.clone()),
                _marker: Default::default(),
            });
        }

        // Look for service in our list of async services
        let async_services = self.async_services.read().await;
        if let Some(callback) = async_services.get(topic) {
            return Ok(MockServiceClient {
                callback: Callback::Async(callback.clone()),
                _marker: Default::default(),
            });
        }

        Err(Error::Disconnected)
    }

    async fn advertise_service<T: RosServiceType + 'static, F>(
        &self,
        topic: &str,
        server: F,
    ) -> roslibrust_common::Result<Self::ServiceServer>
    where
        F: ServiceFn<T>,
    {
        // Type erase the service function here
        let erased_closure = move |message: Vec<u8>| -> std::result::Result<
            Vec<u8>,
            Box<dyn std::error::Error + Send + Sync>,
        > {
            let request = bincode::deserialize(&message[..])
                .map_err(|e| Error::SerializationError(e.to_string()))?;
            let response = server(request)?;
            let bytes = bincode::serialize(&response)
                .map_err(|e| Error::SerializationError(e.to_string()))?;
            Ok(bytes)
        };
        let erased_closure = Arc::new(erased_closure);
        let mut services = self.services.write().await;
        services.insert(topic.to_string(), erased_closure);

        // We technically need to hand back a token that shuts the service down here
        // But we haven't implemented that yet in this mock
        Ok(())
    }

    async fn advertise_async_service<T: RosServiceType + 'static, F, Fut>(
        &self,
        topic: &str,
        server: F,
    ) -> roslibrust_common::Result<Self::ServiceServer>
    where
        F: Fn(T::Request) -> Fut + Send + Sync + 'static,
        Fut: Future<
                Output = std::result::Result<T::Response, Box<dyn std::error::Error + Send + Sync>>,
            > + Send
            + 'static,
    {
        // Place the server into an Arc so we can clone it
        let a_server = Arc::new(server);
        // Wrap the async closure in a Box and convert its future to a pinned box
        let wrapped_closure = Arc::new(move |message: Vec<u8>| -> TypeErasedServiceFuture {
            // Here we have to move a copy of the server into the future
            let server = a_server.clone();
            // Return a future a pinned-box future that will evaluate the user's async closure
            // This is now storable in a data structure
            Box::pin(async move {
                let request = bincode::deserialize(&message[..])
                    .map_err(|e| Error::SerializationError(e.to_string()))?;
                let response = (server)(request).await?;
                let bytes = bincode::serialize(&response)
                    .map_err(|e| Error::SerializationError(e.to_string()))?;
                Ok(bytes)
            })
        });

        // Store the type erased closure
        let mut async_services = self.async_services.write().await;
        async_services.insert(topic.to_string(), wrapped_closure);

        Ok(())
    }
}

/// The publisher type returned by calling [MockRos::advertise].
pub struct MockPublisher<T: RosMessageType> {
    sender: Channel::Sender<Vec<u8>>,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> Publish<T> for MockPublisher<T> {
    async fn publish(&self, data: &T) -> roslibrust_common::Result<()> {
        let data =
            bincode::serialize(data).map_err(|e| Error::SerializationError(e.to_string()))?;
        self.sender.send(data).map_err(|_e| Error::Disconnected)?;
        debug!("Sent data on topic {}", T::ROS_TYPE_NAME);
        Ok(())
    }
}

/// The subscriber type returned by calling [MockRos::subscribe].
pub struct MockSubscriber<T: RosMessageType> {
    receiver: Channel::Receiver<Vec<u8>>,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> Subscribe<T> for MockSubscriber<T> {
    async fn next(&mut self) -> roslibrust_common::Result<T> {
        let data = self
            .receiver
            .recv()
            .await
            .map_err(|_| Error::Disconnected)?;
        let msg = bincode::deserialize(&data[..])
            .map_err(|e| Error::SerializationError(e.to_string()))?;
        debug!("Received data on topic {}", T::ROS_TYPE_NAME);
        Ok(msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roslibrust_test::ros1::std_msgs;
    use roslibrust_test::ros1::std_srvs;

    #[tokio::test(flavor = "multi_thread")]
    async fn test_mock_topics() {
        let mock_ros = MockRos::new();

        let pub_handle = mock_ros
            .advertise::<std_msgs::String>("test_topic")
            .await
            .unwrap();
        let mut sub_handle = mock_ros
            .subscribe::<std_msgs::String>("test_topic")
            .await
            .unwrap();

        let msg = std_msgs::String {
            data: "Hello, world!".to_string(),
        };

        pub_handle.publish(&msg).await.unwrap();

        let received_msg = sub_handle.next().await.unwrap();

        assert_eq!(msg, received_msg);
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_mock_services() {
        let mock_topics = MockRos::new();

        let server_fn = |request: std_srvs::SetBoolRequest| {
            Ok(std_srvs::SetBoolResponse {
                success: request.data,
                message: "You set my bool!".to_string(),
            })
        };

        mock_topics
            .advertise_service::<std_srvs::SetBool, _>("test_service", server_fn)
            .await
            .unwrap();

        let client = mock_topics
            .service_client::<std_srvs::SetBool>("test_service")
            .await
            .unwrap();

        let request = std_srvs::SetBoolRequest { data: true };

        let response = client.call(&request).await.unwrap();
        assert_eq!(response.success, true);
        assert_eq!(response.message, "You set my bool!");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_mock_node() {
        // Proves that MockRos impls the Ros trait (via auto impl in roslibrust_common)
        // and can be used as such
        struct MyNode<T: Ros> {
            ros: T,
        }

        impl<T: Ros> MyNode<T> {
            async fn run(self) {
                let publisher = self
                    .ros
                    .advertise::<std_msgs::String>("/chatter")
                    .await
                    .unwrap();

                publisher
                    .publish(&std_msgs::String {
                        data: "Hello, world!".to_string(),
                    })
                    .await
                    .unwrap();
            }
        }

        let mock_ros = MockRos::new();
        let node = MyNode { ros: mock_ros };
        node.run().await;
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_mock_async_services() {
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let mock_ros = MockRos::new();

        // This is what I want users to be able to write!
        // But it doesn't work!
        // let service = async move |request: std_srvs::SetBoolRequest| -> std::result::Result<
        //     std_srvs::SetBoolResponse,
        //     Box<dyn std::error::Error + Send + Sync>,
        // > {
        //     // Requires that tx is Copy...
        //     tx.send(request.data).await.unwrap();
        //     Ok(std_srvs::SetBoolResponse {
        //         success: true,
        //         message: "You set my bool!".to_string(),
        //     })
        // };

        let service = move |request: std_srvs::SetBoolRequest| {
            let tx = tx.clone();
            async move {
                tx.send(request.data).await.unwrap();
                Ok(std_srvs::SetBoolResponse {
                    success: true,
                    message: "You set my bool!".to_string(),
                })
            }
        };

        let _handle = mock_ros
            .advertise_async_service::<std_srvs::SetBool, _, _>("test_service", service)
            .await
            .unwrap();

        let client = mock_ros
            .service_client::<std_srvs::SetBool>("test_service")
            .await
            .unwrap();

        client
            .call(&std_srvs::SetBoolRequest { data: true })
            .await
            .unwrap();

        let received = rx.recv().await.unwrap();
        assert_eq!(received, true);
    }
}
