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
//!     let my_publisher = ros.advertise::<std_msgs::String>("/my_topic").await?;
//!     my_publisher.publish(&std_msgs::String { data: "Hello, world!".to_string() }).await?;
//!     Ok(())
//! }
//!
//! #[tokio::test]
//! async fn test_my_ros_thing() {
//!     // Create a mock ros instance with new
//!     let ros = roslibrust::mock::MockRos::new();
//!     // Use it like ros:
//!     let test_sub = ros.subscribe::<std_msgs::String>("/my_topic").await?;
//!     // Kick off our object under test
//!     tokio::spawn(my_ros_thing(ros));
//!     // Assert we got the message we expected
//!     assert_eq!(test_sub.next().await.unwrap().unwrap().data, "Hello, world!");
//! }
//! ```
use std::collections::BTreeMap;
use std::sync::Arc;

use roslibrust_common::topic_name::{GlobalTopicName, ToGlobalTopicName};
use roslibrust_common::*;

use tokio::sync::broadcast as Channel;
use tokio::sync::RwLock;

use log::*;

type TypeErasedCallback = Arc<
    dyn Fn(Vec<u8>) -> std::result::Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
        + Send
        + Sync
        + 'static,
>;

// Internal type for storing services
type ServiceStore = RwLock<BTreeMap<String, TypeErasedCallback>>;

/// A mock ROS implementation that can be substituted for any roslibrust backend in unit tests.
///
/// Implements [TopicProvider] and [ServiceProvider] to provide basic ros functionality.
#[derive(Clone)]
#[allow(clippy::type_complexity)]
pub struct MockRos {
    // We could probably achieve some fancier type erasure than actually serializing the data
    // but this ends up being pretty simple
    topics: Arc<RwLock<BTreeMap<String, (Channel::Sender<Vec<u8>>, Channel::Receiver<Vec<u8>>)>>>,
    services: Arc<ServiceStore>,
}

impl Default for MockRos {
    fn default() -> Self {
        Self::new()
    }
}

impl MockRos {
    pub fn new() -> Self {
        Self {
            topics: Arc::new(RwLock::new(BTreeMap::new())),
            services: Arc::new(RwLock::new(BTreeMap::new())),
        }
    }
}

// This is a very basic mocking of sending and receiving messages over topics
// It does not implement automatic shutdown of topics on dropping
impl TopicProvider for MockRos {
    type Publisher<T: RosMessageType> = MockPublisher<T>;
    type Subscriber<T: RosMessageType> = MockSubscriber<T>;

    async fn advertise<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Publisher<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let topic_str = topic.as_ref();
        // Check if we already have this channel
        {
            let topics = self.topics.read().await;
            if let Some((sender, _)) = topics.get(topic_str) {
                debug!("Issued new publisher to existing topic {}", topic_str);
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
        topics.insert(topic_str.to_string(), tx_rx);
        debug!("Created new publisher and channel for topic {}", topic_str);
        Ok(MockPublisher {
            sender: tx_copy,
            _marker: Default::default(),
        })
    }

    async fn subscribe<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Subscriber<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let topic_str = topic.as_ref();
        // Check if we already have this channel
        {
            let topics = self.topics.read().await;
            if let Some((_, receiver)) = topics.get(topic_str) {
                debug!("Issued new subscriber to existing topic {}", topic_str);
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
        topics.insert(topic_str.to_string(), tx_rx);
        debug!("Created new subscriber and channel for topic {}", topic_str);
        Ok(MockSubscriber {
            receiver: rx_copy,
            _marker: Default::default(),
        })
    }
}

/// The handle type returned by calling [MockRos::service_client].
/// Represents a ROS service connection and allows the service to be called multiple times.
pub struct MockServiceClient<T: RosServiceType> {
    // We hold a weak reference to the service store so we can look up the most recently registered
    // service server
    handle: std::sync::Weak<ServiceStore>,
    // We hold the key we'll use to look up the service server
    topic: String,
    // Maker type so Rust understand we're using T internnally without actually holding one.
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosServiceType> Service<T> for MockServiceClient<T> {
    async fn call(&self, request: &T::Request) -> roslibrust_common::Result<T::Response> {
        // Check that service store still exists otherwise ROS has been dropped
        let services = match self.handle.upgrade() {
            Some(services) => services,
            None => {
                return Err(Error::ServerError(
                    "No connection to MockRos backend? Has it been dropped?".to_string(),
                ))
            }
        };

        // Because we might be called in test circumstances where there is a race condition between
        // creating the service server and calling it.
        // We do a yield here to hopefully put ourselves at the back of the task queue,
        // and let tokio create the service_server before we actually make a call.
        tokio::task::yield_now().await;

        // Check if a service exists for this topic
        let callback = {
            let services = services.read().await;
            services.get(&self.topic).cloned()
        };
        let callback = match callback {
            Some(callback) => callback,
            None => {
                return Err(Error::ServerError(format!(
                    "No service server found for topic: {}",
                    self.topic
                )))
            }
        };

        // Serialize incoming data
        let data =
            bincode::serialize(request).map_err(|e| Error::SerializationError(e.to_string()))?;

        let callback = callback.clone();
        // Wrap in a spawn_blocking to uphold trait expectations.
        // Actual service call happens here
        let response = tokio::task::spawn_blocking(move || (callback)(data))
            .await
            .map_err(|_e| Error::Disconnected)?
            .map_err(|e| Error::SerializationError(e.to_string()))?;

        // Deserialize response
        let response = bincode::deserialize(&response[..])
            .map_err(|e| Error::SerializationError(e.to_string()))?;
        Ok(response)
    }
}

impl ServiceProvider for MockRos {
    type ServiceClient<T: RosServiceType> = MockServiceClient<T>;
    type ServiceServer = ();

    async fn call_service<SrvType: RosServiceType>(
        &self,
        service: impl ToGlobalTopicName,
        request: SrvType::Request,
    ) -> Result<SrvType::Response> {
        let service: GlobalTopicName = service.to_global_name()?;
        let client = MockRos::service_client::<SrvType>(self, service.as_ref()).await?;
        client.call(&request).await
    }

    async fn service_client<SrvType: RosServiceType + 'static>(
        &self,
        service: impl ToGlobalTopicName,
    ) -> Result<Self::ServiceClient<SrvType>> {
        let service: GlobalTopicName = service.to_global_name()?;
        // TODO this is currently infallible
        // We don't yet support a way to simulate ROS disconnecting in a test
        Ok(MockServiceClient {
            handle: Arc::downgrade(&self.services),
            topic: String::from(service),
            _marker: Default::default(),
        })
    }

    async fn advertise_service<SrvType: RosServiceType + 'static, F: ServiceFn<SrvType>>(
        &self,
        service: impl ToGlobalTopicName,
        server: F,
    ) -> Result<Self::ServiceServer> {
        let service: GlobalTopicName = service.to_global_name()?;
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
        services.insert(String::from(service), erased_closure);

        // We technically need to hand back a token that shuts the service down here
        // But we haven't implemented that yet in this mock
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
            .advertise::<std_msgs::String>("/test_topic")
            .await
            .unwrap();
        let mut sub_handle = mock_ros
            .subscribe::<std_msgs::String>("/test_topic")
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
            .advertise_service::<std_srvs::SetBool, _>("/test_service", server_fn)
            .await
            .unwrap();

        let client = mock_topics
            .service_client::<std_srvs::SetBool>("/test_service")
            .await
            .unwrap();

        let request = std_srvs::SetBoolRequest { data: true };

        let response = client.call(&request).await.unwrap();
        assert!(response.success);
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
    async fn test_client_server_order_does_not_matter() {
        // Test covers a bug case where clients wouldn't connect if created before servers
        let mock_ros = MockRos::new();
        let client = mock_ros
            .service_client::<std_srvs::SetBool>("/test_service")
            .await
            .unwrap();

        let res1 = client.call(&std_srvs::SetBoolRequest { data: true }).await;
        assert!(
            res1.is_err(),
            "Shouldn't be able to call service before it's advertised"
        );

        // advertise the service now
        let server_fn = |request: std_srvs::SetBoolRequest| {
            Ok(std_srvs::SetBoolResponse {
                success: request.data,
                message: "You set my bool!".to_string(),
            })
        };
        mock_ros
            .advertise_service::<std_srvs::SetBool, _>("/test_service", server_fn)
            .await
            .unwrap();

        // should work now
        let request = std_srvs::SetBoolRequest { data: true };
        let response = client.call(&request).await.unwrap();
        assert!(response.success);
        assert_eq!(response.message, "You set my bool!");
    }

    #[tokio::test(start_paused = true)]
    async fn test_no_pause_needed_for_spawned_server() {
        // Test covers a bug where if you spawned a server in a different task
        // It could appear to not be up due to a race condition.
        let mock_ros = MockRos::new();

        let client = mock_ros
            .service_client::<std_srvs::SetBool>("/test_service")
            .await
            .unwrap();

        async fn spawn_server(mock_ros: MockRos) {
            let server_fn = |request: std_srvs::SetBoolRequest| {
                Ok(std_srvs::SetBoolResponse {
                    success: request.data,
                    message: "You set my bool!".to_string(),
                })
            };
            mock_ros
                .advertise_service::<std_srvs::SetBool, _>("/test_service", server_fn)
                .await
                .unwrap();

            let _ = tokio::signal::ctrl_c().await;
        }

        // Start the server in a spawned task
        tokio::spawn(spawn_server(mock_ros.clone()));

        // Prior to introducing a yield_now() in ServiceClient::call() this would fail consistently
        let request = std_srvs::SetBoolRequest { data: true };
        let response = client.call(&request).await.unwrap();
        assert!(response.success);
        assert_eq!(response.message, "You set my bool!");
    }
}
