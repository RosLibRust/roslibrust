//! A implementation of roslibrust's generic traits for rosbridge_suite communication.
//!
//! [rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite) provides a useful websocket interface to ROS.
//!
//! This server operates over a single port making it easier to use in constrained networking environments and support TLS.
//!
//! Additionally, rosbridge_server exists for both ROS1 and ROS2 with an unchanged API meaning code targeting rosbridge can be used with both ROS1 and ROS2.
//!
//! This backend is fundamentally less efficient than native ROS1 communication, but its added flexibility can be very useful.
//!
//! Basic Example:
//! ```no_run
//! // Normally accessed as roslibrust::{Result, TopicProvider, Publish}
//! use roslibrust_common::{Result, TopicProvider, Publish};
//! // Normally you'd use generated types from roslibrust::codegen
//! use roslibrust_test::ros1::*;
//! use roslibrust_rosbridge::ClientHandle;
//!
//! async fn my_behavior(ros: impl TopicProvider) -> Result<()> {
//!     let publisher = ros.advertise::<std_msgs::String>("/my_topic").await?;
//!     publisher.publish(&std_msgs::String { data: "Hello, world!".to_string() }).await?;
//!     Ok(())
//! }
//!
//! #[tokio::main]
//! async fn main() -> Result<()> {
//!     // Create a rosbridge handle we can use
//!     let ros = ClientHandle::new("ws://localhost:9090").await?;
//!     // Use it like ros:
//!     my_behavior(ros).await?;
//!     Ok(())
//! }
//! ```

use roslibrust_common::topic_name::{GlobalTopicName, ToGlobalTopicName};
use roslibrust_common::*;

/// Serialize a runtime-selected message as the JSON object used by rosbridge.
pub fn serialize_dynamic_message(message: &DynamicMessage) -> DynamicMessageResult<Vec<u8>> {
    let mut bytes = Vec::new();
    let mut serializer = serde_json::Serializer::new(&mut bytes);
    message.serialize_with(&mut serializer)?;
    Ok(bytes)
}

/// Deserialize a rosbridge JSON message using a runtime-selected descriptor.
pub fn deserialize_dynamic_message<'de>(
    descriptor: &'static MessageDescriptor,
    bytes: &'de [u8],
) -> DynamicMessageResult<DynamicMessage> {
    let mut deserializer = serde_json::Deserializer::from_slice(bytes);
    descriptor.deserialize_with(&mut deserializer)
}

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
// Standard return type for all tests to use
#[cfg(test)]
#[allow(dead_code)]
type TestResult = std::result::Result<(), anyhow::Error>;

#[cfg(test)]
mod dynamic_message_tests {
    use super::*;

    #[test]
    fn json_codec_round_trips_runtime_selected_message() {
        let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
            .get("std_msgs/Int16")
            .unwrap();
        let message = descriptor
            .message(DynamicValue::Message(vec![DynamicField {
                name: "data".to_owned(),
                value: DynamicValue::I16(42),
            }]))
            .unwrap();

        let bytes = serialize_dynamic_message(&message).unwrap();
        assert_eq!(bytes, br#"{"data":42}"#);
        assert_eq!(
            deserialize_dynamic_message(descriptor, &bytes)
                .unwrap()
                .value(),
            message.value()
        );
    }
}

/// Communication primitives for the rosbridge_suite protocol
mod comm;
mod rosapi_discovery;

use futures_util::stream::{SplitSink, SplitStream};
use std::collections::HashMap;
use std::time::Duration;
use tokio::net::TcpStream;
use tokio_tungstenite::*;
use tungstenite::Message;

const SERVICE_TYPE_QUERY_TIMEOUT: Duration = Duration::from_secs(5);

/// Used for type erasure of message type so that we can store arbitrary handles
type Callback = std::sync::Arc<dyn Fn(&str) + Send + Sync>;

/// Type erasure of callback for a service
/// Internally this will covert the input string to the Request type
/// Send that converted type into the user's callback
/// Get the result of the user's callback and then serialize that so it can be transmitted
// TODO reconsider use of serde_json::Value here vs. tungstenite::Message vs. &str
// Not quite sure what type we want to erase to?
// I can make a good argument for &str because that should be generic even if we switch
// backends - Carter 2022-10-6
// TODO move out of rosbridge and into "common"
pub(crate) type ServiceCallback = std::sync::Arc<
    dyn Fn(&str) -> std::result::Result<serde_json::Value, Box<dyn std::error::Error + Send + Sync>>
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

/// Rosbridge doesn't have the same concept of service client that ros1 native has
/// This type is used to replicate the api of ros1::ServiceClient, but really it
/// just a thin wrapper around call_service() and has no performance benefits
pub struct ServiceClient<T> {
    _marker: std::marker::PhantomData<T>,
    // Need a client handle so we can fwd to call_service
    client: ClientHandle,
    // Need the topic we are calling on
    topic: String,
}

/// A rosbridge service client using runtime-selected request and response types.
pub struct DynamicServiceClient {
    client: ClientHandle,
    topic: String,
    descriptor: &'static ServiceDescriptor,
}

impl DynamicService for DynamicServiceClient {
    fn descriptor(&self) -> &'static ServiceDescriptor {
        self.descriptor
    }

    async fn call(&self, request: &DynamicMessage) -> Result<DynamicMessage> {
        if request.descriptor().ros_type_name != self.descriptor.request.ros_type_name {
            return Err(Error::SerializationError(format!(
                "service {} expects request {}, but message is {}",
                self.descriptor.ros_service_name,
                self.descriptor.request.ros_type_name,
                request.descriptor().ros_type_name
            )));
        }
        let value = serde_json::to_value(request.value())
            .map_err(|error| Error::SerializationError(error.to_string()))?;
        let response = self.client.call_service_value(&self.topic, value).await?;
        if let Some(error) = response.as_str() {
            return Err(Error::ServerError(error.to_owned()));
        }
        self.descriptor
            .response
            .message_from(&response)
            .map_err(|error| Error::SerializationError(error.to_string()))
    }
}

impl<T: RosServiceType> ServiceClient<T> {
    pub async fn call(&self, request: T::Request) -> Result<T::Response> {
        self.client
            .call_service::<T>(self.topic.as_str(), request)
            .await
    }
}

/// Our underlying communication socket type (maybe move to comm?)
type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

/// We split our underlying socket into two halves with separate locks on read and write.
/// This is the read half.
type Reader = SplitStream<Socket>;

/// We split our underlying socket into two halves with separate locks on read and write.
/// This is the write half.
type Writer = SplitSink<Socket, Message>;

/// Topics have a fundamental queue *per subscriber* this is te queue type used for each subscriber.
type MessageQueue<T> = deadqueue::limited::Queue<T>;

// TODO queue size should be configurable for subscribers
const QUEUE_SIZE: usize = 1_000;

/// Internal tracking structure used to maintain information about each subscription our client has
/// with rosbridge.
pub(crate) struct Subscription {
    /// Map of "subscriber id" -> callback
    /// Subscriber ids are randomly generated
    /// There will be one callback per subscriber to the topic.
    // Note: don't need dashmap here as the subscription is already inside a dashmap
    pub(crate) handles: HashMap<uuid::Uuid, Callback>,
    /// Name of ros type (package_name/message_name), used for re-subscribes
    pub(crate) topic_type: String,
}

pub(crate) struct PublisherHandle {
    pub(crate) topic_type: String,
}

fn normalize_graph_type(type_name: String) -> String {
    roslibrust_common::normalize_ros_type_name(&type_name).into_owned()
}

// Implement the generic Service trait for our ServiceClient
impl<T: RosServiceType> Service<T> for crate::ServiceClient<T> {
    async fn call(&self, request: &T::Request) -> Result<T::Response> {
        // TODO sort out the reference vs clone stuff here
        ServiceClient::call(self, request.clone()).await
    }
}

impl ServiceProvider for crate::ClientHandle {
    type ServiceClient<T: RosServiceType> = crate::ServiceClient<T>;
    type ServiceServer = crate::ServiceHandle;

    async fn call_service<SrvType: RosServiceType>(
        &self,
        service: impl ToGlobalTopicName,
        request: SrvType::Request,
    ) -> Result<SrvType::Response> {
        let service: GlobalTopicName = service.to_global_name()?;
        ClientHandle::call_service::<SrvType>(self, service.as_ref(), request).await
    }

    async fn service_client<SrvType: RosServiceType + 'static>(
        &self,
        service: impl ToGlobalTopicName,
    ) -> Result<Self::ServiceClient<SrvType>> {
        let service: GlobalTopicName = service.to_global_name()?;
        ClientHandle::service_client::<SrvType>(self, service.as_ref()).await
    }

    async fn advertise_service<SrvType: RosServiceType + 'static, F: ServiceFn<SrvType>>(
        &self,
        service: impl ToGlobalTopicName,
        server: F,
    ) -> Result<Self::ServiceServer> {
        let service: GlobalTopicName = service.to_global_name()?;
        ClientHandle::advertise_service(self, service.as_ref(), server).await
    }
}

impl DynamicServiceProvider for crate::ClientHandle {
    type DynamicServiceClient = crate::DynamicServiceClient;

    async fn dynamic_call_service(
        &self,
        service: impl ToGlobalTopicName,
        descriptor: &'static ServiceDescriptor,
        request: DynamicMessage,
    ) -> Result<DynamicMessage> {
        let client =
            DynamicServiceProvider::dynamic_service_client(self, service, descriptor).await?;
        client.call(&request).await
    }

    async fn dynamic_service_client(
        &self,
        service: impl ToGlobalTopicName,
        descriptor: &'static ServiceDescriptor,
    ) -> Result<Self::DynamicServiceClient> {
        let service: GlobalTopicName = service.to_global_name()?;
        Ok(crate::DynamicServiceClient {
            client: self.clone(),
            topic: String::from(service),
            descriptor,
        })
    }
}

impl GraphProvider for crate::ClientHandle {
    async fn list_topics(&self) -> Result<Vec<TopicInfo>> {
        let response = self
            .call_service::<rosapi_discovery::Topics>(
                "/rosapi/topics",
                rosapi_discovery::EmptyRequest {},
            )
            .await?;

        if response.topics.len() != response.types.len() {
            return Err(Error::SerializationError(format!(
                "rosapi returned {} topic names but {} topic types",
                response.topics.len(),
                response.types.len()
            )));
        }

        let mut topics: Vec<_> = response
            .topics
            .into_iter()
            .zip(response.types)
            .map(|(name, type_name)| TopicInfo {
                name,
                type_name: normalize_graph_type(type_name),
            })
            .collect();
        topics.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(topics)
    }

    /// List all services visible through rosbridge.
    ///
    /// # Type Resolution
    ///
    /// This implementation queries the rosapi node for service information. The rosapi node
    /// follows the convention of returning an empty string for service types that cannot be
    /// determined (e.g., due to connection failures or missing metadata), which matches the
    /// behavior documented in the [`GraphProvider::list_services`] trait method.
    async fn list_services(&self) -> Result<Vec<ServiceInfo>> {
        let response = self
            .call_service::<rosapi_discovery::Services>(
                "/rosapi/services",
                rosapi_discovery::EmptyRequest {},
            )
            .await?;

        // Spawn parallel tasks to query service types for all services
        let probe_tasks: Vec<_> = response
            .services
            .into_iter()
            .map(|service| {
                let handle = self.clone();
                tokio::spawn(async move {
                    let type_name = match tokio::time::timeout(
                        SERVICE_TYPE_QUERY_TIMEOUT,
                        handle.call_service::<rosapi_discovery::ServiceType>(
                            "/rosapi/service_type",
                            rosapi_discovery::ServiceTypeRequest {
                                service: service.clone(),
                            },
                        ),
                    )
                    .await
                    {
                        Ok(Ok(service_type)) => normalize_graph_type(service_type.type_name),
                        Ok(Err(err)) => {
                            log::warn!(
                                "Failed to resolve type for service {service}: {err}. Returning empty type."
                            );
                            String::new()
                        }
                        Err(_) => {
                            log::warn!(
                                "Timed out resolving type for service {service}. Returning empty type."
                            );
                            String::new()
                        }
                    };
                    ServiceInfo {
                        name: service,
                        type_name,
                    }
                })
            })
            .collect();

        // Collect results from all parallel tasks
        let mut services = Vec::with_capacity(probe_tasks.len());
        for task in probe_tasks {
            if let Ok(service_info) = task.await {
                services.push(service_info);
            }
        }

        services.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(services)
    }
}

// Implementation of TopicProvider trait for rosbridge client
impl TopicProvider for crate::ClientHandle {
    type Publisher<T: RosMessageType> = crate::Publisher<T>;
    type Subscriber<T: RosMessageType> = crate::Subscriber<T>;

    async fn advertise<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Publisher<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        ClientHandle::advertise::<MsgType>(self, topic.as_ref()).await
    }

    async fn subscribe<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Subscriber<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        ClientHandle::subscribe(self, topic.as_ref()).await
    }
}

impl DynamicPublish for crate::DynamicPublisher {
    fn descriptor(&self) -> &'static MessageDescriptor {
        self.descriptor
    }

    async fn publish(&self, data: &DynamicMessage) -> Result<()> {
        self.publish_dynamic(data).await
    }
}

impl DynamicSubscribe for crate::DynamicSubscriber {
    async fn next(&mut self) -> Result<DynamicMessage> {
        self.next_dynamic().await
    }
}

impl DynamicTopicProvider for crate::ClientHandle {
    type DynamicPublisher = crate::DynamicPublisher;
    type DynamicSubscriber = crate::DynamicSubscriber;

    async fn dynamic_advertise(
        &self,
        topic: impl ToGlobalTopicName,
        descriptor: &'static MessageDescriptor,
    ) -> Result<Self::DynamicPublisher> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        ClientHandle::_dynamic_advertise(self, topic.as_ref(), descriptor).await
    }

    async fn dynamic_subscribe(
        &self,
        topic: impl ToGlobalTopicName,
        descriptor: &'static MessageDescriptor,
    ) -> Result<Self::DynamicSubscriber> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        ClientHandle::_dynamic_subscribe(self, topic.as_ref(), descriptor).await
    }
}

impl<T: RosMessageType> Subscribe<T> for crate::Subscriber<T> {
    async fn next(&mut self) -> Result<T> {
        // TODO: rosbridge subscribe really should emit errors...
        Ok(crate::Subscriber::next(self).await)
    }
}

// Provide an implementation of publish for rosbridge backend
impl<T: RosMessageType> Publish<T> for crate::Publisher<T> {
    async fn publish(&self, data: &T) -> Result<()> {
        self.publish(data).await
    }
}

/// Defines a conversion from a tungstenite error to a roslibrust error
/// Since neither error type is owned by this crate we use a trait to define the conversion
trait MapError {
    type T;
    fn map_to_roslibrust(self) -> Result<Self::T>;
}

// Implementation of conversion from tungstenite error to roslibrust error for a result
impl<T: Send + Sync> MapError for std::result::Result<T, tokio_tungstenite::tungstenite::Error> {
    type T = T;
    fn map_to_roslibrust(self) -> Result<T> {
        match self {
            Ok(t) => Ok(t),
            Err(e) => Err(Error::IoError(std::io::Error::other(e))),
        }
    }
}

#[cfg(test)]
mod test {
    use super::normalize_graph_type;
    use roslibrust_common::*;

    // Prove that we've implemented the topic provider trait fully for ClientHandle
    #[test]
    #[should_panic]
    #[allow(clippy::unnecessary_literal_unwrap)]
    fn topic_provider_can_be_used_at_compile_time() {
        struct MyClient<T: TopicProvider> {
            _client: T,
        }

        // Kinda a hack way to make the compiler prove it could construct a MyClient<ClientHandle> with out actually
        // constructing one at runtime
        let new_mock: std::result::Result<crate::ClientHandle, _> =
            Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            _client: new_mock.unwrap(), // panic
        };
    }

    #[test]
    #[should_panic]
    #[allow(clippy::unnecessary_literal_unwrap)]
    fn confirm_client_handle_impls_ros() {
        struct MyClient<T: Ros> {
            _client: T,
        }

        let new_mock: std::result::Result<crate::ClientHandle, _> =
            Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            // Should panic here, but proves that ClientHandle implements Ros
            // when this test compiles
            _client: new_mock.unwrap(),
        };
    }

    #[test]
    fn normalizes_ros2_interface_graph_types() {
        assert_eq!(
            normalize_graph_type("std_msgs/msg/String".to_string()),
            "std_msgs/String"
        );
        assert_eq!(
            normalize_graph_type("std_srvs/srv/SetBool".to_string()),
            "std_srvs/SetBool"
        );
    }

    #[test]
    fn normalizes_ros2_dds_graph_types() {
        assert_eq!(
            normalize_graph_type("std_msgs::msg::dds_::String_".to_string()),
            "std_msgs/String"
        );
        assert_eq!(
            normalize_graph_type("std_srvs::srv::dds_::SetBool_".to_string()),
            "std_srvs/SetBool"
        );
    }

    #[test]
    fn leaves_ros1_graph_types_unchanged() {
        assert_eq!(
            normalize_graph_type("std_msgs/String".to_string()),
            "std_msgs/String"
        );
    }
}
