//! A implementation of roslibrust's generic traits for native ROS1 communication.
//!
//! This is a pure rust re-implementation of ROS1 communication via xmlrpc and TCPROS.
//! This crate shows performance on par with roscpp.
//!
//! It is recommended to not use this crate directly and instead access if via the `roslibrust` crate with the `ros1` feature enabled.
//!
//! Basic Example:
//! ```no_run
//! // Normally accessed as roslibrust::{Result, TopicProvider, Publish}
//! use roslibrust_common::{Result, TopicProvider, Publish};
//! // Normally you'd use generated types from roslibrust::codegen
//! use roslibrust_test::ros1::*;
//! use roslibrust_ros1::NodeHandle;
//!
//! async fn my_behavior(ros: impl TopicProvider) -> Result<()> {
//!     let publisher = ros.advertise::<std_msgs::String>("/my_topic").await?;
//!     publisher.publish(&std_msgs::String { data: "Hello, world!".to_string() }).await?;
//!     Ok(())
//! }
//!
//! #[tokio::main]
//! async fn main() -> Result<()> {
//!     // Create a ros1 handle we can use
//!     let ros = NodeHandle::new("http://localhost:11311", "my_node").await?;
//!     // Use it like ros:
//!     my_behavior(ros).await?;
//!     Ok(())
//! }
//! ```

use roslibrust_common::topic_name::{GlobalTopicName, ToGlobalTopicName};
use roslibrust_common::Error;
use roslibrust_common::{
    DynamicMessage, DynamicMessageError, DynamicMessageResult, DynamicPublish, DynamicSubscribe,
    DynamicTopicProvider, MessageDescriptor, Publish, RosMessageType, RosServiceType, Service,
    ServiceFn, ServiceProvider, Subscribe, TopicProvider,
};

/// Serialize a runtime-selected message as a ROS1 message body.
///
/// The returned bytes do not include the four-byte TCPROS frame length.
pub fn serialize_dynamic_message(message: &DynamicMessage) -> DynamicMessageResult<Vec<u8>> {
    let mut bytes = Vec::new();
    let mut serializer = roslibrust_serde_rosmsg::Serializer::new(&mut bytes);
    message.serialize_with(&mut serializer)?;
    Ok(bytes)
}

/// Deserialize a ROS1 message body using a runtime-selected message descriptor.
pub fn deserialize_dynamic_message(
    descriptor: &'static MessageDescriptor,
    bytes: &[u8],
) -> DynamicMessageResult<DynamicMessage> {
    let length = u32::try_from(bytes.len()).map_err(|_| DynamicMessageError::Deserialize {
        type_name: descriptor.ros_type_name,
        message: "message body exceeds the ROS1 u32 length limit".to_owned(),
    })?;
    let mut deserializer = roslibrust_serde_rosmsg::Deserializer::new(bytes, length);
    descriptor.deserialize_with(&mut deserializer)
}

/// Deserialize a TCPROS-framed runtime-selected message.
fn deserialize_framed_dynamic_message(
    descriptor: &'static MessageDescriptor,
    bytes: &[u8],
) -> DynamicMessageResult<DynamicMessage> {
    if bytes.len() < 4 {
        return Err(DynamicMessageError::Deserialize {
            type_name: descriptor.ros_type_name,
            message: format!(
                "TCPROS frame is missing its four-byte length prefix (received {} bytes)",
                bytes.len()
            ),
        });
    }

    let body_length = u32::from_le_bytes(
        bytes[..4]
            .try_into()
            .expect("length prefix was checked to contain four bytes"),
    ) as usize;
    let body = &bytes[4..];
    if body_length != body.len() {
        return Err(DynamicMessageError::Deserialize {
            type_name: descriptor.ros_type_name,
            message: format!(
                "TCPROS frame declares a {body_length}-byte body, but received {} bytes",
                body.len()
            ),
        });
    }

    deserialize_dynamic_message(descriptor, body)
}

/// [master_client] module contains code for calling xmlrpc functions on the master
mod master_client;
pub use master_client::*;

mod names;

/// [node] module contains the central Node and NodeHandle APIs
mod node;
pub use node::*;

mod publisher;
pub use publisher::Publisher;
pub use publisher::PublisherAny;
mod service_client;
pub use service_client::ServiceClient;
mod subscriber;
pub use subscriber::Subscriber;
pub use subscriber::SubscriberAny;
mod service_server;
pub use service_server::ServiceServer;
mod tcpros;

/// Provides a common type alias for type erased service server functions.
/// Internally we use this type to store collections of server functions.
/// Uses Bytes for efficient handling of incoming request data.
pub(crate) type TypeErasedCallback = dyn Fn(bytes::Bytes) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
    + Send
    + Sync
    + 'static;

// Implement the generic roslibrust trait
impl TopicProvider for crate::NodeHandle {
    type Publisher<T: RosMessageType> = crate::Publisher<T>;
    type Subscriber<T: RosMessageType> = crate::Subscriber<T>;

    async fn advertise<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> roslibrust_common::Result<Self::Publisher<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        // TODO MAJOR: consider promoting queue size, making unlimited default
        NodeHandle::advertise::<MsgType>(self, topic.as_ref(), 10, false)
            .await
            .map_err(|e| e.into())
    }

    async fn subscribe<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> roslibrust_common::Result<Self::Subscriber<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        // TODO MAJOR: consider promoting queue size, making unlimited default
        NodeHandle::subscribe(self, topic.as_ref(), 10)
            .await
            .map_err(|e| e.into())
    }
}

/// Runtime-typed ROS1 publisher backed by [`PublisherAny`].
pub struct DynamicPublisher {
    publisher: PublisherAny,
    descriptor: &'static MessageDescriptor,
}

impl DynamicPublish for DynamicPublisher {
    fn descriptor(&self) -> &'static MessageDescriptor {
        self.descriptor
    }

    async fn publish(&self, data: &DynamicMessage) -> roslibrust_common::Result<()> {
        if data.descriptor().ros_type_name != self.descriptor.ros_type_name {
            return Err(Error::SerializationError(format!(
                "publisher expects {}, but message is {}",
                self.descriptor.ros_type_name,
                data.descriptor().ros_type_name
            )));
        }

        let body = serialize_dynamic_message(data)
            .map_err(|error| Error::SerializationError(error.to_string()))?;
        let length = u32::try_from(body.len()).map_err(|_| {
            Error::SerializationError("message body exceeds the ROS1 u32 length limit".to_owned())
        })?;
        let mut framed = Vec::with_capacity(body.len() + 4);
        framed.extend_from_slice(&length.to_le_bytes());
        framed.extend_from_slice(&body);
        self.publisher
            .publish(framed)
            .await
            .map_err(|error| Error::SerializationError(error.to_string()))
    }
}

/// Runtime-typed ROS1 subscriber backed by [`SubscriberAny`].
pub struct DynamicSubscriber {
    subscriber: SubscriberAny,
    descriptor: &'static MessageDescriptor,
}

impl DynamicSubscribe for DynamicSubscriber {
    async fn next(&mut self) -> roslibrust_common::Result<DynamicMessage> {
        match self.subscriber.next().await {
            Some(Ok(bytes)) => deserialize_framed_dynamic_message(self.descriptor, &bytes)
                .map_err(|error| Error::SerializationError(error.to_string())),
            Some(Err(error)) => Err(Error::Unexpected(anyhow::anyhow!(error))),
            None => Err(Error::Disconnected),
        }
    }
}

impl DynamicTopicProvider for crate::NodeHandle {
    type DynamicPublisher = DynamicPublisher;
    type DynamicSubscriber = DynamicSubscriber;

    async fn dynamic_advertise(
        &self,
        topic: impl ToGlobalTopicName,
        descriptor: &'static MessageDescriptor,
    ) -> roslibrust_common::Result<Self::DynamicPublisher> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let publisher = NodeHandle::advertise_any(
            self,
            topic.as_ref(),
            descriptor.ros_type_name,
            descriptor.definition,
            10,
            false,
        )
        .await?;
        Ok(DynamicPublisher {
            publisher,
            descriptor,
        })
    }

    async fn dynamic_subscribe(
        &self,
        topic: impl ToGlobalTopicName,
        descriptor: &'static MessageDescriptor,
    ) -> roslibrust_common::Result<Self::DynamicSubscriber> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let subscriber = NodeHandle::subscribe_any(self, topic.as_ref(), 10).await?;
        Ok(DynamicSubscriber {
            subscriber,
            descriptor,
        })
    }
}

impl<T: RosServiceType> Service<T> for ServiceClient<T> {
    async fn call(&self, request: &T::Request) -> roslibrust_common::Result<T::Response> {
        self.call(request).await
    }
}

impl ServiceProvider for crate::NodeHandle {
    type ServiceClient<T: RosServiceType> = crate::ServiceClient<T>;
    type ServiceServer = crate::ServiceServer;

    async fn call_service<SrvType: RosServiceType>(
        &self,
        service: impl ToGlobalTopicName,
        request: SrvType::Request,
    ) -> roslibrust_common::Result<SrvType::Response> {
        let service: GlobalTopicName = service.to_global_name()?;
        // TODO should have a more optimized version of this...
        let client = NodeHandle::service_client::<SrvType>(self, service.as_ref()).await?;
        client.call(&request).await
    }

    async fn service_client<SrvType: RosServiceType + 'static>(
        &self,
        service: impl ToGlobalTopicName,
    ) -> roslibrust_common::Result<Self::ServiceClient<SrvType>> {
        let service: GlobalTopicName = service.to_global_name()?;
        // TODO bad error mapping here...
        NodeHandle::service_client::<SrvType>(self, service.as_ref())
            .await
            .map_err(|e| e.into())
    }

    async fn advertise_service<SrvType: RosServiceType + 'static, F: ServiceFn<SrvType>>(
        &self,
        service: impl ToGlobalTopicName,
        server: F,
    ) -> roslibrust_common::Result<Self::ServiceServer> {
        let service: GlobalTopicName = service.to_global_name()?;
        NodeHandle::advertise_service::<SrvType, F>(self, service.as_ref(), server)
            .await
            .map_err(|e| e.into())
    }
}

impl<T: RosMessageType> Subscribe<T> for crate::Subscriber<T> {
    async fn next(&mut self) -> roslibrust_common::Result<T> {
        let res = crate::Subscriber::next(self).await;
        match res {
            Some(Ok(msg)) => Ok(msg),
            Some(Err(e)) => {
                log::error!("Subscriber got error: {e:?}");
                // TODO gotta do better error conversion / error types here
                Err(Error::Unexpected(anyhow::anyhow!(
                    "Subscriber got error: {e:?}"
                )))
            }
            None => {
                log::error!("Subscriber hit dropped channel");
                Err(Error::Unexpected(anyhow::anyhow!(
                    "Channel closed, something was dropped?"
                )))
            }
        }
    }
}

// Provide an implementation of publish for ros1 backend
impl<T: RosMessageType> Publish<T> for Publisher<T> {
    async fn publish(&self, data: &T) -> roslibrust_common::Result<()> {
        // TODO error type conversion here is terrible and we need to standardize error stuff badly
        self.publish(data)
            .await
            .map_err(|e| Error::SerializationError(e.to_string()))
    }
}

#[cfg(test)]
mod dynamic_message_tests {
    use super::*;
    use roslibrust_common::{DynamicField, DynamicValue};

    fn int16_descriptor() -> &'static MessageDescriptor {
        roslibrust_test::ros1::MESSAGE_REGISTRY
            .get("std_msgs/Int16")
            .unwrap()
    }

    fn int16_message(value: i16) -> DynamicMessage {
        int16_descriptor()
            .message(DynamicValue::Message(vec![DynamicField {
                name: "data".to_owned(),
                value: DynamicValue::I16(value),
            }]))
            .unwrap()
    }

    #[test]
    fn ros1_codec_round_trips_runtime_selected_message() {
        let descriptor = int16_descriptor();
        let message = int16_message(42);

        let bytes = serialize_dynamic_message(&message).unwrap();
        assert_eq!(bytes, 42_i16.to_le_bytes());
        assert_eq!(
            deserialize_dynamic_message(descriptor, &bytes)
                .unwrap()
                .value(),
            message.value()
        );
    }

    #[test]
    fn ros1_codec_deserializes_tcpros_framed_message() {
        let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
            .get("std_msgs/String")
            .unwrap();
        let message = descriptor
            .message(DynamicValue::Message(vec![DynamicField {
                name: "data".to_owned(),
                value: DynamicValue::String("constructed message".to_owned()),
            }]))
            .unwrap();
        let body = serialize_dynamic_message(&message).unwrap();
        let mut frame = Vec::from((body.len() as u32).to_le_bytes());
        frame.extend_from_slice(&body);

        assert_eq!(
            deserialize_framed_dynamic_message(descriptor, &frame)
                .unwrap()
                .value(),
            message.value()
        );
    }

    #[test]
    fn ros1_codec_rejects_missing_tcpros_length_prefix() {
        let error = deserialize_framed_dynamic_message(int16_descriptor(), &[0, 0, 0]).unwrap_err();

        assert!(matches!(error, DynamicMessageError::Deserialize { .. }));
        assert!(error
            .to_string()
            .contains("missing its four-byte length prefix"));
    }

    #[test]
    fn ros1_codec_rejects_mismatched_tcpros_body_length() {
        let error = deserialize_framed_dynamic_message(int16_descriptor(), &[3, 0, 0, 0, 42, 0])
            .unwrap_err();

        assert!(matches!(error, DynamicMessageError::Deserialize { .. }));
        assert!(error
            .to_string()
            .contains("declares a 3-byte body, but received 2 bytes"));
    }
}

#[cfg(test)]
mod test {
    use roslibrust_common::Ros;
    use roslibrust_common::TopicProvider;

    // Prove that we've implemented the topic provider trait fully for NodeHandle
    #[test]
    #[should_panic]
    #[allow(clippy::unnecessary_literal_unwrap)]
    fn topic_provider_can_be_used_with_ros1() {
        struct MyClient<T: TopicProvider> {
            _client: T,
        }

        // Kinda a hack way to make the compiler prove it could construct a MyClient<NodeHandle> with out actually
        // constructing one at runtime
        let new_mock: Result<crate::NodeHandle, _> = Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            // Will panic here which is expect, this test just needs to compile to prove
            // NodeHandle implements TopicProvider
            _client: new_mock.unwrap(), // panic
        };
    }

    #[test]
    #[should_panic]
    #[allow(clippy::unnecessary_literal_unwrap)]
    fn confirm_node_handle_impls_ros() {
        struct MyClient<T: Ros> {
            _client: T,
        }

        let new_mock: Result<crate::NodeHandle, _> = Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            // Will panic here which is expect, this test just needs to compile to prove
            // NodeHandle implements Ros
            _client: new_mock.unwrap(),
        };
    }
}
