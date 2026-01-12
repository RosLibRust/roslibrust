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
    Publish, RosMessageType, RosServiceType, Service, ServiceFn, ServiceProvider, Subscribe,
    TopicProvider,
};

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
pub(crate) type TypeErasedCallback = dyn Fn(Vec<u8>) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
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
