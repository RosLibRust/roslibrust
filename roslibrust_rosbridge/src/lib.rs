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
//!     let publisher = ros.advertise::<std_msgs::String>("my_topic").await?;
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

use roslibrust_common::*;

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

/// Communication primitives for the rosbridge_suite protocol
mod comm;

use futures_util::stream::{SplitSink, SplitStream};
use std::collections::HashMap;
use tokio::net::TcpStream;
use tokio_tungstenite::*;
use tungstenite::Message;

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

// Implement the generic Service trait for our ServiceClient
impl<T: RosServiceType> Service<T> for crate::ServiceClient<T> {
    async fn call(&self, request: &T::Request) -> Result<T::Response> {
        // TODO sort out the reference vs clone stuff here
        ServiceClient::call(&self, request.clone()).await
    }
}

impl ServiceProvider for crate::ClientHandle {
    type ServiceClient<T: RosServiceType> = crate::ServiceClient<T>;
    type ServiceServer = crate::ServiceHandle;

    async fn call_service<T: RosServiceType>(
        &self,
        topic: &str,
        request: T::Request,
    ) -> Result<T::Response> {
        self.call_service::<T>(topic, request).await
    }

    async fn service_client<T: RosServiceType + 'static>(
        &self,
        topic: &str,
    ) -> Result<Self::ServiceClient<T>> {
        self.service_client::<T>(topic).await
    }

    async fn advertise_service<T: RosServiceType + 'static, F>(
        &self,
        topic: &str,
        server: F,
    ) -> Result<Self::ServiceServer>
    where
        F: ServiceFn<T>,
    {
        self.advertise_service(topic, server).await
    }
}

// Implementation of TopicProvider trait for rosbridge client
impl TopicProvider for crate::ClientHandle {
    type Publisher<T: RosMessageType> = crate::Publisher<T>;
    type Subscriber<T: RosMessageType> = crate::Subscriber<T>;

    async fn advertise<T: RosMessageType>(&self, topic: &str) -> Result<Self::Publisher<T>> {
        self.advertise::<T>(topic.as_ref()).await
    }

    async fn subscribe<T: RosMessageType>(&self, topic: &str) -> Result<Self::Subscriber<T>> {
        self.subscribe(topic).await
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
            Err(e) => Err(Error::IoError(std::io::Error::new(
                std::io::ErrorKind::Other,
                e,
            ))),
        }
    }
}

#[cfg(test)]
mod test {
    use roslibrust_common::*;

    // Prove that we've implemented the topic provider trait fully for ClientHandle
    #[test]
    #[should_panic]
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
}
