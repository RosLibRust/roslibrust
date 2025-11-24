use std::future::Future;

use crate::{Result, RosMessageType, RosServiceType, ServiceFn};

// ANCHOR: publish
/// Indicates that something is a publisher and has our expected publish
/// Implementors of this trait are expected to auto-cleanup the publisher when dropped
pub trait Publish<T: RosMessageType> {
    // Note: this is really just syntactic de-sugared `async fn`
    // However see: https://blog.rust-lang.org/2023/12/21/async-fn-rpit-in-traits.html
    // This generates a warning is rust as of writing due to ambiguity around the "Send-ness" of the return type
    // We only plan to work with multi-threaded work stealing executors (e.g. tokio) so we're manually specifying Send
    fn publish(&self, data: &T) -> impl Future<Output = Result<()>> + Send;
}
// ANCHOR_END: publish

/// Represents that an object can act as a subscriber.
/// Types returned by calling [TopicProvider::subscribe], implement this trait.
/// Types implementing this trait are expected to auto-cleanup the subscriber when dropped, and de-register themselves with ROS as needed.
pub trait Subscribe<T: RosMessageType>
where
    Self: Sized,
{
    // TODO need to solidify how we want errors to work with subscribers...
    // TODO ros1 currently requires mut for next, we should change that

    /// Returns the next message on the topic, or an Err as appropriate.
    /// [crate::Error] is currently quite generic, and the different backends can return different error variantes in different circumstances.
    /// We hope to clean-up this error type substantially in the future.
    fn next(&mut self) -> impl Future<Output = Result<T>> + Send;

    /// Converts the subscriber into an async [futures_core::Stream].
    /// This allows using the various adaptors in either [tokio_stream::StreamExt](https://docs.rs/tokio-stream/latest/tokio_stream/trait.StreamExt.html)
    /// or  [futures::stream::StreamExt](https://docs.rs/futures/latest/futures/stream/trait.StreamExt.html) to manipulate the stream.
    ///
    /// See [examples/tokio_stream_operations.rs](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust/examples/tokio_stream_operations.rs) for usage.
    ///
    /// Warning: The returned stream is infinite, and will automatically reconnect if the connection is lost.
    /// Calling collect() or fold() on the stream is likely to result in a deadlock.
    fn into_stream(mut self) -> impl futures_core::Stream<Item = Result<T>> {
        use async_stream::stream;
        stream! {
            loop {
                yield self.next().await;
            }
        }
    }
}

// ANCHOR: topic_provider
/// This trait generically describes the capability of something to act as an async interface to a set of topics
///
/// This trait is largely based on ROS concepts, but could be extended to other protocols / concepts.
/// Fundamentally, it assumes that topics are uniquely identified by a string name (likely an ASCII assumption is buried in here...).
/// It assumes topics only carry one data type, but is not expected to enforce that.
/// It assumes that all actions can fail due to a variety of causes, and by network interruption specifically.
pub trait TopicProvider {
    // These associated types makeup the other half of the API
    // They are expected to be "self-deregistering", where dropping them results in unadvertise or unsubscribe operations as appropriate
    // We require Publisher and Subscriber types to be Send + 'static so they can be sent into different tokio tasks once created
    type Publisher<T: RosMessageType>: Publish<T> + Send + Sync + 'static;
    type Subscriber<T: RosMessageType>: Subscribe<T> + Send + Sync + 'static;

    /// Advertises a topic to be published to and returns a type specific publisher to use.
    ///
    /// The returned publisher is expected to be "self de-registering", where dropping the publisher results in the appropriate unadvertise operation.
    fn advertise<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> impl Future<Output = Result<Self::Publisher<T>>> + Send;

    /// Subscribes to a topic and returns a type specific subscriber to use.
    ///
    /// The returned subscriber is expected to be "self de-registering", where dropping the subscriber results in the appropriate unsubscribe operation.
    fn subscribe<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> impl Future<Output = Result<Self::Subscriber<T>>> + Send;
}
// ANCHOR_END: topic_provider

/// Defines what it means to be something that is callable as a service
pub trait Service<T: RosServiceType> {
    fn call(&self, request: &T::Request) -> impl Future<Output = Result<T::Response>> + Send;
}

/// This trait is analogous to TopicProvider, but instead provides the capability to create service servers and service clients
pub trait ServiceProvider {
    type ServiceClient<T: RosServiceType>: Service<T> + Send + 'static;
    type ServiceServer: Send + 'static;

    /// A "oneshot" service call good for low frequency calls or where the service_provider may not always be available.
    fn call_service<T: RosServiceType>(
        &self,
        topic: &str,
        request: T::Request,
    ) -> impl Future<Output = Result<T::Response>> + Send;

    /// An optimized version of call_service that returns a persistent client that can be used to repeatedly call a service.
    /// Depending on backend this may provide a performance benefit over call_service.
    /// Dropping the returned client will perform all needed cleanup.
    fn service_client<T: RosServiceType + 'static>(
        &self,
        topic: &str,
    ) -> impl Future<Output = Result<Self::ServiceClient<T>>> + Send;

    /// Advertise a service function to be available for clients to call.
    /// A handle is returned that manages the lifetime of the service.
    /// Dropping the handle will perform all needed cleanup.
    /// The service will be active until the handle is dropped.
    /// The service will always be called inside a [tokio::task::spawn_blocking] call.
    /// It is generally okay to perform blocking actions inside the service function.
    ///  - See [roslibrust/examples/ros1_service_server.rs](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust/examples/ros1_service_server.rs) for a sync example of using this function.
    ///  - See [roslibrust/examples/ros1_async_service_server.rs](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust/examples/ros1_async_service_server.rs) for an async example of using this function.
    fn advertise_service<T: RosServiceType + 'static, F>(
        &self,
        topic: &str,
        server: F,
    ) -> impl Future<Output = Result<Self::ServiceServer>> + Send
    where
        F: ServiceFn<T>;
}

// ANCHOR: ros_trait
/// Represents all "standard" ROS functionality generically supported by roslibrust
///
/// Implementors of this trait behave like typical ROS node handles.
/// Cloning the handle does not create additional underlying connections, but instead simply returns another handle
/// to interact with the underlying node.
///
/// Implementors of this trait are expected to be "self de-registering", when the last node handle for a given
/// node is dropped, the underlying node is expected to be shut down and clean-up after itself
pub trait Ros: 'static + Send + Sync + TopicProvider + ServiceProvider + Clone {}
// ANCHOR_END: ros_trait

/// The Ros trait is auto implemented for any type that implements the required traits
impl<T: 'static + Send + Sync + TopicProvider + ServiceProvider + Clone> Ros for T {}
