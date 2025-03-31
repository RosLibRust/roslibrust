use crate::{Result, RosMessageType, RosServiceType, ServiceFn};
use std::future::Future;

/// Indicates that something is a publisher and has our expected publish
/// Implementors of this trait are expected to auto-cleanup the publisher when dropped
pub trait Publish<T: RosMessageType> {
    // Note: this is really just syntactic de-sugared `async fn`
    // However see: https://blog.rust-lang.org/2023/12/21/async-fn-rpit-in-traits.html
    // This generates a warning is rust as of writing due to ambiguity around the "Send-ness" of the return type
    // We only plan to work with multi-threaded work stealing executors (e.g. tokio) so we're manually specifying Send
    fn publish(&self, data: &T) -> impl Future<Output = Result<()>> + Send;
}

/// Indicates that something is a subscriber and has our expected subscribe method
/// Implementors of this trait are expected to auto-cleanup the subscriber when dropped
pub trait Subscribe<T: RosMessageType> {
    // TODO need to solidify how we want errors to work with subscribers...
    // TODO ros1 currently requires mut for next, we should change that
    fn next(&mut self) -> impl Future<Output = Result<T>> + Send;
}

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
    type Publisher<T: RosMessageType>: Publish<T> + Send + 'static;
    type Subscriber<T: RosMessageType>: Subscribe<T> + Send + 'static;

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

/// Defines what it means to be something that is callable as a service
pub trait Service<T: RosServiceType> {
    fn call(&self, request: &T::Request) -> impl Future<Output = Result<T::Response>> + Send;
}

/// This trait is analogous to TopicProvider, but instead provides the capability to create service servers and service clients
pub trait ServiceProvider {
    type ServiceClient<T: RosServiceType>: Service<T> + Send + 'static;
    type ServiceServer;

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
    /// It is recommended to use [ServiceProvider::advertise_async_service] instead of this function.
    /// Blocking inside this function can starve the tokio executor and cause issues for your application.
    fn advertise_service<T: RosServiceType + 'static, F>(
        &self,
        topic: &str,
        server: F,
    ) -> impl Future<Output = Result<Self::ServiceServer>> + Send
    where
        F: ServiceFn<T>;

    /// Advertise an async service function to be available for clients to call.
    /// A handle is returned that manages the lifetime of the service.
    /// Dropping the handle will perform all needed cleanup.
    /// The service will be active until the handle is dropped.
    fn advertise_async_service<T: RosServiceType + 'static, F, Fut>(
        &self,
        topic: &str,
        server: F,
    ) -> impl Future<Output = Result<Self::ServiceServer>> + Send
    where
        F: Fn(T::Request) -> Fut + Send + Sync + 'static,
        Fut: Future<
                Output = std::result::Result<T::Response, Box<dyn std::error::Error + Send + Sync>>,
            > + Send
            + 'static;
}

/// Represents all "standard" ROS functionality generically supported by roslibrust
///
/// Implementors of this trait behave like typical ROS1 node handles.
/// Cloning the handle does not create additional underlying connections, but instead simply returns another handle
/// to interact with the underlying node.
///
/// Implementors of this trait are expected to be "self de-registering", when the last node handle for a given
/// node is dropped, the underlying node is expected to be shut down and clean-up after itself
pub trait Ros: 'static + Send + Sync + TopicProvider + ServiceProvider + Clone {}

/// The Ros trait is auto implemented for any type that implements the required traits
impl<T: 'static + Send + Sync + TopicProvider + ServiceProvider + Clone> Ros for T {}

#[cfg(test)]
mod test {
    // This test specifically fails because TopicProvider is not object safe
    // Traits that have methods with generic parameters cannot be object safe in rust (currently)
    // #[test]
    // fn topic_provider_can_be_constructed() -> TestResult {
    //     let x: Box<dyn TopicProvider> = Box::new(ClientHandle::new(""));
    //     Ok(())
    // }
}
