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
use std::sync::{Arc, RwLock, Weak};

use bincode::Options;
use roslibrust_common::topic_name::{GlobalTopicName, ToGlobalTopicName};
use roslibrust_common::*;

use tokio::sync::broadcast as Channel;

use log::*;

type TypeErasedCallback = Arc<
    dyn Fn(Vec<u8>) -> std::result::Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
        + Send
        + Sync
        + 'static,
>;

struct TopicEntry {
    sender: Channel::Sender<Vec<u8>>,
    receiver: Channel::Receiver<Vec<u8>>,
    type_name: String,
    descriptor: &'static MessageDescriptor,
    registration: Weak<TopicRegistration>,
}

struct ServiceEntry {
    callback: TypeErasedCallback,
    type_name: String,
    registration: Weak<ServiceRegistration>,
}

type TopicStore = RwLock<BTreeMap<String, TopicEntry>>;
type ServiceStore = RwLock<BTreeMap<String, ServiceEntry>>;

// Stores own channels/callbacks, but only handles own registrations. Synchronous locks make
// cleanup immediate in Drop, even outside a Tokio runtime. No store lock is held across await.
struct TopicRegistration {
    store: Weak<TopicStore>,
    topic: String,
}

impl Drop for TopicRegistration {
    fn drop(&mut self) {
        if let Some(store) = self.store.upgrade() {
            let mut topics = store.write().unwrap();
            // A registration whose last handle is dropping can race with a new registration.
            // Only remove our own entry, never its successor.
            if topics
                .get(&self.topic)
                .is_some_and(|entry| std::ptr::eq(entry.registration.as_ptr(), self))
            {
                topics.remove(&self.topic);
            }
        }
    }
}

struct ServiceRegistration {
    store: Weak<ServiceStore>,
    service: String,
}

impl Drop for ServiceRegistration {
    fn drop(&mut self) {
        if let Some(store) = self.store.upgrade() {
            let removed = {
                let mut services = store.write().unwrap();
                if services
                    .get(&self.service)
                    .is_some_and(|entry| std::ptr::eq(entry.registration.as_ptr(), self))
                {
                    services.remove(&self.service)
                } else {
                    None
                }
            };
            // Captured user values may have their own Drop implementation. Release the lock
            // before destroying the callback so those destructors can safely use the backend.
            drop(removed);
        }
    }
}

/// A mock ROS implementation that can be substituted for any roslibrust backend in unit tests.
///
/// Implements [TopicProvider] and [ServiceProvider] to provide basic ros functionality.
/// Topics stay registered until the last publisher or subscriber handle is dropped.
/// Services stay registered until the last [`MockServiceServer`] handle is dropped.
/// Advertising an already active service returns an error; drop its handles before replacing it.
#[derive(Clone)]
#[allow(clippy::type_complexity)]
pub struct MockRos {
    // We could probably achieve some fancier type erasure than actually serializing the data
    // but this ends up being pretty simple
    topics: Arc<TopicStore>,
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

impl MockRos {
    // Atomically check the type, reuse a live registration, or create a new channel.
    fn register_topic(
        &self,
        topic: &str,
        descriptor: &'static MessageDescriptor,
    ) -> Result<(
        Channel::Sender<Vec<u8>>,
        Channel::Receiver<Vec<u8>>,
        Arc<TopicRegistration>,
    )> {
        let mut topics = self.topics.write().unwrap();
        if let Some(entry) = topics.get(topic) {
            // A weak registration can expire just before its Drop acquires this lock.
            // Expired entries may be replaced with a different type.
            if entry.registration.strong_count() > 0
                && !entry.descriptor.is_compatible_with(descriptor)
            {
                return Err(Error::ServerError(format!(
                    "Topic {topic} already registered with schema {}, incompatible with {}",
                    entry.type_name, descriptor.ros_type_name
                )));
            }
            if let Some(registration) = entry.registration.upgrade() {
                return Ok((
                    entry.sender.clone(),
                    entry.receiver.resubscribe(),
                    registration,
                ));
            }
        }
        let (sender, receiver) = Channel::channel(10);
        let registration = Arc::new(TopicRegistration {
            store: Arc::downgrade(&self.topics),
            topic: topic.to_owned(),
        });
        let handles = (sender.clone(), receiver.resubscribe(), registration.clone());
        topics.insert(
            topic.to_owned(),
            TopicEntry {
                sender,
                receiver,
                type_name: descriptor.ros_type_name.to_owned(),
                descriptor,
                registration: Arc::downgrade(&registration),
            },
        );
        Ok(handles)
    }
}

impl TopicProvider for MockRos {
    type Publisher<T: RosMessageType> = MockPublisher<T>;
    type Subscriber<T: RosMessageType> = MockSubscriber<T>;

    async fn advertise<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Publisher<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let (sender, _, registration) =
            self.register_topic(topic.as_ref(), &MsgType::DESCRIPTION)?;
        Ok(MockPublisher {
            sender,
            _registration: registration,
            _marker: Default::default(),
        })
    }

    async fn subscribe<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Subscriber<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let (_, receiver, registration) =
            self.register_topic(topic.as_ref(), &MsgType::DESCRIPTION)?;
        Ok(MockSubscriber {
            receiver,
            _registration: registration,
            _marker: Default::default(),
        })
    }
}

impl DynamicTopicProvider for MockRos {
    type DynamicPublisher = MockDynamicPublisher;
    type DynamicSubscriber = MockDynamicSubscriber;

    async fn dynamic_advertise(
        &self,
        topic: impl ToGlobalTopicName,
        descriptor: &'static MessageDescriptor,
    ) -> Result<Self::DynamicPublisher> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let (sender, _, registration) = self.register_topic(topic.as_ref(), descriptor)?;
        Ok(MockDynamicPublisher {
            sender,
            descriptor,
            _registration: registration,
        })
    }

    async fn dynamic_subscribe(
        &self,
        topic: impl ToGlobalTopicName,
        descriptor: &'static MessageDescriptor,
    ) -> Result<Self::DynamicSubscriber> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let (_, receiver, registration) = self.register_topic(topic.as_ref(), descriptor)?;
        Ok(MockDynamicSubscriber {
            receiver,
            descriptor,
            _registration: registration,
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
            let services = services.read().unwrap();
            services
                .get(&self.topic)
                .filter(|entry| entry.registration.strong_count() > 0)
                .map(|entry| entry.callback.clone())
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

/// Runtime-typed service client.
pub struct MockDynamicServiceClient {
    handle: std::sync::Weak<ServiceStore>,
    topic: String,
    descriptor: &'static ServiceDescriptor,
}

impl DynamicService for MockDynamicServiceClient {
    fn descriptor(&self) -> &'static ServiceDescriptor {
        self.descriptor
    }

    async fn call(&self, request: &DynamicMessage) -> Result<DynamicMessage> {
        if !self
            .descriptor
            .request
            .is_compatible_with(request.descriptor())
        {
            return Err(Error::SerializationError(format!(
                "incompatible request schema: service {} expects {}, but message declares {}",
                self.descriptor.ros_service_name,
                self.descriptor.request.ros_type_name,
                request.descriptor().ros_type_name
            )));
        }
        let services = self.handle.upgrade().ok_or_else(|| {
            Error::ServerError("No connection to MockRos backend? Has it been dropped?".to_owned())
        })?;
        tokio::task::yield_now().await;
        let callback = services
            .read()
            .unwrap()
            .get(&self.topic)
            .filter(|entry| entry.registration.strong_count() > 0)
            .map(|entry| entry.callback.clone())
            .ok_or_else(|| {
                Error::ServerError(format!("No service server found for topic: {}", self.topic))
            })?;

        let mut bytes = Vec::new();
        let options = bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .allow_trailing_bytes();
        let mut serializer = bincode::Serializer::new(&mut bytes, options);
        request
            .serialize_with(&mut serializer)
            .map_err(|error| Error::SerializationError(error.to_string()))?;
        let response = tokio::task::spawn_blocking(move || (callback)(bytes))
            .await
            .map_err(|_| Error::Disconnected)?
            .map_err(|error| Error::SerializationError(error.to_string()))?;
        let options = bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .allow_trailing_bytes();
        let mut deserializer = bincode::Deserializer::from_slice(&response, options);
        self.descriptor
            .response
            .deserialize_with(&mut deserializer)
            .map_err(|error| Error::SerializationError(error.to_string()))
    }
}

impl DynamicServiceProvider for MockRos {
    type DynamicServiceClient = MockDynamicServiceClient;

    async fn dynamic_call_service(
        &self,
        service: impl ToGlobalTopicName,
        descriptor: &'static ServiceDescriptor,
        request: DynamicMessage,
    ) -> Result<DynamicMessage> {
        let client = self.dynamic_service_client(service, descriptor).await?;
        client.call(&request).await
    }

    async fn dynamic_service_client(
        &self,
        service: impl ToGlobalTopicName,
        descriptor: &'static ServiceDescriptor,
    ) -> Result<Self::DynamicServiceClient> {
        let service: GlobalTopicName = service.to_global_name()?;
        Ok(MockDynamicServiceClient {
            handle: Arc::downgrade(&self.services),
            topic: String::from(service),
            descriptor,
        })
    }
}

impl ServiceProvider for MockRos {
    type ServiceClient<T: RosServiceType> = MockServiceClient<T>;
    type ServiceServer = MockServiceServer;

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
        let service_name = service.as_ref().to_string();
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
        let mut services = self.services.write().unwrap();
        if services
            .get(&service_name)
            .is_some_and(|entry| entry.registration.strong_count() > 0)
        {
            return Err(Error::ServerError(format!(
                "Service {service_name} is already advertised; drop its server handles before replacing it"
            )));
        }
        let registration = Arc::new(ServiceRegistration {
            store: Arc::downgrade(&self.services),
            service: service_name.clone(),
        });
        let previous = services.insert(
            service_name,
            ServiceEntry {
                callback: erased_closure,
                type_name: SrvType::ROS_SERVICE_NAME.to_string(),
                registration: Arc::downgrade(&registration),
            },
        );
        drop(services);
        drop(previous);
        Ok(MockServiceServer {
            _registration: registration,
        })
    }
}

impl GraphProvider for MockRos {
    async fn list_topics(&self) -> Result<Vec<TopicInfo>> {
        let topics = self.topics.read().unwrap();
        Ok(topics
            .iter()
            .filter(|(_, entry)| entry.registration.strong_count() > 0)
            .map(|(name, entry)| TopicInfo {
                name: name.clone(),
                type_name: entry.type_name.clone(),
            })
            .collect())
    }

    async fn list_services(&self) -> Result<Vec<ServiceInfo>> {
        let services = self.services.read().unwrap();
        Ok(services
            .iter()
            .filter(|(_, entry)| entry.registration.strong_count() > 0)
            .map(|(name, entry)| ServiceInfo {
                name: name.clone(),
                type_name: entry.type_name.clone(),
            })
            .collect())
    }
}

/// Owns a mock service advertisement. Keep this handle alive while serving requests.
/// Clones share ownership; dropping the last handle immediately unregisters the service.
#[derive(Clone)]
#[must_use = "dropping the service server handle unregisters the service"]
pub struct MockServiceServer {
    _registration: Arc<ServiceRegistration>,
}

/// The publisher type returned by calling [MockRos::advertise].
pub struct MockPublisher<T: RosMessageType> {
    sender: Channel::Sender<Vec<u8>>,
    _registration: Arc<TopicRegistration>,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> Clone for MockPublisher<T> {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
            _registration: self._registration.clone(),
            _marker: Default::default(),
        }
    }
}

impl<T: RosMessageType> Publish<T> for MockPublisher<T> {
    async fn publish(&self, data: &T) -> roslibrust_common::Result<()> {
        let data =
            bincode::serialize(data).map_err(|e| Error::SerializationError(e.to_string()))?;
        self.sender.send(data).map_err(|_e| Error::Disconnected)?;
        debug!("Sent data on topic {}", T::DESCRIPTION.ros_type_name);
        Ok(())
    }
}

/// The subscriber type returned by calling [MockRos::subscribe].
pub struct MockSubscriber<T: RosMessageType> {
    receiver: Channel::Receiver<Vec<u8>>,
    _registration: Arc<TopicRegistration>,
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
        debug!("Received data on topic {}", T::DESCRIPTION.ros_type_name);
        Ok(msg)
    }
}

/// Dynamic publisher returned by [`DynamicTopicProvider::dynamic_advertise`] on [`MockRos`].
#[derive(Clone)]
pub struct MockDynamicPublisher {
    sender: Channel::Sender<Vec<u8>>,
    descriptor: &'static MessageDescriptor,
    _registration: Arc<TopicRegistration>,
}

impl DynamicPublish for MockDynamicPublisher {
    fn descriptor(&self) -> &'static MessageDescriptor {
        self.descriptor
    }

    async fn publish(&self, data: &DynamicMessage) -> Result<()> {
        if !self.descriptor.is_compatible_with(data.descriptor()) {
            return Err(Error::SerializationError(format!(
                "incompatible message schema: publisher expects {}, but message declares {}",
                self.descriptor.ros_type_name,
                data.descriptor().ros_type_name
            )));
        }

        let mut bytes = Vec::new();
        let options = bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .allow_trailing_bytes();
        let mut serializer = bincode::Serializer::new(&mut bytes, options);
        data.serialize_with(&mut serializer)
            .map_err(|error| Error::SerializationError(error.to_string()))?;
        self.sender.send(bytes).map_err(|_| Error::Disconnected)?;
        Ok(())
    }
}

/// Dynamic subscriber returned by [`DynamicTopicProvider::dynamic_subscribe`] on [`MockRos`].
pub struct MockDynamicSubscriber {
    receiver: Channel::Receiver<Vec<u8>>,
    descriptor: &'static MessageDescriptor,
    _registration: Arc<TopicRegistration>,
}

impl DynamicSubscribe for MockDynamicSubscriber {
    async fn next(&mut self) -> Result<DynamicMessage> {
        let bytes = self
            .receiver
            .recv()
            .await
            .map_err(|_| Error::Disconnected)?;
        let options = bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .allow_trailing_bytes();
        let mut deserializer = bincode::Deserializer::from_slice(&bytes, options);
        self.descriptor
            .deserialize_with(&mut deserializer)
            .map_err(|error| Error::SerializationError(error.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roslibrust_test::ros1::std_msgs;
    use roslibrust_test::ros1::std_srvs;

    fn respond(
        request: std_srvs::SetBoolRequest,
    ) -> std::result::Result<std_srvs::SetBoolResponse, ServiceError> {
        Ok(std_srvs::SetBoolResponse {
            success: request.data,
            message: String::new(),
        })
    }

    #[tokio::test]
    async fn service_lifetime_follows_last_handle_and_clients_can_reconnect() {
        let ros = MockRos::new();
        let client = ros
            .service_client::<std_srvs::SetBool>("/lifetime")
            .await
            .unwrap();
        let dynamic_client = ros
            .dynamic_service_client("/lifetime", &std_srvs::SetBool::DESCRIPTION)
            .await
            .unwrap();
        let request = std_srvs::SetBoolRequest { data: true };
        let dynamic_request = std_srvs::SetBoolRequest::DESCRIPTION
            .message_from(&request)
            .unwrap();
        assert!(client.call(&request).await.is_err());
        {
            let server = ros
                .advertise_service::<std_srvs::SetBool, _>("/lifetime", respond)
                .await
                .unwrap();
            let clone = server.clone();
            assert_eq!(ros.list_services().await.unwrap().len(), 1);
            assert!(client.call(&request).await.unwrap().success);
            assert!(dynamic_client.call(&dynamic_request).await.is_ok());
            assert!(ros
                .advertise_service::<std_srvs::SetBool, _>("/lifetime", respond)
                .await
                .is_err());
            assert!(ros
                .advertise_service::<std_srvs::Trigger, _>("/lifetime", |_| Ok(
                    std_srvs::TriggerResponse::default()
                ))
                .await
                .is_err());
            drop(server);
            assert!(client.call(&request).await.unwrap().success);
            assert_eq!(ros.list_services().await.unwrap().len(), 1);
            drop(clone);
        }
        assert!(ros.list_services().await.unwrap().is_empty());
        assert!(client.call(&request).await.is_err());
        assert!(dynamic_client.call(&dynamic_request).await.is_err());
        {
            let _server = ros
                .advertise_service::<std_srvs::SetBool, _>("/lifetime", respond)
                .await
                .unwrap();
            assert!(client.call(&request).await.unwrap().success);
        }
        assert!(ros.list_services().await.unwrap().is_empty());
        let _different_type = ros
            .advertise_service::<std_srvs::Trigger, _>("/lifetime", |_| {
                Ok(std_srvs::TriggerResponse::default())
            })
            .await
            .unwrap();
        assert_eq!(
            ros.list_services().await.unwrap()[0].type_name,
            "std_srvs/Trigger"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn concurrent_service_advertisements_have_one_owner() {
        let ros = MockRos::new();
        let barrier = Arc::new(tokio::sync::Barrier::new(16));
        let mut tasks = tokio::task::JoinSet::new();
        for _ in 0..16 {
            let ros = ros.clone();
            let barrier = barrier.clone();
            tasks.spawn(async move {
                barrier.wait().await;
                ros.advertise_service::<std_srvs::SetBool, _>("/concurrent", respond)
                    .await
            });
        }
        let mut servers = Vec::new();
        while let Some(result) = tasks.join_next().await {
            if let Ok(server) = result.unwrap() {
                servers.push(server);
            }
        }
        assert_eq!(servers.len(), 1);
        assert_eq!(ros.list_services().await.unwrap().len(), 1);
        drop(servers);
        assert!(ros.list_services().await.unwrap().is_empty());
    }

    #[tokio::test]
    async fn pending_old_service_cleanup_does_not_remove_its_replacement() {
        let ros = MockRos::new();
        let server = ros
            .advertise_service::<std_srvs::SetBool, _>("/replace", respond)
            .await
            .unwrap();
        let mut services = ros.services.write().unwrap();
        let old_registration = services["/replace"].registration.clone();
        // Block the old handle's Drop after its Arc count reaches zero, exactly the window
        // in which advertise_service is allowed to replace an expired registration.
        let dropper = std::thread::spawn(move || drop(server));
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
        while old_registration.strong_count() != 0 {
            assert!(
                std::time::Instant::now() < deadline,
                "old handle did not start dropping"
            );
            std::thread::yield_now();
        }
        let registration = Arc::new(ServiceRegistration {
            store: Arc::downgrade(&ros.services),
            service: "/replace".to_owned(),
        });
        services.get_mut("/replace").unwrap().registration = Arc::downgrade(&registration);
        let replacement = MockServiceServer {
            _registration: registration,
        };
        drop(services);
        dropper.join().unwrap();
        assert_eq!(ros.list_services().await.unwrap().len(), 1);
        assert!(
            ros.call_service::<std_srvs::SetBool>(
                "/replace",
                std_srvs::SetBoolRequest { data: true }
            )
            .await
            .unwrap()
            .success
        );
        drop(replacement);
        assert!(ros.list_services().await.unwrap().is_empty());
    }

    #[tokio::test]
    async fn topic_lifetime_is_shared_across_typed_dynamic_and_cloned_handles() {
        let ros = MockRos::new();
        let descriptor = &std_msgs::Int16::DESCRIPTION;
        let publisher = ros.advertise::<std_msgs::Int16>("/lifetime").await.unwrap();
        let clone = publisher.clone();
        let dynamic_publisher = ros
            .dynamic_advertise("/lifetime", descriptor)
            .await
            .unwrap();
        let dynamic_clone = dynamic_publisher.clone();
        let subscriber = ros.subscribe::<std_msgs::Int16>("/lifetime").await.unwrap();
        let mut dynamic_subscriber = ros
            .dynamic_subscribe("/lifetime", descriptor)
            .await
            .unwrap();
        drop(publisher);
        clone.publish(&std_msgs::Int16 { data: 17 }).await.unwrap();
        assert_eq!(
            dynamic_subscriber.next().await.unwrap().get("data"),
            Some(&DynamicValue::I16(17))
        );
        drop(clone);
        drop(dynamic_publisher);
        drop(dynamic_clone);
        assert_eq!(ros.list_topics().await.unwrap().len(), 1);
        assert!(ros
            .advertise::<std_msgs::String>("/lifetime")
            .await
            .is_err());
        drop(subscriber);
        assert_eq!(ros.list_topics().await.unwrap().len(), 1);
        drop(dynamic_subscriber);
        assert!(ros.list_topics().await.unwrap().is_empty());
        {
            let _subscriber = ros
                .dynamic_subscribe("/lifetime", &std_msgs::String::DESCRIPTION)
                .await
                .unwrap();
            let _publisher = ros
                .advertise::<std_msgs::String>("/lifetime")
                .await
                .unwrap();
            assert_eq!(
                ros.list_topics().await.unwrap()[0].type_name,
                "std_msgs/String"
            );
        }
        assert!(ros.list_topics().await.unwrap().is_empty());
        {
            let _publisher = ros
                .dynamic_advertise("/lifetime", descriptor)
                .await
                .unwrap();
        }
        assert!(ros.list_topics().await.unwrap().is_empty());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn concurrent_topic_creation_shares_one_channel() {
        enum Handle {
            Publisher(MockPublisher<std_msgs::Int16>),
            DynamicPublisher(MockDynamicPublisher),
            Subscriber(MockSubscriber<std_msgs::Int16>),
            DynamicSubscriber(MockDynamicSubscriber),
        }
        let ros = MockRos::new();
        let barrier = Arc::new(tokio::sync::Barrier::new(32));
        let mut tasks = tokio::task::JoinSet::new();
        for i in 0..32 {
            let ros = ros.clone();
            let barrier = barrier.clone();
            tasks.spawn(async move {
                barrier.wait().await;
                match i % 4 {
                    0 => Handle::Publisher(
                        ros.advertise::<std_msgs::Int16>("/concurrent")
                            .await
                            .unwrap(),
                    ),
                    1 => Handle::DynamicPublisher(
                        ros.dynamic_advertise("/concurrent", &std_msgs::Int16::DESCRIPTION)
                            .await
                            .unwrap(),
                    ),
                    2 => Handle::Subscriber(
                        ros.subscribe::<std_msgs::Int16>("/concurrent")
                            .await
                            .unwrap(),
                    ),
                    _ => Handle::DynamicSubscriber(
                        ros.dynamic_subscribe("/concurrent", &std_msgs::Int16::DESCRIPTION)
                            .await
                            .unwrap(),
                    ),
                }
            });
        }
        let mut publishers = Vec::new();
        let mut subscribers = Vec::new();
        while let Some(handle) = tasks.join_next().await {
            match handle.unwrap() {
                handle @ (Handle::Publisher(_) | Handle::DynamicPublisher(_)) => {
                    publishers.push(handle)
                }
                handle => subscribers.push(handle),
            }
        }
        assert_eq!(ros.list_topics().await.unwrap().len(), 1);
        for (i, publisher) in publishers.iter().enumerate() {
            let data = i as i16;
            let message = std_msgs::Int16 { data };
            match publisher {
                Handle::Publisher(publisher) => publisher.publish(&message).await.unwrap(),
                Handle::DynamicPublisher(publisher) => publisher
                    .publish(&std_msgs::Int16::DESCRIPTION.message_from(&message).unwrap())
                    .await
                    .unwrap(),
                _ => unreachable!(),
            }
            for subscriber in &mut subscribers {
                tokio::time::timeout(std::time::Duration::from_secs(1), async {
                    match subscriber {
                        Handle::Subscriber(subscriber) => {
                            assert_eq!(subscriber.next().await.unwrap().data, data)
                        }
                        Handle::DynamicSubscriber(subscriber) => assert_eq!(
                            subscriber.next().await.unwrap().get("data"),
                            Some(&DynamicValue::I16(data))
                        ),
                        _ => unreachable!(),
                    }
                })
                .await
                .expect("all subscribers must share every publisher's channel");
            }
        }
        drop(publishers);
        assert_eq!(ros.list_topics().await.unwrap().len(), 1);
        drop(subscribers);
        assert!(ros.list_topics().await.unwrap().is_empty());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn concurrent_conflicting_topic_types_cannot_both_register() {
        for _ in 0..32 {
            let ros = MockRos::new();
            let barrier = Arc::new(tokio::sync::Barrier::new(2));
            let typed = {
                let ros = ros.clone();
                let barrier = barrier.clone();
                tokio::spawn(async move {
                    barrier.wait().await;
                    ros.advertise::<std_msgs::Int16>("/conflict").await
                })
            };
            let dynamic = tokio::spawn({
                let ros = ros.clone();
                async move {
                    barrier.wait().await;
                    ros.dynamic_subscribe("/conflict", &std_msgs::String::DESCRIPTION)
                        .await
                }
            });
            let typed = typed.await.unwrap();
            let dynamic = dynamic.await.unwrap();
            assert_ne!(typed.is_ok(), dynamic.is_ok());
            assert_eq!(ros.list_topics().await.unwrap().len(), 1);
            drop(typed);
            drop(dynamic);
            assert!(ros.list_topics().await.unwrap().is_empty());
        }
    }

    #[test]
    fn handles_are_send_sync_and_cleanup_without_a_runtime() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<MockRos>();
        assert_send_sync::<MockPublisher<std_msgs::Int16>>();
        assert_send_sync::<MockSubscriber<std_msgs::Int16>>();
        assert_send_sync::<MockDynamicPublisher>();
        assert_send_sync::<MockDynamicSubscriber>();
        assert_send_sync::<MockServiceServer>();
        assert_send_sync::<MockServiceClient<std_srvs::SetBool>>();
        assert_send_sync::<MockDynamicServiceClient>();
        let ros = MockRos::new();
        let runtime = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        let (publisher, server) = runtime.block_on(async {
            (
                ros.advertise::<std_msgs::Int16>("/outside_runtime")
                    .await
                    .unwrap(),
                ros.advertise_service::<std_srvs::SetBool, _>("/outside_runtime", respond)
                    .await
                    .unwrap(),
            )
        });
        drop(runtime);
        drop(publisher);
        drop(server);
        assert!(ros.topics.read().unwrap().is_empty());
        assert!(ros.services.read().unwrap().is_empty());
    }

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
    async fn dynamic_topics_interoperate_with_typed_topics() {
        let mock_ros = MockRos::new();
        let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
            .get("std_msgs/Int16")
            .unwrap();

        let typed_publisher = mock_ros
            .advertise::<std_msgs::Int16>("/typed_to_dynamic")
            .await
            .unwrap();
        let mut dynamic_subscriber = mock_ros
            .dynamic_subscribe("/typed_to_dynamic", descriptor)
            .await
            .unwrap();
        typed_publisher
            .publish(&std_msgs::Int16 { data: 42 })
            .await
            .unwrap();
        assert_eq!(
            dynamic_subscriber.next().await.unwrap().value(),
            &DynamicValue::Message(vec![DynamicField {
                name: "data".to_owned(),
                value: DynamicValue::I16(42),
            }])
        );

        let dynamic_publisher = mock_ros
            .dynamic_advertise("/dynamic_to_typed", descriptor)
            .await
            .unwrap();
        let mut typed_subscriber = mock_ros
            .subscribe::<std_msgs::Int16>("/dynamic_to_typed")
            .await
            .unwrap();
        let message = descriptor
            .message(DynamicValue::Message(vec![DynamicField {
                name: "data".to_owned(),
                value: DynamicValue::I16(-7),
            }]))
            .unwrap();
        dynamic_publisher.publish(&message).await.unwrap();
        assert_eq!(
            typed_subscriber.next().await.unwrap(),
            std_msgs::Int16 { data: -7 }
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn dynamic_topics_reject_descriptor_mismatches() {
        let mock_ros = MockRos::new();
        let int16 = roslibrust_test::ros1::MESSAGE_REGISTRY
            .get("std_msgs/Int16")
            .unwrap();
        let string = roslibrust_test::ros1::MESSAGE_REGISTRY
            .get("std_msgs/String")
            .unwrap();

        let publisher = mock_ros.dynamic_advertise("/dynamic", int16).await.unwrap();
        assert!(mock_ros
            .dynamic_subscribe("/dynamic", string)
            .await
            .is_err());

        let wrong_message = string
            .message(DynamicValue::Message(vec![DynamicField {
                name: "data".to_owned(),
                value: DynamicValue::String("wrong type".to_owned()),
            }]))
            .unwrap();
        assert!(publisher.publish(&wrong_message).await.is_err());
    }

    #[tokio::test]
    async fn same_name_descriptors_require_matching_schema_metadata() {
        let mock = MockRos::new();
        let original = &std_msgs::String::DESCRIPTION;
        let independent = Box::leak(Box::new(MessageDescriptor::new::<std_msgs::String>(
            original.ros_type_name,
            original.md5sum,
            original.definition,
            original.ros2_type_name,
            original.ros2_hash,
        )));
        let changed = Box::leak(Box::new(MessageDescriptor::new::<std_msgs::String>(
            original.ros_type_name,
            original.md5sum,
            "uint8 data",
            original.ros2_type_name,
            original.ros2_hash,
        )));
        let publisher = mock.dynamic_advertise("/schema", original).await.unwrap();
        let mut subscriber = mock
            .dynamic_subscribe("/schema", independent)
            .await
            .unwrap();
        assert!(mock.dynamic_advertise("/schema", changed).await.is_err());
        assert!(mock.dynamic_subscribe("/schema", changed).await.is_err());
        let message = independent
            .message_from(&std_msgs::String {
                data: "hello".into(),
            })
            .unwrap();
        publisher.publish(&message).await.unwrap();
        assert_eq!(
            subscriber
                .next()
                .await
                .unwrap()
                .get("data")
                .unwrap()
                .as_str(),
            Some("hello")
        );
        let invalid = changed
            .message_from(&std_msgs::String {
                data: "hello".into(),
            })
            .unwrap();
        assert!(publisher
            .publish(&invalid)
            .await
            .unwrap_err()
            .to_string()
            .contains("incompatible message schema"));

        let service = &std_srvs::SetBool::DESCRIPTION;
        let changed_request = Box::leak(Box::new(
            MessageDescriptor::new::<std_srvs::SetBoolRequest>(
                service.request.ros_type_name,
                service.request.md5sum,
                "uint8 data",
                service.request.ros2_type_name,
                service.request.ros2_hash,
            ),
        ));
        let client = mock
            .dynamic_service_client("/service_schema", service)
            .await
            .unwrap();
        let request = changed_request
            .message_from(&std_srvs::SetBoolRequest { data: true })
            .unwrap();
        assert!(client
            .call(&request)
            .await
            .unwrap_err()
            .to_string()
            .contains("incompatible request schema"));
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

        let _server = mock_topics
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
    async fn test_mock_graph_provider() {
        let mock_ros = MockRos::new();

        let _publisher = mock_ros
            .advertise::<std_msgs::String>("/test_topic")
            .await
            .unwrap();
        let _server = mock_ros
            .advertise_service::<std_srvs::SetBool, _>("/test_service", |request| {
                Ok(std_srvs::SetBoolResponse {
                    success: request.data,
                    message: String::new(),
                })
            })
            .await
            .unwrap();

        assert_eq!(
            mock_ros.list_topics().await.unwrap(),
            vec![TopicInfo {
                name: "/test_topic".to_string(),
                type_name: "std_msgs/String".to_string(),
            }]
        );
        assert_eq!(
            mock_ros.list_services().await.unwrap(),
            vec![ServiceInfo {
                name: "/test_service".to_string(),
                type_name: "std_srvs/SetBool".to_string(),
            }]
        );
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
        let _server = mock_ros
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
            let _server = mock_ros
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
