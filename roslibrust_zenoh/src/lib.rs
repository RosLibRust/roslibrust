//! A crate for interfacing to ROS1 via the [zenoh-ros1-plugin / zenoh-ros1-bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros1).
//!
//! It is not recommended to depend on this crate directly, but instead access it via [roslibrust](https://docs.rs/roslibrust/latest/roslibrust/) with the `zenoh` feature enabled.
use roslibrust_common::topic_name::{GlobalTopicName, ToGlobalTopicName};
use roslibrust_common::*;

use log::*;
use std::{
    collections::{BTreeMap, HashMap},
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};
use tokio::sync::RwLock;
use zenoh::bytes::ZBytes;

const DISCOVERY_NAMESPACE: &str = "*";
const BRIDGE_NAMESPACE: &str = "*";
const DISCOVERY_KEYEXPR: &str = "ros1_discovery_info/*/*/*/*/*/**";
const DISCOVERY_BEACON_PERIOD: Duration = Duration::from_secs(1);
const DISCOVERY_LOST_AFTER: Duration = Duration::from_secs(3);

/// A wrapper around a normal zenoh session that adds roslibrust specific functionality.
/// Should be created via [ZenohClient::new], and then used via the [TopicProvider] and [ServiceProvider] traits.
#[derive(Clone)]
pub struct ZenohClient {
    _discovery_monitor: Arc<DiscoveryMonitor>,
    session: zenoh::Session,
    graph: Arc<RwLock<BTreeMap<String, DiscoveryFact>>>,
}

impl ZenohClient {
    /// Creates a new client wrapped around a Zenoh session
    pub fn new(session: zenoh::Session) -> Self {
        let graph = Arc::new(RwLock::new(BTreeMap::new()));
        let discovery_monitor = Arc::new(spawn_discovery_monitor(session.clone(), graph.clone()));
        Self {
            _discovery_monitor: discovery_monitor,
            session,
            graph,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DiscoveryClass {
    Publisher,
    Subscriber,
    Service,
    Client,
}

impl DiscoveryClass {
    fn as_str(self) -> &'static str {
        match self {
            Self::Publisher => "pub",
            Self::Subscriber => "sub",
            Self::Service => "srv",
            Self::Client => "cl",
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value {
            "pub" => Some(Self::Publisher),
            "sub" => Some(Self::Subscriber),
            "srv" => Some(Self::Service),
            "cl" => Some(Self::Client),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct DiscoveryFact {
    class: DiscoveryClass,
    name: String,
    type_name: String,
    md5sum: String,
}

struct DiscoveryDeclaration {
    shutdown: Option<tokio::sync::oneshot::Sender<()>>,
}

struct DiscoveryMonitor {
    shutdown: Option<tokio::sync::oneshot::Sender<()>>,
}

impl Drop for DiscoveryMonitor {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
    }
}

impl Drop for DiscoveryDeclaration {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
    }
}

impl DiscoveryDeclaration {
    async fn new(
        session: zenoh::Session,
        class: DiscoveryClass,
        name: &str,
        type_name: &str,
        md5sum: &str,
    ) -> Result<Self> {
        let key = make_discovery_key(class, name, type_name, md5sum);
        let publisher = session.declare_publisher(key).await.map_err(|e| {
            Error::Unexpected(anyhow::anyhow!(
                "Failed to declare discovery publisher: {e:?}"
            ))
        })?;
        let (shutdown_tx, mut shutdown_rx) = tokio::sync::oneshot::channel();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(DISCOVERY_BEACON_PERIOD);
            loop {
                if let Err(e) = publisher.put(ZBytes::default()).await {
                    error!("Failed to publish discovery info: {e:?}");
                }
                tokio::select! {
                    _ = interval.tick() => {}
                    _ = &mut shutdown_rx => break,
                }
            }
        });
        Ok(Self {
            shutdown: Some(shutdown_tx),
        })
    }
}

fn spawn_discovery_monitor(
    session: zenoh::Session,
    graph: Arc<RwLock<BTreeMap<String, DiscoveryFact>>>,
) -> DiscoveryMonitor {
    let Ok(handle) = tokio::runtime::Handle::try_current() else {
        error!("Failed to start Zenoh discovery monitor: no active Tokio runtime");
        return DiscoveryMonitor { shutdown: None };
    };
    let (shutdown_tx, mut shutdown_rx) = tokio::sync::oneshot::channel();

    handle.spawn(async move {
        let subscriber = tokio::select! {
            subscriber = session.declare_subscriber(DISCOVERY_KEYEXPR) => {
                match subscriber {
                    Ok(subscriber) => subscriber,
                    Err(e) => {
                        error!("Failed to subscribe to {DISCOVERY_KEYEXPR}: {e:?}");
                        return;
                    }
                }
            }
            _ = &mut shutdown_rx => return,
        };
        let mut active_facts: HashMap<String, Instant> = HashMap::new();
        let mut interval = tokio::time::interval(DISCOVERY_BEACON_PERIOD);
        loop {
            tokio::select! {
                sample = subscriber.recv_async() => {
                    let sample = match sample {
                        Ok(sample) => sample,
                        Err(e) => {
                            error!("Failed to receive discovery info: {e:?}");
                            break;
                        }
                    };
                    let key = sample.key_expr().as_str().to_string();
                    let Some(fact) = parse_discovery_key(&key) else {
                        continue;
                    };
                    active_facts.insert(key.clone(), Instant::now());
                    graph.write().await.insert(key, fact);
                }
                _ = interval.tick() => {
                    let now = Instant::now();
                    let lost: Vec<_> = active_facts
                        .iter()
                        .filter_map(|(key, last_seen)| {
                            now.duration_since(*last_seen)
                                .gt(&DISCOVERY_LOST_AFTER)
                                .then(|| key.clone())
                        })
                        .collect();
                    if !lost.is_empty() {
                        let mut graph = graph.write().await;
                        for key in lost {
                            active_facts.remove(&key);
                            graph.remove(&key);
                        }
                    }
                }
                _ = &mut shutdown_rx => break,
            }
        }
    });
    DiscoveryMonitor {
        shutdown: Some(shutdown_tx),
    }
}

/// The publisher type returned by [TopicProvider::advertise] on [ZenohClient]
/// This type is self de-registering, and dropping the publisher will automatically un-advertise the topic.
pub struct ZenohPublisher<T> {
    publisher: zenoh::pubsub::Publisher<'static>,
    _discovery: DiscoveryDeclaration,
    // Used to track buffer capacity size to minimize allocations for fixed-size streams.
    capacity_hint: AtomicUsize,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> ZenohPublisher<T> {
    /// Checks if there are any connected subscribers.
    /// This can be used to skip expensive message construction when no one is listening.
    pub async fn has_connected_clients(&self) -> bool {
        match self.publisher.matching_status().await {
            Ok(status) => status.matching(),
            Err(e) => {
                // If we can't determine the status, assume there might be subscribers
                // to avoid dropping messages
                warn!("Failed to get matching status: {e:?}, assuming subscribers exist");
                true
            }
        }
    }
}

impl<T: RosMessageType> Publish<T> for ZenohPublisher<T> {
    async fn publish(&self, data: &T) -> Result<()> {
        // Skip serialization if there are no connected clients
        if !self.has_connected_clients().await {
            debug!("Skipping publish - no connected clients");
            return Ok(());
        }

        let size_hint = self.capacity_hint.load(Ordering::Relaxed);
        let mut bytes = Vec::with_capacity(size_hint);
        roslibrust_serde_rosmsg::to_writer_skip_length(&mut bytes, data).map_err(|e| {
            Error::SerializationError(format!("Failed to serialize message: {e:?}"))
        })?;

        if bytes.len() > size_hint {
            self.capacity_hint.store(bytes.len(), Ordering::Relaxed);
        }

        match self.publisher.put(bytes).await {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::Unexpected(anyhow::anyhow!(
                "Failed to publish message to zenoh: {e:?}"
            ))),
        }
    }
}

// Using type alias here, I have no idea why zenoh has this type so deep
type ZenohSubInner =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;

/// The subscriber type returned by [TopicProvider::subscribe] on [ZenohClient].
/// This type is self de-registering, and dropping the subscriber will automatically unsubscribe from the topic.
/// This type is generic on the message type that will be received.
/// It is typically used with types generated by roslibrust's codegen.
pub struct ZenohSubscriber<T> {
    subscriber: ZenohSubInner,
    _discovery: DiscoveryDeclaration,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> Subscribe<T> for ZenohSubscriber<T> {
    async fn next(&mut self) -> Result<T> {
        let next = self.subscriber.recv_async().await;

        let sample = match next {
            Ok(sample) => sample,
            Err(e) => {
                // TODO errors still suck with this API
                return Err(Error::Unexpected(anyhow::anyhow!(
                    "Failed to receive next sample: {e:?}"
                )));
            }
        };

        let msg = deserialize_payload(sample.payload(), "sample")?;
        Ok(msg)
    }
}

fn deserialize_payload<T: RosMessageType>(payload: &ZBytes, context: &str) -> Result<T> {
    // Note: Zenoh decided to not make the 4 byte length header part of the payload.
    let mut reader = payload.reader();
    roslibrust_serde_rosmsg::from_reader_known_length(&mut reader, payload.len() as u32)
        .map_err(|e| Error::SerializationError(format!("Failed to deserialize {context}: {e:?}")))
}

impl TopicProvider for ZenohClient {
    type Publisher<T: RosMessageType> = ZenohPublisher<T>;

    type Subscriber<T: RosMessageType> = ZenohSubscriber<T>;

    async fn advertise<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Publisher<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let mangled_topic = mangle_topic(topic.as_ref(), MsgType::ROS_TYPE_NAME, MsgType::MD5SUM);
        let publisher = match self.session.declare_publisher(mangled_topic).await {
            Ok(publisher) => publisher,
            Err(e) => {
                // TODO errors still suck with this API...
                return Err(Error::Unexpected(anyhow::anyhow!(
                    "Failed to declare publisher: {e:?}"
                )));
            }
        };
        let discovery = DiscoveryDeclaration::new(
            self.session.clone(),
            DiscoveryClass::Publisher,
            topic.as_ref(),
            MsgType::ROS_TYPE_NAME,
            MsgType::MD5SUM,
        )
        .await?;

        Ok(ZenohPublisher {
            publisher,
            _discovery: discovery,
            capacity_hint: 1024.into(),
            _marker: std::marker::PhantomData,
        })
    }

    async fn subscribe<MsgType: RosMessageType>(
        &self,
        topic: impl ToGlobalTopicName,
    ) -> Result<Self::Subscriber<MsgType>> {
        let topic: GlobalTopicName = topic.to_global_name()?;
        let mangled_topic = mangle_topic(topic.as_ref(), MsgType::ROS_TYPE_NAME, MsgType::MD5SUM);
        let sub = match self.session.declare_subscriber(mangled_topic).await {
            Ok(sub) => sub,
            Err(e) => {
                // TODO errors still suck with this API...
                return Err(Error::Unexpected(anyhow::anyhow!(
                    "Failed to declare subscriber: {e:?}"
                )));
            }
        };
        let discovery = DiscoveryDeclaration::new(
            self.session.clone(),
            DiscoveryClass::Subscriber,
            topic.as_ref(),
            MsgType::ROS_TYPE_NAME,
            MsgType::MD5SUM,
        )
        .await?;
        Ok(ZenohSubscriber {
            subscriber: sub,
            _discovery: discovery,
            _marker: std::marker::PhantomData,
        })
    }
}

/// Takes in a regular ros topic and type and returns a zenoh topic mangled in the way the zenoh-ros1-plugin does
fn mangle_topic(topic: &str, type_str: &str, md5sum: &str) -> String {
    // Name mangling stuff!
    // See: https://github.com/eclipse-zenoh/zenoh-plugin-ros1/issues/131
    // Explicit implementation at: https://github.com/eclipse-zenoh/zenoh-plugin-ros1/blob/main/zenoh-plugin-ros1/src/ros_to_zenoh_bridge/topic_utilities.rs
    // Note: the implementation inside of the bridge uses unstable zenoh, duplicating implementation here with stable zenoh instead.

    // Remove leading and trailing slashes in the topic
    let topic = topic.trim_start_matches('/').trim_end_matches("/");
    // Encode the type as hex
    let type_str = hex::encode(type_str.as_bytes());
    format!("{type_str}/{md5sum}/{BRIDGE_NAMESPACE}/{topic}")
}

impl GraphProvider for ZenohClient {
    async fn list_topics(&self) -> Result<Vec<TopicInfo>> {
        let graph = self.graph.read().await;
        let mut topics = BTreeMap::new();
        for fact in graph.values() {
            if matches!(
                fact.class,
                DiscoveryClass::Publisher | DiscoveryClass::Subscriber
            ) {
                insert_discovered_type(
                    &mut topics,
                    fact.name.clone(),
                    fact.type_name.clone(),
                    "topic",
                )?;
            }
        }
        Ok(topics
            .into_iter()
            .map(|(name, type_name)| TopicInfo { name, type_name })
            .collect())
    }

    async fn list_services(&self) -> Result<Vec<ServiceInfo>> {
        let graph = self.graph.read().await;
        let mut services = BTreeMap::new();
        for fact in graph.values() {
            if fact.class == DiscoveryClass::Service {
                insert_discovered_type(
                    &mut services,
                    fact.name.clone(),
                    fact.type_name.clone(),
                    "service",
                )?;
            }
        }
        Ok(services
            .into_iter()
            .map(|(name, type_name)| ServiceInfo { name, type_name })
            .collect())
    }
}

fn make_discovery_key(class: DiscoveryClass, name: &str, type_name: &str, md5sum: &str) -> String {
    let name = name.trim_start_matches('/').trim_end_matches('/');
    format!(
        "ros1_discovery_info/{DISCOVERY_NAMESPACE}/{}/{}/{md5sum}/{BRIDGE_NAMESPACE}/{name}",
        class.as_str(),
        hex::encode(type_name.as_bytes()),
    )
}

fn parse_discovery_key(key: &str) -> Option<DiscoveryFact> {
    let parts: Vec<_> = key.split('/').collect();
    if parts.len() < 7 || parts[0] != "ros1_discovery_info" {
        return None;
    }
    let class = DiscoveryClass::parse(parts[2])?;
    let type_name = decode_hex_type(parts[3])?;
    let md5sum = parts[4].to_string();
    let name = format!("/{}", parts[6..].join("/"));
    Some(DiscoveryFact {
        class,
        name,
        type_name,
        md5sum,
    })
}

fn decode_hex_type(hex_type: &str) -> Option<String> {
    String::from_utf8(hex::decode(hex_type).ok()?).ok()
}

fn insert_discovered_type(
    types: &mut BTreeMap<String, String>,
    name: String,
    type_name: String,
    kind: &str,
) -> Result<()> {
    if let Some(existing) = types.get(&name) {
        if existing != &type_name {
            return Err(Error::ServerError(format!(
                "{kind} {name} discovered with conflicting types {existing} and {type_name}"
            )));
        }
    } else {
        types.insert(name, type_name);
    }
    Ok(())
}

/// The client type returned by [ServiceProvider::service_client] on [ZenohClient]
/// This type allows calling a service multiple times without re-negotiating the connection each time.
pub struct ZenohServiceClient<T: RosServiceType> {
    session: zenoh::Session,
    zenoh_query: String,
    _discovery: DiscoveryDeclaration,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosServiceType> Service<T> for ZenohServiceClient<T> {
    async fn call(&self, request: &T::Request) -> Result<T::Response> {
        // Note: Zenoh decided the 4 byte length header is not part of the payload
        let request_bytes = roslibrust_serde_rosmsg::to_vec_skip_length(request).map_err(|e| {
            Error::SerializationError(format!("Failed to serialize message: {e:?}"))
        })?;

        let query = match self
            .session
            .get(&self.zenoh_query)
            .payload(request_bytes)
            .await
        {
            Ok(query) => query,
            Err(e) => {
                // TODO errors still suck with this API...
                return Err(Error::Unexpected(anyhow::anyhow!(
                    "Failed to create query for service: {e:?}"
                )));
            }
        };

        let response = match query.recv_async().await {
            Ok(data) => data,
            Err(e) => {
                return Err(Error::Unexpected(anyhow::anyhow!(
                    "Failed to receive response from service: {e:?}"
                )));
            }
        };

        // TODO unclear why this is double failable in the API
        let sample = match response.into_result() {
            Ok(bytes) => bytes,
            Err(e) => {
                return Err(Error::Unexpected(anyhow::anyhow!(
                    "Failed to receive sample from service: {e:?}"
                )));
            }
        };

        let msg = deserialize_payload(sample.payload(), "service response")?;
        Ok(msg)
    }
}

/// The type returned by [ServiceProvider::advertise_service] on [ZenohClient].
/// This type is self de-registering, and dropping the server will automatically un-advertise the service.
pub struct ZenohServiceServer {
    // Dropping this will stop zenoh's declaration of the queryable
    _queryable: zenoh::query::Queryable<()>,
    _discovery: DiscoveryDeclaration,
}

impl ServiceProvider for ZenohClient {
    type ServiceClient<T: RosServiceType> = ZenohServiceClient<T>;
    type ServiceServer = ZenohServiceServer;

    async fn call_service<SrvType: RosServiceType>(
        &self,
        service: impl ToGlobalTopicName,
        request: SrvType::Request,
    ) -> Result<SrvType::Response> {
        let service: GlobalTopicName = service.to_global_name()?;
        // TODO should be able to optimize this...
        let client = ZenohClient::service_client::<SrvType>(self, service.as_ref()).await?;
        client.call(&request).await
    }

    async fn service_client<SrvType: RosServiceType + 'static>(
        &self,
        service: impl ToGlobalTopicName,
    ) -> Result<Self::ServiceClient<SrvType>> {
        let service: GlobalTopicName = service.to_global_name()?;
        let mangled_topic =
            mangle_topic(service.as_ref(), SrvType::ROS_SERVICE_NAME, SrvType::MD5SUM);
        let discovery = DiscoveryDeclaration::new(
            self.session.clone(),
            DiscoveryClass::Client,
            service.as_ref(),
            SrvType::ROS_SERVICE_NAME,
            SrvType::MD5SUM,
        )
        .await?;

        Ok(ZenohServiceClient {
            session: self.session.clone(),
            zenoh_query: mangled_topic,
            _discovery: discovery,
            _marker: std::marker::PhantomData,
        })
    }

    async fn advertise_service<SrvType: RosServiceType + 'static, F: ServiceFn<SrvType>>(
        &self,
        service: impl ToGlobalTopicName,
        server: F,
    ) -> Result<Self::ServiceServer> {
        let service: GlobalTopicName = service.to_global_name()?;
        let mangled_topic =
            mangle_topic(service.as_ref(), SrvType::ROS_SERVICE_NAME, SrvType::MD5SUM);

        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();

        let x = self
            .session
            .declare_queryable(mangled_topic)
            .callback(move |query| {
                let _ = tx.send(query).map_err(|e| {
                    error!("Failed to send query: {e:?}");
                });
            })
            .await
            .map_err(|e| {
                Error::Unexpected(anyhow::anyhow!("Failed to declare queryable: {e:?}"))
            })?;
        let discovery = DiscoveryDeclaration::new(
            self.session.clone(),
            DiscoveryClass::Service,
            service.as_ref(),
            SrvType::ROS_SERVICE_NAME,
            SrvType::MD5SUM,
        )
        .await?;

        // Move the server into an Arc so we can ensure lifetime of it remains valid across spawn_blocking:
        let server = std::sync::Arc::new(server);

        // Spawn a task to handle the queries
        // This task will shut down when queryable is dropped
        tokio::spawn(async move {
            while let Some(query) = rx.recv().await {
                debug!("Got query: {query:?}");
                let Some(payload) = query.payload() else {
                    error!("Received a query with no payload for a ros0 service {query:?}");
                    continue;
                };
                let Ok(request) = deserialize_payload(payload, "service request").map_err(|e| {
                    error!("{e:?}");
                }) else {
                    continue;
                };

                // Evaluate the server function inside a spawn_blocking to uphold trait expectations from roslibrust_common
                let server_copy = server.clone();
                let join_response = tokio::task::spawn_blocking(move || server_copy(request)).await;
                let response = match join_response {
                    Ok(Ok(response)) => response,
                    Ok(Err(e)) => {
                        error!("Failed to handle request: {e:?}");
                        continue;
                    }
                    Err(e) => {
                        error!("Failed to join task: {e:?}");
                        continue;
                    }
                };

                let Ok(response_bytes) = roslibrust_serde_rosmsg::to_vec_skip_length(&response)
                    .map_err(|e| {
                        error!("Failed to serialize response: {e:?}");
                    })
                else {
                    continue;
                };

                let _ = query
                    .reply(query.key_expr(), response_bytes)
                    .await
                    .map_err(|e| {
                        error!("Failed to reply to query: {e:?}");
                    });
            }
        });

        Ok(ZenohServiceServer {
            _queryable: x,
            _discovery: discovery,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mangle_topic() {
        assert_eq!(
            mangle_topic(
                "/chatter",
                "std_msgs/String",
                "992ce8a1687cec8c8bd883ec73ca41d1"
            ),
            "7374645f6d7367732f537472696e67/992ce8a1687cec8c8bd883ec73ca41d1/*/chatter"
        );
    }

    #[test]
    fn test_mangle_service() {
        assert_eq!(
            mangle_topic(
                "/service_server_rs/my_set_bool",
                "std_srvs/SetBool",
                "09fb03525b03e7ea1fd3992bafd87e16"
            ),
            "7374645f737276732f536574426f6f6c/09fb03525b03e7ea1fd3992bafd87e16/*/service_server_rs/my_set_bool");
    }

    #[test]
    fn test_make_discovery_key() {
        assert_eq!(
            make_discovery_key(
                DiscoveryClass::Service,
                "/service_server_rs/my_set_bool",
                "std_srvs/SetBool",
                "09fb03525b03e7ea1fd3992bafd87e16"
            ),
            "ros1_discovery_info/*/srv/7374645f737276732f536574426f6f6c/09fb03525b03e7ea1fd3992bafd87e16/*/service_server_rs/my_set_bool"
        );
    }

    #[test]
    fn test_parse_discovery_key() {
        assert_eq!(
            parse_discovery_key(
                "ros1_discovery_info/*/srv/7374645f737276732f536574426f6f6c/09fb03525b03e7ea1fd3992bafd87e16/*/service_server_rs/my_set_bool"
            ),
            Some(DiscoveryFact {
                class: DiscoveryClass::Service,
                name: "/service_server_rs/my_set_bool".to_string(),
                type_name: "std_srvs/SetBool".to_string(),
                md5sum: "09fb03525b03e7ea1fd3992bafd87e16".to_string(),
            })
        );
    }

    #[test]
    #[should_panic]
    #[allow(clippy::unnecessary_literal_unwrap)]
    fn confirm_client_handle_impls_ros() {
        struct MyClient<T: Ros> {
            _client: T,
        }

        let new_mock: std::result::Result<ZenohClient, _> = Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            // Should panic here, but proves that ClientHandle implements Ros
            // when this test compiles
            _client: new_mock.unwrap(),
        };
    }
}
