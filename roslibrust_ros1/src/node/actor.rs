use crate::{
    names::Name, node::XmlRpcServer, publisher::Publication, service_client::ServiceClientLink,
    service_server::ServiceServerLink, subscriber::Subscription, MasterClient, NodeError,
    ServiceClient, TypeErasedCallback,
};
use bytes::Bytes;
use log::*;
use roslibrust_common::{Error, RosMessageType, RosServiceType, ServiceFn};
use std::{
    collections::HashMap,
    io,
    net::Ipv4Addr,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Weak,
    },
};
use tokio::sync::{broadcast, mpsc, Mutex};
use tokio_util::sync::CancellationToken;

/// Represents a communication handle to an underlying node server
/// The node server handles all communication with ROS Master and keeps
/// track of subscriptions, publishers, etc.
/// When all NodeServerHandles are dropped, the Node is dropped and cleaned up.
#[derive(Clone)]
pub(crate) struct NodeServerHandle {
    // Shared reference to the actual node
    pub(crate) node: Arc<Mutex<Node>>,
    // Shared flag to track if the node is alive
    // This is cheaper to check than locking the node
    is_alive: Arc<AtomicBool>,
}

/// Weak reference to a NodeServerHandle
/// This doesn't keep the Node alive and can be upgraded to a NodeServerHandle
#[derive(Clone)]
pub struct WeakNodeServerHandle {
    pub(crate) node: Weak<Mutex<Node>>,
    is_alive: Arc<AtomicBool>,
}

impl WeakNodeServerHandle {
    /// Attempt to upgrade the weak reference to a strong reference
    /// Returns None if the Node has already been dropped
    pub(crate) fn upgrade(&self) -> Option<NodeServerHandle> {
        self.node.upgrade().map(|node| NodeServerHandle {
            node,
            is_alive: self.is_alive.clone(),
        })
    }

    /// Attempt to unregister a publisher with the node
    /// This is a helper method for use in Drop implementations
    /// If the node is already gone, this is a no-op
    pub(crate) fn try_unregister_publisher(&self, topic_name: &str) {
        if let Some(node_handle) = self.upgrade() {
            let topic_name = topic_name.to_string();
            tokio::spawn(async move {
                let mut node = node_handle.node.lock().await;
                if let Err(e) = node.unregister_publisher(&topic_name).await {
                    error!("Failed to unregister publisher {topic_name}: {e:?}");
                }
            });
        } else {
            debug!("Node already dropped, skipping publisher unadvertisement for {topic_name}");
        }
    }

    /// Attempt to unregister a service server with the node
    /// This is a helper method for use in Drop implementations
    /// If the node is already gone, this is a no-op
    pub(crate) fn try_unregister_service_server(&self, service_name: &str) {
        if let Some(node_handle) = self.upgrade() {
            let service_name = service_name.to_string();
            tokio::spawn(async move {
                let mut node = node_handle.node.lock().await;
                if let Err(e) = node.unregister_service_server(&service_name).await {
                    error!("Failed to unregister service server {service_name}: {e:?}");
                }
            });
        } else {
            debug!("Node already dropped, skipping service unadvertisement for {service_name}");
        }
    }
}

impl NodeServerHandle {
    /// Create a weak reference to this handle
    pub(crate) fn downgrade(&self) -> WeakNodeServerHandle {
        WeakNodeServerHandle {
            node: Arc::downgrade(&self.node),
            is_alive: self.is_alive.clone(),
        }
    }

    /// Check if the node is still alive
    /// This is a cheap operation that doesn't require locking the node
    pub(crate) fn is_alive(&self) -> bool {
        self.is_alive.load(Ordering::Relaxed)
    }
    /// Get the URI of the client node.
    pub(crate) async fn get_client_uri(&self) -> Result<String, NodeError> {
        let node = self.node.lock().await;
        Ok(node.client.client_uri().to_owned())
    }

    /// Registers a publisher with the underlying node server
    /// Returns a channel that the raw bytes of a publish can be shoved into to queue the publish
    /// Uses Bytes for efficient cloning (reference counted) when there are multiple subscribers
    pub(crate) async fn register_publisher<T: RosMessageType>(
        &self,
        topic: &str,
        queue_size: usize,
        latching: bool,
    ) -> Result<(broadcast::Sender<Bytes>, mpsc::Sender<()>), NodeError> {
        // Create a weak reference to pass to the publication
        let weak_node = self.downgrade();
        let mut node = self.node.lock().await;
        node.register_publisher(
            topic.to_owned(),
            T::ROS_TYPE_NAME,
            queue_size,
            T::DEFINITION.to_owned(),
            T::MD5SUM.to_owned(),
            latching,
            weak_node,
        )
        .await
    }

    /// Registers a publisher with the underlying node server
    /// Returns a channel that the raw bytes of a publish can be shoved into to queue the publish
    /// Uses Bytes for efficient cloning (reference counted) when there are multiple subscribers
    pub(crate) async fn register_publisher_any(
        &self,
        topic: &str,
        topic_type: &str,
        msg_definition: &str,
        queue_size: usize,
        latching: bool,
    ) -> Result<(broadcast::Sender<Bytes>, mpsc::Sender<()>), NodeError> {
        let md5sum_res =
            roslibrust_common::md5sum::from_message_definition(topic_type, msg_definition);
        let md5sum = match md5sum_res {
            // TODO(lucasw) make a new error type for this?
            Err(err) => {
                log::error!("{:?}", err);
                return Err(NodeError::IoError(io::Error::from(
                    io::ErrorKind::ConnectionAborted,
                )));
            }
            Ok(md5sum_rv) => md5sum_rv,
        };

        // Create a weak reference to pass to the publication
        let weak_node = self.downgrade();
        let mut node = self.node.lock().await;
        node.register_publisher(
            topic.to_owned(),
            topic_type,
            queue_size,
            msg_definition.to_owned(),
            md5sum,
            latching,
            weak_node,
        )
        .await
    }

    pub(crate) async fn unregister_publisher(&self, topic: &str) -> Result<(), NodeError> {
        let mut node = self.node.lock().await;
        node.unregister_publisher(topic).await
    }

    /// Registers a service client with the underlying node server
    /// This returns a channel that can be used for making service calls
    /// service calls will be queued in the channel and resolved when able.
    pub(crate) async fn register_service_client<T: RosServiceType>(
        &self,
        service_name: &Name,
    ) -> Result<ServiceClient<T>, NodeError> {
        let srv_definition =
            String::from_iter([T::Request::DEFINITION, "\n", T::Response::DEFINITION].into_iter());

        let mut node = self.node.lock().await;
        let link = node
            .register_service_client(
                service_name,
                T::ROS_SERVICE_NAME,
                &srv_definition,
                T::MD5SUM,
            )
            .await
            .map_err(|err| {
                log::error!("Failed to register service client: {err}");
                NodeError::IoError(io::Error::from(io::ErrorKind::ConnectionAborted))
            })?;
        let sender = link.get_sender();

        Ok(ServiceClient::new(service_name, sender, link))
    }

    pub(crate) async fn register_service_server<T, F>(
        &self,
        service_name: &Name,
        server: F,
    ) -> Result<(), NodeError>
    where
        T: RosServiceType,
        F: ServiceFn<T>,
    {
        // Type erase the server function here
        // Here we encode the type information of the service type passed in as T into the closure
        // This gives a generic closure that operates on byte arrays that we can then store and use freely
        // Uses Bytes for efficient handling of incoming request data
        let server_typeless =
            move |message: Bytes| -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
                let request = roslibrust_serde_rosmsg::from_slice::<T::Request>(&message)
                    .map_err(|err| Error::SerializationError(err.to_string()))?;
                let response = server(request)?;
                Ok(roslibrust_serde_rosmsg::to_vec(&response)
                    .map_err(|err| Error::SerializationError(err.to_string()))?)
            };
        let server_typeless = Box::new(server_typeless);

        let srv_definition =
            String::from_iter([T::Request::DEFINITION, "\n", T::Response::DEFINITION].into_iter());

        let mut node = self.node.lock().await;
        node.register_service_server(
            service_name,
            T::ROS_SERVICE_NAME,
            &srv_definition,
            server_typeless,
            T::MD5SUM,
        )
        .await
        .map_err(|err| {
            log::error!("Failed to register service server: {err}");
            NodeError::IoError(io::Error::from(io::ErrorKind::ConnectionAborted))
        })
    }

    /// Called to remove a service server
    /// Delegates to the NodeServer
    pub(crate) async fn unadvertise_service(&self, service_name: &str) -> Result<(), NodeError> {
        log::debug!("Queuing unregister service server command for: {service_name:?}");
        let mut node = self.node.lock().await;
        node.unregister_service_server(service_name)
            .await
            .map_err(|e| {
                log::error!("Failed to unadvertise service server {service_name:?}: {e:?}");
                NodeError::IoError(io::Error::from(io::ErrorKind::InvalidData))
            })
    }

    /// Registers a subscription with the underlying node server
    /// If this is the first time the given topic has been subscribed to (by this node)
    /// rosmaster will be informed.
    /// Otherwise, a new rx handle will simply be returned to the existing channel.
    /// Uses Bytes for efficient cloning (reference counted) when there are multiple subscribers
    pub(crate) async fn register_subscriber<T: RosMessageType>(
        &self,
        topic: &str,
        queue_size: usize,
    ) -> Result<broadcast::Receiver<Bytes>, NodeError> {
        let mut node = self.node.lock().await;
        node.register_subscriber(
            topic,
            T::ROS_TYPE_NAME,
            queue_size,
            T::DEFINITION,
            T::MD5SUM,
        )
        .await
        .map_err(|err| {
            log::error!("Failed to register subscriber: {err}");
            err
        })
    }
}

// TODO we sometimes refer to this entity as "Node" and sometimes as "NodeServer"
// we should standardize terminology.
/// Represents a single "real" node, typically only one of these is expected per process
/// but nothing should specifically prevent that.
/// This is sometimes referred to as the NodeServer in the documentation, many NodeHandles can point to one NodeServer
pub(crate) struct Node {
    // The xmlrpc client this node uses to make requests to master
    pub(crate) client: MasterClient,
    // Synchronous version of the client for use in Drop
    sync_client: crate::SyncMasterClient,
    // Map of topic names to the publishing channels associated with the topic
    pub(crate) publishers: HashMap<String, Publication>,
    // Record of subscriptions this node has
    pub(crate) subscriptions: HashMap<String, Subscription>,
    // Map of topic names to the service client handles for each topic
    // Note: decision made to not hold a list of service clients here, instead each call
    // to register_service_client will create a new service client and return a sender to it
    // Okay to have multiple service clients for the same service.
    // Eventually, if we can also make ServiceClient clone()
    // This should give better control of how disconnection and lifetimes work for a given client
    // service_clients: HashMap<String, ServiceClientLink>,
    // Map of topic names to service server handles for each topic
    pub(crate) service_servers: HashMap<String, ServiceServerLink>,
    pub(crate) host_addr: Ipv4Addr,
    pub(crate) hostname: String,
    pub(crate) node_name: Name,
    // Cancellation token to signal shutdown of background tasks (e.g., XML-RPC server)
    // When this Node is dropped, it will cancel this token
    pub(crate) shutdown_token: CancellationToken,
    // Atomic flag to track if the node is alive
    // Set to true in construction, false in shutdown
    // This is shared across all handles to the node
    is_alive: Arc<AtomicBool>,
}

impl Node {
    #[allow(clippy::new_ret_no_self)]
    pub(crate) async fn new(
        master_uri: &str,
        hostname: &str,
        node_name: &Name,
        addr: Ipv4Addr,
    ) -> Result<NodeServerHandle, NodeError> {
        // Create a cancellation token for shutdown signaling
        let shutdown_token = CancellationToken::new();

        // Bind the xmlrpc server first to get the port, but don't start serving yet
        let bound_xmlrpc = XmlRpcServer::bind(addr)?;
        let client_uri = format!("http://{hostname}:{}", bound_xmlrpc.port());

        // Create the master client with the correct URI
        let rosmaster_client =
            MasterClient::new(master_uri, &client_uri, node_name.to_string()).await?;

        // Create a synchronous master client for use in Drop
        let sync_client =
            crate::SyncMasterClient::new(master_uri, &client_uri, node_name.to_string());

        // Create the node
        let is_alive = Arc::new(AtomicBool::new(true));
        let node = Self {
            client: rosmaster_client,
            sync_client,
            publishers: HashMap::new(),
            subscriptions: HashMap::new(),
            service_servers: HashMap::new(),
            host_addr: addr,
            hostname: hostname.to_owned(),
            node_name: node_name.to_owned(),
            shutdown_token: shutdown_token.clone(),
            is_alive: is_alive.clone(),
        };

        let node_arc = Arc::new(Mutex::new(node));
        let node_handle = NodeServerHandle {
            node: node_arc,
            is_alive,
        };

        // Create a weak reference for the xmlrpc server
        // This allows the server to access the node without keeping it alive
        let weak_node = node_handle.downgrade();

        // Now start serving with the properly initialized node
        // Pass the weak reference and cancellation token so the server can shut down when signaled
        bound_xmlrpc.serve(weak_node, shutdown_token);

        Ok(node_handle)
    }

    async fn register_subscriber(
        &mut self,
        topic: &str,
        topic_type: &str,
        queue_size: usize,
        msg_definition: &str,
        md5sum: &str,
    ) -> Result<broadcast::Receiver<Bytes>, NodeError> {
        match self.subscriptions.iter().find(|(key, _)| *key == topic) {
            Some((_topic, subscription)) => Ok(subscription.get_receiver()),
            None => {
                let mut subscription = Subscription::new(
                    &self.node_name,
                    topic,
                    topic_type,
                    queue_size,
                    msg_definition.to_owned(),
                    md5sum.to_owned(),
                );
                let current_publishers = self.client.register_subscriber(topic, topic_type).await?;
                for publisher in current_publishers {
                    if let Err(err) = subscription.add_publisher_source(&publisher).await {
                        log::error!("Unable to create subscriber connection to {publisher} for {topic}: {err}");
                    }
                }
                let receiver = subscription.get_receiver();
                self.subscriptions.insert(topic.to_owned(), subscription);
                Ok(receiver)
            }
        }
    }

    async fn register_publisher(
        &mut self,
        topic: String,
        topic_type: &str,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
        latching: bool,
        weak_node: WeakNodeServerHandle,
    ) -> Result<(broadcast::Sender<Bytes>, mpsc::Sender<()>), NodeError> {
        // Return handle to existing Publication if it exists
        let existing_entry = {
            self.publishers.iter().find_map(|(key, value)| {
                if key.as_str() != topic {
                    return None;
                }
                if value.topic_type() != topic_type {
                    warn!("Attempted to register publisher with different topic type than existing publisher: existing_type={}, new_type={}", value.topic_type(), topic_type);
                    // TODO MAJOR: this is a terrible error type to return...
                    return Some(Err(NodeError::IoError(std::io::Error::from(
                        std::io::ErrorKind::AddrInUse,
                    ))));
                }
                let (sender, shutdown) = value.get_senders();
                match shutdown.upgrade() {
                    Some(shutdown) => {
                        Some(Ok((sender, shutdown)))
                    }
                    None => {
                        error!("We still have an entry for a publication, but it has been shutdown");
                        // TODO MAJOR: this is a terrible error type to return...
                        Some(Err(NodeError::IoError(std::io::Error::from(
                            std::io::ErrorKind::AddrInUse,
                        ))))
                    }
                }
            })
        };
        // If we found an existing publication return the handle to it
        if let Some(handle) = existing_entry {
            let (sender, shutdown) = handle?;
            return Ok((sender, shutdown));
        }

        // Otherwise create a new Publication and advertise
        let (channel, sender, shutdown) = Publication::new(
            &self.node_name,
            latching,
            &topic,
            self.host_addr,
            queue_size,
            &msg_definition,
            &md5sum,
            topic_type,
            weak_node,
        )
        .await
        .map_err(|err| {
            log::error!("Failed to create publishing channel: {err:?}");
            err
        })?;
        self.publishers.insert(topic.clone(), channel);
        let _ = self.client.register_publisher(&topic, topic_type).await?;
        Ok((sender, shutdown))
    }

    pub(crate) async fn unregister_publisher(&mut self, topic: &str) -> Result<(), NodeError> {
        // Tell ros master we are no longer publishing this topic
        let err1 = self.client.unregister_publisher(topic).await;
        // Remove the publication from our internal state
        let err2 = self.publishers.remove(topic);
        if err1.is_err() || err2.is_none() {
            error!(
                "Failure unregistering publisher: {err1:?}, {}",
                err2.is_none()
            );
            // MAJOR TODO: this is a terrible error type to return...
            return Err(NodeError::IoError(std::io::Error::from(
                std::io::ErrorKind::AddrInUse,
            )));
        }
        Ok(())
    }

    /// Checks the internal state of the NodeServer to see if it has a service client registered for this service already
    /// If it does, it returns a Sender to the existing service client
    /// Otherwise, it creates a new service client and returns a Sender to the new service client
    async fn register_service_client(
        &mut self,
        service: &Name,
        service_type: &str,
        srv_definition: &str,
        md5sum: &str,
    ) -> Result<ServiceClientLink, Box<dyn std::error::Error>> {
        log::debug!("Registering service client for {service}");
        let service_name = service.resolve_to_global(&self.node_name).to_string();

        log::debug!("Creating new service client for {service}");
        let service_uri = self.client.lookup_service(&service_name).await?;

        log::debug!("Found service at {service_uri}");
        let server_link = ServiceClientLink::new(
            &self.node_name,
            &service_name,
            service_type,
            &service_uri,
            srv_definition,
            md5sum,
        )
        .await?;

        Ok(server_link)
    }

    /// Registers a type-erased server function with the NodeServer
    async fn register_service_server(
        &mut self,
        service: &Name,
        service_type: &str,
        srv_definition: &str,
        server: Box<TypeErasedCallback>,
        md5sum: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let found = self.service_servers.get_mut(service_type);

        // Create a new service server link
        // This actually hosts the TCP socket and responds to incoming requests
        let link = ServiceServerLink::new(
            server,
            self.host_addr,
            service.clone(),
            self.node_name.clone(),
            service_type.to_string(),
            md5sum.to_string(),
            srv_definition.to_string(),
        )
        .await?;
        let port = link.port();

        // Replace the existing entry or create a new one
        if let Some(server_in_map) = found {
            warn!("Existing service implementation for {service_type} found while registering service server. Previous implementation will be ejected");
            *server_in_map = link;
        } else {
            self.service_servers.insert(service.to_string(), link);
            // This is the address that ros will find this specific service server link
            // Use hostname (not host_addr) so other nodes can connect to us
            let service_uri = format!("rosrpc://{}:{}", self.hostname, port);

            // Inform ROS master we provide this service
            self.client
                .register_service(service.to_string(), service_uri)
                .await?;
        }

        Ok(())
    }

    pub(crate) async fn unregister_service_server(
        &mut self,
        service_name: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(service_link) = self.service_servers.remove(service_name) {
            log::debug!("Removing service_link for: {service_name:?}");
            // Inform rosmaster that we no longer provide this service
            // Use hostname (not host_addr) to match what was registered
            let uri = format!("rosrpc://{}:{}", self.hostname, service_link.port());
            self.client.unregister_service(service_name, uri).await?;
            Ok(())
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Attempt to unregister service that is not currently registered",
            )))
        }
    }

    // Clears any extant node connections with the ros master
    // This is not expected to be called anywhere other than the drop impl
    pub(crate) fn shutdown(&mut self) {
        debug!("Node shutdown called for: {}", self.node_name);

        // Mark the node as no longer alive
        self.is_alive.store(false, Ordering::Relaxed);

        // Tell xmlrpc server to shut down first
        self.shutdown_token.cancel();

        // Spawn cleanup in a detached thread to avoid blocking Drop
        // This is necessary because synchronous HTTP calls from Drop can cause issues
        // when multiple nodes are dropped simultaneously in a tokio runtime
        let sync_client = self.sync_client.clone();
        let subscriptions: Vec<String> = self.subscriptions.keys().cloned().collect();
        let publishers: Vec<String> = self.publishers.keys().cloned().collect();
        let service_servers: Vec<(String, String)> = self
            .service_servers
            .iter()
            .map(|(topic, link)| {
                let uri = format!("rosrpc://{}:{}", self.hostname, link.port());
                (topic.clone(), uri)
            })
            .collect();
        let node_name = self.node_name.to_string();

        for topic in subscriptions {
            debug!("Node shutdown is cleaning up subscription: {topic}");
            match sync_client.unregister_subscriber(&topic) {
                Ok(true) => {
                    debug!("Successfully unregistered subscriber: {topic}");
                }
                Ok(false) => {
                    warn!("Failed to unregister subscriber: {topic}, it was not advertised");
                }
                Err(e) => {
                    error!(
                        "Failed to unregister subscriber: {topic} while shutting down node: {e}"
                    );
                }
            }
        }

        for topic in publishers {
            debug!("Node shutdown is cleaning up publishing: {topic}");
            match sync_client.unregister_publisher(&topic) {
                Ok(true) => {
                    debug!("Successfully unregistered publisher: {topic}");
                }
                Ok(false) => {
                    warn!("Failed to unregister publisher: {topic}, it was not advertised");
                }
                Err(e) => {
                    error!("Failed to unregister publisher: {topic} while shutting down node: {e}");
                }
            }
        }

        for (topic, uri) in service_servers {
            debug!("Node shutdown is cleaning up service: {topic}");
            match sync_client.unregister_service(&topic, uri) {
                Ok(true) => {
                    debug!("Successfully unregistered service: {topic}");
                }
                Ok(false) => {
                    warn!("Failed to unregister service: {topic}, it was not advertised");
                }
                Err(e) => {
                    error!("Failed to unregister service: {topic} while shutting down node: {e}");
                }
            }
        }

        debug!("Node shutdown complete for: {node_name}");
    }
}

// It is important to clean-up any stray topic / service connections when we shut down
// Goal of this implementation is that the node appears fully dead to ROS after this and
// `rosnode list` / `rosnode info` don't show any remaining connections.
impl Drop for Node {
    fn drop(&mut self) {
        self.shutdown();
    }
}
