use crate::{
    ros1::{
        names::Name,
        node::{XmlRpcServer, XmlRpcServerHandle},
        publisher::Publication,
        service_client::{CallServiceRequest, ServiceServerLink},
        subscriber::Subscription,
        MasterClient, NodeError, ProtocolParams,
    },
    RosLibRustError, RosLibRustResult,
};
use abort_on_drop::ChildTask;
use roslibrust_codegen::{RosMessageType, RosServiceType};
use std::{collections::HashMap, io, net::Ipv4Addr, sync::Arc};
use tokio::sync::{broadcast, mpsc, oneshot};

#[derive(Debug)]
pub enum NodeMsg {
    GetMasterUri {
        reply: oneshot::Sender<String>,
    },
    GetClientUri {
        reply: oneshot::Sender<String>,
    },
    GetSubscriptions {
        reply: oneshot::Sender<Vec<(String, String)>>,
    },
    GetPublications {
        reply: oneshot::Sender<Vec<(String, String)>>,
    },
    SetPeerPublishers {
        topic: String,
        publishers: Vec<String>,
    },
    Shutdown,
    RegisterPublisher {
        reply: oneshot::Sender<Result<mpsc::Sender<Vec<u8>>, String>>,
        topic: String,
        topic_type: String,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    },
    RegisterSubscriber {
        reply: oneshot::Sender<Result<broadcast::Receiver<Vec<u8>>, String>>,
        topic: String,
        topic_type: String,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    },
    RegisterServiceClient {
        reply: oneshot::Sender<Result<mpsc::UnboundedSender<CallServiceRequest>, String>>,
        service: Name,
        service_type: String,
        srv_definition: String,
        md5sum: String,
    },
    RequestTopic {
        reply: oneshot::Sender<Result<ProtocolParams, String>>,
        caller_id: String,
        topic: String,
        protocols: Vec<String>,
    },
}

#[derive(Clone)]
pub(crate) struct NodeServerHandle {
    pub(crate) node_server_sender: mpsc::UnboundedSender<NodeMsg>,
    // If this handle should keep the underlying node task alive it will hold an
    // Arc to the underlying node task. This is an option because internal handles
    // within the node shouldn't keep it alive (e.g. what we hand to xml server)
    _node_task: Option<Arc<ChildTask<()>>>,
}

impl NodeServerHandle {
    /// Get the URI of the master node.
    pub async fn get_master_uri(&self) -> Result<String, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender
            .send(NodeMsg::GetMasterUri { reply: sender })?;
        Ok(receiver.await?)
    }

    /// Get the URI of the client node.
    pub async fn get_client_uri(&self) -> Result<String, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender
            .send(NodeMsg::GetClientUri { reply: sender })?;
        Ok(receiver.await?)
    }

    /// Gets the list of topics the node is currently subscribed to.
    /// Returns a tuple of (Topic Name, Topic Type) e.g. ("/rosout", "rosgraph_msgs/Log").
    pub async fn get_subscriptions(&self) -> Result<Vec<(String, String)>, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender
            .send(NodeMsg::GetSubscriptions { reply: sender })?;
        Ok(receiver.await?)
    }

    /// Gets the list of topic the node is currently publishing to.
    /// Returns a tuple of (Topic Name, Topic Type) e.g. ("/rosout", "rosgraph_msgs/Log").
    pub async fn get_publications(&self) -> Result<Vec<(String, String)>, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender
            .send(NodeMsg::GetPublications { reply: sender })?;
        Ok(receiver.await?)
    }

    /// Updates the list of know publishers for a given topic
    /// This is used to know who to reach out to for updates
    pub fn set_peer_publishers(
        &self,
        topic: String,
        publishers: Vec<String>,
    ) -> Result<(), NodeError> {
        Ok(self
            .node_server_sender
            .send(NodeMsg::SetPeerPublishers { topic, publishers })?)
    }

    pub fn shutdown(&self) -> Result<(), NodeError> {
        self.node_server_sender.send(NodeMsg::Shutdown)?;
        Ok(())
    }

    pub async fn register_publisher<T: RosMessageType>(
        &self,
        topic: &str,
        queue_size: usize,
    ) -> Result<mpsc::Sender<Vec<u8>>, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender.send(NodeMsg::RegisterPublisher {
            reply: sender,
            topic: topic.to_owned(),
            topic_type: T::ROS_TYPE_NAME.to_owned(),
            queue_size,
            msg_definition: T::DEFINITION.to_owned(),
            md5sum: T::MD5SUM.to_owned(),
        })?;
        let received = receiver.await?;
        Ok(received.map_err(|_err| {
            NodeError::IoError(io::Error::from(io::ErrorKind::ConnectionAborted))
        })?)
    }

    pub async fn register_service_client<T: RosServiceType>(
        &self,
        service_name: &Name,
    ) -> Result<mpsc::UnboundedSender<CallServiceRequest>, NodeError> {
        // Create a channel for hooking into the node server
        let (sender, receiver) = oneshot::channel();

        // Send the request to the node server and see if it accepts it
        self.node_server_sender
            .send(NodeMsg::RegisterServiceClient {
                reply: sender,
                service: service_name.to_owned(),
                service_type: T::ROS_SERVICE_NAME.to_owned(),
                srv_definition: String::from_iter(
                    [T::Request::DEFINITION, "\n", T::Response::DEFINITION].into_iter(),
                ),
                md5sum: T::MD5SUM.to_owned(),
            })?;
        let received = receiver.await?;
        Ok(received.map_err(|err| {
            log::error!("Failed to register service client: {err}");
            NodeError::IoError(io::Error::from(io::ErrorKind::ConnectionAborted))
        })?)
    }

    pub async fn register_subscriber<T: RosMessageType>(
        &self,
        topic: &str,
        queue_size: usize,
    ) -> Result<broadcast::Receiver<Vec<u8>>, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender.send(NodeMsg::RegisterSubscriber {
            reply: sender,
            topic: topic.to_owned(),
            topic_type: T::ROS_TYPE_NAME.to_owned(),
            queue_size,
            msg_definition: T::DEFINITION.to_owned(),
            md5sum: T::MD5SUM.to_owned(),
        })?;
        let received = receiver.await?;
        Ok(received.map_err(|err| {
            log::error!("Failed to register subscriber: {err}");
            NodeError::IoError(io::Error::from(io::ErrorKind::ConnectionAborted))
        })?)
    }

    pub async fn request_topic(
        &self,
        caller_id: &str,
        topic: &str,
        protocols: &[String],
    ) -> Result<ProtocolParams, NodeError> {
        let (sender, receiver) = oneshot::channel();
        self.node_server_sender.send(NodeMsg::RequestTopic {
            caller_id: caller_id.to_owned(),
            topic: topic.to_owned(),
            protocols: protocols.into(),
            reply: sender,
        })?;
        let received = receiver.await?;
        Ok(received.map_err(|err| {
            log::error!("Fail to coordinate channel between publisher and subscriber: {err}");
            NodeError::IoError(io::Error::from(io::ErrorKind::ConnectionAborted))
        })?)
    }
}

/// Represents a single "real" node, typically only one of these is expected per process
/// but nothing should specifically prevent that.
pub(crate) struct Node {
    // The xmlrpc client this node uses to make requests to master
    client: MasterClient,
    // Server which handles updates from the rosmaster and other ROS nodes
    _xmlrpc_server: XmlRpcServerHandle,
    // Receiver for requests to the Node actor
    node_msg_rx: mpsc::UnboundedReceiver<NodeMsg>,
    // Map of topic names to the publishing channels associated with the topic
    publishers: HashMap<String, Publication>,
    // Record of subscriptions this node has
    subscriptions: HashMap<String, Subscription>,
    // Map of topic names to the service client handles for each topic
    service_clients: HashMap<String, ServiceServerLink>,
    // TODO need signal to shutdown xmlrpc server when node is dropped
    host_addr: Ipv4Addr,
    hostname: String,
    node_name: Name,
}

impl Node {
    pub(crate) async fn new(
        master_uri: &str,
        hostname: &str,
        node_name: &Name,
        addr: Ipv4Addr,
    ) -> Result<NodeServerHandle, NodeError> {
        let (node_sender, node_receiver) = mpsc::unbounded_channel();
        let xml_server_handle = NodeServerHandle {
            node_server_sender: node_sender.clone(),
            // None here because this handle should not keep task alive
            _node_task: None,
        };
        // Create our xmlrpc server and bind our socket so we know our port and can determine our local URI
        let xmlrpc_server = XmlRpcServer::new(addr, xml_server_handle)?;
        let client_uri = format!("http://{hostname}:{}", xmlrpc_server.port());

        let rosmaster_client =
            MasterClient::new(master_uri, client_uri, node_name.to_string()).await?;
        let mut node = Self {
            client: rosmaster_client,
            _xmlrpc_server: xmlrpc_server,
            node_msg_rx: node_receiver,
            publishers: std::collections::HashMap::new(),
            subscriptions: std::collections::HashMap::new(),
            service_clients: std::collections::HashMap::new(),
            host_addr: addr,
            hostname: hostname.to_owned(),
            node_name: node_name.to_owned(),
        };

        let t = Arc::new(
            tokio::spawn(async move {
                loop {
                    match node.node_msg_rx.recv().await {
                        Some(NodeMsg::Shutdown) => {
                            log::info!("Shutdown requested, shutting down node");
                            break;
                        }
                        Some(node_msg) => {
                            node.handle_msg(node_msg).await;
                        }
                        None => {
                            break;
                        }
                    }
                }
            })
            .into(),
        );

        let node_server_handle = NodeServerHandle {
            node_server_sender: node_sender,
            _node_task: Some(t),
        };
        Ok(node_server_handle)
    }

    async fn handle_msg(&mut self, msg: NodeMsg) {
        match msg {
            NodeMsg::GetMasterUri { reply } => {
                let _ = reply.send(self.client.get_master_uri().to_owned());
            }
            NodeMsg::GetClientUri { reply } => {
                let _ = reply.send(self.client.client_uri().to_owned());
            }
            NodeMsg::GetSubscriptions { reply } => {
                let _ = reply.send(
                    self.subscriptions
                        .iter()
                        .map(|(topic_name, subscription)| {
                            (topic_name.clone(), subscription.topic_type().to_owned())
                        })
                        .collect(),
                );
            }
            NodeMsg::GetPublications { reply } => {
                let _ = reply.send(
                    self.publishers
                        .iter()
                        .map(|(key, entry)| (key.clone(), entry.topic_type().to_owned()))
                        .collect(),
                );
            }
            NodeMsg::SetPeerPublishers { topic, publishers } => {
                if let Some(subscription) = self.subscriptions.get_mut(&topic) {
                    for publisher_uri in publishers {
                        if let Err(err) = subscription.add_publisher_source(&publisher_uri).await {
                            log::error!(
                                "Unable to create subscribe stream for topic {topic}: {err}"
                            );
                        }
                    }
                } else {
                    log::warn!(
                        "Got peer publisher update for topic we weren't subscribed to, ignoring"
                    );
                }
            }
            NodeMsg::RegisterPublisher {
                reply,
                topic,
                topic_type,
                queue_size,
                msg_definition,
                md5sum,
            } => {
                let res = self
                    .register_publisher(topic, &topic_type, queue_size, msg_definition, md5sum)
                    .await;
                match res {
                    Ok(handle) => reply.send(Ok(handle)),
                    Err(err) => reply.send(Err(err.to_string())),
                }
                .expect("Failed to reply on oneshot");
            }
            NodeMsg::RegisterSubscriber {
                reply,
                topic,
                topic_type,
                queue_size,
                msg_definition,
                md5sum,
            } => {
                let _ = reply.send(
                    self.register_subscriber(
                        &topic,
                        &topic_type,
                        queue_size,
                        &msg_definition,
                        &md5sum,
                    )
                    .await
                    .map_err(|err| err.to_string()),
                );
            }
            NodeMsg::RegisterServiceClient {
                reply,
                service,
                service_type,
                srv_definition,
                md5sum,
            } => {
                let _ = reply.send(
                    self.register_service_client(&service, &service_type, &srv_definition, &md5sum)
                        .await
                        .map_err(|err| err.to_string()),
                );
            }
            NodeMsg::RequestTopic {
                reply,
                topic,
                protocols,
                ..
            } => {
                // TODO: Should move the actual implementation similar to RegisterPublisher
                if protocols
                    .iter()
                    .find(|proto| proto.as_str() == "TCPROS")
                    .is_some()
                {
                    if let Some((_key, publishing_channel)) =
                        self.publishers.iter().find(|(key, _pub)| *key == &topic)
                    {
                        let protocol_params = ProtocolParams {
                            hostname: self.hostname.clone(),
                            protocol: String::from("TCPROS"), // Hardcoded as the only option for now
                            port: publishing_channel.port(),
                        };
                        let _ = reply.send(Ok(protocol_params));
                    } else {
                        let err_str = format!("Got request for topic {topic} from subscriber which this node does not publish");
                        log::warn!("{err_str}");
                        let _ = reply.send(Err(err_str));
                    }
                } else {
                    let err_str = format!(
                        "No supported protocols in the request from the subscriber: {protocols:?}"
                    );
                    log::error!("{err_str}");
                    let _ = reply.send(Err(err_str));
                }
            }
            NodeMsg::Shutdown => {
                unreachable!("This node msg is handled in the wrapping handling code");
            }
        }
    }

    async fn register_subscriber(
        &mut self,
        topic: &str,
        topic_type: &str,
        queue_size: usize,
        msg_definition: &str,
        md5sum: &str,
    ) -> Result<broadcast::Receiver<Vec<u8>>, NodeError> {
        match self.subscriptions.iter().find(|(key, _)| *key == topic) {
            Some((_topic, subscription)) => Ok(subscription.get_receiver()),
            None => {
                let mut subscription = Subscription::new(
                    &self.node_name,
                    &topic,
                    &topic_type,
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
    ) -> Result<mpsc::Sender<Vec<u8>>, NodeError> {
        let existing_entry = {
            self.publishers.iter().find_map(|(key, value)| {
                if key.as_str() == &topic {
                    if value.topic_type() == topic_type {
                        Some(Ok(value.get_sender()))
                    } else {
                        Some(Err(NodeError::IoError(std::io::Error::from(
                            std::io::ErrorKind::AddrInUse,
                        ))))
                    }
                } else {
                    None
                }
            })
        };

        if let Some(handle) = existing_entry {
            Ok(handle?)
        } else {
            let channel = Publication::new(
                &self.node_name,
                false,
                &topic,
                self.host_addr,
                queue_size,
                &msg_definition,
                &md5sum,
                topic_type,
            )
            .await
            .map_err(|err| {
                log::error!("Failed to create publishing channel: {err:?}");
                err
            })?;
            let handle = channel.get_sender();
            self.publishers.insert(topic.clone(), channel);
            let _current_subscribers = self.client.register_publisher(&topic, topic_type).await?;
            Ok(handle)
        }
    }

    async fn register_service_client(
        &mut self,
        service: &Name,
        service_type: &str,
        srv_definition: &str,
        md5sum: &str,
    ) -> Result<mpsc::UnboundedSender<CallServiceRequest>, Box<dyn std::error::Error>> {
        log::debug!("Registering service client for {service}");
        let service_name = service.resolve_to_global(&self.node_name).to_string();

        let existing_entry = {
            self.service_clients.iter().find_map(|(key, value)| {
                if key.as_str() == &service_name {
                    if value.service_type() == service_type {
                        Some(Ok(value.get_sender()))
                    } else {
                        Some(Err(Box::new(std::io::Error::from(
                            std::io::ErrorKind::AddrInUse,
                        ))))
                    }
                } else {
                    None
                }
            })
        };

        if let Some(handle) = existing_entry {
            log::debug!("Found existing service client for {service}, returning existing handle");
            Ok(handle?)
        } else {
            log::debug!("Creating new service client for {service}");
            let service_uri = self.client.lookup_service(&service_name).await?;
            log::debug!("Found service at {service_uri}");
            let server_link = ServiceServerLink::new(
                &self.node_name,
                &service_name,
                service_type,
                &service_uri,
                srv_definition,
                md5sum,
            )
            .await?;

            let handle = server_link.get_sender();
            self.service_clients.insert(service_name, server_link);
            Ok(handle)
        }
    }
}