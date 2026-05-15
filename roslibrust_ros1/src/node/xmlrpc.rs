use super::Node;
use hyper::{Body, Response, StatusCode};
use log::*;
use std::{
    convert::Infallible,
    net::{Ipv4Addr, SocketAddr},
    sync::Weak,
};
use tokio::sync::Mutex;
use tokio_util::sync::CancellationToken;

#[allow(unused)]
enum RosXmlStatusCode {
    Error,
    Failure,
    Success,
    Other(i32),
}

impl RosXmlStatusCode {
    pub fn code(&self) -> i32 {
        match self {
            RosXmlStatusCode::Error => -1,
            RosXmlStatusCode::Failure => 0,
            RosXmlStatusCode::Success => 1,
            RosXmlStatusCode::Other(code) => *code,
        }
    }
}

/// Hosts an xmlrpc API that rosmaster will call to notify of new subscribers / publishers etc.
/// Is also called by other ros nodes to initiate point to point connections.
/// Note: ROS uses "master/slave" terminology here. We continue to refer to the ROS central server as the ROS master,
/// but are intentionally using "XmlRpcServer" in place of where ROS says "Slave API"
pub(crate) struct XmlRpcServer {}

/// Intermediate structure holding a bound server that hasn't started serving yet
pub(crate) struct BoundXmlRpcServer {
    port: u16,
    server: hyper::server::Builder<hyper::server::conn::AddrIncoming>,
}

impl BoundXmlRpcServer {
    /// Allows getting the port so the node can know its full URI
    pub fn port(&self) -> u16 {
        self.port
    }

    /// Start serving requests with a weak reference to the node and cancellation token
    /// The server will shutdown when the cancellation token is cancelled or when the node is dropped
    pub fn serve(self, weak_node: Weak<Mutex<Node>>, cancellation_token: CancellationToken) {
        let make_svc = hyper::service::make_service_fn(move |connection| {
            debug!("New node xmlrpc connection {connection:?}");
            let weak_node = weak_node.clone();
            async move {
                Ok::<_, Infallible>(hyper::service::service_fn(move |req| {
                    XmlRpcServer::respond(weak_node.clone(), req)
                }))
            }
        });

        let server = self.server.serve(make_svc);

        tokio::spawn(async move {
            tokio::select! {
                result = server => {
                    if let Err(err) = result {
                        log::error!("xmlrpc server encountered error: {err:?}");
                    }
                }
                _ = cancellation_token.cancelled() => {
                    debug!("XmlRpc server shutting down due to cancellation");
                }
            }
        });
    }
}

impl XmlRpcServer {
    /// Bind to an address without starting to serve
    /// Returns a BoundXmlRpcServer that can be used to get the port and then start serving
    pub fn bind(host_addr: Ipv4Addr) -> Result<BoundXmlRpcServer, XmlRpcError> {
        let host_addr = SocketAddr::from((host_addr, 0));
        let server = hyper::server::Server::try_bind(&host_addr)?;
        let port = server.local_addr().port();

        Ok(BoundXmlRpcServer { port, server })
    }

    // Our actual service handler with our error type
    async fn respond_inner(
        weak_node: Weak<Mutex<Node>>,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Box<Response<Body>>> {
        // Try to upgrade the weak reference to a strong reference
        // If this fails, the node has been dropped and we should error out
        let node_arc = weak_node.upgrade().ok_or_else(|| {
            Box::new(Self::make_error_response(
                std::io::Error::new(std::io::ErrorKind::NotFound, "Node has been dropped"),
                "Node no longer exists, shutting down XmlRpc server",
                StatusCode::SERVICE_UNAVAILABLE,
            ))
        })?;
        // Await the bytes of the body
        let body = hyper::body::to_bytes(body).await.map_err(|e| {
            Box::new(Self::make_error_response(
                e,
                "Failed to get bytes from http request on xmlrpc server, request ignored",
                StatusCode::BAD_REQUEST,
            ))
        })?;

        // Parse as string
        let body = String::from_utf8(body.to_vec()).map_err(|e| {
            Box::new(Self::make_error_response(
                e,
                "Failed to parse http body as valid utf8 string, request ignored",
                StatusCode::BAD_REQUEST,
            ))
        })?;

        // Parse as xmlrpc request
        let (method_name, args) = serde_xmlrpc::request_from_str(&body).map_err(|e| {
            Box::new(Self::make_error_response(
                e,
                "Failed to parse valid xmlrpc method request out of body, request ignored",
                StatusCode::BAD_REQUEST,
            ))
        })?;

        // Match on allowable functions
        match method_name.as_str() {
            "getMasterUri" => {
                debug!("getMasterUri called by {args:?}");
                let node = node_arc.lock().await;
                let uri = node.client.get_master_uri().to_owned();
                Self::to_response(uri)
            }
            "getPid" => {
                debug!("getPid called by {args:?}");
                let pid = std::process::id();
                let pid: i32 = pid.try_into().map_err(|e| {
                    Self::make_error_response(e,
                         "Operation system returned a PID which does not fit into i32, and therefor cannot be sent via xmlrpc",
                         StatusCode::INTERNAL_SERVER_ERROR)})?;
                Self::to_response(pid)
            }
            "getSubscriptions" => {
                debug!("getSubscriptions called by {args:?}");
                let node = node_arc.lock().await;
                let subs: Vec<(String, String)> = node
                    .subscriptions
                    .iter()
                    .map(|(topic_name, subscription)| {
                        (topic_name.clone(), subscription.topic_type().to_owned())
                    })
                    .collect();
                match serde_xmlrpc::to_value(subs) {
                    Ok(subs) => Self::to_response(subs),
                    Err(e) => Err(Box::new(Self::make_error_response(
                        e,
                        "Subscriptions contained names which could not be validly serialized to xmlrpc",
                        StatusCode::INTERNAL_SERVER_ERROR)))
                }
            }
            "getPublications" => {
                debug!("getPublications called by {args:?}");
                let node = node_arc.lock().await;
                let pubs: Vec<(String, String)> = node
                    .publishers
                    .iter()
                    .map(|(key, entry)| (key.clone(), entry.topic_type().to_owned()))
                    .collect();
                match serde_xmlrpc::to_value(pubs) {
                    Ok(pubs) => Self::to_response(pubs),
                    Err(e) => Err(Box::new(Self::make_error_response(
                        e,
                        "Publications contained names which could not be validly serialized to xmlrpc",
                        StatusCode::INTERNAL_SERVER_ERROR)))
                }
            }
            "paramUpdate" => {
                // Not supporting params for first cut
                debug!("paramUpdate called by {args:?}");
                unimplemented!()
            }
            "publisherUpdate" => {
                debug!("publisherUpdate called by {args:?}");
                let (_caller_id, topic, publishers): (String, String, Vec<String>) =
                    serde_xmlrpc::from_values(args).map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Failed to parse arguments to publisherUpdate",
                            StatusCode::BAD_REQUEST,
                        )
                    })?;

                let mut node = node_arc.lock().await;
                if let Some(subscription) = node.subscriptions.get_mut(&topic) {
                    // First, remove any publishers that are no longer in the list
                    subscription.remove_stale_publishers(&publishers).await;
                    // Then add any new publishers
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

                // ROS's API is for us to still return an int, but the value is literally named "ignore"...
                Self::to_response(0)
            }
            "requestTopic" => {
                debug!("requestTopic called by {args:?}");
                let (caller_id, topic, protocols): (String, String, Vec<Vec<String>>) =
                    serde_xmlrpc::from_values(args).map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Failed to parse arguments to requestTopic",
                            StatusCode::BAD_REQUEST,
                        )
                    })?;
                let protocols = protocols.iter().flatten().cloned().collect::<Vec<_>>();
                debug!("Request for topic {topic} from {caller_id} via protocols {protocols:?}");

                let node = node_arc.lock().await;
                let params = if protocols.iter().any(|proto| proto.as_str() == "TCPROS") {
                    if let Some((_key, publishing_channel)) =
                        node.publishers.iter().find(|(key, _pub)| *key == &topic)
                    {
                        crate::ProtocolParams {
                            hostname: node.hostname.clone(),
                            protocol: String::from("TCPROS"),
                            port: publishing_channel.port(),
                        }
                    } else {
                        return Err(Box::new(Self::make_error_response(
                            std::io::Error::new(
                                std::io::ErrorKind::NotFound,
                                format!(
                                "Got request for topic {topic} which this node does not publish"
                            ),
                            ),
                            "Topic not found",
                            StatusCode::NOT_FOUND,
                        )));
                    }
                } else {
                    return Err(Box::new(Self::make_error_response(
                        std::io::Error::new(
                            std::io::ErrorKind::Unsupported,
                            format!("No supported protocols in request: {protocols:?}"),
                        ),
                        "Unsupported protocol",
                        StatusCode::BAD_REQUEST,
                    )));
                };

                let response = Self::make_success_response(
                    RosXmlStatusCode::Success,
                    format!("ready on {}:{}", params.hostname.clone(), params.port).as_str(),
                    serde_xmlrpc::to_value((params.protocol, params.hostname, params.port))
                        .unwrap(),
                );

                log::debug!("Sending response for requested topic {response:?}");
                Ok(response)
            }
            "shutdown" => {
                debug!("shutdown called by {args:?}");
                let (caller_id, msg): (String, String) =
                    serde_xmlrpc::from_values(args).map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Failed to parse arguments",
                            StatusCode::BAD_REQUEST,
                        )
                    })?;
                debug!("Received request for shutdown from {caller_id}: {msg}");

                // TODO this is not tested, we need to add a test for this
                // Trigger shutdown by spawning a task that will lock and shutdown the node
                let node_for_shutdown = node_arc.clone();
                tokio::spawn(async move {
                    let mut node_guard = node_for_shutdown.lock().await;
                    node_guard.shutdown();
                });

                Self::to_response(0)
            }
            // getBusStats, getBusInfo <= have decided not to impl these
            _ => {
                let error_str = format!("Client attempted call function {method_name} which is not implemented by the Node's xmlrpc server.");
                warn!("{error_str}");
                Ok(Response::builder()
                    .status(StatusCode::NOT_IMPLEMENTED)
                    .body(Body::from(error_str))
                    .unwrap())
            }
        }
    }

    fn make_success_response(
        status_code: RosXmlStatusCode,
        status_msg: &str,
        value: serde_xmlrpc::Value,
    ) -> Response<Body> {
        match serde_xmlrpc::response_to_string(
            vec![serde_xmlrpc::Value::Array(vec![
                status_code.code().into(),
                status_msg.into(),
                value,
            ])]
            .into_iter(),
        ) {
            Ok(body) => Response::builder()
                .status(StatusCode::OK)
                .body(Body::from(body))
                .unwrap(),
            Err(err) => Self::make_error_response(
                err,
                "Failed to serialize response data into valid xml",
                StatusCode::INTERNAL_SERVER_ERROR,
            ),
        }
    }

    // Helper function for converting serde_xmlrpc stuff into responses
    fn to_response(
        v: impl Into<serde_xmlrpc::Value>,
    ) -> Result<Response<Body>, Box<Response<Body>>> {
        serde_xmlrpc::response_to_string(
            vec![serde_xmlrpc::Value::Array(vec![
                1.into(),
                "".into(),
                v.into(),
            ])]
            .into_iter(),
        )
        .map_err(|e| {
            Box::new(Self::make_error_response(
                e,
                "Failed to serialize response data into valid xml",
                StatusCode::INTERNAL_SERVER_ERROR,
            ))
        })
        .map(|body| {
            Response::builder()
                .status(StatusCode::OK)
                .body(Body::from(body))
                .unwrap()
        })
    }

    // Helper function for converting error to http responses
    fn make_error_response(
        e: impl std::error::Error,
        msg: &str,
        code: hyper::http::StatusCode,
    ) -> Response<Body> {
        let error_msg = format!("{msg}: {e:?}");
        warn!("{error_msg}");
        Response::builder()
            .status(code)
            .body(Body::from(error_msg))
            .unwrap()
    }

    // Is the actual function we hand to hyper
    async fn respond(
        weak_node: Weak<Mutex<Node>>,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Infallible> {
        // Call our inner function and unwrap error type into response
        match Self::respond_inner(weak_node, body).await {
            Ok(body) => Ok(body),
            Err(body) => Ok(*body),
        }
    }
}

#[derive(thiserror::Error, Debug)]
pub enum XmlRpcError {
    #[error(transparent)]
    HyperError(#[from] hyper::Error),
}
