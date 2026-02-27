use crate::{names::Name, tcpros::ConnectionHeader};
use abort_on_drop::ChildTask;
use bytes::Bytes;
use log::*;
use roslibrust_common::{RosMessageType, ShapeShifter};
use std::{collections::HashMap, marker::PhantomData, sync::Arc, time::Duration};
use tokio::{
    io::AsyncWriteExt,
    net::TcpStream,
    sync::{
        broadcast::{self, error::RecvError},
        watch, RwLock,
    },
};

use super::tcpros;

pub struct Subscriber<T> {
    // Uses Bytes for efficient cloning (reference counted) when there are multiple subscribers
    receiver: broadcast::Receiver<Bytes>,
    _phantom: PhantomData<T>,
}

impl<T: RosMessageType> Subscriber<T> {
    pub(crate) fn new(receiver: broadcast::Receiver<Bytes>) -> Self {
        Self {
            receiver,
            _phantom: PhantomData,
        }
    }

    pub async fn next(&mut self) -> Option<Result<T, SubscriberError>> {
        trace!("Subscriber of type {:?} awaiting recv()", T::ROS_TYPE_NAME);
        let data = match self.receiver.recv().await {
            Ok(v) => {
                trace!("Subscriber of type {:?} received data", T::ROS_TYPE_NAME);
                v
            }
            Err(RecvError::Closed) => return None,
            Err(RecvError::Lagged(n)) => return Some(Err(SubscriberError::Lagged(n))),
        };
        trace!(
            "Subscriber of type {:?} deserializing data",
            T::ROS_TYPE_NAME
        );
        let tick = tokio::time::Instant::now();
        match roslibrust_serde_rosmsg::from_slice::<T>(&data[..]) {
            Ok(p) => {
                let duration = tick.elapsed();
                trace!(
                    "Subscriber of type {:?} deserialized data in {duration:?}",
                    T::ROS_TYPE_NAME
                );
                Some(Ok(p))
            }
            Err(e) => Some(Err(e.into())),
        }
    }
}

pub struct SubscriberAny {
    // Uses Bytes for efficient cloning (reference counted) when there are multiple subscribers
    receiver: broadcast::Receiver<Bytes>,
    _phantom: PhantomData<ShapeShifter>,
}

impl SubscriberAny {
    pub(crate) fn new(receiver: broadcast::Receiver<Bytes>) -> Self {
        Self {
            receiver,
            _phantom: PhantomData,
        }
    }

    /// Gets the next message from the subscriber.
    /// Uniquely for SubscriberAny, this returns the raw bytes of the message as Bytes.
    /// Note: over the wire ros messages include a 4 byte length header before the message body.
    /// This function does not return that header, merely the message body.
    /// The returned Bytes is reference counted and cheap to clone.
    pub async fn next(&mut self) -> Option<Result<Bytes, SubscriberError>> {
        let data = match self.receiver.recv().await {
            Ok(v) => v,
            Err(RecvError::Closed) => return None,
            Err(RecvError::Lagged(n)) => return Some(Err(SubscriberError::Lagged(n))),
        };
        Some(Ok(data))
    }
}

/// Retry configuration constants (matching roscpp behavior)
const INITIAL_RETRY_PERIOD: Duration = Duration::from_millis(100);
const MAX_RETRY_PERIOD: Duration = Duration::from_secs(20);

/// Tracks the state of a connection to a single publisher
struct PublisherConnection {
    /// The cached TCP endpoint (e.g., "192.168.1.1:12345") for direct reconnection
    tcp_endpoint: Option<String>,
    /// Sender to signal the reader task to cancel
    cancel_tx: watch::Sender<bool>,
}

/// Shared state for publisher connections, accessible from spawned tasks
struct PublisherConnectionState {
    /// Map from XMLRPC URI to connection info
    connections: HashMap<String, PublisherConnection>,
}

pub struct Subscription {
    subscription_tasks: Vec<ChildTask<()>>,
    // Uses Bytes for efficient cloning (reference counted) when there are multiple subscribers
    _msg_receiver: broadcast::Receiver<Bytes>,
    msg_sender: broadcast::Sender<Bytes>,
    connection_header: ConnectionHeader,
    /// Shared state tracking all publisher connections
    publisher_state: Arc<RwLock<PublisherConnectionState>>,
}

impl Subscription {
    pub fn new(
        node_name: &Name,
        topic_name: &str,
        topic_type: &str,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    ) -> Self {
        // Using Bytes for efficient cloning (reference counted) when there are multiple subscribers
        let (sender, receiver) = broadcast::channel::<Bytes>(queue_size);
        let connection_header = ConnectionHeader {
            caller_id: node_name.to_string(),
            latching: false,
            msg_definition,
            md5sum: Some(md5sum),
            topic: Some(topic_name.to_owned()),
            topic_type: topic_type.to_owned(),
            tcp_nodelay: false,
            service: None,
            persistent: None,
        };

        Self {
            subscription_tasks: vec![],
            _msg_receiver: receiver,
            msg_sender: sender,
            connection_header,
            publisher_state: Arc::new(RwLock::new(PublisherConnectionState {
                connections: HashMap::new(),
            })),
        }
    }

    pub fn topic_type(&self) -> &str {
        self.connection_header.topic_type.as_str()
    }

    pub fn get_receiver(&self) -> broadcast::Receiver<Bytes> {
        self.msg_sender.subscribe()
    }

    pub async fn add_publisher_source(
        &mut self,
        publisher_uri: &str,
    ) -> Result<(), std::io::Error> {
        // Check if we already have a connection (or retry in progress) for this publisher
        let is_new_connection = {
            !self
                .publisher_state
                .read()
                .await
                .connections
                .contains_key(publisher_uri)
        };

        if is_new_connection {
            // Create cancellation channel for this publisher's task
            let (cancel_tx, cancel_rx) = watch::channel(false);

            // Register this publisher in our state before spawning
            {
                let mut state = self.publisher_state.write().await;
                state.connections.insert(
                    publisher_uri.to_owned(),
                    PublisherConnection {
                        tcp_endpoint: None,
                        cancel_tx,
                    },
                );
            }

            let node_name = self.connection_header.caller_id.clone();
            let topic_name = self.connection_header.topic.as_ref().unwrap().clone();
            let connection_header = self.connection_header.clone();
            let sender = self.msg_sender.clone();
            let publisher_state = self.publisher_state.clone();
            let publisher_uri = publisher_uri.to_owned();

            trace!("Creating new subscription connection for {publisher_uri} on {topic_name}");

            let handle = tokio::spawn(async move {
                publisher_reader_task(
                    cancel_rx,
                    publisher_state,
                    publisher_uri,
                    node_name,
                    topic_name,
                    connection_header,
                    sender,
                )
                .await;
            });
            self.subscription_tasks.push(handle.into());
        }

        Ok(())
    }

    /// Removes publishers that are no longer in the provided list.
    /// This is called when rosmaster sends a publisherUpdate with the current list of publishers.
    /// Any publisher we're tracking that isn't in the new list will be cancelled.
    pub async fn remove_stale_publishers(&mut self, current_publishers: &[String]) {
        let mut state = self.publisher_state.write().await;

        // Find publishers that are no longer in the list
        let stale_uris: Vec<String> = state
            .connections
            .keys()
            .filter(|uri| !current_publishers.iter().any(|p| p == *uri))
            .cloned()
            .collect();

        // Cancel and remove stale publishers
        for uri in stale_uris {
            if let Some(conn) = state.connections.remove(&uri) {
                log::debug!("Publisher {uri} no longer in rosmaster list, cancelling connection");
                // Signal the task to cancel - ignore error if receiver is already dropped
                let _ = conn.cancel_tx.send(true);
            }
        }
    }
}

/// The main reader task for a publisher connection.
/// Handles connection establishment, reading messages, and retry with exponential backoff.
async fn publisher_reader_task(
    mut cancel_rx: watch::Receiver<bool>,
    publisher_state: Arc<RwLock<PublisherConnectionState>>,
    publisher_uri: String,
    node_name: String,
    topic_name: String,
    conn_header: ConnectionHeader,
    sender: broadcast::Sender<Bytes>,
) {
    let mut retry_period = INITIAL_RETRY_PERIOD;
    let mut tcp_endpoint: Option<String> = None;

    'connection_loop: loop {
        // Check for cancellation before attempting connection
        if *cancel_rx.borrow() {
            log::debug!(
                "Publisher reader task for {publisher_uri} cancelled before connection attempt"
            );
            break 'connection_loop;
        }

        // Attempt to establish or re-establish connection
        let stream_result = if let Some(ref endpoint) = tcp_endpoint {
            // Retry: connect directly to cached endpoint (skip XMLRPC)
            log::debug!("Retrying direct connection to {endpoint} for topic {topic_name}");
            connect_and_handshake(endpoint, &conn_header, &topic_name).await
        } else {
            // First connection: go through XMLRPC to get the TCP endpoint
            log::debug!(
                "Establishing initial connection to {publisher_uri} for topic {topic_name}"
            );
            match establish_publisher_connection(
                &node_name,
                &topic_name,
                &publisher_uri,
                conn_header.clone(),
            )
            .await
            {
                Ok((stream, endpoint)) => {
                    // Cache the endpoint for future reconnections
                    tcp_endpoint = Some(endpoint.clone());

                    // Update shared state with the TCP endpoint
                    if let Some(conn) = publisher_state
                        .write()
                        .await
                        .connections
                        .get_mut(&publisher_uri)
                    {
                        conn.tcp_endpoint = Some(endpoint);
                    }

                    Ok(stream)
                }
                Err(e) => Err(e),
            }
        };

        let mut stream = match stream_result {
            Ok(s) => {
                log::info!("Connected to publisher {publisher_uri} for topic {topic_name}");
                retry_period = INITIAL_RETRY_PERIOD; // Reset backoff on successful connection
                s
            }
            Err(e) => {
                log::debug!(
                    "Connection to {publisher_uri} failed: {e}, retrying in {retry_period:?}"
                );

                // Wait with cancellation support
                tokio::select! {
                    biased;
                    _ = cancel_rx.changed() => {
                        if *cancel_rx.borrow() {
                            log::debug!("Publisher reader task for {publisher_uri} cancelled during retry wait");
                            break 'connection_loop;
                        }
                    }
                    _ = tokio::time::sleep(retry_period) => {}
                }

                // Exponential backoff
                retry_period = std::cmp::min(retry_period * 2, MAX_RETRY_PERIOD);
                continue 'connection_loop;
            }
        };

        // Read messages until error or cancellation
        loop {
            tokio::select! {
                biased;
                _ = cancel_rx.changed() => {
                    if *cancel_rx.borrow() {
                        log::debug!("Publisher reader task for {publisher_uri} cancelled during read");
                        break 'connection_loop;
                    }
                }
                result = tcpros::receive_body(&mut stream) => {
                    match result {
                        Ok(body) => {
                            trace!(
                                "Subscription to {topic_name} received message from {publisher_uri}"
                            );
                            if sender.send(body).is_err() {
                                log::error!(
                                    "Unable to send message data due to dropped channel, closing connection to {publisher_uri}"
                                );
                                break 'connection_loop;
                            }
                            // Reset retry period on successful message (connection is healthy)
                            retry_period = INITIAL_RETRY_PERIOD;
                        }
                        Err(e) => {
                            log::debug!(
                                "Read error from {publisher_uri}: {e}, will retry connection"
                            );
                            // Break inner loop to retry connection
                            break;
                        }
                    }
                }
            }
        }

        // After read loop breaks due to error, wait before retry
        log::debug!("Connection to {publisher_uri} lost, retrying in {retry_period:?}");

        tokio::select! {
            biased;
            _ = cancel_rx.changed() => {
                if *cancel_rx.borrow() {
                    log::debug!("Publisher reader task for {publisher_uri} cancelled during post-disconnect retry wait");
                    break 'connection_loop;
                }
            }
            _ = tokio::time::sleep(retry_period) => {}
        }

        // Exponential backoff
        retry_period = std::cmp::min(retry_period * 2, MAX_RETRY_PERIOD);
    }

    // Cleanup: remove from publisher_state when task exits
    log::debug!("Publisher reader task for {publisher_uri} exiting, cleaning up state");
    publisher_state
        .write()
        .await
        .connections
        .remove(&publisher_uri);
}

/// Establishes a connection to a publisher via XMLRPC negotiation.
/// Returns both the TcpStream and the TCP endpoint string for potential reconnection.
async fn establish_publisher_connection(
    node_name: &str,
    topic_name: &str,
    publisher_uri: &str,
    conn_header: ConnectionHeader,
) -> Result<(TcpStream, String), std::io::Error> {
    let tcp_endpoint = send_topic_request(node_name, topic_name, publisher_uri).await?;
    let stream = connect_and_handshake(&tcp_endpoint, &conn_header, topic_name).await?;
    Ok((stream, tcp_endpoint))
}

/// Connects directly to a TCP endpoint and performs the TCPROS handshake.
/// Used for both initial connections and reconnections.
async fn connect_and_handshake(
    tcp_endpoint: &str,
    conn_header: &ConnectionHeader,
    topic_name: &str,
) -> Result<TcpStream, std::io::Error> {
    let mut stream = TcpStream::connect(tcp_endpoint).await?;

    let conn_header_bytes = conn_header.to_bytes(true)?;
    stream.write_all(&conn_header_bytes[..]).await?;

    let Ok(responded_header_bytes) = tcpros::receive_header_bytes(&mut stream).await else {
        // Some ROS tools appear to "probe" where they start a connection just to get the header
        log::trace!("Could not read connection header bytes from endpoint: {tcp_endpoint:?}");
        return Err(std::io::Error::from(std::io::ErrorKind::UnexpectedEof));
    };

    let responded_header = match ConnectionHeader::from_bytes(&responded_header_bytes) {
        Ok(header) => header,
        Err(e) => {
            log::error!("Could not parse connection header data sent by publisher: {e:?}");
            return Err(std::io::Error::from(std::io::ErrorKind::InvalidData));
        }
    };

    if conn_header.md5sum == Some("*".to_string())
        || responded_header.md5sum == Some("*".to_string())
        || conn_header.md5sum == responded_header.md5sum
    {
        log::debug!(
            "Established connection with publisher for {:?}",
            conn_header.topic
        );
        Ok(stream)
    } else {
        log::error!(
            "Tried to subscribe to {}, but md5sums do not match. Expected {:?}, received {:?}",
            topic_name,
            conn_header.md5sum,
            responded_header.md5sum
        );
        Err(std::io::ErrorKind::InvalidData)
    }
    .map_err(std::io::Error::from)
}

async fn send_topic_request(
    node_name: &str,
    topic_name: &str,
    publisher_uri: &str,
) -> Result<String, std::io::Error> {
    let xmlrpc_client = reqwest::Client::new();
    let body = serde_xmlrpc::request_to_string(
        "requestTopic",
        vec![
            node_name.into(),
            topic_name.into(),
            serde_xmlrpc::Value::Array(vec![serde_xmlrpc::Value::Array(vec!["TCPROS".into()])]),
        ],
    )
    .unwrap();

    let response = xmlrpc_client
        .post(publisher_uri)
        .body(body)
        .send()
        .await
        .map_err(|err| {
            log::error!("Unable to send subscribe request to publisher: {err}");
            std::io::ErrorKind::ConnectionAborted
        })?;
    if response.status().is_success() {
        if let Ok(response_data) = response.text().await {
            if let Ok((_code, _description, (protocol, hostname, port))) =
                serde_xmlrpc::response_from_str::<(i8, String, (String, String, u16))>(
                    &response_data,
                )
            {
                if protocol == "TCPROS" {
                    let tcpros_endpoint = format!("{hostname}:{port}");
                    log::debug!("Got a TCPROS publisher endpoint at {tcpros_endpoint}");
                    Ok(tcpros_endpoint)
                } else {
                    log::error!("Got unsupported protocol {protocol}");
                    Err(std::io::ErrorKind::Unsupported.into())
                }
            } else {
                log::error!("Failed to deserialize requestTopic response {response_data}");
                Err(std::io::ErrorKind::InvalidData.into())
            }
        } else {
            log::error!("No data received with the response");
            Err(std::io::ErrorKind::InvalidData.into())
        }
    } else {
        log::error!(
            "Failed to request topic data from the publisher's XMLRPC server for {publisher_uri}: {response:#?}"
        );
        Err(std::io::ErrorKind::ConnectionRefused.into())
    }
}

#[derive(thiserror::Error, Debug)]
pub enum SubscriberError {
    /// Deserialize Error from `serde_rosmsg::Error` (stored as String because of dyn Error)
    #[error("serde_rosmsg Error: {0}")]
    DeserializeError(String),
    #[error("you are too slow, {0} messages were skipped")]
    Lagged(u64),
}

impl From<roslibrust_serde_rosmsg::Error> for SubscriberError {
    fn from(value: roslibrust_serde_rosmsg::Error) -> Self {
        Self::DeserializeError(value.to_string())
    }
}
