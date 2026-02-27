//! Tests for subscriber reconnection behavior.
//!
//! These tests verify that subscribers correctly reconnect to publishers
//! when TCP connections are broken, matching roscpp's behavior.

#[cfg(feature = "ros1_test")]
mod tests {
    use roslibrust_common::RosMessageType;
    use roslibrust_test::ros1::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::TcpListener;

    // TODO a lot of the helpers in this file could be made more generic to help other testing efforts

    const TOPIC_TYPE: &str = "std_msgs/String";
    const MD5SUM: &str = roslibrust_test::ros1::std_msgs::String::MD5SUM;

    /// Helper to build a simple connection header for our mock publisher
    fn build_connection_header(
        caller_id: &str,
        topic: &str,
        topic_type: &str,
        md5sum: &str,
    ) -> Vec<u8> {
        use byteorder::{LittleEndian, WriteBytesExt};

        let mut header_data = Vec::with_capacity(1024);
        // Skip the length header for now
        WriteBytesExt::write_u32::<LittleEndian>(&mut header_data, 0).unwrap();

        let fields = [
            format!("callerid={}", caller_id),
            "latching=0".to_string(),
            format!("md5sum={}", md5sum),
            format!("topic={}", topic),
            format!("type={}", topic_type),
            "message_definition=string data\n".to_string(),
        ];

        for field in &fields {
            WriteBytesExt::write_u32::<LittleEndian>(&mut header_data, field.len() as u32).unwrap();
            std::io::Write::write_all(&mut header_data, field.as_bytes()).unwrap();
        }

        // Write the actual length at the beginning
        let len = (header_data.len() - 4) as u32;
        header_data[0..4].copy_from_slice(&len.to_le_bytes());
        header_data
    }

    /// Helper to serialize a std_msgs/String message
    fn serialize_string_msg(data: &str) -> Vec<u8> {
        use byteorder::{LittleEndian, WriteBytesExt};

        let mut msg = Vec::new();
        // String field length
        WriteBytesExt::write_u32::<LittleEndian>(&mut msg, data.len() as u32).unwrap();
        std::io::Write::write_all(&mut msg, data.as_bytes()).unwrap();

        // Wrap with overall message length
        let mut full_msg = Vec::new();
        WriteBytesExt::write_u32::<LittleEndian>(&mut full_msg, msg.len() as u32).unwrap();
        full_msg.extend(msg);
        full_msg
    }

    /// A mock publisher that can be controlled programmatically for testing.
    struct MockPublisher {
        tcp_listener: TcpListener,
        xmlrpc_handle: tokio::task::JoinHandle<()>,
        master_client: roslibrust_ros1::MasterClient,
        topic: String,
    }

    impl MockPublisher {
        async fn new(topic: &str) -> Self {
            // Start TCP listener
            let tcp_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
            let tcp_port = tcp_listener.local_addr().unwrap().port();

            // Create XMLRPC server
            let xmlrpc_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
            let xmlrpc_port = xmlrpc_listener.local_addr().unwrap().port();
            let xmlrpc_uri = format!("http://127.0.0.1:{}", xmlrpc_port);

            // Spawn XMLRPC server task
            let tcp_port_copy = tcp_port;
            let xmlrpc_handle = tokio::spawn(async move {
                loop {
                    let Ok((mut stream, _)) = xmlrpc_listener.accept().await else {
                        break;
                    };
                    let mut buf = vec![0u8; 4096];
                    let Ok(n) = stream.read(&mut buf).await else {
                        continue;
                    };
                    let request = String::from_utf8_lossy(&buf[..n]);

                    if request.contains("requestTopic") {
                        let response = format!(
                            r#"<?xml version="1.0"?>
<methodResponse>
<params>
<param><value><array><data>
<value><i4>1</i4></value>
<value><string>ready</string></value>
<value><array><data>
<value><string>TCPROS</string></value>
<value><string>127.0.0.1</string></value>
<value><i4>{}</i4></value>
</data></array></value>
</data></array></value></param>
</params>
</methodResponse>"#,
                            tcp_port_copy
                        );

                        let http_response = format!(
                            "HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nContent-Length: {}\r\n\r\n{}",
                            response.len(),
                            response
                        );
                        let _ = stream.write_all(http_response.as_bytes()).await;
                    }
                }
            });

            // Register with rosmaster
            let master_client = roslibrust_ros1::MasterClient::new(
                "http://localhost:11311",
                &xmlrpc_uri,
                &format!("/mock_publisher_{}", topic.replace('/', "_")),
            )
            .await
            .unwrap();

            master_client
                .register_publisher(topic, TOPIC_TYPE)
                .await
                .unwrap();

            Self {
                tcp_listener,
                xmlrpc_handle,
                master_client,
                topic: topic.to_string(),
            }
        }

        /// Accept a subscriber connection and perform TCPROS handshake
        async fn accept_subscriber(&self) -> tokio::net::TcpStream {
            let (mut stream, _) = self.tcp_listener.accept().await.unwrap();

            // Read subscriber's connection header
            let mut header_len_bytes = [0u8; 4];
            stream.read_exact(&mut header_len_bytes).await.unwrap();
            let header_len = u32::from_le_bytes(header_len_bytes) as usize;
            let mut header_bytes = vec![0u8; header_len];
            stream.read_exact(&mut header_bytes).await.unwrap();

            // Send publisher connection header
            let response_header = build_connection_header(
                &format!("/mock_publisher_{}", self.topic.replace('/', "_")),
                &self.topic,
                TOPIC_TYPE,
                MD5SUM,
            );
            stream.write_all(&response_header).await.unwrap();

            stream
        }

        /// Accept with timeout
        async fn accept_subscriber_timeout(
            &self,
            timeout: std::time::Duration,
        ) -> Option<tokio::net::TcpStream> {
            match tokio::time::timeout(timeout, self.accept_subscriber()).await {
                Ok(stream) => Some(stream),
                Err(_) => None,
            }
        }

        /// Send a message on an existing stream
        async fn send_message(stream: &mut tokio::net::TcpStream, data: &str) {
            let msg = serialize_string_msg(data);
            stream.write_all(&msg).await.unwrap();
        }

        async fn cleanup(self) {
            self.xmlrpc_handle.abort();
            let _ = self.master_client.unregister_publisher(&self.topic).await;
        }
    }

    /// Test that a subscriber reconnects after the TCP connection to a publisher is broken.
    ///
    /// This test creates a "mock publisher" at the TCP level that:
    /// 1. Registers with rosmaster as a publisher
    /// 2. Accepts TCP connections from subscribers
    /// 3. Performs the TCPROS handshake
    /// 4. Sends a message
    /// 5. Forcibly closes the TCP connection (simulating network failure)
    /// 6. Accepts a new connection (the subscriber should reconnect)
    /// 7. Sends another message
    #[test_log::test(tokio::test)]
    async fn test_subscriber_reconnects_after_tcp_disconnect() {
        const TOPIC: &str = "/test_reconnect_basic";

        let mock_pub = MockPublisher::new(TOPIC).await;

        // Create subscriber
        let sub_nh = roslibrust_ros1::NodeHandle::new(
            "http://localhost:11311",
            "/test_reconnect_basic_subscriber",
        )
        .await
        .unwrap();

        let mut subscriber = sub_nh
            .subscribe::<std_msgs::String>(TOPIC, 1)
            .await
            .unwrap();

        // Accept first connection and send message
        let mut stream = mock_pub.accept_subscriber().await;
        MockPublisher::send_message(&mut stream, "message_before_disconnect").await;

        // Verify subscriber receives it
        let received = tokio::time::timeout(std::time::Duration::from_secs(2), subscriber.next())
            .await
            .expect("Timeout waiting for first message")
            .expect("Subscriber returned None")
            .expect("Failed to deserialize message");
        assert_eq!(received.data, "message_before_disconnect");

        // Break the TCP connection
        drop(stream);

        // Wait for subscriber to reconnect
        let mut new_stream = mock_pub
            .accept_subscriber_timeout(std::time::Duration::from_secs(5))
            .await
            .expect("Subscriber did not reconnect within 5 seconds");

        // Send second message
        MockPublisher::send_message(&mut new_stream, "message_after_reconnect").await;

        // Verify subscriber receives it
        let received = tokio::time::timeout(std::time::Duration::from_secs(2), subscriber.next())
            .await
            .expect("Timeout waiting for second message")
            .expect("Subscriber returned None")
            .expect("Failed to deserialize message");
        assert_eq!(received.data, "message_after_reconnect");

        mock_pub.cleanup().await;
    }

    /// Test that multiple sequential disconnects/reconnects work correctly.
    #[test_log::test(tokio::test)]
    async fn test_multiple_reconnections() {
        const TOPIC: &str = "/test_multiple_reconnect";
        const NUM_CYCLES: usize = 3;

        let mock_pub = MockPublisher::new(TOPIC).await;

        // Create subscriber
        let sub_nh = roslibrust_ros1::NodeHandle::new(
            "http://localhost:11311",
            "/test_multiple_reconnect_subscriber",
        )
        .await
        .unwrap();

        let mut subscriber = sub_nh
            .subscribe::<std_msgs::String>(TOPIC, 1)
            .await
            .unwrap();

        for i in 0..NUM_CYCLES {
            // Accept connection
            let mut stream = mock_pub.accept_subscriber().await;

            // Send message
            let msg_data = format!("message_{}", i);
            MockPublisher::send_message(&mut stream, &msg_data).await;

            // Verify receipt
            let received =
                tokio::time::timeout(std::time::Duration::from_secs(2), subscriber.next())
                    .await
                    .expect(&format!("Timeout waiting for message {}", i))
                    .expect("Subscriber returned None")
                    .expect("Failed to deserialize message");
            assert_eq!(received.data, msg_data);

            // Break connection
            drop(stream);

            // Small delay to ensure the subscriber detects the disconnect
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }

        mock_pub.cleanup().await;
    }

}
