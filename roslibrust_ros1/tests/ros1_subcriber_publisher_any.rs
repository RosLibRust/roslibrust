//! Integration test for PublisherAny and SubscriberAny round-trip communication

#[cfg(feature = "ros1_test")]
mod tests {
    use roslibrust_ros1::NodeHandle;
    use tokio::time::timeout;

    /// Test round-trip publish and subscribe using PublisherAny and SubscriberAny
    /// This test verifies that:
    /// 1. A PublisherAny can publish raw bytes to a topic
    /// 2. A SubscriberAny can receive those raw bytes from the same topic
    /// 3. The received bytes match the published bytes exactly
    #[test_log::test(tokio::test)]
    async fn test_publisher_any_subscriber_any_roundtrip() {
        // Create a node handle
        let nh = NodeHandle::new(
            "http://localhost:11311",
            "test_publisher_any_subscriber_any",
        )
        .await
        .expect("Failed to create node handle");

        // Create a PublisherAny for std_msgs/String
        let publisher = nh
            .advertise_any(
                "/test_roundtrip",
                "std_msgs/String",
                "string data\n",
                1,
                true,
            )
            .await
            .expect("Failed to create publisher");

        // Create a SubscriberAny for the same topic
        let mut subscriber = nh
            .subscribe_any("/test_roundtrip", 1)
            .await
            .expect("Failed to create subscriber");

        // Prepare test data: serialized std_msgs/String with data="hello"
        // Note: API doesn't consider the overall message length header as part of message
        // Overall length header will be added automatically
        // Format: [field_length (4 bytes), data (5 bytes)]
        let test_message: Vec<u8> = vec![
            0x05, 0x00, 0x00, 0x00, // field length = 5
            0x68, 0x65, 0x6c, 0x6c, 0x6f, // "hello"
        ];

        // Publish the message
        publisher
            .publish(&test_message)
            .await
            .expect("Failed to publish message");

        // Subscribe and receive the message with a timeout
        let receive_result =
            timeout(std::time::Duration::from_millis(500), subscriber.next()).await;

        // Verify we received the message
        let received_message = receive_result
            .expect("Timeout waiting for message")
            .expect("Subscriber returned None")
            .expect("Failed to receive message");

        // Verify the received bytes match the published bytes
        assert_eq!(
            received_message, test_message,
            "Received message does not match published message"
        );
    }
}
