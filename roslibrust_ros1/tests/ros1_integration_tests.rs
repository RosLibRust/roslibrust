//! Comprehensive integration tests for ROS1 functionality
//!
//! These tests require a running ROS master at localhost:11311
//! Run with: cargo test --package roslibrust_ros1 --features ros1_test

#[cfg(feature = "ros1_test")]
mod tests {
    use roslibrust_ros1::{MasterClient, NodeHandle};
    use roslibrust_test::ros1::{std_msgs, std_srvs};
    use tokio::time::timeout;

    // ============================================================================
    // SECTION: Publisher/Subscriber Tests
    // ============================================================================

    #[test_log::test(tokio::test)]
    async fn test_publish_subscribe_typed() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_pub_sub_typed")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::String>("/test_pub_sub_typed_topic", 1, false)
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe::<std_msgs::String>("/test_pub_sub_typed_topic", 1)
            .await
            .unwrap();

        // Give time for connection
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        publisher
            .publish(&std_msgs::String {
                data: "hello world".to_owned(),
            })
            .await
            .unwrap();

        let res = timeout(tokio::time::Duration::from_millis(500), subscriber.next()).await;
        let msg = res.unwrap().unwrap().unwrap();
        assert_eq!(msg.data, "hello world");
    }

    #[test_log::test(tokio::test)]
    async fn test_publish_any() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_publish_any")
            .await
            .unwrap();

        let publisher = nh
            .advertise_any(
                "/test_publish_any_topic",
                "std_msgs/String",
                "string data\n",
                1,
                true,
            )
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe::<std_msgs::String>("/test_publish_any_topic", 1)
            .await
            .unwrap();

        // Message body for "test" (without overall length header)
        let msg_raw: Vec<u8> = vec![8, 0, 0, 0, 4, 0, 0, 0, 116, 101, 115, 116];
        publisher.publish(msg_raw).await.unwrap();

        let res = timeout(tokio::time::Duration::from_millis(250), subscriber.next()).await;
        let msg = res.unwrap().unwrap().unwrap();
        assert_eq!(msg.data, "test");
    }

    #[test_log::test(tokio::test)]
    async fn test_subscribe_any() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_subscribe_any")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::String>("/test_subscribe_any_topic", 1, true)
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe_any("/test_subscribe_any_topic", 1)
            .await
            .unwrap();

        publisher
            .publish(&std_msgs::String {
                data: "test".to_owned(),
            })
            .await
            .unwrap();

        let res = timeout(tokio::time::Duration::from_millis(250), subscriber.next()).await;
        let received = res.unwrap().unwrap().unwrap();
        // Expected: [field_length, "test"]
        assert_eq!(received, vec![8, 0, 0, 0, 4, 0, 0, 0, 116, 101, 115, 116]);
    }

    #[test_log::test(tokio::test)]
    async fn test_latching_publisher() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_latching")
            .await
            .unwrap();

        // Create a latching publisher
        let publisher = nh
            .advertise::<std_msgs::String>("/test_latching_topic", 1, true)
            .await
            .unwrap();

        // Publish message before subscriber exists
        publisher
            .publish(&std_msgs::String {
                data: "latched message".to_owned(),
            })
            .await
            .unwrap();

        // Create subscriber after message was published
        let mut subscriber = nh
            .subscribe::<std_msgs::String>("/test_latching_topic", 1)
            .await
            .unwrap();

        // Should receive the latched message
        let msg = subscriber.next().await.unwrap().unwrap();
        assert_eq!(msg.data, "latched message");
    }

    #[test_log::test(tokio::test)]
    async fn test_non_latching_publisher() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_not_latching")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::String>("/test_not_latching_topic", 1, false)
            .await
            .unwrap();

        // Publish before subscriber exists
        publisher
            .publish(&std_msgs::String {
                data: "test".to_owned(),
            })
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe::<std_msgs::String>("/test_not_latching_topic", 1)
            .await
            .unwrap();

        // Should timeout (message was not latched)
        let res = timeout(tokio::time::Duration::from_millis(250), subscriber.next()).await;
        assert!(res.is_err());
    }

    #[test_log::test(tokio::test)]
    async fn test_large_payload() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_large_payload")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::UInt8MultiArray>("/test_large_payload_topic", 1, false)
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe::<std_msgs::UInt8MultiArray>("/test_large_payload_topic", 1)
            .await
            .unwrap();

        // Give time for connection
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // Test with 10KB payload (larger than typical MTU)
        for _i in 0..5 {
            let test_data = vec![42u8; 10_000];
            publisher
                .publish(&std_msgs::UInt8MultiArray {
                    layout: std_msgs::MultiArrayLayout::default(),
                    data: test_data.clone(),
                })
                .await
                .unwrap();

            let msg = subscriber.next().await.unwrap().unwrap();
            assert_eq!(msg.data, test_data);
        }
    }

    // ============================================================================
    // SECTION: Service Tests
    // ============================================================================

    #[test_log::test(tokio::test)]
    async fn test_basic_service() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_basic_service")
            .await
            .unwrap();

        // Advertise a simple service
        let _server = nh
            .advertise_service::<std_srvs::SetBool, _>("/test_basic_service_srv", |req| {
                Ok(std_srvs::SetBoolResponse {
                    success: req.data,
                    message: if req.data {
                        "enabled".to_string()
                    } else {
                        "disabled".to_string()
                    },
                })
            })
            .await
            .unwrap();

        // Give time for service to register
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // Create client and call service
        let client = nh
            .service_client::<std_srvs::SetBool>("/test_basic_service_srv")
            .await
            .unwrap();

        let response = client
            .call(&std_srvs::SetBoolRequest { data: true })
            .await
            .unwrap();

        assert!(response.success);
        assert_eq!(response.message, "enabled");
    }

    #[test_log::test(tokio::test)]
    async fn test_service_multiple_calls() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_service_multiple_calls")
            .await
            .unwrap();

        let _server = nh
            .advertise_service::<std_srvs::Trigger, _>("/test_multiple_calls_srv", |_req| {
                Ok(std_srvs::TriggerResponse {
                    success: true,
                    message: "called".to_string(),
                })
            })
            .await
            .unwrap();

        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        let client = nh
            .service_client::<std_srvs::Trigger>("/test_multiple_calls_srv")
            .await
            .unwrap();

        // Call the service multiple times
        for _ in 0..10 {
            let response = client.call(&std_srvs::TriggerRequest {}).await.unwrap();
            assert!(response.success);
        }
    }

    #[test_log::test(tokio::test)]
    async fn test_service_error() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_service_error")
            .await
            .unwrap();

        let _server = nh
            .advertise_service::<std_srvs::Trigger, _>("/test_service_error_srv", |_req| {
                Err(std::io::Error::new(std::io::ErrorKind::Other, "intentional error").into())
            })
            .await
            .unwrap();

        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        let client = nh
            .service_client::<std_srvs::Trigger>("/test_service_error_srv")
            .await
            .unwrap();

        let result = client.call(&std_srvs::TriggerRequest {}).await;
        assert!(result.is_err());
    }

    #[test_log::test(tokio::test)]
    async fn test_service_not_found() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_service_not_found")
            .await
            .unwrap();

        let result = nh
            .service_client::<std_srvs::Trigger>("/nonexistent_service")
            .await;

        assert!(result.is_err());
    }

    // ============================================================================
    // SECTION: Drop/Cleanup Tests
    // ============================================================================

    #[test_log::test(tokio::test)]
    async fn test_dropping_publisher_unregisters() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_drop_publisher")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::Header>("/test_drop_publisher_topic", 1, false)
            .await
            .unwrap();

        let master_client =
            MasterClient::new("http://localhost:11311", "NAN", "/test_drop_publisher_mc")
                .await
                .unwrap();

        let before = master_client.get_published_topics("").await.unwrap();
        assert!(before.contains(&(
            "/test_drop_publisher_topic".to_string(),
            "std_msgs/Header".to_string()
        )));

        // Drop the publisher
        std::mem::drop(publisher);
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // Verify it's unregistered
        let after = master_client.get_published_topics("").await.unwrap();
        assert!(!after.contains(&(
            "/test_drop_publisher_topic".to_string(),
            "std_msgs/Header".to_string()
        )));
    }

    #[test_log::test(tokio::test)]
    async fn test_dropping_subscriber_unregisters() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_drop_subscriber")
            .await
            .unwrap();

        let _publisher = nh
            .advertise::<std_msgs::String>("/test_drop_subscriber_topic", 1, false)
            .await
            .unwrap();

        let subscriber = nh
            .subscribe::<std_msgs::String>("/test_drop_subscriber_topic", 1)
            .await
            .unwrap();

        let master_client =
            MasterClient::new("http://localhost:11311", "NAN", "/test_drop_subscriber_mc")
                .await
                .unwrap();

        let before = master_client.get_system_state().await.unwrap();
        assert!(before.is_subscribed("/test_drop_subscriber_topic", "/test_drop_subscriber"));

        // Drop the subscriber
        std::mem::drop(subscriber);
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        let after = master_client.get_system_state().await.unwrap();
        assert!(!after.is_subscribed("/test_drop_subscriber_topic", "/test_drop_subscriber"));
    }

    #[test_log::test(tokio::test)]
    async fn test_dropping_service_server_unregisters() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_drop_service")
            .await
            .unwrap();

        let server = nh
            .advertise_service::<std_srvs::Trigger, _>("/test_drop_service_srv", |_req| {
                Ok(std_srvs::TriggerResponse {
                    success: true,
                    message: "ok".to_string(),
                })
            })
            .await
            .unwrap();

        // Verify service works
        let client = nh
            .service_client::<std_srvs::Trigger>("/test_drop_service_srv")
            .await
            .unwrap();
        let _response = client.call(&std_srvs::TriggerRequest {}).await.unwrap();

        // Drop the server
        std::mem::drop(server);
        tokio::time::sleep(tokio::time::Duration::from_millis(250)).await;

        // Should fail to call
        let result = client.call(&std_srvs::TriggerRequest {}).await;
        assert!(result.is_err());

        // Should fail to create new client
        let client2 = nh
            .service_client::<std_srvs::Trigger>("/test_drop_service_srv")
            .await;
        assert!(client2.is_err());
    }

    #[test_log::test(tokio::test)]
    async fn test_multiple_publishers_partial_drop() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_multi_pub")
            .await
            .unwrap();

        let publisher1 = nh
            .advertise::<std_msgs::String>("/test_multi_pub_topic", 1, false)
            .await
            .unwrap();

        let publisher2 = nh
            .advertise::<std_msgs::String>("/test_multi_pub_topic", 1, false)
            .await
            .unwrap();

        let master_client =
            MasterClient::new("http://localhost:11311", "NAN", "/test_multi_pub_mc")
                .await
                .unwrap();

        // Drop first publisher
        std::mem::drop(publisher1);
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // Topic should still be advertised
        let topics = master_client.get_published_topics("").await.unwrap();
        assert!(topics.contains(&(
            "/test_multi_pub_topic".to_string(),
            "std_msgs/String".to_string()
        )));

        // publisher2 should still work
        publisher2
            .publish(&std_msgs::String {
                data: "still works".to_string(),
            })
            .await
            .unwrap();
    }

    #[test_log::test(tokio::test)]
    async fn test_multiple_subscribers_partial_drop() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_multi_sub")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::String>("/test_multi_sub_topic", 1, false)
            .await
            .unwrap();

        let subscriber1 = nh
            .subscribe::<std_msgs::String>("/test_multi_sub_topic", 1)
            .await
            .unwrap();

        let mut subscriber2 = nh
            .subscribe::<std_msgs::String>("/test_multi_sub_topic", 1)
            .await
            .unwrap();

        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        publisher
            .publish(&std_msgs::String {
                data: "test1".to_string(),
            })
            .await
            .unwrap();

        // Drop first subscriber
        std::mem::drop(subscriber1);
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // Publish another message
        publisher
            .publish(&std_msgs::String {
                data: "test2".to_string(),
            })
            .await
            .unwrap();

        // subscriber2 should still receive messages
        let msg = timeout(tokio::time::Duration::from_millis(500), subscriber2.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();

        assert!(msg.data == "test1" || msg.data == "test2");
    }

    #[test_log::test(tokio::test)]
    async fn test_node_full_cleanup() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_node_cleanup")
            .await
            .unwrap();

        let _publisher = nh
            .advertise::<std_msgs::Header>("/test_cleanup_pub", 1, false)
            .await
            .unwrap();

        let _subscriber = nh
            .subscribe::<std_msgs::Header>("/test_cleanup_sub", 1)
            .await
            .unwrap();

        let _service = nh
            .advertise_service::<std_srvs::Trigger, _>("/test_cleanup_srv", |_| {
                Ok(std_srvs::TriggerResponse::default())
            })
            .await
            .unwrap();

        let master_client = MasterClient::new("http://localhost:11311", "NAN", "/test_cleanup_mc")
            .await
            .unwrap();

        let before = master_client.get_system_state().await.unwrap();
        assert!(before.is_publishing("/test_cleanup_pub", "/test_node_cleanup"));
        assert!(before.is_subscribed("/test_cleanup_sub", "/test_node_cleanup"));
        assert!(before.is_service_provider("/test_cleanup_srv", "/test_node_cleanup"));

        // Drop the entire node
        std::mem::drop(nh);

        let after = master_client.get_system_state().await.unwrap();
        assert!(!after.is_publishing("/test_cleanup_pub", "/test_node_cleanup"));
        assert!(!after.is_subscribed("/test_cleanup_sub", "/test_node_cleanup"));
        assert!(!after.is_service_provider("/test_cleanup_srv", "/test_node_cleanup"));
    }
}
