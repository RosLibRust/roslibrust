//! Integration tests for roslibrust_transforms.
//!
//! These tests require a running rosbridge server and are gated behind feature flags.
//! Run with:
//! - `cargo test -p roslibrust_transforms --features ros1_test` for ROS1 testing
//! - `cargo test -p roslibrust_transforms --features ros2_test` for ROS2 testing
//!
//! Use the docker-compose files in the docker folder to start a rosbridge server.

#[cfg(test)]
#[cfg(feature = "running_bridge")]
#[cfg(not(all(feature = "ros1_test", feature = "ros2_test")))]
mod integration_tests {
    use std::time::Duration;

    use roslibrust_rosbridge::{ClientHandle, ClientHandleOptions};
    use tokio::time::timeout;
    use transforms::time::Timestamp;

    const TIMEOUT: Duration = Duration::from_millis(2000);
    const LOCAL_WS: &str = "ws://localhost:9090";

    // Import the appropriate message types based on feature flags
    #[cfg(feature = "ros1_test")]
    use crate::messages::ros1::{geometry_msgs, std_msgs, TFMessage};

    #[cfg(feature = "ros2_test")]
    use crate::messages::ros2::{builtin_interfaces, geometry_msgs, std_msgs, TFMessage};

    /// Helper function to create a TransformStamped message for ROS1
    #[cfg(feature = "ros1_test")]
    fn create_transform_stamped(
        parent_frame: &str,
        child_frame: &str,
        x: f64,
        y: f64,
        z: f64,
        secs: i32,
        nsecs: i32,
    ) -> geometry_msgs::TransformStamped {
        geometry_msgs::TransformStamped {
            header: std_msgs::Header {
                seq: 0,
                stamp: roslibrust::codegen::integral_types::Time { secs, nsecs },
                frame_id: parent_frame.to_string(),
            },
            child_frame_id: child_frame.to_string(),
            transform: geometry_msgs::Transform {
                translation: geometry_msgs::Vector3 { x, y, z },
                rotation: geometry_msgs::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }

    /// Helper function to create a TransformStamped message for ROS2
    #[cfg(feature = "ros2_test")]
    fn create_transform_stamped(
        parent_frame: &str,
        child_frame: &str,
        x: f64,
        y: f64,
        z: f64,
        sec: i32,
        nanosec: u32,
    ) -> geometry_msgs::TransformStamped {
        geometry_msgs::TransformStamped {
            header: std_msgs::Header {
                stamp: builtin_interfaces::Time { sec, nanosec },
                frame_id: parent_frame.to_string(),
            },
            child_frame_id: child_frame.to_string(),
            transform: geometry_msgs::Transform {
                translation: geometry_msgs::Vector3 { x, y, z },
                rotation: geometry_msgs::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }

    /// Test that we can connect to rosbridge and publish/subscribe to /tf
    #[test_log::test(tokio::test)]
    async fn test_tf_round_trip() {
        let client = timeout(
            TIMEOUT,
            ClientHandle::new_with_options(ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT)),
        )
        .await
        .expect("Failed to connect in time")
        .expect("Failed to create client");

        // Advertise on /tf
        let publisher = timeout(TIMEOUT, client.advertise::<TFMessage>("/tf_test"))
            .await
            .expect("Failed to advertise in time")
            .expect("Failed to advertise");

        // Subscribe to /tf
        let subscriber = timeout(TIMEOUT, client.subscribe::<TFMessage>("/tf_test"))
            .await
            .expect("Failed to subscribe in time")
            .expect("Failed to subscribe");

        // Allow subscription to complete
        tokio::time::sleep(Duration::from_millis(200)).await;

        // Create a test message
        #[cfg(feature = "ros1_test")]
        let tf_stamped = create_transform_stamped("world", "base_link", 1.0, 2.0, 3.0, 100, 0);
        #[cfg(feature = "ros2_test")]
        let tf_stamped = create_transform_stamped("world", "base_link", 1.0, 2.0, 3.0, 100, 0);

        let msg = TFMessage {
            transforms: vec![tf_stamped],
        };

        // Publish the message
        timeout(TIMEOUT, publisher.publish(&msg))
            .await
            .expect("Failed to publish in time")
            .expect("Failed to publish");

        // Receive the message
        let received = timeout(TIMEOUT, subscriber.next())
            .await
            .expect("Failed to receive in time");

        assert_eq!(received.transforms.len(), 1);
        assert_eq!(received.transforms[0].header.frame_id, "world");
        assert_eq!(received.transforms[0].child_frame_id, "base_link");
        assert_eq!(received.transforms[0].transform.translation.x, 1.0);
        assert_eq!(received.transforms[0].transform.translation.y, 2.0);
        assert_eq!(received.transforms[0].transform.translation.z, 3.0);
    }

    /// Test using TransformManager with rosbridge backend (ROS1)
    #[cfg(feature = "ros1_test")]
    #[test_log::test(tokio::test)]
    async fn test_transform_listener_ros1() {
        use crate::{Ros1TFMessage, TransformManager};

        let client = timeout(
            TIMEOUT,
            ClientHandle::new_with_options(ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT)),
        )
        .await
        .expect("Failed to connect in time")
        .expect("Failed to create client");

        // Create TransformManager with ROS1 messages
        let manager = timeout(TIMEOUT, TransformManager::<Ros1TFMessage, _>::new(&client))
            .await
            .expect("Failed to create manager in time")
            .expect("Failed to create manager");

        // Publish a transform using rosbridge client
        let publisher = timeout(TIMEOUT, client.advertise::<TFMessage>("/tf"))
            .await
            .expect("Failed to advertise in time")
            .expect("Failed to advertise");

        // Allow listener subscription to complete
        tokio::time::sleep(Duration::from_millis(500)).await;

        let secs = 200i32;
        let nsecs = 500_000_000i32;
        let tf_stamped = create_transform_stamped("world", "sensor", 5.0, 6.0, 7.0, secs, nsecs);
        let msg = TFMessage {
            transforms: vec![tf_stamped],
        };

        timeout(TIMEOUT, publisher.publish(&msg))
            .await
            .expect("Failed to publish in time")
            .expect("Failed to publish");

        // Wait for message to be processed
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Create the timestamp matching what we published
        let timestamp = Timestamp {
            t: (secs as u128) * 1_000_000_000 + (nsecs as u128),
        };

        // Look up the transform
        let transform = timeout(
            TIMEOUT,
            manager.lookup_transform("world", "sensor", timestamp),
        )
        .await
        .expect("Lookup timed out")
        .expect("Failed to lookup transform");

        assert_eq!(transform.parent, "world");
        assert_eq!(transform.child, "sensor");
        assert!((transform.translation.x - 5.0).abs() < 0.001);
        assert!((transform.translation.y - 6.0).abs() < 0.001);
        assert!((transform.translation.z - 7.0).abs() < 0.001);
    }

    /// Test using TransformManager with rosbridge backend (ROS2)
    #[cfg(feature = "ros2_test")]
    #[test_log::test(tokio::test)]
    async fn test_transform_listener_ros2() {
        use crate::{Ros2TFMessage, TransformManager};

        let client = timeout(
            TIMEOUT,
            ClientHandle::new_with_options(ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT)),
        )
        .await
        .expect("Failed to connect in time")
        .expect("Failed to create client");

        // Create TransformManager using ROS2 messages
        let manager = timeout(
            TIMEOUT,
            TransformManager::<Ros2TFMessage, _>::with_buffer_duration(
                &client,
                Duration::from_secs(10),
            ),
        )
        .await
        .expect("Failed to create manager in time")
        .expect("Failed to create manager");

        // Publish a transform using rosbridge client
        let publisher = timeout(TIMEOUT, client.advertise::<TFMessage>("/tf"))
            .await
            .expect("Failed to advertise in time")
            .expect("Failed to advertise");

        // Allow manager subscription to complete
        tokio::time::sleep(Duration::from_millis(500)).await;

        let sec = 200i32;
        let nanosec = 500_000_000u32;
        let tf_stamped = create_transform_stamped("world", "sensor", 5.0, 6.0, 7.0, sec, nanosec);
        let msg = TFMessage {
            transforms: vec![tf_stamped],
        };

        timeout(TIMEOUT, publisher.publish(&msg))
            .await
            .expect("Failed to publish in time")
            .expect("Failed to publish");

        // Wait for message to be processed
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Create the timestamp matching what we published
        let timestamp = Timestamp {
            t: (sec as u128) * 1_000_000_000 + (nanosec as u128),
        };

        // Look up the transform
        let transform = timeout(
            TIMEOUT,
            manager.lookup_transform("world", "sensor", timestamp),
        )
        .await
        .expect("Lookup timed out")
        .expect("Failed to lookup transform");

        assert_eq!(transform.parent, "world");
        assert_eq!(transform.child, "sensor");
        assert!((transform.translation.x - 5.0).abs() < 0.001);
        assert!((transform.translation.y - 6.0).abs() < 0.001);
        assert!((transform.translation.z - 7.0).abs() < 0.001);
    }
}
