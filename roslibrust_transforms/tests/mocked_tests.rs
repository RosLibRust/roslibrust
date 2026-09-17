//! Unit tests for roslibrust_transforms using the MockRos backend.

use std::time::Duration;

use roslibrust_common::{Publish, Subscribe, TopicProvider};
use roslibrust_mock::MockRos;

use roslibrust_transforms::messages::ros1::{geometry_msgs, std_msgs, TFMessage};
use roslibrust_transforms::{
    Quaternion, Ros1TFMessage, RosTimestamp, Stamp, TimeError, TimePoint, Timestamp, Transform,
    TransformManager, Vector3,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct MockTimestamp {
    t: u128,
}

impl TimePoint for MockTimestamp {
    fn duration_since(self, earlier: Self) -> Result<Duration, TimeError> {
        if self.t < earlier.t {
            return Err(TimeError::DurationUnderflow);
        }

        let diff = self.t - earlier.t;
        let secs = diff / 1_000_000_000;
        let nanos = diff % 1_000_000_000;
        if secs > u64::MAX as u128 {
            return Err(TimeError::DurationOverflow);
        }

        Ok(Duration::new(secs as u64, nanos as u32))
    }

    fn checked_sub(self, rhs: Duration) -> Result<Self, TimeError> {
        let rhs_nanos = rhs.as_nanos();
        self.t
            .checked_sub(rhs_nanos)
            .map(|t| Self { t })
            .ok_or(TimeError::DurationUnderflow)
    }

    fn as_seconds_lossy(self) -> f64 {
        self.t as f64 / 1_000_000_000.0
    }
}

impl RosTimestamp for MockTimestamp {
    fn from_ros_time(sec: i32, nsec: u32) -> Self {
        Self {
            t: (sec as u128) * 1_000_000_000 + (nsec as u128),
        }
    }

    fn as_ros_time(self) -> (i32, u32) {
        let secs = self.t / 1_000_000_000;
        let nsecs = self.t % 1_000_000_000;
        if secs > i32::MAX as u128 || nsecs > u32::MAX as u128 {
            panic!("Timestamp overflow when converting to ROS time");
        }

        (secs as i32, nsecs as u32)
    }
}

#[test]
fn test_from_ros_time_clamps_pre_epoch_times() {
    // Timestamp cannot represent times before the unix epoch, they clamp to zero
    assert_eq!(Timestamp::from_ros_time(-1, 500_000_000), Timestamp::zero());
    assert_eq!(
        Timestamp::from_ros_time(1, 500_000_000),
        Timestamp::from_nanos(1_500_000_000)
    );
}

/// Helper function to create a TFMessage with a single transform.
fn create_tf_message(
    parent_frame: &str,
    child_frame: &str,
    x: f64,
    y: f64,
    z: f64,
    secs: i32,
    nsecs: i32,
) -> TFMessage {
    TFMessage {
        transforms: vec![geometry_msgs::TransformStamped {
            header: std_msgs::Header {
                stamp: roslibrust::codegen::integral_types::Time { secs, nsecs },
                frame_id: parent_frame.to_string(),
                seq: 0,
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
        }],
    }
}

#[tokio::test]
async fn test_transform_listener_with_custom_timestamp() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf topic
    let tf_publisher = mock_ros
        .advertise::<TFMessage>("/tf")
        .await
        .expect("Failed to create /tf publisher");

    // Create the manager with a custom timestamp type
    let manager = TransformManager::<Ros1TFMessage, _, MockTimestamp>::new(
        &mock_ros,
        std::time::Duration::from_secs(10),
    )
    .await
    .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Publish a transform and verify conversion into custom timestamp type
    let tf_msg = create_tf_message("world", "custom_frame", 1.0, 2.0, 3.0, 3, 0);
    tf_publisher
        .publish(&tf_msg)
        .await
        .expect("Failed to publish transform");

    // Give the listener time to process the message
    tokio::time::sleep(Duration::from_millis(100)).await;

    let lookup_time = MockTimestamp { t: 3_000_000_000 };
    let transform = manager
        .get_transform("world", "custom_frame", lookup_time)
        .await
        .expect("Failed to look up transform with custom timestamp");
    assert_eq!(transform.timestamp(), Stamp::At(lookup_time));

    // Add another transform through the manager and verify conversion from custom timestamp
    let transform = Transform::new(
        "world",
        "custom_from_manager",
        Vector3::new(4.0, 5.0, 6.0),
        Quaternion::identity(),
        Stamp::At(MockTimestamp { t: 4_000_000_000 }),
    )
    .expect("Failed to build transform with custom timestamp");
    manager
        .add_transform(transform)
        .await
        .expect("Failed to add transform with custom timestamp");

    let retrieved = manager
        .get_transform(
            "world",
            "custom_from_manager",
            MockTimestamp { t: 4_000_000_000 },
        )
        .await
        .expect("Failed to retrieve transform added with custom timestamp");
    assert!((retrieved.translation().x - 4.0).abs() < 1e-6);
    assert!((retrieved.translation().y - 5.0).abs() < 1e-6);
    assert!((retrieved.translation().z - 6.0).abs() < 1e-6);
}

#[tokio::test]
async fn test_transform_listener_receives_tf_messages() {
    tokio::time::pause();
    use crate::Timestamp;

    let mock_ros = MockRos::new();

    // Create a publisher for /tf topic
    let tf_publisher = mock_ros
        .advertise::<TFMessage>("/tf")
        .await
        .expect("Failed to create /tf publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Publish a transform with a specific timestamp
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap();
    let secs = now.as_secs() as i32;
    let nsecs = now.subsec_nanos() as i32;
    let tf_msg = create_tf_message("world", "base_link", 1.0, 2.0, 3.0, secs, nsecs);

    // Calculate the exact timestamp for lookup (same as what from_ros_time uses)
    let lookup_timestamp = Timestamp::from_nanos((secs as u64) * 1_000_000_000 + (nsecs as u64));

    tf_publisher
        .publish(&tf_msg)
        .await
        .expect("Failed to publish transform");

    // Give the listener time to process the message
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Check that the transform is available at the exact timestamp we published
    let result = manager
        .get_transform("world", "base_link", lookup_timestamp)
        .await;
    assert!(
        result.is_ok(),
        "Transform should be available after publishing at the exact timestamp"
    );
}

#[tokio::test]
async fn test_transform_listener_static_transforms() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf_static topic
    let tf_static_publisher = mock_ros
        .advertise::<TFMessage>("/tf_static")
        .await
        .expect("Failed to create /tf_static publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Publish a static transform (its timestamp is ignored for static transforms)
    let tf_msg = create_tf_message("base_link", "camera_link", 0.5, 0.0, 0.3, 0, 0);

    tf_static_publisher
        .publish(&tf_msg)
        .await
        .expect("Failed to publish static transform");

    // Give the listener time to process the message
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Check that the transform is available, static transforms are valid at any lookup time
    let can_transform = manager
        .get_transform("base_link", "camera_link", Timestamp::now())
        .await;
    assert!(
        can_transform.is_ok(),
        "Static transform should be available after publishing"
    );
}

#[tokio::test]
async fn test_lookup_transform_values() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf_static topic
    let tf_static_publisher = mock_ros
        .advertise::<TFMessage>("/tf_static")
        .await
        .expect("Failed to create /tf_static publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Publish a static transform with known values
    let tf_msg = create_tf_message("world", "sensor", 1.5, 2.5, 3.5, 0, 0);

    tf_static_publisher
        .publish(&tf_msg)
        .await
        .expect("Failed to publish transform");

    // Give the listener time to process the message
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Look up the transform and verify its values
    // Static transforms are valid at any lookup time
    let transform = manager
        .get_transform("world", "sensor", Timestamp::now())
        .await
        .expect("Failed to look up transform");

    // Verify translation values
    let translation = transform.translation();
    assert!(
        (translation.x - 1.5).abs() < 1e-6,
        "Expected x=1.5, got {}",
        translation.x
    );
    assert!(
        (translation.y - 2.5).abs() < 1e-6,
        "Expected y=2.5, got {}",
        translation.y
    );
    assert!(
        (translation.z - 3.5).abs() < 1e-6,
        "Expected z=3.5, got {}",
        translation.z
    );

    // Verify rotation is identity (w=1, x=y=z=0)
    let rotation = transform.rotation();
    assert!(
        (rotation.w - 1.0).abs() < 1e-6,
        "Expected rotation.w=1.0, got {}",
        rotation.w
    );
    assert!(
        rotation.x.abs() < 1e-6,
        "Expected rotation.x=0.0, got {}",
        rotation.x
    );
    assert!(
        rotation.y.abs() < 1e-6,
        "Expected rotation.y=0.0, got {}",
        rotation.y
    );
    assert!(
        rotation.z.abs() < 1e-6,
        "Expected rotation.z=0.0, got {}",
        rotation.z
    );
}

#[tokio::test]
async fn test_wait_for_transform_success() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Spawn a task that will publish the transform after a short delay
    // We need to create a separate publisher in the spawned task since MockPublisher doesn't implement Clone
    let mock_ros_clone = mock_ros.clone();
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(100)).await;
        let publisher = mock_ros_clone
            .advertise::<TFMessage>("/tf_static")
            .await
            .expect("Failed to create /tf_static publisher");
        let tf_msg = create_tf_message("world", "delayed_frame", 1.0, 2.0, 3.0, 0, 0);
        publisher
            .publish(&tf_msg)
            .await
            .expect("Failed to publish transform");
    });

    // Wait for the transform - it should succeed after the delayed publish
    // Static transforms are valid at any lookup time
    let start = tokio::time::Instant::now();
    let result = manager
        .wait_for_transform(
            "world",
            "delayed_frame",
            Timestamp::now(),
            Some(Duration::from_secs(2)),
        )
        .await;

    let elapsed = start.elapsed();

    assert!(result.is_ok(), "wait_for_transform should succeed");
    assert!(
        elapsed >= Duration::from_millis(100),
        "Should have waited for the transform to be published"
    );
    assert!(
        elapsed < Duration::from_secs(1),
        "Should not have waited too long"
    );

    // Verify the transform values
    let translation = result.unwrap().translation();
    assert!((translation.x - 1.0).abs() < 1e-6);
    assert!((translation.y - 2.0).abs() < 1e-6);
    assert!((translation.z - 3.0).abs() < 1e-6);
}

#[tokio::test]
async fn test_wait_for_transform_timeout() {
    tokio::time::pause();
    use roslibrust_transforms::TransformManagerError;

    let mock_ros = MockRos::new();

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Wait for a transform that will never be published
    let start = tokio::time::Instant::now();
    let result = manager
        .wait_for_transform(
            "nonexistent_parent",
            "nonexistent_child",
            Timestamp::zero(),
            Some(Duration::from_millis(200)),
        )
        .await;

    let elapsed = start.elapsed();

    assert!(result.is_err(), "wait_for_transform should timeout");
    assert!(
        elapsed >= Duration::from_millis(200),
        "Should have waited for the full timeout"
    );
    assert!(
        elapsed < Duration::from_millis(400),
        "Should not have waited much longer than the timeout"
    );

    // Verify it's a Timeout error
    match result {
        Err(TransformManagerError::Timeout(parent, child, _time)) => {
            assert_eq!(parent, "nonexistent_parent");
            assert_eq!(child, "nonexistent_child");
        }
        _ => panic!("Expected Timeout error"),
    }
}

#[tokio::test]
async fn test_wait_for_transform_immediate_success() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf_static topic
    let tf_static_publisher = mock_ros
        .advertise::<TFMessage>("/tf_static")
        .await
        .expect("Failed to create /tf_static publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Publish the transform first
    let tf_msg = create_tf_message("world", "immediate_frame", 5.0, 6.0, 7.0, 0, 0);
    tf_static_publisher
        .publish(&tf_msg)
        .await
        .expect("Failed to publish transform");

    // Give the listener time to process
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Wait for the transform - it should return immediately since it's already available
    // Static transforms are valid at any lookup time
    let start = std::time::Instant::now();
    let result = manager
        .wait_for_transform(
            "world",
            "immediate_frame",
            Timestamp::now(),
            Some(Duration::from_secs(5)),
        )
        .await;

    let elapsed = start.elapsed();

    assert!(result.is_ok(), "wait_for_transform should succeed");
    assert!(
        elapsed < Duration::from_millis(100),
        "Should return quickly when transform is already available"
    );
}

#[tokio::test]
async fn test_wait_for_transform_default_timeout() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create the manager with a short buffer duration (used as default timeout)
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_millis(150))
            .await
            .expect("Failed to create TransformManager");

    // Wait for a transform that will never be published, using default timeout (None)
    let start = tokio::time::Instant::now();
    let result = manager
        .wait_for_transform("missing_parent", "missing_child", Timestamp::zero(), None)
        .await;

    let elapsed = start.elapsed();

    assert!(result.is_err(), "wait_for_transform should timeout");
    // Should timeout around the buffer_duration (150ms)
    assert!(
        elapsed >= Duration::from_millis(150),
        "Should have waited for the buffer duration timeout, elapsed: {:?}",
        elapsed
    );
    assert!(
        elapsed < Duration::from_millis(300),
        "Should not have waited much longer than the buffer duration"
    );
}

#[tokio::test]
async fn test_get_transform_at_different_times() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf topic
    let tf_publisher = mock_ros
        .advertise::<TFMessage>("/tf")
        .await
        .expect("Failed to create /tf publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // fixed -> a at t=1s
    let fixed_to_a_t1 = create_tf_message("fixed", "a", 1.0, 0.0, 0.0, 1, 0);
    tf_publisher
        .publish(&fixed_to_a_t1)
        .await
        .expect("Failed to publish fixed->a at t=1s");

    // fixed -> a at t=2s
    let fixed_to_a_t2 = create_tf_message("fixed", "a", 2.0, 0.0, 0.0, 2, 0);
    tf_publisher
        .publish(&fixed_to_a_t2)
        .await
        .expect("Failed to publish fixed->a at t=2s");

    // a -> b at t=1s
    let a_to_b_t1 = create_tf_message("a", "b", 0.0, 1.0, 0.0, 1, 0);
    tf_publisher
        .publish(&a_to_b_t1)
        .await
        .expect("Failed to publish a->b at t=1s");

    // Give the listener time to process the messages
    tokio::time::sleep(Duration::from_millis(100)).await;

    let t1 = Timestamp::from_nanos(1_000_000_000);
    let t2 = Timestamp::from_nanos(2_000_000_000);

    let transform = manager
        .get_transform_at("a", t2, "b", t1, "fixed")
        .await
        .expect("Failed to look up transform at different times");

    assert_eq!(transform.parent(), "a");
    assert_eq!(transform.child(), "b");
    assert_eq!(transform.timestamp(), Stamp::At(t2));

    // b at t=1s in fixed is (1, 1, 0), while a at t=2s in fixed is (2, 0, 0)
    // so b at t=1s expressed in a at t=2s is (-1, 1, 0)
    let translation = transform.translation();
    assert!(
        (translation.x + 1.0).abs() < 1e-6,
        "Expected x=-1.0, got {}",
        translation.x
    );
    assert!(
        (translation.y - 1.0).abs() < 1e-6,
        "Expected y=1.0, got {}",
        translation.y
    );
    assert!(
        translation.z.abs() < 1e-6,
        "Expected z=0.0, got {}",
        translation.z
    );
}

#[tokio::test]
async fn test_invalid_transforms_are_dropped() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf topic
    let tf_publisher = mock_ros
        .advertise::<TFMessage>("/tf")
        .await
        .expect("Failed to create /tf publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Publish a transform with a denormalized rotation, which fails validation during conversion
    // and is dropped by the listener
    let mut bad_msg = create_tf_message("world", "bad_frame", 1.0, 0.0, 0.0, 1, 0);
    bad_msg.transforms[0].transform.rotation.w = 2.0;
    tf_publisher
        .publish(&bad_msg)
        .await
        .expect("Failed to publish invalid transform");

    // Publish a valid transform afterwards to verify the listener keeps processing
    let good_msg = create_tf_message("world", "good_frame", 1.0, 0.0, 0.0, 1, 0);
    tf_publisher
        .publish(&good_msg)
        .await
        .expect("Failed to publish valid transform");

    // Give the listener time to process the messages
    tokio::time::sleep(Duration::from_millis(100)).await;

    let t1 = Timestamp::from_nanos(1_000_000_000);
    let result = manager.get_transform("world", "bad_frame", t1).await;
    assert!(
        result.is_err(),
        "Invalid transform should not have been added to the buffer"
    );

    let result = manager.get_transform("world", "good_frame", t1).await;
    assert!(
        result.is_ok(),
        "Valid transform should still be processed after an invalid one"
    );
}

#[tokio::test]
async fn test_add_static_transform_publishes_to_tf_static() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a subscriber on /tf_static to observe what the manager publishes
    let mut tf_static_subscriber = mock_ros
        .subscribe::<TFMessage>("/tf_static")
        .await
        .expect("Failed to create /tf_static subscriber");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Static transforms are published to /tf_static instead of /tf
    let transform = Transform::static_between(
        "base_link",
        "imu_link",
        Vector3::new(0.1, 0.0, 0.2),
        Quaternion::identity(),
    )
    .expect("Failed to build static transform");
    manager
        .add_transform(transform)
        .await
        .expect("Failed to add static transform");

    let msg = tf_static_subscriber
        .next()
        .await
        .expect("Failed to receive message on /tf_static");
    assert_eq!(msg.transforms.len(), 1);
    assert_eq!(msg.transforms[0].header.frame_id, "base_link");
    assert_eq!(msg.transforms[0].child_frame_id, "imu_link");

    // The static transform is also available in the local buffer at any lookup time
    let transform = manager
        .get_transform("base_link", "imu_link", Timestamp::now())
        .await
        .expect("Failed to look up static transform");
    assert!((transform.translation().x - 0.1).abs() < 1e-6);
    assert!((transform.translation().z - 0.2).abs() < 1e-6);
}

#[tokio::test]
async fn test_latest_common_time() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf topic
    let tf_publisher = mock_ros
        .advertise::<TFMessage>("/tf")
        .await
        .expect("Failed to create /tf publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // world -> a is available at t=1s and t=2s, but a -> b only at t=1s
    let world_to_a_t1 = create_tf_message("world", "a", 1.0, 0.0, 0.0, 1, 0);
    tf_publisher
        .publish(&world_to_a_t1)
        .await
        .expect("Failed to publish world->a at t=1s");
    let world_to_a_t2 = create_tf_message("world", "a", 2.0, 0.0, 0.0, 2, 0);
    tf_publisher
        .publish(&world_to_a_t2)
        .await
        .expect("Failed to publish world->a at t=2s");
    let a_to_b_t1 = create_tf_message("a", "b", 0.0, 1.0, 0.0, 1, 0);
    tf_publisher
        .publish(&a_to_b_t1)
        .await
        .expect("Failed to publish a->b at t=1s");

    // Give the listener time to process the messages
    tokio::time::sleep(Duration::from_millis(100)).await;

    // The newest time the whole chain can serve is bounded by the lagging a -> b hop
    let t1 = Timestamp::from_nanos(1_000_000_000);
    let latest = manager
        .latest_common_time("world", "b")
        .await
        .expect("Failed to get latest common time");
    assert_eq!(latest, Stamp::At(t1));

    // And the returned time is servable
    let result = manager.get_transform("world", "b", t1).await;
    assert!(
        result.is_ok(),
        "Transform should be available at the latest common time"
    );
}

#[tokio::test]
async fn test_reparenting_is_rejected_until_frame_removed() {
    tokio::time::pause();
    let mock_ros = MockRos::new();

    // Create a publisher for /tf topic
    let tf_publisher = mock_ros
        .advertise::<TFMessage>("/tf")
        .await
        .expect("Failed to create /tf publisher");

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Give the listener time to subscribe
    tokio::time::sleep(Duration::from_millis(50)).await;

    // tool is first received as a child of world, pinning its parent
    let world_to_tool = create_tf_message("world", "tool", 1.0, 0.0, 0.0, 1, 0);
    tf_publisher
        .publish(&world_to_tool)
        .await
        .expect("Failed to publish world->tool");

    // A transform re-parenting tool under gripper is rejected and dropped with a warning
    let gripper_to_tool = create_tf_message("gripper", "tool", 0.0, 1.0, 0.0, 2, 0);
    tf_publisher
        .publish(&gripper_to_tool)
        .await
        .expect("Failed to publish gripper->tool");

    // Give the listener time to process the messages
    tokio::time::sleep(Duration::from_millis(100)).await;

    let t1 = Timestamp::from_nanos(1_000_000_000);
    let t2 = Timestamp::from_nanos(2_000_000_000);
    assert!(
        manager.get_transform("world", "tool", t1).await.is_ok(),
        "Original parent should still serve"
    );
    assert!(
        manager.get_transform("gripper", "tool", t2).await.is_err(),
        "Re-parented transform should have been dropped"
    );

    // remove_frame() is the escape hatch that allows the frame to be re-added under a new parent
    assert!(manager.remove_frame("tool").await);
    tf_publisher
        .publish(&gripper_to_tool)
        .await
        .expect("Failed to re-publish gripper->tool");
    tokio::time::sleep(Duration::from_millis(100)).await;

    assert!(
        manager.get_transform("gripper", "tool", t2).await.is_ok(),
        "Frame should be re-added under its new parent after remove_frame"
    );
}
