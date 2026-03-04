//! Unit tests for roslibrust_transforms using the MockRos backend.

use std::time::Duration;

use roslibrust_common::{Publish, TopicProvider};
use roslibrust_mock::MockRos;
use transforms::time::{TimeError, TimePoint};

use roslibrust_transforms::messages::ros1::{geometry_msgs, std_msgs, TFMessage};
use roslibrust_transforms::{
    Quaternion, Ros1TFMessage, RosTimestamp, Timestamp, TransformManager, Vector3,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct MockTimestamp {
    t: u128,
}

impl TimePoint for MockTimestamp {
    fn static_timestamp() -> Self {
        Self { t: 0 }
    }

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

    fn checked_add(self, rhs: Duration) -> Result<Self, TimeError> {
        let rhs_nanos = rhs.as_nanos();
        self.t
            .checked_add(rhs_nanos)
            .map(|t| Self { t })
            .ok_or(TimeError::DurationOverflow)
    }

    fn checked_sub(self, rhs: Duration) -> Result<Self, TimeError> {
        let rhs_nanos = rhs.as_nanos();
        self.t
            .checked_sub(rhs_nanos)
            .map(|t| Self { t })
            .ok_or(TimeError::DurationUnderflow)
    }

    fn as_seconds(self) -> Result<f64, TimeError> {
        Ok(self.t as f64 / 1_000_000_000.0)
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

#[tokio::test(flavor = "multi_thread")]
async fn test_transform_listener_creation() {
    let mock_ros = MockRos::new();

    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await;
    assert!(manager.is_ok(), "Failed to create TransformManager");
}

#[tokio::test(flavor = "multi_thread")]
async fn test_transform_listener_with_custom_timestamp() {
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
    assert_eq!(transform.timestamp, lookup_time);

    // Add another transform through the manager and verify conversion from custom timestamp
    let transform = transforms::Transform {
        parent: "world".to_string(),
        child: "custom_from_manager".to_string(),
        translation: Vector3::new(4.0, 5.0, 6.0),
        rotation: Quaternion::identity(),
        timestamp: MockTimestamp { t: 4_000_000_000 },
    };
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
    assert!((retrieved.translation.x - 4.0).abs() < 1e-6);
    assert!((retrieved.translation.y - 5.0).abs() < 1e-6);
    assert!((retrieved.translation.z - 6.0).abs() < 1e-6);
}

#[tokio::test(flavor = "multi_thread")]
async fn test_transform_listener_receives_tf_messages() {
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

    // Calculate the exact timestamp for lookup (same as what convert_transform_stamped uses)
    let lookup_timestamp = Timestamp {
        t: (secs as u128) * 1_000_000_000 + (nsecs as u128),
    };

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

#[tokio::test(flavor = "multi_thread")]
async fn test_transform_listener_static_transforms() {
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

    // Publish a static transform (timestamp doesn't matter for static transforms)
    let tf_msg = create_tf_message("base_link", "camera_link", 0.5, 0.0, 0.3, 0, 0);

    tf_static_publisher
        .publish(&tf_msg)
        .await
        .expect("Failed to publish static transform");

    // Give the listener time to process the message
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Check that the transform is available
    let can_transform = manager
        .get_transform("base_link", "camera_link", Timestamp::zero())
        .await;
    assert!(
        can_transform.is_ok(),
        "Static transform should be available after publishing"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn test_lookup_transform_values() {
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
    // Static transforms use Timestamp::zero()
    let transform = manager
        .get_transform("world", "sensor", Timestamp::zero())
        .await
        .expect("Failed to look up transform");

    // Verify translation values
    assert!(
        (transform.translation.x - 1.5).abs() < 1e-6,
        "Expected x=1.5, got {}",
        transform.translation.x
    );
    assert!(
        (transform.translation.y - 2.5).abs() < 1e-6,
        "Expected y=2.5, got {}",
        transform.translation.y
    );
    assert!(
        (transform.translation.z - 3.5).abs() < 1e-6,
        "Expected z=3.5, got {}",
        transform.translation.z
    );

    // Verify rotation is identity (w=1, x=y=z=0)
    assert!(
        (transform.rotation.w - 1.0).abs() < 1e-6,
        "Expected rotation.w=1.0, got {}",
        transform.rotation.w
    );
    assert!(
        transform.rotation.x.abs() < 1e-6,
        "Expected rotation.x=0.0, got {}",
        transform.rotation.x
    );
    assert!(
        transform.rotation.y.abs() < 1e-6,
        "Expected rotation.y=0.0, got {}",
        transform.rotation.y
    );
    assert!(
        transform.rotation.z.abs() < 1e-6,
        "Expected rotation.z=0.0, got {}",
        transform.rotation.z
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn test_wait_for_transform_success() {
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
    let start = std::time::Instant::now();
    let result = manager
        .wait_for_transform(
            "world",
            "delayed_frame",
            Timestamp::zero(),
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
    let transform = result.unwrap();
    assert!((transform.translation.x - 1.0).abs() < 1e-6);
    assert!((transform.translation.y - 2.0).abs() < 1e-6);
    assert!((transform.translation.z - 3.0).abs() < 1e-6);
}

#[tokio::test(flavor = "multi_thread")]
async fn test_wait_for_transform_timeout() {
    use roslibrust_transforms::TransformManagerError;

    let mock_ros = MockRos::new();

    // Create the manager
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_secs(10))
            .await
            .expect("Failed to create TransformManager");

    // Wait for a transform that will never be published
    let start = std::time::Instant::now();
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

#[tokio::test(flavor = "multi_thread")]
async fn test_wait_for_transform_immediate_success() {
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
    let start = std::time::Instant::now();
    let result = manager
        .wait_for_transform(
            "world",
            "immediate_frame",
            Timestamp::zero(),
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

#[tokio::test(flavor = "multi_thread")]
async fn test_wait_for_transform_default_timeout() {
    let mock_ros = MockRos::new();

    // Create the manager with a short buffer duration (used as default timeout)
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&mock_ros, std::time::Duration::from_millis(150))
            .await
            .expect("Failed to create TransformManager");

    // Wait for a transform that will never be published, using default timeout (None)
    let start = std::time::Instant::now();
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

#[tokio::test(flavor = "multi_thread")]
async fn test_get_transform_at_different_times() {
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

    let t1 = Timestamp { t: 1_000_000_000 };
    let t2 = Timestamp { t: 2_000_000_000 };

    let transform = manager
        .get_transform_at("a", t2, "b", t1, "fixed")
        .await
        .expect("Failed to look up transform at different times");

    assert_eq!(transform.parent, "a");
    assert_eq!(transform.child, "b");
    assert_eq!(transform.timestamp, t2);

    // b at t=1s in fixed is (1, 1, 0), while a at t=2s in fixed is (2, 0, 0)
    // so b at t=1s expressed in a at t=2s is (-1, 1, 0)
    assert!(
        (transform.translation.x + 1.0).abs() < 1e-6,
        "Expected x=-1.0, got {}",
        transform.translation.x
    );
    assert!(
        (transform.translation.y - 1.0).abs() < 1e-6,
        "Expected y=1.0, got {}",
        transform.translation.y
    );
    assert!(
        transform.translation.z.abs() < 1e-6,
        "Expected z=0.0, got {}",
        transform.translation.z
    );
}
