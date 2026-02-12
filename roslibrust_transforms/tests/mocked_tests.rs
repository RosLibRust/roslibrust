//! Unit tests for roslibrust_transforms using the MockRos backend.

use std::time::Duration;

use roslibrust_common::{Publish, TopicProvider};
use roslibrust_mock::MockRos;

use roslibrust_transforms::messages::ros1::{geometry_msgs, std_msgs, TFMessage};
use roslibrust_transforms::{Ros1TFMessage, Timestamp, TransformManager};

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
        Err(TransformManagerError::Timeout(parent, child)) => {
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
