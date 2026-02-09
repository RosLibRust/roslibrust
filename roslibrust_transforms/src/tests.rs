//! Unit tests for roslibrust_transforms using the MockRos backend.

use std::time::Duration;

use roslibrust_common::{Publish, TopicProvider};
use roslibrust_mock::MockRos;

use crate::ros1::{geometry_msgs, std_msgs, TFMessage};
use crate::{Ros1TFMessage, TransformManager};

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

    let manager = TransformManager::<Ros1TFMessage, _>::new(&mock_ros).await;
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
    let manager = TransformManager::<Ros1TFMessage, _>::new(&mock_ros)
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
        .lookup_transform("world", "base_link", lookup_timestamp)
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
    let manager = TransformManager::<Ros1TFMessage, _>::new(&mock_ros)
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
    let can_transform = manager.can_transform("base_link", "camera_link").await;
    assert!(
        can_transform,
        "Static transform should be available after publishing"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn test_lookup_transform_values() {
    use crate::Timestamp;

    let mock_ros = MockRos::new();

    // Create a publisher for /tf_static topic
    let tf_static_publisher = mock_ros
        .advertise::<TFMessage>("/tf_static")
        .await
        .expect("Failed to create /tf_static publisher");

    // Create the manager
    let manager = TransformManager::<Ros1TFMessage, _>::new(&mock_ros)
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
        .lookup_transform("world", "sensor", Timestamp::zero())
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
