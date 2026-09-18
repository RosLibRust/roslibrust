//! Backend-independent integration tests for `DynamicTopicProvider`.
//!
//! Add each backend implementation as a small setup test that calls
//! `exercise_dynamic_topic_provider`; the behavior assertions remain shared.

use roslibrust::{DynamicPublish, DynamicSubscribe, DynamicTopicProvider, MessageDescriptor};
use serde_json::json;
use std::sync::Arc;
use tokio::time::{timeout, Duration};

async fn exercise_dynamic_topic_provider<T>(
    ros: T,
    topic: &str,
    descriptor: &'static MessageDescriptor,
) -> roslibrust::Result<()>
where
    T: DynamicTopicProvider,
{
    let mut subscriber = ros.dynamic_subscribe(topic, descriptor).await?;
    let publisher = Arc::new(ros.dynamic_advertise(topic, descriptor).await?);

    assert_eq!(publisher.descriptor().ros_type_name, "std_msgs/String");

    let error = publisher
        .publish_serializable(&json!({"wrong_field": "not publishable"}))
        .await
        .expect_err("invalid input must be rejected before it reaches the backend");
    assert!(error.to_string().contains("unknown field"));

    let first = descriptor
        .message_from(&json!({"data": "constructed message"}))
        .expect("valid JSON-shaped message");
    // Native transports may legitimately drop publications made before endpoint discovery
    // completes. Publish repeatedly so this assertion measures the provider rather than discovery
    // timing.
    let retry_publisher = publisher.clone();
    let retry_message = first.clone();
    let publish_task = tokio::spawn(async move {
        loop {
            retry_publisher.publish(&retry_message).await.unwrap();
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
    let received = timeout(Duration::from_secs(10), subscriber.next()).await;
    publish_task.abort();
    let received = received.expect("timed out waiting for first dynamic message")?;
    assert_eq!(received.descriptor().ros_type_name, "std_msgs/String");
    assert_eq!(
        serde_json::to_value(received.value()).unwrap(),
        serde_json::to_value(first.value()).unwrap()
    );

    publisher
        .publish_serializable(&json!({"data": "serialized directly"}))
        .await?;
    let received = timeout(Duration::from_secs(2), subscriber.next())
        .await
        .expect("timed out waiting for second dynamic message")?;
    assert_eq!(
        serde_json::to_value(received.value()).unwrap(),
        json!({"data": "serialized directly"})
    );

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn mock_dynamic_topic_provider() {
    let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
        .get("std_msgs/String")
        .unwrap();
    exercise_dynamic_topic_provider(
        roslibrust::mock::MockRos::new(),
        "/dynamic_topic_provider_mock",
        descriptor,
    )
    .await
    .unwrap();
}

#[cfg(feature = "ros1_test")]
#[test_log::test(tokio::test)]
async fn ros1_dynamic_topic_provider() {
    let ros =
        roslibrust::ros1::NodeHandle::new("http://localhost:11311", "/dynamic_topic_provider_ros1")
            .await
            .unwrap();
    let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
        .get("std_msgs/String")
        .unwrap();
    exercise_dynamic_topic_provider(ros, "/dynamic_topic_provider_ros1", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "zenoh_test")]
#[test_log::test(tokio::test(flavor = "multi_thread"))]
async fn zenoh_ros1_dynamic_topic_provider() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let ros = roslibrust_zenoh::ZenohClient::new(session);
    let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
        .get("std_msgs/String")
        .unwrap();
    exercise_dynamic_topic_provider(ros, "/dynamic_topic_provider_zenoh", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "rosbridge_ros1_test")]
#[test_log::test(tokio::test)]
async fn rosbridge_ros1_dynamic_topic_provider() {
    let ros = roslibrust_rosbridge::ClientHandle::new("ws://localhost:9090")
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros1::MESSAGE_REGISTRY
        .get("std_msgs/String")
        .unwrap();
    exercise_dynamic_topic_provider(ros, "/dynamic_topic_provider_bridge1", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "rosbridge_ros2_test")]
#[test_log::test(tokio::test)]
async fn rosbridge_ros2_dynamic_topic_provider() {
    let ros = roslibrust_rosbridge::ClientHandle::new("ws://localhost:9090")
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros2::MESSAGE_REGISTRY
        .get("std_msgs/String")
        .unwrap();
    exercise_dynamic_topic_provider(ros, "/dynamic_topic_provider_bridge2", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "hiroz_test")]
#[test_log::test(tokio::test(flavor = "multi_thread", worker_threads = 1))]
async fn hiroz_dynamic_topic_provider() {
    use hiroz::context::ZContextBuilder;
    use hiroz::Builder;

    let context = ZContextBuilder::default()
        .with_domain_id(0)
        .with_connect_endpoints(["tcp/[::]:7447"])
        .build()
        .unwrap();
    let ros = roslibrust_hiroz::ZenohClient::new(&context, "dynamic_topic_provider_hiroz")
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros2::MESSAGE_REGISTRY
        .get("std_msgs/String")
        .unwrap();
    exercise_dynamic_topic_provider(ros, "/dynamic_topic_provider_hiroz", descriptor)
        .await
        .unwrap();
}
