//! Integration tests for GraphProvider trait across all backends
//!
//! These tests verify that list_topics() and list_services() work correctly
//! for each backend implementation.
//!
//! Run tests for specific backends:
//! - Mock: cargo test --test graph_provider_tests (no feature needed)
//! - ROS1: cargo test --test graph_provider_tests --features ros1_test
//! - ROS2 Zenoh: cargo test --test graph_provider_tests --features ros2_zenoh_test
//! - Rosbridge ROS1: cargo test --test graph_provider_tests --features rosbridge_ros1_test
//! - Rosbridge ROS2: cargo test --test graph_provider_tests --features rosbridge_ros2_test

use roslibrust::traits::*;
use roslibrust_test::ros1::*;

/// Generic test function that exercises GraphProvider topics for any backend
async fn test_graph_provider_topics<T: TopicProvider + ServiceProvider + GraphProvider>(
    ros: T,
) -> roslibrust::Result<()> {
    // Initially, we might have topics from other tests or system, so just verify the call works
    let initial_topics = ros.list_topics().await?;

    // Advertise a topic
    let _publisher = ros
        .advertise::<std_msgs::String>("/test_graph_topic")
        .await?;

    // Give the graph time to update (backends may need time to propagate)
    tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

    // List topics again
    let topics = ros.list_topics().await?;

    // Verify our topic appears in the list
    let found = topics.iter().any(|t| t.name == "/test_graph_topic");
    assert!(
        found,
        "Expected to find /test_graph_topic in list, got: {:?}",
        topics
    );

    // Verify the topic has the correct type
    let topic_info = topics
        .iter()
        .find(|t| t.name == "/test_graph_topic")
        .expect("Topic should exist");
    assert_eq!(
        topic_info.type_name, "std_msgs/String",
        "Topic type should be std_msgs/String"
    );

    // Verify we have more topics now than initially (or at least the same if our topic already existed)
    assert!(
        topics.len() >= initial_topics.len(),
        "Should have at least as many topics after advertising"
    );

    Ok(())
}

/// Generic test function that exercises GraphProvider services for any backend
async fn test_graph_provider_services<T: TopicProvider + ServiceProvider + GraphProvider>(
    ros: T,
) -> roslibrust::Result<()> {
    // Initially, we might have services from other tests or system
    let initial_services = ros.list_services().await?;

    // Advertise a service
    let _service = ros
        .advertise_service::<std_srvs::SetBool, _>("/test_graph_service", |req| {
            Ok(std_srvs::SetBoolResponse {
                success: req.data,
                message: "test".to_string(),
            })
        })
        .await?;

    // Give the graph time to update
    tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

    // List services
    let services = ros.list_services().await?;

    // Verify our service appears in the list
    let found = services.iter().any(|s| s.name == "/test_graph_service");
    assert!(
        found,
        "Expected to find /test_graph_service in list, got: {:?}",
        services
    );

    // Verify the service has the correct type
    let service_info = services
        .iter()
        .find(|s| s.name == "/test_graph_service")
        .expect("Service should exist");
    assert_eq!(
        service_info.type_name, "std_srvs/SetBool",
        "Service type should be std_srvs/SetBool"
    );

    // Verify we have more services now than initially
    assert!(
        services.len() >= initial_services.len(),
        "Should have at least as many services after advertising"
    );

    Ok(())
}

// ============================================================================
// Mock Backend Tests
// ============================================================================

#[tokio::test(flavor = "multi_thread")]
async fn test_mock_graph_provider_topics() {
    let ros = roslibrust::mock::MockRos::new();
    test_graph_provider_topics(ros).await.unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn test_mock_graph_provider_services() {
    let ros = roslibrust::mock::MockRos::new();
    test_graph_provider_services(ros).await.unwrap();
}

// ============================================================================
// ROS1 Backend Tests
// ============================================================================

#[cfg(feature = "ros1_test")]
mod ros1_tests {
    use super::*;
    use roslibrust::ros1::NodeHandle;

    #[test_log::test(tokio::test)]
    async fn test_ros1_graph_provider_topics() {
        let ros = NodeHandle::new("http://localhost:11311", "/test_graph_provider_topics")
            .await
            .unwrap();
        test_graph_provider_topics(ros).await.unwrap();
    }

    #[test_log::test(tokio::test)]
    async fn test_ros1_graph_provider_services() {
        let ros = NodeHandle::new("http://localhost:11311", "/test_graph_provider_services")
            .await
            .unwrap();
        test_graph_provider_services(ros).await.unwrap();
    }
}

// ============================================================================
// ROS2 Zenoh Backend Tests
// ============================================================================

#[cfg(feature = "ros2_zenoh_test")]
mod ros2_zenoh_tests {
    use super::*;
    use ros_z::context::ZContextBuilder;
    use ros_z::Builder;
    use roslibrust_ros2::ZenohClient;

    fn make_test_context() -> ros_z::context::ZContext {
        ZContextBuilder::default()
            .with_domain_id(0)
            .with_connect_endpoints(["tcp/[::]:7447"])
            .build()
            .unwrap()
    }

    #[test_log::test(tokio::test)]
    async fn test_ros2_zenoh_graph_provider_topics() {
        let ctx = make_test_context();
        let ros = ZenohClient::new(&ctx, "test_graph_provider_topics")
            .await
            .unwrap();
        test_graph_provider_topics(ros).await.unwrap();
    }

    #[test_log::test(tokio::test)]
    async fn test_ros2_zenoh_graph_provider_services() {
        let ctx = make_test_context();
        let ros = ZenohClient::new(&ctx, "test_graph_provider_services")
            .await
            .unwrap();
        test_graph_provider_services(ros).await.unwrap();
    }
}

// ============================================================================
// Rosbridge ROS1 Backend Tests
// ============================================================================

#[cfg(feature = "rosbridge_ros1_test")]
mod rosbridge_ros1_tests {
    use super::*;
    use roslibrust_rosbridge::ClientHandle;

    #[test_log::test(tokio::test)]
    async fn test_rosbridge_ros1_graph_provider_topics() {
        let ros = ClientHandle::new("ws://localhost:9090").await.unwrap();
        test_graph_provider_topics(ros).await.unwrap();
    }

    #[test_log::test(tokio::test)]
    async fn test_rosbridge_ros1_graph_provider_services() {
        let ros = ClientHandle::new("ws://localhost:9090").await.unwrap();
        test_graph_provider_services(ros).await.unwrap();
    }
}

// ============================================================================
// Rosbridge ROS2 Backend Tests
// ============================================================================

#[cfg(feature = "rosbridge_ros2_test")]
mod rosbridge_ros2_tests {
    use super::*;
    use roslibrust_rosbridge::ClientHandle;

    #[test_log::test(tokio::test)]
    async fn test_rosbridge_ros2_graph_provider_topics() {
        let ros = ClientHandle::new("ws://localhost:9090").await.unwrap();
        test_graph_provider_topics(ros).await.unwrap();
    }

    #[test_log::test(tokio::test)]
    async fn test_rosbridge_ros2_graph_provider_services() {
        let ros = ClientHandle::new("ws://localhost:9090").await.unwrap();
        test_graph_provider_services(ros).await.unwrap();
    }
}
