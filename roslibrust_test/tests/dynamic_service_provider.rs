use roslibrust::{DynamicService, DynamicServiceProvider, ServiceDescriptor, ServiceProvider};
use roslibrust_test::ros1::std_srvs;
use serde_json::json;
use std::time::Duration;
use tokio::time::timeout;

const SERVICE_CALL_TIMEOUT: Duration = Duration::from_secs(10);

async fn exercise_dynamic_service_provider<T>(
    ros: T,
    service: &str,
    descriptor: &'static ServiceDescriptor,
) -> roslibrust::Result<()>
where
    T: DynamicServiceProvider,
{
    let client = ros.dynamic_service_client(service, descriptor).await?;
    assert_eq!(client.descriptor().ros_service_name, "std_srvs/SetBool");

    let error = client
        .call_serializable(&json!({"wrong_field": true}))
        .await
        .expect_err("invalid input must be rejected before it reaches the backend");
    assert!(error.to_string().contains("unknown field"));

    let response = timeout(
        SERVICE_CALL_TIMEOUT,
        client.call_serializable(&json!({"data": true})),
    )
    .await
    .map_err(|error| {
        roslibrust::Error::Timeout(format!("dynamic service client call timed out: {error}"))
    })??;
    assert_eq!(
        response.get("success").and_then(|value| value.as_bool()),
        Some(true)
    );
    assert_eq!(
        response.get("message").and_then(|value| value.as_str()),
        Some("dynamic response")
    );

    let request = descriptor
        .request
        .message_from(&json!({"data": false}))
        .map_err(|error| roslibrust::Error::SerializationError(error.to_string()))?;
    let response = timeout(
        SERVICE_CALL_TIMEOUT,
        ros.dynamic_call_service(service, descriptor, request),
    )
    .await
    .map_err(|error| {
        roslibrust::Error::Timeout(format!("dynamic one-shot service call timed out: {error}"))
    })??;
    assert_eq!(
        response.get("success").and_then(|value| value.as_bool()),
        Some(false)
    );
    Ok(())
}

#[cfg(any(feature = "rosbridge_ros1_test", feature = "rosbridge_ros2_test"))]
async fn exercise_rosbridge_dynamic_service_provider(
    ros: roslibrust_rosbridge::ClientHandle,
) -> roslibrust::Result<()> {
    // Older ROS 2 rosbridge releases execute service calls synchronously. Advertising a service
    // through rosbridge and calling it through the same bridge consequently deadlocks while it
    // waits for itself to process the response. rosapi is hosted independently and exercises the
    // same dynamic JSON request/response path without depending on that version-specific behavior.
    let descriptor = roslibrust_test::ros1::SERVICE_REGISTRY
        .get("rosapi/Topics")
        .expect("generated rosapi/Topics descriptor");
    let client = ros
        .dynamic_service_client("/rosapi/topics", descriptor)
        .await?;
    assert_eq!(client.descriptor().ros_service_name, "rosapi/Topics");

    let error = client
        .call_serializable(&json!({"unexpected": true}))
        .await
        .expect_err("invalid input must be rejected before it reaches rosbridge");
    assert!(error.to_string().contains("unknown field"));

    let response = timeout(SERVICE_CALL_TIMEOUT, client.call_serializable(&json!({})))
        .await
        .map_err(|error| {
            roslibrust::Error::Timeout(format!(
                "dynamic rosbridge service client call timed out: {error}"
            ))
        })??;
    assert!(response.get("topics").is_some());
    assert!(response.get("types").is_some());

    let request = descriptor
        .request
        .message_from(&json!({}))
        .map_err(|error| roslibrust::Error::SerializationError(error.to_string()))?;
    let response = timeout(
        SERVICE_CALL_TIMEOUT,
        ros.dynamic_call_service("/rosapi/topics", descriptor, request),
    )
    .await
    .map_err(|error| {
        roslibrust::Error::Timeout(format!(
            "dynamic rosbridge one-shot service call timed out: {error}"
        ))
    })??;
    assert!(response.get("topics").is_some());
    assert!(response.get("types").is_some());
    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn mock_dynamic_service_provider() {
    let ros = roslibrust::mock::MockRos::new();
    let _server = ros
        .advertise_service::<std_srvs::SetBool, _>("/dynamic_service_provider_mock", respond)
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros1::SERVICE_REGISTRY
        .get("std_srvs/SetBool")
        .unwrap();
    assert!(std::ptr::eq(
        descriptor,
        roslibrust_test::ros1::SERVICE_REGISTRY
            .get("std_srvs/srv/SetBool")
            .unwrap()
    ));
    exercise_dynamic_service_provider(ros, "/dynamic_service_provider_mock", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "ros1_test")]
#[test_log::test(tokio::test)]
async fn ros1_dynamic_service_provider() {
    let ros =
        roslibrust::ros1::NodeHandle::new("http://localhost:11311", "/dynamic_service_client_ros1")
            .await
            .unwrap();
    let _server = ros
        .advertise_service::<std_srvs::SetBool, _>("/dynamic_service_provider_ros1", respond)
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros1::SERVICE_REGISTRY
        .get("std_srvs/SetBool")
        .unwrap();
    exercise_dynamic_service_provider(ros, "/dynamic_service_provider_ros1", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "rosbridge_ros1_test")]
#[test_log::test(tokio::test)]
async fn rosbridge_ros1_dynamic_service_provider() {
    let ros = roslibrust_rosbridge::ClientHandle::new_with_options(
        roslibrust_rosbridge::ClientHandleOptions::new("ws://localhost:9090")
            .timeout(SERVICE_CALL_TIMEOUT),
    )
    .await
    .unwrap();
    exercise_rosbridge_dynamic_service_provider(ros)
        .await
        .unwrap();
}

#[cfg(feature = "rosbridge_ros2_test")]
#[test_log::test(tokio::test)]
async fn rosbridge_ros2_dynamic_service_provider() {
    let ros = roslibrust_rosbridge::ClientHandle::new_with_options(
        roslibrust_rosbridge::ClientHandleOptions::new("ws://localhost:9090")
            .timeout(SERVICE_CALL_TIMEOUT),
    )
    .await
    .unwrap();
    exercise_rosbridge_dynamic_service_provider(ros)
        .await
        .unwrap();
}

#[cfg(feature = "zenoh_test")]
#[test_log::test(tokio::test(flavor = "multi_thread"))]
async fn zenoh_ros1_dynamic_service_provider() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let ros = roslibrust_zenoh::ZenohClient::new(session);
    let _server = ros
        .advertise_service::<std_srvs::SetBool, _>("/dynamic_service_provider_zenoh", respond)
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros1::SERVICE_REGISTRY
        .get("std_srvs/SetBool")
        .unwrap();
    exercise_dynamic_service_provider(ros, "/dynamic_service_provider_zenoh", descriptor)
        .await
        .unwrap();
}

#[cfg(feature = "hiroz_test")]
#[test_log::test(tokio::test(flavor = "multi_thread", worker_threads = 1))]
async fn hiroz_dynamic_service_provider() {
    use hiroz::context::ZContextBuilder;
    use hiroz::Builder;

    let context = ZContextBuilder::default()
        .with_domain_id(0)
        .with_connect_endpoints(["tcp/[::]:7447"])
        .build()
        .unwrap();
    let ros = roslibrust_hiroz::ZenohClient::new(&context, "dynamic_service_client_hiroz")
        .await
        .unwrap();
    let _server = ros
        .advertise_service::<roslibrust_test::ros2::std_srvs::SetBool, _>(
            "/dynamic_service_provider_hiroz",
            |request| {
                Ok(roslibrust_test::ros2::std_srvs::SetBoolResponse {
                    success: request.data,
                    message: "dynamic response".to_owned(),
                })
            },
        )
        .await
        .unwrap();
    let descriptor = roslibrust_test::ros2::SERVICE_REGISTRY
        .get("std_srvs/SetBool")
        .unwrap();
    exercise_dynamic_service_provider(ros, "/dynamic_service_provider_hiroz", descriptor)
        .await
        .unwrap();
}

fn respond(
    request: std_srvs::SetBoolRequest,
) -> Result<std_srvs::SetBoolResponse, roslibrust::ServiceError> {
    Ok(std_srvs::SetBoolResponse {
        success: request.data,
        message: "dynamic response".to_owned(),
    })
}
