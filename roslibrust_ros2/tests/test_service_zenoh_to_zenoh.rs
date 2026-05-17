#![cfg(feature = "ros2_zenoh_test")]

mod common;

use roslibrust_common::traits::*;
use roslibrust_ros2::ZenohClient;

#[tokio::test(flavor = "multi_thread")]
async fn test_service_zenoh_to_zenoh() {
    let ctx = common::make_test_context();
    let node = ZenohClient::new(&ctx, "test_service_server_zenoh")
        .await
        .unwrap();

    let state = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let state_copy = state.clone();

    let server_fn = move |request: roslibrust_test::ros2::std_srvs::SetBoolRequest| {
        state_copy.store(request.data, std::sync::atomic::Ordering::SeqCst);
        Ok(roslibrust_test::ros2::std_srvs::SetBoolResponse {
            message: "You set my bool!".to_string(),
            success: request.data,
        })
    };

    let _service = node
        .advertise_service::<roslibrust_test::ros2::std_srvs::SetBool, _>(
            "/test_service_zenoh_to_zenoh_set_bool",
            server_fn,
        )
        .await
        .unwrap();

    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    let response = node
        .call_service::<roslibrust_test::ros2::std_srvs::SetBool>(
            "/test_service_zenoh_to_zenoh_set_bool",
            roslibrust_test::ros2::std_srvs::SetBoolRequest { data: true },
        )
        .await
        .expect("Service call should succeed");

    assert!(response.success);
    assert_eq!(response.message, "You set my bool!");
    assert!(state.load(std::sync::atomic::Ordering::SeqCst));
}
