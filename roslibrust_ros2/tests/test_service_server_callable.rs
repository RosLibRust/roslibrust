#![cfg(feature = "ros2_zenoh_test")]

mod common;

use roslibrust_common::traits::*;
use roslibrust_ros2::ZenohClient;

#[tokio::test(flavor = "multi_thread")]
async fn test_service_server_callable() {
    let ctx = common::make_test_context();
    let client = ZenohClient::new(&ctx, "test_service_server_callable_node")
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

    let _service = client
        .advertise_service::<roslibrust_test::ros2::std_srvs::SetBool, _>(
            "/test_service_server_callable_node/set_bool",
            server_fn,
        )
        .await
        .unwrap();

    #[allow(clippy::zombie_processes)]
    let mut srv_call_cmd = std::process::Command::new("ros2")
        .arg("service")
        .arg("call")
        .arg("/test_service_server_callable_node/set_bool")
        .arg("std_srvs/srv/SetBool")
        .arg("data: true")
        .spawn()
        .unwrap();

    tokio::time::timeout(tokio::time::Duration::from_secs(2), async {
        while !state.load(std::sync::atomic::Ordering::SeqCst) {
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("Bool should be set true within 2 seconds");

    srv_call_cmd.kill().unwrap()
}
