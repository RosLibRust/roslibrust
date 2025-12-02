use log::*;
use roslibrust_common::traits::*;

#[tokio::main]
async fn main() {
    env_logger::init();

    let ctx = ZContextBuilder::default()
        .with_domain_id(0)
        .with_connect_endpoints(["tcp/[::]:7447"])
        .build()?;

    let client = roslibrust_ros2::ZenohClient::new(ctx, "test_service_server_callable_node")
        .await
        .unwrap();

    let state = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));

    let state_copy = state.clone();
    // Referencing a message definition from the test package
    // Could sub in use of macro or build.rs message generation here
    let server_fn = move |request: roslibrust_test::ros2::std_srvs::SetBoolRequest| {
        state_copy.store(request.data, std::sync::atomic::Ordering::SeqCst);
        info!("Got request to set bool: {request:?}");
        Ok(roslibrust_test::ros2::std_srvs::SetBoolResponse {
            message: "You set my bool!".to_string(),
            success: request.data,
        })
    };

    // Server will be running for as long as _server is kept alive
    let _server = client
        .advertise_service::<roslibrust_test::ros2::std_srvs::SetBool, _>("/set_bool", server_fn)
        .await
        .unwrap();
    // Server is now up and can be called via command line:
    // ros2 service call /test_service_server_callable_node/set_bool std_srvs/srv/SetBool "data: true"

    // Wait for ctrl_c to kill this process
    tokio::signal::ctrl_c().await.unwrap();
}
