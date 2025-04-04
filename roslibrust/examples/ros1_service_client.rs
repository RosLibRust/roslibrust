#[cfg(feature = "ros1")]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use roslibrust::ros1::NodeHandle;

    env_logger::init();

    let nh = NodeHandle::new("http://localhost:11311", "service_client_rs").await?;
    log::info!("Connected!");

    let client = nh
        .service_client::<rosapi::GetTime>("rosapi/get_time")
        .await?;

    let response = client.call(&rosapi::GetTimeRequest {}).await?;

    log::info!("Got time: {:?}", response);

    let response2 = client.call(&rosapi::GetTimeRequest {}).await?;

    log::info!("Got time: {:?}", response2);

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
