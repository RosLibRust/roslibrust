use log::*;
use roslibrust::ClientHandle;

roslibrust_codegen_macro::find_and_generate_ros_messages!("example_msgs/local_msgs");

/// This example creates a client, and publishes a message to the topic "talker"
/// Running this example at the same time as subscribe_and_log will have the two examples
/// pass messages between each other.
/// To run this example a rosbridge websocket server should be running at the deafult port (9090).
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running wsl2
        .init()
        .unwrap();

    let client = ClientHandle::new("ws://localhost:9090").await?;
    let publisher = client.advertise::<std_msgs::Header>("talker").await?;

    loop {
        let msg = std_msgs::Header::default();
        info!("About to publish");
        publisher.publish(msg).await.unwrap();
        info!("Published msg...");
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
    }
}
