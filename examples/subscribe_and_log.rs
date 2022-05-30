use log::*;
use roslibrust::Client;

/// A basic example of connecting and subscribing to data.
/// This example will log recieved messages if run at the same time as "basic_publisher".
/// A running rosbridge websocker server at the default port (9090) is required to run this example.
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        // TODO had to remove timestamps here to prevent a panic on my laptop with
        // "Could not determine UTC offset on this system"
        // need to investigate futher at some point
        .without_timestamps()
        .init()
        .unwrap();

    let mut client = Client::new("ws://localhost:9090").await?;
    info!("Client connected");

    let mut rx = client
        .subscribe::<roslibrust::test_msgs::Header>("talker")
        .await?;
    info!("Successfully subscribed to topic: talker");

    loop {
        let _ = rx.changed().await;
        let msg = rx.borrow();
        info!("Got msg: {:?}", msg);
    }
}
