//! This example shows off using tokio's StreamExt to manipulate async streams.
use roslibrust_common::traits::*;
use tokio_stream::StreamExt;

roslibrust_codegen_macro::generate_ros_types!("assets/ros1_common_interfaces");

async fn example_publisher_task(publisher: impl roslibrust::Publish<std_msgs::Header>) {
    for seq in 0..50 {
        publisher
            .publish(&std_msgs::Header {
                stamp: std::time::SystemTime::now().try_into().unwrap(),
                seq,
                frame_id: "test".to_string(),
            })
            .await
            .unwrap();
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    }
}

/// Sets up a subscriber that could get either of two versions of a message
/// Note this is currently only supported by the rosbridge backend as this behavior relies on serde_json's fallback capabilities
#[cfg(feature = "mock")]
#[tokio::main]
async fn main() {
    use log::*;
    env_logger::init();

    let mock_ros = roslibrust::mock::MockRos::new();

    let publisher = mock_ros
        .advertise::<std_msgs::Header>("filter_example_talker")
        .await
        .unwrap();

    // Spawn a task to publish in the background
    tokio::spawn(example_publisher_task(publisher));

    // Create a regular subscriber
    let subscriber = mock_ros
        .subscribe::<std_msgs::Header>("filter_example_talker")
        .await
        .unwrap();

    // Convert the subscriber into a stream
    let stream = subscriber.into_stream();

    // Use [tokio_stream::StreamExt::map] to convert from Result<T> to T
    // This can be convienent if you don't care about Lagged, Serialization problems, etc.
    let stream = stream.map(|msg| match msg {
        Ok(msg) => msg,
        Err(e) => {
            panic!("Failed to get message: {e}");
        }
    });

    // Use [tokio_stream::StreamExt::filter] to filter out even seqs, just to demonstrate that filter works
    let stream = stream.filter(|msg| msg.seq % 2 == 1);

    // Use [tokio_stream::StreamExt::timeout] to apply a timeout
    let stream = stream.timeout(std::time::Duration::from_secs(1));

    // Streams have to be pinned:
    // See: https://tokio.rs/tokio/tutorial/streams
    tokio::pin!(stream);

    // Actually run the stream now
    loop {
        let msg = stream.next().await;
        match msg {
            Some(Ok(msg)) => {
                // This will only log odd numbered messages!
                info!("Got msg: {:?}", msg);
            }
            Some(Err(e)) => {
                // After message #49 is logged this will timeout, log and end the example
                error!("Timeout!: {e}");
                break;
            }
            None => {
                // This case will never occur
                // roslibrust Subscriber streams will always continue to yield values
                // Even if they backend disconnects, they are expected to try to reconnect and re-establish yielding values when that happens
                break;
            }
        }
    }
}

#[cfg(not(feature = "mock"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'mock'");
}
