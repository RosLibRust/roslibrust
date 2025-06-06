use roslibrust_zenoh::ZenohClient;

/// IMPORTANT to bring this trait into scope so we can access the functions it provides
/// This trait provides the [next][roslibrust::Subscribe::next] function on ZenohSubscriber
use roslibrust::Subscribe;
/// IMPORTANT to bring this trait into scope so we can access the functions it provides
/// This trait provides the [subscribe][roslibrust::TopicProvider::subscribe] and [advertise][roslibrust::TopicProvider::advertise] functions on ZenohCilent
use roslibrust::TopicProvider;

// Generate rust definitions for our messages
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

// The example expects a zenoh-ros1-bridge to be running see [here](https://github.com/eclipse-zenoh/zenoh-plugin-ros1)
// for details on running the bridge.

// While the bridge is running (with a rosmaster either internally or externally), the following command can be used
// to test the functionality: `rostopic pub -r 1 /chatter std_msgs/String "data: 'hello world'"`
// Or run the publisher example `cargo run --example publisher`
#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let client = ZenohClient::new(session);

    // Create a zenoh subscriber to the ros topic /chatter
    // Internally this handles the "topic mangling" that zenoh-ros1-plugin / zenoh-ros1-bridge performs
    // and sets up deserialization of the ROS1 type into our Rust type
    let mut subscriber = client
        .subscribe::<std_msgs::String>("/chatter")
        .await
        .unwrap();

    loop {
        // Get the next message
        let msg = subscriber.next().await.unwrap();
        // Publish
        println!("Got message: {}", msg.data);
    }
}
