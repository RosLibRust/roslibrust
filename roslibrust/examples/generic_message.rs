//! This example shows off creating a custom generic message, and leveraging serde_json's parsing resolution
//! to decode to the right type.
use roslibrust_common::RosMessageType;

/// We place the ros1 generate code into a module to prevent name collisions with the identically
/// named ros2 types.
mod ros1 {
    roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");
}

mod ros2 {
    roslibrust_codegen_macro::find_and_generate_ros_messages_without_ros_package_path!(
        "assets/ros2_common_interfaces"
    );
}

/// Here we manually define a custom type with the traits needed to send it as a message
#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
/// This serde flag lets the parser know that the incoming type won't say which variant it is
/// instead serde_json will try to deserialize as each variant in order and give whichever one first
/// succeeds
#[serde(untagged)]
enum GenericHeader {
    V1(ros1::std_msgs::Header),
    V2(ros2::std_msgs::Header),
}

/// We need to manually implement this trait for our custom type, normally this is done
/// for us by th code generation
impl RosMessageType for GenericHeader {
    /// Note: this trick only works (well) if the messages we're being generic over have
    /// the same ROS type name.
    const ROS_TYPE_NAME: &'static str = "std_msgs/Header";

    // "*" is used as a wildcard to match any md5sum
    const MD5SUM: &'static str = "*";
    const DEFINITION: &'static str = "";
}

/// Sets up a subscriber that could get either of two versions of a message
/// Note this is currently only supported by the rosbridge backend as this behavior relies on serde_json's fallback capabilities
#[cfg(feature = "rosbridge")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use log::*;
    env_logger::init();

    // An instance of rosbridge needs to be running at this address for the example to work
    let client = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    info!("ClientHandle connected");

    let rx = client.subscribe::<GenericHeader>("talker").await?;
    info!("Successfully subscribed to topic: talker");

    // Once the code reaches here, we'll need someone to publish to trigger these prints
    // In ros1 this would be `rostopic pub /talker <message data>`
    loop {
        let msg = rx.next().await;
        match msg {
            GenericHeader::V1(ros1_header) => {
                info!("Got ros1: {ros1_header:?}");
            }
            GenericHeader::V2(ros2_header) => {
                info!("Got ros2: {ros2_header:?}");
            }
        }
    }
}

#[cfg(not(feature = "rosbridge"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'rosbridge'");
}
