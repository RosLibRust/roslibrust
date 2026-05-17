#![cfg(feature = "ros2_zenoh_test")]

mod common;

use roslibrust_common::traits::*;
use roslibrust_ros2::ZenohClient;

#[tokio::test(flavor = "multi_thread")]
async fn test_subscribe_basic() {
    let ctx = common::make_test_context();
    let client = ZenohClient::new(&ctx, "test_subscribe_basic_node")
        .await
        .unwrap();
    let mut subscriber = client
        .subscribe::<roslibrust_test::ros2::std_msgs::String>("/chatter")
        .await
        .unwrap();

    #[allow(clippy::zombie_processes)]
    let mut pub_cmd = std::process::Command::new("ros2")
        .arg("topic")
        .arg("pub")
        .arg("-t")
        .arg("10")
        .arg("/chatter")
        .arg("std_msgs/msg/String")
        .arg("data: Hello World")
        .spawn()
        .unwrap();

    tokio::time::timeout(tokio::time::Duration::from_secs(2), async {
        let msg = subscriber.next().await.unwrap();
        assert_eq!(msg.data, "Hello World");
    })
    .await
    .unwrap();

    pub_cmd.kill().unwrap();
}
