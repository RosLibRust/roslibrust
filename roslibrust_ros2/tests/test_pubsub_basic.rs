#![cfg(feature = "ros2_zenoh_test")]

mod common;

use roslibrust_common::traits::*;
use roslibrust_ros2::ZenohClient;

#[tokio::test(flavor = "multi_thread")]
async fn test_pubsub_basic() {
    let ctx = common::make_test_context();
    let client = ZenohClient::new(&ctx, "test_publish_basic_node")
        .await
        .unwrap();

    let publisher = client
        .advertise::<roslibrust_test::ros2::std_msgs::String>("/chatter")
        .await
        .unwrap();

    let mut subscriber = client
        .subscribe::<roslibrust_test::ros2::std_msgs::String>("/chatter")
        .await
        .unwrap();

    let msg = roslibrust_test::ros2::std_msgs::String {
        data: "Hello World".to_string(),
    };

    publisher.publish(&msg).await.unwrap();

    tokio::time::timeout(tokio::time::Duration::from_secs(2), async {
        let msg = subscriber.next().await.unwrap();
        assert_eq!(msg.data, "Hello World");
    })
    .await
    .expect("Failed to receive message within 2 seconds");
}
