#![cfg(feature = "ros2_zenoh_test")]

mod common;

use roslibrust_common::traits::*;
use roslibrust_ros2::ZenohClient;
use roslibrust_test::ros2::test_msgs::WStrings;
use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use tokio::time::{sleep, timeout, Duration};

struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn spawn_ros2_relay(input_topic: &str, output_topic: &str) -> ChildGuard {
    let mut child = Command::new("python3")
        .arg(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../roslibrust_test/tests/ros2_wstring_relay.py"
        ))
        .args([input_topic, output_topic])
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("failed to start the rclpy wstring relay");
    let mut ready = String::new();
    BufReader::new(child.stdout.take().unwrap())
        .read_line(&mut ready)
        .expect("failed to read readiness from the rclpy wstring relay");
    assert_eq!(
        ready.trim(),
        "READY",
        "rclpy wstring relay exited before becoming ready"
    );
    ChildGuard(child)
}

fn test_message() -> WStrings {
    WStrings {
        wstring_value: "ハローワールド 🌍".into(),
        wstring_value_default1: "Hello world!".into(),
        wstring_value_default2: "Hellö wörld!".into(),
        wstring_value_default3: "ハローワールド".into(),
        array_of_wstrings: ["one".into(), "二".into(), "🌍".into()],
        bounded_sequence_of_wstrings: vec!["bounded".into(), "文字列".into()],
        unbounded_sequence_of_wstrings: vec!["".into(), "ascii".into(), "四".into()],
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn wstring_round_trips_through_an_rclpy_node() {
    const INPUT_TOPIC: &str = "/roslibrust_ros_z_wstring_input";
    const OUTPUT_TOPIC: &str = "/roslibrust_ros_z_wstring_output";

    let context = common::make_test_context();
    let client = ZenohClient::new(&context, "roslibrust_wstring_interop")
        .await
        .unwrap();
    let publisher = client.advertise::<WStrings>(INPUT_TOPIC).await.unwrap();
    let mut subscriber = client.subscribe::<WStrings>(OUTPUT_TOPIC).await.unwrap();
    let _relay = spawn_ros2_relay(INPUT_TOPIC, OUTPUT_TOPIC);

    sleep(Duration::from_secs(2)).await;
    let expected = test_message();
    publisher.publish(&expected).await.unwrap();

    let received = timeout(Duration::from_secs(5), subscriber.next())
        .await
        .expect("timed out waiting for the rclpy relay")
        .unwrap();
    assert_eq!(received, expected);
}
