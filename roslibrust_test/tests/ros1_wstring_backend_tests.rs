use roslibrust::traits::{Publish, Subscribe, TopicProvider};
use roslibrust_test::ros2::ros2_test_msgs::WStrings;

fn wstrings_message() -> WStrings {
    WStrings {
        wstring_value: "ハローワールド 🌍".into(),
        wstring_value_default1: "Hello world!".into(),
        wstring_value_default2: "Hellö wörld!".into(),
        wstring_value_default3: "ハローワールド".into(),
        array_of_wstrings: ["one".into(), "二".into(), "🌍".into()],
        bounded_sequence_of_wstrings: vec!["bounded".into(), "文字列".into()],
        unbounded_sequence_of_wstrings: vec!["".into(), "ascii".into()],
    }
}

#[test]
fn tcpros_codec_uses_utf8_ros_strings_and_round_trips_wstrings() {
    let message = wstrings_message();

    // roslibrust_ros1 and roslibrust_zenoh both use this codec when publishing.
    let bytes = roslibrust::with_ros1_wstring_compatibility(|| {
        roslibrust_serde_rosmsg::ser::to_vec(&message)
    })
    .unwrap();

    // The first field has ordinary ROS 1 string encoding: its UTF-8 byte length
    // followed by its UTF-8 bytes.
    let encoded_string_bytes = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
    assert_eq!(encoded_string_bytes as usize, message.wstring_value.len());
    assert_eq!(
        &bytes[8..8 + encoded_string_bytes as usize],
        message.wstring_value.as_bytes()
    );

    let decoded: WStrings = roslibrust::with_ros1_wstring_compatibility(|| {
        roslibrust_serde_rosmsg::de::from_slice(&bytes)
    })
    .unwrap();

    assert_eq!(decoded, message);
}

#[test]
fn rosbridge_json_represents_wstrings_as_json_strings() {
    let json = serde_json::to_value(wstrings_message()).unwrap();

    assert_eq!(json["wstring_value"], "ハローワールド 🌍");
    assert_eq!(json["array_of_wstrings"][1], "二");
    assert_eq!(json["bounded_sequence_of_wstrings"][1], "文字列");
}

#[tokio::test]
async fn mock_backend_round_trips_wstrings() {
    let backend = roslibrust::mock::MockRos::new();
    let publisher = backend
        .advertise::<WStrings>("/wstring_test")
        .await
        .unwrap();
    let mut subscriber = backend
        .subscribe::<WStrings>("/wstring_test")
        .await
        .unwrap();
    let message = wstrings_message();

    publisher.publish(&message).await.unwrap();

    assert_eq!(subscriber.next().await.unwrap(), message);
}
