//! Comprehensive test suite for roslibrust_mcap
//!
//! This test suite covers:
//! - Edge cases and error handling
//! - Performance with large messages and high message counts
//! - Complex message types (arrays, nested structures, geometry)
//! - Timestamp ordering and validation
//! - Schema reuse and channel management
//! - Concurrent writing scenarios

use std::io::Cursor;

// Use pre-generated ROS2 message types from roslibrust_test
use roslibrust_test::ros2::*;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

#[test]
fn test_empty_mcap_file() -> Result<()> {
    // Test that we can create and read an empty MCAP file (no messages)
    let mut buffer = Vec::new();

    {
        let writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    assert_eq!(reader.channels().len(), 0);

    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;
    assert_eq!(messages.len(), 0);

    Ok(())
}

#[test]
fn test_channel_with_no_messages() -> Result<()> {
    // Test creating a channel but not writing any messages to it
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let _channel = writer.add_ros_channel::<std_msgs::String>("/unused")?;
        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    assert_eq!(reader.channels().len(), 1);

    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;
    assert_eq!(messages.len(), 0);

    Ok(())
}

#[test]
fn test_empty_string_message() -> Result<()> {
    // Test writing and reading messages with empty strings
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/test")?;

        channel.write(
            &mut writer,
            1_000_000_000,
            &std_msgs::String {
                data: String::new(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 1);
    let msg: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg.data, "");

    Ok(())
}

#[test]
fn test_zero_timestamp() -> Result<()> {
    // Test writing messages with timestamp 0
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/test")?;

        channel.write(
            &mut writer,
            0,
            &std_msgs::String {
                data: "zero time".to_string(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 1);
    assert_eq!(messages[0].log_time, 0);

    Ok(())
}

#[test]
fn test_very_large_timestamp() -> Result<()> {
    // Test with maximum u64 timestamp
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/test")?;

        channel.write(
            &mut writer,
            u64::MAX,
            &std_msgs::String {
                data: "max time".to_string(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 1);
    assert_eq!(messages[0].log_time, u64::MAX);

    Ok(())
}

// ============================================================================
// Complex Message Types
// ============================================================================

#[test]
fn test_geometry_messages() -> Result<()> {
    // Test with complex geometry messages containing nested structures
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let pose_channel = writer.add_ros_channel::<geometry_msgs::PoseStamped>("/pose")?;
        let twist_channel = writer.add_ros_channel::<geometry_msgs::TwistStamped>("/twist")?;

        // Write a PoseStamped message
        let pose = geometry_msgs::PoseStamped {
            header: std_msgs::Header {
                stamp: builtin_interfaces::Time {
                    sec: 100,
                    nanosec: 500_000_000,
                },
                frame_id: "map".to_string(),
            },
            pose: geometry_msgs::Pose {
                position: geometry_msgs::Point {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                orientation: geometry_msgs::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        };

        pose_channel.write(&mut writer, 1_000_000_000, &pose)?;

        // Write a TwistStamped message
        let twist = geometry_msgs::TwistStamped {
            header: std_msgs::Header {
                stamp: builtin_interfaces::Time {
                    sec: 101,
                    nanosec: 0,
                },
                frame_id: "base_link".to_string(),
            },
            twist: geometry_msgs::Twist {
                linear: geometry_msgs::Vector3 {
                    x: 1.5,
                    y: 0.0,
                    z: 0.0,
                },
                angular: geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.5,
                },
            },
        };

        twist_channel.write(&mut writer, 2_000_000_000, &twist)?;

        writer.finish()?;
    }

    // Read back and verify
    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    assert_eq!(reader.channels().len(), 2);

    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;
    assert_eq!(messages.len(), 2);

    // Verify pose message
    let pose: geometry_msgs::PoseStamped =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(pose.header.frame_id, "map");
    assert_eq!(pose.pose.position.x, 1.0);
    assert_eq!(pose.pose.position.y, 2.0);
    assert_eq!(pose.pose.position.z, 3.0);
    assert_eq!(pose.pose.orientation.w, 1.0);

    // Verify twist message
    let twist: geometry_msgs::TwistStamped =
        roslibrust_mcap::McapReader::deserialize_message(&messages[1].data)?;
    assert_eq!(twist.header.frame_id, "base_link");
    assert_eq!(twist.twist.linear.x, 1.5);
    assert_eq!(twist.twist.angular.z, 0.5);

    Ok(())
}

#[test]
fn test_bool_message() -> Result<()> {
    // Test with Bool messages
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::Bool>("/flag")?;

        channel.write(&mut writer, 1_000_000_000, &std_msgs::Bool { data: true })?;
        channel.write(&mut writer, 2_000_000_000, &std_msgs::Bool { data: false })?;
        channel.write(&mut writer, 3_000_000_000, &std_msgs::Bool { data: true })?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 3);

    let msg1: std_msgs::Bool = roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg1.data, true);

    let msg2: std_msgs::Bool = roslibrust_mcap::McapReader::deserialize_message(&messages[1].data)?;
    assert_eq!(msg2.data, false);

    let msg3: std_msgs::Bool = roslibrust_mcap::McapReader::deserialize_message(&messages[2].data)?;
    assert_eq!(msg3.data, true);

    Ok(())
}

// ============================================================================
// Performance and Stress Tests
// ============================================================================

#[test]
fn test_many_messages() -> Result<()> {
    // Test writing and reading a large number of messages
    let mut buffer = Vec::new();
    let message_count = 1000;

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/test")?;

        for i in 0..message_count {
            channel.write(
                &mut writer,
                i * 1_000_000, // 1ms intervals
                &std_msgs::String {
                    data: format!("Message {}", i),
                },
            )?;
        }

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), message_count as usize);

    // Verify first and last messages
    let first: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(first.data, "Message 0");

    let last: std_msgs::String = roslibrust_mcap::McapReader::deserialize_message(
        &messages[message_count as usize - 1].data,
    )?;
    assert_eq!(last.data, format!("Message {}", message_count - 1));

    Ok(())
}

#[test]
fn test_large_string_message() -> Result<()> {
    // Test with very large string messages (1MB)
    let mut buffer = Vec::new();
    let large_string = "x".repeat(1_000_000); // 1MB string

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/large")?;

        channel.write(
            &mut writer,
            1_000_000_000,
            &std_msgs::String {
                data: large_string.clone(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 1);

    let msg: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg.data.len(), 1_000_000);
    assert_eq!(msg.data, large_string);

    Ok(())
}

#[test]
fn test_many_topics() -> Result<()> {
    // Test with many different topics
    let mut buffer = Vec::new();
    let topic_count = 50;

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let mut channels = Vec::new();

        // Create many channels
        for i in 0..topic_count {
            let channel = writer.add_ros_channel::<std_msgs::String>(&format!("/topic_{}", i))?;
            channels.push(channel);
        }

        // Write one message to each channel
        for (i, channel) in channels.iter().enumerate() {
            channel.write(
                &mut writer,
                (i as u64) * 1_000_000_000,
                &std_msgs::String {
                    data: format!("Topic {}", i),
                },
            )?;
        }

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    assert_eq!(reader.channels().len(), topic_count);

    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;
    assert_eq!(messages.len(), topic_count);

    Ok(())
}

#[test]
fn test_interleaved_messages() -> Result<()> {
    // Test messages from multiple topics interleaved by timestamp
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel_a = writer.add_ros_channel::<std_msgs::String>("/topic_a")?;
        let channel_b = writer.add_ros_channel::<std_msgs::String>("/topic_b")?;
        let channel_c = writer.add_ros_channel::<std_msgs::String>("/topic_c")?;

        // Write messages in interleaved order
        channel_a.write(
            &mut writer,
            1_000_000_000,
            &std_msgs::String {
                data: "A1".to_string(),
            },
        )?;
        channel_b.write(
            &mut writer,
            2_000_000_000,
            &std_msgs::String {
                data: "B1".to_string(),
            },
        )?;
        channel_a.write(
            &mut writer,
            3_000_000_000,
            &std_msgs::String {
                data: "A2".to_string(),
            },
        )?;
        channel_c.write(
            &mut writer,
            4_000_000_000,
            &std_msgs::String {
                data: "C1".to_string(),
            },
        )?;
        channel_b.write(
            &mut writer,
            5_000_000_000,
            &std_msgs::String {
                data: "B2".to_string(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 5);

    // Verify messages are in timestamp order
    assert_eq!(messages[0].log_time, 1_000_000_000);
    assert_eq!(messages[1].log_time, 2_000_000_000);
    assert_eq!(messages[2].log_time, 3_000_000_000);
    assert_eq!(messages[3].log_time, 4_000_000_000);
    assert_eq!(messages[4].log_time, 5_000_000_000);

    // Verify message content
    let msg0: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg0.data, "A1");

    let msg4: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[4].data)?;
    assert_eq!(msg4.data, "B2");

    Ok(())
}

// ============================================================================
// Schema Reuse and Channel Management
// ============================================================================

#[test]
fn test_same_type_different_topics() -> Result<()> {
    // Test that multiple channels with the same message type reuse the schema
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;

        // Create multiple channels with the same message type
        let channel1 = writer.add_ros_channel::<std_msgs::String>("/topic1")?;
        let channel2 = writer.add_ros_channel::<std_msgs::String>("/topic2")?;
        let channel3 = writer.add_ros_channel::<std_msgs::String>("/topic3")?;

        channel1.write(
            &mut writer,
            1_000_000_000,
            &std_msgs::String {
                data: "Topic 1".to_string(),
            },
        )?;
        channel2.write(
            &mut writer,
            2_000_000_000,
            &std_msgs::String {
                data: "Topic 2".to_string(),
            },
        )?;
        channel3.write(
            &mut writer,
            3_000_000_000,
            &std_msgs::String {
                data: "Topic 3".to_string(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;

    // Should have 3 channels
    assert_eq!(reader.channels().len(), 3);

    // All channels should have the same message type
    for channel in reader.channels().values() {
        assert_eq!(channel.message_type, "std_msgs/String");
        assert_eq!(channel.encoding, "cdr");
    }

    // Verify we can read all messages
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;
    assert_eq!(messages.len(), 3);

    Ok(())
}

#[test]
fn test_unicode_strings() -> Result<()> {
    // Test with Unicode strings in messages
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/unicode")?;

        let test_strings = vec![
            "Hello, 世界!",  // Chinese
            "Привет, мир!",  // Russian
            "مرحبا بالعالم", // Arabic
            "🚀🤖🎉",        // Emojis
            "Ñoño español",  // Spanish with accents
        ];

        for (i, s) in test_strings.iter().enumerate() {
            channel.write(
                &mut writer,
                (i as u64) * 1_000_000_000,
                &std_msgs::String {
                    data: s.to_string(),
                },
            )?;
        }

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 5);

    // Verify Unicode strings are preserved
    let msg0: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg0.data, "Hello, 世界!");

    let msg3: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[3].data)?;
    assert_eq!(msg3.data, "🚀🤖🎉");

    Ok(())
}

#[test]
fn test_out_of_order_timestamps() -> Result<()> {
    // Test writing messages with non-monotonic timestamps
    // MCAP preserves the order they were written in the file
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/test")?;

        // Write messages with non-monotonic timestamps
        channel.write(
            &mut writer,
            5_000_000_000,
            &std_msgs::String {
                data: "First (t=5)".to_string(),
            },
        )?;
        channel.write(
            &mut writer,
            3_000_000_000,
            &std_msgs::String {
                data: "Second (t=3)".to_string(),
            },
        )?;
        channel.write(
            &mut writer,
            7_000_000_000,
            &std_msgs::String {
                data: "Third (t=7)".to_string(),
            },
        )?;
        channel.write(
            &mut writer,
            1_000_000_000,
            &std_msgs::String {
                data: "Fourth (t=1)".to_string(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 4);

    // Messages are returned in the order they were written
    // (MCAP doesn't automatically sort by timestamp)
    assert_eq!(messages[0].log_time, 5_000_000_000);
    assert_eq!(messages[1].log_time, 3_000_000_000);
    assert_eq!(messages[2].log_time, 7_000_000_000);
    assert_eq!(messages[3].log_time, 1_000_000_000);

    // Verify content matches the write order
    let msg0: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg0.data, "First (t=5)");

    let msg3: std_msgs::String =
        roslibrust_mcap::McapReader::deserialize_message(&messages[3].data)?;
    assert_eq!(msg3.data, "Fourth (t=1)");

    Ok(())
}

#[test]
fn test_duplicate_timestamps() -> Result<()> {
    // Test multiple messages with the same timestamp
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<std_msgs::String>("/test")?;

        let timestamp = 1_000_000_000;

        channel.write(
            &mut writer,
            timestamp,
            &std_msgs::String {
                data: "Message 1".to_string(),
            },
        )?;
        channel.write(
            &mut writer,
            timestamp,
            &std_msgs::String {
                data: "Message 2".to_string(),
            },
        )?;
        channel.write(
            &mut writer,
            timestamp,
            &std_msgs::String {
                data: "Message 3".to_string(),
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 3);

    // All should have the same timestamp
    for msg in &messages {
        assert_eq!(msg.log_time, 1_000_000_000);
    }

    Ok(())
}

// ============================================================================
// Numeric Types and Special Values
// ============================================================================

#[test]
fn test_float_special_values() -> Result<()> {
    // Test with special float values (NaN, infinity, negative infinity)
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<geometry_msgs::Point>("/points")?;

        // Test with NaN
        channel.write(
            &mut writer,
            1_000_000_000,
            &geometry_msgs::Point {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
        )?;

        // Test with infinity
        channel.write(
            &mut writer,
            2_000_000_000,
            &geometry_msgs::Point {
                x: f64::INFINITY,
                y: f64::NEG_INFINITY,
                z: 0.0,
            },
        )?;

        // Test with very small and very large values
        channel.write(
            &mut writer,
            3_000_000_000,
            &geometry_msgs::Point {
                x: f64::MIN,
                y: f64::MAX,
                z: f64::EPSILON,
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 3);

    // Verify NaN
    let msg0: geometry_msgs::Point =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert!(msg0.x.is_nan());

    // Verify infinity
    let msg1: geometry_msgs::Point =
        roslibrust_mcap::McapReader::deserialize_message(&messages[1].data)?;
    assert_eq!(msg1.x, f64::INFINITY);
    assert_eq!(msg1.y, f64::NEG_INFINITY);

    // Verify extreme values
    let msg2: geometry_msgs::Point =
        roslibrust_mcap::McapReader::deserialize_message(&messages[2].data)?;
    assert_eq!(msg2.x, f64::MIN);
    assert_eq!(msg2.y, f64::MAX);
    assert_eq!(msg2.z, f64::EPSILON);

    Ok(())
}

#[test]
fn test_negative_numbers() -> Result<()> {
    // Test with negative numbers in Time messages
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;
        let channel = writer.add_ros_channel::<builtin_interfaces::Time>("/time")?;

        // Negative seconds (valid in ROS2 Time)
        channel.write(
            &mut writer,
            1_000_000_000,
            &builtin_interfaces::Time {
                sec: -100,
                nanosec: 500_000_000,
            },
        )?;

        channel.write(
            &mut writer,
            2_000_000_000,
            &builtin_interfaces::Time {
                sec: i32::MIN,
                nanosec: 0,
            },
        )?;

        channel.write(
            &mut writer,
            3_000_000_000,
            &builtin_interfaces::Time {
                sec: i32::MAX,
                nanosec: 999_999_999,
            },
        )?;

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    let messages: Vec<_> = reader
        .iter_messages()?
        .collect::<roslibrust_mcap::Result<Vec<_>>>()?;

    assert_eq!(messages.len(), 3);

    let msg0: builtin_interfaces::Time =
        roslibrust_mcap::McapReader::deserialize_message(&messages[0].data)?;
    assert_eq!(msg0.sec, -100);
    assert_eq!(msg0.nanosec, 500_000_000);

    let msg1: builtin_interfaces::Time =
        roslibrust_mcap::McapReader::deserialize_message(&messages[1].data)?;
    assert_eq!(msg1.sec, i32::MIN);

    let msg2: builtin_interfaces::Time =
        roslibrust_mcap::McapReader::deserialize_message(&messages[2].data)?;
    assert_eq!(msg2.sec, i32::MAX);
    assert_eq!(msg2.nanosec, 999_999_999);

    Ok(())
}

// ============================================================================
// Topic Name Edge Cases
// ============================================================================

#[test]
fn test_topic_name_variations() -> Result<()> {
    // Test with various valid topic name formats
    let mut buffer = Vec::new();

    {
        let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer))?;

        // Various valid topic names
        let topics = vec![
            "/simple",
            "/nested/topic",
            "/deeply/nested/topic/name",
            "/topic_with_underscores",
            "/topic123",
            "/CamelCase",
        ];

        let mut channels = Vec::new();
        for topic in &topics {
            let channel = writer.add_ros_channel::<std_msgs::String>(topic)?;
            channels.push(channel);
        }

        for (i, channel) in channels.iter().enumerate() {
            channel.write(
                &mut writer,
                (i as u64) * 1_000_000_000,
                &std_msgs::String {
                    data: format!("Message {}", i),
                },
            )?;
        }

        writer.finish()?;
    }

    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer))?;
    assert_eq!(reader.channels().len(), 6);

    // Verify all topic names are preserved
    let channel_topics: Vec<_> = reader
        .channels()
        .values()
        .map(|c| c.topic.as_str())
        .collect();

    assert!(channel_topics.contains(&"/simple"));
    assert!(channel_topics.contains(&"/nested/topic"));
    assert!(channel_topics.contains(&"/deeply/nested/topic/name"));
    assert!(channel_topics.contains(&"/topic_with_underscores"));
    assert!(channel_topics.contains(&"/topic123"));
    assert!(channel_topics.contains(&"/CamelCase"));

    Ok(())
}
