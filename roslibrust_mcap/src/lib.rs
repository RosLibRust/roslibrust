//! # roslibrust_mcap
//!
//! This crate provides utilities for reading and writing [MCAP](https://mcap.dev/) files
//! with ROS message support, integrating with the roslibrust ecosystem.
//!
//! MCAP is a modular container format for heterogeneous timestamped data, commonly used
//! for recording and playing back ROS messages.
//!
//! ## Features
//!
//! - Read MCAP files and deserialize ROS messages
//! - Write MCAP files with ROS messages
//! - Support for both ROS1 and ROS2 message formats
//! - Integration with roslibrust's `RosMessageType` trait
//!
//! ## Reading MCAP Files
//!
//! ```rust,no_run
//! use roslibrust_mcap::McapReader;
//! use std::fs::File;
//! # use roslibrust_mcap::Result;
//!
//! # fn main() -> Result<()> {
//! let file = File::open("recording.mcap")?;
//! let reader = McapReader::new(file)?;
//!
//! for message in reader.iter_messages()? {
//!     let msg = message?;
//!     println!("Channel: {}, Time: {}", msg.channel_id, msg.log_time);
//!     // Deserialize message data based on channel schema
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Writing MCAP Files
//!
//! ```rust,no_run
//! use roslibrust_mcap::McapWriter;
//! use std::fs::File;
//! # use roslibrust_mcap::Result;
//! # use roslibrust_common::RosMessageType;
//! # #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
//! # struct String { data: std::string::String }
//! # impl RosMessageType for String {
//! #     const ROS_TYPE_NAME: &'static str = "std_msgs/String";
//! #     const MD5SUM: &'static str = "";
//! #     const DEFINITION: &'static str = "";
//! # }
//!
//! # fn main() -> Result<()> {
//! let file = File::create("output.mcap")?;
//! let mut writer = McapWriter::new(file)?;
//!
//! // Add a channel for your message type
//! let channel = writer.add_ros_channel::<String>("/my_topic")?;
//!
//! // Write messages
//! # let my_message = String { data: "hello".to_string() };
//! # let timestamp = 1_000_000_000;
//! channel.write(&mut writer, timestamp, &my_message)?;
//!
//! writer.finish()?;
//! # Ok(())
//! # }
//! ```

mod error;
mod reader;
mod writer;

pub use error::{McapError, Result};
pub use reader::McapReader;
pub use writer::{Channel, McapWriter};

/// Re-export the underlying mcap crate for advanced usage
pub use mcap;

/// Metadata about a channel in an MCAP file
#[derive(Debug, Clone)]
pub struct ChannelInfo {
    /// The channel ID
    pub id: u16,
    /// The topic name
    pub topic: String,
    /// The message type (e.g., "std_msgs/String")
    pub message_type: String,
    /// The message encoding (e.g., "cdr" or "json")
    pub encoding: String,
}

/// A message read from an MCAP file
#[derive(Debug)]
pub struct McapMessage {
    /// The channel this message was published on
    pub channel_id: u16,
    /// The sequence number of this message
    pub sequence: u32,
    /// The time the message was logged
    pub log_time: u64,
    /// The time the message was published
    pub publish_time: u64,
    /// The raw message data
    pub data: Vec<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_write_and_read_mcap() -> Result<()> {
        // Create a simple test message type
        use roslibrust_common::RosMessageType;
        use serde::{Deserialize, Serialize};

        #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
        struct TestMessage {
            data: String,
            value: i32,
        }

        impl RosMessageType for TestMessage {
            const ROS_TYPE_NAME: &'static str = "test_msgs/TestMessage";
            const MD5SUM: &'static str = "test";
            const DEFINITION: &'static str = "string data\nint32 value";
            const ROS2_TYPE_NAME: &'static str = "test_msgs/msg/TestMessage";
        }

        // Write an MCAP file
        let mut buffer = Vec::new();
        {
            let mut writer = McapWriter::new(Cursor::new(&mut buffer))?;

            // Add a channel for CDR encoding
            let channel = writer.add_ros_channel::<TestMessage>("/test/topic")?;

            // Write some messages
            let msg1 = TestMessage {
                data: "Hello".to_string(),
                value: 42,
            };
            let msg2 = TestMessage {
                data: "World".to_string(),
                value: 123,
            };

            channel.write(&mut writer, 1000, &msg1)?;
            channel.write(&mut writer, 2000, &msg2)?;

            writer.finish()?;
        }

        // Read the MCAP file back
        let reader = McapReader::new(Cursor::new(&buffer))?;

        // Check channels
        assert_eq!(reader.channels().len(), 1);
        let channel = reader.channel(1).expect("Channel 1 should exist");
        assert_eq!(channel.topic, "/test/topic");
        assert_eq!(channel.message_type, "test_msgs/TestMessage");
        assert_eq!(channel.encoding, "cdr");

        // Read messages
        let messages: Vec<_> = reader.iter_messages()?.collect::<Result<Vec<_>>>()?;
        assert_eq!(messages.len(), 2);

        // Verify first message
        assert_eq!(messages[0].channel_id, 1);
        assert_eq!(messages[0].log_time, 1000);
        let decoded1: TestMessage = McapReader::deserialize_message(&messages[0].data)?;
        assert_eq!(decoded1.data, "Hello");
        assert_eq!(decoded1.value, 42);

        // Verify second message
        assert_eq!(messages[1].channel_id, 1);
        assert_eq!(messages[1].log_time, 2000);
        let decoded2: TestMessage = McapReader::deserialize_message(&messages[1].data)?;
        assert_eq!(decoded2.data, "World");
        assert_eq!(decoded2.value, 123);

        Ok(())
    }

    #[test]
    fn test_basic_functionality() {
        // Basic smoke test to ensure the crate compiles
        assert!(true);
    }
}
