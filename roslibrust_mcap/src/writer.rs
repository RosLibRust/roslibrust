//! MCAP file writing functionality

use crate::{McapError, Result};
use mcap::{records::MessageHeader, Writer};
use roslibrust_common::RosMessageType;
use std::collections::BTreeMap;
use std::io::{Seek, Write};
use std::marker::PhantomData;

/// A writer for creating MCAP files with ROS messages
///
/// This writer only supports CDR encoding (ROS2 format).
pub struct McapWriter<W: Write + Seek> {
    writer: Writer<W>,
    channels: BTreeMap<String, u16>,
    schemas: BTreeMap<String, u16>,
}

/// A channel for writing messages of a specific type to an MCAP file
///
/// This type-safe handle ensures you can only write messages of the correct type
/// to the channel.
pub struct Channel<T: RosMessageType> {
    channel_id: u16,
    _phantom: PhantomData<T>,
}

impl<T: RosMessageType> Channel<T> {
    /// Write a message to this channel
    ///
    /// # Arguments
    ///
    /// * `writer` - The MCAP writer to write to
    /// * `log_time` - The timestamp when the message was logged (nanoseconds)
    /// * `message` - The ROS message to write
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// # use roslibrust_mcap::{McapWriter, Result};
    /// # use roslibrust_common::RosMessageType;
    /// # use std::fs::File;
    /// # #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
    /// # struct String { data: std::string::String }
    /// # impl RosMessageType for String {
    /// #     const ROS_TYPE_NAME: &'static str = "std_msgs/String";
    /// #     const MD5SUM: &'static str = "";
    /// #     const DEFINITION: &'static str = "";
    /// # }
    /// # fn main() -> Result<()> {
    /// # let file = File::create("output.mcap")?;
    /// # let mut writer = McapWriter::new(file)?;
    /// # let my_message = String { data: "hello".to_string() };
    /// # let timestamp = 1_000_000_000;
    /// let channel = writer.add_ros_channel::<String>("/chatter")?;
    /// channel.write(&mut writer, timestamp, &my_message)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn write<W: Write + Seek>(
        &self,
        writer: &mut McapWriter<W>,
        log_time: u64,
        message: &T,
    ) -> Result<()> {
        // Serialize the message using CDR
        let data = cdr::serialize::<_, _, cdr::CdrLe>(message, cdr::Infinite)
            .map_err(|e| McapError::Serialization(format!("CDR serialization failed: {}", e)))?;

        writer.write_message_internal(self.channel_id, log_time, &data)
    }
}

impl<W: Write + Seek> McapWriter<W> {
    /// Create a new MCAP writer with CDR encoding (ROS2 default)
    ///
    /// # Arguments
    ///
    /// * `writer` - A writable destination for MCAP data
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use std::fs::File;
    /// use roslibrust_mcap::McapWriter;
    /// # use roslibrust_mcap::Result;
    ///
    /// # fn main() -> Result<()> {
    /// let file = File::create("output.mcap")?;
    /// let mut writer = McapWriter::new(file)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn new(writer: W) -> Result<Self> {
        let mcap_writer = Writer::new(writer)?;
        Ok(Self {
            writer: mcap_writer,
            channels: BTreeMap::new(),
            schemas: BTreeMap::new(),
        })
    }

    /// Add a channel for a specific ROS message type
    ///
    /// This registers the message type and creates a channel for it.
    /// Returns a type-safe channel handle that can be used to write messages.
    ///
    /// # Arguments
    ///
    /// * `topic` - The topic name (e.g., "/chatter")
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// # use roslibrust_mcap::{McapWriter, Result};
    /// # use roslibrust_common::RosMessageType;
    /// # use std::fs::File;
    /// # #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
    /// # struct String { data: std::string::String }
    /// # impl RosMessageType for String {
    /// #     const ROS_TYPE_NAME: &'static str = "std_msgs/String";
    /// #     const MD5SUM: &'static str = "";
    /// #     const DEFINITION: &'static str = "";
    /// # }
    /// # fn main() -> Result<()> {
    /// # let file = File::create("output.mcap")?;
    /// # let mut writer = McapWriter::new(file)?;
    /// # let my_message = String { data: "hello".to_string() };
    /// # let timestamp = 1_000_000_000;
    /// let channel = writer.add_ros_channel::<String>("/chatter")?;
    /// channel.write(&mut writer, timestamp, &my_message)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn add_ros_channel<T: RosMessageType>(&mut self, topic: &str) -> Result<Channel<T>> {
        // Check if we already have this channel
        let channel_key = format!("{}:cdr", topic);
        if let Some(&channel_id) = self.channels.get(&channel_key) {
            return Ok(Channel {
                channel_id,
                _phantom: PhantomData,
            });
        }

        // Get or create schema for this message type
        let schema_key = T::ROS_TYPE_NAME.to_string();
        let schema_id = if let Some(&id) = self.schemas.get(&schema_key) {
            id
        } else {
            // Create a new schema using the Writer API
            // For CDR, we use the ROS message definition
            let schema_data = T::DEFINITION.as_bytes().to_vec();
            let schema_id = self
                .writer
                .add_schema(T::ROS_TYPE_NAME, "ros2msg", &schema_data)?;
            self.schemas.insert(schema_key, schema_id);
            schema_id
        };

        // Create the channel using the Writer API
        let metadata = BTreeMap::new();
        let channel_id = self
            .writer
            .add_channel(schema_id, topic, "cdr", &metadata)?;

        self.channels.insert(channel_key, channel_id);
        Ok(Channel {
            channel_id,
            _phantom: PhantomData,
        })
    }

    /// Write a message to a channel (internal method)
    ///
    /// This is used internally by the Channel type. Users should use
    /// `channel.write()` instead.
    fn write_message_internal(
        &mut self,
        channel_id: u16,
        log_time: u64,
        data: &[u8],
    ) -> Result<()> {
        let header = MessageHeader {
            channel_id,
            sequence: 0, // Could be tracked per channel if needed
            log_time,
            publish_time: log_time, // Using same time for both
        };

        self.writer.write_to_known_channel(&header, data)?;
        Ok(())
    }

    /// Finish writing the MCAP file
    ///
    /// This must be called to properly close the file and write the summary section.
    pub fn finish(mut self) -> Result<()> {
        self.writer.finish()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_writer_creation() {
        // Use Cursor which implements both Write and Seek
        let buffer = Cursor::new(Vec::new());
        let result = McapWriter::new(buffer);
        assert!(result.is_ok());
    }
}
