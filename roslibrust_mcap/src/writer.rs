//! MCAP file writing functionality

use crate::{McapError, Result};
use mcap::{records::MessageHeader, Writer};
use roslibrust_common::RosMessageType;
use std::collections::BTreeMap;
use std::io::{Seek, Write};
use std::marker::PhantomData;
use std::sync::Arc;

/// A writer for creating MCAP files with ROS messages
///
/// This writer only supports CDR encoding (ROS2 format).
pub struct McapWriter<W: Write + Seek> {
    writer: Writer<W>,
    channels: BTreeMap<String, (u16, SchemaIdentity)>,
    schemas: BTreeMap<&'static str, (u16, SchemaIdentity)>,
    identity: Arc<()>,
}

/// A channel for writing messages of a specific type to an MCAP file
///
/// This type-safe handle ensures you can only write messages of the correct type
/// to the channel. A handle belongs to the writer that created it; writing through
/// another writer returns [`McapError::ForeignChannel`].
pub struct Channel<T: RosMessageType> {
    channel_id: u16,
    writer_identity: Arc<()>,
    _phantom: PhantomData<T>,
}

// Compare all declared ROS schema metadata, not just the name. Identical schemas
// may be represented by different Rust types, but conflicting definitions or
// hashes must never share a channel or schema record.
#[derive(Clone, Copy, PartialEq, Eq)]
struct SchemaIdentity {
    name: &'static str,
    definition: &'static str,
    md5sum: &'static str,
    ros2_name: &'static str,
    ros2_hash: &'static [u8; 32],
}

impl SchemaIdentity {
    fn of<T: RosMessageType>() -> Self {
        Self {
            name: T::DESCRIPTION.ros_type_name,
            definition: T::DESCRIPTION.definition,
            md5sum: T::DESCRIPTION.md5sum,
            ros2_name: T::DESCRIPTION.ros2_type_name,
            ros2_hash: T::DESCRIPTION.ros2_hash,
        }
    }
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
    /// # #[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
    /// # struct String { data: std::string::String }
    /// # impl RosMessageType for String {
    /// #     const DESCRIPTION: roslibrust_common::MessageDescriptor =
    /// #         roslibrust_common::MessageDescriptor::new::<Self>("std_msgs/String", "", "", "", &[0; 32]);
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
        if !Arc::ptr_eq(&self.writer_identity, &writer.identity) {
            return Err(McapError::ForeignChannel(self.channel_id));
        }

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
            identity: Arc::new(()),
        })
    }

    /// Add a channel for a specific ROS message type
    ///
    /// This registers the message type and creates a channel for it.
    /// Returns a type-safe channel handle that can be used to write messages.
    /// Reusing a topic requires identical ROS schema metadata. Reusing a schema
    /// name with different metadata also returns an error.
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
    /// # #[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
    /// # struct String { data: std::string::String }
    /// # impl RosMessageType for String {
    /// #     const DESCRIPTION: roslibrust_common::MessageDescriptor =
    /// #         roslibrust_common::MessageDescriptor::new::<Self>("std_msgs/String", "", "", "", &[0; 32]);
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
        let schema = SchemaIdentity::of::<T>();
        if let Some(&(channel_id, existing)) = self.channels.get(topic) {
            if existing != schema {
                return Err(McapError::IncompatibleChannel(topic.to_string()));
            }
            return Ok(Channel {
                channel_id,
                writer_identity: self.identity.clone(),
                _phantom: PhantomData,
            });
        }

        let schema_id = if let Some(&(id, existing)) = self.schemas.get(schema.name) {
            if existing != schema {
                return Err(McapError::IncompatibleSchema(schema.name.to_string()));
            }
            id
        } else {
            let id =
                self.writer
                    .add_schema(schema.name, "ros2msg", schema.definition.as_bytes())?;
            self.schemas.insert(schema.name, (id, schema));
            id
        };

        let channel_id = self
            .writer
            .add_channel(schema_id, topic, "cdr", &BTreeMap::new())?;
        self.channels
            .insert(topic.to_string(), (channel_id, schema));
        Ok(Channel {
            channel_id,
            writer_identity: self.identity.clone(),
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
