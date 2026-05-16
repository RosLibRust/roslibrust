//! MCAP file reading functionality

use crate::{ChannelInfo, McapError, McapMessage, Result};
use mcap::MessageStream;
use std::collections::BTreeMap;
use std::io::Read;

/// A reader for MCAP files that provides access to messages and metadata
///
/// This reader loads the entire MCAP file into memory for efficient access.
/// For very large files, consider using a memory-mapped approach.
pub struct McapReader {
    data: Vec<u8>,
    channels: BTreeMap<u16, ChannelInfo>,
}

impl McapReader {
    /// Create a new MCAP reader from a readable source
    ///
    /// This reader only supports CDR-encoded messages with ros2msg schema encoding.
    /// An error will be returned if any channel uses a different encoding.
    ///
    /// # Arguments
    ///
    /// * `reader` - A readable source containing MCAP data
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use std::fs::File;
    /// use roslibrust_mcap::McapReader;
    /// # use roslibrust_mcap::Result;
    ///
    /// # fn main() -> Result<()> {
    /// let file = File::open("recording.mcap")?;
    /// let reader = McapReader::new(file)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn new<R: Read>(mut reader: R) -> Result<Self> {
        let mut data = Vec::new();
        reader.read_to_end(&mut data)?;

        let mut channels = BTreeMap::new();

        // Parse the MCAP file to extract channel and schema information
        // We use the summary section if available for efficient access
        if let Ok(Some(summary)) = mcap::read::Summary::read(&data) {
            // Extract channel information
            for (channel_id, channel) in summary.channels.iter() {
                // Validate that the channel uses CDR encoding
                if channel.message_encoding != "cdr" {
                    return Err(McapError::Deserialization(format!(
                        "Channel {} uses unsupported message encoding '{}'. Only 'cdr' is supported.",
                        channel_id, channel.message_encoding
                    )));
                }

                // Validate that the schema uses ros2msg encoding
                if let Some(schema) = &channel.schema {
                    if schema.encoding != "ros2msg" {
                        return Err(McapError::Deserialization(format!(
                            "Channel {} uses unsupported schema encoding '{}'. Only 'ros2msg' is supported.",
                            channel_id, schema.encoding
                        )));
                    }
                }

                let schema_name = channel.schema.as_ref().map(|s| s.name.clone());

                channels.insert(
                    *channel_id,
                    ChannelInfo {
                        id: *channel_id,
                        topic: channel.topic.clone(),
                        message_type: schema_name.unwrap_or_else(|| "unknown".to_string()),
                        encoding: channel.message_encoding.clone(),
                    },
                );
            }
        }

        Ok(Self { data, channels })
    }

    /// Get information about all channels in the MCAP file
    pub fn channels(&self) -> &BTreeMap<u16, ChannelInfo> {
        &self.channels
    }

    /// Get information about a specific channel
    pub fn channel(&self, id: u16) -> Option<&ChannelInfo> {
        self.channels.get(&id)
    }

    /// Create an iterator over all messages in the MCAP file
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// # use roslibrust_mcap::{McapReader, Result};
    /// # use std::fs::File;
    /// # fn main() -> Result<()> {
    /// # let file = File::open("recording.mcap")?;
    /// let reader = McapReader::new(file)?;
    /// for message in reader.iter_messages()? {
    ///     let msg = message?;
    ///     println!("Channel: {}, Time: {}", msg.channel_id, msg.log_time);
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub fn iter_messages(&self) -> Result<MessageIterator<'_>> {
        let stream = MessageStream::new(&self.data)?;
        Ok(MessageIterator { stream })
    }

    /// Deserialize a message from raw bytes using CDR encoding
    ///
    /// # Arguments
    ///
    /// * `data` - The raw message bytes (CDR encoded)
    pub fn deserialize_message<T: serde::de::DeserializeOwned>(data: &[u8]) -> Result<T> {
        cdr::deserialize::<T>(data)
            .map_err(|e| McapError::Deserialization(format!("CDR deserialization failed: {}", e)))
    }
}

/// Iterator over messages in an MCAP file
pub struct MessageIterator<'a> {
    stream: MessageStream<'a>,
}

impl<'a> Iterator for MessageIterator<'a> {
    type Item = Result<McapMessage>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.stream.next() {
            Some(Ok(message)) => Some(Ok(McapMessage {
                channel_id: message.channel.id,
                sequence: message.sequence,
                log_time: message.log_time,
                publish_time: message.publish_time,
                data: message.data.to_vec(),
            })),
            Some(Err(e)) => Some(Err(e.into())),
            None => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reader_creation() {
        // Test that we can create a reader with an empty buffer
        let data: &[u8] = &[];
        let result = McapReader::new(data);
        assert!(result.is_ok());
    }
}
