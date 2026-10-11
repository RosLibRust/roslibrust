//! MCAP file reading functionality

use crate::{ChannelInfo, McapError, McapMessage, Result};
use mcap::{
    records::{op, Record},
    sans_io::{LinearReadEvent, LinearReader, LinearReaderOptions},
    MessageStream,
};
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
    /// An error will be returned if any channel uses a different encoding or the
    /// file is malformed. Construction scans all records, validates available
    /// CRCs, and discovers channels even when the file has no summary.
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

        // A missing summary is valid; a corrupt summary is not. Parse it even
        // though channel discovery below uses records, since summaries can omit
        // repeated schemas/channels and must not be our only source of metadata.
        mcap::read::Summary::read(&data)?;
        let channels = read_channels(&data)?;

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

// Use the configurable record reader so construction checks data-section CRCs
// as well as chunk/summary CRCs. The convenience MessageStream omits data CRC
// validation and does not expose channels that have no messages.
fn read_channels(data: &[u8]) -> Result<BTreeMap<u16, ChannelInfo>> {
    let mut reader = LinearReader::new_with_options(
        LinearReaderOptions::default()
            // Reserve space for opcode/length framing without limiting compression expansion.
            .with_record_length_limit(usize::MAX - 9)
            .with_validate_chunk_crcs(true)
            .with_validate_data_section_crc(true)
            .with_validate_summary_section_crc(true)
            .with_check_finishes_after_end_magic(true),
    );
    let mut remaining = data;
    let mut schemas = BTreeMap::new();
    let mut channels = BTreeMap::new();
    let mut has_header = false;
    let mut has_data_end = false;

    while let Some(event) = reader.next_event() {
        match event? {
            LinearReadEvent::ReadRequest(needed) => {
                let count = needed.min(remaining.len());
                reader.insert(count).copy_from_slice(&remaining[..count]);
                reader.notify_read(count);
                remaining = &remaining[count..];
            }
            LinearReadEvent::Record { opcode, data } => {
                if opcode == op::ATTACHMENT {
                    validate_attachment_bounds(data)?;
                }
                let record = mcap::read::parse_record(opcode, data)?;
                if !has_header && !matches!(record, Record::Header(_)) {
                    return Err(McapError::Deserialization("Missing MCAP header".into()));
                }
                match record {
                    Record::Header(_) => {
                        if has_header {
                            return Err(McapError::Deserialization("Duplicate MCAP header".into()));
                        }
                        has_header = true;
                    }
                    Record::Schema { header, data } => {
                        if header.id == 0 {
                            return Err(mcap::McapError::InvalidSchemaId.into());
                        }
                        let schema = (header.clone(), data.into_owned());
                        if let Some(existing) = schemas.get(&header.id) {
                            if existing != &schema {
                                return Err(mcap::McapError::ConflictingSchemas(header.name).into());
                            }
                        } else {
                            schemas.insert(header.id, schema);
                        }
                    }
                    Record::Channel(channel) => {
                        if channel.message_encoding != "cdr" {
                            return Err(McapError::InvalidEncoding(format!(
                                "Channel {} uses message encoding '{}'; expected 'cdr'",
                                channel.id, channel.message_encoding
                            )));
                        }
                        if channel.schema_id != 0 {
                            let (schema, _) = schemas.get(&channel.schema_id).ok_or_else(|| {
                                mcap::McapError::UnknownSchema(
                                    channel.topic.clone(),
                                    channel.schema_id,
                                )
                            })?;
                            if schema.encoding != "ros2msg" {
                                return Err(McapError::InvalidEncoding(format!(
                                    "Channel {} uses schema encoding '{}'; expected 'ros2msg'",
                                    channel.id, schema.encoding
                                )));
                            }
                        }
                        if let Some(existing) = channels.get(&channel.id) {
                            if existing != &channel {
                                return Err(
                                    mcap::McapError::ConflictingChannels(channel.topic).into()
                                );
                            }
                        } else {
                            channels.insert(channel.id, channel);
                        }
                    }
                    Record::Message { header, .. } => {
                        if has_data_end {
                            return Err(McapError::Deserialization(
                                "Message after MCAP data end".into(),
                            ));
                        }
                        if !channels.contains_key(&header.channel_id) {
                            return Err(mcap::McapError::UnknownChannel(
                                header.sequence,
                                header.channel_id,
                            )
                            .into());
                        }
                    }
                    Record::DataEnd(_) => {
                        if has_data_end {
                            return Err(McapError::Deserialization(
                                "Duplicate MCAP data end".into(),
                            ));
                        }
                        has_data_end = true;
                    }
                    Record::Footer(_) if !has_data_end => {
                        return Err(McapError::Deserialization("Missing MCAP data end".into()));
                    }
                    _ => {}
                }
            }
        }
    }

    Ok(channels
        .into_iter()
        .map(|(id, channel)| {
            let message_type = schemas
                .get(&channel.schema_id)
                .map(|(schema, _)| schema.name.clone())
                .unwrap_or_else(|| "unknown".into());
            (
                id,
                ChannelInfo {
                    id,
                    topic: channel.topic,
                    message_type,
                    encoding: channel.message_encoding,
                },
            )
        })
        .collect())
}

// mcap 0.24's attachment parser slices off the trailing CRC without checking
// that the variable-length header and data length leave room for it first.
fn validate_attachment_bounds(data: &[u8]) -> Result<()> {
    let mut offset: usize = 16; // Log and creation times.
    for _ in 0..2 {
        // Name and media type, each prefixed with a u32 length.
        let end = offset
            .checked_add(4)
            .ok_or(mcap::McapError::UnexpectedEof)?;
        let bytes = data
            .get(offset..end)
            .ok_or(mcap::McapError::UnexpectedEof)?;
        let len = u32::from_le_bytes(bytes.try_into().unwrap()) as usize;
        offset = end.checked_add(len).ok_or(mcap::McapError::UnexpectedEof)?;
    }
    let end = offset
        .checked_add(12)
        .ok_or(mcap::McapError::UnexpectedEof)?;
    if end > data.len() {
        // u64 data length and u32 CRC.
        return Err(mcap::McapError::UnexpectedEof.into());
    }
    Ok(())
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
        // An empty buffer is not a valid, finished MCAP file
        let data: &[u8] = &[];
        let result = McapReader::new(data);
        assert!(matches!(result, Err(McapError::Mcap(_))));
    }
}
