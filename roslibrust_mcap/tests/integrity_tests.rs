//! Regression coverage for schema identity, writer ownership, and file validation.

use std::{collections::BTreeMap, io::Cursor};

use mcap::{
    records::{op, MessageHeader},
    WriteOptions,
};
use roslibrust_common::{MessageDescriptor, RosMessageType};
use roslibrust_mcap::{McapError, McapReader, McapWriter, Result};
use roslibrust_test::ros2::std_msgs;
use serde::{Deserialize, Serialize};

// Test variants share the real generated payload, while deliberately changing one
// descriptor field at a time to exercise schema-conflict detection.
type Text = std_msgs::String;

macro_rules! message_variant {
    ($type:ident $(, $field:ident = $value:expr)* $(,)?) => {
        #[derive(Clone, Debug, Default, Serialize, Deserialize)]
        #[serde(transparent)]
        struct $type(std_msgs::String);

        impl RosMessageType for $type {
            const DESCRIPTION: MessageDescriptor = {
                #[allow(unused_mut)]
                let mut descriptor = std_msgs::String::DESCRIPTION;
                $(descriptor.$field = $value;)*
                MessageDescriptor::new::<Self>(
                    descriptor.ros_type_name,
                    descriptor.md5sum,
                    descriptor.definition,
                    descriptor.ros2_type_name,
                    descriptor.ros2_hash,
                )
            };
        }
    };
}

message_variant!(TextAlias);
message_variant!(
    ChangedDefinition,
    definition = std_msgs::Int32::DESCRIPTION.definition
);
message_variant!(ChangedMd5, md5sum = std_msgs::Int32::DESCRIPTION.md5sum);
message_variant!(
    ChangedRos2Name,
    ros2_type_name = std_msgs::Int32::DESCRIPTION.ros2_type_name
);
message_variant!(
    ChangedHash,
    ros2_hash = std_msgs::Int32::DESCRIPTION.ros2_hash
);

#[test]
fn topic_rejects_incompatible_types_without_affecting_original_channel() -> Result<()> {
    let mut buffer = Vec::new();
    let mut writer = McapWriter::new(Cursor::new(&mut buffer))?;
    let channel = writer.add_ros_channel::<std_msgs::String>("/topic")?;
    assert!(matches!(
        writer.add_ros_channel::<std_msgs::Int32>("/topic"),
        Err(McapError::IncompatibleChannel(_))
    ));
    channel.write(
        &mut writer,
        1,
        &std_msgs::String {
            data: "valid".into(),
        },
    )?;
    writer.finish()?;
    let reader = McapReader::new(buffer.as_slice())?;
    let message = reader.iter_messages()?.next().unwrap()?;
    let decoded: std_msgs::String = McapReader::deserialize_message(&message.data)?;
    assert_eq!(decoded.data, "valid");
    Ok(())
}

fn assert_schema_conflict<T: RosMessageType>() -> Result<()> {
    let mut writer = McapWriter::new(Cursor::new(Vec::new()))?;
    writer.add_ros_channel::<Text>("/original")?;
    assert!(matches!(
        writer.add_ros_channel::<T>("/original"),
        Err(McapError::IncompatibleChannel(_))
    ));
    assert!(matches!(
        writer.add_ros_channel::<T>("/other"),
        Err(McapError::IncompatibleSchema(_))
    ));
    writer.finish()
}

#[test]
fn schema_identity_includes_definition_and_ros_hashes_and_names() -> Result<()> {
    assert_schema_conflict::<ChangedDefinition>()?;
    assert_schema_conflict::<ChangedMd5>()?;
    assert_schema_conflict::<ChangedRos2Name>()?;
    assert_schema_conflict::<ChangedHash>()
}

#[test]
fn compatible_schema_and_topic_reuse_round_trip() -> Result<()> {
    let mut buffer = Vec::new();
    let mut writer = McapWriter::new(Cursor::new(&mut buffer))?;
    writer.add_ros_channel::<Text>("/topic")?;
    let alias = writer.add_ros_channel::<TextAlias>("/topic")?;
    let second = writer.add_ros_channel::<Text>("/second")?;
    alias.write(
        &mut writer,
        1,
        &TextAlias(std_msgs::String {
            data: "alias".into(),
        }),
    )?;
    second.write(
        &mut writer,
        2,
        &Text {
            data: "second".into(),
        },
    )?;
    writer.finish()?;
    let summary = mcap::read::Summary::read(&buffer)?.unwrap();
    assert_eq!(summary.schemas.len(), 1);
    assert_eq!(summary.channels.len(), 2);
    let reader = McapReader::new(buffer.as_slice())?;
    assert_eq!(
        reader.iter_messages()?.collect::<Result<Vec<_>>>()?.len(),
        2
    );
    Ok(())
}

#[test]
fn channel_rejects_foreign_writer_even_with_same_channel_id() -> Result<()> {
    let mut buffer = Vec::new();
    let mut owner = McapWriter::new(Cursor::new(Vec::new()))?;
    let channel = owner.add_ros_channel::<Text>("/topic")?;
    let mut other = McapWriter::new(Cursor::new(&mut buffer))?;
    other.add_ros_channel::<Text>("/topic")?;
    assert!(matches!(
        channel.write(&mut other, 1, &Text::default()),
        Err(McapError::ForeignChannel(_))
    ));
    // Moving the originating writer preserves the handle's ownership.
    let mut moved_owner = owner;
    channel.write(&mut moved_owner, 2, &Text::default())?;
    moved_owner.finish()?;
    other.finish()?;
    let reader = McapReader::new(buffer.as_slice())?;
    assert_eq!(reader.iter_messages()?.count(), 0);
    Ok(())
}

fn fixture(
    options: WriteOptions,
    message_encoding: &str,
    schema_encoding: &str,
    write_message: bool,
) -> Result<Vec<u8>> {
    let mut buffer = Vec::new();
    let mut writer = options.create(Cursor::new(&mut buffer))?;
    let schema = writer.add_schema(
        Text::DESCRIPTION.ros_type_name,
        schema_encoding,
        Text::DESCRIPTION.definition.as_bytes(),
    )?;
    let channel = writer.add_channel(schema, "/topic", message_encoding, &BTreeMap::new())?;
    if write_message {
        writer.write_to_known_channel(
            &MessageHeader {
                channel_id: channel,
                sequence: 0,
                log_time: 1,
                publish_time: 1,
            },
            &cdr::serialize::<_, _, cdr::CdrLe>(
                &Text {
                    data: "payload".into(),
                },
                cdr::Infinite,
            )
            .unwrap(),
        )?;
    }
    writer.finish()?;
    drop(writer);
    Ok(buffer)
}

fn no_summary() -> WriteOptions {
    WriteOptions::new()
        .emit_summary_records(false)
        .emit_summary_offsets(false)
}

#[test]
fn summary_free_files_discover_channels_with_and_without_messages() -> Result<()> {
    for chunks in [false, true] {
        for messages in [false, true] {
            let buffer = fixture(no_summary().use_chunks(chunks), "cdr", "ros2msg", messages)?;
            assert!(mcap::read::Summary::read(&buffer)?.is_none());
            let reader = McapReader::new(buffer.as_slice())?;
            assert_eq!(reader.channels().len(), 1);
            let channel = reader.channels().values().next().unwrap();
            assert_eq!(channel.topic, "/topic");
            assert_eq!(channel.message_type, Text::DESCRIPTION.ros_type_name);
            assert_eq!(
                reader.iter_messages()?.collect::<Result<Vec<_>>>()?.len(),
                usize::from(messages)
            );
        }
    }
    Ok(())
}

#[test]
fn summary_without_repeated_metadata_still_discovers_channels() -> Result<()> {
    let buffer = fixture(
        WriteOptions::new()
            .repeat_channels(false)
            .repeat_schemas(false),
        "cdr",
        "ros2msg",
        true,
    )?;
    assert!(mcap::read::Summary::read(&buffer)?
        .unwrap()
        .channels
        .is_empty());
    assert_eq!(McapReader::new(buffer.as_slice())?.channels().len(), 1);
    Ok(())
}

#[test]
fn unsupported_encodings_rejected_with_or_without_summary() -> Result<()> {
    for options in [WriteOptions::new(), no_summary()] {
        for (message, schema) in [("json", "ros2msg"), ("cdr", "jsonschema")] {
            let buffer = fixture(options.clone(), message, schema, false)?;
            assert!(matches!(
                McapReader::new(buffer.as_slice()),
                Err(McapError::InvalidEncoding(_))
            ));
        }
    }
    Ok(())
}

// Locate top-level records for controlled corruption of otherwise valid files.
fn record_offset(buffer: &[u8], opcode: u8) -> usize {
    let mut offset = mcap::MAGIC.len();
    while offset < buffer.len() - mcap::MAGIC.len() {
        if buffer[offset] == opcode {
            return offset;
        }
        let length =
            u64::from_le_bytes(buffer[offset + 1..offset + 9].try_into().unwrap()) as usize;
        offset += 9 + length;
    }
    panic!("record {opcode} not found")
}

#[test]
fn malformed_magic_truncation_and_record_lengths_are_rejected() -> Result<()> {
    assert!(McapReader::new(&b""[..]).is_err());
    assert!(McapReader::new(&b"not an MCAP"[..]).is_err());
    let buffer = fixture(no_summary().use_chunks(false), "cdr", "ros2msg", true)?;
    assert!(McapReader::new(&buffer[..buffer.len() - 1]).is_err());
    let mut bad_magic = buffer.clone();
    bad_magic[0] = 0;
    assert!(matches!(
        McapReader::new(bad_magic.as_slice()),
        Err(McapError::Mcap(mcap::McapError::BadMagic))
    ));
    let mut bad_length = buffer.clone();
    let channel = record_offset(&bad_length, op::CHANNEL);
    bad_length[channel + 1..channel + 9].copy_from_slice(&u64::MAX.to_le_bytes());
    assert!(McapReader::new(bad_length.as_slice()).is_err());
    let mut trailing = buffer;
    trailing.push(0);
    assert!(McapReader::new(trailing.as_slice()).is_err());
    Ok(())
}

#[test]
fn corrupt_summary_is_not_treated_as_missing() -> Result<()> {
    let mut buffer = fixture(WriteOptions::new(), "cdr", "ros2msg", true)?;
    let footer = record_offset(&buffer, op::FOOTER);
    buffer[footer + 9 + 16] ^= 1; // Summary CRC.
    assert!(matches!(
        McapReader::new(buffer.as_slice()),
        Err(McapError::Mcap(mcap::McapError::BadSummaryCrc { .. }))
    ));
    Ok(())
}

#[test]
fn data_and_chunk_crc_corruption_is_reported_during_construction() -> Result<()> {
    let mut data = fixture(no_summary().use_chunks(false), "cdr", "ros2msg", true)?;
    let end = record_offset(&data, op::DATA_END);
    data[end + 9] ^= 1;
    assert!(matches!(
        McapReader::new(data.as_slice()),
        Err(McapError::Mcap(mcap::McapError::BadDataCrc { .. }))
    ));

    let mut chunk = fixture(no_summary().compression(None), "cdr", "ros2msg", true)?;
    let offset = record_offset(&chunk, op::CHUNK);
    chunk[offset + 9 + 24] ^= 1; // Uncompressed chunk CRC.
    assert!(matches!(
        McapReader::new(chunk.as_slice()),
        Err(McapError::Mcap(mcap::McapError::BadChunkCrc { .. }))
    ));
    Ok(())
}

#[test]
fn unknown_channel_and_schema_references_are_rejected() -> Result<()> {
    let options = no_summary()
        .use_chunks(false)
        .calculate_data_section_crc(false);
    let buffer = fixture(options, "cdr", "ros2msg", true)?;
    let mut unknown_channel = buffer.clone();
    let message = record_offset(&unknown_channel, op::MESSAGE);
    unknown_channel[message + 9..message + 11].copy_from_slice(&42u16.to_le_bytes());
    assert!(matches!(
        McapReader::new(unknown_channel.as_slice()),
        Err(McapError::Mcap(mcap::McapError::UnknownChannel(_, 42)))
    ));
    let mut unknown_schema = buffer;
    let channel = record_offset(&unknown_schema, op::CHANNEL);
    unknown_schema[channel + 11..channel + 13].copy_from_slice(&42u16.to_le_bytes());
    assert!(matches!(
        McapReader::new(unknown_schema.as_slice()),
        Err(McapError::Mcap(mcap::McapError::UnknownSchema(_, 42)))
    ));
    Ok(())
}

#[test]
fn compressed_record_larger_than_file_round_trips() -> Result<()> {
    let mut buffer = Vec::new();
    let message = std_msgs::String {
        data: "x".repeat(1024 * 1024),
    };
    let mut writer = McapWriter::new(Cursor::new(&mut buffer))?;
    let channel = writer.add_ros_channel::<std_msgs::String>("/large")?;
    channel.write(&mut writer, 1, &message)?;
    writer.finish()?;
    assert!(buffer.len() < message.data.len());
    let reader = McapReader::new(buffer.as_slice())?;
    let record = reader.iter_messages()?.next().unwrap()?;
    let decoded: std_msgs::String = McapReader::deserialize_message(&record.data)?;
    assert_eq!(decoded.data, message.data);
    Ok(())
}

#[test]
fn conflicting_repeated_schema_and_channel_records_are_rejected() -> Result<()> {
    let options = WriteOptions::new()
        .use_chunks(false)
        .calculate_data_section_crc(false);
    let buffer = fixture(options, "cdr", "ros2msg", true)?;
    let mut bad_schema = buffer.clone();
    let schema = record_offset(&bad_schema, op::SCHEMA);
    let length =
        u64::from_le_bytes(bad_schema[schema + 1..schema + 9].try_into().unwrap()) as usize;
    bad_schema[schema + 9 + length - "string data".len()] = b'S';
    assert!(matches!(
        McapReader::new(bad_schema.as_slice()),
        Err(McapError::Mcap(mcap::McapError::ConflictingSchemas(_)))
    ));

    let mut bad_channel = buffer;
    let channel = record_offset(&bad_channel, op::CHANNEL);
    // ID/schema ID, string length, then the topic bytes.
    bad_channel[channel + 9 + 4 + 4 + 2] = b'O';
    assert!(matches!(
        McapReader::new(bad_channel.as_slice()),
        Err(McapError::Mcap(mcap::McapError::ConflictingChannels(_)))
    ));
    Ok(())
}

#[test]
fn attachment_without_crc_returns_error_instead_of_panicking() -> Result<()> {
    let options = no_summary()
        .use_chunks(false)
        .calculate_data_section_crc(false);
    let mut buffer = fixture(options, "cdr", "ros2msg", false)?;
    let end = record_offset(&buffer, op::DATA_END);
    // Times, empty name/media type, zero-length data, but no trailing CRC.
    let body = vec![0; 32];
    let mut attachment = vec![op::ATTACHMENT];
    attachment.extend_from_slice(&(body.len() as u64).to_le_bytes());
    attachment.extend_from_slice(&body);
    buffer.splice(end..end, attachment);
    assert!(matches!(
        McapReader::new(buffer.as_slice()),
        Err(McapError::Mcap(mcap::McapError::UnexpectedEof))
    ));
    Ok(())
}

#[test]
fn finished_empty_summary_free_file_is_valid() -> Result<()> {
    let mut buffer = Vec::new();
    let mut writer = no_summary().create(Cursor::new(&mut buffer))?;
    writer.finish()?;
    drop(writer);
    let reader = McapReader::new(buffer.as_slice())?;
    assert!(reader.channels().is_empty());
    assert_eq!(reader.iter_messages()?.count(), 0);
    Ok(())
}
