# roslibrust_mcap

Utilities for reading and writing [MCAP](https://mcap.dev/) files with ROS message support for the roslibrust ecosystem.

## Overview

MCAP is a modular container format for heterogeneous timestamped data, commonly used for recording and playing back ROS messages. This crate provides integration between the Rust MCAP library and roslibrust's message types.

## Features

- **Read MCAP files**: Parse MCAP files and access ROS messages
- **Write MCAP files**: Record ROS messages to MCAP format
- **ROS Integration**: Works with roslibrust's `RosMessageType` trait

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
roslibrust_mcap = "0.1"
```

## Usage

### Writing MCAP Files

```rust
use roslibrust_mcap::McapWriter;
use std::fs::File;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = File::create("output.mcap")?;
    let mut writer = McapWriter::new(file)?;

    // Add a channel for your message type
    let channel_id = writer.add_ros_channel("/my_topic", "std_msgs/String")?;

    // Write messages
    let timestamp = 1234567890; // nanoseconds
    let message_data = b"Hello, MCAP!";
    writer.write_message(channel_id, timestamp, message_data)?;

    // Finish writing
    writer.finish()?;
    Ok(())
}
```

### Reading MCAP Files

```rust
use roslibrust_mcap::McapReader;
use std::fs::File;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = File::open("recording.mcap")?;
    let reader = McapReader::new(file)?;

    // Access channel information
    for (id, channel) in reader.channels() {
        println!("Channel {}: {} ({})", id, channel.topic, channel.message_type);
    }

    Ok(())
}
```

## MCAP Format

MCAP is an open-source container file format for multimodal log data. It supports:

- Self-contained files with embedded schemas
- Efficient seeking and indexing
- Multiple compression options
- Arbitrary metadata attachment

Learn more at [mcap.dev](https://mcap.dev/).

## License

MIT

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

