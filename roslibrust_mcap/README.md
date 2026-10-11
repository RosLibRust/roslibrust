# roslibrust_mcap

Utilities for reading and writing [MCAP](https://mcap.dev/) files with ROS message support for the roslibrust ecosystem.

## Overview

MCAP is a modular container format for heterogeneous timestamped data, commonly used for recording and playing back ROS messages. This crate provides integration between the Rust MCAP library and roslibrust's message types. If you want to use MCAP files in rust, and want automatic serialization/deserialization of ROS messages this crate is for you.

## ROS 2 Compatibility

MCAP files written by this crate are compatible with ROS 2's `ros2 bag` tools. However, MCAP support varies across ROS 2 distributions:

| ROS 2 Distro | MCAP Support | Notes |
|--------------|--------------|-------|
| **Jazzy+** | ✅ Built-in (default) | MCAP is auto-detected, no extra flags needed |
| **Kilted** | ✅ Built-in (default) | MCAP is auto-detected, no extra flags needed |
| **Rolling** | ✅ Built-in (default) | MCAP is auto-detected, no extra flags needed |
| **Iron** | ⚠️ Requires plugin | Install `ros-iron-rosbag2-storage-mcap` |
| **Humble** | ⚠️ Requires plugin | Install `ros-humble-rosbag2-storage-mcap` |
| **Galactic** | ⚠️ Requires plugin | Install `ros-galactic-rosbag2-storage-mcap` |

### Using MCAP files with older ROS 2 distributions (Galactic/Humble/Iron)

1. **Install the MCAP storage plugin:**
   ```bash
   # For Humble:
   sudo apt install ros-humble-rosbag2-storage-mcap

   # For Iron:
   sudo apt install ros-iron-rosbag2-storage-mcap

   # For Galactic:
   sudo apt install ros-galactic-rosbag2-storage-mcap
   ```

2. **Specify the storage format when using `ros2 bag` commands:**
   ```bash
   # Reading bag info
   ros2 bag info -s mcap your_recording.mcap

   # Playing back
   ros2 bag play -s mcap your_recording.mcap

   # Recording (if you want to record in MCAP format)
   ros2 bag record -s mcap -a
   ```

### Using MCAP files with newer ROS 2 distributions (Jazzy/Kilted/Rolling)

MCAP is the default format and is auto-detected. No extra installation or flags are needed:
```bash
ros2 bag info your_recording.mcap
ros2 bag play your_recording.mcap
```

## Features

- **Read MCAP files**: Parse MCAP files and access ROS messages
- **Write MCAP files**: Record ROS messages to MCAP format
- **ROS Integration**: Works with roslibrust's `RosMessageType` trait

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
roslibrust_mcap = "0.26"
```

## Usage

`McapReader::new` accepts finished MCAP files with or without a summary, including
channels with no messages. It scans and decompresses the records during construction
to validate channel/schema references, supported encodings, and available chunk,
data-section, and summary CRCs. Invalid or truncated files return an error immediately.
This adds a full validation pass before message iteration; message payloads are not
retained after validation. The reader supports CDR messages and `ros2msg` schemas.

Channel handles belong to the `McapWriter` that created them. Passing a handle to
another writer returns `McapError::ForeignChannel`. Reusing a topic or schema name
requires identical declared ROS metadata (type names, definition, MD5, and ROS 2
hash); incompatible registrations return `IncompatibleChannel` or
`IncompatibleSchema` without changing the existing registration.

### Writing MCAP Files

```rust
use roslibrust_mcap::McapWriter;
use std::fs::File;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = File::create("output.mcap")?;
    let mut writer = McapWriter::new(file)?;

    // Use your generated ROS message type (here from roslibrust_test).
    let channel = writer.add_ros_channel::<roslibrust_test::ros2::std_msgs::String>("/my_topic")?;

    let timestamp = 1234567890; // nanoseconds
    let message = roslibrust_test::ros2::std_msgs::String { data: "Hello, MCAP!".into() };
    channel.write(&mut writer, timestamp, &message)?;

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
