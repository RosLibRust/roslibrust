# roslibrust_mcap

Utilities for reading and writing [MCAP](https://mcap.dev/) files with ROS message support for the roslibrust ecosystem.

## Overview

MCAP is a modular container format for heterogeneous timestamped data, commonly used for recording and playing back ROS messages. This crate provides integration between the Rust MCAP library and roslibrust's message types.

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