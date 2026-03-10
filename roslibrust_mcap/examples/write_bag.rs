//! Example: Writing messages to an MCAP bag file
//!
//! This example demonstrates how to create an MCAP bag file and write ROS messages to it.
//! The resulting file can be played back using `ros2 bag play` or read using the
//! `read_and_publish` example.
//!
//! # Usage
//!
//! ```bash
//! cargo run -p roslibrust_mcap --example write_bag
//! ```
//!
//! The output file `example_output.mcap` can be inspected with:
//! ```bash
//! # For Jazzy/Kilted/Rolling:
//! ros2 bag info example_output.mcap
//!
//! # For Humble/Iron/Galactic (requires ros-<distro>-rosbag2-storage-mcap):
//! ros2 bag info -s mcap example_output.mcap
//! ```

use roslibrust_mcap::{McapWriter, Result};
use std::fs::File;
use std::time::{SystemTime, UNIX_EPOCH};

// Use pre-generated ROS2 message types
use roslibrust_test::ros2::{builtin_interfaces, std_msgs};

fn main() -> Result<()> {
    // Create the output file
    let output_path = "example_output.mcap";
    let file = File::create(output_path)?;
    let mut writer = McapWriter::new(file)?;

    println!("Writing MCAP file: {}", output_path);

    // Create channels for different message types
    let string_channel = writer.add_ros_channel::<std_msgs::String>("/chatter")?;
    let header_channel = writer.add_ros_channel::<std_msgs::Header>("/header")?;
    let int_channel = writer.add_ros_channel::<std_msgs::Int32>("/counter")?;

    // Get the current time as the base timestamp
    let base_time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    // Write some String messages at 10Hz
    println!("Writing /chatter messages...");
    for i in 0..50 {
        let timestamp = base_time + (i as u64 * 100_000_000); // 100ms apart (10Hz)
        let msg = std_msgs::String {
            data: format!("Hello from Rust! Message #{}", i),
        };
        string_channel.write(&mut writer, timestamp, &msg)?;
    }

    // Write some Header messages at 10Hz
    println!("Writing /header messages...");
    for i in 0..50 {
        let timestamp = base_time + (i as u64 * 100_000_000);
        let secs = (timestamp / 1_000_000_000) as i32;
        let nsecs = (timestamp % 1_000_000_000) as u32;

        let msg = std_msgs::Header {
            stamp: builtin_interfaces::Time {
                sec: secs,
                nanosec: nsecs,
            },
            frame_id: "base_link".to_string(),
        };
        header_channel.write(&mut writer, timestamp, &msg)?;
    }

    // Write some Int32 messages (counter) at 10Hz
    println!("Writing /counter messages...");
    for i in 0..50 {
        let timestamp = base_time + (i as u64 * 100_000_000);
        let msg = std_msgs::Int32 { data: i };
        int_channel.write(&mut writer, timestamp, &msg)?;
    }

    // Finish writing and close the file
    writer.finish()?;

    println!("Successfully wrote {} messages to {}", 150, output_path);
    println!("\nTo verify with ROS2:");
    println!("  ros2 bag info {}", output_path);
    println!("\nTo play back:");
    println!("  ros2 bag play {}", output_path);

    Ok(())
}

