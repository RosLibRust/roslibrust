//! Example showing how to use TransformManager with ROS2 messages via rosbridge.
//!
//! This example connects to a rosbridge server and listens for transforms on /tf and /tf_static.
//! It then periodically looks up the transform between two frames.
//!
//! # Running this example
//!
//! 1. Start a ROS2 rosbridge server:
//!    ```bash
//!    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
//!    ```
//!
//! 2. Run this example:
//!    ```bash
//!    cargo run -p roslibrust_transforms --example transform_listener_ros2
//!    ```
//!
//! 3. Publish some transforms (in another terminal):
//!    ```bash
//!    ros2 run tf2_ros static_transform_publisher --x 1 --y 2 --z 3 --frame-id world --child-frame-id base_link
//!    ```

use std::time::Duration;

use roslibrust_rosbridge::ClientHandle;
use roslibrust_transforms::{Ros2TFMessage, Timestamp, TransformManager};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    log::info!("Connecting to rosbridge at ws://localhost:9090...");

    // Create a rosbridge client to talk over the rosbridge websocket
    let client = ClientHandle::new("ws://localhost:9090").await?;
    log::info!("Connected!");

    // Create a TransformManager with ROS2 message types
    // Note: The only difference from ROS1 is using Ros2TFMessage instead of Ros1TFMessage
    log::info!("Creating TransformManager with ROS2 messages...");
    let manager = TransformManager::<Ros2TFMessage, _>::new(&client).await?;
    log::info!("TransformManager created, listening on /tf and /tf_static");

    // Periodically try to look up transforms
    let mut interval = tokio::time::interval(Duration::from_secs(1));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                log::info!("Received Ctrl+C, shutting down...");
                break;
            }
            _ = interval.tick() => {
                // Try to look up a transform from "world" to "base_link"
                match manager.lookup_latest_transform("world", "base_link").await {
                    Ok(transform) => {
                        log::info!(
                            "Transform world -> base_link: translation=({:.3}, {:.3}, {:.3})",
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z
                        );
                    }
                    Err(e) => {
                        log::warn!("Could not look up transform: {}", e);
                    }
                }

                // Also try looking up with a specific timestamp (for static transforms, use zero)
                match manager.lookup_transform("world", "base_link", Timestamp::zero()).await {
                    Ok(transform) => {
                        log::info!(
                            "Static transform world -> base_link: translation=({:.3}, {:.3}, {:.3})",
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z
                        );
                    }
                    Err(_) => {
                        // Static transform not available yet
                    }
                }
            }
        }
    }

    Ok(())
}
