//! Example showing how to use TransformManager with ROS1 messages via rosbridge.
//!
//! This example connects to a rosbridge server and listens for transforms on /tf and /tf_static.
//! It then periodically looks up the transform between two frames.
//!
//! # Running this example
//!
//! 1. Start a rosbridge server:
//!    ```bash
//!    roslaunch rosbridge_server rosbridge_websocket.launch
//!    ```
//!
//! 2. Run this example:
//!    ```bash
//!    cargo run -p roslibrust_transforms --example transform_listener_ros1
//!    ```
//!
//! 3. Publish some transforms (in another terminal):
//!    ```bash
//!    rosrun tf2_ros static_transform_publisher 1 2 3 0 0 0 world base_link
//!    ```

use std::time::Duration;

use roslibrust_transforms::{Ros1TFMessage, Timestamp, TransformManager};

use log::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    info!("Connecting to rosbridge at ws://localhost:9090...");

    let client =
        roslibrust::ros1::NodeHandle::new("http://localhost:11311", "example_transform_listener")
            .await
            .expect("Failed to create a ROS1 node");
    info!("Connected!");

    // Create a TransformManager with ROS1 message types
    info!("Creating TransformManager...");
    let manager =
        TransformManager::<Ros1TFMessage, _>::new(&client, std::time::Duration::from_secs(10))
            .await?;
    info!("TransformManager created, listening on /tf and /tf_static");

    // Periodically try to look up transforms
    let mut interval = tokio::time::interval(Duration::from_secs(1));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("Received Ctrl+C, shutting down...");
                break;
            }
            _ = interval.tick() => {
                // Try to look up a transform from "world" to "base_link"
                match manager.get_transform("world", "base_link", Timestamp::now()).await {
                    Ok(transform) => {
                        info!(
                            "Transform world -> base_link: translation=({:.3}, {:.3}, {:.3})",
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z
                        );
                    }
                    Err(e) => {
                        warn!("Could not look up transform: {}", e);
                    }
                }

                // Also try looking up with a specific timestamp (for static transforms, use zero)
                match manager.get_transform("world", "base_link", Timestamp::zero()).await {
                    Ok(transform) => {
                        info!(
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
