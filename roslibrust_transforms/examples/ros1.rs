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

use roslibrust_transforms::{Ros1TFMessage, Stamp, Timestamp, TransformManager};

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
                        let translation = transform.translation();
                        info!(
                            "Transform world -> base_link: translation=({:.3}, {:.3}, {:.3})",
                            translation.x,
                            translation.y,
                            translation.z
                        );
                    }
                    Err(e) => {
                        warn!("Could not look up transform: {}", e);
                    }
                }

                // Also look up the newest transform the local buffer can serve
                if let Ok(latest) = manager.latest_common_time("world", "base_link").await {
                    let time = match latest {
                        Stamp::At(time) => time,
                        // Frames connected by static transforms only can be looked up at any time
                        Stamp::Static => Timestamp::now(),
                    };
                    match manager.get_transform("world", "base_link", time).await {
                        Ok(transform) => {
                            let translation = transform.translation();
                            info!(
                                "Latest transform world -> base_link: translation=({:.3}, {:.3}, {:.3})",
                                translation.x,
                                translation.y,
                                translation.z
                            );
                        }
                        Err(e) => {
                            warn!("Could not look up latest transform: {}", e);
                        }
                    }
                }
            }
        }
    }

    Ok(())
}
