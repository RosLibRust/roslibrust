//! The examples shows how TransformManager can be shared between multiple tasks.
//! Run with logging enabled to see the transform being published and looked up.
//! `RUST_LOG=debug cargo run --example shared_manager`

use std::sync::Arc;
use std::time::Duration;

use log::*;

// Example uses the mock backend for simplicity in running / testing, but any backend can be used
use roslibrust::mock::MockRos;
use roslibrust_transforms::{
    Quaternion, Ros1TFMessage, Timestamp, Transform, TransformManager, Vector3,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    info!("Creating mock ros instance...");
    let ros = MockRos::new();
    info!("Creating TransformManager...");
    let tf_mgr = TransformManager::<Ros1TFMessage, _>::new(&ros, Duration::from_secs(10)).await?;
    // Stick the manager in an Arc to make it Clone
    let tf_mgr = Arc::new(tf_mgr);

    // Create a task to periodically publish a transform
    let tf_mgr2 = tf_mgr.clone();
    // TransformManager in an Arc is fully Send + Sync + 'static and can be safely shared between tasks
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        let mut x = 1.0;
        loop {
            interval.tick().await;
            info!("Publishing transform...");
            // Because of internal mutability, add_transform() doesn't require `&mut self`
            // So you don't need to use Mutex<T> or RefCell<T> to share the manager
            tf_mgr2
                .add_transform(Transform {
                    parent: "world".to_string(),
                    child: "robot".to_string(),
                    translation: Vector3::new(x, 0.0, 0.0),
                    rotation: Quaternion::identity(),
                    timestamp: Timestamp::now(),
                })
                .await
                .unwrap();
            x += 1.0;
        }
    });

    // Run the example for 10 seconds
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let mut lookup = (Timestamp::now() + Duration::from_secs(2)).unwrap();
    // In the main task here, we can look up and log the transform
    loop {
        tokio::select! {
            _ = tokio::time::sleep(deadline.saturating_duration_since(tokio::time::Instant::now())) => {
                info!("Deadline reached, shutting down...");
                break;
            }
            _ = tokio::signal::ctrl_c() => {
                info!("Received Ctrl+C, shutting down...");
                break;
            }
            xform = tf_mgr.wait_for_transform("world", "robot", lookup, None) => {
                match xform {
                    Ok(xform) => {
                        info!("Transform: translation=({:.3}, {:.3}, {:.3})",
                            xform.translation.x,
                            xform.translation.y,
                            xform.translation.z
                        );
                    }
                    Err(e) => {
                        warn!("Error looking up transform: {}", e);
                    }
                }
                // Set the next time we want to lookup 1 second in future
                lookup = (lookup + Duration::from_secs(1)).unwrap();
            }
        }
    }
    Ok(())
}
