//! Example showing how to write code that is generic over both the ROS backend
//! and the message type (ROS1 vs ROS2).
//!
//! This pattern allows you to write reusable robotics code that works with any
//! roslibrust backend (rosbridge, ros1 native, zenoh, mock) and any ROS version.

use std::time::Duration;

use roslibrust_common::{Publish, TopicProvider};
use roslibrust_transforms::{TFMessageType, Timestamp, TransformManager, TransformManagerError};

/// A generic robot component that uses transforms.
///
/// This struct is generic over:
/// - `M`: The TFMessage type (Ros1TFMessage or Ros2TFMessage)
/// - `P`: The publisher type (inferred from the TopicProvider)
pub struct RobotTransformHandler<M: TFMessageType, P: Publish<M> + Send + Sync> {
    manager: TransformManager<M, P>,
    base_frame: String,
    sensor_frame: String,
}

impl<M: TFMessageType, P: Publish<M> + Send + Sync> RobotTransformHandler<M, P> {
    /// Create a new RobotTransformHandler.
    pub async fn new<T>(
        ros: &T,
        base_frame: String,
        sensor_frame: String,
    ) -> Result<RobotTransformHandler<M, T::Publisher<M>>, TransformManagerError>
    where
        T: TopicProvider<Publisher<M> = P> + Clone + Send + Sync + 'static,
        T::Subscriber<M>: Send + 'static,
        T::Publisher<M>: Send + Sync,
    {
        let manager = TransformManager::<M, _>::new(ros).await?;
        Ok(RobotTransformHandler {
            manager,
            base_frame,
            sensor_frame,
        })
    }

    /// Get the current transform from sensor to base frame.
    pub async fn get_sensor_to_base_transform(
        &self,
    ) -> Result<transforms::Transform, TransformManagerError> {
        self.manager
            .lookup_latest_transform(&self.base_frame, &self.sensor_frame)
            .await
    }

    /// Check if the transform is available.
    pub async fn is_transform_available(&self) -> bool {
        self.manager
            .can_transform(&self.base_frame, &self.sensor_frame)
            .await
    }

    /// Get a static transform (timestamp = 0).
    pub async fn get_static_transform(
        &self,
    ) -> Result<transforms::Transform, TransformManagerError> {
        self.manager
            .lookup_transform(&self.base_frame, &self.sensor_frame, Timestamp::zero())
            .await
    }
}

// Example usage with rosbridge and ROS1 messages
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use roslibrust_rosbridge::ClientHandle;
    use roslibrust_transforms::Ros1TFMessage;

    env_logger::init();
    log::info!("Connecting to rosbridge...");

    let client = ClientHandle::new("ws://localhost:9090").await?;
    log::info!("Connected!");

    // Create our generic handler with ROS1 messages
    let handler = RobotTransformHandler::<Ros1TFMessage, _>::new(
        &client,
        "base_link".to_string(),
        "camera_link".to_string(),
    )
    .await?;

    log::info!("RobotTransformHandler created!");

    // Periodically check for transforms
    let mut interval = tokio::time::interval(Duration::from_secs(1));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                log::info!("Shutting down...");
                break;
            }
            _ = interval.tick() => {
                if handler.is_transform_available().await {
                    match handler.get_sensor_to_base_transform().await {
                        Ok(tf) => {
                            log::info!(
                                "Sensor position in base frame: ({:.3}, {:.3}, {:.3})",
                                tf.translation.x,
                                tf.translation.y,
                                tf.translation.z
                            );
                        }
                        Err(e) => log::warn!("Failed to get transform: {}", e),
                    }
                } else {
                    log::info!("Waiting for transform between base_link and camera_link...");
                }
            }
        }
    }

    Ok(())
}
