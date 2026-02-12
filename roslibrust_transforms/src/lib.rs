//! A tf2-like transform library for roslibrust.
//!
//! This crate provides a `TransformManager` that subscribes to `/tf` and `/tf_static` topics,
//! maintains a transform buffer using the `transforms` crate, and can publish transforms.
//!
//! # Features
//!
//! - Generic over roslibrust backends (ros1, rosbridge, zenoh, mock)
//! - Supports both ROS1 and ROS2 message formats
//! - Automatic subscription to `/tf` and `/tf_static` topics
//! - Ability to publish transforms via `update_transform()` and `update_static_transform()`
//!
//! # ROS1 vs ROS2
//!
//! The `TransformManager` is generic over the message type. Use the appropriate type alias
//! for your ROS version:
//!
//! - ROS1: `TransformManager::<Ros1TFMessage, _>::new(&ros)`
//! - ROS2: `TransformManager::<Ros2TFMessage, _>::new(&ros)`
//!
//! # Example
//! ```no_run
//! use roslibrust_transforms::{TransformManager, Ros1TFMessage};
//! use roslibrust_common::TopicProvider;
//!
//! async fn example<T: TopicProvider + Clone + Send + Sync + 'static>(ros: T)
//! {
//!     let manager = TransformManager::<Ros1TFMessage, _>::new(&ros).await.unwrap();
//!
//!     // Look up a transform
//!     let mut transform = manager.lookup_latest_transform("base_link", "camera_link").await.unwrap();
//!
//!     // Modify the transform
//!     transform.translation.x += 1.0;
//!     transform.timestamp = transforms::time::Timestamp::now();
//!
//!     // Update the value in the buffer, and publish it's update to other nodes
//!     manager.update_transform(transform).await.unwrap();
//! }
//! ```

pub mod messages;
pub mod transform_manager;
pub use transform_manager::*;

// Re-export useful types from the transforms crate
pub use transforms::geometry::{Quaternion, Transform, Vector3};
pub use transforms::time::Timestamp;
pub use transforms::Registry;
