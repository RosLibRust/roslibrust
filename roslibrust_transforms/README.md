# roslibrust_transforms

A tf2-like transform library for roslibrust, providing `TransformListener` and `TransformBroadcaster` functionality.
Provides a convenient wrapper around the [transforms](https://docs.rs/transforms/latest/transforms/) crate to provide a roslibrust specific API for working with transforms in a ROS environment.

## Features

- **Backend agnostic** — Works with all roslibrust backends (ros1, rosbridge, zenoh, mock)
- **ROS1 & ROS2 support** — Ships with message schemas for both ROS1 and ROS2, and can be mixed and matched with roslibrust backends.
- **Buffered Time Travel** — Leverages the [transforms](https://docs.rs/transforms/latest/transforms/) to store a history of transforms and perform time-based lookups between moving frames in time.

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
roslibrust_transforms = "0.1"
```

### Basic Example

```rust
use roslibrust_transforms::{TransformManager, Ros1TFMessage};

async fn example(ros: impl roslibrust_common::TopicProvider + Clone + Send + Sync + 'static) {
    // Create a TransformManager (subscribes to /tf and /tf_static automatically)
    let manager = TransformManager::<Ros1TFMessage, _>::new(&ros).await.unwrap();

    // Look up a transform
    let transform = manager.lookup_latest_transform("base_link", "camera_link").await.unwrap();

    println!("Translation: {:?}", transform.translation);
    println!("Rotation: {:?}", transform.rotation);
}
```

### ROS1 vs ROS2

The only difference is the message type parameter:

```rust
// ROS1
let manager = TransformManager::<Ros1TFMessage, _>::new(&ros).await?;

// ROS2
let manager = TransformManager::<Ros2TFMessage, _>::new(&ros).await?;
```

### Publishing Transforms

```rust
use roslibrust_transforms::{TransformManager, Ros1TFMessage, Transform, Timestamp};

async fn broadcast_example(ros: impl roslibrust_common::TopicProvider + Clone + Send + Sync + 'static) {
    let manager = TransformManager::<Ros1TFMessage, _>::new(&ros).await.unwrap();

    // Create and publish a dynamic transform
    let transform = Transform {
        parent_frame: "world".to_string(),
        child_frame: "robot".to_string(),
        translation: Default::default(),
        rotation: Default::default(),
        timestamp: Timestamp::now(),
    };
    manager.update_transform(transform).await.unwrap();

    // Or publish a static transform
    let static_tf = Transform {
        parent_frame: "robot".to_string(),
        child_frame: "sensor".to_string(),
        translation: Default::default(),
        rotation: Default::default(),
        timestamp: Timestamp::now(), // Will be set to zero internally
    };
    manager.update_static_transform(static_tf).await.unwrap();
}
```
