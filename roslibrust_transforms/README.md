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
use roslibrust_transforms::{TransformManager, Ros1TFMessage, Timestamp};

async fn example(ros: impl roslibrust_common::TopicProvider + Clone + Send + Sync + 'static) {
    // Create a TransformManager (subscribes to /tf and /tf_static automatically)
    let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await.unwrap();

    // Look up a transform
    let transform = manager.get_transform("base_link", "camera_link", Timestamp::now()).await.unwrap();

    println!("Translation: {:?}", transform.translation());
    println!("Rotation: {:?}", transform.rotation());
}
```

### ROS1 vs ROS2

The only difference is the message type parameter:

```rust
// ROS1
let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await?;

// ROS2
let manager = TransformManager::<Ros2TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await?;
```

### Publishing Transforms

```rust
use roslibrust_transforms::{TransformManager, Ros1TFMessage, Stamp, Timestamp, Transform, Quaternion, Vector3};

async fn broadcast_example(ros: impl roslibrust_common::TopicProvider + Clone + Send + Sync + 'static) {
    let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await.unwrap();

    // Create and publish a dynamic transform, published on /tf
    let transform = Transform::new(
        "world",
        "robot",
        Vector3::new(1.0, 0.0, 0.0),
        Quaternion::identity(),
        Stamp::At(Timestamp::now()),
    ).unwrap();
    manager.add_transform(transform).await.unwrap();

    // Static transforms are valid for all time, and are published on /tf_static
    let static_tf = Transform::static_between(
        "robot",
        "sensor",
        Vector3::new(0.1, 0.0, 0.5),
        Quaternion::identity(),
    ).unwrap();
    manager.add_transform(static_tf).await.unwrap();
}
```
