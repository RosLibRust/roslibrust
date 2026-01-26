# RosLibRust

[![Noetic](https://github.com/roslibrust/roslibrust/actions/workflows/noetic.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/noetic.yml)
[![Galactic](https://github.com/roslibrust/roslibrust/actions/workflows/galactic.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/galactic.yml)
[![Humble](https://github.com/roslibrust/roslibrust/actions/workflows/humble.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/humble.yml)
[![Iron](https://github.com/roslibrust/roslibrust/actions/workflows/iron.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/iron.yml)
[![Kilted](https://github.com/roslibrust/roslibrust/actions/workflows/kilted.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/kilted.yml)
[![Rolling](https://github.com/roslibrust/roslibrust/actions/workflows/rolling.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/rolling.yml)
[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Documentation about the crate is on [docs.rs](https://docs.rs/roslibrust/latest/roslibrust/),
extended guides can be found on [roslibrust.github.io](https://roslibrust.github.io/roslibrust/)

An async rust library for interfacing with ROS1 and ROS2, built on Tokio.

- One Trait Based API - Write your behavior once and use it with any backend! Select the backend you want to use at compile time.
- Pure Rust - No ROS1 or ROS2 dependencies or installation required! Compile time message generation from .msg/.srv files.

This allows writing generic behaviors like:

```rust ,no_run
# use roslibrust_test::ros1::*;
use roslibrust::{TopicProvider, Publish, Subscribe};

async fn relay<T: TopicProvider>(ros: T) -> roslibrust::Result<()> {
    let mut subscriber = ros.subscribe::<std_msgs::String>("/in").await?;
    let mut publisher = ros.advertise::<std_msgs::String>("/out").await?;
    while let Ok(msg) = subscriber.next().await {
        println!("Got message: {}", msg.data);
        publisher.publish(&msg).await?;
    }
    Ok(())
}

#[tokio::main]
async fn main() -> roslibrust::Result<()> {
    // Relay messages over a native ROS2 connection using Zenoh
    // #[cfg(feature = "ros2")]
    // {
    // let ros = roslibrust::ros2::NodeHandle::new("http://localhost:11311", "relay").await?;
    // relay(ros).await?;
    // }

    // Relay messages over a native ROS1 connection via TCPROS
    #[cfg(feature = "ros1")]
    {
    let ros = roslibrust::ros1::NodeHandle::new("http://localhost:11311", "relay").await?;
    relay(ros).await?;
    }

    // Relay messages over a zenoh connection compatible with zenoh-ros1-plugin / zenoh-ros1-bridge
    #[cfg(feature = "zenoh")]
    {
    let ros = roslibrust::zenoh::ZenohClient::new(zenoh::open(zenoh::Config::default()).await.unwrap());
    relay(ros).await?;
    }

    // Relay messages over a rosbridge_server connection with either ROS1 or ROS2!
    #[cfg(feature = "rosbridge")]
    {
    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    relay(ros).await?;
    }

    // Relay messages over a mock ROS connection for testing
    #[cfg(feature = "mock")]
    {
    let ros = roslibrust::mock::MockRos::new();
    relay(ros).await?;
    }

    Ok(())
}
```

All of this is backed by common traits for ROS messages, topics, and services. `roslibrust_codegen` provides generation of Rust types from both ROS1 and ROS2 .msg/.srv files and
`roslibrust_codegen_macro` provides a convenient macro for generating these types:

```rust,ignore
// Will generate types from all packages in ROS_PACKAGE_PATH 
roslibrust_codegen_macro::find_and_generate_ros_messages!();
```

If you want to see what the generated code looks like checkout [our generated messages in our test crate](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust_test/src/ros1.rs).
While the macro is useful for getting started, we recommend using `roslibrust_codegen` with a `build.rs` as shown in [example_package](https://github.com/RosLibRust/roslibrust/tree/master/example_package).
This allows cargo to know when message files are edited and automatically re-generate the code.

## Getting Started / Examples

- Checkout the [Quick Getting Started Guide](https://roslibrust.github.io/roslibrust/quick_getting_started.html) for a brief guide on how to get started with RosLibRust.
- Checkout the [Extended Getting Started Guide](https://roslibrust.github.io/roslibrust/extended_getting_started.html) for a more in depth guide on how to get started with RosLibRust.
- Examples can be found in [examples](https://github.com/RosLibRust/roslibrust/tree/master/roslibrust/examples).

## Contributing

Contribution through reporting of issues encountered and implementation in PRs is welcome! Before landing a large PR with lots of code implemented, please open an issue if there isn't a relevant one already available and chat with a maintainer to make sure the design fits well with all supported platforms and any in-progress implementation efforts.

We uphold the rust lang [Code of Conduct](https://www.rust-lang.org/policies/code-of-conduct).

### Minimum Supported Rust Version / MSRV

MSRV is currently 1.88 due to dependencies (ros-z requires `let_chains` stabilized in 1.88).
