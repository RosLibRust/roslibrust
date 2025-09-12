# RosLibRust

[![Noetic](https://github.com/roslibrust/roslibrust/actions/workflows/noetic.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/noetic.yml)
[![Galactic](https://github.com/roslibrust/roslibrust/actions/workflows/galactic.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/galactic.yml)
[![Humble](https://github.com/roslibrust/roslibrust/actions/workflows/humble.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/humble.yml)
[![Iron](https://github.com/roslibrust/roslibrust/actions/workflows/iron.yml/badge.svg)](https://github.com/roslibrust/roslibrust/actions/workflows/iron.yml)
[![Kilted](https://github.com/roslibrust/roslibrust/actions/workflows/kilted.yml/badge.svg)][https://github.com/roslibrust/roslibrust/actions/workflows/kilted.yml]
[![License:MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An async rust library for interfacing with ROS1 and ROS2, built on Tokio.

- One Trait Based API - Write your behavior once and use it with any backend! Select the backend you want to use at compile time.
- Pure Rust - No ROS1 or ROS2 dependencies or installation required! Compile time message generation from .msg/.srv files.

This allows writing generic behaviors like:

```rust ,no_run
# use roslibrust_test::ros1::*;
use roslibrust::{TopicProvider, Publish, Subscribe};

async fn relay<T: TopicProvider>(ros: T) -> roslibrust::Result<()> {
    let mut subscriber = ros.subscribe::<std_msgs::String>("in").await?;
    let mut publisher = ros.advertise::<std_msgs::String>("out").await?;
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

```no_compile
// Will generate types from all packages in ROS_PACKAGE_PATH 
roslibrust_codegen_macro::find_and_generate_ros_messages!();
```

If you want to see what the generated code looks like check [here](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust_test/src/ros1.rs).
While the macro is useful for getting started, we recommend using `roslibrust_codegen` with a `build.rs` as shown in [example_package](https://github.com/RosLibRust/roslibrust/tree/master/example_package).
This allows cargo to know when message files are edited and automatically re-generate the code.

## Getting Started / Examples

Examples can be found in [examples](https://github.com/RosLibRust/roslibrust/tree/master/roslibrust/examples).
We recommend looking at the examples prefixed with `generic_` first, these examples show the recommended style of using `roslibrust` through the generic traits.
Code written this way can be used with any backend, and critically can be tested with the mock backend.

Examples prefixed with `ros1_`, `rosbridge_`, and `zenoh_` show direct use of specific backends if you are only interested in a single backend.
Some backends may provide additional functionality not available through the generic traits.

To get started with writing a node with `roslibrust` we recommend looking at [example_package](https://github.com/RosLibRust/roslibrust/tree/master/example_package) and setting up your
`Cargo.toml` and `build.rs` in a similar way.
Some important tips to keep in mind with using the crate:

- This crate is built around the [tokio runtime](https://docs.rs/tokio/latest/tokio/) and requires tokio to work. All backends expect to be created inside a tokio runtime.
- The generic traits `TopicProvider` and `ServiceProvider` are not [object safe](https://doc.rust-lang.org/reference/items/traits.html#object-safety) due to their generic parameters. This means you cannot use them as trait objects with `Box<dyn TopicProvider>` or `Box<dyn ServiceProvider>`. Instead, they should be used as compile time generics like `fn foo(ros: impl TopicProvider)` or `struct MyNode<T: TopicProvider> { ros: T }`.
- By default the roslibrust crate does not include any backends. You must enable the specific backends you want to use with features in `Cargo.toml` like `roslibrust = { version = "0.12", features = ["ros1"] }`.

## Contributing

Contribution through reporting of issues encountered and implementation in PRs is welcome! Before landing a large PR with lots of code implemented, please open an issue if there isn't a relevant one already available and chat with a maintainer to make sure the design fits well with all supported platforms and any in-progress implementation efforts.

We uphold the rust lang [Code of Conduct](https://www.rust-lang.org/policies/code-of-conduct).

### Minimum Supported Rust Version / MSRV

MSRV is currently set to 1.75 to enable `async fn` in traits.

We are likely to increase the MSRV to 1.83 when support for `async closures` lands.
