# Introduction

RosLibRust is an alternative to the various existing ROS clients.
RosLibRust may be a great fit for your project, or you may benefit from one of the other existing clients.

- [ros2-rust](https://github.com/ros2-rust/ros2_rust) is ideal if you want to release ROS2 packages to the community, or want to add Rust to an existing ROS2 project. But requires a full ROS2 installation.
- [rosrust](https://github.com/adnanademovic/rosrust) is a solid ROS1 option, but doesn't support `async` and is un-maintained.
- [ros2_client](https://docs.rs/ros2-client/latest/ros2_client/) is a pure rust client for ROS2 that supports `async`. However, it can only talk DDS and not Zenoh that ROS2 is migrating to.

RosLibRust is ideal for systems like facility control systems and cloud tools that need to interact with a variety of ROS systems, and don't want to "become ROS" themselves.
RosLibRust has one API that supports a broad range of ROS versions and protocols.

## Why was RosLibRust created?

RosLibRust was designed to help solve several challenges in the ROS ecosystem:

1. The need for `async` clients. At the time RosLibRust was written there was no `async` for ROS1 at all.
1. The need for pure rust clients.
    1. RosLibRust nodes can be built on a system with only a rust compiler. Official rust docker images are [~500mb](https://hub.docker.com/_/rust).
    1. Once compiled a RosLibRust node is a fully statically linked executable that can be run on any system. A RosLibRust ros1 publisher node is <10MB compiled and can be deployed in 5MB alpine docker image. Show me any other ROS node that I can deploy in <15 MB of size.
1. The ROS1 -> ROS2 migration was painful. While many concepts in ROS remained very similar, the API and build changes were invasive. A API abstraction layer should have made this easier.
1. ROS is pretty painful for both Unit and Integration testing.

## How does RosLibRust solve these problems?

At a high level what RosLibRust provides is:

- [An abstract API for ROS](#an-abstract-api-for-ros)
- [Implementations of the abstract API for ROS1, ROS2, and rosbridge](#backend-implementations)
- [A mock implementation of ROS for testing that allows deterministic "time traveling" tests](#mock-implementation-for-testing)
- [Pure rust generation of types from ROS's .msg/.srv files that is ROS version agnostic](#pure-rust-type-generation)

These combine to make RosLibRust a powerful tool for building ROS nodes in rust.

### An Abstract API for ROS

At the heart of RosLibRust is a set of traits that define the API for interacting with ROS.
These traits are defined in the [roslibrust_common](https://docs.rs/roslibrust_common/latest/roslibrust_common/) crate.

The most core trait is called [Ros](https://docs.rs/roslibrust_common/latest/roslibrust_common/traits/trait.Ros.html):

```rust,ignore
{{#include ../../roslibrust_common/src/traits.rs:ros_trait}}
```

This trait was designed to mimic how "Node Handles" work.
Breaking down this trait:

- `'static` - Something implementing this trait holds no references to non-static data. Making its lifetime very safe.
- `Send` + `Sync` - Something implementing this trait is safe to send and *use* between threads.
- `Clone` - Something implementing this trait can be cloned, this allows us to make more handles and distribute them amongst our node.
- `TopicProvider` - Is another trait in roslibrust that allows us to create publishers and subscribers.
- `ServiceProvider` - Is another trait in roslibrust that allows us to create service clients and servers.

Taking a look at the [TopicProvider](https://docs.rs/roslibrust_common/latest/roslibrust_common/traits/trait.TopicProvider.html) trait:

```rust,ignore
{{#include ../../roslibrust_common/src/traits.rs:topic_provider}}
```

We see a fairly advanced trait by Rust standards, with the following key features:

- We leverage Rust's [Generic Associated Types (GATs)](https://blog.logrocket.com/using-rust-gats-improve-code-app-performance/) to define the Publisher and Subscriber types returned by the trait functions.
  This means that the type (including the size) that a given implementation returns is completely different from other implementations.
  The only requirement is the type implements the [Publish](https://docs.rs/roslibrust_common/latest/roslibrust_common/traits/trait.Publish.html) and [Subscribe](https://docs.rs/roslibrust_common/latest/roslibrust_common/traits/trait.Subscribe.html) traits.
- While at first glance they might not look like it. The `advertise` and `subscribe` functions are actually `async fn` and return Futures.
  This means we don't have to block our application on waiting for a connection to be established.
- The advertise and subscribe functions are further generic over the message type. This is familiar to ROS users, but further compounds the complexity of the trait.

To finish the chain let's look at one more trait, [Publish](https://docs.rs/roslibrust_common/latest/roslibrust_common/traits/trait.Publish.html):

```rust,ignore
{{#include ../../roslibrust_common/src/traits.rs:publish}}
```

Publish finally provides the `publish` function that we use to send messages on a topic, which is again `async` and generic over the message type.
The combination of these traits is fairly complicated, but at the end of the day enables us to write code that is extremely agnostic to the underlying ROS implementation.
Furthermore, there is **NO PERFORMANCE PENALTY** for this abstraction.
Due to how monomorphic generics work in rust the compiler is able to completely erase the generic types at compile time, and optimize the code as if we had written it directly for the specific type.

What's the catch? Slightly worse compile times, but you only pay for what you use.
If you create a generic node that uses the `Ros` trait, and use it with multiple backend implementations,
there will be separate compilations of the node for each backend.

### Backend Implementations

Okay so RosLibRust has this abstract API? How can we actually use it?

RosLibRust provides several implementations of the `Ros` trait:

- [roslibrust_ros1](https://docs.rs/roslibrust_ros1/latest/roslibrust_ros1/) - Implements the TCPROS protocol that was the backbone of ROS1
- [roslibrust_zenoh](https://docs.rs/roslibrust_zenoh/latest/roslibrust_zenoh/) - Implements a variant of ROS1 communication used by [zenoh-ros1-bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros1)
- [roslibrust_rosbridge](https://docs.rs/roslibrust_rosbridge/latest/roslibrust_rosbridge/) - Implements the [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) websocket protocol
- COMING SOON [roslibrust_ros2](https://docs.rs/roslibrust_ros2/latest/roslibrust_ros2/) - Implements the Zenoh communication used by [rmw_zenoh](https://github.com/ros2/rmw_zenoh) in ROS2 from Kilted onwards
- [roslibrust_mock](https://docs.rs/roslibrust_mock/latest/roslibrust_mock/) - Implements a mock ROS perfect for testing

Typically we don't depend on these crates directly, but instead use them by enabling their corresponding features on `roslibrust`.

```toml
[dependencies]
roslibrust = { version = "0.15", features = ["ros1"] }
```

The full list of features is:

- `ros1` - Enables the [roslibrust_ros1](https://docs.rs/roslibrust_ros1/latest/roslibrust_ros1/) backend
- `zenoh` - Enables the [roslibrust_zenoh](https://docs.rs/roslibrust_zenoh/latest/roslibrust_zenoh/) backend
- `rosbridge` - Enables the [roslibrust_rosbridge](https://docs.rs/roslibrust_rosbridge/latest/roslibrust_rosbridge/) backend
- `mock` - Enables the [roslibrust_mock](https://docs.rs/roslibrust_mock/latest/roslibrust_mock/) backend
- `ros2` - COMING SOON Enables the [roslibrust_ros2](https://docs.rs/roslibrust_ros2/latest/roslibrust_ros2/) backend
- `codegen` - Provides access to the [roslibrust_codegen](https://docs.rs/roslibrust_codegen/latest/roslibrust_codegen/) crate for generating ROS message types in build.rs.
- `macro` - Provides access to the [roslibrust_codegen_macro](https://docs.rs/roslibrust_codegen_macro/latest/roslibrust_codegen_macro/) crate for generating ROS message types using a proc-macro.
- `all` - Enables all of the above features.

### Pure Rust Type Generation

RosLibRust has implemented a full parser and code-generator for ROS message types.
This allows us to generate Rust types from ROS .msg/.srv files at compile time.
The generated types are fully compatible with all RosLibRust backends.
Meaning a ROS1 .msg file can be used with ROS2 and vice versa.

The rosbridge backend support "generic" types with parsing fallbacks, see [generic message example](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust/examples/generic_message.rs)

A `build.rs` file can be used to automatically generate Rust types at build time from ROS .msg/.srv files.
See [example_package](https://github.com/RosLibRust/roslibrust/tree/master/example_package) for a full example.

A proc-macro is also provided for generating types at compile time, see [example_package_macro](https://github.com/RosLibRust/roslibrust/tree/master/example_package_macro) for an example.

This breaks tooling free from needing any ROS installation.

### Mock Implementation for Testing

A major challenge for incorporating ROS into larger systems is testing.
Traditionally testing ROS nodes involves either:

1. Breaking all the "ROS Logic" and other logic apart and indpendently testing them.
1. Launching a full ROS system in a testing environment and interacting with it.

The first approach leads to "extra abstraction" and often causes timing and messaging related bugs to be missed.
The second approach is brittle, slow, and often a bottle neck for test times.

RosLibRust provides a mock implementation of ROS for testing that allows deterministic "time traveling" tests.
See [extended getting started guide](extended_getting_started.md#writing-tests-for-our-node) for an example.

