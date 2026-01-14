# Getting Started

Inside the roslibrust repository you'll find [example_package](https://github.com/RosLibRust/roslibrust/tree/master/example_package)
and [example_package_macro](https://github.com/RosLibRust/roslibrust/tree/master/example_package_macro) which serve as good examples of how to integrate roslibrust into a package.

This documentation walks you through how those packages are setup, and how to get started using RosLibRust in your own project.

We recommend using the build.rs approach shown in example_package, as this approach will automatically re-generate your ROS types when the underlying .msg/.srv files are changed.
The macro approach shown in example_package_macro is easier to setup, but can't detect when the underlying .msg/.srv files are changed.
This approach is fine to use if you know your .msg/.srv files are not changing, or if you are ok with manually re-running the macro to generate the types.

This tutorial is written for someone who is new to both Rust and ROS, and assumes no prior knowledge of either.

## Machine Setup

RosLibRust is currently only actively tested on Linux, however it should work on Windows and MacOS as well.
If you run into any issues on Windows or MacOS please open an issue on the [roslibrust github](https://github.com/RosLibRust/roslibrust/issues).

The only requirement to building roslibrust is the rust compiler and toolchain.

These are best installed via the instructions on the [rust website](https://www.rust-lang.org/tools/install).

If you are using Linux you can install the rust toolchain by running:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

RosLibRust currently requires a version of the Rust compiler greater than 1.85. This can be checked by running `rustc --version`.

If you can run this command and see a version greater than 1.85 you're good to move on to the next step.

For actually running the example applications show here you'll need a working ROS installation with the `rosbridge_server` package.
These instructions show using docker images to run isolated ROS environments.
To exactly follow these instructions you'll need `docker engine` installed and running on your system.
However, docker can by bypassed if you prefer to directly install a specific version of ROS on your system.

To install docker engine see the [docker website](https://docs.docker.com/engine/install/).

## Making an Empty Project

For a longer explanation of this section see the [Hello Cargo](https://doc.rust-lang.org/book/ch01-03-hello-cargo.html) section of the Rust Book.

In a folder of your choice run `cargo new my_package`. This will create a new rust package in the `my_package` folder.

Cargo (rust's build tool) will automatically generate several files for you in this directory:

```bash
my_package
├── .git # A git repository is automatically created for you. You should use git!
├── .gitignore # A gitignore file is automatically created for you with good defaults for a Rust project.
├── Cargo.toml # Controls dependencies and other metadata for your package, equivalent to CMakeLists.txt in C++ or package.xml in ROS
└── src # All the "main" code for your package lives in this folder
    └── main.rs # A starting file with some initial code in it
```

Now that we've created this project we can:

```bash
# Change into the directory of our new package.
# Once we've done this cargo commands will automatically detect the Cargo.toml file and know
# that they should operate on this package
cd my_package
# This command with both fully compile our package and all its dependencies, and then run the
# resulting executable
cargo run
```

If you see `Hello, world!` printed to the console you're good to move on to the next step.

## Setting Up Cargo.toml

To use roslibrust we need to modify `Cargo.toml` to add roslibrust as a dependency.

If you're new to Rust you should checkout this chapter in the Cargo Book: [Dependencies](https://doc.rust-lang.org/cargo/guide/dependencies.html)

An example `Cargo.toml` is:

```toml
[package]
name = "my_package"
version = "0.1.0"
edition = "2021"

# What crates your code needs when built regularly (e.g. `cargo build` or `cargo run`)
[dependencies]
# You will need to specify at least one backend to use with roslibrust, available options are [ros1, rosbridge, zenoh, mock] (ros2 is coming soon)
# You will also need to specify the "codegen" feature, as the generated ROS types rely on features from this module
roslibrust = { version = "0.15", features = ["rosbridge", "codegen"] }
# RosLibRust is built on tokio, and requires a multi-threaded tokio runtime.
# You don't need the "full" tokio feature set, but it is a good starting place
tokio = { version = "1", features = ["full"] }

# What crates your code needs for testing and examples
[dev-dependencies]
# For testing you'll want to use the "mock" backend if you specify it here, it won't affect your production builds
roslibrust = { version = "0.15", features = ["mock"] }
tokio = { version = "1", features = ["test-util"] }

# What crates your code needs to run it's build.rs file
[build-dependencies]
# In build.rs we'll use roslibrust's codegen features to generate Rust types from ROS .msg/.srv files
roslibrust = { version = "0.15", features = ["codegen"] }
```

Once we've modified `Cargo.toml` in this way we can run `cargo build`,
and we should see Cargo automatically download and compile all the dependencies we've specified.

If this works you're ready to move on to the next step.

## Setting Up Automatic Rust<->ROS Type Generation

In ROS based systems "nodes" communicate with each other by sending and receiving messages.
Because ROS supports multiple languages (Python, C++, Rust, etc.) a common schema language was needed.
ROS uses calls these custom message definitions "Interfaces" and documents them [in the ROS documentation's Basic Concepts section](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html).

To work ergonomically in Rust with these messages we want a corresponding Type in Rust for each message type.
RosLibRust will automatically generate these types for us, and uses the generate types to serialize and deserialize messages as they are sent and received.

To setup automatic generation of these Rust types from ROS interface files, we'll leverage Rust's build.rs feature.
A `build.rs` file is a special file that Cargo will automatically run before compiling your crate.
This file can be used to generate code, compile native dependencies, or perform any other task needed to build your crate.
Learn more about writing `build.rs` files [in the Cargo Book's build scripts section](https://doc.rust-lang.org/cargo/reference/build-scripts.html).

Let's create a copy of build.rs file from [example_package](https://github.com/RosLibRust/roslibrust/tree/master/example_package) in our package:

```rust,ignore
{{#include ../../example_package/build.rs}}
```

Before this build.rs script can run successfully we'll need to give it some real ROS messages to find.
For this example we'll use some standard ROS2 messages from the [ROS2 Common Interfaces](https://github.com/ros2/common_interfaces) repository.

To clone these messages into our package we can run:

```bash
# Make sure we're in the root of our package
cd my_package
# Make a folder to hold our messages
mkdir assets
# Clone the common interfaces into that folder
git submodule add https://github.com/ros2/common_interfaces assets/common_interfaces
```

<div class="warning">

Warning: ROS messages can refer to other messages in their contents.
For example many messages include a `Header` message from the `std_msgs` package.
For code generation to work correctly you must include all the messages you want to generate, AND all the messages that those messages depend on.

</div>

In this specific case, the messages in `common_interfaces` rely on messages from the `builtin_interfaces` which is not included in
that same repository. To fix this we'll also need to clone the `rcl_interfaces` repository which contains the `builtin_interfaces` package:

```bash
# Make sure we're in the root of our package
cd my_package
# Clone the rcl_interfaces repository into our assets folder
git submodule add https://github.com/ros2/rcl_interfaces assets/rcl_interfaces
```

Now we just need to modify the `search_paths` variable in our `build.rs` file to point at our new messages:

```rust,ignore
let search_paths = vec![
    "assets/common_interfaces".into(),
    "assets/rcl_interfaces/builtin_interfaces".into(),
];
```

Now if we run `cargo build` again we should see Cargo automatically run our `build.rs` file, and generate our Rust types from the ROS messages.
This won't be immediately obvious from the command line, but we can go look in the `target` folder of our package to see the generated code.
The generated code will be in `target/debug/build/my_package-<some hash of our package>/out/messages.rs`.

We can check if the file exists by running:

```bash
find -name "messages.rs"
```

If this prints out a path to a `messages.rs` file we're good to move on to the next step.

## Using Generated Types

Now that we've generated our types we can use them in our code.
Rust luckily has some convenient macros for bringing generated code "into scope" for a crate.

If we open up `src/main.rs` and add the following line at the top of the file:

```rust,ignore
include!(concat!(env!("OUT_DIR"), "/messages.rs"));
```

It will automatically find the generated `messages.rs` and effectively "copy paste" the contents of that file into our `main.rs` file.

Breaking down how that line works:

- `env!("OUT_DIR")` is a macro that expands to the value of the `OUT_DIR` environment variable. This is set by Cargo and points to the folder where our generated code is located.
- `concat!(...)` is a macro that concatenates multiple string literals into a single string literal. In this case we're concatenating the `OUT_DIR` environment variable with the path to our generated code.
- `include!(...)` is a macro that includes the contents of the file at the specified path into the current file. In this case we're including the generated `messages.rs` file into our `main.rs` file.

Once we've added that line to `main.rs` all our generated messages will be available to us in the rest of our code.

For this example we'll keep it simple and leave that line in `main.rs`, but in larger projects it is recommended to move the generated types to either a msgs module or a separate msgs crate in a workspace.

## Writing Our First Node

You're now ready to actually write some code that uses RosLibRust!

We're going to start with a basic example of publishing to a topic.

Modify `src/main.rs` to look like the following:

```rust,ignore
{{#include ../../example_package/examples/getting_started_1.rs:getting_started_1}}
```

There is a lot to break down in this example, it uses many of the features of Rust, Tokio, and RosLibRust.

Let's start with the high level structure:

- Our "node" is defined in an `async fn` this allows to spawn an instance of our node as a new tokio task.
- Our node uses `impl Ros` for the type of the `ros` parameter. This makes the function generic over any roslibrust backend.
- Our main function sets up the dependencies of our node, and then spawns it as a new tokio task to run independently.
- We use `Arc<Mutex<>>` to share mutable state between our tokio tasks.

Right now, **why** we did all these things might not be obvious, but it will be once we start wanting to write more complex nodes and when we want to test those nodes.

## Running Our Node

So far we've avoided installing any version of ROS at all. This is great since we can write and run our node on a system without any ROS
making our code extremely portable. However, to actually run our node we'll want a ROS system to connect to.

One way to set this up would be to go through a full ROS installation for either `ROS1` or `ROS2`, but the recommended approach for roslibrust is use a ROS installation inside a docker container.
This unfortunately introduces the complexity of docker, but it is a very portable and repeatable way to setup a ROS environment.
Furthermore, it makes it extremely easy to experiment with multiple versions of ROS!

ROS provides docker images for both ROS1 and ROS2 on their [dockerhub page](https://hub.docker.com/_/ros).

RosLibRust is publishing extended docker images that include a rust installation and the `rosbridge_server` package.
We use these images for developing RosLibRust, and to run our CI tests.

To startup a ROS2 kilted rosbridge server you can run the following commands:

```bash
# This will startup a docker container with everything installed in it and drop you into a bash shell inside of the container
docker run -it --network host carter12s/roslibrust-ci-kilted:latest bash
# This will now activate the ROS2 installation inside the container
source /opt/ros/kilted/setup.bash
# This will start up the ROS2 zenoh router, and leave it running in the background
ros2 run rmw_zenoh_cpp rmw_zenohd & disown
# This will start the rosbridge server on the default port of 9090
ros2 run rosbridge_server rosbridge_websocket
```

Now in a separate terminal we can run our node.
To be able to actually see what our node is doing we'll enable debug logging with the `RUST_LOG` environment variable ([more info on RUST_LOG](https://docs.rs/env_logger/latest/env_logger/)):

```bash
RUST_LOG=debug cargo run
```

We should now see our terminal output something like:

```txt
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 2.15s
     Running `/home/carter/roslibrust/target/debug/examples/getting_started_1`
[2025-09-21T19:39:54Z DEBUG roslibrust_rosbridge::client] Starting a stubborn_connect attempt to ws://localhost:9090
[2025-09-21T19:39:54Z DEBUG tungstenite::handshake::client] Client handshake done.
[2025-09-21T19:39:54Z DEBUG roslibrust_rosbridge::client] Starting stubborn_spin
[2025-09-21T19:39:54Z DEBUG roslibrust_rosbridge::client] Advertise got lock on comm
[2025-09-21T19:39:54Z DEBUG roslibrust_rosbridge::comm] Sending advertise: Text("{\"op\":\"advertise\",\"topic\":\"/example_counter\",\"type\":\"std_msgs/UInt32\"}")
[2025-09-21T19:39:54Z DEBUG roslibrust_rosbridge::client] Publish got write lock on comm
[2025-09-21T19:39:54Z DEBUG roslibrust_rosbridge::comm] Sending publish: Text("{\"msg\":{\"data\":0},\"op\":\"publish\",\"topic\":\"/example_counter\",\"type\":\"std_msgs/UInt32\"}")
[2025-09-21T19:39:55Z DEBUG roslibrust_rosbridge::client] Publish got write lock on comm
```

We can also confirm everything is working by looking at the output of the rosbridge server in our other terminal.
We should see a "Client connected" message when we startup our node and a "Client disconnected" message when we ctrl+c out of our node.

```txt
[INFO] [1758483594.506878271] [rosbridge_websocket]: Client connected. 1 clients total.
[INFO] [1758483600.897845391] [rosbridge_websocket]: Client disconnected. 0 clients total.
```

## Extending Our Node to Subscribe

Let's have our node now subscribe to its same topic, and "talk to itself".

Along side our `pub_counter` let's add a `sub_counter` function:

```rust,ignore
{{#include ../../example_package/examples/getting_started_2.rs:subscriber}}
```

And then we'll modify our `main` function to spawn both behaviors:

```rust,ignore
{{#include ../../example_package/examples/getting_started_2.rs:main}}
```

Note: you can now see that we're calling `.clone()` on our `ros` and `state` variables when we spawn our tasks.
For both of these variables, that creates an additional "handle" to the underlying data that can be owned by the new task.
Previously, we we're `moving` ownership of these variables into our `pub_counter` task, but now that we want to use them in multiple tasks we need to clone them.

When we run this example (with our docker image still up in the background) we'll see logging like:

```txt
[2025-09-21T19:49:30Z DEBUG roslibrust_rosbridge::client] Publish got write lock on comm
[2025-09-21T19:49:30Z DEBUG roslibrust_rosbridge::comm] Sending publish: Text("{\"msg\":{\"data\":0},\"op\":\"publish\",\"topic\":\"/example_counter\",\"type\":\"std_msgs/UInt32\"}")
[2025-09-21T19:49:30Z DEBUG roslibrust_rosbridge::client] Got message: Text("{\"op\": \"publish\", \"topic\": \"/example_counter\", \"msg\": {\"data\": 0}}")
[2025-09-21T19:49:30Z DEBUG roslibrust_rosbridge::client] got message: {"op": "publish", "topic": "/example_counter", "msg": {"data": 0}}
Got message: 0
```

## Writing Tests for Our Node

Being able to unit and integration test our ROS code is a major feature of RosLibRust.
Let's now ensure our `pub_counter` and `sub_counter` behaviors work together in a test.

At the end of `main.rs` add the following:

```rust,ignore
{{#include ../../example_package/examples/getting_started_2.rs:test}}
```

To understand this test you should first read Tokio's [testing guide](https://tokio.rs/tokio/topics/testing).

The key points are:

- Tokio's runtime can tell when all futures are block on "time pasing"
- When this happens it can determine which future will complete "soonest"
- It can then "fast forward" time to that point and poll all futures again
- This allows us to deterministically test our code that is driven by time "as fast as possible"

We can run this test with `cargo test` and see the following:

```txt
running 1 test
test test::test_pub_counter ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

If we want to see what our test prints to the terminal we can run `cargo test -- --no-capture` and see the following:

```txt
running 1 test
Got message: 0
Got message: 1
Got message: 2
Got message: 3
Got message: 4
Got message: 5
Got message: 6
Got message: 7
Got message: 8
Got message: 9
Test took in realtime Ok(167.189µs)
test test::test_pub_sub_counter ... ok
```

This test takes only 167 microseconds to run! This is because Tokio is able to deterministically fast forward time to the point where our futures will complete.

## Conclusions

In this tutorial we've shown:

- How to setup a new crate to use roslibrust
- How to write generic ROS behaviors that can be tested in isolation and use any ROS backend
- How to run our node against a real ROS system using docker
- How to write a simple integration test that uses the mock ROS backend to test multiple behaviors together

After this you should have a good understanding of how to use RosLibRust to build and test ROS nodes in rust.
