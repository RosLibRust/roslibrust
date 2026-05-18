# roslibrust_codegen_macro
This crate defines the proc-macro for generating ROS message types. It invokes APIs from the `roslibrust_codegen` crate.

## Warning
This macro cannot detect if the message files it generates from have changed. If you are creating a crate which just contains your message definitions and won't be recompiled otherwise, you'll likely want to use `roslibrust_codegen` with a `build.rs` script.

## Usage
If you're generating messages in an environment with ROS installed, no arguments need to be passed.
ROS1 packages are found through `ROS_PACKAGE_PATH`; ROS2 packages are found through
the ament resource indexes in `AMENT_PREFIX_PATH` and `COLCON_PREFIX_PATH`.

```rust
use roslibrust_codegen_macro::generate_ros_types_with_env;

generate_ros_types_with_env!();
```

If you're generating without ROS installed or your environment can't depend on ROS
environment variables, you can specify the exact paths to search:

```rust
use roslibrust_codegen_macro::generate_ros_types;

generate_ros_types!("/path/to/my/msg/package", "/opt/ros/noetic");
```
