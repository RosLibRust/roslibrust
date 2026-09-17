# roslibrust_hiroz

Native ROS 2 backend for [roslibrust](https://crates.io/crates/roslibrust), built on
[hiroz](https://crates.io/crates/hiroz) and compatible with ROS 2's `rmw_zenoh` middleware.

This backend supports ROS 2 Kilted and newer distributions that use `rmw_zenoh`.
Start `rmw_zenohd` before creating a client, then configure a `hiroz::context::ZContext`
for its endpoint and construct a `ZenohClient`.
