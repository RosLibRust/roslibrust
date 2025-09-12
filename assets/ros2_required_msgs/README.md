# ros2_required_msgs

Struggling with how we quite want to support this in with ROS2 message generation.

There are some types that ROS2 message generation assumes are available for hashing purposes:

- builtin_interfaces/Time
- builtin_interfaces/Duration
- service_msgs/ServiceEventInfo

We either basically have to ship roslibrust with hardcoded values for these, or force users to provide them if any
ROS2 hashing is desired.
