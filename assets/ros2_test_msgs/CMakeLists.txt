# This CMake exists so we can trigger message compilation for this package
# And get correct .json and hashes for testing against
cmake_minimum_required(VERSION 3.8)
project(ros2_test_msgs)

# Needed for rosidl_generate_interfaces
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the message and service files
set(msg_files
  "msg/Bool.msg"
  "msg/BoundedInt.msg"
  "msg/BoundedReferenced.msg"
  "msg/BoundedString.msg"
  "msg/Defaults.msg"
  "msg/Stamped.msg"
  "msg/Char.msg"
)

set(srv_files
  "srv/AddTwoInts.srv"
)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
