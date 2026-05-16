# ROS1 C++ Image Benchmark

This catkin package provides a roscpp comparison point for `roslibrust_test/benches/image_bench.rs`.

The default message matches the Rust benchmark:

- topic: `/image_bench`
- queue size: `1`
- latching: `false`
- image: `1920x1080`, `rgb8`, `3` bytes per pixel
- payload: `6,220,800` bytes

## Build

From any catkin workspace:

```bash
mkdir -p ~/cpp_image_bench_ws/src
ln -s /roslibrust/roslibrust_test/cpp_image_bench ~/cpp_image_bench_ws/src/
cd ~/cpp_image_bench_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Same-Process Benchmark

This is the closest shape to `image_bench.rs`, with one process that advertises and subscribes to the same topic:

```bash
roscore
```

In another shell:

```bash
source /opt/ros/noetic/setup.bash
source ~/cpp_image_bench_ws/devel/setup.bash
rosrun roslibrust_cpp_image_bench cpp_image_bench --mode same_process --iterations 1000 --warmup 25
```

Important: roscpp may use intra-process delivery in this mode, so this can be much faster than real TCPROS transport.

## TCPROS Benchmark

Use this mode for a better estimate of roscpp image transport across a real ROS1 publisher/subscriber connection. The reported timing is image publish to tiny ack receipt, so the extra ack cost is included but normally small next to the image transfer.

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source ~/cpp_image_bench_ws/devel/setup.bash
rosrun roslibrust_cpp_image_bench cpp_image_bench --mode responder
```

Terminal 3:

```bash
source /opt/ros/noetic/setup.bash
source ~/cpp_image_bench_ws/devel/setup.bash
rosrun roslibrust_cpp_image_bench cpp_image_bench --mode driver --iterations 1000 --warmup 25
```

The benchmark prints a final `json: {...}` line with nanosecond timings and payload throughput.
