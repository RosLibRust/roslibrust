FROM ros:iron-ros-core
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"

# Required by github CI for submodule support
RUN apt update && apt install -y git

RUN apt update && apt install -y ros-iron-rosbridge-suite

# Install rosbag2 and MCAP storage plugin for MCAP integration testing
# Note: MCAP is not the default storage format in Iron, so we need the plugin
RUN apt update && apt install -y ros-iron-rosbag2 ros-iron-rosbag2-storage-mcap

# Curl required to install rust, build-essential required to build quote & proc-macro2
RUN apt update && apt install -y --fix-missing curl build-essential
# Install latest stable rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
