FROM ros:rolling-ros-core
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"

# Required by github CI for submodule support
RUN apt update && apt install -y git

# Install rosbridge suite for rosbridge backend testing
RUN apt update && apt install -y ros-rolling-rosbridge-suite

# Install rmw_zenoh for zenoh backend testing
RUN apt update && apt install -y ros-rolling-rmw-zenoh-cpp
# Set RMW_IMPLEMENTATION to rmw_zenoh_cpp for zenoh backend testing
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Curl required to install rust, build-essential required to build quote & proc-macro2
RUN apt update && apt install -y --fix-missing curl build-essential
# Install latest stable rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

