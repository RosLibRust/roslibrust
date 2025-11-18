#!/bin/bash
# Publish all public roslibrust packages to crates.io in dependency order
set -e

# Must publish in this order due to dependencies
PACKAGES=(
    roslibrust_common
    roslibrust_codegen
    roslibrust_codegen_macro
    roslibrust_mock
    roslibrust_ros1
    roslibrust_rosbridge
    roslibrust_zenoh
    roslibrust_genmsg
    roslibrust
)

for pkg in "${PACKAGES[@]}"; do
    cargo publish -p "$pkg"
done

