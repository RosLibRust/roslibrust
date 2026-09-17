#![doc = include_str!("../README.md")]

// Re-export common types and traits under the roslibrust namespace
pub use roslibrust_common::*;

// If the ros1 feature is enabled, export the roslibrust_ros1 crate under ros1
#[cfg(feature = "ros1")]
pub use roslibrust_ros1 as ros1;

// If the rosbridge feature is enabled, export the roslibrust_rosbridge crate under rosbridge
#[cfg(feature = "rosbridge")]
pub use roslibrust_rosbridge as rosbridge;

// If the zenoh feature is enabled, export the roslibrust_zenoh crate under zenoh
#[cfg(feature = "zenoh")]
/// Test doc?
pub use roslibrust_zenoh as zenoh;

// If the mock feature is enabled, export the roslibrust_mock crate under mock
#[cfg(feature = "mock")]
pub use roslibrust_mock as mock;

// If the codegen feature is enabled, export the roslibrust_codegen crate under codegen
#[cfg(feature = "codegen")]
pub use roslibrust_codegen as codegen;

// If the macro feature is enabled, export the roslibrust_codegen_macros directly
#[cfg(feature = "macro")]
pub use roslibrust_codegen_macro::find_and_generate_ros_messages;
#[cfg(feature = "macro")]
pub use roslibrust_codegen_macro::find_and_generate_ros_messages_without_ros_package_path;
#[cfg(feature = "macro")]
pub use roslibrust_codegen_macro::generate_ros_types;
#[cfg(feature = "macro")]
pub use roslibrust_codegen_macro::generate_ros_types_with_env;
