# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

### Fixed

- ROS1 xmlrpc servers and TCPROS servers now much more closely follow roscpp behavior. They both explicitly listen on "0.0.0.0" instead of attempting to only listen on a specific interface.
They also correctly fallback to using the IP address of the first interface in the same subnet as the ROS master if neither ROS_IP, ROS_HOSTNAME, or the system's hostname resolve to a valid IP address.
- ROS1 subscribers no longer log errors when a TCP connection fails to reply with a connection header. Some ROS tools appear to "probe" where they start a connection just to get the header.

### Changed

- Reorganized some of the module structure of roslibrust_common. Users pulling things from `roslibrust` will see no change.
- All API calls that were taking a topic name as "&str" now take a "impl ToGlobalTopicName" instead.
This allows for more ergonomic usage of the API, and allows us to extend the API to support substitutions in the future.
The API is now more strict about the format of topic names. See the documentation for GlobalTopicName for more details.
Previously names like "chatter" could be handled differently be different backends that may or may not have added a leading slash automatically.
Now all names **MUST** be a fully resolved name.

## 0.18.0 - November 25th, 2025

### Added

### Fixed

### Changed

- The Subscriber, Publisher, ServiceServer, and ServiceClient associated types on TopicProvider and ServiceProvider now require + Sync for their bounds.
This is specifically designed to make writing async nodes easier as holding a reference to one of these types across an await point is now valid.

## 0.17.0 - November 18th, 2025

### Added

- ROS1 backend now exposes `subscribe_any` to allow subscribing to any topic and receiving raw bytes instead of a deserialized message.

### Fixed

- MockRos ServiceClients now work correctly if created before the service server is advertised.
- MockRos ServiceClients now perform a yield_now() before calling a service to help avoid race conditions in tests.
- Rosbridge backend was having serialization issues with uint8[] and char[] due to undocumented behavior of rosbridge_server using base64 encoding for uint8[] and char[] arrays. This has been fixed.

### Changed

- ROS2 message hashes are now [u8; 32] instead of String and codegen will now fail if it cannot be computed instead of leaving a default value.

## 0.16.0 - October 8th, 2025

### Added

- roslibrust_codegen now support time conversions from chrono::DateTime and chrono::Duration to roslibrust's internal types.
- Subscribers now have a `.to_stream()` method that converts them to a [futures_core::Stream] for easy integration with async Rust. See [examples/tokio_stream_operations.rs](https://github.com/RosLibRust/roslibrust/blob/master/roslibrust/examples/tokio_stream_operations.rs) for usage.

### Fixed

- Removed an un-used dependency on tokio from roslibrust_codegen.

### Changed

- The return type of service functions has been changed from "Box<dyn std::error::Error + Send + Sync + 'static>" to `roslibrust::ServiceError`.
Which is an alias for `anyhow::Error`. This should make it easier to return errors from service functions, and deals with some tricky lifetime issues around holding ServiceServers

## 0.15.0 - June 20th, 2025

### Added

### Fixed

- Make unusual ros message definitions less likely to trip up roslibrust_codegen.

### Changed

- roslibrust::ServiceProvider::ServiceServer is now Send + 'static, fixing several usage issues with holding generic service handles

## 0.14.0 - Apr 7th, 2025

### Added

- Examples and documentation for performing `async` actions in service callbacks.

### Fixed

### Changed

- All user provided service server functions are now executed inside of a `tokio::task::spawn_blocking` call, and blocking withing the service function is now safe.

## 0.13.0 - March 27th, 2025

### Added

- A new `Ros` trait has been added that all backends implement. This trait behaves like a typical ROS1 node handle, and is easier to work with in generics than directly using TopicProvider or ServiceProvider.

### Fixed

### Changed

- ZenohClient is now clone so it can implement the `Ros` trait
- MockRos is now clone so it can implement the `Ros` trait

## 0.12.2 - January 8th, 2025

### Added

### Fixed

- roslibrust_serde_rosmsg verison updated resulting in a significant performance boost on serializing large messages. 95% gains measured on 1080p color image serialization. 25% gains measured on roundtrip image benchmarks with ros1 backend.

### Changed

## 0.12.1 - YANKED packaging mistake

## 0.12.0 - January 7th, 2025

### Added

- roslibrust_mock now provides a basic mock implementation of roslibrust's generic traits for use in building automated testing of nodes.
- roslibrust_zenoh now proivides a Zenoh client that is compatible with the zenoh-ros1-plugin / zenoh-ros1-bridge
- roslibrust_ros1 now provides a ROS1 native client as a standalone crate
- roslibrust_rosbridge now provides a rosbridge client as a standalone crate
- roslibrust_rosapi now provides a generic interface for the rosapi node compatible with both rosbridge and ros1 backends

### Fixed

- Keeping a ros1::ServiceServer alive no longer keeps the underlying node alive past the last ros1::NodeHandle being dropped.
- Dropping the last ros1::NodeHandle results in the node cleaning up any advertises, subscriptions, and services with the ROS master.
- Generated code now includes various lint attributes to suppress warnings.
- TCPROS header parsing now ignores (the undocumented fields) response_type and request_type and doesn't produce warnings on them.
- ROS1 native service servers now respect the "persistent" header field, and automatically close the underlying TCP socket after a single request unless persistent is set to 1 in the connection header.

### Changed

- A major reorganization of the internals of roslibrust was done to improve our ability to support multiple backends.
  As part of the organization the rosbridge backend has been moved under the the `rosbridge` module, so
  `roslibrust::ClientHandle` now becomes `roslibrust::rosbridge::ClientHandle`.
- Internal integral type Time changed from u32 to i32 representation to better align with ROS1
- Conversions between ROS Time and Duration to std::time::Time and std::time::Duration switched to TryFrom as they can be fallible.
- Main crate Error type has been simplified to leak fewer internal types.
- roslibrust_codegen is now intended to be accessed by users via the `codegen` feature flag on roslibrust.
- roslibrust_codegen_macro is now intended to be accessed by users via the `macro` feature flag on roslibrust.
- Code generated by roslibrust_codegen now depends on `roslibrust` with the `codegen` feature flag being available to it.

## 0.11.1

### Added

### Fixed

- ROS1 Native Publishers no longer occasionally truncate very large messages when configured with latching

### Changed

- Passing of large messages containing uint8[] arrays is now substantially faster
- Generated code now relies on serde_bytes to enable faster handling of uint8[] arrays
- Switched to a fork of serde_rosmsg to enable faster handling of uint8[] arrays

## 0.11.0

### Added

- ROS1 Native Publishers now support latching behavior
- The XML RPC client for interacting directly with the rosmaster server has been exposed as a public API
- Experimental: Initial support for writing generic clients that can be compile time specialized for rosbridge or ros1
- Can subscribe to any topic and get raw bytes instead of a deserialized message of known type
- Can publish to any topic and send raw bytes instead of a deserialized message

### Fixed

- ROS1 Native Publishers correctly call unadvertise when dropped
- ROS1 Native Publishers no longer occasionally truncate very large messages (>5MB)

### Changed

- ROS1 Node Handle's advertise() now requires a latching argument
- RosBridge ClientHandle [call_service()] is now templated on the service type instead of on both the request and response type.
This is to bring it in line with the ROS1 API.
- RosBridge ClientHandle::Publisher publish() now takes a reference to the message instead of taking ownership.
This is to bring it in line with the ROS1 API.
- The RosServiceType trait used by codegen now requires 'static + Send + Sync this will not affect any generated types, but custom types may have issue.

## 0.10.2 - August 3rd, 2024

### Added

### Fixed

- Bug with message_definitions provided by Publisher in the connection header not being the fully expanded definition.
- Bug with ROS1 native subscribers not being able to receive messages larger than 4096 bytes.

### Changed

## 0.10.1 - July 5th, 2024

### Added

### Fixed

- Bug with ros1 native publishers not parsing connection headers correctly

### Changed

## 0.10.0 - July 5th, 2024

### Added

- ROS1 native service servers and service clients are now supported (experimental feature)

### Fixed

- The reconnection logic for rosbridge clients was fundamentally broken and failing to reconnect. This has been fixed.
- Generic subscriptions coming from rospy that specified "*" as the md5sum were not properly handled. This has been fixed.

### Changed

- Code generated by roslibrust_codegen now depends only on libraries exposed by roslibrust_codegen. This means that
crates that were previously adding dependencies on serde, serde-big-array, and smart-default will no longer need to do so.
- A significant reworking of the error types in the ROS1 native client was performed to move away from the `Box<dyn Error + Send + Sync>` pattern and instead use the `anyhow` crate.

## 0.9.0 - May 13th, 2024

### Added

- The build.rs example in example_package now correctly informs cargo of filesystem dependencies
- The `advertise_service` method in `rosbridge/client.rs` now accepts closures
- Expose additional methods useful for custom cases not using package manifests or standard ROS2 setups

### Fixed

- Messages containing fixed sized arrays now successfully serialize and deserialize when using ROS1 native communication

### Changed

- The function interface for top level generation functions in `roslibrust_codegen` have been changed to include the list of dependent
filesystem paths that should trigger re-running code generation. Note: new files added to the search paths will not be automatically detected.
- [Breaking Change] Codegen now generates fixed sized arrays as arrays [T; N] instead of Vec<T>
- Removed `find_and_generate_ros_messages_relative_to_manifest_dir!` this proc_macro was changing the current working directory of the compilation job resulting in a variety of strange compilation behaviors. Build.rs scripts are recommended for use cases requiring fine grained control of message generation.
- The function interface for top level generation functions in `roslibrust_codegen` have been changed to include the list of dependent filesystem paths that should trigger re-running code generation. Note: new files added to the search paths will not be automatically detected.
- Refactor the `ros1::node` module into separate smaller pieces. This should be invisible externally (and no changes to examples were required).

## 0.8.0 - October 4th, 2023

### Added

- Experimental support for ROS1 native communication behind the `ros1` feature flag
- Generation of C++ source added via `roslibrust_genmsg` along with arbitrary languages via passed in templates
- Generation of Rust source for actions
- Example for custom generic message usage with rosbridge
- Example for async native ROS1 listener
- Example for async native ROS1 publisher

### Fixed

- Incorrect handling of ROS1 message string constants

### Changed

- `crawl` function in `roslibrust_codegen` updated to a more flexible API
- Overhaul of error handling in roslibrust_codegen to bubble errors up, and remove use of panic! and unwrap(). Significantly better error messages should be produced from proc_macros and build.rs files. Direct usages of the API will need to be updated to handle the returned error type.
- RosMessageType trait now has associated constants for MD5SUM and DEFINITION to enable ROS1 native support. These constants are optional at this time with a default value of "" provided.

## 0.7.0 - March 13, 2022

### Added

- Support for default field values in ROS2 messages
- Added public APIs for getting message data from search and for generating Rust code given message data in roslibrust_codegen
- More useful logs available when running codegen
- Refactor some of the public APIs and types in roslibrust_codegen (concept of `ParsedMessageFile` vs `MessageFile`)
- Added a method `get_md5sum` to `MessageFile`
- Additional code generation API and macro which excludes `ROS_PACKAGE_PATH`

### Fixed

- Bug causing single quoted string constants in message files to not be parsed correctly
- Bug causing float constants in message files to cause compiler errors because `f32 = 0;` is not allowed in rust
- Bug where packages were not properly de-duplicated during discovery.

### Changed

- `advertise_service` and `subscribe` methods on ClientHandle were changed from needing `&mut self` to `&self`

## 0.6.0 - December 12, 2022

### Added

- Initial support for ROS2 message generation
- Initial integration testing for ROS2 all basic functionality covered
- CI testing for Humble

### Fixed

- The generated `char` type within rust is now u8.
- Package names are now determined by the `name` tag within package.xml instead of by directory name

## 0.5.2 - October 31, 2022

### Changed

- No longer generate empty `impl` blocks from message structs that have not associated constants
- Significant improvement to documentation with expanded examples

### Fixed

- `advertise_service` no longer panics if multiple advertise attempts made to same topic

## 0.5.1 - September 18, 2022

Fix to docs.rs build.

## 0.5.0 - September 18, 2022

### Added

- Service server example

### Changed

- `Client` is now `ClientHandle`
- All identifiers in generated code are now escaped with `r#`
- `advertise_service` now returns a `ServiceHandle` which controls the lifetime of the service

### Fixed

- Fixed issue where the spin and reconnect context would never drop even if there were no more `ClientHandle`s
- Fixed parsing issue with triple dashes in comments of service files
- Fixed bug in generation where message properties or constants had names conflicting with Rust reserved keywords

## 0.4.0 - September 18, 2022

Yanked version due to failed publish

## 0.3.0 - September 18, 2022

Yanked version due to failed publish

## 0.2.0 - September 1, 2022

### Added

- Support for service servers
- Into<> helpers for Time and Duration for tokio and std

### Fixed

- Failure in message generation caused by files without extensions
- Failure in message generation caused by failing to quote around string constants
- Failure in message generation caused by bad design of integral types TimeI and DurationI

## 0.1.0 - August 28, 2022

Initial public release

## 0.0.3 - Unreleased

## 0.0.2 - Unreleased

## 0.0.1 - Unreleased

## For Developers

### Release Instructions

Steps:

- Starting on master
- Edit change log
- Revise the version numbers in Cargo.toml files
- Commit the changes
- Publish all crates using `./publish_all.sh` which invokes publish in the correct order
- Push to master
- Tag and push tag
