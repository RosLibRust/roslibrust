//! # roslibrust_common
//! This crate provides common types and traits used throughout the roslibrust ecosystem.

/// The central error type used throughout roslibrust.
///
/// For now all roslibrust backends must coerce their errors into this type.
/// We may in the future allow backends to define their own error types, for now this is a compromise.
///
/// Additionally, this error type is returned from all roslibrust function calls so failure types must be relatively generic.
#[derive(thiserror::Error, Debug)]
pub enum Error {
    /// Is returned when communication is fully lost.
    /// While this error is being returned messages should be assumed to be being lost.
    /// Backends are expected to be "self-healing" and when connection is restored existing Publishers, Subscribers, etc.
    /// should resume functionality without needing to be recreated.
    #[error("No connection to ROS backend")]
    Disconnected,
    /// Some backends aren't able to conclusively determine if an operation has failed.
    /// Timeout will be returned if an operation takes a unexpectedly long time.
    /// For the `rosbridge` backend where this is most frequently encountered the timeout is configurable on the client.
    #[error("Operation timed out: {0}")]
    Timeout(String),
    /// When a message is received but the backend is unable to serialize/deserialize it to the Rust type representing the message type.
    ///
    /// This error is also returned in the event of an md5sum mismatch.
    #[error("Serialization error: {0}")]
    SerializationError(String),
    /// When the backend "server" reports an error this type is returned.
    ///
    /// This can happen when there are internal issues on the rosbridge_server, or with xmlrpc communication with the ros1 master.
    #[error("Rosbridge server reported an error: {0}")]
    ServerError(String),
    /// Returned when there is a fundamental networking error.
    ///
    /// Typically reserved for situations when ports are unavailable, dns lookups fail, etc.
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    /// When a topic name is used that isn't a valid topic name.
    #[error("Name does not meet ROS requirements: {0}")]
    InvalidName(String),
    /// Backends are free to return this error if they encounter any error that doesn't cleanly fit in the other categories.
    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}

/// Generic result type used throughout roslibrust.
pub type Result<T> = std::result::Result<T, Error>;

/// The error type used by [ServiceFn]
///
/// When writing service callbacks this is the error type that should be returned.
pub type ServiceError = anyhow::Error;

/// A generic message type used by some implementations to provide a generic subscriber / publisher without serialization
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct ShapeShifter(Vec<u8>);

// The equivalent of rospy AnyMsg or C++ ShapeShifter, subscribe_any() uses this type
impl RosMessageType for ShapeShifter {
    const ROS_TYPE_NAME: &'static str = "*";
    const MD5SUM: &'static str = "*";
    const DEFINITION: &'static str = "";
}

/// Contains functions for calculating md5sums of message definitions.
///
/// These functions are needed both in roslibrust_ros1 and roslibrust_codegen so they're in this crate
/// for the moment.
pub mod md5sum;

/// Contains the generic traits represent a pubsub system and service system.
/// These traits will be implemented for specific backends to provides access to "ROS Like" functionality.
pub mod traits;
pub use traits::*; // Bring topic provider traits into root namespace

/// Contains the validation logic for topic, service, and action names.
pub mod topic_name;
pub use topic_name::*; // Bring topic name validation into root namespace
