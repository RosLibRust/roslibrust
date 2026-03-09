//! Error types for MCAP operations

use thiserror::Error;

/// Errors that can occur when working with MCAP files
#[derive(Error, Debug)]
pub enum McapError {
    /// Error from the underlying MCAP library
    #[error("MCAP error: {0}")]
    Mcap(#[from] mcap::McapError),

    /// IO error when reading or writing files
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Error deserializing a message
    #[error("Deserialization error: {0}")]
    Deserialization(String),

    /// Error serializing a message
    #[error("Serialization error: {0}")]
    Serialization(String),

    /// Channel not found
    #[error("Channel not found: {0}")]
    ChannelNotFound(u16),

    /// Invalid message encoding
    #[error("Invalid message encoding: {0}")]
    InvalidEncoding(String),

    /// Other errors
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

/// Result type for MCAP operations
pub type Result<T> = std::result::Result<T, McapError>;
