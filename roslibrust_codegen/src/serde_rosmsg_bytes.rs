/// Custom serde module for handling Vec<u8> with rosbridge's base64 encoding
///
/// Rosbridge encodes uint8[] arrays as base64 strings in JSON, which is different from
/// the standard JSON array format that serde_bytes expects. This module provides
/// serialize/deserialize functions that handle both formats transparently.
///
/// This allows the same generated code to work with:
/// - Rosbridge (which sends/receives base64 strings)
/// - Other JSON-based formats (which might use arrays)
/// - Binary formats (which use raw bytes)
use base64::{engine::general_purpose::STANDARD, Engine};
use serde::{Deserialize, Deserializer, Serializer};

/// Serialize a Vec<u8> as a base64 string
///
/// This is compatible with rosbridge's protocol which expects uint8[] as base64.
pub fn serialize<S>(bytes: &Vec<u8>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    // Check if we're serializing to a human-readable format (like JSON)
    if serializer.is_human_readable() {
        // For human-readable formats (JSON), use base64 encoding
        let encoded = STANDARD.encode(bytes);
        serializer.serialize_str(&encoded)
    } else {
        // For binary formats, use serde_bytes for efficiency
        serde_bytes::serialize(bytes, serializer)
    }
}

/// Deserialize a Vec<u8> from either a base64 string or binary format
///
/// This allows the same generated code to work with:
/// - Rosbridge (which sends base64 strings)
/// - Binary formats (which send raw bytes)
pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<u8>, D::Error>
where
    D: Deserializer<'de>,
{
    use serde::de::Error;

    // Check if we're deserializing from a human-readable format (like JSON)
    if deserializer.is_human_readable() {
        // For human-readable formats (JSON/rosbridge), decode base64 strings
        let s = String::deserialize(deserializer)?;
        STANDARD
            .decode(&s)
            .map_err(|e| D::Error::custom(format!("Failed to decode base64 string: {}", e)))
    } else {
        // For binary formats (ROS1 native, ROS2 CDR), use serde_bytes for efficiency
        serde_bytes::deserialize(deserializer)
    }
}
