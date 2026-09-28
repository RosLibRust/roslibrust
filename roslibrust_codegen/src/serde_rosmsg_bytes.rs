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
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serializer};

/// Serialize a `Vec<u8>` as a base64 string
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

/// Deserialize a `Vec<u8>` from either a base64 string or binary format
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

/// Serialize a fixed-size `[u8; N]` as a base64 string
///
/// This allows fixed-length byte arrays (like UUIDs) to be compatible
/// with rosbridge's protocol which expects uint8 arrays as base64.
pub fn fixed_serialize<S, const N: usize>(bytes: &[u8; N], serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    if serializer.is_human_readable() {
        let encoded = STANDARD.encode(bytes);
        serializer.serialize_str(&encoded)
    } else {
        let mut seq = serializer.serialize_tuple(N)?;
        for byte in bytes {
            seq.serialize_element(byte)?;
        }
        seq.end()
    }
}

/// Deserialize a fixed-size `[u8; N]` from either a base64 string or binary format
pub fn fixed_deserialize<'de, D, const N: usize>(deserializer: D) -> Result<[u8; N], D::Error>
where
    D: Deserializer<'de>,
{
    use serde::de::{Error, SeqAccess, Visitor};
    use std::fmt;

    if deserializer.is_human_readable() {
        struct Base64FixedVisitor<const N: usize>;

        impl<'de, const N: usize> Visitor<'de> for Base64FixedVisitor<N> {
            type Value = [u8; N];

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                write!(formatter, "a base64 string decoding to exactly {} bytes", N)
            }

            fn visit_str<E: Error>(self, v: &str) -> Result<Self::Value, E> {
                let mut buf = [0u8; N];
                match STANDARD.decode_slice(v, &mut buf) {
                    Ok(len) if len == N => Ok(buf),
                    Ok(len) => Err(E::custom(format!(
                        "Expected exactly {} bytes, but decoded {}",
                        N, len
                    ))),
                    Err(e) => Err(E::custom(format!("Failed to decode base64 string: {}", e))),
                }
            }
        }

        deserializer.deserialize_str(Base64FixedVisitor::<N>)
    } else {
        // Binary formats expect fixed arrays to be read directly without a length prefix.
        struct ArrayVisitor<const N: usize>;

        impl<'de, const N: usize> Visitor<'de> for ArrayVisitor<N> {
            type Value = [u8; N];

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                write!(formatter, "an array of {} bytes", N)
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<[u8; N], A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut arr = [0u8; N];
                for i in 0..N {
                    arr[i] = seq
                        .next_element()?
                        .ok_or_else(|| Error::invalid_length(i, &self))?;
                }
                Ok(arr)
            }
        }

        deserializer.deserialize_tuple(N, ArrayVisitor::<N>)
    }
}
