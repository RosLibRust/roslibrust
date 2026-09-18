//! Serde adapter for ROS byte sequences.
//!
//! Rosbridge represents `uint8`/`byte`/`char` arrays as base64 strings in JSON, while the native
//! ROS serializers must preserve the distinction between a variable-length byte vector and a
//! fixed-length byte array. The public `serialize` and `deserialize` functions support both Rust
//! representations so generated fields can use one uniform `#[serde(with = ...)]` attribute.

use base64::{engine::general_purpose::STANDARD, Engine};
use serde::{
    de::{Error as _, SeqAccess, Visitor},
    ser::SerializeTuple,
    Deserializer, Serializer,
};
use std::fmt;

/// Implementation detail used to retain native ROS framing for each byte container.
#[doc(hidden)]
pub trait ByteSequence: AsRef<[u8]> + Sized {
    fn from_vec<E: serde::de::Error>(bytes: Vec<u8>) -> Result<Self, E>;

    fn serialize_binary<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error>;

    fn deserialize_binary<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error>;
}

impl ByteSequence for Vec<u8> {
    fn from_vec<E: serde::de::Error>(bytes: Vec<u8>) -> Result<Self, E> {
        Ok(bytes)
    }

    fn serialize_binary<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serde_bytes::serialize(self, serializer)
    }

    fn deserialize_binary<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        serde_bytes::deserialize(deserializer)
    }
}

impl<const N: usize> ByteSequence for [u8; N] {
    fn from_vec<E: serde::de::Error>(bytes: Vec<u8>) -> Result<Self, E> {
        let actual = bytes.len();
        bytes
            .try_into()
            .map_err(|_| E::invalid_length(actual, &FixedLengthExpectation::<N>))
    }

    fn serialize_binary<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut tuple = serializer.serialize_tuple(N)?;
        for byte in self {
            tuple.serialize_element(byte)?;
        }
        tuple.end()
    }

    fn deserialize_binary<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_tuple(N, FixedArrayVisitor::<N>)
    }
}

/// Serialize any generated ROS byte sequence.
pub fn serialize<T, S>(bytes: &T, serializer: S) -> Result<S::Ok, S::Error>
where
    T: ByteSequence,
    S: Serializer,
{
    if serializer.is_human_readable() {
        serializer.serialize_str(&STANDARD.encode(bytes.as_ref()))
    } else {
        bytes.serialize_binary(serializer)
    }
}

/// Deserialize any generated ROS byte sequence.
///
/// Human-readable formats accept both rosbridge's base64 representation and a numeric array. The
/// latter keeps the generated types friendly to other JSON producers.
pub fn deserialize<'de, T, D>(deserializer: D) -> Result<T, D::Error>
where
    T: ByteSequence,
    D: Deserializer<'de>,
{
    if deserializer.is_human_readable() {
        deserializer.deserialize_any(HumanReadableVisitor::<T>(std::marker::PhantomData))
    } else {
        T::deserialize_binary(deserializer)
    }
}

struct HumanReadableVisitor<T>(std::marker::PhantomData<T>);

impl<'de, T: ByteSequence> Visitor<'de> for HumanReadableVisitor<T> {
    type Value = T;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a base64 string or an array of bytes")
    }

    fn visit_str<E: serde::de::Error>(self, value: &str) -> Result<Self::Value, E> {
        let bytes = STANDARD
            .decode(value)
            .map_err(|error| E::custom(format!("failed to decode base64 string: {error}")))?;
        T::from_vec(bytes)
    }

    fn visit_string<E: serde::de::Error>(self, value: String) -> Result<Self::Value, E> {
        self.visit_str(&value)
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut sequence: A) -> Result<Self::Value, A::Error> {
        let mut bytes = Vec::with_capacity(sequence.size_hint().unwrap_or(0));
        while let Some(byte) = sequence.next_element()? {
            bytes.push(byte);
        }
        T::from_vec(bytes)
    }
}

struct FixedLengthExpectation<const N: usize>;

impl<const N: usize> serde::de::Expected for FixedLengthExpectation<N> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "exactly {N} bytes")
    }
}

struct FixedArrayVisitor<const N: usize>;

impl<'de, const N: usize> Visitor<'de> for FixedArrayVisitor<N> {
    type Value = [u8; N];

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "exactly {N} bytes")
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut sequence: A) -> Result<Self::Value, A::Error> {
        let mut bytes = [0; N];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = sequence
                .next_element()?
                .ok_or_else(|| A::Error::invalid_length(index, &self))?;
        }
        Ok(bytes)
    }
}

#[cfg(test)]
mod tests {
    use serde::{Deserialize, Serialize};

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct ByteFields {
        #[serde(with = "crate::serde_rosmsg_bytes")]
        dynamic: Vec<u8>,
        #[serde(with = "crate::serde_rosmsg_bytes")]
        fixed: [u8; 4],
    }

    #[test]
    fn json_uses_base64_for_vectors_and_arrays() {
        let fields = ByteFields {
            dynamic: vec![1, 2, 3],
            fixed: [4, 5, 6, 7],
        };

        let json = serde_json::to_string(&fields).unwrap();
        assert_eq!(json, r#"{"dynamic":"AQID","fixed":"BAUGBw=="}"#);
        assert_eq!(serde_json::from_str::<ByteFields>(&json).unwrap(), fields);
    }

    #[test]
    fn json_also_accepts_numeric_arrays() {
        let fields =
            serde_json::from_str::<ByteFields>(r#"{"dynamic":[1,2,3],"fixed":[4,5,6,7]}"#).unwrap();

        assert_eq!(fields.dynamic, [1, 2, 3]);
        assert_eq!(fields.fixed, [4, 5, 6, 7]);
    }

    #[test]
    fn fixed_array_rejects_wrong_base64_length() {
        let error =
            serde_json::from_str::<ByteFields>(r#"{"dynamic":"AQID","fixed":"BAUG"}"#).unwrap_err();

        assert!(error.to_string().contains("exactly 4 bytes"));
    }
}
