use std::borrow::Cow;

use serde::ser::{SerializeMap, SerializeSeq};

mod value_serde;

/// One named field in a dynamically represented ROS message.
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicField {
    pub name: String,
    pub value: DynamicValue,
}

/// An unvalidated, format-neutral representation of a possible ROS message value.
///
/// Unlike a JSON value, this representation preserves integer widths, byte arrays, field order,
/// and non-finite floating-point values. It intentionally does not carry a ROS schema, so values
/// such as heterogeneous sequences can be represented temporarily. A value has ROS schema
/// guarantees only after [`MessageDescriptor::message`] or [`MessageRegistry::from_value`]
/// produces a [`DynamicMessage`].
#[derive(Clone, Debug, PartialEq)]
pub enum DynamicValue {
    Unit,
    Bool(bool),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    F32(f32),
    F64(f64),
    Char(char),
    String(String),
    Bytes(Vec<u8>),
    Sequence(Vec<DynamicValue>),
    Message(Vec<DynamicField>),
}

/// Serialize the natural value shape without attaching enum variant names.
///
/// This is intended for human-readable interchange formats. Format-specific policies, such as
/// how JSON should represent non-finite floats or bytes, remain the serializer's responsibility.
impl serde::Serialize for DynamicValue {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match self {
            Self::Unit => serializer.serialize_unit(),
            Self::Bool(value) => serializer.serialize_bool(*value),
            Self::I8(value) => serializer.serialize_i8(*value),
            Self::I16(value) => serializer.serialize_i16(*value),
            Self::I32(value) => serializer.serialize_i32(*value),
            Self::I64(value) => serializer.serialize_i64(*value),
            Self::U8(value) => serializer.serialize_u8(*value),
            Self::U16(value) => serializer.serialize_u16(*value),
            Self::U32(value) => serializer.serialize_u32(*value),
            Self::U64(value) => serializer.serialize_u64(*value),
            Self::F32(value) => serializer.serialize_f32(*value),
            Self::F64(value) => serializer.serialize_f64(*value),
            Self::Char(value) => serializer.serialize_char(*value),
            Self::String(value) => serializer.serialize_str(value),
            Self::Bytes(values) => serializer.serialize_bytes(values),
            Self::Sequence(values) => {
                let mut sequence = serializer.serialize_seq(Some(values.len()))?;
                for value in values {
                    sequence.serialize_element(value)?;
                }
                sequence.end()
            }
            Self::Message(fields) => {
                let mut message = serializer.serialize_map(Some(fields.len()))?;
                for field in fields {
                    message.serialize_entry(&field.name, &field.value)?;
                }
                message.end()
            }
        }
    }
}

/// A failure while looking up or converting a runtime-selected ROS message.
#[derive(Debug, thiserror::Error)]
pub enum DynamicMessageError {
    #[error("unknown ROS message type `{0}`")]
    UnknownType(String),
    #[error("invalid value for ROS message type `{type_name}`: {message}")]
    InvalidValue {
        type_name: &'static str,
        message: String,
    },
    #[error("failed to serialize ROS message type `{type_name}`: {message}")]
    Serialize {
        type_name: &'static str,
        message: String,
    },
    #[error("failed to deserialize ROS message type `{type_name}`: {message}")]
    Deserialize {
        type_name: &'static str,
        message: String,
    },
}

/// Result type for runtime message lookup and conversion.
pub type DynamicMessageResult<T> = std::result::Result<T, DynamicMessageError>;

type NormalizeFn = fn(&DynamicValue) -> DynamicMessageResult<DynamicValue>;
type SerializeFn = fn(&DynamicValue, &mut dyn erased_serde::Serializer) -> DynamicMessageResult<()>;
type DeserializeFn =
    for<'de> fn(&mut dyn erased_serde::Deserializer<'de>) -> DynamicMessageResult<DynamicValue>;

/// Generated operations for a single concrete message type.
#[derive(Clone, Copy)]
pub struct MessageOperations {
    normalize: NormalizeFn,
    serialize: SerializeFn,
    deserialize: DeserializeFn,
}

impl MessageOperations {
    #[doc(hidden)]
    pub const fn new(
        normalize: NormalizeFn,
        serialize: SerializeFn,
        deserialize: DeserializeFn,
    ) -> Self {
        Self {
            normalize,
            serialize,
            deserialize,
        }
    }
}

/// Metadata and conversion operations for one generated ROS message type.
#[derive(Clone, Copy)]
pub struct MessageDescriptor {
    pub ros_type_name: &'static str,
    pub md5sum: &'static str,
    pub definition: &'static str,
    pub ros2_type_name: &'static str,
    pub ros2_hash: &'static [u8; 32],
    operations: MessageOperations,
}

impl std::fmt::Debug for MessageDescriptor {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("MessageDescriptor")
            .field("ros_type_name", &self.ros_type_name)
            .field("md5sum", &self.md5sum)
            .field("definition", &self.definition)
            .field("ros2_type_name", &self.ros2_type_name)
            .field("ros2_hash", &self.ros2_hash)
            .finish_non_exhaustive()
    }
}

impl MessageDescriptor {
    #[doc(hidden)]
    pub const fn new(
        ros_type_name: &'static str,
        md5sum: &'static str,
        definition: &'static str,
        ros2_type_name: &'static str,
        ros2_hash: &'static [u8; 32],
        operations: MessageOperations,
    ) -> Self {
        Self {
            ros_type_name,
            md5sum,
            definition,
            ros2_type_name,
            ros2_hash,
            operations,
        }
    }

    /// Validate a dynamic value against this generated message type.
    pub fn validate(&self, value: &DynamicValue) -> DynamicMessageResult<()> {
        (self.operations.normalize)(value).map(|_| ())
    }

    /// Validate and canonicalize a value, then associate it with this descriptor.
    ///
    /// The stored value is regenerated from the concrete message type. This gives it schema field
    /// order, exact scalar widths, and homogeneous sequence element representations.
    pub fn message(&'static self, value: DynamicValue) -> DynamicMessageResult<DynamicMessage> {
        let value = (self.operations.normalize)(&value)?;
        Ok(DynamicMessage {
            descriptor: self,
            value,
        })
    }

    /// Serialize an arbitrary Serde value, validate it against this descriptor, and canonicalize it.
    ///
    /// Structs and string-keyed maps are interpreted as ROS messages. This is useful for command
    /// line tools that construct values using their own input types. Serialization succeeding does
    /// not bypass schema validation: missing fields, unknown fields, incorrect scalar types, and
    /// invalid sequence elements are rejected before a [`DynamicMessage`] is returned.
    pub fn message_from<T>(&'static self, value: &T) -> DynamicMessageResult<DynamicMessage>
    where
        T: serde::Serialize + ?Sized,
    {
        let value = value.serialize(value_serde::Serializer).map_err(|error| {
            DynamicMessageError::InvalidValue {
                type_name: self.ros_type_name,
                message: error.to_string(),
            }
        })?;
        self.message(value)
    }

    /// Deserialize a message using a deserializer selected by the caller.
    pub fn deserialize_with<'de, D>(
        &'static self,
        deserializer: D,
    ) -> DynamicMessageResult<DynamicMessage>
    where
        D: serde::Deserializer<'de>,
    {
        let mut deserializer = <dyn erased_serde::Deserializer<'de>>::erase(deserializer);
        let value = (self.operations.deserialize)(&mut deserializer)?;
        Ok(DynamicMessage {
            descriptor: self,
            value,
        })
    }
}

/// A schema-validated, canonical representation of a runtime-selected ROS message.
///
/// Unlike a raw [`DynamicValue`], values reachable through this type have been round-tripped
/// through their generated Rust message type. Required fields, field types, nested messages,
/// sequence element types, numeric ranges, and fixed array lengths have therefore been checked.
#[derive(Clone, Debug)]
pub struct DynamicMessage {
    descriptor: &'static MessageDescriptor,
    value: DynamicValue,
}

impl DynamicMessage {
    pub fn descriptor(&self) -> &'static MessageDescriptor {
        self.descriptor
    }

    pub fn value(&self) -> &DynamicValue {
        &self.value
    }

    pub fn into_value(self) -> DynamicValue {
        self.value
    }

    /// Serialize using a serializer selected by the caller.
    pub fn serialize_with<S>(&self, serializer: S) -> DynamicMessageResult<()>
    where
        S: serde::Serializer,
    {
        let mut serializer = <dyn erased_serde::Serializer>::erase(serializer);
        (self.descriptor.operations.serialize)(&self.value, &mut serializer)
    }
}

/// An immutable registry generated alongside a set of ROS message types.
///
/// Codegen emits a public `MESSAGE_REGISTRY` in the same scope as its generated package modules.
/// With `include!(...)` at crate root it is available as `crate::MESSAGE_REGISTRY`; if generation is
/// included inside `mod messages`, it is available as `messages::MESSAGE_REGISTRY`.
///
/// The registry is always generated and contains topic and action message types. Service request
/// and response dispatch are not included. Its descriptor slice is sorted by canonical ROS1 type
/// name, providing deterministic iteration and allocation-free binary-search lookup.
#[derive(Clone, Copy, Debug)]
pub struct MessageRegistry {
    descriptors: &'static [MessageDescriptor],
}

impl MessageRegistry {
    #[doc(hidden)]
    pub const fn new(descriptors: &'static [MessageDescriptor]) -> Self {
        Self { descriptors }
    }

    pub fn descriptors(&self) -> &'static [MessageDescriptor] {
        self.descriptors
    }

    /// Look up a message using a ROS1 name, ROS2 interface name, or ROS2 DDS name.
    pub fn get(&self, type_name: &str) -> Option<&'static MessageDescriptor> {
        let normalized = normalize_ros_type_name(type_name);
        self.descriptors
            .binary_search_by_key(&normalized.as_ref(), |descriptor| descriptor.ros_type_name)
            .ok()
            .map(|index| &self.descriptors[index])
    }

    pub fn from_value(
        &self,
        type_name: &str,
        value: DynamicValue,
    ) -> DynamicMessageResult<DynamicMessage> {
        self.get(type_name)
            .ok_or_else(|| DynamicMessageError::UnknownType(type_name.to_owned()))?
            .message(value)
    }

    /// Serialize an arbitrary Serde value and validate it as the runtime-selected message type.
    pub fn message_from<T>(
        &self,
        type_name: &str,
        value: &T,
    ) -> DynamicMessageResult<DynamicMessage>
    where
        T: serde::Serialize + ?Sized,
    {
        self.get(type_name)
            .ok_or_else(|| DynamicMessageError::UnknownType(type_name.to_owned()))?
            .message_from(value)
    }

    pub fn deserialize_with<'de, D>(
        &self,
        type_name: &str,
        deserializer: D,
    ) -> DynamicMessageResult<DynamicMessage>
    where
        D: serde::Deserializer<'de>,
    {
        self.get(type_name)
            .ok_or_else(|| DynamicMessageError::UnknownType(type_name.to_owned()))?
            .deserialize_with(deserializer)
    }
}

/// Normalize common ROS message and service type spellings to `package/Type`.
pub fn normalize_ros_type_name(type_name: &str) -> Cow<'_, str> {
    if let Some((package, rest)) = type_name.split_once("::") {
        if let Some(name) = rest
            .strip_prefix("msg::dds_::")
            .or_else(|| rest.strip_prefix("srv::dds_::"))
            .and_then(|name| name.strip_suffix('_'))
        {
            return Cow::Owned(format!("{package}/{name}"));
        }
    }

    let mut parts = type_name.split('/');
    match (parts.next(), parts.next(), parts.next(), parts.next()) {
        (Some(package), Some("msg" | "srv"), Some(name), None) => {
            Cow::Owned(format!("{package}/{name}"))
        }
        _ => Cow::Borrowed(type_name),
    }
}

/// Transport-neutral helpers used by generated registry callbacks.
#[doc(hidden)]
pub mod support {
    use super::{DynamicMessageError, DynamicMessageResult, DynamicValue};
    use crate::RosMessageType;

    pub fn to_value<T: RosMessageType>(message: T) -> DynamicMessageResult<DynamicValue> {
        serde::Serialize::serialize(&message, super::value_serde::Serializer).map_err(|error| {
            DynamicMessageError::InvalidValue {
                type_name: T::ROS_TYPE_NAME,
                message: error.to_string(),
            }
        })
    }

    pub fn from_value<T: RosMessageType>(value: DynamicValue) -> DynamicMessageResult<T> {
        serde::Deserialize::deserialize(value).map_err(|error: super::value_serde::Error| {
            DynamicMessageError::InvalidValue {
                type_name: T::ROS_TYPE_NAME,
                message: error.to_string(),
            }
        })
    }

    pub fn normalize<T: RosMessageType>(
        value: &DynamicValue,
    ) -> DynamicMessageResult<DynamicValue> {
        from_value::<T>(value.clone()).and_then(to_value)
    }

    pub fn serialize<T: RosMessageType>(
        value: &DynamicValue,
        serializer: &mut dyn erased_serde::Serializer,
    ) -> DynamicMessageResult<()> {
        let message: T = from_value(value.clone())?;
        erased_serde::Serialize::erased_serialize(&message, serializer).map_err(|error| {
            DynamicMessageError::Serialize {
                type_name: T::ROS_TYPE_NAME,
                message: error.to_string(),
            }
        })
    }

    pub fn deserialize<'de, T: RosMessageType>(
        deserializer: &mut dyn erased_serde::Deserializer<'de>,
    ) -> DynamicMessageResult<DynamicValue> {
        let message: T = erased_serde::deserialize(deserializer).map_err(|error| {
            DynamicMessageError::Deserialize {
                type_name: T::ROS_TYPE_NAME,
                message: error.to_string(),
            }
        })?;
        to_value(message)
    }
}

#[cfg(test)]
mod tests {
    use super::normalize_ros_type_name;

    #[test]
    fn normalizes_ros2_message_names() {
        assert_eq!(
            normalize_ros_type_name("std_msgs/String"),
            "std_msgs/String"
        );
        assert_eq!(
            normalize_ros_type_name("std_msgs/msg/String"),
            "std_msgs/String"
        );
        assert_eq!(
            normalize_ros_type_name("std_msgs::msg::dds_::String_"),
            "std_msgs/String"
        );
        assert_eq!(
            normalize_ros_type_name("std_srvs/srv/SetBool"),
            "std_srvs/SetBool"
        );
    }
}
