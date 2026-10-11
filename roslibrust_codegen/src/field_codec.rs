//! Shared support for generated Serde field codecs.
//!
//! Code generation selects the wire adapter and declared bounds. This module owns the
//! validation and codec plumbing so generated message files only contain that configuration.

use std::fmt;

/// Tuple framing for fixed arrays of any length, including non-Copy elements.
pub mod fixed_array;

/// The default Serde adapter, with the same interface as the byte and fixed-array adapters.
pub mod standard {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<T: Serialize + ?Sized, S: Serializer>(
        value: &T,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        value.serialize(serializer)
    }

    pub fn deserialize<'de, T: Deserialize<'de>, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<T, D::Error> {
        T::deserialize(deserializer)
    }
}

/// A neutral error that can be converted into either a Serde serialization or decoding error.
pub struct BoundsError {
    field: &'static str,
    index: Option<usize>,
    kind: &'static str,
    limit: usize,
    actual: usize,
}

impl fmt::Display for BoundsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "field `{field}", field = self.field)?;
        if let Some(index) = self.index {
            write!(formatter, "[{index}]")?;
        }
        write!(
            formatter,
            "` exceeds maximum {} {} (got {})",
            self.kind, self.limit, self.actual
        )
    }
}

pub fn check_length(
    actual: usize,
    limit: usize,
    field: &'static str,
    index: Option<usize>,
    kind: &'static str,
) -> Result<(), BoundsError> {
    if actual > limit {
        return Err(BoundsError {
            field,
            index,
            kind,
            limit,
            actual,
        });
    }
    Ok(())
}

/// Define field codec functions from a local type alias, wire adapter, and ROS bounds.
///
/// Generated code places this macro and a `Value` type alias in a private module, and
/// uses `#[serde(with = "module_name")]` to call the codec. The alias keeps Rust types
/// in normal type syntax, letting rustfmt format every generated field consistently. Sequence
/// limits count elements; string limits count UTF-8 bytes. Validation runs before
/// serialization and after deserialization, preserving the selected adapter's framing.
/// This is generated-code support, rather than a general-purpose derive interface.
#[macro_export]
macro_rules! ros_field_codec {
    (
        field = $field:literal,
        ty = $value_type:ty,
        serde = $serde:path,
        adapter = $adapter:path,
        sequence = $sequence:tt,
        strings = $strings:ident $(($string_limit:expr))? $(,)?
    ) => {
        use $serde as __serde;
        use $adapter as __adapter;

        fn validate(value: &$value_type) -> ::std::result::Result<(), $crate::field_codec::BoundsError> {
            $crate::ros_field_codec!(@sequence value, $field, $sequence);
            $crate::ros_field_codec!(@strings value, $field, $strings $(($string_limit))?);
            ::std::result::Result::Ok(())
        }

        pub fn serialize<S: __serde::Serializer>(
            value: &$value_type,
            serializer: S,
        ) -> ::std::result::Result<S::Ok, S::Error> {
            validate(value).map_err(<S::Error as __serde::ser::Error>::custom)?;
            __adapter::serialize(value, serializer)
        }

        pub fn deserialize<'de, D: __serde::Deserializer<'de>>(
            deserializer: D,
        ) -> ::std::result::Result<$value_type, D::Error> {
            let value: $value_type = __adapter::deserialize(deserializer)?;
            validate(&value).map_err(<D::Error as __serde::de::Error>::custom)?;
            ::std::result::Result::Ok(value)
        }
    };
    (@sequence $value:ident, $field:literal, unbounded) => {};
    (@sequence $value:ident, $field:literal, $limit:literal) => {
        $crate::field_codec::check_length($value.len(), $limit, $field, ::std::option::Option::None, "sequence length")?;
    };
    (@strings $value:ident, $field:literal, unbounded) => {};
    (@strings $value:ident, $field:literal, scalar($limit:expr)) => {
        $crate::field_codec::check_length($value.len(), $limit, $field, ::std::option::Option::None, "string length in bytes")?;
    };
    (@strings $value:ident, $field:literal, sequence($limit:expr)) => {
        for (index, string) in $value.iter().enumerate() {
            $crate::field_codec::check_length(string.len(), $limit, $field, ::std::option::Option::Some(index), "string length in bytes")?;
        }
    };
}
