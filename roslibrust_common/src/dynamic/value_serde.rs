use super::{DynamicField, DynamicValue};
use serde::{
    de::{self, DeserializeSeed, MapAccess, SeqAccess, Visitor},
    forward_to_deserialize_any,
    ser::{
        self, Impossible, SerializeMap, SerializeSeq, SerializeStruct, SerializeTuple,
        SerializeTupleStruct,
    },
    Serialize,
};
use std::fmt;

#[derive(Debug)]
pub struct Error(String);

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl ser::Error for Error {
    fn custom<T: fmt::Display>(message: T) -> Self {
        Self(message.to_string())
    }
}

impl de::Error for Error {
    fn custom<T: fmt::Display>(message: T) -> Self {
        Self(message.to_string())
    }
}

pub struct Serializer;

impl serde::Serializer for Serializer {
    type Ok = DynamicValue;
    type Error = Error;
    type SerializeSeq = SequenceSerializer;
    type SerializeTuple = SequenceSerializer;
    type SerializeTupleStruct = SequenceSerializer;
    type SerializeTupleVariant = Impossible<DynamicValue, Error>;
    type SerializeMap = MessageMapSerializer;
    type SerializeStruct = StructSerializer;
    type SerializeStructVariant = Impossible<DynamicValue, Error>;

    fn serialize_bool(self, value: bool) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Bool(value))
    }

    fn serialize_i8(self, value: i8) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::I8(value))
    }

    fn serialize_i16(self, value: i16) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::I16(value))
    }

    fn serialize_i32(self, value: i32) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::I32(value))
    }

    fn serialize_i64(self, value: i64) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::I64(value))
    }

    fn serialize_i128(self, value: i128) -> Result<Self::Ok, Self::Error> {
        i64::try_from(value)
            .map(DynamicValue::I64)
            .map_err(|_| ser::Error::custom("i128 value does not fit the dynamic representation"))
    }

    fn serialize_u8(self, value: u8) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::U8(value))
    }

    fn serialize_u16(self, value: u16) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::U16(value))
    }

    fn serialize_u32(self, value: u32) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::U32(value))
    }

    fn serialize_u64(self, value: u64) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::U64(value))
    }

    fn serialize_u128(self, value: u128) -> Result<Self::Ok, Self::Error> {
        u64::try_from(value)
            .map(DynamicValue::U64)
            .map_err(|_| ser::Error::custom("u128 value does not fit the dynamic representation"))
    }

    fn serialize_f32(self, value: f32) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::F32(value))
    }

    fn serialize_f64(self, value: f64) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::F64(value))
    }

    fn serialize_char(self, value: char) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Char(value))
    }

    fn serialize_str(self, value: &str) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::String(value.to_owned()))
    }

    fn serialize_bytes(self, value: &[u8]) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Bytes(value.to_owned()))
    }

    fn serialize_none(self) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Unit)
    }

    fn serialize_some<T: ?Sized + Serialize>(self, value: &T) -> Result<Self::Ok, Self::Error> {
        value.serialize(self)
    }

    fn serialize_unit(self) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Unit)
    }

    fn serialize_unit_struct(self, _name: &'static str) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Unit)
    }

    fn serialize_unit_variant(
        self,
        _name: &'static str,
        _variant_index: u32,
        variant: &'static str,
    ) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::String(variant.to_owned()))
    }

    fn serialize_newtype_struct<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        value: &T,
    ) -> Result<Self::Ok, Self::Error> {
        value.serialize(self)
    }

    fn serialize_newtype_variant<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        _variant_index: u32,
        _variant: &'static str,
        _value: &T,
    ) -> Result<Self::Ok, Self::Error> {
        Err(ser::Error::custom(
            "enum variants are not ROS message values",
        ))
    }

    fn serialize_seq(self, len: Option<usize>) -> Result<Self::SerializeSeq, Self::Error> {
        Ok(SequenceSerializer::new(len))
    }

    fn serialize_tuple(self, len: usize) -> Result<Self::SerializeTuple, Self::Error> {
        Ok(SequenceSerializer::new(Some(len)))
    }

    fn serialize_tuple_struct(
        self,
        _name: &'static str,
        len: usize,
    ) -> Result<Self::SerializeTupleStruct, Self::Error> {
        Ok(SequenceSerializer::new(Some(len)))
    }

    fn serialize_tuple_variant(
        self,
        _name: &'static str,
        _variant_index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleVariant, Self::Error> {
        Err(ser::Error::custom(
            "enum variants are not ROS message values",
        ))
    }

    fn serialize_map(self, len: Option<usize>) -> Result<Self::SerializeMap, Self::Error> {
        Ok(MessageMapSerializer {
            fields: Vec::with_capacity(len.unwrap_or(0)),
            next_key: None,
        })
    }

    fn serialize_struct(
        self,
        _name: &'static str,
        len: usize,
    ) -> Result<Self::SerializeStruct, Self::Error> {
        Ok(StructSerializer {
            fields: Vec::with_capacity(len),
        })
    }

    fn serialize_struct_variant(
        self,
        _name: &'static str,
        _variant_index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeStructVariant, Self::Error> {
        Err(ser::Error::custom(
            "enum variants are not ROS message values",
        ))
    }

    fn collect_str<T: ?Sized + fmt::Display>(self, value: &T) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::String(value.to_string()))
    }

    fn is_human_readable(&self) -> bool {
        false
    }
}

/// Accept string-keyed maps as the externally tagged representation of a ROS message.
///
/// `DynamicValue` deliberately has no general-purpose map variant: ROS messages are structs with
/// named fields. Supporting maps only at this serialization boundary lets values such as
/// `serde_json::Value` feed `MessageDescriptor::message_from` without weakening the validated
/// representation.
pub struct MessageMapSerializer {
    fields: Vec<DynamicField>,
    next_key: Option<String>,
}

impl SerializeMap for MessageMapSerializer {
    type Ok = DynamicValue;
    type Error = Error;

    fn serialize_key<T: ?Sized + Serialize>(&mut self, key: &T) -> Result<(), Self::Error> {
        if self.next_key.is_some() {
            return Err(ser::Error::custom("map value is missing"));
        }
        self.next_key = match key.serialize(Serializer)? {
            DynamicValue::String(key) => Some(key),
            _ => {
                return Err(ser::Error::custom(
                    "ROS message field names must be strings",
                ))
            }
        };
        Ok(())
    }

    fn serialize_value<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        let name = self
            .next_key
            .take()
            .ok_or_else(|| ser::Error::custom("map value serialized before its key"))?;
        self.fields.push(DynamicField {
            name,
            value: value.serialize(Serializer)?,
        });
        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        if self.next_key.is_some() {
            return Err(ser::Error::custom("map value is missing"));
        }
        Ok(DynamicValue::Message(self.fields))
    }
}

pub struct SequenceSerializer {
    values: Vec<DynamicValue>,
}

impl SequenceSerializer {
    fn new(len: Option<usize>) -> Self {
        Self {
            values: Vec::with_capacity(len.unwrap_or(0)),
        }
    }

    fn push<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Error> {
        self.values.push(value.serialize(Serializer)?);
        Ok(())
    }
}

impl SerializeSeq for SequenceSerializer {
    type Ok = DynamicValue;
    type Error = Error;

    fn serialize_element<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        self.push(value)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Sequence(self.values))
    }
}

impl SerializeTuple for SequenceSerializer {
    type Ok = DynamicValue;
    type Error = Error;

    fn serialize_element<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        self.push(value)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Sequence(self.values))
    }
}

impl SerializeTupleStruct for SequenceSerializer {
    type Ok = DynamicValue;
    type Error = Error;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        self.push(value)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Sequence(self.values))
    }
}

pub struct StructSerializer {
    fields: Vec<DynamicField>,
}

impl SerializeStruct for StructSerializer {
    type Ok = DynamicValue;
    type Error = Error;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), Self::Error> {
        self.fields.push(DynamicField {
            name: key.to_owned(),
            value: value.serialize(Serializer)?,
        });
        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(DynamicValue::Message(self.fields))
    }
}

impl<'de> serde::Deserializer<'de> for DynamicValue {
    type Error = Error;

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self {
            Self::Unit => visitor.visit_unit(),
            Self::Bool(value) => visitor.visit_bool(value),
            Self::I8(value) => visitor.visit_i8(value),
            Self::I16(value) => visitor.visit_i16(value),
            Self::I32(value) => visitor.visit_i32(value),
            Self::I64(value) => visitor.visit_i64(value),
            Self::U8(value) => visitor.visit_u8(value),
            Self::U16(value) => visitor.visit_u16(value),
            Self::U32(value) => visitor.visit_u32(value),
            Self::U64(value) => visitor.visit_u64(value),
            Self::F32(value) => visitor.visit_f32(value),
            Self::F64(value) => visitor.visit_f64(value),
            Self::Char(value) => visitor.visit_char(value),
            Self::String(value) => visitor.visit_string(value),
            Self::Bytes(value) => visitor.visit_byte_buf(value),
            Self::Sequence(values) => visitor.visit_seq(ValueSeqAccess {
                values: values.into_iter(),
            }),
            Self::Message(fields) => visitor.visit_map(ValueMapAccess {
                entries: fields
                    .into_iter()
                    .map(|field| (DynamicValue::String(field.name), field.value))
                    .collect::<Vec<_>>()
                    .into_iter(),
                value: None,
            }),
        }
    }

    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self {
            Self::Unit => visitor.visit_none(),
            value => visitor.visit_some(value),
        }
    }

    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        visitor.visit_newtype_struct(self)
    }

    fn deserialize_enum<V: Visitor<'de>>(
        self,
        _name: &'static str,
        _variants: &'static [&'static str],
        _visitor: V,
    ) -> Result<V::Value, Self::Error> {
        Err(de::Error::custom(
            "enum variants are not ROS message values",
        ))
    }

    fn deserialize_ignored_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        visitor.visit_unit()
    }

    fn is_human_readable(&self) -> bool {
        false
    }

    forward_to_deserialize_any! {
        bool i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 f32 f64 char str string
        bytes byte_buf unit unit_struct seq tuple tuple_struct map struct identifier
    }
}

struct ValueSeqAccess {
    values: std::vec::IntoIter<DynamicValue>,
}

impl<'de> SeqAccess<'de> for ValueSeqAccess {
    type Error = Error;

    fn next_element_seed<T: DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, Self::Error> {
        self.values
            .next()
            .map(|value| seed.deserialize(value))
            .transpose()
    }

    fn size_hint(&self) -> Option<usize> {
        Some(self.values.len())
    }
}

struct ValueMapAccess {
    entries: std::vec::IntoIter<(DynamicValue, DynamicValue)>,
    value: Option<DynamicValue>,
}

impl<'de> MapAccess<'de> for ValueMapAccess {
    type Error = Error;

    fn next_key_seed<K: DeserializeSeed<'de>>(
        &mut self,
        seed: K,
    ) -> Result<Option<K::Value>, Self::Error> {
        match self.entries.next() {
            Some((key, value)) => {
                self.value = Some(value);
                seed.deserialize(key).map(Some)
            }
            None => Ok(None),
        }
    }

    fn next_value_seed<V: DeserializeSeed<'de>>(
        &mut self,
        seed: V,
    ) -> Result<V::Value, Self::Error> {
        seed.deserialize(
            self.value
                .take()
                .ok_or_else(|| de::Error::custom("map value requested without a key"))?,
        )
    }

    fn size_hint(&self) -> Option<usize> {
        Some(self.entries.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    #[derive(Debug, PartialEq, serde::Deserialize, serde::Serialize)]
    struct Example {
        signed: i16,
        float: f64,
        bytes: serde_bytes::ByteBuf,
    }

    #[test]
    fn preserves_ros_relevant_serde_types() {
        let example = Example {
            signed: -42,
            float: f64::NAN,
            bytes: vec![1, 2, 3].into(),
        };
        let value = example.serialize(Serializer).unwrap();
        let DynamicValue::Message(fields) = &value else {
            panic!("expected message")
        };
        assert_eq!(fields[0].value, DynamicValue::I16(-42));
        assert!(matches!(fields[1].value, DynamicValue::F64(value) if value.is_nan()));
        assert_eq!(fields[2].value, DynamicValue::Bytes(vec![1, 2, 3]));

        let decoded = Example::deserialize(value).unwrap();
        assert_eq!(decoded.signed, -42);
        assert!(decoded.float.is_nan());
        assert_eq!(decoded.bytes, vec![1, 2, 3]);
    }
}
