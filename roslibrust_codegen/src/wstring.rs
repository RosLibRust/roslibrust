use serde::{
    de::Error as DeError, ser::SerializeSeq, Deserialize, Deserializer, Serialize, Serializer,
};
use std::{borrow::Borrow, fmt, ops::Deref, str::FromStr};

/// A ROS 2 `wstring`.
///
/// ROS 2 represents this type as a [`std::u16string`](https://en.cppreference.com/w/cpp/string/basic_string)
/// in C++ and as a DDS `wstring` on the wire. Rust strings are UTF-8, so this
/// wrapper converts to and from UTF-16 when used with a binary serializer such
/// as CDR, while remaining a normal JSON string for human-readable serializers.
#[derive(Clone, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct WString(String);

impl WString {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn into_string(self) -> String {
        self.0
    }
}

impl Serialize for WString {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if serializer.is_human_readable() {
            serializer.serialize_str(&self.0)
        } else {
            let code_units = self.0.encode_utf16();
            let mut sequence = serializer.serialize_seq(Some(code_units.clone().count()))?;
            for code_unit in code_units {
                sequence.serialize_element(&code_unit)?;
            }
            sequence.end()
        }
    }
}

impl<'de> Deserialize<'de> for WString {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        if deserializer.is_human_readable() {
            String::deserialize(deserializer).map(Self)
        } else {
            let code_units = Vec::<u16>::deserialize(deserializer)?;
            String::from_utf16(&code_units)
                .map(Self)
                .map_err(D::Error::custom)
        }
    }
}

impl AsRef<str> for WString {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

impl Borrow<str> for WString {
    fn borrow(&self) -> &str {
        self.as_str()
    }
}

impl Deref for WString {
    type Target = str;

    fn deref(&self) -> &Self::Target {
        self.as_str()
    }
}

impl fmt::Display for WString {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

impl From<String> for WString {
    fn from(value: String) -> Self {
        Self(value)
    }
}

impl From<&str> for WString {
    fn from(value: &str) -> Self {
        Self(value.to_owned())
    }
}

impl From<WString> for String {
    fn from(value: WString) -> Self {
        value.0
    }
}

impl FromStr for WString {
    type Err = std::convert::Infallible;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Ok(Self::from(value))
    }
}

impl PartialEq<str> for WString {
    fn eq(&self, other: &str) -> bool {
        self.0 == other
    }
}

impl PartialEq<&str> for WString {
    fn eq(&self, other: &&str) -> bool {
        self.0 == *other
    }
}

#[cfg(test)]
mod tests {
    use super::WString;

    #[test]
    fn json_uses_a_normal_string() {
        let value = WString::from("ハロー 🌍");
        let encoded = serde_json::to_string(&value).unwrap();
        assert_eq!(encoded, "\"ハロー 🌍\"");
        assert_eq!(serde_json::from_str::<WString>(&encoded).unwrap(), value);
    }

    #[test]
    fn cdr_uses_a_utf16_sequence_without_a_null_terminator() {
        let value = WString::from("A🌍");
        let encoded = cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).unwrap();

        assert_eq!(
            encoded,
            [
                0x00, 0x01, 0x00, 0x00, // CDR little-endian encapsulation
                0x03, 0x00, 0x00, 0x00, // three UTF-16 code units
                0x41, 0x00, // A
                0x3c, 0xd8, 0x0d, 0xdf, // U+1F30D surrogate pair
            ]
        );
        assert_eq!(cdr::deserialize::<WString>(&encoded).unwrap(), value);
    }
}
