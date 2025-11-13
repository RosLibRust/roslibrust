//! Module for calculating the ROS2 hash of a message definition via https://github.com/ros-infrastructure/rep/pull/381/files

use anyhow::bail;
use log::{debug, error, trace};
use serde::Serialize;
use serde_json::ser::Formatter;
use std::{
    collections::{BTreeMap, HashMap},
    io::Write,
};

use crate::{
    parse::{ParsedMessageFile, ParsedServiceFile},
    utils::RosVersion,
    ArrayType, FieldInfo, MessageFile, Ros2Hash,
};

/// The following structs define the format of the JSON file used for hashing in ROS2
/// A quick description of the process to calculate a ROS2 hash:
///  - ROS2 parses the .msg / .srv files
///  - Covert them to the below struct formats
///  - Generate a specifically formatted JSON string of the below structs
///  - Calculate the sha256 hash of the JSON string (utf-8)
///  - Generate a string of the format RIHS01_<hex hash>
///
#[allow(unused)]
#[derive(serde::Deserialize, serde::Serialize)]
pub struct TypeDescriptionFile {
    type_description_msg: TypeDescriptionMsg,
    type_hashes: Vec<TypeHash>,
}

/// Sub-component of the ROS2 JSON file format for hashing
/// Should not be used for other purposes
#[derive(serde::Deserialize, serde::Serialize)]
pub struct TypeDescriptionMsg {
    type_description: TypeDescription,
    referenced_type_descriptions: Vec<TypeDescription>,
}

/// Sub-component of the ROS2 JSON file format for hasing
/// Should not be used for other purposes
#[derive(serde::Deserialize, serde::Serialize)]
pub struct TypeDescription {
    type_name: String,
    fields: Vec<Field>,
}

/// Sub-component of the ROS2 JSON file format for hashing
/// Should not be used for other purposes
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Field {
    name: String,
    #[serde(rename = "type")]
    field_type: FieldType,
    // Value is not used in hashing, included for completeness
    #[serde(skip)]
    _default_value: String,
}

/// Sub-component of the ROS2 JSON file format for hashing
/// Should not be used for other purposes
#[derive(serde::Deserialize, serde::Serialize)]
pub struct FieldType {
    type_id: u8,
    capacity: u32,
    string_capacity: u32,
    nested_type_name: String,
}

/// Sub-component of the ROS2 JSON file format for hashing
/// Should not be used for other purposes
#[allow(unused)]
#[derive(serde::Deserialize, serde::Serialize)]
pub struct TypeHash {
    type_name: String,
    hash_string: String,
}

/// Calculates the ROS2 hash of a message definition, if possible
/// Returns None if the message definition is not hashable (e.g. contains nested types not present in the graph)
pub fn calculate_ros2_hash(
    parsed: &ParsedMessageFile,
    graph: &BTreeMap<String, MessageFile>,
) -> Ros2Hash {
    let type_description_msg = convert_to_type_description(parsed, graph, false)
        .expect("Failed to convert file to a valid ROS2 type description");
    calculate_hash(&type_description_msg)
}

pub fn calculate_ros2_srv_hash(
    parsed: &ParsedServiceFile,
    graph: &BTreeMap<String, MessageFile>,
) -> Ros2Hash {
    // Lot's of incredibly messy naming here due to mix of ROS1 and ROS2 naming style

    // Literal type that will appear in fields
    let request_type = parsed.name.clone() + "_Request";
    let response_type = parsed.name.clone() + "_Response";
    let event_type = parsed.name.clone() + "_Event";

    // Does not include /srv/, is what will be used to lookup type in graph
    let request_ros1_name = parsed.get_full_name() + "_Request";
    let response_ros1_name = parsed.get_full_name() + "_Response";
    let event_ros1_name = parsed.get_full_name() + "_Event";

    // ROS2 has this magical 3rd type the "Event Type"
    // Eventually I think we'll need to generate Rust structs for this type as well...
    // For now just solving this for hashing.
    let event = ParsedMessageFile {
        name: event_type.clone(),
        package: parsed.package.clone(),
        constants: vec![],
        version: Some(RosVersion::ROS2),
        source: "".to_string(),
        path: parsed.path.clone(),
        fields: vec![
            // Every event has an info field with same type
            FieldInfo {
                field_name: "info".to_string(),
                field_type: crate::FieldType {
                    package_name: Some("service_msgs".to_string()),
                    source_package: "service_msgs".to_string(),
                    field_type: "ServiceEventInfo".to_string(),
                    array_info: ArrayType::NotArray,
                    string_capacity: None,
                },
                default: None,
            },
            // Every event has a request field with the request type
            FieldInfo {
                field_name: "request".to_string(),
                field_type: crate::FieldType {
                    package_name: Some(parsed.package.clone()),
                    source_package: parsed.package.clone(),
                    field_type: request_type.clone(),
                    // This field is a bounded list of 1 to represent Optional in a hacky way
                    array_info: ArrayType::Bounded(1),
                    string_capacity: None,
                },
                default: None,
            },
            // Every event has a response field with the response type
            FieldInfo {
                field_name: "response".to_string(),
                field_type: crate::FieldType {
                    package_name: Some(parsed.package.clone()),
                    source_package: parsed.package.clone(),
                    // This field is a bounded list of 1 to represent Optional in a hacky way
                    field_type: response_type.clone(),
                    array_info: ArrayType::Bounded(1),
                    string_capacity: None,
                },
                default: None,
            },
        ],
    };

    let total_message_file = ParsedMessageFile {
        name: parsed.name.clone(),
        package: parsed.package.clone(),
        constants: vec![],
        version: Some(RosVersion::ROS2),
        source: "".to_string(),
        path: parsed.path.clone(),
        fields: vec![
            FieldInfo {
                field_name: "request_message".to_string(),
                field_type: crate::FieldType {
                    package_name: Some(parsed.package.clone()),
                    source_package: parsed.package.clone(),
                    field_type: request_type.clone(),
                    array_info: ArrayType::NotArray,
                    string_capacity: None,
                },
                default: None,
            },
            FieldInfo {
                field_name: "response_message".to_string(),
                field_type: crate::FieldType {
                    package_name: Some(parsed.package.clone()),
                    source_package: parsed.package.clone(),
                    field_type: response_type.clone(),
                    array_info: ArrayType::NotArray,
                    string_capacity: None,
                },
                default: None,
            },
            FieldInfo {
                field_name: "event_message".to_string(),
                field_type: crate::FieldType {
                    package_name: Some(parsed.package.clone()),
                    source_package: parsed.package.clone(),
                    field_type: event_type,
                    array_info: ArrayType::NotArray,
                    string_capacity: None,
                },
                default: None,
            },
        ],
    };

    let mut graph_copy = graph.clone();
    // Create some "bonus" entries in the message graph for the "virtual" types within the service
    // Have to modify naming in these slightly (god this is a pile of hacks I'm sorry)
    let mut request = parsed.request_type.clone();
    request.name = request_type;

    let mut response = parsed.response_type.clone();
    response.name = response_type;

    graph_copy.insert(
        event_ros1_name,
        MessageFile {
            parsed: event,
            // Dummy values
            ros2_hash: Default::default(),
            md5sum: "".to_string(),
            definition: "".to_string(),
            is_fixed_encoding_length: true,
        },
    );
    graph_copy.insert(
        request_ros1_name,
        MessageFile {
            parsed: request,
            // Dummy values
            ros2_hash: Default::default(),
            md5sum: "".to_string(),
            definition: "".to_string(),
            is_fixed_encoding_length: true,
        },
    );
    graph_copy.insert(
        response_ros1_name,
        MessageFile {
            parsed: response,
            // Dummy values
            ros2_hash: Default::default(),
            md5sum: "".to_string(),
            definition: "".to_string(),
            is_fixed_encoding_length: true,
        },
    );

    let total_type_description =
        convert_to_type_description(&total_message_file, &graph_copy, true)
            .expect("Failed to convert service file to a valid ROS2 type description");
    calculate_hash(&total_type_description)
}

/// Calculates the hash from a TypeDescriptionMsg struct
fn calculate_hash(type_description_msg: &TypeDescriptionMsg) -> Ros2Hash {
    let type_description_string = to_ros2_json(type_description_msg);
    trace!("Generate type description string: {type_description_string}");
    use sha2::Digest;
    let mut hasher = sha2::Sha256::new();
    hasher.update(type_description_string);
    let result: [u8; 32] = hasher.finalize().into();
    Ros2Hash(result)
}

/// Taking in a parsed representation of a message convert it to the ROS2 specific type representation
fn convert_to_type_description(
    parsed: &ParsedMessageFile,
    graph: &BTreeMap<String, MessageFile>,
    // This option is a real hack, but is working around ROS2 vs. ROS1 naming differences
    service_naming: bool,
) -> Result<TypeDescriptionMsg, anyhow::Error> {
    let mut fields = vec![];
    let mut referenced_type_descriptions = BTreeMap::new();

    // ROS2 treats empty message types as having a single uint8 field named "structure_needs_at_least_one_member"
    if parsed.fields.is_empty() {
        fields.push(Field {
            name: "structure_needs_at_least_one_member".to_string(),
            field_type: FieldType {
                type_id: 3,
                capacity: 0,
                string_capacity: 0,
                nested_type_name: "".to_string(),
            },
            _default_value: "".to_string(),
        });
    }
    for field in &parsed.fields {
        let nested_type_name = if field.field_type.is_primitive() {
            "".to_string()
        } else {
            if service_naming {
                const SPECIAL_SUFFIX: &[&str] = &["_Request", "_Response", "_Event"];
                if SPECIAL_SUFFIX
                    .iter()
                    .any(|s| field.field_type.field_type.ends_with(s))
                {
                    format!(
                        "{}/srv/{}",
                        field.field_type.source_package, field.field_type.field_type
                    )
                } else {
                    field.get_ros2_full_type_name()
                }
            } else {
                field.get_ros2_full_type_name()
            }
        };

        // Check if there is a length limit on array contents
        let capacity = match field.field_type.array_info {
            ArrayType::Bounded(size) => size as u32,
            ArrayType::FixedLength(size) => size as u32,
            ArrayType::Unbounded => 0,
            ArrayType::NotArray => 0,
        };

        fields.push(Field {
            name: field.field_name.clone(),
            field_type: FieldType {
                type_id: get_field_type_id(&field.field_type)?,
                capacity,
                string_capacity: field.field_type.string_capacity.unwrap_or(0) as u32,
                nested_type_name,
            },
            // Default value is not used in hashing, but included for completeness
            _default_value: "".to_string(),
        });

        if !field.field_type.is_primitive() {
            let sub_message =
                graph
                    .get(field.get_full_type_name().as_str())
                    .ok_or(anyhow::anyhow!(
                        "Failed to find definition for nested type: {} while hashing {} for ROS2",
                        field.get_full_type_name(),
                        parsed.get_full_name()
                    ))?;
            let sub_type_description =
                convert_to_type_description(&sub_message.parsed, graph, true)?;
            for sub_referenced_type in sub_type_description.referenced_type_descriptions {
                referenced_type_descriptions
                    .insert(sub_referenced_type.type_name.clone(), sub_referenced_type);
            }
            referenced_type_descriptions.insert(
                sub_type_description.type_description.type_name.clone(),
                sub_type_description.type_description,
            );
        }
    }
    let referenced_type_descriptions = referenced_type_descriptions
        .into_iter()
        .map(|(_, v)| v)
        .collect::<Vec<_>>();

    Ok(TypeDescriptionMsg {
        type_description: TypeDescription {
            type_name: parsed.get_ros2_full_name(),
            fields,
        },
        referenced_type_descriptions,
    })
}

/// ROS2 requires a very specific JSON format for hashing:
/// - Single line
/// - Spaces after every control : and ,
/// - No other whitespace
pub fn to_ros2_json<T: Serialize>(v: T) -> String {
    let mut buf = Vec::new();
    {
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, Ros2Formatter::default());
        v.serialize(&mut ser).unwrap();
    }
    String::from_utf8(buf).unwrap()
}

#[derive(Default)]
struct Ros2Formatter;

impl Formatter for Ros2Formatter {
    fn begin_object_value<W: ?Sized + Write>(&mut self, writer: &mut W) -> std::io::Result<()> {
        writer.write_all(b": ")
    }

    fn begin_array_value<W>(&mut self, writer: &mut W, first: bool) -> std::io::Result<()>
    where
        W: ?Sized + Write,
    {
        if first {
            Ok(())
        } else {
            writer.write_all(b", ")
        }
    }

    fn begin_object_key<W>(&mut self, writer: &mut W, first: bool) -> std::io::Result<()>
    where
        W: ?Sized + Write,
    {
        if first {
            Ok(())
        } else {
            writer.write_all(b", ")
        }
    }
}

// Directly taken from:
// https://github.com/ros2/rosidl/blob/rolling/rosidl_generator_type_description/rosidl_generator_type_description/__init__.py
lazy_static::lazy_static! {
    static ref FIELD_TYPE_NAME_TO_ID: HashMap<String, u8> = {
        let mut map = HashMap::new();
        map.insert("FIELD_TYPE_NOT_SET".to_string(), 0);

        // Nested type defined in other .msg/.idl files.
        map.insert("FIELD_TYPE_NESTED_TYPE".to_string(), 1);

        // Basic Types
        // Integer Types
        map.insert("FIELD_TYPE_INT8".to_string(), 2);
        map.insert("FIELD_TYPE_UINT8".to_string(), 3);
        map.insert("FIELD_TYPE_INT16".to_string(), 4);
        map.insert("FIELD_TYPE_UINT16".to_string(), 5);
        map.insert("FIELD_TYPE_INT32".to_string(), 6);
        map.insert("FIELD_TYPE_UINT32".to_string(), 7);
        map.insert("FIELD_TYPE_INT64".to_string(), 8);
        map.insert("FIELD_TYPE_UINT64".to_string(), 9);

        // Floating-Point Types
        map.insert("FIELD_TYPE_FLOAT".to_string(), 10);
        map.insert("FIELD_TYPE_DOUBLE".to_string(), 11);
        map.insert("FIELD_TYPE_LONG_DOUBLE".to_string(), 12);

        // Char and WChar Types
        map.insert("FIELD_TYPE_CHAR".to_string(), 13);
        map.insert("FIELD_TYPE_WCHAR".to_string(), 14);

        // Boolean Type
        map.insert("FIELD_TYPE_BOOLEAN".to_string(), 15);

        // Byte/Octet Type
        map.insert("FIELD_TYPE_BYTE".to_string(), 16);

        // String Types
        map.insert("FIELD_TYPE_STRING".to_string(), 17);
        map.insert("FIELD_TYPE_WSTRING".to_string(), 18);

        // Fixed String Types
        // NOTE: - Carter AFAIK ros2 doesn't support fixed length strings, so these types are probably unused
        map.insert("FIELD_TYPE_FIXED_STRING".to_string(), 19);
        map.insert("FIELD_TYPE_FIXED_WSTRING".to_string(), 20);

        // Bounded String Types
        map.insert("FIELD_TYPE_BOUNDED_STRING".to_string(), 21);
        map.insert("FIELD_TYPE_BOUNDED_WSTRING".to_string(), 22);

        // Fixed Sized Array Types
        map.insert("FIELD_TYPE_NESTED_TYPE_ARRAY".to_string(), 49);
        map.insert("FIELD_TYPE_INT8_ARRAY".to_string(), 50);
        map.insert("FIELD_TYPE_UINT8_ARRAY".to_string(), 51);
        map.insert("FIELD_TYPE_INT16_ARRAY".to_string(), 52);
        map.insert("FIELD_TYPE_UINT16_ARRAY".to_string(), 53);
        map.insert("FIELD_TYPE_INT32_ARRAY".to_string(), 54);
        map.insert("FIELD_TYPE_UINT32_ARRAY".to_string(), 55);
        map.insert("FIELD_TYPE_INT64_ARRAY".to_string(), 56);
        map.insert("FIELD_TYPE_UINT64_ARRAY".to_string(), 57);
        map.insert("FIELD_TYPE_FLOAT_ARRAY".to_string(), 58);
        map.insert("FIELD_TYPE_DOUBLE_ARRAY".to_string(), 59);
        map.insert("FIELD_TYPE_LONG_DOUBLE_ARRAY".to_string(), 60);
        map.insert("FIELD_TYPE_CHAR_ARRAY".to_string(), 61);
        map.insert("FIELD_TYPE_WCHAR_ARRAY".to_string(), 62);
        map.insert("FIELD_TYPE_BOOLEAN_ARRAY".to_string(), 63);
        map.insert("FIELD_TYPE_BYTE_ARRAY".to_string(), 64);

        map.insert("FIELD_TYPE_STRING_ARRAY".to_string(), 65);
        map.insert("FIELD_TYPE_WSTRING_ARRAY".to_string(), 66);
        // NOTE: - Carter AFAIK ros2 doesn't support fixed length strings, so these types are probably unused
        map.insert("FIELD_TYPE_FIXED_STRING_ARRAY".to_string(), 67);
        map.insert("FIELD_TYPE_FIXED_WSTRING_ARRAY".to_string(), 68);

        map.insert("FIELD_TYPE_BOUNDED_STRING_ARRAY".to_string(), 69);
        map.insert("FIELD_TYPE_BOUNDED_WSTRING_ARRAY".to_string(), 70);

        // Bounded Sequence Types
        map.insert("FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE".to_string(), 97);
        map.insert("FIELD_TYPE_INT8_BOUNDED_SEQUENCE".to_string(), 98);
        map.insert("FIELD_TYPE_UINT8_BOUNDED_SEQUENCE".to_string(), 99);
        map.insert("FIELD_TYPE_INT16_BOUNDED_SEQUENCE".to_string(), 100);
        map.insert("FIELD_TYPE_UINT16_BOUNDED_SEQUENCE".to_string(), 101);
        map.insert("FIELD_TYPE_INT32_BOUNDED_SEQUENCE".to_string(), 102);
        map.insert("FIELD_TYPE_UINT32_BOUNDED_SEQUENCE".to_string(), 103);
        map.insert("FIELD_TYPE_INT64_BOUNDED_SEQUENCE".to_string(), 104);
        map.insert("FIELD_TYPE_UINT64_BOUNDED_SEQUENCE".to_string(), 105);
        map.insert("FIELD_TYPE_FLOAT_BOUNDED_SEQUENCE".to_string(), 106);
        map.insert("FIELD_TYPE_DOUBLE_BOUNDED_SEQUENCE".to_string(), 107);
        map.insert("FIELD_TYPE_LONG_DOUBLE_BOUNDED_SEQUENCE".to_string(), 108);
        map.insert("FIELD_TYPE_CHAR_BOUNDED_SEQUENCE".to_string(), 109);
        map.insert("FIELD_TYPE_WCHAR_BOUNDED_SEQUENCE".to_string(), 110);
        map.insert("FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE".to_string(), 111);
        map.insert("FIELD_TYPE_BYTE_BOUNDED_SEQUENCE".to_string(), 112);

        map.insert("FIELD_TYPE_STRING_BOUNDED_SEQUENCE".to_string(), 113);
        map.insert("FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE".to_string(), 114);
        // NOTE: - Carter AFAIK ros2 doesn't support fixed length strings, so these types are probably unused
        map.insert("FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE".to_string(), 115);
        map.insert("FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE".to_string(), 116);

        map.insert("FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE".to_string(), 117);
        map.insert("FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE".to_string(), 118);

        // Unbounded Sequence Types
        map.insert("FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE".to_string(), 145);
        map.insert("FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE".to_string(), 146);
        map.insert("FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE".to_string(), 147);
        map.insert("FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE".to_string(), 148);
        map.insert("FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE".to_string(), 149);
        map.insert("FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE".to_string(), 150);
        map.insert("FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE".to_string(), 151);
        map.insert("FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE".to_string(), 152);
        map.insert("FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE".to_string(), 153);
        map.insert("FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE".to_string(), 154);
        map.insert("FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE".to_string(), 155);
        map.insert("FIELD_TYPE_LONG_DOUBLE_UNBOUNDED_SEQUENCE".to_string(), 156);
        map.insert("FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE".to_string(), 157);
        map.insert("FIELD_TYPE_WCHAR_UNBOUNDED_SEQUENCE".to_string(), 158);
        map.insert("FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE".to_string(), 159);
        map.insert("FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE".to_string(), 160);
        map.insert("FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE".to_string(), 161);
        map.insert("FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE".to_string(), 162);

        // NOTE: - Carter AFAIK ros2 doesn't support fixed length strings, so these types are probably unused
        map.insert("FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE".to_string(), 163);
        map.insert("FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE".to_string(), 164);
        map.insert("FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE".to_string(), 165);
        map.insert("FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE".to_string(), 166);

        map
    };

        static ref FIELD_VALUE_TYPE_NAMES: HashMap<&'static str, &'static str> = {
        let mut map = HashMap::new();
        map.insert("nested_type", "NESTED_TYPE");
        map.insert("int8", "INT8");
        map.insert("uint8", "UINT8");
        map.insert("int16", "INT16");
        map.insert("uint16", "UINT16");
        map.insert("int32", "INT32");
        map.insert("uint32", "UINT32");
        map.insert("int64", "INT64");
        map.insert("uint64", "UINT64");
        map.insert("float32", "FLOAT");
        map.insert("float64", "DOUBLE");
        map.insert("long", "LONG_DOUBLE");
        map.insert("char", "CHAR");
        map.insert("wchar", "WCHAR");
        // WHY THE FUCK WAS THIS BOOLEAN IN ROS CODE?
        // map.insert("boolean", "BOOLEAN");
        map.insert("bool", "BOOLEAN");
        map.insert("octet", "BYTE");
        map.insert("string", "STRING");
        // TODO following likely don't work yet
        map.insert("wstring", "WSTRING");
        map.insert("bounded_string", "BOUNDED_STRING");
        map.insert("bounded_wstring", "BOUNDED_WSTRING");
        map
    };
}

/// Returns the string matching the format ROS2's python code uses for lookups
///
/// e.g. "int32[3]" -> "FIELD_TYPE_INT32_ARRAY" and "string<=10" -> "FIELD_TYPE_BOUNDED_STRING"
fn get_field_type_string(field_type: &crate::FieldType) -> String {
    // Okay I have no idea why, I have no idea how, but I find when I look at generated .json output it is doing this
    let temp_field_type = if field_type.field_type == "char" {
        "uint8"
    } else {
        &field_type.field_type
    };
    const PREFIX: &str = "FIELD_TYPE_";
    let core_name = match FIELD_VALUE_TYPE_NAMES.get(temp_field_type) {
        Some(name) => name,
        None => "NESTED_TYPE",
    };
    let string_prefix = match field_type.string_capacity {
        Some(_) => "BOUNDED_",
        None => "",
    };
    let array_suffix = match field_type.array_info {
        ArrayType::FixedLength(_) => "_ARRAY",
        ArrayType::Unbounded => "_UNBOUNDED_SEQUENCE",
        ArrayType::Bounded(_) => "_BOUNDED_SEQUENCE",
        ArrayType::NotArray => "",
    };
    format!("{PREFIX}{string_prefix}{core_name}{array_suffix}")
}

/// Converts our internal representation of a field type to the numeric ID used in the ROS2 hash format
fn get_field_type_id(field_type: &crate::FieldType) -> Result<u8, anyhow::Error> {
    let field_type_string = get_field_type_string(field_type);
    match FIELD_TYPE_NAME_TO_ID.get(&field_type_string) {
        Some(id) => Ok(*id),
        None => {
            bail!("Failed to find field type ID for {}", field_type_string);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use crate::{
        utils::{Package, RosVersion},
        ArrayType,
    };

    /// Basic demonstration of the hashing process, and that our struct formatting matches ros2
    /// Reads in a pre-generated JSON file for std_msgs/String and confirms the hash matches
    /// File was generated by running `rosidl generate std_msgs ./String.msg -I .`
    /// Turns out these json files can just be found in the share/ folder of installed ros2 packages!
    /// This test is independent of our message file parsing logic
    #[test_log::test]
    fn ros2_hashing_against_message_files() {
        let test_data = [
            (
                include_str!("../assets/String.json"),
                "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            ),
            (
                include_str!("../assets/MultiArrayLayout.json"),
                "RIHS01_4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6",
            ),
            (
                include_str!("../assets/MultiArrayDimension.json"),
                "RIHS01_5e773a60a4c7fc8a54985f307c7837aa2994252a126c301957a24e31282c9cbe",
            ),
            (
                include_str!("../assets/ColorRGBA.json"),
                "RIHS01_77a7a5b9ae477306097665106e0413ba74440245b1f3d0c6d6405fe5c7813fe8",
            ),
        ];
        for (test_file, expected_hash) in test_data {
            let parsed: super::TypeDescriptionFile = serde_json::from_str(test_file)
                .expect(format!("Failed to parse test file {test_file}").as_str());
            let hash = parsed.type_hashes[0].hash_string.clone();

            assert_eq!(hash, expected_hash,);

            let calculated_hash = super::calculate_hash(&parsed.type_description_msg);

            assert_eq!(calculated_hash.to_hash_string(), expected_hash);
        }
    }

    /// Double checking our conversion to the ROS2 integer type ids are correct
    #[test_log::test]
    fn spot_check_field_type_id() {
        let mut field = crate::FieldType {
            package_name: Some("std_msgs".to_string()),
            source_package: "std_msgs".to_string(),
            field_type: "string".to_string(),
            array_info: ArrayType::FixedLength(10),
            string_capacity: None,
        };
        assert_eq!(super::get_field_type_id(&field).unwrap(), 65);

        field.array_info = ArrayType::Unbounded;
        assert_eq!(super::get_field_type_id(&field).unwrap(), 161);

        field.field_type = "int32".to_string();
        field.array_info = ArrayType::FixedLength(10);
        assert_eq!(super::get_field_type_id(&field).unwrap(), 54);

        field.array_info = ArrayType::Unbounded;
        assert_eq!(super::get_field_type_id(&field).unwrap(), 150);

        field.array_info = ArrayType::NotArray;
        assert_eq!(super::get_field_type_id(&field).unwrap(), 6);

        field.array_info = ArrayType::Bounded(10);
        assert_eq!(super::get_field_type_id(&field).unwrap(), 102);

        field.field_type = "bool".to_string();
        field.array_info = ArrayType::NotArray;
        assert_eq!(super::get_field_type_id(&field).unwrap(), 15);
    }

    /// End-To-End test from parse -> hash
    /// Note: this test isn't super needed, and we test this more clearly in roslibrust_test package.
    /// Leaving this here, as it's slightly more specific than the end-to-end test in roslibrust_test
    #[test_log::test]
    fn full_hash_tests() {
        // We can at least do basic hashing for a String!
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let package = Package {
            name: "std_msgs".to_string(),
            path: root.join("../assets/ros2_common_interfaces/std_msgs"),
            version: Some(RosVersion::ROS2),
        };

        let (msg, _, _) = crate::parse_ros_files(vec![(
            package,
            root.join("../assets/ros2_common_interfaces/std_msgs/msg/String.msg"),
        )])
        .expect("Failed to parse test file");

        let hash = super::calculate_ros2_hash(&msg[0], &std::collections::BTreeMap::new());

        assert_eq!(
            hash.to_hash_string(),
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
        );
    }

    #[test_log::test]
    fn ros2_hashing_against_service_files() {
        let test_data = [(
            include_str!("../assets/AddTwoInts.json"),
            "RIHS01_e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a",
        )];

        for (test_file, expected_hash) in test_data {
            let parsed: super::TypeDescriptionFile = serde_json::from_str(test_file)
                .expect(format!("Failed to parse test file {test_file}").as_str());
            let hash = parsed.type_hashes[0].hash_string.clone();

            assert_eq!(hash, expected_hash,);

            let calculated_hash = super::calculate_hash(&parsed.type_description_msg);

            assert_eq!(calculated_hash.to_hash_string(), expected_hash);
        }
    }

    #[test_log::test]
    fn ros2_srv_hash_tests() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let package = Package {
            name: "ros2_test_msgs".to_string(),
            path: root.join("../assets/ros2_test_msgs"),
            version: Some(RosVersion::ROS2),
        };

        // Note: to successfully has a ROS2 service we need builtin_interfaces and service_msgs available
        let (msg, srv, _) = crate::parse_ros_files(vec![
            (
                package,
                root.join("../assets/ros2_test_msgs/srv/AddTwoInts.srv"),
            ),
            (
                Package {
                    name: "builtin_interfaces".to_string(),
                    path: root.join("../assets/ros2_required_msgs/rcl_interfaces/builtin_interfaces"),
                    version: Some(RosVersion::ROS2),
                },
                root.join("../assets/ros2_required_msgs/rcl_interfaces/builtin_interfaces/msg/Time.msg"),
            ),
            (
                Package {
                    name: "service_msgs".to_string(),
                    path: root.join("../assets/ros2_required_msgs/rcl_interfaces/service_msgs"),
                    version: Some(RosVersion::ROS2),
                },
                root.join("../assets/ros2_required_msgs/rcl_interfaces/service_msgs/msg/ServiceEventInfo.msg"),
            ),
        ])
        .expect("Failed to parse test file");

        let (resolved_msg, resolved_srv) = crate::resolve_dependency_graph(msg, srv).unwrap();
        let graph = resolved_msg
            .into_iter()
            .map(|msg| (msg.parsed.get_full_name(), msg))
            .collect::<std::collections::BTreeMap<_, _>>();

        let hash = super::calculate_ros2_srv_hash(&resolved_srv[0].parsed, &graph);

        assert_eq!(
            hash.to_hash_string(),
            "RIHS01_cbdcb755e63eba37467c9846fe9f0b458c2989832e888dfd39ecbf8991800ef7"
        );
    }
}
