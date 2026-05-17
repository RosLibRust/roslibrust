use crate::utils::{Package, RosVersion};
use crate::{bail, ArrayType, Error};
use crate::{ConstantInfo, FieldInfo, FieldType, RosLiteral};
use std::collections::HashMap;

mod action;
pub use action::{parse_ros_action_file, ParsedActionFile};
mod msg;
pub use msg::{parse_ros_message_file, ParsedMessageFile};
mod srv;
pub use srv::{parse_ros_service_file, ParsedServiceFile};

// Note: time and duration are primitives in ROS1, but not used in ROS2
// List of types which are "individual data fields" and not containers for other data
pub const ROS_PRIMITIVE_TYPE_LIST: [&str; 17] = [
    "bool", "int8", "uint8", "byte", "char", "int16", "uint16", "int32", "uint32", "int64",
    "uint64", "float32", "float64", "string", "wstring", "time", "duration",
];

lazy_static::lazy_static! {
    pub static ref ROS_TYPE_TO_RUST_TYPE_MAP: HashMap<&'static str, &'static str> = vec![
        ("bool", "bool"),
        ("int8", "i8"),
        ("uint8", "u8"),
        ("byte", "u8"),
        ("char", "u8"), // NOTE: a rust char != C++ char
        ("int16", "i16"),
        ("uint16", "u16"),
        ("int32", "i32"),
        ("uint32", "u32"),
        ("int64", "i64"),
        ("uint64", "u64"),
        ("float32", "f32"),
        ("float64", "f64"),
        ("string", "::std::string::String"),
        ("time", "::roslibrust::codegen::integral_types::Time"),
        ("duration", "::roslibrust::codegen::integral_types::Duration"),
    ].into_iter().collect();

    pub static ref ROS_2_TYPE_TO_RUST_TYPE_MAP: HashMap<&'static str, &'static str> = vec![
        ("bool", "bool"),
        ("int8", "i8"),
        ("uint8", "u8"),
        ("byte", "u8"),
        ("char", "u8"),
        ("int16", "i16"),
        ("uint16", "u16"),
        ("int32", "i32"),
        ("uint32", "u32"),
        ("int64", "i64"),
        ("uint64", "u64"),
        ("float32", "f32"),
        ("float64", "f64"),
        ("string", "::std::string::String"),
        ("builtin_interfaces/Time", "::roslibrust::codegen::integral_types::Time"),
        ("builtin_interfaces/Duration", "::roslibrust::codegen::integral_types::Duration"),
        // ("wstring", TODO),
    ].into_iter().collect();
}

pub fn is_intrinsic_type(version: RosVersion, ros_type: &str) -> bool {
    match version {
        // Treat time and duration as intrinsic in ROS1
        RosVersion::ROS1 => ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(ros_type),
        // In ros 2 builtin_interfaces/Time and builtin_interfaces/Duration are not intrinsic
        RosVersion::ROS2 => ROS_PRIMITIVE_TYPE_LIST.contains(&ros_type),
    }
}

pub fn convert_ros_type_to_rust_type(version: RosVersion, ros_type: &str) -> Option<&'static str> {
    match version {
        RosVersion::ROS1 => ROS_TYPE_TO_RUST_TYPE_MAP.get(ros_type).copied(),
        RosVersion::ROS2 => ROS_2_TYPE_TO_RUST_TYPE_MAP.get(ros_type).copied(),
    }
}

fn parse_field(line: &str, pkg: &Package, msg_name: &str) -> Result<FieldInfo, Error> {
    let pkg_name = pkg.name.as_str();
    let (field_type, remainder) = split_type_from_line(line).ok_or(Error::new(format!(
        "Did not find field_type and field_name on line: {line} while parsing {pkg_name}/{msg_name}"
    )))?;
    let field_type = parse_type(field_type, pkg)?;
    let (field_name, default) = parse_field_name_and_default(remainder).ok_or(Error::new(
        format!("Did not find field_name on line: {line} while parsing {pkg_name}/{msg_name}"),
    ))?;

    let default = if matches!(pkg.version, Some(RosVersion::ROS2)) {
        default.map(RosLiteral::from)
    } else {
        None
    };

    Ok(FieldInfo {
        field_type,
        field_name: field_name.to_string(),
        default,
    })
}

fn parse_constant_field(line: &str, pkg: &Package) -> Result<ConstantInfo, Error> {
    let (constant_type, remainder) = split_type_from_line(line).ok_or(Error::new(format!(
        "Failed to find white space separator while parsing constant information on line {line} for package {pkg:?}"
    )))?;
    let equal = remainder.find('=').ok_or(
        Error::new(format!("Failed to find expected '=' while parsing constant information on line {line} for package {pkg:?}"))
    )?;
    let constant_type = parse_type(constant_type, pkg)?.field_type;
    let constant_name = remainder[..equal].trim().to_string();
    let constant_value = remainder[equal + 1..].trim().to_string();

    Ok(ConstantInfo {
        constant_type,
        constant_name,
        constant_value: constant_value.into(),
    })
}

fn split_type_from_line(line: &str) -> Option<(&str, &str)> {
    let type_end = line.find(char::is_whitespace)?;
    let remainder = line[type_end..].trim_start();
    if remainder.is_empty() {
        return None;
    }
    Some((&line[..type_end], remainder))
}

fn parse_field_name_and_default(remainder: &str) -> Option<(&str, Option<String>)> {
    match remainder.find(char::is_whitespace) {
        Some(name_end) => {
            let default = remainder[name_end..].trim();
            if default.is_empty() {
                Some((&remainder[..name_end], None))
            } else {
                Some((&remainder[..name_end], Some(default.to_owned())))
            }
        }
        None => Some((remainder, None)),
    }
}

/// Looks for # comment character and sub-slices for characters preceding it
/// Note: This should NOT be used for lines that contain string constants,
/// as # characters within string constant values are part of the value.
fn strip_comments(line: &str) -> &str {
    if let Some(token) = line.find('#') {
        return &line[..token];
    }
    line
}

/// Strips comments from a line, but respects string constant values.
/// For string constants, # characters within the value are NOT treated as comments per ROS spec.
fn strip_comments_respecting_string_constants(line: &str) -> &str {
    let trimmed = line.trim_start();

    if is_string_constant_line(trimmed) {
        return line;
    }

    // For everything else (fields, non-string constants, comments), strip normally
    strip_comments(line)
}

fn is_string_constant_line(line: &str) -> bool {
    let Some((type_name, remainder)) = split_type_from_line(line) else {
        return false;
    };
    (type_name == "string"
        || type_name == "wstring"
        || type_name.starts_with("string<=")
        || type_name.starts_with("wstring<="))
        && remainder.contains('=')
}

fn parse_field_type(
    type_str: &str,
    array_info: ArrayType,
    pkg: &Package,
) -> Result<FieldType, Error> {
    let items = type_str.split('/').collect::<Vec<&str>>();

    if items.len() == 1 {
        // If there is only one item (no package redirect)
        let pkg_version = pkg.version.unwrap_or(RosVersion::ROS1);

        let (field_type, string_capacity) = parse_bounded_string(items[0])?;

        Ok(FieldType {
            package_name: if is_intrinsic_type(pkg_version, &field_type) {
                // If it is a fundamental type, no package
                None
            } else {
                // Very special case for "Header"
                if type_str == "Header" {
                    Some("std_msgs".to_owned())
                } else {
                    // Otherwise it is referencing another message in the same package
                    Some(pkg.name.clone())
                }
            },
            source_package: pkg.name.clone(),
            field_type,
            array_info,
            string_capacity,
        })
    } else {
        // If there is more than one item there is a package redirect

        Ok(FieldType {
            package_name: Some(items[0].to_string()),
            source_package: pkg.name.clone(),
            field_type: items[1].to_string(),
            string_capacity: None,
            array_info,
        })
    }
}

/// Specifically handles bounded string types, e.g. "string<=10"
/// Returns the field_type and the string_capacity if it is a bounded string
/// Otherwise returns the original type and None for the capacity
fn parse_bounded_string(type_str: &str) -> Result<(String, Option<usize>), Error> {
    if let Some(stripped) = type_str.strip_prefix("string<=") {
        let capacity = stripped.parse::<usize>().map_err(|err| {
            Error::new(format!(
                "Unable to parse capacity of bounded string: {type_str}: {err}"
            ))
        })?;
        Ok(("string".to_string(), Some(capacity)))
    } else {
        Ok((type_str.to_string(), None))
    }
}

/// Determines the type of a field
/// `type_str` -- Expects the part of the line containing all type information (up to the first space), e.g. "int32[3>=]"
/// `pkg` -- Reference to package this type is within, used for version information and determining relative types
fn parse_type(type_str: &str, pkg: &Package) -> Result<FieldType, Error> {
    // Handle array logic
    let open_bracket_idx = type_str.find('[');
    let close_bracket_idx = type_str.find(']');
    match (open_bracket_idx, close_bracket_idx) {
        (Some(o), Some(c)) => {
            // After having stripped array information, parse the remainder of the type
            let array_info = if c - o == 1 {
                // No size specified
                ArrayType::Unbounded
            } else {
                let fixed_size_str = &type_str[(o + 1)..c];
                let is_bounded;
                let offset;
                // Check if the first two characters are <=
                if fixed_size_str.starts_with("<=") {
                    is_bounded = true;
                    offset = 2;
                } else {
                    is_bounded = false;
                    offset = 0;
                }

                let fixed_size = fixed_size_str[offset..].parse::<usize>().map_err(|err| {
                    Error::new(format!(
                        "Unable to parse size of the array: {type_str}, defaulting to 0: {err}"
                    ))
                })?;
                if is_bounded {
                    ArrayType::Bounded(fixed_size)
                } else {
                    ArrayType::FixedLength(fixed_size)
                }
            };
            parse_field_type(&type_str[..o], array_info, pkg)
        }
        (None, None) => {
            // Not an array parse normally
            parse_field_type(type_str, ArrayType::NotArray, pkg)
        }
        _ => {
            bail!("Found malformed type: {type_str} in package {pkg:?}. Likely file is invalid.");
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{
        parse::parse_type,
        utils::{Package, RosVersion},
        ArrayType,
    };

    // Simple test to just confirm fixed size logic is working correctly on the parse side
    #[test_log::test]
    fn parse_type_handles_fixed_size_correctly() {
        let line = "int32[9]";
        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };
        let parsed = parse_type(line, &pkg).unwrap();
        assert_eq!(parsed.array_info, ArrayType::FixedLength(9));
    }

    #[test_log::test]
    fn parse_type_handles_bounded_size_correctly() {
        let line = "int32[<=9]";
        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };
        let parsed = parse_type(line, &pkg).unwrap();
        assert_eq!(parsed.array_info, ArrayType::Bounded(9));
    }

    #[test_log::test]
    fn parse_type_handles_unbounded_size_correctly() {
        let line = "int32[]";
        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };
        let parsed = parse_type(line, &pkg).unwrap();
        assert_eq!(parsed.array_info, ArrayType::Unbounded);
    }

    #[test_log::test]
    fn parse_constant_with_hash_in_value() {
        use crate::parse::parse_constant_field;

        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };

        // Test parsing a constant with # in the value
        let line = "string HASH_IN_VALUE=foo # bar";
        let constant = parse_constant_field(line, &pkg).unwrap();

        assert_eq!(constant.constant_name, "HASH_IN_VALUE");
        assert_eq!(constant.constant_type, "string");
        // This should be "foo # bar" according to ROS spec
        assert_eq!(constant.constant_value.inner, "foo # bar");
    }

    #[test_log::test]
    fn parse_bounded_string_constant_with_hash_in_value() {
        use crate::parse::parse_ros_message_file;
        use std::path::Path;

        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS2),
        };

        let msg_content = r#"string<=32 HASH_IN_VALUE='foo # bar'
string data
"#;

        let parsed =
            parse_ros_message_file(msg_content, "TestMsg", &pkg, Path::new("test.msg")).unwrap();

        assert_eq!(parsed.constants[0].constant_type, "string");
        assert_eq!(parsed.constants[0].constant_value.inner, "'foo # bar'");
    }

    #[test_log::test]
    fn parse_message_with_hash_in_string_constant() {
        use crate::parse::parse_ros_message_file;
        use std::path::Path;

        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };

        let msg_content = r#"# Test message
string HASH_IN_VALUE=foo # bar
string NO_SPACES=test#value
string HASH_AT_START=#start
string HASH_AT_END=end#
string data
"#;

        let parsed =
            parse_ros_message_file(msg_content, "TestMsg", &pkg, Path::new("test.msg")).unwrap();

        assert_eq!(parsed.constants.len(), 4);
        assert_eq!(parsed.fields.len(), 1);

        // Check all constants have correct values with # characters
        assert_eq!(parsed.constants[0].constant_name, "HASH_IN_VALUE");
        assert_eq!(parsed.constants[0].constant_value.inner, "foo # bar");

        assert_eq!(parsed.constants[1].constant_name, "NO_SPACES");
        assert_eq!(parsed.constants[1].constant_value.inner, "test#value");

        assert_eq!(parsed.constants[2].constant_name, "HASH_AT_START");
        assert_eq!(parsed.constants[2].constant_value.inner, "#start");

        assert_eq!(parsed.constants[3].constant_name, "HASH_AT_END");
        assert_eq!(parsed.constants[3].constant_value.inner, "end#");

        // Check field is parsed correctly
        assert_eq!(parsed.fields[0].field_name, "data");
    }

    #[test_log::test]
    fn parse_non_string_constants_with_comments() {
        use crate::parse::parse_ros_message_file;
        use std::path::Path;

        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };

        // Test that non-string constants still have comments stripped correctly
        let msg_content = r#"# Test message
int32 INT_CONST=42 # this is a comment
float32 FLOAT_CONST=3.14 # another comment
bool BOOL_CONST=true # yet another comment
"#;

        let parsed =
            parse_ros_message_file(msg_content, "TestMsg", &pkg, Path::new("test.msg")).unwrap();

        assert_eq!(parsed.constants.len(), 3);

        // Non-string constants should have comments stripped
        assert_eq!(parsed.constants[0].constant_name, "INT_CONST");
        assert_eq!(parsed.constants[0].constant_value.inner, "42");

        assert_eq!(parsed.constants[1].constant_name, "FLOAT_CONST");
        assert_eq!(parsed.constants[1].constant_value.inner, "3.14");

        assert_eq!(parsed.constants[2].constant_name, "BOOL_CONST");
        assert_eq!(parsed.constants[2].constant_value.inner, "true");
    }
}
