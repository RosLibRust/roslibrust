use crate::utils::{Package, RosVersion};
use crate::{bail, ArrayType, Error};
use crate::{ConstantInfo, FieldInfo, FieldType};
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
    let mut splitter = line.split_whitespace();
    let pkg_name = pkg.name.as_str();
    let field_type = splitter.next().ok_or(Error::new(format!(
        "Did not find field_type on line: {line} while parsing {pkg_name}/{msg_name}"
    )))?;
    let field_type = parse_type(field_type, pkg)?;
    let field_name = splitter.next().ok_or(Error::new(format!(
        "Did not find field_name on line: {line} while parsing {pkg_name}/{msg_name}"
    )))?;

    let sep = line.find(' ').unwrap();
    // Determine if there is a default value for this field
    let default = if matches!(pkg.version, Some(RosVersion::ROS2)) {
        // For ros2 packages only, check if there is a default value
        let line_after_sep = line[sep + 1..].trim();
        match line_after_sep.find(' ') {
            Some(def_start) => {
                let remainder = line_after_sep[def_start..].trim();
                if remainder.is_empty() {
                    None
                } else {
                    Some(remainder.to_owned().into())
                }
            }
            None => {
                // No extra space separator found, not default was provided
                None
            }
        }
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
    let sep = line.find(' ').ok_or(
        Error::new(format!("Failed to find white space seperator ' ' while parsing constant information one line {line} for package {pkg:?}"))
    )?;
    let equal_after_sep = line[sep..].find('=').ok_or(
        Error::new(format!("Failed to find expected '=' while parsing constant information on line {line} for package {pkg:?}"))
    )?;
    let mut constant_type = parse_type(line[..sep].trim(), pkg)?.field_type;
    let constant_name = line[sep + 1..(equal_after_sep + sep)].trim().to_string();

    // Handle the fact that string type should be different for constants than fields
    if constant_type == "String" {
        constant_type = "&'static str".to_string();
    }

    let constant_value = line[sep + equal_after_sep + 1..].trim().to_string();
    Ok(ConstantInfo {
        constant_type,
        constant_name,
        constant_value: constant_value.into(),
    })
}

/// Looks for # comment character and sub-slices for characters preceding it
fn strip_comments(line: &str) -> &str {
    if let Some(token) = line.find('#') {
        return &line[..token];
    }
    line
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
}
