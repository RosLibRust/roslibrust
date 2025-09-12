use crate::parse::{parse_constant_field, parse_field, strip_comments};
use crate::Error;
use crate::{ConstantInfo, FieldInfo, Package, RosVersion};
use std::path::{Path, PathBuf};

/// Describes all information for a single message file available in the file without other context
/// This is different from MessageFile which contains resolved information from dependencies
#[derive(Clone, PartialEq, Debug)]
pub struct ParsedMessageFile {
    pub name: String,
    pub package: String,
    pub fields: Vec<FieldInfo>,
    pub constants: Vec<ConstantInfo>,
    pub version: Option<RosVersion>,
    /// The contents of the message file this instance was parsed from
    pub source: String,
    /// The path where the message was found
    pub path: PathBuf,
}

impl ParsedMessageFile {
    // True iff this message contains a field of type std_msgs/Header
    pub fn has_header(&self) -> bool {
        self.fields.iter().any(|field| {
            field.field_type.field_type.as_str() == "Header"
                && (field.field_type.package_name.is_none()
                    || field.field_type.package_name == Some(String::from("std_msgs")))
        })
    }

    /// Returns full ros1 name of the message e.g. std_msgs/String
    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.package, self.name)
    }

    /// Returns full ros2 name of the message, e.g. std_msgs/msg/String or example_interfaces/srv/AddTwoInts
    pub fn get_ros2_full_name(&self) -> String {
        // Not sure this a safe assumption, but for now we'll extract the name of the parent folder as the "middle name" in ROS2
        let parent = self
            .path
            .parent()
            .expect("All ROS messages should have a parent directory")
            .file_name()
            .expect("All parent directories should have a resolveable file name")
            .to_str()
            .expect("All parent directory names should be valid unicode");
        format!("{}/{}/{}", self.package, parent, self.name)
    }

    /// Returns the fully qualified type name for use with ROS2 zenoh
    /// This is used in the mangling of topic names to subscribe to ros2 topics over zenoh
    pub fn get_ros2_dds_type_name(&self) -> String {
        format!("{}::msg::dds_::{}_", self.package, self.name)
    }
}

/// Converts a ros message file into a struct representation
/// * `data` -- Raw contents of the file as loaded from disk
/// * `name` -- Name of the object being parsed excluding the file extension, e.g. `Header`
/// * `package` -- Name of the package the message is found in, required for relative type paths
/// * `path` -- Path to the message file
pub fn parse_ros_message_file(
    data: &str,
    name: &str,
    package: &Package,
    path: &Path,
) -> Result<ParsedMessageFile, Error> {
    let mut fields = vec![];
    let mut constants = vec![];

    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.is_empty() {
            // Comment only line skip
            continue;
        }
        // Determine if we're looking at a constant or a field
        let sep = line.find(' ').ok_or(
            Error::new(
                format!("Found an invalid ros field line, no space delinting type from name: {line} in {}\n{data}",
                path.display())
            )
        )?;
        let equal_after_sep = line[sep..].find('=');
        if equal_after_sep.is_some() {
            // Since we found an equal sign after a space, this must be a constant
            constants.push(parse_constant_field(line, package)?)
        } else {
            // Is regular field
            fields.push(parse_field(line, package, name)?);
        }
    }
    Ok(ParsedMessageFile {
        fields,
        constants,
        name: name.to_owned(),
        package: package.name.clone(),
        version: package.version,
        source: data.to_owned(),
        path: path.to_owned(),
    })
}
