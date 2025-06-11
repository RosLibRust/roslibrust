use crate::parse::{parse_constant_field, parse_field, strip_comments};
use crate::Error;
use crate::{ConstantInfo, FieldInfo, Package, RosVersion};
use std::path::{Path, PathBuf};

/// Describes all information for a single message file
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
    pub fn has_header(&self) -> bool {
        self.fields.iter().any(|field| {
            field.field_type.field_type.as_str() == "Header"
                && (field.field_type.package_name.is_none()
                    || field.field_type.package_name == Some(String::from("std_msgs")))
        })
    }

    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.package, self.name)
    }
}

/// Converts a ros message file into a struct representation
/// * `data` -- Raw contents of the file as loaded from disk
/// * `name` -- Name of the object being parsed excluding the file extension, e.g. `Header`
/// * `package` -- Name of the package the message is found in, required for relative type paths
/// * `ros2` -- True iff the package is a ros2 package and should be parsed with ros2 logic
pub fn parse_ros_message_file(
    data: &str,
    name: &str,
    package: &Package,
    path: &Path,
) -> Result<ParsedMessageFile, Error> {
    let mut fields = vec![];
    let mut constants = vec![];

    for full_line in data.lines() {
        let line_without_comments = strip_comments(full_line).trim();
        if line_without_comments.is_empty() {
            // Comment only line skip
            continue;
        }
        // Determine if we're looking at a constant or a field
        let sep = line_without_comments.find(' ').ok_or(
            Error::new(
                format!("Found an invalid ros field line, no space delinting type from name: {line_without_comments} in {}\n{data}",
                path.display())
            )
        )?;
        let equal_after_sep = line_without_comments[sep..].find('=');
        if equal_after_sep.is_some() {
            // Since we found an equal sign after a space, this must be a constant
            constants.push(parse_constant_field(
                line_without_comments,
                full_line,
                package,
            )?)
        } else {
            // Is regular field
            fields.push(parse_field(line_without_comments, package, name)?);
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

#[cfg(test)]
mod test {
    use std::path::Path;

    use crate::{
        parse::msg::parse_ros_message_file,
        utils::{Package, RosVersion},
    };

    #[test_log::test]
    fn parse_ros_message_file_works() {
        let data = r#"
# This is a comment
int32 my_int
float64 my_float # with a comment
string my_string
string CONSTANT_WITH_SHARP=# foo
int32 INTEGER_CONSTANT=123 #with comment
"#;
        let package = Package {
            name: "test_package".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };
        let parsed =
            parse_ros_message_file(data, "test.msg", &package, Path::new("test.msg")).unwrap();
        assert_eq!(parsed.fields.len(), 3);
        assert_eq!(parsed.fields[0].field_name, "my_int");
        assert_eq!(parsed.fields[1].field_name, "my_float");
        assert_eq!(parsed.fields[2].field_name, "my_string");
        assert_eq!(parsed.constants[0].constant_name, "CONSTANT_WITH_SHARP");
        assert_eq!(
            parsed.constants[0].constant_value.inner,
            "# foo".to_string()
        );

        assert_eq!(parsed.constants[1].constant_name, "INTEGER_CONSTANT");
        assert_eq!(parsed.constants[1].constant_value.inner, "123".to_string());
    }
}
