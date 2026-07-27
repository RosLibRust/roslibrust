use std::cell::Cell;

use crate::RosMessageType;

thread_local! {
    static ROS1_WSTRING_COMPATIBILITY_DEPTH: Cell<usize> = const { Cell::new(0) };
}

/// Runs a ROS 1 serialization or deserialization operation with `wstring`
/// compatibility enabled.
///
/// This is used internally by ROS 1 backends. ROS 1 has no native `wstring`,
/// so generated ROS 2 `wstring` values are represented as ordinary UTF-8 ROS
/// strings while this scope is active.
#[doc(hidden)]
pub fn with_ros1_wstring_compatibility<T>(operation: impl FnOnce() -> T) -> T {
    struct ScopeGuard;

    impl Drop for ScopeGuard {
        fn drop(&mut self) {
            ROS1_WSTRING_COMPATIBILITY_DEPTH.with(|depth| depth.set(depth.get() - 1));
        }
    }

    ROS1_WSTRING_COMPATIBILITY_DEPTH.with(|depth| depth.set(depth.get() + 1));
    let _guard = ScopeGuard;
    operation()
}

/// Reports whether the current synchronous serde operation is being performed
/// by a ROS 1 backend.
#[doc(hidden)]
pub fn ros1_wstring_compatibility_enabled() -> bool {
    ROS1_WSTRING_COMPATIBILITY_DEPTH.with(|depth| depth.get() > 0)
}

/// ROS 1-compatible connection metadata for a message type.
#[doc(hidden)]
pub struct Ros1MessageDescription {
    pub definition: String,
    pub md5sum: String,
}

/// Converts ROS 2 `wstring` field declarations to their closest ROS 1 `string`
/// equivalents and calculates the MD5 advertised to ROS 1 peers.
#[doc(hidden)]
pub fn ros1_message_description<T: RosMessageType>() -> Ros1MessageDescription {
    let definition = ros1_compatible_definition(T::DEFINITION);
    let md5sum = crate::md5sum::from_message_definition(T::ROS_TYPE_NAME, &definition)
        .unwrap_or_else(|_| T::MD5SUM.to_owned());
    Ros1MessageDescription { definition, md5sum }
}

fn ros1_compatible_definition(definition: &str) -> String {
    definition
        .lines()
        .map(|line| {
            let trimmed = line.trim_start();
            if trimmed.starts_with('#') {
                return line.to_owned();
            }

            let Some(type_end) = trimmed.find(char::is_whitespace) else {
                return line.to_owned();
            };
            let field_type = &trimmed[..type_end];
            if !field_type.starts_with("wstring") {
                return line.to_owned();
            }

            let array = field_type.find('[').map(|start| &field_type[start..]);
            let ros1_array = match array {
                Some(array) if array.starts_with("[<=") => "[]",
                Some(array) => array,
                None => "",
            };
            let replacement = format!("string{ros1_array}");
            let indentation = &line[..line.len() - trimmed.len()];
            let remainder = trimmed[type_end..].trim_start();
            let field_name_end = remainder
                .find(char::is_whitespace)
                .unwrap_or(remainder.len());
            let field_name = &remainder[..field_name_end];
            if field_name.contains('=') {
                format!("{indentation}{replacement} {remainder}")
            } else {
                // ROS 2 permits field defaults, but ROS 1 definitions do not.
                format!("{indentation}{replacement} {field_name}")
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compatibility_scope_is_nested_and_restored() {
        assert!(!ros1_wstring_compatibility_enabled());
        with_ros1_wstring_compatibility(|| {
            assert!(ros1_wstring_compatibility_enabled());
            with_ros1_wstring_compatibility(|| {
                assert!(ros1_wstring_compatibility_enabled());
            });
            assert!(ros1_wstring_compatibility_enabled());
        });
        assert!(!ros1_wstring_compatibility_enabled());
    }

    #[test]
    fn definition_conversion_only_changes_wstring_type_tokens() {
        let definition = r#"# wstring in a comment
wstring wstring_value
wstring<=22 bounded_wstring_value "default"
wstring[3] array_of_wstrings
wstring[<=3] bounded_sequence_of_wstrings
wstring[] unbounded_sequence_of_wstrings"#;

        assert_eq!(
            ros1_compatible_definition(definition),
            r#"# wstring in a comment
string wstring_value
string bounded_wstring_value
string[3] array_of_wstrings
string[] bounded_sequence_of_wstrings
string[] unbounded_sequence_of_wstrings"#
        );
    }
}
