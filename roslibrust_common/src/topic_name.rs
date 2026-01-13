use crate::Error as RError;

/// A unified representation for the names of topics, services, and actions.
///
/// This type tries to conform to the naming conventions of both ROS1 and ROS2,
/// but only supports a subset of the valid names as a result.
///
/// For the time being roslibrust has decided to perform no automatic substitutions of name
/// elements for users, so all names must be "Global" and be fully resolved and start with a `/`.
/// ROS2 refers to these names a "fully qualified name".
///
/// Examples of valid global names (to roslibrust's standards are):
/// * `/chatter`
/// * `/foo/bar/baz`
/// * `/foo_bar_baz`
/// * `/abc123/def_456`
///
/// Examples of invalid Global names:
/// * `chatter` - ROS1 / ROS2 may namespace this differently
/// * `~chatter` - ROS2 rejects this without a '/' between `~` and the name
/// * `~/chatter` - Even with the '/' RosLibRust will reject this as we don't support the '~'
///
///
// For developers of this area look here for ROS1 documentation https://wiki.ros.org/Names
// and here for ROS2 documentation https://design.ros2.org/articles/topic_and_service_names.html
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct GlobalTopicName {
    inner: String,
}

impl GlobalTopicName {
    pub fn new(name: impl Into<String>) -> Result<GlobalTopicName, RError> {
        let name: String = name.into();
        match validate_global_name(&name) {
            Ok(()) => Ok(Self { inner: name }),
            Err(failures) => Err(RError::InvalidName(format!(
                "Invalid topic name: {name}, reasons: {failures:?}"
            ))),
        }
    }
}

/// Can print the name with `{}` syntax as it has a canonical string representation
impl std::fmt::Display for GlobalTopicName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.inner.fmt(f)
    }
}

// Conversion into a String is allowed as it is the canonical representation
impl From<GlobalTopicName> for String {
    fn from(name: GlobalTopicName) -> Self {
        name.inner
    }
}

// Allow GlobalTopicName to be used as &str
impl AsRef<str> for GlobalTopicName {
    fn as_ref(&self) -> &str {
        &self.inner
    }
}

/// This trait represents types that can be converted into a GlobalTopicName
///
/// This trait is in use within roslibrust because we have APIs that want to take either
/// &str, String, GlobalTopicName, or &GlobalTopicName, but we don't want to use TryFrom
/// as the error types are not compatible, and would force additional boilerplate on users.
pub trait ToGlobalTopicName: Send {
    fn to_global_name(self) -> Result<GlobalTopicName, RError>;
}

impl ToGlobalTopicName for GlobalTopicName {
    fn to_global_name(self) -> Result<GlobalTopicName, RError> {
        Ok(self)
    }
}

impl ToGlobalTopicName for &GlobalTopicName {
    fn to_global_name(self) -> Result<GlobalTopicName, RError> {
        Ok(self.clone())
    }
}

impl ToGlobalTopicName for String {
    fn to_global_name(self) -> Result<GlobalTopicName, RError> {
        GlobalTopicName::new(self)
    }
}

impl ToGlobalTopicName for &String {
    fn to_global_name(self) -> Result<GlobalTopicName, RError> {
        GlobalTopicName::new(self)
    }
}

impl ToGlobalTopicName for &str {
    fn to_global_name(self) -> Result<GlobalTopicName, RError> {
        GlobalTopicName::new(self)
    }
}

static GLOBAL_NAME_REGEX: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
    // Best attempt at a regex that matches both ROS1 and ROS2 naming conventions
    regex::Regex::new(r"(?-u)^\/([A-Za-z][A-Za-z0-9_]*)(\/[A-Za-z][A-Za-z0-9_]*)*$").unwrap()
});

/// Check the name against our set of rules for validity
/// Returns a list of reasons the name is invalid
fn validate_global_name(name: &str) -> Result<(), Vec<String>> {
    // First character must be a '/'
    let mut failures = vec![];
    if !name.starts_with('/') {
        failures.push("Name must start with a '/'".to_string());
    }

    // Name must not contain any whitespace
    if name.contains(char::is_whitespace) {
        failures.push("Name must not contain whitespace".to_string());
    }

    // Name must not contain characters other than alphanumeric, underscore, and forward slash
    if !name
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '/')
    {
        failures.push(
            "Name must only contain alphanumeric characters, underscores, and forward slashes"
                .to_string(),
        );
    }

    // Name must not end with a '/'
    if name.ends_with('/') {
        failures.push("Name must not end with a '/'".to_string());
    }

    // Use the ROS1 validation regex for a final check (in a lazy cell)
    if !GLOBAL_NAME_REGEX.is_match(name) {
        failures.push("Name must match the ROS1 name validation regex".to_string());
    }

    if failures.is_empty() {
        Ok(())
    } else {
        Err(failures)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_global_names() {
        assert!(GlobalTopicName::new("/chatter").is_ok());
        assert!(GlobalTopicName::new("/foo/bar/baz").is_ok());
        assert!(GlobalTopicName::new("/foo_bar_baz").is_ok());
        assert!(GlobalTopicName::new("/abc123/def_456").is_ok());

        assert!(GlobalTopicName::new("chatter").is_err());
        assert!(GlobalTopicName::new("chatter/").is_err());
        assert!(GlobalTopicName::new("/chatter/").is_err());
        assert!(GlobalTopicName::new("/chatter ").is_err());
        assert!(GlobalTopicName::new("/chatter space").is_err());
        assert!(GlobalTopicName::new("/chatter#").is_err());
        assert!(GlobalTopicName::new("~chatter").is_err());
        assert!(GlobalTopicName::new("/chatter/{ros2}").is_err());
        assert!(GlobalTopicName::new("/chatter-").is_err());
        assert!(GlobalTopicName::new("/chatter/with space").is_err());
        assert!(GlobalTopicName::new("/chatter/with#hash").is_err());
        assert!(GlobalTopicName::new("/empty//bad").is_err());

        // It is unclear for the ROS documentation if this should be valid or not
        // assert!(GlobalTopicName::new("/123/_456/").is_err());
        // assert!(GlobalTopicName::new("/123").is_err());
    }

    #[test]
    fn type_conversions_exist_and_behave() {
        // Test for the ToGlobalTopicName trait - this is how roslibrust traits work!
        fn generic_with_to_global<MsgType>(name: impl ToGlobalTopicName) {
            let name: GlobalTopicName = name.to_global_name().unwrap();
            assert_eq!(name.to_string(), "/chatter".to_string());
        }

        // Works with String
        generic_with_to_global::<String>("/chatter".to_string());
        // Works with &String
        let chatter = "/chatter".to_string();
        generic_with_to_global::<String>(&chatter);
        // Works with &str
        generic_with_to_global::<String>("/chatter");
        // Works with GlobalTopicName
        generic_with_to_global::<String>(GlobalTopicName::new("/chatter").unwrap());
        // Works with &GlobalTopicName
        generic_with_to_global::<String>(&GlobalTopicName::new("/chatter").unwrap());
    }
}
