use std::collections::{HashMap, HashSet};
use std::io;
use std::path::{Path, PathBuf};

use log::debug;

#[derive(Clone, Debug)]
pub struct Package {
    pub name: String,
    pub path: PathBuf,
    /// For now RosVersion is being left as an option, because our ability to detect the correct version is in question
    pub version: Option<RosVersion>,
}

impl PartialEq for Package {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.version == other.version
    }
}

#[derive(Clone, Debug, PartialEq, Copy)]
pub enum RosVersion {
    ROS1,
    ROS2,
}

const CATKIN_IGNORE: &str = "CATKIN_IGNORE";
const AMENT_IGNORE: &str = "AMENT_IGNORE";
const COLCON_IGNORE: &str = "COLCON_IGNORE";
const PACKAGE_FILE_NAME: &str = "package.xml";
const ROS_PACKAGE_PATH_ENV_VAR: &str = "ROS_PACKAGE_PATH";
const AMENT_PREFIX_PATH_ENV_VAR: &str = "AMENT_PREFIX_PATH";
const COLCON_PREFIX_PATH_ENV_VAR: &str = "COLCON_PREFIX_PATH";
const AMENT_INDEX_PATH: [&str; 4] = ["share", "ament_index", "resource_index", "packages"];

pub fn get_search_paths() -> Vec<PathBuf> {
    if let Some(paths) = std::env::var_os(ROS_PACKAGE_PATH_ENV_VAR) {
        std::env::split_paths(&paths).collect()
    } else {
        log::warn!("No ROS_PACKAGE_PATH defined.");
        vec![]
    }
}

pub fn get_ros2_search_paths() -> Vec<PathBuf> {
    let prefixes = [AMENT_PREFIX_PATH_ENV_VAR, COLCON_PREFIX_PATH_ENV_VAR]
        .into_iter()
        .flat_map(|env_var| match std::env::var_os(env_var) {
            Some(paths) => std::env::split_paths(&paths).collect::<Vec<_>>(),
            None => {
                log::debug!("No {env_var} defined.");
                vec![]
            }
        })
        .collect::<Vec<_>>();

    get_ros2_search_paths_from_prefixes(&prefixes)
}

/// Finds ROS 2 package share directories by reading each prefix's ament resource index.
///
/// A ROS 2 install prefix marks packages with files under
/// `share/ament_index/resource_index/packages/<package_name>`. The corresponding
/// package share directory is expected at `share/<package_name>`.
pub fn get_ros2_search_paths_from_prefixes<P: AsRef<Path>>(prefixes: &[P]) -> Vec<PathBuf> {
    let mut package_paths = Vec::new();
    let mut seen = HashSet::new();

    for prefix in prefixes {
        let prefix = prefix.as_ref();
        let package_index = AMENT_INDEX_PATH
            .into_iter()
            .fold(prefix.to_path_buf(), |path, component| path.join(component));
        let Ok(entries) = std::fs::read_dir(&package_index) else {
            log::debug!(
                "No ament package resource index found at {}",
                package_index.display()
            );
            continue;
        };

        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }

            let Some(package_name) = path.file_name() else {
                continue;
            };
            let package_path = prefix.join("share").join(package_name);
            if !package_path.join(PACKAGE_FILE_NAME).is_file() {
                log::warn!(
                    "Ament package marker {} did not have a matching package.xml at {}",
                    path.display(),
                    package_path.display()
                );
                continue;
            }

            if seen.insert(package_path.clone()) {
                package_paths.push(package_path);
            }
        }
    }

    package_paths
}

/// Finds ROS packages within a list of search paths.
///
/// This function may panic if it reaches a maximum search depth. If this function
/// panics while you're using it, you may have some infinite loop in your paths
/// due to symlinking.
pub fn crawl<P: AsRef<Path>>(search_paths: &[P]) -> Vec<Package> {
    let mut packages = vec![];

    for path in search_paths {
        const MAX_RECURSION_DEPTH: u16 = 1000;
        if let Ok(found_packages) =
            packages_from_path(path.as_ref().to_owned(), MAX_RECURSION_DEPTH)
        {
            packages = [packages, found_packages].concat();
        }
    }
    debug!(
        "Found {:?} packages while crawling search paths.",
        packages.len()
    );
    packages
}

pub fn packages_from_path(mut path: PathBuf, depth: u16) -> io::Result<Vec<Package>> {
    let mut found_packages = vec![];

    if depth == 0 {
        log::error!(
            "Reached depth limit in: {}. Possible symlink loop detected.",
            path.as_os_str().to_string_lossy()
        );
        return Err(io::ErrorKind::Other.into());
    }

    if path.as_path().is_dir() {
        let ignored = [CATKIN_IGNORE, AMENT_IGNORE, COLCON_IGNORE]
            .into_iter()
            .any(|ignore_file| path.join(ignore_file).is_file());

        // We'll only check this directory if no ignore marker file is present.
        if !ignored {
            path.push(PACKAGE_FILE_NAME);
            if path.as_path().is_file() {
                // And there's a package.xml here!
                if let Ok((version, name)) = parse_ros_package_info(&path) {
                    // Remove package.xml from our path
                    assert!(path.pop());

                    log::debug!("Found package {name} at {}", path.display());

                    found_packages.push(Package {
                        name,
                        path,
                        version,
                    });
                }
            } else {
                // No file here, we'll have to go deeper
                assert!(path.pop());
                for subdir in std::fs::read_dir(path)
                    .unwrap()
                    .filter(|entry| match entry {
                        Ok(entry) => entry.path().as_path().is_dir(),
                        Err(_err) => false,
                    })
                    .map(|entry| entry.unwrap())
                {
                    found_packages = [
                        found_packages,
                        packages_from_path(subdir.path(), depth - 1)?,
                    ]
                    .concat()
                }
            }
        }
    } else {
        log::error!("{} is not a directory", path.to_string_lossy())
    }

    Ok(found_packages)
}

pub fn get_message_files(pkg: &Package) -> io::Result<Vec<PathBuf>> {
    Ok(message_files_from_path(pkg.path.as_path(), "msg")?
        .into_iter()
        .chain(message_files_from_path(pkg.path.as_path(), "srv")?)
        .chain(message_files_from_path(pkg.path.as_path(), "action")?)
        .collect())
}

fn message_files_from_path(path: &Path, ext: &str) -> io::Result<Vec<PathBuf>> {
    let mut msg_files = vec![];
    for entry in (std::fs::read_dir(path)?).flatten() {
        if entry.path().as_path().is_dir() {
            msg_files = [
                msg_files,
                message_files_from_path(entry.path().as_path(), ext)?,
            ]
            .concat()
        } else if entry.path().as_path().is_file() {
            if let Some(extension) = entry.path().extension() {
                if extension.to_str().unwrap() == ext {
                    msg_files.push(entry.path())
                }
            }
        }
    }

    Ok(msg_files)
}

pub fn deduplicate_packages(packages: Vec<Package>) -> Vec<Package> {
    fn package_name_fmt(pkg: &Package) -> String {
        format!(
            "{}_{}",
            pkg.name,
            match pkg.version {
                Some(RosVersion::ROS1) => "1",
                Some(RosVersion::ROS2) => "2",
                None => "unknown",
            }
        )
    }

    let mut package_map: HashMap<String, Package> = HashMap::new();
    for package in packages {
        if let Some(duplicate) = package_map.get(package.name.as_str()) {
            if &package == duplicate {
                log::warn!(
                    "Duplicate package found: {}. Discovered at paths: ({}, {})",
                    package.name,
                    duplicate.path.display(),
                    package.path.display()
                );
                log::warn!(
                    "Proceeding with the package found at the first path: {}",
                    duplicate.path.display()
                );
            } else {
                package_map.insert(package_name_fmt(&package), package);
            }
        } else {
            package_map.insert(package_name_fmt(&package), package);
        }
    }

    package_map.into_values().collect()
}

/// Parses a ROS package.xml file, which may be in any of the 3 supported formats,
/// and returns a tuple of (RosVersion, Package Name)
/// Note: the name of the folder the package resides in is NOT the name of the package,
/// although that is the convention.
/// Finding the name is considered infallible and panics if name cannot be determined
/// ROS version determination is heuristic only, and returns None if failed.
/// See: https://answers.ros.org/question/410017/how-to-determine-if-a-package-is-ros1-or-ros2/
fn parse_ros_package_info(
    path: impl AsRef<Path> + std::fmt::Debug,
) -> io::Result<(Option<RosVersion>, String)> {
    use std::fs::File;
    use std::io::BufReader;
    use xml::reader::{EventReader, ParserConfig, XmlEvent};
    const BUILD_TOOL_TAG: &str = "buildtool_depend";
    const NAME_TAG: &str = "name";

    let file = File::open(&path)?;
    let reader = BufReader::new(file);
    let parser = EventReader::new_with_config(
        reader,
        ParserConfig {
            trim_whitespace: true,
            ignore_comments: true,
            ..Default::default()
        },
    );

    let mut in_build = false;
    let mut in_name = false;
    let mut version = None;
    let mut name = None;
    for e in parser {
        match e {
            Ok(XmlEvent::StartElement { name, .. }) => {
                if name.local_name == BUILD_TOOL_TAG {
                    in_build = true;
                } else if name.local_name == NAME_TAG {
                    in_name = true;
                }
            }
            Ok(XmlEvent::EndElement { name, .. }) => {
                if name.local_name == BUILD_TOOL_TAG {
                    in_build = false;
                } else if name.local_name == NAME_TAG {
                    in_name = false;
                }
            }
            Ok(XmlEvent::Characters(data)) => {
                if in_build {
                    log::trace!("Got data inside of {BUILD_TOOL_TAG}: {data}");
                    match data.as_str() {
                        "catkin" => {
                            version = Some(RosVersion::ROS1);
                        }
                        "ament_cmake" | "ament_cmake_ros" | "ament_python" => {
                            version = Some(RosVersion::ROS2);
                        }
                        _ => {}
                    };
                } else if in_name {
                    log::trace!("Got data inside of {NAME_TAG}: {data}");
                    name = Some(data);
                }
            }
            _ => {}
        }
    }

    if let Some(name) = name {
        Ok((version, name))
    } else {
        log::error!(
            "Failed to find the <name> tag within package.xml, which is a required tag: {path:?}"
        );
        Err(io::ErrorKind::Other.into())
    }
}

#[cfg(test)]
mod test {
    use crate::utils;
    use std::fs;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_test_dir(name: &str) -> PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!("roslibrust_codegen_{name}_{nanos}"))
    }

    fn write_package_xml(path: &std::path::Path, package_name: &str, build_tool: &str) {
        fs::write(
            path.join("package.xml"),
            format!(
                r#"<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>test</description>
  <maintainer email="test@example.com">test</maintainer>
  <license>MIT</license>
  <buildtool_depend>{build_tool}</buildtool_depend>
</package>"#
            ),
        )
        .unwrap();
    }

    #[test]
    fn verify_deduplicate_packages() {
        // Wow I am so upset, I thought I was going insane
        // std::Vec::dedup_by only removes *consecutive* elements that are equal
        let packages = vec![
            utils::Package {
                name: "diagnostic_msgs".into(),
                path: "/opt/ros/noetic/share/diagnostic_msgs".into(),
                version: Some(utils::RosVersion::ROS1),
            },
            utils::Package {
                name: "std_msgs".into(),
                path: "/tmp/std_msgs".into(),
                version: Some(utils::RosVersion::ROS1),
            },
            // This duplicate below should be removed
            utils::Package {
                name: "diagnostic_msgs".into(),
                path: "/code/assets/ros1_common_interfaces/common_msgs/diagnostic_msgs".into(),
                version: Some(utils::RosVersion::ROS1),
            },
            // This will be kept because the ROS Version is different
            utils::Package {
                name: "std_msgs".into(),
                path: "/ros2/std_msgs".into(),
                version: Some(utils::RosVersion::ROS2),
            },
        ];

        let deduplicated = utils::deduplicate_packages(packages);
        assert_eq!(deduplicated.len(), 3);
    }

    #[test]
    fn discovers_ros2_packages_from_ament_resource_index() {
        let prefix = temp_test_dir("ament_index");
        let index = prefix.join("share/ament_index/resource_index/packages");
        let package = prefix.join("share/example_interfaces");
        let malformed_package = prefix.join("share/missing_package");
        fs::create_dir_all(&index).unwrap();
        fs::create_dir_all(&package).unwrap();
        fs::write(index.join("example_interfaces"), "").unwrap();
        fs::write(index.join("missing_package"), "").unwrap();
        write_package_xml(&package, "example_interfaces", "ament_cmake");

        let package_paths = utils::get_ros2_search_paths_from_prefixes(&[&prefix]);
        assert_eq!(package_paths, vec![package]);
        assert!(!package_paths.contains(&malformed_package));

        fs::remove_dir_all(prefix).unwrap();
    }

    #[test]
    fn crawl_respects_ros2_ignore_markers() {
        for ignore_file in ["AMENT_IGNORE", "COLCON_IGNORE"] {
            let root = temp_test_dir(ignore_file);
            let ignored_package = root.join("ignored_package");
            fs::create_dir_all(&ignored_package).unwrap();
            fs::write(ignored_package.join(ignore_file), "").unwrap();
            write_package_xml(&ignored_package, "ignored_package", "ament_cmake");

            let packages = utils::crawl(&[&root]);
            assert!(packages.is_empty());

            fs::remove_dir_all(root).unwrap();
        }
    }

    #[test]
    fn parses_ament_python_packages_as_ros2() {
        let root = temp_test_dir("ament_python");
        let package = root.join("python_package");
        fs::create_dir_all(&package).unwrap();
        write_package_xml(&package, "python_package", "ament_python");

        let packages = utils::crawl(&[&root]);
        assert_eq!(packages.len(), 1);
        assert_eq!(packages[0].version, Some(utils::RosVersion::ROS2));

        fs::remove_dir_all(root).unwrap();
    }
}
