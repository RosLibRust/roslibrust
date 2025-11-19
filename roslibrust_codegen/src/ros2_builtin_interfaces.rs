//! Module contains messages which we consider "fundamental" to proper functioning of this crate
//! We don't want to rely on any ROS2 installation, but these types are so fundamental
//! That we want to ship them with the crate.

use std::collections::BTreeMap;

use crate::{parse::parse_ros_message_file, MessageFile};

/// Raw contents of ServiceEventInfo.msg
/// This message was deemed to be "too nessecary" to leave as a dependency.
/// It is required for ROS2 hash calculations on services, and as such is
/// treated like an integral type like Time, Duration.
///
/// If this message changes in the future it will require a codegen update.
const SERVICE_EVENT_INFO: &str = r##"
uint8 REQUEST_SENT = 0
uint8 REQUEST_RECEIVED = 1
uint8 RESPONSE_SENT = 2
uint8 RESPONSE_RECEIVED = 3

# The type of event this message represents
uint8 event_type

# Timestamp for when the event occurred (sent or received time)
builtin_interfaces/Time stamp

# Unique identifier for the client that sent the service request
# Note, this is only unique for the current session.
# The size here has to match the size of rmw_dds_common/msg/Gid,
# but unfortunately we cannot use that message directly due to a
# circular dependency.
char[16] client_gid

# Sequence number for the request
# Combined with the client ID, this creates a unique ID for the service transaction
int64 sequence_number
"##;

// Warning: this is a space at the end of the "The nanonseconds component" line
// Don't remove it, to match message definition exactly!
const TIME_MSG: &str = r##"
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
"##;

// Warning: this is a space at the end of the "The nanonseconds component" line
// Don't remove it, to match message definition exactly!
const DURATION_MSG: &str = r##"
# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
"##;

pub fn get_builtin_interfaces() -> BTreeMap<String, MessageFile> {
    let mut map = BTreeMap::new();

    // Time
    let parsed = parse_ros_message_file(
        TIME_MSG,
        "Time",
        &crate::utils::Package {
            name: "builtin_interfaces".to_string(),
            path: "/tmp".into(),
            version: Some(crate::utils::RosVersion::ROS2),
        },
        std::path::Path::new("/tmp/msg/Time.msg"),
    )
    .expect("Internal representation of Time is invalid");
    let time_msg =
        MessageFile::resolve(parsed, &map).expect("Internal representation of Time is invalid");
    map.insert(time_msg.get_full_name(), time_msg);

    // Duration
    let parsed = parse_ros_message_file(
        DURATION_MSG,
        "Duration",
        &crate::utils::Package {
            name: "builtin_interfaces".to_string(),
            path: "/tmp".into(),
            version: Some(crate::utils::RosVersion::ROS2),
        },
        std::path::Path::new("/tmp/msg/Duration.msg"),
    )
    .expect("Internal representation of Duration is invalid");
    let duration_msg =
        MessageFile::resolve(parsed, &map).expect("Internal representation of Duration is invalid");
    map.insert(duration_msg.get_full_name(), duration_msg);

    // ServiceEventInfo
    let parsed = parse_ros_message_file(
        SERVICE_EVENT_INFO,
        "ServiceEventInfo",
        &crate::utils::Package {
            name: "service_msgs".to_string(),
            path: "/tmp".into(),
            version: Some(crate::utils::RosVersion::ROS2),
        },
        std::path::Path::new("/tmp/msg/ServiceEventInfo.msg"),
    )
    .expect("Internal representation of ServiceEventInfo is invalid");
    let service_event_info = MessageFile::resolve(parsed, &map)
        .expect("Internal representation of ServiceEventInfo is invalid");
    map.insert(service_event_info.get_full_name(), service_event_info);

    map
}
