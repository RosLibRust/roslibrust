#[allow(unused_imports)]
pub mod actionlib_msgs {
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GoalID {
        pub r#stamp: builtin_interfaces::Time,
        pub r#id: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for GoalID {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalID";
        const MD5SUM: &'static str = "9736c630ae528bce23de03af421dbf7d";
        const DEFINITION: &'static str = r####"# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_140ea4a1bad4781edc32030c7983ab3a62c675f861c61c9b2200ae5e5398be24";
        const ROS2_TYPE_NAME: &'static str = "actionlib_msgs::msg::dds_::GoalID_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GoalStatus {
        pub r#goal_id: self::GoalID,
        pub r#status: u8,
        pub r#text: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for GoalStatus {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatus";
        const MD5SUM: &'static str = "f6807dda6aa81059dd13096c960395d1";
        const DEFINITION: &'static str = r####"GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server.
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server.
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State).
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
                            #   (Terminal State).
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State).
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State).
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution.
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but
                            #    the action server has not yet confirmed that the goal is canceled.
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State).
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not
                            #    be sent over the wire by an action server.

# Allow for the user to associate a string with GoalStatus for debugging.
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_c90c64230954d90ee079cbdd22a14bca3650b3d403f5cc3faf0eee992c6ae9ff";
        const ROS2_TYPE_NAME: &'static str = "actionlib_msgs::msg::dds_::GoalStatus_";
    }
    #[allow(unused)]
    impl GoalStatus {
        pub const r#PENDING: u8 = 0u8;
        pub const r#ACTIVE: u8 = 1u8;
        pub const r#PREEMPTED: u8 = 2u8;
        pub const r#SUCCEEDED: u8 = 3u8;
        pub const r#ABORTED: u8 = 4u8;
        pub const r#REJECTED: u8 = 5u8;
        pub const r#PREEMPTING: u8 = 6u8;
        pub const r#RECALLING: u8 = 7u8;
        pub const r#RECALLED: u8 = 8u8;
        pub const r#LOST: u8 = 9u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GoalStatusArray {
        pub r#header: std_msgs::Header,
        pub r#status_list: ::std::vec::Vec<self::GoalStatus>,
    }
    impl ::roslibrust::RosMessageType for GoalStatusArray {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatusArray";
        const MD5SUM: &'static str = "9acbf917d56df306cdf7077c20a1e7de";
        const DEFINITION: &'static str = r####"# Stores the statuses for goals that are currently being tracked
# by an action server
std_msgs/Header header
GoalStatus[] status_list
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server.
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server.
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State).
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
                            #   (Terminal State).
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State).
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State).
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution.
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but
                            #    the action server has not yet confirmed that the goal is canceled.
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State).
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not
                            #    be sent over the wire by an action server.

# Allow for the user to associate a string with GoalStatus for debugging.
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f3230208e8daec53eb7b8da6df1c1d1d4eb095f3082be922661c29a1f855a1bc";
        const ROS2_TYPE_NAME: &'static str = "actionlib_msgs::msg::dds_::GoalStatusArray_";
    }
}
#[allow(unused_imports)]
pub mod builtin_interfaces {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Duration {
        pub r#sec: i32,
        pub r#nanosec: u32,
    }
    impl ::roslibrust::RosMessageType for Duration {
        const ROS_TYPE_NAME: &'static str = "builtin_interfaces/Duration";
        const MD5SUM: &'static str = "8255142433c342f21ece78aae48f7907";
        const DEFINITION: &'static str = r####"# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e8d009f659816f758b75334ee1a9ca5b5c0b859843261f14c7f937349599d93b";
        const ROS2_TYPE_NAME: &'static str = "builtin_interfaces::msg::dds_::Duration_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Time {
        pub r#sec: i32,
        pub r#nanosec: u32,
    }
    impl ::roslibrust::RosMessageType for Time {
        const ROS_TYPE_NAME: &'static str = "builtin_interfaces/Time";
        const MD5SUM: &'static str = "8255142433c342f21ece78aae48f7907";
        const DEFINITION: &'static str = r####"# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197";
        const ROS2_TYPE_NAME: &'static str = "builtin_interfaces::msg::dds_::Time_";
    }
}
#[allow(unused_imports)]
pub mod diagnostic_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct DiagnosticArray {
        pub r#header: std_msgs::Header,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust::RosMessageType for DiagnosticArray {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
        const MD5SUM: &'static str = "a56c9b8d61df740404e365b0e1155dfe";
        const DEFINITION: &'static str = r####"# This message is used to send diagnostic information about the state of the robot.
std_msgs/Header header # for timestamp
DiagnosticStatus[] status # an array of components being reported on
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f9eef075d53ad59209151268b37ab3191f7a854417107bb76ff9d56c4bfee92c";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::DiagnosticArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct DiagnosticStatus {
        pub r#level: u8,
        pub r#name: ::std::string::String,
        pub r#message: ::std::string::String,
        pub r#hardware_id: ::std::string::String,
        pub r#values: ::std::vec::Vec<self::KeyValue>,
    }
    impl ::roslibrust::RosMessageType for DiagnosticStatus {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticStatus";
        const MD5SUM: &'static str = "d0ce08bc6e5ba34c7754f563a9cabaf1";
        const DEFINITION: &'static str = r####"# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value"####;
        const ROS2_HASH: &'static str =
            "RIHS01_405fee4fdad6cce5341d944ac782eb849e414027bbe672cafff502302f03802b";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::DiagnosticStatus_";
    }
    #[allow(unused)]
    impl DiagnosticStatus {
        pub const r#OK: u8 = 0u8;
        pub const r#WARN: u8 = 1u8;
        pub const r#ERROR: u8 = 2u8;
        pub const r#STALE: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct KeyValue {
        pub r#key: ::std::string::String,
        pub r#value: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for KeyValue {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/KeyValue";
        const MD5SUM: &'static str = "cf57fdc6617a881a88c16e768132149c";
        const DEFINITION: &'static str = r####"# What to label this value when viewing.
string key
# A value to track over time.
string value"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d68081eaa540288c5440753baecef0c4e16e81a5f78ad68902ded5100413bb42";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::KeyValue_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AddDiagnosticsRequest {
        pub r#load_namespace: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for AddDiagnosticsRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsRequest";
        const MD5SUM: &'static str = "c26cf6e164288fbc6050d74f838bcdf0";
        const DEFINITION: &'static str = r####"# This service is used as part of the process for loading analyzers at runtime,
# and should be used by a loader script or program, not as a standalone service.
# Information about dynamic addition of analyzers can be found at
# http://wiki.ros.org/diagnostics/Tutorials/Adding%20Analyzers%20at%20Runtime

# The load_namespace parameter defines the namespace where parameters for the
# initialization of analyzers in the diagnostic aggregator have been loaded. The
# value should be a global name (i.e. /my/name/space), not a relative
# (my/name/space) or private (~my/name/space) name. Analyzers will not be added
# if a non-global name is used. The call will also fail if the namespace
# contains parameters that follow a namespace structure that does not conform to
# that expected by the analyzer definitions. See
# http://wiki.ros.org/diagnostics/Tutorials/Configuring%20Diagnostic%20Aggregators
# and http://wiki.ros.org/diagnostics/Tutorials/Using%20the%20GenericAnalyzer
# for examples of the structure of yaml files which are expected to have been
# loaded into the namespace.
string load_namespace"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3375e04f9f4d406c7b3c8aaa016a29af2690fcad4fa6007f46c221c33f8c82d4";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::AddDiagnosticsRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AddDiagnosticsResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for AddDiagnosticsResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsResponse";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const DEFINITION: &'static str = r####"# True if diagnostic aggregator was updated with new diagnostics, False
# otherwise. A false return value means that either there is a bond in the
# aggregator which already used the requested namespace, or the initialization
# of analyzers failed.
bool success

# Message with additional information about the success or failure
string message"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e3aa26757bf712cd2c48e94f1201070d8dab401d1cd0d8853ab0f4579dca5eac";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::AddDiagnosticsResponse_";
    }
    #[allow(dead_code)]
    pub struct AddDiagnostics {}
    impl ::roslibrust::RosServiceType for AddDiagnostics {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/AddDiagnostics";
        const MD5SUM: &'static str = "e6ac9bbde83d0d3186523c3687aecaee";
        const ROS2_HASH: &'static str =
            "RIHS01_6b91084c9f8bffd9b0c1f6883e0e8e491a831f4fc6c6aec77a4efaf3f1ea90b5";
        type Request = AddDiagnosticsRequest;
        type Response = AddDiagnosticsResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SelfTestRequest {}
    impl ::roslibrust::RosMessageType for SelfTestRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####""####;
        const ROS2_HASH: &'static str =
            "RIHS01_f3da6cba8155ce26bb81b029b63bb311073c6cb4183848a37cc16efe3d77b229";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::SelfTestRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SelfTestResponse {
        pub r#id: ::std::string::String,
        pub r#passed: u8,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust::RosMessageType for SelfTestResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestResponse";
        const MD5SUM: &'static str = "ac21b1bab7ab17546986536c22eb34e9";
        const DEFINITION: &'static str = r####"string id
byte passed
DiagnosticStatus[] status
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value"####;
        const ROS2_HASH: &'static str =
            "RIHS01_4b6dd70ee9f20cebad759331530fd0de79bcf3dc8b6372783ef935366bc4ce46";
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::SelfTestResponse_";
    }
    #[allow(dead_code)]
    pub struct SelfTest {}
    impl ::roslibrust::RosServiceType for SelfTest {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/SelfTest";
        const MD5SUM: &'static str = "ac21b1bab7ab17546986536c22eb34e9";
        const ROS2_HASH: &'static str =
            "RIHS01_0e0fc2754cadcf8dd57459806c78a7c3b31ddc65b011a630fd8ce3bcddc00583";
        type Request = SelfTestRequest;
        type Response = SelfTestResponse;
    }
}
#[allow(unused_imports)]
pub mod geometry_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Accel {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Accel {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Accel";
        const MD5SUM: &'static str = "9f195f881246fdfa2798d1d3eebca84a";
        const DEFINITION: &'static str = r####"# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_dc448243ded9b1fcbcca24aba0c22f013dae06c354ba2d849571c0a2a3f57ca0";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Accel_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AccelStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::Accel,
    }
    impl ::roslibrust::RosMessageType for AccelStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelStamped";
        const MD5SUM: &'static str = "714bbd985d9cfc562b1eb5aaa96c7be0";
        const DEFINITION: &'static str = r####"# An accel with reference coordinate frame and timestamp
std_msgs/Header header
Accel accel
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_ef1df9eabae0a708cc049a061ebcddc4e2a5f745730100ba680e086a9698b165";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::AccelStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AccelWithCovariance {
        pub r#accel: self::Accel,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust::codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust::RosMessageType for AccelWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovariance";
        const MD5SUM: &'static str = "ad5a718d699c6be72a02b8d6a139f334";
        const DEFINITION: &'static str = r####"# This expresses acceleration in free space with uncertainty.

Accel accel

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_230d51bd53bc36f260574e73b42941cefe44684753480b6fc330c032c5db5997";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::AccelWithCovariance_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AccelWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::AccelWithCovariance,
    }
    impl ::roslibrust::RosMessageType for AccelWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovarianceStamped";
        const MD5SUM: &'static str = "85dc4e3c87dd8539dc32ab10fd048673";
        const DEFINITION: &'static str = r####"# This represents an estimated accel with reference coordinate frame and timestamp.
std_msgs/Header header
AccelWithCovariance accel
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/AccelWithCovariance
# This expresses acceleration in free space with uncertainty.

Accel accel

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_61c9ad8928e71dd95ce791b2f02809ee2a0bbcc42cd0e4047fd00a822a08e444";
        const ROS2_TYPE_NAME: &'static str =
            "geometry_msgs::msg::dds_::AccelWithCovarianceStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Inertia {
        pub r#m: f64,
        pub r#com: self::Vector3,
        pub r#ixx: f64,
        pub r#ixy: f64,
        pub r#ixz: f64,
        pub r#iyy: f64,
        pub r#iyz: f64,
        pub r#izz: f64,
    }
    impl ::roslibrust::RosMessageType for Inertia {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Inertia";
        const MD5SUM: &'static str = "1d26e4bb6c83ff141c5cf0d883c2b0fe";
        const DEFINITION: &'static str = r####"# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_2ddd5dab5c347825ba2e56c895ddccfd0b8efe53ae931bf67f905529930b4bd7";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Inertia_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InertiaStamped {
        pub r#header: std_msgs::Header,
        pub r#inertia: self::Inertia,
    }
    impl ::roslibrust::RosMessageType for InertiaStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/InertiaStamped";
        const MD5SUM: &'static str = "e5be6ffabc1d75f61d3105f562e8bc87";
        const DEFINITION: &'static str = r####"# An Inertia with a time stamp and reference frame.

std_msgs/Header header
Inertia inertia
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Inertia
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_766be45976252babf7f9d8ac4ae7c912a7ceccf71035622529f27518b695aa09";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::InertiaStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Point {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    impl ::roslibrust::RosMessageType for Point {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point";
        const MD5SUM: &'static str = "4a842b65f413084dc2b10fb484ea7f17";
        const DEFINITION: &'static str = r####"# This contains the position of a point in free space
float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Point_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Point32 {
        pub r#x: f32,
        pub r#y: f32,
        pub r#z: f32,
    }
    impl ::roslibrust::RosMessageType for Point32 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point32";
        const MD5SUM: &'static str = "cc153912f1453b708d221682bc23d9ac";
        const DEFINITION: &'static str = r####"# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_2fc4db7cae16a4582c79a56b66173a8d48d52c7dc520ddc55a0d4bcf2a4bfdbc";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Point32_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PointStamped {
        pub r#header: std_msgs::Header,
        pub r#point: self::Point,
    }
    impl ::roslibrust::RosMessageType for PointStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PointStamped";
        const MD5SUM: &'static str = "3dc055656bed5c4bb0657a41c8d46c59";
        const DEFINITION: &'static str = r####"# This represents a Point with reference coordinate frame and timestamp

std_msgs/Header header
Point point
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_4c0296af86e01e562e9e0405d138a01537247580076c58ea38d7923ac1045897";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::PointStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Polygon {
        pub r#points: ::std::vec::Vec<self::Point32>,
    }
    impl ::roslibrust::RosMessageType for Polygon {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Polygon";
        const MD5SUM: &'static str = "cd60a26494a087f577976f0329fa120e";
        const DEFINITION: &'static str = r####"# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3782f9f0bf044964d692d6c017d705e37611afb1f0bf6a9dee248a7dda0f784a";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Polygon_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PolygonStamped {
        pub r#header: std_msgs::Header,
        pub r#polygon: self::Polygon,
    }
    impl ::roslibrust::RosMessageType for PolygonStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PolygonStamped";
        const MD5SUM: &'static str = "66ae3598ba9dd610ba674198e61d8fa7";
        const DEFINITION: &'static str = r####"# This represents a Polygon with reference coordinate frame and timestamp

std_msgs/Header header
Polygon polygon
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: geometry_msgs/Polygon
# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b7cf07932f1523d4b4088075945c1a0141f7cd21da87cc940fc61652e9138b46";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::PolygonStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Pose {
        pub r#position: self::Point,
        pub r#orientation: self::Quaternion,
    }
    impl ::roslibrust::RosMessageType for Pose {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose";
        const MD5SUM: &'static str = "e45d45a5a1ce597b249e23fb30fc871f";
        const DEFINITION: &'static str = r####"# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d501954e9476cea2996984e812054b68026ae0bfae789d9a10b23daf35cc90fa";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Pose_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Pose2D {
        pub r#x: f64,
        pub r#y: f64,
        pub r#theta: f64,
    }
    impl ::roslibrust::RosMessageType for Pose2D {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose2D";
        const MD5SUM: &'static str = "938fa65709584ad8e77d238529be13b8";
        const DEFINITION: &'static str = r####"# Deprecated as of Foxy and will potentially be removed in any following release.
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d68efa5b46e70f7b16ca23085474fdac5a44b638783ec42f661da64da4724ccc";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Pose2D_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PoseArray {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<self::Pose>,
    }
    impl ::roslibrust::RosMessageType for PoseArray {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseArray";
        const MD5SUM: &'static str = "8f0f2bef738a5217c5210a25031dc811";
        const DEFINITION: &'static str = r####"# An array of poses with a header for global reference.

std_msgs/Header header

Pose[] poses
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_af0cc36d190e104d546d168d6b39df04fa4b4ccecf59cb4c9ed328d3d5004aa0";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::PoseArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PoseStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::Pose,
    }
    impl ::roslibrust::RosMessageType for PoseStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseStamped";
        const MD5SUM: &'static str = "5a783a9c53de9dbde905b1a8e1d332a6";
        const DEFINITION: &'static str = r####"# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::PoseStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PoseWithCovariance {
        pub r#pose: self::Pose,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust::codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust::RosMessageType for PoseWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovariance";
        const MD5SUM: &'static str = "c23e848cf1b7533a8d7c259073a97e6f";
        const DEFINITION: &'static str = r####"# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"####;
        const ROS2_HASH: &'static str =
            "RIHS01_9a7c0fd234b7f45c6098745ecccd773ca1085670e64107135397aee31c02e1bb";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::PoseWithCovariance_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PoseWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::PoseWithCovariance,
    }
    impl ::roslibrust::RosMessageType for PoseWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovarianceStamped";
        const MD5SUM: &'static str = "c693703aeb65d31536e9f01ea76841e4";
        const DEFINITION: &'static str = r####"# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
PoseWithCovariance pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_26432f9803e43727d3c8f668d1fdb3c630f548af631e2f4e31382371bfea3b6e";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::PoseWithCovarianceStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Quaternion {
        #[default(0f64)]
        pub r#x: f64,
        #[default(0f64)]
        pub r#y: f64,
        #[default(0f64)]
        pub r#z: f64,
        #[default(1f64)]
        pub r#w: f64,
    }
    impl ::roslibrust::RosMessageType for Quaternion {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Quaternion";
        const MD5SUM: &'static str = "a779879fadf0160734f906b8c19c7004";
        const DEFINITION: &'static str = r####"# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"####;
        const ROS2_HASH: &'static str =
            "RIHS01_8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Quaternion_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct QuaternionStamped {
        pub r#header: std_msgs::Header,
        pub r#quaternion: self::Quaternion,
    }
    impl ::roslibrust::RosMessageType for QuaternionStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/QuaternionStamped";
        const MD5SUM: &'static str = "6e3f7a64b8b509235ca9dc617a34bf93";
        const DEFINITION: &'static str = r####"# This represents an orientation with reference coordinate frame and timestamp.

std_msgs/Header header
Quaternion quaternion
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_381add86c6c3160644d228ca342182c7fd6c7fab11c7a85ad817a9cc22dbac6e";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::QuaternionStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Transform {
        pub r#translation: self::Vector3,
        pub r#rotation: self::Quaternion,
    }
    impl ::roslibrust::RosMessageType for Transform {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Transform";
        const MD5SUM: &'static str = "ac9eff44abf714214112b05d54a3cf9b";
        const DEFINITION: &'static str = r####"# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_beb83fbe698636351461f6f35d1abb20010c43d55374d81bd041f1ba2581fddc";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Transform_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TransformStamped {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#transform: self::Transform,
    }
    impl ::roslibrust::RosMessageType for TransformStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TransformStamped";
        const MD5SUM: &'static str = "c555e99d9d1f7097738c8861133b3563";
        const DEFINITION: &'static str = r####"# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="https://index.ros.org/p/tf2/">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communicate the full reference for the transform
# in a self contained message.

# The frame id in the header is used as the reference frame of this transform.
std_msgs/Header header

# The frame id of the child frame to which this transform points.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
Transform transform
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_0a241f87d04668d94099cbb5ba11691d5ad32c2f29682e4eb5653424bd275206";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::TransformStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Twist {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Twist {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Twist";
        const MD5SUM: &'static str = "9f195f881246fdfa2798d1d3eebca84a";
        const DEFINITION: &'static str = r####"# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Twist_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TwistStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::Twist,
    }
    impl ::roslibrust::RosMessageType for TwistStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistStamped";
        const MD5SUM: &'static str = "7c6000d4f3aafa80eaf2471ce8172e85";
        const DEFINITION: &'static str = r####"# A twist with reference coordinate frame and timestamp

std_msgs/Header header
Twist twist
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_5f0fcd4f81d5d06ad9b4c4c63e3ea51b82d6ae4d0558f1d475229b1121db6f64";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::TwistStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TwistWithCovariance {
        pub r#twist: self::Twist,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust::codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust::RosMessageType for TwistWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovariance";
        const MD5SUM: &'static str = "1fe8a28e6890a4cc3ae4c3ca5c7d82e6";
        const DEFINITION: &'static str = r####"# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_49f574f033f095d8b6cd1beaca5ca7925e296e84af1716d16c89d38b059c8c18";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::TwistWithCovariance_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TwistWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::TwistWithCovariance,
    }
    impl ::roslibrust::RosMessageType for TwistWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovarianceStamped";
        const MD5SUM: &'static str = "38d53b26c78ee3db59b20ff3e8127275";
        const DEFINITION: &'static str = r####"# This represents an estimated twist with reference coordinate frame and timestamp.

std_msgs/Header header
TwistWithCovariance twist
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_77b67434531e6529b7a0091357b186b6ebdb17fd9ffd3e0c7ce9d3fb11a44563";
        const ROS2_TYPE_NAME: &'static str =
            "geometry_msgs::msg::dds_::TwistWithCovarianceStamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Vector3 {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    impl ::roslibrust::RosMessageType for Vector3 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3";
        const MD5SUM: &'static str = "4a842b65f413084dc2b10fb484ea7f17";
        const DEFINITION: &'static str = r####"# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Vector3_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Vector3Stamped {
        pub r#header: std_msgs::Header,
        pub r#vector: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Vector3Stamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3Stamped";
        const MD5SUM: &'static str = "13595660ee13403f23a0a2a0da07aa81";
        const DEFINITION: &'static str = r####"# This represents a Vector3 with reference coordinate frame and timestamp

# Note that this follows vector semantics with it always anchored at the origin,
# so the rotational elements of a transform are the only parts applied when transforming.

std_msgs/Header header
Vector3 vector
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d4829622288cbb443886e7ea94ea5671a3b1be6bab4ad04224432a65f7d7887a";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Vector3Stamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Wrench {
        pub r#force: self::Vector3,
        pub r#torque: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Wrench {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Wrench";
        const MD5SUM: &'static str = "4f539cf138b23283b520fd271b567936";
        const DEFINITION: &'static str = r####"# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Wrench_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct WrenchStamped {
        pub r#header: std_msgs::Header,
        pub r#wrench: self::Wrench,
    }
    impl ::roslibrust::RosMessageType for WrenchStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/WrenchStamped";
        const MD5SUM: &'static str = "c7f9621a9f5013d52e82cd3c5d14f0fa";
        const DEFINITION: &'static str = r####"# A wrench with reference coordinate frame and timestamp

std_msgs/Header header
Wrench wrench
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_8dc3deaf06b2ab281f9f9a742a8961c328ca7cec16e3fd6586d3a5c83fa78f77";
        const ROS2_TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::WrenchStamped_";
    }
}
#[allow(unused_imports)]
pub mod nav_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GridCells {
        pub r#header: std_msgs::Header,
        pub r#cell_width: f32,
        pub r#cell_height: f32,
        pub r#cells: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust::RosMessageType for GridCells {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GridCells";
        const MD5SUM: &'static str = "7843d58b18b7ea989085689e202af1ea";
        const DEFINITION: &'static str = r####"# An array of cells in a 2D grid

std_msgs/Header header

# Width of each cell
float32 cell_width

# Height of each cell
float32 cell_height

# Each cell is represented by the Point at the center of the cell
geometry_msgs/Point[] cells
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_bb99c2f5d0a04750745a81ec6a8147aa373cce5bd17c8cd6507f2413354a6933";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::GridCells_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MapMetaData {
        pub r#map_load_time: builtin_interfaces::Time,
        pub r#resolution: f32,
        pub r#width: u32,
        pub r#height: u32,
        pub r#origin: geometry_msgs::Pose,
    }
    impl ::roslibrust::RosMessageType for MapMetaData {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/MapMetaData";
        const MD5SUM: &'static str = "f79bb70ae15b6d55a5b3e44724cab637";
        const DEFINITION: &'static str = r####"# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"####;
        const ROS2_HASH: &'static str =
            "RIHS01_2772d4b2000ef2b35dbaeb80fd3946c1369f817fb4f75677d916d27c17d763c8";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::MapMetaData_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct OccupancyGrid {
        pub r#header: std_msgs::Header,
        pub r#info: self::MapMetaData,
        pub r#data: ::std::vec::Vec<i8>,
    }
    impl ::roslibrust::RosMessageType for OccupancyGrid {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/OccupancyGrid";
        const MD5SUM: &'static str = "0ba2e2886c6391b606d4d0c367f70533";
        const DEFINITION: &'static str = r####"# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_8d348150c12913a31ee0ec170fbf25089e4745d17035792a1ba94d6f0bc0cfc7";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::OccupancyGrid_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Odometry {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#pose: geometry_msgs::PoseWithCovariance,
        pub r#twist: geometry_msgs::TwistWithCovariance,
    }
    impl ::roslibrust::RosMessageType for Odometry {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Odometry";
        const MD5SUM: &'static str = "7d13133659beedef5fbafd61a6288fbe";
        const DEFINITION: &'static str = r####"# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::Odometry_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Path {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<geometry_msgs::PoseStamped>,
    }
    impl ::roslibrust::RosMessageType for Path {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Path";
        const MD5SUM: &'static str = "8ca90f27f5084414e7f87268323b896e";
        const DEFINITION: &'static str = r####"# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_1957a5bb3cee5da65c4e52e52b65a93df227efce4c20f8458b36e73066ca334b";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::Path_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GetMapRequest {}
    impl ::roslibrust::RosMessageType for GetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####"# Get the map as a nav_msgs/OccupancyGrid"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f1fcfb164bf2ca24fbc4fb5108adff607f4012b929a1b1f0e3cc77ecdf8e02de";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::GetMapRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GetMapResponse {
        pub r#map: self::OccupancyGrid,
    }
    impl ::roslibrust::RosMessageType for GetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapResponse";
        const MD5SUM: &'static str = "7bff8b2620dbaeb0925e81085fcfa292";
        const DEFINITION: &'static str = r####"# The current map hosted by this map service.
OccupancyGrid map
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f5c00d1fff0f1f5fd05b31dfc0be85343756a93daeb1ff0d628194dd455f092a";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::GetMapResponse_";
    }
    #[allow(dead_code)]
    pub struct GetMap {}
    impl ::roslibrust::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetMap";
        const MD5SUM: &'static str = "7bff8b2620dbaeb0925e81085fcfa292";
        const ROS2_HASH: &'static str =
            "RIHS01_c8ae77c9995b3554b5ba80e4d4d443f970ac65143102a1d893ec24fc07b31147";
        type Request = GetMapRequest;
        type Response = GetMapResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GetPlanRequest {
        pub r#start: geometry_msgs::PoseStamped,
        pub r#goal: geometry_msgs::PoseStamped,
        pub r#tolerance: f32,
    }
    impl ::roslibrust::RosMessageType for GetPlanRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanRequest";
        const MD5SUM: &'static str = "644acb28fec068b394946382ed4f83cd";
        const DEFINITION: &'static str = r####"# Get a plan from the current position to the goal Pose

# The start pose for the plan
geometry_msgs/PoseStamped start

# The final pose of the goal position
geometry_msgs/PoseStamped goal

# If the goal is obstructed, how many meters the planner can
# relax the constraint in x and y before failing.
float32 tolerance
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_85679b70d98dc86c35c702890146936e2c2c4bfc3325d34ba04f9c2413a079e2";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::GetPlanRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GetPlanResponse {
        pub r#plan: self::Path,
    }
    impl ::roslibrust::RosMessageType for GetPlanResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanResponse";
        const MD5SUM: &'static str = "70fd00e01a6f927e3b650a2f434c1c82";
        const DEFINITION: &'static str = r####"# Array of poses from start to goal if one was successfully found.
Path plan
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/Path
# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_73000a494f77a4bd93ee68d49e8903aa938575ed68737f665939fc7e77915912";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::GetPlanResponse_";
    }
    #[allow(dead_code)]
    pub struct GetPlan {}
    impl ::roslibrust::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetPlan";
        const MD5SUM: &'static str = "a08000c8d6c102a04c555fedbdfbde12";
        const ROS2_HASH: &'static str =
            "RIHS01_234f7aff100f5edb8150366601687b027bcdc253db47decb88fff846193fe5e8";
        type Request = GetPlanRequest;
        type Response = GetPlanResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct LoadMapRequest {
        pub r#map_url: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for LoadMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapRequest";
        const MD5SUM: &'static str = "3813ba1ae85fbcd4dc88c90f1426b90b";
        const DEFINITION: &'static str = r####"# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b2991c5afe57c1bcfd58ad47bd327d40a9dddccfef304d05ad06e26cf26d19bf";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::LoadMapRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct LoadMapResponse {
        pub r#map: self::OccupancyGrid,
        pub r#result: u8,
    }
    impl ::roslibrust::RosMessageType for LoadMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapResponse";
        const MD5SUM: &'static str = "6717cafc2c34d2168070ef42cff41356";
        const DEFINITION: &'static str = r####"# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255

# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e3521ab9d783391191ef95820ddfbf8bf4d43d5755bfe4fbecd0b527cdd57c50";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::LoadMapResponse_";
    }
    #[allow(unused)]
    impl LoadMapResponse {
        pub const r#RESULT_SUCCESS: u8 = 0u8;
        pub const r#RESULT_MAP_DOES_NOT_EXIST: u8 = 1u8;
        pub const r#RESULT_INVALID_MAP_DATA: u8 = 2u8;
        pub const r#RESULT_INVALID_MAP_METADATA: u8 = 3u8;
        pub const r#RESULT_UNDEFINED_FAILURE: u8 = 255u8;
    }
    #[allow(dead_code)]
    pub struct LoadMap {}
    impl ::roslibrust::RosServiceType for LoadMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/LoadMap";
        const MD5SUM: &'static str = "7631a819bf39865de4324e3d28f5e85f";
        const ROS2_HASH: &'static str =
            "RIHS01_1a192ac56c40fed2767dac26f0b371785372276bd465c902676d2dca135aae5a";
        type Request = LoadMapRequest;
        type Response = LoadMapResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SetMapRequest {
        pub r#map: self::OccupancyGrid,
        pub r#initial_pose: geometry_msgs::PoseWithCovarianceStamped,
    }
    impl ::roslibrust::RosMessageType for SetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapRequest";
        const MD5SUM: &'static str = "1e9a3a1c4507e1dc505cae231b9f6914";
        const DEFINITION: &'static str = r####"# Set a new map together with an initial pose

# Requested 2D map to be set.
nav_msgs/OccupancyGrid map

# Estimated initial pose when setting new map.
geometry_msgs/PoseWithCovarianceStamped initial_pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
PoseWithCovariance pose
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_c83b83e89b0c4e6d7135c5b58de3c956d7401cc1039df88db46346eea06811ea";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::SetMapRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SetMapResponse {
        pub r#success: bool,
    }
    impl ::roslibrust::RosMessageType for SetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapResponse";
        const MD5SUM: &'static str = "358e233cde0c8a8bcfea4ce193f8fc15";
        const DEFINITION: &'static str = r####"# True if the map was successfully set, false otherwise.
bool success"####;
        const ROS2_HASH: &'static str =
            "RIHS01_9f71efa1566ab7039b4616b17fc9c404ef05dd558eea124c042ce0cdc368af26";
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::msg::dds_::SetMapResponse_";
    }
    #[allow(dead_code)]
    pub struct SetMap {}
    impl ::roslibrust::RosServiceType for SetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/SetMap";
        const MD5SUM: &'static str = "2ce17df740c0777cc84f0a6673c7ef43";
        const ROS2_HASH: &'static str =
            "RIHS01_5e11a5b2ca53d8ae85b666a019f16c9904ebc787828f1f566c4e048a1ddedfb4";
        type Request = SetMapRequest;
        type Response = SetMapResponse;
    }
}
#[allow(unused_imports)]
pub mod ros2_test_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Bool {
        pub r#data: bool,
    }
    impl ::roslibrust::RosMessageType for Bool {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/Bool";
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
        const DEFINITION: &'static str = r####"bool data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_14ae37d5c5f596ff013e8658e1af0c02f9dcad8f0d75e3d6243809c90264acd7";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::Bool_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct BoundedInt {
        #[serde(with = "::roslibrust::codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for BoundedInt {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/BoundedInt";
        const MD5SUM: &'static str = "0a00aaf35761a81662e7431cb0092a31";
        const DEFINITION: &'static str = r####"# As far as I can tell from documentation "<=" is the only valid syntax and "<" is not valid
uint8[<=5] data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_82105d57673153229ad6f8aa943ead090bc9756da35c9b73cf919a64bf902d6c";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::BoundedInt_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct BoundedReferenced {
        pub r#data: ::std::vec::Vec<self::BoundedInt>,
    }
    impl ::roslibrust::RosMessageType for BoundedReferenced {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/BoundedReferenced";
        const MD5SUM: &'static str = "64234c2669823716869cb915600b8262";
        const DEFINITION: &'static str = r####"# Proving bounding works on nested types
BoundedInt[<=3] data
================================================================================
MSG: ros2_test_msgs/BoundedInt
# As far as I can tell from documentation "<=" is the only valid syntax and "<" is not valid
uint8[<=5] data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f6a7e6732d79d9bfff7abc475f2dda336233f75d3b1984537d2acb07054c3975";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::BoundedReferenced_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct BoundedString {
        pub r#data: ::std::string::String,
        pub r#data_list: ::std::vec::Vec<::std::string::String>,
        pub r#data_matrix: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust::RosMessageType for BoundedString {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/BoundedString";
        const MD5SUM: &'static str = "b6c302077b544e8370b6a457e36b583b";
        const DEFINITION: &'static str = r####"# Proving bounding works on strings
string<=5 data
string<=5[] data_list
string<=5[<=3] data_matrix"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e9965b1be42dce770d8936688f88b2f79cc1a828daa2295f1224186fd3c85b70";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::BoundedString_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Char {
        pub r#data: u8,
        pub r#data_array: ::std::vec::Vec<u8>,
        pub r#data_bounded_array: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for Char {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/Char";
        const MD5SUM: &'static str = "85fb532b3badec4e87b20705068b150b";
        const DEFINITION: &'static str = r####"# Char is a slightly weird type
char data
char[] data_array
char[<=11] data_bounded_array"####;
        const ROS2_HASH: &'static str =
            "RIHS01_7398a047b7ae995eeb12bd58e9884a30f25968d9c19a79b929778bebde4e678a";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::Char_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Defaults {
        #[default(42u8)]
        pub r#x: u8,
        # [default (- 2000i16)]
        pub r#y: i16,
        #[default("John Doe")]
        pub r#full_name: ::std::string::String,
        #[default(_code = "vec![-200, -100, 0, 100, 200]")]
        pub r#samples: ::std::vec::Vec<i32>,
        #[default(_code = "vec![-200.0, -1.0, 0.0]")]
        pub r#f_samples: ::std::vec::Vec<f32>,
        #[default(_code = "[\"hello\", \"world\"].iter().map(|x| x.to_string()).collect()")]
        pub r#s_vec: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust::RosMessageType for Defaults {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/Defaults";
        const MD5SUM: &'static str = "43c441dc2b521c313f54affd982b5314";
        const DEFINITION: &'static str = r####"# This message is specifically for testing generating of default values
# Examples based on https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]

# More complicated examples to stress the system, floats with mixed precision
float32[] f_samples [-200, -1.0, 0]
string[] s_vec ["hello", "world"]
# This may or may not be valid ROS, it probably is, but we don't handle yet
# TODO handle this somehow
# string[] s_vec_2 ['hello', 'world']

# TODO ROS says this is valid, but we currently don't handle
#string single_quote 'Jane Doe'"####;
        const ROS2_HASH: &'static str =
            "RIHS01_9a76d2961fd0ba74223ec58b1cf9ab677a4136bc5dbe341b3376d687355474be";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::Defaults_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Stamped {
        pub r#stamp: builtin_interfaces::Time,
        pub r#duration: builtin_interfaces::Duration,
    }
    impl ::roslibrust::RosMessageType for Stamped {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/Stamped";
        const MD5SUM: &'static str = "26348416fe55846a953971fc1981822c";
        const DEFINITION: &'static str = r####"# Demo message to show time and duration work in ROS2 format
builtin_interfaces/Time stamp
builtin_interfaces/Duration duration
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d568324df43c14673c2d014c1c568d399b42e793d32050a4f396fd3cbd2f50cc";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::Stamped_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AddTwoIntsRequest {
        pub r#a: i64,
        pub r#b: i64,
    }
    impl ::roslibrust::RosMessageType for AddTwoIntsRequest {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/AddTwoIntsRequest";
        const MD5SUM: &'static str = "36d09b846be0b371c5f190354dd3153e";
        const DEFINITION: &'static str = r####"int64 a
int64 b"####;
        const ROS2_HASH: &'static str =
            "RIHS01_2ae9759e740cc2c8dfbcb5a3501338bf5ee5892271baf6f273d302ef3b8523fd";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::AddTwoIntsRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct AddTwoIntsResponse {
        pub r#sum: i64,
    }
    impl ::roslibrust::RosMessageType for AddTwoIntsResponse {
        const ROS_TYPE_NAME: &'static str = "ros2_test_msgs/AddTwoIntsResponse";
        const MD5SUM: &'static str = "b88405221c77b1878a3cbbfff53428d7";
        const DEFINITION: &'static str = r####"int64 sum"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f92e50f6d0e6c1a545a17516c5c64217dad070b5c24fcaa7a2734454396a4511";
        const ROS2_TYPE_NAME: &'static str = "ros2_test_msgs::msg::dds_::AddTwoIntsResponse_";
    }
    #[allow(dead_code)]
    pub struct AddTwoInts {}
    impl ::roslibrust::RosServiceType for AddTwoInts {
        const ROS_SERVICE_NAME: &'static str = "ros2_test_msgs/AddTwoInts";
        const MD5SUM: &'static str = "6a2e34150c00229791cc89ff309fff21";
        const ROS2_HASH: &'static str =
            "RIHS01_cbdcb755e63eba37467c9846fe9f0b458c2989832e888dfd39ecbf8991800ef7";
        type Request = AddTwoIntsRequest;
        type Response = AddTwoIntsResponse;
    }
}
#[allow(unused_imports)]
pub mod sensor_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct BatteryState {
        pub r#header: std_msgs::Header,
        pub r#voltage: f32,
        pub r#temperature: f32,
        pub r#current: f32,
        pub r#charge: f32,
        pub r#capacity: f32,
        pub r#design_capacity: f32,
        pub r#percentage: f32,
        pub r#power_supply_status: u8,
        pub r#power_supply_health: u8,
        pub r#power_supply_technology: u8,
        pub r#present: bool,
        pub r#cell_voltage: ::std::vec::Vec<f32>,
        pub r#cell_temperature: ::std::vec::Vec<f32>,
        pub r#location: ::std::string::String,
        pub r#serial_number: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for BatteryState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/BatteryState";
        const MD5SUM: &'static str = "a854891af0c45fe90fe3d9efa22e6a97";
        const DEFINITION: &'static str = r####"# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

std_msgs/Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
float32[] cell_temperature # An array of individual cell temperatures for each cell in the pack
                           # If individual temperatures unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::BatteryState_";
    }
    #[allow(unused)]
    impl BatteryState {
        pub const r#POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0u8;
        pub const r#POWER_SUPPLY_STATUS_CHARGING: u8 = 1u8;
        pub const r#POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2u8;
        pub const r#POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3u8;
        pub const r#POWER_SUPPLY_STATUS_FULL: u8 = 4u8;
        pub const r#POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0u8;
        pub const r#POWER_SUPPLY_HEALTH_GOOD: u8 = 1u8;
        pub const r#POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2u8;
        pub const r#POWER_SUPPLY_HEALTH_DEAD: u8 = 3u8;
        pub const r#POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4u8;
        pub const r#POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5u8;
        pub const r#POWER_SUPPLY_HEALTH_COLD: u8 = 6u8;
        pub const r#POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7u8;
        pub const r#POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct CameraInfo {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#distortion_model: ::std::string::String,
        pub r#d: ::std::vec::Vec<f64>,
        pub r#k: [f64; 9],
        pub r#r: [f64; 9],
        pub r#p: [f64; 12],
        pub r#binning_x: u32,
        pub r#binning_y: u32,
        pub r#roi: self::RegionOfInterest,
    }
    impl ::roslibrust::RosMessageType for CameraInfo {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
        const MD5SUM: &'static str = "dd7e7628562b54bd9369a62e00c4149f";
        const DEFINITION: &'static str = r####"# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of camera
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated.
# Normally this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficent.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  k # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  r # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] p # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::CameraInfo_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct ChannelFloat32 {
        pub r#name: ::std::string::String,
        pub r#values: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for ChannelFloat32 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/ChannelFloat32";
        const MD5SUM: &'static str = "3d40139cdd33dfedcb71ffeeeb42ae7f";
        const DEFINITION: &'static str = r####"# This message is used by the PointCloud message to hold optional data
# associated with each point in the cloud. The length of the values
# array should be the same as the length of the points array in the
# PointCloud, and each value should be associated with the corresponding
# point.
#
# Channel names in existing practice include:
#   "u", "v" - row and column (respectively) in the left stereo image.
#              This is opposite to usual conventions but remains for
#              historical reasons. The newer PointCloud2 message has no
#              such problem.
#   "rgb" - For point clouds produced by color stereo cameras. uint8
#           (R,G,B) values packed into the least significant 24 bits,
#           in order.
#   "intensity" - laser or pixel intensity.
#   "distance"

# The channel name should give semantics of the channel (e.g.
# "intensity" instead of "value").
string name

# The values array should be 1-1 with the elements of the associated
# PointCloud.
float32[] values"####;
        const ROS2_HASH: &'static str =
            "RIHS01_92665437ddf39346f4ba39ee32e648390605b633cc077d40f4bd4d7b58af6cd4";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::ChannelFloat32_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct CompressedImage {
        pub r#header: std_msgs::Header,
        pub r#format: ::std::string::String,
        #[serde(with = "::roslibrust::codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for CompressedImage {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CompressedImage";
        const MD5SUM: &'static str = "94e6ae367279367010dcb2fcd9af9493";
        const DEFINITION: &'static str = r####"# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::CompressedImage_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct FluidPressure {
        pub r#header: std_msgs::Header,
        pub r#fluid_pressure: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust::RosMessageType for FluidPressure {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/FluidPressure";
        const MD5SUM: &'static str = "d316d4254128d0a3efa4f26fcb38930b";
        const DEFINITION: &'static str = r####"# Single pressure reading.  This message is appropriate for measuring the
# pressure inside of a fluid (air, water, etc).  This also includes
# atmospheric or barometric pressure.
#
# This message is not appropriate for force/pressure contact sensors.

std_msgs/Header header # timestamp of the measurement
                             # frame_id is the location of the pressure sensor

float64 fluid_pressure       # Absolute pressure reading in Pascals.

float64 variance             # 0 is interpreted as variance unknown
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::FluidPressure_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Illuminance {
        pub r#header: std_msgs::Header,
        pub r#illuminance: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust::RosMessageType for Illuminance {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Illuminance";
        const MD5SUM: &'static str = "f0d4563fb8b7a9466214d99a65f1c203";
        const DEFINITION: &'static str = r####"# Single photometric illuminance measurement.  Light should be assumed to be
# measured along the sensor's x-axis (the area of detection is the y-z plane).
# The illuminance should have a 0 or positive value and be received with
# the sensor's +X axis pointing toward the light source.
#
# Photometric illuminance is the measure of the human eye's sensitivity of the
# intensity of light encountering or passing through a surface.
#
# All other Photometric and Radiometric measurements should not use this message.
# This message cannot represent:
#  - Luminous intensity (candela/light source output)
#  - Luminance (nits/light output per area)
#  - Irradiance (watt/area), etc.

std_msgs/Header header # timestamp is the time the illuminance was measured
                             # frame_id is the location and direction of the reading

float64 illuminance          # Measurement of the Photometric Illuminance in Lux.

float64 variance             # 0 is interpreted as variance unknown
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b954b25f452fcf81a91c9c2a7e3b3fd85c4c873d452aecb3cfd8fd1da732a22d";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Illuminance_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Image {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#encoding: ::std::string::String,
        pub r#is_bigendian: u8,
        pub r#step: u32,
        #[serde(with = "::roslibrust::codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for Image {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Image";
        const MD5SUM: &'static str = "cb4b625a4511c7f037ca38aa59e33caf";
        const DEFINITION: &'static str = r####"# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Image_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Imu {
        pub r#header: std_msgs::Header,
        pub r#orientation: geometry_msgs::Quaternion,
        pub r#orientation_covariance: [f64; 9],
        pub r#angular_velocity: geometry_msgs::Vector3,
        pub r#angular_velocity_covariance: [f64; 9],
        pub r#linear_acceleration: geometry_msgs::Vector3,
        pub r#linear_acceleration_covariance: [f64; 9],
    }
    impl ::roslibrust::RosMessageType for Imu {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Imu";
        const MD5SUM: &'static str = "71ec98acab7649fe7360000f098e51b6";
        const DEFINITION: &'static str = r####"# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Imu_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct JointState {
        pub r#header: std_msgs::Header,
        pub r#name: ::std::vec::Vec<::std::string::String>,
        pub r#position: ::std::vec::Vec<f64>,
        pub r#velocity: ::std::vec::Vec<f64>,
        pub r#effort: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for JointState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JointState";
        const MD5SUM: &'static str = "76025ed4a519922c1078eb82b0e275b3";
        const DEFINITION: &'static str = r####"# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::JointState_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Joy {
        pub r#header: std_msgs::Header,
        pub r#axes: ::std::vec::Vec<f32>,
        pub r#buttons: ::std::vec::Vec<i32>,
    }
    impl ::roslibrust::RosMessageType for Joy {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Joy";
        const MD5SUM: &'static str = "973063a40a015b6b2919c1f23327b592";
        const DEFINITION: &'static str = r####"# Reports the state of a joystick's axes and buttons.

# The timestamp is the time at which data is received from the joystick.
std_msgs/Header header

# The axes measurements from a joystick.
float32[] axes

# The buttons measurements from a joystick.
int32[] buttons
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_0d356c79cad3401e35ffeb75a96a96e08be3ef896b8b83841d73e890989372c5";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Joy_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct JoyFeedback {
        pub r#type: u8,
        pub r#id: u8,
        pub r#intensity: f32,
    }
    impl ::roslibrust::RosMessageType for JoyFeedback {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedback";
        const MD5SUM: &'static str = "f4dcd73460360d98f36e55ee7f2e46f1";
        const DEFINITION: &'static str = r####"# Declare of the type of feedback
uint8 TYPE_LED    = 0
uint8 TYPE_RUMBLE = 1
uint8 TYPE_BUZZER = 2

uint8 type

# This will hold an id number for each type of each feedback.
# Example, the first led would be id=0, the second would be id=1
uint8 id

# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
float32 intensity"####;
        const ROS2_HASH: &'static str =
            "RIHS01_231dd362f71d6fc08272770d07120ad5fe5874ce2dbac70109b28986834290cd";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::JoyFeedback_";
    }
    #[allow(unused)]
    impl JoyFeedback {
        pub const r#TYPE_LED: u8 = 0u8;
        pub const r#TYPE_RUMBLE: u8 = 1u8;
        pub const r#TYPE_BUZZER: u8 = 2u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct JoyFeedbackArray {
        pub r#array: ::std::vec::Vec<self::JoyFeedback>,
    }
    impl ::roslibrust::RosMessageType for JoyFeedbackArray {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedbackArray";
        const MD5SUM: &'static str = "cde5730a895b1fc4dee6f91b754b213d";
        const DEFINITION: &'static str = r####"# This message publishes values for multiple feedback at once.
JoyFeedback[] array
================================================================================
MSG: sensor_msgs/JoyFeedback
# Declare of the type of feedback
uint8 TYPE_LED    = 0
uint8 TYPE_RUMBLE = 1
uint8 TYPE_BUZZER = 2

uint8 type

# This will hold an id number for each type of each feedback.
# Example, the first led would be id=0, the second would be id=1
uint8 id

# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
float32 intensity"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3287c32e1b688cae04555e465443df3cca7dae76ee4ebf85c4658d585037bcaa";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::JoyFeedbackArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct LaserEcho {
        pub r#echoes: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for LaserEcho {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserEcho";
        const MD5SUM: &'static str = "8bc5ae449b200fba4d552b4225586696";
        const DEFINITION: &'static str = r####"# This message is a submessage of MultiEchoLaserScan and is not intended
# to be used separately.

float32[] echoes  # Multiple values of ranges or intensities.
                  # Each array represents data from the same angle increment."####;
        const ROS2_HASH: &'static str =
            "RIHS01_0fbc05a0db7d37fe52c0f0375356db55da0046f7ef5bd27ca6b34bd0582bc952";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::LaserEcho_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct LaserScan {
        pub r#header: std_msgs::Header,
        pub r#angle_min: f32,
        pub r#angle_max: f32,
        pub r#angle_increment: f32,
        pub r#time_increment: f32,
        pub r#scan_time: f32,
        pub r#range_min: f32,
        pub r#range_max: f32,
        pub r#ranges: ::std::vec::Vec<f32>,
        pub r#intensities: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for LaserScan {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserScan";
        const MD5SUM: &'static str = "74d1ff8cca260280d68caf223b174a40";
        const DEFINITION: &'static str = r####"# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_64c191398013af96509d518dac71d5164f9382553fce5c1f8cca5be7924bd828";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::LaserScan_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MagneticField {
        pub r#header: std_msgs::Header,
        pub r#magnetic_field: geometry_msgs::Vector3,
        pub r#magnetic_field_covariance: [f64; 9],
    }
    impl ::roslibrust::RosMessageType for MagneticField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MagneticField";
        const MD5SUM: &'static str = "bdf89cd6ae874ad3919288d2d047a53e";
        const DEFINITION: &'static str = r####"# Measurement of the Magnetic Field vector at a specific location.
#
# If the covariance of the measurement is known, it should be filled in.
# If all you know is the variance of each measurement, e.g. from the datasheet,
# just put those along the diagonal.
# A covariance matrix of all zeros will be interpreted as "covariance unknown",
# and to use the data a covariance will have to be assumed or gotten from some
# other source.

std_msgs/Header header               # timestamp is the time the
                                           # field was measured
                                           # frame_id is the location and orientation
                                           # of the field measurement

geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                           # field vector in Tesla
                                           # If your sensor does not output 3 axes,
                                           # put NaNs in the components not reported.

float64[9] magnetic_field_covariance       # Row major about x, y, z axes
                                           # 0 is interpreted as variance unknown
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::MagneticField_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MultiDOFJointState {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#twist: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#wrench: ::std::vec::Vec<geometry_msgs::Wrench>,
    }
    impl ::roslibrust::RosMessageType for MultiDOFJointState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiDOFJointState";
        const MD5SUM: &'static str = "1a14cbd02028378130c7a94f7b7160d9";
        const DEFINITION: &'static str = r####"# Representation of state for joints with multiple degrees of freedom,
# following the structure of JointState which can only represent a single degree of freedom.
#
# It is assumed that a joint in a system corresponds to a transform that gets applied
# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
# and those 3DOF can be expressed as a transformation matrix, and that transformation
# matrix can be converted back to (x, y, yaw)
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# wrench associated with them, you can leave the wrench array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] joint_names
geometry_msgs/Transform[] transforms
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_4d4ded702cfba7ff3ec783835c1a1425f75e53939a430ff355d1fee4b3bbc40b";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::MultiDOFJointState_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MultiEchoLaserScan {
        pub r#header: std_msgs::Header,
        pub r#angle_min: f32,
        pub r#angle_max: f32,
        pub r#angle_increment: f32,
        pub r#time_increment: f32,
        pub r#scan_time: f32,
        pub r#range_min: f32,
        pub r#range_max: f32,
        pub r#ranges: ::std::vec::Vec<self::LaserEcho>,
        pub r#intensities: ::std::vec::Vec<self::LaserEcho>,
    }
    impl ::roslibrust::RosMessageType for MultiEchoLaserScan {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiEchoLaserScan";
        const MD5SUM: &'static str = "f65e1d7e3f351478cec13af3b50f5cfe";
        const DEFINITION: &'static str = r####"# Single scan from a multi-echo planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

LaserEcho[] ranges           # range data [m]
                             # (Note: NaNs, values < range_min or > range_max should be discarded)
                             # +Inf measurements are out of range
                             # -Inf measurements are too close to determine exact distance.
LaserEcho[] intensities      # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/LaserEcho
# This message is a submessage of MultiEchoLaserScan and is not intended
# to be used separately.

float32[] echoes  # Multiple values of ranges or intensities.
                  # Each array represents data from the same angle increment.
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_ba5eac341cd5bbb2701527aa4568e8baec172b69cadb9a1945d6f149d087ee48";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::MultiEchoLaserScan_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct NavSatFix {
        pub r#header: std_msgs::Header,
        pub r#status: self::NavSatStatus,
        pub r#latitude: f64,
        pub r#longitude: f64,
        pub r#altitude: f64,
        pub r#position_covariance: [f64; 9],
        pub r#position_covariance_type: u8,
    }
    impl ::roslibrust::RosMessageType for NavSatFix {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatFix";
        const MD5SUM: &'static str = "67a809d0baed1f0d41ca4f460a28e15b";
        const DEFINITION: &'static str = r####"# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header

# Satellite fix status information.
NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.
float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System.
#
# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_62223ab3fe210a15976021da7afddc9e200dc9ec75231c1b6a557fc598a65404";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::NavSatFix_";
    }
    #[allow(unused)]
    impl NavSatFix {
        pub const r#COVARIANCE_TYPE_UNKNOWN: u8 = 0u8;
        pub const r#COVARIANCE_TYPE_APPROXIMATED: u8 = 1u8;
        pub const r#COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2u8;
        pub const r#COVARIANCE_TYPE_KNOWN: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct NavSatStatus {
        pub r#status: i8,
        pub r#service: u16,
    }
    impl ::roslibrust::RosMessageType for NavSatStatus {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatStatus";
        const MD5SUM: &'static str = "331cdbddfa4bc96ffc3b9ad98900a54c";
        const DEFINITION: &'static str = r####"# Navigation Satellite fix status for any Global Navigation Satellite System.
#
# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service"####;
        const ROS2_HASH: &'static str =
            "RIHS01_d1ed3befa628e09571bd273b888ba1c1fd187c9a5e0006b385d7e5e9095a3204";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::NavSatStatus_";
    }
    #[allow(unused)]
    impl NavSatStatus {
        pub const r#STATUS_NO_FIX: i8 = -1i8;
        pub const r#STATUS_FIX: i8 = 0i8;
        pub const r#STATUS_SBAS_FIX: i8 = 1i8;
        pub const r#STATUS_GBAS_FIX: i8 = 2i8;
        pub const r#SERVICE_GPS: u16 = 1u16;
        pub const r#SERVICE_GLONASS: u16 = 2u16;
        pub const r#SERVICE_COMPASS: u16 = 4u16;
        pub const r#SERVICE_GALILEO: u16 = 8u16;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PointCloud {
        pub r#header: std_msgs::Header,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point32>,
        pub r#channels: ::std::vec::Vec<self::ChannelFloat32>,
    }
    impl ::roslibrust::RosMessageType for PointCloud {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud";
        const MD5SUM: &'static str = "24daa8f40e42d4456d006be6be52b658";
        const DEFINITION: &'static str = r####"## THIS MESSAGE IS DEPRECATED AS OF FOXY
## Please use sensor_msgs/PointCloud2

# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: sensor_msgs/ChannelFloat32
# This message is used by the PointCloud message to hold optional data
# associated with each point in the cloud. The length of the values
# array should be the same as the length of the points array in the
# PointCloud, and each value should be associated with the corresponding
# point.
#
# Channel names in existing practice include:
#   "u", "v" - row and column (respectively) in the left stereo image.
#              This is opposite to usual conventions but remains for
#              historical reasons. The newer PointCloud2 message has no
#              such problem.
#   "rgb" - For point clouds produced by color stereo cameras. uint8
#           (R,G,B) values packed into the least significant 24 bits,
#           in order.
#   "intensity" - laser or pixel intensity.
#   "distance"

# The channel name should give semantics of the channel (e.g.
# "intensity" instead of "value").
string name

# The values array should be 1-1 with the elements of the associated
# PointCloud.
float32[] values
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_614593df71d3c2b9bd4604a71b750fd218f0d65c045ea988b713719455a65b3b";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::PointCloud_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PointCloud2 {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#fields: ::std::vec::Vec<self::PointField>,
        pub r#is_bigendian: bool,
        pub r#point_step: u32,
        pub r#row_step: u32,
        #[serde(with = "::roslibrust::codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
        pub r#is_dense: bool,
    }
    impl ::roslibrust::RosMessageType for PointCloud2 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud2";
        const MD5SUM: &'static str = "9fd59d84b32e656d55663e44b07621cb";
        const DEFINITION: &'static str = r####"# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.
#
# The point cloud data may be organized 2d (image-like) or 1d (unordered).
# Point clouds organized as 2d images may be produced by camera depth sensors
# such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
std_msgs/Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

# Common PointField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::PointCloud2_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct PointField {
        pub r#name: ::std::string::String,
        pub r#offset: u32,
        pub r#datatype: u8,
        pub r#count: u32,
    }
    impl ::roslibrust::RosMessageType for PointField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointField";
        const MD5SUM: &'static str = "268eacb2962780ceac86cbd17e328150";
        const DEFINITION: &'static str = r####"# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

# Common PointField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field"####;
        const ROS2_HASH: &'static str =
            "RIHS01_5c6a4750728c2bcfbbf7037225b20b02d4429634732146b742dee1726637ef01";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::PointField_";
    }
    #[allow(unused)]
    impl PointField {
        pub const r#INT8: u8 = 1u8;
        pub const r#UINT8: u8 = 2u8;
        pub const r#INT16: u8 = 3u8;
        pub const r#UINT16: u8 = 4u8;
        pub const r#INT32: u8 = 5u8;
        pub const r#UINT32: u8 = 6u8;
        pub const r#FLOAT32: u8 = 7u8;
        pub const r#FLOAT64: u8 = 8u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Range {
        pub r#header: std_msgs::Header,
        pub r#radiation_type: u8,
        pub r#field_of_view: f32,
        pub r#min_range: f32,
        pub r#max_range: f32,
        pub r#range: f32,
    }
    impl ::roslibrust::RosMessageType for Range {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Range";
        const MD5SUM: &'static str = "e1977ba4a46a6d66d32a4bb49e63a3cf";
        const DEFINITION: &'static str = r####"# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured.
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.
#
# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

std_msgs/Header header # timestamp in the header is the time the ranger
                             # returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range.
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        # Fixed distance rangers require min_range==max_range

float32 range           # range data [m]
                        # (Note: values < range_min or > range_max should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_9430b1915b94d4268ff903679e8ecd09b6a67d331bd028738ec1eeb592891ebd";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Range_";
    }
    #[allow(unused)]
    impl Range {
        pub const r#ULTRASOUND: u8 = 0u8;
        pub const r#INFRARED: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct RegionOfInterest {
        pub r#x_offset: u32,
        pub r#y_offset: u32,
        pub r#height: u32,
        pub r#width: u32,
        pub r#do_rectify: bool,
    }
    impl ::roslibrust::RosMessageType for RegionOfInterest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RegionOfInterest";
        const MD5SUM: &'static str = "bdb633039d588fcccb441a4d43ccfe09";
        const DEFINITION: &'static str = r####"# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify"####;
        const ROS2_HASH: &'static str =
            "RIHS01_ad16bcba5f9131dcdba6fbded19f726f5440e3c513b4fb586dd3027eeed8abb1";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::RegionOfInterest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct RelativeHumidity {
        pub r#header: std_msgs::Header,
        pub r#relative_humidity: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust::RosMessageType for RelativeHumidity {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RelativeHumidity";
        const MD5SUM: &'static str = "86bd4f852f7b1629a975f5c9748adcae";
        const DEFINITION: &'static str = r####"# Single reading from a relative humidity sensor.
# Defines the ratio of partial pressure of water vapor to the saturated vapor
# pressure at a temperature.

std_msgs/Header header # timestamp of the measurement
                             # frame_id is the location of the humidity sensor

float64 relative_humidity    # Expression of the relative humidity
                             # from 0.0 to 1.0.
                             # 0.0 is no partial pressure of water vapor
                             # 1.0 represents partial pressure of saturation

float64 variance             # 0 is interpreted as variance unknown
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_8687c99b4fb393cb2e545e407b5ea7fd0b5d8960bcd849a0f86c544740138839";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::RelativeHumidity_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Temperature {
        pub r#header: std_msgs::Header,
        pub r#temperature: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust::RosMessageType for Temperature {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Temperature";
        const MD5SUM: &'static str = "763585dd205ef2f7971cb27bf4063544";
        const DEFINITION: &'static str = r####"# Single temperature reading.

std_msgs/Header header # timestamp is the time the temperature was measured
                             # frame_id is the location of the temperature reading

float64 temperature          # Measurement of the Temperature in Degrees Celsius.

float64 variance             # 0 is interpreted as variance unknown.
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Temperature_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TimeReference {
        pub r#header: std_msgs::Header,
        pub r#time_ref: builtin_interfaces::Time,
        pub r#source: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for TimeReference {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/TimeReference";
        const MD5SUM: &'static str = "5d254fd46ee1270d203ae56389c33ddc";
        const DEFINITION: &'static str = r####"# Measurement from an external time source not actively synchronized with the system clock.

std_msgs/Header header      # stamp is system time for which measurement was valid
                                  # frame_id is not used

builtin_interfaces/Time time_ref  # corresponding time from this external source
string source                     # (optional) name of time source
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_dd66e84cf40bbb5d5a40472e6ecf2675a031334d4c426abdb2ad41801a8efc99";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::TimeReference_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SetCameraInfoRequest {
        pub r#camera_info: self::CameraInfo,
    }
    impl ::roslibrust::RosMessageType for SetCameraInfoRequest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoRequest";
        const MD5SUM: &'static str = "b68b6bcb6b1cb79875182f7b4170d888";
        const DEFINITION: &'static str = r####"# This service requests that a camera stores the given CameraInfo as that
# camera's calibration information.
#
# The width and height in the camera_info field should match what the
# camera is currently outputting on its camera_info topic, and the camera
# will assume that the region of the imager that is being referred to is
# the region that the camera is currently capturing.

sensor_msgs/CameraInfo camera_info # The camera_info to store
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/CameraInfo
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of camera
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated.
# Normally this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficent.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  k # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  r # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] p # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_09cd0dda20dd062bdd4a0be668e7614a437fe78423c2a524f432db4d7bae2a5d";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::SetCameraInfoRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SetCameraInfoResponse {
        pub r#success: bool,
        pub r#status_message: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for SetCameraInfoResponse {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoResponse";
        const MD5SUM: &'static str = "2ec6f3eff0161f4257b808b12bc830c2";
        const DEFINITION: &'static str = r####"bool success                             # True if the call succeeded
string status_message                    # Used to give details about success"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3ae30a6b9bc664769913157d2866d4ccbe5ecd7ed7b76f704497bc7d292d3d58";
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::SetCameraInfoResponse_";
    }
    #[allow(dead_code)]
    pub struct SetCameraInfo {}
    impl ::roslibrust::RosServiceType for SetCameraInfo {
        const ROS_SERVICE_NAME: &'static str = "sensor_msgs/SetCameraInfo";
        const MD5SUM: &'static str = "09f27e7f0ee56f224480abfd6cd869c0";
        const ROS2_HASH: &'static str =
            "RIHS01_a10cca5d33dc637c8d49db50ab288701a3592bb9cd854f2f16a0659613b68984";
        type Request = SetCameraInfoRequest;
        type Response = SetCameraInfoResponse;
    }
}
#[allow(unused_imports)]
pub mod service_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct ServiceEventInfo {
        pub r#event_type: u8,
        pub r#stamp: builtin_interfaces::Time,
        pub r#client_gid: [u8; 16],
        pub r#sequence_number: i64,
    }
    impl ::roslibrust::RosMessageType for ServiceEventInfo {
        const ROS_TYPE_NAME: &'static str = "service_msgs/ServiceEventInfo";
        const MD5SUM: &'static str = "42561fc0d0d3665a03d59fbb1296daf9";
        const DEFINITION: &'static str = r####"uint8 REQUEST_SENT = 0
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_41bcbbe07a75c9b52bc96bfd5c24d7f0fc0a08c0cb7921b3373c5732345a6f45";
        const ROS2_TYPE_NAME: &'static str = "service_msgs::msg::dds_::ServiceEventInfo_";
    }
    #[allow(unused)]
    impl ServiceEventInfo {
        pub const r#REQUEST_SENT: u8 = 0u8;
        pub const r#REQUEST_RECEIVED: u8 = 1u8;
        pub const r#RESPONSE_SENT: u8 = 2u8;
        pub const r#RESPONSE_RECEIVED: u8 = 3u8;
    }
}
#[allow(unused_imports)]
pub mod shape_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Mesh {
        pub r#triangles: ::std::vec::Vec<self::MeshTriangle>,
        pub r#vertices: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust::RosMessageType for Mesh {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Mesh";
        const MD5SUM: &'static str = "1ffdae9486cd3316a121c578b47a85cc";
        const DEFINITION: &'static str = r####"# Definition of a mesh.

# List of triangles; the index values refer to positions in vertices[].
MeshTriangle[] triangles

# The actual vertices that make up the mesh.
geometry_msgs/Point[] vertices
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices.

uint32[3] vertex_indices"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f2150b82d8ee7e8bc3f396a2b158aefb4b9a5510a474be271ba1268aebb55289";
        const ROS2_TYPE_NAME: &'static str = "shape_msgs::msg::dds_::Mesh_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MeshTriangle {
        pub r#vertex_indices: [u32; 3],
    }
    impl ::roslibrust::RosMessageType for MeshTriangle {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/MeshTriangle";
        const MD5SUM: &'static str = "23688b2e6d2de3d32fe8af104a903253";
        const DEFINITION: &'static str = r####"# Definition of a triangle's vertices.

uint32[3] vertex_indices"####;
        const ROS2_HASH: &'static str =
            "RIHS01_618e5c073eeb729e433ef6226e72c01d995c459fb7d76348c9700409a5020bd0";
        const ROS2_TYPE_NAME: &'static str = "shape_msgs::msg::dds_::MeshTriangle_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Plane {
        pub r#coef: [f64; 4],
    }
    impl ::roslibrust::RosMessageType for Plane {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Plane";
        const MD5SUM: &'static str = "2c1b92ed8f31492f8e73f6a4a44ca796";
        const DEFINITION: &'static str = r####"# Representation of a plane, using the plane equation ax + by + cz + d = 0.
#
# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]
float64[4] coef"####;
        const ROS2_HASH: &'static str =
            "RIHS01_dfbfe8314689c850615d4a727af017e9aa86c10e369a606c8c851ef8f16c58c8";
        const ROS2_TYPE_NAME: &'static str = "shape_msgs::msg::dds_::Plane_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SolidPrimitive {
        pub r#type: u8,
        pub r#dimensions: ::std::vec::Vec<f64>,
        pub r#polygon: geometry_msgs::Polygon,
    }
    impl ::roslibrust::RosMessageType for SolidPrimitive {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/SolidPrimitive";
        const MD5SUM: &'static str = "8c7314fcbb621a6a7a6af3200d726861";
        const DEFINITION: &'static str = r####"# Defines box, sphere, cylinder, cone and prism.
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4
uint8 PRISM=5

# The type of the shape
uint8 type

# The dimensions of the shape
float64[<=3] dimensions  # At no point will dimensions have a length > 3.

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.

# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2

# For the SPHERE type, only one component is used, and it gives the radius of the sphere.
uint8 SPHERE_RADIUS=0

# For the CYLINDER and CONE types, the center line is oriented along the Z axis.
# Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
# height of the cylinder (cone).
# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of
# the base of the cylinder (cone).
# Cone and cylinder primitives are defined to be circular. The tip of the cone
# is pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

# For the type PRISM, the center line is oriented along Z axis.
# The PRISM_HEIGHT component of dimensions gives the
# height of the prism.
# The polygon defines the Z axis centered base of the prism.
# The prism is constructed by extruding the base in +Z and -Z
# directions by half of the PRISM_HEIGHT
# Only x and y fields of the points are used in the polygon.
# Points of the polygon are ordered counter-clockwise.

uint8 PRISM_HEIGHT=0
geometry_msgs/Polygon polygon
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: geometry_msgs/Polygon
# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_2802a15190aadc3f496584df4b0b4c5824d8a0b31aaef839faa75bc34dda38ac";
        const ROS2_TYPE_NAME: &'static str = "shape_msgs::msg::dds_::SolidPrimitive_";
    }
    #[allow(unused)]
    impl SolidPrimitive {
        pub const r#BOX: u8 = 1u8;
        pub const r#SPHERE: u8 = 2u8;
        pub const r#CYLINDER: u8 = 3u8;
        pub const r#CONE: u8 = 4u8;
        pub const r#PRISM: u8 = 5u8;
        pub const r#BOX_X: u8 = 0u8;
        pub const r#BOX_Y: u8 = 1u8;
        pub const r#BOX_Z: u8 = 2u8;
        pub const r#SPHERE_RADIUS: u8 = 0u8;
        pub const r#CYLINDER_HEIGHT: u8 = 0u8;
        pub const r#CYLINDER_RADIUS: u8 = 1u8;
        pub const r#CONE_HEIGHT: u8 = 0u8;
        pub const r#CONE_RADIUS: u8 = 1u8;
        pub const r#PRISM_HEIGHT: u8 = 0u8;
    }
}
#[allow(unused_imports)]
pub mod std_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Bool {
        pub r#data: bool,
    }
    impl ::roslibrust::RosMessageType for Bool {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Bool";
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

bool data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Bool_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Byte {
        pub r#data: u8,
    }
    impl ::roslibrust::RosMessageType for Byte {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Byte";
        const MD5SUM: &'static str = "ad736a2e8818154c487bb80fe42ce43b";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

byte data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e28ca2c62f3fb10c207890755aa7a5a770ccde5646fd66b452f485c48092f327";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Byte_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct ByteMultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for ByteMultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ByteMultiArray";
        const MD5SUM: &'static str = "70ea476cbcfd65ac2f68f3cda1e891fe";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
byte[]            data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_692eff26dd8ca7623e4e90a082f7d83f1cf5deb0b7ba748a2d4d5fbca791db7d";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::ByteMultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Char {
        pub r#data: u8,
    }
    impl ::roslibrust::RosMessageType for Char {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Char";
        const MD5SUM: &'static str = "1bf77f25acecdedba0e224b162199717";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

char data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3ad2d04dd29ba19d04b16659afa3ccaedd691914b02a64e82e252f2fa6a586a9";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Char_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct ColorRGBA {
        pub r#r: f32,
        pub r#g: f32,
        pub r#b: f32,
        pub r#a: f32,
    }
    impl ::roslibrust::RosMessageType for ColorRGBA {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ColorRGBA";
        const MD5SUM: &'static str = "a29a96539573343b1310c73607334b00";
        const DEFINITION: &'static str = r####"float32 r
float32 g
float32 b
float32 a"####;
        const ROS2_HASH: &'static str =
            "RIHS01_77a7a5b9ae477306097665106e0413ba74440245b1f3d0c6d6405fe5c7813fe8";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::ColorRGBA_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Empty {}
    impl ::roslibrust::RosMessageType for Empty {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Empty";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####""####;
        const ROS2_HASH: &'static str =
            "RIHS01_20b625256f32d5dbc0d04fee44f43c41e51c70d3502f84b4a08e7a9c26a96312";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Empty_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Float32 {
        pub r#data: f32,
    }
    impl ::roslibrust::RosMessageType for Float32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32";
        const MD5SUM: &'static str = "73fcbf46b49191e672908e50842a83d4";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float32 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Float32_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Float32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for Float32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32MultiArray";
        const MD5SUM: &'static str = "6a40e0ffa6a17a503ac3f8616991b1f6";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_0599f6f85b4bfca379873a0b4375a0aca022156bd2d7021275d116ed1fa8bfe0";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Float32MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Float64 {
        pub r#data: f64,
    }
    impl ::roslibrust::RosMessageType for Float64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64";
        const MD5SUM: &'static str = "fdb28210bfa9d7c91146260178d9a584";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float64 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_705ba9c3d1a09df43737eb67095534de36fd426c0587779bda2bc51fe790182a";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Float64_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Float64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for Float64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64MultiArray";
        const MD5SUM: &'static str = "4b7d974086d4060e7db4613a7e6c3ba4";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_1025ddc6b9552d191f89ef1a8d2f60f3d373e28b283d8891ddcc974e8c55397f";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Float64MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Header {
        pub r#stamp: builtin_interfaces::Time,
        pub r#frame_id: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for Header {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
        const MD5SUM: &'static str = "3cca5e8ccb31a32fff4058beb6f250e3";
        const DEFINITION: &'static str = r####"# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Header_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int16 {
        pub r#data: i16,
    }
    impl ::roslibrust::RosMessageType for Int16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16";
        const MD5SUM: &'static str = "8524586e34fbd7cb1c08c5f5f1ca0e57";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int16 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_1dcc3464e47c288a55f943a389d337cdb06804de3f5cd7a266b0de718eee17e5";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int16_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i16>,
    }
    impl ::roslibrust::RosMessageType for Int16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16MultiArray";
        const MD5SUM: &'static str = "d9338d7f523fcb692fae9d0a0e9f067c";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int16[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b58810e8e5b90fb19a5062469eb8409f5ab11a446d60de7157a1457e52a076ce";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int16MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int32 {
        pub r#data: i32,
    }
    impl ::roslibrust::RosMessageType for Int32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32";
        const MD5SUM: &'static str = "da5909fbe378aeaf85e547e830cc1bb7";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int32 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int32_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i32>,
    }
    impl ::roslibrust::RosMessageType for Int32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32MultiArray";
        const MD5SUM: &'static str = "1d99f79f8b325b44fee908053e9c945b";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int32[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_84a7346323525d1b4dfca899df3820f245e54009dac5a6b69217d14fdefd1701";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int32MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int64 {
        pub r#data: i64,
    }
    impl ::roslibrust::RosMessageType for Int64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64";
        const MD5SUM: &'static str = "34add168574510e6e17f5d23ecc077ef";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int64 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_8cd1048c2f186b6bd9a92472dc1ce51723c0833a221e2b7aecfff111774f4b49";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int64_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i64>,
    }
    impl ::roslibrust::RosMessageType for Int64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64MultiArray";
        const MD5SUM: &'static str = "54865aa6c65be0448113a2afc6a49270";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int64[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_e60f9fe34d697f0939ad49d33158693c1277fbac0e2f04b7c2995dc21c89b422";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int64MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int8 {
        pub r#data: i8,
    }
    impl ::roslibrust::RosMessageType for Int8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8";
        const MD5SUM: &'static str = "27ffa0c9c4b8fb8492252bcad9e5c57b";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int8 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int8_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Int8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i8>,
    }
    impl ::roslibrust::RosMessageType for Int8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8MultiArray";
        const MD5SUM: &'static str = "d7c1af35a1b4781bbe79e03dd94b7c13";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int8[]            data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f21998d4b492abd63330765d75d5831238d400740386f651f13a872a4d2188db";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int8MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MultiArrayDimension {
        pub r#label: ::std::string::String,
        pub r#size: u32,
        pub r#stride: u32,
    }
    impl ::roslibrust::RosMessageType for MultiArrayDimension {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayDimension";
        const MD5SUM: &'static str = "4cd0c83a8683deae40ecdac60e53bfa8";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_5e773a60a4c7fc8a54985f307c7837aa2994252a126c301957a24e31282c9cbe";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::MultiArrayDimension_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MultiArrayLayout {
        pub r#dim: ::std::vec::Vec<self::MultiArrayDimension>,
        pub r#data_offset: u32,
    }
    impl ::roslibrust::RosMessageType for MultiArrayLayout {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayLayout";
        const MD5SUM: &'static str = "0fed2a11c13e11c5571b4e2a995a91a3";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::MultiArrayLayout_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct String {
        pub r#data: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for String {
        const ROS_TYPE_NAME: &'static str = "std_msgs/String";
        const MD5SUM: &'static str = "992ce8a1687cec8c8bd883ec73ca41d1";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::String_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt16 {
        pub r#data: u16,
    }
    impl ::roslibrust::RosMessageType for UInt16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16";
        const MD5SUM: &'static str = "1df79edf208b629fe6b81923a544552d";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint16 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_08a406e4b022bc22e907f985d6a9e9dd1d4fbecae573549cf49350113e7757b1";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt16_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u16>,
    }
    impl ::roslibrust::RosMessageType for UInt16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16MultiArray";
        const MD5SUM: &'static str = "52f264f1c973c4b73790d384c6cb4484";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint16[]            data        # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_94fe73428ec63baecc774f8fb82406123e9291cf728f1b7c91caf5335129492b";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt16MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt32 {
        pub r#data: u32,
    }
    impl ::roslibrust::RosMessageType for UInt32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32";
        const MD5SUM: &'static str = "304a39449588c7f8ce2df6e8001c5fce";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint32 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_a5c874829b752bc5fa190024b0ad76f578cc278271e855c7d02a818b3516fb4a";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt32_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u32>,
    }
    impl ::roslibrust::RosMessageType for UInt32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32MultiArray";
        const MD5SUM: &'static str = "4d6a180abc9be191b96a7eda6c8a233d";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint32[]          data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_6c2577c7ad3cbdcc2164a41c12f1d5ad314ea320f3fb1ee47e78019fe16bb5b0";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt32MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt64 {
        pub r#data: u64,
    }
    impl ::roslibrust::RosMessageType for UInt64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64";
        const MD5SUM: &'static str = "1b2a79973e8bf53d7b53acb71299cb57";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint64 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_fbdc52018fc13755dce18024d1a671c856aa8b4aaf63adfb095b608f98e8c943";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt64_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u64>,
    }
    impl ::roslibrust::RosMessageType for UInt64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64MultiArray";
        const MD5SUM: &'static str = "6088f127afb1d6c72927aa1247e945af";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint64[]          data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_fc1c685c2f76bdc6983da025cb25d2db5fb5157b059e300f6d957d86f981b366";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt64MultiArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt8 {
        pub r#data: u8,
    }
    impl ::roslibrust::RosMessageType for UInt8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8";
        const MD5SUM: &'static str = "7c8164229e7d2c17eb95e9231617fdee";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint8 data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_6138bd83d8c3569cb80a667db03cfc1629f529fee79d944c39c34e352e72f010";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt8_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UInt8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        #[serde(with = "::roslibrust::codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for UInt8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8MultiArray";
        const MD5SUM: &'static str = "82373f1612381bb6ee473b5cd6f5d89c";
        const DEFINITION: &'static str = r####"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####;
        const ROS2_HASH: &'static str =
            "RIHS01_5687e861b8d307a5e48b7515467ae7a5fc2daf805bd0ce6d8e9e604bade9f385";
        const ROS2_TYPE_NAME: &'static str = "std_msgs::msg::dds_::UInt8MultiArray_";
    }
}
#[allow(unused_imports)]
pub mod std_srvs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct EmptyRequest {}
    impl ::roslibrust::RosMessageType for EmptyRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/EmptyRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####""####;
        const ROS2_HASH: &'static str =
            "RIHS01_f6eb4f4a22d9555ac7d9ae4c283cf4a0887159c4bba75e95ecafe6234d301532";
        const ROS2_TYPE_NAME: &'static str = "std_srvs::msg::dds_::EmptyRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct EmptyResponse {}
    impl ::roslibrust::RosMessageType for EmptyResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/EmptyResponse";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####""####;
        const ROS2_HASH: &'static str =
            "RIHS01_813f728ba19e0894d3fd51c30f1927ae1fe9fecae57767080acff35f7cbcc5f8";
        const ROS2_TYPE_NAME: &'static str = "std_srvs::msg::dds_::EmptyResponse_";
    }
    #[allow(dead_code)]
    pub struct Empty {}
    impl ::roslibrust::RosServiceType for Empty {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Empty";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const ROS2_HASH: &'static str =
            "RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19";
        type Request = EmptyRequest;
        type Response = EmptyResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SetBoolRequest {
        pub r#data: bool,
    }
    impl ::roslibrust::RosMessageType for SetBoolRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/SetBoolRequest";
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
        const DEFINITION: &'static str =
            r####"bool data # e.g. for hardware enabling / disabling"####;
        const ROS2_HASH: &'static str =
            "RIHS01_a930ae8d8d848404ac069e26ecf6e6528275a6a9c0e874431ef4c1c30a048597";
        const ROS2_TYPE_NAME: &'static str = "std_srvs::msg::dds_::SetBoolRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct SetBoolResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for SetBoolResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/SetBoolResponse";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const DEFINITION: &'static str = r####"bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages"####;
        const ROS2_HASH: &'static str =
            "RIHS01_1fd19784a1aa5e9d9f4329c8af61c6f1481f3a4518c7808b8b6b4f0d8ac57071";
        const ROS2_TYPE_NAME: &'static str = "std_srvs::msg::dds_::SetBoolResponse_";
    }
    #[allow(dead_code)]
    pub struct SetBool {}
    impl ::roslibrust::RosServiceType for SetBool {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/SetBool";
        const MD5SUM: &'static str = "09fb03525b03e7ea1fd3992bafd87e16";
        const ROS2_HASH: &'static str =
            "RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a";
        type Request = SetBoolRequest;
        type Response = SetBoolResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TriggerRequest {}
    impl ::roslibrust::RosMessageType for TriggerRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/TriggerRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####""####;
        const ROS2_HASH: &'static str =
            "RIHS01_3ca06c57645a3431192de5b56909bd2045df4d537006cb039624bbdf8f51f0af";
        const ROS2_TYPE_NAME: &'static str = "std_srvs::msg::dds_::TriggerRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct TriggerResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for TriggerResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/TriggerResponse";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const DEFINITION: &'static str = r####"bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages"####;
        const ROS2_HASH: &'static str =
            "RIHS01_23a1b810a6917a5087be330abe53e8c70b48bb8f707e1756f51fab4ca9bbdfd9";
        const ROS2_TYPE_NAME: &'static str = "std_srvs::msg::dds_::TriggerResponse_";
    }
    #[allow(dead_code)]
    pub struct Trigger {}
    impl ::roslibrust::RosServiceType for Trigger {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Trigger";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const ROS2_HASH: &'static str =
            "RIHS01_eeff2cd6fa5ad9d27cdf4dec64818317839b62f212a91e6b5304b634b2062c5f";
        type Request = TriggerRequest;
        type Response = TriggerResponse;
    }
}
#[allow(unused_imports)]
pub mod stereo_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct DisparityImage {
        pub r#header: std_msgs::Header,
        pub r#image: sensor_msgs::Image,
        pub r#f: f32,
        pub r#t: f32,
        pub r#valid_window: sensor_msgs::RegionOfInterest,
        pub r#min_disparity: f32,
        pub r#max_disparity: f32,
        pub r#delta_d: f32,
    }
    impl ::roslibrust::RosMessageType for DisparityImage {
        const ROS_TYPE_NAME: &'static str = "stereo_msgs/DisparityImage";
        const MD5SUM: &'static str = "34550463dd15bbbb0391c0200611ad7d";
        const DEFINITION: &'static str = r####"# Separate header for compatibility with current TimeSynchronizer.
# Likely to be removed in a later release, use image.header instead.
std_msgs/Header header

# Floating point disparity image. The disparities are pre-adjusted for any
# x-offset between the principal points of the two cameras (in the case
# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
sensor_msgs/Image image

# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
float32 f # Focal length, pixels
float32 t # Baseline, world units

# Subwindow of (potentially) valid disparity values.
sensor_msgs/RegionOfInterest valid_window

# The range of disparities searched.
# In the disparity image, any disparity less than min_disparity is invalid.
# The disparity search range defines the horopter, or 3D volume that the
# stereo algorithm can "see". Points with Z outside of:
#     Z_min = fT / max_disparity
#     Z_max = fT / min_disparity
# could not be found.
float32 min_disparity
float32 max_disparity

# Smallest allowed disparity increment. The smallest achievable depth range
# resolution is delta_Z = (Z^2/fT)*delta_d.
float32 delta_d
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_1ec1ff6b5bace919e4544a37f2d96ead9f81783701b7b7a4d97a09325ecf2711";
        const ROS2_TYPE_NAME: &'static str = "stereo_msgs::msg::dds_::DisparityImage_";
    }
}
#[allow(unused_imports)]
pub mod trajectory_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct JointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::JointTrajectoryPoint>,
    }
    impl ::roslibrust::RosMessageType for JointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectory";
        const MD5SUM: &'static str = "c3e4956a725855360bb50519b5226821";
        const DEFINITION: &'static str = r####"# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header

# The names of the active joints in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] joint_names

# Array of trajectory points, which describe the positions, velocities,
# accelerations and/or efforts of the joints at each time point.
JointTrajectoryPoint[] points
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg.

# Single DOF joint positions for each joint relative to their "0" position.
# The units depend on the specific joint type: radians for revolute or
# continuous joints, and meters for prismatic joints.
float64[] positions

# The rate of change in position of each joint. Units are joint type dependent.
# Radians/second for revolute or continuous joints, and meters/second for
# prismatic joints.
float64[] velocities

# Rate of change in velocity of each joint. Units are joint type dependent.
# Radians/second^2 for revolute or continuous joints, and meters/second^2 for
# prismatic joints.
float64[] accelerations

# The torque or the force to be applied at each joint. For revolute/continuous
# joints effort denotes a torque in newton-meters. For prismatic joints, effort
# denotes a force in newtons.
float64[] effort

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
================================================================================
MSG: builtin_interfaces/Duration
# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_179b33eba59d676f6d967ac71fe35e7ca2f64b2f3928f4a018cec115e213796e";
        const ROS2_TYPE_NAME: &'static str = "trajectory_msgs::msg::dds_::JointTrajectory_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct JointTrajectoryPoint {
        pub r#positions: ::std::vec::Vec<f64>,
        pub r#velocities: ::std::vec::Vec<f64>,
        pub r#accelerations: ::std::vec::Vec<f64>,
        pub r#effort: ::std::vec::Vec<f64>,
        pub r#time_from_start: builtin_interfaces::Duration,
    }
    impl ::roslibrust::RosMessageType for JointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectoryPoint";
        const MD5SUM: &'static str = "ed0b8591d35f39b08abca91b0130e2e4";
        const DEFINITION: &'static str = r####"# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg.

# Single DOF joint positions for each joint relative to their "0" position.
# The units depend on the specific joint type: radians for revolute or
# continuous joints, and meters for prismatic joints.
float64[] positions

# The rate of change in position of each joint. Units are joint type dependent.
# Radians/second for revolute or continuous joints, and meters/second for
# prismatic joints.
float64[] velocities

# Rate of change in velocity of each joint. Units are joint type dependent.
# Radians/second^2 for revolute or continuous joints, and meters/second^2 for
# prismatic joints.
float64[] accelerations

# The torque or the force to be applied at each joint. For revolute/continuous
# joints effort denotes a torque in newton-meters. For prismatic joints, effort
# denotes a force in newtons.
float64[] effort

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
================================================================================
MSG: builtin_interfaces/Duration
# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_de8907036d8bd45aac6f30cc9044a3d4a329c42cbf719aff7d95a584cfa532d7";
        const ROS2_TYPE_NAME: &'static str = "trajectory_msgs::msg::dds_::JointTrajectoryPoint_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MultiDOFJointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::MultiDOFJointTrajectoryPoint>,
    }
    impl ::roslibrust::RosMessageType for MultiDOFJointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectory";
        const MD5SUM: &'static str = "8683899094e1d16af9df4c8f8ce7a2c9";
        const DEFINITION: &'static str = r####"# The header is used to specify the coordinate frame and the reference time for the trajectory durations
std_msgs/Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as 
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint
# Each multi-dof joint can specify a transform (up to 6 DOF).
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint.
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint.
geometry_msgs/Twist[] accelerations

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3a18fd095292a65cfde8833c72985a30af981f3ec44494655c6267262b443a4a";
        const ROS2_TYPE_NAME: &'static str = "trajectory_msgs::msg::dds_::MultiDOFJointTrajectory_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MultiDOFJointTrajectoryPoint {
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#velocities: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#accelerations: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#time_from_start: builtin_interfaces::Duration,
    }
    impl ::roslibrust::RosMessageType for MultiDOFJointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectoryPoint";
        const MD5SUM: &'static str = "b420cff1db93574a740e7e2ceb169c84";
        const DEFINITION: &'static str = r####"# Each multi-dof joint can specify a transform (up to 6 DOF).
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint.
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint.
geometry_msgs/Twist[] accelerations

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"####;
        const ROS2_HASH: &'static str =
            "RIHS01_6ada1085b5ee64eaa069b074968e69f0e27c8c5e6f5bb0586dd1c834ef0e32b8";
        const ROS2_TYPE_NAME: &'static str =
            "trajectory_msgs::msg::dds_::MultiDOFJointTrajectoryPoint_";
    }
}
#[allow(unused_imports)]
pub mod visualization_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::ros2_test_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct ImageMarker {
        pub r#header: std_msgs::Header,
        pub r#ns: ::std::string::String,
        pub r#id: i32,
        pub r#type: i32,
        pub r#action: i32,
        pub r#position: geometry_msgs::Point,
        pub r#scale: f32,
        pub r#outline_color: std_msgs::ColorRGBA,
        pub r#filled: u8,
        pub r#fill_color: std_msgs::ColorRGBA,
        pub r#lifetime: builtin_interfaces::Duration,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point>,
        pub r#outline_colors: ::std::vec::Vec<std_msgs::ColorRGBA>,
    }
    impl ::roslibrust::RosMessageType for ImageMarker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/ImageMarker";
        const MD5SUM: &'static str = "e521402e7d84055a370dd7eea688def8";
        const DEFINITION: &'static str = r####"int32 CIRCLE=0
int32 LINE_STRIP=1
int32 LINE_LIST=2
int32 POLYGON=3
int32 POINTS=4

int32 ADD=0
int32 REMOVE=1

std_msgs/Header header
# Namespace which is used with the id to form a unique id.
string ns
# Unique id within the namespace.
int32 id
# One of the above types, e.g. CIRCLE, LINE_STRIP, etc.
int32 type
# Either ADD or REMOVE.
int32 action
# Two-dimensional coordinate position, in pixel-coordinates.
geometry_msgs/Point position
# The scale of the object, e.g. the diameter for a CIRCLE.
float32 scale
# The outline color of the marker.
std_msgs/ColorRGBA outline_color
# Whether or not to fill in the shape with color.
uint8 filled
# Fill color; in the range: [0.0-1.0]
std_msgs/ColorRGBA fill_color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime

# Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.
geometry_msgs/Point[] points
# The color for each line, point, etc. in the points field.
std_msgs/ColorRGBA[] outline_colors
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_603152491ef2331c200a5305230d31f6e8704875944b388da0f547c415d11836";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::ImageMarker_";
    }
    #[allow(unused)]
    impl ImageMarker {
        pub const r#CIRCLE: i32 = 0i32;
        pub const r#LINE_STRIP: i32 = 1i32;
        pub const r#LINE_LIST: i32 = 2i32;
        pub const r#POLYGON: i32 = 3i32;
        pub const r#POINTS: i32 = 4i32;
        pub const r#ADD: i32 = 0i32;
        pub const r#REMOVE: i32 = 1i32;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InteractiveMarker {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
        pub r#description: ::std::string::String,
        pub r#scale: f32,
        pub r#menu_entries: ::std::vec::Vec<self::MenuEntry>,
        pub r#controls: ::std::vec::Vec<self::InteractiveMarkerControl>,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarker";
        const MD5SUM: &'static str = "8c084b95d02f38cac26368fe4ad91dad";
        const DEFINITION: &'static str = r####"# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_3d5b51448b51d73b0f395b94d259edd3a5d269ae9f7d9fd5cceb9ae4b72be346";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::InteractiveMarker_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InteractiveMarkerControl {
        pub r#name: ::std::string::String,
        pub r#orientation: geometry_msgs::Quaternion,
        pub r#orientation_mode: u8,
        pub r#interaction_mode: u8,
        pub r#always_visible: bool,
        pub r#markers: ::std::vec::Vec<self::Marker>,
        pub r#independent_marker_orientation: bool,
        pub r#description: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerControl {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerControl";
        const MD5SUM: &'static str = "22d6471be9b304bf94ff7e0b8a173aec";
        const DEFINITION: &'static str = r####"# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_60e2fa36344f5f4791b24a809542a18bffd555f563550d4b22b3bbfc31ec0ed5";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::InteractiveMarkerControl_";
    }
    #[allow(unused)]
    impl InteractiveMarkerControl {
        pub const r#INHERIT: u8 = 0u8;
        pub const r#FIXED: u8 = 1u8;
        pub const r#VIEW_FACING: u8 = 2u8;
        pub const r#NONE: u8 = 0u8;
        pub const r#MENU: u8 = 1u8;
        pub const r#BUTTON: u8 = 2u8;
        pub const r#MOVE_AXIS: u8 = 3u8;
        pub const r#MOVE_PLANE: u8 = 4u8;
        pub const r#ROTATE_AXIS: u8 = 5u8;
        pub const r#MOVE_ROTATE: u8 = 6u8;
        pub const r#MOVE_3D: u8 = 7u8;
        pub const r#ROTATE_3D: u8 = 8u8;
        pub const r#MOVE_ROTATE_3D: u8 = 9u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InteractiveMarkerFeedback {
        pub r#header: std_msgs::Header,
        pub r#client_id: ::std::string::String,
        pub r#marker_name: ::std::string::String,
        pub r#control_name: ::std::string::String,
        pub r#event_type: u8,
        pub r#pose: geometry_msgs::Pose,
        pub r#menu_entry_id: u32,
        pub r#mouse_point: geometry_msgs::Point,
        pub r#mouse_point_valid: bool,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerFeedback {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerFeedback";
        const MD5SUM: &'static str = "db5dbc58cd804aa8ba18abc7fc554cd2";
        const DEFINITION: &'static str = r####"# Time/frame info.
std_msgs/Header header

# Identifying string. Must be unique in the topic namespace.
string client_id

# Feedback message sent back from the GUI, e.g.
# when the status of an interactive marker was modified by the user.

# Specifies which interactive marker and control this message refers to
string marker_name
string control_name

# Type of the event
# KEEP_ALIVE: sent while dragging to keep up control of the marker
# MENU_SELECT: a menu entry has been selected
# BUTTON_CLICK: a button control has been clicked
# POSE_UPDATE: the pose has been changed using one of the controls
uint8 KEEP_ALIVE = 0
uint8 POSE_UPDATE = 1
uint8 MENU_SELECT = 2
uint8 BUTTON_CLICK = 3

uint8 MOUSE_DOWN = 4
uint8 MOUSE_UP = 5

uint8 event_type

# Current pose of the marker
# Note: Has to be valid for all feedback types.
geometry_msgs/Pose pose

# Contains the ID of the selected menu entry
# Only valid for MENU_SELECT events.
uint32 menu_entry_id

# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
# may contain the 3 dimensional position of the event on the
# control.  If it does, mouse_point_valid will be true.  mouse_point
# will be relative to the frame listed in the header.
geometry_msgs/Point mouse_point
bool mouse_point_valid
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_6cc48741df9f05d19ba7d9ea3101e9fcd1309c9d6bda3c55668ba607492f725e";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::InteractiveMarkerFeedback_";
    }
    #[allow(unused)]
    impl InteractiveMarkerFeedback {
        pub const r#KEEP_ALIVE: u8 = 0u8;
        pub const r#POSE_UPDATE: u8 = 1u8;
        pub const r#MENU_SELECT: u8 = 2u8;
        pub const r#BUTTON_CLICK: u8 = 3u8;
        pub const r#MOUSE_DOWN: u8 = 4u8;
        pub const r#MOUSE_UP: u8 = 5u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InteractiveMarkerInit {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerInit {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerInit";
        const MD5SUM: &'static str = "c96647886404b1a9e49d10038aec586d";
        const DEFINITION: &'static str = r####"# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed a subsequent
# update.  Every update message will have the same sequence number as
# an init message.  Clients will likely want to unsubscribe from the
# init topic after a successful initialization to avoid receiving
# duplicate data.
uint64 seq_num

# All markers.
InteractiveMarker[] markers
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_23fda1b3373b154d9d6408dd7b9f8129b2a2b76b905ee421e8c4109d8bf71f78";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::InteractiveMarkerInit_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InteractiveMarkerPose {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerPose {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerPose";
        const MD5SUM: &'static str = "6b733efaf67a5df110b196fd403982e8";
        const DEFINITION: &'static str = r####"# Time/frame info.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####;
        const ROS2_HASH: &'static str =
            "RIHS01_c60e9a4407d5f709a63e0fe9caea324aee08fe717cd090209ebe35012ce7cb66";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::InteractiveMarkerPose_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct InteractiveMarkerUpdate {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#type: u8,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
        pub r#poses: ::std::vec::Vec<self::InteractiveMarkerPose>,
        pub r#erases: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerUpdate {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerUpdate";
        const MD5SUM: &'static str = "afc698521287ea99dbcb170c75561770";
        const DEFINITION: &'static str = r####"# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed an update.
uint64 seq_num

# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
# UPDATE: Incremental update to previous state.
#         The sequence number must be 1 higher than for
#         the previous update.
# KEEP_ALIVE: Indicates the that the server is still living.
#             The sequence number does not increase.
#             No payload data should be filled out (markers, poses, or erases).
uint8 KEEP_ALIVE = 0
uint8 UPDATE = 1

uint8 type

# Note: No guarantees on the order of processing.
#       Contents must be kept consistent by sender.

# Markers to be added or updated
InteractiveMarker[] markers

# Poses of markers that should be moved
InteractiveMarkerPose[] poses

# Names of markers to be erased
string[] erases
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerPose
# Time/frame info.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_0a8b000c4fd4d50876ac716a7de018911599a2f015795388e956a2ca8b0c54f0";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::InteractiveMarkerUpdate_";
    }
    #[allow(unused)]
    impl InteractiveMarkerUpdate {
        pub const r#KEEP_ALIVE: u8 = 0u8;
        pub const r#UPDATE: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct Marker {
        pub r#header: std_msgs::Header,
        pub r#ns: ::std::string::String,
        pub r#id: i32,
        pub r#type: i32,
        pub r#action: i32,
        pub r#pose: geometry_msgs::Pose,
        pub r#scale: geometry_msgs::Vector3,
        pub r#color: std_msgs::ColorRGBA,
        pub r#lifetime: builtin_interfaces::Duration,
        pub r#frame_locked: bool,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point>,
        pub r#colors: ::std::vec::Vec<std_msgs::ColorRGBA>,
        pub r#texture_resource: ::std::string::String,
        pub r#texture: sensor_msgs::CompressedImage,
        pub r#uv_coordinates: ::std::vec::Vec<self::UVCoordinate>,
        pub r#text: ::std::string::String,
        pub r#mesh_resource: ::std::string::String,
        pub r#mesh_file: self::MeshFile,
        pub r#mesh_use_embedded_materials: bool,
    }
    impl ::roslibrust::RosMessageType for Marker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/Marker";
        const MD5SUM: &'static str = "de7e324ce859da7e8e3049312d09493d";
        const DEFINITION: &'static str = r####"# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_45b13ccf791f225962bf74e746f9644518855d783a6f42ba0cc14fde2b4f3ce0";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::Marker_";
    }
    #[allow(unused)]
    impl Marker {
        pub const r#ARROW: i32 = 0i32;
        pub const r#CUBE: i32 = 1i32;
        pub const r#SPHERE: i32 = 2i32;
        pub const r#CYLINDER: i32 = 3i32;
        pub const r#LINE_STRIP: i32 = 4i32;
        pub const r#LINE_LIST: i32 = 5i32;
        pub const r#CUBE_LIST: i32 = 6i32;
        pub const r#SPHERE_LIST: i32 = 7i32;
        pub const r#POINTS: i32 = 8i32;
        pub const r#TEXT_VIEW_FACING: i32 = 9i32;
        pub const r#MESH_RESOURCE: i32 = 10i32;
        pub const r#TRIANGLE_LIST: i32 = 11i32;
        pub const r#ADD: i32 = 0i32;
        pub const r#MODIFY: i32 = 0i32;
        pub const r#DELETE: i32 = 2i32;
        pub const r#DELETEALL: i32 = 3i32;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MarkerArray {
        pub r#markers: ::std::vec::Vec<self::Marker>,
    }
    impl ::roslibrust::RosMessageType for MarkerArray {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MarkerArray";
        const MD5SUM: &'static str = "a5f21be3bc47b9dc10a485a0f8f72200";
        const DEFINITION: &'static str = r####"Marker[] markers
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_86cb8800b6fb05b5eff1abd7a56f62a5641d3ae9a1c29e78e67e704f1d067dcf";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::MarkerArray_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MenuEntry {
        pub r#id: u32,
        pub r#parent_id: u32,
        pub r#title: ::std::string::String,
        pub r#command: ::std::string::String,
        pub r#command_type: u8,
    }
    impl ::roslibrust::RosMessageType for MenuEntry {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MenuEntry";
        const MD5SUM: &'static str = "b90ec63024573de83b57aa93eb39be2d";
        const DEFINITION: &'static str = r####"# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type"####;
        const ROS2_HASH: &'static str =
            "RIHS01_22170c387c70fd4236232ec902de8604e72ff027342c7c0f28ad9f68c64c51d6";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::MenuEntry_";
    }
    #[allow(unused)]
    impl MenuEntry {
        pub const r#FEEDBACK: u8 = 0u8;
        pub const r#ROSRUN: u8 = 1u8;
        pub const r#ROSLAUNCH: u8 = 2u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct MeshFile {
        pub r#filename: ::std::string::String,
        #[serde(with = "::roslibrust::codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for MeshFile {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MeshFile";
        const MD5SUM: &'static str = "39f264648e441626a1045a7d9ef1ba17";
        const DEFINITION: &'static str = r####"# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data"####;
        const ROS2_HASH: &'static str =
            "RIHS01_7710ece15a148fb7c9b546364cfb215bb06098087bd6394fe5b73a493508f8c4";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::MeshFile_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct UVCoordinate {
        pub r#u: f32,
        pub r#v: f32,
    }
    impl ::roslibrust::RosMessageType for UVCoordinate {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/UVCoordinate";
        const MD5SUM: &'static str = "4f5254e0e12914c461d4b17a0cd07f7f";
        const DEFINITION: &'static str = r####"# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_f27f7ed21fe360c6066944f856b801a0c0d1e94e815b6886444b42d90b196a26";
        const ROS2_TYPE_NAME: &'static str = "visualization_msgs::msg::dds_::UVCoordinate_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GetInteractiveMarkersRequest {}
    impl ::roslibrust::RosMessageType for GetInteractiveMarkersRequest {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r####""####;
        const ROS2_HASH: &'static str =
            "RIHS01_9e90c011a0f639ae1f5123d49b7af15592f7c97d0ad0ef151a33193946004b00";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::GetInteractiveMarkersRequest_";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    pub struct GetInteractiveMarkersResponse {
        pub r#sequence_number: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust::RosMessageType for GetInteractiveMarkersResponse {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersResponse";
        const MD5SUM: &'static str = "3498b331821fc65db485e759de4734e7";
        const DEFINITION: &'static str = r####"# Sequence number.
# Set to the sequence number of the latest update message
# at the time the server received the request.
# Clients use this to detect if any updates were missed.
uint64 sequence_number

# All interactive markers provided by the server.
InteractiveMarker[] markers
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: builtin_interfaces/Duration
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
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"####;
        const ROS2_HASH: &'static str =
            "RIHS01_cd1070bccfa143ff8ae86f53b568c8cace24a12b73a2743587a1c21b2ba0e0f1";
        const ROS2_TYPE_NAME: &'static str =
            "visualization_msgs::msg::dds_::GetInteractiveMarkersResponse_";
    }
    #[allow(dead_code)]
    pub struct GetInteractiveMarkers {}
    impl ::roslibrust::RosServiceType for GetInteractiveMarkers {
        const ROS_SERVICE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkers";
        const MD5SUM: &'static str = "3498b331821fc65db485e759de4734e7";
        const ROS2_HASH: &'static str =
            "RIHS01_bfdfaa861f4a8422b4ec3bd02c6681e4a45a83bdc2913d8facdf923d8ad1376a";
        type Request = GetInteractiveMarkersRequest;
        type Response = GetInteractiveMarkersResponse;
    }
}
