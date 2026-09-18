#[allow(unused_imports)]
pub mod actionlib_msgs {
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GoalID {
        pub r#stamp: ::roslibrust::codegen::integral_types::Time,
        pub r#id: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        GoalID,
        "actionlib_msgs/GoalID",
        "302881f31927c1df708a2dbab0e80ee8",
        r####"# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id"####,
        "actionlib_msgs::msg::dds_::GoalID_",
        &[
            0xfb, 0xfc, 0x57, 0xa3, 0xed, 0x46, 0x0a, 0xf3, 0x14, 0x62, 0x47, 0xe8, 0x16, 0xa1,
            0x52, 0xe4, 0xfd, 0xff, 0x4a, 0x96, 0x79, 0x40, 0xaf, 0x4d, 0x11, 0xac, 0x57, 0xfd,
            0x9c, 0x83, 0x47, 0x23,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GoalStatus {
        pub r#goal_id: self::GoalID,
        pub r#status: u8,
        pub r#text: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        GoalStatus,
        "actionlib_msgs/GoalStatus",
        "d388f9b87b3c471f784434d671988d4a",
        r####"GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id"####,
        "actionlib_msgs::msg::dds_::GoalStatus_",
        &[
            0x1c, 0xa4, 0xd4, 0x75, 0x77, 0x2f, 0xb7, 0x3b, 0x1f, 0x8c, 0xbb, 0xab, 0x8e, 0x60,
            0x31, 0xdb, 0x7e, 0xe6, 0xf2, 0xfe, 0xf7, 0xd5, 0xa1, 0x39, 0xaf, 0x2f, 0x9e, 0xe3,
            0xd6, 0xa1, 0xc2, 0x2f,
        ],
    );
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
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GoalStatusArray {
        pub r#header: std_msgs::Header,
        pub r#status_list: ::std::vec::Vec<self::GoalStatus>,
    }
    ::roslibrust::impl_ros_message_type!(
        GoalStatusArray,
        "actionlib_msgs/GoalStatusArray",
        "8b2b82f13216d0a8ea88bd3af735e619",
        r####"# Stores the statuses for goals that are currently being tracked
# by an action server
Header header
GoalStatus[] status_list
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "actionlib_msgs::msg::dds_::GoalStatusArray_",
        &[
            0xfb, 0x3e, 0x9f, 0x9c, 0x97, 0x35, 0x32, 0x44, 0xf5, 0x31, 0xc6, 0x40, 0xd3, 0xb1,
            0x72, 0xd2, 0x40, 0x74, 0xe2, 0xc6, 0x69, 0x04, 0x8a, 0xc9, 0x94, 0x7f, 0x4a, 0x4f,
            0xc1, 0x1f, 0xad, 0xda,
        ],
    );
}
#[allow(unused_imports)]
pub mod builtin_interfaces {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Duration {
        pub r#sec: i32,
        pub r#nanosec: u32,
    }
    ::roslibrust::impl_ros_message_type!(
        Duration,
        "builtin_interfaces/Duration",
        "8255142433c342f21ece78aae48f7907",
        r####"# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####,
        "builtin_interfaces::msg::dds_::Duration_",
        &[
            0xe8, 0xd0, 0x09, 0xf6, 0x59, 0x81, 0x6f, 0x75, 0x8b, 0x75, 0x33, 0x4e, 0xe1, 0xa9,
            0xca, 0x5b, 0x5c, 0x0b, 0x85, 0x98, 0x43, 0x26, 0x1f, 0x14, 0xc7, 0xf9, 0x37, 0x34,
            0x95, 0x99, 0xd9, 0x3b,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Time {
        pub r#sec: i32,
        pub r#nanosec: u32,
    }
    ::roslibrust::impl_ros_message_type!(
        Time,
        "builtin_interfaces/Time",
        "8255142433c342f21ece78aae48f7907",
        r####"# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec"####,
        "builtin_interfaces::msg::dds_::Time_",
        &[
            0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed, 0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a,
            0x3e, 0xe9, 0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e, 0x42, 0x06, 0xce, 0xa9,
            0xac, 0xf9, 0xc1, 0x97,
        ],
    );
}
#[allow(unused_imports)]
pub mod diagnostic_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct DiagnosticArray {
        pub r#header: std_msgs::Header,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    ::roslibrust::impl_ros_message_type!(
        DiagnosticArray,
        "diagnostic_msgs/DiagnosticArray",
        "60810da900de1dd6ddd437c3503511da",
        r####"# This message is used to send diagnostic information about the state of the robot
Header header #for timestamp
DiagnosticStatus[] status # an array of components being reported on
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status
================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time
================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "diagnostic_msgs::msg::dds_::DiagnosticArray_",
        &[
            0xfd, 0x7d, 0x84, 0x9d, 0x56, 0xd0, 0xdb, 0xa5, 0x4a, 0xfc, 0x9e, 0x86, 0x2c, 0xd5,
            0x1a, 0x41, 0xf1, 0xd3, 0x95, 0xf6, 0x67, 0xc4, 0xf4, 0xd5, 0x31, 0x0c, 0x64, 0x90,
            0x50, 0x1a, 0x95, 0x1e,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct DiagnosticStatus {
        pub r#level: u8,
        pub r#name: ::std::string::String,
        pub r#message: ::std::string::String,
        pub r#hardware_id: ::std::string::String,
        pub r#values: ::std::vec::Vec<self::KeyValue>,
    }
    ::roslibrust::impl_ros_message_type!(
        DiagnosticStatus,
        "diagnostic_msgs/DiagnosticStatus",
        "d0ce08bc6e5ba34c7754f563a9cabaf1",
        r####"# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status
================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time"####,
        "diagnostic_msgs::msg::dds_::DiagnosticStatus_",
        &[
            0x40, 0x5f, 0xee, 0x4f, 0xda, 0xd6, 0xcc, 0xe5, 0x34, 0x1d, 0x94, 0x4a, 0xc7, 0x82,
            0xeb, 0x84, 0x9e, 0x41, 0x40, 0x27, 0xbb, 0xe6, 0x72, 0xca, 0xff, 0xf5, 0x02, 0x30,
            0x2f, 0x03, 0x80, 0x2b,
        ],
    );
    #[allow(unused)]
    impl DiagnosticStatus {
        pub const r#OK: u8 = 0u8;
        pub const r#WARN: u8 = 1u8;
        pub const r#ERROR: u8 = 2u8;
        pub const r#STALE: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct KeyValue {
        pub r#key: ::std::string::String,
        pub r#value: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        KeyValue,
        "diagnostic_msgs/KeyValue",
        "cf57fdc6617a881a88c16e768132149c",
        r####"string key # what to label this value when viewing
string value # a value to track over time"####,
        "diagnostic_msgs::msg::dds_::KeyValue_",
        &[
            0xd6, 0x80, 0x81, 0xea, 0xa5, 0x40, 0x28, 0x8c, 0x54, 0x40, 0x75, 0x3b, 0xae, 0xce,
            0xf0, 0xc4, 0xe1, 0x6e, 0x81, 0xa5, 0xf7, 0x8a, 0xd6, 0x89, 0x02, 0xde, 0xd5, 0x10,
            0x04, 0x13, 0xbb, 0x42,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AddDiagnosticsRequest {
        pub r#load_namespace: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        AddDiagnosticsRequest,
        "diagnostic_msgs/AddDiagnosticsRequest",
        "c26cf6e164288fbc6050d74f838bcdf0",
        r####"# This service is used as part of the process for loading analyzers at runtime,
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
string load_namespace"####,
        "diagnostic_msgs::msg::dds_::AddDiagnosticsRequest_",
        &[
            0x33, 0x75, 0xe0, 0x4f, 0x9f, 0x4d, 0x40, 0x6c, 0x7b, 0x3c, 0x8a, 0xaa, 0x01, 0x6a,
            0x29, 0xaf, 0x26, 0x90, 0xfc, 0xad, 0x4f, 0xa6, 0x00, 0x7f, 0x46, 0xc2, 0x21, 0xc3,
            0x3f, 0x8c, 0x82, 0xd4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AddDiagnosticsResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        AddDiagnosticsResponse,
        "diagnostic_msgs/AddDiagnosticsResponse",
        "937c9679a518e3a18d831e57125ea522",
        r####"# True if diagnostic aggregator was updated with new diagnostics, False
# otherwise. A false return value means that either there is a bond in the
# aggregator which already used the requested namespace, or the initialization
# of analyzers failed.
bool success

# Message with additional information about the success or failure
string message"####,
        "diagnostic_msgs::msg::dds_::AddDiagnosticsResponse_",
        &[
            0xe3, 0xaa, 0x26, 0x75, 0x7b, 0xf7, 0x12, 0xcd, 0x2c, 0x48, 0xe9, 0x4f, 0x12, 0x01,
            0x07, 0x0d, 0x8d, 0xab, 0x40, 0x1d, 0x1c, 0xd0, 0xd8, 0x85, 0x3a, 0xb0, 0xf4, 0x57,
            0x9d, 0xca, 0x5e, 0xac,
        ],
    );
    #[allow(dead_code)]
    pub struct AddDiagnostics {}
    impl ::roslibrust::RosServiceType for AddDiagnostics {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/AddDiagnostics";
        const MD5SUM: &'static str = "e6ac9bbde83d0d3186523c3687aecaee";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x6b, 0x91, 0x08, 0x4c, 0x9f, 0x8b, 0xff, 0xd9, 0xb0, 0xc1, 0xf6, 0x88, 0x3e, 0x0e,
            0x8e, 0x49, 0x1a, 0x83, 0x1f, 0x4f, 0xc6, 0xc6, 0xae, 0xc7, 0x7a, 0x4e, 0xfa, 0xf3,
            0xf1, 0xea, 0x90, 0xb5,
        ];
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::srv::dds_::AddDiagnostics_";
        type Request = AddDiagnosticsRequest;
        type Response = AddDiagnosticsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SelfTestRequest {}
    ::roslibrust::impl_ros_message_type!(
        SelfTestRequest,
        "diagnostic_msgs/SelfTestRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "diagnostic_msgs::msg::dds_::SelfTestRequest_",
        &[
            0xf3, 0xda, 0x6c, 0xba, 0x81, 0x55, 0xce, 0x26, 0xbb, 0x81, 0xb0, 0x29, 0xb6, 0x3b,
            0xb3, 0x11, 0x07, 0x3c, 0x6c, 0xb4, 0x18, 0x38, 0x48, 0xa3, 0x7c, 0xc1, 0x6e, 0xfe,
            0x3d, 0x77, 0xb2, 0x29,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SelfTestResponse {
        pub r#id: ::std::string::String,
        pub r#passed: u8,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    ::roslibrust::impl_ros_message_type!(
        SelfTestResponse,
        "diagnostic_msgs/SelfTestResponse",
        "ac21b1bab7ab17546986536c22eb34e9",
        r####"string id
byte passed
DiagnosticStatus[] status
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status
================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time
================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time"####,
        "diagnostic_msgs::msg::dds_::SelfTestResponse_",
        &[
            0x4b, 0x6d, 0xd7, 0x0e, 0xe9, 0xf2, 0x0c, 0xeb, 0xad, 0x75, 0x93, 0x31, 0x53, 0x0f,
            0xd0, 0xde, 0x79, 0xbc, 0xf3, 0xdc, 0x8b, 0x63, 0x72, 0x78, 0x3e, 0xf9, 0x35, 0x36,
            0x6b, 0xc4, 0xce, 0x46,
        ],
    );
    #[allow(dead_code)]
    pub struct SelfTest {}
    impl ::roslibrust::RosServiceType for SelfTest {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/SelfTest";
        const MD5SUM: &'static str = "ac21b1bab7ab17546986536c22eb34e9";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x0e, 0x0f, 0xc2, 0x75, 0x4c, 0xad, 0xcf, 0x8d, 0xd5, 0x74, 0x59, 0x80, 0x6c, 0x78,
            0xa7, 0xc3, 0xb3, 0x1d, 0xdc, 0x65, 0xb0, 0x11, 0xa6, 0x30, 0xfd, 0x8c, 0xe3, 0xbc,
            0xdd, 0xc0, 0x05, 0x83,
        ];
        const ROS2_TYPE_NAME: &'static str = "diagnostic_msgs::srv::dds_::SelfTest_";
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
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Accel {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    ::roslibrust::impl_ros_message_type!(
        Accel,
        "geometry_msgs/Accel",
        "9f195f881246fdfa2798d1d3eebca84a",
        r####"# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Accel_",
        &[
            0xdc, 0x44, 0x82, 0x43, 0xde, 0xd9, 0xb1, 0xfc, 0xbc, 0xca, 0x24, 0xab, 0xa0, 0xc2,
            0x2f, 0x01, 0x3d, 0xae, 0x06, 0xc3, 0x54, 0xba, 0x2d, 0x84, 0x95, 0x71, 0xc0, 0xa2,
            0xa3, 0xf5, 0x7c, 0xa0,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AccelStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::Accel,
    }
    ::roslibrust::impl_ros_message_type!(
        AccelStamped,
        "geometry_msgs/AccelStamped",
        "d8a98a5d81351b6eb0578c78557e7659",
        r####"# An accel with reference coordinate frame and timestamp
Header header
Accel accel
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::AccelStamped_",
        &[
            0x47, 0xf7, 0x47, 0xd2, 0x60, 0x91, 0xb8, 0x05, 0x7b, 0xd8, 0xe4, 0xc4, 0x9d, 0x9d,
            0x5c, 0xea, 0xcf, 0x6a, 0xe3, 0x92, 0xc1, 0xcc, 0xea, 0x53, 0x22, 0x3f, 0x34, 0x6f,
            0x9c, 0x59, 0x53, 0x2b,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AccelWithCovariance {
        pub r#accel: self::Accel,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust::codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    ::roslibrust::impl_ros_message_type!(
        AccelWithCovariance,
        "geometry_msgs/AccelWithCovariance",
        "ad5a718d699c6be72a02b8d6a139f334",
        r####"# This expresses acceleration in free space with uncertainty.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::AccelWithCovariance_",
        &[
            0x23, 0x0d, 0x51, 0xbd, 0x53, 0xbc, 0x36, 0xf2, 0x60, 0x57, 0x4e, 0x73, 0xb4, 0x29,
            0x41, 0xce, 0xfe, 0x44, 0x68, 0x47, 0x53, 0x48, 0x0b, 0x6f, 0xc3, 0x30, 0xc0, 0x32,
            0xc5, 0xdb, 0x59, 0x97,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AccelWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::AccelWithCovariance,
    }
    ::roslibrust::impl_ros_message_type!(
        AccelWithCovarianceStamped,
        "geometry_msgs/AccelWithCovarianceStamped",
        "96adb295225031ec8d57fb4251b0a886",
        r####"# This represents an estimated accel with reference coordinate frame and timestamp.
Header header
AccelWithCovariance accel
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::AccelWithCovarianceStamped_",
        &[
            0xa3, 0x03, 0x65, 0xcc, 0x01, 0x7d, 0x2c, 0x85, 0xeb, 0x2d, 0x31, 0x0b, 0xb4, 0xf7,
            0x81, 0xce, 0x0d, 0x36, 0x76, 0x22, 0xb0, 0x6e, 0x78, 0x50, 0x8f, 0x00, 0x0f, 0x37,
            0x17, 0x76, 0x14, 0xd6,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
    ::roslibrust::impl_ros_message_type!(
        Inertia,
        "geometry_msgs/Inertia",
        "1d26e4bb6c83ff141c5cf0d883c2b0fe",
        r####"# Mass [kg]
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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Inertia_",
        &[
            0x2d, 0xdd, 0x5d, 0xab, 0x5c, 0x34, 0x78, 0x25, 0xba, 0x2e, 0x56, 0xc8, 0x95, 0xdd,
            0xcc, 0xfd, 0x0b, 0x8e, 0xfe, 0x53, 0xae, 0x93, 0x1b, 0xf6, 0x7f, 0x90, 0x55, 0x29,
            0x93, 0x0b, 0x4b, 0xd7,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct InertiaStamped {
        pub r#header: std_msgs::Header,
        pub r#inertia: self::Inertia,
    }
    ::roslibrust::impl_ros_message_type!(
        InertiaStamped,
        "geometry_msgs/InertiaStamped",
        "ddee48caeab5a966c5e8d166654a9ac7",
        r####"Header header
Inertia inertia
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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::InertiaStamped_",
        &[
            0xbe, 0x07, 0x62, 0x73, 0xdb, 0xb8, 0xe5, 0xac, 0x72, 0x23, 0x2e, 0x91, 0xe5, 0x9b,
            0xc3, 0x95, 0x63, 0xca, 0x1d, 0xb7, 0x7a, 0xd2, 0x0c, 0x2f, 0xa0, 0x3f, 0x1d, 0xf0,
            0x61, 0xdb, 0x7d, 0xe9,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Point {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Point,
        "geometry_msgs/Point",
        "4a842b65f413084dc2b10fb484ea7f17",
        r####"# This contains the position of a point in free space
float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Point_",
        &[
            0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44, 0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11,
            0x44, 0x47, 0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84, 0x3b, 0x9c, 0x43, 0xf4,
            0x2a, 0x7f, 0x68, 0x81,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Point32 {
        pub r#x: f32,
        pub r#y: f32,
        pub r#z: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        Point32,
        "geometry_msgs/Point32",
        "cc153912f1453b708d221682bc23d9ac",
        r####"# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"####,
        "geometry_msgs::msg::dds_::Point32_",
        &[
            0x2f, 0xc4, 0xdb, 0x7c, 0xae, 0x16, 0xa4, 0x58, 0x2c, 0x79, 0xa5, 0x6b, 0x66, 0x17,
            0x3a, 0x8d, 0x48, 0xd5, 0x2c, 0x7d, 0xc5, 0x20, 0xdd, 0xc5, 0x5a, 0x0d, 0x4b, 0xcf,
            0x2a, 0x4b, 0xfd, 0xbc,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PointStamped {
        pub r#header: std_msgs::Header,
        pub r#point: self::Point,
    }
    ::roslibrust::impl_ros_message_type!(
        PointStamped,
        "geometry_msgs/PointStamped",
        "c63aecb41bfdfd6b7e1fac37c7cbe7bf",
        r####"# This represents a Point with reference coordinate frame and timestamp
Header header
Point point
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::PointStamped_",
        &[
            0x16, 0x48, 0x46, 0x59, 0xa6, 0x38, 0x7c, 0xf8, 0xf1, 0xda, 0x65, 0x26, 0xb9, 0x4b,
            0x8d, 0xa0, 0x6a, 0xc4, 0x8d, 0x0c, 0x03, 0xc9, 0x5e, 0x5b, 0xf7, 0xec, 0x4c, 0x08,
            0xfe, 0xbb, 0x77, 0x29,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Polygon {
        pub r#points: ::std::vec::Vec<self::Point32>,
    }
    ::roslibrust::impl_ros_message_type!(
        Polygon,
        "geometry_msgs/Polygon",
        "cd60a26494a087f577976f0329fa120e",
        r####"#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"####,
        "geometry_msgs::msg::dds_::Polygon_",
        &[
            0x37, 0x82, 0xf9, 0xf0, 0xbf, 0x04, 0x49, 0x64, 0xd6, 0x92, 0xd6, 0xc0, 0x17, 0xd7,
            0x05, 0xe3, 0x76, 0x11, 0xaf, 0xb1, 0xf0, 0xbf, 0x6a, 0x9d, 0xee, 0x24, 0x8a, 0x7d,
            0xda, 0x0f, 0x78, 0x4a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PolygonStamped {
        pub r#header: std_msgs::Header,
        pub r#polygon: self::Polygon,
    }
    ::roslibrust::impl_ros_message_type!(
        PolygonStamped,
        "geometry_msgs/PolygonStamped",
        "c6be8f7dc3bee7fe9e8d296070f53340",
        r####"# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
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
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::PolygonStamped_",
        &[
            0xff, 0xfe, 0xb2, 0xb4, 0xa8, 0xd7, 0x52, 0xeb, 0x71, 0xf4, 0x2f, 0x5e, 0xe5, 0xbf,
            0x2e, 0x0a, 0x78, 0x7d, 0x18, 0xd1, 0xcc, 0x94, 0x03, 0xea, 0x8d, 0x87, 0x8d, 0x1e,
            0x8f, 0x88, 0x07, 0xaf,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Pose {
        pub r#position: self::Point,
        pub r#orientation: self::Quaternion,
    }
    ::roslibrust::impl_ros_message_type!(
        Pose,
        "geometry_msgs/Pose",
        "e45d45a5a1ce597b249e23fb30fc871f",
        r####"# A representation of pose in free space, composed of position and orientation. 
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

float64 x
float64 y
float64 z
float64 w"####,
        "geometry_msgs::msg::dds_::Pose_",
        &[
            0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2, 0x99, 0x69, 0x84, 0xe8, 0x12, 0x05,
            0x4b, 0x68, 0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a, 0x10, 0xb2, 0x3d, 0xaf,
            0x35, 0xcc, 0x90, 0xfa,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Pose2D {
        pub r#x: f64,
        pub r#y: f64,
        pub r#theta: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Pose2D,
        "geometry_msgs/Pose2D",
        "938fa65709584ad8e77d238529be13b8",
        r####"# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta"####,
        "geometry_msgs::msg::dds_::Pose2D_",
        &[
            0xd6, 0x8e, 0xfa, 0x5b, 0x46, 0xe7, 0x0f, 0x7b, 0x16, 0xca, 0x23, 0x08, 0x54, 0x74,
            0xfd, 0xac, 0x5a, 0x44, 0xb6, 0x38, 0x78, 0x3e, 0xc4, 0x2f, 0x66, 0x1d, 0xa6, 0x4d,
            0xa4, 0x72, 0x4c, 0xcc,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PoseArray {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<self::Pose>,
    }
    ::roslibrust::impl_ros_message_type!(
        PoseArray,
        "geometry_msgs/PoseArray",
        "916c28c5764443f268b296bb671b9d97",
        r####"# An array of poses with a header for global reference.

Header header

Pose[] poses
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::PoseArray_",
        &[
            0x3d, 0xfa, 0xa2, 0x97, 0xe8, 0xef, 0x84, 0x2d, 0x04, 0xd9, 0xea, 0x4f, 0x53, 0x78,
            0xa8, 0xe1, 0xd1, 0xff, 0x86, 0x6f, 0x29, 0x8d, 0xe7, 0xec, 0xf4, 0x35, 0x41, 0xf8,
            0x61, 0x5d, 0x50, 0xed,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PoseStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::Pose,
    }
    ::roslibrust::impl_ros_message_type!(
        PoseStamped,
        "geometry_msgs/PoseStamped",
        "d3812c3cbc69362b77dc0b19b345f8f5",
        r####"# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::PoseStamped_",
        &[
            0x13, 0x7f, 0xcc, 0xf0, 0xc6, 0x2a, 0x8f, 0x26, 0xbf, 0x49, 0x39, 0x74, 0x73, 0x9b,
            0x58, 0x8f, 0xb8, 0x0d, 0x4b, 0x5c, 0x8d, 0x3a, 0xf6, 0xfe, 0xda, 0x4d, 0x36, 0x45,
            0x20, 0xec, 0x22, 0x6e,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PoseWithCovariance {
        pub r#pose: self::Pose,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust::codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    ::roslibrust::impl_ros_message_type!(
        PoseWithCovariance,
        "geometry_msgs/PoseWithCovariance",
        "c23e848cf1b7533a8d7c259073a97e6f",
        r####"# This represents a pose in free space with uncertainty.

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w"####,
        "geometry_msgs::msg::dds_::PoseWithCovariance_",
        &[
            0x9a, 0x7c, 0x0f, 0xd2, 0x34, 0xb7, 0xf4, 0x5c, 0x60, 0x98, 0x74, 0x5e, 0xcc, 0xcd,
            0x77, 0x3c, 0xa1, 0x08, 0x56, 0x70, 0xe6, 0x41, 0x07, 0x13, 0x53, 0x97, 0xae, 0xe3,
            0x1c, 0x02, 0xe1, 0xbb,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PoseWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::PoseWithCovariance,
    }
    ::roslibrust::impl_ros_message_type!(
        PoseWithCovarianceStamped,
        "geometry_msgs/PoseWithCovarianceStamped",
        "953b798c0f514ff060a53a3498ce6246",
        r####"# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose
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

float64 x
float64 y
float64 z
float64 w
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::PoseWithCovarianceStamped_",
        &[
            0xba, 0x09, 0xde, 0x0d, 0xa1, 0x18, 0xc9, 0xd7, 0x17, 0x53, 0x10, 0xf7, 0x06, 0xfd,
            0x1b, 0x93, 0x10, 0x5e, 0xe7, 0xf7, 0x12, 0x88, 0xc7, 0x1c, 0x23, 0xa0, 0xd1, 0xe9,
            0xe7, 0xe4, 0x48, 0x1a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Quaternion {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
        pub r#w: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Quaternion,
        "geometry_msgs/Quaternion",
        "a779879fadf0160734f906b8c19c7004",
        r####"# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w"####,
        "geometry_msgs::msg::dds_::Quaternion_",
        &[
            0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7, 0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59,
            0x0a, 0x2e, 0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff, 0xf3, 0x8f, 0xbc, 0xe3,
            0x6f, 0x45, 0x86, 0x84,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct QuaternionStamped {
        pub r#header: std_msgs::Header,
        pub r#quaternion: self::Quaternion,
    }
    ::roslibrust::impl_ros_message_type!(
        QuaternionStamped,
        "geometry_msgs/QuaternionStamped",
        "e57f1e547e0e1fd13504588ffc8334e2",
        r####"# This represents an orientation with reference coordinate frame and timestamp.

Header header
Quaternion quaternion
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::QuaternionStamped_",
        &[
            0x65, 0x47, 0x55, 0x14, 0xfe, 0x6b, 0xf5, 0x8e, 0x6b, 0x0e, 0xcb, 0x47, 0xde, 0xc0,
            0x6e, 0xef, 0xd4, 0x06, 0x79, 0xf6, 0x02, 0xa8, 0xc4, 0xcc, 0x6f, 0x1e, 0x49, 0xa0,
            0xff, 0x45, 0xc8, 0x5d,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Transform {
        pub r#translation: self::Vector3,
        pub r#rotation: self::Quaternion,
    }
    ::roslibrust::impl_ros_message_type!(
        Transform,
        "geometry_msgs/Transform",
        "ac9eff44abf714214112b05d54a3cf9b",
        r####"# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Transform_",
        &[
            0xbe, 0xb8, 0x3f, 0xbe, 0x69, 0x86, 0x36, 0x35, 0x14, 0x61, 0xf6, 0xf3, 0x5d, 0x1a,
            0xbb, 0x20, 0x01, 0x0c, 0x43, 0xd5, 0x53, 0x74, 0xd8, 0x1b, 0xd0, 0x41, 0xf1, 0xba,
            0x25, 0x81, 0xfd, 0xdc,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TransformStamped {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#transform: self::Transform,
    }
    ::roslibrust::impl_ros_message_type!(
        TransformStamped,
        "geometry_msgs/TransformStamped",
        "b5764a33bfeb3588febc2682852579b0",
        r####"# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://wiki.ros.org/tf">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::TransformStamped_",
        &[
            0x76, 0xc2, 0xd4, 0xb2, 0x4e, 0x42, 0x44, 0xae, 0x82, 0x0d, 0x8c, 0x98, 0x20, 0x90,
            0x50, 0xcf, 0x29, 0x3b, 0xa0, 0x43, 0x15, 0xaa, 0xde, 0xf5, 0x59, 0xb3, 0xe5, 0xd4,
            0xb9, 0xb8, 0x9d, 0x7c,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Twist {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    ::roslibrust::impl_ros_message_type!(
        Twist,
        "geometry_msgs/Twist",
        "9f195f881246fdfa2798d1d3eebca84a",
        r####"# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Twist_",
        &[
            0x9c, 0x45, 0xbf, 0x16, 0xfe, 0x09, 0x83, 0xd8, 0x0e, 0x3c, 0xfe, 0x75, 0x0d, 0x68,
            0x35, 0x84, 0x3d, 0x26, 0x5a, 0x9a, 0x6c, 0x46, 0xbd, 0x2e, 0x60, 0x9f, 0xcd, 0xdd,
            0xe6, 0xfb, 0x8d, 0x2a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TwistStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::Twist,
    }
    ::roslibrust::impl_ros_message_type!(
        TwistStamped,
        "geometry_msgs/TwistStamped",
        "98d34b0043a2093cf9d9345ab6eef12e",
        r####"# A twist with reference coordinate frame and timestamp
Header header
Twist twist
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::TwistStamped_",
        &[
            0x6a, 0xdd, 0x90, 0x5e, 0x39, 0x19, 0x47, 0x17, 0x56, 0xa5, 0xd5, 0x69, 0x0a, 0x0a,
            0x3b, 0x16, 0xe4, 0x69, 0x5c, 0x8c, 0xa4, 0xb7, 0x6b, 0xf7, 0x3d, 0xe7, 0x8a, 0x7c,
            0xa1, 0xd9, 0x73, 0xf4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TwistWithCovariance {
        pub r#twist: self::Twist,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust::codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    ::roslibrust::impl_ros_message_type!(
        TwistWithCovariance,
        "geometry_msgs/TwistWithCovariance",
        "1fe8a28e6890a4cc3ae4c3ca5c7d82e6",
        r####"# This expresses velocity in free space with uncertainty.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::TwistWithCovariance_",
        &[
            0x49, 0xf5, 0x74, 0xf0, 0x33, 0xf0, 0x95, 0xd8, 0xb6, 0xcd, 0x1b, 0xea, 0xca, 0x5c,
            0xa7, 0x92, 0x5e, 0x29, 0x6e, 0x84, 0xaf, 0x17, 0x16, 0xd1, 0x6c, 0x89, 0xd3, 0x8b,
            0x05, 0x9c, 0x8c, 0x18,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TwistWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::TwistWithCovariance,
    }
    ::roslibrust::impl_ros_message_type!(
        TwistWithCovarianceStamped,
        "geometry_msgs/TwistWithCovarianceStamped",
        "8927a1a12fb2607ceea095b2dc440a96",
        r####"# This represents an estimated twist with reference coordinate frame and timestamp.
Header header
TwistWithCovariance twist
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::TwistWithCovarianceStamped_",
        &[
            0x5d, 0x9c, 0xa5, 0x12, 0xb1, 0xd6, 0x4f, 0x68, 0x82, 0xc0, 0xe1, 0xed, 0x3f, 0x3e,
            0x58, 0x85, 0x13, 0x56, 0x94, 0x27, 0xa0, 0x9f, 0x0d, 0x54, 0x95, 0xdb, 0xf1, 0xb0,
            0xb0, 0x44, 0x3f, 0x8a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Vector3 {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Vector3,
        "geometry_msgs/Vector3",
        "4a842b65f413084dc2b10fb484ea7f17",
        r####"# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Vector3_",
        &[
            0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19, 0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1,
            0x4a, 0xec, 0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a, 0x2a, 0x1f, 0x37, 0xf7,
            0xdb, 0xb0, 0x76, 0x5d,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Vector3Stamped {
        pub r#header: std_msgs::Header,
        pub r#vector: self::Vector3,
    }
    ::roslibrust::impl_ros_message_type!(
        Vector3Stamped,
        "geometry_msgs/Vector3Stamped",
        "7b324c7325e683bf02a9b14b01090ec7",
        r####"# This represents a Vector3 with reference coordinate frame and timestamp
Header header
Vector3 vector
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::Vector3Stamped_",
        &[
            0xcc, 0x30, 0xa6, 0x24, 0x13, 0xb4, 0x62, 0x5e, 0x67, 0xec, 0xe5, 0x7a, 0x3f, 0x06,
            0x5e, 0x1b, 0x5d, 0x51, 0xd8, 0xcf, 0x6a, 0xa4, 0x08, 0x70, 0x61, 0x99, 0xc4, 0xe5,
            0xe8, 0x26, 0x11, 0xfa,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Wrench {
        pub r#force: self::Vector3,
        pub r#torque: self::Vector3,
    }
    ::roslibrust::impl_ros_message_type!(
        Wrench,
        "geometry_msgs/Wrench",
        "4f539cf138b23283b520fd271b567936",
        r####"# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "geometry_msgs::msg::dds_::Wrench_",
        &[
            0x01, 0x8e, 0x85, 0x19, 0xd5, 0x7c, 0x16, 0xad, 0xbe, 0x97, 0xc9, 0xfe, 0x14, 0x60,
            0xef, 0x21, 0xfe, 0xc7, 0xe3, 0x1b, 0xc5, 0x41, 0xde, 0x3d, 0x65, 0x3a, 0x35, 0x89,
            0x56, 0x77, 0xce, 0x52,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct WrenchStamped {
        pub r#header: std_msgs::Header,
        pub r#wrench: self::Wrench,
    }
    ::roslibrust::impl_ros_message_type!(
        WrenchStamped,
        "geometry_msgs/WrenchStamped",
        "d78d3cb249ce23087ade7e7d0c40cfa7",
        r####"# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "geometry_msgs::msg::dds_::WrenchStamped_",
        &[
            0x91, 0x34, 0xc4, 0x7a, 0x80, 0xba, 0x54, 0xcd, 0x3d, 0x73, 0x1c, 0xde, 0x2b, 0x39,
            0x5c, 0x0b, 0xf2, 0x71, 0x09, 0x07, 0x68, 0x47, 0x46, 0xb3, 0x97, 0xc2, 0x56, 0xea,
            0x58, 0x63, 0xde, 0x17,
        ],
    );
}
#[allow(unused_imports)]
pub mod nav_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapAction {
        pub r#action_goal: self::GetMapActionGoal,
        pub r#action_result: self::GetMapActionResult,
        pub r#action_feedback: self::GetMapActionFeedback,
    }
    ::roslibrust::impl_ros_message_type!(
        GetMapAction,
        "nav_msgs/GetMapAction",
        "e611ad23fbf237c031b7536416dc7cd7",
        r####"GetMapActionGoal action_goal
GetMapActionResult action_result
GetMapActionFeedback action_feedback
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/GetMapActionFeedback
Header header
actionlib_msgs/GoalStatus status
GetMapFeedback feedback
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: nav_msgs/GetMapFeedback
# no feedback
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: nav_msgs/GetMapActionGoal
Header header
actionlib_msgs/GoalID goal_id
GetMapGoal goal
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: nav_msgs/GetMapGoal
# Get the map as a nav_msgs/OccupancyGrid
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: nav_msgs/GetMapActionResult
Header header
actionlib_msgs/GoalStatus status
GetMapResult result
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/GetMapResult
nav_msgs/OccupancyGrid map
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: nav_msgs/GetMapFeedback
# no feedback
================================================================================
MSG: nav_msgs/GetMapGoal
# Get the map as a nav_msgs/OccupancyGrid
================================================================================
MSG: nav_msgs/GetMapResult
nav_msgs/OccupancyGrid map
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetMapAction_",
        &[
            0x0e, 0x14, 0xd4, 0x81, 0xcb, 0xfe, 0x0c, 0x00, 0x86, 0x56, 0xe4, 0xf7, 0x38, 0xcf,
            0x87, 0xfb, 0x81, 0xd7, 0x6d, 0x74, 0xaf, 0xc9, 0xef, 0x6a, 0x1f, 0xf4, 0xfa, 0xf1,
            0x9b, 0xad, 0xc4, 0xa6,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapActionFeedback {
        pub r#header: std_msgs::Header,
        pub r#status: actionlib_msgs::GoalStatus,
        pub r#feedback: self::GetMapFeedback,
    }
    ::roslibrust::impl_ros_message_type!(
        GetMapActionFeedback,
        "nav_msgs/GetMapActionFeedback",
        "aae20e09065c3809e8a8e87c4c8953fd",
        r####"Header header
actionlib_msgs/GoalStatus status
GetMapFeedback feedback
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: nav_msgs/GetMapFeedback
# no feedback
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetMapActionFeedback_",
        &[
            0x6f, 0xaa, 0x93, 0xa6, 0xc1, 0x3b, 0xef, 0x0d, 0xb0, 0xe4, 0x2b, 0x0c, 0x80, 0xa1,
            0xa3, 0x2a, 0x4f, 0xd2, 0x21, 0xa2, 0x7a, 0x01, 0x14, 0x93, 0x25, 0xcc, 0x45, 0xc0,
            0xaa, 0x9a, 0x6e, 0x8e,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapActionGoal {
        pub r#header: std_msgs::Header,
        pub r#goal_id: actionlib_msgs::GoalID,
        pub r#goal: self::GetMapGoal,
    }
    ::roslibrust::impl_ros_message_type!(
        GetMapActionGoal,
        "nav_msgs/GetMapActionGoal",
        "4b30be6cd12b9e72826df56b481f40e0",
        r####"Header header
actionlib_msgs/GoalID goal_id
GetMapGoal goal
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: nav_msgs/GetMapGoal
# Get the map as a nav_msgs/OccupancyGrid
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetMapActionGoal_",
        &[
            0x14, 0xf7, 0xef, 0x58, 0xe7, 0x76, 0xa1, 0x35, 0xbd, 0x26, 0xad, 0x33, 0x9a, 0xc7,
            0x21, 0xbb, 0x35, 0xd3, 0xa1, 0x6d, 0xa2, 0x28, 0x39, 0xd7, 0xa9, 0x08, 0x78, 0x75,
            0x9b, 0x6d, 0xfe, 0xbb,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapActionResult {
        pub r#header: std_msgs::Header,
        pub r#status: actionlib_msgs::GoalStatus,
        pub r#result: self::GetMapResult,
    }
    ::roslibrust::impl_ros_message_type!(
        GetMapActionResult,
        "nav_msgs/GetMapActionResult",
        "ac66e5b9a79bb4bbd33dab245236c892",
        r####"Header header
actionlib_msgs/GoalStatus status
GetMapResult result
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/GetMapResult
nav_msgs/OccupancyGrid map
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetMapActionResult_",
        &[
            0x55, 0x62, 0x22, 0xe4, 0xcc, 0xaf, 0x96, 0x29, 0xb7, 0x6b, 0xb0, 0x99, 0xd3, 0x9f,
            0xb1, 0x33, 0x21, 0xd6, 0x28, 0x55, 0x7c, 0x10, 0xd1, 0xbe, 0xdd, 0x6f, 0x6e, 0xd9,
            0x82, 0xd1, 0x85, 0x51,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapFeedback {}
    ::roslibrust::impl_ros_message_type!(
        GetMapFeedback,
        "nav_msgs/GetMapFeedback",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####"# no feedback"####,
        "nav_msgs::msg::dds_::GetMapFeedback_",
        &[
            0x25, 0xf3, 0x83, 0x82, 0xb9, 0x3e, 0xce, 0x57, 0x49, 0xad, 0x0a, 0x5a, 0x85, 0x41,
            0x44, 0xe1, 0xe7, 0xd1, 0x37, 0x00, 0x9e, 0x33, 0xa4, 0x99, 0x61, 0x6c, 0xc6, 0xa8,
            0x59, 0x5d, 0xc3, 0xb7,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapGoal {}
    ::roslibrust::impl_ros_message_type!(
        GetMapGoal,
        "nav_msgs/GetMapGoal",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####"# Get the map as a nav_msgs/OccupancyGrid"####,
        "nav_msgs::msg::dds_::GetMapGoal_",
        &[
            0xf1, 0xdb, 0x16, 0x66, 0x0a, 0x77, 0xe9, 0x15, 0xc3, 0x2e, 0x29, 0x03, 0xe3, 0xb3,
            0xc5, 0x47, 0xbe, 0x09, 0xa8, 0xf6, 0xea, 0x90, 0x55, 0x40, 0x9b, 0x24, 0xc7, 0x16,
            0xb5, 0x4a, 0xed, 0x94,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapResult {
        pub r#map: self::OccupancyGrid,
    }
    ::roslibrust::impl_ros_message_type!(
        GetMapResult,
        "nav_msgs/GetMapResult",
        "6cdd0a18e0aff5b0a3ca2326a89b54ff",
        r####"nav_msgs/OccupancyGrid map
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetMapResult_",
        &[
            0x2b, 0x52, 0x81, 0xde, 0x67, 0xd1, 0x21, 0x65, 0xeb, 0x45, 0xf1, 0x97, 0x90, 0x7b,
            0x88, 0xee, 0x68, 0x5b, 0xb8, 0x73, 0x7b, 0xec, 0x60, 0x88, 0x39, 0x53, 0xe1, 0x36,
            0x23, 0x87, 0xe8, 0x85,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GridCells {
        pub r#header: std_msgs::Header,
        pub r#cell_width: f32,
        pub r#cell_height: f32,
        pub r#cells: ::std::vec::Vec<geometry_msgs::Point>,
    }
    ::roslibrust::impl_ros_message_type!(
        GridCells,
        "nav_msgs/GridCells",
        "b9e4f5df6d28e272ebde00a3994830f5",
        r####"#an array of cells in a 2D grid
Header header
float32 cell_width
float32 cell_height
geometry_msgs/Point[] cells
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GridCells_",
        &[
            0x8d, 0x80, 0x30, 0x51, 0x09, 0xf1, 0x92, 0x8a, 0x41, 0x79, 0x46, 0xfb, 0x42, 0x52,
            0x82, 0x21, 0xdc, 0x67, 0x7e, 0x33, 0xb2, 0x7a, 0xd3, 0x09, 0x1a, 0x0f, 0x58, 0x20,
            0xa4, 0x4e, 0x28, 0x25,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MapMetaData {
        pub r#map_load_time: ::roslibrust::codegen::integral_types::Time,
        pub r#resolution: f32,
        pub r#width: u32,
        pub r#height: u32,
        pub r#origin: geometry_msgs::Pose,
    }
    ::roslibrust::impl_ros_message_type!(
        MapMetaData,
        "nav_msgs/MapMetaData",
        "10cfc8a2818024d3248802c00c95f11b",
        r####"# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w"####,
        "nav_msgs::msg::dds_::MapMetaData_",
        &[
            0x7e, 0xbe, 0xbf, 0x8e, 0x28, 0x8c, 0xff, 0xb2, 0xaf, 0x1c, 0x7f, 0x71, 0xd4, 0x7d,
            0xbc, 0x77, 0x3c, 0x63, 0x4a, 0x3e, 0xd7, 0x92, 0x05, 0xe5, 0x96, 0x23, 0x31, 0x5d,
            0x6b, 0x8e, 0x9e, 0xcb,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct OccupancyGrid {
        pub r#header: std_msgs::Header,
        pub r#info: self::MapMetaData,
        pub r#data: ::std::vec::Vec<i8>,
    }
    ::roslibrust::impl_ros_message_type!(
        OccupancyGrid,
        "nav_msgs/OccupancyGrid",
        "3381f2d731d4076ec5c71b0759edbe4e",
        r####"# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::OccupancyGrid_",
        &[
            0x95, 0x5a, 0x8b, 0x57, 0x81, 0x52, 0x99, 0x07, 0x9f, 0xba, 0x28, 0xa9, 0xbf, 0x9e,
            0x85, 0xab, 0xec, 0x20, 0xd0, 0x6e, 0xb6, 0x58, 0x3b, 0x71, 0x92, 0x3c, 0x6a, 0xc9,
            0x77, 0x4a, 0xf1, 0x1f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Odometry {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#pose: geometry_msgs::PoseWithCovariance,
        pub r#twist: geometry_msgs::TwistWithCovariance,
    }
    ::roslibrust::impl_ros_message_type!(
        Odometry,
        "nav_msgs/Odometry",
        "cd5e73d190d741a2f92e81eda573aca7",
        r####"# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
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

float64 x
float64 y
float64 z
float64 w
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::Odometry_",
        &[
            0x0c, 0xb2, 0x5d, 0x40, 0x97, 0xcf, 0xcd, 0x0b, 0xc1, 0x69, 0x6d, 0xb6, 0x29, 0x47,
            0x79, 0x4d, 0x1c, 0xa0, 0x46, 0x87, 0x9a, 0x75, 0x51, 0x36, 0x87, 0xea, 0x91, 0x88,
            0x2d, 0x7d, 0x4b, 0x78,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Path {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<geometry_msgs::PoseStamped>,
    }
    ::roslibrust::impl_ros_message_type!(
        Path,
        "nav_msgs/Path",
        "6227e2b7e9cce15051f669a5e197bbf7",
        r####"#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::Path_",
        &[
            0xc0, 0x43, 0x08, 0x16, 0x0c, 0x0a, 0x4a, 0x4d, 0x42, 0x0b, 0x67, 0x2e, 0xa4, 0x2d,
            0xe4, 0x07, 0x00, 0xc0, 0x08, 0xcd, 0x19, 0x43, 0x74, 0x79, 0xfd, 0xfc, 0xfd, 0xb6,
            0xcb, 0xc4, 0x99, 0xab,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapRequest {}
    ::roslibrust::impl_ros_message_type!(
        GetMapRequest,
        "nav_msgs/GetMapRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####"# Get the map as a nav_msgs/OccupancyGrid"####,
        "nav_msgs::msg::dds_::GetMapRequest_",
        &[
            0xf1, 0xfc, 0xfb, 0x16, 0x4b, 0xf2, 0xca, 0x24, 0xfb, 0xc4, 0xfb, 0x51, 0x08, 0xad,
            0xff, 0x60, 0x7f, 0x40, 0x12, 0xb9, 0x29, 0xa1, 0xb1, 0xf0, 0xe3, 0xcc, 0x77, 0xec,
            0xdf, 0x8e, 0x02, 0xde,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetMapResponse {
        pub r#map: self::OccupancyGrid,
    }
    ::roslibrust::impl_ros_message_type!(
        GetMapResponse,
        "nav_msgs/GetMapResponse",
        "6cdd0a18e0aff5b0a3ca2326a89b54ff",
        r####"nav_msgs/OccupancyGrid map
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetMapResponse_",
        &[
            0x48, 0xc1, 0x05, 0xf2, 0xff, 0x69, 0x21, 0xb3, 0x21, 0x5d, 0xc7, 0xeb, 0xc6, 0xed,
            0xa3, 0x8b, 0xa2, 0xef, 0x1c, 0x23, 0xfe, 0xcc, 0x64, 0x52, 0x9d, 0xd2, 0x17, 0xb2,
            0x0f, 0xfd, 0xfc, 0xed,
        ],
    );
    #[allow(dead_code)]
    pub struct GetMap {}
    impl ::roslibrust::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetMap";
        const MD5SUM: &'static str = "6cdd0a18e0aff5b0a3ca2326a89b54ff";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x80, 0xa7, 0xb6, 0x65, 0x74, 0x45, 0x1e, 0x8f, 0xf7, 0x42, 0x72, 0x0e, 0x25, 0xc7,
            0xda, 0x4b, 0xf0, 0xcb, 0xbf, 0xc9, 0x35, 0x4b, 0x35, 0x3e, 0x2c, 0xf2, 0x6c, 0x5b,
            0x3b, 0x6a, 0xc4, 0x30,
        ];
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::srv::dds_::GetMap_";
        type Request = GetMapRequest;
        type Response = GetMapResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetPlanRequest {
        pub r#start: geometry_msgs::PoseStamped,
        pub r#goal: geometry_msgs::PoseStamped,
        pub r#tolerance: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        GetPlanRequest,
        "nav_msgs/GetPlanRequest",
        "e25a43e0752bcca599a8c2eef8282df8",
        r####"# Get a plan from the current position to the goal Pose 

# The start pose for the plan
geometry_msgs/PoseStamped start

# The final pose of the goal position
geometry_msgs/PoseStamped goal

# If the goal is obstructed, how many meters the planner can 
# relax the constraint in x and y before failing. 
float32 tolerance
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetPlanRequest_",
        &[
            0x0b, 0x56, 0xf7, 0x7f, 0x00, 0x1c, 0xad, 0x5c, 0xcb, 0x21, 0x55, 0xc4, 0xdd, 0x8c,
            0xe8, 0x76, 0x95, 0x67, 0x2c, 0x06, 0x92, 0xb6, 0x42, 0xc9, 0xda, 0xe5, 0x1c, 0x08,
            0x40, 0x1f, 0xdf, 0x1e,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetPlanResponse {
        pub r#plan: self::Path,
    }
    ::roslibrust::impl_ros_message_type!(
        GetPlanResponse,
        "nav_msgs/GetPlanResponse",
        "0002bc113c0259d71f6cf8cbc9430e18",
        r####"nav_msgs/Path plan
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::GetPlanResponse_",
        &[
            0xfb, 0xb2, 0x44, 0x07, 0x1e, 0xd1, 0x30, 0x7f, 0xa9, 0x20, 0x3a, 0xd9, 0xc6, 0x58,
            0x18, 0x29, 0x8b, 0x87, 0xfd, 0x3b, 0xba, 0x90, 0xd7, 0xbb, 0xb2, 0xdd, 0x8d, 0xb6,
            0x9c, 0xb9, 0xa3, 0x73,
        ],
    );
    #[allow(dead_code)]
    pub struct GetPlan {}
    impl ::roslibrust::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetPlan";
        const MD5SUM: &'static str = "421c8ea4d21c6c9db7054b4bbdf1e024";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xf3, 0x8e, 0xee, 0x91, 0x27, 0x3f, 0xbd, 0x2b, 0x63, 0x09, 0x7b, 0x60, 0x4d, 0x18,
            0x64, 0xc5, 0x27, 0x5d, 0x2c, 0xe6, 0x7c, 0x41, 0x37, 0xcf, 0x2c, 0x00, 0x90, 0x11,
            0x0b, 0x14, 0x4a, 0x8a,
        ];
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::srv::dds_::GetPlan_";
        type Request = GetPlanRequest;
        type Response = GetPlanResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct LoadMapRequest {
        pub r#map_url: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        LoadMapRequest,
        "nav_msgs/LoadMapRequest",
        "3813ba1ae85fbcd4dc88c90f1426b90b",
        r####"# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url"####,
        "nav_msgs::msg::dds_::LoadMapRequest_",
        &[
            0xb2, 0x99, 0x1c, 0x5a, 0xfe, 0x57, 0xc1, 0xbc, 0xfd, 0x58, 0xad, 0x47, 0xbd, 0x32,
            0x7d, 0x40, 0xa9, 0xdd, 0xdc, 0xcf, 0xef, 0x30, 0x4d, 0x05, 0xad, 0x06, 0xe2, 0x6c,
            0xf2, 0x6d, 0x19, 0xbf,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct LoadMapResponse {
        pub r#map: self::OccupancyGrid,
        pub r#result: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        LoadMapResponse,
        "nav_msgs/LoadMapResponse",
        "079b9c828e9f7c1918bf86932fd7267e",
        r####"# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255

# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::LoadMapResponse_",
        &[
            0x6d, 0x76, 0xe6, 0x52, 0x06, 0xc3, 0x5b, 0xff, 0x56, 0xd1, 0x35, 0xbc, 0xcb, 0x5a,
            0x9f, 0xd5, 0x2f, 0x39, 0x89, 0x4d, 0x49, 0x3b, 0x3d, 0x57, 0x07, 0xf1, 0x8b, 0x1c,
            0x37, 0x92, 0x37, 0x79,
        ],
    );
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
        const MD5SUM: &'static str = "22e647fdfbe3b23c8c9f419908afaebd";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x4a, 0xfe, 0xe0, 0xb1, 0x53, 0x37, 0xa7, 0x43, 0x0a, 0x04, 0x21, 0xff, 0x96, 0x54,
            0xaf, 0xe1, 0x55, 0xe2, 0x0b, 0xd4, 0xa1, 0xa3, 0xb4, 0x2c, 0x61, 0x16, 0x25, 0x9e,
            0x21, 0x75, 0xb1, 0x12,
        ];
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::srv::dds_::LoadMap_";
        type Request = LoadMapRequest;
        type Response = LoadMapResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetMapRequest {
        pub r#map: self::OccupancyGrid,
        pub r#initial_pose: geometry_msgs::PoseWithCovarianceStamped,
    }
    ::roslibrust::impl_ros_message_type!(
        SetMapRequest,
        "nav_msgs/SetMapRequest",
        "91149a20d7be299b87c340df8cc94fd4",
        r####"# Set a new map together with an initial pose
nav_msgs/OccupancyGrid map
geometry_msgs/PoseWithCovarianceStamped initial_pose
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

float64 x
float64 y
float64 z
float64 w
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose
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

float64 x
float64 y
float64 z
float64 w
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "nav_msgs::msg::dds_::SetMapRequest_",
        &[
            0x74, 0x4e, 0x7b, 0xda, 0xba, 0x98, 0xc8, 0xed, 0xba, 0x72, 0xec, 0x45, 0xc9, 0xb7,
            0x0c, 0xa3, 0x0d, 0x71, 0xca, 0xf3, 0x9e, 0x57, 0x41, 0x9e, 0x6d, 0xe8, 0x8b, 0xea,
            0x5f, 0x16, 0x63, 0xf3,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetMapResponse {
        pub r#success: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        SetMapResponse,
        "nav_msgs/SetMapResponse",
        "358e233cde0c8a8bcfea4ce193f8fc15",
        r####"bool success"####,
        "nav_msgs::msg::dds_::SetMapResponse_",
        &[
            0x9f, 0x71, 0xef, 0xa1, 0x56, 0x6a, 0xb7, 0x03, 0x9b, 0x46, 0x16, 0xb1, 0x7f, 0xc9,
            0xc4, 0x04, 0xef, 0x05, 0xdd, 0x55, 0x8e, 0xea, 0x12, 0x4c, 0x04, 0x2c, 0xe0, 0xcd,
            0xc3, 0x68, 0xaf, 0x26,
        ],
    );
    #[allow(dead_code)]
    pub struct SetMap {}
    impl ::roslibrust::RosServiceType for SetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/SetMap";
        const MD5SUM: &'static str = "c36922319011e63ed7784112ad4fdd32";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xfc, 0xbb, 0xf5, 0xa7, 0x92, 0x7f, 0x10, 0xdb, 0xa5, 0x79, 0xa5, 0x98, 0x48, 0x00,
            0xff, 0x9e, 0x67, 0xc4, 0x92, 0x14, 0x19, 0xbd, 0x19, 0x50, 0x17, 0x4d, 0x64, 0x05,
            0x5a, 0xbf, 0x88, 0xff,
        ];
        const ROS2_TYPE_NAME: &'static str = "nav_msgs::srv::dds_::SetMap_";
        type Request = SetMapRequest;
        type Response = SetMapResponse;
    }
}
#[allow(unused_imports)]
pub mod rosapi {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TypeDef {
        pub r#type: ::std::string::String,
        pub r#fieldnames: ::std::vec::Vec<::std::string::String>,
        pub r#fieldtypes: ::std::vec::Vec<::std::string::String>,
        pub r#fieldarraylen: ::std::vec::Vec<i32>,
        pub r#examples: ::std::vec::Vec<::std::string::String>,
        pub r#constnames: ::std::vec::Vec<::std::string::String>,
        pub r#constvalues: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        TypeDef,
        "rosapi/TypeDef",
        "80597571d79bbeef6c9c4d98f30116a0",
        r####"string type
string[] fieldnames
string[] fieldtypes
int32[] fieldarraylen
string[] examples
string[] constnames
string[] constvalues"####,
        "rosapi::msg::dds_::TypeDef_",
        &[
            0xee, 0xe8, 0xba, 0x02, 0x43, 0xa5, 0x07, 0xf4, 0x62, 0x06, 0x72, 0x8c, 0xef, 0x96,
            0x5f, 0xaa, 0x9c, 0x96, 0x7a, 0x7e, 0xb1, 0x15, 0xbf, 0xe7, 0xaf, 0x82, 0x00, 0x88,
            0x28, 0xe6, 0x8a, 0x99,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct DeleteParamRequest {
        pub r#name: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        DeleteParamRequest,
        "rosapi/DeleteParamRequest",
        "c1f3d28f1b044c871e6eff2e9fc3c667",
        r####"string name"####,
        "rosapi::msg::dds_::DeleteParamRequest_",
        &[
            0xac, 0xee, 0x1d, 0x0c, 0x10, 0x7e, 0x16, 0x26, 0x75, 0x25, 0x27, 0x70, 0xe1, 0xa5,
            0xf4, 0x48, 0x0f, 0x28, 0x67, 0xc5, 0xa4, 0x78, 0x84, 0xda, 0x04, 0x7c, 0x93, 0x99,
            0x06, 0x7b, 0xc9, 0x29,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct DeleteParamResponse {}
    ::roslibrust::impl_ros_message_type!(
        DeleteParamResponse,
        "rosapi/DeleteParamResponse",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::DeleteParamResponse_",
        &[
            0xa8, 0x30, 0x33, 0x20, 0x21, 0x77, 0x08, 0xbb, 0xe3, 0xa4, 0x1a, 0x9b, 0x64, 0x6d,
            0x99, 0xa8, 0x94, 0x32, 0x70, 0x6e, 0x57, 0xac, 0x77, 0xb0, 0xe7, 0x60, 0x87, 0x78,
            0xbd, 0xe8, 0x6e, 0x19,
        ],
    );
    #[allow(dead_code)]
    pub struct DeleteParam {}
    impl ::roslibrust::RosServiceType for DeleteParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/DeleteParam";
        const MD5SUM: &'static str = "c1f3d28f1b044c871e6eff2e9fc3c667";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xce, 0x10, 0x5e, 0xdc, 0x0a, 0x53, 0xb6, 0x5a, 0x55, 0xbb, 0xe4, 0xc6, 0x8d, 0xcd,
            0x76, 0x37, 0x26, 0x30, 0x51, 0xd9, 0x36, 0x55, 0x0f, 0x80, 0x5d, 0xf2, 0x5a, 0x25,
            0x0b, 0x36, 0x14, 0x99,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::DeleteParam_";
        type Request = DeleteParamRequest;
        type Response = DeleteParamResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetActionServersRequest {}
    ::roslibrust::impl_ros_message_type!(
        GetActionServersRequest,
        "rosapi/GetActionServersRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::GetActionServersRequest_",
        &[
            0xf3, 0x54, 0xdd, 0x4b, 0x33, 0x58, 0xe0, 0x25, 0xd8, 0x11, 0x2e, 0x3a, 0x93, 0x02,
            0xbf, 0xcb, 0xf6, 0xfd, 0x05, 0x4d, 0x3a, 0xdd, 0x59, 0xfd, 0x75, 0x62, 0x2a, 0xa7,
            0xee, 0x1b, 0x8d, 0x1f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetActionServersResponse {
        pub r#action_servers: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        GetActionServersResponse,
        "rosapi/GetActionServersResponse",
        "46807ba271844ac5ba4730a47556b236",
        r####"string[] action_servers"####,
        "rosapi::msg::dds_::GetActionServersResponse_",
        &[
            0x28, 0x8a, 0x51, 0xe5, 0xf0, 0x42, 0x50, 0x71, 0x19, 0xd9, 0x13, 0x6d, 0xdb, 0x45,
            0xc9, 0x80, 0x45, 0xde, 0xce, 0xe6, 0xff, 0x6d, 0x24, 0xdb, 0xcd, 0x74, 0x67, 0x4a,
            0x49, 0x89, 0x56, 0x7a,
        ],
    );
    #[allow(dead_code)]
    pub struct GetActionServers {}
    impl ::roslibrust::RosServiceType for GetActionServers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetActionServers";
        const MD5SUM: &'static str = "46807ba271844ac5ba4730a47556b236";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x8d, 0x8d, 0xb0, 0x60, 0xe1, 0xd6, 0x69, 0xea, 0x3f, 0x52, 0x96, 0x4e, 0x7d, 0x7c,
            0xb0, 0x8d, 0x00, 0xe9, 0x1a, 0x67, 0x6b, 0xc6, 0xf1, 0x57, 0x69, 0xf8, 0x73, 0x4a,
            0x2f, 0x34, 0xd1, 0x06,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::GetActionServers_";
        type Request = GetActionServersRequest;
        type Response = GetActionServersResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetParamRequest {
        pub r#name: ::std::string::String,
        pub r#default: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        GetParamRequest,
        "rosapi/GetParamRequest",
        "1cc3f281ee24ba9406c3e498e4da686f",
        r####"string name
string default"####,
        "rosapi::msg::dds_::GetParamRequest_",
        &[
            0xdf, 0xe9, 0x69, 0x5d, 0x7d, 0xf1, 0xb8, 0xa4, 0xe4, 0x69, 0x08, 0xcf, 0x21, 0x75,
            0x89, 0x5a, 0x30, 0xbc, 0x9e, 0x42, 0x70, 0xd2, 0x10, 0xa7, 0x37, 0x3c, 0x7b, 0x2f,
            0x37, 0x30, 0xb0, 0x35,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetParamResponse {
        pub r#value: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        GetParamResponse,
        "rosapi/GetParamResponse",
        "64e58419496c7248b4ef25731f88b8c3",
        r####"string value"####,
        "rosapi::msg::dds_::GetParamResponse_",
        &[
            0x75, 0xd1, 0xba, 0x88, 0xe3, 0x0c, 0x15, 0x4f, 0xd3, 0x38, 0x72, 0x32, 0xfe, 0xd9,
            0xd1, 0x68, 0x27, 0x21, 0xbd, 0x02, 0xc0, 0xce, 0xb5, 0x6f, 0x23, 0xe8, 0xaf, 0x6b,
            0x8b, 0xed, 0xf5, 0x8a,
        ],
    );
    #[allow(dead_code)]
    pub struct GetParam {}
    impl ::roslibrust::RosServiceType for GetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParam";
        const MD5SUM: &'static str = "e36fd90759dbac1c5159140a7fa8c644";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x47, 0xcf, 0xb3, 0xf5, 0x63, 0x98, 0x39, 0x7d, 0xe6, 0x5f, 0x8a, 0x29, 0x8f, 0x85,
            0x68, 0x30, 0x8e, 0xb5, 0xe0, 0x19, 0xa0, 0xf9, 0x40, 0x47, 0x47, 0xcc, 0x15, 0x7e,
            0x9a, 0xca, 0x3f, 0xe5,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::GetParam_";
        type Request = GetParamRequest;
        type Response = GetParamResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetParamNamesRequest {}
    ::roslibrust::impl_ros_message_type!(
        GetParamNamesRequest,
        "rosapi/GetParamNamesRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::GetParamNamesRequest_",
        &[
            0x94, 0x03, 0xe4, 0xcc, 0xc1, 0xc9, 0xc3, 0xa9, 0xc5, 0xe5, 0xd7, 0x78, 0xb7, 0x75,
            0x6e, 0x57, 0x31, 0x3a, 0xfe, 0xce, 0xc9, 0x60, 0x2c, 0xda, 0x78, 0x4c, 0x99, 0x86,
            0xb1, 0x17, 0x52, 0x71,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetParamNamesResponse {
        pub r#names: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        GetParamNamesResponse,
        "rosapi/GetParamNamesResponse",
        "dc7ae3609524b18034e49294a4ce670e",
        r####"string[] names"####,
        "rosapi::msg::dds_::GetParamNamesResponse_",
        &[
            0xbb, 0xc8, 0xe3, 0xb4, 0x85, 0x81, 0x44, 0xa5, 0xdd, 0xe9, 0xf2, 0x33, 0x4f, 0x6b,
            0xa7, 0x1f, 0x9e, 0xf7, 0xe8, 0x51, 0x7d, 0x71, 0x03, 0xee, 0xdc, 0x48, 0x93, 0x5b,
            0xce, 0x14, 0x34, 0xf4,
        ],
    );
    #[allow(dead_code)]
    pub struct GetParamNames {}
    impl ::roslibrust::RosServiceType for GetParamNames {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParamNames";
        const MD5SUM: &'static str = "dc7ae3609524b18034e49294a4ce670e";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xd3, 0xeb, 0x63, 0x03, 0x07, 0x6c, 0xc1, 0xb9, 0xda, 0x0c, 0x3f, 0xfb, 0x78, 0xbb,
            0x51, 0x0b, 0xe3, 0xbe, 0x0c, 0x1a, 0xc7, 0x17, 0xe0, 0x14, 0x6e, 0x65, 0x1c, 0xe0,
            0x39, 0x48, 0xae, 0x04,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::GetParamNames_";
        type Request = GetParamNamesRequest;
        type Response = GetParamNamesResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetTimeRequest {}
    ::roslibrust::impl_ros_message_type!(
        GetTimeRequest,
        "rosapi/GetTimeRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::GetTimeRequest_",
        &[
            0xbb, 0x93, 0xc9, 0x5c, 0xa2, 0x1b, 0xaa, 0xdc, 0x04, 0x97, 0x42, 0x4b, 0x68, 0x78,
            0x85, 0x7c, 0x7d, 0xde, 0xea, 0x36, 0x50, 0x6b, 0xd7, 0x95, 0x6e, 0x07, 0xb8, 0x25,
            0x91, 0xd5, 0x89, 0x76,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct GetTimeResponse {
        pub r#time: ::roslibrust::codegen::integral_types::Time,
    }
    ::roslibrust::impl_ros_message_type!(
        GetTimeResponse,
        "rosapi/GetTimeResponse",
        "556a4fb76023a469987922359d08a844",
        r####"time time"####,
        "rosapi::msg::dds_::GetTimeResponse_",
        &[
            0x15, 0x35, 0x74, 0x74, 0xa7, 0xc3, 0x08, 0x80, 0xf7, 0x30, 0x50, 0x0d, 0x43, 0xc9,
            0xe2, 0x05, 0x32, 0x83, 0x71, 0x85, 0xa2, 0x6b, 0xb1, 0x3c, 0xa5, 0x3d, 0x42, 0xff,
            0x4b, 0x25, 0x01, 0x1a,
        ],
    );
    #[allow(dead_code)]
    pub struct GetTime {}
    impl ::roslibrust::RosServiceType for GetTime {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetTime";
        const MD5SUM: &'static str = "556a4fb76023a469987922359d08a844";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xcd, 0x9e, 0x73, 0x38, 0x31, 0x16, 0xc6, 0x77, 0xdf, 0x6d, 0xff, 0xca, 0x35, 0xd1,
            0x04, 0xaa, 0x02, 0x6e, 0x7f, 0xc1, 0x22, 0x6d, 0x92, 0x4f, 0xcd, 0x14, 0x35, 0xf4,
            0xa7, 0x34, 0xb9, 0x5a,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::GetTime_";
        type Request = GetTimeRequest;
        type Response = GetTimeResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct HasParamRequest {
        pub r#name: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        HasParamRequest,
        "rosapi/HasParamRequest",
        "c1f3d28f1b044c871e6eff2e9fc3c667",
        r####"string name"####,
        "rosapi::msg::dds_::HasParamRequest_",
        &[
            0x75, 0x85, 0x02, 0xd5, 0x45, 0x57, 0xca, 0x28, 0x2d, 0xb1, 0x13, 0x02, 0xe2, 0xa2,
            0x7d, 0x71, 0xaf, 0x62, 0x0e, 0x6e, 0x44, 0x89, 0xc6, 0x24, 0x6d, 0x86, 0x52, 0xe1,
            0x89, 0xe3, 0xba, 0x62,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct HasParamResponse {
        pub r#exists: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        HasParamResponse,
        "rosapi/HasParamResponse",
        "e8c90de4adc1219c86af9c2874c0c1b5",
        r####"bool exists"####,
        "rosapi::msg::dds_::HasParamResponse_",
        &[
            0x6f, 0xaa, 0x77, 0xc8, 0x32, 0x40, 0x69, 0x1f, 0x9a, 0x86, 0x57, 0x89, 0x6b, 0x1f,
            0x09, 0x62, 0xe2, 0xba, 0xf4, 0xb1, 0x7d, 0xc7, 0x8b, 0x37, 0xc1, 0xbb, 0xc4, 0xe0,
            0x14, 0x09, 0xaa, 0x59,
        ],
    );
    #[allow(dead_code)]
    pub struct HasParam {}
    impl ::roslibrust::RosServiceType for HasParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/HasParam";
        const MD5SUM: &'static str = "ed3df286bd6dff9b961770f577454ea9";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x50, 0x55, 0xf2, 0x5c, 0x75, 0x67, 0x0a, 0x9d, 0x46, 0x7b, 0x70, 0x50, 0x96, 0x94,
            0x22, 0xbe, 0xc5, 0x72, 0xc7, 0xf3, 0x47, 0xa2, 0x42, 0x0e, 0xb0, 0x03, 0x3f, 0x5b,
            0x64, 0x7c, 0xdf, 0xcc,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::HasParam_";
        type Request = HasParamRequest;
        type Response = HasParamResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MessageDetailsRequest {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        MessageDetailsRequest,
        "rosapi/MessageDetailsRequest",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::MessageDetailsRequest_",
        &[
            0x2c, 0x9a, 0xd8, 0xe6, 0xee, 0xb9, 0xcd, 0xd5, 0x53, 0x59, 0x1c, 0xd6, 0x79, 0x13,
            0x0b, 0x20, 0xf4, 0xc0, 0xf8, 0x93, 0xba, 0x43, 0x4e, 0x90, 0xd7, 0x61, 0xdb, 0x09,
            0xc6, 0x4b, 0xe9, 0xa4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MessageDetailsResponse {
        pub r#typedefs: ::std::vec::Vec<self::TypeDef>,
    }
    ::roslibrust::impl_ros_message_type!(
        MessageDetailsResponse,
        "rosapi/MessageDetailsResponse",
        "a6b8995777f214f2ed97a1e4890feb10",
        r####"TypeDef[] typedefs
================================================================================
MSG: rosapi/TypeDef
string type
string[] fieldnames
string[] fieldtypes
int32[] fieldarraylen
string[] examples
string[] constnames
string[] constvalues"####,
        "rosapi::msg::dds_::MessageDetailsResponse_",
        &[
            0x9a, 0x94, 0x26, 0xdd, 0xd2, 0x4e, 0x00, 0xdc, 0xfc, 0xbb, 0x8e, 0x82, 0x92, 0x57,
            0x56, 0x00, 0xac, 0xac, 0x94, 0x91, 0x73, 0x62, 0x4c, 0xfc, 0x50, 0x47, 0x03, 0xa1,
            0xc9, 0x05, 0x9e, 0xc1,
        ],
    );
    #[allow(dead_code)]
    pub struct MessageDetails {}
    impl ::roslibrust::RosServiceType for MessageDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/MessageDetails";
        const MD5SUM: &'static str = "f9c88144f6f6bd888dd99d4e0411905d";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x40, 0x4d, 0xfe, 0x62, 0x26, 0x1b, 0xd0, 0xff, 0x61, 0x65, 0xc0, 0x47, 0xaf, 0x58,
            0xb0, 0xa5, 0x16, 0xc5, 0x75, 0x02, 0xf4, 0xbd, 0xb2, 0x11, 0x9f, 0x79, 0xf1, 0xbb,
            0x5d, 0x74, 0x4b, 0x9c,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::MessageDetails_";
        type Request = MessageDetailsRequest;
        type Response = MessageDetailsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NodeDetailsRequest {
        pub r#node: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        NodeDetailsRequest,
        "rosapi/NodeDetailsRequest",
        "a94c40e70a4b82863e6e52ec16732447",
        r####"string node"####,
        "rosapi::msg::dds_::NodeDetailsRequest_",
        &[
            0xd5, 0x03, 0xc5, 0x92, 0xc9, 0x59, 0x04, 0x87, 0x73, 0xcf, 0xd3, 0x8a, 0x89, 0xb0,
            0xd0, 0x92, 0xa5, 0x82, 0xb5, 0x31, 0xed, 0xb0, 0xea, 0xce, 0x22, 0x2c, 0x21, 0x68,
            0xfc, 0xeb, 0xf2, 0xed,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NodeDetailsResponse {
        pub r#subscribing: ::std::vec::Vec<::std::string::String>,
        pub r#publishing: ::std::vec::Vec<::std::string::String>,
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        NodeDetailsResponse,
        "rosapi/NodeDetailsResponse",
        "3da1cb16c6ec5885ad291735b6244a48",
        r####"string[] subscribing
string[] publishing
string[] services"####,
        "rosapi::msg::dds_::NodeDetailsResponse_",
        &[
            0x88, 0xdc, 0x4c, 0x0b, 0x68, 0x39, 0xf9, 0xb6, 0xbe, 0xfe, 0xd7, 0xe0, 0x1b, 0x16,
            0x1a, 0xe7, 0x2a, 0xae, 0x86, 0x7a, 0x3c, 0xac, 0x03, 0x56, 0x5c, 0xfe, 0x52, 0xe8,
            0x0e, 0xad, 0x6d, 0xc8,
        ],
    );
    #[allow(dead_code)]
    pub struct NodeDetails {}
    impl ::roslibrust::RosServiceType for NodeDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/NodeDetails";
        const MD5SUM: &'static str = "e1d0ced5ab8d5edb5fc09c98eb1d46f6";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x9d, 0x73, 0xb4, 0xa5, 0x87, 0xbe, 0x23, 0xe1, 0xe1, 0x28, 0x97, 0xf7, 0x85, 0x83,
            0x72, 0x6c, 0xb7, 0xa9, 0x6e, 0x2d, 0x55, 0x71, 0x90, 0xec, 0x73, 0x66, 0xff, 0x54,
            0x67, 0x06, 0xff, 0xa6,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::NodeDetails_";
        type Request = NodeDetailsRequest;
        type Response = NodeDetailsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NodesRequest {}
    ::roslibrust::impl_ros_message_type!(
        NodesRequest,
        "rosapi/NodesRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::NodesRequest_",
        &[
            0x91, 0xd0, 0xc6, 0xdd, 0x8c, 0x00, 0x3e, 0xb6, 0x17, 0x36, 0x40, 0x9c, 0x0d, 0x20,
            0x4c, 0xb1, 0x14, 0x5b, 0x9f, 0x34, 0x28, 0x56, 0x1d, 0x17, 0x7c, 0x53, 0x13, 0xa2,
            0x01, 0x5c, 0xb9, 0xc4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NodesResponse {
        pub r#nodes: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        NodesResponse,
        "rosapi/NodesResponse",
        "3d07bfda1268b4f76b16b7ba8a82665d",
        r####"string[] nodes"####,
        "rosapi::msg::dds_::NodesResponse_",
        &[
            0x17, 0x78, 0xbe, 0xcf, 0xce, 0x07, 0x00, 0x6f, 0x5f, 0x37, 0x7f, 0x5c, 0xcd, 0x5d,
            0xc9, 0x4e, 0x57, 0xdc, 0xd5, 0x2c, 0xcc, 0x4a, 0x83, 0xf0, 0xc9, 0x93, 0xb9, 0x13,
            0x87, 0xeb, 0x3b, 0xf4,
        ],
    );
    #[allow(dead_code)]
    pub struct Nodes {}
    impl ::roslibrust::RosServiceType for Nodes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Nodes";
        const MD5SUM: &'static str = "3d07bfda1268b4f76b16b7ba8a82665d";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x2a, 0x90, 0x11, 0xcb, 0xc2, 0xc1, 0x6e, 0x7e, 0x6d, 0xca, 0xd2, 0xbd, 0x7a, 0x59,
            0xbe, 0xe1, 0x01, 0xf4, 0xff, 0x6d, 0x6a, 0x12, 0x6d, 0x18, 0x05, 0x58, 0xbf, 0x4b,
            0xfc, 0x55, 0x29, 0x2e,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::Nodes_";
        type Request = NodesRequest;
        type Response = NodesResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PublishersRequest {
        pub r#topic: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        PublishersRequest,
        "rosapi/PublishersRequest",
        "d8f94bae31b356b24d0427f80426d0c3",
        r####"string topic"####,
        "rosapi::msg::dds_::PublishersRequest_",
        &[
            0x93, 0x2d, 0x3e, 0x8a, 0x8d, 0xaf, 0x10, 0x09, 0x13, 0xe8, 0x9f, 0xc9, 0x63, 0x91,
            0x64, 0xa9, 0x27, 0xd2, 0x67, 0x04, 0x96, 0xaf, 0x74, 0x9c, 0x26, 0x42, 0x1c, 0xb4,
            0x3b, 0x50, 0xde, 0xbb,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PublishersResponse {
        pub r#publishers: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        PublishersResponse,
        "rosapi/PublishersResponse",
        "167d8030c4ca4018261dff8ae5083dc8",
        r####"string[] publishers"####,
        "rosapi::msg::dds_::PublishersResponse_",
        &[
            0xd8, 0x63, 0x69, 0x6b, 0x2b, 0xb2, 0xec, 0x4c, 0x21, 0x2a, 0x98, 0x96, 0x9c, 0x0c,
            0x76, 0xb4, 0xa6, 0x05, 0x39, 0xba, 0x9b, 0x8c, 0x7c, 0x34, 0x19, 0xe7, 0x89, 0xad,
            0xda, 0xe0, 0x08, 0x6e,
        ],
    );
    #[allow(dead_code)]
    pub struct Publishers {}
    impl ::roslibrust::RosServiceType for Publishers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Publishers";
        const MD5SUM: &'static str = "cb37f09944e7ba1fc08ee38f7a94291d";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xeb, 0x2d, 0x03, 0xb6, 0x64, 0x7c, 0x61, 0xdd, 0x61, 0x49, 0x9a, 0x29, 0x40, 0xfa,
            0x0c, 0x92, 0x75, 0x80, 0x93, 0xa9, 0xa7, 0x5c, 0x5d, 0x2a, 0xe2, 0x5e, 0x72, 0xb4,
            0xf0, 0x5c, 0x26, 0xe3,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::Publishers_";
        type Request = PublishersRequest;
        type Response = PublishersResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SearchParamRequest {
        pub r#name: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        SearchParamRequest,
        "rosapi/SearchParamRequest",
        "c1f3d28f1b044c871e6eff2e9fc3c667",
        r####"string name"####,
        "rosapi::msg::dds_::SearchParamRequest_",
        &[
            0xc6, 0x76, 0xbd, 0x21, 0xf3, 0x44, 0x05, 0xf0, 0xfa, 0x39, 0x6c, 0x4e, 0x9a, 0xcb,
            0x4f, 0x3e, 0xa2, 0x73, 0xde, 0x9a, 0x96, 0x0e, 0xd2, 0x2d, 0x3e, 0xa3, 0xd8, 0xee,
            0x30, 0x10, 0xb0, 0x78,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SearchParamResponse {
        pub r#global_name: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        SearchParamResponse,
        "rosapi/SearchParamResponse",
        "87c264f142c2aeca13349d90aeec0386",
        r####"string global_name"####,
        "rosapi::msg::dds_::SearchParamResponse_",
        &[
            0xcc, 0x6c, 0x51, 0x53, 0x29, 0xe6, 0xd1, 0xb4, 0xbc, 0xeb, 0x9c, 0x42, 0x79, 0xe3,
            0x37, 0x1e, 0xf8, 0x45, 0x19, 0x94, 0x03, 0xbb, 0x25, 0x88, 0xf1, 0x88, 0x49, 0xf9,
            0xfc, 0xc8, 0xf6, 0xa8,
        ],
    );
    #[allow(dead_code)]
    pub struct SearchParam {}
    impl ::roslibrust::RosServiceType for SearchParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SearchParam";
        const MD5SUM: &'static str = "dfadc39f113c1cc6d7759508d8461d5a";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x04, 0x46, 0xb1, 0x17, 0x06, 0xa4, 0x35, 0x9a, 0x06, 0x66, 0x61, 0xf5, 0xba, 0x10,
            0xd2, 0x35, 0xad, 0x36, 0xc4, 0xaf, 0xb5, 0x05, 0x7d, 0x9e, 0x58, 0x8b, 0xb5, 0x66,
            0xd8, 0x54, 0x92, 0xf7,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::SearchParam_";
        type Request = SearchParamRequest;
        type Response = SearchParamResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceHostRequest {
        pub r#service: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceHostRequest,
        "rosapi/ServiceHostRequest",
        "1cbcfa13b08f6d36710b9af8741e6112",
        r####"string service"####,
        "rosapi::msg::dds_::ServiceHostRequest_",
        &[
            0xc2, 0x71, 0xb6, 0x96, 0x27, 0x6d, 0x72, 0xfc, 0xe6, 0x67, 0x71, 0x24, 0xba, 0x9c,
            0x37, 0xc1, 0x21, 0x59, 0xc9, 0x74, 0xe1, 0xd1, 0x94, 0x93, 0x58, 0xf1, 0xdd, 0xe2,
            0xee, 0xbd, 0x77, 0x81,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceHostResponse {
        pub r#host: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceHostResponse,
        "rosapi/ServiceHostResponse",
        "092ff9f63242a37704ce411703ec5eaf",
        r####"string host"####,
        "rosapi::msg::dds_::ServiceHostResponse_",
        &[
            0x05, 0xf3, 0x29, 0x4e, 0x3f, 0xc5, 0x47, 0xdc, 0xc9, 0x8d, 0xf4, 0x3e, 0x55, 0x76,
            0xe7, 0x3c, 0x7d, 0x98, 0xa3, 0x29, 0x02, 0x59, 0x5a, 0x01, 0x78, 0xaa, 0x55, 0xb7,
            0xfa, 0x02, 0xde, 0xe1,
        ],
    );
    #[allow(dead_code)]
    pub struct ServiceHost {}
    impl ::roslibrust::RosServiceType for ServiceHost {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceHost";
        const MD5SUM: &'static str = "a1b60006f8ee69637c856c94dd192f5a";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x3e, 0xd3, 0x0c, 0xf0, 0xb5, 0x0c, 0xf2, 0xe9, 0x7c, 0xfa, 0x18, 0x11, 0xb5, 0xfd,
            0x7d, 0xb4, 0x0b, 0xb6, 0x50, 0xa3, 0xf2, 0x65, 0xc4, 0x5a, 0x38, 0xaa, 0x05, 0xbd,
            0xfa, 0x1d, 0x99, 0x5e,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServiceHost_";
        type Request = ServiceHostRequest;
        type Response = ServiceHostResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceNodeRequest {
        pub r#service: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceNodeRequest,
        "rosapi/ServiceNodeRequest",
        "1cbcfa13b08f6d36710b9af8741e6112",
        r####"string service"####,
        "rosapi::msg::dds_::ServiceNodeRequest_",
        &[
            0xe0, 0x00, 0xe0, 0xab, 0x03, 0x1d, 0x83, 0x9a, 0x1f, 0x12, 0x8b, 0x4f, 0x9e, 0x0b,
            0x27, 0x31, 0x58, 0xbb, 0x60, 0xb0, 0xd7, 0x9d, 0xbf, 0xe5, 0xf5, 0x73, 0xed, 0x50,
            0x28, 0xb6, 0xf8, 0x1b,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceNodeResponse {
        pub r#node: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceNodeResponse,
        "rosapi/ServiceNodeResponse",
        "a94c40e70a4b82863e6e52ec16732447",
        r####"string node"####,
        "rosapi::msg::dds_::ServiceNodeResponse_",
        &[
            0xe0, 0xa4, 0x59, 0x0b, 0xfc, 0x35, 0x6e, 0xfa, 0x18, 0xb1, 0xde, 0xea, 0xa6, 0x78,
            0x6b, 0x59, 0xa0, 0x48, 0x8e, 0xea, 0xe1, 0x4b, 0xd5, 0x0c, 0x33, 0x27, 0xf5, 0x9c,
            0x52, 0x93, 0x33, 0x34,
        ],
    );
    #[allow(dead_code)]
    pub struct ServiceNode {}
    impl ::roslibrust::RosServiceType for ServiceNode {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceNode";
        const MD5SUM: &'static str = "bd2a0a45fd7a73a86c8d6051d5a6db8a";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x36, 0xff, 0x1f, 0xd8, 0x61, 0xda, 0x5b, 0x7c, 0x63, 0xc7, 0xae, 0x53, 0xa3, 0x8d,
            0x2c, 0x92, 0x85, 0xd1, 0x54, 0xe0, 0x80, 0xd2, 0x25, 0x15, 0xdb, 0x72, 0x60, 0x05,
            0xbe, 0x23, 0x2a, 0x60,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServiceNode_";
        type Request = ServiceNodeRequest;
        type Response = ServiceNodeResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceProvidersRequest {
        pub r#service: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceProvidersRequest,
        "rosapi/ServiceProvidersRequest",
        "1cbcfa13b08f6d36710b9af8741e6112",
        r####"string service"####,
        "rosapi::msg::dds_::ServiceProvidersRequest_",
        &[
            0xd4, 0xdf, 0xec, 0x1e, 0x4b, 0xb1, 0x70, 0x12, 0xad, 0x61, 0x79, 0x10, 0x79, 0x95,
            0x21, 0x20, 0x89, 0xa5, 0xf0, 0xbf, 0xc0, 0x70, 0x8d, 0x44, 0x56, 0x17, 0x24, 0xd8,
            0x01, 0x82, 0x4a, 0x20,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceProvidersResponse {
        pub r#providers: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceProvidersResponse,
        "rosapi/ServiceProvidersResponse",
        "945f6849f44f061c178ab393b12c1358",
        r####"string[] providers"####,
        "rosapi::msg::dds_::ServiceProvidersResponse_",
        &[
            0x33, 0xe5, 0xcf, 0x26, 0x27, 0x5c, 0xcb, 0xe4, 0x9c, 0x61, 0xcb, 0xd7, 0x35, 0xeb,
            0xc7, 0x12, 0xe1, 0x99, 0x8b, 0x3b, 0x48, 0x7c, 0xde, 0x53, 0x8f, 0xd6, 0xec, 0x7a,
            0x27, 0x74, 0x9c, 0x81,
        ],
    );
    #[allow(dead_code)]
    pub struct ServiceProviders {}
    impl ::roslibrust::RosServiceType for ServiceProviders {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceProviders";
        const MD5SUM: &'static str = "f30b41d5e347454ae5483ee95eef5cc6";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x7e, 0xef, 0xde, 0x1d, 0xb3, 0x7d, 0x84, 0x14, 0x42, 0xcf, 0x4f, 0xc7, 0x73, 0x9a,
            0x9f, 0x10, 0xf5, 0x33, 0x51, 0xe7, 0x9f, 0xd1, 0x14, 0x17, 0xf2, 0x36, 0xfe, 0x6c,
            0x6a, 0x56, 0xc2, 0x9e,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServiceProviders_";
        type Request = ServiceProvidersRequest;
        type Response = ServiceProvidersResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceRequestDetailsRequest {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceRequestDetailsRequest,
        "rosapi/ServiceRequestDetailsRequest",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::ServiceRequestDetailsRequest_",
        &[
            0xe1, 0x1c, 0x9f, 0xdb, 0x5f, 0x50, 0x7c, 0x49, 0x4f, 0xd0, 0x18, 0x2b, 0x63, 0x0b,
            0x1f, 0x34, 0x1e, 0x62, 0xb0, 0x03, 0xfa, 0xb9, 0x04, 0x0d, 0xfe, 0x70, 0xb0, 0x04,
            0x10, 0x71, 0x0f, 0x18,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceRequestDetailsResponse {
        pub r#typedefs: ::std::vec::Vec<self::TypeDef>,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceRequestDetailsResponse,
        "rosapi/ServiceRequestDetailsResponse",
        "a6b8995777f214f2ed97a1e4890feb10",
        r####"TypeDef[] typedefs
================================================================================
MSG: rosapi/TypeDef
string type
string[] fieldnames
string[] fieldtypes
int32[] fieldarraylen
string[] examples
string[] constnames
string[] constvalues"####,
        "rosapi::msg::dds_::ServiceRequestDetailsResponse_",
        &[
            0x1a, 0xf1, 0x7b, 0xb5, 0x88, 0x45, 0xb3, 0xbb, 0x80, 0xaf, 0xac, 0x04, 0x9f, 0x09,
            0xcd, 0x90, 0x83, 0x7f, 0x1c, 0x58, 0x78, 0x2e, 0xc8, 0xab, 0x7d, 0xb4, 0x3d, 0xde,
            0x6a, 0x8e, 0x08, 0x4a,
        ],
    );
    #[allow(dead_code)]
    pub struct ServiceRequestDetails {}
    impl ::roslibrust::RosServiceType for ServiceRequestDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceRequestDetails";
        const MD5SUM: &'static str = "f9c88144f6f6bd888dd99d4e0411905d";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xf5, 0x67, 0xd5, 0x63, 0x96, 0xff, 0x24, 0x8f, 0x4d, 0xdc, 0x4d, 0x4c, 0xf8, 0x24,
            0xf2, 0x7f, 0x0a, 0xac, 0xe4, 0x08, 0x4b, 0x03, 0x99, 0xa1, 0x4d, 0xb0, 0x1c, 0x07,
            0x3a, 0xd8, 0x68, 0xb5,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServiceRequestDetails_";
        type Request = ServiceRequestDetailsRequest;
        type Response = ServiceRequestDetailsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceResponseDetailsRequest {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceResponseDetailsRequest,
        "rosapi/ServiceResponseDetailsRequest",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::ServiceResponseDetailsRequest_",
        &[
            0xc2, 0xba, 0xc5, 0xcf, 0x0d, 0x29, 0xba, 0x4a, 0x32, 0xfa, 0xcb, 0xc0, 0x1d, 0x34,
            0xc2, 0x3a, 0x8c, 0xfd, 0x80, 0x51, 0xfc, 0xf7, 0x31, 0xfc, 0xfb, 0x22, 0xfb, 0xcb,
            0x0f, 0x3f, 0x58, 0x57,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceResponseDetailsResponse {
        pub r#typedefs: ::std::vec::Vec<self::TypeDef>,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceResponseDetailsResponse,
        "rosapi/ServiceResponseDetailsResponse",
        "a6b8995777f214f2ed97a1e4890feb10",
        r####"TypeDef[] typedefs
================================================================================
MSG: rosapi/TypeDef
string type
string[] fieldnames
string[] fieldtypes
int32[] fieldarraylen
string[] examples
string[] constnames
string[] constvalues"####,
        "rosapi::msg::dds_::ServiceResponseDetailsResponse_",
        &[
            0xe3, 0x3b, 0xfe, 0x97, 0xa8, 0x5b, 0x3f, 0x5d, 0xe3, 0x9c, 0x71, 0x09, 0x22, 0xc3,
            0x1a, 0x0c, 0xa1, 0xb7, 0x73, 0xed, 0x7c, 0xe2, 0x11, 0x17, 0x57, 0xa2, 0x6c, 0xe1,
            0xb9, 0xaf, 0xe9, 0xe2,
        ],
    );
    #[allow(dead_code)]
    pub struct ServiceResponseDetails {}
    impl ::roslibrust::RosServiceType for ServiceResponseDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceResponseDetails";
        const MD5SUM: &'static str = "f9c88144f6f6bd888dd99d4e0411905d";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x7f, 0x72, 0xe3, 0xc0, 0x71, 0xd7, 0x88, 0x08, 0xb1, 0x47, 0x27, 0xe7, 0xc6, 0xae,
            0x38, 0xb5, 0x55, 0x91, 0xf4, 0xe8, 0x17, 0x24, 0x0e, 0x37, 0x31, 0xc8, 0x83, 0xf6,
            0x52, 0x1c, 0xc2, 0xb1,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServiceResponseDetails_";
        type Request = ServiceResponseDetailsRequest;
        type Response = ServiceResponseDetailsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceTypeRequest {
        pub r#service: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceTypeRequest,
        "rosapi/ServiceTypeRequest",
        "1cbcfa13b08f6d36710b9af8741e6112",
        r####"string service"####,
        "rosapi::msg::dds_::ServiceTypeRequest_",
        &[
            0xad, 0x79, 0x0a, 0x2a, 0x4f, 0x9f, 0xad, 0x1d, 0xec, 0x6e, 0xba, 0x4c, 0x28, 0x1d,
            0x48, 0x99, 0x06, 0x81, 0x63, 0x4e, 0x35, 0xfa, 0x1e, 0x55, 0x78, 0xb3, 0xb3, 0x0e,
            0x12, 0x67, 0xf4, 0xcf,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceTypeResponse {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceTypeResponse,
        "rosapi/ServiceTypeResponse",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::ServiceTypeResponse_",
        &[
            0xc7, 0xbe, 0x70, 0xc7, 0xc8, 0x32, 0xb7, 0x21, 0xa3, 0xc9, 0x4f, 0x15, 0xdf, 0x22,
            0x58, 0x08, 0x58, 0xd1, 0xd7, 0x9b, 0x20, 0x42, 0xc9, 0x22, 0x7e, 0x60, 0xe8, 0x00,
            0x76, 0x88, 0xd2, 0xd0,
        ],
    );
    #[allow(dead_code)]
    pub struct ServiceType {}
    impl ::roslibrust::RosServiceType for ServiceType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceType";
        const MD5SUM: &'static str = "0e24a2dcdf70e483afc092a35a1f15f7";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x53, 0x4e, 0x4c, 0x2e, 0x2f, 0x8e, 0x8e, 0xa4, 0x9f, 0x3c, 0x8d, 0xe0, 0xc2, 0x3d,
            0xcf, 0x8c, 0xb9, 0xf3, 0xbe, 0x0d, 0x2f, 0x13, 0xe7, 0xe1, 0x72, 0x3b, 0xb5, 0x5a,
            0x0a, 0x8f, 0xc8, 0x48,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServiceType_";
        type Request = ServiceTypeRequest;
        type Response = ServiceTypeResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServicesRequest {}
    ::roslibrust::impl_ros_message_type!(
        ServicesRequest,
        "rosapi/ServicesRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::ServicesRequest_",
        &[
            0x39, 0xae, 0x90, 0x60, 0x69, 0x2a, 0x1f, 0x58, 0x34, 0x34, 0xf6, 0x24, 0x6a, 0x54,
            0x3d, 0x99, 0x15, 0xe6, 0x06, 0x21, 0x2f, 0xce, 0x5b, 0x61, 0x5d, 0x9a, 0x76, 0x84,
            0xcd, 0x95, 0x77, 0xd8,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServicesResponse {
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        ServicesResponse,
        "rosapi/ServicesResponse",
        "e44a7e7bcb900acadbcc28b132378f0c",
        r####"string[] services"####,
        "rosapi::msg::dds_::ServicesResponse_",
        &[
            0x5b, 0x79, 0xe4, 0xdd, 0xf0, 0x94, 0xeb, 0x78, 0x5b, 0x9d, 0x43, 0x80, 0xef, 0x2c,
            0xb6, 0x83, 0x5e, 0x57, 0xa2, 0x0b, 0x1a, 0xdd, 0x13, 0x32, 0xb8, 0x3b, 0xef, 0x25,
            0xc6, 0x2c, 0x5b, 0xb1,
        ],
    );
    #[allow(dead_code)]
    pub struct Services {}
    impl ::roslibrust::RosServiceType for Services {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Services";
        const MD5SUM: &'static str = "e44a7e7bcb900acadbcc28b132378f0c";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xc7, 0x55, 0x4f, 0x5f, 0x0f, 0xb1, 0x45, 0xbf, 0x87, 0x9e, 0x1c, 0x0f, 0x0d, 0x33,
            0xea, 0x7d, 0xa1, 0xf7, 0x06, 0x84, 0xb4, 0xd0, 0x3b, 0x64, 0x27, 0xd0, 0xe5, 0x47,
            0x43, 0x29, 0x6d, 0x22,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::Services_";
        type Request = ServicesRequest;
        type Response = ServicesResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServicesForTypeRequest {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        ServicesForTypeRequest,
        "rosapi/ServicesForTypeRequest",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::ServicesForTypeRequest_",
        &[
            0xb5, 0xb5, 0x2f, 0xe9, 0xb7, 0xbc, 0x33, 0x78, 0x42, 0xea, 0xf1, 0x72, 0xd0, 0x38,
            0x67, 0xb4, 0x61, 0xf7, 0x36, 0xb4, 0xeb, 0x54, 0x75, 0xfe, 0x92, 0x31, 0x00, 0x3b,
            0xe1, 0x89, 0x43, 0x15,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServicesForTypeResponse {
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        ServicesForTypeResponse,
        "rosapi/ServicesForTypeResponse",
        "e44a7e7bcb900acadbcc28b132378f0c",
        r####"string[] services"####,
        "rosapi::msg::dds_::ServicesForTypeResponse_",
        &[
            0x53, 0x19, 0x98, 0x6b, 0xac, 0x06, 0xef, 0xf8, 0xc9, 0x72, 0x87, 0x47, 0x53, 0x4d,
            0x32, 0x3e, 0x89, 0x17, 0x06, 0xb9, 0xcb, 0x59, 0x97, 0x69, 0xd9, 0x2a, 0xd2, 0x02,
            0x65, 0xf2, 0x05, 0xed,
        ],
    );
    #[allow(dead_code)]
    pub struct ServicesForType {}
    impl ::roslibrust::RosServiceType for ServicesForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServicesForType";
        const MD5SUM: &'static str = "93e9fe8ae5a9136008e260fe510bd2b0";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x4a, 0x6d, 0x08, 0x6b, 0x14, 0x32, 0xf9, 0xb8, 0x94, 0x49, 0xb6, 0x20, 0x2c, 0x4d,
            0x01, 0x74, 0x04, 0xa1, 0xe9, 0x89, 0x5a, 0x07, 0xea, 0x0a, 0x96, 0x02, 0xb1, 0x65,
            0x4c, 0x1d, 0xad, 0x41,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::ServicesForType_";
        type Request = ServicesForTypeRequest;
        type Response = ServicesForTypeResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetParamRequest {
        pub r#name: ::std::string::String,
        pub r#value: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        SetParamRequest,
        "rosapi/SetParamRequest",
        "bc6ccc4a57f61779c8eaae61e9f422e0",
        r####"string name
string value"####,
        "rosapi::msg::dds_::SetParamRequest_",
        &[
            0x33, 0xea, 0x50, 0xbf, 0xef, 0x78, 0x46, 0x64, 0x31, 0xf4, 0x5a, 0x35, 0xdf, 0x79,
            0x82, 0x7c, 0x3b, 0x9d, 0xb9, 0xad, 0x45, 0x11, 0xda, 0x64, 0x7f, 0x8c, 0x4d, 0x4c,
            0x33, 0xfe, 0xb4, 0x57,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetParamResponse {}
    ::roslibrust::impl_ros_message_type!(
        SetParamResponse,
        "rosapi/SetParamResponse",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::SetParamResponse_",
        &[
            0xfc, 0x62, 0xe0, 0x54, 0x83, 0x61, 0xc9, 0xbc, 0xbe, 0x7c, 0xf5, 0x9e, 0x95, 0x96,
            0xcb, 0x31, 0x66, 0x0f, 0x8d, 0xed, 0xcd, 0x89, 0x61, 0xd0, 0x87, 0x86, 0x6f, 0x35,
            0x07, 0x46, 0xbf, 0xb6,
        ],
    );
    #[allow(dead_code)]
    pub struct SetParam {}
    impl ::roslibrust::RosServiceType for SetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SetParam";
        const MD5SUM: &'static str = "bc6ccc4a57f61779c8eaae61e9f422e0";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x17, 0x35, 0x56, 0xd6, 0xd5, 0x34, 0x37, 0x34, 0x30, 0x03, 0xd7, 0x51, 0xf8, 0xe0,
            0x30, 0x82, 0xfa, 0x85, 0x40, 0xee, 0x5d, 0x2d, 0x06, 0x6a, 0x6c, 0x49, 0xe2, 0x4d,
            0x1b, 0x02, 0xec, 0x50,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::SetParam_";
        type Request = SetParamRequest;
        type Response = SetParamResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SubscribersRequest {
        pub r#topic: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        SubscribersRequest,
        "rosapi/SubscribersRequest",
        "d8f94bae31b356b24d0427f80426d0c3",
        r####"string topic"####,
        "rosapi::msg::dds_::SubscribersRequest_",
        &[
            0x37, 0x0e, 0xfb, 0xa1, 0xd1, 0x22, 0xc2, 0xef, 0x0b, 0x8f, 0xc9, 0xcc, 0xd6, 0xb9,
            0x1e, 0x3e, 0xbc, 0xd8, 0x70, 0x46, 0xd1, 0x48, 0xe4, 0xcb, 0x75, 0xe2, 0x03, 0xd1,
            0x9d, 0x2f, 0x79, 0x31,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SubscribersResponse {
        pub r#subscribers: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        SubscribersResponse,
        "rosapi/SubscribersResponse",
        "22418cab5ba9531d8c2b738b4e56153b",
        r####"string[] subscribers"####,
        "rosapi::msg::dds_::SubscribersResponse_",
        &[
            0xba, 0x14, 0x51, 0xa5, 0x0f, 0xaa, 0x01, 0xe6, 0x2d, 0x0a, 0x45, 0xa0, 0x7d, 0xd7,
            0x3c, 0x75, 0xce, 0xb4, 0xd8, 0x83, 0xdf, 0xce, 0x02, 0xfe, 0x14, 0x35, 0xc4, 0xd0,
            0x96, 0x56, 0xe2, 0x9b,
        ],
    );
    #[allow(dead_code)]
    pub struct Subscribers {}
    impl ::roslibrust::RosServiceType for Subscribers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Subscribers";
        const MD5SUM: &'static str = "cb387b68f5b29bc1456398ee8476b973";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x31, 0x2f, 0x4f, 0x3d, 0xc7, 0xde, 0x6d, 0x63, 0x59, 0x5a, 0x3b, 0xc8, 0xf8, 0xe4,
            0x01, 0x2e, 0x4b, 0x06, 0x8a, 0xbe, 0xd1, 0xcc, 0x0d, 0x9b, 0xdb, 0x70, 0x52, 0xc0,
            0x57, 0x07, 0x62, 0x32,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::Subscribers_";
        type Request = SubscribersRequest;
        type Response = SubscribersResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicTypeRequest {
        pub r#topic: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicTypeRequest,
        "rosapi/TopicTypeRequest",
        "d8f94bae31b356b24d0427f80426d0c3",
        r####"string topic"####,
        "rosapi::msg::dds_::TopicTypeRequest_",
        &[
            0x7f, 0x19, 0x96, 0xbf, 0xe8, 0x2e, 0x8e, 0x14, 0x17, 0xaf, 0xa7, 0x22, 0x3c, 0x42,
            0x6b, 0xfa, 0xd6, 0x57, 0xa6, 0x31, 0x22, 0x7e, 0xe2, 0xae, 0x39, 0x48, 0xa5, 0x5c,
            0x37, 0xa1, 0xfd, 0x5e,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicTypeResponse {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicTypeResponse,
        "rosapi/TopicTypeResponse",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::TopicTypeResponse_",
        &[
            0xce, 0x0d, 0x9b, 0xa7, 0x2d, 0x71, 0x04, 0x8f, 0xcb, 0xb3, 0xd0, 0x34, 0x50, 0x30,
            0xa8, 0xb0, 0xdc, 0xb7, 0x5b, 0x79, 0x86, 0x85, 0x0a, 0x1d, 0x78, 0xa8, 0xeb, 0xc2,
            0x64, 0xfe, 0xaf, 0xf7,
        ],
    );
    #[allow(dead_code)]
    pub struct TopicType {}
    impl ::roslibrust::RosServiceType for TopicType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicType";
        const MD5SUM: &'static str = "0d30b3f53a0fd5036523a7141e524ddf";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xa8, 0xf2, 0xfd, 0x05, 0x7b, 0xfe, 0x43, 0x95, 0x43, 0x8c, 0x6c, 0xe1, 0x74, 0x80,
            0x17, 0x9b, 0x9d, 0x0c, 0x5a, 0x3b, 0x65, 0x77, 0x8a, 0x5c, 0x07, 0xf2, 0xb4, 0xc1,
            0x2f, 0x25, 0xeb, 0xa3,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::TopicType_";
        type Request = TopicTypeRequest;
        type Response = TopicTypeResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicsRequest {}
    ::roslibrust::impl_ros_message_type!(
        TopicsRequest,
        "rosapi/TopicsRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::TopicsRequest_",
        &[
            0xd1, 0xd6, 0x4f, 0xd4, 0x11, 0xb8, 0x19, 0xbb, 0x18, 0xea, 0x77, 0x3c, 0xbb, 0xe1,
            0x9e, 0xc5, 0xe6, 0x5d, 0x25, 0x5b, 0x42, 0xe0, 0x2d, 0xc2, 0x2d, 0xc7, 0xd2, 0xb5,
            0x1a, 0x89, 0x9d, 0xbd,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicsResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
        pub r#types: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicsResponse,
        "rosapi/TopicsResponse",
        "d966d98fc333fa1f3135af765eac1ba8",
        r####"string[] topics
string[] types"####,
        "rosapi::msg::dds_::TopicsResponse_",
        &[
            0xb5, 0xcb, 0x68, 0xc4, 0xad, 0x40, 0xc6, 0x3d, 0x7e, 0x7a, 0xc2, 0x94, 0x70, 0xbd,
            0x9a, 0x3b, 0x25, 0x20, 0x3b, 0xca, 0xdc, 0xa7, 0x53, 0xd2, 0xce, 0x21, 0x20, 0x52,
            0x3c, 0x8f, 0x9e, 0xd2,
        ],
    );
    #[allow(dead_code)]
    pub struct Topics {}
    impl ::roslibrust::RosServiceType for Topics {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Topics";
        const MD5SUM: &'static str = "d966d98fc333fa1f3135af765eac1ba8";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x7d, 0x2f, 0x05, 0x6d, 0x77, 0x42, 0xab, 0xc4, 0xce, 0xc7, 0x60, 0x2e, 0xf7, 0x59,
            0x6d, 0x3a, 0x9a, 0xf0, 0xc6, 0x37, 0xf1, 0x61, 0x1a, 0x89, 0xb5, 0x0f, 0xda, 0xbe,
            0x08, 0xd7, 0x01, 0x14,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::Topics_";
        type Request = TopicsRequest;
        type Response = TopicsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicsAndRawTypesRequest {}
    ::roslibrust::impl_ros_message_type!(
        TopicsAndRawTypesRequest,
        "rosapi/TopicsAndRawTypesRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "rosapi::msg::dds_::TopicsAndRawTypesRequest_",
        &[
            0x61, 0x62, 0xd5, 0x5a, 0x9f, 0x88, 0xa2, 0xcb, 0x82, 0xb0, 0x87, 0x0b, 0x88, 0xe2,
            0x59, 0xe9, 0xfb, 0xb8, 0xa3, 0x20, 0x3a, 0xb6, 0xd2, 0x51, 0x62, 0xb9, 0x37, 0xb9,
            0xa2, 0x62, 0x82, 0xa1,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicsAndRawTypesResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
        pub r#types: ::std::vec::Vec<::std::string::String>,
        pub r#typedefs_full_text: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicsAndRawTypesResponse,
        "rosapi/TopicsAndRawTypesResponse",
        "e1432466c8f64316723276ba07c59d12",
        r####"string[] topics
string[] types
string[] typedefs_full_text"####,
        "rosapi::msg::dds_::TopicsAndRawTypesResponse_",
        &[
            0xc9, 0xe0, 0x98, 0x91, 0xfb, 0xc7, 0x14, 0xd7, 0x23, 0x51, 0xec, 0x36, 0x5f, 0xe5,
            0xa1, 0x6b, 0xd6, 0x4f, 0x8f, 0x53, 0xa8, 0x04, 0xfc, 0xa8, 0xd6, 0xcd, 0x32, 0x0c,
            0x44, 0x31, 0x78, 0x91,
        ],
    );
    #[allow(dead_code)]
    pub struct TopicsAndRawTypes {}
    impl ::roslibrust::RosServiceType for TopicsAndRawTypes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsAndRawTypes";
        const MD5SUM: &'static str = "e1432466c8f64316723276ba07c59d12";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xd0, 0x23, 0xa2, 0x6f, 0x39, 0xf4, 0x7f, 0x14, 0x27, 0xd0, 0x4b, 0xb2, 0x55, 0xea,
            0x14, 0xa9, 0x5b, 0x35, 0x9b, 0x0a, 0x7d, 0x43, 0xcd, 0x13, 0x75, 0x60, 0x16, 0x91,
            0x69, 0x25, 0x6e, 0x5b,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::TopicsAndRawTypes_";
        type Request = TopicsAndRawTypesRequest;
        type Response = TopicsAndRawTypesResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicsForTypeRequest {
        pub r#type: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicsForTypeRequest,
        "rosapi/TopicsForTypeRequest",
        "dc67331de85cf97091b7d45e5c64ab75",
        r####"string type"####,
        "rosapi::msg::dds_::TopicsForTypeRequest_",
        &[
            0x43, 0x7d, 0x45, 0x6b, 0xfd, 0x0c, 0xb8, 0x6f, 0xd7, 0x6a, 0xf7, 0x4d, 0x74, 0xce,
            0xe7, 0x53, 0x48, 0xe3, 0xc0, 0x77, 0x7c, 0xf6, 0xc2, 0xbd, 0xbc, 0xe5, 0xd8, 0xf4,
            0xe8, 0x1f, 0x87, 0x0d,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicsForTypeResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicsForTypeResponse,
        "rosapi/TopicsForTypeResponse",
        "b0eef9a05d4e829092fc2f2c3c2aad3d",
        r####"string[] topics"####,
        "rosapi::msg::dds_::TopicsForTypeResponse_",
        &[
            0x86, 0xa0, 0x8a, 0x20, 0xcb, 0x73, 0x0a, 0x10, 0xb5, 0x36, 0x17, 0xab, 0x93, 0x95,
            0xb5, 0x54, 0xb0, 0x42, 0x72, 0x97, 0x4a, 0xf9, 0x0b, 0x72, 0x1b, 0x5b, 0x7b, 0xf6,
            0xd2, 0xbb, 0x34, 0xe9,
        ],
    );
    #[allow(dead_code)]
    pub struct TopicsForType {}
    impl ::roslibrust::RosServiceType for TopicsForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsForType";
        const MD5SUM: &'static str = "56f77ff6da756dd27c1ed16ec721072a";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xe9, 0xb7, 0x2a, 0x18, 0xa5, 0x2b, 0x02, 0xf6, 0x95, 0x9c, 0x83, 0x83, 0x2f, 0x71,
            0x8d, 0x09, 0x21, 0x89, 0x95, 0x98, 0x78, 0xdc, 0xb0, 0xcb, 0x53, 0x31, 0xf8, 0xee,
            0x8c, 0x6f, 0x3f, 0xf8,
        ];
        const ROS2_TYPE_NAME: &'static str = "rosapi::srv::dds_::TopicsForType_";
        type Request = TopicsForTypeRequest;
        type Response = TopicsForTypeResponse;
    }
}
#[allow(unused_imports)]
pub mod rosgraph_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Clock {
        pub r#clock: ::roslibrust::codegen::integral_types::Time,
    }
    ::roslibrust::impl_ros_message_type!(
        Clock,
        "rosgraph_msgs/Clock",
        "a9c97c1d230cfc112e270351a944ee47",
        r####"# roslib/Clock is used for publishing simulated time in ROS. 
# This message simply communicates the current time.
# For more information, see http://www.ros.org/wiki/Clock
time clock"####,
        "rosgraph_msgs::msg::dds_::Clock_",
        &[
            0x6b, 0x06, 0x21, 0xf2, 0xe1, 0xb2, 0xe3, 0xe2, 0x2c, 0x32, 0x01, 0x3a, 0x77, 0xc6,
            0xdb, 0x28, 0xa1, 0x04, 0x40, 0x3f, 0x93, 0x76, 0x22, 0xc4, 0x25, 0xa4, 0xa3, 0x82,
            0x30, 0x63, 0xb8, 0xe4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Log {
        pub r#header: std_msgs::Header,
        pub r#level: u8,
        pub r#name: ::std::string::String,
        pub r#msg: ::std::string::String,
        pub r#file: ::std::string::String,
        pub r#function: ::std::string::String,
        pub r#line: u32,
        pub r#topics: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        Log,
        "rosgraph_msgs/Log",
        "acffd30cd6b6de30f120938c17c593fb",
        r####"##
## Severity level constants
##
byte DEBUG=1 #debug level
byte INFO=2  #general level
byte WARN=4  #warning level
byte ERROR=8 #error level
byte FATAL=16 #fatal/critical level
##
## Fields
##
Header header
byte level
string name # name of the node
string msg # message 
string file # file the message came from
string function # function the message came from
uint32 line # line the message came from
string[] topics # topic names that the node publishes
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "rosgraph_msgs::msg::dds_::Log_",
        &[
            0x5c, 0x0f, 0x95, 0xcb, 0x3b, 0x25, 0x12, 0x36, 0x03, 0xef, 0x38, 0xc1, 0x5a, 0xed,
            0xe0, 0x88, 0xca, 0x5d, 0xa5, 0x4c, 0x50, 0x73, 0x85, 0xa2, 0x2a, 0xe0, 0x8e, 0x27,
            0x8f, 0x6c, 0x96, 0x6e,
        ],
    );
    #[allow(unused)]
    impl Log {
        pub const r#DEBUG: u8 = 1u8;
        pub const r#INFO: u8 = 2u8;
        pub const r#WARN: u8 = 4u8;
        pub const r#ERROR: u8 = 8u8;
        pub const r#FATAL: u8 = 16u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TopicStatistics {
        pub r#topic: ::std::string::String,
        pub r#node_pub: ::std::string::String,
        pub r#node_sub: ::std::string::String,
        pub r#window_start: ::roslibrust::codegen::integral_types::Time,
        pub r#window_stop: ::roslibrust::codegen::integral_types::Time,
        pub r#delivered_msgs: i32,
        pub r#dropped_msgs: i32,
        pub r#traffic: i32,
        pub r#period_mean: ::roslibrust::codegen::integral_types::Duration,
        pub r#period_stddev: ::roslibrust::codegen::integral_types::Duration,
        pub r#period_max: ::roslibrust::codegen::integral_types::Duration,
        pub r#stamp_age_mean: ::roslibrust::codegen::integral_types::Duration,
        pub r#stamp_age_stddev: ::roslibrust::codegen::integral_types::Duration,
        pub r#stamp_age_max: ::roslibrust::codegen::integral_types::Duration,
    }
    ::roslibrust::impl_ros_message_type!(
        TopicStatistics,
        "rosgraph_msgs/TopicStatistics",
        "10152ed868c5097a5e2e4a89d7daa710",
        r####"# name of the topic
string topic

# node id of the publisher
string node_pub

# node id of the subscriber
string node_sub

# the statistics apply to this time window
time window_start
time window_stop

# number of messages delivered during the window
int32 delivered_msgs
# numbers of messages dropped during the window
int32 dropped_msgs

# traffic during the window, in bytes
int32 traffic

# mean/stddev/max period between two messages
duration period_mean
duration period_stddev
duration period_max

# mean/stddev/max age of the message based on the
# timestamp in the message header. In case the
# message does not have a header, it will be 0.
duration stamp_age_mean
duration stamp_age_stddev
duration stamp_age_max"####,
        "rosgraph_msgs::msg::dds_::TopicStatistics_",
        &[
            0x1a, 0x05, 0xb4, 0x0f, 0xe8, 0x40, 0x3f, 0x51, 0xa9, 0x28, 0xcc, 0xe6, 0x0c, 0x63,
            0xf1, 0x4b, 0xfe, 0x13, 0x8a, 0x2e, 0xf8, 0xce, 0x6e, 0xeb, 0x16, 0x29, 0x56, 0x37,
            0xab, 0x97, 0xf6, 0x05,
        ],
    );
}
#[allow(unused_imports)]
pub mod sensor_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
    ::roslibrust::impl_ros_message_type!(
        BatteryState,
        "sensor_msgs/BatteryState",
        "4ddae7f048e32fda22cac764685e3974",
        r####"# Constants are chosen to match the enums in the linux kernel
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

Header  header
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
float32[] cell_temperature  # An array of individual cell temperatures for each cell in the pack
                            # If individual temperatures unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::BatteryState_",
        &[
            0x28, 0xe2, 0x8d, 0x61, 0x2a, 0x5e, 0xc8, 0x4b, 0x87, 0x79, 0x53, 0x29, 0x20, 0x5e,
            0x0b, 0xe9, 0xaa, 0xd4, 0x0b, 0x44, 0x19, 0xdf, 0xcc, 0xcb, 0x41, 0x8d, 0xbf, 0x49,
            0xc4, 0xd3, 0xa2, 0xc3,
        ],
    );
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
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct CameraInfo {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#distortion_model: ::std::string::String,
        pub r#D: ::std::vec::Vec<f64>,
        pub r#K: [f64; 9],
        pub r#R: [f64; 9],
        pub r#P: [f64; 12],
        pub r#binning_x: u32,
        pub r#binning_y: u32,
        pub r#roi: self::RegionOfInterest,
    }
    ::roslibrust::impl_ros_message_type!(
        CameraInfo,
        "sensor_msgs/CameraInfo",
        "c9a58c1b0b154e0e6da7578cb991d214",
        r####"# This message defines meta information for a camera. It should be in a
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
Header header    # Header timestamp should be acquisition time of image
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

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

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
float64[12] P # 3x4 row-major matrix


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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::CameraInfo_",
        &[
            0x1e, 0xa8, 0x47, 0x84, 0x96, 0x66, 0xa0, 0x5a, 0xeb, 0x31, 0xf0, 0x03, 0x04, 0x34,
            0x5a, 0xf8, 0x63, 0x5f, 0xa1, 0x5f, 0x7b, 0x8c, 0x83, 0x37, 0x77, 0x47, 0x17, 0xfb,
            0x84, 0xa9, 0x70, 0x5d,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ChannelFloat32 {
        pub r#name: ::std::string::String,
        pub r#values: ::std::vec::Vec<f32>,
    }
    ::roslibrust::impl_ros_message_type!(
        ChannelFloat32,
        "sensor_msgs/ChannelFloat32",
        "3d40139cdd33dfedcb71ffeeeb42ae7f",
        r####"# This message is used by the PointCloud message to hold optional data
# associated with each point in the cloud. The length of the values
# array should be the same as the length of the points array in the
# PointCloud, and each value should be associated with the corresponding
# point.

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
float32[] values"####,
        "sensor_msgs::msg::dds_::ChannelFloat32_",
        &[
            0x92, 0x66, 0x54, 0x37, 0xdd, 0xf3, 0x93, 0x46, 0xf4, 0xba, 0x39, 0xee, 0x32, 0xe6,
            0x48, 0x39, 0x06, 0x05, 0xb6, 0x33, 0xcc, 0x07, 0x7d, 0x40, 0xf4, 0xbd, 0x4d, 0x7b,
            0x58, 0xaf, 0x6c, 0xd4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct CompressedImage {
        pub r#header: std_msgs::Header,
        pub r#format: ::std::string::String,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    ::roslibrust::impl_ros_message_type!(
        CompressedImage,
        "sensor_msgs/CompressedImage",
        "8f7a12909da2c9d3332d540a0977563f",
        r####"# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::CompressedImage_",
        &[
            0x4a, 0xf0, 0x55, 0x4e, 0x90, 0x85, 0x18, 0x8d, 0xdd, 0x48, 0xef, 0x5b, 0x15, 0x26,
            0x84, 0xb8, 0xa8, 0xdb, 0x92, 0x88, 0x22, 0x89, 0xbf, 0xb0, 0x88, 0x2d, 0x0a, 0xf8,
            0xbb, 0xa6, 0x61, 0x3e,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct FluidPressure {
        pub r#header: std_msgs::Header,
        pub r#fluid_pressure: f64,
        pub r#variance: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        FluidPressure,
        "sensor_msgs/FluidPressure",
        "804dc5cea1c5306d6a2eb80b9833befe",
        r####"# Single pressure reading.  This message is appropriate for measuring the
 # pressure inside of a fluid (air, water, etc).  This also includes
 # atmospheric or barometric pressure.

 # This message is not appropriate for force/pressure contact sensors.

 Header header           # timestamp of the measurement
                         # frame_id is the location of the pressure sensor

 float64 fluid_pressure  # Absolute pressure reading in Pascals.

 float64 variance        # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::FluidPressure_",
        &[
            0xa1, 0x09, 0x35, 0x7f, 0x31, 0x70, 0x6d, 0x7b, 0x3e, 0x6b, 0x14, 0xd7, 0x88, 0x02,
            0x10, 0x4e, 0xfa, 0x5e, 0x4b, 0x45, 0xaf, 0x3a, 0x6f, 0x65, 0xa0, 0x71, 0xf5, 0x09,
            0xb4, 0x67, 0xab, 0xad,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Illuminance {
        pub r#header: std_msgs::Header,
        pub r#illuminance: f64,
        pub r#variance: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Illuminance,
        "sensor_msgs/Illuminance",
        "8cf5febb0952fca9d650c3d11a81a188",
        r####"# Single photometric illuminance measurement.  Light should be assumed to be
 # measured along the sensor's x-axis (the area of detection is the y-z plane).
 # The illuminance should have a 0 or positive value and be received with
 # the sensor's +X axis pointing toward the light source.

 # Photometric illuminance is the measure of the human eye's sensitivity of the
 # intensity of light encountering or passing through a surface.

 # All other Photometric and Radiometric measurements should
 # not use this message.
 # This message cannot represent:
 # Luminous intensity (candela/light source output)
 # Luminance (nits/light output per area)
 # Irradiance (watt/area), etc.

 Header header           # timestamp is the time the illuminance was measured
                         # frame_id is the location and direction of the reading

 float64 illuminance     # Measurement of the Photometric Illuminance in Lux.

 float64 variance        # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::Illuminance_",
        &[
            0xcc, 0xfa, 0x71, 0x19, 0x30, 0x0a, 0x44, 0xd6, 0x2d, 0xe6, 0xf3, 0xde, 0x1b, 0x5c,
            0x22, 0x50, 0x81, 0x67, 0x89, 0x6b, 0x50, 0x1a, 0xcd, 0xee, 0x11, 0x98, 0x23, 0x6c,
            0x8c, 0x3b, 0xc7, 0x48,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Image {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#encoding: ::std::string::String,
        pub r#is_bigendian: u8,
        pub r#step: u32,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    ::roslibrust::impl_ros_message_type!(
        Image,
        "sensor_msgs/Image",
        "060021388200f6f0f447d0fcd9c64743",
        r####"# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::Image_",
        &[
            0xc1, 0x3f, 0xc6, 0x16, 0x1b, 0x63, 0x02, 0x05, 0x02, 0x6c, 0xd8, 0xd0, 0xd9, 0x30,
            0xff, 0x86, 0x6e, 0x6c, 0xa1, 0x62, 0xc2, 0x3f, 0x0b, 0x3b, 0x7f, 0xda, 0x23, 0xb0,
            0xf5, 0xbb, 0x93, 0x0b,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Imu {
        pub r#header: std_msgs::Header,
        pub r#orientation: geometry_msgs::Quaternion,
        pub r#orientation_covariance: [f64; 9],
        pub r#angular_velocity: geometry_msgs::Vector3,
        pub r#angular_velocity_covariance: [f64; 9],
        pub r#linear_acceleration: geometry_msgs::Vector3,
        pub r#linear_acceleration_covariance: [f64; 9],
    }
    ::roslibrust::impl_ros_message_type!(
        Imu,
        "sensor_msgs/Imu",
        "6a62c6daae103f4ff57a132d6f95cec2",
        r####"# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::Imu_",
        &[
            0xa4, 0x50, 0x38, 0xe7, 0x78, 0x1c, 0xb3, 0xeb, 0xb0, 0x27, 0xdc, 0x85, 0x0f, 0x67,
            0xff, 0x65, 0xe4, 0x92, 0xa8, 0xf8, 0xb6, 0xd4, 0x8d, 0x9d, 0xda, 0xd9, 0x9d, 0xc8,
            0x44, 0x3a, 0xfb, 0xb4,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct JointState {
        pub r#header: std_msgs::Header,
        pub r#name: ::std::vec::Vec<::std::string::String>,
        pub r#position: ::std::vec::Vec<f64>,
        pub r#velocity: ::std::vec::Vec<f64>,
        pub r#effort: ::std::vec::Vec<f64>,
    }
    ::roslibrust::impl_ros_message_type!(
        JointState,
        "sensor_msgs/JointState",
        "3066dcd76a6cfaef579bd0f34173e9fd",
        r####"# This is a message that holds data to describe the state of a set of torque controlled joints. 
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


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::JointState_",
        &[
            0xff, 0xe2, 0x29, 0x7a, 0xb9, 0x81, 0x1a, 0xb3, 0x6d, 0xcf, 0x6d, 0x1e, 0xad, 0xb8,
            0x7f, 0x73, 0xb3, 0x46, 0x0d, 0x95, 0x7f, 0x65, 0x58, 0x57, 0xd8, 0x04, 0x6c, 0x59,
            0x64, 0xd3, 0x1b, 0xed,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Joy {
        pub r#header: std_msgs::Header,
        pub r#axes: ::std::vec::Vec<f32>,
        pub r#buttons: ::std::vec::Vec<i32>,
    }
    ::roslibrust::impl_ros_message_type!(
        Joy,
        "sensor_msgs/Joy",
        "5a9ea5f83505693b71e785041e67a8bb",
        r####"# Reports the state of a joysticks axes and buttons.
Header header           # timestamp in the header is the time the data is received from the joystick
float32[] axes          # the axes measurements from a joystick
int32[] buttons         # the buttons measurements from a joystick
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::Joy_",
        &[
            0x38, 0x02, 0x3f, 0xfa, 0xd2, 0x7a, 0xce, 0xbd, 0xfd, 0xe4, 0xd5, 0xc0, 0xc6, 0xc0,
            0x76, 0xbc, 0xe6, 0x5f, 0xf6, 0x6f, 0x97, 0xb2, 0xfc, 0xa3, 0xdf, 0x33, 0x7a, 0x5f,
            0x1e, 0xc6, 0x3b, 0x8a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct JoyFeedback {
        pub r#type: u8,
        pub r#id: u8,
        pub r#intensity: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        JoyFeedback,
        "sensor_msgs/JoyFeedback",
        "f4dcd73460360d98f36e55ee7f2e46f1",
        r####"# Declare of the type of feedback
uint8 TYPE_LED    = 0
uint8 TYPE_RUMBLE = 1
uint8 TYPE_BUZZER = 2

uint8 type

# This will hold an id number for each type of each feedback.
# Example, the first led would be id=0, the second would be id=1
uint8 id

# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
float32 intensity"####,
        "sensor_msgs::msg::dds_::JoyFeedback_",
        &[
            0x23, 0x1d, 0xd3, 0x62, 0xf7, 0x1d, 0x6f, 0xc0, 0x82, 0x72, 0x77, 0x0d, 0x07, 0x12,
            0x0a, 0xd5, 0xfe, 0x58, 0x74, 0xce, 0x2d, 0xba, 0xc7, 0x01, 0x09, 0xb2, 0x89, 0x86,
            0x83, 0x42, 0x90, 0xcd,
        ],
    );
    #[allow(unused)]
    impl JoyFeedback {
        pub const r#TYPE_LED: u8 = 0u8;
        pub const r#TYPE_RUMBLE: u8 = 1u8;
        pub const r#TYPE_BUZZER: u8 = 2u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct JoyFeedbackArray {
        pub r#array: ::std::vec::Vec<self::JoyFeedback>,
    }
    ::roslibrust::impl_ros_message_type!(
        JoyFeedbackArray,
        "sensor_msgs/JoyFeedbackArray",
        "cde5730a895b1fc4dee6f91b754b213d",
        r####"# This message publishes values for multiple feedback at once. 
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
float32 intensity"####,
        "sensor_msgs::msg::dds_::JoyFeedbackArray_",
        &[
            0x32, 0x87, 0xc3, 0x2e, 0x1b, 0x68, 0x8c, 0xae, 0x04, 0x55, 0x5e, 0x46, 0x54, 0x43,
            0xdf, 0x3c, 0xca, 0x7d, 0xae, 0x76, 0xee, 0x4e, 0xbf, 0x85, 0xc4, 0x65, 0x8d, 0x58,
            0x50, 0x37, 0xbc, 0xaa,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct LaserEcho {
        pub r#echoes: ::std::vec::Vec<f32>,
    }
    ::roslibrust::impl_ros_message_type!(
        LaserEcho,
        "sensor_msgs/LaserEcho",
        "8bc5ae449b200fba4d552b4225586696",
        r####"# This message is a submessage of MultiEchoLaserScan and is not intended
# to be used separately.

float32[] echoes  # Multiple values of ranges or intensities.
                  # Each array represents data from the same angle increment."####,
        "sensor_msgs::msg::dds_::LaserEcho_",
        &[
            0x0f, 0xbc, 0x05, 0xa0, 0xdb, 0x7d, 0x37, 0xfe, 0x52, 0xc0, 0xf0, 0x37, 0x53, 0x56,
            0xdb, 0x55, 0xda, 0x00, 0x46, 0xf7, 0xef, 0x5b, 0xd2, 0x7c, 0xa6, 0xb3, 0x4b, 0xd0,
            0x58, 0x2b, 0xc9, 0x52,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
    ::roslibrust::impl_ros_message_type!(
        LaserScan,
        "sensor_msgs/LaserScan",
        "90c7ef2dc6895d81024acba2ac42f369",
        r####"# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::LaserScan_",
        &[
            0x4e, 0x71, 0x54, 0x19, 0xc4, 0x42, 0xcf, 0xc8, 0xf3, 0xd0, 0x14, 0x4c, 0xf3, 0x8a,
            0xc1, 0xff, 0x9b, 0x45, 0xbc, 0xb6, 0xff, 0xa8, 0x89, 0xb3, 0xeb, 0xd9, 0xc2, 0x6f,
            0x9f, 0x66, 0x61, 0xd5,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MagneticField {
        pub r#header: std_msgs::Header,
        pub r#magnetic_field: geometry_msgs::Vector3,
        pub r#magnetic_field_covariance: [f64; 9],
    }
    ::roslibrust::impl_ros_message_type!(
        MagneticField,
        "sensor_msgs/MagneticField",
        "2f3b0b43eed0c9501de0fa3ff89a45aa",
        r####"# Measurement of the Magnetic Field vector at a specific location.

 # If the covariance of the measurement is known, it should be filled in
 # (if all you know is the variance of each measurement, e.g. from the datasheet,
 #just put those along the diagonal)
 # A covariance matrix of all zeros will be interpreted as "covariance unknown",
 # and to use the data a covariance will have to be assumed or gotten from some
 # other source


 Header header                        # timestamp is the time the
                                      # field was measured
                                      # frame_id is the location and orientation
                                      # of the field measurement

 geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                      # field vector in Tesla
                                      # If your sensor does not output 3 axes,
                                      # put NaNs in the components not reported.

 float64[9] magnetic_field_covariance # Row major about x, y, z axes
                                      # 0 is interpreted as variance unknown
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::MagneticField_",
        &[
            0x76, 0x81, 0xec, 0x95, 0x14, 0x90, 0x6f, 0x9c, 0xe2, 0xf1, 0xab, 0x4c, 0x59, 0x49,
            0xac, 0x44, 0xf8, 0xae, 0xa6, 0x29, 0x6f, 0x63, 0x2e, 0xf7, 0x47, 0xa4, 0x8d, 0xc9,
            0x42, 0x4a, 0x16, 0x6f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MultiDOFJointState {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#twist: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#wrench: ::std::vec::Vec<geometry_msgs::Wrench>,
    }
    ::roslibrust::impl_ros_message_type!(
        MultiDOFJointState,
        "sensor_msgs/MultiDOFJointState",
        "690f272f0640d2631c305eeb8301e59d",
        r####"# Representation of state for joints with multiple degrees of freedom, 
# following the structure of JointState.
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

Header header

string[] joint_names
geometry_msgs/Transform[] transforms
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::MultiDOFJointState_",
        &[
            0x00, 0xd2, 0xb7, 0x53, 0x0f, 0x6e, 0x56, 0x11, 0x85, 0x28, 0x7e, 0xd0, 0x23, 0xae,
            0xe3, 0x1c, 0xb5, 0x4f, 0x50, 0x5e, 0x41, 0x15, 0x09, 0xe2, 0x9d, 0xdb, 0x0b, 0xf7,
            0xb0, 0x88, 0x47, 0x24,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
    ::roslibrust::impl_ros_message_type!(
        MultiEchoLaserScan,
        "sensor_msgs/MultiEchoLaserScan",
        "6fefb0c6da89d7c8abe4b339f5c2f8fb",
        r####"# Single scan from a multi-echo planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

LaserEcho[] ranges       # range data [m] (Note: NaNs, values < range_min or > range_max should be discarded)
                         # +Inf measurements are out of range
                         # -Inf measurements are too close to determine exact distance.
LaserEcho[] intensities  # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::MultiEchoLaserScan_",
        &[
            0x8a, 0xa1, 0x86, 0x13, 0x76, 0xac, 0x6a, 0xef, 0x73, 0x18, 0x26, 0x66, 0x21, 0x95,
            0x3f, 0xfe, 0xca, 0x5c, 0xa3, 0x64, 0x13, 0xaa, 0xcb, 0x7b, 0x02, 0xf7, 0xfc, 0xdf,
            0x05, 0x7e, 0xe3, 0x25,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NavSatFix {
        pub r#header: std_msgs::Header,
        pub r#status: self::NavSatStatus,
        pub r#latitude: f64,
        pub r#longitude: f64,
        pub r#altitude: f64,
        pub r#position_covariance: [f64; 9],
        pub r#position_covariance_type: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        NavSatFix,
        "sensor_msgs/NavSatFix",
        "2d3a8cd499b9b4a0249fb98fd05cfa48",
        r####"# Navigation Satellite fix for any Global Navigation Satellite System
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
Header header

# satellite fix status information
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
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::NavSatFix_",
        &[
            0xb5, 0x67, 0x5d, 0x95, 0xbf, 0x92, 0xa5, 0x67, 0x76, 0x09, 0x64, 0x3b, 0x52, 0x47,
            0x84, 0x4a, 0x6f, 0x58, 0x62, 0x1e, 0xc8, 0xa2, 0x2d, 0x57, 0x70, 0x02, 0x9d, 0x28,
            0xbe, 0x4d, 0xc7, 0x61,
        ],
    );
    #[allow(unused)]
    impl NavSatFix {
        pub const r#COVARIANCE_TYPE_UNKNOWN: u8 = 0u8;
        pub const r#COVARIANCE_TYPE_APPROXIMATED: u8 = 1u8;
        pub const r#COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2u8;
        pub const r#COVARIANCE_TYPE_KNOWN: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NavSatStatus {
        pub r#status: i8,
        pub r#service: u16,
    }
    ::roslibrust::impl_ros_message_type!(
        NavSatStatus,
        "sensor_msgs/NavSatStatus",
        "331cdbddfa4bc96ffc3b9ad98900a54c",
        r####"# Navigation Satellite fix status for any Global Navigation Satellite System

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

uint16 service"####,
        "sensor_msgs::msg::dds_::NavSatStatus_",
        &[
            0xd1, 0xed, 0x3b, 0xef, 0xa6, 0x28, 0xe0, 0x95, 0x71, 0xbd, 0x27, 0x3b, 0x88, 0x8b,
            0xa1, 0xc1, 0xfd, 0x18, 0x7c, 0x9a, 0x5e, 0x00, 0x06, 0xb3, 0x85, 0xd7, 0xe5, 0xe9,
            0x09, 0x5a, 0x32, 0x04,
        ],
    );
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
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PointCloud {
        pub r#header: std_msgs::Header,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point32>,
        pub r#channels: ::std::vec::Vec<self::ChannelFloat32>,
    }
    ::roslibrust::impl_ros_message_type!(
        PointCloud,
        "sensor_msgs/PointCloud",
        "d8e9c3f5afbdd8a130fd1d2763945fca",
        r####"# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::PointCloud_",
        &[
            0x3d, 0xdc, 0x82, 0x8a, 0x93, 0x82, 0x15, 0x4b, 0xc7, 0xe4, 0x38, 0x2d, 0x80, 0x12,
            0x7a, 0x1a, 0xaa, 0xc2, 0x0e, 0x9a, 0x69, 0x1e, 0x26, 0x42, 0x39, 0x8c, 0xc3, 0xa2,
            0x1e, 0x5a, 0x11, 0x08,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PointCloud2 {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#fields: ::std::vec::Vec<self::PointField>,
        pub r#is_bigendian: bool,
        pub r#point_step: u32,
        pub r#row_step: u32,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
        pub r#is_dense: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        PointCloud2,
        "sensor_msgs/PointCloud2",
        "1158d486dd51d683ce2f1be655c3c181",
        r####"# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

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

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::PointCloud2_",
        &[
            0x1d, 0x07, 0x98, 0xdd, 0x95, 0x68, 0x64, 0x35, 0x83, 0x40, 0x5c, 0x7e, 0xb9, 0x69,
            0x82, 0x8c, 0x34, 0x41, 0x1f, 0xc4, 0xe8, 0x2b, 0xd4, 0xe6, 0xf3, 0x39, 0xa1, 0x61,
            0xee, 0x21, 0x64, 0xa5,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct PointField {
        pub r#name: ::std::string::String,
        pub r#offset: u32,
        pub r#datatype: u8,
        pub r#count: u32,
    }
    ::roslibrust::impl_ros_message_type!(
        PointField,
        "sensor_msgs/PointField",
        "268eacb2962780ceac86cbd17e328150",
        r####"# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field"####,
        "sensor_msgs::msg::dds_::PointField_",
        &[
            0x5c, 0x6a, 0x47, 0x50, 0x72, 0x8c, 0x2b, 0xcf, 0xbb, 0xf7, 0x03, 0x72, 0x25, 0xb2,
            0x0b, 0x02, 0xd4, 0x42, 0x96, 0x34, 0x73, 0x21, 0x46, 0xb7, 0x42, 0xde, 0xe1, 0x72,
            0x66, 0x37, 0xef, 0x01,
        ],
    );
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
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Range {
        pub r#header: std_msgs::Header,
        pub r#radiation_type: u8,
        pub r#field_of_view: f32,
        pub r#min_range: f32,
        pub r#max_range: f32,
        pub r#range: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        Range,
        "sensor_msgs/Range",
        "c005c34273dc426c67a020a87bc24148",
        r####"# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.

# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

Header header           # timestamp in the header is the time the ranger
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
                        # (Note: values < range_min or > range_max
                        # should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::Range_",
        &[
            0xd7, 0xfe, 0x94, 0x75, 0xb0, 0x2d, 0x2d, 0x4d, 0xb0, 0xf7, 0x1d, 0xa2, 0x2f, 0x39,
            0x54, 0x3f, 0x8a, 0x1e, 0x3b, 0x73, 0xdc, 0xaf, 0x75, 0xee, 0xfe, 0xa2, 0x53, 0x90,
            0x86, 0xea, 0xc4, 0xf1,
        ],
    );
    #[allow(unused)]
    impl Range {
        pub const r#ULTRASOUND: u8 = 0u8;
        pub const r#INFRARED: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct RegionOfInterest {
        pub r#x_offset: u32,
        pub r#y_offset: u32,
        pub r#height: u32,
        pub r#width: u32,
        pub r#do_rectify: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        RegionOfInterest,
        "sensor_msgs/RegionOfInterest",
        "bdb633039d588fcccb441a4d43ccfe09",
        r####"# This message is used to specify a region of interest within an image.
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
bool do_rectify"####,
        "sensor_msgs::msg::dds_::RegionOfInterest_",
        &[
            0xad, 0x16, 0xbc, 0xba, 0x5f, 0x91, 0x31, 0xdc, 0xdb, 0xa6, 0xfb, 0xde, 0xd1, 0x9f,
            0x72, 0x6f, 0x54, 0x40, 0xe3, 0xc5, 0x13, 0xb4, 0xfb, 0x58, 0x6d, 0xd3, 0x02, 0x7e,
            0xee, 0xd8, 0xab, 0xb1,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct RelativeHumidity {
        pub r#header: std_msgs::Header,
        pub r#relative_humidity: f64,
        pub r#variance: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        RelativeHumidity,
        "sensor_msgs/RelativeHumidity",
        "8730015b05955b7e992ce29a2678d90f",
        r####"# Single reading from a relative humidity sensor.  Defines the ratio of partial
 # pressure of water vapor to the saturated vapor pressure at a temperature.

 Header header             # timestamp of the measurement
                           # frame_id is the location of the humidity sensor

 float64 relative_humidity # Expression of the relative humidity
                           # from 0.0 to 1.0.
                           # 0.0 is no partial pressure of water vapor
                           # 1.0 represents partial pressure of saturation

 float64 variance          # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::RelativeHumidity_",
        &[
            0x32, 0xde, 0x2b, 0xcb, 0x85, 0xff, 0x48, 0x6f, 0x74, 0xb8, 0x41, 0x1f, 0xd9, 0x8c,
            0xc7, 0x83, 0x1c, 0x07, 0x41, 0xae, 0x39, 0x39, 0xff, 0xa4, 0xb8, 0xb0, 0xfd, 0xe5,
            0xdb, 0x08, 0xf3, 0xf1,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Temperature {
        pub r#header: std_msgs::Header,
        pub r#temperature: f64,
        pub r#variance: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Temperature,
        "sensor_msgs/Temperature",
        "ff71b307acdbe7c871a5a6d7ed359100",
        r####"# Single temperature reading.

 Header header           # timestamp is the time the temperature was measured
                         # frame_id is the location of the temperature reading

 float64 temperature     # Measurement of the Temperature in Degrees Celsius

 float64 variance        # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::Temperature_",
        &[
            0x7a, 0x96, 0x1c, 0xab, 0x54, 0xeb, 0xff, 0x00, 0x71, 0x3f, 0x66, 0xd9, 0x79, 0x6d,
            0x8a, 0xed, 0xba, 0xe6, 0x27, 0xdc, 0x00, 0x92, 0x28, 0xe8, 0x88, 0x26, 0x97, 0x85,
            0x2a, 0xad, 0x0a, 0xe9,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TimeReference {
        pub r#header: std_msgs::Header,
        pub r#time_ref: ::roslibrust::codegen::integral_types::Time,
        pub r#source: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        TimeReference,
        "sensor_msgs/TimeReference",
        "fded64a0265108ba86c3d38fb11c0c16",
        r####"# Measurement from an external time source not actively synchronized with the system clock.

Header header    # stamp is system time for which measurement was valid
                 # frame_id is not used 

time   time_ref  # corresponding time from this external source
string source    # (optional) name of time source
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::TimeReference_",
        &[
            0x26, 0x70, 0x3f, 0xd8, 0x83, 0x0b, 0x3c, 0x5c, 0x37, 0x11, 0x71, 0xec, 0x7d, 0x70,
            0xd2, 0x5e, 0x76, 0x07, 0xc6, 0x38, 0xa8, 0xbf, 0x87, 0x96, 0xa7, 0x36, 0x7b, 0x4d,
            0xe3, 0x40, 0x1e, 0x28,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetCameraInfoRequest {
        pub r#camera_info: self::CameraInfo,
    }
    ::roslibrust::impl_ros_message_type!(
        SetCameraInfoRequest,
        "sensor_msgs/SetCameraInfoRequest",
        "ee34be01fdeee563d0d99cd594d5581d",
        r####"# This service requests that a camera stores the given CameraInfo 
# as that camera's calibration information.
#
# The width and height in the camera_info field should match what the
# camera is currently outputting on its camera_info topic, and the camera
# will assume that the region of the imager that is being referred to is
# the region that the camera is currently capturing.

sensor_msgs/CameraInfo camera_info # The camera_info to store
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
Header header    # Header timestamp should be acquisition time of image
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

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

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
float64[12] P # 3x4 row-major matrix


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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "sensor_msgs::msg::dds_::SetCameraInfoRequest_",
        &[
            0x15, 0x2e, 0xe9, 0x30, 0x14, 0xe5, 0x95, 0x64, 0x35, 0xac, 0x30, 0x00, 0x50, 0x43,
            0x4f, 0x13, 0x12, 0x2d, 0xed, 0x35, 0x17, 0x4b, 0xec, 0xd0, 0xa9, 0xda, 0xff, 0x38,
            0x36, 0x0a, 0x5f, 0x53,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetCameraInfoResponse {
        pub r#success: bool,
        pub r#status_message: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        SetCameraInfoResponse,
        "sensor_msgs/SetCameraInfoResponse",
        "2ec6f3eff0161f4257b808b12bc830c2",
        r####"bool success          # True if the call succeeded
string status_message # Used to give details about success"####,
        "sensor_msgs::msg::dds_::SetCameraInfoResponse_",
        &[
            0x3a, 0xe3, 0x0a, 0x6b, 0x9b, 0xc6, 0x64, 0x76, 0x99, 0x13, 0x15, 0x7d, 0x28, 0x66,
            0xd4, 0xcc, 0xbe, 0x5e, 0xcd, 0x7e, 0xd7, 0xb7, 0x6f, 0x70, 0x44, 0x97, 0xbc, 0x7d,
            0x29, 0x2d, 0x3d, 0x58,
        ],
    );
    #[allow(dead_code)]
    pub struct SetCameraInfo {}
    impl ::roslibrust::RosServiceType for SetCameraInfo {
        const ROS_SERVICE_NAME: &'static str = "sensor_msgs/SetCameraInfo";
        const MD5SUM: &'static str = "bef1df590ed75ed1f393692395e15482";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xdb, 0x6e, 0xa8, 0x1a, 0x98, 0x64, 0xde, 0x00, 0x74, 0xde, 0x64, 0x92, 0x6a, 0x05,
            0x45, 0xf3, 0xc2, 0x9d, 0xe9, 0xa0, 0xbb, 0xe6, 0x4e, 0x1f, 0x7a, 0xbc, 0xf5, 0xa4,
            0x06, 0xf8, 0x37, 0xd8,
        ];
        const ROS2_TYPE_NAME: &'static str = "sensor_msgs::srv::dds_::SetCameraInfo_";
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
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ServiceEventInfo {
        pub r#event_type: u8,
        pub r#stamp: builtin_interfaces::Time,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#client_gid: [u8; 16],
        pub r#sequence_number: i64,
    }
    ::roslibrust::impl_ros_message_type!(
        ServiceEventInfo,
        "service_msgs/ServiceEventInfo",
        "42561fc0d0d3665a03d59fbb1296daf9",
        r####"uint8 REQUEST_SENT = 0
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
uint32 nanosec"####,
        "service_msgs::msg::dds_::ServiceEventInfo_",
        &[
            0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5, 0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24,
            0xd7, 0xf0, 0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3, 0x37, 0x3c, 0x57, 0x32,
            0x34, 0x5a, 0x6f, 0x45,
        ],
    );
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
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Mesh {
        pub r#triangles: ::std::vec::Vec<self::MeshTriangle>,
        pub r#vertices: ::std::vec::Vec<geometry_msgs::Point>,
    }
    ::roslibrust::impl_ros_message_type!(
        Mesh,
        "shape_msgs/Mesh",
        "1ffdae9486cd3316a121c578b47a85cc",
        r####"# Definition of a mesh

# list of triangles; the index values refer to positions in vertices[]
MeshTriangle[] triangles

# the actual vertices that make up the mesh
geometry_msgs/Point[] vertices
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices
uint32[3] vertex_indices"####,
        "shape_msgs::msg::dds_::Mesh_",
        &[
            0xf2, 0x15, 0x0b, 0x82, 0xd8, 0xee, 0x7e, 0x8b, 0xc3, 0xf3, 0x96, 0xa2, 0xb1, 0x58,
            0xae, 0xfb, 0x4b, 0x9a, 0x55, 0x10, 0xa4, 0x74, 0xbe, 0x27, 0x1b, 0xa1, 0x26, 0x8a,
            0xeb, 0xb5, 0x52, 0x89,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MeshTriangle {
        pub r#vertex_indices: [u32; 3],
    }
    ::roslibrust::impl_ros_message_type!(
        MeshTriangle,
        "shape_msgs/MeshTriangle",
        "23688b2e6d2de3d32fe8af104a903253",
        r####"# Definition of a triangle's vertices
uint32[3] vertex_indices"####,
        "shape_msgs::msg::dds_::MeshTriangle_",
        &[
            0x61, 0x8e, 0x5c, 0x07, 0x3e, 0xeb, 0x72, 0x9e, 0x43, 0x3e, 0xf6, 0x22, 0x6e, 0x72,
            0xc0, 0x1d, 0x99, 0x5c, 0x45, 0x9f, 0xb7, 0xd7, 0x63, 0x48, 0xc9, 0x70, 0x04, 0x09,
            0xa5, 0x02, 0x0b, 0xd0,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Plane {
        pub r#coef: [f64; 4],
    }
    ::roslibrust::impl_ros_message_type!(
        Plane,
        "shape_msgs/Plane",
        "2c1b92ed8f31492f8e73f6a4a44ca796",
        r####"# Representation of a plane, using the plane equation ax + by + cz + d = 0

# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]

float64[4] coef"####,
        "shape_msgs::msg::dds_::Plane_",
        &[
            0xdf, 0xbf, 0xe8, 0x31, 0x46, 0x89, 0xc8, 0x50, 0x61, 0x5d, 0x4a, 0x72, 0x7a, 0xf0,
            0x17, 0xe9, 0xaa, 0x86, 0xc1, 0x0e, 0x36, 0x9a, 0x60, 0x6c, 0x8c, 0x85, 0x1e, 0xf8,
            0xf1, 0x6c, 0x58, 0xc8,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SolidPrimitive {
        pub r#type: u8,
        pub r#dimensions: ::std::vec::Vec<f64>,
    }
    ::roslibrust::impl_ros_message_type!(
        SolidPrimitive,
        "shape_msgs/SolidPrimitive",
        "d8f8cbc74c5ff283fca29569ccefb45d",
        r####"# Define box, sphere, cylinder, cone 
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type


# The dimensions of the shape
float64[] dimensions

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array

# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
# sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2


# For the SPHERE type, only one component is used, and it gives the radius of
# the sphere.
uint8 SPHERE_RADIUS=0


# For the CYLINDER and CONE types, the center line is oriented along
# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
# of dimensions gives the height of the cylinder (cone).  The
# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
# radius of the base of the cylinder (cone).  Cone and cylinder
# primitives are defined to be circular. The tip of the cone is
# pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1"####,
        "shape_msgs::msg::dds_::SolidPrimitive_",
        &[
            0x91, 0x3d, 0x1c, 0x15, 0x27, 0x13, 0x96, 0x77, 0x93, 0x71, 0xa1, 0x3f, 0x39, 0x02,
            0x00, 0x19, 0x90, 0xe6, 0x88, 0x2f, 0xf6, 0x58, 0x1a, 0xd1, 0x12, 0x51, 0x1d, 0x1f,
            0x08, 0x0c, 0x06, 0x29,
        ],
    );
    #[allow(unused)]
    impl SolidPrimitive {
        pub const r#BOX: u8 = 1u8;
        pub const r#SPHERE: u8 = 2u8;
        pub const r#CYLINDER: u8 = 3u8;
        pub const r#CONE: u8 = 4u8;
        pub const r#BOX_X: u8 = 0u8;
        pub const r#BOX_Y: u8 = 1u8;
        pub const r#BOX_Z: u8 = 2u8;
        pub const r#SPHERE_RADIUS: u8 = 0u8;
        pub const r#CYLINDER_HEIGHT: u8 = 0u8;
        pub const r#CYLINDER_RADIUS: u8 = 1u8;
        pub const r#CONE_HEIGHT: u8 = 0u8;
        pub const r#CONE_RADIUS: u8 = 1u8;
    }
}
#[allow(unused_imports)]
pub mod std_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Bool {
        pub r#data: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        Bool,
        "std_msgs/Bool",
        "8b94c1b53db61fb6aed406028ad6332a",
        r####"bool data"####,
        "std_msgs::msg::dds_::Bool_",
        &[
            0xfe, 0xb9, 0x1e, 0x99, 0x5f, 0xf9, 0xeb, 0xd0, 0x9c, 0x0c, 0xb3, 0xd2, 0xae, 0xd1,
            0x8b, 0x11, 0x07, 0x75, 0x85, 0x83, 0x9f, 0xb5, 0xdb, 0x80, 0x19, 0x3b, 0x62, 0xd7,
            0x45, 0x28, 0xf6, 0xc9,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Byte {
        pub r#data: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        Byte,
        "std_msgs/Byte",
        "ad736a2e8818154c487bb80fe42ce43b",
        r####"byte data"####,
        "std_msgs::msg::dds_::Byte_",
        &[
            0xe2, 0x8c, 0xa2, 0xc6, 0x2f, 0x3f, 0xb1, 0x0c, 0x20, 0x78, 0x90, 0x75, 0x5a, 0xa7,
            0xa5, 0xa7, 0x70, 0xcc, 0xde, 0x56, 0x46, 0xfd, 0x66, 0xb4, 0x52, 0xf4, 0x85, 0xc4,
            0x80, 0x92, 0xf3, 0x27,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ByteMultiArray {
        pub r#layout: self::MultiArrayLayout,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    ::roslibrust::impl_ros_message_type!(
        ByteMultiArray,
        "std_msgs/ByteMultiArray",
        "70ea476cbcfd65ac2f68f3cda1e891fe",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
byte[]            data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::ByteMultiArray_",
        &[
            0x69, 0x2e, 0xff, 0x26, 0xdd, 0x8c, 0xa7, 0x62, 0x3e, 0x4e, 0x90, 0xa0, 0x82, 0xf7,
            0xd8, 0x3f, 0x1c, 0xf5, 0xde, 0xb0, 0xb7, 0xba, 0x74, 0x8a, 0x2d, 0x4d, 0x5f, 0xbc,
            0xa7, 0x91, 0xdb, 0x7d,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Char {
        pub r#data: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        Char,
        "std_msgs/Char",
        "1bf77f25acecdedba0e224b162199717",
        r####"char data"####,
        "std_msgs::msg::dds_::Char_",
        &[
            0x3a, 0xd2, 0xd0, 0x4d, 0xd2, 0x9b, 0xa1, 0x9d, 0x04, 0xb1, 0x66, 0x59, 0xaf, 0xa3,
            0xcc, 0xae, 0xdd, 0x69, 0x19, 0x14, 0xb0, 0x2a, 0x64, 0xe8, 0x2e, 0x25, 0x2f, 0x2f,
            0xa6, 0xa5, 0x86, 0xa9,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ColorRGBA {
        pub r#r: f32,
        pub r#g: f32,
        pub r#b: f32,
        pub r#a: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        ColorRGBA,
        "std_msgs/ColorRGBA",
        "a29a96539573343b1310c73607334b00",
        r####"float32 r
float32 g
float32 b
float32 a"####,
        "std_msgs::msg::dds_::ColorRGBA_",
        &[
            0x77, 0xa7, 0xa5, 0xb9, 0xae, 0x47, 0x73, 0x06, 0x09, 0x76, 0x65, 0x10, 0x6e, 0x04,
            0x13, 0xba, 0x74, 0x44, 0x02, 0x45, 0xb1, 0xf3, 0xd0, 0xc6, 0xd6, 0x40, 0x5f, 0xe5,
            0xc7, 0x81, 0x3f, 0xe8,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Duration {
        pub r#data: ::roslibrust::codegen::integral_types::Duration,
    }
    ::roslibrust::impl_ros_message_type!(
        Duration,
        "std_msgs/Duration",
        "3e286caf4241d664e55f3ad380e2ae46",
        r####"duration data"####,
        "std_msgs::msg::dds_::Duration_",
        &[
            0x64, 0x12, 0xd7, 0xf2, 0x6b, 0x75, 0xc8, 0xbd, 0xcd, 0x9b, 0x9f, 0xea, 0xe3, 0xa8,
            0xa1, 0x15, 0x7f, 0x39, 0x50, 0x68, 0xa4, 0x0e, 0x97, 0xc0, 0x43, 0x7f, 0x1a, 0x34,
            0xfe, 0x1a, 0x37, 0x94,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Empty {}
    ::roslibrust::impl_ros_message_type!(
        Empty,
        "std_msgs/Empty",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "std_msgs::msg::dds_::Empty_",
        &[
            0x20, 0xb6, 0x25, 0x25, 0x6f, 0x32, 0xd5, 0xdb, 0xc0, 0xd0, 0x4f, 0xee, 0x44, 0xf4,
            0x3c, 0x41, 0xe5, 0x1c, 0x70, 0xd3, 0x50, 0x2f, 0x84, 0xb4, 0xa0, 0x8e, 0x7a, 0x9c,
            0x26, 0xa9, 0x63, 0x12,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Float32 {
        pub r#data: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        Float32,
        "std_msgs/Float32",
        "73fcbf46b49191e672908e50842a83d4",
        r####"float32 data"####,
        "std_msgs::msg::dds_::Float32_",
        &[
            0x71, 0x70, 0xd3, 0xd8, 0xf8, 0x41, 0xf7, 0xbe, 0x31, 0x72, 0xce, 0x5f, 0x4f, 0x59,
            0xf3, 0xa4, 0xd7, 0xf6, 0x3b, 0x04, 0x47, 0xe8, 0xb3, 0x33, 0x27, 0x60, 0x1a, 0xd6,
            0x4d, 0x83, 0xd6, 0xe2,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Float32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f32>,
    }
    ::roslibrust::impl_ros_message_type!(
        Float32MultiArray,
        "std_msgs/Float32MultiArray",
        "6a40e0ffa6a17a503ac3f8616991b1f6",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::Float32MultiArray_",
        &[
            0x05, 0x99, 0xf6, 0xf8, 0x5b, 0x4b, 0xfc, 0xa3, 0x79, 0x87, 0x3a, 0x0b, 0x43, 0x75,
            0xa0, 0xac, 0xa0, 0x22, 0x15, 0x6b, 0xd2, 0xd7, 0x02, 0x12, 0x75, 0xd1, 0x16, 0xed,
            0x1f, 0xa8, 0xbf, 0xe0,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Float64 {
        pub r#data: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Float64,
        "std_msgs/Float64",
        "fdb28210bfa9d7c91146260178d9a584",
        r####"float64 data"####,
        "std_msgs::msg::dds_::Float64_",
        &[
            0x70, 0x5b, 0xa9, 0xc3, 0xd1, 0xa0, 0x9d, 0xf4, 0x37, 0x37, 0xeb, 0x67, 0x09, 0x55,
            0x34, 0xde, 0x36, 0xfd, 0x42, 0x6c, 0x05, 0x87, 0x77, 0x9b, 0xda, 0x2b, 0xc5, 0x1f,
            0xe7, 0x90, 0x18, 0x2a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Float64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f64>,
    }
    ::roslibrust::impl_ros_message_type!(
        Float64MultiArray,
        "std_msgs/Float64MultiArray",
        "4b7d974086d4060e7db4613a7e6c3ba4",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::Float64MultiArray_",
        &[
            0x10, 0x25, 0xdd, 0xc6, 0xb9, 0x55, 0x2d, 0x19, 0x1f, 0x89, 0xef, 0x1a, 0x8d, 0x2f,
            0x60, 0xf3, 0xd3, 0x73, 0xe2, 0x8b, 0x28, 0x3d, 0x88, 0x91, 0xdd, 0xcc, 0x97, 0x4e,
            0x8c, 0x55, 0x39, 0x7f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Header {
        pub r#seq: u32,
        pub r#stamp: ::roslibrust::codegen::integral_types::Time,
        pub r#frame_id: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        Header,
        "std_msgs/Header",
        "2176decaecbce78abc3b96ef049fabed",
        r####"# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "std_msgs::msg::dds_::Header_",
        &[
            0xfc, 0xd1, 0x24, 0x61, 0x88, 0xef, 0xb1, 0x45, 0x97, 0x36, 0x58, 0x89, 0xd5, 0xad,
            0xcd, 0x52, 0xac, 0xb6, 0x89, 0x29, 0x29, 0xa1, 0xd2, 0x77, 0x05, 0xef, 0xcd, 0x34,
            0x3d, 0xc7, 0x96, 0x0f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int16 {
        pub r#data: i16,
    }
    ::roslibrust::impl_ros_message_type!(
        Int16,
        "std_msgs/Int16",
        "8524586e34fbd7cb1c08c5f5f1ca0e57",
        r####"int16 data"####,
        "std_msgs::msg::dds_::Int16_",
        &[
            0x1d, 0xcc, 0x34, 0x64, 0xe4, 0x7c, 0x28, 0x8a, 0x55, 0xf9, 0x43, 0xa3, 0x89, 0xd3,
            0x37, 0xcd, 0xb0, 0x68, 0x04, 0xde, 0x3f, 0x5c, 0xd7, 0xa2, 0x66, 0xb0, 0xde, 0x71,
            0x8e, 0xee, 0x17, 0xe5,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i16>,
    }
    ::roslibrust::impl_ros_message_type!(
        Int16MultiArray,
        "std_msgs/Int16MultiArray",
        "d9338d7f523fcb692fae9d0a0e9f067c",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int16[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::Int16MultiArray_",
        &[
            0xb5, 0x88, 0x10, 0xe8, 0xe5, 0xb9, 0x0f, 0xb1, 0x9a, 0x50, 0x62, 0x46, 0x9e, 0xb8,
            0x40, 0x9f, 0x5a, 0xb1, 0x1a, 0x44, 0x6d, 0x60, 0xde, 0x71, 0x57, 0xa1, 0x45, 0x7e,
            0x52, 0xa0, 0x76, 0xce,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int32 {
        pub r#data: i32,
    }
    ::roslibrust::impl_ros_message_type!(
        Int32,
        "std_msgs/Int32",
        "da5909fbe378aeaf85e547e830cc1bb7",
        r####"int32 data"####,
        "std_msgs::msg::dds_::Int32_",
        &[
            0xb6, 0x57, 0x8d, 0xed, 0x3c, 0x58, 0xc6, 0x26, 0xcf, 0xe8, 0xd1, 0xa6, 0xfb, 0x6e,
            0x04, 0xf7, 0x06, 0xf9, 0x7e, 0x9f, 0x03, 0xd2, 0x72, 0x7c, 0x9f, 0xf4, 0xe7, 0x4b,
            0x1c, 0xef, 0x0d, 0xeb,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i32>,
    }
    ::roslibrust::impl_ros_message_type!(
        Int32MultiArray,
        "std_msgs/Int32MultiArray",
        "1d99f79f8b325b44fee908053e9c945b",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int32[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::Int32MultiArray_",
        &[
            0x84, 0xa7, 0x34, 0x63, 0x23, 0x52, 0x5d, 0x1b, 0x4d, 0xfc, 0xa8, 0x99, 0xdf, 0x38,
            0x20, 0xf2, 0x45, 0xe5, 0x40, 0x09, 0xda, 0xc5, 0xa6, 0xb6, 0x92, 0x17, 0xd1, 0x4f,
            0xde, 0xfd, 0x17, 0x01,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int64 {
        pub r#data: i64,
    }
    ::roslibrust::impl_ros_message_type!(
        Int64,
        "std_msgs/Int64",
        "34add168574510e6e17f5d23ecc077ef",
        r####"int64 data"####,
        "std_msgs::msg::dds_::Int64_",
        &[
            0x8c, 0xd1, 0x04, 0x8c, 0x2f, 0x18, 0x6b, 0x6b, 0xd9, 0xa9, 0x24, 0x72, 0xdc, 0x1c,
            0xe5, 0x17, 0x23, 0xc0, 0x83, 0x3a, 0x22, 0x1e, 0x2b, 0x7a, 0xec, 0xff, 0xf1, 0x11,
            0x77, 0x4f, 0x4b, 0x49,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i64>,
    }
    ::roslibrust::impl_ros_message_type!(
        Int64MultiArray,
        "std_msgs/Int64MultiArray",
        "54865aa6c65be0448113a2afc6a49270",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int64[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::Int64MultiArray_",
        &[
            0xe6, 0x0f, 0x9f, 0xe3, 0x4d, 0x69, 0x7f, 0x09, 0x39, 0xad, 0x49, 0xd3, 0x31, 0x58,
            0x69, 0x3c, 0x12, 0x77, 0xfb, 0xac, 0x0e, 0x2f, 0x04, 0xb7, 0xc2, 0x99, 0x5d, 0xc2,
            0x1c, 0x89, 0xb4, 0x22,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int8 {
        pub r#data: i8,
    }
    ::roslibrust::impl_ros_message_type!(
        Int8,
        "std_msgs/Int8",
        "27ffa0c9c4b8fb8492252bcad9e5c57b",
        r####"int8 data"####,
        "std_msgs::msg::dds_::Int8_",
        &[
            0x26, 0x52, 0x50, 0x65, 0xa4, 0x03, 0xd9, 0x72, 0xcb, 0x67, 0x2f, 0x07, 0x77, 0xe3,
            0x33, 0xf0, 0xc7, 0x99, 0xad, 0x44, 0x4a, 0xe5, 0xfc, 0xd7, 0x9e, 0x43, 0xd1, 0xe7,
            0x3b, 0xd0, 0xf4, 0x40,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Int8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i8>,
    }
    ::roslibrust::impl_ros_message_type!(
        Int8MultiArray,
        "std_msgs/Int8MultiArray",
        "d7c1af35a1b4781bbe79e03dd94b7c13",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int8[]            data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::Int8MultiArray_",
        &[
            0xf2, 0x19, 0x98, 0xd4, 0xb4, 0x92, 0xab, 0xd6, 0x33, 0x30, 0x76, 0x5d, 0x75, 0xd5,
            0x83, 0x12, 0x38, 0xd4, 0x00, 0x74, 0x03, 0x86, 0xf6, 0x51, 0xf1, 0x3a, 0x87, 0x2a,
            0x4d, 0x21, 0x88, 0xdb,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MultiArrayDimension {
        pub r#label: ::std::string::String,
        pub r#size: u32,
        pub r#stride: u32,
    }
    ::roslibrust::impl_ros_message_type!(
        MultiArrayDimension,
        "std_msgs/MultiArrayDimension",
        "4cd0c83a8683deae40ecdac60e53bfa8",
        r####"string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::MultiArrayDimension_",
        &[
            0x5e, 0x77, 0x3a, 0x60, 0xa4, 0xc7, 0xfc, 0x8a, 0x54, 0x98, 0x5f, 0x30, 0x7c, 0x78,
            0x37, 0xaa, 0x29, 0x94, 0x25, 0x2a, 0x12, 0x6c, 0x30, 0x19, 0x57, 0xa2, 0x4e, 0x31,
            0x28, 0x2c, 0x9c, 0xbe,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MultiArrayLayout {
        pub r#dim: ::std::vec::Vec<self::MultiArrayDimension>,
        pub r#data_offset: u32,
    }
    ::roslibrust::impl_ros_message_type!(
        MultiArrayLayout,
        "std_msgs/MultiArrayLayout",
        "0fed2a11c13e11c5571b4e2a995a91a3",
        r####"# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::MultiArrayLayout_",
        &[
            0x4c, 0x66, 0xe6, 0xf7, 0x8e, 0x74, 0x0a, 0xc1, 0x03, 0xa9, 0x4c, 0xf6, 0x32, 0x59,
            0xf9, 0x68, 0xe4, 0x8c, 0x61, 0x7e, 0x76, 0x99, 0xe8, 0x29, 0xb6, 0x3c, 0x21, 0xa5,
            0xcb, 0x50, 0xda, 0xc6,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct String {
        pub r#data: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        String,
        "std_msgs/String",
        "992ce8a1687cec8c8bd883ec73ca41d1",
        r####"string data"####,
        "std_msgs::msg::dds_::String_",
        &[
            0xdf, 0x66, 0x8c, 0x74, 0x04, 0x82, 0xbb, 0xd4, 0x8f, 0xb3, 0x9d, 0x76, 0xa7, 0x0d,
            0xfd, 0x4b, 0xd5, 0x9d, 0xb1, 0x28, 0x80, 0x21, 0x74, 0x35, 0x03, 0x25, 0x9e, 0x94,
            0x8f, 0x6b, 0x1a, 0x18,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Time {
        pub r#data: ::roslibrust::codegen::integral_types::Time,
    }
    ::roslibrust::impl_ros_message_type!(
        Time,
        "std_msgs/Time",
        "cd7166c74c552c311fbcc2fe5a7bc289",
        r####"time data"####,
        "std_msgs::msg::dds_::Time_",
        &[
            0xe5, 0x77, 0xc4, 0x61, 0x30, 0x6c, 0xfd, 0x1c, 0xbf, 0x5b, 0xbf, 0x5c, 0x87, 0x4b,
            0x03, 0x8b, 0x7f, 0x3d, 0xcd, 0x6f, 0x16, 0xa1, 0x9b, 0x7f, 0xe6, 0x69, 0x63, 0xc2,
            0x3d, 0x47, 0x84, 0xa0,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt16 {
        pub r#data: u16,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt16,
        "std_msgs/UInt16",
        "1df79edf208b629fe6b81923a544552d",
        r####"uint16 data"####,
        "std_msgs::msg::dds_::UInt16_",
        &[
            0x08, 0xa4, 0x06, 0xe4, 0xb0, 0x22, 0xbc, 0x22, 0xe9, 0x07, 0xf9, 0x85, 0xd6, 0xa9,
            0xe9, 0xdd, 0x1d, 0x4f, 0xbe, 0xca, 0xe5, 0x73, 0x54, 0x9c, 0xf4, 0x93, 0x50, 0x11,
            0x3e, 0x77, 0x57, 0xb1,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u16>,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt16MultiArray,
        "std_msgs/UInt16MultiArray",
        "52f264f1c973c4b73790d384c6cb4484",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint16[]            data        # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::UInt16MultiArray_",
        &[
            0x94, 0xfe, 0x73, 0x42, 0x8e, 0xc6, 0x3b, 0xae, 0xcc, 0x77, 0x4f, 0x8f, 0xb8, 0x24,
            0x06, 0x12, 0x3e, 0x92, 0x91, 0xcf, 0x72, 0x8f, 0x1b, 0x7c, 0x91, 0xca, 0xf5, 0x33,
            0x51, 0x29, 0x49, 0x2b,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt32 {
        pub r#data: u32,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt32,
        "std_msgs/UInt32",
        "304a39449588c7f8ce2df6e8001c5fce",
        r####"uint32 data"####,
        "std_msgs::msg::dds_::UInt32_",
        &[
            0xa5, 0xc8, 0x74, 0x82, 0x9b, 0x75, 0x2b, 0xc5, 0xfa, 0x19, 0x00, 0x24, 0xb0, 0xad,
            0x76, 0xf5, 0x78, 0xcc, 0x27, 0x82, 0x71, 0xe8, 0x55, 0xc7, 0xd0, 0x2a, 0x81, 0x8b,
            0x35, 0x16, 0xfb, 0x4a,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u32>,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt32MultiArray,
        "std_msgs/UInt32MultiArray",
        "4d6a180abc9be191b96a7eda6c8a233d",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint32[]          data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::UInt32MultiArray_",
        &[
            0x6c, 0x25, 0x77, 0xc7, 0xad, 0x3c, 0xbd, 0xcc, 0x21, 0x64, 0xa4, 0x1c, 0x12, 0xf1,
            0xd5, 0xad, 0x31, 0x4e, 0xa3, 0x20, 0xf3, 0xfb, 0x1e, 0xe4, 0x7e, 0x78, 0x01, 0x9f,
            0xe1, 0x6b, 0xb5, 0xb0,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt64 {
        pub r#data: u64,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt64,
        "std_msgs/UInt64",
        "1b2a79973e8bf53d7b53acb71299cb57",
        r####"uint64 data"####,
        "std_msgs::msg::dds_::UInt64_",
        &[
            0xfb, 0xdc, 0x52, 0x01, 0x8f, 0xc1, 0x37, 0x55, 0xdc, 0xe1, 0x80, 0x24, 0xd1, 0xa6,
            0x71, 0xc8, 0x56, 0xaa, 0x8b, 0x4a, 0xaf, 0x63, 0xad, 0xfb, 0x09, 0x5b, 0x60, 0x8f,
            0x98, 0xe8, 0xc9, 0x43,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u64>,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt64MultiArray,
        "std_msgs/UInt64MultiArray",
        "6088f127afb1d6c72927aa1247e945af",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint64[]          data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::UInt64MultiArray_",
        &[
            0xfc, 0x1c, 0x68, 0x5c, 0x2f, 0x76, 0xbd, 0xc6, 0x98, 0x3d, 0xa0, 0x25, 0xcb, 0x25,
            0xd2, 0xdb, 0x5f, 0xb5, 0x15, 0x7b, 0x05, 0x9e, 0x30, 0x0f, 0x6d, 0x95, 0x7d, 0x86,
            0xf9, 0x81, 0xb3, 0x66,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt8 {
        pub r#data: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt8,
        "std_msgs/UInt8",
        "7c8164229e7d2c17eb95e9231617fdee",
        r####"uint8 data"####,
        "std_msgs::msg::dds_::UInt8_",
        &[
            0x61, 0x38, 0xbd, 0x83, 0xd8, 0xc3, 0x56, 0x9c, 0xb8, 0x0a, 0x66, 0x7d, 0xb0, 0x3c,
            0xfc, 0x16, 0x29, 0xf5, 0x29, 0xfe, 0xe7, 0x9d, 0x94, 0x4c, 0x39, 0xc3, 0x4e, 0x35,
            0x2e, 0x72, 0xf0, 0x10,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct UInt8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    ::roslibrust::impl_ros_message_type!(
        UInt8MultiArray,
        "std_msgs/UInt8MultiArray",
        "82373f1612381bb6ee473b5cd6f5d89c",
        r####"# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

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
================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"####,
        "std_msgs::msg::dds_::UInt8MultiArray_",
        &[
            0x56, 0x87, 0xe8, 0x61, 0xb8, 0xd3, 0x07, 0xa5, 0xe4, 0x8b, 0x75, 0x15, 0x46, 0x7a,
            0xe7, 0xa5, 0xfc, 0x2d, 0xaf, 0x80, 0x5b, 0xd0, 0xce, 0x6d, 0x8e, 0x9e, 0x60, 0x4b,
            0xad, 0xe9, 0xf3, 0x85,
        ],
    );
}
#[allow(unused_imports)]
pub mod std_srvs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct EmptyRequest {}
    ::roslibrust::impl_ros_message_type!(
        EmptyRequest,
        "std_srvs/EmptyRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "std_srvs::msg::dds_::EmptyRequest_",
        &[
            0xf6, 0xeb, 0x4f, 0x4a, 0x22, 0xd9, 0x55, 0x5a, 0xc7, 0xd9, 0xae, 0x4c, 0x28, 0x3c,
            0xf4, 0xa0, 0x88, 0x71, 0x59, 0xc4, 0xbb, 0xa7, 0x5e, 0x95, 0xec, 0xaf, 0xe6, 0x23,
            0x4d, 0x30, 0x15, 0x32,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct EmptyResponse {}
    ::roslibrust::impl_ros_message_type!(
        EmptyResponse,
        "std_srvs/EmptyResponse",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "std_srvs::msg::dds_::EmptyResponse_",
        &[
            0x81, 0x3f, 0x72, 0x8b, 0xa1, 0x9e, 0x08, 0x94, 0xd3, 0xfd, 0x51, 0xc3, 0x0f, 0x19,
            0x27, 0xae, 0x1f, 0xe9, 0xfe, 0xca, 0xe5, 0x77, 0x67, 0x08, 0x0a, 0xcf, 0xf3, 0x5f,
            0x7c, 0xbc, 0xc5, 0xf8,
        ],
    );
    #[allow(dead_code)]
    pub struct Empty {}
    impl ::roslibrust::RosServiceType for Empty {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Empty";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const ROS2_HASH: &'static [u8; 32] = &[
            0x58, 0x88, 0x39, 0x9d, 0xed, 0xec, 0x5c, 0xcc, 0x85, 0xea, 0x64, 0x51, 0x94, 0x9f,
            0xd2, 0xc9, 0xf9, 0x7b, 0xfd, 0xf9, 0x63, 0xf9, 0xa5, 0x88, 0x82, 0x16, 0x39, 0xfc,
            0xd3, 0x1b, 0x5d, 0x19,
        ];
        const ROS2_TYPE_NAME: &'static str = "std_srvs::srv::dds_::Empty_";
        type Request = EmptyRequest;
        type Response = EmptyResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetBoolRequest {
        pub r#data: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        SetBoolRequest,
        "std_srvs/SetBoolRequest",
        "8b94c1b53db61fb6aed406028ad6332a",
        r####"bool data # e.g. for hardware enabling / disabling"####,
        "std_srvs::msg::dds_::SetBoolRequest_",
        &[
            0xa9, 0x30, 0xae, 0x8d, 0x8d, 0x84, 0x84, 0x04, 0xac, 0x06, 0x9e, 0x26, 0xec, 0xf6,
            0xe6, 0x52, 0x82, 0x75, 0xa6, 0xa9, 0xc0, 0xe8, 0x74, 0x43, 0x1e, 0xf4, 0xc1, 0xc3,
            0x0a, 0x04, 0x85, 0x97,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct SetBoolResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        SetBoolResponse,
        "std_srvs/SetBoolResponse",
        "937c9679a518e3a18d831e57125ea522",
        r####"bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages"####,
        "std_srvs::msg::dds_::SetBoolResponse_",
        &[
            0x1f, 0xd1, 0x97, 0x84, 0xa1, 0xaa, 0x5e, 0x9d, 0x9f, 0x43, 0x29, 0xc8, 0xaf, 0x61,
            0xc6, 0xf1, 0x48, 0x1f, 0x3a, 0x45, 0x18, 0xc7, 0x80, 0x8b, 0x8b, 0x6b, 0x4f, 0x0d,
            0x8a, 0xc5, 0x70, 0x71,
        ],
    );
    #[allow(dead_code)]
    pub struct SetBool {}
    impl ::roslibrust::RosServiceType for SetBool {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/SetBool";
        const MD5SUM: &'static str = "09fb03525b03e7ea1fd3992bafd87e16";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xab, 0xe9, 0xe4, 0xbb, 0x6b, 0x41, 0xb4, 0x0e, 0x67, 0x89, 0x71, 0x2c, 0x00, 0xec,
            0x88, 0x71, 0x92, 0x3e, 0x08, 0x9a, 0xf3, 0xf6, 0x67, 0xa7, 0x99, 0x92, 0xa4, 0x28,
            0xcf, 0xf2, 0xda, 0x0a,
        ];
        const ROS2_TYPE_NAME: &'static str = "std_srvs::srv::dds_::SetBool_";
        type Request = SetBoolRequest;
        type Response = SetBoolResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TriggerRequest {}
    ::roslibrust::impl_ros_message_type!(
        TriggerRequest,
        "std_srvs/TriggerRequest",
        "d41d8cd98f00b204e9800998ecf8427e",
        r####""####,
        "std_srvs::msg::dds_::TriggerRequest_",
        &[
            0x3c, 0xa0, 0x6c, 0x57, 0x64, 0x5a, 0x34, 0x31, 0x19, 0x2d, 0xe5, 0xb5, 0x69, 0x09,
            0xbd, 0x20, 0x45, 0xdf, 0x4d, 0x53, 0x70, 0x06, 0xcb, 0x03, 0x96, 0x24, 0xbb, 0xdf,
            0x8f, 0x51, 0xf0, 0xaf,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct TriggerResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        TriggerResponse,
        "std_srvs/TriggerResponse",
        "937c9679a518e3a18d831e57125ea522",
        r####"bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages"####,
        "std_srvs::msg::dds_::TriggerResponse_",
        &[
            0x23, 0xa1, 0xb8, 0x10, 0xa6, 0x91, 0x7a, 0x50, 0x87, 0xbe, 0x33, 0x0a, 0xbe, 0x53,
            0xe8, 0xc7, 0x0b, 0x48, 0xbb, 0x8f, 0x70, 0x7e, 0x17, 0x56, 0xf5, 0x1f, 0xab, 0x4c,
            0xa9, 0xbb, 0xdf, 0xd9,
        ],
    );
    #[allow(dead_code)]
    pub struct Trigger {}
    impl ::roslibrust::RosServiceType for Trigger {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Trigger";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xee, 0xff, 0x2c, 0xd6, 0xfa, 0x5a, 0xd9, 0xd2, 0x7c, 0xdf, 0x4d, 0xec, 0x64, 0x81,
            0x83, 0x17, 0x83, 0x9b, 0x62, 0xf2, 0x12, 0xa9, 0x1e, 0x6b, 0x53, 0x04, 0xb6, 0x34,
            0xb2, 0x06, 0x2c, 0x5f,
        ];
        const ROS2_TYPE_NAME: &'static str = "std_srvs::srv::dds_::Trigger_";
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
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct DisparityImage {
        pub r#header: std_msgs::Header,
        pub r#image: sensor_msgs::Image,
        pub r#f: f32,
        pub r#T: f32,
        pub r#valid_window: sensor_msgs::RegionOfInterest,
        pub r#min_disparity: f32,
        pub r#max_disparity: f32,
        pub r#delta_d: f32,
    }
    ::roslibrust::impl_ros_message_type!(
        DisparityImage,
        "stereo_msgs/DisparityImage",
        "04a177815f75271039fa21f16acad8c9",
        r####"# Separate header for compatibility with current TimeSynchronizer.
# Likely to be removed in a later release, use image.header instead.
Header header

# Floating point disparity image. The disparities are pre-adjusted for any
# x-offset between the principal points of the two cameras (in the case
# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
sensor_msgs/Image image

# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
float32 f # Focal length, pixels
float32 T # Baseline, world units

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
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "stereo_msgs::msg::dds_::DisparityImage_",
        &[
            0xcb, 0xb7, 0xed, 0xc2, 0x9a, 0x4f, 0x58, 0x22, 0x7e, 0xb3, 0xe3, 0x7c, 0x0d, 0xe5,
            0xc9, 0xed, 0xfa, 0xe9, 0xbe, 0x79, 0x0e, 0x8d, 0xa4, 0x31, 0x59, 0x65, 0x39, 0xc5,
            0xb4, 0x06, 0x82, 0xa1,
        ],
    );
}
#[allow(unused_imports)]
pub mod test_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ADSBVehicle {
        pub r#header: std_msgs::Header,
        pub r#ICAO_address: u32,
        pub r#callsign: ::std::string::String,
        pub r#latitude: f64,
        pub r#longitude: f64,
        pub r#altitude: f32,
        pub r#heading: f32,
        pub r#hor_velocity: f32,
        pub r#ver_velocity: f32,
        pub r#altitude_type: u8,
        pub r#emitter_type: u8,
        pub r#tslc: ::roslibrust::codegen::integral_types::Duration,
        pub r#flags: u16,
        pub r#squawk: u16,
    }
    ::roslibrust::impl_ros_message_type!(
        ADSBVehicle,
        "test_msgs/ADSBVehicle",
        "d532685113a66fcc6ba0e6363ace0244",
        r####"# The location and information of an ADSB vehicle
#
# https://mavlink.io/en/messages/common.html#ADSB_VEHICLE

# [[[cog:
# from pymavlink.dialects.v20 import common
#
# def decl_enum(ename, pfx='', bsz=8):
#     enum = sorted(common.enums[ename].items())
#     enum.pop() # remove ENUM_END
#
#     cog.outl("# " + ename)
#     for k, e in enum:
#         sn = e.name[len(ename) + 1:]
#         l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
#         if e.description:
#             l += ' ' * (40 - len(l)) + ' # ' + e.description
#         cog.outl(l)
#
# decl_enum('ADSB_ALTITUDE_TYPE', 'ALT_')
# decl_enum('ADSB_EMITTER_TYPE', 'EMITTER_')
# decl_enum('ADSB_FLAGS', 'FLAG_', 16)
# ]]]
# ADSB_ALTITUDE_TYPE
uint8 ALT_PRESSURE_QNH = 0               # Altitude reported from a Baro source using QNH reference
uint8 ALT_GEOMETRIC = 1                  # Altitude reported from a GNSS source
# ADSB_EMITTER_TYPE
uint8 EMITTER_NO_INFO = 0
uint8 EMITTER_LIGHT = 1
uint8 EMITTER_SMALL = 2
uint8 EMITTER_LARGE = 3
uint8 EMITTER_HIGH_VORTEX_LARGE = 4
uint8 EMITTER_HEAVY = 5
uint8 EMITTER_HIGHLY_MANUV = 6
uint8 EMITTER_ROTOCRAFT = 7
uint8 EMITTER_UNASSIGNED = 8
uint8 EMITTER_GLIDER = 9
uint8 EMITTER_LIGHTER_AIR = 10
uint8 EMITTER_PARACHUTE = 11
uint8 EMITTER_ULTRA_LIGHT = 12
uint8 EMITTER_UNASSIGNED2 = 13
uint8 EMITTER_UAV = 14
uint8 EMITTER_SPACE = 15
uint8 EMITTER_UNASSGINED3 = 16
uint8 EMITTER_EMERGENCY_SURFACE = 17
uint8 EMITTER_SERVICE_SURFACE = 18
uint8 EMITTER_POINT_OBSTACLE = 19
# ADSB_FLAGS
uint16 FLAG_VALID_COORDS = 1
uint16 FLAG_VALID_ALTITUDE = 2
uint16 FLAG_VALID_HEADING = 4
uint16 FLAG_VALID_VELOCITY = 8
uint16 FLAG_VALID_CALLSIGN = 16
uint16 FLAG_VALID_SQUAWK = 32
uint16 FLAG_SIMULATED = 64
uint16 FLAG_VERTICAL_VELOCITY_VALID = 128
uint16 FLAG_BARO_VALID = 256
uint16 FLAG_SOURCE_UAT = 32768
# [[[end]]] (checksum: a34f2a081739921b6e3e443ed0516d8d)

std_msgs/Header header

uint32 ICAO_address
string callsign

float64 latitude
float64 longitude
float32 altitude 	# AMSL

float32 heading		# deg [0..360)
float32 hor_velocity	# m/s
float32 ver_velocity	# m/s

uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
uint8 emitter_type	# Type from ADSB_EMITTER_TYPE enum

duration tslc		# Duration from last communication, seconds [0..255]
uint16 flags		# ADSB_FLAGS bit field
uint16 squawk		# Squawk code
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "test_msgs::msg::dds_::ADSBVehicle_",
        &[
            0x66, 0x96, 0x84, 0xdc, 0xfd, 0x6e, 0x0d, 0x34, 0x7a, 0x61, 0xc9, 0xd8, 0x84, 0x8d,
            0x21, 0x9f, 0x58, 0xc8, 0xd9, 0x63, 0x12, 0xe6, 0x3e, 0x26, 0xbf, 0x2e, 0x49, 0x40,
            0xb9, 0xa7, 0x4d, 0xcc,
        ],
    );
    #[allow(unused)]
    impl ADSBVehicle {
        pub const r#ALT_PRESSURE_QNH: u8 = 0u8;
        pub const r#ALT_GEOMETRIC: u8 = 1u8;
        pub const r#EMITTER_NO_INFO: u8 = 0u8;
        pub const r#EMITTER_LIGHT: u8 = 1u8;
        pub const r#EMITTER_SMALL: u8 = 2u8;
        pub const r#EMITTER_LARGE: u8 = 3u8;
        pub const r#EMITTER_HIGH_VORTEX_LARGE: u8 = 4u8;
        pub const r#EMITTER_HEAVY: u8 = 5u8;
        pub const r#EMITTER_HIGHLY_MANUV: u8 = 6u8;
        pub const r#EMITTER_ROTOCRAFT: u8 = 7u8;
        pub const r#EMITTER_UNASSIGNED: u8 = 8u8;
        pub const r#EMITTER_GLIDER: u8 = 9u8;
        pub const r#EMITTER_LIGHTER_AIR: u8 = 10u8;
        pub const r#EMITTER_PARACHUTE: u8 = 11u8;
        pub const r#EMITTER_ULTRA_LIGHT: u8 = 12u8;
        pub const r#EMITTER_UNASSIGNED2: u8 = 13u8;
        pub const r#EMITTER_UAV: u8 = 14u8;
        pub const r#EMITTER_SPACE: u8 = 15u8;
        pub const r#EMITTER_UNASSGINED3: u8 = 16u8;
        pub const r#EMITTER_EMERGENCY_SURFACE: u8 = 17u8;
        pub const r#EMITTER_SERVICE_SURFACE: u8 = 18u8;
        pub const r#EMITTER_POINT_OBSTACLE: u8 = 19u8;
        pub const r#FLAG_VALID_COORDS: u16 = 1u16;
        pub const r#FLAG_VALID_ALTITUDE: u16 = 2u16;
        pub const r#FLAG_VALID_HEADING: u16 = 4u16;
        pub const r#FLAG_VALID_VELOCITY: u16 = 8u16;
        pub const r#FLAG_VALID_CALLSIGN: u16 = 16u16;
        pub const r#FLAG_VALID_SQUAWK: u16 = 32u16;
        pub const r#FLAG_SIMULATED: u16 = 64u16;
        pub const r#FLAG_VERTICAL_VELOCITY_VALID: u16 = 128u16;
        pub const r#FLAG_BARO_VALID: u16 = 256u16;
        pub const r#FLAG_SOURCE_UAT: u16 = 32768u16;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct ByteArrays {
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#dynamic_uint8: ::std::vec::Vec<u8>,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#dynamic_char: ::std::vec::Vec<u8>,
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#fixed_uint8: [u8; 4],
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#fixed_char: [u8; 4],
    }
    ::roslibrust::impl_ros_message_type!(
        ByteArrays,
        "test_msgs/ByteArrays",
        "72025d44f3736b53cfdcaacc6e70171b",
        r####"uint8[] dynamic_uint8
char[] dynamic_char
uint8[4] fixed_uint8
char[4] fixed_char"####,
        "test_msgs::msg::dds_::ByteArrays_",
        &[
            0xa5, 0x45, 0x22, 0xcc, 0x1a, 0xee, 0xd9, 0xbc, 0xf3, 0x9b, 0xe8, 0xa2, 0x6b, 0x79,
            0x10, 0x90, 0x57, 0xd2, 0x6c, 0x78, 0xfe, 0x58, 0xdd, 0x72, 0x0f, 0x92, 0x34, 0xf6,
            0xf8, 0xe8, 0x21, 0x48,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Constants {}
    ::roslibrust::impl_ros_message_type!(
        Constants,
        "test_msgs/Constants",
        "027df5f26b72c57b1e40902038ca3eec",
        r####"string TEST_STR="/topic"
string TEST_STR_2 = '/topic_2'
# Apparently unquoted strings are also valid?
# Pulled from https://github.com/ros/bond_core/blob/kinetic-devel/bond/msg/Constants.msg
string DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout
float32 TEST_FLOAT=0 # testing"####,
        "test_msgs::msg::dds_::Constants_",
        &[
            0x98, 0x3e, 0x7a, 0xe7, 0x09, 0x9d, 0x65, 0xb3, 0x03, 0x0c, 0x31, 0xc2, 0x89, 0x0b,
            0x0e, 0xc2, 0xb3, 0x32, 0x48, 0x4f, 0xae, 0x9f, 0x28, 0xc6, 0x8e, 0x01, 0x34, 0x0c,
            0x8f, 0x61, 0xed, 0xcf,
        ],
    );
    #[allow(unused)]
    impl Constants {
        pub const r#TEST_STR: &'static str = "\"/topic\"";
        pub const r#TEST_STR_2: &'static str = "'/topic_2'";
        pub const r#DISABLE_HEARTBEAT_TIMEOUT_PARAM: &'static str =
            "/bond_disable_heartbeat_timeout";
        pub const r#TEST_FLOAT: f32 = 0f32;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Float64Stamped {
        pub r#header: std_msgs::Header,
        pub r#value: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        Float64Stamped,
        "test_msgs/Float64Stamped",
        "d053817de0764f9ee90dbc89c4cdd751",
        r####"Header header
float64 value
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "test_msgs::msg::dds_::Float64Stamped_",
        &[
            0x0e, 0xdf, 0xf6, 0x86, 0xff, 0x05, 0xf1, 0xc0, 0x98, 0xbd, 0xb0, 0x11, 0xd8, 0x79,
            0xdf, 0xd5, 0xf7, 0x7d, 0x15, 0xc2, 0x22, 0x72, 0x8e, 0x4f, 0x0e, 0x59, 0x1c, 0xee,
            0x48, 0x1e, 0x65, 0x0f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct LoggerLevel {
        pub r#level: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        LoggerLevel,
        "test_msgs/LoggerLevel",
        "097b0e938d0dd7788057f4cdc9013238",
        r####"string level"####,
        "test_msgs::msg::dds_::LoggerLevel_",
        &[
            0x53, 0xa4, 0xf7, 0xbd, 0xd7, 0x38, 0x29, 0x6e, 0x2b, 0xe5, 0x4a, 0xe5, 0xb3, 0xf6,
            0xd6, 0xb3, 0x2d, 0x6a, 0x48, 0xd1, 0xc2, 0x9c, 0x67, 0x3d, 0x09, 0xd9, 0x68, 0x8a,
            0xcd, 0xf1, 0x32, 0x6b,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Metric {
        pub r#name: ::std::string::String,
        pub r#time: f64,
        pub r#data: ::std::vec::Vec<self::MetricPair>,
    }
    ::roslibrust::impl_ros_message_type!(
        Metric,
        "test_msgs/Metric",
        "474be567370f515a7d5d3f3243aad369",
        r####"#Metric data type
#For logging a set of points, e.g. for a pie chart

string name
float64 time
MetricPair[] data
================================================================================
MSG: test_msgs/MetricPair
#Data type for storing the key/value pairs from the Metric.data map

string key
float64 value"####,
        "test_msgs::msg::dds_::Metric_",
        &[
            0x00, 0x15, 0x40, 0xcf, 0x89, 0x47, 0x6f, 0xa6, 0xe6, 0x57, 0x29, 0x4a, 0x2d, 0xf0,
            0x0c, 0x90, 0xdf, 0x89, 0xab, 0x8d, 0x56, 0x98, 0xfd, 0x48, 0xcd, 0x76, 0xf6, 0xc5,
            0x7c, 0xf0, 0x23, 0x3f,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MetricPair {
        pub r#key: ::std::string::String,
        pub r#value: f64,
    }
    ::roslibrust::impl_ros_message_type!(
        MetricPair,
        "test_msgs/MetricPair",
        "a681f679e1c39fbe570b7737e7cf183d",
        r####"#Data type for storing the key/value pairs from the Metric.data map

string key
float64 value"####,
        "test_msgs::msg::dds_::MetricPair_",
        &[
            0xee, 0x2b, 0x61, 0x2d, 0x4c, 0x4d, 0x79, 0x54, 0x7b, 0xb4, 0xbf, 0x82, 0x35, 0x57,
            0x10, 0xa0, 0x4c, 0x0e, 0x21, 0xac, 0x7f, 0x49, 0x51, 0x73, 0x37, 0x9d, 0xaf, 0x44,
            0xae, 0xaf, 0xa8, 0xf8,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct NodeInfo {
        pub r#node_name: ::std::string::String,
        pub r#pid: i64,
        pub r#status: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        NodeInfo,
        "test_msgs/NodeInfo",
        "7fab1acc377fd48898b00b7f3a897f47",
        r####"string node_name
int64 pid

# Node is created, but is not yet initialized.
uint8 STATUS_UNINITIALIZED=0
# Node is initialized, but not connected.
uint8 STATUS_DISCONNECTED=1
# Node is initialized, connected, and running successfully.
uint8 STATUS_RUNNING=2
# Node is initialized and connected, but has a run error.
uint8 STATUS_RUN_ERROR=3
# Node was running, and is now shutting down.
uint8 STATUS_SHUTTING_DOWN=4
# Node is stopped.
uint8 STATUS_SHUTDOWN=5
uint8 status"####,
        "test_msgs::msg::dds_::NodeInfo_",
        &[
            0xc6, 0x55, 0x3f, 0xfe, 0xe0, 0x2c, 0x4c, 0x6d, 0xe5, 0x8e, 0x64, 0x65, 0x8c, 0x77,
            0xfc, 0x8e, 0xca, 0xef, 0x28, 0x76, 0xe0, 0xc6, 0xce, 0xc0, 0x96, 0xce, 0x81, 0x1f,
            0x43, 0xa3, 0x63, 0x06,
        ],
    );
    #[allow(unused)]
    impl NodeInfo {
        pub const r#STATUS_UNINITIALIZED: u8 = 0u8;
        pub const r#STATUS_DISCONNECTED: u8 = 1u8;
        pub const r#STATUS_RUNNING: u8 = 2u8;
        pub const r#STATUS_RUN_ERROR: u8 = 3u8;
        pub const r#STATUS_SHUTTING_DOWN: u8 = 4u8;
        pub const r#STATUS_SHUTDOWN: u8 = 5u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AddTwoIntsRequest {
        pub r#a: i64,
        pub r#b: i64,
    }
    ::roslibrust::impl_ros_message_type!(
        AddTwoIntsRequest,
        "test_msgs/AddTwoIntsRequest",
        "36d09b846be0b371c5f190354dd3153e",
        r####"# AddTwoInts.srv
# --- for funsies
# From this ROS tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
int64 a
int64 b"####,
        "test_msgs::msg::dds_::AddTwoIntsRequest_",
        &[
            0xc1, 0x62, 0x8b, 0x66, 0xca, 0xf6, 0xd7, 0x8d, 0xb5, 0xb4, 0xec, 0x08, 0x3d, 0xc8,
            0xeb, 0xfa, 0x20, 0x89, 0x87, 0x37, 0xc1, 0xd6, 0x60, 0x7d, 0x58, 0x22, 0x3b, 0x30,
            0xf0, 0x8c, 0x5d, 0x86,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct AddTwoIntsResponse {
        pub r#sum: i64,
    }
    ::roslibrust::impl_ros_message_type!(
        AddTwoIntsResponse,
        "test_msgs/AddTwoIntsResponse",
        "b88405221c77b1878a3cbbfff53428d7",
        r####"# Overflow? What overflow?
int64 sum"####,
        "test_msgs::msg::dds_::AddTwoIntsResponse_",
        &[
            0xf9, 0x26, 0x43, 0x14, 0x05, 0xea, 0xa7, 0x42, 0x23, 0xc6, 0xb5, 0x5f, 0x24, 0xac,
            0xa5, 0xbf, 0xd9, 0x11, 0x82, 0x67, 0x23, 0x88, 0x90, 0x5f, 0x6d, 0x30, 0x65, 0x78,
            0xcd, 0xdb, 0xbe, 0x8b,
        ],
    );
    #[allow(dead_code)]
    pub struct AddTwoInts {}
    impl ::roslibrust::RosServiceType for AddTwoInts {
        const ROS_SERVICE_NAME: &'static str = "test_msgs/AddTwoInts";
        const MD5SUM: &'static str = "6a2e34150c00229791cc89ff309fff21";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xbd, 0xb2, 0x96, 0xe2, 0x0b, 0x04, 0x94, 0x26, 0x7f, 0x32, 0x65, 0xbb, 0x0a, 0x6a,
            0x8a, 0x7e, 0xc6, 0xdd, 0x57, 0xeb, 0x26, 0xf8, 0x81, 0x00, 0xfb, 0x71, 0xe1, 0xe9,
            0x04, 0xae, 0x52, 0x53,
        ];
        const ROS2_TYPE_NAME: &'static str = "test_msgs::srv::dds_::AddTwoInts_";
        type Request = AddTwoIntsRequest;
        type Response = AddTwoIntsResponse;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct RoundTripArrayRequest {
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#bytes: ::std::vec::Vec<u8>,
    }
    ::roslibrust::impl_ros_message_type!(
        RoundTripArrayRequest,
        "test_msgs/RoundTripArrayRequest",
        "d159f2bd8169d3b3339e6f1fce045c6d",
        r####"# Purpose of this array is send and receive a large payload 
uint8[] bytes"####,
        "test_msgs::msg::dds_::RoundTripArrayRequest_",
        &[
            0x7b, 0xa8, 0xb3, 0x5b, 0xee, 0xeb, 0x46, 0x34, 0x1b, 0x87, 0xf4, 0x4e, 0x55, 0x7f,
            0xcf, 0x0b, 0x33, 0xcf, 0xf4, 0xc6, 0xe9, 0x43, 0x43, 0x76, 0x09, 0xa5, 0x12, 0xb2,
            0xa1, 0x34, 0xf9, 0xbf,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct RoundTripArrayResponse {
        #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")]
        pub r#bytes: ::std::vec::Vec<u8>,
    }
    ::roslibrust::impl_ros_message_type!(
        RoundTripArrayResponse,
        "test_msgs/RoundTripArrayResponse",
        "d159f2bd8169d3b3339e6f1fce045c6d",
        r####"uint8[] bytes"####,
        "test_msgs::msg::dds_::RoundTripArrayResponse_",
        &[
            0x08, 0x9a, 0x46, 0xae, 0x31, 0x33, 0x02, 0x3d, 0xff, 0x33, 0xd0, 0x6d, 0x5d, 0xc0,
            0xed, 0x35, 0x75, 0x81, 0x89, 0x8b, 0x01, 0x7a, 0x83, 0x2e, 0xf6, 0x9f, 0x62, 0xa6,
            0x87, 0xe0, 0xab, 0xc3,
        ],
    );
    #[allow(dead_code)]
    pub struct RoundTripArray {}
    impl ::roslibrust::RosServiceType for RoundTripArray {
        const ROS_SERVICE_NAME: &'static str = "test_msgs/RoundTripArray";
        const MD5SUM: &'static str = "6a66b36cb6abf834a48739776dfbe789";
        const ROS2_HASH: &'static [u8; 32] = &[
            0xe2, 0xca, 0xf4, 0x41, 0xa0, 0xcc, 0xba, 0x48, 0x70, 0x9a, 0x5a, 0x8e, 0xb3, 0xf7,
            0x8b, 0x44, 0x64, 0xae, 0x52, 0x34, 0x1f, 0xb2, 0x28, 0x50, 0x05, 0xf0, 0xe6, 0x83,
            0x10, 0x5b, 0x63, 0x36,
        ];
        const ROS2_TYPE_NAME: &'static str = "test_msgs::srv::dds_::RoundTripArray_";
        type Request = RoundTripArrayRequest;
        type Response = RoundTripArrayResponse;
    }
}
#[allow(unused_imports)]
pub mod trajectory_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct JointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::JointTrajectoryPoint>,
    }
    ::roslibrust::impl_ros_message_type!(
        JointTrajectory,
        "trajectory_msgs/JointTrajectory",
        "65b4f94a94d1ed67169da35a02f33d3f",
        r####"Header header
string[] joint_names
JointTrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start"####,
        "trajectory_msgs::msg::dds_::JointTrajectory_",
        &[
            0x41, 0x17, 0xd5, 0x4e, 0x07, 0x22, 0x63, 0x32, 0x76, 0xad, 0x07, 0x0b, 0x95, 0x7d,
            0x8a, 0x2b, 0x86, 0xf4, 0x76, 0x34, 0x91, 0x2c, 0x76, 0xdc, 0x4b, 0x94, 0xa0, 0x10,
            0x1f, 0xe1, 0xd4, 0x12,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct JointTrajectoryPoint {
        pub r#positions: ::std::vec::Vec<f64>,
        pub r#velocities: ::std::vec::Vec<f64>,
        pub r#accelerations: ::std::vec::Vec<f64>,
        pub r#effort: ::std::vec::Vec<f64>,
        pub r#time_from_start: ::roslibrust::codegen::integral_types::Duration,
    }
    ::roslibrust::impl_ros_message_type!(
        JointTrajectoryPoint,
        "trajectory_msgs/JointTrajectoryPoint",
        "f3cd1e1c4d320c79d6985c904ae5dcd3",
        r####"# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start"####,
        "trajectory_msgs::msg::dds_::JointTrajectoryPoint_",
        &[
            0xa1, 0x68, 0x02, 0xe9, 0x94, 0xa8, 0x75, 0x30, 0xf6, 0x32, 0xe3, 0x70, 0xe4, 0x68,
            0xdf, 0x71, 0x33, 0x51, 0x59, 0x2b, 0x04, 0x1a, 0xa0, 0x1a, 0x71, 0x93, 0x30, 0x2e,
            0xce, 0x37, 0x5c, 0xbd,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MultiDOFJointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::MultiDOFJointTrajectoryPoint>,
    }
    ::roslibrust::impl_ros_message_type!(
        MultiDOFJointTrajectory,
        "trajectory_msgs/MultiDOFJointTrajectory",
        "ef145a45a5f47b77b7f5cdde4b16c942",
        r####"# The header is used to specify the coordinate frame and the reference time for the trajectory durations
Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as 
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint
# Each multi-dof joint can specify a transform (up to 6 DOF)
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint 
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint 
geometry_msgs/Twist[] accelerations

duration time_from_start
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "trajectory_msgs::msg::dds_::MultiDOFJointTrajectory_",
        &[
            0x81, 0x4a, 0x4d, 0x15, 0xcf, 0xaf, 0x70, 0x30, 0x4b, 0x12, 0xc4, 0xc3, 0xe1, 0x8b,
            0x35, 0x7f, 0xbf, 0x17, 0x98, 0x3f, 0xbd, 0x28, 0x3d, 0xb3, 0xb1, 0x6a, 0x33, 0x33,
            0xf2, 0x64, 0x73, 0xc3,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MultiDOFJointTrajectoryPoint {
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#velocities: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#accelerations: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#time_from_start: ::roslibrust::codegen::integral_types::Duration,
    }
    ::roslibrust::impl_ros_message_type!(
        MultiDOFJointTrajectoryPoint,
        "trajectory_msgs/MultiDOFJointTrajectoryPoint",
        "3ebe08d1abd5b65862d50e09430db776",
        r####"# Each multi-dof joint can specify a transform (up to 6 DOF)
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint 
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint 
geometry_msgs/Twist[] accelerations

duration time_from_start
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"####,
        "trajectory_msgs::msg::dds_::MultiDOFJointTrajectoryPoint_",
        &[
            0xb1, 0xb0, 0xe9, 0x3c, 0x63, 0xd8, 0x42, 0xfe, 0xe4, 0xe0, 0xc5, 0xdb, 0x34, 0xeb,
            0x5e, 0xb6, 0xee, 0xc9, 0xf7, 0xfc, 0xa8, 0x95, 0xea, 0x14, 0x76, 0xf7, 0x6f, 0x0f,
            0x97, 0xb0, 0xbd, 0xa8,
        ],
    );
}
#[allow(unused_imports)]
pub mod visualization_msgs {
    use super::actionlib_msgs;
    use super::builtin_interfaces;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::service_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
        pub r#lifetime: ::roslibrust::codegen::integral_types::Duration,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point>,
        pub r#outline_colors: ::std::vec::Vec<std_msgs::ColorRGBA>,
    }
    ::roslibrust::impl_ros_message_type!(
        ImageMarker,
        "visualization_msgs/ImageMarker",
        "1de93c67ec8858b831025a08fbf1b35c",
        r####"uint8 CIRCLE=0
uint8 LINE_STRIP=1
uint8 LINE_LIST=2
uint8 POLYGON=3
uint8 POINTS=4

uint8 ADD=0
uint8 REMOVE=1

Header header
string ns		# namespace, used with id to form a unique id
int32 id          	# unique id within the namespace
int32 type        	# CIRCLE/LINE_STRIP/etc.
int32 action      	# ADD/REMOVE
geometry_msgs/Point position # 2D, in pixel-coords
float32 scale	 	# the diameter for a circle, etc.
std_msgs/ColorRGBA outline_color
uint8 filled		# whether to fill in the shape with color
std_msgs/ColorRGBA fill_color # color [0.0-1.0]
duration lifetime       # How long the object should last before being automatically deleted.  0 means forever


geometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords
std_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.
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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "visualization_msgs::msg::dds_::ImageMarker_",
        &[
            0xb8, 0xc2, 0x74, 0x3e, 0x1b, 0xd6, 0xd4, 0xaf, 0x59, 0x7a, 0xd8, 0x38, 0xe5, 0x3b,
            0x1c, 0x1f, 0xca, 0xdc, 0xbc, 0x8a, 0x65, 0xa3, 0xa7, 0x66, 0x4f, 0x65, 0xf7, 0x75,
            0x28, 0x50, 0x6e, 0x1e,
        ],
    );
    #[allow(unused)]
    impl ImageMarker {
        pub const r#CIRCLE: u8 = 0u8;
        pub const r#LINE_STRIP: u8 = 1u8;
        pub const r#LINE_LIST: u8 = 2u8;
        pub const r#POLYGON: u8 = 3u8;
        pub const r#POINTS: u8 = 4u8;
        pub const r#ADD: u8 = 0u8;
        pub const r#REMOVE: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct InteractiveMarker {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
        pub r#description: ::std::string::String,
        pub r#scale: f32,
        pub r#menu_entries: ::std::vec::Vec<self::MenuEntry>,
        pub r#controls: ::std::vec::Vec<self::InteractiveMarkerControl>,
    }
    ::roslibrust::impl_ros_message_type!(
        InteractiveMarker,
        "visualization_msgs/InteractiveMarker",
        "dd86d22909d5a3364b384492e35c10af",
        r####"# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
Header header

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.

# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.

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
uint8 command_type"####,
        "visualization_msgs::msg::dds_::InteractiveMarker_",
        &[
            0xa8, 0x0c, 0x4a, 0x6f, 0xec, 0x77, 0x5f, 0x06, 0x96, 0xb3, 0x72, 0xa9, 0x22, 0xf1,
            0x94, 0xe5, 0xd6, 0x0b, 0x1b, 0xae, 0x60, 0xcb, 0xd7, 0xd5, 0xee, 0x84, 0x23, 0x44,
            0x76, 0xd8, 0x2f, 0x69,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
    ::roslibrust::impl_ros_message_type!(
        InteractiveMarkerControl,
        "visualization_msgs/InteractiveMarkerControl",
        "b3c81e785788195d1840b86c28da1aac",
        r####"# Represents a control that is to be displayed together with an interactive marker

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "visualization_msgs::msg::dds_::InteractiveMarkerControl_",
        &[
            0x34, 0x7e, 0x3c, 0xdc, 0xf6, 0xae, 0x03, 0xaa, 0x41, 0x0c, 0xff, 0x6c, 0x47, 0xf0,
            0x3d, 0x9a, 0x33, 0x17, 0x90, 0x9a, 0xee, 0x36, 0x52, 0x57, 0x9a, 0x6b, 0xe8, 0x78,
            0x61, 0xaf, 0x63, 0x5b,
        ],
    );
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
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
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
    ::roslibrust::impl_ros_message_type!(
        InteractiveMarkerFeedback,
        "visualization_msgs/InteractiveMarkerFeedback",
        "ab0f1eee058667e28c19ff3ffc3f4b78",
        r####"# Time/frame info.
Header header

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "visualization_msgs::msg::dds_::InteractiveMarkerFeedback_",
        &[
            0xac, 0x50, 0x32, 0xab, 0x87, 0xdd, 0x8c, 0xde, 0x53, 0x4c, 0x71, 0xc3, 0x65, 0x71,
            0x0b, 0x64, 0x34, 0x82, 0x39, 0x8f, 0x01, 0x4d, 0xa0, 0x25, 0xf1, 0xd2, 0xe2, 0x8e,
            0xfc, 0xab, 0x43, 0xcc,
        ],
    );
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
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct InteractiveMarkerInit {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    ::roslibrust::impl_ros_message_type!(
        InteractiveMarkerInit,
        "visualization_msgs/InteractiveMarkerInit",
        "d5f2c5045a72456d228676ab91048734",
        r####"# Identifying string. Must be unique in the topic namespace
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
Header header

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.

# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.

# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.

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
uint8 command_type"####,
        "visualization_msgs::msg::dds_::InteractiveMarkerInit_",
        &[
            0xf7, 0x86, 0x4b, 0x7e, 0x14, 0x8e, 0xea, 0x84, 0x67, 0xe6, 0x14, 0x9d, 0xc3, 0xc6,
            0x4b, 0xde, 0x88, 0xd9, 0xd9, 0x28, 0x10, 0xa4, 0xc2, 0x8c, 0x21, 0xbf, 0x38, 0xb0,
            0x2a, 0x1d, 0x5d, 0x66,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct InteractiveMarkerPose {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
    }
    ::roslibrust::impl_ros_message_type!(
        InteractiveMarkerPose,
        "visualization_msgs/InteractiveMarkerPose",
        "a6e6833209a196a38d798dadb02c81f8",
        r####"# Time/frame info.
Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "visualization_msgs::msg::dds_::InteractiveMarkerPose_",
        &[
            0x2a, 0x61, 0x4e, 0x66, 0x63, 0xde, 0xbb, 0x34, 0xc2, 0x12, 0x3e, 0xa1, 0x47, 0xfe,
            0xba, 0x23, 0x05, 0x77, 0x24, 0x69, 0xc9, 0xe3, 0x4a, 0x40, 0x79, 0x07, 0xbe, 0xbc,
            0xa5, 0xfe, 0x09, 0xb2,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct InteractiveMarkerUpdate {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#type: u8,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
        pub r#poses: ::std::vec::Vec<self::InteractiveMarkerPose>,
        pub r#erases: ::std::vec::Vec<::std::string::String>,
    }
    ::roslibrust::impl_ros_message_type!(
        InteractiveMarkerUpdate,
        "visualization_msgs/InteractiveMarkerUpdate",
        "710d308d0a9276d65945e92dd30b3946",
        r####"# Identifying string. Must be unique in the topic namespace
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

#Note: No guarantees on the order of processing.
#      Contents must be kept consistent by sender.

#Markers to be added or updated
InteractiveMarker[] markers

#Poses of markers that should be moved
InteractiveMarkerPose[] poses

#Names of markers to be erased
string[] erases
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
Header header

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.

# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.

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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarkerPose
# Time/frame info.
Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.

# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.

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
uint8 command_type"####,
        "visualization_msgs::msg::dds_::InteractiveMarkerUpdate_",
        &[
            0xfa, 0x9a, 0x2f, 0x09, 0x93, 0x42, 0x79, 0xc3, 0x98, 0x4a, 0xb6, 0x4b, 0x33, 0xf7,
            0xf6, 0xb7, 0x56, 0xa1, 0x29, 0x61, 0xfe, 0x6c, 0x8e, 0x49, 0x28, 0xb2, 0x07, 0x33,
            0x32, 0x5e, 0x95, 0xb3,
        ],
    );
    #[allow(unused)]
    impl InteractiveMarkerUpdate {
        pub const r#KEEP_ALIVE: u8 = 0u8;
        pub const r#UPDATE: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct Marker {
        pub r#header: std_msgs::Header,
        pub r#ns: ::std::string::String,
        pub r#id: i32,
        pub r#type: i32,
        pub r#action: i32,
        pub r#pose: geometry_msgs::Pose,
        pub r#scale: geometry_msgs::Vector3,
        pub r#color: std_msgs::ColorRGBA,
        pub r#lifetime: ::roslibrust::codegen::integral_types::Duration,
        pub r#frame_locked: bool,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point>,
        pub r#colors: ::std::vec::Vec<std_msgs::ColorRGBA>,
        pub r#text: ::std::string::String,
        pub r#mesh_resource: ::std::string::String,
        pub r#mesh_use_embedded_materials: bool,
    }
    ::roslibrust::impl_ros_message_type!(
        Marker,
        "visualization_msgs/Marker",
        "4048c9de2a16f4ae8e0538085ebf1b97",
        r####"# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "visualization_msgs::msg::dds_::Marker_",
        &[
            0x83, 0x37, 0xc8, 0xdc, 0xa1, 0x51, 0xdd, 0xf4, 0xdd, 0xfb, 0x9c, 0x1d, 0x34, 0xfd,
            0x12, 0x4a, 0xa8, 0xec, 0x9e, 0x9d, 0x05, 0x4a, 0xf7, 0xc8, 0x11, 0x71, 0x69, 0x3f,
            0xbf, 0x53, 0x57, 0xa9,
        ],
    );
    #[allow(unused)]
    impl Marker {
        pub const r#ARROW: u8 = 0u8;
        pub const r#CUBE: u8 = 1u8;
        pub const r#SPHERE: u8 = 2u8;
        pub const r#CYLINDER: u8 = 3u8;
        pub const r#LINE_STRIP: u8 = 4u8;
        pub const r#LINE_LIST: u8 = 5u8;
        pub const r#CUBE_LIST: u8 = 6u8;
        pub const r#SPHERE_LIST: u8 = 7u8;
        pub const r#POINTS: u8 = 8u8;
        pub const r#TEXT_VIEW_FACING: u8 = 9u8;
        pub const r#MESH_RESOURCE: u8 = 10u8;
        pub const r#TRIANGLE_LIST: u8 = 11u8;
        pub const r#ADD: u8 = 0u8;
        pub const r#MODIFY: u8 = 0u8;
        pub const r#DELETE: u8 = 2u8;
        pub const r#DELETEALL: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MarkerArray {
        pub r#markers: ::std::vec::Vec<self::Marker>,
    }
    ::roslibrust::impl_ros_message_type!(
        MarkerArray,
        "visualization_msgs/MarkerArray",
        "d155b9ce5188fbaf89745847fd5882d7",
        r####"Marker[] markers
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
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

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id"####,
        "visualization_msgs::msg::dds_::MarkerArray_",
        &[
            0x51, 0x42, 0x5a, 0x13, 0xc8, 0x1e, 0xae, 0xe3, 0x0c, 0x4a, 0xe0, 0x41, 0x33, 0xdd,
            0xfe, 0x52, 0xdf, 0x25, 0xaf, 0x27, 0xee, 0x68, 0xe8, 0xbf, 0x12, 0x70, 0xf9, 0x81,
            0x51, 0x75, 0x09, 0xf3,
        ],
    );
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    #[derive(
        :: roslibrust :: codegen :: Deserialize,
        :: roslibrust :: codegen :: Serialize,
        :: roslibrust :: codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust::codegen::serde")]
    #[serde(deny_unknown_fields)]
    pub struct MenuEntry {
        pub r#id: u32,
        pub r#parent_id: u32,
        pub r#title: ::std::string::String,
        pub r#command: ::std::string::String,
        pub r#command_type: u8,
    }
    ::roslibrust::impl_ros_message_type!(
        MenuEntry,
        "visualization_msgs/MenuEntry",
        "b90ec63024573de83b57aa93eb39be2d",
        r####"# MenuEntry message.

# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.

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
uint8 command_type"####,
        "visualization_msgs::msg::dds_::MenuEntry_",
        &[
            0x22, 0x17, 0x0c, 0x38, 0x7c, 0x70, 0xfd, 0x42, 0x36, 0x23, 0x2e, 0xc9, 0x02, 0xde,
            0x86, 0x04, 0xe7, 0x2f, 0xf0, 0x27, 0x34, 0x2c, 0x7c, 0x0f, 0x28, 0xad, 0x9f, 0x68,
            0xc6, 0x4c, 0x51, 0xd6,
        ],
    );
    #[allow(unused)]
    impl MenuEntry {
        pub const r#FEEDBACK: u8 = 0u8;
        pub const r#ROSRUN: u8 = 1u8;
        pub const r#ROSLAUNCH: u8 = 2u8;
    }
}
use ::roslibrust::RosMessageType as _;
#[doc = r" Runtime lookup and codecs for all generated ROS message types."]
#[allow(dead_code)]
pub static MESSAGE_REGISTRY: ::roslibrust::MessageRegistry = ::roslibrust::MessageRegistry::new(&[
    actionlib_msgs::GoalID::DESCRIPTION,
    actionlib_msgs::GoalStatus::DESCRIPTION,
    actionlib_msgs::GoalStatusArray::DESCRIPTION,
    builtin_interfaces::Duration::DESCRIPTION,
    builtin_interfaces::Time::DESCRIPTION,
    diagnostic_msgs::DiagnosticArray::DESCRIPTION,
    diagnostic_msgs::DiagnosticStatus::DESCRIPTION,
    diagnostic_msgs::KeyValue::DESCRIPTION,
    geometry_msgs::Accel::DESCRIPTION,
    geometry_msgs::AccelStamped::DESCRIPTION,
    geometry_msgs::AccelWithCovariance::DESCRIPTION,
    geometry_msgs::AccelWithCovarianceStamped::DESCRIPTION,
    geometry_msgs::Inertia::DESCRIPTION,
    geometry_msgs::InertiaStamped::DESCRIPTION,
    geometry_msgs::Point::DESCRIPTION,
    geometry_msgs::Point32::DESCRIPTION,
    geometry_msgs::PointStamped::DESCRIPTION,
    geometry_msgs::Polygon::DESCRIPTION,
    geometry_msgs::PolygonStamped::DESCRIPTION,
    geometry_msgs::Pose::DESCRIPTION,
    geometry_msgs::Pose2D::DESCRIPTION,
    geometry_msgs::PoseArray::DESCRIPTION,
    geometry_msgs::PoseStamped::DESCRIPTION,
    geometry_msgs::PoseWithCovariance::DESCRIPTION,
    geometry_msgs::PoseWithCovarianceStamped::DESCRIPTION,
    geometry_msgs::Quaternion::DESCRIPTION,
    geometry_msgs::QuaternionStamped::DESCRIPTION,
    geometry_msgs::Transform::DESCRIPTION,
    geometry_msgs::TransformStamped::DESCRIPTION,
    geometry_msgs::Twist::DESCRIPTION,
    geometry_msgs::TwistStamped::DESCRIPTION,
    geometry_msgs::TwistWithCovariance::DESCRIPTION,
    geometry_msgs::TwistWithCovarianceStamped::DESCRIPTION,
    geometry_msgs::Vector3::DESCRIPTION,
    geometry_msgs::Vector3Stamped::DESCRIPTION,
    geometry_msgs::Wrench::DESCRIPTION,
    geometry_msgs::WrenchStamped::DESCRIPTION,
    nav_msgs::GetMapAction::DESCRIPTION,
    nav_msgs::GetMapActionFeedback::DESCRIPTION,
    nav_msgs::GetMapActionGoal::DESCRIPTION,
    nav_msgs::GetMapActionResult::DESCRIPTION,
    nav_msgs::GetMapFeedback::DESCRIPTION,
    nav_msgs::GetMapGoal::DESCRIPTION,
    nav_msgs::GetMapResult::DESCRIPTION,
    nav_msgs::GridCells::DESCRIPTION,
    nav_msgs::MapMetaData::DESCRIPTION,
    nav_msgs::OccupancyGrid::DESCRIPTION,
    nav_msgs::Odometry::DESCRIPTION,
    nav_msgs::Path::DESCRIPTION,
    rosapi::TypeDef::DESCRIPTION,
    rosgraph_msgs::Clock::DESCRIPTION,
    rosgraph_msgs::Log::DESCRIPTION,
    rosgraph_msgs::TopicStatistics::DESCRIPTION,
    sensor_msgs::BatteryState::DESCRIPTION,
    sensor_msgs::CameraInfo::DESCRIPTION,
    sensor_msgs::ChannelFloat32::DESCRIPTION,
    sensor_msgs::CompressedImage::DESCRIPTION,
    sensor_msgs::FluidPressure::DESCRIPTION,
    sensor_msgs::Illuminance::DESCRIPTION,
    sensor_msgs::Image::DESCRIPTION,
    sensor_msgs::Imu::DESCRIPTION,
    sensor_msgs::JointState::DESCRIPTION,
    sensor_msgs::Joy::DESCRIPTION,
    sensor_msgs::JoyFeedback::DESCRIPTION,
    sensor_msgs::JoyFeedbackArray::DESCRIPTION,
    sensor_msgs::LaserEcho::DESCRIPTION,
    sensor_msgs::LaserScan::DESCRIPTION,
    sensor_msgs::MagneticField::DESCRIPTION,
    sensor_msgs::MultiDOFJointState::DESCRIPTION,
    sensor_msgs::MultiEchoLaserScan::DESCRIPTION,
    sensor_msgs::NavSatFix::DESCRIPTION,
    sensor_msgs::NavSatStatus::DESCRIPTION,
    sensor_msgs::PointCloud::DESCRIPTION,
    sensor_msgs::PointCloud2::DESCRIPTION,
    sensor_msgs::PointField::DESCRIPTION,
    sensor_msgs::Range::DESCRIPTION,
    sensor_msgs::RegionOfInterest::DESCRIPTION,
    sensor_msgs::RelativeHumidity::DESCRIPTION,
    sensor_msgs::Temperature::DESCRIPTION,
    sensor_msgs::TimeReference::DESCRIPTION,
    service_msgs::ServiceEventInfo::DESCRIPTION,
    shape_msgs::Mesh::DESCRIPTION,
    shape_msgs::MeshTriangle::DESCRIPTION,
    shape_msgs::Plane::DESCRIPTION,
    shape_msgs::SolidPrimitive::DESCRIPTION,
    std_msgs::Bool::DESCRIPTION,
    std_msgs::Byte::DESCRIPTION,
    std_msgs::ByteMultiArray::DESCRIPTION,
    std_msgs::Char::DESCRIPTION,
    std_msgs::ColorRGBA::DESCRIPTION,
    std_msgs::Duration::DESCRIPTION,
    std_msgs::Empty::DESCRIPTION,
    std_msgs::Float32::DESCRIPTION,
    std_msgs::Float32MultiArray::DESCRIPTION,
    std_msgs::Float64::DESCRIPTION,
    std_msgs::Float64MultiArray::DESCRIPTION,
    std_msgs::Header::DESCRIPTION,
    std_msgs::Int16::DESCRIPTION,
    std_msgs::Int16MultiArray::DESCRIPTION,
    std_msgs::Int32::DESCRIPTION,
    std_msgs::Int32MultiArray::DESCRIPTION,
    std_msgs::Int64::DESCRIPTION,
    std_msgs::Int64MultiArray::DESCRIPTION,
    std_msgs::Int8::DESCRIPTION,
    std_msgs::Int8MultiArray::DESCRIPTION,
    std_msgs::MultiArrayDimension::DESCRIPTION,
    std_msgs::MultiArrayLayout::DESCRIPTION,
    std_msgs::String::DESCRIPTION,
    std_msgs::Time::DESCRIPTION,
    std_msgs::UInt16::DESCRIPTION,
    std_msgs::UInt16MultiArray::DESCRIPTION,
    std_msgs::UInt32::DESCRIPTION,
    std_msgs::UInt32MultiArray::DESCRIPTION,
    std_msgs::UInt64::DESCRIPTION,
    std_msgs::UInt64MultiArray::DESCRIPTION,
    std_msgs::UInt8::DESCRIPTION,
    std_msgs::UInt8MultiArray::DESCRIPTION,
    stereo_msgs::DisparityImage::DESCRIPTION,
    test_msgs::ADSBVehicle::DESCRIPTION,
    test_msgs::ByteArrays::DESCRIPTION,
    test_msgs::Constants::DESCRIPTION,
    test_msgs::Float64Stamped::DESCRIPTION,
    test_msgs::LoggerLevel::DESCRIPTION,
    test_msgs::Metric::DESCRIPTION,
    test_msgs::MetricPair::DESCRIPTION,
    test_msgs::NodeInfo::DESCRIPTION,
    trajectory_msgs::JointTrajectory::DESCRIPTION,
    trajectory_msgs::JointTrajectoryPoint::DESCRIPTION,
    trajectory_msgs::MultiDOFJointTrajectory::DESCRIPTION,
    trajectory_msgs::MultiDOFJointTrajectoryPoint::DESCRIPTION,
    visualization_msgs::ImageMarker::DESCRIPTION,
    visualization_msgs::InteractiveMarker::DESCRIPTION,
    visualization_msgs::InteractiveMarkerControl::DESCRIPTION,
    visualization_msgs::InteractiveMarkerFeedback::DESCRIPTION,
    visualization_msgs::InteractiveMarkerInit::DESCRIPTION,
    visualization_msgs::InteractiveMarkerPose::DESCRIPTION,
    visualization_msgs::InteractiveMarkerUpdate::DESCRIPTION,
    visualization_msgs::Marker::DESCRIPTION,
    visualization_msgs::MarkerArray::DESCRIPTION,
    visualization_msgs::MenuEntry::DESCRIPTION,
]);
