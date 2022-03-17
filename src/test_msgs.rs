use serde::{Deserialize,Serialize};
use crate::RosMessageType;

#[derive(Deserialize, Serialize, Debug, Default, Clone)]
pub struct Float64Stamped {
    pub
    header: Header,
    pub
    value: f64,
}

impl RosMessageType for Float64Stamped {
    const ROS_TYPE_NAME: &'static str = "test_msgs/Float64Stamped";
}

impl Float64Stamped {
}

#[derive(Deserialize, Serialize, Debug, Default, Clone, PartialEq)]
pub struct Header {
    pub
    seq: u32,
    pub
    stamp: TimeI,
    pub
    frame_id: String,
}

impl RosMessageType for Header {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
}

impl Header {
}

#[derive(Deserialize, Serialize, Debug, Default, Clone)]
pub struct LoggerLevel {
    pub
    level: String,
}

impl RosMessageType for LoggerLevel {
    const ROS_TYPE_NAME: &'static str = "test_msgs/LoggerLevel";
}

impl LoggerLevel {
}

#[derive(Deserialize, Serialize, Debug, Default, Clone)]
pub struct Metric {
    pub
    name: String,
    pub
    time: f64,
    pub
    data: Vec<MetricPair>,
}

impl RosMessageType for Metric {
    const ROS_TYPE_NAME: &'static str = "test_msgs/Metric";
}

impl Metric {
}

#[derive(Deserialize, Serialize, Debug, Default, Clone)]
pub struct MetricPair {
    pub
    key: String,
    pub
    value: f64,
}

impl RosMessageType for MetricPair {
    const ROS_TYPE_NAME: &'static str = "test_msgs/MetricPair";
}

impl MetricPair {
}

#[derive(Deserialize, Serialize, Debug, Default, Clone)]
pub struct NodeInfo {
    pub
    node_name: String,
    pub
    pid: i64,
    pub
    status: u8,
}

impl RosMessageType for NodeInfo {
    const ROS_TYPE_NAME: &'static str = "test_msgs/NodeInfo";
}

impl NodeInfo {
    const STATUS_UNINITIALIZED: u8 = 0;
    const STATUS_DISCONNECTED: u8 = 1;
    const STATUS_RUNNING: u8 = 2;
    const STATUS_RUN_ERROR: u8 = 3;
    const STATUS_SHUTTING_DOWN: u8 = 4;
    const STATUS_SHUTDOWN: u8 = 5;
}

#[derive(Deserialize, Serialize, Debug, Default, Clone, PartialEq)]
pub struct TimeI {
    pub
    secs: u32,
    pub
    nsecs: u32,
}

impl RosMessageType for TimeI {
    const ROS_TYPE_NAME: &'static str = "std_msgs/TimeI";
}

impl TimeI {
}