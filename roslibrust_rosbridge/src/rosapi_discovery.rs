use roslibrust_common::{RosMessageType, RosServiceType};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct EmptyRequest {}

impl RosMessageType for EmptyRequest {
    const DESCRIPTION: roslibrust_common::MessageDescriptor =
        roslibrust_common::MessageDescriptor::new::<Self>("", "", "", "", &[0; 32]);
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TopicsResponse {
    pub topics: Vec<String>,
    pub types: Vec<String>,
}

impl RosMessageType for TopicsResponse {
    const DESCRIPTION: roslibrust_common::MessageDescriptor =
        roslibrust_common::MessageDescriptor::new::<Self>("", "", "", "", &[0; 32]);
}

pub struct Topics;

impl RosServiceType for Topics {
    const ROS_SERVICE_NAME: &'static str = "rosapi/Topics";
    type Request = EmptyRequest;
    type Response = TopicsResponse;
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ServicesResponse {
    pub services: Vec<String>,
}

impl RosMessageType for ServicesResponse {
    const DESCRIPTION: roslibrust_common::MessageDescriptor =
        roslibrust_common::MessageDescriptor::new::<Self>("", "", "", "", &[0; 32]);
}

pub struct Services;

impl RosServiceType for Services {
    const ROS_SERVICE_NAME: &'static str = "rosapi/Services";
    type Request = EmptyRequest;
    type Response = ServicesResponse;
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ServiceTypeRequest {
    pub service: String,
}

impl RosMessageType for ServiceTypeRequest {
    const DESCRIPTION: roslibrust_common::MessageDescriptor =
        roslibrust_common::MessageDescriptor::new::<Self>("", "", "", "", &[0; 32]);
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ServiceTypeResponse {
    #[serde(rename = "type")]
    pub type_name: String,
}

impl RosMessageType for ServiceTypeResponse {
    const DESCRIPTION: roslibrust_common::MessageDescriptor =
        roslibrust_common::MessageDescriptor::new::<Self>("", "", "", "", &[0; 32]);
}

pub struct ServiceType;

impl RosServiceType for ServiceType {
    const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceType";
    type Request = ServiceTypeRequest;
    type Response = ServiceTypeResponse;
}
