use log::*;
use roslibrust_common::*;
use std::result::Result as StdResult;

use ros_z::{
    context::ZContext,
    entity::{TypeHash, TypeInfo},
    msg::{SerdeCdrSerdes, ZMessage, ZService},
    pubsub::{ZPub, ZSub},
    ros_msg::ServiceTypeInfo,
    Builder,
};

/// re-export ros_z for consumers
pub use ros_z;

/// A "newtype" wrapper around ZNode so we can implement roslibrust's traits for it.
pub struct ZenohClient {
    node: ros_z::node::ZNode,
}

type RosZSerdes<T> = SerdeCdrSerdes<RosZMessage<T>>;

struct RosZMessage<T>(T);

impl<T: RosMessageType> ZMessage for RosZMessage<T> {
    type Serdes = RosZSerdes<T>;
}

impl<T: serde::Serialize> serde::Serialize for RosZMessage<T> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> StdResult<S::Ok, S::Error> {
        self.0.serialize(serializer)
    }
}

impl<'de, T: serde::Deserialize<'de>> serde::Deserialize<'de> for RosZMessage<T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> StdResult<Self, D::Error> {
        T::deserialize(deserializer).map(Self)
    }
}

/// The publisher type returned by [TopicProvider::advertise] on [ZenohClient].
pub struct ZenohPublisher<T: RosMessageType> {
    publisher: ZPub<RosZMessage<T>, RosZSerdes<T>>,
}

impl<T: RosMessageType> Publish<T> for ZenohPublisher<T> {
    async fn publish(&self, data: &T) -> Result<()> {
        let data = RosZMessage(data.clone());
        self.publisher
            .async_publish(&data)
            .await
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))
    }
}

/// The subscriber type returned by [TopicProvider::subscribe] on [ZenohClient].
pub struct ZenohSubscriber<T: RosMessageType> {
    subscriber: ZSub<RosZMessage<T>, zenoh::sample::Sample, RosZSerdes<T>>,
}

impl<T: RosMessageType> Subscribe<T> for ZenohSubscriber<T> {
    async fn next(&mut self) -> Result<T> {
        self.subscriber
            .async_recv()
            .await
            .map(|msg| msg.0)
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))
    }
}

impl ZenohClient {
    pub async fn new(
        ctx: &ZContext,
        name: impl AsRef<str>,
    ) -> StdResult<Self, Box<dyn std::error::Error + Send + Sync + 'static>> {
        let node = ctx.create_node(name.as_ref()).build()?;
        Ok(Self { node })
    }
}

/// Build a TypeInfo from a RosMessageType's type name and hash.
fn ros_type_info<T: RosMessageType>() -> TypeInfo {
    TypeInfo::new(T::ROS2_TYPE_NAME, TypeHash::new(1, *T::ROS2_HASH))
}

impl roslibrust_common::TopicProvider for ZenohClient {
    type Publisher<T: RosMessageType> = ZenohPublisher<T>;
    type Subscriber<T: RosMessageType> = ZenohSubscriber<T>;

    async fn advertise<MsgType: RosMessageType>(
        &self,
        topic: impl roslibrust_common::topic_name::ToGlobalTopicName + Send,
    ) -> Result<Self::Publisher<MsgType>> {
        let topic: roslibrust_common::GlobalTopicName = topic.to_global_name()?;
        let publisher = self
            .node
            .create_pub_impl::<RosZMessage<MsgType>>(
                topic.as_ref(),
                Some(ros_type_info::<MsgType>()),
            )
            .build()
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))?;

        Ok(ZenohPublisher { publisher })
    }

    async fn subscribe<MsgType: RosMessageType>(
        &self,
        topic: impl roslibrust_common::topic_name::ToGlobalTopicName + Send,
    ) -> Result<Self::Subscriber<MsgType>> {
        let topic: roslibrust_common::GlobalTopicName = topic.to_global_name()?;
        let sub = self
            .node
            .create_sub_impl::<RosZMessage<MsgType>>(
                topic.as_ref(),
                Some(ros_type_info::<MsgType>()),
            )
            .build()
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))?;

        Ok(ZenohSubscriber { subscriber: sub })
    }
}

pub struct ZenohServiceServer {
    cancellation_token: tokio_util::sync::CancellationToken,
}

impl Drop for ZenohServiceServer {
    fn drop(&mut self) {
        self.cancellation_token.cancel();
    }
}

pub struct ZenohServiceClient<T: RosServiceType> {
    client: ros_z::service::ZClient<Fake<T>>,
    _marker: std::marker::PhantomData<T>,
}

// Orphan-rule shim: ZService (ros_z) cannot be impl'd for T: RosServiceType (roslibrust_common)
// in a downstream crate. This unit struct bridges the two.
struct Fake<T>(std::marker::PhantomData<T>);

impl<T: RosServiceType> ZService for Fake<T> {
    type Request = RosZMessage<T::Request>;
    type Response = RosZMessage<T::Response>;
}

impl<T: RosServiceType> ServiceTypeInfo for Fake<T> {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(T::ROS2_TYPE_NAME, TypeHash::new(1, *T::ROS2_HASH))
    }
}

impl<T: RosServiceType> roslibrust_common::Service<T> for ZenohServiceClient<T> {
    async fn call(&self, request: &T::Request) -> Result<T::Response> {
        let request = RosZMessage(request.clone());
        self.client
            .call(&request)
            .await
            .map(|response| response.0)
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))
    }
}

impl roslibrust_common::ServiceProvider for ZenohClient {
    type ServiceClient<T: RosServiceType> = ZenohServiceClient<T>;
    type ServiceServer = ZenohServiceServer;

    async fn call_service<SrvType: RosServiceType>(
        &self,
        service: impl roslibrust_common::topic_name::ToGlobalTopicName + Send,
        request: SrvType::Request,
    ) -> Result<SrvType::Response> {
        let service: roslibrust_common::GlobalTopicName = service.to_global_name()?;
        let client = ZenohClient::service_client::<SrvType>(self, service.as_ref()).await?;
        client.call(&request).await
    }

    async fn service_client<SrvType: RosServiceType + 'static>(
        &self,
        service: impl roslibrust_common::topic_name::ToGlobalTopicName + Send,
    ) -> Result<Self::ServiceClient<SrvType>> {
        let service: roslibrust_common::GlobalTopicName = service.to_global_name()?;
        let client = self
            .node
            .create_client::<Fake<SrvType>>(service.as_ref())
            .build()
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))?;

        Ok(ZenohServiceClient {
            client,
            _marker: std::marker::PhantomData,
        })
    }

    async fn advertise_service<SrvType: RosServiceType + 'static, F: ServiceFn<SrvType>>(
        &self,
        service: impl roslibrust_common::topic_name::ToGlobalTopicName + Send,
        server: F,
    ) -> Result<Self::ServiceServer> {
        let service: roslibrust_common::GlobalTopicName = service.to_global_name()?;

        let mut svc = self
            .node
            .create_service::<Fake<SrvType>>(service.as_ref())
            .build()
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))?;

        let cancellation_token = tokio_util::sync::CancellationToken::new();
        let server = std::sync::Arc::new(server);
        let service_name = String::from(service);
        let ct_copy = cancellation_token.clone();

        tokio::spawn(async move {
            let body_future = async {
                loop {
                    let req = svc.async_take_request().await;
                    let req = match req {
                        Ok(req) => req,
                        Err(e) => {
                            error!("Failed to take request in service {service_name}: {e:?}");
                            continue;
                        }
                    };
                    debug!(
                        "Got request for service {service_name} with key {:?}",
                        req.id()
                    );

                    let (req, reply) = req.into_parts();
                    let server_copy = server.clone();
                    let response = tokio::task::spawn_blocking(move || server_copy(req.0)).await;

                    let valid_response = match response {
                        Ok(Ok(response)) => response,
                        Ok(Err(e)) => {
                            error!("Failed to handle request in service {service_name}: {e:?}");
                            continue;
                        }
                        Err(e) => {
                            error!("Failed to join task in service {service_name}: {e:?}");
                            continue;
                        }
                    };

                    let valid_response = RosZMessage(valid_response);
                    let send_result = reply.reply(&valid_response).await;
                    if let Err(e) = send_result {
                        error!("Failed to send response to service {service_name}: {e:?}");
                    }
                }
            };

            tokio::select! {
                _ = ct_copy.cancelled() => {}
                _ = body_future => {
                    error!("Service task for {service_name} exited unexpectedly");
                }
            }
        });

        Ok(ZenohServiceServer { cancellation_token })
    }
}
