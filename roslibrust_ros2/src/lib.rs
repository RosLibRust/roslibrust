use log::*;
use roslibrust_common::*;
use std::result::Result as StdResult;

use ros_z::{
    context::ZContext,
    entity::{TypeHash, TypeInfo},
    msg::{CdrCompatSerdes, ZService},
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

/// The publisher type returned by [TopicProvider::advertise] on [ZenohClient].
pub struct ZenohPublisher<T: RosMessageType> {
    publisher: ZPub<T, CdrCompatSerdes>,
}

impl<T: RosMessageType + serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static>
    Publish<T> for ZenohPublisher<T>
{
    async fn publish(&self, data: &T) -> Result<()> {
        self.publisher
            .async_publish(data)
            .await
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))
    }
}

/// The subscriber type returned by [TopicProvider::subscribe] on [ZenohClient].
pub struct ZenohSubscriber<T: RosMessageType> {
    subscriber: ZSub<T, zenoh::sample::Sample, CdrCompatSerdes>,
}

impl<T: RosMessageType + serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static>
    Subscribe<T> for ZenohSubscriber<T>
{
    async fn next(&mut self) -> Result<T> {
        self.subscriber
            .async_recv()
            .await
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
            .create_pub_impl::<MsgType>(topic.as_ref(), Some(ros_type_info::<MsgType>()))
            .with_serdes::<CdrCompatSerdes>()
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
            .create_sub_impl::<MsgType>(topic.as_ref(), Some(ros_type_info::<MsgType>()))
            .with_serdes::<CdrCompatSerdes>()
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
    type Request = T::Request;
    type Response = T::Response;
}

impl<T: RosServiceType> ServiceTypeInfo for Fake<T> {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(T::ROS2_TYPE_NAME, TypeHash::new(1, *T::ROS2_HASH))
    }
}

impl<T: RosServiceType> roslibrust_common::Service<T> for ZenohServiceClient<T> {
    async fn call(&self, request: &T::Request) -> Result<T::Response> {
        self.client
            .send_request(request)
            .await
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))?;

        self.client
            .take_response()
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

        // Local Fake<T> for advertise_service (same orphan workaround as service_client)
        struct LocalFake<T>(T);
        impl<T: RosServiceType> ZService for LocalFake<T> {
            type Request = T::Request;
            type Response = T::Response;
        }
        impl<T: RosServiceType> ServiceTypeInfo for LocalFake<T> {
            fn service_type_info() -> TypeInfo {
                TypeInfo::new(T::ROS2_TYPE_NAME, TypeHash::new(1, *T::ROS2_HASH))
            }
        }

        let mut svc = self
            .node
            .create_service::<LocalFake<SrvType>>(service.as_ref())
            .build()
            .map_err(|e| Error::Unexpected(anyhow::anyhow!(e)))?;

        let cancellation_token = tokio_util::sync::CancellationToken::new();
        let server = std::sync::Arc::new(server);
        let service_name = String::from(service);
        let ct_copy = cancellation_token.clone();

        tokio::spawn(async move {
            let body_future = async {
                loop {
                    let req = svc.take_request_async().await;
                    let (query, req) = match req {
                        Ok(req) => req,
                        Err(e) => {
                            error!("Failed to take request in service {service_name}: {e:?}");
                            continue;
                        }
                    };
                    debug!(
                        "Got request for service {service_name} with key {:?}",
                        query
                    );

                    let server_copy = server.clone();
                    let response = tokio::task::spawn_blocking(move || server_copy(req)).await;

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

                    let send_result = svc.send_response_async(&valid_response, &query).await;
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

#[cfg(test)]
mod tests {

    #[cfg(feature = "ros2_zenoh_test")]
    mod integration_tests {
        use crate::ZenohClient;
        use ros_z::context::ZContext;
        use roslibrust_common::traits::*;

        fn make_test_context() -> ZContext {
            use ros_z::context::ZContextBuilder;
            use ros_z::Builder;

            ZContextBuilder::default()
                .with_domain_id(0)
                .with_connect_endpoints(["tcp/[::]:7447"])
                .build()
                .unwrap()
        }

        #[ignore]
        #[tokio::test(flavor = "multi_thread")]
        async fn test_subscribe_basic() {
            let ctx = make_test_context();
            let client = ZenohClient::new(&ctx, "test_subscribe_basic_node")
                .await
                .unwrap();
            let mut subscriber = client
                .subscribe::<roslibrust_test::ros2::std_msgs::String>("/chatter")
                .await
                .unwrap();

            #[allow(clippy::zombie_processes)]
            let mut pub_cmd = std::process::Command::new("ros2")
                .arg("topic")
                .arg("pub")
                .arg("-t")
                .arg("10")
                .arg("/chatter")
                .arg("std_msgs/msg/String")
                .arg("data: Hello World")
                .spawn()
                .unwrap();

            tokio::time::timeout(tokio::time::Duration::from_secs(2), async {
                let msg = subscriber.next().await.unwrap();
                assert_eq!(msg.data, "Hello World");
            })
            .await
            .unwrap();

            pub_cmd.kill().unwrap();
        }

        #[tokio::test(flavor = "multi_thread")]
        async fn test_pubsub_basic() {
            let ctx = make_test_context();
            let client = ZenohClient::new(&ctx, "test_publish_basic_node")
                .await
                .unwrap();

            let publisher = client
                .advertise::<roslibrust_test::ros2::std_msgs::String>("/chatter")
                .await
                .unwrap();

            let mut subscriber = client
                .subscribe::<roslibrust_test::ros2::std_msgs::String>("/chatter")
                .await
                .unwrap();

            let msg = roslibrust_test::ros2::std_msgs::String {
                data: "Hello World".to_string(),
            };

            publisher.publish(&msg).await.unwrap();

            tokio::time::timeout(tokio::time::Duration::from_secs(2), async {
                let msg = subscriber.next().await.unwrap();
                assert_eq!(msg.data, "Hello World");
            })
            .await
            .expect("Failed to receive message within 2 seconds");
        }

        #[ignore]
        #[tokio::test(flavor = "multi_thread")]
        async fn test_service_server_callable() {
            let ctx = make_test_context();
            let client = ZenohClient::new(&ctx, "test_service_server_callable_node")
                .await
                .unwrap();

            let state = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
            let state_copy = state.clone();
            let server_fn = move |request: roslibrust_test::ros2::std_srvs::SetBoolRequest| {
                state_copy.store(request.data, std::sync::atomic::Ordering::SeqCst);
                Ok(roslibrust_test::ros2::std_srvs::SetBoolResponse {
                    message: "You set my bool!".to_string(),
                    success: request.data,
                })
            };

            let _service = client
                .advertise_service::<roslibrust_test::ros2::std_srvs::SetBool, _>(
                    "/test_service_server_callable_node/set_bool",
                    server_fn,
                )
                .await
                .unwrap();

            #[allow(clippy::zombie_processes)]
            let mut srv_call_cmd = std::process::Command::new("ros2")
                .arg("service")
                .arg("call")
                .arg("/test_service_server_callable_node/set_bool")
                .arg("std_srvs/srv/SetBool")
                .arg("data: true")
                .spawn()
                .unwrap();

            tokio::time::timeout(tokio::time::Duration::from_secs(2), async {
                while !state.load(std::sync::atomic::Ordering::SeqCst) {
                    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
                }
            })
            .await
            .expect("Bool should be set true within 2 seconds");

            srv_call_cmd.kill().unwrap()
        }

        #[ignore]
        #[tokio::test(flavor = "multi_thread")]
        async fn test_service_zenoh_to_zenoh() {
            let ctx = make_test_context();
            let node = ZenohClient::new(&ctx, "test_service_server_zenoh")
                .await
                .unwrap();

            let state = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
            let state_copy = state.clone();

            let server_fn = move |request: roslibrust_test::ros2::std_srvs::SetBoolRequest| {
                state_copy.store(request.data, std::sync::atomic::Ordering::SeqCst);
                Ok(roslibrust_test::ros2::std_srvs::SetBoolResponse {
                    message: "You set my bool!".to_string(),
                    success: request.data,
                })
            };

            let _service = node
                .advertise_service::<roslibrust_test::ros2::std_srvs::SetBool, _>(
                    "/test_service_zenoh_to_zenoh_set_bool",
                    server_fn,
                )
                .await
                .unwrap();

            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

            let response = node
                .call_service::<roslibrust_test::ros2::std_srvs::SetBool>(
                    "/test_service_zenoh_to_zenoh_set_bool",
                    roslibrust_test::ros2::std_srvs::SetBoolRequest { data: true },
                )
                .await
                .expect("Service call should succeed");

            assert!(response.success);
            assert_eq!(response.message, "You set my bool!");
            assert!(state.load(std::sync::atomic::Ordering::SeqCst));
        }
    }
}
