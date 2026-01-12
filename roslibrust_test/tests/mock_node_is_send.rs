//! Test covers a specific problem we tripped over
//!
//! A moderately complex "node" that held various roslibrust types would end up
//! not being compatible with tokio::spawn.

use roslibrust::traits::*;
use roslibrust_test::ros1::*;

// Very basic node that holds a bunch of ros types generically
#[allow(unused)]
struct Node<T: Ros> {
    ros: T,
    publisher: T::Publisher<std_msgs::String>,
    subscriber: T::Subscriber<std_msgs::String>,
    service: T::ServiceServer,
    client: T::ServiceClient<std_srvs::SetBool>,
}

impl<T: Ros> Node<T> {
    // Basic node impl that constructs the types
    async fn new(ros: T) -> Self {
        let publisher = ros
            .advertise::<std_msgs::String>("/test_topic")
            .await
            .unwrap();
        let subscriber = ros
            .subscribe::<std_msgs::String>("/test_topic")
            .await
            .unwrap();
        let service = ros
            .advertise_service::<std_srvs::SetBool, _>("/test_service", |_| {
                Ok(std_srvs::SetBoolResponse {
                    success: true,
                    message: "You set my bool!".to_string(),
                })
            })
            .await
            .unwrap();
        let client = ros
            .service_client::<std_srvs::SetBool>("/test_service")
            .await
            .unwrap();
        Self {
            ros,
            publisher,
            subscriber,
            service,
            client,
        }
    }

    // Very basic usage of the node types to verify they are tokio compatible
    async fn run(mut self) {
        self.publisher
            .publish(&std_msgs::String {
                data: "Hello, world!".to_string(),
            })
            .await
            .unwrap();
        self.client
            .call(&std_srvs::SetBoolRequest { data: true })
            .await
            .unwrap();
        let msg = self.subscriber.next().await.unwrap();
        assert_eq!(msg.data, "Hello, world!");
    }
}

async fn test_node(ros: roslibrust::mock::MockRos) {
    let node = Node::new(ros).await;
    node.run().await;
}

#[tokio::test(flavor = "multi_thread")]
async fn test_mock_node_is_send() {
    let ros = roslibrust::mock::MockRos::new();
    // This previously would fail to compile with:
    /*
       error[E0308]: mismatched types
      --> roslibrust_test/tests/mock_node_is_send.rs:73:5
       |
    73 |     tokio::spawn(test_node(ros)).await.unwrap();
       |     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ one type is more general than the other
       |
       = note: expected enum `Result<_, Box<dyn StdError + Send + Sync>>`
                  found enum `Result<_, Box<(dyn StdError + Send + Sync + 'static)>>`
       */
    // This was fixed by switching the error type of our ServiceFn trait to use 'anyhow::Error'
    // Which solved some very nasty lifetime inference issues.
    // This test simply compiling is enough to really test it
    tokio::spawn(test_node(ros));
}
