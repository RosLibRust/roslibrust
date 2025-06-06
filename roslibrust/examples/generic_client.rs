//! Purpose of this example is to show how the TopicProvider trait can be use
//! to create code that is generic of which communication backend it will use.

// Important to bring these traits into scope so we can use them

#[cfg(all(feature = "rosbridge", feature = "ros1"))]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/std_msgs");

#[cfg(all(feature = "rosbridge", feature = "ros1"))]
#[tokio::main]
async fn main() {
    use roslibrust::{Publish, Ros, Subscribe};
    env_logger::init();

    // TopicProvider cannot be an "Object Safe Trait" due to its generic parameters
    // This means we can't do:

    // Which specific TopicProvider you are going to use must be known at
    // compile time! We can use features to build multiple copies of our
    // executable with different backends. Or mix and match within a
    // single application. The critical part is to make TopicProvider a
    // generic type on you Node.

    struct MyNode<T: Ros> {
        ros: T,
        name: String,
    }

    // Basic example of a node that publishes and subscribes to itself
    impl<T: Ros> MyNode<T> {
        async fn run(self) {
            let publisher = self
                .ros
                .advertise::<std_msgs::String>("/chatter")
                .await
                .unwrap();

            let mut subscriber = self
                .ros
                .subscribe::<std_msgs::String>("/chatter")
                .await
                .unwrap();

            tokio::spawn(async move {
                loop {
                    let msg = subscriber.next().await.unwrap();
                    println!("Got message: {}", msg.data);
                }
            });

            loop {
                let msg = std_msgs::String {
                    data: format!("Hello world from {}", self.name),
                };
                publisher.publish(&msg).await.unwrap();
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            }
        }
    }

    // create a rosbridge handle and start node
    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090")
        .await
        .unwrap();
    let node = MyNode {
        ros,
        name: "rosbridge_node".to_string(),
    };
    tokio::spawn(async move { node.run().await });

    // create a ros1 handle and start node
    let ros = roslibrust::ros1::NodeHandle::new("http://localhost:11311", "/my_node")
        .await
        .unwrap();
    let node = MyNode {
        ros,
        name: "ros1_node".to_string(),
    };
    tokio::spawn(async move { node.run().await });

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        println!("sleeping");
    }

    // With this executable running
    // RUST_LOG=debug cargo run --features ros1,topic_provider --example generic_client
    // You should be able to run `rostopic echo /chatter` and see the two nodes print out their names.
    // Note: this will not run without rosbridge running
}

#[cfg(not(all(feature = "rosbridge", feature = "ros1")))]
fn main() {
    eprintln!(
        "This example does nothing without compiling with the feature 'rosbridge' and 'ros1'"
    );
}
