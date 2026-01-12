// Important to bring the traits we're using into scope
// In this case we need access to the .publish() function on our returned Publisher type from the advertise call
use roslibrust::Publish;

// We're using the macro to generate our types
// We provide the macro with additional search paths beyond the ROS_PACKAGE_PATH
// The paths are assumed to be relative to the crate (or workspace) root
roslibrust::find_and_generate_ros_messages!(
    "assets/ros1_test_msgs",
    "assets/ros1_common_interfaces"
);

// Writing a simple behavior that uses the generic traits from roslibrust
// and the generated types from the macro above.
async fn pub_counter(ros: impl roslibrust::Ros) {
    let publisher = ros
        .advertise::<std_msgs::Int16>("/example_counter")
        .await
        .unwrap();
    let mut counter = 0;
    loop {
        publisher
            .publish(&std_msgs::Int16 { data: counter })
            .await
            .unwrap();
        println!("Published {counter}");
        counter += 1;
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}

// Our actual "main" here doesn't do much, just shows the generate types
// are here and real.
#[tokio::main]
async fn main() {
    // Create a rosbridge client we can use
    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090")
        .await
        .unwrap();
    // Start our behavior while waiting for ctrl_c
    tokio::select! {
        _ = pub_counter(ros) => {}
        _ = tokio::signal::ctrl_c() => {}
    }
}

// Setup a test of our pub_counter behavior
#[cfg(test)]
mod test {
    use super::*;
    use roslibrust::{Subscribe, TopicProvider};

    #[tokio::test]
    async fn test_pub_counter() {
        // See: https://tokio.rs/tokio/topics/testing
        // This test doesn't take 1 second to run even thou it looks like it should!
        // Tokio simulates time in tests if you call pause()
        // This test takes 0.00s to run on a reasonable machine
        tokio::time::pause();
        let ros = roslibrust::mock::MockRos::new();

        // Subscribe to the topic we're publishing to
        let mut subscriber = ros
            .subscribe::<std_msgs::Int16>("/example_counter")
            .await
            .unwrap();

        // Start publishing in the background
        tokio::spawn(async move { pub_counter(ros).await });

        // Confirm we get the first message
        let msg = subscriber.next().await.unwrap();
        assert_eq!(msg.data, 0);

        // Confirm second message quickly times out
        let msg =
            tokio::time::timeout(tokio::time::Duration::from_millis(10), subscriber.next()).await;
        assert!(msg.is_err());

        // Wait a bit
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        // Now get second message
        let msg = subscriber.next().await.unwrap();
        assert_eq!(msg.data, 1);
    }
}
