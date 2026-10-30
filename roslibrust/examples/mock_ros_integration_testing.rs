//! Examples and integration tests for [`MockRos`](roslibrust::mock::MockRos).
//!
//! This file demonstrates how to write generic ROS node logic and test it quickly with
//! deterministic simulated time — no ROS installation required.
//!
//! Run the demo (uses an in-memory mock backend):
//!
//! ```text
//! cargo run -p roslibrust --example mock_ros_integration_testing --features mock
//! ```
//!
//! Run the embedded tests (also uses `MockRos` only):
//!
//! ```text
//! cargo test -p roslibrust --example mock_ros_integration_testing --features mock
//! ```

#[cfg(feature = "mock")]
mod integration {
    use roslibrust_test::ros1::{std_msgs, std_srvs};
    use roslibrust::{Publish, Ros, Service, ServiceProvider, Subscribe};
    use std::sync::Arc;
    use tokio::sync::Mutex;

    /// A publisher that ticks once per second and publishes an incrementing counter.
    pub async fn pub_counter(ros: impl Ros, state: Arc<Mutex<u32>>) {
        let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
        let publisher = ros
            .advertise::<std_msgs::UInt32>("/example_counter")
            .await
            .expect("Could not create publisher");

        loop {
            interval.tick().await;
            let value = *state.lock().await;
            publisher
                .publish(&std_msgs::UInt32 { data: value })
                .await
                .expect("Failed to publish");
            *state.lock().await += 1;
        }
    }

    /// A subscriber that decrements shared state whenever a message arrives.
    pub async fn sub_counter(ros: impl Ros, state: Arc<Mutex<u32>>) {
        let mut subscriber = ros
            .subscribe::<std_msgs::UInt32>("/example_counter")
            .await
            .expect("Could not create subscriber");

        loop {
            let msg = subscriber.next().await.expect("Failed to receive message");
            *state.lock().await -= 1;
            log::debug!("sub_counter saw {}", msg.data);
        }
    }

    /// A more involved node: waits, calls a service, then publishes status strings.
    pub async fn arm_and_report(ros: impl Ros) -> roslibrust::Result<()> {
        let status_pub = ros.advertise::<std_msgs::String>("/mission/status").await?;

        status_pub
            .publish(&std_msgs::String {
                data: "arming".to_owned(),
            })
            .await?;
        tokio::time::sleep(std::time::Duration::from_millis(250)).await;

        let client = ros
            .service_client::<std_srvs::SetBool>("/mission/enable")
            .await?;
        let response = client
            .call(&std_srvs::SetBoolRequest { data: true })
            .await?;
        if !response.success {
            return Err(roslibrust::Error::ServerError(response.message));
        }

        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
        status_pub
            .publish(&std_msgs::String {
                data: "ready".to_owned(),
            })
            .await?;
        Ok(())
    }

    pub async fn run_demo() {
        env_logger::init();

        let ros = roslibrust::mock::MockRos::new();
        let publisher_state = Arc::new(Mutex::new(0u32));
        let subscriber_state = Arc::new(Mutex::new(10u32));

        tokio::spawn(pub_counter(ros.clone(), publisher_state.clone()));
        tokio::spawn(sub_counter(ros.clone(), subscriber_state.clone()));

        ros.advertise_service::<std_srvs::SetBool, _>("/mission/enable", |req| {
            Ok(std_srvs::SetBoolResponse {
                success: req.data,
                message: String::new(),
            })
        })
        .await
        .expect("service setup");

        arm_and_report(ros)
            .await
            .expect("mission should complete against MockRos");

        log::info!(
            "Demo finished (publisher_state={}, subscriber_state={})",
            *publisher_state.lock().await,
            *subscriber_state.lock().await
        );
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use roslibrust::TopicProvider;
        use std::time::Instant;

        fn assert_fast_realtime(elapsed: std::time::Duration) {
            assert!(
                elapsed < std::time::Duration::from_millis(500),
                "expected sub-second wall time with paused tokio clock, got {elapsed:?}"
            );
        }

        #[tokio::test]
        async fn counter_pub_sub_runs_instantly_with_paused_time() {
            let tick = Instant::now();
            tokio::time::pause();

            let ros = roslibrust::mock::MockRos::new();
            let publisher_state = Arc::new(Mutex::new(0u32));
            let subscriber_state = Arc::new(Mutex::new(10u32));

            tokio::spawn(pub_counter(ros.clone(), publisher_state.clone()));
            tokio::spawn(sub_counter(ros, subscriber_state.clone()));

            tokio::time::sleep(std::time::Duration::from_secs(10)).await;

            assert_eq!(*publisher_state.lock().await, 10);
            assert_eq!(*subscriber_state.lock().await, 0);

            let elapsed = tick.elapsed();
            println!("counter integration test wall time: {elapsed:?}");
            assert_fast_realtime(elapsed);
        }

        #[tokio::test(start_paused = true)]
        async fn mission_node_uses_service_and_status_topic() {
            let tick = Instant::now();
            let ros = roslibrust::mock::MockRos::new();

            ros.advertise_service::<std_srvs::SetBool, _>("/mission/enable", |req| {
                Ok(std_srvs::SetBoolResponse {
                    success: req.data,
                    message: String::new(),
                })
            })
            .await
            .unwrap();

            let mut status_sub = ros
                .subscribe::<std_msgs::String>("/mission/status")
                .await
                .unwrap();

            tokio::spawn(async move {
                arm_and_report(ros).await.unwrap();
            });

            assert_eq!(status_sub.next().await.unwrap().data, "arming".to_owned());
            assert_eq!(status_sub.next().await.unwrap().data, "ready".to_owned());

            let elapsed = tick.elapsed();
            println!("mission integration test wall time: {elapsed:?}");
            assert_fast_realtime(elapsed);
        }
    }
}

#[cfg(feature = "mock")]
#[tokio::main]
async fn main() {
    integration::run_demo().await;
}

#[cfg(not(feature = "mock"))]
fn main() {
    eprintln!("Build with --features mock to run this example.");
}
