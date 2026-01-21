//! Integration tests for ROS1 service interoperability between roslibrust and rospy.
//!
//! These tests verify bidirectional service calls work correctly:
//! - roslibrust client calling rospy server
//! - rospy client calling roslibrust server
//!
//! Requirements:
//! - roscore must be running on localhost:11311
//! - ROS1 noetic with rospy must be available
//!
//! Run with: cargo test --package roslibrust_ros1 --test ros1_rospy_service_interop --features ros1_test -- --test-threads=1

#[cfg(feature = "ros1_test")]
mod tests {
    use log::*;
    use roslibrust_ros1::NodeHandle;
    use roslibrust_test::ros1::test_msgs;
    use std::io::{BufRead, BufReader};
    use std::process::{Child, Command, Stdio};
    use std::time::Duration;

    /// Helper to get the path to the test_helpers directory
    fn test_helpers_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("test_helpers")
    }

    /// Guard that kills a process when dropped
    struct ProcessGuard(Child);

    impl Drop for ProcessGuard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }

    /// Spawn a rospy service server and wait for it to be ready
    fn spawn_rospy_server(service_name: &str, node_name: &str) -> Result<ProcessGuard, String> {
        let script_path = test_helpers_dir().join("rospy_add_two_ints_server.py");

        let mut child = Command::new("bash")
            .args([
                "-c",
                &format!(
                    "source /opt/ros/noetic/setup.bash && python3 {} {} {}",
                    script_path.display(),
                    service_name,
                    node_name
                ),
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("Failed to spawn rospy server: {}", e))?;

        // Wait for the "READY" signal from the server
        let stdout = child.stdout.take().ok_or("No stdout")?;
        let reader = BufReader::new(stdout);

        for line in reader.lines() {
            let line = line.map_err(|e| format!("Failed to read line: {}", e))?;
            debug!("rospy server output: {}", line);
            if line.starts_with("READY:") {
                info!("rospy server is ready at {}", service_name);
                break;
            }
        }

        Ok(ProcessGuard(child))
    }

    /// Call a service using rospy client and return the result
    /// This runs in a blocking task to avoid deadlocking the tokio runtime
    async fn call_rospy_client(service_name: &str, a: i64, b: i64) -> Result<i64, String> {
        let script_path = test_helpers_dir().join("rospy_add_two_ints_client.py");
        let service_name = service_name.to_string();

        // Use spawn_blocking to avoid deadlocking the tokio runtime
        // The rospy client subprocess blocks, and we need the tokio runtime
        // to continue polling the service server's TCP listener
        tokio::task::spawn_blocking(move || {
            let output = Command::new("bash")
                .args([
                    "-c",
                    &format!(
                        "source /opt/ros/noetic/setup.bash && python3 {} {} {} {}",
                        script_path.display(),
                        service_name,
                        a,
                        b
                    ),
                ])
                .output()
                .map_err(|e| format!("Failed to run rospy client: {}", e))?;

            let stdout = String::from_utf8_lossy(&output.stdout);
            let stderr = String::from_utf8_lossy(&output.stderr);
            debug!("rospy client stdout: {}", stdout);
            debug!("rospy client stderr: {}", stderr);

            // Parse the result from stdout
            for line in stdout.lines() {
                if let Some(result) = line.strip_prefix("RESULT:") {
                    return result
                        .parse()
                        .map_err(|e| format!("Failed to parse result: {}", e));
                }
                if let Some(error) = line.strip_prefix("ERROR:") {
                    return Err(format!("Service call failed: {}", error));
                }
            }

            Err(format!(
                "No result found in rospy client output. stdout: {}, stderr: {}",
                stdout, stderr
            ))
        })
        .await
        .map_err(|e| format!("Task join error: {}", e))?
    }

    /// Test: roslibrust client calling rospy server
    /// This tests that roslibrust can successfully call a service provided by rospy
    #[test_log::test(tokio::test)]
    async fn test_roslibrust_client_rospy_server() {
        const SERVICE_NAME: &str = "/test_roslibrust_client_rospy_server/add_two_ints";

        // Start the rospy server
        let _server_guard = spawn_rospy_server(SERVICE_NAME, "rospy_server_for_roslibrust_client")
            .expect("Failed to start rospy server");

        // Give the server a moment to fully register with rosmaster
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Create roslibrust node and call the service
        let nh = NodeHandle::new("http://localhost:11311", "/test_roslibrust_client")
            .await
            .expect("Failed to create NodeHandle");

        let client = nh
            .service_client::<test_msgs::AddTwoInts>(SERVICE_NAME)
            .await
            .expect("Failed to create service client");

        // Test multiple calls with different values
        let test_cases = [(5, 3, 8), (100, 200, 300), (-10, 15, 5), (0, 0, 0)];

        for (a, b, expected_sum) in test_cases {
            info!("Testing: {} + {} = {}", a, b, expected_sum);
            let response = client
                .call(&test_msgs::AddTwoIntsRequest { a, b })
                .await
                .expect("Service call failed");

            assert_eq!(
                response.sum, expected_sum,
                "Expected {} + {} = {}, got {}",
                a, b, expected_sum, response.sum
            );
            info!("✓ {} + {} = {} (correct)", a, b, response.sum);
        }
    }

    /// Test: rospy client calling roslibrust server
    /// This tests that rospy can successfully call a service provided by roslibrust
    #[test_log::test(tokio::test)]
    async fn test_rospy_client_roslibrust_server() {
        const SERVICE_NAME: &str = "/test_rospy_client_roslibrust_server/add_two_ints";

        // Create roslibrust node and advertise the service
        let nh = NodeHandle::new("http://localhost:11311", "/test_roslibrust_server")
            .await
            .expect("Failed to create NodeHandle");

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            info!(
                "roslibrust server got request: a={}, b={}",
                request.a, request.b
            );
            Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            })
        };

        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>(SERVICE_NAME, server_fn)
            .await
            .expect("Failed to advertise service");

        info!("roslibrust service server ready at {}", SERVICE_NAME);

        // Give the server time to register with rosmaster
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Test multiple calls using rospy client
        let test_cases = [(7, 8, 15), (1000, 2000, 3000), (-50, 25, -25), (42, 0, 42)];

        for (a, b, expected_sum) in test_cases {
            info!("Testing via rospy client: {} + {} = {}", a, b, expected_sum);
            let result = call_rospy_client(SERVICE_NAME, a, b)
                .await
                .expect("rospy client call failed");

            assert_eq!(
                result, expected_sum,
                "Expected {} + {} = {}, got {}",
                a, b, expected_sum, result
            );
            info!("✓ rospy client: {} + {} = {} (correct)", a, b, result);
        }
    }

    /// Test: Bidirectional - both roslibrust and rospy servers, with cross-calling
    /// This tests the full interoperability scenario
    #[test_log::test(tokio::test)]
    async fn test_bidirectional_service_calls() {
        const ROSPY_SERVICE: &str = "/test_bidirectional/rospy_add_two";
        const ROSLIBRUST_SERVICE: &str = "/test_bidirectional/roslibrust_add_two";

        // Start rospy server
        let _rospy_server_guard = spawn_rospy_server(ROSPY_SERVICE, "rospy_server_bidirectional")
            .expect("Failed to start rospy server");

        // Start roslibrust server
        let nh = NodeHandle::new("http://localhost:11311", "/test_bidirectional_node")
            .await
            .expect("Failed to create NodeHandle");

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            info!(
                "roslibrust bidirectional server got request: a={}, b={}",
                request.a, request.b
            );
            Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            })
        };

        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>(ROSLIBRUST_SERVICE, server_fn)
            .await
            .expect("Failed to advertise service");

        // Give both servers time to register
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Test 1: roslibrust client -> rospy server
        info!("Testing roslibrust client -> rospy server");
        let client = nh
            .service_client::<test_msgs::AddTwoInts>(ROSPY_SERVICE)
            .await
            .expect("Failed to create client for rospy service");

        let response = client
            .call(&test_msgs::AddTwoIntsRequest { a: 11, b: 22 })
            .await
            .expect("Call to rospy server failed");
        assert_eq!(response.sum, 33);
        info!("✓ roslibrust -> rospy: 11 + 22 = 33");

        // Test 2: rospy client -> roslibrust server
        info!("Testing rospy client -> roslibrust server");
        let result = call_rospy_client(ROSLIBRUST_SERVICE, 33, 44)
            .await
            .expect("rospy client call to roslibrust server failed");
        assert_eq!(result, 77);
        info!("✓ rospy -> roslibrust: 33 + 44 = 77");

        info!("✓ Bidirectional service tests passed!");
    }
}
