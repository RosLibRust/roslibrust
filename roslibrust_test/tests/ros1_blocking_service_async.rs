//! Test designed to show how to perform async actions in a blocking service callback

/// Tests require a running roscore
#[cfg(feature = "ros1_test")]
mod ros1_test {
    use log::*;
    use roslibrust::ros1::NodeHandle;
    use roslibrust_test::ros1::*;

    // This test covers a very specific case.
    // We previously weren't evaluating user's service functions inside a spawn_blocking call.
    // This would result in services sometime blocking the tokio runtime,
    // and prevented users from using Handle::block_on to perform async actions within their services.
    #[test_log::test(tokio::test)]
    async fn blocking_service_async() {
        let nh = NodeHandle::new("http://localhost:11311", "blocking_service_async")
            .await
            .unwrap();

        // Using a channel to represent something like another service that a service would like to call
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);

        let service = move |request: std_srvs::TriggerRequest| {
            info!("Got request: {request:?}");

            let handle = tokio::runtime::Handle::current();

            // Note: `move` not used here, future only borrows tx
            handle.block_on(async {
                let _ = tx.send(()).await;
            });

            Ok(std_srvs::TriggerResponse {
                success: true,
                message: "You triggered me!".to_string(),
            })
        };

        let _handle = nh
            .advertise_service::<std_srvs::Trigger, _>("/trigger", service)
            .await
            .unwrap();

        let client = nh
            .service_client::<std_srvs::Trigger>("/trigger")
            .await
            .unwrap();

        let response = client.call(&std_srvs::TriggerRequest {}).await.unwrap();

        rx.recv().await.unwrap();

        assert!(response.success);
        assert_eq!(response.message, "You triggered me!");
    }
}
