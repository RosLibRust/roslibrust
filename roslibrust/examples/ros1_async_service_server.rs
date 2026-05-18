#[cfg(feature = "ros1")]
roslibrust_codegen_macro::generate_ros_types!("assets/ros1_common_interfaces");

/// This example shows how to perform async actions correctly in a service callback.
///
/// This is the recommended way to do async actions in a service callback for the time being.
/// We hope to improve this API in the future with `async closures`.

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use log::*;
    use roslibrust::ros1::NodeHandle;

    // Create a logger to help make this example easier to debug
    env_logger::init();

    // Create a ros1 node and connect to a ros master
    let nh = NodeHandle::new("http://localhost:11311", "service_server_rs").await?;
    log::info!("Connected!");

    // Create an async channel to represent something like another service that a service would like to call
    let (tx, mut rx) = tokio::sync::mpsc::channel(1);

    let server_fn = move |request: std_srvs::SetBoolRequest| {
        log::info!("Got request to set bool: {request:?}");

        // We need an access to the tokio runtime to perform async actions.
        // Luckily, tokio provides a method to get the current runtime handle.
        let handle = tokio::runtime::Handle::current();

        // This handle then lets us use the `block_on` method to run an async block.
        // Note: the async block and future it generates only borrow tx!
        // If you need ownership of your async handles you need to .clone() them and use `async move {}`
        handle.block_on(async {
            // In here we can now perform async actions, like pushing our request into the channel
            let _ = tx.send(request.data).await;
        });

        Ok(std_srvs::SetBoolResponse {
            success: true,
            message: "You set my bool!".to_string(),
        })
    };

    // Start our service running!
    let _handle = nh
        .advertise_service::<std_srvs::SetBool, _>("~/my_set_bool", server_fn)
        .await?;
    info!("Service has started");

    // Setup a task to kill this process when ctrl_c comes in:
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0);
    });

    // As long as _handle is kept alive our service will continue to run

    // For fun, we can also spawn a task to periodically call our service
    let service_client = nh
        .service_client::<std_srvs::SetBool>("~/my_set_bool")
        .await?;
    tokio::spawn(async move {
        let mut bool = false;
        loop {
            bool = !bool;
            service_client
                .call(&std_srvs::SetBoolRequest { data: bool })
                .await
                .unwrap();
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
        }
    });

    // We can also await getting values from our channel
    loop {
        let cur_bool = rx.recv().await.unwrap();
        info!("Current value of our bool out of channel: {cur_bool}");
    }
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
