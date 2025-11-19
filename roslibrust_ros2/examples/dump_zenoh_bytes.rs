// Just a test bed file to start playing with ROS2 Zenoh interaction

/// Make a connection to the zenoh network running rmw_zenohd
/// Note: rmw_zenohd must be running for this to work
/// Note: rmw_zenohd does not support multicast scouting by default, and must be explicitly configured to connect to it!
async fn make_ros_zenoh() -> zenoh::Session {
    // Configure zenoh to explicitly connect to rmw_zenohd
    // See: https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 for router setup
    let mut config = zenoh::Config::default();
    // Default location where rmw_zenohd listens for connections
    config
        .insert_json5("connect/endpoints", "[ \"tcp/[::]:7447\" ]")
        .unwrap();
    let session = zenoh::open(config).await.unwrap();
    session
}

/// Tries to subscribe to /chatter with std_msgs::String
/// This works!
async fn attempt_subscribe() {
    let session = make_ros_zenoh().await;

    // Have to create a liveliness token for ourselves
    let id = session.zid();

    // Note: As far as I can tell we don't actually need to declare liveliness tokens for this to work.
    // I think those are just needed for our node to show up correctly in rmw_ tooling
    // Declare our node exists
    // Node declaration: @ros2_lv/<domain_id>/<session_id>/<node_id>/<node_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>
    // Example from ros2 topic echo: @ros2_lv/0/22363ddafe5dc19901b95b42cce0393b/0/0/NN/%/%/_ros2cli_1106
    let node_token = format!("@ros2_lv/0/{id}/0/0/NN/%/%/roslibrust_test_node");
    println!("Declaring node: {node_token}");
    let _ = session
        .liveliness()
        .declare_token(node_token)
        .await
        .unwrap();

    // Declare our subscription to /client_count
    // Subscription declaration: @ros2_lv/<domain_id>/<session_id>/<node_id>/<entity_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>/<mangled_qualified_name>/<type_name>/<type_hash>/<qos>
    // Example from ros2 topic echo: @ros2_lv/0/22363ddafe5dc19901b95b42cce0393b/0/4/MS/%/%/_ros2cli_1106/%chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,5:,:,:,,
    let sub_token = format!(
        "@ros2_lv/0/{id}/0/10/MS/%/%/roslibrust_test_node/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,10:,:,:,,"
    );
    println!("Declaring subscription: {sub_token}");
    let _ = session.liveliness().declare_token(sub_token).await.unwrap();

    println!("Creating subscriber...");
    // Format for topics and services: <domain_id>/<fully_qualified_name>/<type_name>/<type_hash>
    // Example from ros2 topic echo: 0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
    let sub = session
        .declare_subscriber("0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18")
        .await
        .unwrap();

    while let Ok(sample) = sub.recv_async().await {
        println!("Got sample: {:?}", sample);

        let bytes = sample.payload().to_bytes();

        // Example struct we expect to convert to/from std_msgs::String in ros
        #[derive(serde::Deserialize, serde::Serialize)]
        struct StdMsgString {
            data: String,
        }
        let deserialized = cdr::deserialize::<StdMsgString>(&bytes).unwrap();
        println!("Deserialized: {:?}", deserialized.data);
    }

    /*
    Example output I got from this, while running `ros2 topic pub /chatter std_msgs/msg/String 'data: Hello World'`:

    Got sample: Sample { key_expr: ke`0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18`, payload: ZBytes(ZBuf { slices: [[00, 01, 00, 00, 0c, 00, 00, 00, 48, 65, 6c, 6c, 6f, 20, 57, 6f, 72, 6c, 64, 00]] }), kind: Put, encoding: Encoding(Encoding { id: 0, schema: None }), timestamp: Some(7546407767727260256/189e0640713f6e4b9c06efca248a546b), qos: QoS { inner: QoS { priority: Data, congestion: Drop, express: false } }, attachment: Some(ZBytes(ZBuf { slices: [[04, 00, 00, 00, 00, 00, 00, 00, e3, e2, c2, cf, 00, 3e, 62, 18, 10, c6, 3a, 35, be, e9, 87, 06, 2d, 54, 0c, b3, f1, 64, 1c, 0e, f2]] })) }
     */
}

#[tokio::main]
async fn main() {
    env_logger::init();

    // dump_all_liveness().await;

    attempt_subscribe().await;
}
