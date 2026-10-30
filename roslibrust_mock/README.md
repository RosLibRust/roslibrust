# RosLibRust Mock

In-memory implementation of RosLibRust's [`TopicProvider`](https://docs.rs/roslibrust_common/latest/roslibrust_common/trait.TopicProvider.html), [`ServiceProvider`](https://docs.rs/roslibrust_common/latest/roslibrust_common/trait.ServiceProvider.html), and [`GraphProvider`](https://docs.rs/roslibrust_common/latest/roslibrust_common/trait.GraphProvider.html) traits for unit and integration tests.

Enable it from the main crate with the `mock` feature:

```toml
[dev-dependencies]
roslibrust = { version = "0.26", features = ["mock"] }
```

## Why use MockRos?

- **No ROS install** — tests run anywhere `cargo test` runs.
- **Isolated state** — each `MockRos::new()` is independent, so tests can run in parallel without cross-talk.
- **Deterministic time** — combine with [`tokio::time::pause`](https://docs.rs/tokio/latest/tokio/time/fn.pause.html) or `#[tokio::test(start_paused = true)]` so code that `sleep`s for seconds finishes in microseconds.

## Quick pattern

Write node logic against `impl roslibrust::Ros`, then fixture tests with `MockRos`:

```rust
async fn my_node(ros: impl roslibrust::Ros) -> roslibrust::Result<()> {
    let pub_handle = ros.advertise::<std_msgs::String>("/out").await?;
    pub_handle.publish(&std_msgs::String { data: "hi".into() }).await?;
    Ok(())
}

#[tokio::test]
async fn test_my_node() {
    tokio::time::pause();
    let ros = roslibrust::mock::MockRos::new();
    let mut sub = ros.subscribe::<std_msgs::String>("/out").await.unwrap();
    tokio::spawn(my_node(ros));
    assert_eq!(sub.next().await.unwrap().unwrap().data, "hi");
}
```

## Examples

See the [`mock_ros_integration_testing`](../roslibrust/examples/mock_ros_integration_testing.rs) example in the `roslibrust` crate for:

- Generic publisher/subscriber nodes with simulated one-second intervals
- A multi-step node that calls a service and publishes status updates
- Tests that assert behavior and print wall-clock duration (typically well under one millisecond)

```text
cargo test -p roslibrust --example mock_ros_integration_testing --features mock
cargo run -p roslibrust --example mock_ros_integration_testing --features mock
```

The [extended getting started guide](https://roslibrust.github.io/roslibrust/extended_getting_started.html#writing-tests-for-our-node) in the project book covers the same ideas in a full crate layout.

## Crate layout

This crate is re-exported as [`roslibrust::mock`](https://docs.rs/roslibrust/latest/roslibrust/mock/index.html) when the `mock` feature is enabled. Depending on `roslibrust_mock` directly is supported but usually unnecessary.
