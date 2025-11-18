//! This file is part the getting_started.md documentation in the book.
//! It is included here so it can better be automatically tested

// ANCHOR: getting_started_1
// Bring generated messages into scope:
include!(concat!(env!("OUT_DIR"), "/messages.rs"));

// Bring in traits we need from roslibrust
use roslibrust::traits::{Publish, Ros};

use std::sync::Arc;
use tokio::sync::Mutex;

// Writing a simple publisher behavior using roslibrust's generic traits
async fn pub_counter(ros: impl Ros, state: Arc<Mutex<u32>>) {
    // This will nicely control the rat our code runs at
    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
    // Create a publisher on our topic
    let publisher = ros
        .advertise::<std_msgs::UInt32>("/example_counter")
        .await
        .expect("Could not create publisher!");

    loop {
        // Wait for next tick of our interval timer
        interval.tick().await;

        // Lock our state and read the current value
        let cur_val = *state.lock().await;

        // Publish the current value
        publisher
            .publish(&std_msgs::UInt32 { data: cur_val })
            .await
            .expect("Failed to publish message!");

        // Increment our state
        *state.lock().await += 1;
    }
}

// This macro sets up a basic tokio runtime for us and lets our main function be `async`
#[tokio::main]
async fn main() {
    // Initialize a logger to help with debugging
    env_logger::init();

    // Create a rosbridge client we can use
    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090")
        .await
        .expect("Failed to connect to rosbridge!");

    // Create a shared state we can use to track our counter
    let publisher_state = Arc::new(Mutex::new(0));

    // Spawn a new tokio task to run our publisher:
    tokio::spawn(pub_counter(ros, publisher_state));

    // Wait for ctrl_c
    tokio::signal::ctrl_c().await.unwrap();
}
// ANCHOR_END: getting_started_1
