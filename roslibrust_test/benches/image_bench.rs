use criterion::{criterion_group, criterion_main, Criterion};
use pprof::criterion::{Output, PProfProfiler};
use roslibrust::{Publish, Subscribe, TopicProvider};
use roslibrust_zenoh::ZenohClient;
use std::{
    hint::black_box,
    sync::{Arc, Mutex},
};

struct BenchContextRos1 {
    publisher: roslibrust::ros1::Publisher<roslibrust_test::ros1::sensor_msgs::Image>,
    subscriber: roslibrust::ros1::Subscriber<roslibrust_test::ros1::sensor_msgs::Image>,
    image: roslibrust_test::ros1::sensor_msgs::Image,
    // Need to keep alive
    _client: roslibrust::ros1::NodeHandle,
}

struct BenchContextZenoh {
    publisher: roslibrust_zenoh::ZenohPublisher<roslibrust_test::ros1::sensor_msgs::Image>,
    subscriber: roslibrust_zenoh::ZenohSubscriber<roslibrust_test::ros1::sensor_msgs::Image>,
    image: roslibrust_test::ros1::sensor_msgs::Image,
    // Need to keep alive - separate clients to avoid short-circuiting
    _publisher_client: ZenohClient,
    _subscriber_client: ZenohClient,
}

async fn setup_bench_context_ros1() -> BenchContextRos1 {
    let client = roslibrust::ros1::NodeHandle::new("http://localhost:11311", "image_bench_ros1")
        .await
        .unwrap();
    let publisher = client
        .advertise::<roslibrust_test::ros1::sensor_msgs::Image>("/image_bench_ros1", 1, false)
        .await
        .unwrap();
    let subscriber = client
        .subscribe::<roslibrust_test::ros1::sensor_msgs::Image>("/image_bench_ros1", 1)
        .await
        .unwrap();

    // Wait for pub / sub to establish connection
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    let image = roslibrust_test::ros1::sensor_msgs::Image {
        header: Default::default(),
        height: 1080,
        width: 1920,
        encoding: "rgb8".to_owned(),
        is_bigendian: 0,
        step: 1920 * 3,
        data: vec![0; 1920 * 1080 * 3],
    };

    BenchContextRos1 {
        publisher,
        subscriber,
        image,
        _client: client,
    }
}

async fn setup_bench_context_zenoh() -> BenchContextZenoh {
    // Create separate Zenoh sessions for publisher and subscriber to avoid short-circuiting
    // This better emulates real communication between different nodes
    let publisher_session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let publisher_client = ZenohClient::new(publisher_session);

    let subscriber_session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let subscriber_client = ZenohClient::new(subscriber_session);

    let publisher = publisher_client
        .advertise::<roslibrust_test::ros1::sensor_msgs::Image>("/image_bench_zenoh")
        .await
        .unwrap();
    let subscriber = subscriber_client
        .subscribe::<roslibrust_test::ros1::sensor_msgs::Image>("/image_bench_zenoh")
        .await
        .unwrap();

    // Wait for pub / sub to establish connection
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    let image = roslibrust_test::ros1::sensor_msgs::Image {
        header: Default::default(),
        height: 1080,
        width: 1920,
        encoding: "rgb8".to_owned(),
        is_bigendian: 0,
        step: 1920 * 3,
        data: vec![0; 1920 * 1080 * 3],
    };

    BenchContextZenoh {
        publisher,
        subscriber,
        image,
        _publisher_client: publisher_client,
        _subscriber_client: subscriber_client,
    }
}

async fn bench_iteration_ros1(context: &mut BenchContextRos1) {
    context.publisher.publish(&context.image).await.unwrap();
    let received_image = context.subscriber.next().await.unwrap().unwrap();
    black_box(received_image);
}

async fn bench_iteration_zenoh(context: &mut BenchContextZenoh) {
    context.publisher.publish(&context.image).await.unwrap();
    let received_image = context.subscriber.next().await.unwrap();
    black_box(received_image);
}

fn criterion_benchmark(c: &mut Criterion) {
    env_logger::init();

    // Create a tokio runtime so we can be async
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();

    // Setup ROS1 TCPROS context
    let context_ros1 = runtime.block_on(async {
        let context = setup_bench_context_ros1().await;
        Arc::new(Mutex::new(context))
    });

    c.bench_function("image_bench_ros1_tcpros", |b| {
        b.to_async(&runtime).iter(|| {
            #[allow(clippy::await_holding_lock)]
            async {
                let mut context = context_ros1.lock().unwrap();
                bench_iteration_ros1(&mut context).await
            }
        })
    });

    // Setup Zenoh context
    let context_zenoh = runtime.block_on(async {
        let context = setup_bench_context_zenoh().await;
        Arc::new(Mutex::new(context))
    });

    c.bench_function("image_bench_ros1_zenoh", |b| {
        b.to_async(&runtime).iter(|| {
            #[allow(clippy::await_holding_lock)]
            async {
                let mut context = context_zenoh.lock().unwrap();
                bench_iteration_zenoh(&mut context).await
            }
        })
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().with_profiler(PProfProfiler::new(1000, Output::Flamegraph(None)));
    targets = criterion_benchmark
);
criterion_main!(benches);
