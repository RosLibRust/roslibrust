//! Performance benchmarks for roslibrust_mcap
//!
//! Run with: `cargo bench -p roslibrust_mcap`

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use std::io::Cursor;

use roslibrust_test::ros2::*;

fn benchmark_write_messages(c: &mut Criterion) {
    let mut group = c.benchmark_group("write_messages");

    for message_count in [100, 1000, 10000].iter() {
        group.throughput(Throughput::Elements(*message_count as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(message_count),
            message_count,
            |b, &count| {
                b.iter(|| {
                    let mut buffer = Vec::new();
                    let mut writer =
                        roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer)).unwrap();
                    let channel = writer.add_ros_channel::<std_msgs::String>("/test").unwrap();

                    for i in 0..count {
                        channel
                            .write(
                                &mut writer,
                                i * 1_000_000,
                                &std_msgs::String {
                                    data: format!("Message {}", i),
                                },
                            )
                            .unwrap();
                    }

                    writer.finish().unwrap();
                    black_box(buffer);
                });
            },
        );
    }
    group.finish();
}

fn benchmark_read_messages(c: &mut Criterion) {
    let mut group = c.benchmark_group("read_messages");

    for message_count in [100, 1000, 10000].iter() {
        // Pre-create the MCAP file
        let mut buffer = Vec::new();
        {
            let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer)).unwrap();
            let channel = writer.add_ros_channel::<std_msgs::String>("/test").unwrap();

            for i in 0..*message_count {
                channel
                    .write(
                        &mut writer,
                        i * 1_000_000,
                        &std_msgs::String {
                            data: format!("Message {}", i),
                        },
                    )
                    .unwrap();
            }

            writer.finish().unwrap();
        }

        group.throughput(Throughput::Elements(*message_count as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(message_count),
            message_count,
            |b, _count| {
                b.iter(|| {
                    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer)).unwrap();
                    let messages: Vec<_> = reader
                        .iter_messages()
                        .unwrap()
                        .collect::<roslibrust_mcap::Result<Vec<_>>>()
                        .unwrap();
                    black_box(messages);
                });
            },
        );
    }
    group.finish();
}

fn benchmark_deserialize_messages(c: &mut Criterion) {
    let mut group = c.benchmark_group("deserialize_messages");

    for message_count in [100, 1000, 10000].iter() {
        // Pre-create the MCAP file
        let mut buffer = Vec::new();
        {
            let mut writer = roslibrust_mcap::McapWriter::new(Cursor::new(&mut buffer)).unwrap();
            let channel = writer.add_ros_channel::<std_msgs::String>("/test").unwrap();

            for i in 0..*message_count {
                channel
                    .write(
                        &mut writer,
                        i * 1_000_000,
                        &std_msgs::String {
                            data: format!("Message {}", i),
                        },
                    )
                    .unwrap();
            }

            writer.finish().unwrap();
        }

        group.throughput(Throughput::Elements(*message_count as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(message_count),
            message_count,
            |b, _count| {
                b.iter(|| {
                    let reader = roslibrust_mcap::McapReader::new(Cursor::new(&buffer)).unwrap();
                    let messages: Vec<_> = reader
                        .iter_messages()
                        .unwrap()
                        .map(|msg_result| {
                            let msg = msg_result.unwrap();
                            let deserialized: std_msgs::String =
                                roslibrust_mcap::McapReader::deserialize_message(&msg.data)
                                    .unwrap();
                            deserialized
                        })
                        .collect();
                    black_box(messages);
                });
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    benchmark_write_messages,
    benchmark_read_messages,
    benchmark_deserialize_messages
);
criterion_main!(benches);
