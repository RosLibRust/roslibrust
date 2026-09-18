# Runtime-selected messages

RosLibRust's usual topic API chooses a generated Rust message type at compile time. The dynamic
API is for tools—such as topic inspectors, recorders, bridges, and command-line publishers—that
only learn a ROS type name at runtime.

The API has three distinct layers:

- `DynamicValue` is a transport-neutral value tree. It preserves ROS-relevant scalar widths,
  ordered fields, byte arrays, and non-finite floats, but has no schema by itself.
- `DynamicMessage` pairs a value with a generated `MessageDescriptor`. Creating one validates and
  canonicalizes the value through the generated Rust type.
- `DynamicTopicProvider` lets a backend advertise or subscribe with a `MessageDescriptor` chosen
  at runtime. The backend remains responsible for selecting its wire serializer.

## Looking up and validating a message

Code generation emits a `MESSAGE_REGISTRY` next to the generated modules. It accepts canonical
ROS 1 names and common ROS 2 spellings:

```rust,ignore
use my_messages::MESSAGE_REGISTRY;

let descriptor = MESSAGE_REGISTRY
    .get("std_msgs/String")
    .expect("message type was not generated");

let message = descriptor.message_from(&serde_json::json!({
    "data": "hello from a runtime-selected type"
}))?;
```

`message_from` accepts any Serde `Serialize` value whose shape represents the selected message.
Structs and string-keyed maps are supported. The registry offers the same operation when the type
name is not resolved separately:

```rust,ignore
let message = MESSAGE_REGISTRY.message_from(
    runtime_type_name,
    &serde_json::json!({"data": "hello"}),
)?;
```

Validation checks required and unknown fields, scalar types and ranges, nested messages, sequence
elements, and fixed array lengths. The stored value is rebuilt in schema field order with the
schema's exact scalar widths. A raw `DynamicValue::Sequence` can temporarily contain unlike
elements because it has no element schema; it cannot become a `DynamicMessage` for a homogeneous
ROS sequence unless every element validates.

```rust,ignore
use roslibrust::{DynamicField, DynamicValue};

let raw = DynamicValue::Message(vec![DynamicField {
    name: "data".to_owned(),
    value: DynamicValue::Sequence(vec![
        DynamicValue::I64(1),
        DynamicValue::String("not an integer".to_owned()),
        DynamicValue::I64(3),
    ]),
}]);

// Fails if `data` is an integer sequence in the selected generated type.
let checked = descriptor.message(raw)?;
```

## Dynamic publishing and subscribing

`DynamicTopicProvider` mirrors `TopicProvider`, but takes a descriptor instead of a generic Rust
type:

```rust,ignore
use roslibrust::{DynamicPublish, DynamicSubscribe, DynamicTopicProvider};

let descriptor = MESSAGE_REGISTRY
    .get(runtime_type_name)
    .expect("message type was not generated");

let mut subscriber = ros.dynamic_subscribe("/chatter", descriptor).await?;
let publisher = ros.dynamic_advertise("/chatter", descriptor).await?;

let message = descriptor.message_from(&serde_json::json!({"data": "hello"}))?;
publisher.publish(&message).await?;

let received = subscriber.next().await?;
println!(
    "{}: {}",
    received.descriptor().ros_type_name,
    serde_json::to_string(received.value())?,
);
```

For one-off values, `publish_serializable` combines construction, validation, and publishing:

```rust,ignore
publisher
    .publish_serializable(&serde_json::json!({"data": "hello"}))
    .await?;
```

Construct a `DynamicMessage` once and use `publish` when sending the same value repeatedly.

`DynamicValue` implements `serde::Serialize`, so tools can select their own display or interchange
format. The selected serializer owns format-specific policies; for example, JSON serializers decide
how to represent byte arrays and non-finite floating-point values.

## Serializer ownership

Generated descriptors provide format-neutral Serde conversion operations. They do not choose JSON,
ROS 1 wire encoding, or CDR. A backend implementing `DynamicTopicProvider` chooses its correct
serializer and deserializer, then calls `DynamicMessage::serialize_with` or
`MessageDescriptor::deserialize_with`. This keeps knowledge of transport encoding in the backend
while sharing generated-type lookup and schema validation.

`MockRos`, native ROS 1, rosbridge, the Zenoh ROS 1 bridge backend, and the Hiroz native ROS 2
backend all implement `DynamicTopicProvider`. Each implementation selects its native wire format:
bincode for the mock, ROS serialization for ROS 1 and its Zenoh bridge, JSON for rosbridge, and CDR
for Hiroz.
