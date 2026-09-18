use roslibrust::{DynamicField, DynamicMessageError, DynamicValue};
use roslibrust_test::ros1::MESSAGE_REGISTRY;

fn field(name: &str, value: DynamicValue) -> DynamicField {
    DynamicField {
        name: name.to_owned(),
        value,
    }
}

#[test]
fn dynamic_value_uses_its_natural_serde_shape() {
    let value = DynamicValue::Message(vec![
        field("enabled", DynamicValue::Bool(true)),
        field(
            "samples",
            DynamicValue::Sequence(vec![DynamicValue::I16(-1), DynamicValue::I16(2)]),
        ),
    ]);

    assert_eq!(
        serde_json::to_value(&value).unwrap(),
        serde_json::json!({"enabled": true, "samples": [-1, 2]})
    );
}

fn assert_invalid(type_name: &str, value: DynamicValue, expected: &str) {
    let error = MESSAGE_REGISTRY
        .from_value(type_name, value)
        .expect_err("value should not satisfy the generated ROS schema");
    assert!(
        matches!(error, DynamicMessageError::InvalidValue { .. }),
        "unexpected error variant: {error:?}"
    );
    assert!(
        error.to_string().contains(expected),
        "expected error to contain {expected:?}, got {error}"
    );
}

#[test]
fn raw_values_are_unvalidated_but_dynamic_messages_are_schema_checked() {
    let heterogeneous = DynamicValue::Message(vec![
        field(
            "layout",
            DynamicValue::Message(vec![
                field("dim", DynamicValue::Sequence(vec![])),
                field("data_offset", DynamicValue::I64(0)),
            ]),
        ),
        field(
            "data",
            DynamicValue::Sequence(vec![
                DynamicValue::I64(1),
                DynamicValue::String("2".to_owned()),
                DynamicValue::F64(3.0),
            ]),
        ),
    ]);

    // Schema-free JSON parsing preserves the heterogeneous input. It cannot know that `data`
    // should be an i16 sequence yet.
    let DynamicValue::Message(fields) = &heterogeneous else {
        panic!("expected a message")
    };
    let data = fields.iter().find(|field| field.name == "data").unwrap();
    assert_eq!(
        data.value,
        DynamicValue::Sequence(vec![
            DynamicValue::I64(1),
            DynamicValue::String("2".to_owned()),
            DynamicValue::F64(3.0),
        ])
    );

    // Associating it with a generated descriptor checks every sequence element as i16.
    assert_invalid(
        "std_msgs/Int16MultiArray",
        heterogeneous,
        "invalid type: string",
    );
}

#[test]
fn homogeneous_sequence_with_the_schema_element_type_is_accepted() {
    let value = DynamicValue::Message(vec![
        field(
            "layout",
            DynamicValue::Message(vec![
                field("dim", DynamicValue::Sequence(vec![])),
                field("data_offset", DynamicValue::U32(0)),
            ]),
        ),
        field(
            "data",
            DynamicValue::Sequence(vec![DynamicValue::I16(1), DynamicValue::I16(2)]),
        ),
    ]);

    let message = MESSAGE_REGISTRY
        .from_value("std_msgs/Int16MultiArray", value.clone())
        .unwrap();
    assert_eq!(message.value(), &value);
}

#[test]
fn compatible_numeric_widths_are_canonicalized_to_the_schema_type() {
    let value = DynamicValue::Message(vec![
        field(
            "layout",
            DynamicValue::Message(vec![
                field("dim", DynamicValue::Sequence(vec![])),
                field("data_offset", DynamicValue::U8(0)),
            ]),
        ),
        field(
            "data",
            DynamicValue::Sequence(vec![DynamicValue::I8(1), DynamicValue::I32(2)]),
        ),
    ]);

    let message = MESSAGE_REGISTRY
        .from_value("std_msgs/Int16MultiArray", value)
        .unwrap();
    assert_eq!(
        message.value(),
        &DynamicValue::Message(vec![
            field(
                "layout",
                DynamicValue::Message(vec![
                    field("dim", DynamicValue::Sequence(vec![])),
                    field("data_offset", DynamicValue::U32(0)),
                ]),
            ),
            field(
                "data",
                DynamicValue::Sequence(vec![DynamicValue::I16(1), DynamicValue::I16(2)]),
            ),
        ])
    );
}

#[test]
fn missing_required_field_is_rejected() {
    assert_invalid(
        "std_msgs/Int16",
        DynamicValue::Message(vec![]),
        "missing field `data`",
    );
}

#[test]
fn unknown_field_is_rejected() {
    assert_invalid(
        "std_msgs/Int16",
        DynamicValue::Message(vec![
            field("data", DynamicValue::I16(42)),
            field("unexpected", DynamicValue::Bool(true)),
        ]),
        "unknown field `unexpected`",
    );
}

#[test]
fn duplicate_field_is_rejected() {
    assert_invalid(
        "std_msgs/Int16",
        DynamicValue::Message(vec![
            field("data", DynamicValue::I16(1)),
            field("data", DynamicValue::I16(2)),
        ]),
        "duplicate field `data`",
    );
}

#[test]
fn incorrect_scalar_type_is_rejected() {
    assert_invalid(
        "std_msgs/Int16",
        DynamicValue::Message(vec![field("data", DynamicValue::String("42".to_owned()))]),
        "invalid type: string",
    );
}

#[test]
fn numeric_input_is_range_checked_and_canonicalized_when_parsed_with_a_schema() {
    let message = MESSAGE_REGISTRY
        .message_from("std_msgs/Int16", &serde_json::json!({ "data": 42 }))
        .unwrap();
    assert_eq!(
        message.value(),
        &DynamicValue::Message(vec![field("data", DynamicValue::I16(42))])
    );

    let error = MESSAGE_REGISTRY
        .message_from("std_msgs/Int16", &serde_json::json!({ "data": 40_000 }))
        .unwrap_err();
    assert!(matches!(error, DynamicMessageError::InvalidValue { .. }));
    assert!(error.to_string().contains("invalid value"));
}

#[test]
fn fixed_length_array_requires_exactly_the_schema_length() {
    assert_invalid(
        "shape_msgs/MeshTriangle",
        DynamicValue::Message(vec![field(
            "vertex_indices",
            DynamicValue::Sequence(vec![DynamicValue::U32(0), DynamicValue::U32(1)]),
        )]),
        "invalid length 2, expected an array of length 3",
    );
}

#[test]
fn nested_messages_are_validated_recursively() {
    assert_invalid(
        "geometry_msgs/Pose",
        DynamicValue::Message(vec![
            field(
                "position",
                DynamicValue::Message(vec![
                    field("x", DynamicValue::F64(1.0)),
                    field("y", DynamicValue::F64(2.0)),
                    // Point.z is missing.
                ]),
            ),
            field(
                "orientation",
                DynamicValue::Message(vec![
                    field("x", DynamicValue::F64(0.0)),
                    field("y", DynamicValue::F64(0.0)),
                    field("z", DynamicValue::F64(0.0)),
                    field("w", DynamicValue::F64(1.0)),
                ]),
            ),
        ]),
        "missing field `z`",
    );
}

#[test]
fn field_order_does_not_affect_validation() {
    let value = DynamicValue::Message(vec![
        field("theta", DynamicValue::F64(3.0)),
        field("x", DynamicValue::F64(1.0)),
        field("y", DynamicValue::F64(2.0)),
    ]);
    let message = MESSAGE_REGISTRY
        .from_value("geometry_msgs/Pose2D", value)
        .unwrap();
    let DynamicValue::Message(fields) = message.value() else {
        panic!("expected a message")
    };
    assert_eq!(
        fields
            .iter()
            .map(|field| field.name.as_str())
            .collect::<Vec<_>>(),
        ["x", "y", "theta"]
    );
}
