// Expanding the macro compiles real generated definitions, not stored generated source.
roslibrust_codegen_macro::generate_ros_types!(
    "roslibrust_codegen_macro/tests/fixtures/array_defaults"
);

#[test]
fn fixed_and_vector_default_literals_have_their_declared_types() {
    let defaults = array_defaults::Defaults::default();
    let data: [u8; 2] = defaults.data;
    let signed_values: [i32; 3] = defaults.signed_values;
    let fractions: [f32; 3] = defaults.fractions;
    let flags: [bool; 2] = defaults.flags;
    let labels: [String; 2] = defaults.labels;
    let empty: [u8; 0] = defaults.empty;
    let variable: Vec<u8> = defaults.variable;
    let bounded: Vec<String> = defaults.bounded;

    assert_eq!(data, [1, 2]);
    assert_eq!(signed_values, [-1, 0, 1]);
    assert_eq!(fractions, [-2.0, 0.5, 1.0]);
    assert_eq!(flags, [true, false]);
    assert_eq!(labels, ["hello", "quote \" and slash \\"]);
    assert!(empty.is_empty());
    assert_eq!(variable, [3, 4]);
    assert_eq!(bounded, ["one", "two"]);
}

#[test]
fn large_fixed_arrays_initialize_each_non_copy_element() {
    let defaults = array_defaults::Defaults::default();
    let zeroes: [u32; 40] = defaults.zeroes;
    let numbers: [i64; 40] = defaults.numbers;
    let explicit_names: [String; 40] = defaults.explicit_names;
    let names: [String; 40] = defaults.names;
    let mut children: [array_defaults::Child; 40] = defaults.children;

    assert_eq!(zeroes, [0; 40]);
    assert_eq!(numbers, std::array::from_fn(|index| index as i64 - 20));
    assert_eq!(
        explicit_names,
        std::array::from_fn(|index| index.to_string())
    );
    assert!(names.iter().all(String::is_empty));
    assert!(children.iter().all(|child| child.name == "child"));
    assert!(children
        .iter()
        .all(|child| child.names.iter().all(String::is_empty)));

    children[0].name.push_str(" changed");
    children[0].names[0].push_str("first");
    assert_eq!(children[1].name, "child");
    assert!(children[1].names[0].is_empty());
}

#[test]
fn generated_large_arrays_round_trip_through_serde() {
    let defaults = array_defaults::Defaults::default();
    let json = serde_json::to_value(&defaults).unwrap();
    let decoded: array_defaults::Defaults = serde_json::from_value(json).unwrap();
    assert_eq!(decoded, defaults);
}

#[test]
fn large_non_copy_arrays_preserve_string_bounds() {
    use roslibrust::RosMessageType;

    let mut message = array_defaults::BoundedArrays::default();
    assert!(message.labels.iter().all(String::is_empty));
    assert_eq!(message.flags, [true, false]);
    message.labels[39] = "12345".into();
    let json = serde_json::to_value(&message).unwrap();
    assert_eq!(
        serde_json::from_value::<array_defaults::BoundedArrays>(json.clone()).unwrap(),
        message
    );
    assert!(array_defaults::BoundedArrays::DESCRIPTION
        .message_from(&message)
        .is_ok());

    message.labels[39].push('6');
    assert!(serde_json::to_value(&message)
        .unwrap_err()
        .to_string()
        .contains("labels[39]"));
    assert!(array_defaults::BoundedArrays::DESCRIPTION
        .message_from(&message)
        .is_err());
    let mut invalid = json;
    invalid["labels"][39] = "123456".into();
    assert!(serde_json::from_value::<array_defaults::BoundedArrays>(invalid).is_err());
}
