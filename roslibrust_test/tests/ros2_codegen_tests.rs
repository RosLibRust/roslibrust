use roslibrust_test::ros2::*;
use serde_json::json;

#[test]
fn test_defaults() {
    let x: ros2_test_msgs::Defaults = Default::default();
    assert_eq!(x.x, 42);
    assert_eq!(x.y, -2000);
    assert_eq!(x.full_name, "John Doe");
    assert_eq!(x.samples, vec![-200, -100, 0, 100, 200]);
    assert_eq!(x.s_vec, vec!["hello", "world"]);
    assert_eq!(x.f_samples, vec![-200.0, -1.0, 0.0]);
}

#[test]
fn descriptor_constructs_message_defaults() {
    let descriptor = MESSAGE_REGISTRY.get("ros2_test_msgs/Defaults").unwrap();
    let message = descriptor.default_message().unwrap();

    assert_eq!(
        serde_json::to_value(message.value()).unwrap(),
        json!({
            "x": 42,
            "y": -2000,
            "full_name": "John Doe",
            "samples": [-200, -100, 0, 100, 200],
            "s_vec": ["hello", "world"],
            "f_samples": [-200.0, -1.0, 0.0]
        })
    );
}

#[test]
fn fixed_sized_arrays() {
    // Prove the default works, compiler failure here is the test
    let x: sensor_msgs::NavSatFix = Default::default();
    // Prove the types here match, compiler failure here is the test
    let _y: [f64; 9] = x.position_covariance;
    // NavSatFix is the easy case because it is <32 items long

    // Harder case that deals with arrays >32
    let x: geometry_msgs::TwistWithCovariance = Default::default();
    let _y: [f64; 36] = x.covariance;
}

/// Confirms that ros2 hashes are being populated with correct expected values
#[test]
fn ros2_hash_checks() {
    use roslibrust::{RosMessageType, RosServiceType};
    use roslibrust_test::ros2::*;

    let parse = |x: &str| -> [u8; 32] {
        let stripped = x.strip_prefix("RIHS01_").unwrap();
        hex::decode(stripped).unwrap().try_into().unwrap()
    };

    // Basic string message
    assert_eq!(
        *std_msgs::String::DESCRIPTION.ros2_hash,
        parse("RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18")
    );

    // MultiArrayLayout has a dependency on MultiArrayDimension
    // Proving we can handle chained dependencies
    assert_eq!(
        *std_msgs::MultiArrayLayout::DESCRIPTION.ros2_hash,
        parse("RIHS01_4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6")
    );

    // Empty is an edge case so worth checking
    // Note: this is a weird one... JSON:
    /*
    "type_description_msg": {
      "type_description": {
        "type_name": "std_msgs/msg/Empty",
        "fields": [
          {
            "name": "structure_needs_at_least_one_member",
            "type": {
              "type_id": 3,
              "capacity": 0,
              "string_capacity": 0,
              "nested_type_name": ""
            },
            "default_value": ""
          }
        ]
      },
      "referenced_type_descriptions": []
    }
    */
    assert_eq!(
        *std_msgs::Empty::DESCRIPTION.ros2_hash,
        parse("RIHS01_20b625256f32d5dbc0d04fee44f43c41e51c70d3502f84b4a08e7a9c26a96312")
    );

    // Testing a message with a capacity limit on a primitive
    assert_eq!(
        *ros2_test_msgs::BoundedInt::DESCRIPTION.ros2_hash,
        parse("RIHS01_82105d57673153229ad6f8aa943ead090bc9756da35c9b73cf919a64bf902d6c")
    );

    // Testing a message with a capacity limit on a nested type
    assert_eq!(
        *ros2_test_msgs::BoundedReferenced::DESCRIPTION.ros2_hash,
        parse("RIHS01_f6a7e6732d79d9bfff7abc475f2dda336233f75d3b1984537d2acb07054c3975")
    );

    // Testing a message with a capacity limit on a string
    assert_eq!(
        *ros2_test_msgs::BoundedString::DESCRIPTION.ros2_hash,
        parse("RIHS01_e9965b1be42dce770d8936688f88b2f79cc1a828daa2295f1224186fd3c85b70")
    );

    // Testing a message that references builtin_interfaces/Time and builtin_interfaces/Duration
    assert_eq!(
        *ros2_test_msgs::Stamped::DESCRIPTION.ros2_hash,
        parse("RIHS01_d568324df43c14673c2d014c1c568d399b42e793d32050a4f396fd3cbd2f50cc")
    );

    // Testing a message working with the `char` type
    assert_eq!(
        *ros2_test_msgs::Char::DESCRIPTION.ros2_hash,
        parse("RIHS01_7398a047b7ae995eeb12bd58e9884a30f25968d9c19a79b929778bebde4e678a")
    );

    // Test a message with a bool
    assert_eq!(
        *ros2_test_msgs::Bool::DESCRIPTION.ros2_hash,
        parse("RIHS01_14ae37d5c5f596ff013e8658e1af0c02f9dcad8f0d75e3d6243809c90264acd7")
    );

    // A message that previously failed on us
    assert_eq!(
        *sensor_msgs::CameraInfo::DESCRIPTION.ros2_hash,
        parse("RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275")
    );

    // Very basic service
    assert_eq!(
        *std_srvs::SetBool::ROS2_HASH,
        parse("RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a")
    );

    // Test empty service
    assert_eq!(
        *std_srvs::Empty::ROS2_HASH,
        parse("RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19")
    );

    // More complicated service
    assert_eq!(
        *sensor_msgs::SetCameraInfo::ROS2_HASH,
        parse("RIHS01_a10cca5d33dc637c8d49db50ab288701a3592bb9cd854f2f16a0659613b68984")
    );
}

#[test]
fn bounded_bytes_validate_all_entry_points_and_preserve_framing() {
    use roslibrust::{DynamicValue, RosMessageType};
    use roslibrust_test::ros2::ros2_test_msgs::BoundedInt;
    let descriptor = &BoundedInt::DESCRIPTION;
    for len in [0, 5, 6] {
        let typed = BoundedInt { data: vec![7; len] };
        let valid = len <= 5;
        assert_eq!(serde_json::to_value(&typed).is_ok(), valid);
        assert_eq!(
            serde_json::from_value::<BoundedInt>(json!({"data": vec![7; len]})).is_ok(),
            valid
        );
        assert_eq!(
            descriptor
                .message_from(&json!({"data": vec![7; len]}))
                .is_ok(),
            valid
        );
        let bytes = vec![7; len];
        let value = DynamicValue::Message(vec![roslibrust::DynamicField {
            name: "data".into(),
            value: DynamicValue::Bytes(bytes),
        }]);
        assert_eq!(descriptor.message(value).is_ok(), valid);
        let binary = roslibrust_serde_rosmsg::to_vec(&typed);
        assert_eq!(binary.is_ok(), valid);
        if valid {
            let binary = binary.unwrap();
            assert_eq!(&binary[..4], &((len + 4) as u32).to_le_bytes());
            assert_eq!(&binary[4..8], &(len as u32).to_le_bytes());
            assert_eq!(binary.len(), len + 8);
            assert_eq!(
                roslibrust_serde_rosmsg::from_slice::<BoundedInt>(&binary).unwrap(),
                typed
            );
        }
    }
    assert_eq!(
        serde_json::to_value(BoundedInt {
            data: vec![1, 2, 3, 4, 5]
        })
        .unwrap(),
        json!({"data":"AQIDBAU="})
    );
    let err = serde_json::from_value::<BoundedInt>(json!({"data":"AQIDBAUG"})).unwrap_err();
    assert!(err.to_string().contains("field `data`"));
    let mut wire = 10u32.to_le_bytes().to_vec();
    wire.extend(6u32.to_le_bytes());
    wire.extend([1, 2, 3, 4, 5, 6]);
    assert!(roslibrust_serde_rosmsg::from_slice::<BoundedInt>(&wire)
        .unwrap_err()
        .to_string()
        .contains("field `data`"));
}

#[test]
fn bounded_strings_count_utf8_bytes() {
    use roslibrust::RosMessageType;
    use roslibrust_test::ros2::ros2_test_msgs::BoundedString;
    for text in ["", "12345", "123456", "éé", "ééé"] {
        let valid = text.len() <= 5;
        let typed = BoundedString {
            data: text.into(),
            ..Default::default()
        };
        assert_eq!(serde_json::to_value(&typed).is_ok(), valid);
        assert_eq!(
            serde_json::from_value::<BoundedString>(
                json!({"data":text,"data_list":[],"data_matrix":[]})
            )
            .is_ok(),
            valid
        );
        assert_eq!(
            BoundedString::DESCRIPTION
                .message_from(&json!({"data":text,"data_list":[],"data_matrix":[]}))
                .is_ok(),
            valid
        );
        assert_eq!(roslibrust_serde_rosmsg::to_vec(&typed).is_ok(), valid);
    }
}

#[test]
fn bounded_string_arrays_and_nested_messages_validate_recursively() {
    use roslibrust::RosMessageType;
    use roslibrust_test::ros2::ros2_test_msgs::{BoundedInt, BoundedReferenced, BoundedStrings};
    let valid = json!({"fixed":["a","12345"], "unbounded":["éé"], "bounded":["abc"], "nested":[{"data":"abc","data_list":[],"data_matrix":[]}]});
    let typed: BoundedStrings = serde_json::from_value(valid.clone()).unwrap();
    let default = BoundedStrings::DESCRIPTION.default_message().unwrap();
    assert_eq!(
        default.get("bounded").unwrap().as_sequence().unwrap()[0].as_str(),
        Some("abc")
    );
    assert!(BoundedStrings::DESCRIPTION.message_from(&valid).is_ok());
    let binary = roslibrust_serde_rosmsg::to_vec(&typed).unwrap();
    assert_eq!(
        roslibrust_serde_rosmsg::from_slice::<BoundedStrings>(&binary).unwrap(),
        typed
    );
    for (field, invalid) in [
        ("fixed", json!(["a", "123456"])),
        ("fixed", json!(["a"])),
        ("unbounded", json!(["123456"])),
        ("bounded", json!(["a", "b", "c", "d"])),
        ("bounded", json!(["123456"])),
        (
            "nested",
            json!([{"data":"123456","data_list":[],"data_matrix":[]}]),
        ),
        (
            "nested",
            json!([{"data":"a","data_list":[],"data_matrix":[]},{"data":"b","data_list":[],"data_matrix":[]},{"data":"c","data_list":[],"data_matrix":[]}]),
        ),
    ] {
        let mut value = valid.clone();
        value[field] = invalid;
        assert!(
            serde_json::from_value::<BoundedStrings>(value.clone()).is_err(),
            "{field}"
        );
        assert!(
            BoundedStrings::DESCRIPTION.message_from(&value).is_err(),
            "{field}"
        );
    }
    let mut bad = typed;
    bad.fixed[1] = "123456".into();
    assert!(serde_json::to_value(&bad)
        .unwrap_err()
        .to_string()
        .contains("fixed[1]"));
    assert!(roslibrust_serde_rosmsg::to_vec(&bad).is_err());
    for value in [
        json!({"data":[{"data":[1,2,3,4,5,6]}]}),
        json!({"data":[{"data":[]},{"data":[]},{"data":[]},{"data":[]}]}),
    ] {
        assert!(BoundedReferenced::DESCRIPTION.message_from(&value).is_err());
    }
    let bad = BoundedReferenced {
        data: vec![BoundedInt { data: vec![0; 6] }],
    };
    assert!(serde_json::to_value(&bad).is_err());
}

#[test]
fn descriptor_compatibility_uses_metadata_values() {
    use roslibrust::RosMessageType;
    let original = roslibrust_test::ros2::std_msgs::String::DESCRIPTION;
    let duplicate = original;
    assert!(original.is_compatible_with(&duplicate));
    let changed = roslibrust::MessageDescriptor::new::<std_msgs::String>(
        original.ros_type_name,
        original.md5sum,
        "uint8 data",
        original.ros2_type_name,
        original.ros2_hash,
    );
    assert!(!original.is_compatible_with(&changed));
    let changed_hash = roslibrust::MessageDescriptor::new::<std_msgs::String>(
        original.ros_type_name,
        original.md5sum,
        original.definition,
        original.ros2_type_name,
        &[1; 32],
    );
    assert!(!original.is_compatible_with(&changed_hash));
    assert!(!original.is_compatible_with(&roslibrust_test::ros1::std_msgs::String::DESCRIPTION));
}
