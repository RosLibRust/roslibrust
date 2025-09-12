use roslibrust_test::ros2::*;

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

    // Basic string message
    assert_eq!(
        std_msgs::String::ROS2_HASH,
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    );

    // MultiArrayLayout has a dependency on MultiArrayDimension
    // Proving we can handle chained dependencies
    assert_eq!(
        std_msgs::MultiArrayLayout::ROS2_HASH,
        "RIHS01_4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6"
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
        std_msgs::Empty::ROS2_HASH,
        "RIHS01_20b625256f32d5dbc0d04fee44f43c41e51c70d3502f84b4a08e7a9c26a96312"
    );

    // Testing a message with a capacity limit on a primitive
    assert_eq!(
        ros2_test_msgs::BoundedInt::ROS2_HASH,
        "RIHS01_82105d57673153229ad6f8aa943ead090bc9756da35c9b73cf919a64bf902d6c"
    );

    // Testing a message with a capacity limit on a nested type
    assert_eq!(
        ros2_test_msgs::BoundedReferenced::ROS2_HASH,
        "RIHS01_f6a7e6732d79d9bfff7abc475f2dda336233f75d3b1984537d2acb07054c3975"
    );

    // Testing a message with a capacity limit on a string
    assert_eq!(
        ros2_test_msgs::BoundedString::ROS2_HASH,
        "RIHS01_e9965b1be42dce770d8936688f88b2f79cc1a828daa2295f1224186fd3c85b70"
    );

    // Testing a message that references builtin_interfaces/Time and builtin_interfaces/Duration
    assert_eq!(
        ros2_test_msgs::Stamped::ROS2_HASH,
        "RIHS01_d568324df43c14673c2d014c1c568d399b42e793d32050a4f396fd3cbd2f50cc"
    );

    // Testing a message working with the `char` type
    assert_eq!(
        ros2_test_msgs::Char::ROS2_HASH,
        "RIHS01_7398a047b7ae995eeb12bd58e9884a30f25968d9c19a79b929778bebde4e678a"
    );

    // Test a message with a bool
    assert_eq!(
        ros2_test_msgs::Bool::ROS2_HASH,
        "RIHS01_14ae37d5c5f596ff013e8658e1af0c02f9dcad8f0d75e3d6243809c90264acd7"
    );

    // A message that previously failed on us
    assert_eq!(
        sensor_msgs::CameraInfo::ROS2_HASH,
        "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275"
    );

    // Very basic service
    assert_eq!(
        std_srvs::SetBool::ROS2_HASH,
        "RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a"
    );

    // Test empty service
    assert_eq!(
        std_srvs::Empty::ROS2_HASH,
        "RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19"
    );

    // More complicated service
    assert_eq!(
        sensor_msgs::SetCameraInfo::ROS2_HASH,
        "RIHS01_a10cca5d33dc637c8d49db50ab288701a3592bb9cd854f2f16a0659613b68984"
    );
}
