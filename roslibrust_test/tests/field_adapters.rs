//! Exercise the shared codecs directly, independently of message generation.

use roslibrust::codegen::serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize)]
#[serde(crate = "roslibrust::codegen::serde")]
#[serde(bound(serialize = "T: Serialize", deserialize = "T: Deserialize<'de>"))]
struct Array<T, const N: usize> {
    #[serde(with = "roslibrust::codegen::field_codec::fixed_array")]
    values: [T; N],
    tail: u32,
}

#[test]
fn fixed_arrays_support_every_length_and_non_copy_elements() {
    fn check<const N: usize>() {
        let value = Array {
            values: std::array::from_fn::<_, N, _>(|index| index.to_string()),
            tail: 42,
        };
        let json = serde_json::to_value(&value).unwrap();
        assert_eq!(json["values"].as_array().unwrap().len(), N);
        assert_eq!(
            serde_json::from_value::<Array<String, N>>(json).unwrap(),
            value
        );
        let binary = roslibrust_serde_rosmsg::to_vec(&value).unwrap();
        assert_eq!(
            roslibrust_serde_rosmsg::from_slice::<Array<String, N>>(&binary).unwrap(),
            value
        );
    }
    check::<0>();
    check::<1>();
    check::<32>();
    check::<33>();
    check::<40>();
}

#[test]
fn fixed_array_binary_framing_has_no_length_prefix() {
    let value = Array {
        values: std::array::from_fn::<_, 40, _>(|index| index as u16),
        tail: 0x01020304,
    };
    let binary = roslibrust_serde_rosmsg::to_vec(&value).unwrap();
    let mut payload: Vec<u8> = (0u16..40).flat_map(u16::to_le_bytes).collect();
    payload.extend_from_slice(&value.tail.to_le_bytes());
    // ROS messages have one outer payload length, but fixed arrays have no inner length.
    let mut expected = (payload.len() as u32).to_le_bytes().to_vec();
    expected.extend_from_slice(&payload);
    assert_eq!(binary, expected);
    assert_eq!(
        roslibrust_serde_rosmsg::from_slice::<Array<u16, 40>>(&binary).unwrap(),
        value
    );
    assert!(
        roslibrust_serde_rosmsg::from_slice::<Array<u16, 40>>(&binary[..binary.len() - 1]).is_err()
    );
}

#[test]
fn fixed_arrays_reject_short_and_long_json_arrays() {
    for length in [0, 1, 39, 41] {
        let invalid = serde_json::json!({"values": vec!["value"; length], "tail": 42});
        assert!(serde_json::from_value::<Array<String, 40>>(invalid).is_err());
    }
    assert!(serde_json::from_value::<Array<String, 0>>(
        serde_json::json!({"values": ["extra"], "tail": 42})
    )
    .is_err());
}

#[test]
fn partially_decoded_elements_are_dropped_on_every_error_path() {
    use std::cell::Cell;
    thread_local! { static DROPS: Cell<usize> = const { Cell::new(0) }; }

    // Deliberately no Copy or Default implementation: the codec needs neither.
    #[derive(Debug, Deserialize)]
    #[serde(crate = "roslibrust::codegen::serde", transparent)]
    struct Element(u8);
    impl Drop for Element {
        fn drop(&mut self) {
            DROPS.with(|count| count.set(count.get() + 1));
        }
    }

    for (values, expected_drops) in [("[1,2]", 2), ("[1,2,\"invalid\",4]", 2), ("[1,2,3,4,5]", 4)] {
        DROPS.with(|count| count.set(0));
        let json = format!("{{\"values\":{values},\"tail\":42}}");
        assert!(serde_json::from_str::<Array<Element, 4>>(&json).is_err());
        assert_eq!(DROPS.with(Cell::get), expected_drops);
    }
    DROPS.with(|count| count.set(0));
    let value =
        serde_json::from_str::<Array<Element, 4>>(r#"{"values":[1,2,3,4],"tail":42}"#).unwrap();
    assert_eq!(value.values[3].0, 4);
    assert_eq!(DROPS.with(Cell::get), 0);
    drop(value);
    assert_eq!(DROPS.with(Cell::get), 4);
}

#[test]
fn partially_decoded_elements_are_dropped_when_deserialization_panics() {
    use std::cell::Cell;
    thread_local! { static DROPS: Cell<usize> = const { Cell::new(0) }; }

    #[derive(Debug)]
    struct Element;
    impl<'de> Deserialize<'de> for Element {
        fn deserialize<D: roslibrust::codegen::serde::Deserializer<'de>>(
            deserializer: D,
        ) -> Result<Self, D::Error> {
            let value = u8::deserialize(deserializer)?;
            assert_ne!(value, 3, "deliberate element decoding panic");
            Ok(Self)
        }
    }
    impl Drop for Element {
        fn drop(&mut self) {
            DROPS.with(|count| count.set(count.get() + 1));
        }
    }

    // Element is zero-sized and owns a Drop action; the guard must count values, not bytes.
    assert_eq!(std::mem::size_of::<Element>(), 0);
    let result = std::panic::catch_unwind(|| {
        serde_json::from_str::<Array<Element, 4>>(r#"{"values":[1,2,3,4],"tail":42}"#)
    });
    assert!(result.is_err());
    assert_eq!(DROPS.with(Cell::get), 2);

    DROPS.with(|count| count.set(0));
    let value =
        serde_json::from_str::<Array<Element, 4>>(r#"{"values":[1,2,4,5],"tail":42}"#).unwrap();
    assert_eq!(DROPS.with(Cell::get), 0);
    drop(value);
    assert_eq!(DROPS.with(Cell::get), 4);
}
