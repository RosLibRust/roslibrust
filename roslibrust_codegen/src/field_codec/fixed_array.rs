//! Const-generic fixed-array codec shared by generated fields and the ROS byte adapter.
//!
//! Fixed arrays are tuples on the wire: their lengths come from the schema, so serialization
//! must not write a sequence length prefix. Human-readable formats represent them as arrays.

use serde::{
    de::{Error as _, SeqAccess, Visitor},
    ser::SerializeTuple,
    Deserialize, Deserializer, Serialize, Serializer,
};
use std::{fmt, marker::PhantomData, mem::MaybeUninit};

#[inline]
pub fn serialize<T: Serialize, S: Serializer, const N: usize>(
    value: &[T; N],
    serializer: S,
) -> Result<S::Ok, S::Error> {
    let mut tuple = serializer.serialize_tuple(N)?;
    for element in value {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

#[inline]
pub fn deserialize<'de, T: Deserialize<'de>, D: Deserializer<'de>, const N: usize>(
    deserializer: D,
) -> Result<[T; N], D::Error> {
    deserializer.deserialize_tuple(N, ArrayVisitor::<T, N>(PhantomData))
}

struct ArrayVisitor<T, const N: usize>(PhantomData<T>);

/// Own the initialized prefix so an error or panic drops each decoded element once.
struct PartialArray<T, const N: usize> {
    values: Option<MaybeUninit<[T; N]>>,
    initialized: usize,
}

impl<T, const N: usize> Drop for PartialArray<T, N> {
    #[inline]
    fn drop(&mut self) {
        // Make this a compile-time no-op for primitive arrays, so the compiler can also
        // eliminate prefix bookkeeping when elements have no destructor.
        if !std::mem::needs_drop::<T>() {
            return;
        }
        let Some(values) = &mut self.values else {
            return; // Ownership was transferred to the completed array.
        };
        // SAFETY: the first `initialized` elements were written successfully. The remaining
        // storage is uninitialized and must not be read or dropped. The pointer is aligned
        // and non-null even for zero-length arrays and zero-sized element types.
        unsafe {
            let prefix = std::ptr::slice_from_raw_parts_mut(
                values.as_mut_ptr().cast::<T>(),
                self.initialized,
            );
            std::ptr::drop_in_place(prefix);
        }
    }
}

impl<'de, T: Deserialize<'de>, const N: usize> Visitor<'de> for ArrayVisitor<T, N> {
    type Value = [T; N];

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "an array of length {N}")
    }

    #[inline]
    fn visit_seq<A: SeqAccess<'de>>(self, mut sequence: A) -> Result<Self::Value, A::Error> {
        // Initialize the final array storage directly, without an intermediate allocation,
        // Option array, conversion pass, Copy bound, or Default bound.
        let mut array = PartialArray::<T, N> {
            values: Some(MaybeUninit::uninit()),
            initialized: 0,
        };
        let values = array.values.as_mut().unwrap().as_mut_ptr().cast::<T>();
        for index in 0..N {
            let element = sequence
                .next_element()?
                .ok_or_else(|| A::Error::invalid_length(index, &self))?;
            // SAFETY: index is in 0..N, points to an unwritten array slot, and is aligned
            // for T. Writing cannot panic; the prefix count advances only after the write.
            unsafe {
                values.add(index).write(element);
            }
            array.initialized += 1;
        }
        // Like Serde's array visitor, consume exactly N elements. deserialize_tuple owns
        // the tuple boundary and rejects trailing elements in human-readable formats.
        // SAFETY: all N elements are initialized. Transfer ownership to the returned array
        // and disarm the guard so it cannot drop those elements a second time.
        let values = array.values.take().unwrap();
        Ok(unsafe { values.assume_init() })
    }
}
