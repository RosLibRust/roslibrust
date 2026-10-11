//! Parse ROS constants and defaults into typed initializer expressions.

use proc_macro2::TokenStream;
use quote::{quote, ToTokens};
use serde::de::DeserializeOwned;

use crate::{bail, utils::RosVersion, ArrayType, Error, RosLiteral};

pub(super) fn parse_literal(
    ros_type: &str,
    literal: &RosLiteral,
    array_info: &ArrayType,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    parse_literal_value(ros_type, &literal.inner, array_info, version)
}

// Converts a ROS string to a literal value
// Not intended to be called directly, but only via parse_literal_value.
// Wraps a serde_json deserialize call with our style of error handling.
fn parse_primitive_literal<T: DeserializeOwned + ToTokens>(
    value: &str,
    array_info: &ArrayType,
) -> Result<TokenStream, Error> {
    if !matches!(array_info, ArrayType::NotArray) {
        let parsed: Vec<T> = serde_json::from_str(value).map_err(|e|
            Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>()).as_str(), e)
        )?;
        generate_list_initializer(
            parsed.iter().map(ToTokens::to_token_stream).collect(),
            array_info,
        )
    } else {
        let parsed: T = serde_json::from_str(value).map_err(|e|
            Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>()).as_str(), e)
        )?;
        Ok(quote! { #parsed })
    }
}

/// Build a typed initializer expression directly, without stringifying generated code.
fn generate_list_initializer(
    values: Vec<TokenStream>,
    array_info: &ArrayType,
) -> Result<TokenStream, Error> {
    Ok(match array_info {
        ArrayType::FixedLength(length) => {
            if values.len() != *length {
                bail!(
                    "Fixed array default has {} elements, expected {length}",
                    values.len()
                );
            }
            quote! { [#(#values),*] }
        }
        ArrayType::Unbounded | ArrayType::Bounded(_) => {
            quote! { ::std::vec![#(#values),*] }
        }
        ArrayType::NotArray => unreachable!("List defaults require an array type"),
    })
}

/// Parse a ROS constant or default into a typed Rust expression.
///
/// Numeric lists parse according to their declared element type, so integer-looking float
/// defaults become float literals. Lists initialize arrays or vectors as declared, and string
/// lists own their elements. Scalar strings remain literals: constants borrow them, while field
/// defaults wrap them in `String::from`. ROS 1 scalar strings use the trimmed value verbatim;
/// ROS 2 scalar strings require matching single or double quotes. List syntax is JSON.
pub(super) fn parse_literal_value(
    ros_type: &str,
    value: &str,
    array_info: &ArrayType,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    let is_list = !matches!(array_info, ArrayType::NotArray);
    match ros_type {
        "bool" => parse_primitive_literal::<bool>(&value.to_lowercase(), array_info),
        "float64" => parse_primitive_literal::<f64>(value, array_info),
        "float32" => parse_primitive_literal::<f32>(value, array_info),
        "uint8" | "char" | "byte" => parse_primitive_literal::<u8>(value, array_info),
        "int8" => parse_primitive_literal::<i8>(value, array_info),
        "uint16" => parse_primitive_literal::<u16>(value, array_info),
        "int16" => parse_primitive_literal::<i16>(value, array_info),
        "uint32" => parse_primitive_literal::<u32>(value, array_info),
        "int32" => parse_primitive_literal::<i32>(value, array_info),
        "uint64" => parse_primitive_literal::<u64>(value, array_info),
        "int64" => parse_primitive_literal::<i64>(value, array_info),
        "string" => {
            if is_list {
                let parsed: Vec<String> = serde_json::from_str(value).map_err(|e|
                    Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to Vec<String>").as_str(), e)
                )?;
                let values = parsed
                    .iter()
                    .map(|value| quote! { ::std::string::String::from(#value) })
                    .collect();
                generate_list_initializer(values, array_info)
            } else {
                match version {
                    RosVersion::ROS1 => {
                        // For ROS1 then entire contents except for leading and trailing whitespace are used
                        let value = value.trim();
                        Ok(quote! { #value })
                    }
                    RosVersion::ROS2 => {
                        // For ROS2 value must be in quotes, and either single or double quotes are okay
                        // Keep the contents verbatim, matching the existing ROS 2 parser.
                        let value = value.trim();
                        if value.len() < 2 {
                            bail!("String constant must at least include start and end quotes, cannot be empty: {value}");
                        }
                        let first = value.chars().next().unwrap(); // Unwrap is okay due to previous length check
                        let last = value.chars().last().unwrap(); // Unwrap is okay due to previous length check
                        if first != last || !(first == '\'' || first == '\"') {
                            bail!("ROS2 String constant was found that was not enclosed in single or double quotes: {value}");
                        }
                        let parsed = value[1..value.len() - 1].to_string();
                        Ok(quote! { #parsed })
                    }
                }
            }
        }
        _ => {
            bail!("Found default for type which does not support default: {ros_type}");
        }
    }
}
