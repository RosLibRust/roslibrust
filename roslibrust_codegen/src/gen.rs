use proc_macro2::TokenStream;
use quote::{format_ident, quote, ToTokens};
use serde::de::DeserializeOwned;
use std::str::FromStr;
use syn::parse_quote;

use crate::parse::convert_ros_type_to_rust_type;
use crate::utils::RosVersion;
use crate::{bail, ArrayType, Error};
use crate::{ConstantInfo, FieldInfo, MessageFile, RosLiteral, ServiceFile};

/// Configuration options for code generation
#[derive(Debug, Clone)]
pub struct CodegenOptions {
    /// Whether to include the DEFINITION field in generated code (default: true)
    /// The DEFINITION field can be quite large and may not be needed in all cases
    pub generate_definition: bool,
    /// Whether to use roslibrust's re-exported serde (default: true)
    pub roslibrust_serde: bool,
}

impl Default for CodegenOptions {
    fn default() -> Self {
        Self {
            generate_definition: true,
            roslibrust_serde: true,
        }
    }
}

fn derive_attrs(options: &CodegenOptions, _has_large_array: bool) -> Vec<syn::Attribute> {
    let mut attrs = vec![
        parse_quote! { #[derive(Debug)] },
        parse_quote! { #[derive(Clone)] },
        parse_quote! { #[derive(PartialEq)] },
    ];

    if options.roslibrust_serde {
        // Use roslibrust's re-exported serde with SmartDefault
        attrs.insert(
            0,
            parse_quote! { #[derive(::roslibrust::codegen::Deserialize)] },
        );
        attrs.insert(
            1,
            parse_quote! { #[derive(::roslibrust::codegen::Serialize)] },
        );
        attrs.insert(
            2,
            parse_quote! { #[derive(::roslibrust::codegen::SmartDefault)] },
        );
        attrs.push(parse_quote! { #[serde(crate = "::roslibrust::codegen::serde")] });
        attrs.push(parse_quote! { #[serde(deny_unknown_fields)] });
    } else {
        // Use standard Rust serde with SmartDefault (SmartDefault works with any serde)
        attrs.insert(0, parse_quote! { #[derive(serde::Deserialize)] });
        attrs.insert(1, parse_quote! { #[derive(serde::Serialize)] });
        attrs.insert(2, parse_quote! { #[derive(smart_default::SmartDefault)] });
        attrs.push(parse_quote! { #[serde(deny_unknown_fields)] });
    }

    attrs
}

/// Generates the service for a given service file
/// The service definition defines a struct representing the service an an implementation
/// of the RosServiceType trait for that struct
pub fn generate_service(
    service: ServiceFile,
    options: Option<&CodegenOptions>,
) -> Result<TokenStream, Error> {
    let default_options = CodegenOptions::default();
    let options = options.unwrap_or(&default_options);
    let service_type_name = service.get_full_name();
    let service_md5sum = service.md5sum;
    // Optional for now until we get all the hashing sorted out
    let service_ros2_hash = service.ros2_hash;
    let ros2_type_name = service.parsed.get_ros2_dds_type_name();
    let struct_name = format_ident!("{}", service.parsed.name);
    let request_name = format_ident!("{}", service.parsed.request_type.name);
    let response_name = format_ident!("{}", service.parsed.response_type.name);

    let request_msg = generate_struct(service.request, Some(options))?;
    let response_msg = generate_struct(service.response, Some(options))?;
    Ok(quote! {

        #request_msg
        #response_msg

        #[allow(dead_code)]
        pub struct #struct_name {

        }
        impl ::roslibrust::RosServiceType for #struct_name {
            const ROS_SERVICE_NAME: &'static str = #service_type_name;
            const MD5SUM: &'static str = #service_md5sum;
            const ROS2_HASH: &'static [u8; 32] = &#service_ros2_hash;
            const ROS2_TYPE_NAME: &'static str = #ros2_type_name;
            type Request = #request_name;
            type Response = #response_name;
        }
    })
}

/// Turns a string into a TokenStream that represents a raw string literal of the string
pub fn generate_raw_string_literal(value: &str) -> TokenStream {
    let wrapped = format!("r####\"{}\"####", value);
    TokenStream::from_str(&wrapped).unwrap()
}

pub fn generate_struct(
    msg: MessageFile,
    options: Option<&CodegenOptions>,
) -> Result<TokenStream, Error> {
    let default_options = CodegenOptions::default();
    let options = options.unwrap_or(&default_options);
    let ros_type_name = msg.get_full_name();
    let ros2_type_name = msg.parsed.get_ros2_dds_type_name();

    // Check if any field has a fixed array > 32 (which doesn't impl Default)
    let has_large_array = msg.parsed.fields.iter().any(
        |field| matches!(field.field_type.array_info, ArrayType::FixedLength(len) if len > 32),
    );

    let attrs = derive_attrs(options, has_large_array);
    let struct_name = format_ident!("{}", msg.parsed.name);
    let fields = msg
        .parsed
        .fields
        .into_iter()
        .map(|field| {
            generate_field_definition(
                field,
                &msg.parsed.package,
                msg.parsed.version.unwrap_or(RosVersion::ROS1),
                options,
                &struct_name,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let (fields, field_helpers): (Vec<_>, Vec<_>) = fields.into_iter().unzip();

    let constants = msg
        .parsed
        .constants
        .into_iter()
        .map(|constant| {
            generate_constant_field_definition(
                constant,
                msg.parsed.version.unwrap_or(RosVersion::ROS1),
            )
        })
        .collect::<Result<Vec<TokenStream>, _>>()?;

    let struct_name = format_ident!("{}", msg.parsed.name);
    let md5sum = msg.md5sum;
    let definition = msg.definition;
    let ros2_hash = msg.ros2_hash;

    let raw_message_definition = if options.generate_definition {
        let raw_message_definition = generate_raw_string_literal(&definition);
        quote! { #raw_message_definition }
    } else {
        quote! { "" }
    };
    let trait_impl = quote! {
        ::roslibrust::impl_ros_message_type!(
            #struct_name,
            #ros_type_name,
            #md5sum,
            #raw_message_definition,
            #ros2_type_name,
            &#ros2_hash,
        );
    };

    let field_helpers = if field_helpers.iter().any(|tokens| !tokens.is_empty()) {
        quote! { #[allow(dead_code, non_snake_case)] impl #struct_name { #(#field_helpers)* } }
    } else {
        quote! {}
    };
    let mut base = quote! {
        #[allow(non_snake_case)]
        #[allow(dead_code)]
        #(#attrs )*
        pub struct #struct_name {
            #(#fields )*
        }

        #trait_impl

        #field_helpers
    };

    // Only if we have constants append the impl
    if !constants.is_empty() {
        base.extend(quote! {
            #[allow(unused)]
            impl #struct_name {
                #(#constants )*
            }
        });
    }
    Ok(base)
}

fn generate_field_definition(
    field: FieldInfo,
    msg_pkg: &str,
    version: RosVersion,
    options: &CodegenOptions,
    struct_name: &syn::Ident,
) -> Result<(TokenStream, TokenStream), Error> {
    validate_field_default(&field)?;
    let rust_field_type = match field.field_type.package_name {
        Some(ref pkg) => {
            if pkg.as_str() == msg_pkg {
                format!("self::{}", field.field_type.field_type)
            } else {
                format!("{}::{}", pkg, field.field_type.field_type)
            }
        }
        None => convert_ros_type_to_rust_type(version, &field.field_type.field_type)
            .ok_or(Error::new(format!("No Rust type for {}", field.field_type)))?
            .to_owned(),
    };
    // Wrap type in appropriate Vec or array wrapper based on array information
    let rust_field_type = match field.field_type.array_info {
        ArrayType::Unbounded => {
            format!("::std::vec::Vec<{rust_field_type}>")
        }
        ArrayType::FixedLength(fixed_length) => format!("[{rust_field_type}; {fixed_length}]"),
        ArrayType::NotArray => rust_field_type,
        ArrayType::Bounded(_) => {
            format!("::std::vec::Vec<{rust_field_type}>")
        }
    };
    let rust_field_type = TokenStream::from_str(rust_field_type.as_str()).expect(
        "Somehow we generate a rust type that isn't valid rust syntax. This should not happen!",
    );

    let field_name = format_ident!("r#{}", field.field_name);
    let property_line = quote! { pub #field_name: #rust_field_type, };

    // SmartDefault attributes are needed regardless of generate_serde setting
    let default_line = if let Some(ref default_val) = field.default {
        let default_val = ros_literal_to_rust_literal(
            &field.field_type.field_type,
            default_val,
            &field.field_type.array_info,
            version,
        )?;
        if matches!(
            field.field_type.array_info,
            ArrayType::Unbounded | ArrayType::Bounded(_) | ArrayType::FixedLength(_)
        ) {
            // Array and vector defaults need an expression through SmartDefault's _code.
            quote! {
                #[default(_code = #default_val)]
            }
        } else {
            // Scalar defaults use SmartDefault's constant style.
            quote! {
              #[default(#default_val)]
            }
        }
    } else {
        // Arrays longer than 32 do not implement Default. Initialize each element
        // independently so this also works for String and nested message types.
        if let ArrayType::FixedLength(fixed_array_length) = field.field_type.array_info {
            if fixed_array_length > 32 {
                let default_str = "::std::array::from_fn(|_| ::std::default::Default::default())";
                quote! { #[default(_code = #default_str)]}
            } else {
                quote! {}
            }
        } else {
            quote! {}
        }
    };
    // Rust's derive macros automatically implement traits for arrays up to length 32.
    // For larger arrays, we need special handling via BigArray.
    const MAX_FIXED_ARRAY_LEN: usize = 32;

    let is_byte_field = matches!(
        field.field_type.field_type.as_str(),
        "uint8" | "byte" | "char"
    );

    let mut serde_line = match &field.field_type.array_info {
        // Dynamic-length arrays (Vec<T>)
        ArrayType::Unbounded | ArrayType::Bounded(_) => {
            if is_byte_field {
                if options.roslibrust_serde {
                    // Use roslibrust's custom module that handles both base64 (rosbridge) and binary
                    quote! { #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")] }
                } else {
                    // Use standard serde_bytes for efficient binary serialization
                    quote! { #[serde(with = "serde_bytes")] }
                }
            } else {
                quote! {}
            }
        }
        // Fixed-length arrays larger than 32 need BigArray, except for byte arrays handled by
        // roslibrust's adapter.
        ArrayType::FixedLength(len) if *len > MAX_FIXED_ARRAY_LEN => {
            if is_byte_field && options.roslibrust_serde {
                quote! { #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")] }
            } else {
                quote! { #[serde(with = "::roslibrust::codegen::BigArray")] }
            }
        }
        // Fixed-length arrays <= 32 have automatic trait implementations
        ArrayType::FixedLength(_) => {
            if is_byte_field && options.roslibrust_serde {
                quote! { #[serde(with = "::roslibrust::codegen::serde_rosmsg_bytes")] }
            } else if is_byte_field {
                // Use serde_bytes for efficient serialization of byte arrays
                quote! { #[serde(with = "serde_bytes")] }
            } else {
                quote! {}
            }
        }
        // Non-array types need no special serde attributes
        ArrayType::NotArray => quote! {},
    };
    let mut helpers = quote! {};
    if matches!(field.field_type.array_info, ArrayType::Bounded(_))
        || field.field_type.string_capacity.is_some()
    {
        let serde = if options.roslibrust_serde {
            quote! { ::roslibrust::codegen::serde }
        } else {
            quote! { ::serde }
        };
        let serialize_name = format_ident!("__roslibrust_serialize_{}", field.field_name);
        let deserialize_name = format_ident!("__roslibrust_deserialize_{}", field.field_name);
        let serialize_path = format!("{struct_name}::{serialize_name}");
        let deserialize_path = format!("{struct_name}::{deserialize_name}");
        serde_line = quote! {
            #[serde(serialize_with = #serialize_path, deserialize_with = #deserialize_path)]
        };
        let label = &field.field_name;
        let array_check = if let ArrayType::Bounded(limit) = field.field_type.array_info {
            quote! {
                if value.len() > #limit {
                    return Err(E::custom(format!("field `{}` exceeds maximum sequence length {} (got {})", #label, #limit, value.len())));
                }
            }
        } else {
            quote! {}
        };
        let string_check = if let Some(limit) = field.field_type.string_capacity {
            if matches!(field.field_type.array_info, ArrayType::NotArray) {
                quote! {
                    if value.len() > #limit {
                        return Err(E::custom(format!("field `{}` exceeds maximum string length {} bytes (got {})", #label, #limit, value.len())));
                    }
                }
            } else {
                quote! {
                    for (index, string) in value.iter().enumerate() {
                        if string.len() > #limit {
                            return Err(E::custom(format!("field `{}[{}]` exceeds maximum string length {} bytes (got {})", #label, index, #limit, string.len())));
                        }
                    }
                }
            }
        } else {
            quote! {}
        };
        let check_name = format_ident!("__roslibrust_check_{}", field.field_name);
        // Compose validation with the original adapter so byte vectors retain their length
        // prefix, fixed arrays retain tuple framing, and rosbridge still uses base64.
        let (serialize, deserialize) = if is_byte_field {
            let adapter = if options.roslibrust_serde {
                quote! { ::roslibrust::codegen::serde_rosmsg_bytes }
            } else {
                quote! { ::serde_bytes }
            };
            (
                quote! { #adapter::serialize(value, serializer) },
                quote! { #adapter::deserialize(deserializer) },
            )
        } else if matches!(field.field_type.array_info, ArrayType::FixedLength(n) if n > MAX_FIXED_ARRAY_LEN)
        {
            (
                quote! { ::roslibrust::codegen::BigArray::serialize(value, serializer) },
                quote! { ::roslibrust::codegen::BigArray::deserialize(deserializer) },
            )
        } else {
            (
                quote! { #serde::Serialize::serialize(value, serializer) },
                quote! { #serde::Deserialize::deserialize(deserializer) },
            )
        };
        helpers = quote! {
            fn #check_name<E: #serde::ser::Error>(value: &#rust_field_type) -> Result<(), E> {
                #array_check
                #string_check
                Ok(())
            }
            fn #serialize_name<S: #serde::Serializer>(value: &#rust_field_type, serializer: S) -> Result<S::Ok, S::Error> {
                Self::#check_name::<S::Error>(value)?;
                #serialize
            }
            fn #deserialize_name<'de, D: #serde::Deserializer<'de>>(deserializer: D) -> Result<#rust_field_type, D::Error> {
                let value: #rust_field_type = #deserialize?;
                // Deserialize errors do not implement ser::Error; reuse the same validator
                // with a neutral error and translate it into the caller's error type.
                Self::#check_name::<#serde::de::value::Error>(&value)
                    .map_err(<D::Error as #serde::de::Error>::custom)?;
                Ok(value)
            }
        };
    }
    Ok((
        quote! {
            #default_line
            #serde_line
            #property_line
        },
        helpers,
    ))
}

/// Reject invalid declared bounds before they can become a generated Default implementation.
fn validate_field_default(field: &FieldInfo) -> Result<(), Error> {
    if !matches!(field.field_type.array_info, ArrayType::Bounded(_))
        && field.field_type.string_capacity.is_none()
    {
        return Ok(());
    }
    let Some(default) = &field.default else {
        return Ok(());
    };
    let is_array = !matches!(field.field_type.array_info, ArrayType::NotArray);
    if is_array {
        let literal = if field.field_type.field_type == "bool" {
            default.inner.to_lowercase()
        } else {
            default.inner.clone()
        };
        let values: Vec<serde_json::Value> = serde_json::from_str(&literal)
            .map_err(|error| Error::with("invalid array default", error))?;
        if let ArrayType::Bounded(limit) = field.field_type.array_info {
            if values.len() > limit {
                bail!(
                    "default for field `{}` exceeds maximum sequence length {limit}",
                    field.field_name
                );
            }
        }
        if let Some(limit) = field.field_type.string_capacity {
            for (index, value) in values.iter().enumerate() {
                if value.as_str().is_some_and(|value| value.len() > limit) {
                    bail!("default for field `{}[{index}]` exceeds maximum string length {limit} bytes", field.field_name);
                }
            }
        }
    } else if let Some(limit) = field.field_type.string_capacity {
        let value = default.inner.trim();
        // The literal parser below validates the matching quotation marks.
        if value.len() >= 2
            && value
                .get(1..value.len() - 1)
                .is_some_and(|value| value.len() > limit)
        {
            bail!(
                "default for field `{}` exceeds maximum string length {limit} bytes",
                field.field_name
            );
        }
    }
    Ok(())
}

fn generate_constant_field_definition(
    constant: ConstantInfo,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    let constant_name = format_ident!("r#{}", constant.constant_name);
    let constant_rust_type = convert_ros_constant_type_to_rust_type(version, &constant)?;
    let constant_rust_type = TokenStream::from_str(constant_rust_type).map_err(|err| {
        Error::with(
            format!("Failed to parse {constant_rust_type} into valid rust syntax").as_str(),
            err,
        )
    })?;
    let constant_value = ros_literal_to_rust_literal(
        ros_type_for_constant_literal(&constant.constant_type),
        &constant.constant_value,
        &ArrayType::NotArray,
        version,
    )?;
    Ok(quote! { pub const #constant_name: #constant_rust_type = #constant_value; })
}

fn convert_ros_constant_type_to_rust_type(
    version: RosVersion,
    constant: &ConstantInfo,
) -> Result<&'static str, Error> {
    if constant.constant_type == "&'static str" {
        return Ok("&'static str");
    }

    convert_ros_type_to_rust_type(version, &constant.constant_type)
        .map(|rust_type| {
            if rust_type == "::std::string::String" {
                "&'static str"
            } else {
                rust_type
            }
        })
        .ok_or(Error::new(format!(
            "A constant was detected {constant:?} for which no valid rust type was found."
        )))
}

fn ros_type_for_constant_literal(constant_type: &str) -> &str {
    match constant_type {
        "&'static str" => "string",
        _ => constant_type,
    }
}

pub fn generate_mod(
    pkg_name: String,
    struct_definitions: Vec<TokenStream>,
    all_pkgs: &[String],
) -> TokenStream {
    let mod_name = format_ident!("{}", &pkg_name);
    let all_pkgs = all_pkgs
        .iter()
        .filter(|item| item.as_str() != pkg_name.as_str())
        .map(|pkg| format_ident!("{}", pkg))
        .collect::<Vec<_>>();

    quote! {
        #[allow(unused_imports)]
        pub mod #mod_name {
            #(use super::#all_pkgs; )*

            #(#struct_definitions )*
        }
    }
}

fn ros_literal_to_rust_literal(
    ros_type: &str,
    literal: &RosLiteral,
    array_info: &ArrayType,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    // TODO: The naming of all the functions under this tree seems inaccurate
    parse_ros_value(ros_type, &literal.inner, array_info, version)
}

// Converts a ROS string to a literal value
// Not intended to be called directly, but only via parse_ros_value.
// Wraps a serde_json deserialize call with our style of error handling.
fn generic_parse_value<T: DeserializeOwned + ToTokens>(
    value: &str,
    array_info: &ArrayType,
) -> Result<TokenStream, Error> {
    if !matches!(array_info, ArrayType::NotArray) {
        let parsed: Vec<T> = serde_json::from_str(value).map_err(|e|
            Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>()).as_str(), e)
        )?;
        generate_list_default(
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

/// SmartDefault's _code attribute takes a string containing the initializer expression.
fn generate_list_default(
    values: Vec<TokenStream>,
    array_info: &ArrayType,
) -> Result<TokenStream, Error> {
    let expression = match array_info {
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
    };
    let code = expression.to_string();
    Ok(quote! { #code })
}

/// For a given, which is either a ROS constant or default, parse the constant and convert it into a rust TokenStream
/// which represents the same literal value. This handles frustrating edge cases that are not well documented features
/// in either ROS1 or ROS2 such as:
/// - `f32[] MY_CONST_ARRAY=[0, 1]` we have to convert the value of the constant to `vec![0.0, 1.0]`
/// - `string[] names_field ['first', "second"]` we have to convert the default value to `vec!["first".to_string(), "second".to_string()]
/// Note: I could find NO documentation from ROS about what was actually legal or expected for these value expressions
/// Note: ROS says "strings are not escaped". No idea how the eff I'm actually supposed to handle that so likely bugs...
/// Note: No idea of "constant arrays" are intended to be supported in ROS...
/// `ros_type` -- Expects the string key of the determined rust type to hold the value. Should come from one of the type map constants.
/// `value` -- Expects the trimmed string containing only the value expression
/// `array_info` -- Determines whether list defaults initialize a fixed array or a vector.
fn parse_ros_value(
    ros_type: &str,
    value: &str,
    array_info: &ArrayType,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    let is_list = !matches!(array_info, ArrayType::NotArray);
    match ros_type {
        "bool" => generic_parse_value::<bool>(&value.to_lowercase(), array_info),
        "float64" => generic_parse_value::<f64>(value, array_info),
        "float32" => generic_parse_value::<f32>(value, array_info),
        "uint8" | "char" | "byte" => generic_parse_value::<u8>(value, array_info),
        "int8" => generic_parse_value::<i8>(value, array_info),
        "uint16" => generic_parse_value::<u16>(value, array_info),
        "int16" => generic_parse_value::<i16>(value, array_info),
        "uint32" => generic_parse_value::<u32>(value, array_info),
        "int32" => generic_parse_value::<i32>(value, array_info),
        "uint64" => generic_parse_value::<u64>(value, array_info),
        "int64" => generic_parse_value::<i64>(value, array_info),
        "string" => {
            // String is a special case because of quotes and to_string()
            if is_list {
                // TODO there is a bug here, no idea how I should be attempting to convert / escape single quotes here...
                let parsed: Vec<String> = serde_json::from_str(value).map_err(|e|
                    Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to Vec<String>").as_str(), e)
                )?;
                let values = parsed
                    .iter()
                    .map(|value| quote! { ::std::string::String::from(#value) })
                    .collect();
                generate_list_default(values, array_info)
            } else {
                match version {
                    RosVersion::ROS1 => {
                        // For ROS1 then entire contents except for leading and trailing whitespace are used
                        let value = value.trim();
                        Ok(quote! { #value })
                    }
                    RosVersion::ROS2 => {
                        // For ROS2 value must be in quotes, and either single or double quotes are okay
                        // Strings are not escaped (we think)
                        let value = value.trim();
                        if value.len() < 2 {
                            // TODO would like to provide source file and callsite information for debug, but pretty hard to
                            // Maybe we wrap that in calling function?
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_array_defaults_reject_wrong_element_counts() {
        for (ros_type, value) in [
            ("uint8", "[1]"),
            ("string", "[\"one\", \"two\", \"three\"]"),
        ] {
            let error = parse_ros_value(
                ros_type,
                value,
                &ArrayType::FixedLength(2),
                RosVersion::ROS2,
            )
            .unwrap_err();
            assert!(error.to_string().contains("expected 2"));
        }
    }

    #[test]
    fn defaults_respect_sequence_and_decoded_string_bounds() {
        let field = |array_info, string_capacity, default: &str| FieldInfo {
            field_type: crate::FieldType {
                field_type: "string".into(),
                package_name: None,
                source_package: "test".into(),
                array_info,
                string_capacity,
            },
            field_name: "labels".into(),
            default: Some(RosLiteral::from(default.to_owned())),
        };
        assert!(
            validate_field_default(&field(ArrayType::Bounded(1), None, r#"["a", "b"]"#)).is_err()
        );
        assert!(
            validate_field_default(&field(ArrayType::Bounded(2), Some(2), r#"["a", "é"]"#)).is_ok()
        );
        assert!(validate_field_default(&field(ArrayType::Unbounded, Some(1), r#"["é"]"#)).is_err());
        assert!(validate_field_default(&field(
            ArrayType::FixedLength(1),
            Some(1),
            r#"["\u0061"]"#
        ))
        .is_ok());
        assert!(validate_field_default(&field(ArrayType::NotArray, Some(5), "'12345'")).is_ok());
        assert!(validate_field_default(&field(ArrayType::NotArray, Some(5), "'123456'")).is_err());
    }

    #[test]
    fn bounded_boolean_defaults_keep_case_normalization() {
        let field = FieldInfo {
            field_type: crate::FieldType {
                field_type: "bool".into(),
                package_name: None,
                source_package: "test".into(),
                array_info: ArrayType::Bounded(2),
                string_capacity: None,
            },
            field_name: "flags".into(),
            default: Some(RosLiteral::from("[TRUE,False]".to_owned())),
        };
        assert!(generate_field_definition(
            field,
            "test",
            RosVersion::ROS2,
            &CodegenOptions::default(),
            &format_ident!("Flags")
        )
        .is_ok());
    }

    #[test]
    fn bounds_helpers_follow_selected_serde_import_mode() {
        for roslibrust_serde in [true, false] {
            let options = CodegenOptions {
                roslibrust_serde,
                ..Default::default()
            };
            let field = FieldInfo {
                field_type: crate::FieldType {
                    field_type: "string".into(),
                    package_name: None,
                    source_package: "test".into(),
                    array_info: ArrayType::Bounded(3),
                    string_capacity: Some(5),
                },
                field_name: "labels".into(),
                default: None,
            };
            let (_, helpers) = generate_field_definition(
                field,
                "test",
                RosVersion::ROS2,
                &options,
                &format_ident!("Bounds"),
            )
            .unwrap();
            let helpers = helpers.to_string();
            if roslibrust_serde {
                assert!(helpers.contains("roslibrust :: codegen :: serde"));
            } else {
                assert!(!helpers.contains(":: roslibrust ::"));
                assert!(helpers.contains(":: serde :: de :: value :: Error"));
            }
        }
    }

    #[test_log::test]
    fn string_constants_generate_static_str_type() {
        let constant = ConstantInfo {
            constant_type: "string".to_owned(),
            constant_name: "RUNNING_STATE".to_owned(),
            constant_value: RosLiteral::from("RUNNING".to_owned()),
        };

        let generated = generate_constant_field_definition(constant, RosVersion::ROS1)
            .unwrap()
            .to_string();

        assert!(generated.contains("RUNNING_STATE"));
        assert!(generated.contains("&'static str"));
        assert!(generated.contains("= \"RUNNING\""));
    }
}
