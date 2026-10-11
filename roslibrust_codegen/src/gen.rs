use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use std::str::FromStr;
use syn::parse_quote;

mod field;
mod literal;
use field::generate_field_definition;
use literal::parse_literal;

use crate::parse::convert_ros_type_to_rust_type;
use crate::utils::RosVersion;
use crate::{ArrayType, Error};
use crate::{ConstantInfo, MessageFile, ServiceFile};

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

fn derive_attrs(options: &CodegenOptions, requires_default_impl: bool) -> Vec<syn::Attribute> {
    let default = (!requires_default_impl).then(|| quote! { ::std::default::Default, });
    let mut attrs = if options.roslibrust_serde {
        vec![
            parse_quote! { #[derive(::roslibrust::codegen::Deserialize, ::roslibrust::codegen::Serialize, Debug, Clone, PartialEq, #default)] },
            parse_quote! { #[serde(crate = "::roslibrust::codegen::serde")] },
        ]
    } else {
        vec![
            parse_quote! { #[derive(::serde::Deserialize, ::serde::Serialize, Debug, Clone, PartialEq, #default)] },
        ]
    };
    attrs.push(parse_quote! { #[serde(deny_unknown_fields)] });
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
    let requires_default_impl = fields.iter().any(|field| field.requires_default_impl);
    let attrs = derive_attrs(options, requires_default_impl);
    let default_impl = if requires_default_impl {
        let defaults = fields.iter().map(|field| &field.default);
        quote! {
            impl ::std::default::Default for #struct_name {
                fn default() -> Self {
                    Self { #(#defaults)* }
                }
            }
        }
    } else {
        quote! {}
    };
    let field_definitions = fields.iter().map(|field| &field.definition);
    let field_codecs = fields.iter().map(|field| &field.codec);

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

    let mut base = quote! {
        #[allow(non_snake_case)]
        #[allow(dead_code)]
        #(#attrs )*
        pub struct #struct_name {
            #(#field_definitions)*
        }

        #default_impl

        #trait_impl

        #(#field_codecs)*
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
    let constant_value = parse_literal(
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

#[cfg(test)]
mod tests {
    use super::field::validate_field_default;
    use super::literal::parse_literal_value;
    use super::*;
    use crate::{FieldInfo, RosLiteral};

    #[test]
    fn fixed_array_defaults_reject_wrong_element_counts() {
        for (ros_type, value) in [
            ("uint8", "[1]"),
            ("string", "[\"one\", \"two\", \"three\"]"),
        ] {
            let error = parse_literal_value(
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
            let generated = generate_field_definition(
                field,
                "test",
                RosVersion::ROS2,
                &options,
                &format_ident!("Bounds"),
            )
            .unwrap();
            let helpers = generated.codec.to_string();
            if roslibrust_serde {
                assert!(helpers.contains("roslibrust :: codegen :: serde"));
            } else {
                assert!(helpers.contains("serde = :: serde"));
                assert!(!helpers.contains("serde = :: roslibrust"));
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
