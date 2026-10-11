//! Generate one field by independently selecting its Rust type, default, and wire codec.

use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use super::{literal::parse_literal, CodegenOptions};
use crate::parse::convert_ros_type_to_rust_type;
use crate::utils::RosVersion;
use crate::{bail, ArrayType, Error, FieldInfo};

/// Rust implements Default for arrays up to and including this length.
const MAX_DERIVED_ARRAY_LEN: usize = 32;

pub(super) struct GeneratedField {
    pub definition: TokenStream,
    pub codec: TokenStream,
    /// Field initializer for an explicit Default implementation, when needed.
    pub default: TokenStream,
    pub requires_default_impl: bool,
}

pub(super) fn generate_field_definition(
    field: FieldInfo,
    msg_pkg: &str,
    version: RosVersion,
    options: &CodegenOptions,
    struct_name: &syn::Ident,
) -> Result<GeneratedField, Error> {
    validate_field_default(&field)?;
    let rust_type = field_rust_type(&field, msg_pkg, version, "self")?;
    let codec_type = field_rust_type(&field, msg_pkg, version, "super")?;
    let default = field_default_expression(&field, version)?;
    let requires_default_impl = default.is_some();
    let default = default.unwrap_or_else(|| quote! { ::std::default::Default::default() });
    let adapter = FieldAdapter::select(&field, options);
    let (serde, codec) = field_serde_codec(&field, &codec_type, struct_name, adapter, options);
    let name = format_ident!("r#{}", field.field_name);

    Ok(GeneratedField {
        definition: quote! {
            #serde
            pub #name: #rust_type,
        },
        codec,
        default: quote! { #name: #default, },
        requires_default_impl,
    })
}

fn field_rust_type(
    field: &FieldInfo,
    msg_pkg: &str,
    version: RosVersion,
    local_module: &str,
) -> Result<TokenStream, Error> {
    let element = match &field.field_type.package_name {
        Some(package) if package == msg_pkg => {
            format!("{local_module}::{}", field.field_type.field_type)
        }
        Some(package) => format!("{package}::{}", field.field_type.field_type),
        None => convert_ros_type_to_rust_type(version, &field.field_type.field_type)
            .ok_or_else(|| Error::new(format!("No Rust type for {}", field.field_type)))?
            .to_owned(),
    };
    let element: TokenStream = element
        .parse()
        .map_err(|error| Error::with("Invalid Rust field type", error))?;
    Ok(match field.field_type.array_info {
        ArrayType::NotArray => element,
        ArrayType::FixedLength(length) => {
            let length = syn::LitInt::new(&length.to_string(), proc_macro2::Span::call_site());
            quote! { [#element; #length] }
        }
        ArrayType::Unbounded | ArrayType::Bounded(_) => quote! { ::std::vec::Vec<#element> },
    })
}

fn field_default_expression(
    field: &FieldInfo,
    version: RosVersion,
) -> Result<Option<TokenStream>, Error> {
    if let Some(default) = &field.default {
        let value = parse_literal(
            &field.field_type.field_type,
            default,
            &field.field_type.array_info,
            version,
        )?;
        // Constants use string literals; a scalar field owns its string instead.
        return Ok(Some(
            if field.field_type.field_type == "string"
                && matches!(field.field_type.array_info, ArrayType::NotArray)
            {
                quote! { ::std::string::String::from(#value) }
            } else {
                value
            },
        ));
    }
    if matches!(field.field_type.array_info, ArrayType::FixedLength(length) if length > MAX_DERIVED_ARRAY_LEN)
    {
        // Larger arrays need independent initialization, including non-Copy nested messages.
        return Ok(Some(quote! {
            ::std::array::from_fn(|_| ::std::default::Default::default())
        }));
    }
    Ok(None)
}

/// Select framing once, then use it for both ordinary and bounded fields.
enum FieldAdapter {
    Standard,
    RosBytes,
    SerdeBytes,
    FixedArray,
}

impl FieldAdapter {
    fn select(field: &FieldInfo, options: &CodegenOptions) -> Self {
        let array = &field.field_type.array_info;
        let bytes = matches!(
            field.field_type.field_type.as_str(),
            "uint8" | "byte" | "char"
        );
        if bytes && !matches!(array, ArrayType::NotArray) && options.roslibrust_serde {
            Self::RosBytes
        } else if matches!(array, ArrayType::FixedLength(_)) {
            Self::FixedArray
        } else if bytes && !matches!(array, ArrayType::NotArray) {
            Self::SerdeBytes
        } else {
            Self::Standard
        }
    }

    fn path(&self) -> TokenStream {
        match self {
            Self::Standard => quote! { ::roslibrust::codegen::field_codec::standard },
            Self::RosBytes => quote! { ::roslibrust::codegen::serde_rosmsg_bytes },
            Self::SerdeBytes => quote! { ::serde_bytes },
            Self::FixedArray => quote! { ::roslibrust::codegen::field_codec::fixed_array },
        }
    }
}

fn field_serde_codec(
    field: &FieldInfo,
    rust_type: &TokenStream,
    struct_name: &syn::Ident,
    adapter: FieldAdapter,
    options: &CodegenOptions,
) -> (TokenStream, TokenStream) {
    let has_bounds = matches!(field.field_type.array_info, ArrayType::Bounded(_))
        || field.field_type.string_capacity.is_some();
    if !has_bounds {
        let attribute = match adapter {
            FieldAdapter::Standard => quote! {},
            _ => {
                let path = adapter.path().to_string().replace(' ', "");
                quote! { #[serde(with = #path)] }
            }
        };
        return (attribute, quote! {});
    }

    let module = format_ident!("__roslibrust_{}_{}", struct_name, field.field_name);
    let module_path = module.to_string();
    let label = &field.field_name;
    let serde = if options.roslibrust_serde {
        quote! { ::roslibrust::codegen::serde }
    } else {
        quote! { ::serde }
    };
    let adapter = adapter.path();
    let sequence = match field.field_type.array_info {
        ArrayType::Bounded(limit) => quote! { #limit },
        _ => quote! { unbounded },
    };
    let strings = match (
        field.field_type.string_capacity,
        &field.field_type.array_info,
    ) {
        (Some(limit), ArrayType::NotArray) => quote! { scalar(#limit) },
        (Some(limit), _) => quote! { sequence(#limit) },
        (None, _) => quote! { unbounded },
    };
    (
        quote! { #[serde(with = #module_path)] },
        quote! {
            #[allow(non_snake_case, unused_imports, dead_code)]
            mod #module {
                use super::*;
                type Value = #rust_type;
                ::roslibrust::codegen::ros_field_codec!(
                    field = #label,
                    ty = Value,
                    serde = #serde,
                    adapter = #adapter,
                    sequence = #sequence,
                    strings = #strings,
                );
            }
        },
    )
}

/// Reject invalid declared bounds before they can become a generated Default implementation.
pub(super) fn validate_field_default(field: &FieldInfo) -> Result<(), Error> {
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
        // The literal parser validates the matching quotation marks.
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
