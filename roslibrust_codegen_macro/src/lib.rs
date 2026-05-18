use proc_macro::TokenStream;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Token};

struct RosLibRustMessagePaths {
    paths: Vec<std::path::PathBuf>,
}

/// Parses a comma-separated list of str literals specifying paths.
impl Parse for RosLibRustMessagePaths {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut paths = vec![];
        while let Ok(path) = input.parse::<syn::LitStr>() {
            paths.push(path.value().into());
            if input.parse::<Token![,]>().is_ok() {
                continue;
            } else {
                break;
            }
        }
        Ok(Self { paths })
    }
}

fn generate_with_environment(input_stream: TokenStream) -> TokenStream {
    let RosLibRustMessagePaths { paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);
    match roslibrust_codegen::find_and_generate_ros_messages(paths) {
        Ok((source, _dependent_paths)) => source.into(),
        Err(e) => {
            let error_msg = e.to_string();
            quote::quote!(compile_error!(#error_msg);).into()
        }
    }
}

fn generate_from_paths_only(input_stream: TokenStream) -> TokenStream {
    let RosLibRustMessagePaths { paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);
    match roslibrust_codegen::find_and_generate_ros_messages_without_ros_package_path(paths) {
        // Note: there is not currently a way for proc_macros to indicate that they need to be re-generated
        // We discard the "dependent_paths" part of the response here...
        Ok((source, _dependent_paths)) => source.into(),
        Err(e) => {
            let error_msg = e.to_string();
            quote::quote!( compile_error!(#error_msg); ).into()
        }
    }
}

/// Generates struct definitions and trait impls for ROS types found in the
/// paths provided to the macro, skipping ROS environment discovery.
#[proc_macro]
pub fn generate_ros_types(input_stream: TokenStream) -> TokenStream {
    // Note: there is not currently a way for proc_macros to indicate that they need to be re-generated
    // We discard the "dependent_paths" part of the response here...
    generate_from_paths_only(input_stream)
}

/// Generates struct definitions and trait impls for ROS types found in the
/// paths provided to the macro plus ROS1 paths from `ROS_PACKAGE_PATH` and ROS2
/// packages found through ament resource indexes in `AMENT_PREFIX_PATH` and
/// `COLCON_PREFIX_PATH`.
#[proc_macro]
pub fn generate_ros_types_with_env(input_stream: TokenStream) -> TokenStream {
    // Note: there is not currently a way for proc_macros to indicate that they need to be re-generated
    // We discard the "dependent_paths" part of the response here...
    generate_with_environment(input_stream)
}

/// Deprecated alias for [`generate_ros_types_with_env`].
#[deprecated(note = "use generate_ros_types_with_env! instead")]
#[proc_macro]
pub fn find_and_generate_ros_messages(input_stream: TokenStream) -> TokenStream {
    generate_with_environment(input_stream)
}

/// Deprecated alias for [`generate_ros_types`].
#[deprecated(note = "use generate_ros_types! instead")]
#[proc_macro]
pub fn find_and_generate_ros_messages_without_ros_package_path(
    input_stream: TokenStream,
) -> TokenStream {
    generate_from_paths_only(input_stream)
}
