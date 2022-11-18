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
            if let Ok(_) = input.parse::<Token![,]>() {
                continue;
            } else {
                break;
            }
        }
        Ok(Self { paths })
    }
}

/// Given a list of paths, generates struct definitions and trait impls for any
/// ros messages found within those paths.
/// Paths are relative to where rustc is being invoked from your mileage may vary.
#[proc_macro]
pub fn find_and_generate_ros_messages(input_stream: TokenStream) -> TokenStream {
    let RosLibRustMessagePaths { paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);
    roslibrust_codegen::find_and_generate_ros_messages(paths).into()
}

/// Does the same as find_and_generate_ros_messages, but interprets relative paths
/// as relative to the root of this crate. No idea if this is useful, but I needed it!
#[proc_macro]
pub fn find_and_generate_ros_messages_relative_to_manifest_dir(
    input_stream: TokenStream,
) -> TokenStream {
    let RosLibRustMessagePaths { mut paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);

    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).expect("Failed to set working dir");
    for path in &mut paths {
        *path = path.canonicalize().expect("Failed to canonicalize path");
    }

    roslibrust_codegen::find_and_generate_ros_messages(paths).into()
}
