use ros_z::context::ZContext;
use ros_z::context::ZContextBuilder;
use ros_z::Builder;

pub fn make_test_context() -> ZContext {
    ZContextBuilder::default()
        .with_domain_id(0)
        .with_connect_endpoints(["tcp/[::]:7447"])
        .build()
        .unwrap()
}
