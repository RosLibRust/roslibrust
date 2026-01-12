//! This module contains the top level Node and NodeHandle classes.
//! These wrap the lower level management of a ROS Node connection into a higher level and thread safe API.

use log::*;
use roslibrust_common::Error;

use super::{names::InvalidNameError, RosMasterError};
use std::{
    io,
    net::{IpAddr, Ipv4Addr},
};

pub(crate) mod actor;
mod handle;
mod xmlrpc;
use actor::*;
use anyhow::anyhow;
pub use handle::NodeHandle;
use tokio::sync::{mpsc, oneshot};
use xmlrpc::*;

#[derive(Debug)]
pub struct ProtocolParams {
    pub hostname: String,
    pub protocol: String,
    pub port: u16,
}

// TODO at the end of the day I'd like to offer a builder pattern for configuration that allow manual setting of this or "ros idiomatic" behavior - Carter
/// Following ROS's idiomatic address rules uses ROS_HOSTNAME and ROS_IP to determine the address that server should be hosted at.
/// Returns both the resolved IpAddress of the host (used for actually opening the socket), and the String "hostname" which should
/// be used in the URI.
async fn determine_addr(master_uri: &str) -> Result<(Ipv4Addr, String), RosMasterError> {
    // Note: this is a little messy in the history of development of roslibrust
    // Originally we tried to be "more correct" than ROS and only bind a single local address to listen to for our socket.
    // ROS1 explicitly binds to 0.0.0.0 (see: https://docs.ros.org/en/noetic/api/roscpp/html/transport__tcp_8cpp_source.html) which uses INADDR_ANY
    // Previously this code was determining both the IP to bind to, and the hostname to use for resolving servers on this node (xmlrpc and TCPROS)
    // Now we are hard coding the bind to 0.0.0.0 and just using this function to resolve the hostname
    const IP_ADDR_ANY: Ipv4Addr = Ipv4Addr::new(0, 0, 0, 0);

    // If ROS_HOSTNAME is set, that is next highest precedent
    if let Ok(name) = std::env::var("ROS_HOSTNAME") {
        debug!("ROS_HOSTNAME is set to {name}, using that as hostname for this node");
        return Ok((IP_ADDR_ANY, name));
    }
    // If ROS_IP is set that is next
    if let Ok(ip_str) = std::env::var("ROS_IP") {
        let _ip: Ipv4Addr = ip_str.parse().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!(
                "ROS_IP environment variable did not parse to a valid IpAddr::V4: {e:?}"
            ))
        })?;
        debug!("ROS_IP is set, will use that as hostname for this node: {ip_str}");
        return Ok((IP_ADDR_ANY, ip_str));
    }

    // If neither env var is set, use the computers "hostname"
    let name = gethostname::gethostname();
    let name = name.into_string().map_err(|e| {
        RosMasterError::HostIpResolutionFailure(format!(
            "This host's hostname is a string that cannot be validly converted into a Rust string: {e:?}"
        ))
    })?;
    // Directly matching ROS's logic here: https://docs.ros.org/en/noetic/api/roscpp/html/network_8cpp_source.html
    // If the hostname has something in it, and it isn't localhost we use that
    if !name.is_empty() && name != "localhost" {
        debug!("ROS_HOSTNAME and ROS_IP are not set. Using this computer's hostname of {name} as hostname for this node");
        return Ok((IP_ADDR_ANY, name));
    }

    // Last bit of logic is looking for an interface with an IP in the same subnet as the ROS master
    // If we find one, our hostname will be the IP address of that interface
    // This will resolve loopback interfaces if the master is also on a loopback interface
    if let Some(master_ip) = try_get_master_ip(master_uri).await {
        debug!("Resolved ROS master IP from URI {master_uri} as {master_ip}");
        if let Some(ip) = try_find_addr_in_same_subnet(master_ip) {
            let ip_str = ip.to_string();
            debug!("Neither ROS_IP or ROS_HOSTNAME are set. Found {ip_str} to be an interface IP in the same subnet as the ROS master. Using that as the hostname for this node");
            return Ok((IP_ADDR_ANY, ip_str));
        }
    } else {
        debug!("Could not determine IP of ROS master from it's URI: {master_uri}");
    }

    // At this point I assume the use is having problems, and we should intervene to help them
    Err(RosMasterError::HostIpResolutionFailure(format!(
        "Could not determine a valid network name for this node. Check that one of ROS_IP, ROS_HOSTNAME or the computer's hostname resolve to a valid Ipv4 address"
    )))
}

/// Attempts to find the first interface on this system that is in the same subnet as the master_ip
/// Returns the ipv4 address of the interface if one is found
fn try_find_addr_in_same_subnet(master_ip: Ipv4Addr) -> Option<Ipv4Addr> {
    let local_interfaces = getifs::interfaces().ok()?;
    // For each interface visible on the system
    for iface in local_interfaces {
        // Try to get the ipv4 addresses for this interface
        let Ok(ipv4_addrs) = iface.ipv4_addrs() else {
            debug!("Interface {iface:?} has no ipv4 addresses, skipping");
            continue;
        };
        // Iterate thought the addresses and check if the master_ip is in the same subnet
        for iface_net in ipv4_addrs.iter() {
            if iface_net.contains(&master_ip) {
                debug!("Interface {iface:?} is in the same subnet as the ROS master");
                return Some(iface_net.addr());
            } else {
                debug!("Interface {iface:?} is not in the same subnet as the ROS master, skipping");
            }
        }
    }
    None
}

/// Attempts to determine the ipv4 address of the ROS master from it's uri
///
/// Strongly expects the format to be "http<s>://<hostname>:<port>"
///
/// If it is an IP address it will be parsed, if it is a hostname resolution will be attempted.
async fn try_get_master_ip(master_uri: &str) -> Option<Ipv4Addr> {
    let s = master_uri
        .strip_prefix("http://")
        .or_else(|| master_uri.strip_prefix("https://"))
        .unwrap_or(master_uri);
    let host = s.split(':').next()?;

    if let Ok(ip) = host.parse::<Ipv4Addr>() {
        return Some(ip);
    }

    if let Ok(ip) = hostname_to_ipv4(host).await {
        return Some(ip);
    }

    None
}

/// Given a the name of a host use's std::net::ToSocketAddrs to perform a DNS lookup and return the resulting IP address.
/// This function is intended to be used to determine the correct IP host the socket for the xmlrpc server on.
async fn hostname_to_ipv4(name: &str) -> Result<Ipv4Addr, RosMasterError> {
    let name_with_port = &format!("{name}:0");
    let i = tokio::net::lookup_host(name_with_port).await.map_err(|e| {
        RosMasterError::HostIpResolutionFailure(format!(
            "Failure while attempting to lookup ROS_HOSTNAME: {e:?}"
        ))
    })?;
    for addr in i {
        if let IpAddr::V4(ip) = addr.ip() {
            return Ok(ip);
        }
    }
    Err(RosMasterError::HostIpResolutionFailure(format!(
        "ROS_HOSTNAME resolved to no IPv4 addresses: {name:?}"
    )))
}

#[derive(thiserror::Error, Debug)]
pub enum NodeError {
    #[error(transparent)]
    RosMasterError(#[from] RosMasterError),
    #[error("connection closed")]
    ChannelClosedError,
    #[error(transparent)]
    InvalidName(#[from] InvalidNameError),
    #[error(transparent)]
    XmlRpcError(#[from] XmlRpcError),
    #[error(transparent)]
    IoError(#[from] io::Error),
}

impl From<oneshot::error::RecvError> for NodeError {
    fn from(_value: oneshot::error::RecvError) -> Self {
        NodeError::ChannelClosedError
    }
}

impl<T> From<mpsc::error::SendError<T>> for NodeError {
    fn from(_value: mpsc::error::SendError<T>) -> Self {
        Self::ChannelClosedError
    }
}

// TODO MAJOR: this is kinda messy
// but for historic development reasons (not having a design for errors)
// we produced two different error types for the two different impls (ros1, rosbridge)
// This allows fusing the two error types together so that TopicProider can work
// but we should just better design all the error handling
impl From<NodeError> for Error {
    fn from(value: NodeError) -> Self {
        match value {
            NodeError::RosMasterError(e) => Error::ServerError(e.to_string()),
            NodeError::ChannelClosedError => {
                Error::Unexpected(anyhow!("Channel closed, something was dropped?"))
            }
            NodeError::InvalidName(e) => Error::InvalidName(e.to_string()),
            NodeError::XmlRpcError(e) => Error::SerializationError(e.to_string()),
            NodeError::IoError(e) => Error::IoError(e),
        }
    }
}
