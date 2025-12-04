//! This module contains the top level Node and NodeHandle classes.
//! These wrap the lower level management of a ROS Node connection into a higher level and thread safe API.

use if_addrs::Interface;
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
    // If ROS_IP is set, that trumps anything else
    if let Ok(ip_str) = std::env::var("ROS_IP") {
        let ip = ip_str.parse().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!(
                "ROS_IP environment variable did not parse to a valid IpAddr::V4: {e:?}"
            ))
        })?;
        return Ok((ip, ip_str));
    }
    // If ROS_HOSTNAME is set, that is next highest precedent
    if let Ok(name) = std::env::var("ROS_HOSTNAME") {
        let ip = hostname_to_ipv4(&name).await?;
        return Ok((ip, name));
    }

    // If neither env var is set, use the computers "hostname"
    let name = gethostname::gethostname();
    let name = name.into_string().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!("This host's hostname is a string that cannot be validly converted into a Rust type, and therefore we cannot convert it into an IpAddrv4: {e:?}"))
        })?;

    // Try to find an IP in the same subnet as the ROS master
    if let Some(master_ip) = try_get_master_ip(master_uri) {
        if let Ok(local_interfaces) = if_addrs::get_if_addrs() {
            if let Some(ip) = try_find_addr_in_same_subnet(master_ip, &local_interfaces) {
                return Ok((ip, name));
            }
        }
    }

    // Fallback to just use the first ip we can find
    let ip = hostname_to_ipv4(&name).await?;
    Ok((ip, name))
}

fn try_find_addr_in_same_subnet(
    master_ip: Ipv4Addr,
    local_interfaces: &Vec<Interface>,
) -> Option<Ipv4Addr> {
    for iface in local_interfaces {
        if let if_addrs::IfAddr::V4(ifv4) = &iface.addr {
            if is_in_same_subnet(ifv4.ip, master_ip, ifv4.netmask) {
                return Some(ifv4.ip);
            }
        }
    }
    None
}

fn try_get_master_ip(master_uri: &str) -> Option<Ipv4Addr> {
    let s = master_uri
        .strip_prefix("http://")
        .or_else(|| master_uri.strip_prefix("https://"))
        .unwrap_or(master_uri);
    let host = s.split(':').next()?;
    host.parse::<Ipv4Addr>().ok()
}

fn is_in_same_subnet(ip1: Ipv4Addr, ip2: Ipv4Addr, mask: Ipv4Addr) -> bool {
    let ip1_octets = ip1.octets();
    let ip2_octets = ip2.octets();
    let mask_octets = mask.octets();

    for i in 0..4 {
        if (ip1_octets[i] & mask_octets[i]) != (ip2_octets[i] & mask_octets[i]) {
            return false;
        }
    }
    true
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
