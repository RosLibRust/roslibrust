//! Transform manager implementation.
//!
//! This module provides a TransformManager that subscribes to /tf and /tf_static
//! topics, maintains a transform buffer, and can publish transforms.
//! The manager is generic over the TFMessage type, allowing it to work with both
//! ROS1 and ROS2 message formats.

use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;

use roslibrust_common::{Publish, RosMessageType, Subscribe, TopicProvider};
use tokio::sync::RwLock;
use transforms::geometry::{Quaternion, Vector3};
use transforms::time::Timestamp;
use transforms::Registry;

/// Error types for TransformManager operations.
#[derive(thiserror::Error, Debug)]
pub enum TransformManagerError {
    #[error("Failed to subscribe to topic: {0}")]
    SubscriptionError(String),

    #[error("Failed to advertise topic: {0}")]
    AdvertiseError(String),

    #[error("Failed to publish transform: {0}")]
    PublishError(String),

    #[error("Transform lookup failed: {0}")]
    LookupError(String),

    #[error("ROS communication error: {0}")]
    RosError(#[from] roslibrust_common::Error),
}

/// Trait for converting a TransformStamped message to a `transforms::Transform`.
///
/// This trait abstracts over the differences between ROS1 and ROS2 TransformStamped messages.
pub trait IntoTransform {
    /// Convert this message into a `transforms::Transform`.
    ///
    /// If `is_static` is true, the timestamp should be set to `Timestamp::zero()`.
    fn into_transform(self, is_static: bool) -> transforms::Transform;
}

/// Trait for converting a `transforms::Transform` to a TransformStamped message.
///
/// This trait abstracts over the differences between ROS1 and ROS2 TransformStamped messages.
pub trait FromTransform: Sized {
    /// Create a TransformStamped message from a `transforms::Transform`.
    fn from_transform(transform: &transforms::Transform) -> Self;
}

/// Trait for TFMessage types that contain a list of TransformStamped messages.
///
/// This trait abstracts over the differences between ROS1 and ROS2 TFMessage types.
pub trait TFMessageType: RosMessageType + Send + Clone + 'static {
    /// The TransformStamped type contained in this TFMessage.
    type TransformStamped: IntoTransform + FromTransform + Clone;

    /// Get the transforms from this message.
    fn transforms(self) -> Vec<Self::TransformStamped>;

    /// Create a TFMessage from a list of TransformStamped messages.
    fn from_transforms(transforms: Vec<Self::TransformStamped>) -> Self;
}

/// A manager that subscribes to /tf and /tf_static topics, maintains a transform buffer,
/// and can publish transforms.
///
/// This is the primary interface for getting and setting transforms between coordinate frames.
/// It is generic over:
/// - `M`: The TFMessage type (e.g., ROS1 or ROS2 TFMessage)
/// - `P`: The publisher type (inferred from the TopicProvider used to create the manager)
///
/// The manager works with any roslibrust backend (ros1, rosbridge, zenoh, mock).
pub struct TransformManager<M: TFMessageType, P: Publish<M> + Send + Sync> {
    registry: Arc<RwLock<Registry>>,
    tf_publisher: P,
    tf_static_publisher: P,
    // We hold onto the task handles to keep them alive
    _tf_task: tokio::task::JoinHandle<()>,
    _tf_static_task: tokio::task::JoinHandle<()>,
    _phantom: PhantomData<M>,
}

impl<M: TFMessageType, P: Publish<M> + Send + Sync> TransformManager<M, P> {
    /// Create a new TransformManager that subscribes to /tf and /tf_static and can publish transforms.
    ///
    /// Uses a default buffer duration of 10 seconds.
    pub async fn new<T>(
        ros: &T,
    ) -> Result<TransformManager<M, T::Publisher<M>>, TransformManagerError>
    where
        T: TopicProvider<Publisher<M> = P> + Clone + Send + Sync + 'static,
        T::Subscriber<M>: Send + 'static,
        T::Publisher<M>: Send + Sync,
    {
        Self::with_buffer_duration(ros, Duration::from_secs(10)).await
    }

    /// Create a new TransformManager with a custom buffer duration.
    pub async fn with_buffer_duration<T>(
        ros: &T,
        buffer_duration: Duration,
    ) -> Result<TransformManager<M, T::Publisher<M>>, TransformManagerError>
    where
        T: TopicProvider<Publisher<M> = P> + Clone + Send + Sync + 'static,
        T::Subscriber<M>: Send + 'static,
        T::Publisher<M>: Send + Sync,
    {
        let registry = Arc::new(RwLock::new(Registry::new(buffer_duration)));

        // Subscribe to /tf topic
        let tf_subscriber = ros
            .subscribe::<M>("/tf")
            .await
            .map_err(|e| TransformManagerError::SubscriptionError(e.to_string()))?;

        // Subscribe to /tf_static topic
        let tf_static_subscriber = ros
            .subscribe::<M>("/tf_static")
            .await
            .map_err(|e| TransformManagerError::SubscriptionError(e.to_string()))?;

        // Advertise on /tf topic
        let tf_publisher = ros
            .advertise::<M>("/tf")
            .await
            .map_err(|e| TransformManagerError::AdvertiseError(e.to_string()))?;

        // Advertise on /tf_static topic
        let tf_static_publisher = ros
            .advertise::<M>("/tf_static")
            .await
            .map_err(|e| TransformManagerError::AdvertiseError(e.to_string()))?;

        // Spawn task to handle /tf messages
        let registry_clone = registry.clone();
        let tf_task = tokio::spawn(async move {
            Self::process_tf_messages(tf_subscriber, registry_clone, false).await;
        });

        // Spawn task to handle /tf_static messages
        let registry_clone = registry.clone();
        let tf_static_task = tokio::spawn(async move {
            Self::process_tf_messages(tf_static_subscriber, registry_clone, true).await;
        });

        Ok(TransformManager {
            registry,
            tf_publisher,
            tf_static_publisher,
            _tf_task: tf_task,
            _tf_static_task: tf_static_task,
            _phantom: PhantomData,
        })
    }

    /// Background tokio task to process incoming TF messages.
    async fn process_tf_messages<S: Subscribe<M>>(
        mut subscriber: S,
        registry: Arc<RwLock<Registry>>,
        is_static: bool,
    ) {
        loop {
            match subscriber.next().await {
                Ok(msg) => {
                    let mut reg = registry.write().await;
                    for tf in msg.transforms() {
                        let transform = tf.into_transform(is_static);
                        reg.add_transform(transform);
                    }
                }
                Err(e) => {
                    log::warn!(
                        "Error receiving {} message: {}",
                        if is_static { "/tf_static" } else { "/tf" },
                        e
                    );
                    // Continue trying to receive messages
                }
            }
        }
    }

    /// Look up a transform between two frames at a specific time.
    ///
    /// Returns the transform that converts points from `source_frame` to `target_frame`.
    pub async fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        time: Timestamp,
    ) -> Result<transforms::Transform, TransformManagerError> {
        let mut registry = self.registry.write().await;
        registry
            .get_transform(target_frame, source_frame, time)
            .map_err(|e| TransformManagerError::LookupError(e.to_string()))
    }

    /// Look up the latest available transform between two frames.
    pub async fn lookup_latest_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
    ) -> Result<transforms::Transform, TransformManagerError> {
        self.lookup_transform(target_frame, source_frame, Timestamp::now())
            .await
    }

    /// Check if a transform is available between two frames.
    pub async fn can_transform(&self, target_frame: &str, source_frame: &str) -> bool {
        self.lookup_latest_transform(target_frame, source_frame)
            .await
            .is_ok()
    }

    /// Get direct access to the underlying registry for advanced operations.
    pub fn registry(&self) -> Arc<RwLock<Registry>> {
        self.registry.clone()
    }

    /// Update (publish and add to registry) a dynamic transform.
    ///
    /// This publishes the transform to the /tf topic and adds it to the local registry.
    pub async fn update_transform(
        &self,
        transform: transforms::Transform,
    ) -> Result<(), TransformManagerError> {
        let transform_stamped = M::TransformStamped::from_transform(&transform);
        let msg = M::from_transforms(vec![transform_stamped]);

        // Publish to /tf
        self.tf_publisher
            .publish(&msg)
            .await
            .map_err(|e| TransformManagerError::PublishError(e.to_string()))?;

        // Update registry
        let mut registry = self.registry.write().await;
        registry.add_transform(transform);

        Ok(())
    }

    /// Update (publish and add to registry) a static transform.
    ///
    /// This publishes the transform to the /tf_static topic and adds it to the local registry
    /// with a timestamp of zero (which the transforms crate treats as a static transform).
    pub async fn update_static_transform(
        &self,
        mut transform: transforms::Transform,
    ) -> Result<(), TransformManagerError> {
        // Static transforms use timestamp zero
        transform.timestamp = Timestamp::zero();

        let transform_stamped = M::TransformStamped::from_transform(&transform);
        let msg = M::from_transforms(vec![transform_stamped]);

        // Publish to /tf_static
        self.tf_static_publisher
            .publish(&msg)
            .await
            .map_err(|e| TransformManagerError::PublishError(e.to_string()))?;

        // Update registry
        let mut registry = self.registry.write().await;
        registry.add_transform(transform);

        Ok(())
    }
}

// =============================================================================
// ROS1 Implementation
// =============================================================================

/// ROS1 TFMessage type alias for convenience.
pub type Ros1TFMessage = crate::messages::ros1::TFMessage;

/// ROS1 TransformStamped type alias for convenience.
pub type Ros1TransformStamped = crate::messages::ros1::geometry_msgs::TransformStamped;

impl TFMessageType for Ros1TFMessage {
    type TransformStamped = Ros1TransformStamped;

    fn transforms(self) -> Vec<Self::TransformStamped> {
        self.transforms
    }

    fn from_transforms(transforms: Vec<Self::TransformStamped>) -> Self {
        Ros1TFMessage { transforms }
    }
}

impl IntoTransform for Ros1TransformStamped {
    fn into_transform(self, is_static: bool) -> transforms::Transform {
        let timestamp = if is_static {
            Timestamp::zero()
        } else {
            let nanoseconds = (self.header.stamp.secs as u128) * 1_000_000_000
                + (self.header.stamp.nsecs as u128);
            Timestamp { t: nanoseconds }
        };

        transforms::Transform {
            translation: Vector3::new(
                self.transform.translation.x,
                self.transform.translation.y,
                self.transform.translation.z,
            ),
            rotation: Quaternion {
                w: self.transform.rotation.w,
                x: self.transform.rotation.x,
                y: self.transform.rotation.y,
                z: self.transform.rotation.z,
            },
            timestamp,
            parent: self.header.frame_id,
            child: self.child_frame_id,
        }
    }
}

impl FromTransform for Ros1TransformStamped {
    fn from_transform(transform: &transforms::Transform) -> Self {
        use crate::messages::ros1::{geometry_msgs, std_msgs};

        // Year 2038 problem anyone?
        let secs = transform.timestamp.t / 1_000_000_000;
        let nsecs = transform.timestamp.t % 1_000_000_000;
        if secs > i32::MAX as u128 || nsecs > i32::MAX as u128 {
            panic!("Timestamp overflow when converting to Ros1TransformStamped");
        }

        let secs = secs as i32;
        let nsecs = nsecs as i32;

        Ros1TransformStamped {
            header: std_msgs::Header {
                seq: 0,
                stamp: roslibrust::codegen::integral_types::Time { secs, nsecs },
                frame_id: transform.parent.clone(),
            },
            child_frame_id: transform.child.clone(),
            transform: geometry_msgs::Transform {
                translation: geometry_msgs::Vector3 {
                    x: transform.translation.x,
                    y: transform.translation.y,
                    z: transform.translation.z,
                },
                rotation: geometry_msgs::Quaternion {
                    x: transform.rotation.x,
                    y: transform.rotation.y,
                    z: transform.rotation.z,
                    w: transform.rotation.w,
                },
            },
        }
    }
}

// =============================================================================
// ROS2 Implementation
// =============================================================================

/// ROS2 TFMessage type alias for convenience.
pub type Ros2TFMessage = crate::messages::ros2::TFMessage;

/// ROS2 TransformStamped type alias for convenience.
pub type Ros2TransformStamped = crate::messages::ros2::geometry_msgs::TransformStamped;

impl TFMessageType for Ros2TFMessage {
    type TransformStamped = Ros2TransformStamped;

    fn transforms(self) -> Vec<Self::TransformStamped> {
        self.transforms
    }

    fn from_transforms(transforms: Vec<Self::TransformStamped>) -> Self {
        Ros2TFMessage { transforms }
    }
}

impl IntoTransform for Ros2TransformStamped {
    fn into_transform(self, is_static: bool) -> transforms::Transform {
        let timestamp = if is_static {
            Timestamp::zero()
        } else {
            let nanoseconds = (self.header.stamp.sec as u128) * 1_000_000_000
                + (self.header.stamp.nanosec as u128);
            Timestamp { t: nanoseconds }
        };

        transforms::Transform {
            translation: Vector3::new(
                self.transform.translation.x,
                self.transform.translation.y,
                self.transform.translation.z,
            ),
            rotation: Quaternion {
                w: self.transform.rotation.w,
                x: self.transform.rotation.x,
                y: self.transform.rotation.y,
                z: self.transform.rotation.z,
            },
            timestamp,
            parent: self.header.frame_id,
            child: self.child_frame_id,
        }
    }
}

impl FromTransform for Ros2TransformStamped {
    fn from_transform(transform: &transforms::Transform) -> Self {
        use crate::messages::ros2::{builtin_interfaces, geometry_msgs, std_msgs};

        let sec = (transform.timestamp.t / 1_000_000_000) as i32;
        let nanosec = (transform.timestamp.t % 1_000_000_000) as u32;

        Ros2TransformStamped {
            header: std_msgs::Header {
                stamp: builtin_interfaces::Time { sec, nanosec },
                frame_id: transform.parent.clone(),
            },
            child_frame_id: transform.child.clone(),
            transform: geometry_msgs::Transform {
                translation: geometry_msgs::Vector3 {
                    x: transform.translation.x,
                    y: transform.translation.y,
                    z: transform.translation.z,
                },
                rotation: geometry_msgs::Quaternion {
                    x: transform.rotation.x,
                    y: transform.rotation.y,
                    z: transform.rotation.z,
                    w: transform.rotation.w,
                },
            },
        }
    }
}
