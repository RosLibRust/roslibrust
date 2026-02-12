//! A tf2-like transform library for roslibrust.
//!
//! This crate provides a `TransformManager` that subscribes to `/tf` and `/tf_static` topics,
//! maintains a transform buffer using the `transforms` crate, and can publish transforms.
//!
//! # Features
//!
//! - Generic over roslibrust backends (ros1, rosbridge, zenoh, mock)
//! - Supports both ROS1 and ROS2 message formats
//! - Automatic subscription to `/tf` and `/tf_static` topics
//! - Ability to publish transforms via `update_transform()` and `update_static_transform()`
//!
//! # ROS1 vs ROS2
//!
//! The `TransformManager` is generic over the message type. Use the appropriate type alias
//! for your ROS version:
//!
//! - ROS1: `TransformManager::<Ros1TFMessage, _>::new(&ros)`
//! - ROS2: `TransformManager::<Ros2TFMessage, _>::new(&ros)`
//!
//! # Example
//! ```no_run
//! use roslibrust_transforms::{TransformManager, Ros1TFMessage, Timestamp};
//! use roslibrust::traits::Ros;
//!
//! // Generic over any roslibrust backend
//! async fn example<T: Ros>(ros: T)
//! {
//!     let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await.unwrap();
//!
//!     // Look up a transform
//!     let mut transform = manager.get_transform("base_link", "camera_link", Timestamp::now()).await.unwrap();
//!
//!     // Modify the transform
//!     transform.translation.x += 1.0;
//!     transform.timestamp = transforms::time::Timestamp::now();
//!
//!     // Update the value in the buffer, and publish it's update to other nodes
//!     manager.add_transform(transform).await.unwrap();
//! }
//! ```

pub mod messages;

// Re-export useful types from the transforms crate
pub use transforms::geometry::{Quaternion, Transform, Vector3};
pub use transforms::time::Timestamp;
pub use transforms::Registry;

use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;

use roslibrust_common::{Publish, RosMessageType, Subscribe, TopicProvider};
use tokio::sync::{broadcast, RwLock};
use tokio_util::sync::CancellationToken;

/// Error types for TransformManager operations.
#[derive(thiserror::Error, Debug)]
pub enum TransformManagerError {
    #[error("Transform lookup failed: {0}")]
    LookupError(String),

    #[error("ROS communication error: {0}")]
    RosError(#[from] roslibrust_common::Error),

    #[error("Timeout waiting for transform from '{0}' to '{1}'")]
    Timeout(String, String),
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
/// - `M`: The TFMessage type, either [Ros1TFMessage] or [Ros2TFMessage]
/// - `P`: The publisher type (inferred from the TopicProvider used to create the manager)
///
/// The manager works with any roslibrust backend (ros1, rosbridge, zenoh, mock).
pub struct TransformManager<M: TFMessageType, P: Publish<M> + Send + Sync> {
    registry: Arc<RwLock<Registry>>,
    buffer_duration: Duration,
    /// Broadcast channel to notify waiters when transforms are added
    transform_notify: broadcast::Sender<()>,
    /// Cancellation token to shut down background tasks when dropped
    cancel_token: CancellationToken,
    tf_publisher: P,
    tf_static_publisher: P,
    _phantom: PhantomData<M>,
}

impl<M: TFMessageType, P: Publish<M> + Send + Sync> TransformManager<M, P> {
    /// Create a new TransformManager with a custom buffer duration.
    ///
    /// Typical usage:
    /// ```no_run
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// // Create a roslibrust backend of your choice
    /// let node = roslibrust::ros1::NodeHandle::new("http://localhost:11311", "my_node").await?;
    /// // Create a Transform manager and specify what message type you expect to receive
    /// let manager = roslibrust_transforms::TransformManager::<roslibrust_transforms::Ros1TFMessage, _>::new(&node, std::time::Duration::from_secs(10)).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn new<T>(
        ros: &T,
        buffer_duration: Duration,
    ) -> Result<TransformManager<M, T::Publisher<M>>, TransformManagerError>
    where
        T: TopicProvider<Publisher<M> = P> + Clone + Send + Sync + 'static,
        T::Subscriber<M>: Send + 'static,
        T::Publisher<M>: Send + Sync,
    {
        let registry = Arc::new(RwLock::new(Registry::new(buffer_duration)));

        // Create broadcast channel for notifying waiters when transforms are added
        // Capacity of 16 should be plenty - receivers only care about the most recent notification
        let (transform_notify, _) = broadcast::channel(16);

        // Create cancellation token for shutting down background tasks
        let cancel_token = CancellationToken::new();

        // Subscribe to /tf topic
        let tf_subscriber = ros.subscribe::<M>("/tf").await?;

        // Subscribe to /tf_static topic
        let tf_static_subscriber = ros.subscribe::<M>("/tf_static").await?;

        // Advertise on /tf topic
        let tf_publisher = ros.advertise::<M>("/tf").await?;

        // Advertise on /tf_static topic
        let tf_static_publisher = ros.advertise::<M>("/tf_static").await?;

        // Spawn task to handle /tf messages
        let registry_clone = registry.clone();
        let notify_clone = transform_notify.clone();
        let cancel_clone = cancel_token.clone();
        tokio::spawn(async move {
            Self::process_tf_messages(
                tf_subscriber,
                registry_clone,
                notify_clone,
                cancel_clone,
                false,
            )
            .await;
        });

        // Spawn task to handle /tf_static messages
        let registry_clone = registry.clone();
        let notify_clone = transform_notify.clone();
        let cancel_clone = cancel_token.clone();
        tokio::spawn(async move {
            Self::process_tf_messages(
                tf_static_subscriber,
                registry_clone,
                notify_clone,
                cancel_clone,
                true,
            )
            .await;
        });

        Ok(TransformManager {
            registry,
            buffer_duration,
            transform_notify,
            cancel_token,
            tf_publisher,
            tf_static_publisher,
            _phantom: PhantomData,
        })
    }

    /// Background tokio task to process incoming TF messages.
    async fn process_tf_messages<S: Subscribe<M>>(
        mut subscriber: S,
        registry: Arc<RwLock<Registry>>,
        notify: broadcast::Sender<()>,
        cancel_token: CancellationToken,
        is_static: bool,
    ) {
        loop {
            tokio::select! {
                _ = cancel_token.cancelled() => {
                    log::debug!(
                        "Shutting down {} listener task",
                        if is_static { "/tf_static" } else { "/tf" }
                    );
                    break;
                }
                result = subscriber.next() => {
                    match result {
                        Ok(msg) => {
                            let mut reg = registry.write().await;
                            for tf in msg.transforms() {
                                let transform = tf.into_transform(is_static);
                                reg.add_transform(transform);
                            }
                            // Notify waiters that transforms have been added
                            // Ignore errors - they just mean no one is currently listening
                            let _ = notify.send(());
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
        }
    }

    /// Look up a transform between two frames at a specific time.
    /// Note: this function is async to wait for access to registry, but does not wait for the transform to be available.
    ///
    /// Returns the transform that converts points from `source_frame` to `target_frame`.
    ///
    /// Example:
    /// ```
    /// use roslibrust_transforms::{TransformManager, Ros1TFMessage};
    /// use roslibrust::traits::Ros;
    /// #[tokio::main]
    /// async fn main() -> Result<(), Box<dyn std::error::Error>> {
    ///     // Creating a fake ros instance for this example
    ///     let ros = roslibrust::mock::MockRos::new();
    ///     let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await?;
    ///
    ///     // Camera has moved between t=0 and t=5
    ///     // These updates would be automatically received over the /tf topic if something was publishing them
    ///     let t0 = roslibrust_transforms::Timestamp::now();
    ///     let x0 = transforms::Transform {
    ///         parent: "base_link".to_string(),
    ///         child: "camera_link".to_string(),
    ///         translation: roslibrust_transforms::Vector3::new(0.0, 0.0, 0.0),
    ///         rotation: roslibrust_transforms::Quaternion::identity(),
    ///         timestamp: t0,
    ///     };
    ///     manager.add_transform(x0).await?;
    ///
    ///     let t5 = (t0 + std::time::Duration::from_secs(5)).unwrap();
    ///     let x5 = transforms::Transform {
    ///         parent: "base_link".to_string(),
    ///         child: "camera_link".to_string(),
    ///         translation: roslibrust_transforms::Vector3::new(1.0, 0.0, 0.0),
    ///         rotation: roslibrust_transforms::Quaternion::identity(),
    ///         timestamp: t5,
    ///     };
    ///     manager.add_transform(x5).await?;
    ///
    ///     //  We care to know where the camera was at t=3
    ///     let t3 = (t0 + std::time::Duration::from_secs(3)).unwrap();
    ///     let transform = manager.get_transform("base_link", "camera_link", t3).await?;
    ///
    ///     // Linear interpolation was performed behind the scenes to get the transform at t=3
    ///     assert_eq!(transform.translation.x, 0.6);
    ///     Ok(())
    /// }
    pub async fn get_transform(
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

    /// Wait for a transform to become available between two frames at a specific time.
    ///
    /// This method will poll the registry until the transform is available or until the timeout
    /// is reached. If `timeout` is `None`, the method will use the buffer duration configured
    /// in the constructor as the timeout.
    ///
    /// # Arguments
    ///
    /// * `target_frame` - The frame to transform into
    /// * `source_frame` - The frame to transform from
    /// * `time` - The timestamp for which the transform is requested
    /// * `timeout` - Optional timeout duration. If `None`, uses the buffer duration from the constructor.
    ///
    /// # Returns
    ///
    /// Returns the transform that converts points from `source_frame` to `target_frame`,
    /// or a `Timeout` error if the transform is not available within the timeout period.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use roslibrust_transforms::{TransformManager, Ros1TFMessage, Timestamp};
    /// use std::time::Duration;
    ///
    /// async fn example(manager: &TransformManager<Ros1TFMessage, impl roslibrust_common::Publish<Ros1TFMessage> + Send + Sync>) {
    ///     // Wait up to 5 seconds for the transform
    ///     let transform = manager.wait_for_transform(
    ///         "base_link",
    ///         "camera_link",
    ///         Timestamp::now(),
    ///         Some(Duration::from_secs(5))
    ///     ).await.unwrap();
    ///
    ///     // Or use the default timeout (buffer duration)
    ///     let transform = manager.wait_for_transform(
    ///         "base_link",
    ///         "camera_link",
    ///         Timestamp::now(),
    ///         None
    ///     ).await.unwrap();
    /// }
    /// ```
    pub async fn wait_for_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        time: Timestamp,
        timeout: Option<Duration>,
    ) -> Result<transforms::Transform, TransformManagerError> {
        let timeout_duration = timeout.unwrap_or(self.buffer_duration);
        let deadline = tokio::time::Instant::now() + timeout_duration;

        // Subscribe to transform notifications
        let mut receiver = self.transform_notify.subscribe();

        loop {
            // Try to get the transform
            {
                let mut registry = self.registry.write().await;
                if let Ok(transform) = registry.get_transform(target_frame, source_frame, time) {
                    return Ok(transform);
                }
            }

            // Wait for either a notification or timeout
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                return Err(TransformManagerError::Timeout(
                    target_frame.to_string(),
                    source_frame.to_string(),
                ));
            }

            // Wait for either the final deadline to occur, or for a notification that a transform has been added
            tokio::select! {
                _ = tokio::time::sleep(remaining) => {
                    // Timeout expired - do one final check then return error
                    let mut registry = self.registry.write().await;
                    if let Ok(transform) = registry.get_transform(target_frame, source_frame, time) {
                        return Ok(transform);
                    }
                    return Err(TransformManagerError::Timeout(
                        target_frame.to_string(),
                        source_frame.to_string(),
                    ));
                }
                result = receiver.recv() => {
                    // Got a notification - check for the transform on next loop iteration
                    // Handle lagged receivers by just continuing - we'll check the registry anyway
                    match result {
                        Ok(()) | Err(broadcast::error::RecvError::Lagged(_)) => {
                            // Continue to next iteration to check for transform
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            // Channel closed, shouldn't happen but treat as timeout
                            return Err(TransformManagerError::Timeout(
                                target_frame.to_string(),
                                source_frame.to_string(),
                            ));
                        }
                    }
                }
            }
        }
    }

    /// Update (publish and add to registry) a dynamic transform.
    ///
    /// This publishes the transform to the /tf topic and adds it to the local registry.
    pub async fn add_transform(
        &self,
        transform: transforms::Transform,
    ) -> Result<(), TransformManagerError> {
        let transform_stamped = M::TransformStamped::from_transform(&transform);
        let msg = M::from_transforms(vec![transform_stamped]);

        // Publish to /tf
        self.tf_publisher.publish(&msg).await?;

        // Update registry
        {
            let mut registry = self.registry.write().await;
            registry.add_transform(transform);
        }

        // Notify waiters that a transform has been added
        let _ = self.transform_notify.send(());

        Ok(())
    }

    /// Update (publish and add to registry) a static transform.
    ///
    /// This publishes the transform to the /tf_static topic and adds it to the local registry
    /// with a timestamp of zero (which the transforms crate treats as a static transform).
    /// If the timestamp is not zero, it will be overwritten with zero.
    ///
    /// This function is equivalent to calling [Self::update_transform] with a timestamp of zero, but
    /// provided as an additional function for clarity.
    pub async fn update_static_transform(
        &self,
        mut transform: transforms::Transform,
    ) -> Result<(), TransformManagerError> {
        // Static transforms use timestamp zero
        transform.timestamp = Timestamp::zero();

        let transform_stamped = M::TransformStamped::from_transform(&transform);
        let msg = M::from_transforms(vec![transform_stamped]);

        // Publish to /tf_static
        self.tf_static_publisher.publish(&msg).await?;

        // Update registry
        let mut registry = self.registry.write().await;
        registry.add_transform(transform);
        drop(registry);

        // Notify waiters that a transform has been added
        let _ = self.transform_notify.send(());

        Ok(())
    }
}

impl<M: TFMessageType, P: Publish<M> + Send + Sync> Drop for TransformManager<M, P> {
    fn drop(&mut self) {
        // Cancel the background tasks when the manager is dropped
        self.cancel_token.cancel();
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

        // Year 2038 problem anyone?
        let sec = transform.timestamp.t / 1_000_000_000;
        let nanosec = transform.timestamp.t % 1_000_000_000;
        if sec > i32::MAX as u128 || nanosec > u32::MAX as u128 {
            panic!("Timestamp overflow when converting to Ros2TransformStamped");
        }
        let sec = sec as i32;
        let nanosec = nanosec as u32;

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
