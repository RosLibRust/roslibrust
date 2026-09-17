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
//! - Ability to publish transforms via `add_transform()`
//!
//! # ROS1 vs ROS2
//!
//! The `TransformManager` is generic over the message type. Use the appropriate type alias
//! for your ROS version:
//!
//! - ROS1: `TransformManager::<Ros1TFMessage, _>::new(&ros, buffer_duration)`
//! - ROS2: `TransformManager::<Ros2TFMessage, _>::new(&ros, buffer_duration)`
//!
//! # Example
//! ```no_run
//! use roslibrust_transforms::{Quaternion, Ros1TFMessage, Stamp, Timestamp, Transform, TransformManager, Vector3};
//! use roslibrust::traits::Ros;
//!
//! // Generic over any roslibrust backend
//! async fn example<T: Ros>(ros: T)
//! {
//!     let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await.unwrap();
//!
//!     // Look up a transform
//!     let transform = manager.get_transform("base_link", "camera_link", Timestamp::now()).await.unwrap();
//!
//!     // Build an updated transform from its components
//!     let updated = Transform::new(
//!         transform.parent(),
//!         transform.child(),
//!         transform.translation() + Vector3::new(1.0, 0.0, 0.0),
//!         transform.rotation(),
//!         Stamp::At(Timestamp::now()),
//!     )
//!     .unwrap();
//!
//!     // Update the value in the buffer, and publish it's update to other nodes
//!     manager.add_transform(updated).await.unwrap();
//! }
//! ```

pub mod messages;

// Re-export useful types from the transforms crate
pub use transforms::errors::TransformError;
pub use transforms::geometry::{Quaternion, Vector3};
pub use transforms::time::{Stamp, TimeError, TimePoint, Timestamp};
pub use transforms::{Registry, Transform};

use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;

use roslibrust_common::{Publish, RosMessageType, Subscribe, TopicProvider};
use tokio::sync::{watch, RwLock};
use tokio_util::sync::CancellationToken;

/// Error types for TransformManager operations.
#[derive(thiserror::Error, Debug)]
pub enum TransformManagerError {
    #[error("Transform lookup failed: {0}")]
    LookupError(String),

    #[error("Transform rejected by the registry: {0}")]
    RejectedTransform(String),

    #[error("ROS communication error: {0}")]
    RosError(#[from] roslibrust_common::Error),

    #[error("Timeout waiting for transform from '{0}' to '{1}' at time '{2}'")]
    Timeout(String, String, String),
}

/// Conversion between ROS header timestamps and transform timestamps.
pub trait RosTimestamp: TimePoint {
    /// Convert ROS sec/nsec fields into this timestamp type.
    fn from_ros_time(sec: i32, nsec: u32) -> Self;

    /// Convert this timestamp type into ROS sec/nsec fields.
    fn as_ros_time(self) -> (i32, u32);
}

impl RosTimestamp for Timestamp {
    fn from_ros_time(sec: i32, nsec: u32) -> Self {
        // Timestamp stores u64 nanoseconds since the unix epoch, so times before the epoch
        // have no representation, clamp them to zero
        match u64::try_from(sec) {
            Ok(sec) => Timestamp::from_nanos(sec * 1_000_000_000 + (nsec as u64)),
            Err(_) => Timestamp::zero(),
        }
    }

    fn as_ros_time(self) -> (i32, u32) {
        let nanos = self.as_nanos();
        let secs = nanos / 1_000_000_000;
        let nsecs = nanos % 1_000_000_000;
        (secs as i32, nsecs as u32)
    }
}

impl RosTimestamp for std::time::SystemTime {
    fn from_ros_time(sec: i32, nsec: u32) -> Self {
        use std::time::UNIX_EPOCH;

        if sec >= 0 {
            UNIX_EPOCH + Duration::new(sec as u64, nsec)
        } else {
            UNIX_EPOCH
                .checked_sub(Duration::new((-sec) as u64, nsec))
                .unwrap_or(UNIX_EPOCH)
        }
    }

    fn as_ros_time(self) -> (i32, u32) {
        use std::time::UNIX_EPOCH;

        let duration = self.duration_since(UNIX_EPOCH).unwrap_or_default();
        (duration.as_secs() as i32, duration.subsec_nanos())
    }
}

/// Trait for converting a TransformStamped message to a `transforms::Transform`.
///
/// This trait abstracts over the differences between ROS1 and ROS2 TransformStamped messages.
pub trait IntoTransform<T = Timestamp>
where
    T: TimePoint,
{
    /// Convert this message into a `transforms::Transform`.
    ///
    /// If `is_static` is true, the message's timestamp is ignored and the resulting transform
    /// carries `Stamp::Static`, making it valid for all time.
    ///
    /// Returns an error if the message does not describe a valid transform, e.g. its rotation
    /// is not a unit quaternion or one of its components is NaN or infinite.
    fn into_transform(self, is_static: bool) -> Result<transforms::Transform<T>, TransformError>;
}

/// Trait for converting a `transforms::Transform` to a TransformStamped message.
///
/// This trait abstracts over the differences between ROS1 and ROS2 TransformStamped messages.
pub trait FromTransform<T = Timestamp>: Sized
where
    T: TimePoint,
{
    /// Create a TransformStamped message from a `transforms::Transform`.
    ///
    /// Static transforms carry no instant and are stamped with time zero in the resulting message.
    fn from_transform(transform: &transforms::Transform<T>) -> Self;
}

/// Trait for TFMessage types that contain a list of TransformStamped messages.
///
/// This trait abstracts over the differences between ROS1 and ROS2 TFMessage types.
pub trait TFMessageType<T = Timestamp>: RosMessageType + Send + Clone + 'static
where
    T: TimePoint,
{
    /// The TransformStamped type contained in this TFMessage.
    type TransformStamped: IntoTransform<T> + FromTransform<T> + Clone;

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
pub struct TransformManager<M, P, T = Timestamp>
where
    M: TFMessageType<T>,
    P: Publish<M> + Send + Sync,
    T: TimePoint,
{
    registry: Arc<RwLock<Registry<T>>>,
    buffer_duration: Duration,
    /// Watch channel to notify waiters when transforms are added
    transform_notify: watch::Sender<()>,
    /// Cancellation token to shut down background tasks when dropped
    cancel_token: CancellationToken,
    tf_publisher: P,
    tf_static_publisher: P,
    _phantom: PhantomData<M>,
}

impl<M, P, T> TransformManager<M, P, T>
where
    M: TFMessageType<T>,
    P: Publish<M> + Send + Sync,
    T: TimePoint + Send + Sync + 'static,
{
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
    pub async fn new<R>(
        ros: &R,
        buffer_duration: Duration,
    ) -> Result<TransformManager<M, R::Publisher<M>, T>, TransformManagerError>
    where
        R: TopicProvider<Publisher<M> = P> + Clone + Send + Sync + 'static,
        R::Subscriber<M>: Send + 'static,
        R::Publisher<M>: Send + Sync,
    {
        let registry = Arc::new(RwLock::new(Registry::<T>::with_max_age(buffer_duration)));

        // Create watch channel for notifying waiters when transforms are added
        // Notifications coalesce - receivers only care that something changed since they last checked
        let (transform_notify, _) = watch::channel(());

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
        registry: Arc<RwLock<Registry<T>>>,
        notify: watch::Sender<()>,
        cancel_token: CancellationToken,
        is_static: bool,
    ) {
        let topic = if is_static { "/tf_static" } else { "/tf" };
        loop {
            tokio::select! {
                _ = cancel_token.cancelled() => {
                    log::debug!("Shutting down {topic} listener task");
                    break;
                }
                result = subscriber.next() => {
                    match result {
                        Ok(msg) => {
                            let mut reg = registry.write().await;
                            for tf in <M as TFMessageType<T>>::transforms(msg) {
                                let transform =
                                    match <M::TransformStamped as IntoTransform<T>>::into_transform(
                                        tf, is_static,
                                    ) {
                                        Ok(transform) => transform,
                                        Err(e) => {
                                            log::warn!(
                                                "Dropping invalid transform received on {topic}: {e}"
                                            );
                                            continue;
                                        }
                                    };
                                // Clone the frame names so the warning below can still name them
                                // after the transform is moved into the registry
                                let parent = transform.parent().to_owned();
                                let child = transform.child().to_owned();
                                if let Err(e) = reg.add_transform(transform) {
                                    log::warn!(
                                        "Dropping transform from '{parent}' to '{child}' rejected by the registry on {topic}: {e}"
                                    );
                                }
                            }
                            // Notify waiters that transforms have been added
                            // Ignore errors - they just mean no one is currently listening
                            let _ = notify.send(());
                        }
                        Err(e) => {
                            log::warn!("Error receiving {topic} message: {e}");
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
    /// use roslibrust_transforms::{Quaternion, Ros1TFMessage, Stamp, Timestamp, Transform, TransformManager, Vector3};
    /// use roslibrust::traits::Ros;
    /// #[tokio::main]
    /// async fn main() -> Result<(), Box<dyn std::error::Error>> {
    ///     // Creating a fake ros instance for this example
    ///     let ros = roslibrust::mock::MockRos::new();
    ///     let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await?;
    ///
    ///     // Camera has moved between t=0 and t=5
    ///     // These updates would be automatically received over the /tf topic if something was publishing them
    ///     let t0 = Timestamp::now();
    ///     let x0 = Transform::new(
    ///         "base_link",
    ///         "camera_link",
    ///         Vector3::new(0.0, 0.0, 0.0),
    ///         Quaternion::identity(),
    ///         Stamp::At(t0),
    ///     )?;
    ///     manager.add_transform(x0).await?;
    ///
    ///     let t5 = (t0 + std::time::Duration::from_secs(5)).unwrap();
    ///     let x5 = Transform::new(
    ///         "base_link",
    ///         "camera_link",
    ///         Vector3::new(1.0, 0.0, 0.0),
    ///         Quaternion::identity(),
    ///         Stamp::At(t5),
    ///     )?;
    ///     manager.add_transform(x5).await?;
    ///
    ///     //  We care to know where the camera was at t=3
    ///     let t3 = (t0 + std::time::Duration::from_secs(3)).unwrap();
    ///     let transform = manager.get_transform("base_link", "camera_link", t3).await?;
    ///
    ///     // Linear interpolation was performed behind the scenes to get the transform at t=3
    ///     assert_eq!(transform.translation().x, 0.6);
    ///     Ok(())
    /// }
    pub async fn get_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        time: T,
    ) -> Result<transforms::Transform<T>, TransformManagerError> {
        let registry = self.registry.read().await;
        registry
            .get_transform(target_frame, source_frame, time)
            .map_err(|e| TransformManagerError::LookupError(e.to_string()))
    }

    fn pretty_print_timestamp(time: T) -> String {
        format!("{:.3}s", time.as_seconds_lossy())
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
        time: T,
        timeout: Option<Duration>,
    ) -> Result<transforms::Transform<T>, TransformManagerError> {
        let timeout_duration = timeout.unwrap_or(self.buffer_duration);
        let deadline = tokio::time::Instant::now() + timeout_duration;

        // Subscribe to transform notifications
        let mut receiver = self.transform_notify.subscribe();

        loop {
            // Try to get the transform
            {
                let registry = self.registry.read().await;
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
                    Self::pretty_print_timestamp(time),
                ));
            }

            // Wait for either the final deadline to occur, or for a notification that a transform has been added
            tokio::select! {
                _ = tokio::time::sleep(remaining) => {
                    // Timeout expired - do one final check then return error
                    let registry = self.registry.read().await;
                    if let Ok(transform) = registry.get_transform(target_frame, source_frame, time) {
                        return Ok(transform);
                    }
                    return Err(TransformManagerError::Timeout(
                        target_frame.to_string(),
                        source_frame.to_string(),
                        Self::pretty_print_timestamp(time),
                    ));
                }
                result = receiver.changed() => {
                    // Got a notification - check for the transform on next loop iteration
                    // Notifications coalesce, so a burst of transforms only triggers one check
                    if result.is_err() {
                        // Channel closed, shouldn't happen but treat as timeout
                        return Err(TransformManagerError::Timeout(
                            target_frame.to_string(),
                            source_frame.to_string(),
                            Self::pretty_print_timestamp(time),
                        ));
                    }
                }
            }
        }
    }

    /// Update (publish and add to registry) a transform.
    ///
    /// The transform's stamp picks the topic: dynamic transforms (`Stamp::At`, built with
    /// [Transform::new]) are published to the /tf topic, static transforms (`Stamp::Static`,
    /// built with [Transform::static_between]) are published to the /tf_static topic.
    pub async fn add_transform(
        &self,
        transform: transforms::Transform<T>,
    ) -> Result<(), TransformManagerError> {
        let transform_stamped =
            <M::TransformStamped as FromTransform<T>>::from_transform(&transform);
        let msg = <M as TFMessageType<T>>::from_transforms(vec![transform_stamped]);
        let is_static = transform.timestamp().is_static();

        // Update the registry first so that a transform it rejects (e.g. one that would
        // re-parent a frame) is never published
        {
            let mut registry = self.registry.write().await;
            registry
                .add_transform(transform)
                .map_err(|e| TransformManagerError::RejectedTransform(e.to_string()))?;
        }

        // Notify waiters that a transform has been added, even if the publish below fails
        let _ = self.transform_notify.send(());

        // Publish to the topic matching the transform's kind
        if is_static {
            self.tf_static_publisher.publish(&msg).await?;
        } else {
            self.tf_publisher.publish(&msg).await?;
        }

        Ok(())
    }

    /// Look up a transform between two frames at different times, using a fixed frame.
    ///
    /// This matches tf2's "time travel" lookup. The returned transform converts points
    /// from `source_frame` at `source_time` into `target_frame` at `target_time`.
    ///
    /// Note: this function is async to wait for access to registry, but does not wait for the transform to be available.
    pub async fn get_transform_at(
        &self,
        target_frame: &str,
        target_time: T,
        source_frame: &str,
        source_time: T,
        fixed_frame: &str,
    ) -> Result<transforms::Transform<T>, TransformManagerError> {
        let registry = self.registry.read().await;
        registry
            .get_transform_at(
                target_frame,
                target_time,
                source_frame,
                source_time,
                fixed_frame,
            )
            .map_err(|e| TransformManagerError::LookupError(e.to_string()))
    }

    /// Get the newest time at which a transform between two frames can be served.
    ///
    /// Returns `Stamp::At` with the newest time [Self::get_transform] can serve for the pair,
    /// or `Stamp::Static` if the frames are connected entirely by static transforms, in which
    /// case a transform is available at any time.
    ///
    /// Note: transforms keep arriving in the background, so the returned time is a snapshot.
    /// It can be stale by the time a follow-up [Self::get_transform] runs, and if buffer cleanup
    /// evicts it the follow-up lookup returns an error rather than a wrong transform.
    ///
    /// This is the equivalent of tf2's "latest available transform" lookup:
    /// ```
    /// use roslibrust_transforms::{Quaternion, Ros1TFMessage, Stamp, Timestamp, Transform, TransformManager, Vector3};
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// // Creating a fake ros instance for this example
    /// let ros = roslibrust::mock::MockRos::new();
    /// let manager = TransformManager::<Ros1TFMessage, _>::new(&ros, std::time::Duration::from_secs(10)).await?;
    ///
    /// let stamp = Timestamp::now();
    /// let transform = Transform::new(
    ///     "map",
    ///     "robot",
    ///     Vector3::new(1.0, 0.0, 0.0),
    ///     Quaternion::identity(),
    ///     Stamp::At(stamp),
    /// )?;
    /// manager.add_transform(transform).await?;
    ///
    /// // The newest time a lookup between the two frames can be served is the sample just added
    /// let latest = manager.latest_common_time("map", "robot").await?;
    /// assert_eq!(latest, Stamp::At(stamp));
    ///
    /// // Look up the transform at the newest available time
    /// let time = match latest {
    ///     Stamp::At(time) => time,
    ///     // Frames connected by static transforms only can be looked up at any time
    ///     Stamp::Static => Timestamp::now(),
    /// };
    /// let transform = manager.get_transform("map", "robot", time).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn latest_common_time(
        &self,
        target_frame: &str,
        source_frame: &str,
    ) -> Result<Stamp<T>, TransformManagerError> {
        let registry = self.registry.read().await;
        registry
            .latest_common_time(target_frame, source_frame)
            .map_err(|e| TransformManagerError::LookupError(e.to_string()))
    }

    /// Remove a frame and all of its transforms from the local buffer.
    ///
    /// The transforms crate does not support re-parenting: once a child frame is in the buffer,
    /// transforms for the same child frame under a different parent are rejected (the subscriber
    /// tasks log them as warnings). Removing the frame allows it to be re-added under its new
    /// parent. Note that removing a frame in the middle of the tree strands its descendants
    /// until the removed frame is received or added again.
    ///
    /// This only affects the local buffer, the tf buffers of other nodes are unaffected.
    ///
    /// Returns `true` if the frame existed.
    pub async fn remove_frame(&self, frame: &str) -> bool {
        let mut registry = self.registry.write().await;
        registry.remove_frame(frame)
    }
}

impl<M, P, T> Drop for TransformManager<M, P, T>
where
    M: TFMessageType<T>,
    P: Publish<M> + Send + Sync,
    T: TimePoint,
{
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

impl<T> TFMessageType<T> for Ros1TFMessage
where
    T: RosTimestamp,
{
    type TransformStamped = Ros1TransformStamped;

    fn transforms(self) -> Vec<Self::TransformStamped> {
        self.transforms
    }

    fn from_transforms(transforms: Vec<Self::TransformStamped>) -> Self {
        Ros1TFMessage { transforms }
    }
}

impl<T> IntoTransform<T> for Ros1TransformStamped
where
    T: RosTimestamp,
{
    fn into_transform(self, is_static: bool) -> Result<transforms::Transform<T>, TransformError> {
        let timestamp = if is_static {
            Stamp::Static
        } else {
            Stamp::At(T::from_ros_time(
                self.header.stamp.secs,
                self.header.stamp.nsecs as u32,
            ))
        };

        transforms::Transform::new(
            &self.header.frame_id,
            &self.child_frame_id,
            Vector3::new(
                self.transform.translation.x,
                self.transform.translation.y,
                self.transform.translation.z,
            ),
            Quaternion {
                w: self.transform.rotation.w,
                x: self.transform.rotation.x,
                y: self.transform.rotation.y,
                z: self.transform.rotation.z,
            },
            timestamp,
        )
    }
}

impl<T> FromTransform<T> for Ros1TransformStamped
where
    T: RosTimestamp,
{
    fn from_transform(transform: &transforms::Transform<T>) -> Self {
        use crate::messages::ros1::{geometry_msgs, std_msgs};

        // Static transforms carry no instant, stamp them with time zero on the wire
        let (secs, nsecs) = match transform.timestamp() {
            Stamp::At(time) => time.as_ros_time(),
            Stamp::Static => (0, 0),
        };
        if nsecs > i32::MAX as u32 {
            panic!("Timestamp overflow when converting to Ros1TransformStamped");
        }

        let nsecs = nsecs as i32;
        let translation = transform.translation();
        let rotation = transform.rotation();

        Ros1TransformStamped {
            header: std_msgs::Header {
                seq: 0,
                stamp: roslibrust::codegen::integral_types::Time { secs, nsecs },
                frame_id: transform.parent().to_string(),
            },
            child_frame_id: transform.child().to_string(),
            transform: geometry_msgs::Transform {
                translation: geometry_msgs::Vector3 {
                    x: translation.x,
                    y: translation.y,
                    z: translation.z,
                },
                rotation: geometry_msgs::Quaternion {
                    x: rotation.x,
                    y: rotation.y,
                    z: rotation.z,
                    w: rotation.w,
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

impl<T> TFMessageType<T> for Ros2TFMessage
where
    T: RosTimestamp,
{
    type TransformStamped = Ros2TransformStamped;

    fn transforms(self) -> Vec<Self::TransformStamped> {
        self.transforms
    }

    fn from_transforms(transforms: Vec<Self::TransformStamped>) -> Self {
        Ros2TFMessage { transforms }
    }
}

impl<T> IntoTransform<T> for Ros2TransformStamped
where
    T: RosTimestamp,
{
    fn into_transform(self, is_static: bool) -> Result<transforms::Transform<T>, TransformError> {
        let timestamp = if is_static {
            Stamp::Static
        } else {
            Stamp::At(T::from_ros_time(
                self.header.stamp.sec,
                self.header.stamp.nanosec,
            ))
        };

        transforms::Transform::new(
            &self.header.frame_id,
            &self.child_frame_id,
            Vector3::new(
                self.transform.translation.x,
                self.transform.translation.y,
                self.transform.translation.z,
            ),
            Quaternion {
                w: self.transform.rotation.w,
                x: self.transform.rotation.x,
                y: self.transform.rotation.y,
                z: self.transform.rotation.z,
            },
            timestamp,
        )
    }
}

impl<T> FromTransform<T> for Ros2TransformStamped
where
    T: RosTimestamp,
{
    fn from_transform(transform: &transforms::Transform<T>) -> Self {
        use crate::messages::ros2::{builtin_interfaces, geometry_msgs, std_msgs};

        // Static transforms carry no instant, stamp them with time zero on the wire
        let (sec, nanosec) = match transform.timestamp() {
            Stamp::At(time) => time.as_ros_time(),
            Stamp::Static => (0, 0),
        };

        let translation = transform.translation();
        let rotation = transform.rotation();

        Ros2TransformStamped {
            header: std_msgs::Header {
                stamp: builtin_interfaces::Time { sec, nanosec },
                frame_id: transform.parent().to_string(),
            },
            child_frame_id: transform.child().to_string(),
            transform: geometry_msgs::Transform {
                translation: geometry_msgs::Vector3 {
                    x: translation.x,
                    y: translation.y,
                    z: translation.z,
                },
                rotation: geometry_msgs::Quaternion {
                    x: rotation.x,
                    y: rotation.y,
                    z: rotation.z,
                    w: rotation.w,
                },
            },
        }
    }
}
