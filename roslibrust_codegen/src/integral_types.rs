use simple_error::{bail, SimpleError};

use roslibrust_common::RosMessageType;

/// Matches the integral ros1 type time, with extensions for ease of use
/// NOTE: in ROS1 "Time" is not a message in and of itself and std_msgs/Time should be used.
/// However, in ROS2 "Time" is a message and part of builtin_interfaces/Time.
// Okay some complexities lurk here that I really don't like
// In ROS1 time is i32 secs and i32 nsecs
// In ROS2 time is i32 secs and u32 nsecs
// How many nsecs are there in a sec? +1e9 which will fit inside of either.
// But ROS really doesn't declare what is valid for nsecs larger than 1e9, how should that be handled?
// How should negative nsecs work anyway?
// https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Time.html
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Time {
    // Note: rosbridge appears to accept secs and nsecs in for time without issue?
    // Not sure we should actually rely on this behavior, but ok for now...

    // This alias is required for ros2 where field has been renamed
    #[serde(alias = "sec")]
    pub secs: i32,
    // This alias is required for ros2 where field has been renamed
    #[serde(alias = "nanosec")]
    pub nsecs: i32,
}

/// Provide a standard conversion between ROS time and std::time::SystemTime
impl TryFrom<std::time::SystemTime> for Time {
    type Error = SimpleError;
    fn try_from(val: std::time::SystemTime) -> Result<Self, Self::Error> {
        let delta = match val.duration_since(std::time::UNIX_EPOCH) {
            Ok(delta) => delta,
            Err(e) => bail!("Failed to convert system time into unix epoch: {}", e),
        };
        // TODO our current method doesn't try to handel negative times
        // It is unclear from ROS documentation how these would be generated or how they should be handled
        // For now adopting a strict conversion policy of only converting when it makes clear logical sense
        let downcast_secs = match i32::try_from(delta.as_secs()) {
            Ok(val) => val,
            Err(e) => bail!("Failed to convert seconds to i32: {e:?}"),
        };
        let downcast_nanos = match i32::try_from(delta.subsec_nanos()) {
            Ok(val) => val,
            Err(e) => bail!("Failed to convert nanoseconds to i32: {e:?}"),
        };
        Ok(Time {
            secs: downcast_secs,
            nsecs: downcast_nanos,
        })
    }
}

/// Provide a standard conversion between ROS time and std::time::SystemTime
impl TryFrom<Time> for std::time::SystemTime {
    type Error = SimpleError;
    fn try_from(val: Time) -> Result<Self, Self::Error> {
        // TODO our current method doesn't try to handel negative times
        // It is unclear from ROS documentation how these would be generated or how they should be handled
        // For now adopting a strict conversion policy of only converting when it makes clear logical sense
        let secs = match u64::try_from(val.secs){
            Ok(val) => val,
            Err(e) => bail!( "Failed to convert ROS time to std::time::SystemTime, secs term overflows u64 likely: {val:?}, {e:?}"),
        };
        let nsecs = match u64::try_from(val.nsecs) {
            Ok(val) => val,
            Err(e) => bail!("Failed to convert ROS time to std::time::SystemTime, nsecs term overflows u64 likely: {val:?}, {e:?}"),
        };
        let duration = std::time::Duration::new(secs, nsecs as u32);
        Ok(std::time::UNIX_EPOCH + duration)
    }
}

impl RosMessageType for Time {
    const ROS_TYPE_NAME: &'static str = "builtin_interfaces/Time";
    // TODO: ROS2 support
    const MD5SUM: &'static str = "";
    const DEFINITION: &'static str = "";
}

/// Matches the integral ros1 duration type, with extensions for ease of use
/// NOTE: Is not a message in and of itself use std_msgs/Duration for that
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Duration {
    pub sec: i32,
    pub nsec: i32,
}

/// Conversion from [std::time::Duration] to our internal [Duration] type
/// Note: this provides both [tokio::time::Duration] and [std::time::Duration]
impl TryFrom<std::time::Duration> for Duration {
    type Error = SimpleError;
    fn try_from(val: std::time::Duration) -> Result<Self, Self::Error> {
        let downcast_sec = match i32::try_from(val.as_secs()) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to cast tokio duration to ROS duration, secs could not fit in i32:  {e:?}"
            ),
        };
        let downcast_nsec = match i32::try_from(val.subsec_nanos()) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to cast tokio duration ROS duration, nsecs could not fit in i32: {e:?}"
            ),
        };
        Ok(Duration {
            sec: downcast_sec,
            nsec: downcast_nsec,
        })
    }
}

/// Conversion from our internal [Duration] type to [std::time::Duration]
/// Note: this provides both [tokio::time::Duration] and [std::time::Duration]
impl TryFrom<Duration> for std::time::Duration {
    type Error = SimpleError;
    fn try_from(val: Duration) -> Result<Self, Self::Error> {
        let upcast_sec = match u64::try_from(val.sec) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to cast ROS duration to tokio duration, secs could not fit in u64: {e:?}"
            ),
        };
        let upcast_nsec = match u32::try_from(val.nsec) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to cast ROS duration to tokio duration, nsecs could not fit in u64: {e:?}"
            ),
        };
        Ok(std::time::Duration::new(upcast_sec, upcast_nsec))
    }
}

/// Conversion from chrono::DateTime<chrono::Utc> to our internal Time type
#[cfg(feature = "chrono")]
impl TryFrom<chrono::DateTime<chrono::Utc>> for Time {
    type Error = SimpleError;
    fn try_from(val: chrono::DateTime<chrono::Utc>) -> Result<Self, Self::Error> {
        let downcast_secs = match i32::try_from(val.timestamp()) {
            Ok(val) => val,
            Err(e) => {
                bail!("Failed to convert chrono time to ROS time, secs could not fit in i32: {e:?}")
            }
        };
        let downcast_nanos = match i32::try_from(val.timestamp_subsec_nanos()) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to convert chrono time to ROS time, nsecs could not fit in i32: {e:?}"
            ),
        };
        Ok(Time {
            secs: downcast_secs,
            nsecs: downcast_nanos,
        })
    }
}

/// Conversion from our internal [Time] type to [chrono::DateTime]
#[cfg(feature = "chrono")]
impl TryFrom<Time> for chrono::DateTime<chrono::Utc> {
    type Error = SimpleError;
    fn try_from(val: Time) -> Result<Self, Self::Error> {
        let secs = i64::from(val.secs);
        let nsecs = match u32::try_from(val.nsecs) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to convert ROS time to chrono time, nsecs could not fit in u32: {e:?}"
            ),
        };
        match chrono::DateTime::from_timestamp(secs, nsecs) {
            Some(val) => Ok(val),
            None => bail!("Failed to convert ROS time to chrono time, secs and nsecs could not fit in chrono::DateTime."),
        }
    }
}

/// Conversion from [chrono::Duration] to our internal [Duration] type
#[cfg(feature = "chrono")]
impl TryFrom<chrono::Duration> for Duration {
    type Error = SimpleError;
    fn try_from(val: chrono::Duration) -> Result<Self, Self::Error> {
        // Chrono uses i64 for seconds, ROS uses i32 have to attempt downcast
        let downcast_sec = match i32::try_from(val.num_seconds()) {
            Ok(val) => val,
            Err(e) => bail!(
                "Failed to cast chrono duration to ROS duration, secs could not fit in i32:  {e:?}"
            ),
        };
        Ok(Duration {
            sec: downcast_sec,
            nsec: val.subsec_nanos(),
        })
    }
}

/// Conversion from our internal [Duration] type to [chrono::Duration]
#[cfg(feature = "chrono")]
impl TryFrom<Duration> for chrono::Duration {
    type Error = SimpleError;
    // Note: this conversion shouldn't be fallible, ROS time should always fit in chrono::Duration
    // Just being pedantic about error handling, and matching style of other conversions
    fn try_from(val: Duration) -> Result<Self, Self::Error> {
        let secs = match chrono::Duration::try_seconds(i64::from(val.sec)) {
            Some(val) => val,
            None => bail!("Failed to cast ROS duration to chrono duration, secs could not fit in chrono::Duration."),
        };
        // Not fallible because nanoseconds can't overflow TimeDelta
        let nsecs = chrono::Duration::nanoseconds(i64::from(val.nsec));
        // Probably unnecessary, but being pedantic about error handling
        let total = match secs.checked_add(&nsecs) {
            Some(val) => val,
            None => bail!("Failed to cast ROS duration to chrono duration, addition overflowed when combining secs and nsecs."),
        };
        Ok(total)
    }
}

#[cfg(test)]
mod test {
    #[test]
    fn test_time_conversions() {
        // Basic round trip test of now
        let time = std::time::SystemTime::now();
        let ros_time: crate::Time = time.try_into().unwrap();
        let std_time: std::time::SystemTime = ros_time.try_into().unwrap();
        assert_eq!(time, std_time);

        // Can "min time" convert
        let time = std::time::SystemTime::UNIX_EPOCH;
        let ros_time: crate::Time = time.try_into().unwrap();
        let std_time: std::time::SystemTime = ros_time.try_into().unwrap();
        assert_eq!(time, std_time);

        // Can "max time" convert
        let ros_time = crate::Time {
            secs: i32::MAX,
            nsecs: i32::MAX,
        };
        let std_time: std::time::SystemTime = ros_time.try_into().unwrap();
        assert_eq!(
            std_time,
            std::time::SystemTime::UNIX_EPOCH
                + std::time::Duration::new(i32::MAX as u64, i32::MAX as u32)
        );

        // Can "negative time" convert
        let ros_time = crate::Time {
            secs: i32::MIN,
            nsecs: i32::MIN,
        };
        let std_time: Result<std::time::SystemTime, _> = ros_time.try_into();
        // No it can't, not with how our current implementation is setup
        assert!(std_time.is_err());

        // How about positive time with negative nsecs?
        let ros_time = crate::Time { secs: 1, nsecs: -1 };
        let std_time: Result<std::time::SystemTime, _> = ros_time.try_into();
        // Nope our current implementation doesn't support negative nsecs at all
        // Would need to find some ROS code generating these to really confirm how this should be handled
        assert!(std_time.is_err());
    }

    #[test]
    fn test_duration_conversions() {
        // Basic test
        let tokio_duration = tokio::time::Duration::from_millis(1000);
        let ros_duration: crate::Duration = tokio_duration.try_into().unwrap();
        let roundtrip_duration: tokio::time::Duration = ros_duration.try_into().unwrap();
        assert_eq!(tokio_duration, roundtrip_duration);

        // Confirm std::time::Duration works as well
        let std_duration = std::time::Duration::from_millis(1000);
        let ros_duration: crate::Duration = std_duration.try_into().unwrap();
        let roundtrip_duration: std::time::Duration = ros_duration.try_into().unwrap();
        assert_eq!(std_duration, roundtrip_duration);

        // Test 0 duration
        let tokio_duration = tokio::time::Duration::from_millis(0);
        let ros_duration: crate::Duration = tokio_duration.try_into().unwrap();
        let roundtrip_duration: tokio::time::Duration = ros_duration.try_into().unwrap();
        assert_eq!(tokio_duration, roundtrip_duration);

        // Test negative ros duration
        let ros_duration = crate::Duration { sec: -1, nsec: -1 };
        let tokio_duration: Result<tokio::time::Duration, _> = ros_duration.try_into();
        // Won't work, we currently don't respect negative durations
        assert!(tokio_duration.is_err());
    }

    #[test]
    #[cfg(feature = "chrono")]
    fn test_chrono_duration_conversions() {
        // Basic test
        let chrono_duration = chrono::Duration::seconds(1) + chrono::Duration::nanoseconds(69);
        let ros_duration: crate::Duration = chrono_duration.try_into().unwrap();
        let roundtrip_duration: chrono::Duration = ros_duration.try_into().unwrap();
        assert_eq!(chrono_duration, roundtrip_duration);

        // Test 0 duration
        let chrono_duration = chrono::Duration::seconds(0);
        let ros_duration: crate::Duration = chrono_duration.try_into().unwrap();
        let roundtrip_duration: chrono::Duration = ros_duration.try_into().unwrap();
        assert_eq!(chrono_duration, roundtrip_duration);

        // Test large chrono time that can't fit into ros
        let chrono_duration = chrono::Duration::seconds(i64::MAX / 10_000);
        let ros_duration: Result<crate::Duration, _> = chrono_duration.try_into();
        assert!(ros_duration.is_err());

        // Test negative chrono time
        let chrono_duration = chrono::Duration::seconds(-1) + chrono::Duration::nanoseconds(-42);
        let ros_duration: crate::Duration = chrono_duration.try_into().unwrap();
        let roundtrip_duration: chrono::Duration = ros_duration.try_into().unwrap();
        assert_eq!(chrono_duration, roundtrip_duration);
    }

    #[test]
    #[cfg(feature = "chrono")]
    fn test_chrono_time_conversions() {
        // Basic test
        let now = chrono::offset::Utc::now();
        let ros_time: crate::Time = now.try_into().unwrap();
        let roundtrip_time: chrono::DateTime<chrono::Utc> = ros_time.try_into().unwrap();
        assert_eq!(now, roundtrip_time);

        // Test EPOCH
        let epoch = chrono::DateTime::<chrono::Utc>::UNIX_EPOCH;
        let ros_epoch: crate::Time = epoch.try_into().unwrap();
        let roundtrip_epoch: chrono::DateTime<chrono::Utc> = ros_epoch.try_into().unwrap();
        assert_eq!(epoch, roundtrip_epoch);

        // Test time that can't fit into ros
        let too_large = chrono::DateTime::<chrono::Utc>::UNIX_EPOCH
            + chrono::Duration::seconds(i32::MAX as i64 + 1000);
        let ros_time: Result<crate::Time, _> = too_large.try_into();
        assert!(ros_time.is_err());
    }
}
