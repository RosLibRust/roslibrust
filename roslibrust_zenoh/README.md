# RosLibRust Zenoh

This crate provides a Zenoh client that is compatible with the zenoh-ros1-plugin / zenoh-ros1-bridge.

The plugin / bridge performs "topic mangling" that makes it challenging to directly subscribe to the bridged topics from zenoh.

The goal of this crate is to provide an effective intermediary between ros1 and zenoh, and eventually unify this behind the single TopicProvider trait.

## Graph discovery

`ZenohClient` does not continuously subscribe to ROS1 discovery beacons. Calls to
`GraphProvider::list_topics` and `GraphProvider::list_services` temporarily subscribe
to them for 2.25 seconds, then return a fresh snapshot and drop the subscription.
The collection window spans two of the bridge's one-second beacon intervals to
avoid missing announcements due to timing. Graph queries therefore take about
2.25 seconds, but normal message transport has no discovery-beacon subscriber.
