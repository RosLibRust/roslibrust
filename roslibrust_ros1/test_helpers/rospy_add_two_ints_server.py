#!/usr/bin/env python3
"""
A simple rospy service server for testing bidirectional service calls with roslibrust.
This server implements the AddTwoInts service from test_msgs.
"""
import rospy
import sys
import signal


class AddTwoIntsService:
    """Service server that adds two integers."""

    def __init__(self, service_name):
        self.service_name = service_name
        # Import the service type - we use std_srvs.SetBool as a stand-in
        # since test_msgs may not be compiled. We'll define our own request/response.
        from rospy.impl.tcpros_service import Service

        # Define a simple add_two_ints callback
        def handle_add_two_ints(req):
            rospy.loginfo(f"Received request: a={req.a}, b={req.b}")
            return AddTwoIntsResponse(sum=req.a + req.b)

        self.callback = handle_add_two_ints


# Since test_msgs may not be compiled as a catkin package, we define the service manually
def create_add_two_ints_service_class():
    """Create the AddTwoInts service class dynamically."""
    import genpy
    import rospy

    class AddTwoIntsRequest(genpy.Message):
        _type = "test_msgs/AddTwoIntsRequest"
        _md5sum = "36d09b846be0b371c5f190354dd3153e"
        _full_text = "int64 a\nint64 b"
        _slot_types = ["int64", "int64"]
        __slots__ = ["a", "b"]

        def __init__(self, a=0, b=0):
            self.a = a
            self.b = b

        def serialize(self, buff):
            import struct
            buff.write(struct.pack("<q", self.a))
            buff.write(struct.pack("<q", self.b))

        def deserialize(self, data):
            import struct
            self.a = struct.unpack("<q", data[:8])[0]
            self.b = struct.unpack("<q", data[8:16])[0]
            return self

    class AddTwoIntsResponse(genpy.Message):
        _type = "test_msgs/AddTwoIntsResponse"
        _md5sum = "b88405221c77b1878a3cbbfff53428d7"
        _full_text = "int64 sum"
        _slot_types = ["int64"]
        __slots__ = ["sum"]

        def __init__(self, sum=0):
            self.sum = sum

        def serialize(self, buff):
            import struct
            buff.write(struct.pack("<q", self.sum))

        def deserialize(self, data):
            import struct
            self.sum = struct.unpack("<q", data[:8])[0]
            return self

    class AddTwoInts:
        _type = "test_msgs/AddTwoInts"
        _md5sum = "6a2e34150c00229791cc89ff309fff21"
        _request_class = AddTwoIntsRequest
        _response_class = AddTwoIntsResponse

    return AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse


def main():
    service_name = sys.argv[1] if len(sys.argv) > 1 else "/rospy_add_two_ints"
    node_name = sys.argv[2] if len(sys.argv) > 2 else "rospy_add_two_ints_server"

    rospy.init_node(node_name)

    AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse = create_add_two_ints_service_class()

    def handle_add_two_ints(req):
        result = req.a + req.b
        rospy.loginfo(f"Received request: a={req.a}, b={req.b}, returning sum={result}")
        return AddTwoIntsResponse(sum=result)

    service = rospy.Service(service_name, AddTwoInts, handle_add_two_ints)
    rospy.loginfo(f"AddTwoInts service server ready at {service_name}")

    # Write to stdout to signal we're ready
    print(f"READY:{service_name}", flush=True)

    def signal_handler(sig, frame):
        rospy.loginfo("Shutting down service server...")
        sys.exit(0)

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()


if __name__ == "__main__":
    main()

