#!/usr/bin/env python3
"""
A simple rospy service client for testing bidirectional service calls with roslibrust.
This client calls the AddTwoInts service and prints the result.
"""
import rospy
import sys
import signal


def create_add_two_ints_service_class():
    """Create the AddTwoInts service class dynamically."""
    import genpy

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
    if len(sys.argv) < 4:
        print("Usage: rospy_add_two_ints_client.py <service_name> <a> <b>", file=sys.stderr)
        sys.exit(1)

    service_name = sys.argv[1]
    a = int(sys.argv[2])
    b = int(sys.argv[3])

    rospy.init_node("rospy_add_two_ints_client", anonymous=True)

    AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse = create_add_two_ints_service_class()

    rospy.loginfo(f"Waiting for service {service_name}...")
    rospy.wait_for_service(service_name, timeout=10.0)

    try:
        add_two_ints = rospy.ServiceProxy(service_name, AddTwoInts)
        rospy.loginfo(f"Calling service with a={a}, b={b}")
        resp = add_two_ints(AddTwoIntsRequest(a=a, b=b))
        # Print result in parseable format
        print(f"RESULT:{resp.sum}", flush=True)
        rospy.loginfo(f"Service call successful: {a} + {b} = {resp.sum}")
        return 0
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        print(f"ERROR:{e}", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())

