"""ROS 2 node used by the gated wstring backend interoperability tests."""

import sys

import rclpy
from rclpy.node import Node
from test_msgs.msg import WStrings


class WStringRelay(Node):
    def __init__(self, input_topic: str, output_topic: str) -> None:
        super().__init__("roslibrust_wstring_relay")
        self.publisher = self.create_publisher(WStrings, output_topic, 10)
        self.subscription = self.create_subscription(
            WStrings, input_topic, self.publisher.publish, 10
        )


def main() -> None:
    rclpy.init()
    node = WStringRelay(sys.argv[1], sys.argv[2])
    print("READY", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
