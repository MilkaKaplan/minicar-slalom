"""Container 1 - data acquisition.

Reads the top-view photo of the slalom track and publishes it as a
sensor_msgs/Image on /camera/top_view, once per second, like a camera driver.

QoS RELIABLE + TRANSIENT_LOCAL: the last image is kept, so a processor that
starts later still receives it.
"""
import array

import numpy as np
import rclpy
from PIL import Image as PilImage
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def to_ros_image(rgb, frame_id, stamp):
    """numpy array (height, width, 3) of uint8 -> sensor_msgs/Image."""
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"                 # 3 bytes per pixel: red, green, blue
    msg.is_bigendian = 0
    msg.step = msg.width * 3              # bytes in one row
    msg.data = array.array("B", rgb.tobytes())
    return msg


class TopViewPublisher(Node):
    def __init__(self):
        super().__init__("top_view_publisher")
        self.declare_parameter("image_path", "/data/top_view.jpeg")
        self.declare_parameter("topic", "/camera/top_view")
        self.declare_parameter("frame_id", "top_camera")
        self.declare_parameter("rate_hz", 1.0)

        path = self.get_parameter("image_path").value
        rgb = np.asarray(PilImage.open(path).convert("RGB"), dtype=np.uint8)
        self.get_logger().info(f"Loaded {path}: {rgb.shape[1]} x {rgb.shape[0]} px")
        self.msg = to_ros_image(rgb, self.get_parameter("frame_id").value,
                                self.get_clock().now().to_msg())

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Image, self.get_parameter("topic").value, qos)
        self.count = 0
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.tick()

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.count += 1
        if self.count == 1 or self.count % 10 == 0:
            self.get_logger().info(f"Published image #{self.count} on {self.pub.topic_name}")


def main(args=None):
    rclpy.init(args=args)
    node = TopViewPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
