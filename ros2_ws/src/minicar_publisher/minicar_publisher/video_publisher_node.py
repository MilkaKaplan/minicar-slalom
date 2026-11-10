"""Video version, container 1: plays the drive video like a front camera on the car.

Reads data/drive_video.mp4 and publishes its frames as sensor_msgs/Image on
/camera/front. The header stamp of every frame is its time in the video
(0 s = first frame), so the receiver knows which moment of the drive it is.

It waits until somebody subscribes (so no frame is lost), streams the video
once, then stays idle until it is stopped.
"""
import cv2
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .publisher_node import to_ros_image


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("front_camera_publisher")
        self.declare_parameter("video_path", "/data/drive_video.mp4")
        self.declare_parameter("topic", "/camera/front")
        self.declare_parameter("every_nth", 3)        # 30 fps video -> every 3rd frame = 10 frames per video second
        self.declare_parameter("rate_hz", 20.0)       # how fast to send them (20 Hz = 2x faster than real time)

        path = self.get_parameter("video_path").value
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.nth = int(self.get_parameter("every_nth").value)
        total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.get_logger().info(
            f"Loaded {path}: {total} frames, {self.fps:.0f} fps -> sending every {self.nth}. frame "
            f"({total // self.nth} frames)")

        # RELIABLE + a deep queue: the processor gets every frame, even if it is a bit slower
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Image, self.get_parameter("topic").value, qos)
        self.index = 0          # frame number in the video
        self.sent = 0
        self.finished = False
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    def tick(self):
        if self.finished:
            return
        if self.pub.get_subscription_count() == 0:     # nobody listening yet: wait
            return
        while True:                                     # read until the next frame we want to send
            ok, frame = self.cap.read()
            if not ok:
                self.finished = True
                self.get_logger().info(f"Video finished: sent {self.sent} frames on {self.pub.topic_name}")
                return
            i = self.index
            self.index += 1
            if i % self.nth == 0:
                break
        t = i / self.fps                                # time of this frame in the video
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        self.pub.publish(to_ros_image(rgb, "front_camera", stamp))
        self.sent += 1
        if self.sent % 50 == 0:
            self.get_logger().info(f"Sent {self.sent} frames (video time {t:.1f} s)")


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
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
