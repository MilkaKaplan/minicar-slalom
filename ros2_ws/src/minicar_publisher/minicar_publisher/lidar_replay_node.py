"""LiDAR version, container 1: replays the iPhone recording (Stray Scanner) as live sensors.

For every frame it publishes, all with the same header stamp:
    /phone/depth        sensor_msgs/Image   16UC1, distances in mm (256 x 192)
    /phone/confidence   sensor_msgs/Image   mono8, 0/1/2 = low/medium/high
    /phone/pose         geometry_msgs/PoseStamped   where the phone is (ARKit world, metres)
and once (latched):
    /phone/camera_info  sensor_msgs/CameraInfo  focal length / centre for the depth image

It waits until somebody subscribes, plays the recording once, then stays idle.
"""
import array
import os

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from PIL import Image as PilImage
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def image_msg(arr, encoding, stamp, frame_id):
    msg = Image()
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * arr.dtype.itemsize
    msg.data = array.array("B", np.ascontiguousarray(arr).astype(arr.dtype.newbyteorder("<")).tobytes())
    return msg


class LidarReplay(Node):
    def __init__(self):
        super().__init__("phone_lidar_replay")
        self.declare_parameter("recording_dir", "/data/lidar_drive")
        self.declare_parameter("every_nth", 2)         # ~34 fps recording -> every 2nd frame
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("rgb_width", 1920)      # camera_matrix.csv is for the 1920 px wide video

        d = self.get_parameter("recording_dir").value
        self.dir = d
        self.odo = np.genfromtxt(os.path.join(d, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(9))
        K = np.loadtxt(os.path.join(d, "camera_matrix.csv"), delimiter=",")   # for the 1920 x 1440 video
        w, h = PilImage.open(os.path.join(d, "depth", "000000.png")).size
        s = w / float(self.get_parameter("rgb_width").value)   # depth image = scaled-down video frame
        self.info = CameraInfo()
        self.info.width, self.info.height = w, h
        self.info.k = [K[0, 0] * s, 0.0, K[0, 2] * s, 0.0, K[1, 1] * s, K[1, 2] * s, 0.0, 0.0, 1.0]
        self.nth = int(self.get_parameter("every_nth").value)
        self.get_logger().info(f"Recording {d}: {len(self.odo)} frames, depth {w} x {h}, "
                               f"sending every {self.nth}. frame")

        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_info = self.create_publisher(CameraInfo, "/phone/camera_info", latched)
        self.pub_depth = self.create_publisher(Image, "/phone/depth", reliable)
        self.pub_conf = self.create_publisher(Image, "/phone/confidence", reliable)
        self.pub_pose = self.create_publisher(PoseStamped, "/phone/pose", reliable)
        self.pub_info.publish(self.info)
        self.i = 0
        self.sent = 0
        self.finished = False
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    def tick(self):
        if self.finished or self.pub_pose.get_subscription_count() == 0:
            return
        if self.i >= len(self.odo):
            self.finished = True
            self.get_logger().info(f"Recording finished: sent {self.sent} frames")
            return
        row = self.odo[self.i]
        t = row[0] - self.odo[0, 0]                       # seconds since the start of the recording
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        name = f"{int(self.i):06d}.png"
        depth = np.array(PilImage.open(os.path.join(self.dir, "depth", name)), dtype=np.uint16)
        conf = np.array(PilImage.open(os.path.join(self.dir, "confidence", name)), dtype=np.uint8)
        self.info.header.stamp = stamp
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "arkit_world"
        p, q = pose.pose.position, pose.pose.orientation
        p.x, p.y, p.z = map(float, row[2:5])
        q.x, q.y, q.z, q.w = map(float, row[5:9])
        self.pub_depth.publish(image_msg(depth, "16UC1", stamp, "phone_lidar"))
        self.pub_conf.publish(image_msg(conf, "mono8", stamp, "phone_lidar"))
        self.pub_pose.publish(pose)
        self.sent += 1
        self.i += self.nth
        if self.sent % 100 == 0:
            self.get_logger().info(f"Sent {self.sent} frames (t = {t:.1f} s)")


def main(args=None):
    rclpy.init(args=args)
    node = LidarReplay()
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
