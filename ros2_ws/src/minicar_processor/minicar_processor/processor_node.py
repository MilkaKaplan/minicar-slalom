"""Container 2 - processing.

Subscribes to /camera/top_view (sensor_msgs/Image), runs the pipeline from core.py:
find posts -> centimetres -> slalom there + U-turn + back -> checks, then
  * saves results: slalom_plan.png, result.json, path.csv
  * publishes the path as nav_msgs/Path on /slalom/path (for RViz or a car controller)
    and the post positions as geometry_msgs/PoseArray on /slalom/posts,
and exits after one image (parameter exit_after) plus a few seconds (linger_s).
"""
import csv
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from . import core


def from_ros_image(msg):
    """sensor_msgs/Image (rgb8) -> numpy array (height, width, 3)."""
    if msg.encoding != "rgb8":
        raise ValueError(f"expected rgb8 image, got {msg.encoding}")
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return data.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)


class SlalomProcessor(Node):
    def __init__(self):
        super().__init__("slalom_processor")
        self.declare_parameter("topic", "/camera/top_view")
        self.declare_parameter("output_dir", "/results")
        self.declare_parameter("exit_after", 1)          # images to process; 0 = run forever
        # after the last image keep running a few seconds, so late subscribers
        # (the car simulator) still get the path and the posts
        self.declare_parameter("linger_s", 5.0)
        # Car parameters can be changed from the command line, e.g. -p gap_cm:=7.5
        self.declare_parameter("car_width_cm", core.CAR["half_width"] * 2)
        self.declare_parameter("gap_cm", core.CAR["gap"])
        # Optional hand labels (flat list x1, y1, x2, y2, ...) to report detection accuracy
        self.declare_parameter("hand_labels_px", [300.0, 160.0, 310.0, 520.0, 300.0, 890.0, 300.0, 1250.0])

        self.car = {"half_width": float(self.get_parameter("car_width_cm").value) / 2,
                    "gap": float(self.get_parameter("gap_cm").value)}
        hand = list(self.get_parameter("hand_labels_px").value)
        self.hand = np.array(hand, dtype=float).reshape(-1, 2) if hand else None
        self.out_dir = self.get_parameter("output_dir").value
        self.exit_after = int(self.get_parameter("exit_after").value)
        self.n_done = 0
        self.done = False

        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(Image, self.get_parameter("topic").value,
                                            self.on_image, latched)
        self.pub_path = self.create_publisher(Path, "/slalom/path", latched)
        self.pub_posts = self.create_publisher(PoseArray, "/slalom/posts", latched)
        self.finished_at = None
        self.get_logger().info(f"Waiting for images on {self.sub.topic_name} ...")

    def on_image(self, msg):
        if self.done or self.finished_at is not None:     # already finished, just lingering
            return
        img = from_ros_image(msg)
        self.get_logger().info(f"Received image {msg.width} x {msg.height}, processing ...")

        result, drawing = core.run(img, car=self.car, hand_px=self.hand)
        c = result["checks"]
        self.get_logger().info(
            f"posts={result['n_posts']} scale={result['scale_px_per_cm']} px/cm "
            f"min_gap={min(c['gap_to_each_post_cm'])} cm "
            f"tightest_turn={min(c['tightest_turn_cm'].values())} cm "
            f"length={c['path_length_cm']} cm no_collision={c['no_collision']}")

        self.publish_path(msg.header, drawing["parts"])
        self.publish_posts(msg.header, drawing["posts_cm"])
        self.save(result, drawing, msg)

        self.n_done += 1
        if self.exit_after and self.n_done >= self.exit_after:
            self.get_logger().info(f"Done: results in {self.out_dir}")
            self.finished_at = time.time()

    def publish_posts(self, header, posts_cm):
        arr = PoseArray()
        arr.header.stamp = header.stamp
        arr.header.frame_id = "track"
        for x, y in posts_cm:
            pose = Pose()
            pose.position.x, pose.position.y = float(x) / 100.0, float(y) / 100.0
            pose.orientation.w = 1.0
            arr.poses.append(pose)
        self.pub_posts.publish(arr)

    def check_linger(self):
        if self.finished_at is not None and time.time() - self.finished_at >= self.get_parameter("linger_s").value:
            self.done = True

    def publish_path(self, header, parts):
        path = Path()
        path.header.stamp = header.stamp
        path.header.frame_id = "track"            # x forward from the start, y left, in metres
        for x, y in zip(np.concatenate([p[0] for p in parts.values()])[::10],
                        np.concatenate([p[1] for p in parts.values()])[::10]):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x) / 100.0   # ROS uses metres
            pose.pose.position.y = float(y) / 100.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)

    def save(self, result, drawing, msg):
        os.makedirs(self.out_dir, exist_ok=True)
        core.save_picture(drawing, os.path.join(self.out_dir, "slalom_plan.png"))
        result["source"] = {"topic": self.sub.topic_name, "frame_id": msg.header.frame_id,
                            "width": msg.width, "height": msg.height}
        result["car"] = {"width_cm": self.car["half_width"] * 2, "gap_cm": self.car["gap"]}
        with open(os.path.join(self.out_dir, "result.json"), "w") as f:
            json.dump(result, f, indent=2)
        with open(os.path.join(self.out_dir, "path.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["part", "x_cm", "y_cm"])
            w.writerows(core.path_rows(drawing["parts"]))
        self.get_logger().info("Wrote slalom_plan.png, result.json, path.csv")


def main(args=None):
    rclpy.init(args=args)
    node = SlalomProcessor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
            node.check_linger()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
