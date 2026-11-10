"""Video version, container 2: looks at the front-camera stream and finds the posts.

Subscribes to /camera/front (sensor_msgs/Image, header stamp = time in the video).
On every frame it finds the posts (front_camera.py), remembers where the nearest
one is, and draws them on a small copy of the frame. When the stream stops it saves
into output_dir:
    video_check.png        nearest-post position over time + detection rate per phase
    video_detections.gif   the video with the detected posts drawn on it
    video_check.json       numbers
and exits.
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from . import front_camera as fc
from .processor_node import from_ros_image


class VideoChecker(Node):
    def __init__(self):
        super().__init__("video_checker")
        self.declare_parameter("topic", "/camera/front")
        self.declare_parameter("output_dir", "/results/video")
        self.declare_parameter("idle_timeout_s", 3.0)     # no frames for this long = video finished
        # Drive phases, marked by hand on the video (see docs/dataset_stats.json)
        self.declare_parameter("phase_names", ["there", "U-turn", "back", "end"])
        self.declare_parameter("phase_starts_s", [0.0, 14.0, 22.0, 40.0])

        names = list(self.get_parameter("phase_names").value)
        starts = list(self.get_parameter("phase_starts_s").value) + [1e9]
        self.phases = [(n, starts[i], starts[i + 1]) for i, n in enumerate(names)]
        self.out_dir = self.get_parameter("output_dir").value
        self.t, self.x, self.w, self.n_posts, self.gif = [], [], [], [], []
        self.last_frame_at = None
        self.done = False

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(Image, self.get_parameter("topic").value, self.on_frame, qos)
        self.get_logger().info(f"Waiting for frames on {self.sub.topic_name} ...")

    def on_frame(self, msg):
        rgb = from_ros_image(msg)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        posts, _ = fc.detect(rgb)
        near = fc.nearest(posts)
        centre = rgb.shape[1] / 2
        self.t.append(t)
        self.x.append((near[0] + near[1]) / 2 - centre if near else np.nan)
        self.w.append(near[1] - near[0] if near else 0)
        self.n_posts.append(len(posts))
        if len(self.t) % 2 == 1:                            # every 2nd frame goes into the GIF
            self.gif.append(fc.annotate(rgb, posts, t, fc.phase_of(t, self.phases)))
        if len(self.t) == 1:
            self.get_logger().info("Receiving frames ...")
        elif len(self.t) % 100 == 0:
            self.get_logger().info(f"{len(self.t)} frames processed (video time {t:.1f} s)")
        self.last_frame_at = time.time()

    def check_finished(self):
        if self.done or self.last_frame_at is None:
            return
        if time.time() - self.last_frame_at < self.get_parameter("idle_timeout_s").value:
            return
        self.finish()

    def finish(self):
        order = np.argsort(self.t)
        t, x = np.array(self.t)[order], np.array(self.x)[order]
        events = fc.passing_events(t, x, np.array(self.w)[order])
        os.makedirs(self.out_dir, exist_ok=True)
        rate = fc.save_report(t, x, self.phases, events, os.path.join(self.out_dir, "video_check.png"))
        fc.save_gif(self.gif, os.path.join(self.out_dir, "video_detections.gif"))

        result = {
            "frames_received": int(len(t)),
            "video_time_s": [round(float(t[0]), 2), round(float(t[-1]), 2)],
            "detection_rate_percent_per_phase": rate,
            "posts_passed_events": [{"time_s": te, "post_left_frame_on": side} for te, side in events],
            "events_per_phase": {n: sum(1 for te, _ in events if a <= te < b) for n, a, b in self.phases},
            "note": ("Posts are found by their cool (bluish) white colour. This works on the way there "
                     "(white wall behind) but often fails when the camera faces the windows or the "
                     "office furniture, so the left/right sequence is NOT reliable enough to confirm "
                     "the planned slalom."),
        }
        with open(os.path.join(self.out_dir, "video_check.json"), "w") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f"Frames: {len(t)}, posts found per phase (%): {rate}")
        self.get_logger().info(f"Passing events: {len(events)} -> {result['events_per_phase']}")
        self.get_logger().info(f"Wrote video_check.png, video_detections.gif, video_check.json to {self.out_dir}")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = VideoChecker()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.check_finished()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
