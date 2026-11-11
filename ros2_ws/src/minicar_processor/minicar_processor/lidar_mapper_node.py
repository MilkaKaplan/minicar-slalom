"""LiDAR version, container 2: builds the map from the phone's LiDAR stream and plans the slalom.

Subscribes to /phone/camera_info, /phone/depth, /phone/confidence, /phone/pose
(depth, confidence and pose of one frame have the same header stamp).
When the stream stops it:
  * finds the ground, the posts and the wall on the map (lidar_map.py) - no track
    measurements needed, only our car and the rules (corridor width);
  * plans the slalom there + U-turn + back through the detected posts;
  * compares the real drive (the phone's path) with the plan;
  * saves lidar_map.png, lidar_result.json, path.csv into output_dir;
  * publishes /slalom/path, /slalom/posts and /slalom/limits for the car simulator.
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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

from . import core
from . import lidar_map as lm


def image_to_array(msg):
    dtype = {"16UC1": "<u2", "mono8": "u1"}[msg.encoding]
    a = np.frombuffer(bytes(msg.data), dtype=dtype)
    per_row = msg.step // np.dtype(dtype).itemsize
    return a.reshape(msg.height, per_row)[:, : msg.width]


class LidarMapper(Node):
    def __init__(self):
        super().__init__("lidar_mapper")
        self.declare_parameter("output_dir", "/results/lidar")
        self.declare_parameter("idle_timeout_s", 3.0)
        self.declare_parameter("linger_s", 10.0)
        # what we know in advance: OUR car and the RULES. Everything else is measured.
        self.declare_parameter("car_width_cm", 5.0)
        self.declare_parameter("gap_cm", 10.0)
        self.declare_parameter("corridor_width_cm", 100.0)
        self.declare_parameter("post_radius_cm", 3.5)
        # optional hand measurements, only to report the accuracy of the LiDAR map
        self.declare_parameter("measurements_file", "/data/lidar_drive/measurements.json")

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.car = {"half_width": p("car_width_cm") / 2, "gap": p("gap_cm")}
        self.out_dir = p("output_dir")
        self.builder = None
        self.frames = {}                 # stamp -> {"depth":..., "conf":..., "pose":...}
        self.n_added = 0
        self.last_at = None
        self.finished_at = None
        self.done = False

        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, "/phone/camera_info", self.on_info, latched)
        self.create_subscription(Image, "/phone/depth", lambda m: self.on_part(m, "depth"), reliable)
        self.create_subscription(Image, "/phone/confidence", lambda m: self.on_part(m, "conf"), reliable)
        self.create_subscription(PoseStamped, "/phone/pose", lambda m: self.on_part(m, "pose"), reliable)
        self.pub_path = self.create_publisher(Path, "/slalom/path", latched)
        self.pub_posts = self.create_publisher(PoseArray, "/slalom/posts", latched)
        self.pub_limits = self.create_publisher(Float32MultiArray, "/slalom/limits", latched)
        self.get_logger().info("Waiting for the phone's LiDAR stream ...")

    def on_info(self, msg):
        if self.builder is None:
            k = msg.k
            self.builder = lm.MapBuilder(k[0], k[4], k[2], k[5], msg.width, msg.height)
            self.get_logger().info(f"Camera: {msg.width} x {msg.height}, fx = {k[0]:.1f}")

    def on_part(self, msg, kind):
        if self.finished_at is not None:
            return
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        f = self.frames.setdefault(key, {})
        f[kind] = msg
        self.last_at = time.time()
        if len(f) == 3 and self.builder is not None:
            pose = f["pose"].pose
            self.builder.add(image_to_array(f["depth"]), image_to_array(f["conf"]),
                             [pose.position.x, pose.position.y, pose.position.z],
                             [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            del self.frames[key]
            self.n_added += 1
            if self.n_added % 100 == 0:
                self.get_logger().info(f"{self.n_added} frames added to the map")

    def tick(self):
        if self.finished_at is not None:
            if time.time() - self.finished_at > self.get_parameter("linger_s").value:
                self.done = True
            return
        if self.last_at and time.time() - self.last_at > self.get_parameter("idle_timeout_s").value:
            self.finish()

    def finish(self):
        self.finished_at = time.time()
        self.get_logger().info(f"Stream ended: {self.n_added} frames. Building the map ...")
        res = lm.analyse(self.builder, post_radius_m=self.get_parameter("post_radius_cm").value / 100)
        half_corridor = self.get_parameter("corridor_width_cm").value / 200
        parts, A, track, checks = lm.plan_on_map(res, self.car, half_corridor_m=half_corridor)
        cmp, _ = lm.compare_drive(res, parts, track, self.car)
        posts_cm = res["posts_track_m"] * 100
        spacing = np.linalg.norm(np.diff(posts_cm, axis=0), axis=1)
        wall_after = (res["wall_x_m"] - res["posts_track_m"][-1, 0]) * 100 if res["wall_x_m"] else None
        self.get_logger().info(
            f"posts={len(posts_cm)} spacing={np.round(spacing, 1).tolist()} cm, "
            f"wall {None if wall_after is None else round(wall_after, 1)} cm after the last post; "
            f"plan min gap {min(checks['gap_to_each_post_cm'])} cm; real drive "
            f"{cmp['deviation_from_plan_mean_cm']} cm from the plan on average")

        # publish for the simulator: limits first, then posts and path
        wall_x_cm = res["wall_x_m"] * 100 if res["wall_x_m"] else float(posts_cm[-1, 0] + 100)
        self.pub_limits.publish(Float32MultiArray(data=[float(track["post_r"]), float(track["half_corridor"]),
                                                        float(wall_x_cm)]))
        stamp = self.get_clock().now().to_msg()
        arr = PoseArray()
        arr.header.stamp, arr.header.frame_id = stamp, "track"
        for x, y in posts_cm:
            ps = Pose()
            ps.position.x, ps.position.y = float(x) / 100, float(y) / 100
            ps.orientation.w = 1.0
            arr.poses.append(ps)
        self.pub_posts.publish(arr)
        path = Path()
        path.header.stamp, path.header.frame_id = stamp, "track"
        for x, y in zip(np.concatenate([p[0] for p in parts.values()])[::10],
                        np.concatenate([p[1] for p in parts.values()])[::10]):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x) / 100, float(y) / 100
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

        os.makedirs(self.out_dir, exist_ok=True)
        lm.save_map_plot(res, parts, track, checks, cmp, os.path.join(self.out_dir, "lidar_map.png"))
        result = {
            "frames": self.n_added,
            "map_points": int(len(res["points"])),
            "ground_thickness_cm": round(res["ground_thickness_m"] * 100, 2),
            "phone_above_ground_cm": round(res["camera_above_ground_m"] * 100, 1),
            "candidates": int(res["n_candidates"]),
            "posts_found": int(len(posts_cm)),
            "posts_track_cm": np.round(posts_cm, 1).tolist(),
            "post_spacing_cm": np.round(spacing, 1).tolist(),
            "start_to_post1_cm": round(float(posts_cm[0, 0]), 1),
            "last_post_to_wall_cm": None if wall_after is None else round(float(wall_after), 1),
            "first_side": "left" if res["first_side"] > 0 else "right",
            "plan": {"swing_A_cm": A, "checks": checks},
            "real_drive_vs_plan": cmp,
            "known_in_advance": {"car_width_cm": self.car["half_width"] * 2, "gap_cm": self.car["gap"],
                                 "corridor_width_cm": track["half_corridor"] * 2,
                                 "post_radius_cm": track["post_r"]},
        }
        mf = self.get_parameter("measurements_file").value
        if mf and os.path.exists(mf):
            m = json.load(open(mf))
            result["hand_measurements"] = m
            result["lidar_vs_hand_cm"] = {
                "posts": [len(posts_cm), m.get("posts")],
                "mean_spacing": [round(float(spacing.mean()), 1), m.get("post_spacing_cm")],
                "last_post_to_wall": [None if wall_after is None else round(float(wall_after), 1),
                                      m.get("last_post_to_wall_cm")],
            }
        with open(os.path.join(self.out_dir, "lidar_result.json"), "w") as f:
            json.dump(result, f, indent=2)
        with open(os.path.join(self.out_dir, "path.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["part", "x_cm", "y_cm"])
            w.writerows(core.path_rows(parts))
        self.get_logger().info(f"Wrote lidar_map.png, lidar_result.json, path.csv to {self.out_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
