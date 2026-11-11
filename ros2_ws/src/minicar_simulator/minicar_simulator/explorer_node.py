"""Explorer node (container 2): a car that does NOT know the track drives it.

Subscribes to
    /lidar/map      sensor_msgs/PointCloud2   the LiDAR map = the "world" (the car only sees
                                              the part inside its own virtual sensor cone)
and, only to EVALUATE the result afterwards (the explorer never uses them while driving):
    /slalom/posts   geometry_msgs/PoseArray   posts found on the full map
    /slalom/path    nav_msgs/Path             plan made with the full map
    /slalom/limits  std_msgs/Float32MultiArray  wall position on the full map
Drives the unknown track three times - ideal, realistic (sensor noise + slow steering),
realistic + wind 3 m/s + variable speed - and runs a stress test (more and more sensor noise).
Saves explore.png / explore.gif (the windy drive), explore_conditions.png, explore_stress.png,
explore_result.json and publishes the windy drive on /explore/driven_path.
"""
import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

from . import explorer as ex


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")
        self.declare_parameter("output_dir", "/results/lidar")
        self.declare_parameter("sensor_range_cm", ex.SENSOR["range"])
        self.declare_parameter("sensor_fov_deg", ex.SENSOR["fov_deg"])
        self.declare_parameter("gap_cm", ex.CAR["gap"])
        self.declare_parameter("wind_cm_s", 300.0)     # 0 = calm; the windy drive also has variable speed
        self.declare_parameter("stress_runs", 5)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.sensor = {"range": float(p("sensor_range_cm")), "fov_deg": float(p("sensor_fov_deg"))}
        self.car = dict(ex.CAR, gap=float(p("gap_cm")))
        self.out_dir = p("output_dir")
        self.world = self.posts = self.offline = self.wall = None
        self.done = False

        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, "/lidar/map", self.on_map, latched)
        self.create_subscription(PoseArray, "/slalom/posts", self.on_posts, latched)
        self.create_subscription(Path, "/slalom/path", self.on_path, latched)
        self.create_subscription(Float32MultiArray, "/slalom/limits", self.on_limits, latched)
        self.pub = self.create_publisher(Path, "/explore/driven_path", latched)
        self.get_logger().info(f"Explorer: sensor {self.sensor['range']:.0f} cm, {self.sensor['fov_deg']:.0f} deg, "
                               f"gap {self.car['gap']} cm. Waiting for /lidar/map ...")

    def on_map(self, msg):
        rec = np.frombuffer(bytes(msg.data), dtype=[("x", "<f4"), ("y", "<f4"), ("height", "<f4")])
        self.world = np.column_stack([rec["x"], rec["y"], rec["height"]]).astype(float) * 100   # -> cm
        self.try_run()

    def on_posts(self, msg):
        self.posts = np.array([[q.position.x, q.position.y] for q in msg.poses]) * 100
        self.try_run()

    def on_path(self, msg):
        self.offline = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses]) * 100
        self.try_run()

    def on_limits(self, msg):
        self.wall = float(msg.data[2])
        self.try_run()

    def try_run(self):
        if self.done or any(v is None for v in (self.world, self.posts, self.offline, self.wall)):
            return
        self.get_logger().info(f"World: {len(self.world)} map points. Driving the UNKNOWN track "
                               "under three conditions ...")
        wind = float(self.get_parameter("wind_cm_s").value)
        drives = ex.drive_conditions(self.world, self.posts, self.wall, self.offline,
                                     sensor=self.sensor, car=self.car, wind_cm_s=wind)
        for name, (res, ev) in drives.items():
            self.get_logger().info(
                f"[{name}] finished={ev['finished']} posts {ev['posts_found']}/{ev['posts_true']}, "
                f"wall {ev['wall_found_cm']} cm, re-plans {ev['replans']}, path {ev['path_length_cm']} cm, "
                f"speed there/back {ev['speed_mean_there_cm_s']}/{ev['speed_mean_back_cm_s']} cm/s, "
                f"min gap {min(ev['gap_to_each_post_cm'])} cm, no_collision={ev['no_collision']}")
        res, ev = drives["windy"]                      # the most realistic drive is the main result
        for e in ev["events"]:
            self.get_logger().info(f"  [windy] t={e['t']:5.1f} s  {e['event']}"
                                   + (f" ({e['distance_cm']} cm ahead)" if "distance_cm" in e else ""))

        runs = int(self.get_parameter("stress_runs").value)
        self.get_logger().info(f"[stress] {runs} windy drives for each sensor noise level ...")
        rows = ex.stress_test(self.world, self.posts, self.wall, self.offline, sensor=self.sensor,
                              car=self.car, runs=runs, wind_cm_s=wind)
        for r in rows:
            self.get_logger().info(f"[stress] noise {r['noise_pos_cm']} cm: worst gap {r['gap_worst_cm']} cm, "
                                   f"collisions {r['collisions']}/{r['runs']}")

        out = Path()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), "track"
        lg = res["log"]
        for x, y in zip(lg["x"][::4], lg["y"][::4]):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x, ps.pose.position.y = float(x) / 100, float(y) / 100
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        self.pub.publish(out)

        os.makedirs(self.out_dir, exist_ok=True)
        ex.save_plot(self.world, res, ev, self.posts, self.offline, os.path.join(self.out_dir, "explore.png"))
        ex.save_animation(self.world, res, os.path.join(self.out_dir, "explore.gif"), sensor=self.sensor)
        ex.save_conditions_plot(self.world, drives, self.posts, self.offline,
                                os.path.join(self.out_dir, "explore_conditions.png"))
        ex.save_stress_plot(rows, os.path.join(self.out_dir, "explore_stress.png"))
        result = {name: ev_ for name, (_, ev_) in drives.items()}
        result["stress"] = rows
        result["sensor"], result["car"] = self.sensor, self.car
        with open(os.path.join(self.out_dir, "explore_result.json"), "w") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info("Wrote explore.png/.gif (windy drive), explore_conditions.png, "
                               "explore_stress.png, explore_result.json")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
