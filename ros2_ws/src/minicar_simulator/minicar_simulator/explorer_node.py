"""Explorer node (container 2): a car that does NOT know the track drives it.

Subscribes to
    /lidar/map      sensor_msgs/PointCloud2   the LiDAR map = the "world" (the car only sees
                                              the part inside its own virtual sensor cone)
and, only to EVALUATE the result afterwards (the explorer never uses them while driving):
    /slalom/posts   geometry_msgs/PoseArray   posts found on the full map
    /slalom/path    nav_msgs/Path             plan made with the full map
    /slalom/limits  std_msgs/Float32MultiArray  wall position on the full map
Saves explore.png, explore.gif, explore_result.json and publishes /explore/driven_path.
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
        self.get_logger().info(f"World: {len(self.world)} map points. Driving the UNKNOWN track ...")
        res = ex.explore(self.world, sensor=self.sensor, car=self.car)
        ev = ex.evaluate(res, self.posts, self.wall, self.offline)
        for e in ev["events"]:
            self.get_logger().info(f"  t={e['t']:5.1f} s  {e['event']}"
                                   + (f" ({e['distance_cm']} cm ahead)" if "distance_cm" in e else ""))
        self.get_logger().info(
            f"finished={ev['finished']} posts {ev['posts_found']}/{ev['posts_true']}, wall {ev['wall_found_cm']} "
            f"(map {ev['wall_true_cm']}) cm, re-plans {ev['replans']}, path {ev['path_length_cm']} cm "
            f"(known track {ev.get('offline_plan_length_cm')}), min gap {min(ev['gap_to_each_post_cm'])} cm, "
            f"no_collision={ev['no_collision']}")

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
        ev["sensor"], ev["car"] = self.sensor, self.car
        with open(os.path.join(self.out_dir, "explore_result.json"), "w") as f:
            json.dump(ev, f, indent=2)
        self.get_logger().info("Wrote explore.png, explore.gif, explore_result.json")
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
