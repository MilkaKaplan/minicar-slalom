"""Car simulator node (runs in container 2, next to the processor).

Subscribes to
    /slalom/path   nav_msgs/Path            the planned path (metres, frame "track")
    /slalom/posts  geometry_msgs/PoseArray  where the posts are (metres)
and drives a simulated car along the path (sim.py) in three ways:

  1. IDEAL       perfect sensors, wheels turn instantly
                 -> sim_ideal.png, sim_ideal.gif
  2. REALISTIC   noisy position/heading sensor + slow steering motor (one drive)
                 -> sim_realistic.png, sim_realistic.gif
  3. ROBUSTNESS  many realistic drives with more and more sensor noise
                 -> sim_robustness.png

All numbers go to sim_result.json. The realistic trajectory is published on
/sim/driven_path (nav_msgs/Path). Then the node exits.
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
from std_msgs.msg import Float32MultiArray

from . import sim


class CarSimulator(Node):
    def __init__(self):
        super().__init__("car_simulator")
        self.declare_parameter("output_dir", "/results")
        self.declare_parameter("make_gif", True)
        # Car (cm, deg, cm/s) and controller
        self.declare_parameter("car_width_cm", sim.CAR["half_width"] * 2)
        self.declare_parameter("wheelbase_cm", sim.CAR["wheelbase"])
        self.declare_parameter("max_steer_deg", sim.CAR["max_steer_deg"])
        self.declare_parameter("speed_cm_s", sim.CAR["speed"])
        self.declare_parameter("lookahead_cm", sim.CONTROL["lookahead"])
        # Realistic car: sensor noise and steering motor speed
        self.declare_parameter("noise_pos_cm", sim.REALISTIC["noise_pos_cm"])
        self.declare_parameter("noise_heading_deg", sim.REALISTIC["noise_heading_deg"])
        self.declare_parameter("steer_rate_deg_s", sim.REALISTIC["steer_rate_deg_s"])
        # Robustness test
        self.declare_parameter("robustness_noise_levels_cm", [0.5, 1.0, 2.0, 3.0])
        self.declare_parameter("robustness_runs", 10)
        # Track (cm)
        self.declare_parameter("post_radius_cm", 6.5)
        self.declare_parameter("half_corridor_cm", 50.0)
        self.declare_parameter("last_post_to_wall_cm", 50.0)
        # LiDAR version: post radius, corridor and wall come from the map on /slalom/limits
        self.declare_parameter("wait_for_limits", False)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.car = {"half_width": p("car_width_cm") / 2, "wheelbase": p("wheelbase_cm"),
                    "max_steer_deg": p("max_steer_deg"), "speed": p("speed_cm_s")}
        self.control = dict(sim.CONTROL, lookahead=p("lookahead_cm"))
        self.realistic = {"noise_pos_cm": p("noise_pos_cm"), "noise_heading_deg": p("noise_heading_deg"),
                          "steer_rate_deg_s": p("steer_rate_deg_s"), "seed": 0}
        self.out_dir = p("output_dir")
        self.path_cm = None
        self.posts_cm = None
        self.limits = None              # (post_radius_cm, half_corridor_cm, wall_x_cm) from the map
        self.done = False

        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/slalom/path", self.on_path, latched)
        self.create_subscription(PoseArray, "/slalom/posts", self.on_posts, latched)
        self.create_subscription(Float32MultiArray, "/slalom/limits", self.on_limits, latched)
        self.pub = self.create_publisher(Path, "/sim/driven_path", latched)
        self.get_logger().info(
            f"Car: width {self.car['half_width'] * 2} cm, wheelbase {self.car['wheelbase']} cm, "
            f"max steer {self.car['max_steer_deg']} deg -> R_min {sim.min_turn_radius(self.car):.1f} cm. "
            "Waiting for /slalom/path and /slalom/posts ...")

    def on_path(self, msg):
        self.path_cm = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]) * 100
        self.try_run(msg.header)

    def on_posts(self, msg):
        self.posts_cm = np.array([[p.position.x, p.position.y] for p in msg.poses]) * 100
        self.try_run(msg.header)

    def on_limits(self, msg):
        self.limits = tuple(float(v) for v in msg.data[:3])
        self.try_run(None)

    def drive(self, name, realism, track):
        """One drive: simulate, evaluate, save picture (+ GIF). Returns (log, numbers)."""
        log = sim.simulate(self.path_cm, self.car, self.control, realism)
        ev = sim.evaluate(log, self.posts_cm, *track, self.car)
        self.get_logger().info(
            f"[{name}] finished={ev['finished']} tracking error mean={ev['tracking_error_mean_cm']} "
            f"max={ev['tracking_error_max_cm']} cm, min gap to posts={min(ev['gap_to_each_post_cm'])} cm, "
            f"no_collision={ev['no_collision']}")
        label = {"ideal": "IDEAL car", "realistic": "REALISTIC car (noise + slow steering)"}[name]
        sim.save_plot(log, self.path_cm, self.posts_cm, *track, ev,
                      os.path.join(self.out_dir, f"sim_{name}.png"), label=label)
        if self.get_parameter("make_gif").value:
            sim.save_animation(log, self.path_cm, self.posts_cm, *track,
                               os.path.join(self.out_dir, f"sim_{name}.gif"), self.car)
        ev["realism"] = {k: v for k, v in realism.items() if k != "seed"}
        return log, ev

    def try_run(self, header):
        if self.done or self.path_cm is None or self.posts_cm is None:
            return
        if self.get_parameter("wait_for_limits").value and self.limits is None:
            return
        self.get_logger().info(f"Got path ({len(self.path_cm)} points) and {len(self.posts_cm)} posts, driving ...")
        os.makedirs(self.out_dir, exist_ok=True)
        if self.limits is not None:
            track = self.limits
        else:
            track = (self.get_parameter("post_radius_cm").value,
                     self.get_parameter("half_corridor_cm").value,
                     self.posts_cm[:, 0].max() + self.get_parameter("last_post_to_wall_cm").value)

        _, ideal = self.drive("ideal", sim.IDEAL, track)
        log, realistic = self.drive("realistic", self.realistic, track)

        levels = list(self.get_parameter("robustness_noise_levels_cm").value)
        runs = int(self.get_parameter("robustness_runs").value)
        self.get_logger().info(f"[robustness] {runs} drives for each noise level {levels} cm ...")
        rows = sim.robustness(self.path_cm, self.posts_cm, *track, self.car, self.control,
                              noise_levels_cm=levels, runs=runs,
                              steer_rate_deg_s=self.realistic["steer_rate_deg_s"])
        for r in rows:
            self.get_logger().info(
                f"[robustness] noise {r['noise_pos_cm']} cm: worst error {r['tracking_error_max_worst_cm']} cm, "
                f"worst gap {r['gap_to_post_worst_cm']} cm, collisions {r['collisions']}/{r['runs']}")
        sim.save_robustness_plot(rows, os.path.join(self.out_dir, "sim_robustness.png"))

        # publish the realistic trajectory (every 10th step, metres)
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "track"
        for x, y in zip(log["x"][::10], log["y"][::10]):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x, ps.pose.position.y = float(x) / 100, float(y) / 100
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        self.pub.publish(out)

        result = {
            "car": {"width_cm": self.car["half_width"] * 2, "wheelbase_cm": self.car["wheelbase"],
                    "max_steer_deg": self.car["max_steer_deg"], "speed_cm_s": self.car["speed"],
                    "min_turn_radius_cm": round(float(sim.min_turn_radius(self.car)), 1),
                    "lookahead_cm": self.control["lookahead"]},
            "ideal": ideal,
            "realistic": realistic,
            "robustness": rows,
        }
        with open(os.path.join(self.out_dir, "sim_result.json"), "w") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info("Wrote sim_ideal.png/.gif, sim_realistic.png/.gif, sim_robustness.png, sim_result.json")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = CarSimulator()
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
