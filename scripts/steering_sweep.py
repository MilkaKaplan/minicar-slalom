"""Which car can drive the slalom? Simulate cars with different steering limits.

Uses the planned path and the posts from the last pipeline run (results/path.csv,
results/result.json) and the car simulator (ideal car, no noise).

Run from the minicar folder after `docker compose up`:
    python3 scripts/steering_sweep.py
"""
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "minicar_simulator"))
from minicar_simulator import sim  # noqa: E402

rows = list(csv.DictReader(open("results/path.csv")))
path = np.array([[float(r["x_cm"]), float(r["y_cm"])] for r in rows])
posts = json.load(open("results/result.json"))["posts_cm"]
wall_x = max(p[0] for p in posts) + 50

print("max steer | R_min  | error mean / max | min gap to post | result")
out = []
for steer in [30, 20, 15, 12, 11, 10, 8]:
    car = dict(sim.CAR, max_steer_deg=steer)
    ev = sim.evaluate(sim.simulate(path, car), posts, 6.5, 50, wall_x, car)
    ok = ev["no_collision"] and ev["finished"]
    print(f"   {steer:2d} deg  | {ev['car_min_turn_radius_cm']:5.1f} cm | "
          f"{ev['tracking_error_mean_cm']:5.2f} / {ev['tracking_error_max_cm']:5.2f} cm | "
          f"{min(ev['gap_to_each_post_cm']):6.1f} cm      | {'OK' if ok else 'COLLISION'}")
    out.append({"max_steer_deg": steer, "min_turn_radius_cm": ev["car_min_turn_radius_cm"],
                "tracking_error_mean_cm": ev["tracking_error_mean_cm"],
                "tracking_error_max_cm": ev["tracking_error_max_cm"],
                "min_gap_to_post_cm": min(ev["gap_to_each_post_cm"]), "ok": ok})
os.makedirs("docs", exist_ok=True)
json.dump(out, open("docs/steering_sweep.json", "w"), indent=2)
print("Saved docs/steering_sweep.json")
