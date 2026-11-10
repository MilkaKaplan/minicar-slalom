"""Step 4: the whole processing in one script (steps 2 + 3 joined), WITHOUT ROS.

The functions now live in ros2_ws/src/minicar_processor/minicar_processor/core.py,
so this script and the ROS 2 processor node use exactly the same code.

Run from the minicar folder:
    python3 scripts/step4_pipeline.py
"""
import csv
import json
import os
import sys

import matplotlib.pyplot as plt

# make "minicar_processor" importable without installing it
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "minicar_processor"))
from minicar_processor import core  # noqa: E402

PHOTO = "data/top_view.jpeg"
OUT_DIR = "results"
# Only for the accuracy report
POSTS_HAND_PX = [[300, 160], [310, 520], [300, 890], [300, 1250]]

if __name__ == "__main__":
    img = plt.imread(PHOTO)
    result, drawing = core.run(img, hand_px=POSTS_HAND_PX)
    print(json.dumps(result, indent=2))

    os.makedirs(OUT_DIR, exist_ok=True)
    core.save_picture(drawing, f"{OUT_DIR}/step4_pipeline.png")
    with open(f"{OUT_DIR}/result.json", "w") as f:
        json.dump(result, f, indent=2)
    with open(f"{OUT_DIR}/path.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "x_cm", "y_cm"])
        w.writerows(core.path_rows(drawing["parts"]))
    print(f"Saved: {OUT_DIR}/step4_pipeline.png, {OUT_DIR}/result.json, {OUT_DIR}/path.csv")
