"""LiDAR version: statistics of the iPhone depth data + a side view of the LiDAR map.

Writes:
    docs/images/lidar/depth_histograms.png   distances, confidence shares, confidence vs distance
    docs/images/lidar/side_view.png          side view of the map: flat ground, posts sticking out
    docs/lidar_dataset_stats.json            (updated with the numbers below)

Run from the minicar folder (numpy, scipy, matplotlib, Pillow):
    python3 scripts/lidar_dataset_stats.py
or in Docker:
    docker compose run --rm stats python3 /work/scripts/lidar_dataset_stats.py
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "minicar_processor"))
from minicar_processor import lidar_map as lm  # noqa: E402

D = os.path.join(ROOT, "data", "lidar_drive")
IMG = os.path.join(ROOT, "docs", "images", "lidar")
STATS = os.path.join(ROOT, "docs", "lidar_dataset_stats.json")
CONF_NAMES = ["low", "medium", "high"]
CONF_COLOURS = ["#d73027", "#fee08b", "#1a9850"]


def depth_histograms():
    """Every 5th frame: all depth pixels with their confidence."""
    dist, conf = [], []
    n = len(os.listdir(os.path.join(D, "depth")))
    for i in range(0, n, 5):
        d = np.array(Image.open(f"{D}/depth/{i:06d}.png")).astype(np.float32) / 1000
        c = np.array(Image.open(f"{D}/confidence/{i:06d}.png"))
        m = d > 0
        dist.append(d[m])
        conf.append(c[m])
    dist, conf = np.concatenate(dist), np.concatenate(conf)

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
    # 1. how far are the points? stacked by confidence
    bins = np.arange(0, 8.01, 0.1)
    ax[0].hist([dist[conf == k] for k in range(3)], bins=bins, stacked=True,
               color=CONF_COLOURS, label=CONF_NAMES)
    ax[0].axvline(lm.MAX_RANGE_M, c="black", ls="--")
    ax[0].text(lm.MAX_RANGE_M + 0.1, ax[0].get_ylim()[1] * 0.9, "used: < 2 m", fontsize=9)
    ax[0].set_xlabel("distance [m]")
    ax[0].set_ylabel("number of depth pixels")
    ax[0].set_title("How far are the points?")
    ax[0].legend(title="confidence", fontsize=8)

    # 2. share of low / medium / high confidence
    share = [(conf == k).mean() * 100 for k in range(3)]
    ax[1].bar(CONF_NAMES, share, color=CONF_COLOURS)
    for k, v in enumerate(share):
        ax[1].text(k, v + 1, f"{v:.0f} %", ha="center")
    ax[1].set_ylim(0, max(share) + 10)
    ax[1].set_ylabel("share of depth pixels [%]")
    ax[1].set_title("LiDAR confidence of all pixels")

    # 3. confidence vs distance
    edges = np.arange(0, 6.01, 0.25)
    centres = (edges[:-1] + edges[1:]) / 2
    idx = np.digitize(dist, edges) - 1
    shares = np.full((3, len(centres)), np.nan)
    for b in range(len(centres)):
        m = idx == b
        if m.sum() > 200:
            shares[:, b] = [(conf[m] == k).mean() * 100 for k in range(3)]
    for k in range(3):
        ax[2].plot(centres, shares[k], "-o", ms=3, c=CONF_COLOURS[k], label=CONF_NAMES[k])
    ax[2].axvline(lm.MAX_RANGE_M, c="black", ls="--")
    ax[2].set_xlabel("distance [m]")
    ax[2].set_ylabel("share within this distance [%]")
    ax[2].set_title("Confidence vs distance (high is best at 0.5-2 m)")
    ax[2].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "depth_histograms.png"), dpi=90)
    plt.close(fig)

    near, far = dist < lm.MAX_RANGE_M, dist >= lm.MAX_RANGE_M
    return {
        "depth_pixels_sampled": int(len(dist)),
        "distance_m_percentiles_10_25_50_75_90": np.round(np.percentile(dist, [10, 25, 50, 75, 90]), 2).tolist(),
        "share_closer_than_2m_percent": round(float(near.mean() * 100), 1),
        "confidence_share_percent_low_med_high": [round(float(v), 1) for v in share],
        "high_confidence_percent_closer_than_2m": round(float((conf[near] == 2).mean() * 100), 1),
        "high_confidence_percent_from_2m": round(float((conf[far] == 2).mean() * 100), 1),
    }


def side_view():
    """Side view of the LiDAR map along the track: x forward vs height above ground."""
    K = np.loadtxt(f"{D}/camera_matrix.csv", delimiter=",")
    od = np.genfromtxt(f"{D}/odometry.csv", delimiter=",", skip_header=1, usecols=range(9))
    s = 256 / 1920
    b = lm.MapBuilder(K[0, 0] * s, K[1, 1] * s, K[0, 2] * s, K[1, 2] * s, 256, 192)
    for i in range(0, len(od), 2):
        b.add(np.array(Image.open(f"{D}/depth/{i:06d}.png")), np.array(Image.open(f"{D}/confidence/{i:06d}.png")),
              od[i, 2:5], od[i, 5:9])
    res = lm.analyse(b)
    P, h = res["points"], res["height"] * 100
    t = res["to_track"](P[:, [0, 2]]) * 100
    posts = res["posts_track_m"] * 100

    corridor = np.abs(t[:, 1]) < 50
    near_post = np.zeros(len(P), bool)
    post_heights = []
    for px, py in posts:
        m = corridor & (np.hypot(t[:, 0] - px, t[:, 1] - py) < 8) & (h > 3) & (h < 60)
        near_post |= m
        post_heights.append(round(float(np.percentile(h[m], 95)), 1) if m.sum() > 20 else None)
    ground = corridor & (np.abs(h) < 5) & ~near_post
    other = corridor & (h >= 5) & (h < 60) & ~near_post
    wall_x = res["wall_x_m"] * 100 if res["wall_x_m"] else None

    fig, ax = plt.subplots(figsize=(14, 4.8))
    ax.scatter(t[ground, 0][::3], h[ground][::3], s=0.3, c="#999999", label="ground points")
    ax.scatter(t[other, 0], h[other], s=0.5, c="#8da0cb", label="other things (wall, plants)")
    ax.scatter(t[near_post, 0], h[near_post], s=0.8, c="red", label="posts found")
    ax.axhline(0, c="black", lw=1, label="fitted ground plane")
    th = res["ground_thickness_m"] * 100
    ax.axhspan(-th, th, color="black", alpha=0.08, label=f"ground thickness ±{th:.1f} cm")
    for (px, _), ph in zip(posts, post_heights):
        if ph:
            ax.annotate(f"{ph:.0f} cm", (px, ph), xytext=(px, ph + 7), ha="center", fontsize=9,
                        arrowprops=dict(arrowstyle="-", color="red", lw=0.8))
    if wall_x:
        ax.axvline(wall_x, c="brown", lw=2, label="wall found")
    ax.set_xlim(-20, (wall_x or 450) + 30)
    ax.set_ylim(-15, 65)
    ax.set_xlabel("x, forward from start [cm]")
    ax.set_ylabel("height above ground [cm]")
    ax.set_title("Side view of the LiDAR map (corridor only, |y| < 50 cm): flat ground, posts stick out")
    ax.legend(loc="upper left", fontsize=8, markerscale=8, ncol=2)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "side_view.png"), dpi=90)
    plt.close(fig)
    return {"ground_thickness_cm": round(th, 2), "post_heights_cm_p95": post_heights}


if __name__ == "__main__":
    os.makedirs(IMG, exist_ok=True)
    stats = json.load(open(STATS)) if os.path.exists(STATS) else {}
    stats["depth_histograms"] = depth_histograms()
    stats["side_view"] = side_view()
    json.dump(stats, open(STATS, "w"), indent=2)
    print(json.dumps({k: stats[k] for k in ("depth_histograms", "side_view")}, indent=2))
    print("Saved docs/images/lidar/depth_histograms.png, side_view.png, docs/lidar_dataset_stats.json")
