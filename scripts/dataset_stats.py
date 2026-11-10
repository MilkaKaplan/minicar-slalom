"""Dataset statistics for the README: photos, track measurements, hand labels and the drive video.

Writes into docs/:
    dataset_stats.json                     all numbers
    images/dataset_overview.png            the photos of the dataset
    images/top_view_contrast.png           why the detection threshold is 10
    images/video_frames.png                one frame every 2 s, coloured by drive phase
    images/video_stats.png                 brightness and sharpness of every video frame

Runs in Docker (needs OpenCV for the video), from the minicar folder:
    docker compose run --rm stats
"""
import json
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "minicar_processor"))
from minicar_processor import core  # noqa: E402

DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "docs")
IMG = os.path.join(OUT, "images")

PHOTOS = {   # file -> what it shows
    "top_view.jpeg": "Top view from 2 m: the input of the pipeline",
    "trace.jpeg": "Top view with the path drawn by hand (black there, purple back)",
    "front_start.jpeg": "Car's view at the start, 1 m before post 1",
    "side_view_4_posts.jpeg": "Side view: the 4 posts in a row",
    "side_view_4_pos_middle.jpeg": "First test photo (posts + furniture)",
}
VIDEO = "drive_video.mp4"

# Track dimensions (cm). Exact ones come from the floor tiles (50 x 50 cm): the posts
# stand on tile corners, so spacing, corridor and distances are whole tiles.
# Post size and camera height are only approximate (estimated, not measured).
# The pipeline does not use the camera height: it takes the scale from the photo itself.
TRACK_MEASURED = {
    "tile_size_cm": 50, "posts": 4, "post_spacing_cm": 50, "post_diameter_cm": 13,
    "post_height_cm": 100, "corridor_width_cm": 100, "start_to_post1_cm": 100,
    "post4_to_wall_cm": 50, "top_view_camera_height_cm": 200, "car_width_cm": 5,
    "approximate_values": ["post_diameter_cm", "post_height_cm", "top_view_camera_height_cm"],
}
# Post bases marked by hand on top_view.jpeg (pixels)
HAND_LABELS_PX = [[300, 160], [310, 520], [300, 890], [300, 1250]]

# Phases of the drive video, marked by watching it (seconds)
VIDEO_PHASES = [
    ("there", 0, 14, "#4c72b0"),         # driving along the posts towards the wall
    ("U-turn", 14, 22, "#dd8452"),       # at the wall, turning around
    ("back", 22, 40, "#8172b3"),         # driving back, office furniture in the background
    ("end", 40, 45.1, "#55a868"),        # back at the start, facing the posts again
]
LOW_DETAIL = 10.0   # sharpness below this = blurred or almost featureless frame


def photo_stats():
    rows = []
    fig, ax = plt.subplots(1, len(PHOTOS), figsize=(3.2 * len(PHOTOS), 5.5))
    for a, (name, desc) in zip(ax, PHOTOS.items()):
        img = plt.imread(os.path.join(DATA, name))
        h, w = img.shape[:2]
        rows.append({"file": name, "what": desc, "width_px": w, "height_px": h,
                     "size_kb": round(os.path.getsize(os.path.join(DATA, name)) / 1024, 1),
                     "mean_brightness": round(float(img[..., :3].mean()), 1)})
        a.imshow(img)
        a.set_title(f"{name}\n{w} x {h} px", fontsize=8)
        a.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "dataset_overview.png"), dpi=90)
    plt.close(fig)
    return rows


def hand_label_stats():
    """How well do the hand labels agree with the tile grid (posts on tile corners, 50 cm apart)?"""
    p = np.array(HAND_LABELS_PX, dtype=float)
    spacing_px = np.linalg.norm(np.diff(p, axis=0), axis=1)
    scale = spacing_px.mean() / TRACK_MEASURED["post_spacing_cm"]
    spacing_cm = spacing_px / scale
    offset_cm = (p[:, 0] - p[:, 0].mean()) / scale        # sideways scatter of the posts
    return {
        "hand_labels_px": HAND_LABELS_PX,
        "scale_px_per_cm": round(float(scale), 3),
        "post_spacing_px": np.round(spacing_px, 1).tolist(),
        "post_spacing_cm": np.round(spacing_cm, 1).tolist(),
        "spacing_error_vs_tile_grid_cm": np.round(spacing_cm - TRACK_MEASURED["post_spacing_cm"], 1).tolist(),
        "posts_off_straight_line_cm": np.round(offset_cm, 1).tolist(),
    }


def contrast_stats():
    """Brightness contrast of post pixels vs floor pixels on the top view (used to pick the threshold)."""
    img = plt.imread(os.path.join(DATA, "top_view.jpeg")).astype(float)
    H, W = img.shape[:2]
    brightness = img.mean(axis=2)
    floor = np.kron(core.ndimage.median_filter(brightness[::4, ::4], size=101), np.ones((4, 4)))[:H, :W]
    contrast = brightness - floor
    _, _, extra = core.detect_posts(img, core.TRACK, core.DETECT)
    post_mask = np.zeros((H, W), bool)
    for bl in extra["blobs"]:
        post_mask[bl["pts"][:, 1], bl["pts"][:, 0]] = True
    floor_mask = ~post_mask & (extra["labels"] == 0)

    fig, ax = plt.subplots(figsize=(8, 4))
    bins = np.linspace(-60, 100, 161)
    ax.hist(contrast[floor_mask], bins=bins, density=True, alpha=0.6, label="floor pixels", color="gray")
    ax.hist(contrast[post_mask], bins=bins, density=True, alpha=0.6, label="post pixels (detected)", color="orange")
    for thr, c in [(5, "red"), (10, "green"), (30, "red")]:
        ax.axvline(thr, c=c, ls="--", lw=1.5, label=f"threshold {thr}")
    ax.set_xlabel("brightness minus local floor brightness")
    ax.set_ylabel("share of pixels")
    ax.set_title("Top view: how much brighter than the floor are the posts?")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "top_view_contrast.png"), dpi=90)
    plt.close(fig)
    pct = lambda a: np.round(np.percentile(a, [5, 25, 50, 75, 95]), 1).tolist()  # noqa: E731
    return {
        "floor_contrast_percentiles_5_25_50_75_95": pct(contrast[floor_mask]),
        "post_contrast_percentiles_5_25_50_75_95": pct(contrast[post_mask]),
        "floor_pixels_above_threshold_10_percent": round(float((contrast[floor_mask] > 10).mean() * 100), 1),
        "note": "post pixels = pixels of the detected posts, so they pass the threshold by construction",
    }


def video_stats():
    cap = cv2.VideoCapture(os.path.join(DATA, VIDEO))
    fps = cap.get(cv2.CAP_PROP_FPS)
    brightness, sharpness, thumbs = [], [], []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness.append(float(gray.mean()))
        # sharpness = how strong the fine edges are (variance of the Laplacian); low = blurred
        sharpness.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
        if i % int(round(2 * fps)) == 0:
            thumbs.append((i / fps, cv2.cvtColor(cv2.resize(frame, (120, 215)), cv2.COLOR_BGR2RGB)))
        i += 1
    h, w = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    t = np.arange(len(brightness)) / fps
    brightness, sharpness = np.array(brightness), np.array(sharpness)

    phases = []
    for name, a, b, _ in VIDEO_PHASES:
        m = (t >= a) & (t < b)
        phases.append({"phase": name, "from_s": a, "to_s": min(b, round(float(t[-1]), 2)),
                       "frames": int(m.sum()),
                       "brightness_mean": round(float(brightness[m].mean()), 1),
                       "sharpness_median": round(float(np.median(sharpness[m])), 1),
                       "low_detail_frames": int((sharpness[m] < LOW_DETAIL).sum())})

    # --- picture: brightness + sharpness over time, phases as coloured bands
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for a_ in ax:
        for name, a, b, c in VIDEO_PHASES:
            a_.axvspan(a, b, color=c, alpha=0.12)
    for name, a, b, c in VIDEO_PHASES:
        ax[0].text((a + b) / 2, 1.02, name, transform=ax[0].get_xaxis_transform(), ha="center", color=c)
    ax[0].plot(t, brightness, c="black", lw=1)
    ax[0].set_ylabel("mean brightness (0..255)")
    ax[1].semilogy(t, sharpness, c="teal", lw=0.8)
    ax[1].axhline(LOW_DETAIL, c="red", ls="--", lw=1, label=f"low detail (< {LOW_DETAIL:g})")
    ax[1].set_ylabel("sharpness (log scale)")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="upper left", fontsize=8)
    for a_ in ax:
        a_.grid(alpha=0.3)
    fig.suptitle(f"{VIDEO}: {len(t)} frames, {fps:.0f} fps, {w} x {h} px")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "video_stats.png"), dpi=90)
    plt.close(fig)

    # --- picture: one frame every 2 s, frame colour = phase
    cols = 12
    rows = int(np.ceil(len(thumbs) / cols))
    fig, ax = plt.subplots(rows, cols, figsize=(cols * 1.3, rows * 2.4))
    for a_ in ax.flat:
        a_.axis("off")
    for a_, (ts, im) in zip(ax.flat, thumbs):
        colour = next(c for name, a, b, c in VIDEO_PHASES if a <= ts < b)
        a_.imshow(im)
        a_.set_title(f"{ts:.0f} s", fontsize=8, color=colour)
        for s in a_.spines.values():
            s.set_visible(True)
            s.set_color(colour)
            s.set_linewidth(3)
        a_.axis("on")
        a_.set_xticks([])
        a_.set_yticks([])
    fig.suptitle("Drive video, one frame every 2 s: " +
                 ", ".join(f"{n} {a:g}-{min(b, 45):g} s" for n, a, b, _ in VIDEO_PHASES), fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "video_frames.png"), dpi=90)
    plt.close(fig)

    return {
        "file": VIDEO,
        "size_mb": round(os.path.getsize(os.path.join(DATA, VIDEO)) / 1e6, 2),
        "width_px": w, "height_px": h, "fps": round(float(fps), 2),
        "frames": int(len(t)), "duration_s": round(float(len(t) / fps), 2),
        "brightness_mean": round(float(brightness.mean()), 1),
        "brightness_min_max": [round(float(brightness.min()), 1), round(float(brightness.max()), 1)],
        "sharpness_median": round(float(np.median(sharpness)), 1),
        "low_detail_frames": int((sharpness < LOW_DETAIL).sum()),
        "low_detail_threshold": LOW_DETAIL,
        "phases": phases,
        "note": "Compressed by WhatsApp; phases marked by watching the video.",
    }


if __name__ == "__main__":
    os.makedirs(IMG, exist_ok=True)
    stats = {
        "photos": photo_stats(),
        "track_measured": TRACK_MEASURED,
        "hand_labels": hand_label_stats(),
        "top_view_contrast": contrast_stats(),
        "video": video_stats(),
    }
    with open(os.path.join(OUT, "dataset_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)
    print(json.dumps(stats, indent=2))
    print("Saved docs/dataset_stats.json and docs/images/*.png")
