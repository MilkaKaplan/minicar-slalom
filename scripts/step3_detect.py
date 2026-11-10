"""Step 3: find the post bases on the top-view photo AUTOMATICALLY and compare with the hand labels.

Run from the minicar folder:
    python3 scripts/step3_detect.py

Idea in 5 steps:
  1. "Contrast" = how much brighter each pixel is than the floor around it.
  2. Mask = pixels that are clearly brighter than the floor and not coloured (posts are white/grey).
  3. Clean the mask and split it into separate blobs (connected pieces).
  4. Throw away blobs that are too small or touch the left/right edge (walls, furniture).
  5. For each post blob find where it touches the floor (its base).
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

PHOTO = "data/top_view.jpeg"
OUT_PNG = "results/step3_detection.png"
OUT_CSV = "results/posts_detected.csv"

# Hand labels from step 2 (only used to CHECK the result, not to find the posts)
POSTS_HAND = np.array([[300, 160], [310, 520], [300, 890], [300, 1250]], dtype=float)
PX_PER_CM = 7.27            # from step 2: 50 cm between posts = ~363 px
POST_R_CM = 13.0 / 2

# Tuning knobs
MIN_CONTRAST = 10           # how much brighter than the floor (0..255 brightness units)
MAX_COLOUR = 40             # posts are white/grey: max difference between R, G, B
CLEAN_STEPS = 8             # how strongly to remove thin lines (marble veins, tile seams)
MIN_AREA = 2000             # smallest blob that can be a post, in pixels

img = plt.imread(PHOTO).astype(float)
H, W = img.shape[:2]

# ---------------- 1. contrast ----------------
brightness = img.mean(axis=2)                       # 0 = black, 255 = white
colourfulness = img.max(axis=2) - img.min(axis=2)   # 0 = grey/white, high = coloured (wood table)

# "Floor around each pixel": median brightness in a big window (~400 px).
# Light changes across the photo, so a single threshold for the whole photo does not work.
# (Computed on a 4x smaller copy, because a big median filter is slow.)
small = brightness[::4, ::4]
floor_small = ndimage.median_filter(small, size=101)
floor = np.kron(floor_small, np.ones((4, 4)))[:H, :W]
contrast = brightness - floor

# ---------------- 2. mask ----------------
mask = (contrast > MIN_CONTRAST) & (colourfulness < MAX_COLOUR)

# ---------------- 3. clean + split into blobs ----------------
mask = ndimage.binary_opening(mask, iterations=CLEAN_STEPS)   # removes thin lines and specks
mask = ndimage.binary_fill_holes(mask)                         # fills the dark inside of a roll
labels, n_blobs = ndimage.label(mask)                          # number every separate piece

# ---------------- 4. keep only post-like blobs ----------------
posts = []
for k in range(1, n_blobs + 1):
    ys, xs = np.nonzero(labels == k)
    touches_side = xs.min() <= 2 or xs.max() >= W - 3
    if len(xs) < MIN_AREA or touches_side:
        continue
    centre = np.array([xs.mean(), ys.mean()])
    # Main direction of the blob = direction in which its pixels are most spread out
    # (largest eigenvector of the covariance matrix, "principal axis").
    _, vectors = np.linalg.eigh(np.cov(np.vstack([xs, ys])))
    posts.append({"xs": xs, "ys": ys, "centre": centre, "axis": vectors[:, 1]})

print(f"Blobs found: {n_blobs}, kept as posts: {len(posts)}")

# ---------------- 5. find the base of every post ----------------
# Seen from above, a standing post looks like it is "falling" away from the point
# right under the camera (perspective: the top is closer to the camera).
# So all post axes point to that point; we find it as the point closest to all axes.
M, b = np.zeros((2, 2)), np.zeros(2)
for p in posts:
    d = p["axis"]
    P = np.eye(2) - np.outer(d, d)          # projects onto the direction across the axis
    M += P
    b += P @ p["centre"]
under_camera = np.linalg.solve(M, b)
print(f"Point under the camera (pixels): x = {under_camera[0]:.0f}, y = {under_camera[1]:.0f}")

# The base is the end of the blob CLOSER to that point. That end is the edge of the
# round base, so we move inwards by one post radius to get the base centre.
r_px = POST_R_CM * PX_PER_CM
bases = []
for p in posts:
    pts = np.column_stack([p["xs"], p["ys"]])
    t = (pts - p["centre"]) @ p["axis"]                  # position of each pixel along the axis
    end1 = p["centre"] + t.min() * p["axis"]
    end2 = p["centre"] + t.max() * p["axis"]
    if np.linalg.norm(end1 - under_camera) < np.linalg.norm(end2 - under_camera):
        near, far = end1, end2
    else:
        near, far = end2, end1
    inward = (far - near) / np.linalg.norm(far - near)
    bases.append(near + inward * r_px)
bases = np.array(sorted(bases, key=lambda q: q[1]))       # order: top of photo first

# ---------------- compare with the hand labels ----------------
print("\nPost | detected (px)  | by hand (px) | error")
errors = []
for i, base in enumerate(bases):
    hand = POSTS_HAND[np.argmin(np.linalg.norm(POSTS_HAND - base, axis=1))]
    err_cm = np.linalg.norm(base - hand) / PX_PER_CM
    errors.append(err_cm)
    print(f"  {i + 1}  | ({base[0]:4.0f}, {base[1]:4.0f})   | ({hand[0]:.0f}, {hand[1]:.0f})   | {err_cm:.1f} cm")
if len(bases) == len(POSTS_HAND):
    print(f"All {len(bases)} posts found. Mean error {np.mean(errors):.1f} cm, worst {np.max(errors):.1f} cm")
else:
    print(f"WARNING: expected {len(POSTS_HAND)} posts, found {len(bases)}")

os.makedirs("results", exist_ok=True)
with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["post", "x_px", "y_px"])
    for i, (bx, by) in enumerate(bases, 1):
        w.writerow([i, round(bx, 1), round(by, 1)])
print(f"Saved: {OUT_CSV}")

# ---------------- picture: the 4 stages ----------------
fig, ax = plt.subplots(1, 4, figsize=(16, 8))
ax[0].imshow(img.astype(np.uint8))
ax[0].set_title("1. photo")
ax[1].imshow(contrast, cmap="RdBu_r", vmin=-50, vmax=50)
ax[1].set_title("2. contrast vs floor\n(red = brighter)")
ax[2].imshow(img.astype(np.uint8))
ax[2].imshow(np.ma.masked_equal(labels, 0), cmap="tab20", alpha=0.6)
ax[2].set_title(f"3. blobs ({n_blobs}),\n{len(posts)} kept as posts")
ax[3].imshow(img.astype(np.uint8))
for p in posts:
    c, d = p["centre"], p["axis"]
    ax[3].plot([c[0] - 900 * d[0], c[0] + 900 * d[0]], [c[1] - 900 * d[1], c[1] + 900 * d[1]],
               "y--", lw=0.8)
ax[3].plot(*under_camera, "y*", ms=15, label="under the camera")
ax[3].scatter(POSTS_HAND[:, 0], POSTS_HAND[:, 1], c="red", s=25, label="by hand")
ax[3].scatter(bases[:, 0], bases[:, 1], c="lime", s=25, marker="x", label="detected")
ax[3].set_title("4. post axes -> bases")
ax[3].legend(loc="lower left", fontsize=7)
for a in ax:
    a.set_xlim(0, W)
    a.set_ylim(H, 0)
    a.axis("off")
fig.tight_layout()
fig.savefig(OUT_PNG, dpi=90)
print(f"Saved: {OUT_PNG}")
