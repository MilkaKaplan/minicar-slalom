"""Step 2: hand-marked post bases (pixels) -> centimetres -> slalom there and back -> checks + picture.

Run from the minicar folder:
    python3 scripts/step2_plan.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ---------------- inputs ----------------
PHOTO = "data/top_view.jpeg"

# Post bases marked by hand on the top-view photo (x, y in pixels), first = top of photo
POSTS_PX = np.array([[300, 160], [310, 520], [300, 890], [300, 1250]], dtype=float)

# Measurements of the track (centimetres)
SPACING = 50.0          # between posts
POST_R = 13.0 / 2       # post radius
HALF_CORRIDOR = 50.0    # from post line to corridor edge
START_TO_POST1 = 100.0  # car starts 1 m before the first post
POST4_TO_WALL = 50.0

# Car and plan
CAR_HALF = 5.0 / 2      # car is 5 cm wide
GAP = 10                # wanted gap from post edge to car side (5..10)
A = POST_R + GAP + CAR_HALF   # how far the car centre swings from the post line

# The file name uses GAP, so it must come AFTER GAP is defined
OUT = f"results/step2_slalom_plan_gap{GAP}.png"

# ---------------- 1. pixels -> centimetres ----------------
# Scale: the photo shows posts 50 cm apart, so measure how many pixels that is.
px_per_cm = np.mean(np.diff(POSTS_PX[:, 1])) / SPACING
line_x_px = POSTS_PX[:, 0].mean()          # pixel column of the post line

# Track coordinates: x = forward from the start, y = sideways (+ = left of the car).
# The car drives down the photo, so its left is the photo's right (+x pixels).
posts_cm = np.column_stack([
    START_TO_POST1 + (POSTS_PX[:, 1] - POSTS_PX[0, 1]) / px_per_cm,
    (POSTS_PX[:, 0] - line_x_px) / px_per_cm,
])
first, last = posts_cm[0, 0], posts_cm[-1, 0]
wall_x = last + POST4_TO_WALL

# ---------------- 2. the path: there, U-turn, back ----------------
# The path is now a list of points (x, y), because on the way back x DEcreases.
#
#   THERE  start -> post 4: gentle entry to y = +A at post 1, then the sine
#          (+A at post 1, -A at post 2, +A at post 3, -A at post 4).
#   U-TURN half circle AROUND post 4 with radius A: from y = -A to y = +A.
#          At post 4 the car is already A away and driving straight, so the
#          circle continues the path without a kink.
#   BACK   post 4 -> start: the mirror image of THERE, so every post is passed
#          on the OTHER side (a figure of eight, like the hand drawing).

def entry(x):
    """Gentle sideways move: y = 0 at the start, y = A at post 1, straight at both ends."""
    return A * (1 - np.cos(np.pi * x / first)) / 2


def wave(x):
    """Slalom sine: +A at post 1, -A at post 2, ..."""
    return A * np.cos(np.pi * (x - first) / SPACING)


def there_y(x):
    return np.where(x < first, entry(x), wave(x))


x_there = np.linspace(0, last, 2000)
y_there = there_y(x_there)

angle = np.linspace(-np.pi / 2, np.pi / 2, 400)            # -90 deg .. +90 deg
x_turn = last + A * np.cos(angle)
y_turn = A * np.sin(angle)              # centred on the post line y = 0, like the sine

x_back = x_there[::-1]                                      # from post 4 back to the start
y_back = -y_there[::-1]                                     # mirror: other side of every post

parts = {"there": (x_there, y_there), "U-turn": (x_turn, y_turn), "back": (x_back, y_back)}
x = np.concatenate([p[0] for p in parts.values()])
y = np.concatenate([p[1] for p in parts.values()])
part_of_point = np.concatenate([[name] * len(p[0]) for name, p in parts.items()])

# ---------------- 3. checks ----------------
# Gap to every post along the WHOLE path (not only next to the post)
gaps = [np.min(np.hypot(x - px, y - py)) - POST_R - CAR_HALF for px, py in posts_cm]
gap_side = HALF_CORRIDOR - (np.max(np.abs(y)) + CAR_HALF)
gap_wall = wall_x - (np.max(x) + CAR_HALF)

# Sharpness: radius of the tightest turn. For a path given as points we use the
# general curvature formula with derivatives along the path:
#   curvature = |x' y'' - y' x''| / (x'^2 + y'^2)^1.5,   R = 1 / curvature
def tightest_radius(px, py):
    dx, dy = np.gradient(px), np.gradient(py)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
    return 1 / curvature[2:-2].max()          # skip the very ends (less accurate)


length = np.sum(np.hypot(np.diff(x), np.diff(y)))

print(f"Scale: {px_per_cm:.2f} px per cm")
for i, (p, g) in enumerate(zip(posts_cm, gaps), 1):
    print(f"Post {i}: x = {p[0]:6.1f} cm, y = {p[1]:5.1f} cm, smallest gap to car = {g:4.1f} cm")
print(f"Swing A = {A} cm, gap to corridor edge = {gap_side:.1f} cm, gap to wall = {gap_wall:.1f} cm")
for name, (px, py) in parts.items():
    print(f"Tightest turn on '{name}': R = {tightest_radius(px, py):.1f} cm")
print(f"Path length (there + U-turn + back) = {length:.0f} cm")
ok = min(gaps) > 0 and gap_side > 0 and gap_wall > 0
print("OK: no collisions" if ok else "WARNING: the car touches something!")

# ---------------- 4. picture ----------------
COLOURS = {"there": "black", "U-turn": "red", "back": "purple"}   # like the hand drawing


def body_edges(px, py):
    """Left and right edge of the car body: shift the centre line sideways by CAR_HALF."""
    dx, dy = np.gradient(px), np.gradient(py)
    n = np.hypot(dx, dy)
    nx, ny = -dy / n, dx / n                       # unit vector pointing to the car's left
    return (px + CAR_HALF * nx, py + CAR_HALF * ny), (px - CAR_HALF * nx, py - CAR_HALF * ny)


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 9))

# Left: the photo with marked posts and the path drawn back in pixels
img = plt.imread(PHOTO)
ax1.imshow(img)
for name, (px, py) in parts.items():
    ax1.plot(line_x_px + py * px_per_cm, POSTS_PX[0, 1] + (px - first) * px_per_cm,
             c=COLOURS[name], lw=2, label=name)
ax1.scatter(POSTS_PX[:, 0], POSTS_PX[:, 1], c="red", s=40, zorder=3, label="post base (hand-marked)")
ax1.set_xlim(0, img.shape[1])
ax1.set_ylim(img.shape[0], 0)          # keep the whole photo; the start is above it
ax1.set_title("Top-view photo + planned path")
ax1.legend(loc="lower left", fontsize=8)
ax1.axis("off")

# Right: the same in centimetres, as a map seen from above
for px_, py_ in posts_cm:
    ax2.add_patch(plt.Circle((py_, px_), POST_R, color="gray"))
for name, (px, py) in parts.items():
    (lx, ly), (rx, ry) = body_edges(px, py)
    ax2.plot(ly, lx, c=COLOURS[name], lw=0.6, alpha=0.5)
    ax2.plot(ry, rx, c=COLOURS[name], lw=0.6, alpha=0.5)
    ax2.plot(py, px, c=COLOURS[name], lw=2, label=f"{name} (thin lines = car body)")
ax2.axvline(-HALF_CORRIDOR, c="gray")
ax2.axvline(HALF_CORRIDOR, c="gray", label="corridor edge")
ax2.axhline(wall_x, c="brown", lw=3, label="wall")
ax2.plot(0, 0, "k^", ms=10, label="start / finish")
ax2.set_xlim(-60, 60)
ax2.set_ylim(-10, wall_x + 10)
ax2.invert_xaxis()                      # + y (car's left) is drawn on the left
ax2.set_aspect("equal")
ax2.set_xlabel("y, sideways [cm] (+ = left)")
ax2.set_ylabel("x, forward [cm]")
r_all = min(tightest_radius(px, py) for px, py in parts.values())
ax2.set_title(f"Map: gap {min(gaps):.1f} cm, tightest turn R = {r_all:.0f} cm")
ax2.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=2, fontsize=7)  # below the map
ax2.grid(alpha=0.3)

fig.tight_layout()
os.makedirs("results", exist_ok=True)
fig.savefig(OUT, dpi=100)
print(f"Saved: {OUT}")
