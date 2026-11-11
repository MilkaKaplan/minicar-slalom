"""Processing core of the minicar slalom pipeline (no ROS here).

    top-view photo --> post bases (px) --> centimetres --> slalom there + U-turn + back
                   --> safety checks --> picture

Used by the ROS 2 processor node and by scripts/step4_pipeline.py.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

# Default track measurements (centimetres) - the "metadata" of our dataset
TRACK = {
    "spacing": 50.0,          # between posts
    "post_r": 13.0 / 2,       # post radius
    "half_corridor": 50.0,    # from post line to corridor edge
    "start_to_post1": 100.0,  # car starts 1 m before the first post
    "post4_to_wall": 50.0,
}
# Default car and plan: car 5 cm wide, wanted gap post-edge -> car-side
CAR = {"half_width": 5.0 / 2, "gap": 10.0}
# Detection knobs (chosen in step 3: 10 is the best contrast threshold)
DETECT = {"min_contrast": 10, "max_colour": 40, "clean_steps": 8, "min_area": 2000}


# =====================================================================
# 1. Detection (from step 3)
# =====================================================================
def detect_posts(img, track, knobs):
    """Return post bases in pixels (sorted from the top of the photo), the scale (px per cm)
    and some extra data for the picture."""
    H, W = img.shape[:2]
    brightness = img.mean(axis=2)
    colourfulness = img.max(axis=2) - img.min(axis=2)

    # contrast = brightness minus the floor around (median over a big window)
    floor = np.kron(ndimage.median_filter(brightness[::4, ::4], size=101), np.ones((4, 4)))[:H, :W]
    contrast = brightness - floor

    mask = (contrast > knobs["min_contrast"]) & (colourfulness < knobs["max_colour"])
    mask = ndimage.binary_opening(mask, iterations=knobs["clean_steps"])
    mask = ndimage.binary_fill_holes(mask)
    labels, _ = ndimage.label(mask)

    blobs = []
    for k in range(1, labels.max() + 1):
        ys, xs = np.nonzero(labels == k)
        if len(xs) < knobs["min_area"] or xs.min() <= 2 or xs.max() >= W - 3:
            continue
        centre = np.array([xs.mean(), ys.mean()])
        _, vectors = np.linalg.eigh(np.cov(np.vstack([xs, ys])))
        blobs.append({"pts": np.column_stack([xs, ys]), "centre": centre, "axis": vectors[:, 1]})

    # point under the camera = closest point to all post axes
    M, b = np.zeros((2, 2)), np.zeros(2)
    for bl in blobs:
        P = np.eye(2) - np.outer(bl["axis"], bl["axis"])
        M += P
        b += P @ bl["centre"]
    under_camera = np.linalg.solve(M, b)

    # end of each blob closer to the camera point = edge of the base
    near_ends, inwards = [], []
    for bl in blobs:
        t = (bl["pts"] - bl["centre"]) @ bl["axis"]
        e1, e2 = bl["centre"] + t.min() * bl["axis"], bl["centre"] + t.max() * bl["axis"]
        near, far = (e1, e2) if np.linalg.norm(e1 - under_camera) < np.linalg.norm(e2 - under_camera) else (e2, e1)
        near_ends.append(near)
        inwards.append((far - near) / np.linalg.norm(far - near))
    order = np.argsort([e[1] for e in near_ends])
    near_ends, inwards = np.array(near_ends)[order], np.array(inwards)[order]

    # scale from the photo itself: posts are track["spacing"] cm apart
    scale = np.mean(np.linalg.norm(np.diff(near_ends, axis=0), axis=1)) / track["spacing"]
    bases = near_ends + inwards * track["post_r"] * scale       # move in by one radius
    scale = np.mean(np.linalg.norm(np.diff(bases, axis=0), axis=1)) / track["spacing"]

    extra = {"labels": labels, "blobs": [blobs[i] for i in order], "under_camera": under_camera}
    return bases, scale, extra


# =====================================================================
# 2. Pixels -> centimetres
# =====================================================================
def make_frame(bases_px, scale, track):
    """Track coordinates: x = forward from the start, y = sideways (+ = car's left).
    Built from the detected posts: the post line is the line fitted through them."""
    mean = bases_px.mean(axis=0)
    _, vectors = np.linalg.eigh(np.cov(bases_px.T))
    forward = vectors[:, 1]
    if forward[1] < 0:                       # the car drives down the photo
        forward = -forward
    left = np.array([forward[1], -forward[0]])    # driving down the photo, left = photo's right
    origin = mean + ((bases_px[0] - mean) @ forward) * forward   # post 1 projected on the line
    return {"origin": origin, "forward": forward, "left": left, "scale": scale,
            "x0": track["start_to_post1"]}


def px_to_cm(p, fr):
    d = np.atleast_2d(p) - fr["origin"]
    return np.column_stack([fr["x0"] + d @ fr["forward"] / fr["scale"], d @ fr["left"] / fr["scale"]])


def cm_to_px(x, y, fr):
    return (fr["origin"] + np.outer(np.asarray(x) - fr["x0"], fr["forward"]) * fr["scale"]
            + np.outer(y, fr["left"]) * fr["scale"])


# =====================================================================
# 3. Path planning (from step 2, now through the DETECTED posts)
# =====================================================================
def half_cosine(x0, y0, x1, y1, n=400):
    """Smooth S-shaped piece from (x0, y0) to (x1, y1), driving straight at both ends."""
    x = np.linspace(x0, x1, n)
    t = (x - x0) / (x1 - x0)
    return x, y0 + (y1 - y0) * (1 - np.cos(np.pi * t)) / 2


def plan_path(posts_cm, track, car, first_side=1):
    """There: pass post 1 on the left (first_side=+1) or right (-1), then alternate.
    U-turn around the last post. Back: every post on the other side.
    Returns dict part -> (x, y)."""
    A = track["post_r"] + car["gap"] + car["half_width"]       # swing from the post centre
    side = np.array([first_side if i % 2 == 0 else -first_side for i in range(len(posts_cm))])  # +1 = left
    way_there = np.column_stack([posts_cm[:, 0], posts_cm[:, 1] + side * A])

    xs, ys = [], []
    points = np.vstack([[0.0, 0.0], way_there])        # start, then one waypoint per post
    for (xa, ya), (xb, yb) in zip(points[:-1], points[1:]):
        x, y = half_cosine(xa, ya, xb, yb)
        xs.append(x[:-1]); ys.append(y[:-1])
    xs.append(points[-1:, 0]); ys.append(points[-1:, 1])
    there = (np.concatenate(xs), np.concatenate(ys))

    # U-turn: half circle around the last post, from its "there" side to the other side
    cx, cy = posts_cm[-1]
    s = side[-1]
    angle = np.linspace(0, np.pi, 400)
    turn = (cx + A * np.sin(angle), cy + s * A * np.cos(angle))

    # Back: mirror of "there" around every post (other side), in reverse order
    way_back = np.column_stack([posts_cm[:, 0], posts_cm[:, 1] - side * A])[::-1]
    points = np.vstack([way_back, [0.0, 0.0]])
    xs, ys = [], []
    for (xa, ya), (xb, yb) in zip(points[:-1], points[1:]):
        x, y = half_cosine(xa, ya, xb, yb)
        xs.append(x[:-1]); ys.append(y[:-1])
    xs.append(points[-1:, 0]); ys.append(points[-1:, 1])
    back = (np.concatenate(xs), np.concatenate(ys))
    return {"there": there, "U-turn": turn, "back": back}, A


# =====================================================================
# 4. Checks
# =====================================================================
def tightest_radius(px, py):
    dx, dy = np.gradient(px), np.gradient(py)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    k = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
    return float(1 / k[2:-2].max())


def check_path(parts, posts_cm, track, car):
    x = np.concatenate([p[0] for p in parts.values()])
    y = np.concatenate([p[1] for p in parts.values()])
    wall_x = posts_cm[-1, 0] + track["post4_to_wall"]
    gaps = [float(np.min(np.hypot(x - px, y - py)) - track["post_r"] - car["half_width"])
            for px, py in posts_cm]
    res = {
        "gap_to_each_post_cm": [round(g, 1) for g in gaps],
        "gap_to_corridor_edge_cm": round(track["half_corridor"] - (np.max(np.abs(y)) + car["half_width"]), 1),
        "gap_to_wall_cm": round(wall_x - (np.max(x) + car["half_width"]), 1),
        "tightest_turn_cm": {name: round(tightest_radius(*p), 1) for name, p in parts.items()},
        "path_length_cm": round(float(np.sum(np.hypot(np.diff(x), np.diff(y)))), 0),
    }
    res["no_collision"] = bool(min(gaps) > 0 and res["gap_to_corridor_edge_cm"] > 0 and res["gap_to_wall_cm"] > 0)
    return res


# =====================================================================
# 5. Picture
# =====================================================================
COLOURS = {"there": "black", "U-turn": "red", "back": "purple"}


def draw(img, bases_px, extra, fr, parts, posts_cm, checks, track, path):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 9))
    ax1.imshow(img.astype(np.uint8))
    for bl in extra["blobs"]:
        ax1.scatter(bl["pts"][::40, 0], bl["pts"][::40, 1], s=1, c="cyan", alpha=0.3)
    for name, (x, y) in parts.items():
        p = cm_to_px(x, y, fr)
        ax1.plot(p[:, 0], p[:, 1], c=COLOURS[name], lw=2, label=name)
    ax1.scatter(bases_px[:, 0], bases_px[:, 1], c="lime", marker="x", s=60, zorder=3, label="detected base")
    ax1.set_xlim(0, img.shape[1])
    ax1.set_ylim(img.shape[0], 0)
    ax1.set_title("Photo: detected posts + planned path")
    ax1.legend(loc="lower left", fontsize=8)
    ax1.axis("off")

    for px_, py_ in posts_cm:
        ax2.add_patch(plt.Circle((py_, px_), track["post_r"], color="gray"))
    for name, (x, y) in parts.items():
        ax2.plot(y, x, c=COLOURS[name], lw=2, label=name)
    wall_x = posts_cm[-1, 0] + track["post4_to_wall"]
    ax2.axvline(-track["half_corridor"], c="gray")
    ax2.axvline(track["half_corridor"], c="gray", label="corridor edge")
    ax2.axhline(wall_x, c="brown", lw=3, label="wall")
    ax2.plot(0, 0, "k^", ms=10, label="start / finish")
    ax2.set_xlim(-60, 60)
    ax2.set_ylim(-10, wall_x + 10)
    ax2.invert_xaxis()
    ax2.set_aspect("equal")
    ax2.set_xlabel("y, sideways [cm] (+ = left)")
    ax2.set_ylabel("x, forward [cm]")
    ok = "no collision" if checks["no_collision"] else "COLLISION!"
    ax2.set_title(f"Map from detected posts: min gap {min(checks['gap_to_each_post_cm'])} cm, {ok}")
    ax2.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=3, fontsize=7)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


# =====================================================================
# Everything in one call
# =====================================================================
def run(img, track=TRACK, car=CAR, knobs=DETECT, hand_px=None):
    """Full pipeline on one RGB photo (numpy array, values 0..255).

    Returns (result dict, data for drawing). hand_px = optional hand labels in pixels,
    only used to report the detection error."""
    img = np.asarray(img, dtype=float)
    bases_px, scale, extra = detect_posts(img, track, knobs)
    fr = make_frame(bases_px, scale, track)
    posts_cm = px_to_cm(bases_px, fr)
    parts, A = plan_path(posts_cm, track, car)
    checks = check_path(parts, posts_cm, track, car)
    result = {
        "n_posts": int(len(bases_px)),
        "posts_px": np.round(bases_px, 1).tolist(),
        "posts_cm": np.round(posts_cm, 1).tolist(),
        "scale_px_per_cm": round(float(scale), 3),
        "swing_A_cm": A,
        "checks": checks,
    }
    if hand_px is not None:
        errors = np.linalg.norm(posts_cm - px_to_cm(np.asarray(hand_px, float), fr), axis=1)
        result["detection_error_vs_hand_cm"] = np.round(errors, 1).tolist()
    drawing = {"img": img, "bases_px": bases_px, "extra": extra, "frame": fr,
               "parts": parts, "posts_cm": posts_cm, "checks": checks, "track": track}
    return result, drawing


def save_picture(drawing, path):
    d = drawing
    draw(d["img"], d["bases_px"], d["extra"], d["frame"], d["parts"], d["posts_cm"],
         d["checks"], d["track"], path)


def path_rows(parts, every=10):
    """Path as rows (part, x_cm, y_cm) for a CSV file."""
    rows = []
    for name, (x, y) in parts.items():
        for xi, yi in zip(x[::every], y[::every]):
            rows.append([name, round(float(xi), 2), round(float(yi), 2)])
    return rows
