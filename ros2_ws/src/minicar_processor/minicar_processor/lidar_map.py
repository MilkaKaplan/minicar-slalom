"""LiDAR version (no ROS here): build a 3D map from iPhone depth frames + phone poses,
then find the ground, the posts and the wall WITHOUT knowing the track in advance.

Input per frame (from the Stray Scanner app):
    depth       256 x 192 distances in millimetres (iPhone LiDAR)
    confidence  0 / 1 / 2 = low / medium / high for every depth pixel
    pose        where the phone was (x, y, z in metres) and how it was turned (quaternion),
                in ARKit's world frame: y points UP, x and z are horizontal.

Steps:
  1. every depth pixel -> 3D point in the camera -> 3D point in the world   (MapBuilder)
  2. ground  = the most common height, then a plane fitted through it
  3. posts   = compact things 5..40 cm above the ground, close to where the car drove,
               pieces of one object merged, and only those standing in one line
  4. wall    = the first big thing behind the last post
  5. track frame: x = forward from the start along the posts, y = sideways (+ = left)
"""
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

# Knobs
MIN_CONFIDENCE = 2        # only "high confidence" LiDAR points
MAX_RANGE_M = 2.0         # iPhone LiDAR is accurate only close by
POST_MIN_H, POST_MAX_H = 0.05, 0.40     # m above ground
NEAR_DRIVE_M = 0.5        # posts must be within this distance of the drive
GRID_M = 0.02             # 2 cm cells for clustering
MIN_CELL_POINTS = 30
MIN_POST_POINTS = 2000
MAX_POST_SIZE_M = 0.30


def quat_to_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


class MapBuilder:
    """Collects the 3D points of all frames in one world frame."""

    def __init__(self, fx, fy, cx, cy, width, height):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.v, self.u = np.mgrid[0:height, 0:width]
        self.chunks = []
        self.poses = []

    def add(self, depth_mm, confidence, position, quaternion):
        d = depth_mm.astype(np.float64) / 1000.0
        m = (confidence >= MIN_CONFIDENCE) & (d > 0.1) & (d < MAX_RANGE_M)
        z = d[m]
        x = (self.u[m] - self.cx) * z / self.fx
        y = (self.v[m] - self.cy) * z / self.fy
        # pixel -> camera -> world. (Tested on the data: the poses rotate these camera
        # coordinates straight into the world frame; other axis conventions smear the map.)
        R = quat_to_matrix(*quaternion)
        self.chunks.append(np.column_stack([x, y, z]) @ R.T + np.asarray(position))
        self.poses.append(np.asarray(position, dtype=float))

    def points(self, voxel=0.01):
        """All points, thinned to one point per 1 cm cube."""
        P = np.vstack(self.chunks)
        key = np.floor(P / voxel).astype(np.int64)
        _, first, counts = np.unique(key, axis=0, return_index=True, return_counts=True)
        return P[first], counts           # counts = how many raw points fell into each cube


def fit_ground(P):
    """Ground = most common height (y), refined by a least-squares plane y = a x + b z + c."""
    h, edges = np.histogram(P[:, 1], bins=400, range=(-1.0, 1.0))
    y0 = edges[np.argmax(h)]
    band = np.abs(P[:, 1] - y0) < 0.02
    A = np.column_stack([P[band, 0], P[band, 2], np.ones(band.sum())])
    coef, *_ = np.linalg.lstsq(A, P[band, 1], rcond=None)
    height = P[:, 1] - (coef[0] * P[:, 0] + coef[1] * P[:, 2] + coef[2])   # height above the ground
    thickness = float(np.std(height[np.abs(height) < 0.05]))
    return coef, height, thickness


def find_posts(P, weight, height, drive_xz):
    """Compact clusters of points POST_MIN_H..POST_MAX_H above ground near the drive.
    Returns list of (x, z) centres in metres (world frame) and the cluster sizes."""
    near = cKDTree(drive_xz).query(P[:, [0, 2]])[0] < NEAR_DRIVE_M
    m = (height > POST_MIN_H) & (height < POST_MAX_H) & near
    Q, w = P[m][:, [0, 2]], weight[m]
    o = Q.min(axis=0)
    ix = np.floor((Q - o) / GRID_M).astype(int)
    cnt = np.zeros(ix.max(axis=0) + 1)
    np.add.at(cnt, (ix[:, 0], ix[:, 1]), w)
    labels, n = ndimage.label(cnt >= MIN_CELL_POINTS, structure=np.ones((3, 3)))
    posts = []
    for k in range(1, n + 1):
        mm = labels == k
        cells, cw = np.argwhere(mm), cnt[mm]
        size = (cells.max(axis=0) - cells.min(axis=0) + 1) * GRID_M
        if cw.sum() < MIN_POST_POINTS or size.max() > MAX_POST_SIZE_M:
            continue
        # centre = mean of the occupied cells (each cell counts once). Weighting by the number
        # of points would pull the centre towards the side the phone saw from close by.
        posts.append({"xz": (cells + 0.5).mean(axis=0) * GRID_M + o,
                      "points": int(cw.sum()), "size_m": size.round(3).tolist()})
    return posts


def merge_close(posts, dist=0.20):
    """Pieces closer than `dist` belong to the same object: merge them.
    A post is often seen as two pieces (its two sides, seen on the way there and back,
    a few cm apart because the phone's position drifts), so the centre is the plain
    mean of the pieces."""
    groups = []
    for p in sorted(posts, key=lambda q: -q["points"]):
        for g in groups:
            if np.linalg.norm(np.mean([q["xz"] for q in g], axis=0) - p["xz"]) < dist:
                g.append(p)
                break
        else:
            groups.append([p])
    return [{"xz": np.mean([q["xz"] for q in g], axis=0), "points": sum(q["points"] for q in g),
             "pieces": len(g), "size_m": g[0]["size_m"]} for g in groups]


def on_one_line(posts, tol=0.10):
    """Slalom posts stand in a line. Try the line through every pair of candidates
    (RANSAC-like) and keep the candidates within `tol` metres of the best line."""
    xz = np.array([p["xz"] for p in posts])
    best = None
    for i in range(len(xz)):
        for j in range(i + 1, len(xz)):
            d = xz[j] - xz[i]
            if np.linalg.norm(d) < 0.3:
                continue
            n = np.array([-d[1], d[0]]) / np.linalg.norm(d)        # normal of the line
            inl = np.abs((xz - xz[i]) @ n) < tol
            if best is None or inl.sum() > best.sum():
                best = inl
    return [p for p, k in zip(posts, best) if k]


def track_frame(posts_xz, start_xz):
    """x = forward from the start along the line of posts, y = sideways (+ = left of the car).
    World y is up, so 'left' of a forward direction f = (fx, fz) is (fz, -fx)."""
    c = posts_xz.mean(axis=0)
    _, vec = np.linalg.eigh(np.cov((posts_xz - c).T))
    f = vec[:, 1]
    if (c - start_xz) @ f < 0:                    # point away from the start
        f = -f
    left = np.array([f[1], -f[0]])

    def to_track(xz):
        d = np.atleast_2d(xz) - start_xz
        return np.column_stack([d @ f, d @ left])

    def to_world(xy):
        xy = np.atleast_2d(xy)
        return start_xz + np.outer(xy[:, 0], f) + np.outer(xy[:, 1], left)

    return to_track, to_world


def find_wall(P, height, to_track, last_post_x, min_width=0.5):
    """Distance (along x) to the wall behind the last post. A wall is WIDE: at that distance
    obstacle points must cover at least `min_width` metres across the corridor
    (a stone or a leaf does not count)."""
    t = to_track(P[:, [0, 2]])
    m = (height > POST_MIN_H) & (height < 0.6) & (np.abs(t[:, 1]) < 0.6) & (t[:, 0] > last_post_x + 0.10)
    if m.sum() < 500:
        return None
    x, y = t[m, 0], t[m, 1]
    for x0 in np.arange(last_post_x + 0.10, x.max(), 0.02):          # walk forward in 2 cm steps
        in_slice = (x >= x0) & (x < x0 + 0.04)
        cells = np.unique(np.floor(y[in_slice] / 0.05))              # 5 cm wide cells across
        if len(cells) * 0.05 >= min_width:
            return float(x0)
    return None


def analyse(builder, post_radius_m=0.035):
    """Run the whole map analysis. Returns a dict (metres) for planning and plotting."""
    P, weight = builder.points()
    coef, height, thickness = fit_ground(P)
    drive = np.array(builder.poses)
    drive_xz = drive[:, [0, 2]]
    candidates = merge_close(find_posts(P, weight, height, drive_xz))
    posts = on_one_line(candidates)
    if len(posts) < 2:
        raise RuntimeError(f"only {len(posts)} posts found")
    start = drive_xz[0]
    to_track, to_world = track_frame(np.array([p["xz"] for p in posts]), start)
    posts_t = to_track(np.array([p["xz"] for p in posts]))
    order = np.argsort(posts_t[:, 0])
    posts_t = posts_t[order]
    posts = [posts[i] for i in order]
    wall_x = find_wall(P, height, to_track, posts_t[-1, 0])
    drive_t = to_track(drive_xz)
    # which side did the driver pass post 1 on? (+1 = post on the right = car on the left)
    first = np.argmin(np.abs(drive_t[: len(drive_t) // 2, 0] - posts_t[0, 0]))
    first_side = 1 if drive_t[first, 1] > posts_t[0, 1] else -1
    return {
        "points": P, "height": height, "to_track": to_track, "to_world": to_world,
        "ground_plane": coef.tolist(), "ground_thickness_m": thickness,
        "camera_above_ground_m": float(drive[:, 1].mean() - coef[2]),
        "posts_track_m": posts_t, "posts_info": posts, "wall_x_m": wall_x,
        "n_candidates": len(candidates),
        "candidates_track_m": to_track(np.array([p["xz"] for p in candidates])),
        "drive_track_m": drive_t, "first_side": first_side, "post_radius_m": post_radius_m,
    }


# =====================================================================
# Planning on the LiDAR map + comparison with the real drive
# =====================================================================
def plan_on_map(res, car, half_corridor_m=0.5):
    """Plan the slalom through the detected posts (core.plan_path) in centimetres.
    Known in advance are only OUR car and the RULES (corridor width);
    posts, wall and distances come from the map."""
    from . import core
    posts_cm = res["posts_track_m"] * 100
    track = {
        "post_r": res["post_radius_m"] * 100,
        "half_corridor": half_corridor_m * 100,
        "post4_to_wall": (res["wall_x_m"] - res["posts_track_m"][-1, 0]) * 100 if res["wall_x_m"] else 100.0,
    }
    parts, A = core.plan_path(posts_cm, track, car, first_side=res["first_side"])
    checks = core.check_path(parts, posts_cm, track, car)
    return parts, A, track, checks


def compare_drive(res, parts, track, car):
    """How far was the real drive (the phone) from the plan, and how close to the posts?"""
    plan = np.column_stack([np.concatenate([p[0] for p in parts.values()]),
                            np.concatenate([p[1] for p in parts.values()])])
    drive = res["drive_track_m"] * 100
    dev = cKDTree(plan).query(drive)[0]
    posts = res["posts_track_m"] * 100
    gaps = [float(np.min(np.hypot(drive[:, 0] - px, drive[:, 1] - py)) - track["post_r"] - car["half_width"])
            for px, py in posts]
    return {
        "drive_length_cm": round(float(np.sum(np.linalg.norm(np.diff(drive, axis=0), axis=1))), 1),
        "deviation_from_plan_mean_cm": round(float(dev.mean()), 1),
        "deviation_from_plan_max_cm": round(float(dev.max()), 1),
        "drive_gap_to_each_post_cm": [round(g, 1) for g in gaps],
        "drive_max_sideways_cm": round(float(np.max(np.abs(drive[:, 1]))), 1),
    }, dev


def save_map_plot(res, parts, track, checks, cmp, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    P, h = res["points"], res["height"]
    t = res["to_track"](P[:, [0, 2]]) * 100
    keep = (t[:, 0] > -30) & (t[:, 0] < 480) & (np.abs(t[:, 1]) < 90)
    ground = keep & (np.abs(h) < 0.03)
    above = keep & (h > 0.03) & (h < 0.6)
    posts = res["posts_track_m"] * 100
    cand = res["candidates_track_m"] * 100

    fig, ax = plt.subplots(1, 2, figsize=(13, 10), gridspec_kw={"width_ratios": [1.1, 1]})
    a = ax[0]
    a.scatter(t[ground, 1][::8], t[ground, 0][::8], s=0.3, c="#d9d9d9")
    sc = a.scatter(t[above, 1], t[above, 0], s=0.6, c=h[above] * 100, cmap="viridis", vmin=0, vmax=40)
    fig.colorbar(sc, ax=a, shrink=0.5, label="height above ground [cm]")
    a.scatter(cand[:, 1], cand[:, 0], marker="x", c="gray", s=60)
    for px, py in posts:
        a.add_patch(plt.Circle((py, px), track["post_r"], color="red"))
    if res["wall_x_m"]:
        a.axhline(res["wall_x_m"] * 100, c="brown", lw=3)
    a.plot(0, 0, "k^", ms=10)
    a.set_title("LiDAR map from the iPhone recording")
    _style(a, track, res)
    from matplotlib.lines import Line2D
    a.legend(handles=[
        Line2D([], [], ls="none", marker="o", ms=4, c="#bbbbbb", label="ground"),
        Line2D([], [], ls="none", marker="o", ms=4, c="#2a9d8f", label="above ground (colour = height)"),
        Line2D([], [], ls="none", marker="o", ms=7, c="red", label=f"posts found: {len(posts)}"),
        Line2D([], [], ls="none", marker="x", ms=7, c="gray", label="rejected candidates"),
        Line2D([], [], c="brown", lw=3, label="wall found"),
        Line2D([], [], ls="none", marker="^", ms=7, c="black", label="start"),
    ], loc="lower left", fontsize=7)

    a = ax[1]
    for px, py in posts:
        a.add_patch(plt.Circle((py, px), track["post_r"], color="red"))
    if res["wall_x_m"]:
        a.axhline(res["wall_x_m"] * 100, c="brown", lw=3)
    colours = {"there": "black", "U-turn": "red", "back": "purple"}
    for name, (x, y) in parts.items():
        a.plot(y, x, c=colours[name], lw=2, label=f"plan: {name}")
    d = res["drive_track_m"] * 100
    a.plot(d[:, 1], d[:, 0], c="deepskyblue", lw=1.5, alpha=0.9, label="real drive (phone)")
    a.plot(0, 0, "k^", ms=10)
    a.set_title(f"Plan vs real drive: plan min gap {min(checks['gap_to_each_post_cm'])} cm, "
                f"drive {cmp['deviation_from_plan_mean_cm']} cm from plan on average")
    _style(a, track, res)
    a.legend(loc="lower left", fontsize=7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def _style(a, track, res):
    hc = track["half_corridor"]
    a.axvline(-hc, c="gray", ls="--", lw=1)
    a.axvline(hc, c="gray", ls="--", lw=1)
    a.set_xlim(90, -90)                     # + y (left) drawn on the left
    a.set_ylim(-30, (res["wall_x_m"] or 4.5) * 100 + 30)
    a.set_aspect("equal")
    a.set_xlabel("y, sideways [cm] (+ = left)")
    a.set_ylabel("x, forward from start [cm]")
    a.grid(alpha=0.3)


def save_drive_animation(res, parts, track, car_xy_cm, out_gif, car_half_width_cm=2.5,
                         car_length_cm=8.0, frames=120, fps=12):
    """GIF: the simulated car drives the planned slalom ON the LiDAR map of the yard,
    next to the path the phone really took."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    P, h = res["points"], res["height"]
    t = res["to_track"](P[:, [0, 2]]) * 100
    keep = (t[:, 0] > -30) & (t[:, 0] < 480) & (np.abs(t[:, 1]) < 90)
    ground = keep & (np.abs(h) < 0.03)
    above = keep & (h > 0.03) & (h < 0.6)
    posts = res["posts_track_m"] * 100
    drive = res["drive_track_m"] * 100
    car = np.asarray(car_xy_cm, dtype=float)

    fig, ax = plt.subplots(figsize=(5, 9))
    ax.scatter(t[ground, 1][::10], t[ground, 0][::10], s=0.3, c="#dddddd")
    ax.scatter(t[above, 1], t[above, 0], s=0.5, c=h[above] * 100, cmap="viridis", vmin=0, vmax=40)
    for px, py in posts:
        ax.add_patch(plt.Circle((py, px), track["post_r"], color="red"))
    if res["wall_x_m"]:
        ax.axhline(res["wall_x_m"] * 100, c="brown", lw=3)
    for name, (x, y) in parts.items():
        ax.plot(y, x, "--", c="black", lw=0.8)
    ax.plot(drive[:, 1], drive[:, 0], c="deepskyblue", lw=1, alpha=0.6, label="real drive (phone)")
    trail, = ax.plot([], [], c="orange", lw=2.5, label="simulated car")
    body = Polygon(np.zeros((4, 2)), closed=True, fc="orange", ec="black", zorder=5)
    ax.add_patch(body)
    title = ax.set_title("")
    _style(ax, track, res)
    ax.legend(loc="lower left", fontsize=7)

    heading = np.arctan2(np.gradient(car[:, 1]), np.gradient(car[:, 0]))
    steps = np.linspace(0, len(car) - 1, frames).astype(int)
    w, L = car_half_width_cm, car_length_cm

    def update(i):
        k = steps[i]
        c, s = np.cos(heading[k]), np.sin(heading[k])
        corners = np.array([[-L / 2, -w], [L / 2, -w], [L / 2, w], [-L / 2, w]])
        xy = corners @ np.array([[c, s], [-s, c]]) + car[k]
        body.set_xy(xy[:, ::-1])                       # plot is (y, x)
        trail.set_data(car[: k + 1, 1], car[: k + 1, 0])
        title.set_text(f"Car on the LiDAR map  ({100 * k // (len(car) - 1)} %)")
        return body, trail, title

    FuncAnimation(fig, update, frames=frames, blit=False).save(out_gif, writer=PillowWriter(fps=fps))
    plt.close(fig)
