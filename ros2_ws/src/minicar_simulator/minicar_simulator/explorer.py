"""Explorer (no ROS here): a simulated car that does NOT know the track in advance.

The "world" is the real LiDAR map of the yard, but the car never gets it as a whole.
It only has its own virtual LiDAR (range 2 m, field of view 60 deg, like the iPhone) and
builds its own picture of the track while driving:

  1. look:     the map points inside the sensor cone become "seen"
  2. posts:    compact things 5..40 cm high on the corridor centre line, in the seen points
  3. side:     when the first post appears, pass it on the side with more free room
  4. plan:     slalom through the posts seen so far (+ go straight on after the last one);
               re-plan every time a new post appears
  5. wall:     a wide obstacle behind the last post = the end of the track
               -> full plan: U-turn around the last post and slalom back
  6. drive:    kinematic bicycle + pure pursuit, like the car simulator

Known in advance (rules of the competition + our car): the corridor is 1 m wide and the
start points along it, posts stand on its centre line, the car is 5 cm wide.
All units in this file: centimetres, seconds, degrees.
"""
import numpy as np

SENSOR = {"range": 200.0, "fov_deg": 60.0}
# gap 13 cm instead of 10: on an unknown track a post seen only from one side is known
# less precisely (up to ~9 cm), so the explorer keeps a bigger safety margin
CAR = {"half_width": 2.5, "wheelbase": 5.0, "max_steer_deg": 30.0, "speed": 15.0,
       "gap": 13.0, "lookahead": 6.0}
RULES = {"half_corridor": 50.0, "post_line_tol": 20.0, "post_r": 3.5}   # posts stand on the centre line (+-20 cm)
DETECT = {"min_h": 5.0, "max_h": 40.0, "cell": 2.0, "min_cell_points": 2, "min_points": 15,
          "max_size": 30.0, "merge": 20.0, "wall_width": 50.0}
DT = 0.05


# ---------------------------------------------------------------------------
# perception on the points the car has seen
# ---------------------------------------------------------------------------
def line_of_sight(ang, dist, in_cone, bin_deg=1.0, depth=12.0):
    """Occlusion: in every direction (1 deg wide) the sensor only sees the NEAREST surface
    (plus `depth` cm of it). A post hides the posts behind it."""
    vis = np.zeros(len(ang), bool)
    if not in_cone.any():
        return vis
    b = np.floor(ang[in_cone] / bin_deg).astype(int)
    d = dist[in_cone]
    nearest = {}
    for bi, di in zip(b, d):
        if di < nearest.get(bi, np.inf):
            nearest[bi] = di
    near_d = np.array([nearest[bi] for bi in b])
    vis[np.flatnonzero(in_cone)] = d <= near_d + depth
    return vis


def on_one_line(posts, tol=10.0):
    """Slalom posts stand in a line: keep the candidates closest to the best line
    through any two of them (RANSAC idea). With fewer than 3 candidates keep all."""
    if len(posts) < 3:
        return posts
    xy = np.asarray(posts, float)
    best = None
    for i in range(len(xy)):
        for j in range(i + 1, len(xy)):
            d = xy[j] - xy[i]
            if np.linalg.norm(d) < 30:
                continue
            n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
            inl = np.abs((xy - xy[i]) @ n) < tol
            if best is None or inl.sum() > best.sum():
                best = inl
    return [p for p, k in zip(posts, best) if k]


def find_posts(xyh, car_xy=None):
    """Posts in the seen points (N x 3: x, y, height). Returns sorted list of (x, y).
    car_xy: where the car is (the visible surface is the near side of the post)."""
    m = (xyh[:, 2] > DETECT["min_h"]) & (xyh[:, 2] < DETECT["max_h"]) \
        & (np.abs(xyh[:, 1]) < RULES["post_line_tol"]) & (xyh[:, 0] > 20)
    q = xyh[m, :2]
    if len(q) < DETECT["min_points"]:
        return []
    from scipy import ndimage
    o = q.min(axis=0)
    ix = np.floor((q - o) / DETECT["cell"]).astype(int)
    cnt = np.zeros(ix.max(axis=0) + 1)
    np.add.at(cnt, (ix[:, 0], ix[:, 1]), 1)
    lab, n = ndimage.label(cnt >= DETECT["min_cell_points"], structure=np.ones((3, 3)))
    found = []
    for k in range(1, n + 1):
        mm = lab == k
        cells = np.argwhere(mm)
        size = (cells.max(axis=0) - cells.min(axis=0) + 1) * DETECT["cell"]
        if cnt[mm].sum() < DETECT["min_points"] or size.max() > DETECT["max_size"]:
            continue
        c = (cells + 0.5).mean(axis=0) * DETECT["cell"] + o
        # we see the near side of the post: its centre is one radius further away
        if car_xy is not None:
            u = c - np.asarray(car_xy)
            c = c + u / max(np.linalg.norm(u), 1e-6) * RULES["post_r"]
        found.append(c)
    # pieces of the same post (seen from different angles) -> one post
    posts = []
    for p in sorted(found, key=lambda v: v[0]):
        if posts and np.linalg.norm(posts[-1] - p) < DETECT["merge"]:
            posts[-1] = (posts[-1] + p) / 2
        else:
            posts.append(p)
    return on_one_line([tuple(p) for p in posts])


def find_wall(xyh, last_post_x):
    """First x after `last_post_x` where seen obstacles cover >= wall_width across the
    corridor. Posts are narrow (< 30 cm), so only a wall passes this test."""
    m = (xyh[:, 2] > DETECT["min_h"]) & (xyh[:, 2] < 60) & (np.abs(xyh[:, 1]) < 60) \
        & (xyh[:, 0] > last_post_x + 10)
    if m.sum() < 30:
        return None
    x, y = xyh[m, 0], xyh[m, 1]
    for x0 in np.arange(last_post_x + 10, x.max(), 2.0):
        s = (x >= x0) & (x < x0 + 4)
        if len(np.unique(np.floor(y[s] / 5))) * 5 >= DETECT["wall_width"]:
            return float(x0)
    return None


def choose_side(xyh, post):
    """+1 = pass the first post on the left. Choose the side with fewer obstacles."""
    px, py = post
    near = (np.abs(xyh[:, 0] - px) < 40) & (xyh[:, 2] > DETECT["min_h"]) & (xyh[:, 2] < 60)
    left = (near & (xyh[:, 1] > py + 6) & (xyh[:, 1] < RULES["half_corridor"])).sum()
    right = (near & (xyh[:, 1] < py - 6) & (xyh[:, 1] > -RULES["half_corridor"])).sum()
    return 1 if left <= right else -1


# ---------------------------------------------------------------------------
# planning
# ---------------------------------------------------------------------------
def half_cosine(x0, y0, x1, y1, n=150):
    x = np.linspace(x0, x1, n)
    t = (x - x0) / (x1 - x0) if x1 != x0 else np.zeros(n)
    return x, y0 + (y1 - y0) * (1 - np.cos(np.pi * t)) / 2


def plan_there(posts, side, A, line_y):
    """Start -> alternating waypoints of the posts seen so far -> straight on along the line."""
    pts = [(0.0, 0.0)]
    for i, (px, py) in enumerate(posts):
        s = side if i % 2 == 0 else -side
        pts.append((px, py + s * A))
    last_x = posts[-1][0] if posts else 0.0
    pts.append((last_x + 30, line_y))            # cross the line after the last post
    pts.append((last_x + 120, line_y))           # and look further ahead
    xs, ys = [], []
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
        x, y = half_cosine(xa, ya, xb, yb)
        xs.append(x[:-1]); ys.append(y[:-1])
    return np.column_stack([np.concatenate(xs), np.concatenate(ys)])


def plan_full(posts, side, A):
    """Final plan when the wall is seen: there + U-turn around the last post + back."""
    import importlib
    core = importlib.import_module("minicar_processor.core") if _has_core() else None
    if core is not None:
        track = {"post_r": RULES["post_r"], "half_corridor": RULES["half_corridor"], "post4_to_wall": 50.0}
        parts, _ = core.plan_path(np.array(posts), track, {"half_width": CAR["half_width"], "gap": CAR["gap"]},
                                  first_side=side)
        return np.vstack([np.column_stack(p) for p in parts.values()]), len(parts["there"][0])
    raise RuntimeError("minicar_processor.core is needed for the final plan")


def _has_core():
    try:
        import minicar_processor.core  # noqa: F401
        return True
    except ImportError:
        return False


# ---------------------------------------------------------------------------
# the drive
# ---------------------------------------------------------------------------
def explore(world_xyh, sensor=SENSOR, car=CAR, max_time=240.0, snapshot_every=0.5):
    """Drive the unknown track. world_xyh: N x 3 map points (track frame, cm) above the ground.
    Returns a log with the drive, events and snapshots for the animation."""
    A = RULES["post_r"] + car["gap"] + car["half_width"]
    L, max_steer = car["wheelbase"], np.radians(car["max_steer_deg"])
    seen = np.zeros(len(world_xyh), bool)
    x, y, h = 0.0, 0.0, 0.0
    posts, side, line_y, wall_x = [], None, 0.0, None
    plan = plan_there([], 1, A, 0.0)
    final, there_len, idx = False, len(plan), 0
    events, replans = [], 0
    log = {"t": [], "x": [], "y": [], "heading": []}
    snaps = []
    t, next_snap = 0.0, 0.0
    while t < max_time:
        # 1. look
        d = world_xyh[:, :2] - [x, y]
        dist = np.hypot(d[:, 0], d[:, 1])
        ang = np.degrees(np.arctan2(d[:, 1], d[:, 0]) - h)
        ang = (ang + 180) % 360 - 180
        in_cone = (dist < sensor["range"]) & (np.abs(ang) < sensor["fov_deg"] / 2)
        new = in_cone & line_of_sight(ang, dist, in_cone) & ~seen
        if new.any():
            seen |= new
            # 2. posts in everything seen so far
            if not final:
                found = find_posts(world_xyh[seen], (x, y))
                if wall_x is not None:
                    found = [p for p in found if p[0] < wall_x - 15]      # pieces of the wall are not posts
                if len(found) > len(posts):
                    for p in found[len(posts):]:
                        events.append({"t": round(t, 2), "event": f"post {len(posts) + 1} seen",
                                       "post_cm": [round(float(v), 1) for v in p],
                                       "distance_cm": round(float(np.hypot(p[0] - x, p[1] - y)), 1)})
                    posts = [tuple(p) for p in found]            # positions refine as more is seen
                    if side is None:
                        side = choose_side(world_xyh[seen], posts[0])          # 3. choose the side
                        events.append({"t": round(t, 2), "event": "first post: pass on the "
                                       + ("left" if side > 0 else "right")})
                    line_y = float(np.mean([p[1] for p in posts]))
                    plan = plan_there(posts, side, A, line_y)                 # 4. re-plan
                    there_len, replans = len(plan), replans + 1
                    idx = int(np.argmin(np.hypot(plan[:, 0] - x, plan[:, 1] - y)))
                if posts and wall_x is None:
                    wall_x = find_wall(world_xyh[seen], posts[0][0])           # 5. wall? (anywhere after post 1)
                    if wall_x is not None:
                        events.append({"t": round(t, 2), "event": "wall seen",
                                       "wall_x_cm": round(wall_x, 1), "distance_cm": round(float(wall_x - x), 1)})
                        posts = [p for p in posts if p[0] < wall_x - 15]
        if not final and wall_x is not None and x > posts[-1][0] - 20:
            plan, there_len = plan_full(posts, side, A)
            seg = plan[:there_len]
            idx = int(np.argmin(np.hypot(seg[:, 0] - x, seg[:, 1] - y)))
            final, replans = True, replans + 1
            events.append({"t": round(t, 2), "event": "final plan: U-turn around the last post and back"})

        # 6. drive: pure pursuit on the current plan
        win = plan[idx: idx + 60]
        idx += int(np.argmin(np.hypot(win[:, 0] - x, win[:, 1] - y)))
        if final and idx >= len(plan) - 3 and np.hypot(*(plan[-1] - [x, y])) < 3:
            break
        ahead = plan[idx:]
        far = np.nonzero(np.hypot(ahead[:, 0] - x, ahead[:, 1] - y) >= car["lookahead"])[0]
        tx, ty = ahead[far[0]] if len(far) else plan[-1]
        alpha = np.arctan2(ty - y, tx - x) - h
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        steer = float(np.clip(np.arctan2(2 * L * np.sin(alpha), car["lookahead"]), -max_steer, max_steer))
        v = car["speed"]
        x += v * np.cos(h) * DT
        y += v * np.sin(h) * DT
        h += v / L * np.tan(steer) * DT
        for k, val in zip(log, (t, x, y, h)):
            log[k].append(val)
        if t >= next_snap:
            snaps.append({"t": t, "k": len(log["t"]) - 1, "seen": seen.copy(), "posts": list(posts),
                          "plan": plan.copy(), "wall_x": wall_x, "final": final})
            next_snap += snapshot_every
        t += DT
    log = {k: np.array(v) for k, v in log.items()}
    return {"log": log, "posts": posts, "side": side, "wall_x": wall_x, "events": events,
            "replans": replans, "finished": bool(final and t < max_time), "snapshots": snaps,
            "final_plan": plan, "A": A}


def evaluate(res, true_posts, true_wall_x, offline_plan=None):
    """Compare what the explorer found and drove with the truth from the full map."""
    lg = res["log"]
    x, y = lg["x"], lg["y"]
    tp = np.asarray(true_posts, float)
    gaps = [float(np.min(np.hypot(x - px, y - py)) - RULES["post_r"] - CAR["half_width"]) for px, py in tp]
    found = np.asarray(res["posts"], float)
    err = [float(np.min(np.hypot(found[:, 0] - px, found[:, 1] - py))) for px, py in tp] if len(found) else []
    out = {
        "finished": res["finished"],
        "time_s": round(float(lg["t"][-1]), 1),
        "path_length_cm": round(float(np.sum(np.hypot(np.diff(x), np.diff(y)))), 1),
        "posts_found": len(found), "posts_true": len(tp),
        "post_position_error_cm": [round(e, 1) for e in err],
        "wall_found_cm": None if res["wall_x"] is None else round(res["wall_x"], 1),
        "wall_true_cm": None if true_wall_x is None else round(float(true_wall_x), 1),
        "first_side": None if res["side"] is None else ("left" if res["side"] > 0 else "right"),
        "replans": res["replans"],
        "gap_to_each_post_cm": [round(g, 1) for g in gaps],
        "max_sideways_cm": round(float(np.max(np.abs(y))), 1),
        "no_collision": bool(min(gaps) > 0 and np.max(np.abs(y)) + CAR["half_width"] < RULES["half_corridor"]),
        "events": res["events"],
    }
    if offline_plan is not None:
        from scipy.spatial import cKDTree
        dev = cKDTree(np.asarray(offline_plan)).query(np.column_stack([x, y]))[0]
        out["distance_to_offline_plan_cm"] = {"mean": round(float(dev.mean()), 1), "max": round(float(dev.max()), 1)}
        out["offline_plan_length_cm"] = round(float(np.sum(np.linalg.norm(np.diff(offline_plan, axis=0), axis=1))), 1)
    return out


# ---------------------------------------------------------------------------
# pictures
# ---------------------------------------------------------------------------
def _ax_track(ax, wall_x):
    ax.axvline(-RULES["half_corridor"], c="gray", ls="--", lw=1)
    ax.axvline(RULES["half_corridor"], c="gray", ls="--", lw=1)
    ax.set_xlim(70, -70)                       # + y (left) drawn on the left
    ax.set_ylim(-20, (wall_x or 430) + 40)
    ax.set_aspect("equal")
    ax.set_xlabel("y, sideways [cm] (+ = left)")
    ax.set_ylabel("x, forward [cm]")
    ax.grid(alpha=0.3)


def save_plot(world_xyh, res, ev, true_posts, offline_plan, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    lg = res["log"]
    seen = res["snapshots"][-1]["seen"]
    fig, ax = plt.subplots(1, 2, figsize=(12, 9.5))
    a = ax[0]
    a.scatter(world_xyh[~seen, 1], world_xyh[~seen, 0], s=0.3, c="#e0e0e0")
    a.scatter(world_xyh[seen, 1], world_xyh[seen, 0], s=0.4, c="#5e81ac")
    for px, py in res["posts"]:
        a.add_patch(plt.Circle((py, px), RULES["post_r"], color="red"))
    if res["wall_x"]:
        a.axhline(res["wall_x"], c="brown", lw=3)
    a.plot(lg["y"], lg["x"], c="orange", lw=2)
    a.set_title("What the car saw and found by itself")
    _ax_track(a, res["wall_x"])
    from matplotlib.lines import Line2D
    a.legend(handles=[
        Line2D([], [], ls="none", marker="o", ms=4, c="#cccccc", label="never seen"),
        Line2D([], [], ls="none", marker="o", ms=4, c="#5e81ac", label="seen by the car"),
        Line2D([], [], ls="none", marker="o", ms=7, c="red", label=f"posts found: {len(res['posts'])}"),
        Line2D([], [], c="brown", lw=3, label="wall found"),
        Line2D([], [], c="orange", lw=2, label="explorer drive"),
    ], loc="lower left", fontsize=7)

    a = ax[1]
    for px, py in true_posts:
        a.add_patch(plt.Circle((py, px), RULES["post_r"], color="gray"))
    a.plot(offline_plan[:, 1], offline_plan[:, 0], "--", c="black", lw=1.2,
           label="plan with the full map (known track)")
    a.plot(lg["y"], lg["x"], c="orange", lw=2, label="explorer (unknown track)")
    times = [e for e in ev["events"] if "seen" in e["event"] or "final" in e["event"]]
    for e in times:
        k = min(int(e["t"] / DT), len(lg["x"]) - 1)
        a.plot(lg["y"][k], lg["x"][k], "o", ms=5, c="purple")
    a.plot([], [], "o", c="purple", label="something new seen / re-plan")
    a.set_title(f"Unknown vs known track: {ev['path_length_cm']} vs {ev.get('offline_plan_length_cm')} cm, "
                f"min gap {min(ev['gap_to_each_post_cm'])} cm")
    _ax_track(a, res["wall_x"])
    a.legend(loc="lower left", fontsize=7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def save_animation(world_xyh, res, out_gif, sensor=SENSOR, fps=10):
    """GIF: the car explores; seen points light up, posts appear, the plan changes."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon, Wedge
    lg, snaps = res["log"], res["snapshots"]
    fig, ax = plt.subplots(figsize=(5, 9))
    ax.scatter(world_xyh[:, 1], world_xyh[:, 0], s=0.3, c="#ececec")
    seen_sc = ax.scatter([], [], s=0.5, c="#5e81ac")
    posts_sc = ax.scatter([], [], s=60, c="red", zorder=4)
    plan_ln, = ax.plot([], [], "--", c="black", lw=1)
    trail, = ax.plot([], [], c="orange", lw=2)
    wall_ln = ax.axhline(-1000, c="brown", lw=3)
    cone = Wedge((0, 0), sensor["range"], 0, 1, color="gold", alpha=0.15)
    ax.add_patch(cone)
    body = Polygon(np.zeros((4, 2)), closed=True, fc="orange", ec="black", zorder=5)
    ax.add_patch(body)
    title = ax.set_title("")
    _ax_track(ax, res["wall_x"])

    def update(i):
        sn = snaps[i]
        k = sn["k"]
        x, y, h = lg["x"][k], lg["y"][k], lg["heading"][k]
        seen = sn["seen"]
        seen_sc.set_offsets(world_xyh[seen][:, [1, 0]])
        posts_sc.set_offsets(np.array([[p[1], p[0]] for p in sn["posts"]]) if sn["posts"] else np.empty((0, 2)))
        plan_ln.set_data(sn["plan"][:, 1], sn["plan"][:, 0])
        trail.set_data(lg["y"][: k + 1], lg["x"][: k + 1])
        if sn["wall_x"]:
            wall_ln.set_ydata([sn["wall_x"]] * 2)
        # sensor cone: plot axes are (y, x) and the y axis is mirrored -> angle = 90 + heading
        c = 90 + np.degrees(h)
        cone.set_center((y, x))
        cone.set_theta1(c - sensor["fov_deg"] / 2)
        cone.set_theta2(c + sensor["fov_deg"] / 2)
        cs, sn_ = np.cos(h), np.sin(h)
        corners = np.array([[-4, -2.5], [4, -2.5], [4, 2.5], [-4, 2.5]]) @ np.array([[cs, sn_], [-sn_, cs]]) + [x, y]
        body.set_xy(corners[:, ::-1])
        state = "final plan: U-turn and back" if sn["final"] else "exploring"
        title.set_text(f"t = {sn['t']:.0f} s, posts known: {len(sn['posts'])}, {state}")
        return seen_sc, posts_sc, plan_ln, trail, body, cone, title

    FuncAnimation(fig, update, frames=len(snaps), blit=False).save(out_gif, writer=PillowWriter(fps=fps))
    plt.close(fig)
