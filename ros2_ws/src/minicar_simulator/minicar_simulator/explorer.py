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
          "max_size": 30.0, "merge": 20.0, "merge_along": 30.0, "wall_width": 50.0}
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
    # pieces of the same post (seen from different sides, a few cm apart because the map
    # "doubles" posts) -> one post. Slalom posts stand far apart along the line, so two
    # pieces closer than `merge_along` cm along the corridor are the same post.
    posts = []
    for p in sorted(found, key=lambda v: v[0]):
        if posts and (abs(posts[-1][0] - p[0]) < DETECT["merge_along"]
                      or np.linalg.norm(posts[-1] - p) < DETECT["merge"]):
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
def explore(world_xyh, sensor=SENSOR, car=CAR, max_time=240.0, snapshot_every=0.5,
            realism=None, env=None, sub_steps=5):
    """Drive the unknown track. world_xyh: N x 3 map points (track frame, cm) above the ground.

    realism (as in sim.py): position/heading sensor noise and a slow steering motor; the car
    steers by its NOISY idea of where it is. env (as in sim.py): wind (head wind there, tail
    wind back, side push) and variable speed (slower before tight turns).
    Control runs every DT = 0.05 s, the car's motion in `sub_steps` smaller steps.
    Returns a log with the drive, events and snapshots for the animation."""
    from . import sim
    realism = realism or sim.IDEAL
    env = env or sim.CALM
    rng = np.random.default_rng(realism.get("seed", 0))
    rate = realism.get("steer_rate_deg_s")
    dt = DT / sub_steps
    max_step = np.inf if rate is None else np.radians(rate) * dt
    variable = env.get("variable_speed", False)
    wind = env.get("wind_cm_s", 0.0) * np.array([np.cos(np.radians(env.get("wind_dir_deg", 0.0))),
                                                 np.sin(np.radians(env.get("wind_dir_deg", 0.0)))])
    v = 0.0 if variable else car["speed"]
    steer = 0.0
    A = RULES["post_r"] + car["gap"] + car["half_width"]
    L, max_steer = car["wheelbase"], np.radians(car["max_steer_deg"])
    seen = np.zeros(len(world_xyh), bool)
    x, y, h = 0.0, 0.0, 0.0
    posts, side, line_y, wall_x = [], None, 0.0, None
    plan = plan_there([], 1, A, 0.0)
    final, there_len, idx = False, len(plan), 0
    events, replans = [], 0
    log = {"t": [], "x": [], "y": [], "heading": [], "speed": [], "speed_target": [], "steer_deg": []}
    curv = sim.path_curvature(plan)
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
                    curv = sim.path_curvature(plan)
                if posts and wall_x is None:
                    wall_x = find_wall(world_xyh[seen], posts[0][0])           # 5. wall? (anywhere after post 1)
                    if wall_x is not None:
                        events.append({"t": round(t, 2), "event": "wall seen",
                                       "wall_x_cm": round(wall_x, 1), "distance_cm": round(float(wall_x - x), 1)})
                        posts = [p for p in posts if p[0] < wall_x - 15]
        # turn around only when the last post is close to the wall: if there is more room than
        # 1.5 post spacings, an unseen post may still be there -> keep driving and looking
        spacing = np.mean(np.diff([p[0] for p in posts])) if len(posts) > 1 else 60.0
        last_is_last = wall_x is not None and posts and (wall_x - posts[-1][0]) < 1.5 * spacing
        if not final and last_is_last and x > posts[-1][0] - 20:
            plan, there_len = plan_full(posts, side, A)
            seg = plan[:there_len]
            idx = int(np.argmin(np.hypot(seg[:, 0] - x, seg[:, 1] - y)))
            final, replans = True, replans + 1
            curv = sim.path_curvature(plan)
            events.append({"t": round(t, 2), "event": "final plan: U-turn around the last post and back"})

        # 6. drive: perception runs every DT (20 Hz), steering and motion every dt (100 Hz).
        #    The car only knows its position through a noisy sensor.
        done_drive = False
        for _ in range(sub_steps):
            mx = x + (rng.normal(0, realism["noise_pos_cm"]) if realism["noise_pos_cm"] else 0.0)
            my = y + (rng.normal(0, realism["noise_pos_cm"]) if realism["noise_pos_cm"] else 0.0)
            mh = h + (np.radians(rng.normal(0, realism["noise_heading_deg"])) if realism["noise_heading_deg"] else 0.0)
            win = plan[idx: idx + 60]
            idx += int(np.argmin(np.hypot(win[:, 0] - mx, win[:, 1] - my)))
            if final and idx >= len(plan) - 3 and np.hypot(*(plan[-1] - [mx, my])) < 3:
                done_drive = True
                break
            ahead = plan[idx:]
            far = np.nonzero(np.hypot(ahead[:, 0] - mx, ahead[:, 1] - my) >= car["lookahead"])[0]
            tx, ty = ahead[far[0]] if len(far) else plan[-1]
            alpha = np.arctan2(ty - my, tx - mx) - mh
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
            command = float(np.clip(np.arctan2(2 * L * np.sin(alpha), car["lookahead"]), -max_steer, max_steer))
            # wanted speed: slower before tight turns in the next 15 cm of the plan
            if variable:
                seg = np.hypot(*np.diff(plan[idx: idx + 400], axis=0).T)
                n_ahead = int(np.searchsorted(np.cumsum(seg), env["look_ahead_speed_cm"])) + 1
                k_ahead = float(curv[idx: idx + n_ahead].max()) if idx < len(curv) else 0.0
                v_target = env["max_speed"] if k_ahead < 1e-6 else np.sqrt(env["max_side_accel"] / k_ahead)
                v_target = float(np.clip(v_target, env["min_speed"], env["max_speed"]))
            else:
                v_target = car["speed"]
            steer += float(np.clip(command - steer, -max_step, max_step))     # slow steering motor
            hv = np.array([np.cos(h), np.sin(h)])
            if variable:
                motor = np.clip(3.0 * (v_target - v), -env["max_brake"], env["max_accel"])
                air = v - wind @ hv                      # head wind slows, tail wind pushes
                v = max(v + (motor - env["drag"] * air * abs(air)) * dt, 0.0)
            left_dir = np.array([-hv[1], hv[0]])                # the car's left
            drift = env.get("side_push", 0.0) * (wind @ left_dir)   # side wind pushes the car sideways
            x += (v * hv[0] + drift * left_dir[0]) * dt
            y += (v * hv[1] + drift * left_dir[1]) * dt
            h += v / L * np.tan(steer) * dt
        if done_drive:
            break
        for k, val in zip(log, (t, x, y, h, v, v_target, np.degrees(steer))):
            log[k].append(float(val))
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
    k = int(np.argmax(x))                     # farthest point = U-turn
    sp = lg["speed"]
    out = {
        "speed_mean_there_cm_s": round(float(sp[:k].mean()), 1) if k > 0 else None,
        "speed_mean_back_cm_s": round(float(sp[k:].mean()), 1),
        "speed_min_max_cm_s": [round(float(sp[len(sp) // 20:].min()), 1), round(float(sp.max()), 1)],
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


# ---------------------------------------------------------------------------
# several drives: ideal vs realistic vs windy, and a stress test
# ---------------------------------------------------------------------------
def drive_conditions(world_xyh, true_posts, true_wall_x, offline_plan, sensor=SENSOR, car=CAR,
                     wind_cm_s=300.0):
    """Three drives of the explorer on the unknown track: ideal, realistic, realistic + wind."""
    from . import sim
    windy_env = dict(sim.WINDY, wind_cm_s=wind_cm_s)
    out = {}
    for name, real, env in [("ideal", sim.IDEAL, sim.CALM),
                            ("realistic", sim.REALISTIC, sim.CALM),
                            ("windy", sim.REALISTIC, windy_env)]:
        res = explore(world_xyh, sensor=sensor, car=car, realism=real, env=env)
        ev = evaluate(res, true_posts, true_wall_x, offline_plan)
        ev["conditions"] = {"realism": {k: v for k, v in real.items() if k != "seed"},
                            "wind_cm_s": env.get("wind_cm_s", 0.0), "variable_speed": env.get("variable_speed", False)}
        out[name] = (res, ev)
    return out


def stress_test(world_xyh, true_posts, true_wall_x, offline_plan, sensor=SENSOR, car=CAR,
                noise_levels_cm=(0.5, 1.0, 2.0, 3.0), runs=5, wind_cm_s=300.0):
    """Windy drives with more and more sensor noise, several random seeds each."""
    from . import sim
    env = dict(sim.WINDY, wind_cm_s=wind_cm_s)
    rows = []
    for n in noise_levels_cm:
        gaps, fails, found = [], 0, []
        for seed in range(runs):
            real = {"noise_pos_cm": n, "noise_heading_deg": 2 * n, "steer_rate_deg_s": 120.0, "seed": seed}
            ev = evaluate(explore(world_xyh, sensor=sensor, car=car, realism=real, env=env),
                          true_posts, true_wall_x, offline_plan)
            gaps.append(min(ev["gap_to_each_post_cm"]))
            found.append(ev["posts_found"])
            fails += int(not (ev["no_collision"] and ev["finished"]))
        rows.append({"noise_pos_cm": n, "noise_heading_deg": 2 * n, "runs": runs,
                     "posts_found": sorted(set(found)), "gap_worst_cm": round(float(min(gaps)), 1),
                     "gap_mean_cm": round(float(np.mean(gaps)), 1), "collisions": fails})
    return rows


def save_conditions_plot(world_xyh, drives, true_posts, offline_plan, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = {"ideal": "IDEAL\nexact sensors, constant speed, no wind",
              "realistic": "REALISTIC\nsensor noise 1 cm / 2 deg, slow steering",
              "windy": "REALISTIC + WIND 3 m/s\n+ variable speed"}
    fig = plt.figure(figsize=(16, 9))
    for i, name in enumerate(("ideal", "realistic", "windy")):
        res, ev = drives[name]
        a = fig.add_subplot(1, 4, i + 1)
        a.scatter(world_xyh[:, 1], world_xyh[:, 0], s=0.2, c="#e6e6e6")
        for px, py in true_posts:
            a.add_patch(plt.Circle((py, px), RULES["post_r"], color="gray"))
        a.plot(offline_plan[:, 1], offline_plan[:, 0], "--", c="black", lw=0.8)
        a.plot(res["log"]["y"], res["log"]["x"], c="orange", lw=1.8)
        ok = "no collision" if ev["no_collision"] else "COLLISION"
        a.set_title(f"{labels[name]}\nmin gap {min(ev['gap_to_each_post_cm'])} cm, {ok}", fontsize=9)
        _ax_track(a, res["wall_x"])
    a = fig.add_subplot(2, 4, 4)
    for name, c in (("realistic", "teal"), ("windy", "darkorange")):
        lg = drives[name][0]["log"]
        a.plot(lg["t"], lg["speed"], c=c, label=name)
    lg = drives["windy"][0]["log"]
    turn = lg["t"][int(np.argmax(lg["x"]))]
    a.axvline(turn, c="red", lw=0.8)
    a.text(turn, a.get_ylim()[1], " U-turn (windy)", color="red", fontsize=8, va="top")
    a.set_xlabel("time [s]")
    a.set_ylabel("speed [cm/s]")
    a.set_title("Speed: head wind there, tail wind back,\nslower before posts", fontsize=9)
    a.legend(fontsize=8)
    a.grid(alpha=0.3)
    a = fig.add_subplot(2, 4, 8)
    for name, c in (("realistic", "teal"), ("windy", "darkorange")):
        lg = drives[name][0]["log"]
        a.plot(lg["t"], lg["steer_deg"], c=c, lw=0.8, label=name)
    a.set_xlabel("time [s]")
    a.set_ylabel("steering [deg]")
    a.set_title("Steering (slow motor, noisy sensors)", fontsize=9)
    a.grid(alpha=0.3)
    fig.suptitle("The explorer on the unknown track under three conditions "
                 "(orange: its drive, dashed: plan with the full map)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=85)
    plt.close(fig)


def save_stress_plot(rows, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = [f"{r['noise_pos_cm']} cm\n{r['noise_heading_deg']} deg" for r in rows]
    gaps = [r["gap_worst_cm"] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.bar(labels, gaps, color=["green" if g > 0 else "red" for g in gaps], alpha=0.7)
    ax.axhline(0, c="black", lw=1)
    for i, r in enumerate(rows):
        ax.text(i, max(gaps[i], 0) + 0.3, f"{r['collisions']}/{r['runs']} hit", ha="center", fontsize=9)
    ax.set_ylim(min(min(gaps), 0) - 1.5, max(gaps) + 2)
    ax.set_xlabel("position / heading sensor noise")
    ax.set_ylabel("worst gap car side -> post [cm]")
    ax.set_title(f"Explorer stress test: unknown track, wind 3 m/s, {rows[0]['runs']} drives per level")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)
