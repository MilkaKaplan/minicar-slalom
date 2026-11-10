"""Car simulator (no ROS here): a small car-like robot follows the planned path.

Car model ("kinematic bicycle"): the car has a wheelbase L and front wheels that
can turn at most MAX_STEER degrees. That gives it a smallest possible turning
radius  R_min = L / tan(MAX_STEER). It cannot turn tighter than that.

Controller ("pure pursuit"): at every moment the car looks at a point on the
path a little ahead of itself (LOOKAHEAD cm) and steers along the arc that would
bring it to that point. Like a person walking along a line, looking a few steps ahead.

Because of the turning limit and the "look ahead" the car never follows the
path exactly; the simulator measures how far it deviates and whether it still
stays clear of the posts.

Two versions:
  IDEAL      the car knows exactly where it is and its wheels turn instantly;
  REALISTIC  the car's position sensor is noisy (it "sees" itself a bit off)
             and the steering motor turns the wheels at a limited speed.
"""
import numpy as np

# Default car and controller (centimetres, seconds, degrees)
CAR = {
    "half_width": 2.5,      # car is 5 cm wide
    "wheelbase": 5.0,       # distance between rear and front wheels
    "max_steer_deg": 30.0,  # front wheels turn at most 30 deg -> R_min = 5 / tan 30 = 8.7 cm
    "speed": 20.0,          # cm per second
}
CONTROL = {"lookahead": 6.0, "dt": 0.01, "max_time": 120.0}

# Imperfections of a real car. IDEAL = none of them.
IDEAL = {"noise_pos_cm": 0.0, "noise_heading_deg": 0.0, "steer_rate_deg_s": None, "seed": 0}
REALISTIC = {
    "noise_pos_cm": 1.0,         # position sensor error (standard deviation), cm
    "noise_heading_deg": 2.0,    # heading sensor error (standard deviation), degrees
    "steer_rate_deg_s": 120.0,   # steering motor: at most 120 deg per second
    "seed": 0,                   # random seed, so the "random" noise is reproducible
}


def min_turn_radius(car):
    return car["wheelbase"] / np.tan(np.radians(car["max_steer_deg"]))


def densify(path_xy, step=0.2):
    """Re-sample the path every `step` cm, so the controller has points close together."""
    d = np.hypot(*np.diff(path_xy, axis=0).T)
    s = np.concatenate([[0], np.cumsum(d)])
    s_new = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(s_new, s, path_xy[:, 0]), np.interp(s_new, s, path_xy[:, 1])])


def simulate(path_xy, car=CAR, control=CONTROL, realism=IDEAL):
    """Drive along path_xy (N x 2, cm). Returns dict with the driven trajectory and
    per-step data: x, y, heading, steering angle, distance to the path.

    x, y, heading are where the car REALLY is. The controller only gets a noisy
    measurement of them (realism["noise_..."]); the wheels follow the controller's
    command at most realism["steer_rate_deg_s"] degrees per second."""
    path = densify(np.asarray(path_xy, dtype=float))
    L, dt, la = car["wheelbase"], control["dt"], control["lookahead"]
    max_steer = np.radians(car["max_steer_deg"])
    rng = np.random.default_rng(realism["seed"])
    rate = realism["steer_rate_deg_s"]
    max_step = np.inf if rate is None else np.radians(rate) * dt   # max wheel turn per step
    steer = 0.0                                                     # actual wheel angle

    # start at the beginning of the path, looking along it
    x, y = path[0]
    heading = np.arctan2(*(path[5] - path[0])[::-1])
    idx = 0                                   # how far along the path we are (index)
    window = int(30 / 0.2)                    # search the nearest point only 30 cm ahead
    log = {"t": [], "x": [], "y": [], "heading": [], "steer_deg": [], "error": []}

    t = 0.0
    while t < control["max_time"]:
        # 0. what the car's sensors say (real position + random sensor error)
        mx = x + rng.normal(0, realism["noise_pos_cm"]) if realism["noise_pos_cm"] else x
        my = y + rng.normal(0, realism["noise_pos_cm"]) if realism["noise_pos_cm"] else y
        mh = heading + (np.radians(rng.normal(0, realism["noise_heading_deg"]))
                        if realism["noise_heading_deg"] else 0.0)

        # 1. where am I on the path? (nearest point, only searching forward,
        #    so the car does not "jump" to the way-back part where paths cross)
        seg = path[idx: idx + window]
        dist = np.hypot(seg[:, 0] - mx, seg[:, 1] - my)
        idx += int(np.argmin(dist))
        seg = path[max(idx - window, 0): idx + window]
        error = float(np.hypot(seg[:, 0] - x, seg[:, 1] - y).min())   # REAL distance to the path

        # 2. finished?
        if idx >= len(path) - 2 and np.hypot(*(path[-1] - [mx, my])) < 1.0:
            break

        # 3. target point: first path point at least `lookahead` cm away
        ahead = path[idx:]
        far = np.nonzero(np.hypot(ahead[:, 0] - mx, ahead[:, 1] - my) >= la)[0]
        tx, ty = ahead[far[0]] if len(far) else path[-1]

        # 4. pure pursuit: steer along the arc through the target point
        alpha = np.arctan2(ty - my, tx - mx) - mh
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))          # wrap to -180..180 deg
        command = np.arctan2(2 * L * np.sin(alpha), la)
        command = float(np.clip(command, -max_steer, max_steer))   # the wheels have a limit!
        # the steering motor needs time: the wheels move towards the command step by step
        steer += float(np.clip(command - steer, -max_step, max_step))

        # 5. move the car for dt seconds
        v = car["speed"]
        x += v * np.cos(heading) * dt
        y += v * np.sin(heading) * dt
        heading += v / L * np.tan(steer) * dt

        for k, val in zip(log, [t, x, y, heading, np.degrees(steer), error]):
            log[k].append(float(val))
        t += dt
    log = {k: np.array(v) for k, v in log.items()}
    log["finished"] = bool(t < control["max_time"])
    return log


def evaluate(log, posts_cm, post_r, half_corridor, wall_x, car=CAR):
    """Safety and accuracy numbers for the DRIVEN trajectory."""
    x, y = log["x"], log["y"]
    gaps = [float(np.min(np.hypot(x - px, y - py)) - post_r - car["half_width"]) for px, py in posts_cm]
    gap_side = float(half_corridor - np.max(np.abs(y)) - car["half_width"])
    gap_wall = float(wall_x - np.max(x) - car["half_width"])
    return {
        "finished": log["finished"],
        "time_s": round(float(log["t"][-1]), 2),
        "driven_length_cm": round(float(np.sum(np.hypot(np.diff(x), np.diff(y)))), 1),
        "tracking_error_mean_cm": round(float(log["error"].mean()), 2),
        "tracking_error_max_cm": round(float(log["error"].max()), 2),
        "gap_to_each_post_cm": [round(g, 1) for g in gaps],
        "gap_to_corridor_edge_cm": round(gap_side, 1),
        "gap_to_wall_cm": round(gap_wall, 1),
        "max_steer_used_deg": round(float(np.max(np.abs(log["steer_deg"]))), 1),
        "car_min_turn_radius_cm": round(float(min_turn_radius(car)), 1),
        "no_collision": bool(min(gaps) > 0 and gap_side > 0 and gap_wall > 0),
    }


# =====================================================================
# Pictures
# =====================================================================
def _track(ax, posts_cm, post_r, half_corridor, wall_x):
    import matplotlib.pyplot as plt
    for px, py in posts_cm:
        ax.add_patch(plt.Circle((py, px), post_r, color="gray"))
    ax.axvline(-half_corridor, c="gray")
    ax.axvline(half_corridor, c="gray")
    ax.axhline(wall_x, c="brown", lw=3)
    ax.set_xlim(-60, 60)
    ax.set_ylim(-10, wall_x + 10)
    ax.invert_xaxis()                       # + y (car's left) on the left, like the other maps
    ax.set_aspect("equal")
    ax.set_xlabel("y, sideways [cm] (+ = left)")
    ax.set_ylabel("x, forward [cm]")
    ax.grid(alpha=0.3)


def save_plot(log, path_xy, posts_cm, post_r, half_corridor, wall_x, ev, out_png, label="Simulated drive"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 9))
    ax1 = fig.add_subplot(1, 2, 1)
    _track(ax1, posts_cm, post_r, half_corridor, wall_x)
    ax1.plot(path_xy[:, 1], path_xy[:, 0], "--", c="black", lw=1, label="planned path")
    sc = ax1.scatter(log["y"][::5], log["x"][::5], c=log["error"][::5], s=4, cmap="plasma",
                     vmin=0, vmax=max(1.0, ev["tracking_error_max_cm"]), label="driven by the car")
    fig.colorbar(sc, ax=ax1, shrink=0.6, label="distance from planned path [cm]")
    ok = "no collision" if ev["no_collision"] else "COLLISION!"
    ax1.set_title(f"{label}: car R_min = {ev['car_min_turn_radius_cm']} cm, {ok}")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=2, fontsize=8)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(log["t"], log["error"], c="purple")
    ax2.set_ylabel("distance from path [cm]")
    ax2.set_title(f"Tracking error: mean {ev['tracking_error_mean_cm']} cm, max {ev['tracking_error_max_cm']} cm")
    ax2.grid(alpha=0.3)
    ax3 = fig.add_subplot(2, 2, 4, sharex=ax2)
    ax3.plot(log["t"], log["steer_deg"], c="teal")
    ax3.axhline(0, c="gray", lw=0.5)
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("steering angle [deg]")
    ax3.set_title("Steering (front wheels)")
    ax3.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def save_animation(log, path_xy, posts_cm, post_r, half_corridor, wall_x, out_gif,
                   car=CAR, frames=120, fps=15):
    """GIF: the car (a small rectangle) drives along the path."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    fig, ax = plt.subplots(figsize=(4.5, 9))
    _track(ax, posts_cm, post_r, half_corridor, wall_x)
    ax.plot(path_xy[:, 1], path_xy[:, 0], "--", c="black", lw=0.8)
    trail, = ax.plot([], [], c="orange", lw=2)
    body = Polygon(np.zeros((4, 2)), closed=True, fc="orange", ec="black")
    ax.add_patch(body)
    title = ax.set_title("")
    length = car["wheelbase"] + 3.0                 # body a bit longer than the wheelbase
    w = car["half_width"]
    steps = np.linspace(0, len(log["t"]) - 1, frames).astype(int)

    def update(i):
        k = steps[i]
        x, y, h = log["x"][k], log["y"][k], log["heading"][k]
        # rectangle corners in the car's own frame (x forward, y left), from the rear axle
        corners = np.array([[-1.5, -w], [length - 1.5, -w], [length - 1.5, w], [-1.5, w]])
        rot = np.array([[np.cos(h), -np.sin(h)], [np.sin(h), np.cos(h)]])
        xy = corners @ rot.T + [x, y]
        body.set_xy(xy[:, ::-1])                    # plot is (y, x)
        trail.set_data(log["y"][: k + 1], log["x"][: k + 1])
        title.set_text(f"t = {log['t'][k]:.1f} s")
        return body, trail, title

    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    anim.save(out_gif, writer=PillowWriter(fps=fps))
    plt.close(fig)


# =====================================================================
# Robustness test: many realistic drives with more and more sensor noise
# =====================================================================
def robustness(path_xy, posts_cm, post_r, half_corridor, wall_x, car=CAR, control=CONTROL,
               noise_levels_cm=(0.5, 1.0, 2.0, 3.0), runs=10, steer_rate_deg_s=120.0):
    """For every noise level drive `runs` times with different random noise.
    Returns one row per level: worst tracking error, worst gap to a post, collisions."""
    rows = []
    for n in noise_levels_cm:
        errs, gaps, fails = [], [], 0
        for seed in range(runs):
            real = {"noise_pos_cm": n, "noise_heading_deg": 2 * n,
                    "steer_rate_deg_s": steer_rate_deg_s, "seed": seed}
            ev = evaluate(simulate(path_xy, car, control, real), posts_cm, post_r, half_corridor, wall_x, car)
            errs.append(ev["tracking_error_max_cm"])
            gaps.append(min(ev["gap_to_each_post_cm"]))
            fails += int(not (ev["no_collision"] and ev["finished"]))
        rows.append({"noise_pos_cm": n, "noise_heading_deg": 2 * n, "runs": runs,
                     "tracking_error_max_avg_cm": round(float(np.mean(errs)), 2),
                     "tracking_error_max_worst_cm": round(float(np.max(errs)), 2),
                     "gap_to_post_worst_cm": round(float(np.min(gaps)), 1),
                     "collisions": fails})
    return rows


def save_robustness_plot(rows, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [f"{r['noise_pos_cm']} cm\n{r['noise_heading_deg']} deg" for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    ax1.bar(labels, [r["tracking_error_max_worst_cm"] for r in rows], color="purple", alpha=0.7)
    ax1.set_ylabel("worst distance from path [cm]")
    ax1.set_xlabel("position / heading sensor noise")
    ax1.set_title("How far the car drifts from the path")
    ax1.grid(alpha=0.3, axis="y")
    gaps = [r["gap_to_post_worst_cm"] for r in rows]
    ax2.bar(labels, gaps, color=["green" if g > 0 else "red" for g in gaps], alpha=0.7)
    ax2.axhline(0, c="black", lw=1)
    ax2.set_ylim(min(min(gaps), 0) - 1, max(gaps) + 1.5)      # room for the "x/10 hit" labels
    for i, r in enumerate(rows):
        ax2.text(i, max(gaps[i], 0) + 0.3, f"{r['collisions']}/{r['runs']} hit", ha="center", fontsize=9)
    ax2.set_ylabel("worst gap car side -> post [cm]")
    ax2.set_xlabel("position / heading sensor noise")
    ax2.set_title("Closest pass to a post (below 0 = collision)")
    ax2.grid(alpha=0.3, axis="y")
    fig.suptitle(f"Robustness: {rows[0]['runs']} drives per noise level, steering motor 120 deg/s")
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)
