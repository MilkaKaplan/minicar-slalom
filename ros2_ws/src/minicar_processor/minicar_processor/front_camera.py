"""Video version: find posts on the car's front-camera frames (no ROS here).

Idea: on the video the posts are COOL white (a little blue), while the wall and the
floor are WARM white/grey (a little yellow). So a pixel belongs to a post if its
blue-minus-red value is clearly higher than on the floor.

Posts stand on the floor, so we only look at a band of rows just above the floor
line, and keep columns where most pixels in that band are "post" pixels. A wide
group of such columns = a post; the widest one = the nearest post.

Limits (seen in the results): when the camera turns towards the windows the colours
change and the posts are missed; white chair legs can look like posts.
"""
import numpy as np

# Detection knobs
COOL_MIN = 12        # blue-minus-red must be this much above the floor
MAX_SAT = 90         # posts are not strongly coloured (blue jeans are)
MIN_BRIGHT = 90      # posts are bright
BAND = (220, 480)    # rows just above the floor line (frame is 480 x 860)
MIN_FILL = 0.5       # share of "post" pixels in a column of the band
MIN_WIDTH = 20       # px, narrower groups are ignored


def detect(rgb):
    """rgb: (H, W, 3) uint8. Returns (list of posts as (x_left, x_right), mask)."""
    import cv2
    img = rgb.astype(np.float32)
    cool = img[..., 2] - img[..., 0]
    cool -= np.median(cool[650:, :])                 # compare with the floor at the bottom
    sat = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[..., 1]
    mask = (cool > COOL_MIN) & (sat < MAX_SAT) & (img.mean(axis=2) > MIN_BRIGHT)
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))

    fill = mask[BAND[0]:BAND[1]].mean(axis=0)        # per column
    cols = fill > MIN_FILL
    edges = np.diff(np.concatenate(([0], cols.astype(int), [0])))
    starts, ends = np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0]
    posts = [(int(a), int(b)) for a, b in zip(starts, ends) if b - a >= MIN_WIDTH]
    return posts, mask


def nearest(posts):
    """Widest post = nearest one. Returns (x_left, x_right) or None."""
    return max(posts, key=lambda p: p[1] - p[0]) if posts else None


def passing_events(t, x, width_px, edge=120, jump=120):
    """A post is 'passed' when the tracked nearest post disappears near the edge of the
    frame. Returns list of (time, 'left'/'right')."""
    events = []
    for k in range(1, len(t)):
        was = np.isfinite(x[k - 1])
        gone = not np.isfinite(x[k]) or abs(x[k] - x[k - 1]) > jump
        if was and gone and abs(x[k - 1]) > edge:
            events.append((round(float(t[k - 1]), 1), "left" if x[k - 1] < 0 else "right"))
    return events


def phase_of(t, phases):
    for name, a, b in phases:
        if a <= t < b:
            return name
    return phases[-1][0]


def annotate(rgb, posts, t, phase, scale=3):
    """Small copy of the frame with the detected posts drawn on it."""
    import cv2
    h, w = rgb.shape[:2]
    small = cv2.resize(rgb, (w // scale, h // scale))
    near = nearest(posts)
    for a, b in posts:
        colour = (255, 0, 0) if (a, b) == near else (0, 200, 0)
        cv2.rectangle(small, (a // scale, BAND[0] // scale), (b // scale, BAND[1] // scale), colour, 2)
    cv2.line(small, (w // scale // 2, 0), (w // scale // 2, h // scale), (255, 255, 0), 1)
    cv2.putText(small, f"{t:4.1f}s {phase}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    return small


COLOURS = {"there": "#4c72b0", "U-turn": "#dd8452", "back": "#8172b3", "end": "#55a868"}


def save_report(t, x, phases, events, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 4.5), gridspec_kw={"width_ratios": [3, 1]})
    phases = [(n, a, min(b, float(t.max()))) for n, a, b in phases]    # last phase ends with the video
    for name, a, b in phases:
        ax1.axvspan(a, b, color=COLOURS.get(name, "gray"), alpha=0.12)
        ax1.text((a + b) / 2, 245, name, ha="center", color=COLOURS.get(name, "gray"))
    ax1.scatter(t, x, s=6, c="black", label="nearest post found")
    for te, side in events:
        ax1.axvline(te, c="red" if side == "left" else "green", lw=0.8, alpha=0.6)
    ax1.axhline(0, c="gray", lw=0.8)
    ax1.set_ylim(-260, 260)
    ax1.set_xlabel("video time [s]")
    ax1.set_ylabel("post position in frame [px]\n(- left, + right of centre)")
    ax1.set_title("Nearest post seen by the front camera (red/green lines = post left the frame on the left/right)")
    ax1.grid(alpha=0.3)

    names = [p[0] for p in phases]
    found = []
    for name, a, b in phases:
        m = (t >= a) & (t < b)
        found.append(100 * np.isfinite(x[m]).mean() if m.any() else 0)
    ax2.bar(names, found, color=[COLOURS.get(n, "gray") for n in names])
    for i, v in enumerate(found):
        ax2.text(i, v + 2, f"{v:.0f}%", ha="center")
    ax2.set_ylim(0, 110)
    ax2.set_ylabel("frames with a post found [%]")
    ax2.set_title("Detection rate per phase")
    ax2.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)
    return dict(zip(names, [round(float(v), 1) for v in found]))


def save_gif(frames, out_gif, fps=10):
    from PIL import Image as PilImage
    imgs = [PilImage.fromarray(f) for f in frames]
    imgs[0].save(out_gif, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0)
