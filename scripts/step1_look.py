"""Step 1: open the top-view photo, print its size, save it with a pixel grid.

Run from the minicar folder:
    python3 scripts/step1_look.py
"""
import matplotlib
matplotlib.use("Agg")                 # draw into a file, no window needed
import matplotlib.pyplot as plt

PHOTO = "data/top_view.jpeg"
OUT = "results/step1_top_view_grid.png"

# 1. Read the photo. The result is a numpy array: one number per pixel and colour.
img = plt.imread(PHOTO)

# 2. .shape = (height, width, colours). Colours = 3: red, green, blue.
height, width, colours = img.shape
print(f"Photo: {PHOTO}")
print(f"Size: width {width} px, height {height} px, {colours} colour channels")

# 3. Show the photo with pixel coordinates on the axes, so we can read off
#    where the tile corners and the post bases are.
fig, ax = plt.subplots(figsize=(6, 10))
ax.imshow(img)
ax.set_xticks(range(0, width, 100))
ax.set_yticks(range(0, height, 100))
ax.grid(color="red", alpha=0.4)
ax.set_title("top_view.jpeg with pixel grid (step 100 px)")
fig.tight_layout()

import os
os.makedirs("results", exist_ok=True)
fig.savefig(OUT, dpi=100)
print(f"Saved: {OUT}")
