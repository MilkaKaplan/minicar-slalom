#!/bin/bash
# Copy the result figures used in README.md from results/ to docs/images/.
# Run after: docker compose up ... (main), the video version and scripts/step3_detect.py
set -e
cd "$(dirname "$0")/.."
mkdir -p docs/images
cp results/slalom_plan.png        docs/images/slalom_plan.png
cp results/step3_detection.png    docs/images/detection_steps.png
cp results/sim_ideal.png          docs/images/sim_ideal.png
cp results/sim_realistic.png      docs/images/sim_realistic.png
cp results/sim_realistic.gif      docs/images/sim_realistic.gif
cp results/sim_robustness.png     docs/images/sim_robustness.png
cp results/video/video_check.png  docs/images/video_check.png
# a strip of 8 frames from the annotated video GIF (the GIF itself is 5 MB)
python3 - <<'PY'
from PIL import Image
im = Image.open("results/video/video_detections.gif")
picks = [int(i * (im.n_frames - 1) / 7) for i in range(8)]
tiles = []
for i in picks:
    im.seek(i)
    tiles.append(im.convert("RGB").copy())
w, h = tiles[0].size
out = Image.new("RGB", (w * len(tiles), h), "white")
for k, t in enumerate(tiles):
    out.paste(t, (k * w, 0))
out.save("docs/images/video_detections_strip.png")
PY
echo "docs/images updated"
# LiDAR version (branch lidar-v2), after: docker compose --profile lidar up ...
if [ -d results/lidar ]; then
  mkdir -p docs/images/lidar
  cp results/lidar/lidar_map.png results/lidar/sim_realistic.png results/lidar/sim_robustness.png \
     results/lidar/sim_realistic.gif docs/images/lidar/
fi
