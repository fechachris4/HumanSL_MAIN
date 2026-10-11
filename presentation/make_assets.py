"""Prepare the deck's assets/ folder: copy figures and clips from msc_project/media,
crop the hardware-rejection figure to panels (b, c), and generate the QR codes.

Run once before build.js (re-runnable, deterministic):
    python3 make_assets.py [path/to/msc_project]
"""
import shutil
import sys
from pathlib import Path

import qrcode
from PIL import Image

HERE = Path(__file__).resolve().parent
SRC = Path(sys.argv[1] if len(sys.argv) > 1 else "/home/user/fechachris4/msc_project") / "media"
OUT = HERE / "assets"
OUT.mkdir(exist_ok=True)

COPY = [
    "hw_turn.gif", "hw_walk.gif", "hw_replay.gif", "hold_pose.gif",
    "rig.jpg", "task_illustration.png", "control_loop.png", "velocity_delay.png",
]
for name in COPY:
    shutil.copyfile(SRC / name, OUT / name)

# hw_rejection.png panel (a) plots geometric means (72/68/56 %), which would contradict the
# arithmetic means (71/63/56 %) on the results slide. Keep only the frequency-response panels.
fig = Image.open(SRC / "hw_rejection.png")
fig.crop((500, 0, fig.width, fig.height)).save(OUT / "hw_rejection_bc.png", optimize=True)

NAVY = (0x00, 0x21, 0x47)
for slug, url in [
    ("qr_msc_project", "https://github.com/fechachris4/msc_project"),
    ("qr_humansl_main", "https://github.com/fechachris4/HumanSL_MAIN"),
]:
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, box_size=16, border=2)
    qr.add_data(url)
    qr.make(fit=True)
    qr.make_image(fill_color=NAVY, back_color="white").convert("RGB").save(OUT / f"{slug}.png")

# Horizontal fade for the 36 -> 46 ms tail of the Kalman-lag bar (slide "The cause"): Imperial
# orange D24000 fading to 35 % strength over white. pptxgenjs has no gradient fills.
ORANGE, W_ = (0xD2, 0x40, 0x00), (255, 255, 255)
fade = Image.new("RGB", (400, 80))
for x in range(400):
    t = x / 399 * 0.65  # 0 = full orange, 0.65 = 35 % orange over white
    c = tuple(round(o * (1 - t) + w * t) for o, w in zip(ORANGE, W_))
    for y in range(80):
        fade.putpixel((x, y), c)
fade.save(OUT / "kalman_fade.png")

for p in sorted(OUT.iterdir()):
    print(p.name, Image.open(p).size)
