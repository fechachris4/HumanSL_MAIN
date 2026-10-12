"""Prepare the deck's assets/ folder: copy figures and clips from msc_project/media,
crop the hardware-rejection figure to panels (b, c), start the simulation clip on its last
frame, and generate the QR codes.

Run once before build.js (re-runnable, deterministic):
    python3 make_assets.py [path/to/msc_project]
"""
import shutil
import sys
from pathlib import Path

import qrcode
from PIL import Image, ImageChops, ImageSequence

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

# hold_pose.gif: move the last frame to the front. The loop plays the same sequence, but static
# views (PowerPoint edit view, thumbnails, the PDF) now show the filled RMS counters and full error
# strip instead of "0.0 mm" beside the slide's 31.0 / 6.7 / 2.6 mm. The clip has <= 87 colours per
# frame, so the re-encode is lossless; verified frame by frame below.
def last_frame_first(path):
    im = Image.open(path)
    frames = [f.convert("RGB") for f in ImageSequence.Iterator(im)]
    durations = [f.info.get("duration", 40) for f in ImageSequence.Iterator(Image.open(path))]
    frames, durations = frames[-1:] + frames[:-1], durations[-1:] + durations[:-1]
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=durations,
                   loop=im.info.get("loop", 0), optimize=True)
    check = [f.convert("RGB") for f in ImageSequence.Iterator(Image.open(path))]
    assert len(check) == len(frames), path
    assert all(ImageChops.difference(a, b).getbbox() is None for a, b in zip(check, frames)), path


last_frame_first(OUT / "hold_pose.gif")

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

for p in sorted(OUT.iterdir()):
    print(p.name, Image.open(p).size)
