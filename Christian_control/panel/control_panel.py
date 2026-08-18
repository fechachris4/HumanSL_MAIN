#!/usr/bin/env python3
"""Control panel for the SRL controller — start it, then open a browser.

    python3 Christian_control/panel/control_panel.py
    python3 Christian_control/panel/control_panel.py --lan
    python3 Christian_control/panel/control_panel.py --replay runs/2026-08-11/loop_log_right_20260811_154613.csv

One screen for configuring, planning, running and watching the controller, so
that running the system does not depend on remembering which file holds which
knob, which binary is stale, and which flag the session script wants.

The rule the whole panel serves:

    The browser observes and commands. The controller decides whether motion
    is safe.

Two consequences worth knowing before you press anything. The panel's stop is
a TRIGGER, not a mechanism: it becomes the same SIGINT your keyboard sends,
arriving at the controller's existing stop path, so the browser never gains a
way to stop the arm that the terminal does not already have. And there is no
velocity ramp behind it — the stop ends commanding and hands the arm back to
high-level servoing, which is why the control is labelled "Stop and release
the arm" rather than anything that sounds gentler.

The physical emergency stop is outside all of this. No software here observes
it, which is why the panel says so permanently rather than showing a status it
cannot know.

Starting a session is refused unless the browser is on this machine. Stopping
works from anywhere. Remote stop is always safe; remote start never is.

The work lives in panel/ — one file per job. This file only parses arguments.
"""

import argparse
import sys
from pathlib import Path

# Run as a script from anywhere: this file lives INSIDE the panel package,
# so the directory that must go on the path is the one above it.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from panel import paths, server  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Local web front-end for the SRL controller.")
    parser.add_argument("--port", type=int, default=8765,
                        help="port to listen on (default: 8765)")
    parser.add_argument(
        "--lan", action="store_true",
        help="also listen on this machine's network addresses, so the panel "
             "can be viewed from another machine. Starting a session is still "
             "refused from anywhere but this machine.")
    parser.add_argument(
        "--replay", metavar="CSV",
        help="replay a recorded run instead of following a live one. No robot "
             "is involved; this is how the run screen is developed and shown.")
    parser.add_argument("--replay-speed", type=float, default=1.0,
                        help="replay speed multiplier (default: 1.0)")
    parser.add_argument(
        "--check", action="store_true",
        help="print one diagnostic report and exit, without starting the "
             "panel: build freshness, the session, which run log a solve "
             "would plan from, and the tail of the controller log. Paste it "
             "when asking why something did not work.")
    args = parser.parse_args()

    if args.check:
        from panel import diagnose  # noqa: PLC0415 — only needed for --check
        return diagnose.main()

    if not paths.CONFIG_H.is_file():
        print(f"cannot find {paths.CONFIG_H}", file=sys.stderr)
        return 1
    if not paths.STATIC.is_dir():
        print(f"cannot find the panel's static files at {paths.STATIC}",
              file=sys.stderr)
        return 1

    replay = None
    if args.replay:
        candidate = Path(args.replay).expanduser()
        if not candidate.is_file():
            print(f"cannot find the run to replay: {candidate}", file=sys.stderr)
            return 1
        replay = str(candidate)

    if args.replay_speed <= 0:
        print("--replay-speed must be greater than zero", file=sys.stderr)
        return 1

    server.serve(port=args.port, lan=args.lan, replay=replay,
                 speed=args.replay_speed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
