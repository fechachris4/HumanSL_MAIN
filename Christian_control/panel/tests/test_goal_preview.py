"""The goal preview must draw exactly what was asked, and touch nothing.

The preview path is three pure functions in scene.js (parseGoalPreview,
planeBasis, goalPreviewGeometry) plus a scene setter. These tests pin them
from node the way test_scene_fk pins the forward kinematics, with the
mounting transform recomputed here in Python as an independent oracle for
the frame conversion.

The last test class is the safety property stated as text: the preview code
has no way to make a request. scene.js contains no network primitive at all,
and panel.js's preview handler contains none either — so a preview cannot
save a goal, start a solve, start a session, or reach hardware, because
there is no code path from it to any request.
"""

from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from Christian_control.panel import paths

SCENE_JS = paths.STATIC / "scene.js"
PANEL_JS = paths.STATIC / "panel.js"

# The mount placement of the right arm's base_link, from
# model/dual_arm_mounting.yaml (the same numbers scene.js carries and
# tests/test_dual_arm_mounting.cpp pins against the URDF).
RIGHT_MOUNT_XYZ = (0.0, -0.0567075, 0.0)
RIGHT_MOUNT_ROLL_RAD = 1.2085

# Runs the exported preview functions on JSON fields from argv and prints
# the parse result and, when it parses, the mount-frame geometry. The
# frame matrix is built with the scene's OWN exported-adjacent path by
# re-deriving it here from the same constants scene.js documents; the
# Python side then checks the numbers independently.
NODE_HARNESS = """
import { parseGoalPreview, planeBasis, goalPreviewGeometry } from '%(scene)s';

const fields = JSON.parse(process.argv[2]);
const frameMatrix = process.argv[3] ? JSON.parse(process.argv[3]) : null;
const parsed = parseGoalPreview(fields);
if (!parsed.ok) {
    console.log(JSON.stringify(parsed));
} else {
    const geometry = goalPreviewGeometry(parsed.preview, frameMatrix);
    console.log(JSON.stringify({
        ok: true,
        preview: parsed.preview,
        target: geometry.target,
        centre: geometry.centre,
        points: geometry.points,
        triad: geometry.triad,
        basis: parsed.preview.kind === 'circle'
            ? planeBasis(parsed.preview.normal) : null,
    }));
}
"""


def right_base_to_mount(p: tuple[float, float, float]) -> list[float]:
    """Trans(xyz) then Rx(roll), computed independently of scene.js."""
    c = math.cos(RIGHT_MOUNT_ROLL_RAD)
    s = math.sin(RIGHT_MOUNT_ROLL_RAD)
    return [
        RIGHT_MOUNT_XYZ[0] + p[0],
        RIGHT_MOUNT_XYZ[1] + c * p[1] - s * p[2],
        RIGHT_MOUNT_XYZ[2] + s * p[1] + c * p[2],
    ]


def right_mount_matrix() -> list[float]:
    """The same transform as a row-major 4x4 for the harness."""
    c = math.cos(RIGHT_MOUNT_ROLL_RAD)
    s = math.sin(RIGHT_MOUNT_ROLL_RAD)
    return [1, 0, 0, RIGHT_MOUNT_XYZ[0],
            0, c, -s, RIGHT_MOUNT_XYZ[1],
            0, s, c, RIGHT_MOUNT_XYZ[2],
            0, 0, 0, 1]


class PreviewHarness(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if shutil.which("node") is None:
            raise unittest.SkipTest("node is not installed; cannot run scene.js")
        if not SCENE_JS.is_file():
            raise unittest.SkipTest("scene.js is missing")

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def run_preview(self, fields: dict, frame_matrix: list | None = None) -> dict:
        harness = self.tmp / "preview_harness.mjs"
        harness.write_text(NODE_HARNESS % {"scene": SCENE_JS.as_posix()},
                           encoding="utf-8")
        argv = ["node", str(harness), json.dumps(fields)]
        if frame_matrix is not None:
            argv.append(json.dumps(frame_matrix))
        proc = subprocess.run(argv, capture_output=True, text=True,
                              timeout=60, check=True)
        return json.loads(proc.stdout.strip().splitlines()[-1])


class PointPreview(PreviewHarness):
    def test_mount_frame_point_is_drawn_where_typed(self) -> None:
        result = self.run_preview({"mode": "point", "frame": "mount",
                                   "goal": ["0.5", "0.0", "0.4"],
                                   "rpy_deg": None})
        self.assertTrue(result["ok"])
        self.assertEqual(result["target"], [0.5, 0.0, 0.4])
        self.assertIsNone(result["triad"], "no orientation typed, no triad")

    def test_orientation_gives_a_triad_at_the_point(self) -> None:
        result = self.run_preview({"mode": "point", "frame": "mount",
                                   "goal": [0.5, 0, 0.4],
                                   "rpy_deg": [90, 0, 0]})
        triad = result["triad"]
        self.assertIsNotNone(triad)
        # The triad sits at the goal...
        self.assertAlmostEqual(triad[3], 0.5, places=12)
        self.assertAlmostEqual(triad[7], 0.0, places=12)
        self.assertAlmostEqual(triad[11], 0.4, places=12)
        # ...and Rz*Ry*Rx with roll 90 deg sends +y to +z: column 1 of the
        # rotation is (0, 0, 1).
        self.assertAlmostEqual(triad[1], 0.0, places=12)
        self.assertAlmostEqual(triad[5], 0.0, places=12)
        self.assertAlmostEqual(triad[9], 1.0, places=12)


class CirclePreview(PreviewHarness):
    FIELDS = {"mode": "circle", "frame": "mount",
              "centre": [0.39, 0.386, 0.5213], "radius_m": "0.2",
              "normal": [1, 0, 0], "rpy_deg": [90, 0, 90]}

    def test_the_whole_rim_is_a_circle_of_the_typed_radius(self) -> None:
        result = self.run_preview(self.FIELDS)
        self.assertTrue(result["ok"])
        points = result["points"]
        self.assertGreaterEqual(len(points), 33, "too few points to look round")
        centre = result["centre"]
        for p in points:
            radius = math.dist(p, centre)
            self.assertAlmostEqual(radius, 0.2, places=9)
            # In the plane: the offset from the centre is perpendicular to
            # the normal (x here).
            self.assertAlmostEqual(p[0] - centre[0], 0.0, places=9)
        self.assertLess(math.dist(points[0], points[-1]), 1e-12,
                        "the drawn rim must close")

    def test_start_point_matches_the_bridge_convention(self) -> None:
        """GenerateCircle starts at centre + r*u with PlaneBasis's u.

        For normal x, PlaneBasis seeds with y and u = x cross y = z, so the
        start is the centre displaced +r along z. If this fails, the start
        marker no longer marks where the planned trace will begin.
        """
        result = self.run_preview(self.FIELDS)
        u = result["basis"]["u"]
        self.assertEqual(u, [0, 0, 1])
        self.assertEqual(result["target"],
                         [0.39, 0.386, 0.5213 + 0.2])
        # Fixed orientation: the triad sits at the start point.
        self.assertIsNotNone(result["triad"])
        self.assertAlmostEqual(result["triad"][11], 0.5213 + 0.2, places=12)

    def test_radial_orientation_draws_no_triad(self) -> None:
        fields = dict(self.FIELDS, rpy_deg=None)
        result = self.run_preview(fields)
        self.assertTrue(result["ok"])
        self.assertIsNone(result["triad"])


class FrameConversion(PreviewHarness):
    def test_right_base_point_lands_where_the_mounting_says(self) -> None:
        point = (0.5, 0.1, 0.4)
        result = self.run_preview(
            {"mode": "point", "frame": "right_base", "goal": list(point),
             "rpy_deg": None},
            right_mount_matrix())
        expected = right_base_to_mount(point)
        for got, want in zip(result["target"], expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_mount_frame_needs_no_matrix_and_moves_nothing(self) -> None:
        result = self.run_preview({"mode": "point", "frame": "mount",
                                   "goal": [0.1, 0.2, 0.3], "rpy_deg": None})
        self.assertEqual(result["target"], [0.1, 0.2, 0.3])


class Validation(PreviewHarness):
    def test_non_numeric_goal_is_refused_with_words(self) -> None:
        result = self.run_preview({"mode": "point", "frame": "mount",
                                   "goal": ["0.5", "", "0.4"], "rpy_deg": None})
        self.assertFalse(result["ok"])
        self.assertIn("three finite numbers", result["error"])

    def test_zero_normal_is_refused(self) -> None:
        result = self.run_preview({"mode": "circle", "frame": "mount",
                                   "centre": [0, 0, 0], "radius_m": "0.2",
                                   "normal": [0, 0, 0], "rpy_deg": None})
        self.assertFalse(result["ok"])
        self.assertIn("normal", result["error"])

    def test_non_positive_radius_is_refused(self) -> None:
        for radius in ("-0.1", "0", "nan", ""):
            result = self.run_preview({"mode": "circle", "frame": "mount",
                                       "centre": [0, 0, 0], "radius_m": radius,
                                       "normal": [0, 0, 1], "rpy_deg": None})
            self.assertFalse(result["ok"], f"radius {radius!r} was accepted")

    def test_unknown_frame_names_the_missing_transform(self) -> None:
        result = self.run_preview({"mode": "point", "frame": "world",
                                   "goal": [0, 0, 0], "rpy_deg": None})
        self.assertFalse(result["ok"])
        self.assertIn("Vicon", result["error"],
                      "a world-frame refusal must say the transform is Vicon's")
        self.assertIn("mount", result["error"],
                      "the refusal must list the frames that do work")


class PreviewIsReadOnly(unittest.TestCase):
    """No code path from the preview to a request, stated as text.

    A textual test is legitimate here because the property is textual: the
    preview module and handler must CONTAIN no network primitive, so that no
    future branch of them can reach one either.
    """

    # The primitives a request would need. scene.js is allowed to MENTION
    # /api/dh in a comment (it documents where the DH table comes from), so
    # the endpoint-path check applies to the handler only.
    NETWORK_RE = re.compile(
        r"\bfetch\s*\(|XMLHttpRequest|WebSocket|EventSource"
        r"|postJSON|getJSON")
    HANDLER_RE = re.compile(NETWORK_RE.pattern + r"|/api/")

    def test_scene_js_contains_no_network_primitive_at_all(self) -> None:
        text = SCENE_JS.read_text(encoding="utf-8")
        self.assertIsNone(self.NETWORK_RE.search(text),
                          "scene.js gained a network call; the preview's "
                          "read-only guarantee rests on it having none")

    def test_the_preview_handler_makes_no_request(self) -> None:
        text = PANEL_JS.read_text(encoding="utf-8")
        match = re.search(
            r"function previewGoalCard\(.*?\n\}", text, re.DOTALL)
        self.assertIsNotNone(match, "previewGoalCard is missing from panel.js")
        body = match.group(0)
        self.assertIsNone(self.HANDLER_RE.search(body),
                          "previewGoalCard must not make any request")
        for forbidden in ("saveGoalCard", "solvePlan", "session"):
            self.assertNotIn(forbidden, body,
                             f"previewGoalCard must not reach {forbidden}")


if __name__ == "__main__":
    unittest.main()
