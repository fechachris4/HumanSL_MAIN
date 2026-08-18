"""The browser's forward kinematics must agree with the C++ it draws.

scene.js computes forward kinematics itself so the page can draw the arm at
20 Hz without asking the server for link positions. That makes it a second
implementation of the robot's geometry, and a second implementation is a
second thing that can be wrong. This test pins it against the first one.

The oracle is tools/print_dual_arm_fk, which loads the canonical URDF through
Pinocchio. Its own header says it reads the URDF only — no Kortex, no
connection, no motion — and `ldd` confirms it links no Kortex library, so
running it cannot reach an arm.

Tolerance: the generated DH table carries the URDF's rounded rpy values, which
puts a floor of roughly 2e-5 m on any comparison against Pinocchio. Measured
agreement across the configurations below is under 1e-5 m, so 5e-5 m leaves
room without hiding a real divergence.
"""

from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import unittest
from pathlib import Path

from Christian_control.panel import paths

FK_BINARY = paths.PRINT_DUAL_ARM_FK
SCENE_JS = paths.STATIC / "scene.js"

TOLERANCE_M = 5e-5

# Joint angles in degrees, right arm. Zero, then three configurations chosen to
# move every joint off its zero so an error in one link's DH entry cannot hide.
CONFIGURATIONS = [
    [0, 0, 0, 0, 0, 0, 0],
    [30, 45, -20, 60, 10, -35, 15],
    [90, 20, 0, 90, 0, 45, 0],
    [-45, -30, 120, -90, 60, 30, -60],
]

# Loads the real DH table the server would serve, runs the real exported
# function from scene.js, and prints the tool origin. Kept here rather than in
# a checked-in .mjs file so the test cannot drift from what it claims to test.
NODE_HARNESS = """
import { forwardKinematics } from '%(scene)s';
import { readFileSync } from 'fs';

const text = readFileSync('%(dh)s', 'utf8');
const joints = [];
let current = null;
for (const line of text.split('\\n')) {
    if (line.trim().startsWith('#')) continue;
    const opened = line.match(/^\\s*-\\s*joint_id:\\s*(\\d+)/);
    if (opened) { current = { joint_id: +opened[1] }; joints.push(current); continue; }
    if (!current) continue;
    const scalar = line.match(/^\\s+(\\w+):\\s*"?([^"\\n]*?)"?\\s*$/);
    if (!scalar) continue;
    const value = parseFloat(scalar[2]);
    current[scalar[1]] = Number.isNaN(value) ? scalar[2] : value;
}

const angles = JSON.parse(process.argv[2]);
const result = forwardKinematics(joints, angles, null);
if (!result) { console.log(JSON.stringify({ tool: null })); }
else {
    const m = result.tool;
    console.log(JSON.stringify({ tool: [m[3], m[7], m[11]], links: result.points.length }));
}
"""


def _cpp_tool_position(angles: list[float]) -> list[float]:
    """The right arm's tool origin in its own base_link, from Pinocchio."""
    proc = subprocess.run(
        [str(FK_BINARY), *[str(a) for a in angles]],
        capture_output=True, text=True, timeout=60, check=True)
    for line in proc.stdout.splitlines():
        fields = line.split()
        # "    base_link  p   x   y   z   rpy ..." — the right arm prints first.
        if len(fields) >= 5 and fields[0] == "base_link" and fields[1] == "p":
            return [float(fields[2]), float(fields[3]), float(fields[4])]
    raise AssertionError(f"no base_link line in:\n{proc.stdout}")


def _browser_tool_position(angles: list[float], tmp: Path) -> dict:
    harness = tmp / "fk_harness.mjs"
    harness.write_text(NODE_HARNESS % {
        "scene": SCENE_JS.as_posix(),
        "dh": paths.DH_YAML["right"].as_posix(),
    }, encoding="utf-8")
    proc = subprocess.run(
        ["node", str(harness), json.dumps(angles)],
        capture_output=True, text=True, timeout=60, check=True)
    return json.loads(proc.stdout.strip().splitlines()[-1])


class BrowserFkMatchesTheCpp(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if shutil.which("node") is None:
            raise unittest.SkipTest("node is not installed; cannot run scene.js")
        if not FK_BINARY.is_file():
            raise unittest.SkipTest(
                "print_dual_arm_fk has not been built in this checkout")
        if not paths.DH_YAML["right"].is_file():
            raise unittest.SkipTest(
                "the generated DH table is missing; build planner_bridge")
        if not SCENE_JS.is_file():
            raise unittest.SkipTest("scene.js is missing")

    def setUp(self) -> None:
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def test_tool_position_agrees_at_every_configuration(self) -> None:
        for angles in CONFIGURATIONS:
            with self.subTest(q=angles):
                expected = _cpp_tool_position(angles)
                got = _browser_tool_position(angles, self.tmp)["tool"]
                self.assertIsNotNone(got, "scene.js refused a valid configuration")
                distance = math.dist(expected, got)
                self.assertLess(
                    distance, TOLERANCE_M,
                    f"q={angles}: browser {got} vs C++ {expected}, "
                    f"{distance:.2e} m apart")

    def test_seven_joints_produce_eight_frames(self) -> None:
        """Base origin plus one per joint — the link count the scene draws."""
        result = _browser_tool_position(CONFIGURATIONS[1], self.tmp)
        self.assertEqual(result["links"], 8)

    def test_a_non_finite_angle_draws_no_arm(self) -> None:
        """A row of NaN must produce nothing, never an arm folded at the origin."""
        harness = self.tmp / "nan.mjs"
        harness.write_text(NODE_HARNESS % {
            "scene": SCENE_JS.as_posix(),
            "dh": paths.DH_YAML["right"].as_posix(),
        }, encoding="utf-8")
        proc = subprocess.run(
            ["node", str(harness), '[0,0,null,0,0,0,0]'],
            capture_output=True, text=True, timeout=60, check=True)
        self.assertIsNone(json.loads(proc.stdout.strip())["tool"])


class SceneTestPageMatchesTheGeneratedTable(unittest.TestCase):
    """scene.test.html hardcodes the DH table so it needs no server.

    That copy can drift from the generated one without anyone noticing, and a
    drifted copy would make the eye-check page quietly show a different robot
    from the panel. Compare the numbers rather than trust them.
    """

    PAGE = paths.STATIC / "scene.test.html"

    def test_every_d_value_matches(self) -> None:
        if not self.PAGE.is_file():
            self.skipTest("scene.test.html is missing")
        if not paths.DH_YAML["right"].is_file():
            self.skipTest("the generated DH table is missing; build planner_bridge")

        from Christian_control.panel import dh as dh_module
        generated = dh_module.read("right")["joints"]

        text = self.PAGE.read_text(encoding="utf-8")
        found = re.findall(
            r"joint_id:\s*(\d+).*?a:\s*(-?[\d.eE+]+).*?alpha:\s*(-?[\d.eE+]+)"
            r".*?d:\s*(-?[\d.eE+]+).*?theta_offset:\s*(-?[\d.eE+]+)", text)
        self.assertEqual(len(found), len(generated),
                         "the page does not list every joint")
        for (jid, a, alpha, d, offset), joint in zip(found, generated):
            with self.subTest(joint=jid):
                self.assertEqual(int(jid), joint["joint_id"])
                for name, page_value in (("a", a), ("alpha", alpha),
                                         ("d", d), ("theta_offset", offset)):
                    self.assertAlmostEqual(
                        float(page_value), joint[name], places=9,
                        msg=f"joint {jid} {name} has drifted from the generated table")


if __name__ == "__main__":
    unittest.main()
