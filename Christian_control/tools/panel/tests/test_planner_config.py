"""Tests for planner_config's planner.yaml knobs.

Same rule as test_config_file: the real planner.yaml is read but never
written; write tests work on a copy in a temporary directory.
"""

import shutil
import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import paths, planner_config


class Whitelist(unittest.TestCase):
    def test_every_knob_is_found_in_the_real_file(self):
        knobs = planner_config.read_planner_knobs()
        self.assertEqual(len(knobs), 20)
        missing = [n for n, k in knobs.items() if k["value"] is None]
        self.assertEqual(missing, [], "knob(s) no longer match planner.yaml")

    def test_known_values_read_back(self):
        knobs = planner_config.read_planner_knobs()
        self.assertEqual(knobs["motion.nominal_speed_mps"]["value"], 0.05)
        self.assertEqual(knobs["motion.waypoints"]["value"], 10.0)
        self.assertEqual(knobs["seeding.randomised"]["value"], "false")
        self.assertEqual(
            knobs["goal.position_sigma_xyz"]["value"], [0.01, 0.1, 0.01])

    def test_every_real_key_is_whitelisted(self):
        # planner.yaml refuses unknown keys, so the whitelist and the file
        # must cover each other exactly: a key in the file but not here would
        # be invisible in the browser, which is how "I changed it and nothing
        # happened" starts.
        text = paths.PLANNER_YAML.read_text()
        from Christian_control.tools.panel import yaml_text
        for dotted in planner_config.PLANNER_KNOBS:
            self.assertIsNotNone(
                yaml_text.read_value(text, tuple(dotted.split("."))), dotted)


class WriteAgainstACopy(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_planner_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "planner.yaml"
        shutil.copy2(paths.PLANNER_YAML, self.yaml)
        self.original = self.yaml.read_text()
        self.backup = paths.panel_backup(self.yaml)

    def assert_unchanged(self):
        self.assertEqual(self.yaml.read_text(), self.original)
        self.assertFalse(self.backup.exists())

    def test_round_trip_keeps_every_comment(self):
        ok, value = planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "0.1", self.yaml)
        self.assertTrue(ok)
        self.assertEqual(value, 0.1)
        text = self.yaml.read_text()
        self.assertEqual(
            planner_config.read_planner_knobs(self.yaml)[
                "motion.nominal_speed_mps"]["value"], 0.1)
        # Comment lines are untouched: same count, same content.
        original_comments = [l for l in self.original.split("\n")
                             if l.lstrip().startswith("#")]
        new_comments = [l for l in text.split("\n")
                        if l.lstrip().startswith("#")]
        self.assertEqual(new_comments, original_comments)

    def test_vector_write(self):
        ok, value = planner_config.write_planner_knob(
            "goal.position_sigma_xyz", [0.02, 0.02, 0.02], self.yaml)
        self.assertTrue(ok)
        self.assertEqual(
            planner_config.read_planner_knobs(self.yaml)[
                "goal.position_sigma_xyz"]["value"], [0.02, 0.02, 0.02])

    def test_bool_and_int(self):
        ok, _ = planner_config.write_planner_knob(
            "seeding.randomised", True, self.yaml)
        self.assertTrue(ok)
        ok, _ = planner_config.write_planner_knob(
            "solver.max_iterations", "500", self.yaml)
        self.assertTrue(ok)
        knobs = planner_config.read_planner_knobs(self.yaml)
        self.assertEqual(knobs["seeding.randomised"]["value"], "true")
        self.assertEqual(knobs["solver.max_iterations"]["value"], 500.0)

    def test_backup_written_once_holding_the_original(self):
        planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "0.1", self.yaml)
        planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "0.2", self.yaml)
        self.assertEqual(self.backup.read_text(), self.original)

    def test_unknown_key_refused(self):
        ok, why = planner_config.write_planner_knob("motion.zeta", "1", self.yaml)
        self.assertFalse(ok)
        self.assertIn("whitelist", why)
        self.assert_unchanged()

    def test_rule_violations_refused_with_a_sentence(self):
        for name, bad in [("motion.nominal_speed_mps", "0"),
                          ("motion.nominal_speed_mps", "-1"),
                          ("motion.waypoints", "1"),
                          ("path_following.approach_velocity_fraction", "1.5"),
                          ("solver.max_iterations", "0"),
                          ("motion.nominal_speed_mps", "fast"),
                          ("goal.position_sigma_xyz", [0.01, 0.01]),
                          ("goal.position_sigma_xyz", [0.01, -0.01, 0.01])]:
            ok, why = planner_config.write_planner_knob(name, bad, self.yaml)
            self.assertFalse(ok, f"{name}={bad!r} should be refused")
            self.assertTrue(why, name)
        self.assert_unchanged()
