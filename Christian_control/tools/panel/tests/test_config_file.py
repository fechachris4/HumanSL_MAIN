"""Tests for config_file.

Two rules shape these tests. The real Config.h is read but never written —
every write test works on a copy in a temporary directory, so a failing test
can never leave the controller's compiled settings altered. And the guard
overrides are checked from both sides: absent from the whitelist, and refused
by name if a future edit ever put one there.
"""

import shutil
import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import config_file, paths


class KnobWhitelist(unittest.TestCase):
    def test_every_knob_is_found_in_the_real_config(self):
        knobs = config_file.read_knobs()
        self.assertEqual(len(knobs), 15)
        missing = [name for name, knob in knobs.items() if knob["value"] is None]
        self.assertEqual(missing, [], "knob(s) no longer match Config.h")

    def test_known_values_read_back(self):
        knobs = config_file.read_knobs()
        self.assertEqual(knobs["kKpCartesian"]["value"], "10.0")
        self.assertEqual(knobs["kOrientationEnabled"]["value"], "true")

    def test_guard_overrides_are_not_on_the_whitelist(self):
        for name in config_file.GUARD_OVERRIDES:
            self.assertNotIn(name, config_file.KNOBS)

    def test_guard_overrides_exist_in_config_h_under_those_names(self):
        # If Config.h renamed one, the refusal above would be guarding a name
        # that no longer means anything, so pin the names to the real file.
        text = paths.CONFIG_H.read_text()
        for name in config_file.GUARD_OVERRIDES:
            self.assertIsNotNone(config_file.parse_scalar(text, name), name)


class WriteAgainstACopy(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_config_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.config = self.tmp / "Config.h"
        shutil.copy2(paths.CONFIG_H, self.config)
        self.original = self.config.read_text()
        self.backup = self.config.with_suffix(".h.panel.bak")

    def assert_unchanged(self):
        self.assertEqual(self.config.read_text(), self.original)
        self.assertFalse(self.backup.exists())

    def test_round_trip(self):
        ok, rendered = config_file.write_knob("kKpCartesian", "7.5", self.config)
        self.assertTrue(ok)
        self.assertEqual(rendered, "7.5")
        self.assertEqual(
            config_file.read_knobs(self.config)["kKpCartesian"]["value"], "7.5")
        # Everything else survived the rewrite.
        self.assertEqual(
            config_file.read_knobs(self.config)["kKpRotation"]["value"], "10.0")

    def test_backup_is_written_once_and_holds_the_original(self):
        config_file.write_knob("kKpCartesian", "7.5", self.config)
        self.assertTrue(self.backup.exists())
        self.assertEqual(self.backup.read_text(), self.original)
        config_file.write_knob("kKpCartesian", "8.5", self.config)
        self.assertEqual(self.backup.read_text(), self.original)

    def test_bool_knob_takes_true_false_and_json_booleans(self):
        ok, rendered = config_file.write_knob(
            "kOrientationEnabled", "false", self.config)
        self.assertTrue(ok)
        self.assertEqual(rendered, "false")
        ok, rendered = config_file.write_knob(
            "kOrientationEnabled", True, self.config)
        self.assertTrue(ok)
        self.assertEqual(rendered, "true")
        self.assertEqual(
            config_file.read_knobs(self.config)["kOrientationEnabled"]["value"],
            "true")

    def test_bool_knob_rejects_a_number(self):
        ok, message = config_file.write_knob("kOrientationEnabled", 1, self.config)
        self.assertFalse(ok)
        self.assertIn("true or false", message)
        self.assert_unchanged()

    def test_double_knob_rejects_text(self):
        ok, message = config_file.write_knob("kDlsLambda", "fast", self.config)
        self.assertFalse(ok)
        self.assertIn("number", message)
        self.assert_unchanged()

    def test_double_knob_accepts_an_integer_and_renders_it_as_a_double(self):
        ok, rendered = config_file.write_knob("kDlsLambda", "1", self.config)
        self.assertTrue(ok)
        self.assertEqual(rendered, "1.0")

    def test_guard_overrides_are_refused_and_the_file_is_untouched(self):
        for name in config_file.GUARD_OVERRIDES:
            ok, message = config_file.write_knob(name, "true", self.config)
            self.assertFalse(ok, name)
            self.assertIn("guard override", message)
        self.assert_unchanged()

    def test_unknown_name_is_refused(self):
        ok, message = config_file.write_knob("kNotAKnob", "1.0", self.config)
        self.assertFalse(ok)
        self.assertIn("whitelist", message)
        self.assert_unchanged()


class Thresholds(unittest.TestCase):
    def test_every_threshold_reads_from_the_real_config(self):
        thresholds = config_file.read_thresholds()
        for name, band in thresholds.items():
            self.assertIsNotNone(band["value"], name)
            self.assertTrue(band["consequence"])

    def test_the_values_the_run_screen_draws(self):
        thresholds = config_file.read_thresholds()
        self.assertAlmostEqual(thresholds["kArrivalToleranceM"]["value"], 0.001)
        self.assertAlmostEqual(
            thresholds["kFollowingErrorLimitDeg"]["value"], 3.0)
        self.assertAlmostEqual(
            thresholds["kTrajFollowingErrorStopDeg"]["value"], 8.0)
        self.assertAlmostEqual(
            thresholds["kTrajStartToleranceDeg"]["value"], 2.0)
        self.assertAlmostEqual(thresholds["kControlDtS"]["value"], 0.002)

    def test_orientation_tolerance_is_not_labelled_as_a_stop(self):
        band = config_file.read_thresholds()["kArrivalOrientationToleranceRad"]
        self.assertNotIn("stop", band["consequence"].split("—")[0])


class JointVectorParsing(unittest.TestCase):
    def setUp(self):
        self.text = paths.CONFIG_H.read_text()

    def test_multi_line_vector(self):
        self.assertEqual(
            config_file.parse_joint_vector(self.text, "kJointLowerDeg"),
            [0, -128.9, 0, -147.8, 0, -120.3, 0])

    def test_bounded_mask(self):
        self.assertEqual(
            config_file.parse_joint_vector(self.text, "kJointBoundedMask"),
            [0, 1, 0, 1, 0, 1, 0])

    def test_one_level_alias_is_resolved(self):
        self.assertEqual(
            config_file.parse_joint_vector(self.text, "kQdotLimitDegS"),
            config_file.parse_joint_vector(self.text, "kModelVelocityLimitsDegS"))

    def test_a_computed_vector_is_not_half_parsed(self):
        # kJointSoftwareLimitDeg is written as ternaries; read_joint_limits
        # recomputes it rather than pretending to parse C++ expressions.
        self.assertIsNone(
            config_file.parse_joint_vector(self.text, "kJointSoftwareLimitDeg"))

    def test_a_missing_vector_is_none(self):
        self.assertIsNone(
            config_file.parse_joint_vector(self.text, "kNoSuchVector"))


class JointLimits(unittest.TestCase):
    def setUp(self):
        self.limits = config_file.read_joint_limits()

    def test_only_joints_2_4_6_are_bounded(self):
        self.assertEqual(self.limits["bounded_mask"],
                         [False, True, False, True, False, True, False])

    def test_continuous_joints_carry_no_limit_at_all(self):
        for joint in (0, 2, 4, 6):
            for key in ("lower_deg", "upper_deg", "warn_deg", "error_deg",
                        "software_limit_deg"):
                self.assertIsNone(self.limits[key][joint], (key, joint))

    def test_software_limit_follows_the_config_rule(self):
        # min(upper - margin, warn), per joint.
        self.assertAlmostEqual(self.limits["software_limit_deg"][1], 126.9)
        self.assertAlmostEqual(self.limits["software_limit_deg"][3], 145.0)
        self.assertAlmostEqual(self.limits["software_limit_deg"][5], 118.0)
        self.assertAlmostEqual(self.limits["margin_deg"], 2.0)

    def test_bounded_joints_carry_their_ranges(self):
        self.assertAlmostEqual(self.limits["lower_deg"][1], -128.9)
        self.assertAlmostEqual(self.limits["upper_deg"][1], 128.9)
        self.assertAlmostEqual(self.limits["warn_deg"][5], 118.0)
        self.assertAlmostEqual(self.limits["error_deg"][5], 123.0)

    def test_every_array_is_seven_long(self):
        for key in ("bounded_mask", "lower_deg", "upper_deg", "warn_deg",
                    "error_deg", "software_limit_deg", "velocity_limit_deg_s"):
            self.assertEqual(len(self.limits[key]), 7, key)

    def test_velocity_limit_comes_through_the_alias(self):
        self.assertEqual(self.limits["velocity_limit_deg_s"], [45.0] * 7)

    def test_an_unparseable_config_leaves_everything_unbounded(self):
        # The safe direction to fail in: draw no limit rather than a guess.
        tmp = Path(tempfile.mkdtemp(prefix="panel_config_"))
        self.addCleanup(shutil.rmtree, tmp)
        empty = tmp / "Config.h"
        empty.write_text("namespace config {}\n")
        limits = config_file.read_joint_limits(empty)
        self.assertEqual(limits["bounded_mask"], [False] * 7)
        self.assertEqual(limits["software_limit_deg"], [None] * 7)


if __name__ == "__main__":
    unittest.main()
