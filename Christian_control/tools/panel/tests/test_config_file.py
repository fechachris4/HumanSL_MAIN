"""Tests for config_file.

One rule shapes every write test here: the real Config.h is read but never
written. Every write test works on a copy in a temporary directory, so a
failing test can never leave the controller's compiled settings altered.

Guard overrides are on the whitelist and writable — Christian's explicit
choice, 2026-08-12, reversing the prior exclusion (see the module docstring
and docs/intent/story.md's "Superseded decisions"). What these tests now
guard is that the reversal is total and visible: each guard override is
really on KNOBS, really writable, and really flagged `dangerous` for the
UI — not silently still refused, and not silently indistinguishable from an
ordinary gain.
"""

import shutil
import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import config_file, paths


class KnobWhitelist(unittest.TestCase):
    def test_every_knob_is_found_in_the_real_config(self):
        knobs = config_file.read_knobs()
        self.assertEqual(len(knobs), 16)
        missing = [name for name, knob in knobs.items() if knob["value"] is None]
        self.assertEqual(missing, [], "knob(s) no longer match Config.h")

    def test_known_values_read_back(self):
        knobs = config_file.read_knobs()
        self.assertEqual(knobs["kKpCartesian"]["value"], "10.0")
        self.assertEqual(knobs["kOrientationEnabled"]["value"], "true")

    def test_guard_overrides_are_on_the_whitelist_by_explicit_choice(self):
        knobs = config_file.read_knobs()
        for name in config_file.GUARD_OVERRIDES:
            self.assertIn(name, config_file.KNOBS, name)
            self.assertIn(name, knobs, name)

    def test_guard_overrides_are_flagged_dangerous_and_nothing_else_is(self):
        knobs = config_file.read_knobs()
        for name, entry in knobs.items():
            expected = name in config_file.GUARD_OVERRIDES
            self.assertEqual(entry["dangerous"], expected, name)

    def test_guard_overrides_exist_in_config_h_under_those_names(self):
        # If Config.h renamed one, the whitelist above would be exposing a
        # name that no longer means anything, so pin the names to the file.
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

    def test_guard_overrides_round_trip_like_any_other_bool_knob(self):
        for name in config_file.GUARD_OVERRIDES:
            ok, rendered = config_file.write_knob(name, "true", self.config)
            self.assertTrue(ok, name)
            self.assertEqual(rendered, "true")
            self.assertEqual(
                config_file.read_knobs(self.config)[name]["value"], "true", name)

    def test_unknown_name_is_refused(self):
        ok, message = config_file.write_knob("kNotAKnob", "1.0", self.config)
        self.assertFalse(ok)
        self.assertIn("whitelist", message)
        self.assert_unchanged()


class VectorKnobWhitelist(unittest.TestCase):
    def test_the_alias_is_never_whitelisted(self):
        # kQdotLimitDegS = kModelVelocityLimitsDegS in Config.h; writing the
        # alias's name would change nothing the controller reads, so only
        # the real constant may ever be on this whitelist.
        self.assertNotIn("kQdotLimitDegS", config_file.VECTOR_KNOBS)
        self.assertIn("kModelVelocityLimitsDegS", config_file.VECTOR_KNOBS)

    def test_the_real_constant_reads_back(self):
        # Compare against the constant as written in Config.h, not a pinned
        # number: the velocity limit is a knob the panel is designed to edit,
        # so a legitimate edit must not break this test. What it guards is
        # that read_vector_knobs returns exactly what the file holds.
        source = config_file.parse_joint_vector(
            paths.CONFIG_H.read_text(), "kModelVelocityLimitsDegS",
            alias_depth=0)
        self.assertIsNotNone(source)
        knobs = config_file.read_vector_knobs()
        self.assertEqual(knobs["kModelVelocityLimitsDegS"]["value"], source)


class WriteVectorAgainstACopy(unittest.TestCase):
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
        new_values = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
        ok, rendered = config_file.write_vector_knob(
            "kModelVelocityLimitsDegS", new_values, self.config)
        self.assertTrue(ok)
        self.assertEqual(rendered, new_values)
        self.assertEqual(
            config_file.read_vector_knobs(self.config)
            ["kModelVelocityLimitsDegS"]["value"],
            new_values)
        # A neighbouring scalar knob survived the rewrite untouched.
        self.assertEqual(
            config_file.read_knobs(self.config)["kKpCartesian"]["value"],
            "10.0")

    def test_backup_is_written_once_and_holds_the_original(self):
        config_file.write_vector_knob(
            "kModelVelocityLimitsDegS", [1, 2, 3, 4, 5, 6, 7], self.config)
        self.assertTrue(self.backup.exists())
        self.assertEqual(self.backup.read_text(), self.original)
        config_file.write_vector_knob(
            "kModelVelocityLimitsDegS", [8, 9, 10, 11, 12, 13, 14], self.config)
        self.assertEqual(self.backup.read_text(), self.original)

    def test_accepts_numeric_strings_the_same_as_numbers(self):
        # The panel posts trimmed strings, not client-side-parsed numbers, so
        # an invalid entry fails server-side rather than silently becoming 0.
        ok, rendered = config_file.write_vector_knob(
            "kModelVelocityLimitsDegS",
            ["1", "2", "3", "4", "5", "6", "7"], self.config)
        self.assertTrue(ok)
        self.assertEqual(rendered, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

    def test_wrong_length_is_refused(self):
        ok, message = config_file.write_vector_knob(
            "kModelVelocityLimitsDegS", [1, 2, 3], self.config)
        self.assertFalse(ok)
        self.assertIn("7", message)
        self.assert_unchanged()

    def test_a_non_numeric_entry_is_refused(self):
        ok, message = config_file.write_vector_knob(
            "kModelVelocityLimitsDegS",
            [1, 2, 3, 4, 5, "fast", 7], self.config)
        self.assertFalse(ok)
        self.assertIn("number", message)
        self.assert_unchanged()

    def test_an_empty_entry_is_refused_not_silently_zero(self):
        ok, message = config_file.write_vector_knob(
            "kModelVelocityLimitsDegS",
            [1, 2, 3, 4, 5, "", 7], self.config)
        self.assertFalse(ok)
        self.assertIn("number", message)
        self.assert_unchanged()

    def test_unknown_vector_name_is_refused(self):
        ok, message = config_file.write_vector_knob(
            "kNotAVector", [1, 2, 3, 4, 5, 6, 7], self.config)
        self.assertFalse(ok)
        self.assertIn("whitelist", message)
        self.assert_unchanged()

    def test_the_alias_cannot_be_written(self):
        ok, message = config_file.write_vector_knob(
            "kQdotLimitDegS", [1, 2, 3, 4, 5, 6, 7], self.config)
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
        # The alias kQdotLimitDegS must resolve to the same seven numbers as
        # the source constant — whatever they currently are.
        source = config_file.parse_joint_vector(
            paths.CONFIG_H.read_text(), "kModelVelocityLimitsDegS",
            alias_depth=0)
        self.assertEqual(self.limits["velocity_limit_deg_s"], source)

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
