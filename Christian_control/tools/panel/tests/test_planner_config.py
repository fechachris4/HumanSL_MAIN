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
                          ("goal.position_sigma_xyz", [0.01, -0.01, 0.01]),
                          # float() takes these, and "inf > 0" passes every
                          # range rule in the table, so the range check alone
                          # would write a planner file no solve can use.
                          ("motion.nominal_speed_mps", "inf"),
                          ("motion.nominal_speed_mps", "nan"),
                          ("motion.min_duration_s", "-inf"),
                          ("path_following.validation_dt_s", "Infinity"),
                          ("goal.position_sigma_xyz", [0.01, "inf", 0.01]),
                          ("goal.rotation_sigma_rpy", ["nan", 0.1, 0.1])]:
            ok, why = planner_config.write_planner_knob(name, bad, self.yaml)
            self.assertFalse(ok, f"{name}={bad!r} should be refused")
            self.assertTrue(why, name)
        self.assert_unchanged()

    def test_a_non_finite_value_is_refused_by_name(self):
        ok, why = planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "inf", self.yaml)
        self.assertFalse(ok)
        self.assertIn("finite", why)
        self.assert_unchanged()


class JointLimitsRead(unittest.TestCase):
    def test_all_sections_actuators_and_bounds_are_present(self):
        table = planner_config.read_joint_limits_file()
        for section in planner_config.LIMIT_SECTIONS:
            for actuator in planner_config.ACTUATORS:
                entry = table[section][actuator]
                self.assertIsNotNone(entry["lower_limit"], (section, actuator))
                self.assertIsNotNone(entry["upper_limit"], (section, actuator))
                self.assertTrue(entry["dangerous"], (section, actuator))

    def test_acceleration_is_not_offered_because_nothing_reads_it(self):
        # createJointLimits (trajectory_generation/src/utils.cpp) reads
        # position_limits and velocity_limits only; PlanSolver.cpp sets each
        # acceleration bound to velocity upper x 2. Offering the file's
        # acceleration table would be an edit with no effect.
        self.assertNotIn("acceleration_limits", planner_config.LIMIT_SECTIONS)
        self.assertNotIn("acceleration_limits",
                         planner_config.read_joint_limits_file())

    def test_known_values(self):
        table = planner_config.read_joint_limits_file()
        self.assertAlmostEqual(
            table["position_limits"]["actuator_2"]["upper_limit"], 2.2515)
        self.assertAlmostEqual(
            table["velocity_limits"]["actuator_1"]["lower_limit"], -0.8727)
        self.assertEqual(
            table["position_limits"]["actuator_2"]["unit"], '"radians"')


class JointLimitsWrite(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_jl_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "joint_limits.yaml"
        shutil.copy2(paths.JOINT_LIMITS_YAML, self.yaml)
        self.original = self.yaml.read_text()

    def test_round_trip_touches_one_section_only(self):
        ok, value = planner_config.write_joint_limit(
            "velocity_limits", "actuator_3", "upper_limit", "1.0", self.yaml)
        self.assertTrue(ok)
        self.assertEqual(value, 1.0)
        table = planner_config.read_joint_limits_file(self.yaml)
        self.assertEqual(
            table["velocity_limits"]["actuator_3"]["upper_limit"], 1.0)
        # The same actuator's POSITION limit is untouched — the sections
        # repeat actuator names, so this is the aliasing bug to guard.
        self.assertAlmostEqual(
            table["position_limits"]["actuator_3"]["lower_limit"], -1e20)

    def test_lower_must_stay_below_upper(self):
        ok, why = planner_config.write_joint_limit(
            "velocity_limits", "actuator_3", "lower_limit", "2.0", self.yaml)
        self.assertFalse(ok)
        self.assertIn("upper", why)
        self.assertEqual(self.yaml.read_text(), self.original)

    def test_bad_names_and_values_refused(self):
        for args in [("speed_limits", "actuator_1", "lower_limit", "1"),
                     ("velocity_limits", "actuator_9", "lower_limit", "1"),
                     ("velocity_limits", "actuator_1", "unit", "x"),
                     ("velocity_limits", "actuator_1", "lower_limit", "slow")]:
            ok, _ = planner_config.write_joint_limit(*args, self.yaml)
            self.assertFalse(ok, args)
        self.assertEqual(self.yaml.read_text(), self.original)

    def test_non_finite_bounds_are_refused(self):
        # Nothing downstream validates joint_limits.yaml, so this is the only
        # check there is. NaN is the dangerous one: every comparison with it
        # is false, so it slips straight past the lower-below-upper test.
        for value in ("nan", "inf", "-inf", "Infinity", "NaN"):
            ok, why = planner_config.write_joint_limit(
                "velocity_limits", "actuator_1", "upper_limit", value,
                self.yaml)
            self.assertFalse(ok, value)
            self.assertIn("finite", why)
        self.assertEqual(self.yaml.read_text(), self.original)
