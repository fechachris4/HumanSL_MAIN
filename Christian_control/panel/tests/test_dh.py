"""The generated DH tables are parsed the way the browser needs them."""

import math
import unittest

from Christian_control.panel import dh, paths

SAMPLE = """\
# GENERATED — DO NOT EDIT.
# Derived from the canonical URDF by generate_dh_params

dh_parameters:
  - joint_id: 1
    joint_name: "Actuator1"
    joint_type: "continuous"
    a: 0.0
    alpha: 1.5707963267948966
    d: -0.2848103248984093
    theta_offset: 0
  - joint_id: 2
    joint_name: "Actuator2"
    joint_type: "revolute"
    a: 0.0
    alpha: 1.5707963267948966
    d: -0.011748773153934661
    theta_offset: 3.1415926535897931
"""


class ParseDhYaml(unittest.TestCase):
    def test_reads_each_joint_in_order(self):
        joints = dh.parse_dh_yaml(SAMPLE)
        self.assertEqual([j["joint_id"] for j in joints], [1, 2])

    def test_numeric_fields_are_floats(self):
        first = dh.parse_dh_yaml(SAMPLE)[0]
        self.assertAlmostEqual(first["alpha"], math.pi / 2)
        self.assertAlmostEqual(first["d"], -0.2848103248984093)
        self.assertEqual(first["a"], 0.0)
        self.assertEqual(first["theta_offset"], 0.0)

    def test_names_and_types_keep_their_quotes_off(self):
        first = dh.parse_dh_yaml(SAMPLE)[0]
        self.assertEqual(first["joint_name"], "Actuator1")
        self.assertEqual(first["joint_type"], "continuous")

    def test_comments_are_ignored(self):
        self.assertEqual(len(dh.parse_dh_yaml("# nothing here\n")), 0)


class ReadTheRealTable(unittest.TestCase):
    """The generated file only exists after planner_bridge has been built."""

    def test_right_arm_table_has_seven_joints_when_generated(self):
        table = dh.read("right")
        if not table["exists"]:
            self.skipTest("planner_bridge has not been built in this checkout")
        self.assertEqual(len(table["joints"]), 7)
        for joint in table["joints"]:
            for field in ("a", "alpha", "d", "theta_offset"):
                self.assertIsInstance(joint[field], float)

    def test_unknown_arm_is_reported_not_raised(self):
        table = dh.read("middle")
        self.assertFalse(table["exists"])
        self.assertIn("unknown arm", table["reason"])

    def test_both_arms_are_known(self):
        self.assertEqual(set(paths.DH_YAML), {"right", "left"})


if __name__ == "__main__":
    unittest.main()
