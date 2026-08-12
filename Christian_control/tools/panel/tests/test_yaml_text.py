"""Tests for yaml_text, the panel's comment-preserving YAML-subset editor.

Every edit test asserts two things: the edit landed, and every unrelated
byte survived. Comment preservation is the module's whole reason to exist.
"""

import unittest

from Christian_control.tools.panel import yaml_text

DOC = """\
# top comment
session_arms: left

motion:
  # how fast
  nominal_speed_mps: 0.05
  waypoints: 10

left:
  frame: mount
  path:
    type: circle
    centre: [ 0.31, 0.386, 0.5213 ]
  # trailing note about left
velocity_limits:
  actuator_1:
    lower_limit: -0.8727  # -50.0 deg/s
    unit: "rad/s"
"""


class Locate(unittest.TestCase):
    def test_finds_nested_keys(self):
        lines = DOC.split("\n")
        self.assertEqual(
            yaml_text.locate(lines, ("motion", "nominal_speed_mps")), 5)
        self.assertEqual(
            yaml_text.locate(lines, ("velocity_limits", "actuator_1",
                                     "lower_limit")), 16)

    def test_search_is_bounded_by_the_parent_block(self):
        # 'frame' exists only under left; asking for it under motion is None.
        lines = DOC.split("\n")
        self.assertIsNone(yaml_text.locate(lines, ("motion", "frame")))

    def test_absent_key_is_none(self):
        self.assertIsNone(yaml_text.locate(DOC.split("\n"), ("nowhere",)))


class ReadValue(unittest.TestCase):
    def test_scalars_lists_and_strings(self):
        self.assertEqual(
            yaml_text.read_value(DOC, ("motion", "nominal_speed_mps")), 0.05)
        self.assertEqual(
            yaml_text.read_value(DOC, ("left", "path", "centre")),
            [0.31, 0.386, 0.5213])
        self.assertEqual(yaml_text.read_value(DOC, ("left", "frame")), "mount")


class ReplaceValue(unittest.TestCase):
    def test_replaces_only_the_value_and_keeps_every_comment(self):
        out = yaml_text.replace_value(
            DOC, ("motion", "nominal_speed_mps"), "0.1")
        self.assertIn("nominal_speed_mps: 0.1", out)
        self.assertIn("# how fast", out)
        self.assertIn("# top comment", out)
        # Only that one line changed.
        diff = [(a, b) for a, b in zip(DOC.split("\n"), out.split("\n"))
                if a != b]
        self.assertEqual(len(diff), 1)

    def test_drops_the_stale_inline_comment_on_the_changed_line(self):
        out = yaml_text.replace_value(
            DOC, ("velocity_limits", "actuator_1", "lower_limit"), "-1.0")
        self.assertIn("lower_limit: -1.0", out)
        self.assertNotIn("-50.0 deg/s", out)
        self.assertIn('unit: "rad/s"', out)   # neighbour untouched

    def test_absent_key_returns_none_and_never_invents_one(self):
        self.assertIsNone(yaml_text.replace_value(DOC, ("motion", "zeta"), "1"))


class InsertKey(unittest.TestCase):
    def test_inserts_at_the_end_of_the_parent_block_with_its_indent(self):
        out = yaml_text.insert_key(DOC, ("motion",), "min_duration_s", "4.0")
        lines = out.split("\n")
        i = yaml_text.locate(lines, ("motion", "min_duration_s"))
        self.assertIsNotNone(i)
        self.assertTrue(lines[i].startswith("  min_duration_s:"))
        # It landed inside motion, before the left block begins.
        self.assertLess(i, yaml_text.locate(lines, ("left",)))

    def test_missing_parent_returns_none(self):
        self.assertIsNone(yaml_text.insert_key(DOC, ("ghost",), "k", "1"))


class RemoveKey(unittest.TestCase):
    def test_removes_a_nested_block_and_its_children_only(self):
        out = yaml_text.remove_key(DOC, ("left", "path"))
        self.assertNotIn("type: circle", out)
        self.assertNotIn("centre:", out)
        self.assertIn("frame: mount", out)
        self.assertIn("# trailing note about left", out)

    def test_removes_a_single_scalar_line(self):
        out = yaml_text.remove_key(DOC, ("left", "frame"))
        self.assertNotIn("frame: mount", out)
        self.assertIn("type: circle", out)

    def test_absent_key_returns_none(self):
        self.assertIsNone(yaml_text.remove_key(DOC, ("left", "box")))


class Render(unittest.TestCase):
    def test_each_type_renders_as_the_files_write_it(self):
        self.assertEqual(yaml_text.render(True), "true")
        self.assertEqual(yaml_text.render(False), "false")
        self.assertEqual(yaml_text.render(0.5), "0.5")
        self.assertEqual(yaml_text.render(10), "10")
        self.assertEqual(yaml_text.render([1.0, 2.0, 3.0]), "[ 1.0, 2.0, 3.0 ]")
        self.assertEqual(yaml_text.render("mount"), "mount")
