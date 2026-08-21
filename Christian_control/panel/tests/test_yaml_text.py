from Christian_control.panel import yaml_text


def test_replace_block_preserves_unrelated_text_byte_for_byte():
    original = """# header
obstacles:
  epsilon_dist_m: 0.05  # keep this
  scene: {}
solver:
  max_iterations: 1000
"""

    changed = yaml_text.replace_block(
        original,
        ("obstacles", "scene"),
        "scene:\n    torso:\n      enabled: true",
    )

    assert changed == """# header
obstacles:
  epsilon_dist_m: 0.05  # keep this
  scene:
    torso:
      enabled: true
solver:
  max_iterations: 1000
"""


def test_replace_block_replaces_nested_children_once():
    original = """obstacles:
  epsilon_dist_m: 0.05
  scene:
    old:
      enabled: false
      shape: box
      center_mount_m: [0.0, 0.0, 0.0]
      half_extent_m: [0.1, 0.1, 0.1]
  collision_sigma: 0.001
"""

    changed = yaml_text.replace_block(
        original,
        ("obstacles", "scene"),
        "scene:\n    new:\n      enabled: true",
    )

    assert changed == """obstacles:
  epsilon_dist_m: 0.05
  scene:
    new:
      enabled: true
  collision_sigma: 0.001
"""
    assert "old:" not in changed


def test_replace_block_preserves_trailing_comment_before_the_next_sibling():
    original = """obstacles:
  scene:
    old:
      enabled: false
  # collision weight is deliberately small
  collision_sigma: 0.001
"""

    changed = yaml_text.replace_block(
        original, ("obstacles", "scene"), "scene: {}"
    )

    assert changed == """obstacles:
  scene: {}
  # collision weight is deliberately small
  collision_sigma: 0.001
"""


def test_replace_block_returns_none_for_absent_path():
    original = "obstacles:\n  epsilon_dist_m: 0.05\n"

    assert yaml_text.replace_block(
        original, ("obstacles", "scene"), "scene: {}"
    ) is None


def test_replace_block_finds_single_and_double_quoted_simple_path_keys():
    original = """'obstacles':
  "scene": {}
solver:
  max_iterations: 1000
"""

    changed = yaml_text.replace_block(
        original,
        ("obstacles", "scene"),
        "scene:\n    torso:\n      enabled: true",
    )

    assert changed == """'obstacles':
  scene:
    torso:
      enabled: true
solver:
  max_iterations: 1000
"""
