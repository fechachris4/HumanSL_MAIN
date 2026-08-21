import pytest

from Christian_control.panel import plan


GOAL_WITH_BOX = """session_arms: right
right:
  frame: mount
  goal: [0.5, 0.0, 0.4]
  box:
    center: [0.0, 0.0, 0.0]
    half_extent: [0.1, 0.1, 0.1]
"""


def test_goal_box_is_rejected_as_retired():
    assert plan.validate_goal(GOAL_WITH_BOX) == [
        "right: box is retired — edit obstacles.scene in planner.yaml"
    ]


@pytest.mark.parametrize("box_key", ['"box"', "'box'"])
def test_quoted_goal_box_is_rejected_without_changing_file(tmp_path, box_key):
    proposed = f"""session_arms: right
right:
  frame: mount
  goal: [0.5, 0.0, 0.4]
  {box_key}:
    center: [0.0, 0.0, 0.0]
    half_extent: [0.1, 0.1, 0.1]
"""
    goal_path = tmp_path / "goal.yaml"
    original = """session_arms: right
right:
  frame: mount
  goal: [0.4, 0.0, 0.3]
"""
    goal_path.write_text(original)

    assert plan.validate_goal(proposed) == [
        "right: box is retired — edit obstacles.scene in planner.yaml"
    ]
    ok, message = plan.write_goal(proposed, goal_path)

    assert not ok
    assert message == (
        "right: box is retired — edit obstacles.scene in planner.yaml")
    assert goal_path.read_text() == original


def test_commented_quoted_box_example_is_not_active():
    text = """session_arms: right
right:
  frame: mount
  goal: [0.5, 0.0, 0.4]
  # "box":
  #   center: [0.0, 0.0, 0.0]
"""
    assert plan.validate_goal(text) == []


def test_structured_goal_box_is_rejected_without_changing_file(tmp_path):
    goal_path = tmp_path / "goal.yaml"
    original = """session_arms: right
right:
  frame: mount
  goal: [0.5, 0.0, 0.4]
"""
    goal_path.write_text(original)

    ok, message = plan.write_goal_fields(
        "right",
        {
            "mode": "point",
            "frame": "mount",
            "goal": ["0.4", "0.0", "0.3"],
            "box": {
                "center": ["0.0", "0.0", "0.0"],
                "half_extent": ["0.1", "0.1", "0.1"],
            },
        },
        goal_path,
    )

    assert not ok
    assert message == "box is retired — edit obstacles.scene in planner.yaml"
    assert goal_path.read_text() == original
