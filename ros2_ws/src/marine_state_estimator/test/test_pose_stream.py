"""Plain pytest, no ROS2 required. The sample text below is a real captured
excerpt of `gz topic -e -t /world/sydney_regatta/pose/info -n 1` output from
this project's own M1 diagnostic session (docs/environment.md), trimmed to
a few entities so the test fixture stays readable."""

import pytest
from marine_state_estimator.pose_stream import parse_named_pose

SAMPLE = """header {
  stamp {
    sec: 12
    nsec: 704000000
  }
}
pose {
  name: "wamv"
  id: 143
  position {
    x: -532.00426109262992
    y: 161.99256174487309
    z: -0.070469986336969825
  }
  orientation {
    x: -0.0032132140509756341
    y: 0.012091396946025379
    z: 0.47944326332889436
    w: 0.87748369251165959
  }
}
pose {
  name: "link"
  id: 5
  position {
    x: 0.0
    y: 0.0
    z: 0.0
  }
  orientation {
    w: 1.0
  }
}
pose {
  name: "mb_marker_buoy_black"
  id: 200
  position {
    x: -528.0
    y: 191.0
    z: 0.29
  }
  orientation {
    w: 1.0
  }
}
"""


def test_finds_named_entity():
    pose = parse_named_pose(SAMPLE, "wamv")
    assert pose is not None
    assert pose.position == pytest.approx(
        (-532.00426109262992, 161.99256174487309, -0.070469986336969825)
    )
    assert pose.orientation[3] == pytest.approx(0.87748369251165959)


def test_finds_a_later_entity_not_just_the_first():
    pose = parse_named_pose(SAMPLE, "mb_marker_buoy_black")
    assert pose is not None
    assert pose.position == pytest.approx((-528.0, 191.0, 0.29))


def test_missing_orientation_defaults_to_identity():
    pose = parse_named_pose(SAMPLE, "link")
    assert pose is not None
    assert pose.position == (0.0, 0.0, 0.0)
    assert pose.orientation == (0.0, 0.0, 0.0, 1.0)


def test_unknown_entity_returns_none():
    assert parse_named_pose(SAMPLE, "does_not_exist") is None


def test_does_not_partial_match_a_longer_name():
    # "wamv" must not match e.g. a hypothetical "wamv_extra" entity.
    text_with_similar_name = SAMPLE.replace('name: "wamv"', 'name: "wamv_extra"')
    assert parse_named_pose(text_with_similar_name, "wamv") is None
