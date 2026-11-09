"""Unit tests for alberto_pipeline.   Run them with:  sb-test

CI runs every package's test/ folder on each Pull Request, so these run
there too. They test the plain functions (compute, plan, find_cones):
no ROS, no simulator, so they take milliseconds.

The first tests check the contract: what goes out has the right shape and
range. They pass with the starter code and must keep passing with yours.
The last one checks behaviour and starts switched off: turn it on when
your control steers, then add your own tests in the same style.
"""
import numpy as np
import pytest
from sandbox_msgs.msg import Cone

from alberto_pipeline import control
from alberto_pipeline.control import compute
from alberto_pipeline.perception import find_cones
from alberto_pipeline.planning import plan

STRAIGHT = np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]])
LEFT_BEND = np.array([[0.0, 0.0], [3.0, 0.5], [6.0, 2.0], [8.0, 4.0]])   # + y = left


# ---------- control ----------

@pytest.mark.parametrize("path", [STRAIGHT, LEFT_BEND])
@pytest.mark.parametrize("speed", [0.0, 5.0, 15.0])
def test_control_outputs_are_in_range(path, speed):
    throttle, steering, brake = compute(path, speed)
    assert 0.0 <= throttle <= 1.0
    assert -1.0 <= steering <= 1.0
    assert 0.0 <= brake <= 1.0


def test_control_steers_left_on_a_left_bend():
    _, steering, _ = compute(LEFT_BEND, 5.0)
    assert steering > 0.0          # + = left


def test_control_steers_right_on_a_right_bend():
    control._state.update(steer=0.0, i=0.0)
    _, steering, _ = compute(LEFT_BEND * (1.0, -1.0), 5.0)
    assert steering < 0.0


def test_control_slows_down_for_a_bend():
    s = np.linspace(0.0, 20.0, 81)
    bend = np.column_stack((4.0 * np.sin(s / 4.0), 4.0 - 4.0 * np.cos(s / 4.0)))   # radius 4 m
    assert control.target_speed(control.resample(bend)) < control.target_speed(control.resample(STRAIGHT))


def test_control_brakes_without_a_path():
    throttle, _, brake = compute(np.zeros((1, 2)), 3.0)
    assert throttle == 0.0 and brake == 1.0


# ---------- planning ----------

def test_planning_returns_a_path_from_the_car():
    blue = np.array([[x, 1.5] for x in range(2, 20, 4)], dtype=float)
    yellow = np.array([[x, -1.5] for x in range(2, 20, 4)], dtype=float)
    cones = np.vstack((blue, yellow))
    colors = np.array([Cone.BLUE] * len(blue) + [Cone.YELLOW] * len(yellow), dtype=np.uint8)

    path = np.asarray(plan(cones, colors))
    assert path.ndim == 2 and path.shape[1] == 2
    assert np.allclose(path[0], [0.0, 0.0], atol=0.5)      # starts at the car


def test_planning_survives_no_cones():
    path = np.asarray(plan(np.empty((0, 2)), np.empty(0, dtype=np.uint8)))
    assert path.ndim == 2 and path.shape[1] == 2


# ---------- perception ----------

def test_perception_survives_an_empty_scan():
    xy, colors = find_cones(np.empty((0, 4), dtype=np.float32))
    assert np.asarray(xy).reshape(-1, 2).shape[0] == len(colors) == 0


def test_perception_returns_one_colour_per_cone():
    rng = np.random.default_rng(0)
    ground = np.column_stack((rng.uniform(0, 20, 2000), rng.uniform(-5, 5, 2000),
                              np.full(2000, -1.10), rng.uniform(0, 0.1, 2000)))
    xy, colors = find_cones(ground.astype(np.float32))
    assert np.asarray(xy).reshape(-1, 2).shape[0] == len(colors)
