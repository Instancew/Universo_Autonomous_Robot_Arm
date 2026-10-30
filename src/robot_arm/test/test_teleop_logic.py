"""Joystick jog logic: deadzone, speeds, limits, and the limits read from the real URDF."""
import math
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from robot_arm.teleop.jog_logic import (JOINTS, JointSpec, apply_deadzone, clamp, move_duration,  # noqa: E402
                                        parse_urdf_limits, step_targets)

URDF = (HERE.parent / 'description' / 'urdf' / 'robot_arm.urdf').read_text()
SPECS = parse_urdf_limits(URDF, JOINTS, {'base_joint': 0.5})


def test_limits_read_from_urdf():
    assert SPECS['base_joint'].continuous and SPECS['base_joint'].max_velocity == 0.5
    assert (SPECS['shoulder_joint'].lower, SPECS['shoulder_joint'].upper) == pytest.approx((-1.419011, -0.197281))
    assert (SPECS['elbow_joint'].lower, SPECS['elbow_joint'].upper) == pytest.approx((0.231847, 2.328057))
    assert SPECS['shoulder_joint'].max_velocity == pytest.approx(0.0420)
    assert SPECS['elbow_joint'].max_velocity == pytest.approx(0.0355)


def test_deadzone():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert apply_deadzone(-1.0, 0.1) == pytest.approx(-1.0)
    assert apply_deadzone(0.55, 0.1) == pytest.approx(0.5)


def test_full_stick_moves_at_max_speed():
    start = {'base_joint': 0.0, 'shoulder_joint': -0.5, 'elbow_joint': 1.0}
    out = step_targets(start, {'shoulder_joint': 1.0, 'elbow_joint': -1.0, 'base_joint': 0.5}, SPECS, dt=0.05)
    assert out['shoulder_joint'] - start['shoulder_joint'] == pytest.approx(0.0420 * 0.05)
    assert out['elbow_joint'] - start['elbow_joint'] == pytest.approx(-0.0355 * 0.05)
    assert out['base_joint'] == pytest.approx(0.5 * 0.5 * 0.05)


def test_never_leaves_limits():
    q = {'base_joint': 0.0, 'shoulder_joint': -0.5, 'elbow_joint': 1.0}
    for _ in range(100000):
        q = step_targets(q, {'shoulder_joint': 1.0, 'elbow_joint': 1.0, 'base_joint': 1.0}, SPECS, 0.05, margin=0.01)
    assert q['shoulder_joint'] == pytest.approx(SPECS['shoulder_joint'].upper - 0.01)
    assert q['elbow_joint'] == pytest.approx(SPECS['elbow_joint'].upper - 0.01)
    assert q['base_joint'] > 2 * math.pi            # continuous joint keeps turning (slip ring)


def test_clamp_and_home_duration():
    sh = SPECS['shoulder_joint']
    assert clamp(0.0, sh) == sh.upper
    assert clamp(-2.0, sh) == sh.lower
    cur = {'base_joint': 0.0, 'shoulder_joint': -0.5236, 'elbow_joint': 1.5708}
    home = {'base_joint': 0.0, 'shoulder_joint': -0.24, 'elbow_joint': 2.29}
    assert move_duration(cur, home, SPECS) == pytest.approx(1.2 * (2.29 - 1.5708) / 0.0355)


def test_home_pose_in_config_is_inside_limits():
    import yaml
    cfg = yaml.safe_load((HERE.parent / 'config' / 'joystick.yaml').read_text())
    home = cfg['joy_teleop']['ros__parameters']['home_position']
    for name, value in zip(JOINTS, home):
        spec = SPECS[name]
        assert spec.continuous or spec.lower < value < spec.upper
