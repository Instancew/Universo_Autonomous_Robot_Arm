"""The URDF <-> export joint conversion must make kinematics/fk.py and ik.py agree with the URDF.

fk.py / ik.py are loaded from their files (not through robot_arm.kinematics), so these tests do not
depend on kinematics/__init__.py.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import urdf_reference as ref  # noqa: E402
from robot_arm.autonomy.joint_convention import (  # noqa: E402
    export_to_urdf, export_to_urdf_dict, nearest_base_angle, urdf_to_export)

J = ref.load_joints()
SH = ref.limits(J, 'shoulder_joint')
EL = ref.limits(J, 'elbow_joint')
KIN = HERE.parent / 'robot_arm' / 'kinematics'
TOL_M = 1e-5          # 0.01 mm (measured: about 0.003 mm)


def _load(name):
    spec = importlib.util.spec_from_file_location(f'{name}_conv_test', KIN / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


FK = _load('fk')


def _random_urdf_q(n, seed=0):
    rng = np.random.default_rng(seed)
    return [(rng.uniform(-math.pi, math.pi), rng.uniform(*SH), rng.uniform(*EL)) for _ in range(n)]


def test_round_trip():
    for q in _random_urdf_q(50):
        assert export_to_urdf(*urdf_to_export(*q)) == pytest.approx(q, abs=1e-12)


def test_export_limits_match_fk_limits():
    lo = urdf_to_export(0.0, SH[0], EL[0])
    hi = urdf_to_export(0.0, SH[1], EL[1])
    assert (lo[1], hi[1]) == pytest.approx((FK.JOINT_LIMITS[1][0], FK.JOINT_LIMITS[1][1]), abs=1e-5)
    assert (lo[2], hi[2]) == pytest.approx((FK.JOINT_LIMITS[2][0], FK.JOINT_LIMITS[2][1]), abs=1e-5)


def test_fk_with_converted_angles_matches_urdf():
    for q in _random_urdf_q(300, seed=1):
        tip_fk = FK.fk_position(np.array(urdf_to_export(*q)))
        assert np.linalg.norm(tip_fk - ref.tip(J, *q)) < TOL_M


def test_unconverted_angles_do_not_match_urdf():
    # documents why the conversion is needed: URDF numbers fed to fk.py give a different tip
    q = (0.0, -0.5236, 1.5708)
    assert np.linalg.norm(FK.fk_position(np.array(q)) - ref.tip(J, *q)) > 0.1


def test_ik_output_converted_back_reaches_the_target():
    sys.path.insert(0, str(KIN))
    try:
        import ik as IK
        solutions = IK.ik_all
    except (ImportError, AttributeError) as exc:
        pytest.skip(f'kinematics/ik.py does not provide ik_all: {exc}')
    finally:
        sys.path.remove(str(KIN))
    for q in _random_urdf_q(200, seed=2):
        target = ref.tip(J, *q)
        sols = solutions(target)
        assert sols, 'no IK solution for a reachable target'
        urdf_q = export_to_urdf(*sols[0])
        assert np.linalg.norm(ref.tip(J, *urdf_q) - target) < TOL_M
        assert urdf_q[1:] == pytest.approx(q[1:], abs=1e-4)


def test_dict_has_urdf_joint_names():
    assert list(export_to_urdf_dict((0.1, -1.0, -1.0))) == ['base_joint', 'shoulder_joint', 'elbow_joint']


@pytest.mark.parametrize('target,current,expected', [
    (3.0, -3.0, 3.0 - 2 * math.pi),      # shorter to go through +-pi
    (0.5, 0.0, 0.5),
    (-3.0, 3.0, -3.0 + 2 * math.pi),
    (1.0, 7.0, 1.0 + 2 * math.pi),       # base already turned more than a full turn
])
def test_nearest_base_angle(target, current, expected):
    assert nearest_base_angle(target, current) == pytest.approx(expected)
