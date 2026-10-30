"""Actuator pin-to-pin length (300 mm retracted / 500 mm extended) vs the URDF joint limits.

URDF of 30 Oct: the joint limits are the Onshape mate limits. Pin centres are the CAD M6 pin centre lines.
1. Shoulder geometry check: over the whole shoulder range the actuator stays within 300..500 mm.
2. Elbow geometry check: SKIPPED until the forearm pin is known (needs the STL meshes).
3. Code check: actuator_conversion.actuator_lengths() matches the geometry (skipped until it exists
   and the forearm pin is known).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import urdf_reference as ref  # noqa: E402

J = ref.load_joints()
SH = ref.limits(J, 'shoulder_joint')
EL = ref.limits(J, 'elbow_joint')
RETRACTED_MM, EXTENDED_MM = 300.0, 500.0

# Pin centres in the arm plane [x, z] (base_link, metres) with shoulder = elbow = 0
# (upper arm vertical, arm straight). None = not known yet.
SHOULDER_AXIS = np.array([0.1031763, 0.0645001])
ELBOW_AXIS = np.array([0.1031763, 0.6141142])
BASE_PIN = np.array([-0.1343009, 0.1464861])          # turntable riser, shoulder actuator lower pin (fixed)
UPPER_PIN_SHOULDER = np.array([0.0666603, 0.3285619])  # upper arm, pin nearer the shoulder (264.06 mm along)
UPPER_PIN_ELBOW = np.array([0.0659781, 0.3585541])     # upper arm, pin further out (294.05 mm along)
FOREARM_PIN = None                                     # forearm, elbow actuator upper pin - needs meshes

ELBOW_KNOWN = FOREARM_PIN is not None
needs_forearm_pin = pytest.mark.skipif(not ELBOW_KNOWN, reason='forearm actuator pin not known yet (needs STL meshes)')


def _rot(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def shoulder_mm(q_shoulder):
    # shoulder_joint axis is -y: +q tilts the arm backwards = counter-clockwise in the (x, z) plane
    moving = SHOULDER_AXIS + _rot(UPPER_PIN_SHOULDER - SHOULDER_AXIS, q_shoulder)
    return float(np.linalg.norm(moving - BASE_PIN)) * 1000


def elbow_mm(q_shoulder, q_elbow):
    # both pins move with the upper arm; elbow_joint axis is +y: +q folds the forearm = clockwise in (x, z)
    lower = SHOULDER_AXIS + _rot(UPPER_PIN_ELBOW - SHOULDER_AXIS, q_shoulder)
    elbow = SHOULDER_AXIS + _rot(ELBOW_AXIS - SHOULDER_AXIS, q_shoulder)
    upper = elbow + _rot(FOREARM_PIN - ELBOW_AXIS, q_shoulder - q_elbow)
    return float(np.linalg.norm(upper - lower)) * 1000


def test_axes_match_urdf():
    assert ref.link_pose(J, {}, 'upper_arm_link')[[0, 2], 3] == pytest.approx(SHOULDER_AXIS, abs=1e-6)
    assert ref.link_pose(J, {}, 'forearm_link')[[0, 2], 3] == pytest.approx(ELBOW_AXIS, abs=1e-6)
    assert ref.tip(J, 0.0, 0.0, 0.0)[0] == pytest.approx(SHOULDER_AXIS[0], abs=1e-6)   # straight arm points up


def test_shoulder_actuator_within_stroke_over_urdf_range():
    lengths = [shoulder_mm(q) for q in np.linspace(SH[0], SH[1], 2000)]
    assert min(lengths) >= RETRACTED_MM and max(lengths) <= EXTENDED_MM
    assert shoulder_mm(SH[1]) == pytest.approx(313.24, abs=0.01)     # upper limit (arm highest)
    assert shoulder_mm(SH[0]) == pytest.approx(493.02, abs=0.01)     # lower limit (arm lowest)


@needs_forearm_pin
def test_elbow_actuator_within_stroke_over_urdf_range():
    for qs in np.linspace(SH[0], SH[1], 25):
        for qe in np.linspace(EL[0], EL[1], 200):
            assert RETRACTED_MM <= elbow_mm(qs, qe) <= EXTENDED_MM, (qs, qe)


@needs_forearm_pin
def test_actuator_lengths_function_matches_geometry():
    try:
        from robot_arm.kinematics.actuator_conversion import actuator_lengths
    except ImportError as exc:
        pytest.skip(f'actuator_lengths() not available yet: {exc}')
    for q1, q2 in ((SH[1], EL[0]), (SH[1], EL[1]), (-0.6, 1.2)):
        result = actuator_lengths(q1, q2)
        if isinstance(result, dict):
            s, e = result['shoulder_actuator_mm'], result['elbow_actuator_mm']
        else:
            s, e = result[0], result[1]
        assert s == pytest.approx(shoulder_mm(q1), abs=0.1)
        assert e == pytest.approx(elbow_mm(q1, q2), abs=0.1)
