"""Convert joint angles between the two conventions used in this workspace.

URDF convention (robot_arm.urdf, MoveIt, ros2_control, teleop, joints.yaml, calibration):
    base_joint, shoulder_joint, elbow_joint        [rad]

Onshape-export convention (kinematics/fk.py, ik.py, jacobian.py):
    q = [360, l1End, L2]                           [rad]

Relations (from description/URDF_CHANGES.md and the URDF joint limits):
    360   = -base_joint
    l1End =  shoulder_joint + 0.197281      (11.303 deg)
    L2    =  elbow_joint    - 2.328057      (133.388 deg)

Checked against the URDF: fk.py with converted angles matches the URDF arm_tip to about 0.003 mm
(the offsets are the 6-digit URDF limits, fk.py uses slightly more digits). See docs/conventions.md.

Pure Python (no numpy, no ROS), so it can be unit-tested anywhere.
"""
import math
from typing import Dict, Tuple

SHOULDER_OFFSET_RAD = 0.197281   # l1End = shoulder_joint + SHOULDER_OFFSET_RAD
ELBOW_OFFSET_RAD = 2.328057      # L2    = elbow_joint    - ELBOW_OFFSET_RAD

URDF_JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint')


def urdf_to_export(base_joint: float, shoulder_joint: float, elbow_joint: float) -> Tuple[float, float, float]:
    """URDF joint values -> [360, l1End, L2] as used by kinematics/fk.py and ik.py."""
    return (-base_joint,
            shoulder_joint + SHOULDER_OFFSET_RAD,
            elbow_joint - ELBOW_OFFSET_RAD)


def export_to_urdf(q0: float, l1_end: float, l2: float) -> Tuple[float, float, float]:
    """[360, l1End, L2] (fk.py / ik.py) -> URDF (base_joint, shoulder_joint, elbow_joint)."""
    return (-q0,
            l1_end - SHOULDER_OFFSET_RAD,
            l2 + ELBOW_OFFSET_RAD)


def export_to_urdf_dict(q) -> Dict[str, float]:
    """Same as export_to_urdf, returned as {joint name: value} ready for a trajectory point."""
    return dict(zip(URDF_JOINTS, export_to_urdf(*q)))


def nearest_base_angle(target_wrapped: float, current: float) -> float:
    """The base is continuous (slip ring), but ik.py returns it wrapped to [-pi, pi).

    Return the angle equal to target_wrapped (mod 2*pi) that is closest to `current`,
    so the base takes the shortest way round instead of unwinding.
    """
    two_pi = 2.0 * math.pi
    return target_wrapped + two_pi * round((current - target_wrapped) / two_pi)
