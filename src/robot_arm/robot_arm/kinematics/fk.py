#!/usr/bin/env python3
"""
Forward Kinematics for the Universo Autonomous Robotic Arm.

Autonomous joints:
    1. 360   - Base rotation
    2. l1End - Shoulder rotation
    3. L2    - Elbow rotation

The manually operated gripper fingers are excluded.

Coordinate frame:
    X = Forward
    Y = Left
    Z = Up

Joint order:
    q = [base, shoulder, elbow]

Units:
    Angles: radians
    Lengths: metres

Geometry:
    Based on Robot_Arm_Dimensions_and_Limits.docx
    and the 30 October 2026 CAD/URDF export.

Arm tip:
    Centre of the gripper mounting face.
"""

import numpy as np


# ============================================================
# 1. ROBOT GEOMETRY
# ============================================================

# Shoulder axis relative to base yaw axis
A_SHOULDER = 0.10317653
H_SHOULDER = 0.06449970

# Link lengths
L1 = 0.54961413
L2 = 0.47664959

# Sideways offset of arm centre plane
Y_OFF = -0.00199868

# Reference geometry calibration
E0 = 1.37351263
DELTA = -2.32805707
EPS = -0.09242882


# ============================================================
# 2. JOINT CONFIGURATION
# ============================================================

JOINT_NAMES = ("360", "l1End", "L2")

YAW_CONTINUOUS = False

# Software limits from the 30 October URDF.
# Keep these limits unless deliberately changed and validated.

JOINT_LIMITS = np.array([
    [-np.pi, np.pi],
    [-1.22173, 0.0],
    [-2.09621, 0.0]
], dtype=float)

LIMIT_TOLERANCE_RAD = 1e-6


# ============================================================
# 3. REFERENCE POSE
# ============================================================

REFERENCE_JOINT_ANGLES = np.array([
    0.0,
    0.0,
    0.0
])

# Documented arm-tip position at the reference pose.
REFERENCE_POSITION = np.array([
    0.48640,
    -0.00200,
    0.21448
])


# ============================================================
# 4. ROTATION MATRICES
# ============================================================

def rotx(angle):
    """Rotation matrix about the X-axis."""

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c]
    ])


def roty(angle):
    """Rotation matrix about the Y-axis."""

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
    ])


def rotz(angle):
    """Rotation matrix about the Z-axis."""

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0]
    ])


# ============================================================
# 5. HOMOGENEOUS TRANSFORMATIONS
# ============================================================

def make_T(R=None, p=None):
    """Construct a 4x4 homogeneous transformation."""

    T = np.eye(4)

    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=float)

    if p is not None:
        T[:3, 3] = np.asarray(p, dtype=float)

    return T


def trans(x, y, z):
    """Pure translation."""

    return make_T(p=[x, y, z])


def T_rotx(angle):
    """Homogeneous rotation about X."""

    return make_T(R=rotx(angle))


def T_roty(angle):
    """Homogeneous rotation about Y."""

    return make_T(R=roty(angle))


def T_rotz(angle):
    """Homogeneous rotation about Z."""

    return make_T(R=rotz(angle))


# ============================================================
# 6. INPUT VALIDATION
# ============================================================

def validate_joint_angles(q, check_limits=False):
    """
    Validate [base, shoulder, elbow].

    Returns:
        Validated NumPy array.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (3,):
        raise ValueError(
            "Joint vector must contain exactly 3 values: "
            "[base, shoulder, elbow]."
        )

    if not np.all(np.isfinite(q)):
        raise ValueError("Joint angles must be finite.")

    if check_limits and not within_joint_limits(q):
        raise ValueError(
            "Joint angles are outside the configured "
            "software joint limits."
        )

    return q


def within_joint_limits(q):
    """Return True if all joints are within software limits."""

    q = validate_joint_angles(q)

    return bool(
        np.all(q >= JOINT_LIMITS[:, 0] - LIMIT_TOLERANCE_RAD)
        and
        np.all(q <= JOINT_LIMITS[:, 1] + LIMIT_TOLERANCE_RAD)
    )


def clamp_to_limits(q):
    """Return joint angles clipped to software limits."""

    q = validate_joint_angles(q)

    return np.clip(
        q,
        JOINT_LIMITS[:, 0],
        JOINT_LIMITS[:, 1]
    )


# ============================================================
# 7. FORWARD KINEMATICS
# ============================================================

def frames(q):
    """
    Calculate transformations of the arm.

    Returns:
        [T1, T2, T3, T4]

        T1: Base rotation
        T2: Shoulder frame
        T3: Elbow frame
        T4: Arm tip / gripper mounting face
    """

    q = validate_joint_angles(q)

    # Joint angle definitions
    theta = -q[0]

    e1 = E0 + q[1]

    e2 = e1 + DELTA - q[2]

    # Base rotation
    T1 = T_rotz(theta)

    # Shoulder transformation
    T2 = (
        T1
        @ trans(
            A_SHOULDER,
            Y_OFF,
            H_SHOULDER
        )
        @ T_roty(-e1)
    )

    # Elbow transformation
    T3 = (
        T2
        @ trans(L1, 0.0, 0.0)
        @ T_roty(-(e2 - e1))
    )

    # Arm-tip transformation
    T4 = (
        T3
        @ trans(L2, 0.0, 0.0)
        @ T_roty(-EPS)
        @ T_rotx(-np.pi / 2.0)
    )

    return [T1, T2, T3, T4]


# ============================================================
# 8. END-EFFECTOR TRANSFORMATION
# ============================================================

def fk(q):
    """
    Calculate the complete arm-tip transformation.

    Returns:
        4x4 homogeneous transformation matrix.
    """

    return frames(q)[-1]


# ============================================================
# 9. END-EFFECTOR POSITION
# ============================================================

def fk_position(q):
    """
    Calculate arm-tip Cartesian position.

    Returns:
        [x, y, z] in metres.
    """

    T = fk(q)

    return T[:3, 3].copy()


# ============================================================
# 10. END-EFFECTOR POSE
# ============================================================

def fk_pose(q):
    """
    Calculate arm-tip position and orientation.

    Returns:
        position: 3-element vector in metres
        rotation: 3x3 rotation matrix
    """

    T = fk(q)

    return (
        T[:3, 3].copy(),
        T[:3, :3].copy()
    )


# ============================================================
# 11. JOINT POSITIONS
# ============================================================

def joint_positions(q):
    """
    Return important arm positions.

    Order:
        1. Base rotation axis
        2. Shoulder axis
        3. Elbow axis
        4. Arm tip

    Units: metres.
    """

    T1, T2, T3, T4 = frames(q)

    base_axis = np.array([0.0, 0.0, 0.0])

    shoulder_axis = T2[:3, 3].copy()

    elbow_axis = T3[:3, 3].copy()

    arm_tip = T4[:3, 3].copy()

    return np.array([
        base_axis,
        shoulder_axis,
        elbow_axis,
        arm_tip
    ])


# ============================================================
# 12. REFERENCE VALIDATION
# ============================================================

def validate_reference_pose(tolerance_m=1e-5):
    """
    Compare zero-pose FK against the documented CAD reference.

    Note:
        The zero pose is a geometric reference and is outside
        the permitted shoulder joint range.
    """

    calculated = fk_position(REFERENCE_JOINT_ANGLES)

    error = calculated - REFERENCE_POSITION

    error_norm = np.linalg.norm(error)

    return {
        "calculated_position": calculated,
        "expected_position": REFERENCE_POSITION.copy(),
        "error": error,
        "error_norm": float(error_norm),
        "passed": bool(error_norm <= tolerance_m)
    }


# ============================================================
# 13. TEST
# ============================================================

if __name__ == "__main__":

    np.set_printoptions(
        precision=6,
        suppress=True
    )

    q = REFERENCE_JOINT_ANGLES.copy()

    print("\n========== FORWARD KINEMATICS ==========")

    print("\nJoint order:")
    print(JOINT_NAMES)

    print("\nJoint angles (radians):")
    print(q)

    print("\nJoint angles (degrees):")
    print(np.degrees(q))

    print("\nJoint limits:")
    print(JOINT_LIMITS)

    print("\nWithin joint limits:")
    print(within_joint_limits(q))

    print("\nEnd-effector transformation:")
    print(fk(q))

    print("\nArm-tip position (metres):")
    print(fk_position(q))

    print("\nArm-tip position (millimetres):")
    print(fk_position(q) * 1000.0)

    print("\nJoint positions (metres):")
    print(joint_positions(q))

    print("\n========== REFERENCE VALIDATION ==========")

    result = validate_reference_pose()

    print("\nExpected position (m):")
    print(result["expected_position"])

    print("\nCalculated position (m):")
    print(result["calculated_position"])

    print("\nPosition error (m):")
    print(result["error"])

    print("\nError magnitude (m):")
    print(result["error_norm"])

    print("\nReference validation:")
    print("PASS" if result["passed"] else "FAIL")

def fk_planar(q):
    """
    Return the arm-tip position in the planar radial-vertical space.
    Output: [radial_distance, z] in metres.
    """
    position = fk_position(q)
    radial_distance = np.hypot(position[0], position[1])
    return np.array([radial_distance, position[2]])


def fk_joint_positions(q):
    """Compatibility function returning base, shoulder, elbow and tip."""
    return joint_positions(q)
