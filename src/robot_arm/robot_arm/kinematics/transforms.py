
"""
3D transformation utilities for the robotic arm.

Conventions:
- Angles are in radians.
- Distances are in metres.
- Rotation matrices are 3x3.
- Homogeneous transformation matrices are 4x4.
- Uses the standard Denavit-Hartenberg convention.

These are general mathematical utilities.
Arm-specific geometry is defined in fk.py.
"""

import numpy as np


# ---------------------------------------------------------
# INPUT VALIDATION
# ---------------------------------------------------------

def _validate_vector3(vector, name="vector"):
    """Validate and return a 3-element vector."""
    vector = np.asarray(vector, dtype=float)

    if vector.shape != (3,):
        raise ValueError(f"{name} must contain exactly 3 elements.")

    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain finite values.")

    return vector


def _validate_rotation_matrix(rotation):
    """Validate a 3x3 rotation matrix."""
    rotation = np.asarray(rotation, dtype=float)

    if rotation.shape != (3, 3):
        raise ValueError("Rotation matrix must have shape (3, 3).")

    if not np.all(np.isfinite(rotation)):
        raise ValueError("Rotation matrix must contain finite values.")

    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("Rotation matrix must be orthogonal.")

    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError("Rotation matrix determinant must be +1.")

    return rotation


def _validate_transform(transform):
    """Validate a 4x4 homogeneous transformation matrix."""
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError("Transformation matrix must have shape (4, 4).")

    if not np.all(np.isfinite(transform)):
        raise ValueError("Transformation matrix must contain finite values.")

    if not np.allclose(
        transform[3],
        [0.0, 0.0, 0.0, 1.0],
        atol=1e-6
    ):
        raise ValueError("Invalid homogeneous transformation bottom row.")

    _validate_rotation_matrix(transform[:3, :3])

    return transform


# ---------------------------------------------------------
# ROTATION MATRICES
# ---------------------------------------------------------

def rotation_matrix_x(angle):
    """Return rotation matrix about the X-axis."""
    angle = float(angle)

    if not np.isfinite(angle):
        raise ValueError("Angle must be finite.")

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c]
    ])


def rotation_matrix_y(angle):
    """Return rotation matrix about the Y-axis."""
    angle = float(angle)

    if not np.isfinite(angle):
        raise ValueError("Angle must be finite.")

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
    ])


def rotation_matrix_z(angle):
    """Return rotation matrix about the Z-axis."""
    angle = float(angle)

    if not np.isfinite(angle):
        raise ValueError("Angle must be finite.")

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0]
    ])


# ---------------------------------------------------------
# TRANSLATION MATRIX
# ---------------------------------------------------------

def translation_matrix(x, y, z):
    """Return a 4x4 pure translation matrix."""
    translation = _validate_vector3([x, y, z], "translation")

    transform = np.eye(4)
    transform[:3, 3] = translation

    return transform


# ---------------------------------------------------------
# HOMOGENEOUS TRANSFORMATION
# ---------------------------------------------------------

def homogeneous_transform(rotation, translation):
    """
    Combine rotation and translation into a 4x4 matrix.

    Parameters:
        rotation: 3x3 rotation matrix
        translation: 3-element translation vector

    Returns:
        4x4 homogeneous transformation matrix
    """
    rotation = _validate_rotation_matrix(rotation)
    translation = _validate_vector3(translation, "translation")

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    return transform


# ---------------------------------------------------------
# DENAVIT-HARTENBERG TRANSFORMATION
# ---------------------------------------------------------

def dh_transform(a, alpha, d, theta):
    """
    Return the standard Denavit-Hartenberg transformation.

    Parameters:
        a: Link length (metres)
        alpha: Link twist (radians)
        d: Link offset (metres)
        theta: Joint angle (radians)

    Returns:
        4x4 homogeneous transformation matrix
    """
    a = float(a)
    alpha = float(alpha)
    d = float(d)
    theta = float(theta)

    if not np.all(np.isfinite([a, alpha, d, theta])):
        raise ValueError("DH parameters must be finite.")

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0]
    ])


# ---------------------------------------------------------
# INVERSE TRANSFORMATION
# ---------------------------------------------------------

def inverse_transform(transform):
    """
    Calculate the inverse of a homogeneous transformation.

    For T = [R  p]
            [0  1]

    T_inv = [R.T  -R.T @ p]
            [ 0       1   ]
    """
    transform = _validate_transform(transform)

    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation

    return inverse


# ---------------------------------------------------------
# SELF-TEST
# ---------------------------------------------------------

def run_tests():
    """Run basic mathematical validation tests."""

    tests = []

    Rx = rotation_matrix_x(np.pi / 2)
    Ry = rotation_matrix_y(np.pi / 2)
    Rz = rotation_matrix_z(np.pi / 2)

    tests.append((
        "Rotation matrix X",
        Rx.shape == (3, 3)
        and np.allclose(Rx.T @ Rx, np.eye(3))
    ))

    tests.append((
        "Rotation matrix Y",
        Ry.shape == (3, 3)
        and np.allclose(Ry.T @ Ry, np.eye(3))
    ))

    tests.append((
        "Rotation matrix Z",
        Rz.shape == (3, 3)
        and np.allclose(Rz.T @ Rz, np.eye(3))
    ))

    T_translation = translation_matrix(1.0, 2.0, 3.0)

    tests.append((
        "Translation matrix",
        T_translation.shape == (4, 4)
        and np.allclose(
            T_translation[:3, 3],
            [1.0, 2.0, 3.0]
        )
    ))

    T = homogeneous_transform(
        rotation_matrix_z(0.5),
        [0.2, 0.3, 0.4]
    )

    tests.append((
        "Homogeneous transformation",
        T.shape == (4, 4)
        and np.allclose(T[3], [0.0, 0.0, 0.0, 1.0])
    ))

    T_dh = dh_transform(
        0.5,
        np.pi / 2,
        0.2,
        np.pi / 4
    )

    tests.append((
        "DH transformation",
        T_dh.shape == (4, 4)
        and np.allclose(T_dh[3], [0.0, 0.0, 0.0, 1.0])
    ))

    T_inverse = inverse_transform(T)

    tests.append((
        "Inverse transformation",
        np.allclose(T @ T_inverse, np.eye(4), atol=1e-9)
    ))

    print("\n========== TRANSFORMS TEST ==========\n")

    passed = 0
    failed = 0

    for name, result in tests:
        if result:
            print(f"PASS | {name}")
            passed += 1
        else:
            print(f"FAIL | {name}")
            failed += 1

    print("\n========== FINAL RESULT ==========\n")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")

    if failed == 0:
        print("RESULT: ALL TESTS PASSED")
    else:
        print("RESULT: VALIDATION NEEDS ATTENTION")


if __name__ == "__main__":
    run_tests()
