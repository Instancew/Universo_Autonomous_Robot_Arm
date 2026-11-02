
"""
jacobian.py - Position Jacobian for the 3-DOF robotic arm.

Joint order:
    q[0] = base rotation
    q[1] = shoulder rotation
    q[2] = elbow rotation

The Jacobian maps joint velocities to Cartesian tip velocity:

    p_dot = J(q) @ q_dot

The Jacobian is calculated using central finite differences
of the calibrated forward kinematics model in fk.py.

Reference:
    Robot_Arm_Dimensions_and_Limits.docx
    Calibrated URDF geometry through fk.py
"""

import numpy as np

try:
    from .fk import fk_position
except ImportError:
    from fk import fk_position


# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

DEFAULT_EPSILON = 1e-6
DEFAULT_DAMPING = 0.02


# ---------------------------------------------------------
# Input validation
# ---------------------------------------------------------

def _validate_joint_vector(q):
    """Validate a three-joint configuration."""

    q = np.asarray(q, dtype=float)

    if q.shape != (3,):
        raise ValueError(
            "Joint configuration must contain "
            "[q_base, q_shoulder, q_elbow]."
        )

    if not np.all(np.isfinite(q)):
        raise ValueError("Joint angles must be finite.")

    return q


# ---------------------------------------------------------
# Position Jacobian
# ---------------------------------------------------------

def jacobian_linear(q, epsilon=DEFAULT_EPSILON):
    """
    Calculate the 3x3 Cartesian position Jacobian.

    Args:
        q:
            Joint angles [base, shoulder, elbow] in radians.

        epsilon:
            Small angular perturbation in radians.

    Returns:
        J:
            3x3 Jacobian matrix.

    Units:
        Metres per radian.
    """

    q = _validate_joint_vector(q)

    if not np.isfinite(epsilon) or epsilon <= 0:
        raise ValueError("epsilon must be positive and finite.")

    J = np.zeros((3, 3), dtype=float)

    for i in range(3):

        q_plus = q.copy()
        q_minus = q.copy()

        q_plus[i] += epsilon
        q_minus[i] -= epsilon

        p_plus = fk_position(q_plus)
        p_minus = fk_position(q_minus)

        J[:, i] = (
            p_plus - p_minus
        ) / (2.0 * epsilon)

    return J


# ---------------------------------------------------------
# Damped pseudoinverse
# ---------------------------------------------------------

def damped_pinv(J, damping=DEFAULT_DAMPING):
    """
    Calculate the damped Moore-Penrose pseudoinverse.

    Formula:

        J_damped_pinv =
            J.T @ inv(J @ J.T + damping^2 * I)

    Damping improves numerical stability near singular
    configurations.

    Args:
        J:
            Jacobian matrix.

        damping:
            Positive damping factor.

    Returns:
        Damped pseudoinverse of J.
    """

    J = np.asarray(J, dtype=float)

    if J.ndim != 2:
        raise ValueError("J must be a two-dimensional matrix.")

    if not np.all(np.isfinite(J)):
        raise ValueError("J must contain finite values.")

    if not np.isfinite(damping) or damping <= 0:
        raise ValueError("damping must be positive and finite.")

    identity = np.eye(J.shape[0])

    return (
        J.T
        @ np.linalg.solve(
            J @ J.T + damping ** 2 * identity,
            identity,
        )
    )


# ---------------------------------------------------------
# Cartesian velocity calculation
# ---------------------------------------------------------

def cartesian_velocity(q, joint_velocities):
    """
    Calculate Cartesian tip velocity.

    Args:
        q:
            Current joint angles [rad].

        joint_velocities:
            Joint velocities [rad/s].

    Returns:
        Cartesian velocity [m/s].
    """

    q = _validate_joint_vector(q)

    joint_velocities = np.asarray(
        joint_velocities,
        dtype=float,
    )

    if joint_velocities.shape != (3,):
        raise ValueError(
            "Joint velocities must contain three values."
        )

    if not np.all(np.isfinite(joint_velocities)):
        raise ValueError("Joint velocities must be finite.")

    return jacobian_linear(q) @ joint_velocities


# ---------------------------------------------------------
# Joint velocity calculation
# ---------------------------------------------------------

def joint_velocity_from_cartesian(
    q,
    cartesian_velocities,
    damping=DEFAULT_DAMPING,
):
    """
    Calculate joint velocities from a desired Cartesian velocity.

    Uses damped least squares.

    Args:
        q:
            Current joint configuration [rad].

        cartesian_velocities:
            Desired Cartesian velocity [m/s].

        damping:
            Damping factor.

    Returns:
        Joint velocities [rad/s].
    """

    q = _validate_joint_vector(q)

    cartesian_velocities = np.asarray(
        cartesian_velocities,
        dtype=float,
    )

    if cartesian_velocities.shape != (3,):
        raise ValueError(
            "Cartesian velocity must contain [vx, vy, vz]."
        )

    if not np.all(np.isfinite(cartesian_velocities)):
        raise ValueError(
            "Cartesian velocities must be finite."
        )

    J = jacobian_linear(q)

    return damped_pinv(J, damping) @ cartesian_velocities


# ---------------------------------------------------------
# Self-test
# ---------------------------------------------------------

if __name__ == "__main__":

    np.set_printoptions(
        precision=6,
        suppress=True,
    )

    print("\n========== JACOBIAN TEST ==========\n")

    q = np.array([
        0.6,
        -0.4,
        -1.1,
    ])

    print("Joint angles [rad]:", q)

    J = jacobian_linear(q)

    print("\nPosition Jacobian:")
    print(J)

    print("\nJacobian shape:", J.shape)

    # Test pseudoinverse
    J_pinv = damped_pinv(J)

    print("\nDamped pseudoinverse:")
    print(J_pinv)

    # Test Cartesian velocity
    q_dot = np.array([
        0.1,
        0.2,
        0.15,
    ])

    velocity = cartesian_velocity(q, q_dot)

    print("\nJoint velocities [rad/s]:")
    print(q_dot)

    print("\nCartesian tip velocity [m/s]:")
    print(velocity)

    # Test inverse velocity calculation
    recovered_q_dot = joint_velocity_from_cartesian(
        q,
        velocity,
    )

    reconstructed_velocity = J @ recovered_q_dot

    velocity_error = np.linalg.norm(
        reconstructed_velocity - velocity
    )

    print("\nRecovered joint velocities [rad/s]:")
    print(recovered_q_dot)

    print("\nReconstructed Cartesian velocity [m/s]:")
    print(reconstructed_velocity)

    print("\nCartesian velocity reconstruction error:")
    print(velocity_error)

    # Test results
    checks = [
        (
            "Jacobian shape",
            J.shape == (3, 3),
        ),
        (
            "Finite Jacobian",
            np.all(np.isfinite(J)),
        ),
        (
            "Pseudoinverse shape",
            J_pinv.shape == (3, 3),
        ),
        (
            "Finite pseudoinverse",
            np.all(np.isfinite(J_pinv)),
        ),
        (
            "Velocity calculation",
            np.all(np.isfinite(velocity)),
        ),
        (
            "Velocity reconstruction",
            velocity_error < 5e-4,
        ),
    ]

    print("\n========== FINAL RESULT ==========\n")

    passed = 0
    failed = 0

    for name, result in checks:

        if result:
            print(f"PASS | {name}")
            passed += 1
        else:
            print(f"FAIL | {name}")
            failed += 1

    print("\nPassed:", passed)
    print("Failed:", failed)

    if failed == 0:
        print("RESULT: ALL TESTS PASSED")
    else:
        print("RESULT: VALIDATION NEEDS ATTENTION")
