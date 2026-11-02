"""
ik.py - Inverse kinematics for the 3-DOF robotic arm.

The arm has three joints:
    q[0] = base rotation
    q[1] = shoulder rotation
    q[2] = elbow rotation

The commanded target is the arm tip POSITION [x, y, z].
Tool orientation is determined by the joint configuration.

Analytical IK:
    Calculates possible joint configurations using the calibrated
    geometric model defined in fk.py.

Numerical IK:
    Uses damped least squares and a numerical position Jacobian.
    This avoids depending on the unfinished jacobian.py implementation.
"""

import numpy as np

try:
    from .fk import (
        A_SHOULDER,
        H_SHOULDER,
        L1,
        L2,
        Y_OFF,
        E0,
        DELTA,
        JOINT_LIMITS,
        fk_position,
        within_joint_limits,
        clamp_to_limits,
    )
except ImportError:
    from fk import (
        A_SHOULDER,
        H_SHOULDER,
        L1,
        L2,
        Y_OFF,
        E0,
        DELTA,
        JOINT_LIMITS,
        fk_position,
        within_joint_limits,
        clamp_to_limits,
    )


# ---------------------------------------------------------
# Utility functions
# ---------------------------------------------------------

def wrap_to_pi(angle):
    """Wrap angles to the interval [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def _validate_position(p):
    """Validate a Cartesian target position."""
    p = np.asarray(p, dtype=float)

    if p.shape != (3,):
        raise ValueError("Target position must contain exactly [x, y, z].")

    if not np.all(np.isfinite(p)):
        raise ValueError("Target position must contain finite values.")

    return p


# ---------------------------------------------------------
# Base yaw candidates
# ---------------------------------------------------------

def _yaw_candidates(x, y):
    """
    Calculate possible base yaw angles.

    The arm plane is offset from the base yaw axis by Y_OFF.

    Returns:
        List of (theta, planar_x) pairs.
    """

    r = np.hypot(x, y)

    if r < abs(Y_OFF):
        return []

    s = np.arcsin(np.clip(Y_OFF / r, -1.0, 1.0))
    phi = np.arctan2(y, x)

    xp = np.sqrt(max(r * r - Y_OFF ** 2, 0.0))

    return [
        (phi - s, xp),
        (phi - np.pi + s, -xp),
    ]


# ---------------------------------------------------------
# Analytical inverse kinematics
# ---------------------------------------------------------

def ik_all(p, limits=True, tol=1e-9):
    """
    Calculate all analytical IK solutions.

    Args:
        p:
            Target position [x, y, z] in metres.

        limits:
            If True, reject solutions outside the software joint limits.

        tol:
            Numerical tolerance for reachability.

    Returns:
        List of solutions:
            [q_base, q_shoulder, q_elbow]

        Returns an empty list if no valid solution exists.
    """

    x, y, z = _validate_position(p)

    solutions = []

    for theta, xp in _yaw_candidates(x, y):

        dx = xp - A_SHOULDER
        dz = z - H_SHOULDER

        d2 = dx ** 2 + dz ** 2

        # Law of cosines
        c = (
            d2 - L1 ** 2 - L2 ** 2
        ) / (2.0 * L1 * L2)

        if c > 1.0 + tol or c < -1.0 - tol:
            continue

        c = np.clip(c, -1.0, 1.0)

        g = np.arccos(c)

        # Both elbow configurations
        gamma_candidates = (g, -g) if g > 1e-12 else (0.0,)

        for gamma in gamma_candidates:

            e1 = (
                np.arctan2(dz, dx)
                - np.arctan2(
                    L2 * np.sin(gamma),
                    L1 + L2 * np.cos(gamma),
                )
            )

            q = np.array([
                wrap_to_pi(-theta),
                wrap_to_pi(e1 - E0),
                wrap_to_pi(DELTA - gamma),
            ], dtype=float)

            if limits and not within_joint_limits(q):
                continue

            # Avoid duplicate solutions at singular configurations
            if not any(
                np.linalg.norm(wrap_to_pi(q - existing)) < 1e-8
                for existing in solutions
            ):
                solutions.append(q)

    return solutions


def ik(p, q_seed=None, limits=True):
    """
    Return the best analytical IK solution.

    If q_seed is supplied, return the solution requiring the
    smallest joint movement from the seed.

    Returns None if the target is unreachable.
    """

    solutions = ik_all(p, limits=limits)

    if not solutions:
        return None

    if q_seed is None:
        return solutions[0]

    q_seed = np.asarray(q_seed, dtype=float)

    if q_seed.shape != (3,):
        raise ValueError("q_seed must contain three joint angles.")

    differences = [
        np.abs(wrap_to_pi(solution - q_seed))
        for solution in solutions
    ]

    best_index = int(
        np.argmin([np.sum(diff) for diff in differences])
    )

    return solutions[best_index]


def is_reachable(p):
    """Return True if an IK solution exists within the joint limits."""
    return len(ik_all(p, limits=True)) > 0


# ---------------------------------------------------------
# Numerical position Jacobian
# ---------------------------------------------------------

def numerical_jacobian(q, epsilon=1e-6):
    """
    Calculate the 3x3 position Jacobian using finite differences.

    J[:, i] = change in tip position / change in joint angle i.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (3,):
        raise ValueError("Joint vector must contain three angles.")

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
# Damped least-squares inverse kinematics
# ---------------------------------------------------------

def ik_numeric(
    p,
    q_seed,
    max_iter=200,
    tol=1e-6,
    damping=0.02,
    max_step=0.2,
    respect_limits=True,
):
    """
    Numerical IK using damped least squares.

    Args:
        p:
            Target Cartesian position [x, y, z] in metres.

        q_seed:
            Initial joint configuration [base, shoulder, elbow].

        max_iter:
            Maximum number of iterations.

        tol:
            Cartesian position error tolerance in metres.

        damping:
            Damping factor for numerical stability.

        max_step:
            Maximum absolute joint change per iteration in radians.

        respect_limits:
            Enforce conservative software joint limits.

    Returns:
        (q, converged)
    """

    p = _validate_position(p)

    q = np.asarray(q_seed, dtype=float).copy()

    if q.shape != (3,):
        raise ValueError("q_seed must contain three joint angles.")

    if not np.all(np.isfinite(q)):
        raise ValueError("q_seed must contain finite values.")

    if max_iter <= 0 or tol <= 0 or damping <= 0 or max_step <= 0:
        raise ValueError("Solver parameters must be positive.")

    if respect_limits:
        q = clamp_to_limits(q)

    for _ in range(max_iter):

        current_position = fk_position(q)

        error = p - current_position

        if np.linalg.norm(error) < tol:
            return q, True

        J = numerical_jacobian(q)

        # Damped least-squares:
        # dq = J.T @ inv(J @ J.T + lambda^2 I) @ error

        identity = np.eye(3)

        dq = (
            J.T
            @ np.linalg.solve(
                J @ J.T + damping ** 2 * identity,
                error,
            )
        )

        largest_step = np.max(np.abs(dq))

        if largest_step > max_step:
            dq *= max_step / largest_step

        q += dq

        if respect_limits:
            q = clamp_to_limits(q)

    final_error = np.linalg.norm(p - fk_position(q))

    return q, bool(final_error < tol)


# ---------------------------------------------------------
# Self-test
# ---------------------------------------------------------

if __name__ == "__main__":

    np.set_printoptions(precision=6, suppress=True)

    print("\n========== INVERSE KINEMATICS TEST ==========\n")

    q_true = np.array([0.6, -0.4, -1.1])

    print("Original joint angles [rad]:", q_true)

    target = fk_position(q_true)

    print("Target tip position [m]:", target)

    print("\n--- Analytical IK ---")

    solutions = ik_all(target)

    if not solutions:
        print("No valid analytical solution found.")

    for index, solution in enumerate(solutions):

        calculated_position = fk_position(solution)

        error = np.linalg.norm(
            calculated_position - target
        )

        print(f"\nSolution {index + 1}:")
        print("Joint angles [rad]:", solution)
        print("Joint angles [deg]:", np.degrees(solution))
        print("Position error [m]:", error)

    print("\n--- Best IK solution ---")

    best_solution = ik(target, q_seed=q_true)

    print("Best solution:", best_solution)

    print("\n--- Numerical IK ---")

    q_numeric, converged = ik_numeric(
        target,
        q_seed=np.array([0.0, -0.3, -0.5]),
    )

    print("Numerical solution:", q_numeric)
    print("Converged:", converged)

    print(
        "Numerical position error [m]:",
        np.linalg.norm(fk_position(q_numeric) - target),
    )
