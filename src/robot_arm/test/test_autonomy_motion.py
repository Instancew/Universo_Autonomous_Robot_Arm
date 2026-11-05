"""Behaviour tests for autonomy/, motion/ and the joint-limit loader (pure Python, no ROS)."""
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from robot_arm.autonomy import AutonomyState, MissionManager, RecoveryManager, RecoveryStatus, StateMachine  # noqa: E402
from robot_arm.motion import CollisionChecker, MotionPlanner, TrajectoryExecutor  # noqa: E402
from robot_arm.motion.collision_checker import SphereObstacle  # noqa: E402
from robot_arm.safety.joint_limits_loader import load_joint_limits  # noqa: E402


def _ready_manager():
    mm = MissionManager()
    assert mm.state_machine.transition(AutonomyState.INITIALIZING)
    assert mm.state_machine.transition(AutonomyState.READY)
    return mm


def test_state_machine_blocks_illegal_transition():
    sm = StateMachine()
    assert not sm.transition(AutonomyState.EXECUTING_MISSION)      # IDLE -> EXECUTING is not allowed
    assert sm.state == AutonomyState.IDLE


def test_emergency_stop_has_no_exit():
    sm = StateMachine()
    assert sm.transition(AutonomyState.EMERGENCY_STOP)
    assert not sm.transition(AutonomyState.IDLE) and not sm.reset()


def test_mission_success_returns_to_ready():
    mm = _ready_manager()
    assert mm.start_mission('pick', lambda: True)
    assert mm.state_machine.state == AutonomyState.READY and mm.current_mission is None


def test_mission_failure_goes_to_recovering_then_ready():
    mm = _ready_manager()
    assert not mm.start_mission('pick', lambda: False)
    assert mm.state_machine.state == AutonomyState.RECOVERING and mm.last_error
    assert mm.finish_recovery(True) and mm.state_machine.state == AutonomyState.READY


def test_mission_exception_is_recorded():
    mm = _ready_manager()

    def boom():
        raise RuntimeError('camera lost')
    assert not mm.start_mission('pick', boom)
    assert 'camera lost' in mm.last_error


def test_mission_does_not_report_success_after_emergency_stop():
    mm = _ready_manager()

    def mission():
        mm.state_machine.transition(AutonomyState.EMERGENCY_STOP)
        return True
    assert not mm.start_mission('pick', mission)
    assert mm.state_machine.state == AutonomyState.EMERGENCY_STOP and mm.last_error


def test_recovery_manager_gives_up_after_max_attempts():
    rm = RecoveryManager(max_attempts=2)
    rm.report_failure('x')
    for _ in range(2):
        assert rm.begin_recovery()
        rm.complete_recovery(False)
    assert rm.status == RecoveryStatus.FAILED and not rm.begin_recovery()


def test_planner_synchronises_joints_and_respects_speed():
    traj = MotionPlanner(max_velocity=0.1).plan_trajectory({'a': 0.0, 'b': 0.0}, {'a': 0.5, 'b': 0.1}, time_step=0.1)
    assert traj.duration == pytest.approx(5.0)                       # slowest joint: 0.5 rad at 0.1 rad/s
    assert traj.points[0] == {'a': 0.0, 'b': 0.0} and traj.points[-1] == pytest.approx({'a': 0.5, 'b': 0.1})
    assert all(b > a for a, b in zip(traj.times, traj.times[1:]))


def test_planner_rejects_mismatched_joints_and_bad_input():
    mp = MotionPlanner()
    assert mp.plan_trajectory({'a': 0.0}, {'b': 0.0}) is None
    with pytest.raises(ValueError):
        mp.plan_trajectory({'a': float('nan')}, {'a': 0.0})
    with pytest.raises(ValueError):
        MotionPlanner(max_velocity=0)


def test_executor_sends_points_in_order_and_keeps_error_text():
    traj = MotionPlanner().plan_trajectory({'a': 0.0}, {'a': 0.2}, time_step=0.1)
    sent = []
    assert TrajectoryExecutor(sent.append).execute(traj, wait=False) and len(sent) == len(traj.points)

    def boom(_):
        raise RuntimeError('driver error')
    ex = TrajectoryExecutor(boom)
    assert not ex.execute(traj, wait=False) and 'driver error' in ex.last_error


def test_collision_checker_uses_radius_and_margin():
    cc = CollisionChecker(safety_margin=0.02)
    cc.add_obstacle(SphereObstacle(0.5, 0.0, 0.3, 0.05))
    assert cc.point_in_collision((0.5, 0.0, 0.36))                   # inside radius + margin
    assert not cc.point_in_collision((0.5, 0.0, 0.40))


def test_joints_yaml_loads_with_the_limit_loader():
    # Fails while config/joints.yaml gives the continuous base_joint numeric bounds (the loader forbids them).
    limits, _verified = load_joint_limits(HERE.parent / 'config' / 'joints.yaml')
    assert limits.validate_joint_position('base_joint', 12.0)       # continuous: any angle
    assert limits.validate_joint_position('shoulder_joint', -0.5)
    assert not limits.validate_joint_position('shoulder_joint', 0.0)
