"""Regression tests for the State/Autonomy refactor (pure Python)."""
import math
import pytest

from robot_arm.state import (
    ArmStatus, ControlAuthority, ControlOwner, GripperStatus,
    JointState, StateManager, SystemStatus,
)
from robot_arm.autonomy import AutonomyState, MissionManager, StateMachine


def test_state_manager_starts_unknown_and_uses_urdf_joint_names():
    state = StateManager()
    assert set(state.get_joint_states()) == {"base_joint", "shoulder_joint", "elbow_joint"}
    assert not state.system.jetson_connected
    assert not state.state_ready_for_autonomy
    assert not state.arm.homed


def test_joint_feedback_validation_and_position_error():
    joint = JointState("shoulder_joint")
    joint.set_target(1.0)
    assert joint.position_error is None
    joint.update_feedback(0.4, 0.1, is_moving=True, timestamp=12.0)
    assert joint.position_error == pytest.approx(0.6)
    assert joint.velocity_valid and joint.is_moving
    with pytest.raises(ValueError):
        joint.update_position(float("nan"))
    previous = joint.position
    with pytest.raises(ValueError):
        joint.update_feedback(0.8, float("inf"))
    assert joint.position == previous  # invalid sample is applied atomically


def test_state_manager_requires_all_joints_calibrated_before_homed():
    manager = StateManager()
    for name in manager.get_joint_states():
        assert manager.update_joint_feedback(name, 0.0)
        assert manager.set_joint_calibrated(name)
    assert manager.mark_arm_homed()
    assert manager.arm.homed and manager.arm.status == ArmStatus.IDLE


def test_unknown_joint_feedback_is_rejected():
    assert not StateManager().update_joint_position("wrist_joint", 0.0)


def test_control_authority_is_exclusive_and_estop_latched():
    authority = ControlAuthority()
    assert authority.acquire(ControlOwner.TELEOP)
    assert not authority.acquire(ControlOwner.AUTONOMY)
    assert not authority.release(ControlOwner.AUTONOMY)
    authority.latch_emergency_stop()
    assert authority.owner == ControlOwner.NONE
    assert not authority.acquire(ControlOwner.AUTONOMY)
    assert not authority.reset_emergency_stop(manual_reset_confirmed=False)
    assert authority.reset_emergency_stop(manual_reset_confirmed=True)


def test_homing_is_required_before_ready():
    machine = StateMachine()
    assert machine.transition(AutonomyState.INITIALIZING)
    assert not machine.transition(AutonomyState.READY)
    assert machine.transition(AutonomyState.HOMING)
    assert machine.mark_homed()
    assert machine.transition(AutonomyState.READY)


def test_mission_manager_uses_shared_control_and_readiness_gate():
    machine = StateMachine(homing_required=False)
    assert machine.transition(AutonomyState.INITIALIZING)
    assert machine.transition(AutonomyState.HOMING)
    assert machine.mark_homed()
    assert machine.transition(AutonomyState.READY)
    authority = ControlAuthority()
    manager = MissionManager(machine, control_authority=authority, readiness_check=lambda: False)
    assert not manager.start_mission("pick", lambda: True)
    assert machine.state == AutonomyState.READY
    assert authority.owner == ControlOwner.NONE


def test_mission_failure_recovery_and_control_release():
    machine = StateMachine(homing_required=False)
    assert machine.transition(AutonomyState.INITIALIZING)
    assert machine.transition(AutonomyState.HOMING)
    assert machine.mark_homed()
    assert machine.transition(AutonomyState.READY)
    authority = ControlAuthority()
    manager = MissionManager(machine, control_authority=authority)
    assert not manager.start_mission("pick", lambda: False)
    assert machine.state == AutonomyState.RECOVERING
    assert authority.owner == ControlOwner.RECOVERY
    assert manager.execute_recovery(lambda: True)
    assert machine.state == AutonomyState.READY
    assert authority.owner == ControlOwner.NONE


def test_emergency_stop_cannot_be_cleared_without_manual_confirmation():
    machine = StateMachine(homing_required=False)
    assert machine.transition(AutonomyState.INITIALIZING)
    assert machine.transition(AutonomyState.HOMING)
    assert machine.mark_homed()
    assert machine.transition(AutonomyState.READY)
    manager = MissionManager(machine)
    manager.emergency_stop("operator stop")
    assert machine.state == AutonomyState.EMERGENCY_STOP
    assert not manager.reset_emergency_stop(manual_reset_confirmed=False)
    assert manager.reset_emergency_stop(manual_reset_confirmed=True)
    assert machine.state == AutonomyState.IDLE


def test_gripper_rejects_non_finite_feedback():
    from robot_arm.state import GripperState
    with pytest.raises(ValueError):
        GripperState().update_position(math.inf)


def test_complete_homing_coordinates_state_manager_and_lifecycle():
    state = StateManager()
    for name in state.get_joint_states():
        state.update_joint_feedback(name, 0.0)
        state.set_joint_calibrated(name)
    machine = StateMachine()
    assert machine.transition(AutonomyState.INITIALIZING)
    assert machine.transition(AutonomyState.HOMING)
    manager = MissionManager(machine, state_manager=state)
    assert manager.complete_homing(True)
    assert state.arm.homed
    assert machine.state == AutonomyState.READY


def test_complete_homing_rejects_unverified_state():
    machine = StateMachine()
    assert machine.transition(AutonomyState.INITIALIZING)
    assert machine.transition(AutonomyState.HOMING)
    manager = MissionManager(machine, state_manager=StateManager())
    assert not manager.complete_homing(True)
    assert machine.state == AutonomyState.ERROR



def test_state_readiness_requires_verified_connections_and_idle_status():
    state = StateManager()
    for name in state.get_joint_states():
        state.update_joint_feedback(name, 0.0)
        state.set_joint_calibrated(name)
    assert state.mark_arm_homed()
    for component in ("jetson", "stm32", "can", "camera"):
        state.update_connection(component, True)
    assert not state.state_ready_for_autonomy  # system is still INITIALIZING
    state.set_system_status(SystemStatus.READY)
    assert state.state_ready_for_autonomy
    state.set_arm_status(ArmStatus.MOVING)
    assert not state.state_ready_for_autonomy
