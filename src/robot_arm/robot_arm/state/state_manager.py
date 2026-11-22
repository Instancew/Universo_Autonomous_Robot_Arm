"""Thread-safe aggregate and update API for robot-arm state."""
from threading import RLock
from typing import Dict, Optional

from .arm_state import ArmState, ArmStatus
from .joint_state import JointState
from .gripper_state import GripperState, GripperStatus
from .system_state import SystemState, SystemStatus
from .control_state import ControlAuthority


DEFAULT_JOINTS = ("base_joint", "shoulder_joint", "elbow_joint")


class StateManager:
    """Owns the current system, arm, gripper and command-authority state.

    State objects are returned for compatibility with existing consumers. Use
    this manager's update methods when writing feedback so updates are locked.
    """
    def __init__(self, joint_names=DEFAULT_JOINTS):
        names = tuple(joint_names)
        if not names or len(set(names)) != len(names) or any(not isinstance(n, str) or not n.strip() for n in names):
            raise ValueError("joint_names must contain unique, non-empty names.")
        self._lock = RLock()
        self.system = SystemState()
        self.arm = ArmState()
        self.gripper = GripperState()
        self.control = ControlAuthority()
        for name in names:
            self.arm.add_joint(JointState(name=name))

    def get_arm_state(self) -> ArmState:
        return self.arm

    def get_gripper_state(self) -> GripperState:
        return self.gripper

    def get_system_state(self) -> SystemState:
        return self.system

    def get_joint_states(self) -> Dict[str, JointState]:
        with self._lock:
            return dict(self.arm.joints)

    def update_joint_position(self, joint_name: str, measured_position: float) -> bool:
        return self.update_joint_feedback(joint_name, measured_position)

    def update_joint_feedback(
        self, joint_name: str, measured_position: float,
        measured_velocity: Optional[float] = None, *,
        is_moving: Optional[bool] = None, timestamp: Optional[float] = None,
    ) -> bool:
        with self._lock:
            joint = self.arm.get_joint(joint_name)
            if joint is None:
                return False
            joint.update_feedback(measured_position, measured_velocity,
                                  is_moving=is_moving, timestamp=timestamp)
            return True

    def set_joint_calibrated(self, joint_name: str, calibrated: bool = True) -> bool:
        if not isinstance(calibrated, bool):
            raise TypeError("calibrated must be bool.")
        with self._lock:
            joint = self.arm.get_joint(joint_name)
            if joint is None:
                return False
            joint.is_calibrated = calibrated
            if not calibrated:
                self.arm.mark_homed(False)
            return True

    def mark_arm_homed(self, homed: bool = True) -> bool:
        with self._lock:
            if homed and not (self.arm.all_joints_calibrated and self.arm.all_joint_positions_valid):
                return False
            self.arm.mark_homed(homed)
            return True

    def set_arm_status(self, status: ArmStatus) -> None:
        with self._lock:
            self.arm.set_status(status)

    def set_gripper_status(self, status: GripperStatus) -> None:
        with self._lock:
            self.gripper.set_status(status)

    def set_system_status(self, status: SystemStatus) -> None:
        with self._lock:
            self.system.set_status(status)

    def update_connection(self, component: str, connected: bool) -> None:
        with self._lock:
            self.system.update_connection(component, connected)

    @property
    def state_ready_for_autonomy(self) -> bool:
        with self._lock:
            return (
                self.system.all_required_connections_ready
                and self.arm.homed
                and self.arm.all_joint_positions_valid
                and self.arm.all_joints_calibrated
                and self.arm.status == ArmStatus.IDLE
                and not self.arm.is_moving
                and self.system.status == SystemStatus.READY
                and not self.control.emergency_latched
            )
