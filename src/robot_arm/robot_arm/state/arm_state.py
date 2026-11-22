"""Aggregate state for the three independently controlled arm joints."""
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

from .joint_state import JointState


class ArmStatus(Enum):
    UNKNOWN = "unknown"
    UNHOMED = "unhomed"
    IDLE = "idle"
    HOMING = "homing"
    MOVING = "moving"
    EXECUTING_TASK = "executing_task"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class ArmState:
    status: ArmStatus = ArmStatus.UNHOMED
    joints: Dict[str, JointState] = field(default_factory=dict)
    current_task: Optional[str] = None
    error_message: Optional[str] = None
    homed: bool = False

    def add_joint(self, joint: JointState) -> None:
        if not isinstance(joint, JointState):
            raise TypeError("joint must be a JointState instance.")
        if joint.name in self.joints:
            raise ValueError(f"Joint '{joint.name}' is already registered.")
        self.joints[joint.name] = joint

    def get_joint(self, name: str) -> Optional[JointState]:
        return self.joints.get(name)

    @property
    def all_joint_positions_valid(self) -> bool:
        return bool(self.joints) and all(j.position_valid for j in self.joints.values())

    @property
    def all_joints_calibrated(self) -> bool:
        return bool(self.joints) and all(j.is_calibrated for j in self.joints.values())

    @property
    def is_moving(self) -> bool:
        return any(j.is_moving for j in self.joints.values()) or self.status in {
            ArmStatus.HOMING, ArmStatus.MOVING, ArmStatus.EXECUTING_TASK
        }

    def set_status(self, status: ArmStatus) -> None:
        if not isinstance(status, ArmStatus):
            raise TypeError("status must be an ArmStatus value.")
        self.status = status

    def set_error(self, message: str) -> None:
        if not isinstance(message, str) or not message.strip():
            raise ValueError("Error message must be a non-empty string.")
        self.error_message = message
        self.status = ArmStatus.ERROR

    def clear_error(self) -> None:
        self.error_message = None
        if self.status == ArmStatus.ERROR:
            self.status = ArmStatus.IDLE if self.homed else ArmStatus.UNHOMED

    def mark_homed(self, homed: bool = True) -> None:
        self.homed = bool(homed)
        if not self.homed and self.status not in {ArmStatus.ERROR, ArmStatus.EMERGENCY_STOP}:
            self.status = ArmStatus.UNHOMED
        elif self.homed and self.status == ArmStatus.UNHOMED:
            self.status = ArmStatus.IDLE
