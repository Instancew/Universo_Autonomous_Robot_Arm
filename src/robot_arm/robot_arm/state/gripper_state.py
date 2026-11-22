"""Software state for the currently manually operated gripper."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class GripperStatus(Enum):
    UNKNOWN = "unknown"
    OPEN = "open"
    CLOSED = "closed"
    MOVING = "moving"
    HOLDING_OBJECT = "holding_object"
    ERROR = "error"


@dataclass
class GripperState:
    status: GripperStatus = GripperStatus.UNKNOWN
    position: Optional[float] = None
    target_position: Optional[float] = None
    object_detected: bool = False
    error_message: Optional[str] = None
    position_valid: bool = False

    @staticmethod
    def _validate_position(value: float, label: str) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{label} must be a finite number.")

    def set_status(self, status: GripperStatus) -> None:
        if not isinstance(status, GripperStatus):
            raise TypeError("status must be a GripperStatus value.")
        self.status = status

    def update_position(self, measured_position: float) -> None:
        self._validate_position(measured_position, "measured_position")
        self.position = float(measured_position)
        self.position_valid = True

    def set_target(self, target_position: float) -> None:
        self._validate_position(target_position, "target_position")
        self.target_position = float(target_position)

    def confirm_object(self) -> None:
        self.object_detected = True
        self.status = GripperStatus.HOLDING_OBJECT

    def clear_object(self) -> None:
        self.object_detected = False
        if self.status == GripperStatus.HOLDING_OBJECT:
            self.status = GripperStatus.CLOSED

    def set_error(self, message: str) -> None:
        if not isinstance(message, str) or not message.strip():
            raise ValueError("Error message must be a non-empty string.")
        self.error_message = message
        self.status = GripperStatus.ERROR
