"""Validated, hardware-independent state for one arm joint."""
from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class JointState:
    name: str
    position: float = 0.0
    target_position: Optional[float] = None
    velocity: float = 0.0
    is_moving: bool = False
    is_calibrated: bool = False
    position_valid: bool = False
    velocity_valid: bool = False
    last_update_monotonic: Optional[float] = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("Joint name must be a non-empty string.")
        self._validate_finite(self.position, "position")
        self._validate_finite(self.velocity, "velocity")
        if self.target_position is not None:
            self._validate_finite(self.target_position, "target_position")

    @staticmethod
    def _validate_finite(value: float, label: str) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{label} must be a finite number.")

    @property
    def position_error(self) -> Optional[float]:
        if self.target_position is None or not self.position_valid:
            return None
        return self.target_position - self.position

    def update_position(self, measured_position: float, timestamp: Optional[float] = None) -> None:
        self._validate_finite(measured_position, "measured_position")
        if timestamp is not None:
            self._validate_finite(timestamp, "timestamp")
        self.position = float(measured_position)
        self.position_valid = True
        self.last_update_monotonic = timestamp

    def update_feedback(
        self, measured_position: float, measured_velocity: Optional[float] = None,
        *, is_moving: Optional[bool] = None, timestamp: Optional[float] = None,
    ) -> None:
        # Validate the complete sample before mutating state, so a bad velocity
        # or timestamp cannot leave a partially-applied feedback update.
        self._validate_finite(measured_position, "measured_position")
        if measured_velocity is not None:
            self._validate_finite(measured_velocity, "measured_velocity")
        if timestamp is not None:
            self._validate_finite(timestamp, "timestamp")
        self.position = float(measured_position)
        self.position_valid = True
        self.last_update_monotonic = timestamp
        if measured_velocity is not None:
            self.velocity = float(measured_velocity)
            self.velocity_valid = True
        if is_moving is not None:
            self.is_moving = bool(is_moving)

    def set_target(self, target_position: float) -> None:
        self._validate_finite(target_position, "target_position")
        self.target_position = float(target_position)

    def stop(self) -> None:
        self.is_moving = False
        self.velocity = 0.0
        self.velocity_valid = True
