"""Thread-safe logical command ownership shared by arm command sources.

This is a software arbitration primitive. Every command source (teleop, Servo,
autonomy) must use the same instance for arbitration to be effective system-wide.
It is not a substitute for a hardware emergency-stop circuit.
"""
from enum import Enum
from threading import RLock
from typing import Optional


class ControlOwner(Enum):
    NONE = "none"
    TELEOP = "teleop"
    MOVEIT_SERVO = "moveit_servo"
    AUTONOMY = "autonomy"
    HOMING = "homing"
    RECOVERY = "recovery"


class ControlAuthority:
    def __init__(self) -> None:
        self._owner = ControlOwner.NONE
        self._emergency_latched = False
        self._lock = RLock()

    @property
    def owner(self) -> ControlOwner:
        with self._lock:
            return self._owner

    @property
    def emergency_latched(self) -> bool:
        with self._lock:
            return self._emergency_latched

    def acquire(self, owner: ControlOwner) -> bool:
        if not isinstance(owner, ControlOwner) or owner == ControlOwner.NONE:
            raise ValueError("A valid, non-NONE control owner is required.")
        with self._lock:
            if self._emergency_latched or self._owner not in {ControlOwner.NONE, owner}:
                return False
            self._owner = owner
            return True

    def release(self, owner: ControlOwner) -> bool:
        with self._lock:
            if self._emergency_latched or self._owner != owner:
                return False
            self._owner = ControlOwner.NONE
            return True

    def latch_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_latched = True
            self._owner = ControlOwner.NONE

    def reset_emergency_stop(self, *, manual_reset_confirmed: bool) -> bool:
        with self._lock:
            if not manual_reset_confirmed:
                return False
            self._emergency_latched = False
            self._owner = ControlOwner.NONE
            return True
