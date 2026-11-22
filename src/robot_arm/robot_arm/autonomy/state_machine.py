"""Guarded lifecycle state machine for autonomous arm operation."""
from enum import Enum
from threading import RLock


class AutonomyState(Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    HOMING = "homing"
    READY = "ready"
    EXECUTING_MISSION = "executing_mission"
    RECOVERING = "recovering"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


class StateMachine:
    """Enforce legal lifecycle transitions and a mandatory homing gate.

    Marking the arm homed is an acknowledgement from the homing component; this
    class does not perform physical homing or verify encoder/switch hardware.
    """
    _ALLOWED = {
        AutonomyState.IDLE: {AutonomyState.INITIALIZING, AutonomyState.EMERGENCY_STOP},
        AutonomyState.INITIALIZING: {AutonomyState.HOMING, AutonomyState.ERROR, AutonomyState.EMERGENCY_STOP},
        AutonomyState.HOMING: {AutonomyState.READY, AutonomyState.ERROR, AutonomyState.EMERGENCY_STOP},
        AutonomyState.READY: {AutonomyState.EXECUTING_MISSION, AutonomyState.IDLE, AutonomyState.ERROR, AutonomyState.EMERGENCY_STOP},
        AutonomyState.EXECUTING_MISSION: {AutonomyState.READY, AutonomyState.RECOVERING, AutonomyState.ERROR, AutonomyState.EMERGENCY_STOP},
        AutonomyState.RECOVERING: {AutonomyState.READY, AutonomyState.ERROR, AutonomyState.EMERGENCY_STOP},
        AutonomyState.ERROR: {AutonomyState.IDLE, AutonomyState.EMERGENCY_STOP},
        AutonomyState.EMERGENCY_STOP: set(),
    }

    def __init__(self, *, homing_required: bool = True):
        self._state = AutonomyState.IDLE
        self.homing_required = bool(homing_required)
        self.homed = not self.homing_required
        self.last_transition_error = None
        self._lock = RLock()

    @property
    def state(self) -> AutonomyState:
        with self._lock:
            return self._state

    def transition(self, new_state: AutonomyState) -> bool:
        if not isinstance(new_state, AutonomyState):
            raise TypeError("new_state must be an AutonomyState value.")
        with self._lock:
            if new_state not in self._ALLOWED[self._state]:
                self.last_transition_error = f"Transition {self._state.value} -> {new_state.value} is not allowed."
                return False
            if new_state == AutonomyState.READY and self.homing_required and not self.homed:
                self.last_transition_error = "Homing must be completed before entering READY."
                return False
            self._state = new_state
            self.last_transition_error = None
            return True

    def mark_homed(self) -> bool:
        with self._lock:
            if self._state != AutonomyState.HOMING:
                self.last_transition_error = "Homing can only be confirmed while in HOMING state."
                return False
            self.homed = True
            self.last_transition_error = None
            return True

    def invalidate_homing(self) -> None:
        with self._lock:
            self.homed = False

    def reset(self, *, manual_reset_confirmed: bool = False) -> bool:
        """Reset ERROR, or reset E-stop only after an explicit manual confirmation."""
        with self._lock:
            if self._state == AutonomyState.ERROR:
                self._state = AutonomyState.IDLE
                self.last_transition_error = None
                return True
            if self._state == AutonomyState.EMERGENCY_STOP and manual_reset_confirmed:
                self._state = AutonomyState.IDLE
                self.last_transition_error = None
                return True
            return False

    def latch_emergency_stop(self) -> None:
        with self._lock:
            self._state = AutonomyState.EMERGENCY_STOP
            self.last_transition_error = None
