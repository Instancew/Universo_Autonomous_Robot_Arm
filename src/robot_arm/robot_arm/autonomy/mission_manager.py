"""Coordinate guarded mission execution and bounded recovery."""
from typing import Callable, Optional

from ..state.control_state import ControlAuthority, ControlOwner
from ..state.state_manager import StateManager
from .recovery_manager import RecoveryManager, RecoveryStatus
from .state_machine import StateMachine, AutonomyState


class MissionManager:
    def __init__(
        self, state_machine: Optional[StateMachine] = None,
        recovery_manager: Optional[RecoveryManager] = None,
        control_authority: Optional[ControlAuthority] = None,
        readiness_check: Optional[Callable[[], bool]] = None,
        state_manager: Optional[StateManager] = None,
    ):
        self.state_machine = state_machine or StateMachine()
        self.recovery_manager = recovery_manager or RecoveryManager()
        self.state_manager = state_manager
        self.control_authority = control_authority or (state_manager.control if state_manager else ControlAuthority())
        if readiness_check is not None:
            self.readiness_check = readiness_check
        elif state_manager is not None:
            self.readiness_check = lambda: state_manager.state_ready_for_autonomy
        else:
            self.readiness_check = None
        self.current_mission: Optional[str] = None
        self.last_error: Optional[str] = None

    def complete_homing(self, success: bool, error: Optional[str] = None) -> bool:
        """Complete the HOMING phase and enter READY only after state validation.

        The actual physical homing routine is external. When a StateManager is
        supplied, its joint feedback/calibration checks must pass as well.
        """
        if self.state_machine.state != AutonomyState.HOMING:
            return False
        if not success:
            self.last_error = error or "Homing procedure failed."
            self.state_machine.transition(AutonomyState.ERROR)
            return False
        if self.state_manager is not None and not self.state_manager.mark_arm_homed(True):
            self.last_error = "Homing was reported successful, but StateManager rejected the arm state."
            self.state_machine.transition(AutonomyState.ERROR)
            return False
        if not self.state_machine.mark_homed():
            self.last_error = self.state_machine.last_transition_error or "Homing confirmation was rejected."
            self.state_machine.transition(AutonomyState.ERROR)
            return False
        if not self.state_machine.transition(AutonomyState.READY):
            self.last_error = self.state_machine.last_transition_error or "Could not enter READY after homing."
            self.state_machine.transition(AutonomyState.ERROR)
            return False
        self.last_error = None
        return True

    def start_mission(self, name: str, execute: Callable[[], bool]) -> bool:
        """Run one mission synchronously when lifecycle and readiness permit."""
        if not isinstance(name, str) or not name.strip() or not callable(execute):
            return False
        if self.state_machine.state != AutonomyState.READY:
            return False
        if self.readiness_check is not None:
            try:
                if not self.readiness_check():
                    self.last_error = "System readiness check failed; mission was not started."
                    return False
            except Exception as exc:
                self.last_error = f"System readiness check raised an exception: {exc}"
                return False
        if not self.control_authority.acquire(ControlOwner.AUTONOMY):
            self.last_error = "Arm command authority is unavailable or emergency stop is latched."
            return False
        if not self.state_machine.transition(AutonomyState.EXECUTING_MISSION):
            self.control_authority.release(ControlOwner.AUTONOMY)
            return False

        self.current_mission = name.strip()
        self.last_error = None
        try:
            success = bool(execute())
        except Exception as exc:
            success = False
            self.last_error = f"Mission '{self.current_mission}' raised an exception: {exc}"

        # An E-stop may have been latched while the mission callback was running.
        if self.state_machine.state == AutonomyState.EMERGENCY_STOP or self.control_authority.emergency_latched:
            self.last_error = self.last_error or f"Mission '{self.current_mission}' interrupted by emergency stop."
            self.current_mission = None
            return False

        if success and self.state_machine.transition(AutonomyState.READY):
            self.current_mission = None
            self.control_authority.release(ControlOwner.AUTONOMY)
            return True

        if self.last_error is None:
            self.last_error = f"Mission '{self.current_mission}' failed."
        self.current_mission = None
        self.control_authority.release(ControlOwner.AUTONOMY)
        self.recovery_manager.report_failure(self.last_error)
        if self.state_machine.transition(AutonomyState.RECOVERING):
            self.control_authority.acquire(ControlOwner.RECOVERY)
        return False

    def execute_recovery(self, recover: Callable[[], bool]) -> bool:
        """Run the supplied recovery routine up to RecoveryManager's attempt limit."""
        if not callable(recover) or self.state_machine.state != AutonomyState.RECOVERING:
            return False
        if self.control_authority.owner != ControlOwner.RECOVERY:
            if not self.control_authority.acquire(ControlOwner.RECOVERY):
                return False
        while self.recovery_manager.begin_recovery():
            try:
                success = bool(recover())
                error = None if success else "Recovery routine reported failure."
            except Exception as exc:
                success = False
                error = f"Recovery routine raised an exception: {exc}"
            if self.state_machine.state == AutonomyState.EMERGENCY_STOP or self.control_authority.emergency_latched:
                return False
            self.recovery_manager.complete_recovery(success, error)
            if success:
                return self.finish_recovery(True)
        self.last_error = self.recovery_manager.last_error or "Recovery attempts exhausted."
        return self.finish_recovery(False)

    def finish_recovery(self, recovered: bool) -> bool:
        """Finalize recovery after an external routine has completed."""
        if self.state_machine.state != AutonomyState.RECOVERING:
            return False
        if recovered and self.recovery_manager.status != RecoveryStatus.RECOVERED:
            return False
        target = AutonomyState.READY if recovered else AutonomyState.ERROR
        if not self.state_machine.transition(target):
            return False
        self.current_mission = None
        self.control_authority.release(ControlOwner.RECOVERY)
        if not recovered and self.last_error is None:
            self.last_error = self.recovery_manager.last_error or "Recovery failed."
        return True

    def emergency_stop(self, reason: str = "Emergency stop requested.") -> None:
        """Latch software E-stop. Physical stopping must be handled by safety hardware."""
        self.last_error = reason
        self.control_authority.latch_emergency_stop()
        self.recovery_manager.require_emergency_stop(reason)
        self.state_machine.latch_emergency_stop()
        self.current_mission = None

    def reset_emergency_stop(self, *, manual_reset_confirmed: bool) -> bool:
        """Clear the software latch only after explicit human confirmation."""
        if not manual_reset_confirmed or self.state_machine.state != AutonomyState.EMERGENCY_STOP:
            return False
        if not self.control_authority.reset_emergency_stop(manual_reset_confirmed=True):
            return False
        if not self.state_machine.reset(manual_reset_confirmed=True):
            return False
        self.recovery_manager = RecoveryManager(self.recovery_manager.max_attempts)
        self.last_error = None
        return True
