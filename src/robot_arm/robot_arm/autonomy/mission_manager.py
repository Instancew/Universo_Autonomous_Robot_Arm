"""Coordinate execution of robot-arm missions."""

from typing import Callable, Optional

from .state_machine import StateMachine, AutonomyState


class MissionManager:
    def __init__(self, state_machine: Optional[StateMachine] = None):
        self.state_machine = state_machine or StateMachine()
        self.current_mission: Optional[str] = None
        self.last_error: Optional[str] = None

    def start_mission(self, name: str, execute: Callable[[], bool]) -> bool:
        """Execute a mission only when the autonomy system is READY."""
        if self.state_machine.state != AutonomyState.READY:
            return False

        if not name.strip():
            return False

        if not self.state_machine.transition(
            AutonomyState.EXECUTING_MISSION
        ):
            return False

        self.current_mission = name
        self.last_error = None

        try:
            success = execute()

            if success:
                if self.state_machine.transition(AutonomyState.READY):
                    self.current_mission = None
                    return True

                # The mission finished, but the state machine was moved elsewhere meanwhile
                # (for example EMERGENCY_STOP): do not report success.
                self.last_error = (
                    f"Mission '{name}' finished but the autonomy state is "
                    f"'{self.state_machine.state.value}', not 'executing_mission'."
                )
                self.current_mission = None
                return False

            self.last_error = f"Mission '{name}' failed."

        except Exception as exc:
            self.last_error = (
                f"Mission '{name}' raised an exception: {exc}"
            )

        self.state_machine.transition(AutonomyState.RECOVERING)
        self.current_mission = None
        return False

    def finish_recovery(self, recovered: bool) -> bool:
        """Update autonomy state after external recovery handling."""
        if self.state_machine.state != AutonomyState.RECOVERING:
            return False

        target = (
            AutonomyState.READY if recovered else AutonomyState.ERROR
        )

        if not self.state_machine.transition(target):
            return False

        self.current_mission = None
        return True
