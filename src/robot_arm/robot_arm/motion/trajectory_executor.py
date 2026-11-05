from typing import Callable, Dict, Optional
import time

from .trajectory import Trajectory


class TrajectoryExecutor:
    """Executes trajectory waypoints through a supplied callback."""

    def __init__(
        self,
        command_callback: Callable[[Dict[str, float]], None],
    ):
        self.command_callback = command_callback
        self.is_executing = False
        self.stop_requested = False
        self.last_error: Optional[str] = None

    def execute(
        self,
        trajectory: Trajectory,
        wait: bool = True,
    ) -> bool:
        """
        Send trajectory waypoints to the command callback.

        The callback must handle sending commands to the appropriate
        controller. This class does not communicate with motor hardware.
        """

        if self.is_executing or trajectory.is_empty:
            return False

        self.is_executing = True
        self.stop_requested = False
        self.last_error = None
        start_time = time.monotonic()

        try:
            for positions, waypoint_time in zip(
                trajectory.points, trajectory.times
            ):
                if self.stop_requested:
                    return False

                if wait:
                    elapsed = time.monotonic() - start_time
                    delay = waypoint_time - elapsed

                    if delay > 0:
                        time.sleep(delay)

                if self.stop_requested:
                    return False

                self.command_callback(positions)

            return True

        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False

        finally:
            self.is_executing = False

    def stop(self) -> None:
        """Request that execution stop before the next waypoint."""
        self.stop_requested = True