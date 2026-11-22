"""Bounded recovery-attempt tracking for arm autonomy."""
from enum import Enum
from threading import RLock
from typing import Optional


class RecoveryStatus(Enum):
    IDLE = "idle"
    RECOVERY_REQUIRED = "recovery_required"
    RECOVERING = "recovering"
    RECOVERED = "recovered"
    FAILED = "failed"
    EMERGENCY_STOP = "emergency_stop"


class RecoveryManager:
    def __init__(self, max_attempts: int = 3):
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or max_attempts < 1:
            raise ValueError("max_attempts must be an integer of at least 1.")
        self.max_attempts = max_attempts
        self.attempts = 0
        self.status = RecoveryStatus.IDLE
        self.last_error: Optional[str] = None
        self._lock = RLock()

    def report_failure(self, error: str) -> None:
        if not isinstance(error, str) or not error.strip():
            raise ValueError("error must be a non-empty string.")
        with self._lock:
            if self.status == RecoveryStatus.EMERGENCY_STOP:
                return
            self.last_error = error
            self.attempts = 0
            self.status = RecoveryStatus.RECOVERY_REQUIRED

    def begin_recovery(self) -> bool:
        with self._lock:
            if self.status != RecoveryStatus.RECOVERY_REQUIRED:
                return False
            if self.attempts >= self.max_attempts:
                self.status = RecoveryStatus.FAILED
                return False
            self.attempts += 1
            self.status = RecoveryStatus.RECOVERING
            return True

    def complete_recovery(self, success: bool, error: Optional[str] = None) -> None:
        with self._lock:
            if self.status != RecoveryStatus.RECOVERING:
                raise RuntimeError("Recovery has not been started.")
            if success:
                self.status = RecoveryStatus.RECOVERED
                self.last_error = None
            elif self.attempts >= self.max_attempts:
                self.status = RecoveryStatus.FAILED
                if error:
                    self.last_error = error
            else:
                self.status = RecoveryStatus.RECOVERY_REQUIRED
                if error:
                    self.last_error = error

    def require_emergency_stop(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be a non-empty string.")
        with self._lock:
            self.last_error = reason
            self.status = RecoveryStatus.EMERGENCY_STOP
