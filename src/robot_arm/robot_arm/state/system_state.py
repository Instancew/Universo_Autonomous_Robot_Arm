"""Overall system health; connection defaults are deliberately unknown/false."""
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict


class SystemStatus(Enum):
    INITIALIZING = "initializing"
    NOT_READY = "not_ready"
    READY = "ready"
    BUSY = "busy"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class SystemState:
    status: SystemStatus = SystemStatus.INITIALIZING
    jetson_connected: bool = False
    stm32_connected: bool = False
    can_connected: bool = False
    camera_connected: bool = False
    last_error: str = ""
    connection_details: Dict[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.connection_details.update({
            "jetson": self.jetson_connected,
            "stm32": self.stm32_connected,
            "can": self.can_connected,
            "camera": self.camera_connected,
        })

    def set_status(self, status: SystemStatus) -> None:
        if not isinstance(status, SystemStatus):
            raise TypeError("status must be a SystemStatus value.")
        self.status = status

    def update_connection(self, component: str, connected: bool) -> None:
        if component not in {"jetson", "stm32", "can", "camera"}:
            raise ValueError(f"Unknown component: {component}")
        if not isinstance(connected, bool):
            raise TypeError("connected must be bool.")
        setattr(self, f"{component}_connected", connected)
        self.connection_details[component] = connected

    def set_error(self, message: str) -> None:
        if not isinstance(message, str) or not message.strip():
            raise ValueError("Error message must be a non-empty string.")
        self.last_error = message
        self.status = SystemStatus.ERROR

    def clear_error(self) -> None:
        self.last_error = ""
        if self.status == SystemStatus.ERROR:
            self.status = SystemStatus.INITIALIZING

    @property
    def communications_ready(self) -> bool:
        return self.jetson_connected and self.stm32_connected and self.can_connected

    @property
    def all_required_connections_ready(self) -> bool:
        return self.communications_ready and self.camera_connected
