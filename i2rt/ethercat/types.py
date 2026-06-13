from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class MotorMsg:
    """Single CAN frame embedded in an EtherCAT PDO (see SOEM config.h)."""

    id: int = 0
    rtr: int = 0
    dlc: int = 0
    data: list[int] = field(default_factory=lambda: [0] * 8)

    def clear(self) -> None:
        self.id = 0
        self.rtr = 0
        self.dlc = 0
        self.data = [0] * 8


@dataclass
class EtherCATMsg:
    """EtherCAT gateway process-data message (8 CAN slots per slave)."""

    motor_num: int = 0
    can_ide: int = 0
    motor: list[MotorMsg] = field(default_factory=lambda: [MotorMsg() for _ in range(8)])

    def clear(self) -> None:
        self.motor_num = 0
        self.can_ide = 0
        self.motor = [MotorMsg() for _ in range(8)]


@dataclass
class MotorCommFeedback:
    motor_id: int = 0
    ins_code: int = 0
    motor_fbd: int = 0


@dataclass
class ODMotorMsg:
    """Decoded motor feedback (OD = object dictionary style fields from SOEM demo)."""

    motor_id: int = 0
    temperature: int = 0
    error: int = 0
    angle_actual_int: int = 0
    angle_desired_int: int = 0
    speed_actual_int: int = 0
    speed_desired_int: int = 0
    current_actual_int: int = 0
    current_desired_int: int = 0
    angle_actual_rad: float = 0.0
    angle_desired_rad: float = 0.0
    speed_actual_rad: float = 0.0
    speed_desired_rad: float = 0.0
    current_actual_float: float = 0.0
    current_desired_float: float = 0.0
    angle_actual_float: float = 0.0
    angle_desired_float: float = 0.0
    speed_actual_float: float = 0.0
    speed_desired_float: float = 0.0
    power: float = 0.0
    acceleration: int = 0
    linkage_KP: int = 0
    speed_KI: int = 0
    feedback_KP: int = 0
    feedback_KD: int = 0
