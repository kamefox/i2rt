from __future__ import annotations

import struct
from typing import Literal

from i2rt.ethercat.math_ops import float_to_uint, uint_to_float
from i2rt.ethercat.types import EtherCATMsg, MotorCommFeedback, MotorMsg, ODMotorMsg

KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
POS_MIN, POS_MAX = -12.5, 12.5
SPD_MIN, SPD_MAX = -18.0, 18.0
T_MIN, T_MAX = -30.0, 30.0
I_MIN, I_MAX = -30.0, 30.0

COMM_ACK = 0x00
COMM_AUTO = 0x01

motor_comm_fbd = MotorCommFeedback()
motor_id_check = 0


def _slot(tx_msg: EtherCATMsg, passage: int) -> MotorMsg:
    if not 1 <= passage <= 8:
        raise ValueError(f"passage must be 1..8, got {passage}")
    return tx_msg.motor[passage - 1]


def _float_bytes(value: float) -> bytes:
    return struct.pack("<f", value)


def motor_setzero(tx_msg: EtherCATMsg, passage: int, motor_id: int) -> None:
    slot = _slot(tx_msg, passage)
    tx_msg.can_ide = 0
    slot.id = 0x7FF
    slot.rtr = 0
    slot.dlc = 4
    slot.data = [motor_id >> 8, motor_id & 0xFF, 0x00, 0x03, 0, 0, 0, 0]


def motor_setting(tx_msg: EtherCATMsg, motor_id: int, cmd: int) -> None:
    """Use CAN1 (passage 1) for ID / mode / zero configuration commands."""
    if cmd == 0:
        return
    slot = tx_msg.motor[0]
    tx_msg.can_ide = 0
    slot.id = 0x7FF
    slot.rtr = 0
    slot.dlc = 4
    slot.data = [motor_id >> 8, motor_id & 0xFF, 0x00, cmd, 0, 0, 0, 0]


def motor_id_reset(tx_msg: EtherCATMsg) -> None:
    slot = tx_msg.motor[0]
    tx_msg.can_ide = 0
    slot.id = 0x7FF
    slot.dlc = 6
    slot.rtr = 0
    slot.data = [0x7F, 0x7F, 0x00, 0x05, 0x7F, 0x7F, 0, 0]


def motor_id_setting(tx_msg: EtherCATMsg, motor_id: int, motor_id_new: int) -> None:
    slot = tx_msg.motor[0]
    tx_msg.can_ide = 0
    slot.id = 0x7FF
    slot.dlc = 6
    slot.rtr = 0
    slot.data = [
        motor_id >> 8,
        motor_id & 0xFF,
        0x00,
        0x04,
        motor_id_new >> 8,
        motor_id_new & 0xFF,
        0,
        0,
    ]


def motor_comm_mode_reading(tx_msg: EtherCATMsg, motor_id: int) -> None:
    slot = tx_msg.motor[0]
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = 0x7FF
    slot.dlc = 4
    slot.data = [motor_id >> 8, motor_id & 0xFF, 0x00, 0x81, 0, 0, 0, 0]


def motor_id_reading(tx_msg: EtherCATMsg) -> None:
    slot = tx_msg.motor[0]
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = 0x7FF
    slot.dlc = 4
    slot.data = [0xFF, 0xFF, 0x00, 0x82, 0, 0, 0, 0]


def send_motor_ctrl_cmd(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    kp: float,
    kd: float,
    pos: float,
    spd: float,
    tor: float,
) -> None:
    slot = _slot(tx_msg, passage)
    kp = max(KP_MIN, min(KP_MAX, kp))
    kd = max(KD_MIN, min(KD_MAX, kd))
    pos = max(POS_MIN, min(POS_MAX, pos))
    spd = max(SPD_MIN, min(SPD_MAX, spd))
    tor = max(T_MIN, min(T_MAX, tor))

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9)
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16)
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12)
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12)

    tx_msg.can_ide = 1
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 8
    slot.data = [
        0x00 | (kp_int >> 7),
        ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8),
        kd_int & 0xFF,
        pos_int >> 8,
        pos_int & 0xFF,
        spd_int >> 4,
        (spd_int & 0x0F) << 4 | (tor_int >> 8),
        tor_int & 0xFF,
    ]


def set_motor_position(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    pos: float,
    spd: int,
    cur: int,
    ack_status: int,
) -> None:
    if ack_status > 3:
        return
    slot = _slot(tx_msg, passage)
    pos_bytes = _float_bytes(pos)
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 8
    slot.data = [
        0x20 | (pos_bytes[3] >> 3),
        (pos_bytes[3] << 5) | (pos_bytes[2] >> 3),
        (pos_bytes[2] << 5) | (pos_bytes[1] >> 3),
        (pos_bytes[1] << 5) | (pos_bytes[0] >> 3),
        (pos_bytes[0] << 5) | (spd >> 10),
        (spd & 0x3FC) >> 2,
        (spd & 0x03) << 6 | (cur >> 6),
        (cur & 0x3F) << 2 | ack_status,
    ]


def set_motor_speed(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    spd: float,
    cur: int,
    ack_status: int,
) -> None:
    slot = _slot(tx_msg, passage)
    spd_bytes = _float_bytes(spd)
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 7
    slot.data = [
        0x40 | ack_status,
        spd_bytes[3],
        spd_bytes[2],
        spd_bytes[1],
        spd_bytes[0],
        cur >> 8,
        cur & 0xFF,
        0,
    ]


def set_motor_cur_tor(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    cur_tor: int,
    ctrl_status: int,
    ack_status: int,
) -> None:
    if ack_status > 3 or ctrl_status > 7:
        return
    slot = _slot(tx_msg, passage)
    if ctrl_status:
        cur_tor = max(-3000, min(3000, cur_tor))
    else:
        cur_tor = max(-2000, min(2000, cur_tor))
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 3
    slot.data = [0x60 | (ctrl_status << 2) | ack_status, cur_tor >> 8, cur_tor & 0xFF, 0, 0, 0, 0, 0]


def set_motor_acceleration(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    acc: int,
    ack_status: int,
) -> None:
    if ack_status > 2:
        return
    acc = min(acc, 2000)
    slot = _slot(tx_msg, passage)
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 4
    slot.data = [0xC0 | ack_status, 0x01, acc >> 8, acc & 0xFF, 0, 0, 0, 0]


def set_motor_linkage_speed_ki(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    linkage: int,
    speed_ki: int,
    ack_status: int,
) -> None:
    if ack_status > 2:
        return
    linkage = min(linkage, 10000)
    speed_ki = min(speed_ki, 10000)
    slot = _slot(tx_msg, passage)
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 6
    slot.data = [
        0xC0 | ack_status,
        0x02,
        linkage >> 8,
        linkage & 0xFF,
        speed_ki >> 8,
        speed_ki & 0xFF,
        0,
        0,
    ]


def set_motor_feedback_kp_kd(
    tx_msg: EtherCATMsg,
    passage: int,
    motor_id: int,
    fdb_kp: int,
    fdb_kd: int,
    ack_status: int,
) -> None:
    if ack_status > 2:
        return
    fdb_kp = min(fdb_kp, 10000)
    fdb_kd = min(fdb_kd, 10000)
    slot = _slot(tx_msg, passage)
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 6
    slot.data = [
        0xC0 | ack_status,
        0x03,
        fdb_kp >> 8,
        fdb_kp & 0xFF,
        fdb_kd >> 8,
        fdb_kd & 0xFF,
        0,
        0,
    ]


def get_motor_parameter(tx_msg: EtherCATMsg, passage: int, motor_id: int, param_cmd: int) -> None:
    slot = _slot(tx_msg, passage)
    tx_msg.can_ide = 0
    slot.rtr = 0
    slot.id = motor_id
    slot.dlc = 2
    slot.data = [0xE0, param_cmd, 0, 0, 0, 0, 0, 0]


def _handle_motor_config_message(motor_data: MotorMsg, motor_msg: ODMotorMsg) -> None:
    global motor_comm_fbd
    if motor_data.data[2] != 0x01:
        return
    d = motor_data.data
    if d[0] == 0xFF and d[1] == 0xFF:
        motor_comm_fbd.motor_id = d[3] << 8 | d[4]
        motor_comm_fbd.motor_fbd = 0x06
    elif d[0] == 0x80 and d[1] == 0x80:
        motor_comm_fbd.motor_id = 0
        motor_comm_fbd.motor_fbd = 0x80
    elif d[0] == 0x7F and d[1] == 0x7F:
        motor_comm_fbd.motor_id = 1
        motor_comm_fbd.motor_fbd = 0x05
    else:
        motor_comm_fbd.motor_id = d[0] << 8 | d[1]
        motor_comm_fbd.motor_fbd = d[3]


def _handle_response_mode(motor_data: MotorMsg, motor_msg: ODMotorMsg, ack_status: int) -> None:
    global motor_id_check, motor_comm_fbd
    motor_id_check = motor_data.id
    motor_msg.motor_id = motor_id_check
    motor_msg.error = motor_data.data[0] & 0x1F
    d = motor_data.data

    if ack_status == 1:
        pos_int = d[1] << 8 | d[2]
        spd_int = d[3] << 4 | (d[4] & 0xF0) >> 4
        cur_int = (d[4] & 0x0F) << 8 | d[5]
        motor_msg.angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16)
        motor_msg.speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12)
        motor_msg.current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12)
        motor_msg.temperature = (d[6] - 50) // 2
    elif ack_status == 2:
        motor_msg.angle_actual_float = struct.unpack("<f", bytes([d[4], d[3], d[2], d[1]]))[0]
        motor_msg.current_actual_int = d[5] << 8 | d[6]
        motor_msg.temperature = (d[7] - 50) // 2
        motor_msg.current_actual_float = motor_msg.current_actual_int / 100.0
    elif ack_status == 3:
        motor_msg.speed_actual_float = struct.unpack("<f", bytes([d[4], d[3], d[2], d[1]]))[0]
        motor_msg.current_actual_int = d[5] << 8 | d[6]
        motor_msg.temperature = (d[7] - 50) // 2
        motor_msg.current_actual_float = motor_msg.current_actual_int / 100.0
    elif ack_status == 4:
        if motor_data.dlc != 3:
            return
        motor_comm_fbd.ins_code = d[1]
        motor_comm_fbd.motor_fbd = d[2]
    elif ack_status == 5:
        motor_comm_fbd.ins_code = d[1]
        if motor_data.dlc == 6:
            if motor_comm_fbd.ins_code < 1 or motor_comm_fbd.ins_code > 4:
                return
            value = struct.unpack("<f", bytes([d[5], d[4], d[3], d[2]]))[0]
            if motor_comm_fbd.ins_code == 1:
                motor_msg.angle_actual_float = value
            elif motor_comm_fbd.ins_code == 2:
                motor_msg.speed_actual_float = value
            elif motor_comm_fbd.ins_code == 3:
                motor_msg.current_actual_float = value
            else:
                motor_msg.power = value
        elif motor_data.dlc == 4:
            if motor_comm_fbd.ins_code < 5 or motor_comm_fbd.ins_code > 9:
                return
            data = d[2] << 8 | d[3]
            if motor_comm_fbd.ins_code == 5:
                motor_msg.acceleration = data
            elif motor_comm_fbd.ins_code == 6:
                motor_msg.linkage_KP = data
            elif motor_comm_fbd.ins_code == 7:
                motor_msg.speed_KI = data
            elif motor_comm_fbd.ins_code == 8:
                motor_msg.feedback_KP = data
            else:
                motor_msg.feedback_KD = data


def _handle_automatic_mode(motor_data: MotorMsg, motor_msg: list[ODMotorMsg]) -> None:
    motor_id_t = motor_data.id - 0x205
    if not 0 <= motor_id_t < len(motor_msg):
        return
    msg = motor_msg[motor_id_t]
    d = motor_data.data
    msg.motor_id = motor_data.id
    msg.angle_actual_int = d[0] << 8 | d[1]
    msg.speed_actual_int = struct.unpack(">h", bytes([d[2], d[3]]))[0]
    msg.current_actual_int = d[4] << 8 | d[5]
    msg.temperature = d[6]
    msg.error = d[7]


def rv_can_data_repack(
    rx_msg: EtherCATMsg,
    comm_mode: Literal[0, 1],
    motor_msg: list[ODMotorMsg],
    slave: int = 0,
) -> None:
    """Parse gateway RX PDO into motor feedback structures (SOEM RV_can_data_repack)."""
    del slave
    for i in range(6):
        motor_data = rx_msg.motor[i]
        if motor_data.dlc == 0:
            continue
        if motor_data.id == 0x7FF:
            _handle_motor_config_message(motor_data, motor_msg[i])
        elif comm_mode == COMM_ACK:
            ack_status = motor_data.data[0] >> 5
            _handle_response_mode(motor_data, motor_msg[i], ack_status)
        elif comm_mode == COMM_AUTO:
            _handle_automatic_mode(motor_data, motor_msg)
