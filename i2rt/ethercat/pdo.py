"""Pack/unpack CAN-over-EtherCAT PDO (86 bytes, 6 motor slots)."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

MOTOR_SLOTS = 6
MOTOR_BLOCK_SIZE = 14
PDO_HEADER_SIZE = 2
PDO_SIZE = PDO_HEADER_SIZE + MOTOR_SLOTS * MOTOR_BLOCK_SIZE  # 86


@dataclass
class GatewayCANFrame:
    can_id: int = 0
    rtr: int = 0
    dlc: int = 0
    data: bytes = field(default_factory=lambda: bytes(8))


@dataclass
class GatewayPDO:
    motor_num: int = 0
    can_ide: int = 0
    motors: list[GatewayCANFrame] = field(
        default_factory=lambda: [GatewayCANFrame() for _ in range(MOTOR_SLOTS)]
    )


def _empty_motors() -> list[GatewayCANFrame]:
    return [GatewayCANFrame() for _ in range(MOTOR_SLOTS)]


def pack_rx_pdo(msg: GatewayPDO) -> bytes:
    """Master → slave RxPDO (object 0x7000)."""
    buf = bytearray(PDO_SIZE)
    buf[0] = msg.motor_num & 0xFF
    buf[1] = msg.can_ide & 0xFF
    for i, motor in enumerate(msg.motors[:MOTOR_SLOTS]):
        off = PDO_HEADER_SIZE + i * MOTOR_BLOCK_SIZE
        struct.pack_into("<IBB", buf, off, motor.can_id & 0xFFFFFFFF, motor.rtr & 0xFF, motor.dlc & 0xFF)
        chunk = motor.data[:8]
        buf[off + 6 : off + 6 + len(chunk)] = chunk
    return bytes(buf)


def unpack_tx_pdo(data: bytes) -> GatewayPDO:
    """Slave → master TxPDO (object 0x6000)."""
    raw = data[:PDO_SIZE].ljust(PDO_SIZE, b"\x00")
    msg = GatewayPDO(
        motor_num=raw[0],
        can_ide=raw[1],
        motors=_empty_motors(),
    )
    for i in range(MOTOR_SLOTS):
        off = PDO_HEADER_SIZE + i * MOTOR_BLOCK_SIZE
        can_id, rtr, dlc = struct.unpack_from("<IBB", raw, off)
        msg.motors[i] = GatewayCANFrame(
            can_id=can_id,
            rtr=rtr,
            dlc=dlc,
            data=bytes(raw[off + 6 : off + 14]),
        )
    return msg


def build_rx_pdo_single_motor(motor_id: int, data: bytes, slot: int | None = None) -> bytes:
    """Put one standard CAN frame in gateway slot (passage = slot+1, 默认 passage=motor_id)."""
    if slot is None:
        slot = int(motor_id) - 1
    msg = GatewayPDO(motor_num=MOTOR_SLOTS, can_ide=0, motors=_empty_motors())
    if not 0 <= slot < MOTOR_SLOTS:
        raise ValueError(f"motor_id {motor_id} 超出网关 {MOTOR_SLOTS} 路 CAN 槽位")
    msg.motors[slot] = GatewayCANFrame(
        can_id=motor_id & 0x7FF,
        rtr=0,
        dlc=min(8, max(1, len(data))),
        data=bytes(data[:8]).ljust(8, b"\x00"),
    )
    return pack_rx_pdo(msg)


def find_response_frame(
    tx_pdo: bytes,
    motor_id: int,
    expected_rx_id: int | None = None,
) -> GatewayCANFrame | None:
    """Find first motor slot in TxPDO with a CAN reply matching the motor."""
    inp = unpack_tx_pdo(tx_pdo)
    targets = {motor_id & 0x7FF}
    if expected_rx_id is not None:
        targets.add(expected_rx_id & 0x7FF)
    for motor in inp.motors:
        if motor.dlc <= 0:
            continue
        if (motor.can_id & 0x7FF) in targets:
            return motor
    return None
