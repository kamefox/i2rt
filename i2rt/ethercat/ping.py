"""Ping DM motors through EtherCAT-CAN gateway PDO."""

from __future__ import annotations

import time
from dataclasses import asdict
from typing import List

import can

from i2rt.ethercat.pdo import PDO_SIZE, MOTOR_SLOTS, build_rx_pdo_single_motor, find_response_frame
from i2rt.ethercat.runtime import EtherCATRuntime
from i2rt.motor_drivers.dm_driver import DMSingleMotorCanInterface, MotorType, ReceiveMode

MOTOR_ON_DATA = bytes([0xFF] * 7 + [0xFC])
MOTOR_OFF_DATA = bytes([0xFF] * 7 + [0xFD])
PING_INTERVAL_MS_ALLOWED = frozenset({1, 2, 5, 10, 20, 50})
_EMPTY_PDO = bytes(PDO_SIZE)


def _parse_dm_feedback(data: bytes, can_id: int):
    iface = DMSingleMotorCanInterface.__new__(DMSingleMotorCanInterface)
    iface.receive_mode = ReceiveMode.p16
    iface.name = "ethercat"
    iface.bus = type("Bus", (), {"channel_info": "ethercat"})()
    msg = can.Message(arbitration_id=can_id, data=list(data[:8]), is_extended_id=False)
    return iface.parse_recv_message(msg, MotorType.DM4310, ignore_error=True)


def run_ping_motors_via_ethercat(
    runtime: EtherCATRuntime,
    motor_ids: List[int],
    ping_interval_ms: int = 0,
) -> dict:
    if not motor_ids:
        return {"ok": False, "error": "motor_ids 为空", "transport": "ethercat"}
    if not runtime.connected:
        return {"ok": False, "error": "EtherCAT 未连接或未进入 OP", "transport": "ethercat"}

    interval_s = 0.0
    if int(ping_interval_ms) in PING_INTERVAL_MS_ALLOWED:
        interval_s = int(ping_interval_ms) / 1000.0

    receive_mode = ReceiveMode.p16
    results = []
    online = []
    debug_rows = []

    for idx, mid in enumerate(motor_ids):
        row: dict = {"motor_id": mid, "passage": mid}
        expected_rx = receive_mode.get_receive_id(mid)
        dbg: dict = {"motor_id": mid, "passage": mid}
        try:
            if mid < 1 or mid > 6:
                raise ValueError(f"EtherCAT 网关仅支持 ID 1–6 对应 CAN 槽位，当前 motor_id={mid}")

            frame = None
            last_wkc = 0
            last_rx = b""
            tx = build_rx_pdo_single_motor(mid, MOTOR_ON_DATA)
            dbg["tx_pdo_hex"] = tx[:24].hex()

            for attempt in range(20):
                rx_bytes, last_wkc = runtime.exchange_pdo(tx, cycles=15)
                last_rx = rx_bytes
                frame = find_response_frame(rx_bytes, mid, expected_rx)
                if frame is not None and frame.dlc >= 1:
                    dbg["attempt"] = attempt + 1
                    break
                time.sleep(0.003)

            dbg["wkc"] = last_wkc
            dbg["expected_wkc"] = getattr(runtime, "_expected_wkc", 0)
            dbg["rx_pdo_hex"] = last_rx[:24].hex() if last_rx else ""

            if frame is None:
                raise AssertionError(
                    f"网关 TxPDO 无电机 {mid} 回传 (wkc={last_wkc}, expected={dbg['expected_wkc']})"
                )

            info = _parse_dm_feedback(bytes(frame.data[:8]), frame.can_id & 0x7FF)
            row["ok"] = True
            row["feedback"] = asdict(info)
            row["passage"] = mid
            row["wkc"] = last_wkc
            online.append(mid)
            runtime.exchange_pdo(build_rx_pdo_single_motor(mid, MOTOR_OFF_DATA), cycles=10)
            runtime.exchange_pdo(_EMPTY_PDO, cycles=5)
        except Exception as exc:
            row["ok"] = False
            row["error"] = str(exc)
            dbg["error"] = str(exc)
        results.append(row)
        debug_rows.append(dbg)
        if interval_s > 0 and idx < len(motor_ids) - 1:
            time.sleep(interval_s)

    return {
        "ok": len(online) > 0,
        "transport": "ethercat",
        "channel": runtime.iface,
        "online_motors": online,
        "results": results,
        "ping_motor_ids": list(motor_ids),
        "ping_interval_ms": int(ping_interval_ms) if interval_s > 0 else 0,
        "pdo_bytes": getattr(runtime, "_pdo_size", PDO_SIZE),
        "expected_wkc": getattr(runtime, "_expected_wkc", 0),
        "debug": debug_rows,
        "hint": "EtherCAT Ping 走网关 CAN1/CAN2，PC 上 can0 抓不到；请用示波器/USB-CAN 接网关 CAN 口",
    }
