"""Long-lived EtherCAT master (pysoem) for cyclic PDO exchange."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import pysoem
except ImportError:  # pragma: no cover
    pysoem = None  # type: ignore

from i2rt.ethercat.pdo import PDO_SIZE

_OP_RETRY = 40
_STATE_TIMEOUT_US = 50_000
_PDO_TIMEOUT_US = 2000


def _format_slave_states(master: "pysoem.Master", prefix: str) -> str:
    lines = [prefix]
    try:
        master.read_state()
    except Exception:
        pass
    try:
        lines.append(f"master.state=0x{int(master.state):02x}")
    except Exception:
        pass
    for i, slave in enumerate(master.slaves):
        try:
            al = slave.al_status
        except Exception:
            al = "?"
        lines.append(
            f"slave[{i}] id={slave.id} {slave.name} state=0x{int(slave.state):02x} al_status={al}"
        )
    return "\n".join(lines)


def _zero_slave_outputs(master: "pysoem.Master") -> None:
    for slave in master.slaves:
        size = len(slave.output or b"")
        if size > 0:
            slave.output = bytes(size)


def _request_operational(master: "pysoem.Master") -> None:
    """Match SOEM C++ demo: PDO exchange before/during OP transition."""
    master.state = pysoem.OP_STATE
    master.send_processdata()
    master.receive_processdata(_PDO_TIMEOUT_US)
    master.write_state()
    for _ in range(_OP_RETRY):
        master.send_processdata()
        master.receive_processdata(_PDO_TIMEOUT_US)
        if master.state_check(pysoem.OP_STATE, _STATE_TIMEOUT_US) == pysoem.OP_STATE:
            return
    master.read_state()


class EtherCATRuntime:
    """Keep slaves in OP and exchange process data until stop()."""

    def __init__(self) -> None:
        self.iface = ""
        self.slave_count = 0
        self._master: Optional["pysoem.Master"] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._pdo_size = 0
        self._expected_wkc = 0
        self.last_diag = ""
        self._can_test_active = False
        self._can_test_payload = b""

    @property
    def can_test_active(self) -> bool:
        return self._can_test_active

    @property
    def connected(self) -> bool:
        return self._master is not None and self._running

    def start(self, iface: str) -> dict:
        if pysoem is None:
            raise ImportError("未安装 pysoem，请执行: pip install pysoem")
        iface = (iface or "").strip()
        if not iface:
            raise ValueError("网口名为空")

        master = pysoem.Master()
        try:
            master.open(iface)
            slave_count = master.config_init()
            if slave_count <= 0:
                raise RuntimeError(f"未发现 EtherCAT 从站（网口 {iface}）")

            master.config_map()
            try:
                master.config_dc()
            except Exception as exc:
                logger.warning("config_dc skipped: %s", exc)

            if master.state_check(pysoem.SAFEOP_STATE, _STATE_TIMEOUT_US) != pysoem.SAFEOP_STATE:
                self.last_diag = _format_slave_states(master, "未能进入 SafeOP")
                raise RuntimeError(self.last_diag)

            _zero_slave_outputs(master)
            _request_operational(master)

            master.read_state()
            if master.state != pysoem.OP_STATE:
                self.last_diag = _format_slave_states(master, "从站未能进入 Operational 状态")
                raise RuntimeError(self.last_diag)

            self._pdo_size = len(master.slaves[0].output or b"")
            if self._pdo_size < PDO_SIZE:
                self._pdo_size = PDO_SIZE
            self._expected_wkc = int(getattr(master, "expected_wkc", 0) or 0)
            self._master = master
            self.iface = iface
            self.slave_count = int(slave_count)
            self._running = True
            self._thread = threading.Thread(target=self._cycle_loop, name="ethercat-pdo", daemon=True)
            self._thread.start()

            logger.info(
                "EtherCAT runtime OP on %s, slaves=%d, pdo_out=%d",
                iface,
                slave_count,
                self._pdo_size,
            )
            return {
                "iface": iface,
                "slave_count": self.slave_count,
                "pdo_bytes": self._pdo_size,
            }
        except Exception:
            try:
                master.state = pysoem.INIT_STATE
                master.write_state()
            except Exception:
                pass
            try:
                master.close()
            except Exception:
                pass
            raise

    def stop(self) -> None:
        self._can_test_active = False
        self._can_test_payload = b""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        with self._lock:
            master = self._master
            self._master = None
        if master is not None:
            try:
                master.state = pysoem.INIT_STATE
                master.write_state()
            except Exception:
                pass
            try:
                master.close()
            except Exception:
                pass
        self.iface = ""
        self.slave_count = 0
        self._pdo_size = 0
        self._expected_wkc = 0

    def _fit_output(self, output: bytes) -> bytes:
        size = self._pdo_size if self._pdo_size > 0 else PDO_SIZE
        return bytes(output)[:size].ljust(size, b"\x00")

    def _cycle_loop(self) -> None:
        while self._running:
            with self._lock:
                master = self._master
                if master is None:
                    break
                try:
                    if self._can_test_active and self._can_test_payload:
                        master.slaves[0].output = self._can_test_payload
                    master.send_processdata()
                    master.receive_processdata(_PDO_TIMEOUT_US)
                except Exception as exc:
                    logger.warning("EtherCAT PDO cycle error: %s", exc)
            time.sleep(0.001)

    def start_can_test(
        self,
        can_id: int = 1,
        data: bytes | None = None,
    ) -> dict:
        if not self._master:
            raise RuntimeError("EtherCAT 未连接")
        from i2rt.ethercat.pdo import build_rx_pdo_single_motor

        payload = bytes(data) if data is not None else bytes(range(8))
        if len(payload) < 1 or len(payload) > 8:
            raise ValueError("CAN 数据长度须为 1~8 字节")
        self._can_test_payload = self._fit_output(
            build_rx_pdo_single_motor(int(can_id), payload, slot=int(can_id) - 1)
        )
        self._can_test_active = True
        return {
            "can_id": int(can_id) & 0x7FF,
            "dlc": len(payload),
            "data": list(payload),
            "pdo_hex": self._can_test_payload[:24].hex(),
        }

    def stop_can_test(self) -> None:
        self._can_test_active = False
        self._can_test_payload = b""
        with self._lock:
            master = self._master
            if master is not None and master.slaves:
                master.slaves[0].output = self._fit_output(b"")

    def exchange_pdo(self, output: bytes, cycles: int = 20) -> tuple[bytes, int]:
        """Write RxPDO, cycle process data, return (TxPDO, last_wkc)."""
        if not self._master:
            raise RuntimeError("EtherCAT 未连接")
        payload = self._fit_output(output)
        last_wkc = 0
        with self._lock:
            slave = self._master.slaves[0]
            for _ in range(max(1, cycles)):
                slave.output = payload
                self._master.send_processdata()
                last_wkc = int(self._master.receive_processdata(_PDO_TIMEOUT_US) or 0)
            return bytes(slave.input or b""), last_wkc
