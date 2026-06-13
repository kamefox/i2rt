from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_REL_SOEM_C = Path("I2RT_CODE/EtherCat/soem_demo_c/build/app/master_stack_test")
_REL_SOEM_CPP = Path(
    "EtherCat教程/SOEM_Master例程/soem_demo_cpp/soem_demo_cpp/app_cpp/build/master_stack_test"
)


def _effective_home() -> Path:
    sudo_user = (os.environ.get("SUDO_USER") or "").strip()
    if sudo_user:
        try:
            import pwd

            return Path(pwd.getpwnam(sudo_user).pw_dir)
        except (ImportError, KeyError):
            pass
    return Path.home()


def resolve_soem_master_bin() -> Path:
    """Locate SOEM binary; sudo 下 Path.home() 为 /root，需用 SUDO_USER 家目录。"""
    env = (os.environ.get("I2RT_SOEM_MASTER_BIN") or "").strip()
    if env:
        return Path(env)
    home = _effective_home()
    candidates = [
        home / _REL_SOEM_CPP,
        Path("/home/i2rt") / _REL_SOEM_CPP,
        home / _REL_SOEM_C,
        Path.home() / _REL_SOEM_C,
        Path("/home/i2rt") / _REL_SOEM_C,
    ]
    seen: set[str] = set()
    for path in candidates:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if path.is_file():
            return path
    return candidates[0]


DEFAULT_SOEM_MASTER_BIN = str(resolve_soem_master_bin())


@dataclass
class EtherCATScanResult:
    iface: str
    slave_count: int
    operational: bool
    raw_log: str


@dataclass
class EthernetInterfaceInfo:
    name: str
    up: bool
    carrier: bool
    promisc: bool
    mac: str


def list_ethernet_interfaces() -> list[str]:
    """Return Linux ethernet interface names suitable for EtherCAT (en*, eth*)."""
    return [x.name for x in list_ethernet_interface_details()]


def list_ethernet_interface_details() -> list[EthernetInterfaceInfo]:
    """Return EtherCAT-compatible ethernet interfaces with link state."""
    net_dir = Path("/sys/class/net")
    if not net_dir.is_dir():
        return []
    out: list[EthernetInterfaceInfo] = []
    for entry in sorted(net_dir.iterdir()):
        name = entry.name
        if not (name.startswith("en") or name.startswith("eth")):
            continue
        if not (entry / "device").exists():
            continue
        flags = _read_text(entry / "flags")
        operstate = _read_text(entry / "operstate").lower()
        carrier = _read_text(entry / "carrier") == "1"
        mac = _read_text(entry / "address")
        promisc = bool(flags and int(flags, 0) & 0x100)
        up = operstate not in ("down", "notpresent", "")
        out.append(
            EthernetInterfaceInfo(
                name=name,
                up=up,
                carrier=carrier,
                promisc=promisc,
                mac=mac,
            )
        )
    return out


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def _master_cmd(master_bin: Path, iface: str) -> list[str]:
    cmd = [str(master_bin)]
    # C 版支持 argv 网卡；C++ 版硬编码 enp99s0，多传参数无害
    if "soem_demo_c" in str(master_bin):
        cmd.append(iface)
    if shutil.which("stdbuf"):
        cmd = ["stdbuf", "-oL", "-eL", *cmd]
    return cmd


def _parse_slave_count(raw: str) -> Optional[int]:
    for pattern in (
        r"(\d+)\s+slaves found and configured",
        r"从站数量[：:]\s*(\d+)",
    ):
        match = re.search(pattern, raw)
        if match:
            return int(match.group(1))
    return None


def _scan_done_signal(raw: str) -> bool:
    if "未找到从站" in raw:
        return True
    if "Failed to initialize EtherCAT" in raw:
        return True
    if _parse_slave_count(raw) is not None:
        return True
    return False


def soem_binary_has_raw_cap(master_bin: Path) -> bool:
    try:
        proc = subprocess.run(
            ["getcap", str(master_bin)],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return False
    return "cap_net_raw" in (proc.stdout or "")


def ethercat_scan_allowed() -> bool:
    master_bin = Path(DEFAULT_SOEM_MASTER_BIN)
    return os.geteuid() == 0 or soem_binary_has_raw_cap(master_bin)


def _terminate_proc(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)


class EtherCATMaster:
    """Thin wrapper around the SOEM C demo binary for scan / init checks."""

    def __init__(
        self,
        iface: str,
        master_bin: str | Path = DEFAULT_SOEM_MASTER_BIN,
        scan_timeout_s: float = 20.0,
    ):
        self.iface = iface
        self.master_bin = Path(master_bin)
        self.scan_timeout_s = scan_timeout_s
        self._last_scan: Optional[EtherCATScanResult] = None

    @property
    def last_scan(self) -> Optional[EtherCATScanResult]:
        return self._last_scan

    def scan_slaves(self) -> EtherCATScanResult:
        """Run SOEM master, read output line-by-line, stop once slave count is known."""
        if not self.master_bin.is_file():
            raise FileNotFoundError(f"SOEM master binary not found: {self.master_bin}")
        cmd = _master_cmd(self.master_bin, self.iface)
        logger.info("EtherCAT scan on %s via %s cmd=%s", self.iface, self.master_bin, cmd)
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        chunks: list[str] = []
        deadline = time.monotonic() + self.scan_timeout_s
        try:
            assert proc.stdout is not None
            while time.monotonic() < deadline:
                line = proc.stdout.readline()
                if line:
                    chunks.append(line)
                    if _scan_done_signal("".join(chunks)):
                        break
                    continue
                if proc.poll() is not None:
                    break
                time.sleep(0.05)
        finally:
            _terminate_proc(proc)
            if proc.stdout is not None:
                try:
                    rest = proc.stdout.read()
                    if rest:
                        chunks.append(rest)
                except Exception:
                    pass

        raw = "".join(chunks)
        if not raw.strip() and proc.returncode not in (None, 0, -15):
            raw = f"SOEM master exited with code {proc.returncode}"
        return self._store_scan(raw)

    def _store_scan(self, raw: str, timed_out: bool = False) -> EtherCATScanResult:
        slave_count = _parse_slave_count(raw) or 0
        operational = (
            "Operational state reached" in raw
            or "successfully initialized" in raw
            or slave_count > 0
        )
        if timed_out and slave_count > 0:
            operational = True
        self._last_scan = EtherCATScanResult(
            iface=self.iface,
            slave_count=slave_count,
            operational=operational and slave_count > 0,
            raw_log=raw.strip(),
        )
        return self._last_scan
