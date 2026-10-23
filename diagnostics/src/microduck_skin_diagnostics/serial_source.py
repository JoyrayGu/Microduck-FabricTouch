"""Serial transport for diagnostics.

Read-only by default: opening the port writes nothing. The two known vendor
commands are sent only when explicitly requested with --send-start, and are
byte-for-byte the values observed in the vendor log:

    start: 98 00 00 01 00 99 81
    stop : 98 00 00 03 00 9B 81
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

try:  # pyserial is an explicit dependency, but importing must not hard-fail
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - reported by the CLI instead
    serial = None
    list_ports = None

START_COMMAND = bytes.fromhex("98 00 00 01 00 99 81")
STOP_COMMAND = bytes.fromhex("98 00 00 03 00 9B 81")


class SerialUnavailable(RuntimeError):
    """pyserial is not installed."""


@dataclass
class PortInfo:
    device: str
    description: str
    vid: int | None
    pid: int | None

    def __str__(self) -> str:
        vid = f"{self.vid:04X}" if self.vid else "----"
        pid = f"{self.pid:04X}" if self.pid else "----"
        # Windows returns descriptions in the ANSI code page. Keep the line
        # printable from any console encoding instead of raising on output.
        desc = self.description.encode("ascii", "replace").decode("ascii")
        return f"{self.device}  {desc}  [{vid}:{pid}]"


def list_serial_ports() -> list[PortInfo]:
    if list_ports is None:
        raise SerialUnavailable("pyserial is not installed")
    return [
        PortInfo(p.device, p.description, p.vid, p.pid)
        for p in list_ports.comports()
    ]


class SerialSource:
    """Blocking reads from one port. Never writes unless asked."""

    def __init__(self, port: str, baud: int, read_timeout: float = 0.2) -> None:
        if serial is None:
            raise SerialUnavailable("pyserial is not installed")
        self.port = port
        self.baud = baud
        self._timeout = read_timeout
        self._ser = None

    def __enter__(self) -> "SerialSource":
        self._ser = serial.Serial(self.port, self.baud, timeout=self._timeout)
        return self

    def __exit__(self, *exc_info) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None

    def send_start(self) -> None:
        self._write(START_COMMAND)

    def send_stop(self) -> None:
        self._write(STOP_COMMAND)

    def _write(self, data: bytes) -> None:
        if self._ser is None:
            raise RuntimeError("port is not open")
        self._ser.write(data)
        self._ser.flush()

    def read(self, size: int = 4096) -> bytes:
        """Return up to `size` bytes, or b"" after the port timeout elapses.

        pyserial's read(n) already waits n bytes (bounded by `timeout`), so a
        single 1-byte-blocking probe is used to keep the caller's deadline
        responsive instead of blocking for a full frame.
        """
        if self._ser is None:
            raise RuntimeError("port is not open")
        waiting = getattr(self._ser, "in_waiting", 0) or 0
        if waiting <= 0:
            first = self._ser.read(1)
            if not first:
                return b""
            waiting = max(1, (getattr(self._ser, "in_waiting", 0) or 0) + 1)
            return first + self._ser.read(min(max(0, size - 1), waiting - 1))[: max(0, size - 1)]
        return self._ser.read(min(size, waiting))

    def chunks(self, size: int = 4096) -> Iterator[bytes]:
        while True:
            data = self.read(size)
            if data:
                yield data
            else:
                return
