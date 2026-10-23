"""Explicit, read-only SSH commands for the prepared Raspberry Pi.

This module never stores or accepts a password and never invokes an action
script. It is intentionally a thin Windows-side adapter around OpenSSH.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Sequence


@dataclass(frozen=True)
class PiConnection:
    host: str = "192.168.1.57"
    user: str = "pi4"
    port: int = 22
    identity_file: Path = Path.home() / ".ssh" / "microduck_ed25519"


READONLY_COMMANDS = {
    "identity": "whoami; hostname; hostname -I",
    "files": "ls -la /home/pi4/microduck-replica-next/tools/deployment_validation",
    "status": "cd /home/pi4/microduck-replica-next/tools/deployment_validation && /home/pi4/microduck_venv/bin/python servo_status.py",
    "processes": "ps -ef | grep -E 'return_to_stand|move_to_pose|pose_loop|squat_stand' | grep -v grep || true",
    "ports": "ss -ltnp | grep ':9000' || true",
    "calibration_readme": "sed -n '1,220p' /home/pi4/microduck-replica-next/tools/deployment_validation/README.md",
    "constants_source": "python3 -c 'import pathlib; p=pathlib.Path(\"/home/pi4/microduck-replica-next/microduck/src/constants.py\"); print(\"\\n\".join(str(i)+\": \"+line.encode(\"ascii\", \"backslashreplace\").decode() for i,line in enumerate(p.read_text(encoding=\"utf-8\").splitlines()[:125],1)))'",
    "guided_calibration_source": "python3 -c 'import pathlib; p=pathlib.Path(\"/home/pi4/microduck-replica-next/tools/deployment_validation/guided_joint_calibration.py\"); print(\"\\n\".join(str(i)+\": \"+line.encode(\"ascii\", \"backslashreplace\").decode() for i,line in enumerate(p.read_text(encoding=\"utf-8\").splitlines()[:280],1)))'",
    "direction_probe_source": "python3 -c 'import pathlib; p=pathlib.Path(\"/home/pi4/microduck-replica-next/tools/deployment_validation/pitch_direction_probe.py\"); print(\"\\n\".join(str(i)+\": \"+line.encode(\"ascii\", \"backslashreplace\").decode() for i,line in enumerate(p.read_text(encoding=\"utf-8\").splitlines()[:240],1)))'",
    "move_pose_source": "python3 -c 'import pathlib; p=pathlib.Path(\"/home/pi4/microduck-replica-next/tools/deployment_validation/move_to_pose.py\"); print(\"\\n\".join(str(i)+\": \"+line.encode(\"ascii\", \"backslashreplace\").decode() for i,line in enumerate(p.read_text(encoding=\"utf-8\").splitlines()[:190],1)))'",
}


class ReadOnlySshClient:
    def __init__(self, connection: PiConnection = PiConnection()) -> None:
        self.connection = connection

    def command(self, name: str) -> list[str]:
        if name not in READONLY_COMMANDS:
            raise ValueError(f"unsupported read-only command: {name}")
        target = f"{self.connection.user}@{self.connection.host}"
        return [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            "-i",
            str(self.connection.identity_file),
            "-p",
            str(self.connection.port),
            target,
            READONLY_COMMANDS[name],
        ]

    def run(self, name: str, *, timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            self.command(name),
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )

    def run_many(self, names: Sequence[str]) -> dict[str, subprocess.CompletedProcess[str]]:
        return {name: self.run(name) for name in names}
