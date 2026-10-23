#!/usr/bin/env python3
"""Run a command on or upload a file to the Microduck Raspberry Pi.

Usage:
  pi_ssh.py run "shell command"          stream output of the command
  pi_ssh.py upload <local> <remote>      upload one file via SFTP

The SSH password is read from the PI_PASSWORD environment variable and is
never written to disk.
"""

from __future__ import annotations

import os
import sys
import time

import paramiko

HOST = "192.168.1.57"
USER = "pi4"


def connect() -> paramiko.SSHClient:
    password = os.environ.get("PI_PASSWORD")
    if not password:
        raise SystemExit("PI_PASSWORD environment variable is not set")
    last_error: Exception | None = None
    for attempt in range(1, 4):
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                HOST,
                username=USER,
                password=password,
                look_for_keys=False,
                allow_agent=False,
                timeout=10,
            )
            if attempt > 1:
                print(f"[pi_ssh] connected on attempt {attempt}", flush=True)
            return client
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            print(f"[pi_ssh] attempt {attempt} failed: {exc}", flush=True)
            time.sleep(4)
    raise SystemExit(f"cannot connect to {HOST}: {last_error}")


def main() -> int:
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    mode = sys.argv[1]
    client = connect()
    try:
        if mode == "run":
            command = sys.argv[2]
            transport = client.get_transport()
            channel = transport.open_session()
            channel.set_combine_stderr(True)
            channel.exec_command(command)
            deadline = time.time() + 600
            output = []
            while True:
                while channel.recv_ready():
                    output.append(channel.recv(4096).decode("utf-8", "replace"))
                    sys.stdout.write(output[-1])
                    sys.stdout.flush()
                if channel.exit_status_ready() and not channel.recv_ready():
                    break
                if time.time() > deadline:
                    raise SystemExit("remote command timed out after 600s")
                time.sleep(0.05)
            return channel.recv_exit_status()
        if mode == "upload":
            local, remote = sys.argv[2], sys.argv[3]
            sftp = client.open_sftp()
            sftp.put(local, remote)
            sftp.close()
            print(f"UPLOADED {local} -> {remote}")
            return 0
        raise SystemExit(f"unknown mode: {mode}")
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
