"""Send one gesture event to brain and print the reply.

Used to verify the Windows -> Pi link without any tactile device.
"""
from __future__ import annotations

import argparse
import json
import socket
import time
import uuid


def send(host: str, port: int, gesture: str, timeout: float = 5.0) -> dict:
    event = {
        "version": 1,
        "event_id": uuid.uuid4().hex,
        "gesture": gesture,
        "timestamp_ms": int(time.time() * 1000),
        "confidence": 1.0,
    }
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(event) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(1024)
            if not chunk:
                break
            data += chunk
    text = data.decode("utf-8", errors="replace").strip()
    return {"sent": event, "reply_raw": text}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.57")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--gesture", default="tap",
                        choices=["tap", "single_stroke", "back_forth_stroke"])
    args = parser.parse_args()
    result = send(args.host, args.port, args.gesture)
    print("SENT:", json.dumps(result["sent"], ensure_ascii=False))
    print("REPLY:", result["reply_raw"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
