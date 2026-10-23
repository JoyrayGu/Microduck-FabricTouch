"""CLI entry point for read-only tactile serial diagnostics."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from . import frame_parser as fp
from .serial_source import (
    START_COMMAND,
    STOP_COMMAND,
    SerialSource,
    SerialUnavailable,
    list_serial_ports,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only diagnostics for the Microduck tactile skin serial stream.",
        epilog=(
            "Default mode only reads. Nothing is written to the device unless "
            "--send-start is passed, in which case the two commands observed in the "
            "vendor log are sent (start on entry, stop on exit)."
        ),
    )
    parser.add_argument("--port", help="serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--seconds", type=float, default=10.0, help="capture duration")
    parser.add_argument("--threshold", type=int, default=1,
                        help="cell value counted as active (default 1: report anything above rest)")
    parser.add_argument("--send-start", action="store_true",
                        help="send the vendor start command on entry and stop on exit")
    parser.add_argument("--save-frames", type=Path,
                        help="write raw frames to this .bin file")
    parser.add_argument("--save-limit", type=int, default=0,
                        help="max frames to save; 0 = no limit (default)")
    parser.add_argument("--json-out", type=Path, help="write the summary as JSON")
    parser.add_argument("--list-ports", action="store_true")
    return parser


def run(args: argparse.Namespace) -> int:
    if args.list_ports:
        try:
            ports = list_serial_ports()
        except SerialUnavailable as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        if not ports:
            print("no serial ports found")
        for port in ports:
            print(port)
        return 0

    if not args.port:
        print("ERROR: --port is required (use --list-ports to see candidates)", file=sys.stderr)
        return 2

    sync = fp.FrameSynchronizer()
    frames: list[fp.Frame] = []
    started = time.monotonic()

    try:
        with SerialSource(args.port, args.baud) as source:
            if args.send_start:
                source.send_start()
                print(f"sent start command: {START_COMMAND.hex(' ')}")
            idle_polls = 0
            try:
                while time.monotonic() - started < args.seconds:
                    chunk = source.read()
                    if not chunk:
                        idle_polls += 1
                        time.sleep(0.01)
                        continue
                    frames.extend(sync.feed(chunk))
            finally:
                if args.send_start:
                    try:
                        source.send_stop()
                        print(f"sent stop command: {STOP_COMMAND.hex(' ')}")
                    except Exception as exc:  # noqa: BLE001
                        print(f"warning: could not send stop command: {exc}", file=sys.stderr)
    except SerialUnavailable as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    elapsed = time.monotonic() - started
    summary = fp.summarize(frames, args.threshold)
    summary["port"] = args.port
    summary["baud"] = args.baud
    summary["seconds"] = round(elapsed, 2)
    summary["scan"] = sync.stats.as_dict()
    summary["frames_per_second"] = round(len(frames) / elapsed, 2) if elapsed > 0 else 0.0

    print(f"port={args.port} baud={args.baud} elapsed={elapsed:.2f}s")
    print(f"frames_ok={sync.stats.frames} discarded_bytes={sync.stats.discarded_bytes} bytes_in={sync.stats.bytes_in}")
    if sync.stats.bytes_in == 0:
        print("NO_DATA: device streamed nothing. Try --send-start to request acquisition.")
    hist = summary["scan"]["length_histogram"]
    print(f"frame_lengths={hist}")
    for name, block in summary.get("layouts", {}).items():
        print(
            f"{name}: max={block['observed_max']} "
            f"active={block['mean_active_cells']} centroid={block['mean_centroid']}"
        )
    print(f"frames_per_second={summary['frames_per_second']}")

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"JSON_SAVED {args.json_out}")

    if args.save_frames and frames:
        args.save_frames.parent.mkdir(parents=True, exist_ok=True)
        limit = len(frames) if args.save_limit <= 0 else min(len(frames), args.save_limit)
        with args.save_frames.open("wb") as handle:
            for frame in frames[:limit]:
                handle.write(frame.raw)
        print(f"FRAMES_SAVED {limit}/{len(frames)} -> {args.save_frames}")
        if limit < len(frames):
            print("WARNING: --save-limit truncated the capture before analysis.")

    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
