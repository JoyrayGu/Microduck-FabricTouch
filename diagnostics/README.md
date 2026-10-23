# microduck_skin_diagnostics

Read-only diagnostics for the Microduck tactile skin serial stream.

This project exists to answer questions the existing code only assumed:

1. What frame lengths actually arrive? The vendor log shows 2057 expected but
   frequent 2056 / 999 / 216 reads.
2. Is the array 32×32 or 64×64? The skin config says 32×32, the vendor viewer
   renders 64×64.
3. What is the real value range, rest noise, and per-frame rate?

## Findings

**Resolved: the array is 32×32, 16-bit big-endian.** An earlier version of this
project concluded 64×64 / 4-bit from an `observed_max` of 15. That inference was
wrong: on 1024 cells sampled over 200 frames, a value of 15 will appear by chance,
so it proves nothing about bit width. The byte-parity test settles it — in real
frames the even-index byte high nibble is non-zero 0 times out of 204800, while the
odd-index byte high nibble is non-zero 955 times. Packed 4-bit cells cannot produce
that asymmetry. See `data/calibration-findings.md` section 1.1.

The 2050-byte payload is `2 (frame counter) + 32*32*2`, and light taps peak around
230–263 in raw units, which is far above the 0–8 noise floor.

`config.example.json`'s `device_array: [32,32]` was therefore correct all along.

## Scope

- Opens a serial port and reads bytes. It writes nothing by default.
- `--send-start` is the only write path, and it sends exactly the two commands
  found in the vendor log:
  - start: `98 00 00 01 00 99 81`
  - stop: `98 00 00 03 00 9B 81`
- It never changes device configuration, thresholds, or acquisition mode.
- It does not import or modify `microduck_skin`.

## Usage

Close `SerialDataCollection.exe` first: it holds the port exclusively.

```powershell
$env:PYTHONPATH = "E:\OneDrive\MatNgine\microduck_skin_diagnostics\src"

# list candidates
E:\MiniConda\python.exe -m microduck_skin_diagnostics.cli --list-ports

# passive listen (device may stream nothing unless started)
E:\MiniConda\python.exe -m microduck_skin_diagnostics.cli --port COM7 --seconds 10

# start acquisition, capture 10 s, stop on exit
E:\MiniConda\python.exe -m microduck_skin_diagnostics.cli --port COM7 --seconds 10 `
  --send-start --json-out data/run1.json --save-frames data/run1.bin
```

## Output

Terminal summary:

```text
port=COM7 baud=9600 elapsed=10.00s
frames_ok=1180 discarded_bytes=214
frame_lengths={'2057': 1174, '2056': 6}
32x32_16bit_be: max=2698 active=9.1 centroid=[31.0, 30.5]
64x64_4bit_rejected: max=15 active=37.4 centroid=[31.2, 30.8]
frames_per_second=118.0
```

The decision rule: with a light single touch, the correct layout yields a small
contiguous cluster whose centroid moves smoothly as the finger moves. The wrong
one yields scattered cells or a jumping centroid. The `_rejected` suffix in the
layout name marks the hypothesis that the parity test ruled out.

## Tests

```powershell
$env:PYTHONPATH = "E:\OneDrive\MatNgine\microduck_skin_diagnostics\src"
E:\MiniConda\python.exe -m unittest discover -s tests -v
```

Tests cover frame sync across chunk boundaries, garbage rejection, both layout
sizes, and centroid behaviour. No hardware is touched by the tests.
