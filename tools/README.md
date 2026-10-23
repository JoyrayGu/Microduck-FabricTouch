# microduck_behavior_control

Windows-side, offline-first pose planning for the 15-servo Microduck.

## Current scope

- Hardware mapping follows the project table: IDs 1-5 left leg, 6-10 right leg, 11-14 head/neck, 15 optional mouth.
- Official Microduck home angles are model references only.
- Calibration is required before angle-to-tick conversion.
- The mouth is represented but excluded from standing by default.
- `FakeBus` supports tests without connecting to Raspberry Pi or writing servos.
- No real stand pose has been calibrated yet; `StandController.simulate` accepts explicit test targets and is limited to `FakeBus`.
- `config/calibration_template.json` records all 15 IDs, with unknown calibration fields deliberately unset. The loader rejects schema/mapping errors and refuses stand target generation until all required calibration records are validated.
- `dry_run.py` prints a calibration table and reports `STAND_PLAN=REFUSED` until the required calibration is complete.
- `ssh_readonly.py` defines an explicit, password-free, batch-mode SSH adapter for identity, file, process, port, and servo-status reads only.
- `snapshot.py` saves Pi `servo_status.py` output locally, keyed only by physical IDs 1-14; it does not infer joint names and marks ID 15 as unqueried.
- `data/calibration/direction_observations.json` records manual torque-off direction observations per physical ID. These are raw tick directions only, not validated stand targets.

This project does not replace the Raspberry Pi executor. No SSH or hardware command is performed by these modules. The existing Pi `return_to_stand.py` uses a 14-servo calibration file, so a future integration must resolve ID 15 separately and must not upload guessed joint targets.

## Safety invariant

A future real adapter must read current positions, write current goals, verify them, enable torque, and only then interpolate with feedback. Incomplete calibration, excessive movement, missing feedback, or a communication error must abort and release torque under external support. The existing Pi scripts do not yet provide all these guarantees; the fake-bus simulation is not a hardware safety certification.

## Offline usage

From this directory, set `PYTHONPATH=src` and inspect the template with `python -m microduck_behavior_control.dry_run config/calibration_template.json`. It will refuse to produce a stand plan while calibration is incomplete. Run tests with `python -m unittest discover -s tests -v`. No third-party dependencies are required for the offline modules.

The SSH adapter uses the local private key `~/.ssh/microduck_ed25519` with `BatchMode=yes`, so it fails rather than prompting for a password. Run the Pi read-only inspection with `PYTHONPATH=src python -m microduck_behavior_control.pi_inspect`. Capture a local physical-ID snapshot with `PYTHONPATH=src python -m microduck_behavior_control.snapshot --output data/snapshots/current.json`. The snapshot command does not upload files, enable torque, or send goals. The Pi status script currently reports IDs 1-14 only; ID 15 remains unqueried. SSH support cannot start actions or upload files.
