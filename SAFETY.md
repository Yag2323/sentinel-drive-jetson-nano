# Safety Contract

This repository controls a wheeled prototype capable of unexpected motion.
Source availability is not authorization to energize it.

## Snapshot state

- `integration_gates.json` is fail-closed: `physical_motion_authorized=false`
  and `authorization_integrity_status=LOCKED`.
- `controller_config.json` contains software initial values, not physically
  tuned gains.
- `ipm_config.json` contains an SITL seed, not a physical camera calibration.
- The recovered `manual_motor_control.py` exposes the PCA9685/INA219 adapter,
  but no current gate is accepted by the fail-closed public seed.
- Retained prior physical evidence is summarized in the evidence register; it
  does not authorize this source identity.
- A continuous single-line circle lap is **PROVISIONAL / NOT VERIFIED**.

Do not run a physical motor test from this public snapshot.

## Non-negotiable physical controls

Before any future physical validation:

1. Place the robot on a flat, barrier-bounded test area with no stairs,
   drop-offs, traffic, people or fragile property.
2. Keep the motor battery physically disconnected during software
   initialization and every repositioning step.
3. Use a second person as a spotter, with immediate access to an independent
   motor-battery disconnect for the entire powered test.
4. Raise both driven wheels for the first hardware-adapter validation.
5. Use short, automatically bounded pulses before increasing duration.
6. Stop immediately for unexpected direction, mechanical binding, loose
   wiring, overheating, smoke, abnormal noise or voltage faults.
7. Never hold or stall an energized wheel by hand.
8. Never bypass a validation gate, stale-data check, voltage guard, command
   lease, watchdog or run-time confirmation.

## Configured software limits in this snapshot

These are file facts, not proof of physical effectiveness:

| Guard | Configured value | Trigger behavior |
|---|---:|---|
| Camera observation stale timeout | `0.25 s` | Supervisor refuses/withdraws drive authority. |
| YOLO result stale timeout | `0.75 s` | Obstacle state is not treated as current. |
| Maximum YOLO frame lag | `8 frames` | Result is rejected as too old. |
| Motor command lease | `0.35 s` | Guarded motor output expires and commands OFF. |
| Motor-bus stop voltage | `6.6 V` | Physical output is stopped/refused. |
| Maximum accepted motor voltage | `8.6 V` | Out-of-range supply is refused. |
| Lane confidence minimum | `0.55` | Low-confidence lane state cannot authorize normal drive. |
| Partial-lane motion | `false` | Partial lane state does not authorize motion in the seed config. |

`safe_motor_output.py`, `validation_manager.py` and `track_run.py` are the
authoritative implementation sources. The software lease cannot replace a
physical disconnect and cannot protect against every OS, I2C, driver or power
hardware failure.

## Required evidence chain

Physical motion remains prohibited until all nine gate records are current,
their evidence and artifact hashes match, and the operator explicitly confirms
the bounded run:

1. camera orientation;
2. motor mapping software;
3. current YOLO CSI regression;
4. physical IPM calibration;
5. live IPM dry run;
6. four-position outer-circle validation;
7. integrated control dry run;
8. raised-wheel motor adapter;
9. short floor steering.

The ordered procedure and exact confirmation strings are documented in
`docs/INTEGRATION_RUNBOOK.md`.

`track_run.py`, `guarded_live_circle_run.py` and
`guarded_straight_line_run.py` are included in the authorization artifact
snapshot. Each refuses motor initialization unless the current gate evidence
and source hashes still authorize motion.

## Battery and electrical safety

The photographed prototype uses a two-cell lithium-polymer motor pack. Use only
a charger intended for the exact cell count and chemistry, inspect the pack and
wiring before each session, protect the motor supply with the designed fuse,
and keep polarity and common-ground routing independently verified. Stop using
any swollen, damaged, hot or mechanically compromised pack. Do not infer the
as-built power topology from photographs alone.

## Emergency stop

Software interrupt handling attempts to command all outputs OFF, but the
primary emergency action is the spotter physically disconnecting motor power.
After a fault, disconnect motor power before diagnosis or repositioning.
