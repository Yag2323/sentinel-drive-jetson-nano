# Sentinel Drive technical handover

## Project status

Sentinel Drive is a Jetson Nano differential-drive robot-car prototype. The
repository contains the project-authored camera, single-line, YOLOv5,
controller, motor-output, dashboard and validation code. Physical subsystem
tests and motor-disabled integration tests were performed, but a continuous
autonomous lap on the latest single-line build remains **UNVERIFIED**.

The public configuration is intentionally fail-closed:

- `integration_gates.json`: motion authorization `false / LOCKED`;
- `controller_config.json`: initial gains not physically tuned for the current
  single-line build; and
- `ipm_config.json`: software seed geometry, requiring physical calibration on
  deployment.

## Hardware

| Component | Repo-backed interface | Status |
|---|---|---|
| NVIDIA Jetson Nano | Runs CSI capture, IPM, YOLO, control, logging and dashboard | Physical prototype photographed; exact carrier/RAM/storage SKU not retained |
| IMX219 CSI camera | `sensor-id=0`, Argus/GStreamer, `flip-method=2` | Physical CSI tests retained |
| PCA9685 | Linux I2C bus 1, address `0x40`, 200 Hz | Source recovered in `manual_motor_control.py` |
| INA219 | Linux I2C bus 1, address `0x41`, bus-voltage register `0x02` | Voltage path implemented; calibrated current is not claimed |
| L298N dual H-bridge | Driven by PCA9685 enable and direction channels | Physical module photographed; exact revision/jumper state not retained |
| Two geared DC motors | Differential drive | Open-loop trim; no wheel encoders |
| Motor pack | ONBO 2S 7.4 V, 1550 mAh, 45C LiPo visible in the prototype photographs | XT60/14-AWG lead reported; charger and pack-age records not retained |

### PCA9685 channel allocation

| Channel | Symbol | Function |
|---:|---|---|
| 0 | `LEFT_ENA` | Left enable/PWM |
| 1 | `LEFT_IN1` | Left direction input 1 |
| 2 | `LEFT_IN2` | Left direction input 2 |
| 3 | `RIGHT_IN3` | Right direction input 3 |
| 4 | `RIGHT_IN4` | Right direction input 4 |
| 5 | `RIGHT_ENB` | Right enable/PWM |

### Reported motor-power topology

The project notes report this intended series path:

```text
Battery positive -> fuse -> INA219 VIN+ -> INA219 VIN- -> L298N VS/12V
Battery negative --------------------------------------> common ground
Jetson/PCA9685/INA219 logic grounds -------------------> common ground
L298N outputs ------------------------------------------> left/right motors
```

The physical disconnect must independently remove motor-driver power. The
exact fuse rating, connector assembly, common-ground continuity and installed
disconnect position are not established by a retained schematic/continuity
record, so they must be checked before powered work.

## Software environment

The retained 2026-07-26 preflight evidence records:

| Item | Value |
|---|---|
| NVIDIA L4T | R32.7.6 |
| Python | 3.6.9 |
| CUDA | 10.2 |
| PyTorch | 1.8.0 |
| OpenCV | 3.2.0 |
| NumPy | 1.19.5 |
| GPU | NVIDIA Tegra X1 |
| Virtual environment used on Jetson | `~/ai_robot_car/venv_jetson` |

The project depends on JetPack-compatible PyTorch/OpenCV/GStreamer builds. Do
not replace them with an unreviewed current desktop requirements set.

YOLOv5 v6.0 is provisioned at pinned commit
`956be8e642b5c10af4a1533e09084ca32ff4f21f`; source and weights are deliberately
not vendored in this repository.

## Runtime architecture

1. `gst_camera_bridge.py` owns the CSI camera and supplies latest-frame data.
2. `ipm_lane.py` applies the stored ground-plane transform.
3. `single_line_lane.py` selects and fits one black tape line.
4. `yolov5_runtime.py` asynchronously detects configured COCO objects.
5. `control_core.py` checks freshness, confidence, obstacle and voltage state.
6. `motor_mapping.py` maps logical command to calibrated per-side PWM.
7. `safe_motor_output.py` enforces voltage, deadline, lease and OFF behaviour.
8. `manual_motor_control.py` performs PCA9685 and INA219 I2C operations.
9. JSON/CSV artifacts record the code identity and measured outcome.

## Key source inventory

| File | Purpose |
|---|---|
| `single_line_lane.py` | Connected black-line candidate extraction and polynomial observation. |
| `ipm_lane.py` | Perspective warp, mask generation and detector dispatch. |
| `yolov5_runtime.py` | YOLO detector, async worker, annotations and object-stop policy. |
| `control_core.py` | PID and safety supervisor. |
| `motor_mapping.py` | Piecewise mapping around measured drivetrain deadband. |
| `manual_motor_control.py` | PCA9685/INA219 low-level adapter. |
| `safe_motor_output.py` | Guarded output, watchdog lease and OFF enforcement. |
| `integrated_control_dry_run.py` | Real-CSI lane/YOLO/controller test with no motor commands. |
| `track_run.py` | Main gate-bound physical runner. |
| `guarded_live_circle_run.py` | Validation-gated single-line circle commissioning runner; full-lap result open. |
| `guarded_straight_line_run.py` | Validation-gated empty-lane straight-line runner without YOLO; physical test open. |
| `validation_manager.py` | Evidence-type and source-hash gate manager. |
| `robot_dashboard.py` | Read-only HTTP dashboard with optional CSI stream. |
| `test_single_line_synthetic.py` | Single-line valid/rejection regression. |
| `test_ipm_lane_synthetic.py` | Earlier dual-rail/IPM regression. |

## Persisted calibration and controller values

### Motor mapping (`motor_config.json`)

| Parameter | Value |
|---|---:|
| `reverse_left` / `reverse_right` | `false / false` |
| Left/right deadband | `0.600 / 0.600` |
| Left/right cruise | `0.750 / 0.735` |
| Left/right maximum | `0.980 / 0.9604` |
| Right trim | `0.98` |
| Calibration voltage | `7.688 V` |
| Calibration date | `2026-07-24` |

### Controller (`controller_config.json`)

| Parameter | Value |
|---|---:|
| `kp` | `0.30` |
| `ki` | `0.00` |
| `kd` | `0.04` |
| Integral limit | `0.35` |
| Derivative filter alpha | `0.25` |
| Steering limit | `0.18` |
| Minimum line confidence | `0.55` |
| Camera stale timeout | `0.25 s` |
| YOLO stale timeout | `0.75 s` |
| Maximum YOLO frame lag | `8` |
| Command lease | `0.35 s` |
| Motor voltage bounds | `6.60 V` to `8.60 V` |

`PIDController.update()` uses measured monotonic timestamps. `dt` is checked
and clamped rather than assumed constant. With `ki=0`, the current behaviour is
PD even though the implementation supports PID. The gains are **PROVISIONAL**
for the latest physical configuration.

## Object and stop-sign behaviour

`yolov5_runtime.evaluate_obstacles()` maps:

- `stop sign` to `STOP_SIGN_DETECTED`;
- `traffic light` to `TRAFFIC_LIGHT_UNCLASSIFIED` and STOP;
- person, bicycle, car, motorcycle, bus, truck and chair to
  `OBJECT_DETECTED_<CLASS>` and STOP; and
- no fresh relevant detection to `PATH_CLEAR`.

This is conservative object-aware stopping. There is no obstacle path planner,
metric object distance or steering-around behaviour.

## Safety and fail-safe behaviour

| Guard | Trigger | Response |
|---|---|---|
| Camera validity | Missing, repeated, invalid timestamp or older than `0.25 s` | STOP/OFF |
| Line validity | Any status other than `FULL` or confidence below `0.55` | STOP/OFF |
| YOLO validity | No result, worker error, older than `0.75 s` or lag greater than 8 frames | STOP/OFF |
| Object policy | Any configured relevant detection | STOP/OFF |
| Voltage | Below `6.60 V`, above `8.60 V`, missing or invalid | STOP/OFF |
| Command lease | No refresh within `0.35 s` while the watchdog remains schedulable | Best-effort all-off |
| Motion deadline | Bounded run duration reached | STOP/OFF |
| Exception/Ctrl+C | Any unhandled error or operator interrupt | `finally` close and all-off attempt |
| Evidence gate | Required current evidence/hash absent | Refuse to open motor interface |

The software watchdog is not safety rated. `SIGKILL`, kernel failure,
process-wide deadlock, blocked I2C or hardware failure can prevent an OFF write.
A spotter must retain the physical motor-battery disconnect during every
powered test.

## Measured results

See `docs/project_evidence_register.md` for exact values and artifact hashes.
The retained headline results are:

- YOLOv5n: `8.283126567668365` complete-window FPS;
- IPM lane dry run: `13.25091753132572` FPS and
  `99.73890339425587%` full-lane observations;
- integrated dry run: `8.753886744693606` control FPS and
  `97.75051124744377%` fresh YOLO results;
- integrated object STOP: 41 frames, including
  `OBJECT_DETECTED_PERSON`; and
- motor deadband/cruise: `0.60 / 0.75` at the recorded `7.688 V`
  calibration point.

## Current open items

1. Re-establish physical IPM calibration after any camera-height/angle change.
2. Run the current single-line detector at representative straight and curve
   poses with the motor battery disconnected.
3. Rebuild all current hash-bound validation gates.
4. Retune the current controller conservatively on the physical track.
5. Record a continuous full-lap JSON/CSV result or continue to label it
   unverified.
6. Record a labelled as-built wiring diagram, fuse rating, continuity checks
   and physical disconnect verification.
7. Add a hardware PCA9685 OE/watchdog path if the prototype is developed
   beyond supervised academic testing.

## Safe handover rule

Do not lower a confidence threshold, extend a stale-data timeout, bypass
`validation_manager.py`, remove YOLO from the safety decision or weaken the
voltage/lease checks merely to make the vehicle move. Diagnose the underlying
timing, power, calibration or mechanical problem and regenerate evidence for
the changed build.
