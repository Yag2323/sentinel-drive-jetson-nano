# Sentinel Drive engineering evidence register

Project: Jetson Nano Robot Car / Safety Monitor

Programme: 6FTC2062 BEng Individual Major Project Part B

Student: Kathiravan Yagneshvar

Supervisor: Kim Siong Wong

Academic year: 2026

This register separates direct physical measurements, current-build software
checks, simulation, archived/prior results, and unresolved claims. Every gate
requires generated evidence with the expected test identifier, `passed=true`,
and matching start/end artifact snapshots. `validation_manager.py` verifies
those test-time hashes against the current build and records the evidence-file
SHA-256. Physical IPM calibration writes a JSON sidecar whose path is bound in
`ipm_config.json`; the recorder validates that sidecar rather than trusting a
configuration label alone. A screenshot or recollection can support an
observation but is not interchangeable with accepted current-build evidence.

Keep evidence classification separate from evidence currency. Classifications
used by the gate manager are PHYSICALLY VERIFIED, SOFTWARE VERIFIED, SOFTWARE
VERIFIED ON PHYSICAL CSI, and UNVERIFIED; SIMULATED identifies SITL-only work.
Currency is CURRENT or PRIOR. The seed register's PRIOR TEST EVIDENCE label is
the combined legacy label for a PRIOR result. A VERIFIED result is CURRENT only
while its recorded evidence and artifact hashes still match.

| Subsystem or claim | Result currently supportable | Classification | Evidence | Current gate/status |
|---|---|---|---|---|
| CSI camera produces a live image | Live image observed | PRIOR TEST EVIDENCE | Physical operator observation, 2026-07-26 | Repeat under current gate workflow |
| Physical CSI orientation | Upright with `nvvidconv flip-method=2` | PRIOR TEST EVIDENCE | Physical operator observation, 2026-07-26 | `camera_orientation` remains open until recorded |
| Current dashboard camera orientation | Native shared GI/GStreamer CSI monitor supplied; current dashboard view must be rechecked | UNVERIFIED | `robot_dashboard.py`, `patch_dashboard_camera_rotation.py`, and a dated screenshot when run | Open recheck |
| YOLOv5n real-CSI benchmark | Archived values reported as about 4.8 loop FPS and 245 ms mean inference | PRIOR TEST EVIDENCE | Preserve original JSON/CSV if available | Current regression required |
| YOLOv5s real-CSI baseline | Not yet measured on the current build | UNVERIFIED | `evidence/yolov5s_csi_*.json/.csv/.jpg` | Report-only; cannot satisfy a motion gate |
| YOLO detects a person | Detection seen in an earlier physical camera test | PRIOR TEST EVIDENCE | Archived screenshot/run | Repeat in current dry run |
| Current YOLO processed-frame throughput | Not yet accepted | UNVERIFIED | `evidence/yolov5n_csi_*.json` plus recorded SHA-256 | `yolo_current_regression` open until recorded |
| Earlier asynchronous lane/YOLO control rate | 7.78 reported control-loop FPS | PRIOR TEST EVIDENCE | Archived combined dry-run artifact | Do not label current build |
| Earlier combined lane/YOLO availability | 100% reported usable lane, zero stale YOLO frames, zero oscillations | PRIOR TEST EVIDENCE | Archived combined dry-run artifact | Repeat after physical IPM |
| SITL IPM geometry | Normalized seed only | SIMULATED | `ipm_config.seed.json` | Cannot authorize motion |
| Physical IPM calibration | Not yet accepted | UNVERIFIED | `ipm_config.json`, calibration image and recorded hash | `ipm_physical_calibration` open |
| Physical CSI IPM dry run | Not yet accepted | UNVERIFIED | `evidence/ipm_dry_run_*.json` | `ipm_live_dry_run` open |
| Integrated perception/control dry run | Not yet accepted | UNVERIFIED | `evidence/integrated_dry_*.json/.csv` | `integrated_control_dry_run` open |
| Object safety policy | Code requires any configured relevant detection anywhere in the frame to cause STOP; a stop sign or unclassified traffic light also causes STOP | UNVERIFIED | `yolov5_runtime.py`; current regression/dry-run evidence still required | Accept only after current dry-run evidence |
| Object distance | Box area/path overlap only; no metric distance calibration | UNVERIFIED | Logged descriptive image features | Must not be called metres or stopping distance |
| Partial-lane motion | Configured disabled: `allow_partial_lane_motion=false`, speed scale `0.0` | UNVERIFIED | `controller_config.json`; current safety-supervisor evidence still required | Every `PARTIAL_*` must STOP |
| Motor floor breakpoint | Left 0.600 and right 0.600 at the calibrated operating point | PRIOR TEST EVIDENCE | Physical calibration record | Recheck in final floor configuration |
| Nominal straight cruise pair | Left 0.750, right 0.735 | PRIOR TEST EVIDENCE | Physical floor calibration observation | Piecewise software mapping verified separately |
| Maximum mapped pair | Left 0.980, right 0.9604 | UNVERIFIED | `motor_config.json`, `motor_mapping.py` | Derived configuration, not a separate full-power physical result |
| Straight-line trim | Right trim 0.98; less than 5 cm error over 0.5 m | PRIOR TEST EVIDENCE | Physical operator floor measurement | Operating-point caveat applies |
| New-pack start-to-minimum voltage sag | Approximately 0.372-0.448 V across the recorded 40-98% tests | PRIOR TEST EVIDENCE | Preserve original physical terminal logs | Do not replace with minima spread |
| Spread among new-pack minimum voltages | 72 mV, from 7.232-7.304 V | PRIOR TEST EVIDENCE | Calculation from prior drivetrain records | Not voltage sag |
| Previous-pack voltage sag | Up to approximately 2.49 V | PRIOR TEST EVIDENCE | Earlier motor-test records | Comparative result only |
| Piecewise per-side PWM mapping | Both sides retain a 0.600 floor; right cruise/max are 0.735/0.9604 | UNVERIFIED | `integration_self_test.py`, configuration | Record `motor_mapping_software`; still does not prove physical steering |
| PID structure | Code is time-aware, derivative-filtered and anti-windup; current `Ki=0` | UNVERIFIED | `control_core.py`; record current self-test/dry run | Operational configuration is PD until Ki is tuned |
| PID gains | Kp 0.30, Ki 0.00, Kd 0.04 initial values | UNVERIFIED | `controller_config.json` | Physical tuning open |
| Software command lease | Fake-controller test is designed to check PWM-off after a 0.35 s lease in a responsive process | UNVERIFIED | `integration_self_test.py` | Record current self-test; not a hardware watchdog |
| Raised motor adapter | Not yet accepted for current build | UNVERIFIED | `evidence/raised_motor_adapter_*.json` | `raised_motor_adapter` open |
| Short floor steering | Not yet accepted for current build | UNVERIFIED | `evidence/steering_floor_validation_*.json` | `short_floor_steering` open |
| Autonomous track performance | Not measured for current gated build | UNVERIFIED | `results/*.csv`, `results/*.json`, analysis hashes | Locked until every gate/hash passes |

## FPS and latency terminology

- **Processed-frame throughput** (`complete_measurement_window_fps`) is
  measured frames divided by the full measurement-window duration. For
  `yolo_csi_benchmark.py` it includes CSI acquisition, preprocessing, inference,
  NMS, per-frame overlay work, and per-frame CSV logging inside that window.
  Final evidence-image and summary-JSON writes are outside the timed window.
- **Configured CSI source rate** is 30 Hz for the report-only YOLOv5s baseline
  and 21 Hz for the operational YOLOv5n gate. It is not achieved inference,
  processed-window, dashboard, or control-loop FPS.
- **Capture-plus-detector latency** (`mean_capture_plus_detector_ms`, with
  median and p95 fields) is per-frame capture and detector work; it excludes
  later overlay/evidence work.
- **Inference latency** (`mean_inference_ms`, `median_inference_ms`, and
  `p95_inference_ms`) is synchronized model inference time in milliseconds per
  processed frame.
- **NMS latency** (`mean_nms_ms`) isolates non-maximum suppression.
- **Detector end-to-end latency** (`mean_detector_end_to_end_ms`) covers the
  detector path from preprocessing through decoded detections. It does not
  include camera age.
- **Source-to-result latency** (`mean_source_to_result_ms` and
  `p95_source_to_result_ms`) runs from the trustworthy camera capture timestamp
  to detection-result availability and is the relevant perception-delay metric
  for the safety discussion.
- **Inference reciprocal rate** (`inference_only_fps`, calculated as
  `1000 / mean_inference_ms`) is a
  latency-derived equivalent, not observed camera throughput, sensor frame
  rate, dashboard refresh rate, or control-loop frequency.
- **Control-loop FPS** describes iterations of the integrated controller. It is
  not YOLO throughput and must state whether it is frames/elapsed time or an
  average of instantaneous rates.
- Raw PyTorch and TensorRT results must be labelled separately with model,
  input shape, precision, build manifest, and weights hash. Do not claim 30 FPS
  unless the stated end-to-end definition is actually measured.

## Measurement corrections to preserve in the report

- Voltage sag is starting voltage minus minimum loaded voltage for the same
  run.
- The 72 mV value is only the spread among observed minima; it is not the
  battery's loaded voltage sag.
- INA219 bus voltage is not state of charge.
- Current must not be claimed from an uncalibrated INA219 current register.
- Bounding-box area ratio and path overlap are image-space descriptors, not
  distance in metres or a calibrated stopping-distance decision.
- A timed motor command is not a distance measurement. Report the independent
  distance and lateral-error method and its uncertainty.
- A software simulation/dry-run can verify logic but cannot be classified as a
  physical motor or steering verification.
- If code, configuration, model weights, or a recorded evidence file changes,
  its earlier hash identifies prior evidence; rerun affected gates.
- Hashes do not observe physical changes. Camera pose, track geometry,
  drivetrain, wheel, wiring, driver, or battery/power changes manually
  invalidate the earliest affected physical gate and every downstream result.

## Software stop limitation and independent physical protection

The 0.35 s command lease is useful defence in depth, but it is not a
safety-rated E-stop. It depends on the Python process, watchdog thread,
scheduler, I2C lock and transaction, PCA9685, and motor driver remaining
responsive. It cannot guarantee PWM removal after `SIGKILL`, an OS/kernel
crash, process-wide deadlock, blocked I2C, or hardware failure. `Ctrl+C`,
exception handlers, and `finally` cleanup are also best-effort software paths.

Every powered motor test therefore requires a spotter with immediate control
of the physical motor-battery disconnect. A hardware PCA9685 OE/watchdog path
is recommended future work.

## Immutable current-build safety contract

- Steps 1-4 of the integration runbook use a physically disconnected motor
  battery.
- Camera failure, invalid timestamp, repeated frame, or stale camera data never
  authorizes motion.
- YOLO startup, failure, stale output, or excessive frame lag never authorizes
  motion.
- Missing, invalid, low, or implausibly high motor-bus voltage never authorizes
  motion.
- Anything other than a confident `FULL` lane, including every partial-lane
  state, stops motion.
- Any configured relevant object detection stops motion under the current
  conservative policy; image area is not used to waive the stop.
- The software command lease attempts PWM-off if refreshed commands cease, but
  the physical disconnect remains the independent emergency stop.
- Only one process owns the CSI camera at a time.
- The dashboard remains read-only and exposes no motor-command endpoint.
- Physical motion requires every current gate, intact evidence hashes, explicit
  runtime confirmation, a safe site, and the spotter-controlled disconnect.
