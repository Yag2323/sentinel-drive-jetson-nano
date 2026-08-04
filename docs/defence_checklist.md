# Final-year-project defence checklist

## Recommended ten-minute narrative

1. Define the problem and safety objective: autonomous tape following with
   object-aware fail-safe behavior on constrained Jetson Nano hardware.
2. Show the architecture: single-owner CSI capture, latest-frame fan-out,
   physical IPM lane observation, asynchronous YOLO, safety supervisor,
   provisional PD control, differential mixer, piecewise per-side PWM mapping,
   and a read-only dashboard.
3. Explain the simulation-to-reality result: a logical controller value is not
   a usable wheel PWM. Both measured floor breakpoints are 0.600, while the
   calibrated cruise pair is left 0.750/right 0.735.
4. Explain the power result: distinguish each true loaded voltage drop from the
   72 mV spread among recorded minimum values; show why the earlier pack made
   motion unreliable.
5. Present current perception evidence: upright CSI image, the report-only
   YOLOv5s baseline beside the pinned YOLOv5n integration benchmark, physical
   IPM calibration, IPM live dry run, and integrated motor-disabled DRIVE/STOP
   dry run.
6. Present safety engineering: conservative whole-frame object STOP, FULL-lane
   requirement, freshness checks, voltage checks, hashed validation gates, and
   the software command lease's explicit limitations.
7. Present physical integration evidence in order: raised adapter validation,
   short guarded left/right steering validation, then all track trials.
8. Finish with limitations and work still justified by evidence: encoders,
   metric object range, TensorRT, hardware output-disable protection, tuning
   across battery state of charge, and shared publication from one CSI owner.

## Evidence to keep open before the demonstration

- Current `python validation_manager.py show` output with every required gate,
  including `short_floor_steering`, passing without integrity errors.
- Final build-manifest JSON plus software, configuration, YOLO revision, and
  model-weights hashes.
- Upright dashboard/CSI screenshot and camera-orientation evidence.
- Current YOLOv5s baseline and YOLOv5n integration JSON, raw CSV, and annotated
  frames; keep the report-only/gate distinction visible.
- Physical IPM source quadrilateral, warp, mask, calibration JSON, and live
  dry-run evidence.
- Integrated motor-disabled dry-run CSV/JSON showing at least one DRIVE frame,
  at least one object STOP, and at least one PARTIAL/LOST lane STOP.
- Raised motor-adapter/lease evidence.
- `steering_floor_validation.py` evidence showing short left and right
  corrections and a controlled stop.
- Three or more unedited physical track-run CSV files and their separate
  analyses.
- A short offline video for a safe fallback if live hardware is unavailable.
- Wiring and power-distribution diagrams with the physical motor-battery
  disconnect identified.

For every displayed result, state both its evidence classification (for
example PHYSICALLY VERIFIED, SOFTWARE VERIFIED ON PHYSICAL CSI, SIMULATED, or
UNVERIFIED) and its currency (CURRENT or PRIOR). The retained 2026-07-26
YOLOv5n artifact measured `8.283126567668365` complete-window FPS and
`93.89088915256971 ms` mean inference. It is prior evidence for a changed
source identity and must not be presented as a current motion gate.

## Questions lecturers are likely to ask

- **Why YOLOv5n rather than YOLOv5s?** Show the paired real-CSI measurements,
  then explain the latency/resource/safety trade-off. YOLOv5s is report-only;
  only YOLOv5n is eligible for the integrated motion gate.
- **Why did you not install the latest YOLO repository requirements?** JetPack
  4.x relies on NVIDIA-compatible CUDA PyTorch, torchvision, OpenCV/GStreamer,
  and Python versions. The project uses a pinned YOLOv5 v6 tree and preflight
  checks rather than allowing a generic install to replace working platform
  packages.
- **What FPS did you actually achieve?** Quote
  `complete_measurement_window_fps` as observed processed-window throughput.
  Quote mean/p95 model inference separately from mean/p95 source-to-result
  latency; the latter is the camera-to-result delay relevant to safety. If showing
  `inference_only_fps`, call it a reciprocal-latency rate, not measured
  end-to-end throughput.
- **Why is 30 FPS not shown?** Separate the design objective from a measured
  raw-PyTorch baseline and any independently measured TensorRT result. The
  report-only YOLOv5s test configures a 30 Hz CSI source, while operational
  YOLOv5n uses 21 Hz; neither input setting proves achieved throughput.
- **Why IPM?** A bird's-eye frame stabilizes geometric lane offset, but the
  homography is valid only for the calibrated physical camera pose and track.
- **Can a partial lane still drive?** No. PARTIAL is diagnostic only;
  `allow_partial_lane_motion` is false and the scale is 0.0. Only FULL lane
  evidence can contribute to DRIVE.
- **Why does any relevant object stop the robot even outside the drawn
  corridor?** Metric range and path clearance have not been physically
  calibrated. Therefore any configured relevant detection anywhere in the
  image produces a conservative STOP. Area/corridor values are descriptive,
  not authorization evidence.
- **Is it a true PID?** The structure has P/I/D terms, time delta, filtering,
  and anti-windup, but the safe current configuration has `Ki = 0`; the active
  controller is PD until integral action is tuned and revalidated.
- **How is the right trim applied?** With piecewise side-specific breakpoints:
  left 0.600/0.750/0.980 and right 0.600/0.735/0.9604. The right floor is kept
  at the measured 0.600 rather than multiplied down to 0.588.
- **What happens if YOLO freezes?** Startup/error/stale/frame-lag checks make
  the supervisor request STOP. The motor lease is an additional best-effort
  software layer while the process, scheduler, and I2C path remain functional.
- **Does the 0.35 s lease protect against SIGKILL or every crash?** No. SIGKILL,
  OS/kernel failure, process-wide deadlock, blocked I2C, or hardware failure can
  prevent the OFF write. A spotter-controlled physical battery disconnect is
  the independent protection during testing.
- **Is the run deadline safety-rated?** No. It is a second best-effort software
  thread plus absolute-deadline checks on each PWM refresh. It records a
  conservative energized interval but still shares Python, scheduling, locks,
  I2C, and the PCA9685 with the command path.
- **Why are hashes part of authorization?** Passing evidence is bound to the
  exact relevant artifacts and evidence file. Any later edit, deletion, or
  replacement invalidates the gate; the older result remains prior evidence.
- **How is distance measured?** State the independent physical method and its
  uncertainty. Timed PWM is not distance measurement.
- **Is voltage battery percentage?** No. INA219 reports motor-bus voltage under
  load; it is not a state-of-charge percentage.
- **Why L298N?** Explain availability/cost and measured behavior, then identify
  a modern MOSFET driver as future work.
- **Why no dashboard controls?** The dashboard is deliberately read-only and
  exposes no motor command routes.
- **Why only one camera process?** The CSI/Argus camera has one designated
  owner. That owner fans frames to IPM and YOLO; the dashboard must not open a
  competing camera pipeline.

## Demonstration safety and order

1. Use a barrier-bounded, flat test area without stairs, drop-offs, traffic, or
   observers in the vehicle path.
2. Keep the motor battery physically disconnected for camera orientation, YOLO
   benchmarking, IPM calibration/live validation, and the integrated dry run.
3. Stop every competing CSI process before transferring camera ownership.
4. Connect motor power only for the raised adapter stage, with both wheels
   fully clear of the floor and a spotter holding the disconnect.
5. Run the short floor-steering gate only after the raised gate passes. Use the
   bounded pulse duration and verify both correction directions and STOP.
6. Immediately before any track run, run `validation_manager.py verify`, verify
   artifact/evidence hashes, inspect voltage, and make the explicit runtime
   confirmations `MOTOR-POWER-CONNECTED` and `TRACK-AREA-CLEAR` only after
   hands are off the chassis and wheels.
7. Start with the shortest permitted run and inspect the retained, unedited log before
   increasing duration.
8. Stop immediately after unexpected direction, noise, binding, smoke, smell,
   overheating, a camera-owner conflict, or repeated undervoltage.

The physical disconnect remains mandatory even after the software lease test
passes.
