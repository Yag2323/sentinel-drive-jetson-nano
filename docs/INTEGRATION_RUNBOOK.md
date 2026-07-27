# Sentinel Drive integration runbook

This sequence preserves the Jetson Nano's existing JetPack 4.x, Python 3.6,
CUDA-enabled PyTorch, and NVIDIA GI/GStreamer camera environment. The system
OpenCV build may report GStreamer `NO`; CSI capture must therefore use the
shared GI bridge, not `cv2.VideoCapture(CAP_GSTREAMER)`. Record the exact
versions with `jetson_perception_preflight.py`; do not infer a JetPack point
release from the Linux kernel alone.

Do **not** clone current YOLOv5 `master` over the project runtime and do **not**
run its current `requirements.txt`. Current upstream requirements can require a
newer Python, replace NVIDIA's CUDA-enabled PyTorch/torchvision, or shadow the
platform OpenCV build. `prepare_yolov5_v6.py` preserves an existing
matching checkout or provisions the official v6.0 tag at commit
`956be8e642b5c10af4a1533e09084ca32ff4f21f`, then verifies the official
`yolov5n.pt` and `yolov5s.pt` identities. It never runs `pip` and never
overwrites an unexpected checkout or weight file. A failed provisioning or preflight is a stop
condition, not permission to repair the environment with an unreviewed
command.

Only one process may own `nvarguscamerasrc`. Stop `nvgstcapture-1.0`, the
dashboard camera, and every other CSI user before starting another camera,
YOLO, IPM, dry-run, or track process.

## Safety boundary for this sequence

- Before Step 0, stop every dashboard, camera, autonomy, and motor-test process
  on the Jetson and physically disconnect the motor battery.
- Keep the motor battery **physically disconnected throughout Steps 0-4**.
- A software statement that PWM is off is not a substitute for that physical
  isolation.
- At Step 5, launch with motor power still disconnected. Connect it only after
  the script confirms initial hardware OFF, with both wheels raised and the
  spotter holding the disconnect.
- Floor motion begins only at Step 6, after all earlier evidence gates pass.
- Keep the motor-battery disconnect directly accessible during every powered
  motor test. Do not place a hand near a rotating wheel.

## 0. Transfer, preserve calibration, and identify the build

Before transferring anything, use the Jetson's current terminal to stop every
project process with `Ctrl+C`, then physically disconnect the motor battery.
Confirm that this read-only check prints no running project command (the
`pgrep` command itself may appear briefly on some systems):

```bash
pgrep -af 'robot_dashboard.py|track_run.py|motor_adapter_raised_test.py|steering_floor_validation.py|integrated_control_dry_run.py|ipm_live_dry_run.py|yolo_csi_benchmark.py|nvgstcapture-1.0'
```

Do not transfer or rewrite motion code/configuration while any listed process
is running. Leave motor power physically disconnected until Step 5.

In Windows PowerShell:

```powershell
cd "C:\path\to\sentinel-drive-jetson-nano"
powershell -ExecutionPolicy Bypass -File .\deploy_integration.ps1 -SafetyConfirmed
```

The deployment script first creates a timestamped recovery copy of the remote
dashboard and verifies the transferred dashboard SHA-256 before declaring the
transfer complete.

The public-release example target is `jetson@jetson-nano.local`. If mDNS is unavailable,
use the Jetson's currently verified address rather than an old saved address:

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy_integration.ps1 -HostName "CURRENT_JETSON_IP" -SafetyConfirmed
```

On the Jetson:

```bash
cd ~/ai_robot_car
source venv_jetson/bin/activate
python install_motor_calibration.py
python install_integration_bundle.py
python -m py_compile \
  robot_dashboard.py patch_dashboard_camera_rotation.py gst_camera_bridge.py \
  source_integrity.py prepare_yolov5_v6.py jetson_perception_preflight.py \
  yolov5_runtime.py yolo_csi_benchmark.py ipm_lane.py calibrate_ipm.py \
  install_oval_lane_profile.py test_ipm_lane_synthetic.py \
  ipm_alignment_diagnostic.py ipm_live_dry_run.py control_core.py \
  motor_mapping.py safe_motor_output.py integration_self_test.py \
  integrated_control_dry_run.py motor_adapter_raised_test.py \
  steering_floor_validation.py validation_manager.py track_run.py \
  analyse_track_run.py promote_controller_tuning.py make_build_manifest.py \
  install_integration_bundle.py install_motor_calibration.py
python prepare_yolov5_v6.py
python validation_manager.py record-self-test
python make_build_manifest.py
```

Run these commands in order and stop at the first non-zero exit or `REFUSED` /
`FAIL` message. Do not continue by guessing or installing packages.

The provisioning helper accepts only a clean source tree at the pinned commit.
It verifies the official 3,952,441-byte `yolov5n.pt` asset with SHA-256
`649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f`
and the official 14,698,491-byte `yolov5s.pt` asset with SHA-256
`c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598`.
YOLOv5s is a report-only raw-PyTorch baseline; YOLOv5n remains the sole
integration and motion-gate model. If an unexpected `yolov5_v6` or weight file
already exists, the helper refuses rather than deleting or replacing it;
preserve that item under a backup name before rerunning. Do not run the
repository's `requirements.txt`.

The motor-calibration installer first creates or updates `motor_config.json`
with the measured breakpoints while preserving existing `reverse_left` and
`reverse_right` booleans and backing up any prior file. The integration
installer then makes timestamped backups, replaces
`controller_config.json` with the safe provisional seed, and resets
`integration_gates.json` after the code update. It preserves the calibrated
`motor_config.json` and an existing valid `ipm_config.json`. This deliberate
reset prevents old controller/gate state from silently authorizing new code.
Use `python install_integration_bundle.py --preserve-controller-tuning` only
when the existing controller configuration is strictly valid and already
classified `PHYSICALLY_TUNED`; the installer refuses any other preservation
request. Gate state is reset in either case and must be rebuilt with current
evidence.

Expected software-only result: `INTEGRATION SELF-TEST: PASS`. This uses a fake
motor controller; it does not physically verify a motor, I2C transaction, PWM
output, steering response, or watchdog stop.

Keep the initial build-manifest JSON. Run `make_build_manifest.py` again after
the final configuration and gate state are established. If a core source or
configuration file changes, treat results from the older build as prior
evidence and rerun every affected validation.

## Evidence and hash workflow

Every current-build gate uses a timestamped JSON evidence file. A producer
hashes its relevant code/configuration before and after the complete test and
sets `artifact_snapshot_stable=true` only when they match. Promote evidence
only with `validation_manager.py`; the manager checks its exact test type,
`passed=true`, test-time hashes, current hashes, evidence-file SHA-256 and gate
prerequisites before recomputing motion authorization. Physical IPM
calibration stores its JSON evidence path inside `ipm_config.json`; the
`record-ipm-calibration` command follows and validates that sidecar rather than
trusting the configuration label alone.

```bash
python validation_manager.py show
```

Do not edit, replace, or move a JSON file after recording it. A missing file or
hash mismatch invalidates that evidence. A passing file proves only the test
and build described inside it; it does not retroactively promote an archived
result or a manually copied screenshot.

Hashes cannot detect a physical change. Revalidate after moving the camera,
changing its angle/height, altering tape/track geometry, replacing a motor or
wheel, changing gearing/driver/wiring, or changing the battery/power path.
Record the earliest affected gate again; the manager automatically marks every
transitive downstream gate `STALE`. Camera-pose changes restart at camera
orientation and physical IPM calibration. Track-geometry changes restart at
physical IPM calibration. Drivetrain/power changes require calibration review,
the software mapping test, raised validation, and floor validation before any
new track result.

Use the exact `Evidence:` path printed by the test that just finished. Do not
discover gate evidence with `ls -t`: if a new test fails before writing JSON,
that pattern can silently select an older pass. Before recording, inspect the
chosen JSON with `python -m json.tool PATH` and confirm its `test`, `passed`,
and `completed_utc` values describe the run just observed.

## 1. Persist and verify camera orientation - battery disconnected

Confirm the motor battery is disconnected and stop every other CSI owner.

```bash
cd ~/ai_robot_car
source venv_jetson/bin/activate
python patch_dashboard_camera_rotation.py robot_dashboard.py
python -m py_compile robot_dashboard.py gst_camera_bridge.py
grep -n "gst_camera_bridge\|capture = GstCamera\|flip_method=DEFAULT_CSI_FLIP_METHOD" robot_dashboard.py
python robot_dashboard.py --host 0.0.0.0 --port 8080 --camera csi --camera-source 0 --allow-lan-camera
```

Open `http://JETSON_IP:8080` on the laptop and verify that the current dashboard
feed is upright. Save a dated screenshot. Press `Ctrl+C` and confirm the
dashboard process is stopped before continuing. The transferred dashboard has
native latest-frame CSI support and uses the shared GI/GStreamer `GstCamera`;
the idempotent patch utility validates this architecture and reports it as
already configured. On a recognised earlier camera-enabled dashboard, the
utility creates a timestamped backup before porting CSI away from
`cv2.VideoCapture(CAP_GSTREAMER)`. OpenCV is used only for JPEG encoding, so
its build does not need GStreamer enabled. The dashboard exposes only GET/HEAD
monitoring routes and never imports the motor controller.

Record the current-build observation only after personally seeing an upright
live image:

```bash
python validation_manager.py confirm-camera-orientation
python validation_manager.py show
```

This gate records the operator orientation confirmation and binds the patched
dashboard, patch utility, and shared CSI bridge used by autonomy. Preserve the
dated dashboard screenshot and final build manifest separately. Any later
dashboard-camera pipeline edit invalidates this gate and requires another
visual confirmation.

The earlier interactive `nvgstcapture-1.0` observation is valid prior physical
camera evidence; this dashboard check is a separate current-build observation.

## 2. Benchmark YOLOv5s, then regress YOLOv5n - battery disconnected

Stop the dashboard and confirm the motor battery remains disconnected. Audit
the existing environment without installing or upgrading packages:

```bash
python jetson_perception_preflight.py
```

Proceed only if preflight passes, identifies CUDA as available, imports the
pinned YOLOv5 v6 API, and records the model-weights hash. `nvpmodel` and
`jetson_clocks --show` are performance-context diagnostics rather than
inference dependencies. L4T R32.7.x may print a `WARN jetson_clocks_query`
when the unprivileged process cannot read an EMC-cap sysfs node; that warning
is preserved in evidence but does not override the required CUDA, source,
weights, import, or GStreamer checks. Record the reported 5 W/10 W mode when
interpreting FPS, and do not change power mode merely to silence the warning.
First measure the requested YOLOv5s raw-PyTorch/CUDA-FP16 baseline:

```bash
python yolo_csi_benchmark.py --model yolov5s --camera-fps 30 --seconds 30 --img-size 320 --confidence 0.55 --headless
```

Preserve its printed JSON/CSV/image paths for the report. It is deliberately
labelled `YOLOV5S_REAL_CSI_BASELINE`, `motion_gate_eligible=false`; **do not**
pass that JSON to `validation_manager.py`. On a 4 GB Nano it can be slow or run
out of memory, which is itself a reportable result. Keep the dashboard and all
other camera consumers stopped.

Then measure the lighter model used by the integrated system:

```bash
python yolo_csi_benchmark.py --model yolov5n --camera-fps 21 --seconds 30 --img-size 320 --confidence 0.55 --headless
```

The 30 Hz setting reproduces the requested source configuration for the
report-only YOLOv5s baseline. The integrated system deliberately uses a 21 Hz
CSI source with YOLOv5n to limit load. Neither configured source rate is an
achieved inference or control rate; use the measured fields below.

Use the terms precisely:

- `complete_measurement_window_fps` is observed processed-frame throughput over
  the complete timed window. It includes CSI acquisition, preprocessing,
  inference, NMS, per-frame overlay work, and per-frame CSV logging performed
  inside that window. Final evidence-image and summary-JSON writes occur after
  the timed window and are excluded.
- `mean_capture_plus_detector_ms` (with its median and p95 fields) is per-frame
  capture-plus-detector time; it excludes the later overlay/evidence work.
- `mean_inference_ms`, `median_inference_ms`, and `p95_inference_ms` are
  synchronized model-inference latency statistics per processed frame.
- `mean_nms_ms` isolates non-maximum-suppression time.
- `mean_detector_end_to_end_ms` covers the detector path from preprocessing
  through decoded detections; it is not camera-to-result latency.
- `mean_source_to_result_ms` and `p95_source_to_result_ms` measure from the
  camera capture timestamp to detection-result availability. These are the
  most relevant perception-delay figures for the safety discussion, but still
  exclude later overlay and evidence output.
- `inference_only_fps` is only `1000 / mean_inference_ms`, a reciprocal-latency
  equivalent. It is not observed camera throughput, control-loop frequency, or
  proof that the system processes that many live frames each second.
- The camera's configured capture rate is not the achieved detection rate.

Archived values of approximately 4.8 reported loop FPS and 245 ms mean
inference are **prior test evidence**. Do not present them as a current result
or assume they came from the same build/configuration unless the archived raw
files prove that. A 30 FPS statement remains a design target unless a named,
hash-addressed build measures it end to end.

Record **only the YOLOv5n** current result and immediately verify the gate
state. The manager categorically rejects the YOLOv5s baseline:

```bash
YOLO_EVIDENCE="PASTE_THE_EXACT_EVIDENCE_PATH_PRINTED_BY_THIS_BENCHMARK"
python -m json.tool "$YOLO_EVIDENCE"
python validation_manager.py record yolo_current_regression "$YOLO_EVIDENCE"
python validation_manager.py show
```

## 3. Calibrate and validate physical IPM - battery disconnected

Stop every CSI process and confirm the motor battery remains disconnected. The
supplied quadrilateral is a normalized seed from the WSL/SITL
`src/lane/lane_module.py`; it is simulated initialization, not physical
calibration evidence.

For the oval track, install the paired-row polynomial detector profile before
calibration. This profile follows two independently observed open tape rails;
it does not divide candidates at the image centre and does not require a
horizontal closing line. The installer makes and verifies a timestamped backup,
preserves the existing physical source/destination points exactly, and marks
the old calibration as requiring revalidation. First run the hardware-free
regression, then migrate the configuration:

```bash
python test_ipm_lane_synthetic.py
python install_oval_lane_profile.py
python validation_manager.py show
```

The synthetic test must pass the straight, both same-side curve directions,
concentric-oval, horizontal-crossbar, invalid-width, single-rail,
blank-after-memory, texture-noise, outside-ROI, fragmented-rail,
lookahead-without-local-support, candidate-overflow, non-identity perspective,
asymmetric-outlier, stale-data and fail-safe STOP cases.
After installation, physical motion authorization must be false until the
affected gates below are rebuilt. The detector's current provisional oval
lookahead is `0.62` of warped-frame height; tune it only from retained physical
track evidence, not by bypassing a gate.

Place the stationary robot at its final camera height and pitch on the real
black-tape track. Park it centred on a **straight portion of the oval**, with
both open longitudinal rails visible ahead. Do not use an oval end, crossbar,
or curved corner as the calibration target. Then run through VNC:

```bash
python calibrate_ipm.py
```

Click the centre of the near-left rail, near-right rail, far-right rail, then
far-left rail. These are the same geometric BL, BR, TR, TL points, but all four
must lie on the two open lane boundaries. Press `S` to save, `R` to reset, or
`Q` to cancel. The tool backs up the previous configuration and saves an
evidence image.

For the live validation, leave the motor battery disconnected and manually
align the stationary robot centreline with the marked lane centre. Use
`--expected-centered` only after physically making and checking that alignment;
the flag records this test condition and is required for `passed=true`.

```bash
python validation_manager.py record-ipm-calibration
python ipm_live_dry_run.py --seconds 30 --headless --expected-centered
IPM_EVIDENCE="PASTE_THE_EXACT_EVIDENCE_PATH_PRINTED_BY_THIS_IPM_RUN"
python -m json.tool "$IPM_EVIDENCE"
python validation_manager.py record ipm_live_dry_run "$IPM_EVIDENCE"
python validation_manager.py show
```

The IPM diagnostic passes only with at least 50 frames, 90% `FULL` lane,
95% usable lane, 5 FPS, mean confidence at least 0.55, and mean absolute
centred-reference offset no greater than 0.08. It reports full and partial
observations separately. Partial
lanes may contribute to diagnostic availability metrics, but
`allow_partial_lane_motion=false` and `partial_lane_speed_scale=0.0`: a
`PARTIAL_*` observation never authorizes physical motion in this build. Only a
confident `FULL` lane may reach `DRIVE`.

Before the 30-second gate run, use the diagnostic on the stationary robot at a
straight, curve-entry, curve-apex and curve-exit pose in both turn directions:

```bash
python ipm_alignment_diagnostic.py --warmup-seconds 3 --seconds 10 --headless
```

At every pose, inspect the saved overlay and mask. Both rails must be traced;
the status must remain `FULL`, confidence must remain at least `0.55`, and no
single remembered rail may be described as `FULL`. A diagnostic is not gate
evidence and cannot authorize motion.

## 4. Validate CSI + IPM + YOLO + controller mapping - battery disconnected

Confirm the motor battery is still physically disconnected. This program does
not import the physical motor module; displayed PWM values are calculations.

First show a clear full lane so `DRIVE` is exercised. Then place a
person-sized target or another configured object class in the camera view to
exercise object `STOP`. Finally, obscure/remove enough tape from view to
produce a PARTIAL or LOST observation and exercise lane `STOP`. Motor power
remains physically disconnected for all three conditions.

The integrated evidence additionally requires at least 80% `FULL` lane, 95%
usable lane, 90% fresh YOLO state, 5 FPS, at least 20 controlled frames, and
bounded steering saturation/derivative/sign-flip diagnostics. The deliberately
brief lane obstruction must therefore remain brief enough not to dominate the
run.

```bash
python integrated_control_dry_run.py --seconds 45
```

The current object policy is deliberately conservative: any configured
relevant object detection anywhere in the frame causes `STOP`. Box area and
path overlap are logged only as descriptive image measurements; they are not
calibrated distance and do not authorize motion. A stop sign also causes
`STOP`; a traffic light is treated as `STOP` because its colour/state is not
classified.

The run must include full-lane `DRIVE` frames, object-detection `STOP` frames,
and PARTIAL/LOST lane `STOP` frames, with fresh camera/YOLO data and no
mapped-duty violation. Partial-lane frames must remain stopped.

```bash
CONTROL_EVIDENCE="PASTE_THE_EXACT_EVIDENCE_PATH_PRINTED_BY_THIS_DRY_RUN"
python -m json.tool "$CONTROL_EVIDENCE"
python validation_manager.py record integrated_control_dry_run "$CONTROL_EVIDENCE"
python validation_manager.py show
```

The controller structure is time-aware, derivative-filtered, and
anti-windup, but `Ki=0` and the gains remain provisional. This stage verifies
software sign, safety decisions, and piecewise side mapping without claiming
physical steering performance.

## 5. Raised continuous motor-adapter validation

Stop all camera programs. Secure the chassis so both wheels are completely off
the floor, but keep the motor battery physically disconnected when launching
the script. The controller must first wake the PCA9685 and confirm an initial
hardware OFF write without motor power. A spotter must hold the physical
battery disconnect throughout the test.

```bash
python motor_adapter_raised_test.py
```

Type `RAISED-INITIALISE-OFF` only with motor power disconnected. After the
program reports that controller initialization and hardware OFF succeeded, the
spotter may connect motor power and type `BATTERY-CONNECTED-RAISED`. The test
then applies refreshed commands and deliberately lets the 0.35 s software
command lease expire. Confirm completion only if both wheels ran forward
smoothly and visibly stopped. Its JSON records the conservative energized
duration of that no-refresh lease test; the gate refuses a missing value or a
duration beyond the lease plus the documented scheduler/I2C tolerance.

```bash
MOTOR_EVIDENCE="PASTE_THE_EXACT_EVIDENCE_PATH_PRINTED_BY_THIS_RAISED_TEST"
python -m json.tool "$MOTOR_EVIDENCE"
python validation_manager.py record raised_motor_adapter "$MOTOR_EVIDENCE"
python validation_manager.py show
```

### What the software command lease does not guarantee

The lease is a best-effort software layer. It can de-assert PWM after a normal
control-loop stall only while the Python process, watchdog thread, scheduler,
I2C bus, PCA9685, and motor controller remain responsive. It cannot guarantee
shutdown after `SIGKILL`, an OS/kernel crash, process-wide deadlock, blocked I2C
transaction/lock, or a hardware fault. Python `finally`, `Ctrl+C`, and normal
exception handlers have the same process-level limitation. Therefore:

- the spotter-controlled physical motor-battery disconnect is the independent
  emergency stop for every powered test;
- never describe the 0.35 s lease as a hardware watchdog or safety-rated E-stop;
- the independent deadline thread and measured energized-duration checks are
  also best-effort software protections sharing the same Python/I2C path;
- a future hardware OE/watchdog circuit is required for shutdown independent
  of Python and I2C.

## 6. Short floor-steering validation

This is the first powered floor test of the integrated left/right mapping. Use
a flat, closed lane with barriers, no stairs/drop-offs, and nobody in the
vehicle path. Start the program with motor power physically disconnected and
type `FLOOR-INITIALISE-OFF`; it will confirm the initial hardware OFF write
before any phase can be armed. Follow its disconnect/reposition/reconnect
instructions before each individual pulse. Keep the spotter on the battery
disconnect. Do not proceed from raised-wheel evidence directly to a longer
autonomous run. Each phase records its conservative measured energized
duration (from before the first non-zero EN write through confirmed OFF) and
cannot pass if that value exceeds the requested pulse limit.

```bash
python steering_floor_validation.py
```

Follow the script's exact placement and typed confirmations. It must produce
current-build evidence that straight motion, a short left correction, a short
right correction, and the controlled stop all match the expected physical
response without binding, unexpected reversal, or loss of control.

```bash
STEERING_EVIDENCE="PASTE_THE_EXACT_EVIDENCE_PATH_PRINTED_BY_THIS_FLOOR_TEST"
python -m json.tool "$STEERING_EVIDENCE"
python validation_manager.py record short_floor_steering "$STEERING_EVIDENCE"
python validation_manager.py show
```

Do not continue unless every required gate is `PASS`, every recorded evidence
file exists with its recorded SHA-256, and physical motion authorization is
`True`. Gate authorization does not replace the final site, battery, wiring,
wheel, and spotter checks immediately before a run.

## 7. Guarded track runs and telemetry

Use a flat, closed track with barriers and no stairs, drop-offs, traffic, pets,
or people in the path. The spotter remains outside the path with immediate
access to the battery disconnect. Confirm only one CSI owner and start with
0.5 seconds. **Before every `track_run.py` invocation, physically disconnect
motor power.** The runner warms perception, prompts for initialization, opens
the controller and confirms OFF while disconnected, then explicitly tells the
spotter when motor power may be connected. Enter `MOTOR-POWER-CONNECTED`, keep
hands off the chassis and wheels, and enter `TRACK-AREA-CLEAR` only when the
closed track is clear. The runner then rechecks gates and requires a newer,
clear CSI/YOLO result before opening the motion window; there is no unbounded
operator prompt after that freshness proof:

```bash
python validation_manager.py verify
python track_run.py --enable-motors --purpose tuning --seconds 0.5 --run-label initial_tuning
```

If the motion is controlled and the logs are valid, repeat at 2.0 seconds:

```bash
python track_run.py --enable-motors --purpose tuning --seconds 2.0 --run-label track_tuning_01 --headless
```

These are tuning runs because the current controller configuration is marked
`SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED`. They must not be presented as
final results trials. Use their retained measurements to tune the controller,
then set `verification_state` to `PHYSICALLY_TUNED` only when that claim is
supported. Do not edit that state by hand. Retain successful 0.5 s, 1.0 s and
2.0 s tuning summaries, then run the evidence-checking promotion tool with the
three exact JSON paths:

```bash
python promote_controller_tuning.py --latest
```

`--latest` prints the selected run paths before asking for confirmation. To
pin specific files instead, use:

```bash
python promote_controller_tuning.py \
  "/exact/path/to/the_0.5_second_tuning_summary.json" \
  "/exact/path/to/the_1.0_second_tuning_summary.json" \
  "/exact/path/to/the_2.0_second_tuning_summary.json"
```

The tool verifies each summary and linked CSV, requires the 2.0 s candidate to
contain at least five DRIVE rows, five motor-command rows and two distinct
fresh YOLO results, matching the runner's native result-observation minimums.
It preserves every STOP row rather than treating it as motion. It also requires
a chronological ladder from one identical current validated build, records the
source hashes, asks for explicit physical-behaviour confirmation, backs up the
old config, and writes the promoted config atomically. Because changing
`controller_config.json` changes bound
artifact hashes, the revalidation order must restore the physical isolation
boundary:

1. Physically disconnect the motor battery. Run
   `python validation_manager.py record-self-test`, then repeat Step 4 and
   record its new integrated-dry-run evidence while motor power remains
   disconnected.
2. Secure both wheels off the floor and assign the spotter. Launch Step 5 with
   motor power still disconnected; connect only after its initial OFF
   confirmation, then record the new raised evidence.
3. Proceed to Step 6 only after the raised gate re-passes; keep the guarded
   lane and spotter requirements for the short floor pulses.
4. Run `python validation_manager.py verify` again. Only then collect a
   result-labelled run, again launching with motor power disconnected:

```bash
python track_run.py --enable-motors --purpose results --seconds 2.0 --run-label track_trial_01 --headless
```

The runner requires all physical-motion gates. It commands `STOP` on stale or
missing camera data, anything other than a confident full lane, YOLO
startup/staleness/error/frame lag, any configured relevant object detection,
invalid/low motor-bus voltage, an exception, `Ctrl+C`, or command-lease expiry.
The physical disconnect remains required because software stop paths have the
limitations stated above.

A result-labelled run is accepted only with at least five complete `DRIVE`
rows, five non-zero motor-command rows, and two distinct fresh YOLO results.
Fewer observations can still inform tuning but cannot become a report result.

The requested run time is a maximum, not a promise of continuous drive. A
best-effort independent deadline thread requests OFF before the limit, every
PWM refresh carries the same absolute deadline, and the summary records total
and maximum-continuous energized duration. The spotter/disconnect remains the
independent protection against blocked I2C, scheduler, OS, or hardware faults.

A timed command is not a distance measurement. Measure every run independently
and create report statistics without modifying the raw log:

```bash
TRACK_JSON="PASTE_THE_EXACT_SUMMARY_PATH_PRINTED_BY_THIS_TRACK_RUN"
python -m json.tool "$TRACK_JSON"
python analyse_track_run.py "$TRACK_JSON" --distance-m 1.00 --final-error-cm 4.0 --notes "Trial 1, dry floor"
```

Replace the example values with measurements actually taken for that run.
The track summary stores the raw CSV SHA-256. `analyse_track_run.py` refuses a
missing/changed CSV, a mismatched run ID, row count, frame sequence, safety
counts, motion-row count, or unique-YOLO count before producing report output.

## 8. Dashboard use after testing

The dashboard and autonomy/perception runner must not open separate CSI
pipelines. For the current build, stop one process before starting the other.
A future combined system should make the autonomy process the single camera
owner and publish read-only frames/telemetry to the dashboard.

After the final accepted tests, create a final build manifest and preserve it
beside the raw evidence, analyses, screenshots, and model hash:

```bash
python make_build_manifest.py
```
