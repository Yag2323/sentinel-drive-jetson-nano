# Sentinel Drive Jetson Nano Robot Car — Technical Handover

**Audit date:** 2026-07-27
**Audited repository root:** public source snapshot root
**Project identity:** University: **UNKNOWN — not verifiable from repo**; programme: 6FTC2062 BEng Individual Major Project Part B; student: Kathiravan Yagneshvar; supervisor: Kim Siong Wong; academic year: 2026 (`project_evidence_register.md`).
**Repository build identifier declared by source:** `SENTINEL-INTEGRATION-2026.07.27-OVAL1` (`make_build_manifest.py`).

> Public-release note: this is a file-only audit. The source snapshot still
> omits deployment-local physical evidence/results and `manual_motor_control.py`;
> it does not verify a continuous rounded-oval lap.
> Historical inventory entries for `floor_pulse.py`, `raised_validation.py`
> and the `work/` tree are retained below only to document the development
> record. Those legacy/ungated artifacts are intentionally excluded from this
> curated public snapshot.

## Audit scope and evidence status

This document is based only on files present under the audited repository root. It does not use conversational memory, screenshots, pasted terminal output outside the repository, or an assumed live Jetson state.

The checkout is not a complete copy of the deployed Jetson project:

- `evidence/` contains 0 files.
- `outputs/` contains 0 files.
- `results/` does not exist.
- `integration_gates.json` is the locked seed: all eight gates have `passed=false`; `physical_motion_authorized=false`.
- `ipm_config.json` is `SITL_SEED_NOT_PHYSICALLY_CALIBRATED` and has a null calibration timestamp.
- `manual_motor_control.py` is absent even though the physical motor path imports it and the build/gate system requires it.
- No environment preflight JSON, build-manifest JSON, physical IPM sidecar, YOLO benchmark output, integrated dry-run output, generated raised-motor gate evidence, generated steering-gate evidence, or track result is retained locally. The retained `drivetrain_calibration_record.md` is secondary prior evidence, not a generated current-build gate artifact.

Terminology used below:

- **CURRENT FILE FACT:** directly readable from the present checkout.
- **PRIOR TEST EVIDENCE:** a physical or software result described in a repository Markdown record, but without the original generated evidence in this checkout.
- **PROVISIONAL:** configured, derived, simulated, or not physically revalidated for the current checkout.
- **UNKNOWN — not verifiable from repo:** the repository contains no sufficient source or retained evidence for the claim.

## 1. Hardware

### 1.1 Component inventory

| Component | Exact repo-backed identification | Connection or role documented in the repo | Verification status and missing particulars |
|---|---|---|---|
| Compute platform | NVIDIA Jetson Nano | Runs the camera, IPM, YOLO, safety supervisor, control, logging, and dashboard software. | Exact Nano SKU, RAM, storage, carrier board, and Jetson power supply: **UNKNOWN — not verifiable from repo**. |
| Camera | IMX219 CSI camera | CSI `sensor-id=0`; frames are captured through Argus/GStreamer and shared inside one owning process with IPM and YOLO. | Module vendor, lens, ribbon, CSI header position, camera height, pitch, and field of view: **UNKNOWN — not verifiable from repo**. |
| Voltage monitor | INA219 | Dashboard default is Linux I²C bus `1`, 7-bit address `0x41`, bus-voltage register `0x02`; logically observes the motor bus. | Module vendor, shunt resistance/rating, current-register calibration, and physical terminals: **UNKNOWN — not verifiable from repo**. Current measurement must not be claimed from this checkout. |
| PWM controller | PCA9685 | Logical path: Jetson/I²C → PCA9685 → L298N. | Address `0x40` occurs only in legacy `work/phase_stagger_test.py`, so it is **PROVISIONAL** for the current physical path. PWM frequency and oscillator calibration are **UNKNOWN — not verifiable from repo** because `manual_motor_control.py` is missing. |
| Motor driver | L298N | Receives enable/direction signals from PCA9685 and drives left/right motors. | Exact module revision, jumper state, OUT terminal allocation, onboard-regulator use, and voltage drop: **UNKNOWN — not verifiable from repo**. |
| Left drive motor | **UNKNOWN — not verifiable from repo** | L298N left output; logical enable/direction channels listed below. | Legacy dashboard/setup files mark it for replacement; no current repo artifact records whether replacement occurred. Current condition plus motor type, manufacturer, part number, rated voltage, gear ratio, stall current, wire count, and polarity are **UNKNOWN — not verifiable from repo**. |
| Right drive motor | **UNKNOWN — not verifiable from repo** | L298N right output; logical enable/direction channels listed below. | Motor type, manufacturer, part number, rated voltage, gear ratio, stall current, wire count, and wiring polarity: **UNKNOWN — not verifiable from repo**. |
| Wheel encoders | None | `drivetrain_calibration_record.md` explicitly states that wheel-speed feedback is absent; trim is open-loop feed-forward. | CURRENT FILE FACT. |
| Motor battery | Software uses a 2S voltage-envelope assumption | Motor bus is checked at `6.6 V` low and `8.6 V` high; the dashboard display window ends at `8.4 V`. | Actual pack manufacturer, chemistry, cell model/count/topology, capacity, BMS, discharge rating, connector, and charger: **UNKNOWN — not verifiable from repo**. “2S” is a software assumption, not a verified pack model. |
| Physical motor-battery disconnect | Required design element; actual installed device is **UNKNOWN — not verifiable from repo** | Test procedures require a spotter-controlled, physically independent path capable of removing driver/motor power during every powered test. | Actual presence, device model, placement, and effectiveness: **UNKNOWN — not verifiable from repo**. |
| Fuse | No retained hardware record | No file proves its rating, type, holder, or exact placement. | **UNKNOWN — not verifiable from repo**. |
| Xbox controller | A read-only Linux `evdev` probe exists only in `work/xbox_controller_probe.py` | Not part of the active autonomous motor-command architecture; dashboard only reports input-device presence. | Exact controller model, pairing state, event path, and control mapping: **UNKNOWN — not verifiable from repo**. |

### 1.2 PROVISIONAL intended PCA9685-to-L298N signal map

`safe_motor_output.py` labels these as fixed project assignments in its injected-controller/test path. The production path imports the same names from the missing `manual_motor_control.py`, so deployed values cannot be cross-checked here.

| PCA9685 channel | Logical signal | Intended L298N function |
|---:|---|---|
| 0 | `LEFT_ENA` | Left motor enable/PWM |
| 1 | `LEFT_IN1` | Left direction input 1 |
| 2 | `LEFT_IN2` | Left direction input 2 |
| 3 | `RIGHT_IN3` | Right direction input 3 |
| 4 | `RIGHT_IN4` | Right direction input 4 |
| 5 | `RIGHT_ENB` | Right motor enable/PWM |

### 1.3 I²C, GPIO, and power wiring topology

| Required connection detail | Repo-verifiable statement |
|---|---|
| Jetson SDA/SCL physical header pins | **UNKNOWN — not verifiable from repo**. Only logical I²C bus `1` is configured for the dashboard INA219 reader. |
| Jetson/PCA9685 logic supply and ground | **UNKNOWN — not verifiable from repo**. |
| INA219 SDA/SCL and logic supply | **UNKNOWN — not verifiable from repo**; logical address is `0x41`. |
| PCA9685 address select and physical address | **PROVISIONAL:** legacy script uses `0x40`; current production driver source is absent. |
| Common ground among battery, L298N, INA219, PCA9685, and Jetson | **UNKNOWN — not verifiable from repo**. |
| Battery positive → fuse | **UNKNOWN — not verifiable from repo**. |
| Fuse → INA219 `VIN+` | **UNKNOWN — not verifiable from repo**. |
| INA219 `VIN-` → L298N `VS/12V` | **UNKNOWN — not verifiable from repo**. |
| Battery negative → L298N/common ground | **UNKNOWN — not verifiable from repo**. |
| Physical disconnect position | Repo proves only that it must independently remove motor-driver power; exact series position is **UNKNOWN — not verifiable from repo**. |
| L298N `OUT1..OUT4` to the two motors | **UNKNOWN — not verifiable from repo**. |
| PCA9685 hardware `OE` watchdog/output-disable | **PROVISIONAL / UNVERIFIED:** not evidenced as installed; explicitly listed as future work. |

The only defensible logical power statement is: INA219 is intended to observe motor-bus voltage, while the design requires a physically independent disconnect capable of removing power from the L298N/motors. Whether that disconnect is actually installed and effective is **UNKNOWN — not verifiable from repo**. The repository does not contain a trustworthy electrical schematic for the battery, fuse, INA219 `VIN+`/`VIN-`, junctions, grounds, or rail distribution.

## 2. Software environment

| Item | Exact value supported by this repo | Status/source |
|---|---|---|
| Target platform | NVIDIA Jetson Nano | CURRENT FILE FACT; target declaration only |
| OS/distribution | **UNKNOWN — not verifiable from repo** | No retained preflight/system output |
| JetPack | Reported/intended target: JetPack 4.x family | Installed family and exact point release are **UNKNOWN — not verifiable from repo**; the value comes from `INTEGRATION_RUNBOOK.md`, not retained system evidence |
| L4T release | **UNKNOWN — not verifiable from repo** | `jetson_perception_preflight.py` would read `/etc/nv_tegra_release`; its output is absent |
| Kernel | **UNKNOWN — not verifiable from repo** | No system snapshot |
| Python | Python 3.6-compatible code is required/intended | Exact interpreter version/build **UNKNOWN — not verifiable from repo** |
| Virtual environment | `/home/jetson/ai_robot_car/venv_jetson` | Generic public-release example path; installed contents are **UNKNOWN — not verifiable from repo** |
| PyTorch | **UNKNOWN — not verifiable from repo** | Vendored requirements say `torch>=1.7.0`; that is not an installed version |
| torchvision | **UNKNOWN — not verifiable from repo** | Vendored requirements say `torchvision>=0.8.1`; that is not an installed version |
| OpenCV | **UNKNOWN — not verifiable from repo** | Vendored minimum `opencv-python>=4.1.2` is not an installed version |
| NumPy | **UNKNOWN — not verifiable from repo** | Vendored minimum `numpy>=1.18.5` is not an installed version |
| Pillow | **UNKNOWN — not verifiable from repo** | Vendored minimum `Pillow>=7.1.2` is not an installed version |
| PyYAML | **UNKNOWN — not verifiable from repo** | Vendored minimum `PyYAML>=5.3.1` is not an installed version |
| SciPy | **UNKNOWN — not verifiable from repo** | Vendored minimum `scipy>=1.4.1` is not an installed version |
| Requests | **UNKNOWN — not verifiable from repo** | Vendored minimum `requests>=2.23.0` is not an installed version |
| Plot/log utilities | **UNKNOWN — not verifiable from repo** | Vendored minima: `matplotlib>=3.2.2`, `tqdm>=4.41.0`, `tensorboard>=2.4.1`, `pandas>=1.1.4`, `seaborn>=0.11.0`; `thop` is unpinned. None is an installed-version record. |
| PyGObject / `gi` | **UNKNOWN — not verifiable from repo** | Required by `gst_camera_bridge.py`; no installed-version record |
| SMBus binding | **UNKNOWN — not verifiable from repo** | `robot_dashboard.py` tries `smbus`, then `smbus2`; no installed-version record |
| `evdev` | **UNKNOWN — not verifiable from repo** | Used only by legacy `work/xbox_controller_probe.py`; no installed-version record |
| CUDA runtime | **UNKNOWN — not verifiable from repo** | No retained preflight JSON |
| GStreamer and GI versions | **UNKNOWN — not verifiable from repo** | Required element names are known; versions/presence are not retained |
| Required CSI elements | `nvarguscamerasrc`, `nvvidconv`, `videoconvert`, `appsink` | Checked by preflight when run; current installation status is **UNKNOWN — not verifiable from repo** |
| `nvpmodel` mode | **UNKNOWN — not verifiable from repo** | No retained `nvpmodel -q --verbose` output; the query is diagnostic/non-required in preflight |
| `jetson_clocks` applied/status | **UNKNOWN — not verifiable from repo** | No retained `jetson_clocks --show` output; the query is diagnostic/non-required in preflight |
| YOLO source | Ultralytics YOLOv5 v6.0 commit `956be8e642b5c10af4a1533e09084ca32ff4f21f` | Local vendored Git tree is clean |
| Local YOLO runtime-source/config digest | SHA-256 `58c0bb4ee4f910e6054738701718e9e4da67453422430ad4f96a62a664bd7fdc`; 76 `.py`/`.yaml`/`.yml` files | Direct result of local `source_integrity.source_tree_sha256`; this is not the missing Jetson evidence hash and intentionally excludes shell scripts and model weights |
| YOLOv5n weights | 3,952,441 bytes; SHA-256 `649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f` | Direct local file/hash |
| YOLOv5s weights | 14,698,491 bytes; SHA-256 `c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598` | Direct local file/hash |
| Dependency policy | Preserve NVIDIA/JetPack-compatible CUDA PyTorch and system OpenCV; do not run a generic current YOLO requirements install | `INTEGRATION_RUNBOOK.md`, `prepare_yolov5_v6.py` |

Local Windows `__pycache__` files are not evidence of the Jetson environment.

The next Jetson snapshot must retain the raw output of the following commands; `jetson_perception_preflight.py` alone does not capture every requested environment field:

```bash
cd ~/ai_robot_car
source venv_jetson/bin/activate
cat /etc/os-release
uname -a
dpkg-query -W nvidia-jetpack
cat /etc/nv_tegra_release
python - <<'PY'
import sys
print("sys.executable={}".format(sys.executable))
print("sys.prefix={}".format(sys.prefix))
print("sys.version={}".format(sys.version))
try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    print("PyGObject={}".format(getattr(gi, "__version__", "UNKNOWN")))
    print("GStreamer={}".format(Gst.version_string()))
except Exception as error:
    print("GI/GStreamer query failed: {!r}".format(error))
PY
python -m pip freeze --all
sudo nvpmodel -q --verbose
sudo jetson_clocks --show
```

If `dpkg-query` has no `nvidia-jetpack` package, retain that exact failure rather than inferring a JetPack version from L4T.

## 3. File inventory

Paths in this section are relative to the audited root. Per the requested scope, it enumerates all 71 present `.py`, `.ps1`, and `.sh` files, plus two expected/missing Python sources; non-script configs and records are covered in the other sections. Each row gives its purpose and invocation/parameters; the operationally key project-owned calibration, threshold, timeout, and fail-safe constants are tabulated in Sections 4–6. Commands are the repository’s canonical invocation or syntax; listing a physical-test command does not authorize running it.

### 3.1 Project-owned scripts

| Path | Purpose | Canonical invocation | Required confirmation/input |
|---|---|---|---|
| `analyse_track_run.py` | Validate a `PHYSICAL_TRACK_RUN` JSON/CSV pair and write report-ready analysis. | `python analyse_track_run.py "$TRACK_JSON" --distance-m 1.00 --final-error-cm 4.0 --notes "Trial 1, dry floor"` | No confirmation string; both `--distance-m` and `--final-error-cm` are required for `report_eligible=true`, with positive finite distance and finite error. |
| `calibrate_ipm.py` | Interactively capture four real-camera IPM source points without motor access. | `python calibrate_ipm.py [--sensor-id N]`; although `--config PATH` is parsed, every path other than the default `ipm_config.json` is refused | Click bottom-left, bottom-right, top-right, top-left; lowercase `s` saves, lowercase `r` resets, lowercase `q` or Esc cancels. |
| `control_core.py` | Import-only config validation, time-aware PID, mixer, and safety supervisor. | N/A — imported module | None |
| `deploy_integration.ps1` | Back up the remote dashboard, transfer the integration bundle, and verify dashboard SHA-256. | `powershell -ExecutionPolicy Bypass -File .\deploy_integration.ps1 -SafetyConfirmed` | `-SafetyConfirmed` switch; optional `-HostName`, `-UserName`, `-RemoteDirectory`. |
| `deploy_dashboard_fix.ps1` | Transfer the focused dashboard/camera bridge update with backup/hash verification. | `powershell -ExecutionPolicy Bypass -File .\deploy_dashboard_fix.ps1 -SafetyConfirmed` | `-SafetyConfirmed` switch; optional connection arguments. |
| `floor_pulse.py` | **EXCLUDED LEGACY RECORD:** direct 0.60 s floor PWM pulse. | Historical command: `python floor_pulse.py --duty VALUE [--right-trim VALUE]` | `GO`; not included in the public snapshot. |
| `gst_camera_bridge.py` | Canonical GI/GStreamer CSI bridge and source-timestamp metadata. | `python gst_camera_bridge.py` prints the default pipeline; normally imported | None |
| `install_integration_bundle.py` | Preserve IPM, reset provisional controller/gates, and install seed configs safely. | `python install_integration_bundle.py`; only for a strictly validated tuned file: `python install_integration_bundle.py --preserve-controller-tuning` | None |
| `install_motor_calibration.py` | Atomically install persisted motor calibration while preserving direction booleans. | `python install_motor_calibration.py` | None |
| `integrated_control_dry_run.py` | CSI + IPM + asynchronous YOLO + PID/map validation with no motor import. | `python integrated_control_dry_run.py --seconds 45` (options: `--headless --ipm-config PATH --allow-sitl-seed`) | None; SITL-seed mode cannot satisfy the physical gate. |
| `integration_self_test.py` | Hardware-free regression of mapping, PID, safety, voltage, lease, deadline, and error behavior. | `python integration_self_test.py`; gate wrapper: `python validation_manager.py record-self-test` | None |
| `ipm_alignment_diagnostic.py` | Motor-free live alignment diagnostic; never accepted as gate evidence. | `python ipm_alignment_diagnostic.py --warmup-seconds 4 --seconds 8 [--headless] [--config PATH]` | None; selected config must be `PHYSICALLY_CALIBRATED`, otherwise the tool refuses. |
| `ipm_lane.py` | Import-only IPM warp, black-tape mask, line selection, lane state, offset, and confidence. | N/A — imported module | None |
| `ipm_live_dry_run.py` | Validate physical CSI/IPM lane detection without motors. | `python ipm_live_dry_run.py --seconds 30 --headless --expected-centered` | No interactive string; `--expected-centered` is an explicit operator assertion required for PASS; optional `--allow-sitl-seed` is diagnostic only. |
| `jetson_perception_preflight.py` | Non-hardware-mutating Python/L4T/CUDA/package/GStreamer/YOLO identity audit that writes `evidence/perception_preflight_*.json`. | `python jetson_perception_preflight.py` | None |
| `make_build_manifest.py` | Create a hash-addressed build/config/model manifest. | `python make_build_manifest.py` | None |
| `motor_adapter_raised_test.py` | Raised-wheel straight/right/left and command-lease physical validation. | `python motor_adapter_raised_test.py` | In order: `RAISED-INITIALISE-OFF`, `BATTERY-CONNECTED-RAISED`, `STRAIGHT-PASS`, `RIGHT-PASS`, `LEFT-PASS`, `WATCHDOG-PASS`. |
| `motor_mapping.py` | Import-only piecewise per-side logical-command-to-PWM mapping. | N/A — imported module | None |
| `patch_dashboard_camera_rotation.py` | Idempotently port/patch dashboard CSI capture to the shared flip-2 bridge and make a backup when a change is applied. | `python patch_dashboard_camera_rotation.py robot_dashboard.py` | None |
| `prepare_yolov5_v6.py` | Provision/verify pinned YOLOv5 v6.0 source and n/s weights without running pip. | `python prepare_yolov5_v6.py` | None |
| `raised_validation.py` | **EXCLUDED LEGACY RECORD:** raised duty sweep at 25/30/35/40/50/60/70/80%. | Historical command: `python raised_validation.py` | `RAISED`; not included in the public snapshot. |
| `robot_dashboard.py` | Read-only voltage/system/controller-presence/optional CSI HTTP dashboard. | Demo: `python robot_dashboard.py --demo --host 0.0.0.0 --port 8080`; CSI: `python robot_dashboard.py --host 0.0.0.0 --port 8080 --camera csi --camera-source 0 --allow-lan-camera` | No interactive string; non-loopback CSI exposure requires explicit `--allow-lan-camera`; optional `--i2c-bus 1 --ina-address 0x41 --interval 0.5`. |
| `safe_motor_output.py` | Import-only fail-closed PCA9685/L298N adapter, voltage guard, command lease, and deadline stopper. | N/A — imported module | None |
| `source_integrity.py` | Import-only deterministic YOLO tree/Git identity and hashing helpers. | N/A — imported module | None |
| `steering_floor_validation.py` | Three manually armed guarded floor pulses: straight, right correction, left correction. | `python steering_floor_validation.py` (options: `--seconds 0.20..0.60 --steering 0.05..0.18`) | `FLOOR-INITIALISE-OFF`; `READY-STRAIGHT`/`STRAIGHT-PASS`; `READY-RIGHT`/`RIGHT-PASS`; `READY-LEFT`/`LEFT-PASS`; optional note. |
| `track_run.py` | Validation-gated bounded physical autonomous track runner with immutable telemetry. | Tuning: `python track_run.py --enable-motors --purpose tuning --seconds 0.5 --run-label initial_tuning [--headless]`; results: `python track_run.py --enable-motors --purpose results --seconds 2.0 --run-label track_trial_01 --headless` | Dynamic `TRACK-{PURPOSE_UPPER}-{seconds:.1f}` (for example `TRACK-TUNING-0.5`), then `MOTOR-POWER-CONNECTED`, then `TRACK-AREA-CLEAR`. |
| `validation_manager.py` | Record, verify, invalidate, and display hash-bound integration gates; never commands motors. | `python validation_manager.py show`; `python validation_manager.py verify`; `python validation_manager.py confirm-camera-orientation`; `python validation_manager.py record-self-test`; `python validation_manager.py record-ipm-calibration`; or `python validation_manager.py record GATE EVIDENCE_JSON` | Camera confirmation: `UPRIGHT-FLIP-2`. |
| `yolo_csi_benchmark.py` | Measure pinned YOLOv5n/s on the corrected real CSI feed. | s: `python yolo_csi_benchmark.py --model yolov5s --camera-fps 30 --seconds 30 --img-size 320 --confidence 0.55 --headless`; n: same with `--model yolov5n --camera-fps 21` | None; `--allow-cpu` is diagnostic and cannot satisfy the motion gate. |
| `yolov5_runtime.py` | Import-only YOLOv5 detector, async latest-job worker, drawing, and conservative object policy. | N/A — imported module | None |
| `work/higher_speed_straight_compact.py` | **LEGACY:** compact raised/floor 45% boost then 40% cruise test. | Canonical syntax: `python work/higher_speed_straight_compact.py --mode raised --motor-rated-6v`; floor mode substitutes `floor` | `--motor-rated-6v` is an explicit motor-rating assertion, then `RAISED-HIGH` or `FLOOR-HIGH`; **currently broken** because `manual_motor_control.py` and `work/motor_config.json` are absent. |
| `work/higher_speed_straight_test.py` | **LEGACY:** verbose equivalent with single-use JSON raised gate. | Canonical syntax: `python work/higher_speed_straight_test.py --mode raised --motor-rated-6v`; floor mode substitutes `floor` | `--motor-rated-6v` is an explicit motor-rating assertion, then `RAISED-HIGH` or `FLOOR-HIGH`; **currently broken** because `manual_motor_control.py` and `work/motor_config.json` are absent. |
| `work/phase_stagger_floor_test.py` | **LEGACY:** 35% phase-staggered floor pulse. | Canonical syntax: `python work/phase_stagger_floor_test.py` | `FLOOR-PHASE`; **currently broken** because `manual_motor_control.py` and `work/motor_config.json` are absent. |
| `work/phase_stagger_test.py` | **LEGACY:** 35% raised left-first/phase-shifted-right test. | Canonical syntax: `python work/phase_stagger_test.py` | `PHASE`; **currently broken** because `manual_motor_control.py` is absent. |
| `work/robot_dashboard.py` | Superseded camera-less dashboard copy. | `python work/robot_dashboard.py --demo --host 0.0.0.0 --port 8080` or live INA219 without `--demo` | None |
| `work/xbox_controller_probe.py` | Read-only `evdev` discovery and event probe. | `python work/xbox_controller_probe.py [--device /dev/input/eventN]` | Ctrl+C terminates. |

### 3.2 Absent/orphaned source

| Path | Expected purpose | Invocation/status |
|---|---|---|
| `manual_motor_control.py` | Production PCA9685/INA219 controller constants and low-level hardware I/O. | **MISSING.** Imported by physical motor scripts and required by build/gate manifests; exact implementation is **UNKNOWN — not verifiable from repo**. |
| `straight_trim_probe.py` | **UNKNOWN — not verifiable from repo** | Source absent; only orphan `__pycache__/straight_trim_probe.cpython-311.pyc` exists. Do not treat bytecode as maintained source. |

### 3.3 Vendored third-party YOLOv5 v6.0 scripts

These files are a clean pinned upstream dependency, not project-owned application entry points. The project imports `models.experimental`, `utils.datasets`, `utils.general`, and `utils.torch_utils`; it does not directly invoke the remaining utilities in the handover workflow.

| Path | One-line purpose | Project invocation/confirmation |
|---|---|---|
| `yolov5_v6/detect.py` | Upstream image/video detection CLI. | Not invoked by project; upstream help: `python yolov5_v6/detect.py --help`; none. |
| `yolov5_v6/export.py` | Upstream model export CLI. | Not invoked by project; upstream help: `python yolov5_v6/export.py --help`; none. |
| `yolov5_v6/hubconf.py` | PyTorch Hub model-loading definitions plus an upstream multi-input inference demonstration. | Project imports do not invoke it; upstream demonstration from YOLO root: `cd yolov5_v6 && python hubconf.py`; none. |
| `yolov5_v6/train.py` | Upstream training CLI. | Not invoked by project; upstream help: `python yolov5_v6/train.py --help`; none. |
| `yolov5_v6/val.py` | Upstream validation CLI. | Not invoked by project; upstream help: `python yolov5_v6/val.py --help`; none. |
| `yolov5_v6/models/__init__.py` | Models package initializer. | N/A — import-only; none. |
| `yolov5_v6/models/common.py` | Shared YOLO layer/module definitions. | N/A — import-only; none. |
| `yolov5_v6/models/experimental.py` | Experimental/model-loading helpers used by the project runtime. | N/A — imported by `yolov5_runtime.py`; none. |
| `yolov5_v6/models/tf.py` | TensorFlow/Keras model definitions and PyTorch-to-TensorFlow construction test CLI. | Not invoked by project; from the audited root: `cd yolov5_v6 && python models/tf.py [--weights PATH] [--imgsz N] [--batch-size N] [--dynamic]`; none. |
| `yolov5_v6/models/yolo.py` | YOLO model/parser definitions and model build/profile CLI. | Not invoked by project; from the audited root: `cd yolov5_v6 && python models/yolo.py [--cfg YAML] [--device DEVICE] [--profile]`; none. |
| `yolov5_v6/utils/__init__.py` | Utilities package initializer. | N/A — import-only; none. |
| `yolov5_v6/utils/activations.py` | Activation-layer implementations. | N/A — import-only; none. |
| `yolov5_v6/utils/augmentations.py` | Image augmentation and letterboxing helpers. | N/A — import-only; none. |
| `yolov5_v6/utils/autoanchor.py` | Anchor analysis/recalculation helpers. | N/A — import-only; none. |
| `yolov5_v6/utils/callbacks.py` | Training callback registry. | N/A — import-only; none. |
| `yolov5_v6/utils/datasets.py` | Dataset/load-stream helpers used by the project runtime. | N/A — imported; none. |
| `yolov5_v6/utils/downloads.py` | Upstream asset/download helpers. | N/A — import-only; none. |
| `yolov5_v6/utils/general.py` | General preprocessing, NMS, logging, and coordinate helpers used by the runtime. | N/A — imported; none. |
| `yolov5_v6/utils/loss.py` | Training loss functions. | N/A — import-only; none. |
| `yolov5_v6/utils/metrics.py` | Detection metrics. | N/A — import-only; none. |
| `yolov5_v6/utils/plots.py` | Upstream plotting helpers. | N/A — import-only; none. |
| `yolov5_v6/utils/torch_utils.py` | PyTorch device/model helpers used by the runtime. | N/A — imported; none. |
| `yolov5_v6/utils/aws/__init__.py` | AWS utilities package initializer. | N/A — import-only; none. |
| `yolov5_v6/utils/aws/resume.py` | Upstream helper that searches below the current directory and resumes interrupted training jobs. | Not invoked by project; from YOLO root: `cd yolov5_v6 && python utils/aws/resume.py`; none. |
| `yolov5_v6/utils/flask_rest_api/example_request.py` | Example client that posts `zidane.jpg` to the upstream local Flask inference API. | Not invoked by project; with the API already running, from the audited root: `cd yolov5_v6/data/images && python ../../utils/flask_rest_api/example_request.py`; none. |
| `yolov5_v6/utils/flask_rest_api/restapi.py` | Upstream Flask YOLOv5s inference API example. | Not invoked by project; from YOLO root: `cd yolov5_v6 && python utils/flask_rest_api/restapi.py [--port 5000]`; none. |
| `yolov5_v6/utils/loggers/__init__.py` | Upstream logger integrations. | N/A — import-only; none. |
| `yolov5_v6/utils/loggers/wandb/__init__.py` | Weights & Biases package initializer. | N/A — import-only; none. |
| `yolov5_v6/utils/loggers/wandb/log_dataset.py` | Upstream W&B dataset-artifact uploader. | Not invoked by project; from YOLO root: `cd yolov5_v6 && python utils/loggers/wandb/log_dataset.py [--data YAML] [--single-cls] [--project NAME] [--entity NAME] [--name NAME]`; none. |
| `yolov5_v6/utils/loggers/wandb/sweep.py` | Upstream W&B sweep-agent entry point that launches YOLO training from sweep configuration. | Not invoked by project; within a configured W&B sweep-agent environment, from YOLO root: `cd yolov5_v6 && python utils/loggers/wandb/sweep.py [training arguments]`; none. |
| `yolov5_v6/utils/loggers/wandb/wandb_utils.py` | Upstream W&B integration implementation. | N/A — import-only; none. |
| `yolov5_v6/data/scripts/download_weights.sh` | Upstream helper that downloads YOLOv5 s/m/l/x weights. | Not invoked by project; from the audited root: `cd yolov5_v6 && bash data/scripts/download_weights.sh`; none. |
| `yolov5_v6/data/scripts/get_coco.sh` | Upstream helper that downloads the full COCO 2017 dataset. | Not invoked by project; from the audited root: `cd yolov5_v6 && bash data/scripts/get_coco.sh`; none. |
| `yolov5_v6/data/scripts/get_coco128.sh` | Upstream helper that downloads COCO128. | Not invoked by project; from the audited root: `cd yolov5_v6 && bash data/scripts/get_coco128.sh`; none. |
| `yolov5_v6/utils/aws/mime.sh` | Upstream AWS EC2 multipart user-data template. | Not invoked directly by this project; supplied as EC2 user data in the upstream workflow; none. |
| `yolov5_v6/utils/aws/userdata.sh` | Upstream AWS EC2 provisioning/restart script that clones/pulls the upstream branch and installs generic requirements. | Not invoked by this project and incompatible with its pinned-dependency policy; supplied as EC2 user data only in the upstream workflow; none. |

### 3.4 Per-script key constants and parameters

This index makes the key file-level parameters explicit without reproducing source. “See Section” references point to the exact full tables rather than duplicating every value.

| Project-owned path | Key constants/parameters |
|---|---|
| `analyse_track_run.py` | Requires a `PHYSICAL_TRACK_RUN` summary plus matching CSV/run ID/count/hash/frame sequence; `--distance-m` must be positive finite and `--final-error-cm` finite for `report_eligible=true`; output schema `1`. |
| `calibrate_ipm.py` | Default `ipm_config.json` only; point order BL, BR, TR, TL; default sensor `0`; `1280×720` capture to configured `640×360` output at `21 FPS`; 20 fresh warm-up reads, each with a `3.0 s` pull timeout. |
| `control_core.py` | Config files `controller_config.json` and `motor_config.json`; PID/config/supervisor constants are listed in Sections 4.4, 6.1, and 6.3. |
| `deploy_dashboard_fix.ps1` | Public-release defaults: host `jetson-nano.local`, user `jetson`, remote directory `/home/jetson/ai_robot_car`; requires `-SafetyConfirmed`; transfers four camera/dashboard files and verifies dashboard SHA-256. |
| `deploy_integration.ps1` | Same SSH defaults and `-SafetyConfirmed`; transfers the fixed integration bundle, backs up the dashboard, verifies dashboard SHA-256, and runs the remote installer. |
| `floor_pulse.py` | **Excluded legacy record:** `LED0_ON_L=0x06`; pulse `0.60 s`; guard `6.60 V`; duty accepted in `[0.20,0.98]`; right trim default `1.00`. |
| `gst_camera_bridge.py` | `DEFAULT_CSI_FLIP_METHOD=2`; class defaults: sensor `0`, capture `1280×720`, output `640×360`, `30 FPS`; startup `5 s`, read timeout `2 s`; full pipeline in Section 4.3. |
| `install_integration_bundle.py` | Preserves existing `ipm_config.json`; resets controller to provisional seed unless strict `--preserve-controller-tuning` validates `PHYSICALLY_TUNED`; always resets gates from seed; preserves existing `motor_config.json`. |
| `install_motor_calibration.py` | Target `motor_config.json`; preserves both reverse booleans and atomically installs the exact calibration table in Section 4.1. |
| `integrated_control_dry_run.py` | Duration default `45 s`, accepted `[10,300] s`; camera `21 FPS`; operational YOLO/PID values come from `controller_config.json`; acceptance thresholds in Section 6.5. |
| `integration_self_test.py` | Hardware-free fake-controller regression; exercises mapping, config, supervisor, voltage, lease, deadline, I²C-lock, OFF-write, and exception paths using the Section 4/6 configuration. |
| `ipm_alignment_diagnostic.py` | Requires a `PHYSICALLY_CALIBRATED` selected config; `CAMERA_STALE_LIMIT_SECONDS=0.25`; `LANE_CONFIDENCE_MINIMUM=0.55`; measurement default `8 s`, accepted `[3,60] s`; warm-up default `4 s`, accepted `[1,15] s`; never gate evidence. |
| `ipm_lane.py` | Default config `ipm_config.json`; persisted and hard-coded warp/mask/Hough/lane-state/confidence constants are listed in Section 4.2. |
| `ipm_live_dry_run.py` | Duration default `30 s`, accepted `[5,300] s`; default physical config required; `--expected-centered` required for PASS; acceptance thresholds in Section 6.5. |
| `jetson_perception_preflight.py` | Pinned commit and both weight hashes in Section 2; requires Python `>=3.6`; writes schema-2 `evidence/perception_preflight_<UTC>.json`; `jetson_clocks` query is non-required. |
| `make_build_manifest.py` | Build ID `SENTINEL-INTEGRATION-2026.07.27-OVAL1`; pins the commit and both weight hashes; hashes 29 named core code/config/model artifacts including required `manual_motor_control.py`. |
| `motor_adapter_raised_test.py` | Pulse `0.60 s`; refresh `0.10 s`; steering `0.12`; deadline margin `0.08 s`; raised phase/lease acceptance in Section 6.5. |
| `motor_mapping.py` | Config `motor_config.json`; `STOP_EPSILON=0.02`; exact side-specific piecewise breakpoints in Section 4.1. |
| `patch_dashboard_camera_rotation.py` | Positional dashboard path; imports shared `DEFAULT_CSI_FLIP_METHOD`; requires a recognized source layout; changed-file backup suffix `.before-csi-bridge-<UTC>.bak`. |
| `prepare_yolov5_v6.py` | Repository `https://github.com/ultralytics/yolov5.git`, tag `v6.0`, pinned commit in Section 2; weight URLs `https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5n.pt` and `https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt`; exact n/s byte counts and SHA-256 values in Section 2; installs no requirements. |
| `raised_validation.py` | **Excluded legacy record:** `LED0_ON_L=0x06`; duties `0.25,0.30,0.35,0.40,0.50,0.60,0.70,0.80`; each `1.20 s`; guard `6.60 V`. |
| `robot_dashboard.py` | Schema `2`; default host `127.0.0.1`, port `8080`; MJPEG boundary `sentinel-frame`; controller-name terms `xbox`, `x-box`, `gamepad`, `wireless controller`; I²C/address/register and voltage, staleness, history, camera, LAN-exposure, and HTTP values in Sections 4.3 and 6.6; history limit `600` samples. |
| `safe_motor_output.py` | **PROVISIONAL** injected-controller fallback channels `0..5`; production values come from missing `manual_motor_control.py`; direction reversal `0.08 s`, direction settle `0.02 s`; voltage, lease, lock, retry, and deadline values in Section 6.4. |
| `source_integrity.py` | Included suffixes `.py`, `.yaml`, `.yml`; excluded directories `.git`, `__pycache__`, `runs`; deterministic NUL-delimited path/content SHA-256. |
| `steering_floor_validation.py` | Refresh `0.10 s`; pulse default `0.35 s`, accepted `[0.20,0.60] s`; steering default `0.12`, accepted `[0.05,0.18]`; deadline margin `0.08 s`. |
| `track_run.py` | Deadline margin `0.10 s`; initial/rearm perception timeouts `30.0/10.0 s`; poll `0.01 s`; duration `[0.5,10.0] s`; provisional limit `2.0 s`; results minima `5` DRIVE, `5` motion, `2` YOLO results. |
| `validation_manager.py` | Eight required gates/dependency order in Section 7.3; evidence directory `evidence`; pinned commit/n-weight hash; authorization binds `validation_manager.py`, `source_integrity.py`, and `track_run.py`. |
| `yolo_csi_benchmark.py` | Models n/s; duration default `30 s`, accepted `[5,300] s`; image `320`; confidence `0.55`; IoU `0.45`; warm-up `20`; source rates n=`21`, s=`30`; n minimum `50` frames, s minimum `20`. |
| `yolov5_runtime.py` | Default `yolov5n.pt`; object IDs and legacy area constants in Section 6.3; async worker retains only the latest job/result. |
| `work/higher_speed_straight_compact.py` | Missing `work/motor_config.json`; mandatory `--motor-rated-6v`; start `>=8.00 V`; gate `.high_speed_gate`; raised evidence and floor completion each require minimum `>=7.00 V`; boost/cruise `0.45/0.40` for `0.20/0.40 s`; phases `0/2048`. |
| `work/higher_speed_straight_test.py` | Missing `work/motor_config.json`; mandatory `--motor-rated-6v`; start `>=8.00 V`; gate `.higher_speed_raised_gate.json`; same boost/cruise/times/phases; raised minimum `7.00 V`; gate age `1800.0 s`. |
| `work/phase_stagger_floor_test.py` | Missing `work/motor_config.json`; duty `0.35`; phases `0/2048`; start delay `0.30 s`; together time `0.50 s`; imported cutoff unavailable. |
| `work/phase_stagger_test.py` | PCA9685 address `0x40`; `LED0_ON_L=0x06`; duty `0.35`; phases `0/2048`; start delay `0.30 s`; together time `0.50 s`; imported cutoff unavailable. |
| `work/robot_dashboard.py` | Schema `1`; bus `1`, INA address `0x41`, register `0x02`; voltage `7.00/6.60/1.00/8.40 V`; sample `0.50 s`; history `600`; stale `2.00 s`; no camera. |
| `work/xbox_controller_probe.py` | Optional `--device`; auto-match requires EV_ABS + EV_KEY and one of `xbox`, `x-box`, `microsoft`, `gamepad`, `wireless controller`; axes normalized to `[-1,+1]`. |
| `manual_motor_control.py` | **UNKNOWN — not verifiable from repo**; missing. |
| `straight_trim_probe.py` | **UNKNOWN — not verifiable from repo**; source missing. |

| Vendored path/group | Key constants/parameters |
|---|---|
| `yolov5_v6/detect.py` | Upstream defaults include `weights=yolov5s.pt`, `imgsz=640`, confidence `0.25`, IoU `0.45`, device empty/auto; no project-set values. |
| `yolov5_v6/export.py` | Upstream export CLI defaults to `yolov5s.pt`, image `640`, batch `1`, device `cpu`; no project-set values. |
| `yolov5_v6/hubconf.py` | Demonstration loads pretrained `yolov5s` and runs six mixed inputs; no project-set values. |
| `yolov5_v6/train.py` | Upstream training defaults include `weights=yolov5s.pt`, `cfg=''`, `data=data/coco128.yaml`, `hyp=data/hyps/hyp.scratch.yaml`, epochs `300`, batch `16`, image `640`; no project-set values. |
| `yolov5_v6/val.py` | Upstream validation defaults include `data=data/coco128.yaml`, batch `32`, image `640`, confidence `0.001`, IoU `0.6`; no project-set values. |
| `yolov5_v6/models/tf.py` | Defaults: weights `yolov5s.pt`, image `640×640`, batch `1`, static batch unless `--dynamic`; no project-set values. |
| `yolov5_v6/models/yolo.py` | Defaults: cfg `yolov5s.yaml`, device empty/auto, profiling off; no project-set values. |
| `yolov5_v6/utils/aws/resume.py` | Searches below current directory for `last.pt`; starts resumed training, using incrementing distributed ports beginning after `0`; no project-set values. |
| `yolov5_v6/utils/flask_rest_api/example_request.py` | Fixed URL `http://localhost:5000/v1/object-detection/yolov5s`; fixed input `zidane.jpg`. |
| `yolov5_v6/utils/flask_rest_api/restapi.py` | Default port `5000`; host `0.0.0.0`; route `/v1/object-detection/yolov5s`; inference size `640`; force-reloads upstream Hub model. |
| `yolov5_v6/utils/loggers/wandb/log_dataset.py` | Defaults: data `data/coco128.yaml`, project `YOLOv5`, entity `None`, name `log dataset`, single-class false. |
| `yolov5_v6/utils/loggers/wandb/sweep.py` | Reads W&B sweep config keys `batch_size`, `epochs`, `data` and passes remaining known training args; no project-set values. |
| `yolov5_v6/data/scripts/download_weights.sh` | Calls the upstream latest-release resolver for s/m/l/x weights, with a `v5.0` fallback; this is not the project’s pinned-v6.0 provisioning path. |
| `yolov5_v6/data/scripts/get_coco.sh` | Fixed COCO 2017 image/label download and extraction workflow; no project-set values. |
| `yolov5_v6/data/scripts/get_coco128.sh` | Fixed COCO128 archive URL/extraction workflow; no project-set values. |
| `yolov5_v6/utils/aws/mime.sh` | Multipart MIME/cloud-init template; not a directly executable project script. |
| `yolov5_v6/utils/aws/userdata.sh` | Upstream branch/update and generic-requirements bootstrap; intentionally not used by this pinned project. |
| All remaining vendored `.py` rows above | No project-set constants; upstream import APIs/package initializers only. |

## 4. Calibration constants

### 4.1 Persisted drivetrain calibration

Source: `motor_config.json`. Classification: **PRIOR TEST EVIDENCE** for the measured operating points; configured maxima are **PROVISIONAL/UNVERIFIED**.

| Key | Exact persisted value |
|---|---:|
| `reverse_left` | `false` |
| `reverse_right` | `false` |
| `left_trim` | `1.0` |
| `right_trim` | `0.98` |
| `deadband_duty` | `0.6` |
| `cruise_duty` | `0.75` |
| `max_duty` | `0.98` |
| `left_deadband_duty` | `0.6` |
| `right_deadband_duty` | `0.6` |
| `left_cruise_duty` | `0.75` |
| `right_cruise_duty` | `0.735` |
| `left_max_duty` | `0.98` |
| `right_max_duty` | `0.9604` |
| `calibrated_at_voltage` | `7.688 V` |
| `calibration_date` | `2026-07-24` |

`motor_mapping.py` uses `STOP_EPSILON = 0.02`. A smaller magnitude maps to exactly zero. Non-zero commands use two linear segments per side. The exact logical cruise transition is

```text
(0.75 - 0.60) / (0.98 - 0.60) = 15/38
```

The right physical floor remains `0.600`; the right-side correction begins at cruise/max rather than multiplying the floor down to `0.588`.

### 4.2 Physical IPM homography and calibration geometry

**Physical homography matrix:**

```text
UNKNOWN — not verifiable from repo
```

**Physical source image points:** `UNKNOWN — not verifiable from repo`
**Physical metric/world points:** `UNKNOWN — not verifiable from repo`

The current `ipm_config.json` is a **PROVISIONAL / SIMULATED SITL seed**, not physical calibration. It stores no numeric homography matrix; `ipm_lane.py` derives one at runtime with `cv2.getPerspectiveTransform`. No physical tape dimensions or metric world coordinate system are stored. The destination values below are normalized output-image points, not world points.

Point order is bottom-left, bottom-right, top-right, top-left.

```text
source_points_normalized = [
  [0.15625, 1.0],
  [0.84375, 1.0],
  [0.7,     0.6],
  [0.3,     0.6]
]

destination_points_normalized = [
  [0.234375, 1.0],
  [0.765625, 1.0],
  [0.765625, 0.0],
  [0.234375, 0.0]
]
```

No numerical seed homography is printed here: it is not persisted in the repository and would have to be derived at runtime. Under this handover’s no-inference rule, a calculated seed matrix is not substituted for the missing physical homography.

| Persisted IPM seed key | Exact value |
|---|---:|
| `calibration_state` | `SITL_SEED_NOT_PHYSICALLY_CALIBRATED` |
| `calibration_timestamp_utc` | `null` |
| `frame_width` | `640 px` |
| `frame_height` | `360 px` |
| `black_threshold` | `83` |
| `close_kernel_size` | `9 px` |
| `hough_votes` | `25` |
| `minimum_line_length` | `35 px` |
| `maximum_line_gap` | `70 px` |
| `minimum_vertical_ratio` | `1.2` |
| `lookahead_y_fraction` | `0.75` |
| `minimum_lane_width_fraction` | `0.18` |
| `maximum_lane_width_fraction` | `0.75` |

Hard-coded detector parameters in `ipm_lane.py` also affect every observation:

| Detector stage | Exact current parameter/rule |
|---|---|
| Input/point conversion | Input must be a nonempty 3-channel BGR frame; normalized coordinates multiply `(width-1,height-1)` and are converted to `float32`. |
| Blur and mask | Grayscale; Gaussian kernel `5×5`, sigma `0`; inverse threshold uses persisted `black_threshold`; rectangular close uses persisted kernel; rectangular opening is `3×3`. |
| Edge/Hough transform | Canny thresholds `50/150`; Hough rho `1 px`, theta `π/180 rad`; votes/length/gap come from config. |
| Line acceptance | `abs(dy) >= minimum_vertical_ratio × max(1.0,abs(dx))`; target-y extrapolation allowance is `0.12 × image height`; projected x must be finite and in `[0,width-1]`; left/right split is the image centre. |
| Candidate aggregation | Weighted median of projected x, weighted by Euclidean line length; per-line vertical quality is `abs(dy)/max(1.0,length)`. |
| Remembered full-lane width | First valid FULL width is stored; later valid widths update as `0.8 × old + 0.2 × measured`. A single visible side is PARTIAL only when a remembered width exists. |
| Offset | `(lane_centre-centre_x)/max(1.0,centre_x)`, clamped to `[-1.0,1.0]`; absent centre yields `0.0`. |
| Side support | Sum of accepted line lengths divided by `max(1.0,2.0×height)`, then clamped to `[0.0,1.0]`. |
| Width quality | `clamp(1.0 - abs(lane_width-expected_width)/(0.30×expected_width),0.0,1.0)` when lane width exists and expected width is `>1.0 px`. |
| FULL confidence | `clamp(0.15 + 0.25×left_support + 0.25×right_support + 0.20×vertical_quality + 0.15×width_quality,0.0,1.0)`. |
| PARTIAL confidence | `min(0.49, 0.10 + 0.20×max(left_support,right_support) + 0.15×vertical_quality)`. `LOST`/`INVALID_WIDTH` confidence is `0.0`. |

### 4.3 Camera pipeline

Shared orientation constant: `DEFAULT_CSI_FLIP_METHOD = 2`.

Operational calibration/IPM/integrated/track programs instantiate sensor `0`, capture `1280 × 720`, output `640 × 360` BGR, source rate `21/1`. Exact generated pipeline:

```text
nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)21/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true
```

The bridge class default is `30/1`; the report-only YOLOv5s command configures `30/1`. The dashboard defaults to source `30 FPS`, stream `12 FPS`, `640 × 360`, JPEG quality `82`. Configuration of flip 2 is a file fact; upright physical orientation is **UNKNOWN — not verifiable from repo** because the gate/evidence is absent.

### 4.4 PID and control constants

Source: `controller_config.json`. Entire set is **PROVISIONAL** because `verification_state` is not physically tuned.

| Key | Exact value |
|---|---:|
| `verification_state` | `SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED` |
| `kp` | `0.3` |
| `ki` | `0.0` |
| `kd` | `0.04` |
| `integral_limit` | `0.35` |
| `derivative_filter_alpha` | `0.25` |
| `steering_limit` | `0.18` |
| `lane_confidence_minimum` | `0.55` |
| `partial_lane_timeout_s` | `0.5 s` |
| `allow_partial_lane_motion` | `false` |
| `camera_stale_timeout_s` | `0.25 s` |
| `yolo_image_size` | `256 px` |
| `yolo_confidence_threshold` | `0.45` |
| `yolo_iou_threshold` | `0.45` |
| `yolo_stale_timeout_s` | `0.75 s` |
| `yolo_max_frame_lag` | `8 frames` |
| `command_lease_s` | `0.35 s` |
| `motor_voltage_stop_v` | `6.6 V` |
| `maximum_motor_voltage_v` | `8.6 V` |
| `post_command_settle_s` | `0.06 s` |
| `slow_speed_scale` | `0.0` |
| `partial_lane_speed_scale` | `0.0` |

PID `dt` is measured, not assumed. `PIDController.update()` uses `time.monotonic()` when no timestamp is passed. First update has `dt=0.0` and derivative `0.0`; subsequent elapsed time is clamped to `[0.001, 0.25] s`. Derivative is `(error - previous_error) / dt`, filtered as `0.25 × new_derivative + 0.75 × prior_filtered_derivative`. Integral accumulation is time-scaled, clamped to `[-0.35, +0.35]`, and conditionally suppressed when it would deepen output saturation. Output is clamped to `[-0.18, +0.18]`. With `Ki=0.0`, the active provisional controller is PD despite having a PID implementation.

## 5. Measured results

### 5.1 Duty sweep and supply behavior

| Pack/test | Duty condition | Minimum voltage | Same-run start-to-minimum drop | Date/conditions | Evidence status |
|---|---:|---:|---:|---|---|
| Previous pack, individual sweep points | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | Raw logs are absent; repository text gives only an approximate collapse, so no numeric value is reproduced. |
| Replacement pack, individual sweep points | Individual mapping **UNKNOWN — not verifiable from repo**; prior record says the series covered `0.40–0.98` duty | Individual mapping **UNKNOWN — not verifiable from repo**; documented minima range is `7.232–7.304 V` | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | PRIOR TEST EVIDENCE in `drivetrain_calibration_record.md` and `project_evidence_register.md`; raw logs absent. |

### 5.2 Deadband and drivetrain calibration

| Parameter | Left | Right | Date and conditions | Evidence status |
|---|---:|---:|---|---|
| Physical floor breakpoint | `0.600 duty` | `0.600 duty` | 2026-07-24; motor bus `7.688 V`; wheels on the stated floor surface; open-loop PWM | PRIOR TEST EVIDENCE |
| Selected cruise PWM | `0.750 duty` | `0.735 duty` | 2026-07-24; motor bus `7.688 V`; wheels on the stated floor surface; open-loop PWM | PRIOR TEST EVIDENCE |
| Straight-line observation | Exact error **UNKNOWN — not verifiable from repo** | Exact error **UNKNOWN — not verifiable from repo** | Reported bound `<5 cm` over `0.5 m` during the same calibration | PRIOR TEST EVIDENCE; bound only, not an exact measured error |

### 5.3 YOLOv5 performance

| Model/run | Processed-window pipeline FPS | Inference-only FPS | Mean inference latency | Date/conditions | Evidence status |
|---|---:|---:|---:|---|---|
| YOLOv5s real-CSI baseline | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; intended settings are source `30 Hz`, image `320`, confidence `0.55`; no output retained. |
| YOLOv5n current real-CSI benchmark | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; intended settings are source `21 Hz`, image `320`, confidence `0.55`; no output retained. |
| Archived YOLOv5n claim | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | PRIOR TEST EVIDENCE only; secondary register contains approximate values, deliberately excluded because original JSON/CSV is absent. |
| TensorRT benchmark | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; no TensorRT artifact. |

### 5.4 Integrated control

| Metric | Exact value | Date/conditions | Evidence status |
|---|---:|---|---|
| Current-build control-loop rate | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; `evidence/integrated_dry_*.json` absent. |
| Current-build YOLO freshness | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |
| Current-build FULL/usable lane rate | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |
| Current-build DRIVE/STOP/CAUTION frame counts | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |
| Earlier asynchronous control-loop claim | `7.78 FPS` | **UNKNOWN — not verifiable from repo** | PRIOR TEST EVIDENCE in secondary register only; raw artifact absent. |
| Earlier usable-lane claim | `100%` | **UNKNOWN — not verifiable from repo** | PRIOR TEST EVIDENCE in secondary register only; raw artifact absent. |
| Earlier stale-YOLO-frame claim | `0 frames` | **UNKNOWN — not verifiable from repo** | PRIOR TEST EVIDENCE in secondary register only; raw artifact absent. |
| Earlier oscillation claim | `0` | **UNKNOWN — not verifiable from repo** | PRIOR TEST EVIDENCE in secondary register only; definition and raw artifact absent. |

### 5.5 Physical IPM validation

| Metric | Exact value | Date/conditions | Evidence status |
|---|---:|---|---|
| Physical calibration timestamp | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; local config is a SITL seed. |
| Physical homography | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; physical config/evidence absent. |
| Average loop FPS | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; IPM evidence JSON absent. |
| FULL detection rate | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |
| Usable detection rate | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |
| Mean lane confidence | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |
| Mean absolute lane offset | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; evidence absent. |

### 5.6 Distance and drift

| Observation | Exact retained value | Date/conditions | Evidence status |
|---|---:|---|---|
| Straight-line observation distance | `0.5 m` | 2026-07-24; open-loop floor calibration at `7.688 V` | PRIOR TEST EVIDENCE |
| Lateral error | Exact value **UNKNOWN — not verifiable from repo**; retained bound `<5 cm` | 2026-07-24; open-loop floor calibration at `7.688 V` over `0.5 m` | PRIOR TEST EVIDENCE; bound only |
| Autonomous track-run distance | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; no `results/` directory. |
| Other physical distance/drift trials | **UNKNOWN — not verifiable from repo** | **UNKNOWN — not verifiable from repo** | **PROVISIONAL / UNVERIFIED**; no raw logs. |

## 6. Safety systems

The rules and constants in Sections 6 and 7 are **CURRENT FILE FACT**. Their operational and physical effectiveness on this checkout is **PROVISIONAL / UNTESTED** until the corresponding current, hash-bound gates pass; the local gate file presently authorizes no physical motion.

### 6.1 Controller configuration validation

| Guarded value | Accepted range/rule | Trigger behavior |
|---|---|---|
| `kp`, `ki`, `kd`, `integral_limit` | Each finite JSON number in `[0.0, 2.0]` | Configuration load fails. |
| `derivative_filter_alpha` | `[0.0, 1.0]` | Configuration load fails. |
| `steering_limit` | `[0.01, 0.35]` | Configuration load fails. |
| `lane_confidence_minimum` | `[0.40, 0.95]` | Configuration load fails. |
| `partial_lane_timeout_s` | `[0.0, 1.0]` | Configuration load fails. |
| `allow_partial_lane_motion` | Must be a JSON boolean | Configuration load fails. |
| `camera_stale_timeout_s` | `[0.05, 0.50] s` | Configuration load fails. |
| `yolo_confidence_threshold` | `[0.25, 0.60]` | Configuration load fails. |
| `yolo_iou_threshold` | `[0.30, 0.70]` | Configuration load fails. |
| `yolo_stale_timeout_s` | `[0.25, 1.0] s` | Configuration load fails. |
| `yolo_max_frame_lag` | Integer `[1, 30]` | Configuration load fails. |
| `yolo_image_size` | Multiple of 32 in `[224, 416] px` | Configuration load fails. |
| `command_lease_s` | `[0.25, 0.50] s` | Configuration load fails. |
| `motor_voltage_stop_v` | `[6.50, 7.20] V` and below max voltage | Configuration load fails. |
| `maximum_motor_voltage_v` | `[8.40, 8.70] V` | Configuration load fails. |
| `post_command_settle_s` | `[0.02, 0.15] s` and at most half the lease | Configuration load fails. |
| `slow_speed_scale`, `partial_lane_speed_scale` | Each `[0.0, 0.8]` | Configuration load fails. |
| `verification_state` | `SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED` or `PHYSICALLY_TUNED` | Configuration load fails for any other value. |

#### Motor-mapping configuration validation

| Guarded value | Accepted range/rule | Trigger behavior |
|---|---|---|
| Required keys | `reverse_left`, `reverse_right`, `left_trim`, `right_trim`, `deadband_duty`, `cruise_duty`, `max_duty` must exist. | `load_motor_config()` raises; motion initialization cannot continue. |
| Direction flags | Each must be an actual JSON boolean. | Configuration load raises. |
| Numeric values | Trims, generic duty breakpoints, and any supplied side-specific breakpoints must be finite JSON numbers; booleans are refused as numbers. | Configuration load raises. |
| Generic breakpoints | `0.0 <= deadband_duty < cruise_duty <= max_duty <= 1.0`. | Configuration load raises. |
| Trims | Each must satisfy `0.0 < trim <= 1.0`. | Configuration load raises. |
| Resolved side breakpoints | For each side, `0.0 <= side_deadband_duty < side_cruise_duty <= side_max_duty <= 1.0`; omitted side values are resolved from the generic values and trim. | Configuration load raises. |

#### IPM configuration validation

| Guarded value | Accepted range/rule | Trigger behavior |
|---|---|---|
| Required fields | Both normalized quadrilaterals and all threshold/kernel/Hough/geometry fields must exist. | `load_ipm_config()` raises; IPM initialization cannot continue. |
| Source and destination quadrilaterals | Each is exactly four 2-D list points; every coordinate is a finite JSON number in `[0.0,1.0]`; successive cross products must have one sign and absolute shoelace `twice_area >=0.04`; any absolute cross product `<0.000001` is rejected. | Configuration load raises for malformed, out-of-range, crossed, concave, or degenerate points. |
| `black_threshold` | Strict JSON integer in `[1,254]`. | Configuration load raises. |
| `close_kernel_size` | Strict JSON integer, odd, in `[1,31]`. | Configuration load raises. |
| `hough_votes` | Strict JSON integer in `[5,300]`. | Configuration load raises. |
| `minimum_line_length` | Strict JSON integer in `[5,1000]`. | Configuration load raises. |
| `maximum_line_gap` | Strict JSON integer in `[0,500]`. | Configuration load raises. |
| `minimum_vertical_ratio` | Finite JSON number in `[1.0,20.0]`. | Configuration load raises. |
| `lookahead_y_fraction` | Finite JSON number in `[0.10,0.95]`. | Configuration load raises. |
| Lane-width fractions | Finite JSON numbers satisfying `0.05 <= minimum < maximum <= 0.95`. | Configuration load raises. |

### 6.2 Camera and frame freshness

| Guard/fail-safe | Exact setting | Trigger behavior |
|---|---:|---|
| Single camera owner | One Argus owner at a time | Dashboard/autonomy must not open competing CSI pipelines. |
| Pipeline orientation | `flip-method=2` | Shared fixed orientation; physical correctness still requires gate evidence. |
| Appsink queue | `sync=false max-buffers=1 drop=true` | Old frames are discarded rather than queued. |
| Pipeline startup/error wait | State wait `5 s`; startup error-bus poll `0.5 s` | Failure raises and prevents continuation. |
| Frame-read timeout | Bridge default `2.0 s`; YOLO/IPM/integrated use `2.0 s`; track warm-up/active reads use `max(0.01,min(2.0,remaining))`; calibration uses `3.0 s`; dashboard uses `1.0 s`. | Failed read prevents/halts motion in the motion path; the dashboard continues monitoring attempts without granting authority. |
| Trustworthy source age | Reference timestamp must be `GSTREAMER_RUNNING_TIME`; a computed source age down to `-0.050 s` is tolerated and clamped to zero; PTS must advance. | Invalid/repeated/non-increasing timestamps are not fresh. The `PTS_ORIGIN_FALLBACK` can identify progression but is not trusted to authorize motion. |
| Motion camera age | `<=0.25 s` | Missing/nonfinite timestamp → `CAMERA_TIMESTAMP_MISSING`; negative age → `CAMERA_TIMESTAMP_FUTURE`; age above limit → `CAMERA_STALE`; all request STOP. |
| Generic track perception warm-up timeout | Caller-supplied value must be finite and in `[1.0,60.0] s`. | Any value outside that interval raises before readiness processing. |
| Track pre-motion warm-up | Initial `30.0 s`; post-position rearm `10.0 s`; worker poll `0.01 s` | Timeout/refusal leaves motor interface unopened or outputs OFF. |
| Asynchronous YOLO worker | One queued frame; a busy/full worker refuses the new submission; queue poll `0.1 s`; `stop()` joins for at most `5.0 s`. | No result, worker error, stale age, or excess frame lag requests STOP. `stop()` does not independently raise if the daemon thread remains alive after its join timeout. |

### 6.3 Lane, YOLO, and safety-supervisor policy

| Input/guard | Exact current rule | Trigger/state |
|---|---|---|
| Lane status | Only `FULL` can authorize current motion. | `LOST`, `INVALID_WIDTH`, and every non-full/non-partial state → STOP. |
| Partial lane | `allow_partial_lane_motion=false`; scale `0.0`; timeout `0.5 s`. | Every `PARTIAL_LEFT/RIGHT` → `PARTIAL_LANE_MOTION_NOT_VALIDATED` STOP before timeout can authorize anything. |
| Lane confidence | Finite and `>=0.55`. | Invalid → `LANE_CONFIDENCE_INVALID`; below threshold → `LANE_CONFIDENCE_LOW`; STOP. |
| YOLO startup | Must have a result and no worker error. | No result/error → STOP. |
| YOLO age | Finite, nonnegative, `<=0.75 s`. | Outside range → `YOLO_STALE` STOP. |
| YOLO frame lag | Present and `<=8 frames`. | Missing/excess → `YOLO_FRAME_LAG` STOP. |
| Object classes evaluated | COCO IDs `[0,1,2,3,5,7,9,11,56]`. | Any configured relevant detection uses the policy below. |
| Whole-frame obstacle policy | Any person, bicycle, car, motorcycle, bus, truck, or chair anywhere in the frame. | `OBJECT_DETECTED_<CLASS>` STOP. |
| Stop sign | Any detected stop sign. | `STOP_SIGN_DETECTED` STOP. |
| Traffic light | Colour/state is not classified. | `TRAFFIC_LIGHT_UNCLASSIFIED` STOP. |
| Area constants | `SLOW_AREA_RATIO=0.035`, `STOP_AREA_RATIO=0.12`, `STOP_SIGN_AREA_RATIO=0.008`. | Retained legacy constants and currently unused. Per-detection `area_ratio` and `inside_path` fields are descriptive/logged; neither waives or authorizes physical motion. |
| Reduced-speed path | `slow_speed_scale=0.0`, partial scale `0.0`. | Any would-be reduced-speed condition becomes STOP; `CAUTION` is effectively unreachable with current config/policy. |
| Clear path | All camera, lane, YOLO, and object checks pass. | `DRIVE / ALL_GATES_VALID / speed_scale=1.0`. |

### 6.4 Motor mapping, voltage, lease, and output fail-safe

| Guard/fail-safe | Exact behavior | Trigger result |
|---|---|---|
| Zero region | Command magnitude `<0.02` maps to `0.0`. | Motor output is OFF, not held at deadband. |
| Logical command bounds | Each side clamped to `[-1.0, 1.0]`; nonfinite refused. | Nonfinite → `NONFINITE_COMMAND` and STOP. |
| Reverse policy | `allow_reverse=false` in autonomous adapter. | Negative command → `REVERSE_COMMAND_REFUSED` and STOP. |
| Mixer inner-wheel floor | Steering clamped to `±0.8 × base`; inner normalized command remains at least `20%` of base. | Prevents intentional pivot/reverse from mixer. |
| Direction change | Both enable channels set to zero first; reversal wait `0.08 s`; direction settle `0.02 s`. | Direction is not switched under PWM. |
| Voltage before command | Read on every command. | Nonfinite → `BATTERY_READING_INVALID`; `<6.6 V` → `BATTERY_UNDERVOLTAGE`; `>8.6 V` → `BATTERY_OVERVOLTAGE_OR_SENSOR_FAULT`; each calls all-off and raises. |
| Settled voltage | Read after activation or an absolute duty change `>=0.10`, after `0.06 s`. | Same voltage guards apply. |
| Command lease | `0.35 s`. | If command refresh ceases while Python/scheduler/I²C remain responsive, watchdog requests all-off and latches `COMMAND_LEASE_EXPIRED`; restart is refused. |
| Watchdog polling | `max(0.02, min(0.10, lease/4)) = 0.0875 s`; I²C lock acquisition timeout `0.05 s`. | A lock timeout after lease expiry latches `I2C_LOCK_UNAVAILABLE_AFTER_LEASE`; retry-only lock contention does not set that fault. The physical disconnect remains the independent protection. |
| Command transaction exception | Any exception during voltage/direction/PWM transaction. | If hardware is not already confirmed OFF, attempts `COMMAND_TRANSACTION_FAILED` all-off and latches any all-off failure; if a prior guard already confirmed OFF, it does not relabel that stop. The original exception is preserved. |
| Failed all-off | Hardware OFF write raises. | Software state fails closed; `hardware_off_retry_required=true`; watchdog retries; physical disconnect may be the only guarantee. |
| Close | Up to 3 all-off attempts separated by `0.02 s`; watchdog join `1.0 s`. | Close propagates unresolved OFF/controller error. |
| Optional absolute motion deadline | When supplied, it must be a finite monotonic deadline and is checked around command writes; `track_run.py` supplies it on each refresh and also uses an independent `MotionDeadlineStopper`. The deliberate raised lease-expiry check omits it. | An expired/invalid supplied deadline calls STOP; the helper is best effort and cannot replace physical disconnect. |
| Deadline-stopper cancellation | Cancellation joins its daemon thread for `1.0 s`. | A surviving thread records `DEADLINE_STOP_THREAD_DID_NOT_FINISH`; gate/run validity rejects a non-null stopper error. |

The software lease/deadline cannot guarantee shutdown after `SIGKILL`, OS/kernel failure, process-wide deadlock, scheduler loss, blocked I²C/lock, or PCA9685/L298N failure. A spotter holding the physical motor-battery disconnect is mandatory.

### 6.5 Gate, test, and track-run guards

| Context | Exact guard or acceptance rule | On failure |
|---|---|---|
| Every recorded gate | Exact gate-specific `test` ID; `passed=true`; `artifact_snapshot_stable=true`; `artifact_sha256_at_test` is a dictionary; prerequisite gates already pass; recorded evidence and bound artifact hashes remain current. | Evidence is rejected, or a previously accepted gate becomes WAIT/STALE. |
| Motion authorization | All eight gates passed/current with evidence and artifact SHA-256; authorization identity also binds `validation_manager.py`, `source_integrity.py`, `track_run.py`. | A failed gate remains/returns WAIT or STALE and locks authorization; if all gates pass but an authorization-artifact hash fails, authorization integrity becomes STALE. In both cases `physical_motion_authorized=false` and the motor interface is not opened. |
| Camera-orientation gate | `physical_motor_commands=false`; `operator_confirmed_upright=true`; strict integer `flip_method=2`; generation requires the exact token `UPRIGHT-FLIP-2`. | Evidence rejected. |
| Motor-mapping self-test gate | `physical_motor_commands=false`; strict integer return code `0`; exact marker `INTEGRATION SELF-TEST: PASS`; the producer subprocess timeout is `30.0 s`. | Evidence rejected or producer aborts. |
| YOLO gate | YOLOv5n; `motion_gate_eligible=true`; `motor_commands=false`; CUDA; FP16; source `21 Hz`; flip 2; image `320`; finite confidence and IoU each in `(0.0,1.0]`; pinned clean commit/weights; tracked/current source clean; `source_integrity_stable_during_measurement=true`; full measurement window; at least 50 measured frames; positive throughput. | Evidence rejected. |
| Physical-IPM calibration gate | `motor_commands=false`; `calibration_state=PHYSICALLY_CALIBRATED`; `evidence_image` resolves to an existing file. | Evidence rejected. |
| IPM live gate | `motor_commands=false`; physical/default IPM; full window; at least 50 frames; FULL in `[90.0,100.0]%`; usable in `[95.0,100.0]%`; average loop `>=5.0 FPS`; expected centered; mean absolute offset in `[0.0,0.08]`; mean confidence in `[0.55,1.0]`. Its producer additionally requires a successfully written image and stable artifacts before setting `passed=true`. | Evidence rejected. |
| Integrated dry gate | `physical_motor_commands=false`; CUDA FP16 YOLO; physical/default IPM; full window; at least 50 frames; FULL in `[80.0,100.0]%`; usable in `[95.0,100.0]%`; YOLO fresh in `[90.0,100.0]%`; control `>=5.0 FPS`; at least 1 DRIVE, 1 obstacle STOP, 1 lane STOP, and 20 controlled frames; 0 sign mismatches; 0 duty violations; saturation in `[0.0,20.0]%`; derivative spikes in `[0.0,10.0]%`; flips in `[0.0,15.0]` per 100 controlled frames. Its producer additionally requires a successfully written image and stable artifacts before setting `passed=true`. | Evidence rejected. |
| Raised adapter pulse parameters | Requested `0.60 s`; command deadline `0.52 s` (`0.08 s` margin); refresh `0.10 s`; steering `0.12`. | Stop/refuse evidence. |
| Raised adapter gate | `physical_motor_commands=true`; `wheels_raised=true`; lease software and visual confirmations true; exact lease reason `COMMAND_LEASE_EXPIRED`; hardware OFF true/no retry; exactly 1 watchdog trip; lease energized duration `>0` and `<=command_lease_seconds+0.20` (`<=0.55 s` at the current `0.35 s` lease); pulse `>0` and `<=0.60 s`; maximum actual duration `>0` and `<=pulse`. Exact ordered phases are `STRAIGHT`, `RIGHT_CORRECTION`, `LEFT_CORRECTION`; every phase is `PASS` and operator-confirmed, has both mapped duties in `[0,1]`, actual duration `>0` and `<=pulse`, and a deadline-stopper dictionary whose `stop_error` is null. | Evidence rejected; test code stops/refuses and directs use of the physical disconnect on shutdown failure. |
| Floor-steering gate | True declarations: `physical_motor_commands`, `wheels_on_floor`, clear-lane-and-spotter confirmation, straight/left/right passes, and controlled stop; `autonomous_input_used=false`; maximum pulse `>0` and `<=0.60 s`; maximum actual duration `>0` and `<=pulse`; hardware OFF true/no retry; watchdog fault null/trips `0`. Exact ordered phases are `STRAIGHT`, `RIGHT_CORRECTION`, `LEFT_CORRECTION`; each is `PASS` and operator-confirmed, has duties in `[0,1]`, actual duration `>0` and `<=pulse`, and stopper `stop_error=null`. Right correction requires left duty > right duty; left correction requires right duty > left duty. Producer bounds are pulse `[0.20,0.60] s` (default `0.35 s`), steering `[0.05,0.18]` (default `0.12`), deadline margin `0.08 s`, refresh `0.10 s`, and exact arm/PASS tokens. | Evidence rejected; remaining phases are refused after a failed phase. |
| Physical track command line | `--enable-motors`; duration `[0.5,10.0] s`; physical IPM; CUDA; all gates current. | Refused before motion. |
| Provisional controller track limit | Non-`PHYSICALLY_TUNED` config permits tuning only and at most `2.0 s`. | Results purpose or longer run refused. |
| Track deadline | Motor deadline is requested end minus `0.10 s`. | Deadline thread and per-refresh check request all-off. |
| Track runtime stop paths | No frame, any supervisor STOP, `q`, Ctrl+C, SIGTERM/SIGHUP/SIGQUIT, exception, or finalizer. | PID reset as applicable; motor stop/close/all-off attempted. |
| Tuning run validity | `completed=true`; no recorded error; more than 0 logged frames, DRIVE frames, and motion-command frames; motor-output object exists; no watchdog fault/trip; hardware OFF confirmed/no retry; total energized duration no longer than requested; deadline stopper absent or `stop_error=null`. | `run_successful=false`. |
| Results run validity | Tuning rules plus `purpose=results`, `PHYSICALLY_TUNED`, at least 5 DRIVE rows, 5 motion rows, and 2 distinct YOLO results. | `run_valid_for_results=false`. |

For IPM-live and integrated-dry evidence, the producer requires an image write before it can emit `passed=true`, but `validation_manager.py` does not independently recheck that image path or file when recording those two gates. It does independently check image existence for the physical-IPM calibration gate.

The integrated-dry producer defines its control-quality counters exactly as follows: a steering-sign mismatch is counted when `abs(lane_offset) >= 0.03` and `PID_output × lane_offset <= 0.0`; saturation is counted when `abs(PID_output) >= 0.98 × steering_limit`; a derivative spike is counted when `abs(D_term) >= 0.80 × steering_limit`; a steering flip is a sign change between consecutive nonzero steering samples for which `abs(PID_output) >= 0.04`, and the remembered sign resets on STOP. A nonzero mapped-duty violation is counted when a left/right mapped duty falls outside that side’s persisted `[deadband,max]` interval.

### 6.6 Dashboard-only guards

| Guard/status | Exact threshold/behavior |
|---|---|
| Motion authority | Hard-coded false; dashboard imports no motor module and exposes no command endpoint. |
| HTTP methods | GET/HEAD supported; POST/PUT/PATCH/DELETE return 405. |
| INA plausibility | Nonfinite, `<0.0 V`, or `>16.0 V` becomes sensor fault. |
| Battery absent | `<1.0 V`. |
| Critical monitoring band | `1.0 V <= voltage < 6.6 V`. |
| Manual-recovery monitoring band | `6.6 V <= voltage < 7.0 V`. |
| Autonomous-capable observation | `>=7.0 V`; this still does not authorize motion. |
| Display maximum | `8.4 V`; voltage is not state of charge. |
| Telemetry stale | `>2.0 s`. |
| Camera stale | `>2.5 s`. |
| Camera monitor startup/close | Startup readiness wait `12.0 s`; close joins for `4.0 s`. |
| HTTP connection timeout | Each accepted dashboard connection uses a socket timeout of `5.0 s`. |
| MJPEG latest-frame wait | Each stream iteration waits at most `2.0 s` for a frame newer than the last emitted sequence; the camera monitor method default is also `2.0 s`. |
| HTTP server polling | `serve_forever(poll_interval=0.25)`. |
| CLI TCP port | Strict integer in `[1,65535]`. |
| CLI INA219 address | Parsed as decimal or hexadecimal and restricted to the 7-bit device range `[0x03,0x77]`. |
| CLI camera source | Strict non-negative integer. |
| Sample interval | `[0.20,1.00] s`, default `0.50 s`. |
| Sampler close | Join timeout is `max(3.0, interval × 4.0) s`. |
| LAN camera exposure | Non-loopback CSI requires explicit `--allow-lan-camera`; feed is unauthenticated and must be on a trusted LAN. |

### 6.7 Legacy test guards

| Script | Exact retained guards | Status |
|---|---|---|
| `floor_pulse.py` | Duty `[0.20,0.98]`, pulse `0.60 s`, voltage stop `<6.60 V`, token `GO`, Ctrl+C/finally all-off. | **LEGACY / UNGATED**; missing low-level driver locally. |
| `raised_validation.py` | Steps `0.25,0.30,0.35,0.40,0.50,0.60,0.70,0.80`, each `1.20 s`, voltage stop `<6.60 V`, token `RAISED`. | **LEGACY / UNGATED**. |
| `work/phase_stagger_*` | Duty `0.35`; left phase `0`, right phase `2048`; start delay `0.30 s`; together `0.50 s`; imported `MIN_BATTERY_VOLTAGE`. | **LEGACY**; exact imported cutoff **UNKNOWN — not verifiable from repo**. |
| `work/higher_speed_straight_test.py` | Mandatory `--motor-rated-6v`; starting voltage `>=8.00 V`; boost `0.45` for `0.20 s`; cruise `0.40` for `0.40 s`; raised minimum `7.00 V`; gate max age `1800.0 s`; configuration tolerance `0.001`; floor gate single-use. | **LEGACY / UNGATED**. |
| `work/higher_speed_straight_compact.py` | Mandatory `--motor-rated-6v`; starting voltage `>=8.00 V`; same boost/cruise timings; file-age gate `1800 s`; raised evidence and floor completion each require minimum `>=7.00 V`; raised/floor tokens; floor permission single-use. | **LEGACY / UNGATED**. |

### 6.8 Artifact, deployment, and reporting integrity safeguards

| Context | Exact guard/timeout | Failure behavior |
|---|---|---|
| PowerShell deployment | Both deployment scripts require `-SafetyConfirmed`; they restrict user/host/remote-directory characters, require all transfer files, create a timestamped remote dashboard backup if the remote dashboard is present, stop on SSH/SCP error, and compare remote dashboard SHA-256 with the local file. | Deployment/update refuses or stops; the recovery backup is retained when created. The switch records an operator assertion, not a measured proof that motor power is disconnected. |
| Integration installer | Existing IPM is preserved; existing controller and gate files are backed up; controller is reset to the provisional seed unless `--preserve-controller-tuning` passes strict validation and has `PHYSICALLY_TUNED`; the gate seed is always reinstalled; motor config must exist and be valid JSON. | Exit `1` on missing/invalid seed/config or refused tuning preservation. Seed writes use a temporary file plus `os.replace`. |
| Motor-calibration installer | Existing direction fields must be JSON booleans; existing config is backed up; the persisted calibration in Section 4.1 is written through a same-directory temporary file, flushed, `fsync`ed, then replaced atomically. | Installation refuses and leaves the original/backup recoverable. |
| Pinned YOLO provisioning | Existing source must be the clean pinned commit; existing n/s weights must match both exact SHA-256 and byte count; mismatches are never overwritten. Downloads use a `60 s` URL timeout, temporary file, hash/byte verification, `fsync`, and atomic replace; recursive cleanup is permitted only for a direct project child whose name starts `.yolov5_v6.`. | Provisioning aborts, preserves mismatched existing artifacts, and removes only a validated temporary target. |
| YOLO benchmark input/integrity | Duration `[5.0,300.0] s`; warm-up frames `[0,60]`; weights present/hash-correct; source at pinned clean commit; camera source age `<=0.25 s`; source/weights/artifact snapshot unchanged across measurement. CPU is allowed only with explicit `--allow-cpu` and cannot produce motion-gate-eligible evidence. | Refusal or `passed=false`; no motor modules are used. |
| Jetson preflight subprocesses | External diagnostic command timeout `8 s`; both `nvpmodel -q --verbose` and `jetson_clocks --show` are explicitly non-required. | Required check failure makes preflight FAIL; either optional diagnostic failure is WARN and retained in the JSON. |
| Build manifest | All 29 named core artifacts must exist, including `manual_motor_control.py`; YOLO commit/tree and both weights must match their pinned identities. | `build_complete=false` and exit `1`; missing/mismatched items are listed. |
| Track-result analysis | Strict finite JSON; exact `PHYSICAL_TRACK_RUN`; source `run_successful=true`; linked CSV exists and matches its SHA-256; nonempty rows share run ID; row count and sequential frame IDs match; unique-YOLO/motion/safety counts match summary. Distance, when supplied, is finite and positive; final error, when supplied, is finite. | Analysis refuses. Report eligibility additionally requires source `run_valid_for_results=true` and both manual measurements. |
| IPM alignment diagnostic | Duration `[3.0,60.0] s`; warm-up `[1.0,15.0] s`; camera stale limit remains `0.25 s`; confidence check `0.55`. It imports no motor or gate manager module and marks `validation_evidence=false`. | Produces diagnostics only and can never satisfy a gate. |

## 7. State machine

The transition logic below is **CURRENT FILE FACT**. Its physical timing, shutdown effectiveness, and exercised-state coverage are **PROVISIONAL / UNTESTED** for this checkout because every local gate is unrecorded and no generated state-count evidence is present.

### 7.1 Lane-observation state machine

| Current/input condition | Next/output state | Motion implication |
|---|---|---|
| Both left/right candidates found; width within `0.18–0.75` of warped width | `FULL` | May proceed only if confidence and all other guards pass. |
| Both sides found; width outside that interval | `INVALID_WIDTH` | STOP. |
| Only left side found and a remembered full-lane width exists | `PARTIAL_LEFT` | Diagnostic only; current config forces STOP. |
| Only right side found and a remembered full-lane width exists | `PARTIAL_RIGHT` | Diagnostic only; current config forces STOP. |
| No usable pair/side, or no remembered width for a single side | `LOST` | STOP. |

### 7.2 Safety-supervisor state machine

| State | Entry condition | Exit/transition |
|---|---|---|
| `STOP` | Any camera, lane, confidence, YOLO, frame-lag, object, or zero-scale guard fails. | A later fully valid observation may produce DRIVE; physical output code explicitly stops and resets PID on STOP. |
| `CAUTION` | A validated reduced-speed condition has a nonzero scale below 1.0. | **Currently unreachable** because both reduced scales are `0.0` and partial motion is false; such conditions become STOP. |
| `DRIVE` | Fresh/trustworthy camera, confident FULL lane, fresh/error-free/frame-aligned YOLO, no relevant object, all configuration checks. | Any failed guard immediately returns STOP. |

### 7.3 Validation-gate state machine

| State/transition | Implemented condition |
|---|---|
| `WAIT / UNRECORDED` | Seed/default or no accepted evidence. |
| `PASS / CURRENT` | Expected evidence test ID, `passed=true`, stable test-time artifact snapshot, evidence SHA, current hashes, and prerequisite gates validate. |
| `PASS → STALE` | Bound code/config/evidence disappears or changes, or an upstream prerequisite is re-recorded/invalidated. |
| `LOCKED → authorized` | All eight required gates are CURRENT and authorization-artifact hashes are captured. |
| `authorized → LOCKED` | One or more required gates no longer pass. |
| `authorized/CURRENT → unauthorized/STALE` | All gates still pass, but an authorization-artifact hash is missing or mismatched. |

Required dependency order is: camera orientation and motor-mapping self-test; YOLO regression and physical IPM calibration; IPM live dry run; integrated dry run; raised motor adapter; short floor steering.

| Gate | Exact evidence `test` ID | Classification when accepted | Prerequisites | Hash-bound artifacts |
|---|---|---|---|---|
| `camera_orientation` | `PHYSICAL_CSI_CAMERA_ORIENTATION` | `PHYSICALLY_VERIFIED` | None | `validation_manager.py`, `patch_dashboard_camera_rotation.py`, `robot_dashboard.py`, `gst_camera_bridge.py` |
| `motor_mapping_software` | `INTEGRATION_SOFTWARE_SELF_TEST` | `SOFTWARE_VERIFIED` | None | `validation_manager.py`, `integration_self_test.py`, `control_core.py`, `motor_mapping.py`, `safe_motor_output.py`, `motor_config.json`, `controller_config.json` |
| `yolo_current_regression` | `YOLOV5N_REAL_CSI_BENCHMARK` | `SOFTWARE_VERIFIED_ON_PHYSICAL_CSI` | `camera_orientation` | `validation_manager.py`, `source_integrity.py`, `gst_camera_bridge.py`, `yolov5_runtime.py`, `yolo_csi_benchmark.py`, `yolov5_v6`, `yolov5_v6/yolov5n.pt` |
| `ipm_physical_calibration` | `PHYSICAL_IPM_CALIBRATION` | `PHYSICALLY_VERIFIED` | `camera_orientation` | `validation_manager.py`, `gst_camera_bridge.py`, `ipm_lane.py`, `calibrate_ipm.py`, `ipm_config.json` |
| `ipm_live_dry_run` | `PHYSICAL_CSI_IPM_DRY_RUN` | `SOFTWARE_VERIFIED_ON_PHYSICAL_CSI` | `camera_orientation`, `ipm_physical_calibration` | `validation_manager.py`, `gst_camera_bridge.py`, `ipm_lane.py`, `ipm_live_dry_run.py`, `ipm_config.json` |
| `integrated_control_dry_run` | `INTEGRATED_PERCEPTION_CONTROL_DRY_RUN` | `SOFTWARE_VERIFIED_ON_PHYSICAL_CSI` | camera, motor mapping, YOLO, physical IPM, IPM live | `validation_manager.py`, `source_integrity.py`, `gst_camera_bridge.py`, `ipm_lane.py`, `ipm_config.json`, `yolov5_runtime.py`, `yolov5_v6`, `yolov5_v6/yolov5n.pt`, `control_core.py`, `motor_mapping.py`, `motor_config.json`, `controller_config.json`, `integrated_control_dry_run.py` |
| `raised_motor_adapter` | `RAISED_STEERING_AND_COMMAND_LEASE_VALIDATION` | `PHYSICALLY_VERIFIED` | All preceding gates through integrated dry | `validation_manager.py`, `manual_motor_control.py`, `safe_motor_output.py`, `motor_mapping.py`, `motor_config.json`, `controller_config.json`, `motor_adapter_raised_test.py` |
| `short_floor_steering` | `SHORT_FLOOR_STEERING_VALIDATION` | `PHYSICALLY_VERIFIED` | All preceding gates through raised adapter | `validation_manager.py`, `manual_motor_control.py`, `safe_motor_output.py`, `motor_mapping.py`, `motor_config.json`, `controller_config.json`, `steering_floor_validation.py` |

After all eight gates pass, authorization additionally binds `validation_manager.py`, `source_integrity.py`, and `track_run.py`.

### 7.4 Motor-output state machine

| State/transition | Implemented condition/action |
|---|---|
| Construction → `INITIALISED_OFF` | `controller.all_off()` must succeed before watchdog starts. |
| OFF → command-write-in-progress/active | Finite, allowed normalized command; valid voltage; lease valid; deadline valid if supplied; direction set with EN off; nonzero duty written. |
| Active → OFF attempt | Zero command, supervisor STOP, explicit stop, voltage guard, supplied deadline, lease expiry, transaction exception, operator interrupt, close, or finalizer attempts a hardware all-off write. A successful write reaches OFF; failure reaches retry-required/fault-latched and does not prove hardware is OFF. |
| OFF write fails → retry-required/fault-latched | Software state is zeroed, retry flag set, watchdog retries; restart is refused. |
| Any watchdog fault → no refresh | `_assert_lease_refresh_allowed_locked()` stops and raises. |

### 7.5 Track-run lifecycle

| Phase | Preconditions/transition |
|---|---|
| Locked validation | Require `--enable-motors`, duration, all gates/current hashes, physical IPM, CUDA, and purpose/tuning rules. |
| Motor-free perception warm-up | CSI/IPM/YOLO must reach clear current state within `30.0 s`; motor interface does not exist yet. |
| Explicit OFF initialization | Operator types dynamic track token while motor battery remains disconnected. |
| Hardware OFF object | Reverify gates; construct `SafeMotorOutput`; initial all-off must be confirmed. |
| Physical readiness | Operator types `MOTOR-POWER-CONNECTED` and `TRACK-AREA-CLEAR`. |
| Fresh rearm | A newer clear perception result must pass within `10.0 s`; outputs remain OFF. |
| Bounded active loop | Absolute motor deadline, per-frame supervisor, logging, voltage/lease checks. |
| Final OFF attempt | Deadline, STOP, error, signal, keyboard input, or normal completion attempts motor stop/close and camera/worker close. Hardware OFF is confirmed only when `hardware_off_confirmed=true`, no OFF retry is required, and no close/deadline-stopper error is recorded. |

### 7.6 Exercised frame counts retained in the repo

| State/count requested | Retained count |
|---|---:|
| Integrated `DRIVE` frames | **UNKNOWN — not verifiable from repo** |
| Integrated `STOP` frames | **UNKNOWN — not verifiable from repo** |
| Integrated `CAUTION` frames | **UNKNOWN — not verifiable from repo** |
| Lane `FULL` frames | **UNKNOWN — not verifiable from repo** |
| Lane `PARTIAL_LEFT` frames | **UNKNOWN — not verifiable from repo** |
| Lane `PARTIAL_RIGHT` frames | **UNKNOWN — not verifiable from repo** |
| Lane `INVALID_WIDTH` frames | **UNKNOWN — not verifiable from repo** |
| Lane `LOST` frames | **UNKNOWN — not verifiable from repo** |
| Obstacle-STOP frames | **UNKNOWN — not verifiable from repo** |
| Lane-STOP frames | **UNKNOWN — not verifiable from repo** |
| Track-run DRIVE/motion rows | **UNKNOWN — not verifiable from repo** |
| Unique YOLO results during a track run | **UNKNOWN — not verifiable from repo** |

The code’s acceptance minima (not observed results) are: integrated dry run at least 50 total frames, 20 controlled frames, 1 DRIVE frame, 1 obstacle-STOP frame, and 1 lane-STOP frame; results track run at least 5 DRIVE rows, 5 motion rows, and 2 distinct YOLO results.

## 8. Test history

### 8.1 Dated observations documented in repository records

| Date | Test/observation | Exact retained conditions | Outcome | Evidence path/status |
|---|---|---|---|---|
| 2026-07-24 | Physical drivetrain floor calibration | Motor bus `7.688 V`; wheels on stated floor surface; open-loop PWM | Breakpoints `0.600/0.600`; cruise `0.750/0.735`; retained bound `<5 cm` over `0.5 m` | `drivetrain_calibration_record.md`, `motor_config.json`; PRIOR TEST EVIDENCE |
| 2026-07-26 | CSI live-image observation | Exact command/environment **UNKNOWN — not verifiable from repo** | Live image described as observed | `project_evidence_register.md`; PRIOR TEST EVIDENCE only; generated evidence absent |
| 2026-07-26 | Physical CSI orientation observation | `flip-method=2`; exact command/environment **UNKNOWN — not verifiable from repo** | Upright image described as observed | `project_evidence_register.md`; PRIOR TEST EVIDENCE only; generated evidence absent |

### 8.2 Undated prior claims retained only in the secondary register

| Date | Claimed test | Recorded claim | Repo-verifiable outcome |
|---|---|---|---|
| **UNKNOWN — not verifiable from repo** | YOLOv5n real-CSI benchmark | Secondary register contains approximate performance values | Exact measurements **UNKNOWN — not verifiable from repo**; original JSON/CSV absent. |
| **UNKNOWN — not verifiable from repo** | Physical person detection | Detection observed | PRIOR TEST EVIDENCE only; screenshot/run absent. |
| **UNKNOWN — not verifiable from repo** | Earlier asynchronous lane/YOLO run | `7.78 FPS`, `100%` usable lane, `0` stale YOLO frames, `0` oscillations | PRIOR TEST EVIDENCE only; archived artifact absent. |
| **UNKNOWN — not verifiable from repo** | Replacement-pack `0.40–0.98` series | Minimum range `7.232–7.304 V` | PRIOR TEST EVIDENCE only; individual raw logs absent. |
| **UNKNOWN — not verifiable from repo** | Previous-pack motor tests | Approximate maximum collapse stated in docs | Exact result **UNKNOWN — not verifiable from repo**; raw logs absent. |

### 8.3 Current local gate/test state

| Gate | Current local state | Evidence path |
|---|---|---|
| `camera_orientation` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| `motor_mapping_software` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| `yolo_current_regression` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `PRIOR_TEST_EVIDENCE / UNRECORDED` | Not present |
| `ipm_physical_calibration` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| `ipm_live_dry_run` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| `integrated_control_dry_run` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| `raised_motor_adapter` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| `short_floor_steering` | **PROVISIONAL / UNTESTED**; literal JSON: `passed=false`, `UNVERIFIED / UNRECORDED` | Not present |
| Physical motion authorization | `false`; integrity `LOCKED` | `integration_gates.json` |

A complete chronological current-build test history is **UNKNOWN — not verifiable from repo** because generated evidence/results are absent from this checkout.

## 9. Open items

| Item | Status | Consequence/required resolution |
|---|---|---|
| `manual_motor_control.py` missing | KNOWN BROKEN in this checkout | Physical motor modules cannot be imported; exact PCA9685/INA219 implementation and current hardware assignments cannot be audited; manifest/gate authorization cannot succeed. Recover the exact deployed source and its hash. |
| Left drive motor legacy replacement flag | **PROVISIONAL / UNVERIFIED** | `work/DASHBOARD_SETUP.md` and the superseded `work/robot_dashboard.py` report that the left motor requires replacement. No current repo artifact confirms replacement or present condition. Treat the legacy indication as stale/non-authoritative but unresolved; physically identify, inspect, and resolve it before powered motion. |
| As-built hardware BOM, wiring, and disconnect | **PROVISIONAL / UNVERIFIED** | Actual Jetson/camera variants and camera geometry; PCA9685 production address/frequency; INA219 module, shunt/current calibration, and terminals; L298N revision/jumpers/output allocation; motor specifications/polarity; battery pack/BMS/connectors/charger; fuse; common-ground/rail routing; and the physical disconnect’s actual presence/placement are unresolved. The system cannot be electrically safety-reviewed from this checkout; recover an as-built BOM, schematic, labelled photos, continuity results, and measured rail values. |
| Physical IPM config/evidence absent | **PROVISIONAL / UNVERIFIED** | Current local config is a SITL seed; physical homography/source points/world geometry are **UNKNOWN — not verifiable from repo**. Recover deployed `ipm_config.json` plus calibration image/JSON, or recalibrate physically. |
| Evidence/results not synchronized | **PROVISIONAL / UNVERIFIED** | Exact YOLO, IPM, integrated, motor, steering, and track metrics/history cannot be reproduced or reported as current. |
| All local gates unrecorded; motion authorization locked | **PROVISIONAL / UNVERIFIED** | Each gate is `UNRECORDED`; `physical_motion_authorized=false` and authorization integrity is `LOCKED`, so no physical run is authorized from this checkout. |
| Current-build safety effectiveness | **PROVISIONAL / UNTESTED** | Lease, deadline, voltage, stale-data, gate, and OFF-write rules exist in source, but this checkout retains no current physical/generated gate evidence proving their operational effectiveness. Re-establish the complete gate chain without bypassing thresholds. |
| PID tuning | **PROVISIONAL** | `verification_state=SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED`; `Ki=0.0`; purpose `results` is refused until a tuned config is validated. |
| Wheel-speed feedback | Not implemented | No encoders; trim is open-loop and varies with surface, payload, battery voltage/state, tyres, and mechanics. |
| Metric object range/stopping distance | Not implemented | Bounding-box area and path overlap are descriptive only; conservative whole-frame object STOP remains necessary. |
| Traffic-light semantics | Not implemented | Every traffic-light detection stops; colour/state is not classified. |
| TensorRT | **PROVISIONAL / UNVERIFIED** | No runtime/benchmark evidence; performance optimization is deferred. Do not claim TensorRT FPS. |
| YOLO validation/operation parameter mismatch | **PROVISIONAL / UNVERIFIED** | The canonical benchmark/runbook command uses image `320`, confidence `0.55`, and IoU `0.45`; the recorder binds image `320` but accepts any confidence/IoU in `(0,1]`. Integrated/track operation uses image `256`, confidence `0.45`, and IoU `0.45`. Accepted gate evidence therefore does not directly benchmark the operational input-size/threshold combination. |
| IPM/integrated evidence-image recorder gap | **PROVISIONAL / UNVERIFIED** | The IPM-live and integrated-dry producers require a successful evidence-image write before setting `passed=true`, but their gate recorder trusts `passed=true` and the numeric fields without independently checking that image path/file. Only physical-IPM calibration rechecks image existence. Preserve each producer’s JSON and image together; add recorder-side image validation before treating this as closed. |
| Hardware output-disable watchdog | **PROVISIONAL / UNVERIFIED** | No installed hardware watchdog is evidenced. Software lease/deadline cannot cover OS/process/I²C/hardware failure; physical disconnect remains mandatory. |
| Async YOLO worker shutdown confirmation | **PROVISIONAL / UNTESTED** | `stop()` joins the daemon worker for `5.0 s` but does not raise or record a fault if it remains alive. Current motion finalization still closes the owning process resources, but explicit termination verification is absent. |
| CSI publication to dashboard and autonomy | Deferred | Dashboard and autonomous runner cannot independently own Argus concurrently; a shared publisher is future work. |
| Track planner | Not implemented | There is no square/lap/90-degree-corner planner or finish-line state machine; behavior is local tape-lane centering plus object STOP. |
| Legacy `work/` scripts | **PROVISIONAL / UNGATED** | Not deployed by the current integration script and locally incomplete; do not use as current evidence. |
| Orphan compiled probe | Source missing | `__pycache__/straight_trim_probe.cpython-311.pyc` has no auditable source or invocation. |
| First-party TODO/FIXME | None found | Search found no `TODO`, `FIXME`, `XXX`, or `HACK` in project-owned source. |
| Vendored YOLO TODO comments | Upstream, not project tasks | Present at `yolov5_v6/train.py:394`, `utils/datasets.py:936,967`, `utils/loggers/__init__.py:141`, `utils/loggers/wandb/wandb_utils.py:389`, and `utils/loggers/wandb/log_dataset.py:9`. Preserve with the pinned upstream tree; do not edit casually. |
| Exact OS/package/power-mode context | **UNKNOWN — not verifiable from repo** | Run and retain `jetson_perception_preflight.py` plus every raw environment command listed in Section 2 on the deployed Jetson. |

## 10. Next planned step

The immediate next action is **not another motor run**. First create a synchronized, immutable handover snapshot from the actual Jetson so the tested state can be verified against this source package.

Required preconditions:

1. Physically disconnect the motor battery and keep the disconnect controlled by a spotter.
2. Stop every dashboard, CSI, autonomy, calibration, and motor process.
3. Preserve rather than overwrite the Jetson’s current files.
4. Copy back the exact deployed `manual_motor_control.py`, `motor_config.json`, `controller_config.json`, `ipm_config.json`, `integration_gates.json`, entire `evidence/` directory, entire `results/` directory, generated build manifest, and preflight output.
5. Record SHA-256 for every transferred file and the YOLO source/weights; retain file timestamps and the original directory structure.
6. On the Jetson, run `python jetson_perception_preflight.py`, every raw environment-capture command in Section 2, `python validation_manager.py show`, `python validation_manager.py verify`, and `python make_build_manifest.py`; retain their unedited outputs/artifacts. Do not infer JetPack or package versions from compatibility targets.

Only after synchronization should the next assistant determine the first genuinely unresolved gate. A physical results run additionally requires all eight gates CURRENT, matching artifact/evidence hashes, a `PHYSICALLY_CALIBRATED` IPM, CUDA YOLO, a `PHYSICALLY_TUNED` controller for `--purpose results`, a closed flat barrier-bounded track, no people/traffic/drop-offs, and a spotter holding the independent motor-battery disconnect.
