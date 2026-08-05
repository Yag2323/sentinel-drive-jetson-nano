# Sentinel Drive engineering evidence register

This register distinguishes measured physical results, motor-disabled physical
CSI tests, software regression tests, exploratory observations and unresolved
claims. A retained result remains useful engineering evidence, but it is only a
`CURRENT` motion gate when `validation_manager.py` confirms that its recorded
source hashes still match the deployed build.

## Evidence classification

| Label | Meaning |
|---|---|
| `PHYSICALLY VERIFIED` | Direct physical hardware behaviour was observed under the stated conditions. |
| `SOFTWARE VERIFIED ON PHYSICAL CSI` | Real camera input exercised software logic, but motors were disabled. |
| `SOFTWARE VERIFIED` | Hardware-free regression or static verification. |
| `PRIOR TEST EVIDENCE` | Valid retained result from an earlier artifact identity; not a current motion gate. |
| `EXPLORATORY` | Useful observation without the complete hash-bound evidence contract. |
| `UNVERIFIED` | No acceptable evidence supports the claim. |

## Retained measured results

### Physical CSI perception and integrated dry run

| Test and date | Conditions | Exact measured result | Classification |
|---|---|---|---|
| YOLOv5s real-CSI baseline, 2026-07-26 | CUDA FP16; 30.0 s; 213 frames; pinned YOLOv5 v6 commit | complete-window FPS `7.078025127419382`; inference-only FPS `8.80755624811622`; mean inference `113.53887183109187 ms`; `passed=true` | `PRIOR TEST EVIDENCE` |
| YOLOv5n real-CSI benchmark, 2026-07-26 | CUDA FP16; 30.0 s; 249 frames; 320 image size | complete-window FPS `8.283126567668365`; inference-only FPS `10.650660666074126`; mean inference `93.89088915256971 ms`; p95 source-to-result `194.0077229992312 ms`; `passed=true` | `PRIOR TEST EVIDENCE` |
| IPM physical-CSI dry run used in the final report, 2026-07-26 | Motors not imported; centred reference; 30.0 s; 343 frames | average loop `11.803985748417533 FPS`; full/usable lane `99.12536443148689%`; mean confidence `0.8484607454093648`; mean absolute offset `0.013785545043762011`; `passed=true` | `PRIOR TEST EVIDENCE` |
| Integrated perception/control dry run used in the final report, 2026-07-26 | Physical CSI; motors disabled; 60.0 s; 497 frames | average control `8.737179653391355 FPS`; YOLO fresh rate `95.57344064386318%`; full lane `96.17706237424547%`; usable lane `99.19517102615694%`; 404 DRIVE frames; 45 object-stop frames; 23 lane-stop frames; zero mapped-duty violations; `passed=true` | `PRIOR TEST EVIDENCE` |

The report-aligned integrated dry-run safety reasons include 45
`OBJECT_DETECTED_PERSON` stop frames. This is direct evidence that the
object-detection result reached the supervisor and produced STOP without
energising the motors.

| YOLOv5n evidence | IPM evidence | Integrated control evidence |
|---|---|---|
| ![YOLOv5n physical CSI evidence](images/evidence-yolov5n-csi.jpg) | ![IPM physical CSI evidence](images/evidence-ipm-dry-run.jpg) | ![Integrated dry-run evidence](images/evidence-integrated-dry-run.jpg) |

### Stop-sign observation

![Live motor-disabled YOLO object-detection interface](images/live-yolov5-object-detection-interface.png)

The image above is retained as interface evidence only. Its visible detector
label is `person 0.63`; it does not establish a successful stop-sign
classification. This distinction prevents a test prop from being promoted into
an accuracy claim.

The archived motor-disabled exploratory CSV
`results/combined_perception_20260720_204831.csv` contains three consecutive
rows, frames 70 to 72, with decision `STOP - STOP SIGN`. Each row reports
`stop sign:0.53:0.056:path` and a simultaneous person detection. No matching
hash-bound summary JSON was retained for that exploratory run, so this is
labelled `EXPLORATORY`, not a current validation gate and not a custom-model
accuracy result.

The implemented current policy is independently visible in
`yolov5_runtime.py`: a fresh COCO `stop sign` detection returns
`STOP_SIGN_DETECTED`. `stop_sign_vnc_demo.py` provides a motor-free VNC view and
automatically saves the exact annotated frame plus a JSON sidecar when that
class is genuinely detected.

### Drivetrain calibration

| Parameter | Measured or persisted value | Status |
|---|---:|---|
| Calibration voltage | `7.688 V` | Physical calibration record, 2026-07-24 |
| Left/right deadband duty | `0.600 / 0.600` | Physical calibration record |
| Left/right cruise duty | `0.750 / 0.735` | Physical calibration record |
| Left/right maximum duty | `0.980 / 0.9604` | Persisted piecewise mapping |
| Right trim | `0.98` | Physical calibration record |
| Straight-line lateral error | less than `5 cm` over `0.5 m` | Operator measurement; method uncertainty not independently recorded |

These values are operating-point dependent. Battery state, load, camera pose,
track surface, wheel condition or hardware changes require revalidation.

## Artifact identity

The raw JSON and CSV files are not published because they contain deployment
paths and machine metadata. Their identities are retained here so the original
files can be checked without exposing local information.

| Artifact | SHA-256 |
|---|---|
| `yolov5s_csi_20260726T121156Z.json` | `b3107dcfb73f4a3d13b70ce278b24fd30b881e849bb3de743ea23390f69dc5dd` |
| `yolov5s_csi_20260726T121156Z.csv` | `8c8abc8a4ec180d96121cb473bbf1f84b3d728a0aea88c08b93b3206f0b8ed4a` |
| `yolov5n_csi_20260726T121601Z.json` | `438468ad1367fb8088f56ce47ac75536c45957ea10259d273d4708cc10b11b7d` |
| `yolov5n_csi_20260726T121601Z.csv` | `08f0baabddafbe2ccbf18e5b8e6bd34a289fd1f74f0c0aa9c546d41ec0c78e45` |
| `ipm_dry_run_20260726T162241Z.json` | `697a6d8998382859d71fc2c061acfa9d22e3e9aebe5554c47b7a30ed9aee8b7c` |
| `ipm_dry_run_20260726T162241Z.csv` | `a96e3e1a05a66f60fcedaadfbc7806e0254e5ba6796dec76464a6e09ceaced42` |
| `integrated_dry_20260726T162715Z.json` | `73f12e79c2e0be28c23d092144b5ca91a99533af50e8ac5a8e133661dd7afa25` |
| `integrated_dry_20260726T162715Z.csv` | `e0e82501649212210d34eed6632fefe1cbaef971571e33d931294252a7593d65` |

## Current public-build status

| Claim | Status | Basis |
|---|---|---|
| Single-line detector implementation | `SOFTWARE VERIFIED` after the included synthetic regression passes | `single_line_lane.py`, `test_single_line_synthetic.py` |
| Generic object and stop-sign safety policy | Implemented; current physical revalidation required | `yolov5_runtime.py`, `control_core.py` |
| Object-avoidance steering | `UNVERIFIED / NOT IMPLEMENTED` | The supervisor stops; it does not plan around obstacles |
| Current single-line circle controller tuning | `PROVISIONAL` | Controller gains are marked not physically tuned in `controller_config.json` |
| Guarded straight-line commissioning runner | `PROVISIONAL / PHYSICALLY UNTESTED` | Source review and software self-test only |
| Continuous autonomous circle or oval lap | `UNVERIFIED` | No retained result proves a full lap on the current build |
| Physical motion authorization in a fresh clone | `false / LOCKED` | Fail-closed `integration_gates.json` seed |

## Interpretation rules

- A detected object causes conservative stopping. It is not evidence of path
  planning or obstacle avoidance.
- The generic COCO stop-sign class is not a custom traffic-sign model and does
  not establish sign-recognition accuracy.
- Bounding-box area and IPM overlap are image-space features, not calibrated
  distance in metres.
- INA219 bus voltage is not battery state of charge.
- A software dry run verifies software logic, not physical steering.
- A timed command is not a distance measurement unless distance was measured
  independently.
- A hash proves file identity, not unchanged camera pose, wiring, battery,
  drivetrain or track geometry.
- Any change to source, configuration, weights, camera pose or physical setup
  invalidates the affected gate and every downstream result.
