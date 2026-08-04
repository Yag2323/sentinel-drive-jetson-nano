# Sentinel Drive viva walkthrough

This route presents the repository in about seven minutes without overstating
the evidence.

## 1. Start with the engineering claim

Open the repository `README.md` and say:

> Sentinel Drive is a Jetson Nano differential-drive prototype that combines
> physical CSI line perception with asynchronous YOLOv5 object detection. It
> follows a black line when every safety input is current and commands STOP on
> a missing line, object, stop sign, stale result, voltage fault or command
> timeout.

Immediately state the boundary:

> The subsystem benchmarks and motor-disabled integration run are measured.
> A continuous autonomous lap on the latest single-line build is not yet
> verified, so the repository does not claim one.

That distinction demonstrates evidence discipline rather than weakness.

## 2. Explain the data flow

Open `docs/system_architecture.md`, then point through this sequence:

1. `gst_camera_bridge.py` owns the CSI/Argus camera and applies
   `flip-method=2`.
2. `single_line_lane.py` finds one plausible black component and estimates its
   lookahead position, heading and curvature.
3. `yolov5_runtime.py` processes the latest submitted frame asynchronously.
4. `control_core.py` combines line state, object state, timestamps and voltage.
5. `motor_mapping.py` maps normalized output onto measured motor operating
   points.
6. `safe_motor_output.py` enforces the voltage guard, command lease and OFF
   behaviour.
7. `manual_motor_control.py` is the low-level PCA9685/INA219 adapter.

Useful design sentence:

> Lane perception must be relatively fast for steering, while YOLO is slower,
> so it is isolated in a latest-result worker. The safety supervisor still
> rejects stale or frame-lagged YOLO output.

## 3. Demonstrate line following logic

Open `single_line_lane.py` and search for `observe_single_line`. Explain:

- thresholded black components are generated inside the configured region;
- candidates must satisfy area, support, vertical coverage, tape-width and fit
  constraints;
- ambiguous candidates fail closed;
- a polynomial supplies lookahead offset, near-field offset, heading and
  curvature; and
- only `FULL` with enough confidence can reach the controller.

Then open `test_single_line_synthetic.py`. The useful cases are:

- centred and shifted straight lines;
- left and right curves;
- close perspective tape;
- blank, fragmented and oversized inputs; and
- multiple-line ambiguity and wrong-target takeover.

Software-only demonstration command:

```bash
python3 test_single_line_synthetic.py
```

## 4. Show object and stop-sign behaviour

Open `yolov5_runtime.py` and search for `evaluate_obstacles`.

Explain the policy precisely:

- `stop sign` returns `STOP_SIGN_DETECTED`;
- `traffic light` returns `TRAFFIC_LIGHT_UNCLASSIFIED` because light colour is
  not classified;
- configured people, vehicle and scene-object classes return
  `OBJECT_DETECTED_<CLASS>`; and
- missing, failed or stale YOLO output is handled as unsafe by the supervisor.

Use this sentence if asked about obstacle avoidance:

> The implemented scope is object-aware emergency stopping. It does not steer
> around an obstacle or re-plan a route, so I call it obstacle response rather
> than obstacle-avoidance navigation.

Show `docs/images/evidence-yolov5n-csi.jpg`, then open the decision mapping in
`yolov5_runtime.py`. State that the stop sign is the generic pretrained COCO
class, not a custom-trained model. Only show a physical stop-sign recognition
image after its matching Jetson result has been audited.

## 5. Show the safety boundary

Open `control_core.py`, `safe_motor_output.py` and `SAFETY.md`.

Highlight:

- camera stale timeout: `0.25 s`;
- YOLO stale timeout: `0.75 s`;
- maximum YOLO frame lag: `8`;
- line confidence minimum: `0.55`;
- command lease: `0.35 s`;
- motor-bus stop voltage: `6.60 V`;
- maximum accepted motor-bus voltage: `8.60 V`; and
- every error path calls STOP/OFF.

Then make the limitation explicit:

> The command lease is best-effort software defence. It cannot replace the
> spotter-controlled physical motor-battery disconnect if the OS, process,
> I2C bus or motor controller becomes unresponsive.

## 6. Present measured evidence

Open `docs/project_evidence_register.md` and the three evidence images.

Headline values:

| Evidence | Viva value |
|---|---:|
| YOLOv5n complete physical-CSI throughput | `8.28 FPS` |
| YOLOv5n inference-only equivalent | `10.65 FPS` |
| IPM physical-CSI dry-run throughput | `13.25 FPS` |
| IPM full-lane rate | `99.74%` |
| Integrated motor-disabled control rate | `8.75 FPS` |
| Fresh YOLO results in the integrated run | `97.75%` |
| Object-stop frames in the integrated run | `41` |
| Lane-stop frames in the integrated run | `64` |

Explain that exact unrounded values and artifact SHA-256 identifiers are in the
register, while README values are rounded for readability.

## 7. Finish with engineering judgement

Open `integration_gates.json` and `validation_manager.py`.

Say:

> The public seed is deliberately locked. A physical result is only current
> when the expected evidence type passed and its test-time source hashes still
> match the deployed build. Software tests cannot silently become physical
> evidence.

Finish with the next step:

> The immediate remaining step is final physical controller tuning and a
> retained continuous-lap result on the exact single-line track, camera pose,
> battery and software identity. Until that succeeds, the lap remains
> unverified.

## Likely examiner questions

### Why use YOLO asynchronously?

Raw YOLO throughput is lower than the lane-only loop. Latest-result isolation
prevents steering from waiting for each inference, while freshness and frame
lag checks prevent the vehicle from relying on an old result.

### Why does any relevant object stop the vehicle?

The prototype has no calibrated depth sensor or metric stopping-distance
model. A conservative global STOP is more defensible than treating bounding
box area as metres.

### Why is integral gain zero?

The implemented controller is time-aware and supports PID, but current
`Ki=0`. Initial physical operation therefore uses PD behaviour to avoid
integral wind-up while the gains remain provisional.

### Why is there a motor deadband mapping?

Below measured duty `0.60`, the drivetrain did not reliably move under the
calibration conditions. `motor_mapping.py` maps useful controller output onto
the measured operating band while preserving a true zero-command stop region.

### What would you improve next?

1. complete and retain the current-build continuous-lap test;
2. tune gains across battery voltage and speed conditions;
3. add a hardware OE/watchdog path independent of Python and I2C;
4. add calibrated ranging before considering obstacle path planning; and
5. train and evaluate a domain-specific sign model only if sign recognition is
   promoted from a safety demonstration to a formal requirement.
