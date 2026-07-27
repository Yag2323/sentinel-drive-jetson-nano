# Drivetrain calibration record

## Recorded physical observation

- Calibration date: 2026-07-24
- Calibration motor-bus voltage: 7.688 V
- Test condition: wheels on the stated floor surface, open-loop PWM
- Original evidence type: physical floor calibration observation
- Current-build classification: PRIOR TEST EVIDENCE until the current hashed
  integrated build passes the raised adapter and `short_floor_steering` gates

| Observed parameter | Value |
|---|---:|
| Left physical floor breakpoint | 0.600 |
| Right physical floor breakpoint | 0.600 |
| Selected left cruise PWM | 0.750 |
| Selected right cruise PWM | 0.735 |
| Effective cruise right/left ratio | 0.980 |
| Straight-line error in calibration observation | Less than 5 cm over 0.5 m |
| Largest start-to-minimum loaded drop in the stated replacement-pack series | Approximately 0.448 V |
| Spread between recorded minima in that series | 72 mV (7.232-7.304 V) |
| Maximum collapse observed with the previous pack | Approximately 2.49 V; PRIOR TEST EVIDENCE |

The 72 mV value is not battery voltage sag. It is the range between minimum
voltages from separate trials. Individual start-to-minimum differences are the
loaded voltage drops; the largest stated value in that series was about
0.448 V. Retain the raw logs and initial voltage for every value used in the
report.

## Implemented side-specific mapping

The current configuration uses these explicit breakpoints:

| Side | Floor breakpoint | Cruise breakpoint | Maximum breakpoint |
|---|---:|---:|---:|
| Left | 0.600 | 0.750 | 0.980 |
| Right | 0.600 | 0.735 | 0.9604 |

The left and right floor breakpoints are both 0.600 because both are physical
breakaway measurements. The right-side correction is introduced at cruise and
maximum. It is deliberately not applied as a blanket 0.98 multiplier to every
right-side duty: that would produce a right floor of 0.588 and could put the
wheel back into its measured stall region.

For a non-zero normalized logical magnitude, `motor_mapping.py` uses two linear
segments per side. The logical transition corresponding to calibrated cruise
is:

`(0.750 - 0.600) / (0.980 - 0.600) = 0.3947368421`.

- From just above zero through that logical transition, each side is mapped
  from its 0.600 floor to its side-specific cruise value.
- From the transition through logical 1.0, each side is mapped from cruise to
  its side-specific maximum.
- Commands with magnitude below `STOP_EPSILON = 0.02` map to zero.
- Direction is retained separately from the non-negative PWM magnitude.

The 0.980/0.9604 maximum pair is the configured result of the side correction;
do not call it a physically verified straight-line maximum unless a retained
physical test under the same build and conditions demonstrates that claim.

## Calibration validity and required revalidation

The calibration was made at one motor-bus voltage, surface, payload, tyre
condition, and mechanical state. Static friction and available motor torque
change with battery voltage and state of charge; a floor breakpoint may move as
the pack discharges. The 0.600 values are therefore measured operating points,
not universal motor constants.

The drivetrain has no wheel encoders, so the side mapping is feed-forward
open-loop compensation. It cannot measure or correct actual wheel-speed
difference directly. Camera lane feedback may correct vehicle path at a higher
level, but it does not convert the motor calibration into closed-loop wheel
speed control.

Before the mapping is used in an autonomous track run, the exact current
artifacts and evidence hashes must pass, in order:

1. the hardware-free motor-mapping self-test;
2. the raised-wheel motor-adapter and command-lease validation; and
3. `steering_floor_validation.py`, recorded as gate
   `short_floor_steering`, with brief guarded left and right correction pulses
   and a confirmed controlled stop.

Any subsequent edit to mapped-control code or motor configuration invalidates
the bound gate evidence. Re-run the affected validation rather than relabelling
the 2026-07-24 observation as a current-build pass.

The software lease remains best effort: it depends on a running process,
schedulable watchdog thread, and available I2C path. A spotter-controlled
physical motor-battery disconnect is mandatory for every powered test.
