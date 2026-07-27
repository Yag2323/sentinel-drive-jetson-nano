#!/usr/bin/env python3
"""Map normalized controller commands into the calibrated PWM range."""

import json
import math
from pathlib import Path


CONFIG_PATH = Path(__file__).resolve().parent / "motor_config.json"
STOP_EPSILON = 0.02


def load_motor_config(path=CONFIG_PATH):
    path = Path(path)
    with path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)

    required = (
        "reverse_left",
        "reverse_right",
        "left_trim",
        "right_trim",
        "deadband_duty",
        "cruise_duty",
        "max_duty",
    )
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError(
            "motor_config.json is missing: {}".format(", ".join(missing))
        )

    for key in ("reverse_left", "reverse_right"):
        if not isinstance(config[key], bool):
            raise ValueError("{} must be JSON true or false.".format(key))

    numeric_keys = (
        "left_trim",
        "right_trim",
        "deadband_duty",
        "cruise_duty",
        "max_duty",
    )
    for key in numeric_keys:
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{} must be a JSON number.".format(key))
        if not math.isfinite(float(value)):
            raise ValueError("{} must be finite.".format(key))

    deadband = float(config["deadband_duty"])
    cruise = float(config["cruise_duty"])
    maximum = float(config["max_duty"])
    if not 0.0 <= deadband < cruise <= maximum <= 1.0:
        raise ValueError(
            "Expected 0 <= deadband_duty < cruise_duty <= max_duty <= 1."
        )

    for key in ("left_trim", "right_trim"):
        trim = float(config[key])
        if not 0.0 < trim <= 1.0:
            raise ValueError("{} must be greater than 0 and at most 1.".format(key))

    optional_breakpoints = (
        "left_deadband_duty",
        "right_deadband_duty",
        "left_cruise_duty",
        "right_cruise_duty",
        "left_max_duty",
        "right_max_duty",
    )
    for key in optional_breakpoints:
        if key not in config:
            continue
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{} must be a JSON number.".format(key))
        if not math.isfinite(float(value)):
            raise ValueError("{} must be finite.".format(key))

    resolved = dict(config)
    resolved["left_deadband_duty"] = float(
        config.get("left_deadband_duty", deadband)
    )
    resolved["right_deadband_duty"] = float(
        config.get("right_deadband_duty", deadband)
    )
    resolved["left_cruise_duty"] = float(
        config.get("left_cruise_duty", cruise * float(config["left_trim"]))
    )
    resolved["right_cruise_duty"] = float(
        config.get("right_cruise_duty", cruise * float(config["right_trim"]))
    )
    resolved["left_max_duty"] = float(
        config.get("left_max_duty", maximum * float(config["left_trim"]))
    )
    resolved["right_max_duty"] = float(
        config.get("right_max_duty", maximum * float(config["right_trim"]))
    )
    for side in ("left", "right"):
        floor = resolved[side + "_deadband_duty"]
        side_cruise = resolved[side + "_cruise_duty"]
        side_maximum = resolved[side + "_max_duty"]
        if not 0.0 <= floor < side_cruise <= side_maximum <= 1.0:
            raise ValueError(
                "Expected 0 <= {0}_deadband_duty < {0}_cruise_duty "
                "<= {0}_max_duty <= 1.".format(side)
            )
    return resolved


def map_duty(output, is_right=False, config=None):
    """Return a non-negative PWM magnitude for a normalized PID output."""
    if config is None:
        config = load_motor_config()

    raw_output = float(output)
    if not math.isfinite(raw_output):
        raise ValueError("Controller output must be finite.")

    magnitude = min(abs(raw_output), 1.0)
    if magnitude < STOP_EPSILON:
        return 0.0

    side = "right" if is_right else "left"
    deadband = float(config[side + "_deadband_duty"])
    cruise = float(config[side + "_cruise_duty"])
    maximum = float(config[side + "_max_duty"])
    logical_deadband = float(config["deadband_duty"])
    logical_cruise = float(config["cruise_duty"])
    logical_maximum = float(config["max_duty"])
    cruise_command = (
        (logical_cruise - logical_deadband)
        / (logical_maximum - logical_deadband)
    )

    if magnitude <= cruise_command:
        fraction = magnitude / cruise_command
        duty = deadband + fraction * (cruise - deadband)
    else:
        fraction = (magnitude - cruise_command) / (1.0 - cruise_command)
        duty = cruise + fraction * (maximum - cruise)
    return min(max(duty, deadband), maximum)


def map_signed_duty(output, is_right=False, config=None):
    """Return mapped duty with the original command direction retained."""
    raw_output = float(output)
    duty = map_duty(raw_output, is_right=is_right, config=config)
    if duty == 0.0:
        return 0.0
    return math.copysign(duty, raw_output)
