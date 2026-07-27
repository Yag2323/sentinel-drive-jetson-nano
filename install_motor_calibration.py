#!/usr/bin/env python3
"""Install measured drivetrain calibration while preserving motor directions."""

import argparse
import json
import os
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path


CONFIG_PATH = Path(__file__).resolve().parent / "motor_config.json"

CALIBRATION = {
    "left_trim": 1.00,
    "right_trim": 0.98,
    "deadband_duty": 0.60,
    "cruise_duty": 0.75,
    "max_duty": 0.98,
    # Side-specific breakpoints keep both motors at or above the measured
    # 60% floor while preserving the physically calibrated 75% / 73.5%
    # straight-line cruise pair.
    "left_deadband_duty": 0.60,
    "right_deadband_duty": 0.60,
    "left_cruise_duty": 0.75,
    "right_cruise_duty": 0.735,
    "left_max_duty": 0.98,
    "right_max_duty": 0.9604,
    "calibrated_at_voltage": 7.688,
    "calibration_date": "2026-07-24",
}


def load_existing():
    if not CONFIG_PATH.exists():
        return {}

    try:
        with CONFIG_PATH.open("r", encoding="utf-8") as config_file:
            data = json.load(config_file)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            "Cannot safely read existing {}: {}".format(CONFIG_PATH, error)
        )

    if not isinstance(data, dict):
        raise RuntimeError("Existing motor_config.json is not a JSON object.")
    return data


def direction_value(existing, key):
    value = existing.get(key, False)
    if not isinstance(value, bool):
        raise RuntimeError(
            "Existing {!r} must be true or false; found {!r}.".format(
                key, value
            )
        )
    return value


def make_backup():
    if not CONFIG_PATH.exists():
        return None

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
    backup = CONFIG_PATH.with_name(
        "motor_config.before_calibration_{}.json".format(stamp)
    )
    shutil.copy2(str(CONFIG_PATH), str(backup))
    return backup


def atomic_write(configuration):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".motor_config.", suffix=".tmp", dir=str(CONFIG_PATH.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as config_file:
            json.dump(configuration, config_file, indent=2)
            config_file.write("\n")
            config_file.flush()
            os.fsync(config_file.fileno())
        os.replace(temporary_name, str(CONFIG_PATH))
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Install the measured motor calibration while preserving the "
            "existing direction booleans."
        )
    )
    parser.parse_args(argv)
    try:
        existing = load_existing()
        reverse_left = direction_value(existing, "reverse_left")
        reverse_right = direction_value(existing, "reverse_right")

        backup = make_backup()

        configuration = {
            "reverse_left": reverse_left,
            "reverse_right": reverse_right,
        }
        configuration.update(CALIBRATION)
        atomic_write(configuration)

        print("Motor calibration installed: {}".format(CONFIG_PATH))
        if backup is not None:
            print("Previous configuration backed up: {}".format(backup))
        else:
            print("No previous configuration existed; direction defaults are false.")
        print(json.dumps(configuration, indent=2))
        return 0

    except Exception as error:
        print("INSTALLATION REFUSED: {}".format(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
