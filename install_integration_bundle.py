#!/usr/bin/env python3
"""Install seed configs once while preserving calibration/evidence state."""

from __future__ import print_function

import argparse
import datetime
import json
import os
import shutil
import sys


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
PRESERVE_IF_PRESENT = (
    ("ipm_config.seed.json", "ipm_config.json"),
)


def valid_json(path):
    with open(path, "r") as input_file:
        json.load(input_file)


def backup(path, stamp):
    backup_path = "{}.pre_integration_{}.bak".format(path, stamp)
    shutil.copy2(path, backup_path)
    print("BACKUP: {}".format(backup_path))
    return backup_path


def install_seed(seed_path, target_path):
    temporary = target_path + ".tmp"
    shutil.copy2(seed_path, temporary)
    os.replace(temporary, target_path)
    print("INSTALLED SAFE SEED: {}".format(target_path))


def main():
    parser = argparse.ArgumentParser(
        description="Install fail-safe integration configuration seeds."
    )
    parser.add_argument(
        "--preserve-controller-tuning",
        action="store_true",
        help=(
            "Preserve an existing PHYSICALLY_TUNED controller config after "
            "strict validation. Omit this for a safe provisional reset."
        ),
    )
    arguments = parser.parse_args()
    failed = False
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")

    for seed_name, target_name in PRESERVE_IF_PRESENT:
        seed_path = os.path.join(PROJECT_DIRECTORY, seed_name)
        target_path = os.path.join(PROJECT_DIRECTORY, target_name)
        if not os.path.isfile(seed_path):
            print("MISSING seed: {}".format(seed_path))
            failed = True
            continue
        valid_json(seed_path)
        if os.path.exists(target_path):
            valid_json(target_path)
            print("PRESERVED existing: {}".format(target_path))
            continue
        install_seed(seed_path, target_path)

    controller_seed = os.path.join(
        PROJECT_DIRECTORY, "controller_config.seed.json"
    )
    controller_target = os.path.join(
        PROJECT_DIRECTORY, "controller_config.json"
    )
    if not os.path.isfile(controller_seed):
        print("MISSING seed: {}".format(controller_seed))
        failed = True
    else:
        valid_json(controller_seed)
        preserve_controller = False
        if os.path.isfile(controller_target):
            valid_json(controller_target)
            backup(controller_target, stamp)
            if arguments.preserve_controller_tuning:
                try:
                    from control_core import load_controller_config

                    existing = load_controller_config(controller_target)
                    preserve_controller = (
                        existing.get("verification_state")
                        == "PHYSICALLY_TUNED"
                    )
                except Exception as error:
                    print("Controller config is not valid: {}".format(error))
                if not preserve_controller:
                    print(
                        "REFUSED preservation: controller is not a strictly "
                        "valid PHYSICALLY_TUNED config."
                    )
                    failed = True
            else:
                print(
                    "SAFE RESET requested: existing controller config will "
                    "be replaced by the provisional seed."
                )
        if preserve_controller:
            print("PRESERVED physically tuned controller config.")
        else:
            install_seed(controller_seed, controller_target)

    gate_seed = os.path.join(PROJECT_DIRECTORY, "integration_gates.seed.json")
    gate_target = os.path.join(PROJECT_DIRECTORY, "integration_gates.json")
    if not os.path.isfile(gate_seed):
        print("MISSING seed: {}".format(gate_seed))
        failed = True
    else:
        valid_json(gate_seed)
        if os.path.isfile(gate_target):
            valid_json(gate_target)
            backup(gate_target, stamp)
        install_seed(gate_seed, gate_target)

    motor_config = os.path.join(PROJECT_DIRECTORY, "motor_config.json")
    if not os.path.isfile(motor_config):
        print("MISSING: motor_config.json")
        print("Run install_motor_calibration.py before integration.")
        failed = True
    else:
        valid_json(motor_config)
        print("PRESERVED drivetrain calibration: {}".format(motor_config))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
