#!/usr/bin/env python3
"""Install the fail-safe outer-circle single-line detector profile.

The migration preserves the physical IPM geometry, invalidates calibration,
controller tuning and integration-gate evidence, and writes every changed JSON
file with ``os.replace`` from a temporary file in the same directory.  Gate
state is replaced first, so interruption can only leave the system in a more
conservative state.
"""

from __future__ import print_function

import argparse
import copy
import datetime
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile

from single_line_lane import (
    SINGLE_LINE_DEFAULTS,
    SINGLE_LINE_DETECTOR_MODE,
    validate_single_line_config,
)


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
IPM_CONFIG_PATH = os.path.join(PROJECT_DIRECTORY, "ipm_config.json")
CONTROLLER_CONFIG_PATH = os.path.join(
    PROJECT_DIRECTORY, "controller_config.json"
)
GATES_PATH = os.path.join(PROJECT_DIRECTORY, "integration_gates.json")
GATES_SEED_PATH = os.path.join(
    PROJECT_DIRECTORY, "integration_gates.seed.json"
)

PROFILE_VERSION = "SINGLE_LINE_POLYNOMIAL_V1"
TARGET_LINE_ROLE = "OUTER_CIRCLE_CENTERLINE"
REVALIDATION_STATE = "REVALIDATION_REQUIRED"
PROVISIONAL_CONTROLLER_STATE = (
    "SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED"
)
ALLOWED_SOURCE_DETECTOR_MODES = (
    "PAIRED_ROW_POLYNOMIAL_V2",
    PROFILE_VERSION,
)

CONTROLLER_RESET_NOTE = (
    "Prior physical controller tuning was invalidated when the perception "
    "profile changed to the outer-circle single-line detector. Retune only "
    "after physical IPM and dry-run validation pass on the current build."
)


def utc_stamp():
    return datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")


def utc_iso():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def read_json(path):
    try:
        with open(path, "r") as input_file:
            value = json.load(input_file)
    except (IOError, OSError, ValueError) as error:
        raise RuntimeError(
            "Cannot read valid JSON from {}: {}".format(path, error)
        )
    if not isinstance(value, dict):
        raise RuntimeError("{} must contain a JSON object.".format(path))
    return value


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def canonical(value):
    return json.dumps(value, separators=(",", ":"), sort_keys=True)


def _finite_number(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def validate_geometry(points, name):
    if not isinstance(points, list) or len(points) != 4:
        raise RuntimeError("{} must contain exactly four points.".format(name))
    normalised = []
    for index, point in enumerate(points):
        if not isinstance(point, list) or len(point) != 2:
            raise RuntimeError(
                "{} point {} must be a two-value JSON array.".format(
                    name, index
                )
            )
        if not all(_finite_number(value) for value in point):
            raise RuntimeError(
                "{} point {} must contain finite numbers.".format(name, index)
            )
        x_value = float(point[0])
        y_value = float(point[1])
        if not 0.0 <= x_value <= 1.0 or not 0.0 <= y_value <= 1.0:
            raise RuntimeError(
                "{} point {} lies outside the normalised image.".format(
                    name, index
                )
            )
        normalised.append((x_value, y_value))

    if len(set(normalised)) != 4:
        raise RuntimeError("{} contains repeated points.".format(name))

    twice_area = 0.0
    for index, point in enumerate(normalised):
        following = normalised[(index + 1) % len(normalised)]
        twice_area += point[0] * following[1] - following[0] * point[1]
    if abs(twice_area) * 0.5 < 0.0001:
        raise RuntimeError("{} is degenerate or has near-zero area.".format(name))
    return copy.deepcopy(points)


def build_ipm_candidate(existing, installed_utc):
    mode = existing.get("detector_mode")
    if mode not in ALLOWED_SOURCE_DETECTOR_MODES:
        raise RuntimeError(
            "Unsafe or unknown detector_mode {!r}; expected one of {}.".format(
                mode, ", ".join(ALLOWED_SOURCE_DETECTOR_MODES)
            )
        )
    if SINGLE_LINE_DETECTOR_MODE != PROFILE_VERSION:
        raise RuntimeError(
            "Installed single-line implementation version {!r} does not "
            "match installer profile {!r}.".format(
                SINGLE_LINE_DETECTOR_MODE, PROFILE_VERSION
            )
        )
    if SINGLE_LINE_DEFAULTS.get("detector_mode") != PROFILE_VERSION:
        raise RuntimeError("Single-line defaults identify an unsafe version.")

    source_points = validate_geometry(
        existing.get("source_points_normalized"),
        "source_points_normalized",
    )
    destination_points = validate_geometry(
        existing.get("destination_points_normalized"),
        "destination_points_normalized",
    )
    for dimension in ("frame_width", "frame_height"):
        value = existing.get(dimension)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise RuntimeError("{} must be a positive integer.".format(dimension))

    candidate = copy.deepcopy(existing)
    for key, value in SINGLE_LINE_DEFAULTS.items():
        candidate[key] = copy.deepcopy(value)
    candidate["detector_mode"] = PROFILE_VERSION
    candidate["target_line_role"] = TARGET_LINE_ROLE
    candidate["calibration_state"] = REVALIDATION_STATE
    candidate["calibration_timestamp_utc"] = None
    candidate["single_line_profile_installed_utc"] = installed_utc
    candidate["profile_migration_reason"] = (
        "Detector semantics changed from paired rails to the outer-circle "
        "single centreline; physical calibration and all dependent evidence "
        "must be rebuilt."
    )
    for key in list(candidate):
        if key.startswith("calibration_evidence"):
            candidate.pop(key, None)

    # Assign the preserved copies explicitly and verify byte-equivalent JSON.
    candidate["source_points_normalized"] = source_points
    candidate["destination_points_normalized"] = destination_points
    if canonical(candidate["source_points_normalized"]) != canonical(
        existing["source_points_normalized"]
    ):
        raise RuntimeError("Migration changed physical source points.")
    if canonical(candidate["destination_points_normalized"]) != canonical(
        existing["destination_points_normalized"]
    ):
        raise RuntimeError("Migration changed physical destination points.")

    validated = validate_single_line_config(candidate)
    if validated.get("detector_mode") != PROFILE_VERSION:
        raise RuntimeError("Single-line candidate did not validate.")
    if validated.get("target_line_role") != TARGET_LINE_ROLE:
        raise RuntimeError("Single-line target role was not preserved.")
    if validated.get("calibration_state") == "PHYSICALLY_CALIBRATED":
        raise RuntimeError("Migration failed to invalidate calibration.")
    return candidate


def build_controller_candidate(existing, installed_utc):
    candidate = copy.deepcopy(existing)
    candidate["verification_state"] = PROVISIONAL_CONTROLLER_STATE
    candidate["profile_migration_reason"] = CONTROLLER_RESET_NOTE
    candidate["profile_migration_utc"] = installed_utc
    notes = candidate.get("notes", [])
    if not isinstance(notes, list) or not all(
        isinstance(note, str) for note in notes
    ):
        raise RuntimeError("controller_config.json notes must be a string list.")
    notes = list(notes)
    if CONTROLLER_RESET_NOTE not in notes:
        notes.append(CONTROLLER_RESET_NOTE)
    candidate["notes"] = notes
    return candidate


def validate_gate_seed(seed, existing):
    seed_gates = seed.get("gates")
    existing_gates = existing.get("gates")
    if not isinstance(seed_gates, dict) or not seed_gates:
        raise RuntimeError("integration_gates.seed.json has no gate map.")
    if not isinstance(existing_gates, dict) or not existing_gates:
        raise RuntimeError("integration_gates.json has no gate map.")
    seed_names = set(seed_gates)
    existing_names = set(existing_gates)
    missing_from_seed = existing_names - seed_names
    introduced_by_seed = seed_names - existing_names
    if missing_from_seed:
        raise RuntimeError(
            "Gate seed omits live gates {}; refusing an incomplete reset."
            .format(", ".join(sorted(missing_from_seed)))
        )
    if introduced_by_seed - set(("outer_circle_positions",)):
        raise RuntimeError(
            "Gate seed introduces unsupported gates {}; refusing the reset."
            .format(", ".join(sorted(introduced_by_seed)))
        )
    if seed.get("physical_motion_authorized") is not False:
        raise RuntimeError("Gate seed does not lock physical motion.")
    for name, gate in seed_gates.items():
        if not isinstance(gate, dict):
            raise RuntimeError("Gate seed {} is malformed.".format(name))
        if gate.get("passed") is not False:
            raise RuntimeError("Gate seed {} is already passed.".format(name))
        if gate.get("integrity_status") not in (
            "UNRECORDED",
            "STALE",
            "LOCKED",
        ):
            raise RuntimeError(
                "Gate seed {} has unsafe integrity state {!r}.".format(
                    name, gate.get("integrity_status")
                )
            )
    return copy.deepcopy(seed)


def stage_json(path, value):
    directory = os.path.dirname(path)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".{}-single-line-".format(os.path.basename(path)),
        suffix=".tmp",
        dir=directory,
    )
    try:
        with os.fdopen(descriptor, "w") as output_file:
            json.dump(value, output_file, indent=2, sort_keys=True)
            output_file.write("\n")
            output_file.flush()
            os.fsync(output_file.fileno())
        if os.path.isfile(path):
            shutil.copymode(path, temporary_path)
        read_json(temporary_path)
        return temporary_path
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise


def verified_backup(path, stamp):
    backup_path = "{}.before-single-line-{}.bak".format(path, stamp)
    shutil.copy2(path, backup_path)
    if sha256_file(path) != sha256_file(backup_path):
        try:
            os.unlink(backup_path)
        except OSError:
            pass
        raise RuntimeError("Backup verification failed for {}.".format(path))
    return backup_path


def atomic_restore(target_path, backup_path):
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".{}-rollback-".format(os.path.basename(target_path)),
        suffix=".tmp",
        dir=os.path.dirname(target_path),
    )
    os.close(descriptor)
    try:
        shutil.copy2(backup_path, temporary_path)
        if sha256_file(backup_path) != sha256_file(temporary_path):
            raise RuntimeError("Rollback copy verification failed.")
        os.replace(temporary_path, target_path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass


def print_next_steps():
    print("\nNEXT COMMANDS (motor battery physically disconnected):")
    print("  python calibrate_ipm.py")
    print(
        "  python ipm_alignment_diagnostic.py --warmup-seconds 5 "
        "--seconds 20"
    )
    print(
        "  python outer_circle_position_validation.py "
        "--seconds-per-position 5"
    )
    print("  python validation_manager.py show")
    print("Do not run track_run.py until every rebuilt gate is PASS / CURRENT.")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Atomically install the fail-safe outer-circle single-line "
            "perception profile and invalidate stale physical evidence."
        )
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print the migration plan without writing files.",
    )
    arguments = parser.parse_args()

    try:
        if not os.path.isfile(IPM_CONFIG_PATH):
            raise RuntimeError("Missing required {}.".format(IPM_CONFIG_PATH))
        installed_utc = utc_iso()
        existing_ipm = read_json(IPM_CONFIG_PATH)
        ipm_candidate = build_ipm_candidate(existing_ipm, installed_utc)

        operations = []
        # Safety-first order: revoke gates, revoke tuning, then change detector.
        if os.path.isfile(GATES_PATH):
            if not os.path.isfile(GATES_SEED_PATH):
                raise RuntimeError(
                    "Live integration gates exist but the safe gate seed is "
                    "missing."
                )
            existing_gates = read_json(GATES_PATH)
            gate_seed = read_json(GATES_SEED_PATH)
            gate_candidate = validate_gate_seed(gate_seed, existing_gates)
            operations.append(("integration gates", GATES_PATH, gate_candidate))

        if os.path.isfile(CONTROLLER_CONFIG_PATH):
            existing_controller = read_json(CONTROLLER_CONFIG_PATH)
            controller_candidate = build_controller_candidate(
                existing_controller, installed_utc
            )
            operations.append(
                (
                    "controller tuning",
                    CONTROLLER_CONFIG_PATH,
                    controller_candidate,
                )
            )

        operations.append(("IPM profile", IPM_CONFIG_PATH, ipm_candidate))

        # Stage and re-read all candidates before creating backups or replacing
        # any live file. Temporary files share the target filesystem, which is
        # required for atomic os.replace semantics.
        staged = []
        try:
            for label, path, candidate in operations:
                staged_path = stage_json(path, candidate)
                staged.append((label, path, candidate, staged_path))
            validate_single_line_config(read_json(staged[-1][3]))

            if arguments.dry_run:
                print("SINGLE-LINE PROFILE DRY RUN: PASS")
                print("No files were changed and no backups were created.")
                print("Detector: {}".format(PROFILE_VERSION))
                print("Target:   {}".format(TARGET_LINE_ROLE))
                print("Physical IPM source/destination geometry will be preserved.")
                for label, path, _candidate, _staged_path in staged:
                    print("WOULD RESET: {} -> {}".format(label, path))
                print_next_steps()
                return 0

            stamp = utc_stamp()
            backups = {}
            for _label, path, _candidate, _staged_path in staged:
                backups[path] = verified_backup(path, stamp)

            replaced = []
            try:
                for label, path, _candidate, staged_path in staged:
                    os.replace(staged_path, path)
                    replaced.append((label, path))

                # Verify the installed end state while rollback is still in
                # scope. Any mismatch restores every already-replaced file.
                installed_ipm = read_json(IPM_CONFIG_PATH)
                validate_single_line_config(installed_ipm)
                if canonical(
                    installed_ipm["source_points_normalized"]
                ) != canonical(existing_ipm["source_points_normalized"]):
                    raise RuntimeError("Installed source geometry changed.")
                if canonical(
                    installed_ipm["destination_points_normalized"]
                ) != canonical(
                    existing_ipm["destination_points_normalized"]
                ):
                    raise RuntimeError("Installed destination geometry changed.")
            except Exception as replacement_error:
                rollback_errors = []
                for _label, path in reversed(replaced):
                    try:
                        atomic_restore(path, backups[path])
                    except Exception as rollback_error:
                        rollback_errors.append(
                            "{}: {}".format(path, rollback_error)
                        )
                message = "Atomic migration failed and was rolled back: {}".format(
                    replacement_error
                )
                if rollback_errors:
                    message += " | ROLLBACK ERRORS: {}".format(
                        "; ".join(rollback_errors)
                    )
                raise RuntimeError(message)
            staged = []

            print("SINGLE-LINE OUTER-CIRCLE PROFILE INSTALLED")
            print("Detector: {}".format(PROFILE_VERSION))
            print("Target:   {}".format(TARGET_LINE_ROLE))
            print("Calibration state: {}".format(REVALIDATION_STATE))
            print("Physical source/destination geometry was preserved exactly.")
            print("All existing controller and gate evidence was safely reset.")
            for label, path in replaced:
                print("UPDATED {}: {}".format(label, path))
                print("BACKUP: {}".format(backups[path]))
            print_next_steps()
            return 0
        finally:
            for _label, _path, _candidate, staged_path in staged:
                try:
                    os.unlink(staged_path)
                except OSError:
                    pass
    except Exception as error:
        print("PROFILE INSTALLATION REFUSED: {}".format(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
