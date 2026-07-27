#!/usr/bin/env python3
"""Deterministic source-tree and Git-integrity helpers.

Only source/configuration files that can affect the YOLO runtime are included;
generated detections, bytecode caches, Git metadata, and model weights are
handled separately. Compatible with Python 3.6.
"""

from __future__ import print_function

import hashlib
import os
import subprocess


SOURCE_SUFFIXES = (".py", ".yaml", ".yml")
EXCLUDED_DIRECTORIES = frozenset((".git", "__pycache__", "runs"))


def source_tree_sha256(root_directory):
    """Return ``(digest, file_count)`` for a deterministic source snapshot."""
    root_directory = os.path.realpath(os.path.abspath(root_directory))
    if not os.path.isdir(root_directory):
        raise RuntimeError(
            "Source directory does not exist: {}".format(root_directory)
        )

    source_paths = []
    for current_root, directory_names, file_names in os.walk(
        root_directory, followlinks=False
    ):
        directory_names[:] = sorted(
            name
            for name in directory_names
            if name not in EXCLUDED_DIRECTORIES
        )
        for file_name in sorted(file_names):
            if file_name.lower().endswith(SOURCE_SUFFIXES):
                source_paths.append(os.path.join(current_root, file_name))

    source_paths.sort(
        key=lambda path: os.path.relpath(path, root_directory).replace(
            os.sep, "/"
        )
    )
    if not source_paths:
        raise RuntimeError(
            "No YOLO source files were found in {}".format(root_directory)
        )

    digest = hashlib.sha256()
    for source_path in source_paths:
        relative_path = os.path.relpath(
            source_path, root_directory
        ).replace(os.sep, "/")
        digest.update(relative_path.encode("utf-8"))
        digest.update(b"\x00")
        with open(source_path, "rb") as source_file:
            while True:
                block = source_file.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
        digest.update(b"\x00")
    return digest.hexdigest(), len(source_paths)


def command_output(command, cwd, timeout_seconds=8):
    try:
        output = subprocess.check_output(
            command,
            cwd=cwd,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=timeout_seconds,
        )
        return True, output.strip()
    except Exception as error:
        return False, repr(error)


def git_source_state(repository_directory):
    """Return commit plus tracked and untracked source cleanliness."""
    commit_ok, commit = command_output(
        ["git", "rev-parse", "HEAD"], repository_directory
    )
    status_ok, status = command_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        repository_directory,
    )
    untracked_ok, untracked_output = command_output(
        ["git", "ls-files", "--others", "--exclude-standard"],
        repository_directory,
    )
    untracked_source_files = []
    if untracked_ok:
        for relative_path in untracked_output.splitlines():
            normalized = relative_path.strip().replace("\\", "/")
            components = normalized.split("/")
            if (
                normalized.lower().endswith(SOURCE_SUFFIXES)
                and not any(
                    component in EXCLUDED_DIRECTORIES
                    for component in components
                )
            ):
                untracked_source_files.append(normalized)
    tracked_source_clean = bool(status_ok and not status)
    return {
        "commit_available": bool(commit_ok),
        "commit": commit if commit_ok else None,
        "commit_error": None if commit_ok else commit,
        "tracked_status_available": bool(status_ok),
        "tracked_status": status if status_ok else None,
        "tracked_status_error": None if status_ok else status,
        "tracked_source_clean": tracked_source_clean,
        "untracked_source_status_available": bool(untracked_ok),
        "untracked_source_files": sorted(untracked_source_files),
        "untracked_source_status_error": (
            None if untracked_ok else untracked_output
        ),
        "source_tree_clean": bool(
            tracked_source_clean
            and untracked_ok
            and not untracked_source_files
        ),
    }
