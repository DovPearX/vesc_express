#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

DEFAULT_REMOTE = "https://github.com/svenssonjoel/lispBM.git"
DEFAULT_BRANCH = "master"
DEFAULT_PREFIX = "main/lispBM"

def run(cmd: list[str], capture: bool = False, cwd: Path | None = None) -> str:
    try:
        result = subprocess.run(
            cmd,
            check=True,
            cwd=cwd,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
            text=True,
        )
        return result.stdout.strip() if capture else ""
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {' '.join(cmd)}", file=sys.stderr)
        if capture and e.stderr:
            print(e.stderr, file=sys.stderr)
        raise


def find_git_root(start: Path) -> Path | None:
    current = start.resolve()

    while True:
        if (current / ".git").exists():
            return current

        if current.parent == current:
            return None

        current = current.parent


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Update git subtree (auto-detect repo root)."
    )
    parser.add_argument("--remote", default=DEFAULT_REMOTE)
    parser.add_argument("--branch", default=DEFAULT_BRANCH)
    parser.add_argument("--prefix", default=DEFAULT_PREFIX)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check if update is needed before pulling",
    )
    args = parser.parse_args()

    repo_root = find_git_root(Path.cwd())

    if repo_root is None:
        print("Error: Not inside a git repository.", file=sys.stderr)
        return 1

    print(f"Using repo root: {repo_root}")

    if args.check:
        print("Checking for upstream changes...")

        try:
            run(["git", "fetch", args.remote, args.branch], cwd=repo_root)
        except subprocess.CalledProcessError:
            print("Failed to fetch upstream.", file=sys.stderr)
            return 1

        try:
            local_commit = run(
                ["git", "subtree", "split", "--prefix", args.prefix],
                capture=True,
                cwd=repo_root,
            )
            remote_commit = run(
                ["git", "rev-parse", "FETCH_HEAD"],
                capture=True,
                cwd=repo_root,
            )
        except subprocess.CalledProcessError:
            print("Failed to compare commits.", file=sys.stderr)
            return 1

        if local_commit == remote_commit:
            print("Subtree is already up to date.")
            return 0

        print("Changes detected. Updating subtree...")

    try:
        run(
            [
                "git",
                "subtree",
                "pull",
                "--prefix",
                args.prefix,
                args.remote,
                args.branch,
                "--squash",
            ],
            cwd=repo_root,
        )
        print("Subtree updated successfully.")
    except subprocess.CalledProcessError:
        print("Subtree pull failed.", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())