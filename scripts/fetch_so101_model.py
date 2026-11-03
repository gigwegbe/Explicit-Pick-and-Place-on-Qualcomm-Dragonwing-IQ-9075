#!/usr/bin/env python3
"""Fetch the official SO-101 MuJoCo model without vendoring it in this project."""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def copy_model(src: Path, dest: Path) -> None:
    required = src / "so101_new_calib.xml"
    assets = src / "assets"
    if not required.exists() or not assets.is_dir():
        raise SystemExit(f"Not a valid SO101 simulation directory: {src}")
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(required, dest / required.name)
    shutil.copytree(assets, dest / "assets", dirs_exist_ok=True)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--source", help="Existing path to SO-ARM100/Simulation/SO101")
    p.add_argument("--dest", default=str(Path(__file__).resolve().parents[1] / "model"))
    args = p.parse_args()

    dest = Path(args.dest).resolve()
    if args.source:
        copy_model(Path(args.source).expanduser().resolve(), dest)
        print(f"SO-101 model copied to {dest}")
        return

    if shutil.which("git") is None:
        raise SystemExit("git is required, or pass --source to an existing SO101 model directory.")

    with tempfile.TemporaryDirectory(prefix="so101-model-") as td:
        repo = Path(td) / "SO-ARM100"
        subprocess.run(
            ["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
             "https://github.com/TheRobotStudio/SO-ARM100.git", str(repo)],
            check=True,
        )
        subprocess.run(
            ["git", "-C", str(repo), "sparse-checkout", "set", "Simulation/SO101"],
            check=True,
        )
        copy_model(repo / "Simulation" / "SO101", dest)
    print(f"SO-101 model fetched to {dest}")


if __name__ == "__main__":
    main()
