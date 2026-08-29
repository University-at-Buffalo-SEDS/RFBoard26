from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess


def run_full_simulation(ui, repo_root: Path, architecture: str) -> None:
    """Run this board's file-defined simulation inside Docker."""
    docker = shutil.which("docker")
    if docker is None:
        raise RuntimeError("Docker is required for build.py test --full")

    image = os.environ.get(
        "SEDS_FIRMWARE_SIM_IMAGE",
        f"ghcr.io/university-at-buffalo-seds/firmwaresimulator:{architecture}",
    )
    simulator_source = repo_root.parent / "FirmwareSimulator"
    if simulator_source.joinpath("Dockerfile").is_file():
        image = f"seds-firmware-simulator:{architecture}-local"
        build = [
            docker, "build", "--platform", "linux/amd64",
            "--build-arg", f"SIM_ARCH={architecture}",
            "-t", image, str(simulator_source),
        ]
        ui.say("run", " ".join(build))
        subprocess.run(build, check=True)

    command = [
        docker, "run", "--platform", "linux/amd64", "--rm",
        "-v", f"{repo_root}:/firmware:ro",
        image, "run",
        "--layout", "/firmware/sim/board.json",
        "--firmware-root", "/firmware",
    ]
    ui.say("run", " ".join(command))
    subprocess.run(command, check=True)
