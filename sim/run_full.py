from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def require_docker() -> str:
    docker = shutil.which("docker")
    if docker is None:
        raise RuntimeError(
            "Docker is required for build.py test --all. Install Docker Engine "
            "or Docker Desktop, then retry."
        )
    probe = subprocess.run(
        [docker, "info", "--format", "{{.ServerVersion}}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if probe.returncode != 0:
        detail = (probe.stderr or probe.stdout).strip()
        raise RuntimeError(
            "Docker is installed, but its daemon is not available. On Linux, start "
            "it with 'sudo systemctl start docker'; if access is denied, add your "
            "user to the docker group and log in again. On Docker Desktop, start "
            "the application."
            + (f"\nDocker reported: {detail}" if detail else "")
        )
    return docker


def load_layout_for_build(repo_root: Path, build_subdir: str | None) -> dict:
    layout = json.loads((repo_root / "sim" / "board.json").read_text(encoding="utf-8"))
    if build_subdir is None:
        return layout
    for name, value in layout.get("artifacts", {}).items():
        parts = Path(value).parts
        if len(parts) >= 3 and parts[0] == "build":
            layout["artifacts"][name] = str(
                Path("build", build_subdir, *parts[2:])
            )
    return layout


def run_full_simulation(
    ui, repo_root: Path, architecture: str, build_subdir: str | None = None
) -> None:
    """Run this board's file-defined simulation inside Docker."""
    docker = require_docker()

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

    layout = load_layout_for_build(repo_root, build_subdir)

    with tempfile.TemporaryDirectory(prefix="seds-firmware-layout-") as directory:
        layout_path = Path(directory) / "board.json"
        layout_path.write_text(json.dumps(layout, indent=2), encoding="utf-8")
        command = [
            docker, "run", "--platform", "linux/amd64", "--rm",
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "run",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
        ]
        ui.say("run", " ".join(command))
        subprocess.run(command, check=True)
