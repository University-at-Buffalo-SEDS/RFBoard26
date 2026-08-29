from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

SIMULATOR_REPOSITORY = (
    "https://github.com/University-at-Buffalo-SEDS/FirmwareSimulator.git"
)
SIMULATOR_INTERFACE_VERSION = "2"


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


def _image_exists(docker: str, image: str) -> bool:
    result = subprocess.run(
        [docker, "image", "inspect", image],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def _build_simulator_image(
    ui, docker: str, source: Path, architecture: str, image: str
) -> None:
    build = [
        docker, "build", "--platform", "linux/amd64",
        "--build-arg", f"SIM_ARCH={architecture}",
        "-t", image, str(source),
    ]
    ui.say("run", " ".join(build))
    subprocess.run(build, check=True)


def resolve_simulator_image(ui, docker: str, repo_root: Path, architecture: str) -> str:
    requested = os.environ.get(
        "SEDS_FIRMWARE_SIM_IMAGE",
        f"ghcr.io/university-at-buffalo-seds/firmwaresimulator:{architecture}",
    )
    local = (
        f"seds-firmware-simulator:{architecture}-local-"
        f"v{SIMULATOR_INTERFACE_VERSION}"
    )
    configured_source = os.environ.get("SEDS_FIRMWARE_SIM_SOURCE")
    if configured_source:
        source = Path(configured_source).expanduser().resolve()
        if not source.joinpath("Dockerfile").is_file():
            raise RuntimeError(
                f"SEDS_FIRMWARE_SIM_SOURCE does not contain a Dockerfile: {source}"
            )
        _build_simulator_image(ui, docker, source, architecture, local)
        return local

    ui.say("run", f"{docker} pull {requested}")
    pull = subprocess.run(
        [docker, "pull", requested],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if pull.returncode == 0:
        return requested
    if _image_exists(docker, requested):
        return requested
    if _image_exists(docker, local):
        return local

    git = shutil.which("git")
    if git is None:
        raise RuntimeError(
            "The published simulator image is unavailable and git is required "
            "to build it from source."
        )
    ui.say(
        "info",
        "Published simulator image unavailable; cloning FirmwareSimulator and "
        "building a local image.",
    )
    with tempfile.TemporaryDirectory(prefix="seds-firmware-simulator-") as directory:
        source = Path(directory) / "FirmwareSimulator"
        try:
            subprocess.run(
                [
                    git, "clone", "--depth", "1", "--branch", "main",
                    SIMULATOR_REPOSITORY, str(source),
                ],
                check=True,
            )
            _build_simulator_image(ui, docker, source, architecture, local)
        except subprocess.CalledProcessError as exc:
            detail = (pull.stderr or pull.stdout).strip()
            raise RuntimeError(
                "The simulator image could not be pulled and its source fallback "
                "could not be built."
                + (f"\nRegistry reported: {detail}" if detail else "")
            ) from exc
    return local


def write_container_layout(directory: Path, layout: dict) -> Path:
    directory.chmod(0o755)
    layout_path = directory / "board.json"
    layout_path.write_text(json.dumps(layout, indent=2), encoding="utf-8")
    layout_path.chmod(0o644)
    return layout_path


def run_full_simulation(
    ui, repo_root: Path, architecture: str, build_subdir: str | None = None
) -> None:
    """Run this board's file-defined simulation inside Docker."""
    docker = require_docker()

    image = resolve_simulator_image(ui, docker, repo_root, architecture)

    layout = load_layout_for_build(repo_root, build_subdir)

    with tempfile.TemporaryDirectory(prefix="seds-firmware-layout-") as directory:
        write_container_layout(Path(directory), layout)
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
