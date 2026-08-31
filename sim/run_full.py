from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

SIMULATOR_REPOSITORY = (
    "https://github.com/University-at-Buffalo-SEDS/FirmwareSimulator.git"
)
SIMULATOR_INTERFACE_VERSION = "0.3.2"
FIRMWARE_BRANCH = "migration/sedlaunch-sedsnet-mainline"
FIRMWARE_ORGANIZATION = "University-at-Buffalo-SEDS"


def run_live(command: list[str], label: str) -> None:
    """Run a quiet simulator command with visible liveness updates."""
    print(f"[SIM] {label} started", flush=True)
    started = time.monotonic()
    next_update = started + 5.0
    process = subprocess.Popen(command)
    while process.poll() is None:
        now = time.monotonic()
        if now >= next_update:
            print(f"[SIM] {label} running ({int(now - started)}s elapsed)", flush=True)
            next_update = now + 5.0
        time.sleep(0.25)
    if process.returncode != 0:
        raise subprocess.CalledProcessError(process.returncode, command)
    print(f"[SIM] {label} completed ({int(time.monotonic() - started)}s)", flush=True)


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


def _build_simulator_image(ui, docker: str, source: Path, image: str) -> None:
    build = [
        docker, "build",
        "--progress=plain",
        "-t", image, str(source),
    ]
    ui.say("run", " ".join(build))
    subprocess.run(build, check=True)


def resolve_simulator_image(ui, docker: str, repo_root: Path, _architecture: str) -> str:
    requested = os.environ.get(
        "SEDS_FIRMWARE_SIM_IMAGE",
        "ghcr.io/university-at-buffalo-seds/firmwaresimulator:latest",
    )
    local = f"seds-firmware-simulator:local-v{SIMULATOR_INTERFACE_VERSION}"
    configured_source = os.environ.get("SEDS_FIRMWARE_SIM_SOURCE")
    if configured_source:
        source = Path(configured_source).expanduser().resolve()
        if not source.joinpath("Dockerfile").is_file():
            raise RuntimeError(
                f"SEDS_FIRMWARE_SIM_SOURCE does not contain a Dockerfile: {source}"
            )
        _build_simulator_image(ui, docker, source, local)
        return local

    ui.say("run", f"{docker} pull {requested}")
    # Inherit the terminal streams so layer downloads and extraction remain
    # visible. Capturing these pipes makes a large image pull look hung.
    pull = subprocess.run([docker, "pull", requested])
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
            _build_simulator_image(ui, docker, source, local)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                "The simulator image could not be pulled and its source fallback "
                "could not be built."
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
            docker, "run", "--rm",
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "run",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
        ]
        ui.say("run", " ".join(command))
        run_live(command, "firmware simulation")


def run_memory_profile(
    ui, repo_root: Path, architecture: str, build_subdir: str | None = None
) -> None:
    """Run a long, repeatedly sampled allocator profile against the real ELF."""
    docker = require_docker()
    image = resolve_simulator_image(ui, docker, repo_root, architecture)
    layout = load_layout_for_build(repo_root, build_subdir)
    probes = layout.get("execution", {}).get("memory_probes", [])
    layout["execution"]["memory_probes"] = [
        probe for probe in probes
        if probe.get("name") not in {"network_ready", "discovery_seen", "timesync_valid"}
    ]
    layout["execution"]["memory_probe_warmup_samples"] = 3

    with tempfile.TemporaryDirectory(prefix="seds-firmware-profile-") as directory:
        write_container_layout(Path(directory), layout)
        command = [
            docker, "run", "--rm",
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "profile",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
            "--virtual-time-ms", "10000",
            "--sample-count", "20",
            "--traffic-iterations", "1000000",
        ]
        ui.say("run", " ".join(command))
        run_live(command, "long-duration memory profile")


def _network_peer(repo_root: Path) -> tuple[str, Path]:
    current = json.loads(
        (repo_root / "sim" / "board.json").read_text(encoding="utf-8")
    )["name"]
    peer_name = "PowerBoard26" if current == "RFBoard26" else "RFBoard26"
    configured = os.environ.get("SEDS_FIRMWARE_SIM_PEER_ROOT")
    if configured:
        peer_root = Path(configured).expanduser().resolve()
        if not (peer_root / "sim" / "board.json").is_file():
            raise RuntimeError(
                f"SEDS_FIRMWARE_SIM_PEER_ROOT is not a firmware repository: {peer_root}"
            )
        return peer_name, peer_root

    peer_root = repo_root / "build" / "sim-network-peer" / peer_name
    git = shutil.which("git")
    if git is None:
        raise RuntimeError("git is required to obtain the network-test peer firmware.")
    repository = (
        f"https://github.com/{FIRMWARE_ORGANIZATION}/{peer_name}.git"
    )
    if (peer_root / ".git").is_dir():
        subprocess.run(
            [git, "fetch", "origin", FIRMWARE_BRANCH], cwd=peer_root, check=True
        )
        subprocess.run(
            [git, "checkout", FIRMWARE_BRANCH], cwd=peer_root, check=True
        )
        subprocess.run(
            [git, "pull", "--ff-only", "origin", FIRMWARE_BRANCH],
            cwd=peer_root, check=True,
        )
    else:
        peer_root.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [git, "clone", "--depth", "1", "--branch", FIRMWARE_BRANCH,
             repository, str(peer_root)],
            check=True,
        )
    return peer_name, peer_root


def run_network_simulation(
    ui, repo_root: Path, architecture: str, build_subdir: str | None = None
) -> None:
    """Boot this board with a real peer and require discovery + time sync."""
    docker = require_docker()
    image = resolve_simulator_image(ui, docker, repo_root, architecture)
    peer_name, peer_root = _network_peer(repo_root)
    release = build_subdir is not None and "release" in build_subdir.lower()
    simulation_env = os.environ.copy()
    simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"
    current_build = [
        sys.executable, str(repo_root / "build.py"), "build",
        "--release" if release else "--debug", "--image", "firmware",
    ]
    ui.say("run", " ".join(current_build))
    subprocess.run(current_build, cwd=repo_root, env=simulation_env, check=True)
    peer_build = [
        sys.executable, str(peer_root / "build.py"), "build",
        "--release" if release else "--debug", "--image", "firmware",
    ]
    ui.say("run", " ".join(peer_build))
    subprocess.run(peer_build, cwd=peer_root, env=simulation_env, check=True)

    current_layout = load_layout_for_build(repo_root, build_subdir)
    peer_layout = load_layout_for_build(peer_root, None)
    for layout in (current_layout, peer_layout):
        probes = layout.get("execution", {}).get("memory_probes", [])
        layout["execution"]["memory_probes"] = [
            probe for probe in probes
            if probe.get("name") in {"network_ready", "discovery_seen", "timesync_valid"}
        ]
        layout["execution"]["memory_probe_warmup_samples"] = 3
    current_can = "fdcan2" if current_layout["architecture"] == "stm32g4" else "fdcan1"
    peer_can = "fdcan2"
    topology = {
        "name": f"{current_layout['name']}-{peer_name}-compatibility",
        "quantum_seconds": 0.0001,
        "virtual_time_ms": 10000,
        "sample_count": 20,
        "nodes": [
            {"name": "board", "layout": "/simulation/board.json",
             "firmware_root": "/board"},
            {"name": "peer", "layout": "/simulation/peer.json",
             "firmware_root": "/peer"},
        ],
        "links": [{
            "name": "sedsnet_can", "kind": "can",
            "endpoints": [
                {"node": "board", "peripheral": current_can,
                 "activity_probe": "network_ready", "minimum_activity": 1},
                {"node": "peer", "peripheral": peer_can,
                 "activity_probe": "network_ready", "minimum_activity": 1},
            ],
        }],
    }

    with tempfile.TemporaryDirectory(prefix="seds-firmware-network-") as directory:
        root = Path(directory)
        write_container_layout(root, current_layout)
        (root / "peer.json").write_text(
            json.dumps(peer_layout, indent=2), encoding="utf-8"
        )
        (root / "topology.json").write_text(
            json.dumps(topology, indent=2), encoding="utf-8"
        )
        for path in (root / "peer.json", root / "topology.json"):
            path.chmod(0o644)
        command = [
            docker, "run", "--rm",
            "-v", f"{repo_root}:/board:ro",
            "-v", f"{peer_root}:/peer:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "bay", "--topology", "/simulation/topology.json",
        ]
        ui.say("run", " ".join(command))
        run_live(command, "linked network simulation")
