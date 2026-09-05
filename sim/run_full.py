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
SIMULATOR_INTERFACE_VERSION = "0.4"
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

    # An explicitly selected local development image should be used as-is.
    # Mutable published tags (the default `latest`) are still pulled below so
    # normal board tests automatically pick up simulator releases.
    if "SEDS_FIRMWARE_SIM_IMAGE" in os.environ and _image_exists(docker, requested):
        return requested

    ui.say("run", f"{docker} pull {requested}")
    # Always refresh mutable tags such as latest. Inherit terminal streams so
    # layer downloads and extraction remain visible instead of looking hung.
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
    # Instruction tracing retains every executed PC and can consume gigabytes
    # while an isolated controller repeatedly retries CAN. Register and memory
    # probes still provide the required fault diagnostics for this soak.
    layout["execution"]["trace"] = False

    with tempfile.TemporaryDirectory(prefix="seds-firmware-profile-") as directory:
        write_container_layout(Path(directory), layout)
        command = [
            docker, "run", "--rm",
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "profile",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
            # Renode executes every firmware instruction. The board layout's
            # accelerated HAL tick makes 20 ms sufficient to reach steady
            # scheduler state; allocator longevity is exercised separately by
            # the one-million-packet traffic model below. Longer instruction
            # windows consume unbounded host resources without increasing the
            # modeled STM32 RAM coverage.
            "--virtual-time-ms", "20",
            "--sample-count", "20",
            "--traffic-iterations", "1000000",
        ]
        ui.say("run", " ".join(command))
        run_live(command, "allocator stress and firmware memory profile")


def run_unacknowledged_can_simulation(
    ui, repo_root: Path, architecture: str, build_subdir: str | None = None
) -> None:
    """Prove firmware remains alive when it is the only node on its CAN bus."""
    docker = require_docker()
    image = resolve_simulator_image(ui, docker, repo_root, architecture)
    layout = load_layout_for_build(repo_root, build_subdir)
    probes = layout.get("execution", {}).get("memory_probes", [])
    layout["execution"]["memory_probes"] = [
        probe for probe in probes
        if probe.get("name") not in {"network_ready", "discovery_seen", "timesync_valid"}
    ]
    layout["execution"]["memory_probe_warmup_samples"] = 2
    # An isolated node retries quickly. Keep register and memory probes, but do
    # not retain an unbounded instruction trace in the simulator process.
    layout["execution"]["trace"] = False
    for probe in layout["execution"]["memory_probes"]:
        if probe.get("name") == "fdcan_tx_fail":
            probe.pop("maximum", None)
            probe["minimum"] = 1

    with tempfile.TemporaryDirectory(prefix="seds-firmware-isolated-can-") as directory:
        write_container_layout(Path(directory), layout)
        command = [
            docker, "run", "--rm",
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "profile",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
            "--can-unacknowledged",
            "--virtual-time-ms", "250",
            "--sample-count", "5",
            "--traffic-iterations", "100000",
        ]
        ui.say("run", " ".join(command))
        run_live(command, "disconnected CAN survival simulation")


def _network_peer(repo_root: Path) -> tuple[str, Path]:
    current = json.loads(
        (repo_root / "sim" / "board.json").read_text(encoding="utf-8")
    )["name"]
    # Exercise the link this board uses in the vehicle. Actuation telemetry
    # reaches GroundStation through Gateway, not through RFBoard directly.
    if current == "ActuationBoard":
        peer_name = "gateway-board26"
    else:
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
    """Boot all firmware plus the GroundStation26 host and prove end-to-end SEDSNet."""
    docker = require_docker()
    image = resolve_simulator_image(ui, docker, repo_root, architecture)
    release = build_subdir is not None and "release" in build_subdir.lower()
    simulation_env = os.environ.copy()
    simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"
    boards = [
        ("rf", "RFBoard26", "RFBoard26", 1, "fdcan2"),
        ("power", "PowerBoard26", "PowerBoard26", 2, "fdcan2"),
        ("flight", "FlightComputer26", "FlightComputer26", 4, "fdcan1"),
        ("gateway", "gateway-board", "gateway_board", 8, "fdcan2"),
        ("actuator", "ActuatorBoard26", "ActuationBoard", 16, "fdcan2"),
        ("valve", "ValveBoard26", "Valve_Board26", 32, "fdcan2"),
        ("daq", "DAQ-Board", "DAQ-Board", 64, "fdcan1"),
    ]
    current_name = json.loads((repo_root / "sim" / "board.json").read_text(encoding="utf-8"))["name"]
    roots: dict[str, Path] = {}
    configured_suite_root = os.environ.get("SEDS_FIRMWARE_SIM_SUITE_ROOT")
    suite_root = (
        Path(configured_suite_root).expanduser().resolve()
        if configured_suite_root
        else repo_root / "build" / "sim-full-bay"
    )
    git = shutil.which("git")
    if git is None:
        raise RuntimeError("git is required to obtain the full-bay firmware repositories.")
    for node, repository, layout_name, _bit, _can in boards:
        if layout_name == current_name:
            root = repo_root
        else:
            root = suite_root / repository
            remote = f"https://github.com/{FIRMWARE_ORGANIZATION}/{repository}.git"
            if configured_suite_root:
                if not (root / ".git").is_dir():
                    raise RuntimeError(
                        "SEDS_FIRMWARE_SIM_SUITE_ROOT is missing firmware repository: "
                        f"{root}"
                    )
            elif (root / ".git").is_dir():
                subprocess.run([git, "fetch", "origin", FIRMWARE_BRANCH], cwd=root, check=True)
                subprocess.run([git, "checkout", FIRMWARE_BRANCH], cwd=root, check=True)
                subprocess.run([git, "pull", "--ff-only", "origin", FIRMWARE_BRANCH], cwd=root, check=True)
            else:
                root.parent.mkdir(parents=True, exist_ok=True)
                subprocess.run([git, "clone", "--depth", "1", "--branch", FIRMWARE_BRANCH, remote, str(root)], check=True)
        roots[node] = root
        if repository == "FlightComputer26":
            command = [sys.executable, str(root / "build.py"), "release" if release else "debug", "firmware"]
        else:
            command = [sys.executable, str(root / "build.py"), "build", "--release" if release else "--debug", "--image", "firmware"]
        if os.environ.get("SEDS_FIRMWARE_SIM_SKIP_BUILD") != "1":
            ui.say("run", " ".join(command))
            subprocess.run(command, cwd=root, env=simulation_env, check=True)

    layouts: dict[str, dict] = {}
    for node, _repository, _layout_name, _bit, _can in boards:
        layout = load_layout_for_build(roots[node], None)
        layout["execution"]["memory_probe_warmup_samples"] = 0
        layouts[node] = layout
    topology = {
        "name": "complete-seds-avionics-and-fill-network",
        "quantum_seconds": 0.0001,
        # The linked test proves connectivity; each board's separate profile
        # stage performs the long-duration allocator qualification.
        # The Pico-Fi/radio path deliberately models constrained serial links.
        # Leave enough virtual time for the open command and its status ACK to
        # traverse both directions across the constrained serial links.
        "virtual_time_ms": 10000,
        "sample_count": 5,
        "enforce_end_drop": False,
        # Reboot the FC after GroundStation has published and the FC has
        # persisted underglow=1. Renode retains physical flash across this
        # reset, matching a real power cycle while all network peers stay up.
        "reboots": [
            {"node": "flight", "after_sample": 2},
            {"node": "power", "after_sample": 2},
        ],
        "nodes": [
            {"name": node, "layout": f"/simulation/{node}.json", "firmware_root": f"/nodes/{node}"}
            for node, *_ in boards
        ],
        "host_nodes": [
            {
                "name": "groundstation",
                "binary": "/usr/local/bin/groundstation_backend",
                "cwd": "/opt/groundstation/backend",
                "env": {
                    "GS_DEBUG_PRINTS": "0",
                    "GS_RADIO_DIAGNOSTICS": "1",
                    "RUST_LOG": "info",
                    "GS_SIMULATED_SERIAL_PTY": "1",
                    "GS_LAYOUT_PATH": "/opt/groundstation/backend/layout/layout_hitl.json",
                    "GS_AV_BAY_UNDERGLOW_DEFAULT": "1",
                    "GS_FLIGHT_STATE_DEFAULT": "1",
                    "GS_SIM_UNDERGLOW_SEQUENCE": "1,0,1",
                    "GS_SIM_FLIGHT_BUZZER_SEQUENCE": "1,0,1",
                    "GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1",
                    "GS_HEARTBEAT_INTERVAL_MS": "7000",
                    "GS_SIM_DISABLE_PERIODIC_DISCOVERY": "1",
                    "GS_SIM_COMPACT_INITIAL_DISCOVERY": "1",
                    "GS_SIM_EXPECT_DISCOVERY_NODES": "RF,PB,FC,GB,AB,VB,DAQ",
                    "GS_SIM_FLIGHT_STATE_SEQUENCE": "1,0,1"
                },
                "serial_links": [
                    {"link": "rocket_radio", "env": "GS_AV_BAY_SERIAL_PORT"},
                    {
                        "link": "fill_pico",
                        "env": "GS_SIMULATED_I2C_SOCKET",
                        "transport": "pico_fi_i2c_to_uart"
                    }
                ]
            }
        ],
        "links": [
            {"name": "avionics_can", "kind": "can", "transport_path": ["RFBoard", "PowerBoard", "FlightComputer"],
             "endpoints": [{"node": node, "peripheral": can, "tx_probe": "fdcan_tx_ok", "rx_probe": "fdcan_rx"} for node, _repo, _name, _bit, can in boards[:3]]},
            {"name": "rocket_radio", "kind": "radio",
             "transport_path": ["RF E22 radio", "GroundStation26 host binary"],
             "endpoints": [
                 {"node": "rf", "peripheral": "usart1", "tx_probe": "radio_tx_frames", "rx_probe": "radio_rx_frames"},
                 {"node": "groundstation", "peripheral": "av_bay"}]},
            {"name": "fill_pico", "kind": "pico_fi",
             "transport_path": ["GroundStation26 host binary", "Pico-Fi pair", "Gateway USART2"],
             "endpoints": [
                 {"node": "groundstation", "peripheral": "fill_box"},
                 {"node": "gateway", "peripheral": "usart2", "tx_probe": "uart_tx_frames", "rx_probe": "uart_rx_frames"}]},
            {"name": "fill_can", "kind": "can", "transport_path": ["Gateway", "Actuator", "Valve", "DAQ"],
             "endpoints": [{"node": node, "peripheral": can, "tx_probe": "fdcan_tx_ok", "rx_probe": "fdcan_rx"} for node, _repo, _name, _bit, can in boards[3:]]},
        ],
        # A routed network is not a flat broadcast domain. Require every peer
        # on each local CAN segment plus traffic in both directions across the
        # RF/GroundStation/Pico-Fi/Gateway route.
        "assertions": [
            {"name": "Gateway received GroundStation valve command", "node": "gateway", "probe": "uart_valve_command_count", "minimum": 1},
            {"name": "Gateway routed valve command onto CAN", "node": "gateway", "probe": "can_valve_command_tx_count", "minimum": 1},
            {"name": "GroundStation valve command reached board", "node": "valve", "probe": "valve_commands_received", "minimum": 1},
            {"name": "Valve executed GroundStation command", "node": "valve", "probe": "valve_commands_executed", "minimum": 1},
            {"name": "Valve applied pilot-open command", "node": "valve", "probe": "pilot_valve_state", "minimum": 1},
            {"name": "Valve produced status ACK", "node": "valve", "probe": "umbilical_status_ok", "minimum": 1},
            {"name": "Valve transmitted pilot-open status", "node": "valve", "probe": "pilot_open_status_wire_tx", "minimum": 1},
            {"name": "Gateway received status ACK over CAN", "node": "gateway", "probe": "can_umbilical_status_count", "minimum": 1},
            {"name": "Gateway received pilot-open status", "node": "gateway", "probe": "gateway_pilot_open_status", "minimum": 1},
            {"name": "Gateway forwarded status ACK to GroundStation over UART", "node": "gateway", "probe": "uart_umbilical_status_count", "minimum": 1},
            {"name": "rf applied GroundStation underglow variable", "node": "rf", "probe": "underglow_updates", "minimum": 1},
            {"name": "power applied GroundStation underglow variable", "node": "power", "probe": "underglow_updates", "minimum": 1},
            {"name": "flight applied GroundStation underglow variable", "node": "flight", "probe": "underglow_updates", "minimum": 1},
            {"name": "rf persisted the 1-0-1 underglow sequence", "node": "rf", "probe": "underglow_persist_writes", "minimum": 3},
            {"name": "power persisted the 1-0-1 underglow sequence", "node": "power", "probe": "underglow_persist_writes", "minimum": 3},
            {"name": "flight persisted the 1-0-1 underglow sequence", "node": "flight", "probe": "underglow_persist_writes", "minimum": 3},
            {"name": "rf persistence remained healthy", "node": "rf", "probe": "underglow_persist_errors", "maximum": 0},
            {"name": "power persistence remained healthy", "node": "power", "probe": "underglow_persist_errors", "maximum": 0},
            {"name": "flight persistence remained healthy", "node": "flight", "probe": "underglow_persist_errors", "maximum": 0},
            {"name": "flight applied GroundStation buzzer variable", "node": "flight", "probe": "flight_buzzer_updates", "minimum": 3},
            {"name": "flight persisted the 1-0-1 buzzer sequence", "node": "flight", "probe": "flight_buzzer_persist_writes", "minimum": 3},
            {"name": "flight buzzer persistence remained healthy", "node": "flight", "probe": "flight_buzzer_persist_errors", "maximum": 0},
            {"name": "flight buzzer finished enabled", "node": "flight", "probe": "flight_buzzer_enabled", "minimum": 1},
            {"name": "rf underglow is enabled", "node": "rf", "probe": "underglow_enabled", "minimum": 1},
            {"name": "power underglow is enabled", "node": "power", "probe": "underglow_enabled", "minimum": 1},
            {"name": "flight underglow is enabled", "node": "flight", "probe": "underglow_enabled", "minimum": 1},
            {"name": "flight restored underglow from retained flash after reboot", "node": "flight", "probe": "underglow_boot_restore_valid", "minimum": 1},
            {"name": "flight restored enabled underglow before network resync", "node": "flight", "probe": "underglow_boot_restored_value", "minimum": 1, "maximum": 1},
            {"name": "power restored underglow from retained flash after reboot", "node": "power", "probe": "underglow_boot_restore_valid", "minimum": 1},
            {"name": "power restored enabled underglow before network resync", "node": "power", "probe": "underglow_boot_restored_value", "minimum": 1, "maximum": 1},
            {"name": "flight rejoined the network after reboot", "node": "flight", "probe": "network_ready", "minimum": 1, "sample": 4},
            {"name": "power rejoined the network after reboot", "node": "power", "probe": "network_ready", "minimum": 1, "sample": 4},
            *[
                {"name": f"{node} received GroundStation flight state", "node": node,
                 "probe": "flight_state_updates", "minimum": 1}
                for node, *_ in boards
            ],
            *[
                {"name": f"{node} converged on GroundStation flight state", "node": node,
                 "probe": "flight_state_cache", "minimum": 1, "maximum": 1}
                for node, *_ in boards
            ],
        ],
        "host_log_assertions": [
            {"name": "GroundStation discovered every board by autonomous name",
             "node": "groundstation",
             "contains": "AB,DAQ,FC,GB,PB,RF,VB"}
        ],
    }

    with tempfile.TemporaryDirectory(prefix="seds-firmware-network-") as directory:
        root = Path(directory)
        root.chmod(0o755)
        for node, layout in layouts.items():
            path = root / f"{node}.json"
            path.write_text(json.dumps(layout, indent=2), encoding="utf-8")
            path.chmod(0o644)
        (root / "topology.json").write_text(
            json.dumps(topology, indent=2), encoding="utf-8"
        )
        (root / "topology.json").chmod(0o644)
        command = [docker, "run", "--rm"]
        for node, path in roots.items():
            command += ["-v", f"{path}:/nodes/{node}:ro"]
        command += ["-v", f"{directory}:/simulation:ro", image, "bay", "--topology", "/simulation/topology.json"]
        ui.say("run", " ".join(command))
        run_live(command, "complete seven-board plus GroundStation network simulation")
