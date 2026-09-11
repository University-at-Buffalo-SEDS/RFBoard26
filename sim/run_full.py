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
SIMULATOR_DOCKER_PLATFORM = os.environ.get(
    "SEDS_FIRMWARE_SIM_PLATFORM", "linux/amd64"
)


def docker_run_prefix(docker: str) -> list[str]:
    command = [docker, "run", "--platform", SIMULATOR_DOCKER_PLATFORM, "--rm"]
    network = os.environ.get("SEDS_FIRMWARE_SIM_DOCKER_NETWORK")
    if network:
        command.extend(["--network", network])
    return command


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
        "--platform", SIMULATOR_DOCKER_PLATFORM,
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

    ui.say("run", f"{docker} pull --platform {SIMULATOR_DOCKER_PLATFORM} {requested}")
    # Always refresh mutable tags such as latest. Inherit terminal streams so
    # layer downloads and extraction remain visible instead of looking hung.
    pull = subprocess.run(
        [docker, "pull", "--platform", SIMULATOR_DOCKER_PLATFORM, requested]
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
            *docker_run_prefix(docker),
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
    layout["execution"]["memory_probe_warmup_samples"] = min(
        19,
        max(3, int(layout["execution"].get("memory_probe_warmup_samples", 0))),
    )
    # Instruction tracing retains every executed PC and can consume gigabytes
    # while an isolated controller repeatedly retries CAN. Register and memory
    # probes still provide the required fault diagnostics for this soak.
    layout["execution"]["trace"] = False

    with tempfile.TemporaryDirectory(prefix="seds-firmware-profile-") as directory:
        write_container_layout(Path(directory), layout)
        command = [
            *docker_run_prefix(docker),
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "profile",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
            # Keep real instruction execution long enough for board startup.
            # Behavioral packet counts do not substitute for firmware runtime;
            # the separate linked ten-minute soak checks sustained operation.
            "--virtual-time-ms", str(max(1000, layout["execution"]["virtual_time_ms"])),
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
    # The first three samples cover reset and CAN error-counter propagation.
    # HAL completion semantics differ by STM32 family: some drivers report a
    # successful mailbox enqueue even when no peer ACKs it. Require continued
    # TX progress and fatal-fault counters instead of a family-specific error.
    layout["execution"]["memory_probe_warmup_samples"] = min(
        4,
        max(3, int(layout["execution"].get("memory_probe_warmup_samples", 0))),
    )
    # An isolated node retries quickly. Keep register and memory probes, but do
    # not retain an unbounded instruction trace in the simulator process.
    layout["execution"]["trace"] = False
    for probe in layout["execution"]["memory_probes"]:
        if probe.get("name") == "fdcan_tx_fail":
            probe.pop("maximum", None)
            probe.pop("minimum", None)
        if probe.get("name") == "fdcan_tx_ok":
            probe["minimum"] = 1

    with tempfile.TemporaryDirectory(prefix="seds-firmware-isolated-can-") as directory:
        write_container_layout(Path(directory), layout)
        command = [
            *docker_run_prefix(docker),
            "-v", f"{repo_root}:/firmware:ro",
            "-v", f"{directory}:/simulation:ro",
            image, "profile",
            "--layout", "/simulation/board.json",
            "--firmware-root", "/firmware",
            "--can-unacknowledged",
            "--virtual-time-ms", str(max(1000, layout["execution"]["virtual_time_ms"])),
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
    ui,
    repo_root: Path,
    architecture: str,
    build_subdir: str | None = None,
    *,
    ultra_soak: bool = False,
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
        ("gateway", "gateway-board26", "gateway_board", 8, "fdcan2"),
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
        # The first observations cover reset and ThreadX startup. Network
        # assertions below still require the post-warmup values and traffic.
        layout["execution"]["memory_probe_warmup_samples"] = max(
            2, int(layout["execution"].get("memory_probe_warmup_samples", 0))
        )
        if node in {"rf", "power", "flight"}:
            # These counters intentionally restart at zero after the retained-
            # flash reboot. Their explicit bay assertions below use the run's
            # maximum to prove service happened before and after persistence.
            for probe in layout["execution"]["memory_probes"]:
                if probe["name"] in {
                    "can_rx_service_completions",
                    "telemetry_loop_completions",
                }:
                    probe.pop("minimum", None)
        layouts[node] = layout

    # Reproduce the minimum real avionics topology that exposed the RF relay
    # stall: RF + Flight Computer on CAN, the GroundStation on the RFD900x,
    # and no powered Power Board.  The complete-bay test below can hide this
    # failure because traffic from a third CAN producer keeps aggregate link
    # counters moving even when the RF service loop stops making progress.
    power_absent_topology = {
        "name": "rf-flight-groundstation-power-absent",
        "quantum_seconds": 0.001,
        "virtual_time_ms": int(
            os.environ.get("SEDS_FIRMWARE_SIM_AVIONICS_TIME_MS", "16000")
        ),
        "sample_count": 6,
        "enforce_end_drop": True,
        "nodes": [
            {"name": node, "layout": f"/simulation/{node}.json",
             "firmware_root": f"/nodes/{node}"}
            for node in ("rf", "flight")
        ],
        "host_nodes": [{
            "name": "groundstation",
            "binary": "/usr/local/bin/groundstation_backend",
            "cwd": "/opt/groundstation/backend",
            "env": {
                "GS_DEBUG_PRINTS": "0",
                "RUST_LOG": "info",
                "GS_SIMULATED_SERIAL_PTY": "1",
                "GS_LAYOUT_PATH": "/opt/groundstation/backend/layout/layout_hitl.json",
                "GS_SIM_EXPECT_DISCOVERY_NODES": "RF,FC",
                "GS_SIM_VALIDATE_FLIGHT_COMMAND": "1",
                "GS_SIM_VALIDATE_TELEMETRY_RETURN": "1",
                "GS_HEARTBEAT_INTERVAL_MS": "1000",
                "GS_SIM_ROUTER_TIME_DIVISOR": "8",
                "GS_SIM_COMPACT_INITIAL_DISCOVERY": "1",
                "GS_SIM_CONTROL_STEP_MS": "250",
            },
            "serial_links": [
                {"link": "rocket_radio", "env": "GS_AV_BAY_SERIAL_PORT"}
            ],
        }],
        "links": [
            {"name": "avionics_can", "kind": "can",
             "transport_path": ["RFBoard", "FlightComputer"],
             "endpoints": [
                 {"node": "rf", "peripheral": "fdcan2",
                  "tx_probe": "fdcan_tx_ok", "rx_probe": "fdcan_rx"},
                 {"node": "flight", "peripheral": "fdcan1",
                  "tx_probe": "fdcan_tx_ok", "rx_probe": "fdcan_rx"},
             ]},
            {"name": "rocket_radio", "kind": "radio",
             "transport_path": ["RF RFD900x radio", "GroundStation26 host binary"],
             "endpoints": [
                 {"node": "rf", "peripheral": "usart1",
                  "tx_probe": "radio_tx_frames", "rx_probe": "radio_rx_frames"},
                 {"node": "groundstation", "peripheral": "av_bay"},
             ]},
        ],
        "assertions": [
            {"name": "RF telemetry loop remained live after FC traffic joined",
             "node": "rf", "probe": "telemetry_loop_completions",
             "minimum_gain": 1, "from_sample": 3, "to_sample": 5},
            {"name": "RF continued receiving CAN late in the run",
             "node": "rf", "probe": "fdcan_rx",
             "minimum_gain": 1, "from_sample": 3, "to_sample": 5},
            {"name": "RF continued transmitting radio late in the run",
             "node": "rf", "probe": "radio_tx_frames",
             "minimum_gain": 1, "from_sample": 3, "to_sample": 5},
            {"name": "Flight Computer received the routed command",
             "node": "flight", "probe": "network_flight_commands_received",
             "minimum": 1},
            {"name": "Flight Computer executed the routed command",
             "node": "flight", "probe": "network_flight_commands_processed",
             "minimum": 1},
        ],
        "host_log_assertions": [
            {"name": "GroundStation preference survives discovery and process restart",
             "node": "groundstation",
             "contains": "full-bay preferred discovery master GS verified after named discovery",
             "minimum_occurrences": 2 if perform_reboots else 1},
            {"name": "GroundStation discovered RF and Flight Computer",
             "node": "groundstation", "contains": "FC,RF"},
            {"name": "RF GPS crossed the return path",
             "node": "groundstation",
             "contains": "full-bay RF GPS 1 Hz stream reached GroundStation"},
            {"name": "Flight telemetry crossed the return path",
             "node": "groundstation",
             "contains": "full-bay Flight sensor 1 Hz stream reached GroundStation"},
        ],
    }

    with tempfile.TemporaryDirectory(prefix="seds-firmware-avionics-") as directory:
        root = Path(directory)
        root.chmod(0o755)
        for node in ("rf", "flight"):
            path = root / f"{node}.json"
            path.write_text(json.dumps(layouts[node], indent=2), encoding="utf-8")
            path.chmod(0o644)
        topology_path = root / "topology.json"
        topology_path.write_text(
            json.dumps(power_absent_topology, indent=2), encoding="utf-8"
        )
        topology_path.chmod(0o644)
        command = docker_run_prefix(docker)
        for node in ("rf", "flight"):
            command += ["-v", f"{roots[node]}:/nodes/{node}:ro"]
        command += [
            "-v", f"{directory}:/simulation:ro", image, "bay",
            "--topology", "/simulation/topology.json",
        ]
        ui.say("run", " ".join(command))
        run_live(command, "RF + Flight Computer power-absent liveness simulation")

    skip_reboots = os.environ.get("SEDS_FIRMWARE_SIM_SKIP_REBOOTS") == "1"
    perform_reboots = ultra_soak and not skip_reboots
    virtual_time_ms = (
        int(os.environ.get("SEDS_FIRMWARE_SIM_SOAK_MS", "600000"))
        if ultra_soak
        else int(os.environ.get("SEDS_FIRMWARE_SIM_NETWORK_TIME_MS", "16000"))
    )
    if ultra_soak and virtual_time_ms < 10000:
        raise RuntimeError("SEDS_FIRMWARE_SIM_SOAK_MS must be at least 10000")
    sample_count = (
        12
        if ultra_soak
        else int(os.environ.get("SEDS_FIRMWARE_SIM_NETWORK_SAMPLES", "6"))
    )
    # Leave enough constrained-link time for discovery and the complete
    # managed-variable sequence before exercising retained-flash restart.
    # The ten-minute qualification still observes a full 200 seconds after
    # reboot, so post-restart stalls cannot hide behind pre-reboot traffic.
    reboot_after_sample = (sample_count * 2) // 3
    soak_command_samples = [
        sample for sample in range(1, sample_count)
        # RF CAN acknowledgement is deliberately disabled after sample 3 and
        # restored after sample 4. Commands resume immediately on recovery.
        if sample != 3
    ]
    topology = {
        "name": "complete-seds-avionics-and-fill-network",
        # One millisecond is below the firmware service cadence while avoiding
        # the 10x synchronization overhead of a 100 us multi-machine quantum.
        "quantum_seconds": 0.001,
        # The linked test proves connectivity; each board's separate profile
        # stage performs the long-duration allocator qualification.
        # The Pico-Fi/radio path deliberately models constrained serial links.
        # Leave enough virtual time for the open command and its status ACK to
        # traverse both directions across the constrained serial links.
        # Full autonomous-name discovery crosses CAN, RF, GroundStation,
        # Pico-Fi and CAN before validation controls are emitted. Keep a
        # distinct post-control window so those values are executed and
        # probed before the retained-flash reboot exercise begins.
        "virtual_time_ms": virtual_time_ms,
        "sample_count": sample_count,
        "enforce_end_drop": ultra_soak,
        # Reboot only after discovery and the complete 1-0-1 control sequence
        # have crossed the routed network. Renode retains physical flash
        # across this reset, matching a real power cycle while peers stay up.
        "reboots": [
            {"node": "rf", "after_sample": reboot_after_sample},
            {"node": "power", "after_sample": reboot_after_sample},
            {"node": "flight", "after_sample": reboot_after_sample},
            {"node": "groundstation", "after_sample": reboot_after_sample},
        ] if perform_reboots else [],
        "can_ack_events": (
            [
                {"node": "rf", "peripheral": "fdcan2", "after_sample": 3,
                 "acknowledged": False},
                {"node": "rf", "peripheral": "fdcan2", "after_sample": 4,
                 "acknowledged": True},
            ]
            if ultra_soak
            else []
        ),
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
                    "GS_SIM_VALIDATE_FLIGHT_COMMAND": "1",
                    "GS_SIM_VALIDATE_TELEMETRY_RETURN": "1",
                    # During the optional ten-minute soak, issue one routed
                    # Valve command after every completed sample. The next
                    # sample must observe both board execution and the ACK at
                    # GroundStation, including the final 9:10-10:00 window.
                    "GS_SIM_VALIDATE_SOAK_COMMANDS": "1" if ultra_soak else "0",
                    "GS_SIM_SOAK_COMMAND_SAMPLES": ",".join(
                        str(sample) for sample in soak_command_samples
                    ),
                    # The simulator divides GroundStation router time by eight
                    # to match seven synchronized Renode machines. Keep the
                    # resulting wire heartbeat safely below Valve's 5 s
                    # virtual watchdog deadline.
                    "GS_HEARTBEAT_INTERVAL_MS": "1000",
                    "GS_SIM_ROUTER_TIME_DIVISOR": "8",
                    "GS_SIM_COMPACT_INITIAL_DISCOVERY": "1",
                    "GS_SIM_EXPECT_DISCOVERY_NODES": "RF,PB,FC,GB,AB,VB,DAQ",
                    # Hold each state long enough for delivery and persistence.
                    # Unsynchronized firmware retries at 500 ms without blocking
                    # its router loop; seven emulated MCUs advance slowly.
                    "GS_SIM_CONTROL_STEP_MS": "250",
                    "GS_SIM_VALVE_ROUTE_SETTLE_MS": "1000",
                    # Bounds are measured using GroundStation's simulator-
                    # normalized router clock, not slow host wall time.
                    "GS_SIM_DISCOVERY_MAX_LATENCY_MS": "10000",
                    "GS_SIM_MANAGED_VARIABLE_MAX_LATENCY_MS": "2500",
                    "GS_SIM_VALVE_ACK_MAX_LATENCY_MS": "2500",
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
             "transport_path": ["RF RFD900x radio", "GroundStation26 host binary"],
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
            {"name": "Gateway received status ACK over CAN", "node": "gateway", "probe": "can_umbilical_status_count", "minimum": 1},
            {"name": "Gateway received pilot-open status", "node": "gateway", "probe": "gateway_pilot_open_status", "minimum": 1},
            {"name": "Gateway routed status ACK toward GroundStation", "node": "gateway", "probe": "uart_umbilical_status_tx_count", "minimum": 1},
            {"name": "Gateway transmitted status ACK on Pico-Fi UART", "node": "gateway", "probe": "uart_umbilical_status_count", "minimum": 1},
            {"name": "rf applied the 1-0-1 GroundStation underglow sequence", "node": "rf", "probe": "underglow_updates", "minimum": 3},
            {"name": "power applied the 1-0-1 GroundStation underglow sequence", "node": "power", "probe": "underglow_updates", "minimum": 3},
            {"name": "flight applied the 1-0-1 GroundStation underglow sequence", "node": "flight", "probe": "underglow_updates", "minimum": 3},
            {"name": "rf persisted the final underglow state", "node": "rf", "probe": "underglow_persist_writes", "minimum": 1},
            {"name": "power persisted the final underglow state", "node": "power", "probe": "underglow_persist_writes", "minimum": 1},
            {"name": "flight persisted the final underglow state", "node": "flight", "probe": "underglow_persist_writes", "minimum": 1},
            {"name": "rf restored underglow from retained flash after reset", "node": "rf", "probe": "underglow_persist_restores", "minimum": 1},
            {"name": "power restored underglow from retained flash after reset", "node": "power", "probe": "underglow_persist_restores", "minimum": 1},
            {"name": "flight restored underglow from retained flash after reset", "node": "flight", "probe": "underglow_persist_restores", "minimum": 1},
            {"name": "rf persistence remained healthy", "node": "rf", "probe": "underglow_persist_errors", "maximum": 0},
            {"name": "power persistence remained healthy", "node": "power", "probe": "underglow_persist_errors", "maximum": 0},
            {"name": "flight persistence remained healthy", "node": "flight", "probe": "underglow_persist_errors", "maximum": 0},
            {"name": "flight applied GroundStation buzzer variable", "node": "flight", "probe": "flight_buzzer_updates", "minimum": 3},
            {"name": "flight persisted the final buzzer state", "node": "flight", "probe": "flight_buzzer_persist_writes", "minimum": 1},
            {"name": "flight restored buzzer state from retained flash after reset", "node": "flight", "probe": "flight_buzzer_persist_restores", "minimum": 1},
            {"name": "flight buzzer persistence remained healthy", "node": "flight", "probe": "flight_buzzer_persist_errors", "maximum": 0},
            {"name": "flight restored buzzer before network resync", "node": "flight", "probe": "flight_buzzer_boot_restore_valid", "minimum": 1, "maximum": 1},
            {"name": "flight restored enabled buzzer before network resync", "node": "flight", "probe": "flight_buzzer_boot_restored_value", "minimum": 1, "maximum": 1},
            {"name": "flight buzzer finished enabled", "node": "flight", "probe": "flight_buzzer_enabled", "minimum": 1},
            {"name": "flight performed the configured startup buzz", "node": "flight", "probe": "flight_buzzer_startup_buzzes", "minimum": 1},
            {"name": "flight stopped the startup buzz after its deadline", "node": "flight", "probe": "flight_buzzer_startup_completions", "minimum": 1},
            {"name": "Flight Computer received a routed command", "node": "flight", "probe": "network_flight_commands_received", "minimum": 1},
            {"name": "Flight Computer accepted the routed command", "node": "flight", "probe": "network_flight_commands_accepted", "minimum": 1},
            {"name": "Flight Computer recovery task dequeued commands", "node": "flight", "probe": "recovery_commands_dequeued", "minimum": 1},
            {"name": "Flight Computer recovery task executed the routed command", "node": "flight", "probe": "network_flight_commands_processed", "minimum": 1},
            {"name": "Flight Computer received the expected command id", "node": "flight", "probe": "last_network_flight_command_id", "minimum": 14, "maximum": 14},
            *(
                [
                    {
                        "name": f"{node} CAN transmit resumed after retained-flash reboot",
                        "node": node,
                        "probe": "fdcan_tx_ok",
                        "minimum_gain": 1,
                        "from_sample": reboot_after_sample,
                        "to_sample": sample_count - 1,
                    }
                    for node in ("rf", "power", "flight")
                ]
                if perform_reboots
                else []
            ),
            {"name": "rf underglow is enabled", "node": "rf", "probe": "underglow_enabled", "minimum": 1},
            {"name": "power underglow is enabled", "node": "power", "probe": "underglow_enabled", "minimum": 1},
            {"name": "flight underglow is enabled", "node": "flight", "probe": "underglow_enabled", "minimum": 1},
            {"name": "power restored underglow before network resync", "node": "power", "probe": "underglow_boot_restore_valid", "minimum": 1},
            {"name": "power restored the enabled value before network resync", "node": "power", "probe": "underglow_boot_restored_value", "minimum": 1, "maximum": 1},
            {"name": "flight restored underglow before network resync", "node": "flight", "probe": "underglow_boot_restore_valid", "minimum": 1},
            {"name": "flight restored the enabled value before network resync", "node": "flight", "probe": "underglow_boot_restored_value", "minimum": 1, "maximum": 1},
            {"name": "RF maintained valid source time", "node": "rf", "probe": "timesync_valid", "minimum": 1},
            {"name": "Power synchronized network time", "node": "power", "probe": "timesync_valid", "minimum": 1},
            {"name": "Flight synchronized network time", "node": "flight", "probe": "timesync_valid", "minimum": 1},
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
            *[
                {"name": f"{node} restored flight state from retained flash after reset", "node": node,
                 "probe": "flight_state_restores", "minimum": 1}
                for node in ("rf", "power", "flight")
            ],
            *(
                [
                    {
                        "name": f"{node} CAN transmit still advances late in ten-minute soak",
                        "node": node,
                        "probe": "fdcan_tx_ok",
                        "minimum_gain": 1,
                        "from_sample": sample_count - 3,
                        "to_sample": sample_count - 1,
                    }
                    for node, *_ in boards
                ]
                + [
                    {
                        "name": f"{node} CAN receive still advances late in ten-minute soak",
                        "node": node,
                        "probe": "fdcan_rx",
                        "minimum_gain": 1,
                        "from_sample": sample_count - 3,
                        "to_sample": sample_count - 1,
                    }
                    for node, *_ in boards
                ]
                + [
                    {
                        "name": f"Valve command path remained alive during soak interval {sample + 1}",
                        "node": "valve",
                        "probe": "valve_commands_executed",
                        "minimum_gain": 1,
                        "from_sample": sample,
                        "to_sample": sample + 1,
                    }
                    for sample in (value - 1 for value in soak_command_samples)
                ]
                if ultra_soak
                else []
            ),
        ],
        "host_log_assertions": [
            {"name": "GroundStation discovered every board by autonomous name",
             "node": "groundstation",
             "contains": "AB,DAQ,FC,GB,PB,RF,VB",
             "minimum_occurrences": 2 if perform_reboots else 1},
            {"name": "GroundStation attributed traffic to every board identity",
             "node": "groundstation",
             "contains": "full-bay per-board traffic attribution ready:",
             "minimum_occurrences": 2 if perform_reboots else 1},
            {"name": "GroundStation network graph labelled every board with its own traffic",
             "node": "groundstation",
             "contains": "full-bay network graph attribution ready:",
             "minimum_occurrences": 2 if perform_reboots else 1},
            {"name": "Valve acknowledgement completed the routed return path",
             "node": "groundstation",
             "contains": "full-bay valve ACK reached GroundStation"},
            *(
                [{"name": "Every ten-minute soak command returned an acknowledgement",
                  "node": "groundstation",
                  "contains": "full-bay soak valve command acknowledged",
                  "minimum_occurrences": len(soak_command_samples)}]
                if ultra_soak
                else []
            ),
            {"name": "Discovery completed within its latency bound",
             "node": "groundstation",
             "contains": "full-bay discovery latency within bound"},
            {"name": "Managed variables completed within their latency bound",
             "node": "groundstation",
             "contains": "full-bay managed-variable latency within bound"},
            {"name": "Valve command acknowledgement met its latency bound",
             "node": "groundstation",
             "contains": "full-bay valve ACK latency within bound"},
            {"name": "GroundStation routed a Flight Computer command",
             "node": "groundstation",
             "contains": "full-bay Flight Computer command queued: 14"},
            {"name": "RF GPS telemetry completed the return path",
             "node": "groundstation",
             "contains": "full-bay RF GPS 1 Hz stream reached GroundStation"},
            {"name": "Flight sensor telemetry completed the return path",
             "node": "groundstation",
             "contains": "full-bay Flight sensor 1 Hz stream reached GroundStation"},
            {"name": (
                 "Power telemetry maintained its five-second cadence"
                 if ultra_soak
                 else "Power telemetry completed the return path"
             ),
             "node": "groundstation",
             "contains": (
                 "full-bay Power 5-second stream reached GroundStation"
                 if ultra_soak
                 else "full-bay Power telemetry reached GroundStation"
             )},
            *[
                {"name": f"GroundStation observed {node} fill-system telemetry",
                 "node": "groundstation",
                 "contains": f"full-bay fill telemetry reached GroundStation from {node}"}
                for node in ("GB", "AB", "VB", "DAQ")
            ],
            {"name": "GroundStation received the DAQ calibrated 50 Hz loadcell stream",
             "node": "groundstation",
             "contains": "full-bay DAQ calibrated loadcell 50 Hz stream reached GroundStation"},
        ],
    }

    if not perform_reboots:
        # The 16-second gate covers boot and connected routing. Retained-flash
        # restart belongs to the representative 600-second soak, which leaves
        # a full 200 seconds to prove post-reboot recovery.
        topology["assertions"] = [
            assertion for assertion in topology["assertions"]
            if "restored" not in assertion["name"].lower()
            and "startup buzz" not in assertion["name"].lower()
        ]

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
        command = [
            *docker_run_prefix(docker)
        ]
        for node, path in roots.items():
            command += ["-v", f"{path}:/nodes/{node}:ro"]
        command += ["-v", f"{directory}:/simulation:ro", image, "bay", "--topology", "/simulation/topology.json"]
        ui.say("run", " ".join(command))
        description = (
            "ten-minute seven-board network soak"
            if ultra_soak
            else "complete seven-board plus GroundStation network simulation"
        )
        run_live(command, description)
