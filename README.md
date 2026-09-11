# RFBoard26 firmware

RFBoard26 targets the STM32G491 and bridges the avionics CAN-FD network to the
RFD900x radio link. The two links are separate SEDSNet router sides; SEDSNet
discovery and learned subscriptions select routes, so the board does not
manually fan out application packets. The radio UART runs at 57600 baud. The
radio side relies on the RFD900x link's acknowledgement/retry mechanism;
SEDSNet hop acknowledgements remain disabled there to avoid nested retry
queues, stale managed-variable delivery, and application-traffic starvation.

Telemetry publication is configured at compile time with `RF_TELEMETRY_RATE_HZ`
in `Core/Inc/telemetry_rate.h` (default: 1 Hz). Set a whole-number rate from
1 to 1000 Hz and rebuild/reflash the board. Actual throughput is limited by
sensor acquisition and link capacity; this is not a network variable.

CMake fetches SEDSNet v4.0.27 and SEDS LaunchCore v1.0.0. Neither dependency is
a submodule. LaunchCore generates the linker scripts from
`Bootloader/board_config.h`, packages Slot A firmware, and reserves its approved
persistent-data and delta-update regions.

## Build and flash

```sh
./build.py build --release
./build.py flash --release --method dfu
```

The default flash operation writes the complete `.factory.bin` at
`0x08000000`; use an application-only option only on a board that already has a
compatible bootloader and metadata. Run `./build.py flash --help` for all
supported methods. `./build.py clean` removes generated build output.

OTA output uses the `.seds` extension. A delta is generated when a suitable
previous packaged image exists and fits; otherwise the package requests
LaunchCore full-image recovery.

## Configuration and validation

The board-owned network schema is `config/sedsnet.json`. `sim/board.json`
describes the STM32G491 memory map, GPS peripheral, and memory/network probes
used by FirmwareSimulator. The linked-system topology assembled by
`sim/run_full.py` models the avionics CAN-FD and RFD900x UART router sides.

```sh
./build.py test
./build.py test --all --release
./build.py test --all --release --ultra-soak
```

On Docker hosts that cannot create bridge interfaces (including the Jupiter
validation host), prefix the command with
`SEDS_FIRMWARE_SIM_DOCKER_NETWORK=host`. The linked test requires GroundStation
to label all seven graph nodes, attribute real payload traffic to each board,
and correlate a routed valve command with its returned state ACK.

`--ultra-soak` keeps the normal 16-second full-network test first, then adds a
separate 600,000 ms firmware-time fault/rejoin, command/ACK, and memory-leak
qualification. Commands must execute and return an ACK throughout the soak,
including its final interval.

The full suite builds firmware and OTA artifacts, checks flash/RAM and allocator
limits, boots the real ELF in the containerized simulator, injects traffic and
faults, and validates linked discovery, synchronization, managed variables,
and bidirectional routing.


## Regenerating with STM32CubeMX

Open the checked-in `.ioc` file and generate with the CMake toolchain. Keep user
code enabled. The `.ioc` is the source of truth for the ThreadX and USBX pool
sizes; unit tests compare those values with the generated Azure RTOS headers so
regeneration cannot silently shrink, grow, or repartition the pools.

The top-level CMake project is board-owned and reconnects generated STM32
sources with SEDSNet, LaunchCore, its generated linker scripts, persistence, and
the simulator probes. After generation, run
`python3 build.py test --full --release` before flashing or committing.
