# RFBoard26 firmware

RFBoard26 targets the STM32G491 and bridges the avionics CAN-FD network to the
RFD900x radio link. The two links are separate SEDSNet router sides; SEDSNet
discovery and learned subscriptions select routes, so the board does not
manually fan out application packets. The radio UART runs at 57600 baud.

CMake fetches SEDSNet v4.0.18 and SEDS LaunchCore v1.0.0. Neither dependency is
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
describes the STM32G491 memory map, both network sides, UART, CAN, USB CDC, and
memory probes used by FirmwareSimulator.

```sh
./build.py test
./build.py test --all --release
```

The full suite builds firmware and OTA artifacts, checks flash/RAM and allocator
limits, boots the real ELF in the containerized simulator, injects traffic and
faults, and validates linked discovery, synchronization, managed variables,
and bidirectional routing.
