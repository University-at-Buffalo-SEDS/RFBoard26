# Radio UART receive recovery

A UART error could leave the receiver disabled while the scheduler and watchdog
continued running. The old interrupt callback attempted a single rearm. In
particular, HAL_UART_AbortReceive skips its DMA abort when CR3.DMAR is already
clear, even if the RX DMA handle is still busy. Repeated rearm attempts alone
cannot recover that state.

The radio service now retries recovery in thread context, at most once per
10 ms after a failure. It explicitly aborts a busy RX DMA handle before rearming.
It preserves the independent TX channel and leaves an armed, quiet receiver
alone. Errors raised during recovery remain pending for another service pass.
The g_radio_rx_recoveries counter exposes successful recovery attempts.

## Qualification — 2026-10-20

- All 162 native RF checks passed, including the actual recovery helper under
  address/undefined-behavior sanitizers and a HAL mock reproducing the
  DMAR-clear/busy-DMA failure.
- That regression failed before the explicit RX DMA abort was added.
- The Release factory image built with TLSF, heap packet storage and watchdog
  enabled, using SEDSnet main 781df378aad05be84f39811d49257eab8161134d.
- Jupiter's firmware simulator v0.4.14 booted the actual ARM image and factory
  image. All eight injected DMAR-clear failures recovered within each 100 ms
  observation window, with zero simulator watchdog resets.
- Minimum observed telemetry stack remaining was 8,104 bytes.
- Factory SHA256:
  761febf6bd2ecdfc42b6dd36e63f13b89b9e4da5c6ddd1e397aa5dc3a2df921b

The simulator result covers UART receive recovery, not over-air delivery or a
long-duration mixed-board network soak. Live radio verification is still needed.
UART configuration remains 115200 baud.

Build/flash with:

    python3 build.py --build-subdir RFReceiveRecovery flash --release --allocator tlsf --packet-store heap --method dfu
