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

## Extended and matching-main qualification

The original receive-recovery image also completed 600,000 ms of virtual ARM
execution on Jupiter. Network-loop completions advanced from 29,384 to 598,912,
telemetry stack remaining stayed at 8,104 bytes, and all eight injected recovery
checks passed afterward. No watchdog reset was observed. This was a standalone
RF run; the simulator's traffic-pool counters are behavioral and do not establish
over-air throughput or a full mixed-board network soak.

A second candidate, built in `RFDeliveryMain20261020`, uses SEDSnet main
`13c74abf7f8194299cb60b4b31f994b2eaadd5ef`. It includes the reliable-delivery
history fix and borrowed embedded metadata. Actual ARM and factory boot passed,
as did all eight injected receive-recovery checks (`renode-1ahF9g.log`).
Observed stack remaining reached 8,228 bytes; static RAM use is 114,304 bytes.
Factory SHA256:
`cdf89b73b1d4b895c05a4240568b1c798c3447a1b6b380a705cd457cc0d9d42b`.

This candidate still requires physical radio validation. GroundStation has
observed transport CRC failures and users report delayed controls; the checks
above do not demonstrate that those link symptoms are resolved. Read the actual
modem firmware and settings before applying register changes, because async and
peer-to-peer RFD firmware use different register maps.

Build/flash the matching-main candidate with:

    python3 build.py --build-subdir RFDeliveryMain20261020 flash --release --allocator tlsf --packet-store heap --method dfu
