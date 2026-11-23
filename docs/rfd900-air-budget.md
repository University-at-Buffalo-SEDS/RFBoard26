# RFD900x transmit budget

## What limits RF egress in the shipped profile

`config/rfd-profile.json` (`rfd-async-64k-drain`) keeps UART at 115200 baud
(ATS1=115) and the modem air rate at 64000 bit/s (ATS2=64), and sets
`turn_period_ms = 200`. With a turn period, **air turns are the only RF
egress limiter**: RF starts frames for 30% of each 200 ms period (a 60 ms
burst, at most 691 UART bytes, one DMA write of ordinary frames at most
516 B), then stays quiet so the station answers in the gap. Ordinary frames
that arrive after the UART has been idle 5 ms inside the burst wait for the
next turn; control frames (ACKs, ABORT acks, status) may still start while
the burst lasts. `radio_uart_quiet_remaining_ms()` returns the turn result
before the air-time and burst/gap terms (`Core/Src/radio.c`), and
`tests/test_radio_turn_cadence.py` pins both the turn cadence and that
precedence.

### The air-time limiter is deliberately not applied with turns

The per-frame air-time limiter (below) is still compiled and is used only
when a profile sets `turn_period_ms = 0`. It is not layered on top of the
turns, because that was evaluated in November 2026 and made the link worse.
A host model of the production constants (turn 200/60 ms, 5 ms idle close,
516 B ordinary writes, chained reservations without idle credit; 300 s of
AV-bay relay load of 60–190 B frames plus a 40 B control frame every
250 ms on average) gave:

| AV-bay load | Scheduler | Ordinary latency mean / p99 | Control latency p99 / max | Backlog after 300 s |
| --- | --- | --- | --- | --- |
| 1.0 KB/s | turns only (shipped) | 0.10 / 0.25 s | 143 / 146 ms | 0 |
| 1.0 KB/s | turns + limiter, 30000 bit/s | 2.5 / 6.2 s | 201 / 260 ms | 0 |
| 1.5 KB/s | turns only (shipped) | 0.12 / 0.33 s | 142 / 153 ms | 0 |
| 1.5 KB/s | turns + limiter, 30000 bit/s | 49 / 81 s | 201 / 215 ms | growing |
| 1.5 KB/s | turns + limiter, S5 = 10 readback (5469 bit/s) | 211 / 310 s | 2.4 / 3.0 s | growing (413 B/s delivered) |
| 2.5 KB/s | turns only (shipped) | 0.33 / 1.0 s | 144 / 165 ms | 0 |
| 2.5 KB/s | turns + limiter, 30000 bit/s | 112 / 182 s | 200 / 297 ms | growing (1.7 KB/s delivered) |

Why: the limiter's reservation after a write (about 140 ms for a 516 B
write) is longer than the 5 ms idle close, so each turn carries one write,
and a turn opened by a small control frame carries nothing else. The AV bay
produces about 1.4–1.5 KB/s, so the queue grows without bound, which is the
same failure the 15% burst showed in the faithful fullnet (see the comment
at `RADIO_TURN_BURST_MS`). It also delays commands and ABORT acks by up to
one reservation. Fairness does not need it: the turn already bounds RF to
about 90 ms of air per 200 ms (691 B at 64000 bit/s), leaving the station
its gap, and RRLS relay timing is set by the station side, which the RF
limiter does not affect. When the modem check reads S5 above the profile,
the S5 correction would cut the pacing rate to about 5.5 kbit/s and delivery
to about 0.4 KB/s. Turn shares are unchanged by this decision.

## Air-time limiter (profiles without turns)

With `turn_period_ms = 0` RF paces its UART for the shared channel:
`quiet = ceil((len + 16) x 8000 / air_bps)`, with `air_bps =
usable_air_bps x rf_share_percent / 100` (profile: 30000 x 100%), lowered to
`64000 x 0.94 / (1 + S5)` when the modem readback reports more broadcast
copies than the profile. Reservations chain from actual UART admission; idle
time does not accumulate credit, and a completed UART DMA transfer alone does
not grant the next transmission. Air heard from other modems is charged to
RF's pacing, up to 60%. RF checks the next-send deadline before selecting a
queued frame and returns without sleeping, so CAN and radio RX keep
progressing. `config/rfd-profile.json` supplies the CMake values on every
configure. Explicit custom profiles must be checked across every participant
before deployment.

Primary and RRLS hosts share each physical UART capacity state with SDK traffic;
reconnecting does not create another allowance. A relay may use an expanded
recovery allocation only with a nonce-bound peer grant and a matching RF
application/UART profile proof. Unrecognized or legacy RF40 cannot certify
that expansion. The profile response reports compiled application/UART settings,
not live modem AT configuration. The fill-system link remains Ethernet.

This backpressure does not enable LoRa scheduling, AUX/GRANT handshakes or
receive windows (`RFBOARD_LORA_SCHEDULER=OFF`).

Read `ATI` and `ATI5` before applying a profile. Parameter numbers differ
between SiK/peer-to-peer and ASYNC firmware revisions; use the names printed
by the installed modem. The captured **RFD ASYNC 4.03 / RFD900X2** reports:

| Parameter | Captured register | Meaning |
| --- | --- | --- |
| SERIAL_SPEED | S1 | UART rate (115 means 115200 baud) |
| AIR_SPEED | S2 | Air rate (64 means64000 bit/s) |
| MAX_RETRIES | S4 | Per-message retry limit, **not transmit power** |
| GLOBAL_RETRIES | S5 | Broadcast retry limit |
| TX_ENCAP_METHOD / RX_ENCAP_METHOD | S7 / S8 | Serial encapsulation |
| TXPOWER | S12 | Transmit power in dBm |
| MIN_FREQ | S14 | Minimum frequency, **not a latency window** |
| RTSCTS | S18 | Hardware flow control |

The older [RFDesign ASYNC manual](https://files.rfdesign.com.au/Files/documents/RFD900x%20Asynchronous%20User%20Manual%20V1.1.pdf)
also identifies S4 as retries, S12 as power, and flow control at S18.
The October 28 RRLS readback queried S18 and recorded flow control disabled.
That readback did not query S19 or establish a different register layout.
Do not copy register numbers from a different firmware table. In particular,
the earlier SiK-style `ATS4=30` / `ATS14=120` recipe does not configure 30 dBm /
a 120 ms window on this ASYNC firmware.

Pacing is an application budget, not a measured RF capacity guarantee. Check
both directions and discovery after restarting RF; real antenna loss, modem
retries and contention require field validation beyond the UART simulator.

## Bound packetization delay as well as average airtime

A small, continuous stream can stay below the byte budget while never leaving
an idle boundary for a transparent modem's packetizer. The production host
capacity gate produced 42-byte writes every 49 ms in the fallback share. With
a 50 ms serial-idle envelope and MAX_DATA 3968, the actual simulator modem
model held that stream for up to 5.11 seconds. The older 2 ms / 256-byte test
profile delivered the same writes in 17.2 ms and concealed this failure mode.
The active modems' packetization settings have not yet been read back; this is
a reproduced failure mechanism, not confirmation of their complete firmware
algorithm or configuration.

Without turns, RF and the GS/RRLS UART retain the shared-air reservation and
also bound a batch of writes (`RADIO_PACKET_BURST_MS`). Once that interval has
elapsed, they leave `RADIO_PACKET_GAP_MS` of serial silence after the last
UART stop bit before admitting another batch. The profile and the C defaults
are 250 ms / 15 ms; the original values were 100 ms / 55 ms. With turns the
turn's 5 ms idle close takes this role. Keeping a short batch preserves throughput for telemetry and discovery;
adding a gap after every frame failed an existing reliable-delivery test. RX
continues running throughout. This adds no AUX, GRANT or receive window, and
does not extend recovery leases.

The host uses the configured UART baud and anchors the gap to actual drain
completion. RF anchors it to its DMA completion callback. Both retain existing
frame ownership, queue priority, airtime debt, and tick-wrap handling. On RF
the compile definitions `RADIO_PACKET_BURST_MS` and `RADIO_PACKET_GAP_MS` come
from the profile (250 / 15). Match any explicit override to verified modem
settings; zero gap is retained for controlled negative tests or a modem whose
packetization has independently been verified not to need it.

Default builds explicitly pass `RFBOARD_LORA_SCHEDULER=OFF` to CMake, including
when reusing a build directory that previously selected LoRa. Select
`python3 build.py --radio-mode lora` only for the legacy LoRa hardware. The
GS/RRLS production worker uses independent RX and TX handles and does not
consult the legacy uplink grants or emit scheduler handshakes. A retained
serial framing boundary is not an uplink slot.

The same long-packet fixture now measures 173.7 ms with all 192 transport probes
delivered in order and no FIFO loss. A permanent negative control disables only
the boundary and reproduces 5.11 seconds. The fixture also runs against Renode's
actual virtual clock. These raw transport checks complement the real Mesh/SDK
reliable-command campaign; they do not replace physical command/receipt tests.

## Startup modem check

Once per boot the board checks its own RFD900x against
`config/rfd-profile.json` and repairs local drift (S6, S11, S13, RTSCTS),
matching registers by the names the modem reports in `ATI5`. It runs when the
first FLIGHT_STATE received after boot is Startup, Idle, PreFill or FillTest,
never after a restored flight phase, and holds the radio only for the check
itself (about 3 s, about 11 s if it writes, at most 40 s). Link-wide registers
are written only with `--rfd-profile-rolled-out`. Select the mode with
`build.py build --rfd-modem-check default|skip|verify|fix` (default: fix;
skip for `SEDS_FIRMWARE_SIM_TEST=1`). The result is in BOARD_HEALTH words 13,
15 and 22 and is shown in the ground station's Connectivity → Radio modem
card. Details: GroundStation26 `docs/backend/rfd-modem-check.md`, phase 2.
