# RFD900x transmit budget

The compatible profile keeps UART at 115200 baud (ATS1=115), modem air rate at
64000 bit/s (ATS2=64), and the usable application envelope at 32000 bit/s.
RF reserves 35% of that envelope (1400 charged bytes/s). The four-byte UART
header is included in the frame, with a separate 16-byte application overhead.
A 100-byte payload therefore reserves 86 ms; a 1024-byte payload reserves
746 ms. These are application reservations, not measured RF delivery times.

RF checks the next-send deadline before selecting a queued frame and returns
without sleeping, so CAN and radio RX keep progressing. Command priority,
100 ms collection bursts, and 55 ms packetization gaps remain unchanged.
`config/rfd-profile.json` supplies the actual CMake values on every configure;
the CMake and script fallback share is also 35%. Explicit custom profiles must
be checked across every participant before deployment.

Primary and RRLS hosts share each physical UART capacity state with SDK traffic;
reconnecting does not create another allowance. A relay may use an expanded
recovery allocation only with a nonce-bound peer grant and a matching RF
application/UART profile proof. Unrecognized or legacy RF40 cannot certify
that expansion. The profile response reports compiled application/UART settings,
not live modem AT configuration. The fill-system link remains Ethernet.

This backpressure is active with `RFBOARD_LORA_SCHEDULER=OFF`. It keeps traffic
in the priority queue until the previous reservation has drained. It does not
enable LoRa turn scheduling, AUX/GRANT handshakes or receive windows. Idle time
does not accumulate credit for a later burst. A completed UART DMA transfer
alone does not grant the next radio transmission.

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

RF and the GS/RRLS UART now retain the original shared-air reservation and also
bound a batch of writes to 100 ms. Once that interval has elapsed, they leave
55 ms of serial silence after the last UART stop bit before admitting another
batch. Keeping a short batch preserves throughput for telemetry and discovery;
adding a gap after every frame failed an existing reliable-delivery test. RX
continues running throughout. This adds no AUX, radio turn, GRANT or receive
window, and does not extend recovery leases.

The host uses the configured UART baud and anchors the gap to actual drain
completion. RF anchors it to its DMA completion callback. Both retain existing
frame ownership, queue priority, airtime debt, and tick-wrap handling. Defaults
are `GS_RADIO_PACKET_BURST_MS=100` and `GS_RADIO_PACKET_GAP_MS=55` on GS/RRLS,
and compile definitions `RADIO_PACKET_BURST_MS=100` and
`RADIO_PACKET_GAP_MS=55` on RF. Match any explicit override to verified modem
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
