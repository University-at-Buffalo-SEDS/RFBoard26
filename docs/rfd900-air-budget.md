# RFD900x transmit budget

UART remains 115200 baud (ATS1=115); the confirmed air rate is 64000 bit/s
(ATS2=64). DMA completion drains UART bytes, not the modem's radio FIFO. With
RTS/CTS disabled, limit each end to 40% of the shared air rate. The other 20%
is headroom for modem overhead and retries, not a measured capacity guarantee.

RF checks the next-send deadline before selecting the next queued frame. It
returns without sleeping, so CAN and radio RX keep progressing; commands retain
queue priority. Spacing uses eight bits per air byte, the four-byte UART frame
header already in the frame, and a configurable 16-byte overhead estimate. A
100-byte payload reserves 38 ms; a 1024-byte payload reserves 327 ms. There is
no additional fixed 25 ms delay.

GroundStation's rocket_comms worker applies the same budget with defaults
GS_RADIO_AIR_BIT_RATE_BPS=64000, GS_RADIO_AIR_SHARE_PERCENT=40 and
GS_RADIO_AIR_FRAME_OVERHEAD_BYTES=16. The fill-system link uses the Pi's main Ethernet interface and is not radio-paced. RF build
definitions RADIO_AIR_BIT_RATE_BPS, RADIO_AIR_SHARE_PERCENT and
RADIO_AIR_FRAME_OVERHEAD_BYTES override the matching firmware values.

When GroundStation or RRLS is configured with `GS_RRLS_ROLE=primary` or
`GS_RRLS_ROLE=rrls`, the host default share is 25%, reserving room for the third
radio. `GS_RRLS_VOICE=1` selects the same default. An explicit share override
still applies. Generic router traffic and RRLS forwarding share the same
physical UART capacity state; reconnecting does not create a second allowance.

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
| RTSCTS | S19 | Hardware flow control |

The older [RFDesign ASYNC manual](https://files.rfdesign.com.au/Files/documents/RFD900x%20Asynchronous%20User%20Manual%20V1.1.pdf)
also identifies S4 as retries and S12 as power, but lists flow control at S18.
The captured 4.03 firmware has DUTY_CYCLE at S17 and moves the following entries.
Do not copy register numbers from a different firmware table. In particular,
the earlier SiK-style `ATS4=30` / `ATS14=120` recipe does not configure 30 dBm /
a 120 ms window on this ASYNC firmware.

Pacing is an application budget, not a measured RF capacity guarantee. Check
both directions and discovery after restarting RF; real antenna loss, modem
retries and contention require field validation beyond the UART simulator.
