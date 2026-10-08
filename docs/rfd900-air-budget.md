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
GS_RADIO_AIR_FRAME_OVERHEAD_BYTES=16. Its Pico/I2C path is not paced. RF build
definitions RADIO_AIR_BIT_RATE_BPS, RADIO_AIR_SHARE_PERCENT and
RADIO_AIR_FRAME_OVERHEAD_BYTES override the matching firmware values.

The modem settings must match at both ends. Check ATS6 for transparent/custom
framing and ATS13 for flow control rather than assuming their values. Route
expiry is retained: pacing cannot repair a powered-off modem, antenna fault,
or corrupt framing. Confirm both directions and discovery after restarting RF.
