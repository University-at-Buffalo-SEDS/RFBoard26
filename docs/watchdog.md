# Board watchdog

Build with `./build.py build --release --watchdog` (or CMake
`-DENABLE_BOARD_WATCHDOG=ON`). The default is OFF; the script explicitly clears a
previously enabled CMake cache when the flag is omitted. ThreadX still schedules
all tasks. The watchdog belongs to this board, with no SEDSnet changes.

The independent LSI-driven IWDG has a nominal 16.384 second deadline (/256,
reload 2047). Actual timing follows the MCU's LSI tolerance. A feed requires a
fresh check-in from every monitored worker and at least 250 ms of scheduler
progress. IRQ activity alone cannot feed it. Debug globals expose the reset
cause, missing task mask, feed count, and configuration failure. Reset flags
are captured then cleared so future resets are classified independently.

Monitored workers: network and GPS acquisition. With telemetry disabled, the network bit is excluded; a gateway without
telemetry does not start the watchdog because it has no monitored worker.
This does not monitor every peripheral independently (for example the DAQ SD
writer), and a worker check-in does not prove delivery on a disconnected link.

Install the matching bootloader with the application before enabling the
watchdog: once started, IWDG survives a warm reset. Board bootloader hooks feed
it during startup, storage timing, and the application handoff. A bootloader
that does not service it may enter a reset loop. Bootloader updates require a
wired factory flash. Build and native fault tests do not replace hardware
reset/soak qualification.

Valve and actuator outputs initialize to their normal safe boot state; queued
commands and active output sequences are not replayed by this watchdog.
