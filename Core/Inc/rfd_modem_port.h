#ifndef RFD_MODEM_PORT_H
#define RFD_MODEM_PORT_H
#include <stdint.h>

/* Once, at boot, before CAN and the SEDSnet router start: runs the modem
 * check unless this image already verified the profile or the restored
 * flight phase is past Idle. While it owns the UART, `service` keeps the
 * watchdog served and sleeps one tick. */
void rfd_modem_port_boot(void (*service)(void));
/* Every telemetry loop pass: never holds the radio. A boot that skipped on
 * its verified marker but hears no radio frame within the probation forgets
 * the marker, so the next boot checks the modem again. */
void rfd_modem_port_poll(void (*service)(void));
/* BOARD_HEALTH words 13, 15, 22 (all zero until the startup step ran). */
void rfd_modem_port_status_words(uint32_t words[3]);

#endif
