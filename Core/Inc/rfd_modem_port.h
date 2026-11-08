#ifndef RFD_MODEM_PORT_H
#define RFD_MODEM_PORT_H
#include <stdint.h>

/* Every telemetry loop pass. Runs the modem check once per boot, after the
 * first FLIGHT_STATE says Startup, Idle, PreFill or FillTest. While the check
 * owns the UART, `service` runs one CAN/routing/watchdog pass without the
 * radio and sleeps one tick. */
void rfd_modem_port_poll(void (*service)(void));
/* BOARD_HEALTH words 13, 15, 22 (all zero until the startup step ran). */
void rfd_modem_port_status_words(uint32_t words[3]);

#endif
