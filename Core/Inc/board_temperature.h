#ifndef BOARD_TEMPERATURE_H
#define BOARD_TEMPERATURE_H
#include <stdint.h>
/* Called only from the ADC owner on AB/DAQ; health poll on dedicated ADC boards.
 * Best effort, at most one pair / 5s. Never drives control or watchdog health. */
void board_temperature_poll(void);
uint32_t board_temperature_health_flags(void);
#endif
