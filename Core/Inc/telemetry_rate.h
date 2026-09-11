#ifndef RF_TELEMETRY_RATE_H
#define RF_TELEMETRY_RATE_H

#include <stdint.h>

/* Board-local build setting: whole Hz, 1..1000. Override with -D at build
 * time. This is deliberately not a SEDSNet network variable. */
#ifndef RF_TELEMETRY_RATE_HZ
#define RF_TELEMETRY_RATE_HZ 1U
#endif
#if RF_TELEMETRY_RATE_HZ < 1 || RF_TELEMETRY_RATE_HZ > 1000
#error "RF_TELEMETRY_RATE_HZ must be a whole number from 1 to 1000"
#endif

uint32_t rf_telemetry_period_ms(void);

#endif
