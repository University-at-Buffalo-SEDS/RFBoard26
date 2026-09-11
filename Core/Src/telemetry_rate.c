#include "telemetry_rate.h"

uint32_t rf_telemetry_period_ms(void)
{
  return (1000U + RF_TELEMETRY_RATE_HZ / 2U) / RF_TELEMETRY_RATE_HZ;
}
