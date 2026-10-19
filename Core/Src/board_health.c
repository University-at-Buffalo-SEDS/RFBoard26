#include "board_health.h"
#include "board_uptime.h"
#include "board_watchdog.h"
#include "telemetry.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Board diagnostic words require little-endian encoding"
#endif
#ifdef TELEMETRY_USE_TLSF
extern volatile uint32_t g_telemetry_tlsf_live_bytes, g_telemetry_tlsf_peak_bytes, g_telemetry_tlsf_region_bytes, g_telemetry_tlsf_failures;
#endif
extern uint32_t board_health_can_rx_drops(void);
extern void telemetry_lock(void);
extern void telemetry_unlock(void);
extern uint32_t board_health_uart_rx_drops(void);

void board_health_poll(SedsRouter *router)
{
#ifdef TELEMETRY_ENABLED
  static uint32_t last_report_ms;
  static uint32_t failures;
  if (!router) return;
  const uint64_t uptime = board_uptime_ticks();
  const uint32_t now = (uint32_t)uptime;
  if ((uint32_t)(now - last_report_ms) < 5000U) return;
  last_report_ms = now;
  uint32_t words[24] = {1U, 2U, (uint32_t)uptime, (uint32_t)(uptime >> 32U), BOARD_UPTIME_FREQUENCY_HZ};
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  words[5] = g_watchdog_reset_flags; words[6] = g_watchdog_missing_mask;
#ifdef TELEMETRY_USE_TLSF
  /* Exact existing occupied-byte counters. Avoid interrupt-disabled heap
   * walks, and do not mislabel occupied headroom as contiguous free space. */
  words[1] |= 1U;
  words[7] = g_telemetry_tlsf_live_bytes; words[8] = g_telemetry_tlsf_peak_bytes;
  words[9] = g_telemetry_tlsf_region_bytes; words[10] = g_telemetry_tlsf_failures;
#endif
  words[12] = board_health_can_rx_drops();
  words[1] |= 8U; words[14] = board_health_uart_rx_drops();
  __set_PRIMASK(saved);
  words[11] = (uint32_t)seds_memory_admission_rejected();
  words[23] = failures;
  telemetry_lock();
  /* Static UInt32 schema + exact little-endian bytes: use the existing
   * bounded byte path without pulling every generic numeric converter in. */
  const SedsResult result = seds_router_log_bytes_ex(router, SEDS_DT_BOARD_HEALTH,
      (const uint8_t *)words, sizeof(words), NULL, true);
  telemetry_unlock();
  if (result != SEDS_OK) ++failures;
#else
  (void)router;
#endif
}
