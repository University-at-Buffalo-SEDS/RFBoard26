#include "board_commissioning.h"

#ifdef TELEMETRY_ENABLED
extern int telemetry_router_enter(void);
extern int telemetry_router_leave(void);
#endif
#include "can_tx_queue.h"
#include "board_health.h"
#ifdef TELEMETRY_USE_TLSF
#include "telemetry_tlsf.h"
#endif
// telemetry.c
#include "telemetry.h"
#include "telemetry_rate.h"
#include "av_bay_underglow.h"
#include "flight_state_cache.h"
#include "board_recovery_transport.h"
#include "sim_network_probe.h"
#include "board_can_priority.h"
#include "ota_stream.h"

#include "app_threadx.h"
#include "can_bus.h"
#include "radio.h"
#include "sedsnet_config.h"
#include "board_packet_store.h"
#include "stm32g4xx_hal.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len) {
  (void)data;
  (void)len;
}
#endif

#if defined(__GNUC__) || defined(__clang__)
#define UNUSED_FUNCTION __attribute__((unused))
#else
#define UNUSED_FUNCTION
#endif

#ifndef TELEMETRY_TIMESYNC_MASTER_PRIO
#define TELEMETRY_TIMESYNC_MASTER_PRIO (-1)
#endif

#ifndef SEDS_FIRMWARE_SIM_TEST
#define SEDS_FIRMWARE_SIM_TEST 0
#endif

#ifndef TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS
#define TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS 5000U
#endif

#ifndef TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS
#define TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS 2000U
#endif

#ifndef TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS
#define TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS 2000U
#endif

/* RF leads SEDSnet time only from GPS UTC. Its tick runs from the HSI
 * (about 1%) and is not disciplined by the receiver, so after the last valid
 * GPS date/time it keeps the source role for at most this long (worst case
 * about 0.3 s of drift), then yields to the GS as a consumer. */
#ifndef TELEMETRY_TIMESYNC_GPS_HOLDOVER_MS
#define TELEMETRY_TIMESYNC_GPS_HOLDOVER_MS 30000U
#endif

#ifndef COMMAND_TRACE_PRINTS
#define COMMAND_TRACE_PRINTS 0
#endif

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

#define TELEMETRY_TIMESYNC_ROLE_CONSUMER 0U
#define TELEMETRY_TIMESYNC_ROLE_SOURCE 1U
#define TELEMETRY_FLIGHT_CAN_ID 0x101U
#define TELEMETRY_FLIGHT_HEARTBEAT_CAN_ID 0x001U

static uint32_t telemetry_flight_can_id(const uint8_t *bytes, size_t len, uint8_t priority) {
  return board_can_arbitration_id(bytes, len, priority,
      sim_probe_packed_data_type(bytes, len), (uint32_t)SEDS_DT_HEARTBEAT,
      TELEMETRY_FLIGHT_HEARTBEAT_CAN_ID);
}
#define TELEMETRY_ROUTER_RETRY_MS 500U
#define TELEMETRY_SERVICE_RETRY_MS 5000U
#define TELEMETRY_SERVICE_TIMESYNC 0x1U
#define TELEMETRY_SERVICE_OTA 0x2U
#define TELEMETRY_SERVICE_UNDERGLOW 0x4U
#define TELEMETRY_SERVICE_FLIGHT_STATE 0x8U
#define TELEMETRY_SERVICE_ALL 0xFU

#ifndef RF_SEDSNET_MEMORY_POOL_SIZE
#define RF_SEDSNET_MEMORY_POOL_SIZE 34816U
#endif
#ifndef RF_SEDSNET_QUEUE_BUDGET
#define RF_SEDSNET_QUEUE_BUDGET 9984U
#endif
#ifndef RF_SEDSNET_MAX_RECENT_RX_IDS
#define RF_SEDSNET_MAX_RECENT_RX_IDS 16U
#endif
#ifndef RF_SEDSNET_STARTING_ALLOCATION
#define RF_SEDSNET_STARTING_ALLOCATION 1024U
#endif
/* At 64 kbit/s with a 40% airtime share, a 1 KiB frame blocks the
 * non-preemptible modem path for about 327 ms. Short schema chunks bound
 * that interval to about 67 ms while leaving ordinary telemetry intact. */
#define RF_RADIO_MAX_FRAME_BYTES 192U
#define RF_CAN_MAX_FRAME_BYTES CAN_TX_ALIGNED_FRAME_BYTES
/* These are side-wide receive dictionaries. Four entries cannot retain
 * the GroundStation control flows or the combined FC/PowerBoard CAN flows.
 * Keep them bounded; allocator admission still protects packet/schema work. */
#define RF_CAN_SIDE_TRANSPORT_TEMPLATES 16U
#define RF_RADIO_SIDE_TRANSPORT_TEMPLATES 16U

_Static_assert(RF_SEDSNET_QUEUE_BUDGET >=
                   (RF_SEDSNET_MAX_RECENT_RX_IDS * sizeof(uint64_t)) +
                       (2U * RF_SEDSNET_STARTING_ALLOCATION),
               "SEDSNet pool must leave space after recent IDs and initial RX/TX queues");

static uint8_t g_can_rx_subscribed = 0U;
static uint8_t g_radio_rx_subscribed = 0U;
static int32_t g_can_side_id = -1;
static int32_t g_radio_side_id = -1;
static uint8_t g_local_unix_valid = 0U;
static uint64_t g_local_unix_ms = 0ULL;
static uint32_t g_local_unix_set_ms = 0U;
static uint32_t g_router_retry_after_ms = 0U;
static uint8_t g_radio_link_seen = 0U;

extern void telemetry_memory_profile_mark(uint32_t stage);

static uint8_t g_discovery_schema_announced = 0U;
static uint32_t g_discovery_schema_retry_ms = 0U;

RouterState g_router = {.r = NULL, .created = 0U, .start_time = 0ULL};

/* Exported simulator/HIL health signals. A linked-bay test requires both a
 * remote discovery topology change and a valid SEDSNet network clock. */
#define RF_HEALTH_PROBE __attribute__((used, externally_visible))
#if SEDS_FIRMWARE_SIM_TEST
volatile uint32_t g_rf_memory_counters[16] RF_HEALTH_PROBE;
volatile uint32_t g_rf_memory_snapshot_tick RF_HEALTH_PROBE;
#endif
volatile uint32_t g_telemetry_discovery_seen RF_HEALTH_PROBE = 0U;
volatile uint32_t g_telemetry_timesync_valid RF_HEALTH_PROBE = 0U;
/* Network clock restarts after it disagreed with the station clock by >60 s. */
volatile uint32_t g_network_clock_resyncs RF_HEALTH_PROBE = 0U;
/* A restart cannot fix a source that disagrees with the GS; repeating it on
 * every GS packet kept the clock torn down. Monotonic ticks pace it. */
#define NETWORK_CLOCK_RESYNC_HOLDOFF_TICKS (30U * (ULONG)TX_TIMER_TICKS_PER_SECOND)
static ULONG g_network_clock_resync_tick;
volatile uint32_t g_telemetry_network_ready RF_HEALTH_PROBE = 0U;
/* Optional router services that failed to start (TELEMETRY_SERVICE_* bits).
 * The CAN<->radio relay, discovery and BOARD_HEALTH keep running; failed
 * services are retried on a bounded schedule and reported in BOARD_HEALTH. */
volatile uint32_t g_telemetry_service_faults RF_HEALTH_PROBE = 0U;
volatile int32_t g_telemetry_service_last_error RF_HEALTH_PROBE = 0;
#ifdef TELEMETRY_ENABLED
static uint32_t g_telemetry_service_retry_ms = 0U;
static void telemetry_retry_services_locked(void);
#endif
volatile uint32_t g_telemetry_peer_mask RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_heartbeat_attempts RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_heartbeat_ok RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_heartbeat_fail RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_heartbeat_wire_tx RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_radio_egress_peer_mask RF_HEALTH_PROBE = 0U;
volatile uint32_t g_telemetry_timesync_queued RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_gps_publish_attempts RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_gps_publish_ok RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_avionics_telemetry_ingress RF_HEALTH_PROBE = 0U;
volatile uint32_t g_sim_avionics_telemetry_radio_egress RF_HEALTH_PROBE = 0U;

static SedsResult telemetry_send_can_packet(const uint8_t *bytes, size_t len, uint8_t priority);

static uint64_t tx_raw_now_ms_locked(void) {
  const uint32_t ticks32 = (uint32_t)tx_time_get();
  return ((uint64_t)ticks32 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static UNUSED_FUNCTION uint64_t tx_raw_now_ms(void *user) {
  (void)user;
  return tx_raw_now_ms_locked();
}

/* Without GPS UTC the board is a consumer: a Source announces even without a
 * date, and its uptime would anchor relayed boards that never correct it. */
static uint8_t telemetry_timesync_is_source(void) {
  return (TELEMETRY_TIMESYNC_MASTER_PRIO >= 0 && g_local_unix_valid) ? 1U : 0U;
}

static uint64_t telemetry_timesync_priority(void) {
  // Holdover consumers must yield when RF (1) or GroundStation (50) returns.
  return telemetry_timesync_is_source() ? (uint64_t)TELEMETRY_TIMESYNC_MASTER_PRIO : 100ULL;
}

static uint32_t telemetry_timesync_role(void) {
  return telemetry_timesync_is_source() ? TELEMETRY_TIMESYNC_ROLE_SOURCE
                                        : TELEMETRY_TIMESYNC_ROLE_CONSUMER;
}

static SedsResult telemetry_send_can_packet(const uint8_t *bytes, size_t len, uint8_t priority) {
  return can_bus_send_large(bytes, len, telemetry_flight_can_id(bytes, len, priority)) == HAL_OK
      ? SEDS_OK : SEDS_IO;
}

static bool telemetry_unix_ms_to_utc(uint64_t unix_ms, int32_t *year, uint8_t *month,
                                     uint8_t *day, uint8_t *hour, uint8_t *minute,
                                     uint8_t *second, uint16_t *millisecond) {
  static const uint16_t days_before_month[12] = {0U,   31U,  59U,  90U,  120U, 151U,
                                                 181U, 212U, 243U, 273U, 304U, 334U};
  uint64_t whole_seconds = unix_ms / 1000ULL;
  const uint64_t days_since_epoch = whole_seconds / 86400ULL;
  uint32_t seconds_of_day = (uint32_t)(whole_seconds % 86400ULL);
  int32_t y = 1970;
  uint64_t days = days_since_epoch;

  if (!year || !month || !day || !hour || !minute || !second || !millisecond) {
    return false;
  }

  while (1) {
    const uint32_t y_u32 = (uint32_t)y;
    const uint8_t leap =
        ((y_u32 % 4U) == 0U && ((y_u32 % 100U) != 0U || (y_u32 % 400U) == 0U)) ? 1U : 0U;
    const uint32_t days_in_year = leap ? 366U : 365U;
    if (days < days_in_year) {
      uint8_t m = 1U;
      uint32_t day_of_year = (uint32_t)days;
      for (; m <= 12U; ++m) {
        uint32_t month_start = days_before_month[m - 1U];
        uint32_t month_end =
            (m < 12U) ? days_before_month[m] : (uint32_t)(leap ? 366U : 365U);
        if (leap && m > 2U) {
          month_start += 1U;
          month_end += 1U;
        }
        if (day_of_year < month_end) {
          *year = y;
          *month = m;
          *day = (uint8_t)(day_of_year - month_start + 1U);
          *hour = (uint8_t)(seconds_of_day / 3600U);
          *minute = (uint8_t)((seconds_of_day % 3600U) / 60U);
          *second = (uint8_t)(seconds_of_day % 60U);
          *millisecond = (uint16_t)(unix_ms % 1000ULL);
          return true;
        }
      }
      return false;
    }
    days -= days_in_year;
    ++y;
  }
}

static SedsResult telemetry_apply_local_unix_time_locked(SedsRouter *router) {
  int32_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;

  if (!router || !telemetry_timesync_is_source() || !g_local_unix_valid) {
    return SEDS_OK;
  }

  if (!telemetry_unix_ms_to_utc(g_local_unix_ms, &year, &month, &day, &hour, &minute,
                                &second, &millisecond)) {
    return SEDS_BAD_ARG;
  }

  return seds_router_set_local_network_datetime_millis(router, year, month, day, hour, minute,
                                                       second, millisecond);
}

static SedsResult telemetry_configure_timesync_locked(SedsRouter *router) {
  SedsResult result;

  if (!router) {
    return SEDS_BAD_ARG;
  }

#if SEDS_FIRMWARE_SIM_TEST
  /* The simulator GPS exercises SPI but does not synthesize an NMEA date.
   * Seed only qualification builds so the linked firmware must complete the
   * real SEDSNet time-sync exchange; production still waits for valid GPS. */
  if (TELEMETRY_TIMESYNC_MASTER_PRIO >= 0 && !g_local_unix_valid) {
    g_local_unix_ms = 1767225600000ULL + tx_raw_now_ms_locked();
    g_local_unix_valid = 1U;
  }
#endif

  result = seds_router_configure_timesync(
      router, true, telemetry_timesync_role(), telemetry_timesync_priority(),
      (uint64_t)TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS,
      (uint64_t)TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS,
      (uint64_t)TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS);
  if (result != SEDS_OK) {
    return result;
  }

  return telemetry_apply_local_unix_time_locked(router);
}

uint64_t telemetry_now_ms(void) { return tx_raw_now_ms_locked(); }

static uint64_t telemetry_unix_ms_locked(void) {
#ifndef TELEMETRY_ENABLED
  return g_local_unix_valid ? g_local_unix_ms : 0ULL;
#else
  uint64_t unix_ms = 0ULL;

  if (g_router.r && seds_router_get_network_time_ms(g_router.r, &unix_ms) == SEDS_OK) {
    return unix_ms;
  }

  if (telemetry_timesync_is_source() && g_local_unix_valid) {
    return g_local_unix_ms;
  }

  return 0ULL;
#endif
}

uint64_t telemetry_unix_ms(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return 0;
#endif
  uint64_t result = telemetry_unix_ms_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave()) return 0;
#endif
  return result;
}


uint64_t telemetry_unix_s(void) { return telemetry_unix_ms() / 1000ULL; }

uint8_t telemetry_unix_is_valid(void) { return telemetry_unix_ms() != 0ULL ? 1U : 0U; }

static void telemetry_set_unix_time_ms_locked(uint64_t unix_ms) {
#ifdef TELEMETRY_ENABLED
  const uint8_t was_source = telemetry_timesync_is_source();
#endif
  g_local_unix_ms = unix_ms;
  g_local_unix_valid = (unix_ms != 0ULL) ? 1U : 0U;
  g_local_unix_set_ms = (uint32_t)tx_raw_now_ms_locked();

#ifdef TELEMETRY_ENABLED
  if (g_router.r != NULL) {
    /* A role change resets SEDSnet time state; configuring re-applies GPS time. */
    (void)(telemetry_timesync_is_source() != was_source
               ? telemetry_configure_timesync_locked(g_router.r)
               : telemetry_apply_local_unix_time_locked(g_router.r));
  }
#endif
}

void telemetry_set_unix_time_ms(uint64_t unix_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  telemetry_set_unix_time_ms_locked(unix_ms);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static SedsResult tx_send_with_priority(const uint8_t *bytes, size_t len,
                                        uint8_t priority, void *user);

SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  return tx_send_with_priority(bytes, len, 0U, user);
}


static uint64_t node_now_since_ms(void *user) {
  (void)user;
  const RouterState s = g_router;
  const uint64_t now = tx_raw_now_ms_locked();
  /* Construction and initial maintenance also need an advancing clock. */
  return now - s.start_time;
}

static SedsResult tx_send_with_priority(const uint8_t *bytes, size_t len,
                                        uint8_t priority, void *user) {
  (void)user;

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }
  sim_probe_observe_can_tx(bytes, len);

  return telemetry_send_can_packet(bytes, len, priority);
}

static SedsResult radio_tx_send(const uint8_t *bytes, size_t len,
                                uint8_t priority, void *user) {
  (void)user;
  HAL_StatusTypeDef status;

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }

#if SEDS_FIRMWARE_SIM_TEST
  g_sim_radio_egress_peer_mask |= sim_probe_peer_bit_packed(bytes, len);
  {
    const uint32_t ty = sim_probe_packed_data_type(bytes, len);
    if (ty == (uint32_t)SEDS_DT_GPS_SATELLITE_NUMBER ||
        ty == (uint32_t)SEDS_DT_IMU_DATA ||
        ty == (uint32_t)SEDS_DT_BATTERY_VOLTAGE) {
      g_sim_avionics_telemetry_radio_egress++;
    }
  }
#endif

  status = radio_uart_send_bytes_priority(bytes, len, priority);
  return (status == HAL_OK) ? SEDS_OK : SEDS_IO;
}

static void telemetry_can_rx_locked(const uint8_t *data, size_t len, void *user) {
  (void)user;
  /* Recovery envelopes are for the ground hosts. Only the RRLS-unwrapped
   * canonical uplink is eligible for routing or radio-link liveness. */
  if (radio_uart_current_rx_is_command_frame() && data && len >= 3U &&
      data[0] == 'R' && data[1] == 'R' && data[2] == 1U) {
    return;
  }
  sim_probe_observe_packed(data, len);
#if SEDS_FIRMWARE_SIM_TEST
  {
    const uint32_t ty = sim_probe_packed_data_type(data, len);
    if (ty == (uint32_t)SEDS_DT_IMU_DATA ||
        ty == (uint32_t)SEDS_DT_BATTERY_VOLTAGE) {
      g_sim_avionics_telemetry_ingress++;
    }
  }
#endif

#ifdef TELEMETRY_ENABLED
  if (!data || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_receive_packed_from_side(
        g_router.r, (uint32_t)g_can_side_id, data, len);
  } else {
    (void)seds_router_receive_packed(g_router.r, data, len);
  }
  g_telemetry_discovery_seen = 1U;
#else
  (void)data;
  (void)len;
#endif
}

static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  telemetry_can_rx_locked(data, len, user);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static void telemetry_radio_rx_locked(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (radio_uart_handle_profile_request(data, len,
          radio_uart_current_rx_is_command_frame())) return;
  sim_probe_observe_packed(data, len);

#ifdef TELEMETRY_ENABLED
  if (!data || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

#if COMMAND_TRACE_PRINTS
  printf("Radio uplink RX: kind=%s len=%u preview=",
         radio_uart_current_rx_is_command_frame() ? "command" : "data",
         (unsigned)len);
  for (size_t i = 0U; i < len && i < 12U; i++) {
    printf("%02X%s", data[i], (i + 1U < len && i + 1U < 12U) ? " " : "");
  }
  if (len > 12U) {
    printf(" ...");
  }
  printf("\r\n");
#endif

  if (g_radio_side_id >= 0) {
    (void)seds_router_receive_packed_from_side(
        g_router.r, (uint32_t)g_radio_side_id, data, len);
  } else {
    (void)seds_router_receive_packed(g_router.r, data, len);
  }
  g_radio_link_seen = 1U;
  g_telemetry_discovery_seen = 1U;
#else
  (void)data;
  (void)len;
#endif
}

static void telemetry_radio_rx(const uint8_t *data, size_t len, void *user) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  telemetry_radio_rx_locked(data, len, user);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static void rx_asynchronous_locked(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_receive_packed_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  } else {
    (void)seds_router_receive_packed(g_router.r, bytes, len);
  }
#endif
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  rx_asynchronous_locked(bytes, len);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static UNUSED_FUNCTION void rx_synchronous_locked(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_receive_packed_from_side(g_router.r, (uint32_t)g_can_side_id, bytes,
                                                   len);
  } else {
    (void)seds_router_receive_packed(g_router.r, bytes, len);
  }
#endif
}

static UNUSED_FUNCTION void rx_synchronous(const uint8_t *bytes, size_t len) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return;
#endif
  rx_synchronous_locked(bytes, len);
#ifdef TELEMETRY_ENABLED
  (void)telemetry_router_leave();
#endif
}


static void telemetry_update_network_health(SedsRouter *router) {
#if SEDS_FIRMWARE_SIM_TEST
  /* Fixed counters allocate nothing and stay off production RX paths. */
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - g_rf_memory_snapshot_tick) >= 100U) {
    uint32_t counters[16];
    if (seds_router_copy_memory_counters(router, counters, 16U) == 16) {
      for (unsigned i = 0; i < 16U; ++i) g_rf_memory_counters[i] = counters[i];
    }
    g_rf_memory_snapshot_tick = now;
  }
#endif

  uint64_t network_time_ms = 0ULL;
  const ULONG now_ticks = tx_time_get();
  if (seds_router_get_network_time_ms(router, &network_time_ms) == SEDS_OK) {
    g_telemetry_timesync_valid = 1U;
    if ((g_network_clock_resyncs == 0U ||
         (ULONG)(now_ticks - g_network_clock_resync_tick) >= NETWORK_CLOCK_RESYNC_HOLDOFF_TICKS) &&
        board_recovery_network_clock_resync_needed(network_time_ms) &&
        telemetry_configure_timesync_locked(router) == SEDS_OK) {
      g_network_clock_resyncs++;
      g_network_clock_resync_tick = now_ticks;
    }
  }
  if (g_telemetry_discovery_seen != 0U && g_radio_link_seen != 0U &&
      g_telemetry_timesync_valid != 0U) {
    g_telemetry_network_ready = 1U;
  }
}

/* Bounded GPS holdover: past it, the board drops back to a consumer. The
 * simulator seed stands in for a permanent fix. */
#ifdef TELEMETRY_ENABLED
static void telemetry_expire_gps_time_locked(void) {
#if !SEDS_FIRMWARE_SIM_TEST
  if (g_local_unix_valid && (uint32_t)((uint32_t)tx_raw_now_ms_locked() - g_local_unix_set_ms) >
                                TELEMETRY_TIMESYNC_GPS_HOLDOVER_MS) {
    telemetry_set_unix_time_ms_locked(0ULL);
  }
#endif
}
#endif

static SedsResult telemetry_poll_timesync_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  telemetry_expire_gps_time_locked();

  bool did_queue = false;
  board_commissioning_service();
  (void)flight_state_cache_poll(g_router.r);
  const SedsResult result = seds_router_poll_timesync(g_router.r, &did_queue);
  if (did_queue) {
    g_telemetry_timesync_queued++;
  }
  telemetry_update_network_health(g_router.r);
  return result;
#endif
}

SedsResult telemetry_poll_timesync(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = telemetry_poll_timesync_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult telemetry_announce_discovery_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_announce_discovery(g_router.r);
#endif
}

SedsResult telemetry_announce_discovery(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = telemetry_announce_discovery_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult telemetry_poll_discovery_locked(void) {
#ifdef TELEMETRY_ENABLED
  if (g_router.r) board_health_poll(g_router.r);
#endif

#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  bool did_queue = false;
  /* Cadence polling advertises routes, not schema. Publish the complete local
   * catalog once after startup so existing relays learn newly added types.
   * Leave queue processing running between bounded retries on queue pressure. */
  const uint64_t now = tx_raw_now_ms_locked();
  if (g_discovery_schema_retry_ms == 0U ||
      (int32_t)((uint32_t)now - g_discovery_schema_retry_ms) >= 0) {
    if (!g_discovery_schema_announced) {
      g_discovery_schema_retry_ms = (uint32_t)now + 1000U;
      if (!g_discovery_schema_retry_ms) g_discovery_schema_retry_ms = 1U;
      if (seds_router_announce_discovery(g_router.r) == SEDS_OK) {
        g_discovery_schema_announced = 1U;
      }
    }
  }
  telemetry_retry_services_locked();
  const SedsResult result = seds_router_poll_discovery(g_router.r, &did_queue);
  if (result == SEDS_OK) {
    sim_probe_emit_heartbeat(g_router.r, telemetry_now_ms());
    (void)av_bay_underglow_poll(g_router.r);
  }
  telemetry_update_network_health(g_router.r);
  return result;
#endif
}

SedsResult telemetry_poll_discovery(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = telemetry_poll_discovery_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


#ifdef TELEMETRY_USE_TLSF
static bool radio_admission_probe(size_t additional, size_t largest, uint32_t work)
{
  if (work == SEDS_MEMORY_WORK_SMALL_PROTECTED)
    return telemetry_tlsf_admit_small_protected(additional, largest);
  return telemetry_tlsf_admit(additional, largest);
}
#endif

static SedsResult Board_Recovery_handler(const SedsPacketView *packet, void *user)
{
  (void)user;
  return board_recovery_handle(g_router.r, packet, SEDS_EP_BOARD_RECOVERY,
      (uint32_t)(((uint64_t)tx_time_get() * 1000U) / TX_TIMER_TICKS_PER_SECOND));
}

static const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = SEDS_EP_BOARD_RECOVERY,
          .packet_handler = Board_Recovery_handler,
          .packed_handler = NULL,
          .user = NULL,
      },
};

#ifdef TELEMETRY_ENABLED
/* Start the optional services named in `wanted`; returns the still-failed
 * subset. Each initializer tolerates being repeated after a failure. */
static uint32_t telemetry_start_services_locked(SedsRouter *r, uint32_t wanted) {
  /* Bit i of TELEMETRY_SERVICE_* is entry i; kept as one table and one
   * diagnostic line to stay inside the simulator image's flash budget. */
  static SedsResult (*const start[4])(SedsRouter *) = {
      telemetry_configure_timesync_locked, ota_stream_init,
      av_bay_underglow_init, flight_state_cache_init};
  uint32_t failed = 0U;
  for (uint32_t i = 0U; i < 4U; i++) {
    if ((wanted & (1UL << i)) == 0U) continue;
    const SedsResult result = start[i](r);
    if (result != SEDS_OK) {
      failed |= 1UL << i;
      g_telemetry_service_last_error = (int32_t)result;
    }
    if (i == 0U) telemetry_memory_profile_mark(4U);
    else if (i == 1U) telemetry_memory_profile_mark(5U);
  }
  g_telemetry_service_faults = (g_telemetry_service_faults & ~wanted) | failed;
  if (failed != 0U) {
    /* 1 timesync, 2 OTA stream/storage, 4 underglow, 8 flight-state cache. */
    printf("Error: failed services 0x%lx (%d)\r\n", (unsigned long)failed,
           (int)g_telemetry_service_last_error);
    g_telemetry_service_retry_ms =
        (uint32_t)tx_raw_now_ms_locked() + TELEMETRY_SERVICE_RETRY_MS;
  }
  return failed;
}

/* Bounded background retry of failed optional services (router lock held). */
static void telemetry_retry_services_locked(void) {
  if (!g_router.r || g_telemetry_service_faults == 0U) return;
  const uint32_t now = (uint32_t)tx_raw_now_ms_locked();
  if ((int32_t)(now - g_telemetry_service_retry_ms) < 0) return;
  (void)telemetry_start_services_locked(g_router.r, g_telemetry_service_faults);
}
#endif

static SedsResult init_telemetry_router_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  SedsRouter *r = NULL;
  SedsResult result = SEDS_OK;

  if (g_router.created && g_router.r) {
    return SEDS_OK;
  }

  const uint64_t init_now_ms = tx_raw_now_ms_locked();
  telemetry_memory_profile_mark(0U);

  /* Establish the router clock epoch before constructing the router or adding
   * either side. Those operations schedule discovery and time-sync deadlines.
   * Resetting start_time afterward moves the monotonic clock backwards and can
   * postpone the first cross-side discovery refresh until the raw boot uptime
   * is reached a second time. */
  g_router.start_time = init_now_ms;

  if (!g_can_rx_subscribed) {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK) {
      g_can_rx_subscribed = 1U;
    } else {
      printf("Error: can_bus_subscribe_rx failed\r\n");
    }
  }

  if (!g_radio_rx_subscribed) {
    if (radio_uart_subscribe_rx(telemetry_radio_rx, NULL) == HAL_OK) {
      g_radio_rx_subscribed = 1U;
    } else {
      printf("Error: radio_uart_subscribe_rx failed\r\n");
    }
  }

  const SedsRuntimeMemoryConfig memory = {
      .max_queue_budget = RF_SEDSNET_QUEUE_BUDGET,
      .max_recent_rx_ids = RF_SEDSNET_MAX_RECENT_RX_IDS,
      .starting_queue_size = RF_SEDSNET_STARTING_ALLOCATION,
      .queue_grow_step = 1.0,
  };
#ifdef TELEMETRY_USE_TLSF
  seds_set_memory_admission_probe_v2(radio_admission_probe);
#endif
  result = board_packet_store_init();
  if (result != SEDS_OK) return result;

  /* Match GroundStation's short-frame radio retry cadence. A 500 ms replay
   * gate cannot recover a lost downstream receipt inside a 500 ms target.
   * Keep the same 4.5-second total retry window and all queue/memory limits. */
  SedsRuntimeTuningConfig tuning;
  result = seds_get_runtime_tuning_config(&tuning);
  if (result != SEDS_OK) return result;
  tuning.reliable_retransmit_ms = 250U;
  tuning.reliable_max_retries = 17U;
  result = seds_set_runtime_tuning_config(&tuning);
  if (result != SEDS_OK) return result;

  r = seds_router_new_with_memory(node_now_since_ms, NULL, locals, sizeof(locals)/sizeof(locals[0]),
                                  SEDS_ROUTER_E2E_DISABLED, 0U, &memory);
  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;

    return SEDS_ERR;
  }
  if (seds_router_set_preferred_discovery_master(r, "GS", 2U) != SEDS_OK) {
    printf("Error: GS discovery master\r\n");
    seds_router_free(r);
    return SEDS_ERR;
  }
  telemetry_memory_profile_mark(1U);

  g_can_side_id = seds_router_add_side_packed_profile_with_priority(
      r, "can", 3U, tx_send_with_priority, NULL, false,
      SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE, RF_CAN_MAX_FRAME_BYTES, 0U,
      RF_CAN_SIDE_TRANSPORT_TEMPLATES);
  if (g_can_side_id < 0) {
    printf("Error: failed to add CAN side: %ld\r\n", (long)g_can_side_id);
    g_can_side_id = -1;
  }
  telemetry_memory_profile_mark(2U);

  /* The RFD900x already provides its own acknowledged/retried radio link.
   * Adding SEDSNet hop ACKs on top of that kept old control packets in the
   * relay queue and eventually starved CAN-to-ground application traffic.
   * Register both relay sides before configuring services so discovery can
   * advertise and incrementally update the complete two-sided topology from
   * the first poll, matching the working Gateway relay lifecycle. */
  g_radio_side_id = seds_router_add_side_packed_profile_with_priority(
      r, "radio", 5U, radio_tx_send, NULL, false,
      SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE, RF_RADIO_MAX_FRAME_BYTES, 0U,
      RF_RADIO_SIDE_TRANSPORT_TEMPLATES);
  if (g_radio_side_id < 0) {
    printf("Error: failed to add radio side: %ld\r\n", (long)g_radio_side_id);
    g_radio_side_id = -1;
  }
  if (g_can_side_id < 0 || g_radio_side_id < 0) {
    /* The failing side was printed above; only this path frees the router. */
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;

    return SEDS_ERR;
  }
#if !RFBOARD_LORA_SCHEDULER && defined(SEDS_ENABLE_LATEST_VALUE_EGRESS)
  if (seds_router_set_latest_value_side(r, g_radio_side_id) != SEDS_OK) {
    printf("Error: radio pending-value side\r\n");
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;

    return SEDS_ERR;
  }
#endif
  telemetry_memory_profile_mark(3U);

  /* The router and both relay sides exist. Everything below is an optional
   * service: a failure is recorded and retried in the background, but it
   * must never free the router. Freeing it here announced DISCOVERY_LEAVE,
   * dropped every CAN<->radio frame and hid FC/PB from the ground until the
   * next 500 ms retry, forever, when the failure was persistent. */
  g_discovery_schema_announced = 0U;
  g_discovery_schema_retry_ms = 0U;
  g_telemetry_service_faults = 0U;
  (void)telemetry_start_services_locked(r, TELEMETRY_SERVICE_ALL);
  telemetry_memory_profile_mark(6U);

  /* Prime the first source announcement after the router clock and sides are
   * fully live.  This avoids depending on a later scheduler tick to make a
   * newly booted bay discover a valid network clock. A failure here is not
   * a service fault: the regular timesync poll repeats it. */
  {
    bool did_queue = false;
    result = seds_router_poll_timesync(r, &did_queue);
    if (result == SEDS_OK && did_queue) g_telemetry_timesync_queued++;
  }

  /* Publish only a complete router. RX and lazy producers share its mutex. */
  board_recovery_transport_set_router(r);
  g_router.r = r;
  g_router.created = 1U;
  return SEDS_OK;
#endif
}
SedsResult init_telemetry_router(void)
{
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result;
#ifdef TELEMETRY_ENABLED
  const uint32_t now_ms = (uint32_t)tx_raw_now_ms_locked();
  if (g_router.created && g_router.r) {
    result = SEDS_OK;
  } else if (g_router_retry_after_ms != 0U &&
             (int32_t)(now_ms - g_router_retry_after_ms) < 0) {
    result = SEDS_IO;
  } else {
    result = init_telemetry_router_locked();
    if (result == SEDS_OK) {
      g_router_retry_after_ms = 0U;
    } else {
      g_router_retry_after_ms = now_ms + TELEMETRY_ROUTER_RETRY_MS;
      if (g_router_retry_after_ms == 0U) g_router_retry_after_ms = 1U;
    }
  }
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#else
  result = init_telemetry_router_locked();
#endif
  return result;
}


static inline SedsElemKind guess_kind_from_elem_size(size_t elem_size) {
  if (elem_size == 4U || elem_size == 8U) {
    return SEDS_EK_FLOAT;
  }
  return SEDS_EK_UNSIGNED;
}

static SedsResult log_telemetry_synchronous_locked(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U || element_size == 0U) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_log_typed(g_router.r, data_type, data, element_count, element_size,
                               guess_kind_from_elem_size(element_size));
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_telemetry_synchronous_locked(data_type, data, element_count, element_size);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult log_telemetry_asynchronous_locked(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U || element_size == 0U) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  /*
   * The CAN and radio transports already own their hardware queues.  Queueing
   * the same packet in SEDSNet as well causes burst traffic to grow a second,
   * fragmented heap-backed queue before the telemetry thread can drain it.
   * Dispatch into the transport immediately; the transport scheduler still
   * controls when bytes reach the wire.
   */
  if (data_type == SEDS_DT_GPS_DATA) ++g_sim_gps_publish_attempts;
  const SedsResult result = seds_router_log_typed(
      g_router.r, data_type, data, element_count, element_size,
      guess_kind_from_elem_size(element_size));
  if (data_type == SEDS_DT_GPS_DATA && result == SEDS_OK) ++g_sim_gps_publish_ok;
  return result;
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_telemetry_asynchronous_locked(data_type, data, element_count, element_size);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult log_telemetry_string_asynchronous_locked(SedsDataType data_type, const char *str) {
#ifdef TELEMETRY_ENABLED
  if (!str) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_log_string_ex(g_router.r, data_type, str, strlen(str), NULL, 0);
#else
  (void)data_type;
  (void)str;
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_string_asynchronous(SedsDataType data_type, const char *str) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_telemetry_string_asynchronous_locked(data_type, str);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult dispatch_tx_queue_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = dispatch_tx_queue_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult process_rx_queue_locked(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = process_rx_queue_locked();
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult dispatch_tx_queue_timeout_locked(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = dispatch_tx_queue_timeout_locked(timeout_ms);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult process_rx_queue_timeout_locked(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = process_rx_queue_timeout_locked(timeout_ms);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult process_all_queues_timeout_locked(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = process_all_queues_timeout_locked(timeout_ms);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


static SedsResult log_error_impl_locked(uint8_t queue, const char *fmt, va_list args) {
  va_list args_copy;
  int len = 0;
  int written = 0;

  if (!fmt) {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  va_copy(args_copy, args);
  len = vsnprintf(NULL, 0U, fmt, args_copy);
  va_end(args_copy);

  if (len < 0) {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_TELEMETRY_ERROR, empty, 0U, NULL, queue);
  }

  if (len > 512) {
    len = 512;
  }

  char buf[(size_t)len + 1U];
  written = vsnprintf(buf, (size_t)len + 1U, fmt, args);
  if (written < 0) {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_TELEMETRY_ERROR, empty, 0U, NULL, queue);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_TELEMETRY_ERROR, buf, (size_t)written,
                                   NULL, queue);
}

static SedsResult log_error_impl(uint8_t queue, const char *fmt, va_list args) {
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_enter()) return SEDS_BAD_ARG;
#endif
  SedsResult result = log_error_impl_locked(queue, fmt, args);
#ifdef TELEMETRY_ENABLED
  if (!telemetry_router_leave() && result == SEDS_OK) return SEDS_ERR;
#endif
  return result;
}


SedsResult log_error_asynchronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(1U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_synchronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(0U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(1U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(0U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult print_telemetry_error(const int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  (void)error_code;
  return SEDS_OK;
#else
  const int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0) {
    return (SedsResult)need;
  }

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    (void)log_error_asynchronous("Error: seds_error_to_string failed: %d\r\n", (int)res);
  }

  return res;
#endif
}

void die(const char *fmt, ...) {
  char buf[128];
  va_list args;

  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  while (1) {
    printf("FATAL: %s\r\n", buf);
    HAL_Delay(1000);
  }
}
