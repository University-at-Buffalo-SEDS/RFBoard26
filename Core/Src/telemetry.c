// telemetry.c
#include "telemetry.h"
#include "av_bay_underglow.h"
#include "flight_state_cache.h"
#include "sim_network_probe.h"
#include "ota_stream.h"

#include "app_threadx.h"
#include "can_bus.h"
#include "radio.h"
#include "sedsnet_config.h"
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

static uint32_t telemetry_flight_can_id(const uint8_t *bytes, size_t len) {
  return sim_probe_packed_data_type(bytes, len) == (uint32_t)SEDS_DT_HEARTBEAT
             ? TELEMETRY_FLIGHT_HEARTBEAT_CAN_ID
             : TELEMETRY_FLIGHT_CAN_ID;
}
#define TELEMETRY_PENDING_CAN_DEPTH 3U
#define TELEMETRY_PENDING_CAN_MAX_LEN 128U
#define TELEMETRY_ROUTER_RETRY_MS 5000ULL

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
#define RF_RADIO_MAX_FRAME_BYTES 1024U
#define RF_CAN_MAX_FRAME_BYTES 128U
#define RF_SIDE_TRANSPORT_TEMPLATES 4U

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
static uint64_t g_router_retry_after_ms = 0ULL;
static uint8_t g_radio_link_seen = 0U;

extern void telemetry_memory_profile_mark(uint32_t stage);

typedef struct {
  size_t len;
  uint8_t data[TELEMETRY_PENDING_CAN_MAX_LEN];
} TelemetryPendingCanCommand;

static TelemetryPendingCanCommand g_pending_can[TELEMETRY_PENDING_CAN_DEPTH];
static uint8_t g_pending_can_head = 0U;
static uint8_t g_pending_can_tail = 0U;
static uint8_t g_pending_can_count = 0U;
static uint32_t g_pending_can_drops = 0U;

RouterState g_router = {.r = NULL, .created = 0U, .start_time = 0ULL};

/* Exported simulator/HIL health signals. A linked-bay test requires both a
 * remote discovery topology change and a valid SEDSNet network clock. */
volatile uint32_t g_telemetry_discovery_seen = 0U;
volatile uint32_t g_telemetry_timesync_valid = 0U;
volatile uint32_t g_telemetry_network_ready = 0U;
volatile uint32_t g_telemetry_peer_mask = 0U;
volatile uint32_t g_sim_heartbeat_attempts = 0U;
volatile uint32_t g_sim_heartbeat_ok = 0U;
volatile uint32_t g_sim_heartbeat_fail = 0U;
volatile uint32_t g_sim_heartbeat_wire_tx = 0U;
volatile uint32_t g_sim_radio_egress_peer_mask = 0U;
volatile uint32_t g_telemetry_timesync_queued = 0U;

static SedsResult telemetry_send_or_queue_can_packet(const uint8_t *bytes, size_t len);

static uint64_t tx_raw_now_ms_locked(void) {
  const uint32_t ticks32 = (uint32_t)tx_time_get();
  return ((uint64_t)ticks32 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static UNUSED_FUNCTION uint64_t tx_raw_now_ms(void *user) {
  (void)user;
  return tx_raw_now_ms_locked();
}

static uint8_t telemetry_timesync_is_source(void) {
  return (TELEMETRY_TIMESYNC_MASTER_PRIO >= 0) ? 1U : 0U;
}

static uint64_t telemetry_timesync_priority(void) {
  return telemetry_timesync_is_source() ? (uint64_t)TELEMETRY_TIMESYNC_MASTER_PRIO : 0ULL;
}

static uint32_t telemetry_timesync_role(void) {
  return telemetry_timesync_is_source() ? TELEMETRY_TIMESYNC_ROLE_SOURCE
                                        : TELEMETRY_TIMESYNC_ROLE_CONSUMER;
}

static bool telemetry_enqueue_pending_can_command(const uint8_t *bytes, size_t len) {
  if (!bytes || len == 0U || len > TELEMETRY_PENDING_CAN_MAX_LEN) {
    return false;
  }

  if (g_pending_can_count >= TELEMETRY_PENDING_CAN_DEPTH) {
    g_pending_can_head = (uint8_t)((g_pending_can_head + 1U) % TELEMETRY_PENDING_CAN_DEPTH);
    g_pending_can_count--;
    g_pending_can_drops++;
  }

  g_pending_can[g_pending_can_tail].len = len;
  memcpy(g_pending_can[g_pending_can_tail].data, bytes, len);
  g_pending_can_tail = (uint8_t)((g_pending_can_tail + 1U) % TELEMETRY_PENDING_CAN_DEPTH);
  g_pending_can_count++;
  return true;
}

static SedsResult telemetry_send_or_queue_can_packet(const uint8_t *bytes, size_t len) {
  const HAL_StatusTypeDef status =
      can_bus_send_large(bytes, len, telemetry_flight_can_id(bytes, len));
  if (status == HAL_OK) {
    return SEDS_OK;
  }

  return telemetry_enqueue_pending_can_command(bytes, len) ? SEDS_OK : SEDS_IO;
}

void telemetry_retry_pending_can_commands(void) {
  while (g_pending_can_count > 0U) {
    TelemetryPendingCanCommand *cmd = &g_pending_can[g_pending_can_head];
    const HAL_StatusTypeDef status =
        can_bus_send_large(cmd->data, cmd->len,
                           telemetry_flight_can_id(cmd->data, cmd->len));
    if (status != HAL_OK) {
      return;
    }

    g_pending_can_head = (uint8_t)((g_pending_can_head + 1U) % TELEMETRY_PENDING_CAN_DEPTH);
    g_pending_can_count--;
  }
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

  result = seds_router_configure_timesync(
      router, true, telemetry_timesync_role(), telemetry_timesync_priority(),
      (uint64_t)TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS,
      (uint64_t)TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS,
      (uint64_t)TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS);
  if (result != SEDS_OK) {
    return result;
  }


#if SEDS_FIRMWARE_SIM_TEST
  /* The simulator GPS exercises SPI but does not synthesize an NMEA date.
   * Seed only qualification builds so the linked firmware must complete the
   * real SEDSNet time-sync exchange; production still waits for valid GPS. */
  if (telemetry_timesync_is_source() && !g_local_unix_valid) {
    g_local_unix_ms = 1767225600000ULL + tx_raw_now_ms_locked();
    g_local_unix_valid = 1U;
  }
#endif

  return telemetry_apply_local_unix_time_locked(router);
}

uint64_t telemetry_now_ms(void) { return tx_raw_now_ms_locked(); }

uint64_t telemetry_unix_ms(void) {
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

uint64_t telemetry_unix_s(void) { return telemetry_unix_ms() / 1000ULL; }

uint8_t telemetry_unix_is_valid(void) { return telemetry_unix_ms() != 0ULL ? 1U : 0U; }

void telemetry_set_unix_time_ms(uint64_t unix_ms) {
  g_local_unix_ms = unix_ms;
  g_local_unix_valid = (unix_ms != 0ULL) ? 1U : 0U;

#ifdef TELEMETRY_ENABLED
  if (g_router.r != NULL) {
    (void)telemetry_apply_local_unix_time_locked(g_router.r);
  }
#endif
}

static uint64_t node_now_since_ms(void *user) {
  (void)user;
  const RouterState s = g_router;
  const uint64_t now = tx_raw_now_ms_locked();
  return s.r ? (now - s.start_time) : 0ULL;
}

SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }
  sim_probe_observe_can_tx(bytes, len);

  return telemetry_send_or_queue_can_packet(bytes, len);
}

static SedsResult radio_tx_send(const uint8_t *bytes, size_t len,
                                uint8_t priority, void *user) {
  (void)user;
  HAL_StatusTypeDef status;

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }

#ifdef SEDS_FIRMWARE_SIM_TEST
  g_sim_radio_egress_peer_mask |= sim_probe_peer_bit_packed(bytes, len);
#endif

  status = radio_uart_send_bytes_priority(bytes, len, priority);
  return (status == HAL_OK) ? SEDS_OK : SEDS_IO;
}

static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  sim_probe_observe_packed(data, len);

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

static void telemetry_radio_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
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

void rx_asynchronous(const uint8_t *bytes, size_t len) {
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

static UNUSED_FUNCTION void rx_synchronous(const uint8_t *bytes, size_t len) {
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

static void telemetry_update_network_health(SedsRouter *router) {
  uint64_t network_time_ms = 0ULL;
  if (seds_router_get_network_time_ms(router, &network_time_ms) == SEDS_OK) {
    g_telemetry_timesync_valid = 1U;
  }
  if (g_telemetry_discovery_seen != 0U && g_radio_link_seen != 0U &&
      g_telemetry_timesync_valid != 0U) {
    g_telemetry_network_ready = 1U;
  }
}

SedsResult telemetry_poll_timesync(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  bool did_queue = false;
  (void)flight_state_cache_poll(g_router.r);
  const SedsResult result = seds_router_poll_timesync(g_router.r, &did_queue);
  if (did_queue) {
    g_telemetry_timesync_queued++;
  }
  telemetry_update_network_health(g_router.r);
  return result;
#endif
}

SedsResult telemetry_announce_discovery(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_announce_discovery(g_router.r);
#endif
}

SedsResult telemetry_poll_discovery(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  bool did_queue = false;
  const SedsResult result = seds_router_poll_discovery(g_router.r, &did_queue);
  if (result == SEDS_OK) {
    sim_probe_emit_heartbeat(g_router.r, telemetry_now_ms());
    (void)av_bay_underglow_poll(g_router.r);
  }
  telemetry_update_network_health(g_router.r);
  return result;
#endif
}

SedsResult init_telemetry_router(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  SedsRouter *r = NULL;
  SedsResult result = SEDS_OK;

  if (g_router.created && g_router.r) {
    return SEDS_OK;
  }

  const uint64_t init_now_ms = tx_raw_now_ms_locked();
  if (g_router_retry_after_ms != 0ULL && init_now_ms < g_router_retry_after_ms) {
    return SEDS_ERR;
  }
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
  r = seds_router_new_with_memory(Seds_RM_Relay, node_now_since_ms, NULL, NULL, 0U,
                                  SEDS_ROUTER_E2E_DISABLED, 0U, &memory);
  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;
    g_router_retry_after_ms = init_now_ms + TELEMETRY_ROUTER_RETRY_MS;
    return SEDS_ERR;
  }
  telemetry_memory_profile_mark(1U);

  g_can_side_id = seds_router_add_side_packed_profile(
      r, "can", 3U, tx_send, NULL, false,
      SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE, RF_CAN_MAX_FRAME_BYTES, 0U,
      RF_SIDE_TRANSPORT_TEMPLATES);
  if (g_can_side_id < 0) {
    printf("Error: failed to add CAN side: %ld\r\n", (long)g_can_side_id);
    g_can_side_id = -1;
  }
  telemetry_memory_profile_mark(2U);

  result = telemetry_configure_timesync_locked(r);
  if (result != SEDS_OK) {
    printf("Error: failed to configure telemetry timesync: %d\r\n", (int)result);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;
    g_router_retry_after_ms = init_now_ms + TELEMETRY_ROUTER_RETRY_MS;
    return result;
  }
  telemetry_memory_profile_mark(4U);

  result = ota_stream_init(r);
  if (result != SEDS_OK) {
    printf("Error: failed to bind OTA stream: %d\r\n", (int)result);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_router_retry_after_ms = init_now_ms + TELEMETRY_ROUTER_RETRY_MS;
    return result;
  }
  telemetry_memory_profile_mark(5U);

  result = av_bay_underglow_init(r);
  if (result != SEDS_OK) {
    seds_router_free(r);
    g_router_retry_after_ms = init_now_ms + TELEMETRY_ROUTER_RETRY_MS;
    return result;
  }

  /* Discovery begins from the normal poll loop after link startup. */
  telemetry_memory_profile_mark(6U);

  g_router.r = r;
  (void)flight_state_cache_init(r);
  g_router.created = 1U;
  /* Prime the first source announcement after the router clock and sides are
   * fully live.  This avoids depending on a later scheduler tick to make a
   * newly booted bay discover a valid network clock. */
  {
    bool did_queue = false;
    result = seds_router_poll_timesync(r, &did_queue);
    if (result != SEDS_OK && result != SEDS_IO) {
      seds_router_free(r);
      g_router.r = NULL;
      g_router.created = 0U;
      g_can_side_id = -1;
      g_radio_side_id = -1;
      g_router_retry_after_ms = init_now_ms + TELEMETRY_ROUTER_RETRY_MS;
      return result;
    }
    if (result == SEDS_OK && did_queue) g_telemetry_timesync_queued++;
  }

  /* Add the ground-radio route only after the first CAN time-source
   * announcement has been emitted.  Otherwise route selection can consume
   * the startup control item on radio before the avionics CAN peers see it. */
  /* Discovery topology can exceed one E22 frame. Let SEDSNet split/reassemble
   * those packets instead of rejecting them at the radio framing boundary. */
  g_radio_side_id = seds_router_add_side_packed_profile_with_priority(
      r, "radio", 5U, radio_tx_send, NULL, true,
      SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE, RF_RADIO_MAX_FRAME_BYTES, 0U,
      RF_SIDE_TRANSPORT_TEMPLATES);
  if (g_radio_side_id < 0) {
    printf("Error: failed to add radio side: %ld\r\n", (long)g_radio_side_id);
    g_radio_side_id = -1;
  }
  if (g_can_side_id < 0 || g_radio_side_id < 0) {
    printf("Error: failed to configure CAN/radio relay sides\r\n");
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;
    g_router_retry_after_ms = init_now_ms + TELEMETRY_ROUTER_RETRY_MS;
    return SEDS_ERR;
  }
  telemetry_memory_profile_mark(3U);
  g_router_retry_after_ms = 0ULL;
  return SEDS_OK;
#endif
}

static inline SedsElemKind guess_kind_from_elem_size(size_t elem_size) {
  if (elem_size == 4U || elem_size == 8U) {
    return SEDS_EK_FLOAT;
  }
  return SEDS_EK_UNSIGNED;
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
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

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
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
  return seds_router_log_typed(g_router.r, data_type, data, element_count, element_size,
                               guess_kind_from_elem_size(element_size));
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_string_asynchronous(SedsDataType data_type, const char *str) {
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

SedsResult dispatch_tx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
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

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
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

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
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

static SedsResult log_error_impl(uint8_t queue, const char *fmt, va_list args) {
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
