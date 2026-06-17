// telemetry.c
#include "telemetry.h"

#include "app_threadx.h"
#include "can_bus.h"
#include "radio.h"
#include "sedsprintf.h"
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

#ifndef TELEMETRY_RADIO_GPS_INTERVAL_MS
#define TELEMETRY_RADIO_GPS_INTERVAL_MS 1000ULL
#endif

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

#define TELEMETRY_TIMESYNC_ROLE_CONSUMER 0U
#define TELEMETRY_TIMESYNC_ROLE_SOURCE 1U
#define TELEMETRY_SEDS_FLAG_COMPRESSED_SENDER 0x02U
#define TELEMETRY_SEDS_CRC_BYTES 4U
#define TELEMETRY_FLIGHT_CAN_ID 0x03U
#define TELEMETRY_PENDING_CAN_DEPTH 8U
#define TELEMETRY_PENDING_CAN_MAX_LEN 256U
#define TELEMETRY_GPS_WIRE_MAX_LEN 96U
#define TELEMETRY_DEVICE_IDENTIFIER "RF"
#define TELEMETRY_DEVICE_IDENTIFIER_LEN 2U

_Static_assert(TELEMETRY_DEVICE_IDENTIFIER_LEN == (sizeof(TELEMETRY_DEVICE_IDENTIFIER) - 1U),
               "Telemetry sender length must match device identifier");

static uint8_t g_can_rx_subscribed = 0U;
static uint8_t g_radio_rx_subscribed = 0U;
static int32_t g_can_side_id = -1;
static int32_t g_radio_side_id = -1;
static uint32_t g_pending_radio_flight_commands = 0U;
static size_t g_pending_radio_flight_command_len = 0U;
static uint32_t g_pending_radio_flight_command_crc = 0U;
static uint8_t g_local_unix_valid = 0U;
static uint64_t g_local_unix_ms = 0ULL;
static uint64_t g_radio_last_gps_tx_ms = 0ULL;
static uint8_t g_radio_gps_tx_seen = 0U;
static volatile uint32_t g_radio_gps_throttle_drops = 0U;

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

static bool telemetry_is_flight_command_packet(const uint8_t *bytes, size_t len) {
  const uint32_t endpoint = (uint32_t)SEDS_EP_FLIGHT_CONTROLLER;
  SedsOwnedHeader *header = NULL;
  SedsPacketView view;
  bool matched = false;

  if (!bytes || len == 0U) {
    return false;
  }

  header = seds_pkt_deserialize_header_owned(bytes, len);
  if (!header) {
    return false;
  }

  memset(&view, 0, sizeof(view));
  if (seds_owned_header_view(header, &view) == SEDS_OK &&
      view.ty == (uint32_t)SEDS_DT_FLIGHT_COMMAND &&
      view.endpoints != NULL) {
    for (size_t i = 0U; i < view.num_endpoints; i++) {
      if (view.endpoints[i] == endpoint) {
        matched = true;
        break;
      }
    }
  }

  seds_owned_header_free(header);
  return matched;
}

static uint32_t telemetry_packet_wire_crc(const uint8_t *bytes, size_t len) {
  if (!bytes || len < TELEMETRY_SEDS_CRC_BYTES) {
    return 0U;
  }

  const size_t offset = len - TELEMETRY_SEDS_CRC_BYTES;
  return (uint32_t)bytes[offset] |
         ((uint32_t)bytes[offset + 1U] << 8U) |
         ((uint32_t)bytes[offset + 2U] << 16U) |
         ((uint32_t)bytes[offset + 3U] << 24U);
}

static void telemetry_note_flight_command_sent(const uint8_t *bytes, size_t len) {
  if (g_pending_radio_flight_commands == 0U ||
      len != g_pending_radio_flight_command_len ||
      telemetry_packet_wire_crc(bytes, len) != g_pending_radio_flight_command_crc) {
    return;
  }

  g_pending_radio_flight_commands--;
}

static bool telemetry_radio_gps_throttle_active(uint64_t now_ms) {
  if (g_radio_gps_tx_seen &&
      (now_ms - g_radio_last_gps_tx_ms) < TELEMETRY_RADIO_GPS_INTERVAL_MS) {
    g_radio_gps_throttle_drops++;
    return true;
  }

  return false;
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

static SedsResult telemetry_serialize_gps_packet(const void *data, size_t element_count,
                                                 size_t element_size, uint8_t *out,
                                                 size_t out_len, size_t *written) {
  const uint32_t endpoints[] = {
      (uint32_t)SEDS_EP_SD_CARD,
      (uint32_t)SEDS_EP_GROUND_STATION,
      (uint32_t)SEDS_EP_FLIGHT_CONTROLLER,
  };
  SedsPacketView view;
  int32_t serialized_len;
  const size_t payload_len = element_count * element_size;

  if (!data || !out || !written || element_count != 3U || element_size != sizeof(float)) {
    return SEDS_BAD_ARG;
  }

  memset(&view, 0, sizeof(view));
  view.ty = (uint32_t)SEDS_DT_GPS_DATA;
  view.data_size = payload_len;
  view.sender = TELEMETRY_DEVICE_IDENTIFIER;
  view.sender_len = TELEMETRY_DEVICE_IDENTIFIER_LEN;
  view.endpoints = endpoints;
  view.num_endpoints = sizeof(endpoints) / sizeof(endpoints[0]);
  view.timestamp = telemetry_unix_ms();
  view.payload = (const uint8_t *)data;
  view.payload_len = payload_len;

  serialized_len = seds_pkt_serialize(&view, out, out_len);
  if (serialized_len < 0) {
    return (SedsResult)serialized_len;
  }
  if ((size_t)serialized_len > out_len) {
    return SEDS_PACKET_TOO_LARGE;
  }

  *written = (size_t)serialized_len;
  return SEDS_OK;
}

static SedsResult telemetry_log_gps_direct(const void *data, size_t element_count,
                                           size_t element_size) {
  uint8_t bytes[TELEMETRY_GPS_WIRE_MAX_LEN];
  size_t len = 0U;
  SedsResult result;
  HAL_StatusTypeDef radio_status;
  const uint64_t now_ms = tx_raw_now_ms_locked();

  result = telemetry_serialize_gps_packet(data, element_count, element_size, bytes,
                                          sizeof(bytes), &len);
  if (result != SEDS_OK) {
    return result;
  }

  result = telemetry_send_or_queue_can_packet(bytes, len);

  if (!telemetry_radio_gps_throttle_active(now_ms)) {
    radio_status = radio_uart_send_bytes(bytes, len);
    if (radio_status == HAL_OK) {
      g_radio_last_gps_tx_ms = now_ms;
      g_radio_gps_tx_seen = 1U;
    } else if (radio_status != HAL_BUSY || radio_uart_tx_ready()) {
      /* CAN is the primary path here; radio backpressure must not fail GPS logging. */
      g_radio_gps_throttle_drops++;
    }
  }

  return result;
}

static HAL_StatusTypeDef telemetry_send_or_queue_can_command(const uint8_t *bytes, size_t len) {
  HAL_StatusTypeDef status;

  if (!bytes || len == 0U) {
    return HAL_ERROR;
  }

  status = can_bus_send_large(bytes, len, TELEMETRY_FLIGHT_CAN_ID);
  if (status == HAL_OK) {
    telemetry_note_flight_command_sent(bytes, len);
    return HAL_OK;
  }

  return telemetry_enqueue_pending_can_command(bytes, len) ? HAL_BUSY : status;
}

static SedsResult telemetry_send_or_queue_can_packet(const uint8_t *bytes, size_t len) {
  const HAL_StatusTypeDef status = can_bus_send_large(bytes, len, TELEMETRY_FLIGHT_CAN_ID);
  if (status == HAL_OK) {
    telemetry_note_flight_command_sent(bytes, len);
    return SEDS_OK;
  }

  return telemetry_enqueue_pending_can_command(bytes, len) ? SEDS_OK : SEDS_IO;
}

void telemetry_retry_pending_can_commands(void) {
  while (g_pending_can_count > 0U) {
    TelemetryPendingCanCommand *cmd = &g_pending_can[g_pending_can_head];
    const HAL_StatusTypeDef status =
        can_bus_send_large(cmd->data, cmd->len, TELEMETRY_FLIGHT_CAN_ID);
    if (status != HAL_OK) {
      return;
    }

    telemetry_note_flight_command_sent(cmd->data, cmd->len);
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
  SedsResult result;
  bool is_radio_flight_command = false;

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }

  is_radio_flight_command =
      (g_pending_radio_flight_commands > 0U) &&
      (len == g_pending_radio_flight_command_len) &&
      (telemetry_packet_wire_crc(bytes, len) == g_pending_radio_flight_command_crc);

  result = telemetry_send_or_queue_can_packet(bytes, len);
#if COMMAND_TRACE_PRINTS
  if (result == SEDS_OK && is_radio_flight_command) {
    printf("Flight computer CAN command TX: id=0x%03lx len=%u\r\n",
           (unsigned long)TELEMETRY_FLIGHT_CAN_ID,
           (unsigned)len);
  }
#else
  (void)is_radio_flight_command;
#endif
  return result;
}

static SedsResult radio_tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  HAL_StatusTypeDef status;

  if (!bytes || len == 0U) {
    return SEDS_BAD_ARG;
  }

  status = radio_uart_send_bytes(bytes, len);
  if (status == HAL_BUSY && !radio_uart_tx_ready()) {
    return SEDS_OK;
  }

  return (status == HAL_OK) ? SEDS_OK : SEDS_IO;
}

static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;

#ifdef TELEMETRY_ENABLED
  if (!data || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r, (uint32_t)g_can_side_id, data, len);
  } else {
    (void)seds_router_rx_serialized_packet_to_queue(g_router.r, data, len);
  }
#else
  (void)data;
  (void)len;
#endif
}

static void telemetry_radio_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;

#ifdef TELEMETRY_ENABLED
  if (!data || len == 0U) {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return;
  }

  if (radio_uart_current_rx_is_command_frame() &&
      telemetry_radio_scheduler_handle_command(data, len)) {
    return;
  }

  const bool is_flight_command = telemetry_is_flight_command_packet(data, len);
#if COMMAND_TRACE_PRINTS
  printf("Radio uplink RX: kind=%s len=%u flight_command=%u preview=",
         radio_uart_current_rx_is_command_frame() ? "command" : "data",
         (unsigned)len,
         is_flight_command ? 1U : 0U);
  for (size_t i = 0U; i < len && i < 12U; i++) {
    printf("%02X%s", data[i], (i + 1U < len && i + 1U < 12U) ? " " : "");
  }
  if (len > 12U) {
    printf(" ...");
  }
  printf("\r\n");
#endif

  if (is_flight_command) {
#if COMMAND_TRACE_PRINTS
    HAL_StatusTypeDef can_status;
    printf("Ground station flight command RX: len=%u\r\n", (unsigned)len);
    can_status = telemetry_send_or_queue_can_command(data, len);
    if (can_status == HAL_OK) {
      printf("Flight computer CAN command TX: id=0x%03lx len=%u\r\n",
             (unsigned long)TELEMETRY_FLIGHT_CAN_ID,
             (unsigned)len);
    } else {
      printf("Flight computer CAN command TX queued/failed: id=0x%03lx len=%u status=%ld pending=%u drops=%lu\r\n",
             (unsigned long)TELEMETRY_FLIGHT_CAN_ID,
             (unsigned)len,
             (long)can_status,
             (unsigned)g_pending_can_count,
             (unsigned long)g_pending_can_drops);
    }
#else
    (void)telemetry_send_or_queue_can_command(data, len);
#endif
    return;
  }

  if (g_radio_side_id >= 0) {
    (void)seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r, (uint32_t)g_radio_side_id, data, len);
  } else {
    (void)seds_router_rx_serialized_packet_to_queue(g_router.r, data, len);
  }
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
    (void)seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  } else {
    (void)seds_router_rx_serialized_packet_to_queue(g_router.r, bytes, len);
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
    (void)seds_router_receive_serialized_from_side(g_router.r, (uint32_t)g_can_side_id, bytes,
                                                   len);
  } else {
    (void)seds_router_receive_serialized(g_router.r, bytes, len);
  }
#endif
}

SedsResult telemetry_poll_timesync(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_poll_timesync(g_router.r, NULL);
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

  return seds_router_poll_discovery(g_router.r, NULL);
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

  r = seds_router_new(Seds_RM_Relay, node_now_since_ms, NULL, NULL, 0U);
  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;
    return SEDS_ERR;
  }

  g_can_side_id = seds_router_add_side_serialized(r, "can", 3U, tx_send, NULL, true);
  if (g_can_side_id < 0) {
    printf("Error: failed to add CAN side: %ld\r\n", (long)g_can_side_id);
    g_can_side_id = -1;
  }

  g_radio_side_id = seds_router_add_side_serialized(r, "radio", 5U, radio_tx_send, NULL, false);
  if (g_radio_side_id < 0) {
    printf("Error: failed to add radio side: %ld\r\n", (long)g_radio_side_id);
    g_radio_side_id = -1;
  }

  result = telemetry_configure_timesync_locked(r);
  if (result != SEDS_OK) {
    printf("Error: failed to configure telemetry timesync: %d\r\n", (int)result);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;
    return result;
  }

  result = seds_router_announce_discovery(r);
  if (result != SEDS_OK) {
    printf("Error: failed to announce discovery: %d\r\n", (int)result);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    g_radio_side_id = -1;
    return result;
  }

  g_router.r = r;
  g_router.created = 1U;
  g_router.start_time = tx_raw_now_ms_locked();
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

  if (data_type == SEDS_DT_GPS_DATA) {
    return telemetry_log_gps_direct(data, element_count, element_size);
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK) {
    return SEDS_ERR;
  }

  return seds_router_log_queue_typed(g_router.r, data_type, data, element_count, element_size,
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

  return seds_router_log_string_ex(g_router.r, data_type, str, strlen(str), NULL, 1);
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
