// telemetry.c
#include "telemetry.h"

#include "app_threadx.h" // brings in tx_api.h usually
#include "can_bus.h"
#include "sedsprintf.h"
#include "stm32g4xx_hal.h"

#include <stdarg.h>
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

static uint8_t g_can_rx_subscribed = 0;
static int32_t g_can_side_id = -1;

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

/* ---------------- ThreadX clock helpers (32->64 extender) ---------------- */
static uint64_t tx_raw_now_ms(void *user) {
  (void)user;

  static uint32_t last_ticks32 = 0;
  static uint64_t high = 0;

  uint32_t cur32 = (uint32_t)tx_time_get();
  if (cur32 < last_ticks32) {
    high += (1ULL << 32);
  }
  last_ticks32 = cur32;

  uint64_t ticks64 = high | (uint64_t)cur32;
  return (ticks64 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

/* ---------------- Time sync state (software-only; does NOT affect ThreadX scheduling) ----------------
 *
 * telemetry_now_ms()  = tx_raw_now_ms() + g_master_offset_ms
 * telemetry_unix_ms() = telemetry_now_ms() + g_unix_base_ms   (if valid)
 *
 * Master (RF/GPS board):
 *  - g_master_offset_ms stays 0 (it IS the master)
 *  - telemetry_set_unix_time_ms() updates g_unix_base_ms from GPS unix
 *  - responds to TIME_SYNC_REQUEST packets
 *  - announces unix time periodically
 *
 * Client boards:
 *  - g_master_offset_ms is adjusted from TIME_SYNC_RESPONSE (NTP math)
 *  - g_unix_base_ms is learned from TIME_SYNC_ANNOUNCE from master
 */
static volatile int64_t  g_master_offset_ms = 0;   // ms
static volatile uint64_t g_last_delay_ms    = 0;   // ms (from last NTP response)
static volatile int64_t  g_unix_base_ms     = 0;   // ms
static volatile uint8_t  g_unix_valid       = 0;

static inline int64_t telemetry_master_offset_ms_get(void) {
  return (int64_t)g_master_offset_ms;
}
static inline void telemetry_master_offset_ms_set(int64_t off) {
  g_master_offset_ms = off;
}
static inline uint64_t telemetry_last_delay_ms_get(void) {
  return (uint64_t)g_last_delay_ms;
}
static inline void telemetry_last_delay_ms_set(uint64_t d) {
  g_last_delay_ms = d;
}

/* Public helpers */
uint64_t telemetry_now_ms(void) {
  int64_t t = (int64_t)tx_raw_now_ms(NULL) + telemetry_master_offset_ms_get();
  if (t < 0) t = 0;
  return (uint64_t)t;
}

uint64_t telemetry_unix_ms(void) {
  if (!g_unix_valid) return 0;
  int64_t t = (int64_t)telemetry_now_ms() + (int64_t)g_unix_base_ms;
  if (t < 0) t = 0;
  return (uint64_t)t;
}

uint64_t telemetry_unix_s(void) {
  return telemetry_unix_ms() / 1000ULL;
}

uint8_t telemetry_unix_is_valid(void) {
  return g_unix_valid;
}

void telemetry_set_unix_time_ms(uint64_t unix_ms) {
#if TELEMETRY_TIME_MASTER
  const int64_t now = (int64_t)telemetry_now_ms();
  g_unix_base_ms = (int64_t)unix_ms - now;
  g_unix_valid = 1;
#else
  (void)unix_ms;
#endif
}

static uint64_t node_now_since_ms(void *user);

/* ---------------- NTP math ---------------- */
static void compute_offset_delay(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4,
                                 int64_t *offset_ms, uint64_t *delay_ms) {
  const int64_t o = ((int64_t)(t2 - t1) + (int64_t)(t3 - t4)) / 2;
  const int64_t d = (int64_t)(t4 - t1) - (int64_t)(t3 - t2);
  *offset_ms = o;
  *delay_ms = (d < 0) ? 0 : (uint64_t)d;
}

#ifndef NET_TIMESYNC_MAX_STEP_MS
#define NET_TIMESYNC_MAX_STEP_MS 30000
#endif

#ifndef NET_TIMESYNC_SMOOTH_DIV
#define NET_TIMESYNC_SMOOTH_DIV 4
#endif

static UNUSED_FUNCTION void client_apply_offset_ms(int64_t offset_ms) {
  if (offset_ms > NET_TIMESYNC_MAX_STEP_MS || offset_ms < -NET_TIMESYNC_MAX_STEP_MS) {
    return;
  }

  // Smooth to avoid jitter
  int64_t step = offset_ms / (int64_t)NET_TIMESYNC_SMOOTH_DIV;
  if (step == 0) {
    if (offset_ms > 0) step = 1;
    else if (offset_ms < 0) step = -1;
  }

  g_master_offset_ms += step;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

/* ---------------- TX helpers ---------------- */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  if (!bytes || len == 0) return SEDS_BAD_ARG;
  return (can_bus_send_large(bytes, len, 0x03) == HAL_OK) ? SEDS_OK : SEDS_IO;
}

/* ---------------- Local endpoint handler(s) ---------------- */
SedsResult on_sd_packet(const SedsPacketView *pkt, void *user) {
  (void)user;
  (void)pkt;
  return SEDS_OK;
}

/* ---------------- Time sync endpoint ----------------
 *
 * Handles:
 *  - TIME_SYNC_RESPONSE (clients): compute offset and update g_master_offset_ms
 *  - TIME_SYNC_REQUEST  (master): reply with [seq, t1, t2, t3]
 *  - TIME_SYNC_ANNOUNCE (clients): learn unix_ms base
 *
 * NOTE:
 * This endpoint only updates *software* time. It does NOT affect ThreadX scheduling.
 */
static SedsResult on_timesync(const SedsPacketView *pkt, void *user) {
  (void)user;
  if (!pkt || !pkt->payload) return SEDS_ERR;

  // ---------- Client: handle response ----------
  if (pkt->ty == SEDS_DT_TIME_SYNC_RESPONSE && pkt->payload_len >= 32) {
    uint64_t seq = 0, t1 = 0, t2 = 0, t3 = 0;
    memcpy(&seq, pkt->payload + 0, 8);
    memcpy(&t1,  pkt->payload + 8, 8);
    memcpy(&t2,  pkt->payload + 16, 8);
    memcpy(&t3,  pkt->payload + 24, 8);

    const uint64_t t4 = tx_raw_now_ms(NULL);

    int64_t offset_ms = 0;
    uint64_t delay_ms = 0;
    compute_offset_delay(t1, t2, t3, t4, &offset_ms, &delay_ms);

#if !TELEMETRY_TIME_MASTER
    client_apply_offset_ms(offset_ms);
#endif
    telemetry_last_delay_ms_set(delay_ms);
    (void)seq;
    return SEDS_OK;
  }

  // ---------- Master: handle request ----------
  if (pkt->ty == SEDS_DT_TIME_SYNC_REQUEST && pkt->payload_len >= 16) {
#if TELEMETRY_TIME_MASTER
    uint64_t seq = 0, t1 = 0;
    memcpy(&seq, pkt->payload + 0, 8);
    memcpy(&t1,  pkt->payload + 8, 8);

    // t2: time at receive (master local base)
    const uint64_t t2 = tx_raw_now_ms(NULL);

    // Optional: if you do real work here, set t3 right before sending.
    const uint64_t t3 = tx_raw_now_ms(NULL);

    const uint64_t resp[4] = {seq, t1, t2, t3};

    // Timestamp the packet at t3 (master local base)
    // Router will relay/broadcast it; clients match seq and compute offset.
    return seds_router_log_ts(g_router.r, SEDS_DT_TIME_SYNC_RESPONSE, t3, resp, 4);
#else
    return SEDS_OK;
#endif
  }

  // ---------- Client: learn unix time from announce ----------
  // announce payload:
  //   [priority, unix_ms]
  if (pkt->ty == SEDS_DT_TIME_SYNC_ANNOUNCE && pkt->payload_len >= 16) {
#if !TELEMETRY_TIME_MASTER
    uint64_t priority = 0;
    uint64_t unix_ms  = 0;
    memcpy(&priority, pkt->payload + 0, 8);
    memcpy(&unix_ms,  pkt->payload + 8, 8);

    // Half-RTT correction from last response (best-effort)
    const uint64_t half_delay = telemetry_last_delay_ms_get() / 2ULL;

    // Set base so telemetry_unix_ms() matches
    const int64_t now = (int64_t)telemetry_now_ms();
    g_unix_base_ms = (int64_t)(unix_ms + half_delay) - now;
    g_unix_valid = 1;

    (void)priority;
#endif
    return SEDS_OK;
  }

  return SEDS_OK;
}

/* ---------------- Router timebase ---------------- */
static uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = telemetry_now_ms();
  const RouterState s = g_router;
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- RX helpers ---------------- */
static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  rx_asynchronous(data, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0) return;

  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
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
  if (!bytes || len == 0) return;

  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_receive_serialized_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  } else {
    (void)seds_router_receive_serialized(g_router.r, bytes, len);
  }
#endif
}

/* ---------------- Time sync request/announce ---------------- */
static UNUSED_FUNCTION uint64_t g_timesync_seq = 1;

SedsResult telemetry_timesync_request(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
#if TELEMETRY_TIME_MASTER
  // Master doesn't request.
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }

  const uint64_t t1 = tx_raw_now_ms(NULL);
  const uint64_t req[2] = { g_timesync_seq++, t1 };

  return seds_router_log_ts(g_router.r, SEDS_DT_TIME_SYNC_REQUEST, t1, req, 2);
#endif
#endif
}

SedsResult telemetry_timesync_announce(uint64_t priority, uint64_t unix_ms) {
#ifndef TELEMETRY_ENABLED
  (void)priority;
  (void)unix_ms;
  return SEDS_OK;
#else
#if !TELEMETRY_TIME_MASTER
  (void)priority;
  (void)unix_ms;
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }

  // Announce unix_ms from master (and priority for master election if you want it)
  const uint64_t t = tx_raw_now_ms(NULL);
  const uint64_t announce[2] = {priority, unix_ms};

  return seds_router_log_ts(g_router.r, SEDS_DT_TIME_SYNC_ANNOUNCE, t, announce, 2);
#endif
#endif
}

/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (g_router.created && g_router.r) return SEDS_OK;

  if (!g_can_rx_subscribed) {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK) {
      g_can_rx_subscribed = 1;
    } else {
      printf("Error: can_bus_subscribe_rx failed\r\n");
    }
  }

  const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = (uint32_t)SEDS_EP_SD_CARD,
          .packet_handler = on_sd_packet,
          .serialized_handler = NULL,
          .user = NULL,
      },
      {
          .endpoint = (uint32_t)SEDS_EP_TIME_SYNC,
          .packet_handler = on_timesync,
          .serialized_handler = NULL,
          .user = NULL,
      },
  };

  SedsRouter *r = seds_router_new(
      // Master should be relay too (so it forwards non-local packets),
      // unless you truly want it to sink everything.
      Seds_RM_Relay,
      node_now_since_ms,
      NULL,
      locals,
      (size_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  g_can_side_id = seds_router_add_side_serialized(
      r, "can", 3, tx_send, NULL, false);

  if (g_can_side_id < 0) {
    printf("Error: failed to add CAN side: %ld\r\n", (long)g_can_side_id);
    g_can_side_id = -1;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = telemetry_now_ms();

#if TELEMETRY_TIME_MASTER
  // master offset stays 0
  g_master_offset_ms = 0;
#endif

  return SEDS_OK;
#endif
}

/* ---------------- Logging APIs ---------------- */
static inline SedsElemKind guess_kind_from_elem_size(size_t elem_size) {
  if (elem_size == 4 || elem_size == 8) return SEDS_EK_FLOAT;
  return SEDS_EK_UNSIGNED;
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0) return SEDS_BAD_ARG;

  const SedsElemKind kind = guess_kind_from_elem_size(element_size);

  return seds_router_log_typed_ex(g_router.r, data_type, data, element_count,
                                 element_size, kind, NULL, 0);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0) return SEDS_BAD_ARG;

  const SedsElemKind kind = guess_kind_from_elem_size(element_size);

  return seds_router_log_typed_ex(g_router.r, data_type, data, element_count,
                                 element_size, kind, NULL, 1);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

/* ---------------- Queue processing ---------------- */
SedsResult dispatch_tx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
#endif
}

/* ---------------- Error logging ---------------- */
SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }

  va_list args;
  va_start(args, fmt);

  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy);
  va_end(args_copy);

  if (len < 0) {
    va_end(args);
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 1);
  }

  if (len > 512) len = 512;

  char buf[(size_t)len + 1];
  int written = vsnprintf(buf, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 1);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, buf, (size_t)written, NULL, 1);
#endif
}

SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }

  va_list args;
  va_start(args, fmt);

  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy);
  va_end(args_copy);

  if (len < 0) {
    va_end(args);
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 0);
  }

  if (len > 512) len = 512;

  char buf[(size_t)len + 1];
  int written = vsnprintf(buf, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 0);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, buf, (size_t)written, NULL, 0);
#endif
}

SedsResult print_telemetry_error(const int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  (void)error_code;
  return SEDS_OK;
#else
  const int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0) return (SedsResult)need;

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    (void)log_error_asyncronous("Error: seds_error_to_string failed: %d\r\n", (int)res);
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
