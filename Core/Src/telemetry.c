// telemetry.c  (side-aware routing via 2 router sides)
//
// UPDATED (GPS-correct time):
// - Router timestamps use "correct time" when available:
//      correct_now_ms = tx_raw_now_ms + gps_offset_ms
// - GPS offset is produced by neom9n_thread.c:
//      extern volatile int64_t  g_gps_time_offset_ms;
//      extern volatile uint32_t g_gps_time_offset_valid;
// - If GPS is NOT valid, we can still apply TIME_SYNC_RESPONSE by updating a
//   software network offset (no tx_time_set; RTOS tick scheduling stays stable).
//
// NOTE:
// "Correct time" here means: a monotonic ms counter aligned to GPS time-of-day
// (no epoch/date unless you add GPS date).

#include "telemetry.h"
#include "app_threadx.h"
#include "main.h"
#include "sedsprintf.h"
#include "stm32g4xx_hal.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32g4xx_hal_fdcan.h>
#include <stm32g4xx_hal_uart.h>
#include <string.h>
#include "can_bus.h"
#include "radio.h"

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len) {
  (void)data;
  (void)len;
}
#endif

// From neom9n_thread.c (GPS-derived offset)
extern volatile int64_t  g_gps_time_offset_ms;
extern volatile uint32_t g_gps_time_offset_valid;

static uint8_t g_can_rx_subscribed = 0;
static uint8_t g_radio_rx_subscribed = 0;

/* Side IDs assigned by seds_router_add_side_* */
static int32_t g_side_can = -1;
static int32_t g_side_radio = -1;

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

/* ---------------- Software network offset (fallback when GPS isn't valid) ----------------
 *
 * Boards without GPS can still timesync via packets; we apply a software offset
 * instead of tx_time_set() to avoid disturbing ThreadX scheduling.
 */
static volatile int64_t g_net_time_offset_ms = 0;

/* "Correct" time used by the router:
 * - If GPS valid: tx_raw_now_ms + gps_offset
 * - else:         tx_raw_now_ms + net_offset
 */
static uint64_t correct_now_ms(void) {
  const uint64_t base = tx_raw_now_ms(NULL);

  if (g_gps_time_offset_valid) {
    const int64_t off = g_gps_time_offset_ms;
    if (off >= 0) return base + (uint64_t)off;
    const uint64_t sub = (uint64_t)(-off);
    return (sub > base) ? 0 : (base - sub);
  } else {
    const int64_t off = g_net_time_offset_ms;
    if (off >= 0) return base + (uint64_t)off;
    const uint64_t sub = (uint64_t)(-off);
    return (sub > base) ? 0 : (base - sub);
  }
}

/* Router callback: milliseconds since router start_time */
uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = correct_now_ms();
  const RouterState s = g_router;
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

/* ---------------- Time sync (NTP-style offset/delay) ----------------
 *
 * We DO NOT call tx_time_set() anymore.
 * Instead, when GPS is not valid, we update g_net_time_offset_ms.
 *
 * Payload (u64):
 *   resp[0]=seq, resp[1]=t1, resp[2]=t2, resp[3]=t3
 * and we capture t4 locally on receive.
 */
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

static void net_apply_offset_ms(int64_t offset_ms) {
  // Ignore wild jumps
  if (offset_ms > NET_TIMESYNC_MAX_STEP_MS || offset_ms < -NET_TIMESYNC_MAX_STEP_MS) {
    return;
  }

  // Smooth to avoid jitter
  int64_t step = offset_ms / (int64_t)NET_TIMESYNC_SMOOTH_DIV;
  if (step == 0) {
    if (offset_ms > 0) step = 1;
    else if (offset_ms < 0) step = -1;
  }

  g_net_time_offset_ms += step;
}

static SedsResult on_timesync(const SedsPacketView *pkt, void *user) {
  (void)user;
  if (!pkt || !pkt->payload) return SEDS_ERR;

  if (pkt->ty == SEDS_DT_TIME_SYNC_RESPONSE && pkt->payload_len >= 32) {
    uint64_t seq = 0, t1 = 0, t2 = 0, t3 = 0;
    memcpy(&seq, pkt->payload + 0, 8);
    memcpy(&t1,  pkt->payload + 8, 8);
    memcpy(&t2,  pkt->payload + 16, 8);
    memcpy(&t3,  pkt->payload + 24, 8);

    // capture receive time locally (in our local base time)
    const uint64_t t4 = tx_raw_now_ms(NULL);

    int64_t offset_ms = 0;
    uint64_t delay_ms = 0;
    compute_offset_delay(t1, t2, t3, t4, &offset_ms, &delay_ms);

    // If GPS is valid, GPS is the authority on this board; ignore network corrections.
    if (!g_gps_time_offset_valid) {
      net_apply_offset_ms(offset_ms);
    }

    // Optional debug:
    // printf("timesync seq=%llu offset_ms=%lld delay_ms=%llu gps_valid=%lu\r\n",
    //        (unsigned long long)seq, (long long)offset_ms,
    //        (unsigned long long)delay_ms, (unsigned long)g_gps_time_offset_valid);
  }

  return SEDS_OK;
}

/* ---------------- TX helpers ---------------- */

static SedsResult send_radio_bytes(const uint8_t *bytes, size_t len) {
  return (radio_uart_send_bytes(bytes, len) == HAL_OK) ? SEDS_OK : SEDS_ERR;
}

static SedsResult send_can_bytes(const uint8_t *bytes, size_t len) {
  return (can_bus_send_large(bytes, len, 0x03) == HAL_OK) ? SEDS_OK : SEDS_ERR;
}

static SedsResult tx_send_can(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  if (!bytes || !len) return SEDS_BAD_ARG;
  return send_can_bytes(bytes, len);
}

static SedsResult tx_send_radio(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  if (!bytes || !len) return SEDS_BAD_ARG;
  return send_radio_bytes(bytes, len);
}

/* telemetry.h still declares tx_send(); keep it for compatibility. */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  if (!bytes || !len) return SEDS_BAD_ARG;

  if (send_can_bytes(bytes, len) != SEDS_OK) return SEDS_ERR;
  if (send_radio_bytes(bytes, len) != SEDS_OK) return SEDS_ERR;
  return SEDS_OK;
}

/* ---------------- RX helpers ---------------- */

static void telemetry_radio_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (!data || !len) return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
  }
  if (g_side_radio < 0) return;

  (void)seds_router_rx_serialized_packet_to_queue_from_side(
      g_router.r, (uint32_t)g_side_radio, data, len);
}

static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (!data || !len) return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
  }
  if (g_side_can < 0) return;

  (void)seds_router_rx_serialized_packet_to_queue_from_side(
      g_router.r, (uint32_t)g_side_can, data, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
  if (!bytes || !len) return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
  }
  (void)seds_router_rx_serialized_packet_to_queue(g_router.r, bytes, len);
}

/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif

  if (g_router.created && g_router.r) return SEDS_OK;

  if (!g_can_rx_subscribed) {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK) {
      g_can_rx_subscribed = 1;
    } else {
      printf("Error: can_bus_subscribe_rx failed\r\n");
    }
  }
  if (!g_radio_rx_subscribed) {
    if (radio_uart_subscribe_rx(telemetry_radio_rx, NULL) == HAL_OK) {
      g_radio_rx_subscribed = 1;
    } else {
      printf("Error: radio_uart_subscribe_rx failed\r\n");
    }
  }

  const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = (uint32_t)SEDS_EP_TIME_SYNC,
          .packet_handler = on_timesync,
          .serialized_handler = NULL,
          .user = NULL,
      },
  };

  SedsRouter *r = seds_router_new(
      Seds_RM_Relay,
      node_now_since_ms, // uses correct_now_ms()
      NULL,
      locals,
      (size_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    return SEDS_ERR;
  }

  g_side_can = seds_router_add_side_serialized(
      r, "can", 3, tx_send_can, NULL, false);

  if (g_side_can < 0) {
    printf("Error: add CAN side failed (%d)\r\n", (int)g_side_can);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0;
    return (SedsResult)g_side_can;
  }

  g_side_radio = seds_router_add_side_serialized(
      r, "radio", 5, tx_send_radio, NULL, false);

  if (g_side_radio < 0) {
    printf("Error: add RADIO side failed (%d)\r\n", (int)g_side_radio);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0;
    g_side_can = -1;
    return (SedsResult)g_side_radio;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = correct_now_ms();

  return SEDS_OK;
}

/* ---------------- Logging (unchanged API) ---------------- */

static SedsElemKind infer_kind_from_elem_size(size_t elem_size) {
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

  int32_t expected = seds_dtype_expected_size(data_type);
  if (expected < 0) return (SedsResult)expected;

  size_t total_bytes = element_count * element_size;
  if ((int32_t)total_bytes != expected) return SEDS_SIZE_MISMATCH;

  return seds_router_log_typed_ex(g_router.r,
                                 data_type,
                                 data,
                                 element_count,
                                 element_size,
                                 infer_kind_from_elem_size(element_size),
                                 NULL,
                                 0);
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

  int32_t expected = seds_dtype_expected_size(data_type);
  if (expected < 0) return (SedsResult)expected;

  size_t total_bytes = element_count * element_size;
  if ((int32_t)total_bytes != expected) return SEDS_SIZE_MISMATCH;

  return seds_router_log_typed_ex(g_router.r,
                                 data_type,
                                 data,
                                 element_count,
                                 element_size,
                                 infer_kind_from_elem_size(element_size),
                                 NULL,
                                 1);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

/* ---------------- Queue processing (unchanged) ---------------- */

SedsResult dispatch_tx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_tx_queue(g_router.r);
}

SedsResult process_rx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_rx_queue(g_router.r);
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
}

/* IMPORTANT: announce uses correct time now */
SedsResult telemetry_timesync_announce(uint64_t priority)
{
#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }

  const uint64_t t = correct_now_ms();
  const uint64_t announce[2] = {priority, t};

  return seds_router_log_typed_ex(g_router.r,
                                 SEDS_DT_TIME_SYNC_ANNOUNCE,
                                 announce,
                                 2,
                                 sizeof(uint64_t),
                                 SEDS_EK_UNSIGNED,
                                 &t,
                                 0);
#else
  (void)priority;
  return SEDS_OK;
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
}

/* ---------------- Error logging (string-safe) ---------------- */

SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
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
    return SEDS_ERR;
  }

  if (len > 512) len = 512;

  char msg[(size_t)len + 1];
  int written = vsnprintf(msg, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) return SEDS_ERR;

  return seds_router_log_string_ex(g_router.r,
                                  SEDS_DT_GENERIC_ERROR,
                                  msg,
                                  (size_t)written,
                                  NULL,
                                  1);
}

SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
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
    return SEDS_ERR;
  }

  if (len > 512) len = 512;

  char msg[(size_t)len + 1];
  int written = vsnprintf(msg, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) return SEDS_ERR;

  return seds_router_log_string_ex(g_router.r,
                                  SEDS_DT_GENERIC_ERROR,
                                  msg,
                                  (size_t)written,
                                  NULL,
                                  0);
}

/* ---------------- Error printing ---------------- */

SedsResult print_telemetry_error(const int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0) return (SedsResult)need;

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    (void)log_error_asyncronous("Error: seds_error_to_string failed: %d\r\n", (int)res);
  }
  return res;
}

/* ---------------- Fatal helper ---------------- */
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
