// telemetry.c  (side-aware routing via 2 router sides)

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

static uint8_t g_can_rx_subscribed = 0;
static uint8_t g_radio_rx_subscribed = 0;

/* Side IDs assigned by seds_router_add_side_* */
static int32_t g_side_can = -1;
static int32_t g_side_radio = -1;

/* ---------------- Time helpers: 32->64 extender ---------------- */
static uint64_t stm_now_ms(void *user) {
  (void)user;
  static uint32_t last32 = 0;
  static uint64_t high = 0;
  uint32_t cur32 = HAL_GetTick();
  if (cur32 < last32) {
    high += (1ULL << 32);
  }
  last32 = cur32;
  return high | (uint64_t)cur32;
}

uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = stm_now_ms(NULL);
  const RouterState s = g_router;
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

/* ---------------- TX helpers ---------------- */

static SedsResult send_radio_bytes(const uint8_t *bytes, size_t len) {
  return (radio_uart_send_bytes(bytes, len) == HAL_OK) ? SEDS_OK : SEDS_ERR;
}

static SedsResult send_can_bytes(const uint8_t *bytes, size_t len) {
  return (can_bus_send_large(bytes, len, 0x03) == HAL_OK) ? SEDS_OK : SEDS_ERR;
}

/* NEW API TX callbacks per-side */
static SedsResult tx_send_can(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  return send_can_bytes(bytes, len);
}

static SedsResult tx_send_radio(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  return send_radio_bytes(bytes, len);
}

/* telemetry.h still declares tx_send(); keep it for compatibility.
   With side-aware routing it’s not used by the router anymore. */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  /* Safe fallback: broadcast to both */
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

/* telemetry.h API: generic async RX (no side info available here)
   -> treat as "internal/unknown": enqueue without side tagging. */
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

  /* Subscribe exactly once */
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

  /* Local endpoints (optional). Keep your handler wiring. */
  const SedsLocalEndpointDesc locals[] = {
  };

  SedsRouter *r = seds_router_new(
      Seds_RM_Relay,
      node_now_since_ms,
      NULL,
      locals,
      (size_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    return SEDS_ERR;
  }

  /* Add two sides: CAN and RADIO */
  g_side_can = seds_router_add_side_serialized(
      r,
      "can", 3,
      tx_send_can,
      NULL,
      false);

  if (g_side_can < 0) {
    printf("Error: add CAN side failed (%d)\r\n", (int)g_side_can);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0;
    return (SedsResult)g_side_can;
  }

  g_side_radio = seds_router_add_side_serialized(
      r,
      "radio", 5,
      tx_send_radio,
      NULL,
      false);

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
  g_router.start_time = stm_now_ms(NULL);

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
