#include "telemetry.h"
#include "app_threadx.h" // should bring in tx_api.h; if not, include tx_api.h directly
#include "sedsprintf.h"
#include "stm32g4xx_hal.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32g4xx_hal_fdcan.h>
#include <stm32g4xx_hal_uart.h>
#include "main.h"
#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len) {
  // Implementation for debugging telemetry data
}
#endif

extern FDCAN_HandleTypeDef hfdcan2; // FDCAN handle defined in main.c

extern UART_HandleTypeDef huart1; // UART handle defined in main.c

static uint8_t radio_rx_buf[256];


inline void telemetry_radio_rx_start(void)
{
    // Start interrupt-driven reception; callback fires on IDLE or buffer full.
    // Requires: UART interrupt enabled (CubeMX usually does this).
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, radio_rx_buf, sizeof(radio_rx_buf));
}

/* ---------------- Time helpers: 32->64 extender ---------------- */
static uint64_t stm_now_ms(void *user) {
  (void)user;
  static uint32_t last32 = 0;
  static uint64_t high = 0;
  uint32_t cur32 = HAL_GetTick();
  if (cur32 < last32) {
    high += (1ULL << 32); /* 32-bit wrap (~49.7 days) */
  }
  last32 = cur32;
  return high | (uint64_t)cur32;
}

uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = stm_now_ms(NULL);
  const RouterState s = g_router; /* snapshot */
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

/* ---------------- TX helpers ---------------- */
SedsResult send_can_bus(const uint8_t *bytes, size_t len){
  FDCAN2->TXBAR |= (1 << 0); // Set the corresponding bit to request transmission
      //transmit CAN frame
      FDCAN_TxHeaderTypeDef txHeader;
      txHeader.Identifier = 0x03; // Example CAN ID
      txHeader.IdType = FDCAN_STANDARD_ID;
      txHeader.TxFrameType = FDCAN_DATA_FRAME;
      txHeader.DataLength = len;
      txHeader.ErrorStateIndicator = 0;
      txHeader.BitRateSwitch = FDCAN_BRS_OFF;
      txHeader.FDFormat = FDCAN_FD_CAN;
      txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; 
      txHeader.MessageMarker = 0;
      uint8_t txData[64] = {0}; // Max 64 bytes for CAN FD
      memcpy(txData, bytes, len > 64 ? 64 : len);
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK) {
        return SEDS_ERR;
      }
      return SEDS_OK;
}



SedsResult send_radio(const uint8_t *bytes, size_t len){
  //build uart frame
  if (HAL_UART_Transmit(&huart1, (uint8_t *)bytes, len, HAL_MAX_DELAY) != HAL_OK) {
    return SEDS_ERR;
  }
  return SEDS_OK;
}

/* Config setup */
static inline size_t fdcan_dlc_to_len(uint32_t dlc)
{
    // HAL stores DLC as enum values (0..15). This mapping is standard CAN FD.
    static const uint8_t map[16] = {
        0, 1, 2, 3, 4, 5, 6, 7,
        8, 12, 16, 20, 24, 32, 48, 64
    };
    dlc &= 0xF;
    return map[dlc];
}



/* ---------------- TX path (CANSEND) ---------------- */
SedsResult tx_send(const uint8_t *bytes, size_t len, const struct SedsLinkId *link_id, void *user) {
  (void)user;
  (void)bytes;
  (void)len;

  //can bus send
  if (!link_id) {
    return SEDS_ERR;
  }

  switch (link_id->raw) {
    case can_id: // recieved from the CAN bus, send to radio
      return send_radio(bytes, len);

      break;
    case radio_id: // recieved from the radio, send to CAN bus
      return send_can_bus(bytes, len);
      break;
    default: // received from elsewhere (internal or default), send to both
      if (send_can_bus(bytes, len) != SEDS_OK) {
        return SEDS_ERR;
      }
      return send_radio(bytes, len);
      break;
  }
}

/* ---------------------------- Handlers for the interrupts of receiving from uart and can bus ---------------- */

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0) {
        return;
    }

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    // Drain FIFO1 (could be multiple frames pending)
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &hdr, data) != HAL_OK) {
            break;
        }

        size_t len = fdcan_dlc_to_len(hdr.DataLength);

        rx_asynchronous(data, len, &can_link_id);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) {
        return;
    }

    if (Size > 0) {
        // IMPORTANT: radio_rx_buf is static, so it stays valid after return.
        // BUT the next ReceiveToIdle will overwrite it, so rx_asynchronous must
        // either copy immediately OR you should copy/queue before restarting.
        rx_asynchronous(radio_rx_buf, (size_t)Size, &radio_link_id);
    }

    // Re-arm reception for the next chunk
    telemetry_radio_rx_start();
}


/* ---------------- RX helpers ---------------- */
void rx_synchronous(const uint8_t *bytes, size_t len, const SedsLinkId *link_id) {
  if (!bytes || !len)
    return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return;
  }

  seds_router_receive_serialized_v2(g_router.r,link_id, bytes, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len, const SedsLinkId *link_id) {
  if (!bytes || !len)
    return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return;
  }

  seds_router_rx_serialized_packet_to_queue_v2(g_router.r, link_id, bytes, len);
}


/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  /* Fast check without lock to avoid needless acquire in the common case. */
  if (g_router.created && g_router.r)
    return SEDS_OK;

  if (g_router.created && g_router.r) {

    return SEDS_OK;
  }

  
  SedsRouter *r =
      seds_router_new_v2(Seds_RM_Relay,
                      tx_send,           /* tx callback */
                      NULL,              /* tx_user */
                      node_now_since_ms, /* clock */
                      NULL,
                      0);

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;

    return SEDS_ERR;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = stm_now_ms(NULL);

  return SEDS_OK;
}

/* ---------------- Logging APIs ---------------- */
SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count,
                                     size_t element_size) {
#ifdef TELEMETRY_ENABLED

  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_ERR;

  const size_t total_bytes = element_count * element_size;

  SedsResult res = seds_router_log(g_router.r, data_type, data, total_bytes);

  return res;

#else
  (void)data_type;

  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count,
                                      size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_ERR;

  const size_t total_bytes = element_count * element_size;

  SedsResult res =
      seds_router_log_queue(g_router.r, data_type, data, total_bytes);

  return res;
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
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res = seds_router_process_tx_queue(g_router.r);

  return res;
}

SedsResult process_rx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res = seds_router_process_rx_queue(g_router.r);

  return res;
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res =
      seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);

  return res;
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res =
      seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);

  return res;
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res =
      seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);

  return res;
}

SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  va_list args;
  va_start(args, fmt);

  // First pass: figure out how long the formatted string will be
  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy); // returns length excluding '\0'
  va_end(args_copy);

  if (len < 0) {
    // Formatting failed; handle however makes sense.
    va_end(args);
    const char empty = '\0';
    return log_telemetry_asynchronous(SEDS_DT_GENERIC_ERROR, &empty, 0, 0);
  }

  // Optional safety cap to avoid huge stack allocations:
  if (len > 512)
    len = 512;

  // Second pass: allocate exactly len+1 bytes on the stack (C99 VLA)
  char data[(size_t)len + 1];

  int written = vsnprintf(data, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) {
    const char empty = '\0';
    return log_telemetry_asynchronous(SEDS_DT_GENERIC_ERROR, &empty, 0, 0);
  }

  // `written` should equal `len`, but we clamp just in case
  size_t used = (size_t)written;

  return log_telemetry_asynchronous(SEDS_DT_GENERIC_ERROR, data,
                                    used, // number of bytes actually used
                                    used  // number of elements (chars)
  );
}
SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  va_list args;
  va_start(args, fmt);

  // First pass: figure out how long the formatted string will be
  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy); // returns length excluding '\0'
  va_end(args_copy);

  if (len < 0) {
    // Formatting failed; handle however makes sense.
    va_end(args);
    const char empty = '\0';
    return log_telemetry_synchronous(SEDS_DT_GENERIC_ERROR, &empty, 0, 0);
  }

  // Optional safety cap to avoid huge stack allocations:
  if (len > 512)
    len = 512;

  // Second pass: allocate exactly len+1 bytes on the stack (C99 VLA)
  char data[(size_t)len + 1];

  int written = vsnprintf(data, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) {
    const char empty = '\0';
    return log_telemetry_synchronous(SEDS_DT_GENERIC_ERROR, &empty, 0, 0);
  }

  // `written` should equal `len`, but we clamp just in case
  size_t used = (size_t)written;

  return log_telemetry_synchronous(SEDS_DT_GENERIC_ERROR, data,
                                   used, // number of bytes actually used
                                   used  // number of elements (chars)
  );
}

/* ---------------- Error printing ---------------- */
SedsResult print_telemetry_error(const int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#endif
  /* Use a small fixed buffer to avoid big stack frames. */
  char buf[seds_error_to_string_len(error_code)];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    log_error_asyncronous("Error: seds_error_to_string failed: %d\r\n", res);
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
