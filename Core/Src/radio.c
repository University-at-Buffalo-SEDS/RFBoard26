#include "radio.h"
#include <string.h>
#include <stdint.h>

/* Tune these */
#define RADIO_UART_RX_BUF_SIZE         256
#define RADIO_UART_MAX_SUBSCRIBERS     8
#define RADIO_UART_TX_TIMEOUT_MS       100U
#define RADIO_UART_FRAME_SYNC_0        0xA5U
#define RADIO_UART_FRAME_SYNC_1        0x5AU
#define RADIO_UART_FRAME_HEADER_SIZE   4U
#define RADIO_UART_MAX_PAYLOAD_SIZE    256U
#define RADIO_UART_FRAME_BUF_SIZE      (RADIO_UART_FRAME_HEADER_SIZE + RADIO_UART_MAX_PAYLOAD_SIZE)

/* How many RX chunks we can queue from ISR */
#ifndef RADIO_UART_RX_RING_DEPTH
#define RADIO_UART_RX_RING_DEPTH    8
#endif

typedef struct {
  radio_rx_cb_t cb;
  void *user;
} radio_sub_t;

typedef struct {
  uint16_t len;
  uint8_t  data[RADIO_UART_RX_BUF_SIZE];
} radio_rx_item_t;

static UART_HandleTypeDef *g_huart = NULL;
static uint8_t g_rx_buf[RADIO_UART_RX_BUF_SIZE];
static radio_sub_t g_subs[RADIO_UART_MAX_SUBSCRIBERS];
static uint8_t g_frame_buf[RADIO_UART_FRAME_BUF_SIZE];
static size_t g_frame_len = 0U;

/* ISR -> thread ring buffer */
static volatile uint32_t g_rx_head = 0; /* pop index */
static volatile uint32_t g_rx_tail = 0; /* push index */
static volatile uint32_t g_rx_count = 0;
static radio_rx_item_t g_rx_ring[RADIO_UART_RX_RING_DEPTH];

void radio_uart_init(UART_HandleTypeDef *huart) {
  g_huart = huart;
}

/* Arm RX-to-idle interrupt reception */
HAL_StatusTypeDef radio_uart_start_rx(void) {
  if (!g_huart) return HAL_ERROR;
  return HAL_UARTEx_ReceiveToIdle_IT(g_huart, g_rx_buf, sizeof(g_rx_buf));
}

HAL_StatusTypeDef radio_uart_send_bytes(const uint8_t *bytes, size_t len) {
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0U || len > RADIO_UART_MAX_PAYLOAD_SIZE) return HAL_ERROR;

  uint8_t framed[RADIO_UART_FRAME_BUF_SIZE];
  framed[0] = RADIO_UART_FRAME_SYNC_0;
  framed[1] = RADIO_UART_FRAME_SYNC_1;
  framed[2] = (uint8_t)(len & 0xFFU);
  framed[3] = (uint8_t)((len >> 8U) & 0xFFU);
  memcpy(&framed[RADIO_UART_FRAME_HEADER_SIZE], bytes, len);
  return HAL_UART_Transmit(
      g_huart, framed, (uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len), RADIO_UART_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef radio_uart_subscribe_rx(radio_rx_cb_t cb, void *user) {
  if (!cb) return HAL_ERROR;

  for (unsigned i = 0; i < RADIO_UART_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      return HAL_ERROR; /* duplicate */
    }
  }

  for (unsigned i = 0; i < RADIO_UART_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == NULL) {
      g_subs[i].cb = cb;
      g_subs[i].user = user;
      return HAL_OK;
    }
  }

  return HAL_ERROR; /* full */
}

HAL_StatusTypeDef radio_uart_unsubscribe_rx(radio_rx_cb_t cb, void *user) {
  if (!cb) return HAL_ERROR;
  for (unsigned i = 0; i < RADIO_UART_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      g_subs[i].cb = NULL;
      g_subs[i].user = NULL;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

static inline void radio_notify_rx(const uint8_t *data, size_t len) {
  for (unsigned i = 0; i < RADIO_UART_MAX_SUBSCRIBERS; i++) {
    radio_rx_cb_t cb = g_subs[i].cb;
    if (cb) {
      cb(data, len, g_subs[i].user);
    }
  }
}

static void radio_frame_buf_consume(size_t count)
{
  if (count >= g_frame_len) {
    g_frame_len = 0U;
    return;
  }

  memmove(g_frame_buf, &g_frame_buf[count], g_frame_len - count);
  g_frame_len -= count;
}

static void radio_frame_buf_append(const uint8_t *data, size_t len)
{
  if (!data || len == 0U) {
    return;
  }

  if (len >= RADIO_UART_FRAME_BUF_SIZE) {
    data += len - RADIO_UART_FRAME_BUF_SIZE;
    len = RADIO_UART_FRAME_BUF_SIZE;
    g_frame_len = 0U;
  } else if (g_frame_len + len > RADIO_UART_FRAME_BUF_SIZE) {
    radio_frame_buf_consume(g_frame_len + len - RADIO_UART_FRAME_BUF_SIZE);
  }

  memcpy(&g_frame_buf[g_frame_len], data, len);
  g_frame_len += len;
}

static void radio_process_framed_bytes(const uint8_t *data, size_t len)
{
  radio_frame_buf_append(data, len);

  while (g_frame_len > 0U) {
    size_t sync_pos = 0U;
    uint8_t found_sync = 0U;

    while ((sync_pos + 1U) < g_frame_len) {
      if (g_frame_buf[sync_pos] == RADIO_UART_FRAME_SYNC_0 &&
          g_frame_buf[sync_pos + 1U] == RADIO_UART_FRAME_SYNC_1) {
        found_sync = 1U;
        break;
      }
      sync_pos++;
    }

    if (!found_sync) {
      if (g_frame_buf[g_frame_len - 1U] == RADIO_UART_FRAME_SYNC_0) {
        g_frame_buf[0] = RADIO_UART_FRAME_SYNC_0;
        g_frame_len = 1U;
      } else {
        g_frame_len = 0U;
      }
      return;
    }

    if (sync_pos > 0U) {
      radio_frame_buf_consume(sync_pos);
    }

    if (g_frame_len < RADIO_UART_FRAME_HEADER_SIZE) {
      return;
    }

    const size_t payload_len =
        (size_t)g_frame_buf[2] | ((size_t)g_frame_buf[3] << 8U);
    if (payload_len == 0U || payload_len > RADIO_UART_MAX_PAYLOAD_SIZE) {
      radio_frame_buf_consume(1U);
      continue;
    }

    if (g_frame_len < (RADIO_UART_FRAME_HEADER_SIZE + payload_len)) {
      return;
    }

    radio_notify_rx(&g_frame_buf[RADIO_UART_FRAME_HEADER_SIZE], payload_len);
    radio_frame_buf_consume(RADIO_UART_FRAME_HEADER_SIZE + payload_len);
  }
}

/* Push one RX chunk into ring (ISR context) */
static inline void radio_rx_ring_push_isr(const uint8_t *data, uint16_t len)
{
  if (len == 0) return;
  if (len > RADIO_UART_RX_BUF_SIZE) len = RADIO_UART_RX_BUF_SIZE;

  /* If full: drop */
  if (g_rx_count >= RADIO_UART_RX_RING_DEPTH) {
    return;
  }

  uint32_t tail = g_rx_tail;
  g_rx_ring[tail].len = len;
  memcpy(g_rx_ring[tail].data, data, len);

  tail++;
  if (tail >= RADIO_UART_RX_RING_DEPTH) tail = 0;
  g_rx_tail = tail;
  g_rx_count++;
}

/* Pop one RX chunk from ring (thread context) */
static inline uint8_t radio_rx_ring_pop_thread(radio_rx_item_t *out)
{
  uint8_t have = 0;

  /* Protect head/count against ISR updates */
  __disable_irq();
  if (g_rx_count > 0) {
    uint32_t head = g_rx_head;
    *out = g_rx_ring[head];

    head++;
    if (head >= RADIO_UART_RX_RING_DEPTH) head = 0;
    g_rx_head = head;
    g_rx_count--;
    have = 1;
  }
  __enable_irq();

  return have;
}

/**
 * Call this periodically from a THREAD (NOT ISR).
 * This is the ONLY place subscriber callbacks run.
 */
void radio_uart_process_rx(void)
{
  radio_rx_item_t item;
  while (radio_rx_ring_pop_thread(&item)) {
    radio_process_framed_bytes(item.data, (size_t)item.len);
  }
}

/*
  Own the HAL callback here.
  IMPORTANT: ISR does NOT call subscribers anymore.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (!g_huart) return;
  if (huart->Instance != g_huart->Instance) return;

  if (Size > 0) {
    radio_rx_ring_push_isr(g_rx_buf, Size);
  }

  /* Re-arm for next chunk */
  (void)radio_uart_start_rx();
}
