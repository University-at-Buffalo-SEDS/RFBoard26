#include "radio.h"
#include <string.h>
#include <stdint.h>

/* Tune these */
#define RADIO_UART_RX_BUF_SIZE      256
#define RADIO_UART_MAX_SUBSCRIBERS  8
#define RADIO_UART_TX_TIMEOUT_MS    5U

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
  // return HAL_OK;
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0) return HAL_ERROR;
  return HAL_UART_Transmit(g_huart, (uint8_t *)bytes, (uint16_t)len, RADIO_UART_TX_TIMEOUT_MS);
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
    radio_notify_rx(item.data, (size_t)item.len);
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
