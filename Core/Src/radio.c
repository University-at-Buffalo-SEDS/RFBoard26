#include "radio.h"
#include <string.h>

/* Tune these */
#define RADIO_UART_RX_BUF_SIZE      256
#define RADIO_UART_MAX_SUBSCRIBERS  8

typedef struct {
  radio_rx_cb_t cb;
  void *user;
} radio_sub_t;

static UART_HandleTypeDef *g_huart = NULL;
static uint8_t g_rx_buf[RADIO_UART_RX_BUF_SIZE];
static radio_sub_t g_subs[RADIO_UART_MAX_SUBSCRIBERS];

void radio_uart_init(UART_HandleTypeDef *huart) {
  g_huart = huart;
}

HAL_StatusTypeDef radio_uart_start_rx(void) {
  if (!g_huart) return HAL_ERROR;
  return HAL_UARTEx_ReceiveToIdle_IT(g_huart, g_rx_buf, sizeof(g_rx_buf));
}

HAL_StatusTypeDef radio_uart_send_bytes(const uint8_t *bytes, size_t len) {
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0) return HAL_ERROR;
  return HAL_UART_Transmit(g_huart, (uint8_t *)bytes, (uint16_t)len, HAL_MAX_DELAY);
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

/*
  Own the HAL callback here.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (!g_huart) return;
  if (huart->Instance != g_huart->Instance) return;

  if (Size > 0) {
    /* NOTE: g_rx_buf will be overwritten after we re-arm.
       Subscribers must copy if they need persistence. */
    radio_notify_rx(g_rx_buf, (size_t)Size);
  }

  /* Re-arm for next chunk */
  (void)radio_uart_start_rx();
}
