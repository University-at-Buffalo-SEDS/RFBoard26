#pragma once

#include <stddef.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "tx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*radio_rx_cb_t)(const uint8_t *data, size_t len, void *user);

/* Provide the UART handle used for the radio (e.g. &huart1). */
void radio_uart_init(UART_HandleTypeDef *huart);
UINT radio_uart_init_tx_queue(TX_BYTE_POOL *byte_pool);

/* Start ReceiveToIdle interrupt-driven RX. Call once after init, and it will re-arm itself. */
HAL_StatusTypeDef radio_uart_start_rx(void);

/* Send raw bytes over the radio UART (blocking). */
HAL_StatusTypeDef radio_uart_send_bytes(const uint8_t *bytes, size_t len);
HAL_StatusTypeDef radio_uart_send_plaintext(const uint8_t *bytes, size_t len);
HAL_StatusTypeDef radio_uart_send_command_frame(const uint8_t *bytes, size_t len);
uint8_t radio_uart_tx_ready(void);
uint8_t radio_uart_tx_busy(void);
void radio_uart_process_tx(void);

/* Subscribe a callback that is invoked from radio_uart_process_rx() in thread context. */
HAL_StatusTypeDef radio_uart_subscribe_rx(radio_rx_cb_t cb, void *user);
void radio_uart_process_rx(void);
/* Optional unsubscribe. */
HAL_StatusTypeDef radio_uart_unsubscribe_rx(radio_rx_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif
