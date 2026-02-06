#pragma once

#include <stddef.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*radio_rx_cb_t)(const uint8_t *data, size_t len, void *user);

/* Provide the UART handle used for the radio (e.g. &huart1). */
void radio_uart_init(UART_HandleTypeDef *huart);

/* Start ReceiveToIdle interrupt-driven RX. Call once after init, and it will re-arm itself. */
HAL_StatusTypeDef radio_uart_start_rx(void);

/* Send raw bytes over the radio UART (blocking). */
HAL_StatusTypeDef radio_uart_send_bytes(const uint8_t *bytes, size_t len);

/* Subscribe a callback that is invoked from the UART RxEvent callback (ISR context). */
HAL_StatusTypeDef radio_uart_subscribe_rx(radio_rx_cb_t cb, void *user);

/* Optional unsubscribe. */
HAL_StatusTypeDef radio_uart_unsubscribe_rx(radio_rx_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif
