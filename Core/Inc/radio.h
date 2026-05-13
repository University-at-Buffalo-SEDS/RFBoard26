#pragma once

#include <stddef.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "tx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*radio_rx_cb_t)(const uint8_t *data, size_t len, void *user);

typedef struct {
  uint32_t rx_irq_events;
  uint32_t rx_isr_chunks;
  uint32_t rx_isr_bytes;
  uint32_t rx_isr_drops;
  uint32_t rx_frames_ok;
  uint32_t rx_sync_loss;
  uint32_t rx_bad_len;
  uint32_t rx_errors;
  uint32_t rx_restart_errors;
  uint32_t tx_ok;
  uint32_t tx_errors;
  uint32_t tx_busy;
  uint32_t tx_enqueued;
  uint32_t tx_queue_count;
  uint32_t tx_budget_misses;
  uint32_t aux_busy_count;
  uint32_t tx_dma_started;
  uint32_t tx_dma_complete;
  uint32_t tx_startup_drops;
  uint32_t usart_isr;
  uint32_t usart_cr1;
  uint32_t usart_cr3;
  uint32_t dma_rx_remaining;
  uint32_t tx_quiet_remaining_ms;
  uint8_t aux_high;
  uint8_t e22_mode0_pins_configured;
  uint32_t rx_pin_samples_high;
  uint32_t rx_pin_samples_low;
  uint32_t rx_pin_edges;
  uint8_t tx_pin_high;
  uint8_t rx_pin_high;
  uint16_t last_rx_len;
  uint8_t last_rx_preview_len;
  uint8_t last_rx_preview[16];
  uint16_t last_frame_payload_len;
  uint8_t last_frame_kind;
  uint8_t last_frame_preview_len;
  uint8_t last_frame_preview[16];
} radio_uart_stats_t;

/* Provide the UART handle used for the radio (e.g. &huart1). */
void radio_uart_init(UART_HandleTypeDef *huart);
UINT radio_uart_init_tx_queue(TX_BYTE_POOL *byte_pool);

/* Start ReceiveToIdle interrupt-driven RX. Call once after init, and it will re-arm itself. */
HAL_StatusTypeDef radio_uart_start_rx(void);

/* Queue serialized data bytes for the radio UART. */
HAL_StatusTypeDef radio_uart_send_bytes(const uint8_t *bytes, size_t len);
HAL_StatusTypeDef radio_uart_send_plaintext(const uint8_t *bytes, size_t len);
HAL_StatusTypeDef radio_uart_send_command_frame(const uint8_t *bytes, size_t len);
uint8_t radio_uart_tx_ready(void);
uint8_t radio_uart_tx_busy(void);
uint8_t radio_uart_air_busy(void);
uint32_t radio_uart_tx_queue_count(void);
radio_uart_stats_t radio_uart_stats_snapshot(void);
void radio_uart_process_tx(void);
void radio_uart_process_tx_with_budget(uint32_t budget_ms);
uint8_t radio_uart_current_rx_is_command_frame(void);

/* Subscribe a callback that is invoked from radio_uart_process_rx() in thread context. */
HAL_StatusTypeDef radio_uart_subscribe_rx(radio_rx_cb_t cb, void *user);
void radio_uart_process_rx(void);
/* Optional unsubscribe. */
HAL_StatusTypeDef radio_uart_unsubscribe_rx(radio_rx_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif
