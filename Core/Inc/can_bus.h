#pragma once

#include <stddef.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*can_bus_rx_cb_t)(const uint8_t *data, size_t len, void *user);

/* Init with the FDCAN handle that receives on FIFO1 (e.g. &hfdcan2). */
void can_bus_init(FDCAN_HandleTypeDef *hfdcan);

/* Send raw bytes (len clamped to 64). */
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *bytes, size_t len, uint32_t std_id);

/* Send an arbitrarily large buffer by fragmenting into multiple CAN FD frames. */
HAL_StatusTypeDef can_bus_send_large(const uint8_t *bytes, size_t len, uint32_t std_id);

/*
 * MUST be called periodically from thread/main-loop context.
 * This drains the ISR RX ring, performs reassembly, and invokes subscribers.
 */
void can_bus_process_rx(void);

/*
 * Subscribe a callback to RX events (FIFO1).
 * Can be called at startup before interrupts start firing.
 * Returns HAL_OK on success, HAL_ERROR if the list is full or duplicate.
 */
HAL_StatusTypeDef can_bus_subscribe_rx(can_bus_rx_cb_t cb, void *user);

/*
 * Optional: remove a previously added subscription.
 * Returns HAL_OK if removed, HAL_ERROR if not found.
 */
HAL_StatusTypeDef can_bus_unsubscribe_rx(can_bus_rx_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif
