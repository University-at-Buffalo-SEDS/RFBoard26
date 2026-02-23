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

/* Print FDCAN protocol status + error counters to aid debugging. */
void can_bus_print_status(void);

/*
 * Poll hardware FIFOs (RX FIFO1 and TX event FIFO) and push events into the
 * internal rings. This function is safe to call whether HAL notifications
 * (interrupts) are enabled or not: if notifications are active it will
 * temporarily deactivate them while polling to avoid races, then restore
 * them.
 */
void can_bus_poll(void);

/* Set this node's own standard ID so received copies of our own frames can
	be ignored. Pass 0xFFFFFFFFu to disable. */
void can_bus_set_own_id(uint32_t std_id);

#ifdef __cplusplus
}
#endif
