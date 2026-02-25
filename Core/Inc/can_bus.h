#pragma once

#include <stddef.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*can_bus_rx_cb_t)(const uint8_t *data, size_t len, void *user);

/*
 * Initialize CAN bus module with the FDCAN handle.
 *
 * This must be called once at startup before any other can_bus_* calls.
 *
 * This function:
 *  - Stores the handle
 *  - Enables notifications (unless polling mode enabled)
 *  - Initializes internal queues
 */
void can_bus_init(FDCAN_HandleTypeDef *hfdcan);

/*
 * Send raw bytes (len clamped to 64).
 */
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *bytes, size_t len, uint32_t std_id);

/*
 * Send arbitrarily large buffer via CAN FD fragmentation.
 */
HAL_StatusTypeDef can_bus_send_large(const uint8_t *bytes, size_t len, uint32_t std_id);

/*
 * MUST be called periodically from thread/main-loop context.
 *
 * This function is responsible for ALL processing:
 *
 *  - Drains hardware FIFOs (if ISR flagged or polling detects data)
 *  - Moves frames into internal queues
 *  - Performs reassembly
 *  - Calls subscriber callbacks
 *
 * This function is SAFE to call frequently.
 *
 * It exits immediately if no data is pending.
 *
 * This function is thread-safe ONLY when called from a single thread.
 *
 * ISR NEVER calls subscribers or router directly.
 */
void can_bus_process_rx(void);

/*
 * Subscribe callback to received CAN packets.
 *
 * Callback is invoked from can_bus_process_rx() context ONLY.
 *
 * Safe to call at startup.
 */
HAL_StatusTypeDef can_bus_subscribe_rx(can_bus_rx_cb_t cb, void *user);

/*
 * Remove subscription.
 */
HAL_StatusTypeDef can_bus_unsubscribe_rx(can_bus_rx_cb_t cb, void *user);

/*
 * Print CAN controller status and error counters.
 */
void can_bus_print_status(void);

/*
 * Set own node ID so loopback/self frames can be ignored.
 *
 * Pass 0xFFFFFFFFu to disable filtering.
 */
void can_bus_set_own_id(uint32_t std_id);

/*
 * Legacy compatibility alias.
 *
 * This now simply calls can_bus_process_rx().
 *
 * New code should call can_bus_process_rx() directly.
 */
static inline void can_bus_poll(void)
{
    can_bus_process_rx();
}

#ifdef __cplusplus
}
#endif