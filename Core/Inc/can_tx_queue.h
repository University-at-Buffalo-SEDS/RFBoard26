#pragma once
#include "can_bus.h"
/* Keep the existing 55-byte reassembly stride and 128-byte receive support.
 * Align SDK chunks to two complete CAN fragments instead of padding a third.
 * Larger logical messages still use the SDK's bounded chunk/reassembly path. */
#define CAN_TX_FRAGMENT_PAYLOAD_BYTES 55U
#define CAN_TX_ALIGNED_FRAME_BYTES (2U * CAN_TX_FRAGMENT_PAYLOAD_BYTES)
/* Foreground submit copies the packet. HAL_OK means queued, not bus ACKed. */
HAL_StatusTypeDef can_tx_queue_submit(const uint8_t *data, size_t len, uint32_t id);
/* Thread service reclaims memory; IRQ pump never allocates/frees. */
void can_tx_queue_service(void);
void can_tx_queue_pump(void);
void can_tx_queue_reset(void);
