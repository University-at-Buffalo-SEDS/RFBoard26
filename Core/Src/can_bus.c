#include "can_bus.h"
#include <string.h>

/* Tune this for how many listeners you want. */
#define CAN_BUS_MAX_SUBSCRIBERS  8

typedef struct {
  can_bus_rx_cb_t cb;
  void *user;
} can_bus_sub_t;

static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static can_bus_sub_t g_subs[CAN_BUS_MAX_SUBSCRIBERS];

void can_bus_init(FDCAN_HandleTypeDef *hfdcan) {
  g_hfdcan = hfdcan;
  /* subscribers are static-zeroed; no need to clear */
}

/* CAN FD DLC -> length mapping (HAL stores DLC as 0..15) */
static size_t can_bus_dlc_to_len(uint32_t dlc) {
  static const uint8_t map[16] = {
      0, 1, 2, 3, 4, 5, 6, 7,
      8, 12, 16, 20, 24, 32, 48, 64
  };
  dlc &= 0xF;
  return map[dlc];
}

HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *bytes, size_t len, uint32_t std_id) {
  if (!g_hfdcan) return HAL_ERROR;
  if (!bytes || len == 0) return HAL_ERROR;

  if (len > 64) len = 64;

  FDCAN_TxHeaderTypeDef txHeader;
  txHeader.Identifier = std_id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;

  txHeader.DataLength = (uint32_t)len;

  txHeader.ErrorStateIndicator = 0;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_FD_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  uint8_t txData[64] = {0};
  memcpy(txData, bytes, len);

  return HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &txHeader, txData);
}

HAL_StatusTypeDef can_bus_subscribe_rx(can_bus_rx_cb_t cb, void *user) {
  if (!cb) return HAL_ERROR;

  /* Prevent duplicates */
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      return HAL_ERROR;
    }
  }

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == NULL) {
      g_subs[i].cb = cb;
      g_subs[i].user = user;
      return HAL_OK;
    }
  }

  return HAL_ERROR; /* full */
}

HAL_StatusTypeDef can_bus_unsubscribe_rx(can_bus_rx_cb_t cb, void *user) {
  if (!cb) return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      g_subs[i].cb = NULL;
      g_subs[i].user = NULL;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

/* Internal: notify all subscribers (called from ISR context) */
static inline void can_bus_notify_rx(const uint8_t *data, size_t len) {
  /* ISR-safe: fixed array, no locks */
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    can_bus_rx_cb_t cb = g_subs[i].cb;
    if (cb) {
      cb(data, len, g_subs[i].user);
    }
  }
}

/*
  HAL callback lives in the driver now.
  IMPORTANT: ensure only one definition exists in the entire link.
*/
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0) {
    return;
  }

  FDCAN_RxHeaderTypeDef hdr;
  uint8_t data[64];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &hdr, data) != HAL_OK) {
      break;
    }

    size_t len = can_bus_dlc_to_len(hdr.DataLength);
    can_bus_notify_rx(data, len);
  }
}
