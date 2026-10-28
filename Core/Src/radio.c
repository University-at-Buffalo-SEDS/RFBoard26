#include "radio.h"
#include "launchcore/image.h"
#include "uart_dma_rx.h"
#include "recovery_mesh.h"
#include "main.h"
#include "sedsnet_config.h"
#include "tx_api.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* Tune these */
#define RADIO_UART_RX_BUF_SIZE         256
#define RADIO_UART_MAX_SUBSCRIBERS     8
#define RADIO_UART_TX_TIMEOUT_FLOOR_MS 100U
#define RADIO_UART_TX_TIMEOUT_MARGIN_MS 100U
/* Legacy E22 mode pins only require RADIO_E22_MODE_SETTLE_MS before AUX becomes
 * authoritative. A multi-second hold lets discovery fill the bounded radio
 * queue before its first byte can drain, delaying route convergence and
 * discarding the control packets needed to establish it. */
#define RADIO_UART_TX_STARTUP_DELAY_MS 5U
#define RADIO_UART_FRAME_SYNC_0        0xA5U
#define RADIO_UART_FRAME_SYNC_1        0x5AU
#define RADIO_UART_COMMAND_SYNC_0      0xA6U
#define RADIO_UART_COMMAND_SYNC_1      0x5BU
#define RADIO_UART_ASCII_SYNC_0        0xA7U
#define RADIO_UART_ASCII_SYNC_1        0x7AU
#define RADIO_UART_FRAME_HEADER_SIZE   4U
#define RADIO_UART_MAX_PAYLOAD_SIZE    1024U
#define RADIO_UART_FRAME_BUF_SIZE      (RADIO_UART_FRAME_HEADER_SIZE + RADIO_UART_MAX_PAYLOAD_SIZE)
#define RADIO_PACKET_SERVICE_CREDIT_CAP_MS (2U * 55U)
#define RADIO_UART_SCHED_FALLBACK_FLOW 0U
#define RADIO_UART_SCHED_CONTROL_FLOW  0xFFFFFFFFUL
#define RADIO_UART_PRIORITY_DISCOVERY       255U
#define RADIO_UART_PRIORITY_NETWORK_CONTROL 254U
#define RADIO_UART_PRIORITY_USER_MAX        253U
#define RADIO_UART_CLASS_APPLICATION         0U
#define RADIO_UART_CLASS_DISCOVERY           1U
#define RADIO_UART_CLASS_NETWORK_CONTROL     2U

/* How many RX chunks we can queue from ISR */
#ifndef RADIO_UART_RX_RING_DEPTH
#define RADIO_UART_RX_RING_DEPTH    4
#endif

#ifndef RADIO_UART_TX_QUEUE_DEPTH
#define RADIO_UART_TX_QUEUE_DEPTH   2
#endif

#ifndef RADIO_UART_TX_QUEUE_SLOTS
#define RADIO_UART_TX_QUEUE_SLOTS   12U
#endif

/* A continuously active RFD900x can refill the ISR ring while the telemetry
 * thread is draining it. Bound each pass so CAN routing, SEDSNet timers, and
 * outbound UART service cannot be starved by ingress forever. */
#ifndef RADIO_UART_RX_SERVICE_BUDGET
#define RADIO_UART_RX_SERVICE_BUDGET 8U
#endif

#ifndef RADIO_UART_SIM_RX_BYTE_BUDGET
#define RADIO_UART_SIM_RX_BYTE_BUDGET 128U
#endif

#define RADIO_UART_TX_QUEUE_BYTE_CAPACITY \
  (RADIO_UART_TX_QUEUE_DEPTH * RADIO_UART_FRAME_BUF_SIZE)
#define RADIO_UART_TX_CONTROL_RESERVE_BYTES 256U
#define RADIO_UART_TX_CONTROL_RESERVE_SLOTS 2U
_Static_assert(RADIO_UART_TX_QUEUE_BYTE_CAPACITY >=
                   RADIO_UART_FRAME_BUF_SIZE + RADIO_UART_TX_CONTROL_RESERVE_BYTES,
               "Radio queue must hold a full schema frame plus control reserve");
_Static_assert(RADIO_UART_TX_QUEUE_SLOTS > RADIO_UART_TX_CONTROL_RESERVE_SLOTS,
               "Radio queue must retain general-traffic slots");

#ifndef RADIO_UART_TX_MAX_AGE_MS
#define RADIO_UART_TX_MAX_AGE_MS 1500U
#endif

#ifndef RADIO_UART_TX_FRAMES_PER_SERVICE
#define RADIO_UART_TX_FRAMES_PER_SERVICE 1U
#endif

#ifndef COMMAND_TRACE_PRINTS
#define COMMAND_TRACE_PRINTS 0
#endif

#ifndef RFBOARD_RADIO_LISTEN_ONLY
#define RFBOARD_RADIO_LISTEN_ONLY 0
#endif

// Modem MAC arbitration remains in RFD firmware. Capacity backpressure keeps
// UART admission below shared air capacity; only legacy LoRa uses AUX/turns.
#ifndef RFBOARD_LORA_SCHEDULER
#define RFBOARD_LORA_SCHEDULER 0
#endif

/* Async transparent modems may wait for a serial idle boundary before
 * emitting a short packet. This is independent of LoRa MAC/AUX gating. */
#ifndef RADIO_PACKET_GAP_MS
#define RADIO_PACKET_GAP_MS 55U
#endif
#ifndef RADIO_PACKET_BURST_MS
#define RADIO_PACKET_BURST_MS 100U
#endif
#if RADIO_PACKET_GAP_MS > 1000U || RADIO_PACKET_BURST_MS > 500U
#error "RFD packet gap/burst bounds exceed supported ranges"
#endif

#ifndef RADIO_AIR_BIT_RATE_BPS
#define RADIO_AIR_BIT_RATE_BPS 64000U
#endif

#ifndef RADIO_AIR_SHARE_PERCENT
#define RADIO_AIR_SHARE_PERCENT 40U
#endif

#if RADIO_AIR_SHARE_PERCENT < 1 || RADIO_AIR_SHARE_PERCENT > 100
#error "Radio air share must be between 1 and 100 percent"
#endif

#ifndef RADIO_AIR_FRAME_OVERHEAD_BYTES
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#endif

#if RADIO_AIR_BIT_RATE_BPS < 1200 || RADIO_AIR_BIT_RATE_BPS > 1000000
#error "Radio air rate must be between 1200 and 1000000 bits per second"
#endif
#if RADIO_AIR_FRAME_OVERHEAD_BYTES > 256
#error "Radio frame overhead must be no more than 256 bytes"
#endif

#ifndef RADIO_TX_COOLDOWN_MS
#define RADIO_TX_COOLDOWN_MS 0U
#endif

#if RFBOARD_LORA_SCHEDULER && defined(RADIO_E22_M0_GPIO_Port) && defined(RADIO_E22_M0_Pin) && \
    defined(RADIO_E22_M1_GPIO_Port) && defined(RADIO_E22_M1_Pin)
#define RADIO_E22_HAS_MODE_PINS 1
#else
#define RADIO_E22_HAS_MODE_PINS 0
#endif

#if RFBOARD_LORA_SCHEDULER && defined(RADIO_E22_AUX_GPIO_Port) && defined(RADIO_E22_AUX_Pin)
#define RADIO_E22_HAS_AUX_PIN 1
#else
#define RADIO_E22_HAS_AUX_PIN 0
#endif

#ifndef RADIO_E22_MODE_SETTLE_MS
#define RADIO_E22_MODE_SETTLE_MS 2U
#endif

typedef struct {
  radio_rx_cb_t cb;
  void *user;
} radio_sub_t;

typedef struct {
  uint16_t len;
  uint8_t  data[RADIO_UART_RX_BUF_SIZE];
} radio_rx_item_t;

typedef struct {
  uint16_t len;
  uint32_t flow_id;
  uint32_t enqueued_ms;
  uint8_t priority;
  uint8_t is_heartbeat;
  uint8_t control_class;
  uint8_t data[RADIO_UART_FRAME_BUF_SIZE];
  uint8_t uart_frames; /* Uses the existing trailing alignment byte. */
} radio_tx_item_t;

typedef struct {
  uint16_t len;
  uint16_t offset;
  uint32_t flow_id;
  uint32_t enqueued_ms;
  uint8_t priority;
  uint8_t is_heartbeat;
  uint8_t control_class;
  uint8_t uart_frames; /* Uses the existing descriptor padding byte. */
} radio_tx_queue_item_t;

static UART_HandleTypeDef *g_huart = NULL;
static uint8_t g_rx_buf[RADIO_UART_RX_BUF_SIZE];
static uint16_t g_rx_dma_position;
static radio_sub_t g_subs[RADIO_UART_MAX_SUBSCRIBERS];
static uint8_t g_frame_buf[RADIO_UART_FRAME_BUF_SIZE];
/* Bound a corrupted length header without scanning arbitrary packet payloads
 * for sync bytes. Unsigned elapsed time remains valid across tick wrap. */
#define RADIO_PARTIAL_FRAME_TIMEOUT_MS 2000U
static uint32_t g_partial_frame_started_ms;
static uint8_t g_partial_frame_waiting;
static uint16_t g_frame_len = 0U; /* Frame capacity is fixed at 1028 bytes. */
static uint8_t g_current_rx_is_command_frame = 0U;

/* ISR -> thread ring buffer */
#define RADIO_UART_RX_RING_BYTES (RADIO_UART_RX_RING_DEPTH * RADIO_UART_RX_BUF_SIZE)
#if RADIO_UART_RX_RING_BYTES <= UINT16_MAX
typedef uint16_t radio_rx_index_t;
#else
typedef uint32_t radio_rx_index_t;
#endif
static volatile radio_rx_index_t g_rx_head = 0; /* pop index */
static volatile radio_rx_index_t g_rx_tail = 0; /* push index */
static volatile radio_rx_index_t g_rx_count = 0;
/* Short IDLE callbacks must consume only their bytes, not a full DMA slot.
 * This retains the existing capacity and saves the old per-slot lengths. */

static uint8_t g_rx_ring[RADIO_UART_RX_RING_BYTES];
static volatile uint32_t g_rx_isr_chunks = 0;
static volatile uint32_t g_rx_isr_bytes = 0;
static volatile uint32_t g_rx_isr_drops = 0;
volatile uint32_t g_radio_rx_frames_ok = 0;
static volatile uint32_t g_rx_sync_loss = 0;
static volatile uint32_t g_rx_bad_len = 0;
static volatile uint32_t g_rx_crc_errors = 0;
static volatile uint32_t g_rx_irq_events = 0;
static volatile uint32_t g_rx_errors = 0;
static volatile uint32_t g_rx_restart_errors = 0;
static volatile uint8_t g_rx_restart_pending = 0U;
static uint8_t g_rx_recovery_waiting = 0U;
static uint32_t g_rx_recovery_due_ms = 0U;
volatile uint32_t g_radio_rx_recoveries = 0U;
volatile uint32_t g_radio_rx_service_passes __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_radio_rx_service_budget_hits __attribute__((used, externally_visible)) = 0U;
static volatile uint16_t g_last_rx_len = 0U;
static volatile uint8_t g_last_rx_preview_len = 0U;
static volatile uint8_t g_last_rx_preview[16];
static volatile uint16_t g_last_frame_payload_len = 0U;
static volatile uint8_t g_last_frame_kind = 0U;
static volatile uint8_t g_last_frame_preview_len = 0U;
static volatile uint8_t g_last_frame_preview[16];
static volatile uint32_t g_rx_pin_samples_high = 0U;
static volatile uint32_t g_rx_pin_samples_low = 0U;
static volatile uint32_t g_rx_pin_edges = 0U;
static volatile uint8_t g_rx_pin_last = 0xFFU;
static TX_MUTEX g_radio_tx_queue_mutex;
static uint8_t g_radio_tx_queue_mutex_ready = 0U;
/* Keep the fixed transport queue outside the ThreadX byte pool. The recovered
 * pool space is assigned to the isolated SEDSNet allocator so routed bursts
 * cannot fragment networking memory needed by the next large packet. */
static radio_tx_queue_item_t g_tx_queue[RADIO_UART_TX_QUEUE_SLOTS];
static uint8_t g_tx_queue_bytes[RADIO_UART_TX_QUEUE_BYTE_CAPACITY];
/* Narrow only private bounded indices. Larger configured capacities retain
 * 32-bit metadata; queue slots, payload bytes and allocator pools are unchanged. */
#if RADIO_UART_TX_QUEUE_BYTE_CAPACITY <= UINT16_MAX
static uint16_t g_tx_queue_bytes_used = 0U;
#else
static uint32_t g_tx_queue_bytes_used = 0U;
#endif
#if RADIO_UART_TX_QUEUE_SLOTS <= UINT8_MAX
static uint8_t g_tx_count = 0U;
#else
static uint32_t g_tx_count = 0U;
#endif
volatile uint32_t g_tx_drops = 0U;
volatile uint32_t g_tx_drop_oldest = 0U;
volatile uint32_t g_tx_drop_same_flow = 0U;
volatile uint32_t g_tx_drop_stale = 0U;
volatile uint32_t g_tx_drop_control = 0U;
volatile uint32_t g_tx_drop_discovery = 0U;
volatile uint32_t g_tx_drop_network_control = 0U;
volatile uint32_t g_tx_drop_application = 0U;
volatile uint32_t g_tx_drop_heartbeat = 0U;
static volatile uint32_t g_tx_enqueued = 0U;
static volatile uint32_t g_tx_budget_misses = 0U;
volatile uint32_t g_radio_tx_ok = 0U;
static volatile uint32_t g_tx_errors = 0U;
static volatile uint32_t g_tx_timeouts = 0U;
static volatile uint32_t g_tx_busy = 0U;
static volatile uint32_t g_tx_startup_delays = 0U;
static volatile uint32_t g_tx_startup_drops = 0U;
static uint32_t g_tx_last_flow_id = RADIO_UART_SCHED_FALLBACK_FLOW;
static radio_tx_item_t g_tx_dma_item;
static volatile uint8_t g_tx_dma_busy = 0U;
static volatile uint8_t g_tx_dma_preserves_rx = 0U;
static volatile uint32_t g_tx_dma_started = 0U;
static volatile uint32_t g_tx_dma_complete = 0U;
volatile uint32_t g_tx_dma_recoveries = 0U;
static volatile uint32_t g_tx_dma_started_at_ms = 0U;
static volatile uint32_t g_tx_quiet_until_ms = 0U;
static volatile uint8_t g_tx_air_budget_active;
#if !RFBOARD_LORA_SCHEDULER
static volatile uint8_t g_tx_independent_streak;
static volatile uint32_t g_tx_packet_started_ms;
static volatile uint32_t g_tx_packet_until_ms;
static volatile uint8_t g_tx_packet_active;
static volatile uint32_t g_tx_last_forced_boundary_ms;
static volatile uint8_t g_tx_completed_service_credit_ms;
static volatile uint8_t g_tx_pending_service_ms;
static uint64_t g_tx_closed_ack_ids[4];
static uint8_t g_tx_closed_ack_count;
static uint8_t g_tx_closed_ack_cursor;

#endif
static volatile uint32_t g_aux_busy_count = 0U;
static volatile uint32_t g_e22_mode0_set_at_ms = 0U;

static uint32_t radio_hash_update(uint32_t hash, const uint8_t *data, size_t len) {
  for (size_t i = 0U; i < len; i++) {
    hash ^= (uint32_t)data[i];
    hash *= 16777619UL;
  }
  return hash;
}

static uint32_t radio_hash_finish(uint32_t hash) {
  return (hash == RADIO_UART_SCHED_FALLBACK_FLOW) ? 1UL : hash;
}

static uint32_t radio_hash_type_and_sender(uint64_t data_type,
                                           const uint8_t *sender,
                                           size_t sender_len) {
  uint32_t hash = 2166136261UL;

  for (uint32_t i = 0U; i < 8U; i++) {
    const uint8_t byte = (uint8_t)((data_type >> (8U * i)) & 0xFFU);
    hash = radio_hash_update(hash, &byte, 1U);
  }

  hash = radio_hash_update(hash, sender, sender_len);
  return radio_hash_finish(hash);
}

static uint8_t radio_read_uleb128(const uint8_t *data, size_t len,
                                  size_t *offset, uint64_t *out) {
  uint64_t value = 0ULL;
  uint32_t shift = 0U;

  if (!data || !offset || !out || *offset >= len) {
    return 0U;
  }

  for (uint32_t i = 0U; i < 10U && *offset < len; i++) {
    uint8_t byte = data[(*offset)++];
    value |= ((uint64_t)(byte & 0x7FU)) << shift;
    if ((byte & 0x80U) == 0U) {
      *out = value;
      return 1U;
    }
    shift += 7U;
  }

  return 0U;
}

static uint32_t radio_uart_flow_id_from_payload(const uint8_t *payload, size_t len,
                                                uint64_t *data_type_out) {
  size_t offset = 0U;
  uint64_t data_type = 0ULL;
  uint64_t ignored = 0ULL;
  uint64_t source_address = 0ULL;
  uint8_t source_bytes[4];

  if (!payload || len < 3U) {
    return RADIO_UART_SCHED_FALLBACK_FLOW;
  }

  const uint8_t flags = payload[offset++];
  offset++; /* selected endpoint count */

  if (!radio_read_uleb128(payload, len, &offset, &data_type) ||
      !radio_read_uleb128(payload, len, &offset, &ignored) ||
      !radio_read_uleb128(payload, len, &offset, &ignored)) {
    return RADIO_UART_SCHED_FALLBACK_FLOW;
  }
  if (data_type_out != NULL) {
    *data_type_out = data_type;
  }

  /* SEDSNet v4 optionally carries a nonce before the compact 32-bit source
   * address. Older RF firmware interpreted this area as a sender-string
   * length, assigning every v4 packet to flow zero and coalescing unrelated
   * boards in the one-entry radio queue. */
  if ((flags & 0x08U) != 0U) {
    if (!radio_read_uleb128(payload, len, &offset, &ignored)) {
      return RADIO_UART_SCHED_FALLBACK_FLOW;
    }
  }
  if (!radio_read_uleb128(payload, len, &offset, &source_address) ||
      source_address == 0ULL || source_address > UINT32_MAX) {
    return RADIO_UART_SCHED_FALLBACK_FLOW;
  }

  source_bytes[0] = (uint8_t)(source_address & 0xFFU);
  source_bytes[1] = (uint8_t)((source_address >> 8U) & 0xFFU);
  source_bytes[2] = (uint8_t)((source_address >> 16U) & 0xFFU);
  source_bytes[3] = (uint8_t)((source_address >> 24U) & 0xFFU);
  return radio_hash_type_and_sender(data_type, source_bytes, sizeof(source_bytes));
}

static uint8_t radio_uart_user_priority(uint64_t data_type) {
#ifdef TELEMETRY_ENABLED
  SedsDataTypeInfo info = {0};

  if (data_type > UINT32_MAX ||
      seds_dtype_get_info((uint32_t)data_type, NULL, 0U, &info) != SEDS_OK ||
      !info.exists) {
    return 0U;
  }

  return (info.priority > RADIO_UART_PRIORITY_USER_MAX)
             ? RADIO_UART_PRIORITY_USER_MAX
             : info.priority;
#else
  (void)data_type;
  return 0U; /* No runtime network schema in a telemetry-disabled build. */
#endif
}

static uint8_t radio_uart_packet_priority(uint64_t data_type) {
  /* Discovery establishes the routes required by every other packet. */
  if ((data_type >= 7ULL && data_type <= 12ULL) ||
      (data_type >= 15ULL && data_type <= 17ULL)) {
    return RADIO_UART_PRIORITY_DISCOVERY;
  }

  /* Managed-variable control/value traffic and the network's persisted
   * variables share the same transport priority as time synchronization. */
  if ((data_type >= 4ULL && data_type <= 6ULL) ||
      data_type == 13ULL || data_type == 14ULL ||
      data_type == (uint64_t)SEDS_DT_FLIGHT_STATE ||
      data_type == (uint64_t)SEDS_DT_AV_BAY_UNDERGLOW ||
      data_type == 134ULL) {
    return RADIO_UART_PRIORITY_NETWORK_CONTROL;
  }

  /* Preserve the schema's custom ordering below the reserved network bands. */
  return radio_uart_user_priority(data_type);
}

static uint8_t radio_uart_packet_class(uint64_t data_type) {
  if ((data_type >= 7ULL && data_type <= 12ULL) ||
      (data_type >= 15ULL && data_type <= 17ULL)) {
    return RADIO_UART_CLASS_DISCOVERY;
  }
  if ((data_type >= 4ULL && data_type <= 6ULL) ||
      data_type == 13ULL || data_type == 14ULL ||
      data_type == (uint64_t)SEDS_DT_FLIGHT_STATE ||
      data_type == (uint64_t)SEDS_DT_AV_BAY_UNDERGLOW ||
      data_type == 134ULL) {
    return RADIO_UART_CLASS_NETWORK_CONTROL;
  }
  return RADIO_UART_CLASS_APPLICATION;
}

static uint32_t radio_uart_flow_id_from_frame(const uint8_t *data, uint16_t len,
                                              uint8_t *priority_out,
                                              uint8_t *is_heartbeat_out,
                                              uint8_t *control_class_out) {
  uint16_t payload_len;
  uint64_t data_type = 0ULL;

  if (priority_out != NULL) {
    *priority_out = 0U;
  }
  if (is_heartbeat_out != NULL) {
    *is_heartbeat_out = 0U;
  }
  if (control_class_out != NULL) {
    *control_class_out = RADIO_UART_CLASS_APPLICATION;
  }

  if (!data || len < RADIO_UART_FRAME_HEADER_SIZE) {
    return RADIO_UART_SCHED_FALLBACK_FLOW;
  }

  if (data[0] != RADIO_UART_FRAME_SYNC_0 || data[1] != RADIO_UART_FRAME_SYNC_1) {
    return RADIO_UART_SCHED_CONTROL_FLOW;
  }

  payload_len = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
  if (payload_len == 0U ||
      payload_len > RADIO_UART_MAX_PAYLOAD_SIZE ||
      (uint16_t)(payload_len + RADIO_UART_FRAME_HEADER_SIZE) > len) {
    return RADIO_UART_SCHED_FALLBACK_FLOW;
  }

  const uint32_t flow_id = radio_uart_flow_id_from_payload(
      &data[RADIO_UART_FRAME_HEADER_SIZE], payload_len, &data_type);
  if (priority_out != NULL && flow_id != RADIO_UART_SCHED_FALLBACK_FLOW) {
    *priority_out = radio_uart_packet_priority(data_type);
  }
  if (is_heartbeat_out != NULL) {
    *is_heartbeat_out =
        (data_type == (uint64_t)SEDS_DT_HEARTBEAT) ? 1U : 0U;
  }
  if (control_class_out != NULL && flow_id != RADIO_UART_SCHED_FALLBACK_FLOW) {
    *control_class_out = radio_uart_packet_class(data_type);
  }
  return flow_id;
}

static void radio_uart_classify_packet_drop(uint8_t priority,
                                            uint8_t is_heartbeat,
                                            uint8_t control_class) {
  if (priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL) {
    g_tx_drop_control++;
    if (control_class == RADIO_UART_CLASS_DISCOVERY) {
      g_tx_drop_discovery++;
    } else {
      g_tx_drop_network_control++;
    }
  } else if (is_heartbeat) {
    g_tx_drop_heartbeat++;
  } else {
    g_tx_drop_application++;
  }
}

static void radio_uart_store_preview(volatile uint8_t *dst,
                                     volatile uint8_t *dst_len,
                                     const uint8_t *src,
                                     size_t len) {
  size_t preview_len = len;
  if (preview_len > 16U) {
    preview_len = 16U;
  }
  if (src && preview_len > 0U) {
    for (size_t i = 0U; i < preview_len; i++) {
      dst[i] = src[i];
    }
  }
  *dst_len = (uint8_t)preview_len;
}

static HAL_StatusTypeDef radio_uart_lock_tx_queue(void) {
  if (!g_radio_tx_queue_mutex_ready) {
    return HAL_ERROR;
  }
  return (tx_mutex_get(&g_radio_tx_queue_mutex, TX_WAIT_FOREVER) == TX_SUCCESS) ? HAL_OK : HAL_ERROR;
}

static void radio_uart_unlock_tx_queue(void) {
  if (g_radio_tx_queue_mutex_ready) {
    (void)tx_mutex_put(&g_radio_tx_queue_mutex);
  }
}

static void radio_uart_sample_rx_pin(void)
{
  const uint8_t rx_high =
      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;

  if (rx_high) {
    g_rx_pin_samples_high++;
  } else {
    g_rx_pin_samples_low++;
  }

  if (g_rx_pin_last != 0xFFU && g_rx_pin_last != rx_high) {
    g_rx_pin_edges++;
  }
  g_rx_pin_last = rx_high;
}

static uint32_t radio_uart_queue_index_at_offset(uint32_t offset) {
  return offset;
}

static uint32_t radio_uart_air_ms(uint16_t len);
static uint32_t radio_now_ms(void);
static uint8_t radio_uart_independent_control(const uint8_t *data, uint16_t len,
                                             uint8_t priority);

static void radio_e22_force_mode0(void)
{
#if RADIO_E22_HAS_MODE_PINS
  HAL_GPIO_WritePin(RADIO_E22_M0_GPIO_Port, RADIO_E22_M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_E22_M1_GPIO_Port, RADIO_E22_M1_Pin, GPIO_PIN_RESET);
#endif
  g_e22_mode0_set_at_ms = HAL_GetTick();
}

static uint8_t radio_e22_aux_high(void)
{
#if RADIO_E22_HAS_AUX_PIN
  return (HAL_GPIO_ReadPin(RADIO_E22_AUX_GPIO_Port, RADIO_E22_AUX_Pin) == GPIO_PIN_SET) ? 1U : 0U;
#else
  return 1U;
#endif
}

static uint8_t radio_e22_ready_for_uart(void)
{
#if !RFBOARD_LORA_SCHEDULER
  return 1U;
#else
  const uint32_t now_ms = HAL_GetTick();

  if ((uint32_t)(now_ms - g_e22_mode0_set_at_ms) < RADIO_E22_MODE_SETTLE_MS) {
    g_aux_busy_count++;
    return 0U;
  }

  if (!radio_e22_aux_high()) {
    g_aux_busy_count++;
    return 0U;
  }

  return 1U;
#endif
}

static void radio_uart_remove_queue_index(uint32_t index) {
  if (index >= g_tx_count) {
    return;
  }

  const uint32_t removed_offset = g_tx_queue[index].offset;
  const uint32_t removed_len = g_tx_queue[index].len;
  const uint32_t bytes_after =
      g_tx_queue_bytes_used - removed_offset - removed_len;
  if (bytes_after > 0U) {
    memmove(&g_tx_queue_bytes[removed_offset],
            &g_tx_queue_bytes[removed_offset + removed_len], bytes_after);
  }
  g_tx_queue_bytes_used -= removed_len;

  for (uint32_t i = 0U; i < g_tx_count; i++) {
    if (i != index && g_tx_queue[i].offset > removed_offset) {
      g_tx_queue[i].offset = (uint16_t)(g_tx_queue[i].offset - removed_len);
    }
  }
  if (index + 1U < g_tx_count) {
    memmove(&g_tx_queue[index], &g_tx_queue[index + 1U],
            (g_tx_count - index - 1U) * sizeof(g_tx_queue[0]));
  }
  g_tx_count--;
}

/* Called under the TX mutex. A retried DMA batch still owns every original
 * UART-frame slot even though its bytes now share one queue descriptor. */
static uint32_t radio_uart_owned_frame_count(void)
{
  uint32_t count = 0U;
  for (uint32_t i = 0U; i < g_tx_count; ++i)
    count += g_tx_queue[i].uart_frames ? g_tx_queue[i].uart_frames : 1U;
  return count;
}

static void radio_uart_drop_stale_locked(uint32_t now_ms) {
  uint32_t i = 0U;

  while (i < g_tx_count) {
    uint32_t idx = radio_uart_queue_index_at_offset(i);
    if (g_tx_queue[idx].priority < RADIO_UART_PRIORITY_NETWORK_CONTROL &&
        (uint32_t)(now_ms - g_tx_queue[idx].enqueued_ms) >= RADIO_UART_TX_MAX_AGE_MS) {
      radio_uart_remove_queue_index(idx);
      g_tx_drops++;
      g_tx_drop_stale++;
      continue;
    }
    i++;
  }
}

static HAL_StatusTypeDef radio_uart_enqueue_frame(const uint8_t *data, uint16_t len,
                                                  int16_t priority_override) {
  uint32_t flow_id;
  uint8_t incoming_priority = 0U;
  uint8_t incoming_is_heartbeat = 0U;
  uint8_t incoming_control_class = RADIO_UART_CLASS_APPLICATION;

  if (!data || len == 0U || len > RADIO_UART_FRAME_BUF_SIZE) return HAL_ERROR;
  flow_id = radio_uart_flow_id_from_frame(data, len, &incoming_priority,
                                           &incoming_is_heartbeat,
                                           &incoming_control_class);
  if (priority_override >= 0) {
    incoming_priority = (uint8_t)priority_override;
    incoming_control_class =
        (incoming_priority == RADIO_UART_PRIORITY_DISCOVERY)
            ? RADIO_UART_CLASS_DISCOVERY
            : ((incoming_priority == RADIO_UART_PRIORITY_NETWORK_CONTROL)
                   ? RADIO_UART_CLASS_NETWORK_CONTROL
                   : RADIO_UART_CLASS_APPLICATION);
    /* Side-transport frames begin with SDT, not a canonical packet envelope.
     * Keep distinct compact/chunk frames distinct if the legacy flow parser
     * cannot identify them; coalescing all fallback frames corrupts discovery. */
    if (flow_id == RADIO_UART_SCHED_FALLBACK_FLOW) {
      flow_id = radio_hash_finish(radio_hash_update(2166136261UL, data, len));
    }
  }
  if (radio_uart_lock_tx_queue() != HAL_OK) return HAL_ERROR;

  radio_uart_drop_stale_locked(radio_now_ms());

  /* Latest-value traffic, heartbeats, and repeated discovery snapshots are
   * replaceable while still pending.  Coalesce them before considering any
   * capacity eviction so a burst cannot consume one slot per stale value. */
  for (uint32_t i = 0U; i < g_tx_count; i++) {
    if ((incoming_control_class != RADIO_UART_CLASS_APPLICATION ||
         incoming_is_heartbeat) &&
        g_tx_queue[i].flow_id == flow_id) {
      radio_uart_remove_queue_index(i);
      g_tx_drop_same_flow++;
      break;
    }
  }

  while (radio_uart_owned_frame_count() >= RADIO_UART_TX_QUEUE_SLOTS ||
         g_tx_queue_bytes_used + len > RADIO_UART_TX_QUEUE_BYTE_CAPACITY) {
    uint32_t drop = 0U;
    uint8_t lowest_priority = g_tx_queue[0].priority;
    for (uint32_t i = 1U; i < g_tx_count; ++i) {
      if (g_tx_queue[i].priority < lowest_priority) {
        lowest_priority = g_tx_queue[i].priority;
        drop = i;
      }
    }
    /* Never claim that discovery or network-control traffic was delivered by
     * deleting an equally important pending frame. Backpressure leaves the
     * packet in SEDSNet's bounded queue so the next service pass retries it
     * after the UART drains. Higher-priority discovery may still replace a
     * lower-priority network-variable/timesync frame. */
    if (incoming_priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL &&
        lowest_priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL) {
      radio_uart_unlock_tx_queue();
      return HAL_BUSY;
    }
    if (incoming_priority < lowest_priority) {
      g_tx_drops++;
      radio_uart_classify_packet_drop(incoming_priority,
                                      incoming_is_heartbeat,
                                      incoming_control_class);
      radio_uart_unlock_tx_queue();
      return HAL_OK;
    }

    g_tx_drops++;
    g_tx_drop_oldest++;
    radio_uart_classify_packet_drop(g_tx_queue[drop].priority,
                                    g_tx_queue[drop].is_heartbeat,
                                    g_tx_queue[drop].control_class);
    radio_uart_remove_queue_index(drop);
  }

  radio_tx_queue_item_t *queued = &g_tx_queue[g_tx_count];
  queued->len = len;
  queued->uart_frames = 1U;
  queued->offset = (uint16_t)g_tx_queue_bytes_used;
  queued->flow_id = flow_id;
  queued->enqueued_ms = radio_now_ms();
  queued->priority = incoming_priority;
  queued->is_heartbeat = incoming_is_heartbeat;
  queued->control_class = incoming_control_class;
  memcpy(&g_tx_queue_bytes[g_tx_queue_bytes_used], data, len);
  g_tx_queue_bytes_used += len;
  g_tx_count++;
  g_tx_enqueued++;
  radio_uart_unlock_tx_queue();
  return HAL_OK;
}

/* Accepted frames retain owned storage until DMA. Dependent side-transport
 * frames keep FIFO order; independent controls get reserved bounded space.
 * No successful callback admission is later coalesced or evicted. */
static HAL_StatusTypeDef radio_uart_enqueue_sedsnet_frame(
    const uint8_t *data, uint16_t len, uint8_t priority) {
  if (!data || len == 0U || len > RADIO_UART_FRAME_BUF_SIZE) return HAL_ERROR;
  if (radio_uart_lock_tx_queue() != HAL_OK) return HAL_ERROR;
  /* Routine canonical heartbeats may bypass dependent history at dequeue,
   * but must leave admission space for commands and final receipts. */
  const uint8_t control = priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL &&
      radio_uart_independent_control(data, len, priority);
  const uint32_t slot_limit = RADIO_UART_TX_QUEUE_SLOTS -
      (control ? 0U : RADIO_UART_TX_CONTROL_RESERVE_SLOTS);
  const uint32_t byte_limit = RADIO_UART_TX_QUEUE_BYTE_CAPACITY -
      (control ? 0U : RADIO_UART_TX_CONTROL_RESERVE_BYTES);
  if (radio_uart_owned_frame_count() >= slot_limit || g_tx_queue_bytes_used + len > byte_limit) {
    radio_uart_unlock_tx_queue();
    g_tx_busy++;
    return HAL_BUSY;
  }

  radio_tx_queue_item_t *queued = &g_tx_queue[g_tx_count];
  queued->len = len;
  queued->uart_frames = 1U;
  queued->offset = (uint16_t)g_tx_queue_bytes_used;
  queued->flow_id = RADIO_UART_SCHED_FALLBACK_FLOW;
  queued->enqueued_ms = radio_now_ms();
  queued->priority = priority;
  queued->is_heartbeat = 0U;
  queued->control_class = priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL
                              ? RADIO_UART_CLASS_NETWORK_CONTROL
                              : RADIO_UART_CLASS_APPLICATION;
  memcpy(&g_tx_queue_bytes[g_tx_queue_bytes_used], data, len);
  g_tx_queue_bytes_used += len;
  g_tx_count++;
  g_tx_enqueued++;
  radio_uart_unlock_tx_queue();
  return HAL_OK;
}

/* Only canonical packets are independent of the side-transport dictionary.
 * Input is an owned frame generated by our SEDSNet callback, not RX data.
 * Discovery keeps FIFO order; ACKs/commands/managed values may bypass it. */
static uint8_t radio_uart_independent_control(const uint8_t *data, uint16_t len,
                                             uint8_t priority) {
  if (!data || priority < 200U || len < RADIO_UART_FRAME_HEADER_SIZE + 3U ||
      data[0] != RADIO_UART_FRAME_SYNC_0 || data[1] != RADIO_UART_FRAME_SYNC_1 ||
      ((uint32_t)data[2] | ((uint32_t)data[3] << 8U)) !=
          len - RADIO_UART_FRAME_HEADER_SIZE) return 0U;
  const uint8_t *payload = data + RADIO_UART_FRAME_HEADER_SIZE;
  if (memcmp(payload, "SDT", 3U) == 0) return 0U;
  size_t offset = RADIO_UART_FRAME_HEADER_SIZE + 2U;
  uint64_t type = 0U;
  if (!radio_read_uleb128(data, len, &offset, &type)) return 0U;
  return !((type >= 7U && type <= 12U) || (type >= 15U && type <= 17U));
}

#if !RFBOARD_LORA_SCHEDULER
static uint8_t radio_uart_ack_contracts (const uint8_t *data, uint16_t len,
                                        uint8_t priority, uint64_t ids[4]);
#endif

static uint8_t radio_uart_dequeue_frame_with_budget(radio_tx_item_t *out, uint32_t budget_ms) {
  uint8_t have = 0U;
  if (!out) return 0U;
  if (radio_uart_lock_tx_queue() != HAL_OK) return 0U;

  if (g_tx_count > 0U) {
    uint32_t selected = 0U;
    /* Seven control turns, then a FIFO turn. Within independent controls,
     * honor SDK priority so older heartbeats cannot hold ACKs behind them.
     * Equal priorities retain FIFO order; SDT history and active DMA do not
     * move. Count completed independent priority turns, resetting after
     * ordinary service or the owed FIFO turn. Cumulative ordinary traffic
     * must not spend this seven-control fairness quota. */
#if RFBOARD_LORA_SCHEDULER
    if ((g_radio_tx_ok & 7U) != 7U) {
#else
    if (g_tx_independent_streak < 7U) {
#endif
      uint8_t found_control = 0U;
      for (uint32_t i = 0U; i < g_tx_count; ++i) {
        if (radio_uart_independent_control(&g_tx_queue_bytes[g_tx_queue[i].offset],
                                          g_tx_queue[i].len, g_tx_queue[i].priority) &&
            (!found_control || g_tx_queue[i].priority > g_tx_queue[selected].priority)) {
          selected = i;
          found_control = 1U;
        }
      }
    }
    if (radio_uart_air_ms(g_tx_queue[selected].len) <= budget_ms) {
      out->len = g_tx_queue[selected].len;
      out->uart_frames = g_tx_queue[selected].uart_frames ? g_tx_queue[selected].uart_frames : 1U;
      out->flow_id = g_tx_queue[selected].flow_id;
      out->enqueued_ms = g_tx_queue[selected].enqueued_ms;
      out->priority = g_tx_queue[selected].priority;
      out->is_heartbeat = g_tx_queue[selected].is_heartbeat;
      out->control_class = g_tx_queue[selected].control_class;
      memcpy(out->data, &g_tx_queue_bytes[g_tx_queue[selected].offset], out->len);
      radio_uart_remove_queue_index(selected);
#if !RFBOARD_LORA_SCHEDULER
      /* Batch only already-owned adjacent bytes. Receipts may share a
       * physical burst, while each original UART frame still spends its
       * existing fairness turn. No waiting window, extra slots, or bytes. */
      const uint32_t batch_limit = 192U + RADIO_UART_FRAME_HEADER_SIZE;
      uint64_t ack_ids[4];
      const uint8_t ack_batch = out->priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL &&
          radio_uart_ack_contracts(out->data, out->len, out->priority, ack_ids);
      const uint8_t ordinary_batch = selected == 0U && !out->is_heartbeat &&
          out->priority < RADIO_UART_PRIORITY_NETWORK_CONTROL &&
          !radio_uart_independent_control(out->data, out->len, out->priority);
      if (ordinary_batch || ack_batch) {
        while (selected < g_tx_count) {
          const radio_tx_queue_item_t *next = &g_tx_queue[selected];
          const uint8_t next_frames = next->uart_frames ? next->uart_frames : 1U;
          const uint32_t combined = (uint32_t)out->len + next->len;
          if (next->priority != out->priority || next->flow_id != out->flow_id ||
              next->control_class != out->control_class ||
              next->is_heartbeat != out->is_heartbeat ||
              combined > batch_limit || combined > sizeof(out->data) ||
              radio_uart_air_ms((uint16_t)combined) > budget_ms) break;
          if (ack_batch) {
            const uint32_t frames = (uint32_t)out->uart_frames + next_frames;
            if (frames > 4U || frames > 7U - g_tx_independent_streak ||
                radio_uart_ack_contracts(&g_tx_queue_bytes[next->offset],
                    next->len, next->priority, ack_ids) != next_frames) break;
          } else if (radio_uart_independent_control(&g_tx_queue_bytes[next->offset],
                                                   next->len, next->priority)) break;
          memcpy(&out->data[out->len], &g_tx_queue_bytes[next->offset], next->len);
          out->len = (uint16_t)combined;
          out->uart_frames = (uint8_t)(out->uart_frames + next_frames);
          radio_uart_remove_queue_index(selected);
        }
      }
#endif
      g_tx_last_flow_id = out->flow_id;
      have = 1U;
    } else {
      g_tx_budget_misses++;
    }
  }
  radio_uart_unlock_tx_queue();
  return have;
}

/* Put an item removed for transmission back at the head of the owned FIFO.
 * Compact SEDSNet frames are stateful: losing one frame because the HAL was
 * transiently busy can make every following compact frame undecodable. */
static HAL_StatusTypeDef __attribute__((unused))
radio_uart_requeue_frame_front(const radio_tx_item_t *item) {
  if (!item || item->len == 0U || item->len > RADIO_UART_FRAME_BUF_SIZE) {
    return HAL_ERROR;
  }
  if (radio_uart_lock_tx_queue() != HAL_OK) return HAL_ERROR;
  if (radio_uart_owned_frame_count() + (item->uart_frames ? item->uart_frames : 1U) > RADIO_UART_TX_QUEUE_SLOTS ||
      g_tx_queue_bytes_used + item->len > RADIO_UART_TX_QUEUE_BYTE_CAPACITY) {
    radio_uart_unlock_tx_queue();
    return HAL_BUSY;
  }

  memmove(&g_tx_queue_bytes[item->len], g_tx_queue_bytes,
          g_tx_queue_bytes_used);
  for (uint32_t i = 0U; i < g_tx_count; i++) {
    g_tx_queue[i].offset = (uint16_t)(g_tx_queue[i].offset + item->len);
  }
  if (g_tx_count > 0U) {
    memmove(&g_tx_queue[1], &g_tx_queue[0],
            g_tx_count * sizeof(g_tx_queue[0]));
  }
  g_tx_queue[0].len = item->len;
  g_tx_queue[0].uart_frames = item->uart_frames ? item->uart_frames : 1U;
  g_tx_queue[0].offset = 0U;
  g_tx_queue[0].flow_id = item->flow_id;
  g_tx_queue[0].enqueued_ms = item->enqueued_ms;
  g_tx_queue[0].priority = item->priority;
  g_tx_queue[0].is_heartbeat = item->is_heartbeat;
  g_tx_queue[0].control_class = item->control_class;
  memcpy(g_tx_queue_bytes, item->data, item->len);
  g_tx_queue_bytes_used += item->len;
  g_tx_count++;
  radio_uart_unlock_tx_queue();
  return HAL_OK;
}

static uint32_t radio_uart_tx_timeout_ms(uint16_t len)
{
  uint32_t baud = (g_huart != NULL) ? g_huart->Init.BaudRate : 0U;
  if (baud == 0U) {
    baud = RADIO_BAUD_RATE;
  }

  /*
   * 8N1 UART sends 10 bits per byte. Round up the wire time, then add margin
   * so telemetry frames do not time out before leaving the UART.
   */
  uint64_t wire_ms = (((uint64_t)len * 10ULL * 1000ULL) + (uint64_t)baud - 1ULL) /
                     (uint64_t)baud;
  uint64_t timeout_ms = wire_ms + (uint64_t)RADIO_UART_TX_TIMEOUT_MARGIN_MS;

  if (timeout_ms < (uint64_t)RADIO_UART_TX_TIMEOUT_FLOOR_MS) {
    timeout_ms = (uint64_t)RADIO_UART_TX_TIMEOUT_FLOOR_MS;
  }

  if (timeout_ms > 0xFFFFFFFFULL) {
    timeout_ms = 0xFFFFFFFFULL;
  }

  return (uint32_t)timeout_ms;
}

static uint32_t radio_now_ms(void)
{
  return (uint32_t)(((uint64_t)tx_time_get() * 1000ULL) /
                    (uint64_t)TX_TIMER_TICKS_PER_SECOND);
}

static uint32_t radio_uart_air_ms(uint16_t len)
{
  uint32_t air_bps = (uint32_t)(((uint64_t)RADIO_AIR_BIT_RATE_BPS *
                               RADIO_AIR_SHARE_PERCENT) / 100U);
  if (air_bps == 0U) {
    air_bps = 2400U;
  }
  const uint64_t bytes = (uint64_t)len + (uint64_t)RADIO_AIR_FRAME_OVERHEAD_BYTES;
  return (uint32_t)(((bytes * 8ULL * 1000ULL) + (uint64_t)air_bps - 1ULL) /
                    (uint64_t)air_bps);
}

#if !RFBOARD_LORA_SCHEDULER
/* DMA owns homogeneous batches. Classify the first complete UART span;
 * a batched ACK buffer is longer than its first frame's length header. */
static void radio_uart_account_completed_item(const radio_tx_item_t *item)
{
  if (!item || item->len < RADIO_UART_FRAME_HEADER_SIZE) return;
  /* At quota exhaustion dequeue served the oldest FIFO owner, even when
   * that owner itself was control traffic. One completed FIFO turn pays
   * the obligation; do not keep later ACKs behind additional FIFO items. */
  if (g_tx_independent_streak >= 7U) {
    g_tx_independent_streak = 0U;
    return;
  }
  const uint32_t first_span = RADIO_UART_FRAME_HEADER_SIZE + item->data[2] +
                              ((uint32_t)item->data[3] << 8U);
  if (first_span <= item->len && radio_uart_independent_control(
          item->data, (uint16_t)first_span, item->priority)) {
    const uint32_t frames = item->uart_frames ? item->uart_frames : 1U;
    const uint32_t streak = (uint32_t)g_tx_independent_streak + frames;
    g_tx_independent_streak = (uint8_t)(streak > 7U ? 7U : streak);
  } else {
    g_tx_independent_streak = 0U;
  }
}
#endif

#if !RFBOARD_LORA_SCHEDULER
enum {
  RADIO_PACKET_ACTIVE = 1U,
  RADIO_PACKET_CLOSING = 2U,
  RADIO_PACKET_FORCED_SEEN = 4U
};
#endif

/* Budget starts at actual UART admission. Idle time cannot earn burst credit.
 * This is byte-capacity backpressure, independent of modem turn scheduling. */
static void radio_uart_mark_tx_quiet(uint16_t len)
{
  uint32_t quiet_ms = radio_uart_air_ms(len);
#if RFBOARD_LORA_SCHEDULER
  quiet_ms += RADIO_TX_COOLDOWN_MS;
#else
  const uint32_t now = radio_now_ms();
  if (!(g_tx_packet_active & RADIO_PACKET_ACTIVE) ||
      (int32_t)(now - g_tx_packet_until_ms) >= 0) {
    g_tx_packet_started_ms = now;
    g_tx_packet_active &= (uint8_t)~RADIO_PACKET_CLOSING;
  }
  g_tx_packet_active |= RADIO_PACKET_ACTIVE;
  const uint32_t wire_ms = (uint32_t)(((uint64_t)len * 10000ULL +
      RADIO_BAUD_RATE - 1U) / RADIO_BAUD_RATE);
  g_tx_packet_until_ms = now + wire_ms + RADIO_PACKET_GAP_MS;
#endif
  g_tx_quiet_until_ms = radio_now_ms() + quiet_ms;
  g_tx_air_budget_active = 1U;
}

#if !RFBOARD_LORA_SCHEDULER
/* Inspect only an owned SDK canonical plaintext ACK. Its last eight payload
 * bytes are the stable acknowledged logical ID, independent of ACK retries
 * and owner address. This never reconstructs SDT or allocates in the ISR. */
static uint8_t radio_uart_ack_contract(const uint8_t *data, uint16_t len, uint8_t priority, uint64_t *id)
{
  if (!data || !id || len > RADIO_UART_FRAME_BUF_SIZE || priority < RADIO_UART_PRIORITY_NETWORK_CONTROL ||
      len < RADIO_UART_FRAME_HEADER_SIZE + 2U + 4U + 8U + 4U ||
      !radio_uart_independent_control(data, len, priority)) return 0U;
  const uint8_t *body = data + RADIO_UART_FRAME_HEADER_SIZE;
  const size_t size = len - RADIO_UART_FRAME_HEADER_SIZE;
  if ((body[0] & 0x11U) != 0U || (body[0] & 0xA0U) == 0xA0U) return 0U;
  const size_t payload_end = size - 4U;
  const size_t payload_start = payload_end - 8U;
  size_t offset = 2U;
  uint64_t type, data_size, ignored, source;
  if (!radio_read_uleb128(body, payload_start, &offset, &type) || type != 1U ||
      !radio_read_uleb128(body, payload_start, &offset, &data_size) || data_size != 8U ||
      !radio_read_uleb128(body, payload_start, &offset, &ignored)) return 0U;
  if ((body[0] & 0x08U) &&
      !radio_read_uleb128(body, payload_start, &offset, &ignored)) return 0U;
  if (!radio_read_uleb128(body, payload_start, &offset, &source) ||
      source == 0U || source > UINT32_MAX) return 0U;
  const uint32_t crc = (uint32_t)body[payload_end] |
      ((uint32_t)body[payload_end + 1U] << 8U) |
      ((uint32_t)body[payload_end + 2U] << 16U) |
      ((uint32_t)body[payload_end + 3U] << 24U);
  if (launchcore_crc32(body, payload_end) != crc) return 0U;
  uint64_t value = 0U;
  for (uint32_t i = 0U; i < 8U; ++i) value |= (uint64_t)body[payload_start + i] << (8U * i);
  *id = value;
  return 1U;
}

/* Complete canonical ACK spans only: no allocation or SDT reconstruction.
 * A malformed/non-ACK suffix cannot borrow a receipt's close permit. */
static uint8_t radio_uart_ack_contracts(const uint8_t *data, uint16_t len,
                                       uint8_t priority, uint64_t ids[4])
{
  uint32_t offset = 0U;
  uint8_t count = 0U;
  if (!data || !ids || len > RADIO_UART_FRAME_BUF_SIZE) return 0U;
  while (offset < len && count < 4U) {
    if ((uint32_t)len - offset < RADIO_UART_FRAME_HEADER_SIZE) return 0U;
    const uint32_t span = RADIO_UART_FRAME_HEADER_SIZE + data[offset + 2U] +
                          ((uint32_t)data[offset + 3U] << 8U);
    if (span > (uint32_t)len - offset ||
        !radio_uart_ack_contract(data + offset, (uint16_t)span, priority,
                                 &ids[count])) return 0U;
    offset += span;
    ++count;
  }
  return offset == len ? count : 0U;
}

/* The normal155ms permit remains unchanged. Only a new logical ACK can spend
 * a whole extra idle gap funded by completed noninteractive DMA airtime. */
static void radio_uart_close_packet_batch(const radio_tx_item_t *item)
{
  const uint32_t now = radio_now_ms();
  uint64_t ids[4];
  const uint8_t ack_count = radio_uart_ack_contracts(item->data, item->len,
                                                    item->priority, ids);
  if (!ack_count && !radio_uart_independent_control(item->data, item->len,
                                                   item->priority)) return;
  uint8_t fresh_ack = 0U;
  for (uint8_t n = 0U; n < ack_count; ++n) {
    uint8_t slot = 0U;
    while (slot < g_tx_closed_ack_count && g_tx_closed_ack_ids[slot] != ids[n]) ++slot;
    if (slot == g_tx_closed_ack_count) fresh_ack = 1U;
  }
  /* Retried bytes still transmit, but remembered contracts earn no new close. */
  if (ack_count && !fresh_ack) return;
  if ((g_tx_packet_active & RADIO_PACKET_FORCED_SEEN) &&
      (uint32_t)(now - g_tx_last_forced_boundary_ms) <
          RADIO_PACKET_BURST_MS + RADIO_PACKET_GAP_MS) {
    if (!fresh_ack || RADIO_PACKET_GAP_MS == 0U || RADIO_PACKET_GAP_MS > 55U ||
        g_tx_completed_service_credit_ms < RADIO_PACKET_GAP_MS) return;
    /* One actual physical close consumes one already-funded idle gap. */
    g_tx_completed_service_credit_ms -= RADIO_PACKET_GAP_MS;
  }
  if (ack_count) {
    /* Keep every contract served by this close, even if a fresh suffix would
     * evict an older hit. Fill remaining slots with the newest prior IDs. */
    uint64_t recent[4];
    uint8_t count = 0U;
    for (uint8_t n = ack_count; n > 0U; --n) {
      uint8_t i = 0U;
      while (i < count && recent[i] != ids[n - 1U]) ++i;
      if (i == count) recent[count++] = ids[n - 1U];
    }
    for (uint8_t n = 0U; n < g_tx_closed_ack_count && count < 4U; ++n) {
      const uint8_t slot = (uint8_t)((g_tx_closed_ack_cursor + 3U - n) % 4U);
      const uint64_t id = g_tx_closed_ack_ids[slot];
      uint8_t i = 0U;
      while (i < count && recent[i] != id) ++i;
      if (i == count) recent[count++] = id;
    }
    for (uint8_t i = 0U; i < count; ++i)
      g_tx_closed_ack_ids[i] = recent[count - 1U - i];
    g_tx_closed_ack_count = count;
    g_tx_closed_ack_cursor = (uint8_t)(count % 4U);
  }
  g_tx_last_forced_boundary_ms = now;
  g_tx_packet_active |= RADIO_PACKET_CLOSING | RADIO_PACKET_FORCED_SEEN;
}

static void radio_uart_mark_tx_drained(void)
{
  const uint32_t now = radio_now_ms();
  /* Anchor the packet idle boundary to the actual last UART stop bit. */
  if (!(g_tx_packet_active & RADIO_PACKET_ACTIVE)) g_tx_packet_started_ms = now;
  g_tx_packet_active |= RADIO_PACKET_ACTIVE;
  g_tx_packet_until_ms = now + RADIO_PACKET_GAP_MS;
}
#endif

static uint32_t radio_uart_quiet_remaining_ms(void)
{
  const uint32_t now = radio_now_ms();
  uint32_t remaining = 0U;
  if (g_tx_air_budget_active && (int32_t)(g_tx_quiet_until_ms - now) > 0) {
    remaining = g_tx_quiet_until_ms - now;
  }
#if !RFBOARD_LORA_SCHEDULER
  if ((g_tx_packet_active & RADIO_PACKET_ACTIVE) &&
      ((g_tx_packet_active & RADIO_PACKET_CLOSING) ||
       (uint32_t)(now - g_tx_packet_started_ms) >= RADIO_PACKET_BURST_MS)) {
    if ((int32_t)(g_tx_packet_until_ms - now) > 0) {
      const uint32_t packet_remaining = g_tx_packet_until_ms - now;
      if (remaining < packet_remaining) remaining = packet_remaining;
    } else {
      g_tx_packet_active &= (uint8_t)~(RADIO_PACKET_ACTIVE | RADIO_PACKET_CLOSING);
    }
  }
#endif
  return remaining;
}

uint8_t radio_uart_air_busy(void)
{
  if (radio_uart_quiet_remaining_ms() != 0U) return 1U;
  g_tx_air_budget_active = 0U;
  return 0U;
}

uint8_t radio_uart_tx_ready(void)
{
  uint32_t now_ms = radio_now_ms();
  return (now_ms >= (uint64_t)RADIO_UART_TX_STARTUP_DELAY_MS) ? 1U : 0U;
}

uint8_t radio_uart_tx_busy(void)
{
  return (g_tx_dma_busy || radio_uart_air_busy()) ? 1U : 0U;
}

uint32_t radio_uart_tx_queue_count(void)
{
  uint32_t count;

  if (radio_uart_lock_tx_queue() != HAL_OK) {
    return g_tx_count;
  }

  /* Reading queue depth must not invalidate already accepted SDT history. */
  count = radio_uart_owned_frame_count();
  radio_uart_unlock_tx_queue();
  return count;
}

radio_uart_stats_t radio_uart_stats_snapshot(void)
{
  radio_uart_stats_t stats = {0};
  stats.rx_irq_events = g_rx_irq_events;
  stats.rx_isr_chunks = g_rx_isr_chunks;
  stats.rx_isr_bytes = g_rx_isr_bytes;
  stats.rx_isr_drops = g_rx_isr_drops;
  stats.rx_frames_ok = g_radio_rx_frames_ok;
  stats.rx_sync_loss = g_rx_sync_loss;
  stats.rx_bad_len = g_rx_bad_len;
  stats.rx_errors = g_rx_errors;
  stats.rx_restart_errors = g_rx_restart_errors;
  stats.tx_ok = g_radio_tx_ok;
  stats.tx_errors = g_tx_errors;
  stats.tx_busy = g_tx_busy;
  stats.tx_enqueued = g_tx_enqueued;
  stats.tx_queue_count = radio_uart_tx_queue_count();
  stats.tx_drops = g_tx_drops;
  stats.tx_drop_oldest = g_tx_drop_oldest;
  stats.tx_drop_same_flow = g_tx_drop_same_flow;
  stats.tx_drop_stale = g_tx_drop_stale;
  stats.tx_budget_misses = g_tx_budget_misses;
  stats.aux_busy_count = g_aux_busy_count;
  stats.tx_dma_started = g_tx_dma_started;
  stats.tx_dma_complete = g_tx_dma_complete;
  stats.tx_dma_recoveries = g_tx_dma_recoveries;
  stats.tx_startup_drops = g_tx_startup_drops;
  if (g_huart != NULL) {
    stats.usart_isr = g_huart->Instance->ISR;
    stats.usart_cr1 = g_huart->Instance->CR1;
    stats.usart_cr3 = g_huart->Instance->CR3;
    stats.dma_rx_remaining = (g_huart->hdmarx != NULL) ? __HAL_DMA_GET_COUNTER(g_huart->hdmarx) : 0U;
  } else {
    stats.usart_isr = 0U;
    stats.usart_cr1 = 0U;
    stats.usart_cr3 = 0U;
    stats.dma_rx_remaining = 0U;
  }
  stats.tx_quiet_remaining_ms = radio_uart_quiet_remaining_ms();
  stats.aux_high = radio_e22_aux_high();
  stats.e22_mode0_pins_configured = RADIO_E22_HAS_MODE_PINS ? 1U : 0U;
  stats.rx_pin_samples_high = g_rx_pin_samples_high;
  stats.rx_pin_samples_low = g_rx_pin_samples_low;
  stats.rx_pin_edges = g_rx_pin_edges;
  stats.tx_pin_high = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;
  stats.rx_pin_high = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;
  stats.last_rx_len = g_last_rx_len;
  stats.last_rx_preview_len = g_last_rx_preview_len;
  for (size_t i = 0U; i < sizeof(stats.last_rx_preview); i++) {
    stats.last_rx_preview[i] = g_last_rx_preview[i];
  }
  stats.last_frame_payload_len = g_last_frame_payload_len;
  stats.last_frame_kind = g_last_frame_kind;
  stats.last_frame_preview_len = g_last_frame_preview_len;
  for (size_t i = 0U; i < sizeof(stats.last_frame_preview); i++) {
    stats.last_frame_preview[i] = g_last_frame_preview[i];
  }
  return stats;
}

void radio_uart_init(UART_HandleTypeDef *huart) {
  g_huart = huart;
  radio_e22_force_mode0();
}

UINT radio_uart_init_tx_queue(TX_BYTE_POOL *byte_pool) {
  (void)byte_pool;
  if (g_radio_tx_queue_mutex_ready) {
    return TX_SUCCESS;
  }
  memset(g_tx_queue, 0, sizeof(g_tx_queue));

  if (tx_mutex_create(&g_radio_tx_queue_mutex, "radio_txq_mutex", TX_INHERIT) != TX_SUCCESS) {
    return TX_MUTEX_ERROR;
  }
  g_radio_tx_queue_mutex_ready = 1U;
  return TX_SUCCESS;
}

/* Arm RX-to-idle interrupt reception. */
HAL_StatusTypeDef radio_uart_start_rx(void) {
  if (!g_huart) return HAL_ERROR;

  HAL_StatusTypeDef status =
      HAL_UARTEx_ReceiveToIdle_DMA(g_huart, g_rx_buf, sizeof(g_rx_buf));
  if (status == HAL_OK) g_rx_dma_position = 0U;
  return status;

}

/* A failed ISR/startup rearm must not leave a running board deaf forever.
 * Retry in thread context, without aborting the independent TX DMA channel.
 * A quiet but correctly armed receiver is healthy and is never restarted. */
static void radio_uart_recover_rx_if_needed(void)
{
  if (!g_huart) return;
  if (g_tx_dma_busy && !g_tx_dma_preserves_rx) return;
  if (!g_rx_restart_pending && g_huart->RxState == HAL_UART_STATE_BUSY_RX &&
      (g_huart->Instance->CR3 & USART_CR3_DMAR) != 0U) return;
  const uint32_t now = radio_now_ms();
  if (g_rx_recovery_waiting && (int32_t)(now - g_rx_recovery_due_ms) < 0) return;
  g_rx_recovery_due_ms = now + 10U;
  g_rx_recovery_waiting = 1U;
  /* Clear before attempting recovery: a new error during rearm must retain
   * its own pending flag, even if this attempt reports HAL_OK. */
  g_rx_restart_pending = 0U;
  HAL_StatusTypeDef aborted = HAL_UART_AbortReceive(g_huart);
  /* HAL skips DMA abort when DMAR is already clear. A prior error can leave
   * that request bit off while the RX DMA handle/channel is still busy.
   * Explicitly release that RX handle; TX uses a different DMA channel. */
  if (g_huart->hdmarx &&
      HAL_DMA_GetState(g_huart->hdmarx) == HAL_DMA_STATE_BUSY) {
    const HAL_StatusTypeDef dma_status = HAL_DMA_Abort(g_huart->hdmarx);
    if (dma_status != HAL_OK) aborted = dma_status;
  }
  if (aborted != HAL_OK || radio_uart_start_rx() != HAL_OK) {
    g_rx_restart_pending = 1U;
    ++g_rx_restart_errors;
    return;
  }
  g_rx_recovery_waiting = 0U;
  ++g_radio_rx_recoveries;
}

HAL_StatusTypeDef radio_uart_send_bytes(const uint8_t *bytes, size_t len) {
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0U || len > RADIO_UART_MAX_PAYLOAD_SIZE) return HAL_ERROR;
  if (RFBOARD_RADIO_LISTEN_ONLY) {
    return HAL_BUSY;
  }
  if (!radio_uart_tx_ready()) {
    g_tx_startup_drops++;
    return HAL_BUSY;
  }

  uint8_t framed[RADIO_UART_FRAME_BUF_SIZE];
  framed[0] = RADIO_UART_FRAME_SYNC_0;
  framed[1] = RADIO_UART_FRAME_SYNC_1;
  framed[2] = (uint8_t)(len & 0xFFU);
  framed[3] = (uint8_t)((len >> 8U) & 0xFFU);
  memcpy(&framed[RADIO_UART_FRAME_HEADER_SIZE], bytes, len);
  return radio_uart_enqueue_frame(
      framed, (uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len), -1);
}

HAL_StatusTypeDef radio_uart_send_bytes_priority(const uint8_t *bytes, size_t len,
                                                 uint8_t priority) {
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0U || len > RADIO_UART_MAX_PAYLOAD_SIZE) return HAL_ERROR;
  if (RFBOARD_RADIO_LISTEN_ONLY) return HAL_BUSY;
  if (!radio_uart_tx_ready()) {
    g_tx_startup_drops++;
    return HAL_BUSY;
  }
  if (!radio_e22_ready_for_uart()) {
    g_tx_busy++;
    return HAL_BUSY;
  }

  uint8_t framed[RADIO_UART_FRAME_BUF_SIZE];
  framed[0] = RADIO_UART_FRAME_SYNC_0;
  framed[1] = RADIO_UART_FRAME_SYNC_1;
  framed[2] = (uint8_t)(len & 0xFFU);
  framed[3] = (uint8_t)((len >> 8U) & 0xFFU);
  memcpy(&framed[RADIO_UART_FRAME_HEADER_SIZE], bytes, len);
  return radio_uart_enqueue_sedsnet_frame(
      framed, (uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len), priority);
}

HAL_StatusTypeDef radio_uart_send_plaintext(const uint8_t *bytes, size_t len) {
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0U || len > RADIO_UART_FRAME_BUF_SIZE) return HAL_ERROR;
  if (RFBOARD_RADIO_LISTEN_ONLY) {
    return HAL_BUSY;
  }
  if (!radio_uart_tx_ready()) {
    g_tx_startup_drops++;
    return HAL_BUSY;
  }
  return radio_uart_enqueue_frame(bytes, (uint16_t)len, -1);
}

HAL_StatusTypeDef radio_uart_send_command_frame(const uint8_t *bytes, size_t len) {
  uint8_t framed[RADIO_UART_FRAME_BUF_SIZE];
  HAL_StatusTypeDef status;

  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0U || len > RADIO_UART_MAX_PAYLOAD_SIZE) return HAL_ERROR;
  if (RFBOARD_RADIO_LISTEN_ONLY) {
    return HAL_BUSY;
  }
  if (!radio_uart_tx_ready()) {
    g_tx_startup_drops++;
    return HAL_BUSY;
  }
  if (!radio_e22_ready_for_uart()) {
    return HAL_BUSY;
  }
  if (radio_uart_air_busy()) {
    g_tx_busy++;
    return HAL_BUSY;
  }

  framed[0] = RADIO_UART_COMMAND_SYNC_0;
  framed[1] = RADIO_UART_COMMAND_SYNC_1;
  framed[2] = (uint8_t)(len & 0xFFU);
  framed[3] = (uint8_t)((len >> 8U) & 0xFFU);
  memcpy(&framed[RADIO_UART_FRAME_HEADER_SIZE], bytes, len);

  if (g_tx_dma_busy) {
    return HAL_BUSY;
  }
  g_tx_dma_busy = 1U;
  (void)HAL_UART_AbortReceive(g_huart);
  status = HAL_UART_Transmit(
      g_huart,
      framed,
      (uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len),
      radio_uart_tx_timeout_ms((uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len)));
  g_tx_dma_busy = 0U;
  if (status == HAL_OK) {
    g_radio_tx_ok++;
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    radio_uart_mark_tx_quiet((uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len));
  } else {
    g_tx_errors++;
  }
  if (radio_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }

  return status;
}

uint32_t radio_uart_process_tx(void)
{
  return radio_uart_process_tx_with_budget(0xFFFFFFFFUL);
}

uint32_t radio_uart_process_tx_with_budget(uint32_t budget_ms)
{
  radio_tx_item_t item;
  HAL_StatusTypeDef status;
  uint32_t sent = 0U;

  if (!g_huart) {
    return 0U;
  }

  if (RFBOARD_RADIO_LISTEN_ONLY) {
    return 0U;
  }

  /* A lost UART/DMA completion must not disable the return path forever.
   * Recover in thread context, where it is safe to use the queue mutex, and
   * retry the complete owned frame from its sync header. */
  if (g_tx_dma_busy) {
    const uint32_t elapsed = radio_now_ms() - g_tx_dma_started_at_ms;
    const uint32_t deadline =
        radio_uart_tx_timeout_ms(g_tx_dma_item.len) + 50U;
    if (elapsed >= deadline) {
      (void)HAL_UART_AbortTransmit(g_huart);
#if !RFBOARD_LORA_SCHEDULER
      g_tx_pending_service_ms = 0U;
#endif
      g_tx_dma_busy = 0U;
      g_tx_dma_preserves_rx = 0U;
      g_tx_dma_recoveries++;
      if (radio_uart_requeue_frame_front(&g_tx_dma_item) != HAL_OK) {
        g_tx_drops++;
        radio_uart_classify_packet_drop(g_tx_dma_item.priority,
                                        g_tx_dma_item.is_heartbeat,
                                        g_tx_dma_item.control_class);
      }
    }
  }

  if (!radio_uart_tx_ready() || g_tx_dma_busy || !radio_e22_ready_for_uart()) {
    g_tx_startup_delays++;
    return 0U;
  }

  /* Retain queued frames until bounded shared capacity has drained so a new
   * command can still overtake bulk before DMA offloads it into the modem.
   * RFD owns all MAC/turn arbitration; no receive window or AUX is added. */
  if (radio_uart_air_busy()) return 0U;
  while (sent < RADIO_UART_TX_FRAMES_PER_SERVICE &&
         radio_uart_dequeue_frame_with_budget(&item, budget_ms)) {
    g_tx_dma_item = item;
    g_tx_dma_busy = 1U;
    g_tx_dma_preserves_rx = 1U;
    g_tx_dma_started_at_ms = radio_now_ms();
#if !RFBOARD_LORA_SCHEDULER
    g_tx_pending_service_ms = 0U;
    /* Ordered discovery is noninteractive service too. Its actual completed
     * UART airtime funds the same capped idle allowance as telemetry; ACKs,
     * commands and other independent controls cannot mint this credit. */
    if (!radio_uart_independent_control(g_tx_dma_item.data, g_tx_dma_item.len,
                                        g_tx_dma_item.priority)) {
      const uint32_t cost = radio_uart_air_ms(g_tx_dma_item.len);
      g_tx_pending_service_ms = (uint8_t)(cost > RADIO_PACKET_SERVICE_CREDIT_CAP_MS ? RADIO_PACKET_SERVICE_CREDIT_CAP_MS : cost);
    }
#endif
    status = HAL_UART_Transmit_DMA(g_huart, g_tx_dma_item.data, g_tx_dma_item.len);
    if (status != HAL_OK) {
#if !RFBOARD_LORA_SCHEDULER
      g_tx_pending_service_ms = 0U;
#endif
      g_tx_dma_busy = 0U;
      g_tx_dma_preserves_rx = 0U;
      g_tx_errors++;
      if (status == HAL_BUSY) {
        g_tx_busy++;
      }
      (void)HAL_UART_AbortTransmit(g_huart);
      g_tx_quiet_until_ms = radio_now_ms();
      if (radio_uart_requeue_frame_front(&g_tx_dma_item) != HAL_OK) {
        g_tx_drops++;
        radio_uart_classify_packet_drop(g_tx_dma_item.priority,
                                        g_tx_dma_item.is_heartbeat,
                                        g_tx_dma_item.control_class);
      }
      break;
    }

    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    radio_uart_mark_tx_quiet(g_tx_dma_item.len);
#if !RFBOARD_LORA_SCHEDULER
    if (g_tx_dma_item.priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL) {
      radio_uart_close_packet_batch(&g_tx_dma_item);
    }
#endif
    g_tx_dma_started++;
    sent++;

  }

  return sent;
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

uint8_t radio_uart_current_rx_is_command_frame(void) {
  return g_current_rx_is_command_frame;
}

static void radio_frame_buf_consume(size_t count)
{
  g_partial_frame_waiting = 0U;
  if (count >= g_frame_len) {
    g_frame_len = 0U;
    return;
  }

  memmove(g_frame_buf, &g_frame_buf[count], g_frame_len - count);
  g_frame_len -= count;
}

static void radio_frame_buf_append(const uint8_t *data, size_t len)
{
  if (!data || len == 0U) {
    return;
  }

  if (len >= RADIO_UART_FRAME_BUF_SIZE) {
    data += len - RADIO_UART_FRAME_BUF_SIZE;
    len = RADIO_UART_FRAME_BUF_SIZE;
    g_frame_len = 0U;
  } else if (g_frame_len + len > RADIO_UART_FRAME_BUF_SIZE) {
    radio_frame_buf_consume(g_frame_len + len - RADIO_UART_FRAME_BUF_SIZE);
  }

  memcpy(&g_frame_buf[g_frame_len], data, len);
  g_frame_len += len;
}

static void radio_process_buffered_frames(void)
{

  while (g_frame_len > 0U) {
    size_t sync_pos = 0U;
    uint8_t found_sync = 0U;

    while ((sync_pos + 1U) < g_frame_len) {
      if ((g_frame_buf[sync_pos] == RADIO_UART_FRAME_SYNC_0 &&
           g_frame_buf[sync_pos + 1U] == RADIO_UART_FRAME_SYNC_1) ||
          (g_frame_buf[sync_pos] == RADIO_UART_COMMAND_SYNC_0 &&
           g_frame_buf[sync_pos + 1U] == RADIO_UART_COMMAND_SYNC_1) ||
          (g_frame_buf[sync_pos] == RADIO_UART_ASCII_SYNC_0 &&
           g_frame_buf[sync_pos + 1U] == RADIO_UART_ASCII_SYNC_1)) {
        found_sync = 1U;
        break;
      }
      sync_pos++;
    }

    if (!found_sync) {
      const uint8_t keep_sync_prefix =
          (g_frame_buf[g_frame_len - 1U] == RADIO_UART_FRAME_SYNC_0 ||
           g_frame_buf[g_frame_len - 1U] == RADIO_UART_COMMAND_SYNC_0 ||
           g_frame_buf[g_frame_len - 1U] == RADIO_UART_ASCII_SYNC_0) ? 1U : 0U;
      /* A retained single prefix is normal UART fragmentation, not lost sync. */
      if (g_frame_len > (size_t)keep_sync_prefix) {
        g_rx_sync_loss++;
      }
      if (keep_sync_prefix) {
        g_frame_buf[0] = g_frame_buf[g_frame_len - 1U];
        g_frame_len = 1U;
      } else {
        g_frame_len = 0U;
      }
      return;
    }

    if (sync_pos > 0U) {
      radio_frame_buf_consume(sync_pos);
    }

    if (g_frame_len < RADIO_UART_FRAME_HEADER_SIZE) {
      return;
    }

    const uint8_t is_command_frame =
        (g_frame_buf[0] == RADIO_UART_COMMAND_SYNC_0 &&
         g_frame_buf[1] == RADIO_UART_COMMAND_SYNC_1) ? 1U : 0U;
    const uint8_t is_ascii_frame =
        (g_frame_buf[0] == RADIO_UART_ASCII_SYNC_0 &&
         g_frame_buf[1] == RADIO_UART_ASCII_SYNC_1) ? 1U : 0U;

    const size_t payload_len =
        (size_t)g_frame_buf[2] | ((size_t)g_frame_buf[3] << 8U);
    if (payload_len == 0U || payload_len > RADIO_UART_MAX_PAYLOAD_SIZE) {
      g_rx_bad_len++;
      radio_frame_buf_consume(1U);
      continue;
    }

    if (g_frame_len < (RADIO_UART_FRAME_HEADER_SIZE + payload_len)) {
      const uint32_t now = radio_now_ms();
      if (!g_partial_frame_waiting) {
        g_partial_frame_started_ms = now;
        g_partial_frame_waiting = 1U;
      } else if ((uint32_t)(now - g_partial_frame_started_ms) >=
                 RADIO_PARTIAL_FRAME_TIMEOUT_MS) {
        /* Discard only one byte so an intact following frame can recover. */
        g_rx_bad_len++;
        radio_frame_buf_consume(1U);
        continue;
      }
      return;
    }

    const uint8_t *body = &g_frame_buf[RADIO_UART_FRAME_HEADER_SIZE];
    if (!is_command_frame && !is_ascii_frame && payload_len >= 3U &&
        memcmp(body, "SDT", 3U) == 0) {
      uint32_t expected = 0U;
      if (payload_len >= 8U) {
        const uint8_t *crc = &body[payload_len - 4U];
        expected = (uint32_t)crc[0] | ((uint32_t)crc[1] << 8U) |
                   ((uint32_t)crc[2] << 16U) | ((uint32_t)crc[3] << 24U);
      }
      if (payload_len < 8U || launchcore_crc32(body, payload_len - 4U) != expected) {
        /* A truncated frame can absorb the next frame's sync. Check integrity
         * before consuming the claimed length, then retain bytes for resync. */
        ++g_rx_crc_errors;
        radio_frame_buf_consume(1U);
        continue;
      }
    }

    g_current_rx_is_command_frame = is_command_frame;
    g_last_frame_payload_len = (uint16_t)payload_len;
    g_last_frame_kind = is_command_frame ? 2U : (is_ascii_frame ? 3U : 1U);
    radio_uart_store_preview(g_last_frame_preview,
                             &g_last_frame_preview_len,
                             &g_frame_buf[RADIO_UART_FRAME_HEADER_SIZE],
                             payload_len);
    const uint8_t *notify_data = &g_frame_buf[RADIO_UART_FRAME_HEADER_SIZE];
    size_t notify_len = payload_len;
    uint8_t notify_command = is_command_frame;
    if (recovery_mesh_uplink(&notify_data, &notify_len, &notify_command)) {
      g_current_rx_is_command_frame = notify_command;
      radio_notify_rx(notify_data, notify_len);
    }
    g_current_rx_is_command_frame = 0U;
    g_radio_rx_frames_ok++;
    radio_frame_buf_consume(RADIO_UART_FRAME_HEADER_SIZE + payload_len);
  }
}

/* Drain complete frames before appending bytes from the following frame.
 * A UART chunk can straddle the end of a maximum-size frame; appending the
 * whole chunk first would evict the still-incomplete frame's sync/header. */
static void radio_process_framed_bytes(const uint8_t *data, size_t len)
{
  if (data == NULL) return;
  while (len != 0U) {
    const size_t room = RADIO_UART_FRAME_BUF_SIZE - g_frame_len;
    const size_t take = len < room ? len : room;
    radio_frame_buf_append(data, take);
    data += take;
    len -= take;
    radio_process_buffered_frames();
  }
}

/* Push one RX chunk into ring (ISR context) */
static inline void radio_rx_ring_push_isr(const uint8_t *data, uint16_t len)
{
  if (len == 0) return;
  if (len > RADIO_UART_RX_BUF_SIZE) len = RADIO_UART_RX_BUF_SIZE;

  /* Reject a whole chunk if full; never overwrite already accepted bytes. */
  if (len > RADIO_UART_RX_RING_BYTES - g_rx_count) {
    g_rx_isr_drops++;
    return;
  }

  const uint32_t tail = g_rx_tail;
  const uint32_t first = len < RADIO_UART_RX_RING_BYTES - tail
                             ? len : RADIO_UART_RX_RING_BYTES - tail;
  memcpy(&g_rx_ring[tail], data, first);
  memcpy(g_rx_ring, data + first, len - first);
  g_rx_tail = (tail + len) % RADIO_UART_RX_RING_BYTES;
  g_rx_count += len;
  g_rx_isr_chunks++;
  g_rx_isr_bytes += len;
}

/* Pop bounded byte batches in thread context. ISR fragments need not be frames. */
static inline uint8_t radio_rx_ring_pop_thread(radio_rx_item_t *out)
{
  uint8_t have = 0;
  __disable_irq();
  const uint32_t count = g_rx_count;
  if (count > 0U) {
    const uint32_t len = count < RADIO_UART_RX_BUF_SIZE ? count : RADIO_UART_RX_BUF_SIZE;
    const uint32_t head = g_rx_head;
    const uint32_t first = len < RADIO_UART_RX_RING_BYTES - head
                               ? len : RADIO_UART_RX_RING_BYTES - head;
    out->len = (uint16_t)len;
    memcpy(out->data, &g_rx_ring[head], first);
    memcpy(out->data + first, g_rx_ring, len - first);
    g_rx_head = (head + len) % RADIO_UART_RX_RING_BYTES;
    g_rx_count -= len;
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
  uint32_t processed = 0U;
  radio_uart_sample_rx_pin();
  radio_uart_recover_rx_if_needed();



  radio_rx_item_t item;
  while (processed < RADIO_UART_RX_SERVICE_BUDGET &&
         radio_rx_ring_pop_thread(&item)) {
    radio_process_framed_bytes(item.data, (size_t)item.len);
    processed++;
  }
  if (processed == RADIO_UART_RX_SERVICE_BUDGET && g_rx_count > 0U) {
    g_radio_rx_service_budget_hits++;
  }
  /* Expiry must run even when the damaged frame is followed by silence. */
  radio_process_buffered_frames();
  g_radio_rx_service_passes++;
}

/*
  Own the HAL callback here.
  IMPORTANT: ISR does NOT call subscribers anymore.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (!g_huart) return;
  if (huart->Instance != g_huart->Instance) return;
  g_rx_irq_events++;

  if (Size > 0) {
    g_last_rx_len = Size;
    radio_uart_store_preview(g_last_rx_preview,
                             &g_last_rx_preview_len,
                             g_rx_buf,
                             Size);
    uart_dma_rx_consume(g_rx_buf, sizeof(g_rx_buf), Size,
                        &g_rx_dma_position, radio_rx_ring_push_isr);
  }

  /* Circular DMA remains armed across HT, TC and IDLE. Error recovery owns
   * restarts; rearming here creates a receive gap on every UART fragment. */
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!g_huart) return;
  if (huart->Instance != g_huart->Instance) return;

  const uint8_t preserves_rx = g_tx_dma_preserves_rx;
  const uint8_t uart_frames = g_tx_dma_busy && preserves_rx && g_tx_dma_item.uart_frames
                                  ? g_tx_dma_item.uart_frames : 1U;
#if !RFBOARD_LORA_SCHEDULER
  if (g_tx_dma_busy && preserves_rx) {
    radio_uart_account_completed_item(&g_tx_dma_item);
    const uint32_t credit = (uint32_t)g_tx_completed_service_credit_ms + g_tx_pending_service_ms;
    g_tx_completed_service_credit_ms = (uint8_t)(credit > RADIO_PACKET_SERVICE_CREDIT_CAP_MS ? RADIO_PACKET_SERVICE_CREDIT_CAP_MS : credit);
  }
  g_tx_pending_service_ms = 0U;
  radio_uart_mark_tx_drained();
#endif
  g_tx_dma_busy = 0U;
  g_tx_dma_preserves_rx = 0U;
  g_radio_tx_ok += uart_frames;
  g_tx_dma_complete++;
  if (!preserves_rx && radio_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (!g_huart || huart->Instance != g_huart->Instance) return;
#if !RFBOARD_LORA_SCHEDULER
  /* An RX error may conservatively discard pending service credit too.
   * Never fund an errored TX from a later or spurious completion callback. */
  g_tx_pending_service_ms = 0U;
#endif
  ++g_rx_errors;
  /* HAL has completed its error callback before the service loop rearms RX.
   * A one-shot rearm here can fail while the UART/DMA lock is still busy. */
  g_rx_restart_pending = 1U;
}

uint32_t board_health_uart_rx_drops(void) { return g_rx_isr_drops; }
