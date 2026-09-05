#include "radio.h"
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
/* The E22 mode pins only require RADIO_E22_MODE_SETTLE_MS before AUX becomes
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

#define RADIO_UART_TX_QUEUE_BYTE_CAPACITY \
  (RADIO_UART_TX_QUEUE_DEPTH * RADIO_UART_FRAME_BUF_SIZE)

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

#ifndef RADIO_AIR_BIT_RATE_BPS
#define RADIO_AIR_BIT_RATE_BPS RADIO_BAUD_RATE
#endif

#ifndef RADIO_AIR_FRAME_OVERHEAD_BYTES
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#endif

#ifndef RADIO_TX_COOLDOWN_MS
#define RADIO_TX_COOLDOWN_MS 25U
#endif

#if defined(RADIO_E22_M0_GPIO_Port) && defined(RADIO_E22_M0_Pin) && \
    defined(RADIO_E22_M1_GPIO_Port) && defined(RADIO_E22_M1_Pin)
#define RADIO_E22_HAS_MODE_PINS 1
#else
#define RADIO_E22_HAS_MODE_PINS 0
#endif

#if defined(RADIO_E22_AUX_GPIO_Port) && defined(RADIO_E22_AUX_Pin)
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
} radio_tx_item_t;

typedef struct {
  uint16_t len;
  uint16_t offset;
  uint32_t flow_id;
  uint32_t enqueued_ms;
  uint8_t priority;
  uint8_t is_heartbeat;
  uint8_t control_class;
} radio_tx_queue_item_t;

static UART_HandleTypeDef *g_huart = NULL;
static uint8_t g_rx_buf[RADIO_UART_RX_BUF_SIZE];
static radio_sub_t g_subs[RADIO_UART_MAX_SUBSCRIBERS];
static uint8_t g_frame_buf[RADIO_UART_FRAME_BUF_SIZE];
static size_t g_frame_len = 0U;
static uint8_t g_current_rx_is_command_frame = 0U;

/* ISR -> thread ring buffer */
static volatile uint32_t g_rx_head = 0; /* pop index */
static volatile uint32_t g_rx_tail = 0; /* push index */
static volatile uint32_t g_rx_count = 0;
static radio_rx_item_t g_rx_ring[RADIO_UART_RX_RING_DEPTH];
static volatile uint32_t g_rx_isr_chunks = 0;
static volatile uint32_t g_rx_isr_bytes = 0;
static volatile uint32_t g_rx_isr_drops = 0;
volatile uint32_t g_radio_rx_frames_ok = 0;
static volatile uint32_t g_rx_sync_loss = 0;
static volatile uint32_t g_rx_bad_len = 0;
static volatile uint32_t g_rx_irq_events = 0;
static volatile uint32_t g_rx_errors = 0;
static volatile uint32_t g_rx_restart_errors = 0;
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
static uint32_t g_tx_queue_bytes_used = 0U;
static uint32_t g_tx_count = 0U;
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
static volatile uint32_t g_tx_dma_started = 0U;
static volatile uint32_t g_tx_dma_complete = 0U;
static volatile uint32_t g_tx_quiet_until_ms = 0U;
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
  SedsDataTypeInfo info = {0};

  if (data_type > UINT32_MAX ||
      seds_dtype_get_info((uint32_t)data_type, NULL, 0U, &info) != SEDS_OK ||
      !info.exists) {
    return 0U;
  }

  return (info.priority > RADIO_UART_PRIORITY_USER_MAX)
             ? RADIO_UART_PRIORITY_USER_MAX
             : info.priority;
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

static void radio_uart_drop_item_stale_or_requeue_front(const radio_tx_item_t *item) {
  const uint32_t now_ms = radio_now_ms();

  if (item == NULL) {
    return;
  }

  if (radio_uart_lock_tx_queue() != HAL_OK) {
    return;
  }

  if (item->priority < RADIO_UART_PRIORITY_NETWORK_CONTROL &&
      (uint32_t)(now_ms - item->enqueued_ms) >= RADIO_UART_TX_MAX_AGE_MS) {
    g_tx_drops++;
    g_tx_drop_stale++;
    radio_uart_unlock_tx_queue();
    return;
  }

  if (g_tx_count < RADIO_UART_TX_QUEUE_SLOTS &&
      g_tx_queue_bytes_used + item->len <= RADIO_UART_TX_QUEUE_BYTE_CAPACITY) {
    radio_tx_queue_item_t *queued = &g_tx_queue[g_tx_count];
    queued->len = item->len;
    queued->offset = (uint16_t)g_tx_queue_bytes_used;
    queued->flow_id = item->flow_id;
    queued->enqueued_ms = item->enqueued_ms;
    queued->priority = item->priority;
    queued->is_heartbeat = item->is_heartbeat;
    queued->control_class = item->control_class;
    memcpy(&g_tx_queue_bytes[g_tx_queue_bytes_used], item->data, item->len);
    g_tx_queue_bytes_used += item->len;
    g_tx_count++;
  } else {
    g_tx_drops++;
    g_tx_drop_oldest++;
    radio_uart_classify_packet_drop(item->priority, item->is_heartbeat,
                                    item->control_class);
  }

  radio_uart_unlock_tx_queue();
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

  while (g_tx_count >= RADIO_UART_TX_QUEUE_SLOTS ||
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

static uint8_t radio_uart_dequeue_frame_with_budget(radio_tx_item_t *out, uint32_t budget_ms) {
  uint8_t have = 0U;
  if (!out) return 0U;
  if (radio_uart_lock_tx_queue() != HAL_OK) return 0U;

  radio_uart_drop_stale_locked(radio_now_ms());

  if (g_tx_count > 0U) {
    uint8_t selected_valid = 0U;
    uint32_t selected = 0U;
    uint8_t selected_priority = 0U;

    for (uint32_t i = 0U; i < g_tx_count; i++) {
      uint32_t idx = radio_uart_queue_index_at_offset(i);
      const uint8_t candidate_avoids_last_flow =
          (g_tx_queue[idx].flow_id != g_tx_last_flow_id) ? 1U : 0U;
      const uint8_t selected_avoids_last_flow =
          (g_tx_queue[selected].flow_id != g_tx_last_flow_id) ? 1U : 0U;
      if (radio_uart_air_ms(g_tx_queue[idx].len) <= budget_ms &&
          (!selected_valid ||
           g_tx_queue[idx].priority > selected_priority ||
           (g_tx_queue[idx].priority == selected_priority &&
            candidate_avoids_last_flow > selected_avoids_last_flow))) {
        selected = idx;
        selected_valid = 1U;
        selected_priority = g_tx_queue[idx].priority;
      }
    }

    if (selected_valid) {
      out->len = g_tx_queue[selected].len;
      out->flow_id = g_tx_queue[selected].flow_id;
      out->enqueued_ms = g_tx_queue[selected].enqueued_ms;
      out->priority = g_tx_queue[selected].priority;
      out->is_heartbeat = g_tx_queue[selected].is_heartbeat;
      out->control_class = g_tx_queue[selected].control_class;
      memcpy(out->data, &g_tx_queue_bytes[g_tx_queue[selected].offset], out->len);
      radio_uart_remove_queue_index(selected);
      g_tx_last_flow_id = out->flow_id;
      have = 1U;
    } else {
      g_tx_budget_misses++;
    }
  }
  radio_uart_unlock_tx_queue();
  return have;
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
  uint32_t air_bps = RADIO_AIR_BIT_RATE_BPS;
  if (air_bps == 0U) {
    air_bps = 2400U;
  }
  const uint64_t bytes = (uint64_t)len + (uint64_t)RADIO_AIR_FRAME_OVERHEAD_BYTES;
  return (uint32_t)(((bytes * 10ULL * 1000ULL) + (uint64_t)air_bps - 1ULL) /
                    (uint64_t)air_bps);
}

static void radio_uart_mark_tx_quiet(uint16_t len)
{
  const uint32_t quiet_ms = radio_uart_air_ms(len) + RADIO_TX_COOLDOWN_MS;
  g_tx_quiet_until_ms = radio_now_ms() + quiet_ms;
}

static uint8_t radio_uart_reserve_airtime(uint16_t len, uint32_t budget_ms)
{
  const uint32_t now_ms = radio_now_ms();
  const uint32_t wait_ms = (g_tx_quiet_until_ms > now_ms) ? (g_tx_quiet_until_ms - now_ms) : 0U;
  const uint32_t air_ms = radio_uart_air_ms(len) + RADIO_TX_COOLDOWN_MS;

  if (budget_ms != 0xFFFFFFFFUL &&
      (wait_ms >= budget_ms || air_ms > (uint32_t)(budget_ms - wait_ms))) {
    return 0U;
  }

  g_tx_quiet_until_ms = now_ms + wait_ms + air_ms;
  return 1U;
}

static uint32_t radio_uart_available_air_budget(uint32_t budget_ms)
{
  const uint32_t now_ms = radio_now_ms();
  const uint32_t wait_ms = (g_tx_quiet_until_ms > now_ms) ? (g_tx_quiet_until_ms - now_ms) : 0U;

  if (budget_ms == 0xFFFFFFFFUL) {
    return budget_ms;
  }

  return (wait_ms < budget_ms) ? (uint32_t)(budget_ms - wait_ms) : 0U;
}

uint8_t radio_uart_air_busy(void)
{
  return (radio_now_ms() < g_tx_quiet_until_ms) ? 1U : 0U;
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

  radio_uart_drop_stale_locked(radio_now_ms());
  count = g_tx_count;
  radio_uart_unlock_tx_queue();
  return count;
}

radio_uart_stats_t radio_uart_stats_snapshot(void)
{
  radio_uart_stats_t stats;
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
  stats.tx_queue_count = g_tx_count;
  stats.tx_drops = g_tx_drops;
  stats.tx_drop_oldest = g_tx_drop_oldest;
  stats.tx_drop_same_flow = g_tx_drop_same_flow;
  stats.tx_drop_stale = g_tx_drop_stale;
  stats.tx_budget_misses = g_tx_budget_misses;
  stats.aux_busy_count = g_aux_busy_count;
  stats.tx_dma_started = g_tx_dma_started;
  stats.tx_dma_complete = g_tx_dma_complete;
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
  const uint32_t now_ms = radio_now_ms();
  stats.tx_quiet_remaining_ms =
      (g_tx_quiet_until_ms > now_ms) ? (g_tx_quiet_until_ms - now_ms) : 0U;
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

#ifdef SEDS_FIRMWARE_SIM_TEST
  /* Renode's G4 USART model has no DMA request GPIO. The service loop polls
   * its real RDR register so framing still consumes actual UART traffic. */
  return HAL_OK;
#else
  HAL_StatusTypeDef status =
      HAL_UARTEx_ReceiveToIdle_DMA(g_huart, g_rx_buf, sizeof(g_rx_buf));
  if (status == HAL_OK && g_huart->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(g_huart->hdmarx, DMA_IT_HT);
  }
  return status;
#endif
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
  HAL_StatusTypeDef status;
  (void)priority;
  if (!g_huart) return HAL_ERROR;
  if (!bytes || len == 0U || len > RADIO_UART_MAX_PAYLOAD_SIZE) return HAL_ERROR;
  if (RFBOARD_RADIO_LISTEN_ONLY) return HAL_BUSY;
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

  /* SEDSNet has already scheduled these frames by logical priority and its
   * compact transport relies on their wire order (a full template must arrive
   * before compact frames that reference it).  A second asynchronous priority
   * queue here used to reorder/coalesce SDT frames, which made discovery vanish
   * at the GroundStation and added seconds of latency.  RFD900x accepts the
   * UART byte stream directly, so transmit synchronously and preserve exactly
   * the order selected by SEDSNet. RX DMA remains armed because STM32 UART has
   * independent transmit and receive state machines. */
  if (!radio_e22_ready_for_uart() || g_tx_dma_busy) {
    g_tx_busy++;
    return HAL_BUSY;
  }
  status = HAL_UART_Transmit(
      g_huart, framed, (uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len),
      radio_uart_tx_timeout_ms((uint16_t)(RADIO_UART_FRAME_HEADER_SIZE + len)));
  if (status == HAL_OK) {
    g_radio_tx_ok++;
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
  } else {
    g_tx_errors++;
    if (status == HAL_BUSY) {
      g_tx_busy++;
    }
  }
  return status;
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

  if (!radio_uart_tx_ready() || g_tx_dma_busy || !radio_e22_ready_for_uart()) {
    g_tx_startup_delays++;
    return 0U;
  }

  const uint32_t available_air_budget = radio_uart_available_air_budget(budget_ms);
  if (available_air_budget == 0U) {
    g_tx_budget_misses++;
    return 0U;
  }

  while (sent < RADIO_UART_TX_FRAMES_PER_SERVICE &&
         radio_uart_dequeue_frame_with_budget(&item, available_air_budget)) {
    if (!radio_uart_reserve_airtime(item.len, budget_ms)) {
      radio_uart_drop_item_stale_or_requeue_front(&item);
      return sent;
    }

    g_tx_dma_item = item;
#ifdef SEDS_FIRMWARE_SIM_TEST
    status = HAL_UART_Transmit(g_huart, g_tx_dma_item.data, g_tx_dma_item.len,
                               radio_uart_tx_timeout_ms(g_tx_dma_item.len));
    if (status != HAL_OK) {
      g_tx_errors++;
      break;
    }
    g_radio_tx_ok++;
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    sent++;
    if (radio_uart_start_rx() != HAL_OK) {
      g_rx_restart_errors++;
    }
#else
    g_tx_dma_busy = 1U;
    (void)HAL_UART_AbortReceive(g_huart);
    status = HAL_UART_Transmit_DMA(g_huart, g_tx_dma_item.data, g_tx_dma_item.len);
    if (status != HAL_OK) {
      g_tx_dma_busy = 0U;
      g_tx_errors++;
      if (status == HAL_BUSY) {
        g_tx_busy++;
      }
      (void)HAL_UART_AbortTransmit(g_huart);
      g_tx_quiet_until_ms = radio_now_ms();
      if (radio_uart_start_rx() != HAL_OK) {
        g_rx_restart_errors++;
      }
      break;
    }

    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    g_tx_dma_started++;
    sent++;
#endif
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

static void radio_process_framed_bytes(const uint8_t *data, size_t len)
{
  radio_frame_buf_append(data, len);

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
      g_rx_sync_loss++;
      if (g_frame_buf[g_frame_len - 1U] == RADIO_UART_FRAME_SYNC_0 ||
          g_frame_buf[g_frame_len - 1U] == RADIO_UART_COMMAND_SYNC_0 ||
          g_frame_buf[g_frame_len - 1U] == RADIO_UART_ASCII_SYNC_0) {
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
      return;
    }

    g_current_rx_is_command_frame = is_command_frame;
    g_last_frame_payload_len = (uint16_t)payload_len;
    g_last_frame_kind = is_command_frame ? 2U : (is_ascii_frame ? 3U : 1U);
    radio_uart_store_preview(g_last_frame_preview,
                             &g_last_frame_preview_len,
                             &g_frame_buf[RADIO_UART_FRAME_HEADER_SIZE],
                             payload_len);
    radio_notify_rx(&g_frame_buf[RADIO_UART_FRAME_HEADER_SIZE], payload_len);
    g_current_rx_is_command_frame = 0U;
    g_radio_rx_frames_ok++;
    radio_frame_buf_consume(RADIO_UART_FRAME_HEADER_SIZE + payload_len);
  }
}

/* Push one RX chunk into ring (ISR context) */
static inline void radio_rx_ring_push_isr(const uint8_t *data, uint16_t len)
{
  if (len == 0) return;
  if (len > RADIO_UART_RX_BUF_SIZE) len = RADIO_UART_RX_BUF_SIZE;

  /* If full: drop */
  if (g_rx_count >= RADIO_UART_RX_RING_DEPTH) {
    g_rx_isr_drops++;
    return;
  }

  uint32_t tail = g_rx_tail;
  g_rx_ring[tail].len = len;
  memcpy(g_rx_ring[tail].data, data, len);
  g_rx_isr_chunks++;
  g_rx_isr_bytes += len;

  tail++;
  if (tail >= RADIO_UART_RX_RING_DEPTH) tail = 0;
  g_rx_tail = tail;
  g_rx_count++;
}

/* Pop one RX chunk from ring (thread context) */
static inline uint8_t radio_rx_ring_pop_thread(radio_rx_item_t *out)
{
  uint8_t have = 0;

  /* Protect head/count against ISR updates */
  __disable_irq();
  if (g_rx_count > 0) {
    uint32_t head = g_rx_head;
    *out = g_rx_ring[head];

    head++;
    if (head >= RADIO_UART_RX_RING_DEPTH) head = 0;
    g_rx_head = head;
    g_rx_count--;
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
  radio_uart_sample_rx_pin();

#ifdef SEDS_FIRMWARE_SIM_TEST
  while (__HAL_UART_GET_FLAG(g_huart, UART_FLAG_RXNE) != RESET) {
    const uint8_t byte = (uint8_t)g_huart->Instance->RDR;
    radio_process_framed_bytes(&byte, 1U);
  }
#endif

  radio_rx_item_t item;
  while (radio_rx_ring_pop_thread(&item)) {
    radio_process_framed_bytes(item.data, (size_t)item.len);
  }
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
    radio_rx_ring_push_isr(g_rx_buf, Size);
  }

  /* Re-arm for next chunk */
  if (radio_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }
}

#ifdef SEDS_FIRMWARE_SIM_TEST
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (!g_huart || huart == NULL || huart->Instance != g_huart->Instance) return;
  g_rx_irq_events++;
  radio_rx_ring_push_isr(g_rx_buf, 1U);
  if (radio_uart_start_rx() != HAL_OK) g_rx_restart_errors++;
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!g_huart) return;
  if (huart->Instance != g_huart->Instance) return;

  g_tx_dma_busy = 0U;
  g_radio_tx_ok++;
  g_tx_dma_complete++;
  if (radio_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (!g_huart) return;
  if (huart->Instance != g_huart->Instance) return;

  g_rx_errors++;
  g_tx_dma_busy = 0U;
  (void)HAL_UART_AbortReceive(huart);
  (void)HAL_UART_AbortTransmit(huart);
  if (radio_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }
}
