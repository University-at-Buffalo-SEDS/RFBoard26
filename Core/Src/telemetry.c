#include "telemetry.h"

#include "app_threadx.h" // brings in tx_api.h usually
#include "can_bus.h"
#include "main.h"
#include "radio.h"
#include "sedsprintf.h"
#include "stm32g4xx_hal.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * TIMESYNC NOTE
 * -------------
 * TIME_SYNC_REQUEST handling is deferred:
 * - on_timesync only pushes response words into a local ring buffer.
 * - telemetry_timesync_process_queue() drains that buffer and queues router TX.
 *
 * This avoids callback->router re-entry.
 */

extern void telemetry_lock(void);
extern void telemetry_unlock(void);

#ifndef TELEMETRY_TIMESYNC_RESP_ENABLED
#define TELEMETRY_TIMESYNC_RESP_ENABLED 1
#endif

#ifndef TELEMETRY_TIMESYNC_RESP_TX_CAN
#define TELEMETRY_TIMESYNC_RESP_TX_CAN 0
#endif

#ifndef TELEMETRY_TIMESYNC_RESP_TX_RADIO
#define TELEMETRY_TIMESYNC_RESP_TX_RADIO 0
#endif

#ifndef TELEMETRY_TIMESYNC_DIAG_SKIP_SERIALIZE
#define TELEMETRY_TIMESYNC_DIAG_SKIP_SERIALIZE 1
#endif

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len)
{
  (void)data;
  (void)len;
}
#endif

#if defined(__GNUC__) || defined(__clang__)
#define UNUSED_FUNCTION __attribute__((unused))
#else
#define UNUSED_FUNCTION
#endif

static uint8_t g_can_rx_subscribed = 0;
static uint8_t g_radio_rx_subscribed = 0;
static int32_t g_can_side_id = -1;
static int32_t g_radio_side_id = -1;

/* Timesync diagnostic counters (defined later in this file). */
extern volatile uint32_t g_timesync_req_seen;
extern volatile uint32_t g_timesync_resp_enqueued;
extern volatile uint32_t g_timesync_resp_dequeued;
extern volatile uint32_t g_timesync_resp_ser_fail;
extern volatile uint32_t g_timesync_resp_can_fail;
extern volatile uint32_t g_timesync_resp_radio_fail;
extern volatile uint32_t g_timesync_resp_stage;
extern volatile uint32_t g_timesync_diag_fault_code;

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static int64_t g_master_offset_ms = 0;
static uint64_t g_last_delay_ms = 0;
static int64_t g_unix_base_ms = 0;
static uint8_t g_unix_valid = 0;

RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

static uint64_t tx_raw_now_ms_locked(void)
{
  const uint32_t ticks32 = (uint32_t)tx_time_get();
  return ((uint64_t)ticks32 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static UNUSED_FUNCTION uint64_t tx_raw_now_ms(void *user)
{
  (void)user;
  return tx_raw_now_ms_locked();
}

static uint64_t telemetry_now_ms_locked(void)
{
  int64_t t = (int64_t)tx_raw_now_ms_locked() + g_master_offset_ms;
  if (t < 0)
    t = 0;
  return (uint64_t)t;
}

uint64_t telemetry_now_ms(void)
{
  return telemetry_now_ms_locked();
}

uint64_t telemetry_unix_ms(void)
{
  if (!g_unix_valid)
    return 0;

  int64_t t = (int64_t)telemetry_now_ms_locked() + g_unix_base_ms;
  if (t < 0)
    t = 0;

  return (uint64_t)t;
}

uint64_t telemetry_unix_s(void)
{
  return telemetry_unix_ms() / 1000ULL;
}

uint8_t telemetry_unix_is_valid(void)
{
  return g_unix_valid;
}

void telemetry_set_unix_time_ms(uint64_t unix_ms)
{
#if TELEMETRY_TIME_MASTER
  const int64_t now = (int64_t)telemetry_now_ms_locked();
  if (unix_ms == 0ULL || unix_ms > (uint64_t)INT64_MAX)
    return;

  const int64_t new_base = (int64_t)unix_ms - now;
  if (new_base == 0)
    return; // zero base is reserved to indicate invalid/unset unix time

  g_unix_base_ms = new_base;
  g_unix_valid = 1U;
#else
  (void)unix_ms;
#endif
}

static uint64_t node_now_since_ms(void *user);

static void compute_offset_delay(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4,
                                 int64_t *offset_ms, uint64_t *delay_ms)
{
  const int64_t o = ((int64_t)(t2 - t1) + (int64_t)(t3 - t4)) / 2;
  const int64_t d = (int64_t)(t4 - t1) - (int64_t)(t3 - t2);
  *offset_ms = o;
  *delay_ms = (d < 0) ? 0 : (uint64_t)d;
}

#ifndef NET_TIMESYNC_MAX_STEP_MS
#define NET_TIMESYNC_MAX_STEP_MS 30000
#endif

#ifndef NET_TIMESYNC_SMOOTH_DIV
#define NET_TIMESYNC_SMOOTH_DIV 4
#endif

static UNUSED_FUNCTION void client_apply_offset_ms_locked(int64_t offset_ms)
{
  if (offset_ms > NET_TIMESYNC_MAX_STEP_MS || offset_ms < -NET_TIMESYNC_MAX_STEP_MS)
    return;

  int64_t step = offset_ms / (int64_t)NET_TIMESYNC_SMOOTH_DIV;
  if (step == 0)
  {
    if (offset_ms > 0)
      step = 1;
    else if (offset_ms < 0)
      step = -1;
  }

  g_master_offset_ms += step;
}

static SedsResult tx_send_can(const uint8_t *bytes, size_t len, void *user)
{
  (void)user;
  if (!bytes || len == 0)
    return SEDS_BAD_ARG;

  return (can_bus_send_large(bytes, len, 0x03) == HAL_OK) ? SEDS_OK : SEDS_IO;
}

static SedsResult tx_send_radio(const uint8_t *bytes, size_t len, void *user)
{
  (void)user;
  if (!bytes || len == 0)
    return SEDS_BAD_ARG;


  return (radio_uart_send_bytes(bytes, len) == HAL_OK) ? SEDS_OK : SEDS_IO;
}

/* Backward-compatible symbol for external references; maps to CAN side. */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user)
{
  return tx_send_can(bytes, len, user);
}

/* Deferred timesync response queue (master only). */
#if TELEMETRY_TIME_MASTER
#ifndef TIMESYNC_DEFERRED_QUEUE_DEPTH
#define TIMESYNC_DEFERRED_QUEUE_DEPTH 8
#endif
typedef struct
{
  uint64_t t3_for_ts;
  uint64_t resp_words[4];
} TimesyncDeferredResp;

static TX_QUEUE g_timesync_deferred_queue;
static ULONG g_timesync_deferred_queue_storage[TIMESYNC_DEFERRED_QUEUE_DEPTH *
                                               (sizeof(TimesyncDeferredResp) / sizeof(ULONG))];
static uint8_t g_timesync_deferred_queue_ready = 0U;

static void timesync_deferred_init_locked(void)
{
  if (g_timesync_deferred_queue_ready)
    return;
  if (tx_queue_create(&g_timesync_deferred_queue,
                      "timesync_resp_q",
                      (sizeof(TimesyncDeferredResp) / sizeof(ULONG)),
                      g_timesync_deferred_queue_storage,
                      sizeof(g_timesync_deferred_queue_storage)) == TX_SUCCESS)
  {
    g_timesync_deferred_queue_ready = 1U;
  }
}

static uint8_t timesync_deferred_push_locked(const TimesyncDeferredResp *item)
{
  if (!g_timesync_deferred_queue_ready || !item)
    return 0U;
  return (tx_queue_send(&g_timesync_deferred_queue, (VOID *)item, TX_NO_WAIT) == TX_SUCCESS) ? 1U : 0U;
}

static uint8_t timesync_deferred_pop_locked(TimesyncDeferredResp *out)
{
  if (!g_timesync_deferred_queue_ready || !out)
    return 0U;
  return (tx_queue_receive(&g_timesync_deferred_queue, out, TX_NO_WAIT) == TX_SUCCESS) ? 1U : 0U;
}

static UNUSED_FUNCTION void timesync_diag_set_leds(uint32_t stage)
{
  /* Stage-to-LED map (for no-debugger field triage):
   * 1: BLUE=ON,  GREEN=OFF
   * 2: BLUE=OFF, GREEN=ON
   * 3: BLUE=ON,  GREEN=ON
   * 4: BLUE=OFF, GREEN=OFF
   */
  GPIO_PinState blue = GPIO_PIN_RESET;
  GPIO_PinState green = GPIO_PIN_RESET;
  switch (stage)
  {
  case 1U:
    blue = GPIO_PIN_SET;
    green = GPIO_PIN_RESET;
    break;
  case 2U:
    blue = GPIO_PIN_RESET;
    green = GPIO_PIN_SET;
    break;
  case 3U:
    blue = GPIO_PIN_SET;
    green = GPIO_PIN_SET;
    break;
  case 4U:
  default:
    blue = GPIO_PIN_RESET;
    green = GPIO_PIN_RESET;
    break;
  }
  HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, blue);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, green);
}
#endif

/* Build and transmit a timesync response without routing back through router lock paths. */
static void timesync_send_response_direct(const TimesyncDeferredResp *item)
{
  (void)item;
}

void telemetry_timesync_process_queue(void)
{
  /*
   * Using library-recommended pattern:
   * enqueue TIME_SYNC_RESPONSE directly from on_timesync via
   * seds_router_log_queue_ts(), then normal router queue processing sends it.
   * Keep this function as a compatibility no-op for existing thread call sites.
   */
  return;
}

/* -------------------------------------------------------------------------- */
/* Timesync packet handler                                                    */
/* -------------------------------------------------------------------------- */

/* Optional: expose these if you want to observe sync state elsewhere */
static int64_t g_last_offset_ms = 0;
static uint32_t g_master_offset_valid = 0;

/* Tuning knobs */
#ifndef NET_TIMESYNC_STEP_THRESHOLD_MS
#define NET_TIMESYNC_STEP_THRESHOLD_MS 250 /* if |offset| >= this, step immediately */
#endif

#ifndef NET_TIMESYNC_MAX_STEP_MS
#define NET_TIMESYNC_MAX_STEP_MS 30000 /* safety clamp (already in your file) */
#endif

#ifndef NET_TIMESYNC_SMOOTH_DIV
#define NET_TIMESYNC_SMOOTH_DIV 4 /* slew = offset/div */
#endif

volatile uint32_t g_timesync_req_seen = 0U;
volatile uint32_t g_timesync_resp_enqueued = 0U;
volatile uint32_t g_timesync_resp_dequeued = 0U;
volatile uint32_t g_timesync_resp_ser_fail = 0U;
volatile uint32_t g_timesync_resp_can_fail = 0U;
volatile uint32_t g_timesync_resp_radio_fail = 0U;
volatile uint32_t g_timesync_resp_stage = 0U;
volatile uint32_t g_timesync_diag_fault_code = 0U; /* 0=none, 1=len, 2=serialize */

static SedsResult on_timesync(const SedsPacketView *pkt, void *user)
{
  (void)user;
  if (!pkt || !pkt->payload)
    return SEDS_ERR;
  /* ------------------------------------------------------------ */
  /* TIME_SYNC_RESPONSE: client receives server timestamps         */
  /* Payload: { seq, t1_client, t2_server, t3_server } (4x u64)    */
  /* ------------------------------------------------------------ */
  if (pkt->ty == SEDS_DT_TIME_SYNC_RESPONSE && pkt->payload_len >= 32)
  {
    uint64_t t1 = 0, t2 = 0, t3 = 0;

    /* NOTE: layout matches how master queues resp_words[] */
    memcpy(&t1, pkt->payload + 8, 8);
    memcpy(&t2, pkt->payload + 16, 8);
    memcpy(&t3, pkt->payload + 24, 8);

    /* t4 = client receive time, in the SAME local/raw clock domain as t1 */
    const uint64_t t4 = tx_raw_now_ms_locked();

    int64_t offset_ms = 0;
    uint64_t delay_ms = 0;
    compute_offset_delay(t1, t2, t3, t4, &offset_ms, &delay_ms);

    g_last_delay_ms = delay_ms;

#if !TELEMETRY_TIME_MASTER
    /* Safety clamp: ignore insane jumps */
    if (offset_ms > NET_TIMESYNC_MAX_STEP_MS || offset_ms < -NET_TIMESYNC_MAX_STEP_MS)
      return SEDS_OK;

    g_last_offset_ms = offset_ms;

    /*
     * Apply the offset to THIS board’s notion of "network time":
     * g_master_offset_ms is added to tx_raw_now_ms_locked() in telemetry_now_ms_locked().
     *
     * Policy:
     *  - Large offset => step immediately (converges fast)
     *  - Small offset => slew (reduces jitter)
     */
    int64_t applied = 0;

    if (offset_ms >= (int64_t)NET_TIMESYNC_STEP_THRESHOLD_MS ||
        offset_ms <= -(int64_t)NET_TIMESYNC_STEP_THRESHOLD_MS)
    {
      /* STEP: apply full correction */
      applied = offset_ms;
    }
    else
    {
      /* SLEW: apply fraction */
      applied = offset_ms / (int64_t)NET_TIMESYNC_SMOOTH_DIV;
      if (applied == 0)
        applied = (offset_ms > 0) ? 1 : (offset_ms < 0) ? -1
                                                        : 0;
    }

    if (applied != 0)
    {
      /*
       * If you maintain unix_base_ms relative to telemetry_now_ms_locked(),
       * then changing g_master_offset_ms would “jump” telemetry_unix_ms().
       * Compensate unix_base_ms by the same delta to keep unix time continuous.
       */
      if (g_unix_valid)
        g_unix_base_ms -= applied;

      g_master_offset_ms += applied;
      g_master_offset_valid = 1;
    }
#endif

    return SEDS_OK;
  }

  /* ------------------------------------------------------------ */
  /* TIME_SYNC_REQUEST: master receives client t1 and must respond */
  /* Payload: { seq, t1_client } (2x u64)                          */
  /* ------------------------------------------------------------ */
  if (pkt->ty == SEDS_DT_TIME_SYNC_REQUEST && pkt->payload_len >= 16)
  {
#if TELEMETRY_TIME_MASTER
#if TELEMETRY_TIMESYNC_RESP_ENABLED
    g_timesync_req_seen++;
    uint64_t seq = 0, t1 = 0;
    memcpy(&seq, pkt->payload + 0, 8);
    memcpy(&t1, pkt->payload + 8, 8);

    const uint64_t t2 = tx_raw_now_ms_locked();
    const uint64_t t3 = tx_raw_now_ms_locked();

    const uint64_t resp[4] = {seq, t1, t2, t3};
    if (g_router.r != NULL)
    {
      HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

      const SedsResult s = seds_router_log_queue_ts(
          g_router.r,
          SEDS_DT_TIME_SYNC_RESPONSE,
          t3,
          resp,
          4);
      if (s == SEDS_OK)
        g_timesync_resp_enqueued++;
      else
        g_timesync_resp_ser_fail++;
    }
      HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

#endif
    return SEDS_OK;
#else
    return SEDS_OK;
#endif
  }

  /* ------------------------------------------------------------ */
  /* TIME_SYNC_ANNOUNCE: master sends {priority, unix_ms}          */
  /* Payload: { priority, unix_ms } (2x u64)                       */
  /* ------------------------------------------------------------ */
  if (pkt->ty == SEDS_DT_TIME_SYNC_ANNOUNCE && pkt->payload_len >= 16)
  {
#if !TELEMETRY_TIME_MASTER
    uint64_t unix_ms = 0;
    memcpy(&unix_ms, pkt->payload + 8, 8);

    const uint64_t half_delay = g_last_delay_ms / 2ULL;
    const int64_t now = (int64_t)telemetry_now_ms_locked();

    g_unix_base_ms = (int64_t)(unix_ms + half_delay) - now;
    g_unix_valid = 1;
#endif
    return SEDS_OK;
  }

  return SEDS_OK;
}

static uint64_t node_now_since_ms(void *user)
{
  (void)user;
  const RouterState s = g_router;
  if (!s.r)
    return 0;

  if (telemetry_unix_is_valid())
    return telemetry_unix_ms();

  return telemetry_now_ms_locked();
}

static void telemetry_can_rx(const uint8_t *data, size_t len, void *user)
{
  (void)user;
  if (!data || len == 0)
    return;

  if (!g_router.r)
    return;

  if (g_can_side_id >= 0)
    (void)seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r, (uint32_t)g_can_side_id, data, len);
  else
    (void)seds_router_rx_serialized_packet_to_queue(g_router.r, data, len);
}

static void telemetry_radio_rx(const uint8_t *data, size_t len, void *user)
{
  (void)user;
  if (!data || len == 0)
    return;

  if (!g_router.r)
    return;

  if (g_radio_side_id >= 0)
    (void)seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r, (uint32_t)g_radio_side_id, data, len);
  else
    (void)seds_router_rx_serialized_packet_to_queue(g_router.r, data, len);
}

static SedsResult ensure_router_locked(void)
{
  if (g_router.created && g_router.r)
    return SEDS_OK;

  if (!g_can_rx_subscribed)
  {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK)
      g_can_rx_subscribed = 1;
    else
      printf("Error: can_bus_subscribe_rx failed\r\n");
  }

  if (!g_radio_rx_subscribed)
  {
    if (radio_uart_subscribe_rx(telemetry_radio_rx, NULL) == HAL_OK)
      g_radio_rx_subscribed = 1;
    else
      printf("Error: radio_uart_subscribe_rx failed\r\n");
  }

#ifndef TELEMETRY_ENABLED
  g_router.created = 1;
  return SEDS_OK;
#else
  const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = (uint32_t)SEDS_EP_TIME_SYNC,
          .packet_handler = on_timesync,
          .serialized_handler = NULL,
          .user = NULL,
      },
  };

  SedsRouter *r = seds_router_new(
      Seds_RM_Relay,
      node_now_since_ms,
      NULL,
      locals,
      (size_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r)
  {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  g_can_side_id = seds_router_add_side_serialized(r, "can", 3, tx_send_can, NULL, false);
  g_radio_side_id = seds_router_add_side_serialized(r, "radio", 5, tx_send_radio, NULL, false);

  if (g_can_side_id < 0)
  {
    printf("Error: failed to add CAN side: %ld\r\n", (long)g_can_side_id);
    g_can_side_id = -1;
  }

  if (g_radio_side_id < 0)
  {
    printf("Error: failed to add RADIO side: %ld\r\n", (long)g_radio_side_id);
    g_radio_side_id = -1;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = telemetry_unix_is_valid() ? telemetry_unix_ms()
                                                  : telemetry_now_ms_locked();
#if TELEMETRY_TIME_MASTER
  g_master_offset_ms = 0;
  timesync_deferred_init_locked();
#endif

  return SEDS_OK;
#endif
}

void rx_asynchronous(const uint8_t *bytes, size_t len)
{
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0)
    return;

  if (!g_router.r)
    return;

  SedsRouter *r = g_router.r;
  int32_t side_id = g_can_side_id;

  if (side_id >= 0)
    (void)seds_router_rx_serialized_packet_to_queue_from_side(r, (uint32_t)side_id, bytes, len);
  else
    (void)seds_router_rx_serialized_packet_to_queue(r, bytes, len);
#endif
}

static UNUSED_FUNCTION void rx_synchronous(const uint8_t *bytes, size_t len)
{
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0)
    return;

  if (!g_router.r)
    return;

  SedsRouter *r = g_router.r;
  int32_t side_id = g_can_side_id;

  if (side_id >= 0)
    (void)seds_router_receive_serialized_from_side(r, (uint32_t)side_id, bytes, len);
  else
    (void)seds_router_receive_serialized(r, bytes, len);
#endif
}

static UNUSED_FUNCTION uint64_t g_timesync_seq = 1;

SedsResult telemetry_timesync_request(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
#if TELEMETRY_TIME_MASTER
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  const uint64_t t1 = tx_raw_now_ms_locked();
  const uint64_t req[2] = {g_timesync_seq++, t1};
  return seds_router_log_queue_ts(g_router.r, SEDS_DT_TIME_SYNC_REQUEST, t1, req, 2);
#endif
#endif
}

SedsResult telemetry_timesync_announce(uint64_t priority, uint64_t unix_ms)
{
#ifndef TELEMETRY_ENABLED
  (void)priority;
  (void)unix_ms;
  return SEDS_OK;
#else
#if !TELEMETRY_TIME_MASTER
  (void)priority;
  (void)unix_ms;
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  const uint64_t t = telemetry_unix_ms();
  const uint64_t announce[2] = {priority, unix_ms};
  return seds_router_log_queue_ts(g_router.r, SEDS_DT_TIME_SYNC_ANNOUNCE, t, announce, 2);
#endif
#endif
}

SedsResult init_telemetry_router(void)
{
  return ensure_router_locked();
}

static inline SedsElemKind guess_kind_from_elem_size(size_t elem_size)
{
  if (elem_size == 4 || elem_size == 8)
    return SEDS_EK_FLOAT;
  return SEDS_EK_UNSIGNED;
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size)
{
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_BAD_ARG;

  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  const SedsElemKind kind = guess_kind_from_elem_size(element_size);
  /* Queue-only model: producers enqueue, telemetry thread drains/dispatches. */
  return seds_router_log_typed_ex(g_router.r, data_type, data, element_count,
                                  element_size, kind, NULL, 1);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size)
{
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_BAD_ARG;

  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  const SedsElemKind kind = guess_kind_from_elem_size(element_size);
  return seds_router_log_typed_ex(g_router.r, data_type, data, element_count,
                                  element_size, kind, NULL, 1);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult dispatch_tx_queue(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult log_error_asynchronous(const char *fmt, ...)
{
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  va_list args;
  va_start(args, fmt);
  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy);
  va_end(args_copy);

  if (len < 0)
  {
    va_end(args);
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 1);
  }

  if (len > 512)
    len = 512;

  char buf[(size_t)len + 1];
  int written = vsnprintf(buf, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0)
  {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 1);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, buf, (size_t)written, NULL, 1);
#endif
}

SedsResult log_error_synchronous(const char *fmt, ...)
{
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  if (ensure_router_locked() != SEDS_OK)
    return SEDS_ERR;

  va_list args;
  va_start(args, fmt);
  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy);
  va_end(args_copy);

  if (len < 0)
  {
    va_end(args);
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 1);
  }

  if (len > 512)
    len = 512;

  char buf[(size_t)len + 1];
  int written = vsnprintf(buf, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0)
  {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0, NULL, 1);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, buf, (size_t)written, NULL, 1);
#endif
}

SedsResult print_telemetry_error(const int32_t error_code)
{
#ifndef TELEMETRY_ENABLED
  (void)error_code;
  return SEDS_OK;
#else
  const int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0)
    return (SedsResult)need;

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK)
    printf("Error: %s\r\n", buf);
  else
    (void)log_error_asynchronous("Error: seds_error_to_string failed: %d\r\n", (int)res);

  return res;
#endif
}

void die(const char *fmt, ...)
{
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  while (1)
  {
    printf("FATAL: %s\r\n", buf);
    HAL_Delay(1000);
  }
}
