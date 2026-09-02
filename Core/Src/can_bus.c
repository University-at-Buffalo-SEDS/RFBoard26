// can_bus.c
//
// CAN / CAN-FD helper with:
//  - Subscriber fanout
//  - Correct HAL DLC handling (DataLength is a DLC code, not byte count)
//  - Optional fragmentation/reassembly layer so you can send >64B buffers
//  - ISR does *minimal work*: drains HW FIFO into a lock-free ring buffer
//  - Thread/main-loop calls can_bus_process_rx() to reassemble + notify
//    subscribers
//
// Fixes for “polling mode receives nothing” / “all boards same firmware”:
//  1) Drain BOTH RX FIFO0 and FIFO1 in polling (and ISR support for FIFO0 too).
//  2) Configure an accept-all standard filter (and global filter) so frames
//     aren’t rejected or routed away.
//  3) Enable FIFO0 notifications when not polling.
//
// Notes / Assumptions:
//  - Uses CAN FD frames for fragmentation (default payload 64 bytes).
//  - Fragment frames are distinguished by a small "magic" header in the
//    payload.
//  - Reassembly is bounded (no malloc). Oldest RX frames are dropped on ring
//    overflow.
//  - One producer (ISR) and one consumer (thread calling can_bus_process_rx()).
//  - You can call can_bus_process_rx() from a ThreadX thread, or main
//    superloop.
//
// IMPORTANT CONCURRENCY NOTE:
//  `volatile` head/tail alone does NOT guarantee publish/consume ordering for
//  the ring slot contents by the C language rules. We add `__DMB()` barriers to
//  ensure the slot is fully written before publishing `head` (release), and
//  ensure the consumer sees the slot contents after observing `head` (acquire).

#include "can_bus.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32g4xx_hal.h"

#ifndef CAN_BUS_DEBUG
#define CAN_BUS_DEBUG 0
#endif

#ifndef CAN_BUS_POLLING
#define CAN_BUS_POLLING 0
#endif

#ifndef CAN_BUS_TX_ENQUEUE_TIMEOUT_MS
#define CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 50U
#endif

/* A continuously busy CAN bus must not monopolize the telemetry thread.  A
 * bounded service pass leaves time to drain the much slower radio, service
 * SEDSNet timers, and process commands. */
#ifndef CAN_BUS_RX_SERVICE_BUDGET
#define CAN_BUS_RX_SERVICE_BUDGET 32U
#endif

#define UNUSED_FUNCTION __attribute__((unused))

/* Forward declarations (avoid implicit decl / linkage mismatch) */
static uint32_t can_bus_drain_rx_fifo(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t fifo, uint32_t budget);
static UNUSED_FUNCTION void can_bus_drain_tx_events(FDCAN_HandleTypeDef *hfdcan);

static void can_bus_debug_print(const char *fmt, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n > 0)
  {
    /* Print via printf (USB CDC) */
    (void)printf("%s", buf);
  }
}

#define CAN_BUS_DBG(...) \
  do                     \
  {                      \
    if (CAN_BUS_DEBUG)   \
      can_bus_debug_print(__VA_ARGS__); \
  } while (0)

/* Dump up to N bytes of a CAN frame payload for debug. Called from thread
   context only (cheap string formatting allowed). */
static void can_bus_dbg_dump_bytes(const uint8_t *data, size_t len)
{
  char buf[128];
  size_t pos = 0;
  size_t to = (len < 16) ? len : 16; /* limit output */
  for (size_t i = 0; i < to; ++i)
  {
    int n = snprintf(&buf[pos], sizeof(buf) - pos, "%02X%s", data[i], (i + 1 < to) ? " " : "");
    if (n <= 0)
      break;
    pos += (size_t)n;
    if (pos >= sizeof(buf) - 1)
      break;
  }
  if (len > to && pos < sizeof(buf) - 5)
  {
    snprintf(&buf[pos], sizeof(buf) - pos, " ...");
  }
  CAN_BUS_DBG(" data=%s\r\n", buf);
}

// CMSIS barrier intrinsics (for __DMB()).
#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"
#endif

#ifndef CAN_BUS_MAX_SUBSCRIBERS
#define CAN_BUS_MAX_SUBSCRIBERS 8
#endif

// =========================
// FD DLC helpers
// =========================

static size_t can_bus_dlc_to_len(uint32_t dlc)
{
  static const uint8_t map[16] = {0, 1, 2, 3, 4, 5, 6, 7,
                                  8, 12, 16, 20, 24, 32, 48, 64};
  dlc &= 0xF;
  return map[dlc];
}

static uint32_t can_bus_len_to_dlc(size_t len)
{
  switch (len)
  {
  case 0:
    return FDCAN_DLC_BYTES_0;
  case 1:
    return FDCAN_DLC_BYTES_1;
  case 2:
    return FDCAN_DLC_BYTES_2;
  case 3:
    return FDCAN_DLC_BYTES_3;
  case 4:
    return FDCAN_DLC_BYTES_4;
  case 5:
    return FDCAN_DLC_BYTES_5;
  case 6:
    return FDCAN_DLC_BYTES_6;
  case 7:
    return FDCAN_DLC_BYTES_7;
  case 8:
    return FDCAN_DLC_BYTES_8;
  case 12:
    return FDCAN_DLC_BYTES_12;
  case 16:
    return FDCAN_DLC_BYTES_16;
  case 20:
    return FDCAN_DLC_BYTES_20;
  case 24:
    return FDCAN_DLC_BYTES_24;
  case 32:
    return FDCAN_DLC_BYTES_32;
  case 48:
    return FDCAN_DLC_BYTES_48;
  case 64:
    return FDCAN_DLC_BYTES_64;
  default:
    return 0xFFFFFFFFu;
  }
}

static size_t can_bus_round_up_fd_len(size_t len)
{
  if (len <= 8)
    return len;
  if (len <= 12)
    return 12;
  if (len <= 16)
    return 16;
  if (len <= 20)
    return 20;
  if (len <= 24)
    return 24;
  if (len <= 32)
    return 32;
  if (len <= 48)
    return 48;
  if (len <= 64)
    return 64;
  return 64;
}

// =========================
// Subscriber fanout
// =========================

typedef struct
{
  can_bus_rx_cb_t cb;
  void *user;
} can_bus_sub_t;

static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static can_bus_sub_t g_subs[CAN_BUS_MAX_SUBSCRIBERS];

/* Mask of notifications we activate (0 if polling-only). Stored so poll
  can temporarily disable/restore notifications to avoid IRQ/poll races. */
static uint32_t g_notification_mask = 0;

/* Own standard ID: if set (not 0xFFFFFFFF) we drop received frames with this
  ID to avoid seeing our own transmissions on a shared bus. */
static uint32_t g_own_std_id = 0xFFFFFFFFu;

/* IRQ counter incremented by ISR to help debug whether interrupts fire. */
volatile uint32_t g_fdcan_irq_count __attribute__((used, externally_visible)) = 0;
volatile uint32_t g_fdcan_rx_count __attribute__((used, externally_visible)) = 0;
volatile uint32_t g_fdcan_tx_ok_count __attribute__((used, externally_visible)) = 0;
volatile uint32_t g_fdcan_tx_fail_count __attribute__((used, externally_visible)) = 0;

/*
 * RX/TX pending flags set from ISR.
 * Keep ISRs constant-time: they only set these flags.
 * Thread context (can_bus_process_rx) drains HW FIFOs into the ring.
 */
static volatile uint8_t g_rx_fifo0_pending = 0;
static volatile uint8_t g_rx_fifo1_pending = 0;
static volatile uint8_t g_txevt_pending = 0;

static HAL_StatusTypeDef can_bus_recover_if_bus_off(void)
{
  if (!g_hfdcan)
    return HAL_ERROR;

  FDCAN_ProtocolStatusTypeDef status;
  if (HAL_FDCAN_GetProtocolStatus(g_hfdcan, &status) != HAL_OK)
    return HAL_ERROR;

  if (status.BusOff == 0U)
  {
    if (__HAL_FDCAN_GET_FLAG(g_hfdcan, FDCAN_FLAG_BUS_OFF) != 0U)
      __HAL_FDCAN_CLEAR_FLAG(g_hfdcan, FDCAN_FLAG_BUS_OFF);
    return HAL_OK;
  }

  if (HAL_FDCAN_Stop(g_hfdcan) != HAL_OK)
    return HAL_ERROR;

  if (HAL_FDCAN_Start(g_hfdcan) != HAL_OK)
    return HAL_ERROR;

  if (__HAL_FDCAN_GET_FLAG(g_hfdcan, FDCAN_FLAG_BUS_OFF) != 0U)
    __HAL_FDCAN_CLEAR_FLAG(g_hfdcan, FDCAN_FLAG_BUS_OFF);

  return HAL_OK;
}

static HAL_StatusTypeDef can_bus_enqueue_tx_frame(const FDCAN_TxHeaderTypeDef *hdr,
                                                  const uint8_t *data)
{
  if (!g_hfdcan || !hdr || !data)
    return HAL_ERROR;

  const uint32_t t0 = HAL_GetTick();
  for (;;)
  {
    if (can_bus_recover_if_bus_off() != HAL_OK)
      return HAL_ERROR;

    if (HAL_FDCAN_GetTxFifoFreeLevel(g_hfdcan) > 0U)
    {
      HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, hdr, data);
      if (st == HAL_OK)
      {
        g_fdcan_tx_ok_count++;
        return HAL_OK;
      }

      /*
       * Retry only when queue is temporarily full; for all other errors,
       * return immediately.
       */
      const uint32_t err = HAL_FDCAN_GetError(g_hfdcan);
      if ((err & HAL_FDCAN_ERROR_FIFO_FULL) == 0U)
      {
        g_fdcan_tx_fail_count++;
        return st;
      }
    }

    if ((uint32_t)(HAL_GetTick() - t0) >= (uint32_t)CAN_BUS_TX_ENQUEUE_TIMEOUT_MS)
    {
      g_fdcan_tx_fail_count++;
      return HAL_TIMEOUT;
    }
  }
}

static inline void can_bus_notify_rx(const uint8_t *data, size_t len)
{
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++)
  {
    can_bus_rx_cb_t cb = g_subs[i].cb;
    if (cb)
      cb(data, len, g_subs[i].user);
  }
}

// =========================
// Fragmentation protocol
// =========================

#define CAN_BUS_FRAG_MAGIC ((uint16_t)('S') | ((uint16_t)('D') << 8)) /* 'S''D' in little-endian byte order */
#define CAN_BUS_NODE_ID 1U
#define CAN_BUS_FRAG_WIRE_LEN 64                                      // always send 64B payload frames for frags
#define CAN_BUS_REASM_TIMEOUT_MS 1000u                                // tolerate interleaved multi-sender packets

typedef struct __attribute__((packed))
{
  uint16_t magic;     // CAN_BUS_FRAG_MAGIC
  uint8_t source;     // stable sender token
  uint8_t seq;        // message sequence (wrap OK)
  uint8_t frag_idx;   // 0..frag_cnt-1
  uint8_t frag_cnt;   // total fragments
  uint8_t flags;      // bit0=first, bit1=last (optional)
  uint16_t total_len; // total bytes of reassembled message
} can_bus_frag_hdr_t;

enum
{
  CAN_BUS_FRAG_F_FIRST = 1u << 0,
  CAN_BUS_FRAG_F_LAST = 1u << 1
};

// =========================
// RX ring buffer (ISR -> thread)
// =========================

#ifndef CAN_BUS_RX_RING_DEPTH
#define CAN_BUS_RX_RING_DEPTH 64
#endif

typedef struct
{
  uint32_t std_id; // 11-bit ID in lower bits (we only handle standard here)
  uint8_t len;     // payload bytes (0..64)
  uint8_t data[64];
} can_bus_rx_frame_t;

static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static can_bus_rx_frame_t g_rx_ring[CAN_BUS_RX_RING_DEPTH];

// Small TX event ring (ISR -> thread) for debug printing
#ifndef CAN_BUS_TX_EVT_RING_DEPTH
#define CAN_BUS_TX_EVT_RING_DEPTH 16
#endif

typedef struct
{
  uint32_t id;
  uint32_t event_type;
  uint32_t timestamp;
  uint8_t data_len;
} can_bus_tx_event_t;

static volatile uint16_t g_tx_head = 0;
static volatile uint16_t g_tx_tail = 0;
static can_bus_tx_event_t g_tx_ring[CAN_BUS_TX_EVT_RING_DEPTH];

static inline uint16_t tx_rb_next(uint16_t v)
{
  v++;
  if (v >= CAN_BUS_TX_EVT_RING_DEPTH)
    v = 0;
  return v;
}

static inline int tx_rb_is_full(void) { return tx_rb_next(g_tx_head) == g_tx_tail; }

// push from ISR: drop-oldest on overflow
static inline void tx_rb_push_drop_oldest(uint32_t id, uint32_t evt, uint32_t ts, uint8_t len)
{
  if (tx_rb_is_full())
  {
    g_tx_tail = tx_rb_next(g_tx_tail);
  }
  uint16_t h = g_tx_head;
  g_tx_ring[h].id = id;
  g_tx_ring[h].event_type = evt;
  g_tx_ring[h].timestamp = ts;
  g_tx_ring[h].data_len = len;
  __DMB();
  g_tx_head = tx_rb_next(h);
}

// pop in thread context
static inline int tx_rb_pop(can_bus_tx_event_t *out)
{
  uint16_t t = g_tx_tail;
  uint16_t h = g_tx_head;
  if (h == t)
    return 0;
  __DMB();
  *out = g_tx_ring[t];
  __DMB();
  g_tx_tail = tx_rb_next(t);
  return 1;
}

static inline uint16_t rb_next(uint16_t v)
{
  v++;
  if (v >= CAN_BUS_RX_RING_DEPTH)
    v = 0;
  return v;
}

static inline int __attribute__((unused)) rb_is_empty(void)
{
  return g_rx_head == g_rx_tail;
}

static inline int rb_is_full(void) { return rb_next(g_rx_head) == g_rx_tail; }

// Push frame from ISR. Drop-oldest on overflow.
static inline void rb_push_drop_oldest(uint32_t std_id, const uint8_t *data, uint8_t len)
{
  if (len > 64)
    len = 64;

  if (rb_is_full())
  {
    // drop oldest
    g_rx_tail = rb_next(g_rx_tail);
  }

  uint16_t h = g_rx_head;

  g_rx_ring[h].std_id = std_id;
  g_rx_ring[h].len = len;
  memcpy(g_rx_ring[h].data, data, len);

  __DMB(); // publish slot before updating head (release)
  g_rx_head = rb_next(h);
}

// Pop frame in thread context
static inline int rb_pop(can_bus_rx_frame_t *out)
{
  uint16_t t = g_rx_tail;
  uint16_t h = g_rx_head;

  if (h == t)
    return 0;

  __DMB(); // acquire
  *out = g_rx_ring[t];
  __DMB(); // conservative
  g_rx_tail = rb_next(t);
  return 1;
}

// =========================
// HW FIFO drain
// =========================

/*
 * Drain hardware RX FIFOs (and TX event FIFO for debug) into the software
 * rings. Interrupt callbacks do this immediately; the thread drain is a
 * fallback for polling mode or any missed notification.
 *
 * IMPORTANT:
 *  - Reassembly and subscriber callbacks still happen only in thread context.
 *  - Thread-side fallback masks IRQs while touching the FIFO/ring so the ring
 *    has a single producer at a time.
 */
static void can_bus_drain_hw_to_ring_thread(void)
{
  if (!g_hfdcan)
    return;

  __disable_irq();

  /* RX FIFO0 */
  if (g_rx_fifo0_pending || (HAL_FDCAN_GetRxFifoFillLevel(g_hfdcan, FDCAN_RX_FIFO0) > 0))
  {
    g_rx_fifo0_pending = 0;
    (void)can_bus_drain_rx_fifo(g_hfdcan, FDCAN_RX_FIFO0,
                                CAN_BUS_RX_SERVICE_BUDGET);
    g_rx_fifo0_pending =
        HAL_FDCAN_GetRxFifoFillLevel(g_hfdcan, FDCAN_RX_FIFO0) > 0U;
  }

  /* RX FIFO1 */
  if (g_rx_fifo1_pending || (HAL_FDCAN_GetRxFifoFillLevel(g_hfdcan, FDCAN_RX_FIFO1) > 0))
  {
    g_rx_fifo1_pending = 0;
    (void)can_bus_drain_rx_fifo(g_hfdcan, FDCAN_RX_FIFO1,
                                CAN_BUS_RX_SERVICE_BUDGET);
    g_rx_fifo1_pending =
        HAL_FDCAN_GetRxFifoFillLevel(g_hfdcan, FDCAN_RX_FIFO1) > 0U;
  }

#if CAN_BUS_DEBUG
  /* TX event FIFO */
  if (g_txevt_pending)
  {
    g_txevt_pending = 0;
    can_bus_drain_tx_events(g_hfdcan);
  }
#else
  (void)g_txevt_pending;
#endif

  __enable_irq();
}

// =========================
// Reassembly state
// =========================

#ifndef CAN_BUS_REASM_SLOTS
#define CAN_BUS_REASM_SLOTS 4
#endif

#ifndef CAN_BUS_REASM_MAX_BYTES
#define CAN_BUS_REASM_MAX_BYTES 2048
#endif

#ifndef CAN_BUS_REASM_MAX_FRAGS
#define CAN_BUS_REASM_MAX_FRAGS 64
#endif

typedef struct
{
  uint8_t active;
  uint32_t std_id; // which CAN ID this slot is for
  uint8_t source;
  uint8_t seq;
  uint8_t frag_cnt;
  uint16_t total_len;
  uint8_t data_cap; // payload bytes per frag (wire_len - hdr)
  uint32_t last_tick_ms;
  uint64_t got_mask[(CAN_BUS_REASM_MAX_FRAGS + 63) / 64];
  uint16_t got_count;
  uint8_t buf[CAN_BUS_REASM_MAX_BYTES];
} can_bus_reasm_slot_t;

static can_bus_reasm_slot_t g_reasm[CAN_BUS_REASM_SLOTS];

static void reasm_reset(can_bus_reasm_slot_t *s)
{
  s->active = 0;
  s->std_id = 0;
  s->source = 0;
  s->seq = 0;
  s->frag_cnt = 0;
  s->total_len = 0;
  s->data_cap = 0;
  s->last_tick_ms = 0;
  s->got_count = 0;
  memset(s->got_mask, 0, sizeof(s->got_mask));
}

static can_bus_reasm_slot_t *reasm_get_slot(uint32_t std_id, uint8_t source, uint8_t seq, uint32_t now_ms)
{
  // First try to find existing active slot for std_id
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++)
  {
    if (g_reasm[i].active && g_reasm[i].std_id == std_id && g_reasm[i].source == source)
    {
      // If sequence changed, drop partial and reuse slot
      if (g_reasm[i].seq != seq)
      {
        reasm_reset(&g_reasm[i]);
        g_reasm[i].active = 1;
        g_reasm[i].std_id = std_id;
        g_reasm[i].source = source;
        g_reasm[i].seq = seq;
      }
      g_reasm[i].last_tick_ms = now_ms;
      return &g_reasm[i];
    }
  }

  // Find a free slot
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++)
  {
    if (!g_reasm[i].active)
    {
      reasm_reset(&g_reasm[i]);
      g_reasm[i].active = 1;
      g_reasm[i].std_id = std_id;
      g_reasm[i].source = source;
      g_reasm[i].seq = seq;
      g_reasm[i].last_tick_ms = now_ms;
      return &g_reasm[i];
    }
  }

  // No free slot: drop the stalest slot (oldest last_tick_ms)
  unsigned stalest = 0;
  uint32_t best_age = 0;
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++)
  {
    uint32_t age = (uint32_t)(now_ms - g_reasm[i].last_tick_ms);
    if (age >= best_age)
    {
      best_age = age;
      stalest = i;
    }
  }
  reasm_reset(&g_reasm[stalest]);
  g_reasm[stalest].active = 1;
  g_reasm[stalest].std_id = std_id;
  g_reasm[stalest].source = source;
  g_reasm[stalest].seq = seq;
  g_reasm[stalest].last_tick_ms = now_ms;
  return &g_reasm[stalest];
}

static inline int bit_test(uint64_t *mask, uint16_t idx)
{
  uint16_t w = (uint16_t)(idx / 64);
  uint16_t b = (uint16_t)(idx % 64);
  return (mask[w] >> b) & 1u;
}

static inline void bit_set(uint64_t *mask, uint16_t idx)
{
  uint16_t w = (uint16_t)(idx / 64);
  uint16_t b = (uint16_t)(idx % 64);
  mask[w] |= (1ull << b);
}

static void reasm_expire_old(uint32_t now_ms)
{
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++)
  {
    if (!g_reasm[i].active)
      continue;
    if ((uint32_t)(now_ms - g_reasm[i].last_tick_ms) > CAN_BUS_REASM_TIMEOUT_MS)
    {
      reasm_reset(&g_reasm[i]);
    }
  }
}

// Handle one RX frame (thread context)
static void handle_rx_frame(const can_bus_rx_frame_t *f, uint32_t now_ms)
{
  // Check if this is a fragment frame
  if (f->len >= sizeof(can_bus_frag_hdr_t))
  {
    can_bus_frag_hdr_t hdr;
    memcpy(&hdr, f->data, sizeof(hdr));

    if (hdr.magic == CAN_BUS_FRAG_MAGIC)
    {
      // Validate header fields
      if (hdr.frag_cnt == 0)
        return;
      if (hdr.frag_idx >= hdr.frag_cnt)
        return;
      if (hdr.frag_cnt > CAN_BUS_REASM_MAX_FRAGS)
        return;
      if (hdr.total_len == 0)
        return;
      if (hdr.total_len > CAN_BUS_REASM_MAX_BYTES)
        return;

      const uint8_t *payload = f->data + sizeof(hdr);
      const uint8_t payload_len = (uint8_t)(f->len - sizeof(hdr));

      can_bus_reasm_slot_t *s = reasm_get_slot(f->std_id, hdr.source, hdr.seq, now_ms);

      // If slot was newly created (or reset), initialize message params
      if (s->frag_cnt == 0)
      {
        s->frag_cnt = hdr.frag_cnt;
        s->total_len = hdr.total_len;
        s->data_cap = payload_len; // bytes per fragment (as seen on first frag)
        s->got_count = 0;
        memset(s->got_mask, 0, sizeof(s->got_mask));
      }
      else
      {
        // Must match the in-flight message properties
        if (s->frag_cnt != hdr.frag_cnt || s->total_len != hdr.total_len)
        {
          reasm_reset(s);
          return;
        }
      }

      // Compute where this fragment’s payload should land
      uint32_t off = (uint32_t)hdr.frag_idx * (uint32_t)s->data_cap;
      if (off >= s->total_len)
        return;

      uint32_t take = payload_len;
      if (off + take > s->total_len)
        take = (uint32_t)s->total_len - off;

      // Mark + copy if not already received
      if (!bit_test(s->got_mask, hdr.frag_idx))
      {
        bit_set(s->got_mask, hdr.frag_idx);
        s->got_count++;
        memcpy(&s->buf[off], payload, take);
      }

      s->last_tick_ms = now_ms;

      // Complete?
      if (s->got_count == s->frag_cnt)
      {
        CAN_BUS_DBG("CAN REASM DONE: id=0x%03lx len=%u\r\n",
                    (unsigned long)f->std_id, (unsigned)s->total_len);
        can_bus_notify_rx(s->buf, s->total_len);
        reasm_reset(s);
      }

      return;
    }
  }

  // Not a fragment frame: deliver raw CAN payload
  CAN_BUS_DBG("CAN RX: id=0x%03lx len=%u\r\n", (unsigned long)f->std_id, (unsigned)f->len);
  can_bus_notify_rx(f->data, f->len);
}

// =========================
// RX drain helpers (used by polling and ISR wrappers)
// =========================

static uint32_t can_bus_drain_rx_fifo(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t fifo, uint32_t budget)
{
  FDCAN_RxHeaderTypeDef hdr;
  uint8_t data[64];
  uint32_t drained = 0U;

  while (drained < budget && HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &hdr, data) != HAL_OK)
      break;

    drained++;
    g_fdcan_rx_count++;

    uint32_t std_id = hdr.Identifier & 0x7FFu;

    // Drop our own frames if configured
    if (g_own_std_id != 0xFFFFFFFFu && std_id == g_own_std_id)
      continue;

    size_t len = can_bus_dlc_to_len(hdr.DataLength);
    if (len > 64)
      len = 64;

    rb_push_drop_oldest(std_id, data, (uint8_t)len);
  }
  return drained;
}

static UNUSED_FUNCTION void can_bus_drain_tx_events(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_TxEventFifoTypeDef ev;
  while (HAL_FDCAN_GetTxEvent(hfdcan, &ev) == HAL_OK)
  {
    uint32_t id = ev.Identifier & 0x7FFu;
    uint32_t evt = ev.EventType;
    uint32_t ts = HAL_GetTick();
    uint8_t len = (uint8_t)can_bus_dlc_to_len(ev.DataLength);
    tx_rb_push_drop_oldest(id, evt, ts, len);
  }
}

// =========================
// Filter config (accept-all default)
// =========================
//
// If CubeMX already sets up filters, you can remove/adjust this, but
// having it here prevents the common “receives nothing” failure when
// no filters are configured or frames route to FIFO0 by default.

static void can_bus_config_accept_all(FDCAN_HandleTypeDef *hfdcan)
{
  // Global filter: accept non-matching frames into FIFO0.
  (void)HAL_FDCAN_ConfigGlobalFilter(
      hfdcan,
      FDCAN_ACCEPT_IN_RX_FIFO0, // Non-matching STD
      FDCAN_ACCEPT_IN_RX_FIFO0, // Non-matching EXT
      FDCAN_REJECT_REMOTE,      // Reject remote STD
      FDCAN_REJECT_REMOTE);     // Reject remote EXT

  // Standard ID mask filter: accept all IDs -> FIFO0.
  FDCAN_FilterTypeDef f;
  memset(&f, 0, sizeof(f));
  f.IdType = FDCAN_STANDARD_ID;
  f.FilterIndex = 0;
  f.FilterType = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1 = 0x000; // ID
  f.FilterID2 = 0x000; // mask (0 => don't care => accept all)
  (void)HAL_FDCAN_ConfigFilter(hfdcan, &f);
}

// =========================
// Public API
// =========================

void can_bus_init(FDCAN_HandleTypeDef *hfdcan)
{
  g_hfdcan = hfdcan;

  // reset rings + reasm
  g_rx_head = 0;
  g_rx_tail = 0;
  g_tx_head = 0;
  g_tx_tail = 0;
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++)
    reasm_reset(&g_reasm[i]);

  // Ensure we can actually receive frames even if CubeMX didn’t generate filters
  can_bus_config_accept_all(hfdcan);

  g_notification_mask = 0;

#if defined(CAN_BUS_POLLING) && (CAN_BUS_POLLING != 0)
  /* Polling mode: do not activate HAL notifications / IRQs. Caller should
     call `can_bus_process_rx()` periodically from a thread.
     */
#else
  // Enable BOTH FIFO0 and FIFO1 notifications (common default routing is FIFO0).
  g_notification_mask = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                        FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                        FDCAN_IT_TX_EVT_FIFO_NEW_DATA;

  (void)HAL_FDCAN_ConfigInterruptLines(hfdcan, g_notification_mask, FDCAN_INTERRUPT_LINE1 | FDCAN_INTERRUPT_LINE0);
  (void)HAL_FDCAN_ActivateNotification(hfdcan, g_notification_mask, 0);
#endif

  (void)HAL_FDCAN_Start(hfdcan);
}

HAL_StatusTypeDef can_bus_subscribe_rx(can_bus_rx_cb_t cb, void *user)
{
  if (!cb)
    return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++)
  {
    if (g_subs[i].cb == cb && g_subs[i].user == user)
      return HAL_ERROR;
  }
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++)
  {
    if (g_subs[i].cb == NULL)
    {
      g_subs[i].cb = cb;
      g_subs[i].user = user;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef can_bus_unsubscribe_rx(can_bus_rx_cb_t cb, void *user)
{
  if (!cb)
    return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++)
  {
    if (g_subs[i].cb == cb && g_subs[i].user == user)
    {
      g_subs[i].cb = NULL;
      g_subs[i].user = NULL;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

void can_bus_set_own_id(uint32_t std_id)
{
  g_own_std_id = std_id & 0x7FFu;
}

// Send a single CAN/CAN-FD payload up to 64 bytes.
// If len is not an exact FD size, it rounds up and zero-pads.
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *bytes, size_t len, uint32_t std_id)
{
  if (!g_hfdcan)
    return HAL_ERROR;
  if (!bytes || len == 0)
    return HAL_ERROR;

  if (len > 64)
    len = 64;

  size_t wire_len = can_bus_round_up_fd_len(len);
  uint32_t dlc = can_bus_len_to_dlc(wire_len);
  if (dlc == 0xFFFFFFFFu)
    return HAL_ERROR;

  FDCAN_TxHeaderTypeDef txHeader;
  memset(&txHeader, 0, sizeof(txHeader));
  txHeader.Identifier = std_id & 0x7FFu;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = dlc; // DLC code (HAL expects this)
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;

  // Keep as FD to support >8B payloads; if your bus is classic-only,
  // send only <=8B (or change this to CLASSIC for <=8B).
  txHeader.FDFormat = (wire_len <= 8) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;

#if CAN_BUS_DEBUG
  txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
#else
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
#endif
  txHeader.MessageMarker = 0;

  uint8_t txData[64] = {0};
  memcpy(txData, bytes, len);

  HAL_StatusTypeDef st = can_bus_enqueue_tx_frame(&txHeader, txData);
  if (st == HAL_OK)
  {
    CAN_BUS_DBG("CAN TX queued: id=0x%03lx len=%u\r\n",
                (unsigned long)(txHeader.Identifier & 0x7FFu), (unsigned)len);
  }
  else
  {
    CAN_BUS_DBG("CAN TX queue FAILED: id=0x%03lx len=%u st=%d\r\n",
                (unsigned long)(txHeader.Identifier & 0x7FFu), (unsigned)len, (int)st);
  }
  return st;
}

// Send an arbitrarily large buffer by fragmenting into multiple CAN FD frames.
// This uses fixed 64B frames (DLC=64) and a small header in each frame.
HAL_StatusTypeDef can_bus_send_large(const uint8_t *bytes, size_t len, uint32_t std_id)
{
  if (!g_hfdcan)
    return HAL_ERROR;
  if (!bytes || len == 0)
    return HAL_ERROR;
  if (len > 0xFFFFu)
    return HAL_ERROR; // header uses u16 total_len

  static uint8_t g_seq = 0;
  uint8_t seq = g_seq++;

  const size_t hdr_sz = sizeof(can_bus_frag_hdr_t);
  const size_t wire_len = CAN_BUS_FRAG_WIRE_LEN;
  if (wire_len > 64)
    return HAL_ERROR;
  const size_t data_cap = wire_len - hdr_sz;
  if (data_cap == 0)
    return HAL_ERROR;

  // frag_cnt must fit in u8 with current header design
  size_t frag_cnt_sz = (len + data_cap - 1) / data_cap;
  if (frag_cnt_sz == 0)
    frag_cnt_sz = 1;
  if (frag_cnt_sz > 255)
    return HAL_ERROR;

  uint8_t frag_cnt = (uint8_t)frag_cnt_sz;

  size_t off = 0;
  for (uint8_t idx = 0; idx < frag_cnt; idx++)
  {
    uint8_t frame[64] = {0};

    can_bus_frag_hdr_t hdr;
    hdr.magic = CAN_BUS_FRAG_MAGIC;
    hdr.source = CAN_BUS_NODE_ID;
    hdr.seq = seq;
    hdr.frag_idx = idx;
    hdr.frag_cnt = frag_cnt;
    hdr.flags = 0;
    if (idx == 0)
      hdr.flags |= CAN_BUS_FRAG_F_FIRST;
    if (idx == (uint8_t)(frag_cnt - 1))
      hdr.flags |= CAN_BUS_FRAG_F_LAST;
    hdr.total_len = (uint16_t)len;

    memcpy(frame, &hdr, hdr_sz);

    size_t take = len - off;
    if (take > data_cap)
      take = data_cap;
    memcpy(frame + hdr_sz, bytes + off, take);
    off += take;

    // send a fixed 64-byte payload frame (pads zeros)
    HAL_StatusTypeDef st = can_bus_send_bytes(frame, wire_len, std_id);
    if (st != HAL_OK)
      return st;
  }

  return HAL_OK;
}

// Call this periodically from thread/main-loop context.
// It drains the ISR ring buffer, expires old partial reassembly slots,
// reassembles fragmented messages, and notifies subscribers.
void can_bus_process_rx(void)
{
  uint32_t now = HAL_GetTick();
  reasm_expire_old(now);

    /*
   * Fast-path early exit:
   * - If there is no pending RX/TX event indicated by ISR,
   * - AND our software rings are empty,
   * - AND (in polling builds) the hardware FIFOs are empty,
   * then return quickly.
   */
  if (!g_rx_fifo0_pending && !g_rx_fifo1_pending &&
      (g_rx_head == g_rx_tail) &&
      (g_tx_head == g_tx_tail) &&
      !g_txevt_pending)
  {
#if CAN_BUS_POLLING
    if (!g_hfdcan)
      return;

    const uint32_t f0 = HAL_FDCAN_GetRxFifoFillLevel(g_hfdcan, FDCAN_RX_FIFO0);
    const uint32_t f1 = HAL_FDCAN_GetRxFifoFillLevel(g_hfdcan, FDCAN_RX_FIFO1);
    if (f0 == 0U && f1 == 0U)
      return;
#else
    return;
#endif
  }

  /* Thread-context HW drain (pull FIFO0/FIFO1 into the ring). */
  can_bus_drain_hw_to_ring_thread();

  // Drain any TX event notifications queued by ISR (debug only)
  can_bus_tx_event_t txe;
  while (tx_rb_pop(&txe))
  {
    CAN_BUS_DBG("CAN TX DONE: id=0x%03lx evt=0x%08lx ts=%lu len=%u\r\n",
                (unsigned long)txe.id, (unsigned long)txe.event_type,
                (unsigned long)txe.timestamp, (unsigned)txe.data_len);
  }

  can_bus_rx_frame_t f;
  uint32_t processed = 0U;
  while (processed < CAN_BUS_RX_SERVICE_BUDGET && rb_pop(&f))
  {
    processed++;
    CAN_BUS_DBG("CAN RX RAW: id=0x%03lx len=%u", (unsigned long)f.std_id, (unsigned)f.len);
    can_bus_dbg_dump_bytes(f.data, f.len);
    handle_rx_frame(&f, now);
  }
}

// =========================
// HAL ISR callbacks
// =========================
//
// IMPORTANT: ensure only one definition exists in the entire link.
//
// These callbacks only flag work. Draining an unbounded FIFO in interrupt
// context can starve ThreadX and prevent the RF radio/SEDSNet service loop from
// running under sustained avionics traffic.
// Reassembly and subscriber callbacks happen in can_bus_process_rx().

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0 ||
      hfdcan == NULL ||
      hfdcan != g_hfdcan)
    return;

  g_fdcan_irq_count++;
  g_rx_fifo0_pending = 1U;
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0 ||
      hfdcan == NULL ||
      hfdcan != g_hfdcan)
    return;

  g_fdcan_irq_count++;
  g_rx_fifo1_pending = 1U;
}

// TX event FIFO callback (called in ISR context). We read all new events
// and queue lightweight summaries to the thread via `g_tx_ring`.
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
  (void)hfdcan;
  (void)TxEventFifoITs;
#if CAN_BUS_DEBUG
  g_txevt_pending = 1;
#endif
}
