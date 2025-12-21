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
// Notes / Assumptions:
//  - Uses CAN FD frames for fragmentation (default payload 64 bytes).
//  - Fragment frames are distinguished by a small "magic" header in the
//  payload.
//  - Reassembly is bounded (no malloc). Oldest RX frames are dropped on ring
//  overflow.
//  - One producer (ISR) and one consumer (thread calling can_bus_process_rx()).
//  - You can call can_bus_process_rx() from a ThreadX thread, or main
//  superloop.
//
// IMPORTANT CONCURRENCY NOTE:
//  `volatile` head/tail alone does NOT guarantee publish/consume ordering for
//  the ring slot contents by the C language rules. We add `__DMB()` barriers to
//  ensure the slot is fully written before publishing `head` (release), and
//  ensure the consumer sees the slot contents after observing `head` (acquire).

#include "can_bus.h"
#include <stdint.h>
#include <string.h>

// CMSIS barrier intrinsics (for __DMB()).
// On STM32 this is typically available via core_cm*.h brought in by
// stm32xx_hal.h, but including CMSIS directly is safest if your build allows
// it.
#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"
#endif

#ifndef CAN_BUS_MAX_SUBSCRIBERS
#define CAN_BUS_MAX_SUBSCRIBERS 8
#endif

// =========================
// FD DLC helpers
// =========================

static size_t can_bus_dlc_to_len(uint32_t dlc) {
  static const uint8_t map[16] = {0, 1,  2,  3,  4,  5,  6,  7,
                                  8, 12, 16, 20, 24, 32, 48, 64};
  dlc &= 0xF;
  return map[dlc];
}

static uint32_t can_bus_len_to_dlc(size_t len) {
  switch (len) {
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

static size_t can_bus_round_up_fd_len(size_t len) {
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

typedef struct {
  can_bus_rx_cb_t cb;
  void *user;
} can_bus_sub_t;

static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static can_bus_sub_t g_subs[CAN_BUS_MAX_SUBSCRIBERS];

static inline void can_bus_notify_rx(const uint8_t *data, size_t len) {
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    can_bus_rx_cb_t cb = g_subs[i].cb;
    if (cb)
      cb(data, len, g_subs[i].user);
  }
}

// =========================
// Fragmentation protocol
// =========================
//
// We mark fragment frames by a magic header at the start of payload.
// You can use a dedicated CAN ID range too, but magic is simplest.

#define CAN_BUS_FRAG_MAGIC 0x5344u // 'S''D' (arbitrary)
#define CAN_BUS_FRAG_WIRE_LEN 64   // always send 64B payload frames for frags
#define CAN_BUS_REASM_TIMEOUT_MS 250u // drop partial message after this many ms

typedef struct __attribute__((packed)) {
  uint16_t magic;     // CAN_BUS_FRAG_MAGIC
  uint8_t seq;        // message sequence (wrap OK)
  uint8_t frag_idx;   // 0..frag_cnt-1
  uint8_t frag_cnt;   // total fragments
  uint8_t flags;      // bit0=first, bit1=last (optional)
  uint16_t total_len; // total bytes of reassembled message
} can_bus_frag_hdr_t;

enum { CAN_BUS_FRAG_F_FIRST = 1u << 0, CAN_BUS_FRAG_F_LAST = 1u << 1 };

// =========================
// RX ring buffer (ISR -> thread)
// =========================
//
// ISR drains FDCAN FIFO into this ring. Consumer calls can_bus_process_rx()
// from a thread/main loop.

#ifndef CAN_BUS_RX_RING_DEPTH
#define CAN_BUS_RX_RING_DEPTH 64
#endif

typedef struct {
  uint32_t std_id; // 11-bit ID in lower bits (we only handle standard here)
  uint8_t len;     // payload bytes (0..64)
  uint8_t data[64];
} can_bus_rx_frame_t;

static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static can_bus_rx_frame_t g_rx_ring[CAN_BUS_RX_RING_DEPTH];

static inline uint16_t rb_next(uint16_t v) {
  v++;
  if (v >= CAN_BUS_RX_RING_DEPTH)
    v = 0;
  return v;
}

static inline int __attribute__((unused)) rb_is_empty(void) {
  return g_rx_head == g_rx_tail;
}

static inline int rb_is_full(void) { return rb_next(g_rx_head) == g_rx_tail; }

// Push frame from ISR. Drop-oldest on overflow (hybrid “stay current”
// behavior).
//
// Memory ordering:
//  - We must ensure slot writes are visible before publishing head.
//  - `__DMB()` acts as a release barrier here.
static inline void rb_push_drop_oldest(uint32_t std_id, const uint8_t *data,
                                       uint8_t len) {
  if (len > 64)
    len = 64;

  if (rb_is_full()) {
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
//
// Memory ordering:
//  - After observing head != tail, we must ensure subsequent reads of the slot
//    see the writes that happened-before the producer published head.
//  - `__DMB()` acts as an acquire barrier here.
static inline int rb_pop(can_bus_rx_frame_t *out) {
  uint16_t t = g_rx_tail;
  uint16_t h = g_rx_head;

  if (h == t)
    return 0;

  __DMB(); // ensure slot contents are visible after seeing head advance
           // (acquire)

  *out = g_rx_ring[t];

  __DMB(); // ensure slot read completes before we advance tail (conservative)
  g_rx_tail = rb_next(t);
  return 1;
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

typedef struct {
  uint8_t active;
  uint32_t std_id; // which CAN ID this slot is for
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

static void reasm_reset(can_bus_reasm_slot_t *s) {
  s->active = 0;
  s->std_id = 0;
  s->seq = 0;
  s->frag_cnt = 0;
  s->total_len = 0;
  s->data_cap = 0;
  s->last_tick_ms = 0;
  s->got_count = 0;
  memset(s->got_mask, 0, sizeof(s->got_mask));
}

static can_bus_reasm_slot_t *reasm_get_slot(uint32_t std_id, uint8_t seq,
                                            uint32_t now_ms) {
  // First try to find existing active slot for std_id
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++) {
    if (g_reasm[i].active && g_reasm[i].std_id == std_id) {
      // If sequence changed, drop partial and reuse slot
      if (g_reasm[i].seq != seq) {
        reasm_reset(&g_reasm[i]);
      }
      g_reasm[i].last_tick_ms = now_ms;
      return &g_reasm[i];
    }
  }

  // Find a free slot
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++) {
    if (!g_reasm[i].active) {
      reasm_reset(&g_reasm[i]);
      g_reasm[i].active = 1;
      g_reasm[i].std_id = std_id;
      g_reasm[i].seq = seq;
      g_reasm[i].last_tick_ms = now_ms;
      return &g_reasm[i];
    }
  }

  // No free slot: drop the stalest slot (oldest last_tick_ms)
  unsigned stalest = 0;
  uint32_t best_age = 0;
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++) {
    uint32_t age = (uint32_t)(now_ms - g_reasm[i].last_tick_ms);
    if (age >= best_age) {
      best_age = age;
      stalest = i;
    }
  }
  reasm_reset(&g_reasm[stalest]);
  g_reasm[stalest].active = 1;
  g_reasm[stalest].std_id = std_id;
  g_reasm[stalest].seq = seq;
  g_reasm[stalest].last_tick_ms = now_ms;
  return &g_reasm[stalest];
}

static inline int bit_test(uint64_t *mask, uint16_t idx) {
  uint16_t w = (uint16_t)(idx / 64);
  uint16_t b = (uint16_t)(idx % 64);
  return (mask[w] >> b) & 1u;
}

static inline void bit_set(uint64_t *mask, uint16_t idx) {
  uint16_t w = (uint16_t)(idx / 64);
  uint16_t b = (uint16_t)(idx % 64);
  mask[w] |= (1ull << b);
}

static void reasm_expire_old(uint32_t now_ms) {
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++) {
    if (!g_reasm[i].active)
      continue;
    if ((uint32_t)(now_ms - g_reasm[i].last_tick_ms) >
        CAN_BUS_REASM_TIMEOUT_MS) {
      reasm_reset(&g_reasm[i]);
    }
  }
}

// Handle one RX frame (thread context)
static void handle_rx_frame(const can_bus_rx_frame_t *f, uint32_t now_ms) {
  // Check if this is a fragment frame
  if (f->len >= sizeof(can_bus_frag_hdr_t)) {
    can_bus_frag_hdr_t hdr;
    memcpy(&hdr, f->data, sizeof(hdr));

    if (hdr.magic == CAN_BUS_FRAG_MAGIC) {
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

      // We expect fixed 64B wire frames for frags by default.
      // But tolerate smaller frames as long as consistent.
      can_bus_reasm_slot_t *s = reasm_get_slot(f->std_id, hdr.seq, now_ms);

      // If slot was newly created (or reset), initialize message params
      if (s->frag_cnt == 0) {
        s->frag_cnt = hdr.frag_cnt;
        s->total_len = hdr.total_len;
        s->data_cap =
            payload_len; // data bytes available in each fragment frame
        s->got_count = 0;
        memset(s->got_mask, 0, sizeof(s->got_mask));
      } else {
        // Must match the in-flight message properties
        if (s->frag_cnt != hdr.frag_cnt) {
          reasm_reset(s);
          return;
        }
        if (s->total_len != hdr.total_len) {
          reasm_reset(s);
          return;
        }
        // If payload_len changes, we tolerate it (often last frame is shorter),
        // but offset math uses s->data_cap established at first fragment.
      }

      // Compute where this fragment’s payload should land
      uint32_t off = (uint32_t)hdr.frag_idx * (uint32_t)s->data_cap;
      if (off >= s->total_len)
        return;

      uint32_t take = payload_len;
      if (off + take > s->total_len)
        take = (uint32_t)s->total_len - off;

      // Mark + copy if not already received
      if (!bit_test(s->got_mask, hdr.frag_idx)) {
        bit_set(s->got_mask, hdr.frag_idx);
        s->got_count++;
        memcpy(&s->buf[off], payload, take);
      }

      s->last_tick_ms = now_ms;

      // Complete?
      if (s->got_count == s->frag_cnt) {
        can_bus_notify_rx(s->buf, s->total_len);
        reasm_reset(s);
      }

      return;
    }
  }

  // Not a fragment frame: deliver raw CAN payload
  can_bus_notify_rx(f->data, f->len);
}

// =========================
// Public API
// =========================

void can_bus_init(FDCAN_HandleTypeDef *hfdcan) {
  g_hfdcan = hfdcan;
  // subscribers static-zeroed
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(hfdcan);

  // reset rings + reasm
  g_rx_head = 0;
  g_rx_tail = 0;
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++) {
    reasm_reset(&g_reasm[i]);
  }
}

HAL_StatusTypeDef can_bus_subscribe_rx(can_bus_rx_cb_t cb, void *user) {
  if (!cb)
    return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user)
      return HAL_ERROR;
  }
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == NULL) {
      g_subs[i].cb = cb;
      g_subs[i].user = user;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef can_bus_unsubscribe_rx(can_bus_rx_cb_t cb, void *user) {
  if (!cb)
    return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      g_subs[i].cb = NULL;
      g_subs[i].user = NULL;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

// Send a single CAN/CAN-FD payload up to 64 bytes.
// If len is not an exact FD size, it rounds up and zero-pads.
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *bytes, size_t len,
                                     uint32_t std_id) {
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
  txHeader.FDFormat = FDCAN_FD_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  uint8_t txData[64] = {0};
  memcpy(txData, bytes, len);

  return HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &txHeader, txData);
}

// Send an arbitrarily large buffer by fragmenting into multiple CAN FD frames.
// This uses fixed 64B frames (DLC=64) and a small header in each frame.
HAL_StatusTypeDef can_bus_send_large(const uint8_t *bytes, size_t len,
                                     uint32_t std_id) {
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
  for (uint8_t idx = 0; idx < frag_cnt; idx++) {
    uint8_t frame[64] = {0};

    can_bus_frag_hdr_t hdr;
    hdr.magic = CAN_BUS_FRAG_MAGIC;
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
void can_bus_process_rx(void) {
  uint32_t now = HAL_GetTick();
  reasm_expire_old(now);

  can_bus_rx_frame_t f;
  while (rb_pop(&f)) {
    handle_rx_frame(&f, now);
  }
}

// =========================
// HAL ISR callback
// =========================
//
// IMPORTANT: ensure only one definition exists in the entire link.
//
// This ISR does minimal work: drains RX FIFO1 into our ring buffer.
// Reassembly and subscriber callbacks happen in can_bus_process_rx().

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0)
    return;

  FDCAN_RxHeaderTypeDef hdr;
  uint8_t data[64];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &hdr, data) != HAL_OK) {
      break;
    }

    // Only handle standard IDs here; extend as needed
    uint32_t std_id = hdr.Identifier & 0x7FFu;

    // hdr.DataLength is DLC code in HAL
    size_t len = can_bus_dlc_to_len(hdr.DataLength);
    if (len > 64)
      len = 64;

    // Push into ring; drop-oldest on overflow
    rb_push_drop_oldest(std_id, data, (uint8_t)len);
  }
}
