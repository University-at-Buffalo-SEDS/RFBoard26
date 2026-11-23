#ifndef CAN_LINK_H
#define CAN_LINK_H
/* CAN link format, version 2 (identical in every board; the whole stack
 * updates together, so no older frame layout is accepted).
 *
 * 11-bit identifier:
 *   bits 8..7  band: 0x000 controls/ACKs/heartbeat, 0x080 protected
 *              metadata, 0x100 data (from board_can_arbitration_id)
 *   bits 6..3  board (1..15; the board's control id)
 *   bits 2..0  link kind (below)
 * Band, then board, decide arbitration exactly as the version 1 identifiers
 * (band | board) did. The kind bits order only one board's own frames within
 * one band, and every board sends from its TX FIFO in submission order.
 *
 * The payload carries no link header. A packet of up to 64 bytes is one
 * SINGLE frame. A longer packet (at most CAN_LINK_MAX_PACKET) is FIRST,
 * MIDDLE..., LAST; every frame but LAST carries exactly 64 bytes. When the
 * CAN FD length code rounds a frame above its data, the PAD bit is set and
 * the frame's last byte holds the pad length (1..15), so the receiver
 * recovers the exact packet length from bytes that were padding anyway.
 *
 * Integrity: the CAN CRC covers each frame and the SEDSnet CRC32 covers each
 * packet. One board's fragments of one band are consecutive on the bus (its
 * queue never interleaves two packets of one band), so FIRST/MIDDLE/LAST is
 * enough to reassemble; a lost fragment leaves an orphan that is dropped, or
 * a short packet that fails its CRC32. */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CAN_LINK_FRAME_BYTES 64U
#define CAN_LINK_MAX_PACKET 128U
#define CAN_LINK_BAND_MASK 0x180U
#define CAN_LINK_BOARD_MASK 0x0FU
#define CAN_LINK_STREAM_MASK 0x1F8U
#define CAN_LINK_KIND_MASK 0x7U

enum {
  CAN_LINK_SINGLE = 0U,
  CAN_LINK_PAD = 1U, /* with SINGLE or LAST */
  CAN_LINK_FIRST = 2U,
  CAN_LINK_MIDDLE = 3U,
  CAN_LINK_LAST = 4U,
};

/* Wire identifier of a frame of kind `kind` for base id `band | board`. */
static inline uint32_t can_link_id(uint32_t base, unsigned kind) {
  return (base & CAN_LINK_BAND_MASK) | ((base & CAN_LINK_BOARD_MASK) << 3) |
         (kind & CAN_LINK_KIND_MASK);
}
/* Board token (control id) of a wire identifier. */
static inline unsigned can_link_board(uint32_t id) {
  return (unsigned)((id >> 3) & CAN_LINK_BOARD_MASK);
}

/* Smallest CAN FD data length that holds n bytes (n <= 64). */
static inline size_t can_link_dlc_len(size_t n) {
  if (n <= 8U) return n;
  if (n <= 24U) return (n + 3U) & ~(size_t)3U;
  if (n <= 32U) return 32U;
  if (n <= 48U) return 48U;
  return 64U;
}

/* Build the frame that carries `packet[offset..]` of a `len`-byte packet.
 * Writes the frame bytes to `out`, its identifier to `*id` and the packet
 * bytes it carries to `*take`; returns the frame's data length. */
static inline size_t can_link_frame(const uint8_t *packet, size_t len,
                                    size_t offset, uint32_t base,
                                    uint8_t out[CAN_LINK_FRAME_BYTES],
                                    uint32_t *id, size_t *take) {
  size_t n = len - offset;
  if (n > CAN_LINK_FRAME_BYTES) n = CAN_LINK_FRAME_BYTES;
  const unsigned first = offset == 0U, last = offset + n == len;
  unsigned kind = first ? (last ? CAN_LINK_SINGLE : CAN_LINK_FIRST)
                        : (last ? CAN_LINK_LAST : CAN_LINK_MIDDLE);
  memcpy(out, packet + offset, n);
  const size_t wire = can_link_dlc_len(n);
  if (wire != n) {
    memset(out + n, 0, wire - n);
    out[wire - 1U] = (uint8_t)(wire - n);
    kind |= CAN_LINK_PAD;
  }
  *id = can_link_id(base, kind);
  *take = n;
  return wire;
}

/* Packet bytes in a received frame, without its padding; 0 if malformed. */
static inline size_t can_link_payload_len(uint32_t id, const uint8_t *data,
                                          size_t len) {
  const unsigned kind = id & CAN_LINK_KIND_MASK;
  if (kind > CAN_LINK_LAST + CAN_LINK_PAD || len == 0U || len > 64U) return 0U;
  if (kind == CAN_LINK_FIRST || kind == CAN_LINK_MIDDLE)
    return len == CAN_LINK_FRAME_BYTES ? len : 0U;
  if (!(kind & CAN_LINK_PAD)) return can_link_dlc_len(len) == len ? len : 0U;
  const size_t pad = data[len - 1U];
  if (pad == 0U || pad >= len || can_link_dlc_len(len - pad) != len) return 0U;
  return len - pad;
}

/* Does this frame start or complete a packet by itself (SINGLE)? */
static inline int can_link_is_single(uint32_t id) {
  return (id & CAN_LINK_KIND_MASK) <= CAN_LINK_PAD;
}

/* Reassembly of FIRST/MIDDLE/LAST frames, one slot per sending stream. */
typedef struct {
  uint32_t stream; /* band | board bits of the wire identifier */
  uint32_t last_ms;
  uint16_t len;
  uint8_t active;
  uint8_t buf[CAN_LINK_MAX_PACKET];
} can_link_slot_t;

enum {
  CAN_LINK_RX_PARTIAL = 0,    /* fragment stored */
  CAN_LINK_RX_COMPLETE = 1,   /* *out / *out_len hold a packet */
  CAN_LINK_RX_MALFORMED = -1, /* bad kind, length or pad: frame dropped */
  CAN_LINK_RX_ORPHAN = -2,    /* MIDDLE/LAST without FIRST: dropped */
  CAN_LINK_RX_RESTART = -3,   /* FIRST replaced an unfinished packet */
  CAN_LINK_RX_OVERSIZE = -4,  /* packet over CAN_LINK_MAX_PACKET: dropped */
};

static inline can_link_slot_t *can_link_find(can_link_slot_t *slots,
                                             unsigned count, uint32_t stream) {
  for (unsigned i = 0; i < count; ++i)
    if (slots[i].active && slots[i].stream == stream) return &slots[i];
  return NULL;
}

/* Feed one received frame. A SINGLE frame is returned in place; a LAST frame
 * returns the slot buffer, valid until the next call. */
static inline int can_link_rx(can_link_slot_t *slots, unsigned count,
                              uint32_t id, const uint8_t *data, size_t len,
                              uint32_t now_ms, const uint8_t **out,
                              size_t *out_len) {
  const size_t n = can_link_payload_len(id, data, len);
  if (n == 0U) return CAN_LINK_RX_MALFORMED;
  const unsigned kind = id & CAN_LINK_KIND_MASK;
  if (kind <= CAN_LINK_PAD) {
    *out = data;
    *out_len = n;
    return CAN_LINK_RX_COMPLETE;
  }
  const uint32_t stream = id & CAN_LINK_STREAM_MASK;
  can_link_slot_t *s = can_link_find(slots, count, stream);
  if (kind == CAN_LINK_FIRST) {
    int result = CAN_LINK_RX_PARTIAL;
    if (s) {
      result = CAN_LINK_RX_RESTART;
    } else {
      /* A free slot, else the one idle longest. */
      uint32_t oldest = 0U;
      for (unsigned i = 0; i < count; ++i) {
        if (!slots[i].active) { s = &slots[i]; break; }
        const uint32_t age = now_ms - slots[i].last_ms;
        if (!s || age >= oldest) { s = &slots[i]; oldest = age; }
      }
      if (s->active) result = CAN_LINK_RX_RESTART;
    }
    memcpy(s->buf, data, n);
    s->stream = stream;
    s->len = (uint16_t)n;
    s->active = 1U;
    s->last_ms = now_ms;
    return result;
  }
  if (!s) return CAN_LINK_RX_ORPHAN;
  if ((size_t)s->len + n > CAN_LINK_MAX_PACKET) {
    s->active = 0U;
    return CAN_LINK_RX_OVERSIZE;
  }
  memcpy(s->buf + s->len, data, n);
  s->len = (uint16_t)(s->len + n);
  s->last_ms = now_ms;
  if (kind == CAN_LINK_MIDDLE) return CAN_LINK_RX_PARTIAL;
  s->active = 0U;
  *out = s->buf;
  *out_len = s->len;
  return CAN_LINK_RX_COMPLETE;
}

/* Drop unfinished packets idle for more than timeout_ms; returns how many. */
static inline unsigned can_link_expire(can_link_slot_t *slots, unsigned count,
                                       uint32_t now_ms, uint32_t timeout_ms) {
  unsigned expired = 0U;
  for (unsigned i = 0; i < count; ++i) {
    if (slots[i].active && now_ms - slots[i].last_ms > timeout_ms) {
      slots[i].active = 0U;
      ++expired;
    }
  }
  return expired;
}
#endif
