"""CAN link format v2 (Core/Inc/can_link.h): framing, padding, reassembly,
resynchronisation after lost or damaged frames, and arbitration order."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include "can_link.h"
#include "board_can_priority.h"
#include <assert.h>
#include <stdio.h>

static uint8_t frames[8][64]; static size_t lens[8]; static uint32_t ids[8];

/* Frame a packet; returns the frame count. */
static unsigned send(const uint8_t *p, size_t len, uint32_t base) {
  unsigned n = 0; size_t off = 0;
  while (off < len) {
    size_t take;
    lens[n] = can_link_frame(p, len, off, base, frames[n], &ids[n], &take);
    assert(take > 0 && take <= 64 && lens[n] >= take && lens[n] <= 64);
    assert(can_link_dlc_len(lens[n]) == lens[n]);
    off += take; ++n;
  }
  return n;
}

static int feed(can_link_slot_t *s, unsigned count, unsigned f, uint32_t now,
                const uint8_t **out, size_t *n) {
  return can_link_rx(s, count, ids[f], frames[f], lens[f], now, out, n);
}

int main(void) {
  uint8_t pkt[CAN_LINK_MAX_PACKET + 1];
  can_link_slot_t slots[3] = {0};
  const uint8_t *out; size_t n;

  /* Every length round-trips with no payload header and exact padding. */
  for (size_t len = 1; len <= CAN_LINK_MAX_PACKET; ++len) {
    for (size_t i = 0; i < len; ++i) pkt[i] = (uint8_t)(i * 13 + len);
    const unsigned frames_used = send(pkt, len, 0x106);
    assert(frames_used == (len + 63) / 64);
    for (unsigned f = 0; f < frames_used; ++f) {
      assert((ids[f] & ~CAN_LINK_KIND_MASK) == 0x130);  /* data band, board 6 */
      assert(can_link_board(ids[f]) == 6);
      if (f + 1 < frames_used) assert(lens[f] == 64);
    }
    size_t tail = len - (frames_used - 1) * 64;
    /* Old format: 9 header bytes per frame. Never more bytes now. */
    assert(lens[frames_used - 1] == can_link_dlc_len(tail));
    for (unsigned f = 0; f < frames_used; ++f) {
      int r = feed(slots, 3, f, 0, &out, &n);
      if (f + 1 < frames_used) { assert(r == CAN_LINK_RX_PARTIAL); continue; }
      assert(r == CAN_LINK_RX_COMPLETE && n == len && !memcmp(out, pkt, len));
    }
  }
  /* Kinds: one frame is SINGLE; two are FIRST + LAST; pad marked when used. */
  send(pkt, 8, 0x006); assert((ids[0] & 7) == CAN_LINK_SINGLE && lens[0] == 8);
  send(pkt, 9, 0x006); assert((ids[0] & 7) == (CAN_LINK_SINGLE | CAN_LINK_PAD) && lens[0] == 12 && frames[0][11] == 3);
  send(pkt, 100, 0x006);
  assert((ids[0] & 7) == CAN_LINK_FIRST && (ids[1] & 7) == (CAN_LINK_LAST | CAN_LINK_PAD));
  assert(lens[1] == 48 && frames[1][47] == 12);

  /* Damaged padding or reserved kinds are refused, never misdelivered. */
  send(pkt, 9, 0x006);
  frames[0][11] = 0; assert(feed(slots, 3, 0, 0, &out, &n) == CAN_LINK_RX_MALFORMED);
  frames[0][11] = 12; assert(feed(slots, 3, 0, 0, &out, &n) == CAN_LINK_RX_MALFORMED);
  frames[0][11] = 8; assert(feed(slots, 3, 0, 0, &out, &n) == CAN_LINK_RX_MALFORMED); /* 4 B would be exact */
  frames[0][11] = 3; ids[0] = (ids[0] & ~7U) | 6U; assert(feed(slots, 3, 0, 0, &out, &n) == CAN_LINK_RX_MALFORMED);
  ids[0] = (ids[0] & ~7U) | 7U; assert(feed(slots, 3, 0, 0, &out, &n) == CAN_LINK_RX_MALFORMED);
  send(pkt, 100, 0x006); lens[0] = 48; /* a FIRST frame is always 64 bytes */
  assert(feed(slots, 3, 0, 0, &out, &n) == CAN_LINK_RX_MALFORMED);

  /* Resync: a LAST or MIDDLE with no FIRST (lost FIRST, or joined mid-packet)
   * is an orphan; the next packet is delivered normally. */
  memset(slots, 0, sizeof slots);
  for (size_t i = 0; i < 128; ++i) pkt[i] = (uint8_t)i;
  send(pkt, 128, 0x106);
  assert(feed(slots, 3, 1, 0, &out, &n) == CAN_LINK_RX_ORPHAN);
  uint32_t mid = (ids[1] & ~7U) | CAN_LINK_MIDDLE;
  assert(can_link_rx(slots, 3, mid, frames[0], 64, 0, &out, &n) == CAN_LINK_RX_ORPHAN);
  send(pkt, 128, 0x106);
  assert(feed(slots, 3, 0, 1, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(feed(slots, 3, 1, 1, &out, &n) == CAN_LINK_RX_COMPLETE && n == 128);
  /* Lost LAST: the next FIRST restarts the stream and that packet completes. */
  send(pkt, 128, 0x106);
  assert(feed(slots, 3, 0, 2, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(feed(slots, 3, 0, 3, &out, &n) == CAN_LINK_RX_RESTART);
  assert(feed(slots, 3, 1, 3, &out, &n) == CAN_LINK_RX_COMPLETE && n == 128 && out[127] == 127);
  /* A lost MIDDLE of a longer packet is caught by the size bound or by the
   * packet CRC above this layer; it never overruns the slot. */
  assert(feed(slots, 3, 0, 4, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(slots, 3, mid, frames[0], 64, 4, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(slots, 3, mid, frames[0], 64, 4, &out, &n) == CAN_LINK_RX_OVERSIZE);
  assert(feed(slots, 3, 1, 4, &out, &n) == CAN_LINK_RX_ORPHAN);
  /* Unfinished packets expire. */
  assert(feed(slots, 3, 0, 10, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_expire(slots, 3, 1010, 1000) == 0);
  assert(can_link_expire(slots, 3, 1011, 1000) == 1);
  assert(feed(slots, 3, 1, 1011, &out, &n) == CAN_LINK_RX_ORPHAN);

  /* Streams of different boards and bands interleave independently; with
   * every slot busy the one idle longest is reused. */
  memset(slots, 0, sizeof slots);
  uint8_t a[128], b[128], c[128];
  for (int i = 0; i < 128; ++i) { a[i] = (uint8_t)i; b[i] = (uint8_t)~i; c[i] = (uint8_t)(i ^ 0x5A); }
  uint8_t fa[2][64], fb[2][64], fc[2][64]; uint32_t ia[2], ib[2], ic[2]; size_t t, la[2], lb[2], lc[2];
  for (int f = 0; f < 2; ++f) {
    la[f] = can_link_frame(a, 128, (size_t)f * 64, 0x107, fa[f], &ia[f], &t);
    lb[f] = can_link_frame(b, 128, (size_t)f * 64, 0x087, fb[f], &ib[f], &t);
    lc[f] = can_link_frame(c, 128, (size_t)f * 64, 0x105, fc[f], &ic[f], &t);
  }
  assert(can_link_rx(slots, 3, ia[0], fa[0], la[0], 1, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(slots, 3, ib[0], fb[0], lb[0], 2, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(slots, 3, ic[0], fc[0], lc[0], 3, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(slots, 3, ib[1], fb[1], lb[1], 4, &out, &n) == CAN_LINK_RX_COMPLETE && !memcmp(out, b, 128));
  assert(can_link_rx(slots, 3, ic[1], fc[1], lc[1], 5, &out, &n) == CAN_LINK_RX_COMPLETE && !memcmp(out, c, 128));
  assert(can_link_rx(slots, 3, ia[1], fa[1], la[1], 6, &out, &n) == CAN_LINK_RX_COMPLETE && !memcmp(out, a, 128));
  can_link_slot_t two[2] = {0};
  assert(can_link_rx(two, 2, ia[0], fa[0], la[0], 1, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(two, 2, ib[0], fb[0], lb[0], 2, &out, &n) == CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(two, 2, ic[0], fc[0], lc[0], 9, &out, &n) == CAN_LINK_RX_RESTART);
  assert(can_link_rx(two, 2, ia[1], fa[1], la[1], 9, &out, &n) == CAN_LINK_RX_ORPHAN);
  assert(can_link_rx(two, 2, ic[1], fc[1], lc[1], 9, &out, &n) == CAN_LINK_RX_COMPLETE && !memcmp(out, c, 128));

  /* Arbitration: every control frame of every board (abort, commands, ACKs,
   * heartbeat) beats every metadata frame, which beats every data frame;
   * within a band, board order is the version 1 order (lower board first). */
  uint32_t max_control = 0, min_meta = 0x7FF, max_meta = 0, min_data = 0x7FF;
  for (uint32_t board = 1; board <= 7; ++board) {
    for (unsigned k = 0; k <= CAN_LINK_LAST + CAN_LINK_PAD; ++k) {
      const uint32_t c0 = can_link_id(board, k), m0 = can_link_id(0x080 | board, k),
                     d0 = can_link_id(0x100 | board, k);
      assert(c0 < 0x80 && m0 >= 0x80 && m0 < 0x100 && d0 >= 0x100 && d0 < 0x180);
      if (c0 > max_control) max_control = c0;
      if (m0 < min_meta) min_meta = m0;
      if (m0 > max_meta) max_meta = m0;
      if (d0 < min_data) min_data = d0;
      for (uint32_t other = board + 1; other <= 7; ++other)
        for (unsigned j = 0; j <= CAN_LINK_LAST + CAN_LINK_PAD; ++j)
          assert(c0 < can_link_id(other, j) && d0 < can_link_id(0x100 | other, j));
    }
  }
  assert(max_control < min_meta && max_meta < min_data);
  /* The SDK band choice is unchanged: a high-priority command uses the
   * control band whatever its frame kind. */
  const uint8_t canonical[4] = {0x02, 0x01, 0x05, 0x00};
  assert(board_can_arbitration_id(canonical, 4, 250, 5, 1, 0x006) == 0x006);
  assert(board_can_arbitration_id(canonical, 4, 0, 5, 1, 0x006) == 0x106);
  puts("can_link ok");
  return 0;
}
'''


class CanLinkTests(unittest.TestCase):
    def test_framing_padding_reassembly_resync_and_priority(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p / 'test.c').write_text(HARNESS)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(ROOT / 'Core/Inc'),
                            str(p / 'test.c'), '-o', str(p / 'test')], check=True)
            out = subprocess.run([str(p / 'test')], check=True, capture_output=True, text=True)
            self.assertIn('can_link ok', out.stdout)

    def test_every_board_copy_is_identical_in_this_repo_layout(self):
        # The header is shared verbatim across boards; keep it self-contained.
        text = (ROOT / 'Core/Inc/can_link.h').read_text()
        for needle in ('#include "', 'HAL_', 'stm32'):
            self.assertNotIn(needle, text)


if __name__ == '__main__':
    unittest.main()
