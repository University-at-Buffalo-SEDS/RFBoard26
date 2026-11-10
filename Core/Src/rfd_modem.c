/* RFD900x ASYNC modem check. Mirrors GroundStation26 rfd_modem.rs:
 * RFDesign "RFD900x Asynchronous User Manual" V1.1: "+++" after at least one
 * second of host silence, ATSn=X then AT&W then ATZ, ATI5 lists all EEPROM
 * parameters, and S2-S5, S7-S9, S14-S17, S19 must match on every node.
 * No dynamic memory; answers are parsed one line at a time. */
#include "rfd_modem.h"

#include <string.h>

const rfd_timing_t RFD_TIMING = {
    .guard = 1100U, .plus_reply = 2500U, .command = 1000U, .idle = 150U,
    .save = 2000U, .reboot = 4000U, .overall = 16000U,
};

#define BROADCAST_DESTID 32768
#define LINE_MAX 96U

/* Plan order is fixed; the status block reads these indices. Registers
 * without an expectation (NETID, TXPOWER, frequencies, ...) are not read:
 * fewer commands keep the link down for less time. */
enum { P_S1, P_S2, P_S3, P_S4, P_S5, P_S6, P_S7, P_S8, P_S11, P_S13, P_LBT, P_RTSCTS, P_COUNT };
_Static_assert(P_COUNT == RFD_PLAN_MAX, "plan size");

void rfd_plan_init(rfd_plan_t *plan, const rfd_profile_values_t *p)
{
  enum { REPORT = 0, LINK = 1, FIX = 2 };
  /* Value sources: profile fields, then fixed targets. */
  const uint32_t source[11] = {
      p->serial_baud / 1000U, p->air_bps / 1000U, p->max_data_bytes, p->max_retries,
      p->broadcast_retries, p->serial_break_ms, p->tx_encapsulation, p->rx_encapsulation,
      BROADCAST_DESTID, 0U, p->hardware_flow_control ? 1U : 0U,
  };
  /* Accepted ATI5 names, '|'-separated; V1.1 names first, then ASYNC 4.03
   * (RFD900x2) names. The number is the V1.1 fallback for unnamed modems.
   * S1 change would cut this host off; S2 and LBT_RSSI are never written by
   * one node. Link-wide registers (Table 3-2) are written only for a
   * rolled-out profile. Local: S6, S11, S13, RTSCTS. */
  static const struct { const char *names; uint8_t number, cls, src; } base[P_COUNT] = {
      {"SERIAL_SPEED", 1, REPORT, 0}, {"AIR_SPEED", 2, REPORT, 1},
      {"MAX_DATA", 3, LINK, 2}, {"MAX_RETRIES", 4, LINK, 3}, {"GLOBAL_RETRIES", 5, LINK, 4},
      {"SER_BREAK_DETECT_MS|SER_BREAK_DETECT|SERIAL_BREAK|SER_BRK_DETMS", 6, FIX, 5},
      {"TXENCAP|TX_ENCAP|TX_ENCAP_METHOD", 7, LINK, 6},
      {"RXENCAP|RX_ENCAP|RX_ENCAP_METHOD", 8, LINK, 7},
      {"DESTID|DEST_ID", 11, FIX, 8}, {"MAVLINK1|MAVLINK", 13, FIX, 9},
      {"LBT_RSSI", 17, REPORT, 9}, {"RTSCTS", 18, FIX, 10},
  };
  for (unsigned i = 0; i < P_COUNT; i++) {
    const uint32_t v = source[base[i].src];
    const uint8_t cls = base[i].cls;
    plan->regs[i].names = base[i].names;
    plan->regs[i].number = base[i].number;
    plan->regs[i].policy = (cls == FIX || (cls == LINK && p->rolled_out_all_nodes))
                               ? RFD_POLICY_FIX : RFD_POLICY_REPORT;
    plan->regs[i].expected = v > 65535U ? RFD_UNREAD : (int32_t)v;
  }
  plan->count = P_COUNT;
}

/* --- text helpers -------------------------------------------------------- */

static char upper(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }
static bool alnum(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

/* True if `name` equals one '|'-separated entry of `names`, comparing
 * letters and digits only, case-insensitively. */
static bool names_match(const char *name, size_t len, const char *names)
{
  for (;;) {
    size_t i = 0;
    bool same = true;
    for (;;) {
      while (i < len && !alnum(name[i])) i++;
      while (*names && *names != '|' && !alnum(*names)) names++;
      const bool name_end = i == len, entry_end = !*names || *names == '|';
      if (name_end || entry_end) {
        same = same && name_end && entry_end;
        break;
      }
      if (upper(name[i]) != upper(*names)) same = false;
      i++;
      names++;
    }
    if (same) return true;
    while (*names && *names != '|') names++;
    if (!*names) return false;
    names++;
  }
}

/* Whole-line unsigned decimal of 1..9 digits (S-register values are never
 * negative; anything else is treated as unread and never written). */
static bool parse_int(const char *s, size_t len, int32_t *out)
{
  while (len && s[0] == ' ') { s++; len--; }
  while (len && s[len - 1] == ' ') len--;
  if (len == 0 || len > 9) return false;
  int32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    if (s[i] < '0' || s[i] > '9') return false;
    v = v * 10 + (s[i] - '0');
  }
  *out = v;
  return true;
}

/* --- snapshot ------------------------------------------------------------ */

typedef struct {
  int32_t value[P_COUNT];
  uint8_t number[P_COUNT];
  uint32_t has_name, name_ok, by_name;
  bool any_named;
} snapshot_t;

static void snap_reset(snapshot_t *s, const rfd_plan_t *plan)
{
  memset(s, 0, sizeof(*s));
  for (unsigned i = 0; i < plan->count; i++) {
    s->value[i] = RFD_UNREAD;
    s->number[i] = plan->regs[i].number;
  }
}

/* One ATI5 line: "S19:RTSCTS=0". */
static void snap_ati5_line(snapshot_t *s, const rfd_plan_t *plan, const char *line, size_t len)
{
  if (len < 5 || line[0] != 'S') return;
  size_t i = 1;
  unsigned number = 0;
  while (i < len && line[i] == ' ') i++;
  size_t digits = 0;
  while (i < len && line[i] >= '0' && line[i] <= '9') {
    number = number * 10U + (unsigned)(line[i] - '0');
    i++;
    if (++digits > 3) return;
  }
  while (i < len && line[i] == ' ') i++;
  if (digits == 0 || number > 255U || i >= len || line[i] != ':') return;
  i++;
  while (i < len && line[i] == ' ') i++;
  const char *name = &line[i];
  size_t name_len = 0;
  while (i < len && line[i] != '=') { i++; name_len++; }
  if (i >= len) return;
  while (name_len && name[name_len - 1] == ' ') name_len--;
  if (name_len == 0 || name_len > 32) return;
  for (size_t k = 0; k < name_len; k++) {
    const char c = name[k];
    if (!alnum(c) && c != '_' && c != '/' && c != '.' && c != '-') return;
  }
  int32_t value;
  if (!parse_int(&line[i + 1], len - i - 1, &value)) return;
  s->any_named = true;
  for (unsigned p = 0; p < plan->count; p++) {
    const uint32_t bit = (uint32_t)1U << p;
    if (names_match(name, name_len, plan->regs[p].names)) {
      s->by_name |= bit;
      s->has_name |= bit;
      s->name_ok |= bit;
      s->value[p] = value;
      s->number[p] = (uint8_t)number;
    } else if (number == plan->regs[p].number && !(s->by_name & bit)) {
      /* Documented number, other name: kept only if the name never appears. */
      s->has_name |= bit;
      s->name_ok &= ~bit;
      s->value[p] = value;
      s->number[p] = (uint8_t)number;
    }
  }
}

static rfd_map_t map_source(bool async_identity, const snapshot_t *s, const rfd_plan_t *plan)
{
  bool all_named = true;
  for (unsigned p = 0; p < plan->count; p++) {
    if (plan->regs[p].policy == RFD_POLICY_FIX && !(s->by_name & ((uint32_t)1U << p))) all_named = false;
  }
  if (all_named) return RFD_MAP_ATI5;
  /* Names exist but differ from every known map: never guess. */
  if (s->any_named) return RFD_MAP_UNCONFIRMED;
  static const uint16_t air[] = {12, 56, 64, 100, 125, 188, 200, 224, 250, 500, 830};
  bool air_ok = false;
  for (unsigned k = 0; k < sizeof(air) / sizeof(air[0]); k++) air_ok |= s->value[P_S2] == air[k];
  const bool fingerprint = async_identity && s->value[P_S1] != RFD_UNREAD &&
                           s->value[P_S1] == plan->regs[P_S1].expected &&
                           s->value[P_S3] >= 1 && s->value[P_S3] <= 4068 && air_ok &&
                           s->value[P_S11] >= 1 && s->value[P_S11] <= 65535;
  return fingerprint ? RFD_MAP_FINGERPRINT : RFD_MAP_UNCONFIRMED;
}

static rfd_status_t reg_status(const rfd_plan_t *plan, const snapshot_t *s, unsigned p)
{
  const uint32_t bit = (uint32_t)1U << p;
  if (plan->regs[p].policy == RFD_POLICY_INFO) return RFD_ST_INFO;
  if (s->value[p] == RFD_UNREAD) return RFD_ST_UNREAD;
  if ((s->has_name & bit) && !(s->name_ok & bit)) return RFD_ST_NAME_MISMATCH;
  return s->value[p] == plan->regs[p].expected ? RFD_ST_MATCH : RFD_ST_DIFFERS;
}

/* --- driver -------------------------------------------------------------- */

enum { ABORT_IO = -1, ABORT_DEADLINE = -2, ABORT_GATE = -3 };
#define TRY(expr) do { const int rc_ = (expr); if (rc_ < 0) return rc_; } while (0)

typedef void (*line_fn)(void *ctx, const char *line, size_t len);

typedef struct {
  const rfd_io_t *io;
  const rfd_timing_t *t;
  rfd_gate_fn gate;
  void *gate_ctx;
  uint32_t start;
  char line[LINE_MAX];
  size_t line_len;
  bool saw_plus_ok;
  /* What a fast exit must undo: "+++" sent, command mode, unsaved writes. */
  bool plus_sent, in_command, ram_dirty;
  /* answer */
  bool ok, error;
  unsigned lines;
} driver_t;

static uint32_t now(driver_t *d) { return d->io->now_ms(d->io->ctx); }

/* Every wait passes here: the overall bound covers reboots and the verify,
 * and a closed flight gate stops the check within one poll. */
static int left(driver_t *d, uint32_t *remaining)
{
  const uint32_t elapsed = now(d) - d->start;
  if (elapsed >= d->t->overall) return ABORT_DEADLINE;
  if (!d->gate(d->gate_ctx)) return ABORT_GATE;
  if (remaining) *remaining = d->t->overall - elapsed;
  return 0;
}

static int dsleep(driver_t *d, uint32_t ms)
{
  while (ms) {
    uint32_t rem;
    TRY(left(d, &rem));
    uint32_t step = ms < 20U ? ms : 20U;
    if (step > rem) step = rem;
    d->io->sleep_ms(d->io->ctx, step);
    ms -= step;
  }
  return 0;
}

static int write_all(driver_t *d, const char *bytes, size_t len)
{
  unsigned stalls = 0;
  while (len) {
    TRY(left(d, NULL));
    const int n = d->io->write(d->io->ctx, (const uint8_t *)bytes, len);
    if (n < 0) return ABORT_IO;
    if (n == 0) {
      if (++stalls > 50U) return ABORT_IO;
      d->io->sleep_ms(d->io->ctx, 10U);
      continue;
    }
    stalls = 0;
    const size_t used = (size_t)n < len ? (size_t)n : len;
    bytes += used;
    len -= used;
  }
  return 0;
}

static int pump(driver_t *d, uint8_t *buf, size_t cap)
{
  const int n = d->io->read(d->io->ctx, buf, cap);
  return n < 0 ? ABORT_IO : n;
}

/* Discards input. Air data may stream, so the window bounds it. */
static int drain(driver_t *d, uint32_t window)
{
  uint8_t buf[32];
  const uint32_t start = now(d);
  uint32_t last = start;
  for (;;) {
    TRY(left(d, NULL));
    int n;
    TRY(n = pump(d, buf, sizeof(buf)));
    const uint32_t t = now(d);
    if (n > 0) last = t;
    if (t - start >= window || t - last >= 100U) break;
  }
  d->line_len = 0;
  return 0;
}

/* Waits for the "+++" answer. Air data may itself contain "OK", so it counts
 * only after a guard time and once the line has gone quiet. */
static int wait_for_plus_ok(driver_t *d)
{
  uint8_t buf[32];
  uint8_t tail[3] = {0, 0, 0};
  const uint32_t start = now(d);
  const uint32_t settle = d->t->guard + 100U;
  uint32_t last = start;
  bool seen = false;
  for (;;) {
    TRY(left(d, NULL));
    int n;
    TRY(n = pump(d, buf, sizeof(buf)));
    for (int i = 0; i < n; i++) {
      tail[0] = tail[1];
      tail[1] = tail[2];
      tail[2] = buf[i];
      if (tail[0] == 'O' && tail[1] == 'K' && (tail[2] == '\r' || tail[2] == '\n')) seen = true;
    }
    const uint32_t t = now(d);
    if (n > 0) last = t;
    if (seen && t - start >= settle && t - last >= 200U) return 1;
    if (t - start >= d->t->plus_reply) return 0;
  }
}

/* Exact match: the modem echoes commands verbatim. */
static bool line_is(const char *line, size_t len, const char *word)
{
  return len == strlen(word) && memcmp(line, word, len) == 0;
}

/* Sends one AT command; each answer line goes to on_line. Ends on OK/ERROR,
 * on a quiet gap after at least one line, or on timeout. */
static int command(driver_t *d, const char *cmd, uint32_t timeout, line_fn on_line, void *ctx)
{
  uint8_t buf[32];
  d->ok = d->error = false;
  d->lines = 0;
  d->line_len = 0;
  TRY(write_all(d, cmd, strlen(cmd)));
  TRY(write_all(d, "\r", 1));
  const uint32_t start = now(d);
  uint32_t last = start;
  for (;;) {
    int n;
    TRY(n = pump(d, buf, sizeof(buf)));
    const uint32_t t = now(d);
    if (n > 0) last = t;
    for (int i = 0; i < n && !d->ok && !d->error; i++) {
      const char c = (char)buf[i];
      if (c != '\r' && c != '\n') {
        if ((unsigned char)c < 0x20U || (unsigned char)c > 0x7EU) continue;
        if (d->line_len < LINE_MAX - 1U) d->line[d->line_len++] = c;
        continue;
      }
      const char *line = d->line;
      size_t len = d->line_len;
      d->line_len = 0;
      while (len && line[0] == ' ') { line++; len--; }
      while (len && line[len - 1] == ' ') len--;
      if (len == 0 || line_is(line, len, cmd)) continue;
      if (len == 2 && line[0] == 'O' && line[1] == 'K') {
        d->ok = true;
      } else if (len == 5 && memcmp(line, "ERROR", 5) == 0) {
        d->error = true;
      } else if (d->lines < 64U) {
        d->lines++;
        if (on_line) on_line(ctx, line, len);
      }
    }
    if (d->ok || d->error) return 0;
    if ((t - last >= d->t->idle && d->lines) || t - start >= timeout) return 0;
    TRY(left(d, NULL));
  }
}

static int enter_command_mode(driver_t *d)
{
  for (int attempt = 0; attempt < 2; attempt++) {
    TRY(dsleep(d, d->t->guard));
    TRY(drain(d, 200U));
    d->plus_sent = true;
    TRY(write_all(d, "+++", 3));
    int got;
    TRY(got = wait_for_plus_ok(d));
    if (got) {
      d->saw_plus_ok = true;
      /* Air data can contain "OK"; in command mode this answer is clean. */
      TRY(command(d, "AT", d->t->command, NULL, NULL));
      if (d->ok) return d->in_command = true;
    }
    if (attempt == 0) {
      /* Already in command mode: "+++" was just text. */
      TRY(write_all(d, "\r", 1));
      TRY(drain(d, 200U));
      TRY(command(d, "AT", d->t->command, NULL, NULL));
      if (d->ok) return d->in_command = true;
    }
  }
  return 0;
}

static int reboot(driver_t *d, rfd_report_t *r)
{
  TRY(write_all(d, "ATZ\r", 4));
  /* ATZ does not answer; the modem restarts in data mode. */
  d->plus_sent = d->in_command = d->ram_dirty = false;
  r->flags |= RFD_FLAG_REBOOTED;
  r->data_mode = RFD_DATA_REBOOTED;
  return dsleep(d, d->t->reboot);
}

/* ATO usually prints nothing; only ERROR means it failed. Fallback: ATZ. */
static void exit_command_mode(driver_t *d, rfd_report_t *r)
{
  for (int i = 0; i < 2; i++) {
    if (command(d, "ATO", 300U, NULL, NULL) < 0) break;
    if (!d->error) {
      d->plus_sent = d->in_command = false;
      r->data_mode = RFD_DATA_RESUMED;
      r->flags |= RFD_FLAG_RESUMED;
      return;
    }
  }
  r->flags |= RFD_FLAG_ATTENTION;
  (void)reboot(d, r);
}

/* Bound or flight gate hit: leave at once, without waiting for an answer.
 * Unsaved writes are discarded by ATZ; otherwise "\r" cancels a pending
 * "+++" inside its guard time and ATO leaves command mode. */
static void fast_exit(driver_t *d, rfd_report_t *r)
{
  if (d->in_command && d->ram_dirty) {
    if (d->io->write(d->io->ctx, (const uint8_t *)"ATZ\r", 4U) == 4) {
      r->flags |= RFD_FLAG_REBOOTED;
      r->data_mode = RFD_DATA_REBOOTED;
    }
  } else if (d->plus_sent) {
    (void)d->io->write(d->io->ctx, (const uint8_t *)"\rATO\r", 5U);
  }
}

typedef struct {
  snapshot_t *snap;
  const rfd_plan_t *plan;
  int32_t first_int;
  bool have_int;
  bool async_identity, identity_seen, forwarding;
} parse_ctx_t;

static void on_ati5(void *ctx, const char *line, size_t len)
{
  parse_ctx_t *c = ctx;
  snap_ati5_line(c->snap, c->plan, line, len);
}

static void on_int(void *ctx, const char *line, size_t len)
{
  parse_ctx_t *c = ctx;
  if (!c->have_int) c->have_int = parse_int(line, len, &c->first_int);
}

static void on_identity(void *ctx, const char *line, size_t len)
{
  parse_ctx_t *c = ctx;
  if (c->identity_seen) return;
  c->identity_seen = true;
  for (size_t i = 0; i + 5 <= len; i++) {
    if (line_is(&line[i], 5, "ASYNC")) c->async_identity = true;
  }
}

/* Forwarding table. ASYNC 4.03 prints "slot:src->dst" entries separated by
 * commas ("0:0->0,1:0->0,...| FwdBuff=13320"): an entry exists when the
 * number after ':' or after "->" is nonzero. The manual's form is
 * "AT+Fx=Src,Dst": any "Src,Dst" pair other than 0,0. */
static void on_forwarding(void *ctx, const char *line, size_t len)
{
  parse_ctx_t *c = ctx;
  bool arrow = false;
  for (size_t k = 0; k + 1 < len; k++) arrow |= line[k] == '-' && line[k + 1] == '>';
  char prev = 0; /* character before the current number */
  bool pair_open = false, pair_nonzero = false;
  for (size_t i = 0; i < len;) {
    if (line[i] < '0' || line[i] > '9') { prev = line[i++]; continue; }
    bool nonzero = false;
    while (i < len && line[i] >= '0' && line[i] <= '9') nonzero |= line[i++] != '0';
    const char next = i < len ? line[i] : 0;
    if (arrow) {
      if ((prev == ':' || prev == '>') && nonzero) c->forwarding = true;
    } else if (pair_open && prev == ',' && next != ',') {
      if (pair_nonzero || nonzero) c->forwarding = true;
      pair_open = false;
    } else {
      pair_open = next == ',' && prev != ',';
      pair_nonzero = nonzero;
    }
  }
}

static int read_snapshot(driver_t *d, const rfd_plan_t *plan, snapshot_t *s)
{
  parse_ctx_t c = {.snap = s, .plan = plan};
  snap_reset(s, plan);
  TRY(command(d, "ATI5", d->t->command * 2U, on_ati5, &c));
  for (unsigned p = 0; p < plan->count; p++) {
    if (s->has_name & ((uint32_t)1U << p)) continue;
    char cmd[8] = "ATS";
    unsigned n = plan->regs[p].number, k = 3;
    if (n >= 10U) cmd[k++] = (char)('0' + n / 10U);
    cmd[k++] = (char)('0' + n % 10U);
    cmd[k++] = '?';
    cmd[k] = 0;
    c.have_int = false;
    TRY(command(d, cmd, d->t->command, on_int, &c));
    if (!d->error && c.have_int) s->value[p] = c.first_int;
  }
  return 0;
}

static void fill_observed(rfd_report_t *r, const snapshot_t *s)
{
  static const uint8_t idx[7] = {P_S2, P_S3, P_S4, P_S5, P_S6, P_S11, P_RTSCTS};
  for (unsigned k = 0; k < 7U; k++) r->observed[k] = s->value[idx[k]];
  r->rtscts_number = (s->by_name & (1UL << P_RTSCTS)) ? s->number[P_RTSCTS] : 0U;
}

static void fmt_write(char *cmd, uint8_t number, uint16_t value)
{
  char digits[6];
  unsigned k = 0, n = 0;
  cmd[k++] = 'A'; cmd[k++] = 'T'; cmd[k++] = 'S';
  if (number >= 100U) cmd[k++] = (char)('0' + number / 100U);
  if (number >= 10U) cmd[k++] = (char)('0' + (number / 10U) % 10U);
  cmd[k++] = (char)('0' + number % 10U);
  cmd[k++] = '=';
  do { digits[n++] = (char)('0' + value % 10U); value /= 10U; } while (value);
  while (n) cmd[k++] = digits[--n];
  cmd[k] = 0;
}

static int run_inner(driver_t *d, const rfd_plan_t *plan, rfd_mode_t mode, rfd_gate_fn gate,
                     void *gate_ctx, bool allow_save, rfd_report_t *r)
{
  snapshot_t snap;
  int entered;
  TRY(entered = enter_command_mode(d));
  if (!entered) {
    r->outcome = RFD_OUT_NO_RESPONSE;
    r->flags |= RFD_FLAG_ATTENTION;
    if (d->saw_plus_ok) {
      /* It answered "+++" but not AT: it may sit in command mode. */
      exit_command_mode(d, r);
      r->data_mode = RFD_DATA_UNKNOWN;
    }
    return 0;
  }
  r->data_mode = RFD_DATA_UNKNOWN;
  parse_ctx_t c = {.snap = &snap, .plan = plan};
  TRY(command(d, "ATI", d->t->command, on_identity, &c));
  TRY(read_snapshot(d, plan, &snap));
  TRY(command(d, "AT+F?", d->t->command, on_forwarding, &c));
  if (c.forwarding) r->flags |= RFD_FLAG_FORWARDING | RFD_FLAG_ATTENTION;
  const rfd_map_t source = map_source(c.async_identity, &snap, plan);
  r->map_source = (uint8_t)source;
  fill_observed(r, &snap);

  uint32_t pending = 0;
  bool other_differs = false;
  for (unsigned p = 0; p < plan->count; p++) {
    const rfd_status_t st = reg_status(plan, &snap, p);
    if (st == RFD_ST_DIFFERS && plan->regs[p].policy == RFD_POLICY_FIX) {
      pending |= (uint32_t)1U << p;
    } else if (st == RFD_ST_DIFFERS || st == RFD_ST_UNREAD || st == RFD_ST_NAME_MISMATCH) {
      other_differs = true;
    }
  }
  if (other_differs) r->flags |= RFD_FLAG_ATTENTION;
  if (!pending) {
    r->outcome = other_differs ? RFD_OUT_MISMATCH : RFD_OUT_MATCHED;
    exit_command_mode(d, r);
    return 0;
  }
  bool blocked = mode != RFD_MODE_FIX || source == RFD_MAP_UNCONFIRMED || !allow_save;
  if (!blocked && !gate(gate_ctx)) {
    blocked = true;
    r->flags |= RFD_FLAG_GATE_SKIPPED;
  }
  if (blocked) {
    r->outcome = RFD_OUT_MISMATCH;
    exit_command_mode(d, r);
    return 0;
  }
  for (unsigned p = 0; p < plan->count; p++) {
    if (!(pending & ((uint32_t)1U << p)) || r->write_count >= RFD_MAX_WRITES) continue;
    char cmd[16];
    const uint8_t number = snap.number[p];
    const uint16_t value = (uint16_t)plan->regs[p].expected;
    fmt_write(cmd, number, value);
    d->ram_dirty = true;
    TRY(command(d, cmd, d->t->command, NULL, NULL));
    const unsigned w = r->write_count++;
    r->write_number[w] = number;
    if (!d->ok) {
      /* ATZ without AT&W reloads EEPROM and discards every RAM change. */
      r->outcome = RFD_OUT_APPLY_FAILED;
      return reboot(d, r);
    }
  }
  if (!gate(gate_ctx)) {
    r->outcome = RFD_OUT_MISMATCH;
    r->flags |= RFD_FLAG_GATE_SKIPPED;
    return reboot(d, r);
  }
  r->save_attempted = true;
  TRY(command(d, "AT&W", d->t->save, NULL, NULL));
  if (!d->ok) {
    r->outcome = RFD_OUT_APPLY_FAILED;
    return reboot(d, r);
  }
  r->flags |= RFD_FLAG_SAVED;
  d->ram_dirty = false;
  TRY(reboot(d, r));
  TRY(entered = enter_command_mode(d));
  if (!entered) {
    r->outcome = RFD_OUT_VERIFY_FAILED;
    return 0;
  }
  r->data_mode = RFD_DATA_UNKNOWN;
  TRY(read_snapshot(d, plan, &snap));
  fill_observed(r, &snap);
  bool still = false, unwritten = false;
  for (unsigned p = 0; p < plan->count; p++) {
    const rfd_status_t st = reg_status(plan, &snap, p);
    if ((pending & ((uint32_t)1U << p)) && snap.value[p] != plan->regs[p].expected) still = true;
    if (plan->regs[p].policy == RFD_POLICY_REPORT &&
        (st == RFD_ST_DIFFERS || st == RFD_ST_UNREAD || st == RFD_ST_NAME_MISMATCH)) {
      unwritten = true;
    }
  }
  r->outcome = still ? RFD_OUT_VERIFY_FAILED : unwritten ? RFD_OUT_MISMATCH : RFD_OUT_APPLIED;
  exit_command_mode(d, r);
  return 0;
}

void rfd_report_skipped(rfd_report_t *r, rfd_mode_t mode, uint8_t flags)
{
  memset(r, 0, sizeof(*r));
  r->outcome = RFD_OUT_SKIPPED;
  r->map_source = RFD_MAP_UNCONFIRMED;
  r->mode = (uint8_t)mode;
  r->data_mode = RFD_DATA_UNTOUCHED;
  r->flags = flags;
  for (unsigned k = 0; k < 7U; k++) r->observed[k] = RFD_UNREAD;
}

void rfd_modem_run(const rfd_io_t *io, const rfd_plan_t *plan, rfd_mode_t mode,
                   const rfd_timing_t *timing, rfd_gate_fn gate, void *gate_ctx,
                   bool allow_save, rfd_report_t *out)
{
  rfd_report_skipped(out, mode, 0U);
  if (mode == RFD_MODE_SKIP) return;
  if (!gate(gate_ctx)) {
    out->flags |= RFD_FLAG_GATE_SKIPPED;
    return;
  }
  driver_t d;
  memset(&d, 0, sizeof(d));
  d.io = io;
  d.t = timing;
  d.gate = gate;
  d.gate_ctx = gate_ctx;
  d.start = io->now_ms(io->ctx);
  out->outcome = RFD_OUT_FAILED;
  const int rc = run_inner(&d, plan, mode, gate, gate_ctx, allow_save, out);
  if (rc < 0) {
    const bool touched = d.plus_sent || d.in_command || (out->flags & RFD_FLAG_REBOOTED);
    if (rc != ABORT_IO) fast_exit(&d, out);
    out->outcome = (out->flags & RFD_FLAG_SAVED) ? RFD_OUT_VERIFY_FAILED : RFD_OUT_FAILED;
    if (rc == ABORT_GATE) {
      out->flags |= RFD_FLAG_GATE_SKIPPED;
      if (!(out->flags & RFD_FLAG_SAVED))
        out->outcome = touched ? RFD_OUT_MISMATCH : RFD_OUT_SKIPPED;
    }
    if (out->data_mode == RFD_DATA_UNTOUCHED && touched) out->data_mode = RFD_DATA_UNKNOWN;
  }
  if (out->outcome != RFD_OUT_MATCHED && out->outcome != RFD_OUT_APPLIED &&
      out->outcome != RFD_OUT_SKIPPED) {
    out->flags |= RFD_FLAG_ATTENTION;
  }
}

/* --- flight gate and status block ---------------------------------------- */

/* Startup and Idle only: PreFill and FillTest are hazardous, and an Abort
 * sent over the radio there must never meet a held radio. */
bool rfd_flight_state_safe(uint8_t state) { return state <= 1U; }

/* Unread (negative) -> all ones; values at or above it -> all ones minus 1. */
static uint32_t field(int32_t v, uint32_t ones)
{
  return v < 0 ? ones : ((uint32_t)v >= ones ? ones - 1U : (uint32_t)v);
}

void rfd_status_words(const rfd_report_t *r, bool rolled_out, uint32_t words[3])
{
  const int32_t *o = r->observed;
  const uint32_t rts = o[6] < 0 ? 3U : (o[6] > 1 ? 2U : (uint32_t)o[6]);
  words[0] = 1U | ((uint32_t)(r->outcome & 0xFU) << 4) | ((uint32_t)(r->map_source & 3U) << 8) |
             ((uint32_t)(r->mode & 3U) << 10) | ((rolled_out ? 1U : 0U) << 12) | (rts << 13) |
             ((uint32_t)r->flags << 16) | (field(o[3], 0xFFU) << 24);
  words[1] = field(o[1], 0xFFFFU) | (field(o[4], 0xFFFFU) << 16);
  words[2] = field(o[5], 0xFFFFU) | (field(o[0], 0xFFU) << 16) | (field(o[2], 0xFFU) << 24);
}
