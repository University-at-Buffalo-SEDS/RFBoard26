/* Native tests for Core/Src/rfd_modem.c against a simulated RFD900x ASYNC
 * modem. The double follows the same rules as the GroundStation26 test
 * modem (rfd_modem_tests.rs): "+++" only after one second of host silence and
 * OK after a further second, echoed command characters, ATI/ATI5/ATSn?
 * answers without a trailing OK, ATZ reloading EEPROM and rebooting into data
 * mode, and ATO without output. Virtual clock; no serial device. */
#include "rfd_modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(cond)                                                             \
  do {                                                                          \
    if (!(cond)) {                                                              \
      fprintf(stderr, "%s:%d: %s: CHECK(%s) failed\n", __FILE__, __LINE__,      \
              current_test, #cond);                                             \
      exit(1);                                                                  \
    }                                                                           \
  } while (0)

static const char *current_test = "";

enum { DATA, COMMAND, REBOOTING };
#define MAX_REGS 32
#define QUEUE 16384

typedef struct {
  uint8_t number;
  const char *name;
  int32_t ram, eeprom;
} reg_t;

typedef struct {
  uint64_t now_us;
  int state;
  uint64_t reboot_until;
  reg_t regs[MAX_REGS];
  int nregs;
  uint64_t q_at[QUEUE];
  uint8_t q_b[QUEUE];
  size_t q_head, q_len;
  uint64_t last_host_byte;
  uint64_t plus_at;
  int pluses;
  char line[128];
  size_t line_len;
  unsigned air_bytes, eeprom_writes, written;
  /* faults */
  int dead, garbage, ato_error, save_does_not_persist, no_ati5_names, noise;
  int refuse_reg;
  size_t max_write;
  long write_error_after;
  const char *forwarding;
  /* write log of every ATSn=X the modem accepted */
  uint8_t set_numbers[16];
  int nset;
} modem_t;

static const reg_t DEFAULTS[] = {
    {0, "FORMAT", 27, 27}, {1, "SERIAL_SPEED", 115, 115}, {2, "AIR_SPEED", 64, 64},
    {3, "MAX_DATA", 3968, 3968}, {4, "MAX_RETRIES", 10, 10}, {5, "GLOBAL_RETRIES", 10, 10},
    {6, "SER_BREAK_DETECT_MS", 50, 50}, {7, "TXENCAP", 0, 0}, {8, "RXENCAP", 0, 0},
    {9, "NETID", 25, 25}, {10, "NODEID", 1, 1}, {11, "DESTID", 32768, 32768},
    {12, "TXPOWER", 30, 30}, {13, "MAVLINK1", 0, 0}, {14, "MIN_FREQ", 915000, 915000},
    {15, "MAX_FREQ", 928000, 928000}, {16, "NUM_CHANNELS", 20, 20}, {17, "LBT_RSSI", 0, 0},
    {18, "RTSCTS", 0, 0}, {19, "ENCRYPTION_LEVEL", 0, 0}, {22, "ANTMODE", 2, 2},
};

static modem_t *modem_new(void)
{
  modem_t *m = calloc(1, sizeof(*m));
  m->now_us = 100000000ULL;
  m->state = DATA;
  m->nregs = (int)(sizeof(DEFAULTS) / sizeof(DEFAULTS[0]));
  memcpy(m->regs, DEFAULTS, sizeof(DEFAULTS));
  m->refuse_reg = -1;
  m->max_write = (size_t)-1;
  m->write_error_after = -1;
  m->forwarding = "Forwarding buffer 100";
  return m;
}

static reg_t *reg(modem_t *m, int number)
{
  for (int i = 0; i < m->nregs; i++) {
    if (m->regs[i].number == number) return &m->regs[i];
  }
  return NULL;
}

static void set_reg(modem_t *m, int number, const char *name, int32_t value)
{
  reg_t *r = reg(m, number);
  if (!r) {
    r = &m->regs[m->nregs++];
    r->number = (uint8_t)number;
    /* keep ATI5 ordered */
    for (int i = m->nregs - 1; i > 0 && m->regs[i - 1].number > m->regs[i].number; i--) {
      reg_t t = m->regs[i]; m->regs[i] = m->regs[i - 1]; m->regs[i - 1] = t; r = &m->regs[i - 1];
    }
  }
  if (name) r->name = name;
  r->ram = r->eeprom = value;
}

static void push(modem_t *m, uint64_t at, uint8_t b)
{
  if (m->q_len == QUEUE) return;
  const size_t i = (m->q_head + m->q_len++) % QUEUE;
  m->q_at[i] = at;
  m->q_b[i] = b;
}

static void say(modem_t *m, const char *text)
{
  const uint64_t at = m->now_us + 5000U;
  for (; *text; text++) push(m, at, (uint8_t)*text);
}

static void tick(modem_t *m)
{
  if (m->state == REBOOTING && m->now_us >= m->reboot_until) {
    m->state = DATA;
    for (int i = 0; i < m->nregs; i++) m->regs[i].ram = m->regs[i].eeprom;
  }
  if (m->pluses == 3 && m->now_us >= m->plus_at + 1000000U && m->last_host_byte <= m->plus_at) {
    m->pluses = 0;
    if (!m->dead) {
      m->state = COMMAND;
      say(m, "OK\r\n");
    }
  }
}

static void execute(modem_t *m, const char *cmd)
{
  char out[2048];
  if (m->garbage) {
    say(m, "\x7f#%\r\nZ9q\r\n");
    return;
  }
  out[0] = 0;
  if (!strcmp(cmd, "AT")) {
    strcpy(out, "OK\r\n");
  } else if (!strcmp(cmd, "ATI")) {
    strcpy(out, "RFD ASYNC 4.03 on RFD900X2\r\n");
  } else if (!strcmp(cmd, "ATI5")) {
    size_t k = 0;
    for (int i = 0; i < m->nregs; i++) {
      if (m->no_ati5_names) {
        k += (size_t)snprintf(out + k, sizeof(out) - k, "%ld\r\n", (long)m->regs[i].ram);
      } else {
        k += (size_t)snprintf(out + k, sizeof(out) - k, "S%u:%s=%ld\r\n", m->regs[i].number,
                              m->regs[i].name, (long)m->regs[i].ram);
      }
    }
  } else if (!strcmp(cmd, "AT+F?")) {
    snprintf(out, sizeof(out), "%s\r\n", m->forwarding);
  } else if (!strcmp(cmd, "AT&W")) {
    if (!m->save_does_not_persist) {
      for (int i = 0; i < m->nregs; i++) m->regs[i].eeprom = m->regs[i].ram;
    }
    m->eeprom_writes++;
    strcpy(out, "OK\r\n");
  } else if (!strcmp(cmd, "ATZ")) {
    m->state = REBOOTING;
    m->reboot_until = m->now_us + 1500000U;
  } else if (!strcmp(cmd, "ATO")) {
    if (m->ato_error) strcpy(out, "ERROR\r\n");
    else m->state = DATA;
  } else {
    int n;
    long v;
    char tail;
    if (sscanf(cmd, "ATS%d?%c", &n, &tail) == 1 && cmd[strlen(cmd) - 1] == '?' && reg(m, n)) {
      snprintf(out, sizeof(out), "%ld\r\n", (long)reg(m, n)->ram);
    } else if (sscanf(cmd, "ATS%d=%ld%c", &n, &v, &tail) == 2 && reg(m, n) && n != m->refuse_reg) {
      reg(m, n)->ram = (int32_t)v;
      if (m->nset < 16) m->set_numbers[m->nset++] = (uint8_t)n;
      strcpy(out, "OK\r\n");
    } else {
      strcpy(out, "ERROR\r\n");
    }
  }
  say(m, out);
}

static int io_write(void *ctx, const uint8_t *bytes, size_t len)
{
  modem_t *m = ctx;
  tick(m);
  if (m->write_error_after >= 0 && (long)m->written >= m->write_error_after) return -1;
  const size_t n = len < m->max_write ? len : m->max_write;
  m->written += (unsigned)n;
  for (size_t i = 0; i < n; i++) {
    const uint8_t b = bytes[i];
    const int silent = m->now_us - m->last_host_byte >= 1000000U;
    if (m->state == DATA) {
      if (b == '+' && (m->pluses > 0 || silent) && m->pluses < 3) {
        m->pluses++;
        m->plus_at = m->now_us;
      } else {
        m->air_bytes += 1U + (unsigned)m->pluses;
        m->pluses = 0;
      }
    } else if (m->state == COMMAND) {
      if (!m->garbage) push(m, m->now_us, b);
      if (b == '\r') {
        m->line[m->line_len] = 0;
        char *s = m->line;
        while (*s == ' ') s++;
        size_t l = strlen(s);
        while (l && s[l - 1] == ' ') s[--l] = 0;
        if (l) execute(m, s);
        m->line_len = 0;
      } else if (m->line_len < sizeof(m->line) - 1) {
        m->line[m->line_len++] = (char)b;
      }
    }
    m->last_host_byte = m->now_us;
  }
  m->now_us += 87U * n;
  return (int)n;
}

static int io_read(void *ctx, uint8_t *buf, size_t cap)
{
  modem_t *m = ctx;
  tick(m);
  if (m->noise && m->state == DATA) {
    static const uint8_t decoy[] = {0x00, 0x7e, 'O', 'K', '\r', '\n', 0x55, 0xaa};
    for (size_t i = 0; i < sizeof(decoy); i++) push(m, m->now_us, decoy[i]);
  }
  size_t n = 0;
  while (n < cap && m->q_len && m->q_at[m->q_head] <= m->now_us) {
    buf[n++] = m->q_b[m->q_head];
    m->q_head = (m->q_head + 1) % QUEUE;
    m->q_len--;
  }
  m->now_us += n ? 87U * n : 20000U;
  return (int)n;
}

static uint32_t io_now(void *ctx) { return (uint32_t)(((modem_t *)ctx)->now_us / 1000U); }
static void io_sleep(void *ctx, uint32_t ms)
{
  modem_t *m = ctx;
  m->now_us += (uint64_t)ms * 1000U;
  tick(m);
}

static bool gate_open(void *ctx) { (void)ctx; return true; }
static bool gate_closed(void *ctx) { (void)ctx; return false; }
/* Opens for the first n calls, then closes (flight state changed mid-check). */
static bool gate_countdown(void *ctx)
{
  int *n = ctx;
  return (*n)-- > 0;
}

static rfd_profile_values_t profile(bool rolled_out)
{
  rfd_profile_values_t p = {
      .serial_baud = 115200, .air_bps = 64000, .max_data_bytes = 3968, .max_retries = 10,
      .broadcast_retries = 10, .serial_break_ms = 50, .tx_encapsulation = 0,
      .rx_encapsulation = 0, .hardware_flow_control = false, .rolled_out_all_nodes = rolled_out,
  };
  return p;
}

static rfd_report_t check_full(modem_t *m, rfd_profile_values_t p, rfd_mode_t mode,
                               rfd_gate_fn gate, void *gate_ctx, bool allow_save)
{
  rfd_plan_t plan;
  rfd_report_t r;
  rfd_plan_init(&plan, &p);
  const rfd_io_t io = {io_write, io_read, io_now, io_sleep, m};
  rfd_modem_run(&io, &plan, mode, &RFD_TIMING, gate, gate_ctx, allow_save, &r);
  return r;
}

static rfd_report_t check(modem_t *m, rfd_mode_t mode)
{
  return check_full(m, profile(false), mode, gate_open, NULL, true);
}

#define TEST(name) static void name(void)
#define RUN(name) do { current_test = #name; name(); printf("ok %s\n", #name); } while (0)

TEST(matching_modem_is_read_and_left_alone)
{
  modem_t *m = modem_new();
  const uint64_t t0 = m->now_us;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_MATCHED);
  CHECK(r.write_count == 0 && m->eeprom_writes == 0);
  CHECK(!(r.flags & (RFD_FLAG_SAVED | RFD_FLAG_REBOOTED | RFD_FLAG_ATTENTION)));
  CHECK(r.flags & RFD_FLAG_RESUMED);
  CHECK(r.data_mode == RFD_DATA_RESUMED && m->state == DATA);
  CHECK(r.map_source == RFD_MAP_ATI5);
  CHECK(r.observed[0] == 64 && r.observed[1] == 3968 && r.observed[5] == 32768);
  CHECK(r.observed[6] == 0 && r.rtscts_number == 18);
  CHECK(m->now_us - t0 < 6000000U);
  free(m);
}

TEST(local_drift_is_written_saved_rebooted_and_verified_once)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  set_reg(m, 11, NULL, 1);
  set_reg(m, 18, NULL, 1);
  const uint64_t t0 = m->now_us;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_APPLIED);
  CHECK(r.write_count == 3);
  CHECK(r.write_number[0] == 6);
  CHECK(r.write_number[1] == 11);
  CHECK(r.write_number[2] == 18);
  CHECK(r.flags & RFD_FLAG_SAVED && r.flags & RFD_FLAG_REBOOTED && r.flags & RFD_FLAG_RESUMED);
  CHECK(!(r.flags & RFD_FLAG_ATTENTION));
  CHECK(m->eeprom_writes == 1);
  CHECK(reg(m, 6)->eeprom == 50 && reg(m, 11)->eeprom == 32768 && reg(m, 18)->eeprom == 0);
  CHECK(r.observed[4] == 50 && r.observed[5] == 32768);
  CHECK(m->state == DATA && m->air_bytes == 0);
  CHECK(m->now_us - t0 < RFD_TIMING.overall * 1000ULL);
  free(m);
}

TEST(verify_mode_never_writes)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  rfd_report_t r = check(m, RFD_MODE_VERIFY);
  CHECK(r.outcome == RFD_OUT_MISMATCH && (r.flags & RFD_FLAG_ATTENTION));
  CHECK(r.write_count == 0 && m->eeprom_writes == 0 && m->nset == 0);
  CHECK(r.observed[4] == 20);
  CHECK(m->state == DATA);
  free(m);
}

TEST(skip_mode_and_closed_gate_send_nothing)
{
  modem_t *m = modem_new();
  rfd_report_t r = check(m, RFD_MODE_SKIP);
  CHECK(r.outcome == RFD_OUT_SKIPPED && m->written == 0);
  r = check_full(m, profile(false), RFD_MODE_FIX, gate_closed, NULL, true);
  CHECK(r.outcome == RFD_OUT_SKIPPED && (r.flags & RFD_FLAG_GATE_SKIPPED));
  CHECK(m->written == 0); /* not even +++ */
  CHECK(r.observed[1] == RFD_UNREAD);
  free(m);
}

TEST(gate_closing_after_the_read_prevents_every_write)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  int opens = 1; /* allowed to start, closed at the first-write check */
  rfd_report_t r = check_full(m, profile(false), RFD_MODE_FIX, gate_countdown, &opens, true);
  CHECK(r.outcome == RFD_OUT_MISMATCH && (r.flags & RFD_FLAG_GATE_SKIPPED));
  CHECK(r.write_count == 0 && m->nset == 0 && m->eeprom_writes == 0);
  CHECK(m->state == DATA);
  /* Closing between the writes and AT&W discards the RAM change. */
  free(m);
  m = modem_new();
  set_reg(m, 6, NULL, 20);
  opens = 2;
  r = check_full(m, profile(false), RFD_MODE_FIX, gate_countdown, &opens, true);
  CHECK(r.outcome == RFD_OUT_MISMATCH && (r.flags & RFD_FLAG_GATE_SKIPPED));
  CHECK(r.write_count == 1 && m->eeprom_writes == 0);
  CHECK((r.flags & RFD_FLAG_REBOOTED) && reg(m, 6)->ram == 20 && reg(m, 6)->eeprom == 20);
  free(m);
}

TEST(silent_modem_gives_up_and_leaves_the_link)
{
  modem_t *m = modem_new();
  m->dead = 1;
  const uint64_t t0 = m->now_us;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_NO_RESPONSE && (r.flags & RFD_FLAG_ATTENTION));
  CHECK(m->eeprom_writes == 0 && r.write_count == 0);
  CHECK(m->now_us - t0 < 12000000U);
  CHECK(r.observed[1] == RFD_UNREAD);
  free(m);
}

TEST(serial_failure_mid_check_is_bounded)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  m->write_error_after = 20;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_FAILED && (r.flags & RFD_FLAG_ATTENTION));
  CHECK(m->eeprom_writes == 0);
  free(m);
}

TEST(modem_that_never_answers_is_bounded_by_the_overall_limit)
{
  /* Answers +++ but then echoes nothing and never says OK to AT. */
  modem_t *m = modem_new();
  m->garbage = 1;
  const uint64_t t0 = m->now_us;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome != RFD_OUT_APPLIED && r.write_count == 0 && m->eeprom_writes == 0);
  CHECK(m->now_us - t0 < (RFD_TIMING.overall + 5000U) * 1000ULL);
  free(m);
}

TEST(partial_serial_writes_are_completed)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  m->max_write = 1;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_APPLIED && m->eeprom_writes == 1);
  free(m);
}

TEST(refused_write_discards_ram_changes_without_saving)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  set_reg(m, 11, NULL, 1);
  m->refuse_reg = 11;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_APPLY_FAILED && m->eeprom_writes == 0);
  CHECK((r.flags & RFD_FLAG_REBOOTED) && r.data_mode == RFD_DATA_REBOOTED);
  CHECK(r.write_count == 2 && r.write_number[1] == 11);
  m->now_us += 5000000U;
  tick(m);
  CHECK(m->state == DATA && reg(m, 6)->ram == 20);
  free(m);
}

TEST(modem_that_ignores_ato_is_rebooted_into_data_mode)
{
  modem_t *m = modem_new();
  m->ato_error = 1;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_MATCHED && r.data_mode == RFD_DATA_REBOOTED);
  CHECK(r.flags & RFD_FLAG_ATTENTION);
  m->now_us += 5000000U;
  tick(m);
  CHECK(m->state == DATA && m->eeprom_writes == 0);
  free(m);
}

TEST(save_that_does_not_persist_is_reported_and_not_retried)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  m->save_does_not_persist = 1;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_VERIFY_FAILED && m->eeprom_writes == 1);
  CHECK(m->state == DATA);
  free(m);
}

TEST(link_wide_registers_not_rolled_out_are_reported_never_written)
{
  modem_t *m = modem_new();
  set_reg(m, 2, NULL, 125);  /* always report-only */
  set_reg(m, 3, NULL, 2048); /* link-wide */
  set_reg(m, 5, NULL, 0);    /* link-wide */
  set_reg(m, 17, NULL, 3);   /* LBT_RSSI */
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_MISMATCH && (r.flags & RFD_FLAG_ATTENTION));
  CHECK(r.write_count == 0 && m->nset == 0 && m->eeprom_writes == 0);
  CHECK(reg(m, 3)->eeprom == 2048 && reg(m, 5)->eeprom == 0);
  CHECK(r.observed[0] == 125 && r.observed[1] == 2048 && r.observed[3] == 0);
  /* Local drift beside it is still repaired, and the result stays mismatch. */
  free(m);
  m = modem_new();
  set_reg(m, 5, NULL, 0);
  set_reg(m, 6, NULL, 20);
  r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_MISMATCH && r.write_count == 1 && r.write_number[0] == 6);
  CHECK(reg(m, 6)->eeprom == 50 && reg(m, 5)->eeprom == 0);
  free(m);
}

TEST(rolled_out_profile_writes_link_wide_registers_but_never_s2_or_s17)
{
  modem_t *m = modem_new();
  set_reg(m, 5, NULL, 0);
  set_reg(m, 3, NULL, 2048);
  set_reg(m, 17, NULL, 3);
  rfd_report_t r = check_full(m, profile(true), RFD_MODE_FIX, gate_open, NULL, true);
  CHECK(r.outcome == RFD_OUT_MISMATCH); /* S17 left in place */
  CHECK(r.write_count == 2 && r.write_number[0] == 3 && r.write_number[1] == 5);
  CHECK(reg(m, 5)->eeprom == 10 && reg(m, 3)->eeprom == 3968 && reg(m, 17)->eeprom == 3);
  free(m);
}

TEST(unknown_register_names_block_writes)
{
  modem_t *m = modem_new();
  set_reg(m, 6, "SOMETHING_ELSE", 20);
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.map_source == RFD_MAP_UNCONFIRMED && r.outcome == RFD_OUT_MISMATCH);
  CHECK(r.write_count == 0 && m->nset == 0);
  CHECK(r.observed[4] == 20);
  free(m);
}

TEST(unnamed_ati5_falls_back_to_a_v1_1_fingerprint)
{
  modem_t *m = modem_new();
  m->no_ati5_names = 1;
  set_reg(m, 6, NULL, 20);
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.map_source == RFD_MAP_FINGERPRINT && r.outcome == RFD_OUT_APPLIED);
  CHECK(reg(m, 6)->eeprom == 50);
  free(m);
}

TEST(modem_left_in_command_mode_is_recovered)
{
  modem_t *m = modem_new();
  m->state = COMMAND;
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_MATCHED && m->state == DATA);
  free(m);
}

TEST(streaming_air_data_with_a_decoy_ok_is_handled)
{
  modem_t *m = modem_new();
  m->noise = 1;
  set_reg(m, 6, NULL, 20);
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_APPLIED && m->eeprom_writes == 1);
  free(m);
}

TEST(used_save_budget_blocks_writes)
{
  modem_t *m = modem_new();
  set_reg(m, 6, NULL, 20);
  rfd_report_t r = check_full(m, profile(false), RFD_MODE_FIX, gate_open, NULL, false);
  CHECK(r.outcome == RFD_OUT_MISMATCH && r.write_count == 0 && m->eeprom_writes == 0);
  free(m);
}

TEST(forwarding_entries_are_flagged)
{
  modem_t *m = modem_new();
  m->forwarding = "F0=1,3 F1=0,0";
  rfd_report_t r = check(m, RFD_MODE_VERIFY);
  CHECK(r.outcome == RFD_OUT_MATCHED);
  CHECK((r.flags & RFD_FLAG_FORWARDING) && (r.flags & RFD_FLAG_ATTENTION));
  free(m);
  m = modem_new();
  m->forwarding = "F0=0,0";
  r = check(m, RFD_MODE_VERIFY);
  CHECK(!(r.flags & RFD_FLAG_FORWARDING));
  /* ASYNC 4.03 field format: the comma between entries is not a pair. */
  free(m);
  m = modem_new();
  m->forwarding = "0:0->0,1:0->0,2:0->0,3:0->0,4:0->0,5:0->0,6:0->0,7:0->0,8:0->0,9:0->0, | FwdBuff=13320";
  r = check(m, RFD_MODE_VERIFY);
  CHECK(r.outcome == RFD_OUT_MATCHED && !(r.flags & (RFD_FLAG_FORWARDING | RFD_FLAG_ATTENTION)));
  free(m);
  m = modem_new();
  m->forwarding = "0:0->0,1:3->65535,2:0->0, | FwdBuff=13320";
  r = check(m, RFD_MODE_VERIFY);
  CHECK(r.flags & RFD_FLAG_FORWARDING);
  free(m);
}

/* RFD ASYNC 4.03 on RFD900x2 as read in the field: S6-S8 renamed, DUTY_CYCLE
 * at S17 shifts LBT_RSSI to S18 and RTSCTS to S19. */
static modem_t *field_403(void)
{
  modem_t *m = modem_new();
  set_reg(m, 6, "SER_BRK_DETMS", 50);
  set_reg(m, 7, "TX_ENCAP_METHOD", 0);
  set_reg(m, 8, "RX_ENCAP_METHOD", 0);
  set_reg(m, 17, "DUTY_CYCLE", 100);
  set_reg(m, 18, "LBT_RSSI", 0);
  set_reg(m, 19, "RTSCTS", 0);
  set_reg(m, 20, "MAX_WINDOW", 131);
  set_reg(m, 22, "RESV", 0);
  return m;
}

TEST(field_403_layout_is_matched_by_register_name)
{
  modem_t *m = field_403();
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.map_source == RFD_MAP_ATI5 && r.outcome == RFD_OUT_MATCHED);
  CHECK(!(r.flags & RFD_FLAG_ATTENTION));
  CHECK(r.write_count == 0 && m->eeprom_writes == 0);
  CHECK(r.rtscts_number == 19 && r.observed[6] == 0);
  free(m);
}

TEST(field_403_layout_writes_flow_control_to_its_named_register)
{
  modem_t *m = field_403();
  set_reg(m, 19, NULL, 1);
  set_reg(m, 6, NULL, 20);
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_APPLIED);
  CHECK(r.write_count == 2);
  CHECK(r.write_number[0] == 6 && reg(m, 6)->eeprom == 50);
  CHECK(r.write_number[1] == 19);
  for (int i = 0; i < m->nset; i++) CHECK(m->set_numbers[i] != 18 && m->set_numbers[i] != 17);
  CHECK(reg(m, 18)->eeprom == 0 && reg(m, 17)->eeprom == 100 && reg(m, 19)->eeprom == 0);
  free(m);
}

TEST(field_403_lbt_rssi_drift_is_reported_at_its_named_register)
{
  modem_t *m = field_403();
  set_reg(m, 18, NULL, 5); /* LBT_RSSI: report only */
  rfd_report_t r = check(m, RFD_MODE_FIX);
  CHECK(r.outcome == RFD_OUT_MISMATCH && r.write_count == 0 && m->nset == 0);
  free(m);
}

static const rfd_expect_t *named(const rfd_plan_t *plan, const char *first)
{
  for (unsigned i = 0; i < plan->count; i++) {
    if (!strncmp(plan->regs[i].names, first, strlen(first))) return &plan->regs[i];
  }
  return NULL;
}

TEST(plan_policies_follow_rollout_and_flow_control)
{
  rfd_plan_t plan;
  rfd_profile_values_t p = profile(false);
  rfd_plan_init(&plan, &p);
  CHECK(plan.count == 12);
  for (unsigned i = 0; i < plan.count; i++) {
    const rfd_expect_t *e = &plan.regs[i];
    CHECK(e->policy != RFD_POLICY_INFO && e->expected != RFD_UNREAD);
    if (e->number >= 3 && e->number <= 8 && e->number != 6) CHECK(e->policy == RFD_POLICY_REPORT);
    if (e->number == 6 || e->number == 11 || e->number == 13) CHECK(e->policy == RFD_POLICY_FIX);
  }
  CHECK(named(&plan, "RTSCTS")->policy == RFD_POLICY_FIX && named(&plan, "RTSCTS")->expected == 0);
  CHECK(named(&plan, "LBT_RSSI")->policy == RFD_POLICY_REPORT);
  CHECK(named(&plan, "MAX_DATA")->expected == 3968 && named(&plan, "DESTID")->expected == 32768);
  p.rolled_out_all_nodes = true;
  p.hardware_flow_control = true;
  p.broadcast_retries = 0;
  p.serial_break_ms = 20;
  rfd_plan_init(&plan, &p);
  for (unsigned i = 0; i < plan.count; i++) {
    const rfd_expect_t *e = &plan.regs[i];
    if (e->number >= 3 && e->number <= 8) CHECK(e->policy == RFD_POLICY_FIX);
  }
  /* Never S1, S2 or LBT_RSSI, even when rolled out. */
  CHECK(named(&plan, "SERIAL_SPEED")->policy == RFD_POLICY_REPORT);
  CHECK(named(&plan, "AIR_SPEED")->policy == RFD_POLICY_REPORT);
  CHECK(named(&plan, "LBT_RSSI")->policy == RFD_POLICY_REPORT);
  CHECK(named(&plan, "RTSCTS")->expected == 1);
  CHECK(named(&plan, "GLOBAL_RETRIES")->expected == 0 && named(&plan, "SER_BREAK")->expected == 20);
}

TEST(flight_gate_runs_only_on_a_safe_network_flight_state)
{
  /* No FLIGHT_STATE yet: wait (the radio keeps running meanwhile). */
  CHECK(rfd_gate_decide(false, 0, false, 0) == RFD_GATE_WAIT);
  /* A restored safe state still waits for the network value. */
  CHECK(rfd_gate_decide(true, 1, false, 0) == RFD_GATE_WAIT);
  for (unsigned s = 0; s <= 15U; s++) {
    const uint8_t st = (uint8_t)s;
    const bool safe = s <= 3U; /* Startup, Idle, PreFill, FillTest */
    CHECK(rfd_flight_state_safe(st) == safe);
    CHECK(rfd_gate_decide(false, 0, true, st) == (safe ? RFD_GATE_RUN : RFD_GATE_SKIP));
    CHECK(rfd_gate_decide(true, 0, true, st) == (safe ? RFD_GATE_RUN : RFD_GATE_SKIP));
    /* A restored flight phase (brown-out reboot) skips at once, even if the
     * network later says Startup. */
    if (!safe) CHECK(rfd_gate_decide(true, st, false, 0) == RFD_GATE_SKIP);
    if (!safe) CHECK(rfd_gate_decide(true, st, true, 0) == RFD_GATE_SKIP);
  }
  CHECK(!rfd_flight_state_safe(7) && !rfd_flight_state_safe(255));
}

TEST(status_words_pack_the_report)
{
  modem_t *m = field_403();
  set_reg(m, 6, NULL, 20);
  rfd_report_t r = check(m, RFD_MODE_FIX);
  uint32_t w[3];
  rfd_status_words(&r, false, w);
  CHECK((w[0] & 0xFU) == 1U);
  CHECK(((w[0] >> 4) & 0xFU) == RFD_OUT_APPLIED);
  CHECK(((w[0] >> 8) & 3U) == RFD_MAP_ATI5);
  CHECK(((w[0] >> 10) & 3U) == RFD_MODE_FIX);
  CHECK(((w[0] >> 12) & 1U) == 0U);
  CHECK(((w[0] >> 13) & 3U) == 0U);
  CHECK(((w[0] >> 16) & 0xFFU) == (RFD_FLAG_SAVED | RFD_FLAG_REBOOTED | RFD_FLAG_RESUMED));
  CHECK((w[0] >> 24) == 10U);
  CHECK(w[1] == (3968U | (50U << 16)));
  CHECK(w[2] == (32768U | (64U << 16) | (10U << 24)));
  rfd_report_skipped(&r, RFD_MODE_SKIP, RFD_FLAG_GATE_SKIPPED);
  rfd_status_words(&r, true, w);
  CHECK(w[0] == (1U | (2U << 8) | (3U << 13) | (1U << 12) | ((uint32_t)RFD_FLAG_GATE_SKIPPED << 16) | (0xFFU << 24)));
  CHECK(w[1] == 0xFFFFFFFFU && w[2] == 0xFFFFFFFFU);
  free(m);
}

int main(void)
{
  RUN(matching_modem_is_read_and_left_alone);
  RUN(local_drift_is_written_saved_rebooted_and_verified_once);
  RUN(verify_mode_never_writes);
  RUN(skip_mode_and_closed_gate_send_nothing);
  RUN(gate_closing_after_the_read_prevents_every_write);
  RUN(silent_modem_gives_up_and_leaves_the_link);
  RUN(serial_failure_mid_check_is_bounded);
  RUN(modem_that_never_answers_is_bounded_by_the_overall_limit);
  RUN(partial_serial_writes_are_completed);
  RUN(refused_write_discards_ram_changes_without_saving);
  RUN(modem_that_ignores_ato_is_rebooted_into_data_mode);
  RUN(save_that_does_not_persist_is_reported_and_not_retried);
  RUN(link_wide_registers_not_rolled_out_are_reported_never_written);
  RUN(rolled_out_profile_writes_link_wide_registers_but_never_s2_or_s17);
  RUN(unknown_register_names_block_writes);
  RUN(unnamed_ati5_falls_back_to_a_v1_1_fingerprint);
  RUN(modem_left_in_command_mode_is_recovered);
  RUN(streaming_air_data_with_a_decoy_ok_is_handled);
  RUN(used_save_budget_blocks_writes);
  RUN(forwarding_entries_are_flagged);
  RUN(field_403_layout_is_matched_by_register_name);
  RUN(field_403_layout_writes_flow_control_to_its_named_register);
  RUN(field_403_lbt_rssi_drift_is_reported_at_its_named_register);
  RUN(plan_policies_follow_rollout_and_flow_control);
  RUN(flight_gate_runs_only_on_a_safe_network_flight_state);
  RUN(status_words_pack_the_report);
  return 0;
}
