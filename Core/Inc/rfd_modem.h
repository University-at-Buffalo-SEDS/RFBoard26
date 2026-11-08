#ifndef RFD_MODEM_H
#define RFD_MODEM_H
/* Startup check of this board's RFD900x ASYNC modem against the selected RFD
 * profile. Same algorithm and constants as GroundStation26
 * backend/src/comms/rfd_modem.rs; see GroundStation26 docs/backend/rfd-modem-check.md.
 *
 * This file is hardware independent: the board supplies byte I/O, a clock and
 * a gate. Registers are resolved by the name the modem reports in ATI5 (RFD
 * ASYNC 4.03 on RFD900x2 moves LBT_RSSI to S18 and RTSCTS to S19); the
 * documented number is only a fallback for reading, and writes go to the
 * modem's own register number. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RFD_UNREAD INT32_MIN

typedef enum { RFD_MODE_SKIP = 0, RFD_MODE_VERIFY = 1, RFD_MODE_FIX = 2 } rfd_mode_t;
typedef enum { RFD_POLICY_FIX = 0, RFD_POLICY_REPORT = 1, RFD_POLICY_INFO = 2 } rfd_policy_t;
/* Wire values (BOARD_HEALTH status block); same order as the GS outcomes. */
typedef enum {
  RFD_OUT_SKIPPED = 0, RFD_OUT_MATCHED = 1, RFD_OUT_APPLIED = 2, RFD_OUT_MISMATCH = 3,
  RFD_OUT_APPLY_FAILED = 4, RFD_OUT_VERIFY_FAILED = 5, RFD_OUT_NO_RESPONSE = 6,
  RFD_OUT_FAILED = 7
} rfd_outcome_t;
typedef enum { RFD_MAP_ATI5 = 0, RFD_MAP_FINGERPRINT = 1, RFD_MAP_UNCONFIRMED = 2 } rfd_map_t;
typedef enum {
  RFD_DATA_UNTOUCHED = 0, RFD_DATA_RESUMED = 1, RFD_DATA_REBOOTED = 2, RFD_DATA_UNKNOWN = 3
} rfd_data_mode_t;
typedef enum {
  RFD_ST_INFO = 0, RFD_ST_MATCH, RFD_ST_DIFFERS, RFD_ST_UNREAD, RFD_ST_NAME_MISMATCH
} rfd_status_t;

#define RFD_FLAG_SAVED        0x01U /* AT&W acknowledged */
#define RFD_FLAG_REBOOTED     0x02U /* ATZ sent */
#define RFD_FLAG_RESUMED      0x04U /* data mode resumed with ATO */
#define RFD_FLAG_FORWARDING   0x08U /* AT+F? lists forwarding entries */
#define RFD_FLAG_GATE_SKIPPED 0x10U /* flight gate skipped or stopped it */
#define RFD_FLAG_PENDING      0x20U /* check not finished yet */
#define RFD_FLAG_ATTENTION    0x40U /* a warning the operator should see */

typedef struct {
  uint32_t serial_baud, air_bps, max_data_bytes, max_retries, broadcast_retries;
  uint32_t serial_break_ms, tx_encapsulation, rx_encapsulation;
  bool hardware_flow_control;
  /* Profile set on GS, RRLS and RF: only then are link-wide registers written. */
  bool rolled_out_all_nodes;
} rfd_profile_values_t;

typedef struct {
  const char *names;        /* accepted ATI5 names, '|'-separated */
  int32_t expected;         /* RFD_UNREAD: no expectation */
  uint8_t number;           /* documented V1.1 number, fallback only */
  uint8_t policy;
} rfd_expect_t;

#define RFD_PLAN_MAX 12U
typedef struct {
  rfd_expect_t regs[RFD_PLAN_MAX];
  uint8_t count;
} rfd_plan_t;

typedef struct {
  uint32_t guard, plus_reply, command, idle, save, reboot, overall;
} rfd_timing_t;
extern const rfd_timing_t RFD_TIMING;

#define RFD_MAX_WRITES 8U
typedef struct {
  uint8_t outcome, map_source, mode, data_mode, flags;
  uint8_t write_count;
  bool save_attempted;
  uint8_t write_number[RFD_MAX_WRITES];
  /* Observed by name: S2 AIR_SPEED, S3, S4, S5, S6 serial break, S11, RTSCTS. */
  int32_t observed[7];
  uint8_t rtscts_number; /* register the modem names RTSCTS (0: unknown) */
} rfd_report_t;

typedef struct {
  /* Returns bytes accepted (0 = none yet) or < 0 on error. */
  int (*write)(void *ctx, const uint8_t *bytes, size_t len);
  /* Returns bytes read, 0 after a short poll with nothing received, < 0 error. */
  int (*read)(void *ctx, uint8_t *buf, size_t cap);
  uint32_t (*now_ms)(void *ctx);
  void (*sleep_ms)(void *ctx, uint32_t ms);
  void *ctx;
} rfd_io_t;

/* True while reconfiguration is allowed (safe flight state). */
typedef bool (*rfd_gate_fn)(void *ctx);

void rfd_plan_init(rfd_plan_t *plan, const rfd_profile_values_t *p);
void rfd_report_skipped(rfd_report_t *r, rfd_mode_t mode, uint8_t flags);
/* One bounded check. The gate is consulted before command mode, before the
 * first write and before AT&W. allow_save is false once AT&W was used. */
void rfd_modem_run(const rfd_io_t *io, const rfd_plan_t *plan, rfd_mode_t mode,
                   const rfd_timing_t *timing, rfd_gate_fn gate, void *gate_ctx,
                   bool allow_save, rfd_report_t *out);

/* Flight gate. Startup, Idle, PreFill and FillTest only. The first
 * validated FLIGHT_STATE received after boot decides; a restored (persisted)
 * flight phase skips at once. Until a value arrives the check waits. */
typedef enum { RFD_GATE_WAIT = 0, RFD_GATE_RUN = 1, RFD_GATE_SKIP = 2 } rfd_gate_t;
bool rfd_flight_state_safe(uint8_t state);
rfd_gate_t rfd_gate_decide(bool restored_known, uint8_t restored_state, bool network_seen,
                           uint8_t network_state);

/* BOARD_HEALTH words 13, 15 and 22 on RF (flags 4/32/64 stay clear):
 * w13: [3:0] version 1, [7:4] outcome, [9:8] map source, [11:10] mode,
 *      [12] rolled out, [14:13] RTSCTS (3 unread), [23:16] flags, [31:24] S5;
 * w15: S3 | S6 << 16;  w22: S11 | S2 << 16 | S4 << 24.
 * u16 fields 0xFFFF and u8 fields 0xFF mean unread; 0xFE is "254 or more". */
void rfd_status_words(const rfd_report_t *r, bool rolled_out, uint32_t words[3]);

#ifdef __cplusplus
}
#endif
#endif
