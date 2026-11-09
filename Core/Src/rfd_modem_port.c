/* Board side of the startup RFD modem check (spec: GroundStation26
 * docs/backend/rfd-modem-check.md, phase 2). Polled once per telemetry loop
 * pass; the check runs at most once per MCU boot:
 *
 *  - a restored (persisted) flight phase, for example after a brown-out
 *    reboot in flight, skips at once;
 *  - otherwise the radio runs normally until the first validated
 *    FLIGHT_STATE arrives. FLIGHT_STATE is written only by the ground
 *    station, so in practice it arrives over the radio itself. Startup, Idle,
 *    PreFill or FillTest runs the check; any other state skips it for this
 *    boot;
 *  - only the check itself holds the radio (no frame TX, RX ignored): about
 *    3 s to read, about 11 s if it writes, bounded at 40 s. CAN, routing and
 *    the watchdog keep running, and a FLIGHT_STATE change over CAN closes the
 *    gate before the first write and before AT&W.
 *
 * Result: BOARD_HEALTH words 13, 15 and 22 (rfd_status_words). */
#include "rfd_modem.h"
#include "rfd_modem_port.h"

#include "flight_state_cache.h"
#include "radio.h"
#include "tx_api.h"

#ifndef RFD_MODEM_CHECK_MODE
#define RFD_MODEM_CHECK_MODE 0 /* skip unless the build selects verify or fix */
#endif
#ifndef RFD_PROFILE_ROLLED_OUT
#define RFD_PROFILE_ROLLED_OUT 0
#endif
#ifndef RFBOARD_LORA_SCHEDULER
#define RFBOARD_LORA_SCHEDULER 0
#endif
/* Selected profile modem section (config/rfd-profile.json via CMake). */
#ifndef RFD_PROFILE_MAX_DATA_BYTES
#define RFD_PROFILE_SERIAL_BAUD 115200U
#define RFD_PROFILE_AIR_BPS 64000U
#define RFD_PROFILE_MAX_DATA_BYTES 3968U
#define RFD_PROFILE_MAX_RETRIES 10U
#define RFD_PROFILE_BROADCAST_RETRIES 1U
#define RFD_PROFILE_SERIAL_BREAK_MS 50U
#define RFD_PROFILE_TX_ENCAPSULATION 0U
#define RFD_PROFILE_RX_ENCAPSULATION 0U
#endif
#if defined(RADIO_PACKET_GAP_MS) && (RADIO_PACKET_GAP_MS < RFD_PROFILE_SERIAL_BREAK_MS + 5U)
#error "RADIO_PACKET_GAP_MS must be at least the modem serial break + 5 ms"
#endif

extern volatile uint32_t g_flight_state_cache_updates;

/* Read by the simulator from the ELF; also the BOARD_HEALTH block. */
volatile uint32_t g_rfd_modem_status[3] __attribute__((used, externally_visible));
/* Radio hold during the check, in ms. */
volatile uint32_t g_rfd_modem_hold_ms __attribute__((used, externally_visible));
static uint8_t g_rfd_saved_this_boot;

static uint32_t now_ms(void *ctx)
{
  (void)ctx;
  return (uint32_t)(((uint64_t)tx_time_get() * 1000ULL) / TX_TIMER_TICKS_PER_SECOND);
}

static void publish(const rfd_report_t *r)
{
  uint32_t w[3];
  rfd_status_words(r, RFD_PROFILE_ROLLED_OUT != 0, w);
  g_rfd_modem_status[0] = w[0];
  g_rfd_modem_status[1] = w[1];
  g_rfd_modem_status[2] = w[2];
}

static void (*g_service)(void);

static int io_write(void *ctx, const uint8_t *bytes, size_t len)
{
  (void)ctx;
  return radio_uart_modem_write(bytes, len);
}

static int io_read(void *ctx, uint8_t *buf, size_t cap)
{
  (void)ctx;
  const int n = radio_uart_modem_read(buf, cap);
  if (n == 0) g_service(); /* one service pass, then about 1 ms of sleep */
  return n;
}

static void io_sleep(void *ctx, uint32_t ms)
{
  const uint32_t start = now_ms(ctx);
  while (now_ms(ctx) - start < ms) g_service();
}

static bool gate_open(void *ctx)
{
  (void)ctx;
  return flight_state_cache_has_value() && rfd_flight_state_safe(flight_state_cache_value());
}

enum { PORT_INIT = 0, PORT_WAITING, PORT_DONE };
static uint8_t g_port_state;
static bool g_restored_known;
static uint8_t g_restored;
static uint32_t g_updates_at_boot;

static void finish(const rfd_report_t *report)
{
  publish(report);
  g_port_state = PORT_DONE;
}

void rfd_modem_port_poll(void (*service)(void))
{
  rfd_report_t report;
  const rfd_mode_t mode = (rfd_mode_t)RFD_MODEM_CHECK_MODE;
  if (g_port_state == PORT_DONE) return;
  if (g_port_state == PORT_INIT) {
    rfd_report_skipped(&report, mode, 0U);
    if (mode == RFD_MODE_SKIP || RFBOARD_LORA_SCHEDULER || service == NULL) {
      finish(&report);
      return;
    }
    /* Taken before any network FLIGHT_STATE can be applied. */
    g_restored_known = flight_state_cache_has_value();
    g_restored = flight_state_cache_value();
    g_updates_at_boot = g_flight_state_cache_updates;
    report.flags = RFD_FLAG_PENDING;
    publish(&report);
    g_port_state = PORT_WAITING;
  }
  const rfd_gate_t gate =
      rfd_gate_decide(g_restored_known, g_restored, g_flight_state_cache_updates != g_updates_at_boot,
                      flight_state_cache_value());
  if (gate == RFD_GATE_WAIT) return;
  rfd_report_skipped(&report, mode, RFD_FLAG_GATE_SKIPPED);
  if (gate == RFD_GATE_SKIP) {
    finish(&report);
    return;
  }
  /* A TX DMA in flight: take the UART on a later pass. */
  if (!radio_uart_set_modem_hold(1U)) return;
  const uint32_t hold_start = now_ms(NULL);
  g_service = service;
  rfd_plan_t plan;
  const rfd_profile_values_t values = {
      .serial_baud = RFD_PROFILE_SERIAL_BAUD,
      .air_bps = RFD_PROFILE_AIR_BPS,
      .max_data_bytes = RFD_PROFILE_MAX_DATA_BYTES,
      .max_retries = RFD_PROFILE_MAX_RETRIES,
      .broadcast_retries = RFD_PROFILE_BROADCAST_RETRIES,
      .serial_break_ms = RFD_PROFILE_SERIAL_BREAK_MS,
      .tx_encapsulation = RFD_PROFILE_TX_ENCAPSULATION,
      .rx_encapsulation = RFD_PROFILE_RX_ENCAPSULATION,
      .hardware_flow_control = false, /* USART1 runs without RTS/CTS */
      .rolled_out_all_nodes = RFD_PROFILE_ROLLED_OUT != 0,
  };
  const rfd_io_t io = {io_write, io_read, now_ms, io_sleep, NULL};
  rfd_plan_init(&plan, &values);
  rfd_modem_run(&io, &plan, mode, &RFD_TIMING, gate_open, NULL, !g_rfd_saved_this_boot, &report);
  if (report.save_attempted) g_rfd_saved_this_boot = 1U;
  (void)radio_uart_set_modem_hold(0U);
  g_rfd_modem_hold_ms = now_ms(NULL) - hold_start;
  finish(&report);
}

void rfd_modem_port_status_words(uint32_t words[3])
{
  words[0] = g_rfd_modem_status[0];
  words[1] = g_rfd_modem_status[1];
  words[2] = g_rfd_modem_status[2];
}
