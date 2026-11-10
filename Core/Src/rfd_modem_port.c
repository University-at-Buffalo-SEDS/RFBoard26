/* Board side of the startup RFD modem check (spec: GroundStation26
 * docs/backend/rfd-modem-check.md, phase 2). It runs at most once per MCU
 * boot, and only at boot:
 *
 *  - rfd_modem_port_boot runs it before the telemetry thread starts CAN and
 *    the SEDSnet router, so no station traffic can be missed and no receipt
 *    stranded while it holds the radio (no frame TX, RX ignored). Run from
 *    the first applied station FLIGHT_STATE instead, it once started right
 *    after an operator Abort (G-hitl: Abort 40.25 s, command mode 41.33 s,
 *    data again only at 48.51 s);
 *  - it runs only from a restored Startup or Idle, or no restored phase. A
 *    restored phase past Idle (a reboot during a test or in flight) skips it
 *    for this boot;
 *  - a check that matched, or applied and verified, this profile is
 *    remembered in flash against this firmware image; later boots of the
 *    same image skip it. A skipping boot that hears no radio frame within
 *    RFD_LINK_PROBATION_MS forgets it, so the NEXT boot checks a swapped or
 *    reset modem; nothing ever holds the radio after boot;
 *  - the hold lasts about 3 s to read, 10 to 14 s if it writes, never past
 *    RFD_TIMING.overall (16 s, reboots and verify included). The watchdog
 *    keeps being served.
 *
 * Result: BOARD_HEALTH words 13, 15 and 22 (rfd_status_words). */
#include "rfd_modem.h"
#include "rfd_modem_port.h"

#include "flight_state_cache.h"
#include "board_config.h"
#include "launchcore/image.h"
#include "persistent_store.h"
#include "radio.h"
#include "tx_api.h"

#include <stddef.h>

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

extern volatile uint32_t g_radio_rx_frames_ok;

/* A skipping boot with no radio frame by then forgets the marker. */
#ifndef RFD_LINK_PROBATION_MS
#define RFD_LINK_PROBATION_MS 60000U
#endif

/* "RFDV": this profile matched, or was applied and verified, on a boot. */
#define RFD_VERIFIED_KEY 0x56444652u
#define RFD_MIX(h, v) ((((uint32_t)(h)) ^ (uint32_t)(v)) * 16777619u)
#define RFD_PROFILE_SIGNATURE                                                   \
  RFD_MIX(RFD_MIX(RFD_MIX(RFD_MIX(RFD_MIX(RFD_MIX(RFD_MIX(RFD_MIX(RFD_MIX(       \
      2166136261u, RFD_PROFILE_SERIAL_BAUD), RFD_PROFILE_AIR_BPS),              \
      RFD_PROFILE_MAX_DATA_BYTES), RFD_PROFILE_MAX_RETRIES),                    \
      RFD_PROFILE_BROADCAST_RETRIES), RFD_PROFILE_SERIAL_BREAK_MS),             \
      RFD_PROFILE_TX_ENCAPSULATION), RFD_PROFILE_RX_ENCAPSULATION),             \
      RFD_PROFILE_ROLLED_OUT)

/* The running image's CRC from its LaunchCore header: a new image re-checks. */
__attribute__((weak)) uint32_t rfd_modem_port_image_id(void)
{
  return *(const volatile uint32_t *)(BOARD_SLOT_A_BASE +
                                      offsetof(launchcore_image_header_t, image_crc32));
}

static uint32_t verified_marker(void)
{
  return RFD_MIX(RFD_PROFILE_SIGNATURE, rfd_modem_port_image_id());
}

/* Read by the simulator from the ELF; also the BOARD_HEALTH block. */
volatile uint32_t g_rfd_modem_status[3] __attribute__((used, externally_visible));
/* Radio hold during the check, in ms. */
volatile uint32_t g_rfd_modem_hold_ms __attribute__((used, externally_visible));

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

/* Set when a boot skipped the check on its verified marker: the link must
 * then prove the modem within RFD_LINK_PROBATION_MS. */
static uint8_t g_probation;
static uint32_t g_boot_ms;

static bool gate_open(void *ctx)
{
  (void)ctx;
  return !flight_state_cache_has_value() || rfd_flight_state_safe(flight_state_cache_value());
}

void rfd_modem_port_boot(void (*service)(void))
{
  rfd_report_t report;
  const rfd_mode_t mode = (rfd_mode_t)RFD_MODEM_CHECK_MODE;
  uint32_t marker = 0U;
  size_t size = sizeof(marker);
  g_boot_ms = now_ms(NULL);
  rfd_report_skipped(&report, mode, 0U);
  if (mode == RFD_MODE_SKIP || RFBOARD_LORA_SCHEDULER || service == NULL) {
    publish(&report);
    return;
  }
  if (persistent_store_get(RFD_VERIFIED_KEY, &marker, &size) == LAUNCHCORE_PERSIST_OK &&
      size == sizeof(marker) && marker == verified_marker()) {
    report.flags = RFD_FLAG_VERIFIED;
    publish(&report);
    g_probation = 1U;
    return;
  }
  /* The restored phase is the only one known before the network starts. */
  if (!gate_open(NULL) || !radio_uart_set_modem_hold(1U)) {
    report.flags = RFD_FLAG_GATE_SKIPPED;
    publish(&report);
    return;
  }
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
  const uint32_t hold_start = now_ms(NULL);
  rfd_modem_run(&io, &plan, mode, &RFD_TIMING, gate_open, NULL, true, &report);
  (void)radio_uart_set_modem_hold(0U);
  g_rfd_modem_hold_ms = now_ms(NULL) - hold_start;
  if ((report.outcome == RFD_OUT_MATCHED || report.outcome == RFD_OUT_APPLIED) &&
      !(report.flags & (RFD_FLAG_ATTENTION | RFD_FLAG_GATE_SKIPPED))) {
    marker = verified_marker();
    (void)persistent_store_set(RFD_VERIFIED_KEY, &marker, sizeof(marker));
  }
  publish(&report);
}

void rfd_modem_port_poll(void (*service)(void))
{
  (void)service;
  if (!g_probation) return;
  if (g_radio_rx_frames_ok != 0U) {
    g_probation = 0U;
    return;
  }
  if (now_ms(NULL) - g_boot_ms < RFD_LINK_PROBATION_MS) return;
  /* No link: the modem may be swapped or reset. The next boot checks it. */
  const uint32_t marker = 0U;
  (void)persistent_store_set(RFD_VERIFIED_KEY, &marker, sizeof(marker));
  g_probation = 0U;
}

void rfd_modem_port_status_words(uint32_t words[3])
{
  words[0] = g_rfd_modem_status[0];
  words[1] = g_rfd_modem_status[1];
  words[2] = g_rfd_modem_status[2];
}
