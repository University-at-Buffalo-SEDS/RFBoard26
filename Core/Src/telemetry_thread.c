// telemetry_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "radio.h"
#include "main.h"
#include <stdio.h>

TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE (16U *1024U)
#define TELEMETRY_THREAD_SLEEP_TICKS 5U
#define TELEMETRY_QUEUE_BUDGET_MS 50U
#define RADIO_WINDOW_CONTROL_KIND 0x01U
#define RADIO_WINDOW_DOWNLINK_OPEN 0U
#define RADIO_WINDOW_UPLINK_OPEN 1U
#define RADIO_DOWNLINK_WINDOW_MS 300U
#define RADIO_UPLINK_WINDOW_MS 75U
#define GPS_NO_FIX_SATELLITE_INTERVAL_MS 1000ULL
#define TELEMETRY_DISCOVERY_ANNOUNCE_INTERVAL_MS 2000ULL
#define TELEMETRY_THREAD_PRIORITY 3U

static uint64_t radio_window_now_ms(void)
{
    return ((uint64_t)tx_time_get() * 1000ULL) /
           (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static void telemetry_emit_radio_window(uint8_t uplink_open, uint16_t duration_ms)
{
    uint8_t payload[4];
    payload[0] = RADIO_WINDOW_CONTROL_KIND;
    payload[1] = uplink_open ? RADIO_WINDOW_UPLINK_OPEN : RADIO_WINDOW_DOWNLINK_OPEN;
    payload[2] = (uint8_t)(duration_ms & 0xFFU);
    payload[3] = (uint8_t)((duration_ms >> 8U) & 0xFFU);
    (void)radio_uart_send_command_frame(payload, sizeof(payload));
}

static void telemetry_emit_no_gps_satellite_count(uint64_t now_ms,
                                                  uint64_t *next_emit_ms)
{
    if (g_neom9n_has_fix != 0U || now_ms < *next_emit_ms || !radio_uart_tx_ready()) {
        return;
    }

    const uint8_t satellite_count = 0U;
    SedsResult result = log_telemetry_asynchronous(SEDS_DT_GPS_SATELLITE_NUMBER,
                                                   &satellite_count,
                                                   1,
                                                   sizeof(satellite_count));
    // printf("RF telemetry queued GPS satellites=%u result=%ld\r\n",
        //    (unsigned)satellite_count,
        //    (long)result);
    *next_emit_ms = now_ms + GPS_NO_FIX_SATELLITE_INTERVAL_MS;
}

static void telemetry_announce_discovery_if_due(uint64_t now_ms,
                                                uint64_t *next_emit_ms)
{
    if (now_ms < *next_emit_ms || !radio_uart_tx_ready()) {
        return;
    }

    (void)telemetry_announce_discovery();
    *next_emit_ms = now_ms + TELEMETRY_DISCOVERY_ANNOUNCE_INTERVAL_MS;
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    uint8_t radio_window_started = 0U;
    uint8_t radio_uplink_open = 0U;
    uint64_t radio_window_deadline_ms = 0U;
    uint64_t next_no_gps_satellite_emit_ms = 0U;
    uint64_t next_discovery_announce_ms = 0U;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();
    for (;;) {
        const uint64_t now_ms = radio_window_now_ms();

        /* Poll hardware FIFO and then process reassembly + router queues. */
        radio_uart_process_rx();
        if (radio_uart_tx_ready()) {
            if (!radio_window_started) {
                radio_window_started = 1U;
                radio_uplink_open = 0U;
                radio_window_deadline_ms = now_ms + RADIO_DOWNLINK_WINDOW_MS;
                telemetry_emit_radio_window(0U, RADIO_DOWNLINK_WINDOW_MS);
            } else if (now_ms >= radio_window_deadline_ms) {
                radio_uplink_open = !radio_uplink_open;
                radio_window_deadline_ms =
                    now_ms + (radio_uplink_open ? RADIO_UPLINK_WINDOW_MS : RADIO_DOWNLINK_WINDOW_MS);
                telemetry_emit_radio_window(
                    radio_uplink_open,
                    radio_uplink_open ? RADIO_UPLINK_WINDOW_MS : RADIO_DOWNLINK_WINDOW_MS);
            }
        }
        if (!radio_uplink_open) {
            radio_uart_process_tx();
        }
        can_bus_process_rx();
        telemetry_announce_discovery_if_due(now_ms, &next_discovery_announce_ms);
        telemetry_emit_no_gps_satellite_count(now_ms, &next_no_gps_satellite_emit_ms);
        (void)telemetry_poll_discovery();
        (void)process_all_queues_timeout(TELEMETRY_QUEUE_BUDGET_MS);
        (void)telemetry_poll_timesync();
        if (!radio_uplink_open) {
            radio_uart_process_tx();
        }

        tx_thread_sleep(TELEMETRY_THREAD_SLEEP_TICKS);

        // HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    }
}

UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool)
{
    if (radio_uart_init_tx_queue(byte_pool) != TX_SUCCESS)
    {
      return TX_POOL_ERROR;
    }

        CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       TELEMETRY_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,
                                   pointer,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   TELEMETRY_THREAD_PRIORITY,
                                   TELEMETRY_THREAD_PRIORITY,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
