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
#define TELEMETRY_THREAD_SLEEP_TICKS 1U
#define TELEMETRY_QUEUE_BUDGET_MS 5U
#define TELEMETRY_ALIVE_PRINT_INTERVAL_MS 5000ULL
#define RADIO_WINDOW_CONTROL_KIND 0x01U
#define RADIO_WINDOW_DOWNLINK_OPEN 0U
#define RADIO_WINDOW_UPLINK_OPEN 1U
#define RADIO_DOWNLINK_WINDOW_MS 500U
#define RADIO_UPLINK_WINDOW_MS 500U
#define RADIO_TX_GUARD_MS 75U
#define RADIO_UPLINK_TURNAROUND_MS 250U
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

static uint16_t telemetry_advertised_window_duration(uint8_t uplink_open, uint16_t duration_ms)
{
    (void)uplink_open;
    return duration_ms;
}

static void telemetry_process_radio_tx_if_allowed(uint8_t uplink_open,
                                                  uint64_t now_ms,
                                                  uint64_t window_deadline_ms)
{
    if (uplink_open || now_ms >= window_deadline_ms) {
        return;
    }

    const uint64_t remaining_ms = window_deadline_ms - now_ms;
    if (remaining_ms <= RADIO_TX_GUARD_MS) {
        return;
    }

    radio_uart_process_tx_with_budget((uint32_t)(remaining_ms - RADIO_TX_GUARD_MS));
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

static void telemetry_print_alive_if_due(uint64_t now_ms, uint64_t *next_print_ms)
{
    if (now_ms < *next_print_ms) {
        return;
    }
    const radio_uart_stats_t radio_stats = radio_uart_stats_snapshot();
    const char *frame_kind = "none";
    if (radio_stats.last_frame_kind == 1U) {
        frame_kind = "data";
    } else if (radio_stats.last_frame_kind == 2U) {
        frame_kind = "command";
    } else if (radio_stats.last_frame_kind == 3U) {
        frame_kind = "ascii";
    }

    printf("RFBoard telemetry alive: uptime_ms=%lu radio_tx_ready=%lu radio_tx_busy=%lu "
           "rx_irq=%lu rx_chunks=%lu rx_bytes=%lu rx_frames=%lu rx_sync_loss=%lu "
           "rx_bad_len=%lu rx_errors=%lu rx_restarts=%lu tx_ok=%lu tx_errors=%lu "
           "tx_busy_count=%lu tx_enqueued=%lu tx_queue=%lu tx_budget_misses=%lu "
           "aux=%lu mode0_pins=%lu aux_busy=%lu tx_dma_started=%lu tx_dma_complete=%lu "
           "tx_startup_drops=%lu last_rx_len=%lu last_rx_preview=",
           (unsigned long)(uint32_t)now_ms,
           (unsigned long)radio_uart_tx_ready(),
           (unsigned long)radio_uart_tx_busy(),
           (unsigned long)radio_stats.rx_irq_events,
           (unsigned long)radio_stats.rx_isr_chunks,
           (unsigned long)radio_stats.rx_isr_bytes,
           (unsigned long)radio_stats.rx_frames_ok,
           (unsigned long)radio_stats.rx_sync_loss,
           (unsigned long)radio_stats.rx_bad_len,
           (unsigned long)radio_stats.rx_errors,
           (unsigned long)radio_stats.rx_restart_errors,
           (unsigned long)radio_stats.tx_ok,
           (unsigned long)radio_stats.tx_errors,
           (unsigned long)radio_stats.tx_busy,
           (unsigned long)radio_stats.tx_enqueued,
           (unsigned long)radio_stats.tx_queue_count,
           (unsigned long)radio_stats.tx_budget_misses,
           (unsigned long)radio_stats.aux_high,
           (unsigned long)radio_stats.e22_mode0_pins_configured,
           (unsigned long)radio_stats.aux_busy_count,
           (unsigned long)radio_stats.tx_dma_started,
           (unsigned long)radio_stats.tx_dma_complete,
           (unsigned long)radio_stats.tx_startup_drops,
           (unsigned long)radio_stats.last_rx_len);
    for (uint8_t i = 0U; i < radio_stats.last_rx_preview_len; i++) {
        printf("%02X%s",
               radio_stats.last_rx_preview[i],
               (i + 1U < radio_stats.last_rx_preview_len) ? " " : "");
    }
    printf(" last_frame_kind=%s last_frame_payload_len=%lu last_frame_preview=",
           frame_kind,
           (unsigned long)radio_stats.last_frame_payload_len);
    for (uint8_t i = 0U; i < radio_stats.last_frame_preview_len; i++) {
        printf("%02X%s",
               radio_stats.last_frame_preview[i],
               (i + 1U < radio_stats.last_frame_preview_len) ? " " : "");
    }
    printf(" usart_isr=0x%08lX usart_cr1=0x%08lX usart_cr3=0x%08lX dma_rx_remaining=%lu tx_quiet_remaining_ms=%lu "
           "pa9_tx=%lu pa10_rx=%lu pa10_high_samples=%lu pa10_low_samples=%lu pa10_edges=%lu\r\n",
           (unsigned long)radio_stats.usart_isr,
           (unsigned long)radio_stats.usart_cr1,
           (unsigned long)radio_stats.usart_cr3,
           (unsigned long)radio_stats.dma_rx_remaining,
           (unsigned long)radio_stats.tx_quiet_remaining_ms,
           (unsigned long)radio_stats.tx_pin_high,
           (unsigned long)radio_stats.rx_pin_high,
           (unsigned long)radio_stats.rx_pin_samples_high,
           (unsigned long)radio_stats.rx_pin_samples_low,
           (unsigned long)radio_stats.rx_pin_edges);
    *next_print_ms = now_ms + TELEMETRY_ALIVE_PRINT_INTERVAL_MS;
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    uint8_t radio_window_started = 0U;
    uint8_t radio_uplink_open = 0U;
    uint64_t radio_window_deadline_ms = 0U;
    uint64_t next_discovery_announce_ms = 0U;
    uint64_t next_alive_print_ms = 0U;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();
    for (;;) {
        const uint64_t now_ms = radio_window_now_ms();
        telemetry_print_alive_if_due(now_ms, &next_alive_print_ms);

        /* Poll hardware FIFO and then process reassembly + router queues. */
        radio_uart_process_rx();
        if (radio_uart_tx_ready()) {
            if (!radio_window_started) {
                radio_window_started = 1U;
                radio_uplink_open = 0U;
                radio_window_deadline_ms = now_ms + RADIO_DOWNLINK_WINDOW_MS;
                telemetry_emit_radio_window(0U, RADIO_DOWNLINK_WINDOW_MS);
            } else if (now_ms >= radio_window_deadline_ms && !radio_uart_tx_busy()) {
                radio_uplink_open = !radio_uplink_open;
                radio_window_deadline_ms =
                    now_ms + (radio_uplink_open ? RADIO_UPLINK_WINDOW_MS : RADIO_DOWNLINK_WINDOW_MS);
                telemetry_emit_radio_window(
                    radio_uplink_open,
                    telemetry_advertised_window_duration(
                        radio_uplink_open,
                        radio_uplink_open ? RADIO_UPLINK_WINDOW_MS : RADIO_DOWNLINK_WINDOW_MS));
            }
        }
        telemetry_process_radio_tx_if_allowed(radio_uplink_open, now_ms, radio_window_deadline_ms);
        can_bus_process_rx();
        telemetry_announce_discovery_if_due(now_ms, &next_discovery_announce_ms);
        (void)telemetry_poll_discovery();
        (void)process_all_queues_timeout(TELEMETRY_QUEUE_BUDGET_MS);
        (void)telemetry_poll_timesync();
        telemetry_process_radio_tx_if_allowed(radio_uplink_open, now_ms, radio_window_deadline_ms);

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
