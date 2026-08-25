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
#define RADIO_SCHED_MAGIC_0 0x52U
#define RADIO_SCHED_MAGIC_1 0x53U
#define RADIO_SCHED_VERSION 1U
#define RADIO_SCHED_DOWNLINK 0U
#define RADIO_SCHED_UPLINK 1U
#define RADIO_SCHED_FLAG_HAS_MORE 0x01U
#define RADIO_SCHED_FLAG_YIELD 0x02U
#define RADIO_SCHED_BURST_MESSAGES 5U
#define RADIO_SCHED_DOWNLINK_TIMEOUT_MS 300ULL
#define RADIO_SCHED_MIN_DOWNLINK_BUDGET_MS 1U
#define RADIO_SCHED_UPLINK_TIMEOUT_MS 4000ULL
#define RADIO_SCHED_IDLE_UPLINK_POLL_MS 50ULL
#define RADIO_SCHED_GRANT_RETRY_MS 50ULL
#define RADIO_SCHED_GRANT_REANNOUNCE_MS 5000ULL
#define RADIO_SCHED_TURNAROUND_MS 75ULL
#define RADIO_SCHED_UPLINK_TO_DOWNLINK_TURNAROUND_MS 150ULL
#define RADIO_SCHED_GS_TX_TURNAROUND_MS 600U
#define TELEMETRY_DISCOVERY_ANNOUNCE_INTERVAL_MS 2000ULL
#define TELEMETRY_THREAD_PRIORITY 3U

#ifndef RADIO_SCHEDULER_ENABLED
#define RADIO_SCHEDULER_ENABLED 0
#endif

#ifndef TELEMETRY_ALIVE_PRINTS
#define TELEMETRY_ALIVE_PRINTS 0
#endif

static volatile uint32_t g_radio_downlink_windows = 0U;
static volatile uint32_t g_radio_uplink_windows = 0U;
static volatile uint32_t g_radio_downlink_tx_calls = 0U;
static volatile uint32_t g_radio_uplink_rx_only_loops = 0U;
static volatile uint32_t g_radio_sched_grant_failures = 0U;
static volatile uint8_t g_radio_sched_gs_seq = 0U;
static volatile uint8_t g_radio_sched_gs_flags = 0U;
static volatile uint8_t g_radio_sched_gs_seen = 0U;

static uint64_t radio_window_now_ms(void)
{
    return ((uint64_t)tx_time_get() * 1000ULL) /
           (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

uint8_t telemetry_radio_scheduler_handle_command(const uint8_t *data, size_t len)
{
#if RADIO_SCHEDULER_ENABLED
    if (data == NULL || len < 7U ||
        data[0] != RADIO_SCHED_MAGIC_0 ||
        data[1] != RADIO_SCHED_MAGIC_1 ||
        data[2] != RADIO_SCHED_VERSION) {
        return 0U;
    }

    g_radio_sched_gs_seq = data[4];
    g_radio_sched_gs_flags = data[6];
    g_radio_sched_gs_seen = 1U;
#if COMMAND_TRACE_PRINTS
    printf("Radio scheduler status RX: seq=%u flags=0x%02x\r\n",
           (unsigned)g_radio_sched_gs_seq,
           (unsigned)g_radio_sched_gs_flags);
#endif
    return 1U;
#else
    (void)data;
    (void)len;
    return 0U;
#endif
}

#if RADIO_SCHEDULER_ENABLED
static HAL_StatusTypeDef telemetry_emit_radio_grant(uint8_t turn, uint8_t seq)
{
    uint8_t payload[9];
    HAL_StatusTypeDef status;
    payload[0] = RADIO_SCHED_MAGIC_0;
    payload[1] = RADIO_SCHED_MAGIC_1;
    payload[2] = RADIO_SCHED_VERSION;
    payload[3] = turn;
    payload[4] = seq;
    payload[5] = RADIO_SCHED_BURST_MESSAGES;
    payload[6] = (radio_uart_tx_queue_count() > 0U) ? RADIO_SCHED_FLAG_HAS_MORE : 0U;
    payload[7] = (uint8_t)(RADIO_SCHED_GS_TX_TURNAROUND_MS & 0xFFU);
    payload[8] = (uint8_t)((RADIO_SCHED_GS_TX_TURNAROUND_MS >> 8U) & 0xFFU);
    status = radio_uart_send_command_frame(payload, sizeof(payload));
#if COMMAND_TRACE_PRINTS
    printf("Radio scheduler grant %s seq=%u credit=%u status=%ld\r\n",
           (turn == RADIO_SCHED_UPLINK) ? "uplink" : "downlink",
           (unsigned)seq,
           (unsigned)RADIO_SCHED_BURST_MESSAGES,
           (long)status);
#endif
    return status;
}
#endif

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
#if TELEMETRY_ALIVE_PRINTS
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
           "tx_drops=%lu tx_drop_oldest=%lu tx_drop_same_flow=%lu tx_drop_stale=%lu "
           "aux=%lu mode0_pins=%lu aux_busy=%lu tx_dma_started=%lu tx_dma_complete=%lu "
           "tx_startup_drops=%lu win_down=%lu win_up=%lu down_tx_calls=%lu up_rx_loops=%lu sched_grant_fail=%lu "
           "last_rx_len=%lu last_rx_preview=",
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
           (unsigned long)radio_stats.tx_drops,
           (unsigned long)radio_stats.tx_drop_oldest,
           (unsigned long)radio_stats.tx_drop_same_flow,
           (unsigned long)radio_stats.tx_drop_stale,
           (unsigned long)radio_stats.aux_high,
           (unsigned long)radio_stats.e22_mode0_pins_configured,
           (unsigned long)radio_stats.aux_busy_count,
           (unsigned long)radio_stats.tx_dma_started,
           (unsigned long)radio_stats.tx_dma_complete,
           (unsigned long)radio_stats.tx_startup_drops,
           (unsigned long)g_radio_downlink_windows,
           (unsigned long)g_radio_uplink_windows,
           (unsigned long)g_radio_downlink_tx_calls,
           (unsigned long)g_radio_uplink_rx_only_loops,
           (unsigned long)g_radio_sched_grant_failures,
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
#else
    (void)now_ms;
    (void)next_print_ms;
#endif
}

#if RADIO_SCHEDULER_ENABLED
static uint32_t telemetry_radio_downlink_budget_ms(uint64_t now_ms,
                                                   uint64_t turn_started_ms)
{
    const uint64_t elapsed_ms = now_ms - turn_started_ms;
    if (elapsed_ms >= RADIO_SCHED_DOWNLINK_TIMEOUT_MS) {
        return 0U;
    }

    const uint64_t remaining_ms = RADIO_SCHED_DOWNLINK_TIMEOUT_MS - elapsed_ms;
    if (remaining_ms > 0xFFFFFFFFULL) {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)remaining_ms;
}
#endif

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;
#if RADIO_SCHEDULER_ENABLED
    uint8_t radio_turn = RADIO_SCHED_DOWNLINK;
    uint8_t radio_turn_seq = 0U;
    uint8_t radio_turn_sent = 0U;
    uint8_t radio_turn_grant_sent = 0U;
    uint64_t radio_turn_started_ms = 0U;
    uint64_t next_control_retry_ms = 0U;
    uint64_t next_grant_reannounce_ms = 0U;
    uint64_t next_idle_uplink_poll_ms = 0U;
#endif
    uint64_t next_discovery_announce_ms = 0U;
    uint64_t next_alive_print_ms = 0U;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();
    for (;;) {
        const uint64_t now_ms = radio_window_now_ms();
        telemetry_print_alive_if_due(now_ms, &next_alive_print_ms);

        /* Poll hardware FIFO and then process reassembly + router queues. */
        radio_uart_process_rx();
        can_bus_process_rx();
        telemetry_retry_pending_can_commands();
        telemetry_announce_discovery_if_due(now_ms, &next_discovery_announce_ms);
        (void)telemetry_poll_discovery();
        (void)process_rx_queue_timeout(TELEMETRY_QUEUE_BUDGET_MS);
        telemetry_retry_pending_can_commands();
        (void)dispatch_tx_queue_timeout(TELEMETRY_QUEUE_BUDGET_MS);
        (void)telemetry_poll_timesync();

#if RADIO_SCHEDULER_ENABLED
        if (radio_turn_started_ms == 0U) {
            radio_turn_started_ms = now_ms;
            radio_turn_seq++;
            g_radio_downlink_windows++;
            if (telemetry_emit_radio_grant(RADIO_SCHED_DOWNLINK, radio_turn_seq) == HAL_OK) {
                radio_turn_grant_sent = 1U;
                next_grant_reannounce_ms = now_ms + RADIO_SCHED_GRANT_REANNOUNCE_MS;
            } else {
                radio_turn_grant_sent = 0U;
                g_radio_sched_grant_failures++;
                next_control_retry_ms = now_ms + RADIO_SCHED_GRANT_RETRY_MS;
            }
        }

        if (radio_turn == RADIO_SCHED_DOWNLINK) {
            const uint8_t downlink_timeout =
                (now_ms - radio_turn_started_ms) >= RADIO_SCHED_DOWNLINK_TIMEOUT_MS;
            if (((radio_turn_sent >= RADIO_SCHED_BURST_MESSAGES) ||
                 downlink_timeout ||
                 (radio_uart_tx_queue_count() == 0U && now_ms >= next_idle_uplink_poll_ms)) &&
                !radio_uart_tx_busy()) {
                radio_turn = RADIO_SCHED_UPLINK;
                radio_turn_sent = 0U;
                radio_turn_seq++;
                radio_turn_grant_sent = 0U;
                radio_turn_started_ms = now_ms;
                next_control_retry_ms = now_ms + RADIO_SCHED_TURNAROUND_MS;
                next_grant_reannounce_ms = now_ms + RADIO_SCHED_TURNAROUND_MS + RADIO_SCHED_GRANT_REANNOUNCE_MS;
                next_idle_uplink_poll_ms = now_ms + RADIO_SCHED_IDLE_UPLINK_POLL_MS;
                g_radio_uplink_windows++;
                g_radio_sched_gs_seen = 0U;
            } else if (radio_turn_grant_sent) {
                const uint32_t downlink_budget_ms =
                    telemetry_radio_downlink_budget_ms(now_ms, radio_turn_started_ms);
                if (downlink_budget_ms >= RADIO_SCHED_MIN_DOWNLINK_BUDGET_MS &&
                    radio_uart_process_tx_with_budget(downlink_budget_ms) > 0U) {
                    radio_turn_sent++;
                    g_radio_downlink_tx_calls++;
                }
            }
        } else {
            g_radio_uplink_rx_only_loops++;
            const uint8_t gs_done =
                g_radio_sched_gs_seen &&
                g_radio_sched_gs_seq == radio_turn_seq &&
                (g_radio_sched_gs_flags & RADIO_SCHED_FLAG_YIELD) != 0U;
            const uint8_t uplink_timeout =
                (now_ms - radio_turn_started_ms) >= RADIO_SCHED_UPLINK_TIMEOUT_MS;
            if (gs_done || uplink_timeout) {
                radio_turn = RADIO_SCHED_DOWNLINK;
                radio_turn_sent = 0U;
                radio_turn_seq++;
                radio_turn_grant_sent = 0U;
                radio_turn_started_ms = now_ms;
                next_control_retry_ms = now_ms + RADIO_SCHED_UPLINK_TO_DOWNLINK_TURNAROUND_MS;
                next_grant_reannounce_ms = now_ms + RADIO_SCHED_UPLINK_TO_DOWNLINK_TURNAROUND_MS + RADIO_SCHED_GRANT_REANNOUNCE_MS;
                g_radio_downlink_windows++;
                g_radio_sched_gs_seen = 0U;
            }
        }

        if (!radio_turn_grant_sent && now_ms >= next_control_retry_ms) {
            if (telemetry_emit_radio_grant(radio_turn, radio_turn_seq) == HAL_OK) {
                radio_turn_grant_sent = 1U;
                next_grant_reannounce_ms = now_ms + RADIO_SCHED_GRANT_REANNOUNCE_MS;
            } else {
                radio_turn_grant_sent = 0U;
                g_radio_sched_grant_failures++;
                next_control_retry_ms = now_ms + RADIO_SCHED_GRANT_RETRY_MS;
            }
        } else if (radio_turn_grant_sent &&
                   now_ms >= next_grant_reannounce_ms &&
                   !radio_uart_tx_busy() &&
                   (radio_turn == RADIO_SCHED_UPLINK ||
                    (radio_turn == RADIO_SCHED_DOWNLINK && radio_uart_tx_queue_count() == 0U))) {
            if (telemetry_emit_radio_grant(radio_turn, radio_turn_seq) == HAL_OK) {
                next_grant_reannounce_ms = now_ms + RADIO_SCHED_GRANT_REANNOUNCE_MS;
            } else {
                g_radio_sched_grant_failures++;
                next_grant_reannounce_ms = now_ms + RADIO_SCHED_GRANT_RETRY_MS;
            }
        }
#else
        (void)radio_uart_process_tx();
#endif

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
