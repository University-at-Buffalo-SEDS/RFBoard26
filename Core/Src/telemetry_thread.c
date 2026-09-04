// telemetry_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "tx_thread.h"
#include "telemetry.h"
#include "ota_stream.h"
#include "can_bus.h"
#include "radio.h"
#include "main.h"
#include <stdio.h>

TX_THREAD telemetry_thread;
extern FDCAN_HandleTypeDef hfdcan2;
volatile uint32_t g_telemetry_stack_used = 0U;
volatile uint32_t g_telemetry_stack_remaining = 0U;
volatile uint32_t g_telemetry_stack_start __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_stack_end __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_telemetry_init_stage __attribute__((used, externally_visible)) = 0U;
volatile int32_t g_telemetry_init_result __attribute__((used, externally_visible)) = 0;
#define TELEMETRY_THREAD_STACK_SIZE (12U * 1024U)
#define TELEMETRY_THREAD_SLEEP_TICKS 1U
#define TELEMETRY_QUEUE_BUDGET_MS 1U
#define TELEMETRY_ALIVE_PRINT_INTERVAL_MS 5000ULL
#define TELEMETRY_THREAD_PRIORITY 3U

#ifndef TELEMETRY_ALIVE_PRINTS
#define TELEMETRY_ALIVE_PRINTS 0
#endif

static uint64_t radio_window_now_ms(void)
{
    return ((uint64_t)tx_time_get() * 1000ULL) /
           (uint64_t)TX_TIMER_TICKS_PER_SECOND;
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
           "tx_startup_drops=%lu "
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

static void telemetry_update_stack_profile(void)
{
    _tx_thread_stack_analyze(&telemetry_thread);
    const uintptr_t start = (uintptr_t)telemetry_thread.tx_thread_stack_start;
    const uintptr_t end = (uintptr_t)telemetry_thread.tx_thread_stack_end;
    const uintptr_t highest =
        (uintptr_t)telemetry_thread.tx_thread_stack_highest_ptr;
    /* ThreadX leaves highest_ptr null when stack checking is unavailable or
     * before its first successful scan. Do not turn that instrumentation gap
     * into a false zero-margin result. */
    if ((highest >= start) && (highest <= end)) {
        g_telemetry_stack_used =
            (uint32_t)(end - highest + sizeof(ULONG));
        g_telemetry_stack_remaining = (uint32_t)(highest - start);
    }
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    uint64_t next_alive_print_ms = 0U;

    // Publish a nonzero stack margin before router/flash/link initialization.
    // Keeping this updated at the start of every service pass also lets the
    // health monitor distinguish a busy relay from a thread that never ran.
    telemetry_update_stack_profile();

    /* The transport must be fully configured before router initialization can
     * emit its initial time-source and discovery traffic. */
    g_telemetry_init_stage = 1U;
    can_bus_init(&hfdcan2);
    g_telemetry_init_stage = 2U;

    // Ensure router exists early (so we can send requests immediately)
    g_telemetry_init_result = (int32_t)init_telemetry_router();
    g_telemetry_init_stage = 3U;
    for (;;) {
        telemetry_update_stack_profile();
        const uint64_t now_ms = radio_window_now_ms();
        telemetry_print_alive_if_due(now_ms, &next_alive_print_ms);

        /* Poll hardware FIFO and then process reassembly + router queues. */
        radio_uart_process_rx();
        can_bus_process_rx();
        telemetry_retry_pending_can_commands();
        /* SEDSNet owns discovery cadence.  Forcing an additional full snapshot
         * here creates a burst larger than the constrained radio queue and
         * bypasses its adaptive fast/slow discovery behavior. */
        (void)telemetry_poll_discovery();
        telemetry_retry_pending_can_commands();
        /* RF is a relay between avionics CAN and the ground radio. Draining
         * only TX leaves every packet received from either side stranded in
         * the router RX queue, eventually exhausting the fixed allocator and
         * preventing any cross-link forwarding. Interleave RX fanout and TX
         * dispatch within the same bounded service window. */
        (void)process_all_queues_timeout(TELEMETRY_QUEUE_BUDGET_MS);
        (void)telemetry_poll_timesync();
        ota_stream_poll();

        (void)radio_uart_process_tx();

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

    g_telemetry_stack_start = (uint32_t)(uintptr_t)pointer;
    g_telemetry_stack_end = (uint32_t)(uintptr_t)pointer + TELEMETRY_THREAD_STACK_SIZE;
    g_telemetry_stack_remaining = TELEMETRY_THREAD_STACK_SIZE;

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
