// telemetry_hooks_threadx.c
#include "tx_api.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "main.h"

#ifndef TELEMETRY_STDIO_DEBUG
#define TELEMETRY_STDIO_DEBUG 0
#endif

static TX_BYTE_POOL *rust_byte_pool_external = NULL;
static TX_MUTEX g_telemetry_mutex;
static UINT g_telemetry_mutex_ready = 0U;
volatile uint32_t g_telemetry_lock_get_fail = 0U;
volatile uint32_t g_telemetry_lock_put_fail = 0U;
volatile uint32_t g_telemetry_alloc_fail = 0U;
volatile uint32_t g_telemetry_panic_count = 0U;
volatile size_t g_telemetry_last_alloc_request = 0U;
volatile ULONG g_telemetry_alloc_failure_available = 0U;
volatile ULONG g_telemetry_alloc_failure_fragments = 0U;
volatile ULONG g_telemetry_pool_available = 0U;
volatile ULONG g_telemetry_pool_low_water = ~0UL;
volatile ULONG g_telemetry_pool_fragments = 0U;
volatile size_t g_telemetry_max_alloc_request = 0U;
volatile uint32_t g_telemetry_alloc_count = 0U;
volatile uint32_t g_telemetry_free_count = 0U;
/* Stages: pre-router, router, CAN, radio, timesync, OTA, discovery, reserved. */
volatile ULONG g_telemetry_profile_available[8] = {0U};
volatile ULONG g_telemetry_profile_fragments[8] = {0U};
static volatile uint8_t g_last_err_memory_hint = 0U;
static volatile uint8_t g_last_err_mutex_hint = 0U;

static void telemetry_busy_delay(volatile uint32_t n)
{
    while (n--) { __NOP(); }
}

static void telemetry_panic_led_loop_memory(void)
{
    __disable_irq();
    for (;;)
    {
        /* Memory panic: two BLUE pulses, GREEN off, then pause. */
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);
        telemetry_busy_delay(9000000U);
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_RESET);
        telemetry_busy_delay(5000000U);
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);
        telemetry_busy_delay(9000000U);
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_RESET);
        telemetry_busy_delay(25000000U);
    }
}

static void telemetry_panic_led_loop_mutex(void)
{
    __disable_irq();
    for (;;)
    {
        /* Mutex panic: two GREEN pulses, BLUE off, then pause. */
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
        telemetry_busy_delay(9000000U);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        telemetry_busy_delay(5000000U);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
        telemetry_busy_delay(9000000U);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        telemetry_busy_delay(25000000U);
    }
}

static void telemetry_panic_led_loop_unknown(void)
{
    __disable_irq();
    for (;;)
    {
        /* Unknown panic: alternating BLUE/GREEN. */
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        telemetry_busy_delay(12000000U);
        HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
        telemetry_busy_delay(12000000U);
    }
}

static int str_contains_ci_n(const char *s, size_t n, const char *needle)
{
    if (!s || !needle) return 0;
    size_t needle_len = strlen(needle);
    if (needle_len == 0U || n < needle_len) return 0;

    for (size_t i = 0; i + needle_len <= n; ++i)
    {
        size_t j = 0;
        for (; j < needle_len; ++j)
        {
            char a = s[i + j];
            char b = needle[j];
            if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
            if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
            if (a != b) break;
        }
        if (j == needle_len) return 1;
    }
    return 0;
}

static void telemetry_memory_profile_sample(void)
{
    ULONG available = 0U;
    ULONG fragments = 0U;
    if (rust_byte_pool_external == NULL)
    {
        return;
    }
    if (tx_byte_pool_info_get(rust_byte_pool_external, TX_NULL,
                              &available, &fragments,
                              TX_NULL, TX_NULL, TX_NULL) == TX_SUCCESS)
    {
        g_telemetry_pool_available = available;
        g_telemetry_pool_fragments = fragments;
        if (available < g_telemetry_pool_low_water)
        {
            g_telemetry_pool_low_water = available;
        }
    }
}

void telemetry_memory_profile_mark(uint32_t stage)
{
    telemetry_memory_profile_sample();
    if (stage < 8U)
    {
        g_telemetry_profile_available[stage] = g_telemetry_pool_available;
        g_telemetry_profile_fragments[stage] = g_telemetry_pool_fragments;
    }
}

void telemetry_set_byte_pool(TX_BYTE_POOL *pool)
{
    rust_byte_pool_external = pool;
    telemetry_memory_profile_sample();
}

void telemetry_init_lock(void)
{
    if (g_telemetry_mutex_ready == 0U)
    {
        if (tx_mutex_create(&g_telemetry_mutex, "telemetry_mutex", TX_INHERIT) == TX_SUCCESS)
        {
            g_telemetry_mutex_ready = 1U;
        }
    }
}
void telemetry_lock(void)
{
    if (g_telemetry_mutex_ready == 0U)
    {
        return;
    }

    TX_THREAD *self = tx_thread_identify();
    if (self == TX_NULL)
    {
        /* Never block in ISR/startup context. */
        return;
    }

    UINT st = tx_mutex_get(&g_telemetry_mutex, TX_WAIT_FOREVER);
    if (st != TX_SUCCESS)
    {
        g_telemetry_lock_get_fail++;
    }
}

void telemetry_unlock(void)
{
    if (g_telemetry_mutex_ready == 0U)
    {
        return;
    }

    TX_THREAD *self = tx_thread_identify();
    if (self == TX_NULL)
    {
        /* Never block/touch mutex in ISR/startup context. */
        return;
    }

    UINT st = tx_mutex_put(&g_telemetry_mutex);
    if (st != TX_SUCCESS)
    {
        g_telemetry_lock_put_fail++;
    }
}

void *telemetryMalloc(size_t xSize)
{
    void *ptr = NULL;

    /* Defensive: if byte pool isn't registered yet, return NULL */
    if (rust_byte_pool_external == NULL)
    {
        return NULL;
    }

    if (xSize == 0U)
    {
        /* Rust allocator contract expects non-NULL for successful alloc. */
        xSize = 1U;
    }
    g_telemetry_last_alloc_request = xSize;
    if (xSize > g_telemetry_max_alloc_request)
    {
        g_telemetry_max_alloc_request = xSize;
    }

    /*
     * Allow a brief wait so telemetry bursts don't immediately fail allocator
     * requests and trigger panic paths in Rust.
     */
    if (tx_byte_allocate(rust_byte_pool_external, &ptr, xSize, 5) != TX_SUCCESS)
    {
        ULONG available = 0U;
        ULONG fragments = 0U;
        (void)tx_byte_pool_info_get(rust_byte_pool_external, TX_NULL,
                                    &available, &fragments,
                                    TX_NULL, TX_NULL, TX_NULL);
        g_telemetry_alloc_failure_available = available;
        g_telemetry_alloc_failure_fragments = fragments;
        g_telemetry_alloc_fail++;
        return NULL;
    }
    g_telemetry_alloc_count++;
    telemetry_memory_profile_sample();
    return ptr;
}

void telemetryFree(void *pv)
{
    if (pv != NULL)
    {
        (void)tx_byte_release(pv);
        g_telemetry_free_count++;
        telemetry_memory_profile_sample();
    }
}

void seds_error_msg(const char *str, size_t len)
{
    if (str != NULL && len > 0U)
    {
        g_last_err_memory_hint = (uint8_t)(
            str_contains_ci_n(str, len, "alloc") ||
            str_contains_ci_n(str, len, "memory") ||
            str_contains_ci_n(str, len, "oom"));
        g_last_err_mutex_hint = (uint8_t)(
            str_contains_ci_n(str, len, "mutex") ||
            str_contains_ci_n(str, len, "lock"));
#if TELEMETRY_STDIO_DEBUG
        printf("%.*s\r\n", (int)len, str);
#endif
    }
}

void telemetry_panic_hook(const char *str, size_t len)
{
    g_telemetry_panic_count++;

    if (str != NULL && len > 0U)
    {
        // printf("PANIC: %.*s\r\n", (int)len, str);
    }

    /* Prefer explicit text match if available. */
    if ((str != NULL && len > 0U &&
         (str_contains_ci_n(str, len, "alloc") || str_contains_ci_n(str, len, "memory"))) ||
        (g_telemetry_alloc_fail != 0U))
    {
        telemetry_panic_led_loop_memory();
    }

    if ((str != NULL && len > 0U &&
         (str_contains_ci_n(str, len, "mutex") || str_contains_ci_n(str, len, "lock"))) ||
        (g_last_err_mutex_hint != 0U) ||
        ((g_telemetry_lock_get_fail + g_telemetry_lock_put_fail) != 0U))
    {
        telemetry_panic_led_loop_mutex();
    }

    telemetry_panic_led_loop_unknown();
    
}
