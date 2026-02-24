// telemetry_hooks_threadx.c
#include "tx_api.h"
#include <stddef.h>
#include <stdio.h>
#include "main.h"

static TX_BYTE_POOL *rust_byte_pool_external = NULL;
static TX_MUTEX g_telemetry_mutex;
static UINT g_telemetry_mutex_ready = 0U;
static TX_THREAD *g_telemetry_mutex_owner = TX_NULL;
static UINT g_telemetry_mutex_recursion = 0U;

void telemetry_set_byte_pool(TX_BYTE_POOL *pool)
{
    rust_byte_pool_external = pool;
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
    // if (self == TX_NULL)
    // {
    //     /* Not in thread context; cannot safely block on a mutex. */
    //     return;
    // }

    if (g_telemetry_mutex_owner == self)
    {
        /* Manual recursion for platforms where mutexes are not re-entrant. */
        g_telemetry_mutex_recursion++;
        return;
    }

    if (tx_mutex_get(&g_telemetry_mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
    {
        g_telemetry_mutex_owner = self;
        g_telemetry_mutex_recursion = 1U;
    }
}

void telemetry_unlock(void)
{
    if (g_telemetry_mutex_ready == 0U)
    {
        return;
    }

    TX_THREAD *self = tx_thread_identify();
    // if (self == TX_NULL)
    // {
    //     /* Not in thread context; ignore. */
    //     return;
    // }

    if (g_telemetry_mutex_owner != self)
    {
        return;
    }

    if (g_telemetry_mutex_recursion > 1U)
    {
        g_telemetry_mutex_recursion--;
        return;
    }

    if (tx_mutex_put(&g_telemetry_mutex) == TX_SUCCESS)
    {
        g_telemetry_mutex_owner = TX_NULL;
        g_telemetry_mutex_recursion = 0U;
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

    /* Never block allocator: on tight memory, fail fast instead of wedging threads. */
    if (tx_byte_allocate(rust_byte_pool_external, &ptr, xSize, TX_NO_WAIT) != TX_SUCCESS)
    {
        return NULL;
    }
    return ptr;
}

void telemetryFree(void *pv)
{
    if (pv != NULL)
    {
        (void)tx_byte_release(pv);
    }
}

void seds_error_msg(const char *str, size_t len)
{
    (void)len;
    printf("%s\r\n", str);
}

void telemetry_panic_hook(const char *str, size_t len)
{
    (void)len;
    if (str != NULL)
    {
        printf("PANIC: %s\r\n", str);
    }
    Error_Handler();
    
}