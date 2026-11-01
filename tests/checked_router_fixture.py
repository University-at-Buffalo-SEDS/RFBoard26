"""Live FC checked ownership functions with controlled ThreadX primitives."""
from pathlib import Path
from router_lifecycle_fixture import function

def checked_hooks(root):
    source=(Path(root)/'Core/Src/telemetry_hooks.c').read_text()
    prefix=r"""
#include <stddef.h>
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER (~0U)
#define TX_NULL NULL
#define TX_MUTEX_ID 0x4D555445U
typedef unsigned UINT;
typedef struct { unsigned id; } TX_THREAD;
typedef struct { unsigned tx_mutex_id; } TX_MUTEX;
static TX_MUTEX g_telemetry_mutex={TX_MUTEX_ID};
static unsigned g_telemetry_mutex_ready=1,g_telemetry_lock_get_fail,g_telemetry_lock_put_fail;
static unsigned fixture_isr,fixture_get_failure,fixture_put_failure;
static TX_THREAD *tx_thread_identify(void){return (TX_THREAD*)1;}
static unsigned __get_IPSR(void){return fixture_isr;}
static unsigned tx_mutex_get(TX_MUTEX *mu,unsigned timeout){assert(mu==&g_telemetry_mutex&&timeout==TX_WAIT_FOREVER);if(fixture_get_failure)return 1;telemetry_lock();return TX_SUCCESS;}
static unsigned tx_mutex_put(TX_MUTEX *mu){assert(mu==&g_telemetry_mutex);telemetry_unlock();return fixture_put_failure?1:TX_SUCCESS;}
"""
    if (Path(root)/'Core/Inc/board_telemetry_mutex.h').is_file():
        header=(Path(root)/'Core/Inc/board_telemetry_mutex.h').read_text().replace('#include "tx_api.h"','')
        prefix+='\n#define TX_NOT_OWNED 30U\n'+header+'\nstatic BoardTelemetryMutex g_telemetry_owner;\n'
    return prefix+'\n'.join(function(source,name) for name in ('telemetry_router_context_ready','telemetry_router_enter','telemetry_router_leave'))
