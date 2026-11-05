#include "persistent_store.h"
#include "commission_record.h"
#include <string.h>
#include "launchcore/storage.h"
#include "tx_api.h"
#include "main.h"
#include <stdbool.h>
#include <stdatomic.h>
extern const launchcore_storage_driver_t launchcore_board_storage_driver;
static TX_MUTEX mutex;
static uint8_t phase; /* 0 unavailable, 1 explicit exclusive boot, 2 runtime */
static _Atomic bool controls_ready;
static _Atomic bool commissioning_hold,commissioning_complete,ownership_faulted;
volatile uint32_t g_persistent_store_errors;

static bool ownership_result(bool success)
{
    if (!success) {
        ownership_faulted = true;
        controls_ready = false;
        ++g_persistent_store_errors;
    }
    return success;
}
bool persistent_store_enter(void)
{
    if (ownership_faulted) return false; /* Broken ownership is not healed by a retry. */
    if (__get_IPSR() != 0U) return ownership_result(false);
    if (phase == 1U) return ownership_result(tx_thread_identify() == TX_NULL);
    if(phase!=2U||tx_thread_identify()==TX_NULL)return ownership_result(false);
    UINT status=tx_mutex_get(&mutex,TX_WAIT_FOREVER);
    if(status==TX_SUCCESS)return true;
    if(status==TX_NOT_AVAILABLE||status==TX_WAIT_ABORTED){
        controls_ready=false;++g_persistent_store_errors;return false;
    }
    return ownership_result(false); /* Invalid/deleted owner lifetime, not BUSY. */
}
bool persistent_store_leave(void)
{
    if (__get_IPSR() != 0U) return ownership_result(false);
    if (phase == 1U) return ownership_result(tx_thread_identify() == TX_NULL);
    return ownership_result(phase == 2U && tx_thread_identify() != TX_NULL &&
        tx_mutex_put(&mutex) == TX_SUCCESS);
}
static launchcore_persist_status_t result(launchcore_persist_status_t status)
{
    const bool released = persistent_store_leave();
    if (!released && status == LAUNCHCORE_PERSIST_OK) status = LAUNCHCORE_PERSIST_STORAGE_ERROR;
    if (status == LAUNCHCORE_PERSIST_STORAGE_ERROR || status == LAUNCHCORE_PERSIST_CORRUPT) {
        controls_ready = false;
        ++g_persistent_store_errors;
    }
    return status;
}
static launchcore_persist_status_t inspect(void)
{
    launchcore_storage_set_driver(&launchcore_board_storage_driver);
    if (launchcore_storage_init() != LAUNCHCORE_STORAGE_OK) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status = launchcore_persist_probe_strict(&info);
    if (status == LAUNCHCORE_PERSIST_OK && info.health != LAUNCHCORE_PERSIST_HEALTH_HEALTHY && info.health != LAUNCHCORE_PERSIST_HEALTH_VIRGIN) {
        controls_ready = false;
        return LAUNCHCORE_PERSIST_CORRUPT;
    }
    return status;
}
static uint64_t decode(const uint8_t *bytes)
{
    uint64_t value = 0U;
    for (unsigned i = 0; i < 8U; ++i) value |= (uint64_t)bytes[i] << (8U * i);
    return value;
}
static void validate_controls(void)
{
    uint8_t bytes[16]; size_t size = 8U;
    launchcore_persist_strict_info_t info;
    controls_ready = false;
    if (launchcore_persist_get_strict(UINT32_C(0x42524745), bytes, &size, &info) != LAUNCHCORE_PERSIST_OK ||
        size != 8U || decode(bytes) == 0U || decode(bytes) == UINT64_MAX) return;
    size = sizeof(bytes);
    const launchcore_persist_status_t floor_status=launchcore_persist_get_strict(UINT32_C(0x4252464c),bytes,&size,&info);
    const bool no_prior_recovery=floor_status==LAUNCHCORE_PERSIST_NOT_FOUND&&
        info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT;
    if(!no_prior_recovery&&(floor_status!=LAUNCHCORE_PERSIST_OK||size!=sizeof(bytes)||
        decode(bytes)==0U||decode(bytes+8U)==0U||decode(bytes+8U)==UINT64_MAX))return;
    uint8_t marker[COMMISSION_RECORD_SIZE]; size=sizeof marker;
    launchcore_persist_status_t status=launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info);
    CommissionRecord record;
    commissioning_complete=false;
    if(status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT)commissioning_complete=true;
    else if(status==LAUNCHCORE_PERSIST_OK&&commission_record_decode(marker,size,&record)){
        if(record.state)commissioning_hold=true;
        else commissioning_complete=true;
    } else return; /* Never treat a deleted/unknown/v1 marker as legacy ABSENT. */
    controls_ready = true;
}
bool persistent_store_controls_ready(void) { return controls_ready&&!commissioning_hold&&!ownership_faulted; }
bool persistent_store_recovery_ready(void){return controls_ready&&!ownership_faulted;}
bool persistent_store_commission_pending(void){return commissioning_hold;}
void persistent_store_hold_commissioning(void){commissioning_hold=true;}
void persistent_store_note_failure(void){controls_ready=false;++g_persistent_store_errors;}
bool persistent_store_publish_commission_complete(void){
 if(ownership_faulted||!controls_ready||!commissioning_complete)return false;
 commissioning_hold=false;return true;
}
/* Caller holds the checked physical owner. No SDK/state-owner operation here.
 * Exact critical group readback is the only runtime metadata publication. */
bool persistent_store_verify_commissioned_locked(uint64_t generation,const uint8_t floor[16],const uint8_t marker[40]){
 CommissionRecord expected;
 if(ownership_faulted)return false;
 if(!generation||generation==UINT64_MAX||!commission_read64(floor)||!commission_read64(floor+8)||commission_read64(floor+8)==UINT64_MAX||!commission_record_decode(marker,40,&expected))return false;
 uint8_t observed[40];size_t size=8;launchcore_persist_strict_info_t info;
 bool valid=launchcore_persist_get_strict(UINT32_C(0x42524745),observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==8&&commission_read64(observed)==generation;
 size=16;if(valid)valid=launchcore_persist_get_strict(UINT32_C(0x4252464c),observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==16&&!memcmp(observed,floor,16);
 size=40;if(valid)valid=launchcore_persist_get_strict(COMMISSION_RECORD_KEY,observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==40&&!memcmp(observed,marker,40);
 if(!valid){persistent_store_note_failure();return false;}
 commissioning_complete=expected.state==0;controls_ready=true;return true;
}

/* Normal exclusive boot establishes bookkeeping, not application recovery.
 * Strict ABSENT permits first initialization; tombstones, ambiguous banks and
 * corrupt records are never repaired or reclassified as new storage. */
static launchcore_persist_status_t initialize_boot_identity(void)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    uint8_t identity[8]; size_t size=sizeof identity;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status=launchcore_persist_get_strict(UINT32_C(0x42524745),identity,&size,&info);
    if(status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&
       (info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN||info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY)) {
        const bool virgin=info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN;
        commission_write64(identity,1U);
        status=virgin?launchcore_persist_provision_virgin_strict(UINT32_C(0x42524745),identity,sizeof identity,&info):
            launchcore_persist_set_strict(UINT32_C(0x42524745),identity,sizeof identity,&info);
        if(status==LAUNCHCORE_PERSIST_OK) {
            size=sizeof identity;
            status=launchcore_persist_get_strict(UINT32_C(0x42524745),identity,&size,&info);
        }
    }
    if(status==LAUNCHCORE_PERSIST_OK&&(size!=sizeof identity||commission_read64(identity)==0U||commission_read64(identity)==UINT64_MAX))
        status=LAUNCHCORE_PERSIST_CORRUPT;
    if(status==LAUNCHCORE_PERSIST_NOT_FOUND) status=LAUNCHCORE_PERSIST_CORRUPT;
    if(status!=LAUNCHCORE_PERSIST_OK){ownership_faulted=true;persistent_store_note_failure();}
    return result(status);
}

/* MAIN is the only caller, before any task/SDK/OTA owner exists. No ISR calls
 * any storage API. End this phase before creating application tasks. */
launchcore_persist_status_t persistent_store_boot_begin(void)
{
    if (phase != 0U || __get_IPSR() != 0U || tx_thread_identify() != TX_NULL)
        return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    phase = 1U;
    launchcore_persist_status_t status = persistent_store_init();
    if (status == LAUNCHCORE_PERSIST_OK) status=initialize_boot_identity();
    if (status == LAUNCHCORE_PERSIST_OK) validate_controls();
    return status;
}
UINT persistent_store_runtime_start(void)
{
    controls_ready = controls_ready && phase == 1U;
    phase = 0U;
    if (__get_IPSR() != 0U) return TX_CALLER_ERROR;
    UINT status = tx_mutex_create(&mutex, "persist", TX_INHERIT);
    if (status == TX_SUCCESS) phase = 2U;
    else { controls_ready = false; ++g_persistent_store_errors; }
    return status;
}
launchcore_persist_status_t persistent_store_init(void)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    return result(inspect());
}
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *size)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status = launchcore_persist_get_strict(key, value, size, &info);
    const bool ordinary_absent=status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY&&
        info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(key==UINT32_C(0x4252464c)||key==COMMISSION_RECORD_KEY);
    if ((key == UINT32_C(0x42524745) || key == UINT32_C(0x4252464c) || key == COMMISSION_RECORD_KEY) && status != LAUNCHCORE_PERSIST_OK && !ordinary_absent)
        controls_ready = false;
    return result(status);
}
launchcore_persist_status_t persistent_store_set(uint32_t key, const void *value, size_t size)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status = launchcore_persist_set_strict(key, value, size, &info);
    if (status == LAUNCHCORE_PERSIST_OK && (key == UINT32_C(0x42524745) || key == UINT32_C(0x4252464c) || key == COMMISSION_RECORD_KEY))
        validate_controls(); /* Does not clear the independent Stop/reset/launch inhibit. */
    return result(status);
}
