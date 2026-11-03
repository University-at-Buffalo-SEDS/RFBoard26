#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int launchcore_status_t;
enum{LAUNCHCORE_OK=0,LAUNCHCORE_ERR_NO_SPACE,LAUNCHCORE_ERR_BAD_STATE,LAUNCHCORE_ERR_BAD_IMAGE,LAUNCHCORE_ERR_VERIFY,LAUNCHCORE_ERR_FLASH,LAUNCHCORE_ERR_METADATA};
launchcore_status_t launchcore_delta_update_begin(uint32_t);launchcore_status_t launchcore_delta_update_write(const void*,size_t);
launchcore_status_t launchcore_delta_update_finish(void);launchcore_status_t launchcore_delta_update_abort(void);launchcore_status_t launchcore_confirm_boot(void);
