#ifndef AV_BAY_UNDERGLOW_H
#define AV_BAY_UNDERGLOW_H

#include "sedsnet.h"
#include "sedsnet_config.h"

extern volatile uint32_t g_av_bay_underglow_enabled;
extern volatile uint32_t g_av_bay_underglow_updates;
extern volatile uint32_t g_av_bay_underglow_persist_restores;
extern volatile uint32_t g_av_bay_underglow_persist_writes;
extern volatile uint32_t g_av_bay_underglow_persist_errors;

void av_bay_underglow_restore(void);
SedsResult av_bay_underglow_init(SedsRouter *router);
SedsResult av_bay_underglow_poll(SedsRouter *router);

#endif
