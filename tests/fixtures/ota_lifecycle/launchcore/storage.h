#pragma once
typedef struct{int marker;}launchcore_storage_driver_t;typedef int launchcore_storage_status_t;
#define LAUNCHCORE_STORAGE_OK 0
void launchcore_storage_set_driver(const launchcore_storage_driver_t*);launchcore_storage_status_t launchcore_storage_init(void);
