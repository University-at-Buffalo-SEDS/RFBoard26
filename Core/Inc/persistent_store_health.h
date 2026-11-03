#ifndef PERSISTENT_STORE_HEALTH_H
#define PERSISTENT_STORE_HEALTH_H
#include <stdbool.h>
/* RAM-only critical-journal admission state, owned by the strict wrapper. */
bool persistent_store_controls_ready(void);
#endif
