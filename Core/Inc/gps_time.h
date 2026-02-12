#include <stdint.h>

#define CURRENT_MILLENNIA 0x7D0

// Smooth offset updates to avoid jitter
#ifndef GPS_OFFSET_SMOOTH_DIV
#define GPS_OFFSET_SMOOTH_DIV 8
#endif

#ifndef GPS_OFFSET_MAX_STEP_MS
#define GPS_OFFSET_MAX_STEP_MS 60000 // ignore insane jumps > 60s
#endif

int is_leap_year(int year);
void gps_offset_update_ms(int64_t measured_offset_ms);
uint64_t utc_to_epoch_ms(uint8_t day,
    uint8_t month,
    uint8_t year,
    uint8_t hours,
    uint8_t minutes,
    uint8_t seconds,
    uint16_t milliseconds
);
