#include "gps_time.h"

// Global GPS time offset (exported symbols)
volatile int64_t g_gps_time_offset_ms = 0;     // ms
volatile uint32_t g_gps_time_offset_valid = 0; // 0/1

static const int days_in_month[12] = {
    31,     // Jan
    28,     // Feb
    31,     // Mar
    30,     // Apr
    31,     // May
    30,     // Jun
    31,     // Jul
    31,     // Aug
    30,     // Sep
    31,     // Oct
    30,    // Nov
    31     // Dec
};

// time offset accessor API (also globally link-visible)
int64_t gps_time_offset_ms_get(void) { 
    return g_gps_time_offset_ms; 
}
uint32_t gps_time_offset_valid_get(void) { 
    return g_gps_time_offset_valid; 
}

int is_leap_year(int year) {
    return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
}

void gps_offset_update_ms(int64_t measured_offset_ms) {
    if (measured_offset_ms > GPS_OFFSET_MAX_STEP_MS || measured_offset_ms < -GPS_OFFSET_MAX_STEP_MS) {
        return;
    }

    // Smooth: new = old + measured/Div
    int64_t step = measured_offset_ms / (int64_t)GPS_OFFSET_SMOOTH_DIV;
    if (step == 0) {
        if (measured_offset_ms > 0) {
            step = 1;
        }
        else if (measured_offset_ms < 0) {
            step = -1;
        }
    }

    g_gps_time_offset_ms += step;
    g_gps_time_offset_valid = 1;
}

uint64_t utc_to_epoch_ms(uint8_t day,
    uint8_t month,
    uint8_t year,
    uint8_t hours,
    uint8_t minutes,
    uint8_t seconds,
    uint16_t milliseconds)
{
    int full_year = CURRENT_MILLENNIA + year; // 2000 + yy
    uint64_t total_days = 0;

    // Count days from 1970 to current year
    for (int y = 1970; y < full_year; y++) {
        total_days += is_leap_year(y) ? 366 : 365;
    }

    // Count days from January to current month
    for (int m = 1; m < month; m++) {
        total_days += days_in_month[m - 1];
        if (m == 2 && is_leap_year(full_year)) {
            total_days += 1; // February leap day
        }
    }

    // Days in current month (day is 1-based)
    total_days += (day - 1);

    // Convert everything to milliseconds
    uint64_t total_ms = total_days * 86400000ULL; // days → ms
    total_ms += hours * 3600000ULL;
    total_ms += minutes * 60000ULL;
    total_ms += seconds * 1000ULL;
    total_ms += milliseconds;

    return (uint64_t)total_ms;
}