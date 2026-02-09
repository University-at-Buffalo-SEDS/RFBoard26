#include "gps_time.h"

static const int days_in_month[12] = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31
};


int is_leap_year(int year) {
    return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
}


double utc_to_epoch_ms(uint8_t day,
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
    uint64_t total_ms = total_days * 86400000LL; // days → ms
    total_ms += hours * 3600000LL;
    total_ms += minutes * 60000LL;
    total_ms += seconds * 1000LL;
    total_ms += milliseconds;

    return (double)total_ms;
}