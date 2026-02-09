#include <stdint.h>

#define CURRENT_MILLENNIA 0x7D0

int is_leap_year(int year);
double utc_to_epoch_ms(uint8_t day,
    uint8_t month,
    uint8_t year,
    uint8_t hours,
    uint8_t minutes,
    uint8_t seconds,
    uint16_t milliseconds);
