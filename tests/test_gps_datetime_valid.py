"""RF takes GPS time only from a fix with a plausible date.

Before a fix the receiver still reports its power-up clock with a placeholder
date. Accepting that made RF the network time source (priority 1, ahead of the
ground station) and pulled every board to RF uptime.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from router_lifecycle_fixture import function

ROOT = Path(__file__).resolve().parents[1]

PROGRAM = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define GPS_MIN_VALID_YEAR 25U
#define GPS_MAX_VALID_YEAR 79U
typedef struct {
    uint8_t day, month, year, hours, minutes, seconds;
    uint16_t milliseconds;
    bool valid_fix;
} NEOM9N_t;
@@VALID@@
int main(void) {
    NEOM9N_t fixed = {9, 11, 26, 3, 41, 7, 250, true};
    assert(gps_datetime_valid(&fixed));
    NEOM9N_t no_fix = fixed;
    no_fix.valid_fix = false;
    assert(!gps_datetime_valid(&no_fix));
    NEOM9N_t placeholder = fixed;
    placeholder.year = 80;
    assert(!gps_datetime_valid(&placeholder));
    placeholder.year = 0;
    assert(!gps_datetime_valid(&placeholder));
    NEOM9N_t bad_month = fixed;
    bad_month.month = 13;
    assert(!gps_datetime_valid(&bad_month));
    assert(!gps_datetime_valid(NULL));
    return 0;
}
'''


class GpsDatetimeValidTests(unittest.TestCase):
    def test_only_a_fix_with_a_plausible_date_is_gps_time(self):
        source = (ROOT / 'Core/Src/neom9n.c').read_text()
        program = PROGRAM.replace('@@VALID@@', function(source, 'gps_datetime_valid'))
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'valid.c'
            exe = Path(tmp) / 'valid'
            c.write_text(program)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
