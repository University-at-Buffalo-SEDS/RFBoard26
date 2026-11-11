"""Compile the production air-turn helpers of radio.c and drive them in time.

With a turn period (profile transport.turn_period_ms) RF starts frames for
30% of the period, then stays quiet until the period has passed, so frames
queued meanwhile leave together in the next turn and the station answers in
the gap (GroundStation26 docs/backend/rfd-modem-check.md, "Turn-taking")."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def block(source, start_marker, end_marker):
    start = source.index(start_marker)
    return source[start:source.index(end_marker, start)]


class TurnCadence(unittest.TestCase):
    def test_one_burst_per_turn_then_quiet_until_the_next(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        helpers = block(source, '#define RADIO_TURN_BURST_MS', '#endif')
        program = '''
#include <stdint.h>
#include <assert.h>
#define RFBOARD_LORA_SCHEDULER 0
#define RADIO_TURN_PERIOD_MS 200U
''' + helpers + '''
int main(void) {
  /* Idle: a frame may start at once and opens the turn. */
  assert(radio_uart_turn_remaining_ms(1000U) == 0U);
  radio_uart_turn_note_start(1000U);
  /* Inside the turn's 60 ms burst, more frames may start back to back. */
  assert(radio_uart_turn_remaining_ms(1010U) == 0U);
  radio_uart_turn_note_start(1010U);
  assert(radio_uart_turn_remaining_ms(1059U) == 0U);
  /* After the burst: quiet until the period has passed. */
  assert(radio_uart_turn_remaining_ms(1060U) == 140U);
  assert(radio_uart_turn_remaining_ms(1150U) == 50U);
  assert(radio_uart_turn_remaining_ms(1199U) == 1U);
  /* The next turn opens with the next frame, 200 ms after the last. */
  assert(radio_uart_turn_remaining_ms(1200U) == 0U);
  radio_uart_turn_note_start(1205U);
  assert(radio_uart_turn_remaining_ms(1240U) == 0U);
  assert(radio_uart_turn_remaining_ms(1270U) == 135U);
  /* Wraparound of the millisecond clock. */
  radio_uart_turn_remaining_ms(0xFFFFFFF0U + 400U);
  radio_uart_turn_note_start(0xFFFFFFF0U);
  assert(radio_uart_turn_remaining_ms(0xFFFFFFF0U + 70U) == 130U);
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'turn.c').write_text(program)
            subprocess.run(['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-o',
                            str(path / 'turn'), str(path / 'turn.c')], check=True)
            subprocess.run([str(path / 'turn')], check=True)

    def test_turn_cadence_is_declared_in_the_profile_reply(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        header = (ROOT / 'Core/Inc/recovery_profile.h').read_text()
        self.assertIn('#define RECOVERY_PROFILE_TURN_CADENCE 8U', header)
        self.assertIn('RECOVERY_PROFILE_TURN_CADENCE)) != 0U)', header, 'the reply builder accepts the flag')
        self.assertIn('flags |= RECOVERY_PROFILE_TURN_CADENCE;', source)
        quiet = block(source, 'static uint32_t radio_uart_quiet_remaining_ms(void)', '\n}\n')
        self.assertIn('return radio_uart_turn_remaining_ms(now);', quiet,
                      'the turn replaces burst/gap and airtime pacing')


if __name__ == '__main__':
    unittest.main()
