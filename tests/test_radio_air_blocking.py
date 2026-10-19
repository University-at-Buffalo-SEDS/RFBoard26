"""The largest generated RF frame must leave room for command/receipt turns."""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class RadioAirBlockingTests(unittest.TestCase):
    def test_actual_frame_cap_keeps_nonpreemptible_air_block_below_70ms(self):
        radio=(ROOT/'Core/Src/radio.c').read_text()
        telemetry=(ROOT/'Core/Src/telemetry.c').read_text()
        cap=int(re.search(r'#define RF_RADIO_MAX_FRAME_BYTES (\d+)U',telemetry)[1])
        function=re.search(r'static uint32_t radio_uart_air_ms\(uint16_t len\)\n\{.*?\n\}',radio,re.S)[0]
        defines='\n'.join(re.search(r'#define '+name+r'\s+\d+U',radio)[0] for name in ['RADIO_AIR_BIT_RATE_BPS','RADIO_AIR_SHARE_PERCENT','RADIO_AIR_FRAME_OVERHEAD_BYTES'])
        source='#include <stdint.h>\n#include <assert.h>\n'+defines+'\n'+function+f'\nint main(void){{assert(radio_uart_air_ms({cap}+4U)<=70U);assert(radio_uart_air_ms(1028U)>300U);}}'
        with tempfile.TemporaryDirectory() as d:
            exe=str(Path(d)/'air-budget')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-x','c','-','-o',exe],input=source,text=True,check=True)
            subprocess.run([exe],check=True)
