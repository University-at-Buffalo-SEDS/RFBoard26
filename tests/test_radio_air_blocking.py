"""The optional legacy LoRa scheduler keeps individual transmissions bounded."""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]

def default_block(source, name):
    start=source.index('#ifndef '+name+'\n')
    lines=source[start:].splitlines(keepends=True)
    depth=0
    for i,line in enumerate(lines):
        if line.startswith(('#if ', '#ifdef ', '#ifndef ')): depth+=1
        elif line.startswith('#endif'):
            depth-=1
            if depth==0: return ''.join(lines[:i+1])
    raise AssertionError('unclosed default block '+name)

class RadioAirBlockingTests(unittest.TestCase):
    def test_legacy_lora_frame_cap_keeps_air_block_below_70ms(self):
        radio=(ROOT/'Core/Src/radio.c').read_text()
        telemetry=(ROOT/'Core/Src/telemetry.c').read_text()
        cap=int(re.search(r'#define RF_RADIO_MAX_FRAME_BYTES (\d+)U',telemetry)[1])
        function=re.search(r'static uint32_t radio_uart_air_ms\(uint16_t len\)\n\{.*?\n\}',radio,re.S)[0]
        defines='\n'.join(default_block(radio,name) for name in ['RADIO_AIR_BIT_RATE_BPS','RADIO_AIR_SHARE_PERCENT','RADIO_AIR_FRAME_OVERHEAD_BYTES'])
        source='#include <stdint.h>\n#include <assert.h>\n#define RFBOARD_LORA_SCHEDULER 1\n'+defines+'\n'+function+f'\nint main(void){{assert(radio_uart_air_ms({cap}+4U)<=70U);assert(radio_uart_air_ms(1028U)>300U);}}'
        with tempfile.TemporaryDirectory() as d:
            exe=str(Path(d)/'air-budget')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-x','c','-','-o',exe],input=source,text=True,check=True)
            subprocess.run([exe],check=True)

    def test_profile_disabled_defaults_preserve_both_transports(self):
        radio=(ROOT/'Core/Src/radio.c').read_text()
        defines='\n'.join(default_block(radio,name) for name in ['RADIO_AIR_BIT_RATE_BPS','RADIO_AIR_SHARE_PERCENT'])
        for scheduler,rate,share in [(0,30000,100),(1,64000,40)]:
            with self.subTest(scheduler=scheduler), tempfile.TemporaryDirectory() as d:
                exe=str(Path(d)/'defaults')
                code=f'#define RFBOARD_LORA_SCHEDULER {scheduler}\n'+defines+f'\n_Static_assert(RADIO_AIR_BIT_RATE_BPS=={rate}, "air default");\n_Static_assert(RADIO_AIR_SHARE_PERCENT=={share}, "share default");\nint main(void){{return 0;}}'
                subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-x','c','-','-o',exe],input=code,text=True,check=True)
                subprocess.run([exe],check=True)
