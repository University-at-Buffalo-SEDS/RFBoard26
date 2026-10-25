"""Link real probe declarations with the production ARM linker script under LTO/GC."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class ReadonlyAllocatorProbeTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('arm-none-eabi-gcc') and shutil.which('arm-none-eabi-nm'),'ARM GNU tools required')
    def test_tlsf_zero_probes_survive_lto_gc_without_ram_or_attribute_warnings(self):
        source=(ROOT/'Core/Src/telemetry_hooks.c').read_text()
        macro=re.search(r'^#define THREADX_ONLY_PROBE const .*$',source,re.M).group(0)
        declarations=re.findall(r'^THREADX_ONLY_PROBE [^;]+;',source,re.M)
        self.assertEqual(len(declarations),4)
        names=[re.search(r'\b(g_telemetry_\w+)\s*=',line).group(1) for line in declarations]
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);c=directory/'probes.c';elf=directory/'probes.elf'
            c.write_text('#include <stdint.h>\ntypedef unsigned long ULONG;\n'+macro+'\n'+'\n'.join(declarations)+'\nvoid Reset_Handler(void){for(;;){}}\n')
            subprocess.run(['arm-none-eabi-gcc','-mthumb','-mcpu=cortex-m4','-Os','-flto','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-nostdlib',str(c),'-Wl,--gc-sections','-T'+str(ROOT/'STM32G491XX_FLASH.ld'),'-o',str(elf)],check=True,capture_output=True,text=True)
            symbols=subprocess.check_output(['arm-none-eabi-nm','--defined-only',str(elf)],text=True)
            for name in names:
                match=re.search(r'^([0-9a-f]+) R '+name+'$',symbols,re.M)
                self.assertIsNotNone(match,'Probe discarded or allocated writable RAM: '+name)
                self.assertTrue(0x08000000<=int(match.group(1),16)<0x08080000)
