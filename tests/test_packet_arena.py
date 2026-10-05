from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock
from dataclasses import fields
import build

ROOT=Path(__file__).resolve().parents[1]
class PacketArenaTests(unittest.TestCase):
    def test_build_selection_and_stale_cache_disable(self):
        for compact in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                commands=[]
                if hasattr(build, 'make_parser'):
                    args=build.make_parser().parse_args(['build','--release'] + (['--packet-store','compact'] if compact else []))
                    cfg=build.build_cfg_from_args(mock.Mock(),args)
                    self.assertEqual(cfg.sedsnet_ref, 'dev' if compact else 'main')
                    cfg.repo_root=Path(directory)
                    cfg.build_dir.mkdir(parents=True)
                    (cfg.build_dir / (cfg.project_name+'.elf')).touch()
                    if hasattr(cfg,'use_preset'): cfg.use_preset=False
                    with mock.patch.object(build,'run',side_effect=lambda ui,cmd,**kw:commands.append(cmd)):
                        build.configure_and_build(mock.Mock(),cfg)
                else:
                    options={key:False for key in build.ALL_OPTIONS}
                    options['packet-store-compact']=compact
                    with mock.patch.object(build,'PROJECT',Path(directory)), mock.patch.object(build,'run',side_effect=commands.append):
                        build.configure(Path(directory)/'build','Release',options)
                self.assertIn('-DSEDSNET_GIT_REF='+('dev' if compact else 'main'),commands[0])
                self.assertIn('-DSEDSNET_COMPACT_PACKET_STORE='+('ON' if compact else 'OFF'),commands[0])
                self.assertIn('-DSEDSNET_COMPACT_PACKET_COMPRESSION=OFF',commands[0])

    def test_arena_failure_retry_and_router_recreation(self):
        stub=r'''#pragma once
#include <stdint.h>
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -14
static unsigned calls;
static int result;
static int seds_packet_store_configure(unsigned bytes,unsigned handles,unsigned limit)
{ calls++; if(bytes!=4096 || handles!=32 || limit!=512) return -1; return result; }
'''
        body=r'''#include "board_packet_store.h"
#include <assert.h>
int main(void) {
#ifdef SEDS_ENABLE_COMPACT_PACKET_STORE
 result=-14; assert(board_packet_store_init()==-14 && calls==1);
 result=0; assert(board_packet_store_init()==0 && calls==2);
 for(unsigned i=0;i<20;i++) assert(board_packet_store_init()==0);
 assert(calls==2 && g_board_packet_store_init_result==0);
#else
 assert(board_packet_store_init()==0 && calls==0);
 (void)seds_packet_store_configure; (void)result;
#endif
}
'''
        for enabled in (False,True):
            with tempfile.TemporaryDirectory() as directory:
                d=Path(directory)
                (d/'sedsnet_config.h').write_text(stub)
                (d/'board_packet_store.h').write_text((ROOT/'Core/Inc/board_packet_store.h').read_text())
                cmd=['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-DBOARD_PACKET_ARENA_BYTES=4096','-DBOARD_PACKET_ARENA_HANDLES=32','-I',str(d)]
                if enabled: cmd+=['-DSEDS_ENABLE_COMPACT_PACKET_STORE=1']
                subprocess.run(cmd+['-x','c','-','-o',str(d/'test')],input=body,text=True,check=True)
                subprocess.run([str(d/'test')],check=True,timeout=5)

    def test_arena_initialization_precedes_router(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        self.assertLess(source.index('result = board_packet_store_init();'),source.index('r = seds_router_new'))
        self.assertIn('if (result != SEDS_OK) return result;',source)
