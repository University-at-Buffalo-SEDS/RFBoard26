"""Actual ThreadX create/search startup witness; only interrupt hooks are inert.
Host pointers cost 16 bytes/block versus target8, so remaining capacity is conservative.
The parent pool is not a Rust quota: only two startup reservations consume it.
"""
import re
import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class ParentPoolStartupTests(unittest.TestCase):
 def test_actual_threadx_startup_reservations_keep_two_kib_unused(self):
  app=(ROOT/'Core/Src/app_threadx.c').read_text()
  gps=(ROOT/'Core/Src/neom9n_thread.c').read_text().split('UINT create_neom9n_thread(',1)[1]
  thread=(ROOT/'Core/Src/telemetry_thread.c').read_text()
  self.assertEqual(app.count('tx_byte_allocate('),1)
  self.assertIn('RF_SEDSNET_MEMORY_POOL_SIZE - RF_SEDSNET_EMERGENCY_POOL_SIZE',app)
  self.assertNotIn('tx_byte_allocate(',gps)
  self.assertIn('(void)byte_pool;',gps)
  self.assertEqual(thread.count('tx_byte_allocate('),1)
  self.assertIn('#define TELEMETRY_THREAD_STACK_SIZE (14U * 1024U)',thread)
  config=(ROOT/'AZURE_RTOS/App/app_azure_rtos_config.h').read_text()
  capacity=int(re.search(r'#define TX_APP_MEM_POOL_SIZE\s+(\d+)',config)[1])
  common=ROOT/'Middlewares/ST/threadx/common/src'
  actual='\n'.join((common/('tx_byte_pool_'+n+'.c')).read_text().split('UINT  _tx_byte_pool_create',1)[1].join(['UINT  _tx_byte_pool_create','']) if n=='create' else 'UCHAR  *_tx_byte_pool_search'+(common/('tx_byte_pool_'+n+'.c')).read_text().split('UCHAR  *_tx_byte_pool_search',1)[1] for n in ['create','search'])
  header=r'''#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
typedef unsigned UINT;typedef uintptr_t ULONG;typedef uintptr_t ALIGN_TYPE;
typedef unsigned char UCHAR;typedef char CHAR;typedef void VOID;typedef int TX_THREAD;
typedef struct TX_BYTE_POOL_STRUCT {CHAR*tx_byte_pool_name;UCHAR*tx_byte_pool_start;ULONG tx_byte_pool_size;UCHAR*tx_byte_pool_list,*tx_byte_pool_search;ULONG tx_byte_pool_available;UINT tx_byte_pool_fragments;TX_THREAD*tx_byte_pool_owner;ULONG tx_byte_pool_id;struct TX_BYTE_POOL_STRUCT*tx_byte_pool_created_next,*tx_byte_pool_created_previous;} TX_BYTE_POOL;
static TX_BYTE_POOL *_tx_byte_pool_created_ptr;static ULONG _tx_byte_pool_created_count;
#define TX_INTERRUPT_SAVE_AREA
#define TX_DISABLE
#define TX_RESTORE
#define TX_NULL NULL
#define TX_SUCCESS 0
#define TX_EMPTY 0
#define TX_FALSE 0
#define TX_TRUE 1
#define TX_BYTE_POOL_ID 1
#define TX_BYTE_BLOCK_FREE ((ULONG)0xFFFFEEEEUL)
#define TX_BYTE_BLOCK_MIN ((ULONG)20)
#define TX_MEMSET memset
#define TX_THREAD_GET_CURRENT(p) p=NULL;
#define TX_VOID_TO_UCHAR_POINTER_CONVERT(p) ((UCHAR*)(p))
#define TX_BYTE_POOL_TO_UCHAR_POINTER_CONVERT(p) ((UCHAR*)(p))
#define TX_UCHAR_TO_INDIRECT_UCHAR_POINTER_CONVERT(p) ((UCHAR**)(p))
#define TX_UCHAR_TO_ALIGN_TYPE_POINTER_CONVERT(p) ((ALIGN_TYPE*)(p))
#define TX_UCHAR_POINTER_ADD(p,n) ((p)+(n))
#define TX_UCHAR_POINTER_SUB(p,n) ((p)-(n))
#define TX_UCHAR_POINTER_DIF(p,q) ((ULONG)((p)-(q)))
#define TX_BYTE_POOL_CREATE_EXTENSION(p)
#define TX_TRACE_OBJECT_REGISTER(...)
#define TX_TRACE_IN_LINE_INSERT(...)
#define TX_EL_BYTE_POOL_CREATE_INSERT
'''
  main=r'''
int main(void){
 _Alignas(16) static UCHAR memory[CAPACITY];TX_BYTE_POOL pool;
 assert(_tx_byte_pool_create(&pool,"startup",memory,sizeof memory)==TX_SUCCESS);
 UCHAR*sdk=_tx_byte_pool_search(&pool,68096U-19456U);assert(sdk);
 UCHAR*stack=_tx_byte_pool_search(&pool,14U*1024U);assert(stack);
 assert(stack>=sdk+(68096U-19456U));
 assert(pool.tx_byte_pool_available>=2048U+2U*sizeof(void*));
 UCHAR*slack=_tx_byte_pool_search(&pool,2048U);assert(slack);
 memset(sdk,0x31,68096U-19456U);memset(stack,0x42,14U*1024U);memset(slack,0x53,2048U);
 assert(sdk[48639]==0x31&&stack[14335]==0x42&&slack[2047]==0x53);
 printf("parent=%zu SDK=48640 stack=14336 unused_after_headers=%zu host_block_header=%zu\n",sizeof memory,(size_t)(pool.tx_byte_pool_available+2048U+2U*sizeof(void*)),2U*sizeof(void*));return 0;}
'''.replace('CAPACITY',str(capacity))
  with tempfile.TemporaryDirectory()as d:
   f=Path(d)/'test.c';f.write_text(header+actual+main);exe=Path(d)/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer',str(f),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
