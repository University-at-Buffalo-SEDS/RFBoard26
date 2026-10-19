"""A schema burst must not starve a different board's delivery receipt."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class CanArbitrationPriorityTests(unittest.TestCase):
    def test_schema_burst_preserves_receipt_latency_and_every_bulk_frame(self):
        code=(ROOT/'Core/Inc/board_can_priority.h').read_text()+r'''
#include <assert.h>
#include <stdio.h>
static unsigned run(int legacy) {
 const uint8_t schema[]={'S','D','T',3,0,1};
 const uint8_t ack[]={12,0,1,2,0,1};
 unsigned metadata_id=legacy?1:board_can_arbitration_id(schema,sizeof(schema),254,0,120,1);
 unsigned receipt_id=board_can_arbitration_id(ack,sizeof(ack),255,1,120,3);
 unsigned sent_bulk=0,sent_ack=0,first_ack_us=0,elapsed_us=0;
 /* RF (id1) has a sustained schema backlog; FC (id3) has three receipts.
  * Each complete 64-byte CAN-FD frame occupies a conservative600us at1M.
  * Every accepted bulk frame must still leave in its original order. */
 while(sent_bulk<2048 || sent_ack<3) {
  if(sent_ack<3 && (sent_bulk==2048 || receipt_id<metadata_id)) {
   elapsed_us+=600; if(sent_ack++==0)first_ack_us=elapsed_us;
  } else { assert(sent_bulk<2048);sent_bulk++;elapsed_us+=600; }
 }
 assert(sent_bulk==2048 && sent_ack==3);return first_ack_us;
}
int main(void) {
 unsigned old_delay=run(1),new_delay=run(0);
 assert(old_delay>500000 && new_delay<=500000);
 const uint8_t canonical[]={12,1,42};
 const uint8_t opaque[]={83,68,7};
 assert(board_can_arbitration_id(canonical,3,254,1234,120,1)==1);
 assert(board_can_arbitration_id(canonical,3,255,1,120,1)==1);
 assert(board_can_arbitration_id(canonical,3,0,120,120,1)==1);
 assert(board_can_arbitration_id(canonical,3,255,10,120,1)==0x81);
 assert(board_can_arbitration_id(canonical,3,0,1234,120,1)==0x101);
 assert(board_can_arbitration_id(opaque,3,255,0,120,1)==1);
 assert(board_can_arbitration_id(NULL,0,255,0,120,1)==0x101);
 printf("schema arbitration: old ACK%uus, corrected ACK%uus; all2048 bulk frames and3 receipts retained\n",old_delay,new_delay);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe=Path(tmp)/'arbitration'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,check=True)
            subprocess.run([str(exe)],check=True)
