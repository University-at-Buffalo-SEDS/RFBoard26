"""Actual RF gates/DMA and owned queue: bounded asynchronous receipt tails."""
from pathlib import Path
import subprocess
import re
import tempfile
from unittest import mock
import unittest
import test_radio_funded_packet_boundary as funded
import test_radio_control_fifo as fifo

HELPERS = funded.CHECKS[:funded.CHECKS.index('int main(void)')].replace('static void normal(', 'static void __attribute__((unused)) normal(') + r'''
static void __attribute__((unused)) fresh(unsigned id){
 queue(fc,sizeof(fc),255);
 candidate.data[candidate.len-12]^=(uint8_t)id;
 const uint32_t crc=launchcore_crc32(candidate.data+4,candidate.len-8);
 for(unsigned n=0;n<4;++n)candidate.data[candidate.len-4+n]=(uint8_t)(crc>>(n*8));
}
'''

class LateReceiptTailTests(unittest.TestCase):
    def run_native(self, body):
        funded.FundedAckBoundaryTests()._run(HELPERS+'int main(void){'+body+'\nreturn 0;}')

    def test_async_receipts_share_first_stop_window_without_credit_or_debt_forgiveness(self):
        self.run_native(r'''
 reset();queue(und,sizeof(und),255);start(1000);
 assert(g_tx_receipt_tail.ready==1);done(1005);
 const unsigned close=g_tx_last_forced_boundary_ms, credit=g_tx_completed_service_credit_ms;
 assert(g_tx_receipt_tail.first_stop_ms==1005 && g_tx_receipt_tail.ready==2);
 fresh(1);start(1010);assert(g_tx_quiet_until_ms==1045);done(1015);
 assert(g_tx_receipt_tail.first_stop_ms==1005 && g_tx_receipt_tail.bytes==109);
 fresh(2);start(1020);assert(g_tx_quiet_until_ms==1067);done(1025);
 assert(g_tx_receipt_tail.first_stop_ms==1005 && g_tx_receipt_tail.frames==3);
 assert(g_tx_last_forced_boundary_ms==close && g_tx_completed_service_credit_ms==credit);
 assert(g_tx_independent_streak==3 && g_tx_packet_until_ms==1080);
 fresh(3);clock_ms=1030;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1); /* 217>196 */
 assert(g_tx_receipt_tail.bytes==163 && g_tx_receipt_tail.frames==3);
''')

    def test_first_stop_deadline_never_slides_and_future_small_frames_cannot_exceed_four(self):
        self.run_native(r'''
 reset();queue(und,sizeof(und),255);start(1000);done(1005);
 fresh(1);start(1025);done(1030);
 fresh(2);clock_ms=1033;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 assert(g_tx_receipt_tail.first_stop_ms==1005); /* 28ms after FIRST stop, not3ms after suffix */
 reset();queue(und,sizeof(und),255);start(2000);done(2005);
 g_tx_receipt_tail.frames=4;fresh(1);clock_ms=2010;
 assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
''')

    def test_crc_duplicates_dependent_history_and_fifo_obligation_cannot_fund_tail(self):
        self.run_native(r'''
 reset();queue(und,sizeof(und),255);start(1000);done(1005);
 queue(und,sizeof(und),255);clock_ms=1010;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 fresh(1);candidate.data[candidate.len-1]^=1;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 normal(124);assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 fresh(1);g_tx_independent_streak=7;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 assert(g_tx_receipt_tail.frames==1 && g_tx_receipt_tail.bytes==sizeof(und));
''')

    def test_pending_partial_error_refusal_and_recovery_do_not_authorize_tail(self):
        self.run_native(r'''
 reset();queue(und,sizeof(und),255);start(1000);
 fresh(1);clock_ms=1010;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1); /* DMA still owns prefix */
 HAL_UART_ErrorCallback(g_huart);done(1011);
 clock_ms=1015;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 reset();queue(und,sizeof(und),255);hal_status=HAL_BUSY;clock_ms=2000;
 assert(!radio_uart_process_tx_with_budget(2000) && g_tx_receipt_tail.ready==0);
 reset();queue(und,sizeof(und),255);start(3000);done(3005);
 fresh(1);hal_status=HAL_BUSY;clock_ms=3010;assert(!radio_uart_process_tx_with_budget(2000));
 assert(g_tx_receipt_tail.ready==0 && g_tx_quiet_until_ms==3023);
 fresh(2);hal_status=HAL_OK;clock_ms=3011;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 reset();queue(und,sizeof(und),255);start(4000);
 offered=0;clock_ms=4200;(void)radio_uart_process_tx_with_budget(2000);
 assert(g_tx_receipt_tail.ready==0 && g_tx_dma_recoveries>0);
''')

    def test_first_uart_stop_time_wraps_safely(self):
        self.run_native(r'''
 reset();queue(und,sizeof(und),255);start(0xfffffff0U);done(0xfffffff5U);
 fresh(1);start(0xfffffffaU);done(0xffffffffU);
 fresh(2);start(4U);done(9U);
 assert(g_tx_receipt_tail.first_stop_ms==0xfffffff5U && g_tx_receipt_tail.frames==3);
 fresh(3);clock_ms=0x11U;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
''')

    def test_original_no_tail_admission_rejects_same_early_second_receipt(self):
        # Restore only the old closing/loan admission rule in the actual source.
        # No sibling checkout is needed, and compile failures cannot satisfy
        # this negative control.
        original = funded.ROOT
        source = (original / 'Core/Src/radio.c').read_text()
        source, replacements = re.subn(
            r'late_ack_tail = g_tx_receipt_tail.ready == 2U &&.*?;',
            'late_ack_tail = 0U;', source, count=1, flags=re.S)
        self.assertEqual(replacements, 1)
        compile_ok = False
        run = subprocess.run
        def observed_run(command, *args, **kwargs):
            nonlocal compile_ok
            result = run(command, *args, **kwargs)
            if command[0] == 'cc':
                compile_ok = result.returncode == 0
            return result
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'Core/Src').mkdir(parents=True)
            (root / 'tests').mkdir()
            (root / 'Core/Src/radio.c').write_text(source)
            (root / 'tests/radio_crc_reference.h').write_bytes(
                (original / 'tests/radio_crc_reference.h').read_bytes())
            try:
                funded.ROOT = root
                with mock.patch.object(subprocess, 'run', side_effect=observed_run):
                    with self.assertRaises(subprocess.CalledProcessError):
                        self.run_native(r'''
 reset();queue(und,sizeof(und),255);start(1000);done(1005);
 queue(fc,sizeof(fc),255);clock_ms=1010;
 assert(radio_uart_process_tx_with_budget(2000)==1);
''')
                self.assertTrue(compile_ok)
            finally:
                funded.ROOT = original

    def test_already_queued_receipts_bypass_interposed_sdt_without_reordering_history(self):
        frames='\n'.join(re.search(r'static const uint8_t '+name+r'\[\]=\{[^;]+;', funded.ACK_FIXTURES).group() for name in ['und','fc'])
        fifo.RadioControlFifoTests()._run_case(frames+r'''int main(void){
 radio_uart_drop_stale_locked(0);
 uint8_t a[16]={165,90,12,0,'S','D','T',3,0x11};
 uint8_t b[16]={165,90,12,0,'S','D','T',3,0x22};
 radio_tx_item_t first,second;
 assert(radio_uart_enqueue_sedsnet_frame(und,sizeof(und),255)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(a,sizeof(a),254)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(fc,sizeof(fc),255)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(b,sizeof(b),254)==HAL_OK);
 uint8_t third[sizeof(fc)];memcpy(third,fc,sizeof(third));third[sizeof(third)-12]^=1;
 const uint32_t crc=launchcore_crc32(third+4,sizeof(third)-8);
 for(unsigned n=0;n<4;++n)third[sizeof(third)-4+n]=(uint8_t)(crc>>(8*n));
 assert(radio_uart_enqueue_sedsnet_frame(third,sizeof(third),255)==HAL_OK);
 assert(radio_uart_dequeue_frame_with_budget(&first,2000));
 assert(first.len==sizeof(und)); /* Metadata stops original adjacent coalescing. */
 uint64_t ids[4];assert(radio_uart_ack_contracts(first.data,first.len,first.priority,ids)==1);
 g_tx_closed_ack_ids[0]=ids[0];g_tx_closed_ack_count=1;
 g_tx_receipt_tail.bytes=first.len;g_tx_receipt_tail.frames=1;g_tx_receipt_tail.ready=2;
 g_tx_ack_only_admission=2;
 assert(radio_uart_dequeue_frame_with_budget(&second,2000));
 assert(second.len==sizeof(fc)&&memcmp(second.data,fc,sizeof(fc))==0);
 g_tx_ack_only_admission=0;
 assert(radio_uart_ack_contracts(second.data,second.len,second.priority,ids)==1);
 g_tx_closed_ack_ids[1]=ids[0];g_tx_closed_ack_count=2;
 g_tx_receipt_tail.bytes+=second.len;g_tx_receipt_tail.frames=2;
 g_tx_ack_only_admission=2;
 assert(radio_uart_dequeue_frame_with_budget(&second,2000));
 assert(second.len==sizeof(third)&&memcmp(second.data,third,sizeof(third))==0);
 assert(g_tx_count==2 && g_tx_queue_bytes_used==sizeof(a)+sizeof(b));
 assert(memcmp(g_tx_queue_bytes,a,sizeof(a))==0);
 assert(memcmp(g_tx_queue_bytes+sizeof(a),b,sizeof(b))==0);
 return 0;}''')
