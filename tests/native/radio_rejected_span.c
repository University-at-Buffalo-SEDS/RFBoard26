#include "recovery_mesh.h"
#include "radio_crc_reference.h"
static const uint8_t read0[]={166,91,17,0,82,82,1,0,42,0,0,0,0,0,0,0,1,64,231,180,23};
static const uint8_t read1[]={166,91,95,0,82,82,1};
static const uint8_t read2[]={166,91,35,0,82,82,1,5,202,1,228,226,173,106,1,0,243,133,2,151,215,142,162,13,132,154,150,127,132,0,0,154,122,231,80,121,153,23,218};
static const uint8_t read3[]={166,91,95,0,82,82,1,5,136,1,233,238,158,90,64,0,181,53,151,215,142,162,13,249,140,173,91,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,191,15,226,135,31,221,116,198};
static const uint8_t read4[]={166,91,34,0,82,82,1,13,1,3,42,0,0,0,0,0,0,0,235,202,61,113,185,127,150,78,71,144,89,38,87,181,227,172,60,197,186,162};
static const uint8_t damaged[]={166,91,128,0,82,82,1,6,51,51,51,51,51,51,51,51,166,91,0,4,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,135,55,212,181,166,91,35,0,82,82,1,5,202,1,228,226,173,106,1,0,243,133,2,151,215,142,162,13,132,154,150,127,132,0,0,154,122,231,80,121,153,23,218};

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define RADIO_UART_FRAME_BUF_SIZE 1028U
#define RADIO_UART_MAX_PAYLOAD_SIZE 1024U
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_FRAME_SYNC_0 0xA5
#define RADIO_UART_FRAME_SYNC_1 0x5A
#define RADIO_UART_COMMAND_SYNC_0 0xA6
#define RADIO_UART_COMMAND_SYNC_1 0x5B
#define RADIO_UART_ASCII_SYNC_0 0xA7
#define RADIO_UART_ASCII_SYNC_1 0x7A
#define RADIO_PARTIAL_FRAME_TIMEOUT_MS 2000U
static uint8_t g_frame_buf[1028],g_current_rx_is_command_frame,g_last_frame_kind;
static uint8_t g_last_frame_preview[16],g_last_frame_preview_len,g_partial_frame_waiting;
static size_t g_frame_len,g_last_frame_payload_len;
static uint32_t g_partial_frame_started_ms,g_rx_sync_loss,g_rx_bad_len,g_radio_rx_frames_ok;
static uint32_t mock_now,received;static unsigned expected_command;
static const uint8_t* expected[2];static size_t expected_len[2];
static uint32_t radio_now_ms(void){return mock_now;}
static void radio_uart_store_preview(uint8_t*a,uint8_t*b,const uint8_t*c,size_t n){(void)a;(void)b;(void)c;(void)n;}
static void radio_notify_rx(const uint8_t*p,size_t n){
 assert(received<2);assert(n==expected_len[received]);assert(!memcmp(p,expected[received],n));assert(g_current_rx_is_command_frame==expected_command);++received;
}
/* ACTUAL_PARSER_UNDER_TEST */
static void reset(void){memset(&g_rejected_frame,0,sizeof(g_rejected_frame));g_frame_len=0;g_partial_frame_waiting=0;mock_now=0;received=0;g_rx_crc_errors=0;g_rx_bad_len=0;}
static void additional_regressions(void);
int main(void){
 expected[0]=read2+8;expected_len[0]=sizeof(read2)-12;expected[1]=read3+8;expected_len[1]=sizeof(read3)-12;
 for(unsigned chunk=1;chunk<=204;++chunk){
  reset();const uint8_t*reads[]={read0,read1,read2,read3,read4};const size_t sizes[]={sizeof(read0),sizeof(read1),sizeof(read2),sizeof(read3),sizeof(read4)};const uint32_t times[]={0,65,90,200,395};
  for(unsigned r=0;r<5;++r){
   while(mock_now+5<times[r]){mock_now+=5;radio_process_buffered_frames();}
   mock_now=times[r];
   for(size_t at=0;at<sizes[r];){size_t take=sizes[r]-at;if(take>chunk)take=chunk;radio_process_framed_bytes(reads[r]+at,take);at+=take;}
  }
  assert(received==2&&g_rx_crc_errors==1&&g_frame_len==0);
 }
 puts("actual_7B_prefix_distinct_command_retry: exact2 callbacks, expected1CRCfailure, tail0, chunks1..204");
 reset();expected[0]=read2+8;expected_len[0]=sizeof(read2)-12;
 radio_process_framed_bytes(damaged,sizeof(damaged));
 assert(g_rx_crc_errors==1); /* Real CRC failure established before false sync. */
#ifndef CHARACTERIZE_ORIGINAL_TIMEOUT
 for(unsigned poll=1;poll<=8&&received==0;++poll){mock_now=poll;radio_process_buffered_frames();}
 assert(mock_now<=8);
 if(received!=1){fprintf(stderr,"EXPECTED_OLD_NEGATIVE: provenCRCfailed parent, complete distinctRR command buffered but callback=%u pending=%zu\n",received,g_frame_len);return 23;}
#else
 assert(received==0);mock_now=1999;radio_process_buffered_frames();assert(received==0);
 mock_now=2000;radio_process_buffered_frames();assert(received==1&&g_frame_len==0);
 puts("known_bad_parent_false_inner_header: waits1999ms, exact recovery2000ms; no permanent loss claim");
#endif
 additional_regressions();
 return 0;
}

static void additional_regressions(void){
 /* Unknown incomplete parent MUST retain original timeout, including wrap. */
 reset();expected_command=0;expected[0]=read2+8;expected_len[0]=sizeof(read2)-12;
 const uint8_t unknown[]={0xa6,0x5b,0,4};
 mock_now=UINT32_MAX-500U;radio_process_framed_bytes(unknown,sizeof(unknown));radio_process_framed_bytes(read2,sizeof(read2));
 assert(received==0&&g_rejected_frame.remaining==0);
 mock_now+=1999;radio_process_buffered_frames();assert(received==0);
 ++mock_now;radio_process_buffered_frames();assert(received==1);
 /* CRC-valid MAX parent containing a fully valid nested RR frame is ONE outer
  * payload, never an excuse to steal its nested bytes. */
 reset();uint8_t maximum[1028];maximum[0]=0xa5;maximum[1]=0x5a;maximum[2]=0;maximum[3]=4;
 memset(maximum+4,0x44,1020);memcpy(maximum+100,read2,sizeof(read2));uint32_t crc=launchcore_crc32(maximum+4,1020);
 for(unsigned n=0;n<4;++n)maximum[1024+n]=(uint8_t)(crc>>(8*n));
 expected[0]=maximum+4;expected_len[0]=1024;
 radio_process_framed_bytes(maximum,400);mock_now=1999;radio_process_buffered_frames();assert(received==0);
 radio_process_framed_bytes(maximum+400,sizeof(maximum)-400);assert(received==1&&g_frame_len==0);
 /* Admission permission ends after a validated frame. The next unrelated
  * incomplete parent cannot inherit scanning privilege. */
 reset();expected[0]=read2+8;expected_len[0]=sizeof(read2)-12;
 radio_process_framed_bytes(damaged,sizeof(damaged));
 for(unsigned n=0;n<8&&received==0;++n)radio_process_buffered_frames();
 assert(received==1&&g_rejected_frame.remaining==0);
 radio_process_framed_bytes(unknown,sizeof(unknown));radio_process_framed_bytes(read2,sizeof(read2));assert(received==1);
 mock_now=1999;radio_process_buffered_frames();assert(received==1);
 mock_now=2000;expected[1]=read2+8;expected_len[1]=sizeof(read2)-12;radio_process_buffered_frames();assert(received==2);
 /* An unfinished scan keeps progressing even while new bytes arrive. */
 reset();expected[0]=read2+8;expected_len[0]=sizeof(read2)-12;
 radio_process_framed_bytes(damaged,sizeof(damaged)-sizeof(read2));
 for(size_t n=0;n<sizeof(read2);++n){radio_process_framed_bytes(read2+n,1);radio_process_buffered_frames();}
 for(unsigned n=0;n<16&&received==0;++n)radio_process_buffered_frames();
 assert(received==1&&g_frame_len==0);
 /* Work is bounded and an insufficient CRC remainder yields BEFORE candidate.
  * Fresh MAX budget always fits the largest possible complete later anchor.
  * This direct helper seam proves work progress, not UART ownership. */
 reset();memcpy(g_frame_buf+1,maximum,sizeof(maximum)-1); /* replace with complete nearMAX frame */
 const size_t len=1019;g_frame_buf[1]=0xa5;g_frame_buf[2]=0x5a;g_frame_buf[3]=(uint8_t)len;g_frame_buf[4]=(uint8_t)(len>>8);
 memset(g_frame_buf+5,0x33,len-4);crc=launchcore_crc32(g_frame_buf+5,len-4);
 for(unsigned n=0;n<4;++n)g_frame_buf[5+len-4+n]=(uint8_t)(crc>>(8*n));
 g_frame_len=1+4+len;g_rejected_frame.remaining=1;g_rejected_frame.cursor=1;g_rejected_frame.epoch_length=g_frame_len;g_rejected_frame.retry_from=UINT16_MAX;
 unsigned steps=64;size_t bytes=10;assert(radio_rejected_anchor(&steps,&bytes)==0&&g_rejected_frame.cursor==1);
 steps=64;bytes=RADIO_UART_MAX_PAYLOAD_SIZE;assert(radio_rejected_anchor(&steps,&bytes)==1&&bytes==1024-(len-4));
 /* Consuming/mutating an equally long buffer resets cursor/cache. */
 g_rejected_frame.remaining=300;g_rejected_frame.cursor=200;g_rejected_frame.epoch_length=g_frame_len;g_rejected_frame.retry_from=77;
 radio_frame_buf_consume(5);assert(g_rejected_frame.remaining==295&&g_rejected_frame.cursor==1&&g_rejected_frame.retry_from==UINT16_MAX);
 const uint8_t noise[5]={0};radio_frame_buf_append(noise,5);assert(g_frame_len==1024);
 /* Overflow rebase and full replacement obey original bounds, clearing old
  * permission. Do not promise arbitrary-corruption losslessness. */
 radio_frame_buf_append(noise,5);assert(g_frame_len==1028&&g_rejected_frame.remaining==294);
 uint8_t replacement[1028]={0};radio_frame_buf_append(replacement,sizeof(replacement));assert(g_frame_len==1028&&g_rejected_frame.remaining==0);
 radio_process_buffered_frames();assert(!g_frame_len&&!g_rejected_frame.remaining);
 /* A scan consumes <=64 positions, then resumes on unchanged data. */
 reset();memset(g_frame_buf,0x11,400);g_frame_len=400;g_rejected_frame.remaining=399;g_rejected_frame.cursor=1;g_rejected_frame.epoch_length=400;g_rejected_frame.retry_from=UINT16_MAX;
 steps=64;bytes=1024;assert(!radio_rejected_anchor(&steps,&bytes)&&steps==0&&g_rejected_frame.cursor==65);
 steps=64;bytes=1024;assert(!radio_rejected_anchor(&steps,&bytes)&&g_rejected_frame.cursor==129);


 /* Legacy non-CRC RS and ASCII continue using their original contracts. */
 reset();const uint8_t scheduler[]={0xa6,0x5b,7,0,'R','S',1,1,0,0,1};
 expected_command=1;expected[0]=scheduler+4;expected_len[0]=sizeof(scheduler)-4;
 radio_process_framed_bytes(scheduler,sizeof(scheduler));assert(received==1);
 reset();const uint8_t ascii[]={0xa7,0x7a,3,0,'A','B','C'};
 expected_command=0;expected[0]=ascii+4;expected_len[0]=3;
 radio_process_framed_bytes(ascii,sizeof(ascii));assert(received==1);
 /* Multiple expensive CRC candidates exhaust remaining work, but each fresh
  * poll fits one MAX-sized payload and reaches the later validated anchor. */
 reset();memset(g_frame_buf,0x11,sizeof(g_frame_buf));g_frame_len=1028;
 const size_t offsets[]={1,10};const size_t lengths[]={800,1010};
 for(unsigned n=0;n<2;++n){size_t at=offsets[n],length=lengths[n];g_frame_buf[at]=0xa6;g_frame_buf[at+1]=0x5b;g_frame_buf[at+2]=(uint8_t)length;g_frame_buf[at+3]=(uint8_t)(length>>8);}
 memcpy(g_frame_buf+20,read2,sizeof(read2));
 g_rejected_frame.remaining=1000;g_rejected_frame.cursor=1;g_rejected_frame.epoch_length=1028;g_rejected_frame.retry_from=UINT16_MAX;
 steps=64;bytes=1024;assert(!radio_rejected_anchor(&steps,&bytes)&&g_rejected_frame.cursor==10);
 steps=64;bytes=1024;assert(!radio_rejected_anchor(&steps,&bytes)&&g_rejected_frame.cursor==20);
 steps=64;bytes=1024;assert(radio_rejected_anchor(&steps,&bytes)==20);
 puts("MAX/nested/unexpired/wrap/lifecycle/growth/rebase/overflow/budget-resume controls PASS");
}
