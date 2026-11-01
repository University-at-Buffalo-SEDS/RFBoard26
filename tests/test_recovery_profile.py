"""Compile the production profile codec, queued driver and RX consumer."""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest
import zlib

from checked_router_fixture import checked_hooks
from router_lifecycle_fixture import function as actual_function
ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def envelope(body):
    data = b"RR\x01\x0e" + body
    return data + zlib.crc32(data).to_bytes(4, "little")


def array(name, data):
    return f"static const uint8_t {name}[]={{" + ",".join(map(str, data)) + "};\n"


class RecoveryProfile(unittest.TestCase):
    def test_codec_driver_and_sdk_isolation(self):
        nonce = bytes(range(1, 17))
        query = envelope(b"\x01\x00" + nonce)
        reply = envelope(b"\x01\x01" + nonce + (115200).to_bytes(4, "little")
                         + (32000).to_bytes(4, "little") + b"\x23"
                         + (16).to_bytes(2, "little") + (100).to_bytes(2, "little")
                         + (55).to_bytes(2, "little") + b"\x00")
        self.assertEqual((len(query), len(reply)), (26, 42))
        source = (ROOT / "Core/Src/radio.c").read_text()
        start = source.index("/* Profile control never")
        driver = source[start:source.index("HAL_StatusTypeDef radio_uart_send_bytes", start)]
        telemetry=(ROOT/'Core/Src/telemetry.c').read_text()
        consumer='\n'.join(actual_function(telemetry,n)for n in ('telemetry_radio_rx_locked','telemetry_radio_rx'))
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "recovery_mesh.h"
#define RFBOARD_LORA_SCHEDULER 0
#define RFBOARD_RADIO_LISTEN_ONLY 0
#define RADIO_AIR_BIT_RATE_BPS 32000U
#define RADIO_AIR_SHARE_PERCENT 35U
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#define RADIO_PACKET_BURST_MS 100U
#define RADIO_PACKET_GAP_MS 55U
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_COMMAND_SYNC_0 0xa6U
#define RADIO_UART_COMMAND_SYNC_1 0x5bU
#define UART_HWCONTROL_NONE 0U
#define UART_WORDLENGTH_8B 8U
#define UART_STOPBITS_1 1U
#define UART_PARITY_NONE 0U
#define HAL_OK 0U
#define HAL_BUSY 2U
#define TELEMETRY_ENABLED 1
#define COMMAND_TRACE_PRINTS 0
#define SEDS_OK 0
typedef unsigned HAL_StatusTypeDef;
typedef struct { struct {uint32_t BaudRate,HwFlowCtl,WordLength,StopBits,Parity;} Init; } UART_HandleTypeDef;
static UART_HandleTypeDef uart={{115200,0,8,1,0}}, *g_huart=&uart;
static uint32_t now, offered, accepted, ready=1, enqueue_status=HAL_OK;
static uint8_t captured[46];
static uint32_t HAL_GetTick(void) {return now;}
static uint8_t radio_uart_tx_ready(void) {return (uint8_t)ready;}
static HAL_StatusTypeDef radio_uart_enqueue_frame(const uint8_t *p,uint16_t n,int16_t priority) {
  ++offered; assert(n==sizeof(captured) && priority==253);
  assert(p[0]==0xa6 && p[1]==0x5b && p[2]==42 && p[3]==0);
  if(enqueue_status==HAL_OK) {memcpy(captured,p,n);++accepted;}
  return enqueue_status;
}
static uint8_t command_frame=1, g_radio_link_seen, g_telemetry_discovery_seen;
static unsigned observed,routed,initialized;
static struct {void *r;} g_router={(void*)1};
static int g_radio_side_id=0;
static uint8_t radio_uart_current_rx_is_command_frame(void) {return command_frame;}
static void sim_probe_observe_packed(const uint8_t *p,size_t n) {(void)p;(void)n;++observed;}
static int init_telemetry_router(void) {++initialized;return SEDS_OK;}
static int seds_router_receive_packed_from_side(void *r,uint32_t s,const uint8_t *p,size_t n) {
  (void)r;(void)s;(void)p;(void)n;++routed;return 0;
}
static int seds_router_receive_packed(void *r,const uint8_t *p,size_t n) {
  return seds_router_receive_packed_from_side(r,0,p,n);
}
'''
        code += array("query", query) + array("expected", reply) + driver +'\nstatic unsigned locked;\nvoid telemetry_lock(void){assert(!locked);++locked;}\nvoid telemetry_unlock(void){assert(locked==1);--locked;}\n'+checked_hooks(ROOT)+'\n'+consumer
        code += r'''
int main(void) {
  const recovery_profile_config_t config={115200,32000,35,16,100,55,0};
  uint8_t out[42],bad[27],zero[26];
  assert(recovery_profile_build_reply(query,sizeof(query),1,&config,out));
  assert(!memcmp(out,expected,sizeof(out)));
  assert(!recovery_profile_build_reply(NULL,26,1,&config,out));
  assert(!recovery_profile_build_reply(query,26,0,&config,out));
  assert(!recovery_profile_build_reply(query,26,1,NULL,out));
  assert(!recovery_profile_build_reply(query,26,1,&config,NULL));
  for(size_t n=0;n<26;++n) assert(!recovery_profile_is_query(query,n,1));
  memcpy(bad,query,26);bad[26]=0;assert(!recovery_profile_is_query(bad,27,1));
  for(size_t i=0;i<26;++i) for(unsigned b=0;b<8;++b) {
    memcpy(bad,query,26);bad[i]^=(uint8_t)(1U<<b);
    memset(out,0xcc,sizeof(out));assert(!recovery_profile_build_reply(bad,26,1,&config,out));
    for(size_t j=0;j<sizeof(out);++j) assert(out[j]==0xcc);
  }
  memcpy(zero,query,26);memset(zero+6,0,16);
  recovery_profile_write_u32(zero+22,recovery_profile_crc32(zero,22));
  assert(!recovery_profile_is_query(zero,26,1));
  assert(!recovery_profile_is_query(expected,sizeof(expected),1));
  const uint8_t *p=query;size_t n=sizeof(query);uint8_t command=1;
  assert(recovery_mesh_uplink(&p,&n,&command));
  assert(p==query && n==26 && command==1);
  recovery_profile_config_t mismatch=config;mismatch.rf_share_percent=40;
  assert(recovery_profile_build_reply(query,26,1,&mismatch,out) && out[30]==40);
  mismatch=config;mismatch.flags=4;
  assert(recovery_profile_build_reply(query,26,1,&mismatch,out) && out[37]==4);
  mismatch.flags=8;assert(!recovery_profile_build_reply(query,26,1,&mismatch,out));
  telemetry_radio_rx(query,sizeof(query),NULL);
  assert(accepted==1 && !memcmp(captured+4,expected,42));
  assert(!observed && !routed && !initialized && !g_radio_link_seen && !g_telemetry_discovery_seen);
  now=499;telemetry_radio_rx(query,sizeof(query),NULL);assert(offered==1);
  now=500;enqueue_status=HAL_BUSY;telemetry_radio_rx(query,sizeof(query),NULL);assert(offered==2 && accepted==1);
  now=501;enqueue_status=HAL_OK;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==2);
  now=1000;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==2);
  now=1001;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==3);
  now=1501;ready=0;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==3);
  ready=1;g_huart=NULL;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==3);
  g_huart=&uart;now=UINT32_MAX-100;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==4);
  now=398;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==4);
  now=399;telemetry_radio_rx(query,sizeof(query),NULL);assert(accepted==5);
  const uint8_t ordinary[]={0x4a,1,0};command_frame=0;
  telemetry_radio_rx(ordinary,sizeof(ordinary),NULL);
  assert(observed==1 && routed==1 && g_radio_link_seen && g_telemetry_discovery_seen);
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "profile.c").write_text(code)
            subprocess.run(["cc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-fno-pie", "-no-pie",
                            "-I", str(ROOT / "Core/Inc"), str(path / "profile.c"),
                            "-o", str(path / "profile")], check=True)
            subprocess.run([str(path / "profile")], check=True)


if __name__ == "__main__":
    unittest.main()
