#include "board_recovery_wire.h"
static uint64_t read_le64(const uint8_t *p) {
 uint64_t value=0U; for (unsigned i=0U;i<8U;i++) value|=(uint64_t)p[i]<<(8U*i); return value;
}
static uint32_t read_le32(const uint8_t *p) {
 uint32_t value=0U; for (unsigned i=0U;i<4U;i++) value|=(uint32_t)p[i]<<(8U*i); return value;
}
static void write_le64(uint8_t *p,uint64_t value) {
 for (unsigned i=0U;i<8U;i++) p[i]=(uint8_t)(value>>(8U*i));
}
static void write_le32(uint8_t *p,uint32_t value) {
 for (unsigned i=0U;i<4U;i++) p[i]=(uint8_t)(value>>(8U*i));
}
uint8_t board_recovery_decode_request(const uint8_t *wire,size_t length,BoardRecoveryRequest *request) {
 if (wire==0 || request==0 || length!=BOARD_RECOVERY_REQUEST_SIZE ||
     wire[0]!=BOARD_RECOVERY_VERSION || wire[1]>BOARD_RECOVERY_RECOVER || wire[3]!=0U) return 0U;
 BoardRecoveryRequest parsed;
 parsed.operation=wire[1]; parsed.desired_state=wire[2];
 parsed.nonce=read_le64(wire+4U); parsed.expected_boot_generation=read_le64(wire+12U);
 parsed.expected_abort_epoch=read_le32(wire+20U); parsed.issuer_session=read_le64(wire+24U);
 if (parsed.nonce==0U || parsed.issuer_session==0U) return 0U;
 if (parsed.operation==BOARD_RECOVERY_RECOVER &&
     (parsed.desired_state>1U || parsed.expected_boot_generation==0U || parsed.expected_abort_epoch==0U)) return 0U;
 *request=parsed; return 1U;
}
void board_recovery_encode_status(uint8_t wire[BOARD_RECOVERY_STATUS_SIZE],
 uint8_t result,uint8_t actual_state,uint8_t flags,const BoardRecoveryRequest *request,
 uint64_t boot_generation,uint32_t abort_epoch,uint32_t uptime_ms,uint32_t commanded_outputs) {
 wire[0]=BOARD_RECOVERY_VERSION; wire[1]=result; wire[2]=actual_state; wire[3]=flags;
 write_le64(wire+4U,request->nonce); write_le64(wire+12U,boot_generation);
 write_le32(wire+20U,abort_epoch); write_le32(wire+24U,uptime_ms);
 write_le32(wire+28U,commanded_outputs); write_le64(wire+32U,request->issuer_session);
}
