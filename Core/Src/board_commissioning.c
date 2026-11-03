#include "board_commissioning.h"
#include "commission_record.h"
#include "persistent_store.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include "tx_api.h"
#include <string.h>
enum {COMMISSION_NONE,COMMISSION_IDENTITY,COMMISSION_FLOOR};
/* Boot before tasks; subsequently only the existing SDK/router owner accesses
 * the tuple/history/eligibility. Cross-owner pending is the store atomic hold. */
static uint8_t eligible;
static bool uncertain_first,unverified_floor_first;
static bool history_valid,known_identity,known_floor,repairable_floor,publication_pending,normal_eligible;
static CommissionRecord history;
typedef struct {uint64_t generation,floor_time,floor_issuer;uint8_t kind,marker;bool identity,floor_absent,virgin;CommissionRecord history;} CommissionView;
static bool read_floor_locked(CommissionView*v){
 uint8_t wire[16];size_t size=16;launchcore_persist_strict_info_t info;
 launchcore_persist_status_t status=launchcore_persist_get_strict(UINT32_C(0x4252464c),wire,&size,&info);
 v->floor_absent=status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY||info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN);
 if(v->floor_absent)return true;
 if(status!=LAUNCHCORE_PERSIST_OK||size!=16||!commission_read64(wire)||!commission_read64(wire+8)||commission_read64(wire+8)==UINT64_MAX)return false;
 v->floor_issuer=commission_read64(wire);v->floor_time=commission_read64(wire+8);return true;
}
static bool classify_locked(CommissionView*v){
 *v=(CommissionView){0};uint8_t wire[40];size_t size=40;launchcore_persist_strict_info_t info;
 launchcore_persist_status_t status=launchcore_persist_get_strict(COMMISSION_RECORD_KEY,wire,&size,&info);
 if(status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY||info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN))v->marker=0;
 else if(status==LAUNCHCORE_PERSIST_OK&&commission_record_decode(wire,size,&v->history))v->marker=v->history.state?2:1;
 else return false;
 size=8;status=launchcore_persist_get_strict(UINT32_C(0x42524745),wire,&size,&info);
 bool absent=status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY||info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN);
 v->virgin=absent&&info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN;
 if(!absent){if(status!=LAUNCHCORE_PERSIST_OK||size!=8||!commission_read64(wire)||commission_read64(wire)==UINT64_MAX)return false;v->generation=commission_read64(wire);v->identity=true;}
 if(!read_floor_locked(v))return false;
 if(absent&&(v->marker==0||(v->marker==2&&v->history.kind==0&&v->history.generation==UINT64_MAX)))v->kind=COMMISSION_IDENTITY;
 else if(v->identity&&v->floor_absent&&v->marker==0)v->kind=COMMISSION_FLOOR;
 return true;
}
static uint64_t view_floor(const CommissionView*v){return v->marker==2&&v->history.source>v->floor_time?v->history.source:v->floor_time;}
void board_commissioning_probe_boot(void){
 uncertain_first=false;unverified_floor_first=false;eligible=COMMISSION_NONE;history_valid=known_identity=known_floor=repairable_floor=publication_pending=normal_eligible=false;
 if(!persistent_store_enter())return;
 CommissionView view;bool valid=classify_locked(&view);bool released=persistent_store_leave();
 /* A valid PENDING read records a prior explicit Stop even if a later key is
  * tombstoned/unusable. Retain that Stop; never make the key repairable. */
 if(view.marker){history=view.history;history_valid=true;}
 if(view.marker==2)persistent_store_hold_commissioning();
 if(!valid||!released){persistent_store_note_failure();return;}
 uint64_t current=board_recovery_boot_current();
 known_identity=view.identity&&current&&current!=UINT64_MAX&&view.generation==current;
 known_floor=known_identity&&!view.floor_absent;
 normal_eligible=known_floor&&view.marker!=2;
 repairable_floor=known_identity&&view.floor_absent&&view.marker==2;
 if(view.floor_time&&!view.floor_absent&&!board_recovery_floor_accept_bootstrap(view.floor_time)){persistent_store_note_failure();return;}
 if(view.marker==2&&!board_recovery_floor_note_bootstrap_attempt(history.source)){persistent_store_note_failure();return;}
 if(view.kind==COMMISSION_IDENTITY&&current==0)eligible=COMMISSION_IDENTITY;
 else if(view.kind==COMMISSION_FLOOR&&known_identity)eligible=COMMISSION_FLOOR;
}
bool board_commissioning_pending(void){return persistent_store_commission_pending();}
bool board_commissioning_uncertain_first(void){return uncertain_first&&history_valid&&history.kind==0&&history.generation==UINT64_MAX&&!known_identity&&persistent_store_commission_pending();}
uint64_t board_commissioning_query_generation(void){
 uint64_t generation=board_recovery_boot_current();if(generation==UINT64_MAX||board_commissioning_uncertain_first())return 0;
 if(generation)return generation;
 return eligible==COMMISSION_IDENTITY?UINT64_MAX:0;
}
uint8_t board_commissioning_query_result(void){
 if(board_commissioning_uncertain_first())return BOARD_RECOVERY_RESULT_IO;
 if(!board_commissioning_query_generation())return BOARD_RECOVERY_RESULT_UNSUPPORTED;
 if(persistent_store_controls_ready())return BOARD_RECOVERY_RESULT_OK;
 if(board_commissioning_pending()&&known_identity&&known_floor&&!publication_pending&&persistent_store_recovery_ready())return BOARD_RECOVERY_RESULT_OK;
 return BOARD_RECOVERY_RESULT_IO;
}

enum {JOB_FIRST=1,JOB_SECOND=2,JOB_NORMAL=3,JOB_RECONCILE=4};
static struct {BoardRecoveryRequest request;uint64_t source;void*context;CommissionReply reply;CommissionClock clock;uint32_t applied_epoch;uint8_t kind,phase,result;bool valid,waiting,dirty,owner_started,durable,progress_sent;}job;
static struct {uint64_t nonce,issuer,source,generation;uint32_t epoch;bool valid;}query;
static uint64_t attempted_time;
static bool fresh(uint64_t source,uint64_t now){return source&&source!=UINT64_MAX&&now&&now!=UINT64_MAX&&(now>=source?now-source<=5000U:source-now<=1000U);}
static bool same_tuple(uint64_t issuer,uint64_t nonce,uint64_t other_issuer,uint64_t other_nonce){return issuer==other_issuer&&nonce==other_nonce;}
static bool same_request(const BoardRecoveryRequest*a,const BoardRecoveryRequest*b){return a->operation==b->operation&&a->desired_state==b->desired_state&&a->nonce==b->nonce&&a->issuer_session==b->issuer_session&&a->expected_boot_generation==b->expected_boot_generation&&a->expected_abort_epoch==b->expected_abort_epoch;}
static bool same_history(const BoardRecoveryRequest*r,uint64_t source){return r->operation==BOARD_RECOVERY_RECOVER&&r->desired_state==0&&r->expected_abort_epoch==history.epoch&&r->expected_boot_generation==history.generation&&source==history.source;}
static uint64_t query_cutoff(void){uint64_t cutoff=0;if(!board_recovery_floor_cached_cutoff(&cutoff))return UINT64_MAX;if(history_valid&&history.source>cutoff)cutoff=history.source;if(attempted_time>cutoff)cutoff=attempted_time;return cutoff;}
bool board_commissioning_note_query(const BoardRecoveryRequest*r,uint64_t source,uint64_t now,uint32_t epoch){
 if(!board_commissioning_pending())return true;
 query.valid=false;uint64_t generation=board_recovery_boot_current();bool uncertain=board_commissioning_uncertain_first();
 if(!r||r->operation!=BOARD_RECOVERY_QUERY||!r->nonce||!r->issuer_session||!history_valid||(!uncertain&&(!known_identity||(!known_floor&&!repairable_floor&&!unverified_floor_first)||!generation||generation==UINT64_MAX))||!fresh(source,now)||source<=query_cutoff()||same_tuple(r->issuer_session,r->nonce,history.issuer,history.nonce) )return false;
 BoardRecoverySnapshot live={0};uint8_t required=BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_ABORT_LATCHED;
 if(board_recovery_snapshot(&live)!=0||live.abort_epoch!=epoch||(live.flags&required)!=required||((live.flags&BOARD_RECOVERY_OUTPUTS_VALID)&&!(live.flags&BOARD_RECOVERY_SAFE_OUTPUTS)))return false;
 query.nonce=r->nonce;query.issuer=r->issuer_session;query.source=source;query.generation=generation;query.epoch=epoch;query.valid=true;return true;
}
/* -2 means metadata is unclassified/ineligible, never a legacy repair path.
 * -1 means serialized service owes a reply. Neither is a wire result. */
int32_t board_commissioning_stage(const BoardRecoveryRequest*r,uint64_t source,uint64_t now,uint32_t epoch,void*context,CommissionReply reply,CommissionClock clock){
 if(!r||!reply||!clock||r->operation!=BOARD_RECOVERY_RECOVER||r->desired_state!=0||!r->nonce||!r->issuer_session)return BOARD_RECOVERY_RESULT_INVALID;
 bool duplicate=job.valid&&same_tuple(r->issuer_session,r->nonce,job.request.issuer_session,job.request.nonce);
 if(duplicate&&(!same_request(r,&job.request)||source!=job.source))return BOARD_RECOVERY_RESULT_INVALID;
 if(duplicate){
  if(!fresh(source,now))return BOARD_RECOVERY_RESULT_STALE;
  if(job.waiting)return job.kind==JOB_NORMAL&&job.durable?BOARD_RECOVERY_RESULT_PENDING:-1;
  if(job.result==BOARD_RECOVERY_RESULT_OK&&epoch!=job.applied_epoch)return BOARD_RECOVERY_RESULT_STALE;
  return job.result;
 }
 if(history_valid&&same_tuple(r->issuer_session,r->nonce,history.issuer,history.nonce)){
  if(!same_history(r,source))return BOARD_RECOVERY_RESULT_INVALID;
  return history.kind==0||history.state?BOARD_RECOVERY_RESULT_IO:BOARD_RECOVERY_RESULT_STALE;
 }
 if(!board_commissioning_pending()&&eligible==COMMISSION_NONE&&!normal_eligible)return -2;
 if(job.waiting)return BOARD_RECOVERY_RESULT_IO;
 if(!fresh(source,now)||!epoch||epoch==UINT32_MAX||r->expected_abort_epoch!=epoch||source<=query_cutoff()||(job.valid&&source<=job.source))return BOARD_RECOVERY_RESULT_STALE;
 uint8_t kind=0;uint64_t generation=board_recovery_boot_current();
 if(board_commissioning_uncertain_first()&&r->expected_boot_generation==UINT64_MAX&&query.valid&&!query.generation&&query.epoch==epoch&&fresh(query.source,now)&&r->issuer_session==query.issuer&&!same_tuple(r->issuer_session,r->nonce,query.issuer,query.nonce))kind=JOB_RECONCILE;
 else if(unverified_floor_first&&history_valid&&history.kind==0&&known_identity&&generation&&generation!=UINT64_MAX&&history.generation==generation&&r->expected_boot_generation==generation&&query.valid&&query.generation==generation&&query.epoch==epoch&&fresh(query.source,now)&&r->issuer_session==query.issuer&&!same_tuple(r->issuer_session,r->nonce,query.issuer,query.nonce))kind=JOB_SECOND;
 else if(!board_commissioning_uncertain_first()&&eligible==COMMISSION_IDENTITY&&!generation&&r->expected_boot_generation==UINT64_MAX)kind=JOB_FIRST;
 else if(eligible==COMMISSION_FLOOR&&generation&&generation!=UINT64_MAX&&r->expected_boot_generation==generation)kind=JOB_FIRST;
 else if(normal_eligible&&(!history_valid||history.state==0)&&generation&&generation!=UINT64_MAX&&r->expected_boot_generation==generation)kind=JOB_NORMAL;
 else if(board_commissioning_pending()&&known_identity&&(known_floor||repairable_floor)&&query.valid&&query.generation==generation&&query.epoch==epoch&&fresh(query.source,now)&&r->expected_boot_generation==generation&&r->issuer_session==query.issuer&&!same_tuple(r->issuer_session,r->nonce,query.issuer,query.nonce))kind=history.kind==1?JOB_NORMAL:JOB_SECOND;
 else return BOARD_RECOVERY_RESULT_STALE;
 query.valid=false;
 /* Retire the exact original intent BEFORE state or media effects. Failed jobs
  * require a new explicit nonce/time, never an SDK-ID allocation/restamp. */
 job.request=*r;job.source=source;job.context=context;job.reply=reply;job.clock=clock;job.kind=kind;job.phase=0;job.result=BOARD_RECOVERY_RESULT_IO;job.valid=true;job.waiting=true;job.dirty=false;job.owner_started=false;job.applied_epoch=0;job.durable=false;job.progress_sent=false;attempted_time=source;
 /* Immediate RAM admission exclusion does not claim cold durability before
  * the strict PENDING commit. Concrete owner performs no flash/SDK here. */
 persistent_store_hold_commissioning();job.owner_started=true;
 int32_t prepared=board_recovery_commission_prepare(r->expected_abort_epoch);
 if(prepared!=0){job.waiting=false;job.result=prepared==-10?BOARD_RECOVERY_RESULT_STALE:BOARD_RECOVERY_RESULT_IO;board_recovery_commission_retain();return job.result;}
 job.phase=1;return -1;
}

static bool equal_history(const CommissionRecord*a,const CommissionRecord*b){return a->kind==b->kind&&a->epoch==b->epoch&&a->source==b->source&&a->nonce==b->nonce&&a->issuer==b->issuer&&a->generation==b->generation;}
/* Reclassify after uncertain physical IO only while still owning media. Never
 * infer ABSENT from an error. Published identity is independently strict-read. */
static bool publish_view(const CommissionView*v){
 if(unverified_floor_first&&v->marker==2&&v->history.state==1&&history_valid&&equal_history(&v->history,&history))unverified_floor_first=false;
 if(v->marker){history=v->history;history_valid=true;}
 if(v->marker==2)persistent_store_hold_commissioning();
 uint64_t current=board_recovery_boot_current();
 if(v->identity&&!current&&v->marker==2){if(!board_recovery_boot_publish_commission(v->generation))return false;current=v->generation;}
 known_identity=v->identity&&v->generation==current&&current&&current!=UINT64_MAX;
 if(known_identity)uncertain_first=false;
 known_floor=known_identity&&!v->floor_absent;
 normal_eligible=known_floor&&v->marker!=2;
 repairable_floor=known_identity&&v->floor_absent&&v->marker==2;
 eligible=!current&&v->kind==COMMISSION_IDENTITY?COMMISSION_IDENTITY:known_identity&&v->kind==COMMISSION_FLOOR?COMMISSION_FLOOR:COMMISSION_NONE;
 if(unverified_floor_first)eligible=COMMISSION_NONE;
 return true;
}
static bool write_pending_first(void){
 bool reconcile=job.kind==JOB_RECONCILE;
 if(!board_recovery_commission_held(job.request.expected_abort_epoch)||!board_recovery_floor_note_bootstrap_attempt(job.source))return false;
 persistent_store_hold_commissioning();publication_pending=true;
 CommissionRecord record={.state=1,.kind=0,.epoch=job.request.expected_abort_epoch,.source=job.source,.nonce=job.request.nonce,.issuer=job.request.issuer_session,.generation=job.request.expected_boot_generation};
 bool max_first=job.request.expected_boot_generation==UINT64_MAX&&!board_recovery_boot_current();
 if(max_first&&!history_valid){history=record;history_valid=true;} /* Genuine RAM intent, not a durability claim. */
 uint8_t first_kind=eligible;
 if(!persistent_store_enter())return false;
 if(max_first){uncertain_first=true;eligible=COMMISSION_NONE;}
 CommissionView view;bool valid=classify_locked(&view);uint64_t generation=0;
 bool readonly_identity=false;
 if(valid){
  bool original=history_valid&&history.kind==0&&history.generation==UINT64_MAX;
  if(reconcile){
   valid=original&&max_first&&job.source>history.source&&job.source>view_floor(&view)&&(view.marker==0||(view.marker==2&&equal_history(&view.history,&history)));
   valid=valid&&(view.identity?view.marker==2:view.kind==COMMISSION_IDENTITY);
   readonly_identity=valid&&view.identity;record=history;
  }else{
   bool identity_case=first_kind==COMMISSION_IDENTITY&&view.kind==COMMISSION_IDENTITY&&max_first;
   bool floor_case=first_kind==COMMISSION_FLOOR&&view.kind==COMMISSION_FLOOR&&view.identity&&view.generation==job.request.expected_boot_generation&&view.generation==board_recovery_boot_current();
   valid=(identity_case||floor_case)&&job.source>view_floor(&view);
   if(valid&&view.marker==2){valid=history_valid&&equal_history(&view.history,&history);record=view.history;}
  }
  generation=view.identity?view.generation:1U;
  valid=valid&&board_recovery_commission_held(job.request.expected_abort_epoch);
 }
 uint8_t marker[40],floor[16],identity[8],observed[40];size_t size=40;launchcore_persist_strict_info_t info;
 commission_record_encode(marker,&record);commission_write64(floor,readonly_identity?view.floor_issuer:job.request.issuer_session);commission_write64(floor+8,readonly_identity?view.floor_time:job.source);commission_write64(identity,generation);
 /* A genuine attempt, not verified history. Capture only after the same
  * checked owner classified current identity + healthy trulyABSENT floor and
  * actual Stop/epoch. Retire initial eligibility before uncertain effects. */
 if(valid&&!reconcile&&!max_first&&first_kind==COMMISSION_FLOOR){history=record;history_valid=true;unverified_floor_first=true;}
 eligible=COMMISSION_NONE;
 if(valid&&view.marker==0){valid=(view.virgin?launchcore_persist_provision_virgin_strict(COMMISSION_RECORD_KEY,marker,40,&info):launchcore_persist_set_strict(COMMISSION_RECORD_KEY,marker,40,&info))==LAUNCHCORE_PERSIST_OK;}
 if(valid){valid=launchcore_persist_get_strict(COMMISSION_RECORD_KEY,observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==40&&!memcmp(observed,marker,40);}
 /* Exact verified PENDING + initial strictABSENT floor permits a later fresh
  * SECOND reclassification. This is admission only, never readiness/proof
  * that an uncertain subsequent floor write succeeded or remained absent. */
 if(valid&&!reconcile&&!max_first&&first_kind==COMMISSION_FLOOR){history=record;history_valid=true;repairable_floor=true;}
 if(valid&&!view.identity){valid=board_recovery_commission_held(job.request.expected_abort_epoch)&&launchcore_persist_set_strict(UINT32_C(0x42524745),identity,8,&info)==LAUNCHCORE_PERSIST_OK;}
 if(valid&&!readonly_identity){valid=board_recovery_commission_held(job.request.expected_abort_epoch)&&launchcore_persist_set_strict(UINT32_C(0x4252464c),floor,16,&info)==LAUNCHCORE_PERSIST_OK;}
 if(valid&&(!readonly_identity||!view.floor_absent)){valid=persistent_store_verify_commissioned_locked(generation,floor,marker);}
 CommissionView actual;bool observed_valid=classify_locked(&actual);bool released=persistent_store_leave();
 if(!released){persistent_store_note_failure();return false;}
 if(observed_valid&&(uncertain_first||unverified_floor_first)&&actual.marker&&(!history_valid||actual.history.state!=1||!equal_history(&actual.history,&history)))observed_valid=false;
 if(observed_valid&&!publish_view(&actual))valid=false;
 if(!valid||!observed_valid){persistent_store_note_failure();return false;}
 if(!board_recovery_boot_publish_commission(generation))return false;
 if((!readonly_identity||!view.floor_absent)&&!board_recovery_floor_accept_bootstrap(readonly_identity?view.floor_time:job.source))return false;
 known_identity=true;known_floor=!readonly_identity||!view.floor_absent;repairable_floor=readonly_identity&&view.floor_absent;eligible=COMMISSION_NONE;publication_pending=false;uncertain_first=false;
 return board_recovery_commission_held(job.request.expected_abort_epoch);
}
/* Exact Pending readback before any newer floor. Both banks/CRC/ancestry
 * remain the public strict reader's responsibility under this same owner. */
static bool readback_pending_locked(const uint8_t expected[40]){
 uint8_t observed[40];size_t size=sizeof observed;launchcore_persist_strict_info_t info;
 return launchcore_persist_get_strict(COMMISSION_RECORD_KEY,observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==40&&!memcmp(observed,expected,40);
}
static bool write_second_floor(void){
 uint64_t admitted=0,verified=0;if(!board_recovery_floor_cached_cutoff(&admitted))return false;
 bool verified_known=board_recovery_floor_read_cutoff(&verified)!=0;
 if(!board_recovery_commission_held(job.request.expected_abort_epoch)||!board_recovery_floor_note_bootstrap_attempt(job.source))return false;
 publication_pending=true;
 if(!persistent_store_enter())return false;
 CommissionView view;bool classified=classify_locked(&view);
 bool bound=classified&&view.identity&&view.generation==job.request.expected_boot_generation&&view.generation==board_recovery_boot_current()&&history_valid&&job.source>view_floor(&view)&&board_recovery_commission_held(job.request.expected_abort_epoch);
 /* Exact PRESENT Pending may finish under this new intent. If BOTH keys are
  * trulyABSENT/HEALTHY (classify_locked's strict predicates), no durable first
  * leg existed: persist original attempted bytes only as HISTORY with the NEW
  * authorized floor, return IO/hold and require another fresh explicit leg.
  * Tombstones, unknown health and mismatched lineage never enter either path. */
 bool first_only=bound&&unverified_floor_first&&history.kind==0&&history.state==1&&history.generation==view.generation&&view.kind==COMMISSION_FLOOR&&view.marker==0&&view.floor_absent;
 bool valid=bound&&(first_only||(view.marker&&equal_history(&view.history,&history)&&(!unverified_floor_first||(view.marker==2&&view.history.state==1&&history.state==1))));
 if(first_only)view.history=history;
 if(valid&&!view.floor_absent)valid=(view.floor_time==history.source&&view.floor_issuer==history.issuer)||(verified_known&&view.floor_time==verified)||(admitted>history.source&&view.floor_time==admitted);
 /* A completed earlier application is not Pending for this NEW intent.
  * Preserve original immutable history while reopening its lifecycle before
  * publishing a newer durable floor. Actual desired comes from the fresh job. */
 view.history.state=1U;
 uint8_t floor[16],marker[40];commission_write64(floor,job.request.issuer_session);commission_write64(floor+8,job.source);commission_record_encode(marker,&view.history);launchcore_persist_strict_info_t info;
 if(valid&&(first_only||view.marker==1))valid=board_recovery_commission_held(job.request.expected_abort_epoch)&&launchcore_persist_set_strict(COMMISSION_RECORD_KEY,marker,40,&info)==LAUNCHCORE_PERSIST_OK&&readback_pending_locked(marker);
 if(valid){valid=board_recovery_commission_held(job.request.expected_abort_epoch)&&launchcore_persist_set_strict(UINT32_C(0x4252464c),floor,16,&info)==LAUNCHCORE_PERSIST_OK&&persistent_store_verify_commissioned_locked(view.generation,floor,marker);}
 bool released=persistent_store_leave();
 if(!valid||!released){persistent_store_note_failure();return false;}
 history=view.history;
 if(!board_recovery_floor_accept_bootstrap(job.source)){return false;}
 known_floor=true;repairable_floor=false;publication_pending=false;unverified_floor_first=false;eligible=COMMISSION_NONE;
 if(!board_recovery_commission_held(job.request.expected_abort_epoch))return false;
 if(first_only)job.kind=JOB_FIRST; /* IO-only history leg; never enter apply. */
 return true;
}
static bool write_complete(void){
 uint32_t actual_epoch=0;
 if(!board_recovery_commission_applied(job.request.expected_abort_epoch,&actual_epoch)||!persistent_store_enter())return false;
 CommissionView view;bool valid=classify_locked(&view)&&view.identity&&view.generation==job.request.expected_boot_generation&&view.generation==board_recovery_boot_current()&&view.marker&&history_valid&&equal_history(&view.history,&history)&&!view.floor_absent&&view.floor_time==job.source&&view.floor_issuer==job.request.issuer_session&&board_recovery_commission_applied(job.request.expected_abort_epoch,&actual_epoch);
 CommissionRecord complete=history;complete.state=0;uint8_t marker[40],floor[16];commission_record_encode(marker,&complete);commission_write64(floor,job.request.issuer_session);commission_write64(floor+8,job.source);launchcore_persist_strict_info_t info;
 if(valid&&view.marker==2){valid=launchcore_persist_set_strict(COMMISSION_RECORD_KEY,marker,40,&info)==LAUNCHCORE_PERSIST_OK;}
 if(valid){valid=persistent_store_verify_commissioned_locked(view.generation,floor,marker);}
 bool released=persistent_store_leave();
 if(!valid||!released){persistent_store_note_failure();return false;}
 history=complete;normal_eligible=true;uint64_t cutoff=0;
 if(!board_recovery_floor_cached_cutoff(&cutoff)||cutoff!=job.source){publication_pending=true;return false;}
 if(!board_recovery_commission_finish(job.request.expected_abort_epoch,actual_epoch)){return false;}
 job.applied_epoch=actual_epoch;publication_pending=false;eligible=COMMISSION_NONE;return true;
}


/* Normal healthy recovery keeps one-request behavior. Its own exact intent is
 * durable before floor/FSTATE, after actual owned Stop/quiescence. No critical
 * ABSENT/deleted key can enter this path, including explicit IO reconciliation. */
static bool write_pending_normal(void){
 if(!board_recovery_commission_held(job.request.expected_abort_epoch)||!board_recovery_floor_note_bootstrap_attempt(job.source))return false;
 publication_pending=true;
 if(!persistent_store_enter())return false;
 CommissionView view;bool valid=classify_locked(&view)&&view.identity&&!view.floor_absent&&view.generation==job.request.expected_boot_generation&&view.generation==board_recovery_boot_current()&&job.source>view_floor(&view)&&board_recovery_commission_held(job.request.expected_abort_epoch);
 if(valid&&view.marker==2)valid=history_valid&&view.history.kind==1&&equal_history(&view.history,&history);
 CommissionRecord record={.state=1,.kind=1,.epoch=job.request.expected_abort_epoch,.source=job.source,.nonce=job.request.nonce,.issuer=job.request.issuer_session,.generation=job.request.expected_boot_generation};
 uint8_t marker[40],floor[16],observed[40];commission_record_encode(marker,&record);commission_write64(floor,job.request.issuer_session);commission_write64(floor+8,job.source);size_t size=40;launchcore_persist_strict_info_t info;
 if(valid){valid=launchcore_persist_set_strict(COMMISSION_RECORD_KEY,marker,40,&info)==LAUNCHCORE_PERSIST_OK;}
 if(valid){valid=launchcore_persist_get_strict(COMMISSION_RECORD_KEY,observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==40&&!memcmp(observed,marker,40);if(valid)job.durable=true;}
 if(valid){valid=board_recovery_commission_held(job.request.expected_abort_epoch)&&launchcore_persist_set_strict(UINT32_C(0x4252464c),floor,16,&info)==LAUNCHCORE_PERSIST_OK&&persistent_store_verify_commissioned_locked(view.generation,floor,marker);}
 CommissionView actual;bool observed_valid=classify_locked(&actual);bool released=persistent_store_leave();
 if(!released){persistent_store_note_failure();return false;}
 if(observed_valid&&!publish_view(&actual))valid=false;
 if(!valid||!observed_valid){persistent_store_note_failure();return false;}
 if(!board_recovery_floor_accept_bootstrap(job.source))return false;
 known_identity=known_floor=true;repairable_floor=false;eligible=COMMISSION_NONE;publication_pending=false;
 return board_recovery_commission_held(job.request.expected_abort_epoch);
}

static bool reply_delayed;static uint32_t reply_tick;
static void finish_job(uint8_t result,bool retain){
 if(retain&&job.owner_started)board_recovery_commission_retain();
 job.waiting=false;job.dirty=true;job.result=result;reply_delayed=false;
}
void board_commissioning_service(void){
 if(job.valid&&job.waiting){
  uint64_t now=job.clock(job.context);
  if(!fresh(job.source,now)){finish_job(BOARD_RECOVERY_RESULT_STALE,true);}
  else{
   BoardRecoverySnapshot snapshot={0};
   if(board_recovery_snapshot(&snapshot)!=0){finish_job(BOARD_RECOVERY_RESULT_IO,true);}
   else if(job.phase<2&&snapshot.abort_epoch!=job.request.expected_abort_epoch){finish_job(BOARD_RECOVERY_RESULT_STALE,true);}
   else{
    if(job.waiting&&job.phase==1&&board_recovery_commission_held(job.request.expected_abort_epoch)){
     if(job.kind==JOB_FIRST||job.kind==JOB_RECONCILE){bool stored=write_pending_first();finish_job(BOARD_RECOVERY_RESULT_IO,!stored);}
     else if(!(job.kind==JOB_NORMAL?write_pending_normal():write_second_floor())){finish_job(BOARD_RECOVERY_RESULT_IO,true);}
     else if(job.kind==JOB_FIRST)finish_job(BOARD_RECOVERY_RESULT_IO,false);
     else job.phase=2;
    }
    if(job.waiting&&job.phase==2){
     int32_t applied=board_recovery_commission_apply(job.request.expected_abort_epoch);
     if(applied!=-14){
      if(applied!=0){finish_job(applied==-10?BOARD_RECOVERY_RESULT_STALE:BOARD_RECOVERY_RESULT_IO,true);}
      else if(!fresh(job.source,job.clock(job.context))||!write_complete()){finish_job(BOARD_RECOVERY_RESULT_IO,true);}
      else finish_job(BOARD_RECOVERY_RESULT_OK,false);
     }
    }
   }
  }
 }
 if(job.valid&&job.waiting&&job.kind==JOB_NORMAL&&job.durable&&!job.progress_sent&&job.reply){
  if(job.reply(job.context,&job.request,BOARD_RECOVERY_RESULT_PENDING))job.progress_sent=true;
 }
 if(job.valid&&job.dirty&&job.reply){
  uint32_t tick=(uint32_t)tx_time_get();
  if(reply_delayed&&(uint32_t)(tick-reply_tick)<(TX_TIMER_TICKS_PER_SECOND+9U)/10U)return;
  uint8_t result=job.result;BoardRecoverySnapshot snapshot={0};
  if(result==BOARD_RECOVERY_RESULT_OK&&(board_recovery_snapshot(&snapshot)!=0||snapshot.abort_epoch!=job.applied_epoch||!persistent_store_controls_ready()))result=BOARD_RECOVERY_RESULT_STALE;
  if(!job.reply(job.context,&job.request,result)){reply_delayed=true;reply_tick=tick;return;}
  job.dirty=false;reply_delayed=false;
 }
}
