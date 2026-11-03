#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef int32_t SedsResult;typedef uint32_t SedsDataType;typedef struct {int marker;} SedsRouter;
#define SEDS_OK 0
#define SEDS_ERR -1
#define SEDS_BAD_ARG -2
#define SEDS_HANDLER_ERROR -3
#define SEDS_IO -9
#define SEDS_P2P_STREAM_ACCEPTED 1U
#define SEDS_P2P_STREAM_DATA 3U
#define SEDS_P2P_STREAM_CLOSED 4U
#define SEDS_P2P_STREAM_RESET 5U
typedef struct {uint32_t ty;const uint8_t*payload;size_t payload_len;const char*sender;size_t sender_len;uint64_t timestamp;} SedsPacketView;
typedef struct {uint8_t kind;uint32_t stream_id,peer_stream_id,sequence;const char*peer_hostname;size_t peer_hostname_len;uint32_t peer_address;uint16_t local_port,peer_port;const uint8_t*payload;size_t payload_len;} SedsP2pStreamEventView;
typedef SedsResult(*SedsP2pStreamHandlerFn)(const SedsP2pStreamEventView*,void*);
SedsResult seds_router_bind_p2p_stream_port(SedsRouter*,uint16_t,SedsP2pStreamHandlerFn,void*);
SedsResult seds_router_send_p2p_stream(SedsRouter*,uint32_t,const uint8_t*,size_t);
SedsResult seds_router_enable_network_variable(SedsRouter*,uint32_t,bool,bool);
SedsResult seds_router_on_network_variable_update(SedsRouter*,uint32_t,SedsResult(*)(const SedsPacketView*,void*),void*);
SedsResult seds_router_request_managed_variable(SedsRouter*,uint32_t);
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*,uint32_t,bool*);
bool seds_router_sender_matches_hostname(const SedsRouter*,const char*,size_t,const char*,size_t);
SedsResult seds_router_get_network_time_ms(SedsRouter*,uint64_t*);
