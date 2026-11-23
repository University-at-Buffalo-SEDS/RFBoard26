#include "can_tx_queue.h"
#include "main.h"
#include <string.h>
#include "board_clock_us.h"

#ifndef CAN_TX_QUEUE_BUDGET
#define CAN_TX_QUEUE_BUDGET 6144U
#endif
#define CAN_TX_MAX_PACKET CAN_LINK_MAX_PACKET
#define CAN_TX_MAX_PENDING 48U
#define CAN_TX_MAX_AGE_MS 1000U
#define CAN_TX_PUMP_FRAMES 3U

/* These hooks are foreground-only and preserve allocator emergency headroom. */
extern void *telemetry_can_tx_allocate_priority(size_t bytes, uint8_t protected_control);
extern void telemetryFree(void *ptr);

typedef struct can_tx_packet {
    struct can_tx_packet *next;
    uint32_t queued_ms;
    uint16_t id, len, offset;
    uint8_t watched;
    uint8_t data[];
} can_tx_packet;
static can_tx_packet *head, *tail, *retired;
/* One watched time sync packet: 1 next submit is watched, 2 queued, 3 last
 * fragment in hardware buffer `watch_buf`, 4 sent at `watch_us`.
 * Every path that loses the watched packet returns the state to 0, so a
 * later exchange can arm again: expiry or reset of the queued packet (2),
 * a bus-off reset that aborts the hardware buffer (3), a refused or
 * oversize submit (1), and a send that never reached submit (disarm). A
 * hardware-buffer watch (3) whose completion never arrives is abandoned
 * after CAN_TX_MAX_AGE_MS so it cannot hold the watch until reboot. */
#define CAN_TX_WATCH_MAX 72U
static volatile uint8_t watch_state;
static uint8_t watch_len;
static uint32_t watch_buf;
static uint32_t watch_buffered_ms;
static uint64_t watch_us;
static uint8_t watch_bytes[CAN_TX_WATCH_MAX];
volatile uint32_t g_can_tx_watch_abandoned;

static uint32_t lock(void);
static void unlock(uint32_t saved);

void can_tx_queue_watch_next(void) {
    const uint32_t saved = lock();
    if (watch_state == 3U &&
        (uint32_t)(HAL_GetTick() - watch_buffered_ms) >= CAN_TX_MAX_AGE_MS) {
        ++g_can_tx_watch_abandoned;
        watch_state = 0U;
    }
    if (watch_state != 2U && watch_state != 3U) watch_state = 1U;
    unlock(saved);
}

void can_tx_queue_watch_disarm(void) {
    const uint32_t saved = lock();
    if (watch_state == 1U) watch_state = 0U;
    unlock(saved);
}

void can_tx_queue_note_complete(uint32_t buffers) {
    if (watch_state == 3U && (buffers & watch_buf) != 0U) {
        watch_us = board_now_us();
        watch_state = 4U;
    }
}

uint8_t can_tx_queue_take_watched_sent(const uint8_t **bytes, size_t *len, uint64_t *sent_us) {
    if (watch_state != 4U) return 0U;
    *bytes = watch_bytes; *len = watch_len; *sent_us = watch_us;
    watch_state = 0U;
    return 1U;
}
volatile uint32_t g_can_tx_queue_bytes;
volatile uint32_t g_can_tx_pending;
volatile uint32_t g_can_tx_high_water;
volatile uint32_t g_can_tx_rejected;
volatile uint32_t g_can_tx_expired;
/* Submitted completely to hardware message RAM; not end-to-end acknowledged. */
volatile uint32_t g_can_tx_submitted;

static uint32_t lock(void) { uint32_t saved = __get_PRIMASK(); __disable_irq(); return saved; }
static void unlock(uint32_t saved) { __set_PRIMASK(saved); }
static size_t charge(size_t len) {
    /* Include alignment and conservative allocator metadata in the byte cap. */
    return ((sizeof(can_tx_packet) + len + 7U) & ~(size_t)7U) + 16U;
}
/* Metadata and receipts retain admission capacity when ordinary telemetry
 * fills its share. Keep all SDT metadata/data in original order; canonical
 * receipts may interrupt bulk without invalidating dictionary/chunk order. */
static int queue_full(size_t cost, uint32_t id) {
    const unsigned urgent = id < 0x80U;
    const unsigned metadata = id < 0x100U;
    const unsigned count_limit = CAN_TX_MAX_PENDING - (urgent ? 0U : metadata ? 4U : 12U);
    const size_t byte_limit = CAN_TX_QUEUE_BUDGET - (urgent ? 0U : metadata ? 512U : 1536U);
    return g_can_tx_pending >= count_limit || g_can_tx_queue_bytes > byte_limit ||
        cost > byte_limit - g_can_tx_queue_bytes;
}
static void retire_head(void) {
    can_tx_packet *done = head;
    /* A watched packet leaving the queue unsent (expiry, reset) ends the
     * watch; a sent one has already advanced it to 3. */
    if (done->watched && watch_state == 2U) {
        ++g_can_tx_watch_abandoned;
        watch_state = 0U;
    }
    head = done->next;
    if (!head) tail = NULL;
    done->next = retired;
    retired = done;
    --g_can_tx_pending;
}
static void reap(void) {
    uint32_t saved = lock();
    can_tx_packet *list = retired;
    retired = NULL;
    unlock(saved);
    while (list) {
        can_tx_packet *next = list->next;
        size_t cost = charge(list->len);
        telemetryFree(list);
        saved = lock();
        g_can_tx_queue_bytes -= cost;
        unlock(saved);
        list = next;
    }
}

void can_tx_queue_pump(void) {
    const uint32_t saved = lock();
    for (unsigned work = 0; head && work < CAN_TX_PUMP_FRAMES; ++work) {
        if ((uint32_t)(HAL_GetTick() - head->queued_ms) >= CAN_TX_MAX_AGE_MS) {
            ++g_can_tx_expired;
            retire_head();
            continue;
        }
        /* Link format v2 (can_link.h): no payload header; the frame kind
         * and padding ride in the identifier and the padding bytes. */
        uint8_t frame[CAN_LINK_FRAME_BYTES];
        uint32_t wire_id;
        size_t take;
        const size_t used = can_link_frame(head->data, head->len, head->offset,
                                           head->id, frame, &wire_id, &take);
        /* HAL copies into message RAM synchronously, but never waits for wire
         * completion. BUSY/error retains this exact fragment for later service. */
        if (can_bus_send_bytes(frame, used, wire_id) != HAL_OK) break;
        head->offset += take;
        if (head->offset == head->len) {
            if (head->watched && watch_state == 2U) {
                watch_buf = can_bus_latest_tx_buffer();
                watch_buffered_ms = HAL_GetTick();
                watch_state = 3U;
                head->watched = 0U;
            }
            ++g_can_tx_submitted;
            retire_head();
        }
    }
    unlock(saved);
}

void can_tx_queue_service(void) {
    reap();
    can_tx_queue_pump();
    reap();
}

HAL_StatusTypeDef can_tx_queue_submit(const uint8_t *data, size_t len, uint32_t id) {
    /* A watch request applies to this submit only, accepted or not. */
    uint32_t saved = lock();
    const uint8_t watch = watch_state == 1U && data && len && len <= CAN_TX_WATCH_MAX;
    if (watch_state == 1U) watch_state = 0U;
    unlock(saved);
    if (!data || !len || len > CAN_TX_MAX_PACKET) return HAL_ERROR;
    id &= 0x7ffU;
    can_tx_queue_service();
    const size_t cost = charge(len);
    saved = lock();
    int full = queue_full(cost, id);
    if (full) ++g_can_tx_rejected;
    unlock(saved);
    if (full) return HAL_BUSY;
    can_tx_packet *packet = telemetry_can_tx_allocate_priority(sizeof(*packet) + len, id < 0x100U);
    if (!packet) {
        saved = lock(); ++g_can_tx_rejected; unlock(saved);
        return HAL_BUSY;
    }
    packet->next = NULL;
    packet->queued_ms = HAL_GetTick();
    packet->id = (uint16_t)(id & 0x7ffU);
    packet->len = (uint16_t)len;
    packet->offset = 0;
    packet->watched = 0U;
    memcpy(packet->data, data, len);
    saved = lock();
    /* Another foreground sender may have filled the queue during allocation. */
    full = queue_full(cost, id);
    if (full) {
        ++g_can_tx_rejected;
        unlock(saved);
        telemetryFree(packet);
        return HAL_BUSY;
    }
    if (watch && watch_state == 0U) {
        memcpy(watch_bytes, data, len);
        watch_len = (uint8_t)len;
        packet->watched = 1U;
        watch_state = 2U;
    }
    if (id < 0x80U) {
        can_tx_packet **at = &head;
        while (*at && (*at)->id < 0x80U) at = &(*at)->next;
        packet->next = *at;
        *at = packet;
        if (packet->next == NULL) tail = packet;
    } else {
        if (tail) tail->next = packet; else head = packet;
        tail = packet;
    }
    ++g_can_tx_pending;
    g_can_tx_queue_bytes += cost;
    if (g_can_tx_pending > g_can_tx_high_water) g_can_tx_high_water = g_can_tx_pending;
    unlock(saved);
    can_tx_queue_service();
    return HAL_OK;
}

void can_tx_queue_reset(void) {
    const uint32_t saved = lock();
    while (head) retire_head();
    /* Reset follows HAL_FDCAN_AbortTxRequest or controller init: a watched
     * last fragment in a hardware buffer will never complete. */
    if (watch_state == 3U) {
        ++g_can_tx_watch_abandoned;
        watch_state = 0U;
    }
    unlock(saved);
    reap();
}
