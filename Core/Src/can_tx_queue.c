#include "can_tx_queue.h"
#include "main.h"
#include <string.h>

#ifndef CAN_TX_QUEUE_BUDGET
#define CAN_TX_QUEUE_BUDGET 6144U
#endif
#define CAN_TX_MAX_PACKET 128U
#define CAN_TX_MAX_PENDING 48U
#define CAN_TX_MAX_AGE_MS 1000U
#define CAN_TX_PUMP_FRAMES 3U
#define CAN_TX_FRAGMENT_BYTES CAN_TX_FRAGMENT_PAYLOAD_BYTES

/* These hooks are foreground-only and preserve allocator emergency headroom. */
extern void *telemetry_can_tx_allocate_priority(size_t bytes, uint8_t protected_control);
extern void telemetryFree(void *ptr);

typedef struct can_tx_packet {
    struct can_tx_packet *next;
    uint32_t queued_ms;
    uint16_t id, len, offset;
    uint8_t seq;
    uint8_t data[];
} can_tx_packet;
static can_tx_packet *head, *tail, *retired;
static uint8_t next_seq;
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
        const unsigned index = head->offset / CAN_TX_FRAGMENT_BYTES;
        const unsigned count = (head->len + CAN_TX_FRAGMENT_BYTES - 1U) / CAN_TX_FRAGMENT_BYTES;
        uint8_t frame[64] = {'S', 'D', 1U, head->seq, (uint8_t)index, (uint8_t)count,
            (uint8_t)((index == 0U ? 1U : 0U) | (index + 1U == count ? 2U : 0U)),
            (uint8_t)head->len, (uint8_t)(head->len >> 8U)};
        size_t take = head->len - head->offset;
        if (take > CAN_TX_FRAGMENT_BYTES) take = CAN_TX_FRAGMENT_BYTES;
        memcpy(frame + 9U, head->data + head->offset, take);
        /* HAL copies into message RAM synchronously, but never waits for wire
         * completion. BUSY/error retains this exact fragment for later service. */
        /* Single-fragment packets need no fixed reassembly stride. HAL
         * rounds the used bytes to a legal DLC; retain 64-byte multi-frame
         * packets for compatibility with existing receivers. */
        const size_t used = count == 1U ? 9U + take : sizeof(frame);
        if (can_bus_send_bytes(frame, used, head->id) != HAL_OK) break;
        head->offset += take;
        if (head->offset == head->len) {
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
    if (!data || !len || len > CAN_TX_MAX_PACKET) return HAL_ERROR;
    id &= 0x7ffU;
    can_tx_queue_service();
    const size_t cost = charge(len);
    uint32_t saved = lock();
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
    packet->seq = next_seq++;
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
    unlock(saved);
    reap();
}
