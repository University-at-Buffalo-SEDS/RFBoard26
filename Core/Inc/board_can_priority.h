#ifndef BOARD_CAN_PRIORITY_H
#define BOARD_CAN_PRIORITY_H
#include <stddef.h>
#include <stdint.h>

/* Owned SDK output uses three CAN arbitration bands, keeping the board's
 * source bits unchanged: short controls/ACKs, protected metadata, then data.
 * Metadata stays below0x100 so reserved receive capacity remains available.
 * SDK logical priorities and dictionary/chunk order are unchanged. */
static inline uint32_t board_can_arbitration_id(const uint8_t *bytes, size_t len,
        uint8_t priority, uint32_t data_type, uint32_t heartbeat_type,
        uint32_t control_id) {
    if (bytes == NULL || len == 0U) return control_id | 0x100U;
    const unsigned side_transport = len >= 3U && bytes[0] == 'S' &&
                                     bytes[1] == 'D' && bytes[2] == 'T';
    /* DiscoverySchema is the stable built-in wire type10. ACKs and small
     * frozen controls are canonical; bulk reliable transfers may use SDT. */
    if (data_type == 10U || (priority >= 200U && side_transport))
        return control_id | 0x80U;
    if (priority >= 200U || data_type == heartbeat_type) return control_id;
    return control_id | 0x100U;
}
#endif
