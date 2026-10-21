#include <stdint.h>
#include <stddef.h>
static uint32_t g_rx_crc_errors;
uint32_t launchcore_crc32(const void *data, size_t length) {
    const uint8_t *bytes = data;
    uint32_t crc = UINT32_MAX;
    while (length--) {
        crc ^= *bytes++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xedb88320) : 0U);
    }
    return ~crc;
}
