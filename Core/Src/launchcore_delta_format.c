#include "launchcore/delta.h"
#include "launchcore/image.h"
#include <stddef.h>

/* The application validates only the staged delta header. The decoder and
 * SHA-256 implementation stay in the bootloader. */
uint32_t launchcore_delta_header_crc(const launchcore_delta_header_t *header)
{
    return launchcore_crc32(header,
                            offsetof(launchcore_delta_header_t, header_crc32));
}

uint32_t launchcore_delta_record_crc(const launchcore_delta_record_t *record)
{
    return launchcore_crc32(record,
                            offsetof(launchcore_delta_record_t, header_crc32));
}

bool launchcore_delta_header_is_valid(const launchcore_delta_header_t *header,
                                      uint32_t available_size)
{
    return header != NULL && header->magic == LAUNCHCORE_DELTA_MAGIC &&
           header->version == LAUNCHCORE_DELTA_VERSION &&
           header->header_size >= sizeof(*header) &&
           header->records_offset >= header->header_size &&
           header->records_offset <= header->total_size &&
           header->total_size <= available_size &&
           header->record_count != 0u && header->erase_size != 0u &&
           header->base_image_size != 0u && header->target_image_size != 0u &&
           header->header_crc32 == launchcore_delta_header_crc(header);
}

