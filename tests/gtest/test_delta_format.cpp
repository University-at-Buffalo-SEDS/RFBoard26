#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

extern "C" {
#include "launchcore/delta.h"

uint32_t launchcore_crc32(const void *data, size_t size)
{
    const auto *bytes = static_cast<const uint8_t *>(data);
    uint32_t hash = 2166136261u;
    for (size_t index = 0; index < size; ++index) {
        hash = (hash ^ bytes[index]) * 16777619u;
    }
    return hash;
}
}

static launchcore_delta_header_t valid_header()
{
    launchcore_delta_header_t header{};
    header.magic = LAUNCHCORE_DELTA_MAGIC;
    header.version = LAUNCHCORE_DELTA_VERSION;
    header.header_size = sizeof(header);
    header.records_offset = sizeof(header);
    header.total_size = sizeof(header) + 64u;
    header.record_count = 1u;
    header.erase_size = 2048u;
    header.base_image_size = 4096u;
    header.target_image_size = 4096u;
    header.header_crc32 = launchcore_delta_header_crc(&header);
    return header;
}

TEST(DeltaFormat, AcceptsWellFormedHeader)
{
    const auto header = valid_header();
    EXPECT_TRUE(launchcore_delta_header_is_valid(&header, header.total_size));
}

TEST(DeltaFormat, RejectsNullTruncatedAndCorruptHeaders)
{
    EXPECT_FALSE(launchcore_delta_header_is_valid(nullptr, 4096u));

    auto header = valid_header();
    EXPECT_FALSE(launchcore_delta_header_is_valid(&header, header.total_size - 1u));

    header = valid_header();
    header.record_count = 0u;
    header.header_crc32 = launchcore_delta_header_crc(&header);
    EXPECT_FALSE(launchcore_delta_header_is_valid(&header, header.total_size));

    header = valid_header();
    header.header_crc32 ^= 1u;
    EXPECT_FALSE(launchcore_delta_header_is_valid(&header, header.total_size));
}

TEST(DeltaFormat, RejectsInvalidRecordBounds)
{
    auto header = valid_header();
    header.records_offset = header.header_size - 1u;
    header.header_crc32 = launchcore_delta_header_crc(&header);
    EXPECT_FALSE(launchcore_delta_header_is_valid(&header, header.total_size));

    header = valid_header();
    header.records_offset = header.total_size + 1u;
    header.header_crc32 = launchcore_delta_header_crc(&header);
    EXPECT_FALSE(launchcore_delta_header_is_valid(&header, header.total_size));
}

