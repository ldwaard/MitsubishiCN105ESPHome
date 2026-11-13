/// test_pac_cache_index.cpp — Tests for cn105_protocol::pac_cache_index_for_code().
/// Deps: cn105_protocol.h (production code, no mocks needed)
///
/// The PAC response cache has one fixed slot per request code the scheduler polls.
/// These tests pin down that mapping so an unknown code can never take (or evict)
/// a slot that belongs to a polled code.
#include <gtest/gtest.h>
#include <set>
#include "cn105_protocol.h"

using cn105_protocol::kPacCacheableCodes;
using cn105_protocol::kPacCacheSlotCount;
using cn105_protocol::pac_cache_index_for_code;

// ════════════════════════════════════════════════════════════════
// Every polled code has its own slot
// ════════════════════════════════════════════════════════════════

TEST(PacCacheIndex, SlotCountMatchesNumberOfPolledCodes) {
    EXPECT_EQ(kPacCacheSlotCount, 8);
}

TEST(PacCacheIndex, EveryCacheableCodeMapsToItsOwnTableIndex) {
    for (int slot_index = 0; slot_index < kPacCacheSlotCount; slot_index++) {
        EXPECT_EQ(pac_cache_index_for_code(kPacCacheableCodes[slot_index]), slot_index)
            << "code 0x" << std::hex << static_cast<int>(kPacCacheableCodes[slot_index]);
    }
}

TEST(PacCacheIndex, SlotIndicesAreDistinctAndInRange) {
    std::set<int> seen_slot_indices;
    for (int slot_index = 0; slot_index < kPacCacheSlotCount; slot_index++) {
        const int mapped_index = pac_cache_index_for_code(kPacCacheableCodes[slot_index]);
        EXPECT_GE(mapped_index, 0);
        EXPECT_LT(mapped_index, kPacCacheSlotCount);
        seen_slot_indices.insert(mapped_index);
    }
    EXPECT_EQ(static_cast<int>(seen_slot_indices.size()), kPacCacheSlotCount);
}

TEST(PacCacheIndex, CacheableCodesAreUnique) {
    std::set<uint8_t> seen_codes;
    for (int slot_index = 0; slot_index < kPacCacheSlotCount; slot_index++) {
        EXPECT_TRUE(seen_codes.insert(kPacCacheableCodes[slot_index]).second)
            << "duplicate code at table index " << slot_index;
    }
}

// ════════════════════════════════════════════════════════════════
// Known codes (explicit, so a table edit is a deliberate test change)
// ════════════════════════════════════════════════════════════════

TEST(PacCacheIndex, PolledCodesAreCacheable) {
    const uint8_t polled_codes[] = {
        0x02,   // settings
        0x03,   // room temperature
        0x04,   // error info
        0x06,   // status
        0x09,   // standby / sub-modes
        0x42,   // HVAC options
        0x20,   // function settings part 1
        0x22,   // function settings part 2
    };
    for (uint8_t polled_code : polled_codes) {
        EXPECT_GE(pac_cache_index_for_code(polled_code), 0)
            << "code 0x" << std::hex << static_cast<int>(polled_code);
    }
}

// ════════════════════════════════════════════════════════════════
// Codes that must never be cached
// ════════════════════════════════════════════════════════════════

TEST(PacCacheIndex, DisabledTimersCodeIsNotCacheable) {
    // 0x05 (timers) is registered but permanently disabled, so it is never polled.
    EXPECT_EQ(pac_cache_index_for_code(0x05), -1);
}

TEST(PacCacheIndex, UnpolledCodesAreNotCacheable) {
    // A controller can request codes the ESP never polls (e.g. 0x10, or 0x00).
    // They must not be able to take a slot from a polled code.
    EXPECT_EQ(pac_cache_index_for_code(0x00), -1);
    EXPECT_EQ(pac_cache_index_for_code(0x10), -1);
    EXPECT_EQ(pac_cache_index_for_code(0x07), -1);
    EXPECT_EQ(pac_cache_index_for_code(0xFF), -1);
}

TEST(PacCacheIndex, ResponseCommandBytesAreNotCacheableCodes) {
    // Frame command bytes (0x61 ack, 0x62 data) must not be confused with request codes.
    EXPECT_EQ(pac_cache_index_for_code(0x61), -1);
    EXPECT_EQ(pac_cache_index_for_code(0x62), -1);
}

TEST(PacCacheIndex, EveryUnlistedByteReturnsMinusOne) {
    std::set<uint8_t> cacheable_codes(kPacCacheableCodes, kPacCacheableCodes + kPacCacheSlotCount);
    for (int byte_value = 0; byte_value <= 0xFF; byte_value++) {
        if (cacheable_codes.count(static_cast<uint8_t>(byte_value)) == 0) {
            EXPECT_EQ(pac_cache_index_for_code(static_cast<uint8_t>(byte_value)), -1)
                << "byte 0x" << std::hex << byte_value;
        }
    }
}

// ════════════════════════════════════════════════════════════════
// Function settings SET -> cached GET reply it makes stale
// ════════════════════════════════════════════════════════════════

TEST(PacCacheIndex, FunctionSetPartsMapToTheirOwnGetCodes) {
    EXPECT_EQ(cn105_protocol::functions_get_code_for_set(0x1F), 0x20);
    EXPECT_EQ(cn105_protocol::functions_get_code_for_set(0x21), 0x22);
}

TEST(PacCacheIndex, FunctionGetCodesHaveCacheSlots) {
    EXPECT_GE(pac_cache_index_for_code(cn105_protocol::functions_get_code_for_set(0x1F)), 0);
    EXPECT_GE(pac_cache_index_for_code(cn105_protocol::functions_get_code_for_set(0x21)), 0);
}

TEST(PacCacheIndex, OtherSetTypesMapToNoGetCode) {
    for (int byte_value = 0; byte_value <= 0xFF; byte_value++) {
        if (byte_value == 0x1F || byte_value == 0x21) {
            continue;
        }
        EXPECT_EQ(cn105_protocol::functions_get_code_for_set(static_cast<uint8_t>(byte_value)), 0)
            << "byte 0x" << std::hex << byte_value;
    }
}
