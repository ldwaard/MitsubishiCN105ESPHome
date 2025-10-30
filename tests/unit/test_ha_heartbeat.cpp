/// test_ha_heartbeat.cpp — Tests for cn105_protocol::is_ha_heartbeat_fresh().
/// Deps: cn105_protocol.h (production code, no mocks needed)
///
/// The predicate is a pure elapsed-time check using unsigned subtraction, so it
/// stays correct across millis() rollover. The firmware initialises
/// last_ha_heartbeat_ms_ to UINT32_MAX, which makes the heartbeat look fresh for
/// roughly the first `timeout` milliseconds after boot (boot grace window).
#include <gtest/gtest.h>
#include "cn105_protocol.h"

using cn105_protocol::is_ha_heartbeat_fresh;

// ════════════════════════════════════════════════════════════════
// Basic freshness
// ════════════════════════════════════════════════════════════════

TEST(HaHeartbeat, WithinTimeoutIsFresh) {
    EXPECT_TRUE(is_ha_heartbeat_fresh(10000, 100000, 120000));
}

TEST(HaHeartbeat, JustUnderTimeoutIsFresh) {
    // elapsed = 119999 ms
    EXPECT_TRUE(is_ha_heartbeat_fresh(10000, 129999, 120000));
}

TEST(HaHeartbeat, ExactlyAtTimeoutIsStale) {
    // elapsed = 120000 ms, comparison is strict
    EXPECT_FALSE(is_ha_heartbeat_fresh(10000, 130000, 120000));
}

TEST(HaHeartbeat, PastTimeoutIsStale) {
    EXPECT_FALSE(is_ha_heartbeat_fresh(10000, 140000, 120000));
}

TEST(HaHeartbeat, HeartbeatThisInstantIsFresh) {
    EXPECT_TRUE(is_ha_heartbeat_fresh(5000, 5000, 120000));
}

TEST(HaHeartbeat, ZeroTimeoutIsNeverFresh) {
    EXPECT_FALSE(is_ha_heartbeat_fresh(5000, 5000, 0));
    EXPECT_FALSE(is_ha_heartbeat_fresh(5000, 6000, 0));
}

// ════════════════════════════════════════════════════════════════
// millis() rollover
// ════════════════════════════════════════════════════════════════

TEST(HaHeartbeat, SurvivesMillisRollover) {
    const uint32_t last_heartbeat_ms = UINT32_MAX - 10;
    const uint32_t now_ms = 20;   // 31 ms elapsed across the wrap
    EXPECT_TRUE(is_ha_heartbeat_fresh(last_heartbeat_ms, now_ms, 100));
    EXPECT_TRUE(is_ha_heartbeat_fresh(last_heartbeat_ms, now_ms, 32));
    EXPECT_FALSE(is_ha_heartbeat_fresh(last_heartbeat_ms, now_ms, 31));
}

TEST(HaHeartbeat, StaleAcrossRolloverWhenTimeoutExceeded) {
    const uint32_t last_heartbeat_ms = UINT32_MAX - 1000;
    const uint32_t now_ms = 200000;   // ~200 s elapsed across the wrap
    EXPECT_FALSE(is_ha_heartbeat_fresh(last_heartbeat_ms, now_ms, 120000));
}

// ════════════════════════════════════════════════════════════════
// Boot grace window: last_ha_heartbeat_ms_ starts at UINT32_MAX
// ════════════════════════════════════════════════════════════════

TEST(HaHeartbeat, BootSentinelIsFreshAtBoot) {
    EXPECT_TRUE(is_ha_heartbeat_fresh(UINT32_MAX, 0, 120000));
}

TEST(HaHeartbeat, BootSentinelStaysFreshUntilTimeout) {
    // elapsed = now + 1 because of the wrap from UINT32_MAX
    EXPECT_TRUE(is_ha_heartbeat_fresh(UINT32_MAX, 119998, 120000));    // elapsed 119999
    EXPECT_FALSE(is_ha_heartbeat_fresh(UINT32_MAX, 119999, 120000));   // elapsed 120000
}

TEST(HaHeartbeat, BootSentinelIsStaleAfterGraceWindow) {
    EXPECT_FALSE(is_ha_heartbeat_fresh(UINT32_MAX, 200000, 120000));
}

// ════════════════════════════════════════════════════════════════
// "Not required" configuration: timeout is UINT32_MAX
// ════════════════════════════════════════════════════════════════

TEST(HaHeartbeat, NotRequiredTimeoutIsFreshForLongElapsedTimes) {
    EXPECT_TRUE(is_ha_heartbeat_fresh(0, 1000000, UINT32_MAX));
    EXPECT_TRUE(is_ha_heartbeat_fresh(0, UINT32_MAX - 1, UINT32_MAX));
}

TEST(HaHeartbeat, NotRequiredTimeoutIsFreshWithBootSentinel) {
    EXPECT_TRUE(is_ha_heartbeat_fresh(UINT32_MAX, 0, UINT32_MAX));
    EXPECT_TRUE(is_ha_heartbeat_fresh(UINT32_MAX, 3600000, UINT32_MAX));
}