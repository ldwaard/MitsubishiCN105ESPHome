/// test_pac_reconnect_decision.cpp — Tests for cn105_protocol::should_reconnect_during_passthrough().
/// Deps: cn105_protocol.h (production code, no mocks needed)
///
/// During PAC passthrough the ESP reconnects to the heat pump when the link looks stale or
/// when too many forwarded requests in a row got no reply. It never reconnects while a
/// handshake is in flight or too soon after the previous reconnect.
#include <gtest/gtest.h>
#include "cn105_protocol.h"

using cn105_protocol::should_reconnect_during_passthrough;

namespace {
constexpr uint8_t kMaxUnansweredForwards = 3;
constexpr uint32_t kMinReconnectIntervalMs = 15000;
constexpr uint32_t kWellPastIntervalMs = 60000;

bool decide(
    bool handshake_in_flight,
    bool connection_active,
    uint8_t unanswered_forwards,
    uint32_t since_last_reconnect_ms
) {
    return should_reconnect_during_passthrough(
        handshake_in_flight,
        connection_active,
        unanswered_forwards,
        kMaxUnansweredForwards,
        since_last_reconnect_ms,
        kMinReconnectIntervalMs);
}
}

// ════════════════════════════════════════════════════════════════
// Healthy link: regression for the inverted connection check
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, HealthyLinkWithNoFailuresNeverReconnects) {
    EXPECT_FALSE(decide(false, true, 0, kWellPastIntervalMs));
}

TEST(PacReconnect, HealthyLinkStaysConnectedForAnyElapsedTime) {
    EXPECT_FALSE(decide(false, true, 0, 0));
    EXPECT_FALSE(decide(false, true, 0, kMinReconnectIntervalMs));
    EXPECT_FALSE(decide(false, true, 0, UINT32_MAX));
}

// ════════════════════════════════════════════════════════════════
// Stale link
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, StaleLinkReconnects) {
    EXPECT_TRUE(decide(false, false, 0, kWellPastIntervalMs));
}

// ════════════════════════════════════════════════════════════════
// Unanswered forwards
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, BelowFailureThresholdDoesNotReconnect) {
    EXPECT_FALSE(decide(false, true, 1, kWellPastIntervalMs));
    EXPECT_FALSE(decide(false, true, kMaxUnansweredForwards - 1, kWellPastIntervalMs));
}

TEST(PacReconnect, AtFailureThresholdReconnects) {
    EXPECT_TRUE(decide(false, true, kMaxUnansweredForwards, kWellPastIntervalMs));
}

TEST(PacReconnect, AboveFailureThresholdReconnects) {
    EXPECT_TRUE(decide(false, true, kMaxUnansweredForwards + 1, kWellPastIntervalMs));
}

// ════════════════════════════════════════════════════════════════
// Guards: no reconnect storm, no interference with a handshake
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, HandshakeInFlightBlocksStaleLinkReconnect) {
    EXPECT_FALSE(decide(true, false, 0, kWellPastIntervalMs));
}

TEST(PacReconnect, HandshakeInFlightBlocksFailureReconnect) {
    EXPECT_FALSE(decide(true, true, kMaxUnansweredForwards, kWellPastIntervalMs));
}

TEST(PacReconnect, JustUnderMinIntervalBlocksReconnect) {
    EXPECT_FALSE(decide(false, false, 0, kMinReconnectIntervalMs - 1));
    EXPECT_FALSE(decide(false, true, kMaxUnansweredForwards, kMinReconnectIntervalMs - 1));
}

TEST(PacReconnect, ExactlyAtMinIntervalAllowsReconnect) {
    EXPECT_TRUE(decide(false, false, 0, kMinReconnectIntervalMs));
    EXPECT_TRUE(decide(false, true, kMaxUnansweredForwards, kMinReconnectIntervalMs));
}

TEST(PacReconnect, ImmediatelyAfterReconnectNeverReconnectsAgain) {
    EXPECT_FALSE(decide(false, false, kMaxUnansweredForwards, 0));
}

// ════════════════════════════════════════════════════════════════
// Both triggers together
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, StaleLinkAndFailuresReconnectOnce) {
    EXPECT_TRUE(decide(false, false, kMaxUnansweredForwards, kWellPastIntervalMs));
}
