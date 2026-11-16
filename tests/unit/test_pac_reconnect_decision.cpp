/// test_pac_reconnect_decision.cpp — Tests for cn105_protocol::should_reconnect_during_passthrough().
/// Deps: cn105_protocol.h (production code, no mocks needed)
///
/// During PAC passthrough the ESP reconnects to the heat pump only when too many forwarded
/// requests in a row got no reply. It never reconnects while a handshake is in flight or
/// too soon after the previous reconnect.
#include <gtest/gtest.h>
#include "cn105_protocol.h"

using cn105_protocol::should_reconnect_during_passthrough;

namespace {
constexpr uint8_t kMaxUnansweredForwards = 3;
constexpr uint32_t kMinReconnectIntervalMs = 15000;
constexpr uint32_t kWellPastIntervalMs = 60000;

bool decide(
    bool handshake_in_flight,
    uint8_t unanswered_forwards,
    uint32_t since_last_reconnect_ms
) {
    return should_reconnect_during_passthrough(
        handshake_in_flight,
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
    EXPECT_FALSE(decide(false, 0, kWellPastIntervalMs));
}

TEST(PacReconnect, HealthyLinkStaysConnectedForAnyElapsedTime) {
    EXPECT_FALSE(decide(false, 0, 0));
    EXPECT_FALSE(decide(false, 0, kMinReconnectIntervalMs));
    EXPECT_FALSE(decide(false, 0, UINT32_MAX));
}

// ════════════════════════════════════════════════════════════════
// Unanswered forwards
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, BelowFailureThresholdDoesNotReconnect) {
    EXPECT_FALSE(decide(false, 1, kWellPastIntervalMs));
    EXPECT_FALSE(decide(false, kMaxUnansweredForwards - 1, kWellPastIntervalMs));
}

TEST(PacReconnect, AtFailureThresholdReconnects) {
    EXPECT_TRUE(decide(false, kMaxUnansweredForwards, kWellPastIntervalMs));
}

TEST(PacReconnect, AboveFailureThresholdReconnects) {
    EXPECT_TRUE(decide(false, kMaxUnansweredForwards + 1, kWellPastIntervalMs));
}

// ════════════════════════════════════════════════════════════════
// Guards: no reconnect storm, no interference with a handshake
// ════════════════════════════════════════════════════════════════

TEST(PacReconnect, HandshakeInFlightBlocksFailureReconnect) {
    EXPECT_FALSE(decide(true, kMaxUnansweredForwards, kWellPastIntervalMs));
}

TEST(PacReconnect, JustUnderMinIntervalBlocksReconnect) {
    EXPECT_FALSE(decide(false, 0, kMinReconnectIntervalMs - 1));
    EXPECT_FALSE(decide(false, kMaxUnansweredForwards, kMinReconnectIntervalMs - 1));
}

TEST(PacReconnect, ExactlyAtMinIntervalAllowsReconnect) {
    EXPECT_TRUE(decide(false, kMaxUnansweredForwards, kMinReconnectIntervalMs));
}

TEST(PacReconnect, ExactlyAtMinIntervalWithNoFailuresDoesNotReconnect) {
    EXPECT_FALSE(decide(false, 0, kMinReconnectIntervalMs));
}

TEST(PacReconnect, ImmediatelyAfterReconnectNeverReconnectsAgain) {
    EXPECT_FALSE(decide(false, kMaxUnansweredForwards, 0));
}
