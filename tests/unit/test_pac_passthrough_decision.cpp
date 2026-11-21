#include <gtest/gtest.h>
#include "cn105_protocol.h"

using cn105_protocol::should_pac_passthrough_be_active;

namespace {
constexpr uint32_t kEnterTimeoutMs = 60000;
constexpr uint32_t kLeaveStableMs = 5000;
}

TEST(PacDecision, ForcedAlwaysPassthrough) {
    EXPECT_TRUE(should_pac_passthrough_be_active(false, true, true, 999999, kEnterTimeoutMs, kLeaveStableMs));
    EXPECT_TRUE(should_pac_passthrough_be_active(true, true, true, 999999, kEnterTimeoutMs, kLeaveStableMs));
}

TEST(PacDecision, EntersOnlyAfterTimeout) {
    EXPECT_FALSE(should_pac_passthrough_be_active(false, false, false, 59999, kEnterTimeoutMs, kLeaveStableMs));
    EXPECT_TRUE(should_pac_passthrough_be_active(false, false, false, 60000, kEnterTimeoutMs, kLeaveStableMs));
}

TEST(PacDecision, NeverEntersWhenAutomaticEntryDisabled) {
    EXPECT_FALSE(should_pac_passthrough_be_active(false, false, false, 999999, UINT32_MAX, kLeaveStableMs));
}

TEST(PacDecision, StaysUntilHomeAssistantStable) {
    EXPECT_TRUE(should_pac_passthrough_be_active(true, false, true, 4999, kEnterTimeoutMs, kLeaveStableMs));
    EXPECT_FALSE(should_pac_passthrough_be_active(true, false, true, 5000, kEnterTimeoutMs, kLeaveStableMs));
}

// Regression for the bounce bug: HA flaps back down while in passthrough.
TEST(PacDecision, StaysWhenHomeAssistantDropsAgainDuringHoldoff) {
    EXPECT_TRUE(should_pac_passthrough_be_active(true, false, false, 0, kEnterTimeoutMs, kLeaveStableMs));
    EXPECT_TRUE(should_pac_passthrough_be_active(true, false, false, 30000, kEnterTimeoutMs, kLeaveStableMs));
}

TEST(PacDecision, ReleasingSwitchWhileHomeAssistantDownStaysInPassthrough) {
    EXPECT_TRUE(should_pac_passthrough_be_active(true, false, false, 10000, kEnterTimeoutMs, kLeaveStableMs));
}
