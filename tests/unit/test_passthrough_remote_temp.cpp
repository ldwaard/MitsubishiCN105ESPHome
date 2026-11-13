/// test_passthrough_remote_temp.cpp — Tests for cn105_protocol::passthrough_remote_temperature().
/// Deps: cn105_protocol.h (production code, no mocks needed)
///
/// During PAC passthrough the ESP sends Home Assistant's remote temperature while it is set.
/// When it times out, the controller's last remote temperature takes over while it is no older
/// than one keep-alive interval; otherwise the heat pump reverts to its internal sensor (0).
#include <gtest/gtest.h>
#include "cn105_protocol.h"

using cn105_protocol::passthrough_remote_temperature;

namespace {
constexpr uint32_t kKeepAliveMs = 20000;
constexpr uint32_t kDefaultKeepAliveMs = 20000;
}

TEST(PassthroughRemoteTemp, HomeAssistantValueTakesPriority) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(21.5f, 19.0f, 1000, 2000, kKeepAliveMs, kDefaultKeepAliveMs), 21.5f);
}

TEST(PassthroughRemoteTemp, HomeAssistantValueUsedEvenWithoutController) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(21.5f, 0.0f, 0, 999999, kKeepAliveMs, kDefaultKeepAliveMs), 21.5f);
}

TEST(PassthroughRemoteTemp, RecentControllerValueReplacesMissingHomeAssistant) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, 1000, 6000, kKeepAliveMs, kDefaultKeepAliveMs), 19.0f);
}

TEST(PassthroughRemoteTemp, ControllerValueAtAgeLimitIsStillUsed) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, 1000, 1000 + kKeepAliveMs, kKeepAliveMs, kDefaultKeepAliveMs), 19.0f);
}

TEST(PassthroughRemoteTemp, StaleControllerValueRevertsToInternalSensor) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, 1000, 1001 + kKeepAliveMs, kKeepAliveMs, kDefaultKeepAliveMs), 0.0f);
}

TEST(PassthroughRemoteTemp, ControllerInternalSensorRequestRevertsToInternalSensor) {
    // The controller sent "use internal sensor" (decoded as 0): nothing to hand over.
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 0.0f, 1000, 2000, kKeepAliveMs, kDefaultKeepAliveMs), 0.0f);
}

TEST(PassthroughRemoteTemp, NoValuesRevertsToInternalSensor) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 0.0f, 0, 0, kKeepAliveMs, kDefaultKeepAliveMs), 0.0f);
}

TEST(PassthroughRemoteTemp, DisabledKeepAliveUsesDefaultInterval) {
    constexpr uint32_t kDefault = 30000;
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, 1000, 1000 + kDefault, 0, kDefault), 19.0f);
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, 1000, 1001 + kDefault, 0, kDefault), 0.0f);
}

TEST(PassthroughRemoteTemp, LongerKeepAliveAllowsOlderControllerValue) {
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, 1000, 1000 + 60000, 60000, kDefaultKeepAliveMs), 19.0f);
}

TEST(PassthroughRemoteTemp, AgeSurvivesMillisRollover) {
    // Stored 5 s before the 32-bit wrap, read 5 s after it: 10 s old.
    const uint32_t stored_ms = UINT32_MAX - 4999;
    const uint32_t now_ms = 5000;
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, stored_ms, now_ms, kKeepAliveMs, kDefaultKeepAliveMs), 19.0f);
    EXPECT_FLOAT_EQ(passthrough_remote_temperature(0.0f, 19.0f, stored_ms, now_ms + 15000, kKeepAliveMs, kDefaultKeepAliveMs), 0.0f);
}
