/// test_request_scheduler.cpp — Tests for esphome::RequestScheduler (production code).
/// Deps: request_scheduler.cpp compiled with CN105_UNIT_TEST, scheduler_test_support.h
///
/// The scheduler keeps at most one request outstanding. A reply only advances the cycle
/// while its request is awaiting; duplicates and replies to a request that was given up
/// on (soft timeout) are ignored, so the cycle can never fork into two outstanding requests.
#include <gtest/gtest.h>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include "request_scheduler.h"

namespace esphome {
class CN105Climate {};
}

namespace {
uint32_t fakeClockMs = 1000;
}

uint32_t scheduler_test_clock_ms() {
    return fakeClockMs;
}

namespace {

using esphome::CN105Climate;
using esphome::InfoRequest;
using esphome::RequestScheduler;

constexpr uint8_t kSettingsCode = 0x02;
constexpr uint8_t kStandbyCode = 0x09;
constexpr uint8_t kStatusCode = 0x06;
constexpr uint32_t kStandbySoftTimeoutMs = 500;

struct PendingTimeout {
    std::string name;
    uint32_t delayMs;
    std::function<void()> callback;
};

class SchedulerHarness {
public:
    SchedulerHarness()
        : scheduler(
            [this](uint8_t code) { this->sentCodes.push_back(code); return true; },
            [this](const std::string& name, uint32_t delayMs, std::function<void()> callback) {
                this->replaceTimeout(name, delayMs, std::move(callback));
            },
            [this]() { ++this->terminateCount; },
            [this]() -> CN105Climate* { return &this->climate; },
            [this]() { ++this->abandonCount; }
        ) {
        InfoRequest settings("settings", "Settings", kSettingsCode, 3, 0);
        settings.onResponse = [this](CN105Climate&) { this->respondedCodes.push_back(kSettingsCode); };
        this->scheduler.register_request(settings);

        InfoRequest standby("standby", "Standby", kStandbyCode, 3, kStandbySoftTimeoutMs);
        standby.onResponse = [this](CN105Climate&) { this->respondedCodes.push_back(kStandbyCode); };
        this->scheduler.register_request(standby);

        InfoRequest status("status", "Status", kStatusCode, 3, 0);
        status.onResponse = [this](CN105Climate&) { this->respondedCodes.push_back(kStatusCode); };
        this->scheduler.register_request(status);
    }

    void startCycle() {
        this->scheduler.send_next_after(0x00);
    }

    bool fireTimeoutFor(uint8_t code) {
        char name[32];
        std::snprintf(name, sizeof(name), "info_timeout_0x%02X", code);
        for (size_t index = 0; index < this->pendingTimeouts.size(); ++index) {
            if (this->pendingTimeouts[index].name == name) {
                std::function<void()> callback = this->pendingTimeouts[index].callback;
                this->pendingTimeouts.erase(this->pendingTimeouts.begin() + index);
                callback();
                return true;
            }
        }
        return false;
    }

    CN105Climate climate;
    std::vector<uint8_t> sentCodes;
    std::vector<uint8_t> respondedCodes;
    std::vector<PendingTimeout> pendingTimeouts;
    int terminateCount = 0;
    int abandonCount = 0;
    RequestScheduler scheduler;

private:
    void replaceTimeout(const std::string& name, uint32_t delayMs, std::function<void()> callback) {
        for (size_t index = 0; index < this->pendingTimeouts.size(); ++index) {
            if (this->pendingTimeouts[index].name == name) {
                this->pendingTimeouts.erase(this->pendingTimeouts.begin() + index);
                break;
            }
        }
        this->pendingTimeouts.push_back({name, delayMs, std::move(callback)});
    }
};

}  // namespace

// ════════════════════════════════════════════════════════════════
// Normal cycle
// ════════════════════════════════════════════════════════════════

TEST(RequestScheduler, CycleSendsOneRequestPerReply) {
    SchedulerHarness harness;
    harness.startCycle();
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode}));

    EXPECT_TRUE(harness.scheduler.process_response(kSettingsCode));
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStandbyCode}));

    EXPECT_TRUE(harness.scheduler.process_response(kStandbyCode));
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStandbyCode, kStatusCode}));
    EXPECT_EQ(harness.terminateCount, 0);

    EXPECT_TRUE(harness.scheduler.process_response(kStatusCode));
    EXPECT_EQ(harness.terminateCount, 1);
    EXPECT_EQ(harness.respondedCodes, (std::vector<uint8_t>{kSettingsCode, kStandbyCode, kStatusCode}));
}

TEST(RequestScheduler, UnmanagedCodeIsNotHandled) {
    SchedulerHarness harness;
    harness.startCycle();
    EXPECT_FALSE(harness.scheduler.process_response(0x77));
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode}));
}

// ════════════════════════════════════════════════════════════════
// Duplicate and unsolicited replies
// ════════════════════════════════════════════════════════════════

TEST(RequestScheduler, DuplicateReplyDoesNotSendAnotherRequest) {
    SchedulerHarness harness;
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
    ASSERT_EQ(harness.sentCodes.size(), 2u);

    EXPECT_TRUE(harness.scheduler.process_response(kSettingsCode));

    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStandbyCode}));
    EXPECT_EQ(harness.respondedCodes, (std::vector<uint8_t>{kSettingsCode}));
}

TEST(RequestScheduler, ReplyBeforeAnyRequestIsIgnored) {
    SchedulerHarness harness;
    EXPECT_TRUE(harness.scheduler.process_response(kSettingsCode));
    EXPECT_TRUE(harness.sentCodes.empty());
    EXPECT_TRUE(harness.respondedCodes.empty());
    EXPECT_EQ(harness.terminateCount, 0);
}

TEST(RequestScheduler, ReplyForARequestThatIsNotTheCurrentOneIsIgnored) {
    SchedulerHarness harness;
    harness.startCycle();

    EXPECT_TRUE(harness.scheduler.process_response(kStatusCode));

    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode}));
    EXPECT_TRUE(harness.respondedCodes.empty());
}

// ════════════════════════════════════════════════════════════════
// Soft timeout
// ════════════════════════════════════════════════════════════════

TEST(RequestScheduler, SoftTimeoutAbandonsThenAdvancesOnce) {
    SchedulerHarness harness;
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
    ASSERT_EQ(harness.sentCodes.back(), kStandbyCode);
    ASSERT_EQ(harness.pendingTimeouts.size(), 1u);
    EXPECT_EQ(harness.pendingTimeouts[0].delayMs, kStandbySoftTimeoutMs);

    ASSERT_TRUE(harness.fireTimeoutFor(kStandbyCode));

    EXPECT_EQ(harness.abandonCount, 1);
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStandbyCode, kStatusCode}));
}

TEST(RequestScheduler, ReplyAfterSoftTimeoutIsIgnored) {
    SchedulerHarness harness;
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
    ASSERT_TRUE(harness.fireTimeoutFor(kStandbyCode));
    const std::vector<uint8_t> sentBeforeReply = harness.sentCodes;

    EXPECT_TRUE(harness.scheduler.process_response(kStandbyCode));

    EXPECT_EQ(harness.sentCodes, sentBeforeReply);
    EXPECT_EQ(harness.terminateCount, 0);
    EXPECT_EQ(harness.respondedCodes, (std::vector<uint8_t>{kSettingsCode}));
}

TEST(RequestScheduler, TimerLeftOverFromAnAnsweredRequestDoesNothing) {
    SchedulerHarness harness;
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
    ASSERT_TRUE(harness.scheduler.process_response(kStandbyCode));
    const std::vector<uint8_t> sentBeforeTimer = harness.sentCodes;

    ASSERT_TRUE(harness.fireTimeoutFor(kStandbyCode));

    EXPECT_EQ(harness.abandonCount, 0);
    EXPECT_EQ(harness.sentCodes, sentBeforeTimer);
}

TEST(RequestScheduler, RepeatedSoftTimeoutsDisableTheRequest) {
    SchedulerHarness harness;
    for (int attempt = 0; attempt < 3; ++attempt) {
        harness.startCycle();
        ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
        ASSERT_TRUE(harness.fireTimeoutFor(kStandbyCode));
        ASSERT_TRUE(harness.scheduler.process_response(kStatusCode));
    }
    EXPECT_EQ(harness.abandonCount, 3);

    harness.sentCodes.clear();
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStatusCode}));
}

TEST(RequestScheduler, SoftTimeoutWithoutAbandonCallbackStillAdvances) {
    std::vector<uint8_t> sentCodes;
    std::vector<std::function<void()>> timeoutCallbacks;
    RequestScheduler scheduler(
        [&sentCodes](uint8_t code) { sentCodes.push_back(code); return true; },
        [&timeoutCallbacks](const std::string&, uint32_t, std::function<void()> callback) {
            timeoutCallbacks.push_back(std::move(callback));
        });
    InfoRequest standby("standby", "Standby", kStandbyCode, 3, kStandbySoftTimeoutMs);
    scheduler.register_request(standby);
    InfoRequest status("status", "Status", kStatusCode, 3, 0);
    scheduler.register_request(status);

    scheduler.send_next_after(0x00);
    ASSERT_EQ(timeoutCallbacks.size(), 1u);
    timeoutCallbacks[0]();

    EXPECT_EQ(sentCodes, (std::vector<uint8_t>{kStandbyCode, kStatusCode}));
}

TEST(RequestScheduler, ResetPendingMakesStaleSoftTimeoutHarmless) {
    SchedulerHarness harness;
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));   // standby now awaiting, timer armed
    ASSERT_EQ(harness.pendingTimeouts.size(), 1u);
    const std::vector<uint8_t> sentBefore = harness.sentCodes;

    harness.scheduler.reset_pending();                                // link dropped
    ASSERT_TRUE(harness.fireTimeoutFor(kStandbyCode));                // stale timer from the dead session

    EXPECT_EQ(harness.abandonCount, 0);                               // did not release anyone's claim
    EXPECT_EQ(harness.sentCodes, sentBefore);                         // did not continue the dead cycle
}

TEST(RequestScheduler, ResetPendingIgnoresRepliesFromTheDeadSession) {
    SchedulerHarness harness;
    harness.startCycle();
    harness.scheduler.reset_pending();

    EXPECT_TRUE(harness.scheduler.process_response(kSettingsCode));   // managed, but not awaiting
    EXPECT_TRUE(harness.respondedCodes.empty());
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode}));
}

TEST(RequestScheduler, ResetPendingKeepsFailureCountsAndDisabledState) {
    SchedulerHarness harness;
    for (int attempt = 0; attempt < 3; ++attempt) {
        harness.startCycle();
        ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
        ASSERT_TRUE(harness.fireTimeoutFor(kStandbyCode));
        ASSERT_TRUE(harness.scheduler.process_response(kStatusCode));
    }                                                                 // standby is now disabled

    harness.scheduler.reset_pending();
    harness.sentCodes.clear();
    harness.startCycle();
    ASSERT_TRUE(harness.scheduler.process_response(kSettingsCode));
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStatusCode}));  // still skipped
}

TEST(RequestScheduler, CycleWorksNormallyAfterReset) {
    SchedulerHarness harness;
    harness.startCycle();
    harness.scheduler.reset_pending();
    harness.sentCodes.clear();

    harness.startCycle();
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode}));
    EXPECT_TRUE(harness.scheduler.process_response(kSettingsCode));
    EXPECT_EQ(harness.sentCodes, (std::vector<uint8_t>{kSettingsCode, kStandbyCode}));
}

TEST(RequestScheduler, FailedSendDoesNotMarkAwaitingOrArmTimeout) {
    bool sendOk = false, responded = false;
    std::vector<std::string> timeouts;
    RequestScheduler scheduler(
        [&sendOk](uint8_t) { return sendOk; },
        [&timeouts](const std::string& n, uint32_t, std::function<void()>) { timeouts.push_back(n); });
    InfoRequest standby("standby", "Standby", kStandbyCode, 3, kStandbySoftTimeoutMs);
    standby.onResponse = [&responded](CN105Climate&) { responded = true; };
    scheduler.register_request(standby);

    scheduler.send_next_after(0x00);                       // send fails

    EXPECT_TRUE(timeouts.empty());                         // no timer for a request that never went out
    EXPECT_TRUE(scheduler.process_response(kStandbyCode)); // managed...
    EXPECT_FALSE(responded);                               // ...but not awaiting, so ignored
}

TEST(RequestScheduler, FailedSendDoesNotConsumeTheIntervalSlot) {
    bool sendOk = false;
    std::vector<uint8_t> sent;
    RequestScheduler scheduler([&](uint8_t c) { if (sendOk) sent.push_back(c); return sendOk; });
    InfoRequest funcs("functions1", "Functions", 0x20, 3, 0, /*interval_ms=*/60000);
    scheduler.register_request(funcs);

    scheduler.send_next_after(0x00);                       // fails
    sendOk = true;
    scheduler.send_next_after(0x00);                       // must retry, not skip for the interval

    EXPECT_EQ(sent, (std::vector<uint8_t>{0x20}));
}
