/// test_setpoint_persistence.cpp — Regression tests for HEAT_COOL band
/// persistence across reboots (opt-in `supports.restore_setpoints`).
///
/// Bug: the dual-setpoint band is synthetic — a Mitsubishi unit stores only a
/// single setpoint and runs HEAT_COOL as hardware AUTO, so the user's low/high
/// band lives only in RAM. On reboot, setup() NANs target_temperature_low/high
/// (componentEntries.cpp) and the unit's hardware-AUTO report drives the entity
/// back to CLIMATE_MODE_AUTO (hp_readings.cpp), losing both the HEAT_COOL mode
/// and the band.
///
/// Fix (opt-in): persist {version, mode, low, high} to flash on every applied
/// control, and in setup() re-seed HEAT_COOL + the band before the first
/// settings read. The existing AUTO-branch guard in checkPowerAndModeSettings()
/// then keeps HEAT_COOL; an explicit HEAT/COOL/OFF reported by the unit (user
/// changed it on the IR remote while powered off) still overrides the seed
/// (remote wins), so no extra conflict handling is needed.
///
/// Pattern: standalone reimplementation of the restore decision (same approach
/// as test_mode_preservation.cpp / test_calculate_temp.cpp). The production code
/// is a method on the full CN105Climate object using esphome preferences and
/// climate::ClimateMode, which the gtest mocks do not provide, so a local mirror
/// is used. The mirror is kept faithful to restore_setpoint_state_() /
/// save_setpoint_state_() in components/cn105/climateControls.cpp.
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

namespace {

// Local mirror of esphome::climate::ClimateMode (numeric values are irrelevant
// to the decision; only "is it HEAT_COOL" matters, mirroring the production
// `s.mode == static_cast<uint8_t>(climate::CLIMATE_MODE_HEAT_COOL)` check).
enum class Mode : uint8_t { OFF, HEAT_COOL, COOL, HEAT, FAN_ONLY, DRY, AUTO };

constexpr uint8_t kStateVersion = 1;

// Mirror of CN105Climate::SetpointState (cn105.h).
struct SetpointState {
    uint8_t version;
    uint8_t mode;
    float target_low;
    float target_high;
};

struct RestoreResult {
    bool seed;        // did we re-seed HEAT_COOL + band?
    Mode mode;
    float low;
    float high;
};

// Mirror of CN105Climate::save_setpoint_state_() (climateControls.cpp):
// packs the current mode + band into the persisted struct.
SetpointState saveState(Mode mode, float low, float high) {
    return SetpointState{kStateVersion, static_cast<uint8_t>(mode), low, high};
}

// Mirror of CN105Climate::restore_setpoint_state_() (climateControls.cpp):
// decides whether to re-seed HEAT_COOL + band on boot.
RestoreResult restoreDecision(bool restoreEnabled, bool loaded,
                              const SetpointState& s, bool supportsDual) {
    RestoreResult r{false, Mode::OFF, NAN, NAN};
    if (!restoreEnabled) return r;                                   // feature off
    if (!loaded) return r;                                           // nothing in flash
    if (s.version != kStateVersion) return r;                        // version mismatch
    if (s.mode != static_cast<uint8_t>(Mode::HEAT_COOL)) return r;   // only HEAT_COOL is synthetic
    if (std::isnan(s.target_low) || std::isnan(s.target_high)) return r;
    if (!supportsDual) return r;                                     // dual_setpoint not enabled
    return RestoreResult{true, Mode::HEAT_COOL, s.target_low, s.target_high};
}

}  // namespace

// ── Happy path: a saved HEAT_COOL band is restored exactly ──

TEST(SetpointPersistenceTest, RestoresHeatCoolBand) {
    auto s = saveState(Mode::HEAT_COOL, 20.0f, 24.0f);
    auto r = restoreDecision(/*restoreEnabled=*/true, /*loaded=*/true, s, /*dual=*/true);
    EXPECT_TRUE(r.seed);
    EXPECT_EQ(r.mode, Mode::HEAT_COOL);
    EXPECT_FLOAT_EQ(r.low, 20.0f);
    EXPECT_FLOAT_EQ(r.high, 24.0f);
}

TEST(SetpointPersistenceTest, SaveRestoreRoundTripIsLossless) {
    auto s = saveState(Mode::HEAT_COOL, 18.5f, 23.5f);
    auto r = restoreDecision(true, true, s, true);
    ASSERT_TRUE(r.seed);
    EXPECT_FLOAT_EQ(r.low, 18.5f);
    EXPECT_FLOAT_EQ(r.high, 23.5f);
}

// ── Opt-in: nothing happens unless the feature is enabled ──

TEST(SetpointPersistenceTest, DisabledDoesNotRestore) {
    auto s = saveState(Mode::HEAT_COOL, 20.0f, 24.0f);
    EXPECT_FALSE(restoreDecision(/*restoreEnabled=*/false, true, s, true).seed);
}

TEST(SetpointPersistenceTest, NoSavedStateDoesNotRestore) {
    SetpointState empty{};  // load() failed
    EXPECT_FALSE(restoreDecision(true, /*loaded=*/false, empty, true).seed);
}

// ── Guards: never restore a stale / non-HEAT_COOL / invalid record ──

TEST(SetpointPersistenceTest, VersionMismatchDoesNotRestore) {
    SetpointState s{kStateVersion + 1u, static_cast<uint8_t>(Mode::HEAT_COOL), 20.0f, 24.0f};
    EXPECT_FALSE(restoreDecision(true, true, s, true).seed);
}

TEST(SetpointPersistenceTest, SingleModeDoesNotRestore) {
    // Single modes (HEAT/COOL) are persisted by the heat pump itself, so we
    // must not force HEAT_COOL when the last saved mode was single.
    auto s = saveState(Mode::COOL, 22.0f, 22.0f);
    EXPECT_FALSE(restoreDecision(true, true, s, true).seed);
}

TEST(SetpointPersistenceTest, NanBandDoesNotRestore) {
    auto s = saveState(Mode::HEAT_COOL, NAN, 24.0f);
    EXPECT_FALSE(restoreDecision(true, true, s, true).seed);
}

TEST(SetpointPersistenceTest, DualNotSupportedDoesNotRestore) {
    auto s = saveState(Mode::HEAT_COOL, 20.0f, 24.0f);
    EXPECT_FALSE(restoreDecision(true, true, s, /*dual=*/false).seed);
}
