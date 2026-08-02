/// test_mode_preservation.cpp — Regression tests for the HEAT_COOL mode-clobber
/// bug in checkPowerAndModeSettings (components/cn105/hp_readings.cpp).
///
/// Bug: a dual-setpoint unit driven in HEAT_COOL runs the heat pump in hardware
/// AUTO; the unit then reports its *active operating direction* ("HEAT"/"COOL")
/// in the settings packet. The unguarded HEAT/COOL branches used to overwrite
/// this->mode, dropping the user out of HEAT_COOL and collapsing the dual band.
/// Only the AUTO branch was guarded. The fix guards HEAT and COOL the same way,
/// gated on supports_dual_setpoint_ so genuine single-setpoint builds are
/// unaffected.
///
/// Pattern: standalone reimplementation of the mode-selection ladder (same
/// approach as test_calculate_temp.cpp / test_real_frames.cpp), parameterised on
/// (power, reportedMode, currentMode, supportsDual). The production code is a
/// method on the full CN105Climate object and the gtest mocks do not define
/// esphome::climate::ClimateMode, so a local enum mirror is used. This mirror is
/// kept byte-faithful to the production ladder below it.
#include <gtest/gtest.h>
#include <cstring>

namespace {

enum class Mode { OFF, HEAT, COOL, DRY, FAN_ONLY, AUTO, HEAT_COOL };

// Mirror of CN105Climate::checkPowerAndModeSettings()'s mode ladder
// (hp_readings.cpp). `current` is this->mode on entry; the return value is
// this->mode on exit.
Mode resolveMode(const char* power, const char* reported,
                 Mode current, bool supportsDual) {
    if (std::strcmp(power, "ON") != 0) {
        return Mode::OFF;
    }
    const bool holdHeatCool =
        supportsDual && current == Mode::HEAT_COOL;
    if (std::strcmp(reported, "HEAT") == 0) {
        return holdHeatCool ? current : Mode::HEAT;
    } else if (std::strcmp(reported, "DRY") == 0) {
        return Mode::DRY;
    } else if (std::strcmp(reported, "COOL") == 0) {
        return holdHeatCool ? current : Mode::COOL;
    } else if (std::strcmp(reported, "FAN") == 0) {
        return Mode::FAN_ONLY;
    } else if (std::strcmp(reported, "AUTO") == 0) {
        // Pre-existing guard: stay in HEAT_COOL even if HP says AUTO.
        return current == Mode::HEAT_COOL ? current : Mode::AUTO;
    }
    // Unknown mode: production logs a warning and leaves this->mode unchanged.
    return current;
}

}  // namespace

// ── The bug repro: in HEAT_COOL, a reported operating direction must not flip us ──

TEST(ModePreservationTest, HeatCool_Preserved_WhenUnitReportsCool) {
    EXPECT_EQ(resolveMode("ON", "COOL", Mode::HEAT_COOL, /*dual=*/true),
              Mode::HEAT_COOL);
}

TEST(ModePreservationTest, HeatCool_Preserved_WhenUnitReportsHeat) {
    EXPECT_EQ(resolveMode("ON", "HEAT", Mode::HEAT_COOL, /*dual=*/true),
              Mode::HEAT_COOL);
}

TEST(ModePreservationTest, HeatCool_Preserved_WhenUnitReportsAuto) {
    // Pins the pre-existing AUTO guard.
    EXPECT_EQ(resolveMode("ON", "AUTO", Mode::HEAT_COOL, /*dual=*/true),
              Mode::HEAT_COOL);
}

// ── A genuine user COOL/HEAT command still applies ──
// (processModeChange sets this->mode before this read-path runs, and the
// read-path is skipped entirely while a user demand is pending; once it clears,
// current already equals the user's chosen mode, so the guard does not trap it.)

TEST(ModePreservationTest, UserCool_StillApplies) {
    EXPECT_EQ(resolveMode("ON", "COOL", Mode::COOL, /*dual=*/true),
              Mode::COOL);
}

TEST(ModePreservationTest, UserHeat_StillApplies) {
    EXPECT_EQ(resolveMode("ON", "HEAT", Mode::HEAT, /*dual=*/true),
              Mode::HEAT);
}

// ── Single-setpoint builds are unaffected by the guard ──

TEST(ModePreservationTest, SingleSetpointBuild_CoolApplies) {
    EXPECT_EQ(resolveMode("ON", "COOL", Mode::HEAT_COOL, /*dual=*/false),
              Mode::COOL);
}

TEST(ModePreservationTest, SingleSetpointBuild_HeatApplies) {
    EXPECT_EQ(resolveMode("ON", "HEAT", Mode::HEAT_COOL, /*dual=*/false),
              Mode::HEAT);
}

// ── Other modes / power unchanged ──

TEST(ModePreservationTest, Dry_AppliesEvenFromHeatCool) {
    EXPECT_EQ(resolveMode("ON", "DRY", Mode::HEAT_COOL, /*dual=*/true),
              Mode::DRY);
}

TEST(ModePreservationTest, Fan_AppliesEvenFromHeatCool) {
    EXPECT_EQ(resolveMode("ON", "FAN", Mode::HEAT_COOL, /*dual=*/true),
              Mode::FAN_ONLY);
}

TEST(ModePreservationTest, PowerOff_AlwaysOff) {
    EXPECT_EQ(resolveMode("OFF", "COOL", Mode::HEAT_COOL, /*dual=*/true),
              Mode::OFF);
    EXPECT_EQ(resolveMode("OFF", "HEAT", Mode::HEAT, /*dual=*/false),
              Mode::OFF);
}
