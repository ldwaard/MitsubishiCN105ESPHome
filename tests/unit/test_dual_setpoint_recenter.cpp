/// test_dual_setpoint_recenter.cpp — Regression tests for issue #673:
/// HEAT_COOL dual-setpoint band must never be re-derived from the transmitted
/// single setpoint.
/// Deps: localization.h (production), esphome_stubs.h
///
/// Pattern: standalone reimplementation of the band logic around the PRODUCTION
/// FahrenheitSupport class (same approach as test_calculate_temp.cpp), with
/// `this->mode` replaced by a parameter. Mirrors:
///   - the get/set target-temperature wrappers (utils.cpp): every store crosses
///     HP→UI, every read crosses UI→HP through the Fahrenheit table;
///   - calculateTemperatureSetting (utils.cpp, encoding B);
///   - the HEAT_COOL deadband from controlTemperature (climateControls.cpp):
///     in-band → transmitted setpoint = current room temperature;
///   - updateTargetTemperaturesFromSettings (utils.cpp) dispatch, both the
///     FIXED version (AUTO and HEAT_COOL preserve the band) and the PRE-FIX
///     version (HEAT_COOL falls into the generic recentring else-branch) so the
///     ratchet itself is demonstrated and pinned.
///
/// Hardware-observed bug (issue #673, fahrenheit_compatibility "standard",
/// encoding B): band 61/82°F crept to 62/83 then 63/84 (requested +1.0°C in two
/// +0.5°C passes, width preserved); 65/71°F was coincidentally stable.
#include <gtest/gtest.h>
#include <cmath>
#include "esphome_stubs.h"
#include "localization.h"

using esphome::FahrenheitMode;
using esphome::FahrenheitSupport;

namespace {

enum class Mode { HEAT, COOL, DRY, AUTO, HEAT_COOL };

// Mirror of calculateTemperatureSetting (utils.cpp), encoding B
float calcSettingB(float s) {
    s = std::round(2.0f * s) / 2.0f;
    return s < 10.0f ? 10.0f : (s > 31.0f ? 31.0f : s);
}

struct BandSim {
    FahrenheitSupport fahr;
    // ESPHome base-class storage is UI-Celsius
    float low_ui = NAN;
    float high_ui = NAN;

    // Mirrors of setTargetTemperatureLow/High (utils.cpp): store HP→UI
    void setLow(float hp) { low_ui = fahr.normalizeHeatpumpTemperatureToUiTemperature(hp); }
    void setHigh(float hp) { high_ui = fahr.normalizeHeatpumpTemperatureToUiTemperature(hp); }
    // Mirrors of getTargetTemperatureLow/High (utils.cpp): read UI→HP
    float getLow() { return fahr.normalizeUiTemperatureToHeatpumpTemperature(low_ui); }
    float getHigh() { return fahr.normalizeUiTemperatureToHeatpumpTemperature(high_ui); }

    // Mirror of the user write path: processTemperatureChange converts the
    // ClimateCall UI values to HP space, handleDualSetpointBoth stores them.
    void userSetsBandF(int lowF, int highF) {
        const float lowHp = fahr.normalizeUiTemperatureToHeatpumpTemperature((lowF - 32.0f) / 1.8f);
        const float highHp = fahr.normalizeUiTemperatureToHeatpumpTemperature((highF - 32.0f) / 1.8f);
        setLow(lowHp);
        setHigh(highHp);
    }

    // Mirror of the controlTemperature HEAT_COOL deadband (climateControls.cpp)
    float deadband(float currentHp) {
        const float lo = getLow();
        const float hi = getHigh();
        if (currentHp < lo) return lo;
        if (currentHp > hi) return hi;
        return currentHp;
    }

    // Mirror of updateTargetTemperaturesFromSettings (utils.cpp) — FIXED
    // dispatch: AUTO and HEAT_COOL both take the band-preserving branch.
    void updateFromSettingsFixed(Mode mode, float temperature) {
        updateFromSettings(mode, temperature, /*heatCoolPreserves=*/true);
    }

    // PRE-FIX dispatch: HEAT_COOL falls through to the recentring else-branch.
    void updateFromSettingsPreFix(Mode mode, float temperature) {
        updateFromSettings(mode, temperature, /*heatCoolPreserves=*/false);
    }

private:
    void updateFromSettings(Mode mode, float temperature, bool heatCoolPreserves) {
        const bool preserving =
            (mode == Mode::AUTO) || (mode == Mode::HEAT_COOL && heatCoolPreserves);
        if (mode == Mode::HEAT) {
            setLow(temperature);
            if (std::isnan(getHigh())) setHigh(temperature);
        } else if (mode == Mode::COOL || mode == Mode::DRY) {
            setHigh(temperature);
            if (std::isnan(getLow())) setLow(temperature);
        } else if (preserving) {
            const bool lowDefined = !std::isnan(getLow());
            const bool highDefined = !std::isnan(getHigh());
            if (lowDefined && highDefined) {
                // keep dual setpoints
            } else if (lowDefined) {
                setHigh(getLow() + 2.0f);
            } else if (highDefined) {
                setLow(getHigh() - 2.0f);
            } else {
                setLow(temperature - 2.0f);
                setHigh(temperature + 2.0f);
            }
        } else {
            // generic else-branch (the pre-fix HEAT_COOL path): recentre the
            // band around the quantized median when it differs from the
            // reported/sent setpoint.
            if (std::isnan(getLow())) setLow(temperature);
            if (std::isnan(getHigh())) setHigh(temperature);
            const float theoretical = calcSettingB((getLow() + getHigh()) / 2.0f);
            if (theoretical != temperature) {
                const float delta = (getHigh() - getLow()) / 2.0f;
                setLow(theoretical - delta);
                setHigh(theoretical + delta);
            }
        }
    }
};

// One full firmware pass: the write-path publish + the read-path settings sync
// both call updateTargetTemperaturesFromSettings with the deadband value.
void runFixedPasses(BandSim& sim, float currentHp, int passes) {
    for (int i = 0; i < passes; ++i) {
        const float sent = calcSettingB(sim.deadband(currentHp));
        sim.updateFromSettingsFixed(Mode::HEAT_COOL, sent);  // write path
        sim.updateFromSettingsFixed(Mode::HEAT_COOL, sent);  // read path
    }
}

int displayedF(float ui) { return static_cast<int>(std::lround(ui * 1.8f + 32.0f)); }

}  // namespace

// ========================================================
// The #673 repro: band 61/82°F, room 66°F (in-band), STANDARD table.
// With the fix the band must not move — pre-fix it crept to 63/84.
// ========================================================

TEST(DualSetpointRecenterTest, HeatCool_Standard_61_82F_BandNeverMoves) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::STANDARD);
    sim.userSetsBandF(61, 82);
    const float low0 = sim.getLow();
    const float high0 = sim.getHigh();

    runFixedPasses(sim, /*current 66F=*/18.5f, /*passes=*/3);

    EXPECT_FLOAT_EQ(sim.getLow(), low0);
    EXPECT_FLOAT_EQ(sim.getHigh(), high0);
    EXPECT_EQ(displayedF(sim.low_ui), 61);
    EXPECT_EQ(displayedF(sim.high_ui), 82);
}

TEST(DualSetpointRecenterTest, HeatCool_Alt_61_82F_BandNeverMoves) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::ALT);
    sim.userSetsBandF(61, 82);
    const float low0 = sim.getLow();
    const float high0 = sim.getHigh();

    runFixedPasses(sim, 18.5f, 3);

    EXPECT_FLOAT_EQ(sim.getLow(), low0);
    EXPECT_FLOAT_EQ(sim.getHigh(), high0);
}

TEST(DualSetpointRecenterTest, HeatCool_CelsiusOff_BandNeverMoves) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::OFF);
    sim.low_ui = 16.0f;
    sim.high_ui = 27.5f;

    runFixedPasses(sim, 18.5f, 3);

    EXPECT_FLOAT_EQ(sim.getLow(), 16.0f);
    EXPECT_FLOAT_EQ(sim.getHigh(), 27.5f);
}

TEST(DualSetpointRecenterTest, HeatCool_Standard_65_71F_StaysExact) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::STANDARD);
    sim.userSetsBandF(65, 71);

    runFixedPasses(sim, 18.5f, 3);

    EXPECT_EQ(displayedF(sim.low_ui), 65);
    EXPECT_EQ(displayedF(sim.high_ui), 71);
}

// Deadband edge: current temperature outside the band transmits a bound; the
// band itself must still not move.
TEST(DualSetpointRecenterTest, HeatCool_Standard_OutOfBandCurrent_BandNeverMoves) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::STANDARD);
    sim.userSetsBandF(61, 82);
    const float low0 = sim.getLow();
    const float high0 = sim.getHigh();

    runFixedPasses(sim, /*current 55F, below band=*/12.8f, 3);

    EXPECT_FLOAT_EQ(sim.getLow(), low0);
    EXPECT_FLOAT_EQ(sim.getHigh(), high0);
}

// ========================================================
// Behavior of the other modes is unchanged by the fix.
// ========================================================

TEST(DualSetpointRecenterTest, LegacyAuto_KeepsDefinedBand) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::STANDARD);
    sim.userSetsBandF(65, 75);
    const float low0 = sim.getLow();
    const float high0 = sim.getHigh();

    sim.updateFromSettingsFixed(Mode::AUTO, 21.0f);

    EXPECT_FLOAT_EQ(sim.getLow(), low0);
    EXPECT_FLOAT_EQ(sim.getHigh(), high0);
}

TEST(DualSetpointRecenterTest, Heat_AssignsLowDirectly) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::STANDARD);
    sim.userSetsBandF(61, 82);

    sim.updateFromSettingsFixed(Mode::HEAT, 18.0f);  // unit reports 18.0C (65F)

    EXPECT_NEAR(sim.getLow(), 18.0f, 1e-4f);
}

TEST(DualSetpointRecenterTest, HeatCool_NaNBounds_InitializedAroundSetpoint) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::OFF);

    sim.updateFromSettingsFixed(Mode::HEAT_COOL, 21.0f);

    EXPECT_FLOAT_EQ(sim.getLow(), 19.0f);
    EXPECT_FLOAT_EQ(sim.getHigh(), 23.0f);
}

// ========================================================
// The ratchet itself, pinned: with the PRE-FIX dispatch the same scenario
// creeps +0.5°C per pass on both bounds (this is what issue #673 observed
// on hardware as 61/82 → 62/83 → 63/84). If this test ever fails, the
// mirror has drifted from the analysis — re-verify against utils.cpp.
// ========================================================

TEST(DualSetpointRecenterTest, PreFixDispatch_DemonstratesTheRatchet) {
    BandSim sim;
    sim.fahr.setUseFahrenheitSupportMode(FahrenheitMode::STANDARD);
    sim.userSetsBandF(61, 82);
    const float low0 = sim.getLow();    // 16.0
    const float high0 = sim.getHigh();  // 27.5

    const float sent = calcSettingB(sim.deadband(18.5f));  // in-band → 18.5
    sim.updateFromSettingsPreFix(Mode::HEAT_COOL, sent);   // write-path pass
    sim.updateFromSettingsPreFix(Mode::HEAT_COOL, sent);   // read-path pass

    EXPECT_NEAR(sim.getLow(), low0 + 1.0f, 1e-3f);    // 61F → 63F
    EXPECT_NEAR(sim.getHigh(), high0 + 1.0f, 1e-3f);  // 82F → 84F
    EXPECT_EQ(displayedF(sim.low_ui), 63);
    EXPECT_EQ(displayedF(sim.high_ui), 84);
}
