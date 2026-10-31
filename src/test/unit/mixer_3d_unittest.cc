/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// 3D mode motor outputs, both directions: idle, dynamic idle (3d_dyn_idle) and the DShot 3D ranges

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "config/feature.h"
    #include "drivers/dshot.h"
    #include "drivers/motor.h"
    #include "fc/controlrate_profile.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/mixer.h"
    #include "flight/mixer_init.h"
    #include "flight/pid.h"
    #include "pg/motor.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "rx/rx.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    #include "io/gps.h"
    #include "pg/rpm_filter.h"
    #include "sensors/battery.h"
    #include "flight/failsafe.h"
    #include "flight/mixer_tricopter.h"
    #include "fc/core.h"
    #include "fc/rc.h"
    #include "drivers/time.h"

    PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
    PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t armingFlags;
    uint16_t flightModeFlags;
    uint8_t stateFlags;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    pidAxisData_t pidData[XYZ_AXIS_COUNT];
    gpsSolutionData_t gpsSol;
    bool useDshotTelemetry;

    static pidProfile_t testPidProfile;
    pidProfile_t *currentPidProfile = &testPidProfile;
    static controlRateConfig_t testRateProfile;
    controlRateConfig_t *currentControlRateProfile = &testRateProfile;

    static bool testFeature3d = true;
    static bool testDshot = true;
    static float testDT = 1.0f / 8000.0f;
    static uint16_t testSagCellVoltage = 380;
    uint16_t getBatterySagCellVoltage(void) { return testSagCellVoltage; }
    static bool testAntiGravityReversed, testAntiGravityHolding, testAntiGravityApproaching;

    bool featureIsEnabled(uint32_t mask) { return mask == FEATURE_3D ? testFeature3d : false; }
    bool airmodeIsEnabled(void) { return true; }
    bool isAirmodeActivated(void) { return true; }
    throttleStatus_e calculateThrottleStatus(void) { return THROTTLE_HIGH; }
    bool failsafeIsActive(void) { return false; }
    float getDigitalIdleOffset(const motorConfig_t *cfg) { return cfg->digitalIdleOffsetValue * 0.0001f; }
    float getRcDeflection(int) { return 0.0f; }
    float getRcDeflectionAbs(int) { return 0.0f; }
    float getMaxRcDeflectionAbs(void) { return 0.0f; }
    bool isFlipOverAfterCrashActive(void) { return false; }
    bool isLaunchControlActive(void) { return false; }
    bool isMotorProtocolDshot(void) { return testDshot; }
    bool isMotorsReversed(void) { return false; }
    bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
    void mixerTricopterInit(void) {}
    float mixerTricopterMotorCorrection(int) { return 0.0f; }
    unsigned motorDeviceCount(void) { return 4; }
    void motorInitEndpoints(const motorConfig_t *cfg, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
    {
        dshotInitEndpoints(cfg, outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
    }
    void motorWriteAll(float *) {}
    void delay(timeMs_t) {}
    float pidGetDT() { return testDT; }
    float pidGetPidFrequency() { return 1.0f / testDT; }
    void pidResetIterm(void) {}
    void pidSetItermFreeze3d(bool) {}
    void pidUpdateTpaFactor(float) {}
    void pidUpdateAntiGravityThrottleFilter(float) {}
    void pidAntiGravity3dReversal(bool reversed, bool holding, bool approaching) { testAntiGravityReversed = reversed; testAntiGravityHolding = holding; testAntiGravityApproaching = approaching; }
}

// DShot bidirectional telemetry word for an eRPM value (GCR period format eeem mmmm mmmm), decoded by the real dshot.c
static uint16_t encodeErpm(float rpm, int poles)
{
    const float erpm100 = rpm * (poles / 2.0f) / 100.0f;
    if (erpm100 < 1.0f) {
        return 0x0fff;   // motor stopped
    }
    uint32_t period = lrintf(600000.0f / erpm100);
    int e = 0;
    while (period > 0x1ff) {
        period >>= 1;
        e++;
    }
    return (e << 9) | period;
}

static const int POLES = 12;
static timeUs_t nowUs = 0;

static void setAllMotorRpm(float rpm)
{
    for (int i = 0; i < 4; i++) {
        dshotTelemetryState.motorState[i].rawValue = encodeErpm(rpm, POLES);
        dshotTelemetryState.motorState[i].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_eRPM;
    }
}

// one PID loop: telemetry in, mixer out
static void runLoops(int loops, float rpm)
{
    for (int n = 0; n < loops; n++) {
        setAllMotorRpm(rpm);
        dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
        updateDshotTelemetry();
        nowUs += lrintf(testDT * 1e6f);
        mixTable(nowUs);
    }
}

// flat for the motor reading: in 3D the mixer outputs one direction's DShot band (forward 1048-2047, reverse 48-1047)
static float minMotor(void) { float m = 1e9f; for (int i = 0; i < 4; i++) m = MIN(m, motor[i]); return m; }
static float maxMotor(void) { float m = -1e9f; for (int i = 0; i < 4; i++) m = MAX(m, motor[i]); return m; }

static void setup(bool dynIdle3d, uint8_t dynIdleMinRpm)
{
    nowUs = 0;
    testFeature3d = true;
    testDshot = true;
    useDshotTelemetry = true;
    memset(pidData, 0, sizeof(pidData));
    memset(&dshotTelemetryState, 0, sizeof(dshotTelemetryState));

    pgResetAll();
    debugMode = DEBUG_3D_ITERM;
    flight3DConfigMutable()->deadband3d_throttle = 3;
    flight3DConfigMutable()->iterm_reset_ms3d = 250;   // firmware defaults (this PG is registered here without its reset template)
    flight3DConfigMutable()->iterm_min_ms3d = 50;
    flight3DConfigMutable()->iterm_max_ms3d = 400;
    flight3DConfigMutable()->iterm_rpm_pct3d = 40;
    flight3DConfigMutable()->iterm_learn_hz_x10 = 5;
    flight3DConfigMutable()->iterm_learn_thr3d = 50;
    flight3DConfigMutable()->ag_approach_pct3d = 0;
    flight3DConfigMutable()->dyn_idle3d = dynIdle3d;
    motorConfigMutable()->digitalIdleOffsetValue = 150;   // dshot_idle_value 1.5%
    motorConfigMutable()->motorPoleCount = POLES;
    motorConfigMutable()->dev.useDshotTelemetry = true;
    rxConfigMutable()->midrc = 1500;
    rpmFilterConfigMutable()->rpm_filter_lpf_hz = 150;
    mixerConfigMutable()->mixerMode = MIXER_QUADX;
    mixerConfigMutable()->mixer_type = MIXER_LEGACY;

    memset(&testPidProfile, 0, sizeof(testPidProfile));
    testPidProfile.pidSumLimit = 500;
    testPidProfile.motor_output_limit = 100;
    testPidProfile.vbat_sag_compensation = 0;
    batteryConfigMutable()->vbatwarningcellvoltage = 350;
    testSagCellVoltage = 380;
    testPidProfile.pidSumLimitYaw = 400;
    testPidProfile.dyn_idle_min_rpm = dynIdleMinRpm;
    testPidProfile.dyn_idle_p_gain = 50;
    testPidProfile.dyn_idle_i_gain = 50;
    testPidProfile.dyn_idle_d_gain = 50;
    testPidProfile.dyn_idle_max_increase = 100;    // 10%
    testPidProfile.dyn_idle_start_increase = 50;
    memset(&testRateProfile, 0, sizeof(testRateProfile));
    testRateProfile.throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF;
    testRateProfile.throttle_limit_percent = 100;

    initDshotTelemetry(lrintf(testDT * 1e6f));
    mixerInit(MIXER_QUADX);
    mixerInitProfile();
    ENABLE_ARMING_FLAG(ARMED);
}

// motor direction from the 3D_ITERM debug (slot 0 = mixer sign)
static int motorDirection(void) { return debug[0]; }

static void setThrottle(float rc) { rcCommand[THROTTLE] = rc; rcData[THROTTLE] = rc; }

// DShot 3D bands (motor values), from the real dshotInitEndpoints with dshot_idle_value 1.5%
static float forwardIdle(void) { return DSHOT_3D_FORWARD_MIN_THROTTLE + 0.015f * (DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE); }
static float reverseIdle(void) { return DSHOT_MIN_THROTTLE + 0.015f * (DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - DSHOT_MIN_THROTTLE); }

TEST(Mixer3dTest, stockOutputsStayInEachDirectionsBand)
{
    setup(false, 0);
    setThrottle(1700);
    runLoops(10, 20000);
    EXPECT_EQ(1, motorDirection());
    EXPECT_GE(minMotor(), forwardIdle() - 0.5f);
    EXPECT_LE(maxMotor(), DSHOT_MAX_THROTTLE);

    setThrottle(1300);
    runLoops(10, 20000);
    EXPECT_EQ(-1, motorDirection());
    EXPECT_GE(minMotor(), reverseIdle() - 0.5f);
    EXPECT_LE(maxMotor(), DSHOT_3D_FORWARD_MIN_THROTTLE - 1);
}

TEST(Mixer3dTest, sameThrottleMagnitudeSamePlaceInBothBands)
{
    setup(false, 0);
    setThrottle(1700);
    runLoops(10, 20000);
    const float fwd = (motor[0] - forwardIdle()) / (DSHOT_MAX_THROTTLE - forwardIdle());
    setThrottle(1300);
    runLoops(10, 20000);
    const float rev = (motor[0] - reverseIdle()) / (DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - reverseIdle());
    EXPECT_NEAR(fwd, rev, 0.002f);
    EXPECT_NEAR(fwd, 197.0f / 497.0f, 0.01f);   // (1700 - 1503) / (2000 - 1503)
}

TEST(Mixer3dTest, stockIgnoresDynIdleIn3d)
{
    // stock 4.5.1: dyn_idle_min_rpm set, motors stopped -> 3D motors stay at static idle (no rpm control in 3D)
    setup(false, 50);
    setThrottle(1503);   // stick at the top of the deadband: zero throttle forward
    runLoops(10, 20000);
    setThrottle(1501);   // inside the deadband -> NORMAL_TO_DEADBAND, motors idle forward
    runLoops(200, 0);
    // ...except the stock 1% zero-throttle offset, which is applied in 3D too
    EXPECT_NEAR(minMotor(), forwardIdle() + 0.01f * (DSHOT_MAX_THROTTLE - forwardIdle()), 0.6f);
}

static void expectDynIdleRaise(float rc, bool forward)
{
    const float bandLow = forward ? forwardIdle() : reverseIdle();
    const float bandHigh = forward ? DSHOT_MAX_THROTTLE : DSHOT_3D_FORWARD_MIN_THROTTLE - 1;

    setup(false, 50);
    setThrottle(rc);
    runLoops(20, 20000);
    runLoops(200, 0);
    const float off = minMotor();

    setup(true, 50);
    setThrottle(rc);
    runLoops(20, 20000);
    runLoops(200, 0);
    const float on = minMotor();

    EXPECT_EQ(forward ? 1 : -1, motorDirection());
    EXPECT_GT(on, off + 5.0f);                                    // raises the motors of THIS direction (reverse: toward 1047 = more reverse thrust)
    EXPECT_GE(on, bandLow + 0.10f * (bandHigh - bandLow) - 1.0f);  // saturated: idle + dyn_idle_max_increase (10%) of the band
    EXPECT_LE(maxMotor(), bandHigh);                               // never leaves the band (never crosses into the other direction)
    EXPECT_GE(minMotor(), bandLow);
}

TEST(Mixer3dTest, dynIdleRaisesForwardIdleWhenMotorsStopped) { expectDynIdleRaise(1503, true); }
TEST(Mixer3dTest, dynIdleRaisesReverseIdleWhenMotorsStopped) { expectDynIdleRaise(1497, false); }
TEST(Mixer3dTest, dynIdleRaisesForwardAtLowThrottle) { expectDynIdleRaise(1530, true); }
TEST(Mixer3dTest, dynIdleRaisesReverseAtLowThrottle) { expectDynIdleRaise(1470, false); }

static void expectNoRaiseWhenFast(float rc)
{
    setup(false, 50);
    setThrottle(rc);
    runLoops(300, 20000);
    float off[4];
    for (int i = 0; i < 4; i++) off[i] = motor[i];
    setup(true, 50);
    setThrottle(rc);
    runLoops(300, 20000);
    for (int i = 0; i < 4; i++) {
        EXPECT_NEAR(off[i], motor[i], 0.01f);
    }
}

TEST(Mixer3dTest, dynIdleIdleWhenMotorsFastForward) { expectNoRaiseWhenFast(1700); }
TEST(Mixer3dTest, dynIdleIdleWhenMotorsFastReverse) { expectNoRaiseWhenFast(1300); }

TEST(Mixer3dTest, dynIdleAtCenterStickStaysInLastDirection)
{
    // NORMAL_TO_DEADBAND and INVERTED_TO_DEADBAND: stick centered keeps the last direction; the raise goes there
    setup(true, 50);
    setThrottle(1300);
    runLoops(20, 20000);
    setThrottle(1500);
    runLoops(200, 0);
    EXPECT_EQ(-1, motorDirection());
    EXPECT_GT(minMotor(), reverseIdle() + 50.0f);
    EXPECT_LE(maxMotor(), DSHOT_3D_FORWARD_MIN_THROTTLE - 1);

    setup(true, 50);
    setThrottle(1700);
    runLoops(20, 20000);
    setThrottle(1500);
    runLoops(200, 0);
    EXPECT_EQ(1, motorDirection());
    EXPECT_GT(minMotor(), forwardIdle() + 50.0f);
    // no 1% zero-throttle offset with 3D dyn idle: at full rpm the center-stick idle is exactly the static idle
    runLoops(4000, 20000);
    EXPECT_NEAR(minMotor(), forwardIdle(), 0.6f);
}

TEST(Mixer3dTest, dynIdleFullThrottleUnchanged)
{
    // the raise compresses the range from the bottom; full throttle still reaches the top of each band
    setup(true, 50);
    setThrottle(2000);
    runLoops(200, 0);
    EXPECT_NEAR(maxMotor(), DSHOT_MAX_THROTTLE, 0.6f);
    setThrottle(1000);
    runLoops(200, 0);
    EXPECT_NEAR(maxMotor(), DSHOT_3D_FORWARD_MIN_THROTTLE - 1, 0.6f);
}

TEST(Mixer3dTest, dynIdleWithoutTelemetryDoesNothing)
{
    setup(true, 50);
    useDshotTelemetry = false;
    mixerInitProfile();
    setThrottle(1497);
    runLoops(200, 0);
    EXPECT_NEAR(minMotor(), reverseIdle(), 0.6f);
}

TEST(Mixer3dTest, reversalTellsAntiGravity)
{
    setup(false, 0);
    flight3DConfigMutable()->anti_gravity3d = ANTI_GRAVITY_3D_GATED;
    setThrottle(1700);
    runLoops(20, 20000);
    setThrottle(1300);
    runLoops(1, 20000);
    EXPECT_TRUE(testAntiGravityReversed);
    EXPECT_TRUE(testAntiGravityHolding);
    runLoops(1, 20000);
    EXPECT_FALSE(testAntiGravityReversed);
    runLoops(8000, 20000);   // 1 s later the I-term hold (250 ms) is over
    EXPECT_FALSE(testAntiGravityHolding);
}

static void expectMixKeptWithDynIdle(float rc)
{
    // a roll correction must push the same motors up in both directions, with or without the raised idle
    float off[4], on[4];
    for (int pass = 0; pass < 2; pass++) {
        setup(pass == 1, 50);
        pidData[FD_ROLL].Sum = 100.0f;
        setThrottle(rc);
        runLoops(20, 20000);
        runLoops(200, 0);
        for (int i = 0; i < 4; i++) (pass ? on : off)[i] = motor[i];
    }
    const bool forward = rc > 1500;
    const float bandLow = forward ? forwardIdle() : reverseIdle();
    const float bandHigh = forward ? DSHOT_MAX_THROTTLE : DSHOT_3D_FORWARD_MIN_THROTTLE - 1;
    for (int i = 0; i < 4; i++) {
        EXPECT_GE(on[i], off[i] - 0.01f);
        EXPECT_GE(on[i], bandLow);
        EXPECT_LE(on[i], bandHigh);
        for (int j = 0; j < 4; j++) {
            if (off[i] > off[j] + 1.0f) {
                EXPECT_GT(on[i], on[j]);   // same motors high / low
            }
        }
    }
}

TEST(Mixer3dTest, dynIdleKeepsRollMixForward) { expectMixKeptWithDynIdle(1530); }
TEST(Mixer3dTest, dynIdleKeepsRollMixReverse) { expectMixKeptWithDynIdle(1470); }

TEST(Mixer3dTest, dynIdleFollowsTheReversal)
{
    // forward at low rpm (raised), then flip to reverse: the raise is applied in the reverse band at once, nothing lands in between
    setup(true, 50);
    setThrottle(1530);
    runLoops(20, 20000);
    runLoops(200, 0);
    EXPECT_GT(minMotor(), forwardIdle() + 50.0f);
    setThrottle(1470);
    runLoops(1, 0);
    EXPECT_EQ(-1, motorDirection());
    EXPECT_GT(minMotor(), reverseIdle() + 50.0f);
    EXPECT_LE(maxMotor(), DSHOT_3D_FORWARD_MIN_THROTTLE - 1);
}

TEST(Mixer3dTest, dynIdleQuickUnwind)
{
    // a reversal saturates dyn idle I (slowest motor ~0 rpm); once the motors are fast again it must be gone within 50 ms,
    // not ~2 s (stock "falls slowly"), or the next flip / center stick would start with raised idle
    setup(true, 50);
    setThrottle(1530);
    runLoops(20, 20000);
    runLoops(200, 0);
    EXPECT_NEAR(mixerRuntime.dynIdleI, 0.10f, 0.001f);
    runLoops(400, 30000);                       // 50 ms at 8 kHz
    EXPECT_NEAR(mixerRuntime.dynIdleI, 0.0f, 0.0001f);
    setThrottle(1500);                          // center stick right after: exact static idle
    runLoops(8, 30000);
    EXPECT_NEAR(minMotor(), forwardIdle(), 0.6f);
}


TEST(Mixer3dTest, antiGravityApproachFlag)
{
    setup(false, 0);
    flight3DConfigMutable()->anti_gravity3d = ANTI_GRAVITY_3D_GATED;
    setThrottle(1700);
    runLoops(800, 20000);
    setThrottle(1560);                          // default 3d_ag_approach_pct 0: approach gate off (GATED = v0.3.0)
    runLoops(1, 20000);
    EXPECT_FALSE(testAntiGravityApproaching);
    flight3DConfigMutable()->ag_approach_pct3d = 15;
    setThrottle(1700);
    runLoops(800, 20000);
    EXPECT_FALSE(testAntiGravityApproaching);
    setThrottle(1560);                          // 57 / 497 = 11% throttle, falling -> approach
    runLoops(1, 20000);
    EXPECT_TRUE(testAntiGravityApproaching);
    setThrottle(1600);                          // rising again in the same direction (punch): not an approach
    runLoops(1, 20000);
    EXPECT_FALSE(testAntiGravityApproaching);
    setThrottle(1580);                          // falling but above 15%: not yet
    runLoops(1, 20000);
    EXPECT_FALSE(testAntiGravityApproaching);
}

TEST(Mixer3dTest, dynIdleSlowFallNearTarget)
{
    // just above dyn_idle_min_rpm (within 25%) the stock slow fall is kept, so I can't hunt around the target
    setup(true, 50);
    setThrottle(1530);
    runLoops(20, 20000);
    runLoops(200, 0);
    const float saturated = mixerRuntime.dynIdleI;
    runLoops(400, 5600);                        // 50 ms at 5600 rpm: 12% above the 5000 rpm target
    EXPECT_GT(mixerRuntime.dynIdleI, saturated * 0.95f);
}

static void setupSag(uint16_t cellVoltage, bool dynIdle)
{
    setup(dynIdle, dynIdle ? 50 : 0);
    testPidProfile.vbat_sag_compensation = 100;
    testSagCellVoltage = cellVoltage;
    mixerInitProfile();
}

TEST(Mixer3dTest, sagCapsTopOfBothDirectionsWhenFull)
{
    // full battery (4.20 V/cell), vbat_sag_compensation 100, warning 3.50 V: the top 70/420 = 16.7% of each band is held back
    const float f = 70.0f / 420.0f;
    setupSag(420, false);
    setThrottle(2000);
    runLoops(20, 20000);
    EXPECT_NEAR(maxMotor(), DSHOT_MAX_THROTTLE - f * (DSHOT_MAX_THROTTLE - forwardIdle()), 1.0f);
    setThrottle(1000);
    runLoops(4000, 20000);
    EXPECT_EQ(-1, motorDirection());
    EXPECT_NEAR(maxMotor(), (DSHOT_3D_FORWARD_MIN_THROTTLE - 1) - f * ((DSHOT_3D_FORWARD_MIN_THROTTLE - 1) - reverseIdle()), 1.0f);
}

TEST(Mixer3dTest, sagNoCapWhenBatteryLow)
{
    setupSag(350, false);                       // at the warning voltage: nothing held back
    setThrottle(2000);
    runLoops(20, 20000);
    EXPECT_NEAR(maxMotor(), DSHOT_MAX_THROTTLE, 0.6f);
    setThrottle(1000);
    runLoops(4000, 20000);
    EXPECT_NEAR(maxMotor(), DSHOT_3D_FORWARD_MIN_THROTTLE - 1, 0.6f);
}

TEST(Mixer3dTest, sagLeavesIdleAndHalfwayScales)
{
    setupSag(385, false);                       // halfway between 4.20 and 3.50: half the cap
    const float f = 0.5f * 70.0f / 420.0f;
    setThrottle(1700);
    runLoops(20, 20000);
    const float fwd = (motor[0] - forwardIdle()) / (DSHOT_MAX_THROTTLE - forwardIdle());
    EXPECT_NEAR(fwd, (1.0f - f) * 197.0f / 497.0f, 0.01f);
    setThrottle(1501);                          // centre stick: exact idle, sag compensation never touches the bottom
    runLoops(20, 20000);
    EXPECT_NEAR(minMotor(), forwardIdle(), 0.6f);
    EXPECT_NEAR(maxMotor(), forwardIdle(), 0.6f);
}

TEST(Mixer3dTest, sagWithDynIdleStaysInBand)
{
    // both on, motors stopped (dyn idle saturated) and battery full: idle raised, top held back, everything in band
    setupSag(420, true);
    setThrottle(1470);
    runLoops(20, 20000);
    runLoops(200, 0);
    const float f = 70.0f / 420.0f;
    const float top = (DSHOT_3D_FORWARD_MIN_THROTTLE - 1) - f * ((DSHOT_3D_FORWARD_MIN_THROTTLE - 1) - reverseIdle());
    EXPECT_GT(minMotor(), reverseIdle() + 50.0f);
    EXPECT_LE(maxMotor(), top + 0.6f);
    setThrottle(1000);
    runLoops(1, 0);
    EXPECT_NEAR(maxMotor(), top, 1.0f);         // full reverse reaches the held-back top, not past it
}
