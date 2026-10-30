/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "io/gps.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

float getMotorMixRange(void)
{
    return motorMixRange;
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;

// per-direction I-term memory: [0] normal, [1] inverted motor direction. Kept across disarms, cleared on reboot.
static FAST_DATA_ZERO_INIT float iterm3dMemory[2][XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float iterm3dThrottleAverage;    // per-direction throttle while flying (0..1)

#ifdef USE_DSHOT_TELEMETRY
// RPM release: per-motor restart detection after a 3D reversal (eRPM in telemetry units of 100)
static FAST_DATA_ZERO_INIT float iterm3dErpm[MAX_SUPPORTED_MOTORS];      // last plausible eRPM reading
static FAST_DATA_ZERO_INIT timeUs_t iterm3dErpmUs[MAX_SUPPORTED_MOTORS]; // when it was accepted
static FAST_DATA_ZERO_INIT float iterm3dErpmAverage;                      // slow average while flying
static FAST_DATA_ZERO_INIT uint8_t iterm3dMotorLow;                       // bit per motor: dropped below the threshold since the reversal
static FAST_DATA_ZERO_INIT uint8_t iterm3dMotorRestarted;                 // bit per motor: back above it in the new direction

// a real spool-up takes tens of ms; an AM32 desync reading jumps straight to a fixed high value
#define ITERM_3D_MAX_ERPM_RISE_PER_S 20.0f      // as multiples of the average eRPM per second
#define ITERM_3D_ERPM_RISE_SLACK     50.0f      // 5000 eRPM of telemetry noise

// eRPM is unsigned, so a motor counts as restarted only after it has dropped below the threshold
// (props crossing zero) and then climbed back above it
static bool iterm3dAllMotorsRestarted(timeUs_t currentTimeUs)
{
    const int motorCount = mixerRuntime.motorCount;
    const float threshold = iterm3dErpmAverage * flight3DConfig()->iterm_rpm_pct3d * 0.01f;
    const float maxRisePerUs = iterm3dErpmAverage * ITERM_3D_MAX_ERPM_RISE_PER_S * 1e-6f;

    for (int i = 0; i < motorCount; i++) {
        const float erpm = getDshotErpm(i);
        const timeDelta_t sinceUs = cmpTimeUs(currentTimeUs, iterm3dErpmUs[i]);
        // falling readings are always accepted, rising ones only as fast as a motor can spin up, and never by
        // more than one average step at once: a desync reading that persists must not become plausible over time
        const float maxRise = MIN(maxRisePerUs * sinceUs, iterm3dErpmAverage) + ITERM_3D_ERPM_RISE_SLACK;
        if (iterm3dErpmAverage <= 0.0f || erpm <= iterm3dErpm[i] + maxRise) {
            iterm3dErpm[i] = erpm;
            iterm3dErpmUs[i] = currentTimeUs;
        }
        if (iterm3dErpm[i] < threshold) {
            iterm3dMotorLow |= 1 << i;
        } else if (iterm3dMotorLow & (1 << i)) {
            iterm3dMotorRestarted |= 1 << i;
        }
    }
    return iterm3dMotorRestarted == (1 << motorCount) - 1;
}
#endif

// true while I-term must stay held after the last reversal
static bool iterm3dHold(bool reversed, timeUs_t currentTimeUs, timeDelta_t sinceReversalUs)
{
    static FAST_DATA_ZERO_INIT bool released;
    const flight3DConfig_t *cfg = flight3DConfig();
    bool restarted = false;

#ifdef USE_DSHOT_TELEMETRY
    if (reversed) {
        // always, so a telemetry dropout at the reversal can't carry over the previous flip's restarts
        iterm3dMotorLow = 0;
        iterm3dMotorRestarted = 0;
    }
    if (isDshotTelemetryActive()) {
        // tracked every loop so the plausibility filter stays current
        restarted = iterm3dAllMotorsRestarted(currentTimeUs) && iterm3dErpmAverage > 0.0f;
    }
#endif

    if (reversed) {
        released = false;
    }
    if (!released) {
        if (cfg->iterm_release3d == ITERM_3D_RELEASE_RPM) {
            // as soon as every motor has restarted, within [min, max]; bigger props restart slower, so the cap is separate
            released = (restarted && sinceReversalUs >= cfg->iterm_min_ms3d * 1000) || sinceReversalUs >= cfg->iterm_max_ms3d * 1000;
        } else {
            released = sinceReversalUs >= cfg->iterm_reset_ms3d * 1000;
        }
    }
    return !released;
}

static bool iterm3dAxisUsesMemory(uint8_t axes, int axis)
{
    switch (axes) {
    case ITERM_3D_AXES_PITCH:
        return axis == FD_PITCH;
    case ITERM_3D_AXES_ROLL_PITCH:
        return axis != FD_YAW;
    default:
        return true;
    }
}

static void update3dIterm(bool reversed, bool hold, timeDelta_t sinceReversalUs, float directionThrottle)
{
    const flight3DConfig_t *cfg = flight3DConfig();
    const uint8_t mode = cfg->iterm_mode3d;
    const int dir = motorOutputMixSign < 0 ? 1 : 0;
    // learn only in steady flight: flying, past the hold, mix not saturated, and not on the approach to center
    // (with little throttle, I-term swings with the deceleration and anti-gravity instead of holding the trim).
    // "Little throttle" is relative to this quad's own average, so it works for any size / power-to-weight.
    const bool flying = ARMING_FLAG(ARMED) && (isAirmodeActivated() || calculateThrottleStatus() != THROTTLE_LOW);
    if (flying && !hold) {
        iterm3dThrottleAverage += mixerRuntime.iterm3dThrottleAverageK * (directionThrottle - iterm3dThrottleAverage);
    }
    const bool steady = flying && !hold && getMotorMixRange() < 1.0f;
    const bool learn = steady && directionThrottle >= iterm3dThrottleAverage * cfg->iterm_learn_thr3d * 0.01f;

    static FAST_DATA_ZERO_INIT bool wasHolding;
    const bool justReleased = wasHolding && !hold;
    wasHolding = hold;

    if (mode == ITERM_3D_PER_DIRECTION && ARMING_FLAG(ARMED)) {
        const bool loadAtRelease = cfg->iterm_load3d == ITERM_3D_LOAD_RELEASE;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            const bool useMemory = iterm3dAxisUsesMemory(cfg->iterm_axes3d, axis);
            if (reversed) {
                // start the new direction from its own remembered trim (other axes start from zero, like stock);
                // with LOAD_RELEASE, stay at zero while the props restart and load once they have grip again
                pidData[axis].I = (useMemory && !loadAtRelease) ? iterm3dMemory[dir][axis] : 0.0f;
            } else if (justReleased && loadAtRelease && useMemory) {
                pidData[axis].I = iterm3dMemory[dir][axis];
            } else if (learn && useMemory) {
                iterm3dMemory[dir][axis] += mixerRuntime.iterm3dMemoryK * (pidData[axis].I - iterm3dMemory[dir][axis]);
            }
        }
    }

#ifdef USE_DSHOT_TELEMETRY
    // the eRPM reference uses all steady flight, not only the throttle range the memory learns from
    if (steady && isDshotTelemetryActive()) {
        float erpmSum = 0.0f;
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            erpmSum += iterm3dErpm[i];
        }
        iterm3dErpmAverage += mixerRuntime.iterm3dRpmAverageK * (erpmSum / mixerRuntime.motorCount - iterm3dErpmAverage);
    }
#endif

    if (mode == ITERM_3D_RESET) {
        pidSetItermFreeze3d(false);
        if (hold) {
            pidResetIterm();
        }
    } else {
        // props have little authority while crossing zero rpm, don't wind up
        pidSetItermFreeze3d(hold);
    }

    DEBUG_SET(DEBUG_3D_ITERM, 0, motorOutputMixSign);
    DEBUG_SET(DEBUG_3D_ITERM, 1, hold ? MAX(1, sinceReversalUs / 1000) : 0); // ms since reversal while holding
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        DEBUG_SET(DEBUG_3D_ITERM, 2 + axis, lrintf(iterm3dMemory[0][axis] * 10.0f));
        DEBUG_SET(DEBUG_3D_ITERM, 5 + axis, lrintf(iterm3dMemory[1][axis] * 10.0f));
    }
}

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;

    float currentThrottleInputRange = 0;
    if (mixerRuntime.feature3dEnabled) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;
        const int8_t previousMixSign = motorOutputMixSign;

        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }

        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }

        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isFlipOverAfterCrashActive()) {
            // INVERTED
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                !flight3DConfigMutable()->switched_mode3d) ||
                isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;

#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        const bool reversed = motorOutputMixSign != previousMixSign;
        const timeDelta_t sinceReversalUs = cmpTimeUs(currentTimeUs, reversalTimeUs);
        update3dIterm(reversed, iterm3dHold(reversed, currentTimeUs, sinceReversalUs), sinceReversalUs,
            currentThrottleInputRange > 0.0f ? throttle / currentThrottleInputRange : 0.0f);
    } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = PWM_RANGE;
#ifdef USE_DYN_IDLE
        if (mixerRuntime.dynIdleMinRps > 0.0f) {
            const float maxIncrease = isAirmodeActivated()
                ? mixerRuntime.dynIdleMaxIncrease : mixerRuntime.dynIdleStartIncrease;
            float minRps = getMinMotorFrequencyHz();
            DEBUG_SET(DEBUG_DYN_IDLE, 3, lrintf(minRps * 10.0f));
            float rpsError = mixerRuntime.dynIdleMinRps - minRps;
            // PT1 type lowpass delay and smoothing for D
            minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);
            float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
            mixerRuntime.prevMinRps = minRps;
            float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
            rpsError = MAX(-0.1f, rpsError); //I rises fast, falls slowly
            mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
            mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0f, maxIncrease);
            motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0f, maxIncrease);
            DEBUG_SET(DEBUG_DYN_IDLE, 0, MAX(-1000, lrintf(dynIdleP * 10000)));
            DEBUG_SET(DEBUG_DYN_IDLE, 1, lrintf(mixerRuntime.dynIdleI * 10000));
            DEBUG_SET(DEBUG_DYN_IDLE, 2, lrintf(dynIdleD * 10000));
        } else {
            motorRangeMinIncrease = 0;
        }
#endif

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        float motorRangeAttenuationFactor = 0;
        // reduce motorRangeMax when battery is full
        if (mixerRuntime.vbatSagCompensationFactor > 0.0f) {
            const uint16_t currentCellVoltage = getBatterySagCellVoltage();
            // batteryGoodness = 1 when voltage is above vbatFull, and 0 when voltage is below vbatLow
            float batteryGoodness = 1.0f - constrainf((mixerRuntime.vbatFull - currentCellVoltage) / mixerRuntime.vbatRangeToCompensate, 0.0f, 1.0f);
            motorRangeAttenuationFactor = (mixerRuntime.vbatRangeToCompensate / mixerRuntime.vbatFull) * batteryGoodness * mixerRuntime.vbatSagCompensationFactor;
            DEBUG_SET(DEBUG_BATTERY, 2, lrintf(batteryGoodness * 100));
            DEBUG_SET(DEBUG_BATTERY, 3, lrintf(motorRangeAttenuationFactor * 1000));
        }
        motorRangeMax = isFlipOverAfterCrashActive() ? mixerRuntime.motorOutputHigh : mixerRuntime.motorOutputHigh - motorRangeAttenuationFactor * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
#else
        motorRangeMax = mixerRuntime.motorOutputHigh;
#endif
        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

#define CRASH_FLIP_DEADBAND 20
#define CRASH_FLIP_STICK_MINF 0.15f

static void applyFlipOverAfterCrashModeToMotors(void)
{
    if (ARMING_FLAG(ARMED)) {
        const float flipPowerFactor = 1.0f - mixerConfig()->crashflip_expo / 100.0f;
        const float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        const float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        const float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);

        const float stickDeflectionPitchExpo = flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
        const float stickDeflectionRollExpo = flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
        const float stickDeflectionYawExpo = flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);

        float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
        float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
        float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);

        float stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));
        float stickDeflectionExpoLength = sqrtf(sq(stickDeflectionPitchExpo) + sq(stickDeflectionRollExpo));

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            stickDeflectionExpoLength = stickDeflectionYawExpo;
            signRoll = 0;
            signPitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0;
        }

        const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
        const float cosThreshold = sqrtf(3.0f)/2.0f; // cos(PI/6.0f)

        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        const float crashFlipStickMinExpo = flipPowerFactor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flipPowerFactor);
        const float flipStickRange = 1.0f - crashFlipStickMinExpo;
        const float flipPower = MAX(0.0f, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

        for (int i = 0; i < mixerRuntime.motorCount; ++i) {
            float motorOutputNormalised =
                signPitch * mixerRuntime.currentMixer[i].pitch +
                signRoll * mixerRuntime.currentMixer[i].roll +
                signYaw * mixerRuntime.currentMixer[i].yaw;

            if (motorOutputNormalised < 0) {
                if (mixerConfig()->crashflip_motor_percent > 0) {
                    motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutputNormalised = 0;
                }
            }
            motorOutputNormalised = MIN(1.0f, flipPower * motorOutputNormalised);
            float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? mixerRuntime.disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);

            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

#ifdef USE_RPM_LIMIT
#define STICK_HIGH_DEADBAND 5    // deadband to make sure throttle cap can raise, even with maxcheck set around 2000
static void applyRpmLimiter(mixerRuntime_t *mixer)
{
    static float prevError = 0.0f;
    const float unsmoothedAverageRpm = getDshotRpmAverage();
    const float averageRpm = pt1FilterApply(&mixer->rpmLimiterAverageRpmFilter, unsmoothedAverageRpm);
    const float error = averageRpm - mixer->rpmLimiterRpmLimit;

    // PID
    const float p = error * mixer->rpmLimiterPGain;
    const float d = (error - prevError) * mixer->rpmLimiterDGain; // rpmLimiterDGain already adjusted for looprate (see mixer_init.c)
    mixer->rpmLimiterI += error * mixer->rpmLimiterIGain;         // rpmLimiterIGain already adjusted for looprate (see mixer_init.c)
    mixer->rpmLimiterI = MAX(0.0f, mixer->rpmLimiterI);
    float pidOutput = p + mixer->rpmLimiterI + d;

    // Throttle limit learning
    if (error > 0.0f && rcCommand[THROTTLE] < rxConfig()->maxcheck) {
        mixer->rpmLimiterThrottleScale *= 1.0f - 4.8f * pidGetDT();
    } else if (pidOutput < -400.0f * pidGetDT() && lrintf(rcCommand[THROTTLE]) >= rxConfig()->maxcheck - STICK_HIGH_DEADBAND && !areMotorsSaturated()) { // Throttle accel corresponds with motor accel
        mixer->rpmLimiterThrottleScale *= 1.0f + 3.2f * pidGetDT();
    }
    mixer->rpmLimiterThrottleScale = constrainf(mixer->rpmLimiterThrottleScale, 0.01f, 1.0f);

    float rpmLimiterThrottleScaleOffset = pt1FilterApply(&mixer->rpmLimiterThrottleScaleOffsetFilter, constrainf(mixer->rpmLimiterRpmLimit / motorEstimateMaxRpm(), 0.0f, 1.0f) - mixer->rpmLimiterInitialThrottleScale);
    throttle *= constrainf(mixer->rpmLimiterThrottleScale + rpmLimiterThrottleScaleOffset, 0.0f, 1.0f);

    // Output
    pidOutput = MAX(0.0f, pidOutput);
    throttle = constrainf(throttle - pidOutput, 0.0f, 1.0f);
    prevError = error;

    DEBUG_SET(DEBUG_RPM_LIMIT, 0, lrintf(averageRpm));
    DEBUG_SET(DEBUG_RPM_LIMIT, 1, lrintf(rpmLimiterThrottleScaleOffset * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 2, lrintf(mixer->rpmLimiterThrottleScale * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 3, lrintf(throttle * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 4, lrintf(error));
    DEBUG_SET(DEBUG_RPM_LIMIT, 5, lrintf(p * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 6, lrintf(mixer->rpmLimiterI * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 7, lrintf(d * 100.0f));
}
#endif // USE_RPM_LIMIT

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
#ifdef USE_THRUST_LINEARIZATION
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
    DEBUG_SET(DEBUG_EZLANDING, 1, throttle * 10000U);
    // DEBUG_EZLANDING 0 is the ezLanding factor 2 is the throttle limit
}

static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100 && !RPM_LIMIT_ACTIVE) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

static void applyMotorStop(void)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

static void applyMixerAdjustmentLinear(float *motorMix, const bool airmodeEnabled)
{
    float airmodeTransitionPercent = 1.0f;
    float motorDeltaScale = 0.5f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
        motorDeltaScale *= airmodeTransitionPercent; // this should be half of the motor authority allowed
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    const float motorMixDelta = motorDeltaScale * motorMixRange;

    float minMotor = FLT_MAX;
    float maxMotor = FLT_MIN;

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        if (mixerConfig()->mixer_type == MIXER_LINEAR) {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
        } else {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + fabsf(motorMix[i]), motorMix[i] - fabsf(motorMix[i]));
        }
        motorMix[i] *= motorMixNormalizationFactor;

        maxMotor = MAX(motorMix[i], maxMotor);
        minMotor = MIN(motorMix[i], minMotor);
    }

    // constrain throttle so it won't clip any outputs
    throttle = constrainf(throttle, -minMotor, 1.0f - maxMotor);
}

static float calcEzLandLimit(float maxDeflection, float speed)
{
    // calculate limit to where the mixer can raise the throttle based on RPY stick deflection
    // 0.0 = no increas allowed, 1.0 = 100% increase allowed
    const float deflectionLimit = mixerRuntime.ezLandingThreshold > 0.0f ? fminf(1.0f, maxDeflection / mixerRuntime.ezLandingThreshold) : 0.0f;
    DEBUG_SET(DEBUG_EZLANDING, 4, lrintf(deflectionLimit * 10000.0f));

    // calculate limit to where the mixer can raise the throttle based on speed
    // TODO sanity checks like number of sats, dop, accuracy?
    const float speedLimit = mixerRuntime.ezLandingSpeed > 0.0f ? fminf(1.0f, speed / mixerRuntime.ezLandingSpeed) : 0.0f;
    DEBUG_SET(DEBUG_EZLANDING, 5, lrintf(speedLimit * 10000.0f));

    // get the highest of the limits from deflection, speed, and the base ez_landing_limit
    const float deflectionAndSpeedLimit = fmaxf(deflectionLimit, speedLimit);
    return fmaxf(mixerRuntime.ezLandingLimit, deflectionAndSpeedLimit);
}

static void applyMixerAdjustmentEzLand(float *motorMix, const float motorMixMin, const float motorMixMax)
{
    // Calculate factor for normalizing motor mix range to <= 1.0
    const float baseNormalizationFactor = motorMixRange > 1.0f ? 1.0f / motorMixRange : 1.0f;
    const float normalizedMotorMixMin = motorMixMin * baseNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * baseNormalizationFactor;

#ifdef USE_GPS
    const float speed = STATE(GPS_FIX) ? gpsSol.speed3d / 100.0f : 0.0f;  // m/s
#else
    const float speed = 0.0f;
#endif

    const float ezLandLimit = calcEzLandLimit(getMaxRcDeflectionAbs(), speed);
    // use the largest of throttle and limit calculated from RPY stick positions
    float upperLimit = fmaxf(ezLandLimit, throttle);
    // limit throttle to avoid clipping the highest motor output
    upperLimit = fminf(upperLimit, 1.0f - normalizedMotorMixMax);

    // Lower throttle Limit
    const float epsilon = 1.0e-6f;  // add small value to avoid divisions by zero
    const float absMotorMixMin = fabsf(normalizedMotorMixMin) + epsilon;
    const float lowerLimit = fminf(upperLimit, absMotorMixMin);

    // represents how much motor values have to be scaled to avoid clipping
    const float ezLandFactor = upperLimit / absMotorMixMin;

    // scale motor values
    const float normalizationFactor = baseNormalizationFactor * fminf(1.0f, ezLandFactor);
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= normalizationFactor;
    }
    motorMixRange *= baseNormalizationFactor;
    // Make anti windup recognize reduced authority range
    motorMixRange = fmaxf(motorMixRange, 1.0f / ezLandFactor);

    // Constrain throttle
    throttle = constrainf(throttle, lowerLimit, upperLimit);

    // Log ezLandFactor, upper throttle limit, and ezLandFactor if throttle was zero
    DEBUG_SET(DEBUG_EZLANDING, 0, fminf(1.0f, ezLandFactor) * 10000U);
    // DEBUG_EZLANDING 1 is the adjusted throttle
    DEBUG_SET(DEBUG_EZLANDING, 2, upperLimit * 10000U);
    DEBUG_SET(DEBUG_EZLANDING, 3, fminf(1.0f, ezLandLimit / absMotorMixMin) * 10000U);
    // DEBUG_EZLANDING 4 and 5 is the upper limits based on stick input and speed respectively
}

static void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled)
{
#ifdef USE_AIRMODE_LPF
    const float unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    float airmodeThrottleChange = 0.0f;
#endif
    float airmodeTransitionPercent = 1.0f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= motorMixNormalizationFactor;
    }

    const float normalizedMotorMixMin = motorMixMin * motorMixNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * motorMixNormalizationFactor;
    throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);

#ifdef USE_AIRMODE_LPF
    airmodeThrottleChange = constrainf(unadjustedThrottle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax) - unadjustedThrottle;
    pidUpdateAirmodeLpf(airmodeThrottleChange);
#endif
}

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    if (isFlipOverAfterCrashActive()) {
        applyFlipOverAfterCrashModeToMotors();
        return;
    }

    const bool launchControlActive = isLaunchControlActive();

    motorMixer_t * activeMixer = &mixerRuntime.currentMixer[0];
#ifdef USE_LAUNCH_CONTROL
    if (launchControlActive && (currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY)) {
        activeMixer = &mixerRuntime.launchControlMixer[0];
    }
#endif

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

    // and for TPA
    pidUpdateTpaFactor(throttle);

#ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

    // apply throttle boost when throttle moves quickly
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode
    mixerThrottle = throttle;

#ifdef USE_DYN_IDLE
    // Set min throttle offset of 1% when stick is at zero and dynamic idle is active
    if (mixerRuntime.dynIdleMinRps > 0.0f) {
        throttle = MAX(throttle, 0.01f);
    }
#endif

#ifdef USE_THRUST_LINEARIZATION
    // reduce throttle to offset additional motor output
    throttle = pidCompensateThrustLinearization(throttle);
#endif

#ifdef USE_RPM_LIMIT
    if (RPM_LIMIT_ACTIVE && useDshotTelemetry && ARMING_FLAG(ARMED)) {
        applyRpmLimiter(&mixerRuntime);
    }
#endif

    // Find roll/pitch/yaw desired output
    // ??? Where is the optimal location for this code?
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
    const bool airmodeEnabled = airmodeIsEnabled() || launchControlActive;
#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;
    }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if (launchControlActive) {
        throttle = 0.0f;
    }
#endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    motorMixRange = motorMixMax - motorMixMin;

    switch (mixerConfig()->mixer_type) {
    case MIXER_LEGACY:
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
        break;
    case MIXER_LINEAR:
    case MIXER_DYNAMIC:
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
        break;
    case MIXER_EZLANDING:
        applyMixerAdjustmentEzLand(motorMix, motorMixMin, motorMixMax);
        break;
    default:
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
        break;
    }

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}
