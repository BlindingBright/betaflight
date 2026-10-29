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

#pragma once

#include "platform.h"

#include "flight/mixer.h"

// per-direction 3D I-term memory follows I-term slowly, so it holds the steady trim
// (e.g. a CG offset) of each direction rather than the last maneuver's windup
#define ITERM_3D_MEMORY_CUTOFF_HZ 0.5f

typedef struct mixerRuntime_s {
    uint8_t motorCount;
    motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];
#ifdef USE_LAUNCH_CONTROL
    motorMixer_t launchControlMixer[MAX_SUPPORTED_MOTORS];
#endif
    bool feature3dEnabled;
    float iterm3dMemoryK;               // PT1 gain of the per-direction I-term memory
    float motorOutputLow;
    float motorOutputHigh;
    float disarmMotorOutput;
    float deadbandMotor3dHigh;
    float deadbandMotor3dLow;
#ifdef USE_DYN_IDLE
    float dynIdleMaxIncrease;
    float dynIdleStartIncrease;
    float dynIdleMinRps;
    float dynIdlePGain;
    float prevMinRps;
    float dynIdleIGain;
    float dynIdleDGain;
    float dynIdleI;
    float minRpsDelayK;
#endif
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    float vbatSagCompensationFactor;
    float vbatFull;
    float vbatRangeToCompensate;
#endif
#if defined(USE_RPM_LIMIT)
    float rpmLimiterRpmLimit;
    float rpmLimiterThrottleScale;
    float rpmLimiterInitialThrottleScale;
    float rpmLimiterPGain;
    float rpmLimiterIGain;
    float rpmLimiterDGain;
    float rpmLimiterI;
    pt1Filter_t rpmLimiterAverageRpmFilter;
    pt1Filter_t rpmLimiterThrottleScaleOffsetFilter;
#endif
    float ezLandingThreshold;
    float ezLandingLimit;
    float ezLandingSpeed;
} mixerRuntime_t;

extern mixerRuntime_t mixerRuntime;
