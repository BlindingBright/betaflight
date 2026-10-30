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

#include <stdbool.h>

#include "common/filter.h"
#include "pg/pg.h"

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

#define PRIMARY_CHANNEL_COUNT (THROTTLE + 1)

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

#define AIRMODEDEADBAND 12

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))
#define THR_MASK (3 << (2 * THROTTLE))

#define CONTROL_RATE_CONFIG_RC_EXPO_MAX  100

#define CONTROL_RATE_CONFIG_RC_RATES_MAX  255

#define CONTROL_RATE_CONFIG_RATE_LIMIT_MIN  200
#define CONTROL_RATE_CONFIG_RATE_LIMIT_MAX  1998

// (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
#define CONTROL_RATE_CONFIG_RATE_MAX  255

extern float rcCommand[4];

typedef struct rcSmoothingFilter_s {
    bool filterInitialized;
    pt3Filter_t filterSetpoint[4];
    pt3Filter_t filterRcDeflection[2];
    pt3Filter_t filterFeedforward[3];

    uint8_t setpointCutoffSetting;
    uint8_t throttleCutoffSetting;
    uint8_t feedforwardCutoffSetting;

    uint16_t setpointCutoffFrequency;
    uint16_t throttleCutoffFrequency;
    uint16_t feedforwardCutoffFrequency;

    float smoothedRxRateHz;
    uint8_t sampleCount;
    uint8_t debugAxis;

    float autoSmoothnessFactorSetpoint;
    float autoSmoothnessFactorFeedforward;
    float autoSmoothnessFactorThrottle;
} rcSmoothingFilter_t;

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_deadband;              // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
    uint8_t alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
    bool yaw_control_reversed;            // invert control direction of yaw
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef enum {
    ITERM_3D_RESET = 0,         // zero I-term for the window (stock behaviour)
    ITERM_3D_HOLD,              // keep I-term, freeze integration for the window
    ITERM_3D_PER_DIRECTION,     // load the remembered I-term of the new direction, freeze for the window
} iterm3dMode_e;

typedef enum {
    ITERM_3D_RELEASE_TIME = 0,  // hold I-term for 3d_iterm_reset_ms after a reversal
    ITERM_3D_RELEASE_RPM,       // hold until every motor has restarted (eRPM), between 3d_iterm_min_ms and 3d_iterm_reset_ms
} iterm3dRelease_e;

typedef enum {
    ITERM_3D_AXES_ALL = 0,
    ITERM_3D_AXES_PITCH,
    ITERM_3D_AXES_ROLL_PITCH,
} iterm3dAxes_e;

typedef enum {
    ITERM_3D_LOAD_FLIP = 0,     // load the new direction's memory at the reversal, hold it frozen while the props restart
    ITERM_3D_LOAD_RELEASE,      // hold I-term at zero while the props restart (like stock), load the memory at release
} iterm3dLoad_e;

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
    uint16_t limit3d_low;                   // pwm output value for max negative thrust
    uint16_t limit3d_high;                  // pwm output value for max positive thrust
    uint8_t switched_mode3d;                // enable '3D Switched Mode'
    uint16_t iterm_reset_ms3d;              // I-term reset/freeze window after a motor reversal
    uint8_t iterm_mode3d;                   // iterm3dMode_e: what happens to I-term on a motor reversal
    // v0.2: appended after the v0.1 fields (and their padding byte) so older saved settings load unchanged
    uint16_t iterm_min_ms3d;                // shortest I-term hold after a reversal (RPM release)
    uint8_t iterm_release3d;                // iterm3dRelease_e
    uint8_t iterm_rpm_pct3d;                // RPM release: restart threshold, % of the average in-flight eRPM
    uint8_t iterm_learn_hz_x10;             // per-direction memory learning rate, 0.1 Hz units
    uint8_t iterm_axes3d;                   // iterm3dAxes_e: axes that use the per-direction memory
    uint8_t iterm_learn_thr3d;              // memory learns only above this % of the average in-flight throttle (not on the approach to center)
    uint16_t iterm_max_ms3d;                // RPM release: longest I-term hold (fallback if a restart isn't seen)
    uint8_t iterm_load3d;                   // iterm3dLoad_e: when the per-direction memory is applied
} flight3DConfig_t;

PG_DECLARE(flight3DConfig_t, flight3DConfig);

typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          // calibrate the gyro right before the first arm
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

bool areUsingSticksToArm(void);

throttleStatus_e calculateThrottleStatus(void);
void processRcStickPositions();

bool isUsingSticksForArming(void);

void rcControlsInit(void);
