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

#include <stdint.h>

// Throttle safety in normal 3D mode (stick center = motors stopped, below center = reverse).
// The 2D code paths scale the throttle toward the bottom of the stick, which in 3D is full reverse.

#define THROTTLE_3D_STICK_CENTER 500    // processRcCommand's 0..1000 throttle stick, center

// low-voltage cutoff: shrink the throttle toward stick center (zero thrust), not toward the bottom (full reverse)
static inline int32_t lowVoltageCutoffThrottle3d(int32_t stick, int percentage)
{
    return THROTTLE_3D_STICK_CENTER + (stick - THROTTLE_3D_STICK_CENTER) * percentage / 100;
}

// failsafe stage 2 throttle: failsafe_throttle counts as thrust above stick center in the motors' current direction.
// Values at or below center (the default 1000, or 2D-style "gentle" values like 1300) give zero thrust instead of reverse.
static inline float failsafeThrottle3d(float failsafeThrottle, float midrc, int direction)
{
    const float thrust = failsafeThrottle > midrc ? failsafeThrottle - midrc : 0.0f;
    return direction < 0 ? midrc - thrust : midrc + thrust;
}

// throttle curve midpoint: in 3D the curve's midpoint is stick center, so thr_mid other than 50 would move zero thrust off
// center (into one direction's deadband edge or past it)
static inline uint8_t throttleCurveMid3d(uint8_t thrMid8)
{
    (void)thrMid8;
    return 50;
}
