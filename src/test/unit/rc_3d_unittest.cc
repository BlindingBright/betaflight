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

// 3D throttle safety: low-voltage cutoff, failsafe stage 2 throttle, throttle curve midpoint

#include <stdint.h>

extern "C" {
    #include "fc/rc_3d.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(Rc3dTest, lowVoltageCutoffShrinksTowardCenter)
{
    // stock 2D math would turn a centered stick (500) at 50% cutoff into 250 = half reverse in 3D
    EXPECT_EQ(500, lowVoltageCutoffThrottle3d(500, 50));
    EXPECT_EQ(750, lowVoltageCutoffThrottle3d(1000, 50));   // full forward -> half forward
    EXPECT_EQ(250, lowVoltageCutoffThrottle3d(0, 50));      // full reverse -> half reverse
    EXPECT_EQ(600, lowVoltageCutoffThrottle3d(700, 50));
    EXPECT_EQ(400, lowVoltageCutoffThrottle3d(300, 50));
    for (int stick = 0; stick <= 1000; stick += 50) {
        EXPECT_EQ(stick, lowVoltageCutoffThrottle3d(stick, 100));   // 100% = no cutoff
        // never crosses center: forward stays forward, reverse stays reverse
        const int32_t out = lowVoltageCutoffThrottle3d(stick, 30);
        EXPECT_TRUE((stick >= 500 && out >= 500) || (stick < 500 && out <= 500)) << stick;
    }
}

TEST(Rc3dTest, failsafeThrottleNeverReverses)
{
    const float midrc = 1500.0f;
    // default failsafe_throttle 1000 (= full reverse if used raw in 3D) -> zero thrust
    EXPECT_FLOAT_EQ(1500.0f, failsafeThrottle3d(1000.0f, midrc));
    // 2D-style "gentle landing" value: zero thrust, not 40% reverse
    EXPECT_FLOAT_EQ(1500.0f, failsafeThrottle3d(1300.0f, midrc));
    EXPECT_FLOAT_EQ(1500.0f, failsafeThrottle3d(1500.0f, midrc));
    // a 3D-aware value above center keeps working as upright thrust (failsafe levels the quad upright)
    EXPECT_FLOAT_EQ(1650.0f, failsafeThrottle3d(1650.0f, midrc));
    EXPECT_FLOAT_EQ(2000.0f, failsafeThrottle3d(2000.0f, midrc));
    // never below center, whatever the setting
    for (float v = 885.0f; v <= 2115.0f; v += 5.0f) {
        EXPECT_GE(failsafeThrottle3d(v, midrc), midrc);
    }
}

TEST(Rc3dTest, throttleCurveMidIsCenter)
{
    EXPECT_EQ(50, throttleCurveMid3d(50));
    EXPECT_EQ(50, throttleCurveMid3d(45));
    EXPECT_EQ(50, throttleCurveMid3d(70));
}
