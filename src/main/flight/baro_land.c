/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * BAROLAND: a pilot-invoked controlled descent on the barometer alone.
 *
 * The descent itself is not new code. Failsafe auto-landing already flies a
 * baro-only descent through alt hold, so this mode reuses that path verbatim
 * (see alt_hold_multirotor.c) and only adds what a switch-invoked landing
 * needs: a touchdown detector and a backstop deadline, both ending in a
 * disarm. Position hold rides along when the estimator has valid XY, exactly
 * as it does under failsafe; with no GPS fix it zeroes its output and the
 * craft descends level, drifting with the wind.
 *
 * Unlike a failsafe this is an ordinary flight mode: the pilot exits it at any
 * point by releasing the switch, which is why the detector has to be strict.
 * A false positive here disarms a flying aircraft.
 */

#include "platform.h"

#ifdef USE_BARO_LAND

#include <math.h>

#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "drivers/time.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "flight/alt_hold.h"
#include "flight/autopilot.h"
#include "flight/position.h"

#include "pg/autopilot.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "baro_land.h"

// The descent needs a moment to actually get moving before a stalled-descent
// test means anything: on entry the craft may still be climbing from the
// pilot's last throttle input, and the altitude controller has to wind its
// integrator down before throttle drops. Until this has elapsed no touchdown
// can be declared.
#define BARO_LAND_SETTLE_MS 2000

typedef struct {
    bool isActive;
    timeMs_t timeoutAtMs;       // 0 when baroland_timeout is disabled
    timeMs_t settledAtMs;
    timeMs_t touchdownSinceMs;  // 0 while the touchdown conditions are not met
} baroLandState_t;

static baroLandState_t baroLand;

void baroLandInit(void)
{
    baroLand.isActive = false;
    baroLand.timeoutAtMs = 0;
    baroLand.settledAtMs = 0;
    baroLand.touchdownSinceMs = 0;
}

bool baroLandIsActive(void)
{
    return baroLand.isActive;
}

// Normalise a throttle in PWM units the way altitudeControl() normalises its
// own output, so the two can be compared directly.
static float baroLandNormaliseThrottle(float throttlePwm)
{
    const float lowEnd = MAX(rxConfig()->mincheck, PWM_RANGE_MIN);
    return constrainf(scaleRangef(throttlePwm, lowEnd, PWM_RANGE_MAX, 0.0f, 1.0f), 0.0f, 1.0f);
}

// Touchdown is inferred, not measured: the ground is what stops a commanded
// descent. All three touchdown conditions below must hold together, because each
// one alone has a benign in-flight explanation — low altitude on a low pass, a
// momentarily zero vertical speed at the top of a baro wobble, low throttle in a
// dive.
static bool baroLandTouchdownDetected(void)
{
    // the throttle test below reads the altitude controller's output, which only
    // means anything while that controller owns the throttle
    if (!isAltHoldActive()) {
        return false;
    }

    if (!isBelowLandingAltitude()) {
        return false;
    }

    // commanded to descend, but no longer descending
    if (fabsf(getAltitudeDerivativeControl()) > (float)baroLandConfig()->touchdownVelocity) {
        return false;
    }

    // and the altitude controller has given up on throttle trying to get down.
    // Threshold sits midway between the controller's floor and the measured
    // hover, so it tracks the craft's own ap_throttle_min / hover throttle.
    const float floorThrottle = baroLandNormaliseThrottle(autopilotConfig()->throttleMin);
    const float hoverThrottle = baroLandNormaliseThrottle(autopilotGetEffectiveHoverThrottlePwm());
    return getAutopilotThrottle() < (floorThrottle + hoverThrottle) * 0.5f;
}

// The descent profile, in absolute cm/s, replacing the failsafe curve alt hold
// would otherwise fly. Two things make a pilot-invoked landing different from a
// failsafe one. It has to arrive gently, so the last stretch is flown at a flat,
// slow descentRate rather than the failsafe profile's untapered 1x climbRate --
// there is no flare anywhere in the altitude controller, and the altitude target
// marches below zero regardless, so whatever rate is commanded at the flare
// height is the rate it touches down at. And the rate must not be tied to
// alt_hold_climb_rate, which also sets the pilot's throttle-stick authority in
// ordinary alt hold; one should not have to be crippled to soften the other.
//
// Above the flare height the rate ramps linearly with altitude, reaching
// maxDescentRate ten flare-heights up, so a descent from a real height does not
// crawl. Altitude here is relative to where the craft armed, not to the ground.
float baroLandDescentRateCmS(float altitudeCm)
{
    const float flareAltCm = (float)baroLandConfig()->flareAltitude;
    const float descentRate = (float)baroLandConfig()->descentRate;
    const float maxDescentRate = MAX((float)baroLandConfig()->maxDescentRate, descentRate);

    if (altitudeCm <= flareAltCm) {
        return descentRate;
    }

    const float ramp = constrainf((altitudeCm - flareAltCm) / (flareAltCm * 9.0f), 0.0f, 1.0f);
    return descentRate + ramp * (maxDescentRate - descentRate);
}

void baroLandUpdate(void)
{
    const timeMs_t nowMs = millis();

    if (!FLIGHT_MODE(BARO_LAND_MODE)) {
        if (baroLand.isActive) {
            baroLandInit();
        }
        return;
    }

    if (!baroLand.isActive) {
        baroLand.isActive = true;
        baroLand.settledAtMs = nowMs + BARO_LAND_SETTLE_MS;
        baroLand.touchdownSinceMs = 0;
        baroLand.timeoutAtMs = baroLandConfig()->timeout
            ? nowMs + baroLandConfig()->timeout * 1000
            : 0;
    }

    if (cmp32(nowMs, baroLand.settledAtMs) < 0) {
        return;
    }

    if (baroLandTouchdownDetected()) {
        if (!baroLand.touchdownSinceMs) {
            baroLand.touchdownSinceMs = nowMs;
        }
        // touchdownTime is in tenths of a second
        if (cmp32(nowMs, baroLand.touchdownSinceMs + baroLandConfig()->touchdownTime * 100) >= 0) {
            disarm(DISARM_REASON_LANDING);
            return;
        }
    } else {
        baroLand.touchdownSinceMs = 0;
    }

    // Backstop: the detector needs a working baro to fire at all, so bound how
    // long the mode can hold the craft in a descent it cannot finish.
    if (baroLand.timeoutAtMs && cmp32(nowMs, baroLand.timeoutAtMs) >= 0) {
        disarm(DISARM_REASON_LANDING);
    }
}

#endif // USE_BARO_LAND
