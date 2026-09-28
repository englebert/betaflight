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

#pragma once

#include <stdbool.h>

#include "common/time.h"
#include "common/utils.h"

#include "pg/baro_land.h"

#ifdef USE_BARO_LAND

void baroLandInit(void);

// Ticked from processRxModes() once the BARO_LAND_MODE flag for this cycle has
// been decided. Uses millis() deadlines rather than accumulated intervals, so
// the RX-rate cadence it runs at does not need to be uniform.
void baroLandUpdate(void);

// Descent velocity command for the current height, in cm/s (positive is the
// rate of descent). Alt hold flies this in place of the failsafe profile.
float baroLandDescentRateCmS(float altitudeCm);

// True while the pilot-invoked baro landing owns the descent. Alt hold reads
// this to drive the descent profile, and core.c to keep the hold modes engaged.
bool baroLandIsActive(void);

#else
static inline void baroLandInit(void) { }
static inline void baroLandUpdate(void) { }
static inline bool baroLandIsActive(void) { return false; }
static inline float baroLandDescentRateCmS(float altitudeCm) { UNUSED(altitudeCm); return 0.0f; }
#endif // USE_BARO_LAND
