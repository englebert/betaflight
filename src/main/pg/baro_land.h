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

#include <stdint.h>

#include "pg/pg.h"

typedef struct baroLandConfig_s {
    uint8_t timeout;            // seconds in the mode before the backstop disarm; 0 disables it
    uint8_t touchdownTime;      // tenths of a second the touchdown conditions must hold before disarming
    uint8_t touchdownVelocity;  // cm/s; vertical speed at or below this counts as "no longer descending"
} baroLandConfig_t;

PG_DECLARE(baroLandConfig_t, baroLandConfig);
