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

#include "platform.h"

#ifdef USE_BARO_LAND

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "baro_land.h"

PG_REGISTER_WITH_RESET_TEMPLATE(baroLandConfig_t, baroLandConfig, PG_BARO_LAND_CONFIG, 1);

PG_RESET_TEMPLATE(baroLandConfig_t, baroLandConfig,
    .flareAltitude = 150,      // 1.5 m
    .maxDescentRate = 250,     // 2.5 m/s when high
    .descentRate = 40,         // 0.4 m/s final approach
    .timeout = 60,             // matches the failsafe_landing_time backstop
    .touchdownTime = 10,       // 1.0s of stalled descent at low throttle
    .touchdownVelocity = 50,   // 0.5 m/s
);

#endif // USE_BARO_LAND
