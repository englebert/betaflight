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

#include <stdint.h>

#include "platform.h"

#ifdef USE_INA226

#include "drivers/bus_i2c.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/ina226.h"

#ifndef INA226_I2C_INSTANCE
#define INA226_I2C_INSTANCE I2CINVALID
#endif

#ifndef INA226_I2C_ADDRESS
#define INA226_I2C_ADDRESS 0x40
#endif

// A 1 mOhm shunt gives 2.5mA of resolution and 81.9A of range, which suits the
// shunts commonly fitted to INA226 breakout boards used on small multirotors.
#ifndef INA226_DEFAULT_SHUNT_MICROOHMS
#define INA226_DEFAULT_SHUNT_MICROOHMS 1000
#endif

// Averaging 4 samples over the default 1.1ms conversion times gives a complete
// shunt plus bus conversion every 8.8ms, comfortably faster than the 50Hz
// battery tasks that consume it.
#ifndef INA226_DEFAULT_AVERAGING
#define INA226_DEFAULT_AVERAGING 1
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(ina226Config_t, ina226Config, PG_INA226_CONFIG, 0);

PG_RESET_TEMPLATE(ina226Config_t, ina226Config,
    .device = I2C_DEV_TO_CFG(INA226_I2C_INSTANCE),
    .address = INA226_I2C_ADDRESS,
    .shuntMicroOhms = INA226_DEFAULT_SHUNT_MICROOHMS,
    .vbatScale = 1000,
    .currentOffset = 0,
    .averaging = INA226_DEFAULT_AVERAGING,
    .currentInvert = 0,
);

#endif // USE_INA226
