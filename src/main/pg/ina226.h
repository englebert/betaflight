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

#include "pg/pg.h"

#define INA226_SHUNT_MICROOHMS_MIN 50     // 0.05 mOhm
#define INA226_SHUNT_MICROOHMS_MAX 65535  // 65.5 mOhm

#define INA226_VBAT_SCALE_MIN 500         // 0.500x
#define INA226_VBAT_SCALE_MAX 2000        // 2.000x

typedef struct ina226Config_s {
    uint8_t device;          // I2C bus, in CLI numbering (I2C_DEV_TO_CFG); 0 selects no bus
    uint8_t address;         // 7 bit I2C address, 0x40..0x4F depending on the A0/A1 straps
    uint16_t shuntMicroOhms; // value of the shunt resistor in micro-ohms
    uint16_t vbatScale;      // bus voltage trim in 1/1000ths, 1000 is unity
    int16_t currentOffset;   // zero-current offset in centiamps, added after scaling
    uint8_t averaging;       // index into the device's sample averaging table, 0..7
    uint8_t currentInvert;   // negate the shunt reading for a reversed shunt connection
} ina226Config_t;

PG_DECLARE(ina226Config_t, ina226Config);
