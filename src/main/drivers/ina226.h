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
#include <stdint.h>

// TI INA226 bidirectional current/power monitor.
//
// The device is read directly from its shunt and bus voltage registers; the
// on-chip current/power registers (and therefore the calibration register) are
// not used, which keeps the scaling entirely in software and avoids the
// quantisation imposed by the calibration register's current LSB.

#define INA226_SHUNT_VOLTAGE_LSB_NV 2500    // 2.5uV per LSB, in nanovolts
#define INA226_BUS_VOLTAGE_LSB_UV   1250    // 1.25mV per LSB, in microvolts

// Number of samples the device averages internally, indexed by the AVG field of
// the configuration register.
#define INA226_AVERAGING_COUNT 8

// The A0/A1 straps select one of sixteen fixed addresses.
#define INA226_ADDRESS_MIN 0x40
#define INA226_ADDRESS_MAX 0x4F

typedef struct ina226Data_s {
    int32_t shuntVoltageMicrovolts; // signed, +/- 81920uV full scale
    uint32_t busVoltageMillivolts;  // 0 to 40960mV full scale
} ina226Data_t;

// Outcome of a bare-bones identity probe at one bus/address.
typedef enum {
    INA226_PROBE_ABSENT = 0, // nothing acknowledged the address
    INA226_PROBE_FOREIGN,    // something answered, but it is not an INA226
    INA226_PROBE_FOUND,
} ina226ProbeResult_e;

// Read the manufacturer ID and configuration registers at one bus/address
// without touching the driver's own device or its cached data, so it is safe to
// call while the sensor is running. `device` is in CLI bus numbering, as stored
// in ina226Config()->device. Both out-params may be NULL.
//
// The die ID register is deliberately not consulted: it sits at 0xFF, which
// this bus layer reserves as its "raw transfer" sentinel, so it cannot be
// addressed. See the note in ina226.c.
ina226ProbeResult_e ina226Probe(uint8_t device, uint8_t address, uint16_t *manufacturerId, uint16_t *configuration);

// Probe for the device on the configured bus and place it in continuous
// conversion mode. Returns true if the device responded with the expected
// manufacturer and die IDs. Repeat calls after a successful detection are
// no-ops, so both the voltage and the current meter may call this.
bool ina226Init(void);

// True once ina226Init() has found a device.
bool ina226IsDetected(void);

// Read shunt and bus voltage into the driver's cache. Reads are rate limited
// internally, so calling this from more than one task is cheap. Returns false
// if the device is absent or the transfer failed, in which case the cached
// data is left untouched.
bool ina226Update(void);

// Most recent successful reading. Zeroed until the first successful update.
const ina226Data_t *ina226GetData(void);
