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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_INA226

#include "build/debug.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"

#include "pg/ina226.h"

#include "drivers/ina226.h"

#define INA226_REG_CONFIGURATION  0x00
#define INA226_REG_SHUNT_VOLTAGE  0x01
#define INA226_REG_BUS_VOLTAGE    0x02
#define INA226_REG_MANUFACTURER_ID 0xFE

#define INA226_MANUFACTURER_ID    0x5449  // 'TI'

// The die ID lives at register 0xFF, which cannot be reached through this bus
// layer: i2cRead()/i2cWriteBuffer() both treat a register of 0xFF as a sentinel
// meaning "no register address, transfer raw", so asking for it silently reads
// from whatever the device's pointer happens to hold. Identity is therefore
// established from the manufacturer ID plus the configuration register, whose
// bits 14:12 always read back as 0b100 on an INA226.
#define INA226_CONFIG_RESERVED_MASK 0x7000

// Configuration register fields
#define INA226_CONFIG_RESERVED    0x4000  // bits 14:12 read back as 0b100
#define INA226_CONFIG_AVG_SHIFT   9
#define INA226_CONFIG_VBUSCT_SHIFT 6
#define INA226_CONFIG_VSHCT_SHIFT 3

#define INA226_CONVERSION_TIME_1100US 4     // conversion time field value for 1.1ms
#define INA226_MODE_SHUNT_BUS_CONTINUOUS 7

// The registers are read one at a time with non-blocking transfers, driven by
// whichever of the two battery tasks calls in. Both tasks run at 50Hz, so a
// full shunt plus bus cycle completes around 30 times a second, which is well
// clear of the 8.8ms the device needs to convert both at the default averaging.
typedef enum {
    INA226_STATE_START_SHUNT = 0,
    INA226_STATE_READ_SHUNT,
    INA226_STATE_START_BUS,
    INA226_STATE_READ_BUS,
} ina226State_e;

static extDevice_t ina226Dev;
static bool ina226Detected = false;
static ina226Data_t ina226Data;
static ina226State_e ina226State;
// Written by the I2C interrupt handler, so it cannot live on the stack.
static uint8_t ina226Buffer[2];

// A blocking read fails outright when another device on the bus holds it (the
// barometer shares I2C1 here), so a probe retries before calling an address
// empty. A device that is simply absent NACKs immediately, so a wasted attempt
// is cheap.
#define INA226_PROBE_ATTEMPTS 3

static bool ina226ReadWordDev(const extDevice_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];

    if (!i2cBusReadRegisterBuffer(dev, reg, buf, sizeof(buf))) {
        return false;
    }

    *value = (buf[0] << 8) | buf[1];

    return true;
}

ina226ProbeResult_e ina226Probe(uint8_t device, uint8_t address, uint16_t *manufacturerId, uint16_t *configuration)
{
    extDevice_t dev;

    memset(&dev, 0, sizeof(dev));

    if (!i2cBusSetInstance(&dev, device)) {
        return INA226_PROBE_ABSENT;
    }
    dev.busType_u.i2c.address = address;

    uint16_t manufacturer = 0;
    uint16_t config = 0;
    bool answered = false;

    for (int attempt = 0; attempt < INA226_PROBE_ATTEMPTS && !answered; attempt++) {
        answered = ina226ReadWordDev(&dev, INA226_REG_MANUFACTURER_ID, &manufacturer)
            && ina226ReadWordDev(&dev, INA226_REG_CONFIGURATION, &config);
    }

    if (manufacturerId) {
        *manufacturerId = manufacturer;
    }
    if (configuration) {
        *configuration = config;
    }

    if (!answered) {
        return INA226_PROBE_ABSENT;
    }

    if (manufacturer != INA226_MANUFACTURER_ID
        || (config & INA226_CONFIG_RESERVED_MASK) != INA226_CONFIG_RESERVED) {
        return INA226_PROBE_FOREIGN;
    }

    return INA226_PROBE_FOUND;
}

static bool ina226WriteWord(uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { value >> 8, value & 0xFF };

    return i2cWriteBuffer(ina226Dev.bus->busType_u.i2c.device, ina226Dev.busType_u.i2c.address, reg, sizeof(buf), buf);
}

bool ina226IsDetected(void)
{
    return ina226Detected;
}

bool ina226Init(void)
{
    if (ina226Detected) {
        return true;
    }

    memset(&ina226Dev, 0, sizeof(ina226Dev));
    memset(&ina226Data, 0, sizeof(ina226Data));

    if (!i2cBusSetInstance(&ina226Dev, ina226Config()->device)) {
        return false;
    }
    ina226Dev.busType_u.i2c.address = ina226Config()->address;

    if (ina226Probe(ina226Config()->device, ina226Config()->address, NULL, NULL) != INA226_PROBE_FOUND) {
        return false;
    }

    const uint16_t configuration = INA226_CONFIG_RESERVED
        | ((ina226Config()->averaging & 0x07) << INA226_CONFIG_AVG_SHIFT)
        | (INA226_CONVERSION_TIME_1100US << INA226_CONFIG_VBUSCT_SHIFT)
        | (INA226_CONVERSION_TIME_1100US << INA226_CONFIG_VSHCT_SHIFT)
        | INA226_MODE_SHUNT_BUS_CONTINUOUS;

    if (!ina226WriteWord(INA226_REG_CONFIGURATION, configuration)) {
        return false;
    }

    i2cBusDeviceRegister(&ina226Dev);

    ina226State = INA226_STATE_START_SHUNT;
    ina226Detected = true;

    return true;
}

static uint16_t ina226BufferToWord(void)
{
    return (ina226Buffer[0] << 8) | ina226Buffer[1];
}

bool ina226Update(void)
{
    if (!ina226Detected) {
        return false;
    }

    bool error = false;

    switch (ina226State) {
    case INA226_STATE_START_SHUNT:
        // A busy bus (the barometer shares it) just defers us by one call.
        if (i2cBusReadRegisterBufferStart(&ina226Dev, INA226_REG_SHUNT_VOLTAGE, ina226Buffer, sizeof(ina226Buffer))) {
            ina226State = INA226_STATE_READ_SHUNT;
        }
        break;

    case INA226_STATE_READ_SHUNT:
        if (i2cBusBusy(&ina226Dev, &error)) {
            break;
        }
        if (!error) {
            // The shunt register is two's complement, +/-81920uV full scale.
            const int16_t raw = (int16_t)ina226BufferToWord();
            ina226Data.shuntVoltageMicrovolts = ((int32_t)raw * INA226_SHUNT_VOLTAGE_LSB_NV) / 1000;
            DEBUG_SET(DEBUG_CURRENT_SENSOR, 4, raw);
        }
        ina226State = INA226_STATE_START_BUS;
        break;

    case INA226_STATE_START_BUS:
        if (i2cBusReadRegisterBufferStart(&ina226Dev, INA226_REG_BUS_VOLTAGE, ina226Buffer, sizeof(ina226Buffer))) {
            ina226State = INA226_STATE_READ_BUS;
        }
        break;

    case INA226_STATE_READ_BUS:
        if (i2cBusBusy(&ina226Dev, &error)) {
            break;
        }
        if (!error) {
            // The bus register is unsigned, 1.25mV per LSB.
            ina226Data.busVoltageMillivolts = ((uint32_t)ina226BufferToWord() * INA226_BUS_VOLTAGE_LSB_UV) / 1000;
            DEBUG_SET(DEBUG_CURRENT_SENSOR, 5, ina226Data.busVoltageMillivolts / 10);
        }
        ina226State = INA226_STATE_START_SHUNT;
        break;
    }

    // A failed transfer leaves the previous reading in place and the machine
    // simply carries on with the next register.
    return !error;
}

const ina226Data_t *ina226GetData(void)
{
    return &ina226Data;
}

#endif // USE_INA226
