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

#pragma once

#define FC_TARGET_MCU     STM32F722

#define BOARD_NAME        JHEF7DUAL
#define MANUFACTURER_ID   JHEF

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_ACCGYRO_LSM6DSK320X
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_FLASH_PY25Q128HA

#define BEEPER_PIN           PC15
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB4
#define MOTOR4_PIN           PB3
#define MOTOR5_PIN           PC9
#define MOTOR6_PIN           PC8
#define RX_PPM_PIN           PA3
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PA15
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define ADC_VBAT_PIN         PC2
#define ADC_RSSI_PIN         PC0
#define ADC_CURR_PIN         PC1
#define PINIO1_PIN           PC14
#define PINIO2_PIN           PB9
#define FLASH_CS_PIN         PC13
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_2_EXTI_PIN      PC3
#define GYRO_1_CS_PIN        PB2
#define GYRO_2_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA3 , 3, -1) \
    TIMER_PIN_MAP( 1, PB0 , 2,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PB4 , 1,  0) \
    TIMER_PIN_MAP( 4, PB3 , 1,  0) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PC8 , 2,  0) \
    TIMER_PIN_MAP( 7, PA8 , 1,  0)

#define ADC3_DMA_OPT        1

#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_2
#define BARO_I2C_INSTANCE I2CDEV_1
#define MAG_I2C_INSTANCE I2CDEV_1
#define ADC_INSTANCE ADC3
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_INA226
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_INA226
#define DEFAULT_CURRENT_METER_SCALE 450
#define BEEPER_INVERTED
#define PINIO1_BOX 40
#define PINIO2_BOX 41
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW90_DEG
#define GYRO_2_SPI_INSTANCE SPI1
#define GYRO_2_ALIGN CW90_DEG

// --- LittleBee 1.0 additions -------------------------------------------------
//
// An INA226 with a 0.001 ohm shunt sits on the pack side of the power wiring
// and supplies both the pack voltage and the pack current. The board's own VBAT
// divider cannot be used here: the flight controller is fed from a 12V boost
// converter (the 2S pack browns the board out below ~6.5V), so ADC_VBAT_PIN
// reads the regulated 12V rail rather than the pack.
//
// Wiring: INA226 SCL -> PB6, SDA -> PB7 (I2C1, shared with the baro), VS -> 5V,
// A0/A1 -> GND for address 0x40, VBUS -> pack positive, IN+/IN- across the shunt.

#define USE_INA226
#define INA226_I2C_INSTANCE            I2CDEV_1
#define INA226_I2C_ADDRESS             0x40
#define INA226_DEFAULT_SHUNT_MICROOHMS 1000   // 0.001 ohm, +/-81.9A range, 2.5mA resolution

// GPS is not auto-enabled on a 512KB target (common_pre.h gates it on
// TARGET_FLASH_SIZE >= 1024), so opt in explicitly. This also derives
// USE_GPS_NMEA, USE_GPS_UBLOX and USE_GPS_RESCUE in common_post.h.
#define USE_GPS

// Altitude and position hold sit in the same TARGET_FLASH_SIZE >= 1024 block
// as GPS, so they also need enabling by hand. USE_POSITION_HOLD requires
// USE_GPS and/or USE_OPTICALFLOW, satisfied by the USE_GPS above.
#define USE_ALTITUDE_HOLD
#define USE_POSITION_HOLD

// BAROLAND: a switch-invoked baro-only controlled descent that disarms on
// touchdown. Flies the same descent as failsafe auto-landing, so it needs
// USE_ALTITUDE_HOLD above; no GPS is required.
#define USE_BARO_LAND

// Free flash by dropping the OSD stack (CMS, MAX7456, MSP displayport), the
// VTX stack (SmartAudio, Tramp, MSP VTX, VTX table, VTX control) and the
// addressable LED strip. Delete a define to get that stack back.
#define EXCLUDE_OSD
#define EXCLUDE_VTX
#define EXCLUDE_LED_STRIP

// MAVLink telemetry is another TARGET_FLASH_SIZE >= 1024 feature, so opt in
// explicitly. Selecting MAVLINK as a telemetry protocol without this compiles
// to a no-op: the provider claims no serial function and the port stays idle.
#define USE_TELEMETRY_MAVLINK
