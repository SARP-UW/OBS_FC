/**
 * This file is part of the Titan Flight Computer Project
 * Copyright (c) 2025 UW SARP
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file sensors/include/barometer.c
 * @authors Jude Merritt
 * @brief MS561101BA03 Barometer driver
 */

#include <stdint.h>
#include "include/spi.h"
#include "include/errc.h"
#include "myWork/systick.h"
#include "myWork/barometer.h"

#define D1_BASE_CMD 0x40
#define D2_BASE_CMD 0x50
#define ADC_READ    0x00
#define RESET       0x1E

// Calibration coefficients
typedef enum {
    PROM_ADDR_MANUFACTURER = 0xA0,
    PROM_ADDR_C1           = 0xA2,
    PROM_ADDR_C2           = 0xA4,
    PROM_ADDR_C3           = 0xA6,
    PROM_ADDR_C4           = 0xA8,
    PROM_ADDR_C5           = 0xAA,
    PROM_ADDR_C6           = 0xAC,
    PROM_ADDR_CRC          = 0xAE
}prom_addr_t;

static inline ti_errc_t validate_dev_values(barometer_t *dev) {
    ti_errc_t status = TI_ERRC_NONE;

    //Check that SPI instance is valid
    if ((dev->device.instance < 1) || (dev->device.instance > 6)) status = TI_ERRC_INVALID_ARG;

    //TODO: May also want to check CS pin

    //Check that OSR value is valid
    if ((dev->osr < 0x00) || (dev->osr > 0x08)) status = TI_ERRC_INVALID_ARG;

    return status;
}

// Provides the necessary conversion time delay based on the specified oversampling ratio (OSR) using the SysTick timer.
static void barometer_delay(uint8_t osr) {
    uint8_t conversion_time;

   /*
    * The conversion times are based on the following table:
    * OSR   Min.   Max.
    * 4096  7.40   9.04
    * 2048  3.72   4.54
    * 1024  1.88   2.28
    * 512   0.95   1.17
    * 256   0.48   0.60
    */
    switch (osr) {
        case (OSR_256):  conversion_time = 1;  break;
        case (OSR_512):  conversion_time = 2;  break;
        case (OSR_1024): conversion_time = 3;  break;
        case (OSR_2048): conversion_time = 5;  break;
        case (OSR_4096): conversion_time = 10; break;
        default: return TI_ERRC_INVALID_ARG;
    }

    delay(conversion_time);
}

// Sends a command to the sensor and reads the multi-byte response.
static uint32_t barometer_transfer(barometer_t *dev, uint8_t cmd, uint8_t bytes_to_read) {
    uint8_t tx[4] = {cmd, 0, 0, 0};
    uint8_t rx[4] = {0, 0, 0, 0};
    uint32_t result = 0;

    struct spi_sync_transfer_t transfer = {
        .device = dev->device,
        .source = tx,
        .dest = rx,
        .size = bytes_to_read + 1,
        .timeout = 1,
        .read_inc = true
    };

    spi_block(dev->device);
    spi_sync_transfer_t(&transfer);
    spi_unblock(dev->device);

    if (bytes_to_read == 2) {
        result = (uint32_t)((rx[1] << 8) | rx[2]);
    } else if (bytes_to_read == 3) {
        result = (uint32_t)((rx[1] << 16) | (rx[2] << 8) | rx[3]);
    }

    return result;
}

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/

ti_errc_t barometer_init(barometer_t *dev) {
    // Check OSR and device fields
    ti_errc_t status = validate_dev(dev);
    if (status != TI_ERRC_NONE) return status;

    // Reset the sensor
    barometer_transfer(dev, RESET, 0);

    // Wait for internal reload
    barometer_delay(dev->osr);

    // Read PROM values
    dev->calibration_data.sens     = (uint16_t)barometer_transfer(dev, PROM_ADDR_C1, 2);
    dev->calibration_data.off      = (uint16_t)barometer_transfer(dev, PROM_ADDR_C2, 2);
    dev->calibration_data.tcs      = (uint16_t)barometer_transfer(dev, PROM_ADDR_C3, 2);
    dev->calibration_data.tco      = (uint16_t)barometer_transfer(dev, PROM_ADDR_C4, 2);
    dev->calibration_data.t_ref    = (uint16_t)barometer_transfer(dev, PROM_ADDR_C5, 2);
    dev->calibration_data.tempsens = (uint16_t)barometer_transfer(dev, PROM_ADDR_C6, 2);

    return TI_ERRC_NONE;
}

barometer_result_t *get_barometer_data(barometer_t *dev) {
    barometer_result_t result;
    barometer_result_t *ptr_result = &result;

    result.errc = TI_ERRC_NONE;

    // Get raw D1 pressure data
    barometer_transfer(dev, D1_BASE_CMD + dev->osr, 0);
    barometer_delay(dev->osr);
    uint32_t D1 = barometer_transfer(dev, ADC_READ, 3);

    // Get raw D2 temperature data
    barometer_transfer(dev, D2_BASE_CMD + dev->osr, 0);
    barometer_delay(dev->osr);
    uint32_t D2 = barometer_transfer(dev, ADC_READ, 3);

    if ((D1 || D2) <= 0) result.errc = TI_ERRC_UNKNOWN;

    // Calculate temperature difference
    int32_t dT = D2 - ((int32_t)dev->calibration_data.t_ref << 8);

    // Calculate actual temperature
    int32_t temp = 2000 + (((int64_t)dT * dev->calibration_data.tempsens) >> 23);

    // Calculate initial offset and sensitivity
    int64_t off  = ((int64_t)dev->calibration_data.off << 16) + (((int64_t)dev->calibration_data.tco * dT) >> 7);
    int64_t sens = ((int64_t)dev->calibration_data.sens << 15) + (((int64_t)dev->calibration_data.tcs * dT) >> 8);

    // Second order temperature compensation
    int64_t T2    = 0;
    int64_t OFF2  = 0;
    int64_t SENS2 = 0;

    // If temperature is below 20°C
    if (temp < 2000) {
        T2    = ((int64_t)dT * dT) >> 31;
        OFF2  = 5 * ((int64_t)(temp - 2000) * (temp - 2000)) >> 1;
        SENS2 = 5 * ((int64_t)(temp - 2000) * (temp - 2000)) >> 2;

        // If temperature if below -15°C
        if (temp < -1500) {
            OFF2  = OFF2 + 7 * ((int64_t)(temp + 1500) * (temp + 1500));
            SENS2 = SENS2 + (11 * ((int64_t)(temp + 1500) * (temp + 1500)) >> 1);
        }
    }

    temp -= T2;
    off  -= OFF2;
    sens -= SENS2;

    // Calculate temperature compensated pressure
    int32_t P = (((D1 * sens) >> 21) - off) >> 15;

    // Results
    result.pressure    = (float)P / 100.0f;    // Units of mbar/hPa
    result.temperature = (float)temp / 100.0f; // Units of Celcius

    if ((result.pressure || result.temperature) <= 0) result.errc = TI_ERRC_UNKNOWN;

    return ptr_result;
}

/**
 * TODO:
 * 1. Do I need to initialize spi via spi_init or should the client do this before using the barometer */