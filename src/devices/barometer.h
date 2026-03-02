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
 * @file devices/barometer.h
 * @authors Jude Merritt
 * @brief MS561101BA03 Barometer driver
 */

#pragma once
#include <stdint.h>
#include "peripheral/spi.h"
#include "peripheral/errc.h"

/**************************************************************************************************
 * @section Type definitions
 **************************************************************************************************/

/**
 * @brief Oversampling ratios
 */
typedef enum {
    OSR_256  = 0x00, // 256 samples per measurement
    OSR_512  = 0x02, // 512 samples per measurement
    OSR_1024 = 0x04, // 1024 samples per measurement
    OSR_2048 = 0x06, // 2048 samples per measurement
    OSR_4096 = 0x08  // 4096 samples per measurement
}barometer_osr_t;

/**
 * @brief SPI specifications
 */
typedef struct {
    uint8_t inst;
    uint8_t ss_pin; 
}barometer_spi_dev;

/**
 * @brief Configuration data
 */
typedef struct {
    uint16_t sens;     // C1 Presure sensitivity
    uint16_t off;      // C2 Pressure offset
    uint16_t tcs;      // C3 Temperature coefficient of pressure sensitivity
    uint16_t tco;      // C4 Temperature coefficient of pressure offset
    uint16_t t_ref;    // C5 Reference temperature
    uint16_t tempsens; // C6 Temperature coefficient of the temperature
}barometer_calibration_data_t; 

/**
 * @brief Pressure and temperature results
 */
typedef struct {
    float pressure;    // Temperature compensated pressure from 10 mbar to 1200 mbar with 0.01 mbar resolution
    float temperature; // Temperature from -40 C to 85 C with 0.01 C resulution
}barometer_result_t;

/**
 * @brief Barometer device struct
 */
typedef struct {
    barometer_spi_dev spi_dev;                     // SPI specifications
    barometer_osr_t osr;                           // Oversampling setting
    barometer_calibration_data_t calibration_data; // Device configuration
    barometer_result_t result;                     // Sampled pressure and temperature
} barometer_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initializes the MS561101BA03 barometer and loads calibration data.
 *
 * @param dev pointer to the barometer_t structure
 * @return ti_errc_t TI_ERRC_NONE if no errors occur, and an error code otherwise
 */
enum ti_errc_t barometer_init(barometer_t *dev);

/**
 * @brief Performs a conversion and calculates compensated pressure and temperature.
 *
 * @param dev pointer to the barometer_t structure
 * @return ti_errc_t TI_ERRC_NONE if no errors occur, and an error code otherwise
 */
enum ti_errc_t get_barometer_data(barometer_t *dev);