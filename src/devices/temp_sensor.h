/**
 * This file is part of the Titan Flight Computer Project
 * Copyright (c) 2026 UW SARP
 *
 * @file devices/temp_sensor.h
 * @authors Mahir Emran
 * @brief TMP125 2°C accurate digital temperature sensor driver
 */

#pragma once
#include <stdint.h>
#include "peripheral/spi.h"
#include "peripheral/errc.h"

/**************************************************************************************************
 * @section Type definitions
 **************************************************************************************************/

/**
 * @brief TMP125 operating mode
 * Abstracted power states. The driver implementation will handle
 * the necessary SPI bit manipulations to wake/sleep the device.
 */
typedef enum {
    TMP125_MODE_CONTINUOUS = 0x00,  // Stays awake, 120ms conversion loop
    TMP125_MODE_POLLING    = 0x01,  // Auto-sleeps between reads (lowest power)
} tmp125_mode_t;

/**
 * @brief Temperature reading result
 */
typedef struct {
    float temperature;   // Temperature in °C (0.25 °C resolution)
    ti_errc_t errc;      // Error code from the most recent read
} tmp125_result_t;

/**
 * @brief TMP125 configuration
 */
typedef struct {
    tmp125_mode_t mode;  // Desired power mode
} tmp125_config_t;

/** @brief SPI config for TMP125 */
typedef struct {
    uint8_t  spi_inst;      // SPI instance (e.g., 1 for SPI1)
    uint8_t  ss_pin;        // SPI Slave Select pin number
} tmp125_spi_t;

/** @brief TMP125 device handle */
typedef struct {
    tmp125_spi_t  spi_config; // SPI config for TMP125
    tmp125_config_t config;  // Sensor configuration
} tmp125_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initializes the TMP125 temperature sensor.
 *
 * Configures the SPI interface. If configured for POLLING mode, 
 * immediately puts the device into hardware shutdown to save power.
 *
 * @param dev Pointer to the tmp125_t device structure.
 * @return ti_errc_t TI_ERRC_NONE on success.
 */
ti_errc_t tmp125_init(tmp125_t *dev);

/**
 * @brief Reads the current temperature from the TMP125.
 *
 * If the device is in POLLING mode, this function will automatically 
 * wake the device, trigger a conversion, read the result, and put it 
 * back to sleep. If CONTINUOUS, it simply fetches the latest register value.
 *
 * @param dev Pointer to the tmp125_t device structure.
 * @return tmp125_result_t Struct containing the temperature and error code.
 */
tmp125_result_t tmp125_read_temp(tmp125_t *dev);
