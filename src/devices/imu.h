/**
 * This file is part of the Titan Flight Computer Project
 * Copyright (c) 2026 UW SARP
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
 * @file devices/imu.c
 * @authors Jude Merritt
 * @brief ICM-42688-P IMU driver implementation
 */

#include "internal/mmio.h"
#include "peripheral/spi.h"
#include <stdint.h>

#pragma once

// IMU device struct
struct imu_spi_dev {
    uint8_t inst;   // SPI Instance
    uint8_t ss_pin; // Slave Select Pin
};

struct result { 
    int16_t accel_x; 
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp; // Internal temperature
};

/**
 * @brief Initializes the ICM-42688-P IMU. 
 * 
 * @param dev  pointer to the imu_spi_dev structure. 
 */
void imu_init(struct imu_spi_dev* dev);

/**
 * @brief Performs a burst read of the sensor registers and returns them 
 *        by updating the result structure. Ensure that the desired SPI
 *        instance has been initialized before calling this function. 
 * 
 * @param dev  pointer to the imu_spi_dev structure. 
 * @param result  pointer to the result structure. 
 * @return 1 if the parameters are valid and -1 if not. 
 */
int imu_transfer(struct imu_spi_dev* dev, struct result* result);