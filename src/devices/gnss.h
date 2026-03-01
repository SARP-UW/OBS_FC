/**
 * This file is part of the Titan Flight Computer Project
 * Copyright (c) 2026 UW SARP
 *
 * @file devices/gnss.h
 * @authors Mahir Emran
 * @brief NEO-M8Q-01A u-blox M8 concurrent GNSS module driver
 */

#pragma once
#include <stdint.h>
#include "peripheral/spi.h"
#include "peripheral/errc.h"

/**************************************************************************************************
 * @section Type definitions
 **************************************************************************************************/

/** @brief GNSS constellation selection */
typedef enum {
    GNSS_CONSTELLATION_GPS       = 0x01,
    GNSS_CONSTELLATION_GLONASS   = 0x02,
    GNSS_CONSTELLATION_BEIDOU    = 0x04,
    GNSS_CONSTELLATION_GALILEO   = 0x08,
    GNSS_CONSTELLATION_SBAS      = 0x10,
    GNSS_CONSTELLATION_QZSS      = 0x20,
} gnss_constellation_t;

/** @brief GNSS fix type */
typedef enum {
    GNSS_FIX_NONE    = 0x00,
    GNSS_FIX_DEAD    = 0x01,
    GNSS_FIX_2D      = 0x02,
    GNSS_FIX_3D      = 0x03,
    GNSS_FIX_COMBINED = 0x04,
    GNSS_FIX_TIME    = 0x05,
} gnss_fix_type_t;

/** @brief GNSS power mode */
typedef enum {
    GNSS_POWER_CONTINUOUS  = 0x00,
    GNSS_POWER_SAVE        = 0x01,
} gnss_power_mode_t;

/** @brief GNSS dynamic platform model */
typedef enum {
    GNSS_DYN_PORTABLE    = 0,
    GNSS_DYN_STATIONARY  = 2,
    GNSS_DYN_PEDESTRIAN  = 3,
    GNSS_DYN_AUTOMOTIVE  = 4,
    GNSS_DYN_SEA         = 5,
    GNSS_DYN_AIRBORNE_1G = 6,
    GNSS_DYN_AIRBORNE_2G = 7,
    GNSS_DYN_AIRBORNE_4G = 8, // Recommended for rockets
} gnss_dynamic_model_t;

/** @brief Navigation PVT (Position, Velocity, Time) solution */
typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    gnss_fix_type_t fix;
    uint8_t  num_sv;        // Number of satellites used
    int32_t  lon;           // Longitude (deg * 1e-7)
    int32_t  lat;           // Latitude (deg * 1e-7)
    int32_t  height;        // Height above ellipsoid (mm)
    int32_t  h_msl;         // Height above mean sea level (mm)
    int32_t  vel_n;         // NED north velocity (mm/s)
    int32_t  vel_e;         // NED east velocity (mm/s)
    int32_t  vel_d;         // NED down velocity (mm/s)
    uint16_t p_dop;         // Position DOP (0.01 scale)
} gnss_pvt_t;

/** @brief GNSS configuration */
typedef struct {
    uint16_t meas_rate_ms;          
    uint8_t  constellation_mask;    
    gnss_dynamic_model_t dyn_model; 
    gnss_power_mode_t power_mode;   
} gnss_config_t;

/** @brief GNSS device handle */
typedef struct {
    uint8_t  spi_inst;      // SPI instance (e.g., 1 for SPI1)
    uint8_t  ss_pin;        // SPI Slave Select pin number
    gnss_config_t config;   // Active configuration
    uint8_t  initialized;   // 1 if device has been initialized
} gnss_t;


/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initializes the NEO-M8Q-01A GNSS module over SPI.
 *
 * Applies the constellations, nav rate, dynamic model, and power mode defined 
 * in dev->config. Blocks until initialization sequence is acknowledged by the module.
 *
 * @param dev Pointer to the gnss_t device structure.
 * @return ti_errc_t TI_ERRC_NONE on success.
 */
ti_errc_t gnss_init(gnss_t *dev);

/**
 * @brief Polls the latest UBX-NAV-PVT solution from the GNSS module.
 *
 * @param dev Pointer to the gnss_t device structure.
 * @param pvt Pointer to a gnss_pvt_t structure to populate with the fix data.
 * @return ti_errc_t TI_ERRC_NONE on success.
 */
ti_errc_t gnss_get_pvt(gnss_t *dev, gnss_pvt_t *pvt);
