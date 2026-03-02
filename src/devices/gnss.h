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

/** @brief GNSS constellation selection.
 *  Bitmask flags — OR together to enable multiple satellite systems simultaneously.
 *  More constellations = more visible satellites = better fix accuracy and resilience.
 *  NEO-M8Q can track up to 3 constellations concurrently (hardware limit). */
typedef enum {
    GNSS_CONSTELLATION_GPS       = 0x01,  // US system, 31 satellites. Most universal.
    GNSS_CONSTELLATION_GLONASS   = 0x02,  // Russian system, 24 satellites. Good high-latitude coverage.
    GNSS_CONSTELLATION_BEIDOU    = 0x04,  // Chinese system, 35 satellites. Strong Asia-Pacific coverage.
    GNSS_CONSTELLATION_GALILEO   = 0x08,  // EU system, 28 satellites. Highest civilian accuracy.
    GNSS_CONSTELLATION_SBAS      = 0x10,  // Augmentation system — ground stations broadcast corrections
                                           //   via geostationary satellites. Improves accuracy, not a full constellation.
    GNSS_CONSTELLATION_QZSS      = 0x20,  // Japanese regional system, 4 satellites over Asia-Pacific.
} gnss_constellation_t;

/** @brief GNSS fix type — quality of the current navigation solution.
 *  Must be >= GNSS_FIX_3D to trust position/velocity data during flight. */
typedef enum {
    GNSS_FIX_NONE     = 0x00, // No fix at all. Not enough satellites visible.
    GNSS_FIX_DEAD     = 0x01, // Dead reckoning only — estimating from last known position + velocity.
                               //   No satellite confirmation, unreliable.
    GNSS_FIX_2D       = 0x02, // 3 satellites locked — gives lat/lon but NO altitude.
    GNSS_FIX_3D       = 0x03, // 4+ satellites locked — gives lat/lon/altitude. Minimum for flight.
    GNSS_FIX_COMBINED = 0x04, // 3D fix + dead reckoning blended. Best accuracy.
    GNSS_FIX_TIME     = 0x05, // Enough satellites to determine time, but not position.
                               //   Happens early in startup before full acquisition.
} gnss_fix_type_t;

/** @brief GNSS power mode — tradeoff between accuracy and battery life. */
typedef enum {
    GNSS_POWER_CONTINUOUS  = 0x00, // Always tracking satellites (~25-30 mA). Best accuracy,
                                    //   fastest fixes. Use during flight — can't afford gaps.
    GNSS_POWER_SAVE        = 0x01, // Duty-cycles the tracking engine (~10-15 mA). Sleeps between
                                    //   fixes. Slower time-to-fix. Use on ground during standby.
} gnss_power_mode_t;

/** @brief GNSS dynamic platform model.
 *  Tells the module's Kalman filter what motion profile to expect. If set too low,
 *  the filter rejects valid high-speed/high-altitude measurements as "noise" and
 *  you LOSE YOUR FIX mid-flight. MUST use AIRBORNE_4G for rockets.
 *
 *  Model          | Max Alt  | Max Vel   | Max Accel
 *  PORTABLE       | 12 km    | 310 m/s   | —
 *  STATIONARY     | 9 km     | 10 m/s    | —
 *  PEDESTRIAN     | 9 km     | 30 m/s    | —
 *  AUTOMOTIVE     | 9 km     | 100 m/s   | —
 *  SEA            | 500 m    | 25 m/s    | —
 *  AIRBORNE_1G    | 50 km    | 100 m/s   | 1G
 *  AIRBORNE_2G    | 50 km    | 250 m/s   | 2G
 *  AIRBORNE_4G    | 50 km    | 500 m/s   | 4G   <-- use this
 */
typedef enum {
    GNSS_DYN_PORTABLE    = 0,  // General use. 12km / 310m/s cap.
    GNSS_DYN_STATIONARY  = 2,  // Fixed position. 9km / 10m/s cap.
    GNSS_DYN_PEDESTRIAN  = 3,  // Walking. 9km / 30m/s cap.
    GNSS_DYN_AUTOMOTIVE  = 4,  // Cars. 9km / 100m/s cap.
    GNSS_DYN_SEA         = 5,  // Boats. 500m / 25m/s cap.
    GNSS_DYN_AIRBORNE_1G = 6,  // Light aircraft. 50km / 100m/s / 1G.
    GNSS_DYN_AIRBORNE_2G = 7,  // Fast/aerobatic aircraft. 50km / 250m/s / 2G.
    GNSS_DYN_AIRBORNE_4G = 8,  // Rockets. 50km / 500m/s / 4G.
} gnss_dynamic_model_t;

/** @brief Navigation PVT (Position, Velocity, Time) solution.
 *  This is the main data struct returned by gnss_get_pvt(). Contains
 *  the full navigation fix: where we are, how fast we're moving, and when. */
typedef struct {
    uint16_t year;          // UTC year
    uint8_t  month;         // UTC month (1-12)
    uint8_t  day;           // UTC day (1-31)
    uint8_t  hour;          // UTC hour (0-23)
    uint8_t  min;           // UTC minute (0-59)
    uint8_t  sec;           // UTC second (0-60, 60 = leap second)
    gnss_fix_type_t fix;    // Quality of the solution (None/2D/3D/etc). Must be >= 3D to trust position.
    uint8_t  num_sv;        // Number of satellites used in this fix. More = better accuracy.
    int32_t  lon;           // Longitude in degrees * 1e-7 (e.g., -1224000000 = -122.4°)
    int32_t  lat;           // Latitude in degrees * 1e-7 (e.g., 474000000 = 47.4°)
    int32_t  height;        // Height above WGS84 ellipsoid in mm (mathematical Earth model)
    int32_t  h_msl;         // Height above mean sea level in mm (what people call "altitude")
    int32_t  vel_n;         // North velocity in mm/s (NED frame). Positive = moving north.
    int32_t  vel_e;         // East velocity in mm/s (NED frame). Positive = moving east.
    int32_t  vel_d;         // Down velocity in mm/s (NED frame). Positive = moving DOWN.
                            //   Sign flip (positive → negative) indicates apogee.
    uint16_t p_dop;         // Position Dilution of Precision (scaled by 0.01).
                            //   Lower = better geometry. <2.0 = excellent, >5.0 = poor.
} gnss_pvt_t;

/** @brief GNSS configuration. Sent to the module during gnss_init(). */
typedef struct {
    uint16_t meas_rate_ms;          // How often to compute a fix (ms). 1000=1Hz, 200=5Hz, 100=10Hz.
                                    //   Faster = more SPI traffic + power. Use 1Hz on pad, 5-10Hz in flight.
    uint8_t  constellation_mask;    // Bitmask of which satellite systems to use. OR together:
                                    //   GPS=0x01 | GLONASS=0x02 | BeiDou=0x04 | Galileo=0x08
                                    //   More constellations = more satellites = better fix.
    gnss_dynamic_model_t dyn_model; // Tells the module's filter what motion to expect.
                                    //   MUST be AIRBORNE_4G for rockets, or fix drops out at high alt/speed.
    gnss_power_mode_t power_mode;   // CONTINUOUS = always tracking (use in flight).
                                    //   POWER_SAVE = duty-cycles tracking (use on ground to save battery).
} gnss_config_t;

/** @brief SPI config for GNSS */
typedef struct {
    uint8_t  spi_inst;      // SPI peripheral instance (1=SPI1, 2=SPI2, etc.)
    uint8_t  ss_pin;        // Slave Select GPIO pin — directly from schematic.
} gnss_spi_t;

/** @brief GNSS device handle. Allocate one of these and pass to all gnss_* functions. */
typedef struct {
    gnss_spi_t  spi_config; // SPI bus + chip select pin for this module
    gnss_config_t config;   // Active configuration (written to module during init)
    uint8_t  initialized;   // Set to 1 after gnss_init() succeeds. Guards against use-before-init.
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
