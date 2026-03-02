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
 * @file peripheral/actuator.h
 * @authors Mahir Emran, Shlok Rathi
 * @brief Actuator driver interface for MAX22216/MAX22217 solenoid controller.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "peripheral/errc.h"
#include "peripheral/pwm.h"

/**************************************************************************************************
 * @section Macros
 **************************************************************************************************/

/** @brief Dummy data word for read-only SPI transactions. */
#define ACTUATOR_SPI_DUMMY_DATA      0x0000

/** @brief Read/write bit in the SPI address byte (bit 7). */
#define ACTUATOR_SPI_RW_BIT          0x80

/** @brief Per-channel register stride in the register map. */
#define ACTUATOR_CH_STRIDE           0x0E

/** @brief Base address of channel 0 registers. */
#define ACTUATOR_CH0_BASE            0x09

/** @brief Channel register offset — duty cycle low-to-high transition. */
#define ACTUATOR_CH_REG_DCL2H        0x00

/** @brief Channel register offset — duty cycle high. */
#define ACTUATOR_CH_REG_DCH          0x01

/** @brief Channel register offset — duty cycle low. */
#define ACTUATOR_CH_REG_DCL          0x02

/** @brief Channel register offset — low-to-high transition time. */
#define ACTUATOR_CH_REG_TIMEL2H      0x03

/** @brief Channel register offset — control register 0. */
#define ACTUATOR_CH_REG_CTRL0        0x04

/** @brief Channel register offset — control register 1. */
#define ACTUATOR_CH_REG_CTRL1        0x05

/** @brief I-Monitor register address for channel 0. */
#define ACTUATOR_IMONITOR_CH0        0x45

/** @brief I-Monitor register address for channel 1. */
#define ACTUATOR_IMONITOR_CH1        0x4D

/** @brief I-Monitor register address for channel 2. */
#define ACTUATOR_IMONITOR_CH2        0x55

/** @brief I-Monitor register address for channel 3. */
#define ACTUATOR_IMONITOR_CH3        0x5D

/** @brief Global control register address. */
#define ACTUATOR_REG_GLOBAL_CTRL     0x00

/** @brief Global configuration register address. */
#define ACTUATOR_REG_GLOBAL_CFG      0x01

/** @brief Status register address. */
#define ACTUATOR_REG_STATUS          0x02

/** @brief Fault register 0 address. */
#define ACTUATOR_REG_FAULT0          0x65

/** @brief Fault register 1 address. */
#define ACTUATOR_REG_FAULT1          0x66

/** @brief Bit position of active flag in GLOBAL_CFG register. */
#define ACTUATOR_GLOBAL_CFG_ACTIVE_POS  15

/** @brief Bit mask of F_PWM_M field in GLOBAL_CTRL register. */
#define ACTUATOR_GLOBAL_CTRL_FPWMM_MSK  0x00F0

/** @brief Bit position of F_PWM_M field in GLOBAL_CTRL register. */
#define ACTUATOR_GLOBAL_CTRL_FPWMM_POS  4

/**************************************************************************************************
 * @section Type Definitions
 **************************************************************************************************/

/** @brief Actuator channel index. */
typedef enum {
  ACTUATOR_CHANNEL_0 = 0,
  ACTUATOR_CHANNEL_1 = 1,
  ACTUATOR_CHANNEL_2 = 2,
  ACTUATOR_CHANNEL_3 = 3,
  ACTUATOR_CHANNEL_COUNT = 4,
} actuator_channel_t;

/** @brief Control mode selection for a channel. */
typedef enum {
  ACTUATOR_CTRL_VDR_VDR = 0,
  ACTUATOR_CTRL_CDR_CDR = 1,
  ACTUATOR_CTRL_LIMITER_VDR = 2,
  ACTUATOR_CTRL_VDR_CDR = 3,
} actuator_ctrl_mode_t;

/** @brief PWM frequency divider. */
typedef enum {
  ACTUATOR_PWM_DIV_1 = 0,
  ACTUATOR_PWM_DIV_2 = 1,
  ACTUATOR_PWM_DIV_4 = 2,
  ACTUATOR_PWM_DIV_8 = 3,
} actuator_pwm_div_t;

/** @brief Output slew rate selection. */
typedef enum {
  ACTUATOR_SLEW_FAST = 0,
  ACTUATOR_SLEW_400V_US = 1,
  ACTUATOR_SLEW_200V_US = 2,
  ACTUATOR_SLEW_100V_US = 3,
} actuator_slew_rate_t;

/** @brief Blanking time selection. */
typedef enum {
  ACTUATOR_TBLANK_0 = 0,
  ACTUATOR_TBLANK_24 = 1,
  ACTUATOR_TBLANK_48 = 2,
  ACTUATOR_TBLANK_96 = 3,
} actuator_tblank_t;

/** @brief Actuator configuration. */
typedef struct {
  uint8_t spi_instance;                /** @brief SPI peripheral instance number. */
  uint8_t ss_pin;                      /** @brief Slave-select GPIO pin. */
  struct ti_pwm_config_t pwm_config;   /** @brief Optional PWM output configuration. */
  bool has_pwm_config;                 /** @brief True if pwm_config is populated. */
  uint8_t enable_pin;                  /** @brief Hardware enable GPIO pin. */
  uint8_t fault_pin;                   /** @brief Fault input GPIO pin (active-low). */
  uint8_t stat0_pin;                   /** @brief Status output 0 GPIO pin. */
  uint8_t stat1_pin;                   /** @brief Status output 1 GPIO pin. */
  uint8_t crc_en_pin;                  /** @brief CRC enable GPIO pin. */
  bool enable_crc;                     /** @brief True to enable SPI CRC checking. */
} actuator_config_t;

/** @brief Actuator device handle. */
typedef struct {
  actuator_config_t config;            /** @brief Active configuration. */
} actuator_t;

/** @brief Per-channel solenoid / actuator configuration. */
typedef struct {
  uint16_t dc_l2h;                     /** @brief Duty cycle for low-to-high transition. */
  uint16_t dc_h;                       /** @brief Duty cycle high state. */
  uint16_t dc_l;                       /** @brief Duty cycle low state. */
  uint16_t time_l2h;                   /** @brief Low-to-high transition time. */
  uint8_t ramp;                        /** @brief Ramp rate value (0–255). */
  bool ramp_up;                        /** @brief Enable current ramp-up. */
  bool ramp_mid;                       /** @brief Enable mid-ramp. */
  bool ramp_down;                      /** @brief Enable current ramp-down. */
  bool open_load_enable;               /** @brief Enable open-load detection. */
  bool h2l_enable;                     /** @brief Enable high-to-low transition. */
  bool hhf_enable;                     /** @brief Enable HHF protection. */
  actuator_ctrl_mode_t ctrl_mode;      /** @brief Control mode for the channel. */
  bool high_side;                      /** @brief True for high-side drive. */
  actuator_pwm_div_t pwm_div;         /** @brief PWM frequency divider. */
  actuator_tblank_t t_blank;           /** @brief Blanking time setting. */
  actuator_slew_rate_t slew_rate;      /** @brief Output slew rate. */
  uint8_t gain;                        /** @brief Current sense amplifier gain (0–3). */
  uint8_t snsf;                        /** @brief Sense filtering setting (0–3). */
} actuator_channel_config_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initializes the MAX22216/MAX22217 actuator and configures SPI and GPIO pins.
 *
 * @param dev     Pointer to the actuator device handle.
 * @param config  Pointer to the actuator configuration.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_init(actuator_t *dev, const actuator_config_t *config);

/**
 * @brief Asserts or de-asserts the hardware enable pin.
 *
 * @param dev     Pointer to the actuator device handle.
 * @param enable  True to enable, false to disable.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_set_enable(actuator_t *dev, bool enable);

/**
 * @brief Sets the ACTIVE bit in the GLOBAL_CFG register.
 *
 * @param dev     Pointer to the actuator device handle.
 * @param active  True to activate, false to deactivate.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_set_active(actuator_t *dev, bool active);

/**
 * @brief Sets the master PWM frequency divider (F_PWM_M) in GLOBAL_CTRL.
 *
 * @param dev      Pointer to the actuator device handle.
 * @param f_pwm_m  Master PWM frequency selector (0–15).
 * @return TI_ERRC_NONE on success, TI_ERRC_INVALID_ARG if f_pwm_m > 15.
 */
enum ti_errc_t actuator_set_pwm_master(actuator_t *dev, uint8_t f_pwm_m);

/**
 * @brief Writes a full channel configuration to the MAX22216/MAX22217.
 *
 * @param dev      Pointer to the actuator device handle.
 * @param channel  Channel index (0–3).
 * @param cfg      Pointer to the channel configuration.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_configure_channel(actuator_t *dev, actuator_channel_t channel, const actuator_channel_config_t *cfg);

/**
 * @brief Enables or disables a single channel in GLOBAL_CTRL.
 *
 * @param dev      Pointer to the actuator device handle.
 * @param channel  Channel index (0–3).
 * @param enable   True to enable, false to disable.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_set_channel_enable(actuator_t *dev, actuator_channel_t channel, bool enable);

/**
 * @brief Reads the STATUS register.
 *
 * @param dev         Pointer to the actuator device handle.
 * @param status      Pointer to store the 16-bit status value.
 * @param status_out  Optional pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_read_status(actuator_t *dev, uint16_t *status, uint8_t *status_out);

/**
 * @brief Reads both FAULT0 and FAULT1 registers.
 *
 * @param dev         Pointer to the actuator device handle.
 * @param fault0      Pointer to store the 16-bit FAULT0 value.
 * @param fault1      Pointer to store the 16-bit FAULT1 value.
 * @param status_out  Optional pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_read_fault(actuator_t *dev, uint16_t *fault0, uint16_t *fault1, uint8_t *status_out);

/**
 * @brief Reads the I-Monitor register for a given channel.
 *
 * @param dev         Pointer to the actuator device handle.
 * @param channel     Channel index (0–3).
 * @param i_monitor   Pointer to store the 16-bit I-Monitor value.
 * @param status_out  Optional pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t actuator_read_i_monitor(actuator_t *dev, actuator_channel_t channel, uint16_t *i_monitor, uint8_t *status_out);