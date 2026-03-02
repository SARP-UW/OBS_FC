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
 * @note Untested driver.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "internal/mmio.h"
#include "peripheral/errc.h"
#include "peripheral/spi.h"

/**************************************************************************************************
 * @section Type Definitions
 **************************************************************************************************/

typedef enum {
  ACTUATOR_CHANNEL_0 = 0,
  ACTUATOR_CHANNEL_1 = 1,
  ACTUATOR_CHANNEL_2 = 2,
  ACTUATOR_CHANNEL_3 = 3,
  ACTUATOR_CHANNEL_COUNT = 4,
} actuator_channel_t;

typedef enum {
  ACTUATOR_CTRL_VDR_VDR = 0,
  ACTUATOR_CTRL_CDR_CDR = 1,
  ACTUATOR_CTRL_LIMITER_VDR = 2,
  ACTUATOR_CTRL_VDR_CDR = 3,
} actuator_ctrl_mode_t;

typedef enum {
  ACTUATOR_PWM_DIV_1 = 0,
  ACTUATOR_PWM_DIV_2 = 1,
  ACTUATOR_PWM_DIV_4 = 2,
  ACTUATOR_PWM_DIV_8 = 3,
} actuator_pwm_div_t;

typedef enum {
  ACTUATOR_SLEW_FAST = 0,
  ACTUATOR_SLEW_400V_US = 1,
  ACTUATOR_SLEW_200V_US = 2,
  ACTUATOR_SLEW_100V_US = 3,
} actuator_slew_rate_t;

typedef enum {
  ACTUATOR_TBLANK_0 = 0,
  ACTUATOR_TBLANK_24 = 1,
  ACTUATOR_TBLANK_48 = 2,
  ACTUATOR_TBLANK_96 = 3,
} actuator_tblank_t;

typedef struct {
  spi_device_t spi_device;
  spi_config_t spi_config;
  uint8_t enable_pin;
  uint8_t fault_pin;
  uint8_t stat0_pin;
  uint8_t stat1_pin;
  uint8_t crc_en_pin;
  bool enable_crc;
} actuator_config_t;

typedef struct {
  spi_device_t spi_device;
  uint8_t enable_pin;
  uint8_t fault_pin;
  uint8_t stat0_pin;
  uint8_t stat1_pin;
  uint8_t crc_en_pin;
  bool enable_crc;
} actuator_t;

typedef struct {
  uint16_t dc_l2h;
  uint16_t dc_h;
  uint16_t dc_l;
  uint16_t time_l2h;
  uint8_t ramp;
  bool ramp_up;
  bool ramp_mid;
  bool ramp_down;
  bool open_load_enable;
  bool h2l_enable;
  bool hhf_enable;
  actuator_ctrl_mode_t ctrl_mode;
  bool high_side;
  actuator_pwm_div_t pwm_div;
  actuator_tblank_t t_blank;
  actuator_slew_rate_t slew_rate;
  uint8_t gain;
  uint8_t snsf;
} actuator_channel_config_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initialize an actuator device instance.
 *
 * This configures the SPI controller, CS pin, and any optional GPIO pins
 * (ENABLE, FAULT, STAT0, STAT1, CRC_EN).
 */
enum ti_errc_t actuator_init(actuator_t *dev, const actuator_config_t *config);

/**
 * @brief Drive the ENABLE pin high or low.
 */
enum ti_errc_t actuator_set_enable(actuator_t *dev, bool enable);

/**
 * @brief Write a 16-bit value to a register address.
 */
enum ti_errc_t actuator_write_reg(actuator_t *dev, uint8_t addr,
                                 uint16_t value, uint8_t *status_out);

/**
 * @brief Read a 16-bit value from a register address.
 */
enum ti_errc_t actuator_read_reg(actuator_t *dev, uint8_t addr,
                                uint16_t *value, uint8_t *status_out);

/**
 * @brief Set ACTIVE bit in `GLOBAL_CFG`.
 */
enum ti_errc_t actuator_set_active(actuator_t *dev, bool active);

/**
 * @brief Set master PWM frequency divider (F_PWM_M).
 */
enum ti_errc_t actuator_set_pwm_master(actuator_t *dev, uint8_t f_pwm_m);

/**
 * @brief Configure channel registers for solenoid/actuator control.
 */
enum ti_errc_t actuator_configure_channel(actuator_t *dev,
                                          actuator_channel_t channel,
                                          const actuator_channel_config_t *cfg);

/**
 * @brief Enable or disable a channel using `GLOBAL_CTRL` CNTL bits.
 */
enum ti_errc_t actuator_set_channel_enable(actuator_t *dev,
                                           actuator_channel_t channel,
                                           bool enable);

/**
 * @brief Read the `STATUS` register.
 */
enum ti_errc_t actuator_read_status(actuator_t *dev, uint16_t *status,
                                   uint8_t *status_out);

/**
 * @brief Read the fault log registers.
 */
enum ti_errc_t actuator_read_fault(actuator_t *dev, uint16_t *fault0,
                                  uint16_t *fault1, uint8_t *status_out);

/**
 * @brief Read the I_MONITOR register for a channel.
 */
enum ti_errc_t actuator_read_i_monitor(actuator_t *dev,
                                      actuator_channel_t channel,
                                      uint16_t *i_monitor,
                                      uint8_t *status_out);
