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
 * @file peripheral/actuator.c
 * @authors Mahir Emran, Shlok Rathi
 * @brief Actuator driver implementation for MAX22216/MAX22217 solenoid controller.
 * @note Untested driver.
 */

#include "peripheral/actuator.h"
#include "peripheral/gpio.h"

// 16-bit dummy word used for read transactions.
#define ACTUATOR_SPI_DUMMY_DATA 0x0000
// MSB=1 indicates write, MSB=0 indicates read.
#define ACTUATOR_SPI_RW_BIT     0x80

// Per-channel register block stride and base for channel 0.
#define ACTUATOR_CH_STRIDE      0x0E
#define ACTUATOR_CH0_BASE       0x09

// Offsets within each channel register block.
#define ACTUATOR_CH_REG_DC_L2H   0x00
#define ACTUATOR_CH_REG_DC_H     0x01
#define ACTUATOR_CH_REG_DC_L     0x02
#define ACTUATOR_CH_REG_TIME_L2H 0x03
#define ACTUATOR_CH_REG_CTRL0    0x04
#define ACTUATOR_CH_REG_CTRL1    0x05

// I-monitor register addresses per channel (non-contiguous).
#define ACTUATOR_IMONITOR_CH0    0x45
#define ACTUATOR_IMONITOR_CH1    0x50
#define ACTUATOR_IMONITOR_CH2    0x57
#define ACTUATOR_IMONITOR_CH3    0x60
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
 * @file peripheral/actuator.c
 * @authors Mahir Emran, Shlok Rathi
 * @brief Actuator driver implementation for MAX22216/MAX22217 solenoid controller.
 * @note Untested driver.
 */

#include "peripheral/actuator.h"
#include "peripheral/gpio.h"

// 16-bit dummy word used for read transactions.
#define ACTUATOR_SPI_DUMMY_DATA 0x0000
// MSB=1 indicates write, MSB=0 indicates read.
#define ACTUATOR_SPI_RW_BIT     0x80

// Per-channel register block stride and base for channel 0.
#define ACTUATOR_CH_STRIDE      0x0E
#define ACTUATOR_CH0_BASE       0x09

// Offsets within each channel register block.
#define ACTUATOR_CH_REG_DC_L2H   0x00
#define ACTUATOR_CH_REG_DC_H     0x01
#define ACTUATOR_CH_REG_DC_L     0x02
#define ACTUATOR_CH_REG_TIME_L2H 0x03
#define ACTUATOR_CH_REG_CTRL0    0x04
#define ACTUATOR_CH_REG_CTRL1    0x05

// I-monitor register addresses per channel (non-contiguous).
#define ACTUATOR_IMONITOR_CH0    0x45
#define ACTUATOR_IMONITOR_CH1    0x50
#define ACTUATOR_IMONITOR_CH2    0x57
#define ACTUATOR_IMONITOR_CH3    0x60

// Validate channel enum is within device range.
static inline bool actuator_channel_valid(actuator_channel_t channel) {
  return channel >= ACTUATOR_CHANNEL_0 && channel < ACTUATOR_CHANNEL_COUNT;
}

// Compute base register address for a channel.
static inline uint8_t actuator_channel_base(actuator_channel_t channel) {
  return (uint8_t)(ACTUATOR_CH0_BASE + (ACTUATOR_CH_STRIDE * channel));
}

// Compute register address for a channel register offset.
static inline uint8_t actuator_channel_reg(actuator_channel_t channel,
                                           uint8_t offset) {
  return (uint8_t)(actuator_channel_base(channel) + offset);
}

// Map channel to its dedicated I-monitor register address.
static inline uint8_t actuator_imonitor_reg(actuator_channel_t channel) {
  switch (channel) {
  case ACTUATOR_CHANNEL_0:
    return ACTUATOR_IMONITOR_CH0;
  case ACTUATOR_CHANNEL_1:
    return ACTUATOR_IMONITOR_CH1;
  case ACTUATOR_CHANNEL_2:
    return ACTUATOR_IMONITOR_CH2;
  case ACTUATOR_CHANNEL_3:
    return ACTUATOR_IMONITOR_CH3;
  default:
    return ACTUATOR_IMONITOR_CH0;
  }
}

// Perform a full-duplex 3-byte SPI transfer and return status/data.
static enum ti_errc_t actuator_spi_transfer(actuator_t *dev, uint8_t addr,
                                            bool write, uint16_t data_in,
                                            uint16_t *data_out,
                                            uint8_t *status_out) {
  if (dev == NULL)
    return TI_ERRC_INVALID_ARG;

  uint8_t tx[3] = {0};
  uint8_t rx[3] = {0};

  // Command byte: R/W bit in MSB, 7-bit address in LSBs.
  tx[0] = (uint8_t)((write ? ACTUATOR_SPI_RW_BIT : 0x00) | (addr & 0x7F));
  tx[1] = (uint8_t)(data_in >> 8);
  tx[2] = (uint8_t)(data_in & 0xFF);

  if (dev->spi_device.gpio_pin != 0) {
    // Assert manual chip-select if configured.
    tal_set_pin(dev->spi_device.gpio_pin, 0);
  }

  struct spi_sync_transfer_t transfer = {
      .device = dev->spi_device,
      .source = tx,
      .dest = rx,
      .size = sizeof(tx),
      .timeout = 1000000,
      .read_inc = true,
  };

  // Execute synchronous transfer.
  enum ti_errc_t errc = spi_transfer_sync(&transfer);

  if (dev->spi_device.gpio_pin != 0) {
    // Deassert chip-select.
    tal_set_pin(dev->spi_device.gpio_pin, 1);
  }

  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  // Status is returned in the first byte; data in the remaining bytes.
  if (status_out != NULL) {
    *status_out = rx[0];
  }
  if (data_out != NULL) {
    *data_out = (uint16_t)((rx[1] << 8) | rx[2]);
  }

  return TI_ERRC_NONE;
}

// Read-modify-write a register with mask/value.
static enum ti_errc_t actuator_update_reg(actuator_t *dev, uint8_t addr,
                                          uint16_t mask, uint16_t value) {
  uint16_t reg_val = 0;
  enum ti_errc_t errc = actuator_read_reg(dev, addr, &reg_val, NULL);
  if (errc != TI_ERRC_NONE)
    return errc;
  reg_val = (uint16_t)((reg_val & ~mask) | (value & mask));
  return actuator_write_reg(dev, addr, reg_val, NULL);
}

// Initialize SPI, device struct, and GPIO pins used by the driver.
enum ti_errc_t actuator_init(actuator_t *dev, const actuator_config_t *config) {
  if (dev == NULL || config == NULL)
    return TI_ERRC_INVALID_ARG;

  if (config->enable_crc) {
    // CRC is not supported by this driver yet.
    return TI_ERRC_INVALID_ARG;
  }

  enum ti_errc_t errc = spi_init(config->spi_device.instance,
                                 (spi_config_t *)&config->spi_config);
  if (errc != TI_ERRC_NONE)
    return errc;

  errc = spi_device_init(config->spi_device);
  if (errc != TI_ERRC_NONE)
    return errc;

  // Store configuration locally.
  dev->spi_device = config->spi_device;
  dev->enable_pin = config->enable_pin;
  dev->fault_pin = config->fault_pin;
  dev->stat0_pin = config->stat0_pin;
  dev->stat1_pin = config->stat1_pin;
  dev->crc_en_pin = config->crc_en_pin;
  dev->enable_crc = config->enable_crc;

  if (dev->enable_pin != 0) {
    // Enable pin: output, default low.
    tal_enable_clock(dev->enable_pin);
    tal_set_mode(dev->enable_pin, 1);
    tal_set_pin(dev->enable_pin, 0);
  }

  if (dev->fault_pin != 0) {
    // Fault pin: input with pull-up.
    tal_enable_clock(dev->fault_pin);
    tal_set_mode(dev->fault_pin, 0);
    tal_pull_pin(dev->fault_pin, 1);
  }

  if (dev->stat0_pin != 0) {
    // Status pins: input, no pull.
    tal_enable_clock(dev->stat0_pin);
    tal_set_mode(dev->stat0_pin, 0);
    tal_pull_pin(dev->stat0_pin, 0);
  }

  if (dev->stat1_pin != 0) {
    // Status pins: input, no pull.
    tal_enable_clock(dev->stat1_pin);
    tal_set_mode(dev->stat1_pin, 0);
    tal_pull_pin(dev->stat1_pin, 0);
  }

  if (dev->crc_en_pin != 0) {
    // CRC enable pin: output, default low.
    tal_enable_clock(dev->crc_en_pin);
    tal_set_mode(dev->crc_en_pin, 1);
    tal_set_pin(dev->crc_en_pin, 0);
  }

  return TI_ERRC_NONE;
}

// Drive the device enable pin high/low.
enum ti_errc_t actuator_set_enable(actuator_t *dev, bool enable) {
  if (dev == NULL || dev->enable_pin == 0)
    return TI_ERRC_INVALID_ARG;
  tal_set_pin(dev->enable_pin, enable ? 1 : 0);
  return TI_ERRC_NONE;
}

// Write a 16-bit register; optional status byte out.
enum ti_errc_t actuator_write_reg(actuator_t *dev, uint8_t addr,
                                 uint16_t value, uint8_t *status_out) {
  return actuator_spi_transfer(dev, addr, true, value, NULL, status_out);
}

// Read a 16-bit register; performs a dummy read then a capture read.
enum ti_errc_t actuator_read_reg(actuator_t *dev, uint8_t addr,
                                uint16_t *value, uint8_t *status_out) {
  enum ti_errc_t errc = actuator_spi_transfer(dev, addr, false,
                                              ACTUATOR_SPI_DUMMY_DATA, NULL,
                                              NULL);
  if (errc != TI_ERRC_NONE)
    return errc;

  return actuator_spi_transfer(dev, addr, false, ACTUATOR_SPI_DUMMY_DATA,
                               value, status_out);
}

// Set the global ACTIVE bit to arm/disarm outputs.
enum ti_errc_t actuator_set_active(actuator_t *dev, bool active) {
  uint16_t mask = (uint16_t)(1u << ACTUATOR_GLOBAL_CFG_ACTIVE_POS);
  uint16_t value = (uint16_t)((active ? 1u : 0u)
                              << ACTUATOR_GLOBAL_CFG_ACTIVE_POS);
  return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CFG, mask, value);
}

// Configure the master PWM frequency divider.
enum ti_errc_t actuator_set_pwm_master(actuator_t *dev, uint8_t f_pwm_m) {
  if (f_pwm_m > 0x0F)
    return TI_ERRC_INVALID_ARG;
  uint16_t mask = (uint16_t)ACTUATOR_GLOBAL_CTRL_F_PWM_M_MSK;
  uint16_t value = (uint16_t)(f_pwm_m << ACTUATOR_GLOBAL_CTRL_F_PWM_M_POS);
  return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CTRL, mask, value);
}

// Configure all per-channel registers from a config struct.
enum ti_errc_t actuator_configure_channel(actuator_t *dev,
                                          actuator_channel_t channel,
                                          const actuator_channel_config_t *cfg) {
  if (dev == NULL || cfg == NULL)
    return TI_ERRC_INVALID_ARG;
  if (!actuator_channel_valid(channel))
    return TI_ERRC_INVALID_ARG;

  // Resolve channel-specific register addresses.
  uint8_t dc_l2h = actuator_channel_reg(channel, ACTUATOR_CH_REG_DC_L2H);
  uint8_t dc_h = actuator_channel_reg(channel, ACTUATOR_CH_REG_DC_H);
  uint8_t dc_l = actuator_channel_reg(channel, ACTUATOR_CH_REG_DC_L);
  uint8_t time_l2h = actuator_channel_reg(channel, ACTUATOR_CH_REG_TIME_L2H);
  uint8_t ctrl0 = actuator_channel_reg(channel, ACTUATOR_CH_REG_CTRL0);
  uint8_t ctrl1 = actuator_channel_reg(channel, ACTUATOR_CH_REG_CTRL1);

  // Write duty-cycle and timing parameters.
  enum ti_errc_t errc = actuator_write_reg(dev, dc_l2h, cfg->dc_l2h, NULL);
  if (errc != TI_ERRC_NONE)
    return errc;
  errc = actuator_write_reg(dev, dc_h, cfg->dc_h, NULL);
  if (errc != TI_ERRC_NONE)
    return errc;
  errc = actuator_write_reg(dev, dc_l, cfg->dc_l, NULL);
  if (errc != TI_ERRC_NONE)
    return errc;
  errc = actuator_write_reg(dev, time_l2h, cfg->time_l2h, NULL);
  if (errc != TI_ERRC_NONE)
    return errc;

  // Build CTRL0 word from config bitfields.
  uint16_t ctrl0_val = 0;
  ctrl0_val |= (uint16_t)((cfg->ctrl_mode & 0x3)
                          << ACTUATOR_CFG_CTRL0_CTRL_MODE_POS);
  ctrl0_val |= (uint16_t)((cfg->hhf_enable ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL0_HHF_EN_POS);
  ctrl0_val |= (uint16_t)((cfg->open_load_enable ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL0_OL_EN_POS);
  ctrl0_val |= (uint16_t)((cfg->h2l_enable ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL0_H2L_EN_POS);
  ctrl0_val |= (uint16_t)((cfg->ramp_down ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL0_RDWE_POS);
  ctrl0_val |= (uint16_t)((cfg->ramp_mid ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL0_RMDE_POS);
  ctrl0_val |= (uint16_t)((cfg->ramp_up ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL0_RUPE_POS);
  ctrl0_val |= (uint16_t)(cfg->ramp);

  // Build CTRL1 word from config bitfields.
  uint16_t ctrl1_val = 0;
  ctrl1_val |= (uint16_t)((cfg->high_side ? 1u : 0u)
                          << ACTUATOR_CFG_CTRL1_HSNLS_POS);
  ctrl1_val |= (uint16_t)((cfg->pwm_div & 0x3)
                          << ACTUATOR_CFG_CTRL1_F_PWM_POS);
  ctrl1_val |= (uint16_t)((cfg->t_blank & 0x3)
                          << ACTUATOR_CFG_CTRL1_T_BLANK_POS);
  ctrl1_val |= (uint16_t)((cfg->slew_rate & 0x3)
                          << ACTUATOR_CFG_CTRL1_SLEW_POS);
  ctrl1_val |= (uint16_t)((cfg->gain & 0x3)
                          << ACTUATOR_CFG_CTRL1_GAIN_POS);
  ctrl1_val |= (uint16_t)((cfg->snsf & 0x3)
                          << ACTUATOR_CFG_CTRL1_SNSF_POS);

  // Write control words.
  errc = actuator_write_reg(dev, ctrl0, ctrl0_val, NULL);
  if (errc != TI_ERRC_NONE)
    return errc;

  return actuator_write_reg(dev, ctrl1, ctrl1_val, NULL);
}

// Enable/disable a channel via global control register.
enum ti_errc_t actuator_set_channel_enable(actuator_t *dev,
                                           actuator_channel_t channel,
                                           bool enable) {
  if (dev == NULL)
    return TI_ERRC_INVALID_ARG;
  if (!actuator_channel_valid(channel))
    return TI_ERRC_INVALID_ARG;

  uint16_t mask = (uint16_t)(1u << ACTUATOR_GLOBAL_CTRL_CNTL_POS(channel));
  uint16_t value = (uint16_t)((enable ? 1u : 0u)
                              << ACTUATOR_GLOBAL_CTRL_CNTL_POS(channel));
  return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CTRL, mask, value);
}

// Read status register (optionally return status byte).
enum ti_errc_t actuator_read_status(actuator_t *dev, uint16_t *status,
                                   uint8_t *status_out) {
  if (status == NULL)
    return TI_ERRC_INVALID_ARG;
  return actuator_read_reg(dev, ACTUATOR_REG_STATUS, status, status_out);
}

// Read both fault registers.
enum ti_errc_t actuator_read_fault(actuator_t *dev, uint16_t *fault0,
                                  uint16_t *fault1, uint8_t *status_out) {
  if (fault0 == NULL || fault1 == NULL)
    return TI_ERRC_INVALID_ARG;

  enum ti_errc_t errc = actuator_read_reg(dev, ACTUATOR_REG_FAULT0, fault0,
                                          status_out);
  if (errc != TI_ERRC_NONE)
    return errc;
  return actuator_read_reg(dev, ACTUATOR_REG_FAULT1, fault1, status_out);
}

// Read per-channel current monitor register.
enum ti_errc_t actuator_read_i_monitor(actuator_t *dev,
                                      actuator_channel_t channel,
                                      uint16_t *i_monitor,
                                      uint8_t *status_out) {
  if (i_monitor == NULL)
    return TI_ERRC_INVALID_ARG;
  if (!actuator_channel_valid(channel))
    return TI_ERRC_INVALID_ARG;

  return actuator_read_reg(dev, actuator_imonitor_reg(channel), i_monitor,
                           status_out);
}
