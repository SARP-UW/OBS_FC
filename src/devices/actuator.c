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
 */
#include "actuator.h"
#include "peripheral/gpio.h"
#include "peripheral/spi.h"

/**************************************************************************************************
 * @section Private Helper Functions
 **************************************************************************************************/

/**
 * @brief Validates that a channel index is in the range [0, ACTUATOR_CHANNEL_COUNT).
 *
 * @param channel  Channel index to validate.
 * @return True if channel is valid, false otherwise.
 */
static inline bool actuator_channel_valid(actuator_channel_t channel) {
    return channel >= ACTUATOR_CHANNEL_0 && channel < ACTUATOR_CHANNEL_COUNT;
}

/**
 * @brief Returns the register base address for the given channel.
 *
 * @param channel  Channel index (0–3).
 * @return Base register address for the channel.
 */
static inline uint8_t actuator_channel_base(actuator_channel_t channel) {
    return ACTUATOR_CH0_BASE + (ACTUATOR_CH_STRIDE * channel);
}

/**
 * @brief Performs a single 3-byte SPI transaction (address + 16-bit data).
 *
 * @param dev         Pointer to the actuator device handle.
 * @param addr        7-bit register address.
 * @param write       True for write, false for read.
 * @param data_in     16-bit data to send (ignored for reads).
 * @param data_out    Pointer to store the 16-bit received data (may be NULL).
 * @param status_out  Pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t actuator_spi_transfer(actuator_t *dev, uint8_t addr, bool write, uint16_t data_in, uint16_t *data_out, uint8_t *status_out) {
    if (!dev) return TI_ERRC_INVALID_ARG;
    
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};

    tx[0] = (uint8_t)((write ? ACTUATOR_SPI_RW_BIT : 0x00) | (addr & 0x7F));
    tx[1] = (data_in >> 8) & 0xFF;
    tx[2] = data_in & 0xFF;

    int result = spi_transfer_sync(dev->config.spi_instance, dev->config.ss_pin, tx, rx, 3);
    if (result != 1) return TI_ERRC_UNKNOWN;

    if (status_out) *status_out = rx[0];
    if (data_out) *data_out = ((uint16_t)rx[1] << 8) | rx[2];

    return TI_ERRC_NONE;
}

/**
 * @brief Writes a 16-bit value to a register.
 *
 * @param dev         Pointer to the actuator device handle.
 * @param addr        Register address.
 * @param value       Value to write.
 * @param status_out  Optional pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t actuator_write_reg(actuator_t *dev, uint8_t addr, uint16_t value, uint8_t *status_out) {
    return actuator_spi_transfer(dev, addr, true, value, NULL, status_out);
}

/**
 * @brief Reads a 16-bit value from a register.
 *
 * Requires two SPI cycles: the first latches the address, the second returns
 * the data.
 *
 * @param dev         Pointer to the actuator device handle.
 * @param addr        Register address.
 * @param value       Pointer to store the 16-bit register value.
 * @param status_out  Optional pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t actuator_read_reg(actuator_t *dev, uint8_t addr, uint16_t *value, uint8_t *status_out) {
    enum ti_errc_t err = actuator_spi_transfer(dev, addr, false, ACTUATOR_SPI_DUMMY_DATA, NULL, NULL);
    if (err != TI_ERRC_NONE) return err;
    return actuator_spi_transfer(dev, addr, false, ACTUATOR_SPI_DUMMY_DATA, value, status_out);
}

/**
 * @brief Performs a read-modify-write on a register.
 *
 * @param dev    Pointer to the actuator device handle.
 * @param addr   Register address.
 * @param mask   Bit mask of the field to modify.
 * @param value  New value (pre-shifted to align with mask).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t actuator_update_reg(actuator_t *dev, uint8_t addr, uint16_t mask, uint16_t value) {
    uint16_t reg_val = 0;
    enum ti_errc_t err = actuator_read_reg(dev, addr, &reg_val, NULL);
    if (err != TI_ERRC_NONE) return err;
    
    reg_val = (reg_val & ~mask) | (value & mask);
    return actuator_write_reg(dev, addr, reg_val, NULL);
}

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/

enum ti_errc_t actuator_init(actuator_t *dev, const actuator_config_t *config) {
    if (!dev || !config) return TI_ERRC_INVALID_ARG;
    dev->config = *config;

    uint8_t ss_list[1] = { dev->config.ss_pin };
    if (spi_init(dev->config.spi_instance, ss_list, 1) != 1) return TI_ERRC_INVALID_ARG;

    if (dev->config.enable_pin) {
        tal_enable_clock(dev->config.enable_pin);
        tal_set_mode(dev->config.enable_pin, 1);
        tal_set_pin(dev->config.enable_pin, 0); // Default to off
    }
    if (dev->config.fault_pin) {
        tal_enable_clock(dev->config.fault_pin);
        tal_set_mode(dev->config.fault_pin, 0);
        tal_pull_pin(dev->config.fault_pin, 1);
    }

    return TI_ERRC_NONE;
}

enum ti_errc_t actuator_set_enable(actuator_t *dev, bool enable) {
    if (!dev || dev->config.enable_pin == 0) return TI_ERRC_INVALID_ARG;
    tal_set_pin(dev->config.enable_pin, enable ? 1 : 0);
    return TI_ERRC_NONE;
}

enum ti_errc_t actuator_set_active(actuator_t *dev, bool active) {
    uint16_t mask = (1U << ACTUATOR_GLOBAL_CFG_ACTIVE_POS);
    uint16_t val = (active ? 1U : 0U) << ACTUATOR_GLOBAL_CFG_ACTIVE_POS;
    return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CFG, mask, val);
}

enum ti_errc_t actuator_set_pwm_master(actuator_t *dev, uint8_t f_pwm_m) {
    if (f_pwm_m > 0x0F) return TI_ERRC_INVALID_ARG;
    uint16_t mask = ACTUATOR_GLOBAL_CTRL_FPWMM_MSK; 
    uint16_t val = (uint16_t)f_pwm_m << ACTUATOR_GLOBAL_CTRL_FPWMM_POS;
    return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CTRL, mask, val);
}

enum ti_errc_t actuator_configure_channel(actuator_t *dev, actuator_channel_t channel, const actuator_channel_config_t *cfg) {
    if (!dev || !cfg || !actuator_channel_valid(channel)) return TI_ERRC_INVALID_ARG;
    
    uint8_t base = actuator_channel_base(channel);
    
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_DCL2H, cfg->dc_l2h, NULL);   
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_DCH, cfg->dc_h, NULL);     
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_DCL, cfg->dc_l, NULL);     
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_TIMEL2H, cfg->time_l2h, NULL); 
    
    uint16_t ctrl0_val = 
        ((cfg->ctrl_mode & 0x3) << 14) |
        ((cfg->hhf_enable ? 1 : 0) << 13) |
        ((cfg->open_load_enable ? 1 : 0) << 12) |
        ((cfg->h2l_enable ? 1 : 0) << 11) |
        ((cfg->ramp_down ? 1 : 0) << 10) |
        ((cfg->ramp_mid ? 1 : 0) << 9) |
        ((cfg->ramp_up ? 1 : 0) << 8) |
        (cfg->ramp & 0xFF);

    uint16_t ctrl1_val = 
        ((cfg->high_side ? 1 : 0) << 10) |
        ((cfg->pwm_div & 0x3) << 8) |
        ((cfg->t_blank & 0x3) << 6) |
        ((cfg->slew_rate & 0x3) << 4) |
        ((cfg->gain & 0x3) << 2) |
        (cfg->snsf & 0x3);
    
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_CTRL0, ctrl0_val, NULL);
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_CTRL1, ctrl1_val, NULL);

    return TI_ERRC_NONE;
}

enum ti_errc_t actuator_set_channel_enable(actuator_t *dev, actuator_channel_t channel, bool enable) {
    if (!dev || !actuator_channel_valid(channel)) return TI_ERRC_INVALID_ARG;
    uint16_t mask = (1U << channel); 
    uint16_t val = (enable ? 1U : 0U) << channel;
    return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CTRL, mask, val);
}

enum ti_errc_t actuator_read_status(actuator_t *dev, uint16_t *status, uint8_t *status_out) {
    if (!status) return TI_ERRC_INVALID_ARG;
    return actuator_read_reg(dev, ACTUATOR_REG_STATUS, status, status_out);
}

enum ti_errc_t actuator_read_fault(actuator_t *dev, uint16_t *fault0, uint16_t *fault1, uint8_t *status_out) {
    if (!fault0 || !fault1) return TI_ERRC_INVALID_ARG;
    enum ti_errc_t err = actuator_read_reg(dev, ACTUATOR_REG_FAULT0, fault0, status_out);
    if (err != TI_ERRC_NONE) return err;
    return actuator_read_reg(dev, ACTUATOR_REG_FAULT1, fault1, status_out);
}

enum ti_errc_t actuator_read_i_monitor(actuator_t *dev, actuator_channel_t channel, uint16_t *i_monitor, uint8_t *status_out) {
    if (!i_monitor || !actuator_channel_valid(channel)) return TI_ERRC_INVALID_ARG;
    
    uint8_t imon_reg = ACTUATOR_IMONITOR_CH0 + (channel * 8); 
    
    return actuator_read_reg(dev, imon_reg, i_monitor, status_out);
}