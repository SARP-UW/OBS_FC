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
    
    /*
     * MAX22216 SPI frame format (3 bytes total):
     *   Byte 0: [RW | A6 A5 A4 A3 A2 A1 A0]
     *           Bit 7 = 1 for write, 0 for read. Bits 6:0 = 7-bit register address.
     *   Byte 1: DATA[15:8]  (high byte of 16-bit register data)
     *   Byte 2: DATA[7:0]   (low byte of 16-bit register data)
     *
     * SPI is full-duplex, so while we send 3 bytes, we simultaneously receive 3 bytes.
     * rx[0] = status byte from the MAX22216 (fault/ready flags)
     * rx[1:2] = register data from the previous read (or garbage on first read)
     */
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};

    // Build address byte: set bit 7 if writing, mask address to 7 bits
    tx[0] = (uint8_t)((write ? ACTUATOR_SPI_RW_BIT : 0x00) | (addr & 0x7F));
    // Split 16-bit data into two bytes (big-endian, MSB first)
    tx[1] = (data_in >> 8) & 0xFF;
    tx[2] = data_in & 0xFF;

    // Perform the SPI transaction — uses the SPI instance and SS pin from our spi_config
    int result = spi_transfer_sync(dev->spi_config.spi_inst, dev->spi_config.ss_pin, tx, rx, 3);
    if (result != 1) return TI_ERRC_UNKNOWN;

    // rx[0] always contains the MAX22216 status byte (sent back on first clock byte)
    if (status_out) *status_out = rx[0];
    // rx[1:2] contain the 16-bit register data (reassemble from big-endian)
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
 * IMPORTANT: The MAX22216 requires TWO SPI cycles to read a register.
 * Cycle 1: Send the read address — the chip latches it internally but does NOT
 *          return the data yet (it returns stale data from the previous transaction).
 * Cycle 2: Send the same read address again — NOW the chip returns the actual
 *          register data from Cycle 1.
 * This is a quirk of the MAX22216 SPI protocol. If you only do one cycle,
 * you get garbage (data from whatever the last transaction was).
 *
 * @param dev         Pointer to the actuator device handle.
 * @param addr        Register address.
 * @param value       Pointer to store the 16-bit register value.
 * @param status_out  Optional pointer to store the SPI status byte (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t actuator_read_reg(actuator_t *dev, uint8_t addr, uint16_t *value, uint8_t *status_out) {
    // Cycle 1: Send address, discard response (it's stale data)
    enum ti_errc_t err = actuator_spi_transfer(dev, addr, false, ACTUATOR_SPI_DUMMY_DATA, NULL, NULL);
    if (err != TI_ERRC_NONE) return err;
    // Cycle 2: Send same address again, NOW we get the real data back
    return actuator_spi_transfer(dev, addr, false, ACTUATOR_SPI_DUMMY_DATA, value, status_out);
}

/**
 * @brief Performs a read-modify-write on a register.
 *
 * Used when you want to change specific bits in a register without
 * clobbering the other bits. For example, enabling one channel in
 * GLOBAL_CTRL without disabling the others.
 *
 * Steps: 1) Read current register value
 *        2) Clear the bits covered by `mask`
 *        3) OR in the new `value` (which must be pre-shifted to align with mask)
 *        4) Write the modified value back
 *
 * @param dev    Pointer to the actuator device handle.
 * @param addr   Register address.
 * @param mask   Bit mask of the field to modify.
 * @param value  New value (pre-shifted to align with mask).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t actuator_update_reg(actuator_t *dev, uint8_t addr, uint16_t mask, uint16_t value) {
    uint16_t reg_val = 0;
    // Step 1: Read the current register contents
    enum ti_errc_t err = actuator_read_reg(dev, addr, &reg_val, NULL);
    if (err != TI_ERRC_NONE) return err;
    
    // Step 2+3: Clear masked bits, then set new value within those bits
    //   Example: reg_val=0xFF00, mask=0x00F0, value=0x0030
    //   -> (0xFF00 & ~0x00F0) | (0x0030 & 0x00F0) = 0xFF00 | 0x0030 = 0xFF30
    reg_val = (reg_val & ~mask) | (value & mask);
    // Step 4: Write back
    return actuator_write_reg(dev, addr, reg_val, NULL);
}

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/

enum ti_errc_t actuator_init(actuator_t *dev, const actuator_spi_t *spi_config, const actuator_config_t *config) {
    if (!dev || !spi_config || !config) return TI_ERRC_INVALID_ARG;
    // Copy both configs into the device handle so all future calls can reference them
    dev->spi_config = *spi_config;
    dev->config = *config;

    // Initialize the SPI peripheral with our slave-select pin.
    // spi_init needs an array of SS pins (supports multiple slaves) — we only have one.
    uint8_t ss_list[1] = { dev->spi_config.ss_pin };
    if (spi_init(dev->spi_config.spi_inst, ss_list, 1) != 1) return TI_ERRC_INVALID_ARG;

    // Configure the hardware ENABLE pin as a GPIO output.
    // This is the physical kill switch — pulling it low cuts power to all H-bridges
    // on the MAX22216, regardless of what the software registers say.
    // Default to OFF (0) for safety — valves should not actuate until explicitly commanded.
    if (dev->config.enable_pin) {
        tal_enable_clock(dev->config.enable_pin);  // Enable GPIO port clock (STM32 requirement)
        tal_set_mode(dev->config.enable_pin, 1);   // Set as output (mode 1 = output)
        tal_set_pin(dev->config.enable_pin, 0);    // Start with enable OFF (valves safe)
    }

    // Configure the FAULT pin as a GPIO input with internal pull-up.
    // The MAX22216 drives this pin LOW when a fault occurs (open load, overcurrent, etc.).
    // Pull-up ensures it reads HIGH (no fault) when not actively driven by the chip.
    if (dev->config.fault_pin) {
        tal_enable_clock(dev->config.fault_pin);
        tal_set_mode(dev->config.fault_pin, 0);    // Set as input (mode 0 = input)
        tal_pull_pin(dev->config.fault_pin, 1);    // Enable internal pull-up
    }

    return TI_ERRC_NONE;
}

enum ti_errc_t actuator_set_enable(actuator_t *dev, bool enable) {
    if (!dev || dev->config.enable_pin == 0) return TI_ERRC_INVALID_ARG;
    // This is the HARDWARE enable — a GPIO pin connected to the MAX22216 EN pin.
    // It's separate from the SOFTWARE active bit (set_active). Both must be true
    // for the actuator to drive outputs. This is a defense-in-depth safety pattern:
    //   - Software crash? Hardware enable is still off by default.
    //   - SPI bus stuck? Pull the GPIO to kill outputs.
    tal_set_pin(dev->config.enable_pin, enable ? 1 : 0);
    return TI_ERRC_NONE;
}

enum ti_errc_t actuator_set_active(actuator_t *dev, bool active) {
    // This is the SOFTWARE active bit — bit 15 of the GLOBAL_CFG register.
    // Even if hardware enable is high, the chip won't drive outputs unless
    // this bit is also set. Two independent safety layers.
    uint16_t mask = (1U << ACTUATOR_GLOBAL_CFG_ACTIVE_POS);
    uint16_t val = (active ? 1U : 0U) << ACTUATOR_GLOBAL_CFG_ACTIVE_POS;
    return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CFG, mask, val);
}

enum ti_errc_t actuator_set_pwm_master(actuator_t *dev, uint8_t f_pwm_m) {
    // F_PWM_M is the master PWM frequency selector (bits [7:4] of GLOBAL_CTRL).
    // This sets the base frequency that all 4 channels derive from.
    // Each channel can further divide this with its own pwm_div setting.
    // Valid range: 0–15 (4-bit field).
    if (f_pwm_m > 0x0F) return TI_ERRC_INVALID_ARG;
    uint16_t mask = ACTUATOR_GLOBAL_CTRL_FPWMM_MSK; 
    uint16_t val = (uint16_t)f_pwm_m << ACTUATOR_GLOBAL_CTRL_FPWMM_POS;
    return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CTRL, mask, val);
}

enum ti_errc_t actuator_configure_channel(actuator_t *dev, actuator_channel_t channel, const actuator_channel_config_t *cfg) {
    if (!dev || !cfg || !actuator_channel_valid(channel)) return TI_ERRC_INVALID_ARG;
    
    // Each channel owns a contiguous block of registers starting at:
    //   base = 0x09 + (0x0E * channel)
    // Channel 0: 0x09–0x16, Channel 1: 0x17–0x24, Channel 2: 0x25–0x32, Channel 3: 0x33–0x40
    uint8_t base = actuator_channel_base(channel);
    
    // Write the 4 duty-cycle / timing registers.
    // These define the solenoid's "hit-and-hold" current waveform:
    //   DCL2H  = duty cycle during the low-to-high transition (the initial "hit" — high current to open fast)
    //   DCH    = duty cycle during the high (hold) phase (reduced current to hold open without overheating)
    //   DCL    = duty cycle during the low phase (off or very low current)
    //   TIMEL2H = how long the L2H phase lasts before switching to the hold phase
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_DCL2H, cfg->dc_l2h, NULL);   
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_DCH, cfg->dc_h, NULL);     
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_DCL, cfg->dc_l, NULL);     
    actuator_write_reg(dev, base + ACTUATOR_CH_REG_TIMEL2H, cfg->time_l2h, NULL); 
    
    // Pack CTRL0 register — a 16-bit bitfield with control loop settings:
    //   [15:14] ctrl_mode   — VDR/CDR selection (voltage vs current drive mode)
    //   [13]    hhf_enable  — High-side H-bridge fault detection
    //   [12]    open_load   — Detect if the solenoid coil is disconnected
    //   [11]    h2l_enable  — Enable high-to-low transition control
    //   [10]    ramp_down   — Gradual current decrease when turning off
    //   [9]     ramp_mid    — Gradual mid-phase ramping
    //   [8]     ramp_up     — Gradual current increase when turning on (prevents inrush spikes)
    //   [7:0]   ramp        — Ramp rate value (how fast the gradual changes happen)
    uint16_t ctrl0_val = 
        ((cfg->ctrl_mode & 0x3) << 14) |
        ((cfg->hhf_enable ? 1 : 0) << 13) |
        ((cfg->open_load_enable ? 1 : 0) << 12) |
        ((cfg->h2l_enable ? 1 : 0) << 11) |
        ((cfg->ramp_down ? 1 : 0) << 10) |
        ((cfg->ramp_mid ? 1 : 0) << 9) |
        ((cfg->ramp_up ? 1 : 0) << 8) |
        (cfg->ramp & 0xFF);

    // Pack CTRL1 register — a 16-bit bitfield with output stage settings:
    //   [10]   high_side  — 1=high-side drive, 0=low-side drive (depends on PCB wiring to solenoid)
    //   [9:8]  pwm_div    — Per-channel PWM frequency divider (applied on top of master F_PWM_M)
    //   [7:6]  t_blank    — Blanking time after switching edges before current sensing
    //                        (prevents false overcurrent triggers from switching transients)
    //   [5:4]  slew_rate  — How fast the output voltage transitions (Fast/400V/200V/100V per μs)
    //                        Slower = less EMI noise, faster = more responsive valve actuation
    //   [3:2]  gain       — Current sense amplifier gain (higher gain = more sensitive monitoring)
    //   [1:0]  snsf       — Sense filtering (smooths current measurements, reduces noise)
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
    // Each channel has a dedicated enable bit in GLOBAL_CTRL (bits [3:0]).
    // Bit 0 = channel 0, bit 1 = channel 1, etc.
    // Uses read-modify-write so enabling one channel doesn't disable the others.
    uint16_t mask = (1U << channel); 
    uint16_t val = (enable ? 1U : 0U) << channel;
    return actuator_update_reg(dev, ACTUATOR_REG_GLOBAL_CTRL, mask, val);
}

enum ti_errc_t actuator_read_status(actuator_t *dev, uint16_t *status, uint8_t *status_out) {
    if (!status) return TI_ERRC_INVALID_ARG;
    // STATUS register (0x02) contains per-channel and global status flags.
    // The caller interprets the bitfield based on the MAX22216 datasheet.
    return actuator_read_reg(dev, ACTUATOR_REG_STATUS, status, status_out);
}

enum ti_errc_t actuator_read_fault(actuator_t *dev, uint16_t *fault0, uint16_t *fault1, uint8_t *status_out) {
    if (!fault0 || !fault1) return TI_ERRC_INVALID_ARG;
    // Two separate fault registers capture different failure modes:
    //   FAULT0 (0x65) — overcurrent, thermal shutdown, supply undervoltage
    //   FAULT1 (0x66) — open-load detection, watchdog timeout
    // Both are read-to-clear: reading them acknowledges and resets the fault flags.
    // If the fault condition persists, the bits will set again on the next read.
    enum ti_errc_t err = actuator_read_reg(dev, ACTUATOR_REG_FAULT0, fault0, status_out);
    if (err != TI_ERRC_NONE) return err;
    return actuator_read_reg(dev, ACTUATOR_REG_FAULT1, fault1, status_out);
}

enum ti_errc_t actuator_read_i_monitor(actuator_t *dev, actuator_channel_t channel, uint16_t *i_monitor, uint8_t *status_out) {
    if (!i_monitor || !actuator_channel_valid(channel)) return TI_ERRC_INVALID_ARG;
    
    // I-Monitor registers are NOT contiguous with the channel config registers.
    // They live at addresses: CH0=0x45, CH1=0x4D, CH2=0x55, CH3=0x5D
    // Stride between them is 8 (not 0x0E like the config registers).
    // Returns a raw 16-bit ADC value proportional to the solenoid coil current.
    // Used to verify the valve actually moved — a current spike = solenoid energized.
    uint8_t imon_reg = ACTUATOR_IMONITOR_CH0 + (channel * 8); 
    
    return actuator_read_reg(dev, imon_reg, i_monitor, status_out);
}