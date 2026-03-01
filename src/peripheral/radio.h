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
 * @file peripheral/radio.h
 * @authors Mahir Emran
 * @brief Driver interface for the Silicon Labs Si4468 radio.
 * @note Untested driver.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "internal/mmio.h"
#include "peripheral/errc.h"
#include "peripheral/gpio.h"
#include "peripheral/spi.h"

/**************************************************************************************************
 * @section Macros
 **************************************************************************************************/
#define RADIO_MAX_PACKET_SIZE SI446X_MAX_PACKET_SIZE

/**************************************************************************************************
 * @section Type Definitions
 **************************************************************************************************/

typedef struct {
  const uint8_t *data;
  size_t len;
} radio_cmd_t;

typedef struct {
  uint8_t spi_instance;
  uint8_t ss_pin;
  uint8_t reset_pin;
  uint8_t nirq_pin;
  bool reset_active_high;
  uint8_t channel;
  uint32_t cts_timeout;
  uint32_t spi_timeout;
  uint32_t reset_delay_cycles;
  bool apply_errata_12;
  const uint8_t *patch_data;
  size_t patch_len;
  const uint8_t *patch_cmd;
  size_t patch_cmd_len;

  const uint8_t *power_up_cmd;
  size_t power_up_len;
  const uint8_t *gpio_cfg_cmd;
  size_t gpio_cfg_len;
  const radio_cmd_t *init_cmds;
  size_t init_cmd_count;
} radio_config_t;

typedef struct {
  uint8_t spi_instance;
  uint8_t ss_pin;
  uint8_t reset_pin;
  uint8_t nirq_pin;
  bool reset_active_high;
  uint8_t channel;
  uint32_t cts_timeout;
  uint32_t spi_timeout;
  uint32_t reset_delay_cycles;
  bool apply_errata_12;
} radio_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initialize a radio device instance.
 *
 * Configures SPI, GPIO pins, resets the device, applies optional patches
 * and configuration commands.  See radio_config_t for available options.
 */
enum ti_errc_t radio_init(radio_t *dev, const radio_config_t *config);

/**
 * @brief Toggle the hardware reset line and re-apply errata workarounds.
 */
enum ti_errc_t radio_reset(radio_t *dev);

/**
 * @brief Transmit a single packet.
 *
 * Clears the TX FIFO, writes @p data, starts TX on the device channel,
 * then re-enters RX mode.  Mirrors the transmit flow in radio.py.
 *
 * @param data  Pointer to packet bytes.
 * @param len   Number of bytes (1..RADIO_MAX_PACKET_SIZE).
 */
enum ti_errc_t radio_transmit(radio_t *dev, const uint8_t *data, size_t len);

/**
 * @brief Receive a single packet.
 *
 * Reads the packet length via PACKET_INFO, then reads that many bytes
 * from the RX FIFO.  Mirrors the receive flow in radio.py.
 *
 * @param data       Output buffer (must hold at least @p max_len bytes).
 * @param max_len    Maximum bytes to read.
 * @param actual_len Set to the number of bytes actually received.
 */
enum ti_errc_t radio_receive(radio_t *dev, uint8_t *data, size_t max_len,
                             size_t *actual_len);

/**
 * @brief Read and clear the chip interrupt status registers.
 */
enum ti_errc_t radio_get_int_status(radio_t *dev, uint8_t *ph_status,
                                   uint8_t *modem_status,
                                   uint8_t *chip_status);

/**
 * @brief Return true when the NIRQ pin is asserted (active-low).
 */
bool radio_nirq_asserted(radio_t *dev);
