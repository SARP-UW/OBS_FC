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
  spi_device_t spi_device;
  spi_config_t spi_config;
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
  spi_device_t spi_device;
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

enum ti_errc_t radio_init(radio_t *dev, const radio_config_t *config);

enum ti_errc_t radio_reset(radio_t *dev);

enum ti_errc_t radio_apply_cmds(radio_t *dev, const radio_cmd_t *cmds,
                               size_t count);

enum ti_errc_t radio_send_cmd(radio_t *dev, const uint8_t *cmd, size_t len);

enum ti_errc_t radio_send_cmd_get_resp(radio_t *dev, const uint8_t *cmd,
                                      size_t cmd_len, uint8_t *resp,
                                      size_t resp_len);

enum ti_errc_t radio_upload_patch(radio_t *dev, const uint8_t *patch_data,
                                 size_t patch_len);

enum ti_errc_t radio_write_tx_fifo(radio_t *dev, const uint8_t *data,
                                  size_t len);

enum ti_errc_t radio_read_rx_fifo(radio_t *dev, uint8_t *data, size_t len);

enum ti_errc_t radio_start_tx(radio_t *dev, uint8_t channel, uint8_t condition,
                             uint16_t length, uint16_t tx_delay);

enum ti_errc_t radio_start_rx(radio_t *dev, uint8_t channel,
                             const uint8_t *args, size_t args_len);

enum ti_errc_t radio_change_state(radio_t *dev, uint8_t next_state);

enum ti_errc_t radio_read_frr(radio_t *dev, uint8_t frr_index,
                             uint8_t *value);

enum ti_errc_t radio_get_int_status(radio_t *dev, uint8_t *ph_status,
                                   uint8_t *modem_status,
                                   uint8_t *chip_status);

enum ti_errc_t radio_fifo_info(radio_t *dev, uint8_t arg, uint8_t *resp,
                              size_t resp_len);

enum ti_errc_t radio_get_packet_info(radio_t *dev, uint8_t *resp,
                                    size_t resp_len);

bool radio_nirq_asserted(radio_t *dev);
