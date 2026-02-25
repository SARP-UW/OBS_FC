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
 * @file peripheral/radio.c
 * @authors Mahir Emran
 * @brief Driver implementation for the Silicon Labs Si4468 radio.
 * @note Untested driver.
 */

#include "peripheral/radio.h"

#include <string.h>

#include "peripheral/errc.h"

#define RADIO_DEFAULT_CTS_TIMEOUT 1000000U
#define RADIO_DEFAULT_SPI_TIMEOUT 1000000U
#define RADIO_DEFAULT_RESET_DELAY 100000U
#define RADIO_CMD_BUFFER_MAX SI446X_CMD_BUFFER_SIZE

#define RADIO_PATCH_CHUNK_SIZE 64U

static const uint8_t radio_errata12_cmd[] = {0xF1, 0x47, 0x4B, 0x00};

/**************************************************************************************************
 * @section Private Helpers
 **************************************************************************************************/
static void radio_delay_cycles(uint32_t cycles) {
  for (uint32_t i = 0; i < cycles; i++) {
    asm("nop");
  }
}

static enum ti_errc_t radio_spi_transfer(radio_t *dev, const uint8_t *tx,
                                        uint8_t *rx, size_t len) {
  if (dev == NULL || tx == NULL || rx == NULL || len == 0) {
    return TI_ERRC_INVALID_ARG;
  }

  if (dev->spi_device.gpio_pin != 0) {
    tal_set_pin(dev->spi_device.gpio_pin, 0);
  }

  struct spi_sync_transfer_t transfer = {
      .device = dev->spi_device,
      .source = (void *)tx,
      .dest = (void *)rx,
      .size = len,
      .timeout = dev->spi_timeout,
      .read_inc = true,
  };

  enum ti_errc_t errc = spi_transfer_sync(&transfer);

  if (dev->spi_device.gpio_pin != 0) {
    tal_set_pin(dev->spi_device.gpio_pin, 1);
  }

  return errc;
}

static enum ti_errc_t radio_read_cmd_buf(radio_t *dev, uint8_t *cts,
                                        uint8_t *resp, size_t resp_len) {
  if (dev == NULL || cts == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  size_t total = 2 + resp_len;
  if (resp_len > RADIO_CMD_BUFFER_MAX || total > (RADIO_CMD_BUFFER_MAX + 2)) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t tx[RADIO_CMD_BUFFER_MAX + 2] = {0};
  uint8_t rx[RADIO_CMD_BUFFER_MAX + 2] = {0};

  // READ_CMD_BUFFER returns CTS + optional response bytes.
  tx[0] = SI446X_CMD_READ_CMD_BUFFER;
  memset(&tx[1], 0, total - 1);

  enum ti_errc_t errc = radio_spi_transfer(dev, tx, rx, total);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  *cts = rx[1];
  if (resp != NULL && resp_len > 0) {
    memcpy(resp, &rx[2], resp_len);
  }

  return TI_ERRC_NONE;
}

static enum ti_errc_t radio_wait_cts(radio_t *dev) {
  if (dev == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  uint32_t timeout = dev->cts_timeout;
  if (timeout == 0) {
    timeout = RADIO_DEFAULT_CTS_TIMEOUT;
  }

  // Poll CTS until the chip reports ready or we time out.
  for (uint32_t i = 0; i < timeout; i++) {
    uint8_t cts = 0;
    enum ti_errc_t errc = radio_read_cmd_buf(dev, &cts, NULL, 0);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    if (cts == SI446X_CTS_READY_VALUE) {
      return TI_ERRC_NONE;
    }
  }

  return TI_ERRC_RADIO_CTS_TIMEOUT;
}

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/

enum ti_errc_t radio_init(radio_t *dev, const radio_config_t *config) {
  if (dev == NULL || config == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  enum ti_errc_t errc = spi_init(config->spi_device.instance,
                                 (spi_config_t *)&config->spi_config);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  errc = spi_device_init(config->spi_device);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  dev->spi_device = config->spi_device;
  dev->reset_pin = config->reset_pin;
  dev->nirq_pin = config->nirq_pin;
  dev->reset_active_high = config->reset_active_high;
  dev->channel = config->channel;
  dev->cts_timeout = config->cts_timeout;
  dev->spi_timeout = config->spi_timeout == 0 ? RADIO_DEFAULT_SPI_TIMEOUT
                                              : config->spi_timeout;
  dev->reset_delay_cycles =
      config->reset_delay_cycles == 0 ? RADIO_DEFAULT_RESET_DELAY
                                      : config->reset_delay_cycles;
  dev->apply_errata_12 = config->apply_errata_12;

  if (dev->reset_pin != 0) {
    tal_enable_clock(dev->reset_pin);
    tal_set_mode(dev->reset_pin, 1);
  }

  if (dev->nirq_pin != 0) {
    tal_enable_clock(dev->nirq_pin);
    tal_set_mode(dev->nirq_pin, 0);
    tal_pull_pin(dev->nirq_pin, 1);
  }

  // Reset first so the device starts from a known state.
  errc = radio_reset(dev);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  if (config->power_up_cmd != NULL && config->power_up_len > 0) {
    errc = radio_send_cmd(dev, config->power_up_cmd, config->power_up_len);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_wait_cts(dev);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
  }

  if (dev->apply_errata_12) {
    // Errata 12 requires a command sequence and state toggles after power-up.
    errc = radio_send_cmd(dev, radio_errata12_cmd, sizeof(radio_errata12_cmd));
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_wait_cts(dev);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_change_state(dev, SI446X_STATE_SLEEP);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_change_state(dev, SI446X_STATE_READY);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
  }

  if (config->patch_data != NULL && config->patch_len > 0) {
    // Apply the patch before configuration commands.
    errc = radio_upload_patch(dev, config->patch_data, config->patch_len);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    if (config->patch_cmd != NULL && config->patch_cmd_len > 0) {
      errc = radio_send_cmd(dev, config->patch_cmd, config->patch_cmd_len);
      if (errc != TI_ERRC_NONE) {
        return errc;
      }
      errc = radio_wait_cts(dev);
      if (errc != TI_ERRC_NONE) {
        return errc;
      }
    }
  }

  if (config->gpio_cfg_cmd != NULL && config->gpio_cfg_len > 0) {
    errc = radio_send_cmd(dev, config->gpio_cfg_cmd, config->gpio_cfg_len);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_wait_cts(dev);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
  }

  if (config->init_cmds != NULL && config->init_cmd_count > 0) {
    errc = radio_apply_cmds(dev, config->init_cmds, config->init_cmd_count);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
  }

  errc = radio_get_int_status(dev, NULL, NULL, NULL);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  return TI_ERRC_NONE;
}

enum ti_errc_t radio_reset(radio_t *dev) {
  if (dev == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  // Allow boards that use external reset wiring.
  if (dev->reset_pin == 0) {
    return TI_ERRC_NONE;
  }

  uint8_t active = dev->reset_active_high ? 1 : 0;
  uint8_t inactive = dev->reset_active_high ? 0 : 1;

  // Toggle the reset line with a simple delay between edges.
  tal_set_pin(dev->reset_pin, active);
  radio_delay_cycles(dev->reset_delay_cycles);
  tal_set_pin(dev->reset_pin, inactive);
  radio_delay_cycles(dev->reset_delay_cycles);

  if (dev->apply_errata_12) {
    // Errata 12 sequence assumes the device is ready to accept commands.
    enum ti_errc_t errc = radio_send_cmd(dev, radio_errata12_cmd,
                                         sizeof(radio_errata12_cmd));
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_wait_cts(dev);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_change_state(dev, SI446X_STATE_SLEEP);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_change_state(dev, SI446X_STATE_READY);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
  }

  return TI_ERRC_NONE;
}

enum ti_errc_t radio_apply_cmds(radio_t *dev, const radio_cmd_t *cmds,
                               size_t count) {
  if (dev == NULL || cmds == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  for (size_t i = 0; i < count; i++) {
    if (cmds[i].data == NULL || cmds[i].len == 0) {
      return TI_ERRC_INVALID_ARG;
    }
    enum ti_errc_t errc = radio_send_cmd(dev, cmds[i].data, cmds[i].len);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    errc = radio_wait_cts(dev);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
  }

  return TI_ERRC_NONE;
}

enum ti_errc_t radio_send_cmd(radio_t *dev, const uint8_t *cmd, size_t len) {
  if (dev == NULL || cmd == NULL || len == 0) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t rx[RADIO_CMD_BUFFER_MAX] = {0};
  if (len > RADIO_CMD_BUFFER_MAX) {
    return TI_ERRC_INVALID_ARG;
  }

  return radio_spi_transfer(dev, cmd, rx, len);
}

enum ti_errc_t radio_send_cmd_get_resp(radio_t *dev, const uint8_t *cmd,
                                      size_t cmd_len, uint8_t *resp,
                                      size_t resp_len) {
  if (dev == NULL || cmd == NULL || cmd_len == 0) {
    return TI_ERRC_INVALID_ARG;
  }

  if (resp_len > RADIO_CMD_BUFFER_MAX) {
    return TI_ERRC_INVALID_ARG;
  }

  // Standard command sequence: send, wait CTS, then read response.
  enum ti_errc_t errc = radio_send_cmd(dev, cmd, cmd_len);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  errc = radio_wait_cts(dev);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  if (resp != NULL && resp_len > 0) {
    uint8_t cts = 0;
    errc = radio_read_cmd_buf(dev, &cts, resp, resp_len);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }
    if (cts != SI446X_CTS_READY_VALUE) {
      return TI_ERRC_RADIO_CTS_TIMEOUT;
    }
  }

  return TI_ERRC_NONE;
}

enum ti_errc_t radio_upload_patch(radio_t *dev, const uint8_t *patch_data,
                                 size_t patch_len) {
  if (dev == NULL || patch_data == NULL || patch_len == 0) {
    return TI_ERRC_INVALID_ARG;
  }

  size_t offset = 0;
  // Patch bytes are pushed through the TX FIFO in chunks.
  while (offset < patch_len) {
    size_t chunk_len = patch_len - offset;
    if (chunk_len > RADIO_PATCH_CHUNK_SIZE) {
      chunk_len = RADIO_PATCH_CHUNK_SIZE;
    }

    uint8_t tx[RADIO_PATCH_CHUNK_SIZE + 1] = {0};
    uint8_t rx[RADIO_PATCH_CHUNK_SIZE + 1] = {0};

    tx[0] = SI446X_CMD_WRITE_TX_FIFO;
    memcpy(&tx[1], &patch_data[offset], chunk_len);

    enum ti_errc_t errc = radio_spi_transfer(dev, tx, rx, chunk_len + 1);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }

    errc = radio_wait_cts(dev);
    if (errc != TI_ERRC_NONE) {
      return errc;
    }

    offset += chunk_len;
  }

  return TI_ERRC_NONE;
}

enum ti_errc_t radio_write_tx_fifo(radio_t *dev, const uint8_t *data,
                                  size_t len) {
  if (dev == NULL || data == NULL || len == 0 || len > RADIO_MAX_PACKET_SIZE) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t tx[RADIO_MAX_PACKET_SIZE + 1] = {0};
  uint8_t rx[RADIO_MAX_PACKET_SIZE + 1] = {0};

  tx[0] = SI446X_CMD_WRITE_TX_FIFO;
  memcpy(&tx[1], data, len);

  // WRITE_TX_FIFO is a fast-access command without CTS.
  return radio_spi_transfer(dev, tx, rx, len + 1);
}

enum ti_errc_t radio_read_rx_fifo(radio_t *dev, uint8_t *data, size_t len) {
  if (dev == NULL || data == NULL || len == 0 || len > RADIO_MAX_PACKET_SIZE) {
    return TI_ERRC_INVALID_ARG;
  }
  uint8_t tx[RADIO_MAX_PACKET_SIZE + 1] = {0};
  uint8_t rx[RADIO_MAX_PACKET_SIZE + 1] = {0};
  
  tx[0] = SI446X_CMD_READ_RX_FIFO; // 0x77
  
  // Single contiguous SPI transfer
  enum ti_errc_t errc = radio_spi_transfer(dev, tx, rx, len + 1);
  if (errc == TI_ERRC_NONE) {
    memcpy(data, &rx[1], len);
  }
  return errc;
}


enum ti_errc_t radio_start_tx(radio_t *dev, uint8_t channel, uint8_t condition,
                             uint16_t length, uint16_t tx_delay) {
  if (dev == NULL || length > RADIO_MAX_PACKET_SIZE) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t cmd[7] = {0};
  cmd[0] = SI446X_CMD_START_TX;
  cmd[1] = channel;
  cmd[2] = condition;
  cmd[3] = (uint8_t)(length >> 8);
  cmd[4] = (uint8_t)(length & 0xFF);
  cmd[5] = (uint8_t)(tx_delay >> 8);
  cmd[6] = (uint8_t)(tx_delay & 0xFF);

  enum ti_errc_t errc = radio_send_cmd(dev, cmd, sizeof(cmd));
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  return radio_wait_cts(dev);
}

enum ti_errc_t radio_start_rx(radio_t *dev, uint8_t channel,
                             const uint8_t *args, size_t args_len) {
  if (dev == NULL || (args_len > 0 && args == NULL)) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t tx[RADIO_CMD_BUFFER_MAX] = {0};
  uint8_t rx[RADIO_CMD_BUFFER_MAX] = {0};
  if (args_len + 2 > RADIO_CMD_BUFFER_MAX) {
    return TI_ERRC_INVALID_ARG;
  }

  // START_RX can include optional arguments when provided.
  tx[0] = SI446X_CMD_START_RX;
  tx[1] = channel;
  if (args_len > 0) {
    memcpy(&tx[2], args, args_len);
  }

  enum ti_errc_t errc = radio_spi_transfer(dev, tx, rx, args_len + 2);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  return radio_wait_cts(dev);
}

enum ti_errc_t radio_change_state(radio_t *dev, uint8_t next_state) {
  if (dev == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t cmd[2] = {SI446X_CMD_CHANGE_STATE, next_state};
  enum ti_errc_t errc = radio_send_cmd(dev, cmd, sizeof(cmd));
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  return radio_wait_cts(dev);
}

enum ti_errc_t radio_read_frr(radio_t *dev, uint8_t frr_index,
                             uint8_t *value) {
  if (dev == NULL || value == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  if (frr_index > 3) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t cmd = (uint8_t)(SI446X_CMD_READ_FRR_A + frr_index);
  uint8_t tx[2] = {cmd, 0x00};
  uint8_t rx[2] = {0};

  enum ti_errc_t errc = radio_spi_transfer(dev, tx, rx, sizeof(tx));
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  *value = rx[1];
  return TI_ERRC_NONE;
}

enum ti_errc_t radio_get_int_status(radio_t *dev, uint8_t *ph_status,
                                   uint8_t *modem_status,
                                   uint8_t *chip_status) {
  if (dev == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t cmd[4] = {SI446X_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00};
  uint8_t resp[7] = {0};

  enum ti_errc_t errc = radio_send_cmd(dev, cmd, sizeof(cmd));
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  errc = radio_wait_cts(dev);
  if (errc != TI_ERRC_NONE) {
    return errc;
  }

  uint8_t cts = 0;
  errc = radio_read_cmd_buf(dev, &cts, resp, sizeof(resp));
  if (errc != TI_ERRC_NONE) {
    return errc;
  }
  if (cts != SI446X_CTS_READY_VALUE) {
    return TI_ERRC_RADIO_CTS_TIMEOUT;
  }

  if (chip_status != NULL) {
    *chip_status = resp[4];
  }
  if (modem_status != NULL) {
    *modem_status = resp[5];
  }
  if (ph_status != NULL) {
    *ph_status = resp[6];
  }

  return TI_ERRC_NONE;
}

enum ti_errc_t radio_fifo_info(radio_t *dev, uint8_t arg, uint8_t *resp,
                              size_t resp_len) {
  if (dev == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t cmd[2] = {SI446X_CMD_FIFO_INFO, arg};
  return radio_send_cmd_get_resp(dev, cmd, sizeof(cmd), resp, resp_len);
}

enum ti_errc_t radio_get_packet_info(radio_t *dev, uint8_t *resp,
                                    size_t resp_len) {
  if (dev == NULL) {
    return TI_ERRC_INVALID_ARG;
  }

  uint8_t cmd = SI446X_CMD_PACKET_INFO;
  return radio_send_cmd_get_resp(dev, &cmd, 1, resp, resp_len);
}

bool radio_nirq_asserted(radio_t *dev) {
  if (dev == NULL || dev->nirq_pin == 0) {
    return false;
  }

  return !tal_read_pin(dev->nirq_pin);
}
