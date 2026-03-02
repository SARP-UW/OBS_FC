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
 */

#include "radio.h"
#include "peripheral/errc.h"
#include "peripheral/spi.h"
#include "peripheral/gpio.h"
#include <string.h>

/**************************************************************************************************
 * @section Macros
 **************************************************************************************************/

/** @brief Default timeout iteration count for CTS polling. */
#define RADIO_DEFAULT_CTS_TIMEOUT  1000000U

/** @brief Default timeout iteration count for SPI transfers. */
#define RADIO_DEFAULT_SPI_TIMEOUT  1000000U

/** @brief Blocking delay loop count after reset assertion/de-assertion. */
#define RADIO_DEFAULT_RESET_DELAY  100000U

/** @brief Maximum packet payload size in bytes. */
#define RADIO_MAX_PACKET_SIZE      64U

/** @brief Internal command buffer size for SPI transactions. */
#define SI446X_CMD_BUFFER_SIZE     16U

/** @brief Si446x command: Start TX. */
#define SI446X_CMD_START_TX        0x31

/** @brief Si446x command: Start RX. */
#define SI446X_CMD_START_RX        0x32

/** @brief Si446x command: Change operating state. */
#define SI446X_CMD_CHANGE_STATE    0x34

/** @brief Si446x command: Read command buffer (CTS check). */
#define SI446X_CMD_READ_CMD_BUFF   0x44

/** @brief Si446x command: Write TX FIFO. */
#define SI446X_CMD_WRITE_TX_FIFO   0x66

/** @brief Si446x command: Read RX FIFO. */
#define SI446X_CMD_READ_RX_FIFO    0x77

/** @brief Si446x command: Get pending interrupt status. */
#define SI446X_CMD_GET_INT_STATUS  0x20

/** @brief Si446x command: FIFO info / reset. */
#define SI446X_CMD_FIFO_INFO       0x15

/** @brief Si446x command: Get received packet info. */
#define SI446X_CMD_PACKET_INFO     0x16

/** @brief Si446x state: Sleep. */
#define SI446X_STATE_SLEEP         0x01

/** @brief Si446x state: Ready. */
#define SI446X_STATE_READY         0x03

/** @brief CTS ready byte value returned by the Si446x. */
#define SI446X_CTS_READY_VALUE     0xFF

/**************************************************************************************************
 * @section Static Configuration Data
 **************************************************************************************************/

/** @brief Power-up command sequence (WDS-generated). */
static const uint8_t radio_power_up_cmd[] = { 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80 };
static const size_t  radio_power_up_len = sizeof(radio_power_up_cmd);

/** @brief Firmware patch data (populate from WDS export if needed). */
static const uint8_t radio_patch_data[] = { /* Add your patch array here */ };
static const size_t  radio_patch_len = 0;

/** @brief GPIO pin configuration command (WDS-generated). */
static const uint8_t radio_gpio_cfg_cmd[] = { 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const size_t  radio_gpio_cfg_len = sizeof(radio_gpio_cfg_cmd);

/** @brief Errata 12 workaround command. */
static const uint8_t radio_errata_12_cmd[] = { 0xF1, 0x47, 0x4B, 0x00 };

/** @brief Default arguments for START_RX command. */
static const uint8_t radio_rx_default_args[] = { 0x00, 0x00, 0x08, 0x08, 0x08 };

/**************************************************************************************************
 * @section Forward Declarations
 **************************************************************************************************/

static enum ti_errc_t radio_wait_cts(radio_t *dev);
static enum ti_errc_t radio_send_cmd(radio_t *dev, const uint8_t *cmd, size_t len);
static enum ti_errc_t radio_send_cmd_get_resp(radio_t *dev, const uint8_t *cmd, size_t cmd_len, uint8_t *resp, size_t resp_len);
static enum ti_errc_t radio_change_state(radio_t *dev, uint8_t next_state);
static enum ti_errc_t radio_write_tx_fifo(radio_t *dev, const uint8_t *data, size_t len);
static enum ti_errc_t radio_read_rx_fifo(radio_t *dev, uint8_t *data, size_t len);
static enum ti_errc_t radio_start_tx(radio_t *dev, uint8_t channel, uint8_t condition, uint16_t length, uint16_t tx_delay);
static enum ti_errc_t radio_start_rx(radio_t *dev, uint8_t channel, const uint8_t *args, size_t args_len);
static enum ti_errc_t radio_fifo_info(radio_t *dev, uint8_t arg, uint8_t *resp, size_t resp_len);
static enum ti_errc_t radio_get_packet_info(radio_t *dev, uint8_t *resp, size_t resp_len);

/**************************************************************************************************
 * @section Private Helper Functions
 **************************************************************************************************/

/**
 * @brief Polls the Si446x until CTS (Clear-To-Send) is asserted.
 *
 * @param dev  Pointer to the radio device handle.
 * @return TI_ERRC_NONE on success, TI_ERRC_RADIO_CTS_TIMEOUT on timeout.
 */
static enum ti_errc_t radio_wait_cts(radio_t *dev) {
    if (!dev) return TI_ERRC_INVALID_ARG;
    for (uint32_t i = 0; i < RADIO_DEFAULT_CTS_TIMEOUT; i++) {
        uint8_t tx[2] = { SI446X_CMD_READ_CMD_BUFF, 0x00 };
        uint8_t rx[2] = { 0x00, 0x00 };
        spi_transfer_sync(dev->config.spi_instance, dev->config.ss_pin, tx, rx, 2);
        if (rx[1] == SI446X_CTS_READY_VALUE) {
            return TI_ERRC_NONE;
        }
    }
    return TI_ERRC_RADIO_CTS_TIMEOUT;
}

/**
 * @brief Sends a command to the Si446x and waits for CTS.
 *
 * @param dev  Pointer to the radio device handle.
 * @param cmd  Pointer to the command byte array.
 * @param len  Number of command bytes.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_send_cmd(radio_t *dev, const uint8_t *cmd, size_t len) {
    if (!dev || !cmd || len == 0) return TI_ERRC_INVALID_ARG;
    uint8_t tx[SI446X_CMD_BUFFER_SIZE] = {0};
    uint8_t rx[SI446X_CMD_BUFFER_SIZE] = {0};
    memcpy(tx, cmd, len);
    
    if (spi_transfer_sync(dev->config.spi_instance, dev->config.ss_pin, tx, rx, len) != 1) {
        return TI_ERRC_UNKNOWN;
    }
    return radio_wait_cts(dev);
}

/**
 * @brief Sends a command and reads back a response from the command buffer.
 *
 * @param dev       Pointer to the radio device handle.
 * @param cmd       Pointer to the command byte array.
 * @param cmd_len   Number of command bytes.
 * @param resp      Pointer to the response buffer (may be NULL).
 * @param resp_len  Number of response bytes to read.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_send_cmd_get_resp(radio_t *dev, const uint8_t *cmd, size_t cmd_len, uint8_t *resp, size_t resp_len) {
    enum ti_errc_t err = radio_send_cmd(dev, cmd, cmd_len);
    if (err != TI_ERRC_NONE) return err;

    uint8_t tx[SI446X_CMD_BUFFER_SIZE] = {0};
    uint8_t rx[SI446X_CMD_BUFFER_SIZE] = {0};
    tx[0] = SI446X_CMD_READ_CMD_BUFF;
    
    /* Transfer size is resp_len + 2 to account for the command byte and CTS byte */
    if (spi_transfer_sync(dev->config.spi_instance, dev->config.ss_pin, tx, rx, resp_len + 2) != 1) {
        return TI_ERRC_UNKNOWN;
    }

    if (rx[1] == SI446X_CTS_READY_VALUE && resp != NULL) {
        memcpy(resp, &rx[2], resp_len);
    } else {
        return TI_ERRC_RADIO_CTS_TIMEOUT;
    }
    return TI_ERRC_NONE;
}

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/

enum ti_errc_t radio_init(radio_t *dev, const radio_config_t *config) {
    if (!dev || !config) return TI_ERRC_INVALID_ARG;
    dev->config = *config;
    dev->apply_errata_12 = true;

    uint8_t ss_list[1] = { dev->config.ss_pin };
    if (spi_init(dev->config.spi_instance, ss_list, 1) != 1) return TI_ERRC_INVALID_ARG;

    if (dev->config.reset_pin) {
        tal_enable_clock(dev->config.reset_pin);
        tal_set_mode(dev->config.reset_pin, 1);
    }
    if (dev->config.nirq_pin) {
        tal_enable_clock(dev->config.nirq_pin);
        tal_set_mode(dev->config.nirq_pin, 0);
        tal_pull_pin(dev->config.nirq_pin, 1);
    }

    enum ti_errc_t err = radio_reset(dev);
    if (err != TI_ERRC_NONE) return err;

    if (radio_power_up_len > 0) {
        err = radio_send_cmd(dev, radio_power_up_cmd, radio_power_up_len);
        if (err != TI_ERRC_NONE) return err;
    }

    if (dev->apply_errata_12) {
        radio_send_cmd(dev, radio_errata_12_cmd, sizeof(radio_errata_12_cmd));
        radio_change_state(dev, SI446X_STATE_SLEEP);
        radio_change_state(dev, SI446X_STATE_READY);
    }

    if (radio_gpio_cfg_len > 0) {
        radio_send_cmd(dev, radio_gpio_cfg_cmd, radio_gpio_cfg_len);
    }

    return TI_ERRC_NONE;
}

enum ti_errc_t radio_reset(radio_t *dev) {
    if (!dev) return TI_ERRC_INVALID_ARG;
    if (dev->config.reset_pin > 0) {
        uint8_t active = dev->config.reset_active_high ? 1 : 0;
        uint8_t inactive = dev->config.reset_active_high ? 0 : 1;
        
        tal_set_pin(dev->config.reset_pin, active);
        for(volatile int i=0; i<10000; i++); /* Simple blocking delay */
        tal_set_pin(dev->config.reset_pin, inactive);
        for(volatile int i=0; i<10000; i++);
    }
    return TI_ERRC_NONE;
}

enum ti_errc_t radio_transmit(radio_t *dev, const uint8_t *data, size_t len) {
    if (!dev || !data || len > RADIO_MAX_PACKET_SIZE) return TI_ERRC_INVALID_ARG;
    radio_fifo_info(dev, 0x01, NULL, 0); // Reset TX FIFO
    radio_write_tx_fifo(dev, data, len);
    radio_start_tx(dev, dev->config.channel, 0x03, len, 0);
    return TI_ERRC_NONE;
}

enum ti_errc_t radio_receive(radio_t *dev, uint8_t *data, size_t max_len, size_t *actual_len) {
    if (!dev || !data || !actual_len) return TI_ERRC_INVALID_ARG;
    uint8_t resp[2] = {0};
    radio_get_packet_info(dev, resp, 2);
    
    // Convert two response bytes into a 16-bit length
    uint16_t pkt_len = ((uint16_t)resp[0] << 8) | resp[1];
    
    if (pkt_len == 0) {
        *actual_len = 0;
        return TI_ERRC_NONE;
    }
    
    size_t read_len = (pkt_len > max_len) ? max_len : pkt_len;
    radio_read_rx_fifo(dev, data, read_len);
    *actual_len = read_len;
    
    radio_fifo_info(dev, 0x02, NULL, 0); // Clear RX FIFO
    radio_start_rx(dev, dev->config.channel, radio_rx_default_args, sizeof(radio_rx_default_args));
    return TI_ERRC_NONE;
}

enum ti_errc_t radio_get_int_status(radio_t *dev, uint8_t *ph_status, uint8_t *modem_status, uint8_t *chip_status) {
    if (!dev) return TI_ERRC_INVALID_ARG;
    uint8_t cmd[4] = { SI446X_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00 };
    uint8_t resp[8] = {0};
    
    enum ti_errc_t err = radio_send_cmd_get_resp(dev, cmd, 4, resp, 8);
    if (err == TI_ERRC_NONE) {
        if (ph_status) *ph_status = resp[3];       // PH_STATUS
        if (modem_status) *modem_status = resp[5]; // MODEM_STATUS
        if (chip_status) *chip_status = resp[7];   // CHIP_STATUS
    }
    return err;
}

bool radio_nirq_asserted(radio_t *dev) {
    if (!dev || dev->config.nirq_pin == 0) return false;
    return (tal_read_pin(dev->config.nirq_pin) == 0); /* Active-low */
}

/**************************************************************************************************
 * @section Private Si446x Command Helpers
 **************************************************************************************************/

/**
 * @brief Sends a CHANGE_STATE command to transition the Si446x operating state.
 *
 * @param dev         Pointer to the radio device handle.
 * @param next_state  Target Si446x state (e.g. SI446X_STATE_READY).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_change_state(radio_t *dev, uint8_t next_state) {
    uint8_t cmd[] = { SI446X_CMD_CHANGE_STATE, next_state };
    return radio_send_cmd(dev, cmd, 2);
}

/**
 * @brief Writes data into the Si446x TX FIFO.
 *
 * @param dev   Pointer to the radio device handle.
 * @param data  Pointer to the payload bytes.
 * @param len   Number of bytes to write.
 * @return TI_ERRC_NONE on success.
 */
static enum ti_errc_t radio_write_tx_fifo(radio_t *dev, const uint8_t *data, size_t len) {
    uint8_t tx[RADIO_MAX_PACKET_SIZE + 1];
    uint8_t rx[RADIO_MAX_PACKET_SIZE + 1] = {0};
    tx[0] = SI446X_CMD_WRITE_TX_FIFO;
    memcpy(&tx[1], data, len);
    spi_transfer_sync(dev->config.spi_instance, dev->config.ss_pin, tx, rx, len + 1);
    return TI_ERRC_NONE;
}

/**
 * @brief Reads data from the Si446x RX FIFO.
 *
 * @param dev   Pointer to the radio device handle.
 * @param data  Destination buffer for the received bytes.
 * @param len   Number of bytes to read.
 * @return TI_ERRC_NONE on success.
 */
static enum ti_errc_t radio_read_rx_fifo(radio_t *dev, uint8_t *data, size_t len) {
    uint8_t tx[RADIO_MAX_PACKET_SIZE + 1] = { SI446X_CMD_READ_RX_FIFO };
    uint8_t rx[RADIO_MAX_PACKET_SIZE + 1] = {0};
    spi_transfer_sync(dev->config.spi_instance, dev->config.ss_pin, tx, rx, len + 1);
    memcpy(data, &rx[1], len);
    return TI_ERRC_NONE;
}

/**
 * @brief Starts a transmission on the specified RF channel.
 *
 * @param dev        Pointer to the radio device handle.
 * @param channel    RF channel number.
 * @param condition  TX condition byte (e.g. 0x03 for immediate start).
 * @param length     Packet length in bytes.
 * @param tx_delay   Delay before transmission (in radio clock ticks).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_start_tx(radio_t *dev, uint8_t channel, uint8_t condition, uint16_t length, uint16_t tx_delay) {
    uint8_t cmd[] = { SI446X_CMD_START_TX, channel, condition, (uint8_t)(length >> 8), (uint8_t)length, (uint8_t)(tx_delay >> 8), (uint8_t)tx_delay };
    return radio_send_cmd(dev, cmd, 7);
}

/**
 * @brief Starts reception on the specified RF channel.
 *
 * @param dev       Pointer to the radio device handle.
 * @param channel   RF channel number.
 * @param args      Pointer to additional START_RX arguments.
 * @param args_len  Number of additional argument bytes (max 6).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_start_rx(radio_t *dev, uint8_t channel, const uint8_t *args, size_t args_len) {
    uint8_t cmd[8] = { SI446X_CMD_START_RX, channel };
    if (args && args_len <= 6) memcpy(&cmd[2], args, args_len);
    return radio_send_cmd(dev, cmd, 2 + args_len);
}

/**
 * @brief Queries or resets the Si446x FIFO state.
 *
 * @param dev       Pointer to the radio device handle.
 * @param arg       FIFO info argument (0x01 = reset TX, 0x02 = reset RX).
 * @param resp      Pointer to the response buffer (may be NULL).
 * @param resp_len  Number of response bytes to read.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_fifo_info(radio_t *dev, uint8_t arg, uint8_t *resp, size_t resp_len) {
    uint8_t cmd[] = { SI446X_CMD_FIFO_INFO, arg };
    return radio_send_cmd_get_resp(dev, cmd, 2, resp, resp_len);
}

/**
 * @brief Retrieves information about the last received packet.
 *
 * @param dev       Pointer to the radio device handle.
 * @param resp      Pointer to the response buffer.
 * @param resp_len  Number of response bytes to read.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
static enum ti_errc_t radio_get_packet_info(radio_t *dev, uint8_t *resp, size_t resp_len) {
    uint8_t cmd[] = { SI446X_CMD_PACKET_INFO, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return radio_send_cmd_get_resp(dev, cmd, 6, resp, resp_len);
}
