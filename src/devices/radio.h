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
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "peripheral/errc.h"

/**************************************************************************************************
 * @section Type Definitions
 **************************************************************************************************/

/** @brief SPI config for radio. */
typedef struct {
  uint8_t spi_inst;          // SPI peripheral instance (1=SPI1, 2=SPI2, etc.)
  uint8_t ss_pin;            // Slave Select GPIO pin — directly from schematic.
} radio_spi_t;

/** @brief Radio hardware and channel configuration — everything except SPI bus identity. */
typedef struct {
  uint8_t reset_pin;         // Reset GPIO pin. Directly from Si4468 SDN pin.
  uint8_t nirq_pin;          // nIRQ (interrupt request) GPIO pin, active-low.
                             //   Goes low when the radio has an event (TX done, RX packet, fault).
  bool reset_active_high;    // True if reset pin is active-high (depends on PCB design).
  uint8_t channel;           // Default RF channel number for TX/RX operations.
} radio_config_t;

/** @brief Radio device handle. Allocate one and pass to all radio_* functions. */
typedef struct {
  radio_spi_t spi_config;    // SPI bus + chip select pin for this device.
  radio_config_t config;     // Active device configuration.
  bool apply_errata_12;      // True if Si446x errata 12 workaround should be applied.
                             //   Set automatically during radio_init(). Fixes a known PLL lock bug.
} radio_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initializes the Si4468 radio and applies power-up configuration.
 *
 * Configures SPI and GPIO pins, performs a hardware reset, sends the power-up
 * command sequence, and optionally applies the errata 12 workaround.
 *
 * @param dev        Pointer to the radio device handle.
 * @param spi_config Pointer to the SPI configuration (instance + SS pin).
 * @param config     Pointer to the radio configuration (reset/nirq pins, channel).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t radio_init(radio_t *dev, const radio_spi_t *spi_config, const radio_config_t *config);

/**
 * @brief Performs a hardware reset of the Si4468.
 *
 * @param dev  Pointer to the radio device handle.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t radio_reset(radio_t *dev);

/**
 * @brief Transmits a packet over the configured RF channel.
 *
 * Resets the TX FIFO, writes the data, and starts transmission.
 *
 * @param dev   Pointer to the radio device handle.
 * @param data  Pointer to the transmit data buffer.
 * @param len   Number of bytes to transmit (max 64).
 * @return TI_ERRC_NONE on success, TI_ERRC_INVALID_ARG if len exceeds the
 *         maximum packet size.
 */
enum ti_errc_t radio_transmit(radio_t *dev, const uint8_t *data, size_t len);

/**
 * @brief Receives a packet from the RX FIFO.
 *
 * Reads the received packet length, copies up to max_len bytes into the
 * destination buffer, clears the RX FIFO, and restarts reception.
 *
 * @param dev         Pointer to the radio device handle.
 * @param data        Pointer to the receive data buffer.
 * @param max_len     Maximum number of bytes to receive.
 * @param actual_len  Pointer to store the actual number of bytes received.
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t radio_receive(radio_t *dev, uint8_t *data, size_t max_len, size_t *actual_len);

/**
 * @brief Reads the pending interrupt status from the Si4468.
 *
 * @param dev            Pointer to the radio device handle.
 * @param ph_status      Pointer to store the packet-handler interrupt status (may be NULL).
 * @param modem_status   Pointer to store the modem interrupt status (may be NULL).
 * @param chip_status    Pointer to store the chip interrupt status (may be NULL).
 * @return TI_ERRC_NONE on success, or an appropriate error code on failure.
 */
enum ti_errc_t radio_get_int_status(radio_t *dev, uint8_t *ph_status, uint8_t *modem_status, uint8_t *chip_status);

/**
 * @brief Checks whether the nIRQ pin is asserted (active-low).
 *
 * @param dev  Pointer to the radio device handle.
 * @return True if nIRQ is asserted (low), false otherwise.
 */
bool radio_nirq_asserted(radio_t *dev);
