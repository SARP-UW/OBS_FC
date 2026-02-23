/**
 * This file is part of the Titan Project.
 * Copyright (c) 2024 UW SARP
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
 * @file peripheral/errc.h
 * @authors Aaron McBride
 * @brief Error code definitions and utility functions.
 */

#pragma once
#include <stddef.h>
#include <stdint.h>

/** @brief Error code enum. */
enum ti_errc_t {
  TI_ERRC_NONE = 0, /** @brief No error occurred. */
  TI_ERRC_UNKNOWN,  /** @brief Unknown error occurred. */
  TI_ERRC_INVALID_ARG, /** @brief Invalid argument passed to function. */
  TI_ERRC_SPI_MAX_DEV, /** @brief Reached max device count for spi instance*/
  TI_ERRC_SPI_NO_CONTEXT,  /** @brief Couldn't find context for spi device*/
  TI_ERRC_GNSS_CONFIG_FAIL, /** @brief Failed to configure GNSS peripheral*/
  TI_ERRC_DMA_NO_AVAIL_STREAM, /** @brief Failed to find available dma stream*/
  TI_ERRC_SPI_BLOCKING_TIMEOUT, /** @brief Blocking SPI transfer timed out */
  TI_ERRC_UBX_CHECKSUM, /** @brief Wrong ubx checksum */
  TI_ERRC_UBX_HEADER, /** @brief Wrong ubx header */
  TI_ERRC_UBX_CLASS_ID, /** @brief Wrong ubx class and id */
  TI_ERRC_UBX_SIZE, /** @brief Wrong ubx size */
  TI_ERRC_UBX_PAYLOAD, /** @brief Wrong ubx payload */
  TI_ERRC_RADIO_CTS_TIMEOUT, /** @brief Radio CTS timeout */
  TI_ERRC_MUTEX_LOCKED, /** @brief Failed to enable EXTI ISR because mutex is locked */
  TI_ERRC_MUTEX_UNLOCKED, /** @brief Failed to disable EXTI ISR because mutex is unlocked */
  TI_ERRC_MUTEX_TIMEOUT, /** @brief Failed to disable EXTI ISR because mutex timed out */
  TI_ERRC_SPI_NOT_LOCKED,
};

/**
 * @brief Gets the name of an error code.
 * @param errc_in (enum ti_errc_t) The target error code.
 * @param errc_out (enum ti_errc_t*) Output error code for this function.
 * @returns (const char*) String containing the name of @p [errc].
 */
const char* ti_get_errc_name(enum ti_errc_t errc_in, enum ti_errc_t* errc_out);

/**
 * @brief Gets the description of an error code.
 * @param errc_in (enum ti_errc_t) The target error code.
 * @param errc_out (enum ti_errc_t*) Output error code for this function.
 * @returns (const char*) String containing the description of @p [errc].
 */
const char* ti_get_errc_desc(enum ti_errc_t errc_in, enum ti_errc_t* errc_out);