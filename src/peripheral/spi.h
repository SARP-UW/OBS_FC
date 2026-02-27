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
 * @file peripheral/spi.h
 * @authors Jude Merritt
 * @brief Implementation of SPI driver interface
 */

#pragma once

#include <stdint.h>

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/

/**
 * @brief Initialize an SPI interface.
 *
 * Prepares the specified SPI instance for communication. This must be called
 * before attempting any SPI transfers.
 *
 * @param inst  Identifier of the SPI instance to initialize.
 *
 * @return 1 if initialization finishes and the imput parameters are valid.
 */
int spi_init(uint8_t inst);

/**
 * @brief Perform an SPI data transfer with blocking. 
 * 
 * SPI1: NSS -> PA4  | SCK -> PA5  | MISO -> PA6  | MOSI -> PA7  
 * 
 * SPI2: NSS -> PB12 | SCK -> PB13 | MISO -> PB14 | MOSI -> PB15 
 * 
 * SPI3: NSS -> PA15 | SCK -> PC10 | MISO -> PC11 | MOSI -> PC12 
 * 
 * SPI4: NSS -> PE4  | SCK -> PE2  | MISO -> PE5  | MOSI -> PE6  
 * 
 * SPI5: NSS -> PF6  | SCK -> PF7  | MISO -> PF8  | MOSI -> PF9  
 * 
 * SPI6: NSS -> PG8  | SCK -> PG12 | MISO -> PG13 | MOSI -> PG14  
 *
 * Sends data from the source buffer while simultaneously receiving data into
 * the destination buffer. The function does not return until the entire
 * transfer is complete.
 *
 * @param inst  SPI instance to use for the transfer.
 * @param src   Pointer to the transmit buffer.
 * @param dst   Pointer to the receive buffer.
 * @param size  Number of bytes to transfer.
 *
 * @return 1 if the transfer finishes and the imput parameters are valid.
 */
int spi_transfer_sync(uint8_t inst, void* src, void* dst, uint8_t size);