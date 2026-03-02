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
 * @param ss_list  An array of SS (slave select) pin numbers. Each SS pin represents one slave device. 
 * @param slave_count  The number of slave devices attached to the instance. This number must match the
 *                     number of elements in the ss_list array. 
 * 
 * @return 1 if initialization finishes and the imput parameters are valid and -1 otherwise. 
 */
int spi_init(uint8_t inst, uint8_t* ss_list, uint8_t slave_count);

/**
 * @brief Perform an SPI data transfer with blocking. 
 * 
 * SPI1: SCK -> PA5  | MISO -> PA6  | MOSI -> PA7  
 * 
 * SPI2: SCK -> PB13 | MISO -> PB14 | MOSI -> PB15 
 * 
 * SPI3: SCK -> PC10 | MISO -> PC11 | MOSI -> PC12 
 * 
 * SPI4: SCK -> PE2  | MISO -> PE5  | MOSI -> PE6  
 * 
 * SPI5: SCK -> PF7  | MISO -> PF8  | MOSI -> PF9  
 * 
 * SPI6: SCK -> PG12 | MISO -> PG13 | MOSI -> PG14  
 *
 * Sends data from the source buffer while simultaneously receiving data into
 * the destination buffer. The function does not return until the entire
 * transfer is complete.
 *
 * @param inst  SPI instance to use for the transfer.
 * @param src   Pointer to the transmit (source) buffer.
 * @param dst   Pointer to the receive (destination) buffer.
 * @param size  Number of bytes to transfer.
 * @param ss_pin  The SS pin of the slave SPI will communicate with. 
 *
 * @return 1 if the transfer finishes and the imput parameters are valid and -1 otherwise. 
 */
int spi_transfer_sync(uint8_t inst, uint8_t ss_pin, void* src, void* dst, uint8_t size);