// Written by Jude Merritt

#pragma once

#include <stdint.h>

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
 * SPI1: NSS -> A4  | SCK -> A5  | MISO -> A6  | MOSI -> A7  
 * 
 * SPI2: NSS -> B12 | SCK -> B13 | MISO -> B14 | MOSI -> B15 
 * 
 * SPI3: NSS -> A15 | SCK -> C10 | MISO -> C11 | MOSI -> C12 
 * 
 * SPI4: NSS -> E4  | SCK -> E2  | MISO -> E5  | MOSI -> E6  
 * 
 * SPI5: NSS -> F6  | SCK -> F7  | MISO -> F8  | MOSI -> F9  
 * 
 * SPI6: NSS -> G8  | SCK -> G12 | MISO -> G13 | MOSI -> G14  
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