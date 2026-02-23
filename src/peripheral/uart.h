/**
 * This file is part of the Titan Flight Computer Project
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
 * @file peripheral/uart.h
 * @authors Joanna Zhou, Charles Faisandier, Jude Merritt
 * @brief UART Driver Interface. Doesn't support synchronous mode.
 */
#pragma once
#include "../internal/dma.h"
#include "../internal/mmio.h"
#include "../util/error.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**************************************************************************************************
 * @section Data Structures
 **************************************************************************************************/
static const rw_reg32_t **UARTx_CRx;

/**************************************************************************************************
 * @section Type Definitions
 **************************************************************************************************/
typedef enum {
  UART1 = 1,
  UART2,
  UART3,
  UART4,
  UART5,
  UART6,
  UART7,
  UART8,
  UART_CHANNEL_COUNT,
} uart_channel_t;

typedef enum {
  UART_PARITY_DISABLED,
  UART_PARITY_EVEN,
  UART_PARITY_ODD,
} uart_parity_t;

typedef enum {
  UART_DATALENGTH_7,
  UART_DATALENGTH_8,
  UART_DATALENGTH_9,
} uart_datalength_t;

typedef struct {
  uart_channel_t channel;
  uart_parity_t parity;
  uart_datalength_t data_length;
  uint32_t clk_freq;
  uint32_t baud_rate;
} uart_config_t;

typedef struct {
  bool *busy;
  uart_channel_t channel;
} uart_context_t;

/**************************************************************************************************
 * @section Function Definitions
 **************************************************************************************************/
/**
 * @brief Initializes the specified UART channel.
 *
 * @param flag: Error flag
 * @param usart_config: Config struct
 * @param dma_tx: TX DMA stream config
 * @param dma_rx: RX DMA stream config
 * @return true if initialization is successful, false otherwise.
 */
bool uart_init(uart_config_t *usart_config, dma_callback_t *callback,
               periph_dma_config_t *tx_stream, periph_dma_config_t *rx_stream);

/**
 * @brief Sends data over the specified UART channel. Asyncronous function.
 *
 * @param channel USART channel
 * @param tx_buff Pointer to the data buffer to be transmitted.
 * @param size Number of bytes to transmit.
 *
 * @return true if data transmission is successful, false otherwise.
 */
bool uart_write_async(uart_channel_t channel, uint8_t *tx_buff, uint32_t size);

/**
 * @brief Receives data from the specified UART channel. Asyncronous function
 *
 * @param channel USART channel
 * @param rx_buff Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to read.
 */
bool uart_read_async(uart_channel_t channel, uint8_t *rx_buff, uint32_t size);

/**
 * @brief Sends data over the specified UART channel. Blocking (syncronous)
 * function.
 *
 * @param channel USART channel
 * @param tx_buff Pointer to the data buffer to be transmitted.
 * @param size Number of bytes to transmit.
 *
 * @return true if data transmission is successful, false otherwise.
 */
bool uart_write_blocking(uart_channel_t channel, uint8_t *tx_buff,
                         uint32_t size);

/**
 * @brief Receives data from the specified UART channel. Blocking (syncronous)
 * function.
 *
 * @param channel USART channel
 * @param rx_buff Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to read.
 */
bool uart_read_blocking(uart_channel_t channel, uint8_t *rx_buff,
                        uint32_t size);

static inline bool verify_transfer_parameters(uart_channel_t channel, uint8_t *buff,
                                       size_t size);