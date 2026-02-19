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
 * @file src/app/dma.h
 * @authors Charles Faisandier
 * @brief DMA Public Interface.
 */
#pragma once

#include <stdint.h>
#include "../util/error.h"

/**************************************************************************************************
 * @section Type Definitions
 **************************************************************************************************/
// Enum to identify the specific DMA controller instance
#define DMA_INSTANCE_MIN 1
typedef enum {
    DMA1 = DMA_INSTANCE_MIN,
    DMA2,
    DMA_INSTANCE_COUNT
} dma_instance_t;

// Enum to identify a specific DMA stream/channel within an instance
// For DMA1/2, typically 0-7. For MDMA/BDMA, might be single or fewer channels.
#define DMA_STREAM_MIN 0
typedef enum {
    DMA_STREAM_0 = DMA_STREAM_MIN,
    DMA_STREAM_1,
    DMA_STREAM_2,
    DMA_STREAM_3,
    DMA_STREAM_4,
    DMA_STREAM_5,
    DMA_STREAM_6,
    DMA_STREAM_7,
    DMA_STREAM_COUNT 
} dma_stream_t;

// Enum for transfer direction
#define DMA_DIRECTION_MIN 0
typedef enum {
    PERIPH_TO_MEM = DMA_DIRECTION_MIN,
    MEM_TO_PERIPH,
    DMA_DIR_COUNT
} dma_direction_t;

// Enum for data transfer size (per item)
#define DMA_DATA_SIZE_MIN 0
typedef enum {
    DMA_DATA_SIZE_BYTE = DMA_DATA_SIZE_MIN,
    DMA_DATA_SIZE_HALFWORD,
    DMA_DATA_SIZE_WORD,    
    DMA_DATA_SIZE_COUNT
} dma_data_size_t;

// Enum for DMA priority
#define DMA_PRIORITY_MIN 0
typedef enum {
    DMA_PRIORITY_LOW = DMA_PRIORITY_MIN,
    DMA_PRIORITY_MEDIUM,
    DMA_PRIORITY_HIGH,
    DMA_PRIORITY_VERY_HIGH,
    DMA_PRIORITY_COUNT
} dma_priority_t;

#define DMA_FIFO_THRESHOLD_MIN 0
typedef enum {
    DMA_FIFO_THRESHOLD_FULL = DMA_FIFO_THRESHOLD_MIN, 
    DMA_FIFO_THRESHOLD_THREEQUARTERS, 
    DMA_FIFO_THRESHOLD_HALF, 
    DMA_FIFO_THRESHOLD_QUARTER, 
    DMA_FIFO_THRESHOLD_COUNT
} dma_fifo_threshold_t;

// Callback function type for DMA events
typedef void (*dma_callback_t)(bool success, void *context);

// Configuration structure for a DMA stream
typedef struct {
    dma_instance_t   instance;      // DMA1, DMA2, MDMA
    dma_stream_t     stream;        // Specific stream/channel (0-7 for DMA1/2)
    uint32_t         request_id;    // DMAMUX request ID for peripheral (if applicable, e.g., DMA_REQUEST_USART1_TX)
    dma_direction_t  direction;
    dma_data_size_t  src_data_size; // Source data width
    dma_data_size_t  dest_data_size; // Destination data width
    dma_priority_t   priority;
    bool             fifo_enabled;   // Generally disabled for sending instructions to peripherals,
                                        // but enabled for high-throughput transfers
    dma_fifo_threshold_t fifo_threshold; // FIFO threshold for DMA1/2 (e.g., DMA_FIFO_THRESHOLD_FULL)
    // Callback for this stream
    dma_callback_t   callback;
} dma_config_t;

/**
 * @brief Peripheral DMA Config structure.
 *
 * Exact copy of spi_dma_config_t, but w/o the request_id field.
 * The peripheral driver should be able to figure out the request_id.
 */
typedef struct {
    dma_instance_t   instance;      // DMA1, DMA2, MDMA, BDMA
    dma_stream_t     stream;        // Specific stream/channel (0-7 for DMA1/2)
    dma_direction_t  direction;
    dma_data_size_t  src_data_size; // Source data width
    dma_data_size_t  dest_data_size; // Destination data width
    bool             src_inc_enabled; // Source address increment
    bool             dest_inc_enabled; // Destination address increment
    dma_priority_t   priority;
    bool             fifo_enabled;
    uint32_t         fifo_threshold;  // FIFO threshold for RX stream. Currently left defined by
                                      // the caller, might become internal depending on usecase.
} periph_dma_config_t;

/**
 * @brief DMA transfer config
 * 
 * Specifies all necessary parameters for dma transfer.
 */
typedef struct {
    dma_instance_t instance;
    dma_stream_t stream;
    const void* src;
    void* dest;
    size_t size;
    void *context;
    bool disable_mem_inc; // Useful for dummy spi transactions
} dma_transfer_t;

// Used to track rx/tx stream/instance for peripheral instances
typedef struct {
    dma_instance_t rx_instance; // TODO: Perhaps simplify this so only one instance
    dma_instance_t tx_instance;
    dma_stream_t rx_stream;
    dma_stream_t tx_stream;
} dma_periph_streaminfo_t;

/**************************************************************************************************
* @section Public Functions
**************************************************************************************************/
/**
 * @brief Initializes the DMA subsystem (enables clocks for all DMA controllers).
 * Should be called once during system boot.
 */
tal_err_t *dma_init(void);

/**
 * @brief Configures a specific DMA stream to a specific request ID, and enables it.
 * This allocates and sets up the chosen stream based on the provided configuration.
 * @param config Pointer to the configuration structure.
 * @return true if the stream was successfully configured, false otherwise.
 */
bool dma_configure_stream(const dma_config_t* config);

/**
 * @brief Starts a DMA transfer for the specified stream.
 * This function initiates the transfer based on the previously configured settings.
 * @param instance The DMA instance (DMA1, DMA2, MDMA, BDMA).
 * @param stream The specific stream to start.
 * @param src Pointer to the source data buffer.
 * @param dest Pointer to the destination data buffer.
 * @param size Number of bytes to transfer.
 * @return bool, whether the transfer was successfully started.
 */
bool dma_start_transfer( dma_transfer_t *dma_transfer);

/**
 * @brief Checks the validity of a DMA peripheral config
 * @param config The config to check.
 * @return bool Whether the config is valid.
 */
inline static bool check_periph_dma_config_validity(periph_dma_config_t *dma_config);