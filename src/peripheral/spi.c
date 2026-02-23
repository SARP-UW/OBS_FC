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
 * @file peripheral/spi.c
 * @authors Charles Faisandier, Jude Merritt
 * @brief Implementation of SPI driver interface
 */
#include "spi.h"
#include "internal/mmio.h"
#include "gpio.h"
#include <stdint.h>
// #include "mutex.h"
#include "errc.h"
#include "internal/dma.h"

#define DATA_REG_SIZE 32
#define MAX_DEVICES_PER_INSTANCE 5
#define MAX_PRESCALER 256

/**************************************************************************************************
 * @section Internal Data Structures
 **************************************************************************************************/
// Used to look up SPI DMAMUX request numbers
// Note: SPI6 Does not seem to have requests
// Index 0 is RX index 1 is TX
const static uint8_t spi_dmamux_req[SPI_INSTANCE_COUNT][2] = {
    [1] = {
        [0] = 37,
        [1] = 38,
    },
    [2] = {
        [0] = 39,
        [1] = 40,
    },
    [3] = {
        [0] = 61,
        [1] = 62,
    },
    [4] = {
        [0] = 83,
        [1] = 84,
    },
    [5] = {
        [0] = 85,
        [1] = 86,
    }
};

// Stores SPI contexts
static spi_context_t spi_context_arr[SPI_INSTANCE_COUNT + 1][MAX_DEVICES_PER_INSTANCE] = {0};

// Store config
static spi_config_t configs[SPI_INSTANCE_COUNT + 1] = {0};

// Mutexes/
// struct ti_mutex_t mutex[SPI_INSTANCE_COUNT + 1];

// Timeouts
// uint32_t mutex_timeouts[SPI_INSTANCE_COUNT + 1];

/**************************************************************************************************
 * @section Private Function Implementations
 **************************************************************************************************/
inline static bool check_spi_config_validity(spi_config_t *spi_config) {
    if (spi_config == NULL) {
        return false;
    }
    if (spi_config->mode < 0 || spi_config->mode > 3) {
        return false;
    }
    if (spi_config->data_size != 8 && spi_config->data_size != 16) {
        return false;
    }
    if (spi_config->baudrate_prescaler < 2 ||
        spi_config->baudrate_prescaler > MAX_PRESCALER ||
        spi_config->baudrate_prescaler & (spi_config->baudrate_prescaler - 1)) {
        return false;
    }
    if (spi_config->first_bit < 0 || spi_config->first_bit > 1) {
        return false;
    }
    return true;
}

static void spi_dma_callback(bool success, spi_context_t *context) {
    // De-init spi dma transaction if both streams are finished.
    // spi_callback_t callback = context->callback;
    // uint8_t instance = context->device.instance;
    // if (success) {
    //     context->num_complete++;
    // } else {
    //     spi_unblock(context->device);
    //     callback(false);
    // }
    // if (context->num_complete == 2) {
    //     // Disable DMA requests
    //     CLR_FIELD(SPIx_CFG1[instance], SPIx_CFG1_RXDMAEN);
    //     CLR_FIELD(SPIx_CFG1[instance], SPIx_CFG1_TXDMAEN);
    //     context->num_complete = 0;
    //     spi_unblock(context->device);
    //     callback(true);
    // }
}

static bool check_device_valid(spi_device_t device) {
    if (device.instance > SPI_INSTANCE_COUNT)
        return false;
    if (device.gpio_pin == 0 || device.gpio_pin > 140)
        // TODO: Implement better way to check if gpio pin is valid
        return false;
    return true;
}

/**************************************************************************************************
 * @section Public Function Implementations
 **************************************************************************************************/
/**
 * @brief Initializes an SPI controller. Configures a TX and RX DMA stream for the SPI peripheral. 
 * It's important to choose SPI parameters that are compatible with all devices that will share the
 * controller.
 * 
 * @param flag Flag pointer for error heandling
 * @param spi_config Point to spi_config structure
 * @param tx_stream DMA configuration for TX stream
 * @param rx_stream DMA configuration for RX stream
 */
int spi_init(uint8_t instance, spi_config_t *spi_config) {
    // Parameter checking
    if (instance > SPI_INSTANCE_COUNT)
        return TI_ERRC_INVALID_ARG;
    if (!check_spi_config_validity(spi_config))
        return TI_ERRC_INVALID_ARG;
    
    // Save the spi_config
    configs[instance] = *spi_config;

    // Enable gpio clocks for miso mosi and clk
    tal_enable_clock(spi_config->miso_pin);
    tal_enable_clock(spi_config->mosi_pin);
    tal_enable_clock(spi_config->clk_pin);

    // Set gpio drain push-pull
    tal_set_drain(spi_config->miso_pin, 0);
    tal_set_drain(spi_config->mosi_pin, 0);
    tal_set_drain(spi_config->clk_pin, 0);

    // Set alternate function mode
    tal_set_mode(spi_config->miso_pin, 2);
    tal_set_mode(spi_config->mosi_pin, 2);
    tal_set_mode(spi_config->clk_pin, 2);

    // Create mutexes
    // ti_create_mutex(&mutex[instance]);

    // Set alternate function
    // TODO: Set alternate function
    tal_alternate_mode(spi_config->clk_pin, 6);
    tal_alternate_mode(spi_config->mosi_pin, 6);
    tal_alternate_mode(spi_config->miso_pin, 6);

    // Set gpio speed
    tal_set_speed(spi_config->miso_pin, 3);
    tal_set_speed(spi_config->mosi_pin, 3);
    tal_set_speed(spi_config->clk_pin, 2);

    // Enable SPI Peripheral Clock
    switch (instance) {
        case (1):
            // SET_FIELD(RCC_APB2ENR, RCC_APB2ENR_SPI[1]); // TODO: Can't find this field
            break;
        case (2):
            SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_SPIxEN[2]);
            break;
        case (3):
            SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_SPIxEN[3]);
            break;
        case (4):
            // SET_FIELD(RCC_APB2ENR, RCC_APB2ENR_SPI4EN); // TODO: Can't find this field
            break;
        case (5):
            // SET_FIELD(RCC_APB2ENR, RCC_ABP2ENR_SPI5EN); // TODO: Can't find this field
            break;
        case (6):
            // SET_FIELD(RCC_ABP4ENR, RCC_ABP4ENR_SPI6EN); // TODO: Can't find this field
            break;
    }

    // Configure SPI Mode
    switch (spi_config->mode) {
        case (0):
            CLR_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPOL);
            CLR_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPHA);
            break;
        case (1):
            CLR_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPOL);
            SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPHA);
            break;
        case (2):
            SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPOL);
            CLR_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPHA);
            break;
        case (3):
            SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPOL);
            SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_CPHA);
            break;
    }

    // Configure Baude Rate Prescaler
    switch (spi_config->baudrate_prescaler) {
        case (256):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b111);
            break;
        case (128):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b110);
            break;
        case (64):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b101);
            break;
        case (32):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b100);
            break;
        case (16):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b011);
            break;
        case (8):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b010);
            break;
        case (4):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b001);
            break;
        case (2):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_MBR, 0b000);
            break;
    }

    // Set the Data Frame Format
    switch (spi_config->data_size) {
        case (16):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_DSIZE, 0b1111);
            break;
        case (8):
            WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_DSIZE, 0b111);
            break;
    }

    // Configure First Bit Transmitted
    switch (spi_config->first_bit) {
        case (0):
            SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_LSBFRST);
            break;
        case (1):
            CLR_FIELD(SPIx_CFG2[instance], SPIx_CFG2_LSBFRST);
            break;
    }

    // Configure as master
    SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_MASTER);

    // Conffigure SPI software-NSS
    CLR_FIELD(SPIx_CFG2[instance], SPIx_CFG2_SSOE);
    SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_SSM);

    // Maybe not needed
    WRITE_FIELD(SPIx_CFG1[instance], SPIx_CFG1_FTHVL, 0x0);
    SET_FIELD(SPIx_CFG2[instance], SPIx_CFG2_AFCNTR);

    // Enable the SPI
    SET_FIELD(SPIx_CR1[instance], SPIx_CR1_SPE);

    return TI_ERRC_NONE;
}

int spi_device_init(spi_device_t device) {
    // TODO: Implement this function
    if (!check_device_valid(device))
        return TI_ERRC_INVALID_ARG;

    // De-reference struct for readablity
    uint8_t instance = device.instance;
    uint8_t gpio_pin = device.gpio_pin;

    // Enable GPIO port clock
    tal_enable_clock(gpio_pin);

    // Configure pin mode as output
    tal_set_mode(gpio_pin, 1);

    // Set up device context
    bool found = false;
    for (int i = 0; i < MAX_DEVICES_PER_INSTANCE; i++) {
        if (spi_context_arr[instance][i].device.gpio_pin == 0) {
            spi_context_arr[instance][i].device = device;
            found = true;
            break;
        }
    }
    if (!found)
        return TI_ERRC_SPI_MAX_DEV;

    // Set initial state
    tal_pull_pin(gpio_pin, 1);
    tal_set_pin(gpio_pin, 1);

    return TI_ERRC_NONE;
}

bool spi_is_blocked(spi_device_t device) {
    // True when mutex is locked and pin is low.
    return !tal_read_pin(device.gpio_pin);
    // return (ti_is_mutex_locked(mutex[device.instance]) && !tal_read_pin(device.gpio_pin));
}

int spi_transfer_sync(struct spi_sync_transfer_t *transfer) {
    spi_device_t device = transfer->device;
    void *source = transfer->source;
    void *dest = transfer->dest;
    size_t size = transfer->size;
    uint32_t timeout = transfer->timeout;
    bool read_inc = transfer->read_inc;
    asm("BKPT #0");

    // Perform blocking transfer
    for (int i = 0; i < size; i++) {
        while (!READ_FIELD(SPIx_SR[device.instance], SPIx_SR_TXP));
        *SPIx_TXDR[device.instance] = ((uint8_t *)source)[i];
        SET_FIELD(SPIx_CR1[device.instance], SPIx_CR1_CSTART);
        ((uint8_t *)dest)[i] = *SPIx_TXDR[device.instance];
    }

    return TI_ERRC_NONE;
}

 int spi_transfer_async(struct spi_async_transfer_t *transfer) {
    return 0;
 }
//     spi_device_t device = transfer->device;
//     void *source = transfer->source;
//     void *dest = transfer->dest;
//     size_t size = transfer->size;
//     spi_callback_t callback = transfer->callback;
//     bool write_fifo = transfer->write_fifo;
//     bool read_fifo = transfer->read_fifo;
//     bool write_mem_inc = transfer->write_mem_inc;
//     bool read_mem_inc = transfer->read_mem_inc;

//     // If spi transaction isn't locked, exit w/ error.
//     if (!spi_is_blocked(device)) {
//         return TI_ERRC_SPI_NOT_LOCKED;
//     }

//     // Get context
//     spi_context_t *context = NULL;
//     for (int i = 0; i < MAX_DEVICES_PER_INSTANCE; i++) {
//         if (spi_context_arr[device.instance][i].device.instance == device.instance &&
//             spi_context_arr[device.instance][i].device.gpio_pin == device.gpio_pin) {
//             context = &(spi_context_arr[device.instance][i]);
//         }
//     }
//     if (context == NULL)
//         return TI_ERRC_SPI_NO_CONTEXT;
//     context->callback = callback;
    
//     // TX Transfer
//     dma_transfer_t tx_transfer = {
//         .request_id = spi_dmamux_req[device.instance][1],
//         .direction = MEM_TO_PERIPH,
//         .src_data_size = configs[device.instance].data_size,
//         .dest_data_size = configs[device.instance].data_size,
//         .priority = configs[device.instance].priority,
//         .callback = &spi_dma_callback,
//         .fifo_enabled = !transfer->write_fifo,
//         .src = source,
//         .dest = SPIx_TXDR[device.instance],
//         .size = size,
//         .context = context,
//         .mem_inc = write_mem_inc,
//     };
//     dma_start_transfer(&tx_transfer);

//     // RX Transfer
//     uint16_t dummy_buffer;
//     dma_transfer_t rx_transfer = {
//         .request_id = spi_dmamux_req[device.instance][0],
//         .direction = PERIPH_TO_MEM,
//         .src_data_size = configs[device.instance].data_size,
//         .dest_data_size = configs[device.instance].data_size,
//         .priority = configs[device.instance].priority,
//         .callback = &spi_dma_callback,
//         .fifo_enabled = false,
//         .src = SPIx_RXDR[device.instance],
//         .dest = &dummy_buffer,
//         .size = size,
//         .context = context,
//         .mem_inc = read_mem_inc,
//     };
//     dma_start_transfer(&rx_transfer);

//     // Enable DMA requests
//     SET_FIELD(SPIx_CFG1[device.instance], SPIx_CFG1_RXDMAEN);
//     SET_FIELD(SPIx_CFG1[device.instance], SPIx_CFG1_TXDMAEN);


//     return TI_ERRC_NONE;
//  }

// int spi_block(spi_device_t device) {
//     // int errc = ti_acquire_mutex(mutex[device.instance], mutex_timeouts[device.instance]);
//     // if (errc != TI_ERRC_NONE) {
//     //     return errc;
//     // }
//     tal_set_pin(device.gpio_pin, 0);
//     return TI_ERRC_NONE;
// }

// int spi_unblock(spi_device_t device) {
//     tal_set_pin(device.gpio_pin, 1);
//     // int errc = ti_release_mutex(mutex[device.instance], mutex_timeouts[device.instance]);
//     // if (errc != TI_ERRC_NONE) {
//     //     return 1;
//     // }
//     return TI_ERRC_NONE;
// }
