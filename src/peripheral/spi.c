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
 * @file peripheral/spi.c
 * @authors Jude Merritt
 * @brief Implementation of SPI driver interface
 */

#include "peripheral/spi.h"
#include "internal/mmio.h"
#include "peripheral/gpio.h"
#include <stdint.h>

#define INST1_SCK 44
#define INST1_MISO 45
#define INST1_MOSI 46

#define INST2_SCK 73
#define INST2_MISO 74
#define INST2_MOSI 75

#define INST3_SCK 109
#define INST3_MISO 110
#define INST3_MOSI 111

#define INST4_SCK 1
#define INST4_MISO 4
#define INST4_MOSI 5

#define INST5_SCK 21
#define INST5_MISO 22
#define INST5_MOSI 23

#define INST6_SCK 125
#define INST6_MISO 126
#define INST6_MOSI 127


// SPI instances
enum inst {
    INST_ONE = 1,
    INST_TWO,
    INST_THREE,
    INST_FOUR,
    INST_FIVE,
    INST_SIX
};

// Enables the clock of all SS pins
static inline void enable_ss_clocks(uint8_t* ss_list, uint8_t slave_count) {
    for (int i = 0; i < slave_count; i++) {
        tal_enable_clock(ss_list[i]);
    }
}

// Sets all SS pins to push pull mode
static inline void ss_push_pull(uint8_t* ss_list, uint8_t slave_count) {
    for (int i = 0; i < slave_count; i++) {
        tal_set_drain(ss_list[i], 0);
    }
}

// Sets all SS pins to output mode
static inline void ss_output_mode(uint8_t* ss_list, uint8_t slave_count) {
    for (int i = 0; i < slave_count; i++) {
        tal_set_mode(ss_list[i], 1);
    }
}

// Sets all SS pins to high
static inline void ss_high(uint8_t* ss_list, uint8_t slave_count) {
    for (int i = 0; i < slave_count; i++) {
        tal_set_pin(ss_list[i], 1);
    }
}

int spi_init(uint8_t inst, uint8_t* ss_list, uint8_t slave_count) {
    if (inst > 6 || inst < 1) {
        return -1;
    }

    // Enable clocks for MOSI, MISO, and SCK
    switch (inst) {
        case INST_ONE:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
            break;
        case INST_TWO:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            break;
        case INST_THREE:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOCEN);
            break;
        case INST_FOUR:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOEEN);
            break;
        case INST_FIVE:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOFEN);
            break;
        case INST_SIX:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOGEN);
            break;
    }

    // Enable clocks for all SS pins
    enable_ss_clocks(ss_list, slave_count);

    // Configure pins
    switch (inst) {
        case INST_ONE:
            // Set push pull
            ss_push_pull(ss_list, slave_count);
            tal_set_drain(INST1_SCK, 0);
            tal_set_drain(INST1_MOSI, 0);
            // Set mode 
            ss_output_mode(ss_list, slave_count);
            tal_set_mode(INST1_SCK, 2);
            tal_set_mode(INST1_MISO, 2);
            tal_set_mode(INST1_MOSI, 2);
            // Set alternate mode
            tal_alternate_mode(INST1_SCK, 0101);
            tal_alternate_mode(INST1_MISO, 0101);
            tal_alternate_mode(INST1_MOSI, 0101);
            // Set very high speed
            tal_set_speed(INST1_SCK, 3);
            tal_set_speed(INST1_MISO, 3);
            tal_set_speed(INST1_MOSI, 3);
            break;
        case INST_TWO:
            // Set push pull
            ss_push_pull(ss_list, slave_count);
            tal_set_drain(INST2_SCK, 0);
            tal_set_drain(INST2_MOSI, 0);
            // Set mode 
            ss_output_mode(ss_list, slave_count);
            tal_set_mode(INST2_SCK, 2);
            tal_set_mode(INST2_MISO, 2);
            tal_set_mode(INST2_MOSI, 2);
            // Set alternate mode
            tal_alternate_mode(INST2_SCK, 0101);
            tal_alternate_mode(INST2_MISO, 0101);
            tal_alternate_mode(INST2_MOSI, 0101);
            // Set very high speed
            tal_set_speed(INST2_SCK, 3);
            tal_set_speed(INST2_MISO, 3);
            tal_set_speed(INST2_MOSI, 3);
            break;
        case INST_THREE:
            // Set push pull
            ss_push_pull(ss_list, slave_count); 
            tal_set_drain(INST3_SCK, 0);
            tal_set_drain(INST3_MOSI, 0);
            // Set mode 
            ss_output_mode(ss_list, slave_count);
            tal_set_mode(INST3_SCK, 2);
            tal_set_mode(INST3_MISO, 2);
            tal_set_mode(INST3_MOSI, 2);
            // Set alternate mode
            tal_alternate_mode(INST3_SCK, 0110);
            tal_alternate_mode(INST3_MISO, 0110);
            tal_alternate_mode(INST3_MOSI, 0110);
            // Set very high speed
            tal_set_speed(INST3_SCK, 3);
            tal_set_speed(INST3_MISO, 3);
            tal_set_speed(INST3_MOSI, 3);
            break;
        case INST_FOUR:
            // Set push pull
            ss_push_pull(ss_list, slave_count);
            tal_set_drain(INST4_SCK, 0);
            tal_set_drain(INST4_MOSI, 0);
            // Set mode 
            ss_output_mode(ss_list, slave_count);
            tal_set_mode(INST4_SCK, 2);
            tal_set_mode(INST4_MISO, 2);
            tal_set_mode(INST4_MOSI, 2);
            // Set alternate mode
            tal_alternate_mode(INST4_SCK, 0101);
            tal_alternate_mode(INST4_MISO, 0101);
            tal_alternate_mode(INST4_MOSI, 0101);
            // Set very high speed
            tal_set_speed(INST4_SCK, 3);
            tal_set_speed(INST4_MISO, 3);
            tal_set_speed(INST4_MOSI, 3);
            break;
        case INST_FIVE:
            // Set push pull
            ss_push_pull(ss_list, slave_count);
            tal_set_drain(INST5_SCK, 0);
            tal_set_drain(INST5_MOSI, 0);
            // Set mode 
            ss_output_mode(ss_list, slave_count);
            tal_set_mode(INST5_SCK, 2);
            tal_set_mode(INST5_MISO, 2);
            tal_set_mode(INST5_MOSI, 2);
            // Set alternate mode
            tal_alternate_mode(INST5_SCK, 0101);
            tal_alternate_mode(INST5_MISO, 0101);
            tal_alternate_mode(INST5_MOSI, 0101);
            // Set very high speed
            tal_set_speed(INST5_SCK, 3);
            tal_set_speed(INST5_MISO, 3);
            tal_set_speed(INST5_MOSI, 3);
            break;
        case INST_SIX:
            // Set push pull
            ss_push_pull(ss_list, slave_count);
            tal_set_drain(INST6_SCK, 0);
            tal_set_drain(INST6_MOSI, 0);
            // Set mode 
            ss_output_mode(ss_list, slave_count);
            tal_set_mode(INST6_SCK, 2);
            tal_set_mode(INST6_MISO, 2);
            tal_set_mode(INST6_MOSI, 2);
            // Set alternate mode
            tal_alternate_mode(INST6_SCK, 0101);
            tal_alternate_mode(INST6_MISO, 0101);
            tal_alternate_mode(INST6_MOSI, 0101);
            // Set very high speed
            tal_set_speed(INST6_SCK, 3);
            tal_set_speed(INST6_MISO, 3);
            tal_set_speed(INST6_MOSI, 3);
            break;
    }

    // High speed clock enable
    SET_FIELD(RCC_CR, RCC_CR_HSION);
    while(!READ_FIELD(RCC_CR, RCC_CR_HSIRDY));

    // Set clock source
    if (inst < 4) {
        WRITE_FIELD(RCC_D2CCIP1R, RCC_D2CCIP1R_SPI123SRC, 0b100);
    } else if (inst == 4 || inst == 5) {
        WRITE_FIELD(RCC_D2CCIP1R, RCC_D2CCIP1R_SPI45SRC, 0b011);
    } else { // (inst == 6)
        WRITE_FIELD(RCC_D3CCIPR, RCC_D3CCIPR_SPI6SRC, 0b011);
    }
    // Enable SPI clock
    switch (inst) {
        case INST_ONE:
            SET_FIELD(RCC_APB2ENR, RCC_APB2ENR_SPI1EN);
            break;
        case INST_TWO:
            SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_SPIxEN[2]);
            break;
        case INST_THREE:
            SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_SPIxEN[3]);
            break;
        case INST_FOUR:
            SET_FIELD(RCC_APB2ENR, RCC_APB2ENR_SPI4EN);
            break;
        case INST_FIVE:
            SET_FIELD(RCC_APB2ENR, RCC_APB2ENR_SPI5EN);
            break;
        case INST_SIX:
            SET_FIELD(RCC_APB4ENR, RCC_APB4ENR_SPI6EN);
            break;
    }
    
    // Ensure SS lines are high
    ss_high(ss_list, slave_count);

    // Ensure SPI hardware is disabled before config
    CLR_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE);
    while(READ_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE));
    // Clear mode selection field
    CLR_FIELD(SPIx_CGFR[inst], SPIx_CGFR_I2SMOD);
    // Set threshold level
    WRITE_FIELD(SPIx_CFG1[inst], SPIx_CFG1_FTHVL, 0x00);
    // Set baudrate prescaler ( 64MHz /8  = 8MHz) 
    WRITE_FIELD(SPIx_CFG1[inst], SPIx_CFG1_MBR, 0b111); 
    // Set data size
    WRITE_FIELD(SPIx_CFG1[inst], SPIx_CFG1_DSIZE, 0b00111);
    // Set clock polarities
    CLR_FIELD(SPIx_CFG2[inst], SPIx_CFG2_CPOL);
    CLR_FIELD(SPIx_CFG2[inst], SPIx_CFG2_CPHA);
    // Slave management
    SET_FIELD(SPIx_CFG2[inst], SPIx_CFG2_SSM);
    SET_FIELD(SPIx_CR1[inst], SPIx_CR1_SSI);
    // Set SPI as master
    SET_FIELD(SPIx_CFG2[inst], SPIx_CFG2_MASTER);

    return 1;
}

int spi_transfer_sync(uint8_t inst, uint8_t ss_pin, void* src, void* dst, uint8_t size) {
    if (size == 0 || ss_pin > 255) return -1;

    CLR_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE);
    WRITE_FIELD(SPIx_CR2[inst], SPIx_CR2_TSIZE, size);
    SET_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE);

    while(!READ_FIELD(SPIx_SR[inst], SPIx_SR_TXP));

    // Pull SS pin low
    tal_set_pin(ss_pin, 0);

    for (int i = 0; i < size; i++) {
        while (!READ_FIELD(SPIx_SR[inst], SPIx_SR_TXP));
        *(volatile uint8_t *)SPIx_TXDR[inst] = ((uint8_t *)src)[i];

        // Start transfer
        if (i == 0) {
            SET_FIELD(SPIx_CR1[inst], SPIx_CR1_CSTART);
        }

        while (!READ_FIELD(SPIx_SR[inst], SPIx_SR_RXP) && !READ_FIELD(SPIx_SR[inst], SPIx_SR_EOT));

        ((uint8_t *)dst)[i] = *(volatile uint8_t *)SPIx_RXDR[inst];
    }

    // Wait for end of tranfer
    while (!READ_FIELD(SPIx_SR[inst], SPIx_SR_EOT));
    SET_FIELD(SPIx_IFCR[inst], SPIx_IFCR_EOTC);

    // Pull SS pin high to end transfer
    tal_set_pin(ss_pin, 1);

    return 1;
}