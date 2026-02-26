// Written by Jude Merritt

#include "peripheral/spi.h"
#include "internal/mmio.h"
#include <stdint.h>

// SPI instances
enum inst {
    INST_ONE = 1,
    INST_TWO,
    INST_THREE,
    INST_FOUR,
    INST_FIVE,
    INST_SIX
};

static void cs_low(uint8_t inst) {
    switch (inst) {
        case INST_ONE:
            CLR_FIELD(GPIOx_ODR[0], GPIOx_ODR_ODx[4]);
            break;
        case INST_TWO:
            CLR_FIELD(GPIOx_ODR[1], GPIOx_ODR_ODx[12]);  
            break;  
        case INST_THREE:
            CLR_FIELD(GPIOx_ODR[0], GPIOx_ODR_ODx[15]); 
            break;
        case INST_FOUR:
            CLR_FIELD(GPIOx_ODR[4], GPIOx_ODR_ODx[4]); 
            break;
        case INST_FIVE:
            CLR_FIELD(GPIOx_ODR[5], GPIOx_ODR_ODx[6]);
            break;
        case INST_SIX:
            CLR_FIELD(GPIOx_ODR[6], GPIOx_ODR_ODx[14]);
            break;
    }
}

static void cs_high(uint8_t inst) {
    switch (inst) {
        case INST_ONE:
            SET_FIELD(GPIOx_ODR[0], GPIOx_ODR_ODx[4]);
            break;
        case INST_TWO:
            SET_FIELD(GPIOx_ODR[1], GPIOx_ODR_ODx[12]);  
            break;  
        case INST_THREE:
            SET_FIELD(GPIOx_ODR[0], GPIOx_ODR_ODx[15]); 
            break;
        case INST_FOUR:
            SET_FIELD(GPIOx_ODR[4], GPIOx_ODR_ODx[4]); 
            break;
        case INST_FIVE:
            SET_FIELD(GPIOx_ODR[5], GPIOx_ODR_ODx[6]);
            break;
        case INST_SIX:
            SET_FIELD(GPIOx_ODR[6], GPIOx_ODR_ODx[14]);
            break;
    }
}

int spi_init(uint8_t inst) {
    if (inst > 6 || inst < 1) {
        return -1;
    }

    // Enable clocks
    switch (inst) {
        case INST_ONE:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
            break;
        case INST_TWO:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            break;
        case INST_THREE:
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
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
    // Configure pins
    switch (inst) {
        case INST_ONE:
            // Set push pull
            CLR_FIELD(GPIOx_OTYPER[0], GPIOx_OTYPER_OTx[4]);
            CLR_FIELD(GPIOx_OTYPER[0], GPIOx_OTYPER_OTx[5]);
            CLR_FIELD(GPIOx_OTYPER[0], GPIOx_OTYPER_OTx[7]);
            // Set mode 
            WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[4], 0b01);
            WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[5], 0b10);
            WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[6], 0b10);
            WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[7], 0b10);
            // Set alternate mode
            WRITE_FIELD(GPIOx_AFRL[0], GPIOx_AFRL_AFSELx[5], 0b0101);
            WRITE_FIELD(GPIOx_AFRL[0], GPIOx_AFRL_AFSELx[6], 0b0101);
            WRITE_FIELD(GPIOx_AFRL[0], GPIOx_AFRL_AFSELx[7], 0b0101);
            // Set very high speed
            WRITE_FIELD(GPIOx_OSPEEDR[0],GPIOx_OSPEEDR_OSPEEDx[5], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[0],GPIOx_OSPEEDR_OSPEEDx[6], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[0],GPIOx_OSPEEDR_OSPEEDx[7], 0b11);
            break;
        case INST_TWO:
            // Set push pull
            CLR_FIELD(GPIOx_OTYPER[1], GPIOx_OTYPER_OTx[12]);
            CLR_FIELD(GPIOx_OTYPER[1], GPIOx_OTYPER_OTx[13]);
            CLR_FIELD(GPIOx_OTYPER[1], GPIOx_OTYPER_OTx[15]);
            // Set mode 
            WRITE_FIELD(GPIOx_MODER[1], GPIOx_MODER_MODEx[12], 0b01);
            WRITE_FIELD(GPIOx_MODER[1], GPIOx_MODER_MODEx[13], 0b10);
            WRITE_FIELD(GPIOx_MODER[1], GPIOx_MODER_MODEx[14], 0b10);
            WRITE_FIELD(GPIOx_MODER[1], GPIOx_MODER_MODEx[15], 0b10);
            // Set alternate mode
            WRITE_FIELD(GPIOx_AFRH[1], GPIOx_AFRH_AFSELx[5], 0b0101);
            WRITE_FIELD(GPIOx_AFRH[1], GPIOx_AFRH_AFSELx[6], 0b0101);
            WRITE_FIELD(GPIOx_AFRH[1], GPIOx_AFRH_AFSELx[7], 0b0101);
            // Set very high speed
            WRITE_FIELD(GPIOx_OSPEEDR[1],GPIOx_OSPEEDR_OSPEEDx[13], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[1],GPIOx_OSPEEDR_OSPEEDx[14], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[1],GPIOx_OSPEEDR_OSPEEDx[15], 0b11);
            break;
        case INST_THREE:
            // Set push pull
            CLR_FIELD(GPIOx_OTYPER[0], GPIOx_OTYPER_OTx[15]); 
            CLR_FIELD(GPIOx_OTYPER[2], GPIOx_OTYPER_OTx[10]);
            CLR_FIELD(GPIOx_OTYPER[2], GPIOx_OTYPER_OTx[12]);
            // Set mode 
            WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[15], 0b01);
            WRITE_FIELD(GPIOx_MODER[2], GPIOx_MODER_MODEx[10], 0b10);
            WRITE_FIELD(GPIOx_MODER[2], GPIOx_MODER_MODEx[11], 0b10);
            WRITE_FIELD(GPIOx_MODER[2], GPIOx_MODER_MODEx[12], 0b10);
            // Set alternate mode
            WRITE_FIELD(GPIOx_AFRH[2], GPIOx_AFRH_AFSELx[2], 0b0110);
            WRITE_FIELD(GPIOx_AFRH[2], GPIOx_AFRH_AFSELx[3], 0b0110);
            WRITE_FIELD(GPIOx_AFRH[2], GPIOx_AFRH_AFSELx[4], 0b0110);
            // Set very high speed
            WRITE_FIELD(GPIOx_OSPEEDR[2],GPIOx_OSPEEDR_OSPEEDx[10], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[2],GPIOx_OSPEEDR_OSPEEDx[11], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[2],GPIOx_OSPEEDR_OSPEEDx[12], 0b11);
            break;
        case INST_FOUR:
            // Set push pull
            CLR_FIELD(GPIOx_OTYPER[4], GPIOx_OTYPER_OTx[4]);
            CLR_FIELD(GPIOx_OTYPER[4], GPIOx_OTYPER_OTx[2]);
            CLR_FIELD(GPIOx_OTYPER[4], GPIOx_OTYPER_OTx[6]);
            // Set mode 
            WRITE_FIELD(GPIOx_MODER[4], GPIOx_MODER_MODEx[4], 0b01); 
            WRITE_FIELD(GPIOx_MODER[4], GPIOx_MODER_MODEx[2], 0b10);
            WRITE_FIELD(GPIOx_MODER[4], GPIOx_MODER_MODEx[5], 0b10);
            WRITE_FIELD(GPIOx_MODER[4], GPIOx_MODER_MODEx[6], 0b10);
            // Set alternate mode
            WRITE_FIELD(GPIOx_AFRL[4], GPIOx_AFRL_AFSELx[2], 0b0101);
            WRITE_FIELD(GPIOx_AFRL[4], GPIOx_AFRL_AFSELx[5], 0b0101);
            WRITE_FIELD(GPIOx_AFRL[4], GPIOx_AFRL_AFSELx[6], 0b0101);
            // Set very high speed
            WRITE_FIELD(GPIOx_OSPEEDR[4],GPIOx_OSPEEDR_OSPEEDx[2], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[4],GPIOx_OSPEEDR_OSPEEDx[5], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[4],GPIOx_OSPEEDR_OSPEEDx[6], 0b11);
            break;
        case INST_FIVE:
            // Set push pull
            CLR_FIELD(GPIOx_OTYPER[5], GPIOx_OTYPER_OTx[6]);
            CLR_FIELD(GPIOx_OTYPER[5], GPIOx_OTYPER_OTx[7]);
            CLR_FIELD(GPIOx_OTYPER[5], GPIOx_OTYPER_OTx[9]);
            // Set mode 
            WRITE_FIELD(GPIOx_MODER[5], GPIOx_MODER_MODEx[6], 0b01); 
            WRITE_FIELD(GPIOx_MODER[5], GPIOx_MODER_MODEx[7], 0b10);
            WRITE_FIELD(GPIOx_MODER[5], GPIOx_MODER_MODEx[8], 0b10);
            WRITE_FIELD(GPIOx_MODER[5], GPIOx_MODER_MODEx[9], 0b10);
            // Set alternate mode
            WRITE_FIELD(GPIOx_AFRL[5], GPIOx_AFRL_AFSELx[7], 0b0101);
            WRITE_FIELD(GPIOx_AFRH[5], GPIOx_AFRH_AFSELx[0], 0b0101);
            WRITE_FIELD(GPIOx_AFRH[5], GPIOx_AFRH_AFSELx[1], 0b0101);
            // Set very high speed
            WRITE_FIELD(GPIOx_OSPEEDR[5],GPIOx_OSPEEDR_OSPEEDx[7], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[5],GPIOx_OSPEEDR_OSPEEDx[8], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[5],GPIOx_OSPEEDR_OSPEEDx[9], 0b11);
            break;
        case INST_SIX:
            // Set push pull
            CLR_FIELD(GPIOx_OTYPER[6], GPIOx_OTYPER_OTx[8]);
            CLR_FIELD(GPIOx_OTYPER[6], GPIOx_OTYPER_OTx[12]);
            CLR_FIELD(GPIOx_OTYPER[6], GPIOx_OTYPER_OTx[14]);
            // Set mode 
            WRITE_FIELD(GPIOx_MODER[6], GPIOx_MODER_MODEx[8], 0b01);
            WRITE_FIELD(GPIOx_MODER[6], GPIOx_MODER_MODEx[12], 0b10);
            WRITE_FIELD(GPIOx_MODER[6], GPIOx_MODER_MODEx[13], 0b10);
            WRITE_FIELD(GPIOx_MODER[6], GPIOx_MODER_MODEx[14], 0b10);
            // Set alternate mode
            WRITE_FIELD(GPIOx_AFRH[6], GPIOx_AFRH_AFSELx[4], 0b0101);
            WRITE_FIELD(GPIOx_AFRH[6], GPIOx_AFRH_AFSELx[5], 0b0101);
            WRITE_FIELD(GPIOx_AFRH[6], GPIOx_AFRH_AFSELx[6], 0b0101);
            // Set very high speed
            WRITE_FIELD(GPIOx_OSPEEDR[6],GPIOx_OSPEEDR_OSPEEDx[12], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[6],GPIOx_OSPEEDR_OSPEEDx[13], 0b11);
            WRITE_FIELD(GPIOx_OSPEEDR[6],GPIOx_OSPEEDR_OSPEEDx[14], 0b11);
            break;
    }

    // High speed clock enable
    SET_FIELD(RCC_CR, RCC_CR_HSION);
    while(!READ_FIELD(RCC_CR, RCC_CR_HSIRDY));

    // Set clock source
    if (inst < 4) {
        WRITE_FIELD(RCC_D2CCIP1R, RCC_D2CCIP1R_SPI123SRC, 0b100);
    } else if (inst == 4 || inst == 5) {
        WRITE_FIELD(RCC_D2CCIP1R, RCC_D2CCIP1R_SPI45SRC, 0b100);
    } else { // (inst == 6)
        WRITE_FIELD(RCC_D3CCIPR, RCC_D3CCIPR_SPI6SRC, 0b100);
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
    
    // Ensure CS line is high
    cs_high(inst);

    // Ensure SPI hardware is disabled before config
    CLR_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE);
    while(READ_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE));
    // Clear mode selection field
    CLR_FIELD(SPIx_CGFR[inst], SPIx_CGFR_I2SMOD);
    // Set threshold level
    WRITE_FIELD(SPIx_CFG1[inst], SPIx_CFG1_FTHVL, 0x0);
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

int spi_transfer_sync(uint8_t inst, void* src, void* dst, uint8_t size) {
    if (size == 0) return -1;

    CLR_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE);
    WRITE_FIELD(SPIx_CR2[inst], SPIx_CR2_TSIZE, size);
    SET_FIELD(SPIx_CR1[inst], SPIx_CR1_SPE);

    while(!READ_FIELD(SPIx_SR[inst], SPIx_SR_TXP));

    // Pull CS pin low
    cs_low(inst);

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

    // Pull CS pin high to end transfer
    cs_high(inst);

    return 1;
}