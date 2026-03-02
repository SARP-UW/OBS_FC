#include <stdint.h>
#include <stdbool.h>
#include "../internal/mmio.h"
#include "errc.h"
#include "qspi.h"

void qspi_init() {
    // Enable RHB3 clock and reset QSPI
    SET_FIELD(RCC_AHB3ENR, RCC_AHB3ENR_QSPIEN);
    SET_FIELD(RCC_AHB3RSTR, RCC_AHB3RSTR_QSPIRST);
    CLR_FIELD(RCC_AHB3RSTR, RCC_AHB3RSTR_QSPIRST);

    // Configure GPIO pins
    // QSPI_CLK : PB2
    SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);                    // Enable GPIOB clock
    WRITE_FIELD(GPIOx_MODER[1], GPIOx_MODER_MODEx[2], 0b10);        // Set to alternate mode
    WRITE_FIELD(GPIOx_AFRL[1], GPIOx_AFRL_AFSELx[2], 0b1001);       // Define as alternate function nine (AF9)
    WRITE_FIELD(GPIOx_OSPEEDR[1], GPIOx_OSPEEDR_OSPEEDx[2], 0b11);  // Set to very high speed
    WRITE_FIELD(GPIOx_PUPDR[1], GPIOx_PUPDR_PUPDx[2], 0b10);        // Pull CS pin down (CS should be low when idle)

    // QSPI_CS : PG6
    SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOGEN);                    // Enable GPIOG clock
    WRITE_FIELD(GPIOx_MODER[6], GPIOx_MODER_MODEx[6], 0b10);        // Set to alternate mode
    WRITE_FIELD(GPIOx_AFRL[6], GPIOx_AFRL_AFSELx[6], 0b1010);       // Define as alternate function nine (AF10)
    WRITE_FIELD(GPIOx_OSPEEDR[6], GPIOx_OSPEEDR_OSPEEDx[6], 0b11);  // Set to very high speed
    WRITE_FIELD(GPIOx_PUPDR[6], GPIOx_PUPDR_PUPDx[6], 0b01);        // Pull CS pin up

    // QSPI_BK1_IO0 : PD11
    SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIODEN);                    // Enable GPIOD clock
    WRITE_FIELD(GPIOx_MODER[3], GPIOx_MODER_MODEx[11], 0b10);       // Set to alternate mode
    WRITE_FIELD(GPIOx_AFRH[3], GPIOx_AFRH_AFSELx[3], 0b1001);       // Define as alternate function nine (AF9) //TODO: Add field to MMIO
    WRITE_FIELD(GPIOx_OSPEEDR[3], GPIOx_OSPEEDR_OSPEEDx[11], 0b11); // Set to very high speed

    // QSPI_BK1_IO1 : PD12
    WRITE_FIELD(GPIOx_MODER[3], GPIOx_MODER_MODEx[12], 0b10);       // Set to alternate mode
    WRITE_FIELD(GPIOx_AFRH[3], GPIOx_AFRH_AFSELx[4], 0b1001);       // Define as alternate function nine (AF9)
    WRITE_FIELD(GPIOx_OSPEEDR[3], GPIOx_OSPEEDR_OSPEEDx[12], 0b11); // Set to very high speed

    // QSPI_BK1_IO2 : PE2
    SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOEEN);                    // Enable GPIOE clock
    WRITE_FIELD(GPIOx_MODER[4], GPIOx_MODER_MODEx[2], 0b10);        // Set to alternate mode
    WRITE_FIELD(GPIOx_AFRL[4], GPIOx_AFRL_AFSELx[2], 0b1001);       // Define as alternate function nine (AF9)
    WRITE_FIELD(GPIOx_OSPEEDR[4], GPIOx_OSPEEDR_OSPEEDx[2], 0b11);  // Set to very high speed

    // QSPI_BK1_IO3 : PD13
    WRITE_FIELD(GPIOx_MODER[3], GPIOx_MODER_MODEx[13], 0b10);        // Set to alternate mode
    WRITE_FIELD(GPIOx_AFRH[3], GPIOx_AFRH_AFSELx[5], 0b1001);        // Define as alternate function nine (AF9)
    WRITE_FIELD(GPIOx_OSPEEDR[3], GPIOx_OSPEEDR_OSPEEDx[13], 0b11);  // Set to very high speed

    // Device Configuration
    WRITE_FIELD(QUADSPI_CR, QUADSPI_CR_PRESCALER, 2U); // TODO: Find what factor the kernel clock speed needs to be divided by
    SET_FIELD(QUADSPI_CR, QUADSPI_CR_SSHIFT);          // This seems to add some extra stability at high speeds (waits extra half-cycle to sample data)
    WRITE_FIELD(QUADSPI_CR, QUADSPI_CR_FTHRES, 3U);    // Raises the FIFO threshold flag when FIFO contains four bytes (N + 1)
    CLR_FIELD(QUADSPI_CR, QUADSPI_CR_DFM);             // Duel-flash mode disabled
    CLR_FIELD(QUADSPI_CR, QUADSPI_CR_FSEL);            // FLASH 1 selected

    WRITE_FIELD(QUADSPI_DCR, QUADSPI_DCR_FSIZE, 23U);  // TODO: Find the number of bytes in MB your external chip has and write n to this register, where n is 2^n = chip bytes (MB)
    WRITE_FIELD(QUADSPI_DCR, QUADSPI_DCR_CSHT, 3U);     // Defines the minimum number of cycles chip select must remain high
    CLR_FIELD(QUADSPI_DCR, QUADSPI_DCR_CKMODE);         // CLK must stay low when NCS is high

    SET_FIELD(QUADSPI_CR, QUADSPI_CR_EN);               // Enable quadspi
}

enum ti_errc_t qspi_command_blk(qspi_cmd_t *cmd, uint8_t *buf, bool is_read) {
    if (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY)) return TI_ERRC_BUSY;

    if (cmd->data_size > 0) {
        WRITE_FIELD(QUADSPI_DLR, QUADSPI_DLR_DL, cmd->data_size - 1);
    }

    uint32_t fmode = is_read ? 0b01 : 0b00;             

    uint32_t ccr_val = (fmode << 26)                |  // Combine all QUADSPI_CCR fields into one 32-bit value
                       (cmd->data_mode << 24)       |  // ----------------------------------------------------
                       (cmd->dummy_cycles << 18)    |  // The command sequence begins as soon as you write to 
                       (cmd->address_size << 12 )   |  // the QUADSPI_CCR register. Therefore, it is important
                       (cmd->address_mode << 10)    |  // to perform just one write operation.
                       (cmd->instruction_mode << 8) |  
                       (cmd->instruction);

    // Write to CCR
    WRITE_FIELD(QUADSPI_CCR, QUADSPI_CCR_REG, ccr_val);

    // If necessary, specify the address to be sent to external memory
    if (cmd->address_mode != 0b00) {
        WRITE_FIELD(QUADSPI_AR, QUADSPI_AR_REG, cmd->address);
    }

    // Run main data loop
    for (uint32_t i = 0; i < cmd->data_size; i++) {
        if (is_read) {
            while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_FLEVEL) == 0);
            
            buf[i] = (uint8_t)READ_FIELD(QUADSPI_DR, QUADSPI_DR_REG); 
        } else { // (is_write)
            while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_FLEVEL) >= 32);  
            
            WRITE_FIELD(QUADSPI_DR, QUADSPI_DR_REG, buf[i]);          
        }
    }

    // Wait for the busy flag and transfer complete flag
    while (!READ_FIELD(QUADSPI_SR, QUADSPI_SR_TCF));  
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY));  
    
    WRITE_WO_FIELD(QUADSPI_FCR, QUADSPI_FCR_CTCF, 1U); 
    WRITE_WO_FIELD(QUADSPI_FCR, QUADSPI_FCR_CTEF, 1U); 

    return TI_ERRC_NONE;
}

enum ti_errc_t qspi_poll_status_blk() {
    // Stop automatic polling mode after a match
    SET_FIELD(QUADSPI_CR, QUADSPI_CR_APMS);

    // Look for internal busy bit to be zero
    WRITE_FIELD(QUADSPI_PSMAR, QUADSPI_PSMAR_REG, 0x00); 
    WRITE_FIELD(QUADSPI_PSMKR, QUADSPI_PSMKR_REG, 0x01); 
    WRITE_FIELD(QUADSPI_PIR, QUADSPI_PIR_INTERVAL, 32U); 

    uint32_t ccr_val = (0b10 << 26)             | // Set FMODE to 0b10 for automatic polling mode
                       (QSPI_MODE_QUAD << 24)   | // Data uses all four qspi lines                
                       (0U << 18)               | // No dummy bytes
                       (QSPI_MODE_NONE << 10)   | // No address phase
                       (QSPI_MODE_SINGLE << 8)  | // Instruction over single qspi line
                       (0x05);                    // Read status register instruction                   

    // Write to CCR
    WRITE_FIELD(QUADSPI_CCR, QUADSPI_CCR_REG, ccr_val); 

    // Wait for the hardware match-flag (keeps polling until flag is raised)
    while (!READ_FIELD(QUADSPI_SR, QUADSPI_SR_SMF));

    WRITE_WO_FIELD(QUADSPI_FCR, QUADSPI_FCR_CSMF, 1U); 
    
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY)); 
    
    CLR_FIELD(QUADSPI_CR, QUADSPI_CR_APMS);

    return TI_ERRC_NONE;
}

enum ti_errc_t qspi_enter_memory_mapped(qspi_cmd_t *cmd) {
    // Ensure the QSPI is not busy
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY));

    // Configure the CCR for Memory Mapped Mode
    uint32_t ccr_val =
        (0b11 << 26)               | // FMODE: 0b11 = Memory Mapped
        (QSPI_MODE_SINGLE << 24)   | // DMODE: Data on 1 line
        (8 << 18)                  | // DCYC: 8 dummy cycles (required for 0x0B)
        (QSPI_MODE_SINGLE << 12)   | // ADSIZE: 24-bit address (0b10)
        (QSPI_MODE_SINGLE << 10)   | // ADMODE: Address on 1 line
        (QSPI_MODE_SINGLE << 8)    | // IMODE: Instruction on 1 line
        (0x0B);                      // Instruction: Fast Read

    WRITE_FIELD(QUADSPI_CCR, QUADSPI_CCR_REG, ccr_val);

    return TI_ERRC_NONE;
}

enum ti_errc_t qspi_exit_memory_mapped() {
    // Abort any ongoing memory-mapped access
    SET_FIELD(QUADSPI_CR, QUADSPI_CR_ABORT);

    // Wait for busy and abort flags
    while (READ_FIELD(QUADSPI_CR, QUADSPI_CR_ABORT)); 
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY)); 

    return TI_ERRC_NONE;
}