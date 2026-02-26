#include <stdint.h>
#include <stdbool.h>
#include "../internal/mmio.h"
#include "errc.h"
#include "qspi.h"

// Single-chip quadspi driver using flash 1 pins.
// This driver uses blocking. This may be problematic, as some
// flash operations can take a long time. It will likely be best to
// update this to use interrupts instead.

ti_errc_t qspi_init() {
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
    WRITE_FIELD(QUADSPI_CR, QUADSPI_CR_PRESCALER, 2U); // TODO: Find the kernel clock speed and find out what factor it needs to be divided by
    SET_FIELD(QUADSPI_CR, QUADSPI_CR_SSHIFT);          // This seems to add some extra stability
    WRITE_FIELD(QUADSPI_CR, QUADSPI_CR_FTHRES, 3U);    // Raises the FIFO threshold flag when FIFO contains three bytes
    CLR_FIELD(QUADSPI_CR, QUADSPI_CR_DFM);             // Duel-flash mode disabled (this is assuming that we're not using two external memories)
    CLR_FIELD(QUADSPI_CR, QUADSPI_CR_FSEL);            // FLASH 1 selected

    WRITE_FIELD(QUADSPI_DCR, QUADSPI_DCR_FSIZE, 23U);  // TODO: Find the number of bytes in MB your chip has and write n to this register, where n is 2^n = chip bytes (MB)
    WRITE_FIELD(QUADSPI_DCR, QUADSPI_DCR_CSHT, 3U);     // Defines the minimum number of cycles chip select must remain high
    CLR_FIELD(QUADSPI_DCR, QUADSPI_DCR_CKMODE);         // CLK must stay low when NCS is high

    SET_FIELD(QUADSPI_CR, QUADSPI_CR_EN);               // Enable quadspi

    return TI_ERRC_NONE; // TODO: Add error checks or return something else
}

ti_errc_t qspi_command(qspi_cmd_t *cmd, uint8_t *buf, bool is_read) {
    // Ensure that qspi is not busy
    if (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY)) return TI_ERRC_BUSY;

    // Specify the data size
    WRITE_FIELD(QUADSPI_DLR, QUADSPI_DLR_DL, cmd->data_size - 1);

    // Write to the Communication Control Register (QUADSPI_CCR)
    uint32_t fmode = is_read ? 0b01 : 0b00;            // 01 for Read, 00 for Write

    uint32_t ccr_val = (fmode << 26)                |  // Combine all QUADSPI_CCR fields into one 32-bit value
                       (cmd->data_mode << 24)       |  // ----------------------------------------------------
                       (cmd->dummy_cycles << 18)    |  // The command sequence begins as soon as we write to
                       (cmd->address_size << 12 )   |  // the QUADSPI_CCR register. Therefore, it is important
                       (cmd->address_mode << 10)    |  // to perform just one write operation.
                       (cmd->instruction_mode << 8) |
                       (cmd->instruction);

    WRITE_FIELD(QUADSPI_CCR, QUADSPI_CCR_REG, ccr_val); // Write to CCR

    // If necessary, specify the address to be sent to external memory
    if (cmd->address_mode != 0b00) WRITE_FIELD(QUADSPI_AR, QUADSPI_AR_REG, cmd->address);

    // Run main data loop to fill/depleat the data we want to read/send
    for (uint32_t i = 0; i < cmd->data_size; i++) {
        if (is_read) {
            while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_FLEVEL) == 0 && // Wait for FLEVEL or TCF flag
                   READ_FIELD(QUADSPI_SR, QUADSPI_SR_TCF) == 0);
            buf[i] = (uint8_t)READ_FIELD(QUADSPI_DR, QUADSPI_DR_REG); // Read from the data register
        } else {
            while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_FLEVEL) >= 32);  //Wait for FLEVEL flag
            WRITE_FIELD(QUADSPI_DR, QUADSPI_DR_REG, buf[i]);          // Write to data register
        }
    }

    // Wait for busy flag to be clear, then reset the FIFO threshold flag (FTF)
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_TCF) == 0);
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY) != 0);
    WRITE_WOFIELD(QUADSPI_FCR, QUADSPI_FCR_CTCF, 1U);
    WRITE_WOFIELD(QUADSPI_FCR, QUADSPI_FCR_CTEF, 1U);

    return TI_ERRC_NONE;
}

ti_errc_t qspi_poll_status_blk() {
    // Set match and mask values to check busy bit
    WRITE_FIELD(QUADSPI_PSMAR, QUADSPI_PSMAR_REG, 0x00); // Set match value
    WRITE_FIELD(QUADSPI_PSMKR, QUADSPI_PSMKR_REG, 0x01); // Set mask value

    // Set polling interval: how often QSPI is pulling the CS line low to "check-in" with flash
    WRITE_FIELD(QUADSPI_PIR, QUADSPI_PIR_INTERVAL, 32U); // Wait 32 cycles per "check-in"

    uint32_t ccr_val = (0b10 << 26)             | // Set FMODE to 0b10 for automatic polling mode
                       (QSPI_MODE_QUAD << 24)   | // Data uses all four qspi lines                 // TODO: Double check that this is correct for polling mode
                       (0U << 18)               | // No dummy bytes
                       (QSPI_MODE_NONE << 10)   | // No address phase
                       (QSPI_MODE_SINGLE << 8)  | // Instruction over single qspi line
                       (0x05);                    // Read status register instruction

    WRITE_FIELD(QUADSPI_CCR, QUADSPI_CCR_REG, ccr_val); // Write to all important fields at once

    // Wait for the hardware match-flag
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_SMF) == 0);

    WRITE_WOFIELD(QUADSPI_FCR, QUADSPI_FCR_CSMF, 1U); // Clear SMF
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY) != 0); // Wait until not busy

    return TI_ERRC_NONE;
}

ti_errc_t qspi_enter_memory_mapped(qspi_cmd_t *cmd) {
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
}

ti_errc_t qspi_exit_memory_mapped() {
    // Abort any ongoing memory-mapped access
    SET_BIT(QUADSPI_CR, QUADSPI_CR_ABORT);

    while (READ_BIT(QUADSPI_CR, QUADSPI_CR_ABORT)); // Wait for the abort to complete
    while (READ_FIELD(QUADSPI_SR, QUADSPI_SR_BUSY)); // Wait for the busy flag to clear
}

/**
 * TODO:
 * 1. Create a timeout system for your while loops that check the status register.
 *    You could set a timeout value (number of ticks), the subtract from it each time through the while loop.
 *    When the timeout variable reaches zero, you return an error code. See if there is a better way to do this
 *    before implementing.
 *
 * 3. Double check the main data loop of qspi_command. Even with timeouts, I'm worried that it's an inefficient and
 *    time consuming way to send data.
 *
 * 4. Double check what actually needs to be put into the CCR for each mode.
 *
 */