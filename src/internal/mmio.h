/**
 * This file is part of the Titan Project.
 * Copyright (c) 2025 UW SARP
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
 * @file internal/mmio.h
 * @authors Aaron McBride
 * @brief Resources and utilities for memory-mapped I/O.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * @section MMIO Type Definitions
 **************************************************************************************************/

typedef volatile uint64_t* rw_reg64_t; /** @brief 64 bit read-write register pointer type. */
typedef volatile uint32_t* rw_reg32_t; /** @brief 32 bit read-write register pointer type. */
typedef volatile uint16_t* rw_reg16_t; /** @brief 16 bit read-write register pointer type. */
typedef volatile uint8_t*  rw_reg8_t;  /** @brief 8 bit read-write register pointer type. */

typedef const volatile uint64_t* ro_reg64_t; /** @brief 64 bit read-only register pointer type. */
typedef const volatile uint32_t* ro_reg32_t; /** @brief 32 bit read-only register pointer type. */
typedef const volatile uint16_t* ro_reg16_t; /** @brief 16 bit read-only register pointer type. */
typedef const volatile uint8_t*  ro_reg8_t;  /** @brief 8 bit read-only register pointer type. */

typedef struct { uint64_t msk; int32_t pos; } field64_t; /** @brief 64 bit register field type. */
typedef struct { uint32_t msk; int32_t pos; } field32_t; /** @brief 32 bit register field type. */
typedef struct { uint16_t msk; int32_t pos; } field16_t; /** @brief 16 bit register field type. */
typedef struct { uint8_t  msk; int32_t pos; } field8_t;  /** @brief 8 bit register field type. */

typedef struct {
    volatile uint32_t *CR1;
    volatile uint32_t *CR2;
    volatile uint32_t *CR3;
    volatile uint32_t *BRR;
    volatile uint32_t *TDR;
    volatile uint32_t *RDR;
    
} UART_Regs;

static UART_Regs UART_MAP[9] = {
    [4] = { (uint32_t*)0x40004C00U, (uint32_t*)0x40004C04U, (uint32_t*)0x40004C08U, 
            (uint32_t*)0x40004C0CU, (uint32_t*)0x40004C28U, (uint32_t*)0x40004C24U },
    [5] = { (uint32_t*)0x40005000U, (uint32_t*)0x40005004U, (uint32_t*)0x40005008U, 
            (uint32_t*)0x4000500CU, (uint32_t*)0x40005028U, (uint32_t*)0x40005024U },
    [7] = { (uint32_t*)0x40007800U, (uint32_t*)0x40007804U, (uint32_t*)0x40007808U, 
            (uint32_t*)0x4000780CU, (uint32_t*)0x40007828U, (uint32_t*)0x40007824U },
    [8] = { (uint32_t*)0x40007C00U, (uint32_t*)0x40007C04U, (uint32_t*)0x40007C08U, 
            (uint32_t*)0x40007C0CU, (uint32_t*)0x40007C28U, (uint32_t*)0x40007C24U },
};

/**************************************************************************************************
 * @section MMIO Utilities
 **************************************************************************************************/

/**
 * @brief Constructs a field struct from a position and width.
 * @param ftype (type) The type of the field to construct.
 * @param pos (integral value) The position of the field.
 * @param width (integral value) The width of the field.
 * @returns (field_t) A struct which defines the location of the specified field.
 * @note - All arguments of this macro are only expanded once.
 */
#define MAKE_FIELD(ftype, pos, width) ({ \
  const __auto_type _pos = (pos); \
  const __auto_type _width = (width); \
  (ftype) { \
    .msk = ((UINT64_C(1) << (_width)) - 1U) << _pos, \
    .pos = _pos \
  }; \
})

/**
 * @brief Determines if a value fits in the range of a field.
 * @param field (field_t) The target field.
 * @param value (integral value) The target value.
 * @returns (bool) True if @p [value] can be represented in @p [field], 
 *          or false otherwise.
 * @note - All arguments of this macro are only expanded once.
 */
#define IN_RANGE_FIELD(field, value) ({ \
  const __auto_type _field = (field); \
  const __auto_type _value = (value); \
  (_value & ~(_field.msk >> _field.pos)) == 0U; \
})

/**
 * @brief Writes a value to a field.
 * @param dst (integral pointer) The location to write to.
 * @param field (field_t) The target field.
 * @param value (integral value) The value to write to the field.
 * @returns (integral value) The value assigned to @p [dst].
 * @note - All arguments of this macro are only expanded once.
 */
#define WRITE_FIELD(dst, field, value) ({ \
  const __auto_type _dst = (dst); \
  const __auto_type _field = (field); \
  const __auto_type _value = (value); \
 *_dst = (*_dst & ~_field.msk) | ((_value << _field.pos) & _field.msk); \
})

/**
 * @brief Writes a value to a write-only field.
 * @param dst (integral pointer) The location to write to.
 * @param field (field_t) The target field.
 * @param value (integral value) The value to write to the field.
 * @returns (integral value) The value assigned to @p [dst].
 * @note - All arguments of this macro are only expanded once.
 */
#define WRITE_WO_FIELD(dst, field, value) ({ \
  const __auto_type _dst = (dst); \
  const __auto_type _field = (field); \
  const __auto_type _value = (value); \
 *_dst = (_value << _field.pos) & _field.msk; \
})

/**
 * @brief Sets all the bits in a field.
 * @param dst (integral pointer) The location to write to.
 * @param field (field_t) The target field.
 * @returns (integral value) The value assigned to @p [dst].
 * @note - All arguments of this macro are only expanded once.
 */
#define SET_FIELD(dst, field) ({ \
  const __auto_type _dst = (dst); \
  const __auto_type _field = (field); \
  *_dst |= _field.msk; \
})

/**
 * @brief Sets all the bits in a write-only field.
 * @param dst (integral pointer) The location to write to.
 * @param field (field_t) The target field.
 * @returns (integral value) The value assigned to @p [dst].
 * @note - All arguments of this macro are only expanded once.
 */
#define SET_WO_FIELD(dst, field) ({ \
  const __auto_type _dst = (dst); \
  const __auto_type _field = (field); \
 *_dst = _field.msk; \
})

/**
 * @brief Clears all the bits in a field.
 * @param dst (integral pointer) The location to write to.
 * @param field (field_t) The target field.
 * @returns (integral value) The value assigned to @p [dst].
 * @note - All arguments of this macro are only expanded once.
 */
#define CLR_FIELD(dst, field) ({ \
  const __auto_type _dst = (dst); \
  const __auto_type _field = (field); \
 *_dst &= ~_field.msk; \
})

/**
 * @brief Toggles all the bits in a field.
 * @param dst (integral pointer) The location to write to.
 * @param field (field_t) The target field.
 * @returns (integral value) The value assigned to @p [dst].
 * @note - All arguments of this macro are only expanded once.
 */
#define TOGL_FIELD(dst, field) ({ \
  const __auto_type _dst = (dst); \
  const __auto_type _field = (field); \
 *_dst ^= _field.msk; \
})

/**
 * @brief Reads the value of a field
 * @param src (integral pointer) The location to read from.
 * @param field (field_t) The target field.
 * @returns (integral value) The value of @p [field] in @p [src].
 * @note - All arguments of this macro are only expanded once.
 */
#define READ_FIELD(src, field) ({ \
  const __auto_type _src = (src); \
  const __auto_type _field = (field); \
  (*_src & _field.msk) >> _field.pos; \
})

/**
 * @brief Determines if a field is set.
 * @param src (integral pointer) The location to read from.
 * @param field (field_t) The target field.
 * @returns (bool) True if all bits  @p [field] in @p [src] are set, or false otherwise.
 * @note - All arguments of this macro are only expanded once.
 */
#define IS_FIELD_SET(src, field) ({ \
  const __auto_type _src = (src); \
  const __auto_type _field = (field); \
  (*_src & _field.msk) == _field.msk; \
})

/**
 * @brief Determines if a field is clear.
 * @param src (integral pointer) The location to read from.
 * @param field (field_t) The target field.
 * @returns (bool) True if all bits in @p [field] in @p [src] are clear, or false otherwise.
 * @note - All arguments of this macro are only expanded once.
 */
#define IS_FIELD_CLR(src, field) ({ \
  const __auto_type _src = (src); \
  const __auto_type _field = (field); \
  (*_src & _field.msk) == 0U; \
})

#define TO_FIELD(value, field) ({ \
  const __auto_type _value = (value); \
  const __auto_type _field = (field); \
  (_value << _field.pos) & _field.msk; \
})

/**************************************************************************************************
 * @section Actuator (MAX22216/MAX22217) Register Definitions
 **************************************************************************************************/
#define ACTUATOR_REG_GLOBAL_CTRL   0x00
#define ACTUATOR_REG_GLOBAL_CFG    0x01
#define ACTUATOR_REG_STATUS        0x02
#define ACTUATOR_REG_STATUS_CFG    0x03
#define ACTUATOR_REG_DC_H2L        0x04
#define ACTUATOR_REG_VM_MONITOR    0x05
#define ACTUATOR_REG_VM_THRESHOLD  0x06
#define ACTUATOR_REG_F_AC          0x07
#define ACTUATOR_REG_U_AC_SCAN     0x08

#define ACTUATOR_REG_FAULT0        0x65
#define ACTUATOR_REG_FAULT1        0x66

#define ACTUATOR_GLOBAL_CTRL_F_PWM_M_POS 4
#define ACTUATOR_GLOBAL_CTRL_F_PWM_M_MSK (0xFu << ACTUATOR_GLOBAL_CTRL_F_PWM_M_POS)
#define ACTUATOR_GLOBAL_CTRL_CNTL_POS(ch) (ch)

#define ACTUATOR_GLOBAL_CFG_ACTIVE_POS    15
#define ACTUATOR_GLOBAL_CFG_MASKS_POS     8
#define ACTUATOR_GLOBAL_CFG_CNTL_POL_POS  7
#define ACTUATOR_GLOBAL_CFG_STAT_POL_POS  6
#define ACTUATOR_GLOBAL_CFG_VDR_MODE_POS  4
#define ACTUATOR_GLOBAL_CFG_CHS_POS       0

#define ACTUATOR_CFG_CTRL0_CTRL_MODE_POS  14
#define ACTUATOR_CFG_CTRL0_HHF_EN_POS     13
#define ACTUATOR_CFG_CTRL0_OL_EN_POS      12
#define ACTUATOR_CFG_CTRL0_H2L_EN_POS     11
#define ACTUATOR_CFG_CTRL0_RDWE_POS       10
#define ACTUATOR_CFG_CTRL0_RMDE_POS       9
#define ACTUATOR_CFG_CTRL0_RUPE_POS       8

#define ACTUATOR_CFG_CTRL1_HSNLS_POS      10
#define ACTUATOR_CFG_CTRL1_F_PWM_POS      8
#define ACTUATOR_CFG_CTRL1_T_BLANK_POS    6
#define ACTUATOR_CFG_CTRL1_SLEW_POS       4
#define ACTUATOR_CFG_CTRL1_GAIN_POS       2
#define ACTUATOR_CFG_CTRL1_SNSF_POS       0

/**************************************************************************************************
 * @section COMP1 Definitions
 **************************************************************************************************/

/** @subsection COMP1 Register Definitions */

extern ro_reg32_t const COMP1_SR;    /** @brief Comparator status register. */
extern rw_reg32_t const COMP1_ICFR;  /** @brief Comparator interrupt clear flag register. */
extern rw_reg32_t const COMP1_OR;    /** @brief Comparator option register. */
extern rw_reg32_t const COMP1_CFGR1; /** @brief Comparator configuration register 1. */
extern rw_reg32_t const COMP1_CFGR2; /** @brief Comparator configuration register 2. */

/** @subsection COMP1 Register Field Definitions */

extern const field32_t COMP1_OR_AFOP;        /** @brief Selection of source for alternate function of output ports. */
extern const field32_t COMP1_OR_OR;          /** @brief Option register. */
extern const field32_t COMP1_CFGR1_EN;       /** @brief COMP channel 1 enable bit. */
extern const field32_t COMP1_CFGR1_BRGEN;    /** @brief Scaler bridge enable. */
extern const field32_t COMP1_CFGR1_SCALEN;   /** @brief Voltage scaler enable bit. */
extern const field32_t COMP1_CFGR1_POLARITY; /** @brief COMP channel 1 polarity selection bit. */
extern const field32_t COMP1_CFGR1_ITEN;     /** @brief COMP channel 1 interrupt enable. */
extern const field32_t COMP1_CFGR1_HYST;     /** @brief COMP channel 1 hysteresis selection bits. */
extern const field32_t COMP1_CFGR1_PWRMODE;  /** @brief Power mode of the COMP channel 1. */
extern const field32_t COMP1_CFGR1_INMSEL;   /** @brief COMP channel 1 inverting input selection field. */
extern const field32_t COMP1_CFGR1_INPSEL;   /** @brief COMP channel 1 non-inverting input selection bit. */
extern const field32_t COMP1_CFGR1_BLANKING; /** @brief COMP channel 1 blanking source selection bits. */
extern const field32_t COMP1_CFGR1_LOCK;     /** @brief Lock bit. */
extern const field32_t COMP1_CFGR2_EN;       /** @brief COMP channel 1 enable bit. */
extern const field32_t COMP1_CFGR2_BRGEN;    /** @brief Scaler bridge enable. */
extern const field32_t COMP1_CFGR2_SCALEN;   /** @brief Voltage scaler enable bit. */
extern const field32_t COMP1_CFGR2_POLARITY; /** @brief COMP channel 1 polarity selection bit. */
extern const field32_t COMP1_CFGR2_WINMODE;  /** @brief Window comparator mode selection bit. */
extern const field32_t COMP1_CFGR2_ITEN;     /** @brief COMP channel 1 interrupt enable. */
extern const field32_t COMP1_CFGR2_HYST;     /** @brief COMP channel 1 hysteresis selection bits. */
extern const field32_t COMP1_CFGR2_PWRMODE;  /** @brief Power mode of the COMP channel 1. */
extern const field32_t COMP1_CFGR2_INMSEL;   /** @brief COMP channel 1 inverting input selection field. */
extern const field32_t COMP1_CFGR2_INPSEL;   /** @brief COMP channel 1 non-inverting input selection bit. */
extern const field32_t COMP1_CFGR2_BLANKING; /** @brief COMP channel 1 blanking source selection bits. */
extern const field32_t COMP1_CFGR2_LOCK;     /** @brief Lock bit. */

/** @subsection Enumerated COMP1 Register Field Definitions */

extern const field32_t COMP1_SR_CxVAL[3];   /** @brief COMP channel 1 output status bit. */
extern const field32_t COMP1_SR_CxIF[3];    /** @brief COMP channel 1 interrupt flag. */
extern const field32_t COMP1_ICFR_CCxIF[3]; /** @brief Clear COMP channel 1 interrupt flag. */

/**************************************************************************************************
 * CRS Definitions
 **************************************************************************************************/

/** @subsection CRS Register Definitions */

extern rw_reg32_t const CRS_CR;   /** @brief CRS control register. */
extern rw_reg32_t const CRS_CFGR; /** @brief This register can be written only when the frequency error counter is disabled (CEN bit is cleared in CRS_CR). When the counter is enabled, this register is write-protected. */
extern ro_reg32_t const CRS_ISR;  /** @brief CRS interrupt and status register. */
extern rw_reg32_t const CRS_ICR;  /** @brief CRS interrupt flag clear register. */

/** @subsection CRS Register Field Definitions */

extern const field32_t CRS_CR_SYNCOKIE;   /** @brief SYNC event OK interrupt enable. */
extern const field32_t CRS_CR_SYNCWARNIE; /** @brief SYNC warning interrupt enable. */
extern const field32_t CRS_CR_ERRIE;      /** @brief Synchronization or trimming error interrupt enable. */
extern const field32_t CRS_CR_ESYNCIE;    /** @brief Expected SYNC interrupt enable. */
extern const field32_t CRS_CR_CEN;        /** @brief Frequency error counter enable this bit enables the oscillator clock for the frequency error counter. When this bit is set, the CRS_CFGR register is write-protected and cannot be modified. */
extern const field32_t CRS_CR_AUTOTRIMEN; /** @brief Automatic trimming enable this bit enables the automatic hardware adjustment of TRIM bits according to the measured frequency error between two SYNC events. If this bit is set, the TRIM bits are read-only. The TRIM value can be adjusted by hardware by one or two steps at a time, depending on the measured frequency error value. Refer to section7.3.4: frequency error evaluation and automatic trimming for more details. */
extern const field32_t CRS_CR_SWSYNC;     /** @brief Generate software SYNC event this bit is set by software in order to generate a software SYNC event. It is automatically cleared by hardware. */
extern const field32_t CRS_CR_TRIM;       /** @brief HSI48 oscillator smooth trimming these bits provide a user-programmable trimming value to the HSI48 oscillator. They can be programmed to adjust to variations in voltage and temperature that influence the frequency of the HSI48. The default value is 32, which corresponds to the middle of the trimming interval. The trimming step is around 67 khz between two consecutive TRIM steps. A higher TRIM value corresponds to a higher output frequency. When the AUTOTRIMEN bit is set, this field is controlled by hardware and is read-only. */
extern const field32_t CRS_CFGR_RELOAD;   /** @brief Counter reload value RELOAD is the value to be loaded in the frequency error counter with each SYNC event. Refer to section7.3.3: frequency error measurement for more details about counter behavior. */
extern const field32_t CRS_CFGR_FELIM;    /** @brief Frequency error limit FELIM contains the value to be used to evaluate the captured frequency error value latched in the FECAP[15:0] bits of the CRS_ISR register. Refer to section7.3.4: frequency error evaluation and automatic trimming for more details about FECAP evaluation. */
extern const field32_t CRS_CFGR_SYNCDIV;  /** @brief SYNC divider these bits are set and cleared by software to control the division factor of the SYNC signal. */
extern const field32_t CRS_CFGR_SYNCSRC;  /** @brief SYNC signal source selection these bits are set and cleared by software to select the SYNC signal source. Note: when using USB LPM (link power management) and the device is in sleep mode, the periodic USB SOF will not be generated by the host. No SYNC signal will therefore be provided to the CRS to calibrate the HSI48 on the run. To guarantee the required clock precision after waking up from sleep mode, the LSE or reference clock on the gpios should be used as SYNC signal. */
extern const field32_t CRS_CFGR_SYNCPOL;  /** @brief SYNC polarity selection this bit is set and cleared by software to select the input polarity for the SYNC signal source. */
extern const field32_t CRS_ISR_SYNCOKF;   /** @brief SYNC event OK flag this flag is set by hardware when the measured frequency error is smaller than FELIM * 3. This means that either no adjustment of the TRIM value is needed or that an adjustment by one trimming step is enough to compensate the frequency error. An interrupt is generated if the SYNCOKIE bit is set in the CRS_CR register. It is cleared by software by setting the SYNCOKC bit in the CRS_ICR register. */
extern const field32_t CRS_ISR_SYNCWARNF; /** @brief SYNC warning flag this flag is set by hardware when the measured frequency error is greater than or equal to FELIM * 3, but smaller than FELIM * 128. This means that to compensate the frequency error, the TRIM value must be adjusted by two steps or more. An interrupt is generated if the SYNCWARNIE bit is set in the CRS_CR register. It is cleared by software by setting the SYNCWARNC bit in the CRS_ICR register. */
extern const field32_t CRS_ISR_ERRF;      /** @brief Error flag this flag is set by hardware in case of any synchronization or trimming error. It is the logical OR of the TRIMOVF, SYNCMISS and SYNCERR bits. An interrupt is generated if the ERRIE bit is set in the CRS_CR register. It is cleared by software in reaction to setting the ERRC bit in the CRS_ICR register, which clears the TRIMOVF, SYNCMISS and SYNCERR bits. */
extern const field32_t CRS_ISR_ESYNCF;    /** @brief Expected SYNC flag this flag is set by hardware when the frequency error counter reached a zero value. An interrupt is generated if the ESYNCIE bit is set in the CRS_CR register. It is cleared by software by setting the ESYNCC bit in the CRS_ICR register. */
extern const field32_t CRS_ISR_SYNCERR;   /** @brief SYNC error this flag is set by hardware when the SYNC pulse arrives before the ESYNC event and the measured frequency error is greater than or equal to FELIM * 128. This means that the frequency error is too big (internal frequency too low) to be compensated by adjusting the TRIM value, and that some other action should be taken. An interrupt is generated if the ERRIE bit is set in the CRS_CR register. It is cleared by software by setting the ERRC bit in the CRS_ICR register. */
extern const field32_t CRS_ISR_SYNCMISS;  /** @brief SYNC missed this flag is set by hardware when the frequency error counter reached value FELIM * 128 and no SYNC was detected, meaning either that a SYNC pulse was missed or that the frequency error is too big (internal frequency too high) to be compensated by adjusting the TRIM value, and that some other action should be taken. At this point, the frequency error counter is stopped (waiting for a next SYNC) and an interrupt is generated if the ERRIE bit is set in the CRS_CR register. It is cleared by software by setting the ERRC bit in the CRS_ICR register. */
extern const field32_t CRS_ISR_TRIMOVF;   /** @brief Trimming overflow or underflow this flag is set by hardware when the automatic trimming tries to over- or under-flow the TRIM value. An interrupt is generated if the ERRIE bit is set in the CRS_CR register. It is cleared by software by setting the ERRC bit in the CRS_ICR register. */
extern const field32_t CRS_ISR_FEDIR;     /** @brief Frequency error direction FEDIR is the counting direction of the frequency error counter latched in the time of the last SYNC event. It shows whether the actual frequency is below or above the target. */
extern const field32_t CRS_ISR_FECAP;     /** @brief Frequency error capture FECAP is the frequency error counter value latched in the time of the last SYNC event. Refer to section7.3.4: frequency error evaluation and automatic trimming for more details about FECAP usage. */
extern const field32_t CRS_ICR_SYNCOKC;   /** @brief SYNC event OK clear flag writing 1 to this bit clears the SYNCOKF flag in the CRS_ISR register. */
extern const field32_t CRS_ICR_SYNCWARNC; /** @brief SYNC warning clear flag writing 1 to this bit clears the SYNCWARNF flag in the CRS_ISR register. */
extern const field32_t CRS_ICR_ERRC;      /** @brief Error clear flag writing 1 to this bit clears TRIMOVF, SYNCMISS and SYNCERR bits and consequently also the ERRF flag in the CRS_ISR register. */
extern const field32_t CRS_ICR_ESYNCC;    /** @brief Expected SYNC clear flag writing 1 to this bit clears the ESYNCF flag in the CRS_ISR register. */

/**************************************************************************************************
 * @section DAC Definitions
 **************************************************************************************************/

/** @subsection DAC Register Definitions */

extern rw_reg32_t const DAC_CR;      /** @brief DAC control register. */
extern rw_reg32_t const DAC_SWTRGR;  /** @brief DAC software trigger register. */
extern rw_reg32_t const DAC_DHR12R1; /** @brief DAC channel1 12-bit right-aligned data holding register. */
extern rw_reg32_t const DAC_DHR12L1; /** @brief DAC channel1 12-bit left aligned data holding register. */
extern rw_reg32_t const DAC_DHR8R1;  /** @brief DAC channel1 8-bit right aligned data holding register. */
extern rw_reg32_t const DAC_DHR12R2; /** @brief DAC channel2 12-bit right aligned data holding register. */
extern rw_reg32_t const DAC_DHR12L2; /** @brief DAC channel2 12-bit left aligned data holding register. */
extern rw_reg32_t const DAC_DHR8R2;  /** @brief DAC channel2 8-bit right-aligned data holding register. */
extern rw_reg32_t const DAC_DHR12RD; /** @brief Dual DAC 12-bit right-aligned data holding register. */
extern rw_reg32_t const DAC_DHR12LD; /** @brief DUAL DAC 12-bit left aligned data holding register. */
extern rw_reg32_t const DAC_DHR8RD;  /** @brief DUAL DAC 8-bit right aligned data holding register. */
extern ro_reg32_t const DAC_DOR1;    /** @brief DAC channel1 data output register. */
extern ro_reg32_t const DAC_DOR2;    /** @brief DAC channel2 data output register. */
extern rw_reg32_t const DAC_SR;      /** @brief DAC status register. */
extern rw_reg32_t const DAC_CCR;     /** @brief DAC calibration control register. */
extern rw_reg32_t const DAC_MCR;     /** @brief DAC mode control register. */
extern rw_reg32_t const DAC_SHSR1;   /** @brief DAC sample and hold sample time register 1. */
extern rw_reg32_t const DAC_SHSR2;   /** @brief DAC sample and hold sample time register 2. */
extern rw_reg32_t const DAC_SHHR;    /** @brief DAC sample and hold hold time register. */
extern rw_reg32_t const DAC_SHRR;    /** @brief DAC sample and hold refresh time register. */

/** @subsection DAC Register Field Definitions */

extern const field32_t DAC_DHR12R1_DACC1DHR; /** @brief DAC channel1 12-bit right-aligned data these bits are written by software which specifies 12-bit data for DAC channel1. */
extern const field32_t DAC_DHR12L1_DACC1DHR; /** @brief DAC channel1 12-bit left-aligned data these bits are written by software which specifies 12-bit data for DAC channel1. */
extern const field32_t DAC_DHR8R1_DACC1DHR;  /** @brief DAC channel1 8-bit right-aligned data these bits are written by software which specifies 8-bit data for DAC channel1. */
extern const field32_t DAC_DHR12R2_DACC2DHR; /** @brief DAC channel2 12-bit right-aligned data these bits are written by software which specifies 12-bit data for DAC channel2. */
extern const field32_t DAC_DHR12L2_DACC2DHR; /** @brief DAC channel2 12-bit left-aligned data these bits are written by software which specify 12-bit data for DAC channel2. */
extern const field32_t DAC_DHR8R2_DACC2DHR;  /** @brief DAC channel2 8-bit right-aligned data these bits are written by software which specifies 8-bit data for DAC channel2. */
extern const field32_t DAC_DOR1_DACC1DOR;    /** @brief DAC channel1 data output these bits are read-only, they contain data output for DAC channel1. */
extern const field32_t DAC_DOR2_DACC2DOR;    /** @brief DAC channel2 data output these bits are read-only, they contain data output for DAC channel2. */
extern const field32_t DAC_SHSR1_TSAMPLE1;   /** @brief DAC channel 1 sample time (only valid in sample &amp; hold mode) these bits can be written when the DAC channel1 is disabled or also during normal operation. In the latter case, the write can be done only when bwstx of DAC_SR register is low, if bwstx=1, the write operation is ignored. */
extern const field32_t DAC_SHSR2_TSAMPLE2;   /** @brief DAC channel 2 sample time (only valid in sample &amp; hold mode) these bits can be written when the DAC channel2 is disabled or also during normal operation. In the latter case, the write can be done only when bwstx of DAC_SR register is low, if bwstx=1, the write operation is ignored. */

/** @subsection Enumerated DAC Register Field Definitions */

extern const field32_t DAC_CR_ENx[3];           /** @brief DAC channel1 enable this bit is set and cleared by software to enable/disable DAC channel1. */
extern const field32_t DAC_CR_TENx[3];          /** @brief DAC channel1 trigger enable. */
extern const field32_t DAC_CR_TSELx[3];         /** @brief DAC channel1 trigger selection these bits select the external event used to trigger DAC channel1. Note: only used if bit TEN1 = 1 (DAC channel1 trigger enabled). */
extern const field32_t DAC_CR_WAVEx[3];         /** @brief DAC channel1 noise/triangle wave generation enable these bits are set and cleared by software. Note: only used if bit TEN1 = 1 (DAC channel1 trigger enabled). */
extern const field32_t DAC_CR_MAMPx[3];         /** @brief DAC channel1 mask/amplitude selector these bits are written by software to select mask in wave generation mode or amplitude in triangle generation mode. = 1011: unmask bits[11:0] of LFSR/ triangle amplitude equal to 4095. */
extern const field32_t DAC_CR_DMAENx[3];        /** @brief DAC channel1 DMA enable this bit is set and cleared by software. */
extern const field32_t DAC_CR_DMAUDRIEx[3];     /** @brief DAC channel1 DMA underrun interrupt enable this bit is set and cleared by software. */
extern const field32_t DAC_CR_CENx[3];          /** @brief DAC channel 1 calibration enable this bit is set and cleared by software to enable/disable DAC channel 1 calibration, it can be written only if bit EN1=0 into DAC_CR (the calibration mode can be entered/exit only when the DAC channel is disabled) otherwise, the write operation is ignored. */
extern const field32_t DAC_SWTRGR_SWTRIGx[3];   /** @brief DAC channel1 software trigger this bit is set by software to trigger the DAC in software trigger mode. Note: this bit is cleared by hardware (one APB1 clock cycle later) once the DAC_DHR1 register value has been loaded into the DAC_DOR1 register. */
extern const field32_t DAC_DHR12RD_DACCxDHR[3]; /** @brief DAC channel1 12-bit right-aligned data these bits are written by software which specifies 12-bit data for DAC channel1. */
extern const field32_t DAC_SR_DMAUDRx[3];       /** @brief DAC channel1 DMA underrun flag this bit is set by hardware and cleared by software (by writing it to 1). */
extern const field32_t DAC_SR_CAL_FLAGx[3];     /** @brief DAC channel 1 calibration offset status this bit is set and cleared by hardware. */
extern const field32_t DAC_SR_BWSTx[3];         /** @brief DAC channel 1 busy writing sample time flag this bit is systematically set just after sample & hold mode enable and is set each time the software writes the register DAC_SHSR1, it is cleared by hardware when the write operation of DAC_SHSR1 is complete. (It takes about 3LSI periods of synchronization). */
extern const field32_t DAC_CCR_OTRIMx[3];       /** @brief DAC channel 1 offset trimming value. */
extern const field32_t DAC_MCR_MODEx[3];        /** @brief DAC channel 1 mode these bits can be written only when the DAC is disabled and not in the calibration mode (when bit EN1=0 and bit CEN1 =0 in the DAC_CR register). If EN1=1 or CEN1 =1 the write operation is ignored. They can be set and cleared by software to select the DAC channel 1 mode: DAC channel 1 in normal mode DAC channel 1 in sample &amp; hold mode. */
extern const field32_t DAC_SHHR_THOLDx[3];      /** @brief DAC channel 1 hold time (only valid in sample &amp; hold mode) hold time= (THOLD[9:0]) x T LSI. */
extern const field32_t DAC_SHRR_TREFRESHx[3];   /** @brief DAC channel 1 refresh time (only valid in sample &amp; hold mode) refresh time= (TREFRESH[7:0]) x T LSI. */

/**************************************************************************************************
 * @section BDMA Definitions
 **************************************************************************************************/

/** @subsection BDMA Register Definitions */

extern ro_reg32_t const BDMA_ISR;  /** @brief DMA interrupt status register. */
extern rw_reg32_t const BDMA_IFCR; /** @brief DMA interrupt flag clear register. */

/** @subsection Enumerated BDMA Register Definitions */

extern rw_reg32_t const BDMA_CCRx[9];   /** @brief DMA channel x configuration register. */
extern rw_reg32_t const BDMA_CNDTRx[9]; /** @brief DMA channel x number of data register. */
extern rw_reg32_t const BDMA_CPARx[9];  /** @brief This register must not be written when the channel is enabled. */
extern rw_reg32_t const BDMA_CMARx[9];  /** @brief This register must not be written when the channel is enabled. */

/** @subsection BDMA Register Field Definitions */

extern const field32_t BDMA_CCRx_EN;      /** @brief Channel enable this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_TCIE;    /** @brief Transfer complete interrupt enable this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_HTIE;    /** @brief Half transfer interrupt enable this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_TEIE;    /** @brief Transfer error interrupt enable this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_DIR;     /** @brief Data transfer direction this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_CIRC;    /** @brief Circular mode this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_PINC;    /** @brief Peripheral increment mode this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_MINC;    /** @brief Memory increment mode this bit is set and cleared by software. */
extern const field32_t BDMA_CCRx_PSIZE;   /** @brief Peripheral size these bits are set and cleared by software. */
extern const field32_t BDMA_CCRx_MSIZE;   /** @brief Memory size these bits are set and cleared by software. */
extern const field32_t BDMA_CCRx_PL;      /** @brief Channel priority level these bits are set and cleared by software. */
extern const field32_t BDMA_CCRx_MEM2MEM; /** @brief Memory to memory mode this bit is set and cleared by software. */
extern const field32_t BDMA_CNDTRx_NDT;   /** @brief Number of data to transfer number of data to be transferred (0 up to 65535). This register can only be written when the channel is disabled. Once the channel is enabled, this register is read-only, indicating the remaining bytes to be transmitted. This register decrements after each DMA transfer. Once the transfer is completed, this register can either stay at zero or be reloaded automatically by the value previously programmed if the channel is configured in auto-reload mode. If this register is zero, no transaction can be served whether the channel is enabled or not. */

/** @subsection Enumerated BDMA Register Field Definitions */

extern const field32_t BDMA_ISR_GIFx[9];    /** @brief Channel x global interrupt flag (x = 1..8) this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the DMA_IFCR register. */
extern const field32_t BDMA_ISR_TCIFx[9];   /** @brief Channel x transfer complete flag (x = 1..8) this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the DMA_IFCR register. */
extern const field32_t BDMA_ISR_HTIFx[9];   /** @brief Channel x half transfer flag (x = 1..8) this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the DMA_IFCR register. */
extern const field32_t BDMA_ISR_TEIFx[9];   /** @brief Channel x transfer error flag (x = 1..8) this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the DMA_IFCR register. */
extern const field32_t BDMA_IFCR_CGIFx[9];  /** @brief Channel x global interrupt clear this bit is set and cleared by software. */
extern const field32_t BDMA_IFCR_CTCIFx[9]; /** @brief Channel x transfer complete clear this bit is set and cleared by software. */
extern const field32_t BDMA_IFCR_CHTIFx[9]; /** @brief Channel x half transfer clear this bit is set and cleared by software. */
extern const field32_t BDMA_IFCR_CTEIFx[9]; /** @brief Channel x transfer error clear this bit is set and cleared by software. */

/**************************************************************************************************
 * @section DMA2D Definitions
 **************************************************************************************************/

/** @subsection DMA2D Register Definitions */

extern rw_reg32_t const DMA2D_CR;      /** @brief DMA2D control register. */
extern ro_reg32_t const DMA2D_ISR;     /** @brief DMA2D interrupt status register. */
extern rw_reg32_t const DMA2D_IFCR;    /** @brief DMA2D interrupt flag clear register. */
extern rw_reg32_t const DMA2D_FGMAR;   /** @brief DMA2D foreground memory address register. */
extern rw_reg32_t const DMA2D_FGOR;    /** @brief DMA2D foreground offset register. */
extern rw_reg32_t const DMA2D_BGMAR;   /** @brief DMA2D background memory address register. */
extern rw_reg32_t const DMA2D_BGOR;    /** @brief DMA2D background offset register. */
extern rw_reg32_t const DMA2D_FGPFCCR; /** @brief DMA2D foreground PFC control register. */
extern rw_reg32_t const DMA2D_FGCOLR;  /** @brief DMA2D foreground color register. */
extern rw_reg32_t const DMA2D_BGPFCCR; /** @brief DMA2D background PFC control register. */
extern rw_reg32_t const DMA2D_BGCOLR;  /** @brief DMA2D background color register. */
extern rw_reg32_t const DMA2D_FGCMAR;  /** @brief DMA2D foreground CLUT memory address register. */
extern rw_reg32_t const DMA2D_BGCMAR;  /** @brief DMA2D background CLUT memory address register. */
extern rw_reg32_t const DMA2D_OPFCCR;  /** @brief DMA2D output PFC control register. */
extern rw_reg32_t const DMA2D_OCOLR;   /** @brief DMA2D output color register. */
extern rw_reg32_t const DMA2D_OMAR;    /** @brief DMA2D output memory address register. */
extern rw_reg32_t const DMA2D_OOR;     /** @brief DMA2D output offset register. */
extern rw_reg32_t const DMA2D_NLR;     /** @brief DMA2D number of line register. */
extern rw_reg32_t const DMA2D_LWR;     /** @brief DMA2D line watermark register. */
extern rw_reg32_t const DMA2D_AMTCR;   /** @brief DMA2D AXI master timer configuration register. */

/** @subsection DMA2D Register Field Definitions */

extern const field32_t DMA2D_CR_START;      /** @brief Start this bit can be used to launch the DMA2D according to the parameters loaded in the various configuration registers. */
extern const field32_t DMA2D_CR_SUSP;       /** @brief Suspend this bit can be used to suspend the current transfer. This bit is set and reset by software. It is automatically reset by hardware when the START bit is reset. */
extern const field32_t DMA2D_CR_ABORT;      /** @brief Abort this bit can be used to abort the current transfer. This bit is set by software and is automatically reset by hardware when the START bit is reset. */
extern const field32_t DMA2D_CR_TEIE;       /** @brief Transfer error interrupt enable this bit is set and cleared by software. */
extern const field32_t DMA2D_CR_TCIE;       /** @brief Transfer complete interrupt enable this bit is set and cleared by software. */
extern const field32_t DMA2D_CR_TWIE;       /** @brief Transfer watermark interrupt enable this bit is set and cleared by software. */
extern const field32_t DMA2D_CR_CAEIE;      /** @brief CLUT access error interrupt enable this bit is set and cleared by software. */
extern const field32_t DMA2D_CR_CTCIE;      /** @brief CLUT transfer complete interrupt enable this bit is set and cleared by software. */
extern const field32_t DMA2D_CR_CEIE;       /** @brief Configuration error interrupt enable this bit is set and cleared by software. */
extern const field32_t DMA2D_CR_MODE;       /** @brief DMA2D mode this bit is set and cleared by software. It cannot be modified while a transfer is ongoing. */
extern const field32_t DMA2D_ISR_TEIF;      /** @brief Transfer error interrupt flag this bit is set when an error occurs during a DMA transfer (data transfer or automatic CLUT loading). */
extern const field32_t DMA2D_ISR_TCIF;      /** @brief Transfer complete interrupt flag this bit is set when a DMA2D transfer operation is complete (data transfer only). */
extern const field32_t DMA2D_ISR_TWIF;      /** @brief Transfer watermark interrupt flag this bit is set when the last pixel of the watermarked line has been transferred. */
extern const field32_t DMA2D_ISR_CAEIF;     /** @brief CLUT access error interrupt flag this bit is set when the CPU accesses the CLUT while the CLUT is being automatically copied from a system memory to the internal DMA2D. */
extern const field32_t DMA2D_ISR_CTCIF;     /** @brief CLUT transfer complete interrupt flag this bit is set when the CLUT copy from a system memory area to the internal DMA2D memory is complete. */
extern const field32_t DMA2D_ISR_CEIF;      /** @brief Configuration error interrupt flag this bit is set when the START bit of DMA2D_CR, DMA2DFGPFCCR or DMA2D_BGPFCCR is set and a wrong configuration has been programmed. */
extern const field32_t DMA2D_IFCR_CTEIF;    /** @brief Clear transfer error interrupt flag programming this bit to 1 clears the TEIF flag in the DMA2D_ISR register. */
extern const field32_t DMA2D_IFCR_CTCIF;    /** @brief Clear transfer complete interrupt flag programming this bit to 1 clears the TCIF flag in the DMA2D_ISR register. */
extern const field32_t DMA2D_IFCR_CTWIF;    /** @brief Clear transfer watermark interrupt flag programming this bit to 1 clears the TWIF flag in the DMA2D_ISR register. */
extern const field32_t DMA2D_IFCR_CAECIF;   /** @brief Clear CLUT access error interrupt flag programming this bit to 1 clears the CAEIF flag in the DMA2D_ISR register. */
extern const field32_t DMA2D_IFCR_CCTCIF;   /** @brief Clear CLUT transfer complete interrupt flag programming this bit to 1 clears the CTCIF flag in the DMA2D_ISR register. */
extern const field32_t DMA2D_IFCR_CCEIF;    /** @brief Clear configuration error interrupt flag programming this bit to 1 clears the CEIF flag in the DMA2D_ISR register. */
extern const field32_t DMA2D_FGOR_LO;       /** @brief Line offset line offset used for the foreground expressed in pixel. This value is used to generate the address. It is added at the end of each line to determine the starting address of the next line. These bits can only be written when data transfers are disabled. Once a data transfer has started, they become read-only. If the image format is 4-bit per pixel, the line offset must be even. */
extern const field32_t DMA2D_BGOR_LO;       /** @brief Line offset line offset used for the background image (expressed in pixel). This value is used for the address generation. It is added at the end of each line to determine the starting address of the next line. These bits can only be written when data transfers are disabled. Once data transfer has started, they become read-only. If the image format is 4-bit per pixel, the line offset must be even. */
extern const field32_t DMA2D_FGPFCCR_CM;    /** @brief Color mode these bits defines the color format of the foreground image. They can only be written when data transfers are disabled. Once the transfer has started, they are read-only. Others: meaningless. */
extern const field32_t DMA2D_FGPFCCR_CCM;   /** @brief CLUT color mode this bit defines the color format of the CLUT. It can only be written when the transfer is disabled. Once the CLUT transfer has started, this bit is read-only. */
extern const field32_t DMA2D_FGPFCCR_START; /** @brief Start this bit can be set to start the automatic loading of the CLUT. It is automatically reset: ** at the end of the transfer ** when the transfer is aborted by the user application by setting the ABORT bit in DMA2D_CR ** when a transfer error occurs ** when the transfer has not started due to a configuration error or another transfer operation already ongoing (data transfer or automatic background CLUT transfer). */
extern const field32_t DMA2D_FGPFCCR_CS;    /** @brief CLUT size these bits define the size of the CLUT used for the foreground image. Once the CLUT transfer has started, this field is read-only. The number of CLUT entries is equal to CS[7:0] + 1. */
extern const field32_t DMA2D_FGPFCCR_AM;    /** @brief Alpha mode these bits select the alpha channel value to be used for the foreground image. They can only be written data the transfer are disabled. Once the transfer has started, they become read-only. Other configurations are meaningless. */
extern const field32_t DMA2D_FGPFCCR_CSS;   /** @brief Chroma sub-sampling these bits define the chroma sub-sampling mode for ycbcr color mode. Once the transfer has started, these bits are read-only. Others: meaningless. */
extern const field32_t DMA2D_FGPFCCR_AI;    /** @brief Alpha inverted this bit inverts the alpha value. Once the transfer has started, this bit is read-only. */
extern const field32_t DMA2D_FGPFCCR_RBS;   /** @brief Red blue swap this bit allows to swap the R &amp; B to support BGR or ABGR color formats. Once the transfer has started, this bit is read-only. */
extern const field32_t DMA2D_FGPFCCR_ALPHA; /** @brief Alpha value these bits define a fixed alpha channel value which can replace the original alpha value or be multiplied by the original alpha value according to the alpha mode selected through the AM[1:0] bits. These bits can only be written when data transfers are disabled. Once a transfer has started, they become read-only. */
extern const field32_t DMA2D_FGCOLR_BLUE;   /** @brief Blue value these bits defines the blue value for the A4 or A8 mode of the foreground image. They can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_FGCOLR_GREEN;  /** @brief Green value these bits defines the green value for the A4 or A8 mode of the foreground image. They can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_FGCOLR_RED;    /** @brief Red value these bits defines the red value for the A4 or A8 mode of the foreground image. They can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_BGPFCCR_CM;    /** @brief Color mode these bits define the color format of the foreground image. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. Others: meaningless. */
extern const field32_t DMA2D_BGPFCCR_CCM;   /** @brief CLUT color mode these bits define the color format of the CLUT. This register can only be written when the transfer is disabled. Once the CLUT transfer has started, this bit is read-only. */
extern const field32_t DMA2D_BGPFCCR_START; /** @brief Start this bit is set to start the automatic loading of the CLUT. This bit is automatically reset: ** at the end of the transfer ** when the transfer is aborted by the user application by setting the ABORT bit in the DMA2D_CR ** when a transfer error occurs ** when the transfer has not started due to a configuration error or another transfer operation already on going (data transfer or automatic background CLUT transfer). */
extern const field32_t DMA2D_BGPFCCR_CS;    /** @brief CLUT size these bits define the size of the CLUT used for the BG. Once the CLUT transfer has started, this field is read-only. The number of CLUT entries is equal to CS[7:0] + 1. */
extern const field32_t DMA2D_BGPFCCR_AM;    /** @brief Alpha mode these bits define which alpha channel value to be used for the background image. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. Others: meaningless. */
extern const field32_t DMA2D_BGPFCCR_AI;    /** @brief Alpha inverted this bit inverts the alpha value. Once the transfer has started, this bit is read-only. */
extern const field32_t DMA2D_BGPFCCR_RBS;   /** @brief Red blue swap this bit allows to swap the R &amp; B to support BGR or ABGR color formats. Once the transfer has started, this bit is read-only. */
extern const field32_t DMA2D_BGPFCCR_ALPHA; /** @brief Alpha value these bits define a fixed alpha channel value which can replace the original alpha value or be multiplied with the original alpha value according to the alpha mode selected with bits AM[1: 0]. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_BGCOLR_BLUE;   /** @brief Blue value these bits define the blue value for the A4 or A8 mode of the background. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_BGCOLR_GREEN;  /** @brief Green value these bits define the green value for the A4 or A8 mode of the background. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_BGCOLR_RED;    /** @brief Red value these bits define the red value for the A4 or A8 mode of the background. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_OPFCCR_CM;     /** @brief Color mode these bits define the color format of the output image. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. Others: meaningless. */
extern const field32_t DMA2D_OPFCCR_AI;     /** @brief Alpha inverted this bit inverts the alpha value. Once the transfer has started, this bit is read-only. */
extern const field32_t DMA2D_OPFCCR_RBS;    /** @brief Red blue swap this bit allows to swap the R &amp; B to support BGR or ABGR color formats. Once the transfer has started, this bit is read-only. */
extern const field32_t DMA2D_OCOLR_BLUE;    /** @brief Blue value these bits define the blue value of the output image. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_OCOLR_GREEN;   /** @brief Green value these bits define the green value of the output image. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_OCOLR_RED;     /** @brief Red value these bits define the red value of the output image. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_OCOLR_ALPHA;   /** @brief Alpha channel value these bits define the alpha channel of the output color. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_OOR_LO;        /** @brief Line offset line offset used for the output (expressed in pixels). This value is used for the address generation. It is added at the end of each line to determine the starting address of the next line. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_NLR_NL;        /** @brief Number of lines number of lines of the area to be transferred. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_NLR_PL;        /** @brief Pixel per lines number of pixels per lines of the area to be transferred. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. If any of the input image format is 4-bit per pixel, pixel per lines must be even. */
extern const field32_t DMA2D_LWR_LW;        /** @brief Line watermark these bits allow to configure the line watermark for interrupt generation. An interrupt is raised when the last pixel of the watermarked line has been transferred. These bits can only be written when data transfers are disabled. Once the transfer has started, they are read-only. */
extern const field32_t DMA2D_AMTCR_EN;      /** @brief Enable enables the dead time functionality. */
extern const field32_t DMA2D_AMTCR_DT;      /** @brief Dead time dead time value in the AXI clock cycle inserted between two consecutive accesses on the AXI master port. These bits represent the minimum guaranteed number of cycles between two consecutive AXI accesses. */

/**************************************************************************************************
 * @section DMAMUXx Definitions
 **************************************************************************************************/

/** @subsection Enumerated DMAMUXx Register Definitions */

extern rw_reg32_t const DMAMUXx_CxCR[3][8];  /** @brief DMAMux - DMA request line multiplexer channel x control register. */
extern rw_reg32_t const DMAMUXx_RGxCR[3][8]; /** @brief DMAMux - DMA request generator channel x control register. */
extern ro_reg32_t const DMAMUXx_RGSR[3];     /** @brief DMAMux - DMA request generator status register. */
extern rw_reg32_t const DMAMUXx_RGCFR[3];    /** @brief DMAMux - DMA request generator clear flag register. */
extern ro_reg32_t const DMAMUXx_CSR[3];      /** @brief DMAMUX request line multiplexer interrupt channel status register. */
extern rw_reg32_t const DMAMUXx_CFR[3];      /** @brief DMAMUX request line multiplexer interrupt clear flag register. */

/** @subsection DMAMUXx Register Field Definitions */

extern const field32_t DMAMUXx_CxCR_DMAREQ_ID; /** @brief Input DMA request line selected. */
extern const field32_t DMAMUXx_CxCR_SOIE;      /** @brief Interrupt enable at synchronization event overrun. */
extern const field32_t DMAMUXx_CxCR_EGE;       /** @brief Event generation enable/disable. */
extern const field32_t DMAMUXx_CxCR_SE;        /** @brief Synchronous operating mode enable/disable. */
extern const field32_t DMAMUXx_CxCR_SPOL;      /** @brief Synchronization event type selector defines the synchronization event on the selected synchronization input:. */
extern const field32_t DMAMUXx_CxCR_NBREQ;     /** @brief Number of DMA requests to forward defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: this field can only be written when both SE and EGE bits are reset. */
extern const field32_t DMAMUXx_CxCR_SYNC_ID;   /** @brief Synchronization input selected. */
extern const field32_t DMAMUXx_RGxCR_SIG_ID;   /** @brief DMA request trigger input selected. */
extern const field32_t DMAMUXx_RGxCR_OIE;      /** @brief Interrupt enable at trigger event overrun. */
extern const field32_t DMAMUXx_RGxCR_GE;       /** @brief DMA request generator channel enable/disable. */
extern const field32_t DMAMUXx_RGxCR_GPOL;     /** @brief DMA request generator trigger event type selection defines the trigger event on the selected DMA request trigger input. */
extern const field32_t DMAMUXx_RGxCR_GNBREQ;   /** @brief Number of DMA requests to generate defines the number of DMA requests generated after a trigger event, then stop generating. The actual number of generated DMA requests is GNBREQ+1. Note: this field can only be written when GE bit is reset. */
extern const field32_t DMAMUXx_RGSR_OF;        /** @brief Trigger event overrun flag the flag is set when a trigger event occurs on DMA request generator channel x, while the DMA request generator counter value is lower than GNBREQ. The flag is cleared by writing 1 to the corresponding cofx bit in DMAMUX_RGCFR register. */
extern const field32_t DMAMUXx_RGCFR_COF;      /** @brief Clear trigger event overrun flag upon setting, this bit clears the corresponding overrun flag ofx in the DMAMUX_RGCSR register. */
extern const field32_t DMAMUXx_CSR_SOF;        /** @brief Synchronization overrun event flag. */
extern const field32_t DMAMUXx_CFR_CSOF;       /** @brief Clear synchronization overrun event flag. */

/**************************************************************************************************
 * @section FMC Definitions
 **************************************************************************************************/

/** @subsection FMC Register Definitions */

extern rw_reg32_t const FMC_PCR;   /** @brief NAND flash control registers. */
extern rw_reg32_t const FMC_SR;    /** @brief This register contains information about the FIFO status and interrupt. The FMC features a FIFO that is used when writing to memories to transfer up to 16 words of data.this is used to quickly write to the FIFO and free the AXI bus for transactions to peripherals other than the FMC, while the FMC is draining its FIFO into the memory. One of these register bits indicates the status of the FIFO, for ECC purposes.the ECC is calculated while the data are written to the memory. To read the correct ECC, the software must consequently wait until the FIFO is empty. */
extern rw_reg32_t const FMC_PMEM;  /** @brief The FMC_PMEM read/write register contains the timing information for NAND flash memory bank. This information is used to access either the common memory space of the NAND flash for command, address write access and data read/write access. */
extern rw_reg32_t const FMC_PATT;  /** @brief The FMC_PATT read/write register contains the timing information for NAND flash memory bank. It is used for 8-bit accesses to the attribute memory space of the NAND flash for the last address write access if the timing must differ from that of previous accesses (for ready/busy management, refer to section20.8.5: NAND flash prewait feature). */
extern ro_reg32_t const FMC_ECCR;  /** @brief This register contain the current error correction code value computed by the ECC computation modules of the FMC NAND controller. When the CPU reads/writes the data from a NAND flash memory page at the correct address (refer to section20.8.6: computation of the error correction code (ECC) in NAND flash memory), the data read/written from/to the NAND flash memory are processed automatically by the ECC computation module. When X bytes have been read (according to the ECCPS field in the FMC_PCR registers), the CPU must read the computed ECC value from the FMC_ECC registers. It then verifies if these computed parity data are the same as the parity value recorded in the spare area, to determine whether a page is valid, and, to correct it otherwise. The FMC_ECCR register should be cleared after being read by setting the ECCEN bit to 0. To compute a new data block, the ECCEN bit must be set to 1. */
extern rw_reg32_t const FMC_SDCMR; /** @brief This register contains the command issued when the SDRAM device is accessed. This register is used to initialize the SDRAM device, and to activate the self-refresh and the power-down modes. As soon as the MODE field is written, the command will be issued only to one or to both SDRAM banks according to CTB1 and CTB2 command bits. This register is the same for both SDRAM banks. */
extern rw_reg32_t const FMC_SDRTR; /** @brief This register sets the refresh rate in number of SDCLK clock cycles between the refresh cycles by configuring the refresh timer count value.examplewhere 64 ms is the SDRAM refresh period.the refresh rate must be increased by 20 SDRAM clock cycles (as in the above example) to obtain a safe margin if an internal refresh request occurs when a read request has been accepted. It corresponds to a COUNT value of 0000111000000 (448). This 13-bit field is loaded into a timer which is decremented using the SDRAM clock. This timer generates a refresh pulse when zero is reached. The COUNT value must be set at least to 41 SDRAM clock cycles.as soon as the FMC_SDRTR register is programmed, the timer starts counting. If the value programmed in the register is 0, no refresh is carried out. This register must not be reprogrammed after the initialization procedure to avoid modifying the refresh rate.each time a refresh pulse is generated, this 13-bit COUNT field is reloaded into the counter.if a memory access is in progress, the auto-refresh request is delayed. However, if the memory access and auto-refresh requests are generated simultaneously, the auto-refresh takes precedence. If the memory access occurs during a refresh operation, the request is buffered to be processed when the refresh is complete.this register is common to SDRAM bank 1 and bank 2. */
extern ro_reg32_t const FMC_SDSR;  /** @brief SDRAM status register. */

/** @subsection Enumerated FMC Register Definitions */

extern rw_reg32_t const FMC_BCRx[5];  /** @brief This register contains the control information of each memory bank, used for srams, PSRAM and NOR flash memories. */
extern rw_reg32_t const FMC_BTRx[5];  /** @brief This register contains the control information of each memory bank, used for srams, PSRAM and NOR flash memories.if the EXTMOD bit is set in the fmc_bcrx register, then this register is partitioned for write and read access, that is, 2 registers are available: one to configure read accesses (this register) and one to configure write accesses (fmc_bwtrx registers). */
extern rw_reg32_t const FMC_BWTRx[5]; /** @brief This register contains the control information of each memory bank. It is used for srams, psrams and NOR flash memories. When the EXTMOD bit is set in the fmc_bcrx register, then this register is active for write access. */
extern rw_reg32_t const FMC_SDCRx[3]; /** @brief This register contains the control parameters for each SDRAM memory bank. */
extern rw_reg32_t const FMC_SDTRx[3]; /** @brief This register contains the timing parameters of each SDRAM bank. */

/** @subsection FMC Register Field Definitions */

extern const field32_t FMC_BCRx_MBKEN;     /** @brief Memory bank enable bit this bit enables the memory bank. After reset bank1 is enabled, all others are disabled. Accessing a disabled bank causes an ERROR on AXI bus. */
extern const field32_t FMC_BCRx_MUXEN;     /** @brief Address/data multiplexing enable bit when this bit is set, the address and data values are multiplexed on the data bus, valid only with NOR and PSRAM memories:. */
extern const field32_t FMC_BCRx_MTYP;      /** @brief Memory type these bits define the type of external memory attached to the corresponding memory bank:. */
extern const field32_t FMC_BCRx_MWID;      /** @brief Memory data bus width defines the external memory device width, valid for all type of memories. */
extern const field32_t FMC_BCRx_FACCEN;    /** @brief Flash access enable this bit enables NOR flash memory access operations. */
extern const field32_t FMC_BCRx_BURSTEN;   /** @brief Burst enable bit this bit enables/disables synchronous accesses during read operations. It is valid only for synchronous memories operating in burst mode:. */
extern const field32_t FMC_BCRx_WAITPOL;   /** @brief Wait signal polarity bit this bit defines the polarity of the wait signal from memory used for either in synchronous or asynchronous mode:. */
extern const field32_t FMC_BCRx_WAITCFG;   /** @brief Wait timing configuration the NWAIT signal indicates whether the data from the memory are valid or if a wait state must be inserted when accessing the memory in synchronous mode. This configuration bit determines if NWAIT is asserted by the memory one clock cycle before the wait state or during the wait state:. */
extern const field32_t FMC_BCRx_WREN;      /** @brief Write enable bit this bit indicates whether write operations are enabled/disabled in the bank by the FMC:. */
extern const field32_t FMC_BCRx_WAITEN;    /** @brief Wait enable bit this bit enables/disables wait-state insertion via the NWAIT signal when accessing the memory in synchronous mode. */
extern const field32_t FMC_BCRx_EXTMOD;    /** @brief Extended mode enable. This bit enables the FMC to program the write timings for asynchronous accesses inside the FMC_BWTR register, thus resulting in different timings for read and write operations. Note: when the extended mode is disabled, the FMC can operate in mode1 or mode2 as follows: ** mode 1 is the default mode when the SRAM/PSRAM memory type is selected (MTYP =0x0 or 0x01) ** mode 2 is the default mode when the NOR memory type is selected (MTYP = 0x10). */
extern const field32_t FMC_BCRx_ASYNCWAIT; /** @brief Wait signal during asynchronous transfers this bit enables/disables the FMC to use the wait signal even during an asynchronous protocol. */
extern const field32_t FMC_BCRx_CPSIZE;    /** @brief CRAM page size these are used for cellular RAM 1.5 which does not allow burst access to cross the address boundaries between pages. When these bits are configured, the FMC controller splits automatically the burst access when the memory page size is reached (refer to memory datasheet for page size). Other configuration: reserved. */
extern const field32_t FMC_BCRx_CBURSTRW;  /** @brief Write burst enable for PSRAM (CRAM) operating in burst mode, the bit enables synchronous accesses during write operations. The enable bit for synchronous read accesses is the BURSTEN bit in the fmc_bcrx register. */
extern const field32_t FMC_BCRx_CCLKEN;    /** @brief Continuous clock enable this bit enables the FMC_CLK clock output to external memory devices. Note: the CCLKEN bit of the FMC_BCR2..4 registers is dont care. It is only enabled through the FMC_BCR1 register. Bank 1 must be configured in synchronous mode to generate the FMC_CLK continuous clock. If CCLKEN bit is set, the FMC_CLK clock ratio is specified by CLKDIV value in the FMC_BTR1 register. CLKDIV in FMC_BWTR1 is dont care. If the synchronous mode is used and CCLKEN bit is set, the synchronous memories connected to other banks than bank 1 are clocked by the same clock (the CLKDIV value in the FMC_BTR2..4 and FMC_BWTR2..4 registers for other banks has no effect.). */
extern const field32_t FMC_BCRx_WFDIS;     /** @brief Write FIFO disable this bit disables the write FIFO used by the FMC controller. Note: the WFDIS bit of the FMC_BCR2..4 registers is dont care. It is only enabled through the FMC_BCR1 register. */
extern const field32_t FMC_BCRx_BMAP;      /** @brief FMC bank mapping these bits allows different to remap SDRAM bank2 or swap the FMC NOR/PSRAM and SDRAM banks.refer to table 10 for note: the BMAP bits of the FMC_BCR2..4 registers are dont care. It is only enabled through the FMC_BCR1 register. */
extern const field32_t FMC_BCRx_FMCEN;     /** @brief FMC controller enable this bit enables/disables the FMC controller. Note: the FMCEN bit of the FMC_BCR2..4 registers is dont care. It is only enabled through the FMC_BCR1 register. */
extern const field32_t FMC_BTRx_ADDSET;    /** @brief Address setup phase duration these bits are written by software to define the duration of the address setup phase (refer to figure81 to figure93), used in srams, roms and asynchronous NOR flash: for each access mode address setup phase duration, please refer to the respective figure (refer to figure81 to figure93). Note: in synchronous accesses, this value is dont care. In muxed mode or mode D, the minimum value for ADDSET is 1. */
extern const field32_t FMC_BTRx_ADDHLD;    /** @brief Address-hold phase duration these bits are written by software to define the duration of the address hold phase (refer to figure81 to figure93), used in mode D or multiplexed accesses: for each access mode address-hold phase duration, please refer to the respective figure (figure81 to figure93). Note: in synchronous accesses, this value is not used, the address hold phase is always 1 memory clock period duration. */
extern const field32_t FMC_BTRx_DATAST;    /** @brief Data-phase duration these bits are written by software to define the duration of the data phase (refer to figure81 to figure93), used in asynchronous accesses: for each memory type and access mode data-phase duration, please refer to the respective figure (figure81 to figure93). Example: mode1, write access, DATAST=1: data-phase duration= DATAST+1 = 2 KCK_FMC clock cycles. Note: in synchronous accesses, this value is dont care. */
extern const field32_t FMC_BTRx_BUSTURN;   /** @brief Bus turnaround phase duration these bits are written by software to add a delay at the end of a write-to-read or read-to write transaction. The programmed bus turnaround delay is inserted between an asynchronous read (in muxed or mode D) or write transaction and any other asynchronous /synchronous read/write from/to a static bank. If a read operation is performed, the bank can be the same or a different one, whereas it must be different in case of write operation to the bank, except in muxed mode or mode D. In some cases, whatever the programmed BUSTRUN values, the bus turnaround delay is fixed as follows: the bus turnaround delay is not inserted between two consecutive asynchronous write transfers to the same static memory bank except in muxed mode and mode D. There is a bus turnaround delay of 1 FMC clock cycle between: two consecutive asynchronous read transfers to the same static memory bank except for modes muxed and D. An asynchronous read to an asynchronous or synchronous write to any static bank or dynamic bank except in modes muxed and D mode. There is a bus turnaround delay of 2 FMC clock cycle between: two consecutive synchronous write operations (in burst or single mode) to the same bank. A synchronous write (burst or single) access and an asynchronous write or read transfer to or from static memory bank (the bank can be the same or a different one in case of a read operation. Two consecutive synchronous read operations (in burst or single mode) followed by any synchronous/asynchronous read or write from/to another static memory bank. There is a bus turnaround delay of 3 FMC clock cycle between: two consecutive synchronous write operations (in burst or single mode) to different static banks. A synchronous write access (in burst or single mode) and a synchronous read from the same or a different bank. The bus turnaround delay allows to match the minimum time between consecutive transactions (tehel from nex high to nex low) and the maximum time required by the memory to free the data bus after a read access (tehqz): (BUSTRUN + 1) KCK_FMC period &#8805; tehelmin and (BUSTRUN + 2)KCK_FMC period &#8805; tehqzmax if EXTMOD = 0 (BUSTRUN + 2)KCK_FMC period &#8805; max (tehelmin, tehqzmax) if EXTMOD = 126. ... */
extern const field32_t FMC_BTRx_CLKDIV;    /** @brief Clock divide ratio (for FMC_CLK signal) these bits define the period of FMC_CLK clock output signal, expressed in number of KCK_FMC cycles: in asynchronous NOR flash, SRAM or PSRAM accesses, this value is dont care. Note: refer to section20.6.5: synchronous transactions for FMC_CLK divider ratio formula). */
extern const field32_t FMC_BTRx_DATLAT;    /** @brief Data latency for synchronous memory for synchronous access with read write burst mode enabled these bits define the number of memory clock cycles. */
extern const field32_t FMC_BTRx_ACCMOD;    /** @brief Access mode these bits specify the asynchronous access modes as shown in the timing diagrams. They are taken into account only when the EXTMOD bit in the fmc_bcrx register is 1. */
extern const field32_t FMC_PCR_PWAITEN;    /** @brief Wait feature enable bit. This bit enables the wait feature for the NAND flash memory bank:. */
extern const field32_t FMC_PCR_PBKEN;      /** @brief NAND flash memory bank enable bit. This bit enables the memory bank. Accessing a disabled memory bank causes an ERROR on AXI bus. */
extern const field32_t FMC_PCR_PWID;       /** @brief Data bus width. These bits define the external memory device width. */
extern const field32_t FMC_PCR_ECCEN;      /** @brief ECC computation logic enable bit. */
extern const field32_t FMC_PCR_TCLR;       /** @brief CLE to RE delay. These bits set time from CLE low to RE low in number of KCK_FMC clock cycles. The time is give by the following formula: t_clr = (TCLR + SET + 2) TKCK_FMC where TKCK_FMC is the KCK_FMC clock period note: set is MEMSET or ATTSET according to the addressed space. */
extern const field32_t FMC_PCR_TAR;        /** @brief ALE to RE delay. These bits set time from ALE low to RE low in number of KCK_FMC clock cycles. Time is: t_ar = (TAR + SET + 2) TKCK_FMC where TKCK_FMC is the FMC clock period note: set is MEMSET or ATTSET according to the addressed space. */
extern const field32_t FMC_PCR_ECCPS;      /** @brief ECC page size. These bits define the page size for the extended ECC:. */
extern const field32_t FMC_SR_IRS;         /** @brief Interrupt rising edge status the flag is set by hardware and reset by software. Note: if this bit is written by software to 1 it will be set. */
extern const field32_t FMC_SR_ILS;         /** @brief Interrupt high-level status the flag is set by hardware and reset by software. */
extern const field32_t FMC_SR_IFS;         /** @brief Interrupt falling edge status the flag is set by hardware and reset by software. Note: if this bit is written by software to 1 it will be set. */
extern const field32_t FMC_SR_IREN;        /** @brief Interrupt rising edge detection enable bit. */
extern const field32_t FMC_SR_ILEN;        /** @brief Interrupt high-level detection enable bit. */
extern const field32_t FMC_SR_IFEN;        /** @brief Interrupt falling edge detection enable bit. */
extern const field32_t FMC_SR_FEMPT;       /** @brief FIFO empty. Read-only bit that provides the status of the FIFO. */
extern const field32_t FMC_PMEM_MEMSET;    /** @brief Common memory x setup time these bits define the number of KCK_FMC (+1) clock cycles to set up the address before the command assertion (NWE, NOE), for NAND flash read or write access to common memory space:. */
extern const field32_t FMC_PMEM_MEMWAIT;   /** @brief Common memory wait time these bits define the minimum number of KCK_FMC (+1) clock cycles to assert the command (NWE, NOE), for NAND flash read or write access to common memory space. The duration of command assertion is extended if the wait signal (NWAIT) is active (low) at the end of the programmed value of KCK_FMC:. */
extern const field32_t FMC_PMEM_MEMHOLD;   /** @brief Common memory hold time these bits define the number of KCK_FMC clock cycles for write accesses and KCK_FMC+1 clock cycles for read accesses during which the address is held (and data for write accesses) after the command is de-asserted (NWE, NOE), for NAND flash read or write access to common memory space:. */
extern const field32_t FMC_PMEM_MEMHIZ;    /** @brief Common memory x data bus hi-z time these bits define the number of KCK_FMC clock cycles during which the data bus is kept hi-z after the start of a NAND flash write access to common memory space. This is only valid for write transactions:. */
extern const field32_t FMC_PATT_ATTSET;    /** @brief Attribute memory setup time these bits define the number of KCK_FMC (+1) clock cycles to set up address before the command assertion (NWE, NOE), for NAND flash read or write access to attribute memory space:. */
extern const field32_t FMC_PATT_ATTWAIT;   /** @brief Attribute memory wait time these bits define the minimum number of x KCK_FMC (+1) clock cycles to assert the command (NWE, NOE), for NAND flash read or write access to attribute memory space. The duration for command assertion is extended if the wait signal (NWAIT) is active (low) at the end of the programmed value of KCK_FMC:. */
extern const field32_t FMC_PATT_ATTHOLD;   /** @brief Attribute memory hold time these bits define the number of KCK_FMC clock cycles during which the address is held (and data for write access) after the command de-assertion (NWE, NOE), for NAND flash read or write access to attribute memory space:. */
extern const field32_t FMC_PATT_ATTHIZ;    /** @brief Attribute memory data bus hi-z time these bits define the number of KCK_FMC clock cycles during which the data bus is kept in hi-z after the start of a NAND flash write access to attribute memory space on socket. Only valid for writ transaction:. */
extern const field32_t FMC_BWTRx_ADDSET;   /** @brief Address setup phase duration. These bits are written by software to define the duration of the address setup phase in KCK_FMC cycles (refer to figure81 to figure93), used in asynchronous accesses: ... Note: in synchronous accesses, this value is not used, the address setup phase is always 1 flash clock period duration. In muxed mode, the minimum ADDSET value is 1. */
extern const field32_t FMC_BWTRx_ADDHLD;   /** @brief Address-hold phase duration. These bits are written by software to define the duration of the address hold phase (refer to figure81 to figure93), used in asynchronous multiplexed accesses: ... Note: in synchronous NOR flash accesses, this value is not used, the address hold phase is always 1 flash clock period duration. */
extern const field32_t FMC_BWTRx_DATAST;   /** @brief Data-phase duration. These bits are written by software to define the duration of the data phase (refer to figure81 to figure93), used in asynchronous SRAM, PSRAM and NOR flash memory accesses:. */
extern const field32_t FMC_BWTRx_BUSTURN;  /** @brief Bus turnaround phase duration these bits are written by software to add a delay at the end of a write transaction to match the minimum time between consecutive transactions (tehel from enx high to enx low): (BUSTRUN + 1) KCK_FMC period &#8805; tehelmin. The programmed bus turnaround delay is inserted between a an asynchronous write transfer and any other asynchronous /synchronous read or write transfer to or from a static bank. If a read operation is performed, the bank can be the same or a different one, whereas it must be different in case of write operation to the bank, except in muxed mode or mode D. In some cases, whatever the programmed BUSTRUN values, the bus turnaround delay is fixed as follows: the bus turnaround delay is not inserted between two consecutive asynchronous write transfers to the same static memory bank except for muxed mode and mode D. There is a bus turnaround delay of 2 FMC clock cycle between: two consecutive synchronous write operations (in burst or single mode) to the same bank A synchronous write transfer ((in burst or single mode) and an asynchronous write or read transfer to or from static memory bank. There is a bus turnaround delay of 3 FMC clock cycle between: two consecutive synchronous write operations (in burst or single mode) to different static banks. A synchronous write transfer (in burst or single mode) and a synchronous read from the same or a different bank. ... */
extern const field32_t FMC_BWTRx_ACCMOD;   /** @brief Access mode. These bits specify the asynchronous access modes as shown in the next timing diagrams.these bits are taken into account only when the EXTMOD bit in the fmc_bcrx register is 1. */
extern const field32_t FMC_SDCRx_NC;       /** @brief Number of column address bits these bits define the number of bits of a column address. */
extern const field32_t FMC_SDCRx_NR;       /** @brief Number of row address bits these bits define the number of bits of a row address. */
extern const field32_t FMC_SDCRx_MWID;     /** @brief Memory data bus width. These bits define the memory device width. */
extern const field32_t FMC_SDCRx_NB;       /** @brief Number of internal banks this bit sets the number of internal banks. */
extern const field32_t FMC_SDCRx_CAS;      /** @brief CAS latency this bits sets the SDRAM CAS latency in number of memory clock cycles. */
extern const field32_t FMC_SDCRx_WP;       /** @brief Write protection this bit enables write mode access to the SDRAM bank. */
extern const field32_t FMC_SDCRx_SDCLK;    /** @brief SDRAM clock configuration these bits define the SDRAM clock period for both SDRAM banks and allow disabling the clock before changing the frequency. In this case the SDRAM must be re-initialized. Note: the corresponding bits in the FMC_SDCR2 register is read only. */
extern const field32_t FMC_SDCRx_RBURST;   /** @brief Burst read this bit enables burst read mode. The SDRAM controller anticipates the next read commands during the CAS latency and stores data in the read FIFO. Note: the corresponding bit in the FMC_SDCR2 register is read only. */
extern const field32_t FMC_SDCRx_RPIPE;    /** @brief Read pipe these bits define the delay, in KCK_FMC clock cycles, for reading data after CAS latency. Note: the corresponding bits in the FMC_SDCR2 register is read only. */
extern const field32_t FMC_SDTRx_TMRD;     /** @brief Load mode register to active these bits define the delay between a load mode register command and an active or refresh command in number of memory clock cycles. .... */
extern const field32_t FMC_SDTRx_TXSR;     /** @brief Exit self-refresh delay these bits define the delay from releasing the self-refresh command to issuing the activate command in number of memory clock cycles. .... Note: if two SDRAM devices are used, the FMC_SDTR1 and FMC_SDTR2 must be programmed with the same TXSR timing corresponding to the slowest SDRAM device. */
extern const field32_t FMC_SDTRx_TRAS;     /** @brief Self refresh time these bits define the minimum self-refresh period in number of memory clock cycles. .... */
extern const field32_t FMC_SDTRx_TRC;      /** @brief Row cycle delay these bits define the delay between the refresh command and the activate command, as well as the delay between two consecutive refresh commands. It is expressed in number of memory clock cycles. The TRC timing is only configured in the FMC_SDTR1 register. If two SDRAM devices are used, the TRC must be programmed with the timings of the slowest device. .... Note: TRC must match the TRC and TRFC (auto refresh period) timings defined in the SDRAM device datasheet. Note: the corresponding bits in the FMC_SDTR2 register are dont care. */
extern const field32_t FMC_SDTRx_TWR;      /** @brief Recovery delay these bits define the delay between a write and a precharge command in number of memory clock cycles. .... Note: TWR must be programmed to match the write recovery time (twr) defined in the SDRAM datasheet, and to guarantee that: TWR &#8805; TRAS - TRCD and TWR &#8805;TRC - TRCD - TRP example: TRAS= 4 cycles, TRCD= 2 cycles. So, TWR &gt;= 2 cycles. TWR must be programmed to 0x1. If two SDRAM devices are used, the FMC_SDTR1 and FMC_SDTR2 must be programmed with the same TWR timing corresponding to the slowest SDRAM device. */
extern const field32_t FMC_SDTRx_TRP;      /** @brief Row precharge delay these bits define the delay between a precharge command and another command in number of memory clock cycles. The TRP timing is only configured in the FMC_SDTR1 register. If two SDRAM devices are used, the TRP must be programmed with the timing of the slowest device. .... Note: the corresponding bits in the FMC_SDTR2 register are dont care. */
extern const field32_t FMC_SDTRx_TRCD;     /** @brief Row to column delay these bits define the delay between the activate command and a read/write command in number of memory clock cycles. .... */
extern const field32_t FMC_SDCMR_MODE;     /** @brief Command mode these bits define the command issued to the SDRAM device. Note: when a command is issued, at least one command target bank bit ( CTB1 or CTB2) must be set otherwise the command will be ignored. Note: if two SDRAM banks are used, the auto-refresh and PALL command must be issued simultaneously to the two devices with CTB1 and CTB2 bits set otherwise the command will be ignored. Note: if only one SDRAM bank is used and a command is issued with its associated CTB bit set, the other CTB bit of the unused bank must be kept to 0. */
extern const field32_t FMC_SDCMR_NRFS;     /** @brief Number of auto-refresh these bits define the number of consecutive auto-refresh commands issued when MODE = 011. .... */
extern const field32_t FMC_SDCMR_MRD;      /** @brief Mode register definition this 14-bit field defines the SDRAM mode register content. The mode register is programmed using the load mode register command. The MRD[13:0] bits are also used to program the extended mode register for mobile SDRAM. */
extern const field32_t FMC_SDRTR_CRE;      /** @brief Clear refresh error flag this bit is used to clear the refresh error flag (RE) in the status register. */
extern const field32_t FMC_SDRTR_COUNT;    /** @brief Refresh timer count this 13-bit field defines the refresh rate of the SDRAM device. It is expressed in number of memory clock cycles. It must be set at least to 41 SDRAM clock cycles (0x29). Refresh rate = (COUNT + 1) x SDRAM frequency clock COUNT = (SDRAM refresh period / number of rows) - 20. */
extern const field32_t FMC_SDRTR_REIE;     /** @brief RES interrupt enable. */
extern const field32_t FMC_SDSR_RE;        /** @brief Refresh error flag an interrupt is generated if REIE = 1 and RE = 1. */

/** @subsection Enumerated FMC Register Field Definitions */

extern const field32_t FMC_SDCMR_CTBx[3];  /** @brief Command target bank 1 this bit indicates whether the command will be issued to SDRAM bank 1 or not. */
extern const field32_t FMC_SDSR_MODESx[3]; /** @brief Status mode for bank 1 these bits define the status mode of SDRAM bank 1. */

/**************************************************************************************************
 * @section CEC Definitions
 **************************************************************************************************/

/** @subsection CEC Register Definitions */

extern rw_reg32_t const CEC_CR;   /** @brief CEC control register. */
extern rw_reg32_t const CEC_CFGR; /** @brief This register is used to configure the HDMI-CEC controller. It is mandatory to write CEC_CFGR only when CECEN=0. */
extern rw_reg32_t const CEC_TXDR; /** @brief CEC tx data register. */
extern ro_reg32_t const CEC_RXDR; /** @brief CEC rx data register. */
extern rw_reg32_t const CEC_ISR;  /** @brief CEC interrupt and status register. */
extern rw_reg32_t const CEC_IER;  /** @brief CEC interrupt enable register. */

/** @subsection CEC Register Field Definitions */

extern const field32_t CEC_CR_CECEN;      /** @brief CEC enable the CECEN bit is set and cleared by software. CECEN=1 starts message reception and enables the TXSOM control. CECEN=0 disables the CEC peripheral, clears all bits of CEC_CR register and aborts any on-going reception or transmission. */
extern const field32_t CEC_CR_TXSOM;      /** @brief Tx start of message TXSOM is set by software to command transmission of the first byte of a CEC message. If the CEC message consists of only one byte, TXEOM must be set before of TXSOM. Start-Bit is effectively started on the CEC line after SFT is counted. If TXSOM is set while a message reception is ongoing, transmission will start after the end of reception. TXSOM is cleared by hardware after the last byte of the message is sent with a positive acknowledge (TXEND=1), in case of transmission underrun (TXUDR=1), negative acknowledge (TXACKE=1), and transmission error (TXERR=1). It is also cleared by CECEN=0. It is not cleared and transmission is automatically retried in case of arbitration lost (ARBLST=1). TXSOM can be also used as a status bit informing application whether any transmission request is pending or under execution. The application can abort a transmission request at any time by clearing the CECEN bit. Note: TXSOM must be set when CECEN=1 TXSOM must be set when transmission data is available into TXDR headers first four bits containing own peripheral address are taken from TXDR[7:4], not from CEC_CFGR.OAR which is used only for reception. */
extern const field32_t CEC_CR_TXEOM;      /** @brief Tx end of message the TXEOM bit is set by software to command transmission of the last byte of a CEC message. TXEOM is cleared by hardware at the same time and under the same conditions as for TXSOM. Note: TXEOM must be set when CECEN=1 TXEOM must be set before writing transmission data to TXDR if TXEOM is set when TXSOM=0, transmitted message will consist of 1 byte (HEADER) only (PING message). */
extern const field32_t CEC_CFGR_SFT;      /** @brief Signal free time SFT bits are set by software. In the sft=0x0 configuration the number of nominal data bit periods waited before transmission is ruled by hardware according to the transmission history. In all the other configurations the SFT number is determined by software. * 0x0 ** 2.5 data-bit periods if CEC is the last bus initiator with unsuccessful transmission (ARBLST=1, TXERR=1, TXUDR=1 or TXACKE= 1) ** 4 data-bit periods if CEC is the new bus initiator ** 6 data-bit periods if CEC is the last bus initiator with successful transmission (TXEOM=1) * 0x1: 0.5 nominal data bit periods * 0x2: 1.5 nominal data bit periods * 0x3: 2.5 nominal data bit periods * 0x4: 3.5 nominal data bit periods * 0x5: 4.5 nominal data bit periods * 0x6: 5.5 nominal data bit periods * 0x7: 6.5 nominal data bit periods. */
extern const field32_t CEC_CFGR_RXTOL;    /** @brief Rx-Tolerance the RXTOL bit is set and cleared by software. ** start-bit, +/- 200 s rise, +/- 200 s fall. ** data-bit: +/- 200 s rise. +/- 350 s fall. ** start-bit: +/- 400 s rise, +/- 400 s fall ** data-bit: +/-300 s rise, +/- 500 s fall. */
extern const field32_t CEC_CFGR_BRESTP;   /** @brief Rx-Stop on bit rising error the BRESTP bit is set and cleared by software. */
extern const field32_t CEC_CFGR_BREGEN;   /** @brief Generate error-bit on bit rising error the BREGEN bit is set and cleared by software. Note: if BRDNOGEN=0, an error-bit is generated upon BRE detection with BRESTP=1 in broadcast even if BREGEN=0. */
extern const field32_t CEC_CFGR_LBPEGEN;  /** @brief Generate error-bit on long bit period error the LBPEGEN bit is set and cleared by software. Note: if BRDNOGEN=0, an error-bit is generated upon LBPE detection in broadcast even if LBPEGEN=0. */
extern const field32_t CEC_CFGR_BRDNOGEN; /** @brief Avoid error-bit generation in broadcast the BRDNOGEN bit is set and cleared by software. */
extern const field32_t CEC_CFGR_SFTOPT;   /** @brief SFT option bit the SFTOPT bit is set and cleared by software. */
extern const field32_t CEC_CFGR_OAR;      /** @brief Own addresses configuration the OAR bits are set by software to select which destination logical addresses has to be considered in receive mode. Each bit, when set, enables the CEC logical address identified by the given bit position. At the end of HEADER reception, the received destination address is compared with the enabled addresses. In case of matching address, the incoming message is acknowledged and received. In case of non-matching address, the incoming message is received only in listen mode (LSTN=1), but without acknowledge sent. Broadcast messages are always received. Example: OAR = 0b000 0000 0010 0001 means that CEC acknowledges addresses 0x0 and 0x5. Consequently, each message directed to one of these addresses is received. */
extern const field32_t CEC_CFGR_LSTN;     /** @brief Listen mode LSTN bit is set and cleared by software. */
extern const field32_t CEC_TXDR_TXD;      /** @brief Tx data register. TXD is a write-only register containing the data byte to be transmitted. Note: TXD must be written when TXSTART=1. */
extern const field32_t CEC_RXDR_RXD;      /** @brief Rx data register. RXD is read-only and contains the last data byte which has been received from the CEC line. */
extern const field32_t CEC_ISR_RXBR;      /** @brief Rx-Byte received the RXBR bit is set by hardware to inform application that a new byte has been received from the CEC line and stored into the RXD buffer. RXBR is cleared by software write at 1. */
extern const field32_t CEC_ISR_RXEND;     /** @brief End of reception RXEND is set by hardware to inform application that the last byte of a CEC message is received from the CEC line and stored into the RXD buffer. RXEND is set at the same time of RXBR. RXEND is cleared by software write at 1. */
extern const field32_t CEC_ISR_RXOVR;     /** @brief Rx-Overrun RXOVR is set by hardware if RXBR is not yet cleared at the time a new byte is received on the CEC line and stored into RXD. RXOVR assertion stops message reception so that no acknowledge is sent. In case of broadcast, a negative acknowledge is sent. RXOVR is cleared by software write at 1. */
extern const field32_t CEC_ISR_BRE;       /** @brief Rx-Bit rising error BRE is set by hardware in case a data-bit waveform is detected with bit rising error. BRE is set either at the time the misplaced rising edge occurs, or at the end of the maximum BRE tolerance allowed by RXTOL, in case rising edge is still longing. BRE stops message reception if BRESTP=1. BRE generates an error-bit on the CEC line if BREGEN=1. BRE is cleared by software write at 1. */
extern const field32_t CEC_ISR_SBPE;      /** @brief Rx-Short bit period error SBPE is set by hardware in case a data-bit waveform is detected with short bit period error. SBPE is set at the time the anticipated falling edge occurs. SBPE generates an error-bit on the CEC line. SBPE is cleared by software write at 1. */
extern const field32_t CEC_ISR_LBPE;      /** @brief Rx-Long bit period error LBPE is set by hardware in case a data-bit waveform is detected with long bit period error. LBPE is set at the end of the maximum bit-extension tolerance allowed by RXTOL, in case falling edge is still longing. LBPE always stops reception of the CEC message. LBPE generates an error-bit on the CEC line if LBPEGEN=1. In case of broadcast, error-bit is generated even in case of LBPEGEN=0. LBPE is cleared by software write at 1. */
extern const field32_t CEC_ISR_RXACKE;    /** @brief Rx-Missing acknowledge in receive mode, RXACKE is set by hardware to inform application that no acknowledge was seen on the CEC line. RXACKE applies only for broadcast messages and in listen mode also for not directly addressed messages (destination address not enabled in OAR). RXACKE aborts message reception. RXACKE is cleared by software write at 1. */
extern const field32_t CEC_ISR_ARBLST;    /** @brief Arbitration lost ARBLST is set by hardware to inform application that CEC device is switching to reception due to arbitration lost event following the TXSOM command. ARBLST can be due either to a contending CEC device starting earlier or starting at the same time but with higher HEADER priority. After ARBLST assertion TXSOM bit keeps pending for next transmission attempt. ARBLST is cleared by software write at 1. */
extern const field32_t CEC_ISR_TXBR;      /** @brief Tx-Byte request TXBR is set by hardware to inform application that the next transmission data has to be written to TXDR. TXBR is set when the 4th bit of currently transmitted byte is sent. Application must write the next byte to TXDR within 6 nominal data-bit periods before transmission underrun error occurs (TXUDR). TXBR is cleared by software write at 1. */
extern const field32_t CEC_ISR_TXEND;     /** @brief End of transmission TXEND is set by hardware to inform application that the last byte of the CEC message has been successfully transmitted. TXEND clears the TXSOM and TXEOM control bits. TXEND is cleared by software write at 1. */
extern const field32_t CEC_ISR_TXUDR;     /** @brief Tx-Buffer underrun in transmission mode, TXUDR is set by hardware if application was not in time to load TXDR before of next byte transmission. TXUDR aborts message transmission and clears TXSOM and TXEOM control bits. TXUDR is cleared by software write at 1. */
extern const field32_t CEC_ISR_TXERR;     /** @brief Tx-Error in transmission mode, TXERR is set by hardware if the CEC initiator detects low impedance on the CEC line while it is released. TXERR aborts message transmission and clears TXSOM and TXEOM controls. TXERR is cleared by software write at 1. */
extern const field32_t CEC_ISR_TXACKE;    /** @brief Tx-Missing acknowledge error in transmission mode, TXACKE is set by hardware to inform application that no acknowledge was received. In case of broadcast transmission, TXACKE informs application that a negative acknowledge was received. TXACKE aborts message transmission and clears TXSOM and TXEOM controls. TXACKE is cleared by software write at 1. */
extern const field32_t CEC_IER_RXBRIE;    /** @brief Rx-Byte received interrupt enable the RXBRIE bit is set and cleared by software. */
extern const field32_t CEC_IER_RXENDIE;   /** @brief End of reception interrupt enable the RXENDIE bit is set and cleared by software. */
extern const field32_t CEC_IER_RXOVRIE;   /** @brief Rx-Buffer overrun interrupt enable the RXOVRIE bit is set and cleared by software. */
extern const field32_t CEC_IER_BREIE;     /** @brief Bit rising error interrupt enable the BREIE bit is set and cleared by software. */
extern const field32_t CEC_IER_SBPEIE;    /** @brief Short bit period error interrupt enable the SBPEIE bit is set and cleared by software. */
extern const field32_t CEC_IER_LBPEIE;    /** @brief Long bit period error interrupt enable the LBPEIE bit is set and cleared by software. */
extern const field32_t CEC_IER_RXACKIE;   /** @brief Rx-Missing acknowledge error interrupt enable the RXACKIE bit is set and cleared by software. */
extern const field32_t CEC_IER_ARBLSTIE;  /** @brief Arbitration lost interrupt enable the ARBLSTIE bit is set and cleared by software. */
extern const field32_t CEC_IER_TXBRIE;    /** @brief Tx-Byte request interrupt enable the TXBRIE bit is set and cleared by software. */
extern const field32_t CEC_IER_TXENDIE;   /** @brief Tx-End of message interrupt enable the TXENDIE bit is set and cleared by software. */
extern const field32_t CEC_IER_TXUDRIE;   /** @brief Tx-Underrun interrupt enable the TXUDRIE bit is set and cleared by software. */
extern const field32_t CEC_IER_TXERRIE;   /** @brief Tx-Error interrupt enable the TXERRIE bit is set and cleared by software. */
extern const field32_t CEC_IER_TXACKIE;   /** @brief Tx-Missing acknowledge error interrupt enable the TXACKEIE bit is set and cleared by software. */

/**************************************************************************************************
 * @section HSEM Definitions
 **************************************************************************************************/

/** @subsection HSEM Register Definitions */

extern rw_reg32_t const HSEM_IER;  /** @brief HSEM interrupt enable register. */
extern ro_reg32_t const HSEM_ICR;  /** @brief HSEM interrupt clear register. */
extern ro_reg32_t const HSEM_ISR;  /** @brief HSEM interrupt status register. */
extern ro_reg32_t const HSEM_MISR; /** @brief HSEM masked interrupt status register. */
extern rw_reg32_t const HSEM_CR;   /** @brief HSEM clear register. */
extern rw_reg32_t const HSEM_KEYR; /** @brief HSEM interrupt clear register. */

/** @subsection Enumerated HSEM Register Definitions */

extern rw_reg32_t const HSEM_Rx[32];   /** @brief HSEM register HSEM_R0 HSEM_R31. */
extern ro_reg32_t const HSEM_RLRx[32]; /** @brief HSEM read lock register. */

/** @subsection HSEM Register Field Definitions */

extern const field32_t HSEM_Rx_PROCID;   /** @brief Semaphore process id. */
extern const field32_t HSEM_Rx_COREID;   /** @brief Semaphore core id. */
extern const field32_t HSEM_Rx_LOCK;     /** @brief Lock indication. */
extern const field32_t HSEM_RLRx_PROCID; /** @brief Semaphore process id. */
extern const field32_t HSEM_RLRx_COREID; /** @brief Semaphore core id. */
extern const field32_t HSEM_RLRx_LOCK;   /** @brief Lock indication. */
extern const field32_t HSEM_CR_COREID;   /** @brief Core id of semaphores to be cleared. */
extern const field32_t HSEM_CR_KEY;      /** @brief Semaphore clear key. */
extern const field32_t HSEM_KEYR_KEY;    /** @brief Semaphore clear key. */

/** @subsection Enumerated HSEM Register Field Definitions */

extern const field32_t HSEM_IER_ISEMx[32]; /** @brief Interrupt semaphore n enable bit. */

/**************************************************************************************************
 * @section I2Cx Definitions
 **************************************************************************************************/

/** @subsection Enumerated I2Cx Register Definitions */

extern rw_reg32_t const I2Cx_CR1[5];      /** @brief Access: no wait states, except if a write access occurs while a write access to this register is ongoing. In this case, wait states are inserted in the second write access until the previous one is completed. The latency of the second write access can be up to 2 x PCLK1 + 6 x I2CCLK. */
extern rw_reg32_t const I2Cx_CR2[5];      /** @brief Access: no wait states, except if a write access occurs while a write access to this register is ongoing. In this case, wait states are inserted in the second write access until the previous one is completed. The latency of the second write access can be up to 2 x PCLK1 + 6 x I2CCLK. */
extern rw_reg32_t const I2Cx_OAR1[5];     /** @brief Access: no wait states, except if a write access occurs while a write access to this register is ongoing. In this case, wait states are inserted in the second write access until the previous one is completed. The latency of the second write access can be up to 2 x PCLK1 + 6 x I2CCLK. */
extern rw_reg32_t const I2Cx_OAR2[5];     /** @brief Access: no wait states, except if a write access occurs while a write access to this register is ongoing. In this case, wait states are inserted in the second write access until the previous one is completed. The latency of the second write access can be up to 2 x PCLK1 + 6 x I2CCLK. */
extern rw_reg32_t const I2Cx_TIMINGR[5];  /** @brief Access: no wait states. */
extern rw_reg32_t const I2Cx_TIMEOUTR[5]; /** @brief Access: no wait states, except if a write access occurs while a write access to this register is ongoing. In this case, wait states are inserted in the second write access until the previous one is completed. The latency of the second write access can be up to 2 x PCLK1 + 6 x I2CCLK. */
extern rw_reg32_t const I2Cx_ISR[5];      /** @brief Access: no wait states. */
extern rw_reg32_t const I2Cx_ICR[5];      /** @brief Access: no wait states. */
extern ro_reg32_t const I2Cx_PECR[5];     /** @brief Access: no wait states. */
extern ro_reg32_t const I2Cx_RXDR[5];     /** @brief Access: no wait states. */
extern rw_reg32_t const I2Cx_TXDR[5];     /** @brief Access: no wait states. */

/** @subsection I2Cx Register Field Definitions */

extern const field32_t I2Cx_CR1_PE;            /** @brief Peripheral enable note: when PE=0, the I2C SCL and SDA lines are released. Internal state machines and status bits are put back to their reset value. When cleared, PE must be kept low for at least 3 APB clock cycles. */
extern const field32_t I2Cx_CR1_TXIE;          /** @brief TX interrupt enable. */
extern const field32_t I2Cx_CR1_RXIE;          /** @brief RX interrupt enable. */
extern const field32_t I2Cx_CR1_ADDRIE;        /** @brief Address match interrupt enable (slave only). */
extern const field32_t I2Cx_CR1_NACKIE;        /** @brief Not acknowledge received interrupt enable. */
extern const field32_t I2Cx_CR1_STOPIE;        /** @brief STOP detection interrupt enable. */
extern const field32_t I2Cx_CR1_TCIE;          /** @brief Transfer complete interrupt enable note: any of these events will generate an interrupt: transfer complete (TC) transfer complete reload (TCR). */
extern const field32_t I2Cx_CR1_ERRIE;         /** @brief Error interrupts enable note: any of these errors generate an interrupt: arbitration loss (ARLO) bus error detection (BERR) overrun/underrun (OVR) timeout detection (TIMEOUT) PEC error detection (PECERR) alert pin event detection (ALERT). */
extern const field32_t I2Cx_CR1_DNF;           /** @brief Digital noise filter these bits are used to configure the digital noise filter on SDA and SCL input. The digital filter will filter spikes with a length of up to DNF[3:0] * ti2cclk ... Note: if the analog filter is also enabled, the digital filter is added to the analog filter. This filter can only be programmed when the I2C is disabled (PE = 0). */
extern const field32_t I2Cx_CR1_ANFOFF;        /** @brief Analog noise filter OFF note: this bit can only be programmed when the I2C is disabled (PE = 0). */
extern const field32_t I2Cx_CR1_TXDMAEN;       /** @brief DMA transmission requests enable. */
extern const field32_t I2Cx_CR1_RXDMAEN;       /** @brief DMA reception requests enable. */
extern const field32_t I2Cx_CR1_SBC;           /** @brief Slave byte control this bit is used to enable hardware byte control in slave mode. */
extern const field32_t I2Cx_CR1_NOSTRETCH;     /** @brief Clock stretching disable this bit is used to disable clock stretching in slave mode. It must be kept cleared in master mode. Note: this bit can only be programmed when the I2C is disabled (PE = 0). */
extern const field32_t I2Cx_CR1_WUPEN;         /** @brief Wakeup from stop mode enable note: if the wakeup from stop mode feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. Note: WUPEN can be set only when DNF = 0000. */
extern const field32_t I2Cx_CR1_GCEN;          /** @brief General call enable. */
extern const field32_t I2Cx_CR1_SMBHEN;        /** @brief SMBus host address enable note: if the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_CR1_SMBDEN;        /** @brief SMBus device default address enable note: if the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_CR1_ALERTEN;       /** @brief SMBus alert enable device mode (SMBHEN=0): host mode (SMBHEN=1): note: when ALERTEN=0, the SMBA pin can be used as a standard GPIO. If the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_CR1_PECEN;         /** @brief PEC enable note: if the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_CR2_RD_WRN;        /** @brief Transfer direction (master mode) note: changing this bit when the START bit is set is not allowed. */
extern const field32_t I2Cx_CR2_ADD10;         /** @brief 10-bit addressing mode (master mode) note: changing this bit when the START bit is set is not allowed. */
extern const field32_t I2Cx_CR2_HEAD10R;       /** @brief 10-bit address header only read direction (master receiver mode) note: changing this bit when the START bit is set is not allowed. */
extern const field32_t I2Cx_CR2_START;         /** @brief Start generation this bit is set by software, and cleared by hardware after the start followed by the address sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can also be cleared by software by writing 1 to the ADDRCF bit in the I2C_ICR register. If the I2C is already in master mode with AUTOEND = 0, setting this bit generates a repeated start condition when RELOAD=0, after the end of the NBYTES transfer. Otherwise setting this bit will generate a START condition once the bus is free. Note: writing 0 to this bit has no effect. The START bit can be set even if the bus is BUSY or I2C is in slave mode. This bit has no effect when RELOAD is set. */
extern const field32_t I2Cx_CR2_STOP;          /** @brief Stop generation (master mode) the bit is set by software, cleared by hardware when a stop condition is detected, or when PE = 0. In master mode: note: writing 0 to this bit has no effect. */
extern const field32_t I2Cx_CR2_NACK;          /** @brief NACK generation (slave mode) the bit is set by software, cleared by hardware when the NACK is sent, or when a STOP condition or an address matched is received, or when PE=0. Note: writing 0 to this bit has no effect. This bit is used in slave mode only: in master receiver mode, NACK is automatically generated after last byte preceding STOP or RESTART condition, whatever the NACK bit value. When an overrun occurs in slave receiver NOSTRETCH mode, a NACK is automatically generated whatever the NACK bit value. When hardware PEC checking is enabled (PECBYTE=1), the PEC acknowledge value does not depend on the NACK value. */
extern const field32_t I2Cx_CR2_NBYTES;        /** @brief Number of bytes the number of bytes to be transmitted/received is programmed there. This field is dont care in slave mode with SBC=0. Note: changing these bits when the START bit is set is not allowed. */
extern const field32_t I2Cx_CR2_RELOAD;        /** @brief NBYTES reload mode this bit is set and cleared by software. */
extern const field32_t I2Cx_CR2_AUTOEND;       /** @brief Automatic end mode (master mode) this bit is set and cleared by software. Note: this bit has no effect in slave mode or when the RELOAD bit is set. */
extern const field32_t I2Cx_CR2_PECBYTE;       /** @brief Packet error checking byte this bit is set by software, and cleared by hardware when the PEC is transferred, or when a STOP condition or an address matched is received, also when PE=0. Note: writing 0 to this bit has no effect. This bit has no effect when RELOAD is set. This bit has no effect is slave mode when SBC=0. If the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_OAR1_OA1;          /** @brief Interface address 7-bit addressing mode: dont care 10-bit addressing mode: bits 9:8 of address note: these bits can be written only when OA1EN=0. OA1[7:1]: interface address bits 7:1 of address note: these bits can be written only when OA1EN=0. OA1[0]: interface address 7-bit addressing mode: dont care 10-bit addressing mode: bit 0 of address note: this bit can be written only when OA1EN=0. */
extern const field32_t I2Cx_OAR1_OA1MODE;      /** @brief Own address 1 10-bit mode note: this bit can be written only when OA1EN=0. */
extern const field32_t I2Cx_OAR1_OA1EN;        /** @brief Own address 1 enable. */
extern const field32_t I2Cx_OAR2_OA2;          /** @brief Interface address bits 7:1 of address note: these bits can be written only when OA2EN=0. */
extern const field32_t I2Cx_OAR2_OA2MSK;       /** @brief Own address 2 masks note: these bits can be written only when OA2EN=0. As soon as OA2MSK is not equal to 0, the reserved I2C addresses (0b0000xxx and 0b1111xxx) are not acknowledged even if the comparison matches. */
extern const field32_t I2Cx_OAR2_OA2EN;        /** @brief Own address 2 enable. */
extern const field32_t I2Cx_TIMINGR_SCLL;      /** @brief SCL low period (master mode) this field is used to generate the SCL low period in master mode. TSCLL = (SCLL+1) x tpresc note: SCLL is also used to generate tbuf and tsu:sta timings. */
extern const field32_t I2Cx_TIMINGR_SCLH;      /** @brief SCL high period (master mode) this field is used to generate the SCL high period in master mode. TSCLH = (SCLH+1) x tpresc note: SCLH is also used to generate tsu:sto and thd:sta timing. */
extern const field32_t I2Cx_TIMINGR_SDADEL;    /** @brief Data hold time this field is used to generate the delay tsdadel between SCL falling edge and SDA edge. In master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during tsdadel. TSDADEL= SDADEL x tpresc note: SDADEL is used to generate thd:dat timing. */
extern const field32_t I2Cx_TIMINGR_SCLDEL;    /** @brief Data setup time this field is used to generate a delay tscldel between SDA edge and SCL rising edge. In master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during tscldel. TSCLDEL = (SCLDEL+1) x tpresc note: tscldel is used to generate tsu:dat timing. */
extern const field32_t I2Cx_TIMINGR_PRESC;     /** @brief Timing prescaler this field is used to prescale I2CCLK in order to generate the clock period tpresc used for data setup and hold counters (refer to I2C timings on page9) and for SCL high and low level counters (refer to I2C master initialization on page24). TPRESC = (PRESC+1) x ti2cclk. */
extern const field32_t I2Cx_TIMEOUTR_TIMEOUTA; /** @brief Bus timeout A this field is used to configure: the SCL low timeout condition ttimeout when TIDLE=0 ttimeout= (TIMEOUTA+1) x 2048 x ti2cclk the bus idle condition (both SCL and SDA high) when TIDLE=1 tidle= (TIMEOUTA+1) x 4 x ti2cclk note: these bits can be written only when TIMOUTEN=0. */
extern const field32_t I2Cx_TIMEOUTR_TIDLE;    /** @brief Idle clock timeout detection note: this bit can be written only when TIMOUTEN=0. */
extern const field32_t I2Cx_TIMEOUTR_TIMOUTEN; /** @brief Clock timeout enable. */
extern const field32_t I2Cx_TIMEOUTR_TIMEOUTB; /** @brief Bus timeout B this field is used to configure the cumulative clock extension timeout: in master mode, the master cumulative clock low extend time (tlow:mext) is detected in slave mode, the slave cumulative clock low extend time (tlow:sext) is detected tlow:ext= (TIMEOUTB+1) x 2048 x ti2cclk note: these bits can be written only when TEXTEN=0. */
extern const field32_t I2Cx_TIMEOUTR_TEXTEN;   /** @brief Extended clock timeout enable. */
extern const field32_t I2Cx_ISR_TXE;           /** @brief Transmit data register empty (transmitters) this bit is set by hardware when the I2C_TXDR register is empty. It is cleared when the next data to be sent is written in the I2C_TXDR register. This bit can be written to 1 by software in order to flush the transmit data register I2C_TXDR. Note: this bit is set by hardware when PE=0. */
extern const field32_t I2Cx_ISR_TXIS;          /** @brief Transmit interrupt status (transmitters) this bit is set by hardware when the I2C_TXDR register is empty and the data to be transmitted must be written in the I2C_TXDR register. It is cleared when the next data to be sent is written in the I2C_TXDR register. This bit can be written to 1 by software when NOSTRETCH=1 only, in order to generate a TXIS event (interrupt if TXIE=1 or DMA request if TXDMAEN=1). Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_RXNE;          /** @brief Receive data register not empty (receivers) this bit is set by hardware when the received data is copied into the I2C_RXDR register, and is ready to be read. It is cleared when I2C_RXDR is read. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_ADDR;          /** @brief Address matched (slave mode) this bit is set by hardware as soon as the received slave address matched with one of the enabled slave addresses. It is cleared by software by setting ADDRCF bit. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_NACKF;         /** @brief Not acknowledge received flag this flag is set by hardware when a NACK is received after a byte transmission. It is cleared by software by setting the NACKCF bit. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_STOPF;         /** @brief Stop detection flag this flag is set by hardware when a stop condition is detected on the bus and the peripheral is involved in this transfer: either as a master, provided that the STOP condition is generated by the peripheral. Or as a slave, provided that the peripheral has been addressed previously during this transfer. It is cleared by software by setting the STOPCF bit. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_TC;            /** @brief Transfer complete (master mode) this flag is set by hardware when RELOAD=0, AUTOEND=0 and NBYTES data have been transferred. It is cleared by software when START bit or STOP bit is set. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_TCR;           /** @brief Transfer complete reload this flag is set by hardware when RELOAD=1 and NBYTES data have been transferred. It is cleared by software when NBYTES is written to a non-zero value. Note: this bit is cleared by hardware when PE=0. This flag is only for master mode, or for slave mode when the SBC bit is set. */
extern const field32_t I2Cx_ISR_BERR;          /** @brief Bus error this flag is set by hardware when a misplaced start or stop condition is detected whereas the peripheral is involved in the transfer. The flag is not set during the address phase in slave mode. It is cleared by software by setting BERRCF bit. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_ARLO;          /** @brief Arbitration lost this flag is set by hardware in case of arbitration loss. It is cleared by software by setting the ARLOCF bit. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_OVR;           /** @brief Overrun/Underrun (slave mode) this flag is set by hardware in slave mode with NOSTRETCH=1, when an overrun/underrun error occurs. It is cleared by software by setting the OVRCF bit. Note: this bit is cleared by hardware when PE=0. */
extern const field32_t I2Cx_ISR_PECERR;        /** @brief PEC error in reception this flag is set by hardware when the received PEC does not match with the PEC register content. A NACK is automatically sent after the wrong PEC reception. It is cleared by software by setting the PECCF bit. Note: this bit is cleared by hardware when PE=0. If the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_ISR_TIMEOUT;       /** @brief Timeout or tlow detection flag this flag is set by hardware when a timeout or extended clock timeout occurred. It is cleared by software by setting the TIMEOUTCF bit. Note: this bit is cleared by hardware when PE=0. If the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_ISR_ALERT;         /** @brief SMBus alert this flag is set by hardware when SMBHEN=1 (smbus host configuration), ALERTEN=1 and a SMBALERT event (falling edge) is detected on SMBA pin. It is cleared by software by setting the ALERTCF bit. Note: this bit is cleared by hardware when PE=0. If the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_ISR_BUSY;          /** @brief Bus busy this flag indicates that a communication is in progress on the bus. It is set by hardware when a START condition is detected. It is cleared by hardware when a stop condition is detected, or when PE=0. */
extern const field32_t I2Cx_ISR_DIR;           /** @brief Transfer direction (slave mode) this flag is updated when an address match event occurs (ADDR=1). */
extern const field32_t I2Cx_ISR_ADDCODE;       /** @brief Address match code (slave mode) these bits are updated with the received address when an address match event occurs (ADDR = 1). In the case of a 10-bit address, ADDCODE provides the 10-bit header followed by the 2 msbs of the address. */
extern const field32_t I2Cx_ICR_ADDRCF;        /** @brief Address matched flag clear writing 1 to this bit clears the ADDR flag in the I2C_ISR register. Writing 1 to this bit also clears the START bit in the I2C_CR2 register. */
extern const field32_t I2Cx_ICR_NACKCF;        /** @brief Not acknowledge flag clear writing 1 to this bit clears the ACKF flag in I2C_ISR register. */
extern const field32_t I2Cx_ICR_STOPCF;        /** @brief Stop detection flag clear writing 1 to this bit clears the STOPF flag in the I2C_ISR register. */
extern const field32_t I2Cx_ICR_BERRCF;        /** @brief Bus error flag clear writing 1 to this bit clears the BERRF flag in the I2C_ISR register. */
extern const field32_t I2Cx_ICR_ARLOCF;        /** @brief Arbitration lost flag clear writing 1 to this bit clears the ARLO flag in the I2C_ISR register. */
extern const field32_t I2Cx_ICR_OVRCF;         /** @brief Overrun/Underrun flag clear writing 1 to this bit clears the OVR flag in the I2C_ISR register. */
extern const field32_t I2Cx_ICR_PECCF;         /** @brief PEC error flag clear writing 1 to this bit clears the PECERR flag in the I2C_ISR register. Note: if the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_ICR_TIMOUTCF;      /** @brief Timeout detection flag clear writing 1 to this bit clears the TIMEOUT flag in the I2C_ISR register. Note: if the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_ICR_ALERTCF;       /** @brief Alert flag clear writing 1 to this bit clears the ALERT flag in the I2C_ISR register. Note: if the smbus feature is not supported, this bit is reserved and forced by hardware to 0. Please refer to section25.3: I2C implementation. */
extern const field32_t I2Cx_PECR_PEC;          /** @brief Packet error checking register this field contains the internal PEC when PECEN=1. The PEC is cleared by hardware when PE=0. */
extern const field32_t I2Cx_RXDR_RXDATA;       /** @brief 8-bit receive data data byte received from the I2C bus. */
extern const field32_t I2Cx_TXDR_TXDATA;       /** @brief 8-bit transmit data data byte to be transmitted to the I2C bus. Note: these bits can be written only when TXE=1. */
extern const field32_t I2Cx_CR2_SADD_7BIT;     /** @todo */
extern const field32_t I2Cx_CR2_SADD_9BIT;     /** @todo */

/** @subsection Enumerated I2Cx Register Field Definitions */

extern const field32_t I2Cx_CR2_SADDx[10]; /** @brief Slave address bit 0 (master mode) in 7-bit addressing mode (ADD10 = 0): this bit is dont care in 10-bit addressing mode (ADD10 = 1): this bit should be written with bit 0 of the slave address to be sent note: changing these bits when the START bit is set is not allowed. */

/**************************************************************************************************
 * @section GPIOx Definitions
 **************************************************************************************************/

/** @subsection Enumerated GPIOx Register Definitions */

extern rw_reg32_t const GPIOx_MODER[11];   /** @brief GPIO port mode register. */
extern rw_reg32_t const GPIOx_OTYPER[11];  /** @brief GPIO port output type register. */
extern rw_reg32_t const GPIOx_OSPEEDR[11]; /** @brief GPIO port output speed register. */
extern rw_reg32_t const GPIOx_PUPDR[11];   /** @brief GPIO port pull-up/pull-down register. */
extern ro_reg32_t const GPIOx_IDR[11];     /** @brief GPIO port input data register. */
extern rw_reg32_t const GPIOx_ODR[11];     /** @brief GPIO port output data register. */
extern rw_reg32_t const GPIOx_BSRR[11];    /** @brief GPIO port bit set/reset register. */
extern rw_reg32_t const GPIOx_LCKR[11];    /** @brief This register is used to lock the configuration of the port bits when a correct write sequence is applied to bit 16 (LCKK). The value of bits [15:0] is used to lock the configuration of the GPIO. During the write sequence, the value of LCKR[15:0] must not change. When the LOCK sequence has been applied on a port bit, the value of this port bit can no longer be modified until the next MCU reset or peripheral reset.a specific write sequence is used to write to the gpiox_lckr register. Only word access (32-bit long) is allowed during this locking sequence.each lock bit freezes a specific configuration register (control and alternate function registers). */
extern rw_reg32_t const GPIOx_AFRL[11];    /** @brief GPIO alternate function low register. */
extern rw_reg32_t const GPIOx_AFRH[11];    /** @brief GPIO alternate function high register. */

/** @subsection GPIOx Register Field Definitions */

extern const field32_t GPIOx_LCKR_LCKK; /** @brief Lock key this bit can be read any time. It can only be modified using the lock key write sequence. LOCK key write sequence: WR LCKR[16] = 1 + LCKR[15:0] WR LCKR[16] = 0 + LCKR[15:0] WR LCKR[16] = 1 + LCKR[15:0] RD LCKR RD LCKR[16] = 1 (this read operation is optional but it confirms that the lock is active) note: during the LOCK key write sequence, the value of LCK[15:0] must not change. Any error in the lock sequence aborts the lock. After the first lock sequence on any bit of the port, any read access on the LCKK bit will return 1 until the next MCU reset or peripheral reset. */

/** @subsection Enumerated GPIOx Register Field Definitions */

extern const field32_t GPIOx_MODER_MODEx[16];     /** @brief [1:0]: port x configuration bits (y = 0..15) these bits are written by software to configure the I/O mode. */
extern const field32_t GPIOx_OTYPER_OTx[16];      /** @brief Port x configuration bits (y = 0..15) these bits are written by software to configure the I/O output type. */
extern const field32_t GPIOx_OSPEEDR_OSPEEDx[16]; /** @brief [1:0]: port x configuration bits (y = 0..15) these bits are written by software to configure the I/O output speed. Note: refer to the device datasheet for the frequency specifications and the power supply and load conditions for each speed. */
extern const field32_t GPIOx_PUPDR_PUPDx[16];     /** @brief [1:0]: port x configuration bits (y = 0..15) these bits are written by software to configure the I/O pull-up or pull-down. */
extern const field32_t GPIOx_IDR_IDx[16];         /** @brief Port input data bit (y = 0..15) these bits are read-only. They contain the input value of the corresponding I/O port. */
extern const field32_t GPIOx_ODR_ODx[16];         /** @brief Port output data bit these bits can be read and written by software. Note: for atomic bit set/reset, the OD bits can be individually set and/or reset by writing to the gpiox_bsrr or gpiox_brr registers (x = A..F). */
extern const field32_t GPIOx_BSRR_BSx[16];        /** @brief Port x set bit y (y= 0..15) these bits are write-only. A read to these bits returns the value 0x0000. */
extern const field32_t GPIOx_BSRR_BRx[16];        /** @brief Port x reset bit y (y = 0..15) these bits are write-only. A read to these bits returns the value 0x0000. Note: if both bsx and brx are set, bsx has priority. */
extern const field32_t GPIOx_LCKR_LCKx[16];       /** @brief Port x lock bit y (y= 0..15) these bits are read/write but can only be written when the LCKK bit is 0. */
extern const field32_t GPIOx_AFRL_AFSELx[8];      /** @brief [3:0]: alternate function selection for port x pin y (y = 0..7) these bits are written by software to configure alternate function i/os afsely selection:. */
extern const field32_t GPIOx_AFRH_AFSELx[8];      /** @brief [3:0]: alternate function selection for port x pin y (y = 0..7) these bits are written by software to configure alternate function i/os afsely selection:. */

/**************************************************************************************************
 * @section JPEG Definitions
 **************************************************************************************************/

/** @subsection JPEG Register Definitions */

extern rw_reg32_t const JPEG_CONFR0; /** @brief JPEG codec control register. */
extern rw_reg32_t const JPEG_CONFR1; /** @brief JPEG codec configuration register 1. */
extern rw_reg32_t const JPEG_CONFR2; /** @brief JPEG codec configuration register 2. */
extern rw_reg32_t const JPEG_CONFR3; /** @brief JPEG codec configuration register 3. */
extern rw_reg32_t const JPEG_CR;     /** @brief JPEG control register. */
extern ro_reg32_t const JPEG_SR;     /** @brief JPEG status register. */
extern rw_reg32_t const JPEG_CFR;    /** @brief JPEG clear flag register. */
extern rw_reg32_t const JPEG_DIR;    /** @brief JPEG data input register. */
extern ro_reg32_t const JPEG_DOR;    /** @brief JPEG data output register. */

/** @subsection Enumerated JPEG Register Definitions */

extern rw_reg32_t const JPEG_CONFRNx[5]; /** @brief JPEG codec configuration register 4-7. */

/** @subsection JPEG Register Field Definitions */

extern const field32_t JPEG_CONFR0_START;      /** @brief Start this bit start or stop the encoding or decoding process. Read this register always return 0. */
extern const field32_t JPEG_CONFR1_NF;         /** @brief Number of color components this field defines the number of color components minus 1. */
extern const field32_t JPEG_CONFR1_DE;         /** @brief Decoding enable this bit selects the coding or decoding process. */
extern const field32_t JPEG_CONFR1_COLORSPACE; /** @brief Color space this filed defines the number of quantization tables minus 1 to insert in the output stream. */
extern const field32_t JPEG_CONFR1_NS;         /** @brief Number of components for scan this field defines the number of components minus 1 for scan header marker segment. */
extern const field32_t JPEG_CONFR1_HDR;        /** @brief Header processing this bit enable the header processing (generation/parsing). */
extern const field32_t JPEG_CONFR1_YSIZE;      /** @brief Y size this field defines the number of lines in source image. */
extern const field32_t JPEG_CONFR2_NMCU;       /** @brief Number of MCU for encoding: this field defines the number of MCU units minus 1 to encode. For decoding: this field indicates the number of complete MCU units minus 1 to be decoded (this field is updated after the JPEG header parsing). If the decoded image size has not a X or Y size multiple of 8 or 16 (depending on the sub-sampling process), the resulting incomplete or empty MCU must be added to this value to get the total number of MCU generated. */
extern const field32_t JPEG_CONFR3_XSIZE;      /** @brief X size this field defines the number of pixels per line. */
extern const field32_t JPEG_CONFRNx_HD;        /** @brief Huffman DC selects the huffman table for encoding the DC coefficients. */
extern const field32_t JPEG_CONFRNx_HA;        /** @brief Huffman AC selects the huffman table for encoding the AC coefficients. */
extern const field32_t JPEG_CONFRNx_QT;        /** @brief Quantization table selects quantization table associated with a color component. */
extern const field32_t JPEG_CONFRNx_NB;        /** @brief Number of block number of data units minus 1 that belong to a particular color in the MCU. */
extern const field32_t JPEG_CONFRNx_VSF;       /** @brief Vertical sampling factor vertical sampling factor for component i. */
extern const field32_t JPEG_CONFRNx_HSF;       /** @brief Horizontal sampling factor horizontal sampling factor for component i. */
extern const field32_t JPEG_CR_JCEN;           /** @brief JPEG core enable enable the JPEG codec core. */
extern const field32_t JPEG_CR_IFTIE;          /** @brief Input FIFO threshold interrupt enable this bit enables the interrupt generation when input FIFO reach the threshold. */
extern const field32_t JPEG_CR_IFNFIE;         /** @brief Input FIFO not full interrupt enable this bit enables the interrupt generation when input FIFO is not empty. */
extern const field32_t JPEG_CR_OFTIE;          /** @brief Output FIFO threshold interrupt enable this bit enables the interrupt generation when output FIFO reach the threshold. */
extern const field32_t JPEG_CR_OFNEIE;         /** @brief Output FIFO not empty interrupt enable this bit enables the interrupt generation when output FIFO is not empty. */
extern const field32_t JPEG_CR_EOCIE;          /** @brief End of conversion interrupt enable this bit enables the interrupt generation on the end of conversion. */
extern const field32_t JPEG_CR_HPDIE;          /** @brief Header parsing done interrupt enable this bit enables the interrupt generation on the header parsing operation. */
extern const field32_t JPEG_CR_IDMAEN;         /** @brief Input DMA enable enable the DMA request generation for the input FIFO. */
extern const field32_t JPEG_CR_ODMAEN;         /** @brief Output DMA enable enable the DMA request generation for the output FIFO. */
extern const field32_t JPEG_CR_IFF;            /** @brief Input FIFO flush this bit flush the input FIFO. This bit is always read as 0. */
extern const field32_t JPEG_CR_OFF;            /** @brief Output FIFO flush this bit flush the output FIFO. This bit is always read as 0. */
extern const field32_t JPEG_SR_IFTF;           /** @brief Input FIFO threshold flag this bit is set when the input FIFO is not full and is bellow its threshold. */
extern const field32_t JPEG_SR_IFNFF;          /** @brief Input FIFO not full flag this bit is set when the input FIFO is not full (a data can be written). */
extern const field32_t JPEG_SR_OFTF;           /** @brief Output FIFO threshold flag this bit is set when the output FIFO is not empty and has reach its threshold. */
extern const field32_t JPEG_SR_OFNEF;          /** @brief Output FIFO not empty flag this bit is set when the output FIFO is not empty (a data is available). */
extern const field32_t JPEG_SR_EOCF;           /** @brief End of conversion flag this bit is set when the JPEG codec core has finished the encoding or the decoding process and than last data has been sent to the output FIFO. */
extern const field32_t JPEG_SR_HPDF;           /** @brief Header parsing done flag this bit is set in decode mode when the JPEG codec has finished the parsing of the headers and the internal registers have been updated. */
extern const field32_t JPEG_SR_COF;            /** @brief Codec operation flag this bit is set when when a JPEG codec operation is on going (encoding or decoding). */
extern const field32_t JPEG_CFR_CEOCF;         /** @brief Clear end of conversion flag writing 1 clears the end of conversion flag of the JPEG status register. */
extern const field32_t JPEG_CFR_CHPDF;         /** @brief Clear header parsing done flag writing 1 clears the header parsing done flag of the JPEG status register. */

/**************************************************************************************************
 * @section MDMA Definitions
 **************************************************************************************************/

/** @subsection MDMA Register Definitions */

extern ro_reg32_t const MDMA_MDMA_GISR0;   /** @brief MDMA global interrupt/status register. */
extern ro_reg32_t const MDMA_MDMA_C0ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C0IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C1ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C1IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C2ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C2IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C3ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C3IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C4ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C4IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C5ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C5IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C6ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C6IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C7ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C7IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C8ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C8IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C9ISR;   /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C9IFCR;  /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C10ISR;  /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C10IFCR; /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C11ISR;  /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C11IFCR; /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C12ISR;  /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C12IFCR; /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C13ISR;  /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C13IFCR; /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C14ISR;  /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C14IFCR; /** @brief MDMA channel x interrupt flag clear register. */
extern ro_reg32_t const MDMA_MDMA_C15ISR;  /** @brief MDMA channel x interrupt/status register. */
extern rw_reg32_t const MDMA_MDMA_C15IFCR; /** @brief MDMA channel x interrupt flag clear register. */

/** @subsection Enumerated MDMA Register Definitions */

extern ro_reg32_t const MDMA_MDMA_CxESR[16];   /** @brief MDMA channel x error status register. */
extern rw_reg32_t const MDMA_MDMA_CxCR[16];    /** @brief This register is used to control the concerned channel. */
extern rw_reg32_t const MDMA_MDMA_CxTCR[16];   /** @brief This register is used to configure the concerned channel. */
extern rw_reg32_t const MDMA_MDMA_CxBNDTR[16]; /** @brief MDMA channel x block number of data register. */
extern rw_reg32_t const MDMA_MDMA_CxSAR[16];   /** @brief MDMA channel x source address register. */
extern rw_reg32_t const MDMA_MDMA_CxDAR[16];   /** @brief MDMA channel x destination address register. */
extern rw_reg32_t const MDMA_MDMA_CxBRUR[16];  /** @brief MDMA channel x block repeat address update register. */
extern rw_reg32_t const MDMA_MDMA_CxLAR[16];   /** @brief MDMA channel x link address register. */
extern rw_reg32_t const MDMA_MDMA_CxTBR[16];   /** @brief MDMA channel x trigger and bus selection register. */
extern rw_reg32_t const MDMA_MDMA_CxMAR[16];   /** @brief MDMA channel x mask address register. */
extern rw_reg32_t const MDMA_MDMA_CxMDR[16];   /** @brief MDMA channel x mask data register. */

/** @subsection MDMA Register Field Definitions */

extern const field32_t MDMA_MDMA_C0ISR_TEIF0;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C0ISR_CTCIF0;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C0ISR_BRTIF0;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C0ISR_BTIF0;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C0ISR_TCIF0;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C0ISR_CRQA0;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C0IFCR_CTEIF0;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C0IFCR_CCTCIF0;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C0IFCR_CBRTIF0;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C0IFCR_CBTIF0;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C0IFCR_CLTCIF0;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_CxESR_TEA;        /** @brief Transfer error address these bits are set and cleared by HW, in case of an MDMA data transfer error. It is used in conjunction with TED. This field indicates the 7 lsbits of the address which generated a transfer/access error. It may be used by SW to retrieve the failing address, by adding this value (truncated to the buffer transfer length size) to the current SAR/DAR value. Note: the SAR/DAR current value doesnt reflect this last address due to the FIFO management system. The SAR/DAR are only updated at the end of a (buffer) transfer (of TLEN+1 bytes). Note: it is not set in case of a link data error. */
extern const field32_t MDMA_MDMA_CxESR_TED;        /** @brief Transfer error direction these bit is set and cleared by HW, in case of an MDMA data transfer error. */
extern const field32_t MDMA_MDMA_CxESR_TELD;       /** @brief Transfer error link data these bit is set by HW, in case of a transfer error while reading the block link data structure. It is cleared by software writing 1 to the cteifx bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_CxESR_TEMD;       /** @brief Transfer error mask data these bit is set by HW, in case of a transfer error while writing the mask data. It is cleared by software writing 1 to the cteifx bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_CxESR_ASE;        /** @brief Address/Size error these bit is set by HW, when the programmed address is not aligned with the data size. TED will indicate whether the problem is on the source or destination. It is cleared by software writing 1 to the cteifx bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_CxESR_BSE;        /** @brief Block size error these bit is set by HW, when the block size is not an integer multiple of the data size either for source or destination. TED will indicate whether the problem is on the source or destination. It is cleared by software writing 1 to the cteifx bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_CxCR_EN;          /** @brief Channel enable. */
extern const field32_t MDMA_MDMA_CxCR_TEIE;        /** @brief Transfer error interrupt enable this bit is set and cleared by software. */
extern const field32_t MDMA_MDMA_CxCR_CTCIE;       /** @brief Channel transfer complete interrupt enable this bit is set and cleared by software. */
extern const field32_t MDMA_MDMA_CxCR_BRTIE;       /** @brief Block repeat transfer interrupt enable this bit is set and cleared by software. */
extern const field32_t MDMA_MDMA_CxCR_BTIE;        /** @brief Block transfer interrupt enable this bit is set and cleared by software. */
extern const field32_t MDMA_MDMA_CxCR_TCIE;        /** @brief Buffer transfer complete interrupt enable this bit is set and cleared by software. */
extern const field32_t MDMA_MDMA_CxCR_PL;          /** @brief Priority level these bits are set and cleared by software. These bits are protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxCR_BEX;         /** @brief Byte endianness exchange. */
extern const field32_t MDMA_MDMA_CxCR_HEX;         /** @brief Half word endianes exchange. */
extern const field32_t MDMA_MDMA_CxCR_WEX;         /** @brief Word endianness exchange. */
extern const field32_t MDMA_MDMA_CxCR_SWRQ;        /** @brief SW request writing a 1 into this bit sets the crqax in mdma_isry register, activating the request on channel x note: either the whole cxcr register or the 8-bit/16-bit register @ address offset: 0x4e + 0x40 chn may be used for SWRQ activation. In case of a SW request, acknowledge is not generated (neither HW signal, nor cxmar write access). */
extern const field32_t MDMA_MDMA_CxTCR_SINC;       /** @brief Source increment mode these bits are set and cleared by software. These bits are protected and can be written only if EN is 0 note: when source is AHB (SBUS=1), SINC = 00 is forbidden. In linked list mode, at the end of a block (single or last block in repeated block transfer mode), this register will be loaded from memory (from address given by current LAR[31:0] + 0x00). */
extern const field32_t MDMA_MDMA_CxTCR_DINC;       /** @brief Destination increment mode these bits are set and cleared by software. These bits are protected and can be written only if EN is 0 note: when destination is AHB (DBUS=1), DINC = 00 is forbidden. */
extern const field32_t MDMA_MDMA_CxTCR_SSIZE;      /** @brief Source data size these bits are set and cleared by software. These bits are protected and can be written only if EN is 0 note: if a value of 11 is programmed for the TCM access/ahb port, a transfer error will occur (TEIF bit set) if SINCOS &lt; SSIZE and SINC &#8800; 00, the result will be unpredictable. Note: SSIZE = 11 (double-word) is forbidden when source is TCM/AHB bus (SBUS=1). */
extern const field32_t MDMA_MDMA_CxTCR_DSIZE;      /** @brief Destination data size these bits are set and cleared by software. These bits are protected and can be written only if EN is 0. Note: if a value of 11 is programmed for the TCM access/ahb port, a transfer error will occur (TEIF bit set) if DINCOS &lt; DSIZE and DINC &#8800; 00, the result will be unpredictable. Note: DSIZE = 11 (double-word) is forbidden when destination is TCM/AHB bus (DBUS=1). */
extern const field32_t MDMA_MDMA_CxTCR_SINCOS;     /** @brief Source increment offset size. */
extern const field32_t MDMA_MDMA_CxTCR_DINCOS;     /** @brief Destination increment offset. */
extern const field32_t MDMA_MDMA_CxTCR_SBURST;     /** @brief Source burst transfer configuration. */
extern const field32_t MDMA_MDMA_CxTCR_DBURST;     /** @brief Destination burst transfer configuration. */
extern const field32_t MDMA_MDMA_CxTCR_TLEN;       /** @brief Buffer transfer lengh. */
extern const field32_t MDMA_MDMA_CxTCR_PKE;        /** @brief PacK enable these bit is set and cleared by software. If the source size is smaller than the destination, it will be padded according to the PAM value. If the source data size is larger than the destination one, it will be truncated. The alignment will be done according to the PAM[0] value. This bit is protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxTCR_PAM;        /** @brief Padding/Alignement mode these bits are set and cleared by software. Case 1: source data size smaller than destination data size - 3 options are valid. Case 2: source data size larger than destination data size. The remainder part is discarded. When PKE = 1 or DSIZE=SSIZE, these bits are ignored. These bits are protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxTCR_TRGM;       /** @brief Trigger mode these bits are set and cleared by software. Note: if TRGM is 11 for the current block, all the values loaded at the end of the current block through the linked list mechanism must keep the same value (TRGM=11) and the same SWRM value, otherwise the result is undefined. These bits are protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxTCR_SWRM;       /** @brief SW request mode this bit is set and cleared by software. If a HW or SW request is currently active, the bit change will be delayed until the current transfer is completed. If the cxmar contains a valid address, the cxmdr value will also be written @ cxmar address. This bit is protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxTCR_BWM;        /** @brief Bufferable write mode this bit is set and cleared by software. This bit is protected and can be written only if EN is 0. Note: all MDMA destination accesses are non-cacheable. */
extern const field32_t MDMA_MDMA_CxBNDTR_BNDT;     /** @brief Block number of data to transfer. */
extern const field32_t MDMA_MDMA_CxBNDTR_BRSUM;    /** @brief Block repeat source address update mode these bits are protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxBNDTR_BRDUM;    /** @brief Block repeat destination address update mode these bits are protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxBNDTR_BRC;      /** @brief Block repeat count this field contains the number of repetitions of the current block (0 to 4095). When the channel is enabled, this register is read-only, indicating the remaining number of blocks, excluding the current one. This register decrements after each complete block transfer. Once the last block transfer has completed, this register can either stay at zero or be reloaded automatically from memory (in linked list mode - i.e. Link address valid). These bits are protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxBRUR_SUV;       /** @brief Source adresse update value. */
extern const field32_t MDMA_MDMA_CxBRUR_DUV;       /** @brief Destination address update. */
extern const field32_t MDMA_MDMA_CxTBR_TSEL;       /** @brief Trigger selection. */
extern const field32_t MDMA_MDMA_CxTBR_SBUS;       /** @brief Source BUS select this bit is protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_CxTBR_DBUS;       /** @brief Destination BUS slect this bit is protected and can be written only if EN is 0. */
extern const field32_t MDMA_MDMA_C1ISR_TEIF1;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C1ISR_CTCIF1;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C1ISR_BRTIF1;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C1ISR_BTIF1;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C1ISR_TCIF1;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C1ISR_CRQA1;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C1IFCR_CTEIF1;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C1IFCR_CCTCIF1;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C1IFCR_CBRTIF1;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C1IFCR_CBTIF1;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C1IFCR_CLTCIF1;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C2ISR_TEIF2;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C2ISR_CTCIF2;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C2ISR_BRTIF2;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C2ISR_BTIF2;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C2ISR_TCIF2;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C2ISR_CRQA2;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C2IFCR_CTEIF2;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C2IFCR_CCTCIF2;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C2IFCR_CBRTIF2;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C2IFCR_CBTIF2;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C2IFCR_CLTCIF2;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C3ISR_TEIF3;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C3ISR_CTCIF3;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C3ISR_BRTIF3;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C3ISR_BTIF3;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C3ISR_TCIF3;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C3ISR_CRQA3;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C3IFCR_CTEIF3;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C3IFCR_CCTCIF3;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C3IFCR_CBRTIF3;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C3IFCR_CBTIF3;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C3IFCR_CLTCIF3;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C4ISR_TEIF4;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C4ISR_CTCIF4;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C4ISR_BRTIF4;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C4ISR_BTIF4;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C4ISR_TCIF4;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C4ISR_CRQA4;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C4IFCR_CTEIF4;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C4IFCR_CCTCIF4;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C4IFCR_CBRTIF4;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C4IFCR_CBTIF4;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C4IFCR_CLTCIF4;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C5ISR_TEIF5;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C5ISR_CTCIF5;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C5ISR_BRTIF5;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C5ISR_BTIF5;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C5ISR_TCIF5;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C5ISR_CRQA5;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C5IFCR_CTEIF5;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C5IFCR_CCTCIF5;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C5IFCR_CBRTIF5;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C5IFCR_CBTIF5;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C5IFCR_CLTCIF5;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C6ISR_TEIF6;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C6ISR_CTCIF6;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C6ISR_BRTIF6;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C6ISR_BTIF6;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C6ISR_TCIF6;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C6ISR_CRQA6;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C6IFCR_CTEIF6;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C6IFCR_CCTCIF6;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C6IFCR_CBRTIF6;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C6IFCR_CBTIF6;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C6IFCR_CLTCIF6;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C7ISR_TEIF7;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C7ISR_CTCIF7;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C7ISR_BRTIF7;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C7ISR_BTIF7;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C7ISR_TCIF7;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C7ISR_CRQA7;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C7IFCR_CTEIF7;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C7IFCR_CCTCIF7;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C7IFCR_CBRTIF7;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C7IFCR_CBTIF7;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C7IFCR_CLTCIF7;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C8ISR_TEIF8;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C8ISR_CTCIF8;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C8ISR_BRTIF8;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C8ISR_BTIF8;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C8ISR_TCIF8;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C8ISR_CRQA8;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C8IFCR_CTEIF8;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C8IFCR_CCTCIF8;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C8IFCR_CBRTIF8;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C8IFCR_CBTIF8;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C8IFCR_CLTCIF8;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C9ISR_TEIF9;      /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C9ISR_CTCIF9;     /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C9ISR_BRTIF9;     /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C9ISR_BTIF9;      /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C9ISR_TCIF9;      /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C9ISR_CRQA9;      /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C9IFCR_CTEIF9;    /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C9IFCR_CCTCIF9;   /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C9IFCR_CBRTIF9;   /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C9IFCR_CBTIF9;    /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C9IFCR_CLTCIF9;   /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C10ISR_TEIF10;    /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C10ISR_CTCIF10;   /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C10ISR_BRTIF10;   /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C10ISR_BTIF10;    /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C10ISR_TCIF10;    /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C10ISR_CRQA10;    /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C10IFCR_CTEIF10;  /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C10IFCR_CCTCIF10; /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C10IFCR_CBRTIF10; /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C10IFCR_CBTIF10;  /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C10IFCR_CLTCIF10; /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C11ISR_TEIF11;    /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C11ISR_CTCIF11;   /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C11ISR_BRTIF11;   /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C11ISR_BTIF11;    /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C11ISR_TCIF11;    /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C11ISR_CRQA11;    /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C11IFCR_CTEIF11;  /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C11IFCR_CCTCIF11; /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C11IFCR_CBRTIF11; /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C11IFCR_CBTIF11;  /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C11IFCR_CLTCIF11; /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C12ISR_TEIF12;    /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C12ISR_CTCIF12;   /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C12ISR_BRTIF12;   /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C12ISR_BTIF12;    /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C12ISR_TCIF12;    /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C12ISR_CRQA12;    /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C12IFCR_CTEIF12;  /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C12IFCR_CCTCIF12; /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C12IFCR_CBRTIF12; /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C12IFCR_CBTIF12;  /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C12IFCR_CLTCIF12; /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C13ISR_TEIF13;    /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C13ISR_CTCIF13;   /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C13ISR_BRTIF13;   /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C13ISR_BTIF13;    /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C13ISR_TCIF13;    /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C13ISR_CRQA13;    /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C13IFCR_CTEIF13;  /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C13IFCR_CCTCIF13; /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C13IFCR_CBRTIF13; /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C13IFCR_CBTIF13;  /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C13IFCR_CLTCIF13; /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C14ISR_TEIF14;    /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C14ISR_CTCIF14;   /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C14ISR_BRTIF14;   /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C14ISR_BTIF14;    /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C14ISR_TCIF14;    /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C14ISR_CRQA14;    /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C14IFCR_CTEIF14;  /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C14IFCR_CCTCIF14; /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C14IFCR_CBRTIF14; /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C14IFCR_CBTIF14;  /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C14IFCR_CLTCIF14; /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C15ISR_TEIF15;    /** @brief Channel x transfer error interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C15ISR_CTCIF15;   /** @brief Channel x channel transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. CTC is set when the last block was transferred and the channel has been automatically disabled. CTC is also set when the channel is suspended, as a result of writing EN bit to 0. */
extern const field32_t MDMA_MDMA_C15ISR_BRTIF15;   /** @brief Channel x block repeat transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C15ISR_BTIF15;    /** @brief Channel x block transfer complete interrupt flag this bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the dma_ifcry register. */
extern const field32_t MDMA_MDMA_C15ISR_TCIF15;    /** @brief Channel x buffer transfer complete. */
extern const field32_t MDMA_MDMA_C15ISR_CRQA15;    /** @brief Channel x request active flag. */
extern const field32_t MDMA_MDMA_C15IFCR_CTEIF15;  /** @brief Channel x clear transfer error interrupt flag writing a 1 into this bit clears teifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C15IFCR_CCTCIF15; /** @brief Clear channel transfer complete interrupt flag for channel x writing a 1 into this bit clears ctcifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C15IFCR_CBRTIF15; /** @brief Channel x clear block repeat transfer complete interrupt flag writing a 1 into this bit clears brtifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C15IFCR_CBTIF15;  /** @brief Channel x clear block transfer complete interrupt flag writing a 1 into this bit clears btifx in the mdma_isry register. */
extern const field32_t MDMA_MDMA_C15IFCR_CLTCIF15; /** @brief CLear buffer transfer complete interrupt flag for channel x writing a 1 into this bit clears tcifx in the mdma_isry register. */

/** @subsection Enumerated MDMA Register Field Definitions */

extern const field32_t MDMA_MDMA_GISR0_GIFx[16]; /** @brief Channel x global interrupt flag (x=...) this bit is set and reset by hardware. It is a logical OR of all the channel x interrupt flags (ctcifx, btifx, brtifx, teifx) which are enabled in the interrupt mask register (ctciex, btiex, brtiex, teiex). */

/**************************************************************************************************
 * @section QUADSPI Definitions
 **************************************************************************************************/

/** @subsection QUADSPI Register Definitions */

extern rw_reg32_t const QUADSPI_CR;    /** @brief QUADSPI control register. */
extern rw_reg32_t const QUADSPI_DCR;   /** @brief QUADSPI device configuration register. */
extern ro_reg32_t const QUADSPI_SR;    /** @brief QUADSPI status register. */
extern rw_reg32_t const QUADSPI_FCR;   /** @brief QUADSPI flag clear register. */
extern rw_reg32_t const QUADSPI_DLR;   /** @brief QUADSPI data length register. */
extern rw_reg32_t const QUADSPI_CCR;   /** @brief QUADSPI communication configuration register. */
extern rw_reg32_t const QUADSPI_AR;    /** @brief QUADSPI address register. */
extern rw_reg32_t const QUADSPI_ABR;   /** @brief QUADSPI alternate bytes registers. */
extern rw_reg32_t const QUADSPI_DR;    /** @brief QUADSPI data register. */
extern rw_reg32_t const QUADSPI_PSMKR; /** @brief QUADSPI polling status mask register. */
extern rw_reg32_t const QUADSPI_PSMAR; /** @brief QUADSPI polling status match register. */
extern rw_reg32_t const QUADSPI_PIR;   /** @brief QUADSPI polling interval register. */
extern rw_reg32_t const QUADSPI_LPTR;  /** @brief QUADSPI low-power timeout register. */

/** @subsection QUADSPI Register Field Definitions */

extern const field32_t QUADSPI_CR_EN;           /** @brief Enable enable the QUADSPI. */
extern const field32_t QUADSPI_CR_ABORT;        /** @brief Abort request this bit aborts the on-going command sequence. It is automatically reset once the abort is complete. This bit stops the current transfer. In polling mode or memory-mapped mode, this bit also reset the APM bit or the DM bit. */
extern const field32_t QUADSPI_CR_DMAEN;        /** @brief DMA enable in indirect mode, DMA can be used to input or output data via the QUADSPI_DR register. DMA transfers are initiated when the FIFO threshold flag, FTF, is set. */
extern const field32_t QUADSPI_CR_TCEN;         /** @brief Timeout counter enable this bit is valid only when memory-mapped mode (FMODE = 11) is selected. Activating this bit causes the chip select (ncs) to be released (and thus reduces consumption) if there has not been an access after a certain amount of time, where this time is defined by TIMEOUT[15:0] (QUADSPI_LPTR). Enable the timeout counter. By default, the QUADSPI never stops its prefetch operation, keeping the previous read operation active with ncs maintained low, even if no access to the flash memory occurs for a long time. Since flash memories tend to consume more when ncs is held low, the application might want to activate the timeout counter (TCEN = 1, QUADSPI_CR[3]) so that ncs is released after a period of TIMEOUT[15:0] (QUADSPI_LPTR) cycles have elapsed without an access since when the FIFO becomes full with prefetch data. This bit can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_CR_SSHIFT;       /** @brief Sample shift by default, the QUADSPI samples data 1/2 of a CLK cycle after the data is driven by the flash memory. This bit allows the data is to be sampled later in order to account for external signal delays. Firmware must assure that SSHIFT = 0 when in DDR mode (when DDRM = 1). This field can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_CR_DFM;          /** @brief Dual-flash mode this bit activates dual-flash mode, where two external flash memories are used simultaneously to double throughput and capacity. This bit can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_CR_FSEL;         /** @brief Flash memory selection this bit selects the flash memory to be addressed in single flash mode (when DFM = 0). This bit can be modified only when BUSY = 0. This bit is ignored when DFM = 1. */
extern const field32_t QUADSPI_CR_FTHRES;       /** @brief FIFO threshold level defines, in indirect mode, the threshold number of bytes in the FIFO that will cause the FIFO threshold flag (FTF, QUADSPI_SR[2]) to be set. In indirect write mode (FMODE = 00): ... In indirect read mode (FMODE = 01): ... If DMAEN = 1, then the DMA controller for the corresponding channel must be disabled before changing the FTHRES value. */
extern const field32_t QUADSPI_CR_TEIE;         /** @brief Transfer error interrupt enable this bit enables the transfer error interrupt. */
extern const field32_t QUADSPI_CR_TCIE;         /** @brief Transfer complete interrupt enable this bit enables the transfer complete interrupt. */
extern const field32_t QUADSPI_CR_FTIE;         /** @brief FIFO threshold interrupt enable this bit enables the FIFO threshold interrupt. */
extern const field32_t QUADSPI_CR_SMIE;         /** @brief Status match interrupt enable this bit enables the status match interrupt. */
extern const field32_t QUADSPI_CR_TOIE;         /** @brief TimeOut interrupt enable this bit enables the timeout interrupt. */
extern const field32_t QUADSPI_CR_APMS;         /** @brief Automatic poll mode stop this bit determines if automatic polling is stopped after a match. This bit can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_CR_PMM;          /** @brief Polling match mode this bit indicates which method should be used for determining a match during automatic polling mode. This bit can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_CR_PRESCALER;    /** @brief Clock prescaler. */
extern const field32_t QUADSPI_DCR_CKMODE;      /** @brief Indicates the level that clk takes between command. */
extern const field32_t QUADSPI_DCR_CSHT;        /** @brief Chip select high time CSHT+1 defines the minimum number of CLK cycles which the chip select (ncs) must remain high between commands issued to the flash memory. ... This field can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_DCR_FSIZE;       /** @brief Flash memory size this field defines the size of external memory using the following formula: number of bytes in flash memory = 2[FSIZE+1] FSIZE+1 is effectively the number of address bits required to address the flash memory. The flash memory capacity can be up to 4GB (addressed using 32 bits) in indirect mode, but the addressable space in memory-mapped mode is limited to 256MB. If DFM = 1, FSIZE indicates the total capacity of the two flash memories together. This field can be modified only when BUSY = 0. */
extern const field32_t QUADSPI_SR_TEF;          /** @brief Transfer error flag this bit is set in indirect mode when an invalid address is being accessed in indirect mode. It is cleared by writing 1 to CTEF. */
extern const field32_t QUADSPI_SR_TCF;          /** @brief Transfer complete flag this bit is set in indirect mode when the programmed number of data has been transferred or in any mode when the transfer has been aborted.it is cleared by writing 1 to CTCF. */
extern const field32_t QUADSPI_SR_FTF;          /** @brief FIFO threshold flag in indirect mode, this bit is set when the FIFO threshold has been reached, or if there is any data left in the FIFO after reads from the flash memory are complete. It is cleared automatically as soon as threshold condition is no longer true. In automatic polling mode this bit is set every time the status register is read, and the bit is cleared when the data register is read. */
extern const field32_t QUADSPI_SR_SMF;          /** @brief Status match flag this bit is set in automatic polling mode when the unmasked received data matches the corresponding bits in the match register (QUADSPI_PSMAR). It is cleared by writing 1 to CSMF. */
extern const field32_t QUADSPI_SR_TOF;          /** @brief Timeout flag this bit is set when timeout occurs. It is cleared by writing 1 to CTOF. */
extern const field32_t QUADSPI_SR_BUSY;         /** @brief Busy this bit is set when an operation is on going. This bit clears automatically when the operation with the flash memory is finished and the FIFO is empty. */
extern const field32_t QUADSPI_SR_FLEVEL;       /** @brief FIFO level this field gives the number of valid bytes which are being held in the FIFO. FLEVEL = 0 when the FIFO is empty, and 16 when it is full. In memory-mapped mode and in automatic status polling mode, FLEVEL is zero. */
extern const field32_t QUADSPI_FCR_CTEF;        /** @brief Clear transfer error flag writing 1 clears the TEF flag in the QUADSPI_SR register. */
extern const field32_t QUADSPI_FCR_CTCF;        /** @brief Clear transfer complete flag writing 1 clears the TCF flag in the QUADSPI_SR register. */
extern const field32_t QUADSPI_FCR_CSMF;        /** @brief Clear status match flag writing 1 clears the SMF flag in the QUADSPI_SR register. */
extern const field32_t QUADSPI_FCR_CTOF;        /** @brief Clear timeout flag writing 1 clears the TOF flag in the QUADSPI_SR register. */
extern const field32_t QUADSPI_CCR_INSTRUCTION; /** @brief Instruction instruction to be send to the external SPI device. This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_IMODE;       /** @brief Instruction mode this field defines the instruction phase mode of operation: this field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_ADMODE;      /** @brief Address mode this field defines the address phase mode of operation: this field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_ADSIZE;      /** @brief Address size this bit defines address size: this field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_ABMODE;      /** @brief Alternate bytes mode this field defines the alternate-bytes phase mode of operation: this field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_ABSIZE;      /** @brief Alternate bytes size this bit defines alternate bytes size: this field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_DCYC;        /** @brief Number of dummy cycles this field defines the duration of the dummy phase. In both SDR and DDR modes, it specifies a number of CLK cycles (0-31). This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_DMODE;       /** @brief Data mode this field defines the data phases mode of operation: this field also determines the dummy phase mode of operation. This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_FMODE;       /** @brief Functional mode this field defines the QUADSPI functional mode of operation. If DMAEN = 1 already, then the DMA controller for the corresponding channel must be disabled before changing the FMODE value. This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_SIOO;        /** @brief Send instruction only once mode see section15.3.11: sending the instruction only once on page13. This bit has no effect when IMODE = 00. This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_DHHC;        /** @brief DDR hold delay the data output by 1/4 of the QUADSPI output clock cycle in DDR mode: this feature is only active in DDR mode. This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_CCR_DDRM;        /** @brief Double data rate mode this bit sets the DDR mode for the address, alternate byte and data phase: this field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_PIR_INTERVAL;    /** @brief Polling interval number of CLK cycles between to read during automatic polling phases. This field can be written only when BUSY = 0. */
extern const field32_t QUADSPI_LPTR_TIMEOUT;    /** @brief Timeout period after each access in memory-mapped mode, the QUADSPI prefetches the subsequent bytes and holds these bytes in the FIFO. This field indicates how many CLK cycles the QUADSPI waits after the FIFO becomes full until it raises ncs, putting the flash memory in a lower-consumption state. This field can be written only when BUSY = 0. */

/**************************************************************************************************
 * @section RNG Definitions
 **************************************************************************************************/

/** @subsection RNG Register Definitions */

extern rw_reg32_t const RNG_CR; /** @brief RNG control register. */
extern rw_reg32_t const RNG_SR; /** @brief RNG status register. */
extern ro_reg32_t const RNG_DR; /** @brief The RNG_DR register is a read-only register that delivers a 32-bit random value when read. The content of this register is valid when DRDY= 1, even if RNGEN=0. */

/** @subsection RNG Register Field Definitions */

extern const field32_t RNG_CR_RNGEN; /** @brief Random number generator enable. */
extern const field32_t RNG_CR_IE;    /** @brief Interrupt enable. */
extern const field32_t RNG_CR_CED;   /** @brief Clock error detection note: the clock error detection can be used only when ck_rc48 or ck_pll1_q (ck_pll1_q = 48mhz) source is selected otherwise, CED bit must be equal to 1. The clock error detection cannot be enabled nor disabled on the fly when RNG peripheral is enabled, to enable or disable CED the RNG must be disabled. */
extern const field32_t RNG_SR_DRDY;  /** @brief Data ready note: if IE=1 in RNG_CR, an interrupt is generated when DRDY=1. It can rise when the peripheral is disabled. When the output buffer becomes empty (after reading RNG_DR), this bit returns to 0 until a new random value is generated. */
extern const field32_t RNG_SR_CECS;  /** @brief Clock error current status note: this bit is meaningless if CED (clock error detection) bit in RNG_CR is equal to 1. */
extern const field32_t RNG_SR_SECS;  /** @brief Seed error current status ** more than 64 consecutive bits at the same value (0 or 1) ** more than 32 consecutive alternances of 0 and 1 (0101010101...01). */
extern const field32_t RNG_SR_CEIS;  /** @brief Clock error interrupt status this bit is set at the same time as CECS. It is cleared by writing it to 0. An interrupt is pending if IE = 1 in the RNG_CR register. Note: this bit is meaningless if CED (clock error detection) bit in RNG_CR is equal to 1. */
extern const field32_t RNG_SR_SEIS;  /** @brief Seed error interrupt status this bit is set at the same time as SECS. It is cleared by writing it to 0. ** more than 64 consecutive bits at the same value (0 or 1) ** more than 32 consecutive alternances of 0 and 1 (0101010101...01) an interrupt is pending if IE = 1 in the RNG_CR register. */

/**************************************************************************************************
 * @section RTC Definitions
 **************************************************************************************************/

/** @subsection RTC Register Definitions */

extern rw_reg32_t const RTC_RTC_TR;       /** @brief The RTC_TR is the calendar time shadow register. This register must be written in initialization mode only. Refer to calendar initialization and configuration on page9 and reading the calendar on page10.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_DR;       /** @brief The RTC_DR is the calendar date shadow register. This register must be written in initialization mode only. Refer to calendar initialization and configuration on page9 and reading the calendar on page10.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_CR;       /** @brief RTC control register. */
extern rw_reg32_t const RTC_RTC_ISR;      /** @brief This register is write protected (except for RTC_ISR[13:8] bits). The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_PRER;     /** @brief This register must be written in initialization mode only. The initialization must be performed in two separate write accesses. Refer to calendar initialization and configuration on page9.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_WUTR;     /** @brief This register can be written only when WUTWF is set to 1 in rtc_isr.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_ALRMAR;   /** @brief This register can be written only when ALRAWF is set to 1 in RTC_ISR, or in initialization mode.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_ALRMBR;   /** @brief This register can be written only when ALRBWF is set to 1 in RTC_ISR, or in initialization mode.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_WPR;      /** @brief RTC write protection register. */
extern ro_reg32_t const RTC_RTC_SSR;      /** @brief RTC sub second register. */
extern rw_reg32_t const RTC_RTC_SHIFTR;   /** @brief This register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern ro_reg32_t const RTC_RTC_TSTR;     /** @brief The content of this register is valid only when TSF is set to 1 in RTC_ISR. It is cleared when TSF bit is reset. */
extern ro_reg32_t const RTC_RTC_TSDR;     /** @brief The content of this register is valid only when TSF is set to 1 in RTC_ISR. It is cleared when TSF bit is reset. */
extern ro_reg32_t const RTC_RTC_TSSSR;    /** @brief The content of this register is valid only when RTC_ISR/TSF is set. It is cleared when the RTC_ISR/TSF bit is reset. */
extern rw_reg32_t const RTC_RTC_CALR;     /** @brief This register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_TAMPCR;   /** @brief RTC tamper and alternate function configuration register. */
extern rw_reg32_t const RTC_RTC_ALRMASSR; /** @brief This register can be written only when ALRAE is reset in RTC_CR register, or in initialization mode.this register is write protected. The write access procedure is described in RTC register write protection on page9. */
extern rw_reg32_t const RTC_RTC_ALRMBSSR; /** @brief This register can be written only when ALRBE is reset in RTC_CR register, or in initialization mode.this register is write protected.the write access procedure is described in section: RTC register write protection. */
extern rw_reg32_t const RTC_RTC_OR;       /** @brief RTC option register. */

/** @subsection Enumerated RTC Register Definitions */

extern rw_reg32_t const RTC_RTC_BKPxR[32]; /** @brief RTC backup registers. */

/** @subsection RTC Register Field Definitions */

extern const field32_t RTC_RTC_TR_SU;             /** @brief Second units in BCD format. */
extern const field32_t RTC_RTC_TR_ST;             /** @brief Second tens in BCD format. */
extern const field32_t RTC_RTC_TR_MNU;            /** @brief Minute units in BCD format. */
extern const field32_t RTC_RTC_TR_MNT;            /** @brief Minute tens in BCD format. */
extern const field32_t RTC_RTC_TR_HU;             /** @brief Hour units in BCD format. */
extern const field32_t RTC_RTC_TR_HT;             /** @brief Hour tens in BCD format. */
extern const field32_t RTC_RTC_TR_PM;             /** @brief AM/PM notation. */
extern const field32_t RTC_RTC_DR_DU;             /** @brief Date units in BCD format. */
extern const field32_t RTC_RTC_DR_DT;             /** @brief Date tens in BCD format. */
extern const field32_t RTC_RTC_DR_MU;             /** @brief Month units in BCD format. */
extern const field32_t RTC_RTC_DR_MT;             /** @brief Month tens in BCD format. */
extern const field32_t RTC_RTC_DR_WDU;            /** @brief Week day units. */
extern const field32_t RTC_RTC_DR_YU;             /** @brief Year units in BCD format. */
extern const field32_t RTC_RTC_DR_YT;             /** @brief Year tens in BCD format. */
extern const field32_t RTC_RTC_CR_WUCKSEL;        /** @brief Wakeup clock selection. */
extern const field32_t RTC_RTC_CR_TSEDGE;         /** @brief Time-stamp event active edge TSE must be reset when TSEDGE is changed to avoid unwanted TSF setting. */
extern const field32_t RTC_RTC_CR_REFCKON;        /** @brief RTC_REFIN reference clock detection enable (50 or 60hz) note: PREDIV_S must be 0x00ff. */
extern const field32_t RTC_RTC_CR_BYPSHAD;        /** @brief Bypass the shadow registers note: if the frequency of the APB clock is less than seven times the frequency of RTCCLK, BYPSHAD must be set to 1. */
extern const field32_t RTC_RTC_CR_FMT;            /** @brief Hour format. */
extern const field32_t RTC_RTC_CR_ALRAE;          /** @brief Alarm A enable. */
extern const field32_t RTC_RTC_CR_ALRBE;          /** @brief Alarm B enable. */
extern const field32_t RTC_RTC_CR_WUTE;           /** @brief Wakeup timer enable. */
extern const field32_t RTC_RTC_CR_TSE;            /** @brief Timestamp enable. */
extern const field32_t RTC_RTC_CR_ALRAIE;         /** @brief Alarm A interrupt enable. */
extern const field32_t RTC_RTC_CR_ALRBIE;         /** @brief Alarm B interrupt enable. */
extern const field32_t RTC_RTC_CR_WUTIE;          /** @brief Wakeup timer interrupt enable. */
extern const field32_t RTC_RTC_CR_TSIE;           /** @brief Time-stamp interrupt enable. */
extern const field32_t RTC_RTC_CR_ADD1H;          /** @brief Add 1 hour (summer time change) when this bit is set outside initialization mode, 1 hour is added to the calendar time. This bit is always read as 0. */
extern const field32_t RTC_RTC_CR_SUB1H;          /** @brief Subtract 1 hour (winter time change) when this bit is set outside initialization mode, 1 hour is subtracted to the calendar time if the current hour is not 0. This bit is always read as 0. Setting this bit has no effect when current hour is 0. */
extern const field32_t RTC_RTC_CR_BKP;            /** @brief Backup this bit can be written by the user to memorize whether the daylight saving time change has been performed or not. */
extern const field32_t RTC_RTC_CR_COSEL;          /** @brief Calibration output selection when COE=1, this bit selects which signal is output on RTC_CALIB. These frequencies are valid for RTCCLK at 32.768 khz and prescalers at their default values (PREDIV_A=127 and PREDIV_S=255). Refer to section24.3.15: calibration clock output. */
extern const field32_t RTC_RTC_CR_POL;            /** @brief Output polarity this bit is used to configure the polarity of RTC_ALARM output. */
extern const field32_t RTC_RTC_CR_OSEL;           /** @brief Output selection these bits are used to select the flag to be routed to RTC_ALARM output. */
extern const field32_t RTC_RTC_CR_COE;            /** @brief Calibration output enable this bit enables the RTC_CALIB output. */
extern const field32_t RTC_RTC_CR_ITSE;           /** @brief Timestamp on internal event enable. */
extern const field32_t RTC_RTC_ISR_ALRAWF;        /** @brief Alarm A write flag this bit is set by hardware when alarm A values can be changed, after the ALRAE bit has been set to 0 in RTC_CR. It is cleared by hardware in initialization mode. */
extern const field32_t RTC_RTC_ISR_ALRBWF;        /** @brief Alarm B write flag this bit is set by hardware when alarm B values can be changed, after the ALRBE bit has been set to 0 in RTC_CR. It is cleared by hardware in initialization mode. */
extern const field32_t RTC_RTC_ISR_WUTWF;         /** @brief Wakeup timer write flag this bit is set by hardware up to 2 RTCCLK cycles after the WUTE bit has been set to 0 in RTC_CR, and is cleared up to 2 RTCCLK cycles after the WUTE bit has been set to 1. The wakeup timer values can be changed when WUTE bit is cleared and WUTWF is set. */
extern const field32_t RTC_RTC_ISR_SHPF;          /** @brief Shift operation pending this flag is set by hardware as soon as a shift operation is initiated by a write to the RTC_SHIFTR register. It is cleared by hardware when the corresponding shift operation has been executed. Writing to the SHPF bit has no effect. */
extern const field32_t RTC_RTC_ISR_INITS;         /** @brief Initialization status flag this bit is set by hardware when the calendar year field is different from 0 (backup domain reset state). */
extern const field32_t RTC_RTC_ISR_RSF;           /** @brief Registers synchronization flag this bit is set by hardware each time the calendar registers are copied into the shadow registers (rtc_ssrx, rtc_trx and rtc_drx). This bit is cleared by hardware in initialization mode, while a shift operation is pending (SHPF=1), or when in bypass shadow register mode (BYPSHAD=1). This bit can also be cleared by software. It is cleared either by software or by hardware in initialization mode. */
extern const field32_t RTC_RTC_ISR_INITF;         /** @brief Initialization flag when this bit is set to 1, the RTC is in initialization state, and the time, date and prescaler registers can be updated. */
extern const field32_t RTC_RTC_ISR_INIT;          /** @brief Initialization mode. */
extern const field32_t RTC_RTC_ISR_ALRAF;         /** @brief Alarm A flag this flag is set by hardware when the time/date registers (RTC_TR and RTC_DR) match the alarm A register (RTC_ALRMAR). This flag is cleared by software by writing 0. */
extern const field32_t RTC_RTC_ISR_ALRBF;         /** @brief Alarm B flag this flag is set by hardware when the time/date registers (RTC_TR and RTC_DR) match the alarm B register (RTC_ALRMBR). This flag is cleared by software by writing 0. */
extern const field32_t RTC_RTC_ISR_WUTF;          /** @brief Wakeup timer flag this flag is set by hardware when the wakeup auto-reload counter reaches 0. This flag is cleared by software by writing 0. This flag must be cleared by software at least 1.5 RTCCLK periods before WUTF is set to 1 again. */
extern const field32_t RTC_RTC_ISR_TSF;           /** @brief Time-stamp flag this flag is set by hardware when a time-stamp event occurs. This flag is cleared by software by writing 0. */
extern const field32_t RTC_RTC_ISR_TSOVF;         /** @brief Time-stamp overflow flag this flag is set by hardware when a time-stamp event occurs while TSF is already set. This flag is cleared by software by writing 0. It is recommended to check and then clear TSOVF only after clearing the TSF bit. Otherwise, an overflow might not be noticed if a time-stamp event occurs immediately before the TSF bit is cleared. */
extern const field32_t RTC_RTC_ISR_RECALPF;       /** @brief Recalibration pending flag the RECALPF status flag is automatically set to 1 when software writes to the RTC_CALR register, indicating that the RTC_CALR register is blocked. When the new calibration settings are taken into account, this bit returns to 0. Refer to re-calibration on-the-fly. */
extern const field32_t RTC_RTC_ISR_ITSF;          /** @brief Internal ttime-stamp flag. */
extern const field32_t RTC_RTC_PRER_PREDIV_S;     /** @brief Synchronous prescaler factor this is the synchronous division factor: ck_spre frequency = ck_apre frequency/(prediv_s+1). */
extern const field32_t RTC_RTC_PRER_PREDIV_A;     /** @brief Asynchronous prescaler factor this is the asynchronous division factor: ck_apre frequency = RTCCLK frequency/(prediv_a+1). */
extern const field32_t RTC_RTC_WUTR_WUT;          /** @brief Wakeup auto-reload value bits when the wakeup timer is enabled (WUTE set to 1), the WUTF flag is set every (WUT[15:0] + 1) ck_wut cycles. The ck_wut period is selected through WUCKSEL[2:0] bits of the RTC_CR register when WUCKSEL[2] = 1, the wakeup timer becomes 17-bits and WUCKSEL[1] effectively becomes WUT[16] the most-significant bit to be reloaded into the timer. The first assertion of WUTF occurs (WUT+1) ck_wut cycles after WUTE is set. Setting WUT[15:0] to 0x0000 with WUCKSEL[2:0] =011 (RTCCLK/2) is forbidden. */
extern const field32_t RTC_RTC_ALRMAR_SU;         /** @brief Second units in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_ST;         /** @brief Second tens in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_MNU;        /** @brief Minute units in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_MNT;        /** @brief Minute tens in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_HU;         /** @brief Hour units in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_HT;         /** @brief Hour tens in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_PM;         /** @brief AM/PM notation. */
extern const field32_t RTC_RTC_ALRMAR_DU;         /** @brief Date units or day in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_DT;         /** @brief Date tens in BCD format. */
extern const field32_t RTC_RTC_ALRMAR_WDSEL;      /** @brief Week day selection. */
extern const field32_t RTC_RTC_ALRMBR_SU;         /** @brief Second units in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_ST;         /** @brief Second tens in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_MNU;        /** @brief Minute units in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_MNT;        /** @brief Minute tens in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_HU;         /** @brief Hour units in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_HT;         /** @brief Hour tens in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_PM;         /** @brief AM/PM notation. */
extern const field32_t RTC_RTC_ALRMBR_DU;         /** @brief Date units or day in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_DT;         /** @brief Date tens in BCD format. */
extern const field32_t RTC_RTC_ALRMBR_WDSEL;      /** @brief Week day selection. */
extern const field32_t RTC_RTC_WPR_KEY;           /** @brief Write protection key this byte is written by software. Reading this byte always returns 0x00. Refer to RTC register write protection for a description of how to unlock RTC register write protection. */
extern const field32_t RTC_RTC_SSR_SS;            /** @brief Sub second value SS[15:0] is the value in the synchronous prescaler counter. The fraction of a second is given by the formula below: second fraction = (PREDIV_S - SS) / (PREDIV_S + 1) note: SS can be larger than PREDIV_S only after a shift operation. In that case, the correct time/date is one second less than as indicated by RTC_TR/RTC_DR. */
extern const field32_t RTC_RTC_SHIFTR_SUBFS;      /** @brief Subtract a fraction of a second these bits are write only and is always read as zero. Writing to this bit has no effect when a shift operation is pending (when SHPF=1, in RTC_ISR). The value which is written to SUBFS is added to the synchronous prescaler counter. Since this counter counts down, this operation effectively subtracts from (delays) the clock by: delay (seconds) = SUBFS / (PREDIV_S + 1) A fraction of a second can effectively be added to the clock (advancing the clock) when the ADD1S function is used in conjunction with SUBFS, effectively advancing the clock by: advance (seconds) = (1 - (SUBFS / (PREDIV_S + 1))). Note: writing to SUBFS causes RSF to be cleared. Software can then wait until RSF=1 to be sure that the shadow registers have been updated with the shifted time. */
extern const field32_t RTC_RTC_SHIFTR_ADD1S;      /** @brief Add one second this bit is write only and is always read as zero. Writing to this bit has no effect when a shift operation is pending (when SHPF=1, in RTC_ISR). This function is intended to be used with SUBFS (see description below) in order to effectively add a fraction of a second to the clock in an atomic operation. */
extern const field32_t RTC_RTC_TSTR_SU;           /** @brief Second units in BCD format. */
extern const field32_t RTC_RTC_TSTR_ST;           /** @brief Second tens in BCD format. */
extern const field32_t RTC_RTC_TSTR_MNU;          /** @brief Minute units in BCD format. */
extern const field32_t RTC_RTC_TSTR_MNT;          /** @brief Minute tens in BCD format. */
extern const field32_t RTC_RTC_TSTR_HU;           /** @brief Hour units in BCD format. */
extern const field32_t RTC_RTC_TSTR_HT;           /** @brief Hour tens in BCD format. */
extern const field32_t RTC_RTC_TSTR_PM;           /** @brief AM/PM notation. */
extern const field32_t RTC_RTC_TSDR_DU;           /** @brief Date units in BCD format. */
extern const field32_t RTC_RTC_TSDR_DT;           /** @brief Date tens in BCD format. */
extern const field32_t RTC_RTC_TSDR_MU;           /** @brief Month units in BCD format. */
extern const field32_t RTC_RTC_TSDR_MT;           /** @brief Month tens in BCD format. */
extern const field32_t RTC_RTC_TSDR_WDU;          /** @brief Week day units. */
extern const field32_t RTC_RTC_TSSSR_SS;          /** @brief Sub second value SS[15:0] is the value of the synchronous prescaler counter when the timestamp event occurred. */
extern const field32_t RTC_RTC_CALR_CALM;         /** @brief Calibration minus the frequency of the calendar is reduced by masking CALM out of 220 RTCCLK pulses (32 seconds if the input frequency is 32768 hz). This decreases the frequency of the calendar with a resolution of 0.9537 ppm. To increase the frequency of the calendar, this feature should be used in conjunction with CALP. See section24.3.12: RTC smooth digital calibration on page13. */
extern const field32_t RTC_RTC_CALR_CALP;         /** @brief Increase frequency of RTC by 488.5 ppm this feature is intended to be used in conjunction with CALM, which lowers the frequency of the calendar with a fine resolution. If the input frequency is 32768 hz, the number of RTCCLK pulses added during a 32-second window is calculated as follows: (512 * CALP) - CALM. Refer to section24.3.12: RTC smooth digital calibration. */
extern const field32_t RTC_RTC_TAMPCR_TAMPIE;     /** @brief Tamper interrupt enable. */
extern const field32_t RTC_RTC_TAMPCR_TAMPTS;     /** @brief Activate timestamp on tamper detection event TAMPTS is valid even if TSE=0 in the RTC_CR register. */
extern const field32_t RTC_RTC_TAMPCR_TAMPFREQ;   /** @brief Tamper sampling frequency determines the frequency at which each of the rtc_tampx inputs are sampled. */
extern const field32_t RTC_RTC_TAMPCR_TAMPFLT;    /** @brief RTC_TAMPx filter count these bits determines the number of consecutive samples at the specified level (TAMP*TRG) needed to activate a tamper event. TAMPFLT is valid for each of the rtc_tampx inputs. */
extern const field32_t RTC_RTC_TAMPCR_TAMPPRCH;   /** @brief RTC_TAMPx precharge duration these bit determines the duration of time during which the pull-up/is activated before each sample. TAMPPRCH is valid for each of the rtc_tampx inputs. */
extern const field32_t RTC_RTC_TAMPCR_TAMPPUDIS;  /** @brief RTC_TAMPx pull-up disable this bit determines if each of the rtc_tampx pins are pre-charged before each sample. */
extern const field32_t RTC_RTC_ALRMASSR_SS;       /** @brief Sub seconds value this value is compared with the contents of the synchronous prescaler counter to determine if alarm A is to be activated. Only bits 0 up MASKSS-1 are compared. */
extern const field32_t RTC_RTC_ALRMASSR_MASKSS;   /** @brief Mask the most-significant bits starting at this bit ... The overflow bits of the synchronous counter (bits 15) is never compared. This bit can be different from 0 only after a shift operation. */
extern const field32_t RTC_RTC_ALRMBSSR_SS;       /** @brief Sub seconds value this value is compared with the contents of the synchronous prescaler counter to determine if alarm B is to be activated. Only bits 0 up to MASKSS-1 are compared. */
extern const field32_t RTC_RTC_ALRMBSSR_MASKSS;   /** @brief Mask the most-significant bits starting at this bit ... The overflow bits of the synchronous counter (bits 15) is never compared. This bit can be different from 0 only after a shift operation. */
extern const field32_t RTC_RTC_OR_RTC_ALARM_TYPE; /** @brief RTC_ALARM output type on PC13. */
extern const field32_t RTC_RTC_OR_RTC_OUT_RMP;    /** @brief RTC_OUT remap. */

/** @subsection Enumerated RTC Register Field Definitions */

extern const field32_t RTC_RTC_ISR_TAMPxF[4];          /** @brief RTC_TAMP1 detection flag this flag is set by hardware when a tamper detection event is detected on the RTC_TAMP1 input. It is cleared by software writing 0. */
extern const field32_t RTC_RTC_ALRMAR_MSKx[5];         /** @brief Alarm A seconds mask. */
extern const field32_t RTC_RTC_CALR_CALWx[17];         /** @brief Use an 8-second calibration cycle period when CALW8 is set to 1, the 8-second calibration cycle period is selected. Note: CALM[1:0] are stuck at 00; when CALW8= 1. Refer to section24.3.12: RTC smooth digital calibration. */
extern const field32_t RTC_RTC_TAMPCR_TAMPxE[4];       /** @brief RTC_TAMP1 input detection enable. */
extern const field32_t RTC_RTC_TAMPCR_TAMPxTRG[4];     /** @brief Active level for RTC_TAMP1 input if TAMPFLT != 00 if TAMPFLT = 00:. */
extern const field32_t RTC_RTC_TAMPCR_TAMPxIE[4];      /** @brief Tamper 1 interrupt enable. */
extern const field32_t RTC_RTC_TAMPCR_TAMPxNOERASE[4]; /** @brief Tamper 1 no erase. */
extern const field32_t RTC_RTC_TAMPCR_TAMPxMF[4];      /** @brief Tamper 1 mask flag. */

/**************************************************************************************************
 * @section SAIx Definitions
 **************************************************************************************************/

/** @subsection Enumerated SAIx Register Definitions */

extern rw_reg32_t const SAIx_SAI_GCR[5];    /** @brief Global configuration register. */
extern rw_reg32_t const SAIx_SAI_ACR1[5];   /** @brief Configuration register 1. */
extern rw_reg32_t const SAIx_SAI_ACR2[5];   /** @brief Configuration register 2. */
extern rw_reg32_t const SAIx_SAI_AFRCR[5];  /** @brief This register has no meaning in AC97 and SPDIF audio protocol. */
extern rw_reg32_t const SAIx_SAI_ASLOTR[5]; /** @brief This register has no meaning in AC97 and SPDIF audio protocol. */
extern rw_reg32_t const SAIx_SAI_AIM[5];    /** @brief Interrupt mask register 2. */
extern ro_reg32_t const SAIx_SAI_ASR[5];    /** @brief Status register. */
extern rw_reg32_t const SAIx_SAI_ACLRFR[5]; /** @brief Clear flag register. */
extern rw_reg32_t const SAIx_SAI_ADR[5];    /** @brief Data register. */
extern rw_reg32_t const SAIx_SAI_BCR1[5];   /** @brief Configuration register 1. */
extern rw_reg32_t const SAIx_SAI_BCR2[5];   /** @brief Configuration register 2. */
extern rw_reg32_t const SAIx_SAI_BFRCR[5];  /** @brief This register has no meaning in AC97 and SPDIF audio protocol. */
extern rw_reg32_t const SAIx_SAI_BSLOTR[5]; /** @brief This register has no meaning in AC97 and SPDIF audio protocol. */
extern rw_reg32_t const SAIx_SAI_BIM[5];    /** @brief Interrupt mask register 2. */
extern ro_reg32_t const SAIx_SAI_BSR[5];    /** @brief Status register. */
extern rw_reg32_t const SAIx_SAI_BCLRFR[5]; /** @brief Clear flag register. */
extern rw_reg32_t const SAIx_SAI_BDR[5];    /** @brief Data register. */
extern rw_reg32_t const SAIx_SAI_PDMCR[5];  /** @brief PDM control register. */
extern rw_reg32_t const SAIx_SAI_PDMDLY[5]; /** @brief PDM delay register. */

/** @subsection SAIx Register Field Definitions */

extern const field32_t SAIx_SAI_GCR_SYNCOUT;     /** @brief Synchronization outputs these bits are set and cleared by software. */
extern const field32_t SAIx_SAI_GCR_SYNCIN;      /** @brief Synchronization inputs. */
extern const field32_t SAIx_SAI_ACR1_MODE;       /** @brief SAIx audio block mode immediately. */
extern const field32_t SAIx_SAI_ACR1_PRTCFG;     /** @brief Protocol configuration. These bits are set and cleared by software. These bits have to be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_ACR1_DS;         /** @brief Data size. These bits are set and cleared by software. These bits are ignored when the SPDIF protocols are selected (bit PRTCFG[1:0]), because the frame and the data size are fixed in such case. When the companding mode is selected through COMP[1:0] bits, DS[1:0] are ignored since the data size is fixed to 8 bits by the algorithm. These bits must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_ACR1_LSBFIRST;   /** @brief Least significant bit first. This bit is set and cleared by software. It must be configured when the audio block is disabled. This bit has no meaning in AC97 audio protocol since AC97 data are always transferred with the MSB first. This bit has no meaning in SPDIF audio protocol since in SPDIF data are always transferred with LSB first. */
extern const field32_t SAIx_SAI_ACR1_CKSTR;      /** @brief Clock strobing edge. This bit is set and cleared by software. It must be configured when the audio block is disabled. This bit has no meaning in SPDIF audio protocol. */
extern const field32_t SAIx_SAI_ACR1_SYNCEN;     /** @brief Synchronization enable. These bits are set and cleared by software. They must be configured when the audio sub-block is disabled. Note: the audio sub-block should be configured as asynchronous when SPDIF mode is enabled. */
extern const field32_t SAIx_SAI_ACR1_MONO;       /** @brief Mono mode. This bit is set and cleared by software. It is meaningful only when the number of slots is equal to 2. When the mono mode is selected, slot 0 data are duplicated on slot 1 when the audio block operates as a transmitter. In reception mode, the slot1 is discarded and only the data received from slot 0 are stored. Refer to section: mono/stereo mode for more details. */
extern const field32_t SAIx_SAI_ACR1_OUTDRIV;    /** @brief Output drive. This bit is set and cleared by software. Note: this bit has to be set before enabling the audio block and after the audio block configuration. */
extern const field32_t SAIx_SAI_ACR1_SAIXEN;     /** @brief Audio block enable where x is A or B. This bit is set by software. To switch off the audio block, the application software must program this bit to 0 and poll the bit till it reads back 0, meaning that the block is completely disabled. Before setting this bit to 1, check that it is set to 0, otherwise the enable command will not be taken into account. This bit allows to control the state of saix audio block. If it is disabled when an audio frame transfer is ongoing, the ongoing transfer completes and the cell is fully disabled at the end of this audio frame transfer. Note: when saix block is configured in master mode, the clock must be present on the input of saix before setting SAIXEN bit. */
extern const field32_t SAIx_SAI_ACR1_DMAEN;      /** @brief DMA enable. This bit is set and cleared by software. Note: since the audio block defaults to operate as a transmitter after reset, the MODE[1:0] bits must be configured before setting DMAEN to avoid a DMA request in receiver mode. */
extern const field32_t SAIx_SAI_ACR1_NOMCK;      /** @brief No divider. */
extern const field32_t SAIx_SAI_ACR1_MCKDIV;     /** @brief Master clock divider. These bits are set and cleared by software. These bits are meaningless when the audio block operates in slave mode. They have to be configured when the audio block is disabled. Others: the master clock frequency is calculated accordingly to the following formula:. */
extern const field32_t SAIx_SAI_ACR1_OSR;        /** @brief Oversampling ratio for master clock. */
extern const field32_t SAIx_SAI_ACR2_FTH;        /** @brief FIFO threshold. This bit is set and cleared by software. */
extern const field32_t SAIx_SAI_ACR2_FFLUSH;     /** @brief FIFO flush. This bit is set by software. It is always read as 0. This bit should be configured when the SAI is disabled. */
extern const field32_t SAIx_SAI_ACR2_TRIS;       /** @brief Tristate management on data line. This bit is set and cleared by software. It is meaningful only if the audio block is configured as a transmitter. This bit is not used when the audio block is configured in SPDIF mode. It should be configured when SAI is disabled. Refer to section: output data line management on an inactive slot for more details. */
extern const field32_t SAIx_SAI_ACR2_MUTE;       /** @brief Mute. This bit is set and cleared by software. It is meaningful only when the audio block operates as a transmitter. The MUTE value is linked to value of MUTEVAL if the number of slots is lower or equal to 2, or equal to 0 if it is greater than 2. Refer to section: mute mode for more details. Note: this bit is meaningless and should not be used for SPDIF audio blocks. */
extern const field32_t SAIx_SAI_ACR2_MUTEVAL;    /** @brief Mute value. This bit is set and cleared by software.it must be written before enabling the audio block: SAIXEN. This bit is meaningful only when the audio block operates as a transmitter, the number of slots is lower or equal to 2 and the MUTE bit is set. If more slots are declared, the bit value sent during the transmission in mute mode is equal to 0, whatever the value of MUTEVAL. If the number of slot is lower or equal to 2 and MUTEVAL = 1, the MUTE value transmitted for each slot is the one sent during the previous frame. Refer to section: mute mode for more details. Note: this bit is meaningless and should not be used for SPDIF audio blocks. */
extern const field32_t SAIx_SAI_ACR2_MUTECNT;    /** @brief Mute counter. These bits are set and cleared by software. They are used only in reception mode. The value set in these bits is compared to the number of consecutive mute frames detected in reception. When the number of mute frames is equal to this value, the flag MUTEDET will be set and an interrupt will be generated if bit MUTEDETIE is set. Refer to section: mute mode for more details. */
extern const field32_t SAIx_SAI_ACR2_CPL;        /** @brief Complement bit. This bit is set and cleared by software. It defines the type of complement to be used for companding mode note: this bit has effect only when the companding mode is -law algorithm or a-law algorithm. */
extern const field32_t SAIx_SAI_ACR2_COMP;       /** @brief Companding mode. These bits are set and cleared by software. The -law and the a-law log are a part of the CCITT G.711 recommendation, the type of complement that will be used depends on CPL bit. The data expansion or data compression are determined by the state of bit MODE[0]. The data compression is applied if the audio block is configured as a transmitter. The data expansion is automatically applied when the audio block is configured as a receiver. Refer to section: companding mode for more details. Note: companding mode is applicable only when TDM is selected. */
extern const field32_t SAIx_SAI_AFRCR_FRL;       /** @brief Frame length. These bits are set and cleared by software. They define the audio frame length expressed in number of SCK clock cycles: the number of bits in the frame is equal to FRL[7:0] + 1. The minimum number of bits to transfer in an audio frame must be equal to 8, otherwise the audio block will behaves in an unexpected way. This is the case when the data size is 8 bits and only one slot 0 is defined in NBSLOT[4:0] of sai_xslotr register (NBSLOT[3:0] = 0000). In master mode, if the master clock (available on mclk_x pin) is used, the frame length should be aligned with a number equal to a power of 2, ranging from 8 to 256. When the master clock is not used (NODIV = 1), it is recommended to program the frame length to an value ranging from 8 to 256. These bits are meaningless and are not used in AC97 or SPDIF audio block configuration. */
extern const field32_t SAIx_SAI_AFRCR_FSALL;     /** @brief Frame synchronization active level length. These bits are set and cleared by software. They specify the length in number of bit clock (SCK) + 1 (FSALL[6:0] + 1) of the active level of the FS signal in the audio frame these bits are meaningless and are not used in AC97 or SPDIF audio block configuration. They must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_AFRCR_FSDEF;     /** @brief Frame synchronization definition. This bit is set and cleared by software. When the bit is set, the number of slots defined in the sai_xslotr register has to be even. It means that half of this number of slots will be dedicated to the left channel and the other slots for the right channel (e.g: this bit has to be set for I2S or msb/lsb-justified protocols...). This bit is meaningless and is not used in AC97 or SPDIF audio block configuration. It must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_AFRCR_FSPOL;     /** @brief Frame synchronization polarity. This bit is set and cleared by software. It is used to configure the level of the start of frame on the FS signal. It is meaningless and is not used in AC97 or SPDIF audio block configuration. This bit must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_AFRCR_FSOFF;     /** @brief Frame synchronization offset. This bit is set and cleared by software. It is meaningless and is not used in AC97 or SPDIF audio block configuration. This bit must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_ASLOTR_FBOFF;    /** @brief First bit offset these bits are set and cleared by software. The value set in this bitfield defines the position of the first data transfer bit in the slot. It represents an offset value. In transmission mode, the bits outside the data field are forced to 0. In reception mode, the extra received bits are discarded. These bits must be set when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_ASLOTR_SLOTSZ;   /** @brief Slot size this bits is set and cleared by software. The slot size must be higher or equal to the data size. If this condition is not respected, the behavior of the SAI will be undetermined. Refer to section: output data line management on an inactive slot for information on how to drive SD line. These bits must be set when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_ASLOTR_NBSLOT;   /** @brief Number of slots in an audio frame. These bits are set and cleared by software. The value set in this bitfield represents the number of slots + 1 in the audio frame (including the number of inactive slots). The maximum number of slots is 16. The number of slots should be even if FSDEF bit in the sai_xfrcr register is set. The number of slots must be configured when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_ASLOTR_SLOTEN;   /** @brief Slot enable. These bits are set and cleared by software. Each SLOTEN bit corresponds to a slot position from 0 to 15 (maximum 16 slots). The slot must be enabled when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_AIM_OVRUDRIE;    /** @brief Overrun/underrun interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt is generated if the OVRUDR bit in the sai_xsr register is set. */
extern const field32_t SAIx_SAI_AIM_MUTEDETIE;   /** @brief Mute detection interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt is generated if the MUTEDET bit in the sai_xsr register is set. This bit has a meaning only if the audio block is configured in receiver mode. */
extern const field32_t SAIx_SAI_AIM_WCKCFGIE;    /** @brief Wrong clock configuration interrupt enable. This bit is set and cleared by software. This bit is taken into account only if the audio block is configured as a master (MODE[1] = 0) and NODIV = 0. It generates an interrupt if the WCKCFG flag in the sai_xsr register is set. Note: this bit is used only in TDM mode and is meaningless in other modes. */
extern const field32_t SAIx_SAI_AIM_FREQIE;      /** @brief FIFO request interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt is generated if the FREQ bit in the sai_xsr register is set. Since the audio block defaults to operate as a transmitter after reset, the MODE bit must be configured before setting FREQIE to avoid a parasitic interruption in receiver mode,. */
extern const field32_t SAIx_SAI_AIM_CNRDYIE;     /** @brief Codec not ready interrupt enable (AC97). This bit is set and cleared by software. When the interrupt is enabled, the audio block detects in the slot 0 (tag0) of the AC97 frame if the codec connected to this line is ready or not. If it is not ready, the CNRDY flag in the sai_xsr register is set and an interruption i generated. This bit has a meaning only if the AC97 mode is selected through PRTCFG[1:0] bits and the audio block is operates as a receiver. */
extern const field32_t SAIx_SAI_AIM_AFSDETIE;    /** @brief Anticipated frame synchronization detection interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt will be generated if the AFSDET bit in the sai_xsr register is set. This bit is meaningless in AC97, SPDIF mode or when the audio block operates as a master. */
extern const field32_t SAIx_SAI_AIM_LFSDETIE;    /** @brief Late frame synchronization detection interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt will be generated if the LFSDET bit is set in the sai_xsr register. This bit is meaningless in AC97, SPDIF mode or when the audio block operates as a master. */
extern const field32_t SAIx_SAI_ASR_OVRUDR;      /** @brief Overrun / underrun. This bit is read only. The overrun and underrun conditions can occur only when the audio block is configured as a receiver and a transmitter, respectively. It can generate an interrupt if OVRUDRIE bit is set in sai_xim register. This flag is cleared when the software sets COVRUDR bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_ASR_MUTEDET;     /** @brief Mute detection. This bit is read only. This flag is set if consecutive 0 values are received in each slot of a given audio frame and for a consecutive number of audio frames (set in the MUTECNT bit in the sai_xcr2 register). It can generate an interrupt if MUTEDETIE bit is set in sai_xim register. This flag is cleared when the software sets bit CMUTEDET in the sai_xclrfr register. */
extern const field32_t SAIx_SAI_ASR_WCKCFG;      /** @brief Wrong clock configuration flag. This bit is read only. This bit is used only when the audio block operates in master mode (MODE[1] = 0) and NODIV = 0. It can generate an interrupt if WCKCFGIE bit is set in sai_xim register. This flag is cleared when the software sets CWCKCFG bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_ASR_FREQ;        /** @brief FIFO request. This bit is read only. The request depends on the audio block configuration: if the block is configured in transmission mode, the FIFO request is related to a write request operation in the sai_xdr. If the block configured in reception, the FIFO request related to a read request operation from the sai_xdr. This flag can generate an interrupt if FREQIE bit is set in sai_xim register. */
extern const field32_t SAIx_SAI_ASR_CNRDY;       /** @brief Codec not ready. This bit is read only. This bit is used only when the AC97 audio protocol is selected in the sai_xcr1 register and configured in receiver mode. It can generate an interrupt if CNRDYIE bit is set in sai_xim register. This flag is cleared when the software sets CCNRDY bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_ASR_AFSDET;      /** @brief Anticipated frame synchronization detection. This bit is read only. This flag can be set only if the audio block is configured in slave mode. It is not used in ac97or SPDIF mode. It can generate an interrupt if AFSDETIE bit is set in sai_xim register. This flag is cleared when the software sets CAFSDET bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_ASR_LFSDET;      /** @brief Late frame synchronization detection. This bit is read only. This flag can be set only if the audio block is configured in slave mode. It is not used in AC97 or SPDIF mode. It can generate an interrupt if LFSDETIE bit is set in the sai_xim register. This flag is cleared when the software sets bit CLFSDET in sai_xclrfr register. */
extern const field32_t SAIx_SAI_ASR_FLVL;        /** @brief FIFO level threshold. This bit is read only. The FIFO level threshold flag is managed only by hardware and its setting depends on SAI block configuration (transmitter or receiver mode). If the SAI block is configured as transmitter: if SAI block is configured as receiver:. */
extern const field32_t SAIx_SAI_ACLRFR_COVRUDR;  /** @brief Clear overrun / underrun. This bit is write only. Programming this bit to 1 clears the OVRUDR flag in the sai_xsr register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_ACLRFR_CMUTEDET; /** @brief Mute detection flag. This bit is write only. Programming this bit to 1 clears the MUTEDET flag in the sai_xsr register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_ACLRFR_CWCKCFG;  /** @brief Clear wrong clock configuration flag. This bit is write only. Programming this bit to 1 clears the WCKCFG flag in the sai_xsr register. This bit is used only when the audio block is set as master (MODE[1] = 0) and NODIV = 0 in the sai_xcr1 register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_ACLRFR_CCNRDY;   /** @brief Clear codec not ready flag. This bit is write only. Programming this bit to 1 clears the CNRDY flag in the sai_xsr register. This bit is used only when the AC97 audio protocol is selected in the sai_xcr1 register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_ACLRFR_CAFSDET;  /** @brief Clear anticipated frame synchronization detection flag. This bit is write only. Programming this bit to 1 clears the AFSDET flag in the sai_xsr register. It is not used in ac97or SPDIF mode. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_ACLRFR_CLFSDET;  /** @brief Clear late frame synchronization detection flag. This bit is write only. Programming this bit to 1 clears the LFSDET flag in the sai_xsr register. This bit is not used in ac97or SPDIF mode reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_BCR1_MODE;       /** @brief SAIx audio block mode immediately. */
extern const field32_t SAIx_SAI_BCR1_PRTCFG;     /** @brief Protocol configuration. These bits are set and cleared by software. These bits have to be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_BCR1_DS;         /** @brief Data size. These bits are set and cleared by software. These bits are ignored when the SPDIF protocols are selected (bit PRTCFG[1:0]), because the frame and the data size are fixed in such case. When the companding mode is selected through COMP[1:0] bits, DS[1:0] are ignored since the data size is fixed to 8 bits by the algorithm. These bits must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_BCR1_LSBFIRST;   /** @brief Least significant bit first. This bit is set and cleared by software. It must be configured when the audio block is disabled. This bit has no meaning in AC97 audio protocol since AC97 data are always transferred with the MSB first. This bit has no meaning in SPDIF audio protocol since in SPDIF data are always transferred with LSB first. */
extern const field32_t SAIx_SAI_BCR1_CKSTR;      /** @brief Clock strobing edge. This bit is set and cleared by software. It must be configured when the audio block is disabled. This bit has no meaning in SPDIF audio protocol. */
extern const field32_t SAIx_SAI_BCR1_SYNCEN;     /** @brief Synchronization enable. These bits are set and cleared by software. They must be configured when the audio sub-block is disabled. Note: the audio sub-block should be configured as asynchronous when SPDIF mode is enabled. */
extern const field32_t SAIx_SAI_BCR1_MONO;       /** @brief Mono mode. This bit is set and cleared by software. It is meaningful only when the number of slots is equal to 2. When the mono mode is selected, slot 0 data are duplicated on slot 1 when the audio block operates as a transmitter. In reception mode, the slot1 is discarded and only the data received from slot 0 are stored. Refer to section: mono/stereo mode for more details. */
extern const field32_t SAIx_SAI_BCR1_OUTDRIV;    /** @brief Output drive. This bit is set and cleared by software. Note: this bit has to be set before enabling the audio block and after the audio block configuration. */
extern const field32_t SAIx_SAI_BCR1_SAIXEN;     /** @brief Audio block enable where x is A or B. This bit is set by software. To switch off the audio block, the application software must program this bit to 0 and poll the bit till it reads back 0, meaning that the block is completely disabled. Before setting this bit to 1, check that it is set to 0, otherwise the enable command will not be taken into account. This bit allows to control the state of saix audio block. If it is disabled when an audio frame transfer is ongoing, the ongoing transfer completes and the cell is fully disabled at the end of this audio frame transfer. Note: when saix block is configured in master mode, the clock must be present on the input of saix before setting SAIXEN bit. */
extern const field32_t SAIx_SAI_BCR1_DMAEN;      /** @brief DMA enable. This bit is set and cleared by software. Note: since the audio block defaults to operate as a transmitter after reset, the MODE[1:0] bits must be configured before setting DMAEN to avoid a DMA request in receiver mode. */
extern const field32_t SAIx_SAI_BCR1_NOMCK;      /** @brief No divider. */
extern const field32_t SAIx_SAI_BCR1_MCKDIV;     /** @brief Master clock divider. These bits are set and cleared by software. These bits are meaningless when the audio block operates in slave mode. They have to be configured when the audio block is disabled. Others: the master clock frequency is calculated accordingly to the following formula:. */
extern const field32_t SAIx_SAI_BCR1_OSR;        /** @brief Oversampling ratio for master clock. */
extern const field32_t SAIx_SAI_BCR2_FTH;        /** @brief FIFO threshold. This bit is set and cleared by software. */
extern const field32_t SAIx_SAI_BCR2_FFLUSH;     /** @brief FIFO flush. This bit is set by software. It is always read as 0. This bit should be configured when the SAI is disabled. */
extern const field32_t SAIx_SAI_BCR2_TRIS;       /** @brief Tristate management on data line. This bit is set and cleared by software. It is meaningful only if the audio block is configured as a transmitter. This bit is not used when the audio block is configured in SPDIF mode. It should be configured when SAI is disabled. Refer to section: output data line management on an inactive slot for more details. */
extern const field32_t SAIx_SAI_BCR2_MUTE;       /** @brief Mute. This bit is set and cleared by software. It is meaningful only when the audio block operates as a transmitter. The MUTE value is linked to value of MUTEVAL if the number of slots is lower or equal to 2, or equal to 0 if it is greater than 2. Refer to section: mute mode for more details. Note: this bit is meaningless and should not be used for SPDIF audio blocks. */
extern const field32_t SAIx_SAI_BCR2_MUTEVAL;    /** @brief Mute value. This bit is set and cleared by software.it must be written before enabling the audio block: SAIXEN. This bit is meaningful only when the audio block operates as a transmitter, the number of slots is lower or equal to 2 and the MUTE bit is set. If more slots are declared, the bit value sent during the transmission in mute mode is equal to 0, whatever the value of MUTEVAL. If the number of slot is lower or equal to 2 and MUTEVAL = 1, the MUTE value transmitted for each slot is the one sent during the previous frame. Refer to section: mute mode for more details. Note: this bit is meaningless and should not be used for SPDIF audio blocks. */
extern const field32_t SAIx_SAI_BCR2_MUTECNT;    /** @brief Mute counter. These bits are set and cleared by software. They are used only in reception mode. The value set in these bits is compared to the number of consecutive mute frames detected in reception. When the number of mute frames is equal to this value, the flag MUTEDET will be set and an interrupt will be generated if bit MUTEDETIE is set. Refer to section: mute mode for more details. */
extern const field32_t SAIx_SAI_BCR2_CPL;        /** @brief Complement bit. This bit is set and cleared by software. It defines the type of complement to be used for companding mode note: this bit has effect only when the companding mode is -law algorithm or a-law algorithm. */
extern const field32_t SAIx_SAI_BCR2_COMP;       /** @brief Companding mode. These bits are set and cleared by software. The -law and the a-law log are a part of the CCITT G.711 recommendation, the type of complement that will be used depends on CPL bit. The data expansion or data compression are determined by the state of bit MODE[0]. The data compression is applied if the audio block is configured as a transmitter. The data expansion is automatically applied when the audio block is configured as a receiver. Refer to section: companding mode for more details. Note: companding mode is applicable only when TDM is selected. */
extern const field32_t SAIx_SAI_BFRCR_FRL;       /** @brief Frame length. These bits are set and cleared by software. They define the audio frame length expressed in number of SCK clock cycles: the number of bits in the frame is equal to FRL[7:0] + 1. The minimum number of bits to transfer in an audio frame must be equal to 8, otherwise the audio block will behaves in an unexpected way. This is the case when the data size is 8 bits and only one slot 0 is defined in NBSLOT[4:0] of sai_xslotr register (NBSLOT[3:0] = 0000). In master mode, if the master clock (available on mclk_x pin) is used, the frame length should be aligned with a number equal to a power of 2, ranging from 8 to 256. When the master clock is not used (NODIV = 1), it is recommended to program the frame length to an value ranging from 8 to 256. These bits are meaningless and are not used in AC97 or SPDIF audio block configuration. */
extern const field32_t SAIx_SAI_BFRCR_FSALL;     /** @brief Frame synchronization active level length. These bits are set and cleared by software. They specify the length in number of bit clock (SCK) + 1 (FSALL[6:0] + 1) of the active level of the FS signal in the audio frame these bits are meaningless and are not used in AC97 or SPDIF audio block configuration. They must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_BFRCR_FSDEF;     /** @brief Frame synchronization definition. This bit is set and cleared by software. When the bit is set, the number of slots defined in the sai_xslotr register has to be even. It means that half of this number of slots will be dedicated to the left channel and the other slots for the right channel (e.g: this bit has to be set for I2S or msb/lsb-justified protocols...). This bit is meaningless and is not used in AC97 or SPDIF audio block configuration. It must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_BFRCR_FSPOL;     /** @brief Frame synchronization polarity. This bit is set and cleared by software. It is used to configure the level of the start of frame on the FS signal. It is meaningless and is not used in AC97 or SPDIF audio block configuration. This bit must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_BFRCR_FSOFF;     /** @brief Frame synchronization offset. This bit is set and cleared by software. It is meaningless and is not used in AC97 or SPDIF audio block configuration. This bit must be configured when the audio block is disabled. */
extern const field32_t SAIx_SAI_BSLOTR_FBOFF;    /** @brief First bit offset these bits are set and cleared by software. The value set in this bitfield defines the position of the first data transfer bit in the slot. It represents an offset value. In transmission mode, the bits outside the data field are forced to 0. In reception mode, the extra received bits are discarded. These bits must be set when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_BSLOTR_SLOTSZ;   /** @brief Slot size this bits is set and cleared by software. The slot size must be higher or equal to the data size. If this condition is not respected, the behavior of the SAI will be undetermined. Refer to section: output data line management on an inactive slot for information on how to drive SD line. These bits must be set when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_BSLOTR_NBSLOT;   /** @brief Number of slots in an audio frame. These bits are set and cleared by software. The value set in this bitfield represents the number of slots + 1 in the audio frame (including the number of inactive slots). The maximum number of slots is 16. The number of slots should be even if FSDEF bit in the sai_xfrcr register is set. The number of slots must be configured when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_BSLOTR_SLOTEN;   /** @brief Slot enable. These bits are set and cleared by software. Each SLOTEN bit corresponds to a slot position from 0 to 15 (maximum 16 slots). The slot must be enabled when the audio block is disabled. They are ignored in AC97 or SPDIF mode. */
extern const field32_t SAIx_SAI_BIM_OVRUDRIE;    /** @brief Overrun/underrun interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt is generated if the OVRUDR bit in the sai_xsr register is set. */
extern const field32_t SAIx_SAI_BIM_MUTEDETIE;   /** @brief Mute detection interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt is generated if the MUTEDET bit in the sai_xsr register is set. This bit has a meaning only if the audio block is configured in receiver mode. */
extern const field32_t SAIx_SAI_BIM_WCKCFGIE;    /** @brief Wrong clock configuration interrupt enable. This bit is set and cleared by software. This bit is taken into account only if the audio block is configured as a master (MODE[1] = 0) and NODIV = 0. It generates an interrupt if the WCKCFG flag in the sai_xsr register is set. Note: this bit is used only in TDM mode and is meaningless in other modes. */
extern const field32_t SAIx_SAI_BIM_FREQIE;      /** @brief FIFO request interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt is generated if the FREQ bit in the sai_xsr register is set. Since the audio block defaults to operate as a transmitter after reset, the MODE bit must be configured before setting FREQIE to avoid a parasitic interruption in receiver mode,. */
extern const field32_t SAIx_SAI_BIM_CNRDYIE;     /** @brief Codec not ready interrupt enable (AC97). This bit is set and cleared by software. When the interrupt is enabled, the audio block detects in the slot 0 (tag0) of the AC97 frame if the codec connected to this line is ready or not. If it is not ready, the CNRDY flag in the sai_xsr register is set and an interruption i generated. This bit has a meaning only if the AC97 mode is selected through PRTCFG[1:0] bits and the audio block is operates as a receiver. */
extern const field32_t SAIx_SAI_BIM_AFSDETIE;    /** @brief Anticipated frame synchronization detection interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt will be generated if the AFSDET bit in the sai_xsr register is set. This bit is meaningless in AC97, SPDIF mode or when the audio block operates as a master. */
extern const field32_t SAIx_SAI_BIM_LFSDETIE;    /** @brief Late frame synchronization detection interrupt enable. This bit is set and cleared by software. When this bit is set, an interrupt will be generated if the LFSDET bit is set in the sai_xsr register. This bit is meaningless in AC97, SPDIF mode or when the audio block operates as a master. */
extern const field32_t SAIx_SAI_BSR_OVRUDR;      /** @brief Overrun / underrun. This bit is read only. The overrun and underrun conditions can occur only when the audio block is configured as a receiver and a transmitter, respectively. It can generate an interrupt if OVRUDRIE bit is set in sai_xim register. This flag is cleared when the software sets COVRUDR bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_BSR_MUTEDET;     /** @brief Mute detection. This bit is read only. This flag is set if consecutive 0 values are received in each slot of a given audio frame and for a consecutive number of audio frames (set in the MUTECNT bit in the sai_xcr2 register). It can generate an interrupt if MUTEDETIE bit is set in sai_xim register. This flag is cleared when the software sets bit CMUTEDET in the sai_xclrfr register. */
extern const field32_t SAIx_SAI_BSR_WCKCFG;      /** @brief Wrong clock configuration flag. This bit is read only. This bit is used only when the audio block operates in master mode (MODE[1] = 0) and NODIV = 0. It can generate an interrupt if WCKCFGIE bit is set in sai_xim register. This flag is cleared when the software sets CWCKCFG bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_BSR_FREQ;        /** @brief FIFO request. This bit is read only. The request depends on the audio block configuration: if the block is configured in transmission mode, the FIFO request is related to a write request operation in the sai_xdr. If the block configured in reception, the FIFO request related to a read request operation from the sai_xdr. This flag can generate an interrupt if FREQIE bit is set in sai_xim register. */
extern const field32_t SAIx_SAI_BSR_CNRDY;       /** @brief Codec not ready. This bit is read only. This bit is used only when the AC97 audio protocol is selected in the sai_xcr1 register and configured in receiver mode. It can generate an interrupt if CNRDYIE bit is set in sai_xim register. This flag is cleared when the software sets CCNRDY bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_BSR_AFSDET;      /** @brief Anticipated frame synchronization detection. This bit is read only. This flag can be set only if the audio block is configured in slave mode. It is not used in ac97or SPDIF mode. It can generate an interrupt if AFSDETIE bit is set in sai_xim register. This flag is cleared when the software sets CAFSDET bit in sai_xclrfr register. */
extern const field32_t SAIx_SAI_BSR_LFSDET;      /** @brief Late frame synchronization detection. This bit is read only. This flag can be set only if the audio block is configured in slave mode. It is not used in AC97 or SPDIF mode. It can generate an interrupt if LFSDETIE bit is set in the sai_xim register. This flag is cleared when the software sets bit CLFSDET in sai_xclrfr register. */
extern const field32_t SAIx_SAI_BSR_FLVL;        /** @brief FIFO level threshold. This bit is read only. The FIFO level threshold flag is managed only by hardware and its setting depends on SAI block configuration (transmitter or receiver mode). If the SAI block is configured as transmitter: if SAI block is configured as receiver:. */
extern const field32_t SAIx_SAI_BCLRFR_COVRUDR;  /** @brief Clear overrun / underrun. This bit is write only. Programming this bit to 1 clears the OVRUDR flag in the sai_xsr register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_BCLRFR_CMUTEDET; /** @brief Mute detection flag. This bit is write only. Programming this bit to 1 clears the MUTEDET flag in the sai_xsr register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_BCLRFR_CWCKCFG;  /** @brief Clear wrong clock configuration flag. This bit is write only. Programming this bit to 1 clears the WCKCFG flag in the sai_xsr register. This bit is used only when the audio block is set as master (MODE[1] = 0) and NODIV = 0 in the sai_xcr1 register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_BCLRFR_CCNRDY;   /** @brief Clear codec not ready flag. This bit is write only. Programming this bit to 1 clears the CNRDY flag in the sai_xsr register. This bit is used only when the AC97 audio protocol is selected in the sai_xcr1 register. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_BCLRFR_CAFSDET;  /** @brief Clear anticipated frame synchronization detection flag. This bit is write only. Programming this bit to 1 clears the AFSDET flag in the sai_xsr register. It is not used in ac97or SPDIF mode. Reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_BCLRFR_CLFSDET;  /** @brief Clear late frame synchronization detection flag. This bit is write only. Programming this bit to 1 clears the LFSDET flag in the sai_xsr register. This bit is not used in ac97or SPDIF mode reading this bit always returns the value 0. */
extern const field32_t SAIx_SAI_PDMCR_PDMEN;     /** @brief PDM enable. */
extern const field32_t SAIx_SAI_PDMCR_MICNBR;    /** @brief Number of microphones. */

/** @subsection Enumerated SAIx Register Field Definitions */

extern const field32_t SAIx_SAI_PDMCR_CKENx[5];   /** @brief Clock enable of bitstream clock number 1. */
extern const field32_t SAIx_SAI_PDMDLY_DLYMxL[5]; /** @brief Delay line adjust for first microphone of pair 1. */
extern const field32_t SAIx_SAI_PDMDLY_DLYMxR[5]; /** @brief Delay line adjust for second microphone of pair 1. */

/**************************************************************************************************
 * @section SDMMCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated SDMMCx Register Definitions */

extern rw_reg32_t const SDMMCx_POWER[3];      /** @brief SDMMC power control register. */
extern rw_reg32_t const SDMMCx_CLKCR[3];      /** @brief The SDMMC_CLKCR register controls the SDMMC_CK output clock, the SDMMC_RX_CLK receive clock, and the bus width. */
extern rw_reg32_t const SDMMCx_ARGR[3];       /** @brief The SDMMC_ARGR register contains a 32-bit command argument, which is sent to a card as part of a command message. */
extern rw_reg32_t const SDMMCx_CMDR[3];       /** @brief The SDMMC_CMDR register contains the command index and command type bits. The command index is sent to a card as part of a command message. The command type bits control the command path state machine (CPSM). */
extern ro_reg32_t const SDMMCx_RESP1R[3];     /** @brief The SDMMC_RESP1/2/3/4R registers contain the status of a card, which is part of the received response. */
extern ro_reg32_t const SDMMCx_RESP2R[3];     /** @brief The SDMMC_RESP1/2/3/4R registers contain the status of a card, which is part of the received response. */
extern ro_reg32_t const SDMMCx_RESP3R[3];     /** @brief The SDMMC_RESP1/2/3/4R registers contain the status of a card, which is part of the received response. */
extern ro_reg32_t const SDMMCx_RESP4R[3];     /** @brief The SDMMC_RESP1/2/3/4R registers contain the status of a card, which is part of the received response. */
extern rw_reg32_t const SDMMCx_DTIMER[3];     /** @brief The SDMMC_DTIMER register contains the data timeout period, in card bus clock periods. A counter loads the value from the SDMMC_DTIMER register, and starts decrementing when the data path state machine (DPSM) enters the wait_r or busy state. If the timer reaches 0 while the DPSM is in either of these states, the timeout status flag is set. */
extern rw_reg32_t const SDMMCx_DLENR[3];      /** @brief The SDMMC_DLENR register contains the number of data bytes to be transferred. The value is loaded into the data counter when data transfer starts. */
extern rw_reg32_t const SDMMCx_DCTRL[3];      /** @brief The SDMMC_DCTRL register control the data path state machine (DPSM). */
extern ro_reg32_t const SDMMCx_DCNTR[3];      /** @brief The SDMMC_DCNTR register loads the value from the data length register (see SDMMC_DLENR) when the DPSM moves from the idle state to the wait_r or wait_s state. As data is transferred, the counter decrements the value until it reaches 0. The DPSM then moves to the idle state and when there has been no error, the data status end flag (DATAEND) is set. */
extern ro_reg32_t const SDMMCx_STAR[3];       /** @brief The SDMMC_STAR register is a read-only register. It contains two types of flag:static flags (bits [29,21,11:0]): these bits remain asserted until they are cleared by writing to the SDMMC interrupt clear register (see sdmmc_icr)dynamic flags (bits [20:12]): these bits change state depending on the state of the underlying logic (for example, FIFO full and empty flags are asserted and de-asserted as data while written to the FIFO). */
extern rw_reg32_t const SDMMCx_ICR[3];        /** @brief The SDMMC_ICR register is a write-only register. Writing a bit with 1 clears the corresponding bit in the SDMMC_STAR status register. */
extern rw_reg32_t const SDMMCx_MASKR[3];      /** @brief The interrupt mask register determines which status flags generate an interrupt request by setting the corresponding bit to 1. */
extern rw_reg32_t const SDMMCx_ACKTIMER[3];   /** @brief The SDMMC_ACKTIMER register contains the acknowledgment timeout period, in SDMMC_CK bus clock periods. A counter loads the value from the SDMMC_ACKTIMER register, and starts decrementing when the data path state machine (DPSM) enters the wait_ack state. If the timer reaches 0 while the DPSM is in this states, the acknowledgment timeout status flag is set. */
extern rw_reg32_t const SDMMCx_IDMACTRLR[3];  /** @brief The receive and transmit fifos can be read or written as 32-bit wide registers. The fifos contain 32 entries on 32 sequential addresses. This allows the CPU to use its load and store multiple operands to read from/write to the FIFO. */
extern rw_reg32_t const SDMMCx_IDMABSIZER[3]; /** @brief The SDMMC_IDMABSIZER register contains the buffers size when in double buffer configuration. */
extern rw_reg32_t const SDMMCx_IDMABASE0R[3]; /** @brief The SDMMC_IDMABASE0R register contains the memory buffer base address in single buffer configuration and the buffer 0 base address in double buffer configuration. */
extern rw_reg32_t const SDMMCx_IDMABASE1R[3]; /** @brief The SDMMC_IDMABASE1R register contains the double buffer configuration second buffer memory base address. */
extern rw_reg32_t const SDMMCx_FIFOR[3];      /** @brief The receive and transmit fifos can be only read or written as word (32-bit) wide registers. The fifos contain 16 entries on sequential addresses. This allows the CPU to use its load and store multiple operands to read from/write to the fifo.when accessing SDMMC_FIFOR with half word or byte access an AHB bus fault is generated. */
extern ro_reg32_t const SDMMCx_RESPCMDR[3];   /** @brief SDMMC command response register. */

/** @subsection SDMMCx Register Field Definitions */

extern const field32_t SDMMCx_POWER_PWRCTRL;       /** @brief SDMMC state control bits. These bits can only be written when the SDMMC is not in the power-on state (PWRCTRL?11). These bits are used to define the functional state of the SDMMC signals: any further write will be ignored, PWRCTRL value will keep 11. */
extern const field32_t SDMMCx_POWER_VSWITCH;       /** @brief Voltage switch sequence start. This bit is used to start the timing critical section of the voltage switch sequence:. */
extern const field32_t SDMMCx_POWER_VSWITCHEN;     /** @brief Voltage switch procedure enable. This bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). This bit is used to stop the SDMMC_CK after the voltage switch command response:. */
extern const field32_t SDMMCx_POWER_DIRPOL;        /** @brief Data and command direction signals polarity selection. This bit can only be written when the SDMMC is in the power-off state (PWRCTRL = 00). */
extern const field32_t SDMMCx_CLKCR_CLKDIV;        /** @brief Clock divide factor this bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0). This field defines the divide factor between the input clock (SDMMCCLK) and the output clock (SDMMC_CK): SDMMC_CK frequency = SDMMCCLK / [2 * CLKDIV]. 0xx: etc.. Xxx: etc.. */
extern const field32_t SDMMCx_CLKCR_PWRSAV;        /** @brief Power saving configuration bit this bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0) for power saving, the SDMMC_CK clock output can be disabled when the bus is idle by setting PWRSAV:. */
extern const field32_t SDMMCx_CLKCR_WIDBUS;        /** @brief Wide bus mode enable bit this bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0). */
extern const field32_t SDMMCx_CLKCR_NEGEDGE;       /** @brief SDMMC_CK dephasing selection bit for data and command. This bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0). When clock division = 1 (CLKDIV = 0), this bit has no effect. Data and command change on SDMMC_CK falling edge. When clock division &gt;1 (CLKDIV &gt; 0) &amp; DDR = 0: - SDMMC_CK edge occurs on SDMMCCLK rising edge. When clock division >1 (CLKDIV > 0) & DDR = 1: - data changed on the SDMMCCLK falling edge succeeding a SDMMC_CK edge. - SDMMC_CK edge occurs on SDMMCCLK rising edge. - data changed on the SDMMC_CK falling edge succeeding a SDMMC_CK edge. - SDMMC_CK edge occurs on SDMMCCLK rising edge. */
extern const field32_t SDMMCx_CLKCR_HWFC_EN;       /** @brief Hardware flow control enable this bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0) when hardware flow control is enabled, the meaning of the TXFIFOE and RXFIFOF flags change, please see SDMMC status register definition in section56.8.11. */
extern const field32_t SDMMCx_CLKCR_DDR;           /** @brief Data rate signaling selection this bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0) DDR rate shall only be selected with 4-bit or 8-bit wide bus mode. (WIDBUS &gt; 00). DDR = 1 has no effect when WIDBUS = 00 (1-bit wide bus). DDR rate shall only be selected with clock division &gt;1. (CLKDIV &gt; 0). */
extern const field32_t SDMMCx_CLKCR_BUSSPEED;      /** @brief Bus speed mode selection between DS, HS, SDR12, SDR25 and SDR50, DDR50, SDR104. This bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0). */
extern const field32_t SDMMCx_CLKCR_SELCLKRX;      /** @brief Receive clock selection. These bits can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0). */
extern const field32_t SDMMCx_CMDR_CMDINDEX;       /** @brief Command index. This bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). The command index is sent to the card as part of a command message. */
extern const field32_t SDMMCx_CMDR_CMDTRANS;       /** @brief The CPSM treats the command as a data transfer command, stops the interrupt period, and signals dataenable to the DPSM this bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). If this bit is set, the CPSM issues an end of interrupt period and issues dataenable signal to the DPSM when the command is sent. */
extern const field32_t SDMMCx_CMDR_CMDSTOP;        /** @brief The CPSM treats the command as a stop transmission command and signals abort to the DPSM. This bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). If this bit is set, the CPSM issues the abort signal to the DPSM when the command is sent. */
extern const field32_t SDMMCx_CMDR_WAITRESP;       /** @brief Wait for response bits. This bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). They are used to configure whether the CPSM is to wait for a response, and if yes, which kind of response. */
extern const field32_t SDMMCx_CMDR_WAITINT;        /** @brief CPSM waits for interrupt request. If this bit is set, the CPSM disables command timeout and waits for an card interrupt request (response). If this bit is cleared in the CPSM wait state, will cause the abort of the interrupt mode. */
extern const field32_t SDMMCx_CMDR_WAITPEND;       /** @brief CPSM waits for end of data transfer (cmdpend internal signal) from DPSM. This bit when set, the CPSM waits for the end of data transfer trigger before it starts sending a command. WAITPEND is only taken into account when DTMODE = MMC stream data transfer, WIDBUS = 1-bit wide bus mode, DPSMACT = 1 and DTDIR = from host to card. */
extern const field32_t SDMMCx_CMDR_CPSMEN;         /** @brief Command path state machine (CPSM) enable bit this bit is written 1 by firmware, and cleared by hardware when the CPSM enters the idle state. If this bit is set, the CPSM is enabled. When DTEN = 1, no command will be transfered nor boot procedure will be started. CPSMEN is cleared to 0. */
extern const field32_t SDMMCx_CMDR_DTHOLD;         /** @brief Hold new data block transmission and reception in the DPSM. If this bit is set, the DPSM will not move from the wait_s state to the send state or from the wait_r state to the receive state. */
extern const field32_t SDMMCx_CMDR_BOOTMODE;       /** @brief Select the boot mode procedure to be used. This bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). */
extern const field32_t SDMMCx_CMDR_BOOTEN;         /** @brief Enable boot mode procedure. */
extern const field32_t SDMMCx_CMDR_CMDSUSPEND;     /** @brief The CPSM treats the command as a suspend or resume command and signals interrupt period start/end. This bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). CMDSUSPEND = 1 and CMDTRANS = 0 suspend command, start interrupt period when response bit BS=0. CMDSUSPEND = 1 and CMDTRANS = 1 resume command with data, end interrupt period when response bit DF=1. */
extern const field32_t SDMMCx_DLENR_DATALENGTH;    /** @brief Data length value this register can only be written by firmware when DPSM is inactive (DPSMACT = 0). Number of data bytes to be transferred. When DDR = 1 DATALENGTH is truncated to a multiple of 2. (The last odd byte is not transfered) when DATALENGTH = 0 no data will be transfered, when requested by a CPSMEN and CMDTRANS = 1 also no command will be transfered. DTEN and CPSMEN are cleared to 0. */
extern const field32_t SDMMCx_DCTRL_DTEN;          /** @brief Data transfer enable bit this bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). This bit is cleared by hardware when data transfer completes. This bit shall only be used to transfer data when no associated data transfer command is used, i.e. Shall not be used with SD or emmc cards. */
extern const field32_t SDMMCx_DCTRL_DTDIR;         /** @brief Data transfer direction selection this bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_DCTRL_DTMODE;        /** @brief Data transfer mode selection. This bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_DCTRL_DBLOCKSIZE;    /** @brief Data block size this bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). Define the data block length when the block data transfer mode is selected: when DATALENGTH is not a multiple of DBLOCKSIZE, the transfered data is truncated at a multiple of DBLOCKSIZE. (Any remain data will not be transfered.) when DDR = 1, DBLOCKSIZE = 0000 shall not be used. (No data will be transfered). */
extern const field32_t SDMMCx_DCTRL_RWSTART;       /** @brief Read wait start. If this bit is set, read wait operation starts. */
extern const field32_t SDMMCx_DCTRL_RWSTOP;        /** @brief Read wait stop this bit is written by firmware and auto cleared by hardware when the DPSM moves from the READ_WAIT state to the WAIT_R or IDLE state. */
extern const field32_t SDMMCx_DCTRL_RWMOD;         /** @brief Read wait mode. This bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_DCTRL_SDIOEN;        /** @brief SD I/O interrupt enable functions this bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). If this bit is set, the DPSM enables the SD I/O card specific interrupt operation. */
extern const field32_t SDMMCx_DCTRL_BOOTACKEN;     /** @brief Enable the reception of the boot acknowledgment. This bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_DCTRL_FIFORST;       /** @brief FIFO reset, will flush any remaining data. This bit can only be written by firmware when IDMAEN= 0 and DPSM is active (DPSMACT = 1). This bit will only take effect when a transfer error or transfer hold occurs. */
extern const field32_t SDMMCx_DCNTR_DATACOUNT;     /** @brief Data count value when read, the number of remaining data bytes to be transferred is returned. Write has no effect. */
extern const field32_t SDMMCx_STAR_CCRCFAIL;       /** @brief Command response received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_DCRCFAIL;       /** @brief Data block sent/received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_CTIMEOUT;       /** @brief Command response timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. The command timeout period has a fixed value of 64 SDMMC_CK clock periods. */
extern const field32_t SDMMCx_STAR_DTIMEOUT;       /** @brief Data timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_TXUNDERR;       /** @brief Transmit FIFO underrun error or IDMA read transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_RXOVERR;        /** @brief Received FIFO overrun error or IDMA write transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_CMDREND;        /** @brief Command response received (CRC check passed, or no CRC). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_CMDSENT;        /** @brief Command sent (no response required). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_DATAEND;        /** @brief Data transfer ended correctly. (data counter, DATACOUNT is zero and no errors occur). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_DHOLD;          /** @brief Data transfer hold. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_DBCKEND;        /** @brief Data block sent/received. (CRC check passed) and DPSM moves to the READWAIT state. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_DABORT;         /** @brief Data transfer aborted by CMD12. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_DPSMACT;        /** @brief Data path state machine active, i.e. Not in idle state. This is a hardware status flag only, does not generate an interrupt. */
extern const field32_t SDMMCx_STAR_CPSMACT;        /** @brief Command path state machine active, i.e. Not in idle state. This is a hardware status flag only, does not generate an interrupt. */
extern const field32_t SDMMCx_STAR_TXFIFOHE;       /** @brief Transmit FIFO half empty at least half the number of words can be written into the FIFO. This bit is cleared when the FIFO becomes half+1 full. */
extern const field32_t SDMMCx_STAR_RXFIFOHF;       /** @brief Receive FIFO half full there are at least half the number of words in the FIFO. This bit is cleared when the FIFO becomes half+1 empty. */
extern const field32_t SDMMCx_STAR_TXFIFOF;        /** @brief Transmit FIFO full this is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes empty. */
extern const field32_t SDMMCx_STAR_RXFIFOF;        /** @brief Receive FIFO full this bit is cleared when one FIFO location becomes empty. */
extern const field32_t SDMMCx_STAR_TXFIFOE;        /** @brief Transmit FIFO empty this bit is cleared when one FIFO location becomes full. */
extern const field32_t SDMMCx_STAR_RXFIFOE;        /** @brief Receive FIFO empty this is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes full. */
extern const field32_t SDMMCx_STAR_BUSYD0;         /** @brief Inverted value of SDMMC_D0 line (busy), sampled at the end of a CMD response and a second time 2 SDMMC_CK cycles after the CMD response. This bit is reset to not busy when the SDMMCD0 line changes from busy to not busy. This bit does not signal busy due to data transfer. This is a hardware status flag only, it does not generate an interrupt. */
extern const field32_t SDMMCx_STAR_BUSYD0END;      /** @brief End of SDMMC_D0 busy following a CMD response detected. This indicates only end of busy following a CMD response. This bit does not signal busy due to data transfer. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_SDIOIT;         /** @brief SDIO interrupt received. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_ACKFAIL;        /** @brief Boot acknowledgment received (boot acknowledgment check fail). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_ACKTIMEOUT;     /** @brief Boot acknowledgment timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_VSWEND;         /** @brief Voltage switch critical timing section completion. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_CKSTOP;         /** @brief SDMMC_CK stopped in voltage switch procedure. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_IDMATE;         /** @brief IDMA transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_STAR_IDMABTC;        /** @brief IDMA buffer transfer complete. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. */
extern const field32_t SDMMCx_ICR_CCRCFAILC;       /** @brief CCRCFAIL flag clear bit set by software to clear the CCRCFAIL flag. */
extern const field32_t SDMMCx_ICR_DCRCFAILC;       /** @brief DCRCFAIL flag clear bit set by software to clear the DCRCFAIL flag. */
extern const field32_t SDMMCx_ICR_CTIMEOUTC;       /** @brief CTIMEOUT flag clear bit set by software to clear the CTIMEOUT flag. */
extern const field32_t SDMMCx_ICR_DTIMEOUTC;       /** @brief DTIMEOUT flag clear bit set by software to clear the DTIMEOUT flag. */
extern const field32_t SDMMCx_ICR_TXUNDERRC;       /** @brief TXUNDERR flag clear bit set by software to clear TXUNDERR flag. */
extern const field32_t SDMMCx_ICR_RXOVERRC;        /** @brief RXOVERR flag clear bit set by software to clear the RXOVERR flag. */
extern const field32_t SDMMCx_ICR_CMDRENDC;        /** @brief CMDREND flag clear bit set by software to clear the CMDREND flag. */
extern const field32_t SDMMCx_ICR_CMDSENTC;        /** @brief CMDSENT flag clear bit set by software to clear the CMDSENT flag. */
extern const field32_t SDMMCx_ICR_DATAENDC;        /** @brief DATAEND flag clear bit set by software to clear the DATAEND flag. */
extern const field32_t SDMMCx_ICR_DHOLDC;          /** @brief DHOLD flag clear bit set by software to clear the DHOLD flag. */
extern const field32_t SDMMCx_ICR_DBCKENDC;        /** @brief DBCKEND flag clear bit set by software to clear the DBCKEND flag. */
extern const field32_t SDMMCx_ICR_DABORTC;         /** @brief DABORT flag clear bit set by software to clear the DABORT flag. */
extern const field32_t SDMMCx_ICR_BUSYD0ENDC;      /** @brief BUSYD0END flag clear bit set by software to clear the BUSYD0END flag. */
extern const field32_t SDMMCx_ICR_SDIOITC;         /** @brief SDIOIT flag clear bit set by software to clear the SDIOIT flag. */
extern const field32_t SDMMCx_ICR_ACKFAILC;        /** @brief ACKFAIL flag clear bit set by software to clear the ACKFAIL flag. */
extern const field32_t SDMMCx_ICR_ACKTIMEOUTC;     /** @brief ACKTIMEOUT flag clear bit set by software to clear the ACKTIMEOUT flag. */
extern const field32_t SDMMCx_ICR_VSWENDC;         /** @brief VSWEND flag clear bit set by software to clear the VSWEND flag. */
extern const field32_t SDMMCx_ICR_CKSTOPC;         /** @brief CKSTOP flag clear bit set by software to clear the CKSTOP flag. */
extern const field32_t SDMMCx_ICR_IDMATEC;         /** @brief IDMA transfer error clear bit set by software to clear the IDMATE flag. */
extern const field32_t SDMMCx_ICR_IDMABTCC;        /** @brief IDMA buffer transfer complete clear bit set by software to clear the IDMABTC flag. */
extern const field32_t SDMMCx_MASKR_CCRCFAILIE;    /** @brief Command CRC fail interrupt enable set and cleared by software to enable/disable interrupt caused by command CRC failure. */
extern const field32_t SDMMCx_MASKR_DCRCFAILIE;    /** @brief Data CRC fail interrupt enable set and cleared by software to enable/disable interrupt caused by data CRC failure. */
extern const field32_t SDMMCx_MASKR_CTIMEOUTIE;    /** @brief Command timeout interrupt enable set and cleared by software to enable/disable interrupt caused by command timeout. */
extern const field32_t SDMMCx_MASKR_DTIMEOUTIE;    /** @brief Data timeout interrupt enable set and cleared by software to enable/disable interrupt caused by data timeout. */
extern const field32_t SDMMCx_MASKR_TXUNDERRIE;    /** @brief Tx FIFO underrun error interrupt enable set and cleared by software to enable/disable interrupt caused by tx FIFO underrun error. */
extern const field32_t SDMMCx_MASKR_RXOVERRIE;     /** @brief Rx FIFO overrun error interrupt enable set and cleared by software to enable/disable interrupt caused by rx FIFO overrun error. */
extern const field32_t SDMMCx_MASKR_CMDRENDIE;     /** @brief Command response received interrupt enable set and cleared by software to enable/disable interrupt caused by receiving command response. */
extern const field32_t SDMMCx_MASKR_CMDSENTIE;     /** @brief Command sent interrupt enable set and cleared by software to enable/disable interrupt caused by sending command. */
extern const field32_t SDMMCx_MASKR_DATAENDIE;     /** @brief Data end interrupt enable set and cleared by software to enable/disable interrupt caused by data end. */
extern const field32_t SDMMCx_MASKR_DHOLDIE;       /** @brief Data hold interrupt enable set and cleared by software to enable/disable the interrupt generated when sending new data is hold in the DPSM wait_s state. */
extern const field32_t SDMMCx_MASKR_DBCKENDIE;     /** @brief Data block end interrupt enable set and cleared by software to enable/disable interrupt caused by data block end. */
extern const field32_t SDMMCx_MASKR_DABORTIE;      /** @brief Data transfer aborted interrupt enable set and cleared by software to enable/disable interrupt caused by a data transfer being aborted. */
extern const field32_t SDMMCx_MASKR_TXFIFOHEIE;    /** @brief Tx FIFO half empty interrupt enable set and cleared by software to enable/disable interrupt caused by tx FIFO half empty. */
extern const field32_t SDMMCx_MASKR_RXFIFOHFIE;    /** @brief Rx FIFO half full interrupt enable set and cleared by software to enable/disable interrupt caused by rx FIFO half full. */
extern const field32_t SDMMCx_MASKR_RXFIFOFIE;     /** @brief Rx FIFO full interrupt enable set and cleared by software to enable/disable interrupt caused by rx FIFO full. */
extern const field32_t SDMMCx_MASKR_TXFIFOEIE;     /** @brief Tx FIFO empty interrupt enable set and cleared by software to enable/disable interrupt caused by tx FIFO empty. */
extern const field32_t SDMMCx_MASKR_BUSYD0ENDIE;   /** @brief BUSYD0END interrupt enable set and cleared by software to enable/disable the interrupt generated when SDMMC_D0 signal changes from busy to NOT busy following a CMD response. */
extern const field32_t SDMMCx_MASKR_SDIOITIE;      /** @brief SDIO mode interrupt received interrupt enable set and cleared by software to enable/disable the interrupt generated when receiving the SDIO mode interrupt. */
extern const field32_t SDMMCx_MASKR_ACKFAILIE;     /** @brief Acknowledgment fail interrupt enable set and cleared by software to enable/disable interrupt caused by acknowledgment fail. */
extern const field32_t SDMMCx_MASKR_ACKTIMEOUTIE;  /** @brief Acknowledgment timeout interrupt enable set and cleared by software to enable/disable interrupt caused by acknowledgment timeout. */
extern const field32_t SDMMCx_MASKR_VSWENDIE;      /** @brief Voltage switch critical timing section completion interrupt enable set and cleared by software to enable/disable the interrupt generated when voltage switch critical timing section completion. */
extern const field32_t SDMMCx_MASKR_CKSTOPIE;      /** @brief Voltage switch clock stopped interrupt enable set and cleared by software to enable/disable interrupt caused by voltage switch clock stopped. */
extern const field32_t SDMMCx_MASKR_IDMABTCIE;     /** @brief IDMA buffer transfer complete interrupt enable set and cleared by software to enable/disable the interrupt generated when the IDMA has transferred all data belonging to a memory buffer. */
extern const field32_t SDMMCx_ACKTIMER_ACKTIME;    /** @brief Boot acknowledgment timeout period this bit can only be written by firmware when CPSM is disabled (CPSMEN = 0). Boot acknowledgment timeout period expressed in card bus clock periods. */
extern const field32_t SDMMCx_IDMACTRLR_IDMAEN;    /** @brief IDMA enable this bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_IDMACTRLR_IDMABMODE; /** @brief Buffer mode selection. This bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_IDMACTRLR_IDMABACT;  /** @brief Double buffer mode active buffer indication this bit can only be written by firmware when DPSM is inactive (DPSMACT = 0). When IDMA is enabled this bit is toggled by hardware. */
extern const field32_t SDMMCx_IDMABSIZER_IDMABNDT; /** @brief Number of transfers per buffer. This 8-bit value shall be multiplied by 8 to get the size of the buffer in 32-bit words and by 32 to get the size of the buffer in bytes. Example: IDMABNDT = 0x01: buffer size = 8 words = 32 bytes. These bits can only be written by firmware when DPSM is inactive (DPSMACT = 0). */
extern const field32_t SDMMCx_RESPCMDR_RESPCMD;    /** @brief Response command index. */

/**************************************************************************************************
 * @section VREFBUF Definitions
 **************************************************************************************************/

/** @subsection VREFBUF Register Definitions */

extern rw_reg32_t const VREFBUF_CSR; /** @brief VREFBUF control and status register. */
extern rw_reg32_t const VREFBUF_CCR; /** @brief VREFBUF calibration control register. */

/** @subsection VREFBUF Register Field Definitions */

extern const field32_t VREFBUF_CSR_ENVR; /** @brief Voltage reference buffer mode enable this bit is used to enable the voltage reference buffer mode. */
extern const field32_t VREFBUF_CSR_HIZ;  /** @brief High impedance mode this bit controls the analog switch to connect or not the VREF+ pin. Refer to table196: VREF buffer modes for the mode descriptions depending on ENVR bit configuration. */
extern const field32_t VREFBUF_CSR_VRR;  /** @brief Voltage reference buffer ready. */
extern const field32_t VREFBUF_CSR_VRS;  /** @brief Voltage reference scale these bits select the value generated by the voltage reference buffer. Other: reserved. */
extern const field32_t VREFBUF_CCR_TRIM; /** @brief Trimming code these bits are automatically initialized after reset with the trimming value stored in the flash memory during the production test. Writing into these bits allows to tune the internal reference buffer voltage. */

/**************************************************************************************************
 * @section IWDGx Definitions
 **************************************************************************************************/

/** @subsection Enumerated IWDGx Register Definitions */

extern rw_reg32_t const IWDGx_KR[3];   /** @brief Key register. */
extern rw_reg32_t const IWDGx_PR[3];   /** @brief Prescaler register. */
extern rw_reg32_t const IWDGx_RLR[3];  /** @brief Reload register. */
extern ro_reg32_t const IWDGx_SR[3];   /** @brief Status register. */
extern rw_reg32_t const IWDGx_WINR[3]; /** @brief Window register. */

/** @subsection IWDGx Register Field Definitions */

extern const field32_t IWDGx_KR_KEY;   /** @brief Key value (write only, read 0x0000) these bits must be written by software at regular intervals with the key value 0xaaaa, otherwise the watchdog generates a reset when the counter reaches 0. Writing the key value 0x5555 to enable access to the IWDG_PR, IWDG_RLR and IWDG_WINR registers (see section23.3.6: register access protection) writing the key value cccch starts the watchdog (except if the hardware watchdog option is selected). */
extern const field32_t IWDGx_PR_PR;    /** @brief Prescaler divider these bits are write access protected see section23.3.6: register access protection. They are written by software to select the prescaler divider feeding the counter clock. PVU bit of IWDG_SR must be reset in order to be able to change the prescaler divider. Note: reading this register returns the prescaler value from the VDD voltage domain. This value may not be up to date/valid if a write operation to this register is ongoing. For this reason the value read from this register is valid only when the PVU bit in the IWDG_SR register is reset. */
extern const field32_t IWDGx_RLR_RL;   /** @brief Watchdog counter reload value these bits are write access protected see section23.3.6. They are written by software to define the value to be loaded in the watchdog counter each time the value 0xaaaa is written in the IWDG_KR register. The watchdog counter counts down from this value. The timeout period is a function of this value and the clock prescaler. Refer to the datasheet for the timeout information. The RVU bit in the IWDG_SR register must be reset in order to be able to change the reload value. Note: reading this register returns the reload value from the VDD voltage domain. This value may not be up to date/valid if a write operation to this register is ongoing on this register. For this reason the value read from this register is valid only when the RVU bit in the IWDG_SR register is reset. */
extern const field32_t IWDGx_SR_PVU;   /** @brief Watchdog prescaler value update this bit is set by hardware to indicate that an update of the prescaler value is ongoing. It is reset by hardware when the prescaler update operation is completed in the VDD voltage domain (takes up to 5 RC 40 khz cycles). Prescaler value can be updated only when PVU bit is reset. */
extern const field32_t IWDGx_SR_RVU;   /** @brief Watchdog counter reload value update this bit is set by hardware to indicate that an update of the reload value is ongoing. It is reset by hardware when the reload value update operation is completed in the VDD voltage domain (takes up to 5 RC 40 khz cycles). Reload value can be updated only when RVU bit is reset. */
extern const field32_t IWDGx_SR_WVU;   /** @brief Watchdog counter window value update this bit is set by hardware to indicate that an update of the window value is ongoing. It is reset by hardware when the reload value update operation is completed in the VDD voltage domain (takes up to 5 RC 40 khz cycles). Window value can be updated only when WVU bit is reset. This bit is generated only if generic window = 1. */
extern const field32_t IWDGx_WINR_WIN; /** @brief Watchdog counter window value these bits are write access protected see section23.3.6. These bits contain the high limit of the window value to be compared to the downcounter. To prevent a reset, the downcounter must be reloaded when its value is lower than the window register value and greater than 0x0 the WVU bit in the IWDG_SR register must be reset in order to be able to change the reload value. Note: reading this register returns the reload value from the VDD voltage domain. This value may not be valid if a write operation to this register is ongoing. For this reason the value read from this register is valid only when the WVU bit in the IWDG_SR register is reset. */

/**************************************************************************************************
 * @section WWDGx Definitions
 **************************************************************************************************/

/** @subsection Enumerated WWDGx Register Definitions */

extern rw_reg32_t const WWDGx_CR[3];  /** @brief Control register. */
extern rw_reg32_t const WWDGx_CFR[3]; /** @brief Configuration register. */
extern rw_reg32_t const WWDGx_SR[3];  /** @brief Status register. */

/** @subsection WWDGx Register Field Definitions */

extern const field32_t WWDGx_CR_T;      /** @brief 7-bit counter (MSB to LSB) these bits contain the value of the watchdog counter. It is decremented every (4096 x 2WDGTB[1:0]) PCLK cycles. A reset is produced when it is decremented from 0x40 to 0x3f (T6 becomes cleared). */
extern const field32_t WWDGx_CR_WDGA;   /** @brief Activation bit this bit is set by software and only cleared by hardware after a reset. When WDGA=1, the watchdog can generate a reset. */
extern const field32_t WWDGx_CFR_W;     /** @brief 7-bit window value these bits contain the window value to be compared to the downcounter. */
extern const field32_t WWDGx_CFR_WDGTB; /** @brief Timer base the time base of the prescaler can be modified as follows:. */
extern const field32_t WWDGx_CFR_EWI;   /** @brief Early wakeup interrupt when set, an interrupt occurs whenever the counter reaches the value 0x40. This interrupt is only cleared by hardware after a reset. */
extern const field32_t WWDGx_SR_EWIF;   /** @brief Early wakeup interrupt flag this bit is set by hardware when the counter has reached the value 0x40. It must be cleared by software by writing 0. A write of 1 has no effect. This bit is also set if the interrupt is not enabled. */

/**************************************************************************************************
 * @section PWR Definitions
 **************************************************************************************************/

/** @subsection PWR Register Definitions */

extern rw_reg32_t const PWR_CR1;     /** @brief PWR control register 1. */
extern ro_reg32_t const PWR_CSR1;    /** @brief PWR control status register 1. */
extern rw_reg32_t const PWR_CR2;     /** @brief This register is not reset by wakeup from standby mode, RESET signal and VDD POR. It is only reset by VSW POR and VSWRST reset. This register shall not be accessed when VSWRST bit in RCC_BDCR register resets the VSW domain.after reset, PWR_CR2 register is write-protected. Prior to modifying its content, the DBP bit in PWR_CR1 register must be set to disable the write protection. */
extern rw_reg32_t const PWR_CR3;     /** @brief Reset only by POR only, not reset by wakeup from standby mode and RESET pad. The lower byte of this register is written once after POR and shall be written before changing VOS level or ck_sys clock frequency. No limitation applies to the upper bytes.programming data corresponding to an invalid combination of SDLEVEL, SDEXTHP, SDEN, LDOEN and BYPASS bits (see table9) will be ignored: data will not be written, the written-once mechanism will lock the register and any further write access will be ignored. The default supply configuration will be kept and the ACTVOSRDY bit in PWR control status register 1 (PWR_CSR1) will go on indicating invalid voltage levels. The system shall be power cycled before writing a new value. */
extern rw_reg32_t const PWR_CPUCR;   /** @brief This register allows controlling CPU1 power. */
extern rw_reg32_t const PWR_D3CR;    /** @brief This register allows controlling D3 domain power.following reset VOSRDY will be read 1 by software. */
extern rw_reg32_t const PWR_WKUPCR;  /** @brief Reset only by system reset, not reset by wakeup from standby mode5 wait states are required when writing this register (when clearing a WKUPF bit in PWR_WKUPFR, the AHB write access will complete after the WKUPF has been cleared). */
extern rw_reg32_t const PWR_WKUPFR;  /** @brief Reset only by system reset, not reset by wakeup from standby mode. */
extern rw_reg32_t const PWR_WKUPEPR; /** @brief Reset only by system reset, not reset by wakeup from standby mode. */

/** @subsection PWR Register Field Definitions */

extern const field32_t PWR_CR1_LPDS;       /** @brief Low-power deepsleep with SVOS3 (SVOS4 and SVOS5 always use low-power, regardless of the setting of this bit). */
extern const field32_t PWR_CR1_PVDE;       /** @brief Programmable voltage detector enable. */
extern const field32_t PWR_CR1_PLS;        /** @brief Programmable voltage detector level selection these bits select the voltage threshold detected by the PVD. Note: refer to section electrical characteristics of the product datasheet for more details. */
extern const field32_t PWR_CR1_DBP;        /** @brief Disable backup domain write protection in reset state, the RCC_BDCR register, the RTC registers (including the backup registers), BREN and MOEN bits in PWR_CR2 register, are protected against parasitic write access. This bit must be set to enable write access to these registers. */
extern const field32_t PWR_CR1_FLPS;       /** @brief Flash low-power mode in dstop mode this bit allows to obtain the best trade-off between low-power consumption and restart time when exiting from dstop mode. When it is set, the flash memory enters low-power mode when D1 domain is in dstop mode. */
extern const field32_t PWR_CR1_SVOS;       /** @brief System stop mode voltage scaling selection these bits control the VCORE voltage level in system stop mode, to obtain the best trade-off between power consumption and performance. */
extern const field32_t PWR_CR1_AVDEN;      /** @brief Peripheral voltage monitor on VDDA enable. */
extern const field32_t PWR_CR1_ALS;        /** @brief Analog voltage detector level selection these bits select the voltage threshold detected by the AVD. */
extern const field32_t PWR_CSR1_PVDO;      /** @brief Programmable voltage detect output this bit is set and cleared by hardware. It is valid only if the PVD has been enabled by the PVDE bit. Note: since the PVD is disabled in standby mode, this bit is equal to 0 after standby or reset until the PVDE bit is set. */
extern const field32_t PWR_CSR1_ACTVOSRDY; /** @brief Voltage levels ready bit for currently used VOS and SDLEVEL this bit is set to 1 by hardware when the voltage regulator and the SD converter are both disabled and bypass mode is selected in PWR control register 3 (PWR_CR3). */
extern const field32_t PWR_CSR1_ACTVOS;    /** @brief VOS currently applied for VCORE voltage scaling selection. These bits reflect the last VOS value applied to the PMU. */
extern const field32_t PWR_CSR1_AVDO;      /** @brief Analog voltage detector output on VDDA this bit is set and cleared by hardware. It is valid only if AVD on VDDA is enabled by the AVDEN bit. Note: since the AVD is disabled in standby mode, this bit is equal to 0 after standby or reset until the AVDEN bit is set. */
extern const field32_t PWR_CR2_BREN;       /** @brief Backup regulator enable when set, the backup regulator (used to maintain the backup RAM content in standby and VBAT modes) is enabled. If BREN is reset, the backup regulator is switched off. The backup RAM can still be used in run and stop modes. However, its content will be lost in standby and VBAT modes. If BREN is set, the application must wait till the backup regulator ready flag (BRRDY) is set to indicate that the data written into the SRAM will be maintained in standby and VBAT modes. */
extern const field32_t PWR_CR2_MONEN;      /** @brief VBAT and temperature monitoring enable when set, the VBAT supply and temperature monitoring is enabled. */
extern const field32_t PWR_CR2_BRRDY;      /** @brief Backup regulator ready this bit is set by hardware to indicate that the backup regulator is ready. */
extern const field32_t PWR_CR2_VBATL;      /** @brief VBAT level monitoring versus low threshold. */
extern const field32_t PWR_CR2_VBATH;      /** @brief VBAT level monitoring versus high threshold. */
extern const field32_t PWR_CR2_TEMPL;      /** @brief Temperature level monitoring versus low threshold. */
extern const field32_t PWR_CR2_TEMPH;      /** @brief Temperature level monitoring versus high threshold. */
extern const field32_t PWR_CR3_BYPASS;     /** @brief Power management unit bypass. */
extern const field32_t PWR_CR3_LDOEN;      /** @brief Low drop-out regulator enable. */
extern const field32_t PWR_CR3_SDEN;       /** @brief SD converter enable. */
extern const field32_t PWR_CR3_VBE;        /** @brief VBAT charging enable. */
extern const field32_t PWR_CR3_VBRS;       /** @brief VBAT charging resistor selection. */
extern const field32_t PWR_CR3_USB33DEN;   /** @brief VDD33USB voltage level detector enable. */
extern const field32_t PWR_CR3_USBREGEN;   /** @brief USB regulator enable. */
extern const field32_t PWR_CR3_USB33RDY;   /** @brief USB supply ready. */
extern const field32_t PWR_CPUCR_STOPF;    /** @brief STOP flag this bit is set by hardware and cleared only by any reset or by setting the CPU1 CSSF bit. */
extern const field32_t PWR_CPUCR_SBF;      /** @brief System standby flag this bit is set by hardware and cleared only by a POR (power-on reset) or by setting the CPU1 CSSF bit. */
extern const field32_t PWR_CPUCR_CSSF;     /** @brief Clear D1 domain CPU1 standby, stop and HOLD flags (always read as 0) this bit is cleared to 0 by hardware. */
extern const field32_t PWR_CPUCR_RUN_D3;   /** @brief Keep system D3 domain in run mode regardless of the CPU sub-systems modes. */
extern const field32_t PWR_D3CR_VOSRDY;    /** @brief VOS ready bit for VCORE voltage scaling output selection. This bit is set to 1 by hardware when bypass mode is selected in PWR control register 3 (PWR_CR3). */
extern const field32_t PWR_D3CR_VOS;       /** @brief Voltage scaling selection according to performance these bits control the VCORE voltage level and allow to obtains the best trade-off between power consumption and performance: when increasing the performance, the voltage scaling shall be changed before increasing the system frequency. When decreasing performance, the system frequency shall first be decreased before changing the voltage scaling. */
extern const field32_t PWR_WKUPCR_WKUPC;   /** @brief Clear wakeup pin flag for WKUP. These bits are always read as 0. */

/** @subsection Enumerated PWR Register Field Definitions */

extern const field32_t PWR_CPUCR_PDDS_Dx[4];     /** @brief D1 domain power down deepsleep selection. This bit allows CPU1 to define the deepsleep mode for D1 domain. */
extern const field32_t PWR_CPUCR_SBF_Dx[3];      /** @brief D1 domain dstandby flag this bit is set by hardware and cleared by any system reset or by setting the CPU1 CSSF bit. Once set, this bit can be cleared only when the D1 domain is no longer in dstandby mode. */
extern const field32_t PWR_WKUPFR_WKUPFx[7];     /** @brief Wakeup pin WKUPF flag. This bit is set by hardware and cleared only by a reset pin or by setting the wkupcn+1 bit in the PWR wakeup clear register (PWR_WKUPCR). */
extern const field32_t PWR_WKUPEPR_WKUPENx[7];   /** @brief Enable wakeup pin wkupn+1 each bit is set and cleared by software. Note: an additional wakeup event is detected if wkupn+1 pin is enabled (by setting the wkupenn+1 bit) when wkupn+1 pin level is already high when wkuppn+1 selects rising edge, or low when wkuppn+1 selects falling edge. */
extern const field32_t PWR_WKUPEPR_WKUPPx[7];    /** @brief Wakeup pin polarity bit for wkupn-7 these bits define the polarity used for event detection on wkupn-7 external wakeup pin. */
extern const field32_t PWR_WKUPEPR_WKUPPUPDx[7]; /** @brief Wakeup pin pull configuration. */

/**************************************************************************************************
 * @section SPIx Definitions
 **************************************************************************************************/

/** @subsection Enumerated SPIx Register Definitions */

extern rw_reg32_t const SPIx_CR1[7];     /** @brief Control register 1. */
extern rw_reg32_t const SPIx_CR2[7];     /** @brief Control register 2. */
extern rw_reg32_t const SPIx_CFG1[7];    /** @brief Configuration register 1. */
extern rw_reg32_t const SPIx_CFG2[7];    /** @brief Configuration register 2. */
extern rw_reg32_t const SPIx_IER[7];     /** @brief Interrupt enable register. */
extern ro_reg32_t const SPIx_SR[7];      /** @brief Status register. */
extern rw_reg32_t const SPIx_IFCR[7];    /** @brief Interrupt/Status flags clear register. */
extern rw_reg32_t const SPIx_TXDR[7];    /** @brief Transmit data register. */
extern ro_reg32_t const SPIx_RXDR[7];    /** @brief Receive data register. */
extern rw_reg32_t const SPIx_CRCPOLY[7]; /** @brief Polynomial register. */
extern rw_reg32_t const SPIx_TXCRC[7];   /** @brief Transmitter CRC register. */
extern rw_reg32_t const SPIx_RXCRC[7];   /** @brief Receiver CRC register. */
extern rw_reg32_t const SPIx_UDRDR[7];   /** @brief Underrun data register. */
extern rw_reg32_t const SPIx_CGFR[7];    /** @brief Configuration register. */

/** @subsection SPIx Register Field Definitions */

extern const field32_t SPIx_CR1_IOLOCK;   /** @brief Locking the AF configuration of associated ios. */
extern const field32_t SPIx_CR1_TCRCI;    /** @brief CRC calculation initialization pattern control for transmitter. */
extern const field32_t SPIx_CR1_RCRCI;    /** @brief CRC calculation initialization pattern control for receiver. */
extern const field32_t SPIx_CR1_CRC33_17; /** @brief 32-bit CRC polynomial configuration. */
extern const field32_t SPIx_CR1_SSI;      /** @brief Internal SS signal input level. */
extern const field32_t SPIx_CR1_HDDIR;    /** @brief Rx/Tx direction at half-duplex mode. */
extern const field32_t SPIx_CR1_CSUSP;    /** @brief Master suspend request. */
extern const field32_t SPIx_CR1_CSTART;   /** @brief Master transfer start. */
extern const field32_t SPIx_CR1_MASRX;    /** @brief Master automatic SUSP in receive mode. */
extern const field32_t SPIx_CR1_SPE;      /** @brief Serial peripheral enable. */
extern const field32_t SPIx_CR2_TSER;     /** @brief Number of data transfer extension to be reload into TSIZE just when a previous. */
extern const field32_t SPIx_CR2_TSIZE;    /** @brief Number of data at current transfer. */
extern const field32_t SPIx_CFG1_MBR;     /** @brief Master baud rate. */
extern const field32_t SPIx_CFG1_CRCEN;   /** @brief Hardware CRC computation enable. */
extern const field32_t SPIx_CFG1_CRCSIZE; /** @brief Length of CRC frame to be transacted and compared. */
extern const field32_t SPIx_CFG1_TXDMAEN; /** @brief Tx DMA stream enable. */
extern const field32_t SPIx_CFG1_RXDMAEN; /** @brief Rx DMA stream enable. */
extern const field32_t SPIx_CFG1_UDRDET;  /** @brief Detection of underrun condition at slave transmitter. */
extern const field32_t SPIx_CFG1_UDRCFG;  /** @brief Behavior of slave transmitter at underrun condition. */
extern const field32_t SPIx_CFG1_FTHVL;   /** @brief Threshold level. */
extern const field32_t SPIx_CFG1_DSIZE;   /** @brief Number of bits in at single SPI data frame. */
extern const field32_t SPIx_CFG2_AFCNTR;  /** @brief Alternate function gpios control. */
extern const field32_t SPIx_CFG2_SSOM;    /** @brief SS output management in master mode. */
extern const field32_t SPIx_CFG2_SSOE;    /** @brief SS output enable. */
extern const field32_t SPIx_CFG2_SSIOP;   /** @brief SS input/output polarity. */
extern const field32_t SPIx_CFG2_SSM;     /** @brief Software management of SS signal input. */
extern const field32_t SPIx_CFG2_CPOL;    /** @brief Clock polarity. */
extern const field32_t SPIx_CFG2_CPHA;    /** @brief Clock phase. */
extern const field32_t SPIx_CFG2_LSBFRST; /** @brief Data frame format. */
extern const field32_t SPIx_CFG2_MASTER;  /** @brief SPI master. */
extern const field32_t SPIx_CFG2_SP;      /** @brief Serial protocol. */
extern const field32_t SPIx_CFG2_COMM;    /** @brief SPI communication mode. */
extern const field32_t SPIx_CFG2_IOSWP;   /** @brief Swap functionality of MISO and MOSI pins. */
extern const field32_t SPIx_CFG2_MIDI;    /** @brief Master inter-data idleness. */
extern const field32_t SPIx_CFG2_MSSI;    /** @brief Master SS idleness. */
extern const field32_t SPIx_IER_TSERFIE;  /** @brief Additional number of transactions reload interrupt enable. */
extern const field32_t SPIx_IER_MODFIE;   /** @brief Mode fault interrupt enable. */
extern const field32_t SPIx_IER_TIFREIE;  /** @brief TIFRE interrupt enable. */
extern const field32_t SPIx_IER_CRCEIE;   /** @brief CRC interrupt enable. */
extern const field32_t SPIx_IER_OVRIE;    /** @brief OVR interrupt enable. */
extern const field32_t SPIx_IER_UDRIE;    /** @brief UDR interrupt enable. */
extern const field32_t SPIx_IER_TXTFIE;   /** @brief TXTFIE interrupt enable. */
extern const field32_t SPIx_IER_EOTIE;    /** @brief EOT, SUSP and TXC interrupt enable. */
extern const field32_t SPIx_IER_DPXPIE;   /** @brief DXP interrupt enabled. */
extern const field32_t SPIx_IER_TXPIE;    /** @brief TXP interrupt enable. */
extern const field32_t SPIx_IER_RXPIE;    /** @brief RXP interrupt enable. */
extern const field32_t SPIx_SR_CTSIZE;    /** @brief Number of data frames remaining in current TSIZE session. */
extern const field32_t SPIx_SR_RXWNE;     /** @brief RxFIFO word not empty. */
extern const field32_t SPIx_SR_RXPLVL;    /** @brief RxFIFO packing level. */
extern const field32_t SPIx_SR_TXC;       /** @brief TxFIFO transmission complete. */
extern const field32_t SPIx_SR_SUSP;      /** @brief SUSPend. */
extern const field32_t SPIx_SR_TSERF;     /** @brief Additional number of SPI data to be transacted was reload. */
extern const field32_t SPIx_SR_MODF;      /** @brief Mode fault. */
extern const field32_t SPIx_SR_TIFRE;     /** @brief TI frame format error. */
extern const field32_t SPIx_SR_CRCE;      /** @brief CRC error. */
extern const field32_t SPIx_SR_OVR;       /** @brief Overrun. */
extern const field32_t SPIx_SR_UDR;       /** @brief Underrun at slave transmission mode. */
extern const field32_t SPIx_SR_TXTF;      /** @brief Transmission transfer filled. */
extern const field32_t SPIx_SR_EOT;       /** @brief End of transfer. */
extern const field32_t SPIx_SR_DXP;       /** @brief Duplex packet. */
extern const field32_t SPIx_SR_TXP;       /** @brief Tx-Packet space available. */
extern const field32_t SPIx_SR_RXP;       /** @brief Rx-Packet available. */
extern const field32_t SPIx_IFCR_SUSPC;   /** @brief SUSPend flag clear. */
extern const field32_t SPIx_IFCR_TSERFC;  /** @brief TSERFC flag clear. */
extern const field32_t SPIx_IFCR_MODFC;   /** @brief Mode fault flag clear. */
extern const field32_t SPIx_IFCR_TIFREC;  /** @brief TI frame format error flag clear. */
extern const field32_t SPIx_IFCR_CRCEC;   /** @brief CRC error flag clear. */
extern const field32_t SPIx_IFCR_OVRC;    /** @brief Overrun flag clear. */
extern const field32_t SPIx_IFCR_UDRC;    /** @brief Underrun flag clear. */
extern const field32_t SPIx_IFCR_TXTFC;   /** @brief Transmission transfer filled flag clear. */
extern const field32_t SPIx_IFCR_EOTC;    /** @brief End of transfer flag clear. */
extern const field32_t SPIx_CGFR_MCKOE;   /** @brief Master clock output enable. */
extern const field32_t SPIx_CGFR_ODD;     /** @brief Odd factor for the prescaler. */
extern const field32_t SPIx_CGFR_I2SDIV;  /** @brief I2S linear prescaler. */
extern const field32_t SPIx_CGFR_DATFMT;  /** @brief Data format. */
extern const field32_t SPIx_CGFR_WSINV;   /** @brief Fixed channel length in SLAVE. */
extern const field32_t SPIx_CGFR_FIXCH;   /** @brief Word select inversion. */
extern const field32_t SPIx_CGFR_CKPOL;   /** @brief Serial audio clock polarity. */
extern const field32_t SPIx_CGFR_CHLEN;   /** @brief Channel length (number of bits per audio channel). */
extern const field32_t SPIx_CGFR_DATLEN;  /** @brief Data length to be transferred. */
extern const field32_t SPIx_CGFR_PCMSYNC; /** @brief PCM frame synchronization. */
extern const field32_t SPIx_CGFR_I2SSTD;  /** @brief I2S standard selection. */
extern const field32_t SPIx_CGFR_I2SCFG;  /** @brief I2S configuration mode. */
extern const field32_t SPIx_CGFR_I2SMOD;  /** @brief I2S mode selection. */

/**************************************************************************************************
 * @section LTDC Definitions
 **************************************************************************************************/

/** @subsection LTDC Register Definitions */

extern rw_reg32_t const LTDC_SSCR;  /** @brief Synchronization size configuration register. */
extern rw_reg32_t const LTDC_BPCR;  /** @brief Back porch configuration register. */
extern rw_reg32_t const LTDC_AWCR;  /** @brief Active width configuration register. */
extern rw_reg32_t const LTDC_TWCR;  /** @brief Total width configuration register. */
extern rw_reg32_t const LTDC_GCR;   /** @brief Global control register. */
extern rw_reg32_t const LTDC_SRCR;  /** @brief Shadow reload configuration register. */
extern rw_reg32_t const LTDC_BCCR;  /** @brief Background color configuration register. */
extern rw_reg32_t const LTDC_IER;   /** @brief Interrupt enable register. */
extern ro_reg32_t const LTDC_ISR;   /** @brief Interrupt status register. */
extern rw_reg32_t const LTDC_ICR;   /** @brief Interrupt clear register. */
extern rw_reg32_t const LTDC_LIPCR; /** @brief Line interrupt position configuration register. */
extern ro_reg32_t const LTDC_CPSR;  /** @brief Current position status register. */
extern ro_reg32_t const LTDC_CDSR;  /** @brief Current display status register. */

/** @subsection Enumerated LTDC Register Definitions */

extern rw_reg32_t const LTDC_LxCR[3];     /** @brief Layerx control register. */
extern rw_reg32_t const LTDC_LxWHPCR[3];  /** @brief Layerx window horizontal position configuration register. */
extern rw_reg32_t const LTDC_LxWVPCR[3];  /** @brief Layerx window vertical position configuration register. */
extern rw_reg32_t const LTDC_LxCKCR[3];   /** @brief Layerx color keying configuration register. */
extern rw_reg32_t const LTDC_LxPFCR[3];   /** @brief Layerx pixel format configuration register. */
extern rw_reg32_t const LTDC_LxCACR[3];   /** @brief Layerx constant alpha configuration register. */
extern rw_reg32_t const LTDC_LxDCCR[3];   /** @brief Layerx default color configuration register. */
extern rw_reg32_t const LTDC_LxBFCR[3];   /** @brief Layerx blending factors configuration register. */
extern rw_reg32_t const LTDC_LxCFBAR[3];  /** @brief Layerx color frame buffer address register. */
extern rw_reg32_t const LTDC_LxCFBLR[3];  /** @brief Layerx color frame buffer length register. */
extern rw_reg32_t const LTDC_LxCFBLNR[3]; /** @brief Layerx colorframe buffer line number register. */
extern rw_reg32_t const LTDC_LxCLUTWR[3]; /** @brief Layerx CLUT write register. */

/** @subsection LTDC Register Field Definitions */

extern const field32_t LTDC_SSCR_HSW;         /** @brief Horizontal synchronization width (in units of pixel clock period). */
extern const field32_t LTDC_SSCR_VSH;         /** @brief Vertical synchronization height (in units of horizontal scan line). */
extern const field32_t LTDC_BPCR_AHBP;        /** @brief Accumulated horizontal back porch (in units of pixel clock period). */
extern const field32_t LTDC_BPCR_AVBP;        /** @brief Accumulated vertical back porch (in units of horizontal scan line). */
extern const field32_t LTDC_AWCR_AAV;         /** @brief AAV. */
extern const field32_t LTDC_AWCR_AAH;         /** @brief Accumulated active height (in units of horizontal scan line). */
extern const field32_t LTDC_TWCR_TOTALW;      /** @brief Total width (in units of pixel clock period). */
extern const field32_t LTDC_TWCR_TOTALH;      /** @brief Total height (in units of horizontal scan line). */
extern const field32_t LTDC_GCR_HSPOL;        /** @brief Horizontal synchronization polarity. */
extern const field32_t LTDC_GCR_VSPOL;        /** @brief Vertical synchronization polarity. */
extern const field32_t LTDC_GCR_DEPOL;        /** @brief Data enable polarity. */
extern const field32_t LTDC_GCR_PCPOL;        /** @brief Pixel clock polarity. */
extern const field32_t LTDC_GCR_DEN;          /** @brief Dither enable. */
extern const field32_t LTDC_GCR_DRW;          /** @brief Dither red width. */
extern const field32_t LTDC_GCR_DGW;          /** @brief Dither green width. */
extern const field32_t LTDC_GCR_DBW;          /** @brief Dither blue width. */
extern const field32_t LTDC_GCR_LTDCEN;       /** @brief LCD-TFT controller enable bit. */
extern const field32_t LTDC_SRCR_VBR;         /** @brief Vertical blanking reload. */
extern const field32_t LTDC_SRCR_IMR;         /** @brief Immediate reload. */
extern const field32_t LTDC_BCCR_BCBLUE;      /** @brief Background color blue value. */
extern const field32_t LTDC_BCCR_BCGREEN;     /** @brief Background color green value. */
extern const field32_t LTDC_BCCR_BCRED;       /** @brief Background color red value. */
extern const field32_t LTDC_IER_RRIE;         /** @brief Register reload interrupt enable. */
extern const field32_t LTDC_IER_TERRIE;       /** @brief Transfer error interrupt enable. */
extern const field32_t LTDC_IER_FUIE;         /** @brief FIFO underrun interrupt enable. */
extern const field32_t LTDC_IER_LIE;          /** @brief Line interrupt enable. */
extern const field32_t LTDC_ISR_RRIF;         /** @brief Register reload interrupt flag. */
extern const field32_t LTDC_ISR_TERRIF;       /** @brief Transfer error interrupt flag. */
extern const field32_t LTDC_ISR_FUIF;         /** @brief FIFO underrun interrupt flag. */
extern const field32_t LTDC_ISR_LIF;          /** @brief Line interrupt flag. */
extern const field32_t LTDC_ICR_CRRIF;        /** @brief Clears register reload interrupt flag. */
extern const field32_t LTDC_ICR_CTERRIF;      /** @brief Clears the transfer error interrupt flag. */
extern const field32_t LTDC_ICR_CFUIF;        /** @brief Clears the FIFO underrun interrupt flag. */
extern const field32_t LTDC_ICR_CLIF;         /** @brief Clears the line interrupt flag. */
extern const field32_t LTDC_LIPCR_LIPOS;      /** @brief Line interrupt position. */
extern const field32_t LTDC_CPSR_CXPOS;       /** @brief Current X position. */
extern const field32_t LTDC_CPSR_CYPOS;       /** @brief Current Y position. */
extern const field32_t LTDC_CDSR_HSYNCS;      /** @brief Horizontal synchronization display status. */
extern const field32_t LTDC_CDSR_VSYNCS;      /** @brief Vertical synchronization display status. */
extern const field32_t LTDC_CDSR_HDES;        /** @brief Horizontal data enable display status. */
extern const field32_t LTDC_CDSR_VDES;        /** @brief Vertical data enable display status. */
extern const field32_t LTDC_LxCR_CLUTEN;      /** @brief Color look-up table enable. */
extern const field32_t LTDC_LxCR_COLKEN;      /** @brief Color keying enable. */
extern const field32_t LTDC_LxCR_LEN;         /** @brief Layer enable. */
extern const field32_t LTDC_LxWHPCR_WHSPPOS;  /** @brief Window horizontal stop position. */
extern const field32_t LTDC_LxWHPCR_WHSTPOS;  /** @brief Window horizontal start position. */
extern const field32_t LTDC_LxWVPCR_WVSPPOS;  /** @brief Window vertical stop position. */
extern const field32_t LTDC_LxWVPCR_WVSTPOS;  /** @brief Window vertical start position. */
extern const field32_t LTDC_LxCKCR_CKRED;     /** @brief Color key red value. */
extern const field32_t LTDC_LxCKCR_CKGREEN;   /** @brief Color key green value. */
extern const field32_t LTDC_LxCKCR_CKBLUE;    /** @brief Color key blue value. */
extern const field32_t LTDC_LxPFCR_PF;        /** @brief Pixel format. */
extern const field32_t LTDC_LxCACR_CONSTA;    /** @brief Constant alpha. */
extern const field32_t LTDC_LxDCCR_DCALPHA;   /** @brief Default color alpha. */
extern const field32_t LTDC_LxDCCR_DCRED;     /** @brief Default color red. */
extern const field32_t LTDC_LxDCCR_DCGREEN;   /** @brief Default color green. */
extern const field32_t LTDC_LxDCCR_DCBLUE;    /** @brief Default color blue. */
extern const field32_t LTDC_LxCFBLR_CFBP;     /** @brief Color frame buffer pitch in bytes. */
extern const field32_t LTDC_LxCFBLR_CFBLL;    /** @brief Color frame buffer line length. */
extern const field32_t LTDC_LxCFBLNR_CFBLNBR; /** @brief Frame buffer line number. */
extern const field32_t LTDC_LxCLUTWR_CLUTADD; /** @brief CLUT address. */
extern const field32_t LTDC_LxCLUTWR_RED;     /** @brief Red value. */
extern const field32_t LTDC_LxCLUTWR_GREEN;   /** @brief Green value. */
extern const field32_t LTDC_LxCLUTWR_BLUE;    /** @brief Blue value. */

/** @subsection Enumerated LTDC Register Field Definitions */

extern const field32_t LTDC_LxBFCR_BFx[3]; /** @brief Blending factor 1. */

/**************************************************************************************************
 * @section SPDIFRX Definitions
 **************************************************************************************************/

/** @subsection SPDIFRX Register Definitions */

extern rw_reg32_t const SPDIFRX_CR;    /** @brief Control register. */
extern rw_reg32_t const SPDIFRX_IMR;   /** @brief Interrupt mask register. */
extern ro_reg32_t const SPDIFRX_SR;    /** @brief Status register. */
extern rw_reg32_t const SPDIFRX_IFCR;  /** @brief Interrupt flag clear register. */
extern ro_reg32_t const SPDIFRX_DR_00; /** @brief Data input register. */
extern ro_reg32_t const SPDIFRX_CSR;   /** @brief Channel status register. */
extern ro_reg32_t const SPDIFRX_DIR;   /** @brief Debug information register. */
extern ro_reg32_t const SPDIFRX_VERR;  /** @brief SPDIFRX version register. */
extern ro_reg32_t const SPDIFRX_IDR;   /** @brief SPDIFRX identification register. */
extern ro_reg32_t const SPDIFRX_SIDR;  /** @brief SPDIFRX size identification register. */
extern ro_reg32_t const SPDIFRX_DR_01; /** @brief Data input register. */
extern ro_reg32_t const SPDIFRX_DR_10; /** @brief Data input register. */

/** @subsection SPDIFRX Register Field Definitions */

extern const field32_t SPDIFRX_CR_SPDIFRXEN; /** @brief Peripheral block enable. */
extern const field32_t SPDIFRX_CR_RXDMAEN;   /** @brief Receiver DMA enable for data flow. */
extern const field32_t SPDIFRX_CR_RXSTEO;    /** @brief STerEO mode. */
extern const field32_t SPDIFRX_CR_DRFMT;     /** @brief RX data format. */
extern const field32_t SPDIFRX_CR_PMSK;      /** @brief Mask parity error bit. */
extern const field32_t SPDIFRX_CR_VMSK;      /** @brief Mask of validity bit. */
extern const field32_t SPDIFRX_CR_CUMSK;     /** @brief Mask of channel status and user bits. */
extern const field32_t SPDIFRX_CR_PTMSK;     /** @brief Mask of preamble type bits. */
extern const field32_t SPDIFRX_CR_CBDMAEN;   /** @brief Control buffer DMA enable for control flow. */
extern const field32_t SPDIFRX_CR_CHSEL;     /** @brief Channel selection. */
extern const field32_t SPDIFRX_CR_NBTR;      /** @brief Maximum allowed re-tries during synchronization phase. */
extern const field32_t SPDIFRX_CR_WFA;       /** @brief Wait for activity. */
extern const field32_t SPDIFRX_CR_INSEL;     /** @brief Input selection. */
extern const field32_t SPDIFRX_CR_CKSEN;     /** @brief Symbol clock enable. */
extern const field32_t SPDIFRX_CR_CKSBKPEN;  /** @brief Backup symbol clock enable. */
extern const field32_t SPDIFRX_IMR_RXNEIE;   /** @brief RXNE interrupt enable. */
extern const field32_t SPDIFRX_IMR_CSRNEIE;  /** @brief Control buffer ready interrupt enable. */
extern const field32_t SPDIFRX_IMR_PERRIE;   /** @brief Parity error interrupt enable. */
extern const field32_t SPDIFRX_IMR_OVRIE;    /** @brief Overrun error interrupt enable. */
extern const field32_t SPDIFRX_IMR_SBLKIE;   /** @brief Synchronization block detected interrupt enable. */
extern const field32_t SPDIFRX_IMR_SYNCDIE;  /** @brief Synchronization done. */
extern const field32_t SPDIFRX_IMR_IFEIE;    /** @brief Serial interface error interrupt enable. */
extern const field32_t SPDIFRX_SR_RXNE;      /** @brief Read data register not empty. */
extern const field32_t SPDIFRX_SR_CSRNE;     /** @brief Control buffer register is not empty. */
extern const field32_t SPDIFRX_SR_PERR;      /** @brief Parity error. */
extern const field32_t SPDIFRX_SR_OVR;       /** @brief Overrun error. */
extern const field32_t SPDIFRX_SR_SBD;       /** @brief Synchronization block detected. */
extern const field32_t SPDIFRX_SR_SYNCD;     /** @brief Synchronization done. */
extern const field32_t SPDIFRX_SR_FERR;      /** @brief Framing error. */
extern const field32_t SPDIFRX_SR_SERR;      /** @brief Synchronization error. */
extern const field32_t SPDIFRX_SR_TERR;      /** @brief Time-out error. */
extern const field32_t SPDIFRX_SR_WIDTH5;    /** @brief Duration of 5 symbols counted with SPDIF_CLK. */
extern const field32_t SPDIFRX_IFCR_PERRCF;  /** @brief Clears the parity error flag. */
extern const field32_t SPDIFRX_IFCR_OVRCF;   /** @brief Clears the overrun error flag. */
extern const field32_t SPDIFRX_IFCR_SBDCF;   /** @brief Clears the synchronization block detected flag. */
extern const field32_t SPDIFRX_IFCR_SYNCDCF; /** @brief Clears the synchronization done flag. */
extern const field32_t SPDIFRX_DR_00_DR;     /** @brief Parity error bit. */
extern const field32_t SPDIFRX_DR_00_PE;     /** @brief Parity error bit. */
extern const field32_t SPDIFRX_DR_00_V;      /** @brief Validity bit. */
extern const field32_t SPDIFRX_DR_00_U;      /** @brief User bit. */
extern const field32_t SPDIFRX_DR_00_C;      /** @brief Channel status bit. */
extern const field32_t SPDIFRX_DR_00_PT;     /** @brief Preamble type. */
extern const field32_t SPDIFRX_CSR_USR;      /** @brief User data information. */
extern const field32_t SPDIFRX_CSR_CS;       /** @brief Channel A status information. */
extern const field32_t SPDIFRX_CSR_SOB;      /** @brief Start of block. */
extern const field32_t SPDIFRX_DIR_THI;      /** @brief Threshold HIGH. */
extern const field32_t SPDIFRX_DIR_TLO;      /** @brief Threshold LOW. */
extern const field32_t SPDIFRX_VERR_MINREV;  /** @brief Minor revision. */
extern const field32_t SPDIFRX_VERR_MAJREV;  /** @brief Major revision. */
extern const field32_t SPDIFRX_DR_01_PE;     /** @brief Parity error bit. */
extern const field32_t SPDIFRX_DR_01_V;      /** @brief Validity bit. */
extern const field32_t SPDIFRX_DR_01_U;      /** @brief User bit. */
extern const field32_t SPDIFRX_DR_01_C;      /** @brief Channel status bit. */
extern const field32_t SPDIFRX_DR_01_PT;     /** @brief Preamble type. */
extern const field32_t SPDIFRX_DR_01_DR;     /** @brief Data value. */

/** @subsection Enumerated SPDIFRX Register Field Definitions */

extern const field32_t SPDIFRX_DR_10_DRNLx[3]; /** @brief Data value. */

/**************************************************************************************************
 * @section ADCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated ADCx Register Definitions */

extern rw_reg32_t const ADCx_ISR[4];      /** @brief ADC interrupt and status register. */
extern rw_reg32_t const ADCx_IER[4];      /** @brief ADC interrupt enable register. */
extern rw_reg32_t const ADCx_CR[4];       /** @brief ADC control register. */
extern rw_reg32_t const ADCx_CFGR[4];     /** @brief ADC configuration register 1. */
extern rw_reg32_t const ADCx_CFGR2[4];    /** @brief ADC configuration register 2. */
extern rw_reg32_t const ADCx_SMPR1[4];    /** @brief ADC sampling time register 1. */
extern rw_reg32_t const ADCx_SMPR2[4];    /** @brief ADC sampling time register 2. */
extern rw_reg32_t const ADCx_LTR1[4];     /** @brief ADC analog watchdog 1 threshold register. */
extern rw_reg32_t const ADCx_LHTR1[4];    /** @brief ADC analog watchdog 2 threshold register. */
extern rw_reg32_t const ADCx_SQR1[4];     /** @brief ADC group regular sequencer ranks register 1. */
extern rw_reg32_t const ADCx_SQR2[4];     /** @brief ADC group regular sequencer ranks register 2. */
extern rw_reg32_t const ADCx_SQR3[4];     /** @brief ADC group regular sequencer ranks register 3. */
extern rw_reg32_t const ADCx_SQR4[4];     /** @brief ADC group regular sequencer ranks register 4. */
extern ro_reg32_t const ADCx_DR[4];       /** @brief ADC group regular conversion data register. */
extern rw_reg32_t const ADCx_JSQR[4];     /** @brief ADC group injected sequencer register. */
extern rw_reg32_t const ADCx_OFRx[4][5];  /** @brief ADC offset number 1 register. */
extern ro_reg32_t const ADCx_JDR1[4];     /** @brief ADC group injected sequencer rank 1 register. */
extern ro_reg32_t const ADCx_JDR2[4];     /** @brief ADC group injected sequencer rank 2 register. */
extern ro_reg32_t const ADCx_JDR3[4];     /** @brief ADC group injected sequencer rank 3 register. */
extern ro_reg32_t const ADCx_JDR4[4];     /** @brief ADC group injected sequencer rank 4 register. */
extern rw_reg32_t const ADCx_AWD2CR[4];   /** @brief ADC analog watchdog 2 configuration register. */
extern rw_reg32_t const ADCx_AWD3CR[4];   /** @brief ADC analog watchdog 3 configuration register. */
extern rw_reg32_t const ADCx_DIFSEL[4];   /** @brief ADC channel differential or single-ended mode selection register. */
extern rw_reg32_t const ADCx_CALFACT[4];  /** @brief ADC calibration factors register. */
extern rw_reg32_t const ADCx_PCSEL[4];    /** @brief ADC pre channel selection register. */
extern rw_reg32_t const ADCx_LTR2[4];     /** @brief ADC watchdog lower threshold register 2. */
extern rw_reg32_t const ADCx_HTR2[4];     /** @brief ADC watchdog higher threshold register 2. */
extern rw_reg32_t const ADCx_LTR3[4];     /** @brief ADC watchdog lower threshold register 3. */
extern rw_reg32_t const ADCx_HTR3[4];     /** @brief ADC watchdog higher threshold register 3. */
extern rw_reg32_t const ADCx_CALFACT2[4]; /** @brief ADC calibration factor register 2. */

/** @subsection ADCx Register Field Definitions */

extern const field32_t ADCx_ISR_JQOVF;           /** @brief ADC group injected contexts queue overflow flag. */
extern const field32_t ADCx_ISR_JEOS;            /** @brief ADC group injected end of sequence conversions flag. */
extern const field32_t ADCx_ISR_JEOC;            /** @brief ADC group injected end of unitary conversion flag. */
extern const field32_t ADCx_ISR_OVR;             /** @brief ADC group regular overrun flag. */
extern const field32_t ADCx_ISR_EOS;             /** @brief ADC group regular end of sequence conversions flag. */
extern const field32_t ADCx_ISR_EOC;             /** @brief ADC group regular end of unitary conversion flag. */
extern const field32_t ADCx_ISR_EOSMP;           /** @brief ADC group regular end of sampling flag. */
extern const field32_t ADCx_ISR_ADRDY;           /** @brief ADC ready flag. */
extern const field32_t ADCx_IER_JQOVFIE;         /** @brief ADC group injected contexts queue overflow interrupt. */
extern const field32_t ADCx_IER_JEOSIE;          /** @brief ADC group injected end of sequence conversions interrupt. */
extern const field32_t ADCx_IER_JEOCIE;          /** @brief ADC group injected end of unitary conversion interrupt. */
extern const field32_t ADCx_IER_OVRIE;           /** @brief ADC group regular overrun interrupt. */
extern const field32_t ADCx_IER_EOSIE;           /** @brief ADC group regular end of sequence conversions interrupt. */
extern const field32_t ADCx_IER_EOCIE;           /** @brief ADC group regular end of unitary conversion interrupt. */
extern const field32_t ADCx_IER_EOSMPIE;         /** @brief ADC group regular end of sampling interrupt. */
extern const field32_t ADCx_IER_ADRDYIE;         /** @brief ADC ready interrupt. */
extern const field32_t ADCx_CR_ADCAL;            /** @brief ADC calibration. */
extern const field32_t ADCx_CR_ADCALDIF;         /** @brief ADC differential mode for calibration. */
extern const field32_t ADCx_CR_DEEPPWD;          /** @brief ADC deep power down enable. */
extern const field32_t ADCx_CR_ADVREGEN;         /** @brief ADC voltage regulator enable. */
extern const field32_t ADCx_CR_ADCALLIN;         /** @brief Linearity calibration. */
extern const field32_t ADCx_CR_BOOST;            /** @brief Boost mode control. */
extern const field32_t ADCx_CR_JADSTP;           /** @brief ADC group injected conversion stop. */
extern const field32_t ADCx_CR_ADSTP;            /** @brief ADC group regular conversion stop. */
extern const field32_t ADCx_CR_JADSTART;         /** @brief ADC group injected conversion start. */
extern const field32_t ADCx_CR_ADSTART;          /** @brief ADC group regular conversion start. */
extern const field32_t ADCx_CR_ADDIS;            /** @brief ADC disable. */
extern const field32_t ADCx_CR_ADEN;             /** @brief ADC enable. */
extern const field32_t ADCx_CFGR_JQDIS;          /** @brief ADC group injected contexts queue disable. */
extern const field32_t ADCx_CFGR_AWDCH1CH;       /** @brief ADC analog watchdog 1 monitored channel selection. */
extern const field32_t ADCx_CFGR_JAUTO;          /** @brief ADC group injected automatic trigger mode. */
extern const field32_t ADCx_CFGR_JAWD1EN;        /** @brief ADC analog watchdog 1 enable on scope ADC group injected. */
extern const field32_t ADCx_CFGR_AWD1EN;         /** @brief ADC analog watchdog 1 enable on scope ADC group regular. */
extern const field32_t ADCx_CFGR_AWD1SGL;        /** @brief ADC analog watchdog 1 monitoring a single channel or all channels. */
extern const field32_t ADCx_CFGR_JQM;            /** @brief ADC group injected contexts queue mode. */
extern const field32_t ADCx_CFGR_JDISCEN;        /** @brief ADC group injected sequencer discontinuous mode. */
extern const field32_t ADCx_CFGR_DISCNUM;        /** @brief ADC group regular sequencer discontinuous number of ranks. */
extern const field32_t ADCx_CFGR_DISCEN;         /** @brief ADC group regular sequencer discontinuous mode. */
extern const field32_t ADCx_CFGR_AUTDLY;         /** @brief ADC low power auto wait. */
extern const field32_t ADCx_CFGR_CONT;           /** @brief ADC group regular continuous conversion mode. */
extern const field32_t ADCx_CFGR_OVRMOD;         /** @brief ADC group regular overrun configuration. */
extern const field32_t ADCx_CFGR_EXTEN;          /** @brief ADC group regular external trigger polarity. */
extern const field32_t ADCx_CFGR_EXTSEL;         /** @brief ADC group regular external trigger source. */
extern const field32_t ADCx_CFGR_RES;            /** @brief ADC data resolution. */
extern const field32_t ADCx_CFGR_DMNGT;          /** @brief ADC DMA transfer enable. */
extern const field32_t ADCx_CFGR2_ROVSE;         /** @brief ADC oversampler enable on scope ADC group regular. */
extern const field32_t ADCx_CFGR2_JOVSE;         /** @brief ADC oversampler enable on scope ADC group injected. */
extern const field32_t ADCx_CFGR2_OVSS;          /** @brief ADC oversampling shift. */
extern const field32_t ADCx_CFGR2_TROVS;         /** @brief ADC oversampling discontinuous mode (triggered mode) for ADC group regular. */
extern const field32_t ADCx_CFGR2_ROVSM;         /** @brief Regular oversampling mode. */
extern const field32_t ADCx_CFGR2_OSR;           /** @brief Oversampling ratio. */
extern const field32_t ADCx_CFGR2_LSHIFT;        /** @brief Left shift factor. */
extern const field32_t ADCx_LTR1_LTR1;           /** @brief ADC analog watchdog 1 threshold low. */
extern const field32_t ADCx_LHTR1_LHTR1;         /** @brief ADC analog watchdog 2 threshold low. */
extern const field32_t ADCx_SQR1_L3;             /** @brief L3. */
extern const field32_t ADCx_DR_RDATA;            /** @brief ADC group regular conversion data. */
extern const field32_t ADCx_JSQR_JEXTEN;         /** @brief ADC group injected external trigger polarity. */
extern const field32_t ADCx_JSQR_JEXTSEL;        /** @brief ADC group injected external trigger source. */
extern const field32_t ADCx_JSQR_JL;             /** @brief ADC group injected sequencer scan length. */
extern const field32_t ADCx_OFRx_SSATE;          /** @brief ADC offset number 1 enable. */
extern const field32_t ADCx_OFRx_OFFSET1_CH;     /** @brief ADC offset number 1 channel selection. */
extern const field32_t ADCx_OFRx_OFFSET1;        /** @brief ADC offset number 1 offset level. */
extern const field32_t ADCx_AWD2CR_AWD2CH;       /** @brief ADC analog watchdog 2 monitored channel selection. */
extern const field32_t ADCx_AWD3CR_AWD3CH;       /** @brief ADC analog watchdog 3 monitored channel selection. */
extern const field32_t ADCx_DIFSEL_DIFSEL;       /** @brief ADC channel differential or single-ended mode for channel. */
extern const field32_t ADCx_CALFACT_CALFACT_D;   /** @brief ADC calibration factor in differential mode. */
extern const field32_t ADCx_CALFACT_CALFACT_S;   /** @brief ADC calibration factor in single-ended mode. */
extern const field32_t ADCx_PCSEL_PCSEL;         /** @brief Channel x (vinp[i]) pre selection. */
extern const field32_t ADCx_LTR2_LTR2;           /** @brief Analog watchdog 2 lower threshold. */
extern const field32_t ADCx_HTR2_HTR2;           /** @brief Analog watchdog 2 higher threshold. */
extern const field32_t ADCx_LTR3_LTR3;           /** @brief Analog watchdog 3 lower threshold. */
extern const field32_t ADCx_HTR3_HTR3;           /** @brief Analog watchdog 3 higher threshold. */
extern const field32_t ADCx_CALFACT2_LINCALFACT; /** @brief Linearity calibration factor. */

/** @subsection Enumerated ADCx Register Field Definitions */

extern const field32_t ADCx_ISR_AWDx[4];       /** @brief ADC analog watchdog 1 flag. */
extern const field32_t ADCx_IER_AWDxIE[4];     /** @brief ADC analog watchdog 1 interrupt. */
extern const field32_t ADCx_CR_LINCALRDYWx[7]; /** @brief Linearity calibration ready word 1. */
extern const field32_t ADCx_CFGR2_RSHIFTx[5];  /** @brief Right-shift data after offset 1 correction. */
extern const field32_t ADCx_SMPR1_SMPx[10];    /** @brief ADC channel 1 sampling time selection. */
extern const field32_t ADCx_SMPR2_SMP1x[10];   /** @brief ADC channel 10 sampling time selection. */
extern const field32_t ADCx_SQR1_SQx[5];       /** @brief ADC group regular sequencer rank 1. */
extern const field32_t ADCx_SQR3_SQ1x[5];      /** @brief ADC group regular sequencer rank 10. */
extern const field32_t ADCx_JSQR_JSQx[5];      /** @brief ADC group injected sequencer rank 1. */

/**************************************************************************************************
 * @section ADCx_COMMON Definitions
 **************************************************************************************************/

/** @subsection Enumerated ADCx_COMMON Register Definitions */

extern ro_reg32_t const ADCx_COMMON_CSR[13];  /** @brief ADC common status register. */
extern rw_reg32_t const ADCx_COMMON_CCR[13];  /** @brief ADC common control register. */
extern ro_reg32_t const ADCx_COMMON_CDR[13];  /** @brief ADC common regular data register for dual and triple modes. */
extern ro_reg32_t const ADCx_COMMON_CDR2[13]; /** @brief ADC x common regular data register for 32-bit dual mode. */

/** @subsection ADCx_COMMON Register Field Definitions */

extern const field32_t ADCx_COMMON_CSR_ADRDY_MST; /** @brief Master ADC ready. */
extern const field32_t ADCx_COMMON_CSR_EOSMP_MST; /** @brief End of sampling phase flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_EOC_MST;   /** @brief End of regular conversion of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_EOS_MST;   /** @brief End of regular sequence flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_OVR_MST;   /** @brief Overrun flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_JEOC_MST;  /** @brief End of injected conversion flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_JEOS_MST;  /** @brief End of injected sequence flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_JQOVF_MST; /** @brief Injected context queue overflow flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_ADRDY_SLV; /** @brief Slave ADC ready. */
extern const field32_t ADCx_COMMON_CSR_EOSMP_SLV; /** @brief End of sampling phase flag of the slave ADC. */
extern const field32_t ADCx_COMMON_CSR_EOC_SLV;   /** @brief End of regular conversion of the slave ADC. */
extern const field32_t ADCx_COMMON_CSR_EOS_SLV;   /** @brief End of regular sequence flag of the slave ADC. */
extern const field32_t ADCx_COMMON_CSR_OVR_SLV;   /** @brief Overrun flag of the slave ADC. */
extern const field32_t ADCx_COMMON_CSR_JEOC_SLV;  /** @brief End of injected conversion flag of the slave ADC. */
extern const field32_t ADCx_COMMON_CSR_JEOS_SLV;  /** @brief End of injected sequence flag of the slave ADC. */
extern const field32_t ADCx_COMMON_CSR_JQOVF_SLV; /** @brief Injected context queue overflow flag of the slave ADC. */
extern const field32_t ADCx_COMMON_CCR_DUAL;      /** @brief Dual ADC mode selection. */
extern const field32_t ADCx_COMMON_CCR_DELAY;     /** @brief Delay between 2 sampling phases. */
extern const field32_t ADCx_COMMON_CCR_DAMDF;     /** @brief Dual ADC mode data format. */
extern const field32_t ADCx_COMMON_CCR_CKMODE;    /** @brief ADC clock mode. */
extern const field32_t ADCx_COMMON_CCR_PRESC;     /** @brief ADC prescaler. */
extern const field32_t ADCx_COMMON_CCR_VREFEN;    /** @brief VREFINT enable. */
extern const field32_t ADCx_COMMON_CCR_TSEN;      /** @brief Temperature sensor enable. */
extern const field32_t ADCx_COMMON_CCR_VBATEN;    /** @brief VBAT enable. */
extern const field32_t ADCx_COMMON_CDR_RDATA_SLV; /** @brief Regular data of the slave ADC. */
extern const field32_t ADCx_COMMON_CDR_RDATA_MST; /** @brief Regular data of the master ADC. */

/** @subsection Enumerated ADCx_COMMON Register Field Definitions */

extern const field32_t ADCx_COMMON_CSR_AWDx_MST[4]; /** @brief Analog watchdog 1 flag of the master ADC. */
extern const field32_t ADCx_COMMON_CSR_AWDx_SLV[4]; /** @brief Analog watchdog 1 flag of the slave ADC. */

/**************************************************************************************************
 * @section CRC Definitions
 **************************************************************************************************/

/** @subsection CRC Register Definitions */

extern rw_reg32_t const CRC_DR;   /** @brief Data register. */
extern rw_reg32_t const CRC_IDR;  /** @brief Independent data register. */
extern rw_reg32_t const CRC_CR;   /** @brief Control register. */
extern rw_reg32_t const CRC_INIT; /** @brief Initial CRC value. */
extern rw_reg32_t const CRC_POL;  /** @brief CRC polynomial. */

/** @subsection CRC Register Field Definitions */

extern const field32_t CRC_CR_RESET;    /** @brief RESET bit. */
extern const field32_t CRC_CR_POLYSIZE; /** @brief Polynomial size. */
extern const field32_t CRC_CR_REV_IN;   /** @brief Reverse input data. */
extern const field32_t CRC_CR_REV_OUT;  /** @brief Reverse output data. */

/**************************************************************************************************
 * @section RCC Definitions
 **************************************************************************************************/

/** @subsection RCC Register Definitions */

extern rw_reg32_t const RCC_CR;            /** @brief Clock control register. */
extern rw_reg32_t const RCC_ICSCR;         /** @brief RCC internal clock source calibration register. */
extern ro_reg32_t const RCC_CRRCR;         /** @brief RCC clock recovery RC register. */
extern rw_reg32_t const RCC_CFGR;          /** @brief RCC clock configuration register. */
extern rw_reg32_t const RCC_D1CFGR;        /** @brief RCC domain 1 clock configuration register. */
extern rw_reg32_t const RCC_D2CFGR;        /** @brief RCC domain 2 clock configuration register. */
extern rw_reg32_t const RCC_D3CFGR;        /** @brief RCC domain 3 clock configuration register. */
extern rw_reg32_t const RCC_PLLCKSELR;     /** @brief RCC plls clock source selection register. */
extern rw_reg32_t const RCC_PLLCFGR;       /** @brief RCC plls configuration register. */
extern rw_reg32_t const RCC_PLL1DIVR;      /** @brief RCC PLL1 dividers configuration register. */
extern rw_reg32_t const RCC_PLL1FRACR;     /** @brief RCC PLL1 fractional divider register. */
extern rw_reg32_t const RCC_PLL2DIVR;      /** @brief RCC PLL2 dividers configuration register. */
extern rw_reg32_t const RCC_PLL2FRACR;     /** @brief RCC PLL2 fractional divider register. */
extern rw_reg32_t const RCC_PLL3DIVR;      /** @brief RCC PLL3 dividers configuration register. */
extern rw_reg32_t const RCC_PLL3FRACR;     /** @brief RCC PLL3 fractional divider register. */
extern rw_reg32_t const RCC_D1CCIPR;       /** @brief RCC domain 1 kernel clock configuration register. */
extern rw_reg32_t const RCC_D2CCIP1R;      /** @brief RCC domain 2 kernel clock configuration register. */
extern rw_reg32_t const RCC_D2CCIP2R;      /** @brief RCC domain 2 kernel clock configuration register. */
extern rw_reg32_t const RCC_D3CCIPR;       /** @brief RCC domain 3 kernel clock configuration register. */
extern rw_reg32_t const RCC_CIER;          /** @brief RCC clock source interrupt enable register. */
extern rw_reg32_t const RCC_CIFR;          /** @brief RCC clock source interrupt flag register. */
extern rw_reg32_t const RCC_CICR;          /** @brief RCC clock source interrupt clear register. */
extern rw_reg32_t const RCC_BDCR;          /** @brief RCC backup domain control register. */
extern rw_reg32_t const RCC_CSR;           /** @brief RCC clock control and status register. */
extern rw_reg32_t const RCC_AHB3RSTR;      /** @brief RCC AHB3 reset register. */
extern rw_reg32_t const RCC_AHB1RSTR;      /** @brief RCC AHB1 peripheral reset register. */
extern rw_reg32_t const RCC_AHB2RSTR;      /** @brief RCC AHB2 peripheral reset register. */
extern rw_reg32_t const RCC_AHB4RSTR;      /** @brief RCC AHB4 peripheral reset register. */
extern rw_reg32_t const RCC_APB3RSTR;      /** @brief RCC APB3 peripheral reset register. */
extern rw_reg32_t const RCC_APB1LRSTR;     /** @brief RCC APB1 peripheral reset register. */
extern rw_reg32_t const RCC_APB1HRSTR;     /** @brief RCC APB1 peripheral reset register. */
extern rw_reg32_t const RCC_APB2RSTR;      /** @brief RCC APB2 peripheral reset register. */
extern rw_reg32_t const RCC_APB4RSTR;      /** @brief RCC APB4 peripheral reset register. */
extern rw_reg32_t const RCC_GCR;           /** @brief RCC global control register. */
extern rw_reg32_t const RCC_D3AMR;         /** @brief RCC D3 autonomous mode register. */
extern rw_reg32_t const RCC_RSR;           /** @brief RCC reset status register. */
extern rw_reg32_t const RCC_C1_RSR;        /** @brief RCC reset status register. */
extern rw_reg32_t const RCC_C1_AHB3ENR;    /** @brief RCC AHB3 clock register. */
extern rw_reg32_t const RCC_AHB3ENR;       /** @brief RCC AHB3 clock register. */
extern rw_reg32_t const RCC_AHB1ENR;       /** @brief RCC AHB1 clock register. */
extern rw_reg32_t const RCC_C1_AHB1ENR;    /** @brief RCC AHB1 clock register. */
extern rw_reg32_t const RCC_C1_AHB2ENR;    /** @brief RCC AHB2 clock register. */
extern rw_reg32_t const RCC_AHB2ENR;       /** @brief RCC AHB2 clock register. */
extern rw_reg32_t const RCC_AHB4ENR;       /** @brief RCC AHB4 clock register. */
extern rw_reg32_t const RCC_C1_AHB4ENR;    /** @brief RCC AHB4 clock register. */
extern rw_reg32_t const RCC_C1_APB3ENR;    /** @brief RCC APB3 clock register. */
extern rw_reg32_t const RCC_APB3ENR;       /** @brief RCC APB3 clock register. */
extern rw_reg32_t const RCC_APB1LENR;      /** @brief RCC APB1 clock register. */
extern rw_reg32_t const RCC_C1_APB1LENR;   /** @brief RCC APB1 clock register. */
extern rw_reg32_t const RCC_APB1HENR;      /** @brief RCC APB1 clock register. */
extern rw_reg32_t const RCC_C1_APB1HENR;   /** @brief RCC APB1 clock register. */
extern rw_reg32_t const RCC_C1_APB2ENR;    /** @brief RCC APB2 clock register. */
extern rw_reg32_t const RCC_APB2ENR;       /** @brief RCC APB2 clock register. */
extern rw_reg32_t const RCC_APB4ENR;       /** @brief RCC APB4 clock register. */
extern rw_reg32_t const RCC_C1_APB4ENR;    /** @brief RCC APB4 clock register. */
extern rw_reg32_t const RCC_C1_AHB3LPENR;  /** @brief RCC AHB3 sleep clock register. */
extern rw_reg32_t const RCC_AHB3LPENR;     /** @brief RCC AHB3 sleep clock register. */
extern rw_reg32_t const RCC_AHB1LPENR;     /** @brief RCC AHB1 sleep clock register. */
extern rw_reg32_t const RCC_C1_AHB1LPENR;  /** @brief RCC AHB1 sleep clock register. */
extern rw_reg32_t const RCC_C1_AHB2LPENR;  /** @brief RCC AHB2 sleep clock register. */
extern rw_reg32_t const RCC_AHB2LPENR;     /** @brief RCC AHB2 sleep clock register. */
extern rw_reg32_t const RCC_AHB4LPENR;     /** @brief RCC AHB4 sleep clock register. */
extern rw_reg32_t const RCC_C1_AHB4LPENR;  /** @brief RCC AHB4 sleep clock register. */
extern rw_reg32_t const RCC_C1_APB3LPENR;  /** @brief RCC APB3 sleep clock register. */
extern rw_reg32_t const RCC_APB3LPENR;     /** @brief RCC APB3 sleep clock register. */
extern rw_reg32_t const RCC_APB1LLPENR;    /** @brief RCC APB1 low sleep clock register. */
extern rw_reg32_t const RCC_C1_APB1LLPENR; /** @brief RCC APB1 low sleep clock register. */
extern rw_reg32_t const RCC_C1_APB1HLPENR; /** @brief RCC APB1 high sleep clock register. */
extern rw_reg32_t const RCC_APB1HLPENR;    /** @brief RCC APB1 high sleep clock register. */
extern rw_reg32_t const RCC_APB2LPENR;     /** @brief RCC APB2 sleep clock register. */
extern rw_reg32_t const RCC_C1_APB2LPENR;  /** @brief RCC APB2 sleep clock register. */
extern rw_reg32_t const RCC_C1_APB4LPENR;  /** @brief RCC APB4 sleep clock register. */
extern rw_reg32_t const RCC_APB4LPENR;     /** @brief RCC APB4 sleep clock register. */

/** @subsection RCC Register Field Definitions */

extern const field32_t RCC_CR_HSION;                       /** @brief Internal high-speed clock enable. */
extern const field32_t RCC_CR_HSIKERON;                    /** @brief High speed internal clock enable in stop mode. */
extern const field32_t RCC_CR_HSIRDY;                      /** @brief HSI clock ready flag. */
extern const field32_t RCC_CR_HSIDIV;                      /** @brief HSI clock divider. */
extern const field32_t RCC_CR_HSIDIVF;                     /** @brief HSI divider flag. */
extern const field32_t RCC_CR_CSION;                       /** @brief CSI clock enable. */
extern const field32_t RCC_CR_CSIRDY;                      /** @brief CSI clock ready flag. */
extern const field32_t RCC_CR_CSIKERON;                    /** @brief CSI clock enable in stop mode. */
extern const field32_t RCC_CR_RC48ON;                      /** @brief RC48 clock enable. */
extern const field32_t RCC_CR_RC48RDY;                     /** @brief RC48 clock ready flag. */
extern const field32_t RCC_CR_HSEON;                       /** @brief HSE clock enable. */
extern const field32_t RCC_CR_HSERDY;                      /** @brief HSE clock ready flag. */
extern const field32_t RCC_CR_HSEBYP;                      /** @brief HSE clock bypass. */
extern const field32_t RCC_CR_HSECSSON;                    /** @brief HSE clock security system enable. */
extern const field32_t RCC_ICSCR_HSICAL;                   /** @brief HSI clock calibration. */
extern const field32_t RCC_ICSCR_HSITRIM;                  /** @brief HSI clock trimming. */
extern const field32_t RCC_ICSCR_CSICAL;                   /** @brief CSI clock calibration. */
extern const field32_t RCC_ICSCR_CSITRIM;                  /** @brief CSI clock trimming. */
extern const field32_t RCC_CRRCR_RC48CAL;                  /** @brief Internal RC 48 mhz clock calibration. */
extern const field32_t RCC_CFGR_SW;                        /** @brief System clock switch. */
extern const field32_t RCC_CFGR_SWS;                       /** @brief System clock switch status. */
extern const field32_t RCC_CFGR_STOPWUCK;                  /** @brief System clock selection after a wake up from system stop. */
extern const field32_t RCC_CFGR_STOPKERWUCK;               /** @brief Kernel clock selection after a wake up from system stop. */
extern const field32_t RCC_CFGR_RTCPRE;                    /** @brief HSE division factor for RTC clock. */
extern const field32_t RCC_CFGR_HRTIMSEL;                  /** @brief High resolution timer clock prescaler selection. */
extern const field32_t RCC_CFGR_TIMPRE;                    /** @brief Timers clocks prescaler selection. */
extern const field32_t RCC_D1CFGR_HPRE;                    /** @brief D1 domain AHB prescaler. */
extern const field32_t RCC_D1CFGR_D1PPRE;                  /** @brief D1 domain APB3 prescaler. */
extern const field32_t RCC_D1CFGR_D1CPRE;                  /** @brief D1 domain core prescaler. */
extern const field32_t RCC_D3CFGR_D3PPRE;                  /** @brief D3 domain APB4 prescaler. */
extern const field32_t RCC_PLLCKSELR_PLLSRC;               /** @brief DIVMx and plls clock source selection. */
extern const field32_t RCC_PLL1DIVR_DIVN1;                 /** @brief Multiplication factor for PLL1 VCO. */
extern const field32_t RCC_PLL1DIVR_DIVP1;                 /** @brief PLL1 DIVP division factor. */
extern const field32_t RCC_PLL1DIVR_DIVQ1;                 /** @brief PLL1 DIVQ division factor. */
extern const field32_t RCC_PLL1DIVR_DIVR1;                 /** @brief PLL1 DIVR division factor. */
extern const field32_t RCC_PLL1FRACR_FRACN1;               /** @brief Fractional part of the multiplication factor for PLL1 VCO. */
extern const field32_t RCC_PLL2DIVR_DIVN1;                 /** @brief Multiplication factor for PLL1 VCO. */
extern const field32_t RCC_PLL2DIVR_DIVP1;                 /** @brief PLL1 DIVP division factor. */
extern const field32_t RCC_PLL2DIVR_DIVQ1;                 /** @brief PLL1 DIVQ division factor. */
extern const field32_t RCC_PLL2DIVR_DIVR1;                 /** @brief PLL1 DIVR division factor. */
extern const field32_t RCC_PLL2FRACR_FRACN2;               /** @brief Fractional part of the multiplication factor for PLL VCO. */
extern const field32_t RCC_PLL3DIVR_DIVN3;                 /** @brief Multiplication factor for PLL1 VCO. */
extern const field32_t RCC_PLL3DIVR_DIVP3;                 /** @brief PLL DIVP division factor. */
extern const field32_t RCC_PLL3DIVR_DIVQ3;                 /** @brief PLL DIVQ division factor. */
extern const field32_t RCC_PLL3DIVR_DIVR3;                 /** @brief PLL DIVR division factor. */
extern const field32_t RCC_PLL3FRACR_FRACN3;               /** @brief Fractional part of the multiplication factor for PLL3 VCO. */
extern const field32_t RCC_D1CCIPR_FMCSRC;                 /** @brief FMC kernel clock source selection. */
extern const field32_t RCC_D1CCIPR_QSPISRC;                /** @brief QUADSPI kernel clock source selection. */
extern const field32_t RCC_D1CCIPR_SDMMCSRC;               /** @brief SDMMC kernel clock source selection. */
extern const field32_t RCC_D1CCIPR_CKPERSRC;               /** @brief Per_ck clock source selection. */
extern const field32_t RCC_D2CCIP1R_SAI1SRC;               /** @brief SAI1 and DFSDM1 kernel aclk clock source selection. */
extern const field32_t RCC_D2CCIP1R_SAI23SRC;              /** @brief SAI2 and SAI3 kernel clock source selection. */
extern const field32_t RCC_D2CCIP1R_SPI123SRC;             /** @brief SPI/I2S1,2 and 3 kernel clock source selection. */
extern const field32_t RCC_D2CCIP1R_SPI45SRC;              /** @brief SPI4 and 5 kernel clock source selection. */
extern const field32_t RCC_D2CCIP1R_SPDIFSRC;              /** @brief SPDIFRX kernel clock source selection. */
extern const field32_t RCC_D2CCIP1R_DFSDM1SRC;             /** @brief DFSDM1 kernel clk clock source selection. */
extern const field32_t RCC_D2CCIP1R_FDCANSRC;              /** @brief FDCAN kernel clock source selection. */
extern const field32_t RCC_D2CCIP1R_SWPSRC;                /** @brief SWPMI kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_USART234578SRC;        /** @brief USART2/3, UART4,5, 7/8 (APB1) kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_USART16SRC;            /** @brief USART1 and 6 kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_RNGSRC;                /** @brief RNG kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_I2C123SRC;             /** @brief I2C1,2,3 kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_USBSRC;                /** @brief USBOTG 1 and 2 kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_CECSRC;                /** @brief HDMI-CEC kernel clock source selection. */
extern const field32_t RCC_D2CCIP2R_LPTIM1SRC;             /** @brief LPTIM1 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_LPUART1SRC;             /** @brief LPUART1 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_I2C4SRC;                /** @brief I2C4 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_LPTIM2SRC;              /** @brief LPTIM2 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_LPTIM345SRC;            /** @brief LPTIM3,4,5 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_ADCSRC;                 /** @brief SAR ADC kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_SAI4ASRC;               /** @brief Sub-Block A of SAI4 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_SAI4BSRC;               /** @brief Sub-Block B of SAI4 kernel clock source selection. */
extern const field32_t RCC_D3CCIPR_SPI6SRC;                /** @brief SPI6 kernel clock source selection. */
extern const field32_t RCC_CIER_LSIRDYIE;                  /** @brief LSI ready interrupt enable. */
extern const field32_t RCC_CIER_LSERDYIE;                  /** @brief LSE ready interrupt enable. */
extern const field32_t RCC_CIER_HSIRDYIE;                  /** @brief HSI ready interrupt enable. */
extern const field32_t RCC_CIER_HSERDYIE;                  /** @brief HSE ready interrupt enable. */
extern const field32_t RCC_CIER_CSIRDYIE;                  /** @brief CSI ready interrupt enable. */
extern const field32_t RCC_CIER_RC48RDYIE;                 /** @brief RC48 ready interrupt enable. */
extern const field32_t RCC_CIER_LSECSSIE;                  /** @brief LSE clock security system interrupt enable. */
extern const field32_t RCC_CIFR_LSIRDYF;                   /** @brief LSI ready interrupt flag. */
extern const field32_t RCC_CIFR_LSERDYF;                   /** @brief LSE ready interrupt flag. */
extern const field32_t RCC_CIFR_HSIRDYF;                   /** @brief HSI ready interrupt flag. */
extern const field32_t RCC_CIFR_HSERDYF;                   /** @brief HSE ready interrupt flag. */
extern const field32_t RCC_CIFR_CSIRDY;                    /** @brief CSI ready interrupt flag. */
extern const field32_t RCC_CIFR_RC48RDYF;                  /** @brief RC48 ready interrupt flag. */
extern const field32_t RCC_CIFR_LSECSSF;                   /** @brief LSE clock security system interrupt flag. */
extern const field32_t RCC_CIFR_HSECSSF;                   /** @brief HSE clock security system interrupt flag. */
extern const field32_t RCC_CICR_LSIRDYC;                   /** @brief LSI ready interrupt clear. */
extern const field32_t RCC_CICR_LSERDYC;                   /** @brief LSE ready interrupt clear. */
extern const field32_t RCC_CICR_HSIRDYC;                   /** @brief HSI ready interrupt clear. */
extern const field32_t RCC_CICR_HSERDYC;                   /** @brief HSE ready interrupt clear. */
extern const field32_t RCC_CICR_HSE_READY_INTERRUPT_CLEAR; /** @brief CSI ready interrupt clear. */
extern const field32_t RCC_CICR_RC48RDYC;                  /** @brief RC48 ready interrupt clear. */
extern const field32_t RCC_CICR_LSECSSC;                   /** @brief LSE clock security system interrupt clear. */
extern const field32_t RCC_CICR_HSECSSC;                   /** @brief HSE clock security system interrupt clear. */
extern const field32_t RCC_BDCR_LSEON;                     /** @brief LSE oscillator enabled. */
extern const field32_t RCC_BDCR_LSERDY;                    /** @brief LSE oscillator ready. */
extern const field32_t RCC_BDCR_LSEBYP;                    /** @brief LSE oscillator bypass. */
extern const field32_t RCC_BDCR_LSEDRV;                    /** @brief LSE oscillator driving capability. */
extern const field32_t RCC_BDCR_LSECSSON;                  /** @brief LSE clock security system enable. */
extern const field32_t RCC_BDCR_LSECSSD;                   /** @brief LSE clock security system failure detection. */
extern const field32_t RCC_BDCR_RTCSRC;                    /** @brief RTC clock source selection. */
extern const field32_t RCC_BDCR_RTCEN;                     /** @brief RTC clock enable. */
extern const field32_t RCC_BDCR_VSWRST;                    /** @brief VSwitch domain software reset. */
extern const field32_t RCC_CSR_LSION;                      /** @brief LSI oscillator enable. */
extern const field32_t RCC_CSR_LSIRDY;                     /** @brief LSI oscillator ready. */
extern const field32_t RCC_AHB3RSTR_MDMARST;               /** @brief MDMA block reset. */
extern const field32_t RCC_AHB3RSTR_DMA2DRST;              /** @brief DMA2D block reset. */
extern const field32_t RCC_AHB3RSTR_JPGDECRST;             /** @brief JPGDEC block reset. */
extern const field32_t RCC_AHB3RSTR_FMCRST;                /** @brief FMC block reset. */
extern const field32_t RCC_AHB3RSTR_QSPIRST;               /** @brief QUADSPI and QUADSPI delay block reset. */
extern const field32_t RCC_AHB3RSTR_SDMMC1RST;             /** @brief SDMMC1 and SDMMC1 delay block reset. */
extern const field32_t RCC_AHB3RSTR_CPURST;                /** @brief CPU reset. */
extern const field32_t RCC_AHB1RSTR_ADC12RST;              /** @brief ADC1&2 block reset. */
extern const field32_t RCC_AHB1RSTR_ETH1MACRST;            /** @brief ETH1MAC block reset. */
extern const field32_t RCC_AHB2RSTR_CAMITFRST;             /** @brief CAMITF block reset. */
extern const field32_t RCC_AHB2RSTR_CRYPTRST;              /** @brief Cryptography block reset. */
extern const field32_t RCC_AHB2RSTR_HASHRST;               /** @brief Hash block reset. */
extern const field32_t RCC_AHB2RSTR_RNGRST;                /** @brief Random number generator block reset. */
extern const field32_t RCC_AHB2RSTR_SDMMC2RST;             /** @brief SDMMC2 and SDMMC2 delay block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOARST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOBRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOCRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIODRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOERST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOFRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOGRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOHRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOIRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOJRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_GPIOKRST;              /** @brief GPIO block reset. */
extern const field32_t RCC_AHB4RSTR_CRCRST;                /** @brief CRC block reset. */
extern const field32_t RCC_AHB4RSTR_BDMARST;               /** @brief BDMA block reset. */
extern const field32_t RCC_AHB4RSTR_ADC3RST;               /** @brief ADC3 block reset. */
extern const field32_t RCC_AHB4RSTR_HSEMRST;               /** @brief HSEM block reset. */
extern const field32_t RCC_APB3RSTR_LTDCRST;               /** @brief LTDC block reset. */
extern const field32_t RCC_APB1LRSTR_LPTIM1RST;            /** @brief TIM block reset. */
extern const field32_t RCC_APB1LRSTR_SPDIFRXRST;           /** @brief SPDIFRX block reset. */
extern const field32_t RCC_APB1LRSTR_HDMICECRST;           /** @brief HDMI-CEC block reset. */
extern const field32_t RCC_APB1LRSTR_DAC12RST;             /** @brief DAC1 and 2 blocks reset. */
extern const field32_t RCC_APB1HRSTR_CRSRST;               /** @brief Clock recovery system reset. */
extern const field32_t RCC_APB1HRSTR_SWPRST;               /** @brief SWPMI block reset. */
extern const field32_t RCC_APB1HRSTR_OPAMPRST;             /** @brief OPAMP block reset. */
extern const field32_t RCC_APB1HRSTR_MDIOSRST;             /** @brief MDIOS block reset. */
extern const field32_t RCC_APB1HRSTR_FDCANRST;             /** @brief FDCAN block reset. */
extern const field32_t RCC_APB2RSTR_DFSDM1RST;             /** @brief DFSDM1 block reset. */
extern const field32_t RCC_APB2RSTR_HRTIMRST;              /** @brief HRTIM block reset. */
extern const field32_t RCC_APB4RSTR_SYSCFGRST;             /** @brief SYSCFG block reset. */
extern const field32_t RCC_APB4RSTR_LPUART1RST;            /** @brief LPUART1 block reset. */
extern const field32_t RCC_APB4RSTR_SPI6RST;               /** @brief SPI6 block reset. */
extern const field32_t RCC_APB4RSTR_I2C4RST;               /** @brief I2C4 block reset. */
extern const field32_t RCC_APB4RSTR_COMP12RST;             /** @brief COMP12 blocks reset. */
extern const field32_t RCC_APB4RSTR_VREFRST;               /** @brief VREF block reset. */
extern const field32_t RCC_APB4RSTR_SAI4RST;               /** @brief SAI4 block reset. */
extern const field32_t RCC_GCR_WW1RSC;                     /** @brief WWDG1 reset scope control. */
extern const field32_t RCC_D3AMR_BDMAAMEN;                 /** @brief BDMA and DMAMUX autonomous mode enable. */
extern const field32_t RCC_D3AMR_LPUART1AMEN;              /** @brief LPUART1 autonomous mode enable. */
extern const field32_t RCC_D3AMR_SPI6AMEN;                 /** @brief SPI6 autonomous mode enable. */
extern const field32_t RCC_D3AMR_I2C4AMEN;                 /** @brief I2C4 autonomous mode enable. */
extern const field32_t RCC_D3AMR_COMP12AMEN;               /** @brief COMP12 autonomous mode enable. */
extern const field32_t RCC_D3AMR_VREFAMEN;                 /** @brief VREF autonomous mode enable. */
extern const field32_t RCC_D3AMR_RTCAMEN;                  /** @brief RTC autonomous mode enable. */
extern const field32_t RCC_D3AMR_CRCAMEN;                  /** @brief CRC autonomous mode enable. */
extern const field32_t RCC_D3AMR_SAI4AMEN;                 /** @brief SAI4 autonomous mode enable. */
extern const field32_t RCC_D3AMR_ADC3AMEN;                 /** @brief ADC3 autonomous mode enable. */
extern const field32_t RCC_D3AMR_BKPSRAMAMEN;              /** @brief Backup RAM autonomous mode enable. */
extern const field32_t RCC_D3AMR_SRAM4AMEN;                /** @brief SRAM4 autonomous mode enable. */
extern const field32_t RCC_RSR_RMVF;                       /** @brief Remove reset flag. */
extern const field32_t RCC_RSR_CPURSTF;                    /** @brief CPU reset flag. */
extern const field32_t RCC_RSR_BORRSTF;                    /** @brief BOR reset flag. */
extern const field32_t RCC_RSR_PINRSTF;                    /** @brief Pin reset flag (NRST). */
extern const field32_t RCC_RSR_PORRSTF;                    /** @brief POR/PDR reset flag. */
extern const field32_t RCC_RSR_SFTRSTF;                    /** @brief System reset from CPU reset flag. */
extern const field32_t RCC_RSR_IWDG1RSTF;                  /** @brief Independent watchdog reset flag. */
extern const field32_t RCC_RSR_WWDG1RSTF;                  /** @brief Window watchdog reset flag. */
extern const field32_t RCC_RSR_LPWRRSTF;                   /** @brief Reset due to illegal D1 dstandby or CPU cstop flag. */
extern const field32_t RCC_C1_RSR_RMVF;                    /** @brief Remove reset flag. */
extern const field32_t RCC_C1_RSR_CPURSTF;                 /** @brief CPU reset flag. */
extern const field32_t RCC_C1_RSR_BORRSTF;                 /** @brief BOR reset flag. */
extern const field32_t RCC_C1_RSR_PINRSTF;                 /** @brief Pin reset flag (NRST). */
extern const field32_t RCC_C1_RSR_PORRSTF;                 /** @brief POR/PDR reset flag. */
extern const field32_t RCC_C1_RSR_SFTRSTF;                 /** @brief System reset from CPU reset flag. */
extern const field32_t RCC_C1_RSR_IWDG1RSTF;               /** @brief Independent watchdog reset flag. */
extern const field32_t RCC_C1_RSR_WWDG1RSTF;               /** @brief Window watchdog reset flag. */
extern const field32_t RCC_C1_RSR_LPWRRSTF;                /** @brief Reset due to illegal D1 dstandby or CPU cstop flag. */
extern const field32_t RCC_C1_AHB3ENR_MDMAEN;              /** @brief MDMA peripheral clock enable. */
extern const field32_t RCC_C1_AHB3ENR_DMA2DEN;             /** @brief DMA2D peripheral clock enable. */
extern const field32_t RCC_C1_AHB3ENR_JPGDECEN;            /** @brief JPGDEC peripheral clock enable. */
extern const field32_t RCC_C1_AHB3ENR_FMCEN;               /** @brief FMC peripheral clocks enable. */
extern const field32_t RCC_C1_AHB3ENR_QSPIEN;              /** @brief QUADSPI and QUADSPI delay clock enable. */
extern const field32_t RCC_C1_AHB3ENR_SDMMC1EN;            /** @brief SDMMC1 and SDMMC1 delay clock enable. */
extern const field32_t RCC_AHB3ENR_MDMAEN;                 /** @brief MDMA peripheral clock enable. */
extern const field32_t RCC_AHB3ENR_DMA2DEN;                /** @brief DMA2D peripheral clock enable. */
extern const field32_t RCC_AHB3ENR_JPGDECEN;               /** @brief JPGDEC peripheral clock enable. */
extern const field32_t RCC_AHB3ENR_FMCEN;                  /** @brief FMC peripheral clocks enable. */
extern const field32_t RCC_AHB3ENR_QSPIEN;                 /** @brief QUADSPI and QUADSPI delay clock enable. */
extern const field32_t RCC_AHB3ENR_SDMMC1EN;               /** @brief SDMMC1 and SDMMC1 delay clock enable. */
extern const field32_t RCC_AHB1ENR_ADC12EN;                /** @brief ADC1/2 peripheral clocks enable. */
extern const field32_t RCC_AHB1ENR_ETH1MACEN;              /** @brief Ethernet MAC bus interface clock enable. */
extern const field32_t RCC_AHB1ENR_ETH1TXEN;               /** @brief Ethernet transmission clock enable. */
extern const field32_t RCC_AHB1ENR_ETH1RXEN;               /** @brief Ethernet reception clock enable. */
extern const field32_t RCC_C1_AHB1ENR_ADC12EN;             /** @brief ADC1/2 peripheral clocks enable. */
extern const field32_t RCC_C1_AHB1ENR_ETH1MACEN;           /** @brief Ethernet MAC bus interface clock enable. */
extern const field32_t RCC_C1_AHB1ENR_ETH1TXEN;            /** @brief Ethernet transmission clock enable. */
extern const field32_t RCC_C1_AHB1ENR_ETH1RXEN;            /** @brief Ethernet reception clock enable. */
extern const field32_t RCC_C1_AHB2ENR_CAMITFEN;            /** @brief CAMITF peripheral clock enable. */
extern const field32_t RCC_C1_AHB2ENR_CRYPTEN;             /** @brief CRYPT peripheral clock enable. */
extern const field32_t RCC_C1_AHB2ENR_HASHEN;              /** @brief HASH peripheral clock enable. */
extern const field32_t RCC_C1_AHB2ENR_RNGEN;               /** @brief RNG peripheral clocks enable. */
extern const field32_t RCC_C1_AHB2ENR_SDMMC2EN;            /** @brief SDMMC2 and SDMMC2 delay clock enable. */
extern const field32_t RCC_AHB2ENR_CAMITFEN;               /** @brief CAMITF peripheral clock enable. */
extern const field32_t RCC_AHB2ENR_CRYPTEN;                /** @brief CRYPT peripheral clock enable. */
extern const field32_t RCC_AHB2ENR_HASHEN;                 /** @brief HASH peripheral clock enable. */
extern const field32_t RCC_AHB2ENR_RNGEN;                  /** @brief RNG peripheral clocks enable. */
extern const field32_t RCC_AHB2ENR_SDMMC2EN;               /** @brief SDMMC2 and SDMMC2 delay clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOAEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOBEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOCEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIODEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOEEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOFEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOGEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOHEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOIEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOJEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_GPIOKEN;                /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_CRCEN;                  /** @brief CRC peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_BDMAEN;                 /** @brief BDMA and DMAMUX2 clock enable. */
extern const field32_t RCC_AHB4ENR_ADC3EN;                 /** @brief ADC3 peripheral clocks enable. */
extern const field32_t RCC_AHB4ENR_HSEMEN;                 /** @brief HSEM peripheral clock enable. */
extern const field32_t RCC_AHB4ENR_BKPRAMEN;               /** @brief Backup RAM clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOAEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOBEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOCEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIODEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOEEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOFEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOGEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOHEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOIEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOJEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_GPIOKEN;             /** @brief 0GPIO peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_CRCEN;               /** @brief CRC peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_BDMAEN;              /** @brief BDMA and DMAMUX2 clock enable. */
extern const field32_t RCC_C1_AHB4ENR_ADC3EN;              /** @brief ADC3 peripheral clocks enable. */
extern const field32_t RCC_C1_AHB4ENR_HSEMEN;              /** @brief HSEM peripheral clock enable. */
extern const field32_t RCC_C1_AHB4ENR_BKPRAMEN;            /** @brief Backup RAM clock enable. */
extern const field32_t RCC_C1_APB3ENR_LTDCEN;              /** @brief LTDC peripheral clock enable. */
extern const field32_t RCC_C1_APB3ENR_WWDG1EN;             /** @brief WWDG1 clock enable. */
extern const field32_t RCC_APB3ENR_LTDCEN;                 /** @brief LTDC peripheral clock enable. */
extern const field32_t RCC_APB3ENR_WWDG1EN;                /** @brief WWDG1 clock enable. */
extern const field32_t RCC_APB1LENR_LPTIM1EN;              /** @brief LPTIM1 peripheral clocks enable. */
extern const field32_t RCC_APB1LENR_SPDIFRXEN;             /** @brief SPDIFRX peripheral clocks enable. */
extern const field32_t RCC_APB1LENR_HDMICECEN;             /** @brief HDMI-CEC peripheral clock enable. */
extern const field32_t RCC_APB1LENR_DAC12EN;               /** @brief DAC1&2 peripheral clock enable. */
extern const field32_t RCC_C1_APB1LENR_LPTIM1EN;           /** @brief LPTIM1 peripheral clocks enable. */
extern const field32_t RCC_C1_APB1LENR_SPDIFRXEN;          /** @brief SPDIFRX peripheral clocks enable. */
extern const field32_t RCC_C1_APB1LENR_HDMICECEN;          /** @brief HDMI-CEC peripheral clock enable. */
extern const field32_t RCC_C1_APB1LENR_DAC12EN;            /** @brief DAC1&2 peripheral clock enable. */
extern const field32_t RCC_APB1HENR_CRSEN;                 /** @brief Clock recovery system peripheral clock enable. */
extern const field32_t RCC_APB1HENR_SWPEN;                 /** @brief SWPMI peripheral clocks enable. */
extern const field32_t RCC_APB1HENR_OPAMPEN;               /** @brief OPAMP peripheral clock enable. */
extern const field32_t RCC_APB1HENR_MDIOSEN;               /** @brief MDIOS peripheral clock enable. */
extern const field32_t RCC_APB1HENR_FDCANEN;               /** @brief FDCAN peripheral clocks enable. */
extern const field32_t RCC_C1_APB1HENR_CRSEN;              /** @brief Clock recovery system peripheral clock enable. */
extern const field32_t RCC_C1_APB1HENR_SWPEN;              /** @brief SWPMI peripheral clocks enable. */
extern const field32_t RCC_C1_APB1HENR_OPAMPEN;            /** @brief OPAMP peripheral clock enable. */
extern const field32_t RCC_C1_APB1HENR_MDIOSEN;            /** @brief MDIOS peripheral clock enable. */
extern const field32_t RCC_C1_APB1HENR_FDCANEN;            /** @brief FDCAN peripheral clocks enable. */
extern const field32_t RCC_C1_APB2ENR_DFSDM1EN;            /** @brief DFSDM1 peripheral clocks enable. */
extern const field32_t RCC_C1_APB2ENR_HRTIMEN;             /** @brief HRTIM peripheral clock enable. */
extern const field32_t RCC_APB2ENR_DFSDM1EN;               /** @brief DFSDM1 peripheral clocks enable. */
extern const field32_t RCC_APB2ENR_HRTIMEN;                /** @brief HRTIM peripheral clock enable. */
extern const field32_t RCC_APB4ENR_SYSCFGEN;               /** @brief SYSCFG peripheral clock enable. */
extern const field32_t RCC_APB4ENR_LPUART1EN;              /** @brief LPUART1 peripheral clocks enable. */
extern const field32_t RCC_APB4ENR_SPI6EN;                 /** @brief SPI6 peripheral clocks enable. */
extern const field32_t RCC_APB4ENR_I2C4EN;                 /** @brief I2C4 peripheral clocks enable. */
extern const field32_t RCC_APB4ENR_COMP12EN;               /** @brief COMP1/2 peripheral clock enable. */
extern const field32_t RCC_APB4ENR_VREFEN;                 /** @brief VREF peripheral clock enable. */
extern const field32_t RCC_APB4ENR_RTCAPBEN;               /** @brief RTC APB clock enable. */
extern const field32_t RCC_APB4ENR_SAI4EN;                 /** @brief SAI4 peripheral clocks enable. */
extern const field32_t RCC_C1_APB4ENR_SYSCFGEN;            /** @brief SYSCFG peripheral clock enable. */
extern const field32_t RCC_C1_APB4ENR_LPUART1EN;           /** @brief LPUART1 peripheral clocks enable. */
extern const field32_t RCC_C1_APB4ENR_SPI6EN;              /** @brief SPI6 peripheral clocks enable. */
extern const field32_t RCC_C1_APB4ENR_I2C4EN;              /** @brief I2C4 peripheral clocks enable. */
extern const field32_t RCC_C1_APB4ENR_COMP12EN;            /** @brief COMP1/2 peripheral clock enable. */
extern const field32_t RCC_C1_APB4ENR_VREFEN;              /** @brief VREF peripheral clock enable. */
extern const field32_t RCC_C1_APB4ENR_RTCAPBEN;            /** @brief RTC APB clock enable. */
extern const field32_t RCC_C1_APB4ENR_SAI4EN;              /** @brief SAI4 peripheral clocks enable. */
extern const field32_t RCC_C1_AHB3LPENR_MDMALPEN;          /** @brief MDMA clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_DMA2DLPEN;         /** @brief DMA2D clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_JPGDECLPEN;        /** @brief JPGDEC clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_FLITFLPEN;         /** @brief FLITF clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_FMCLPEN;           /** @brief FMC peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_QSPILPEN;          /** @brief QUADSPI and QUADSPI delay clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_SDMMC1LPEN;        /** @brief SDMMC1 and SDMMC1 delay clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_D1DTCM1LPEN;       /** @brief D1DTCM1 block clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_DTCM2LPEN;         /** @brief D1 DTCM2 block clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_ITCMLPEN;          /** @brief D1ITCM block clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB3LPENR_AXISRAMLPEN;       /** @brief AXISRAM block clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_MDMALPEN;             /** @brief MDMA clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_DMA2DLPEN;            /** @brief DMA2D clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_JPGDECLPEN;           /** @brief JPGDEC clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_FLITFLPEN;            /** @brief FLITF clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_FMCLPEN;              /** @brief FMC peripheral clocks enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_QSPILPEN;             /** @brief QUADSPI and QUADSPI delay clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_SDMMC1LPEN;           /** @brief SDMMC1 and SDMMC1 delay clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_D1DTCM1LPEN;          /** @brief D1DTCM1 block clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_DTCM2LPEN;            /** @brief D1 DTCM2 block clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_ITCMLPEN;             /** @brief D1ITCM block clock enable during csleep mode. */
extern const field32_t RCC_AHB3LPENR_AXISRAMLPEN;          /** @brief AXISRAM block clock enable during csleep mode. */
extern const field32_t RCC_AHB1LPENR_ADC12LPEN;            /** @brief ADC1/2 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_AHB1LPENR_ETH1MACLPEN;          /** @brief Ethernet MAC bus interface clock enable during csleep mode. */
extern const field32_t RCC_AHB1LPENR_ETH1TXLPEN;           /** @brief Ethernet transmission clock enable during csleep mode. */
extern const field32_t RCC_AHB1LPENR_ETH1RXLPEN;           /** @brief Ethernet reception clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB1LPENR_ADC12LPEN;         /** @brief ADC1/2 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_AHB1LPENR_ETH1MACLPEN;       /** @brief Ethernet MAC bus interface clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB1LPENR_ETH1TXLPEN;        /** @brief Ethernet transmission clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB1LPENR_ETH1RXLPEN;        /** @brief Ethernet reception clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB2LPENR_CAMITFLPEN;        /** @brief CAMITF peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB2LPENR_CRYPTLPEN;         /** @brief CRYPT peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB2LPENR_HASHLPEN;          /** @brief HASH peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB2LPENR_SDMMC2LPEN;        /** @brief SDMMC2 and SDMMC2 delay clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB2LPENR_RNGLPEN;           /** @brief RNG peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB2LPENR_CAMITFLPEN;           /** @brief CAMITF peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB2LPENR_CRYPTLPEN;            /** @brief CRYPT peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB2LPENR_HASHLPEN;             /** @brief HASH peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB2LPENR_SDMMC2LPEN;           /** @brief SDMMC2 and SDMMC2 delay clock enable during csleep mode. */
extern const field32_t RCC_AHB2LPENR_RNGLPEN;              /** @brief RNG peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOALPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOBLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOCLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIODLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOELPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOFLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOGLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOHLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOILPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOJLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_GPIOKLPEN;            /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_CRCLPEN;              /** @brief CRC peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_BDMALPEN;             /** @brief BDMA clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_ADC3LPEN;             /** @brief ADC3 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_BKPRAMLPEN;           /** @brief Backup RAM clock enable during csleep mode. */
extern const field32_t RCC_AHB4LPENR_SRAM4LPEN;            /** @brief SRAM4 clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOALPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOBLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOCLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIODLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOELPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOFLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOGLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOHLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOILPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOJLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_GPIOKLPEN;         /** @brief GPIO peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_CRCLPEN;           /** @brief CRC peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_BDMALPEN;          /** @brief BDMA clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_ADC3LPEN;          /** @brief ADC3 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_BKPRAMLPEN;        /** @brief Backup RAM clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB4LPENR_SRAM4LPEN;         /** @brief SRAM4 clock enable during csleep mode. */
extern const field32_t RCC_C1_APB3LPENR_LTDCLPEN;          /** @brief LTDC peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB3LPENR_WWDG1LPEN;         /** @brief WWDG1 clock enable during csleep mode. */
extern const field32_t RCC_APB3LPENR_LTDCLPEN;             /** @brief LTDC peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB3LPENR_WWDG1LPEN;            /** @brief WWDG1 clock enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_LPTIM1LPEN;          /** @brief LPTIM1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_SPDIFRXLPEN;         /** @brief SPDIFRX peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_HDMICECLPEN;         /** @brief HDMI-CEC peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_DAC12LPEN;           /** @brief DAC1/2 peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB1LLPENR_LPTIM1LPEN;       /** @brief LPTIM1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB1LLPENR_SPDIFRXLPEN;      /** @brief SPDIFRX peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB1LLPENR_HDMICECLPEN;      /** @brief HDMI-CEC peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB1LLPENR_DAC12LPEN;        /** @brief DAC1/2 peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB1HLPENR_CRSLPEN;          /** @brief Clock recovery system peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB1HLPENR_SWPLPEN;          /** @brief SWPMI peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB1HLPENR_OPAMPLPEN;        /** @brief OPAMP peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB1HLPENR_MDIOSLPEN;        /** @brief MDIOS peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB1HLPENR_FDCANLPEN;        /** @brief FDCAN peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1HLPENR_CRSLPEN;             /** @brief Clock recovery system peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB1HLPENR_SWPLPEN;             /** @brief SWPMI peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1HLPENR_OPAMPLPEN;           /** @brief OPAMP peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB1HLPENR_MDIOSLPEN;           /** @brief MDIOS peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB1HLPENR_FDCANLPEN;           /** @brief FDCAN peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB2LPENR_DFSDM1LPEN;           /** @brief DFSDM1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB2LPENR_HRTIMLPEN;            /** @brief HRTIM peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB2LPENR_DFSDM1LPEN;        /** @brief DFSDM1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB2LPENR_HRTIMLPEN;         /** @brief HRTIM peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_SYSCFGLPEN;        /** @brief SYSCFG peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_LPUART1LPEN;       /** @brief LPUART1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_SPI6LPEN;          /** @brief SPI6 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_I2C4LPEN;          /** @brief I2C4 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_COMP12LPEN;        /** @brief COMP1/2 peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_VREFLPEN;          /** @brief VREF peripheral clock enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_RTCAPBLPEN;        /** @brief RTC APB clock enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_SAI4LPEN;          /** @brief SAI4 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_SYSCFGLPEN;           /** @brief SYSCFG peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_LPUART1LPEN;          /** @brief LPUART1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_SPI6LPEN;             /** @brief SPI6 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_I2C4LPEN;             /** @brief I2C4 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_COMP12LPEN;           /** @brief COMP1/2 peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_VREFLPEN;             /** @brief VREF peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_RTCAPBLPEN;           /** @brief RTC APB clock enable during csleep mode. */
extern const field32_t RCC_APB4LPENR_SAI4LPEN;             /** @brief SAI4 peripheral clocks enable during csleep mode. */

/** @subsection Enumerated RCC Register Field Definitions */

extern const field32_t RCC_CR_DxCKRDY[3];              /** @brief D1 domain clocks ready flag. */
extern const field32_t RCC_CR_PLLxON[4];               /** @brief PLL1 enable. */
extern const field32_t RCC_CR_PLLxRDY[4];              /** @brief PLL1 clock ready flag. */
extern const field32_t RCC_CFGR_MCOxPRE[3];            /** @brief MCO1 prescaler. */
extern const field32_t RCC_CFGR_MCOxSEL[3];            /** @brief Micro-controller clock output 1. */
extern const field32_t RCC_D2CFGR_D2PPREx[3];          /** @brief D2 domain APB1 prescaler. */
extern const field32_t RCC_PLLCKSELR_DIVMx[4];         /** @brief Prescaler for PLL1. */
extern const field32_t RCC_PLLCFGR_PLLxFRACEN[4];      /** @brief PLL1 fractional latch enable. */
extern const field32_t RCC_PLLCFGR_PLLxVCOSEL[4];      /** @brief PLL1 VCO selection. */
extern const field32_t RCC_PLLCFGR_PLLxRGE[4];         /** @brief PLL1 input frequency range. */
extern const field32_t RCC_PLLCFGR_DIVPxEN[4];         /** @brief PLL1 DIVP divider output enable. */
extern const field32_t RCC_PLLCFGR_DIVQxEN[4];         /** @brief PLL1 DIVQ divider output enable. */
extern const field32_t RCC_PLLCFGR_DIVRxEN[4];         /** @brief PLL1 DIVR divider output enable. */
extern const field32_t RCC_CIER_PLLxRDYIE[4];          /** @brief PLL1 ready interrupt enable. */
extern const field32_t RCC_CIFR_PLLxRDYF[4];           /** @brief PLL1 ready interrupt flag. */
extern const field32_t RCC_CICR_PLLxRDYC[4];           /** @brief PLL1 ready interrupt clear. */
extern const field32_t RCC_AHB1RSTR_DMAxRST[3];        /** @brief DMA1 block reset. */
extern const field32_t RCC_AHB1RSTR_USBxOTGRST[3];     /** @brief USB1OTG block reset. */
extern const field32_t RCC_APB1LRSTR_TIMxRST[15];      /** @brief TIM block reset. */
extern const field32_t RCC_APB1LRSTR_SPIxRST[4];       /** @brief SPI2 block reset. */
extern const field32_t RCC_APB1LRSTR_USARTxRST[9];     /** @brief USART2 block reset. */
extern const field32_t RCC_APB1LRSTR_UARTxRST[6];      /** @brief UART4 block reset. */
extern const field32_t RCC_APB1LRSTR_I2CxRST[4];       /** @brief I2C1 block reset. */
extern const field32_t RCC_APB2RSTR_SAIxRST[4];        /** @brief SAI1 block reset. */
extern const field32_t RCC_APB4RSTR_LPTIMxRST[6];      /** @brief LPTIM2 block reset. */
extern const field32_t RCC_D3AMR_LPTIMxAMEN[6];        /** @brief LPTIM2 autonomous mode enable. */
extern const field32_t RCC_RSR_DxRSTF[3];              /** @brief D1 domain power switch reset flag. */
extern const field32_t RCC_AHB1ENR_DMAxEN[3];          /** @brief DMA1 clock enable. */
extern const field32_t RCC_AHB1ENR_USBxOTGEN[3];       /** @brief USB1OTG peripheral clocks enable. */
extern const field32_t RCC_AHB1ENR_USBxULPIEN[3];      /** @brief USB_PHY1 clocks enable. */
extern const field32_t RCC_C1_AHB2ENR_SRAMxEN[4];      /** @brief SRAM1 block enable. */
extern const field32_t RCC_APB1LENR_TIMxEN[15];        /** @brief TIM peripheral clock enable. */
extern const field32_t RCC_APB1LENR_SPIxEN[4];         /** @brief SPI2 peripheral clocks enable. */
extern const field32_t RCC_APB1LENR_USARTxEN[9];       /** @brief USART2 peripheral clocks enable. */
extern const field32_t RCC_APB1LENR_UARTxEN[6];        /** @brief UART4 peripheral clocks enable. */
extern const field32_t RCC_APB1LENR_I2CxEN[4];         /** @brief I2C1 peripheral clocks enable. */
extern const field32_t RCC_C1_APB2ENR_SAIxEN[4];       /** @brief SAI1 peripheral clocks enable. */
extern const field32_t RCC_APB4ENR_LPTIMxEN[6];        /** @brief LPTIM2 peripheral clocks enable. */
extern const field32_t RCC_AHB1LPENR_DMAxLPEN[3];      /** @brief DMA1 clock enable during csleep mode. */
extern const field32_t RCC_AHB1LPENR_USBxOTGLPEN[3];   /** @brief USB1OTG peripheral clock enable during csleep mode. */
extern const field32_t RCC_AHB1LPENR_USBxULPILPEN[3];  /** @brief USB_PHY1 clock enable during csleep mode. */
extern const field32_t RCC_C1_AHB2LPENR_SRAMxLPEN[4];  /** @brief SRAM1 clock enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_TIMxLPEN[15];    /** @brief TIM2 peripheral clock enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_SPIxLPEN[4];     /** @brief SPI2 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_USARTxLPEN[9];   /** @brief USART2 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_UARTxLPEN[6];    /** @brief UART4 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB1LLPENR_I2CxLPEN[4];     /** @brief I2C1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_APB2LPENR_SAIxLPEN[4];      /** @brief SAI1 peripheral clocks enable during csleep mode. */
extern const field32_t RCC_C1_APB4LPENR_LPTIMxLPEN[6]; /** @brief LPTIM2 peripheral clocks enable during csleep mode. */

/**************************************************************************************************
 * @section LPTIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated LPTIMx Register Definitions */

extern ro_reg32_t const LPTIMx_ISR[6];   /** @brief Interrupt and status register. */
extern rw_reg32_t const LPTIMx_ICR[6];   /** @brief Interrupt clear register. */
extern rw_reg32_t const LPTIMx_IER[6];   /** @brief Interrupt enable register. */
extern rw_reg32_t const LPTIMx_CFGR[6];  /** @brief Configuration register. */
extern rw_reg32_t const LPTIMx_CR[6];    /** @brief Control register. */
extern rw_reg32_t const LPTIMx_CMP[6];   /** @brief Compare register. */
extern rw_reg32_t const LPTIMx_ARR[6];   /** @brief Autoreload register. */
extern ro_reg32_t const LPTIMx_CNT[6];   /** @brief Counter register. */
extern rw_reg32_t const LPTIMx_CFGR2[6]; /** @brief LPTIM configuration register 2. */

/** @subsection LPTIMx Register Field Definitions */

extern const field32_t LPTIMx_ISR_DOWN;       /** @brief Counter direction change up to down. */
extern const field32_t LPTIMx_ISR_UP;         /** @brief Counter direction change down to up. */
extern const field32_t LPTIMx_ISR_ARROK;      /** @brief Autoreload register update OK. */
extern const field32_t LPTIMx_ISR_CMPOK;      /** @brief Compare register update OK. */
extern const field32_t LPTIMx_ISR_EXTTRIG;    /** @brief External trigger edge event. */
extern const field32_t LPTIMx_ISR_ARRM;       /** @brief Autoreload match. */
extern const field32_t LPTIMx_ISR_CMPM;       /** @brief Compare match. */
extern const field32_t LPTIMx_ICR_DOWNCF;     /** @brief Direction change to down clear flag. */
extern const field32_t LPTIMx_ICR_UPCF;       /** @brief Direction change to UP clear flag. */
extern const field32_t LPTIMx_ICR_ARROKCF;    /** @brief Autoreload register update OK clear flag. */
extern const field32_t LPTIMx_ICR_CMPOKCF;    /** @brief Compare register update OK clear flag. */
extern const field32_t LPTIMx_ICR_EXTTRIGCF;  /** @brief External trigger valid edge clear flag. */
extern const field32_t LPTIMx_ICR_ARRMCF;     /** @brief Autoreload match clear flag. */
extern const field32_t LPTIMx_ICR_CMPMCF;     /** @brief Compare match clear flag. */
extern const field32_t LPTIMx_IER_DOWNIE;     /** @brief Direction change to down interrupt enable. */
extern const field32_t LPTIMx_IER_UPIE;       /** @brief Direction change to UP interrupt enable. */
extern const field32_t LPTIMx_IER_ARROKIE;    /** @brief Autoreload register update OK interrupt enable. */
extern const field32_t LPTIMx_IER_CMPOKIE;    /** @brief Compare register update OK interrupt enable. */
extern const field32_t LPTIMx_IER_EXTTRIGIE;  /** @brief External trigger valid edge interrupt enable. */
extern const field32_t LPTIMx_IER_ARRMIE;     /** @brief Autoreload match interrupt enable. */
extern const field32_t LPTIMx_IER_CMPMIE;     /** @brief Compare match interrupt enable. */
extern const field32_t LPTIMx_CFGR_ENC;       /** @brief Encoder mode enable. */
extern const field32_t LPTIMx_CFGR_COUNTMODE; /** @brief Counter mode enabled. */
extern const field32_t LPTIMx_CFGR_PRELOAD;   /** @brief Registers update mode. */
extern const field32_t LPTIMx_CFGR_WAVPOL;    /** @brief Waveform shape polarity. */
extern const field32_t LPTIMx_CFGR_WAVE;      /** @brief Waveform shape. */
extern const field32_t LPTIMx_CFGR_TIMOUT;    /** @brief Timeout enable. */
extern const field32_t LPTIMx_CFGR_TRIGEN;    /** @brief Trigger enable and polarity. */
extern const field32_t LPTIMx_CFGR_TRIGSEL;   /** @brief Trigger selector. */
extern const field32_t LPTIMx_CFGR_PRESC;     /** @brief Clock prescaler. */
extern const field32_t LPTIMx_CFGR_TRGFLT;    /** @brief Configurable digital filter for trigger. */
extern const field32_t LPTIMx_CFGR_CKFLT;     /** @brief Configurable digital filter for external clock. */
extern const field32_t LPTIMx_CFGR_CKPOL;     /** @brief Clock polarity. */
extern const field32_t LPTIMx_CFGR_CKSEL;     /** @brief Clock selector. */
extern const field32_t LPTIMx_CR_ENABLE;      /** @brief LPTIM enable. */
extern const field32_t LPTIMx_CR_SNGSTRT;     /** @brief LPTIM start in single mode. */
extern const field32_t LPTIMx_CR_CNTSTRT;     /** @brief Timer start in continuous mode. */
extern const field32_t LPTIMx_CR_COUNTRST;    /** @brief Counter reset. */
extern const field32_t LPTIMx_CR_RSTARE;      /** @brief Reset after read enable. */
extern const field32_t LPTIMx_CMP_CMP;        /** @brief Compare value. */
extern const field32_t LPTIMx_ARR_ARR;        /** @brief Auto reload value. */
extern const field32_t LPTIMx_CNT_CNT;        /** @brief Counter value. */

/** @subsection Enumerated LPTIMx Register Field Definitions */

extern const field32_t LPTIMx_CFGR2_INxSEL[3]; /** @brief LPTIM input 1 selection. */

/**************************************************************************************************
 * @section LPUART1 Definitions
 **************************************************************************************************/

/** @subsection LPUART1 Register Definitions */

extern rw_reg32_t const LPUART1_CR1;   /** @brief Control register 1. */
extern rw_reg32_t const LPUART1_CR2;   /** @brief Control register 2. */
extern rw_reg32_t const LPUART1_CR3;   /** @brief Control register 3. */
extern rw_reg32_t const LPUART1_BRR;   /** @brief Baud rate register. */
extern rw_reg32_t const LPUART1_GTPR;  /** @brief Guard time and prescaler register. */
extern rw_reg32_t const LPUART1_RTOR;  /** @brief Receiver timeout register. */
extern rw_reg32_t const LPUART1_RQR;   /** @brief Request register. */
extern ro_reg32_t const LPUART1_ISR;   /** @brief Interrupt & status register. */
extern rw_reg32_t const LPUART1_ICR;   /** @brief Interrupt flag clear register. */
extern ro_reg32_t const LPUART1_RDR;   /** @brief Receive data register. */
extern rw_reg32_t const LPUART1_TDR;   /** @brief Transmit data register. */
extern rw_reg32_t const LPUART1_PRESC; /** @brief Prescaler register. */

/** @subsection LPUART1 Register Field Definitions */

extern const field32_t LPUART1_CR1_RXFFIE;      /** @brief RXFIFO full interrupt enable. */
extern const field32_t LPUART1_CR1_TXFEIE;      /** @brief TXFIFO empty interrupt enable. */
extern const field32_t LPUART1_CR1_FIFOEN;      /** @brief FIFO mode enable. */
extern const field32_t LPUART1_CR1_DEAT;        /** @brief Driver enable assertion time. */
extern const field32_t LPUART1_CR1_DEDT;        /** @brief Driver enable deassertion time. */
extern const field32_t LPUART1_CR1_CMIE;        /** @brief Character match interrupt enable. */
extern const field32_t LPUART1_CR1_MME;         /** @brief Mute mode enable. */
extern const field32_t LPUART1_CR1_WAKE;        /** @brief Receiver wakeup method. */
extern const field32_t LPUART1_CR1_PCE;         /** @brief Parity control enable. */
extern const field32_t LPUART1_CR1_PS;          /** @brief Parity selection. */
extern const field32_t LPUART1_CR1_PEIE;        /** @brief PE interrupt enable. */
extern const field32_t LPUART1_CR1_TXEIE;       /** @brief Interrupt enable. */
extern const field32_t LPUART1_CR1_TCIE;        /** @brief Transmission complete interrupt enable. */
extern const field32_t LPUART1_CR1_RXNEIE;      /** @brief RXNE interrupt enable. */
extern const field32_t LPUART1_CR1_IDLEIE;      /** @brief IDLE interrupt enable. */
extern const field32_t LPUART1_CR1_TE;          /** @brief Transmitter enable. */
extern const field32_t LPUART1_CR1_RE;          /** @brief Receiver enable. */
extern const field32_t LPUART1_CR1_UESM;        /** @brief USART enable in stop mode. */
extern const field32_t LPUART1_CR1_UE;          /** @brief USART enable. */
extern const field32_t LPUART1_CR2_ADD;         /** @brief Address of the USART node. */
extern const field32_t LPUART1_CR2_MSBFIRST;    /** @brief Most significant bit first. */
extern const field32_t LPUART1_CR2_DATAINV;     /** @brief Binary data inversion. */
extern const field32_t LPUART1_CR2_TXINV;       /** @brief TX pin active level inversion. */
extern const field32_t LPUART1_CR2_RXINV;       /** @brief RX pin active level inversion. */
extern const field32_t LPUART1_CR2_SWAP;        /** @brief Swap TX/RX pins. */
extern const field32_t LPUART1_CR2_STOP;        /** @brief STOP bits. */
extern const field32_t LPUART1_CR2_ADDM7;       /** @brief 7-bit address detection/4-bit address detection. */
extern const field32_t LPUART1_CR3_TXFTCFG;     /** @brief TXFIFO threshold configuration. */
extern const field32_t LPUART1_CR3_RXFTIE;      /** @brief RXFIFO threshold interrupt enable. */
extern const field32_t LPUART1_CR3_RXFTCFG;     /** @brief Receive FIFO threshold configuration. */
extern const field32_t LPUART1_CR3_TXFTIE;      /** @brief TXFIFO threshold interrupt enable. */
extern const field32_t LPUART1_CR3_WUFIE;       /** @brief Wakeup from stop mode interrupt enable. */
extern const field32_t LPUART1_CR3_WUS;         /** @brief Wakeup from stop mode interrupt flag selection. */
extern const field32_t LPUART1_CR3_DEP;         /** @brief Driver enable polarity selection. */
extern const field32_t LPUART1_CR3_DEM;         /** @brief Driver enable mode. */
extern const field32_t LPUART1_CR3_DDRE;        /** @brief DMA disable on reception error. */
extern const field32_t LPUART1_CR3_OVRDIS;      /** @brief Overrun disable. */
extern const field32_t LPUART1_CR3_CTSIE;       /** @brief CTS interrupt enable. */
extern const field32_t LPUART1_CR3_CTSE;        /** @brief CTS enable. */
extern const field32_t LPUART1_CR3_RTSE;        /** @brief RTS enable. */
extern const field32_t LPUART1_CR3_DMAT;        /** @brief DMA enable transmitter. */
extern const field32_t LPUART1_CR3_DMAR;        /** @brief DMA enable receiver. */
extern const field32_t LPUART1_CR3_HDSEL;       /** @brief Half-duplex selection. */
extern const field32_t LPUART1_CR3_EIE;         /** @brief Error interrupt enable. */
extern const field32_t LPUART1_BRR_BRR;         /** @brief BRR. */
extern const field32_t LPUART1_GTPR_GT;         /** @brief Guard time value. */
extern const field32_t LPUART1_GTPR_PSC;        /** @brief Prescaler value. */
extern const field32_t LPUART1_RTOR_BLEN;       /** @brief Block length. */
extern const field32_t LPUART1_RTOR_RTO;        /** @brief Receiver timeout value. */
extern const field32_t LPUART1_RQR_TXFRQ;       /** @brief Transmit data flush request. */
extern const field32_t LPUART1_RQR_RXFRQ;       /** @brief Receive data flush request. */
extern const field32_t LPUART1_RQR_MMRQ;        /** @brief Mute mode request. */
extern const field32_t LPUART1_RQR_SBKRQ;       /** @brief Send break request. */
extern const field32_t LPUART1_RQR_ABRRQ;       /** @brief Auto baud rate request. */
extern const field32_t LPUART1_ISR_TXFT;        /** @brief TXFIFO threshold flag. */
extern const field32_t LPUART1_ISR_RXFT;        /** @brief RXFIFO threshold flag. */
extern const field32_t LPUART1_ISR_RXFF;        /** @brief RXFIFO full. */
extern const field32_t LPUART1_ISR_TXFE;        /** @brief TXFIFO empty. */
extern const field32_t LPUART1_ISR_REACK;       /** @brief REACK. */
extern const field32_t LPUART1_ISR_TEACK;       /** @brief TEACK. */
extern const field32_t LPUART1_ISR_WUF;         /** @brief WUF. */
extern const field32_t LPUART1_ISR_RWU;         /** @brief RWU. */
extern const field32_t LPUART1_ISR_SBKF;        /** @brief SBKF. */
extern const field32_t LPUART1_ISR_CMF;         /** @brief CMF. */
extern const field32_t LPUART1_ISR_BUSY;        /** @brief BUSY. */
extern const field32_t LPUART1_ISR_CTS;         /** @brief CTS. */
extern const field32_t LPUART1_ISR_CTSIF;       /** @brief CTSIF. */
extern const field32_t LPUART1_ISR_TXE;         /** @brief TXE. */
extern const field32_t LPUART1_ISR_TC;          /** @brief TC. */
extern const field32_t LPUART1_ISR_RXNE;        /** @brief RXNE. */
extern const field32_t LPUART1_ISR_IDLE;        /** @brief IDLE. */
extern const field32_t LPUART1_ISR_ORE;         /** @brief ORE. */
extern const field32_t LPUART1_ISR_NE;          /** @brief NE. */
extern const field32_t LPUART1_ISR_FE;          /** @brief FE. */
extern const field32_t LPUART1_ISR_PE;          /** @brief PE. */
extern const field32_t LPUART1_ICR_WUCF;        /** @brief Wakeup from stop mode clear flag. */
extern const field32_t LPUART1_ICR_CMCF;        /** @brief Character match clear flag. */
extern const field32_t LPUART1_ICR_CTSCF;       /** @brief CTS clear flag. */
extern const field32_t LPUART1_ICR_TCCF;        /** @brief Transmission complete clear flag. */
extern const field32_t LPUART1_ICR_IDLECF;      /** @brief Idle line detected clear flag. */
extern const field32_t LPUART1_ICR_ORECF;       /** @brief Overrun error clear flag. */
extern const field32_t LPUART1_ICR_NCF;         /** @brief Noise detected clear flag. */
extern const field32_t LPUART1_ICR_FECF;        /** @brief Framing error clear flag. */
extern const field32_t LPUART1_ICR_PECF;        /** @brief Parity error clear flag. */
extern const field32_t LPUART1_RDR_RDR;         /** @brief Receive data value. */
extern const field32_t LPUART1_TDR_TDR;         /** @brief Transmit data value. */
extern const field32_t LPUART1_PRESC_PRESCALER; /** @brief Clock prescaler. */

/** @subsection Enumerated LPUART1 Register Field Definitions */

extern const field32_t LPUART1_CR1_Mx[2]; /** @brief Word length. */

/**************************************************************************************************
 * @section SYSCFG Definitions
 **************************************************************************************************/

/** @subsection SYSCFG Register Definitions */

extern rw_reg32_t const SYSCFG_PMCR;    /** @brief Peripheral mode configuration register. */
extern rw_reg32_t const SYSCFG_EXTICR1; /** @brief External interrupt configuration register 1. */
extern rw_reg32_t const SYSCFG_EXTICR2; /** @brief External interrupt configuration register 2. */
extern rw_reg32_t const SYSCFG_EXTICR3; /** @brief External interrupt configuration register 3. */
extern rw_reg32_t const SYSCFG_EXTICR4; /** @brief External interrupt configuration register 4. */
extern rw_reg32_t const SYSCFG_CCCSR;   /** @brief Compensation cell control/status register. */
extern ro_reg32_t const SYSCFG_CCVR;    /** @brief SYSCFG compensation cell value register. */
extern rw_reg32_t const SYSCFG_CCCR;    /** @brief SYSCFG compensation cell code register. */
extern ro_reg32_t const SYSCFG_PKGR;    /** @brief SYSCFG package register. */
extern ro_reg32_t const SYSCFG_UR0;     /** @brief SYSCFG user register 0. */
extern rw_reg32_t const SYSCFG_UR2;     /** @brief SYSCFG user register 2. */
extern rw_reg32_t const SYSCFG_UR3;     /** @brief SYSCFG user register 3. */
extern ro_reg32_t const SYSCFG_UR4;     /** @brief SYSCFG user register 4. */
extern ro_reg32_t const SYSCFG_UR5;     /** @brief SYSCFG user register 5. */
extern ro_reg32_t const SYSCFG_UR6;     /** @brief SYSCFG user register 6. */
extern ro_reg32_t const SYSCFG_UR7;     /** @brief SYSCFG user register 7. */
extern ro_reg32_t const SYSCFG_UR8;     /** @brief SYSCFG user register 8. */
extern ro_reg32_t const SYSCFG_UR9;     /** @brief SYSCFG user register 9. */
extern ro_reg32_t const SYSCFG_UR10;    /** @brief SYSCFG user register 10. */
extern ro_reg32_t const SYSCFG_UR11;    /** @brief SYSCFG user register 11. */
extern ro_reg32_t const SYSCFG_UR12;    /** @brief SYSCFG user register 12. */
extern ro_reg32_t const SYSCFG_UR13;    /** @brief SYSCFG user register 13. */
extern rw_reg32_t const SYSCFG_UR14;    /** @brief SYSCFG user register 14. */
extern ro_reg32_t const SYSCFG_UR15;    /** @brief SYSCFG user register 15. */
extern ro_reg32_t const SYSCFG_UR16;    /** @brief SYSCFG user register 16. */
extern ro_reg32_t const SYSCFG_UR17;    /** @brief SYSCFG user register 17. */

/** @subsection SYSCFG Register Field Definitions */

extern const field32_t SYSCFG_PMCR_BOOSTE;    /** @brief Booster enable. */
extern const field32_t SYSCFG_PMCR_EPIS;      /** @brief Ethernet PHY interface selection. */
extern const field32_t SYSCFG_CCCSR_EN;       /** @brief Enable. */
extern const field32_t SYSCFG_CCCSR_CS;       /** @brief Code selection. */
extern const field32_t SYSCFG_CCCSR_READY;    /** @brief Compensation cell ready flag. */
extern const field32_t SYSCFG_CCCSR_HSLV;     /** @brief High-speed at low-voltage. */
extern const field32_t SYSCFG_CCVR_NCV;       /** @brief NMOS compensation value. */
extern const field32_t SYSCFG_CCVR_PCV;       /** @brief PMOS compensation value. */
extern const field32_t SYSCFG_CCCR_NCC;       /** @brief NMOS compensation code. */
extern const field32_t SYSCFG_CCCR_PCC;       /** @brief PMOS compensation code. */
extern const field32_t SYSCFG_PKGR_PKG;       /** @brief Package. */
extern const field32_t SYSCFG_UR0_BKS;        /** @brief Bank swap. */
extern const field32_t SYSCFG_UR0_RDP;        /** @brief Readout protection. */
extern const field32_t SYSCFG_UR2_BORH;       /** @brief BOR_LVL brownout reset threshold level. */
extern const field32_t SYSCFG_UR2_BOOT_ADD0;  /** @brief Boot address 0. */
extern const field32_t SYSCFG_UR3_BOOT_ADD1;  /** @brief Boot address 1. */
extern const field32_t SYSCFG_UR4_MEPAD_1;    /** @brief Mass erase protected area disabled for bank 1. */
extern const field32_t SYSCFG_UR5_MESAD_1;    /** @brief Mass erase secured area disabled for bank 1. */
extern const field32_t SYSCFG_UR5_WRPN_1;     /** @brief Write protection for flash bank 1. */
extern const field32_t SYSCFG_UR6_PA_BEG_1;   /** @brief Protected area start address for bank 1. */
extern const field32_t SYSCFG_UR6_PA_END_1;   /** @brief Protected area end address for bank 1. */
extern const field32_t SYSCFG_UR7_SA_BEG_1;   /** @brief Secured area start address for bank 1. */
extern const field32_t SYSCFG_UR7_SA_END_1;   /** @brief Secured area end address for bank 1. */
extern const field32_t SYSCFG_UR8_MEPAD_2;    /** @brief Mass erase protected area disabled for bank 2. */
extern const field32_t SYSCFG_UR8_MESAD_2;    /** @brief Mass erase secured area disabled for bank 2. */
extern const field32_t SYSCFG_UR9_WRPN_2;     /** @brief Write protection for flash bank 2. */
extern const field32_t SYSCFG_UR9_PA_BEG_2;   /** @brief Protected area start address for bank 2. */
extern const field32_t SYSCFG_UR10_PA_END_2;  /** @brief Protected area end address for bank 2. */
extern const field32_t SYSCFG_UR10_SA_BEG_2;  /** @brief Secured area start address for bank 2. */
extern const field32_t SYSCFG_UR11_SA_END_2;  /** @brief Secured area end address for bank 2. */
extern const field32_t SYSCFG_UR11_IWDG1M;    /** @brief Independent watchdog 1 mode. */
extern const field32_t SYSCFG_UR12_SECURE;    /** @brief Secure mode. */
extern const field32_t SYSCFG_UR13_SDRS;      /** @brief Secured DTCM RAM size. */
extern const field32_t SYSCFG_UR13_D1SBRST;   /** @brief D1 standby reset. */
extern const field32_t SYSCFG_UR14_D1STPRST;  /** @brief D1 stop reset. */
extern const field32_t SYSCFG_UR15_FZIWDGSTB; /** @brief Freeze independent watchdog in standby mode. */
extern const field32_t SYSCFG_UR16_FZIWDGSTP; /** @brief Freeze independent watchdog in stop mode. */
extern const field32_t SYSCFG_UR16_PKP;       /** @brief Private key programmed. */
extern const field32_t SYSCFG_UR17_IO_HSLV;   /** @brief I/O high speed / low voltage. */

/** @subsection Enumerated SYSCFG Register Field Definitions */

extern const field32_t SYSCFG_PMCR_I2CxFMP[5];   /** @brief I2C1 fm+. */
extern const field32_t SYSCFG_PMCR_PBxFMP[10];   /** @brief PB(6) fm+. */
extern const field32_t SYSCFG_PMCR_PAxSO[2];     /** @brief PA0 switch open. */
extern const field32_t SYSCFG_PMCR_PCxSO[4];     /** @brief PC2 switch open. */
extern const field32_t SYSCFG_EXTICR1_EXTIx[4];  /** @brief EXTI x configuration (x = 0 to 3). */
extern const field32_t SYSCFG_EXTICR3_EXTI1x[2]; /** @brief EXTI10. */

/**************************************************************************************************
 * @section EXTI Definitions
 **************************************************************************************************/

/** @subsection EXTI Register Definitions */

extern rw_reg32_t const EXTI_RTSR1;   /** @brief EXTI rising trigger selection register. */
extern rw_reg32_t const EXTI_FTSR1;   /** @brief EXTI falling trigger selection register. */
extern rw_reg32_t const EXTI_SWIER1;  /** @brief EXTI software interrupt event register. */
extern rw_reg32_t const EXTI_D3PMR1;  /** @brief EXTI D3 pending mask register. */
extern rw_reg32_t const EXTI_D3PCR1L; /** @brief EXTI D3 pending clear selection register low. */
extern rw_reg32_t const EXTI_D3PCR1H; /** @brief EXTI D3 pending clear selection register high. */
extern rw_reg32_t const EXTI_RTSR2;   /** @brief EXTI rising trigger selection register. */
extern rw_reg32_t const EXTI_FTSR2;   /** @brief EXTI falling trigger selection register. */
extern rw_reg32_t const EXTI_SWIER2;  /** @brief EXTI software interrupt event register. */
extern rw_reg32_t const EXTI_D3PMR2;  /** @brief EXTI D3 pending mask register. */
extern rw_reg32_t const EXTI_D3PCR2L; /** @brief EXTI D3 pending clear selection register low. */
extern rw_reg32_t const EXTI_D3PCR2H; /** @brief EXTI D3 pending clear selection register high. */
extern rw_reg32_t const EXTI_RTSR3;   /** @brief EXTI rising trigger selection register. */
extern rw_reg32_t const EXTI_FTSR3;   /** @brief EXTI falling trigger selection register. */
extern rw_reg32_t const EXTI_SWIER3;  /** @brief EXTI software interrupt event register. */
extern rw_reg32_t const EXTI_D3PMR3;  /** @brief EXTI D3 pending mask register. */
extern rw_reg32_t const EXTI_D3PCR3H; /** @brief EXTI D3 pending clear selection register high. */
extern rw_reg32_t const EXTI_CPUIMR1; /** @brief EXTI interrupt mask register. */
extern rw_reg32_t const EXTI_CPUEMR1; /** @brief EXTI event mask register. */
extern rw_reg32_t const EXTI_CPUPR1;  /** @brief EXTI pending register. */
extern rw_reg32_t const EXTI_CPUIMR2; /** @brief EXTI interrupt mask register. */
extern rw_reg32_t const EXTI_CPUEMR2; /** @brief EXTI event mask register. */
extern ro_reg32_t const EXTI_CPUPR2;  /** @brief EXTI pending register. */
extern ro_reg32_t const EXTI_CPUIMR3; /** @brief EXTI interrupt mask register. */
extern ro_reg32_t const EXTI_CPUEMR3; /** @brief EXTI event mask register. */
extern ro_reg32_t const EXTI_CPUPR3;  /** @brief EXTI pending register. */

/** @subsection EXTI Register Field Definitions */

extern const field32_t EXTI_D3PMR3_MR88;   /** @brief D3 pending mask on event input x+64. */
extern const field32_t EXTI_D3PCR3H_PCS88; /** @brief D3 pending request clear input signal selection on event input x= truncate N+160/2. */

/** @subsection Enumerated EXTI Register Field Definitions */

extern const field32_t EXTI_RTSR1_TRx[22];     /** @brief Rising trigger event configuration bit of configurable event input. */
extern const field32_t EXTI_SWIER1_SWIERx[22]; /** @brief Rising trigger event configuration bit of configurable event input. */
extern const field32_t EXTI_D3PMR1_MRx[26];    /** @brief Rising trigger event configuration bit of configurable event input. */
extern const field32_t EXTI_D3PCR1L_PCSx[16];  /** @brief D3 pending request clear input signal selection on event input x = truncate (n/2). */
extern const field32_t EXTI_D3PMR2_MR3x[6];    /** @brief D3 pending mask on event input x+32. */
extern const field32_t EXTI_D3PCR2L_PCS3x[6];  /** @brief D3 pending request clear input signal selection on event input x = truncate ((n+64)/2). */
extern const field32_t EXTI_D3PCR2H_PCS4x[10]; /** @brief Pending request clear input signal selection on event input x= truncate ((n+96)/2). */
extern const field32_t EXTI_RTSR3_TR8x[7];     /** @brief Rising trigger event configuration bit of configurable event input x+64. */
extern const field32_t EXTI_SWIER3_SWIER8x[7]; /** @brief Software interrupt on line x+64. */
extern const field32_t EXTI_CPUPR1_PRx[22];    /** @brief CPU event mask on event input x. */
extern const field32_t EXTI_CPUIMR3_MR6x[10];  /** @brief CPU interrupt mask on direct event input x+64. */
extern const field32_t EXTI_CPUPR3_PR8x[7];    /** @brief Configurable event inputs x+64 pending bit. */

/**************************************************************************************************
 * @section DELAY_BLOCK_SDMMCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated DELAY_BLOCK_SDMMCx Register Definitions */

extern rw_reg32_t const DELAY_BLOCK_SDMMCx_CR[3];   /** @brief DLYB control register. */
extern rw_reg32_t const DELAY_BLOCK_SDMMCx_CFGR[3]; /** @brief DLYB configuration register. */

/** @subsection DELAY_BLOCK_SDMMCx Register Field Definitions */

extern const field32_t DELAY_BLOCK_SDMMCx_CR_DEN;    /** @brief Delay block enable bit. */
extern const field32_t DELAY_BLOCK_SDMMCx_CR_SEN;    /** @brief Sampler length enable bit. */
extern const field32_t DELAY_BLOCK_SDMMCx_CFGR_SEL;  /** @brief Select the phase for the output clock. */
extern const field32_t DELAY_BLOCK_SDMMCx_CFGR_UNIT; /** @brief Delay defines the delay of a unit delay cell. */
extern const field32_t DELAY_BLOCK_SDMMCx_CFGR_LNG;  /** @brief Delay line length value. */
extern const field32_t DELAY_BLOCK_SDMMCx_CFGR_LNGF; /** @brief Length valid flag. */

/**************************************************************************************************
 * @section DELAY_BLOCK_QUADSPI Definitions
 **************************************************************************************************/

/** @subsection DELAY_BLOCK_QUADSPI Register Definitions */

extern rw_reg32_t const DELAY_BLOCK_QUADSPI_CR;   /** @brief DLYB control register. */
extern rw_reg32_t const DELAY_BLOCK_QUADSPI_CFGR; /** @brief DLYB configuration register. */

/** @subsection DELAY_BLOCK_QUADSPI Register Field Definitions */

extern const field32_t DELAY_BLOCK_QUADSPI_CR_DEN;    /** @brief Delay block enable bit. */
extern const field32_t DELAY_BLOCK_QUADSPI_CR_SEN;    /** @brief Sampler length enable bit. */
extern const field32_t DELAY_BLOCK_QUADSPI_CFGR_SEL;  /** @brief Select the phase for the output clock. */
extern const field32_t DELAY_BLOCK_QUADSPI_CFGR_UNIT; /** @brief Delay defines the delay of a unit delay cell. */
extern const field32_t DELAY_BLOCK_QUADSPI_CFGR_LNG;  /** @brief Delay line length value. */
extern const field32_t DELAY_BLOCK_QUADSPI_CFGR_LNGF; /** @brief Length valid flag. */

/**************************************************************************************************
 * @section FLASH Definitions
 **************************************************************************************************/

/** @subsection FLASH Register Definitions */

extern rw_reg32_t const FLASH_ACR;        /** @brief Access control register. */
extern rw_reg32_t const FLASH_ACR_;       /** @brief Access control register. */
extern rw_reg32_t const FLASH_KEYR1;      /** @brief FLASH key register for bank 1. */
extern rw_reg32_t const FLASH_OPTKEYR;    /** @brief FLASH option key register. */
extern rw_reg32_t const FLASH_OPTKEYR_;   /** @brief FLASH option key register. */
extern rw_reg32_t const FLASH_CR1;        /** @brief FLASH control register for bank 1. */
extern rw_reg32_t const FLASH_SR1;        /** @brief FLASH status register for bank 1. */
extern rw_reg32_t const FLASH_CCR1;       /** @brief FLASH clear control register for bank 1. */
extern rw_reg32_t const FLASH_OPTCR;      /** @brief FLASH option control register. */
extern rw_reg32_t const FLASH_OPTCR_;     /** @brief FLASH option control register. */
extern rw_reg32_t const FLASH_OPTSR_CUR_; /** @brief FLASH option status register. */
extern rw_reg32_t const FLASH_OPTSR_CUR;  /** @brief FLASH option status register. */
extern rw_reg32_t const FLASH_OPTSR_PRG;  /** @brief FLASH option status register. */
extern rw_reg32_t const FLASH_OPTSR_PRG_; /** @brief FLASH option status register. */
extern rw_reg32_t const FLASH_OPTCCR_;    /** @brief FLASH option clear control register. */
extern rw_reg32_t const FLASH_OPTCCR;     /** @brief FLASH option clear control register. */
extern ro_reg32_t const FLASH_PRAR_CUR1;  /** @brief FLASH protection address for bank 1. */
extern rw_reg32_t const FLASH_PRAR_PRG1;  /** @brief FLASH protection address for bank 1. */
extern rw_reg32_t const FLASH_SCAR_CUR1;  /** @brief FLASH secure address for bank 1. */
extern rw_reg32_t const FLASH_SCAR_PRG1;  /** @brief FLASH secure address for bank 1. */
extern ro_reg32_t const FLASH_WPSN_CUR1R; /** @brief FLASH write sector protection for bank 1. */
extern rw_reg32_t const FLASH_WPSN_PRG1R; /** @brief FLASH write sector protection for bank 1. */
extern ro_reg32_t const FLASH_BOOT_CURR;  /** @brief FLASH register with boot address. */
extern ro_reg32_t const FLASH_BOOT_PRGR;  /** @brief FLASH register with boot address. */
extern rw_reg32_t const FLASH_CRCDATAR;   /** @brief FLASH CRC data register. */
extern ro_reg32_t const FLASH_ECC_FA1R;   /** @brief FLASH ECC fail address for bank 1. */
extern ro_reg32_t const FLASH_KEYR2;      /** @brief FLASH key register for bank 2. */
extern rw_reg32_t const FLASH_CR2;        /** @brief FLASH control register for bank 2. */
extern rw_reg32_t const FLASH_SR2;        /** @brief FLASH status register for bank 2. */
extern rw_reg32_t const FLASH_CCR2;       /** @brief FLASH clear control register for bank 2. */
extern ro_reg32_t const FLASH_PRAR_CUR2;  /** @brief FLASH protection address for bank 1. */
extern rw_reg32_t const FLASH_PRAR_PRG2;  /** @brief FLASH protection address for bank 2. */
extern rw_reg32_t const FLASH_SCAR_CUR2;  /** @brief FLASH secure address for bank 2. */
extern rw_reg32_t const FLASH_SCAR_PRG2;  /** @brief FLASH secure address for bank 2. */
extern ro_reg32_t const FLASH_WPSN_CUR2R; /** @brief FLASH write sector protection for bank 2. */
extern rw_reg32_t const FLASH_WPSN_PRG2R; /** @brief FLASH write sector protection for bank 2. */
extern ro_reg32_t const FLASH_ECC_FA2R;   /** @brief FLASH ECC fail address for bank 2. */

/** @subsection Enumerated FLASH Register Definitions */

extern rw_reg32_t const FLASH_CRCCRx[3];    /** @brief FLASH CRC control register for bank 1. */
extern rw_reg32_t const FLASH_CRCSADDxR[3]; /** @brief FLASH CRC start address register for bank 1. */
extern rw_reg32_t const FLASH_CRCEADDxR[3]; /** @brief FLASH CRC end address register for bank 1. */

/** @subsection FLASH Register Field Definitions */

extern const field32_t FLASH_ACR_LATENCY;                /** @brief Read latency. */
extern const field32_t FLASH_ACR_WRHIGHFREQ;             /** @brief Flash signal delay. */
extern const field32_t FLASH_ACR__LATENCY;               /** @brief Read latency. */
extern const field32_t FLASH_ACR__WRHIGHFREQ;            /** @brief Flash signal delay. */
extern const field32_t FLASH_CR1_LOCK1;                  /** @brief Bank 1 configuration lock bit. */
extern const field32_t FLASH_CR1_PG1;                    /** @brief Bank 1 program enable bit. */
extern const field32_t FLASH_CR1_SER1;                   /** @brief Bank 1 sector erase request. */
extern const field32_t FLASH_CR1_BER1;                   /** @brief Bank 1 erase request. */
extern const field32_t FLASH_CR1_PSIZE1;                 /** @brief Bank 1 program size. */
extern const field32_t FLASH_CR1_FW1;                    /** @brief Bank 1 write forcing control bit. */
extern const field32_t FLASH_CR1_START1;                 /** @brief Bank 1 bank or sector erase start control bit. */
extern const field32_t FLASH_CR1_SNB1;                   /** @brief Bank 1 sector erase selection number. */
extern const field32_t FLASH_CR1_CRC_EN;                 /** @brief Bank 1 CRC control bit. */
extern const field32_t FLASH_CR1_EOPIE1;                 /** @brief Bank 1 end-of-program interrupt control bit. */
extern const field32_t FLASH_CR1_WRPERRIE1;              /** @brief Bank 1 write protection error interrupt enable bit. */
extern const field32_t FLASH_CR1_PGSERRIE1;              /** @brief Bank 1 programming sequence error interrupt enable bit. */
extern const field32_t FLASH_CR1_STRBERRIE1;             /** @brief Bank 1 strobe error interrupt enable bit. */
extern const field32_t FLASH_CR1_INCERRIE1;              /** @brief Bank 1 inconsistency error interrupt enable bit. */
extern const field32_t FLASH_CR1_OPERRIE1;               /** @brief Bank 1 write/erase error interrupt enable bit. */
extern const field32_t FLASH_CR1_RDPERRIE1;              /** @brief Bank 1 read protection error interrupt enable bit. */
extern const field32_t FLASH_CR1_RDSERRIE1;              /** @brief Bank 1 secure error interrupt enable bit. */
extern const field32_t FLASH_CR1_SNECCERRIE1;            /** @brief Bank 1 ECC single correction error interrupt enable bit. */
extern const field32_t FLASH_CR1_DBECCERRIE1;            /** @brief Bank 1 ECC double detection error interrupt enable bit. */
extern const field32_t FLASH_CR1_CRCENDIE1;              /** @brief Bank 1 end of CRC calculation interrupt enable bit. */
extern const field32_t FLASH_SR1_BSY1;                   /** @brief Bank 1 ongoing program flag. */
extern const field32_t FLASH_SR1_WBNE1;                  /** @brief Bank 1 write buffer not empty flag. */
extern const field32_t FLASH_SR1_QW1;                    /** @brief Bank 1 wait queue flag. */
extern const field32_t FLASH_SR1_CRC_BUSY1;              /** @brief Bank 1 CRC busy flag. */
extern const field32_t FLASH_SR1_EOP1;                   /** @brief Bank 1 end-of-program flag. */
extern const field32_t FLASH_SR1_WRPERR1;                /** @brief Bank 1 write protection error flag. */
extern const field32_t FLASH_SR1_PGSERR1;                /** @brief Bank 1 programming sequence error flag. */
extern const field32_t FLASH_SR1_STRBERR1;               /** @brief Bank 1 strobe error flag. */
extern const field32_t FLASH_SR1_INCERR1;                /** @brief Bank 1 inconsistency error flag. */
extern const field32_t FLASH_SR1_OPERR1;                 /** @brief Bank 1 write/erase error flag. */
extern const field32_t FLASH_SR1_RDPERR1;                /** @brief Bank 1 read protection error flag. */
extern const field32_t FLASH_SR1_RDSERR1;                /** @brief Bank 1 secure error flag. */
extern const field32_t FLASH_SR1_SNECCERR11;             /** @brief Bank 1 single correction error flag. */
extern const field32_t FLASH_SR1_DBECCERR1;              /** @brief Bank 1 ECC double detection error flag. */
extern const field32_t FLASH_SR1_CRCEND1;                /** @brief Bank 1 crc-complete flag. */
extern const field32_t FLASH_CCR1_CLR_EOP1;              /** @brief Bank 1 EOP1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_WRPERR1;           /** @brief Bank 1 WRPERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_PGSERR1;           /** @brief Bank 1 PGSERR1 flag clear bi. */
extern const field32_t FLASH_CCR1_CLR_STRBERR1;          /** @brief Bank 1 STRBERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_INCERR1;           /** @brief Bank 1 INCERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_OPERR1;            /** @brief Bank 1 OPERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_RDPERR1;           /** @brief Bank 1 RDPERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_RDSERR1;           /** @brief Bank 1 RDSERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_SNECCERR1;         /** @brief Bank 1 SNECCERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_DBECCERR1;         /** @brief Bank 1 DBECCERR1 flag clear bit. */
extern const field32_t FLASH_CCR1_CLR_CRCEND1;           /** @brief Bank 1 CRCEND1 flag clear bit. */
extern const field32_t FLASH_OPTCR_OPTLOCK;              /** @brief FLASH_OPTCR lock option configuration bit. */
extern const field32_t FLASH_OPTCR_OPTSTART;             /** @brief Option byte start change option configuration bit. */
extern const field32_t FLASH_OPTCR_MER;                  /** @brief Flash mass erase enable bit. */
extern const field32_t FLASH_OPTCR_OPTCHANGEERRIE;       /** @brief Option byte change error interrupt enable bit. */
extern const field32_t FLASH_OPTCR_SWAP_BANK;            /** @brief Bank swapping configuration bit. */
extern const field32_t FLASH_OPTCR__OPTLOCK;             /** @brief FLASH_OPTCR lock option configuration bit. */
extern const field32_t FLASH_OPTCR__OPTSTART;            /** @brief Option byte start change option configuration bit. */
extern const field32_t FLASH_OPTCR__MER;                 /** @brief Flash mass erase enable bit. */
extern const field32_t FLASH_OPTCR__OPTCHANGEERRIE;      /** @brief Option byte change error interrupt enable bit. */
extern const field32_t FLASH_OPTCR__SWAP_BANK;           /** @brief Bank swapping configuration bit. */
extern const field32_t FLASH_OPTSR_CUR__OPT_BUSY;        /** @brief Option byte change ongoing flag. */
extern const field32_t FLASH_OPTSR_CUR__BOR_LEV;         /** @brief Brownout level option status bit. */
extern const field32_t FLASH_OPTSR_CUR__IWDG1_HW;        /** @brief IWDG1 control option status bit. */
extern const field32_t FLASH_OPTSR_CUR__NRST_STOP_D1;    /** @brief D1 dstop entry reset option status bit. */
extern const field32_t FLASH_OPTSR_CUR__NRST_STBY_D1;    /** @brief D1 dstandby entry reset option status bit. */
extern const field32_t FLASH_OPTSR_CUR__RDP;             /** @brief Readout protection level option status byte. */
extern const field32_t FLASH_OPTSR_CUR__FZ_IWDG_STOP;    /** @brief IWDG stop mode freeze option status bit. */
extern const field32_t FLASH_OPTSR_CUR__FZ_IWDG_SDBY;    /** @brief IWDG standby mode freeze option status bit. */
extern const field32_t FLASH_OPTSR_CUR__ST_RAM_SIZE;     /** @brief DTCM RAM size option status. */
extern const field32_t FLASH_OPTSR_CUR__SECURITY;        /** @brief Security enable option status bit. */
extern const field32_t FLASH_OPTSR_CUR__RSS1;            /** @brief User option bit 1. */
extern const field32_t FLASH_OPTSR_CUR__PERSO_OK;        /** @brief Device personalization status bit. */
extern const field32_t FLASH_OPTSR_CUR__IO_HSLV;         /** @brief I/O high-speed at low-voltage status bit (PRODUCT_BELOW_25V). */
extern const field32_t FLASH_OPTSR_CUR__OPTCHANGEERR;    /** @brief Option byte change error flag. */
extern const field32_t FLASH_OPTSR_CUR__SWAP_BANK_OPT;   /** @brief Bank swapping option status bit. */
extern const field32_t FLASH_OPTSR_CUR_OPT_BUSY;         /** @brief Option byte change ongoing flag. */
extern const field32_t FLASH_OPTSR_CUR_BOR_LEV;          /** @brief Brownout level option status bit. */
extern const field32_t FLASH_OPTSR_CUR_IWDG1_HW;         /** @brief IWDG1 control option status bit. */
extern const field32_t FLASH_OPTSR_CUR_NRST_STOP_D1;     /** @brief D1 dstop entry reset option status bit. */
extern const field32_t FLASH_OPTSR_CUR_NRST_STBY_D1;     /** @brief D1 dstandby entry reset option status bit. */
extern const field32_t FLASH_OPTSR_CUR_RDP;              /** @brief Readout protection level option status byte. */
extern const field32_t FLASH_OPTSR_CUR_FZ_IWDG_STOP;     /** @brief IWDG stop mode freeze option status bit. */
extern const field32_t FLASH_OPTSR_CUR_FZ_IWDG_SDBY;     /** @brief IWDG standby mode freeze option status bit. */
extern const field32_t FLASH_OPTSR_CUR_ST_RAM_SIZE;      /** @brief DTCM RAM size option status. */
extern const field32_t FLASH_OPTSR_CUR_SECURITY;         /** @brief Security enable option status bit. */
extern const field32_t FLASH_OPTSR_CUR_RSS1;             /** @brief User option bit 1. */
extern const field32_t FLASH_OPTSR_CUR_PERSO_OK;         /** @brief Device personalization status bit. */
extern const field32_t FLASH_OPTSR_CUR_IO_HSLV;          /** @brief I/O high-speed at low-voltage status bit (PRODUCT_BELOW_25V). */
extern const field32_t FLASH_OPTSR_CUR_OPTCHANGEERR;     /** @brief Option byte change error flag. */
extern const field32_t FLASH_OPTSR_CUR_SWAP_BANK_OPT;    /** @brief Bank swapping option status bit. */
extern const field32_t FLASH_OPTSR_PRG_BOR_LEV;          /** @brief BOR reset level option configuration bits. */
extern const field32_t FLASH_OPTSR_PRG_IWDG1_HW;         /** @brief IWDG1 option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG_NRST_STOP_D1;     /** @brief Option byte erase after D1 dstop option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG_NRST_STBY_D1;     /** @brief Option byte erase after D1 dstandby option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG_RDP;              /** @brief Readout protection level option configuration byte. */
extern const field32_t FLASH_OPTSR_PRG_FZ_IWDG_STOP;     /** @brief IWDG stop mode freeze option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG_FZ_IWDG_SDBY;     /** @brief IWDG standby mode freeze option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG_ST_RAM_SIZE;      /** @brief DTCM size select option configuration bits. */
extern const field32_t FLASH_OPTSR_PRG_SECURITY;         /** @brief Security option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG_IO_HSLV;          /** @brief I/O high-speed at low-voltage (PRODUCT_BELOW_25V). */
extern const field32_t FLASH_OPTSR_PRG_SWAP_BANK_OPT;    /** @brief Bank swapping option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__BOR_LEV;         /** @brief BOR reset level option configuration bits. */
extern const field32_t FLASH_OPTSR_PRG__IWDG1_HW;        /** @brief IWDG1 option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__NRST_STOP_D1;    /** @brief Option byte erase after D1 dstop option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__NRST_STBY_D1;    /** @brief Option byte erase after D1 dstandby option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__RDP;             /** @brief Readout protection level option configuration byte. */
extern const field32_t FLASH_OPTSR_PRG__FZ_IWDG_STOP;    /** @brief IWDG stop mode freeze option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__FZ_IWDG_SDBY;    /** @brief IWDG standby mode freeze option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__ST_RAM_SIZE;     /** @brief DTCM size select option configuration bits. */
extern const field32_t FLASH_OPTSR_PRG__SECURITY;        /** @brief Security option configuration bit. */
extern const field32_t FLASH_OPTSR_PRG__IO_HSLV;         /** @brief I/O high-speed at low-voltage (PRODUCT_BELOW_25V). */
extern const field32_t FLASH_OPTSR_PRG__SWAP_BANK_OPT;   /** @brief Bank swapping option configuration bit. */
extern const field32_t FLASH_OPTCCR__CLR_OPTCHANGEERR;   /** @brief OPTCHANGEERR reset bit. */
extern const field32_t FLASH_OPTCCR_CLR_OPTCHANGEERR;    /** @brief OPTCHANGEERR reset bit. */
extern const field32_t FLASH_PRAR_CUR1_PROT_AREA_START1; /** @brief Bank 1 lowest PCROP protected address. */
extern const field32_t FLASH_PRAR_CUR1_PROT_AREA_END1;   /** @brief Bank 1 highest PCROP protected address. */
extern const field32_t FLASH_PRAR_CUR1_DMEP1;            /** @brief Bank 1 PCROP protected erase enable option status bit. */
extern const field32_t FLASH_PRAR_PRG1_PROT_AREA_START1; /** @brief Bank 1 lowest PCROP protected address configuration. */
extern const field32_t FLASH_PRAR_PRG1_PROT_AREA_END1;   /** @brief Bank 1 highest PCROP protected address configuration. */
extern const field32_t FLASH_PRAR_PRG1_DMEP1;            /** @brief Bank 1 PCROP protected erase enable option configuration bit. */
extern const field32_t FLASH_SCAR_CUR1_SEC_AREA_START1;  /** @brief Bank 1 lowest secure protected address. */
extern const field32_t FLASH_SCAR_CUR1_SEC_AREA_END1;    /** @brief Bank 1 highest secure protected address. */
extern const field32_t FLASH_SCAR_CUR1_DMES1;            /** @brief Bank 1 secure protected erase enable option status bit. */
extern const field32_t FLASH_SCAR_PRG1_SEC_AREA_START1;  /** @brief Bank 1 lowest secure protected address configuration. */
extern const field32_t FLASH_SCAR_PRG1_SEC_AREA_END1;    /** @brief Bank 1 highest secure protected address configuration. */
extern const field32_t FLASH_SCAR_PRG1_DMES1;            /** @brief Bank 1 secure protected erase enable option configuration bit. */
extern const field32_t FLASH_WPSN_CUR1R_WRPSN1;          /** @brief Bank 1 sector write protection option status byte. */
extern const field32_t FLASH_WPSN_PRG1R_WRPSN1;          /** @brief Bank 1 sector write protection configuration byte. */
extern const field32_t FLASH_CRCCRx_CRC_SECT;            /** @brief Bank 1 CRC sector number. */
extern const field32_t FLASH_CRCCRx_ALL_BANK;            /** @brief Bank 1 CRC select bit. */
extern const field32_t FLASH_CRCCRx_CRC_BY_SECT;         /** @brief Bank 1 CRC sector mode select bit. */
extern const field32_t FLASH_CRCCRx_ADD_SECT;            /** @brief Bank 1 CRC sector select bit. */
extern const field32_t FLASH_CRCCRx_CLEAN_SECT;          /** @brief Bank 1 CRC sector list clear bit. */
extern const field32_t FLASH_CRCCRx_START_CRC;           /** @brief Bank 1 CRC start bit. */
extern const field32_t FLASH_CRCCRx_CLEAN_CRC;           /** @brief Bank 1 CRC clear bit. */
extern const field32_t FLASH_CRCCRx_CRC_BURST;           /** @brief Bank 1 CRC burst size. */
extern const field32_t FLASH_ECC_FA1R_FAIL_ECC_ADDR1;    /** @brief Bank 1 ECC error address. */
extern const field32_t FLASH_CR2_LOCK2;                  /** @brief Bank 2 configuration lock bit. */
extern const field32_t FLASH_CR2_PG2;                    /** @brief Bank 2 program enable bit. */
extern const field32_t FLASH_CR2_SER2;                   /** @brief Bank 2 sector erase request. */
extern const field32_t FLASH_CR2_BER2;                   /** @brief Bank 2 erase request. */
extern const field32_t FLASH_CR2_PSIZE2;                 /** @brief Bank 2 program size. */
extern const field32_t FLASH_CR2_FW2;                    /** @brief Bank 2 write forcing control bit. */
extern const field32_t FLASH_CR2_START2;                 /** @brief Bank 2 bank or sector erase start control bit. */
extern const field32_t FLASH_CR2_SNB2;                   /** @brief Bank 2 sector erase selection number. */
extern const field32_t FLASH_CR2_CRC_EN;                 /** @brief Bank 2 CRC control bit. */
extern const field32_t FLASH_CR2_EOPIE2;                 /** @brief Bank 2 end-of-program interrupt control bit. */
extern const field32_t FLASH_CR2_WRPERRIE2;              /** @brief Bank 2 write protection error interrupt enable bit. */
extern const field32_t FLASH_CR2_PGSERRIE2;              /** @brief Bank 2 programming sequence error interrupt enable bit. */
extern const field32_t FLASH_CR2_STRBERRIE2;             /** @brief Bank 2 strobe error interrupt enable bit. */
extern const field32_t FLASH_CR2_INCERRIE2;              /** @brief Bank 2 inconsistency error interrupt enable bit. */
extern const field32_t FLASH_CR2_OPERRIE2;               /** @brief Bank 2 write/erase error interrupt enable bit. */
extern const field32_t FLASH_CR2_RDPERRIE2;              /** @brief Bank 2 read protection error interrupt enable bit. */
extern const field32_t FLASH_CR2_RDSERRIE2;              /** @brief Bank 2 secure error interrupt enable bit. */
extern const field32_t FLASH_CR2_SNECCERRIE2;            /** @brief Bank 2 ECC single correction error interrupt enable bit. */
extern const field32_t FLASH_CR2_DBECCERRIE2;            /** @brief Bank 2 ECC double detection error interrupt enable bit. */
extern const field32_t FLASH_CR2_CRCENDIE2;              /** @brief Bank 2 end of CRC calculation interrupt enable bit. */
extern const field32_t FLASH_SR2_BSY2;                   /** @brief Bank 2 ongoing program flag. */
extern const field32_t FLASH_SR2_WBNE2;                  /** @brief Bank 2 write buffer not empty flag. */
extern const field32_t FLASH_SR2_QW2;                    /** @brief Bank 2 wait queue flag. */
extern const field32_t FLASH_SR2_CRC_BUSY2;              /** @brief Bank 2 CRC busy flag. */
extern const field32_t FLASH_SR2_EOP2;                   /** @brief Bank 2 end-of-program flag. */
extern const field32_t FLASH_SR2_WRPERR2;                /** @brief Bank 2 write protection error flag. */
extern const field32_t FLASH_SR2_PGSERR2;                /** @brief Bank 2 programming sequence error flag. */
extern const field32_t FLASH_SR2_STRBERR2;               /** @brief Bank 2 strobe error flag. */
extern const field32_t FLASH_SR2_INCERR2;                /** @brief Bank 2 inconsistency error flag. */
extern const field32_t FLASH_SR2_OPERR2;                 /** @brief Bank 2 write/erase error flag. */
extern const field32_t FLASH_SR2_RDPERR2;                /** @brief Bank 2 read protection error flag. */
extern const field32_t FLASH_SR2_RDSERR2;                /** @brief Bank 2 secure error flag. */
extern const field32_t FLASH_SR2_SNECCERR2;              /** @brief Bank 2 single correction error flag. */
extern const field32_t FLASH_SR2_DBECCERR2;              /** @brief Bank 2 ECC double detection error flag. */
extern const field32_t FLASH_SR2_CRCEND2;                /** @brief Bank 2 crc-complete flag. */
extern const field32_t FLASH_CCR2_CLR_EOP2;              /** @brief Bank 1 EOP1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_WRPERR2;           /** @brief Bank 2 WRPERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_PGSERR2;           /** @brief Bank 2 PGSERR1 flag clear bi. */
extern const field32_t FLASH_CCR2_CLR_STRBERR2;          /** @brief Bank 2 STRBERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_INCERR2;           /** @brief Bank 2 INCERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_OPERR2;            /** @brief Bank 2 OPERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_RDPERR2;           /** @brief Bank 2 RDPERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_RDSERR1;           /** @brief Bank 1 RDSERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_SNECCERR2;         /** @brief Bank 2 SNECCERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_DBECCERR1;         /** @brief Bank 1 DBECCERR1 flag clear bit. */
extern const field32_t FLASH_CCR2_CLR_CRCEND2;           /** @brief Bank 2 CRCEND1 flag clear bit. */
extern const field32_t FLASH_PRAR_CUR2_PROT_AREA_START2; /** @brief Bank 2 lowest PCROP protected address. */
extern const field32_t FLASH_PRAR_CUR2_PROT_AREA_END2;   /** @brief Bank 2 highest PCROP protected address. */
extern const field32_t FLASH_PRAR_CUR2_DMEP2;            /** @brief Bank 2 PCROP protected erase enable option status bit. */
extern const field32_t FLASH_PRAR_PRG2_PROT_AREA_START2; /** @brief Bank 2 lowest PCROP protected address configuration. */
extern const field32_t FLASH_PRAR_PRG2_PROT_AREA_END2;   /** @brief Bank 2 highest PCROP protected address configuration. */
extern const field32_t FLASH_PRAR_PRG2_DMEP2;            /** @brief Bank 2 PCROP protected erase enable option configuration bit. */
extern const field32_t FLASH_SCAR_CUR2_SEC_AREA_START2;  /** @brief Bank 2 lowest secure protected address. */
extern const field32_t FLASH_SCAR_CUR2_SEC_AREA_END2;    /** @brief Bank 2 highest secure protected address. */
extern const field32_t FLASH_SCAR_CUR2_DMES2;            /** @brief Bank 2 secure protected erase enable option status bit. */
extern const field32_t FLASH_SCAR_PRG2_SEC_AREA_START2;  /** @brief Bank 2 lowest secure protected address configuration. */
extern const field32_t FLASH_SCAR_PRG2_SEC_AREA_END2;    /** @brief Bank 2 highest secure protected address configuration. */
extern const field32_t FLASH_SCAR_PRG2_DMES2;            /** @brief Bank 2 secure protected erase enable option configuration bit. */
extern const field32_t FLASH_WPSN_CUR2R_WRPSN2;          /** @brief Bank 2 sector write protection option status byte. */
extern const field32_t FLASH_WPSN_PRG2R_WRPSN2;          /** @brief Bank 2 sector write protection configuration byte. */
extern const field32_t FLASH_ECC_FA2R_FAIL_ECC_ADDR2;    /** @brief Bank 2 ECC error address. */

/** @subsection Enumerated FLASH Register Field Definitions */

extern const field32_t FLASH_OPTSR_PRG_RSSx[3];      /** @brief User option configuration bit 1. */
extern const field32_t FLASH_BOOT_CURR_BOOT_ADDx[2]; /** @brief Boot address 0. */

/**************************************************************************************************
 * @section AXI Definitions
 **************************************************************************************************/

/** @subsection AXI Register Definitions */

extern ro_reg32_t const AXI_AXI_PERIPH_ID_4; /** @brief AXI interconnect - peripheral ID4 register. */
extern ro_reg32_t const AXI_AXI_PERIPH_ID_0; /** @brief AXI interconnect - peripheral ID0 register. */
extern ro_reg32_t const AXI_AXI_PERIPH_ID_1; /** @brief AXI interconnect - peripheral ID1 register. */
extern ro_reg32_t const AXI_AXI_PERIPH_ID_2; /** @brief AXI interconnect - peripheral ID2 register. */
extern ro_reg32_t const AXI_AXI_PERIPH_ID_3; /** @brief AXI interconnect - peripheral ID3 register. */
extern ro_reg32_t const AXI_AXI_COMP_ID_0;   /** @brief AXI interconnect - component ID0 register. */
extern ro_reg32_t const AXI_AXI_COMP_ID_1;   /** @brief AXI interconnect - component ID1 register. */
extern ro_reg32_t const AXI_AXI_COMP_ID_2;   /** @brief AXI interconnect - component ID2 register. */
extern ro_reg32_t const AXI_AXI_COMP_ID_3;   /** @brief AXI interconnect - component ID3 register. */

/** @subsection Enumerated AXI Register Definitions */

extern rw_reg32_t const AXI_AXI_TARGx_FN_MOD_ISS_BM[8]; /** @brief AXI interconnect - TARG x bus matrix issuing functionality register. */
extern rw_reg32_t const AXI_AXI_TARGx_FN_MOD2[8];       /** @brief AXI interconnect - TARG x bus matrix functionality 2 register. */
extern rw_reg32_t const AXI_AXI_TARGx_FN_MOD_LB[3];     /** @brief AXI interconnect - TARG x long burst functionality modification. */
extern rw_reg32_t const AXI_AXI_TARGx_FN_MOD[8];        /** @brief AXI interconnect - TARG x long burst functionality modification. */
extern rw_reg32_t const AXI_AXI_INIx_FN_MOD2[4];        /** @brief AXI interconnect - INI x functionality modification 2 register. */
extern rw_reg32_t const AXI_AXI_INIx_FN_MOD_AHB[4];     /** @brief AXI interconnect - INI x AHB functionality modification register. */
extern rw_reg32_t const AXI_AXI_INIx_READ_QOS[7];       /** @brief AXI interconnect - INI x read qos register. */
extern rw_reg32_t const AXI_AXI_INIx_WRITE_QOS[7];      /** @brief AXI interconnect - INI x write qos register. */
extern rw_reg32_t const AXI_AXI_INIx_FN_MOD[7];         /** @brief AXI interconnect - INI x issuing functionality modification register. */

/** @subsection AXI Register Field Definitions */

extern const field32_t AXI_AXI_PERIPH_ID_4_JEP106CON;                  /** @brief JEP106 continuation code. */
extern const field32_t AXI_AXI_PERIPH_ID_4_KCOUNT4;                    /** @brief Register file size. */
extern const field32_t AXI_AXI_PERIPH_ID_0_PARTNUM;                    /** @brief Peripheral part number bits 0 to 7. */
extern const field32_t AXI_AXI_PERIPH_ID_1_PARTNUM;                    /** @brief Peripheral part number bits 8 to 11. */
extern const field32_t AXI_AXI_PERIPH_ID_1_JEP106I;                    /** @brief JEP106 identity bits 0 to 3. */
extern const field32_t AXI_AXI_PERIPH_ID_2_JEP106ID;                   /** @brief JEP106 identity bits 4 to 6. */
extern const field32_t AXI_AXI_PERIPH_ID_2_JEDEC;                      /** @brief JEP106 code flag. */
extern const field32_t AXI_AXI_PERIPH_ID_2_REVISION;                   /** @brief Peripheral revision number. */
extern const field32_t AXI_AXI_PERIPH_ID_3_CUST_MOD_NUM;               /** @brief Customer modification. */
extern const field32_t AXI_AXI_PERIPH_ID_3_REV_AND;                    /** @brief Customer version. */
extern const field32_t AXI_AXI_COMP_ID_0_PREAMBLE;                     /** @brief Preamble bits 0 to 7. */
extern const field32_t AXI_AXI_COMP_ID_1_PREAMBLE;                     /** @brief Preamble bits 8 to 11. */
extern const field32_t AXI_AXI_COMP_ID_1_CLASS;                        /** @brief Component class. */
extern const field32_t AXI_AXI_COMP_ID_2_PREAMBLE;                     /** @brief Preamble bits 12 to 19. */
extern const field32_t AXI_AXI_COMP_ID_3_PREAMBLE;                     /** @brief Preamble bits 20 to 27. */
extern const field32_t AXI_AXI_TARGx_FN_MOD_ISS_BM_READ_ISS_OVERRIDE;  /** @brief READ_ISS_OVERRIDE. */
extern const field32_t AXI_AXI_TARGx_FN_MOD_ISS_BM_WRITE_ISS_OVERRIDE; /** @brief Switch matrix write issuing override for target. */
extern const field32_t AXI_AXI_TARGx_FN_MOD2_BYPASS_MERGE;             /** @brief Disable packing of beats to match the output data width. */
extern const field32_t AXI_AXI_TARGx_FN_MOD_LB_FN_MOD_LB;              /** @brief Controls burst breaking of long bursts. */
extern const field32_t AXI_AXI_TARGx_FN_MOD_READ_ISS_OVERRIDE;         /** @brief Override AMIB read issuing capability. */
extern const field32_t AXI_AXI_TARGx_FN_MOD_WRITE_ISS_OVERRIDE;        /** @brief Override AMIB write issuing capability. */
extern const field32_t AXI_AXI_INIx_FN_MOD2_BYPASS_MERGE;              /** @brief Disables alteration of transactions by the up-sizer unless required by the protocol. */
extern const field32_t AXI_AXI_INIx_FN_MOD_AHB_RD_INC_OVERRIDE;        /** @brief Converts all ahb-lite write transactions to a series of single beat AXI. */
extern const field32_t AXI_AXI_INIx_FN_MOD_AHB_WR_INC_OVERRIDE;        /** @brief Converts all ahb-lite read transactions to a series of single beat AXI. */
extern const field32_t AXI_AXI_INIx_READ_QOS_AR_QOS;                   /** @brief Read channel qos setting. */
extern const field32_t AXI_AXI_INIx_WRITE_QOS_AW_QOS;                  /** @brief Write channel qos setting. */
extern const field32_t AXI_AXI_INIx_FN_MOD_READ_ISS_OVERRIDE;          /** @brief Override ASIB read issuing capability. */
extern const field32_t AXI_AXI_INIx_FN_MOD_WRITE_ISS_OVERRIDE;         /** @brief Override ASIB write issuing capability. */

/**************************************************************************************************
 * @section HASH Definitions
 **************************************************************************************************/

/** @subsection HASH Register Definitions */

extern rw_reg32_t const HASH_CR;       /** @brief Control register. */
extern rw_reg32_t const HASH_DIN;      /** @brief Data input register. */
extern rw_reg32_t const HASH_STR;      /** @brief Start register. */
extern ro_reg32_t const HASH_HR0;      /** @brief Digest registers. */
extern ro_reg32_t const HASH_HR1;      /** @brief Digest registers. */
extern ro_reg32_t const HASH_HR2;      /** @brief Digest registers. */
extern ro_reg32_t const HASH_HR3;      /** @brief Digest registers. */
extern ro_reg32_t const HASH_HR4;      /** @brief Digest registers. */
extern rw_reg32_t const HASH_IMR;      /** @brief Interrupt enable register. */
extern rw_reg32_t const HASH_SR;       /** @brief Status register. */
extern rw_reg32_t const HASH_CSR0;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR1;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR2;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR3;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR4;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR5;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR6;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR7;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR8;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR9;     /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR10;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR11;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR12;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR13;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR14;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR15;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR16;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR17;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR18;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR19;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR20;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR21;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR22;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR23;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR24;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR25;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR26;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR27;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR28;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR29;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR30;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR31;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR32;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR33;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR34;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR35;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR36;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR37;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR38;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR39;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR40;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR41;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR42;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR43;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR44;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR45;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR46;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR47;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR48;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR49;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR50;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR51;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR52;    /** @brief Context swap registers. */
extern rw_reg32_t const HASH_CSR53;    /** @brief Context swap registers. */
extern ro_reg32_t const HASH_HASH_HR0; /** @brief HASH digest register. */
extern ro_reg32_t const HASH_HASH_HR1; /** @brief Read-only. */
extern ro_reg32_t const HASH_HASH_HR2; /** @brief Read-only. */
extern ro_reg32_t const HASH_HASH_HR3; /** @brief Read-only. */
extern ro_reg32_t const HASH_HASH_HR4; /** @brief Read-only. */
extern ro_reg32_t const HASH_HASH_HR5; /** @brief Read-only. */
extern ro_reg32_t const HASH_HASH_HR6; /** @brief Read-only. */
extern ro_reg32_t const HASH_HASH_HR7; /** @brief Read-only. */

/** @subsection HASH Register Field Definitions */

extern const field32_t HASH_CR_INIT;     /** @brief Initialize message digest calculation. */
extern const field32_t HASH_CR_DMAE;     /** @brief DMA enable. */
extern const field32_t HASH_CR_DATATYPE; /** @brief Data type selection. */
extern const field32_t HASH_CR_MODE;     /** @brief Mode selection. */
extern const field32_t HASH_CR_NBW;      /** @brief Number of words already pushed. */
extern const field32_t HASH_CR_DINNE;    /** @brief DIN not empty. */
extern const field32_t HASH_CR_MDMAT;    /** @brief Multiple DMA transfers. */
extern const field32_t HASH_CR_LKEY;     /** @brief Long key selection. */
extern const field32_t HASH_STR_DCAL;    /** @brief Digest calculation. */
extern const field32_t HASH_STR_NBLW;    /** @brief Number of valid bits in the last word of the message. */
extern const field32_t HASH_IMR_DCIE;    /** @brief Digest calculation completion interrupt enable. */
extern const field32_t HASH_IMR_DINIE;   /** @brief Data input interrupt enable. */
extern const field32_t HASH_SR_BUSY;     /** @brief Busy bit. */
extern const field32_t HASH_SR_DMAS;     /** @brief DMA status. */
extern const field32_t HASH_SR_DCIS;     /** @brief Digest calculation completion interrupt status. */
extern const field32_t HASH_SR_DINIS;    /** @brief Data input interrupt status. */

/** @subsection Enumerated HASH Register Field Definitions */

extern const field32_t HASH_CR_ALGOx[2]; /** @brief Algorithm selection. */

/**************************************************************************************************
 * @section CRYP Definitions
 **************************************************************************************************/

/** @subsection CRYP Register Definitions */

extern rw_reg32_t const CRYP_CR;         /** @brief Control register. */
extern ro_reg32_t const CRYP_SR;         /** @brief Status register. */
extern rw_reg32_t const CRYP_DIN;        /** @brief Data input register. */
extern ro_reg32_t const CRYP_DOUT;       /** @brief Data output register. */
extern rw_reg32_t const CRYP_DMACR;      /** @brief DMA control register. */
extern rw_reg32_t const CRYP_IMSCR;      /** @brief Interrupt mask set/clear register. */
extern ro_reg32_t const CRYP_RISR;       /** @brief Raw interrupt status register. */
extern ro_reg32_t const CRYP_MISR;       /** @brief Masked interrupt status register. */
extern rw_reg32_t const CRYP_K0LR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K0RR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K1LR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K1RR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K2LR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K2RR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K3LR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_K3RR;       /** @brief Key registers. */
extern rw_reg32_t const CRYP_IV0LR;      /** @brief Initialization vector registers. */
extern rw_reg32_t const CRYP_IV0RR;      /** @brief Initialization vector registers. */
extern rw_reg32_t const CRYP_IV1LR;      /** @brief Initialization vector registers. */
extern rw_reg32_t const CRYP_IV1RR;      /** @brief Initialization vector registers. */
extern rw_reg32_t const CRYP_CSGCMCCM0R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM1R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM2R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM3R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM4R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM5R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM6R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCMCCM7R; /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM0R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM1R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM2R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM3R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM4R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM5R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM6R;    /** @brief Context swap register. */
extern rw_reg32_t const CRYP_CSGCM7R;    /** @brief Context swap register. */

/** @subsection CRYP Register Field Definitions */

extern const field32_t CRYP_CR_ALGODIR;   /** @brief Algorithm direction. */
extern const field32_t CRYP_CR_DATATYPE;  /** @brief Data type selection. */
extern const field32_t CRYP_CR_KEYSIZE;   /** @brief Key size selection (AES mode only). */
extern const field32_t CRYP_CR_FFLUSH;    /** @brief FIFO flush. */
extern const field32_t CRYP_CR_CRYPEN;    /** @brief Cryptographic processor enable. */
extern const field32_t CRYP_CR_GCM_CCMPH; /** @brief GCM_CCMPH. */
extern const field32_t CRYP_SR_BUSY;      /** @brief Busy bit. */
extern const field32_t CRYP_SR_OFFU;      /** @brief Output FIFO full. */
extern const field32_t CRYP_SR_OFNE;      /** @brief Output FIFO not empty. */
extern const field32_t CRYP_SR_IFNF;      /** @brief Input FIFO not full. */
extern const field32_t CRYP_SR_IFEM;      /** @brief Input FIFO empty. */
extern const field32_t CRYP_DMACR_DOEN;   /** @brief DMA output enable. */
extern const field32_t CRYP_DMACR_DIEN;   /** @brief DMA input enable. */
extern const field32_t CRYP_IMSCR_OUTIM;  /** @brief Output FIFO service interrupt mask. */
extern const field32_t CRYP_IMSCR_INIM;   /** @brief Input FIFO service interrupt mask. */
extern const field32_t CRYP_RISR_OUTRIS;  /** @brief Output FIFO service raw interrupt status. */
extern const field32_t CRYP_RISR_INRIS;   /** @brief Input FIFO service raw interrupt status. */
extern const field32_t CRYP_MISR_OUTMIS;  /** @brief Output FIFO service masked interrupt status. */
extern const field32_t CRYP_MISR_INMIS;   /** @brief Input FIFO service masked interrupt status. */

/** @subsection Enumerated CRYP Register Field Definitions */

extern const field32_t CRYP_CR_ALGOMODEx[4]; /** @brief Algorithm mode. */
extern const field32_t CRYP_K0LR_B2x[56];    /** @brief B224. */
extern const field32_t CRYP_K0LR_B22x[10];   /** @brief B225. */
extern const field32_t CRYP_K0RR_B19x[10];   /** @brief B192. */
extern const field32_t CRYP_K0RR_B20x[10];   /** @brief B201. */
extern const field32_t CRYP_K1LR_B1x[92];    /** @brief B160. */
extern const field32_t CRYP_K1LR_B16x[10];   /** @brief B161. */
extern const field32_t CRYP_K1RR_B12x[10];   /** @brief B129. */
extern const field32_t CRYP_K2LR_B9x[10];    /** @brief B96. */
extern const field32_t CRYP_K2LR_B10x[10];   /** @brief B101. */
extern const field32_t CRYP_K2RR_Bx[96];     /** @brief B64. */
extern const field32_t CRYP_K2RR_B6x[10];    /** @brief B65. */
extern const field32_t CRYP_K3LR_B3x[10];    /** @brief B33. */
extern const field32_t CRYP_IV0LR_IVx[32];   /** @brief IV0. */
extern const field32_t CRYP_IV0LR_IV3x[2];   /** @brief IV30. */
extern const field32_t CRYP_IV0RR_IV6x[4];   /** @brief IV60. */
extern const field32_t CRYP_IV1LR_IV9x[6];   /** @brief IV90. */
extern const field32_t CRYP_IV1RR_IV1x[28];  /** @brief IV100. */
extern const field32_t CRYP_IV1RR_IV12x[8];  /** @brief IV120. */

/**************************************************************************************************
 * @section DCMI Definitions
 **************************************************************************************************/

/** @subsection DCMI Register Definitions */

extern rw_reg32_t const DCMI_CR;     /** @brief Control register 1. */
extern ro_reg32_t const DCMI_SR;     /** @brief Status register. */
extern ro_reg32_t const DCMI_RIS;    /** @brief Raw interrupt status register. */
extern rw_reg32_t const DCMI_IER;    /** @brief Interrupt enable register. */
extern ro_reg32_t const DCMI_MIS;    /** @brief Masked interrupt status register. */
extern rw_reg32_t const DCMI_ICR;    /** @brief Interrupt clear register. */
extern rw_reg32_t const DCMI_ESCR;   /** @brief Embedded synchronization code register. */
extern rw_reg32_t const DCMI_ESUR;   /** @brief Embedded synchronization unmask register. */
extern rw_reg32_t const DCMI_CWSTRT; /** @brief Crop window start. */
extern rw_reg32_t const DCMI_CWSIZE; /** @brief Crop window size. */
extern ro_reg32_t const DCMI_DR;     /** @brief Data register. */

/** @subsection DCMI Register Field Definitions */

extern const field32_t DCMI_CR_OELS;        /** @brief Odd/Even line select (line select start). */
extern const field32_t DCMI_CR_LSM;         /** @brief Line select mode. */
extern const field32_t DCMI_CR_OEBS;        /** @brief Odd/Even byte select (byte select start). */
extern const field32_t DCMI_CR_BSM;         /** @brief Byte select mode. */
extern const field32_t DCMI_CR_ENABLE;      /** @brief DCMI enable. */
extern const field32_t DCMI_CR_EDM;         /** @brief Extended data mode. */
extern const field32_t DCMI_CR_FCRC;        /** @brief Frame capture rate control. */
extern const field32_t DCMI_CR_VSPOL;       /** @brief Vertical synchronization polarity. */
extern const field32_t DCMI_CR_HSPOL;       /** @brief Horizontal synchronization polarity. */
extern const field32_t DCMI_CR_PCKPOL;      /** @brief Pixel clock polarity. */
extern const field32_t DCMI_CR_ESS;         /** @brief Embedded synchronization select. */
extern const field32_t DCMI_CR_JPEG;        /** @brief JPEG format. */
extern const field32_t DCMI_CR_CROP;        /** @brief Crop feature. */
extern const field32_t DCMI_CR_CM;          /** @brief Capture mode. */
extern const field32_t DCMI_CR_CAPTURE;     /** @brief Capture enable. */
extern const field32_t DCMI_SR_FNE;         /** @brief FIFO not empty. */
extern const field32_t DCMI_SR_VSYNC;       /** @brief VSYNC. */
extern const field32_t DCMI_SR_HSYNC;       /** @brief HSYNC. */
extern const field32_t DCMI_RIS_LINE_RIS;   /** @brief Line raw interrupt status. */
extern const field32_t DCMI_RIS_VSYNC_RIS;  /** @brief VSYNC raw interrupt status. */
extern const field32_t DCMI_RIS_ERR_RIS;    /** @brief Synchronization error raw interrupt status. */
extern const field32_t DCMI_RIS_OVR_RIS;    /** @brief Overrun raw interrupt status. */
extern const field32_t DCMI_RIS_FRAME_RIS;  /** @brief Capture complete raw interrupt status. */
extern const field32_t DCMI_IER_LINE_IE;    /** @brief Line interrupt enable. */
extern const field32_t DCMI_IER_VSYNC_IE;   /** @brief VSYNC interrupt enable. */
extern const field32_t DCMI_IER_ERR_IE;     /** @brief Synchronization error interrupt enable. */
extern const field32_t DCMI_IER_OVR_IE;     /** @brief Overrun interrupt enable. */
extern const field32_t DCMI_IER_FRAME_IE;   /** @brief Capture complete interrupt enable. */
extern const field32_t DCMI_MIS_LINE_MIS;   /** @brief Line masked interrupt status. */
extern const field32_t DCMI_MIS_VSYNC_MIS;  /** @brief VSYNC masked interrupt status. */
extern const field32_t DCMI_MIS_ERR_MIS;    /** @brief Synchronization error masked interrupt status. */
extern const field32_t DCMI_MIS_OVR_MIS;    /** @brief Overrun masked interrupt status. */
extern const field32_t DCMI_MIS_FRAME_MIS;  /** @brief Capture complete masked interrupt status. */
extern const field32_t DCMI_ICR_LINE_ISC;   /** @brief Line interrupt status clear. */
extern const field32_t DCMI_ICR_VSYNC_ISC;  /** @brief Vertical synch interrupt status clear. */
extern const field32_t DCMI_ICR_ERR_ISC;    /** @brief Synchronization error interrupt status clear. */
extern const field32_t DCMI_ICR_OVR_ISC;    /** @brief Overrun interrupt status clear. */
extern const field32_t DCMI_ICR_FRAME_ISC;  /** @brief Capture complete interrupt status clear. */
extern const field32_t DCMI_ESCR_FEC;       /** @brief Frame end delimiter code. */
extern const field32_t DCMI_ESCR_LEC;       /** @brief Line end delimiter code. */
extern const field32_t DCMI_ESCR_LSC;       /** @brief Line start delimiter code. */
extern const field32_t DCMI_ESCR_FSC;       /** @brief Frame start delimiter code. */
extern const field32_t DCMI_ESUR_FEU;       /** @brief Frame end delimiter unmask. */
extern const field32_t DCMI_ESUR_LEU;       /** @brief Line end delimiter unmask. */
extern const field32_t DCMI_ESUR_LSU;       /** @brief Line start delimiter unmask. */
extern const field32_t DCMI_ESUR_FSU;       /** @brief Frame start delimiter unmask. */
extern const field32_t DCMI_CWSTRT_VST;     /** @brief Vertical start line count. */
extern const field32_t DCMI_CWSTRT_HOFFCNT; /** @brief Horizontal offset count. */
extern const field32_t DCMI_CWSIZE_VLINE;   /** @brief Vertical line count. */
extern const field32_t DCMI_CWSIZE_CAPCNT;  /** @brief Capture count. */

/** @subsection Enumerated DCMI Register Field Definitions */

extern const field32_t DCMI_DR_BYTEx[4]; /** @brief Data byte 0. */

/**************************************************************************************************
 * @section OTGx_HS_GLOBAL Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_GLOBAL Register Definitions */

extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GOTGCTL[3];         /** @brief OTG_HS control and status register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GOTGINT[3];         /** @brief OTG_HS interrupt register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GAHBCFG[3];         /** @brief OTG_HS AHB configuration register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GUSBCFG[3];         /** @brief OTG_HS USB configuration register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRSTCTL[3];         /** @brief OTG_HS reset register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GINTSTS[3];         /** @brief OTG_HS core interrupt register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GINTMSK[3];         /** @brief OTG_HS interrupt mask register. */
extern ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST[3];    /** @brief OTG_HS receive status debug read register (host mode). */
extern ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST[3];    /** @brief OTG_HS status read and pop register (host mode). */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXFSIZ[3];         /** @brief OTG_HS receive FIFO size register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_HNPTXFSIZ_HOST[3];  /** @brief OTG_HS nonperiodic transmit FIFO size register (host mode). */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_DIEPTXF0_DEVICE[3]; /** @brief OTG_HS nonperiodic transmit FIFO size register (host mode). */
extern ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS[3];        /** @brief OTG_HS nonperiodic transmit fifo/queue status register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GCCFG[3];           /** @brief OTG_HS general core configuration register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_CID[3];             /** @brief OTG_HS core ID register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_HPTXFSIZ[3];        /** @brief OTG_HS host periodic transmit FIFO size register. */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_DIEPTXFx[3][8];     /** @brief OTG_HS device IN endpoint transmit FIFO size register. */
extern ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE[3];  /** @brief OTG_HS receive status debug read register (host mode). */
extern ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE[3];  /** @brief OTG_HS status read and pop register (host mode). */
extern rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GLPMCFG[3];         /** @brief OTG core LPM configuration register. */

/** @subsection OTGx_HS_GLOBAL Register Field Definitions */

extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_SRQSCS;            /** @brief Session request success. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_SRQ;               /** @brief Session request. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_HNGSCS;            /** @brief Host negotiation success. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_HNPRQ;             /** @brief HNP request. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_HSHNPEN;           /** @brief Host set HNP enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_DHNPEN;            /** @brief Device HNP enabled. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_CIDSTS;            /** @brief Connector ID status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_DBCT;              /** @brief Long/short debounce time. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_ASVLD;             /** @brief A-session valid. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_BSVLD;             /** @brief B-session valid. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_EHEN;              /** @brief Embedded host enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_SEDET;             /** @brief Session end detected. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_SRSSCHG;           /** @brief Session request success status change. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_HNSSCHG;           /** @brief Host negotiation success status change. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_HNGDET;            /** @brief Host negotiation detected. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_ADTOCHG;           /** @brief A-device timeout change. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_DBCDNE;            /** @brief Debounce done. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_IDCHNG;            /** @brief ID input pin changed. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_GINT;              /** @brief Global interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_HBSTLEN;           /** @brief Burst length/type. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_DMAEN;             /** @brief DMA enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_TXFELVL;           /** @brief TxFIFO empty level. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_PTXFELVL;          /** @brief Periodic txfifo empty level. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_TOCAL;             /** @brief FS timeout calibration. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PHYSEL;            /** @brief USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_SRPCAP;            /** @brief SRP-capable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_HNPCAP;            /** @brief HNP-capable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_TRDT;              /** @brief USB turnaround time. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PHYLPCS;           /** @brief PHY low-power clock select. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIFSLS;          /** @brief ULPI FS/LS select. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIAR;            /** @brief ULPI auto-resume. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPICSM;           /** @brief ULPI clock suspendm. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIEVBUSD;        /** @brief ULPI external VBUS drive. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIEVBUSI;        /** @brief ULPI external VBUS indicator. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_TSDPS;             /** @brief TermSel dline pulsing selection. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PCCI;              /** @brief Indicator complement. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PTCI;              /** @brief Indicator pass through. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIIPD;           /** @brief ULPI interface protect disable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_FHMOD;             /** @brief Forced host mode. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_FDMOD;             /** @brief Forced peripheral mode. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_CSRST;             /** @brief Core soft reset. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_HSRST;             /** @brief HCLK soft reset. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_FCRST;             /** @brief Host frame counter reset. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_RXFFLSH;           /** @brief RxFIFO flush. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_TXFFLSH;           /** @brief TxFIFO flush. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_TXFNUM;            /** @brief TxFIFO number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_AHBIDL;            /** @brief AHB master idle. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_DMAREQ;            /** @brief DMA request signal enabled for USB OTG HS. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_CMOD;              /** @brief Current mode of operation. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_MMIS;              /** @brief Mode mismatch interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_OTGINT;            /** @brief OTG interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_SOF;               /** @brief Start of frame. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_RXFLVL;            /** @brief RxFIFO nonempty. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_NPTXFE;            /** @brief Nonperiodic txfifo empty. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_GINAKEFF;          /** @brief Global IN nonperiodic NAK effective. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_BOUTNAKEFF;        /** @brief Global OUT NAK effective. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_ESUSP;             /** @brief Early suspend. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_USBSUSP;           /** @brief USB suspend. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_USBRST;            /** @brief USB reset. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_ENUMDNE;           /** @brief Enumeration done. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_ISOODRP;           /** @brief Isochronous OUT packet dropped interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_EOPF;              /** @brief End of periodic frame interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_IEPINT;            /** @brief IN endpoint interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_OEPINT;            /** @brief OUT endpoint interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_IISOIXFR;          /** @brief Incomplete isochronous IN transfer. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_PXFR_INCOMPISOOUT; /** @brief Incomplete periodic transfer. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_DATAFSUSP;         /** @brief Data fetch suspended. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_HPRTINT;           /** @brief Host port interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_HCINT;             /** @brief Host channels interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_PTXFE;             /** @brief Periodic txfifo empty. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_CIDSCHG;           /** @brief Connector ID status change. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_DISCINT;           /** @brief Disconnect detected interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_SRQINT;            /** @brief Session request/new session detected interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_WKUINT;            /** @brief Resume/remote wakeup detected interrupt. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_MMISM;             /** @brief Mode mismatch interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_OTGINT;            /** @brief OTG interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_SOFM;              /** @brief Start of frame mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_RXFLVLM;           /** @brief Receive FIFO nonempty mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_NPTXFEM;           /** @brief Nonperiodic txfifo empty mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_GINAKEFFM;         /** @brief Global nonperiodic IN NAK effective mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_GONAKEFFM;         /** @brief Global OUT NAK effective mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_ESUSPM;            /** @brief Early suspend mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_USBSUSPM;          /** @brief USB suspend mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_USBRST;            /** @brief USB reset mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_ENUMDNEM;          /** @brief Enumeration done mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_ISOODRPM;          /** @brief Isochronous OUT packet dropped interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_EOPFM;             /** @brief End of periodic frame interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_IEPINT;            /** @brief IN endpoints interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_OEPINT;            /** @brief OUT endpoints interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_IISOIXFRM;         /** @brief Incomplete isochronous IN transfer mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_PXFRM_IISOOXFRM;   /** @brief Incomplete periodic transfer mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_FSUSPM;            /** @brief Data fetch suspended mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_PRTIM;             /** @brief Host port interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_HCIM;              /** @brief Host channels interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_PTXFEM;            /** @brief Periodic txfifo empty mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_CIDSCHGM;          /** @brief Connector ID status change mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_DISCINT;           /** @brief Disconnect detected interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_SRQIM;             /** @brief Session request/new session detected interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_WUIM;              /** @brief Resume/remote wakeup detected interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_RSTDE;             /** @brief Reset detected interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_LPMINTM;           /** @brief LPM interrupt mask. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_CHNUM;        /** @brief Channel number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_BCNT;         /** @brief Byte count. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_DPID;         /** @brief Data PID. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_PKTSTS;       /** @brief Packet status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_CHNUM;        /** @brief Channel number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_BCNT;         /** @brief Byte count. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_DPID;         /** @brief Data PID. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_PKTSTS;       /** @brief Packet status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXFSIZ_RXFD;              /** @brief RxFIFO depth. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_HNPTXFSIZ_HOST_NPTXFSA;    /** @brief Nonperiodic transmit RAM start address. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_HNPTXFSIZ_HOST_NPTXFD;     /** @brief Nonperiodic txfifo depth. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXF0_DEVICE_TX0FSA;    /** @brief Endpoint 0 transmit RAM start address. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXF0_DEVICE_TX0FD;     /** @brief Endpoint 0 txfifo depth. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS_NPTXFSAV;         /** @brief Nonperiodic txfifo space available. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS_NPTQXSAV;         /** @brief Nonperiodic transmit request queue space available. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS_NPTXQTOP;         /** @brief Top of the nonperiodic transmit request queue. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PWRDWN;              /** @brief Power down. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_BCDEN;               /** @brief Battery charging detector (BCD) enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_DCDEN;               /** @brief Data contact detection (DCD) mode enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PDEN;                /** @brief Primary detection (PD) mode enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_SDEN;                /** @brief Secondary detection (SD) mode enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_VBDEN;               /** @brief USB VBUS detection enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_DCDET;               /** @brief Data contact detection (DCD) status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PDET;                /** @brief Primary detection (PD) status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_SDET;                /** @brief Secondary detection (SD) status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PS2DET;              /** @brief DM pull-up detection status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_HPTXFSIZ_PTXSA;            /** @brief Host periodic txfifo start address. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_HPTXFSIZ_PTXFD;            /** @brief Host periodic txfifo depth. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXFx_INEPTXSA;         /** @brief IN endpoint fifox transmit RAM start address. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXFx_INEPTXFD;         /** @brief IN endpoint txfifo depth. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_EPNUM;      /** @brief Endpoint number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_BCNT;       /** @brief Byte count. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_DPID;       /** @brief Data PID. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_PKTSTS;     /** @brief Packet status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_FRMNUM;     /** @brief Frame number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_EPNUM;      /** @brief Endpoint number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_BCNT;       /** @brief Byte count. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_DPID;       /** @brief Data PID. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_PKTSTS;     /** @brief Packet status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_FRMNUM;     /** @brief Frame number. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMEN;             /** @brief LPM support enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMACK;            /** @brief LPM token acknowledge enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_BESL;              /** @brief Best effort service latency. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_REMWAKE;           /** @brief BRemoteWake value. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_L1SSEN;            /** @brief L1 shallow sleep enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_BESLTHRS;          /** @brief BESL threshold. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_L1DSEN;            /** @brief L1 deep sleep enable. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMRST;            /** @brief LPM response. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_SLPSTS;            /** @brief Port sleep status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_L1RSMOK;           /** @brief Sleep state resume OK. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMCHIDX;          /** @brief LPM channel index. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMRCNT;           /** @brief LPM retry count. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_SNDLPM;            /** @brief Send LPM transaction. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMRCNTSTS;        /** @brief LPM retry count status. */
extern const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_ENBESL;            /** @brief Enable best effort service latency. */

/**************************************************************************************************
 * @section OTGx_HS_HOST Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_HOST Register Definitions */

extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCFG[3];        /** @brief OTG_HS host configuration register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HFIR[3];        /** @brief OTG_HS host frame interval register. */
extern ro_reg32_t const OTGx_HS_HOST_OTG_HS_HFNUM[3];       /** @brief OTG_HS host frame number/frame time remaining register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HPTXSTS[3];     /** @brief OTG_HS_Host periodic transmit fifo/queue status register. */
extern ro_reg32_t const OTGx_HS_HOST_OTG_HS_HAINT[3];       /** @brief OTG_HS host all channels interrupt register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HAINTMSK[3];    /** @brief OTG_HS host all channels interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HPRT[3];        /** @brief OTG_HS host port control and status register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCCHARx[3][16]; /** @brief OTG_HS host channel-0 characteristics register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCSPLTx[3][16]; /** @brief OTG_HS host channel-0 split control register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTx[3][16];  /** @brief OTG_HS host channel-11 interrupt register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK0[3];   /** @brief OTG_HS host channel-11 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK1[3];   /** @brief OTG_HS host channel-1 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK2[3];   /** @brief OTG_HS host channel-2 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK3[3];   /** @brief OTG_HS host channel-3 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK4[3];   /** @brief OTG_HS host channel-4 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK5[3];   /** @brief OTG_HS host channel-5 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK6[3];   /** @brief OTG_HS host channel-6 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK7[3];   /** @brief OTG_HS host channel-7 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK8[3];   /** @brief OTG_HS host channel-8 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK9[3];   /** @brief OTG_HS host channel-9 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK10[3];  /** @brief OTG_HS host channel-10 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK11[3];  /** @brief OTG_HS host channel-11 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCTSIZx[3][16]; /** @brief OTG_HS host channel-11 transfer size register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCDMAx[3][16];  /** @brief OTG_HS host channel-0 DMA address register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK12[3];  /** @brief OTG_HS host channel-12 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK13[3];  /** @brief OTG_HS host channel-13 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK14[3];  /** @brief OTG_HS host channel-14 interrupt mask register. */
extern rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK15[3];  /** @brief OTG_HS host channel-15 interrupt mask register. */

/** @subsection OTGx_HS_HOST Register Field Definitions */

extern const field32_t OTGx_HS_HOST_OTG_HS_HCFG_FSLSPCS;      /** @brief FS/LS PHY clock select. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCFG_FSLSS;        /** @brief FS- and ls-only support. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HFIR_FRIVL;        /** @brief Frame interval. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HFNUM_FRNUM;       /** @brief Frame number. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HFNUM_FTREM;       /** @brief Frame time remaining. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPTXSTS_PTXFSAVL;  /** @brief Periodic transmit data FIFO space available. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPTXSTS_PTXQSAV;   /** @brief Periodic transmit request queue space available. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPTXSTS_PTXQTOP;   /** @brief Top of the periodic transmit request queue. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HAINT_HAINT;       /** @brief Channel interrupts. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HAINTMSK_HAINTM;   /** @brief Channel interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PCSTS;        /** @brief Port connect status. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PCDET;        /** @brief Port connect detected. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PENA;         /** @brief Port enable. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PENCHNG;      /** @brief Port enable/disable change. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_POCA;         /** @brief Port overcurrent active. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_POCCHNG;      /** @brief Port overcurrent change. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PRES;         /** @brief Port resume. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PSUSP;        /** @brief Port suspend. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PRST;         /** @brief Port reset. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PLSTS;        /** @brief Port line status. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PPWR;         /** @brief Port power. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PTCTL;        /** @brief Port test control. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PSPD;         /** @brief Port speed. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_MPSIZ;     /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_EPNUM;     /** @brief Endpoint number. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_EPDIR;     /** @brief Endpoint direction. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_LSDEV;     /** @brief Low-speed device. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_EPTYP;     /** @brief Endpoint type. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_MC;        /** @brief Multi count (MC) / error count (EC). */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_DAD;       /** @brief Device address. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_ODDFRM;    /** @brief Odd frame. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_CHDIS;     /** @brief Channel disable. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_CHENA;     /** @brief Channel enable. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_PRTADDR;   /** @brief Port address. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_HUBADDR;   /** @brief Hub address. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_XACTPOS;   /** @brief XACTPOS. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_COMPLSPLT; /** @brief Do complete split. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_SPLITEN;   /** @brief Split enable. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_XFRC;       /** @brief Transfer completed. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_CHH;        /** @brief Channel halted. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_AHBERR;     /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_STALL;      /** @brief STALL response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_NAK;        /** @brief NAK response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_ACK;        /** @brief ACK response received/transmitted interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_NYET;       /** @brief Response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_TXERR;      /** @brief Transaction error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_BBERR;      /** @brief Babble error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_FRMOR;      /** @brief Frame overrun. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_DTERR;      /** @brief Data toggle error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_XFRCM;   /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_CHHM;    /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_AHBERR;  /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_STALLM;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_NAKM;    /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_ACKM;    /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_NYET;    /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_TXERRM;  /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_BBERRM;  /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_FRMORM;  /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_DTERRM;  /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_XFRCM;  /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_CHHM;   /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_AHBERR; /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_STALLM; /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_NAKM;   /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_ACKM;   /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_NYET;   /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_TXERRM; /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_BBERRM; /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_FRMORM; /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_DTERRM; /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_XFRCM;  /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_CHHM;   /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_AHBERR; /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_STALLM; /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_NAKM;   /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_ACKM;   /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_NYET;   /** @brief Response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_TXERRM; /** @brief Transaction error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_BBERRM; /** @brief Babble error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_FRMORM; /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_DTERRM; /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCTSIZx_XFRSIZ;    /** @brief Transfer size. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCTSIZx_PKTCNT;    /** @brief Packet count. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCTSIZx_DPID;      /** @brief Data PID. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_XFRCM;  /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_CHHM;   /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_AHBERR; /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_STALLM; /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_NAKM;   /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_ACKM;   /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_NYET;   /** @brief Response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_TXERRM; /** @brief Transaction error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_BBERRM; /** @brief Babble error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_FRMORM; /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_DTERRM; /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_XFRCM;  /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_CHHM;   /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_AHBERR; /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_STALLM; /** @brief STALLM response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_NAKM;   /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_ACKM;   /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_NYET;   /** @brief Response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_TXERRM; /** @brief Transaction error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_BBERRM; /** @brief Babble error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_FRMORM; /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_DTERRM; /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_XFRCM;  /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_CHHM;   /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_AHBERR; /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_STALLM; /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_NAKM;   /** @brief NAKM response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_ACKM;   /** @brief ACKM response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_NYET;   /** @brief Response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_TXERRM; /** @brief Transaction error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_BBERRM; /** @brief Babble error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_FRMORM; /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_DTERRM; /** @brief Data toggle error mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_XFRCM;  /** @brief Transfer completed mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_CHHM;   /** @brief Channel halted mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_AHBERR; /** @brief AHB error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_STALL;  /** @brief STALL response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_NAKM;   /** @brief NAK response received interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_ACKM;   /** @brief ACK response received/transmitted interrupt mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_NYET;   /** @brief Response received interrupt. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_TXERRM; /** @brief Transaction error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_BBERRM; /** @brief Babble error. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_FRMORM; /** @brief Frame overrun mask. */
extern const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_DTERRM; /** @brief Data toggle error mask. */

/**************************************************************************************************
 * @section OTGx_HS_DEVICE Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_DEVICE Register Definitions */

extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DCFG[3];        /** @brief OTG_HS device configuration register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DCTL[3];        /** @brief OTG_HS device control register. */
extern ro_reg32_t const OTGx_HS_DEVICE_OTG_HS_DSTS[3];        /** @brief OTG_HS device status register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPMSK[3];     /** @brief OTG_HS device IN endpoint common interrupt mask register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPMSK[3];     /** @brief OTG_HS device OUT endpoint common interrupt mask register. */
extern ro_reg32_t const OTGx_HS_DEVICE_OTG_HS_DAINT[3];       /** @brief OTG_HS device all endpoints interrupt register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DAINTMSK[3];    /** @brief OTG_HS all endpoints interrupt mask register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DVBUSDIS[3];    /** @brief OTG_HS device VBUS discharge time register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DVBUSPULSE[3];  /** @brief OTG_HS device VBUS pulsing time register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DTHRCTL[3];     /** @brief OTG_HS device threshold control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPEMPMSK[3];  /** @brief OTG_HS device IN endpoint FIFO empty interrupt mask register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DEACHINT[3];    /** @brief OTG_HS device each endpoint interrupt register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DEACHINTMSK[3]; /** @brief OTG_HS device each endpoint interrupt register mask. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPCTLx[3][8]; /** @brief OTG device endpoint-0 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPINTx[3][8]; /** @brief OTG device endpoint-0 interrupt register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ0[3];   /** @brief OTG_HS device IN endpoint 0 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPDMAx[3][6]; /** @brief OTG_HS device endpoint-1 DMA address register. */
extern ro_reg32_t const OTGx_HS_DEVICE_OTG_HS_DTXFSTSx[3][8]; /** @brief OTG_HS device IN endpoint transmit FIFO status register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1[3];   /** @brief OTG_HS device endpoint transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2[3];   /** @brief OTG_HS device endpoint transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3[3];   /** @brief OTG_HS device endpoint transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4[3];   /** @brief OTG_HS device endpoint transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5[3];   /** @brief OTG_HS device endpoint transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL0[3];    /** @brief OTG_HS device control OUT endpoint 0 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL1[3];    /** @brief OTG device endpoint-1 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL2[3];    /** @brief OTG device endpoint-2 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL3[3];    /** @brief OTG device endpoint-3 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPINTx[3][8]; /** @brief OTG_HS device endpoint-0 interrupt register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0[3];   /** @brief OTG_HS device endpoint-0 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1[3];   /** @brief OTG_HS device endpoint-1 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2[3];   /** @brief OTG_HS device endpoint-2 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3[3];   /** @brief OTG_HS device endpoint-3 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4[3];   /** @brief OTG_HS device endpoint-4 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6[3];   /** @brief OTG device endpoint-5 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7[3];   /** @brief OTG device endpoint-5 interrupt register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL4[3];    /** @brief OTG device endpoint-4 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL5[3];    /** @brief OTG device endpoint-5 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL6[3];    /** @brief OTG device endpoint-6 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL7[3];    /** @brief OTG device endpoint-7 control register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5[3];   /** @brief OTG_HS device endpoint-5 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6[3];   /** @brief OTG_HS device endpoint-6 transfer size register. */
extern rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7[3];   /** @brief OTG_HS device endpoint-7 transfer size register. */

/** @subsection OTGx_HS_DEVICE Register Field Definitions */

extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_DSPD;                /** @brief Device speed. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_NZLSOHSK;            /** @brief Nonzero-length status OUT handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_DAD;                 /** @brief Device address. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_PFIVL;               /** @brief Periodic (micro)frame interval. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_PERSCHIVL;           /** @brief Periodic scheduling interval. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_RWUSIG;              /** @brief Remote wakeup signaling. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_SDIS;                /** @brief Soft disconnect. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_GINSTS;              /** @brief Global IN NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_GONSTS;              /** @brief Global OUT NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_TCTL;                /** @brief Test control. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_SGINAK;              /** @brief Set global IN NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_CGINAK;              /** @brief Clear global IN NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_SGONAK;              /** @brief Set global OUT NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_CGONAK;              /** @brief Clear global OUT NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_POPRGDNE;            /** @brief Power-on programming done. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_SUSPSTS;             /** @brief Suspend status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_ENUMSPD;             /** @brief Enumerated speed. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_EERR;                /** @brief Erratic error. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_FNSOF;               /** @brief Frame number of the received SOF. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_XFRCM;            /** @brief Transfer completed interrupt mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_EPDM;             /** @brief Endpoint disabled interrupt mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_TOM;              /** @brief Timeout condition mask (nonisochronous endpoints). */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_ITTXFEMSK;        /** @brief IN token received when txfifo empty mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_INEPNMM;          /** @brief IN token received with EP mismatch mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_INEPNEM;          /** @brief IN endpoint NAK effective mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_TXFURM;           /** @brief FIFO underrun mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_BIM;              /** @brief BNA interrupt mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_XFRCM;            /** @brief Transfer completed interrupt mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_EPDM;             /** @brief Endpoint disabled interrupt mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_STUPM;            /** @brief SETUP phase done mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_OTEPDM;           /** @brief OUT token received when endpoint disabled mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_B2BSTUP;          /** @brief Back-to-back SETUP packets received mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_OPEM;             /** @brief OUT packet error mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_BOIM;             /** @brief BNA interrupt mask. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DAINT_IEPINT;             /** @brief IN endpoint interrupt bits. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DAINT_OEPINT;             /** @brief OUT endpoint interrupt bits. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DAINTMSK_IEPM;            /** @brief IN EP interrupt mask bits. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DAINTMSK_OEPM;            /** @brief OUT EP interrupt mask bits. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DVBUSDIS_VBUSDT;          /** @brief Device VBUS discharge time. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DVBUSPULSE_DVBUSP;        /** @brief Device VBUS pulsing time. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_NONISOTHREN;      /** @brief Nonisochronous IN endpoints threshold enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_ISOTHREN;         /** @brief ISO IN endpoint threshold enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_TXTHRLEN;         /** @brief Transmit threshold length. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_RXTHREN;          /** @brief Receive threshold enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_RXTHRLEN;         /** @brief Receive threshold length. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_ARPEN;            /** @brief Arbiter parking enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPEMPMSK_INEPTXFEM;     /** @brief IN EP tx FIFO empty interrupt mask bits. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINT_IEP1INT;         /** @brief IN endpoint 1interrupt bit. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINT_OEP1INT;         /** @brief OUT endpoint 1 interrupt bit. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINTMSK_IEP1INTM;     /** @brief IN endpoint 1 interrupt mask bit. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINTMSK_OEP1INTM;     /** @brief OUT endpoint 1 interrupt mask bit. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EONUM_DPID;      /** @brief Even/odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_TXFNUM;          /** @brief TxFIFO number. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_SD0PID_SEVNFRM;  /** @brief Set DATA0 PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_XFRC;            /** @brief Transfer completed interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_EPDISD;          /** @brief Endpoint disabled interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_TOC;             /** @brief Timeout condition. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_ITTXFE;          /** @brief IN token received when txfifo is empty. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_INEPNE;          /** @brief IN endpoint NAK effective. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_TXFE;            /** @brief Transmit FIFO empty. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_TXFIFOUDRN;      /** @brief Transmit fifo underrun. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_BNA;             /** @brief Buffer not available interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_PKTDRPSTS;       /** @brief Packet dropped status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_BERR;            /** @brief Babble error interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_NAK;             /** @brief NAK interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ0_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ0_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DTXFSTSx_INEPTFSAV;       /** @brief IN endpoint txfifo space avail. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_XFRC;            /** @brief Transfer completed interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_EPDISD;          /** @brief Endpoint disabled interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_STUP;            /** @brief SETUP phase done. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_OTEPDIS;         /** @brief OUT token received when endpoint disabled. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_B2BSTUP;         /** @brief Back-to-back SETUP packets received. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_NYET;            /** @brief NYET interrupt. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0_STUPCNT;        /** @brief SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7_MCNT;           /** @brief Multi count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_MPSIZ;           /** @brief Maximum packet size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_USBAEP;          /** @brief USB active endpoint. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EONUM_DPID;      /** @brief Even odd frame/endpoint data PID. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_NAKSTS;          /** @brief NAK status. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EPTYP;           /** @brief Endpoint type. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SNPM;            /** @brief Snoop mode. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_STALL;           /** @brief STALL handshake. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_CNAK;            /** @brief Clear NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SNAK;            /** @brief Set NAK. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SD0PID_SEVNFRM;  /** @brief Set DATA0 pid/set even frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SODDFRM;         /** @brief Set odd frame. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EPDIS;           /** @brief Endpoint disable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EPENA;           /** @brief Endpoint enable. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7_XFRSIZ;         /** @brief Transfer size. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7_PKTCNT;         /** @brief Packet count. */
extern const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7_RXDPID_STUPCNT; /** @brief Received data PID/SETUP packet count. */

/**************************************************************************************************
 * @section OTGx_HS_PWRCLK Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_PWRCLK Register Definitions */

extern rw_reg32_t const OTGx_HS_PWRCLK_OTG_HS_PCGCR[3]; /** @brief Power and clock gating control register. */

/** @subsection OTGx_HS_PWRCLK Register Field Definitions */

extern const field32_t OTGx_HS_PWRCLK_OTG_HS_PCGCR_STPPCLK;  /** @brief Stop PHY clock. */
extern const field32_t OTGx_HS_PWRCLK_OTG_HS_PCGCR_GATEHCLK; /** @brief Gate HCLK. */
extern const field32_t OTGx_HS_PWRCLK_OTG_HS_PCGCR_PHYSUSP;  /** @brief PHY suspended. */

/**************************************************************************************************
 * @section ETHERNET_MAC Definitions
 **************************************************************************************************/

/** @subsection ETHERNET_MAC Register Definitions */

extern rw_reg32_t const ETHERNET_MAC_DMAMR;                              /** @brief DMA mode register. */
extern rw_reg32_t const ETHERNET_MAC_DMASBMR;                            /** @brief System bus mode register. */
extern ro_reg32_t const ETHERNET_MAC_DMAISR;                             /** @brief Interrupt status register. */
extern ro_reg32_t const ETHERNET_MAC_DMADSR;                             /** @brief Debug status register. */
extern rw_reg32_t const ETHERNET_MAC_DMACCR;                             /** @brief Channel control register. */
extern rw_reg32_t const ETHERNET_MAC_DMACTXCR;                           /** @brief Channel transmit control register. */
extern rw_reg32_t const ETHERNET_MAC_DMACRXCR;                           /** @brief Channel receive control register. */
extern rw_reg32_t const ETHERNET_MAC_DMACTXDLAR;                         /** @brief Channel tx descriptor list address register. */
extern rw_reg32_t const ETHERNET_MAC_DMACRXDLAR;                         /** @brief Channel rx descriptor list address register. */
extern rw_reg32_t const ETHERNET_MAC_DMACTXDTPR;                         /** @brief Channel tx descriptor tail pointer register. */
extern rw_reg32_t const ETHERNET_MAC_DMACRXDTPR;                         /** @brief Channel rx descriptor tail pointer register. */
extern rw_reg32_t const ETHERNET_MAC_DMACTXRLR;                          /** @brief Channel tx descriptor ring length register. */
extern rw_reg32_t const ETHERNET_MAC_DMACRXRLR;                          /** @brief Channel rx descriptor ring length register. */
extern rw_reg32_t const ETHERNET_MAC_DMACIER;                            /** @brief Channel interrupt enable register. */
extern rw_reg32_t const ETHERNET_MAC_DMACRXIWTR;                         /** @brief Channel rx interrupt watchdog timer register. */
extern ro_reg32_t const ETHERNET_MAC_DMACCATXDR;                         /** @brief Channel current application transmit descriptor register. */
extern ro_reg32_t const ETHERNET_MAC_DMACCARXDR;                         /** @brief Channel current application receive descriptor register. */
extern ro_reg32_t const ETHERNET_MAC_DMACCATXBR;                         /** @brief Channel current application transmit buffer register. */
extern ro_reg32_t const ETHERNET_MAC_DMACCARXBR;                         /** @brief Channel current application receive buffer register. */
extern rw_reg32_t const ETHERNET_MAC_DMACSR;                             /** @brief Channel status register. */
extern ro_reg32_t const ETHERNET_MAC_DMACMFCR;                           /** @brief Channel missed frame count register. */
extern rw_reg32_t const ETHERNET_MAC_MTLOMR;                             /** @brief Operating mode register. */
extern ro_reg32_t const ETHERNET_MAC_MTLISR;                             /** @brief Interrupt status register. */
extern rw_reg32_t const ETHERNET_MAC_MTLTXQOMR;                          /** @brief Tx queue operating mode register. */
extern ro_reg32_t const ETHERNET_MAC_MTLTXQUR;                           /** @brief Tx queue underflow register. */
extern ro_reg32_t const ETHERNET_MAC_MTLTXQDR;                           /** @brief Tx queue debug register. */
extern rw_reg32_t const ETHERNET_MAC_MTLQICSR;                           /** @brief Queue interrupt control status register. */
extern rw_reg32_t const ETHERNET_MAC_MTLRXQOMR;                          /** @brief Rx queue operating mode register. */
extern ro_reg32_t const ETHERNET_MAC_MTLRXQMPOCR;                        /** @brief Rx queue missed packet and overflow counter register. */
extern ro_reg32_t const ETHERNET_MAC_MTLRXQDR;                           /** @brief Rx queue debug register. */
extern rw_reg32_t const ETHERNET_MAC_MACCR;                              /** @brief Operating mode configuration register. */
extern rw_reg32_t const ETHERNET_MAC_MACECR;                             /** @brief Extended operating mode configuration register. */
extern rw_reg32_t const ETHERNET_MAC_MACPFR;                             /** @brief Packet filtering control register. */
extern rw_reg32_t const ETHERNET_MAC_MACWTR;                             /** @brief Watchdog timeout register. */
extern rw_reg32_t const ETHERNET_MAC_MACHT0R;                            /** @brief Hash table 0 register. */
extern rw_reg32_t const ETHERNET_MAC_MACHT1R;                            /** @brief Hash table 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACVTR;                             /** @brief VLAN tag register. */
extern rw_reg32_t const ETHERNET_MAC_MACVHTR;                            /** @brief VLAN hash table register. */
extern rw_reg32_t const ETHERNET_MAC_MACVIR;                             /** @brief VLAN inclusion register. */
extern rw_reg32_t const ETHERNET_MAC_MACIVIR;                            /** @brief Inner VLAN inclusion register. */
extern rw_reg32_t const ETHERNET_MAC_MACQTXFCR;                          /** @brief Tx queue flow control register. */
extern rw_reg32_t const ETHERNET_MAC_MACRXFCR;                           /** @brief Rx flow control register. */
extern ro_reg32_t const ETHERNET_MAC_MACISR;                             /** @brief Interrupt status register. */
extern rw_reg32_t const ETHERNET_MAC_MACIER;                             /** @brief Interrupt enable register. */
extern ro_reg32_t const ETHERNET_MAC_MACRXTXSR;                          /** @brief Rx tx status register. */
extern rw_reg32_t const ETHERNET_MAC_MACPCSR;                            /** @brief PMT control status register. */
extern rw_reg32_t const ETHERNET_MAC_MACRWKPFR;                          /** @brief Remove wakeup packet filter register. */
extern rw_reg32_t const ETHERNET_MAC_MACLCSR;                            /** @brief LPI control status register. */
extern rw_reg32_t const ETHERNET_MAC_MACLTCR;                            /** @brief LPI timers control register. */
extern rw_reg32_t const ETHERNET_MAC_MACLETR;                            /** @brief LPI entry timer register. */
extern rw_reg32_t const ETHERNET_MAC_MAC1USTCR;                          /** @brief 1-microsecond-tick counter register. */
extern ro_reg32_t const ETHERNET_MAC_MACVR;                              /** @brief Version register. */
extern ro_reg32_t const ETHERNET_MAC_MACHWF1R;                           /** @brief HW feature 1 register. */
extern ro_reg32_t const ETHERNET_MAC_MACHWF2R;                           /** @brief HW feature 2 register. */
extern rw_reg32_t const ETHERNET_MAC_MACMDIOAR;                          /** @brief MDIO address register. */
extern rw_reg32_t const ETHERNET_MAC_MACMDIODR;                          /** @brief MDIO data register. */
extern rw_reg32_t const ETHERNET_MAC_MACARPAR;                           /** @brief ARP address register. */
extern rw_reg32_t const ETHERNET_MAC_MACA0HR;                            /** @brief Address 0 high register. */
extern rw_reg32_t const ETHERNET_MAC_MACA1HR;                            /** @brief Address 1 high register. */
extern rw_reg32_t const ETHERNET_MAC_MACA2HR;                            /** @brief Address 2 high register. */
extern rw_reg32_t const ETHERNET_MAC_MACA3HR;                            /** @brief Address 3 high register. */
extern rw_reg32_t const ETHERNET_MAC_MMC_CONTROL;                        /** @brief MMC control register. */
extern ro_reg32_t const ETHERNET_MAC_MMC_RX_INTERRUPT;                   /** @brief MMC rx interrupt register. */
extern ro_reg32_t const ETHERNET_MAC_MMC_TX_INTERRUPT;                   /** @brief MMC tx interrupt register. */
extern rw_reg32_t const ETHERNET_MAC_MMC_RX_INTERRUPT_MASK;              /** @brief MMC rx interrupt mask register. */
extern rw_reg32_t const ETHERNET_MAC_MMC_TX_INTERRUPT_MASK;              /** @brief MMC tx interrupt mask register. */
extern ro_reg32_t const ETHERNET_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS;   /** @brief Tx single collision good packets register. */
extern ro_reg32_t const ETHERNET_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS; /** @brief Tx multiple collision good packets register. */
extern ro_reg32_t const ETHERNET_MAC_TX_PACKET_COUNT_GOOD;               /** @brief Tx packet count good register. */
extern ro_reg32_t const ETHERNET_MAC_RX_CRC_ERROR_PACKETS;               /** @brief Rx CRC error packets register. */
extern ro_reg32_t const ETHERNET_MAC_RX_ALIGNMENT_ERROR_PACKETS;         /** @brief Rx alignment error packets register. */
extern ro_reg32_t const ETHERNET_MAC_RX_UNICAST_PACKETS_GOOD;            /** @brief Rx unicast packets good register. */
extern ro_reg32_t const ETHERNET_MAC_TX_LPI_USEC_CNTR;                   /** @brief Tx LPI microsecond timer register. */
extern ro_reg32_t const ETHERNET_MAC_TX_LPI_TRAN_CNTR;                   /** @brief Tx LPI transition counter register. */
extern ro_reg32_t const ETHERNET_MAC_RX_LPI_USEC_CNTR;                   /** @brief Rx LPI microsecond counter register. */
extern ro_reg32_t const ETHERNET_MAC_RX_LPI_TRAN_CNTR;                   /** @brief Rx LPI transition counter register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3L4C0R;                         /** @brief L3 and L4 control 0 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL4A0R;                           /** @brief Layer4 address filter 0 register. */
extern ro_reg32_t const ETHERNET_MAC_MACDR;                              /** @brief Debug register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A00R;                          /** @brief MACL3A00R. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A10R;                          /** @brief Layer3 address 1 filter 0 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A20;                           /** @brief Layer3 address 2 filter 0 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A30;                           /** @brief Layer3 address 3 filter 0 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3L4C1R;                         /** @brief L3 and L4 control 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL4A1R;                           /** @brief Layer 4 address filter 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A01R;                          /** @brief Layer3 address 0 filter 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A11R;                          /** @brief Layer3 address 1 filter 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A21R;                          /** @brief Layer3 address 2 filter 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACL3A31R;                          /** @brief Layer3 address 3 filter 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACTSCR;                            /** @brief Timestamp control register. */
extern rw_reg32_t const ETHERNET_MAC_MACSSIR;                            /** @brief Sub-second increment register. */
extern ro_reg32_t const ETHERNET_MAC_MACSTSR;                            /** @brief System time seconds register. */
extern ro_reg32_t const ETHERNET_MAC_MACSTNR;                            /** @brief System time nanoseconds register. */
extern rw_reg32_t const ETHERNET_MAC_MACSTSUR;                           /** @brief System time seconds update register. */
extern rw_reg32_t const ETHERNET_MAC_MACSTNUR;                           /** @brief System time nanoseconds update register. */
extern rw_reg32_t const ETHERNET_MAC_MACTSAR;                            /** @brief Timestamp addend register. */
extern ro_reg32_t const ETHERNET_MAC_MACTSSR;                            /** @brief Timestamp status register. */
extern ro_reg32_t const ETHERNET_MAC_MACTXTSSNR;                         /** @brief Tx timestamp status nanoseconds register. */
extern ro_reg32_t const ETHERNET_MAC_MACTXTSSSR;                         /** @brief Tx timestamp status seconds register. */
extern rw_reg32_t const ETHERNET_MAC_MACACR;                             /** @brief Auxiliary control register. */
extern ro_reg32_t const ETHERNET_MAC_MACATSNR;                           /** @brief Auxiliary timestamp nanoseconds register. */
extern ro_reg32_t const ETHERNET_MAC_MACATSSR;                           /** @brief Auxiliary timestamp seconds register. */
extern rw_reg32_t const ETHERNET_MAC_MACTSIACR;                          /** @brief Timestamp ingress asymmetric correction register. */
extern rw_reg32_t const ETHERNET_MAC_MACTSEACR;                          /** @brief Timestamp egress asymmetric correction register. */
extern rw_reg32_t const ETHERNET_MAC_MACTSICNR;                          /** @brief Timestamp ingress correction nanosecond register. */
extern rw_reg32_t const ETHERNET_MAC_MACTSECNR;                          /** @brief Timestamp egress correction nanosecond register. */
extern rw_reg32_t const ETHERNET_MAC_MACPPSCR;                           /** @brief PPS control register. */
extern rw_reg32_t const ETHERNET_MAC_MACPPSTTSR;                         /** @brief PPS target time seconds register. */
extern rw_reg32_t const ETHERNET_MAC_MACPPSTTNR;                         /** @brief PPS target time nanoseconds register. */
extern rw_reg32_t const ETHERNET_MAC_MACPPSIR;                           /** @brief PPS interval register. */
extern rw_reg32_t const ETHERNET_MAC_MACPPSWR;                           /** @brief PPS width register. */
extern rw_reg32_t const ETHERNET_MAC_MACPOCR;                            /** @brief PTP offload control register. */
extern rw_reg32_t const ETHERNET_MAC_MACSPI0R;                           /** @brief PTP source port identity 0 register. */
extern rw_reg32_t const ETHERNET_MAC_MACSPI1R;                           /** @brief PTP source port identity 1 register. */
extern rw_reg32_t const ETHERNET_MAC_MACSPI2R;                           /** @brief PTP source port identity 2 register. */
extern rw_reg32_t const ETHERNET_MAC_MACLMIR;                            /** @brief Log message interval register. */

/** @subsection Enumerated ETHERNET_MAC Register Definitions */

extern rw_reg32_t const ETHERNET_MAC_MACAxLR[4]; /** @brief Address 0 low register. */

/** @subsection ETHERNET_MAC Register Field Definitions */

extern const field32_t ETHERNET_MAC_DMAMR_SWR;                         /** @brief Software reset. */
extern const field32_t ETHERNET_MAC_DMAMR_DA;                          /** @brief DMA tx or rx arbitration scheme. */
extern const field32_t ETHERNET_MAC_DMAMR_TXPR;                        /** @brief Transmit priority. */
extern const field32_t ETHERNET_MAC_DMAMR_PR;                          /** @brief Priority ratio. */
extern const field32_t ETHERNET_MAC_DMAMR_INTM;                        /** @brief Interrupt mode. */
extern const field32_t ETHERNET_MAC_DMASBMR_FB;                        /** @brief Fixed burst length. */
extern const field32_t ETHERNET_MAC_DMASBMR_AAL;                       /** @brief Address-Aligned beats. */
extern const field32_t ETHERNET_MAC_DMASBMR_MB;                        /** @brief Mixed burst. */
extern const field32_t ETHERNET_MAC_DMASBMR_RB;                        /** @brief Rebuild incrx burst. */
extern const field32_t ETHERNET_MAC_DMAISR_DC0IS;                      /** @brief DMA channel interrupt status. */
extern const field32_t ETHERNET_MAC_DMAISR_MTLIS;                      /** @brief MTL interrupt status. */
extern const field32_t ETHERNET_MAC_DMAISR_MACIS;                      /** @brief MAC interrupt status. */
extern const field32_t ETHERNET_MAC_DMADSR_AXWHSTS;                    /** @brief AHB master write channel. */
extern const field32_t ETHERNET_MAC_DMADSR_RPS0;                       /** @brief DMA channel receive process state. */
extern const field32_t ETHERNET_MAC_DMADSR_TPS0;                       /** @brief DMA channel transmit process state. */
extern const field32_t ETHERNET_MAC_DMACCR_MSS;                        /** @brief Maximum segment size. */
extern const field32_t ETHERNET_MAC_DMACCR_PBLX8;                      /** @brief 8xPBL mode. */
extern const field32_t ETHERNET_MAC_DMACCR_DSL;                        /** @brief Descriptor skip length. */
extern const field32_t ETHERNET_MAC_DMACTXCR_ST;                       /** @brief Start or stop transmission command. */
extern const field32_t ETHERNET_MAC_DMACTXCR_OSF;                      /** @brief Operate on second packet. */
extern const field32_t ETHERNET_MAC_DMACTXCR_TSE;                      /** @brief TCP segmentation enabled. */
extern const field32_t ETHERNET_MAC_DMACTXCR_TXPBL;                    /** @brief Transmit programmable burst length. */
extern const field32_t ETHERNET_MAC_DMACRXCR_SR;                       /** @brief Start or stop receive command. */
extern const field32_t ETHERNET_MAC_DMACRXCR_RBSZ;                     /** @brief Receive buffer size. */
extern const field32_t ETHERNET_MAC_DMACRXCR_RXPBL;                    /** @brief RXPBL. */
extern const field32_t ETHERNET_MAC_DMACRXCR_RPF;                      /** @brief DMA rx channel packet flush. */
extern const field32_t ETHERNET_MAC_DMACTXDLAR_TDESLA;                 /** @brief Start of transmit list. */
extern const field32_t ETHERNET_MAC_DMACRXDLAR_RDESLA;                 /** @brief Start of receive list. */
extern const field32_t ETHERNET_MAC_DMACTXDTPR_TDT;                    /** @brief Transmit descriptor tail pointer. */
extern const field32_t ETHERNET_MAC_DMACRXDTPR_RDT;                    /** @brief Receive descriptor tail pointer. */
extern const field32_t ETHERNET_MAC_DMACTXRLR_TDRL;                    /** @brief Transmit descriptor ring length. */
extern const field32_t ETHERNET_MAC_DMACRXRLR_RDRL;                    /** @brief Receive descriptor ring length. */
extern const field32_t ETHERNET_MAC_DMACIER_TIE;                       /** @brief Transmit interrupt enable. */
extern const field32_t ETHERNET_MAC_DMACIER_TXSE;                      /** @brief Transmit stopped enable. */
extern const field32_t ETHERNET_MAC_DMACIER_TBUE;                      /** @brief Transmit buffer unavailable enable. */
extern const field32_t ETHERNET_MAC_DMACIER_RIE;                       /** @brief Receive interrupt enable. */
extern const field32_t ETHERNET_MAC_DMACIER_RBUE;                      /** @brief Receive buffer unavailable enable. */
extern const field32_t ETHERNET_MAC_DMACIER_RSE;                       /** @brief Receive stopped enable. */
extern const field32_t ETHERNET_MAC_DMACIER_RWTE;                      /** @brief Receive watchdog timeout enable. */
extern const field32_t ETHERNET_MAC_DMACIER_ETIE;                      /** @brief Early transmit interrupt enable. */
extern const field32_t ETHERNET_MAC_DMACIER_ERIE;                      /** @brief Early receive interrupt enable. */
extern const field32_t ETHERNET_MAC_DMACIER_FBEE;                      /** @brief Fatal bus error enable. */
extern const field32_t ETHERNET_MAC_DMACIER_CDEE;                      /** @brief Context descriptor error enable. */
extern const field32_t ETHERNET_MAC_DMACIER_AIE;                       /** @brief Abnormal interrupt summary enable. */
extern const field32_t ETHERNET_MAC_DMACIER_NIE;                       /** @brief Normal interrupt summary enable. */
extern const field32_t ETHERNET_MAC_DMACRXIWTR_RWT;                    /** @brief Receive interrupt watchdog timer count. */
extern const field32_t ETHERNET_MAC_DMACSR_TI;                         /** @brief Transmit interrupt. */
extern const field32_t ETHERNET_MAC_DMACSR_TPS;                        /** @brief Transmit process stopped. */
extern const field32_t ETHERNET_MAC_DMACSR_TBU;                        /** @brief Transmit buffer unavailable. */
extern const field32_t ETHERNET_MAC_DMACSR_RI;                         /** @brief Receive interrupt. */
extern const field32_t ETHERNET_MAC_DMACSR_RBU;                        /** @brief Receive buffer unavailable. */
extern const field32_t ETHERNET_MAC_DMACSR_RPS;                        /** @brief Receive process stopped. */
extern const field32_t ETHERNET_MAC_DMACSR_RWT;                        /** @brief Receive watchdog timeout. */
extern const field32_t ETHERNET_MAC_DMACSR_ET;                         /** @brief Early transmit interrupt. */
extern const field32_t ETHERNET_MAC_DMACSR_ER;                         /** @brief Early receive interrupt. */
extern const field32_t ETHERNET_MAC_DMACSR_FBE;                        /** @brief Fatal bus error. */
extern const field32_t ETHERNET_MAC_DMACSR_CDE;                        /** @brief Context descriptor error. */
extern const field32_t ETHERNET_MAC_DMACSR_AIS;                        /** @brief Abnormal interrupt summary. */
extern const field32_t ETHERNET_MAC_DMACSR_NIS;                        /** @brief Normal interrupt summary. */
extern const field32_t ETHERNET_MAC_DMACSR_TEB;                        /** @brief Tx DMA error bits. */
extern const field32_t ETHERNET_MAC_DMACSR_REB;                        /** @brief Rx DMA error bits. */
extern const field32_t ETHERNET_MAC_DMACMFCR_MFC;                      /** @brief Dropped packet counters. */
extern const field32_t ETHERNET_MAC_DMACMFCR_MFCO;                     /** @brief Overflow status of the MFC counter. */
extern const field32_t ETHERNET_MAC_MTLOMR_DTXSTS;                     /** @brief DTXSTS. */
extern const field32_t ETHERNET_MAC_MTLOMR_CNTPRST;                    /** @brief CNTPRST. */
extern const field32_t ETHERNET_MAC_MTLOMR_CNTCLR;                     /** @brief CNTCLR. */
extern const field32_t ETHERNET_MAC_MTLISR_Q0IS;                       /** @brief Queue interrupt status. */
extern const field32_t ETHERNET_MAC_MTLTXQOMR_FTQ;                     /** @brief Flush transmit queue. */
extern const field32_t ETHERNET_MAC_MTLTXQOMR_TSF;                     /** @brief Transmit store and forward. */
extern const field32_t ETHERNET_MAC_MTLTXQOMR_TXQEN;                   /** @brief Transmit queue enable. */
extern const field32_t ETHERNET_MAC_MTLTXQOMR_TTC;                     /** @brief Transmit threshold control. */
extern const field32_t ETHERNET_MAC_MTLTXQOMR_TQS;                     /** @brief Transmit queue size. */
extern const field32_t ETHERNET_MAC_MTLTXQUR_UFFRMCNT;                 /** @brief Underflow packet counter. */
extern const field32_t ETHERNET_MAC_MTLTXQUR_UFCNTOVF;                 /** @brief UFCNTOVF. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_TXQPAUSED;                /** @brief TXQPAUSED. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_TRCSTS;                   /** @brief TRCSTS. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_TWCSTS;                   /** @brief TWCSTS. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_TXQSTS;                   /** @brief TXQSTS. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_TXSTSFSTS;                /** @brief TXSTSFSTS. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_PTXQ;                     /** @brief PTXQ. */
extern const field32_t ETHERNET_MAC_MTLTXQDR_STXSTSF;                  /** @brief STXSTSF. */
extern const field32_t ETHERNET_MAC_MTLQICSR_TXUNFIS;                  /** @brief TXUNFIS. */
extern const field32_t ETHERNET_MAC_MTLQICSR_TXUIE;                    /** @brief TXUIE. */
extern const field32_t ETHERNET_MAC_MTLQICSR_RXOVFIS;                  /** @brief RXOVFIS. */
extern const field32_t ETHERNET_MAC_MTLQICSR_RXOIE;                    /** @brief RXOIE. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_RTC;                     /** @brief RTC. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_FUP;                     /** @brief FUP. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_FEP;                     /** @brief FEP. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_RSF;                     /** @brief RSF. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_DIS_TCP_EF;              /** @brief DIS_TCP_EF. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_EHFC;                    /** @brief EHFC. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_RFA;                     /** @brief RFA. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_RFD;                     /** @brief RFD. */
extern const field32_t ETHERNET_MAC_MTLRXQOMR_RQS;                     /** @brief RQS. */
extern const field32_t ETHERNET_MAC_MTLRXQMPOCR_OVFPKTCNT;             /** @brief OVFPKTCNT. */
extern const field32_t ETHERNET_MAC_MTLRXQMPOCR_OVFCNTOVF;             /** @brief OVFCNTOVF. */
extern const field32_t ETHERNET_MAC_MTLRXQMPOCR_MISPKTCNT;             /** @brief MISPKTCNT. */
extern const field32_t ETHERNET_MAC_MTLRXQMPOCR_MISCNTOVF;             /** @brief MISCNTOVF. */
extern const field32_t ETHERNET_MAC_MTLRXQDR_RWCSTS;                   /** @brief RWCSTS. */
extern const field32_t ETHERNET_MAC_MTLRXQDR_RRCSTS;                   /** @brief RRCSTS. */
extern const field32_t ETHERNET_MAC_MTLRXQDR_RXQSTS;                   /** @brief RXQSTS. */
extern const field32_t ETHERNET_MAC_MTLRXQDR_PRXQ;                     /** @brief PRXQ. */
extern const field32_t ETHERNET_MAC_MACCR_RE;                          /** @brief Receiver enable. */
extern const field32_t ETHERNET_MAC_MACCR_TE;                          /** @brief TE. */
extern const field32_t ETHERNET_MAC_MACCR_PRELEN;                      /** @brief PRELEN. */
extern const field32_t ETHERNET_MAC_MACCR_DC;                          /** @brief DC. */
extern const field32_t ETHERNET_MAC_MACCR_BL;                          /** @brief BL. */
extern const field32_t ETHERNET_MAC_MACCR_DR;                          /** @brief DR. */
extern const field32_t ETHERNET_MAC_MACCR_DCRS;                        /** @brief DCRS. */
extern const field32_t ETHERNET_MAC_MACCR_DO;                          /** @brief DO. */
extern const field32_t ETHERNET_MAC_MACCR_ECRSFD;                      /** @brief ECRSFD. */
extern const field32_t ETHERNET_MAC_MACCR_LM;                          /** @brief LM. */
extern const field32_t ETHERNET_MAC_MACCR_DM;                          /** @brief DM. */
extern const field32_t ETHERNET_MAC_MACCR_FES;                         /** @brief FES. */
extern const field32_t ETHERNET_MAC_MACCR_JE;                          /** @brief JE. */
extern const field32_t ETHERNET_MAC_MACCR_JD;                          /** @brief JD. */
extern const field32_t ETHERNET_MAC_MACCR_WD;                          /** @brief WD. */
extern const field32_t ETHERNET_MAC_MACCR_ACS;                         /** @brief ACS. */
extern const field32_t ETHERNET_MAC_MACCR_CST;                         /** @brief CST. */
extern const field32_t ETHERNET_MAC_MACCR_S2KP;                        /** @brief S2KP. */
extern const field32_t ETHERNET_MAC_MACCR_GPSLCE;                      /** @brief GPSLCE. */
extern const field32_t ETHERNET_MAC_MACCR_IPG;                         /** @brief IPG. */
extern const field32_t ETHERNET_MAC_MACCR_IPC;                         /** @brief IPC. */
extern const field32_t ETHERNET_MAC_MACCR_SARC;                        /** @brief SARC. */
extern const field32_t ETHERNET_MAC_MACCR_ARPEN;                       /** @brief ARPEN. */
extern const field32_t ETHERNET_MAC_MACECR_GPSL;                       /** @brief GPSL. */
extern const field32_t ETHERNET_MAC_MACECR_DCRCC;                      /** @brief DCRCC. */
extern const field32_t ETHERNET_MAC_MACECR_SPEN;                       /** @brief SPEN. */
extern const field32_t ETHERNET_MAC_MACECR_USP;                        /** @brief USP. */
extern const field32_t ETHERNET_MAC_MACECR_EIPGEN;                     /** @brief EIPGEN. */
extern const field32_t ETHERNET_MAC_MACECR_EIPG;                       /** @brief EIPG. */
extern const field32_t ETHERNET_MAC_MACPFR_PR;                         /** @brief PR. */
extern const field32_t ETHERNET_MAC_MACPFR_HUC;                        /** @brief HUC. */
extern const field32_t ETHERNET_MAC_MACPFR_HMC;                        /** @brief HMC. */
extern const field32_t ETHERNET_MAC_MACPFR_DAIF;                       /** @brief DAIF. */
extern const field32_t ETHERNET_MAC_MACPFR_PM;                         /** @brief PM. */
extern const field32_t ETHERNET_MAC_MACPFR_DBF;                        /** @brief DBF. */
extern const field32_t ETHERNET_MAC_MACPFR_PCF;                        /** @brief PCF. */
extern const field32_t ETHERNET_MAC_MACPFR_SAIF;                       /** @brief SAIF. */
extern const field32_t ETHERNET_MAC_MACPFR_SAF;                        /** @brief SAF. */
extern const field32_t ETHERNET_MAC_MACPFR_HPF;                        /** @brief HPF. */
extern const field32_t ETHERNET_MAC_MACPFR_VTFE;                       /** @brief VTFE. */
extern const field32_t ETHERNET_MAC_MACPFR_IPFE;                       /** @brief IPFE. */
extern const field32_t ETHERNET_MAC_MACPFR_DNTU;                       /** @brief DNTU. */
extern const field32_t ETHERNET_MAC_MACPFR_RA;                         /** @brief RA. */
extern const field32_t ETHERNET_MAC_MACWTR_WTO;                        /** @brief WTO. */
extern const field32_t ETHERNET_MAC_MACWTR_PWE;                        /** @brief PWE. */
extern const field32_t ETHERNET_MAC_MACVTR_VL;                         /** @brief VL. */
extern const field32_t ETHERNET_MAC_MACVTR_ETV;                        /** @brief ETV. */
extern const field32_t ETHERNET_MAC_MACVTR_VTIM;                       /** @brief VTIM. */
extern const field32_t ETHERNET_MAC_MACVTR_ESVL;                       /** @brief ESVL. */
extern const field32_t ETHERNET_MAC_MACVTR_ERSVLM;                     /** @brief ERSVLM. */
extern const field32_t ETHERNET_MAC_MACVTR_DOVLTC;                     /** @brief DOVLTC. */
extern const field32_t ETHERNET_MAC_MACVTR_EVLS;                       /** @brief EVLS. */
extern const field32_t ETHERNET_MAC_MACVTR_EVLRXS;                     /** @brief EVLRXS. */
extern const field32_t ETHERNET_MAC_MACVTR_VTHM;                       /** @brief VTHM. */
extern const field32_t ETHERNET_MAC_MACVTR_EDVLP;                      /** @brief EDVLP. */
extern const field32_t ETHERNET_MAC_MACVTR_ERIVLT;                     /** @brief ERIVLT. */
extern const field32_t ETHERNET_MAC_MACVTR_EIVLS;                      /** @brief EIVLS. */
extern const field32_t ETHERNET_MAC_MACVTR_EIVLRXS;                    /** @brief EIVLRXS. */
extern const field32_t ETHERNET_MAC_MACVHTR_VLHT;                      /** @brief VLHT. */
extern const field32_t ETHERNET_MAC_MACVIR_VLT;                        /** @brief VLT. */
extern const field32_t ETHERNET_MAC_MACVIR_VLC;                        /** @brief VLC. */
extern const field32_t ETHERNET_MAC_MACVIR_VLP;                        /** @brief VLP. */
extern const field32_t ETHERNET_MAC_MACVIR_CSVL;                       /** @brief CSVL. */
extern const field32_t ETHERNET_MAC_MACVIR_VLTI;                       /** @brief VLTI. */
extern const field32_t ETHERNET_MAC_MACIVIR_VLT;                       /** @brief VLT. */
extern const field32_t ETHERNET_MAC_MACIVIR_VLC;                       /** @brief VLC. */
extern const field32_t ETHERNET_MAC_MACIVIR_VLP;                       /** @brief VLP. */
extern const field32_t ETHERNET_MAC_MACIVIR_CSVL;                      /** @brief CSVL. */
extern const field32_t ETHERNET_MAC_MACIVIR_VLTI;                      /** @brief VLTI. */
extern const field32_t ETHERNET_MAC_MACQTXFCR_FCB_BPA;                 /** @brief FCB_BPA. */
extern const field32_t ETHERNET_MAC_MACQTXFCR_TFE;                     /** @brief TFE. */
extern const field32_t ETHERNET_MAC_MACQTXFCR_PLT;                     /** @brief PLT. */
extern const field32_t ETHERNET_MAC_MACQTXFCR_DZPQ;                    /** @brief DZPQ. */
extern const field32_t ETHERNET_MAC_MACQTXFCR_PT;                      /** @brief PT. */
extern const field32_t ETHERNET_MAC_MACRXFCR_RFE;                      /** @brief RFE. */
extern const field32_t ETHERNET_MAC_MACRXFCR_UP;                       /** @brief UP. */
extern const field32_t ETHERNET_MAC_MACISR_PHYIS;                      /** @brief PHYIS. */
extern const field32_t ETHERNET_MAC_MACISR_PMTIS;                      /** @brief PMTIS. */
extern const field32_t ETHERNET_MAC_MACISR_LPIIS;                      /** @brief LPIIS. */
extern const field32_t ETHERNET_MAC_MACISR_MMCIS;                      /** @brief MMCIS. */
extern const field32_t ETHERNET_MAC_MACISR_MMCRXIS;                    /** @brief MMCRXIS. */
extern const field32_t ETHERNET_MAC_MACISR_MMCTXIS;                    /** @brief MMCTXIS. */
extern const field32_t ETHERNET_MAC_MACISR_TSIS;                       /** @brief TSIS. */
extern const field32_t ETHERNET_MAC_MACISR_TXSTSIS;                    /** @brief TXSTSIS. */
extern const field32_t ETHERNET_MAC_MACISR_RXSTSIS;                    /** @brief RXSTSIS. */
extern const field32_t ETHERNET_MAC_MACIER_PHYIE;                      /** @brief PHYIE. */
extern const field32_t ETHERNET_MAC_MACIER_PMTIE;                      /** @brief PMTIE. */
extern const field32_t ETHERNET_MAC_MACIER_LPIIE;                      /** @brief LPIIE. */
extern const field32_t ETHERNET_MAC_MACIER_TSIE;                       /** @brief TSIE. */
extern const field32_t ETHERNET_MAC_MACIER_TXSTSIE;                    /** @brief TXSTSIE. */
extern const field32_t ETHERNET_MAC_MACIER_RXSTSIE;                    /** @brief RXSTSIE. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_TJT;                     /** @brief TJT. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_NCARR;                   /** @brief NCARR. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_LCARR;                   /** @brief LCARR. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_EXDEF;                   /** @brief EXDEF. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_LCOL;                    /** @brief LCOL. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_EXCOL;                   /** @brief LCOL. */
extern const field32_t ETHERNET_MAC_MACRXTXSR_RWT;                     /** @brief RWT. */
extern const field32_t ETHERNET_MAC_MACPCSR_PWRDWN;                    /** @brief PWRDWN. */
extern const field32_t ETHERNET_MAC_MACPCSR_MGKPKTEN;                  /** @brief MGKPKTEN. */
extern const field32_t ETHERNET_MAC_MACPCSR_RWKPKTEN;                  /** @brief RWKPKTEN. */
extern const field32_t ETHERNET_MAC_MACPCSR_MGKPRCVD;                  /** @brief MGKPRCVD. */
extern const field32_t ETHERNET_MAC_MACPCSR_RWKPRCVD;                  /** @brief RWKPRCVD. */
extern const field32_t ETHERNET_MAC_MACPCSR_GLBLUCAST;                 /** @brief GLBLUCAST. */
extern const field32_t ETHERNET_MAC_MACPCSR_RWKPFE;                    /** @brief RWKPFE. */
extern const field32_t ETHERNET_MAC_MACPCSR_RWKPTR;                    /** @brief RWKPTR. */
extern const field32_t ETHERNET_MAC_MACPCSR_RWKFILTRST;                /** @brief RWKFILTRST. */
extern const field32_t ETHERNET_MAC_MACLCSR_TLPIEN;                    /** @brief TLPIEN. */
extern const field32_t ETHERNET_MAC_MACLCSR_TLPIEX;                    /** @brief TLPIEX. */
extern const field32_t ETHERNET_MAC_MACLCSR_RLPIEN;                    /** @brief RLPIEN. */
extern const field32_t ETHERNET_MAC_MACLCSR_RLPIEX;                    /** @brief RLPIEX. */
extern const field32_t ETHERNET_MAC_MACLCSR_TLPIST;                    /** @brief TLPIST. */
extern const field32_t ETHERNET_MAC_MACLCSR_RLPIST;                    /** @brief RLPIST. */
extern const field32_t ETHERNET_MAC_MACLCSR_LPIEN;                     /** @brief LPIEN. */
extern const field32_t ETHERNET_MAC_MACLCSR_PLS;                       /** @brief PLS. */
extern const field32_t ETHERNET_MAC_MACLCSR_PLSEN;                     /** @brief PLSEN. */
extern const field32_t ETHERNET_MAC_MACLCSR_LPITXA;                    /** @brief LPITXA. */
extern const field32_t ETHERNET_MAC_MACLCSR_LPITE;                     /** @brief LPITE. */
extern const field32_t ETHERNET_MAC_MACLCSR_LPITCSE;                   /** @brief LPITCSE. */
extern const field32_t ETHERNET_MAC_MACLTCR_TWT;                       /** @brief TWT. */
extern const field32_t ETHERNET_MAC_MACLTCR_LST;                       /** @brief LST. */
extern const field32_t ETHERNET_MAC_MACLETR_LPIET;                     /** @brief LPIET. */
extern const field32_t ETHERNET_MAC_MAC1USTCR_TIC_1US_CNTR;            /** @brief TIC_1US_CNTR. */
extern const field32_t ETHERNET_MAC_MACVR_SNPSVER;                     /** @brief SNPSVER. */
extern const field32_t ETHERNET_MAC_MACVR_USERVER;                     /** @brief USERVER. */
extern const field32_t ETHERNET_MAC_MACHWF1R_RXFIFOSIZE;               /** @brief RXFIFOSIZE. */
extern const field32_t ETHERNET_MAC_MACHWF1R_TXFIFOSIZE;               /** @brief TXFIFOSIZE. */
extern const field32_t ETHERNET_MAC_MACHWF1R_OSTEN;                    /** @brief OSTEN. */
extern const field32_t ETHERNET_MAC_MACHWF1R_PTOEN;                    /** @brief PTOEN. */
extern const field32_t ETHERNET_MAC_MACHWF1R_ADVTHWORD;                /** @brief ADVTHWORD. */
extern const field32_t ETHERNET_MAC_MACHWF1R_DCBEN;                    /** @brief DCBEN. */
extern const field32_t ETHERNET_MAC_MACHWF1R_SPHEN;                    /** @brief SPHEN. */
extern const field32_t ETHERNET_MAC_MACHWF1R_TSOEN;                    /** @brief TSOEN. */
extern const field32_t ETHERNET_MAC_MACHWF1R_DBGMEMA;                  /** @brief DBGMEMA. */
extern const field32_t ETHERNET_MAC_MACHWF1R_AVSEL;                    /** @brief AVSEL. */
extern const field32_t ETHERNET_MAC_MACHWF1R_HASHTBLSZ;                /** @brief HASHTBLSZ. */
extern const field32_t ETHERNET_MAC_MACHWF1R_L3L4FNUM;                 /** @brief L3L4FNUM. */
extern const field32_t ETHERNET_MAC_MACHWF2R_RXQCNT;                   /** @brief RXQCNT. */
extern const field32_t ETHERNET_MAC_MACHWF2R_TXQCNT;                   /** @brief TXQCNT. */
extern const field32_t ETHERNET_MAC_MACHWF2R_RXCHCNT;                  /** @brief RXCHCNT. */
extern const field32_t ETHERNET_MAC_MACHWF2R_TXCHCNT;                  /** @brief TXCHCNT. */
extern const field32_t ETHERNET_MAC_MACHWF2R_PPSOUTNUM;                /** @brief PPSOUTNUM. */
extern const field32_t ETHERNET_MAC_MACHWF2R_AUXSNAPNUM;               /** @brief AUXSNAPNUM. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_MB;                      /** @brief MB. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_C45E;                    /** @brief C45E. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_GOC;                     /** @brief GOC. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_SKAP;                    /** @brief SKAP. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_CR;                      /** @brief CR. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_NTC;                     /** @brief NTC. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_RDA;                     /** @brief RDA. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_PA;                      /** @brief PA. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_BTB;                     /** @brief BTB. */
extern const field32_t ETHERNET_MAC_MACMDIOAR_PSE;                     /** @brief PSE. */
extern const field32_t ETHERNET_MAC_MACMDIODR_MD;                      /** @brief MD. */
extern const field32_t ETHERNET_MAC_MACMDIODR_RA;                      /** @brief RA. */
extern const field32_t ETHERNET_MAC_MACA0HR_ADDRHI;                    /** @brief ADDRHI. */
extern const field32_t ETHERNET_MAC_MACA0HR_AE;                        /** @brief AE. */
extern const field32_t ETHERNET_MAC_MACA1HR_ADDRHI;                    /** @brief ADDRHI. */
extern const field32_t ETHERNET_MAC_MACA1HR_MBC;                       /** @brief MBC. */
extern const field32_t ETHERNET_MAC_MACA1HR_SA;                        /** @brief SA. */
extern const field32_t ETHERNET_MAC_MACA1HR_AE;                        /** @brief AE. */
extern const field32_t ETHERNET_MAC_MACA2HR_ADDRHI;                    /** @brief ADDRHI. */
extern const field32_t ETHERNET_MAC_MACA2HR_MBC;                       /** @brief MBC. */
extern const field32_t ETHERNET_MAC_MACA2HR_SA;                        /** @brief SA. */
extern const field32_t ETHERNET_MAC_MACA2HR_AE;                        /** @brief AE. */
extern const field32_t ETHERNET_MAC_MACA3HR_ADDRHI;                    /** @brief ADDRHI. */
extern const field32_t ETHERNET_MAC_MACA3HR_MBC;                       /** @brief MBC. */
extern const field32_t ETHERNET_MAC_MACA3HR_SA;                        /** @brief SA. */
extern const field32_t ETHERNET_MAC_MACA3HR_AE;                        /** @brief AE. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_CNTRST;                /** @brief CNTRST. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_CNTSTOPRO;             /** @brief CNTSTOPRO. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_RSTONRD;               /** @brief RSTONRD. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_CNTFREEZ;              /** @brief CNTFREEZ. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_CNTPRST;               /** @brief CNTPRST. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_CNTPRSTLVL;            /** @brief CNTPRSTLVL. */
extern const field32_t ETHERNET_MAC_MMC_CONTROL_UCDBC;                 /** @brief UCDBC. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXCRCERPIS;       /** @brief RXCRCERPIS. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXALGNERPIS;      /** @brief RXALGNERPIS. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXUCGPIS;         /** @brief RXUCGPIS. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXLPIUSCIS;       /** @brief RXLPIUSCIS. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXLPITRCIS;       /** @brief RXLPITRCIS. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXSCOLGPIS;       /** @brief TXSCOLGPIS. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXMCOLGPIS;       /** @brief TXMCOLGPIS. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXGPKTIS;         /** @brief TXGPKTIS. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXLPIUSCIS;       /** @brief TXLPIUSCIS. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXLPITRCIS;       /** @brief TXLPITRCIS. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXCRCERPIM;  /** @brief RXCRCERPIM. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXALGNERPIM; /** @brief RXALGNERPIM. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXUCGPIM;    /** @brief RXUCGPIM. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXLPIUSCIM;  /** @brief RXLPIUSCIM. */
extern const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXLPITRCIM;  /** @brief RXLPITRCIM. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXSCOLGPIM;  /** @brief TXSCOLGPIM. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXMCOLGPIM;  /** @brief TXMCOLGPIM. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXGPKTIM;    /** @brief TXGPKTIM. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXLPIUSCIM;  /** @brief TXLPIUSCIM. */
extern const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXLPITRCIM;  /** @brief TXLPITRCIM. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L3SAM0;                 /** @brief L3SAM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L3SAIM0;                /** @brief L3SAIM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L3DAM0;                 /** @brief L3DAM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L3DAIM0;                /** @brief L3DAIM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L3HSBM0;                /** @brief L3HSBM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L3HDBM0;                /** @brief L3HDBM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L4SPM0;                 /** @brief L4SPM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L4SPIM0;                /** @brief L4SPIM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L4DPM0;                 /** @brief L4DPM0. */
extern const field32_t ETHERNET_MAC_MACL3L4C0R_L4DPIM0;                /** @brief L4DPIM0. */
extern const field32_t ETHERNET_MAC_MACL4A0R_L4SP0;                    /** @brief L4SP0. */
extern const field32_t ETHERNET_MAC_MACL4A0R_L4DP0;                    /** @brief L4DP0. */
extern const field32_t ETHERNET_MAC_MACDR_RPESTS;                      /** @brief RPESTS. */
extern const field32_t ETHERNET_MAC_MACDR_RFCFCSTS;                    /** @brief RFCFCSTS. */
extern const field32_t ETHERNET_MAC_MACDR_TPESTS;                      /** @brief TPESTS. */
extern const field32_t ETHERNET_MAC_MACDR_TFCSTS;                      /** @brief TFCSTS. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L3SAM1;                 /** @brief L3SAM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L3SAIM1;                /** @brief L3SAIM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L3DAM1;                 /** @brief L3DAM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L3DAIM1;                /** @brief L3DAIM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L3HSBM1;                /** @brief L3HSBM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L3HDBM1;                /** @brief L3HDBM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L4SPM1;                 /** @brief L4SPM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L4SPIM1;                /** @brief L4SPIM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L4DPM1;                 /** @brief L4DPM1. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_L4DPIM1;                /** @brief L4DPIM1. */
extern const field32_t ETHERNET_MAC_MACL4A1R_L4SP1;                    /** @brief L4SP1. */
extern const field32_t ETHERNET_MAC_MACL4A1R_L4DP1;                    /** @brief L4DP1. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSENA;                     /** @brief TSENA. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSCFUPDT;                  /** @brief TSCFUPDT. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSINIT;                    /** @brief TSINIT. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSUPDT;                    /** @brief TSUPDT. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSADDREG;                  /** @brief TSADDREG. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSENALL;                   /** @brief TSENALL. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSCTRLSSR;                 /** @brief TSCTRLSSR. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSVER2ENA;                 /** @brief TSVER2ENA. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSIPENA;                   /** @brief TSIPENA. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSEVNTENA;                 /** @brief TSEVNTENA. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSMSTRENA;                 /** @brief TSMSTRENA. */
extern const field32_t ETHERNET_MAC_MACTSCR_SNAPTYPSEL;                /** @brief SNAPTYPSEL. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSENMACADDR;               /** @brief TSENMACADDR. */
extern const field32_t ETHERNET_MAC_MACTSCR_CSC;                       /** @brief CSC. */
extern const field32_t ETHERNET_MAC_MACTSCR_TXTSSTSM;                  /** @brief TXTSSTSM. */
extern const field32_t ETHERNET_MAC_MACSSIR_SNSINC;                    /** @brief SNSINC. */
extern const field32_t ETHERNET_MAC_MACSSIR_SSINC;                     /** @brief SSINC. */
extern const field32_t ETHERNET_MAC_MACSTNR_TSSS;                      /** @brief TSSS. */
extern const field32_t ETHERNET_MAC_MACSTNUR_TSSS;                     /** @brief TSSS. */
extern const field32_t ETHERNET_MAC_MACSTNUR_ADDSUB;                   /** @brief ADDSUB. */
extern const field32_t ETHERNET_MAC_MACTSSR_TSSOVF;                    /** @brief TSSOVF. */
extern const field32_t ETHERNET_MAC_MACTSSR_TSTARGT0;                  /** @brief TSTARGT0. */
extern const field32_t ETHERNET_MAC_MACTSSR_AUXTSTRIG;                 /** @brief AUXTSTRIG. */
extern const field32_t ETHERNET_MAC_MACTSSR_TSTRGTERR0;                /** @brief TSTRGTERR0. */
extern const field32_t ETHERNET_MAC_MACTSSR_TXTSSIS;                   /** @brief TXTSSIS. */
extern const field32_t ETHERNET_MAC_MACTSSR_ATSSTN;                    /** @brief ATSSTN. */
extern const field32_t ETHERNET_MAC_MACTSSR_ATSSTM;                    /** @brief ATSSTM. */
extern const field32_t ETHERNET_MAC_MACTSSR_ATSNS;                     /** @brief ATSNS. */
extern const field32_t ETHERNET_MAC_MACTXTSSNR_TXTSSLO;                /** @brief TXTSSLO. */
extern const field32_t ETHERNET_MAC_MACTXTSSNR_TXTSSMIS;               /** @brief TXTSSMIS. */
extern const field32_t ETHERNET_MAC_MACACR_ATSFC;                      /** @brief ATSFC. */
extern const field32_t ETHERNET_MAC_MACATSNR_AUXTSLO;                  /** @brief AUXTSLO. */
extern const field32_t ETHERNET_MAC_MACPPSCR_PPSCTRL;                  /** @brief PPSCTRL. */
extern const field32_t ETHERNET_MAC_MACPPSCR_PPSEN0;                   /** @brief PPSEN0. */
extern const field32_t ETHERNET_MAC_MACPPSCR_TRGTMODSEL0;              /** @brief TRGTMODSEL0. */
extern const field32_t ETHERNET_MAC_MACPPSTTSR_TSTRH0;                 /** @brief TSTRH0. */
extern const field32_t ETHERNET_MAC_MACPPSTTNR_TTSL0;                  /** @brief TTSL0. */
extern const field32_t ETHERNET_MAC_MACPPSTTNR_TRGTBUSY0;              /** @brief TRGTBUSY0. */
extern const field32_t ETHERNET_MAC_MACPOCR_PTOEN;                     /** @brief PTOEN. */
extern const field32_t ETHERNET_MAC_MACPOCR_ASYNCEN;                   /** @brief ASYNCEN. */
extern const field32_t ETHERNET_MAC_MACPOCR_APDREQEN;                  /** @brief APDREQEN. */
extern const field32_t ETHERNET_MAC_MACPOCR_ASYNCTRIG;                 /** @brief ASYNCTRIG. */
extern const field32_t ETHERNET_MAC_MACPOCR_APDREQTRIG;                /** @brief APDREQTRIG. */
extern const field32_t ETHERNET_MAC_MACPOCR_DRRDIS;                    /** @brief DRRDIS. */
extern const field32_t ETHERNET_MAC_MACPOCR_DN;                        /** @brief DN. */
extern const field32_t ETHERNET_MAC_MACSPI2R_SPI2;                     /** @brief SPI2. */
extern const field32_t ETHERNET_MAC_MACLMIR_LSI;                       /** @brief LSI. */
extern const field32_t ETHERNET_MAC_MACLMIR_DRSYNCR;                   /** @brief DRSYNCR. */
extern const field32_t ETHERNET_MAC_MACLMIR_LMPDRI;                    /** @brief LMPDRI. */

/** @subsection Enumerated ETHERNET_MAC Register Field Definitions */

extern const field32_t ETHERNET_MAC_MACL3L4C0R_LxPEN0[5]; /** @brief L3PEN0. */
extern const field32_t ETHERNET_MAC_MACL3L4C1R_LxPEN1[5]; /** @brief L3PEN1. */
extern const field32_t ETHERNET_MAC_MACTSCR_TSIPVxENA[7]; /** @brief TSIPV4ENA. */
extern const field32_t ETHERNET_MAC_MACACR_ATSENx[4];     /** @brief ATSEN0. */

/**************************************************************************************************
 * @section DMAx Definitions
 **************************************************************************************************/

/** @subsection Enumerated DMAx Register Definitions */

extern ro_reg32_t const DMAx_LISR[3];      /** @brief Low interrupt status register. */
extern ro_reg32_t const DMAx_HISR[3];      /** @brief High interrupt status register. */
extern rw_reg32_t const DMAx_LIFCR[3];     /** @brief Low interrupt flag clear register. */
extern rw_reg32_t const DMAx_HIFCR[3];     /** @brief High interrupt flag clear register. */
extern rw_reg32_t const DMAx_S0CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_SxNDTR[3][8]; /** @brief Stream x number of data register. */
extern rw_reg32_t const DMAx_SxPAR[3][8];  /** @brief Stream x peripheral address register. */
extern rw_reg32_t const DMAx_SxM0AR[3][8]; /** @brief Stream x memory 0 address register. */
extern rw_reg32_t const DMAx_SxM1AR[3][8]; /** @brief Stream x memory 1 address register. */
extern rw_reg32_t const DMAx_SxFCR[3][8];  /** @brief Stream x FIFO control register. */
extern rw_reg32_t const DMAx_S1CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_S2CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_S3CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_S4CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_S5CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_S6CR[3];      /** @brief Stream x configuration register. */
extern rw_reg32_t const DMAx_S7CR[3];      /** @brief Stream x configuration register. */

/** @subsection DMAx Register Field Definitions */

extern const field32_t DMAx_S0CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S0CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S0CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S0CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S0CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S0CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S0CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S0CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S0CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S0CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S0CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S0CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S0CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S0CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S0CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S0CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S0CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S0CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_SxNDTR_NDT;  /** @brief Number of data items to transfer. */
extern const field32_t DMAx_SxFCR_FEIE;  /** @brief FIFO error interrupt enable. */
extern const field32_t DMAx_SxFCR_FS;    /** @brief FIFO status. */
extern const field32_t DMAx_SxFCR_DMDIS; /** @brief Direct mode disable. */
extern const field32_t DMAx_SxFCR_FTH;   /** @brief FIFO threshold selection. */
extern const field32_t DMAx_S1CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S1CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S1CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S1CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S1CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S1CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S1CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S1CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S1CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S1CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S1CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S1CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S1CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S1CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S1CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S1CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S1CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S1CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S1CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_S2CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S2CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S2CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S2CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S2CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S2CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S2CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S2CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S2CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S2CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S2CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S2CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S2CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S2CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S2CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S2CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S2CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S2CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S2CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_S3CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S3CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S3CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S3CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S3CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S3CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S3CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S3CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S3CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S3CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S3CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S3CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S3CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S3CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S3CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S3CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S3CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S3CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S3CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_S4CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S4CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S4CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S4CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S4CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S4CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S4CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S4CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S4CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S4CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S4CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S4CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S4CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S4CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S4CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S4CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S4CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S4CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S4CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_S5CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S5CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S5CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S5CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S5CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S5CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S5CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S5CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S5CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S5CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S5CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S5CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S5CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S5CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S5CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S5CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S5CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S5CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S5CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_S6CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S6CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S6CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S6CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S6CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S6CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S6CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S6CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S6CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S6CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S6CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S6CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S6CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S6CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S6CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S6CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S6CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S6CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S6CR_EN;     /** @brief Stream enable / flag stream ready when read low. */
extern const field32_t DMAx_S7CR_MBURST; /** @brief Memory burst transfer configuration. */
extern const field32_t DMAx_S7CR_PBURST; /** @brief Peripheral burst transfer configuration. */
extern const field32_t DMAx_S7CR_ACK;    /** @brief ACK. */
extern const field32_t DMAx_S7CR_CT;     /** @brief Current target (only in double buffer mode). */
extern const field32_t DMAx_S7CR_DBM;    /** @brief Double buffer mode. */
extern const field32_t DMAx_S7CR_PL;     /** @brief Priority level. */
extern const field32_t DMAx_S7CR_PINCOS; /** @brief Peripheral increment offset size. */
extern const field32_t DMAx_S7CR_MSIZE;  /** @brief Memory data size. */
extern const field32_t DMAx_S7CR_PSIZE;  /** @brief Peripheral data size. */
extern const field32_t DMAx_S7CR_MINC;   /** @brief Memory increment mode. */
extern const field32_t DMAx_S7CR_PINC;   /** @brief Peripheral increment mode. */
extern const field32_t DMAx_S7CR_CIRC;   /** @brief Circular mode. */
extern const field32_t DMAx_S7CR_DIR;    /** @brief Data transfer direction. */
extern const field32_t DMAx_S7CR_PFCTRL; /** @brief Peripheral flow controller. */
extern const field32_t DMAx_S7CR_TCIE;   /** @brief Transfer complete interrupt enable. */
extern const field32_t DMAx_S7CR_HTIE;   /** @brief Half transfer interrupt enable. */
extern const field32_t DMAx_S7CR_TEIE;   /** @brief Transfer error interrupt enable. */
extern const field32_t DMAx_S7CR_DMEIE;  /** @brief Direct mode error interrupt enable. */
extern const field32_t DMAx_S7CR_EN;     /** @brief Stream enable / flag stream ready when read low. */

/** @subsection Enumerated DMAx Register Field Definitions */

extern const field32_t DMAx_LISR_TCIFx[4];    /** @brief Stream x transfer complete interrupt flag (x = 3..0). */
extern const field32_t DMAx_LISR_HTIFx[4];    /** @brief Stream x half transfer interrupt flag (x=3..0). */
extern const field32_t DMAx_LISR_TEIFx[4];    /** @brief Stream x transfer error interrupt flag (x=3..0). */
extern const field32_t DMAx_LISR_DMEIFx[4];   /** @brief Stream x direct mode error interrupt flag (x=3..0). */
extern const field32_t DMAx_LISR_FEIFx[4];    /** @brief Stream x FIFO error interrupt flag (x=3..0). */
extern const field32_t DMAx_LIFCR_CTCIFx[4];  /** @brief Stream x clear transfer complete interrupt flag (x = 3..0). */
extern const field32_t DMAx_LIFCR_CHTIFx[4];  /** @brief Stream x clear half transfer interrupt flag (x = 3..0). */
extern const field32_t DMAx_LIFCR_CTEIFx[4];  /** @brief Stream x clear transfer error interrupt flag (x = 3..0). */
extern const field32_t DMAx_LIFCR_CDMEIFx[4]; /** @brief Stream x clear direct mode error interrupt flag (x = 3..0). */
extern const field32_t DMAx_LIFCR_CFEIFx[4];  /** @brief Stream x clear FIFO error interrupt flag (x = 3..0). */

/**************************************************************************************************
 * @section HRTIM_MASTER Definitions
 **************************************************************************************************/

/** @subsection HRTIM_MASTER Register Definitions */

extern rw_reg32_t const HRTIM_MASTER_MCR;    /** @brief Master timer control register. */
extern ro_reg32_t const HRTIM_MASTER_MISR;   /** @brief Master timer interrupt status register. */
extern rw_reg32_t const HRTIM_MASTER_MICR;   /** @brief Master timer interrupt clear register. */
extern rw_reg32_t const HRTIM_MASTER_MDIER4; /** @brief MDIER4. */
extern rw_reg32_t const HRTIM_MASTER_MCNTR;  /** @brief Master timer counter register. */
extern rw_reg32_t const HRTIM_MASTER_MPER;   /** @brief Master timer period register. */
extern rw_reg32_t const HRTIM_MASTER_MREP;   /** @brief Master timer repetition register. */
extern rw_reg32_t const HRTIM_MASTER_MCMP1R; /** @brief Master timer compare 1 register. */
extern rw_reg32_t const HRTIM_MASTER_MCMP2R; /** @brief Master timer compare 2 register. */
extern rw_reg32_t const HRTIM_MASTER_MCMP3R; /** @brief Master timer compare 3 register. */
extern rw_reg32_t const HRTIM_MASTER_MCMP4R; /** @brief Master timer compare 4 register. */

/** @subsection HRTIM_MASTER Register Field Definitions */

extern const field32_t HRTIM_MASTER_MCR_BRSTDMA;   /** @brief Burst DMA update. */
extern const field32_t HRTIM_MASTER_MCR_MREPU;     /** @brief Master timer repetition update. */
extern const field32_t HRTIM_MASTER_MCR_PREEN;     /** @brief Preload enable. */
extern const field32_t HRTIM_MASTER_MCR_DACSYNC;   /** @brief AC synchronization. */
extern const field32_t HRTIM_MASTER_MCR_TECEN;     /** @brief Timer E counter enable. */
extern const field32_t HRTIM_MASTER_MCR_TDCEN;     /** @brief Timer D counter enable. */
extern const field32_t HRTIM_MASTER_MCR_TCCEN;     /** @brief Timer C counter enable. */
extern const field32_t HRTIM_MASTER_MCR_TBCEN;     /** @brief Timer B counter enable. */
extern const field32_t HRTIM_MASTER_MCR_TACEN;     /** @brief Timer A counter enable. */
extern const field32_t HRTIM_MASTER_MCR_MCEN;      /** @brief Master counter enable. */
extern const field32_t HRTIM_MASTER_MCR_SYNC_SRC;  /** @brief Synchronization source. */
extern const field32_t HRTIM_MASTER_MCR_SYNC_OUT;  /** @brief Synchronization output. */
extern const field32_t HRTIM_MASTER_MCR_SYNCSTRTM; /** @brief Synchronization starts master. */
extern const field32_t HRTIM_MASTER_MCR_SYNCRSTM;  /** @brief Synchronization resets master. */
extern const field32_t HRTIM_MASTER_MCR_SYNC_IN;   /** @brief Ynchronization input. */
extern const field32_t HRTIM_MASTER_MCR_HALF;      /** @brief Half mode enable. */
extern const field32_t HRTIM_MASTER_MCR_RETRIG;    /** @brief Master re-triggerable mode. */
extern const field32_t HRTIM_MASTER_MCR_CONT;      /** @brief Master continuous mode. */
extern const field32_t HRTIM_MASTER_MCR_CK_PSC;    /** @brief HRTIM master clock prescaler. */
extern const field32_t HRTIM_MASTER_MISR_MUPD;     /** @brief Master update interrupt flag. */
extern const field32_t HRTIM_MASTER_MISR_SYNC;     /** @brief Sync input interrupt flag. */
extern const field32_t HRTIM_MASTER_MISR_MREP;     /** @brief Master repetition interrupt flag. */
extern const field32_t HRTIM_MASTER_MICR_MUPDC;    /** @brief Master update interrupt flag clear. */
extern const field32_t HRTIM_MASTER_MICR_SYNCC;    /** @brief Sync input interrupt flag clear. */
extern const field32_t HRTIM_MASTER_MICR_MREPC;    /** @brief Repetition interrupt flag clear. */
extern const field32_t HRTIM_MASTER_MDIER4_MUPDDE; /** @brief MUPDDE. */
extern const field32_t HRTIM_MASTER_MDIER4_SYNCDE; /** @brief SYNCDE. */
extern const field32_t HRTIM_MASTER_MDIER4_MREPDE; /** @brief MREPDE. */
extern const field32_t HRTIM_MASTER_MDIER4_MUPDIE; /** @brief MUPDIE. */
extern const field32_t HRTIM_MASTER_MDIER4_SYNCIE; /** @brief SYNCIE. */
extern const field32_t HRTIM_MASTER_MDIER4_MREPIE; /** @brief MREPIE. */
extern const field32_t HRTIM_MASTER_MCNTR_MCNT;    /** @brief Counter value. */
extern const field32_t HRTIM_MASTER_MPER_MPER;     /** @brief Master timer period value. */
extern const field32_t HRTIM_MASTER_MREP_MREP;     /** @brief Master timer repetition counter value. */
extern const field32_t HRTIM_MASTER_MCMP1R_MCMP1;  /** @brief Master timer compare 1 value. */
extern const field32_t HRTIM_MASTER_MCMP2R_MCMP2;  /** @brief Master timer compare 2 value. */
extern const field32_t HRTIM_MASTER_MCMP3R_MCMP3;  /** @brief Master timer compare 3 value. */
extern const field32_t HRTIM_MASTER_MCMP4R_MCMP4;  /** @brief Master timer compare 4 value. */

/** @subsection Enumerated HRTIM_MASTER Register Field Definitions */

extern const field32_t HRTIM_MASTER_MISR_MCMPx[5];     /** @brief Master compare 1 interrupt flag. */
extern const field32_t HRTIM_MASTER_MICR_MCMPxC[5];    /** @brief Master compare 1 interrupt flag clear. */
extern const field32_t HRTIM_MASTER_MDIER4_MCMPxDE[5]; /** @brief MCMP1DE. */
extern const field32_t HRTIM_MASTER_MDIER4_MCMPxIE[5]; /** @brief MCMP1IE. */

/**************************************************************************************************
 * @section HRTIM_TIMA Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMA Register Definitions */

extern rw_reg32_t const HRTIM_TIMA_TIMACR;    /** @brief Timerx control register. */
extern ro_reg32_t const HRTIM_TIMA_TIMAISR;   /** @brief Timerx interrupt status register. */
extern rw_reg32_t const HRTIM_TIMA_TIMAICR;   /** @brief Timerx interrupt clear register. */
extern rw_reg32_t const HRTIM_TIMA_TIMADIER5; /** @brief TIMxDIER5. */
extern rw_reg32_t const HRTIM_TIMA_CNTAR;     /** @brief Timerx counter register. */
extern rw_reg32_t const HRTIM_TIMA_PERAR;     /** @brief Timerx period register. */
extern rw_reg32_t const HRTIM_TIMA_REPAR;     /** @brief Timerx repetition register. */
extern rw_reg32_t const HRTIM_TIMA_CMP1AR;    /** @brief Timerx compare 1 register. */
extern rw_reg32_t const HRTIM_TIMA_CMP1CAR;   /** @brief Timerx compare 1 compound register. */
extern rw_reg32_t const HRTIM_TIMA_CMP2AR;    /** @brief Timerx compare 2 register. */
extern rw_reg32_t const HRTIM_TIMA_CMP3AR;    /** @brief Timerx compare 3 register. */
extern rw_reg32_t const HRTIM_TIMA_CMP4AR;    /** @brief Timerx compare 4 register. */
extern ro_reg32_t const HRTIM_TIMA_CPT1AR;    /** @brief Timerx capture 1 register. */
extern ro_reg32_t const HRTIM_TIMA_CPT2AR;    /** @brief Timerx capture 2 register. */
extern rw_reg32_t const HRTIM_TIMA_DTAR;      /** @brief Timerx deadtime register. */
extern rw_reg32_t const HRTIM_TIMA_EEFAR1;    /** @brief Timerx external event filtering register 1. */
extern rw_reg32_t const HRTIM_TIMA_EEFAR2;    /** @brief Timerx external event filtering register 2. */
extern rw_reg32_t const HRTIM_TIMA_RSTAR;     /** @brief TimerA reset register. */
extern rw_reg32_t const HRTIM_TIMA_CHPAR;     /** @brief Timerx chopper register. */
extern rw_reg32_t const HRTIM_TIMA_OUTAR;     /** @brief Timerx output register. */
extern rw_reg32_t const HRTIM_TIMA_FLTAR;     /** @brief Timerx fault register. */

/** @subsection Enumerated HRTIM_TIMA Register Definitions */

extern rw_reg32_t const HRTIM_TIMA_SETAxR[3];  /** @brief Timerx output1 set register. */
extern rw_reg32_t const HRTIM_TIMA_RSTAxR[3];  /** @brief Timerx output1 reset register. */
extern rw_reg32_t const HRTIM_TIMA_CPTxACR[3]; /** @brief Timerx capture 2 control register. */

/** @subsection HRTIM_TIMA Register Field Definitions */

extern const field32_t HRTIM_TIMA_TIMACR_UPDGAT;      /** @brief Update gating. */
extern const field32_t HRTIM_TIMA_TIMACR_PREEN;       /** @brief Preload enable. */
extern const field32_t HRTIM_TIMA_TIMACR_DACSYNC;     /** @brief AC synchronization. */
extern const field32_t HRTIM_TIMA_TIMACR_MSTU;        /** @brief Master timer update. */
extern const field32_t HRTIM_TIMA_TIMACR_TEU;         /** @brief TEU. */
extern const field32_t HRTIM_TIMA_TIMACR_TDU;         /** @brief TDU. */
extern const field32_t HRTIM_TIMA_TIMACR_TCU;         /** @brief TCU. */
extern const field32_t HRTIM_TIMA_TIMACR_TBU;         /** @brief TBU. */
extern const field32_t HRTIM_TIMA_TIMACR_TXRSTU;      /** @brief Timerx reset update. */
extern const field32_t HRTIM_TIMA_TIMACR_TXREPU;      /** @brief Timer x repetition update. */
extern const field32_t HRTIM_TIMA_TIMACR_SYNCSTRTX;   /** @brief Synchronization starts timer x. */
extern const field32_t HRTIM_TIMA_TIMACR_SYNCRSTX;    /** @brief Synchronization resets timer x. */
extern const field32_t HRTIM_TIMA_TIMACR_PSHPLL;      /** @brief Push-Pull mode enable. */
extern const field32_t HRTIM_TIMA_TIMACR_HALF;        /** @brief Half mode enable. */
extern const field32_t HRTIM_TIMA_TIMACR_RETRIG;      /** @brief Re-triggerable mode. */
extern const field32_t HRTIM_TIMA_TIMACR_CONT;        /** @brief Continuous mode. */
extern const field32_t HRTIM_TIMA_TIMACR_CK_PSCX;     /** @brief HRTIM timer x clock prescaler. */
extern const field32_t HRTIM_TIMA_TIMAISR_IPPSTAT;    /** @brief Idle push pull status. */
extern const field32_t HRTIM_TIMA_TIMAISR_CPPSTAT;    /** @brief Current push pull status. */
extern const field32_t HRTIM_TIMA_TIMAISR_DLYPRT;     /** @brief Delayed protection flag. */
extern const field32_t HRTIM_TIMA_TIMAISR_RST;        /** @brief Reset interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAISR_UPD;        /** @brief Update interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAISR_REP;        /** @brief Repetition interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAICR_DLYPRTC;    /** @brief Delayed protection flag clear. */
extern const field32_t HRTIM_TIMA_TIMAICR_RSTC;       /** @brief Reset interrupt flag clear. */
extern const field32_t HRTIM_TIMA_TIMAICR_UPDC;       /** @brief Update interrupt flag clear. */
extern const field32_t HRTIM_TIMA_TIMAICR_REPC;       /** @brief Repetition interrupt flag clear. */
extern const field32_t HRTIM_TIMA_TIMADIER5_DLYPRTDE; /** @brief DLYPRTDE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_RSTDE;    /** @brief RSTDE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_SETX2DE;  /** @brief SETx2DE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_SET1XDE;  /** @brief SET1xDE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_UPDDE;    /** @brief UPDDE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_REPDE;    /** @brief REPDE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_DLYPRTIE; /** @brief DLYPRTIE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_RSTIE;    /** @brief RSTIE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_SETX2IE;  /** @brief SETx2IE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_SET1XIE;  /** @brief SET1xIE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_UPDIE;    /** @brief UPDIE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_REPIE;    /** @brief REPIE. */
extern const field32_t HRTIM_TIMA_CNTAR_CNTX;         /** @brief Timerx counter value. */
extern const field32_t HRTIM_TIMA_PERAR_PERX;         /** @brief Timerx period value. */
extern const field32_t HRTIM_TIMA_REPAR_REPX;         /** @brief Timerx repetition counter value. */
extern const field32_t HRTIM_TIMA_CMP1AR_CMP1X;       /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMA_CMP1CAR_REPX;       /** @brief Timerx repetition value (aliased from hrtim_repx register). */
extern const field32_t HRTIM_TIMA_CMP1CAR_CMP1X;      /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMA_CMP2AR_CMP2X;       /** @brief Timerx compare 2 value. */
extern const field32_t HRTIM_TIMA_CMP3AR_CMP3X;       /** @brief Timerx compare 3 value. */
extern const field32_t HRTIM_TIMA_CMP4AR_CMP4X;       /** @brief Timerx compare 4 value. */
extern const field32_t HRTIM_TIMA_CPT1AR_CPT1X;       /** @brief Timerx capture 1 value. */
extern const field32_t HRTIM_TIMA_CPT2AR_CPT2X;       /** @brief Timerx capture 2 value. */
extern const field32_t HRTIM_TIMA_DTAR_DTFLKX;        /** @brief Deadtime falling lock. */
extern const field32_t HRTIM_TIMA_DTAR_DTFSLKX;       /** @brief Deadtime falling sign lock. */
extern const field32_t HRTIM_TIMA_DTAR_SDTFX;         /** @brief Sign deadtime falling value. */
extern const field32_t HRTIM_TIMA_DTAR_DTFX;          /** @brief Deadtime falling value. */
extern const field32_t HRTIM_TIMA_DTAR_DTRLKX;        /** @brief Deadtime rising lock. */
extern const field32_t HRTIM_TIMA_DTAR_DTRSLKX;       /** @brief Deadtime rising sign lock. */
extern const field32_t HRTIM_TIMA_DTAR_DTPRSC;        /** @brief Deadtime prescaler. */
extern const field32_t HRTIM_TIMA_DTAR_SDTRX;         /** @brief Sign deadtime rising value. */
extern const field32_t HRTIM_TIMA_DTAR_DTRX;          /** @brief Deadtime rising value. */
extern const field32_t HRTIM_TIMA_SETAxR_UPDATE;      /** @brief Registers update (transfer preload to active). */
extern const field32_t HRTIM_TIMA_SETAxR_MSTPER;      /** @brief Master period. */
extern const field32_t HRTIM_TIMA_SETAxR_PER;         /** @brief Timer A period. */
extern const field32_t HRTIM_TIMA_SETAxR_RESYNC;      /** @brief Timer A resynchronizaton. */
extern const field32_t HRTIM_TIMA_SETAxR_SST;         /** @brief Software set trigger. */
extern const field32_t HRTIM_TIMA_RSTAxR_UPDATE;      /** @brief UPDATE. */
extern const field32_t HRTIM_TIMA_RSTAxR_MSTPER;      /** @brief MSTPER. */
extern const field32_t HRTIM_TIMA_RSTAxR_PER;         /** @brief PER. */
extern const field32_t HRTIM_TIMA_RSTAxR_RESYNC;      /** @brief RESYNC. */
extern const field32_t HRTIM_TIMA_RSTAxR_SRT;         /** @brief SRT. */
extern const field32_t HRTIM_TIMA_RSTAR_MSTPER;       /** @brief Master timer period. */
extern const field32_t HRTIM_TIMA_RSTAR_UPDT;         /** @brief Timer A update reset. */
extern const field32_t HRTIM_TIMA_CHPAR_STRTPW;       /** @brief STRTPW. */
extern const field32_t HRTIM_TIMA_CHPAR_CHPDTY;       /** @brief Timerx chopper duty cycle value. */
extern const field32_t HRTIM_TIMA_CHPAR_CHPFRQ;       /** @brief Timerx carrier frequency value. */
extern const field32_t HRTIM_TIMA_CPTxACR_TE1RST;     /** @brief Timer E output 1 reset. */
extern const field32_t HRTIM_TIMA_CPTxACR_TE1SET;     /** @brief Timer E output 1 set. */
extern const field32_t HRTIM_TIMA_CPTxACR_TD1RST;     /** @brief Timer D output 1 reset. */
extern const field32_t HRTIM_TIMA_CPTxACR_TD1SET;     /** @brief Timer D output 1 set. */
extern const field32_t HRTIM_TIMA_CPTxACR_TC1RST;     /** @brief Timer C output 1 reset. */
extern const field32_t HRTIM_TIMA_CPTxACR_TC1SET;     /** @brief Timer C output 1 set. */
extern const field32_t HRTIM_TIMA_CPTxACR_TB1RST;     /** @brief Timer B output 1 reset. */
extern const field32_t HRTIM_TIMA_CPTxACR_TB1SET;     /** @brief Timer B output 1 set. */
extern const field32_t HRTIM_TIMA_CPTxACR_UDPCPT;     /** @brief Update capture. */
extern const field32_t HRTIM_TIMA_CPTxACR_SWCPT;      /** @brief Software capture. */
extern const field32_t HRTIM_TIMA_OUTAR_DLYPRT;       /** @brief Delayed protection. */
extern const field32_t HRTIM_TIMA_OUTAR_DLYPRTEN;     /** @brief Delayed protection enable. */
extern const field32_t HRTIM_TIMA_OUTAR_DTEN;         /** @brief Deadtime enable. */
extern const field32_t HRTIM_TIMA_FLTAR_FLTLCK;       /** @brief Fault sources lock. */

/** @subsection Enumerated HRTIM_TIMA Register Field Definitions */

extern const field32_t HRTIM_TIMA_TIMACR_DELCMPx[5];    /** @brief Delayed CMP2 mode. */
extern const field32_t HRTIM_TIMA_TIMAISR_OxSTAT[3];    /** @brief Output 1 state. */
extern const field32_t HRTIM_TIMA_TIMAISR_RSTXx[3];     /** @brief Output 1 reset interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAISR_SETXx[3];     /** @brief Output 1 set interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAISR_CPTx[3];      /** @brief Capture1 interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAISR_CMPx[5];      /** @brief Compare 1 interrupt flag. */
extern const field32_t HRTIM_TIMA_TIMAICR_RSTXxC[3];    /** @brief Output 1 reset flag clear. */
extern const field32_t HRTIM_TIMA_TIMAICR_SETxXC[3];    /** @brief Output 1 set flag clear. */
extern const field32_t HRTIM_TIMA_TIMAICR_CPTxC[3];     /** @brief Capture1 interrupt flag clear. */
extern const field32_t HRTIM_TIMA_TIMAICR_CMPxC[5];     /** @brief Compare 1 interrupt flag clear. */
extern const field32_t HRTIM_TIMA_TIMADIER5_RSTXxDE[3]; /** @brief RSTx1DE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_CPTxDE[3];  /** @brief CPT1DE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_CMPxDE[5];  /** @brief CMP1DE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_RSTXxIE[3]; /** @brief RSTx1IE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_CPTxIE[3];  /** @brief CPT1IE. */
extern const field32_t HRTIM_TIMA_TIMADIER5_CMPxIE[5];  /** @brief CMP1IE. */
extern const field32_t HRTIM_TIMA_SETAxR_EXTEVNTx[11];  /** @brief External event 1. */
extern const field32_t HRTIM_TIMA_SETAxR_TIMEVNTx[10];  /** @brief Timer event 1. */
extern const field32_t HRTIM_TIMA_SETAxR_MSTCMPx[5];    /** @brief Master compare 1. */
extern const field32_t HRTIM_TIMA_EEFAR1_EExFLTR[6];    /** @brief External event 1 filter. */
extern const field32_t HRTIM_TIMA_EEFAR1_EExLTCH[6];    /** @brief External event 1 latch. */
extern const field32_t HRTIM_TIMA_RSTAR_TIMECMPx[5];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMA_RSTAR_TIMDCMPx[5];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIMA_RSTAR_TIMCCMPx[5];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIMA_RSTAR_TIMBCMPx[5];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIMA_CPTxACR_TECMPx[3];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMA_CPTxACR_TDCMPx[3];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIMA_CPTxACR_TCCMPx[3];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIMA_CPTxACR_TBCMPx[3];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIMA_CPTxACR_EXEVxCPT[11]; /** @brief External event 1 capture. */
extern const field32_t HRTIM_TIMA_OUTAR_DIDLx[3];       /** @brief Output 1 deadtime upon burst mode idle entry. */
extern const field32_t HRTIM_TIMA_OUTAR_CHPx[3];        /** @brief Output 1 chopper enable. */
extern const field32_t HRTIM_TIMA_OUTAR_FAULTx[3];      /** @brief Output 1 fault state. */
extern const field32_t HRTIM_TIMA_OUTAR_IDLESx[3];      /** @brief Output 1 idle state. */
extern const field32_t HRTIM_TIMA_OUTAR_IDLEMx[3];      /** @brief Output 1 idle mode. */
extern const field32_t HRTIM_TIMA_OUTAR_POLx[3];        /** @brief Output 1 polarity. */
extern const field32_t HRTIM_TIMA_FLTAR_FLTxEN[6];      /** @brief Fault 1 enable. */

/**************************************************************************************************
 * @section HRTIM_TIMB Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMB Register Definitions */

extern rw_reg32_t const HRTIM_TIMB_TIMBCR;    /** @brief Timerx control register. */
extern ro_reg32_t const HRTIM_TIMB_TIMBISR;   /** @brief Timerx interrupt status register. */
extern rw_reg32_t const HRTIM_TIMB_TIMBICR;   /** @brief Timerx interrupt clear register. */
extern rw_reg32_t const HRTIM_TIMB_TIMBDIER5; /** @brief TIMxDIER5. */
extern rw_reg32_t const HRTIM_TIMB_CNTR;      /** @brief Timerx counter register. */
extern rw_reg32_t const HRTIM_TIMB_PERBR;     /** @brief Timerx period register. */
extern rw_reg32_t const HRTIM_TIMB_REPBR;     /** @brief Timerx repetition register. */
extern rw_reg32_t const HRTIM_TIMB_CMP1BR;    /** @brief Timerx compare 1 register. */
extern rw_reg32_t const HRTIM_TIMB_CMP1CBR;   /** @brief Timerx compare 1 compound register. */
extern rw_reg32_t const HRTIM_TIMB_CMP2BR;    /** @brief Timerx compare 2 register. */
extern rw_reg32_t const HRTIM_TIMB_CMP3BR;    /** @brief Timerx compare 3 register. */
extern rw_reg32_t const HRTIM_TIMB_CMP4BR;    /** @brief Timerx compare 4 register. */
extern ro_reg32_t const HRTIM_TIMB_CPT1BR;    /** @brief Timerx capture 1 register. */
extern ro_reg32_t const HRTIM_TIMB_CPT2BR;    /** @brief Timerx capture 2 register. */
extern rw_reg32_t const HRTIM_TIMB_DTBR;      /** @brief Timerx deadtime register. */
extern rw_reg32_t const HRTIM_TIMB_EEFBR1;    /** @brief Timerx external event filtering register 1. */
extern rw_reg32_t const HRTIM_TIMB_EEFBR2;    /** @brief Timerx external event filtering register 2. */
extern rw_reg32_t const HRTIM_TIMB_RSTBR;     /** @brief TimerA reset register. */
extern rw_reg32_t const HRTIM_TIMB_CHPBR;     /** @brief Timerx chopper register. */
extern rw_reg32_t const HRTIM_TIMB_OUTBR;     /** @brief Timerx output register. */
extern rw_reg32_t const HRTIM_TIMB_FLTBR;     /** @brief Timerx fault register. */

/** @subsection Enumerated HRTIM_TIMB Register Definitions */

extern rw_reg32_t const HRTIM_TIMB_SETBxR[3];  /** @brief Timerx output1 set register. */
extern rw_reg32_t const HRTIM_TIMB_RSTBxR[3];  /** @brief Timerx output1 reset register. */
extern rw_reg32_t const HRTIM_TIMB_CPTxBCR[3]; /** @brief Timerx capture 2 control register. */

/** @subsection HRTIM_TIMB Register Field Definitions */

extern const field32_t HRTIM_TIMB_TIMBCR_UPDGAT;      /** @brief Update gating. */
extern const field32_t HRTIM_TIMB_TIMBCR_PREEN;       /** @brief Preload enable. */
extern const field32_t HRTIM_TIMB_TIMBCR_DACSYNC;     /** @brief AC synchronization. */
extern const field32_t HRTIM_TIMB_TIMBCR_MSTU;        /** @brief Master timer update. */
extern const field32_t HRTIM_TIMB_TIMBCR_TEU;         /** @brief TEU. */
extern const field32_t HRTIM_TIMB_TIMBCR_TDU;         /** @brief TDU. */
extern const field32_t HRTIM_TIMB_TIMBCR_TCU;         /** @brief TCU. */
extern const field32_t HRTIM_TIMB_TIMBCR_TBU;         /** @brief TBU. */
extern const field32_t HRTIM_TIMB_TIMBCR_TXRSTU;      /** @brief Timerx reset update. */
extern const field32_t HRTIM_TIMB_TIMBCR_TXREPU;      /** @brief Timer x repetition update. */
extern const field32_t HRTIM_TIMB_TIMBCR_SYNCSTRTX;   /** @brief Synchronization starts timer x. */
extern const field32_t HRTIM_TIMB_TIMBCR_SYNCRSTX;    /** @brief Synchronization resets timer x. */
extern const field32_t HRTIM_TIMB_TIMBCR_PSHPLL;      /** @brief Push-Pull mode enable. */
extern const field32_t HRTIM_TIMB_TIMBCR_HALF;        /** @brief Half mode enable. */
extern const field32_t HRTIM_TIMB_TIMBCR_RETRIG;      /** @brief Re-triggerable mode. */
extern const field32_t HRTIM_TIMB_TIMBCR_CONT;        /** @brief Continuous mode. */
extern const field32_t HRTIM_TIMB_TIMBCR_CK_PSCX;     /** @brief HRTIM timer x clock prescaler. */
extern const field32_t HRTIM_TIMB_TIMBISR_IPPSTAT;    /** @brief Idle push pull status. */
extern const field32_t HRTIM_TIMB_TIMBISR_CPPSTAT;    /** @brief Current push pull status. */
extern const field32_t HRTIM_TIMB_TIMBISR_DLYPRT;     /** @brief Delayed protection flag. */
extern const field32_t HRTIM_TIMB_TIMBISR_RST;        /** @brief Reset interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBISR_UPD;        /** @brief Update interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBISR_REP;        /** @brief Repetition interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBICR_DLYPRTC;    /** @brief Delayed protection flag clear. */
extern const field32_t HRTIM_TIMB_TIMBICR_RSTC;       /** @brief Reset interrupt flag clear. */
extern const field32_t HRTIM_TIMB_TIMBICR_UPDC;       /** @brief Update interrupt flag clear. */
extern const field32_t HRTIM_TIMB_TIMBICR_REPC;       /** @brief Repetition interrupt flag clear. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_DLYPRTDE; /** @brief DLYPRTDE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_RSTDE;    /** @brief RSTDE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_SETX2DE;  /** @brief SETx2DE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_SET1XDE;  /** @brief SET1xDE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_UPDDE;    /** @brief UPDDE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_REPDE;    /** @brief REPDE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_DLYPRTIE; /** @brief DLYPRTIE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_RSTIE;    /** @brief RSTIE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_SETX2IE;  /** @brief SETx2IE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_SET1XIE;  /** @brief SET1xIE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_UPDIE;    /** @brief UPDIE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_REPIE;    /** @brief REPIE. */
extern const field32_t HRTIM_TIMB_CNTR_CNTX;          /** @brief Timerx counter value. */
extern const field32_t HRTIM_TIMB_PERBR_PERX;         /** @brief Timerx period value. */
extern const field32_t HRTIM_TIMB_REPBR_REPX;         /** @brief Timerx repetition counter value. */
extern const field32_t HRTIM_TIMB_CMP1BR_CMP1X;       /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMB_CMP1CBR_REPX;       /** @brief Timerx repetition value (aliased from hrtim_repx register). */
extern const field32_t HRTIM_TIMB_CMP1CBR_CMP1X;      /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMB_CMP2BR_CMP2X;       /** @brief Timerx compare 2 value. */
extern const field32_t HRTIM_TIMB_CMP3BR_CMP3X;       /** @brief Timerx compare 3 value. */
extern const field32_t HRTIM_TIMB_CMP4BR_CMP4X;       /** @brief Timerx compare 4 value. */
extern const field32_t HRTIM_TIMB_CPT1BR_CPT1X;       /** @brief Timerx capture 1 value. */
extern const field32_t HRTIM_TIMB_CPT2BR_CPT2X;       /** @brief Timerx capture 2 value. */
extern const field32_t HRTIM_TIMB_DTBR_DTFLKX;        /** @brief Deadtime falling lock. */
extern const field32_t HRTIM_TIMB_DTBR_DTFSLKX;       /** @brief Deadtime falling sign lock. */
extern const field32_t HRTIM_TIMB_DTBR_SDTFX;         /** @brief Sign deadtime falling value. */
extern const field32_t HRTIM_TIMB_DTBR_DTFX;          /** @brief Deadtime falling value. */
extern const field32_t HRTIM_TIMB_DTBR_DTRLKX;        /** @brief Deadtime rising lock. */
extern const field32_t HRTIM_TIMB_DTBR_DTRSLKX;       /** @brief Deadtime rising sign lock. */
extern const field32_t HRTIM_TIMB_DTBR_DTPRSC;        /** @brief Deadtime prescaler. */
extern const field32_t HRTIM_TIMB_DTBR_SDTRX;         /** @brief Sign deadtime rising value. */
extern const field32_t HRTIM_TIMB_DTBR_DTRX;          /** @brief Deadtime rising value. */
extern const field32_t HRTIM_TIMB_SETBxR_UPDATE;      /** @brief Registers update (transfer preload to active). */
extern const field32_t HRTIM_TIMB_SETBxR_MSTPER;      /** @brief Master period. */
extern const field32_t HRTIM_TIMB_SETBxR_PER;         /** @brief Timer A period. */
extern const field32_t HRTIM_TIMB_SETBxR_RESYNC;      /** @brief Timer A resynchronizaton. */
extern const field32_t HRTIM_TIMB_SETBxR_SST;         /** @brief Software set trigger. */
extern const field32_t HRTIM_TIMB_RSTBxR_UPDATE;      /** @brief UPDATE. */
extern const field32_t HRTIM_TIMB_RSTBxR_MSTPER;      /** @brief MSTPER. */
extern const field32_t HRTIM_TIMB_RSTBxR_PER;         /** @brief PER. */
extern const field32_t HRTIM_TIMB_RSTBxR_RESYNC;      /** @brief RESYNC. */
extern const field32_t HRTIM_TIMB_RSTBxR_SRT;         /** @brief SRT. */
extern const field32_t HRTIM_TIMB_RSTBR_MSTPER;       /** @brief Master timer period. */
extern const field32_t HRTIM_TIMB_RSTBR_UPDT;         /** @brief Timer A update reset. */
extern const field32_t HRTIM_TIMB_CHPBR_STRTPW;       /** @brief STRTPW. */
extern const field32_t HRTIM_TIMB_CHPBR_CHPDTY;       /** @brief Timerx chopper duty cycle value. */
extern const field32_t HRTIM_TIMB_CHPBR_CHPFRQ;       /** @brief Timerx carrier frequency value. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TE1RST;     /** @brief Timer E output 1 reset. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TE1SET;     /** @brief Timer E output 1 set. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TD1RST;     /** @brief Timer D output 1 reset. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TD1SET;     /** @brief Timer D output 1 set. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TC1RST;     /** @brief Timer C output 1 reset. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TC1SET;     /** @brief Timer C output 1 set. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TA1RST;     /** @brief Timer A output 1 reset. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TA1SET;     /** @brief Timer A output 1 set. */
extern const field32_t HRTIM_TIMB_CPTxBCR_UDPCPT;     /** @brief Update capture. */
extern const field32_t HRTIM_TIMB_CPTxBCR_SWCPT;      /** @brief Software capture. */
extern const field32_t HRTIM_TIMB_OUTBR_DLYPRT;       /** @brief Delayed protection. */
extern const field32_t HRTIM_TIMB_OUTBR_DLYPRTEN;     /** @brief Delayed protection enable. */
extern const field32_t HRTIM_TIMB_OUTBR_DTEN;         /** @brief Deadtime enable. */
extern const field32_t HRTIM_TIMB_FLTBR_FLTLCK;       /** @brief Fault sources lock. */

/** @subsection Enumerated HRTIM_TIMB Register Field Definitions */

extern const field32_t HRTIM_TIMB_TIMBCR_DELCMPx[5];    /** @brief Delayed CMP2 mode. */
extern const field32_t HRTIM_TIMB_TIMBISR_OxSTAT[3];    /** @brief Output 1 state. */
extern const field32_t HRTIM_TIMB_TIMBISR_RSTXx[3];     /** @brief Output 1 reset interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBISR_SETXx[3];     /** @brief Output 1 set interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBISR_CPTx[3];      /** @brief Capture1 interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBISR_CMPx[5];      /** @brief Compare 1 interrupt flag. */
extern const field32_t HRTIM_TIMB_TIMBICR_RSTXxC[3];    /** @brief Output 1 reset flag clear. */
extern const field32_t HRTIM_TIMB_TIMBICR_SETxXC[3];    /** @brief Output 1 set flag clear. */
extern const field32_t HRTIM_TIMB_TIMBICR_CPTxC[3];     /** @brief Capture1 interrupt flag clear. */
extern const field32_t HRTIM_TIMB_TIMBICR_CMPxC[5];     /** @brief Compare 1 interrupt flag clear. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_RSTXxDE[3]; /** @brief RSTx1DE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_CPTxDE[3];  /** @brief CPT1DE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_CMPxDE[5];  /** @brief CMP1DE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_RSTXxIE[3]; /** @brief RSTx1IE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_CPTxIE[3];  /** @brief CPT1IE. */
extern const field32_t HRTIM_TIMB_TIMBDIER5_CMPxIE[5];  /** @brief CMP1IE. */
extern const field32_t HRTIM_TIMB_SETBxR_EXTEVNTx[11];  /** @brief External event 1. */
extern const field32_t HRTIM_TIMB_SETBxR_TIMEVNTx[10];  /** @brief Timer event 1. */
extern const field32_t HRTIM_TIMB_SETBxR_MSTCMPx[5];    /** @brief Master compare 1. */
extern const field32_t HRTIM_TIMB_EEFBR1_EExFLTR[6];    /** @brief External event 1 filter. */
extern const field32_t HRTIM_TIMB_EEFBR1_EExLTCH[6];    /** @brief External event 1 latch. */
extern const field32_t HRTIM_TIMB_RSTBR_TIMECMPx[5];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMB_RSTBR_TIMDCMPx[5];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIMB_RSTBR_TIMCCMPx[5];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIMB_RSTBR_TIMACMPx[5];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TECMPx[3];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TDCMPx[3];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TCCMPx[3];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIMB_CPTxBCR_TACMPx[3];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIMB_CPTxBCR_EXEVxCPT[11]; /** @brief External event 1 capture. */
extern const field32_t HRTIM_TIMB_OUTBR_DIDLx[3];       /** @brief Output 1 deadtime upon burst mode idle entry. */
extern const field32_t HRTIM_TIMB_OUTBR_CHPx[3];        /** @brief Output 1 chopper enable. */
extern const field32_t HRTIM_TIMB_OUTBR_FAULTx[3];      /** @brief Output 1 fault state. */
extern const field32_t HRTIM_TIMB_OUTBR_IDLESx[3];      /** @brief Output 1 idle state. */
extern const field32_t HRTIM_TIMB_OUTBR_IDLEMx[3];      /** @brief Output 1 idle mode. */
extern const field32_t HRTIM_TIMB_OUTBR_POLx[3];        /** @brief Output 1 polarity. */
extern const field32_t HRTIM_TIMB_FLTBR_FLTxEN[6];      /** @brief Fault 1 enable. */

/**************************************************************************************************
 * @section HRTIM_TIMC Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMC Register Definitions */

extern rw_reg32_t const HRTIM_TIMC_TIMCCR;    /** @brief Timerx control register. */
extern ro_reg32_t const HRTIM_TIMC_TIMCISR;   /** @brief Timerx interrupt status register. */
extern rw_reg32_t const HRTIM_TIMC_TIMCICR;   /** @brief Timerx interrupt clear register. */
extern rw_reg32_t const HRTIM_TIMC_TIMCDIER5; /** @brief TIMxDIER5. */
extern rw_reg32_t const HRTIM_TIMC_CNTCR;     /** @brief Timerx counter register. */
extern rw_reg32_t const HRTIM_TIMC_PERCR;     /** @brief Timerx period register. */
extern rw_reg32_t const HRTIM_TIMC_REPCR;     /** @brief Timerx repetition register. */
extern rw_reg32_t const HRTIM_TIMC_CMP1CR;    /** @brief Timerx compare 1 register. */
extern rw_reg32_t const HRTIM_TIMC_CMP1CCR;   /** @brief Timerx compare 1 compound register. */
extern rw_reg32_t const HRTIM_TIMC_CMP2CR;    /** @brief Timerx compare 2 register. */
extern rw_reg32_t const HRTIM_TIMC_CMP3CR;    /** @brief Timerx compare 3 register. */
extern rw_reg32_t const HRTIM_TIMC_CMP4CR;    /** @brief Timerx compare 4 register. */
extern ro_reg32_t const HRTIM_TIMC_CPT1CR;    /** @brief Timerx capture 1 register. */
extern ro_reg32_t const HRTIM_TIMC_CPT2CR;    /** @brief Timerx capture 2 register. */
extern rw_reg32_t const HRTIM_TIMC_DTCR;      /** @brief Timerx deadtime register. */
extern rw_reg32_t const HRTIM_TIMC_EEFCR1;    /** @brief Timerx external event filtering register 1. */
extern rw_reg32_t const HRTIM_TIMC_EEFCR2;    /** @brief Timerx external event filtering register 2. */
extern rw_reg32_t const HRTIM_TIMC_RSTCR;     /** @brief TimerA reset register. */
extern rw_reg32_t const HRTIM_TIMC_CHPCR;     /** @brief Timerx chopper register. */
extern rw_reg32_t const HRTIM_TIMC_OUTCR;     /** @brief Timerx output register. */
extern rw_reg32_t const HRTIM_TIMC_FLTCR;     /** @brief Timerx fault register. */

/** @subsection Enumerated HRTIM_TIMC Register Definitions */

extern rw_reg32_t const HRTIM_TIMC_SETCxR[3];  /** @brief Timerx output1 set register. */
extern rw_reg32_t const HRTIM_TIMC_RSTCxR[3];  /** @brief Timerx output1 reset register. */
extern rw_reg32_t const HRTIM_TIMC_CPTxCCR[3]; /** @brief Timerx capture 2 control register. */

/** @subsection HRTIM_TIMC Register Field Definitions */

extern const field32_t HRTIM_TIMC_TIMCCR_UPDGAT;      /** @brief Update gating. */
extern const field32_t HRTIM_TIMC_TIMCCR_PREEN;       /** @brief Preload enable. */
extern const field32_t HRTIM_TIMC_TIMCCR_DACSYNC;     /** @brief AC synchronization. */
extern const field32_t HRTIM_TIMC_TIMCCR_MSTU;        /** @brief Master timer update. */
extern const field32_t HRTIM_TIMC_TIMCCR_TEU;         /** @brief TEU. */
extern const field32_t HRTIM_TIMC_TIMCCR_TDU;         /** @brief TDU. */
extern const field32_t HRTIM_TIMC_TIMCCR_TCU;         /** @brief TCU. */
extern const field32_t HRTIM_TIMC_TIMCCR_TBU;         /** @brief TBU. */
extern const field32_t HRTIM_TIMC_TIMCCR_TXRSTU;      /** @brief Timerx reset update. */
extern const field32_t HRTIM_TIMC_TIMCCR_TXREPU;      /** @brief Timer x repetition update. */
extern const field32_t HRTIM_TIMC_TIMCCR_SYNCSTRTX;   /** @brief Synchronization starts timer x. */
extern const field32_t HRTIM_TIMC_TIMCCR_SYNCRSTX;    /** @brief Synchronization resets timer x. */
extern const field32_t HRTIM_TIMC_TIMCCR_PSHPLL;      /** @brief Push-Pull mode enable. */
extern const field32_t HRTIM_TIMC_TIMCCR_HALF;        /** @brief Half mode enable. */
extern const field32_t HRTIM_TIMC_TIMCCR_RETRIG;      /** @brief Re-triggerable mode. */
extern const field32_t HRTIM_TIMC_TIMCCR_CONT;        /** @brief Continuous mode. */
extern const field32_t HRTIM_TIMC_TIMCCR_CK_PSCX;     /** @brief HRTIM timer x clock prescaler. */
extern const field32_t HRTIM_TIMC_TIMCISR_IPPSTAT;    /** @brief Idle push pull status. */
extern const field32_t HRTIM_TIMC_TIMCISR_CPPSTAT;    /** @brief Current push pull status. */
extern const field32_t HRTIM_TIMC_TIMCISR_DLYPRT;     /** @brief Delayed protection flag. */
extern const field32_t HRTIM_TIMC_TIMCISR_RST;        /** @brief Reset interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCISR_UPD;        /** @brief Update interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCISR_REP;        /** @brief Repetition interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCICR_DLYPRTC;    /** @brief Delayed protection flag clear. */
extern const field32_t HRTIM_TIMC_TIMCICR_RSTC;       /** @brief Reset interrupt flag clear. */
extern const field32_t HRTIM_TIMC_TIMCICR_UPDC;       /** @brief Update interrupt flag clear. */
extern const field32_t HRTIM_TIMC_TIMCICR_REPC;       /** @brief Repetition interrupt flag clear. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_DLYPRTDE; /** @brief DLYPRTDE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_RSTDE;    /** @brief RSTDE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_SETX2DE;  /** @brief SETx2DE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_SET1XDE;  /** @brief SET1xDE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_UPDDE;    /** @brief UPDDE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_REPDE;    /** @brief REPDE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_DLYPRTIE; /** @brief DLYPRTIE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_RSTIE;    /** @brief RSTIE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_SETX2IE;  /** @brief SETx2IE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_SET1XIE;  /** @brief SET1xIE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_UPDIE;    /** @brief UPDIE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_REPIE;    /** @brief REPIE. */
extern const field32_t HRTIM_TIMC_CNTCR_CNTX;         /** @brief Timerx counter value. */
extern const field32_t HRTIM_TIMC_PERCR_PERX;         /** @brief Timerx period value. */
extern const field32_t HRTIM_TIMC_REPCR_REPX;         /** @brief Timerx repetition counter value. */
extern const field32_t HRTIM_TIMC_CMP1CR_CMP1X;       /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMC_CMP1CCR_REPX;       /** @brief Timerx repetition value (aliased from hrtim_repx register). */
extern const field32_t HRTIM_TIMC_CMP1CCR_CMP1X;      /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMC_CMP2CR_CMP2X;       /** @brief Timerx compare 2 value. */
extern const field32_t HRTIM_TIMC_CMP3CR_CMP3X;       /** @brief Timerx compare 3 value. */
extern const field32_t HRTIM_TIMC_CMP4CR_CMP4X;       /** @brief Timerx compare 4 value. */
extern const field32_t HRTIM_TIMC_CPT1CR_CPT1X;       /** @brief Timerx capture 1 value. */
extern const field32_t HRTIM_TIMC_CPT2CR_CPT2X;       /** @brief Timerx capture 2 value. */
extern const field32_t HRTIM_TIMC_DTCR_DTFLKX;        /** @brief Deadtime falling lock. */
extern const field32_t HRTIM_TIMC_DTCR_DTFSLKX;       /** @brief Deadtime falling sign lock. */
extern const field32_t HRTIM_TIMC_DTCR_SDTFX;         /** @brief Sign deadtime falling value. */
extern const field32_t HRTIM_TIMC_DTCR_DTFX;          /** @brief Deadtime falling value. */
extern const field32_t HRTIM_TIMC_DTCR_DTRLKX;        /** @brief Deadtime rising lock. */
extern const field32_t HRTIM_TIMC_DTCR_DTRSLKX;       /** @brief Deadtime rising sign lock. */
extern const field32_t HRTIM_TIMC_DTCR_DTPRSC;        /** @brief Deadtime prescaler. */
extern const field32_t HRTIM_TIMC_DTCR_SDTRX;         /** @brief Sign deadtime rising value. */
extern const field32_t HRTIM_TIMC_DTCR_DTRX;          /** @brief Deadtime rising value. */
extern const field32_t HRTIM_TIMC_SETCxR_UPDATE;      /** @brief Registers update (transfer preload to active). */
extern const field32_t HRTIM_TIMC_SETCxR_MSTPER;      /** @brief Master period. */
extern const field32_t HRTIM_TIMC_SETCxR_PER;         /** @brief Timer A period. */
extern const field32_t HRTIM_TIMC_SETCxR_RESYNC;      /** @brief Timer A resynchronizaton. */
extern const field32_t HRTIM_TIMC_SETCxR_SST;         /** @brief Software set trigger. */
extern const field32_t HRTIM_TIMC_RSTCxR_UPDATE;      /** @brief UPDATE. */
extern const field32_t HRTIM_TIMC_RSTCxR_MSTPER;      /** @brief MSTPER. */
extern const field32_t HRTIM_TIMC_RSTCxR_PER;         /** @brief PER. */
extern const field32_t HRTIM_TIMC_RSTCxR_RESYNC;      /** @brief RESYNC. */
extern const field32_t HRTIM_TIMC_RSTCxR_SRT;         /** @brief SRT. */
extern const field32_t HRTIM_TIMC_RSTCR_MSTPER;       /** @brief Master timer period. */
extern const field32_t HRTIM_TIMC_RSTCR_UPDT;         /** @brief Timer A update reset. */
extern const field32_t HRTIM_TIMC_CHPCR_STRTPW;       /** @brief STRTPW. */
extern const field32_t HRTIM_TIMC_CHPCR_CHPDTY;       /** @brief Timerx chopper duty cycle value. */
extern const field32_t HRTIM_TIMC_CHPCR_CHPFRQ;       /** @brief Timerx carrier frequency value. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TE1RST;     /** @brief Timer E output 1 reset. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TE1SET;     /** @brief Timer E output 1 set. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TD1RST;     /** @brief Timer D output 1 reset. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TD1SET;     /** @brief Timer D output 1 set. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TB1RST;     /** @brief Timer B output 1 reset. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TB1SET;     /** @brief Timer B output 1 set. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TA1RST;     /** @brief Timer A output 1 reset. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TA1SET;     /** @brief Timer A output 1 set. */
extern const field32_t HRTIM_TIMC_CPTxCCR_UDPCPT;     /** @brief Update capture. */
extern const field32_t HRTIM_TIMC_CPTxCCR_SWCPT;      /** @brief Software capture. */
extern const field32_t HRTIM_TIMC_OUTCR_DLYPRT;       /** @brief Delayed protection. */
extern const field32_t HRTIM_TIMC_OUTCR_DLYPRTEN;     /** @brief Delayed protection enable. */
extern const field32_t HRTIM_TIMC_OUTCR_DTEN;         /** @brief Deadtime enable. */
extern const field32_t HRTIM_TIMC_FLTCR_FLTLCK;       /** @brief Fault sources lock. */

/** @subsection Enumerated HRTIM_TIMC Register Field Definitions */

extern const field32_t HRTIM_TIMC_TIMCCR_DELCMPx[5];    /** @brief Delayed CMP2 mode. */
extern const field32_t HRTIM_TIMC_TIMCISR_OxSTAT[3];    /** @brief Output 1 state. */
extern const field32_t HRTIM_TIMC_TIMCISR_RSTXx[3];     /** @brief Output 1 reset interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCISR_SETXx[3];     /** @brief Output 1 set interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCISR_CPTx[3];      /** @brief Capture1 interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCISR_CMPx[5];      /** @brief Compare 1 interrupt flag. */
extern const field32_t HRTIM_TIMC_TIMCICR_RSTXxC[3];    /** @brief Output 1 reset flag clear. */
extern const field32_t HRTIM_TIMC_TIMCICR_SETxXC[3];    /** @brief Output 1 set flag clear. */
extern const field32_t HRTIM_TIMC_TIMCICR_CPTxC[3];     /** @brief Capture1 interrupt flag clear. */
extern const field32_t HRTIM_TIMC_TIMCICR_CMPxC[5];     /** @brief Compare 1 interrupt flag clear. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_RSTXxDE[3]; /** @brief RSTx1DE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_CPTxDE[3];  /** @brief CPT1DE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_CMPxDE[5];  /** @brief CMP1DE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_RSTXxIE[3]; /** @brief RSTx1IE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_CPTxIE[3];  /** @brief CPT1IE. */
extern const field32_t HRTIM_TIMC_TIMCDIER5_CMPxIE[5];  /** @brief CMP1IE. */
extern const field32_t HRTIM_TIMC_SETCxR_EXTEVNTx[11];  /** @brief External event 1. */
extern const field32_t HRTIM_TIMC_SETCxR_TIMEVNTx[10];  /** @brief Timer event 1. */
extern const field32_t HRTIM_TIMC_SETCxR_MSTCMPx[5];    /** @brief Master compare 1. */
extern const field32_t HRTIM_TIMC_EEFCR1_EExFLTR[6];    /** @brief External event 1 filter. */
extern const field32_t HRTIM_TIMC_EEFCR1_EExLTCH[6];    /** @brief External event 1 latch. */
extern const field32_t HRTIM_TIMC_RSTCR_TIMECMPx[5];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMC_RSTCR_TIMDCMPx[5];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIMC_RSTCR_TIMBCMPx[5];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIMC_RSTCR_TIMACMPx[5];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TECMPx[3];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TDCMPx[3];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TBCMPx[3];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIMC_CPTxCCR_TACMPx[3];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIMC_CPTxCCR_EXEVxCPT[11]; /** @brief External event 1 capture. */
extern const field32_t HRTIM_TIMC_OUTCR_DIDLx[3];       /** @brief Output 1 deadtime upon burst mode idle entry. */
extern const field32_t HRTIM_TIMC_OUTCR_CHPx[3];        /** @brief Output 1 chopper enable. */
extern const field32_t HRTIM_TIMC_OUTCR_FAULTx[3];      /** @brief Output 1 fault state. */
extern const field32_t HRTIM_TIMC_OUTCR_IDLESx[3];      /** @brief Output 1 idle state. */
extern const field32_t HRTIM_TIMC_OUTCR_IDLEMx[3];      /** @brief Output 1 idle mode. */
extern const field32_t HRTIM_TIMC_OUTCR_POLx[3];        /** @brief Output 1 polarity. */
extern const field32_t HRTIM_TIMC_FLTCR_FLTxEN[6];      /** @brief Fault 1 enable. */

/**************************************************************************************************
 * @section HRTIM_TIMD Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMD Register Definitions */

extern rw_reg32_t const HRTIM_TIMD_TIMDCR;    /** @brief Timerx control register. */
extern ro_reg32_t const HRTIM_TIMD_TIMDISR;   /** @brief Timerx interrupt status register. */
extern rw_reg32_t const HRTIM_TIMD_TIMDICR;   /** @brief Timerx interrupt clear register. */
extern rw_reg32_t const HRTIM_TIMD_TIMDDIER5; /** @brief TIMxDIER5. */
extern rw_reg32_t const HRTIM_TIMD_CNTDR;     /** @brief Timerx counter register. */
extern rw_reg32_t const HRTIM_TIMD_PERDR;     /** @brief Timerx period register. */
extern rw_reg32_t const HRTIM_TIMD_REPDR;     /** @brief Timerx repetition register. */
extern rw_reg32_t const HRTIM_TIMD_CMP1DR;    /** @brief Timerx compare 1 register. */
extern rw_reg32_t const HRTIM_TIMD_CMP1CDR;   /** @brief Timerx compare 1 compound register. */
extern rw_reg32_t const HRTIM_TIMD_CMP2DR;    /** @brief Timerx compare 2 register. */
extern rw_reg32_t const HRTIM_TIMD_CMP3DR;    /** @brief Timerx compare 3 register. */
extern rw_reg32_t const HRTIM_TIMD_CMP4DR;    /** @brief Timerx compare 4 register. */
extern ro_reg32_t const HRTIM_TIMD_CPT1DR;    /** @brief Timerx capture 1 register. */
extern ro_reg32_t const HRTIM_TIMD_CPT2DR;    /** @brief Timerx capture 2 register. */
extern rw_reg32_t const HRTIM_TIMD_DTDR;      /** @brief Timerx deadtime register. */
extern rw_reg32_t const HRTIM_TIMD_EEFDR1;    /** @brief Timerx external event filtering register 1. */
extern rw_reg32_t const HRTIM_TIMD_EEFDR2;    /** @brief Timerx external event filtering register 2. */
extern rw_reg32_t const HRTIM_TIMD_RSTDR;     /** @brief TimerA reset register. */
extern rw_reg32_t const HRTIM_TIMD_CHPDR;     /** @brief Timerx chopper register. */
extern rw_reg32_t const HRTIM_TIMD_OUTDR;     /** @brief Timerx output register. */
extern rw_reg32_t const HRTIM_TIMD_FLTDR;     /** @brief Timerx fault register. */

/** @subsection Enumerated HRTIM_TIMD Register Definitions */

extern rw_reg32_t const HRTIM_TIMD_SETDxR[3];  /** @brief Timerx output1 set register. */
extern rw_reg32_t const HRTIM_TIMD_RSTDxR[3];  /** @brief Timerx output1 reset register. */
extern rw_reg32_t const HRTIM_TIMD_CPTxDCR[3]; /** @brief Timerx capture 2 control register. */

/** @subsection HRTIM_TIMD Register Field Definitions */

extern const field32_t HRTIM_TIMD_TIMDCR_UPDGAT;      /** @brief Update gating. */
extern const field32_t HRTIM_TIMD_TIMDCR_PREEN;       /** @brief Preload enable. */
extern const field32_t HRTIM_TIMD_TIMDCR_DACSYNC;     /** @brief AC synchronization. */
extern const field32_t HRTIM_TIMD_TIMDCR_MSTU;        /** @brief Master timer update. */
extern const field32_t HRTIM_TIMD_TIMDCR_TEU;         /** @brief TEU. */
extern const field32_t HRTIM_TIMD_TIMDCR_TDU;         /** @brief TDU. */
extern const field32_t HRTIM_TIMD_TIMDCR_TCU;         /** @brief TCU. */
extern const field32_t HRTIM_TIMD_TIMDCR_TBU;         /** @brief TBU. */
extern const field32_t HRTIM_TIMD_TIMDCR_TXRSTU;      /** @brief Timerx reset update. */
extern const field32_t HRTIM_TIMD_TIMDCR_TXREPU;      /** @brief Timer x repetition update. */
extern const field32_t HRTIM_TIMD_TIMDCR_SYNCSTRTX;   /** @brief Synchronization starts timer x. */
extern const field32_t HRTIM_TIMD_TIMDCR_SYNCRSTX;    /** @brief Synchronization resets timer x. */
extern const field32_t HRTIM_TIMD_TIMDCR_PSHPLL;      /** @brief Push-Pull mode enable. */
extern const field32_t HRTIM_TIMD_TIMDCR_HALF;        /** @brief Half mode enable. */
extern const field32_t HRTIM_TIMD_TIMDCR_RETRIG;      /** @brief Re-triggerable mode. */
extern const field32_t HRTIM_TIMD_TIMDCR_CONT;        /** @brief Continuous mode. */
extern const field32_t HRTIM_TIMD_TIMDCR_CK_PSCX;     /** @brief HRTIM timer x clock prescaler. */
extern const field32_t HRTIM_TIMD_TIMDISR_IPPSTAT;    /** @brief Idle push pull status. */
extern const field32_t HRTIM_TIMD_TIMDISR_CPPSTAT;    /** @brief Current push pull status. */
extern const field32_t HRTIM_TIMD_TIMDISR_DLYPRT;     /** @brief Delayed protection flag. */
extern const field32_t HRTIM_TIMD_TIMDISR_RST;        /** @brief Reset interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDISR_UPD;        /** @brief Update interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDISR_REP;        /** @brief Repetition interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDICR_DLYPRTC;    /** @brief Delayed protection flag clear. */
extern const field32_t HRTIM_TIMD_TIMDICR_RSTC;       /** @brief Reset interrupt flag clear. */
extern const field32_t HRTIM_TIMD_TIMDICR_UPDC;       /** @brief Update interrupt flag clear. */
extern const field32_t HRTIM_TIMD_TIMDICR_REPC;       /** @brief Repetition interrupt flag clear. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_DLYPRTDE; /** @brief DLYPRTDE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_RSTDE;    /** @brief RSTDE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_SETX2DE;  /** @brief SETx2DE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_SET1XDE;  /** @brief SET1xDE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_UPDDE;    /** @brief UPDDE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_REPDE;    /** @brief REPDE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_DLYPRTIE; /** @brief DLYPRTIE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_RSTIE;    /** @brief RSTIE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_SETX2IE;  /** @brief SETx2IE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_SET1XIE;  /** @brief SET1xIE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_UPDIE;    /** @brief UPDIE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_REPIE;    /** @brief REPIE. */
extern const field32_t HRTIM_TIMD_CNTDR_CNTX;         /** @brief Timerx counter value. */
extern const field32_t HRTIM_TIMD_PERDR_PERX;         /** @brief Timerx period value. */
extern const field32_t HRTIM_TIMD_REPDR_REPX;         /** @brief Timerx repetition counter value. */
extern const field32_t HRTIM_TIMD_CMP1DR_CMP1X;       /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMD_CMP1CDR_REPX;       /** @brief Timerx repetition value (aliased from hrtim_repx register). */
extern const field32_t HRTIM_TIMD_CMP1CDR_CMP1X;      /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIMD_CMP2DR_CMP2X;       /** @brief Timerx compare 2 value. */
extern const field32_t HRTIM_TIMD_CMP3DR_CMP3X;       /** @brief Timerx compare 3 value. */
extern const field32_t HRTIM_TIMD_CMP4DR_CMP4X;       /** @brief Timerx compare 4 value. */
extern const field32_t HRTIM_TIMD_CPT1DR_CPT1X;       /** @brief Timerx capture 1 value. */
extern const field32_t HRTIM_TIMD_CPT2DR_CPT2X;       /** @brief Timerx capture 2 value. */
extern const field32_t HRTIM_TIMD_DTDR_DTFLKX;        /** @brief Deadtime falling lock. */
extern const field32_t HRTIM_TIMD_DTDR_DTFSLKX;       /** @brief Deadtime falling sign lock. */
extern const field32_t HRTIM_TIMD_DTDR_SDTFX;         /** @brief Sign deadtime falling value. */
extern const field32_t HRTIM_TIMD_DTDR_DTFX;          /** @brief Deadtime falling value. */
extern const field32_t HRTIM_TIMD_DTDR_DTRLKX;        /** @brief Deadtime rising lock. */
extern const field32_t HRTIM_TIMD_DTDR_DTRSLKX;       /** @brief Deadtime rising sign lock. */
extern const field32_t HRTIM_TIMD_DTDR_DTPRSC;        /** @brief Deadtime prescaler. */
extern const field32_t HRTIM_TIMD_DTDR_SDTRX;         /** @brief Sign deadtime rising value. */
extern const field32_t HRTIM_TIMD_DTDR_DTRX;          /** @brief Deadtime rising value. */
extern const field32_t HRTIM_TIMD_SETDxR_UPDATE;      /** @brief Registers update (transfer preload to active). */
extern const field32_t HRTIM_TIMD_SETDxR_MSTPER;      /** @brief Master period. */
extern const field32_t HRTIM_TIMD_SETDxR_PER;         /** @brief Timer A period. */
extern const field32_t HRTIM_TIMD_SETDxR_RESYNC;      /** @brief Timer A resynchronizaton. */
extern const field32_t HRTIM_TIMD_SETDxR_SST;         /** @brief Software set trigger. */
extern const field32_t HRTIM_TIMD_RSTDxR_UPDATE;      /** @brief UPDATE. */
extern const field32_t HRTIM_TIMD_RSTDxR_MSTPER;      /** @brief MSTPER. */
extern const field32_t HRTIM_TIMD_RSTDxR_PER;         /** @brief PER. */
extern const field32_t HRTIM_TIMD_RSTDxR_RESYNC;      /** @brief RESYNC. */
extern const field32_t HRTIM_TIMD_RSTDxR_SRT;         /** @brief SRT. */
extern const field32_t HRTIM_TIMD_RSTDR_MSTPER;       /** @brief Master timer period. */
extern const field32_t HRTIM_TIMD_RSTDR_UPDT;         /** @brief Timer A update reset. */
extern const field32_t HRTIM_TIMD_CHPDR_STRTPW;       /** @brief STRTPW. */
extern const field32_t HRTIM_TIMD_CHPDR_CHPDTY;       /** @brief Timerx chopper duty cycle value. */
extern const field32_t HRTIM_TIMD_CHPDR_CHPFRQ;       /** @brief Timerx carrier frequency value. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TE1RST;     /** @brief Timer E output 1 reset. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TE1SET;     /** @brief Timer E output 1 set. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TC1RST;     /** @brief Timer C output 1 reset. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TC1SET;     /** @brief Timer C output 1 set. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TB1RST;     /** @brief Timer B output 1 reset. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TB1SET;     /** @brief Timer B output 1 set. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TA1RST;     /** @brief Timer A output 1 reset. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TA1SET;     /** @brief Timer A output 1 set. */
extern const field32_t HRTIM_TIMD_CPTxDCR_UDPCPT;     /** @brief Update capture. */
extern const field32_t HRTIM_TIMD_CPTxDCR_SWCPT;      /** @brief Software capture. */
extern const field32_t HRTIM_TIMD_OUTDR_DLYPRT;       /** @brief Delayed protection. */
extern const field32_t HRTIM_TIMD_OUTDR_DLYPRTEN;     /** @brief Delayed protection enable. */
extern const field32_t HRTIM_TIMD_OUTDR_DTEN;         /** @brief Deadtime enable. */
extern const field32_t HRTIM_TIMD_FLTDR_FLTLCK;       /** @brief Fault sources lock. */

/** @subsection Enumerated HRTIM_TIMD Register Field Definitions */

extern const field32_t HRTIM_TIMD_TIMDCR_DELCMPx[5];    /** @brief Delayed CMP2 mode. */
extern const field32_t HRTIM_TIMD_TIMDISR_OxSTAT[3];    /** @brief Output 1 state. */
extern const field32_t HRTIM_TIMD_TIMDISR_RSTXx[3];     /** @brief Output 1 reset interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDISR_SETXx[3];     /** @brief Output 1 set interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDISR_CPTx[3];      /** @brief Capture1 interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDISR_CMPx[5];      /** @brief Compare 1 interrupt flag. */
extern const field32_t HRTIM_TIMD_TIMDICR_RSTXxC[3];    /** @brief Output 1 reset flag clear. */
extern const field32_t HRTIM_TIMD_TIMDICR_SETxXC[3];    /** @brief Output 1 set flag clear. */
extern const field32_t HRTIM_TIMD_TIMDICR_CPTxC[3];     /** @brief Capture1 interrupt flag clear. */
extern const field32_t HRTIM_TIMD_TIMDICR_CMPxC[5];     /** @brief Compare 1 interrupt flag clear. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_RSTXxDE[3]; /** @brief RSTx1DE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_CPTxDE[3];  /** @brief CPT1DE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_CMPxDE[5];  /** @brief CMP1DE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_RSTXxIE[3]; /** @brief RSTx1IE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_CPTxIE[3];  /** @brief CPT1IE. */
extern const field32_t HRTIM_TIMD_TIMDDIER5_CMPxIE[5];  /** @brief CMP1IE. */
extern const field32_t HRTIM_TIMD_SETDxR_EXTEVNTx[11];  /** @brief External event 1. */
extern const field32_t HRTIM_TIMD_SETDxR_TIMEVNTx[10];  /** @brief Timer event 1. */
extern const field32_t HRTIM_TIMD_SETDxR_MSTCMPx[5];    /** @brief Master compare 1. */
extern const field32_t HRTIM_TIMD_EEFDR1_EExFLTR[6];    /** @brief External event 1 filter. */
extern const field32_t HRTIM_TIMD_EEFDR1_EExLTCH[6];    /** @brief External event 1 latch. */
extern const field32_t HRTIM_TIMD_RSTDR_TIMECMPx[5];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMD_RSTDR_TIMCCMPx[5];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIMD_RSTDR_TIMBCMPx[5];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIMD_RSTDR_TIMACMPx[5];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TECMPx[3];    /** @brief Timer E compare 1. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TCCMPx[3];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TBCMPx[3];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIMD_CPTxDCR_TACMPx[3];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIMD_CPTxDCR_EXEVxCPT[11]; /** @brief External event 1 capture. */
extern const field32_t HRTIM_TIMD_OUTDR_DIDLx[3];       /** @brief Output 1 deadtime upon burst mode idle entry. */
extern const field32_t HRTIM_TIMD_OUTDR_CHPx[3];        /** @brief Output 1 chopper enable. */
extern const field32_t HRTIM_TIMD_OUTDR_FAULTx[3];      /** @brief Output 1 fault state. */
extern const field32_t HRTIM_TIMD_OUTDR_IDLESx[3];      /** @brief Output 1 idle state. */
extern const field32_t HRTIM_TIMD_OUTDR_IDLEMx[3];      /** @brief Output 1 idle mode. */
extern const field32_t HRTIM_TIMD_OUTDR_POLx[3];        /** @brief Output 1 polarity. */
extern const field32_t HRTIM_TIMD_FLTDR_FLTxEN[6];      /** @brief Fault 1 enable. */

/**************************************************************************************************
 * @section HRTIM_TIME Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIME Register Definitions */

extern rw_reg32_t const HRTIM_TIME_TIMECR;    /** @brief Timerx control register. */
extern ro_reg32_t const HRTIM_TIME_TIMEISR;   /** @brief Timerx interrupt status register. */
extern rw_reg32_t const HRTIM_TIME_TIMEICR;   /** @brief Timerx interrupt clear register. */
extern rw_reg32_t const HRTIM_TIME_TIMEDIER5; /** @brief TIMxDIER5. */
extern rw_reg32_t const HRTIM_TIME_CNTER;     /** @brief Timerx counter register. */
extern rw_reg32_t const HRTIM_TIME_PERER;     /** @brief Timerx period register. */
extern rw_reg32_t const HRTIM_TIME_REPER;     /** @brief Timerx repetition register. */
extern rw_reg32_t const HRTIM_TIME_CMP1ER;    /** @brief Timerx compare 1 register. */
extern rw_reg32_t const HRTIM_TIME_CMP1CER;   /** @brief Timerx compare 1 compound register. */
extern rw_reg32_t const HRTIM_TIME_CMP2ER;    /** @brief Timerx compare 2 register. */
extern rw_reg32_t const HRTIM_TIME_CMP3ER;    /** @brief Timerx compare 3 register. */
extern rw_reg32_t const HRTIM_TIME_CMP4ER;    /** @brief Timerx compare 4 register. */
extern ro_reg32_t const HRTIM_TIME_CPT1ER;    /** @brief Timerx capture 1 register. */
extern ro_reg32_t const HRTIM_TIME_CPT2ER;    /** @brief Timerx capture 2 register. */
extern rw_reg32_t const HRTIM_TIME_DTER;      /** @brief Timerx deadtime register. */
extern rw_reg32_t const HRTIM_TIME_EEFER1;    /** @brief Timerx external event filtering register 1. */
extern rw_reg32_t const HRTIM_TIME_EEFER2;    /** @brief Timerx external event filtering register 2. */
extern rw_reg32_t const HRTIM_TIME_RSTER;     /** @brief TimerA reset register. */
extern rw_reg32_t const HRTIM_TIME_CHPER;     /** @brief Timerx chopper register. */
extern rw_reg32_t const HRTIM_TIME_OUTER;     /** @brief Timerx output register. */
extern rw_reg32_t const HRTIM_TIME_FLTER;     /** @brief Timerx fault register. */

/** @subsection Enumerated HRTIM_TIME Register Definitions */

extern rw_reg32_t const HRTIM_TIME_SETExR[3];  /** @brief Timerx output1 set register. */
extern rw_reg32_t const HRTIM_TIME_RSTExR[3];  /** @brief Timerx output1 reset register. */
extern rw_reg32_t const HRTIM_TIME_CPTxECR[3]; /** @brief Timerx capture 2 control register. */

/** @subsection HRTIM_TIME Register Field Definitions */

extern const field32_t HRTIM_TIME_TIMECR_UPDGAT;      /** @brief Update gating. */
extern const field32_t HRTIM_TIME_TIMECR_PREEN;       /** @brief Preload enable. */
extern const field32_t HRTIM_TIME_TIMECR_DACSYNC;     /** @brief AC synchronization. */
extern const field32_t HRTIM_TIME_TIMECR_MSTU;        /** @brief Master timer update. */
extern const field32_t HRTIM_TIME_TIMECR_TEU;         /** @brief TEU. */
extern const field32_t HRTIM_TIME_TIMECR_TDU;         /** @brief TDU. */
extern const field32_t HRTIM_TIME_TIMECR_TCU;         /** @brief TCU. */
extern const field32_t HRTIM_TIME_TIMECR_TBU;         /** @brief TBU. */
extern const field32_t HRTIM_TIME_TIMECR_TXRSTU;      /** @brief Timerx reset update. */
extern const field32_t HRTIM_TIME_TIMECR_TXREPU;      /** @brief Timer x repetition update. */
extern const field32_t HRTIM_TIME_TIMECR_SYNCSTRTX;   /** @brief Synchronization starts timer x. */
extern const field32_t HRTIM_TIME_TIMECR_SYNCRSTX;    /** @brief Synchronization resets timer x. */
extern const field32_t HRTIM_TIME_TIMECR_PSHPLL;      /** @brief Push-Pull mode enable. */
extern const field32_t HRTIM_TIME_TIMECR_HALF;        /** @brief Half mode enable. */
extern const field32_t HRTIM_TIME_TIMECR_RETRIG;      /** @brief Re-triggerable mode. */
extern const field32_t HRTIM_TIME_TIMECR_CONT;        /** @brief Continuous mode. */
extern const field32_t HRTIM_TIME_TIMECR_CK_PSCX;     /** @brief HRTIM timer x clock prescaler. */
extern const field32_t HRTIM_TIME_TIMEISR_IPPSTAT;    /** @brief Idle push pull status. */
extern const field32_t HRTIM_TIME_TIMEISR_CPPSTAT;    /** @brief Current push pull status. */
extern const field32_t HRTIM_TIME_TIMEISR_DLYPRT;     /** @brief Delayed protection flag. */
extern const field32_t HRTIM_TIME_TIMEISR_RST;        /** @brief Reset interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEISR_UPD;        /** @brief Update interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEISR_REP;        /** @brief Repetition interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEICR_DLYPRTC;    /** @brief Delayed protection flag clear. */
extern const field32_t HRTIM_TIME_TIMEICR_RSTC;       /** @brief Reset interrupt flag clear. */
extern const field32_t HRTIM_TIME_TIMEICR_UPDC;       /** @brief Update interrupt flag clear. */
extern const field32_t HRTIM_TIME_TIMEICR_REPC;       /** @brief Repetition interrupt flag clear. */
extern const field32_t HRTIM_TIME_TIMEDIER5_DLYPRTDE; /** @brief DLYPRTDE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_RSTDE;    /** @brief RSTDE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_SETX2DE;  /** @brief SETx2DE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_SET1XDE;  /** @brief SET1xDE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_UPDDE;    /** @brief UPDDE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_REPDE;    /** @brief REPDE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_DLYPRTIE; /** @brief DLYPRTIE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_RSTIE;    /** @brief RSTIE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_SETX2IE;  /** @brief SETx2IE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_SET1XIE;  /** @brief SET1xIE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_UPDIE;    /** @brief UPDIE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_REPIE;    /** @brief REPIE. */
extern const field32_t HRTIM_TIME_CNTER_CNTX;         /** @brief Timerx counter value. */
extern const field32_t HRTIM_TIME_PERER_PERX;         /** @brief Timerx period value. */
extern const field32_t HRTIM_TIME_REPER_REPX;         /** @brief Timerx repetition counter value. */
extern const field32_t HRTIM_TIME_CMP1ER_CMP1X;       /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIME_CMP1CER_REPX;       /** @brief Timerx repetition value (aliased from hrtim_repx register). */
extern const field32_t HRTIM_TIME_CMP1CER_CMP1X;      /** @brief Timerx compare 1 value. */
extern const field32_t HRTIM_TIME_CMP2ER_CMP2X;       /** @brief Timerx compare 2 value. */
extern const field32_t HRTIM_TIME_CMP3ER_CMP3X;       /** @brief Timerx compare 3 value. */
extern const field32_t HRTIM_TIME_CMP4ER_CMP4X;       /** @brief Timerx compare 4 value. */
extern const field32_t HRTIM_TIME_CPT1ER_CPT1X;       /** @brief Timerx capture 1 value. */
extern const field32_t HRTIM_TIME_CPT2ER_CPT2X;       /** @brief Timerx capture 2 value. */
extern const field32_t HRTIM_TIME_DTER_DTFLKX;        /** @brief Deadtime falling lock. */
extern const field32_t HRTIM_TIME_DTER_DTFSLKX;       /** @brief Deadtime falling sign lock. */
extern const field32_t HRTIM_TIME_DTER_SDTFX;         /** @brief Sign deadtime falling value. */
extern const field32_t HRTIM_TIME_DTER_DTFX;          /** @brief Deadtime falling value. */
extern const field32_t HRTIM_TIME_DTER_DTRLKX;        /** @brief Deadtime rising lock. */
extern const field32_t HRTIM_TIME_DTER_DTRSLKX;       /** @brief Deadtime rising sign lock. */
extern const field32_t HRTIM_TIME_DTER_DTPRSC;        /** @brief Deadtime prescaler. */
extern const field32_t HRTIM_TIME_DTER_SDTRX;         /** @brief Sign deadtime rising value. */
extern const field32_t HRTIM_TIME_DTER_DTRX;          /** @brief Deadtime rising value. */
extern const field32_t HRTIM_TIME_SETExR_UPDATE;      /** @brief Registers update (transfer preload to active). */
extern const field32_t HRTIM_TIME_SETExR_MSTPER;      /** @brief Master period. */
extern const field32_t HRTIM_TIME_SETExR_PER;         /** @brief Timer A period. */
extern const field32_t HRTIM_TIME_SETExR_RESYNC;      /** @brief Timer A resynchronizaton. */
extern const field32_t HRTIM_TIME_SETExR_SST;         /** @brief Software set trigger. */
extern const field32_t HRTIM_TIME_RSTExR_UPDATE;      /** @brief UPDATE. */
extern const field32_t HRTIM_TIME_RSTExR_MSTPER;      /** @brief MSTPER. */
extern const field32_t HRTIM_TIME_RSTExR_PER;         /** @brief PER. */
extern const field32_t HRTIM_TIME_RSTExR_RESYNC;      /** @brief RESYNC. */
extern const field32_t HRTIM_TIME_RSTExR_SRT;         /** @brief SRT. */
extern const field32_t HRTIM_TIME_RSTER_MSTPER;       /** @brief Master timer period. */
extern const field32_t HRTIM_TIME_RSTER_UPDT;         /** @brief Timer A update reset. */
extern const field32_t HRTIM_TIME_CHPER_STRTPW;       /** @brief STRTPW. */
extern const field32_t HRTIM_TIME_CHPER_CHPDTY;       /** @brief Timerx chopper duty cycle value. */
extern const field32_t HRTIM_TIME_CHPER_CHPFRQ;       /** @brief Timerx carrier frequency value. */
extern const field32_t HRTIM_TIME_CPTxECR_TD1RST;     /** @brief Timer D output 1 reset. */
extern const field32_t HRTIM_TIME_CPTxECR_TD1SET;     /** @brief Timer D output 1 set. */
extern const field32_t HRTIM_TIME_CPTxECR_TC1RST;     /** @brief Timer C output 1 reset. */
extern const field32_t HRTIM_TIME_CPTxECR_TC1SET;     /** @brief Timer C output 1 set. */
extern const field32_t HRTIM_TIME_CPTxECR_TB1RST;     /** @brief Timer B output 1 reset. */
extern const field32_t HRTIM_TIME_CPTxECR_TB1SET;     /** @brief Timer B output 1 set. */
extern const field32_t HRTIM_TIME_CPTxECR_TA1RST;     /** @brief Timer A output 1 reset. */
extern const field32_t HRTIM_TIME_CPTxECR_TA1SET;     /** @brief Timer A output 1 set. */
extern const field32_t HRTIM_TIME_CPTxECR_UDPCPT;     /** @brief Update capture. */
extern const field32_t HRTIM_TIME_CPTxECR_SWCPT;      /** @brief Software capture. */
extern const field32_t HRTIM_TIME_OUTER_DLYPRT;       /** @brief Delayed protection. */
extern const field32_t HRTIM_TIME_OUTER_DLYPRTEN;     /** @brief Delayed protection enable. */
extern const field32_t HRTIM_TIME_OUTER_DTEN;         /** @brief Deadtime enable. */
extern const field32_t HRTIM_TIME_FLTER_FLTLCK;       /** @brief Fault sources lock. */

/** @subsection Enumerated HRTIM_TIME Register Field Definitions */

extern const field32_t HRTIM_TIME_TIMECR_DELCMPx[5];    /** @brief Delayed CMP2 mode. */
extern const field32_t HRTIM_TIME_TIMEISR_OxSTAT[3];    /** @brief Output 1 state. */
extern const field32_t HRTIM_TIME_TIMEISR_RSTXx[3];     /** @brief Output 1 reset interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEISR_SETXx[3];     /** @brief Output 1 set interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEISR_CPTx[3];      /** @brief Capture1 interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEISR_CMPx[5];      /** @brief Compare 1 interrupt flag. */
extern const field32_t HRTIM_TIME_TIMEICR_RSTXxC[3];    /** @brief Output 1 reset flag clear. */
extern const field32_t HRTIM_TIME_TIMEICR_SETxXC[3];    /** @brief Output 1 set flag clear. */
extern const field32_t HRTIM_TIME_TIMEICR_CPTxC[3];     /** @brief Capture1 interrupt flag clear. */
extern const field32_t HRTIM_TIME_TIMEICR_CMPxC[5];     /** @brief Compare 1 interrupt flag clear. */
extern const field32_t HRTIM_TIME_TIMEDIER5_RSTXxDE[3]; /** @brief RSTx1DE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_CPTxDE[3];  /** @brief CPT1DE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_CMPxDE[5];  /** @brief CMP1DE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_RSTXxIE[3]; /** @brief RSTx1IE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_CPTxIE[3];  /** @brief CPT1IE. */
extern const field32_t HRTIM_TIME_TIMEDIER5_CMPxIE[5];  /** @brief CMP1IE. */
extern const field32_t HRTIM_TIME_SETExR_EXTEVNTx[11];  /** @brief External event 1. */
extern const field32_t HRTIM_TIME_SETExR_TIMEVNTx[10];  /** @brief Timer event 1. */
extern const field32_t HRTIM_TIME_SETExR_MSTCMPx[5];    /** @brief Master compare 1. */
extern const field32_t HRTIM_TIME_EEFER1_EExFLTR[6];    /** @brief External event 1 filter. */
extern const field32_t HRTIM_TIME_EEFER1_EExLTCH[6];    /** @brief External event 1 latch. */
extern const field32_t HRTIM_TIME_RSTER_TIMDCMPx[5];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIME_RSTER_TIMCCMPx[5];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIME_RSTER_TIMBCMPx[5];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIME_RSTER_TIMACMPx[5];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIME_CPTxECR_TDCMPx[3];    /** @brief Timer D compare 1. */
extern const field32_t HRTIM_TIME_CPTxECR_TCCMPx[3];    /** @brief Timer C compare 1. */
extern const field32_t HRTIM_TIME_CPTxECR_TBCMPx[3];    /** @brief Timer B compare 1. */
extern const field32_t HRTIM_TIME_CPTxECR_TACMPx[3];    /** @brief Timer A compare 1. */
extern const field32_t HRTIM_TIME_CPTxECR_EXEVxCPT[11]; /** @brief External event 1 capture. */
extern const field32_t HRTIM_TIME_OUTER_DIDLx[3];       /** @brief Output 1 deadtime upon burst mode idle entry. */
extern const field32_t HRTIM_TIME_OUTER_CHPx[3];        /** @brief Output 1 chopper enable. */
extern const field32_t HRTIM_TIME_OUTER_FAULTx[3];      /** @brief Output 1 fault state. */
extern const field32_t HRTIM_TIME_OUTER_IDLESx[3];      /** @brief Output 1 idle state. */
extern const field32_t HRTIM_TIME_OUTER_IDLEMx[3];      /** @brief Output 1 idle mode. */
extern const field32_t HRTIM_TIME_OUTER_POLx[3];        /** @brief Output 1 polarity. */
extern const field32_t HRTIM_TIME_FLTER_FLTxEN[6];      /** @brief Fault 1 enable. */

/**************************************************************************************************
 * @section HRTIM_COMMON Definitions
 **************************************************************************************************/

/** @subsection HRTIM_COMMON Register Definitions */

extern rw_reg32_t const HRTIM_COMMON_CR1;     /** @brief Control register 1. */
extern rw_reg32_t const HRTIM_COMMON_CR2;     /** @brief Control register 2. */
extern rw_reg32_t const HRTIM_COMMON_ISR;     /** @brief Interrupt status register. */
extern rw_reg32_t const HRTIM_COMMON_ICR;     /** @brief Interrupt clear register. */
extern rw_reg32_t const HRTIM_COMMON_IER;     /** @brief Interrupt enable register. */
extern rw_reg32_t const HRTIM_COMMON_OENR;    /** @brief Output enable register. */
extern rw_reg32_t const HRTIM_COMMON_DISR;    /** @brief DISR. */
extern ro_reg32_t const HRTIM_COMMON_ODSR;    /** @brief Output disable status register. */
extern rw_reg32_t const HRTIM_COMMON_BMCR;    /** @brief Burst mode control register. */
extern rw_reg32_t const HRTIM_COMMON_BMTRG;   /** @brief BMTRG. */
extern rw_reg32_t const HRTIM_COMMON_BMCMPR6; /** @brief BMCMPR6. */
extern rw_reg32_t const HRTIM_COMMON_BMPER;   /** @brief Burst mode period register. */
extern rw_reg32_t const HRTIM_COMMON_EECR1;   /** @brief Timer external event control register 1. */
extern rw_reg32_t const HRTIM_COMMON_EECR2;   /** @brief Timer external event control register 2. */
extern rw_reg32_t const HRTIM_COMMON_EECR3;   /** @brief Timer external event control register 3. */
extern rw_reg32_t const HRTIM_COMMON_ADC1R;   /** @brief ADC trigger 1 register. */
extern rw_reg32_t const HRTIM_COMMON_ADC2R;   /** @brief ADC trigger 2 register. */
extern rw_reg32_t const HRTIM_COMMON_ADC3R;   /** @brief ADC trigger 3 register. */
extern rw_reg32_t const HRTIM_COMMON_ADC4R;   /** @brief ADC trigger 4 register. */
extern rw_reg32_t const HRTIM_COMMON_DLLCR;   /** @brief DLL control register. */
extern rw_reg32_t const HRTIM_COMMON_FLTINR1; /** @brief HRTIM fault input register 1. */
extern rw_reg32_t const HRTIM_COMMON_FLTINR2; /** @brief HRTIM fault input register 2. */
extern rw_reg32_t const HRTIM_COMMON_BDMUPDR; /** @brief BDMUPDR. */
extern rw_reg32_t const HRTIM_COMMON_BDTXUPR; /** @brief Burst DMA timerx update register. */
extern rw_reg32_t const HRTIM_COMMON_BDMADR;  /** @brief Burst DMA data register. */

/** @subsection HRTIM_COMMON Register Field Definitions */

extern const field32_t HRTIM_COMMON_CR1_TEUDIS;        /** @brief Timer E update disable. */
extern const field32_t HRTIM_COMMON_CR1_TDUDIS;        /** @brief Timer D update disable. */
extern const field32_t HRTIM_COMMON_CR1_TCUDIS;        /** @brief Timer C update disable. */
extern const field32_t HRTIM_COMMON_CR1_TBUDIS;        /** @brief Timer B update disable. */
extern const field32_t HRTIM_COMMON_CR1_TAUDIS;        /** @brief Timer A update disable. */
extern const field32_t HRTIM_COMMON_CR1_MUDIS;         /** @brief Master update disable. */
extern const field32_t HRTIM_COMMON_CR2_TERST;         /** @brief Timer E counter software reset. */
extern const field32_t HRTIM_COMMON_CR2_TDRST;         /** @brief Timer D counter software reset. */
extern const field32_t HRTIM_COMMON_CR2_TCRST;         /** @brief Timer C counter software reset. */
extern const field32_t HRTIM_COMMON_CR2_TBRST;         /** @brief Timer B counter software reset. */
extern const field32_t HRTIM_COMMON_CR2_TARST;         /** @brief Timer A counter software reset. */
extern const field32_t HRTIM_COMMON_CR2_MRST;          /** @brief Master counter software reset. */
extern const field32_t HRTIM_COMMON_CR2_TESWU;         /** @brief Timer E software update. */
extern const field32_t HRTIM_COMMON_CR2_TDSWU;         /** @brief Timer D software update. */
extern const field32_t HRTIM_COMMON_CR2_TCSWU;         /** @brief Timer C software update. */
extern const field32_t HRTIM_COMMON_CR2_TBSWU;         /** @brief Timer B software update. */
extern const field32_t HRTIM_COMMON_CR2_TASWU;         /** @brief Timer A software update. */
extern const field32_t HRTIM_COMMON_CR2_MSWU;          /** @brief Master timer software update. */
extern const field32_t HRTIM_COMMON_ISR_BMPER;         /** @brief Burst mode period interrupt flag. */
extern const field32_t HRTIM_COMMON_ISR_DLLRDY;        /** @brief DLL ready interrupt flag. */
extern const field32_t HRTIM_COMMON_ISR_SYSFLT;        /** @brief System fault interrupt flag. */
extern const field32_t HRTIM_COMMON_ICR_BMPERC;        /** @brief Burst mode period flag clear. */
extern const field32_t HRTIM_COMMON_ICR_DLLRDYC;       /** @brief DLL ready interrupt flag clear. */
extern const field32_t HRTIM_COMMON_ICR_SYSFLTC;       /** @brief System fault interrupt flag clear. */
extern const field32_t HRTIM_COMMON_IER_BMPERIE;       /** @brief Burst mode period interrupt enable. */
extern const field32_t HRTIM_COMMON_IER_DLLRDYIE;      /** @brief DLL ready interrupt enable. */
extern const field32_t HRTIM_COMMON_IER_SYSFLTE;       /** @brief System fault interrupt enable. */
extern const field32_t HRTIM_COMMON_BMCR_BMSTAT;       /** @brief Burst mode status. */
extern const field32_t HRTIM_COMMON_BMCR_TEBM;         /** @brief Timer E burst mode. */
extern const field32_t HRTIM_COMMON_BMCR_TDBM;         /** @brief Timer D burst mode. */
extern const field32_t HRTIM_COMMON_BMCR_TCBM;         /** @brief Timer C burst mode. */
extern const field32_t HRTIM_COMMON_BMCR_TBBM;         /** @brief Timer B burst mode. */
extern const field32_t HRTIM_COMMON_BMCR_TABM;         /** @brief Timer A burst mode. */
extern const field32_t HRTIM_COMMON_BMCR_MTBM;         /** @brief Master timer burst mode. */
extern const field32_t HRTIM_COMMON_BMCR_BMPREN;       /** @brief Burst mode preload enable. */
extern const field32_t HRTIM_COMMON_BMCR_BMPRSC;       /** @brief Burst mode prescaler. */
extern const field32_t HRTIM_COMMON_BMCR_BMCLK;        /** @brief Burst mode clock source. */
extern const field32_t HRTIM_COMMON_BMCR_BMOM;         /** @brief Burst mode operating mode. */
extern const field32_t HRTIM_COMMON_BMCR_BME;          /** @brief Burst mode enable. */
extern const field32_t HRTIM_COMMON_BMTRG_OCHPEV;      /** @brief OCHPEV. */
extern const field32_t HRTIM_COMMON_BMTRG_TEREP;       /** @brief TEREP. */
extern const field32_t HRTIM_COMMON_BMTRG_TERST;       /** @brief TERST. */
extern const field32_t HRTIM_COMMON_BMTRG_TDREP;       /** @brief TDREP. */
extern const field32_t HRTIM_COMMON_BMTRG_TDRST;       /** @brief TDRST. */
extern const field32_t HRTIM_COMMON_BMTRG_TCREP;       /** @brief TCREP. */
extern const field32_t HRTIM_COMMON_BMTRG_TCRST;       /** @brief TCRST. */
extern const field32_t HRTIM_COMMON_BMTRG_TBREP;       /** @brief TBREP. */
extern const field32_t HRTIM_COMMON_BMTRG_TBRST;       /** @brief TBRST. */
extern const field32_t HRTIM_COMMON_BMTRG_TAREP;       /** @brief TAREP. */
extern const field32_t HRTIM_COMMON_BMTRG_TARST;       /** @brief TARST. */
extern const field32_t HRTIM_COMMON_BMTRG_MSTREP;      /** @brief MSTREP. */
extern const field32_t HRTIM_COMMON_BMTRG_MSTRST;      /** @brief MSTRST. */
extern const field32_t HRTIM_COMMON_BMTRG_SW;          /** @brief SW. */
extern const field32_t HRTIM_COMMON_BMCMPR6_BMCMP;     /** @brief BMCMP. */
extern const field32_t HRTIM_COMMON_BMPER_BMPER;       /** @brief Burst mode period. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TEPER;    /** @brief ADC trigger 1 on timer E period. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TDPER;    /** @brief ADC trigger 1 on timer D period. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TCPER;    /** @brief ADC trigger 1 on timer C period. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TBRST;    /** @brief ADC trigger 1 on timer B reset. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TBPER;    /** @brief ADC trigger 1 on timer B period. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TARST;    /** @brief ADC trigger 1 on timer A reset. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TAPER;    /** @brief ADC trigger 1 on timer A period. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1MPER;     /** @brief ADC trigger 1 on master period. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TERST;    /** @brief ADC trigger 2 on timer E reset. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TDRST;    /** @brief ADC trigger 2 on timer D reset. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TDPER;    /** @brief ADC trigger 2 on timer D period. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TCRST;    /** @brief ADC trigger 2 on timer C reset. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TCPER;    /** @brief ADC trigger 2 on timer C period. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TBPER;    /** @brief ADC trigger 2 on timer B period. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TAPER;    /** @brief ADC trigger 2 on timer A period. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2MPER;     /** @brief ADC trigger 2 on master period. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TEPER;    /** @brief AD1TEPER. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TDPER;    /** @brief AD1TDPER. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TCPER;    /** @brief AD1TCPER. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TBRST;    /** @brief AD1TBRST. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TBPER;    /** @brief AD1TBPER. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TARST;    /** @brief AD1TARST. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1TAPER;    /** @brief AD1TAPER. */
extern const field32_t HRTIM_COMMON_ADC3R_AD1MPER;     /** @brief AD1MPER. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TERST;    /** @brief AD2TERST. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TDRST;    /** @brief AD2TDRST. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TDPER;    /** @brief AD2TDPER. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TCRST;    /** @brief AD2TCRST. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TCPER;    /** @brief AD2TCPER. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TBPER;    /** @brief AD2TBPER. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2TAPER;    /** @brief AD2TAPER. */
extern const field32_t HRTIM_COMMON_ADC4R_AD2MPER;     /** @brief AD2MPER. */
extern const field32_t HRTIM_COMMON_DLLCR_CALRTE;      /** @brief DLL calibration rate. */
extern const field32_t HRTIM_COMMON_DLLCR_CALEN;       /** @brief DLL calibration enable. */
extern const field32_t HRTIM_COMMON_DLLCR_CAL;         /** @brief DLL calibration start. */
extern const field32_t HRTIM_COMMON_FLTINR2_FLTSD;     /** @brief FLTSD. */
extern const field32_t HRTIM_COMMON_FLTINR2_FLT5LCK;   /** @brief FLT5LCK. */
extern const field32_t HRTIM_COMMON_FLTINR2_FLT5F;     /** @brief FLT5F. */
extern const field32_t HRTIM_COMMON_FLTINR2_FLT5SRC;   /** @brief FLT5SRC. */
extern const field32_t HRTIM_COMMON_FLTINR2_FLT5P;     /** @brief FLT5P. */
extern const field32_t HRTIM_COMMON_FLTINR2_FLT5E;     /** @brief FLT5E. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MREP;      /** @brief MREP. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MPER;      /** @brief MPER. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MCNT;      /** @brief MCNT. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MDIER;     /** @brief MDIER. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MICR;      /** @brief MICR. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MCR;       /** @brief MCR. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXFLTR;  /** @brief HRTIM_FLTxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXOUTR;  /** @brief HRTIM_OUTxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXCHPR;  /** @brief HRTIM_CHPxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXRSTR;  /** @brief HRTIM_RSTxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMX_DTXR; /** @brief HRTIM_DTxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXREP;   /** @brief HRTIM_REPxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXPER;   /** @brief HRTIM_PERxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXCNT;   /** @brief HRTIM_CNTxR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXDIER;  /** @brief HRTIM_TIMxDIER register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXICR;   /** @brief HRTIM_TIMxICR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXCR;    /** @brief HRTIM_TIMxCR register update enable. */

/** @subsection Enumerated HRTIM_COMMON Register Field Definitions */

extern const field32_t HRTIM_COMMON_CR1_ADxUSRC[5];       /** @brief ADC trigger 1 update source. */
extern const field32_t HRTIM_COMMON_ISR_FLTx[6];          /** @brief Fault 1 interrupt flag. */
extern const field32_t HRTIM_COMMON_ICR_FLTxC[6];         /** @brief Fault 1 interrupt flag clear. */
extern const field32_t HRTIM_COMMON_IER_FLTxIE[6];        /** @brief Fault 1 interrupt enable. */
extern const field32_t HRTIM_COMMON_OENR_TExOEN[3];       /** @brief Timer E output 1 enable. */
extern const field32_t HRTIM_COMMON_OENR_TDxOEN[3];       /** @brief Timer D output 1 enable. */
extern const field32_t HRTIM_COMMON_OENR_TCxOEN[3];       /** @brief Timer C output 1 enable. */
extern const field32_t HRTIM_COMMON_OENR_TBxOEN[3];       /** @brief Timer B output 1 enable. */
extern const field32_t HRTIM_COMMON_OENR_TAxOEN[3];       /** @brief Timer A output 1 enable. */
extern const field32_t HRTIM_COMMON_DISR_TExODIS[3];      /** @brief TE1ODIS. */
extern const field32_t HRTIM_COMMON_DISR_TDxODIS[3];      /** @brief TD1ODIS. */
extern const field32_t HRTIM_COMMON_DISR_TCxODIS[3];      /** @brief TC1ODIS. */
extern const field32_t HRTIM_COMMON_DISR_TBxODIS[3];      /** @brief TB1ODIS. */
extern const field32_t HRTIM_COMMON_DISR_TAxODIS[3];      /** @brief TA1ODIS. */
extern const field32_t HRTIM_COMMON_ODSR_TExODS[3];       /** @brief Timer E output 1 disable status. */
extern const field32_t HRTIM_COMMON_ODSR_TDxODS[3];       /** @brief Timer D output 1 disable status. */
extern const field32_t HRTIM_COMMON_ODSR_TCxODS[3];       /** @brief Timer C output 1 disable status. */
extern const field32_t HRTIM_COMMON_ODSR_TBxODS[3];       /** @brief Timer B output 1 disable status. */
extern const field32_t HRTIM_COMMON_ODSR_TAxODS[3];       /** @brief Timer A output 1 disable status. */
extern const field32_t HRTIM_COMMON_BMTRG_TECMPx[3];      /** @brief TECMP1. */
extern const field32_t HRTIM_COMMON_BMTRG_TDCMPx[3];      /** @brief TDCMP1. */
extern const field32_t HRTIM_COMMON_BMTRG_TCCMPx[3];      /** @brief TCCMP1. */
extern const field32_t HRTIM_COMMON_BMTRG_TBCMPx[3];      /** @brief TBCMP1. */
extern const field32_t HRTIM_COMMON_BMTRG_TACMPx[3];      /** @brief TACMP1. */
extern const field32_t HRTIM_COMMON_BMTRG_MSTCMPx[5];     /** @brief MSTCMP1. */
extern const field32_t HRTIM_COMMON_EECR1_EExFAST[6];     /** @brief External event 1 fast mode. */
extern const field32_t HRTIM_COMMON_EECR1_EExSNS[6];      /** @brief External event 1 sensitivity. */
extern const field32_t HRTIM_COMMON_EECR1_EExPOL[6];      /** @brief External event 1 polarity. */
extern const field32_t HRTIM_COMMON_EECR1_EExSRC[6];      /** @brief External event 1 source. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TECx[5];     /** @brief ADC trigger 1 on timer E compare 2. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TDCx[5];     /** @brief ADC trigger 1 on timer D compare 2. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TCCx[5];     /** @brief ADC trigger 1 on timer C compare 2. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TBCx[5];     /** @brief ADC trigger 1 on timer B compare 2. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1TACx[5];     /** @brief ADC trigger 1 on timer A compare 2. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1EEVx[6];     /** @brief ADC trigger 1 on external event 1. */
extern const field32_t HRTIM_COMMON_ADC1R_AD1MCx[5];      /** @brief ADC trigger 1 on master compare 1. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TECx[5];     /** @brief ADC trigger 2 on timer E compare 2. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TDCx[5];     /** @brief ADC trigger 2 on timer D compare 2. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TCCx[5];     /** @brief ADC trigger 2 on timer C compare 2. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TBCx[5];     /** @brief ADC trigger 2 on timer B compare 2. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2TACx[5];     /** @brief ADC trigger 2 on timer A compare 2. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2EEVx[11];    /** @brief ADC trigger 2 on external event 6. */
extern const field32_t HRTIM_COMMON_ADC2R_AD2MCx[5];      /** @brief ADC trigger 2 on master compare 1. */
extern const field32_t HRTIM_COMMON_FLTINR1_FLTxLCK[5];   /** @brief FLT1LCK. */
extern const field32_t HRTIM_COMMON_FLTINR1_FLTxF[5];     /** @brief FLT1F. */
extern const field32_t HRTIM_COMMON_FLTINR1_FLTxSRC[5];   /** @brief FLT1SRC. */
extern const field32_t HRTIM_COMMON_FLTINR1_FLTxP[5];     /** @brief FLT1P. */
extern const field32_t HRTIM_COMMON_FLTINR1_FLTxE[5];     /** @brief FLT1E. */
extern const field32_t HRTIM_COMMON_BDMUPDR_MCMPx[5];     /** @brief MCMP1. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXEEFRx[3]; /** @brief HRTIM_EEFxR1 register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXRSTxR[3]; /** @brief HRTIM_RST1xR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXSETxR[3]; /** @brief HRTIM_SET1xR register update enable. */
extern const field32_t HRTIM_COMMON_BDTXUPR_TIMXCMPx[5];  /** @brief HRTIM_CMP1xR register update enable. */

/**************************************************************************************************
 * @section DFSDM Definitions
 **************************************************************************************************/

/** @subsection Enumerated DFSDM Register Definitions */

extern rw_reg32_t const DFSDM_DFSDM_CHCFGxR1[8];  /** @brief DFSDM channel configuration 0 register 1. */
extern rw_reg32_t const DFSDM_DFSDM_CHCFGxR2[8];  /** @brief DFSDM channel configuration 0 register 2. */
extern rw_reg32_t const DFSDM_DFSDM_AWSCDxR[8];   /** @brief DFSDM analog watchdog and short-circuit detector register. */
extern ro_reg32_t const DFSDM_DFSDM_CHWDATxR[8];  /** @brief DFSDM channel watchdog filter data register. */
extern rw_reg32_t const DFSDM_DFSDM_CHDATINxR[8]; /** @brief DFSDM channel data input register. */
extern rw_reg32_t const DFSDM_DFSDMx_CR1[4];      /** @brief DFSDM control register 1. */
extern rw_reg32_t const DFSDM_DFSDMx_CR2[4];      /** @brief DFSDM control register 2. */
extern ro_reg32_t const DFSDM_DFSDMx_ISR[4];      /** @brief DFSDM interrupt and status register. */
extern rw_reg32_t const DFSDM_DFSDMx_ICR[4];      /** @brief DFSDM interrupt flag clear register. */
extern rw_reg32_t const DFSDM_DFSDMx_JCHGR[4];    /** @brief DFSDM injected channel group selection register. */
extern rw_reg32_t const DFSDM_DFSDMx_FCR[4];      /** @brief DFSDM filter control register. */
extern ro_reg32_t const DFSDM_DFSDMx_JDATAR[4];   /** @brief DFSDM data register for injected group. */
extern ro_reg32_t const DFSDM_DFSDMx_RDATAR[4];   /** @brief DFSDM data register for the regular channel. */
extern rw_reg32_t const DFSDM_DFSDMx_AWHTR[4];    /** @brief DFSDM analog watchdog high threshold register. */
extern rw_reg32_t const DFSDM_DFSDMx_AWLTR[4];    /** @brief DFSDM analog watchdog low threshold register. */
extern ro_reg32_t const DFSDM_DFSDMx_AWSR[4];     /** @brief DFSDM analog watchdog status register. */
extern rw_reg32_t const DFSDM_DFSDMx_AWCFR[4];    /** @brief DFSDM analog watchdog clear flag register. */
extern ro_reg32_t const DFSDM_DFSDMx_EXMAX[4];    /** @brief DFSDM extremes detector maximum register. */
extern ro_reg32_t const DFSDM_DFSDMx_EXMIN[4];    /** @brief DFSDM extremes detector minimum register. */
extern ro_reg32_t const DFSDM_DFSDMx_CNVTIMR[4];  /** @brief DFSDM conversion timer register. */

/** @subsection DFSDM Register Field Definitions */

extern const field32_t DFSDM_DFSDM_CHCFGxR1_SITP;     /** @brief Serial interface type for channel 0. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_SPICKSEL; /** @brief SPI clock select for channel 0. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_SCDEN;    /** @brief Short-circuit detector enable on channel 0. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_CKABEN;   /** @brief Clock absence detector enable on channel 0. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_CHEN;     /** @brief Channel 0 enable. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_CHINSEL;  /** @brief Channel inputs selection. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_DATMPX;   /** @brief Input data multiplexer for channel 0. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_DATPACK;  /** @brief Data packing mode in dfsdm_chdatinyr register. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_CKOUTDIV; /** @brief Output serial clock divider. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_CKOUTSRC; /** @brief Output serial clock source selection. */
extern const field32_t DFSDM_DFSDM_CHCFGxR1_DFSDMEN;  /** @brief Global enable for DFSDM interface. */
extern const field32_t DFSDM_DFSDM_CHCFGxR2_DTRBS;    /** @brief Data right bit-shift for channel 0. */
extern const field32_t DFSDM_DFSDM_CHCFGxR2_OFFSET;   /** @brief 24-bit calibration offset for channel 0. */
extern const field32_t DFSDM_DFSDM_AWSCDxR_SCDT;      /** @brief Short-circuit detector threshold for channel 0. */
extern const field32_t DFSDM_DFSDM_AWSCDxR_BKSCD;     /** @brief Break signal assignment for short-circuit detector on channel 0. */
extern const field32_t DFSDM_DFSDM_AWSCDxR_AWFOSR;    /** @brief Analog watchdog filter oversampling ratio (decimation rate) on channel 0. */
extern const field32_t DFSDM_DFSDM_AWSCDxR_AWFORD;    /** @brief Analog watchdog sinc filter order on channel 0. */
extern const field32_t DFSDM_DFSDM_CHWDATxR_WDATA;    /** @brief Input channel y watchdog data. */
extern const field32_t DFSDM_DFSDMx_CR1_DFEN;         /** @brief DFSDM enable. */
extern const field32_t DFSDM_DFSDMx_CR1_JSWSTART;     /** @brief Start a conversion of the injected group of channels. */
extern const field32_t DFSDM_DFSDMx_CR1_JSYNC;        /** @brief Launch an injected conversion synchronously with the DFSDM0 JSWSTART trigger. */
extern const field32_t DFSDM_DFSDMx_CR1_JSCAN;        /** @brief Scanning conversion mode for injected conversions. */
extern const field32_t DFSDM_DFSDMx_CR1_JDMAEN;       /** @brief DMA channel enabled to read data for the injected channel group. */
extern const field32_t DFSDM_DFSDMx_CR1_JEXTSEL;      /** @brief Trigger signal selection for launching injected conversions. */
extern const field32_t DFSDM_DFSDMx_CR1_JEXTEN;       /** @brief Trigger enable and trigger edge selection for injected conversions. */
extern const field32_t DFSDM_DFSDMx_CR1_RSWSTART;     /** @brief Software start of a conversion on the regular channel. */
extern const field32_t DFSDM_DFSDMx_CR1_RCONT;        /** @brief Continuous mode selection for regular conversions. */
extern const field32_t DFSDM_DFSDMx_CR1_RSYNC;        /** @brief Launch regular conversion synchronously with DFSDM0. */
extern const field32_t DFSDM_DFSDMx_CR1_RDMAEN;       /** @brief DMA channel enabled to read data for the regular conversion. */
extern const field32_t DFSDM_DFSDMx_CR1_RCH;          /** @brief Regular channel selection. */
extern const field32_t DFSDM_DFSDMx_CR1_FAST;         /** @brief Fast conversion mode selection for regular conversions. */
extern const field32_t DFSDM_DFSDMx_CR1_AWFSEL;       /** @brief Analog watchdog fast mode select. */
extern const field32_t DFSDM_DFSDMx_CR2_JEOCIE;       /** @brief Injected end of conversion interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_REOCIE;       /** @brief Regular end of conversion interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_JOVRIE;       /** @brief Injected data overrun interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_ROVRIE;       /** @brief Regular data overrun interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_AWDIE;        /** @brief Analog watchdog interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_SCDIE;        /** @brief Short-circuit detector interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_CKABIE;       /** @brief Clock absence interrupt enable. */
extern const field32_t DFSDM_DFSDMx_CR2_EXCH;         /** @brief Extremes detector channel selection. */
extern const field32_t DFSDM_DFSDMx_CR2_AWDCH;        /** @brief Analog watchdog channel selection. */
extern const field32_t DFSDM_DFSDMx_ISR_JEOCF;        /** @brief End of injected conversion flag. */
extern const field32_t DFSDM_DFSDMx_ISR_REOCF;        /** @brief End of regular conversion flag. */
extern const field32_t DFSDM_DFSDMx_ISR_JOVRF;        /** @brief Injected conversion overrun flag. */
extern const field32_t DFSDM_DFSDMx_ISR_ROVRF;        /** @brief Regular conversion overrun flag. */
extern const field32_t DFSDM_DFSDMx_ISR_AWDF;         /** @brief Analog watchdog. */
extern const field32_t DFSDM_DFSDMx_ISR_JCIP;         /** @brief Injected conversion in progress status. */
extern const field32_t DFSDM_DFSDMx_ISR_RCIP;         /** @brief Regular conversion in progress status. */
extern const field32_t DFSDM_DFSDMx_ISR_CKABF;        /** @brief Clock absence flag. */
extern const field32_t DFSDM_DFSDMx_ISR_SCDF;         /** @brief Short-circuit detector flag. */
extern const field32_t DFSDM_DFSDMx_ICR_CLRJOVRF;     /** @brief Clear the injected conversion overrun flag. */
extern const field32_t DFSDM_DFSDMx_ICR_CLRROVRF;     /** @brief Clear the regular conversion overrun flag. */
extern const field32_t DFSDM_DFSDMx_ICR_CLRCKABF;     /** @brief Clear the clock absence flag. */
extern const field32_t DFSDM_DFSDMx_ICR_CLRSCDF;      /** @brief Clear the short-circuit detector flag. */
extern const field32_t DFSDM_DFSDMx_JCHGR_JCHG;       /** @brief Injected channel group selection. */
extern const field32_t DFSDM_DFSDMx_FCR_IOSR;         /** @brief Integrator oversampling ratio (averaging length). */
extern const field32_t DFSDM_DFSDMx_FCR_FOSR;         /** @brief Sinc filter oversampling ratio (decimation rate). */
extern const field32_t DFSDM_DFSDMx_FCR_FORD;         /** @brief Sinc filter order. */
extern const field32_t DFSDM_DFSDMx_JDATAR_JDATACH;   /** @brief Injected channel most recently converted. */
extern const field32_t DFSDM_DFSDMx_JDATAR_JDATA;     /** @brief Injected group conversion data. */
extern const field32_t DFSDM_DFSDMx_RDATAR_RDATACH;   /** @brief Regular channel most recently converted. */
extern const field32_t DFSDM_DFSDMx_RDATAR_RPEND;     /** @brief Regular channel pending data. */
extern const field32_t DFSDM_DFSDMx_RDATAR_RDATA;     /** @brief Regular channel conversion data. */
extern const field32_t DFSDM_DFSDMx_AWHTR_BKAWH;      /** @brief Break signal assignment to analog watchdog high threshold event. */
extern const field32_t DFSDM_DFSDMx_AWHTR_AWHT;       /** @brief Analog watchdog high threshold. */
extern const field32_t DFSDM_DFSDMx_AWLTR_BKAWL;      /** @brief Break signal assignment to analog watchdog low threshold event. */
extern const field32_t DFSDM_DFSDMx_AWLTR_AWLT;       /** @brief Analog watchdog low threshold. */
extern const field32_t DFSDM_DFSDMx_AWSR_AWLTF;       /** @brief Analog watchdog low threshold flag. */
extern const field32_t DFSDM_DFSDMx_AWSR_AWHTF;       /** @brief Analog watchdog high threshold flag. */
extern const field32_t DFSDM_DFSDMx_AWCFR_CLRAWLTF;   /** @brief Clear the analog watchdog low threshold flag. */
extern const field32_t DFSDM_DFSDMx_AWCFR_CLRAWHTF;   /** @brief Clear the analog watchdog high threshold flag. */
extern const field32_t DFSDM_DFSDMx_EXMAX_EXMAXCH;    /** @brief Extremes detector maximum data channel. */
extern const field32_t DFSDM_DFSDMx_EXMAX_EXMAX;      /** @brief Extremes detector maximum value. */
extern const field32_t DFSDM_DFSDMx_EXMIN_EXMINCH;    /** @brief Extremes detector minimum data channel. */
extern const field32_t DFSDM_DFSDMx_EXMIN_EXMIN;      /** @brief Extremes detector minimum value. */
extern const field32_t DFSDM_DFSDMx_CNVTIMR_CNVCNT;   /** @brief 28-bit timer counting conversion time. */

/** @subsection Enumerated DFSDM Register Field Definitions */

extern const field32_t DFSDM_DFSDM_CHDATINxR_INDATx[2]; /** @brief Input data for channel 0. */

/**************************************************************************************************
 * @section G2_TIM1x Definitions
 **************************************************************************************************/

/** @subsection Enumerated G2_TIM1x Register Definitions */

extern rw_reg32_t const G2_TIM1x_CR1[8];          /** @brief Control register 1. */
extern rw_reg32_t const G2_TIM1x_CR2[8];          /** @brief Control register 2. */
extern rw_reg32_t const G2_TIM1x_DIER[8];         /** @brief DMA/Interrupt enable register. */
extern rw_reg32_t const G2_TIM1x_SR[8];           /** @brief Status register. */
extern rw_reg32_t const G2_TIM1x_EGR[8];          /** @brief Event generation register. */
extern rw_reg32_t const G2_TIM1x_CCMR1_OUTPUT[8]; /** @brief Capture/compare mode register (output mode). */
extern rw_reg32_t const G2_TIM1x_CCMR1_INPUT[8];  /** @brief Capture/compare mode register (output mode). */
extern rw_reg32_t const G2_TIM1x_CCER[8];         /** @brief Capture/compare enable register. */
extern rw_reg32_t const G2_TIM1x_CNT[8];          /** @brief Counter. */
extern rw_reg32_t const G2_TIM1x_PSC[8];          /** @brief Prescaler. */
extern rw_reg32_t const G2_TIM1x_ARR[8];          /** @brief Auto-reload register. */
extern rw_reg32_t const G2_TIM1x_RCR[8];          /** @brief Repetition counter register. */
extern rw_reg32_t const G2_TIM1x_CCR1[8];         /** @brief Capture/compare register 1. */
extern rw_reg32_t const G2_TIM1x_BDTR[8];         /** @brief Break and dead-time register. */
extern rw_reg32_t const G2_TIM1x_DCR[8];          /** @brief DMA control register. */
extern rw_reg32_t const G2_TIM1x_DMAR[8];         /** @brief DMA address for full transfer. */
extern rw_reg32_t const G2_TIM1x_TIM16_AF1[8];    /** @brief TIM16 alternate function register 1. */
extern rw_reg32_t const G2_TIM1x_TIM16_TISEL[8];  /** @brief TIM16 input selection register. */

/** @subsection G2_TIM1x Register Field Definitions */

extern const field32_t G2_TIM1x_CR1_CEN;             /** @brief Counter enable. */
extern const field32_t G2_TIM1x_CR1_UDIS;            /** @brief Update disable. */
extern const field32_t G2_TIM1x_CR1_URS;             /** @brief Update request source. */
extern const field32_t G2_TIM1x_CR1_OPM;             /** @brief One-pulse mode. */
extern const field32_t G2_TIM1x_CR1_ARPE;            /** @brief Auto-reload preload enable. */
extern const field32_t G2_TIM1x_CR1_CKD;             /** @brief Clock division. */
extern const field32_t G2_TIM1x_CR1_UIFREMAP;        /** @brief UIF status bit remapping. */
extern const field32_t G2_TIM1x_CR2_OIS1N;           /** @brief Output idle state 1. */
extern const field32_t G2_TIM1x_CR2_OIS1;            /** @brief Output idle state 1. */
extern const field32_t G2_TIM1x_CR2_CCDS;            /** @brief Capture/compare DMA selection. */
extern const field32_t G2_TIM1x_CR2_CCUS;            /** @brief Capture/compare control update selection. */
extern const field32_t G2_TIM1x_CR2_CCPC;            /** @brief Capture/compare preloaded control. */
extern const field32_t G2_TIM1x_DIER_UIE;            /** @brief Update interrupt enable. */
extern const field32_t G2_TIM1x_DIER_CC1IE;          /** @brief Capture/Compare 1 interrupt enable. */
extern const field32_t G2_TIM1x_DIER_COMIE;          /** @brief COM interrupt enable. */
extern const field32_t G2_TIM1x_DIER_BIE;            /** @brief Break interrupt enable. */
extern const field32_t G2_TIM1x_DIER_UDE;            /** @brief Update DMA request enable. */
extern const field32_t G2_TIM1x_DIER_CC1DE;          /** @brief Capture/Compare 1 DMA request enable. */
extern const field32_t G2_TIM1x_DIER_COMDE;          /** @brief COM DMA request enable. */
extern const field32_t G2_TIM1x_SR_CC1OF;            /** @brief Capture/Compare 1 overcapture flag. */
extern const field32_t G2_TIM1x_SR_BIF;              /** @brief Break interrupt flag. */
extern const field32_t G2_TIM1x_SR_COMIF;            /** @brief COM interrupt flag. */
extern const field32_t G2_TIM1x_SR_CC1IF;            /** @brief Capture/compare 1 interrupt flag. */
extern const field32_t G2_TIM1x_SR_UIF;              /** @brief Update interrupt flag. */
extern const field32_t G2_TIM1x_EGR_BG;              /** @brief Break generation. */
extern const field32_t G2_TIM1x_EGR_COMG;            /** @brief Capture/Compare control update generation. */
extern const field32_t G2_TIM1x_EGR_CC1G;            /** @brief Capture/compare 1 generation. */
extern const field32_t G2_TIM1x_EGR_UG;              /** @brief Update generation. */
extern const field32_t G2_TIM1x_CCMR1_OUTPUT_CC1S;   /** @brief Capture/Compare 1 selection. */
extern const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1FE;  /** @brief Output compare 1 fast enable. */
extern const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1PE;  /** @brief Output compare 1 preload enable. */
extern const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1M;   /** @brief Output compare 1 mode. */
extern const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1M_3; /** @brief Output compare 1 mode. */
extern const field32_t G2_TIM1x_CCMR1_INPUT_IC1F;    /** @brief Input capture 1 filter. */
extern const field32_t G2_TIM1x_CCMR1_INPUT_IC1PSC;  /** @brief Input capture 1 prescaler. */
extern const field32_t G2_TIM1x_CCMR1_INPUT_CC1S;    /** @brief Capture/Compare 1 selection. */
extern const field32_t G2_TIM1x_CCER_CC1NP;          /** @brief Capture/Compare 1 output polarity. */
extern const field32_t G2_TIM1x_CCER_CC1NE;          /** @brief Capture/Compare 1 complementary output enable. */
extern const field32_t G2_TIM1x_CCER_CC1P;           /** @brief Capture/Compare 1 output polarity. */
extern const field32_t G2_TIM1x_CCER_CC1E;           /** @brief Capture/Compare 1 output enable. */
extern const field32_t G2_TIM1x_CNT_CNT;             /** @brief Counter value. */
extern const field32_t G2_TIM1x_CNT_UIFCPY;          /** @brief UIF copy. */
extern const field32_t G2_TIM1x_PSC_PSC;             /** @brief Prescaler value. */
extern const field32_t G2_TIM1x_ARR_ARR;             /** @brief Auto-reload value. */
extern const field32_t G2_TIM1x_RCR_REP;             /** @brief Repetition counter value. */
extern const field32_t G2_TIM1x_CCR1_CCR1;           /** @brief Capture/Compare 1 value. */
extern const field32_t G2_TIM1x_BDTR_DTG;            /** @brief Dead-time generator setup. */
extern const field32_t G2_TIM1x_BDTR_LOCK;           /** @brief Lock configuration. */
extern const field32_t G2_TIM1x_BDTR_OSSI;           /** @brief Off-state selection for idle mode. */
extern const field32_t G2_TIM1x_BDTR_OSSR;           /** @brief Off-state selection for run mode. */
extern const field32_t G2_TIM1x_BDTR_BKE;            /** @brief Break enable. */
extern const field32_t G2_TIM1x_BDTR_BKP;            /** @brief Break polarity. */
extern const field32_t G2_TIM1x_BDTR_AOE;            /** @brief Automatic output enable. */
extern const field32_t G2_TIM1x_BDTR_MOE;            /** @brief Main output enable. */
extern const field32_t G2_TIM1x_BDTR_BKF;            /** @brief Break filter. */
extern const field32_t G2_TIM1x_DCR_DBL;             /** @brief DMA burst length. */
extern const field32_t G2_TIM1x_DCR_DBA;             /** @brief DMA base address. */
extern const field32_t G2_TIM1x_DMAR_DMAB;           /** @brief DMA register for burst accesses. */
extern const field32_t G2_TIM1x_TIM16_AF1_BKINE;     /** @brief BRK BKIN input enable. */
extern const field32_t G2_TIM1x_TIM16_AF1_BKDFBK1E;  /** @brief BRK dfsdm1_break[1] enable. */
extern const field32_t G2_TIM1x_TIM16_AF1_BKINP;     /** @brief BRK BKIN input polarity. */
extern const field32_t G2_TIM1x_TIM16_TISEL_TI1SEL;  /** @brief Selects TI1[0] to TI1[15] input. */

/** @subsection Enumerated G2_TIM1x Register Field Definitions */

extern const field32_t G2_TIM1x_TIM16_AF1_BKCMPxE[3]; /** @brief BRK COMP1 enable. */
extern const field32_t G2_TIM1x_TIM16_AF1_BKCMPxP[3]; /** @brief BRK COMP1 input polarity. */

/**************************************************************************************************
 * @section TIM15 Definitions
 **************************************************************************************************/

/** @subsection TIM15 Register Definitions */

extern rw_reg32_t const TIM15_CR1;          /** @brief Control register 1. */
extern rw_reg32_t const TIM15_CR2;          /** @brief Control register 2. */
extern rw_reg32_t const TIM15_SMCR;         /** @brief Slave mode control register. */
extern rw_reg32_t const TIM15_DIER;         /** @brief DMA/Interrupt enable register. */
extern rw_reg32_t const TIM15_SR;           /** @brief Status register. */
extern rw_reg32_t const TIM15_EGR;          /** @brief Event generation register. */
extern rw_reg32_t const TIM15_CCMR1_OUTPUT; /** @brief Capture/compare mode register (output mode). */
extern rw_reg32_t const TIM15_CCMR1_INPUT;  /** @brief Capture/compare mode register 1 (input mode). */
extern rw_reg32_t const TIM15_CCER;         /** @brief Capture/compare enable register. */
extern rw_reg32_t const TIM15_CNT;          /** @brief Counter. */
extern rw_reg32_t const TIM15_PSC;          /** @brief Prescaler. */
extern rw_reg32_t const TIM15_ARR;          /** @brief Auto-reload register. */
extern rw_reg32_t const TIM15_RCR;          /** @brief Repetition counter register. */
extern rw_reg32_t const TIM15_CCR1;         /** @brief Capture/compare register 1. */
extern rw_reg32_t const TIM15_CCR2;         /** @brief Capture/compare register 2. */
extern rw_reg32_t const TIM15_BDTR;         /** @brief Break and dead-time register. */
extern rw_reg32_t const TIM15_DCR;          /** @brief DMA control register. */
extern rw_reg32_t const TIM15_DMAR;         /** @brief DMA address for full transfer. */
extern rw_reg32_t const TIM15_AF1;          /** @brief TIM15 alternate fdfsdm1_breakon register 1. */
extern rw_reg32_t const TIM15_TISEL;        /** @brief TIM15 input selection register. */

/** @subsection TIM15 Register Field Definitions */

extern const field32_t TIM15_CR1_CEN;       /** @brief Counter enable. */
extern const field32_t TIM15_CR1_UDIS;      /** @brief Update disable. */
extern const field32_t TIM15_CR1_URS;       /** @brief Update request source. */
extern const field32_t TIM15_CR1_OPM;       /** @brief One-pulse mode. */
extern const field32_t TIM15_CR1_ARPE;      /** @brief Auto-reload preload enable. */
extern const field32_t TIM15_CR1_CKD;       /** @brief Clock division. */
extern const field32_t TIM15_CR1_UIFREMAP;  /** @brief UIF status bit remapping. */
extern const field32_t TIM15_CR2_CCPC;      /** @brief Capture/compare preloaded control. */
extern const field32_t TIM15_CR2_CCUS;      /** @brief Capture/compare control update selection. */
extern const field32_t TIM15_CR2_CCDS;      /** @brief Capture/compare DMA selection. */
extern const field32_t TIM15_CR2_MMS;       /** @brief Master mode selection. */
extern const field32_t TIM15_CR2_TI1S;      /** @brief TI1 selection. */
extern const field32_t TIM15_CR2_OIS1N;     /** @brief Output idle state 1. */
extern const field32_t TIM15_SMCR_SMS;      /** @brief Slave mode selection. */
extern const field32_t TIM15_SMCR_TS_2_0;   /** @brief Trigger selection. */
extern const field32_t TIM15_SMCR_MSM;      /** @brief Master/Slave mode. */
extern const field32_t TIM15_SMCR_SMS_3;    /** @brief Slave mode selection bit 3. */
extern const field32_t TIM15_SMCR_TS_4_3;   /** @brief Trigger selection - bit 4:3. */
extern const field32_t TIM15_DIER_UIE;      /** @brief Update interrupt enable. */
extern const field32_t TIM15_DIER_COMIE;    /** @brief COM interrupt enable. */
extern const field32_t TIM15_DIER_TIE;      /** @brief Trigger interrupt enable. */
extern const field32_t TIM15_DIER_BIE;      /** @brief Break interrupt enable. */
extern const field32_t TIM15_DIER_UDE;      /** @brief Update DMA request enable. */
extern const field32_t TIM15_DIER_COMDE;    /** @brief COM DMA request enable. */
extern const field32_t TIM15_DIER_TDE;      /** @brief Trigger DMA request enable. */
extern const field32_t TIM15_SR_BIF;        /** @brief Break interrupt flag. */
extern const field32_t TIM15_SR_TIF;        /** @brief Trigger interrupt flag. */
extern const field32_t TIM15_SR_COMIF;      /** @brief COM interrupt flag. */
extern const field32_t TIM15_SR_UIF;        /** @brief Update interrupt flag. */
extern const field32_t TIM15_EGR_BG;        /** @brief Break generation. */
extern const field32_t TIM15_EGR_TG;        /** @brief Trigger generation. */
extern const field32_t TIM15_EGR_COMG;      /** @brief Capture/Compare control update generation. */
extern const field32_t TIM15_EGR_UG;        /** @brief Update generation. */
extern const field32_t TIM15_CCER_CC1NE;    /** @brief Capture/Compare 1 complementary output enable. */
extern const field32_t TIM15_CNT_CNT;       /** @brief Counter value. */
extern const field32_t TIM15_CNT_UIFCPY;    /** @brief UIF copy. */
extern const field32_t TIM15_PSC_PSC;       /** @brief Prescaler value. */
extern const field32_t TIM15_ARR_ARR;       /** @brief Auto-reload value. */
extern const field32_t TIM15_RCR_REP;       /** @brief Repetition counter value. */
extern const field32_t TIM15_CCR1_CCR1;     /** @brief Capture/Compare 1 value. */
extern const field32_t TIM15_CCR2_CCR2;     /** @brief Capture/Compare 2 value. */
extern const field32_t TIM15_BDTR_MOE;      /** @brief Main output enable. */
extern const field32_t TIM15_BDTR_AOE;      /** @brief Automatic output enable. */
extern const field32_t TIM15_BDTR_BKP;      /** @brief Break polarity. */
extern const field32_t TIM15_BDTR_BKE;      /** @brief Break enable. */
extern const field32_t TIM15_BDTR_OSSR;     /** @brief Off-state selection for run mode. */
extern const field32_t TIM15_BDTR_OSSI;     /** @brief Off-state selection for idle mode. */
extern const field32_t TIM15_BDTR_LOCK;     /** @brief Lock configuration. */
extern const field32_t TIM15_BDTR_DTG;      /** @brief Dead-time generator setup. */
extern const field32_t TIM15_BDTR_BKF;      /** @brief Break filter. */
extern const field32_t TIM15_DCR_DBL;       /** @brief DMA burst length. */
extern const field32_t TIM15_DCR_DBA;       /** @brief DMA base address. */
extern const field32_t TIM15_DMAR_DMAB;     /** @brief DMA register for burst accesses. */
extern const field32_t TIM15_AF1_BKINE;     /** @brief BRK BKIN input enable. */
extern const field32_t TIM15_AF1_BKDF1BK0E; /** @brief BRK dfsdm1_break[0] enable. */
extern const field32_t TIM15_AF1_BKINP;     /** @brief BRK BKIN input polarity. */

/** @subsection Enumerated TIM15 Register Field Definitions */

extern const field32_t TIM15_CR2_OISx[3];            /** @brief Output idle state 1. */
extern const field32_t TIM15_DIER_CCxIE[3];          /** @brief Capture/Compare 1 interrupt enable. */
extern const field32_t TIM15_DIER_CCxDE[3];          /** @brief Capture/Compare 1 DMA request enable. */
extern const field32_t TIM15_SR_CCxOF[3];            /** @brief Capture/Compare 1 overcapture flag. */
extern const field32_t TIM15_SR_CCxIF[3];            /** @brief Capture/compare 1 interrupt flag. */
extern const field32_t TIM15_EGR_CCxG[3];            /** @brief Capture/compare 1 generation. */
extern const field32_t TIM15_CCMR1_OUTPUT_CCxS[3];   /** @brief Capture/Compare 1 selection. */
extern const field32_t TIM15_CCMR1_OUTPUT_OCxFE[3];  /** @brief Output compare 1 fast enable. */
extern const field32_t TIM15_CCMR1_OUTPUT_OCxPE[3];  /** @brief Output compare 1 preload enable. */
extern const field32_t TIM15_CCMR1_OUTPUT_OCxM[3];   /** @brief Output compare 1 mode. */
extern const field32_t TIM15_CCMR1_OUTPUT_OCxM_3[3]; /** @brief Output compare 1 mode bit 3. */
extern const field32_t TIM15_CCMR1_INPUT_ICxF[3];    /** @brief Input capture 1 filter. */
extern const field32_t TIM15_CCMR1_INPUT_ICxPSC[3];  /** @brief Input capture 1 prescaler. */
extern const field32_t TIM15_CCER_CCxNP[3];          /** @brief Capture/Compare 1 output polarity. */
extern const field32_t TIM15_CCER_CCxP[3];           /** @brief Capture/Compare 1 output polarity. */
extern const field32_t TIM15_CCER_CCxE[3];           /** @brief Capture/Compare 1 output enable. */
extern const field32_t TIM15_AF1_BKCMPxE[3];         /** @brief BRK COMP1 enable. */
extern const field32_t TIM15_AF1_BKCMPxP[3];         /** @brief BRK COMP1 input polarity. */
extern const field32_t TIM15_TISEL_TIxSEL[3];        /** @brief Selects TI1[0] to TI1[15] input. */

/**************************************************************************************************
 * @section USARTx Definitions
 **************************************************************************************************/

/** @subsection Enumerated USARTx Register Definitions */

extern rw_reg32_t const USARTx_CR1[7];   /** @brief Control register 1. */
extern rw_reg32_t const USARTx_CR2[7];   /** @brief Control register 2. */
extern rw_reg32_t const USARTx_CR3[7];   /** @brief Control register 3. */
extern rw_reg32_t const USARTx_BRR[7];   /** @brief Baud rate register. */
extern rw_reg32_t const USARTx_GTPR[7];  /** @brief Guard time and prescaler register. */
extern rw_reg32_t const USARTx_RTOR[7];  /** @brief Receiver timeout register. */
extern rw_reg32_t const USARTx_RQR[7];   /** @brief Request register. */
extern ro_reg32_t const USARTx_ISR[7];   /** @brief Interrupt & status register. */
extern rw_reg32_t const USARTx_ICR[7];   /** @brief Interrupt flag clear register. */
extern ro_reg32_t const USARTx_RDR[7];   /** @brief Receive data register. */
extern rw_reg32_t const USARTx_TDR[7];   /** @brief Transmit data register. */
extern rw_reg32_t const USARTx_PRESC[7]; /** @brief USART prescaler register. */

/** @subsection USARTx Register Field Definitions */

extern const field32_t USARTx_CR1_RXFFIE;      /** @brief RXFIFO full interrupt enable. */
extern const field32_t USARTx_CR1_TXFEIE;      /** @brief TXFIFO empty interrupt enable. */
extern const field32_t USARTx_CR1_FIFOEN;      /** @brief FIFO mode enable. */
extern const field32_t USARTx_CR1_EOBIE;       /** @brief End of block interrupt enable. */
extern const field32_t USARTx_CR1_RTOIE;       /** @brief Receiver timeout interrupt enable. */
extern const field32_t USARTx_CR1_OVER8;       /** @brief Oversampling mode. */
extern const field32_t USARTx_CR1_CMIE;        /** @brief Character match interrupt enable. */
extern const field32_t USARTx_CR1_MME;         /** @brief Mute mode enable. */
extern const field32_t USARTx_CR1_WAKE;        /** @brief Receiver wakeup method. */
extern const field32_t USARTx_CR1_PCE;         /** @brief Parity control enable. */
extern const field32_t USARTx_CR1_PS;          /** @brief Parity selection. */
extern const field32_t USARTx_CR1_PEIE;        /** @brief PE interrupt enable. */
extern const field32_t USARTx_CR1_TXEIE;       /** @brief Interrupt enable. */
extern const field32_t USARTx_CR1_TCIE;        /** @brief Transmission complete interrupt enable. */
extern const field32_t USARTx_CR1_RXNEIE;      /** @brief RXNE interrupt enable. */
extern const field32_t USARTx_CR1_IDLEIE;      /** @brief IDLE interrupt enable. */
extern const field32_t USARTx_CR1_TE;          /** @brief Transmitter enable. */
extern const field32_t USARTx_CR1_RE;          /** @brief Receiver enable. */
extern const field32_t USARTx_CR1_UESM;        /** @brief USART enable in stop mode. */
extern const field32_t USARTx_CR1_UE;          /** @brief USART enable. */
extern const field32_t USARTx_CR2_ADD4_7;      /** @brief Address of the USART node. */
extern const field32_t USARTx_CR2_ADD0_3;      /** @brief Address of the USART node. */
extern const field32_t USARTx_CR2_RTOEN;       /** @brief Receiver timeout enable. */
extern const field32_t USARTx_CR2_ABREN;       /** @brief Auto baud rate enable. */
extern const field32_t USARTx_CR2_MSBFIRST;    /** @brief Most significant bit first. */
extern const field32_t USARTx_CR2_TAINV;       /** @brief Binary data inversion. */
extern const field32_t USARTx_CR2_TXINV;       /** @brief TX pin active level inversion. */
extern const field32_t USARTx_CR2_RXINV;       /** @brief RX pin active level inversion. */
extern const field32_t USARTx_CR2_SWAP;        /** @brief Swap TX/RX pins. */
extern const field32_t USARTx_CR2_LINEN;       /** @brief LIN mode enable. */
extern const field32_t USARTx_CR2_STOP;        /** @brief STOP bits. */
extern const field32_t USARTx_CR2_CLKEN;       /** @brief Clock enable. */
extern const field32_t USARTx_CR2_CPOL;        /** @brief Clock polarity. */
extern const field32_t USARTx_CR2_CPHA;        /** @brief Clock phase. */
extern const field32_t USARTx_CR2_LBCL;        /** @brief Last bit clock pulse. */
extern const field32_t USARTx_CR2_LBDIE;       /** @brief LIN break detection interrupt enable. */
extern const field32_t USARTx_CR2_LBDL;        /** @brief LIN break detection length. */
extern const field32_t USARTx_CR2_ADDM7;       /** @brief 7-bit address detection/4-bit address detection. */
extern const field32_t USARTx_CR2_DIS_NSS;     /** @brief When the DSI_NSS bit is set, the NSS pin input is ignored. */
extern const field32_t USARTx_CR2_SLVEN;       /** @brief Synchronous slave mode enable. */
extern const field32_t USARTx_CR3_TXFTCFG;     /** @brief TXFIFO threshold configuration. */
extern const field32_t USARTx_CR3_RXFTIE;      /** @brief RXFIFO threshold interrupt enable. */
extern const field32_t USARTx_CR3_RXFTCFG;     /** @brief Receive FIFO threshold configuration. */
extern const field32_t USARTx_CR3_TCBGTIE;     /** @brief Transmission complete before guard time, interrupt enable. */
extern const field32_t USARTx_CR3_TXFTIE;      /** @brief TXFIFO threshold interrupt enable. */
extern const field32_t USARTx_CR3_WUFIE;       /** @brief Wakeup from stop mode interrupt enable. */
extern const field32_t USARTx_CR3_WUS;         /** @brief Wakeup from stop mode interrupt flag selection. */
extern const field32_t USARTx_CR3_SCARCNT;     /** @brief Smartcard auto-retry count. */
extern const field32_t USARTx_CR3_DEP;         /** @brief Driver enable polarity selection. */
extern const field32_t USARTx_CR3_DEM;         /** @brief Driver enable mode. */
extern const field32_t USARTx_CR3_DDRE;        /** @brief DMA disable on reception error. */
extern const field32_t USARTx_CR3_OVRDIS;      /** @brief Overrun disable. */
extern const field32_t USARTx_CR3_ONEBIT;      /** @brief One sample bit method enable. */
extern const field32_t USARTx_CR3_CTSIE;       /** @brief CTS interrupt enable. */
extern const field32_t USARTx_CR3_CTSE;        /** @brief CTS enable. */
extern const field32_t USARTx_CR3_RTSE;        /** @brief RTS enable. */
extern const field32_t USARTx_CR3_DMAT;        /** @brief DMA enable transmitter. */
extern const field32_t USARTx_CR3_DMAR;        /** @brief DMA enable receiver. */
extern const field32_t USARTx_CR3_SCEN;        /** @brief Smartcard mode enable. */
extern const field32_t USARTx_CR3_NACK;        /** @brief Smartcard NACK enable. */
extern const field32_t USARTx_CR3_HDSEL;       /** @brief Half-duplex selection. */
extern const field32_t USARTx_CR3_IRLP;        /** @brief Ir low-power. */
extern const field32_t USARTx_CR3_IREN;        /** @brief Ir mode enable. */
extern const field32_t USARTx_CR3_EIE;         /** @brief Error interrupt enable. */
extern const field32_t USARTx_BRR_BRR_4_15;    /** @brief DIV_Mantissa. */
extern const field32_t USARTx_BRR_BRR_0_3;     /** @brief DIV_Fraction. */
extern const field32_t USARTx_GTPR_GT;         /** @brief Guard time value. */
extern const field32_t USARTx_GTPR_PSC;        /** @brief Prescaler value. */
extern const field32_t USARTx_RTOR_BLEN;       /** @brief Block length. */
extern const field32_t USARTx_RTOR_RTO;        /** @brief Receiver timeout value. */
extern const field32_t USARTx_RQR_TXFRQ;       /** @brief Transmit data flush request. */
extern const field32_t USARTx_RQR_RXFRQ;       /** @brief Receive data flush request. */
extern const field32_t USARTx_RQR_MMRQ;        /** @brief Mute mode request. */
extern const field32_t USARTx_RQR_SBKRQ;       /** @brief Send break request. */
extern const field32_t USARTx_RQR_ABRRQ;       /** @brief Auto baud rate request. */
extern const field32_t USARTx_ISR_TXFT;        /** @brief TXFIFO threshold flag. */
extern const field32_t USARTx_ISR_RXFT;        /** @brief RXFIFO threshold flag. */
extern const field32_t USARTx_ISR_TCBGT;       /** @brief Transmission complete before guard time flag. */
extern const field32_t USARTx_ISR_RXFF;        /** @brief RXFIFO full. */
extern const field32_t USARTx_ISR_TXFE;        /** @brief TXFIFO empty. */
extern const field32_t USARTx_ISR_REACK;       /** @brief REACK. */
extern const field32_t USARTx_ISR_TEACK;       /** @brief TEACK. */
extern const field32_t USARTx_ISR_WUF;         /** @brief WUF. */
extern const field32_t USARTx_ISR_RWU;         /** @brief RWU. */
extern const field32_t USARTx_ISR_SBKF;        /** @brief SBKF. */
extern const field32_t USARTx_ISR_CMF;         /** @brief CMF. */
extern const field32_t USARTx_ISR_BUSY;        /** @brief BUSY. */
extern const field32_t USARTx_ISR_ABRF;        /** @brief ABRF. */
extern const field32_t USARTx_ISR_ABRE;        /** @brief ABRE. */
extern const field32_t USARTx_ISR_UDR;         /** @brief SPI slave underrun error flag. */
extern const field32_t USARTx_ISR_EOBF;        /** @brief EOBF. */
extern const field32_t USARTx_ISR_RTOF;        /** @brief RTOF. */
extern const field32_t USARTx_ISR_CTS;         /** @brief CTS. */
extern const field32_t USARTx_ISR_CTSIF;       /** @brief CTSIF. */
extern const field32_t USARTx_ISR_LBDF;        /** @brief LBDF. */
extern const field32_t USARTx_ISR_TXE;         /** @brief TXE. */
extern const field32_t USARTx_ISR_TC;          /** @brief TC. */
extern const field32_t USARTx_ISR_RXNE;        /** @brief RXNE. */
extern const field32_t USARTx_ISR_IDLE;        /** @brief IDLE. */
extern const field32_t USARTx_ISR_ORE;         /** @brief ORE. */
extern const field32_t USARTx_ISR_NF;          /** @brief NF. */
extern const field32_t USARTx_ISR_FE;          /** @brief FE. */
extern const field32_t USARTx_ISR_PE;          /** @brief PE. */
extern const field32_t USARTx_ICR_WUCF;        /** @brief Wakeup from stop mode clear flag. */
extern const field32_t USARTx_ICR_CMCF;        /** @brief Character match clear flag. */
extern const field32_t USARTx_ICR_UDRCF;       /** @brief SPI slave underrun clear flag. */
extern const field32_t USARTx_ICR_EOBCF;       /** @brief End of block clear flag. */
extern const field32_t USARTx_ICR_RTOCF;       /** @brief Receiver timeout clear flag. */
extern const field32_t USARTx_ICR_CTSCF;       /** @brief CTS clear flag. */
extern const field32_t USARTx_ICR_LBDCF;       /** @brief LIN break detection clear flag. */
extern const field32_t USARTx_ICR_TCBGTC;      /** @brief Transmission complete before guard time clear flag. */
extern const field32_t USARTx_ICR_TCCF;        /** @brief Transmission complete clear flag. */
extern const field32_t USARTx_ICR_TXFECF;      /** @brief TXFIFO empty clear flag. */
extern const field32_t USARTx_ICR_IDLECF;      /** @brief Idle line detected clear flag. */
extern const field32_t USARTx_ICR_ORECF;       /** @brief Overrun error clear flag. */
extern const field32_t USARTx_ICR_NCF;         /** @brief Noise detected clear flag. */
extern const field32_t USARTx_ICR_FECF;        /** @brief Framing error clear flag. */
extern const field32_t USARTx_ICR_PECF;        /** @brief Parity error clear flag. */
extern const field32_t USARTx_RDR_RDR;         /** @brief Receive data value. */
extern const field32_t USARTx_TDR_TDR;         /** @brief Transmit data value. */
extern const field32_t USARTx_PRESC_PRESCALER; /** @brief Clock prescaler. */

/** @subsection Enumerated USARTx Register Field Definitions */

extern const field32_t USARTx_CR1_Mx[2];      /** @brief Word length. */
extern const field32_t USARTx_CR1_DEATx[5];   /** @brief DEAT0. */
extern const field32_t USARTx_CR1_DEDTx[5];   /** @brief DEDT0. */
extern const field32_t USARTx_CR2_ABRMODx[2]; /** @brief ABRMOD0. */

/**************************************************************************************************
 * @section UARTx Definitions
 **************************************************************************************************/

/** @subsection Enumerated UARTx Register Definitions */

extern rw_reg32_t const UARTx_CR1[9];   /** @brief Control register 1. */
extern rw_reg32_t const UARTx_CR2[9];   /** @brief Control register 2. */
extern rw_reg32_t const UARTx_CR3[9];   /** @brief Control register 3. */
extern rw_reg32_t const UARTx_BRR[9];   /** @brief Baud rate register. */
extern rw_reg32_t const UARTx_GTPR[9];  /** @brief Guard time and prescaler register. */
extern rw_reg32_t const UARTx_RTOR[9];  /** @brief Receiver timeout register. */
extern rw_reg32_t const UARTx_RQR[9];   /** @brief Request register. */
extern ro_reg32_t const UARTx_ISR[9];   /** @brief Interrupt & status register. */
extern rw_reg32_t const UARTx_ICR[9];   /** @brief Interrupt flag clear register. */
extern ro_reg32_t const UARTx_RDR[9];   /** @brief Receive data register. */
extern rw_reg32_t const UARTx_TDR[9];   /** @brief Transmit data register. */
extern rw_reg32_t const UARTx_PRESC[9]; /** @brief USART prescaler register. */

/** @subsection UARTx Register Field Definitions */

extern const field32_t UARTx_CR1_RXFFIE;      /** @brief RXFIFO full interrupt enable. */
extern const field32_t UARTx_CR1_TXFEIE;      /** @brief TXFIFO empty interrupt enable. */
extern const field32_t UARTx_CR1_FIFOEN;      /** @brief FIFO mode enable. */
extern const field32_t UARTx_CR1_EOBIE;       /** @brief End of block interrupt enable. */
extern const field32_t UARTx_CR1_RTOIE;       /** @brief Receiver timeout interrupt enable. */
extern const field32_t UARTx_CR1_OVER8;       /** @brief Oversampling mode. */
extern const field32_t UARTx_CR1_CMIE;        /** @brief Character match interrupt enable. */
extern const field32_t UARTx_CR1_MME;         /** @brief Mute mode enable. */
extern const field32_t UARTx_CR1_WAKE;        /** @brief Receiver wakeup method. */
extern const field32_t UARTx_CR1_PCE;         /** @brief Parity control enable. */
extern const field32_t UARTx_CR1_PS;          /** @brief Parity selection. */
extern const field32_t UARTx_CR1_PEIE;        /** @brief PE interrupt enable. */
extern const field32_t UARTx_CR1_TXEIE;       /** @brief Interrupt enable. */
extern const field32_t UARTx_CR1_TCIE;        /** @brief Transmission complete interrupt enable. */
extern const field32_t UARTx_CR1_RXNEIE;      /** @brief RXNE interrupt enable. */
extern const field32_t UARTx_CR1_IDLEIE;      /** @brief IDLE interrupt enable. */
extern const field32_t UARTx_CR1_TE;          /** @brief Transmitter enable. */
extern const field32_t UARTx_CR1_RE;          /** @brief Receiver enable. */
extern const field32_t UARTx_CR1_UESM;        /** @brief USART enable in stop mode. */
extern const field32_t UARTx_CR1_UE;          /** @brief USART enable. */
extern const field32_t UARTx_CR2_ADD4_7;      /** @brief Address of the USART node. */
extern const field32_t UARTx_CR2_ADD0_3;      /** @brief Address of the USART node. */
extern const field32_t UARTx_CR2_RTOEN;       /** @brief Receiver timeout enable. */
extern const field32_t UARTx_CR2_ABREN;       /** @brief Auto baud rate enable. */
extern const field32_t UARTx_CR2_MSBFIRST;    /** @brief Most significant bit first. */
extern const field32_t UARTx_CR2_TAINV;       /** @brief Binary data inversion. */
extern const field32_t UARTx_CR2_TXINV;       /** @brief TX pin active level inversion. */
extern const field32_t UARTx_CR2_RXINV;       /** @brief RX pin active level inversion. */
extern const field32_t UARTx_CR2_SWAP;        /** @brief Swap TX/RX pins. */
extern const field32_t UARTx_CR2_LINEN;       /** @brief LIN mode enable. */
extern const field32_t UARTx_CR2_STOP;        /** @brief STOP bits. */
extern const field32_t UARTx_CR2_CLKEN;       /** @brief Clock enable. */
extern const field32_t UARTx_CR2_CPOL;        /** @brief Clock polarity. */
extern const field32_t UARTx_CR2_CPHA;        /** @brief Clock phase. */
extern const field32_t UARTx_CR2_LBCL;        /** @brief Last bit clock pulse. */
extern const field32_t UARTx_CR2_LBDIE;       /** @brief LIN break detection interrupt enable. */
extern const field32_t UARTx_CR2_LBDL;        /** @brief LIN break detection length. */
extern const field32_t UARTx_CR2_ADDM7;       /** @brief 7-bit address detection/4-bit address detection. */
extern const field32_t UARTx_CR2_DIS_NSS;     /** @brief When the DSI_NSS bit is set, the NSS pin input is ignored. */
extern const field32_t UARTx_CR2_SLVEN;       /** @brief Synchronous slave mode enable. */
extern const field32_t UARTx_CR3_TXFTCFG;     /** @brief TXFIFO threshold configuration. */
extern const field32_t UARTx_CR3_RXFTIE;      /** @brief RXFIFO threshold interrupt enable. */
extern const field32_t UARTx_CR3_RXFTCFG;     /** @brief Receive FIFO threshold configuration. */
extern const field32_t UARTx_CR3_TCBGTIE;     /** @brief Transmission complete before guard time, interrupt enable. */
extern const field32_t UARTx_CR3_TXFTIE;      /** @brief TXFIFO threshold interrupt enable. */
extern const field32_t UARTx_CR3_WUFIE;       /** @brief Wakeup from stop mode interrupt enable. */
extern const field32_t UARTx_CR3_WUS;         /** @brief Wakeup from stop mode interrupt flag selection. */
extern const field32_t UARTx_CR3_SCARCNT;     /** @brief Smartcard auto-retry count. */
extern const field32_t UARTx_CR3_DEP;         /** @brief Driver enable polarity selection. */
extern const field32_t UARTx_CR3_DEM;         /** @brief Driver enable mode. */
extern const field32_t UARTx_CR3_DDRE;        /** @brief DMA disable on reception error. */
extern const field32_t UARTx_CR3_OVRDIS;      /** @brief Overrun disable. */
extern const field32_t UARTx_CR3_ONEBIT;      /** @brief One sample bit method enable. */
extern const field32_t UARTx_CR3_CTSIE;       /** @brief CTS interrupt enable. */
extern const field32_t UARTx_CR3_CTSE;        /** @brief CTS enable. */
extern const field32_t UARTx_CR3_RTSE;        /** @brief RTS enable. */
extern const field32_t UARTx_CR3_DMAT;        /** @brief DMA enable transmitter. */
extern const field32_t UARTx_CR3_DMAR;        /** @brief DMA enable receiver. */
extern const field32_t UARTx_CR3_SCEN;        /** @brief Smartcard mode enable. */
extern const field32_t UARTx_CR3_NACK;        /** @brief Smartcard NACK enable. */
extern const field32_t UARTx_CR3_HDSEL;       /** @brief Half-duplex selection. */
extern const field32_t UARTx_CR3_IRLP;        /** @brief Ir low-power. */
extern const field32_t UARTx_CR3_IREN;        /** @brief Ir mode enable. */
extern const field32_t UARTx_CR3_EIE;         /** @brief Error interrupt enable. */
extern const field32_t UARTx_BRR_BRR_4_15;    /** @brief DIV_Mantissa. */
extern const field32_t UARTx_BRR_BRR_0_3;     /** @brief DIV_Fraction. */
extern const field32_t UARTx_GTPR_GT;         /** @brief Guard time value. */
extern const field32_t UARTx_GTPR_PSC;        /** @brief Prescaler value. */
extern const field32_t UARTx_RTOR_BLEN;       /** @brief Block length. */
extern const field32_t UARTx_RTOR_RTO;        /** @brief Receiver timeout value. */
extern const field32_t UARTx_RQR_TXFRQ;       /** @brief Transmit data flush request. */
extern const field32_t UARTx_RQR_RXFRQ;       /** @brief Receive data flush request. */
extern const field32_t UARTx_RQR_MMRQ;        /** @brief Mute mode request. */
extern const field32_t UARTx_RQR_SBKRQ;       /** @brief Send break request. */
extern const field32_t UARTx_RQR_ABRRQ;       /** @brief Auto baud rate request. */
extern const field32_t UARTx_ISR_TXFT;        /** @brief TXFIFO threshold flag. */
extern const field32_t UARTx_ISR_RXFT;        /** @brief RXFIFO threshold flag. */
extern const field32_t UARTx_ISR_TCBGT;       /** @brief Transmission complete before guard time flag. */
extern const field32_t UARTx_ISR_RXFF;        /** @brief RXFIFO full. */
extern const field32_t UARTx_ISR_TXFE;        /** @brief TXFIFO empty. */
extern const field32_t UARTx_ISR_REACK;       /** @brief REACK. */
extern const field32_t UARTx_ISR_TEACK;       /** @brief TEACK. */
extern const field32_t UARTx_ISR_WUF;         /** @brief WUF. */
extern const field32_t UARTx_ISR_RWU;         /** @brief RWU. */
extern const field32_t UARTx_ISR_SBKF;        /** @brief SBKF. */
extern const field32_t UARTx_ISR_CMF;         /** @brief CMF. */
extern const field32_t UARTx_ISR_BUSY;        /** @brief BUSY. */
extern const field32_t UARTx_ISR_ABRF;        /** @brief ABRF. */
extern const field32_t UARTx_ISR_ABRE;        /** @brief ABRE. */
extern const field32_t UARTx_ISR_UDR;         /** @brief SPI slave underrun error flag. */
extern const field32_t UARTx_ISR_EOBF;        /** @brief EOBF. */
extern const field32_t UARTx_ISR_RTOF;        /** @brief RTOF. */
extern const field32_t UARTx_ISR_CTS;         /** @brief CTS. */
extern const field32_t UARTx_ISR_CTSIF;       /** @brief CTSIF. */
extern const field32_t UARTx_ISR_LBDF;        /** @brief LBDF. */
extern const field32_t UARTx_ISR_TXE;         /** @brief TXE. */
extern const field32_t UARTx_ISR_TC;          /** @brief TC. */
extern const field32_t UARTx_ISR_RXNE;        /** @brief RXNE. */
extern const field32_t UARTx_ISR_IDLE;        /** @brief IDLE. */
extern const field32_t UARTx_ISR_ORE;         /** @brief ORE. */
extern const field32_t UARTx_ISR_NF;          /** @brief NF. */
extern const field32_t UARTx_ISR_FE;          /** @brief FE. */
extern const field32_t UARTx_ISR_PE;          /** @brief PE. */
extern const field32_t UARTx_ICR_WUCF;        /** @brief Wakeup from stop mode clear flag. */
extern const field32_t UARTx_ICR_CMCF;        /** @brief Character match clear flag. */
extern const field32_t UARTx_ICR_UDRCF;       /** @brief SPI slave underrun clear flag. */
extern const field32_t UARTx_ICR_EOBCF;       /** @brief End of block clear flag. */
extern const field32_t UARTx_ICR_RTOCF;       /** @brief Receiver timeout clear flag. */
extern const field32_t UARTx_ICR_CTSCF;       /** @brief CTS clear flag. */
extern const field32_t UARTx_ICR_LBDCF;       /** @brief LIN break detection clear flag. */
extern const field32_t UARTx_ICR_TCBGTC;      /** @brief Transmission complete before guard time clear flag. */
extern const field32_t UARTx_ICR_TCCF;        /** @brief Transmission complete clear flag. */
extern const field32_t UARTx_ICR_TXFECF;      /** @brief TXFIFO empty clear flag. */
extern const field32_t UARTx_ICR_IDLECF;      /** @brief Idle line detected clear flag. */
extern const field32_t UARTx_ICR_ORECF;       /** @brief Overrun error clear flag. */
extern const field32_t UARTx_ICR_NCF;         /** @brief Noise detected clear flag. */
extern const field32_t UARTx_ICR_FECF;        /** @brief Framing error clear flag. */
extern const field32_t UARTx_ICR_PECF;        /** @brief Parity error clear flag. */
extern const field32_t UARTx_RDR_RDR;         /** @brief Receive data value. */
extern const field32_t UARTx_TDR_TDR;         /** @brief Transmit data value. */
extern const field32_t UARTx_PRESC_PRESCALER; /** @brief Clock prescaler. */

/** @subsection Enumerated UARTx Register Field Definitions */

extern const field32_t UARTx_CR1_Mx[2];      /** @brief Word length. */
extern const field32_t UARTx_CR1_DEATx[5];   /** @brief DEAT0. */
extern const field32_t UARTx_CR1_DEDTx[5];   /** @brief DEDT0. */
extern const field32_t UARTx_CR2_ABRMODx[2]; /** @brief ABRMOD0. */

/**************************************************************************************************
 * @section A_TIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated A_TIMx Register Definitions */

extern rw_reg32_t const A_TIMx_CR1[9];          /** @brief Control register 1. */
extern rw_reg32_t const A_TIMx_CR2[9];          /** @brief Control register 2. */
extern rw_reg32_t const A_TIMx_SMCR[9];         /** @brief Slave mode control register. */
extern rw_reg32_t const A_TIMx_DIER[9];         /** @brief DMA/Interrupt enable register. */
extern rw_reg32_t const A_TIMx_SR[9];           /** @brief Status register. */
extern rw_reg32_t const A_TIMx_EGR[9];          /** @brief Event generation register. */
extern rw_reg32_t const A_TIMx_CCMR1_OUTPUT[9]; /** @brief Capture/compare mode register 1 (output mode). */
extern rw_reg32_t const A_TIMx_CCMR1_INPUT[9];  /** @brief Capture/compare mode register 1 (output mode). */
extern rw_reg32_t const A_TIMx_CCMR2_OUTPUT[9]; /** @brief Capture/compare mode register 2 (output mode). */
extern rw_reg32_t const A_TIMx_CCMR2_INPUT[9];  /** @brief Capture/compare mode register 2 (output mode). */
extern rw_reg32_t const A_TIMx_CCER[9];         /** @brief Capture/compare enable register. */
extern rw_reg32_t const A_TIMx_CNT[9];          /** @brief Counter. */
extern rw_reg32_t const A_TIMx_PSC[9];          /** @brief Prescaler. */
extern rw_reg32_t const A_TIMx_ARR[9];          /** @brief Auto-reload register. */
extern rw_reg32_t const A_TIMx_CCR1[9];         /** @brief Capture/compare register 1. */
extern rw_reg32_t const A_TIMx_CCR2[9];         /** @brief Capture/compare register 2. */
extern rw_reg32_t const A_TIMx_CCR3[9];         /** @brief Capture/compare register 3. */
extern rw_reg32_t const A_TIMx_CCR4[9];         /** @brief Capture/compare register 4. */
extern rw_reg32_t const A_TIMx_DCR[9];          /** @brief DMA control register. */
extern rw_reg32_t const A_TIMx_DMAR[9];         /** @brief DMA address for full transfer. */
extern rw_reg32_t const A_TIMx_RCR[9];          /** @brief Repetition counter register. */
extern rw_reg32_t const A_TIMx_BDTR[9];         /** @brief Break and dead-time register. */
extern rw_reg32_t const A_TIMx_CCMR3_OUTPUT[9]; /** @brief Capture/compare mode register 3 (output mode). */
extern rw_reg32_t const A_TIMx_CCR5[9];         /** @brief Capture/compare register 5. */
extern rw_reg32_t const A_TIMx_CRR6[9];         /** @brief Capture/compare register 6. */
extern rw_reg32_t const A_TIMx_AF1[9];          /** @brief TIM1 alternate function option register 1. */
extern rw_reg32_t const A_TIMx_AF2[9];          /** @brief TIM1 alternate function odfsdm1_breakster 2. */
extern rw_reg32_t const A_TIMx_TISEL[9];        /** @brief TIM1 timer input selection register. */

/** @subsection A_TIMx Register Field Definitions */

extern const field32_t A_TIMx_CR1_CEN;             /** @brief Counter enable. */
extern const field32_t A_TIMx_CR1_UDIS;            /** @brief Update disable. */
extern const field32_t A_TIMx_CR1_URS;             /** @brief Update request source. */
extern const field32_t A_TIMx_CR1_OPM;             /** @brief One-pulse mode. */
extern const field32_t A_TIMx_CR1_DIR;             /** @brief Direction. */
extern const field32_t A_TIMx_CR1_CMS;             /** @brief Center-aligned mode selection. */
extern const field32_t A_TIMx_CR1_ARPE;            /** @brief Auto-reload preload enable. */
extern const field32_t A_TIMx_CR1_CKD;             /** @brief Clock division. */
extern const field32_t A_TIMx_CR1_UIFREMAP;        /** @brief UIF status bit remapping. */
extern const field32_t A_TIMx_CR2_MMS2;            /** @brief Master mode selection 2. */
extern const field32_t A_TIMx_CR2_TI1S;            /** @brief TI1 selection. */
extern const field32_t A_TIMx_CR2_MMS;             /** @brief Master mode selection. */
extern const field32_t A_TIMx_CR2_CCDS;            /** @brief Capture/compare DMA selection. */
extern const field32_t A_TIMx_CR2_CCUS;            /** @brief Capture/compare control update selection. */
extern const field32_t A_TIMx_CR2_CCPC;            /** @brief Capture/compare preloaded control. */
extern const field32_t A_TIMx_SMCR_SMS;            /** @brief Slave mode selection. */
extern const field32_t A_TIMx_SMCR_TS;             /** @brief Trigger selection. */
extern const field32_t A_TIMx_SMCR_MSM;            /** @brief Master/Slave mode. */
extern const field32_t A_TIMx_SMCR_ETF;            /** @brief External trigger filter. */
extern const field32_t A_TIMx_SMCR_ETPS;           /** @brief External trigger prescaler. */
extern const field32_t A_TIMx_SMCR_ECE;            /** @brief External clock enable. */
extern const field32_t A_TIMx_SMCR_ETP;            /** @brief External trigger polarity. */
extern const field32_t A_TIMx_SMCR_SMS_3;          /** @brief Slave mode selection - bit 3. */
extern const field32_t A_TIMx_SMCR_TS_4_3;         /** @brief Trigger selection - bit 4:3. */
extern const field32_t A_TIMx_DIER_TDE;            /** @brief Trigger DMA request enable. */
extern const field32_t A_TIMx_DIER_COMDE;          /** @brief COM DMA request enable. */
extern const field32_t A_TIMx_DIER_UDE;            /** @brief Update DMA request enable. */
extern const field32_t A_TIMx_DIER_TIE;            /** @brief Trigger interrupt enable. */
extern const field32_t A_TIMx_DIER_UIE;            /** @brief Update interrupt enable. */
extern const field32_t A_TIMx_DIER_BIE;            /** @brief Break interrupt enable. */
extern const field32_t A_TIMx_DIER_COMIE;          /** @brief COM interrupt enable. */
extern const field32_t A_TIMx_SR_SBIF;             /** @brief System break interrupt flag. */
extern const field32_t A_TIMx_SR_B2IF;             /** @brief Break 2 interrupt flag. */
extern const field32_t A_TIMx_SR_BIF;              /** @brief Break interrupt flag. */
extern const field32_t A_TIMx_SR_TIF;              /** @brief Trigger interrupt flag. */
extern const field32_t A_TIMx_SR_COMIF;            /** @brief COM interrupt flag. */
extern const field32_t A_TIMx_SR_UIF;              /** @brief Update interrupt flag. */
extern const field32_t A_TIMx_EGR_UG;              /** @brief Update generation. */
extern const field32_t A_TIMx_EGR_COMG;            /** @brief Capture/Compare control update generation. */
extern const field32_t A_TIMx_EGR_TG;              /** @brief Trigger generation. */
extern const field32_t A_TIMx_EGR_BG;              /** @brief Break generation. */
extern const field32_t A_TIMx_EGR_B2G;             /** @brief Break 2 generation. */
extern const field32_t A_TIMx_CCMR1_INPUT_IC2PCS;  /** @brief Input capture 2 prescaler. */
extern const field32_t A_TIMx_CCMR1_INPUT_ICPCS;   /** @brief Input capture 1 prescaler. */
extern const field32_t A_TIMx_CCMR2_OUTPUT_OC3M_3; /** @brief Output compare 3 mode - bit 3. */
extern const field32_t A_TIMx_CCMR2_OUTPUT_OC4M_4; /** @brief Output compare 4 mode - bit 3. */
extern const field32_t A_TIMx_CNT_CNT;             /** @brief Counter value. */
extern const field32_t A_TIMx_CNT_UIFCPY;          /** @brief UIF copy. */
extern const field32_t A_TIMx_PSC_PSC;             /** @brief Prescaler value. */
extern const field32_t A_TIMx_ARR_ARR;             /** @brief Auto-reload value. */
extern const field32_t A_TIMx_CCR1_CCR1;           /** @brief Capture/Compare 1 value. */
extern const field32_t A_TIMx_CCR2_CCR2;           /** @brief Capture/Compare 2 value. */
extern const field32_t A_TIMx_CCR3_CCR3;           /** @brief Capture/Compare value. */
extern const field32_t A_TIMx_CCR4_CCR4;           /** @brief Capture/Compare value. */
extern const field32_t A_TIMx_DCR_DBL;             /** @brief DMA burst length. */
extern const field32_t A_TIMx_DCR_DBA;             /** @brief DMA base address. */
extern const field32_t A_TIMx_DMAR_DMAB;           /** @brief DMA register for burst accesses. */
extern const field32_t A_TIMx_RCR_REP;             /** @brief Repetition counter value. */
extern const field32_t A_TIMx_BDTR_DTG;            /** @brief Dead-time generator setup. */
extern const field32_t A_TIMx_BDTR_LOCK;           /** @brief Lock configuration. */
extern const field32_t A_TIMx_BDTR_OSSI;           /** @brief Off-state selection for idle mode. */
extern const field32_t A_TIMx_BDTR_OSSR;           /** @brief Off-state selection for run mode. */
extern const field32_t A_TIMx_BDTR_BKE;            /** @brief Break enable. */
extern const field32_t A_TIMx_BDTR_BKP;            /** @brief Break polarity. */
extern const field32_t A_TIMx_BDTR_AOE;            /** @brief Automatic output enable. */
extern const field32_t A_TIMx_BDTR_MOE;            /** @brief Main output enable. */
extern const field32_t A_TIMx_BDTR_BKF;            /** @brief Break filter. */
extern const field32_t A_TIMx_BDTR_BK2F;           /** @brief Break 2 filter. */
extern const field32_t A_TIMx_BDTR_BK2E;           /** @brief Break 2 enable. */
extern const field32_t A_TIMx_BDTR_BK2P;           /** @brief Break 2 polarity. */
extern const field32_t A_TIMx_CCR5_CCR5;           /** @brief Capture/Compare 5 value. */
extern const field32_t A_TIMx_CRR6_CCR6;           /** @brief Capture/Compare 6 value. */
extern const field32_t A_TIMx_AF1_BKINE;           /** @brief BRK BKIN input enable. */
extern const field32_t A_TIMx_AF1_BKDF1BK0E;       /** @brief BRK dfsdm1_break[0] enable. */
extern const field32_t A_TIMx_AF1_BKINP;           /** @brief BRK BKIN input polarity. */
extern const field32_t A_TIMx_AF1_ETRSEL;          /** @brief ETR source selection. */
extern const field32_t A_TIMx_AF2_BK2INE;          /** @brief BRK2 BKIN input enable. */
extern const field32_t A_TIMx_AF2_BK2DF1BK1E;      /** @brief BRK2 dfsdm1_break[1] enable. */
extern const field32_t A_TIMx_AF2_BK2INP;          /** @brief BRK2 BKIN2 input polarity. */

/** @subsection Enumerated A_TIMx Register Field Definitions */

extern const field32_t A_TIMx_CR2_OISx[7];            /** @brief Output idle state 1. */
extern const field32_t A_TIMx_CR2_OISxN[4];           /** @brief Output idle state 1. */
extern const field32_t A_TIMx_DIER_CCxDE[5];          /** @brief Capture/Compare 1 DMA request enable. */
extern const field32_t A_TIMx_DIER_CCxIE[5];          /** @brief Capture/Compare 1 interrupt enable. */
extern const field32_t A_TIMx_SR_CCxIF[7];            /** @brief Capture/compare 1 interrupt flag. */
extern const field32_t A_TIMx_SR_CCxOF[5];            /** @brief Capture/Compare 1 overcapture flag. */
extern const field32_t A_TIMx_EGR_CCxG[5];            /** @brief Capture/compare 1 generation. */
extern const field32_t A_TIMx_CCMR1_OUTPUT_CCxS[3];   /** @brief Capture/Compare 1 selection. */
extern const field32_t A_TIMx_CCMR1_OUTPUT_OCxFE[3];  /** @brief Output compare 1 fast enable. */
extern const field32_t A_TIMx_CCMR1_OUTPUT_OCxPE[3];  /** @brief Output compare 1 preload enable. */
extern const field32_t A_TIMx_CCMR1_OUTPUT_OCxM[3];   /** @brief Output compare 1 mode. */
extern const field32_t A_TIMx_CCMR1_OUTPUT_OCxCE[3];  /** @brief Output compare 1 clear enable. */
extern const field32_t A_TIMx_CCMR1_OUTPUT_OCxM_3[3]; /** @brief Output compare 1 mode - bit 3. */
extern const field32_t A_TIMx_CCMR1_INPUT_ICxF[3];    /** @brief Input capture 1 filter. */
extern const field32_t A_TIMx_CCMR2_INPUT_ICxPSC[5];  /** @brief Input capture 3 prescaler. */
extern const field32_t A_TIMx_CCER_CCxE[7];           /** @brief Capture/Compare 1 output enable. */
extern const field32_t A_TIMx_CCER_CCxP[7];           /** @brief Capture/Compare 1 output polarity. */
extern const field32_t A_TIMx_CCER_CCxNE[4];          /** @brief Capture/Compare 1 complementary output enable. */
extern const field32_t A_TIMx_CCER_CCxNP[5];          /** @brief Capture/Compare 1 output polarity. */
extern const field32_t A_TIMx_CCMR3_OUTPUT_OCxM3[7];  /** @brief Output compare 5 mode. */
extern const field32_t A_TIMx_CCR5_GC5Cx[4];          /** @brief Group channel 5 and channel 1. */
extern const field32_t A_TIMx_AF1_BKCMPxE[3];         /** @brief BRK COMP1 enable. */
extern const field32_t A_TIMx_AF1_BKCMPxP[3];         /** @brief BRK COMP1 input polarity. */
extern const field32_t A_TIMx_AF2_BK2CMPxE[3];        /** @brief BRK2 COMP1 enable. */
extern const field32_t A_TIMx_AF2_BK2CMPxP[3];        /** @brief BRK2 COMP1 input polarit. */
extern const field32_t A_TIMx_TISEL_TIxSEL[5];        /** @brief Selects TI1[0] to TI1[15] input. */

/**************************************************************************************************
 * @section FDCANx Definitions
 **************************************************************************************************/

/** @subsection Enumerated FDCANx Register Definitions */

extern ro_reg32_t const FDCANx_FDCAN_CREL[3];   /** @brief FDCAN core release register. */
extern ro_reg32_t const FDCANx_FDCAN_ENDN[3];   /** @brief FDCAN core release register. */
extern ro_reg32_t const FDCANx_FDCAN_DBTP[3];   /** @brief FDCAN data bit timing and prescaler register. */
extern ro_reg32_t const FDCANx_FDCAN_TEST[3];   /** @brief FDCAN test register. */
extern ro_reg32_t const FDCANx_FDCAN_RWD[3];    /** @brief FDCAN RAM watchdog register. */
extern rw_reg32_t const FDCANx_FDCAN_CCCR[3];   /** @brief FDCAN CC control register. */
extern rw_reg32_t const FDCANx_FDCAN_NBTP[3];   /** @brief FDCAN nominal bit timing and prescaler register. */
extern rw_reg32_t const FDCANx_FDCAN_TSCC[3];   /** @brief FDCAN timestamp counter configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TSCV[3];   /** @brief FDCAN timestamp counter value register. */
extern rw_reg32_t const FDCANx_FDCAN_TOCC[3];   /** @brief FDCAN timeout counter configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TOCV[3];   /** @brief FDCAN timeout counter value register. */
extern rw_reg32_t const FDCANx_FDCAN_ECR[3];    /** @brief FDCAN error counter register. */
extern rw_reg32_t const FDCANx_FDCAN_PSR[3];    /** @brief FDCAN protocol status register. */
extern ro_reg32_t const FDCANx_FDCAN_TDCR[3];   /** @brief FDCAN transmitter delay compensation register. */
extern ro_reg32_t const FDCANx_FDCAN_IR[3];     /** @brief FDCAN interrupt register. */
extern ro_reg32_t const FDCANx_FDCAN_IE[3];     /** @brief FDCAN interrupt enable register. */
extern ro_reg32_t const FDCANx_FDCAN_ILS[3];    /** @brief FDCAN interrupt line select register. */
extern rw_reg32_t const FDCANx_FDCAN_ILE[3];    /** @brief FDCAN interrupt line enable register. */
extern rw_reg32_t const FDCANx_FDCAN_GFC[3];    /** @brief FDCAN global filter configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_SIDFC[3];  /** @brief FDCAN standard ID filter configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_XIDFC[3];  /** @brief FDCAN extended ID filter configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_XIDAM[3];  /** @brief FDCAN extended ID and mask register. */
extern ro_reg32_t const FDCANx_FDCAN_HPMS[3];   /** @brief FDCAN high priority message status register. */
extern ro_reg32_t const FDCANx_FDCAN_NDAT1[3];  /** @brief FDCAN new data 1 register. */
extern ro_reg32_t const FDCANx_FDCAN_NDAT2[3];  /** @brief FDCAN new data 2 register. */
extern rw_reg32_t const FDCANx_FDCAN_RXF0C[3];  /** @brief FDCAN rx FIFO 0 configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_RXF0S[3];  /** @brief FDCAN rx FIFO 0 status register. */
extern rw_reg32_t const FDCANx_FDCAN_RXF0A[3];  /** @brief CAN rx FIFO 0 acknowledge register. */
extern rw_reg32_t const FDCANx_FDCAN_RXBC[3];   /** @brief FDCAN rx buffer configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_RXF1C[3];  /** @brief FDCAN rx FIFO 1 configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_RXF1S[3];  /** @brief FDCAN rx FIFO 1 status register. */
extern rw_reg32_t const FDCANx_FDCAN_RXF1A[3];  /** @brief FDCAN rx FIFO 1 acknowledge register. */
extern rw_reg32_t const FDCANx_FDCAN_RXESC[3];  /** @brief FDCAN rx buffer element size configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TXBC[3];   /** @brief FDCAN tx buffer configuration register. */
extern ro_reg32_t const FDCANx_FDCAN_TXFQS[3];  /** @brief FDCAN tx fifo/queue status register. */
extern rw_reg32_t const FDCANx_FDCAN_TXESC[3];  /** @brief FDCAN tx buffer element size configuration register. */
extern ro_reg32_t const FDCANx_FDCAN_TXBRP[3];  /** @brief FDCAN tx buffer request pending register. */
extern rw_reg32_t const FDCANx_FDCAN_TXBAR[3];  /** @brief FDCAN tx buffer add request register. */
extern rw_reg32_t const FDCANx_FDCAN_TXBCR[3];  /** @brief FDCAN tx buffer cancellation request register. */
extern rw_reg32_t const FDCANx_FDCAN_TXBTO[3];  /** @brief FDCAN tx buffer transmission occurred register. */
extern ro_reg32_t const FDCANx_FDCAN_TXBCF[3];  /** @brief FDCAN tx buffer cancellation finished register. */
extern rw_reg32_t const FDCANx_FDCAN_TXBTIE[3]; /** @brief FDCAN tx buffer transmission interrupt enable register. */
extern rw_reg32_t const FDCANx_FDCAN_TXBCIE[3]; /** @brief FDCAN tx buffer cancellation finished interrupt enable register. */
extern rw_reg32_t const FDCANx_FDCAN_TXEFC[3];  /** @brief FDCAN tx event FIFO configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TXEFS[3];  /** @brief FDCAN tx event FIFO status register. */
extern rw_reg32_t const FDCANx_FDCAN_TXEFA[3];  /** @brief FDCAN tx event FIFO acknowledge register. */
extern rw_reg32_t const FDCANx_FDCAN_TTTMC[3];  /** @brief FDCAN TT trigger memory configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TTRMC[3];  /** @brief FDCAN TT reference message configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TTOCF[3];  /** @brief FDCAN TT operation configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TTMLM[3];  /** @brief FDCAN TT matrix limits register. */
extern rw_reg32_t const FDCANx_FDCAN_TURCF[3];  /** @brief FDCAN TUR configuration register. */
extern rw_reg32_t const FDCANx_FDCAN_TTOCN[3];  /** @brief FDCAN TT operation control register. */
extern rw_reg32_t const FDCANx_CAN_TTGTP[3];    /** @brief FDCAN TT global time preset register. */
extern rw_reg32_t const FDCANx_FDCAN_TTTMK[3];  /** @brief FDCAN TT time mark register. */
extern rw_reg32_t const FDCANx_FDCAN_TTIR[3];   /** @brief FDCAN TT interrupt register. */
extern rw_reg32_t const FDCANx_FDCAN_TTIE[3];   /** @brief FDCAN TT interrupt enable register. */
extern rw_reg32_t const FDCANx_FDCAN_TTILS[3];  /** @brief FDCAN TT interrupt line select register. */
extern rw_reg32_t const FDCANx_FDCAN_TTOST[3];  /** @brief FDCAN TT operation status register. */
extern ro_reg32_t const FDCANx_FDCAN_TURNA[3];  /** @brief FDCAN TUR numerator actual register. */
extern ro_reg32_t const FDCANx_FDCAN_TTLGT[3];  /** @brief FDCAN TT local and global time register. */
extern ro_reg32_t const FDCANx_FDCAN_TTCTC[3];  /** @brief FDCAN TT cycle time and count register. */
extern ro_reg32_t const FDCANx_FDCAN_TTCPT[3];  /** @brief FDCAN TT capture time register. */
extern ro_reg32_t const FDCANx_FDCAN_TTCSM[3];  /** @brief FDCAN TT cycle sync mark register. */
extern rw_reg32_t const FDCANx_FDCAN_TTTS[3];   /** @brief FDCAN TT trigger select register. */

/** @subsection FDCANx Register Field Definitions */

extern const field32_t FDCANx_FDCAN_CREL_REL;     /** @brief Core release. */
extern const field32_t FDCANx_FDCAN_CREL_STEP;    /** @brief Step of core release. */
extern const field32_t FDCANx_FDCAN_CREL_SUBSTEP; /** @brief Sub-step of core release. */
extern const field32_t FDCANx_FDCAN_CREL_YEAR;    /** @brief Timestamp year. */
extern const field32_t FDCANx_FDCAN_CREL_MON;     /** @brief Timestamp month. */
extern const field32_t FDCANx_FDCAN_CREL_DAY;     /** @brief Timestamp day. */
extern const field32_t FDCANx_FDCAN_DBTP_DSJW;    /** @brief Synchronization jump width. */
extern const field32_t FDCANx_FDCAN_DBTP_DBRP;    /** @brief Data bit rate prescaler. */
extern const field32_t FDCANx_FDCAN_DBTP_TDC;     /** @brief Transceiver delay compensation. */
extern const field32_t FDCANx_FDCAN_TEST_LBCK;    /** @brief Loop back mode. */
extern const field32_t FDCANx_FDCAN_TEST_TX;      /** @brief Loop back mode. */
extern const field32_t FDCANx_FDCAN_TEST_RX;      /** @brief Control of transmit pin. */
extern const field32_t FDCANx_FDCAN_RWD_WDV;      /** @brief Watchdog value. */
extern const field32_t FDCANx_FDCAN_RWD_WDC;      /** @brief Watchdog configuration. */
extern const field32_t FDCANx_FDCAN_CCCR_INIT;    /** @brief Initialization. */
extern const field32_t FDCANx_FDCAN_CCCR_CCE;     /** @brief Configuration change enable. */
extern const field32_t FDCANx_FDCAN_CCCR_ASM;     /** @brief ASM restricted operation mode. */
extern const field32_t FDCANx_FDCAN_CCCR_CSA;     /** @brief Clock stop acknowledge. */
extern const field32_t FDCANx_FDCAN_CCCR_CSR;     /** @brief Clock stop request. */
extern const field32_t FDCANx_FDCAN_CCCR_MON;     /** @brief Bus monitoring mode. */
extern const field32_t FDCANx_FDCAN_CCCR_DAR;     /** @brief Disable automatic retransmission. */
extern const field32_t FDCANx_FDCAN_CCCR_TEST;    /** @brief Test mode enable. */
extern const field32_t FDCANx_FDCAN_CCCR_FDOE;    /** @brief FD operation enable. */
extern const field32_t FDCANx_FDCAN_CCCR_BSE;     /** @brief FDCAN bit rate switching. */
extern const field32_t FDCANx_FDCAN_CCCR_PXHD;    /** @brief Protocol exception handling disable. */
extern const field32_t FDCANx_FDCAN_CCCR_EFBI;    /** @brief Edge filtering during bus integration. */
extern const field32_t FDCANx_FDCAN_CCCR_TXP;     /** @brief TXP. */
extern const field32_t FDCANx_FDCAN_CCCR_NISO;    /** @brief Non ISO operation. */
extern const field32_t FDCANx_FDCAN_NBTP_NSJW;    /** @brief NSJW: nominal (re)synchronization jump width. */
extern const field32_t FDCANx_FDCAN_NBTP_NBRP;    /** @brief Bit rate prescaler. */
extern const field32_t FDCANx_FDCAN_NBTP_NTSEG1;  /** @brief Nominal time segment before sample point. */
extern const field32_t FDCANx_FDCAN_NBTP_TSEG2;   /** @brief Nominal time segment after sample point. */
extern const field32_t FDCANx_FDCAN_TSCC_TCP;     /** @brief Timestamp counter prescaler. */
extern const field32_t FDCANx_FDCAN_TSCC_TSS;     /** @brief Timestamp select. */
extern const field32_t FDCANx_FDCAN_TSCV_TSC;     /** @brief Timestamp counter. */
extern const field32_t FDCANx_FDCAN_TOCC_ETOC;    /** @brief Enable timeout counter. */
extern const field32_t FDCANx_FDCAN_TOCC_TOS;     /** @brief Timeout select. */
extern const field32_t FDCANx_FDCAN_TOCC_TOP;     /** @brief Timeout period. */
extern const field32_t FDCANx_FDCAN_TOCV_TOC;     /** @brief Timeout counter. */
extern const field32_t FDCANx_FDCAN_ECR_CEL;      /** @brief AN error logging. */
extern const field32_t FDCANx_FDCAN_ECR_RP;       /** @brief Receive error passive. */
extern const field32_t FDCANx_FDCAN_ECR_TREC;     /** @brief Receive error counter. */
extern const field32_t FDCANx_FDCAN_ECR_TEC;      /** @brief Transmit error counter. */
extern const field32_t FDCANx_FDCAN_PSR_LEC;      /** @brief Last error code. */
extern const field32_t FDCANx_FDCAN_PSR_ACT;      /** @brief Activity. */
extern const field32_t FDCANx_FDCAN_PSR_EP;       /** @brief Error passive. */
extern const field32_t FDCANx_FDCAN_PSR_EW;       /** @brief Warning status. */
extern const field32_t FDCANx_FDCAN_PSR_BO;       /** @brief Bus_Off status. */
extern const field32_t FDCANx_FDCAN_PSR_DLEC;     /** @brief Data last error code. */
extern const field32_t FDCANx_FDCAN_PSR_RESI;     /** @brief ESI flag of last received FDCAN message. */
extern const field32_t FDCANx_FDCAN_PSR_RBRS;     /** @brief BRS flag of last received FDCAN message. */
extern const field32_t FDCANx_FDCAN_PSR_REDL;     /** @brief Received FDCAN message. */
extern const field32_t FDCANx_FDCAN_PSR_PXE;      /** @brief Protocol exception event. */
extern const field32_t FDCANx_FDCAN_PSR_TDCV;     /** @brief Transmitter delay compensation value. */
extern const field32_t FDCANx_FDCAN_TDCR_TDCF;    /** @brief Transmitter delay compensation filter window length. */
extern const field32_t FDCANx_FDCAN_TDCR_TDCO;    /** @brief Transmitter delay compensation offset. */
extern const field32_t FDCANx_FDCAN_IR_HPM;       /** @brief High priority message. */
extern const field32_t FDCANx_FDCAN_IR_TC;        /** @brief Transmission completed. */
extern const field32_t FDCANx_FDCAN_IR_TCF;       /** @brief Transmission cancellation finished. */
extern const field32_t FDCANx_FDCAN_IR_TEF;       /** @brief Tx FIFO empty. */
extern const field32_t FDCANx_FDCAN_IR_TEFN;      /** @brief Tx event FIFO new entry. */
extern const field32_t FDCANx_FDCAN_IR_TEFW;      /** @brief Tx event FIFO watermark reached. */
extern const field32_t FDCANx_FDCAN_IR_TEFF;      /** @brief Tx event FIFO full. */
extern const field32_t FDCANx_FDCAN_IR_TEFL;      /** @brief Tx event FIFO element lost. */
extern const field32_t FDCANx_FDCAN_IR_TSW;       /** @brief Timestamp wraparound. */
extern const field32_t FDCANx_FDCAN_IR_MRAF;      /** @brief Message RAM access failure. */
extern const field32_t FDCANx_FDCAN_IR_TOO;       /** @brief Timeout occurred. */
extern const field32_t FDCANx_FDCAN_IR_DRX;       /** @brief Message stored to dedicated rx buffer. */
extern const field32_t FDCANx_FDCAN_IR_ELO;       /** @brief Error logging overflow. */
extern const field32_t FDCANx_FDCAN_IR_EP;        /** @brief Error passive. */
extern const field32_t FDCANx_FDCAN_IR_EW;        /** @brief Warning status. */
extern const field32_t FDCANx_FDCAN_IR_BO;        /** @brief Bus_Off status. */
extern const field32_t FDCANx_FDCAN_IR_WDI;       /** @brief Watchdog interrupt. */
extern const field32_t FDCANx_FDCAN_IR_PEA;       /** @brief Protocol error in arbitration phase (nominal bit time is used). */
extern const field32_t FDCANx_FDCAN_IR_PED;       /** @brief Protocol error in data phase (data bit time is used). */
extern const field32_t FDCANx_FDCAN_IR_ARA;       /** @brief Access to reserved address. */
extern const field32_t FDCANx_FDCAN_IE_HPME;      /** @brief High priority message enable. */
extern const field32_t FDCANx_FDCAN_IE_TCE;       /** @brief Transmission completed enable. */
extern const field32_t FDCANx_FDCAN_IE_TCFE;      /** @brief Transmission cancellation finished enable. */
extern const field32_t FDCANx_FDCAN_IE_TEFE;      /** @brief Tx FIFO empty enable. */
extern const field32_t FDCANx_FDCAN_IE_TEFNE;     /** @brief Tx event FIFO new entry enable. */
extern const field32_t FDCANx_FDCAN_IE_TEFWE;     /** @brief Tx event FIFO watermark reached enable. */
extern const field32_t FDCANx_FDCAN_IE_TEFFE;     /** @brief Tx event FIFO full enable. */
extern const field32_t FDCANx_FDCAN_IE_TEFLE;     /** @brief Tx event FIFO element lost enable. */
extern const field32_t FDCANx_FDCAN_IE_TSWE;      /** @brief Timestamp wraparound enable. */
extern const field32_t FDCANx_FDCAN_IE_MRAFE;     /** @brief Message RAM access failure enable. */
extern const field32_t FDCANx_FDCAN_IE_TOOE;      /** @brief Timeout occurred enable. */
extern const field32_t FDCANx_FDCAN_IE_DRXE;      /** @brief Message stored to dedicated rx buffer enable. */
extern const field32_t FDCANx_FDCAN_IE_BECE;      /** @brief Bit error corrected interrupt enable. */
extern const field32_t FDCANx_FDCAN_IE_BEUE;      /** @brief Bit error uncorrected interrupt enable. */
extern const field32_t FDCANx_FDCAN_IE_ELOE;      /** @brief Error logging overflow enable. */
extern const field32_t FDCANx_FDCAN_IE_EPE;       /** @brief Error passive enable. */
extern const field32_t FDCANx_FDCAN_IE_EWE;       /** @brief Warning status enable. */
extern const field32_t FDCANx_FDCAN_IE_BOE;       /** @brief Bus_Off status enable. */
extern const field32_t FDCANx_FDCAN_IE_WDIE;      /** @brief Watchdog interrupt enable. */
extern const field32_t FDCANx_FDCAN_IE_PEAE;      /** @brief Protocol error in arbitration phase enable. */
extern const field32_t FDCANx_FDCAN_IE_PEDE;      /** @brief Protocol error in data phase enable. */
extern const field32_t FDCANx_FDCAN_IE_ARAE;      /** @brief Access to reserved address enable. */
extern const field32_t FDCANx_FDCAN_ILS_HPML;     /** @brief High priority message interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TCL;      /** @brief Transmission completed interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TCFL;     /** @brief Transmission cancellation finished interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TEFL;     /** @brief Tx FIFO empty interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TEFNL;    /** @brief Tx event FIFO new entry interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TEFWL;    /** @brief Tx event FIFO watermark reached interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TEFFL;    /** @brief Tx event FIFO full interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TEFLL;    /** @brief Tx event FIFO element lost interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TSWL;     /** @brief Timestamp wraparound interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_MRAFL;    /** @brief Message RAM access failure interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_TOOL;     /** @brief Timeout occurred interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_DRXL;     /** @brief Message stored to dedicated rx buffer interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_BECL;     /** @brief Bit error corrected interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_BEUL;     /** @brief Bit error uncorrected interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_ELOL;     /** @brief Error logging overflow interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_EPL;      /** @brief Error passive interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_EWL;      /** @brief Warning status interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_BOL;      /** @brief Bus_Off status. */
extern const field32_t FDCANx_FDCAN_ILS_WDIL;     /** @brief Watchdog interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_PEAL;     /** @brief Protocol error in arbitration phase line. */
extern const field32_t FDCANx_FDCAN_ILS_PEDL;     /** @brief Protocol error in data phase line. */
extern const field32_t FDCANx_FDCAN_ILS_ARAL;     /** @brief Access to reserved address line. */
extern const field32_t FDCANx_FDCAN_GFC_RRFE;     /** @brief Reject remote frames extended. */
extern const field32_t FDCANx_FDCAN_GFC_RRFS;     /** @brief Reject remote frames standard. */
extern const field32_t FDCANx_FDCAN_GFC_ANFE;     /** @brief Accept non-matching frames extended. */
extern const field32_t FDCANx_FDCAN_GFC_ANFS;     /** @brief Accept non-matching frames standard. */
extern const field32_t FDCANx_FDCAN_SIDFC_FLSSA;  /** @brief Filter list standard start address. */
extern const field32_t FDCANx_FDCAN_SIDFC_LSS;    /** @brief List size standard. */
extern const field32_t FDCANx_FDCAN_XIDFC_FLESA;  /** @brief Filter list standard start address. */
extern const field32_t FDCANx_FDCAN_XIDFC_LSE;    /** @brief List size extended. */
extern const field32_t FDCANx_FDCAN_XIDAM_EIDM;   /** @brief Extended ID mask. */
extern const field32_t FDCANx_FDCAN_HPMS_BIDX;    /** @brief Buffer index. */
extern const field32_t FDCANx_FDCAN_HPMS_MSI;     /** @brief Message storage indicator. */
extern const field32_t FDCANx_FDCAN_HPMS_FIDX;    /** @brief Filter index. */
extern const field32_t FDCANx_FDCAN_HPMS_FLST;    /** @brief Filter list. */
extern const field32_t FDCANx_FDCAN_RXF0C_F0SA;   /** @brief Rx FIFO 0 start address. */
extern const field32_t FDCANx_FDCAN_RXF0C_F0S;    /** @brief Rx FIFO 0 size. */
extern const field32_t FDCANx_FDCAN_RXF0C_F0WM;   /** @brief FIFO 0 watermark. */
extern const field32_t FDCANx_FDCAN_RXF0S_F0FL;   /** @brief Rx FIFO 0 fill level. */
extern const field32_t FDCANx_FDCAN_RXF0S_F0G;    /** @brief Rx FIFO 0 get index. */
extern const field32_t FDCANx_FDCAN_RXF0S_F0P;    /** @brief Rx FIFO 0 put index. */
extern const field32_t FDCANx_FDCAN_RXF0S_F0F;    /** @brief Rx FIFO 0 full. */
extern const field32_t FDCANx_FDCAN_RXF0S_RF0L;   /** @brief Rx FIFO 0 message lost. */
extern const field32_t FDCANx_FDCAN_RXF0A_FA01;   /** @brief Rx FIFO 0 acknowledge index. */
extern const field32_t FDCANx_FDCAN_RXBC_RBSA;    /** @brief Rx buffer start address. */
extern const field32_t FDCANx_FDCAN_RXF1C_F1SA;   /** @brief Rx FIFO 1 start address. */
extern const field32_t FDCANx_FDCAN_RXF1C_F1S;    /** @brief Rx FIFO 1 size. */
extern const field32_t FDCANx_FDCAN_RXF1C_F1WM;   /** @brief Rx FIFO 1 watermark. */
extern const field32_t FDCANx_FDCAN_RXF1S_F1FL;   /** @brief Rx FIFO 1 fill level. */
extern const field32_t FDCANx_FDCAN_RXF1S_F1GI;   /** @brief Rx FIFO 1 get index. */
extern const field32_t FDCANx_FDCAN_RXF1S_F1PI;   /** @brief Rx FIFO 1 put index. */
extern const field32_t FDCANx_FDCAN_RXF1S_F1F;    /** @brief Rx FIFO 1 full. */
extern const field32_t FDCANx_FDCAN_RXF1S_RF1L;   /** @brief Rx FIFO 1 message lost. */
extern const field32_t FDCANx_FDCAN_RXF1S_DMS;    /** @brief Debug message status. */
extern const field32_t FDCANx_FDCAN_RXF1A_F1AI;   /** @brief Rx FIFO 1 acknowledge index. */
extern const field32_t FDCANx_FDCAN_RXESC_RBDS;   /** @brief Rx buffer data field size:. */
extern const field32_t FDCANx_FDCAN_TXBC_TBSA;    /** @brief Tx buffers start address. */
extern const field32_t FDCANx_FDCAN_TXBC_NDTB;    /** @brief Number of dedicated transmit buffers. */
extern const field32_t FDCANx_FDCAN_TXBC_TFQS;    /** @brief Transmit fifo/queue size. */
extern const field32_t FDCANx_FDCAN_TXBC_TFQM;    /** @brief Tx fifo/queue mode. */
extern const field32_t FDCANx_FDCAN_TXFQS_TFFL;   /** @brief Tx FIFO free level. */
extern const field32_t FDCANx_FDCAN_TXFQS_TFGI;   /** @brief TFGI. */
extern const field32_t FDCANx_FDCAN_TXFQS_TFQPI;  /** @brief Tx fifo/queue put index. */
extern const field32_t FDCANx_FDCAN_TXFQS_TFQF;   /** @brief Tx fifo/queue full. */
extern const field32_t FDCANx_FDCAN_TXESC_TBDS;   /** @brief Tx buffer data field size:. */
extern const field32_t FDCANx_FDCAN_TXEFC_EFSA;   /** @brief Event FIFO start address. */
extern const field32_t FDCANx_FDCAN_TXEFC_EFS;    /** @brief Event FIFO size. */
extern const field32_t FDCANx_FDCAN_TXEFC_EFWM;   /** @brief Event FIFO watermark. */
extern const field32_t FDCANx_FDCAN_TXEFS_EFFL;   /** @brief Event FIFO fill level. */
extern const field32_t FDCANx_FDCAN_TXEFS_EFGI;   /** @brief Event FIFO get index. */
extern const field32_t FDCANx_FDCAN_TXEFS_EFF;    /** @brief Event FIFO full. */
extern const field32_t FDCANx_FDCAN_TXEFS_TEFL;   /** @brief Tx event FIFO element lost. */
extern const field32_t FDCANx_FDCAN_TXEFA_EFAI;   /** @brief Event FIFO acknowledge index. */
extern const field32_t FDCANx_FDCAN_TTTMC_TMSA;   /** @brief Trigger memory start address. */
extern const field32_t FDCANx_FDCAN_TTTMC_TME;    /** @brief Trigger memory elements. */
extern const field32_t FDCANx_FDCAN_TTRMC_RID;    /** @brief Reference identifier. */
extern const field32_t FDCANx_FDCAN_TTRMC_XTD;    /** @brief Extended identifier. */
extern const field32_t FDCANx_FDCAN_TTRMC_RMPS;   /** @brief Reference message payload select. */
extern const field32_t FDCANx_FDCAN_TTOCF_OM;     /** @brief Operation mode. */
extern const field32_t FDCANx_FDCAN_TTOCF_GEN;    /** @brief Gap enable. */
extern const field32_t FDCANx_FDCAN_TTOCF_TM;     /** @brief Time master. */
extern const field32_t FDCANx_FDCAN_TTOCF_LDSDL;  /** @brief LD of synchronization deviation limit. */
extern const field32_t FDCANx_FDCAN_TTOCF_IRTO;   /** @brief Initial reference trigger offset. */
extern const field32_t FDCANx_FDCAN_TTOCF_EECS;   /** @brief Enable external clock synchronization. */
extern const field32_t FDCANx_FDCAN_TTOCF_AWL;    /** @brief Application watchdog limit. */
extern const field32_t FDCANx_FDCAN_TTOCF_EGTF;   /** @brief Enable global time filtering. */
extern const field32_t FDCANx_FDCAN_TTOCF_ECC;    /** @brief Enable clock calibration. */
extern const field32_t FDCANx_FDCAN_TTOCF_EVTP;   /** @brief Event trigger polarity. */
extern const field32_t FDCANx_FDCAN_TTMLM_CCM;    /** @brief Cycle count max. */
extern const field32_t FDCANx_FDCAN_TTMLM_CSS;    /** @brief Cycle start synchronization. */
extern const field32_t FDCANx_FDCAN_TTMLM_TXEW;   /** @brief Tx enable window. */
extern const field32_t FDCANx_FDCAN_TTMLM_ENTT;   /** @brief Expected number of tx triggers. */
extern const field32_t FDCANx_FDCAN_TURCF_NCL;    /** @brief Numerator configuration low. */
extern const field32_t FDCANx_FDCAN_TURCF_DC;     /** @brief Denominator configuration. */
extern const field32_t FDCANx_FDCAN_TURCF_ELT;    /** @brief Enable local time. */
extern const field32_t FDCANx_FDCAN_TTOCN_SGT;    /** @brief Set global time. */
extern const field32_t FDCANx_FDCAN_TTOCN_ECS;    /** @brief External clock synchronization. */
extern const field32_t FDCANx_FDCAN_TTOCN_SWP;    /** @brief Stop watch polarity. */
extern const field32_t FDCANx_FDCAN_TTOCN_SWS;    /** @brief Stop watch source. */
extern const field32_t FDCANx_FDCAN_TTOCN_RTIE;   /** @brief Register time mark interrupt pulse enable. */
extern const field32_t FDCANx_FDCAN_TTOCN_TMC;    /** @brief Register time mark compare. */
extern const field32_t FDCANx_FDCAN_TTOCN_TTIE;   /** @brief Trigger time mark interrupt pulse enable. */
extern const field32_t FDCANx_FDCAN_TTOCN_GCS;    /** @brief Gap control select. */
extern const field32_t FDCANx_FDCAN_TTOCN_FGP;    /** @brief Finish gap. */
extern const field32_t FDCANx_FDCAN_TTOCN_TMG;    /** @brief Time mark gap. */
extern const field32_t FDCANx_FDCAN_TTOCN_NIG;    /** @brief Next is gap. */
extern const field32_t FDCANx_FDCAN_TTOCN_ESCN;   /** @brief External synchronization control. */
extern const field32_t FDCANx_FDCAN_TTOCN_LCKC;   /** @brief TT operation control register locked. */
extern const field32_t FDCANx_CAN_TTGTP_NCL;      /** @brief Time preset. */
extern const field32_t FDCANx_CAN_TTGTP_CTP;      /** @brief Cycle time target phase. */
extern const field32_t FDCANx_FDCAN_TTTMK_TM;     /** @brief Time mark. */
extern const field32_t FDCANx_FDCAN_TTTMK_TICC;   /** @brief Time mark cycle code. */
extern const field32_t FDCANx_FDCAN_TTTMK_LCKM;   /** @brief TT time mark register locked. */
extern const field32_t FDCANx_FDCAN_TTIR_SBC;     /** @brief Start of basic cycle. */
extern const field32_t FDCANx_FDCAN_TTIR_SMC;     /** @brief Start of matrix cycle. */
extern const field32_t FDCANx_FDCAN_TTIR_CSM;     /** @brief Change of synchronization mode. */
extern const field32_t FDCANx_FDCAN_TTIR_SOG;     /** @brief Start of gap. */
extern const field32_t FDCANx_FDCAN_TTIR_RTMI;    /** @brief Register time mark interrupt. */
extern const field32_t FDCANx_FDCAN_TTIR_TTMI;    /** @brief Trigger time mark event internal. */
extern const field32_t FDCANx_FDCAN_TTIR_SWE;     /** @brief Stop watch event. */
extern const field32_t FDCANx_FDCAN_TTIR_GTW;     /** @brief Global time wrap. */
extern const field32_t FDCANx_FDCAN_TTIR_GTD;     /** @brief Global time discontinuity. */
extern const field32_t FDCANx_FDCAN_TTIR_GTE;     /** @brief Global time error. */
extern const field32_t FDCANx_FDCAN_TTIR_TXU;     /** @brief Tx count underflow. */
extern const field32_t FDCANx_FDCAN_TTIR_TXO;     /** @brief Tx count overflow. */
extern const field32_t FDCANx_FDCAN_TTIR_ELC;     /** @brief Error level changed. */
extern const field32_t FDCANx_FDCAN_TTIR_IWTG;    /** @brief Initialization watch trigger. */
extern const field32_t FDCANx_FDCAN_TTIR_WT;      /** @brief Watch trigger. */
extern const field32_t FDCANx_FDCAN_TTIR_AW;      /** @brief Application watchdog. */
extern const field32_t FDCANx_FDCAN_TTIR_CER;     /** @brief Configuration error. */
extern const field32_t FDCANx_FDCAN_TTIE_SBCE;    /** @brief Start of basic cycle interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_SMCE;    /** @brief Start of matrix cycle interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_CSME;    /** @brief Change of synchronization mode interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_SOGE;    /** @brief Start of gap interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_RTMIE;   /** @brief Register time mark interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_TTMIE;   /** @brief Trigger time mark event internal interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_SWEE;    /** @brief Stop watch event interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_GTWE;    /** @brief Global time wrap interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_GTDE;    /** @brief Global time discontinuity interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_GTEE;    /** @brief Global time error interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_TXUE;    /** @brief Tx count underflow interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_TXOE;    /** @brief Tx count overflow interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_ELCE;    /** @brief Change error level interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_IWTGE;   /** @brief Initialization watch trigger interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_WTE;     /** @brief Watch trigger interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_AWE;     /** @brief Application watchdog interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTIE_CERE;    /** @brief Configuration error interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTILS_SBCL;   /** @brief Start of basic cycle interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_SMCL;   /** @brief Start of matrix cycle interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_CSML;   /** @brief Change of synchronization mode interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_SOGL;   /** @brief Start of gap interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_RTMIL;  /** @brief Register time mark interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_TTMIL;  /** @brief Trigger time mark event internal interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_SWEL;   /** @brief Stop watch event interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_GTWL;   /** @brief Global time wrap interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_GTDL;   /** @brief Global time discontinuity interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_GTEL;   /** @brief Global time error interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_TXUL;   /** @brief Tx count underflow interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_TXOL;   /** @brief Tx count overflow interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_ELCL;   /** @brief Change error level interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_IWTGL;  /** @brief Initialization watch trigger interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_WTL;    /** @brief Watch trigger interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_AWL;    /** @brief Application watchdog interrupt line. */
extern const field32_t FDCANx_FDCAN_TTILS_CERL;   /** @brief Configuration error interrupt line. */
extern const field32_t FDCANx_FDCAN_TTOST_EL;     /** @brief Error level. */
extern const field32_t FDCANx_FDCAN_TTOST_MS;     /** @brief Master state. */
extern const field32_t FDCANx_FDCAN_TTOST_SYS;    /** @brief Synchronization state. */
extern const field32_t FDCANx_FDCAN_TTOST_GTP;    /** @brief Quality of global time phase. */
extern const field32_t FDCANx_FDCAN_TTOST_QCS;    /** @brief Quality of clock speed. */
extern const field32_t FDCANx_FDCAN_TTOST_RTO;    /** @brief Reference trigger offset. */
extern const field32_t FDCANx_FDCAN_TTOST_WGTD;   /** @brief Wait for global time discontinuity. */
extern const field32_t FDCANx_FDCAN_TTOST_GFI;    /** @brief Gap finished indicator. */
extern const field32_t FDCANx_FDCAN_TTOST_TMP;    /** @brief Time master priority. */
extern const field32_t FDCANx_FDCAN_TTOST_GSI;    /** @brief Gap started indicator. */
extern const field32_t FDCANx_FDCAN_TTOST_WFE;    /** @brief Wait for event. */
extern const field32_t FDCANx_FDCAN_TTOST_AWE;    /** @brief Application watchdog event. */
extern const field32_t FDCANx_FDCAN_TTOST_WECS;   /** @brief Wait for external clock synchronization. */
extern const field32_t FDCANx_FDCAN_TTOST_SPL;    /** @brief Schedule phase lock. */
extern const field32_t FDCANx_FDCAN_TURNA_NAV;    /** @brief Numerator actual value. */
extern const field32_t FDCANx_FDCAN_TTLGT_LT;     /** @brief Local time. */
extern const field32_t FDCANx_FDCAN_TTLGT_GT;     /** @brief Global time. */
extern const field32_t FDCANx_FDCAN_TTCTC_CT;     /** @brief Cycle time. */
extern const field32_t FDCANx_FDCAN_TTCTC_CC;     /** @brief Cycle count. */
extern const field32_t FDCANx_FDCAN_TTCPT_CT;     /** @brief Cycle count value. */
extern const field32_t FDCANx_FDCAN_TTCPT_SWV;    /** @brief Stop watch value. */
extern const field32_t FDCANx_FDCAN_TTCSM_CSM;    /** @brief Cycle sync mark. */
extern const field32_t FDCANx_FDCAN_TTTS_SWTDEL;  /** @brief Stop watch trigger input selection. */
extern const field32_t FDCANx_FDCAN_TTTS_EVTSEL;  /** @brief Event trigger input selection. */

/** @subsection Enumerated FDCANx Register Field Definitions */

extern const field32_t FDCANx_FDCAN_DBTP_DTSEGx[3]; /** @brief Data time segment after sample point. */
extern const field32_t FDCANx_FDCAN_IR_RFxN[2];     /** @brief Rx FIFO 0 new message. */
extern const field32_t FDCANx_FDCAN_IR_RFxW[2];     /** @brief Rx FIFO 0 full. */
extern const field32_t FDCANx_FDCAN_IR_RFxF[2];     /** @brief Rx FIFO 0 full. */
extern const field32_t FDCANx_FDCAN_IR_RFxL[2];     /** @brief Rx FIFO 0 message lost. */
extern const field32_t FDCANx_FDCAN_IE_RFxNE[2];    /** @brief Rx FIFO 0 new message enable. */
extern const field32_t FDCANx_FDCAN_IE_RFxWE[2];    /** @brief Rx FIFO 0 full enable. */
extern const field32_t FDCANx_FDCAN_IE_RFxFE[2];    /** @brief Rx FIFO 0 full enable. */
extern const field32_t FDCANx_FDCAN_IE_RFxLE[2];    /** @brief Rx FIFO 0 message lost enable. */
extern const field32_t FDCANx_FDCAN_ILS_RFxNL[2];   /** @brief Rx FIFO 0 new message interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_RFxWL[2];   /** @brief Rx FIFO 0 watermark reached interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_RFxFL[2];   /** @brief Rx FIFO 0 full interrupt line. */
extern const field32_t FDCANx_FDCAN_ILS_RFxLL[2];   /** @brief Rx FIFO 0 message lost interrupt line. */
extern const field32_t FDCANx_FDCAN_ILE_EINTx[2];   /** @brief Enable interrupt line 0. */
extern const field32_t FDCANx_FDCAN_NDAT1_NDx[32];  /** @brief New data. */
extern const field32_t FDCANx_FDCAN_NDAT2_ND3x[10]; /** @brief New data. */
extern const field32_t FDCANx_FDCAN_RXESC_FxDS[2];  /** @brief Rx FIFO 1 data field size:. */
extern const field32_t FDCANx_FDCAN_TTIR_SEx[3];    /** @brief Scheduling error 1. */
extern const field32_t FDCANx_FDCAN_TTIE_SExE[3];   /** @brief Scheduling error 1 interrupt enable. */
extern const field32_t FDCANx_FDCAN_TTILS_SExL[3];  /** @brief Scheduling error 1 interrupt line. */

/**************************************************************************************************
 * @section CAN_CCU Definitions
 **************************************************************************************************/

/** @subsection CAN_CCU Register Definitions */

extern rw_reg32_t const CAN_CCU_CREL;  /** @brief Clock calibration unit core release register. */
extern rw_reg32_t const CAN_CCU_CCFG;  /** @brief Calibration configuration register. */
extern rw_reg32_t const CAN_CCU_CSTAT; /** @brief Calibration status register. */
extern rw_reg32_t const CAN_CCU_CWD;   /** @brief Calibration watchdog register. */
extern rw_reg32_t const CAN_CCU_IR;    /** @brief Clock calibration unit interrupt register. */
extern rw_reg32_t const CAN_CCU_IE;    /** @brief Clock calibration unit interrupt enable register. */

/** @subsection CAN_CCU Register Field Definitions */

extern const field32_t CAN_CCU_CREL_DAY;     /** @brief Time stamp day. */
extern const field32_t CAN_CCU_CREL_MON;     /** @brief Time stamp month. */
extern const field32_t CAN_CCU_CREL_YEAR;    /** @brief Time stamp year. */
extern const field32_t CAN_CCU_CREL_SUBSTEP; /** @brief Sub-step of core release. */
extern const field32_t CAN_CCU_CREL_STEP;    /** @brief Step of core release. */
extern const field32_t CAN_CCU_CREL_REL;     /** @brief Core release. */
extern const field32_t CAN_CCU_CCFG_TQBT;    /** @brief Time quanta per bit time. */
extern const field32_t CAN_CCU_CCFG_BCC;     /** @brief Bypass clock calibration. */
extern const field32_t CAN_CCU_CCFG_CFL;     /** @brief Calibration field length. */
extern const field32_t CAN_CCU_CCFG_OCPM;    /** @brief Oscillator clock periods minimum. */
extern const field32_t CAN_CCU_CCFG_CDIV;    /** @brief Clock divider. */
extern const field32_t CAN_CCU_CCFG_SWR;     /** @brief Software reset. */
extern const field32_t CAN_CCU_CSTAT_OCPC;   /** @brief Oscillator clock period counter. */
extern const field32_t CAN_CCU_CSTAT_TQC;    /** @brief Time quanta counter. */
extern const field32_t CAN_CCU_CSTAT_CALS;   /** @brief Calibration state. */
extern const field32_t CAN_CCU_CWD_WDC;      /** @brief WDC. */
extern const field32_t CAN_CCU_CWD_WDV;      /** @brief WDV. */
extern const field32_t CAN_CCU_IR_CWE;       /** @brief Calibration watchdog event. */
extern const field32_t CAN_CCU_IR_CSC;       /** @brief Calibration state changed. */
extern const field32_t CAN_CCU_IE_CWEE;      /** @brief Calibration watchdog event enable. */
extern const field32_t CAN_CCU_IE_CSCE;      /** @brief Calibration state changed enable. */

/**************************************************************************************************
 * @section MDIOS Definitions
 **************************************************************************************************/

/** @subsection MDIOS Register Definitions */

extern rw_reg32_t const MDIOS_MDIOS_CR;      /** @brief MDIOS configuration register. */
extern ro_reg32_t const MDIOS_MDIOS_WRFR;    /** @brief MDIOS write flag register. */
extern rw_reg32_t const MDIOS_MDIOS_CWRFR;   /** @brief MDIOS clear write flag register. */
extern ro_reg32_t const MDIOS_MDIOS_RDFR;    /** @brief MDIOS read flag register. */
extern rw_reg32_t const MDIOS_MDIOS_CRDFR;   /** @brief MDIOS clear read flag register. */
extern ro_reg32_t const MDIOS_MDIOS_SR;      /** @brief MDIOS status register. */
extern rw_reg32_t const MDIOS_MDIOS_CLRFR;   /** @brief MDIOS clear flag register. */
extern ro_reg32_t const MDIOS_MDIOS_DINR0;   /** @brief MDIOS input data register 0. */
extern ro_reg32_t const MDIOS_MDIOS_DINR1;   /** @brief MDIOS input data register 1. */
extern ro_reg32_t const MDIOS_MDIOS_DINR2;   /** @brief MDIOS input data register 2. */
extern ro_reg32_t const MDIOS_MDIOS_DINR3;   /** @brief MDIOS input data register 3. */
extern ro_reg32_t const MDIOS_MDIOS_DINR4;   /** @brief MDIOS input data register 4. */
extern ro_reg32_t const MDIOS_MDIOS_DINR5;   /** @brief MDIOS input data register 5. */
extern ro_reg32_t const MDIOS_MDIOS_DINR6;   /** @brief MDIOS input data register 6. */
extern ro_reg32_t const MDIOS_MDIOS_DINR7;   /** @brief MDIOS input data register 7. */
extern ro_reg32_t const MDIOS_MDIOS_DINR8;   /** @brief MDIOS input data register 8. */
extern ro_reg32_t const MDIOS_MDIOS_DINR9;   /** @brief MDIOS input data register 9. */
extern ro_reg32_t const MDIOS_MDIOS_DINR10;  /** @brief MDIOS input data register 10. */
extern ro_reg32_t const MDIOS_MDIOS_DINR11;  /** @brief MDIOS input data register 11. */
extern ro_reg32_t const MDIOS_MDIOS_DINR12;  /** @brief MDIOS input data register 12. */
extern ro_reg32_t const MDIOS_MDIOS_DINR13;  /** @brief MDIOS input data register 13. */
extern ro_reg32_t const MDIOS_MDIOS_DINR14;  /** @brief MDIOS input data register 14. */
extern ro_reg32_t const MDIOS_MDIOS_DINR15;  /** @brief MDIOS input data register 15. */
extern ro_reg32_t const MDIOS_MDIOS_DINR16;  /** @brief MDIOS input data register 16. */
extern ro_reg32_t const MDIOS_MDIOS_DINR17;  /** @brief MDIOS input data register 17. */
extern ro_reg32_t const MDIOS_MDIOS_DINR18;  /** @brief MDIOS input data register 18. */
extern ro_reg32_t const MDIOS_MDIOS_DINR19;  /** @brief MDIOS input data register 19. */
extern ro_reg32_t const MDIOS_MDIOS_DINR20;  /** @brief MDIOS input data register 20. */
extern ro_reg32_t const MDIOS_MDIOS_DINR21;  /** @brief MDIOS input data register 21. */
extern ro_reg32_t const MDIOS_MDIOS_DINR22;  /** @brief MDIOS input data register 22. */
extern ro_reg32_t const MDIOS_MDIOS_DINR23;  /** @brief MDIOS input data register 23. */
extern ro_reg32_t const MDIOS_MDIOS_DINR24;  /** @brief MDIOS input data register 24. */
extern ro_reg32_t const MDIOS_MDIOS_DINR25;  /** @brief MDIOS input data register 25. */
extern ro_reg32_t const MDIOS_MDIOS_DINR26;  /** @brief MDIOS input data register 26. */
extern ro_reg32_t const MDIOS_MDIOS_DINR27;  /** @brief MDIOS input data register 27. */
extern ro_reg32_t const MDIOS_MDIOS_DINR28;  /** @brief MDIOS input data register 28. */
extern ro_reg32_t const MDIOS_MDIOS_DINR29;  /** @brief MDIOS input data register 29. */
extern ro_reg32_t const MDIOS_MDIOS_DINR30;  /** @brief MDIOS input data register 30. */
extern ro_reg32_t const MDIOS_MDIOS_DINR31;  /** @brief MDIOS input data register 31. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR0;  /** @brief MDIOS output data register 0. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR1;  /** @brief MDIOS output data register 1. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR2;  /** @brief MDIOS output data register 2. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR3;  /** @brief MDIOS output data register 3. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR4;  /** @brief MDIOS output data register 4. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR5;  /** @brief MDIOS output data register 5. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR6;  /** @brief MDIOS output data register 6. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR7;  /** @brief MDIOS output data register 7. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR8;  /** @brief MDIOS output data register 8. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR9;  /** @brief MDIOS output data register 9. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR10; /** @brief MDIOS output data register 10. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR11; /** @brief MDIOS output data register 11. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR12; /** @brief MDIOS output data register 12. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR13; /** @brief MDIOS output data register 13. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR14; /** @brief MDIOS output data register 14. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR15; /** @brief MDIOS output data register 15. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR16; /** @brief MDIOS output data register 16. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR17; /** @brief MDIOS output data register 17. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR18; /** @brief MDIOS output data register 18. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR19; /** @brief MDIOS output data register 19. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR20; /** @brief MDIOS output data register 20. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR21; /** @brief MDIOS output data register 21. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR22; /** @brief MDIOS output data register 22. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR23; /** @brief MDIOS output data register 23. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR24; /** @brief MDIOS output data register 24. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR25; /** @brief MDIOS output data register 25. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR26; /** @brief MDIOS output data register 26. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR27; /** @brief MDIOS output data register 27. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR28; /** @brief MDIOS output data register 28. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR29; /** @brief MDIOS output data register 29. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR30; /** @brief MDIOS output data register 30. */
extern rw_reg32_t const MDIOS_MDIOS_DOUTR31; /** @brief MDIOS output data register 31. */

/** @subsection MDIOS Register Field Definitions */

extern const field32_t MDIOS_MDIOS_CR_EN;           /** @brief Peripheral enable. */
extern const field32_t MDIOS_MDIOS_CR_WRIE;         /** @brief Register write interrupt enable. */
extern const field32_t MDIOS_MDIOS_CR_RDIE;         /** @brief Register read interrupt enable. */
extern const field32_t MDIOS_MDIOS_CR_EIE;          /** @brief Error interrupt enable. */
extern const field32_t MDIOS_MDIOS_CR_DPC;          /** @brief Disable preamble check. */
extern const field32_t MDIOS_MDIOS_CR_PORT_ADDRESS; /** @brief Slaves's address. */
extern const field32_t MDIOS_MDIOS_SR_PERF;         /** @brief Preamble error flag. */
extern const field32_t MDIOS_MDIOS_SR_SERF;         /** @brief Start error flag. */
extern const field32_t MDIOS_MDIOS_SR_TERF;         /** @brief Turnaround error flag. */
extern const field32_t MDIOS_MDIOS_CLRFR_CPERF;     /** @brief Clear the preamble error flag. */
extern const field32_t MDIOS_MDIOS_CLRFR_CSERF;     /** @brief Clear the start error flag. */
extern const field32_t MDIOS_MDIOS_CLRFR_CTERF;     /** @brief Clear the turnaround error flag. */
extern const field32_t MDIOS_MDIOS_DINR0_DIN0;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR1_DIN1;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR2_DIN2;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR3_DIN3;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR4_DIN4;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR5_DIN5;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR6_DIN6;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR7_DIN7;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR8_DIN8;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR9_DIN9;      /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR10_DIN10;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR11_DIN11;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR12_DIN12;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR13_DIN13;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR14_DIN14;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR15_DIN15;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR16_DIN16;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR17_DIN17;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR18_DIN18;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR19_DIN19;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR20_DIN20;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR21_DIN21;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR22_DIN22;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR23_DIN23;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR24_DIN24;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR25_DIN25;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR26_DIN26;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR27_DIN27;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR28_DIN28;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR29_DIN29;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR30_DIN30;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DINR31_DIN31;    /** @brief Input data received from MDIO master during write frames. */
extern const field32_t MDIOS_MDIOS_DOUTR0_DOUT0;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR1_DOUT1;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR2_DOUT2;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR3_DOUT3;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR4_DOUT4;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR5_DOUT5;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR6_DOUT6;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR7_DOUT7;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR8_DOUT8;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR9_DOUT9;    /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR10_DOUT10;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR11_DOUT11;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR12_DOUT12;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR13_DOUT13;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR14_DOUT14;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR15_DOUT15;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR16_DOUT16;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR17_DOUT17;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR18_DOUT18;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR19_DOUT19;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR20_DOUT20;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR21_DOUT21;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR22_DOUT22;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR23_DOUT23;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR24_DOUT24;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR25_DOUT25;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR26_DOUT26;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR27_DOUT27;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR28_DOUT28;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR29_DOUT29;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR30_DOUT30;  /** @brief Output data sent to MDIO master during read frames. */
extern const field32_t MDIOS_MDIOS_DOUTR31_DOUT31;  /** @brief Output data sent to MDIO master during read frames. */

/**************************************************************************************************
 * @section OPAMP Definitions
 **************************************************************************************************/

/** @subsection OPAMP Register Definitions */

extern rw_reg32_t const OPAMP_OPAMP1_CSR; /** @brief OPAMP1 control/status register. */
extern rw_reg32_t const OPAMP_OPAMP2_CSR; /** @brief OPAMP2 control/status register. */

/** @subsection Enumerated OPAMP Register Definitions */

extern rw_reg32_t const OPAMP_OPAMPx_OTR[3];   /** @brief OPAMP1 offset trimming register in normal mode. */
extern rw_reg32_t const OPAMP_OPAMPx_HSOTR[3]; /** @brief OPAMP1 offset trimming register in low-power mode. */

/** @subsection OPAMP Register Field Definitions */

extern const field32_t OPAMP_OPAMP1_CSR_OPAEN;           /** @brief Operational amplifier enable. */
extern const field32_t OPAMP_OPAMP1_CSR_FORCE_VP;        /** @brief Force internal reference on VP (reserved for test. */
extern const field32_t OPAMP_OPAMP1_CSR_VP_SEL;          /** @brief Operational amplifier PGA mode. */
extern const field32_t OPAMP_OPAMP1_CSR_VM_SEL;          /** @brief Inverting input selection. */
extern const field32_t OPAMP_OPAMP1_CSR_OPAHSM;          /** @brief Operational amplifier high-speed mode. */
extern const field32_t OPAMP_OPAMP1_CSR_CALON;           /** @brief Calibration mode enabled. */
extern const field32_t OPAMP_OPAMP1_CSR_CALSEL;          /** @brief Calibration selection. */
extern const field32_t OPAMP_OPAMP1_CSR_PGA_GAIN;        /** @brief Allows to switch from AOP offset trimmed values to AOP offset. */
extern const field32_t OPAMP_OPAMP1_CSR_USERTRIM;        /** @brief User trimming enable. */
extern const field32_t OPAMP_OPAMP1_CSR_TSTREF;          /** @brief OPAMP calibration reference voltage output control (reserved for test). */
extern const field32_t OPAMP_OPAMP1_CSR_CALOUT;          /** @brief Operational amplifier calibration output. */
extern const field32_t OPAMP_OPAMPx_OTR_TRIMOFFSETN;     /** @brief Trim for NMOS differential pairs. */
extern const field32_t OPAMP_OPAMPx_OTR_TRIMOFFSETP;     /** @brief Trim for PMOS differential pairs. */
extern const field32_t OPAMP_OPAMPx_HSOTR_TRIMLPOFFSETN; /** @brief Trim for NMOS differential pairs. */
extern const field32_t OPAMP_OPAMPx_HSOTR_TRIMLPOFFSETP; /** @brief Trim for PMOS differential pairs. */
extern const field32_t OPAMP_OPAMP2_CSR_OPAEN;           /** @brief Operational amplifier enable. */
extern const field32_t OPAMP_OPAMP2_CSR_FORCE_VP;        /** @brief Force internal reference on VP (reserved for test). */
extern const field32_t OPAMP_OPAMP2_CSR_VM_SEL;          /** @brief Inverting input selection. */
extern const field32_t OPAMP_OPAMP2_CSR_OPAHSM;          /** @brief Operational amplifier high-speed mode. */
extern const field32_t OPAMP_OPAMP2_CSR_CALON;           /** @brief Calibration mode enabled. */
extern const field32_t OPAMP_OPAMP2_CSR_CALSEL;          /** @brief Calibration selection. */
extern const field32_t OPAMP_OPAMP2_CSR_PGA_GAIN;        /** @brief Operational amplifier programmable amplifier gain value. */
extern const field32_t OPAMP_OPAMP2_CSR_USERTRIM;        /** @brief User trimming enable. */
extern const field32_t OPAMP_OPAMP2_CSR_TSTREF;          /** @brief OPAMP calibration reference voltage output control (reserved for test). */
extern const field32_t OPAMP_OPAMP2_CSR_CALOUT;          /** @brief Operational amplifier calibration output. */

/**************************************************************************************************
 * @section SWPMI Definitions
 **************************************************************************************************/

/** @subsection SWPMI Register Definitions */

extern rw_reg32_t const SWPMI_CR;  /** @brief SWPMI configuration/control register. */
extern rw_reg32_t const SWPMI_BRR; /** @brief SWPMI bitrate register. */
extern ro_reg32_t const SWPMI_ISR; /** @brief SWPMI interrupt and status register. */
extern rw_reg32_t const SWPMI_ICR; /** @brief SWPMI interrupt flag clear register. */
extern rw_reg32_t const SWPMI_IER; /** @brief SWPMI interrupt enable register. */
extern ro_reg32_t const SWPMI_RFL; /** @brief SWPMI receive frame length register. */
extern rw_reg32_t const SWPMI_TDR; /** @brief SWPMI transmit data register. */
extern ro_reg32_t const SWPMI_RDR; /** @brief SWPMI receive data register. */
extern rw_reg32_t const SWPMI_OR;  /** @brief SWPMI option register. */

/** @subsection SWPMI Register Field Definitions */

extern const field32_t SWPMI_CR_RXDMA;     /** @brief Reception DMA enable. */
extern const field32_t SWPMI_CR_TXDMA;     /** @brief Transmission DMA enable. */
extern const field32_t SWPMI_CR_RXMODE;    /** @brief Reception buffering mode. */
extern const field32_t SWPMI_CR_TXMODE;    /** @brief Transmission buffering mode. */
extern const field32_t SWPMI_CR_LPBK;      /** @brief Loopback mode enable. */
extern const field32_t SWPMI_CR_SWPACT;    /** @brief Single wire protocol master interface activate. */
extern const field32_t SWPMI_CR_DEACT;     /** @brief Single wire protocol master interface deactivate. */
extern const field32_t SWPMI_CR_SWPTEN;    /** @brief Single wire protocol master transceiver enable. */
extern const field32_t SWPMI_BRR_BR;       /** @brief Bitrate prescaler. */
extern const field32_t SWPMI_ISR_RXBFF;    /** @brief Receive buffer full flag. */
extern const field32_t SWPMI_ISR_TXBEF;    /** @brief Transmit buffer empty flag. */
extern const field32_t SWPMI_ISR_RXBERF;   /** @brief Receive CRC error flag. */
extern const field32_t SWPMI_ISR_RXOVRF;   /** @brief Receive overrun error flag. */
extern const field32_t SWPMI_ISR_TXUNRF;   /** @brief Transmit underrun error flag. */
extern const field32_t SWPMI_ISR_RXNE;     /** @brief Receive data register not empty. */
extern const field32_t SWPMI_ISR_TXE;      /** @brief Transmit data register empty. */
extern const field32_t SWPMI_ISR_TCF;      /** @brief Transfer complete flag. */
extern const field32_t SWPMI_ISR_SRF;      /** @brief Slave resume flag. */
extern const field32_t SWPMI_ISR_SUSP;     /** @brief SUSPEND flag. */
extern const field32_t SWPMI_ISR_DEACTF;   /** @brief DEACTIVATED flag. */
extern const field32_t SWPMI_ISR_RDYF;     /** @brief Transceiver ready flag. */
extern const field32_t SWPMI_ICR_CRXBFF;   /** @brief Clear receive buffer full flag. */
extern const field32_t SWPMI_ICR_CTXBEF;   /** @brief Clear transmit buffer empty flag. */
extern const field32_t SWPMI_ICR_CRXBERF;  /** @brief Clear receive CRC error flag. */
extern const field32_t SWPMI_ICR_CRXOVRF;  /** @brief Clear receive overrun error flag. */
extern const field32_t SWPMI_ICR_CTXUNRF;  /** @brief Clear transmit underrun error flag. */
extern const field32_t SWPMI_ICR_CTCF;     /** @brief Clear transfer complete flag. */
extern const field32_t SWPMI_ICR_CSRF;     /** @brief Clear slave resume flag. */
extern const field32_t SWPMI_ICR_CRDYF;    /** @brief Clear transceiver ready flag. */
extern const field32_t SWPMI_IER_RXBFIE;   /** @brief Receive buffer full interrupt enable. */
extern const field32_t SWPMI_IER_TXBEIE;   /** @brief Transmit buffer empty interrupt enable. */
extern const field32_t SWPMI_IER_RXBERIE;  /** @brief Receive CRC error interrupt enable. */
extern const field32_t SWPMI_IER_RXOVRIE;  /** @brief Receive overrun error interrupt enable. */
extern const field32_t SWPMI_IER_TXUNRIE;  /** @brief Transmit underrun error interrupt enable. */
extern const field32_t SWPMI_IER_RIE;      /** @brief Receive interrupt enable. */
extern const field32_t SWPMI_IER_TIE;      /** @brief Transmit interrupt enable. */
extern const field32_t SWPMI_IER_TCIE;     /** @brief Transmit complete interrupt enable. */
extern const field32_t SWPMI_IER_SRIE;     /** @brief Slave resume interrupt enable. */
extern const field32_t SWPMI_IER_RDYIE;    /** @brief Transceiver ready interrupt enable. */
extern const field32_t SWPMI_RFL_RFL;      /** @brief Receive frame length. */
extern const field32_t SWPMI_OR_SWP_TBYP;  /** @brief SWP transceiver bypass. */
extern const field32_t SWPMI_OR_SWP_CLASS; /** @brief SWP class selection. */

/**************************************************************************************************
 * @section G_TIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated G_TIMx Register Definitions */

extern rw_reg32_t const G_TIMx_CR1[15];          /** @brief Control register 1. */
extern rw_reg32_t const G_TIMx_CR2[15];          /** @brief Control register 2. */
extern rw_reg32_t const G_TIMx_SMCR[15];         /** @brief Slave mode control register. */
extern rw_reg32_t const G_TIMx_DIER[15];         /** @brief DMA/Interrupt enable register. */
extern rw_reg32_t const G_TIMx_SR[15];           /** @brief Status register. */
extern rw_reg32_t const G_TIMx_EGR[15];          /** @brief Event generation register. */
extern rw_reg32_t const G_TIMx_CCMR1_OUTPUT[15]; /** @brief Capture/compare mode register 1 (output mode). */
extern rw_reg32_t const G_TIMx_CCMR1_INPUT[15];  /** @brief Capture/compare mode register 1 (output mode). */
extern rw_reg32_t const G_TIMx_CCMR2_OUTPUT[15]; /** @brief Capture/compare mode register 2 (output mode). */
extern rw_reg32_t const G_TIMx_CCMR2_INPUT[15];  /** @brief Capture/compare mode register 2 (output mode). */
extern rw_reg32_t const G_TIMx_CCER[15];         /** @brief Capture/compare enable register. */
extern rw_reg32_t const G_TIMx_CNT[15];          /** @brief Counter. */
extern rw_reg32_t const G_TIMx_PSC[15];          /** @brief Prescaler. */
extern rw_reg32_t const G_TIMx_ARR[15];          /** @brief Auto-reload register. */
extern rw_reg32_t const G_TIMx_CCR1[15];         /** @brief Capture/compare register 1. */
extern rw_reg32_t const G_TIMx_CCR2[15];         /** @brief Capture/compare register 2. */
extern rw_reg32_t const G_TIMx_CCR3[15];         /** @brief Capture/compare register 3. */
extern rw_reg32_t const G_TIMx_CCR4[15];         /** @brief Capture/compare register 4. */
extern rw_reg32_t const G_TIMx_DCR[15];          /** @brief DMA control register. */
extern rw_reg32_t const G_TIMx_DMAR[15];         /** @brief DMA address for full transfer. */
extern rw_reg32_t const G_TIMx_AF1[15];          /** @brief TIM alternate function option register 1. */
extern rw_reg32_t const G_TIMx_TISEL[15];        /** @brief TIM timer input selection register. */

/** @subsection G_TIMx Register Field Definitions */

extern const field32_t G_TIMx_CR1_UIFREMAP;       /** @brief UIF status bit remapping. */
extern const field32_t G_TIMx_CR1_CKD;            /** @brief Clock division. */
extern const field32_t G_TIMx_CR1_ARPE;           /** @brief Auto-reload preload enable. */
extern const field32_t G_TIMx_CR1_CMS;            /** @brief Center-aligned mode selection. */
extern const field32_t G_TIMx_CR1_DIR;            /** @brief Direction. */
extern const field32_t G_TIMx_CR1_OPM;            /** @brief One-pulse mode. */
extern const field32_t G_TIMx_CR1_URS;            /** @brief Update request source. */
extern const field32_t G_TIMx_CR1_UDIS;           /** @brief Update disable. */
extern const field32_t G_TIMx_CR1_CEN;            /** @brief Counter enable. */
extern const field32_t G_TIMx_CR2_TI1S;           /** @brief TI1 selection. */
extern const field32_t G_TIMx_CR2_MMS;            /** @brief Master mode selection. */
extern const field32_t G_TIMx_CR2_CCDS;           /** @brief Capture/compare DMA selection. */
extern const field32_t G_TIMx_SMCR_TS_4_3;        /** @brief Trigger selection. */
extern const field32_t G_TIMx_SMCR_SMS_3;         /** @brief Slave mode selection - bit 3. */
extern const field32_t G_TIMx_SMCR_ETP;           /** @brief External trigger polarity. */
extern const field32_t G_TIMx_SMCR_ECE;           /** @brief External clock enable. */
extern const field32_t G_TIMx_SMCR_ETPS;          /** @brief External trigger prescaler. */
extern const field32_t G_TIMx_SMCR_ETF;           /** @brief External trigger filter. */
extern const field32_t G_TIMx_SMCR_MSM;           /** @brief Master/Slave mode. */
extern const field32_t G_TIMx_SMCR_TS;            /** @brief Trigger selection. */
extern const field32_t G_TIMx_SMCR_SMS;           /** @brief Slave mode selection. */
extern const field32_t G_TIMx_DIER_TDE;           /** @brief Trigger DMA request enable. */
extern const field32_t G_TIMx_DIER_UDE;           /** @brief Update DMA request enable. */
extern const field32_t G_TIMx_DIER_TIE;           /** @brief Trigger interrupt enable. */
extern const field32_t G_TIMx_DIER_UIE;           /** @brief Update interrupt enable. */
extern const field32_t G_TIMx_SR_TIF;             /** @brief Trigger interrupt flag. */
extern const field32_t G_TIMx_SR_UIF;             /** @brief Update interrupt flag. */
extern const field32_t G_TIMx_EGR_TG;             /** @brief Trigger generation. */
extern const field32_t G_TIMx_EGR_UG;             /** @brief Update generation. */
extern const field32_t G_TIMx_CCMR1_INPUT_IC2PCS; /** @brief Input capture 2 prescaler. */
extern const field32_t G_TIMx_CCMR1_INPUT_ICPCS;  /** @brief Input capture 1 prescaler. */
extern const field32_t G_TIMx_CCMR2_OUTPUT_O24CE; /** @brief O24CE. */
extern const field32_t G_TIMx_CCMR2_OUTPUT_OC3CE; /** @brief OC3CE. */
extern const field32_t G_TIMx_CNT_CNT_L;          /** @brief Low counter value. */
extern const field32_t G_TIMx_CNT_CNT_H;          /** @brief High counter value. */
extern const field32_t G_TIMx_PSC_PSC;            /** @brief Prescaler value. */
extern const field32_t G_TIMx_ARR_ARR_H;          /** @brief High auto-reload value. */
extern const field32_t G_TIMx_ARR_ARR_L;          /** @brief Low auto-reload value. */
extern const field32_t G_TIMx_ARR_ARR_32B;
extern const field32_t G_TIMx_CCR1_CCR1_H;        /** @brief High capture/compare 1 value. */
extern const field32_t G_TIMx_CCR1_CCR1_L;        /** @brief Low capture/compare 1 value. */
extern const field32_t G_TIMx_CCR1_CCR1_32B;
extern const field32_t G_TIMx_CCR2_CCR2_H;        /** @brief High capture/compare 2 value. */
extern const field32_t G_TIMx_CCR2_CCR2_L;        /** @brief Low capture/compare 2 value. */
extern const field32_t G_TIMx_CCR2_CCR2_32B;
extern const field32_t G_TIMx_CCR3_CCR3_H;        /** @brief High capture/compare value. */
extern const field32_t G_TIMx_CCR3_CCR3_L;        /** @brief Low capture/compare value. */
extern const field32_t G_TIMx_CCR3_CCR3_32B;
extern const field32_t G_TIMx_CCR4_CCR4_H;        /** @brief High capture/compare value. */
extern const field32_t G_TIMx_CCR4_CCR4_L;        /** @brief Low capture/compare value. */
extern const field32_t G_TIMx_CCR4_CCR4_32B;
extern const field32_t G_TIMx_DCR_DBL;            /** @brief DMA burst length. */
extern const field32_t G_TIMx_DCR_DBA;            /** @brief DMA base address. */
extern const field32_t G_TIMx_DMAR_DMAB;          /** @brief DMA register for burst accesses. */
extern const field32_t G_TIMx_AF1_ETRSEL;         /** @brief ETR source selection. */

/** @subsection Enumerated G_TIMx Register Field Definitions */

extern const field32_t G_TIMx_DIER_CCxDE[5];          /** @brief Capture/Compare 1 DMA request enable. */
extern const field32_t G_TIMx_DIER_CCxIE[5];          /** @brief Capture/Compare 1 interrupt enable. */
extern const field32_t G_TIMx_SR_CCxOF[5];            /** @brief Capture/Compare 1 overcapture flag. */
extern const field32_t G_TIMx_SR_CCxIF[5];            /** @brief Capture/compare 1 interrupt flag. */
extern const field32_t G_TIMx_EGR_CCxG[5];            /** @brief Capture/compare 1 generation. */
extern const field32_t G_TIMx_CCMR1_OUTPUT_CCxS[3];   /** @brief CC1S. */
extern const field32_t G_TIMx_CCMR1_OUTPUT_OCxFE[3];  /** @brief OC1FE. */
extern const field32_t G_TIMx_CCMR1_OUTPUT_OCxPE[3];  /** @brief OC1PE. */
extern const field32_t G_TIMx_CCMR1_OUTPUT_OCxM[3];   /** @brief OC1M. */
extern const field32_t G_TIMx_CCMR1_OUTPUT_OCxCE[3];  /** @brief OC1CE. */
extern const field32_t G_TIMx_CCMR1_OUTPUT_OCxM_3[3]; /** @brief Output compare 1 mode - bit 3. */
extern const field32_t G_TIMx_CCMR1_INPUT_ICxF[3];    /** @brief Input capture 1 filter. */
extern const field32_t G_TIMx_CCMR2_INPUT_ICxPSC[5];  /** @brief Input capture 3 prescaler. */
extern const field32_t G_TIMx_CCMR2_OUTPUT_OCxM[5];   
extern const field32_t G_TIMx_CCMR2_OUTPUT_OCxPE[5]; 
extern const field32_t G_TIMx_CCER_CCxNP[5];          /** @brief Capture/Compare 1 output polarity. */
extern const field32_t G_TIMx_CCER_CCxP[5];           /** @brief Capture/Compare 1 output polarity. */
extern const field32_t G_TIMx_CCER_CCxE[5];           /** @brief Capture/Compare 1 output enable. */
extern const field32_t G_TIMx_TISEL_TIxSEL[5];        /** @brief TI1[0] to TI1[15] input selection. */

/**************************************************************************************************
 * @section B_TIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated B_TIMx Register Definitions */

extern rw_reg32_t const B_TIMx_CR1[8];  /** @brief Control register 1. */
extern rw_reg32_t const B_TIMx_CR2[8];  /** @brief Control register 2. */
extern rw_reg32_t const B_TIMx_DIER[8]; /** @brief DMA/Interrupt enable register. */
extern rw_reg32_t const B_TIMx_SR[8];   /** @brief Status register. */
extern rw_reg32_t const B_TIMx_EGR[8];  /** @brief Event generation register. */
extern rw_reg32_t const B_TIMx_CNT[8];  /** @brief Counter. */
extern rw_reg32_t const B_TIMx_PSC[8];  /** @brief Prescaler. */
extern rw_reg32_t const B_TIMx_ARR[8];  /** @brief Auto-reload register. */

/** @subsection B_TIMx Register Field Definitions */

extern const field32_t B_TIMx_CR1_UIFREMAP; /** @brief UIF status bit remapping. */
extern const field32_t B_TIMx_CR1_ARPE;     /** @brief Auto-reload preload enable. */
extern const field32_t B_TIMx_CR1_OPM;      /** @brief One-pulse mode. */
extern const field32_t B_TIMx_CR1_URS;      /** @brief Update request source. */
extern const field32_t B_TIMx_CR1_UDIS;     /** @brief Update disable. */
extern const field32_t B_TIMx_CR1_CEN;      /** @brief Counter enable. */
extern const field32_t B_TIMx_CR2_MMS;      /** @brief Master mode selection. */
extern const field32_t B_TIMx_DIER_UDE;     /** @brief Update DMA request enable. */
extern const field32_t B_TIMx_DIER_UIE;     /** @brief Update interrupt enable. */
extern const field32_t B_TIMx_SR_UIF;       /** @brief Update interrupt flag. */
extern const field32_t B_TIMx_EGR_UG;       /** @brief Update generation. */
extern const field32_t B_TIMx_CNT_CNT;      /** @brief Low counter value. */
extern const field32_t B_TIMx_CNT_UIFCPY;   /** @brief UIF copy. */
extern const field32_t B_TIMx_PSC_PSC;      /** @brief Prescaler value. */
extern const field32_t B_TIMx_ARR_ARR;      /** @brief Low auto-reload value. */

/**************************************************************************************************
 * @section NVIC Definitions
 **************************************************************************************************/

/** @subsection NVIC Register Definitions */

extern rw_reg32_t const NVIC_STIR; /** @brief Software trigger interrupt register. */

/** @subsection Enumerated NVIC Register Definitions */

extern rw_reg32_t const NVIC_ISERx[4]; /** @brief Interrupt set-enable register. */
extern rw_reg32_t const NVIC_ICERx[4]; /** @brief Interrupt clear-enable register. */
extern rw_reg32_t const NVIC_ISPRx[4]; /** @brief Interrupt set-pending register. */
extern rw_reg32_t const NVIC_ICPRx[4]; /** @brief Interrupt clear-pending register. */
extern ro_reg32_t const NVIC_IABRx[4]; /** @brief Interrupt active bit register. */
extern rw_reg32_t const NVIC_IPRx[39]; /** @brief Interrupt priority register. */

/** @subsection NVIC_STIR Register Field Definitions */

extern const field32_t NVIC_STIR_INTID; /** @brief Software generated interrupt ID. */

/** @subsection Enumerated NVIC Register Field Definitions */

extern const field32_t NVIC_IPRx_IPR_Nx[4]; /** @brief IPR_N0. */

/**************************************************************************************************
 * @section MPU Definitions
 **************************************************************************************************/

/** @subsection MPU Register Definitions */

extern ro_reg32_t const MPU_MPU_TYPER; /** @brief MPU type register. */
extern ro_reg32_t const MPU_MPU_CTRL;  /** @brief MPU control register. */
extern rw_reg32_t const MPU_MPU_RNR;   /** @brief MPU region number register. */
extern rw_reg32_t const MPU_MPU_RBAR;  /** @brief MPU region base address register. */
extern rw_reg32_t const MPU_MPU_RASR;  /** @brief MPU region attribute and size register. */

/** @subsection MPU Register Field Definitions */

extern const field32_t MPU_MPU_TYPER_SEPARATE;  /** @brief Separate flag. */
extern const field32_t MPU_MPU_TYPER_DREGION;   /** @brief Number of MPU data regions. */
extern const field32_t MPU_MPU_TYPER_IREGION;   /** @brief Number of MPU instruction regions. */
extern const field32_t MPU_MPU_CTRL_ENABLE;     /** @brief Enables the MPU. */
extern const field32_t MPU_MPU_CTRL_HFNMIENA;   /** @brief Enables the operation of MPU during hard fault. */
extern const field32_t MPU_MPU_CTRL_PRIVDEFENA; /** @brief Enable priviliged software access to default memory map. */
extern const field32_t MPU_MPU_RNR_REGION;      /** @brief MPU region. */
extern const field32_t MPU_MPU_RBAR_REGION;     /** @brief MPU region field. */
extern const field32_t MPU_MPU_RBAR_VALID;      /** @brief MPU region number valid. */
extern const field32_t MPU_MPU_RBAR_ADDR;       /** @brief Region base address field. */
extern const field32_t MPU_MPU_RASR_ENABLE;     /** @brief Region enable bit. */
extern const field32_t MPU_MPU_RASR_SIZE;       /** @brief Size of the MPU protection region. */
extern const field32_t MPU_MPU_RASR_SRD;        /** @brief Subregion disable bits. */
extern const field32_t MPU_MPU_RASR_B;          /** @brief Memory attribute. */
extern const field32_t MPU_MPU_RASR_C;          /** @brief Memory attribute. */
extern const field32_t MPU_MPU_RASR_S;          /** @brief Shareable memory attribute. */
extern const field32_t MPU_MPU_RASR_TEX;        /** @brief Memory attribute. */
extern const field32_t MPU_MPU_RASR_AP;         /** @brief Access permission. */
extern const field32_t MPU_MPU_RASR_XN;         /** @brief Instruction access disable bit. */

/**************************************************************************************************
 * @section STK Definitions
 **************************************************************************************************/

/** @subsection STK Register Definitions */

extern rw_reg32_t const STK_CSR;   /** @brief SysTick control and status register. */
extern rw_reg32_t const STK_RVR;   /** @brief SysTick reload value register. */
extern rw_reg32_t const STK_CVR;   /** @brief SysTick current value register. */
extern rw_reg32_t const STK_CALIB; /** @brief SysTick calibration value register. */

/** @subsection STK Register Field Definitions */

extern const field32_t STK_CSR_ENABLE;    /** @brief Counter enable. */
extern const field32_t STK_CSR_TICKINT;   /** @brief SysTick exception request enable. */
extern const field32_t STK_CSR_CLKSOURCE; /** @brief Clock source selection. */
extern const field32_t STK_CSR_COUNTFLAG; /** @brief COUNTFLAG. */
extern const field32_t STK_RVR_RELOAD;    /** @brief RELOAD value. */
extern const field32_t STK_CVR_CURRENT;   /** @brief Current counter value. */
extern const field32_t STK_CALIB_TENMS;   /** @brief Calibration value. */
extern const field32_t STK_CALIB_SKEW;    /** @brief SKEW flag: indicates whether the TENMS value is exact. */
extern const field32_t STK_CALIB_NOREF;   /** @brief NOREF flag. Reads as zero. */

/**************************************************************************************************
 * @section FPU Definitions
 **************************************************************************************************/

/** @subsection FPU Register Definitions */

extern rw_reg32_t const FPU_FPCCR; /** @brief Floating-point context control register. */
extern rw_reg32_t const FPU_FPCAR; /** @brief Floating-point context address register. */
extern rw_reg32_t const FPU_FPSCR; /** @brief Floating-point status control register. */
extern rw_reg32_t const FPU_CPACR; /** @brief Coprocessor access control register. */

/** @subsection FPU Register Field Definitions */

extern const field32_t FPU_FPCCR_LSPACT;  /** @brief LSPACT. */
extern const field32_t FPU_FPCCR_USER;    /** @brief USER. */
extern const field32_t FPU_FPCCR_THREAD;  /** @brief THREAD. */
extern const field32_t FPU_FPCCR_HFRDY;   /** @brief HFRDY. */
extern const field32_t FPU_FPCCR_MMRDY;   /** @brief MMRDY. */
extern const field32_t FPU_FPCCR_BFRDY;   /** @brief BFRDY. */
extern const field32_t FPU_FPCCR_MONRDY;  /** @brief MONRDY. */
extern const field32_t FPU_FPCCR_LSPEN;   /** @brief LSPEN. */
extern const field32_t FPU_FPCCR_ASPEN;   /** @brief ASPEN. */
extern const field32_t FPU_FPCAR_ADDRESS; /** @brief Location of unpopulated floating-point. */
extern const field32_t FPU_FPSCR_IOC;     /** @brief Invalid operation cumulative exception bit. */
extern const field32_t FPU_FPSCR_DZC;     /** @brief Division by zero cumulative exception bit. */
extern const field32_t FPU_FPSCR_OFC;     /** @brief Overflow cumulative exception bit. */
extern const field32_t FPU_FPSCR_UFC;     /** @brief Underflow cumulative exception bit. */
extern const field32_t FPU_FPSCR_IXC;     /** @brief Inexact cumulative exception bit. */
extern const field32_t FPU_FPSCR_IDC;     /** @brief Input denormal cumulative exception bit. */
extern const field32_t FPU_FPSCR_RMODE;   /** @brief Rounding mode control field. */
extern const field32_t FPU_FPSCR_FZ;      /** @brief Flush-to-zero mode control bit:. */
extern const field32_t FPU_FPSCR_DN;      /** @brief Default nan mode control bit. */
extern const field32_t FPU_FPSCR_AHP;     /** @brief Alternative half-precision control bit. */
extern const field32_t FPU_FPSCR_V;       /** @brief Overflow condition code flag. */
extern const field32_t FPU_FPSCR_C;       /** @brief Carry condition code flag. */
extern const field32_t FPU_FPSCR_Z;       /** @brief Zero condition code flag. */
extern const field32_t FPU_FPSCR_N;       /** @brief Negative condition code flag. */
extern const field32_t FPU_CPACR_CP;      /** @brief CP. */

/**************************************************************************************************
 * @section SCB Definitions
 **************************************************************************************************/

/** @subsection SCB Register Definitions */

extern ro_reg32_t const SCB_CPUID;       /** @brief CPUID base register. */
extern rw_reg32_t const SCB_ICSR;        /** @brief Interrupt control and state register. */
extern rw_reg32_t const SCB_VTOR;        /** @brief Vector table offset register. */
extern rw_reg32_t const SCB_AIRCR;       /** @brief Application interrupt and reset control register. */
extern rw_reg32_t const SCB_SCR;         /** @brief System control register. */
extern rw_reg32_t const SCB_CCR;         /** @brief Configuration and control register. */
extern rw_reg32_t const SCB_SHPR1;       /** @brief System handler priority registers. */
extern rw_reg32_t const SCB_SHPR2;       /** @brief System handler priority registers. */
extern rw_reg32_t const SCB_SHPR3;       /** @brief System handler priority registers. */
extern rw_reg32_t const SCB_SHCRS;       /** @brief System handler control and state register. */
extern rw_reg32_t const SCB_CFSR;        /** @brief Configurable fault status register. */
extern rw_reg32_t const SCB_HFSR;        /** @brief Hard fault status register. */
extern rw_reg32_t const SCB_MMFAR;       /** @brief Memory management fault address register. */
extern rw_reg32_t const SCB_BFAR;        /** @brief Bus fault address register. */
extern rw_reg32_t const SCB_ACTRL_ACTRL; /** @brief Auxiliary control register. */

/** @subsection SCB Register Field Definitions */

extern const field32_t SCB_CPUID_REVISION;             /** @brief Revision number. */
extern const field32_t SCB_CPUID_PARTNO;               /** @brief Part number of the processor. */
extern const field32_t SCB_CPUID_CONSTANT;             /** @brief Reads as 0xf. */
extern const field32_t SCB_CPUID_VARIANT;              /** @brief Variant number. */
extern const field32_t SCB_CPUID_IMPLEMENTER;          /** @brief Implementer code. */
extern const field32_t SCB_ICSR_VECTACTIVE;            /** @brief Active vector. */
extern const field32_t SCB_ICSR_RETTOBASE;             /** @brief Return to base level. */
extern const field32_t SCB_ICSR_VECTPENDING;           /** @brief Pending vector. */
extern const field32_t SCB_ICSR_ISRPENDING;            /** @brief Interrupt pending flag. */
extern const field32_t SCB_ICSR_PENDSTCLR;             /** @brief SysTick exception clear-pending bit. */
extern const field32_t SCB_ICSR_PENDSTSET;             /** @brief SysTick exception set-pending bit. */
extern const field32_t SCB_ICSR_PENDSVCLR;             /** @brief PendSV clear-pending bit. */
extern const field32_t SCB_ICSR_PENDSVSET;             /** @brief PendSV set-pending bit. */
extern const field32_t SCB_ICSR_NMIPENDSET;            /** @brief NMI set-pending bit. */
extern const field32_t SCB_VTOR_TBLOFF;                /** @brief Vector table base offset field. */
extern const field32_t SCB_AIRCR_VECTRESET;            /** @brief VECTRESET. */
extern const field32_t SCB_AIRCR_VECTCLRACTIVE;        /** @brief VECTCLRACTIVE. */
extern const field32_t SCB_AIRCR_SYSRESETREQ;          /** @brief SYSRESETREQ. */
extern const field32_t SCB_AIRCR_PRIGROUP;             /** @brief PRIGROUP. */
extern const field32_t SCB_AIRCR_ENDIANESS;            /** @brief ENDIANESS. */
extern const field32_t SCB_AIRCR_VECTKEYSTAT;          /** @brief Register key. */
extern const field32_t SCB_SCR_SLEEPONEXIT;            /** @brief SLEEPONEXIT. */
extern const field32_t SCB_SCR_SLEEPDEEP;              /** @brief SLEEPDEEP. */
extern const field32_t SCB_SCR_SEVEONPEND;             /** @brief Send event on pending bit. */
extern const field32_t SCB_CCR_NONBASETHRDENA;         /** @brief Configures how the processor enters thread mode. */
extern const field32_t SCB_CCR_USERSETMPEND;           /** @brief USERSETMPEND. */
extern const field32_t SCB_CCR_UNALIGN__TRP;           /** @brief UNALIGN_ TRP. */
extern const field32_t SCB_CCR_DIV_0_TRP;              /** @brief DIV_0_TRP. */
extern const field32_t SCB_CCR_BFHFNMIGN;              /** @brief BFHFNMIGN. */
extern const field32_t SCB_CCR_STKALIGN;               /** @brief STKALIGN. */
extern const field32_t SCB_CCR_DC;                     /** @brief DC. */
extern const field32_t SCB_CCR_IC;                     /** @brief IC. */
extern const field32_t SCB_CCR_BP;                     /** @brief BP. */
extern const field32_t SCB_SHPR2_PRI_11;               /** @brief Priority of system handler 11. */
extern const field32_t SCB_SHCRS_MEMFAULTACT;          /** @brief Memory management fault exception active bit. */
extern const field32_t SCB_SHCRS_BUSFAULTACT;          /** @brief Bus fault exception active bit. */
extern const field32_t SCB_SHCRS_USGFAULTACT;          /** @brief Usage fault exception active bit. */
extern const field32_t SCB_SHCRS_SVCALLACT;            /** @brief SVC call active bit. */
extern const field32_t SCB_SHCRS_MONITORACT;           /** @brief Debug monitor active bit. */
extern const field32_t SCB_SHCRS_PENDSVACT;            /** @brief PendSV exception active bit. */
extern const field32_t SCB_SHCRS_SYSTICKACT;           /** @brief SysTick exception active bit. */
extern const field32_t SCB_SHCRS_USGFAULTPENDED;       /** @brief Usage fault exception pending bit. */
extern const field32_t SCB_SHCRS_MEMFAULTPENDED;       /** @brief Memory management fault exception pending bit. */
extern const field32_t SCB_SHCRS_BUSFAULTPENDED;       /** @brief Bus fault exception pending bit. */
extern const field32_t SCB_SHCRS_SVCALLPENDED;         /** @brief SVC call pending bit. */
extern const field32_t SCB_SHCRS_MEMFAULTENA;          /** @brief Memory management fault enable bit. */
extern const field32_t SCB_SHCRS_BUSFAULTENA;          /** @brief Bus fault enable bit. */
extern const field32_t SCB_SHCRS_USGFAULTENA;          /** @brief Usage fault enable bit. */
extern const field32_t SCB_CFSR_IACCVIOL;              /** @brief IACCVIOL. */
extern const field32_t SCB_CFSR_DACCVIOL;              /** @brief DACCVIOL. */
extern const field32_t SCB_CFSR_MUNSTKERR;             /** @brief MUNSTKERR. */
extern const field32_t SCB_CFSR_MSTKERR;               /** @brief MSTKERR. */
extern const field32_t SCB_CFSR_MLSPERR;               /** @brief MLSPERR. */
extern const field32_t SCB_CFSR_MMARVALID;             /** @brief MMARVALID. */
extern const field32_t SCB_CFSR_IBUSERR;               /** @brief Instruction bus error. */
extern const field32_t SCB_CFSR_PRECISERR;             /** @brief Precise data bus error. */
extern const field32_t SCB_CFSR_IMPRECISERR;           /** @brief Imprecise data bus error. */
extern const field32_t SCB_CFSR_UNSTKERR;              /** @brief Bus fault on unstacking for a return from exception. */
extern const field32_t SCB_CFSR_STKERR;                /** @brief Bus fault on stacking for exception entry. */
extern const field32_t SCB_CFSR_LSPERR;                /** @brief Bus fault on floating-point lazy state preservation. */
extern const field32_t SCB_CFSR_BFARVALID;             /** @brief Bus fault address register (BFAR) valid flag. */
extern const field32_t SCB_CFSR_UNDEFINSTR;            /** @brief Undefined instruction usage fault. */
extern const field32_t SCB_CFSR_INVSTATE;              /** @brief Invalid state usage fault. */
extern const field32_t SCB_CFSR_INVPC;                 /** @brief Invalid PC load usage fault. */
extern const field32_t SCB_CFSR_NOCP;                  /** @brief No coprocessor usage fault. */
extern const field32_t SCB_CFSR_UNALIGNED;             /** @brief Unaligned access usage fault. */
extern const field32_t SCB_CFSR_DIVBYZERO;             /** @brief Divide by zero usage fault. */
extern const field32_t SCB_HFSR_VECTTBL;               /** @brief Vector table hard fault. */
extern const field32_t SCB_HFSR_FORCED;                /** @brief Forced hard fault. */
extern const field32_t SCB_HFSR_DEBUG_VT;              /** @brief Reserved for debug use. */
extern const field32_t SCB_ACTRL_ACTRL_DISFOLD;        /** @brief DISFOLD. */
extern const field32_t SCB_ACTRL_ACTRL_FPEXCODIS;      /** @brief FPEXCODIS. */
extern const field32_t SCB_ACTRL_ACTRL_DISRAMODE;      /** @brief DISRAMODE. */
extern const field32_t SCB_ACTRL_ACTRL_DISITMATBFLUSH; /** @brief DISITMATBFLUSH. */

/** @subsection Enumerated SCB Register Field Definitions */

extern const field32_t SCB_SHPR1_PRI_x[7];  /** @brief Priority of system handler 4. */
extern const field32_t SCB_SHPR3_PRI_1x[6]; /** @brief Priority of system handler 14. */

/**************************************************************************************************
 * @section Debug Definitions
 **************************************************************************************************/

/** @subsection Debug Register Definitions */

extern rw_reg32_t const DBG_DHCSR; /** @brief Debug halting control and status register. */
extern rw_reg32_t const DBG_DCRSR; /** @brief Debug core register selector register. */
extern rw_reg32_t const DBG_DCRDR; /** @brief Debug core register data register. */
extern rw_reg32_t const DBG_DEMCR; /** @brief Debug exception and monitor control register. */

/** @subsection Debug Register Field Definitions */

extern const field32_t DBG_DHCSR_C_DEBUGEN;    /** @brief Debug enable. */
extern const field32_t DBG_DHCSR_C_HALT;       /** @brief Halt the core. */
extern const field32_t DBG_DHCSR_C_STEP;       /** @brief Step the core. */
extern const field32_t DBG_DHCSR_C_MASKINTS;   /** @brief Mask interrupts when stepping. */
extern const field32_t DBG_DHCSR_C_SNAPSTALL;  /** @brief Allow imprecise entry to debug state. */
extern const field32_t DBG_DHCSR_S_REGRDY;     /** @brief Register read/write operation is complete. */
extern const field32_t DBG_DHCSR_S_HALT;       /** @brief Core is in debug state. */
extern const field32_t DBG_DHCSR_S_SLEEP;      /** @brief Core is sleeping. */
extern const field32_t DBG_DHCSR_S_LOCKUP;     /** @brief Core is in lockup state. */
extern const field32_t DBG_DHCSR_S_RETIRE_ST;  /** @brief Indicates whether the processor has completed execution of an instruction since the last read. */
extern const field32_t DBG_DHCSR_S_RESET_ST;   /** @brief Indicates the processor has been reset since the last read. */
extern const field32_t DBG_DHCSR_DBGKEY;       /** @brief Debug key. */
extern const field32_t DBG_DCRSR_REGSEL;       /** @brief Register selector. */
extern const field32_t DBG_DCRSR_REGWnR;       /** @brief Register write/not-read. */
extern const field32_t DBG_DCRDR_DBGTMP;       /** @brief Debug temporary data for register read/write. */
extern const field32_t DBG_DEMCR_VC_CORERESET; /** @brief Reset vector catch. */
extern const field32_t DBG_DEMCR_VC_MMERR;     /** @brief Memory management error vector catch. */
extern const field32_t DBG_DEMCR_VC_NOCPERR;   /** @brief Coprocessor error vector catch. */
extern const field32_t DBG_DEMCR_VC_CHKERR;    /** @brief Check error vector catch. */
extern const field32_t DBG_DEMCR_VC_STATERR;   /** @brief State error vector catch. */
extern const field32_t DBG_DEMCR_VC_BUSERR;    /** @brief Bus error vector catch. */
extern const field32_t DBG_DEMCR_VC_INTERR;    /** @brief Interrupt error vector catch. */
extern const field32_t DBG_DEMCR_VC_HARDERR;   /** @brief Hard error vector catch. */
extern const field32_t DBG_DEMCR_MON_EN;       /** @brief Monitor enable. */
extern const field32_t DBG_DEMCR_MON_PEND;     /** @brief Monitor pend. */
extern const field32_t DBG_DEMCR_MON_STEP;     /** @brief Monitor step. */
extern const field32_t DBG_DEMCR_MON_REQ;      /** @brief Monitor request. */
extern const field32_t DBG_DEMCR_TRCENA;       /** @brief Trace enable. */

/**************************************************************************************************
 * @section PF Definitions
 **************************************************************************************************/

/** @subsection PF Register Definitions */

extern ro_reg32_t const PF_CLIDR;  /** @brief Cache level ID register. */
extern ro_reg32_t const PF_CTR;    /** @brief Cache type register. */
extern ro_reg32_t const PF_CCSIDR; /** @brief Cache size ID register. */

/** @subsection PF Register Field Definitions */

extern const field32_t PF_CLIDR_LOUIS;          /** @brief LoUIS. */
extern const field32_t PF_CLIDR_LOC;            /** @brief LoC. */
extern const field32_t PF_CLIDR_LOU;            /** @brief LoU. */
extern const field32_t PF_CTR_IMINLINE;         /** @brief IminLine. */
extern const field32_t PF_CTR_DMINLINE;         /** @brief DMinLine. */
extern const field32_t PF_CTR_ERG;              /** @brief ERG. */
extern const field32_t PF_CTR_CWG;              /** @brief CWG. */
extern const field32_t PF_CTR_FORMAT;           /** @brief Format. */
extern const field32_t PF_CCSIDR_LINESIZE;      /** @brief LineSize. */
extern const field32_t PF_CCSIDR_ASSOCIATIVITY; /** @brief Associativity. */
extern const field32_t PF_CCSIDR_NUMSETS;       /** @brief NumSets. */
extern const field32_t PF_CCSIDR_WA;            /** @brief WA. */
extern const field32_t PF_CCSIDR_RA;            /** @brief RA. */
extern const field32_t PF_CCSIDR_WB;            /** @brief WB. */
extern const field32_t PF_CCSIDR_WT;            /** @brief WT. */

/** @subsection Enumerated PF Register Field Definitions */

extern const field32_t PF_CLIDR_CLx[8]; /** @brief CL1. */

/**************************************************************************************************
 * @section AC Definitions
 **************************************************************************************************/

/** @subsection AC Register Definitions */

extern rw_reg32_t const AC_ITCMCR; /** @brief Instruction and data tightly-coupled memory control registers. */
extern rw_reg32_t const AC_DTCMCR; /** @brief Instruction and data tightly-coupled memory control registers. */
extern rw_reg32_t const AC_AHBPCR; /** @brief AHBP control register. */
extern rw_reg32_t const AC_CACR;   /** @brief Auxiliary cache control register. */
extern rw_reg32_t const AC_AHBSCR; /** @brief AHB slave control register. */
extern rw_reg32_t const AC_ABFSR;  /** @brief Auxiliary bus fault status register. */

/** @subsection AC Register Field Definitions */

extern const field32_t AC_ITCMCR_EN;        /** @brief EN. */
extern const field32_t AC_ITCMCR_RMW;       /** @brief RMW. */
extern const field32_t AC_ITCMCR_RETEN;     /** @brief RETEN. */
extern const field32_t AC_ITCMCR_SZ;        /** @brief SZ. */
extern const field32_t AC_DTCMCR_EN;        /** @brief EN. */
extern const field32_t AC_DTCMCR_RMW;       /** @brief RMW. */
extern const field32_t AC_DTCMCR_RETEN;     /** @brief RETEN. */
extern const field32_t AC_DTCMCR_SZ;        /** @brief SZ. */
extern const field32_t AC_AHBPCR_EN;        /** @brief EN. */
extern const field32_t AC_AHBPCR_SZ;        /** @brief SZ. */
extern const field32_t AC_CACR_SIWT;        /** @brief SIWT. */
extern const field32_t AC_CACR_ECCEN;       /** @brief ECCEN. */
extern const field32_t AC_CACR_FORCEWT;     /** @brief FORCEWT. */
extern const field32_t AC_AHBSCR_CTL;       /** @brief CTL. */
extern const field32_t AC_AHBSCR_TPRI;      /** @brief TPRI. */
extern const field32_t AC_AHBSCR_INITCOUNT; /** @brief INITCOUNT. */
extern const field32_t AC_ABFSR_ITCM;       /** @brief ITCM. */
extern const field32_t AC_ABFSR_DTCM;       /** @brief DTCM. */
extern const field32_t AC_ABFSR_AHBP;       /** @brief AHBP. */
extern const field32_t AC_ABFSR_AXIM;       /** @brief AXIM. */
extern const field32_t AC_ABFSR_EPPB;       /** @brief EPPB. */
extern const field32_t AC_ABFSR_AXIMTYPE;   /** @brief AXIMTYPE. */

/**************************************************************************************************
 * @section RAMECCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated RAMECCx Register Definitions */

extern rw_reg32_t const RAMECCx_IER[4];    /** @brief RAMECC interrupt enable register. */
extern rw_reg32_t const RAMECCx_M1CR[4];   /** @brief RAMECC monitor x configuration register. */
extern rw_reg32_t const RAMECCx_M1SR[4];   /** @brief RAMECC monitor x status register. */
extern rw_reg32_t const RAMECCx_M1FAR[4];  /** @brief RAMECC monitor x failing address register. */
extern rw_reg32_t const RAMECCx_M1FDRL[4]; /** @brief RAMECC monitor x failing data low register. */
extern rw_reg32_t const RAMECCx_M1FDRH[4]; /** @brief RAMECC monitor x failing data high register. */
extern rw_reg32_t const RAMECCx_M1FECR[4]; /** @brief RAMECC monitor x failing ECC error code register. */
extern rw_reg32_t const RAMECCx_M2CR[4];   /** @brief RAMECC monitor x configuration register. */
extern rw_reg32_t const RAMECCx_M2SR[4];   /** @brief RAMECC monitor x status register. */
extern rw_reg32_t const RAMECCx_M2FAR[4];  /** @brief RAMECC monitor x failing address register. */
extern rw_reg32_t const RAMECCx_M2FDRL[4]; /** @brief RAMECC monitor x failing data low register. */
extern ro_reg32_t const RAMECCx_M2FDRH[4]; /** @brief RAMECC monitor x failing data high register. */
extern ro_reg32_t const RAMECCx_M2FECR[4]; /** @brief RAMECC monitor x failing ECC error code register. */
extern ro_reg32_t const RAMECCx_M3CR[4];   /** @brief RAMECC monitor x configuration register. */
extern ro_reg32_t const RAMECCx_M3SR[4];   /** @brief RAMECC monitor x status register. */
extern rw_reg32_t const RAMECCx_M3FAR[4];  /** @brief RAMECC monitor x failing address register. */
extern ro_reg32_t const RAMECCx_M3FDRL[4]; /** @brief RAMECC monitor x failing data low register. */
extern ro_reg32_t const RAMECCx_M3FDRH[4]; /** @brief RAMECC monitor x failing data high register. */
extern ro_reg32_t const RAMECCx_M3FECR[4]; /** @brief RAMECC monitor x failing ECC error code register. */
extern ro_reg32_t const RAMECCx_M4CR[4];   /** @brief RAMECC monitor x configuration register. */
extern ro_reg32_t const RAMECCx_M4SR[4];   /** @brief RAMECC monitor x status register. */
extern ro_reg32_t const RAMECCx_M4FAR[4];  /** @brief RAMECC monitor x failing address register. */
extern rw_reg32_t const RAMECCx_M4FDRL[4]; /** @brief RAMECC monitor x failing data low register. */
extern ro_reg32_t const RAMECCx_M4FDRH[4]; /** @brief RAMECC monitor x failing data high register. */
extern ro_reg32_t const RAMECCx_M4FECR[4]; /** @brief RAMECC monitor x failing data high register. */
extern ro_reg32_t const RAMECCx_M5CR[4];   /** @brief RAMECC monitor x configuration register. */
extern rw_reg32_t const RAMECCx_M5SR[4];   /** @brief RAMECC monitor x status register. */
extern rw_reg32_t const RAMECCx_M5FAR[4];  /** @brief RAMECC monitor x failing address register. */
extern ro_reg32_t const RAMECCx_M5FDRL[4]; /** @brief RAMECC monitor x failing data low register. */
extern ro_reg32_t const RAMECCx_M5FDRH[4]; /** @brief RAMECC monitor x failing data high register. */
extern ro_reg32_t const RAMECCx_M5FECR[4]; /** @brief RAMECC monitor x failing ECC error code register. */

/** @subsection RAMECCx Register Field Definitions */

extern const field32_t RAMECCx_IER_GIE;          /** @brief Global interrupt enable. */
extern const field32_t RAMECCx_IER_GECCSEIE_;    /** @brief Global ECC single error interrupt enable. */
extern const field32_t RAMECCx_IER_GECCDEIE;     /** @brief Global ECC double error interrupt enable. */
extern const field32_t RAMECCx_IER_GECCDEBWIE;   /** @brief Global ECC double error on byte write (BW) interrupt enable. */
extern const field32_t RAMECCx_M1CR_ECCSEIE;     /** @brief ECC single error interrupt enable. */
extern const field32_t RAMECCx_M1CR_ECCDEIE;     /** @brief ECC double error interrupt enable. */
extern const field32_t RAMECCx_M1CR_ECCDEBWIE;   /** @brief ECC double error on byte write (BW) interrupt enable. */
extern const field32_t RAMECCx_M1CR_ECCELEN;     /** @brief ECC error latching enable. */
extern const field32_t RAMECCx_M1SR_ECCSEIE;     /** @brief ECC single error interrupt enable. */
extern const field32_t RAMECCx_M1SR_ECCDEIE;     /** @brief ECC double error interrupt enable. */
extern const field32_t RAMECCx_M1SR_ECCDEBWIE;   /** @brief ECC double error on byte write (BW) interrupt enable. */
extern const field32_t RAMECCx_M1SR_ECCELEN;     /** @brief ECC error latching enable. */
extern const field32_t RAMECCx_M1FAR_ECCSEIE;    /** @brief ECC single error interrupt enable. */
extern const field32_t RAMECCx_M1FAR_ECCDEIE;    /** @brief ECC double error interrupt enable. */
extern const field32_t RAMECCx_M1FAR_ECCDEBWIE;  /** @brief ECC double error on byte write (BW) interrupt enable. */
extern const field32_t RAMECCx_M1FAR_ECCELEN;    /** @brief ECC error latching enable. */
extern const field32_t RAMECCx_M1FDRL_ECCSEIE;   /** @brief ECC single error interrupt enable. */
extern const field32_t RAMECCx_M1FDRL_ECCDEIE;   /** @brief ECC double error interrupt enable. */
extern const field32_t RAMECCx_M1FDRL_ECCDEBWIE; /** @brief ECC double error on byte write (BW) interrupt enable. */
extern const field32_t RAMECCx_M1FDRL_ECCELEN;   /** @brief ECC error latching enable. */
extern const field32_t RAMECCx_M1FDRH_ECCSEIE;   /** @brief ECC single error interrupt enable. */
extern const field32_t RAMECCx_M1FDRH_ECCDEIE;   /** @brief ECC double error interrupt enable. */
extern const field32_t RAMECCx_M1FDRH_ECCDEBWIE; /** @brief ECC double error on byte write (BW) interrupt enable. */
extern const field32_t RAMECCx_M1FDRH_ECCELEN;   /** @brief ECC error latching enable. */
extern const field32_t RAMECCx_M1FECR_SEDCF;     /** @brief ECC single error detected and corrected flag. */
extern const field32_t RAMECCx_M1FECR_DEDF;      /** @brief ECC double error detected flag. */
extern const field32_t RAMECCx_M1FECR_DEBWDF;    /** @brief ECC double error on byte write (BW) detected flag. */
extern const field32_t RAMECCx_M2CR_SEDCF;       /** @brief ECC single error detected and corrected flag. */
extern const field32_t RAMECCx_M2CR_DEDF;        /** @brief ECC double error detected flag. */
extern const field32_t RAMECCx_M2CR_DEBWDF;      /** @brief ECC double error on byte write (BW) detected flag. */
extern const field32_t RAMECCx_M2SR_SEDCF;       /** @brief ECC single error detected and corrected flag. */
extern const field32_t RAMECCx_M2SR_DEDF;        /** @brief ECC double error detected flag. */
extern const field32_t RAMECCx_M2SR_DEBWDF;      /** @brief ECC double error on byte write (BW) detected flag. */
extern const field32_t RAMECCx_M2FAR_SEDCF;      /** @brief ECC single error detected and corrected flag. */
extern const field32_t RAMECCx_M2FAR_DEDF;       /** @brief ECC double error detected flag. */
extern const field32_t RAMECCx_M2FAR_DEBWDF;     /** @brief ECC double error on byte write (BW) detected flag. */
extern const field32_t RAMECCx_M2FDRL_SEDCF;     /** @brief ECC single error detected and corrected flag. */
extern const field32_t RAMECCx_M2FDRL_DEDF;      /** @brief ECC double error detected flag. */
extern const field32_t RAMECCx_M2FDRL_DEBWDF;    /** @brief ECC double error on byte write (BW) detected flag. */

/**************************************************************************************************
 * @section ART Definitions
 **************************************************************************************************/

/** @subsection ART Register Definitions */

extern rw_reg32_t const ART_CTR; /** @brief Control register. */

/** @subsection ART Register Field Definitions */

extern const field32_t ART_CTR_EN;         /** @brief Cache enable. */
extern const field32_t ART_CTR_PCACHEADDR; /** @brief Cacheable page index. */

/**************************************************************************************************
 * @section Si4468 Radio Definitions
 **************************************************************************************************/

/** @subsection Si4468 Command Opcodes */

#define SI446X_CMD_POWER_UP          0x02
#define SI446X_CMD_GET_INT_STATUS    0x20
#define SI446X_CMD_GPIO_PIN_CFG      0x13
#define SI446X_CMD_SET_PROPERTY      0x11
#define SI446X_CMD_FIFO_INFO         0x15
#define SI446X_CMD_PACKET_INFO       0x16
#define SI446X_CMD_START_TX          0x31
#define SI446X_CMD_START_RX          0x32
#define SI446X_CMD_CHANGE_STATE      0x34
#define SI446X_CMD_READ_CMD_BUFFER   0x44
#define SI446X_CMD_READ_FRR_A        0x50
#define SI446X_CMD_READ_FRR_B        0x51
#define SI446X_CMD_READ_FRR_C        0x52
#define SI446X_CMD_READ_FRR_D        0x53
#define SI446X_CMD_WRITE_TX_FIFO     0x66
#define SI446X_CMD_READ_RX_FIFO      0x77

/** @subsection Si4468 Misc Constants */

#define SI446X_CTS_READY_VALUE       0xFF
#define SI446X_CMD_BUFFER_SIZE       16U
#define SI446X_FIFO_SIZE_BYTES       64U
#define SI446X_MAX_PACKET_SIZE       64U

/** @subsection Si4468 State IDs */

#define SI446X_STATE_SLEEP           0x01
#define SI446X_STATE_READY           0x03

/** @subsection Si4468 Property Group IDs */

#define SI446X_PROP_GROUP_GLOBAL     0x00
#define SI446X_PROP_GROUP_INT_CTL    0x01
#define SI446X_PROP_GROUP_FRR_CTL    0x02
#define SI446X_PROP_GROUP_PREAMBLE   0x10
#define SI446X_PROP_GROUP_SYNC       0x11
#define SI446X_PROP_GROUP_PKT        0x12
#define SI446X_PROP_GROUP_MODEM      0x20
#define SI446X_PROP_GROUP_CHFLT      0x21
#define SI446X_PROP_GROUP_PA         0x22
#define SI446X_PROP_GROUP_SYNTH      0x23

/** @subsection Si4468 Global Properties */

#define SI446X_PROP_GLOBAL_XO_TUNE   0x0000
#define SI446X_PROP_GLOBAL_CLK_CFG   0x0001
#define SI446X_PROP_GLOBAL_CONFIG    0x0003

/** @subsection Si4468 Interrupt Control Properties */

#define SI446X_PROP_INT_CTL_ENABLE   0x0100
#define SI446X_PROP_INT_CTL_PH_ENABLE 0x0101
#define SI446X_PROP_INT_CTL_MODEM_ENABLE 0x0102
#define SI446X_PROP_INT_CTL_CHIP_ENABLE 0x0103

/** @subsection Si4468 FRR Control Properties */

#define SI446X_PROP_FRR_CTL_A_MODE   0x0200
#define SI446X_PROP_FRR_CTL_B_MODE   0x0201
#define SI446X_PROP_FRR_CTL_C_MODE   0x0202
#define SI446X_PROP_FRR_CTL_D_MODE   0x0203

/** @subsection Si4468 Preamble Properties */

#define SI446X_PROP_PREAMBLE_TX_LENGTH      0x1000
#define SI446X_PROP_PREAMBLE_CONFIG_STD_1   0x1001
#define SI446X_PROP_PREAMBLE_CONFIG_NSTD    0x1002
#define SI446X_PROP_PREAMBLE_CONFIG_STD_2   0x1003
#define SI446X_PROP_PREAMBLE_CONFIG         0x1004
#define SI446X_PROP_PREAMBLE_PATTERN_31_24  0x1005
#define SI446X_PROP_PREAMBLE_PATTERN_23_16  0x1006
#define SI446X_PROP_PREAMBLE_PATTERN_15_8   0x1007
#define SI446X_PROP_PREAMBLE_PATTERN_7_0    0x1008

/** @subsection Si4468 Sync Properties */

#define SI446X_PROP_SYNC_CONFIG       0x1100
#define SI446X_PROP_SYNC_BITS_31_24   0x1101
#define SI446X_PROP_SYNC_BITS_23_16   0x1102
#define SI446X_PROP_SYNC_BITS_15_8    0x1103
#define SI446X_PROP_SYNC_BITS_7_0     0x1104
#define SI446X_PROP_SYNC_CONFIG2      0x1105
#define SI446X_PROP_SYNC_BITS2_31_24  0x1106
#define SI446X_PROP_SYNC_BITS2_23_16  0x1107
#define SI446X_PROP_SYNC_BITS2_15_8   0x1108
#define SI446X_PROP_SYNC_BITS2_7_0    0x1109

/** @subsection Si4468 Packet Handler Properties */

#define SI446X_PROP_PKT_CRC_CONFIG        0x1200
#define SI446X_PROP_PKT_WHT_POLY_15_8     0x1201
#define SI446X_PROP_PKT_WHT_POLY_7_0      0x1202
#define SI446X_PROP_PKT_WHT_SEED_15_8     0x1203
#define SI446X_PROP_PKT_WHT_SEED_7_0      0x1204
#define SI446X_PROP_PKT_WHT_BIT_NUM       0x1205
#define SI446X_PROP_PKT_CONFIG1           0x1206
#define SI446X_PROP_PKT_CONFIG2           0x1207
#define SI446X_PROP_PKT_LEN               0x1208
#define SI446X_PROP_PKT_LEN_FIELD_SOURCE  0x1209
#define SI446X_PROP_PKT_LEN_ADJUST        0x120A
#define SI446X_PROP_PKT_TX_THRESHOLD      0x120B
#define SI446X_PROP_PKT_RX_THRESHOLD      0x120C
#define SI446X_PROP_PKT_FIELD_1_LENGTH_12_8 0x120D
#define SI446X_PROP_PKT_FIELD_1_LENGTH_7_0  0x120E
#define SI446X_PROP_PKT_FIELD_1_CONFIG      0x120F
#define SI446X_PROP_PKT_FIELD_1_CRC_CONFIG  0x1210
#define SI446X_PROP_PKT_FIELD_2_LENGTH_12_8 0x1211
#define SI446X_PROP_PKT_FIELD_2_LENGTH_7_0  0x1212
#define SI446X_PROP_PKT_FIELD_2_CONFIG      0x1213
#define SI446X_PROP_PKT_FIELD_2_CRC_CONFIG  0x1214
#define SI446X_PROP_PKT_FIELD_3_LENGTH_12_8 0x1215
#define SI446X_PROP_PKT_FIELD_3_LENGTH_7_0  0x1216
#define SI446X_PROP_PKT_FIELD_3_CONFIG      0x1217
#define SI446X_PROP_PKT_FIELD_3_CRC_CONFIG  0x1218
#define SI446X_PROP_PKT_FIELD_4_LENGTH_12_8 0x1219
#define SI446X_PROP_PKT_FIELD_4_LENGTH_7_0  0x121A
#define SI446X_PROP_PKT_FIELD_4_CONFIG      0x121B
#define SI446X_PROP_PKT_FIELD_4_CRC_CONFIG  0x121C
#define SI446X_PROP_PKT_FIELD_5_LENGTH_12_8 0x121D
#define SI446X_PROP_PKT_FIELD_5_LENGTH_7_0  0x121E
#define SI446X_PROP_PKT_FIELD_5_CONFIG      0x121F
#define SI446X_PROP_PKT_FIELD_5_CRC_CONFIG  0x1220
#define SI446X_PROP_PKT_RX_FIELD_1_LENGTH_12_8 0x1221
#define SI446X_PROP_PKT_RX_FIELD_1_LENGTH_7_0  0x1222
#define SI446X_PROP_PKT_RX_FIELD_1_CONFIG      0x1223
#define SI446X_PROP_PKT_RX_FIELD_1_CRC_CONFIG  0x1224
#define SI446X_PROP_PKT_RX_FIELD_2_LENGTH_12_8 0x1225
#define SI446X_PROP_PKT_RX_FIELD_2_LENGTH_7_0  0x1226
#define SI446X_PROP_PKT_RX_FIELD_2_CONFIG      0x1227
#define SI446X_PROP_PKT_RX_FIELD_2_CRC_CONFIG  0x1228
#define SI446X_PROP_PKT_RX_FIELD_3_LENGTH_12_8 0x1229
#define SI446X_PROP_PKT_RX_FIELD_3_LENGTH_7_0  0x122A
#define SI446X_PROP_PKT_RX_FIELD_3_CONFIG      0x122B
#define SI446X_PROP_PKT_RX_FIELD_3_CRC_CONFIG  0x122C
#define SI446X_PROP_PKT_RX_FIELD_4_LENGTH_12_8 0x122D
#define SI446X_PROP_PKT_RX_FIELD_4_LENGTH_7_0  0x122E
#define SI446X_PROP_PKT_RX_FIELD_4_CONFIG      0x122F
#define SI446X_PROP_PKT_RX_FIELD_4_CRC_CONFIG  0x1230
#define SI446X_PROP_PKT_RX_FIELD_5_LENGTH_12_8 0x1231
#define SI446X_PROP_PKT_RX_FIELD_5_LENGTH_7_0  0x1232
#define SI446X_PROP_PKT_RX_FIELD_5_CONFIG      0x1233
#define SI446X_PROP_PKT_RX_FIELD_5_CRC_CONFIG  0x1234
#define SI446X_PROP_PKT_CRC_SEED_31_24          0x1236
#define SI446X_PROP_PKT_CRC_SEED_23_16          0x1237
#define SI446X_PROP_PKT_CRC_SEED_15_8           0x1238
#define SI446X_PROP_PKT_CRC_SEED_7_0            0x1239

/** @subsection Si4468 Modem Properties */

#define SI446X_PROP_MODEM_MOD_TYPE       0x2000
#define SI446X_PROP_MODEM_MAP_CONTROL    0x2001
#define SI446X_PROP_MODEM_DSM_CTRL       0x2002
#define SI446X_PROP_MODEM_DATA_RATE_2    0x2003
#define SI446X_PROP_MODEM_DATA_RATE_1    0x2004
#define SI446X_PROP_MODEM_DATA_RATE_0    0x2005
#define SI446X_PROP_MODEM_TX_NCO_MODE_3  0x2006
#define SI446X_PROP_MODEM_TX_NCO_MODE_2  0x2007
#define SI446X_PROP_MODEM_TX_NCO_MODE_1  0x2008
#define SI446X_PROP_MODEM_TX_NCO_MODE_0  0x2009
#define SI446X_PROP_MODEM_FREQ_DEV_2     0x200A
#define SI446X_PROP_MODEM_FREQ_DEV_1     0x200B
#define SI446X_PROP_MODEM_FREQ_DEV_0     0x200C
#define SI446X_PROP_MODEM_TX_RAMP_DELAY  0x2018
#define SI446X_PROP_MODEM_MDM_CTRL       0x2019
#define SI446X_PROP_MODEM_IF_CONTROL     0x201A
#define SI446X_PROP_MODEM_IF_FREQ_2      0x201B
#define SI446X_PROP_MODEM_IF_FREQ_1      0x201C
#define SI446X_PROP_MODEM_IF_FREQ_0      0x201D
#define SI446X_PROP_MODEM_DECIMATION_CFG1 0x201E
#define SI446X_PROP_MODEM_DECIMATION_CFG0 0x201F
#define SI446X_PROP_MODEM_DECIMATION_CFG2 0x2020
#define SI446X_PROP_MODEM_IFPKD_THRESHOLDS 0x2021
#define SI446X_PROP_MODEM_BCR_OSR_1      0x2022
#define SI446X_PROP_MODEM_BCR_OSR_0      0x2023
#define SI446X_PROP_MODEM_BCR_NCO_OFFSET_2 0x2024
#define SI446X_PROP_MODEM_BCR_NCO_OFFSET_1 0x2025
#define SI446X_PROP_MODEM_BCR_NCO_OFFSET_0 0x2026
#define SI446X_PROP_MODEM_BCR_GAIN_1     0x2027
#define SI446X_PROP_MODEM_BCR_GAIN_0     0x2028
#define SI446X_PROP_MODEM_BCR_GEAR       0x2029
#define SI446X_PROP_MODEM_BCR_MISC1      0x202A
#define SI446X_PROP_MODEM_BCR_MISC0      0x202B
#define SI446X_PROP_MODEM_AFC_GEAR       0x202C
#define SI446X_PROP_MODEM_AFC_WAIT       0x202D
#define SI446X_PROP_MODEM_AFC_GAIN_1     0x202E
#define SI446X_PROP_MODEM_AFC_GAIN_0     0x202F
#define SI446X_PROP_MODEM_AFC_LIMITER_1  0x2030
#define SI446X_PROP_MODEM_AFC_LIMITER_0  0x2031
#define SI446X_PROP_MODEM_AFC_MISC       0x2032
#define SI446X_PROP_MODEM_AGC_CONTROL    0x2035
#define SI446X_PROP_MODEM_AGC_WINDOW_SIZE 0x2038
#define SI446X_PROP_MODEM_AGC_RFPD_DECAY 0x2039
#define SI446X_PROP_MODEM_AGC_IFPD_DECAY 0x203A
#define SI446X_PROP_MODEM_FSK4_GAIN1     0x203B
#define SI446X_PROP_MODEM_FSK4_GAIN0     0x203C
#define SI446X_PROP_MODEM_FSK4_TH1       0x203D
#define SI446X_PROP_MODEM_FSK4_TH0       0x203E
#define SI446X_PROP_MODEM_FSK4_MAP       0x203F
#define SI446X_PROP_MODEM_OOK_PDTC       0x2040
#define SI446X_PROP_MODEM_OOK_BLOPK      0x2041
#define SI446X_PROP_MODEM_OOK_CNT1       0x2042
#define SI446X_PROP_MODEM_OOK_MISC       0x2043
#define SI446X_PROP_MODEM_RAW_CONTROL    0x2045
#define SI446X_PROP_MODEM_RAW_EYE_1      0x2046
#define SI446X_PROP_MODEM_RAW_EYE_0      0x2047
#define SI446X_PROP_MODEM_ANT_DIV_MODE   0x2048
#define SI446X_PROP_MODEM_ANT_DIV_CONTROL 0x2049
#define SI446X_PROP_MODEM_RSSI_THRESH    0x204A
#define SI446X_PROP_MODEM_RSSI_JUMP_THRESH 0x204B
#define SI446X_PROP_MODEM_RSSI_CONTROL   0x204C
#define SI446X_PROP_MODEM_RSSI_CONTROL2  0x204D
#define SI446X_PROP_MODEM_RSSI_COMP      0x204E
#define SI446X_PROP_MODEM_RAW_SEARCH2    0x2050
#define SI446X_PROP_MODEM_CLKGEN_BAND    0x2051
#define SI446X_PROP_MODEM_SPIKE_DET      0x2054
#define SI446X_PROP_MODEM_ONE_SHOT_AFC   0x2055
#define SI446X_PROP_MODEM_RSSI_MUTE      0x2057
#define SI446X_PROP_MODEM_DSA_CTRL1      0x205B
#define SI446X_PROP_MODEM_DSA_CTRL2      0x205C
#define SI446X_PROP_MODEM_DSA_QUAL       0x205D
#define SI446X_PROP_MODEM_DSA_RSSI       0x205E
#define SI446X_PROP_MODEM_DSA_MISC       0x205F

/** @subsection Si4468 Channel Filter Properties */

#define SI446X_PROP_CHFLT_RX1_COE13      0x2100
#define SI446X_PROP_CHFLT_RX1_COE12      0x2101
#define SI446X_PROP_CHFLT_RX1_COE11      0x2102
#define SI446X_PROP_CHFLT_RX1_COE10      0x2103
#define SI446X_PROP_CHFLT_RX1_COE9       0x2104
#define SI446X_PROP_CHFLT_RX1_COE8       0x2105
#define SI446X_PROP_CHFLT_RX1_COE7       0x2106
#define SI446X_PROP_CHFLT_RX1_COE6       0x2107
#define SI446X_PROP_CHFLT_RX1_COE5       0x2108
#define SI446X_PROP_CHFLT_RX1_COE4       0x2109
#define SI446X_PROP_CHFLT_RX1_COE3       0x210A
#define SI446X_PROP_CHFLT_RX1_COE2       0x210B
#define SI446X_PROP_CHFLT_RX1_COE1       0x210C
#define SI446X_PROP_CHFLT_RX1_COE0       0x210D
#define SI446X_PROP_CHFLT_RX1_COEM0      0x210E
#define SI446X_PROP_CHFLT_RX1_COEM1      0x210F
#define SI446X_PROP_CHFLT_RX1_COEM2      0x2110
#define SI446X_PROP_CHFLT_RX1_COEM3      0x2111
#define SI446X_PROP_CHFLT_RX2_COE13      0x2112
#define SI446X_PROP_CHFLT_RX2_COE12      0x2113
#define SI446X_PROP_CHFLT_RX2_COE11      0x2114
#define SI446X_PROP_CHFLT_RX2_COE10      0x2115
#define SI446X_PROP_CHFLT_RX2_COE9       0x2116
#define SI446X_PROP_CHFLT_RX2_COE8       0x2117
#define SI446X_PROP_CHFLT_RX2_COE7       0x2118
#define SI446X_PROP_CHFLT_RX2_COE6       0x2119
#define SI446X_PROP_CHFLT_RX2_COE5       0x211A
#define SI446X_PROP_CHFLT_RX2_COE4       0x211B
#define SI446X_PROP_CHFLT_RX2_COE3       0x211C
#define SI446X_PROP_CHFLT_RX2_COE2       0x211D
#define SI446X_PROP_CHFLT_RX2_COE1       0x211E
#define SI446X_PROP_CHFLT_RX2_COE0       0x211F
#define SI446X_PROP_CHFLT_RX2_COEM0      0x2120
#define SI446X_PROP_CHFLT_RX2_COEM1      0x2121
#define SI446X_PROP_CHFLT_RX2_COEM2      0x2122
#define SI446X_PROP_CHFLT_RX2_COEM3      0x2123

/** @subsection Si4468 PA Properties */

#define SI446X_PROP_PA_MODE             0x2200
#define SI446X_PROP_PA_PWR_LVL          0x2201
#define SI446X_PROP_PA_BIAS_CLKDUTY     0x2202
#define SI446X_PROP_PA_TC               0x2203
