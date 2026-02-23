/**
 * This file is part of the Titan Project.
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
 * @file internal/interrupt.c
 * @authors Aaron McBride
 * @brief Implementation of reference information for system interrupts.
 */

#include "interrupt.h"

/**************************************************************************************************
 * @section Miscellaneous Constants
 **************************************************************************************************/

const int32_t IRQ_COUNT = 150;

const int32_t NVIC_PRIO_BITS = 4;

const int32_t NVIC_MAX_PRIO = 16;

const int32_t IRQ_EXC_OFFSET = 16;

/**************************************************************************************************
 * @section Exception Numbers
 **************************************************************************************************/

const int32_t RST_EXC_NUM         = 1;
const int32_t NMI_EXC_NUM         = 2;
const int32_t HARD_FAULT_EXC_NUM  = 3;
const int32_t MEM_MANAGE_EXC_NUM  = 4;
const int32_t BUS_FAULT_EXC_NUM   = 5;
const int32_t USAGE_FAULT_EXC_NUM = 6;
const int32_t SVC_EXC_NUM         = 11;
const int32_t DEBUG_MON_EXC_NUM   = 12;
const int32_t PENDSV_EXC_NUM      = 14;
const int32_t SYSTICK_EXC_NUM     = 15;

/**************************************************************************************************
 * @section IRQ Numbers
 **************************************************************************************************/

/** @subsection Non-enumerated IRQ number constants. */

const int32_t PVD_PVM_IRQ_NUM                = 1;
const int32_t RTC_TAMP_STAMP_CSS_LSE_IRQ_NUM = 2;
const int32_t RTC_WKUP_IRQ_NUM               = 3;
const int32_t FLASH_IRQ_NUM                  = 4;
const int32_t RCC_IRQ_NUM                    = 5;
const int32_t ADC1_2_IRQ_NUM                 = 18;
const int32_t EXTI9_5_IRQ_NUM                = 23;
const int32_t TIM1_BRK_IRQ_NUM               = 24;
const int32_t TIM1_UP_IRQ_NUM                = 25;
const int32_t TIM1_TRG_COM_IRQ_NUM           = 26;
const int32_t EXTI15_10_IRQ_NUM              = 40;
const int32_t RTC_ALARM_IRQ_NUM              = 41;
const int32_t TIM8_BRK_TIM12_IRQ_NUM         = 43;
const int32_t TIM8_UP_TIM13_IRQ_NUM          = 44;
const int32_t TIM8_TRG_COM_TIM14_IRQ_NUM     = 45;
const int32_t FMC_IRQ_NUM                    = 48;
const int32_t TIM6_DAC_IRQ_NUM               = 54;
const int32_t ETH_IRQ_NUM                    = 61;
const int32_t ETH_WKUP_IRQ_NUM               = 62;
const int32_t FDCAN_CAL_IRQ_NUM              = 63;
const int32_t OTG_HS_EP1_OUT_IRQ_NUM         = 74;
const int32_t OTG_HS_EP1_IN_IRQ_NUM          = 75;
const int32_t OTG_HS_WKUP_IRQ_NUM            = 76;
const int32_t OTG_HS_IRQ_NUM                 = 77;
const int32_t DCMI_IRQ_NUM                   = 78;
const int32_t CRYP_IRQ_NUM                   = 79;
const int32_t HASH_RNG_IRQ_NUM               = 80;
const int32_t LTDC_IRQ_NUM                   = 88;
const int32_t LTDC_ER_IRQ_NUM                = 89;
const int32_t DMA2D_IRQ_NUM                  = 90;
const int32_t QUADSPI_IRQ_NUM                = 92;
const int32_t CEC_IRQ_NUM                    = 94;
const int32_t SPDIF_IRQ_NUM                  = 97;
const int32_t OTG_FS_EP1_OUT_IRQ_NUM         = 98;
const int32_t OTG_FS_EP1_IN_IRQ_NUM          = 99;
const int32_t OTG_FS_WKUP_IRQ_NUM            = 100;
const int32_t OTG_FS_IRQ_NUM                 = 101;
const int32_t DMAMUX1_OV_IRQ_NUM             = 102;
const int32_t HRTIM_MST_IRQ_NUM              = 103;
const int32_t HRTIM_FLT_IRQ_NUM              = 109;
const int32_t SWPMI_IRQ_NUM                  = 115;
const int32_t MDIOS_WKUP_IRQ_NUM             = 119;
const int32_t MDIOS_IRQ_NUM                  = 120;
const int32_t JPEG_IRQ_NUM                   = 121;
const int32_t MDMA_IRQ_NUM                   = 122;
const int32_t ADC3_IRQ_NUM                   = 127;
const int32_t DMAMUX2_OVR_IRQ_NUM            = 128;
const int32_t COMP_IRQ_NUM                   = 137;
const int32_t LPUART_IRQ_NUM                 = 142;
const int32_t CRS_IRQ_NUM                    = 144;
const int32_t WKUP_IRQ_NUM                   = 149;

/** @subsection Enumerated IRQ number constants. */

const int32_t WWDGx_IRQ_NUM[3] = {
  [1] = 0,
  [2] = 0,
};

const int32_t CPUx_SEV_IRQ_NUM[3] = {
  [1] = 64,
  [2] = 65,
};

const int32_t HSEMx_IRQ_NUM[2] = {
  [0] = 125,
  [1] = 126,
};

const int32_t WWDGx_RST_IRQ_NUM[3] = {
  [1] = 143,
  [2] = 143,
};

const int32_t CPUx_FPU_IRQ_NUM[3] = {
  [1] = 81,
  [2] = 81,
};

const int32_t CPUx_HOLD_CORE_IRQ_NUM[3] = {
  [1] = 148,
  [2] = 148,
};

const int32_t HRTIM_TIMx_IRQ_NUM[6] = {
  [1] = 104,
  [2] = 105,
  [3] = 106,
  [4] = 107,
  [5] = 108,
};

const int32_t BDMA_CHx_IRQ_NUM[9] = {
  [1] = 129,
  [2] = 130,
  [3] = 131,
  [4] = 132,
  [5] = 133,
  [6] = 134,
  [7] = 135,
  [8] = 136,
};

const int32_t I2Cx_EV_IRQ_NUM[5] = {
  [1] = 31,
  [2] = 33,
  [3] = 72,
  [4] = 95,
};

const int32_t I2Cx_ER_IRQ_NUM[5] = {
  [1] = 32,
  [2] = 34,
  [3] = 73,
  [4] = 96,
};

const int32_t SAIx_IRQ_NUM[5] = {
  [1] = 87,
  [2] = 91,
  [3] = 114,
  [4] = 146,
};

const int32_t SDMMCx_IRQ_NUM[3] = {
  [1] = 49,
  [2] = 124,
};

const int32_t SPIx_IRQ_NUM[7] = {
  [1] = 35,
  [2] = 36,
  [3] = 51,
  [4] = 84,
  [5] = 85,
  [6] = 86,
};

const int32_t LPTIMx_IRQ_NUM[6] = {
  [1] = 93,
  [2] = 138,
  [3] = 139,
  [4] = 140,
  [5] = 141,
};

const int32_t EXTIx_IRQ_NUM[5] = {
  [0] = 6,
  [1] = 7,
  [2] = 8,
  [3] = 9,
  [4] = 10,
};

const int32_t DFSDM1_FLTx_IRQ_NUM[4] = {
  [0] = 110,
  [1] = 111,
  [2] = 112,
  [3] = 113,
};

const int32_t TIMx_IRQ_NUM[18] = {
  [2]  = 28,
  [3]  = 29,
  [4]  = 30,
  [5]  = 50,
  [7]  = 55,
  [15] = 116,
  [16] = 117,
  [17] = 118,
};

const int32_t USARTx_IRQ_NUM[7] = {
  [1] = 37,
  [2] = 38,
  [3] = 39,
  [6] = 71,
};

const int32_t UARTx_IRQ_NUM[9] = {
  [4] = 52,
  [5] = 53,
  [7] = 82,
  [8] = 83,
};

const int32_t TIMx_CC_IRQ_NUM[9] = {
  [1] = 27,
  [8] = 46,
};

const int32_t DMAx_STRx_IRQ_NUM[3][8] = {
  [1] = {
    [0] = 11,
    [1] = 12,
    [2] = 13,
    [3] = 14,
    [4] = 15,
    [5] = 16,
    [6] = 17,
    [7] = 47,
  },
  [2] = {
    [0] = 56,
    [1] = 57,
    [2] = 58,
    [3] = 59,
    [4] = 60,
    [5] = 68,
    [6] = 69,
    [7] = 70,
  },
};

const int32_t FDCANx_ITx_IRQ_NUM[3][2] = {
  [1] = {
    [0] = 19,
    [1] = 21,
  },
  [2] = {
    [0] = 20,
    [1] = 22,
  },
};
