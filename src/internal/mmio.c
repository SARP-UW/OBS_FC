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
 * @file internal/mmio.c
 * @authors Aaron McBride
 * @brief Implementation of resources for memory-mapped I/O.
 */

#include "internal/mmio.h"

/**************************************************************************************************
 * @section COMP1 Definitions
 **************************************************************************************************/

/** @subsection COMP1 Register Definitions */

ro_reg32_t const COMP1_SR    = (ro_reg32_t)0x58003800U;
rw_reg32_t const COMP1_ICFR  = (rw_reg32_t)0x58003804U;
rw_reg32_t const COMP1_OR    = (rw_reg32_t)0x58003808U;
rw_reg32_t const COMP1_CFGR1 = (rw_reg32_t)0x5800380CU;
rw_reg32_t const COMP1_CFGR2 = (rw_reg32_t)0x58003810U;

/** @subsection COMP1 Register Field Definitions */

const field32_t COMP1_OR_AFOP        = {.msk = 0x000007FFU, .pos = 0};
const field32_t COMP1_OR_OR          = {.msk = 0xFFFFF800U, .pos = 11};
const field32_t COMP1_CFGR1_EN       = {.msk = 0x00000001U, .pos = 0};
const field32_t COMP1_CFGR1_BRGEN    = {.msk = 0x00000002U, .pos = 1};
const field32_t COMP1_CFGR1_SCALEN   = {.msk = 0x00000004U, .pos = 2};
const field32_t COMP1_CFGR1_POLARITY = {.msk = 0x00000008U, .pos = 3};
const field32_t COMP1_CFGR1_ITEN     = {.msk = 0x00000040U, .pos = 6};
const field32_t COMP1_CFGR1_HYST     = {.msk = 0x00000300U, .pos = 8};
const field32_t COMP1_CFGR1_PWRMODE  = {.msk = 0x00003000U, .pos = 12};
const field32_t COMP1_CFGR1_INMSEL   = {.msk = 0x00070000U, .pos = 16};
const field32_t COMP1_CFGR1_INPSEL   = {.msk = 0x00100000U, .pos = 20};
const field32_t COMP1_CFGR1_BLANKING = {.msk = 0x0F000000U, .pos = 24};
const field32_t COMP1_CFGR1_LOCK     = {.msk = 0x80000000U, .pos = 31};
const field32_t COMP1_CFGR2_EN       = {.msk = 0x00000001U, .pos = 0};
const field32_t COMP1_CFGR2_BRGEN    = {.msk = 0x00000002U, .pos = 1};
const field32_t COMP1_CFGR2_SCALEN   = {.msk = 0x00000004U, .pos = 2};
const field32_t COMP1_CFGR2_POLARITY = {.msk = 0x00000008U, .pos = 3};
const field32_t COMP1_CFGR2_WINMODE  = {.msk = 0x00000010U, .pos = 4};
const field32_t COMP1_CFGR2_ITEN     = {.msk = 0x00000040U, .pos = 6};
const field32_t COMP1_CFGR2_HYST     = {.msk = 0x00000300U, .pos = 8};
const field32_t COMP1_CFGR2_PWRMODE  = {.msk = 0x00003000U, .pos = 12};
const field32_t COMP1_CFGR2_INMSEL   = {.msk = 0x00070000U, .pos = 16};
const field32_t COMP1_CFGR2_INPSEL   = {.msk = 0x00100000U, .pos = 20};
const field32_t COMP1_CFGR2_BLANKING = {.msk = 0x0F000000U, .pos = 24};
const field32_t COMP1_CFGR2_LOCK     = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated COMP1 Register Field Definitions */

field32_t const COMP1_SR_CxVAL[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const COMP1_SR_CxIF[3] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const COMP1_ICFR_CCxIF[3] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

/**************************************************************************************************
 * @section CRS Definitions
 **************************************************************************************************/

/** @subsection CRS Register Definitions */

rw_reg32_t const CRS_CR   = (rw_reg32_t)0x40008400U;
rw_reg32_t const CRS_CFGR = (rw_reg32_t)0x40008404U;
ro_reg32_t const CRS_ISR  = (ro_reg32_t)0x40008408U;
rw_reg32_t const CRS_ICR  = (rw_reg32_t)0x4000840CU;

/** @subsection CRS Register Field Definitions */

const field32_t CRS_CR_SYNCOKIE   = {.msk = 0x00000001U, .pos = 0};
const field32_t CRS_CR_SYNCWARNIE = {.msk = 0x00000002U, .pos = 1};
const field32_t CRS_CR_ERRIE      = {.msk = 0x00000004U, .pos = 2};
const field32_t CRS_CR_ESYNCIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t CRS_CR_CEN        = {.msk = 0x00000020U, .pos = 5};
const field32_t CRS_CR_AUTOTRIMEN = {.msk = 0x00000040U, .pos = 6};
const field32_t CRS_CR_SWSYNC     = {.msk = 0x00000080U, .pos = 7};
const field32_t CRS_CR_TRIM       = {.msk = 0x00003F00U, .pos = 8};
const field32_t CRS_CFGR_RELOAD   = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t CRS_CFGR_FELIM    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t CRS_CFGR_SYNCDIV  = {.msk = 0x07000000U, .pos = 24};
const field32_t CRS_CFGR_SYNCSRC  = {.msk = 0x30000000U, .pos = 28};
const field32_t CRS_CFGR_SYNCPOL  = {.msk = 0x80000000U, .pos = 31};
const field32_t CRS_ISR_SYNCOKF   = {.msk = 0x00000001U, .pos = 0};
const field32_t CRS_ISR_SYNCWARNF = {.msk = 0x00000002U, .pos = 1};
const field32_t CRS_ISR_ERRF      = {.msk = 0x00000004U, .pos = 2};
const field32_t CRS_ISR_ESYNCF    = {.msk = 0x00000008U, .pos = 3};
const field32_t CRS_ISR_SYNCERR   = {.msk = 0x00000100U, .pos = 8};
const field32_t CRS_ISR_SYNCMISS  = {.msk = 0x00000200U, .pos = 9};
const field32_t CRS_ISR_TRIMOVF   = {.msk = 0x00000400U, .pos = 10};
const field32_t CRS_ISR_FEDIR     = {.msk = 0x00008000U, .pos = 15};
const field32_t CRS_ISR_FECAP     = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t CRS_ICR_SYNCOKC   = {.msk = 0x00000001U, .pos = 0};
const field32_t CRS_ICR_SYNCWARNC = {.msk = 0x00000002U, .pos = 1};
const field32_t CRS_ICR_ERRC      = {.msk = 0x00000004U, .pos = 2};
const field32_t CRS_ICR_ESYNCC    = {.msk = 0x00000008U, .pos = 3};

/**************************************************************************************************
 * @section DAC Definitions
 **************************************************************************************************/

/** @subsection DAC Register Definitions */

rw_reg32_t const DAC_CR      = (rw_reg32_t)0x40007400U;
rw_reg32_t const DAC_SWTRGR  = (rw_reg32_t)0x40007404U;
rw_reg32_t const DAC_DHR12R1 = (rw_reg32_t)0x40007408U;
rw_reg32_t const DAC_DHR12L1 = (rw_reg32_t)0x4000740CU;
rw_reg32_t const DAC_DHR8R1  = (rw_reg32_t)0x40007410U;
rw_reg32_t const DAC_DHR12R2 = (rw_reg32_t)0x40007414U;
rw_reg32_t const DAC_DHR12L2 = (rw_reg32_t)0x40007418U;
rw_reg32_t const DAC_DHR8R2  = (rw_reg32_t)0x4000741CU;
rw_reg32_t const DAC_DHR12RD = (rw_reg32_t)0x40007420U;
rw_reg32_t const DAC_DHR12LD = (rw_reg32_t)0x40007424U;
rw_reg32_t const DAC_DHR8RD  = (rw_reg32_t)0x40007428U;
ro_reg32_t const DAC_DOR1    = (ro_reg32_t)0x4000742CU;
ro_reg32_t const DAC_DOR2    = (ro_reg32_t)0x40007430U;
rw_reg32_t const DAC_SR      = (rw_reg32_t)0x40007434U;
rw_reg32_t const DAC_CCR     = (rw_reg32_t)0x40007438U;
rw_reg32_t const DAC_MCR     = (rw_reg32_t)0x4000743CU;
rw_reg32_t const DAC_SHSR1   = (rw_reg32_t)0x40007440U;
rw_reg32_t const DAC_SHSR2   = (rw_reg32_t)0x40007444U;
rw_reg32_t const DAC_SHHR    = (rw_reg32_t)0x40007448U;
rw_reg32_t const DAC_SHRR    = (rw_reg32_t)0x4000744CU;

/** @subsection DAC Register Field Definitions */

const field32_t DAC_DHR12R1_DACC1DHR = {.msk = 0x00000FFFU, .pos = 0};
const field32_t DAC_DHR12L1_DACC1DHR = {.msk = 0x0000FFF0U, .pos = 4};
const field32_t DAC_DHR8R1_DACC1DHR  = {.msk = 0x000000FFU, .pos = 0};
const field32_t DAC_DHR12R2_DACC2DHR = {.msk = 0x00000FFFU, .pos = 0};
const field32_t DAC_DHR12L2_DACC2DHR = {.msk = 0x0000FFF0U, .pos = 4};
const field32_t DAC_DHR8R2_DACC2DHR  = {.msk = 0x000000FFU, .pos = 0};
const field32_t DAC_DOR1_DACC1DOR    = {.msk = 0x00000FFFU, .pos = 0};
const field32_t DAC_DOR2_DACC2DOR    = {.msk = 0x00000FFFU, .pos = 0};
const field32_t DAC_SHSR1_TSAMPLE1   = {.msk = 0x000003FFU, .pos = 0};
const field32_t DAC_SHSR2_TSAMPLE2   = {.msk = 0x000003FFU, .pos = 0};

/** @subsection Enumerated DAC Register Field Definitions */

field32_t const DAC_CR_ENx[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00010000U, .pos = 16},
};

field32_t const DAC_CR_TENx[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const DAC_CR_TSELx[3] = {
  [1] = {.msk = 0x0000001CU, .pos = 2},
  [2] = {.msk = 0x001C0000U, .pos = 18},
};

field32_t const DAC_CR_WAVEx[3] = {
  [1] = {.msk = 0x000000C0U, .pos = 6},
  [2] = {.msk = 0x00C00000U, .pos = 22},
};

field32_t const DAC_CR_MAMPx[3] = {
  [1] = {.msk = 0x00000F00U, .pos = 8},
  [2] = {.msk = 0x0F000000U, .pos = 24},
};

field32_t const DAC_CR_DMAENx[3] = {
  [1] = {.msk = 0x00001000U, .pos = 12},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const DAC_CR_DMAUDRIEx[3] = {
  [1] = {.msk = 0x00002000U, .pos = 13},
  [2] = {.msk = 0x20000000U, .pos = 29},
};

field32_t const DAC_CR_CENx[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const DAC_SWTRGR_SWTRIGx[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const DAC_DHR12RD_DACCxDHR[3] = {
  [1] = {.msk = 0x00000FFFU, .pos = 0},
  [2] = {.msk = 0x0FFF0000U, .pos = 16},
};

field32_t const DAC_SR_DMAUDRx[3] = {
  [1] = {.msk = 0x00002000U, .pos = 13},
  [2] = {.msk = 0x20000000U, .pos = 29},
};

field32_t const DAC_SR_CAL_FLAGx[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const DAC_SR_BWSTx[3] = {
  [1] = {.msk = 0x00008000U, .pos = 15},
  [2] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const DAC_CCR_OTRIMx[3] = {
  [1] = {.msk = 0x0000001FU, .pos = 0},
  [2] = {.msk = 0x001F0000U, .pos = 16},
};

field32_t const DAC_MCR_MODEx[3] = {
  [1] = {.msk = 0x00000007U, .pos = 0},
  [2] = {.msk = 0x00070000U, .pos = 16},
};

field32_t const DAC_SHHR_THOLDx[3] = {
  [1] = {.msk = 0x000003FFU, .pos = 0},
  [2] = {.msk = 0x03FF0000U, .pos = 16},
};

field32_t const DAC_SHRR_TREFRESHx[3] = {
  [1] = {.msk = 0x000000FFU, .pos = 0},
  [2] = {.msk = 0x00FF0000U, .pos = 16},
};

/**************************************************************************************************
 * @section BDMA Definitions
 **************************************************************************************************/

/** @subsection BDMA Register Definitions */

ro_reg32_t const BDMA_ISR  = (ro_reg32_t)0x58025400U;
rw_reg32_t const BDMA_IFCR = (rw_reg32_t)0x58025404U;

/** @subsection Enumerated BDMA Register Definitions */

rw_reg32_t const BDMA_CCRx[9] = {
  [1] = (rw_reg32_t)0x58025408U,
  [2] = (rw_reg32_t)0x5802541CU,
  [3] = (rw_reg32_t)0x58025430U,
  [4] = (rw_reg32_t)0x58025444U,
  [5] = (rw_reg32_t)0x58025458U,
  [6] = (rw_reg32_t)0x5802546CU,
  [7] = (rw_reg32_t)0x58025480U,
  [8] = (rw_reg32_t)0x58025494U,
};

rw_reg32_t const BDMA_CNDTRx[9] = {
  [1] = (rw_reg32_t)0x5802540CU,
  [2] = (rw_reg32_t)0x58025420U,
  [3] = (rw_reg32_t)0x58025434U,
  [4] = (rw_reg32_t)0x58025448U,
  [5] = (rw_reg32_t)0x5802545CU,
  [6] = (rw_reg32_t)0x58025470U,
  [7] = (rw_reg32_t)0x58025484U,
  [8] = (rw_reg32_t)0x58025498U,
};

rw_reg32_t const BDMA_CPARx[9] = {
  [1] = (rw_reg32_t)0x58025410U,
  [2] = (rw_reg32_t)0x58025424U,
  [3] = (rw_reg32_t)0x58025438U,
  [4] = (rw_reg32_t)0x5802544CU,
  [5] = (rw_reg32_t)0x58025460U,
  [6] = (rw_reg32_t)0x58025474U,
  [7] = (rw_reg32_t)0x58025488U,
  [8] = (rw_reg32_t)0x5802549CU,
};

rw_reg32_t const BDMA_CMARx[9] = {
  [1] = (rw_reg32_t)0x58025414U,
  [2] = (rw_reg32_t)0x58025428U,
  [3] = (rw_reg32_t)0x5802543CU,
  [4] = (rw_reg32_t)0x58025450U,
  [5] = (rw_reg32_t)0x58025464U,
  [6] = (rw_reg32_t)0x58025478U,
  [7] = (rw_reg32_t)0x5802548CU,
  [8] = (rw_reg32_t)0x580254A0U,
};

/** @subsection BDMA Register Field Definitions */

const field32_t BDMA_CCRx_EN      = {.msk = 0x00000001U, .pos = 0};
const field32_t BDMA_CCRx_TCIE    = {.msk = 0x00000002U, .pos = 1};
const field32_t BDMA_CCRx_HTIE    = {.msk = 0x00000004U, .pos = 2};
const field32_t BDMA_CCRx_TEIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t BDMA_CCRx_DIR     = {.msk = 0x00000010U, .pos = 4};
const field32_t BDMA_CCRx_CIRC    = {.msk = 0x00000020U, .pos = 5};
const field32_t BDMA_CCRx_PINC    = {.msk = 0x00000040U, .pos = 6};
const field32_t BDMA_CCRx_MINC    = {.msk = 0x00000080U, .pos = 7};
const field32_t BDMA_CCRx_PSIZE   = {.msk = 0x00000300U, .pos = 8};
const field32_t BDMA_CCRx_MSIZE   = {.msk = 0x00000C00U, .pos = 10};
const field32_t BDMA_CCRx_PL      = {.msk = 0x00003000U, .pos = 12};
const field32_t BDMA_CCRx_MEM2MEM = {.msk = 0x00004000U, .pos = 14};
const field32_t BDMA_CNDTRx_NDT   = {.msk = 0x0000FFFFU, .pos = 0};

/** @subsection Enumerated BDMA Register Field Definitions */

field32_t const BDMA_ISR_GIFx[9] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000100U, .pos = 8},
  [4] = {.msk = 0x00001000U, .pos = 12},
  [5] = {.msk = 0x00010000U, .pos = 16},
  [6] = {.msk = 0x00100000U, .pos = 20},
  [7] = {.msk = 0x01000000U, .pos = 24},
  [8] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const BDMA_ISR_TCIFx[9] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000020U, .pos = 5},
  [3] = {.msk = 0x00000200U, .pos = 9},
  [4] = {.msk = 0x00002000U, .pos = 13},
  [5] = {.msk = 0x00020000U, .pos = 17},
  [6] = {.msk = 0x00200000U, .pos = 21},
  [7] = {.msk = 0x02000000U, .pos = 25},
  [8] = {.msk = 0x20000000U, .pos = 29},
};

field32_t const BDMA_ISR_HTIFx[9] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00004000U, .pos = 14},
  [5] = {.msk = 0x00040000U, .pos = 18},
  [6] = {.msk = 0x00400000U, .pos = 22},
  [7] = {.msk = 0x04000000U, .pos = 26},
  [8] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const BDMA_ISR_TEIFx[9] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00008000U, .pos = 15},
  [5] = {.msk = 0x00080000U, .pos = 19},
  [6] = {.msk = 0x00800000U, .pos = 23},
  [7] = {.msk = 0x08000000U, .pos = 27},
  [8] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const BDMA_IFCR_CGIFx[9] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000100U, .pos = 8},
  [4] = {.msk = 0x00001000U, .pos = 12},
  [5] = {.msk = 0x00010000U, .pos = 16},
  [6] = {.msk = 0x00100000U, .pos = 20},
  [7] = {.msk = 0x01000000U, .pos = 24},
  [8] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const BDMA_IFCR_CTCIFx[9] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000020U, .pos = 5},
  [3] = {.msk = 0x00000200U, .pos = 9},
  [4] = {.msk = 0x00002000U, .pos = 13},
  [5] = {.msk = 0x00020000U, .pos = 17},
  [6] = {.msk = 0x00200000U, .pos = 21},
  [7] = {.msk = 0x02000000U, .pos = 25},
  [8] = {.msk = 0x20000000U, .pos = 29},
};

field32_t const BDMA_IFCR_CHTIFx[9] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00004000U, .pos = 14},
  [5] = {.msk = 0x00040000U, .pos = 18},
  [6] = {.msk = 0x00400000U, .pos = 22},
  [7] = {.msk = 0x04000000U, .pos = 26},
  [8] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const BDMA_IFCR_CTEIFx[9] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00008000U, .pos = 15},
  [5] = {.msk = 0x00080000U, .pos = 19},
  [6] = {.msk = 0x00800000U, .pos = 23},
  [7] = {.msk = 0x08000000U, .pos = 27},
  [8] = {.msk = 0x80000000U, .pos = 31},
};

/**************************************************************************************************
 * @section DMA2D Definitions
 **************************************************************************************************/

/** @subsection DMA2D Register Definitions */

rw_reg32_t const DMA2D_CR      = (rw_reg32_t)0x52001000U;
ro_reg32_t const DMA2D_ISR     = (ro_reg32_t)0x52001004U;
rw_reg32_t const DMA2D_IFCR    = (rw_reg32_t)0x52001008U;
rw_reg32_t const DMA2D_FGMAR   = (rw_reg32_t)0x5200100CU;
rw_reg32_t const DMA2D_FGOR    = (rw_reg32_t)0x52001010U;
rw_reg32_t const DMA2D_BGMAR   = (rw_reg32_t)0x52001014U;
rw_reg32_t const DMA2D_BGOR    = (rw_reg32_t)0x52001018U;
rw_reg32_t const DMA2D_FGPFCCR = (rw_reg32_t)0x5200101CU;
rw_reg32_t const DMA2D_FGCOLR  = (rw_reg32_t)0x52001020U;
rw_reg32_t const DMA2D_BGPFCCR = (rw_reg32_t)0x52001024U;
rw_reg32_t const DMA2D_BGCOLR  = (rw_reg32_t)0x52001028U;
rw_reg32_t const DMA2D_FGCMAR  = (rw_reg32_t)0x5200102CU;
rw_reg32_t const DMA2D_BGCMAR  = (rw_reg32_t)0x52001030U;
rw_reg32_t const DMA2D_OPFCCR  = (rw_reg32_t)0x52001034U;
rw_reg32_t const DMA2D_OCOLR   = (rw_reg32_t)0x52001038U;
rw_reg32_t const DMA2D_OMAR    = (rw_reg32_t)0x5200103CU;
rw_reg32_t const DMA2D_OOR     = (rw_reg32_t)0x52001040U;
rw_reg32_t const DMA2D_NLR     = (rw_reg32_t)0x52001044U;
rw_reg32_t const DMA2D_LWR     = (rw_reg32_t)0x52001048U;
rw_reg32_t const DMA2D_AMTCR   = (rw_reg32_t)0x5200104CU;

/** @subsection DMA2D Register Field Definitions */

const field32_t DMA2D_CR_START      = {.msk = 0x00000001U, .pos = 0};
const field32_t DMA2D_CR_SUSP       = {.msk = 0x00000002U, .pos = 1};
const field32_t DMA2D_CR_ABORT      = {.msk = 0x00000004U, .pos = 2};
const field32_t DMA2D_CR_TEIE       = {.msk = 0x00000100U, .pos = 8};
const field32_t DMA2D_CR_TCIE       = {.msk = 0x00000200U, .pos = 9};
const field32_t DMA2D_CR_TWIE       = {.msk = 0x00000400U, .pos = 10};
const field32_t DMA2D_CR_CAEIE      = {.msk = 0x00000800U, .pos = 11};
const field32_t DMA2D_CR_CTCIE      = {.msk = 0x00001000U, .pos = 12};
const field32_t DMA2D_CR_CEIE       = {.msk = 0x00002000U, .pos = 13};
const field32_t DMA2D_CR_MODE       = {.msk = 0x00030000U, .pos = 16};
const field32_t DMA2D_ISR_TEIF      = {.msk = 0x00000001U, .pos = 0};
const field32_t DMA2D_ISR_TCIF      = {.msk = 0x00000002U, .pos = 1};
const field32_t DMA2D_ISR_TWIF      = {.msk = 0x00000004U, .pos = 2};
const field32_t DMA2D_ISR_CAEIF     = {.msk = 0x00000008U, .pos = 3};
const field32_t DMA2D_ISR_CTCIF     = {.msk = 0x00000010U, .pos = 4};
const field32_t DMA2D_ISR_CEIF      = {.msk = 0x00000020U, .pos = 5};
const field32_t DMA2D_IFCR_CTEIF    = {.msk = 0x00000001U, .pos = 0};
const field32_t DMA2D_IFCR_CTCIF    = {.msk = 0x00000002U, .pos = 1};
const field32_t DMA2D_IFCR_CTWIF    = {.msk = 0x00000004U, .pos = 2};
const field32_t DMA2D_IFCR_CAECIF   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMA2D_IFCR_CCTCIF   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMA2D_IFCR_CCEIF    = {.msk = 0x00000020U, .pos = 5};
const field32_t DMA2D_FGOR_LO       = {.msk = 0x00003FFFU, .pos = 0};
const field32_t DMA2D_BGOR_LO       = {.msk = 0x00003FFFU, .pos = 0};
const field32_t DMA2D_FGPFCCR_CM    = {.msk = 0x0000000FU, .pos = 0};
const field32_t DMA2D_FGPFCCR_CCM   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMA2D_FGPFCCR_START = {.msk = 0x00000020U, .pos = 5};
const field32_t DMA2D_FGPFCCR_CS    = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DMA2D_FGPFCCR_AM    = {.msk = 0x00030000U, .pos = 16};
const field32_t DMA2D_FGPFCCR_CSS   = {.msk = 0x000C0000U, .pos = 18};
const field32_t DMA2D_FGPFCCR_AI    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMA2D_FGPFCCR_RBS   = {.msk = 0x00200000U, .pos = 21};
const field32_t DMA2D_FGPFCCR_ALPHA = {.msk = 0xFF000000U, .pos = 24};
const field32_t DMA2D_FGCOLR_BLUE   = {.msk = 0x000000FFU, .pos = 0};
const field32_t DMA2D_FGCOLR_GREEN  = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DMA2D_FGCOLR_RED    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DMA2D_BGPFCCR_CM    = {.msk = 0x0000000FU, .pos = 0};
const field32_t DMA2D_BGPFCCR_CCM   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMA2D_BGPFCCR_START = {.msk = 0x00000020U, .pos = 5};
const field32_t DMA2D_BGPFCCR_CS    = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DMA2D_BGPFCCR_AM    = {.msk = 0x00030000U, .pos = 16};
const field32_t DMA2D_BGPFCCR_AI    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMA2D_BGPFCCR_RBS   = {.msk = 0x00200000U, .pos = 21};
const field32_t DMA2D_BGPFCCR_ALPHA = {.msk = 0xFF000000U, .pos = 24};
const field32_t DMA2D_BGCOLR_BLUE   = {.msk = 0x000000FFU, .pos = 0};
const field32_t DMA2D_BGCOLR_GREEN  = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DMA2D_BGCOLR_RED    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DMA2D_OPFCCR_CM     = {.msk = 0x00000007U, .pos = 0};
const field32_t DMA2D_OPFCCR_AI     = {.msk = 0x00100000U, .pos = 20};
const field32_t DMA2D_OPFCCR_RBS    = {.msk = 0x00200000U, .pos = 21};
const field32_t DMA2D_OCOLR_BLUE    = {.msk = 0x000000FFU, .pos = 0};
const field32_t DMA2D_OCOLR_GREEN   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DMA2D_OCOLR_RED     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DMA2D_OCOLR_ALPHA   = {.msk = 0xFF000000U, .pos = 24};
const field32_t DMA2D_OOR_LO        = {.msk = 0x00003FFFU, .pos = 0};
const field32_t DMA2D_NLR_NL        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t DMA2D_NLR_PL        = {.msk = 0x3FFF0000U, .pos = 16};
const field32_t DMA2D_LWR_LW        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t DMA2D_AMTCR_EN      = {.msk = 0x00000001U, .pos = 0};
const field32_t DMA2D_AMTCR_DT      = {.msk = 0x0000FF00U, .pos = 8};

/**************************************************************************************************
 * @section DMAMUXx Definitions
 **************************************************************************************************/

/** @subsection Enumerated DMAMUXx Register Definitions */

rw_reg32_t const DMAMUXx_CxCR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40020800U,
    [1] = (rw_reg32_t)0x40020804U,
    [2] = (rw_reg32_t)0x40020808U,
    [3] = (rw_reg32_t)0x4002080CU,
    [4] = (rw_reg32_t)0x40020810U,
    [5] = (rw_reg32_t)0x40020814U,
    [6] = (rw_reg32_t)0x40020818U,
    [7] = (rw_reg32_t)0x4002081CU,
  },
  [2] = {
    [0] = (rw_reg32_t)0x58025800U,
    [1] = (rw_reg32_t)0x58025804U,
    [2] = (rw_reg32_t)0x58025808U,
    [3] = (rw_reg32_t)0x5802580CU,
    [4] = (rw_reg32_t)0x58025810U,
    [5] = (rw_reg32_t)0x58025814U,
    [6] = (rw_reg32_t)0x58025818U,
    [7] = (rw_reg32_t)0x5802581CU,
  },
};

rw_reg32_t const DMAMUXx_RGxCR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40020900U,
    [1] = (rw_reg32_t)0x40020904U,
    [2] = (rw_reg32_t)0x40020908U,
    [3] = (rw_reg32_t)0x4002090CU,
    [4] = (rw_reg32_t)0x40020910U,
    [5] = (rw_reg32_t)0x40020914U,
    [6] = (rw_reg32_t)0x40020918U,
    [7] = (rw_reg32_t)0x4002091CU,
  },
  [2] = {
    [0] = (rw_reg32_t)0x58025900U,
    [1] = (rw_reg32_t)0x58025904U,
    [2] = (rw_reg32_t)0x58025908U,
    [3] = (rw_reg32_t)0x5802590CU,
    [4] = (rw_reg32_t)0x58025910U,
    [5] = (rw_reg32_t)0x58025914U,
    [6] = (rw_reg32_t)0x58025918U,
    [7] = (rw_reg32_t)0x5802591CU,
  },
};

ro_reg32_t const DMAMUXx_RGSR[3] = {
  [1] = (ro_reg32_t)0x40020940U,
  [2] = (ro_reg32_t)0x58025940U,
};

rw_reg32_t const DMAMUXx_RGCFR[3] = {
  [1] = (rw_reg32_t)0x40020944U,
  [2] = (rw_reg32_t)0x58025944U,
};

ro_reg32_t const DMAMUXx_CSR[3] = {
  [1] = (ro_reg32_t)0x40020880U,
  [2] = (ro_reg32_t)0x58025880U,
};

rw_reg32_t const DMAMUXx_CFR[3] = {
  [1] = (rw_reg32_t)0x40020884U,
  [2] = (rw_reg32_t)0x58025884U,
};

/** @subsection DMAMUXx Register Field Definitions */

const field32_t DMAMUXx_CxCR_DMAREQ_ID = {.msk = 0x000000FFU, .pos = 0};
const field32_t DMAMUXx_CxCR_SOIE      = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAMUXx_CxCR_EGE       = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAMUXx_CxCR_SE        = {.msk = 0x00010000U, .pos = 16};
const field32_t DMAMUXx_CxCR_SPOL      = {.msk = 0x00060000U, .pos = 17};
const field32_t DMAMUXx_CxCR_NBREQ     = {.msk = 0x00F80000U, .pos = 19};
const field32_t DMAMUXx_CxCR_SYNC_ID   = {.msk = 0x1F000000U, .pos = 24};
const field32_t DMAMUXx_RGxCR_SIG_ID   = {.msk = 0x0000001FU, .pos = 0};
const field32_t DMAMUXx_RGxCR_OIE      = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAMUXx_RGxCR_GE       = {.msk = 0x00010000U, .pos = 16};
const field32_t DMAMUXx_RGxCR_GPOL     = {.msk = 0x00060000U, .pos = 17};
const field32_t DMAMUXx_RGxCR_GNBREQ   = {.msk = 0x00F80000U, .pos = 19};
const field32_t DMAMUXx_RGSR_OF        = {.msk = 0x000000FFU, .pos = 0};
const field32_t DMAMUXx_RGCFR_COF      = {.msk = 0x000000FFU, .pos = 0};
const field32_t DMAMUXx_CSR_SOF        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t DMAMUXx_CFR_CSOF       = {.msk = 0x0000FFFFU, .pos = 0};

/**************************************************************************************************
 * @section FMC Definitions
 **************************************************************************************************/

/** @subsection FMC Register Definitions */

rw_reg32_t const FMC_PCR   = (rw_reg32_t)0x52004080U;
rw_reg32_t const FMC_SR    = (rw_reg32_t)0x52004084U;
rw_reg32_t const FMC_PMEM  = (rw_reg32_t)0x52004088U;
rw_reg32_t const FMC_PATT  = (rw_reg32_t)0x5200408CU;
ro_reg32_t const FMC_ECCR  = (ro_reg32_t)0x52004094U;
rw_reg32_t const FMC_SDCMR = (rw_reg32_t)0x52004150U;
rw_reg32_t const FMC_SDRTR = (rw_reg32_t)0x52004154U;
ro_reg32_t const FMC_SDSR  = (ro_reg32_t)0x52004158U;

/** @subsection Enumerated FMC Register Definitions */

rw_reg32_t const FMC_BCRx[5] = {
  [1] = (rw_reg32_t)0x52004000U,
  [2] = (rw_reg32_t)0x52004008U,
  [3] = (rw_reg32_t)0x52004010U,
  [4] = (rw_reg32_t)0x52004018U,
};

rw_reg32_t const FMC_BTRx[5] = {
  [1] = (rw_reg32_t)0x52004004U,
  [2] = (rw_reg32_t)0x5200400CU,
  [3] = (rw_reg32_t)0x52004014U,
  [4] = (rw_reg32_t)0x5200401CU,
};

rw_reg32_t const FMC_BWTRx[5] = {
  [1] = (rw_reg32_t)0x52004104U,
  [2] = (rw_reg32_t)0x5200410CU,
  [3] = (rw_reg32_t)0x52004114U,
  [4] = (rw_reg32_t)0x5200411CU,
};

rw_reg32_t const FMC_SDCRx[3] = {
  [1] = (rw_reg32_t)0x52004140U,
  [2] = (rw_reg32_t)0x52004144U,
};

rw_reg32_t const FMC_SDTRx[3] = {
  [1] = (rw_reg32_t)0x52004148U,
  [2] = (rw_reg32_t)0x5200414CU,
};

/** @subsection FMC Register Field Definitions */

const field32_t FMC_BCRx_MBKEN     = {.msk = 0x00000001U, .pos = 0};
const field32_t FMC_BCRx_MUXEN     = {.msk = 0x00000002U, .pos = 1};
const field32_t FMC_BCRx_MTYP      = {.msk = 0x0000000CU, .pos = 2};
const field32_t FMC_BCRx_MWID      = {.msk = 0x00000030U, .pos = 4};
const field32_t FMC_BCRx_FACCEN    = {.msk = 0x00000040U, .pos = 6};
const field32_t FMC_BCRx_BURSTEN   = {.msk = 0x00000100U, .pos = 8};
const field32_t FMC_BCRx_WAITPOL   = {.msk = 0x00000200U, .pos = 9};
const field32_t FMC_BCRx_WAITCFG   = {.msk = 0x00000800U, .pos = 11};
const field32_t FMC_BCRx_WREN      = {.msk = 0x00001000U, .pos = 12};
const field32_t FMC_BCRx_WAITEN    = {.msk = 0x00002000U, .pos = 13};
const field32_t FMC_BCRx_EXTMOD    = {.msk = 0x00004000U, .pos = 14};
const field32_t FMC_BCRx_ASYNCWAIT = {.msk = 0x00008000U, .pos = 15};
const field32_t FMC_BCRx_CPSIZE    = {.msk = 0x00070000U, .pos = 16};
const field32_t FMC_BCRx_CBURSTRW  = {.msk = 0x00080000U, .pos = 19};
const field32_t FMC_BCRx_CCLKEN    = {.msk = 0x00100000U, .pos = 20};
const field32_t FMC_BCRx_WFDIS     = {.msk = 0x00200000U, .pos = 21};
const field32_t FMC_BCRx_BMAP      = {.msk = 0x03000000U, .pos = 24};
const field32_t FMC_BCRx_FMCEN     = {.msk = 0x80000000U, .pos = 31};
const field32_t FMC_BTRx_ADDSET    = {.msk = 0x0000000FU, .pos = 0};
const field32_t FMC_BTRx_ADDHLD    = {.msk = 0x000000F0U, .pos = 4};
const field32_t FMC_BTRx_DATAST    = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FMC_BTRx_BUSTURN   = {.msk = 0x000F0000U, .pos = 16};
const field32_t FMC_BTRx_CLKDIV    = {.msk = 0x00F00000U, .pos = 20};
const field32_t FMC_BTRx_DATLAT    = {.msk = 0x0F000000U, .pos = 24};
const field32_t FMC_BTRx_ACCMOD    = {.msk = 0x30000000U, .pos = 28};
const field32_t FMC_PCR_PWAITEN    = {.msk = 0x00000002U, .pos = 1};
const field32_t FMC_PCR_PBKEN      = {.msk = 0x00000004U, .pos = 2};
const field32_t FMC_PCR_PWID       = {.msk = 0x00000030U, .pos = 4};
const field32_t FMC_PCR_ECCEN      = {.msk = 0x00000040U, .pos = 6};
const field32_t FMC_PCR_TCLR       = {.msk = 0x00001E00U, .pos = 9};
const field32_t FMC_PCR_TAR        = {.msk = 0x0001E000U, .pos = 13};
const field32_t FMC_PCR_ECCPS      = {.msk = 0x000E0000U, .pos = 17};
const field32_t FMC_SR_IRS         = {.msk = 0x00000001U, .pos = 0};
const field32_t FMC_SR_ILS         = {.msk = 0x00000002U, .pos = 1};
const field32_t FMC_SR_IFS         = {.msk = 0x00000004U, .pos = 2};
const field32_t FMC_SR_IREN        = {.msk = 0x00000008U, .pos = 3};
const field32_t FMC_SR_ILEN        = {.msk = 0x00000010U, .pos = 4};
const field32_t FMC_SR_IFEN        = {.msk = 0x00000020U, .pos = 5};
const field32_t FMC_SR_FEMPT       = {.msk = 0x00000040U, .pos = 6};
const field32_t FMC_PMEM_MEMSET    = {.msk = 0x000000FFU, .pos = 0};
const field32_t FMC_PMEM_MEMWAIT   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FMC_PMEM_MEMHOLD   = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FMC_PMEM_MEMHIZ    = {.msk = 0xFF000000U, .pos = 24};
const field32_t FMC_PATT_ATTSET    = {.msk = 0x000000FFU, .pos = 0};
const field32_t FMC_PATT_ATTWAIT   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FMC_PATT_ATTHOLD   = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FMC_PATT_ATTHIZ    = {.msk = 0xFF000000U, .pos = 24};
const field32_t FMC_BWTRx_ADDSET   = {.msk = 0x0000000FU, .pos = 0};
const field32_t FMC_BWTRx_ADDHLD   = {.msk = 0x000000F0U, .pos = 4};
const field32_t FMC_BWTRx_DATAST   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FMC_BWTRx_BUSTURN  = {.msk = 0x000F0000U, .pos = 16};
const field32_t FMC_BWTRx_ACCMOD   = {.msk = 0x30000000U, .pos = 28};
const field32_t FMC_SDCRx_NC       = {.msk = 0x00000003U, .pos = 0};
const field32_t FMC_SDCRx_NR       = {.msk = 0x0000000CU, .pos = 2};
const field32_t FMC_SDCRx_MWID     = {.msk = 0x00000030U, .pos = 4};
const field32_t FMC_SDCRx_NB       = {.msk = 0x00000040U, .pos = 6};
const field32_t FMC_SDCRx_CAS      = {.msk = 0x00000180U, .pos = 7};
const field32_t FMC_SDCRx_WP       = {.msk = 0x00000200U, .pos = 9};
const field32_t FMC_SDCRx_SDCLK    = {.msk = 0x00000C00U, .pos = 10};
const field32_t FMC_SDCRx_RBURST   = {.msk = 0x00001000U, .pos = 12};
const field32_t FMC_SDCRx_RPIPE    = {.msk = 0x00006000U, .pos = 13};
const field32_t FMC_SDTRx_TMRD     = {.msk = 0x0000000FU, .pos = 0};
const field32_t FMC_SDTRx_TXSR     = {.msk = 0x000000F0U, .pos = 4};
const field32_t FMC_SDTRx_TRAS     = {.msk = 0x00000F00U, .pos = 8};
const field32_t FMC_SDTRx_TRC      = {.msk = 0x0000F000U, .pos = 12};
const field32_t FMC_SDTRx_TWR      = {.msk = 0x000F0000U, .pos = 16};
const field32_t FMC_SDTRx_TRP      = {.msk = 0x00F00000U, .pos = 20};
const field32_t FMC_SDTRx_TRCD     = {.msk = 0x0F000000U, .pos = 24};
const field32_t FMC_SDCMR_MODE     = {.msk = 0x00000007U, .pos = 0};
const field32_t FMC_SDCMR_NRFS     = {.msk = 0x000001E0U, .pos = 5};
const field32_t FMC_SDCMR_MRD      = {.msk = 0x007FFE00U, .pos = 9};
const field32_t FMC_SDRTR_CRE      = {.msk = 0x00000001U, .pos = 0};
const field32_t FMC_SDRTR_COUNT    = {.msk = 0x00003FFEU, .pos = 1};
const field32_t FMC_SDRTR_REIE     = {.msk = 0x00004000U, .pos = 14};
const field32_t FMC_SDSR_RE        = {.msk = 0x00000001U, .pos = 0};

/** @subsection Enumerated FMC Register Field Definitions */

field32_t const FMC_SDCMR_CTBx[3] = {
  [1] = {.msk = 0x00000010U, .pos = 4},
  [2] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const FMC_SDSR_MODESx[3] = {
  [1] = {.msk = 0x00000006U, .pos = 1},
  [2] = {.msk = 0x00000018U, .pos = 3},
};

/**************************************************************************************************
 * @section CEC Definitions
 **************************************************************************************************/

/** @subsection CEC Register Definitions */

rw_reg32_t const CEC_CR   = (rw_reg32_t)0x40006C00U;
rw_reg32_t const CEC_CFGR = (rw_reg32_t)0x40006C04U;
rw_reg32_t const CEC_TXDR = (rw_reg32_t)0x40006C08U;
ro_reg32_t const CEC_RXDR = (ro_reg32_t)0x40006C0CU;
rw_reg32_t const CEC_ISR  = (rw_reg32_t)0x40006C10U;
rw_reg32_t const CEC_IER  = (rw_reg32_t)0x40006C14U;

/** @subsection CEC Register Field Definitions */

const field32_t CEC_CR_CECEN      = {.msk = 0x00000001U, .pos = 0};
const field32_t CEC_CR_TXSOM      = {.msk = 0x00000002U, .pos = 1};
const field32_t CEC_CR_TXEOM      = {.msk = 0x00000004U, .pos = 2};
const field32_t CEC_CFGR_SFT      = {.msk = 0x00000007U, .pos = 0};
const field32_t CEC_CFGR_RXTOL    = {.msk = 0x00000008U, .pos = 3};
const field32_t CEC_CFGR_BRESTP   = {.msk = 0x00000010U, .pos = 4};
const field32_t CEC_CFGR_BREGEN   = {.msk = 0x00000020U, .pos = 5};
const field32_t CEC_CFGR_LBPEGEN  = {.msk = 0x00000040U, .pos = 6};
const field32_t CEC_CFGR_BRDNOGEN = {.msk = 0x00000080U, .pos = 7};
const field32_t CEC_CFGR_SFTOPT   = {.msk = 0x00000100U, .pos = 8};
const field32_t CEC_CFGR_OAR      = {.msk = 0x7FFF0000U, .pos = 16};
const field32_t CEC_CFGR_LSTN     = {.msk = 0x80000000U, .pos = 31};
const field32_t CEC_TXDR_TXD      = {.msk = 0x000000FFU, .pos = 0};
const field32_t CEC_RXDR_RXD      = {.msk = 0x000000FFU, .pos = 0};
const field32_t CEC_ISR_RXBR      = {.msk = 0x00000001U, .pos = 0};
const field32_t CEC_ISR_RXEND     = {.msk = 0x00000002U, .pos = 1};
const field32_t CEC_ISR_RXOVR     = {.msk = 0x00000004U, .pos = 2};
const field32_t CEC_ISR_BRE       = {.msk = 0x00000008U, .pos = 3};
const field32_t CEC_ISR_SBPE      = {.msk = 0x00000010U, .pos = 4};
const field32_t CEC_ISR_LBPE      = {.msk = 0x00000020U, .pos = 5};
const field32_t CEC_ISR_RXACKE    = {.msk = 0x00000040U, .pos = 6};
const field32_t CEC_ISR_ARBLST    = {.msk = 0x00000080U, .pos = 7};
const field32_t CEC_ISR_TXBR      = {.msk = 0x00000100U, .pos = 8};
const field32_t CEC_ISR_TXEND     = {.msk = 0x00000200U, .pos = 9};
const field32_t CEC_ISR_TXUDR     = {.msk = 0x00000400U, .pos = 10};
const field32_t CEC_ISR_TXERR     = {.msk = 0x00000800U, .pos = 11};
const field32_t CEC_ISR_TXACKE    = {.msk = 0x00001000U, .pos = 12};
const field32_t CEC_IER_RXBRIE    = {.msk = 0x00000001U, .pos = 0};
const field32_t CEC_IER_RXENDIE   = {.msk = 0x00000002U, .pos = 1};
const field32_t CEC_IER_RXOVRIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t CEC_IER_BREIE     = {.msk = 0x00000008U, .pos = 3};
const field32_t CEC_IER_SBPEIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t CEC_IER_LBPEIE    = {.msk = 0x00000020U, .pos = 5};
const field32_t CEC_IER_RXACKIE   = {.msk = 0x00000040U, .pos = 6};
const field32_t CEC_IER_ARBLSTIE  = {.msk = 0x00000080U, .pos = 7};
const field32_t CEC_IER_TXBRIE    = {.msk = 0x00000100U, .pos = 8};
const field32_t CEC_IER_TXENDIE   = {.msk = 0x00000200U, .pos = 9};
const field32_t CEC_IER_TXUDRIE   = {.msk = 0x00000400U, .pos = 10};
const field32_t CEC_IER_TXERRIE   = {.msk = 0x00000800U, .pos = 11};
const field32_t CEC_IER_TXACKIE   = {.msk = 0x00001000U, .pos = 12};

/**************************************************************************************************
 * @section HSEM Definitions
 **************************************************************************************************/

/** @subsection HSEM Register Definitions */

rw_reg32_t const HSEM_IER  = (rw_reg32_t)0x58026500U;
ro_reg32_t const HSEM_ICR  = (ro_reg32_t)0x58026504U;
ro_reg32_t const HSEM_ISR  = (ro_reg32_t)0x58026508U;
ro_reg32_t const HSEM_MISR = (ro_reg32_t)0x5802650CU;
rw_reg32_t const HSEM_CR   = (rw_reg32_t)0x58026540U;
rw_reg32_t const HSEM_KEYR = (rw_reg32_t)0x58026544U;

/** @subsection Enumerated HSEM Register Definitions */

rw_reg32_t const HSEM_Rx[32] = {
  [0]  = (rw_reg32_t)0x58026400U,
  [1]  = (rw_reg32_t)0x58026404U,
  [2]  = (rw_reg32_t)0x58026408U,
  [3]  = (rw_reg32_t)0x5802640CU,
  [4]  = (rw_reg32_t)0x58026410U,
  [5]  = (rw_reg32_t)0x58026414U,
  [6]  = (rw_reg32_t)0x58026418U,
  [7]  = (rw_reg32_t)0x5802641CU,
  [8]  = (rw_reg32_t)0x58026420U,
  [9]  = (rw_reg32_t)0x58026424U,
  [10] = (rw_reg32_t)0x58026428U,
  [11] = (rw_reg32_t)0x5802642CU,
  [12] = (rw_reg32_t)0x58026430U,
  [13] = (rw_reg32_t)0x58026434U,
  [14] = (rw_reg32_t)0x58026438U,
  [15] = (rw_reg32_t)0x5802643CU,
  [16] = (rw_reg32_t)0x58026440U,
  [17] = (rw_reg32_t)0x58026444U,
  [18] = (rw_reg32_t)0x58026448U,
  [19] = (rw_reg32_t)0x5802644CU,
  [20] = (rw_reg32_t)0x58026450U,
  [21] = (rw_reg32_t)0x58026454U,
  [22] = (rw_reg32_t)0x58026458U,
  [23] = (rw_reg32_t)0x5802645CU,
  [24] = (rw_reg32_t)0x58026460U,
  [25] = (rw_reg32_t)0x58026464U,
  [26] = (rw_reg32_t)0x58026468U,
  [27] = (rw_reg32_t)0x5802646CU,
  [28] = (rw_reg32_t)0x58026470U,
  [29] = (rw_reg32_t)0x58026474U,
  [30] = (rw_reg32_t)0x58026478U,
  [31] = (rw_reg32_t)0x5802647CU,
};

ro_reg32_t const HSEM_RLRx[32] = {
  [0]  = (ro_reg32_t)0x58026480U,
  [1]  = (ro_reg32_t)0x58026484U,
  [2]  = (ro_reg32_t)0x58026488U,
  [3]  = (ro_reg32_t)0x5802648CU,
  [4]  = (ro_reg32_t)0x58026490U,
  [5]  = (ro_reg32_t)0x58026494U,
  [6]  = (ro_reg32_t)0x58026498U,
  [7]  = (ro_reg32_t)0x5802649CU,
  [8]  = (ro_reg32_t)0x580264A0U,
  [9]  = (ro_reg32_t)0x580264A4U,
  [10] = (ro_reg32_t)0x580264A8U,
  [11] = (ro_reg32_t)0x580264ACU,
  [12] = (ro_reg32_t)0x580264B0U,
  [13] = (ro_reg32_t)0x580264B4U,
  [14] = (ro_reg32_t)0x580264B8U,
  [15] = (ro_reg32_t)0x580264BCU,
  [16] = (ro_reg32_t)0x580264C0U,
  [17] = (ro_reg32_t)0x580264C4U,
  [18] = (ro_reg32_t)0x580264C8U,
  [19] = (ro_reg32_t)0x580264CCU,
  [20] = (ro_reg32_t)0x580264D0U,
  [21] = (ro_reg32_t)0x580264D4U,
  [22] = (ro_reg32_t)0x580264D8U,
  [23] = (ro_reg32_t)0x580264DCU,
  [24] = (ro_reg32_t)0x580264E0U,
  [25] = (ro_reg32_t)0x580264E4U,
  [26] = (ro_reg32_t)0x580264E8U,
  [27] = (ro_reg32_t)0x580264ECU,
  [28] = (ro_reg32_t)0x580264F0U,
  [29] = (ro_reg32_t)0x580264F4U,
  [30] = (ro_reg32_t)0x580264F8U,
  [31] = (ro_reg32_t)0x580264FCU,
};

/** @subsection HSEM Register Field Definitions */

const field32_t HSEM_Rx_PROCID   = {.msk = 0x000000FFU, .pos = 0};
const field32_t HSEM_Rx_COREID   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t HSEM_Rx_LOCK     = {.msk = 0x80000000U, .pos = 31};
const field32_t HSEM_RLRx_PROCID = {.msk = 0x000000FFU, .pos = 0};
const field32_t HSEM_RLRx_COREID = {.msk = 0x0000FF00U, .pos = 8};
const field32_t HSEM_RLRx_LOCK   = {.msk = 0x80000000U, .pos = 31};
const field32_t HSEM_CR_COREID   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t HSEM_CR_KEY      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t HSEM_KEYR_KEY    = {.msk = 0xFFFF0000U, .pos = 16};

/** @subsection Enumerated HSEM Register Field Definitions */

field32_t const HSEM_IER_ISEMx[32] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
  [16] = {.msk = 0x00010000U, .pos = 16},
  [17] = {.msk = 0x00020000U, .pos = 17},
  [18] = {.msk = 0x00040000U, .pos = 18},
  [19] = {.msk = 0x00080000U, .pos = 19},
  [20] = {.msk = 0x00100000U, .pos = 20},
  [21] = {.msk = 0x00200000U, .pos = 21},
  [22] = {.msk = 0x00400000U, .pos = 22},
  [23] = {.msk = 0x00800000U, .pos = 23},
  [24] = {.msk = 0x01000000U, .pos = 24},
  [25] = {.msk = 0x02000000U, .pos = 25},
  [26] = {.msk = 0x04000000U, .pos = 26},
  [27] = {.msk = 0x08000000U, .pos = 27},
  [28] = {.msk = 0x10000000U, .pos = 28},
  [29] = {.msk = 0x20000000U, .pos = 29},
  [30] = {.msk = 0x40000000U, .pos = 30},
  [31] = {.msk = 0x80000000U, .pos = 31},
};

/**************************************************************************************************
 * @section I2Cx Definitions
 **************************************************************************************************/

/** @subsection Enumerated I2Cx Register Definitions */

rw_reg32_t const I2Cx_CR1[5] = {
  [1] = (rw_reg32_t)0x40005400U,
  [2] = (rw_reg32_t)0x40005800U,
  [3] = (rw_reg32_t)0x40005C00U,
  [4] = (rw_reg32_t)0x58001C00U,
};

rw_reg32_t const I2Cx_CR2[5] = {
  [1] = (rw_reg32_t)0x40005404U,
  [2] = (rw_reg32_t)0x40005804U,
  [3] = (rw_reg32_t)0x40005C04U,
  [4] = (rw_reg32_t)0x58001C04U,
};

rw_reg32_t const I2Cx_OAR1[5] = {
  [1] = (rw_reg32_t)0x40005408U,
  [2] = (rw_reg32_t)0x40005808U,
  [3] = (rw_reg32_t)0x40005C08U,
  [4] = (rw_reg32_t)0x58001C08U,
};

rw_reg32_t const I2Cx_OAR2[5] = {
  [1] = (rw_reg32_t)0x4000540CU,
  [2] = (rw_reg32_t)0x4000580CU,
  [3] = (rw_reg32_t)0x40005C0CU,
  [4] = (rw_reg32_t)0x58001C0CU,
};

rw_reg32_t const I2Cx_TIMINGR[5] = {
  [1] = (rw_reg32_t)0x40005410U,
  [2] = (rw_reg32_t)0x40005810U,
  [3] = (rw_reg32_t)0x40005C10U,
  [4] = (rw_reg32_t)0x58001C10U,
};

rw_reg32_t const I2Cx_TIMEOUTR[5] = {
  [1] = (rw_reg32_t)0x40005414U,
  [2] = (rw_reg32_t)0x40005814U,
  [3] = (rw_reg32_t)0x40005C14U,
  [4] = (rw_reg32_t)0x58001C14U,
};

rw_reg32_t const I2Cx_ISR[5] = {
  [1] = (rw_reg32_t)0x40005418U,
  [2] = (rw_reg32_t)0x40005818U,
  [3] = (rw_reg32_t)0x40005C18U,
  [4] = (rw_reg32_t)0x58001C18U,
};

rw_reg32_t const I2Cx_ICR[5] = {
  [1] = (rw_reg32_t)0x4000541CU,
  [2] = (rw_reg32_t)0x4000581CU,
  [3] = (rw_reg32_t)0x40005C1CU,
  [4] = (rw_reg32_t)0x58001C1CU,
};

ro_reg32_t const I2Cx_PECR[5] = {
  [1] = (ro_reg32_t)0x40005420U,
  [2] = (ro_reg32_t)0x40005820U,
  [3] = (ro_reg32_t)0x40005C20U,
  [4] = (ro_reg32_t)0x58001C20U,
};

ro_reg32_t const I2Cx_RXDR[5] = {
  [1] = (ro_reg32_t)0x40005424U,
  [2] = (ro_reg32_t)0x40005824U,
  [3] = (ro_reg32_t)0x40005C24U,
  [4] = (ro_reg32_t)0x58001C24U,
};

rw_reg32_t const I2Cx_TXDR[5] = {
  [1] = (rw_reg32_t)0x40005428U,
  [2] = (rw_reg32_t)0x40005828U,
  [3] = (rw_reg32_t)0x40005C28U,
  [4] = (rw_reg32_t)0x58001C28U,
};

/** @subsection I2Cx Register Field Definitions */

const field32_t I2Cx_CR1_PE            = {.msk = 0x00000001U, .pos = 0};
const field32_t I2Cx_CR1_TXIE          = {.msk = 0x00000002U, .pos = 1};
const field32_t I2Cx_CR1_RXIE          = {.msk = 0x00000004U, .pos = 2};
const field32_t I2Cx_CR1_ADDRIE        = {.msk = 0x00000008U, .pos = 3};
const field32_t I2Cx_CR1_NACKIE        = {.msk = 0x00000010U, .pos = 4};
const field32_t I2Cx_CR1_STOPIE        = {.msk = 0x00000020U, .pos = 5};
const field32_t I2Cx_CR1_TCIE          = {.msk = 0x00000040U, .pos = 6};
const field32_t I2Cx_CR1_ERRIE         = {.msk = 0x00000080U, .pos = 7};
const field32_t I2Cx_CR1_DNF           = {.msk = 0x00000F00U, .pos = 8};
const field32_t I2Cx_CR1_ANFOFF        = {.msk = 0x00001000U, .pos = 12};
const field32_t I2Cx_CR1_TXDMAEN       = {.msk = 0x00004000U, .pos = 14};
const field32_t I2Cx_CR1_RXDMAEN       = {.msk = 0x00008000U, .pos = 15};
const field32_t I2Cx_CR1_SBC           = {.msk = 0x00010000U, .pos = 16};
const field32_t I2Cx_CR1_NOSTRETCH     = {.msk = 0x00020000U, .pos = 17};
const field32_t I2Cx_CR1_WUPEN         = {.msk = 0x00040000U, .pos = 18};
const field32_t I2Cx_CR1_GCEN          = {.msk = 0x00080000U, .pos = 19};
const field32_t I2Cx_CR1_SMBHEN        = {.msk = 0x00100000U, .pos = 20};
const field32_t I2Cx_CR1_SMBDEN        = {.msk = 0x00200000U, .pos = 21};
const field32_t I2Cx_CR1_ALERTEN       = {.msk = 0x00400000U, .pos = 22};
const field32_t I2Cx_CR1_PECEN         = {.msk = 0x00800000U, .pos = 23};
const field32_t I2Cx_CR2_RD_WRN        = {.msk = 0x00000400U, .pos = 10};
const field32_t I2Cx_CR2_ADD10         = {.msk = 0x00000800U, .pos = 11};
const field32_t I2Cx_CR2_HEAD10R       = {.msk = 0x00001000U, .pos = 12};
const field32_t I2Cx_CR2_START         = {.msk = 0x00002000U, .pos = 13};
const field32_t I2Cx_CR2_STOP          = {.msk = 0x00004000U, .pos = 14};
const field32_t I2Cx_CR2_NACK          = {.msk = 0x00008000U, .pos = 15};
const field32_t I2Cx_CR2_NBYTES        = {.msk = 0x00FF0000U, .pos = 16};
const field32_t I2Cx_CR2_RELOAD        = {.msk = 0x01000000U, .pos = 24};
const field32_t I2Cx_CR2_AUTOEND       = {.msk = 0x02000000U, .pos = 25};
const field32_t I2Cx_CR2_PECBYTE       = {.msk = 0x04000000U, .pos = 26};
const field32_t I2Cx_OAR1_OA1          = {.msk = 0x000003FFU, .pos = 0};
const field32_t I2Cx_OAR1_OA1MODE      = {.msk = 0x00000400U, .pos = 10};
const field32_t I2Cx_OAR1_OA1EN        = {.msk = 0x00008000U, .pos = 15};
const field32_t I2Cx_OAR2_OA2          = {.msk = 0x000000FEU, .pos = 1};
const field32_t I2Cx_OAR2_OA2MSK       = {.msk = 0x00000700U, .pos = 8};
const field32_t I2Cx_OAR2_OA2EN        = {.msk = 0x00008000U, .pos = 15};
const field32_t I2Cx_TIMINGR_SCLL      = {.msk = 0x000000FFU, .pos = 0};
const field32_t I2Cx_TIMINGR_SCLH      = {.msk = 0x0000FF00U, .pos = 8};
const field32_t I2Cx_TIMINGR_SDADEL    = {.msk = 0x000F0000U, .pos = 16};
const field32_t I2Cx_TIMINGR_SCLDEL    = {.msk = 0x00F00000U, .pos = 20};
const field32_t I2Cx_TIMINGR_PRESC     = {.msk = 0xF0000000U, .pos = 28};
const field32_t I2Cx_TIMEOUTR_TIMEOUTA = {.msk = 0x00000FFFU, .pos = 0};
const field32_t I2Cx_TIMEOUTR_TIDLE    = {.msk = 0x00001000U, .pos = 12};
const field32_t I2Cx_TIMEOUTR_TIMOUTEN = {.msk = 0x00008000U, .pos = 15};
const field32_t I2Cx_TIMEOUTR_TIMEOUTB = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t I2Cx_TIMEOUTR_TEXTEN   = {.msk = 0x80000000U, .pos = 31};
const field32_t I2Cx_ISR_TXE           = {.msk = 0x00000001U, .pos = 0};
const field32_t I2Cx_ISR_TXIS          = {.msk = 0x00000002U, .pos = 1};
const field32_t I2Cx_ISR_RXNE          = {.msk = 0x00000004U, .pos = 2};
const field32_t I2Cx_ISR_ADDR          = {.msk = 0x00000008U, .pos = 3};
const field32_t I2Cx_ISR_NACKF         = {.msk = 0x00000010U, .pos = 4};
const field32_t I2Cx_ISR_STOPF         = {.msk = 0x00000020U, .pos = 5};
const field32_t I2Cx_ISR_TC            = {.msk = 0x00000040U, .pos = 6};
const field32_t I2Cx_ISR_TCR           = {.msk = 0x00000080U, .pos = 7};
const field32_t I2Cx_ISR_BERR          = {.msk = 0x00000100U, .pos = 8};
const field32_t I2Cx_ISR_ARLO          = {.msk = 0x00000200U, .pos = 9};
const field32_t I2Cx_ISR_OVR           = {.msk = 0x00000400U, .pos = 10};
const field32_t I2Cx_ISR_PECERR        = {.msk = 0x00000800U, .pos = 11};
const field32_t I2Cx_ISR_TIMEOUT       = {.msk = 0x00001000U, .pos = 12};
const field32_t I2Cx_ISR_ALERT         = {.msk = 0x00002000U, .pos = 13};
const field32_t I2Cx_ISR_BUSY          = {.msk = 0x00008000U, .pos = 15};
const field32_t I2Cx_ISR_DIR           = {.msk = 0x00010000U, .pos = 16};
const field32_t I2Cx_ISR_ADDCODE       = {.msk = 0x00FE0000U, .pos = 17};
const field32_t I2Cx_ICR_ADDRCF        = {.msk = 0x00000008U, .pos = 3};
const field32_t I2Cx_ICR_NACKCF        = {.msk = 0x00000010U, .pos = 4};
const field32_t I2Cx_ICR_STOPCF        = {.msk = 0x00000020U, .pos = 5};
const field32_t I2Cx_ICR_BERRCF        = {.msk = 0x00000100U, .pos = 8};
const field32_t I2Cx_ICR_ARLOCF        = {.msk = 0x00000200U, .pos = 9};
const field32_t I2Cx_ICR_OVRCF         = {.msk = 0x00000400U, .pos = 10};
const field32_t I2Cx_ICR_PECCF         = {.msk = 0x00000800U, .pos = 11};
const field32_t I2Cx_ICR_TIMOUTCF      = {.msk = 0x00001000U, .pos = 12};
const field32_t I2Cx_ICR_ALERTCF       = {.msk = 0x00002000U, .pos = 13};
const field32_t I2Cx_PECR_PEC          = {.msk = 0x000000FFU, .pos = 0};
const field32_t I2Cx_RXDR_RXDATA       = {.msk = 0x000000FFU, .pos = 0};
const field32_t I2Cx_TXDR_TXDATA       = {.msk = 0x000000FFU, .pos = 0};
const field32_t I2Cx_CR2_SADD_7BIT     = {.msk = 0x000000FCU, .pos = 1};  /** @todo */
const field32_t I2Cx_CR2_SADD_9BIT     = {.msk = 0x000003FFU, .pos = 0};  /** @todo */

/** @subsection Enumerated I2Cx Register Field Definitions */

field32_t const I2Cx_CR2_SADDx[10] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
};

/**************************************************************************************************
 * @section GPIOx Definitions
 **************************************************************************************************/

/** @subsection Enumerated GPIOx Register Definitions */

rw_reg32_t const GPIOx_MODER[11] = {
  [0]  = (rw_reg32_t)0x58020000U,
  [1]  = (rw_reg32_t)0x58020400U,
  [2]  = (rw_reg32_t)0x58020800U,
  [3]  = (rw_reg32_t)0x58020C00U,
  [4]  = (rw_reg32_t)0x58021000U,
  [5]  = (rw_reg32_t)0x58021400U,
  [6]  = (rw_reg32_t)0x58021800U,
  [7]  = (rw_reg32_t)0x58021C00U,
  [8]  = (rw_reg32_t)0x58022000U,
  [9]  = (rw_reg32_t)0x58022400U,
  [10] = (rw_reg32_t)0x58022800U,
};

rw_reg32_t const GPIOx_OTYPER[11] = {
  [0]  = (rw_reg32_t)0x58020004U,
  [1]  = (rw_reg32_t)0x58020404U,
  [2]  = (rw_reg32_t)0x58020804U,
  [3]  = (rw_reg32_t)0x58020C04U,
  [4]  = (rw_reg32_t)0x58021004U,
  [5]  = (rw_reg32_t)0x58021404U,
  [6]  = (rw_reg32_t)0x58021804U,
  [7]  = (rw_reg32_t)0x58021C04U,
  [8]  = (rw_reg32_t)0x58022004U,
  [9]  = (rw_reg32_t)0x58022404U,
  [10] = (rw_reg32_t)0x58022804U,
};

rw_reg32_t const GPIOx_OSPEEDR[11] = {
  [0]  = (rw_reg32_t)0x58020008U,
  [1]  = (rw_reg32_t)0x58020408U,
  [2]  = (rw_reg32_t)0x58020808U,
  [3]  = (rw_reg32_t)0x58020C08U,
  [4]  = (rw_reg32_t)0x58021008U,
  [5]  = (rw_reg32_t)0x58021408U,
  [6]  = (rw_reg32_t)0x58021808U,
  [7]  = (rw_reg32_t)0x58021C08U,
  [8]  = (rw_reg32_t)0x58022008U,
  [9]  = (rw_reg32_t)0x58022408U,
  [10] = (rw_reg32_t)0x58022808U,
};

rw_reg32_t const GPIOx_PUPDR[11] = {
  [0]  = (rw_reg32_t)0x5802000CU,
  [1]  = (rw_reg32_t)0x5802040CU,
  [2]  = (rw_reg32_t)0x5802080CU,
  [3]  = (rw_reg32_t)0x58020C0CU,
  [4]  = (rw_reg32_t)0x5802100CU,
  [5]  = (rw_reg32_t)0x5802140CU,
  [6]  = (rw_reg32_t)0x5802180CU,
  [7]  = (rw_reg32_t)0x58021C0CU,
  [8]  = (rw_reg32_t)0x5802200CU,
  [9]  = (rw_reg32_t)0x5802240CU,
  [10] = (rw_reg32_t)0x5802280CU,
};

ro_reg32_t const GPIOx_IDR[11] = {
  [0]  = (ro_reg32_t)0x58020010U,
  [1]  = (ro_reg32_t)0x58020410U,
  [2]  = (ro_reg32_t)0x58020810U,
  [3]  = (ro_reg32_t)0x58020C10U,
  [4]  = (ro_reg32_t)0x58021010U,
  [5]  = (ro_reg32_t)0x58021410U,
  [6]  = (ro_reg32_t)0x58021810U,
  [7]  = (ro_reg32_t)0x58021C10U,
  [8]  = (ro_reg32_t)0x58022010U,
  [9]  = (ro_reg32_t)0x58022410U,
  [10] = (ro_reg32_t)0x58022810U,
};

rw_reg32_t const GPIOx_ODR[11] = {
  [0]  = (rw_reg32_t)0x58020014U,
  [1]  = (rw_reg32_t)0x58020414U,
  [2]  = (rw_reg32_t)0x58020814U,
  [3]  = (rw_reg32_t)0x58020C14U,
  [4]  = (rw_reg32_t)0x58021014U,
  [5]  = (rw_reg32_t)0x58021414U,
  [6]  = (rw_reg32_t)0x58021814U,
  [7]  = (rw_reg32_t)0x58021C14U,
  [8]  = (rw_reg32_t)0x58022014U,
  [9]  = (rw_reg32_t)0x58022414U,
  [10] = (rw_reg32_t)0x58022814U,
};

rw_reg32_t const GPIOx_BSRR[11] = {
  [0]  = (rw_reg32_t)0x58020018U,
  [1]  = (rw_reg32_t)0x58020418U,
  [2]  = (rw_reg32_t)0x58020818U,
  [3]  = (rw_reg32_t)0x58020C18U,
  [4]  = (rw_reg32_t)0x58021018U,
  [5]  = (rw_reg32_t)0x58021418U,
  [6]  = (rw_reg32_t)0x58021818U,
  [7]  = (rw_reg32_t)0x58021C18U,
  [8]  = (rw_reg32_t)0x58022018U,
  [9]  = (rw_reg32_t)0x58022418U,
  [10] = (rw_reg32_t)0x58022818U,
};

rw_reg32_t const GPIOx_LCKR[11] = {
  [0]  = (rw_reg32_t)0x5802001CU,
  [1]  = (rw_reg32_t)0x5802041CU,
  [2]  = (rw_reg32_t)0x5802081CU,
  [3]  = (rw_reg32_t)0x58020C1CU,
  [4]  = (rw_reg32_t)0x5802101CU,
  [5]  = (rw_reg32_t)0x5802141CU,
  [6]  = (rw_reg32_t)0x5802181CU,
  [7]  = (rw_reg32_t)0x58021C1CU,
  [8]  = (rw_reg32_t)0x5802201CU,
  [9]  = (rw_reg32_t)0x5802241CU,
  [10] = (rw_reg32_t)0x5802281CU,
};

rw_reg32_t const GPIOx_AFRL[11] = {
  [0]  = (rw_reg32_t)0x58020020U,
  [1]  = (rw_reg32_t)0x58020420U,
  [2]  = (rw_reg32_t)0x58020820U,
  [3]  = (rw_reg32_t)0x58020C20U,
  [4]  = (rw_reg32_t)0x58021020U,
  [5]  = (rw_reg32_t)0x58021420U,
  [6]  = (rw_reg32_t)0x58021820U,
  [7]  = (rw_reg32_t)0x58021C20U,
  [8]  = (rw_reg32_t)0x58022020U,
  [9]  = (rw_reg32_t)0x58022420U,
  [10] = (rw_reg32_t)0x58022820U,
};

rw_reg32_t const GPIOx_AFRH[11] = {
  [0]  = (rw_reg32_t)0x58020024U,
  [1]  = (rw_reg32_t)0x58020424U,
  [2]  = (rw_reg32_t)0x58020824U,
  [3]  = (rw_reg32_t)0x58020C24U,
  [4]  = (rw_reg32_t)0x58021024U,
  [5]  = (rw_reg32_t)0x58021424U,
  [6]  = (rw_reg32_t)0x58021824U,
  [7]  = (rw_reg32_t)0x58021C24U,
  [8]  = (rw_reg32_t)0x58022024U,
  [9]  = (rw_reg32_t)0x58022424U,
  [10] = (rw_reg32_t)0x58022824U,
};

/** @subsection GPIOx Register Field Definitions */

const field32_t GPIOx_LCKR_LCKK = {.msk = 0x00010000U, .pos = 16};

/** @subsection Enumerated GPIOx Register Field Definitions */

field32_t const GPIOx_MODER_MODEx[16] = {
  [0]  = {.msk = 0x00000003U, .pos = 0},
  [1]  = {.msk = 0x0000000CU, .pos = 2},
  [2]  = {.msk = 0x00000030U, .pos = 4},
  [3]  = {.msk = 0x000000C0U, .pos = 6},
  [4]  = {.msk = 0x00000300U, .pos = 8},
  [5]  = {.msk = 0x00000C00U, .pos = 10},
  [6]  = {.msk = 0x00003000U, .pos = 12},
  [7]  = {.msk = 0x0000C000U, .pos = 14},
  [8]  = {.msk = 0x00030000U, .pos = 16},
  [9]  = {.msk = 0x000C0000U, .pos = 18},
  [10] = {.msk = 0x00300000U, .pos = 20},
  [11] = {.msk = 0x00C00000U, .pos = 22},
  [12] = {.msk = 0x03000000U, .pos = 24},
  [13] = {.msk = 0x0C000000U, .pos = 26},
  [14] = {.msk = 0x30000000U, .pos = 28},
  [15] = {.msk = 0xC0000000U, .pos = 30},
};

field32_t const GPIOx_OTYPER_OTx[16] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const GPIOx_OSPEEDR_OSPEEDx[16] = {
  [0]  = {.msk = 0x00000003U, .pos = 0},
  [1]  = {.msk = 0x0000000CU, .pos = 2},
  [2]  = {.msk = 0x00000030U, .pos = 4},
  [3]  = {.msk = 0x000000C0U, .pos = 6},
  [4]  = {.msk = 0x00000300U, .pos = 8},
  [5]  = {.msk = 0x00000C00U, .pos = 10},
  [6]  = {.msk = 0x00003000U, .pos = 12},
  [7]  = {.msk = 0x0000C000U, .pos = 14},
  [8]  = {.msk = 0x00030000U, .pos = 16},
  [9]  = {.msk = 0x000C0000U, .pos = 18},
  [10] = {.msk = 0x00300000U, .pos = 20},
  [11] = {.msk = 0x00C00000U, .pos = 22},
  [12] = {.msk = 0x03000000U, .pos = 24},
  [13] = {.msk = 0x0C000000U, .pos = 26},
  [14] = {.msk = 0x30000000U, .pos = 28},
  [15] = {.msk = 0xC0000000U, .pos = 30},
};

field32_t const GPIOx_PUPDR_PUPDx[16] = {
  [0]  = {.msk = 0x00000003U, .pos = 0},
  [1]  = {.msk = 0x0000000CU, .pos = 2},
  [2]  = {.msk = 0x00000030U, .pos = 4},
  [3]  = {.msk = 0x000000C0U, .pos = 6},
  [4]  = {.msk = 0x00000300U, .pos = 8},
  [5]  = {.msk = 0x00000C00U, .pos = 10},
  [6]  = {.msk = 0x00003000U, .pos = 12},
  [7]  = {.msk = 0x0000C000U, .pos = 14},
  [8]  = {.msk = 0x00030000U, .pos = 16},
  [9]  = {.msk = 0x000C0000U, .pos = 18},
  [10] = {.msk = 0x00300000U, .pos = 20},
  [11] = {.msk = 0x00C00000U, .pos = 22},
  [12] = {.msk = 0x03000000U, .pos = 24},
  [13] = {.msk = 0x0C000000U, .pos = 26},
  [14] = {.msk = 0x30000000U, .pos = 28},
  [15] = {.msk = 0xC0000000U, .pos = 30},
};

field32_t const GPIOx_IDR_IDx[16] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const GPIOx_ODR_ODx[16] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const GPIOx_BSRR_BSx[16] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const GPIOx_BSRR_BRx[16] = {
  [0]  = {.msk = 0x00010000U, .pos = 16},
  [1]  = {.msk = 0x00020000U, .pos = 17},
  [2]  = {.msk = 0x00040000U, .pos = 18},
  [3]  = {.msk = 0x00080000U, .pos = 19},
  [4]  = {.msk = 0x00100000U, .pos = 20},
  [5]  = {.msk = 0x00200000U, .pos = 21},
  [6]  = {.msk = 0x00400000U, .pos = 22},
  [7]  = {.msk = 0x00800000U, .pos = 23},
  [8]  = {.msk = 0x01000000U, .pos = 24},
  [9]  = {.msk = 0x02000000U, .pos = 25},
  [10] = {.msk = 0x04000000U, .pos = 26},
  [11] = {.msk = 0x08000000U, .pos = 27},
  [12] = {.msk = 0x10000000U, .pos = 28},
  [13] = {.msk = 0x20000000U, .pos = 29},
  [14] = {.msk = 0x40000000U, .pos = 30},
  [15] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const GPIOx_LCKR_LCKx[16] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const GPIOx_AFRL_AFSELx[8] = {
  [0] = {.msk = 0x0000000FU, .pos = 0},
  [1] = {.msk = 0x000000F0U, .pos = 4},
  [2] = {.msk = 0x00000F00U, .pos = 8},
  [3] = {.msk = 0x0000F000U, .pos = 12},
  [4] = {.msk = 0x000F0000U, .pos = 16},
  [5] = {.msk = 0x00F00000U, .pos = 20},
  [6] = {.msk = 0x0F000000U, .pos = 24},
  [7] = {.msk = 0xF0000000U, .pos = 28},
};

field32_t const GPIOx_AFRH_AFSELx[8] = {
  [0] = {.msk = 0x0000000FU, .pos = 0},
  [1] = {.msk = 0x000000F0U, .pos = 4},
  [2] = {.msk = 0x00000F00U, .pos = 8},
  [3] = {.msk = 0x0000F000U, .pos = 12},
  [4] = {.msk = 0x000F0000U, .pos = 16},
  [5] = {.msk = 0x00F00000U, .pos = 20},
  [6] = {.msk = 0x0F000000U, .pos = 24},
  [7] = {.msk = 0xF0000000U, .pos = 28},
};

/**************************************************************************************************
 * @section JPEG Definitions
 **************************************************************************************************/

/** @subsection JPEG Register Definitions */

rw_reg32_t const JPEG_CONFR0 = (rw_reg32_t)0x52003000U;
rw_reg32_t const JPEG_CONFR1 = (rw_reg32_t)0x52003004U;
rw_reg32_t const JPEG_CONFR2 = (rw_reg32_t)0x52003008U;
rw_reg32_t const JPEG_CONFR3 = (rw_reg32_t)0x5200300CU;
rw_reg32_t const JPEG_CR     = (rw_reg32_t)0x52003030U;
ro_reg32_t const JPEG_SR     = (ro_reg32_t)0x52003034U;
rw_reg32_t const JPEG_CFR    = (rw_reg32_t)0x52003038U;
rw_reg32_t const JPEG_DIR    = (rw_reg32_t)0x52003040U;
ro_reg32_t const JPEG_DOR    = (ro_reg32_t)0x52003044U;

/** @subsection Enumerated JPEG Register Definitions */

rw_reg32_t const JPEG_CONFRNx[5] = {
  [1] = (rw_reg32_t)0x52003010U,
  [2] = (rw_reg32_t)0x52003014U,
  [3] = (rw_reg32_t)0x52003018U,
  [4] = (rw_reg32_t)0x5200301CU,
};

/** @subsection JPEG Register Field Definitions */

const field32_t JPEG_CONFR0_START      = {.msk = 0x00000001U, .pos = 0};
const field32_t JPEG_CONFR1_NF         = {.msk = 0x00000003U, .pos = 0};
const field32_t JPEG_CONFR1_DE         = {.msk = 0x00000008U, .pos = 3};
const field32_t JPEG_CONFR1_COLORSPACE = {.msk = 0x00000030U, .pos = 4};
const field32_t JPEG_CONFR1_NS         = {.msk = 0x000000C0U, .pos = 6};
const field32_t JPEG_CONFR1_HDR        = {.msk = 0x00000100U, .pos = 8};
const field32_t JPEG_CONFR1_YSIZE      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t JPEG_CONFR2_NMCU       = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t JPEG_CONFR3_XSIZE      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t JPEG_CONFRNx_HD        = {.msk = 0x00000001U, .pos = 0};
const field32_t JPEG_CONFRNx_HA        = {.msk = 0x00000002U, .pos = 1};
const field32_t JPEG_CONFRNx_QT        = {.msk = 0x0000000CU, .pos = 2};
const field32_t JPEG_CONFRNx_NB        = {.msk = 0x000000F0U, .pos = 4};
const field32_t JPEG_CONFRNx_VSF       = {.msk = 0x00000F00U, .pos = 8};
const field32_t JPEG_CONFRNx_HSF       = {.msk = 0x0000F000U, .pos = 12};
const field32_t JPEG_CR_JCEN           = {.msk = 0x00000001U, .pos = 0};
const field32_t JPEG_CR_IFTIE          = {.msk = 0x00000002U, .pos = 1};
const field32_t JPEG_CR_IFNFIE         = {.msk = 0x00000004U, .pos = 2};
const field32_t JPEG_CR_OFTIE          = {.msk = 0x00000008U, .pos = 3};
const field32_t JPEG_CR_OFNEIE         = {.msk = 0x00000010U, .pos = 4};
const field32_t JPEG_CR_EOCIE          = {.msk = 0x00000020U, .pos = 5};
const field32_t JPEG_CR_HPDIE          = {.msk = 0x00000040U, .pos = 6};
const field32_t JPEG_CR_IDMAEN         = {.msk = 0x00000800U, .pos = 11};
const field32_t JPEG_CR_ODMAEN         = {.msk = 0x00001000U, .pos = 12};
const field32_t JPEG_CR_IFF            = {.msk = 0x00002000U, .pos = 13};
const field32_t JPEG_CR_OFF            = {.msk = 0x00004000U, .pos = 14};
const field32_t JPEG_SR_IFTF           = {.msk = 0x00000002U, .pos = 1};
const field32_t JPEG_SR_IFNFF          = {.msk = 0x00000004U, .pos = 2};
const field32_t JPEG_SR_OFTF           = {.msk = 0x00000008U, .pos = 3};
const field32_t JPEG_SR_OFNEF          = {.msk = 0x00000010U, .pos = 4};
const field32_t JPEG_SR_EOCF           = {.msk = 0x00000020U, .pos = 5};
const field32_t JPEG_SR_HPDF           = {.msk = 0x00000040U, .pos = 6};
const field32_t JPEG_SR_COF            = {.msk = 0x00000080U, .pos = 7};
const field32_t JPEG_CFR_CEOCF         = {.msk = 0x00000020U, .pos = 5};
const field32_t JPEG_CFR_CHPDF         = {.msk = 0x00000040U, .pos = 6};

/**************************************************************************************************
 * @section MDMA Definitions
 **************************************************************************************************/

/** @subsection MDMA Register Definitions */

ro_reg32_t const MDMA_MDMA_GISR0   = (ro_reg32_t)0x52000000U;
ro_reg32_t const MDMA_MDMA_C0ISR   = (ro_reg32_t)0x52000040U;
rw_reg32_t const MDMA_MDMA_C0IFCR  = (rw_reg32_t)0x52000044U;
ro_reg32_t const MDMA_MDMA_C1ISR   = (ro_reg32_t)0x52000080U;
rw_reg32_t const MDMA_MDMA_C1IFCR  = (rw_reg32_t)0x52000084U;
ro_reg32_t const MDMA_MDMA_C2ISR   = (ro_reg32_t)0x520000C0U;
rw_reg32_t const MDMA_MDMA_C2IFCR  = (rw_reg32_t)0x520000C4U;
ro_reg32_t const MDMA_MDMA_C3ISR   = (ro_reg32_t)0x52000100U;
rw_reg32_t const MDMA_MDMA_C3IFCR  = (rw_reg32_t)0x52000104U;
ro_reg32_t const MDMA_MDMA_C4ISR   = (ro_reg32_t)0x52000140U;
rw_reg32_t const MDMA_MDMA_C4IFCR  = (rw_reg32_t)0x52000144U;
ro_reg32_t const MDMA_MDMA_C5ISR   = (ro_reg32_t)0x52000180U;
rw_reg32_t const MDMA_MDMA_C5IFCR  = (rw_reg32_t)0x52000184U;
ro_reg32_t const MDMA_MDMA_C6ISR   = (ro_reg32_t)0x520001C0U;
rw_reg32_t const MDMA_MDMA_C6IFCR  = (rw_reg32_t)0x520001C4U;
ro_reg32_t const MDMA_MDMA_C7ISR   = (ro_reg32_t)0x52000200U;
rw_reg32_t const MDMA_MDMA_C7IFCR  = (rw_reg32_t)0x52000204U;
ro_reg32_t const MDMA_MDMA_C8ISR   = (ro_reg32_t)0x52000240U;
rw_reg32_t const MDMA_MDMA_C8IFCR  = (rw_reg32_t)0x52000244U;
ro_reg32_t const MDMA_MDMA_C9ISR   = (ro_reg32_t)0x52000280U;
rw_reg32_t const MDMA_MDMA_C9IFCR  = (rw_reg32_t)0x52000284U;
ro_reg32_t const MDMA_MDMA_C10ISR  = (ro_reg32_t)0x520002C0U;
rw_reg32_t const MDMA_MDMA_C10IFCR = (rw_reg32_t)0x520002C4U;
ro_reg32_t const MDMA_MDMA_C11ISR  = (ro_reg32_t)0x52000300U;
rw_reg32_t const MDMA_MDMA_C11IFCR = (rw_reg32_t)0x52000304U;
ro_reg32_t const MDMA_MDMA_C12ISR  = (ro_reg32_t)0x52000340U;
rw_reg32_t const MDMA_MDMA_C12IFCR = (rw_reg32_t)0x52000344U;
ro_reg32_t const MDMA_MDMA_C13ISR  = (ro_reg32_t)0x52000380U;
rw_reg32_t const MDMA_MDMA_C13IFCR = (rw_reg32_t)0x52000384U;
ro_reg32_t const MDMA_MDMA_C14ISR  = (ro_reg32_t)0x520003C0U;
rw_reg32_t const MDMA_MDMA_C14IFCR = (rw_reg32_t)0x520003C4U;
ro_reg32_t const MDMA_MDMA_C15ISR  = (ro_reg32_t)0x52000400U;
rw_reg32_t const MDMA_MDMA_C15IFCR = (rw_reg32_t)0x52000404U;

/** @subsection Enumerated MDMA Register Definitions */

ro_reg32_t const MDMA_MDMA_CxESR[16] = {
  [0]  = (ro_reg32_t)0x52000048U,
  [1]  = (ro_reg32_t)0x52000088U,
  [2]  = (ro_reg32_t)0x520000C8U,
  [3]  = (ro_reg32_t)0x52000108U,
  [4]  = (ro_reg32_t)0x52000148U,
  [5]  = (ro_reg32_t)0x52000188U,
  [6]  = (ro_reg32_t)0x520001C8U,
  [7]  = (ro_reg32_t)0x52000208U,
  [8]  = (ro_reg32_t)0x52000248U,
  [9]  = (ro_reg32_t)0x52000288U,
  [10] = (ro_reg32_t)0x520002C8U,
  [11] = (ro_reg32_t)0x52000308U,
  [12] = (ro_reg32_t)0x52000348U,
  [13] = (ro_reg32_t)0x52000388U,
  [14] = (ro_reg32_t)0x520003C8U,
  [15] = (ro_reg32_t)0x52000408U,
};

rw_reg32_t const MDMA_MDMA_CxCR[16] = {
  [0]  = (rw_reg32_t)0x5200004CU,
  [1]  = (rw_reg32_t)0x5200008CU,
  [2]  = (rw_reg32_t)0x520000CCU,
  [3]  = (rw_reg32_t)0x5200010CU,
  [4]  = (rw_reg32_t)0x5200014CU,
  [5]  = (rw_reg32_t)0x5200018CU,
  [6]  = (rw_reg32_t)0x520001CCU,
  [7]  = (rw_reg32_t)0x5200020CU,
  [8]  = (rw_reg32_t)0x5200024CU,
  [9]  = (rw_reg32_t)0x5200028CU,
  [10] = (rw_reg32_t)0x520002CCU,
  [11] = (rw_reg32_t)0x5200030CU,
  [12] = (rw_reg32_t)0x5200034CU,
  [13] = (rw_reg32_t)0x5200038CU,
  [14] = (rw_reg32_t)0x520003CCU,
  [15] = (rw_reg32_t)0x5200040CU,
};

rw_reg32_t const MDMA_MDMA_CxTCR[16] = {
  [0]  = (rw_reg32_t)0x52000050U,
  [1]  = (rw_reg32_t)0x52000090U,
  [2]  = (rw_reg32_t)0x520000D0U,
  [3]  = (rw_reg32_t)0x52000110U,
  [4]  = (rw_reg32_t)0x52000150U,
  [5]  = (rw_reg32_t)0x52000190U,
  [6]  = (rw_reg32_t)0x520001D0U,
  [7]  = (rw_reg32_t)0x52000210U,
  [8]  = (rw_reg32_t)0x52000250U,
  [9]  = (rw_reg32_t)0x52000290U,
  [10] = (rw_reg32_t)0x520002D0U,
  [11] = (rw_reg32_t)0x52000310U,
  [12] = (rw_reg32_t)0x52000350U,
  [13] = (rw_reg32_t)0x52000390U,
  [14] = (rw_reg32_t)0x520003D0U,
  [15] = (rw_reg32_t)0x52000410U,
};

rw_reg32_t const MDMA_MDMA_CxBNDTR[16] = {
  [0]  = (rw_reg32_t)0x52000054U,
  [1]  = (rw_reg32_t)0x52000094U,
  [2]  = (rw_reg32_t)0x520000D4U,
  [3]  = (rw_reg32_t)0x52000114U,
  [4]  = (rw_reg32_t)0x52000154U,
  [5]  = (rw_reg32_t)0x52000194U,
  [6]  = (rw_reg32_t)0x520001D4U,
  [7]  = (rw_reg32_t)0x52000214U,
  [8]  = (rw_reg32_t)0x52000254U,
  [9]  = (rw_reg32_t)0x52000294U,
  [10] = (rw_reg32_t)0x520002D4U,
  [11] = (rw_reg32_t)0x52000314U,
  [12] = (rw_reg32_t)0x52000354U,
  [13] = (rw_reg32_t)0x52000394U,
  [14] = (rw_reg32_t)0x520003D4U,
  [15] = (rw_reg32_t)0x52000414U,
};

rw_reg32_t const MDMA_MDMA_CxSAR[16] = {
  [0]  = (rw_reg32_t)0x52000058U,
  [1]  = (rw_reg32_t)0x52000098U,
  [2]  = (rw_reg32_t)0x520000D8U,
  [3]  = (rw_reg32_t)0x52000118U,
  [4]  = (rw_reg32_t)0x52000158U,
  [5]  = (rw_reg32_t)0x52000198U,
  [6]  = (rw_reg32_t)0x520001D8U,
  [7]  = (rw_reg32_t)0x52000218U,
  [8]  = (rw_reg32_t)0x52000258U,
  [9]  = (rw_reg32_t)0x52000298U,
  [10] = (rw_reg32_t)0x520002D8U,
  [11] = (rw_reg32_t)0x52000318U,
  [12] = (rw_reg32_t)0x52000358U,
  [13] = (rw_reg32_t)0x52000398U,
  [14] = (rw_reg32_t)0x520003D8U,
  [15] = (rw_reg32_t)0x52000418U,
};

rw_reg32_t const MDMA_MDMA_CxDAR[16] = {
  [0]  = (rw_reg32_t)0x5200005CU,
  [1]  = (rw_reg32_t)0x5200009CU,
  [2]  = (rw_reg32_t)0x520000DCU,
  [3]  = (rw_reg32_t)0x5200011CU,
  [4]  = (rw_reg32_t)0x5200015CU,
  [5]  = (rw_reg32_t)0x5200019CU,
  [6]  = (rw_reg32_t)0x520001DCU,
  [7]  = (rw_reg32_t)0x5200021CU,
  [8]  = (rw_reg32_t)0x5200025CU,
  [9]  = (rw_reg32_t)0x5200029CU,
  [10] = (rw_reg32_t)0x520002DCU,
  [11] = (rw_reg32_t)0x5200031CU,
  [12] = (rw_reg32_t)0x5200035CU,
  [13] = (rw_reg32_t)0x5200039CU,
  [14] = (rw_reg32_t)0x520003DCU,
  [15] = (rw_reg32_t)0x5200041CU,
};

rw_reg32_t const MDMA_MDMA_CxBRUR[16] = {
  [0]  = (rw_reg32_t)0x52000060U,
  [1]  = (rw_reg32_t)0x520000A0U,
  [2]  = (rw_reg32_t)0x520000E0U,
  [3]  = (rw_reg32_t)0x52000120U,
  [4]  = (rw_reg32_t)0x52000160U,
  [5]  = (rw_reg32_t)0x520001A0U,
  [6]  = (rw_reg32_t)0x520001E0U,
  [7]  = (rw_reg32_t)0x52000220U,
  [8]  = (rw_reg32_t)0x52000260U,
  [9]  = (rw_reg32_t)0x520002A0U,
  [10] = (rw_reg32_t)0x520002E0U,
  [11] = (rw_reg32_t)0x52000320U,
  [12] = (rw_reg32_t)0x52000360U,
  [13] = (rw_reg32_t)0x520003A0U,
  [14] = (rw_reg32_t)0x520003E0U,
  [15] = (rw_reg32_t)0x52000420U,
};

rw_reg32_t const MDMA_MDMA_CxLAR[16] = {
  [0]  = (rw_reg32_t)0x52000064U,
  [1]  = (rw_reg32_t)0x520000A4U,
  [2]  = (rw_reg32_t)0x520000E4U,
  [3]  = (rw_reg32_t)0x52000124U,
  [4]  = (rw_reg32_t)0x52000164U,
  [5]  = (rw_reg32_t)0x520001A4U,
  [6]  = (rw_reg32_t)0x520001E4U,
  [7]  = (rw_reg32_t)0x52000224U,
  [8]  = (rw_reg32_t)0x52000264U,
  [9]  = (rw_reg32_t)0x520002A4U,
  [10] = (rw_reg32_t)0x520002E4U,
  [11] = (rw_reg32_t)0x52000324U,
  [12] = (rw_reg32_t)0x52000364U,
  [13] = (rw_reg32_t)0x520003A4U,
  [14] = (rw_reg32_t)0x520003E4U,
  [15] = (rw_reg32_t)0x52000424U,
};

rw_reg32_t const MDMA_MDMA_CxTBR[16] = {
  [0]  = (rw_reg32_t)0x52000068U,
  [1]  = (rw_reg32_t)0x520000A8U,
  [2]  = (rw_reg32_t)0x520000E8U,
  [3]  = (rw_reg32_t)0x52000128U,
  [4]  = (rw_reg32_t)0x52000168U,
  [5]  = (rw_reg32_t)0x520001A8U,
  [6]  = (rw_reg32_t)0x520001E8U,
  [7]  = (rw_reg32_t)0x52000228U,
  [8]  = (rw_reg32_t)0x52000268U,
  [9]  = (rw_reg32_t)0x520002A8U,
  [10] = (rw_reg32_t)0x520002E8U,
  [11] = (rw_reg32_t)0x52000328U,
  [12] = (rw_reg32_t)0x52000368U,
  [13] = (rw_reg32_t)0x520003A8U,
  [14] = (rw_reg32_t)0x520003E8U,
  [15] = (rw_reg32_t)0x52000428U,
};

rw_reg32_t const MDMA_MDMA_CxMAR[16] = {
  [0]  = (rw_reg32_t)0x52000070U,
  [1]  = (rw_reg32_t)0x520000B0U,
  [2]  = (rw_reg32_t)0x520000F0U,
  [3]  = (rw_reg32_t)0x52000130U,
  [4]  = (rw_reg32_t)0x52000170U,
  [5]  = (rw_reg32_t)0x520001B0U,
  [6]  = (rw_reg32_t)0x520001F0U,
  [7]  = (rw_reg32_t)0x52000230U,
  [8]  = (rw_reg32_t)0x52000270U,
  [9]  = (rw_reg32_t)0x520002B0U,
  [10] = (rw_reg32_t)0x520002F0U,
  [11] = (rw_reg32_t)0x52000330U,
  [12] = (rw_reg32_t)0x52000370U,
  [13] = (rw_reg32_t)0x520003B0U,
  [14] = (rw_reg32_t)0x520003F0U,
  [15] = (rw_reg32_t)0x52000430U,
};

rw_reg32_t const MDMA_MDMA_CxMDR[16] = {
  [0]  = (rw_reg32_t)0x52000074U,
  [1]  = (rw_reg32_t)0x520000B4U,
  [2]  = (rw_reg32_t)0x520000F4U,
  [3]  = (rw_reg32_t)0x52000134U,
  [4]  = (rw_reg32_t)0x52000174U,
  [5]  = (rw_reg32_t)0x520001B4U,
  [6]  = (rw_reg32_t)0x520001F4U,
  [7]  = (rw_reg32_t)0x52000234U,
  [8]  = (rw_reg32_t)0x52000274U,
  [9]  = (rw_reg32_t)0x520002B4U,
  [10] = (rw_reg32_t)0x520002F4U,
  [11] = (rw_reg32_t)0x52000334U,
  [12] = (rw_reg32_t)0x52000374U,
  [13] = (rw_reg32_t)0x520003B4U,
  [14] = (rw_reg32_t)0x520003F4U,
  [15] = (rw_reg32_t)0x52000434U,
};

/** @subsection MDMA Register Field Definitions */

const field32_t MDMA_MDMA_C0ISR_TEIF0      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C0ISR_CTCIF0     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C0ISR_BRTIF0     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C0ISR_BTIF0      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C0ISR_TCIF0      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C0ISR_CRQA0      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C0IFCR_CTEIF0    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C0IFCR_CCTCIF0   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C0IFCR_CBRTIF0   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C0IFCR_CBTIF0    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C0IFCR_CLTCIF0   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_CxESR_TEA        = {.msk = 0x0000007FU, .pos = 0};
const field32_t MDMA_MDMA_CxESR_TED        = {.msk = 0x00000080U, .pos = 7};
const field32_t MDMA_MDMA_CxESR_TELD       = {.msk = 0x00000100U, .pos = 8};
const field32_t MDMA_MDMA_CxESR_TEMD       = {.msk = 0x00000200U, .pos = 9};
const field32_t MDMA_MDMA_CxESR_ASE        = {.msk = 0x00000400U, .pos = 10};
const field32_t MDMA_MDMA_CxESR_BSE        = {.msk = 0x00000800U, .pos = 11};
const field32_t MDMA_MDMA_CxCR_EN          = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_CxCR_TEIE        = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_CxCR_CTCIE       = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_CxCR_BRTIE       = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_CxCR_BTIE        = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_CxCR_TCIE        = {.msk = 0x00000020U, .pos = 5};
const field32_t MDMA_MDMA_CxCR_PL          = {.msk = 0x000000C0U, .pos = 6};
const field32_t MDMA_MDMA_CxCR_BEX         = {.msk = 0x00001000U, .pos = 12};
const field32_t MDMA_MDMA_CxCR_HEX         = {.msk = 0x00002000U, .pos = 13};
const field32_t MDMA_MDMA_CxCR_WEX         = {.msk = 0x00004000U, .pos = 14};
const field32_t MDMA_MDMA_CxCR_SWRQ        = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_CxTCR_SINC       = {.msk = 0x00000003U, .pos = 0};
const field32_t MDMA_MDMA_CxTCR_DINC       = {.msk = 0x0000000CU, .pos = 2};
const field32_t MDMA_MDMA_CxTCR_SSIZE      = {.msk = 0x00000030U, .pos = 4};
const field32_t MDMA_MDMA_CxTCR_DSIZE      = {.msk = 0x000000C0U, .pos = 6};
const field32_t MDMA_MDMA_CxTCR_SINCOS     = {.msk = 0x00000300U, .pos = 8};
const field32_t MDMA_MDMA_CxTCR_DINCOS     = {.msk = 0x00000C00U, .pos = 10};
const field32_t MDMA_MDMA_CxTCR_SBURST     = {.msk = 0x00007000U, .pos = 12};
const field32_t MDMA_MDMA_CxTCR_DBURST     = {.msk = 0x00038000U, .pos = 15};
const field32_t MDMA_MDMA_CxTCR_TLEN       = {.msk = 0x01FC0000U, .pos = 18};
const field32_t MDMA_MDMA_CxTCR_PKE        = {.msk = 0x02000000U, .pos = 25};
const field32_t MDMA_MDMA_CxTCR_PAM        = {.msk = 0x0C000000U, .pos = 26};
const field32_t MDMA_MDMA_CxTCR_TRGM       = {.msk = 0x30000000U, .pos = 28};
const field32_t MDMA_MDMA_CxTCR_SWRM       = {.msk = 0x40000000U, .pos = 30};
const field32_t MDMA_MDMA_CxTCR_BWM        = {.msk = 0x80000000U, .pos = 31};
const field32_t MDMA_MDMA_CxBNDTR_BNDT     = {.msk = 0x0001FFFFU, .pos = 0};
const field32_t MDMA_MDMA_CxBNDTR_BRSUM    = {.msk = 0x00040000U, .pos = 18};
const field32_t MDMA_MDMA_CxBNDTR_BRDUM    = {.msk = 0x00080000U, .pos = 19};
const field32_t MDMA_MDMA_CxBNDTR_BRC      = {.msk = 0xFFF00000U, .pos = 20};
const field32_t MDMA_MDMA_CxBRUR_SUV       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDMA_MDMA_CxBRUR_DUV       = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t MDMA_MDMA_CxTBR_TSEL       = {.msk = 0x0000003FU, .pos = 0};
const field32_t MDMA_MDMA_CxTBR_SBUS       = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_CxTBR_DBUS       = {.msk = 0x00020000U, .pos = 17};
const field32_t MDMA_MDMA_C1ISR_TEIF1      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C1ISR_CTCIF1     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C1ISR_BRTIF1     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C1ISR_BTIF1      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C1ISR_TCIF1      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C1ISR_CRQA1      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C1IFCR_CTEIF1    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C1IFCR_CCTCIF1   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C1IFCR_CBRTIF1   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C1IFCR_CBTIF1    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C1IFCR_CLTCIF1   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C2ISR_TEIF2      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C2ISR_CTCIF2     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C2ISR_BRTIF2     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C2ISR_BTIF2      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C2ISR_TCIF2      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C2ISR_CRQA2      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C2IFCR_CTEIF2    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C2IFCR_CCTCIF2   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C2IFCR_CBRTIF2   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C2IFCR_CBTIF2    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C2IFCR_CLTCIF2   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C3ISR_TEIF3      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C3ISR_CTCIF3     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C3ISR_BRTIF3     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C3ISR_BTIF3      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C3ISR_TCIF3      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C3ISR_CRQA3      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C3IFCR_CTEIF3    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C3IFCR_CCTCIF3   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C3IFCR_CBRTIF3   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C3IFCR_CBTIF3    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C3IFCR_CLTCIF3   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C4ISR_TEIF4      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C4ISR_CTCIF4     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C4ISR_BRTIF4     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C4ISR_BTIF4      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C4ISR_TCIF4      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C4ISR_CRQA4      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C4IFCR_CTEIF4    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C4IFCR_CCTCIF4   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C4IFCR_CBRTIF4   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C4IFCR_CBTIF4    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C4IFCR_CLTCIF4   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C5ISR_TEIF5      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C5ISR_CTCIF5     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C5ISR_BRTIF5     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C5ISR_BTIF5      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C5ISR_TCIF5      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C5ISR_CRQA5      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C5IFCR_CTEIF5    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C5IFCR_CCTCIF5   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C5IFCR_CBRTIF5   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C5IFCR_CBTIF5    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C5IFCR_CLTCIF5   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C6ISR_TEIF6      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C6ISR_CTCIF6     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C6ISR_BRTIF6     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C6ISR_BTIF6      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C6ISR_TCIF6      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C6ISR_CRQA6      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C6IFCR_CTEIF6    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C6IFCR_CCTCIF6   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C6IFCR_CBRTIF6   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C6IFCR_CBTIF6    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C6IFCR_CLTCIF6   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C7ISR_TEIF7      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C7ISR_CTCIF7     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C7ISR_BRTIF7     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C7ISR_BTIF7      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C7ISR_TCIF7      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C7ISR_CRQA7      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C7IFCR_CTEIF7    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C7IFCR_CCTCIF7   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C7IFCR_CBRTIF7   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C7IFCR_CBTIF7    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C7IFCR_CLTCIF7   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C8ISR_TEIF8      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C8ISR_CTCIF8     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C8ISR_BRTIF8     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C8ISR_BTIF8      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C8ISR_TCIF8      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C8ISR_CRQA8      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C8IFCR_CTEIF8    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C8IFCR_CCTCIF8   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C8IFCR_CBRTIF8   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C8IFCR_CBTIF8    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C8IFCR_CLTCIF8   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C9ISR_TEIF9      = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C9ISR_CTCIF9     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C9ISR_BRTIF9     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C9ISR_BTIF9      = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C9ISR_TCIF9      = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C9ISR_CRQA9      = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C9IFCR_CTEIF9    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C9IFCR_CCTCIF9   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C9IFCR_CBRTIF9   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C9IFCR_CBTIF9    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C9IFCR_CLTCIF9   = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C10ISR_TEIF10    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C10ISR_CTCIF10   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C10ISR_BRTIF10   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C10ISR_BTIF10    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C10ISR_TCIF10    = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C10ISR_CRQA10    = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C10IFCR_CTEIF10  = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C10IFCR_CCTCIF10 = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C10IFCR_CBRTIF10 = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C10IFCR_CBTIF10  = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C10IFCR_CLTCIF10 = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C11ISR_TEIF11    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C11ISR_CTCIF11   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C11ISR_BRTIF11   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C11ISR_BTIF11    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C11ISR_TCIF11    = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C11ISR_CRQA11    = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C11IFCR_CTEIF11  = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C11IFCR_CCTCIF11 = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C11IFCR_CBRTIF11 = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C11IFCR_CBTIF11  = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C11IFCR_CLTCIF11 = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C12ISR_TEIF12    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C12ISR_CTCIF12   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C12ISR_BRTIF12   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C12ISR_BTIF12    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C12ISR_TCIF12    = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C12ISR_CRQA12    = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C12IFCR_CTEIF12  = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C12IFCR_CCTCIF12 = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C12IFCR_CBRTIF12 = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C12IFCR_CBTIF12  = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C12IFCR_CLTCIF12 = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C13ISR_TEIF13    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C13ISR_CTCIF13   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C13ISR_BRTIF13   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C13ISR_BTIF13    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C13ISR_TCIF13    = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C13ISR_CRQA13    = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C13IFCR_CTEIF13  = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C13IFCR_CCTCIF13 = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C13IFCR_CBRTIF13 = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C13IFCR_CBTIF13  = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C13IFCR_CLTCIF13 = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C14ISR_TEIF14    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C14ISR_CTCIF14   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C14ISR_BRTIF14   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C14ISR_BTIF14    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C14ISR_TCIF14    = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C14ISR_CRQA14    = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C14IFCR_CTEIF14  = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C14IFCR_CCTCIF14 = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C14IFCR_CBRTIF14 = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C14IFCR_CBTIF14  = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C14IFCR_CLTCIF14 = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C15ISR_TEIF15    = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C15ISR_CTCIF15   = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C15ISR_BRTIF15   = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C15ISR_BTIF15    = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C15ISR_TCIF15    = {.msk = 0x00000010U, .pos = 4};
const field32_t MDMA_MDMA_C15ISR_CRQA15    = {.msk = 0x00010000U, .pos = 16};
const field32_t MDMA_MDMA_C15IFCR_CTEIF15  = {.msk = 0x00000001U, .pos = 0};
const field32_t MDMA_MDMA_C15IFCR_CCTCIF15 = {.msk = 0x00000002U, .pos = 1};
const field32_t MDMA_MDMA_C15IFCR_CBRTIF15 = {.msk = 0x00000004U, .pos = 2};
const field32_t MDMA_MDMA_C15IFCR_CBTIF15  = {.msk = 0x00000008U, .pos = 3};
const field32_t MDMA_MDMA_C15IFCR_CLTCIF15 = {.msk = 0x00000010U, .pos = 4};

/** @subsection Enumerated MDMA Register Field Definitions */

field32_t const MDMA_MDMA_GISR0_GIFx[16] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
};

/**************************************************************************************************
 * @section QUADSPI Definitions
 **************************************************************************************************/

/** @subsection QUADSPI Register Definitions */

rw_reg32_t const QUADSPI_CR    = (rw_reg32_t)0x52005000U;
rw_reg32_t const QUADSPI_DCR   = (rw_reg32_t)0x52005004U;
ro_reg32_t const QUADSPI_SR    = (ro_reg32_t)0x52005008U;
rw_reg32_t const QUADSPI_FCR   = (rw_reg32_t)0x5200500CU;
rw_reg32_t const QUADSPI_DLR   = (rw_reg32_t)0x52005010U;
rw_reg32_t const QUADSPI_CCR   = (rw_reg32_t)0x52005014U;
rw_reg32_t const QUADSPI_AR    = (rw_reg32_t)0x52005018U;
rw_reg32_t const QUADSPI_ABR   = (rw_reg32_t)0x5200501CU;
rw_reg32_t const QUADSPI_DR    = (rw_reg32_t)0x52005020U;
rw_reg32_t const QUADSPI_PSMKR = (rw_reg32_t)0x52005024U;
rw_reg32_t const QUADSPI_PSMAR = (rw_reg32_t)0x52005028U;
rw_reg32_t const QUADSPI_PIR   = (rw_reg32_t)0x5200502CU;
rw_reg32_t const QUADSPI_LPTR  = (rw_reg32_t)0x52005030U;

/** @subsection QUADSPI Register Field Definitions */

const field32_t QUADSPI_CR_EN           = {.msk = 0x00000001U, .pos = 0};
const field32_t QUADSPI_CR_ABORT        = {.msk = 0x00000002U, .pos = 1};
const field32_t QUADSPI_CR_DMAEN        = {.msk = 0x00000004U, .pos = 2};
const field32_t QUADSPI_CR_TCEN         = {.msk = 0x00000008U, .pos = 3};
const field32_t QUADSPI_CR_SSHIFT       = {.msk = 0x00000010U, .pos = 4};
const field32_t QUADSPI_CR_DFM          = {.msk = 0x00000040U, .pos = 6};
const field32_t QUADSPI_CR_FSEL         = {.msk = 0x00000080U, .pos = 7};
const field32_t QUADSPI_CR_FTHRES       = {.msk = 0x00001F00U, .pos = 8};
const field32_t QUADSPI_CR_TEIE         = {.msk = 0x00010000U, .pos = 16};
const field32_t QUADSPI_CR_TCIE         = {.msk = 0x00020000U, .pos = 17};
const field32_t QUADSPI_CR_FTIE         = {.msk = 0x00040000U, .pos = 18};
const field32_t QUADSPI_CR_SMIE         = {.msk = 0x00080000U, .pos = 19};
const field32_t QUADSPI_CR_TOIE         = {.msk = 0x00100000U, .pos = 20};
const field32_t QUADSPI_CR_APMS         = {.msk = 0x00400000U, .pos = 22};
const field32_t QUADSPI_CR_PMM          = {.msk = 0x00800000U, .pos = 23};
const field32_t QUADSPI_CR_PRESCALER    = {.msk = 0xFF000000U, .pos = 24};
const field32_t QUADSPI_DCR_CKMODE      = {.msk = 0x00000001U, .pos = 0};
const field32_t QUADSPI_DCR_CSHT        = {.msk = 0x00000700U, .pos = 8};
const field32_t QUADSPI_DCR_FSIZE       = {.msk = 0x001F0000U, .pos = 16};
const field32_t QUADSPI_SR_TEF          = {.msk = 0x00000001U, .pos = 0};
const field32_t QUADSPI_SR_TCF          = {.msk = 0x00000002U, .pos = 1};
const field32_t QUADSPI_SR_FTF          = {.msk = 0x00000004U, .pos = 2};
const field32_t QUADSPI_SR_SMF          = {.msk = 0x00000008U, .pos = 3};
const field32_t QUADSPI_SR_TOF          = {.msk = 0x00000010U, .pos = 4};
const field32_t QUADSPI_SR_BUSY         = {.msk = 0x00000020U, .pos = 5};
const field32_t QUADSPI_SR_FLEVEL       = {.msk = 0x00003F00U, .pos = 8};
const field32_t QUADSPI_FCR_CTEF        = {.msk = 0x00000001U, .pos = 0};
const field32_t QUADSPI_FCR_CTCF        = {.msk = 0x00000002U, .pos = 1};
const field32_t QUADSPI_FCR_CSMF        = {.msk = 0x00000008U, .pos = 3};
const field32_t QUADSPI_FCR_CTOF        = {.msk = 0x00000010U, .pos = 4};
const field32_t QUADSPI_CCR_INSTRUCTION = {.msk = 0x000000FFU, .pos = 0};
const field32_t QUADSPI_CCR_IMODE       = {.msk = 0x00000300U, .pos = 8};
const field32_t QUADSPI_CCR_ADMODE      = {.msk = 0x00000C00U, .pos = 10};
const field32_t QUADSPI_CCR_ADSIZE      = {.msk = 0x00003000U, .pos = 12};
const field32_t QUADSPI_CCR_ABMODE      = {.msk = 0x0000C000U, .pos = 14};
const field32_t QUADSPI_CCR_ABSIZE      = {.msk = 0x00030000U, .pos = 16};
const field32_t QUADSPI_CCR_DCYC        = {.msk = 0x007C0000U, .pos = 18};
const field32_t QUADSPI_CCR_DMODE       = {.msk = 0x03000000U, .pos = 24};
const field32_t QUADSPI_CCR_FMODE       = {.msk = 0x0C000000U, .pos = 26};
const field32_t QUADSPI_CCR_SIOO        = {.msk = 0x10000000U, .pos = 28};
const field32_t QUADSPI_CCR_DHHC        = {.msk = 0x40000000U, .pos = 30};
const field32_t QUADSPI_CCR_DDRM        = {.msk = 0x80000000U, .pos = 31};
const field32_t QUADSPI_PIR_INTERVAL    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t QUADSPI_LPTR_TIMEOUT    = {.msk = 0x0000FFFFU, .pos = 0};

/**************************************************************************************************
 * @section RNG Definitions
 **************************************************************************************************/

/** @subsection RNG Register Definitions */

rw_reg32_t const RNG_CR = (rw_reg32_t)0x48021800U;
rw_reg32_t const RNG_SR = (rw_reg32_t)0x48021804U;
ro_reg32_t const RNG_DR = (ro_reg32_t)0x48021808U;

/** @subsection RNG Register Field Definitions */

const field32_t RNG_CR_RNGEN = {.msk = 0x00000004U, .pos = 2};
const field32_t RNG_CR_IE    = {.msk = 0x00000008U, .pos = 3};
const field32_t RNG_CR_CED   = {.msk = 0x00000020U, .pos = 5};
const field32_t RNG_SR_DRDY  = {.msk = 0x00000001U, .pos = 0};
const field32_t RNG_SR_CECS  = {.msk = 0x00000002U, .pos = 1};
const field32_t RNG_SR_SECS  = {.msk = 0x00000004U, .pos = 2};
const field32_t RNG_SR_CEIS  = {.msk = 0x00000020U, .pos = 5};
const field32_t RNG_SR_SEIS  = {.msk = 0x00000040U, .pos = 6};

/**************************************************************************************************
 * @section RTC Definitions
 **************************************************************************************************/

/** @subsection RTC Register Definitions */

rw_reg32_t const RTC_RTC_TR       = (rw_reg32_t)0x58004000U;
rw_reg32_t const RTC_RTC_DR       = (rw_reg32_t)0x58004004U;
rw_reg32_t const RTC_RTC_CR       = (rw_reg32_t)0x58004008U;
rw_reg32_t const RTC_RTC_ISR      = (rw_reg32_t)0x5800400CU;
rw_reg32_t const RTC_RTC_PRER     = (rw_reg32_t)0x58004010U;
rw_reg32_t const RTC_RTC_WUTR     = (rw_reg32_t)0x58004014U;
rw_reg32_t const RTC_RTC_ALRMAR   = (rw_reg32_t)0x5800401CU;
rw_reg32_t const RTC_RTC_ALRMBR   = (rw_reg32_t)0x58004020U;
rw_reg32_t const RTC_RTC_WPR      = (rw_reg32_t)0x58004024U;
ro_reg32_t const RTC_RTC_SSR      = (ro_reg32_t)0x58004028U;
rw_reg32_t const RTC_RTC_SHIFTR   = (rw_reg32_t)0x5800402CU;
ro_reg32_t const RTC_RTC_TSTR     = (ro_reg32_t)0x58004030U;
ro_reg32_t const RTC_RTC_TSDR     = (ro_reg32_t)0x58004034U;
ro_reg32_t const RTC_RTC_TSSSR    = (ro_reg32_t)0x58004038U;
rw_reg32_t const RTC_RTC_CALR     = (rw_reg32_t)0x5800403CU;
rw_reg32_t const RTC_RTC_TAMPCR   = (rw_reg32_t)0x58004040U;
rw_reg32_t const RTC_RTC_ALRMASSR = (rw_reg32_t)0x58004044U;
rw_reg32_t const RTC_RTC_ALRMBSSR = (rw_reg32_t)0x58004048U;
rw_reg32_t const RTC_RTC_OR       = (rw_reg32_t)0x5800404CU;

/** @subsection Enumerated RTC Register Definitions */

rw_reg32_t const RTC_RTC_BKPxR[32] = {
  [0]  = (rw_reg32_t)0x58004050U,
  [1]  = (rw_reg32_t)0x58004054U,
  [2]  = (rw_reg32_t)0x58004058U,
  [3]  = (rw_reg32_t)0x5800405CU,
  [4]  = (rw_reg32_t)0x58004060U,
  [5]  = (rw_reg32_t)0x58004064U,
  [6]  = (rw_reg32_t)0x58004068U,
  [7]  = (rw_reg32_t)0x5800406CU,
  [8]  = (rw_reg32_t)0x58004070U,
  [9]  = (rw_reg32_t)0x58004074U,
  [10] = (rw_reg32_t)0x58004078U,
  [11] = (rw_reg32_t)0x5800407CU,
  [12] = (rw_reg32_t)0x58004080U,
  [13] = (rw_reg32_t)0x58004084U,
  [14] = (rw_reg32_t)0x58004088U,
  [15] = (rw_reg32_t)0x5800408CU,
  [16] = (rw_reg32_t)0x58004090U,
  [17] = (rw_reg32_t)0x58004094U,
  [18] = (rw_reg32_t)0x58004098U,
  [19] = (rw_reg32_t)0x5800409CU,
  [20] = (rw_reg32_t)0x580040A0U,
  [21] = (rw_reg32_t)0x580040A4U,
  [22] = (rw_reg32_t)0x580040A8U,
  [23] = (rw_reg32_t)0x580040ACU,
  [24] = (rw_reg32_t)0x580040B0U,
  [25] = (rw_reg32_t)0x580040B4U,
  [26] = (rw_reg32_t)0x580040B8U,
  [27] = (rw_reg32_t)0x580040BCU,
  [28] = (rw_reg32_t)0x580040C0U,
  [29] = (rw_reg32_t)0x580040C4U,
  [30] = (rw_reg32_t)0x580040C8U,
  [31] = (rw_reg32_t)0x580040CCU,
};

/** @subsection RTC Register Field Definitions */

const field32_t RTC_RTC_TR_SU             = {.msk = 0x0000000FU, .pos = 0};
const field32_t RTC_RTC_TR_ST             = {.msk = 0x00000070U, .pos = 4};
const field32_t RTC_RTC_TR_MNU            = {.msk = 0x00000F00U, .pos = 8};
const field32_t RTC_RTC_TR_MNT            = {.msk = 0x00007000U, .pos = 12};
const field32_t RTC_RTC_TR_HU             = {.msk = 0x000F0000U, .pos = 16};
const field32_t RTC_RTC_TR_HT             = {.msk = 0x00300000U, .pos = 20};
const field32_t RTC_RTC_TR_PM             = {.msk = 0x00400000U, .pos = 22};
const field32_t RTC_RTC_DR_DU             = {.msk = 0x0000000FU, .pos = 0};
const field32_t RTC_RTC_DR_DT             = {.msk = 0x00000030U, .pos = 4};
const field32_t RTC_RTC_DR_MU             = {.msk = 0x00000F00U, .pos = 8};
const field32_t RTC_RTC_DR_MT             = {.msk = 0x00001000U, .pos = 12};
const field32_t RTC_RTC_DR_WDU            = {.msk = 0x0000E000U, .pos = 13};
const field32_t RTC_RTC_DR_YU             = {.msk = 0x000F0000U, .pos = 16};
const field32_t RTC_RTC_DR_YT             = {.msk = 0x00F00000U, .pos = 20};
const field32_t RTC_RTC_CR_WUCKSEL        = {.msk = 0x00000007U, .pos = 0};
const field32_t RTC_RTC_CR_TSEDGE         = {.msk = 0x00000008U, .pos = 3};
const field32_t RTC_RTC_CR_REFCKON        = {.msk = 0x00000010U, .pos = 4};
const field32_t RTC_RTC_CR_BYPSHAD        = {.msk = 0x00000020U, .pos = 5};
const field32_t RTC_RTC_CR_FMT            = {.msk = 0x00000040U, .pos = 6};
const field32_t RTC_RTC_CR_ALRAE          = {.msk = 0x00000100U, .pos = 8};
const field32_t RTC_RTC_CR_ALRBE          = {.msk = 0x00000200U, .pos = 9};
const field32_t RTC_RTC_CR_WUTE           = {.msk = 0x00000400U, .pos = 10};
const field32_t RTC_RTC_CR_TSE            = {.msk = 0x00000800U, .pos = 11};
const field32_t RTC_RTC_CR_ALRAIE         = {.msk = 0x00001000U, .pos = 12};
const field32_t RTC_RTC_CR_ALRBIE         = {.msk = 0x00002000U, .pos = 13};
const field32_t RTC_RTC_CR_WUTIE          = {.msk = 0x00004000U, .pos = 14};
const field32_t RTC_RTC_CR_TSIE           = {.msk = 0x00008000U, .pos = 15};
const field32_t RTC_RTC_CR_ADD1H          = {.msk = 0x00010000U, .pos = 16};
const field32_t RTC_RTC_CR_SUB1H          = {.msk = 0x00020000U, .pos = 17};
const field32_t RTC_RTC_CR_BKP            = {.msk = 0x00040000U, .pos = 18};
const field32_t RTC_RTC_CR_COSEL          = {.msk = 0x00080000U, .pos = 19};
const field32_t RTC_RTC_CR_POL            = {.msk = 0x00100000U, .pos = 20};
const field32_t RTC_RTC_CR_OSEL           = {.msk = 0x00600000U, .pos = 21};
const field32_t RTC_RTC_CR_COE            = {.msk = 0x00800000U, .pos = 23};
const field32_t RTC_RTC_CR_ITSE           = {.msk = 0x01000000U, .pos = 24};
const field32_t RTC_RTC_ISR_ALRAWF        = {.msk = 0x00000001U, .pos = 0};
const field32_t RTC_RTC_ISR_ALRBWF        = {.msk = 0x00000002U, .pos = 1};
const field32_t RTC_RTC_ISR_WUTWF         = {.msk = 0x00000004U, .pos = 2};
const field32_t RTC_RTC_ISR_SHPF          = {.msk = 0x00000008U, .pos = 3};
const field32_t RTC_RTC_ISR_INITS         = {.msk = 0x00000010U, .pos = 4};
const field32_t RTC_RTC_ISR_RSF           = {.msk = 0x00000020U, .pos = 5};
const field32_t RTC_RTC_ISR_INITF         = {.msk = 0x00000040U, .pos = 6};
const field32_t RTC_RTC_ISR_INIT          = {.msk = 0x00000080U, .pos = 7};
const field32_t RTC_RTC_ISR_ALRAF         = {.msk = 0x00000100U, .pos = 8};
const field32_t RTC_RTC_ISR_ALRBF         = {.msk = 0x00000200U, .pos = 9};
const field32_t RTC_RTC_ISR_WUTF          = {.msk = 0x00000400U, .pos = 10};
const field32_t RTC_RTC_ISR_TSF           = {.msk = 0x00000800U, .pos = 11};
const field32_t RTC_RTC_ISR_TSOVF         = {.msk = 0x00001000U, .pos = 12};
const field32_t RTC_RTC_ISR_RECALPF       = {.msk = 0x00010000U, .pos = 16};
const field32_t RTC_RTC_ISR_ITSF          = {.msk = 0x00020000U, .pos = 17};
const field32_t RTC_RTC_PRER_PREDIV_S     = {.msk = 0x00007FFFU, .pos = 0};
const field32_t RTC_RTC_PRER_PREDIV_A     = {.msk = 0x007F0000U, .pos = 16};
const field32_t RTC_RTC_WUTR_WUT          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t RTC_RTC_ALRMAR_SU         = {.msk = 0x0000000FU, .pos = 0};
const field32_t RTC_RTC_ALRMAR_ST         = {.msk = 0x00000070U, .pos = 4};
const field32_t RTC_RTC_ALRMAR_MNU        = {.msk = 0x00000F00U, .pos = 8};
const field32_t RTC_RTC_ALRMAR_MNT        = {.msk = 0x00007000U, .pos = 12};
const field32_t RTC_RTC_ALRMAR_HU         = {.msk = 0x000F0000U, .pos = 16};
const field32_t RTC_RTC_ALRMAR_HT         = {.msk = 0x00300000U, .pos = 20};
const field32_t RTC_RTC_ALRMAR_PM         = {.msk = 0x00400000U, .pos = 22};
const field32_t RTC_RTC_ALRMAR_DU         = {.msk = 0x0F000000U, .pos = 24};
const field32_t RTC_RTC_ALRMAR_DT         = {.msk = 0x30000000U, .pos = 28};
const field32_t RTC_RTC_ALRMAR_WDSEL      = {.msk = 0x40000000U, .pos = 30};
const field32_t RTC_RTC_ALRMBR_SU         = {.msk = 0x0000000FU, .pos = 0};
const field32_t RTC_RTC_ALRMBR_ST         = {.msk = 0x00000070U, .pos = 4};
const field32_t RTC_RTC_ALRMBR_MNU        = {.msk = 0x00000F00U, .pos = 8};
const field32_t RTC_RTC_ALRMBR_MNT        = {.msk = 0x00007000U, .pos = 12};
const field32_t RTC_RTC_ALRMBR_HU         = {.msk = 0x000F0000U, .pos = 16};
const field32_t RTC_RTC_ALRMBR_HT         = {.msk = 0x00300000U, .pos = 20};
const field32_t RTC_RTC_ALRMBR_PM         = {.msk = 0x00400000U, .pos = 22};
const field32_t RTC_RTC_ALRMBR_DU         = {.msk = 0x0F000000U, .pos = 24};
const field32_t RTC_RTC_ALRMBR_DT         = {.msk = 0x30000000U, .pos = 28};
const field32_t RTC_RTC_ALRMBR_WDSEL      = {.msk = 0x40000000U, .pos = 30};
const field32_t RTC_RTC_WPR_KEY           = {.msk = 0x000000FFU, .pos = 0};
const field32_t RTC_RTC_SSR_SS            = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t RTC_RTC_SHIFTR_SUBFS      = {.msk = 0x00007FFFU, .pos = 0};
const field32_t RTC_RTC_SHIFTR_ADD1S      = {.msk = 0x80000000U, .pos = 31};
const field32_t RTC_RTC_TSTR_SU           = {.msk = 0x0000000FU, .pos = 0};
const field32_t RTC_RTC_TSTR_ST           = {.msk = 0x00000070U, .pos = 4};
const field32_t RTC_RTC_TSTR_MNU          = {.msk = 0x00000F00U, .pos = 8};
const field32_t RTC_RTC_TSTR_MNT          = {.msk = 0x00007000U, .pos = 12};
const field32_t RTC_RTC_TSTR_HU           = {.msk = 0x000F0000U, .pos = 16};
const field32_t RTC_RTC_TSTR_HT           = {.msk = 0x00300000U, .pos = 20};
const field32_t RTC_RTC_TSTR_PM           = {.msk = 0x00400000U, .pos = 22};
const field32_t RTC_RTC_TSDR_DU           = {.msk = 0x0000000FU, .pos = 0};
const field32_t RTC_RTC_TSDR_DT           = {.msk = 0x00000030U, .pos = 4};
const field32_t RTC_RTC_TSDR_MU           = {.msk = 0x00000F00U, .pos = 8};
const field32_t RTC_RTC_TSDR_MT           = {.msk = 0x00001000U, .pos = 12};
const field32_t RTC_RTC_TSDR_WDU          = {.msk = 0x0000E000U, .pos = 13};
const field32_t RTC_RTC_TSSSR_SS          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t RTC_RTC_CALR_CALM         = {.msk = 0x000001FFU, .pos = 0};
const field32_t RTC_RTC_CALR_CALP         = {.msk = 0x00008000U, .pos = 15};
const field32_t RTC_RTC_TAMPCR_TAMPIE     = {.msk = 0x00000004U, .pos = 2};
const field32_t RTC_RTC_TAMPCR_TAMPTS     = {.msk = 0x00000080U, .pos = 7};
const field32_t RTC_RTC_TAMPCR_TAMPFREQ   = {.msk = 0x00000700U, .pos = 8};
const field32_t RTC_RTC_TAMPCR_TAMPFLT    = {.msk = 0x00001800U, .pos = 11};
const field32_t RTC_RTC_TAMPCR_TAMPPRCH   = {.msk = 0x00006000U, .pos = 13};
const field32_t RTC_RTC_TAMPCR_TAMPPUDIS  = {.msk = 0x00008000U, .pos = 15};
const field32_t RTC_RTC_ALRMASSR_SS       = {.msk = 0x00007FFFU, .pos = 0};
const field32_t RTC_RTC_ALRMASSR_MASKSS   = {.msk = 0x0F000000U, .pos = 24};
const field32_t RTC_RTC_ALRMBSSR_SS       = {.msk = 0x00007FFFU, .pos = 0};
const field32_t RTC_RTC_ALRMBSSR_MASKSS   = {.msk = 0x0F000000U, .pos = 24};
const field32_t RTC_RTC_OR_RTC_ALARM_TYPE = {.msk = 0x00000001U, .pos = 0};
const field32_t RTC_RTC_OR_RTC_OUT_RMP    = {.msk = 0x00000002U, .pos = 1};

/** @subsection Enumerated RTC Register Field Definitions */

field32_t const RTC_RTC_ISR_TAMPxF[4] = {
  [1] = {.msk = 0x00002000U, .pos = 13},
  [2] = {.msk = 0x00004000U, .pos = 14},
  [3] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const RTC_RTC_ALRMAR_MSKx[5] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00008000U, .pos = 15},
  [3] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const RTC_RTC_CALR_CALWx[17] = {
  [8]  = {.msk = 0x00004000U, .pos = 14},
  [16] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const RTC_RTC_TAMPCR_TAMPxE[4] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000008U, .pos = 3},
  [3] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const RTC_RTC_TAMPCR_TAMPxTRG[4] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000040U, .pos = 6},
};

field32_t const RTC_RTC_TAMPCR_TAMPxIE[4] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00080000U, .pos = 19},
  [3] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const RTC_RTC_TAMPCR_TAMPxNOERASE[4] = {
  [1] = {.msk = 0x00020000U, .pos = 17},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [3] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const RTC_RTC_TAMPCR_TAMPxMF[4] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00200000U, .pos = 21},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

/**************************************************************************************************
 * @section SAIx Definitions
 **************************************************************************************************/

/** @subsection Enumerated SAIx Register Definitions */

rw_reg32_t const SAIx_SAI_GCR[5] = {
  [1] = (rw_reg32_t)0x40015800U,
  [2] = (rw_reg32_t)0x40015C00U,
  [3] = (rw_reg32_t)0x40016000U,
  [4] = (rw_reg32_t)0x58005400U,
};

rw_reg32_t const SAIx_SAI_ACR1[5] = {
  [1] = (rw_reg32_t)0x40015804U,
  [2] = (rw_reg32_t)0x40015C04U,
  [3] = (rw_reg32_t)0x40016004U,
  [4] = (rw_reg32_t)0x58005404U,
};

rw_reg32_t const SAIx_SAI_ACR2[5] = {
  [1] = (rw_reg32_t)0x40015808U,
  [2] = (rw_reg32_t)0x40015C08U,
  [3] = (rw_reg32_t)0x40016008U,
  [4] = (rw_reg32_t)0x58005408U,
};

rw_reg32_t const SAIx_SAI_AFRCR[5] = {
  [1] = (rw_reg32_t)0x4001580CU,
  [2] = (rw_reg32_t)0x40015C0CU,
  [3] = (rw_reg32_t)0x4001600CU,
  [4] = (rw_reg32_t)0x5800540CU,
};

rw_reg32_t const SAIx_SAI_ASLOTR[5] = {
  [1] = (rw_reg32_t)0x40015810U,
  [2] = (rw_reg32_t)0x40015C10U,
  [3] = (rw_reg32_t)0x40016010U,
  [4] = (rw_reg32_t)0x58005410U,
};

rw_reg32_t const SAIx_SAI_AIM[5] = {
  [1] = (rw_reg32_t)0x40015814U,
  [2] = (rw_reg32_t)0x40015C14U,
  [3] = (rw_reg32_t)0x40016014U,
  [4] = (rw_reg32_t)0x58005414U,
};

ro_reg32_t const SAIx_SAI_ASR[5] = {
  [1] = (ro_reg32_t)0x40015818U,
  [2] = (ro_reg32_t)0x40015C18U,
  [3] = (ro_reg32_t)0x40016018U,
  [4] = (ro_reg32_t)0x58005418U,
};

rw_reg32_t const SAIx_SAI_ACLRFR[5] = {
  [1] = (rw_reg32_t)0x4001581CU,
  [2] = (rw_reg32_t)0x40015C1CU,
  [3] = (rw_reg32_t)0x4001601CU,
  [4] = (rw_reg32_t)0x5800541CU,
};

rw_reg32_t const SAIx_SAI_ADR[5] = {
  [1] = (rw_reg32_t)0x40015820U,
  [2] = (rw_reg32_t)0x40015C20U,
  [3] = (rw_reg32_t)0x40016020U,
  [4] = (rw_reg32_t)0x58005420U,
};

rw_reg32_t const SAIx_SAI_BCR1[5] = {
  [1] = (rw_reg32_t)0x40015824U,
  [2] = (rw_reg32_t)0x40015C24U,
  [3] = (rw_reg32_t)0x40016024U,
  [4] = (rw_reg32_t)0x58005424U,
};

rw_reg32_t const SAIx_SAI_BCR2[5] = {
  [1] = (rw_reg32_t)0x40015828U,
  [2] = (rw_reg32_t)0x40015C28U,
  [3] = (rw_reg32_t)0x40016028U,
  [4] = (rw_reg32_t)0x58005428U,
};

rw_reg32_t const SAIx_SAI_BFRCR[5] = {
  [1] = (rw_reg32_t)0x4001582CU,
  [2] = (rw_reg32_t)0x40015C2CU,
  [3] = (rw_reg32_t)0x4001602CU,
  [4] = (rw_reg32_t)0x5800542CU,
};

rw_reg32_t const SAIx_SAI_BSLOTR[5] = {
  [1] = (rw_reg32_t)0x40015830U,
  [2] = (rw_reg32_t)0x40015C30U,
  [3] = (rw_reg32_t)0x40016030U,
  [4] = (rw_reg32_t)0x58005430U,
};

rw_reg32_t const SAIx_SAI_BIM[5] = {
  [1] = (rw_reg32_t)0x40015834U,
  [2] = (rw_reg32_t)0x40015C34U,
  [3] = (rw_reg32_t)0x40016034U,
  [4] = (rw_reg32_t)0x58005434U,
};

ro_reg32_t const SAIx_SAI_BSR[5] = {
  [1] = (ro_reg32_t)0x40015838U,
  [2] = (ro_reg32_t)0x40015C38U,
  [3] = (ro_reg32_t)0x40016038U,
  [4] = (ro_reg32_t)0x58005438U,
};

rw_reg32_t const SAIx_SAI_BCLRFR[5] = {
  [1] = (rw_reg32_t)0x4001583CU,
  [2] = (rw_reg32_t)0x40015C3CU,
  [3] = (rw_reg32_t)0x4001603CU,
  [4] = (rw_reg32_t)0x5800543CU,
};

rw_reg32_t const SAIx_SAI_BDR[5] = {
  [1] = (rw_reg32_t)0x40015840U,
  [2] = (rw_reg32_t)0x40015C40U,
  [3] = (rw_reg32_t)0x40016040U,
  [4] = (rw_reg32_t)0x58005440U,
};

rw_reg32_t const SAIx_SAI_PDMCR[5] = {
  [1] = (rw_reg32_t)0x40015844U,
  [2] = (rw_reg32_t)0x40015C44U,
  [3] = (rw_reg32_t)0x40016044U,
  [4] = (rw_reg32_t)0x58005444U,
};

rw_reg32_t const SAIx_SAI_PDMDLY[5] = {
  [1] = (rw_reg32_t)0x40015848U,
  [2] = (rw_reg32_t)0x40015C48U,
  [3] = (rw_reg32_t)0x40016048U,
  [4] = (rw_reg32_t)0x58005448U,
};

/** @subsection SAIx Register Field Definitions */

const field32_t SAIx_SAI_GCR_SYNCOUT     = {.msk = 0x00000030U, .pos = 4};
const field32_t SAIx_SAI_GCR_SYNCIN      = {.msk = 0x00000003U, .pos = 0};
const field32_t SAIx_SAI_ACR1_MODE       = {.msk = 0x00000003U, .pos = 0};
const field32_t SAIx_SAI_ACR1_PRTCFG     = {.msk = 0x0000000CU, .pos = 2};
const field32_t SAIx_SAI_ACR1_DS         = {.msk = 0x000000E0U, .pos = 5};
const field32_t SAIx_SAI_ACR1_LSBFIRST   = {.msk = 0x00000100U, .pos = 8};
const field32_t SAIx_SAI_ACR1_CKSTR      = {.msk = 0x00000200U, .pos = 9};
const field32_t SAIx_SAI_ACR1_SYNCEN     = {.msk = 0x00000C00U, .pos = 10};
const field32_t SAIx_SAI_ACR1_MONO       = {.msk = 0x00001000U, .pos = 12};
const field32_t SAIx_SAI_ACR1_OUTDRIV    = {.msk = 0x00002000U, .pos = 13};
const field32_t SAIx_SAI_ACR1_SAIXEN     = {.msk = 0x00010000U, .pos = 16};
const field32_t SAIx_SAI_ACR1_DMAEN      = {.msk = 0x00020000U, .pos = 17};
const field32_t SAIx_SAI_ACR1_NOMCK      = {.msk = 0x00080000U, .pos = 19};
const field32_t SAIx_SAI_ACR1_MCKDIV     = {.msk = 0x00F00000U, .pos = 20};
const field32_t SAIx_SAI_ACR1_OSR        = {.msk = 0x04000000U, .pos = 26};
const field32_t SAIx_SAI_ACR2_FTH        = {.msk = 0x00000007U, .pos = 0};
const field32_t SAIx_SAI_ACR2_FFLUSH     = {.msk = 0x00000008U, .pos = 3};
const field32_t SAIx_SAI_ACR2_TRIS       = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_ACR2_MUTE       = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_ACR2_MUTEVAL    = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_ACR2_MUTECNT    = {.msk = 0x00001F80U, .pos = 7};
const field32_t SAIx_SAI_ACR2_CPL        = {.msk = 0x00002000U, .pos = 13};
const field32_t SAIx_SAI_ACR2_COMP       = {.msk = 0x0000C000U, .pos = 14};
const field32_t SAIx_SAI_AFRCR_FRL       = {.msk = 0x000000FFU, .pos = 0};
const field32_t SAIx_SAI_AFRCR_FSALL     = {.msk = 0x00007F00U, .pos = 8};
const field32_t SAIx_SAI_AFRCR_FSDEF     = {.msk = 0x00010000U, .pos = 16};
const field32_t SAIx_SAI_AFRCR_FSPOL     = {.msk = 0x00020000U, .pos = 17};
const field32_t SAIx_SAI_AFRCR_FSOFF     = {.msk = 0x00040000U, .pos = 18};
const field32_t SAIx_SAI_ASLOTR_FBOFF    = {.msk = 0x0000001FU, .pos = 0};
const field32_t SAIx_SAI_ASLOTR_SLOTSZ   = {.msk = 0x000000C0U, .pos = 6};
const field32_t SAIx_SAI_ASLOTR_NBSLOT   = {.msk = 0x00000F00U, .pos = 8};
const field32_t SAIx_SAI_ASLOTR_SLOTEN   = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SAIx_SAI_AIM_OVRUDRIE    = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_AIM_MUTEDETIE   = {.msk = 0x00000002U, .pos = 1};
const field32_t SAIx_SAI_AIM_WCKCFGIE    = {.msk = 0x00000004U, .pos = 2};
const field32_t SAIx_SAI_AIM_FREQIE      = {.msk = 0x00000008U, .pos = 3};
const field32_t SAIx_SAI_AIM_CNRDYIE     = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_AIM_AFSDETIE    = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_AIM_LFSDETIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_ASR_OVRUDR      = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_ASR_MUTEDET     = {.msk = 0x00000002U, .pos = 1};
const field32_t SAIx_SAI_ASR_WCKCFG      = {.msk = 0x00000004U, .pos = 2};
const field32_t SAIx_SAI_ASR_FREQ        = {.msk = 0x00000008U, .pos = 3};
const field32_t SAIx_SAI_ASR_CNRDY       = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_ASR_AFSDET      = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_ASR_LFSDET      = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_ASR_FLVL        = {.msk = 0x00070000U, .pos = 16};
const field32_t SAIx_SAI_ACLRFR_COVRUDR  = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_ACLRFR_CMUTEDET = {.msk = 0x00000002U, .pos = 1};
const field32_t SAIx_SAI_ACLRFR_CWCKCFG  = {.msk = 0x00000004U, .pos = 2};
const field32_t SAIx_SAI_ACLRFR_CCNRDY   = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_ACLRFR_CAFSDET  = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_ACLRFR_CLFSDET  = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_BCR1_MODE       = {.msk = 0x00000003U, .pos = 0};
const field32_t SAIx_SAI_BCR1_PRTCFG     = {.msk = 0x0000000CU, .pos = 2};
const field32_t SAIx_SAI_BCR1_DS         = {.msk = 0x000000E0U, .pos = 5};
const field32_t SAIx_SAI_BCR1_LSBFIRST   = {.msk = 0x00000100U, .pos = 8};
const field32_t SAIx_SAI_BCR1_CKSTR      = {.msk = 0x00000200U, .pos = 9};
const field32_t SAIx_SAI_BCR1_SYNCEN     = {.msk = 0x00000C00U, .pos = 10};
const field32_t SAIx_SAI_BCR1_MONO       = {.msk = 0x00001000U, .pos = 12};
const field32_t SAIx_SAI_BCR1_OUTDRIV    = {.msk = 0x00002000U, .pos = 13};
const field32_t SAIx_SAI_BCR1_SAIXEN     = {.msk = 0x00010000U, .pos = 16};
const field32_t SAIx_SAI_BCR1_DMAEN      = {.msk = 0x00020000U, .pos = 17};
const field32_t SAIx_SAI_BCR1_NOMCK      = {.msk = 0x00080000U, .pos = 19};
const field32_t SAIx_SAI_BCR1_MCKDIV     = {.msk = 0x00F00000U, .pos = 20};
const field32_t SAIx_SAI_BCR1_OSR        = {.msk = 0x04000000U, .pos = 26};
const field32_t SAIx_SAI_BCR2_FTH        = {.msk = 0x00000007U, .pos = 0};
const field32_t SAIx_SAI_BCR2_FFLUSH     = {.msk = 0x00000008U, .pos = 3};
const field32_t SAIx_SAI_BCR2_TRIS       = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_BCR2_MUTE       = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_BCR2_MUTEVAL    = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_BCR2_MUTECNT    = {.msk = 0x00001F80U, .pos = 7};
const field32_t SAIx_SAI_BCR2_CPL        = {.msk = 0x00002000U, .pos = 13};
const field32_t SAIx_SAI_BCR2_COMP       = {.msk = 0x0000C000U, .pos = 14};
const field32_t SAIx_SAI_BFRCR_FRL       = {.msk = 0x000000FFU, .pos = 0};
const field32_t SAIx_SAI_BFRCR_FSALL     = {.msk = 0x00007F00U, .pos = 8};
const field32_t SAIx_SAI_BFRCR_FSDEF     = {.msk = 0x00010000U, .pos = 16};
const field32_t SAIx_SAI_BFRCR_FSPOL     = {.msk = 0x00020000U, .pos = 17};
const field32_t SAIx_SAI_BFRCR_FSOFF     = {.msk = 0x00040000U, .pos = 18};
const field32_t SAIx_SAI_BSLOTR_FBOFF    = {.msk = 0x0000001FU, .pos = 0};
const field32_t SAIx_SAI_BSLOTR_SLOTSZ   = {.msk = 0x000000C0U, .pos = 6};
const field32_t SAIx_SAI_BSLOTR_NBSLOT   = {.msk = 0x00000F00U, .pos = 8};
const field32_t SAIx_SAI_BSLOTR_SLOTEN   = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SAIx_SAI_BIM_OVRUDRIE    = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_BIM_MUTEDETIE   = {.msk = 0x00000002U, .pos = 1};
const field32_t SAIx_SAI_BIM_WCKCFGIE    = {.msk = 0x00000004U, .pos = 2};
const field32_t SAIx_SAI_BIM_FREQIE      = {.msk = 0x00000008U, .pos = 3};
const field32_t SAIx_SAI_BIM_CNRDYIE     = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_BIM_AFSDETIE    = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_BIM_LFSDETIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_BSR_OVRUDR      = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_BSR_MUTEDET     = {.msk = 0x00000002U, .pos = 1};
const field32_t SAIx_SAI_BSR_WCKCFG      = {.msk = 0x00000004U, .pos = 2};
const field32_t SAIx_SAI_BSR_FREQ        = {.msk = 0x00000008U, .pos = 3};
const field32_t SAIx_SAI_BSR_CNRDY       = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_BSR_AFSDET      = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_BSR_LFSDET      = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_BSR_FLVL        = {.msk = 0x00070000U, .pos = 16};
const field32_t SAIx_SAI_BCLRFR_COVRUDR  = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_BCLRFR_CMUTEDET = {.msk = 0x00000002U, .pos = 1};
const field32_t SAIx_SAI_BCLRFR_CWCKCFG  = {.msk = 0x00000004U, .pos = 2};
const field32_t SAIx_SAI_BCLRFR_CCNRDY   = {.msk = 0x00000010U, .pos = 4};
const field32_t SAIx_SAI_BCLRFR_CAFSDET  = {.msk = 0x00000020U, .pos = 5};
const field32_t SAIx_SAI_BCLRFR_CLFSDET  = {.msk = 0x00000040U, .pos = 6};
const field32_t SAIx_SAI_PDMCR_PDMEN     = {.msk = 0x00000001U, .pos = 0};
const field32_t SAIx_SAI_PDMCR_MICNBR    = {.msk = 0x00000030U, .pos = 4};

/** @subsection Enumerated SAIx Register Field Definitions */

field32_t const SAIx_SAI_PDMCR_CKENx[5] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const SAIx_SAI_PDMDLY_DLYMxL[5] = {
  [1] = {.msk = 0x00000007U, .pos = 0},
  [2] = {.msk = 0x00000700U, .pos = 8},
  [3] = {.msk = 0x00070000U, .pos = 16},
  [4] = {.msk = 0x07000000U, .pos = 24},
};

field32_t const SAIx_SAI_PDMDLY_DLYMxR[5] = {
  [1] = {.msk = 0x00000070U, .pos = 4},
  [2] = {.msk = 0x00007000U, .pos = 12},
  [3] = {.msk = 0x00700000U, .pos = 20},
  [4] = {.msk = 0x70000000U, .pos = 28},
};

/**************************************************************************************************
 * @section SDMMCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated SDMMCx Register Definitions */

rw_reg32_t const SDMMCx_POWER[3] = {
  [1] = (rw_reg32_t)0x52007000U,
  [2] = (rw_reg32_t)0x48022400U,
};

rw_reg32_t const SDMMCx_CLKCR[3] = {
  [1] = (rw_reg32_t)0x52007004U,
  [2] = (rw_reg32_t)0x48022404U,
};

rw_reg32_t const SDMMCx_ARGR[3] = {
  [1] = (rw_reg32_t)0x52007008U,
  [2] = (rw_reg32_t)0x48022408U,
};

rw_reg32_t const SDMMCx_CMDR[3] = {
  [1] = (rw_reg32_t)0x5200700CU,
  [2] = (rw_reg32_t)0x4802240CU,
};

ro_reg32_t const SDMMCx_RESP1R[3] = {
  [1] = (ro_reg32_t)0x52007014U,
  [2] = (ro_reg32_t)0x48022414U,
};

ro_reg32_t const SDMMCx_RESP2R[3] = {
  [1] = (ro_reg32_t)0x52007018U,
  [2] = (ro_reg32_t)0x48022418U,
};

ro_reg32_t const SDMMCx_RESP3R[3] = {
  [1] = (ro_reg32_t)0x5200701CU,
  [2] = (ro_reg32_t)0x4802241CU,
};

ro_reg32_t const SDMMCx_RESP4R[3] = {
  [1] = (ro_reg32_t)0x52007020U,
  [2] = (ro_reg32_t)0x48022420U,
};

rw_reg32_t const SDMMCx_DTIMER[3] = {
  [1] = (rw_reg32_t)0x52007024U,
  [2] = (rw_reg32_t)0x48022424U,
};

rw_reg32_t const SDMMCx_DLENR[3] = {
  [1] = (rw_reg32_t)0x52007028U,
  [2] = (rw_reg32_t)0x48022428U,
};

rw_reg32_t const SDMMCx_DCTRL[3] = {
  [1] = (rw_reg32_t)0x5200702CU,
  [2] = (rw_reg32_t)0x4802242CU,
};

ro_reg32_t const SDMMCx_DCNTR[3] = {
  [1] = (ro_reg32_t)0x52007030U,
  [2] = (ro_reg32_t)0x48022430U,
};

ro_reg32_t const SDMMCx_STAR[3] = {
  [1] = (ro_reg32_t)0x52007034U,
  [2] = (ro_reg32_t)0x48022434U,
};

rw_reg32_t const SDMMCx_ICR[3] = {
  [1] = (rw_reg32_t)0x52007038U,
  [2] = (rw_reg32_t)0x48022438U,
};

rw_reg32_t const SDMMCx_MASKR[3] = {
  [1] = (rw_reg32_t)0x5200703CU,
  [2] = (rw_reg32_t)0x4802243CU,
};

rw_reg32_t const SDMMCx_ACKTIMER[3] = {
  [1] = (rw_reg32_t)0x52007040U,
  [2] = (rw_reg32_t)0x48022440U,
};

rw_reg32_t const SDMMCx_IDMACTRLR[3] = {
  [1] = (rw_reg32_t)0x52007050U,
  [2] = (rw_reg32_t)0x48022450U,
};

rw_reg32_t const SDMMCx_IDMABSIZER[3] = {
  [1] = (rw_reg32_t)0x52007054U,
  [2] = (rw_reg32_t)0x48022454U,
};

rw_reg32_t const SDMMCx_IDMABASE0R[3] = {
  [1] = (rw_reg32_t)0x52007058U,
  [2] = (rw_reg32_t)0x48022458U,
};

rw_reg32_t const SDMMCx_IDMABASE1R[3] = {
  [1] = (rw_reg32_t)0x5200705CU,
  [2] = (rw_reg32_t)0x4802245CU,
};

rw_reg32_t const SDMMCx_FIFOR[3] = {
  [1] = (rw_reg32_t)0x52007080U,
  [2] = (rw_reg32_t)0x48022480U,
};

ro_reg32_t const SDMMCx_RESPCMDR[3] = {
  [1] = (ro_reg32_t)0x52007010U,
  [2] = (ro_reg32_t)0x48022410U,
};

/** @subsection SDMMCx Register Field Definitions */

const field32_t SDMMCx_POWER_PWRCTRL       = {.msk = 0x00000003U, .pos = 0};
const field32_t SDMMCx_POWER_VSWITCH       = {.msk = 0x00000004U, .pos = 2};
const field32_t SDMMCx_POWER_VSWITCHEN     = {.msk = 0x00000008U, .pos = 3};
const field32_t SDMMCx_POWER_DIRPOL        = {.msk = 0x00000010U, .pos = 4};
const field32_t SDMMCx_CLKCR_CLKDIV        = {.msk = 0x000003FFU, .pos = 0};
const field32_t SDMMCx_CLKCR_PWRSAV        = {.msk = 0x00001000U, .pos = 12};
const field32_t SDMMCx_CLKCR_WIDBUS        = {.msk = 0x0000C000U, .pos = 14};
const field32_t SDMMCx_CLKCR_NEGEDGE       = {.msk = 0x00010000U, .pos = 16};
const field32_t SDMMCx_CLKCR_HWFC_EN       = {.msk = 0x00020000U, .pos = 17};
const field32_t SDMMCx_CLKCR_DDR           = {.msk = 0x00040000U, .pos = 18};
const field32_t SDMMCx_CLKCR_BUSSPEED      = {.msk = 0x00080000U, .pos = 19};
const field32_t SDMMCx_CLKCR_SELCLKRX      = {.msk = 0x00300000U, .pos = 20};
const field32_t SDMMCx_CMDR_CMDINDEX       = {.msk = 0x0000003FU, .pos = 0};
const field32_t SDMMCx_CMDR_CMDTRANS       = {.msk = 0x00000040U, .pos = 6};
const field32_t SDMMCx_CMDR_CMDSTOP        = {.msk = 0x00000080U, .pos = 7};
const field32_t SDMMCx_CMDR_WAITRESP       = {.msk = 0x00000300U, .pos = 8};
const field32_t SDMMCx_CMDR_WAITINT        = {.msk = 0x00000400U, .pos = 10};
const field32_t SDMMCx_CMDR_WAITPEND       = {.msk = 0x00000800U, .pos = 11};
const field32_t SDMMCx_CMDR_CPSMEN         = {.msk = 0x00001000U, .pos = 12};
const field32_t SDMMCx_CMDR_DTHOLD         = {.msk = 0x00002000U, .pos = 13};
const field32_t SDMMCx_CMDR_BOOTMODE       = {.msk = 0x00004000U, .pos = 14};
const field32_t SDMMCx_CMDR_BOOTEN         = {.msk = 0x00008000U, .pos = 15};
const field32_t SDMMCx_CMDR_CMDSUSPEND     = {.msk = 0x00010000U, .pos = 16};
const field32_t SDMMCx_DLENR_DATALENGTH    = {.msk = 0x01FFFFFFU, .pos = 0};
const field32_t SDMMCx_DCTRL_DTEN          = {.msk = 0x00000001U, .pos = 0};
const field32_t SDMMCx_DCTRL_DTDIR         = {.msk = 0x00000002U, .pos = 1};
const field32_t SDMMCx_DCTRL_DTMODE        = {.msk = 0x0000000CU, .pos = 2};
const field32_t SDMMCx_DCTRL_DBLOCKSIZE    = {.msk = 0x000000F0U, .pos = 4};
const field32_t SDMMCx_DCTRL_RWSTART       = {.msk = 0x00000100U, .pos = 8};
const field32_t SDMMCx_DCTRL_RWSTOP        = {.msk = 0x00000200U, .pos = 9};
const field32_t SDMMCx_DCTRL_RWMOD         = {.msk = 0x00000400U, .pos = 10};
const field32_t SDMMCx_DCTRL_SDIOEN        = {.msk = 0x00000800U, .pos = 11};
const field32_t SDMMCx_DCTRL_BOOTACKEN     = {.msk = 0x00001000U, .pos = 12};
const field32_t SDMMCx_DCTRL_FIFORST       = {.msk = 0x00002000U, .pos = 13};
const field32_t SDMMCx_DCNTR_DATACOUNT     = {.msk = 0x01FFFFFFU, .pos = 0};
const field32_t SDMMCx_STAR_CCRCFAIL       = {.msk = 0x00000001U, .pos = 0};
const field32_t SDMMCx_STAR_DCRCFAIL       = {.msk = 0x00000002U, .pos = 1};
const field32_t SDMMCx_STAR_CTIMEOUT       = {.msk = 0x00000004U, .pos = 2};
const field32_t SDMMCx_STAR_DTIMEOUT       = {.msk = 0x00000008U, .pos = 3};
const field32_t SDMMCx_STAR_TXUNDERR       = {.msk = 0x00000010U, .pos = 4};
const field32_t SDMMCx_STAR_RXOVERR        = {.msk = 0x00000020U, .pos = 5};
const field32_t SDMMCx_STAR_CMDREND        = {.msk = 0x00000040U, .pos = 6};
const field32_t SDMMCx_STAR_CMDSENT        = {.msk = 0x00000080U, .pos = 7};
const field32_t SDMMCx_STAR_DATAEND        = {.msk = 0x00000100U, .pos = 8};
const field32_t SDMMCx_STAR_DHOLD          = {.msk = 0x00000200U, .pos = 9};
const field32_t SDMMCx_STAR_DBCKEND        = {.msk = 0x00000400U, .pos = 10};
const field32_t SDMMCx_STAR_DABORT         = {.msk = 0x00000800U, .pos = 11};
const field32_t SDMMCx_STAR_DPSMACT        = {.msk = 0x00001000U, .pos = 12};
const field32_t SDMMCx_STAR_CPSMACT        = {.msk = 0x00002000U, .pos = 13};
const field32_t SDMMCx_STAR_TXFIFOHE       = {.msk = 0x00004000U, .pos = 14};
const field32_t SDMMCx_STAR_RXFIFOHF       = {.msk = 0x00008000U, .pos = 15};
const field32_t SDMMCx_STAR_TXFIFOF        = {.msk = 0x00010000U, .pos = 16};
const field32_t SDMMCx_STAR_RXFIFOF        = {.msk = 0x00020000U, .pos = 17};
const field32_t SDMMCx_STAR_TXFIFOE        = {.msk = 0x00040000U, .pos = 18};
const field32_t SDMMCx_STAR_RXFIFOE        = {.msk = 0x00080000U, .pos = 19};
const field32_t SDMMCx_STAR_BUSYD0         = {.msk = 0x00100000U, .pos = 20};
const field32_t SDMMCx_STAR_BUSYD0END      = {.msk = 0x00200000U, .pos = 21};
const field32_t SDMMCx_STAR_SDIOIT         = {.msk = 0x00400000U, .pos = 22};
const field32_t SDMMCx_STAR_ACKFAIL        = {.msk = 0x00800000U, .pos = 23};
const field32_t SDMMCx_STAR_ACKTIMEOUT     = {.msk = 0x01000000U, .pos = 24};
const field32_t SDMMCx_STAR_VSWEND         = {.msk = 0x02000000U, .pos = 25};
const field32_t SDMMCx_STAR_CKSTOP         = {.msk = 0x04000000U, .pos = 26};
const field32_t SDMMCx_STAR_IDMATE         = {.msk = 0x08000000U, .pos = 27};
const field32_t SDMMCx_STAR_IDMABTC        = {.msk = 0x10000000U, .pos = 28};
const field32_t SDMMCx_ICR_CCRCFAILC       = {.msk = 0x00000001U, .pos = 0};
const field32_t SDMMCx_ICR_DCRCFAILC       = {.msk = 0x00000002U, .pos = 1};
const field32_t SDMMCx_ICR_CTIMEOUTC       = {.msk = 0x00000004U, .pos = 2};
const field32_t SDMMCx_ICR_DTIMEOUTC       = {.msk = 0x00000008U, .pos = 3};
const field32_t SDMMCx_ICR_TXUNDERRC       = {.msk = 0x00000010U, .pos = 4};
const field32_t SDMMCx_ICR_RXOVERRC        = {.msk = 0x00000020U, .pos = 5};
const field32_t SDMMCx_ICR_CMDRENDC        = {.msk = 0x00000040U, .pos = 6};
const field32_t SDMMCx_ICR_CMDSENTC        = {.msk = 0x00000080U, .pos = 7};
const field32_t SDMMCx_ICR_DATAENDC        = {.msk = 0x00000100U, .pos = 8};
const field32_t SDMMCx_ICR_DHOLDC          = {.msk = 0x00000200U, .pos = 9};
const field32_t SDMMCx_ICR_DBCKENDC        = {.msk = 0x00000400U, .pos = 10};
const field32_t SDMMCx_ICR_DABORTC         = {.msk = 0x00000800U, .pos = 11};
const field32_t SDMMCx_ICR_BUSYD0ENDC      = {.msk = 0x00200000U, .pos = 21};
const field32_t SDMMCx_ICR_SDIOITC         = {.msk = 0x00400000U, .pos = 22};
const field32_t SDMMCx_ICR_ACKFAILC        = {.msk = 0x00800000U, .pos = 23};
const field32_t SDMMCx_ICR_ACKTIMEOUTC     = {.msk = 0x01000000U, .pos = 24};
const field32_t SDMMCx_ICR_VSWENDC         = {.msk = 0x02000000U, .pos = 25};
const field32_t SDMMCx_ICR_CKSTOPC         = {.msk = 0x04000000U, .pos = 26};
const field32_t SDMMCx_ICR_IDMATEC         = {.msk = 0x08000000U, .pos = 27};
const field32_t SDMMCx_ICR_IDMABTCC        = {.msk = 0x10000000U, .pos = 28};
const field32_t SDMMCx_MASKR_CCRCFAILIE    = {.msk = 0x00000001U, .pos = 0};
const field32_t SDMMCx_MASKR_DCRCFAILIE    = {.msk = 0x00000002U, .pos = 1};
const field32_t SDMMCx_MASKR_CTIMEOUTIE    = {.msk = 0x00000004U, .pos = 2};
const field32_t SDMMCx_MASKR_DTIMEOUTIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t SDMMCx_MASKR_TXUNDERRIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t SDMMCx_MASKR_RXOVERRIE     = {.msk = 0x00000020U, .pos = 5};
const field32_t SDMMCx_MASKR_CMDRENDIE     = {.msk = 0x00000040U, .pos = 6};
const field32_t SDMMCx_MASKR_CMDSENTIE     = {.msk = 0x00000080U, .pos = 7};
const field32_t SDMMCx_MASKR_DATAENDIE     = {.msk = 0x00000100U, .pos = 8};
const field32_t SDMMCx_MASKR_DHOLDIE       = {.msk = 0x00000200U, .pos = 9};
const field32_t SDMMCx_MASKR_DBCKENDIE     = {.msk = 0x00000400U, .pos = 10};
const field32_t SDMMCx_MASKR_DABORTIE      = {.msk = 0x00000800U, .pos = 11};
const field32_t SDMMCx_MASKR_TXFIFOHEIE    = {.msk = 0x00004000U, .pos = 14};
const field32_t SDMMCx_MASKR_RXFIFOHFIE    = {.msk = 0x00008000U, .pos = 15};
const field32_t SDMMCx_MASKR_RXFIFOFIE     = {.msk = 0x00020000U, .pos = 17};
const field32_t SDMMCx_MASKR_TXFIFOEIE     = {.msk = 0x00040000U, .pos = 18};
const field32_t SDMMCx_MASKR_BUSYD0ENDIE   = {.msk = 0x00200000U, .pos = 21};
const field32_t SDMMCx_MASKR_SDIOITIE      = {.msk = 0x00400000U, .pos = 22};
const field32_t SDMMCx_MASKR_ACKFAILIE     = {.msk = 0x00800000U, .pos = 23};
const field32_t SDMMCx_MASKR_ACKTIMEOUTIE  = {.msk = 0x01000000U, .pos = 24};
const field32_t SDMMCx_MASKR_VSWENDIE      = {.msk = 0x02000000U, .pos = 25};
const field32_t SDMMCx_MASKR_CKSTOPIE      = {.msk = 0x04000000U, .pos = 26};
const field32_t SDMMCx_MASKR_IDMABTCIE     = {.msk = 0x10000000U, .pos = 28};
const field32_t SDMMCx_ACKTIMER_ACKTIME    = {.msk = 0x01FFFFFFU, .pos = 0};
const field32_t SDMMCx_IDMACTRLR_IDMAEN    = {.msk = 0x00000001U, .pos = 0};
const field32_t SDMMCx_IDMACTRLR_IDMABMODE = {.msk = 0x00000002U, .pos = 1};
const field32_t SDMMCx_IDMACTRLR_IDMABACT  = {.msk = 0x00000004U, .pos = 2};
const field32_t SDMMCx_IDMABSIZER_IDMABNDT = {.msk = 0x00001FE0U, .pos = 5};
const field32_t SDMMCx_RESPCMDR_RESPCMD    = {.msk = 0x0000003FU, .pos = 0};

/**************************************************************************************************
 * @section VREFBUF Definitions
 **************************************************************************************************/

/** @subsection VREFBUF Register Definitions */

rw_reg32_t const VREFBUF_CSR = (rw_reg32_t)0x58003C00U;
rw_reg32_t const VREFBUF_CCR = (rw_reg32_t)0x58003C04U;

/** @subsection VREFBUF Register Field Definitions */

const field32_t VREFBUF_CSR_ENVR = {.msk = 0x00000001U, .pos = 0};
const field32_t VREFBUF_CSR_HIZ  = {.msk = 0x00000002U, .pos = 1};
const field32_t VREFBUF_CSR_VRR  = {.msk = 0x00000008U, .pos = 3};
const field32_t VREFBUF_CSR_VRS  = {.msk = 0x00000070U, .pos = 4};
const field32_t VREFBUF_CCR_TRIM = {.msk = 0x0000003FU, .pos = 0};

/**************************************************************************************************
 * @section IWDGx Definitions
 **************************************************************************************************/

/** @subsection Enumerated IWDGx Register Definitions */

rw_reg32_t const IWDGx_KR[3] = {
  [1] = (rw_reg32_t)0x58004800U,
  [2] = (rw_reg32_t)0x58004C00U,
};

rw_reg32_t const IWDGx_PR[3] = {
  [1] = (rw_reg32_t)0x58004804U,
  [2] = (rw_reg32_t)0x58004C04U,
};

rw_reg32_t const IWDGx_RLR[3] = {
  [1] = (rw_reg32_t)0x58004808U,
  [2] = (rw_reg32_t)0x58004C08U,
};

ro_reg32_t const IWDGx_SR[3] = {
  [1] = (ro_reg32_t)0x5800480CU,
  [2] = (ro_reg32_t)0x58004C0CU,
};

rw_reg32_t const IWDGx_WINR[3] = {
  [1] = (rw_reg32_t)0x58004810U,
  [2] = (rw_reg32_t)0x58004C10U,
};

/** @subsection IWDGx Register Field Definitions */

const field32_t IWDGx_KR_KEY   = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t IWDGx_PR_PR    = {.msk = 0x00000007U, .pos = 0};
const field32_t IWDGx_RLR_RL   = {.msk = 0x00000FFFU, .pos = 0};
const field32_t IWDGx_SR_PVU   = {.msk = 0x00000001U, .pos = 0};
const field32_t IWDGx_SR_RVU   = {.msk = 0x00000002U, .pos = 1};
const field32_t IWDGx_SR_WVU   = {.msk = 0x00000004U, .pos = 2};
const field32_t IWDGx_WINR_WIN = {.msk = 0x00000FFFU, .pos = 0};

/**************************************************************************************************
 * @section WWDGx Definitions
 **************************************************************************************************/

/** @subsection Enumerated WWDGx Register Definitions */

rw_reg32_t const WWDGx_CR[3] = {
  [1] = (rw_reg32_t)0x50003000U,
  [2] = (rw_reg32_t)0x40002C00U,
};

rw_reg32_t const WWDGx_CFR[3] = {
  [1] = (rw_reg32_t)0x50003004U,
  [2] = (rw_reg32_t)0x40002C04U,
};

rw_reg32_t const WWDGx_SR[3] = {
  [1] = (rw_reg32_t)0x50003008U,
  [2] = (rw_reg32_t)0x40002C08U,
};

/** @subsection WWDGx Register Field Definitions */

const field32_t WWDGx_CR_T      = {.msk = 0x0000007FU, .pos = 0};
const field32_t WWDGx_CR_WDGA   = {.msk = 0x00000080U, .pos = 7};
const field32_t WWDGx_CFR_W     = {.msk = 0x0000007FU, .pos = 0};
const field32_t WWDGx_CFR_WDGTB = {.msk = 0x00001800U, .pos = 11};
const field32_t WWDGx_CFR_EWI   = {.msk = 0x00000200U, .pos = 9};
const field32_t WWDGx_SR_EWIF   = {.msk = 0x00000001U, .pos = 0};

/**************************************************************************************************
 * @section PWR Definitions
 **************************************************************************************************/

/** @subsection PWR Register Definitions */

rw_reg32_t const PWR_CR1     = (rw_reg32_t)0x58024800U;
ro_reg32_t const PWR_CSR1    = (ro_reg32_t)0x58024804U;
rw_reg32_t const PWR_CR2     = (rw_reg32_t)0x58024808U;
rw_reg32_t const PWR_CR3     = (rw_reg32_t)0x5802480CU;
rw_reg32_t const PWR_CPUCR   = (rw_reg32_t)0x58024810U;
rw_reg32_t const PWR_D3CR    = (rw_reg32_t)0x58024818U;
rw_reg32_t const PWR_WKUPCR  = (rw_reg32_t)0x58024820U;
rw_reg32_t const PWR_WKUPFR  = (rw_reg32_t)0x58024824U;
rw_reg32_t const PWR_WKUPEPR = (rw_reg32_t)0x58024828U;

/** @subsection PWR Register Field Definitions */

const field32_t PWR_CR1_LPDS       = {.msk = 0x00000001U, .pos = 0};
const field32_t PWR_CR1_PVDE       = {.msk = 0x00000010U, .pos = 4};
const field32_t PWR_CR1_PLS        = {.msk = 0x000000E0U, .pos = 5};
const field32_t PWR_CR1_DBP        = {.msk = 0x00000100U, .pos = 8};
const field32_t PWR_CR1_FLPS       = {.msk = 0x00000200U, .pos = 9};
const field32_t PWR_CR1_SVOS       = {.msk = 0x0000C000U, .pos = 14};
const field32_t PWR_CR1_AVDEN      = {.msk = 0x00010000U, .pos = 16};
const field32_t PWR_CR1_ALS        = {.msk = 0x00060000U, .pos = 17};
const field32_t PWR_CSR1_PVDO      = {.msk = 0x00000010U, .pos = 4};
const field32_t PWR_CSR1_ACTVOSRDY = {.msk = 0x00002000U, .pos = 13};
const field32_t PWR_CSR1_ACTVOS    = {.msk = 0x0000C000U, .pos = 14};
const field32_t PWR_CSR1_AVDO      = {.msk = 0x00010000U, .pos = 16};
const field32_t PWR_CR2_BREN       = {.msk = 0x00000001U, .pos = 0};
const field32_t PWR_CR2_MONEN      = {.msk = 0x00000010U, .pos = 4};
const field32_t PWR_CR2_BRRDY      = {.msk = 0x00010000U, .pos = 16};
const field32_t PWR_CR2_VBATL      = {.msk = 0x00100000U, .pos = 20};
const field32_t PWR_CR2_VBATH      = {.msk = 0x00200000U, .pos = 21};
const field32_t PWR_CR2_TEMPL      = {.msk = 0x00400000U, .pos = 22};
const field32_t PWR_CR2_TEMPH      = {.msk = 0x00800000U, .pos = 23};
const field32_t PWR_CR3_BYPASS     = {.msk = 0x00000001U, .pos = 0};
const field32_t PWR_CR3_LDOEN      = {.msk = 0x00000002U, .pos = 1};
const field32_t PWR_CR3_SDEN       = {.msk = 0x00000004U, .pos = 2};
const field32_t PWR_CR3_VBE        = {.msk = 0x00000100U, .pos = 8};
const field32_t PWR_CR3_VBRS       = {.msk = 0x00000200U, .pos = 9};
const field32_t PWR_CR3_USB33DEN   = {.msk = 0x01000000U, .pos = 24};
const field32_t PWR_CR3_USBREGEN   = {.msk = 0x02000000U, .pos = 25};
const field32_t PWR_CR3_USB33RDY   = {.msk = 0x04000000U, .pos = 26};
const field32_t PWR_CPUCR_STOPF    = {.msk = 0x00000020U, .pos = 5};
const field32_t PWR_CPUCR_SBF      = {.msk = 0x00000040U, .pos = 6};
const field32_t PWR_CPUCR_CSSF     = {.msk = 0x00000200U, .pos = 9};
const field32_t PWR_CPUCR_RUN_D3   = {.msk = 0x00000800U, .pos = 11};
const field32_t PWR_D3CR_VOSRDY    = {.msk = 0x00002000U, .pos = 13};
const field32_t PWR_D3CR_VOS       = {.msk = 0x0000C000U, .pos = 14};
const field32_t PWR_WKUPCR_WKUPC   = {.msk = 0x0000003FU, .pos = 0};

/** @subsection Enumerated PWR Register Field Definitions */

field32_t const PWR_CPUCR_PDDS_Dx[4] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const PWR_CPUCR_SBF_Dx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const PWR_WKUPFR_WKUPFx[7] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
  [6] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const PWR_WKUPEPR_WKUPENx[7] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
  [6] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const PWR_WKUPEPR_WKUPPx[7] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
  [5] = {.msk = 0x00001000U, .pos = 12},
  [6] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const PWR_WKUPEPR_WKUPPUPDx[7] = {
  [1] = {.msk = 0x00030000U, .pos = 16},
  [2] = {.msk = 0x000C0000U, .pos = 18},
  [3] = {.msk = 0x00300000U, .pos = 20},
  [4] = {.msk = 0x00C00000U, .pos = 22},
  [5] = {.msk = 0x03000000U, .pos = 24},
  [6] = {.msk = 0x0C000000U, .pos = 26},
};

/**************************************************************************************************
 * @section SPIx Definitions
 **************************************************************************************************/

/** @subsection Enumerated SPIx Register Definitions */

rw_reg32_t const SPIx_CR1[7] = {
  [1] = (rw_reg32_t)0x40013000U,
  [2] = (rw_reg32_t)0x40003800U,
  [3] = (rw_reg32_t)0x40003C00U,
  [4] = (rw_reg32_t)0x40013400U,
  [5] = (rw_reg32_t)0x40015000U,
  [6] = (rw_reg32_t)0x58001400U,
};

rw_reg32_t const SPIx_CR2[7] = {
  [1] = (rw_reg32_t)0x40013004U,
  [2] = (rw_reg32_t)0x40003804U,
  [3] = (rw_reg32_t)0x40003C04U,
  [4] = (rw_reg32_t)0x40013404U,
  [5] = (rw_reg32_t)0x40015004U,
  [6] = (rw_reg32_t)0x58001404U,
};

rw_reg32_t const SPIx_CFG1[7] = {
  [1] = (rw_reg32_t)0x40013008U,
  [2] = (rw_reg32_t)0x40003808U,
  [3] = (rw_reg32_t)0x40003C08U,
  [4] = (rw_reg32_t)0x40013408U,
  [5] = (rw_reg32_t)0x40015008U,
  [6] = (rw_reg32_t)0x58001408U,
};

rw_reg32_t const SPIx_CFG2[7] = {
  [1] = (rw_reg32_t)0x4001300CU,
  [2] = (rw_reg32_t)0x4000380CU,
  [3] = (rw_reg32_t)0x40003C0CU,
  [4] = (rw_reg32_t)0x4001340CU,
  [5] = (rw_reg32_t)0x4001500CU,
  [6] = (rw_reg32_t)0x5800140CU,
};

rw_reg32_t const SPIx_IER[7] = {
  [1] = (rw_reg32_t)0x40013010U,
  [2] = (rw_reg32_t)0x40003810U,
  [3] = (rw_reg32_t)0x40003C10U,
  [4] = (rw_reg32_t)0x40013410U,
  [5] = (rw_reg32_t)0x40015010U,
  [6] = (rw_reg32_t)0x58001410U,
};

ro_reg32_t const SPIx_SR[7] = {
  [1] = (ro_reg32_t)0x40013014U,
  [2] = (ro_reg32_t)0x40003814U,
  [3] = (ro_reg32_t)0x40003C14U,
  [4] = (ro_reg32_t)0x40013414U,
  [5] = (ro_reg32_t)0x40015014U,
  [6] = (ro_reg32_t)0x58001414U,
};

rw_reg32_t const SPIx_IFCR[7] = {
  [1] = (rw_reg32_t)0x40013018U,
  [2] = (rw_reg32_t)0x40003818U,
  [3] = (rw_reg32_t)0x40003C18U,
  [4] = (rw_reg32_t)0x40013418U,
  [5] = (rw_reg32_t)0x40015018U,
  [6] = (rw_reg32_t)0x58001418U,
};

rw_reg32_t const SPIx_TXDR[7] = {
  [1] = (rw_reg32_t)0x40013020U,
  [2] = (rw_reg32_t)0x40003820U,
  [3] = (rw_reg32_t)0x40003C20U,
  [4] = (rw_reg32_t)0x40013420U,
  [5] = (rw_reg32_t)0x40015020U,
  [6] = (rw_reg32_t)0x58001420U,
};

ro_reg32_t const SPIx_RXDR[7] = {
  [1] = (ro_reg32_t)0x40013030U,
  [2] = (ro_reg32_t)0x40003830U,
  [3] = (ro_reg32_t)0x40003C30U,
  [4] = (ro_reg32_t)0x40013430U,
  [5] = (ro_reg32_t)0x40015030U,
  [6] = (ro_reg32_t)0x58001430U,
};

rw_reg32_t const SPIx_CRCPOLY[7] = {
  [1] = (rw_reg32_t)0x40013040U,
  [2] = (rw_reg32_t)0x40003840U,
  [3] = (rw_reg32_t)0x40003C40U,
  [4] = (rw_reg32_t)0x40013440U,
  [5] = (rw_reg32_t)0x40015040U,
  [6] = (rw_reg32_t)0x58001440U,
};

rw_reg32_t const SPIx_TXCRC[7] = {
  [1] = (rw_reg32_t)0x40013044U,
  [2] = (rw_reg32_t)0x40003844U,
  [3] = (rw_reg32_t)0x40003C44U,
  [4] = (rw_reg32_t)0x40013444U,
  [5] = (rw_reg32_t)0x40015044U,
  [6] = (rw_reg32_t)0x58001444U,
};

rw_reg32_t const SPIx_RXCRC[7] = {
  [1] = (rw_reg32_t)0x40013048U,
  [2] = (rw_reg32_t)0x40003848U,
  [3] = (rw_reg32_t)0x40003C48U,
  [4] = (rw_reg32_t)0x40013448U,
  [5] = (rw_reg32_t)0x40015048U,
  [6] = (rw_reg32_t)0x58001448U,
};

rw_reg32_t const SPIx_UDRDR[7] = {
  [1] = (rw_reg32_t)0x4001304CU,
  [2] = (rw_reg32_t)0x4000384CU,
  [3] = (rw_reg32_t)0x40003C4CU,
  [4] = (rw_reg32_t)0x4001344CU,
  [5] = (rw_reg32_t)0x4001504CU,
  [6] = (rw_reg32_t)0x5800144CU,
};

rw_reg32_t const SPIx_CGFR[7] = {
  [1] = (rw_reg32_t)0x40013050U,
  [2] = (rw_reg32_t)0x40003850U,
  [3] = (rw_reg32_t)0x40003C50U,
  [4] = (rw_reg32_t)0x40013450U,
  [5] = (rw_reg32_t)0x40015050U,
  [6] = (rw_reg32_t)0x58001450U,
};

/** @subsection SPIx Register Field Definitions */

const field32_t SPIx_CR1_IOLOCK   = {.msk = 0x00010000U, .pos = 16};
const field32_t SPIx_CR1_TCRCI    = {.msk = 0x00008000U, .pos = 15};
const field32_t SPIx_CR1_RCRCI    = {.msk = 0x00004000U, .pos = 14};
const field32_t SPIx_CR1_CRC33_17 = {.msk = 0x00002000U, .pos = 13};
const field32_t SPIx_CR1_SSI      = {.msk = 0x00001000U, .pos = 12};
const field32_t SPIx_CR1_HDDIR    = {.msk = 0x00000800U, .pos = 11};
const field32_t SPIx_CR1_CSUSP    = {.msk = 0x00000400U, .pos = 10};
const field32_t SPIx_CR1_CSTART   = {.msk = 0x00000200U, .pos = 9};
const field32_t SPIx_CR1_MASRX    = {.msk = 0x00000100U, .pos = 8};
const field32_t SPIx_CR1_SPE      = {.msk = 0x00000001U, .pos = 0};
const field32_t SPIx_CR2_TSER     = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SPIx_CR2_TSIZE    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t SPIx_CFG1_MBR     = {.msk = 0x70000000U, .pos = 28};
const field32_t SPIx_CFG1_CRCEN   = {.msk = 0x00400000U, .pos = 22};
const field32_t SPIx_CFG1_CRCSIZE = {.msk = 0x001F0000U, .pos = 16};
const field32_t SPIx_CFG1_TXDMAEN = {.msk = 0x00008000U, .pos = 15};
const field32_t SPIx_CFG1_RXDMAEN = {.msk = 0x00004000U, .pos = 14};
const field32_t SPIx_CFG1_UDRDET  = {.msk = 0x00001800U, .pos = 11};
const field32_t SPIx_CFG1_UDRCFG  = {.msk = 0x00000600U, .pos = 9};
const field32_t SPIx_CFG1_FTHVL   = {.msk = 0x000001E0U, .pos = 5};
const field32_t SPIx_CFG1_DSIZE   = {.msk = 0x0000001FU, .pos = 0};
const field32_t SPIx_CFG2_AFCNTR  = {.msk = 0x80000000U, .pos = 31};
const field32_t SPIx_CFG2_SSOM    = {.msk = 0x40000000U, .pos = 30};
const field32_t SPIx_CFG2_SSOE    = {.msk = 0x20000000U, .pos = 29};
const field32_t SPIx_CFG2_SSIOP   = {.msk = 0x10000000U, .pos = 28};
const field32_t SPIx_CFG2_SSM     = {.msk = 0x04000000U, .pos = 26};
const field32_t SPIx_CFG2_CPOL    = {.msk = 0x02000000U, .pos = 25};
const field32_t SPIx_CFG2_CPHA    = {.msk = 0x01000000U, .pos = 24};
const field32_t SPIx_CFG2_LSBFRST = {.msk = 0x00800000U, .pos = 23};
const field32_t SPIx_CFG2_MASTER  = {.msk = 0x00400000U, .pos = 22};
const field32_t SPIx_CFG2_SP      = {.msk = 0x00380000U, .pos = 19};
const field32_t SPIx_CFG2_COMM    = {.msk = 0x00060000U, .pos = 17};
const field32_t SPIx_CFG2_IOSWP   = {.msk = 0x00008000U, .pos = 15};
const field32_t SPIx_CFG2_MIDI    = {.msk = 0x000000F0U, .pos = 4};
const field32_t SPIx_CFG2_MSSI    = {.msk = 0x0000000FU, .pos = 0};
const field32_t SPIx_IER_TSERFIE  = {.msk = 0x00000400U, .pos = 10};
const field32_t SPIx_IER_MODFIE   = {.msk = 0x00000200U, .pos = 9};
const field32_t SPIx_IER_TIFREIE  = {.msk = 0x00000100U, .pos = 8};
const field32_t SPIx_IER_CRCEIE   = {.msk = 0x00000080U, .pos = 7};
const field32_t SPIx_IER_OVRIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t SPIx_IER_UDRIE    = {.msk = 0x00000020U, .pos = 5};
const field32_t SPIx_IER_TXTFIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t SPIx_IER_EOTIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t SPIx_IER_DPXPIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t SPIx_IER_TXPIE    = {.msk = 0x00000002U, .pos = 1};
const field32_t SPIx_IER_RXPIE    = {.msk = 0x00000001U, .pos = 0};
const field32_t SPIx_SR_CTSIZE    = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SPIx_SR_RXWNE     = {.msk = 0x00008000U, .pos = 15};
const field32_t SPIx_SR_RXPLVL    = {.msk = 0x00006000U, .pos = 13};
const field32_t SPIx_SR_TXC       = {.msk = 0x00001000U, .pos = 12};
const field32_t SPIx_SR_SUSP      = {.msk = 0x00000800U, .pos = 11};
const field32_t SPIx_SR_TSERF     = {.msk = 0x00000400U, .pos = 10};
const field32_t SPIx_SR_MODF      = {.msk = 0x00000200U, .pos = 9};
const field32_t SPIx_SR_TIFRE     = {.msk = 0x00000100U, .pos = 8};
const field32_t SPIx_SR_CRCE      = {.msk = 0x00000080U, .pos = 7};
const field32_t SPIx_SR_OVR       = {.msk = 0x00000040U, .pos = 6};
const field32_t SPIx_SR_UDR       = {.msk = 0x00000020U, .pos = 5};
const field32_t SPIx_SR_TXTF      = {.msk = 0x00000010U, .pos = 4};
const field32_t SPIx_SR_EOT       = {.msk = 0x00000008U, .pos = 3};
const field32_t SPIx_SR_DXP       = {.msk = 0x00000004U, .pos = 2};
const field32_t SPIx_SR_TXP       = {.msk = 0x00000002U, .pos = 1};
const field32_t SPIx_SR_RXP       = {.msk = 0x00000001U, .pos = 0};
const field32_t SPIx_IFCR_SUSPC   = {.msk = 0x00000800U, .pos = 11};
const field32_t SPIx_IFCR_TSERFC  = {.msk = 0x00000400U, .pos = 10};
const field32_t SPIx_IFCR_MODFC   = {.msk = 0x00000200U, .pos = 9};
const field32_t SPIx_IFCR_TIFREC  = {.msk = 0x00000100U, .pos = 8};
const field32_t SPIx_IFCR_CRCEC   = {.msk = 0x00000080U, .pos = 7};
const field32_t SPIx_IFCR_OVRC    = {.msk = 0x00000040U, .pos = 6};
const field32_t SPIx_IFCR_UDRC    = {.msk = 0x00000020U, .pos = 5};
const field32_t SPIx_IFCR_TXTFC   = {.msk = 0x00000010U, .pos = 4};
const field32_t SPIx_IFCR_EOTC    = {.msk = 0x00000008U, .pos = 3};
const field32_t SPIx_CGFR_MCKOE   = {.msk = 0x02000000U, .pos = 25};
const field32_t SPIx_CGFR_ODD     = {.msk = 0x01000000U, .pos = 24};
const field32_t SPIx_CGFR_I2SDIV  = {.msk = 0x00FF0000U, .pos = 16};
const field32_t SPIx_CGFR_DATFMT  = {.msk = 0x00004000U, .pos = 14};
const field32_t SPIx_CGFR_WSINV   = {.msk = 0x00002000U, .pos = 13};
const field32_t SPIx_CGFR_FIXCH   = {.msk = 0x00001000U, .pos = 12};
const field32_t SPIx_CGFR_CKPOL   = {.msk = 0x00000800U, .pos = 11};
const field32_t SPIx_CGFR_CHLEN   = {.msk = 0x00000400U, .pos = 10};
const field32_t SPIx_CGFR_DATLEN  = {.msk = 0x00000300U, .pos = 8};
const field32_t SPIx_CGFR_PCMSYNC = {.msk = 0x00000080U, .pos = 7};
const field32_t SPIx_CGFR_I2SSTD  = {.msk = 0x00000030U, .pos = 4};
const field32_t SPIx_CGFR_I2SCFG  = {.msk = 0x0000000EU, .pos = 1};
const field32_t SPIx_CGFR_I2SMOD  = {.msk = 0x00000001U, .pos = 0};

/**************************************************************************************************
 * @section LTDC Definitions
 **************************************************************************************************/

/** @subsection LTDC Register Definitions */

rw_reg32_t const LTDC_SSCR  = (rw_reg32_t)0x50001008U;
rw_reg32_t const LTDC_BPCR  = (rw_reg32_t)0x5000100CU;
rw_reg32_t const LTDC_AWCR  = (rw_reg32_t)0x50001010U;
rw_reg32_t const LTDC_TWCR  = (rw_reg32_t)0x50001014U;
rw_reg32_t const LTDC_GCR   = (rw_reg32_t)0x50001018U;
rw_reg32_t const LTDC_SRCR  = (rw_reg32_t)0x50001024U;
rw_reg32_t const LTDC_BCCR  = (rw_reg32_t)0x5000102CU;
rw_reg32_t const LTDC_IER   = (rw_reg32_t)0x50001034U;
ro_reg32_t const LTDC_ISR   = (ro_reg32_t)0x50001038U;
rw_reg32_t const LTDC_ICR   = (rw_reg32_t)0x5000103CU;
rw_reg32_t const LTDC_LIPCR = (rw_reg32_t)0x50001040U;
ro_reg32_t const LTDC_CPSR  = (ro_reg32_t)0x50001044U;
ro_reg32_t const LTDC_CDSR  = (ro_reg32_t)0x50001048U;

/** @subsection Enumerated LTDC Register Definitions */

rw_reg32_t const LTDC_LxCR[3] = {
  [1] = (rw_reg32_t)0x50001084U,
  [2] = (rw_reg32_t)0x50001104U,
};

rw_reg32_t const LTDC_LxWHPCR[3] = {
  [1] = (rw_reg32_t)0x50001088U,
  [2] = (rw_reg32_t)0x50001108U,
};

rw_reg32_t const LTDC_LxWVPCR[3] = {
  [1] = (rw_reg32_t)0x5000108CU,
  [2] = (rw_reg32_t)0x5000110CU,
};

rw_reg32_t const LTDC_LxCKCR[3] = {
  [1] = (rw_reg32_t)0x50001090U,
  [2] = (rw_reg32_t)0x50001110U,
};

rw_reg32_t const LTDC_LxPFCR[3] = {
  [1] = (rw_reg32_t)0x50001094U,
  [2] = (rw_reg32_t)0x50001114U,
};

rw_reg32_t const LTDC_LxCACR[3] = {
  [1] = (rw_reg32_t)0x50001098U,
  [2] = (rw_reg32_t)0x50001118U,
};

rw_reg32_t const LTDC_LxDCCR[3] = {
  [1] = (rw_reg32_t)0x5000109CU,
  [2] = (rw_reg32_t)0x5000111CU,
};

rw_reg32_t const LTDC_LxBFCR[3] = {
  [1] = (rw_reg32_t)0x500010A0U,
  [2] = (rw_reg32_t)0x50001120U,
};

rw_reg32_t const LTDC_LxCFBAR[3] = {
  [1] = (rw_reg32_t)0x500010ACU,
  [2] = (rw_reg32_t)0x5000112CU,
};

rw_reg32_t const LTDC_LxCFBLR[3] = {
  [1] = (rw_reg32_t)0x500010B0U,
  [2] = (rw_reg32_t)0x50001130U,
};

rw_reg32_t const LTDC_LxCFBLNR[3] = {
  [1] = (rw_reg32_t)0x500010B4U,
  [2] = (rw_reg32_t)0x50001134U,
};

rw_reg32_t const LTDC_LxCLUTWR[3] = {
  [1] = (rw_reg32_t)0x500010C4U,
  [2] = (rw_reg32_t)0x50001144U,
};

/** @subsection LTDC Register Field Definitions */

const field32_t LTDC_SSCR_HSW         = {.msk = 0x03FF0000U, .pos = 16};
const field32_t LTDC_SSCR_VSH         = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_BPCR_AHBP        = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t LTDC_BPCR_AVBP        = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_AWCR_AAV         = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t LTDC_AWCR_AAH         = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_TWCR_TOTALW      = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t LTDC_TWCR_TOTALH      = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_GCR_HSPOL        = {.msk = 0x80000000U, .pos = 31};
const field32_t LTDC_GCR_VSPOL        = {.msk = 0x40000000U, .pos = 30};
const field32_t LTDC_GCR_DEPOL        = {.msk = 0x20000000U, .pos = 29};
const field32_t LTDC_GCR_PCPOL        = {.msk = 0x10000000U, .pos = 28};
const field32_t LTDC_GCR_DEN          = {.msk = 0x00010000U, .pos = 16};
const field32_t LTDC_GCR_DRW          = {.msk = 0x00007000U, .pos = 12};
const field32_t LTDC_GCR_DGW          = {.msk = 0x00000700U, .pos = 8};
const field32_t LTDC_GCR_DBW          = {.msk = 0x00000070U, .pos = 4};
const field32_t LTDC_GCR_LTDCEN       = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_SRCR_VBR         = {.msk = 0x00000002U, .pos = 1};
const field32_t LTDC_SRCR_IMR         = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_BCCR_BCBLUE      = {.msk = 0x000000FFU, .pos = 0};
const field32_t LTDC_BCCR_BCGREEN     = {.msk = 0x0000FF00U, .pos = 8};
const field32_t LTDC_BCCR_BCRED       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t LTDC_IER_RRIE         = {.msk = 0x00000008U, .pos = 3};
const field32_t LTDC_IER_TERRIE       = {.msk = 0x00000004U, .pos = 2};
const field32_t LTDC_IER_FUIE         = {.msk = 0x00000002U, .pos = 1};
const field32_t LTDC_IER_LIE          = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_ISR_RRIF         = {.msk = 0x00000008U, .pos = 3};
const field32_t LTDC_ISR_TERRIF       = {.msk = 0x00000004U, .pos = 2};
const field32_t LTDC_ISR_FUIF         = {.msk = 0x00000002U, .pos = 1};
const field32_t LTDC_ISR_LIF          = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_ICR_CRRIF        = {.msk = 0x00000008U, .pos = 3};
const field32_t LTDC_ICR_CTERRIF      = {.msk = 0x00000004U, .pos = 2};
const field32_t LTDC_ICR_CFUIF        = {.msk = 0x00000002U, .pos = 1};
const field32_t LTDC_ICR_CLIF         = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_LIPCR_LIPOS      = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_CPSR_CXPOS       = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t LTDC_CPSR_CYPOS       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t LTDC_CDSR_HSYNCS      = {.msk = 0x00000008U, .pos = 3};
const field32_t LTDC_CDSR_VSYNCS      = {.msk = 0x00000004U, .pos = 2};
const field32_t LTDC_CDSR_HDES        = {.msk = 0x00000002U, .pos = 1};
const field32_t LTDC_CDSR_VDES        = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_LxCR_CLUTEN      = {.msk = 0x00000010U, .pos = 4};
const field32_t LTDC_LxCR_COLKEN      = {.msk = 0x00000002U, .pos = 1};
const field32_t LTDC_LxCR_LEN         = {.msk = 0x00000001U, .pos = 0};
const field32_t LTDC_LxWHPCR_WHSPPOS  = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t LTDC_LxWHPCR_WHSTPOS  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t LTDC_LxWVPCR_WVSPPOS  = {.msk = 0x07FF0000U, .pos = 16};
const field32_t LTDC_LxWVPCR_WVSTPOS  = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_LxCKCR_CKRED     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t LTDC_LxCKCR_CKGREEN   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t LTDC_LxCKCR_CKBLUE    = {.msk = 0x000000FFU, .pos = 0};
const field32_t LTDC_LxPFCR_PF        = {.msk = 0x00000007U, .pos = 0};
const field32_t LTDC_LxCACR_CONSTA    = {.msk = 0x000000FFU, .pos = 0};
const field32_t LTDC_LxDCCR_DCALPHA   = {.msk = 0xFF000000U, .pos = 24};
const field32_t LTDC_LxDCCR_DCRED     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t LTDC_LxDCCR_DCGREEN   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t LTDC_LxDCCR_DCBLUE    = {.msk = 0x000000FFU, .pos = 0};
const field32_t LTDC_LxCFBLR_CFBP     = {.msk = 0x1FFF0000U, .pos = 16};
const field32_t LTDC_LxCFBLR_CFBLL    = {.msk = 0x00001FFFU, .pos = 0};
const field32_t LTDC_LxCFBLNR_CFBLNBR = {.msk = 0x000007FFU, .pos = 0};
const field32_t LTDC_LxCLUTWR_CLUTADD = {.msk = 0xFF000000U, .pos = 24};
const field32_t LTDC_LxCLUTWR_RED     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t LTDC_LxCLUTWR_GREEN   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t LTDC_LxCLUTWR_BLUE    = {.msk = 0x000000FFU, .pos = 0};

/** @subsection Enumerated LTDC Register Field Definitions */

field32_t const LTDC_LxBFCR_BFx[3] = {
  [1] = {.msk = 0x00000700U, .pos = 8},
  [2] = {.msk = 0x00000007U, .pos = 0},
};

/**************************************************************************************************
 * @section SPDIFRX Definitions
 **************************************************************************************************/

/** @subsection SPDIFRX Register Definitions */

rw_reg32_t const SPDIFRX_CR    = (rw_reg32_t)0x40004000U;
rw_reg32_t const SPDIFRX_IMR   = (rw_reg32_t)0x40004004U;
ro_reg32_t const SPDIFRX_SR    = (ro_reg32_t)0x40004008U;
rw_reg32_t const SPDIFRX_IFCR  = (rw_reg32_t)0x4000400CU;
ro_reg32_t const SPDIFRX_DR_00 = (ro_reg32_t)0x40004010U;
ro_reg32_t const SPDIFRX_CSR   = (ro_reg32_t)0x40004014U;
ro_reg32_t const SPDIFRX_DIR   = (ro_reg32_t)0x40004018U;
ro_reg32_t const SPDIFRX_VERR  = (ro_reg32_t)0x400043F4U;
ro_reg32_t const SPDIFRX_IDR   = (ro_reg32_t)0x400043F8U;
ro_reg32_t const SPDIFRX_SIDR  = (ro_reg32_t)0x400043FCU;
ro_reg32_t const SPDIFRX_DR_01 = (ro_reg32_t)0x40004010U;
ro_reg32_t const SPDIFRX_DR_10 = (ro_reg32_t)0x40004010U;

/** @subsection SPDIFRX Register Field Definitions */

const field32_t SPDIFRX_CR_SPDIFRXEN = {.msk = 0x00000003U, .pos = 0};
const field32_t SPDIFRX_CR_RXDMAEN   = {.msk = 0x00000004U, .pos = 2};
const field32_t SPDIFRX_CR_RXSTEO    = {.msk = 0x00000008U, .pos = 3};
const field32_t SPDIFRX_CR_DRFMT     = {.msk = 0x00000030U, .pos = 4};
const field32_t SPDIFRX_CR_PMSK      = {.msk = 0x00000040U, .pos = 6};
const field32_t SPDIFRX_CR_VMSK      = {.msk = 0x00000080U, .pos = 7};
const field32_t SPDIFRX_CR_CUMSK     = {.msk = 0x00000100U, .pos = 8};
const field32_t SPDIFRX_CR_PTMSK     = {.msk = 0x00000200U, .pos = 9};
const field32_t SPDIFRX_CR_CBDMAEN   = {.msk = 0x00000400U, .pos = 10};
const field32_t SPDIFRX_CR_CHSEL     = {.msk = 0x00000800U, .pos = 11};
const field32_t SPDIFRX_CR_NBTR      = {.msk = 0x00003000U, .pos = 12};
const field32_t SPDIFRX_CR_WFA       = {.msk = 0x00004000U, .pos = 14};
const field32_t SPDIFRX_CR_INSEL     = {.msk = 0x00070000U, .pos = 16};
const field32_t SPDIFRX_CR_CKSEN     = {.msk = 0x00100000U, .pos = 20};
const field32_t SPDIFRX_CR_CKSBKPEN  = {.msk = 0x00200000U, .pos = 21};
const field32_t SPDIFRX_IMR_RXNEIE   = {.msk = 0x00000001U, .pos = 0};
const field32_t SPDIFRX_IMR_CSRNEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t SPDIFRX_IMR_PERRIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t SPDIFRX_IMR_OVRIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t SPDIFRX_IMR_SBLKIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t SPDIFRX_IMR_SYNCDIE  = {.msk = 0x00000020U, .pos = 5};
const field32_t SPDIFRX_IMR_IFEIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t SPDIFRX_SR_RXNE      = {.msk = 0x00000001U, .pos = 0};
const field32_t SPDIFRX_SR_CSRNE     = {.msk = 0x00000002U, .pos = 1};
const field32_t SPDIFRX_SR_PERR      = {.msk = 0x00000004U, .pos = 2};
const field32_t SPDIFRX_SR_OVR       = {.msk = 0x00000008U, .pos = 3};
const field32_t SPDIFRX_SR_SBD       = {.msk = 0x00000010U, .pos = 4};
const field32_t SPDIFRX_SR_SYNCD     = {.msk = 0x00000020U, .pos = 5};
const field32_t SPDIFRX_SR_FERR      = {.msk = 0x00000040U, .pos = 6};
const field32_t SPDIFRX_SR_SERR      = {.msk = 0x00000080U, .pos = 7};
const field32_t SPDIFRX_SR_TERR      = {.msk = 0x00000100U, .pos = 8};
const field32_t SPDIFRX_SR_WIDTH5    = {.msk = 0x7FFF0000U, .pos = 16};
const field32_t SPDIFRX_IFCR_PERRCF  = {.msk = 0x00000004U, .pos = 2};
const field32_t SPDIFRX_IFCR_OVRCF   = {.msk = 0x00000008U, .pos = 3};
const field32_t SPDIFRX_IFCR_SBDCF   = {.msk = 0x00000010U, .pos = 4};
const field32_t SPDIFRX_IFCR_SYNCDCF = {.msk = 0x00000020U, .pos = 5};
const field32_t SPDIFRX_DR_00_DR     = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t SPDIFRX_DR_00_PE     = {.msk = 0x01000000U, .pos = 24};
const field32_t SPDIFRX_DR_00_V      = {.msk = 0x02000000U, .pos = 25};
const field32_t SPDIFRX_DR_00_U      = {.msk = 0x04000000U, .pos = 26};
const field32_t SPDIFRX_DR_00_C      = {.msk = 0x08000000U, .pos = 27};
const field32_t SPDIFRX_DR_00_PT     = {.msk = 0x30000000U, .pos = 28};
const field32_t SPDIFRX_CSR_USR      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t SPDIFRX_CSR_CS       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t SPDIFRX_CSR_SOB      = {.msk = 0x01000000U, .pos = 24};
const field32_t SPDIFRX_DIR_THI      = {.msk = 0x00001FFFU, .pos = 0};
const field32_t SPDIFRX_DIR_TLO      = {.msk = 0x1FFF0000U, .pos = 16};
const field32_t SPDIFRX_VERR_MINREV  = {.msk = 0x0000000FU, .pos = 0};
const field32_t SPDIFRX_VERR_MAJREV  = {.msk = 0x000000F0U, .pos = 4};
const field32_t SPDIFRX_DR_01_PE     = {.msk = 0x00000001U, .pos = 0};
const field32_t SPDIFRX_DR_01_V      = {.msk = 0x00000002U, .pos = 1};
const field32_t SPDIFRX_DR_01_U      = {.msk = 0x00000004U, .pos = 2};
const field32_t SPDIFRX_DR_01_C      = {.msk = 0x00000008U, .pos = 3};
const field32_t SPDIFRX_DR_01_PT     = {.msk = 0x00000030U, .pos = 4};
const field32_t SPDIFRX_DR_01_DR     = {.msk = 0xFFFFFF00U, .pos = 8};

/** @subsection Enumerated SPDIFRX Register Field Definitions */

field32_t const SPDIFRX_DR_10_DRNLx[3] = {
  [1] = {.msk = 0x0000FFFFU, .pos = 0},
  [2] = {.msk = 0xFFFF0000U, .pos = 16},
};

/**************************************************************************************************
 * @section ADCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated ADCx Register Definitions */

rw_reg32_t const ADCx_ISR[4] = {
  [1] = (rw_reg32_t)0x40022000U,
  [2] = (rw_reg32_t)0x40022100U,
  [3] = (rw_reg32_t)0x58026000U,
};

rw_reg32_t const ADCx_IER[4] = {
  [1] = (rw_reg32_t)0x40022004U,
  [2] = (rw_reg32_t)0x40022104U,
  [3] = (rw_reg32_t)0x58026004U,
};

rw_reg32_t const ADCx_CR[4] = {
  [1] = (rw_reg32_t)0x40022008U,
  [2] = (rw_reg32_t)0x40022108U,
  [3] = (rw_reg32_t)0x58026008U,
};

rw_reg32_t const ADCx_CFGR[4] = {
  [1] = (rw_reg32_t)0x4002200CU,
  [2] = (rw_reg32_t)0x4002210CU,
  [3] = (rw_reg32_t)0x5802600CU,
};

rw_reg32_t const ADCx_CFGR2[4] = {
  [1] = (rw_reg32_t)0x40022010U,
  [2] = (rw_reg32_t)0x40022110U,
  [3] = (rw_reg32_t)0x58026010U,
};

rw_reg32_t const ADCx_SMPR1[4] = {
  [1] = (rw_reg32_t)0x40022014U,
  [2] = (rw_reg32_t)0x40022114U,
  [3] = (rw_reg32_t)0x58026014U,
};

rw_reg32_t const ADCx_SMPR2[4] = {
  [1] = (rw_reg32_t)0x40022018U,
  [2] = (rw_reg32_t)0x40022118U,
  [3] = (rw_reg32_t)0x58026018U,
};

rw_reg32_t const ADCx_LTR1[4] = {
  [1] = (rw_reg32_t)0x40022020U,
  [2] = (rw_reg32_t)0x40022120U,
  [3] = (rw_reg32_t)0x58026020U,
};

rw_reg32_t const ADCx_LHTR1[4] = {
  [1] = (rw_reg32_t)0x40022024U,
  [2] = (rw_reg32_t)0x40022124U,
  [3] = (rw_reg32_t)0x58026024U,
};

rw_reg32_t const ADCx_SQR1[4] = {
  [1] = (rw_reg32_t)0x40022030U,
  [2] = (rw_reg32_t)0x40022130U,
  [3] = (rw_reg32_t)0x58026030U,
};

rw_reg32_t const ADCx_SQR2[4] = {
  [1] = (rw_reg32_t)0x40022034U,
  [2] = (rw_reg32_t)0x40022134U,
  [3] = (rw_reg32_t)0x58026034U,
};

rw_reg32_t const ADCx_SQR3[4] = {
  [1] = (rw_reg32_t)0x40022038U,
  [2] = (rw_reg32_t)0x40022138U,
  [3] = (rw_reg32_t)0x58026038U,
};

rw_reg32_t const ADCx_SQR4[4] = {
  [1] = (rw_reg32_t)0x4002203CU,
  [2] = (rw_reg32_t)0x4002213CU,
  [3] = (rw_reg32_t)0x5802603CU,
};

ro_reg32_t const ADCx_DR[4] = {
  [1] = (ro_reg32_t)0x40022040U,
  [2] = (ro_reg32_t)0x40022140U,
  [3] = (ro_reg32_t)0x58026040U,
};

rw_reg32_t const ADCx_JSQR[4] = {
  [1] = (rw_reg32_t)0x4002204CU,
  [2] = (rw_reg32_t)0x4002214CU,
  [3] = (rw_reg32_t)0x5802604CU,
};

rw_reg32_t const ADCx_OFRx[4][5] = {
  [1] = {
    [1] = (rw_reg32_t)0x40022060U,
    [2] = (rw_reg32_t)0x40022064U,
    [3] = (rw_reg32_t)0x40022068U,
    [4] = (rw_reg32_t)0x4002206CU,
  },
  [2] = {
    [1] = (rw_reg32_t)0x40022160U,
    [2] = (rw_reg32_t)0x40022164U,
    [3] = (rw_reg32_t)0x40022168U,
    [4] = (rw_reg32_t)0x4002216CU,
  },
  [3] = {
    [1] = (rw_reg32_t)0x58026060U,
    [2] = (rw_reg32_t)0x58026064U,
    [3] = (rw_reg32_t)0x58026068U,
    [4] = (rw_reg32_t)0x5802606CU,
  },
};

ro_reg32_t const ADCx_JDR1[4] = {
  [1] = (ro_reg32_t)0x40022080U,
  [2] = (ro_reg32_t)0x40022180U,
  [3] = (ro_reg32_t)0x58026080U,
};

ro_reg32_t const ADCx_JDR2[4] = {
  [1] = (ro_reg32_t)0x40022084U,
  [2] = (ro_reg32_t)0x40022184U,
  [3] = (ro_reg32_t)0x58026084U,
};

ro_reg32_t const ADCx_JDR3[4] = {
  [1] = (ro_reg32_t)0x40022088U,
  [2] = (ro_reg32_t)0x40022188U,
  [3] = (ro_reg32_t)0x58026088U,
};

ro_reg32_t const ADCx_JDR4[4] = {
  [1] = (ro_reg32_t)0x4002208CU,
  [2] = (ro_reg32_t)0x4002218CU,
  [3] = (ro_reg32_t)0x5802608CU,
};

rw_reg32_t const ADCx_AWD2CR[4] = {
  [1] = (rw_reg32_t)0x400220A0U,
  [2] = (rw_reg32_t)0x400221A0U,
  [3] = (rw_reg32_t)0x580260A0U,
};

rw_reg32_t const ADCx_AWD3CR[4] = {
  [1] = (rw_reg32_t)0x400220A4U,
  [2] = (rw_reg32_t)0x400221A4U,
  [3] = (rw_reg32_t)0x580260A4U,
};

rw_reg32_t const ADCx_DIFSEL[4] = {
  [1] = (rw_reg32_t)0x400220C0U,
  [2] = (rw_reg32_t)0x400221C0U,
  [3] = (rw_reg32_t)0x580260C0U,
};

rw_reg32_t const ADCx_CALFACT[4] = {
  [1] = (rw_reg32_t)0x400220C4U,
  [2] = (rw_reg32_t)0x400221C4U,
  [3] = (rw_reg32_t)0x580260C4U,
};

rw_reg32_t const ADCx_PCSEL[4] = {
  [1] = (rw_reg32_t)0x4002201CU,
  [2] = (rw_reg32_t)0x4002211CU,
  [3] = (rw_reg32_t)0x5802601CU,
};

rw_reg32_t const ADCx_LTR2[4] = {
  [1] = (rw_reg32_t)0x400220B0U,
  [2] = (rw_reg32_t)0x400221B0U,
  [3] = (rw_reg32_t)0x580260B0U,
};

rw_reg32_t const ADCx_HTR2[4] = {
  [1] = (rw_reg32_t)0x400220B4U,
  [2] = (rw_reg32_t)0x400221B4U,
  [3] = (rw_reg32_t)0x580260B4U,
};

rw_reg32_t const ADCx_LTR3[4] = {
  [1] = (rw_reg32_t)0x400220B8U,
  [2] = (rw_reg32_t)0x400221B8U,
  [3] = (rw_reg32_t)0x580260B8U,
};

rw_reg32_t const ADCx_HTR3[4] = {
  [1] = (rw_reg32_t)0x400220BCU,
  [2] = (rw_reg32_t)0x400221BCU,
  [3] = (rw_reg32_t)0x580260BCU,
};

rw_reg32_t const ADCx_CALFACT2[4] = {
  [1] = (rw_reg32_t)0x400220C8U,
  [2] = (rw_reg32_t)0x400221C8U,
  [3] = (rw_reg32_t)0x580260C8U,
};

/** @subsection ADCx Register Field Definitions */

const field32_t ADCx_ISR_JQOVF           = {.msk = 0x00000400U, .pos = 10};
const field32_t ADCx_ISR_JEOS            = {.msk = 0x00000040U, .pos = 6};
const field32_t ADCx_ISR_JEOC            = {.msk = 0x00000020U, .pos = 5};
const field32_t ADCx_ISR_OVR             = {.msk = 0x00000010U, .pos = 4};
const field32_t ADCx_ISR_EOS             = {.msk = 0x00000008U, .pos = 3};
const field32_t ADCx_ISR_EOC             = {.msk = 0x00000004U, .pos = 2};
const field32_t ADCx_ISR_EOSMP           = {.msk = 0x00000002U, .pos = 1};
const field32_t ADCx_ISR_ADRDY           = {.msk = 0x00000001U, .pos = 0};
const field32_t ADCx_IER_JQOVFIE         = {.msk = 0x00000400U, .pos = 10};
const field32_t ADCx_IER_JEOSIE          = {.msk = 0x00000040U, .pos = 6};
const field32_t ADCx_IER_JEOCIE          = {.msk = 0x00000020U, .pos = 5};
const field32_t ADCx_IER_OVRIE           = {.msk = 0x00000010U, .pos = 4};
const field32_t ADCx_IER_EOSIE           = {.msk = 0x00000008U, .pos = 3};
const field32_t ADCx_IER_EOCIE           = {.msk = 0x00000004U, .pos = 2};
const field32_t ADCx_IER_EOSMPIE         = {.msk = 0x00000002U, .pos = 1};
const field32_t ADCx_IER_ADRDYIE         = {.msk = 0x00000001U, .pos = 0};
const field32_t ADCx_CR_ADCAL            = {.msk = 0x80000000U, .pos = 31};
const field32_t ADCx_CR_ADCALDIF         = {.msk = 0x40000000U, .pos = 30};
const field32_t ADCx_CR_DEEPPWD          = {.msk = 0x20000000U, .pos = 29};
const field32_t ADCx_CR_ADVREGEN         = {.msk = 0x10000000U, .pos = 28};
const field32_t ADCx_CR_ADCALLIN         = {.msk = 0x00010000U, .pos = 16};
const field32_t ADCx_CR_BOOST            = {.msk = 0x00000300U, .pos = 8};
const field32_t ADCx_CR_JADSTP           = {.msk = 0x00000020U, .pos = 5};
const field32_t ADCx_CR_ADSTP            = {.msk = 0x00000010U, .pos = 4};
const field32_t ADCx_CR_JADSTART         = {.msk = 0x00000008U, .pos = 3};
const field32_t ADCx_CR_ADSTART          = {.msk = 0x00000004U, .pos = 2};
const field32_t ADCx_CR_ADDIS            = {.msk = 0x00000002U, .pos = 1};
const field32_t ADCx_CR_ADEN             = {.msk = 0x00000001U, .pos = 0};
const field32_t ADCx_CFGR_JQDIS          = {.msk = 0x80000000U, .pos = 31};
const field32_t ADCx_CFGR_AWDCH1CH       = {.msk = 0x7C000000U, .pos = 26};
const field32_t ADCx_CFGR_JAUTO          = {.msk = 0x02000000U, .pos = 25};
const field32_t ADCx_CFGR_JAWD1EN        = {.msk = 0x01000000U, .pos = 24};
const field32_t ADCx_CFGR_AWD1EN         = {.msk = 0x00800000U, .pos = 23};
const field32_t ADCx_CFGR_AWD1SGL        = {.msk = 0x00400000U, .pos = 22};
const field32_t ADCx_CFGR_JQM            = {.msk = 0x00200000U, .pos = 21};
const field32_t ADCx_CFGR_JDISCEN        = {.msk = 0x00100000U, .pos = 20};
const field32_t ADCx_CFGR_DISCNUM        = {.msk = 0x000E0000U, .pos = 17};
const field32_t ADCx_CFGR_DISCEN         = {.msk = 0x00010000U, .pos = 16};
const field32_t ADCx_CFGR_AUTDLY         = {.msk = 0x00004000U, .pos = 14};
const field32_t ADCx_CFGR_CONT           = {.msk = 0x00002000U, .pos = 13};
const field32_t ADCx_CFGR_OVRMOD         = {.msk = 0x00001000U, .pos = 12};
const field32_t ADCx_CFGR_EXTEN          = {.msk = 0x00000C00U, .pos = 10};
const field32_t ADCx_CFGR_EXTSEL         = {.msk = 0x000003E0U, .pos = 5};
const field32_t ADCx_CFGR_RES            = {.msk = 0x0000001CU, .pos = 2};
const field32_t ADCx_CFGR_DMNGT          = {.msk = 0x00000003U, .pos = 0};
const field32_t ADCx_CFGR2_ROVSE         = {.msk = 0x00000001U, .pos = 0};
const field32_t ADCx_CFGR2_JOVSE         = {.msk = 0x00000002U, .pos = 1};
const field32_t ADCx_CFGR2_OVSS          = {.msk = 0x000001E0U, .pos = 5};
const field32_t ADCx_CFGR2_TROVS         = {.msk = 0x00000200U, .pos = 9};
const field32_t ADCx_CFGR2_ROVSM         = {.msk = 0x00000400U, .pos = 10};
const field32_t ADCx_CFGR2_OSR           = {.msk = 0x03FF0000U, .pos = 16};
const field32_t ADCx_CFGR2_LSHIFT        = {.msk = 0xF0000000U, .pos = 28};
const field32_t ADCx_LTR1_LTR1           = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_LHTR1_LHTR1         = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_SQR1_L3             = {.msk = 0x0000000FU, .pos = 0};
const field32_t ADCx_DR_RDATA            = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ADCx_JSQR_JEXTEN         = {.msk = 0x00000180U, .pos = 7};
const field32_t ADCx_JSQR_JEXTSEL        = {.msk = 0x0000007CU, .pos = 2};
const field32_t ADCx_JSQR_JL             = {.msk = 0x00000003U, .pos = 0};
const field32_t ADCx_OFRx_SSATE          = {.msk = 0x80000000U, .pos = 31};
const field32_t ADCx_OFRx_OFFSET1_CH     = {.msk = 0x7C000000U, .pos = 26};
const field32_t ADCx_OFRx_OFFSET1        = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_AWD2CR_AWD2CH       = {.msk = 0x000FFFFFU, .pos = 0};
const field32_t ADCx_AWD3CR_AWD3CH       = {.msk = 0x001FFFFEU, .pos = 1};
const field32_t ADCx_DIFSEL_DIFSEL       = {.msk = 0x000FFFFFU, .pos = 0};
const field32_t ADCx_CALFACT_CALFACT_D   = {.msk = 0x07FF0000U, .pos = 16};
const field32_t ADCx_CALFACT_CALFACT_S   = {.msk = 0x000007FFU, .pos = 0};
const field32_t ADCx_PCSEL_PCSEL         = {.msk = 0x000FFFFFU, .pos = 0};
const field32_t ADCx_LTR2_LTR2           = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_HTR2_HTR2           = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_LTR3_LTR3           = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_HTR3_HTR3           = {.msk = 0x03FFFFFFU, .pos = 0};
const field32_t ADCx_CALFACT2_LINCALFACT = {.msk = 0x3FFFFFFFU, .pos = 0};

/** @subsection Enumerated ADCx Register Field Definitions */

field32_t const ADCx_ISR_AWDx[4] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
  [3] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const ADCx_IER_AWDxIE[4] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
  [3] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const ADCx_CR_LINCALRDYWx[7] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
  [4] = {.msk = 0x02000000U, .pos = 25},
  [5] = {.msk = 0x04000000U, .pos = 26},
  [6] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const ADCx_CFGR2_RSHIFTx[5] = {
  [1] = {.msk = 0x00000800U, .pos = 11},
  [2] = {.msk = 0x00001000U, .pos = 12},
  [3] = {.msk = 0x00002000U, .pos = 13},
  [4] = {.msk = 0x00004000U, .pos = 14},
};

field32_t const ADCx_SMPR1_SMPx[10] = {
  [1]  = {.msk = 0x00000038U, .pos = 3},
  [2]  = {.msk = 0x000001C0U, .pos = 6},
  [3]  = {.msk = 0x00000E00U, .pos = 9},
  [4]  = {.msk = 0x00007000U, .pos = 12},
  [5]  = {.msk = 0x00038000U, .pos = 15},
  [6]  = {.msk = 0x001C0000U, .pos = 18},
  [7]  = {.msk = 0x00E00000U, .pos = 21},
  [8]  = {.msk = 0x07000000U, .pos = 24},
  [9]  = {.msk = 0x38000000U, .pos = 27},
};

field32_t const ADCx_SMPR2_SMP1x[10] = {
  [0]  = {.msk = 0x00000007U, .pos = 0},
  [1]  = {.msk = 0x00000038U, .pos = 3},
  [2]  = {.msk = 0x000001C0U, .pos = 6},
  [3]  = {.msk = 0x00000E00U, .pos = 9},
  [4]  = {.msk = 0x00007000U, .pos = 12},
  [5]  = {.msk = 0x00038000U, .pos = 15},
  [6]  = {.msk = 0x001C0000U, .pos = 18},
  [7]  = {.msk = 0x00E00000U, .pos = 21},
  [8]  = {.msk = 0x07000000U, .pos = 24},
  [9]  = {.msk = 0x38000000U, .pos = 27},
};

field32_t const ADCx_SQR1_SQx[5] = {
  [1] = {.msk = 0x000007C0U, .pos = 6},
  [2] = {.msk = 0x0001F000U, .pos = 12},
  [3] = {.msk = 0x007C0000U, .pos = 18},
  [4] = {.msk = 0x1F000000U, .pos = 24},
};

field32_t const ADCx_SQR3_SQ1x[5] = {
  [0] = {.msk = 0x0000001FU, .pos = 0},
  [1] = {.msk = 0x000007C0U, .pos = 6},
  [2] = {.msk = 0x0001F000U, .pos = 12},
  [3] = {.msk = 0x007C0000U, .pos = 18},
  [4] = {.msk = 0x1F000000U, .pos = 24},
};

field32_t const ADCx_JSQR_JSQx[5] = {
  [1] = {.msk = 0x00003E00U, .pos = 9},
  [2] = {.msk = 0x000F8000U, .pos = 15},
  [3] = {.msk = 0x03E00000U, .pos = 21},
  [4] = {.msk = 0xF8000000U, .pos = 27},
};

/**************************************************************************************************
 * @section ADCx_COMMON Definitions
 **************************************************************************************************/

/** @subsection Enumerated ADCx_COMMON Register Definitions */

ro_reg32_t const ADCx_COMMON_CSR[13] = {
  [3]  = (ro_reg32_t)0x58026300U,
  [12] = (ro_reg32_t)0x40022300U,
};

rw_reg32_t const ADCx_COMMON_CCR[13] = {
  [3]  = (rw_reg32_t)0x58026308U,
  [12] = (rw_reg32_t)0x40022308U,
};

ro_reg32_t const ADCx_COMMON_CDR[13] = {
  [3]  = (ro_reg32_t)0x5802630CU,
  [12] = (ro_reg32_t)0x4002230CU,
};

ro_reg32_t const ADCx_COMMON_CDR2[13] = {
  [3]  = (ro_reg32_t)0x58026310U,
  [12] = (ro_reg32_t)0x40022310U,
};

/** @subsection ADCx_COMMON Register Field Definitions */

const field32_t ADCx_COMMON_CSR_ADRDY_MST = {.msk = 0x00000001U, .pos = 0};
const field32_t ADCx_COMMON_CSR_EOSMP_MST = {.msk = 0x00000002U, .pos = 1};
const field32_t ADCx_COMMON_CSR_EOC_MST   = {.msk = 0x00000004U, .pos = 2};
const field32_t ADCx_COMMON_CSR_EOS_MST   = {.msk = 0x00000008U, .pos = 3};
const field32_t ADCx_COMMON_CSR_OVR_MST   = {.msk = 0x00000010U, .pos = 4};
const field32_t ADCx_COMMON_CSR_JEOC_MST  = {.msk = 0x00000020U, .pos = 5};
const field32_t ADCx_COMMON_CSR_JEOS_MST  = {.msk = 0x00000040U, .pos = 6};
const field32_t ADCx_COMMON_CSR_JQOVF_MST = {.msk = 0x00000400U, .pos = 10};
const field32_t ADCx_COMMON_CSR_ADRDY_SLV = {.msk = 0x00010000U, .pos = 16};
const field32_t ADCx_COMMON_CSR_EOSMP_SLV = {.msk = 0x00020000U, .pos = 17};
const field32_t ADCx_COMMON_CSR_EOC_SLV   = {.msk = 0x00040000U, .pos = 18};
const field32_t ADCx_COMMON_CSR_EOS_SLV   = {.msk = 0x00080000U, .pos = 19};
const field32_t ADCx_COMMON_CSR_OVR_SLV   = {.msk = 0x00100000U, .pos = 20};
const field32_t ADCx_COMMON_CSR_JEOC_SLV  = {.msk = 0x00200000U, .pos = 21};
const field32_t ADCx_COMMON_CSR_JEOS_SLV  = {.msk = 0x00400000U, .pos = 22};
const field32_t ADCx_COMMON_CSR_JQOVF_SLV = {.msk = 0x04000000U, .pos = 26};
const field32_t ADCx_COMMON_CCR_DUAL      = {.msk = 0x0000001FU, .pos = 0};
const field32_t ADCx_COMMON_CCR_DELAY     = {.msk = 0x00000F00U, .pos = 8};
const field32_t ADCx_COMMON_CCR_DAMDF     = {.msk = 0x0000C000U, .pos = 14};
const field32_t ADCx_COMMON_CCR_CKMODE    = {.msk = 0x00030000U, .pos = 16};
const field32_t ADCx_COMMON_CCR_PRESC     = {.msk = 0x003C0000U, .pos = 18};
const field32_t ADCx_COMMON_CCR_VREFEN    = {.msk = 0x00400000U, .pos = 22};
const field32_t ADCx_COMMON_CCR_TSEN      = {.msk = 0x00800000U, .pos = 23};
const field32_t ADCx_COMMON_CCR_VBATEN    = {.msk = 0x01000000U, .pos = 24};
const field32_t ADCx_COMMON_CDR_RDATA_SLV = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t ADCx_COMMON_CDR_RDATA_MST = {.msk = 0x0000FFFFU, .pos = 0};

/** @subsection Enumerated ADCx_COMMON Register Field Definitions */

field32_t const ADCx_COMMON_CSR_AWDx_MST[4] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
  [3] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const ADCx_COMMON_CSR_AWDx_SLV[4] = {
  [1] = {.msk = 0x00800000U, .pos = 23},
  [2] = {.msk = 0x01000000U, .pos = 24},
  [3] = {.msk = 0x02000000U, .pos = 25},
};

/**************************************************************************************************
 * @section CRC Definitions
 **************************************************************************************************/

/** @subsection CRC Register Definitions */

rw_reg32_t const CRC_DR   = (rw_reg32_t)0x58024C00U;
rw_reg32_t const CRC_IDR  = (rw_reg32_t)0x58024C04U;
rw_reg32_t const CRC_CR   = (rw_reg32_t)0x58024C08U;
rw_reg32_t const CRC_INIT = (rw_reg32_t)0x58024C0CU;
rw_reg32_t const CRC_POL  = (rw_reg32_t)0x58024C10U;

/** @subsection CRC Register Field Definitions */

const field32_t CRC_CR_RESET    = {.msk = 0x00000001U, .pos = 0};
const field32_t CRC_CR_POLYSIZE = {.msk = 0x00000018U, .pos = 3};
const field32_t CRC_CR_REV_IN   = {.msk = 0x00000060U, .pos = 5};
const field32_t CRC_CR_REV_OUT  = {.msk = 0x00000080U, .pos = 7};

/**************************************************************************************************
 * @section RCC Definitions
 **************************************************************************************************/

/** @subsection RCC Register Definitions */

rw_reg32_t const RCC_CR            = (rw_reg32_t)0x58024400U;
rw_reg32_t const RCC_ICSCR         = (rw_reg32_t)0x58024404U;
ro_reg32_t const RCC_CRRCR         = (ro_reg32_t)0x58024408U;
rw_reg32_t const RCC_CFGR          = (rw_reg32_t)0x58024410U;
rw_reg32_t const RCC_D1CFGR        = (rw_reg32_t)0x58024418U;
rw_reg32_t const RCC_D2CFGR        = (rw_reg32_t)0x5802441CU;
rw_reg32_t const RCC_D3CFGR        = (rw_reg32_t)0x58024420U;
rw_reg32_t const RCC_PLLCKSELR     = (rw_reg32_t)0x58024428U;
rw_reg32_t const RCC_PLLCFGR       = (rw_reg32_t)0x5802442CU;
rw_reg32_t const RCC_PLL1DIVR      = (rw_reg32_t)0x58024430U;
rw_reg32_t const RCC_PLL1FRACR     = (rw_reg32_t)0x58024434U;
rw_reg32_t const RCC_PLL2DIVR      = (rw_reg32_t)0x58024438U;
rw_reg32_t const RCC_PLL2FRACR     = (rw_reg32_t)0x5802443CU;
rw_reg32_t const RCC_PLL3DIVR      = (rw_reg32_t)0x58024440U;
rw_reg32_t const RCC_PLL3FRACR     = (rw_reg32_t)0x58024444U;
rw_reg32_t const RCC_D1CCIPR       = (rw_reg32_t)0x5802444CU;
rw_reg32_t const RCC_D2CCIP1R      = (rw_reg32_t)0x58024450U;
rw_reg32_t const RCC_D2CCIP2R      = (rw_reg32_t)0x58024454U;
rw_reg32_t const RCC_D3CCIPR       = (rw_reg32_t)0x58024458U;
rw_reg32_t const RCC_CIER          = (rw_reg32_t)0x58024460U;
rw_reg32_t const RCC_CIFR          = (rw_reg32_t)0x58024464U;
rw_reg32_t const RCC_CICR          = (rw_reg32_t)0x58024468U;
rw_reg32_t const RCC_BDCR          = (rw_reg32_t)0x58024470U;
rw_reg32_t const RCC_CSR           = (rw_reg32_t)0x58024474U;
rw_reg32_t const RCC_AHB3RSTR      = (rw_reg32_t)0x5802447CU;
rw_reg32_t const RCC_AHB1RSTR      = (rw_reg32_t)0x58024480U;
rw_reg32_t const RCC_AHB2RSTR      = (rw_reg32_t)0x58024484U;
rw_reg32_t const RCC_AHB4RSTR      = (rw_reg32_t)0x58024488U;
rw_reg32_t const RCC_APB3RSTR      = (rw_reg32_t)0x5802448CU;
rw_reg32_t const RCC_APB1LRSTR     = (rw_reg32_t)0x58024490U;
rw_reg32_t const RCC_APB1HRSTR     = (rw_reg32_t)0x58024494U;
rw_reg32_t const RCC_APB2RSTR      = (rw_reg32_t)0x58024498U;
rw_reg32_t const RCC_APB4RSTR      = (rw_reg32_t)0x5802449CU;
rw_reg32_t const RCC_GCR           = (rw_reg32_t)0x580244A0U;
rw_reg32_t const RCC_D3AMR         = (rw_reg32_t)0x580244A8U;
rw_reg32_t const RCC_RSR           = (rw_reg32_t)0x580244D0U;
rw_reg32_t const RCC_C1_RSR        = (rw_reg32_t)0x58024530U;
rw_reg32_t const RCC_C1_AHB3ENR    = (rw_reg32_t)0x58024534U;
rw_reg32_t const RCC_AHB3ENR       = (rw_reg32_t)0x580244D4U;
rw_reg32_t const RCC_AHB1ENR       = (rw_reg32_t)0x580244D8U;
rw_reg32_t const RCC_C1_AHB1ENR    = (rw_reg32_t)0x58024538U;
rw_reg32_t const RCC_C1_AHB2ENR    = (rw_reg32_t)0x5802453CU;
rw_reg32_t const RCC_AHB2ENR       = (rw_reg32_t)0x580244DCU;
rw_reg32_t const RCC_AHB4ENR       = (rw_reg32_t)0x580244E0U;
rw_reg32_t const RCC_C1_AHB4ENR    = (rw_reg32_t)0x58024540U;
rw_reg32_t const RCC_C1_APB3ENR    = (rw_reg32_t)0x58024544U;
rw_reg32_t const RCC_APB3ENR       = (rw_reg32_t)0x580244E4U;
rw_reg32_t const RCC_APB1LENR      = (rw_reg32_t)0x580244E8U;
rw_reg32_t const RCC_C1_APB1LENR   = (rw_reg32_t)0x58024548U;
rw_reg32_t const RCC_APB1HENR      = (rw_reg32_t)0x580244ECU;
rw_reg32_t const RCC_C1_APB1HENR   = (rw_reg32_t)0x5802454CU;
rw_reg32_t const RCC_C1_APB2ENR    = (rw_reg32_t)0x58024550U;
rw_reg32_t const RCC_APB2ENR       = (rw_reg32_t)0x580244F0U;
rw_reg32_t const RCC_APB4ENR       = (rw_reg32_t)0x580244F4U;
rw_reg32_t const RCC_C1_APB4ENR    = (rw_reg32_t)0x58024554U;
rw_reg32_t const RCC_C1_AHB3LPENR  = (rw_reg32_t)0x5802455CU;
rw_reg32_t const RCC_AHB3LPENR     = (rw_reg32_t)0x580244FCU;
rw_reg32_t const RCC_AHB1LPENR     = (rw_reg32_t)0x58024500U;
rw_reg32_t const RCC_C1_AHB1LPENR  = (rw_reg32_t)0x58024560U;
rw_reg32_t const RCC_C1_AHB2LPENR  = (rw_reg32_t)0x58024564U;
rw_reg32_t const RCC_AHB2LPENR     = (rw_reg32_t)0x58024504U;
rw_reg32_t const RCC_AHB4LPENR     = (rw_reg32_t)0x58024508U;
rw_reg32_t const RCC_C1_AHB4LPENR  = (rw_reg32_t)0x58024568U;
rw_reg32_t const RCC_C1_APB3LPENR  = (rw_reg32_t)0x5802456CU;
rw_reg32_t const RCC_APB3LPENR     = (rw_reg32_t)0x5802450CU;
rw_reg32_t const RCC_APB1LLPENR    = (rw_reg32_t)0x58024510U;
rw_reg32_t const RCC_C1_APB1LLPENR = (rw_reg32_t)0x58024570U;
rw_reg32_t const RCC_C1_APB1HLPENR = (rw_reg32_t)0x58024574U;
rw_reg32_t const RCC_APB1HLPENR    = (rw_reg32_t)0x58024514U;
rw_reg32_t const RCC_APB2LPENR     = (rw_reg32_t)0x58024518U;
rw_reg32_t const RCC_C1_APB2LPENR  = (rw_reg32_t)0x58024578U;
rw_reg32_t const RCC_C1_APB4LPENR  = (rw_reg32_t)0x5802457CU;
rw_reg32_t const RCC_APB4LPENR     = (rw_reg32_t)0x5802451CU;

/** @subsection RCC Register Field Definitions */

const field32_t RCC_CR_HSION                       = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_CR_HSIKERON                    = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_CR_HSIRDY                      = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_CR_HSIDIV                      = {.msk = 0x00000018U, .pos = 3};
const field32_t RCC_CR_HSIDIVF                     = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_CR_CSION                       = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_CR_CSIRDY                      = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_CR_CSIKERON                    = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_CR_RC48ON                      = {.msk = 0x00001000U, .pos = 12};
const field32_t RCC_CR_RC48RDY                     = {.msk = 0x00002000U, .pos = 13};
const field32_t RCC_CR_HSEON                       = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_CR_HSERDY                      = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_CR_HSEBYP                      = {.msk = 0x00040000U, .pos = 18};
const field32_t RCC_CR_HSECSSON                    = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_ICSCR_HSICAL                   = {.msk = 0x00000FFFU, .pos = 0};
const field32_t RCC_ICSCR_HSITRIM                  = {.msk = 0x0003F000U, .pos = 12};
const field32_t RCC_ICSCR_CSICAL                   = {.msk = 0x03FC0000U, .pos = 18};
const field32_t RCC_ICSCR_CSITRIM                  = {.msk = 0x7C000000U, .pos = 26};
const field32_t RCC_CRRCR_RC48CAL                  = {.msk = 0x000003FFU, .pos = 0};
const field32_t RCC_CFGR_SW                        = {.msk = 0x00000007U, .pos = 0};
const field32_t RCC_CFGR_SWS                       = {.msk = 0x00000038U, .pos = 3};
const field32_t RCC_CFGR_STOPWUCK                  = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_CFGR_STOPKERWUCK               = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_CFGR_RTCPRE                    = {.msk = 0x00003F00U, .pos = 8};
const field32_t RCC_CFGR_HRTIMSEL                  = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_CFGR_TIMPRE                    = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_D1CFGR_HPRE                    = {.msk = 0x0000000FU, .pos = 0};
const field32_t RCC_D1CFGR_D1PPRE                  = {.msk = 0x00000070U, .pos = 4};
const field32_t RCC_D1CFGR_D1CPRE                  = {.msk = 0x00000F00U, .pos = 8};
const field32_t RCC_D3CFGR_D3PPRE                  = {.msk = 0x00000070U, .pos = 4};
const field32_t RCC_PLLCKSELR_PLLSRC               = {.msk = 0x00000003U, .pos = 0};
const field32_t RCC_PLL1DIVR_DIVN1                 = {.msk = 0x000001FFU, .pos = 0};
const field32_t RCC_PLL1DIVR_DIVP1                 = {.msk = 0x0000FE00U, .pos = 9};
const field32_t RCC_PLL1DIVR_DIVQ1                 = {.msk = 0x007F0000U, .pos = 16};
const field32_t RCC_PLL1DIVR_DIVR1                 = {.msk = 0x7F000000U, .pos = 24};
const field32_t RCC_PLL1FRACR_FRACN1               = {.msk = 0x0000FFF8U, .pos = 3};
const field32_t RCC_PLL2DIVR_DIVN1                 = {.msk = 0x000001FFU, .pos = 0};
const field32_t RCC_PLL2DIVR_DIVP1                 = {.msk = 0x0000FE00U, .pos = 9};
const field32_t RCC_PLL2DIVR_DIVQ1                 = {.msk = 0x007F0000U, .pos = 16};
const field32_t RCC_PLL2DIVR_DIVR1                 = {.msk = 0x7F000000U, .pos = 24};
const field32_t RCC_PLL2FRACR_FRACN2               = {.msk = 0x0000FFF8U, .pos = 3};
const field32_t RCC_PLL3DIVR_DIVN3                 = {.msk = 0x000001FFU, .pos = 0};
const field32_t RCC_PLL3DIVR_DIVP3                 = {.msk = 0x0000FE00U, .pos = 9};
const field32_t RCC_PLL3DIVR_DIVQ3                 = {.msk = 0x007F0000U, .pos = 16};
const field32_t RCC_PLL3DIVR_DIVR3                 = {.msk = 0x7F000000U, .pos = 24};
const field32_t RCC_PLL3FRACR_FRACN3               = {.msk = 0x0000FFF8U, .pos = 3};
const field32_t RCC_D1CCIPR_FMCSRC                 = {.msk = 0x00000003U, .pos = 0};
const field32_t RCC_D1CCIPR_QSPISRC                = {.msk = 0x00000030U, .pos = 4};
const field32_t RCC_D1CCIPR_SDMMCSRC               = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_D1CCIPR_CKPERSRC               = {.msk = 0x30000000U, .pos = 28};
const field32_t RCC_D2CCIP1R_SAI1SRC               = {.msk = 0x00000007U, .pos = 0};
const field32_t RCC_D2CCIP1R_SAI23SRC              = {.msk = 0x000001C0U, .pos = 6};
const field32_t RCC_D2CCIP1R_SPI123SRC             = {.msk = 0x00007000U, .pos = 12};
const field32_t RCC_D2CCIP1R_SPI45SRC              = {.msk = 0x00070000U, .pos = 16};
const field32_t RCC_D2CCIP1R_SPDIFSRC              = {.msk = 0x00300000U, .pos = 20};
const field32_t RCC_D2CCIP1R_DFSDM1SRC             = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_D2CCIP1R_FDCANSRC              = {.msk = 0x30000000U, .pos = 28};
const field32_t RCC_D2CCIP1R_SWPSRC                = {.msk = 0x80000000U, .pos = 31};
const field32_t RCC_D2CCIP2R_USART234578SRC        = {.msk = 0x00000007U, .pos = 0};
const field32_t RCC_D2CCIP2R_USART16SRC            = {.msk = 0x00000038U, .pos = 3};
const field32_t RCC_D2CCIP2R_RNGSRC                = {.msk = 0x00000300U, .pos = 8};
const field32_t RCC_D2CCIP2R_I2C123SRC             = {.msk = 0x00003000U, .pos = 12};
const field32_t RCC_D2CCIP2R_USBSRC                = {.msk = 0x00300000U, .pos = 20};
const field32_t RCC_D2CCIP2R_CECSRC                = {.msk = 0x00C00000U, .pos = 22};
const field32_t RCC_D2CCIP2R_LPTIM1SRC             = {.msk = 0x70000000U, .pos = 28};
const field32_t RCC_D3CCIPR_LPUART1SRC             = {.msk = 0x00000007U, .pos = 0};
const field32_t RCC_D3CCIPR_I2C4SRC                = {.msk = 0x00000300U, .pos = 8};
const field32_t RCC_D3CCIPR_LPTIM2SRC              = {.msk = 0x00001C00U, .pos = 10};
const field32_t RCC_D3CCIPR_LPTIM345SRC            = {.msk = 0x0000E000U, .pos = 13};
const field32_t RCC_D3CCIPR_ADCSRC                 = {.msk = 0x00030000U, .pos = 16};
const field32_t RCC_D3CCIPR_SAI4ASRC               = {.msk = 0x00E00000U, .pos = 21};
const field32_t RCC_D3CCIPR_SAI4BSRC               = {.msk = 0x07000000U, .pos = 24};
const field32_t RCC_D3CCIPR_SPI6SRC                = {.msk = 0x70000000U, .pos = 28};
const field32_t RCC_CIER_LSIRDYIE                  = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_CIER_LSERDYIE                  = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_CIER_HSIRDYIE                  = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_CIER_HSERDYIE                  = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_CIER_CSIRDYIE                  = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_CIER_RC48RDYIE                 = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_CIER_LSECSSIE                  = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_CIFR_LSIRDYF                   = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_CIFR_LSERDYF                   = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_CIFR_HSIRDYF                   = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_CIFR_HSERDYF                   = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_CIFR_CSIRDY                    = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_CIFR_RC48RDYF                  = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_CIFR_LSECSSF                   = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_CIFR_HSECSSF                   = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_CICR_LSIRDYC                   = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_CICR_LSERDYC                   = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_CICR_HSIRDYC                   = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_CICR_HSERDYC                   = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_CICR_HSE_READY_INTERRUPT_CLEAR = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_CICR_RC48RDYC                  = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_CICR_LSECSSC                   = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_CICR_HSECSSC                   = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_BDCR_LSEON                     = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_BDCR_LSERDY                    = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_BDCR_LSEBYP                    = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_BDCR_LSEDRV                    = {.msk = 0x00000018U, .pos = 3};
const field32_t RCC_BDCR_LSECSSON                  = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_BDCR_LSECSSD                   = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_BDCR_RTCSRC                    = {.msk = 0x00000300U, .pos = 8};
const field32_t RCC_BDCR_RTCEN                     = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_BDCR_VSWRST                    = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_CSR_LSION                      = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_CSR_LSIRDY                     = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_AHB3RSTR_MDMARST               = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB3RSTR_DMA2DRST              = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB3RSTR_JPGDECRST             = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB3RSTR_FMCRST                = {.msk = 0x00001000U, .pos = 12};
const field32_t RCC_AHB3RSTR_QSPIRST               = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_AHB3RSTR_SDMMC1RST             = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_AHB3RSTR_CPURST                = {.msk = 0x80000000U, .pos = 31};
const field32_t RCC_AHB1RSTR_ADC12RST              = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB1RSTR_ETH1MACRST            = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_AHB2RSTR_CAMITFRST             = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB2RSTR_CRYPTRST              = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB2RSTR_HASHRST               = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB2RSTR_RNGRST                = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB2RSTR_SDMMC2RST             = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB4RSTR_GPIOARST              = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB4RSTR_GPIOBRST              = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_AHB4RSTR_GPIOCRST              = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_AHB4RSTR_GPIODRST              = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_AHB4RSTR_GPIOERST              = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB4RSTR_GPIOFRST              = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB4RSTR_GPIOGRST              = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB4RSTR_GPIOHRST              = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_AHB4RSTR_GPIOIRST              = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_AHB4RSTR_GPIOJRST              = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB4RSTR_GPIOKRST              = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_AHB4RSTR_CRCRST                = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_AHB4RSTR_BDMARST               = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_AHB4RSTR_ADC3RST               = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_AHB4RSTR_HSEMRST               = {.msk = 0x02000000U, .pos = 25};
const field32_t RCC_APB3RSTR_LTDCRST               = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_APB1LRSTR_LPTIM1RST            = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_APB1LRSTR_SPDIFRXRST           = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_APB1LRSTR_HDMICECRST           = {.msk = 0x08000000U, .pos = 27};
const field32_t RCC_APB1LRSTR_DAC12RST             = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_APB1HRSTR_CRSRST               = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_APB1HRSTR_SWPRST               = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_APB1HRSTR_OPAMPRST             = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_APB1HRSTR_MDIOSRST             = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_APB1HRSTR_FDCANRST             = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_APB2RSTR_DFSDM1RST             = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_APB2RSTR_HRTIMRST              = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_APB4RSTR_SYSCFGRST             = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_APB4RSTR_LPUART1RST            = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_APB4RSTR_SPI6RST               = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_APB4RSTR_I2C4RST               = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_APB4RSTR_COMP12RST             = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_APB4RSTR_VREFRST               = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_APB4RSTR_SAI4RST               = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_GCR_WW1RSC                     = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_D3AMR_BDMAAMEN                 = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_D3AMR_LPUART1AMEN              = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_D3AMR_SPI6AMEN                 = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_D3AMR_I2C4AMEN                 = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_D3AMR_COMP12AMEN               = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_D3AMR_VREFAMEN                 = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_D3AMR_RTCAMEN                  = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_D3AMR_CRCAMEN                  = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_D3AMR_SAI4AMEN                 = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_D3AMR_ADC3AMEN                 = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_D3AMR_BKPSRAMAMEN              = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_D3AMR_SRAM4AMEN                = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_RSR_RMVF                       = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_RSR_CPURSTF                    = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_RSR_BORRSTF                    = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_RSR_PINRSTF                    = {.msk = 0x00400000U, .pos = 22};
const field32_t RCC_RSR_PORRSTF                    = {.msk = 0x00800000U, .pos = 23};
const field32_t RCC_RSR_SFTRSTF                    = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_RSR_IWDG1RSTF                  = {.msk = 0x04000000U, .pos = 26};
const field32_t RCC_RSR_WWDG1RSTF                  = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_RSR_LPWRRSTF                   = {.msk = 0x40000000U, .pos = 30};
const field32_t RCC_C1_RSR_RMVF                    = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_RSR_CPURSTF                 = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_C1_RSR_BORRSTF                 = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_C1_RSR_PINRSTF                 = {.msk = 0x00400000U, .pos = 22};
const field32_t RCC_C1_RSR_PORRSTF                 = {.msk = 0x00800000U, .pos = 23};
const field32_t RCC_C1_RSR_SFTRSTF                 = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_C1_RSR_IWDG1RSTF               = {.msk = 0x04000000U, .pos = 26};
const field32_t RCC_C1_RSR_WWDG1RSTF               = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_RSR_LPWRRSTF                = {.msk = 0x40000000U, .pos = 30};
const field32_t RCC_C1_AHB3ENR_MDMAEN              = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_C1_AHB3ENR_DMA2DEN             = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_AHB3ENR_JPGDECEN            = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB3ENR_FMCEN               = {.msk = 0x00001000U, .pos = 12};
const field32_t RCC_C1_AHB3ENR_QSPIEN              = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_C1_AHB3ENR_SDMMC1EN            = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_AHB3ENR_MDMAEN                 = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB3ENR_DMA2DEN                = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB3ENR_JPGDECEN               = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB3ENR_FMCEN                  = {.msk = 0x00001000U, .pos = 12};
const field32_t RCC_AHB3ENR_QSPIEN                 = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_AHB3ENR_SDMMC1EN               = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_AHB1ENR_ADC12EN                = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB1ENR_ETH1MACEN              = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_AHB1ENR_ETH1TXEN               = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_AHB1ENR_ETH1RXEN               = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_C1_AHB1ENR_ADC12EN             = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB1ENR_ETH1MACEN           = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_C1_AHB1ENR_ETH1TXEN            = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_AHB1ENR_ETH1RXEN            = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_C1_AHB2ENR_CAMITFEN            = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_C1_AHB2ENR_CRYPTEN             = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_AHB2ENR_HASHEN              = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB2ENR_RNGEN               = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_C1_AHB2ENR_SDMMC2EN            = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB2ENR_CAMITFEN               = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB2ENR_CRYPTEN                = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB2ENR_HASHEN                 = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB2ENR_RNGEN                  = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB2ENR_SDMMC2EN               = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB4ENR_GPIOAEN                = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB4ENR_GPIOBEN                = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_AHB4ENR_GPIOCEN                = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_AHB4ENR_GPIODEN                = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_AHB4ENR_GPIOEEN                = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB4ENR_GPIOFEN                = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB4ENR_GPIOGEN                = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB4ENR_GPIOHEN                = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_AHB4ENR_GPIOIEN                = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_AHB4ENR_GPIOJEN                = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB4ENR_GPIOKEN                = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_AHB4ENR_CRCEN                  = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_AHB4ENR_BDMAEN                 = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_AHB4ENR_ADC3EN                 = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_AHB4ENR_HSEMEN                 = {.msk = 0x02000000U, .pos = 25};
const field32_t RCC_AHB4ENR_BKPRAMEN               = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_AHB4ENR_GPIOAEN             = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_C1_AHB4ENR_GPIOBEN             = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_C1_AHB4ENR_GPIOCEN             = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_C1_AHB4ENR_GPIODEN             = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_C1_AHB4ENR_GPIOEEN             = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_AHB4ENR_GPIOFEN             = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB4ENR_GPIOGEN             = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_C1_AHB4ENR_GPIOHEN             = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_C1_AHB4ENR_GPIOIEN             = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_C1_AHB4ENR_GPIOJEN             = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_C1_AHB4ENR_GPIOKEN             = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_C1_AHB4ENR_CRCEN               = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_C1_AHB4ENR_BDMAEN              = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_C1_AHB4ENR_ADC3EN              = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_C1_AHB4ENR_HSEMEN              = {.msk = 0x02000000U, .pos = 25};
const field32_t RCC_C1_AHB4ENR_BKPRAMEN            = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_APB3ENR_LTDCEN              = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_C1_APB3ENR_WWDG1EN             = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_APB3ENR_LTDCEN                 = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_APB3ENR_WWDG1EN                = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_APB1LENR_LPTIM1EN              = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_APB1LENR_SPDIFRXEN             = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_APB1LENR_HDMICECEN             = {.msk = 0x08000000U, .pos = 27};
const field32_t RCC_APB1LENR_DAC12EN               = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_APB1LENR_LPTIM1EN           = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_C1_APB1LENR_SPDIFRXEN          = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_APB1LENR_HDMICECEN          = {.msk = 0x08000000U, .pos = 27};
const field32_t RCC_C1_APB1LENR_DAC12EN            = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_APB1HENR_CRSEN                 = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_APB1HENR_SWPEN                 = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_APB1HENR_OPAMPEN               = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_APB1HENR_MDIOSEN               = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_APB1HENR_FDCANEN               = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_C1_APB1HENR_CRSEN              = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_C1_APB1HENR_SWPEN              = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_C1_APB1HENR_OPAMPEN            = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_APB1HENR_MDIOSEN            = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_APB1HENR_FDCANEN            = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_C1_APB2ENR_DFSDM1EN            = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_APB2ENR_HRTIMEN             = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_APB2ENR_DFSDM1EN               = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_APB2ENR_HRTIMEN                = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_APB4ENR_SYSCFGEN               = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_APB4ENR_LPUART1EN              = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_APB4ENR_SPI6EN                 = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_APB4ENR_I2C4EN                 = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_APB4ENR_COMP12EN               = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_APB4ENR_VREFEN                 = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_APB4ENR_RTCAPBEN               = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_APB4ENR_SAI4EN                 = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_C1_APB4ENR_SYSCFGEN            = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_C1_APB4ENR_LPUART1EN           = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_C1_APB4ENR_SPI6EN              = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_APB4ENR_I2C4EN              = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_C1_APB4ENR_COMP12EN            = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_C1_APB4ENR_VREFEN              = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_C1_APB4ENR_RTCAPBEN            = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_APB4ENR_SAI4EN              = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_C1_AHB3LPENR_MDMALPEN          = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_C1_AHB3LPENR_DMA2DLPEN         = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_AHB3LPENR_JPGDECLPEN        = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB3LPENR_FLITFLPEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_C1_AHB3LPENR_FMCLPEN           = {.msk = 0x00001000U, .pos = 12};
const field32_t RCC_C1_AHB3LPENR_QSPILPEN          = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_C1_AHB3LPENR_SDMMC1LPEN        = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_AHB3LPENR_D1DTCM1LPEN       = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_AHB3LPENR_DTCM2LPEN         = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_AHB3LPENR_ITCMLPEN          = {.msk = 0x40000000U, .pos = 30};
const field32_t RCC_C1_AHB3LPENR_AXISRAMLPEN       = {.msk = 0x80000000U, .pos = 31};
const field32_t RCC_AHB3LPENR_MDMALPEN             = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB3LPENR_DMA2DLPEN            = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB3LPENR_JPGDECLPEN           = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB3LPENR_FLITFLPEN            = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_AHB3LPENR_FMCLPEN              = {.msk = 0x00001000U, .pos = 12};
const field32_t RCC_AHB3LPENR_QSPILPEN             = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_AHB3LPENR_SDMMC1LPEN           = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_AHB3LPENR_D1DTCM1LPEN          = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_AHB3LPENR_DTCM2LPEN            = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_AHB3LPENR_ITCMLPEN             = {.msk = 0x40000000U, .pos = 30};
const field32_t RCC_AHB3LPENR_AXISRAMLPEN          = {.msk = 0x80000000U, .pos = 31};
const field32_t RCC_AHB1LPENR_ADC12LPEN            = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB1LPENR_ETH1MACLPEN          = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_AHB1LPENR_ETH1TXLPEN           = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_AHB1LPENR_ETH1RXLPEN           = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_C1_AHB1LPENR_ADC12LPEN         = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB1LPENR_ETH1MACLPEN       = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_C1_AHB1LPENR_ETH1TXLPEN        = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_AHB1LPENR_ETH1RXLPEN        = {.msk = 0x00020000U, .pos = 17};
const field32_t RCC_C1_AHB2LPENR_CAMITFLPEN        = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_C1_AHB2LPENR_CRYPTLPEN         = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_AHB2LPENR_HASHLPEN          = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB2LPENR_SDMMC2LPEN        = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_C1_AHB2LPENR_RNGLPEN           = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB2LPENR_CAMITFLPEN           = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB2LPENR_CRYPTLPEN            = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB2LPENR_HASHLPEN             = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB2LPENR_SDMMC2LPEN           = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB2LPENR_RNGLPEN              = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB4LPENR_GPIOALPEN            = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_AHB4LPENR_GPIOBLPEN            = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_AHB4LPENR_GPIOCLPEN            = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_AHB4LPENR_GPIODLPEN            = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_AHB4LPENR_GPIOELPEN            = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_AHB4LPENR_GPIOFLPEN            = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_AHB4LPENR_GPIOGLPEN            = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_AHB4LPENR_GPIOHLPEN            = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_AHB4LPENR_GPIOILPEN            = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_AHB4LPENR_GPIOJLPEN            = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_AHB4LPENR_GPIOKLPEN            = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_AHB4LPENR_CRCLPEN              = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_AHB4LPENR_BDMALPEN             = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_AHB4LPENR_ADC3LPEN             = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_AHB4LPENR_BKPRAMLPEN           = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_AHB4LPENR_SRAM4LPEN            = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_AHB4LPENR_GPIOALPEN         = {.msk = 0x00000001U, .pos = 0};
const field32_t RCC_C1_AHB4LPENR_GPIOBLPEN         = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_C1_AHB4LPENR_GPIOCLPEN         = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_C1_AHB4LPENR_GPIODLPEN         = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_C1_AHB4LPENR_GPIOELPEN         = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_AHB4LPENR_GPIOFLPEN         = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_AHB4LPENR_GPIOGLPEN         = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_C1_AHB4LPENR_GPIOHLPEN         = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_C1_AHB4LPENR_GPIOILPEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_C1_AHB4LPENR_GPIOJLPEN         = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_C1_AHB4LPENR_GPIOKLPEN         = {.msk = 0x00000400U, .pos = 10};
const field32_t RCC_C1_AHB4LPENR_CRCLPEN           = {.msk = 0x00080000U, .pos = 19};
const field32_t RCC_C1_AHB4LPENR_BDMALPEN          = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_C1_AHB4LPENR_ADC3LPEN          = {.msk = 0x01000000U, .pos = 24};
const field32_t RCC_C1_AHB4LPENR_BKPRAMLPEN        = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_AHB4LPENR_SRAM4LPEN         = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_APB3LPENR_LTDCLPEN          = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_C1_APB3LPENR_WWDG1LPEN         = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_APB3LPENR_LTDCLPEN             = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_APB3LPENR_WWDG1LPEN            = {.msk = 0x00000040U, .pos = 6};
const field32_t RCC_APB1LLPENR_LPTIM1LPEN          = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_APB1LLPENR_SPDIFRXLPEN         = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_APB1LLPENR_HDMICECLPEN         = {.msk = 0x08000000U, .pos = 27};
const field32_t RCC_APB1LLPENR_DAC12LPEN           = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_APB1LLPENR_LPTIM1LPEN       = {.msk = 0x00000200U, .pos = 9};
const field32_t RCC_C1_APB1LLPENR_SPDIFRXLPEN      = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_APB1LLPENR_HDMICECLPEN      = {.msk = 0x08000000U, .pos = 27};
const field32_t RCC_C1_APB1LLPENR_DAC12LPEN        = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_APB1HLPENR_CRSLPEN          = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_C1_APB1HLPENR_SWPLPEN          = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_C1_APB1HLPENR_OPAMPLPEN        = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_C1_APB1HLPENR_MDIOSLPEN        = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_APB1HLPENR_FDCANLPEN        = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_APB1HLPENR_CRSLPEN             = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_APB1HLPENR_SWPLPEN             = {.msk = 0x00000004U, .pos = 2};
const field32_t RCC_APB1HLPENR_OPAMPLPEN           = {.msk = 0x00000010U, .pos = 4};
const field32_t RCC_APB1HLPENR_MDIOSLPEN           = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_APB1HLPENR_FDCANLPEN           = {.msk = 0x00000100U, .pos = 8};
const field32_t RCC_APB2LPENR_DFSDM1LPEN           = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_APB2LPENR_HRTIMLPEN            = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_APB2LPENR_DFSDM1LPEN        = {.msk = 0x10000000U, .pos = 28};
const field32_t RCC_C1_APB2LPENR_HRTIMLPEN         = {.msk = 0x20000000U, .pos = 29};
const field32_t RCC_C1_APB4LPENR_SYSCFGLPEN        = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_C1_APB4LPENR_LPUART1LPEN       = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_C1_APB4LPENR_SPI6LPEN          = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_C1_APB4LPENR_I2C4LPEN          = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_C1_APB4LPENR_COMP12LPEN        = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_C1_APB4LPENR_VREFLPEN          = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_C1_APB4LPENR_RTCAPBLPEN        = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_C1_APB4LPENR_SAI4LPEN          = {.msk = 0x00200000U, .pos = 21};
const field32_t RCC_APB4LPENR_SYSCFGLPEN           = {.msk = 0x00000002U, .pos = 1};
const field32_t RCC_APB4LPENR_LPUART1LPEN          = {.msk = 0x00000008U, .pos = 3};
const field32_t RCC_APB4LPENR_SPI6LPEN             = {.msk = 0x00000020U, .pos = 5};
const field32_t RCC_APB4LPENR_I2C4LPEN             = {.msk = 0x00000080U, .pos = 7};
const field32_t RCC_APB4LPENR_COMP12LPEN           = {.msk = 0x00004000U, .pos = 14};
const field32_t RCC_APB4LPENR_VREFLPEN             = {.msk = 0x00008000U, .pos = 15};
const field32_t RCC_APB4LPENR_RTCAPBLPEN           = {.msk = 0x00010000U, .pos = 16};
const field32_t RCC_APB4LPENR_SAI4LPEN             = {.msk = 0x00200000U, .pos = 21};

/** @subsection Enumerated RCC Register Field Definitions */

field32_t const RCC_CR_DxCKRDY[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const RCC_CR_PLLxON[4] = {
  [1] = {.msk = 0x01000000U, .pos = 24},
  [2] = {.msk = 0x04000000U, .pos = 26},
  [3] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const RCC_CR_PLLxRDY[4] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x08000000U, .pos = 27},
  [3] = {.msk = 0x20000000U, .pos = 29},
};

field32_t const RCC_CFGR_MCOxPRE[3] = {
  [1] = {.msk = 0x003C0000U, .pos = 18},
  [2] = {.msk = 0x1E000000U, .pos = 25},
};

field32_t const RCC_CFGR_MCOxSEL[3] = {
  [1] = {.msk = 0x01C00000U, .pos = 22},
  [2] = {.msk = 0xE0000000U, .pos = 29},
};

field32_t const RCC_D2CFGR_D2PPREx[3] = {
  [1] = {.msk = 0x00000070U, .pos = 4},
  [2] = {.msk = 0x00000700U, .pos = 8},
};

field32_t const RCC_PLLCKSELR_DIVMx[4] = {
  [1] = {.msk = 0x000003F0U, .pos = 4},
  [2] = {.msk = 0x0003F000U, .pos = 12},
  [3] = {.msk = 0x03F00000U, .pos = 20},
};

field32_t const RCC_PLLCFGR_PLLxFRACEN[4] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_PLLCFGR_PLLxVCOSEL[4] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000020U, .pos = 5},
  [3] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const RCC_PLLCFGR_PLLxRGE[4] = {
  [1] = {.msk = 0x0000000CU, .pos = 2},
  [2] = {.msk = 0x000000C0U, .pos = 6},
  [3] = {.msk = 0x00000C00U, .pos = 10},
};

field32_t const RCC_PLLCFGR_DIVPxEN[4] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00080000U, .pos = 19},
  [3] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const RCC_PLLCFGR_DIVQxEN[4] = {
  [1] = {.msk = 0x00020000U, .pos = 17},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [3] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const RCC_PLLCFGR_DIVRxEN[4] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00200000U, .pos = 21},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const RCC_CIER_PLLxRDYIE[4] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_CIFR_PLLxRDYF[4] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_CICR_PLLxRDYC[4] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_AHB1RSTR_DMAxRST[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const RCC_AHB1RSTR_USBxOTGRST[3] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const RCC_APB1LRSTR_TIMxRST[15] = {
  [2]  = {.msk = 0x00000001U, .pos = 0},
  [3]  = {.msk = 0x00000002U, .pos = 1},
  [4]  = {.msk = 0x00000004U, .pos = 2},
  [5]  = {.msk = 0x00000008U, .pos = 3},
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [12] = {.msk = 0x00000040U, .pos = 6},
  [13] = {.msk = 0x00000080U, .pos = 7},
  [14] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_APB1LRSTR_SPIxRST[4] = {
  [2] = {.msk = 0x00004000U, .pos = 14},
  [3] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const RCC_APB1LRSTR_USARTxRST[9] = {
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [7] = {.msk = 0x40000000U, .pos = 30},
  [8] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const RCC_APB1LRSTR_UARTxRST[6] = {
  [4] = {.msk = 0x00080000U, .pos = 19},
  [5] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const RCC_APB1LRSTR_I2CxRST[4] = {
  [1] = {.msk = 0x00200000U, .pos = 21},
  [2] = {.msk = 0x00400000U, .pos = 22},
  [3] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const RCC_APB2RSTR_SAIxRST[4] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const RCC_APB4RSTR_LPTIMxRST[6] = {
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
  [5] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const RCC_D3AMR_LPTIMxAMEN[6] = {
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
  [5] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const RCC_RSR_DxRSTF[3] = {
  [1] = {.msk = 0x00080000U, .pos = 19},
  [2] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const RCC_AHB1ENR_DMAxEN[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const RCC_AHB1ENR_USBxOTGEN[3] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const RCC_AHB1ENR_USBxULPIEN[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const RCC_C1_AHB2ENR_SRAMxEN[4] = {
  [1] = {.msk = 0x20000000U, .pos = 29},
  [2] = {.msk = 0x40000000U, .pos = 30},
  [3] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const RCC_APB1LENR_TIMxEN[15] = {
  [2]  = {.msk = 0x00000001U, .pos = 0},
  [3]  = {.msk = 0x00000002U, .pos = 1},
  [4]  = {.msk = 0x00000004U, .pos = 2},
  [5]  = {.msk = 0x00000008U, .pos = 3},
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [12] = {.msk = 0x00000040U, .pos = 6},
  [13] = {.msk = 0x00000080U, .pos = 7},
  [14] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_APB1LENR_SPIxEN[4] = {
  [2] = {.msk = 0x00004000U, .pos = 14},
  [3] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const RCC_APB1LENR_USARTxEN[9] = {
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [7] = {.msk = 0x40000000U, .pos = 30},
  [8] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const RCC_APB1LENR_UARTxEN[6] = {
  [4] = {.msk = 0x00080000U, .pos = 19},
  [5] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const RCC_APB1LENR_I2CxEN[4] = {
  [1] = {.msk = 0x00200000U, .pos = 21},
  [2] = {.msk = 0x00400000U, .pos = 22},
  [3] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const RCC_C1_APB2ENR_SAIxEN[4] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const RCC_APB4ENR_LPTIMxEN[6] = {
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
  [5] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const RCC_AHB1LPENR_DMAxLPEN[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const RCC_AHB1LPENR_USBxOTGLPEN[3] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const RCC_AHB1LPENR_USBxULPILPEN[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const RCC_C1_AHB2LPENR_SRAMxLPEN[4] = {
  [1] = {.msk = 0x20000000U, .pos = 29},
  [2] = {.msk = 0x40000000U, .pos = 30},
  [3] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const RCC_APB1LLPENR_TIMxLPEN[15] = {
  [2]  = {.msk = 0x00000001U, .pos = 0},
  [3]  = {.msk = 0x00000002U, .pos = 1},
  [4]  = {.msk = 0x00000004U, .pos = 2},
  [5]  = {.msk = 0x00000008U, .pos = 3},
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [12] = {.msk = 0x00000040U, .pos = 6},
  [13] = {.msk = 0x00000080U, .pos = 7},
  [14] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const RCC_APB1LLPENR_SPIxLPEN[4] = {
  [2] = {.msk = 0x00004000U, .pos = 14},
  [3] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const RCC_APB1LLPENR_USARTxLPEN[9] = {
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [7] = {.msk = 0x40000000U, .pos = 30},
  [8] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const RCC_APB1LLPENR_UARTxLPEN[6] = {
  [4] = {.msk = 0x00080000U, .pos = 19},
  [5] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const RCC_APB1LLPENR_I2CxLPEN[4] = {
  [1] = {.msk = 0x00200000U, .pos = 21},
  [2] = {.msk = 0x00400000U, .pos = 22},
  [3] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const RCC_APB2LPENR_SAIxLPEN[4] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const RCC_C1_APB4LPENR_LPTIMxLPEN[6] = {
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
  [5] = {.msk = 0x00001000U, .pos = 12},
};

/**************************************************************************************************
 * @section LPTIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated LPTIMx Register Definitions */

ro_reg32_t const LPTIMx_ISR[6] = {
  [1] = (ro_reg32_t)0x40002400U,
  [2] = (ro_reg32_t)0x58002400U,
  [3] = (ro_reg32_t)0x58002800U,
  [4] = (ro_reg32_t)0x58002C00U,
  [5] = (ro_reg32_t)0x58003000U,
};

rw_reg32_t const LPTIMx_ICR[6] = {
  [1] = (rw_reg32_t)0x40002404U,
  [2] = (rw_reg32_t)0x58002404U,
  [3] = (rw_reg32_t)0x58002804U,
  [4] = (rw_reg32_t)0x58002C04U,
  [5] = (rw_reg32_t)0x58003004U,
};

rw_reg32_t const LPTIMx_IER[6] = {
  [1] = (rw_reg32_t)0x40002408U,
  [2] = (rw_reg32_t)0x58002408U,
  [3] = (rw_reg32_t)0x58002808U,
  [4] = (rw_reg32_t)0x58002C08U,
  [5] = (rw_reg32_t)0x58003008U,
};

rw_reg32_t const LPTIMx_CFGR[6] = {
  [1] = (rw_reg32_t)0x4000240CU,
  [2] = (rw_reg32_t)0x5800240CU,
  [3] = (rw_reg32_t)0x5800280CU,
  [4] = (rw_reg32_t)0x58002C0CU,
  [5] = (rw_reg32_t)0x5800300CU,
};

rw_reg32_t const LPTIMx_CR[6] = {
  [1] = (rw_reg32_t)0x40002410U,
  [2] = (rw_reg32_t)0x58002410U,
  [3] = (rw_reg32_t)0x58002810U,
  [4] = (rw_reg32_t)0x58002C10U,
  [5] = (rw_reg32_t)0x58003010U,
};

rw_reg32_t const LPTIMx_CMP[6] = {
  [1] = (rw_reg32_t)0x40002414U,
  [2] = (rw_reg32_t)0x58002414U,
  [3] = (rw_reg32_t)0x58002814U,
  [4] = (rw_reg32_t)0x58002C14U,
  [5] = (rw_reg32_t)0x58003014U,
};

rw_reg32_t const LPTIMx_ARR[6] = {
  [1] = (rw_reg32_t)0x40002418U,
  [2] = (rw_reg32_t)0x58002418U,
  [3] = (rw_reg32_t)0x58002818U,
  [4] = (rw_reg32_t)0x58002C18U,
  [5] = (rw_reg32_t)0x58003018U,
};

ro_reg32_t const LPTIMx_CNT[6] = {
  [1] = (ro_reg32_t)0x4000241CU,
  [2] = (ro_reg32_t)0x5800241CU,
  [3] = (ro_reg32_t)0x5800281CU,
  [4] = (ro_reg32_t)0x58002C1CU,
  [5] = (ro_reg32_t)0x5800301CU,
};

rw_reg32_t const LPTIMx_CFGR2[6] = {
  [1] = (rw_reg32_t)0x40002424U,
  [2] = (rw_reg32_t)0x58002424U,
  [3] = (rw_reg32_t)0x58002824U,
  [4] = (rw_reg32_t)0x58002C24U,
  [5] = (rw_reg32_t)0x58003024U,
};

/** @subsection LPTIMx Register Field Definitions */

const field32_t LPTIMx_ISR_DOWN       = {.msk = 0x00000040U, .pos = 6};
const field32_t LPTIMx_ISR_UP         = {.msk = 0x00000020U, .pos = 5};
const field32_t LPTIMx_ISR_ARROK      = {.msk = 0x00000010U, .pos = 4};
const field32_t LPTIMx_ISR_CMPOK      = {.msk = 0x00000008U, .pos = 3};
const field32_t LPTIMx_ISR_EXTTRIG    = {.msk = 0x00000004U, .pos = 2};
const field32_t LPTIMx_ISR_ARRM       = {.msk = 0x00000002U, .pos = 1};
const field32_t LPTIMx_ISR_CMPM       = {.msk = 0x00000001U, .pos = 0};
const field32_t LPTIMx_ICR_DOWNCF     = {.msk = 0x00000040U, .pos = 6};
const field32_t LPTIMx_ICR_UPCF       = {.msk = 0x00000020U, .pos = 5};
const field32_t LPTIMx_ICR_ARROKCF    = {.msk = 0x00000010U, .pos = 4};
const field32_t LPTIMx_ICR_CMPOKCF    = {.msk = 0x00000008U, .pos = 3};
const field32_t LPTIMx_ICR_EXTTRIGCF  = {.msk = 0x00000004U, .pos = 2};
const field32_t LPTIMx_ICR_ARRMCF     = {.msk = 0x00000002U, .pos = 1};
const field32_t LPTIMx_ICR_CMPMCF     = {.msk = 0x00000001U, .pos = 0};
const field32_t LPTIMx_IER_DOWNIE     = {.msk = 0x00000040U, .pos = 6};
const field32_t LPTIMx_IER_UPIE       = {.msk = 0x00000020U, .pos = 5};
const field32_t LPTIMx_IER_ARROKIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t LPTIMx_IER_CMPOKIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t LPTIMx_IER_EXTTRIGIE  = {.msk = 0x00000004U, .pos = 2};
const field32_t LPTIMx_IER_ARRMIE     = {.msk = 0x00000002U, .pos = 1};
const field32_t LPTIMx_IER_CMPMIE     = {.msk = 0x00000001U, .pos = 0};
const field32_t LPTIMx_CFGR_ENC       = {.msk = 0x01000000U, .pos = 24};
const field32_t LPTIMx_CFGR_COUNTMODE = {.msk = 0x00800000U, .pos = 23};
const field32_t LPTIMx_CFGR_PRELOAD   = {.msk = 0x00400000U, .pos = 22};
const field32_t LPTIMx_CFGR_WAVPOL    = {.msk = 0x00200000U, .pos = 21};
const field32_t LPTIMx_CFGR_WAVE      = {.msk = 0x00100000U, .pos = 20};
const field32_t LPTIMx_CFGR_TIMOUT    = {.msk = 0x00080000U, .pos = 19};
const field32_t LPTIMx_CFGR_TRIGEN    = {.msk = 0x00060000U, .pos = 17};
const field32_t LPTIMx_CFGR_TRIGSEL   = {.msk = 0x0000E000U, .pos = 13};
const field32_t LPTIMx_CFGR_PRESC     = {.msk = 0x00000E00U, .pos = 9};
const field32_t LPTIMx_CFGR_TRGFLT    = {.msk = 0x000000C0U, .pos = 6};
const field32_t LPTIMx_CFGR_CKFLT     = {.msk = 0x00000018U, .pos = 3};
const field32_t LPTIMx_CFGR_CKPOL     = {.msk = 0x00000006U, .pos = 1};
const field32_t LPTIMx_CFGR_CKSEL     = {.msk = 0x00000001U, .pos = 0};
const field32_t LPTIMx_CR_ENABLE      = {.msk = 0x00000001U, .pos = 0};
const field32_t LPTIMx_CR_SNGSTRT     = {.msk = 0x00000002U, .pos = 1};
const field32_t LPTIMx_CR_CNTSTRT     = {.msk = 0x00000004U, .pos = 2};
const field32_t LPTIMx_CR_COUNTRST    = {.msk = 0x00000008U, .pos = 3};
const field32_t LPTIMx_CR_RSTARE      = {.msk = 0x00000010U, .pos = 4};
const field32_t LPTIMx_CMP_CMP        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t LPTIMx_ARR_ARR        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t LPTIMx_CNT_CNT        = {.msk = 0x0000FFFFU, .pos = 0};

/** @subsection Enumerated LPTIMx Register Field Definitions */

field32_t const LPTIMx_CFGR2_INxSEL[3] = {
  [1] = {.msk = 0x00000003U, .pos = 0},
  [2] = {.msk = 0x00000030U, .pos = 4},
};

/**************************************************************************************************
 * @section LPUART1 Definitions
 **************************************************************************************************/

/** @subsection LPUART1 Register Definitions */

rw_reg32_t const LPUART1_CR1   = (rw_reg32_t)0x58000C00U;
rw_reg32_t const LPUART1_CR2   = (rw_reg32_t)0x58000C04U;
rw_reg32_t const LPUART1_CR3   = (rw_reg32_t)0x58000C08U;
rw_reg32_t const LPUART1_BRR   = (rw_reg32_t)0x58000C0CU;
rw_reg32_t const LPUART1_GTPR  = (rw_reg32_t)0x58000C10U;
rw_reg32_t const LPUART1_RTOR  = (rw_reg32_t)0x58000C14U;
rw_reg32_t const LPUART1_RQR   = (rw_reg32_t)0x58000C18U;
ro_reg32_t const LPUART1_ISR   = (ro_reg32_t)0x58000C1CU;
rw_reg32_t const LPUART1_ICR   = (rw_reg32_t)0x58000C20U;
ro_reg32_t const LPUART1_RDR   = (ro_reg32_t)0x58000C24U;
rw_reg32_t const LPUART1_TDR   = (rw_reg32_t)0x58000C28U;
rw_reg32_t const LPUART1_PRESC = (rw_reg32_t)0x58000C2CU;

/** @subsection LPUART1 Register Field Definitions */

const field32_t LPUART1_CR1_RXFFIE      = {.msk = 0x80000000U, .pos = 31};
const field32_t LPUART1_CR1_TXFEIE      = {.msk = 0x40000000U, .pos = 30};
const field32_t LPUART1_CR1_FIFOEN      = {.msk = 0x20000000U, .pos = 29};
const field32_t LPUART1_CR1_DEAT        = {.msk = 0x03E00000U, .pos = 21};
const field32_t LPUART1_CR1_DEDT        = {.msk = 0x001F0000U, .pos = 16};
const field32_t LPUART1_CR1_CMIE        = {.msk = 0x00004000U, .pos = 14};
const field32_t LPUART1_CR1_MME         = {.msk = 0x00002000U, .pos = 13};
const field32_t LPUART1_CR1_WAKE        = {.msk = 0x00000800U, .pos = 11};
const field32_t LPUART1_CR1_PCE         = {.msk = 0x00000400U, .pos = 10};
const field32_t LPUART1_CR1_PS          = {.msk = 0x00000200U, .pos = 9};
const field32_t LPUART1_CR1_PEIE        = {.msk = 0x00000100U, .pos = 8};
const field32_t LPUART1_CR1_TXEIE       = {.msk = 0x00000080U, .pos = 7};
const field32_t LPUART1_CR1_TCIE        = {.msk = 0x00000040U, .pos = 6};
const field32_t LPUART1_CR1_RXNEIE      = {.msk = 0x00000020U, .pos = 5};
const field32_t LPUART1_CR1_IDLEIE      = {.msk = 0x00000010U, .pos = 4};
const field32_t LPUART1_CR1_TE          = {.msk = 0x00000008U, .pos = 3};
const field32_t LPUART1_CR1_RE          = {.msk = 0x00000004U, .pos = 2};
const field32_t LPUART1_CR1_UESM        = {.msk = 0x00000002U, .pos = 1};
const field32_t LPUART1_CR1_UE          = {.msk = 0x00000001U, .pos = 0};
const field32_t LPUART1_CR2_ADD         = {.msk = 0xFF000000U, .pos = 24};
const field32_t LPUART1_CR2_MSBFIRST    = {.msk = 0x00080000U, .pos = 19};
const field32_t LPUART1_CR2_DATAINV     = {.msk = 0x00040000U, .pos = 18};
const field32_t LPUART1_CR2_TXINV       = {.msk = 0x00020000U, .pos = 17};
const field32_t LPUART1_CR2_RXINV       = {.msk = 0x00010000U, .pos = 16};
const field32_t LPUART1_CR2_SWAP        = {.msk = 0x00008000U, .pos = 15};
const field32_t LPUART1_CR2_STOP        = {.msk = 0x00003000U, .pos = 12};
const field32_t LPUART1_CR2_ADDM7       = {.msk = 0x00000010U, .pos = 4};
const field32_t LPUART1_CR3_TXFTCFG     = {.msk = 0xE0000000U, .pos = 29};
const field32_t LPUART1_CR3_RXFTIE      = {.msk = 0x10000000U, .pos = 28};
const field32_t LPUART1_CR3_RXFTCFG     = {.msk = 0x0E000000U, .pos = 25};
const field32_t LPUART1_CR3_TXFTIE      = {.msk = 0x00800000U, .pos = 23};
const field32_t LPUART1_CR3_WUFIE       = {.msk = 0x00400000U, .pos = 22};
const field32_t LPUART1_CR3_WUS         = {.msk = 0x00300000U, .pos = 20};
const field32_t LPUART1_CR3_DEP         = {.msk = 0x00008000U, .pos = 15};
const field32_t LPUART1_CR3_DEM         = {.msk = 0x00004000U, .pos = 14};
const field32_t LPUART1_CR3_DDRE        = {.msk = 0x00002000U, .pos = 13};
const field32_t LPUART1_CR3_OVRDIS      = {.msk = 0x00001000U, .pos = 12};
const field32_t LPUART1_CR3_CTSIE       = {.msk = 0x00000400U, .pos = 10};
const field32_t LPUART1_CR3_CTSE        = {.msk = 0x00000200U, .pos = 9};
const field32_t LPUART1_CR3_RTSE        = {.msk = 0x00000100U, .pos = 8};
const field32_t LPUART1_CR3_DMAT        = {.msk = 0x00000080U, .pos = 7};
const field32_t LPUART1_CR3_DMAR        = {.msk = 0x00000040U, .pos = 6};
const field32_t LPUART1_CR3_HDSEL       = {.msk = 0x00000008U, .pos = 3};
const field32_t LPUART1_CR3_EIE         = {.msk = 0x00000001U, .pos = 0};
const field32_t LPUART1_BRR_BRR         = {.msk = 0x000FFFFFU, .pos = 0};
const field32_t LPUART1_GTPR_GT         = {.msk = 0x0000FF00U, .pos = 8};
const field32_t LPUART1_GTPR_PSC        = {.msk = 0x000000FFU, .pos = 0};
const field32_t LPUART1_RTOR_BLEN       = {.msk = 0xFF000000U, .pos = 24};
const field32_t LPUART1_RTOR_RTO        = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t LPUART1_RQR_TXFRQ       = {.msk = 0x00000010U, .pos = 4};
const field32_t LPUART1_RQR_RXFRQ       = {.msk = 0x00000008U, .pos = 3};
const field32_t LPUART1_RQR_MMRQ        = {.msk = 0x00000004U, .pos = 2};
const field32_t LPUART1_RQR_SBKRQ       = {.msk = 0x00000002U, .pos = 1};
const field32_t LPUART1_RQR_ABRRQ       = {.msk = 0x00000001U, .pos = 0};
const field32_t LPUART1_ISR_TXFT        = {.msk = 0x08000000U, .pos = 27};
const field32_t LPUART1_ISR_RXFT        = {.msk = 0x04000000U, .pos = 26};
const field32_t LPUART1_ISR_RXFF        = {.msk = 0x01000000U, .pos = 24};
const field32_t LPUART1_ISR_TXFE        = {.msk = 0x00800000U, .pos = 23};
const field32_t LPUART1_ISR_REACK       = {.msk = 0x00400000U, .pos = 22};
const field32_t LPUART1_ISR_TEACK       = {.msk = 0x00200000U, .pos = 21};
const field32_t LPUART1_ISR_WUF         = {.msk = 0x00100000U, .pos = 20};
const field32_t LPUART1_ISR_RWU         = {.msk = 0x00080000U, .pos = 19};
const field32_t LPUART1_ISR_SBKF        = {.msk = 0x00040000U, .pos = 18};
const field32_t LPUART1_ISR_CMF         = {.msk = 0x00020000U, .pos = 17};
const field32_t LPUART1_ISR_BUSY        = {.msk = 0x00010000U, .pos = 16};
const field32_t LPUART1_ISR_CTS         = {.msk = 0x00000400U, .pos = 10};
const field32_t LPUART1_ISR_CTSIF       = {.msk = 0x00000200U, .pos = 9};
const field32_t LPUART1_ISR_TXE         = {.msk = 0x00000080U, .pos = 7};
const field32_t LPUART1_ISR_TC          = {.msk = 0x00000040U, .pos = 6};
const field32_t LPUART1_ISR_RXNE        = {.msk = 0x00000020U, .pos = 5};
const field32_t LPUART1_ISR_IDLE        = {.msk = 0x00000010U, .pos = 4};
const field32_t LPUART1_ISR_ORE         = {.msk = 0x00000008U, .pos = 3};
const field32_t LPUART1_ISR_NE          = {.msk = 0x00000004U, .pos = 2};
const field32_t LPUART1_ISR_FE          = {.msk = 0x00000002U, .pos = 1};
const field32_t LPUART1_ISR_PE          = {.msk = 0x00000001U, .pos = 0};
const field32_t LPUART1_ICR_WUCF        = {.msk = 0x00100000U, .pos = 20};
const field32_t LPUART1_ICR_CMCF        = {.msk = 0x00020000U, .pos = 17};
const field32_t LPUART1_ICR_CTSCF       = {.msk = 0x00000200U, .pos = 9};
const field32_t LPUART1_ICR_TCCF        = {.msk = 0x00000040U, .pos = 6};
const field32_t LPUART1_ICR_IDLECF      = {.msk = 0x00000010U, .pos = 4};
const field32_t LPUART1_ICR_ORECF       = {.msk = 0x00000008U, .pos = 3};
const field32_t LPUART1_ICR_NCF         = {.msk = 0x00000004U, .pos = 2};
const field32_t LPUART1_ICR_FECF        = {.msk = 0x00000002U, .pos = 1};
const field32_t LPUART1_ICR_PECF        = {.msk = 0x00000001U, .pos = 0};
const field32_t LPUART1_RDR_RDR         = {.msk = 0x000001FFU, .pos = 0};
const field32_t LPUART1_TDR_TDR         = {.msk = 0x000001FFU, .pos = 0};
const field32_t LPUART1_PRESC_PRESCALER = {.msk = 0x0000000FU, .pos = 0};

/** @subsection Enumerated LPUART1 Register Field Definitions */

field32_t const LPUART1_CR1_Mx[2] = {
  [0] = {.msk = 0x00001000U, .pos = 12},
  [1] = {.msk = 0x10000000U, .pos = 28},
};

/**************************************************************************************************
 * @section SYSCFG Definitions
 **************************************************************************************************/

/** @subsection SYSCFG Register Definitions */

rw_reg32_t const SYSCFG_PMCR    = (rw_reg32_t)0x58000404U;
rw_reg32_t const SYSCFG_EXTICR1 = (rw_reg32_t)0x58000408U;
rw_reg32_t const SYSCFG_EXTICR2 = (rw_reg32_t)0x5800040CU;
rw_reg32_t const SYSCFG_EXTICR3 = (rw_reg32_t)0x58000410U;
rw_reg32_t const SYSCFG_EXTICR4 = (rw_reg32_t)0x58000414U;
rw_reg32_t const SYSCFG_CCCSR   = (rw_reg32_t)0x58000420U;
ro_reg32_t const SYSCFG_CCVR    = (ro_reg32_t)0x58000424U;
rw_reg32_t const SYSCFG_CCCR    = (rw_reg32_t)0x58000428U;
ro_reg32_t const SYSCFG_PKGR    = (ro_reg32_t)0x58000524U;
ro_reg32_t const SYSCFG_UR0     = (ro_reg32_t)0x58000700U;
rw_reg32_t const SYSCFG_UR2     = (rw_reg32_t)0x58000708U;
rw_reg32_t const SYSCFG_UR3     = (rw_reg32_t)0x5800070CU;
ro_reg32_t const SYSCFG_UR4     = (ro_reg32_t)0x58000710U;
ro_reg32_t const SYSCFG_UR5     = (ro_reg32_t)0x58000714U;
ro_reg32_t const SYSCFG_UR6     = (ro_reg32_t)0x58000718U;
ro_reg32_t const SYSCFG_UR7     = (ro_reg32_t)0x5800071CU;
ro_reg32_t const SYSCFG_UR8     = (ro_reg32_t)0x58000720U;
ro_reg32_t const SYSCFG_UR9     = (ro_reg32_t)0x58000724U;
ro_reg32_t const SYSCFG_UR10    = (ro_reg32_t)0x58000728U;
ro_reg32_t const SYSCFG_UR11    = (ro_reg32_t)0x5800072CU;
ro_reg32_t const SYSCFG_UR12    = (ro_reg32_t)0x58000730U;
ro_reg32_t const SYSCFG_UR13    = (ro_reg32_t)0x58000734U;
rw_reg32_t const SYSCFG_UR14    = (rw_reg32_t)0x58000738U;
ro_reg32_t const SYSCFG_UR15    = (ro_reg32_t)0x5800073CU;
ro_reg32_t const SYSCFG_UR16    = (ro_reg32_t)0x58000740U;
ro_reg32_t const SYSCFG_UR17    = (ro_reg32_t)0x58000744U;

/** @subsection SYSCFG Register Field Definitions */

const field32_t SYSCFG_PMCR_BOOSTE    = {.msk = 0x00000100U, .pos = 8};
const field32_t SYSCFG_PMCR_EPIS      = {.msk = 0x00E00000U, .pos = 21};
const field32_t SYSCFG_CCCSR_EN       = {.msk = 0x00000001U, .pos = 0};
const field32_t SYSCFG_CCCSR_CS       = {.msk = 0x00000002U, .pos = 1};
const field32_t SYSCFG_CCCSR_READY    = {.msk = 0x00000100U, .pos = 8};
const field32_t SYSCFG_CCCSR_HSLV     = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_CCVR_NCV       = {.msk = 0x0000000FU, .pos = 0};
const field32_t SYSCFG_CCVR_PCV       = {.msk = 0x000000F0U, .pos = 4};
const field32_t SYSCFG_CCCR_NCC       = {.msk = 0x0000000FU, .pos = 0};
const field32_t SYSCFG_CCCR_PCC       = {.msk = 0x000000F0U, .pos = 4};
const field32_t SYSCFG_PKGR_PKG       = {.msk = 0x0000000FU, .pos = 0};
const field32_t SYSCFG_UR0_BKS        = {.msk = 0x00000001U, .pos = 0};
const field32_t SYSCFG_UR0_RDP        = {.msk = 0x00FF0000U, .pos = 16};
const field32_t SYSCFG_UR2_BORH       = {.msk = 0x00000003U, .pos = 0};
const field32_t SYSCFG_UR2_BOOT_ADD0  = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SYSCFG_UR3_BOOT_ADD1  = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SYSCFG_UR4_MEPAD_1    = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR5_MESAD_1    = {.msk = 0x00000001U, .pos = 0};
const field32_t SYSCFG_UR5_WRPN_1     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t SYSCFG_UR6_PA_BEG_1   = {.msk = 0x00000FFFU, .pos = 0};
const field32_t SYSCFG_UR6_PA_END_1   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t SYSCFG_UR7_SA_BEG_1   = {.msk = 0x00000FFFU, .pos = 0};
const field32_t SYSCFG_UR7_SA_END_1   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t SYSCFG_UR8_MEPAD_2    = {.msk = 0x00000001U, .pos = 0};
const field32_t SYSCFG_UR8_MESAD_2    = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR9_WRPN_2     = {.msk = 0x000000FFU, .pos = 0};
const field32_t SYSCFG_UR9_PA_BEG_2   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t SYSCFG_UR10_PA_END_2  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t SYSCFG_UR10_SA_BEG_2  = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t SYSCFG_UR11_SA_END_2  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t SYSCFG_UR11_IWDG1M    = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR12_SECURE    = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR13_SDRS      = {.msk = 0x00000003U, .pos = 0};
const field32_t SYSCFG_UR13_D1SBRST   = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR14_D1STPRST  = {.msk = 0x00000001U, .pos = 0};
const field32_t SYSCFG_UR15_FZIWDGSTB = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR16_FZIWDGSTP = {.msk = 0x00000001U, .pos = 0};
const field32_t SYSCFG_UR16_PKP       = {.msk = 0x00010000U, .pos = 16};
const field32_t SYSCFG_UR17_IO_HSLV   = {.msk = 0x00000001U, .pos = 0};

/** @subsection Enumerated SYSCFG Register Field Definitions */

field32_t const SYSCFG_PMCR_I2CxFMP[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const SYSCFG_PMCR_PBxFMP[10] = {
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [8]  = {.msk = 0x00000040U, .pos = 6},
  [9]  = {.msk = 0x00000080U, .pos = 7},
};

field32_t const SYSCFG_PMCR_PAxSO[2] = {
  [0] = {.msk = 0x01000000U, .pos = 24},
  [1] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const SYSCFG_PMCR_PCxSO[4] = {
  [2] = {.msk = 0x04000000U, .pos = 26},
  [3] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const SYSCFG_EXTICR1_EXTIx[4] = {
  [0] = {.msk = 0x0000000FU, .pos = 0},
  [1] = {.msk = 0x000000F0U, .pos = 4},
  [2] = {.msk = 0x00000F00U, .pos = 8},
  [3] = {.msk = 0x0000F000U, .pos = 12},
};

field32_t const SYSCFG_EXTICR3_EXTI1x[2] = {
  [0] = {.msk = 0x00000F00U, .pos = 8},
};

/**************************************************************************************************
 * @section EXTI Definitions
 **************************************************************************************************/

/** @subsection EXTI Register Definitions */

rw_reg32_t const EXTI_RTSR1   = (rw_reg32_t)0x58000000U;
rw_reg32_t const EXTI_FTSR1   = (rw_reg32_t)0x58000004U;
rw_reg32_t const EXTI_SWIER1  = (rw_reg32_t)0x58000008U;
rw_reg32_t const EXTI_D3PMR1  = (rw_reg32_t)0x5800000CU;
rw_reg32_t const EXTI_D3PCR1L = (rw_reg32_t)0x58000010U;
rw_reg32_t const EXTI_D3PCR1H = (rw_reg32_t)0x58000014U;
rw_reg32_t const EXTI_RTSR2   = (rw_reg32_t)0x58000020U;
rw_reg32_t const EXTI_FTSR2   = (rw_reg32_t)0x58000024U;
rw_reg32_t const EXTI_SWIER2  = (rw_reg32_t)0x58000028U;
rw_reg32_t const EXTI_D3PMR2  = (rw_reg32_t)0x5800002CU;
rw_reg32_t const EXTI_D3PCR2L = (rw_reg32_t)0x58000030U;
rw_reg32_t const EXTI_D3PCR2H = (rw_reg32_t)0x58000034U;
rw_reg32_t const EXTI_RTSR3   = (rw_reg32_t)0x58000040U;
rw_reg32_t const EXTI_FTSR3   = (rw_reg32_t)0x58000044U;
rw_reg32_t const EXTI_SWIER3  = (rw_reg32_t)0x58000048U;
rw_reg32_t const EXTI_D3PMR3  = (rw_reg32_t)0x5800004CU;
rw_reg32_t const EXTI_D3PCR3H = (rw_reg32_t)0x58000054U;
rw_reg32_t const EXTI_CPUIMR1 = (rw_reg32_t)0x58000080U;
rw_reg32_t const EXTI_CPUEMR1 = (rw_reg32_t)0x58000084U;
rw_reg32_t const EXTI_CPUPR1  = (rw_reg32_t)0x58000088U;
rw_reg32_t const EXTI_CPUIMR2 = (rw_reg32_t)0x58000090U;
rw_reg32_t const EXTI_CPUEMR2 = (rw_reg32_t)0x58000094U;
ro_reg32_t const EXTI_CPUPR2  = (ro_reg32_t)0x58000098U;
ro_reg32_t const EXTI_CPUIMR3 = (ro_reg32_t)0x580000A0U;
ro_reg32_t const EXTI_CPUEMR3 = (ro_reg32_t)0x580000A4U;
ro_reg32_t const EXTI_CPUPR3  = (ro_reg32_t)0x580000A8U;

/** @subsection EXTI Register Field Definitions */

const field32_t EXTI_D3PMR3_MR88   = {.msk = 0x01000000U, .pos = 24};
const field32_t EXTI_D3PCR3H_PCS88 = {.msk = 0x000C0000U, .pos = 18};

/** @subsection Enumerated EXTI Register Field Definitions */

field32_t const EXTI_RTSR1_TRx[22] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
  [16] = {.msk = 0x00010000U, .pos = 16},
  [17] = {.msk = 0x00020000U, .pos = 17},
  [18] = {.msk = 0x00040000U, .pos = 18},
  [19] = {.msk = 0x00080000U, .pos = 19},
  [20] = {.msk = 0x00100000U, .pos = 20},
  [21] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const EXTI_SWIER1_SWIERx[22] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
  [16] = {.msk = 0x00010000U, .pos = 16},
  [17] = {.msk = 0x00020000U, .pos = 17},
  [18] = {.msk = 0x00040000U, .pos = 18},
  [19] = {.msk = 0x00080000U, .pos = 19},
  [20] = {.msk = 0x00100000U, .pos = 20},
  [21] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const EXTI_D3PMR1_MRx[26] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
  [19] = {.msk = 0x00080000U, .pos = 19},
  [20] = {.msk = 0x00100000U, .pos = 20},
  [21] = {.msk = 0x00200000U, .pos = 21},
  [25] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const EXTI_D3PCR1L_PCSx[16] = {
  [0]  = {.msk = 0x00000003U, .pos = 0},
  [1]  = {.msk = 0x0000000CU, .pos = 2},
  [2]  = {.msk = 0x00000030U, .pos = 4},
  [3]  = {.msk = 0x000000C0U, .pos = 6},
  [4]  = {.msk = 0x00000300U, .pos = 8},
  [5]  = {.msk = 0x00000C00U, .pos = 10},
  [6]  = {.msk = 0x00003000U, .pos = 12},
  [7]  = {.msk = 0x0000C000U, .pos = 14},
  [8]  = {.msk = 0x00030000U, .pos = 16},
  [9]  = {.msk = 0x000C0000U, .pos = 18},
  [10] = {.msk = 0x00300000U, .pos = 20},
  [11] = {.msk = 0x00C00000U, .pos = 22},
  [12] = {.msk = 0x03000000U, .pos = 24},
  [13] = {.msk = 0x0C000000U, .pos = 26},
  [14] = {.msk = 0x30000000U, .pos = 28},
  [15] = {.msk = 0xC0000000U, .pos = 30},
};

field32_t const EXTI_D3PMR2_MR3x[6] = {
  [5] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const EXTI_D3PCR2L_PCS3x[6] = {
  [4] = {.msk = 0x00000030U, .pos = 4},
};

field32_t const EXTI_D3PCR2H_PCS4x[10] = {
  [9]  = {.msk = 0x0000000CU, .pos = 2},
};

field32_t const EXTI_RTSR3_TR8x[7] = {
  [2] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00100000U, .pos = 20},
  [5] = {.msk = 0x00200000U, .pos = 21},
  [6] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const EXTI_SWIER3_SWIER8x[7] = {
  [2] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00100000U, .pos = 20},
  [5] = {.msk = 0x00200000U, .pos = 21},
  [6] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const EXTI_CPUPR1_PRx[22] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
  [16] = {.msk = 0x00010000U, .pos = 16},
  [17] = {.msk = 0x00020000U, .pos = 17},
  [18] = {.msk = 0x00040000U, .pos = 18},
  [19] = {.msk = 0x00080000U, .pos = 19},
  [20] = {.msk = 0x00100000U, .pos = 20},
  [21] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const EXTI_CPUIMR3_MR6x[10] = {
  [5]  = {.msk = 0x00000002U, .pos = 1},
  [6]  = {.msk = 0x00000004U, .pos = 2},
  [7]  = {.msk = 0x00000008U, .pos = 3},
  [8]  = {.msk = 0x00000010U, .pos = 4},
  [9]  = {.msk = 0x00000020U, .pos = 5},
};

field32_t const EXTI_CPUPR3_PR8x[7] = {
  [2] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00100000U, .pos = 20},
  [5] = {.msk = 0x00200000U, .pos = 21},
  [6] = {.msk = 0x00400000U, .pos = 22},
};

/**************************************************************************************************
 * @section DELAY_BLOCK_SDMMCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated DELAY_BLOCK_SDMMCx Register Definitions */

rw_reg32_t const DELAY_BLOCK_SDMMCx_CR[3] = {
  [1] = (rw_reg32_t)0x52008000U,
  [2] = (rw_reg32_t)0x48022800U,
};

rw_reg32_t const DELAY_BLOCK_SDMMCx_CFGR[3] = {
  [1] = (rw_reg32_t)0x52008004U,
  [2] = (rw_reg32_t)0x48022804U,
};

/** @subsection DELAY_BLOCK_SDMMCx Register Field Definitions */

const field32_t DELAY_BLOCK_SDMMCx_CR_DEN    = {.msk = 0x00000001U, .pos = 0};
const field32_t DELAY_BLOCK_SDMMCx_CR_SEN    = {.msk = 0x00000002U, .pos = 1};
const field32_t DELAY_BLOCK_SDMMCx_CFGR_SEL  = {.msk = 0x0000000FU, .pos = 0};
const field32_t DELAY_BLOCK_SDMMCx_CFGR_UNIT = {.msk = 0x00007F00U, .pos = 8};
const field32_t DELAY_BLOCK_SDMMCx_CFGR_LNG  = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t DELAY_BLOCK_SDMMCx_CFGR_LNGF = {.msk = 0x80000000U, .pos = 31};

/**************************************************************************************************
 * @section DELAY_BLOCK_QUADSPI Definitions
 **************************************************************************************************/

/** @subsection DELAY_BLOCK_QUADSPI Register Definitions */

rw_reg32_t const DELAY_BLOCK_QUADSPI_CR   = (rw_reg32_t)0x52006000U;
rw_reg32_t const DELAY_BLOCK_QUADSPI_CFGR = (rw_reg32_t)0x52006004U;

/** @subsection DELAY_BLOCK_QUADSPI Register Field Definitions */

const field32_t DELAY_BLOCK_QUADSPI_CR_DEN    = {.msk = 0x00000001U, .pos = 0};
const field32_t DELAY_BLOCK_QUADSPI_CR_SEN    = {.msk = 0x00000002U, .pos = 1};
const field32_t DELAY_BLOCK_QUADSPI_CFGR_SEL  = {.msk = 0x0000000FU, .pos = 0};
const field32_t DELAY_BLOCK_QUADSPI_CFGR_UNIT = {.msk = 0x00007F00U, .pos = 8};
const field32_t DELAY_BLOCK_QUADSPI_CFGR_LNG  = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t DELAY_BLOCK_QUADSPI_CFGR_LNGF = {.msk = 0x80000000U, .pos = 31};

/**************************************************************************************************
 * @section FLASH Definitions
 **************************************************************************************************/

/** @subsection FLASH Register Definitions */

rw_reg32_t const FLASH_ACR        = (rw_reg32_t)0x52002000U;
rw_reg32_t const FLASH_ACR_       = (rw_reg32_t)0x52002100U;
rw_reg32_t const FLASH_KEYR1      = (rw_reg32_t)0x52002004U;
rw_reg32_t const FLASH_OPTKEYR    = (rw_reg32_t)0x52002008U;
rw_reg32_t const FLASH_OPTKEYR_   = (rw_reg32_t)0x52002108U;
rw_reg32_t const FLASH_CR1        = (rw_reg32_t)0x5200200CU;
rw_reg32_t const FLASH_SR1        = (rw_reg32_t)0x52002010U;
rw_reg32_t const FLASH_CCR1       = (rw_reg32_t)0x52002014U;
rw_reg32_t const FLASH_OPTCR      = (rw_reg32_t)0x52002018U;
rw_reg32_t const FLASH_OPTCR_     = (rw_reg32_t)0x52002118U;
rw_reg32_t const FLASH_OPTSR_CUR_ = (rw_reg32_t)0x5200211CU;
rw_reg32_t const FLASH_OPTSR_CUR  = (rw_reg32_t)0x5200201CU;
rw_reg32_t const FLASH_OPTSR_PRG  = (rw_reg32_t)0x52002020U;
rw_reg32_t const FLASH_OPTSR_PRG_ = (rw_reg32_t)0x52002120U;
rw_reg32_t const FLASH_OPTCCR_    = (rw_reg32_t)0x52002124U;
rw_reg32_t const FLASH_OPTCCR     = (rw_reg32_t)0x52002024U;
ro_reg32_t const FLASH_PRAR_CUR1  = (ro_reg32_t)0x52002028U;
rw_reg32_t const FLASH_PRAR_PRG1  = (rw_reg32_t)0x5200202CU;
rw_reg32_t const FLASH_SCAR_CUR1  = (rw_reg32_t)0x52002030U;
rw_reg32_t const FLASH_SCAR_PRG1  = (rw_reg32_t)0x52002034U;
ro_reg32_t const FLASH_WPSN_CUR1R = (ro_reg32_t)0x52002038U;
rw_reg32_t const FLASH_WPSN_PRG1R = (rw_reg32_t)0x5200203CU;
ro_reg32_t const FLASH_BOOT_CURR  = (ro_reg32_t)0x52002040U;
ro_reg32_t const FLASH_BOOT_PRGR  = (ro_reg32_t)0x52002044U;
rw_reg32_t const FLASH_CRCDATAR   = (rw_reg32_t)0x5200205CU;
ro_reg32_t const FLASH_ECC_FA1R   = (ro_reg32_t)0x52002060U;
ro_reg32_t const FLASH_KEYR2      = (ro_reg32_t)0x52002104U;
rw_reg32_t const FLASH_CR2        = (rw_reg32_t)0x5200210CU;
rw_reg32_t const FLASH_SR2        = (rw_reg32_t)0x52002110U;
rw_reg32_t const FLASH_CCR2       = (rw_reg32_t)0x52002114U;
ro_reg32_t const FLASH_PRAR_CUR2  = (ro_reg32_t)0x52002128U;
rw_reg32_t const FLASH_PRAR_PRG2  = (rw_reg32_t)0x5200202CU;
rw_reg32_t const FLASH_SCAR_CUR2  = (rw_reg32_t)0x52002130U;
rw_reg32_t const FLASH_SCAR_PRG2  = (rw_reg32_t)0x52002134U;
ro_reg32_t const FLASH_WPSN_CUR2R = (ro_reg32_t)0x52002138U;
rw_reg32_t const FLASH_WPSN_PRG2R = (rw_reg32_t)0x5200213CU;
ro_reg32_t const FLASH_ECC_FA2R   = (ro_reg32_t)0x52002160U;

/** @subsection Enumerated FLASH Register Definitions */

rw_reg32_t const FLASH_CRCCRx[3] = {
  [1] = (rw_reg32_t)0x52002050U,
  [2] = (rw_reg32_t)0x52002150U,
};

rw_reg32_t const FLASH_CRCSADDxR[3] = {
  [1] = (rw_reg32_t)0x52002054U,
  [2] = (rw_reg32_t)0x52002154U,
};

rw_reg32_t const FLASH_CRCEADDxR[3] = {
  [1] = (rw_reg32_t)0x52002058U,
  [2] = (rw_reg32_t)0x52002158U,
};

/** @subsection FLASH Register Field Definitions */

const field32_t FLASH_ACR_LATENCY                = {.msk = 0x00000007U, .pos = 0};
const field32_t FLASH_ACR_WRHIGHFREQ             = {.msk = 0x00000030U, .pos = 4};
const field32_t FLASH_ACR__LATENCY               = {.msk = 0x00000007U, .pos = 0};
const field32_t FLASH_ACR__WRHIGHFREQ            = {.msk = 0x00000030U, .pos = 4};
const field32_t FLASH_CR1_LOCK1                  = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_CR1_PG1                    = {.msk = 0x00000002U, .pos = 1};
const field32_t FLASH_CR1_SER1                   = {.msk = 0x00000004U, .pos = 2};
const field32_t FLASH_CR1_BER1                   = {.msk = 0x00000008U, .pos = 3};
const field32_t FLASH_CR1_PSIZE1                 = {.msk = 0x00000030U, .pos = 4};
const field32_t FLASH_CR1_FW1                    = {.msk = 0x00000040U, .pos = 6};
const field32_t FLASH_CR1_START1                 = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_CR1_SNB1                   = {.msk = 0x00000700U, .pos = 8};
const field32_t FLASH_CR1_CRC_EN                 = {.msk = 0x00008000U, .pos = 15};
const field32_t FLASH_CR1_EOPIE1                 = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_CR1_WRPERRIE1              = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_CR1_PGSERRIE1              = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_CR1_STRBERRIE1             = {.msk = 0x00080000U, .pos = 19};
const field32_t FLASH_CR1_INCERRIE1              = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_CR1_OPERRIE1               = {.msk = 0x00400000U, .pos = 22};
const field32_t FLASH_CR1_RDPERRIE1              = {.msk = 0x00800000U, .pos = 23};
const field32_t FLASH_CR1_RDSERRIE1              = {.msk = 0x01000000U, .pos = 24};
const field32_t FLASH_CR1_SNECCERRIE1            = {.msk = 0x02000000U, .pos = 25};
const field32_t FLASH_CR1_DBECCERRIE1            = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_CR1_CRCENDIE1              = {.msk = 0x08000000U, .pos = 27};
const field32_t FLASH_SR1_BSY1                   = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_SR1_WBNE1                  = {.msk = 0x00000002U, .pos = 1};
const field32_t FLASH_SR1_QW1                    = {.msk = 0x00000004U, .pos = 2};
const field32_t FLASH_SR1_CRC_BUSY1              = {.msk = 0x00000008U, .pos = 3};
const field32_t FLASH_SR1_EOP1                   = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_SR1_WRPERR1                = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_SR1_PGSERR1                = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_SR1_STRBERR1               = {.msk = 0x00080000U, .pos = 19};
const field32_t FLASH_SR1_INCERR1                = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_SR1_OPERR1                 = {.msk = 0x00400000U, .pos = 22};
const field32_t FLASH_SR1_RDPERR1                = {.msk = 0x00800000U, .pos = 23};
const field32_t FLASH_SR1_RDSERR1                = {.msk = 0x01000000U, .pos = 24};
const field32_t FLASH_SR1_SNECCERR11             = {.msk = 0x02000000U, .pos = 25};
const field32_t FLASH_SR1_DBECCERR1              = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_SR1_CRCEND1                = {.msk = 0x08000000U, .pos = 27};
const field32_t FLASH_CCR1_CLR_EOP1              = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_CCR1_CLR_WRPERR1           = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_CCR1_CLR_PGSERR1           = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_CCR1_CLR_STRBERR1          = {.msk = 0x00080000U, .pos = 19};
const field32_t FLASH_CCR1_CLR_INCERR1           = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_CCR1_CLR_OPERR1            = {.msk = 0x00400000U, .pos = 22};
const field32_t FLASH_CCR1_CLR_RDPERR1           = {.msk = 0x00800000U, .pos = 23};
const field32_t FLASH_CCR1_CLR_RDSERR1           = {.msk = 0x01000000U, .pos = 24};
const field32_t FLASH_CCR1_CLR_SNECCERR1         = {.msk = 0x02000000U, .pos = 25};
const field32_t FLASH_CCR1_CLR_DBECCERR1         = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_CCR1_CLR_CRCEND1           = {.msk = 0x08000000U, .pos = 27};
const field32_t FLASH_OPTCR_OPTLOCK              = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_OPTCR_OPTSTART             = {.msk = 0x00000002U, .pos = 1};
const field32_t FLASH_OPTCR_MER                  = {.msk = 0x00000010U, .pos = 4};
const field32_t FLASH_OPTCR_OPTCHANGEERRIE       = {.msk = 0x40000000U, .pos = 30};
const field32_t FLASH_OPTCR_SWAP_BANK            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_OPTCR__OPTLOCK             = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_OPTCR__OPTSTART            = {.msk = 0x00000002U, .pos = 1};
const field32_t FLASH_OPTCR__MER                 = {.msk = 0x00000010U, .pos = 4};
const field32_t FLASH_OPTCR__OPTCHANGEERRIE      = {.msk = 0x40000000U, .pos = 30};
const field32_t FLASH_OPTCR__SWAP_BANK           = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_OPTSR_CUR__OPT_BUSY        = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_OPTSR_CUR__BOR_LEV         = {.msk = 0x0000000CU, .pos = 2};
const field32_t FLASH_OPTSR_CUR__IWDG1_HW        = {.msk = 0x00000010U, .pos = 4};
const field32_t FLASH_OPTSR_CUR__NRST_STOP_D1    = {.msk = 0x00000040U, .pos = 6};
const field32_t FLASH_OPTSR_CUR__NRST_STBY_D1    = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_OPTSR_CUR__RDP             = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FLASH_OPTSR_CUR__FZ_IWDG_STOP    = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_OPTSR_CUR__FZ_IWDG_SDBY    = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_OPTSR_CUR__ST_RAM_SIZE     = {.msk = 0x00180000U, .pos = 19};
const field32_t FLASH_OPTSR_CUR__SECURITY        = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_OPTSR_CUR__RSS1            = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_OPTSR_CUR__PERSO_OK        = {.msk = 0x10000000U, .pos = 28};
const field32_t FLASH_OPTSR_CUR__IO_HSLV         = {.msk = 0x20000000U, .pos = 29};
const field32_t FLASH_OPTSR_CUR__OPTCHANGEERR    = {.msk = 0x40000000U, .pos = 30};
const field32_t FLASH_OPTSR_CUR__SWAP_BANK_OPT   = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_OPTSR_CUR_OPT_BUSY         = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_OPTSR_CUR_BOR_LEV          = {.msk = 0x0000000CU, .pos = 2};
const field32_t FLASH_OPTSR_CUR_IWDG1_HW         = {.msk = 0x00000010U, .pos = 4};
const field32_t FLASH_OPTSR_CUR_NRST_STOP_D1     = {.msk = 0x00000040U, .pos = 6};
const field32_t FLASH_OPTSR_CUR_NRST_STBY_D1     = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_OPTSR_CUR_RDP              = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FLASH_OPTSR_CUR_FZ_IWDG_STOP     = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_OPTSR_CUR_FZ_IWDG_SDBY     = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_OPTSR_CUR_ST_RAM_SIZE      = {.msk = 0x00180000U, .pos = 19};
const field32_t FLASH_OPTSR_CUR_SECURITY         = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_OPTSR_CUR_RSS1             = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_OPTSR_CUR_PERSO_OK         = {.msk = 0x10000000U, .pos = 28};
const field32_t FLASH_OPTSR_CUR_IO_HSLV          = {.msk = 0x20000000U, .pos = 29};
const field32_t FLASH_OPTSR_CUR_OPTCHANGEERR     = {.msk = 0x40000000U, .pos = 30};
const field32_t FLASH_OPTSR_CUR_SWAP_BANK_OPT    = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_OPTSR_PRG_BOR_LEV          = {.msk = 0x0000000CU, .pos = 2};
const field32_t FLASH_OPTSR_PRG_IWDG1_HW         = {.msk = 0x00000010U, .pos = 4};
const field32_t FLASH_OPTSR_PRG_NRST_STOP_D1     = {.msk = 0x00000040U, .pos = 6};
const field32_t FLASH_OPTSR_PRG_NRST_STBY_D1     = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_OPTSR_PRG_RDP              = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FLASH_OPTSR_PRG_FZ_IWDG_STOP     = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_OPTSR_PRG_FZ_IWDG_SDBY     = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_OPTSR_PRG_ST_RAM_SIZE      = {.msk = 0x00180000U, .pos = 19};
const field32_t FLASH_OPTSR_PRG_SECURITY         = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_OPTSR_PRG_IO_HSLV          = {.msk = 0x20000000U, .pos = 29};
const field32_t FLASH_OPTSR_PRG_SWAP_BANK_OPT    = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_OPTSR_PRG__BOR_LEV         = {.msk = 0x0000000CU, .pos = 2};
const field32_t FLASH_OPTSR_PRG__IWDG1_HW        = {.msk = 0x00000010U, .pos = 4};
const field32_t FLASH_OPTSR_PRG__NRST_STOP_D1    = {.msk = 0x00000040U, .pos = 6};
const field32_t FLASH_OPTSR_PRG__NRST_STBY_D1    = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_OPTSR_PRG__RDP             = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FLASH_OPTSR_PRG__FZ_IWDG_STOP    = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_OPTSR_PRG__FZ_IWDG_SDBY    = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_OPTSR_PRG__ST_RAM_SIZE     = {.msk = 0x00180000U, .pos = 19};
const field32_t FLASH_OPTSR_PRG__SECURITY        = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_OPTSR_PRG__IO_HSLV         = {.msk = 0x20000000U, .pos = 29};
const field32_t FLASH_OPTSR_PRG__SWAP_BANK_OPT   = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_OPTCCR__CLR_OPTCHANGEERR   = {.msk = 0x40000000U, .pos = 30};
const field32_t FLASH_OPTCCR_CLR_OPTCHANGEERR    = {.msk = 0x40000000U, .pos = 30};
const field32_t FLASH_PRAR_CUR1_PROT_AREA_START1 = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_PRAR_CUR1_PROT_AREA_END1   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_PRAR_CUR1_DMEP1            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_PRAR_PRG1_PROT_AREA_START1 = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_PRAR_PRG1_PROT_AREA_END1   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_PRAR_PRG1_DMEP1            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_SCAR_CUR1_SEC_AREA_START1  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_SCAR_CUR1_SEC_AREA_END1    = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_SCAR_CUR1_DMES1            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_SCAR_PRG1_SEC_AREA_START1  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_SCAR_PRG1_SEC_AREA_END1    = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_SCAR_PRG1_DMES1            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_WPSN_CUR1R_WRPSN1          = {.msk = 0x000000FFU, .pos = 0};
const field32_t FLASH_WPSN_PRG1R_WRPSN1          = {.msk = 0x000000FFU, .pos = 0};
const field32_t FLASH_CRCCRx_CRC_SECT            = {.msk = 0x00000007U, .pos = 0};
const field32_t FLASH_CRCCRx_ALL_BANK            = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_CRCCRx_CRC_BY_SECT         = {.msk = 0x00000100U, .pos = 8};
const field32_t FLASH_CRCCRx_ADD_SECT            = {.msk = 0x00000200U, .pos = 9};
const field32_t FLASH_CRCCRx_CLEAN_SECT          = {.msk = 0x00000400U, .pos = 10};
const field32_t FLASH_CRCCRx_START_CRC           = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_CRCCRx_CLEAN_CRC           = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_CRCCRx_CRC_BURST           = {.msk = 0x00300000U, .pos = 20};
const field32_t FLASH_ECC_FA1R_FAIL_ECC_ADDR1    = {.msk = 0x00007FFFU, .pos = 0};
const field32_t FLASH_CR2_LOCK2                  = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_CR2_PG2                    = {.msk = 0x00000002U, .pos = 1};
const field32_t FLASH_CR2_SER2                   = {.msk = 0x00000004U, .pos = 2};
const field32_t FLASH_CR2_BER2                   = {.msk = 0x00000008U, .pos = 3};
const field32_t FLASH_CR2_PSIZE2                 = {.msk = 0x00000030U, .pos = 4};
const field32_t FLASH_CR2_FW2                    = {.msk = 0x00000040U, .pos = 6};
const field32_t FLASH_CR2_START2                 = {.msk = 0x00000080U, .pos = 7};
const field32_t FLASH_CR2_SNB2                   = {.msk = 0x00000700U, .pos = 8};
const field32_t FLASH_CR2_CRC_EN                 = {.msk = 0x00008000U, .pos = 15};
const field32_t FLASH_CR2_EOPIE2                 = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_CR2_WRPERRIE2              = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_CR2_PGSERRIE2              = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_CR2_STRBERRIE2             = {.msk = 0x00080000U, .pos = 19};
const field32_t FLASH_CR2_INCERRIE2              = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_CR2_OPERRIE2               = {.msk = 0x00400000U, .pos = 22};
const field32_t FLASH_CR2_RDPERRIE2              = {.msk = 0x00800000U, .pos = 23};
const field32_t FLASH_CR2_RDSERRIE2              = {.msk = 0x01000000U, .pos = 24};
const field32_t FLASH_CR2_SNECCERRIE2            = {.msk = 0x02000000U, .pos = 25};
const field32_t FLASH_CR2_DBECCERRIE2            = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_CR2_CRCENDIE2              = {.msk = 0x08000000U, .pos = 27};
const field32_t FLASH_SR2_BSY2                   = {.msk = 0x00000001U, .pos = 0};
const field32_t FLASH_SR2_WBNE2                  = {.msk = 0x00000002U, .pos = 1};
const field32_t FLASH_SR2_QW2                    = {.msk = 0x00000004U, .pos = 2};
const field32_t FLASH_SR2_CRC_BUSY2              = {.msk = 0x00000008U, .pos = 3};
const field32_t FLASH_SR2_EOP2                   = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_SR2_WRPERR2                = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_SR2_PGSERR2                = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_SR2_STRBERR2               = {.msk = 0x00080000U, .pos = 19};
const field32_t FLASH_SR2_INCERR2                = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_SR2_OPERR2                 = {.msk = 0x00400000U, .pos = 22};
const field32_t FLASH_SR2_RDPERR2                = {.msk = 0x00800000U, .pos = 23};
const field32_t FLASH_SR2_RDSERR2                = {.msk = 0x01000000U, .pos = 24};
const field32_t FLASH_SR2_SNECCERR2              = {.msk = 0x02000000U, .pos = 25};
const field32_t FLASH_SR2_DBECCERR2              = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_SR2_CRCEND2                = {.msk = 0x08000000U, .pos = 27};
const field32_t FLASH_CCR2_CLR_EOP2              = {.msk = 0x00010000U, .pos = 16};
const field32_t FLASH_CCR2_CLR_WRPERR2           = {.msk = 0x00020000U, .pos = 17};
const field32_t FLASH_CCR2_CLR_PGSERR2           = {.msk = 0x00040000U, .pos = 18};
const field32_t FLASH_CCR2_CLR_STRBERR2          = {.msk = 0x00080000U, .pos = 19};
const field32_t FLASH_CCR2_CLR_INCERR2           = {.msk = 0x00200000U, .pos = 21};
const field32_t FLASH_CCR2_CLR_OPERR2            = {.msk = 0x00400000U, .pos = 22};
const field32_t FLASH_CCR2_CLR_RDPERR2           = {.msk = 0x00800000U, .pos = 23};
const field32_t FLASH_CCR2_CLR_RDSERR1           = {.msk = 0x01000000U, .pos = 24};
const field32_t FLASH_CCR2_CLR_SNECCERR2         = {.msk = 0x02000000U, .pos = 25};
const field32_t FLASH_CCR2_CLR_DBECCERR1         = {.msk = 0x04000000U, .pos = 26};
const field32_t FLASH_CCR2_CLR_CRCEND2           = {.msk = 0x08000000U, .pos = 27};
const field32_t FLASH_PRAR_CUR2_PROT_AREA_START2 = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_PRAR_CUR2_PROT_AREA_END2   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_PRAR_CUR2_DMEP2            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_PRAR_PRG2_PROT_AREA_START2 = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_PRAR_PRG2_PROT_AREA_END2   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_PRAR_PRG2_DMEP2            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_SCAR_CUR2_SEC_AREA_START2  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_SCAR_CUR2_SEC_AREA_END2    = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_SCAR_CUR2_DMES2            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_SCAR_PRG2_SEC_AREA_START2  = {.msk = 0x00000FFFU, .pos = 0};
const field32_t FLASH_SCAR_PRG2_SEC_AREA_END2    = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FLASH_SCAR_PRG2_DMES2            = {.msk = 0x80000000U, .pos = 31};
const field32_t FLASH_WPSN_CUR2R_WRPSN2          = {.msk = 0x000000FFU, .pos = 0};
const field32_t FLASH_WPSN_PRG2R_WRPSN2          = {.msk = 0x000000FFU, .pos = 0};
const field32_t FLASH_ECC_FA2R_FAIL_ECC_ADDR2    = {.msk = 0x00007FFFU, .pos = 0};

/** @subsection Enumerated FLASH Register Field Definitions */

field32_t const FLASH_OPTSR_PRG_RSSx[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const FLASH_BOOT_CURR_BOOT_ADDx[2] = {
  [0] = {.msk = 0x0000FFFFU, .pos = 0},
  [1] = {.msk = 0xFFFF0000U, .pos = 16},
};

/**************************************************************************************************
 * @section AXI Definitions
 **************************************************************************************************/

/** @subsection AXI Register Definitions */

ro_reg32_t const AXI_AXI_PERIPH_ID_4 = (ro_reg32_t)0x51001FD0U;
ro_reg32_t const AXI_AXI_PERIPH_ID_0 = (ro_reg32_t)0x51001FE0U;
ro_reg32_t const AXI_AXI_PERIPH_ID_1 = (ro_reg32_t)0x51001FE4U;
ro_reg32_t const AXI_AXI_PERIPH_ID_2 = (ro_reg32_t)0x51001FE8U;
ro_reg32_t const AXI_AXI_PERIPH_ID_3 = (ro_reg32_t)0x51001FECU;
ro_reg32_t const AXI_AXI_COMP_ID_0   = (ro_reg32_t)0x51001FF0U;
ro_reg32_t const AXI_AXI_COMP_ID_1   = (ro_reg32_t)0x51001FF4U;
ro_reg32_t const AXI_AXI_COMP_ID_2   = (ro_reg32_t)0x51001FF8U;
ro_reg32_t const AXI_AXI_COMP_ID_3   = (ro_reg32_t)0x51001FFCU;

/** @subsection Enumerated AXI Register Definitions */

rw_reg32_t const AXI_AXI_TARGx_FN_MOD_ISS_BM[8] = {
  [1] = (rw_reg32_t)0x51002008U,
  [2] = (rw_reg32_t)0x51003008U,
  [3] = (rw_reg32_t)0x51004008U,
  [4] = (rw_reg32_t)0x51005008U,
  [5] = (rw_reg32_t)0x51006008U,
  [6] = (rw_reg32_t)0x51007008U,
  [7] = (rw_reg32_t)0x5100800CU,
};

rw_reg32_t const AXI_AXI_TARGx_FN_MOD2[8] = {
  [1] = (rw_reg32_t)0x51002024U,
  [2] = (rw_reg32_t)0x51003024U,
  [7] = (rw_reg32_t)0x51008024U,
};

rw_reg32_t const AXI_AXI_TARGx_FN_MOD_LB[3] = {
  [1] = (rw_reg32_t)0x5100202CU,
  [2] = (rw_reg32_t)0x5100302CU,
};

rw_reg32_t const AXI_AXI_TARGx_FN_MOD[8] = {
  [1] = (rw_reg32_t)0x51002108U,
  [2] = (rw_reg32_t)0x51003108U,
  [7] = (rw_reg32_t)0x51008108U,
};

rw_reg32_t const AXI_AXI_INIx_FN_MOD2[4] = {
  [1] = (rw_reg32_t)0x51042024U,
  [3] = (rw_reg32_t)0x51044024U,
};

rw_reg32_t const AXI_AXI_INIx_FN_MOD_AHB[4] = {
  [1] = (rw_reg32_t)0x51042028U,
  [3] = (rw_reg32_t)0x51044028U,
};

rw_reg32_t const AXI_AXI_INIx_READ_QOS[7] = {
  [1] = (rw_reg32_t)0x51042100U,
  [2] = (rw_reg32_t)0x51043100U,
  [3] = (rw_reg32_t)0x51044100U,
  [4] = (rw_reg32_t)0x51045100U,
  [5] = (rw_reg32_t)0x51046100U,
  [6] = (rw_reg32_t)0x51047100U,
};

rw_reg32_t const AXI_AXI_INIx_WRITE_QOS[7] = {
  [1] = (rw_reg32_t)0x51042104U,
  [2] = (rw_reg32_t)0x51043104U,
  [3] = (rw_reg32_t)0x51044104U,
  [4] = (rw_reg32_t)0x51045104U,
  [5] = (rw_reg32_t)0x51046104U,
  [6] = (rw_reg32_t)0x51047104U,
};

rw_reg32_t const AXI_AXI_INIx_FN_MOD[7] = {
  [1] = (rw_reg32_t)0x51042108U,
  [2] = (rw_reg32_t)0x51043108U,
  [3] = (rw_reg32_t)0x51044108U,
  [4] = (rw_reg32_t)0x51045108U,
  [5] = (rw_reg32_t)0x51046108U,
  [6] = (rw_reg32_t)0x51047108U,
};

/** @subsection AXI Register Field Definitions */

const field32_t AXI_AXI_PERIPH_ID_4_JEP106CON                  = {.msk = 0x0000000FU, .pos = 0};
const field32_t AXI_AXI_PERIPH_ID_4_KCOUNT4                    = {.msk = 0x000000F0U, .pos = 4};
const field32_t AXI_AXI_PERIPH_ID_0_PARTNUM                    = {.msk = 0x000000FFU, .pos = 0};
const field32_t AXI_AXI_PERIPH_ID_1_PARTNUM                    = {.msk = 0x0000000FU, .pos = 0};
const field32_t AXI_AXI_PERIPH_ID_1_JEP106I                    = {.msk = 0x000000F0U, .pos = 4};
const field32_t AXI_AXI_PERIPH_ID_2_JEP106ID                   = {.msk = 0x00000007U, .pos = 0};
const field32_t AXI_AXI_PERIPH_ID_2_JEDEC                      = {.msk = 0x00000008U, .pos = 3};
const field32_t AXI_AXI_PERIPH_ID_2_REVISION                   = {.msk = 0x000000F0U, .pos = 4};
const field32_t AXI_AXI_PERIPH_ID_3_CUST_MOD_NUM               = {.msk = 0x0000000FU, .pos = 0};
const field32_t AXI_AXI_PERIPH_ID_3_REV_AND                    = {.msk = 0x000000F0U, .pos = 4};
const field32_t AXI_AXI_COMP_ID_0_PREAMBLE                     = {.msk = 0x000000FFU, .pos = 0};
const field32_t AXI_AXI_COMP_ID_1_PREAMBLE                     = {.msk = 0x0000000FU, .pos = 0};
const field32_t AXI_AXI_COMP_ID_1_CLASS                        = {.msk = 0x000000F0U, .pos = 4};
const field32_t AXI_AXI_COMP_ID_2_PREAMBLE                     = {.msk = 0x000000FFU, .pos = 0};
const field32_t AXI_AXI_COMP_ID_3_PREAMBLE                     = {.msk = 0x000000FFU, .pos = 0};
const field32_t AXI_AXI_TARGx_FN_MOD_ISS_BM_READ_ISS_OVERRIDE  = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_TARGx_FN_MOD_ISS_BM_WRITE_ISS_OVERRIDE = {.msk = 0x00000002U, .pos = 1};
const field32_t AXI_AXI_TARGx_FN_MOD2_BYPASS_MERGE             = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_TARGx_FN_MOD_LB_FN_MOD_LB              = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_TARGx_FN_MOD_READ_ISS_OVERRIDE         = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_TARGx_FN_MOD_WRITE_ISS_OVERRIDE        = {.msk = 0x00000002U, .pos = 1};
const field32_t AXI_AXI_INIx_FN_MOD2_BYPASS_MERGE              = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_INIx_FN_MOD_AHB_RD_INC_OVERRIDE        = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_INIx_FN_MOD_AHB_WR_INC_OVERRIDE        = {.msk = 0x00000002U, .pos = 1};
const field32_t AXI_AXI_INIx_READ_QOS_AR_QOS                   = {.msk = 0x0000000FU, .pos = 0};
const field32_t AXI_AXI_INIx_WRITE_QOS_AW_QOS                  = {.msk = 0x0000000FU, .pos = 0};
const field32_t AXI_AXI_INIx_FN_MOD_READ_ISS_OVERRIDE          = {.msk = 0x00000001U, .pos = 0};
const field32_t AXI_AXI_INIx_FN_MOD_WRITE_ISS_OVERRIDE         = {.msk = 0x00000002U, .pos = 1};

/**************************************************************************************************
 * @section HASH Definitions
 **************************************************************************************************/

/** @subsection HASH Register Definitions */

rw_reg32_t const HASH_CR       = (rw_reg32_t)0x48021400U;
rw_reg32_t const HASH_DIN      = (rw_reg32_t)0x48021404U;
rw_reg32_t const HASH_STR      = (rw_reg32_t)0x48021408U;
ro_reg32_t const HASH_HR0      = (ro_reg32_t)0x4802140CU;
ro_reg32_t const HASH_HR1      = (ro_reg32_t)0x48021410U;
ro_reg32_t const HASH_HR2      = (ro_reg32_t)0x48021414U;
ro_reg32_t const HASH_HR3      = (ro_reg32_t)0x48021418U;
ro_reg32_t const HASH_HR4      = (ro_reg32_t)0x4802141CU;
rw_reg32_t const HASH_IMR      = (rw_reg32_t)0x48021420U;
rw_reg32_t const HASH_SR       = (rw_reg32_t)0x48021424U;
rw_reg32_t const HASH_CSR0     = (rw_reg32_t)0x480214F8U;
rw_reg32_t const HASH_CSR1     = (rw_reg32_t)0x480214FCU;
rw_reg32_t const HASH_CSR2     = (rw_reg32_t)0x48021500U;
rw_reg32_t const HASH_CSR3     = (rw_reg32_t)0x48021504U;
rw_reg32_t const HASH_CSR4     = (rw_reg32_t)0x48021508U;
rw_reg32_t const HASH_CSR5     = (rw_reg32_t)0x4802150CU;
rw_reg32_t const HASH_CSR6     = (rw_reg32_t)0x48021510U;
rw_reg32_t const HASH_CSR7     = (rw_reg32_t)0x48021514U;
rw_reg32_t const HASH_CSR8     = (rw_reg32_t)0x48021518U;
rw_reg32_t const HASH_CSR9     = (rw_reg32_t)0x4802151CU;
rw_reg32_t const HASH_CSR10    = (rw_reg32_t)0x48021520U;
rw_reg32_t const HASH_CSR11    = (rw_reg32_t)0x48021524U;
rw_reg32_t const HASH_CSR12    = (rw_reg32_t)0x48021528U;
rw_reg32_t const HASH_CSR13    = (rw_reg32_t)0x4802152CU;
rw_reg32_t const HASH_CSR14    = (rw_reg32_t)0x48021530U;
rw_reg32_t const HASH_CSR15    = (rw_reg32_t)0x48021534U;
rw_reg32_t const HASH_CSR16    = (rw_reg32_t)0x48021538U;
rw_reg32_t const HASH_CSR17    = (rw_reg32_t)0x4802153CU;
rw_reg32_t const HASH_CSR18    = (rw_reg32_t)0x48021540U;
rw_reg32_t const HASH_CSR19    = (rw_reg32_t)0x48021544U;
rw_reg32_t const HASH_CSR20    = (rw_reg32_t)0x48021548U;
rw_reg32_t const HASH_CSR21    = (rw_reg32_t)0x4802154CU;
rw_reg32_t const HASH_CSR22    = (rw_reg32_t)0x48021550U;
rw_reg32_t const HASH_CSR23    = (rw_reg32_t)0x48021554U;
rw_reg32_t const HASH_CSR24    = (rw_reg32_t)0x48021558U;
rw_reg32_t const HASH_CSR25    = (rw_reg32_t)0x4802155CU;
rw_reg32_t const HASH_CSR26    = (rw_reg32_t)0x48021560U;
rw_reg32_t const HASH_CSR27    = (rw_reg32_t)0x48021564U;
rw_reg32_t const HASH_CSR28    = (rw_reg32_t)0x48021568U;
rw_reg32_t const HASH_CSR29    = (rw_reg32_t)0x4802156CU;
rw_reg32_t const HASH_CSR30    = (rw_reg32_t)0x48021570U;
rw_reg32_t const HASH_CSR31    = (rw_reg32_t)0x48021574U;
rw_reg32_t const HASH_CSR32    = (rw_reg32_t)0x48021578U;
rw_reg32_t const HASH_CSR33    = (rw_reg32_t)0x4802157CU;
rw_reg32_t const HASH_CSR34    = (rw_reg32_t)0x48021580U;
rw_reg32_t const HASH_CSR35    = (rw_reg32_t)0x48021584U;
rw_reg32_t const HASH_CSR36    = (rw_reg32_t)0x48021588U;
rw_reg32_t const HASH_CSR37    = (rw_reg32_t)0x4802158CU;
rw_reg32_t const HASH_CSR38    = (rw_reg32_t)0x48021590U;
rw_reg32_t const HASH_CSR39    = (rw_reg32_t)0x48021594U;
rw_reg32_t const HASH_CSR40    = (rw_reg32_t)0x48021598U;
rw_reg32_t const HASH_CSR41    = (rw_reg32_t)0x4802159CU;
rw_reg32_t const HASH_CSR42    = (rw_reg32_t)0x480215A0U;
rw_reg32_t const HASH_CSR43    = (rw_reg32_t)0x480215A4U;
rw_reg32_t const HASH_CSR44    = (rw_reg32_t)0x480215A8U;
rw_reg32_t const HASH_CSR45    = (rw_reg32_t)0x480215ACU;
rw_reg32_t const HASH_CSR46    = (rw_reg32_t)0x480215B0U;
rw_reg32_t const HASH_CSR47    = (rw_reg32_t)0x480215B4U;
rw_reg32_t const HASH_CSR48    = (rw_reg32_t)0x480215B8U;
rw_reg32_t const HASH_CSR49    = (rw_reg32_t)0x480215BCU;
rw_reg32_t const HASH_CSR50    = (rw_reg32_t)0x480215C0U;
rw_reg32_t const HASH_CSR51    = (rw_reg32_t)0x480215C4U;
rw_reg32_t const HASH_CSR52    = (rw_reg32_t)0x480215C8U;
rw_reg32_t const HASH_CSR53    = (rw_reg32_t)0x480215CCU;
ro_reg32_t const HASH_HASH_HR0 = (ro_reg32_t)0x48021710U;
ro_reg32_t const HASH_HASH_HR1 = (ro_reg32_t)0x48021714U;
ro_reg32_t const HASH_HASH_HR2 = (ro_reg32_t)0x48021718U;
ro_reg32_t const HASH_HASH_HR3 = (ro_reg32_t)0x4802171CU;
ro_reg32_t const HASH_HASH_HR4 = (ro_reg32_t)0x48021720U;
ro_reg32_t const HASH_HASH_HR5 = (ro_reg32_t)0x48021724U;
ro_reg32_t const HASH_HASH_HR6 = (ro_reg32_t)0x48021728U;
ro_reg32_t const HASH_HASH_HR7 = (ro_reg32_t)0x4802172CU;

/** @subsection HASH Register Field Definitions */

const field32_t HASH_CR_INIT     = {.msk = 0x00000004U, .pos = 2};
const field32_t HASH_CR_DMAE     = {.msk = 0x00000008U, .pos = 3};
const field32_t HASH_CR_DATATYPE = {.msk = 0x00000030U, .pos = 4};
const field32_t HASH_CR_MODE     = {.msk = 0x00000040U, .pos = 6};
const field32_t HASH_CR_NBW      = {.msk = 0x00000F00U, .pos = 8};
const field32_t HASH_CR_DINNE    = {.msk = 0x00001000U, .pos = 12};
const field32_t HASH_CR_MDMAT    = {.msk = 0x00002000U, .pos = 13};
const field32_t HASH_CR_LKEY     = {.msk = 0x00010000U, .pos = 16};
const field32_t HASH_STR_DCAL    = {.msk = 0x00000100U, .pos = 8};
const field32_t HASH_STR_NBLW    = {.msk = 0x0000001FU, .pos = 0};
const field32_t HASH_IMR_DCIE    = {.msk = 0x00000002U, .pos = 1};
const field32_t HASH_IMR_DINIE   = {.msk = 0x00000001U, .pos = 0};
const field32_t HASH_SR_BUSY     = {.msk = 0x00000008U, .pos = 3};
const field32_t HASH_SR_DMAS     = {.msk = 0x00000004U, .pos = 2};
const field32_t HASH_SR_DCIS     = {.msk = 0x00000002U, .pos = 1};
const field32_t HASH_SR_DINIS    = {.msk = 0x00000001U, .pos = 0};

/** @subsection Enumerated HASH Register Field Definitions */

field32_t const HASH_CR_ALGOx[2] = {
  [0] = {.msk = 0x00000080U, .pos = 7},
  [1] = {.msk = 0x00040000U, .pos = 18},
};

/**************************************************************************************************
 * @section CRYP Definitions
 **************************************************************************************************/

/** @subsection CRYP Register Definitions */

rw_reg32_t const CRYP_CR         = (rw_reg32_t)0x48021000U;
ro_reg32_t const CRYP_SR         = (ro_reg32_t)0x48021004U;
rw_reg32_t const CRYP_DIN        = (rw_reg32_t)0x48021008U;
ro_reg32_t const CRYP_DOUT       = (ro_reg32_t)0x4802100CU;
rw_reg32_t const CRYP_DMACR      = (rw_reg32_t)0x48021010U;
rw_reg32_t const CRYP_IMSCR      = (rw_reg32_t)0x48021014U;
ro_reg32_t const CRYP_RISR       = (ro_reg32_t)0x48021018U;
ro_reg32_t const CRYP_MISR       = (ro_reg32_t)0x4802101CU;
rw_reg32_t const CRYP_K0LR       = (rw_reg32_t)0x48021020U;
rw_reg32_t const CRYP_K0RR       = (rw_reg32_t)0x48021024U;
rw_reg32_t const CRYP_K1LR       = (rw_reg32_t)0x48021028U;
rw_reg32_t const CRYP_K1RR       = (rw_reg32_t)0x4802102CU;
rw_reg32_t const CRYP_K2LR       = (rw_reg32_t)0x48021030U;
rw_reg32_t const CRYP_K2RR       = (rw_reg32_t)0x48021034U;
rw_reg32_t const CRYP_K3LR       = (rw_reg32_t)0x48021038U;
rw_reg32_t const CRYP_K3RR       = (rw_reg32_t)0x4802103CU;
rw_reg32_t const CRYP_IV0LR      = (rw_reg32_t)0x48021040U;
rw_reg32_t const CRYP_IV0RR      = (rw_reg32_t)0x48021044U;
rw_reg32_t const CRYP_IV1LR      = (rw_reg32_t)0x48021048U;
rw_reg32_t const CRYP_IV1RR      = (rw_reg32_t)0x4802104CU;
rw_reg32_t const CRYP_CSGCMCCM0R = (rw_reg32_t)0x48021050U;
rw_reg32_t const CRYP_CSGCMCCM1R = (rw_reg32_t)0x48021054U;
rw_reg32_t const CRYP_CSGCMCCM2R = (rw_reg32_t)0x48021058U;
rw_reg32_t const CRYP_CSGCMCCM3R = (rw_reg32_t)0x4802105CU;
rw_reg32_t const CRYP_CSGCMCCM4R = (rw_reg32_t)0x48021060U;
rw_reg32_t const CRYP_CSGCMCCM5R = (rw_reg32_t)0x48021064U;
rw_reg32_t const CRYP_CSGCMCCM6R = (rw_reg32_t)0x48021068U;
rw_reg32_t const CRYP_CSGCMCCM7R = (rw_reg32_t)0x4802106CU;
rw_reg32_t const CRYP_CSGCM0R    = (rw_reg32_t)0x48021070U;
rw_reg32_t const CRYP_CSGCM1R    = (rw_reg32_t)0x48021074U;
rw_reg32_t const CRYP_CSGCM2R    = (rw_reg32_t)0x48021078U;
rw_reg32_t const CRYP_CSGCM3R    = (rw_reg32_t)0x4802107CU;
rw_reg32_t const CRYP_CSGCM4R    = (rw_reg32_t)0x48021080U;
rw_reg32_t const CRYP_CSGCM5R    = (rw_reg32_t)0x48021084U;
rw_reg32_t const CRYP_CSGCM6R    = (rw_reg32_t)0x48021088U;
rw_reg32_t const CRYP_CSGCM7R    = (rw_reg32_t)0x4802108CU;

/** @subsection CRYP Register Field Definitions */

const field32_t CRYP_CR_ALGODIR   = {.msk = 0x00000004U, .pos = 2};
const field32_t CRYP_CR_DATATYPE  = {.msk = 0x000000C0U, .pos = 6};
const field32_t CRYP_CR_KEYSIZE   = {.msk = 0x00000300U, .pos = 8};
const field32_t CRYP_CR_FFLUSH    = {.msk = 0x00004000U, .pos = 14};
const field32_t CRYP_CR_CRYPEN    = {.msk = 0x00008000U, .pos = 15};
const field32_t CRYP_CR_GCM_CCMPH = {.msk = 0x00030000U, .pos = 16};
const field32_t CRYP_SR_BUSY      = {.msk = 0x00000010U, .pos = 4};
const field32_t CRYP_SR_OFFU      = {.msk = 0x00000008U, .pos = 3};
const field32_t CRYP_SR_OFNE      = {.msk = 0x00000004U, .pos = 2};
const field32_t CRYP_SR_IFNF      = {.msk = 0x00000002U, .pos = 1};
const field32_t CRYP_SR_IFEM      = {.msk = 0x00000001U, .pos = 0};
const field32_t CRYP_DMACR_DOEN   = {.msk = 0x00000002U, .pos = 1};
const field32_t CRYP_DMACR_DIEN   = {.msk = 0x00000001U, .pos = 0};
const field32_t CRYP_IMSCR_OUTIM  = {.msk = 0x00000002U, .pos = 1};
const field32_t CRYP_IMSCR_INIM   = {.msk = 0x00000001U, .pos = 0};
const field32_t CRYP_RISR_OUTRIS  = {.msk = 0x00000002U, .pos = 1};
const field32_t CRYP_RISR_INRIS   = {.msk = 0x00000001U, .pos = 0};
const field32_t CRYP_MISR_OUTMIS  = {.msk = 0x00000002U, .pos = 1};
const field32_t CRYP_MISR_INMIS   = {.msk = 0x00000001U, .pos = 0};

/** @subsection Enumerated CRYP Register Field Definitions */

field32_t const CRYP_CR_ALGOMODEx[4] = {
  [0] = {.msk = 0x00000038U, .pos = 3},
  [3] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const CRYP_K0LR_B2x[56] = {
  [24] = {.msk = 0x00000001U, .pos = 0},
  [30] = {.msk = 0x00000040U, .pos = 6},
  [31] = {.msk = 0x00000080U, .pos = 7},
  [32] = {.msk = 0x00000100U, .pos = 8},
  [33] = {.msk = 0x00000200U, .pos = 9},
  [34] = {.msk = 0x00000400U, .pos = 10},
  [35] = {.msk = 0x00000800U, .pos = 11},
  [36] = {.msk = 0x00001000U, .pos = 12},
  [37] = {.msk = 0x00002000U, .pos = 13},
  [38] = {.msk = 0x00004000U, .pos = 14},
  [39] = {.msk = 0x00008000U, .pos = 15},
  [40] = {.msk = 0x00010000U, .pos = 16},
  [41] = {.msk = 0x00020000U, .pos = 17},
  [42] = {.msk = 0x00040000U, .pos = 18},
  [43] = {.msk = 0x00080000U, .pos = 19},
  [44] = {.msk = 0x00100000U, .pos = 20},
  [45] = {.msk = 0x00200000U, .pos = 21},
  [46] = {.msk = 0x00400000U, .pos = 22},
  [47] = {.msk = 0x00800000U, .pos = 23},
  [48] = {.msk = 0x01000000U, .pos = 24},
  [49] = {.msk = 0x02000000U, .pos = 25},
  [50] = {.msk = 0x04000000U, .pos = 26},
  [51] = {.msk = 0x08000000U, .pos = 27},
  [52] = {.msk = 0x10000000U, .pos = 28},
  [53] = {.msk = 0x20000000U, .pos = 29},
  [54] = {.msk = 0x40000000U, .pos = 30},
  [55] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const CRYP_K0LR_B22x[10] = {
  [5]  = {.msk = 0x00000002U, .pos = 1},
  [6]  = {.msk = 0x00000004U, .pos = 2},
  [7]  = {.msk = 0x00000008U, .pos = 3},
  [8]  = {.msk = 0x00000010U, .pos = 4},
  [9]  = {.msk = 0x00000020U, .pos = 5},
};

field32_t const CRYP_K0RR_B19x[10] = {
  [2]  = {.msk = 0x00000001U, .pos = 0},
  [3]  = {.msk = 0x00000002U, .pos = 1},
  [4]  = {.msk = 0x00000004U, .pos = 2},
  [5]  = {.msk = 0x00000008U, .pos = 3},
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [8]  = {.msk = 0x00000040U, .pos = 6},
  [9]  = {.msk = 0x00000080U, .pos = 7},
};

field32_t const CRYP_K0RR_B20x[10] = {
  [1]  = {.msk = 0x00000200U, .pos = 9},
  [2]  = {.msk = 0x00000400U, .pos = 10},
  [3]  = {.msk = 0x00000800U, .pos = 11},
  [4]  = {.msk = 0x00001000U, .pos = 12},
  [5]  = {.msk = 0x00002000U, .pos = 13},
  [6]  = {.msk = 0x00004000U, .pos = 14},
  [7]  = {.msk = 0x00008000U, .pos = 15},
  [8]  = {.msk = 0x00010000U, .pos = 16},
  [9]  = {.msk = 0x00020000U, .pos = 17},
};

field32_t const CRYP_K1LR_B1x[92] = {
  [60] = {.msk = 0x00000001U, .pos = 0},
  [70] = {.msk = 0x00000400U, .pos = 10},
  [71] = {.msk = 0x00000800U, .pos = 11},
  [72] = {.msk = 0x00001000U, .pos = 12},
  [73] = {.msk = 0x00002000U, .pos = 13},
  [74] = {.msk = 0x00004000U, .pos = 14},
  [75] = {.msk = 0x00008000U, .pos = 15},
  [76] = {.msk = 0x00010000U, .pos = 16},
  [77] = {.msk = 0x00020000U, .pos = 17},
  [78] = {.msk = 0x00040000U, .pos = 18},
  [79] = {.msk = 0x00080000U, .pos = 19},
  [80] = {.msk = 0x00100000U, .pos = 20},
  [81] = {.msk = 0x00200000U, .pos = 21},
  [82] = {.msk = 0x00400000U, .pos = 22},
  [83] = {.msk = 0x00800000U, .pos = 23},
  [84] = {.msk = 0x01000000U, .pos = 24},
  [85] = {.msk = 0x02000000U, .pos = 25},
  [86] = {.msk = 0x04000000U, .pos = 26},
  [87] = {.msk = 0x08000000U, .pos = 27},
  [88] = {.msk = 0x10000000U, .pos = 28},
  [89] = {.msk = 0x20000000U, .pos = 29},
  [90] = {.msk = 0x40000000U, .pos = 30},
  [91] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const CRYP_K1LR_B16x[10] = {
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
};

field32_t const CRYP_K1RR_B12x[10] = {
  [9]  = {.msk = 0x00000002U, .pos = 1},
};

field32_t const CRYP_K2LR_B9x[10] = {
  [6]  = {.msk = 0x00000001U, .pos = 0},
  [7]  = {.msk = 0x00000002U, .pos = 1},
  [8]  = {.msk = 0x00000004U, .pos = 2},
  [9]  = {.msk = 0x00000008U, .pos = 3},
};

field32_t const CRYP_K2LR_B10x[10] = {
  [1]  = {.msk = 0x00000020U, .pos = 5},
  [2]  = {.msk = 0x00000040U, .pos = 6},
  [3]  = {.msk = 0x00000080U, .pos = 7},
  [4]  = {.msk = 0x00000100U, .pos = 8},
  [5]  = {.msk = 0x00000200U, .pos = 9},
  [6]  = {.msk = 0x00000400U, .pos = 10},
  [7]  = {.msk = 0x00000800U, .pos = 11},
  [8]  = {.msk = 0x00001000U, .pos = 12},
  [9]  = {.msk = 0x00002000U, .pos = 13},
};

field32_t const CRYP_K2RR_Bx[96] = {
  [64] = {.msk = 0x00000001U, .pos = 0},
  [70] = {.msk = 0x00000040U, .pos = 6},
  [71] = {.msk = 0x00000080U, .pos = 7},
  [72] = {.msk = 0x00000100U, .pos = 8},
  [73] = {.msk = 0x00000200U, .pos = 9},
  [74] = {.msk = 0x00000400U, .pos = 10},
  [75] = {.msk = 0x00000800U, .pos = 11},
  [76] = {.msk = 0x00001000U, .pos = 12},
  [77] = {.msk = 0x00002000U, .pos = 13},
  [78] = {.msk = 0x00004000U, .pos = 14},
  [79] = {.msk = 0x00008000U, .pos = 15},
  [80] = {.msk = 0x00010000U, .pos = 16},
  [81] = {.msk = 0x00020000U, .pos = 17},
  [82] = {.msk = 0x00040000U, .pos = 18},
  [83] = {.msk = 0x00080000U, .pos = 19},
  [84] = {.msk = 0x00100000U, .pos = 20},
  [85] = {.msk = 0x00200000U, .pos = 21},
  [86] = {.msk = 0x00400000U, .pos = 22},
  [87] = {.msk = 0x00800000U, .pos = 23},
  [88] = {.msk = 0x01000000U, .pos = 24},
  [89] = {.msk = 0x02000000U, .pos = 25},
  [90] = {.msk = 0x04000000U, .pos = 26},
  [91] = {.msk = 0x08000000U, .pos = 27},
  [92] = {.msk = 0x10000000U, .pos = 28},
  [93] = {.msk = 0x20000000U, .pos = 29},
  [94] = {.msk = 0x40000000U, .pos = 30},
  [95] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const CRYP_K2RR_B6x[10] = {
  [5]  = {.msk = 0x00000002U, .pos = 1},
  [6]  = {.msk = 0x00000004U, .pos = 2},
  [7]  = {.msk = 0x00000008U, .pos = 3},
  [8]  = {.msk = 0x00000010U, .pos = 4},
  [9]  = {.msk = 0x00000020U, .pos = 5},
};

field32_t const CRYP_K3LR_B3x[10] = {
  [3]  = {.msk = 0x00000002U, .pos = 1},
  [4]  = {.msk = 0x00000004U, .pos = 2},
  [5]  = {.msk = 0x00000008U, .pos = 3},
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [8]  = {.msk = 0x00000040U, .pos = 6},
  [9]  = {.msk = 0x00000080U, .pos = 7},
};

field32_t const CRYP_IV0LR_IVx[32] = {
  [0]  = {.msk = 0x80000000U, .pos = 31},
  [1]  = {.msk = 0x40000000U, .pos = 30},
  [2]  = {.msk = 0x20000000U, .pos = 29},
  [3]  = {.msk = 0x10000000U, .pos = 28},
  [4]  = {.msk = 0x08000000U, .pos = 27},
  [5]  = {.msk = 0x04000000U, .pos = 26},
  [6]  = {.msk = 0x02000000U, .pos = 25},
  [7]  = {.msk = 0x01000000U, .pos = 24},
  [8]  = {.msk = 0x00800000U, .pos = 23},
  [9]  = {.msk = 0x00400000U, .pos = 22},
  [10] = {.msk = 0x00200000U, .pos = 21},
  [11] = {.msk = 0x00100000U, .pos = 20},
  [12] = {.msk = 0x00080000U, .pos = 19},
  [13] = {.msk = 0x00040000U, .pos = 18},
  [14] = {.msk = 0x00020000U, .pos = 17},
  [15] = {.msk = 0x00010000U, .pos = 16},
  [16] = {.msk = 0x00008000U, .pos = 15},
  [17] = {.msk = 0x00004000U, .pos = 14},
  [18] = {.msk = 0x00002000U, .pos = 13},
  [19] = {.msk = 0x00001000U, .pos = 12},
  [20] = {.msk = 0x00000800U, .pos = 11},
  [21] = {.msk = 0x00000400U, .pos = 10},
  [22] = {.msk = 0x00000200U, .pos = 9},
  [23] = {.msk = 0x00000100U, .pos = 8},
  [24] = {.msk = 0x00000080U, .pos = 7},
  [25] = {.msk = 0x00000040U, .pos = 6},
  [26] = {.msk = 0x00000020U, .pos = 5},
  [27] = {.msk = 0x00000010U, .pos = 4},
  [28] = {.msk = 0x00000008U, .pos = 3},
  [29] = {.msk = 0x00000004U, .pos = 2},
  [31] = {.msk = 0x00000001U, .pos = 0},
};

field32_t const CRYP_IV0LR_IV3x[2] = {
  [0] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const CRYP_IV0RR_IV6x[4] = {
  [0] = {.msk = 0x00000008U, .pos = 3},
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const CRYP_IV1LR_IV9x[6] = {
  [0] = {.msk = 0x00000020U, .pos = 5},
  [1] = {.msk = 0x00000010U, .pos = 4},
  [2] = {.msk = 0x00000008U, .pos = 3},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const CRYP_IV1RR_IV1x[28] = {
  [0]  = {.msk = 0x08000000U, .pos = 27},
  [1]  = {.msk = 0x04000000U, .pos = 26},
  [2]  = {.msk = 0x02000000U, .pos = 25},
  [3]  = {.msk = 0x01000000U, .pos = 24},
  [4]  = {.msk = 0x00800000U, .pos = 23},
  [5]  = {.msk = 0x00400000U, .pos = 22},
  [6]  = {.msk = 0x00200000U, .pos = 21},
  [7]  = {.msk = 0x00100000U, .pos = 20},
  [8]  = {.msk = 0x00080000U, .pos = 19},
  [9]  = {.msk = 0x00040000U, .pos = 18},
  [10] = {.msk = 0x00020000U, .pos = 17},
  [11] = {.msk = 0x00010000U, .pos = 16},
  [12] = {.msk = 0x00008000U, .pos = 15},
  [13] = {.msk = 0x00004000U, .pos = 14},
  [14] = {.msk = 0x00002000U, .pos = 13},
  [15] = {.msk = 0x00001000U, .pos = 12},
  [16] = {.msk = 0x00000800U, .pos = 11},
  [17] = {.msk = 0x00000400U, .pos = 10},
  [18] = {.msk = 0x00000200U, .pos = 9},
  [19] = {.msk = 0x00000100U, .pos = 8},
  [27] = {.msk = 0x00000001U, .pos = 0},
};

field32_t const CRYP_IV1RR_IV12x[8] = {
  [0] = {.msk = 0x00000080U, .pos = 7},
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000020U, .pos = 5},
  [3] = {.msk = 0x00000010U, .pos = 4},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000004U, .pos = 2},
  [6] = {.msk = 0x00000002U, .pos = 1},
};

/**************************************************************************************************
 * @section DCMI Definitions
 **************************************************************************************************/

/** @subsection DCMI Register Definitions */

rw_reg32_t const DCMI_CR     = (rw_reg32_t)0x48020000U;
ro_reg32_t const DCMI_SR     = (ro_reg32_t)0x48020004U;
ro_reg32_t const DCMI_RIS    = (ro_reg32_t)0x48020008U;
rw_reg32_t const DCMI_IER    = (rw_reg32_t)0x4802000CU;
ro_reg32_t const DCMI_MIS    = (ro_reg32_t)0x48020010U;
rw_reg32_t const DCMI_ICR    = (rw_reg32_t)0x48020014U;
rw_reg32_t const DCMI_ESCR   = (rw_reg32_t)0x48020018U;
rw_reg32_t const DCMI_ESUR   = (rw_reg32_t)0x4802001CU;
rw_reg32_t const DCMI_CWSTRT = (rw_reg32_t)0x48020020U;
rw_reg32_t const DCMI_CWSIZE = (rw_reg32_t)0x48020024U;
ro_reg32_t const DCMI_DR     = (ro_reg32_t)0x48020028U;

/** @subsection DCMI Register Field Definitions */

const field32_t DCMI_CR_OELS        = {.msk = 0x00100000U, .pos = 20};
const field32_t DCMI_CR_LSM         = {.msk = 0x00080000U, .pos = 19};
const field32_t DCMI_CR_OEBS        = {.msk = 0x00040000U, .pos = 18};
const field32_t DCMI_CR_BSM         = {.msk = 0x00030000U, .pos = 16};
const field32_t DCMI_CR_ENABLE      = {.msk = 0x00004000U, .pos = 14};
const field32_t DCMI_CR_EDM         = {.msk = 0x00000C00U, .pos = 10};
const field32_t DCMI_CR_FCRC        = {.msk = 0x00000300U, .pos = 8};
const field32_t DCMI_CR_VSPOL       = {.msk = 0x00000080U, .pos = 7};
const field32_t DCMI_CR_HSPOL       = {.msk = 0x00000040U, .pos = 6};
const field32_t DCMI_CR_PCKPOL      = {.msk = 0x00000020U, .pos = 5};
const field32_t DCMI_CR_ESS         = {.msk = 0x00000010U, .pos = 4};
const field32_t DCMI_CR_JPEG        = {.msk = 0x00000008U, .pos = 3};
const field32_t DCMI_CR_CROP        = {.msk = 0x00000004U, .pos = 2};
const field32_t DCMI_CR_CM          = {.msk = 0x00000002U, .pos = 1};
const field32_t DCMI_CR_CAPTURE     = {.msk = 0x00000001U, .pos = 0};
const field32_t DCMI_SR_FNE         = {.msk = 0x00000004U, .pos = 2};
const field32_t DCMI_SR_VSYNC       = {.msk = 0x00000002U, .pos = 1};
const field32_t DCMI_SR_HSYNC       = {.msk = 0x00000001U, .pos = 0};
const field32_t DCMI_RIS_LINE_RIS   = {.msk = 0x00000010U, .pos = 4};
const field32_t DCMI_RIS_VSYNC_RIS  = {.msk = 0x00000008U, .pos = 3};
const field32_t DCMI_RIS_ERR_RIS    = {.msk = 0x00000004U, .pos = 2};
const field32_t DCMI_RIS_OVR_RIS    = {.msk = 0x00000002U, .pos = 1};
const field32_t DCMI_RIS_FRAME_RIS  = {.msk = 0x00000001U, .pos = 0};
const field32_t DCMI_IER_LINE_IE    = {.msk = 0x00000010U, .pos = 4};
const field32_t DCMI_IER_VSYNC_IE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DCMI_IER_ERR_IE     = {.msk = 0x00000004U, .pos = 2};
const field32_t DCMI_IER_OVR_IE     = {.msk = 0x00000002U, .pos = 1};
const field32_t DCMI_IER_FRAME_IE   = {.msk = 0x00000001U, .pos = 0};
const field32_t DCMI_MIS_LINE_MIS   = {.msk = 0x00000010U, .pos = 4};
const field32_t DCMI_MIS_VSYNC_MIS  = {.msk = 0x00000008U, .pos = 3};
const field32_t DCMI_MIS_ERR_MIS    = {.msk = 0x00000004U, .pos = 2};
const field32_t DCMI_MIS_OVR_MIS    = {.msk = 0x00000002U, .pos = 1};
const field32_t DCMI_MIS_FRAME_MIS  = {.msk = 0x00000001U, .pos = 0};
const field32_t DCMI_ICR_LINE_ISC   = {.msk = 0x00000010U, .pos = 4};
const field32_t DCMI_ICR_VSYNC_ISC  = {.msk = 0x00000008U, .pos = 3};
const field32_t DCMI_ICR_ERR_ISC    = {.msk = 0x00000004U, .pos = 2};
const field32_t DCMI_ICR_OVR_ISC    = {.msk = 0x00000002U, .pos = 1};
const field32_t DCMI_ICR_FRAME_ISC  = {.msk = 0x00000001U, .pos = 0};
const field32_t DCMI_ESCR_FEC       = {.msk = 0xFF000000U, .pos = 24};
const field32_t DCMI_ESCR_LEC       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DCMI_ESCR_LSC       = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DCMI_ESCR_FSC       = {.msk = 0x000000FFU, .pos = 0};
const field32_t DCMI_ESUR_FEU       = {.msk = 0xFF000000U, .pos = 24};
const field32_t DCMI_ESUR_LEU       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DCMI_ESUR_LSU       = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DCMI_ESUR_FSU       = {.msk = 0x000000FFU, .pos = 0};
const field32_t DCMI_CWSTRT_VST     = {.msk = 0x1FFF0000U, .pos = 16};
const field32_t DCMI_CWSTRT_HOFFCNT = {.msk = 0x00003FFFU, .pos = 0};
const field32_t DCMI_CWSIZE_VLINE   = {.msk = 0x3FFF0000U, .pos = 16};
const field32_t DCMI_CWSIZE_CAPCNT  = {.msk = 0x00003FFFU, .pos = 0};

/** @subsection Enumerated DCMI Register Field Definitions */

field32_t const DCMI_DR_BYTEx[4] = {
  [0] = {.msk = 0x000000FFU, .pos = 0},
  [1] = {.msk = 0x0000FF00U, .pos = 8},
  [2] = {.msk = 0x00FF0000U, .pos = 16},
  [3] = {.msk = 0xFF000000U, .pos = 24},
};

/**************************************************************************************************
 * @section OTGx_HS_GLOBAL Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_GLOBAL Register Definitions */

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GOTGCTL[3] = {
  [1] = (rw_reg32_t)0x40040000U,
  [2] = (rw_reg32_t)0x40080000U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GOTGINT[3] = {
  [1] = (rw_reg32_t)0x40040004U,
  [2] = (rw_reg32_t)0x40080004U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GAHBCFG[3] = {
  [1] = (rw_reg32_t)0x40040008U,
  [2] = (rw_reg32_t)0x40080008U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GUSBCFG[3] = {
  [1] = (rw_reg32_t)0x4004000CU,
  [2] = (rw_reg32_t)0x4008000CU,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRSTCTL[3] = {
  [1] = (rw_reg32_t)0x40040010U,
  [2] = (rw_reg32_t)0x40080010U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GINTSTS[3] = {
  [1] = (rw_reg32_t)0x40040014U,
  [2] = (rw_reg32_t)0x40080014U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GINTMSK[3] = {
  [1] = (rw_reg32_t)0x40040018U,
  [2] = (rw_reg32_t)0x40080018U,
};

ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST[3] = {
  [1] = (ro_reg32_t)0x4004001CU,
  [2] = (ro_reg32_t)0x4008001CU,
};

ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST[3] = {
  [1] = (ro_reg32_t)0x40040020U,
  [2] = (ro_reg32_t)0x40080020U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXFSIZ[3] = {
  [1] = (rw_reg32_t)0x40040024U,
  [2] = (rw_reg32_t)0x40080024U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_HNPTXFSIZ_HOST[3] = {
  [1] = (rw_reg32_t)0x40040028U,
  [2] = (rw_reg32_t)0x40080028U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_DIEPTXF0_DEVICE[3] = {
  [1] = (rw_reg32_t)0x40040028U,
  [2] = (rw_reg32_t)0x40080028U,
};

ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS[3] = {
  [1] = (ro_reg32_t)0x4004002CU,
  [2] = (ro_reg32_t)0x4008002CU,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GCCFG[3] = {
  [1] = (rw_reg32_t)0x40040038U,
  [2] = (rw_reg32_t)0x40080038U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_CID[3] = {
  [1] = (rw_reg32_t)0x4004003CU,
  [2] = (rw_reg32_t)0x4008003CU,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_HPTXFSIZ[3] = {
  [1] = (rw_reg32_t)0x40040100U,
  [2] = (rw_reg32_t)0x40080100U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_DIEPTXFx[3][8] = {
  [1] = {
    [1] = (rw_reg32_t)0x40040104U,
    [2] = (rw_reg32_t)0x40040108U,
    [3] = (rw_reg32_t)0x4004011CU,
    [4] = (rw_reg32_t)0x40040120U,
    [5] = (rw_reg32_t)0x40040124U,
    [6] = (rw_reg32_t)0x40040128U,
    [7] = (rw_reg32_t)0x4004012CU,
  },
  [2] = {
    [1] = (rw_reg32_t)0x40080104U,
    [2] = (rw_reg32_t)0x40080108U,
    [3] = (rw_reg32_t)0x4008011CU,
    [4] = (rw_reg32_t)0x40080120U,
    [5] = (rw_reg32_t)0x40080124U,
    [6] = (rw_reg32_t)0x40080128U,
    [7] = (rw_reg32_t)0x4008012CU,
  },
};

ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE[3] = {
  [1] = (ro_reg32_t)0x4004001CU,
  [2] = (ro_reg32_t)0x4008001CU,
};

ro_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE[3] = {
  [1] = (ro_reg32_t)0x40040020U,
  [2] = (ro_reg32_t)0x40080020U,
};

rw_reg32_t const OTGx_HS_GLOBAL_OTG_HS_GLPMCFG[3] = {
  [1] = (rw_reg32_t)0x40040054U,
  [2] = (rw_reg32_t)0x40080054U,
};

/** @subsection OTGx_HS_GLOBAL Register Field Definitions */

const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_SRQSCS            = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_SRQ               = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_HNGSCS            = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_HNPRQ             = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_HSHNPEN           = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_DHNPEN            = {.msk = 0x00000800U, .pos = 11};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_CIDSTS            = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_DBCT              = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_ASVLD             = {.msk = 0x00040000U, .pos = 18};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_BSVLD             = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGCTL_EHEN              = {.msk = 0x00001000U, .pos = 12};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_SEDET             = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_SRSSCHG           = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_HNSSCHG           = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_HNGDET            = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_ADTOCHG           = {.msk = 0x00040000U, .pos = 18};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_DBCDNE            = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GOTGINT_IDCHNG            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_GINT              = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_HBSTLEN           = {.msk = 0x0000001EU, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_DMAEN             = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_TXFELVL           = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GAHBCFG_PTXFELVL          = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_TOCAL             = {.msk = 0x00000007U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PHYSEL            = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_SRPCAP            = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_HNPCAP            = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_TRDT              = {.msk = 0x00003C00U, .pos = 10};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PHYLPCS           = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIFSLS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIAR            = {.msk = 0x00040000U, .pos = 18};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPICSM           = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIEVBUSD        = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIEVBUSI        = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_TSDPS             = {.msk = 0x00400000U, .pos = 22};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PCCI              = {.msk = 0x00800000U, .pos = 23};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_PTCI              = {.msk = 0x01000000U, .pos = 24};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_ULPIIPD           = {.msk = 0x02000000U, .pos = 25};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_FHMOD             = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GUSBCFG_FDMOD             = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_CSRST             = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_HSRST             = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_FCRST             = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_RXFFLSH           = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_TXFFLSH           = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_TXFNUM            = {.msk = 0x000007C0U, .pos = 6};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_AHBIDL            = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRSTCTL_DMAREQ            = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_CMOD              = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_MMIS              = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_OTGINT            = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_SOF               = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_RXFLVL            = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_NPTXFE            = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_GINAKEFF          = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_BOUTNAKEFF        = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_ESUSP             = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_USBSUSP           = {.msk = 0x00000800U, .pos = 11};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_USBRST            = {.msk = 0x00001000U, .pos = 12};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_ENUMDNE           = {.msk = 0x00002000U, .pos = 13};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_ISOODRP           = {.msk = 0x00004000U, .pos = 14};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_EOPF              = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_IEPINT            = {.msk = 0x00040000U, .pos = 18};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_OEPINT            = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_IISOIXFR          = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_PXFR_INCOMPISOOUT = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_DATAFSUSP         = {.msk = 0x00400000U, .pos = 22};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_HPRTINT           = {.msk = 0x01000000U, .pos = 24};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_HCINT             = {.msk = 0x02000000U, .pos = 25};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_PTXFE             = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_CIDSCHG           = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_DISCINT           = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_SRQINT            = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTSTS_WKUINT            = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_MMISM             = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_OTGINT            = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_SOFM              = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_RXFLVLM           = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_NPTXFEM           = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_GINAKEFFM         = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_GONAKEFFM         = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_ESUSPM            = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_USBSUSPM          = {.msk = 0x00000800U, .pos = 11};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_USBRST            = {.msk = 0x00001000U, .pos = 12};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_ENUMDNEM          = {.msk = 0x00002000U, .pos = 13};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_ISOODRPM          = {.msk = 0x00004000U, .pos = 14};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_EOPFM             = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_IEPINT            = {.msk = 0x00040000U, .pos = 18};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_OEPINT            = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_IISOIXFRM         = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_PXFRM_IISOOXFRM   = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_FSUSPM            = {.msk = 0x00400000U, .pos = 22};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_PRTIM             = {.msk = 0x01000000U, .pos = 24};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_HCIM              = {.msk = 0x02000000U, .pos = 25};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_PTXFEM            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_CIDSCHGM          = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_DISCINT           = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_SRQIM             = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_WUIM              = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_RSTDE             = {.msk = 0x00800000U, .pos = 23};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GINTMSK_LPMINTM           = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_CHNUM        = {.msk = 0x0000000FU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_BCNT         = {.msk = 0x00007FF0U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_DPID         = {.msk = 0x00018000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_HOST_PKTSTS       = {.msk = 0x001E0000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_CHNUM        = {.msk = 0x0000000FU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_BCNT         = {.msk = 0x00007FF0U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_DPID         = {.msk = 0x00018000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_HOST_PKTSTS       = {.msk = 0x001E0000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXFSIZ_RXFD              = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_HNPTXFSIZ_HOST_NPTXFSA    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_HNPTXFSIZ_HOST_NPTXFD     = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXF0_DEVICE_TX0FSA    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXF0_DEVICE_TX0FD     = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS_NPTXFSAV         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS_NPTQXSAV         = {.msk = 0x00FF0000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GNPTXSTS_NPTXQTOP         = {.msk = 0x7F000000U, .pos = 24};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PWRDWN              = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_BCDEN               = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_DCDEN               = {.msk = 0x00040000U, .pos = 18};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PDEN                = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_SDEN                = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_VBDEN               = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_DCDET               = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PDET                = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_SDET                = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GCCFG_PS2DET              = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_GLOBAL_OTG_HS_HPTXFSIZ_PTXSA            = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_HPTXFSIZ_PTXFD            = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXFx_INEPTXSA         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_DIEPTXFx_INEPTXFD         = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_EPNUM      = {.msk = 0x0000000FU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_BCNT       = {.msk = 0x00007FF0U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_DPID       = {.msk = 0x00018000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_PKTSTS     = {.msk = 0x001E0000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSR_DEVICE_FRMNUM     = {.msk = 0x01E00000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_EPNUM      = {.msk = 0x0000000FU, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_BCNT       = {.msk = 0x00007FF0U, .pos = 4};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_DPID       = {.msk = 0x00018000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_PKTSTS     = {.msk = 0x001E0000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GRXSTSP_DEVICE_FRMNUM     = {.msk = 0x01E00000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMEN             = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMACK            = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_BESL              = {.msk = 0x0000003CU, .pos = 2};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_REMWAKE           = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_L1SSEN            = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_BESLTHRS          = {.msk = 0x00000F00U, .pos = 8};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_L1DSEN            = {.msk = 0x00001000U, .pos = 12};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMRST            = {.msk = 0x00006000U, .pos = 13};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_SLPSTS            = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_L1RSMOK           = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMCHIDX          = {.msk = 0x001E0000U, .pos = 17};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMRCNT           = {.msk = 0x00E00000U, .pos = 21};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_SNDLPM            = {.msk = 0x01000000U, .pos = 24};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_LPMRCNTSTS        = {.msk = 0x0E000000U, .pos = 25};
const field32_t OTGx_HS_GLOBAL_OTG_HS_GLPMCFG_ENBESL            = {.msk = 0x10000000U, .pos = 28};

/**************************************************************************************************
 * @section OTGx_HS_HOST Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_HOST Register Definitions */

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCFG[3] = {
  [1] = (rw_reg32_t)0x40040400U,
  [2] = (rw_reg32_t)0x40080400U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HFIR[3] = {
  [1] = (rw_reg32_t)0x40040404U,
  [2] = (rw_reg32_t)0x40080404U,
};

ro_reg32_t const OTGx_HS_HOST_OTG_HS_HFNUM[3] = {
  [1] = (ro_reg32_t)0x40040408U,
  [2] = (ro_reg32_t)0x40080408U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HPTXSTS[3] = {
  [1] = (rw_reg32_t)0x40040410U,
  [2] = (rw_reg32_t)0x40080410U,
};

ro_reg32_t const OTGx_HS_HOST_OTG_HS_HAINT[3] = {
  [1] = (ro_reg32_t)0x40040414U,
  [2] = (ro_reg32_t)0x40080414U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HAINTMSK[3] = {
  [1] = (rw_reg32_t)0x40040418U,
  [2] = (rw_reg32_t)0x40080418U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HPRT[3] = {
  [1] = (rw_reg32_t)0x40040440U,
  [2] = (rw_reg32_t)0x40080440U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCCHARx[3][16] = {
  [1] = {
    [0]  = (rw_reg32_t)0x40040500U,
    [1]  = (rw_reg32_t)0x40040520U,
    [2]  = (rw_reg32_t)0x40040540U,
    [3]  = (rw_reg32_t)0x40040560U,
    [4]  = (rw_reg32_t)0x40040580U,
    [5]  = (rw_reg32_t)0x400405A0U,
    [6]  = (rw_reg32_t)0x400405C0U,
    [7]  = (rw_reg32_t)0x400405E0U,
    [8]  = (rw_reg32_t)0x40040600U,
    [9]  = (rw_reg32_t)0x40040620U,
    [10] = (rw_reg32_t)0x40040640U,
    [11] = (rw_reg32_t)0x40040660U,
    [12] = (rw_reg32_t)0x40040678U,
    [13] = (rw_reg32_t)0x40040690U,
    [14] = (rw_reg32_t)0x400406A8U,
    [15] = (rw_reg32_t)0x400406C0U,
  },
  [2] = {
    [0]  = (rw_reg32_t)0x40080500U,
    [1]  = (rw_reg32_t)0x40080520U,
    [2]  = (rw_reg32_t)0x40080540U,
    [3]  = (rw_reg32_t)0x40080560U,
    [4]  = (rw_reg32_t)0x40080580U,
    [5]  = (rw_reg32_t)0x400805A0U,
    [6]  = (rw_reg32_t)0x400805C0U,
    [7]  = (rw_reg32_t)0x400805E0U,
    [8]  = (rw_reg32_t)0x40080600U,
    [9]  = (rw_reg32_t)0x40080620U,
    [10] = (rw_reg32_t)0x40080640U,
    [11] = (rw_reg32_t)0x40080660U,
    [12] = (rw_reg32_t)0x40080678U,
    [13] = (rw_reg32_t)0x40080690U,
    [14] = (rw_reg32_t)0x400806A8U,
    [15] = (rw_reg32_t)0x400806C0U,
  },
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCSPLTx[3][16] = {
  [1] = {
    [0]  = (rw_reg32_t)0x40040504U,
    [1]  = (rw_reg32_t)0x40040524U,
    [2]  = (rw_reg32_t)0x40040544U,
    [3]  = (rw_reg32_t)0x40040564U,
    [4]  = (rw_reg32_t)0x40040584U,
    [5]  = (rw_reg32_t)0x400405A4U,
    [6]  = (rw_reg32_t)0x400405C4U,
    [7]  = (rw_reg32_t)0x400405E4U,
    [8]  = (rw_reg32_t)0x40040604U,
    [9]  = (rw_reg32_t)0x40040624U,
    [10] = (rw_reg32_t)0x40040644U,
    [11] = (rw_reg32_t)0x40040664U,
    [12] = (rw_reg32_t)0x4004067CU,
    [13] = (rw_reg32_t)0x40040694U,
    [14] = (rw_reg32_t)0x400406ACU,
    [15] = (rw_reg32_t)0x400406C4U,
  },
  [2] = {
    [0]  = (rw_reg32_t)0x40080504U,
    [1]  = (rw_reg32_t)0x40080524U,
    [2]  = (rw_reg32_t)0x40080544U,
    [3]  = (rw_reg32_t)0x40080564U,
    [4]  = (rw_reg32_t)0x40080584U,
    [5]  = (rw_reg32_t)0x400805A4U,
    [6]  = (rw_reg32_t)0x400805C4U,
    [7]  = (rw_reg32_t)0x400805E4U,
    [8]  = (rw_reg32_t)0x40080604U,
    [9]  = (rw_reg32_t)0x40080624U,
    [10] = (rw_reg32_t)0x40080644U,
    [11] = (rw_reg32_t)0x40080664U,
    [12] = (rw_reg32_t)0x4008067CU,
    [13] = (rw_reg32_t)0x40080694U,
    [14] = (rw_reg32_t)0x400806ACU,
    [15] = (rw_reg32_t)0x400806C4U,
  },
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTx[3][16] = {
  [1] = {
    [0]  = (rw_reg32_t)0x40040508U,
    [1]  = (rw_reg32_t)0x40040528U,
    [2]  = (rw_reg32_t)0x40040548U,
    [3]  = (rw_reg32_t)0x40040568U,
    [4]  = (rw_reg32_t)0x40040588U,
    [5]  = (rw_reg32_t)0x400405A8U,
    [6]  = (rw_reg32_t)0x400405C8U,
    [7]  = (rw_reg32_t)0x400405E8U,
    [8]  = (rw_reg32_t)0x40040608U,
    [9]  = (rw_reg32_t)0x40040628U,
    [10] = (rw_reg32_t)0x40040648U,
    [11] = (rw_reg32_t)0x40040668U,
    [12] = (rw_reg32_t)0x40040680U,
    [13] = (rw_reg32_t)0x40040698U,
    [14] = (rw_reg32_t)0x400406B0U,
    [15] = (rw_reg32_t)0x400406C8U,
  },
  [2] = {
    [0]  = (rw_reg32_t)0x40080508U,
    [1]  = (rw_reg32_t)0x40080528U,
    [2]  = (rw_reg32_t)0x40080548U,
    [3]  = (rw_reg32_t)0x40080568U,
    [4]  = (rw_reg32_t)0x40080588U,
    [5]  = (rw_reg32_t)0x400805A8U,
    [6]  = (rw_reg32_t)0x400805C8U,
    [7]  = (rw_reg32_t)0x400805E8U,
    [8]  = (rw_reg32_t)0x40080608U,
    [9]  = (rw_reg32_t)0x40080628U,
    [10] = (rw_reg32_t)0x40080648U,
    [11] = (rw_reg32_t)0x40080668U,
    [12] = (rw_reg32_t)0x40080680U,
    [13] = (rw_reg32_t)0x40080698U,
    [14] = (rw_reg32_t)0x400806B0U,
    [15] = (rw_reg32_t)0x400806C8U,
  },
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK0[3] = {
  [1] = (rw_reg32_t)0x4004050CU,
  [2] = (rw_reg32_t)0x4008050CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK1[3] = {
  [1] = (rw_reg32_t)0x4004052CU,
  [2] = (rw_reg32_t)0x4008052CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK2[3] = {
  [1] = (rw_reg32_t)0x4004054CU,
  [2] = (rw_reg32_t)0x4008054CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK3[3] = {
  [1] = (rw_reg32_t)0x4004056CU,
  [2] = (rw_reg32_t)0x4008056CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK4[3] = {
  [1] = (rw_reg32_t)0x4004058CU,
  [2] = (rw_reg32_t)0x4008058CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK5[3] = {
  [1] = (rw_reg32_t)0x400405ACU,
  [2] = (rw_reg32_t)0x400805ACU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK6[3] = {
  [1] = (rw_reg32_t)0x400405CCU,
  [2] = (rw_reg32_t)0x400805CCU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK7[3] = {
  [1] = (rw_reg32_t)0x400405ECU,
  [2] = (rw_reg32_t)0x400805ECU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK8[3] = {
  [1] = (rw_reg32_t)0x4004060CU,
  [2] = (rw_reg32_t)0x4008060CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK9[3] = {
  [1] = (rw_reg32_t)0x4004062CU,
  [2] = (rw_reg32_t)0x4008062CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK10[3] = {
  [1] = (rw_reg32_t)0x4004064CU,
  [2] = (rw_reg32_t)0x4008064CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK11[3] = {
  [1] = (rw_reg32_t)0x4004066CU,
  [2] = (rw_reg32_t)0x4008066CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCTSIZx[3][16] = {
  [1] = {
    [0]  = (rw_reg32_t)0x40040510U,
    [1]  = (rw_reg32_t)0x40040530U,
    [2]  = (rw_reg32_t)0x40040550U,
    [3]  = (rw_reg32_t)0x40040570U,
    [4]  = (rw_reg32_t)0x40040590U,
    [5]  = (rw_reg32_t)0x400405B0U,
    [6]  = (rw_reg32_t)0x400405D0U,
    [7]  = (rw_reg32_t)0x400405F0U,
    [8]  = (rw_reg32_t)0x40040610U,
    [9]  = (rw_reg32_t)0x40040630U,
    [10] = (rw_reg32_t)0x40040650U,
    [11] = (rw_reg32_t)0x40040670U,
    [12] = (rw_reg32_t)0x40040688U,
    [13] = (rw_reg32_t)0x400406A0U,
    [14] = (rw_reg32_t)0x400406B8U,
    [15] = (rw_reg32_t)0x400406D0U,
  },
  [2] = {
    [0]  = (rw_reg32_t)0x40080510U,
    [1]  = (rw_reg32_t)0x40080530U,
    [2]  = (rw_reg32_t)0x40080550U,
    [3]  = (rw_reg32_t)0x40080570U,
    [4]  = (rw_reg32_t)0x40080590U,
    [5]  = (rw_reg32_t)0x400805B0U,
    [6]  = (rw_reg32_t)0x400805D0U,
    [7]  = (rw_reg32_t)0x400805F0U,
    [8]  = (rw_reg32_t)0x40080610U,
    [9]  = (rw_reg32_t)0x40080630U,
    [10] = (rw_reg32_t)0x40080650U,
    [11] = (rw_reg32_t)0x40080670U,
    [12] = (rw_reg32_t)0x40080688U,
    [13] = (rw_reg32_t)0x400806A0U,
    [14] = (rw_reg32_t)0x400806B8U,
    [15] = (rw_reg32_t)0x400806D0U,
  },
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCDMAx[3][16] = {
  [1] = {
    [0]  = (rw_reg32_t)0x40040514U,
    [1]  = (rw_reg32_t)0x40040534U,
    [2]  = (rw_reg32_t)0x40040554U,
    [3]  = (rw_reg32_t)0x40040574U,
    [4]  = (rw_reg32_t)0x40040594U,
    [5]  = (rw_reg32_t)0x400405B4U,
    [6]  = (rw_reg32_t)0x400405D4U,
    [7]  = (rw_reg32_t)0x400405F4U,
    [8]  = (rw_reg32_t)0x40040614U,
    [9]  = (rw_reg32_t)0x40040634U,
    [10] = (rw_reg32_t)0x40040654U,
    [11] = (rw_reg32_t)0x40040674U,
    [12] = (rw_reg32_t)0x4004068CU,
    [13] = (rw_reg32_t)0x400406A4U,
    [14] = (rw_reg32_t)0x400406BCU,
    [15] = (rw_reg32_t)0x400406D4U,
  },
  [2] = {
    [0]  = (rw_reg32_t)0x40080514U,
    [1]  = (rw_reg32_t)0x40080534U,
    [2]  = (rw_reg32_t)0x40080554U,
    [3]  = (rw_reg32_t)0x40080574U,
    [4]  = (rw_reg32_t)0x40080594U,
    [5]  = (rw_reg32_t)0x400805B4U,
    [6]  = (rw_reg32_t)0x400805D4U,
    [7]  = (rw_reg32_t)0x400805F4U,
    [8]  = (rw_reg32_t)0x40080614U,
    [9]  = (rw_reg32_t)0x40080634U,
    [10] = (rw_reg32_t)0x40080654U,
    [11] = (rw_reg32_t)0x40080674U,
    [12] = (rw_reg32_t)0x4008068CU,
    [13] = (rw_reg32_t)0x400806A4U,
    [14] = (rw_reg32_t)0x400806BCU,
    [15] = (rw_reg32_t)0x400806D4U,
  },
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK12[3] = {
  [1] = (rw_reg32_t)0x40040684U,
  [2] = (rw_reg32_t)0x40080684U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK13[3] = {
  [1] = (rw_reg32_t)0x4004069CU,
  [2] = (rw_reg32_t)0x4008069CU,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK14[3] = {
  [1] = (rw_reg32_t)0x400406B4U,
  [2] = (rw_reg32_t)0x400806B4U,
};

rw_reg32_t const OTGx_HS_HOST_OTG_HS_HCINTMSK15[3] = {
  [1] = (rw_reg32_t)0x400406CCU,
  [2] = (rw_reg32_t)0x400806CCU,
};

/** @subsection OTGx_HS_HOST Register Field Definitions */

const field32_t OTGx_HS_HOST_OTG_HS_HCFG_FSLSPCS      = {.msk = 0x00000003U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCFG_FSLSS        = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HFIR_FRIVL        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HFNUM_FRNUM       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HFNUM_FTREM       = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_HOST_OTG_HS_HPTXSTS_PTXFSAVL  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HPTXSTS_PTXQSAV   = {.msk = 0x00FF0000U, .pos = 16};
const field32_t OTGx_HS_HOST_OTG_HS_HPTXSTS_PTXQTOP   = {.msk = 0xFF000000U, .pos = 24};
const field32_t OTGx_HS_HOST_OTG_HS_HAINT_HAINT       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HAINTMSK_HAINTM   = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PCSTS        = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PCDET        = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PENA         = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PENCHNG      = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_POCA         = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_POCCHNG      = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PRES         = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PSUSP        = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PRST         = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PLSTS        = {.msk = 0x00000C00U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PPWR         = {.msk = 0x00001000U, .pos = 12};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PTCTL        = {.msk = 0x0001E000U, .pos = 13};
const field32_t OTGx_HS_HOST_OTG_HS_HPRT_PSPD         = {.msk = 0x00060000U, .pos = 17};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_MPSIZ     = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_EPNUM     = {.msk = 0x00007800U, .pos = 11};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_EPDIR     = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_LSDEV     = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_EPTYP     = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_MC        = {.msk = 0x00300000U, .pos = 20};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_DAD       = {.msk = 0x1FC00000U, .pos = 22};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_ODDFRM    = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_CHDIS     = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_HOST_OTG_HS_HCCHARx_CHENA     = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_PRTADDR   = {.msk = 0x0000007FU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_HUBADDR   = {.msk = 0x00003F80U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_XACTPOS   = {.msk = 0x0000C000U, .pos = 14};
const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_COMPLSPLT = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_HOST_OTG_HS_HCSPLTx_SPLITEN   = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_XFRC       = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_CHH        = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_AHBERR     = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_STALL      = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_NAK        = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_ACK        = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_NYET       = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_TXERR      = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_BBERR      = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_FRMOR      = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTx_DTERR      = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK0_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK1_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK2_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK3_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK4_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK5_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK6_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK7_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK8_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_XFRCM   = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_CHHM    = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_AHBERR  = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_STALLM  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_NAKM    = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_ACKM    = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_NYET    = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_TXERRM  = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_BBERRM  = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_FRMORM  = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK9_DTERRM  = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_XFRCM  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_CHHM   = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_AHBERR = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_STALLM = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_NAKM   = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_ACKM   = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_NYET   = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_TXERRM = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_BBERRM = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_FRMORM = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK10_DTERRM = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_XFRCM  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_CHHM   = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_AHBERR = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_STALLM = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_NAKM   = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_ACKM   = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_NYET   = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_TXERRM = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_BBERRM = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_FRMORM = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK11_DTERRM = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCTSIZx_XFRSIZ    = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCTSIZx_PKTCNT    = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_HOST_OTG_HS_HCTSIZx_DPID      = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_XFRCM  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_CHHM   = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_AHBERR = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_STALLM = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_NAKM   = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_ACKM   = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_NYET   = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_TXERRM = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_BBERRM = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_FRMORM = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK12_DTERRM = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_XFRCM  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_CHHM   = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_AHBERR = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_STALLM = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_NAKM   = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_ACKM   = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_NYET   = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_TXERRM = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_BBERRM = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_FRMORM = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK13_DTERRM = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_XFRCM  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_CHHM   = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_AHBERR = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_STALLM = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_NAKM   = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_ACKM   = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_NYET   = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_TXERRM = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_BBERRM = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_FRMORM = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK14_DTERRM = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_XFRCM  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_CHHM   = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_AHBERR = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_STALL  = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_NAKM   = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_ACKM   = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_NYET   = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_TXERRM = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_BBERRM = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_FRMORM = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_HOST_OTG_HS_HCINTMSK15_DTERRM = {.msk = 0x00000400U, .pos = 10};

/**************************************************************************************************
 * @section OTGx_HS_DEVICE Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_DEVICE Register Definitions */

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DCFG[3] = {
  [1] = (rw_reg32_t)0x40040800U,
  [2] = (rw_reg32_t)0x40080800U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DCTL[3] = {
  [1] = (rw_reg32_t)0x40040804U,
  [2] = (rw_reg32_t)0x40080804U,
};

ro_reg32_t const OTGx_HS_DEVICE_OTG_HS_DSTS[3] = {
  [1] = (ro_reg32_t)0x40040808U,
  [2] = (ro_reg32_t)0x40080808U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPMSK[3] = {
  [1] = (rw_reg32_t)0x40040810U,
  [2] = (rw_reg32_t)0x40080810U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPMSK[3] = {
  [1] = (rw_reg32_t)0x40040814U,
  [2] = (rw_reg32_t)0x40080814U,
};

ro_reg32_t const OTGx_HS_DEVICE_OTG_HS_DAINT[3] = {
  [1] = (ro_reg32_t)0x40040818U,
  [2] = (ro_reg32_t)0x40080818U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DAINTMSK[3] = {
  [1] = (rw_reg32_t)0x4004081CU,
  [2] = (rw_reg32_t)0x4008081CU,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DVBUSDIS[3] = {
  [1] = (rw_reg32_t)0x40040828U,
  [2] = (rw_reg32_t)0x40080828U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DVBUSPULSE[3] = {
  [1] = (rw_reg32_t)0x4004082CU,
  [2] = (rw_reg32_t)0x4008082CU,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DTHRCTL[3] = {
  [1] = (rw_reg32_t)0x40040830U,
  [2] = (rw_reg32_t)0x40080830U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPEMPMSK[3] = {
  [1] = (rw_reg32_t)0x40040834U,
  [2] = (rw_reg32_t)0x40080834U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DEACHINT[3] = {
  [1] = (rw_reg32_t)0x40040838U,
  [2] = (rw_reg32_t)0x40080838U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DEACHINTMSK[3] = {
  [1] = (rw_reg32_t)0x4004083CU,
  [2] = (rw_reg32_t)0x4008083CU,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPCTLx[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40040900U,
    [1] = (rw_reg32_t)0x40040920U,
    [2] = (rw_reg32_t)0x40040940U,
    [3] = (rw_reg32_t)0x40040960U,
    [4] = (rw_reg32_t)0x40040980U,
    [5] = (rw_reg32_t)0x400409A0U,
    [6] = (rw_reg32_t)0x400409C0U,
    [7] = (rw_reg32_t)0x400409E0U,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40080900U,
    [1] = (rw_reg32_t)0x40080920U,
    [2] = (rw_reg32_t)0x40080940U,
    [3] = (rw_reg32_t)0x40080960U,
    [4] = (rw_reg32_t)0x40080980U,
    [5] = (rw_reg32_t)0x400809A0U,
    [6] = (rw_reg32_t)0x400809C0U,
    [7] = (rw_reg32_t)0x400809E0U,
  },
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPINTx[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40040908U,
    [1] = (rw_reg32_t)0x40040928U,
    [2] = (rw_reg32_t)0x40040948U,
    [3] = (rw_reg32_t)0x40040968U,
    [4] = (rw_reg32_t)0x40040988U,
    [5] = (rw_reg32_t)0x400409A8U,
    [6] = (rw_reg32_t)0x400409C8U,
    [7] = (rw_reg32_t)0x400409E8U,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40080908U,
    [1] = (rw_reg32_t)0x40080928U,
    [2] = (rw_reg32_t)0x40080948U,
    [3] = (rw_reg32_t)0x40080968U,
    [4] = (rw_reg32_t)0x40080988U,
    [5] = (rw_reg32_t)0x400809A8U,
    [6] = (rw_reg32_t)0x400809C8U,
    [7] = (rw_reg32_t)0x400809E8U,
  },
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ0[3] = {
  [1] = (rw_reg32_t)0x40040910U,
  [2] = (rw_reg32_t)0x40080910U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPDMAx[3][6] = {
  [1] = {
    [1] = (rw_reg32_t)0x40040914U,
    [2] = (rw_reg32_t)0x40040934U,
    [3] = (rw_reg32_t)0x40040954U,
    [4] = (rw_reg32_t)0x40040974U,
    [5] = (rw_reg32_t)0x40040994U,
  },
  [2] = {
    [1] = (rw_reg32_t)0x40080914U,
    [2] = (rw_reg32_t)0x40080934U,
    [3] = (rw_reg32_t)0x40080954U,
    [4] = (rw_reg32_t)0x40080974U,
    [5] = (rw_reg32_t)0x40080994U,
  },
};

ro_reg32_t const OTGx_HS_DEVICE_OTG_HS_DTXFSTSx[3][8] = {
  [1] = {
    [0] = (ro_reg32_t)0x40040918U,
    [1] = (ro_reg32_t)0x40040938U,
    [2] = (ro_reg32_t)0x40040958U,
    [3] = (ro_reg32_t)0x40040978U,
    [4] = (ro_reg32_t)0x40040998U,
    [5] = (ro_reg32_t)0x400409B8U,
    [6] = (ro_reg32_t)0x400409A4U,
    [7] = (ro_reg32_t)0x400409ACU,
  },
  [2] = {
    [0] = (ro_reg32_t)0x40080918U,
    [1] = (ro_reg32_t)0x40080938U,
    [2] = (ro_reg32_t)0x40080958U,
    [3] = (ro_reg32_t)0x40080978U,
    [4] = (ro_reg32_t)0x40080998U,
    [5] = (ro_reg32_t)0x400809B8U,
    [6] = (ro_reg32_t)0x400809A4U,
    [7] = (ro_reg32_t)0x400809ACU,
  },
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1[3] = {
  [1] = (rw_reg32_t)0x40040930U,
  [2] = (rw_reg32_t)0x40080930U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2[3] = {
  [1] = (rw_reg32_t)0x40040950U,
  [2] = (rw_reg32_t)0x40080950U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3[3] = {
  [1] = (rw_reg32_t)0x40040970U,
  [2] = (rw_reg32_t)0x40080970U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4[3] = {
  [1] = (rw_reg32_t)0x40040990U,
  [2] = (rw_reg32_t)0x40080990U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5[3] = {
  [1] = (rw_reg32_t)0x400409B0U,
  [2] = (rw_reg32_t)0x400809B0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL0[3] = {
  [1] = (rw_reg32_t)0x40040B00U,
  [2] = (rw_reg32_t)0x40080B00U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL1[3] = {
  [1] = (rw_reg32_t)0x40040B20U,
  [2] = (rw_reg32_t)0x40080B20U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL2[3] = {
  [1] = (rw_reg32_t)0x40040B40U,
  [2] = (rw_reg32_t)0x40080B40U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL3[3] = {
  [1] = (rw_reg32_t)0x40040B60U,
  [2] = (rw_reg32_t)0x40080B60U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPINTx[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40040B08U,
    [1] = (rw_reg32_t)0x40040B28U,
    [2] = (rw_reg32_t)0x40040B48U,
    [3] = (rw_reg32_t)0x40040B68U,
    [4] = (rw_reg32_t)0x40040B88U,
    [5] = (rw_reg32_t)0x40040BA8U,
    [6] = (rw_reg32_t)0x40040BC8U,
    [7] = (rw_reg32_t)0x40040BE8U,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40080B08U,
    [1] = (rw_reg32_t)0x40080B28U,
    [2] = (rw_reg32_t)0x40080B48U,
    [3] = (rw_reg32_t)0x40080B68U,
    [4] = (rw_reg32_t)0x40080B88U,
    [5] = (rw_reg32_t)0x40080BA8U,
    [6] = (rw_reg32_t)0x40080BC8U,
    [7] = (rw_reg32_t)0x40080BE8U,
  },
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0[3] = {
  [1] = (rw_reg32_t)0x40040B10U,
  [2] = (rw_reg32_t)0x40080B10U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1[3] = {
  [1] = (rw_reg32_t)0x40040B30U,
  [2] = (rw_reg32_t)0x40080B30U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2[3] = {
  [1] = (rw_reg32_t)0x40040B50U,
  [2] = (rw_reg32_t)0x40080B50U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3[3] = {
  [1] = (rw_reg32_t)0x40040B70U,
  [2] = (rw_reg32_t)0x40080B70U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4[3] = {
  [1] = (rw_reg32_t)0x40040B90U,
  [2] = (rw_reg32_t)0x40080B90U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6[3] = {
  [1] = (rw_reg32_t)0x400409A0U,
  [2] = (rw_reg32_t)0x400809A0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7[3] = {
  [1] = (rw_reg32_t)0x400409A8U,
  [2] = (rw_reg32_t)0x400809A8U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL4[3] = {
  [1] = (rw_reg32_t)0x40040B80U,
  [2] = (rw_reg32_t)0x40080B80U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL5[3] = {
  [1] = (rw_reg32_t)0x40040BA0U,
  [2] = (rw_reg32_t)0x40080BA0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL6[3] = {
  [1] = (rw_reg32_t)0x40040BC0U,
  [2] = (rw_reg32_t)0x40080BC0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPCTL7[3] = {
  [1] = (rw_reg32_t)0x40040BE0U,
  [2] = (rw_reg32_t)0x40080BE0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5[3] = {
  [1] = (rw_reg32_t)0x40040BB0U,
  [2] = (rw_reg32_t)0x40080BB0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6[3] = {
  [1] = (rw_reg32_t)0x40040BD0U,
  [2] = (rw_reg32_t)0x40080BD0U,
};

rw_reg32_t const OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7[3] = {
  [1] = (rw_reg32_t)0x40040BF0U,
  [2] = (rw_reg32_t)0x40080BF0U,
};

/** @subsection OTGx_HS_DEVICE Register Field Definitions */

const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_DSPD                = {.msk = 0x00000003U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_NZLSOHSK            = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_DAD                 = {.msk = 0x000007F0U, .pos = 4};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_PFIVL               = {.msk = 0x00001800U, .pos = 11};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCFG_PERSCHIVL           = {.msk = 0x03000000U, .pos = 24};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_RWUSIG              = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_SDIS                = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_GINSTS              = {.msk = 0x00000004U, .pos = 2};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_GONSTS              = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_TCTL                = {.msk = 0x00000070U, .pos = 4};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_SGINAK              = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_CGINAK              = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_SGONAK              = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_CGONAK              = {.msk = 0x00000400U, .pos = 10};
const field32_t OTGx_HS_DEVICE_OTG_HS_DCTL_POPRGDNE            = {.msk = 0x00000800U, .pos = 11};
const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_SUSPSTS             = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_ENUMSPD             = {.msk = 0x00000006U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_EERR                = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_DEVICE_OTG_HS_DSTS_FNSOF               = {.msk = 0x003FFF00U, .pos = 8};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_XFRCM            = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_EPDM             = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_TOM              = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_ITTXFEMSK        = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_INEPNMM          = {.msk = 0x00000020U, .pos = 5};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_INEPNEM          = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_TXFURM           = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPMSK_BIM              = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_XFRCM            = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_EPDM             = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_STUPM            = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_OTEPDM           = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_B2BSTUP          = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_OPEM             = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPMSK_BOIM             = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_DEVICE_OTG_HS_DAINT_IEPINT             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DAINT_OEPINT             = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DAINTMSK_IEPM            = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DAINTMSK_OEPM            = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DVBUSDIS_VBUSDT          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DVBUSPULSE_DVBUSP        = {.msk = 0x00000FFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_NONISOTHREN      = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_ISOTHREN         = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_TXTHRLEN         = {.msk = 0x000007FCU, .pos = 2};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_RXTHREN          = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_RXTHRLEN         = {.msk = 0x03FE0000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTHRCTL_ARPEN            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPEMPMSK_INEPTXFEM     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINT_IEP1INT         = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINT_OEP1INT         = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINTMSK_IEP1INTM     = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DEACHINTMSK_OEP1INTM     = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_TXFNUM          = {.msk = 0x03C00000U, .pos = 22};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPCTLx_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_XFRC            = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_EPDISD          = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_TOC             = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_ITTXFE          = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_INEPNE          = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_TXFE            = {.msk = 0x00000080U, .pos = 7};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_TXFIFOUDRN      = {.msk = 0x00000100U, .pos = 8};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_BNA             = {.msk = 0x00000200U, .pos = 9};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_PKTDRPSTS       = {.msk = 0x00000800U, .pos = 11};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_BERR            = {.msk = 0x00001000U, .pos = 12};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPINTx_NAK             = {.msk = 0x00002000U, .pos = 13};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ0_XFRSIZ         = {.msk = 0x0000007FU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ0_PKTCNT         = {.msk = 0x00180000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DTXFSTSx_INEPTFSAV       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ1_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ2_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ3_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ4_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ5_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_MPSIZ           = {.msk = 0x00000003U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL0_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL1_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL2_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL3_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_XFRC            = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_EPDISD          = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_STUP            = {.msk = 0x00000008U, .pos = 3};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_OTEPDIS         = {.msk = 0x00000010U, .pos = 4};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_B2BSTUP         = {.msk = 0x00000040U, .pos = 6};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPINTx_NYET            = {.msk = 0x00004000U, .pos = 14};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0_XFRSIZ         = {.msk = 0x0000007FU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0_PKTCNT         = {.msk = 0x00080000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ0_STUPCNT        = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ1_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ2_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ3_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ4_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ6_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DIEPTSIZ7_MCNT           = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL4_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL5_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL6_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_MPSIZ           = {.msk = 0x000007FFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_USBAEP          = {.msk = 0x00008000U, .pos = 15};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EONUM_DPID      = {.msk = 0x00010000U, .pos = 16};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_NAKSTS          = {.msk = 0x00020000U, .pos = 17};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EPTYP           = {.msk = 0x000C0000U, .pos = 18};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SNPM            = {.msk = 0x00100000U, .pos = 20};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_STALL           = {.msk = 0x00200000U, .pos = 21};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_CNAK            = {.msk = 0x04000000U, .pos = 26};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SNAK            = {.msk = 0x08000000U, .pos = 27};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SD0PID_SEVNFRM  = {.msk = 0x10000000U, .pos = 28};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_SODDFRM         = {.msk = 0x20000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EPDIS           = {.msk = 0x40000000U, .pos = 30};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPCTL7_EPENA           = {.msk = 0x80000000U, .pos = 31};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ5_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ6_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7_XFRSIZ         = {.msk = 0x0007FFFFU, .pos = 0};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7_PKTCNT         = {.msk = 0x1FF80000U, .pos = 19};
const field32_t OTGx_HS_DEVICE_OTG_HS_DOEPTSIZ7_RXDPID_STUPCNT = {.msk = 0x60000000U, .pos = 29};

/**************************************************************************************************
 * @section OTGx_HS_PWRCLK Definitions
 **************************************************************************************************/

/** @subsection Enumerated OTGx_HS_PWRCLK Register Definitions */

rw_reg32_t const OTGx_HS_PWRCLK_OTG_HS_PCGCR[3] = {
  [1] = (rw_reg32_t)0x40040E00U,
  [2] = (rw_reg32_t)0x40080E00U,
};

/** @subsection OTGx_HS_PWRCLK Register Field Definitions */

const field32_t OTGx_HS_PWRCLK_OTG_HS_PCGCR_STPPCLK  = {.msk = 0x00000001U, .pos = 0};
const field32_t OTGx_HS_PWRCLK_OTG_HS_PCGCR_GATEHCLK = {.msk = 0x00000002U, .pos = 1};
const field32_t OTGx_HS_PWRCLK_OTG_HS_PCGCR_PHYSUSP  = {.msk = 0x00000010U, .pos = 4};

/**************************************************************************************************
 * @section ETHERNET_MAC Definitions
 **************************************************************************************************/

/** @subsection ETHERNET_MAC Register Definitions */

rw_reg32_t const ETHERNET_MAC_DMAMR                              = (rw_reg32_t)0x40029000U;
rw_reg32_t const ETHERNET_MAC_DMASBMR                            = (rw_reg32_t)0x40029004U;
ro_reg32_t const ETHERNET_MAC_DMAISR                             = (ro_reg32_t)0x40029008U;
ro_reg32_t const ETHERNET_MAC_DMADSR                             = (ro_reg32_t)0x4002900CU;
rw_reg32_t const ETHERNET_MAC_DMACCR                             = (rw_reg32_t)0x40029100U;
rw_reg32_t const ETHERNET_MAC_DMACTXCR                           = (rw_reg32_t)0x40029104U;
rw_reg32_t const ETHERNET_MAC_DMACRXCR                           = (rw_reg32_t)0x40029108U;
rw_reg32_t const ETHERNET_MAC_DMACTXDLAR                         = (rw_reg32_t)0x40029114U;
rw_reg32_t const ETHERNET_MAC_DMACRXDLAR                         = (rw_reg32_t)0x4002911CU;
rw_reg32_t const ETHERNET_MAC_DMACTXDTPR                         = (rw_reg32_t)0x40029120U;
rw_reg32_t const ETHERNET_MAC_DMACRXDTPR                         = (rw_reg32_t)0x40029128U;
rw_reg32_t const ETHERNET_MAC_DMACTXRLR                          = (rw_reg32_t)0x4002912CU;
rw_reg32_t const ETHERNET_MAC_DMACRXRLR                          = (rw_reg32_t)0x40029130U;
rw_reg32_t const ETHERNET_MAC_DMACIER                            = (rw_reg32_t)0x40029134U;
rw_reg32_t const ETHERNET_MAC_DMACRXIWTR                         = (rw_reg32_t)0x40029138U;
ro_reg32_t const ETHERNET_MAC_DMACCATXDR                         = (ro_reg32_t)0x40029144U;
ro_reg32_t const ETHERNET_MAC_DMACCARXDR                         = (ro_reg32_t)0x4002914CU;
ro_reg32_t const ETHERNET_MAC_DMACCATXBR                         = (ro_reg32_t)0x40029154U;
ro_reg32_t const ETHERNET_MAC_DMACCARXBR                         = (ro_reg32_t)0x4002915CU;
rw_reg32_t const ETHERNET_MAC_DMACSR                             = (rw_reg32_t)0x40029160U;
ro_reg32_t const ETHERNET_MAC_DMACMFCR                           = (ro_reg32_t)0x4002916CU;
rw_reg32_t const ETHERNET_MAC_MTLOMR                             = (rw_reg32_t)0x40028C00U;
ro_reg32_t const ETHERNET_MAC_MTLISR                             = (ro_reg32_t)0x40028C20U;
rw_reg32_t const ETHERNET_MAC_MTLTXQOMR                          = (rw_reg32_t)0x40028D00U;
ro_reg32_t const ETHERNET_MAC_MTLTXQUR                           = (ro_reg32_t)0x40028D04U;
ro_reg32_t const ETHERNET_MAC_MTLTXQDR                           = (ro_reg32_t)0x40028D08U;
rw_reg32_t const ETHERNET_MAC_MTLQICSR                           = (rw_reg32_t)0x40028D2CU;
rw_reg32_t const ETHERNET_MAC_MTLRXQOMR                          = (rw_reg32_t)0x40028D30U;
ro_reg32_t const ETHERNET_MAC_MTLRXQMPOCR                        = (ro_reg32_t)0x40028D34U;
ro_reg32_t const ETHERNET_MAC_MTLRXQDR                           = (ro_reg32_t)0x40028D38U;
rw_reg32_t const ETHERNET_MAC_MACCR                              = (rw_reg32_t)0x40028000U;
rw_reg32_t const ETHERNET_MAC_MACECR                             = (rw_reg32_t)0x40028004U;
rw_reg32_t const ETHERNET_MAC_MACPFR                             = (rw_reg32_t)0x40028008U;
rw_reg32_t const ETHERNET_MAC_MACWTR                             = (rw_reg32_t)0x4002800CU;
rw_reg32_t const ETHERNET_MAC_MACHT0R                            = (rw_reg32_t)0x40028010U;
rw_reg32_t const ETHERNET_MAC_MACHT1R                            = (rw_reg32_t)0x40028014U;
rw_reg32_t const ETHERNET_MAC_MACVTR                             = (rw_reg32_t)0x40028050U;
rw_reg32_t const ETHERNET_MAC_MACVHTR                            = (rw_reg32_t)0x40028058U;
rw_reg32_t const ETHERNET_MAC_MACVIR                             = (rw_reg32_t)0x40028060U;
rw_reg32_t const ETHERNET_MAC_MACIVIR                            = (rw_reg32_t)0x40028064U;
rw_reg32_t const ETHERNET_MAC_MACQTXFCR                          = (rw_reg32_t)0x40028070U;
rw_reg32_t const ETHERNET_MAC_MACRXFCR                           = (rw_reg32_t)0x40028090U;
ro_reg32_t const ETHERNET_MAC_MACISR                             = (ro_reg32_t)0x400280B0U;
rw_reg32_t const ETHERNET_MAC_MACIER                             = (rw_reg32_t)0x400280B4U;
ro_reg32_t const ETHERNET_MAC_MACRXTXSR                          = (ro_reg32_t)0x400280B8U;
rw_reg32_t const ETHERNET_MAC_MACPCSR                            = (rw_reg32_t)0x400280C0U;
rw_reg32_t const ETHERNET_MAC_MACRWKPFR                          = (rw_reg32_t)0x400280C4U;
rw_reg32_t const ETHERNET_MAC_MACLCSR                            = (rw_reg32_t)0x400280D0U;
rw_reg32_t const ETHERNET_MAC_MACLTCR                            = (rw_reg32_t)0x400280D4U;
rw_reg32_t const ETHERNET_MAC_MACLETR                            = (rw_reg32_t)0x400280D8U;
rw_reg32_t const ETHERNET_MAC_MAC1USTCR                          = (rw_reg32_t)0x400280DCU;
ro_reg32_t const ETHERNET_MAC_MACVR                              = (ro_reg32_t)0x40028110U;
ro_reg32_t const ETHERNET_MAC_MACHWF1R                           = (ro_reg32_t)0x40028120U;
ro_reg32_t const ETHERNET_MAC_MACHWF2R                           = (ro_reg32_t)0x40028124U;
rw_reg32_t const ETHERNET_MAC_MACMDIOAR                          = (rw_reg32_t)0x40028200U;
rw_reg32_t const ETHERNET_MAC_MACMDIODR                          = (rw_reg32_t)0x40028204U;
rw_reg32_t const ETHERNET_MAC_MACARPAR                           = (rw_reg32_t)0x40028AE0U;
rw_reg32_t const ETHERNET_MAC_MACA0HR                            = (rw_reg32_t)0x40028300U;
rw_reg32_t const ETHERNET_MAC_MACA1HR                            = (rw_reg32_t)0x40028308U;
rw_reg32_t const ETHERNET_MAC_MACA2HR                            = (rw_reg32_t)0x40028310U;
rw_reg32_t const ETHERNET_MAC_MACA3HR                            = (rw_reg32_t)0x40028318U;
rw_reg32_t const ETHERNET_MAC_MMC_CONTROL                        = (rw_reg32_t)0x40028700U;
ro_reg32_t const ETHERNET_MAC_MMC_RX_INTERRUPT                   = (ro_reg32_t)0x40028704U;
ro_reg32_t const ETHERNET_MAC_MMC_TX_INTERRUPT                   = (ro_reg32_t)0x40028708U;
rw_reg32_t const ETHERNET_MAC_MMC_RX_INTERRUPT_MASK              = (rw_reg32_t)0x4002870CU;
rw_reg32_t const ETHERNET_MAC_MMC_TX_INTERRUPT_MASK              = (rw_reg32_t)0x40028710U;
ro_reg32_t const ETHERNET_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS   = (ro_reg32_t)0x4002874CU;
ro_reg32_t const ETHERNET_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS = (ro_reg32_t)0x40028750U;
ro_reg32_t const ETHERNET_MAC_TX_PACKET_COUNT_GOOD               = (ro_reg32_t)0x40028768U;
ro_reg32_t const ETHERNET_MAC_RX_CRC_ERROR_PACKETS               = (ro_reg32_t)0x40028794U;
ro_reg32_t const ETHERNET_MAC_RX_ALIGNMENT_ERROR_PACKETS         = (ro_reg32_t)0x40028798U;
ro_reg32_t const ETHERNET_MAC_RX_UNICAST_PACKETS_GOOD            = (ro_reg32_t)0x400287C4U;
ro_reg32_t const ETHERNET_MAC_TX_LPI_USEC_CNTR                   = (ro_reg32_t)0x400287ECU;
ro_reg32_t const ETHERNET_MAC_TX_LPI_TRAN_CNTR                   = (ro_reg32_t)0x400287F0U;
ro_reg32_t const ETHERNET_MAC_RX_LPI_USEC_CNTR                   = (ro_reg32_t)0x400287F4U;
ro_reg32_t const ETHERNET_MAC_RX_LPI_TRAN_CNTR                   = (ro_reg32_t)0x400287F8U;
rw_reg32_t const ETHERNET_MAC_MACL3L4C0R                         = (rw_reg32_t)0x40028900U;
rw_reg32_t const ETHERNET_MAC_MACL4A0R                           = (rw_reg32_t)0x40028904U;
ro_reg32_t const ETHERNET_MAC_MACDR                              = (ro_reg32_t)0x40028114U;
rw_reg32_t const ETHERNET_MAC_MACL3A00R                          = (rw_reg32_t)0x40028910U;
rw_reg32_t const ETHERNET_MAC_MACL3A10R                          = (rw_reg32_t)0x40028914U;
rw_reg32_t const ETHERNET_MAC_MACL3A20                           = (rw_reg32_t)0x40028918U;
rw_reg32_t const ETHERNET_MAC_MACL3A30                           = (rw_reg32_t)0x4002891CU;
rw_reg32_t const ETHERNET_MAC_MACL3L4C1R                         = (rw_reg32_t)0x40028930U;
rw_reg32_t const ETHERNET_MAC_MACL4A1R                           = (rw_reg32_t)0x40028934U;
rw_reg32_t const ETHERNET_MAC_MACL3A01R                          = (rw_reg32_t)0x40028940U;
rw_reg32_t const ETHERNET_MAC_MACL3A11R                          = (rw_reg32_t)0x40028944U;
rw_reg32_t const ETHERNET_MAC_MACL3A21R                          = (rw_reg32_t)0x40028948U;
rw_reg32_t const ETHERNET_MAC_MACL3A31R                          = (rw_reg32_t)0x4002894CU;
rw_reg32_t const ETHERNET_MAC_MACTSCR                            = (rw_reg32_t)0x40028B00U;
rw_reg32_t const ETHERNET_MAC_MACSSIR                            = (rw_reg32_t)0x40028B04U;
ro_reg32_t const ETHERNET_MAC_MACSTSR                            = (ro_reg32_t)0x40028B08U;
ro_reg32_t const ETHERNET_MAC_MACSTNR                            = (ro_reg32_t)0x40028B0CU;
rw_reg32_t const ETHERNET_MAC_MACSTSUR                           = (rw_reg32_t)0x40028B10U;
rw_reg32_t const ETHERNET_MAC_MACSTNUR                           = (rw_reg32_t)0x40028B14U;
rw_reg32_t const ETHERNET_MAC_MACTSAR                            = (rw_reg32_t)0x40028B18U;
ro_reg32_t const ETHERNET_MAC_MACTSSR                            = (ro_reg32_t)0x40028B20U;
ro_reg32_t const ETHERNET_MAC_MACTXTSSNR                         = (ro_reg32_t)0x40028B30U;
ro_reg32_t const ETHERNET_MAC_MACTXTSSSR                         = (ro_reg32_t)0x40028B34U;
rw_reg32_t const ETHERNET_MAC_MACACR                             = (rw_reg32_t)0x40028B40U;
ro_reg32_t const ETHERNET_MAC_MACATSNR                           = (ro_reg32_t)0x40028B48U;
ro_reg32_t const ETHERNET_MAC_MACATSSR                           = (ro_reg32_t)0x40028B4CU;
rw_reg32_t const ETHERNET_MAC_MACTSIACR                          = (rw_reg32_t)0x40028B50U;
rw_reg32_t const ETHERNET_MAC_MACTSEACR                          = (rw_reg32_t)0x40028B54U;
rw_reg32_t const ETHERNET_MAC_MACTSICNR                          = (rw_reg32_t)0x40028B58U;
rw_reg32_t const ETHERNET_MAC_MACTSECNR                          = (rw_reg32_t)0x40028B5CU;
rw_reg32_t const ETHERNET_MAC_MACPPSCR                           = (rw_reg32_t)0x40028B70U;
rw_reg32_t const ETHERNET_MAC_MACPPSTTSR                         = (rw_reg32_t)0x40028B80U;
rw_reg32_t const ETHERNET_MAC_MACPPSTTNR                         = (rw_reg32_t)0x40028B84U;
rw_reg32_t const ETHERNET_MAC_MACPPSIR                           = (rw_reg32_t)0x40028B88U;
rw_reg32_t const ETHERNET_MAC_MACPPSWR                           = (rw_reg32_t)0x40028B8CU;
rw_reg32_t const ETHERNET_MAC_MACPOCR                            = (rw_reg32_t)0x40028BC0U;
rw_reg32_t const ETHERNET_MAC_MACSPI0R                           = (rw_reg32_t)0x40028BC4U;
rw_reg32_t const ETHERNET_MAC_MACSPI1R                           = (rw_reg32_t)0x40028BC8U;
rw_reg32_t const ETHERNET_MAC_MACSPI2R                           = (rw_reg32_t)0x40028BCCU;
rw_reg32_t const ETHERNET_MAC_MACLMIR                            = (rw_reg32_t)0x40028BD0U;

/** @subsection Enumerated ETHERNET_MAC Register Definitions */

rw_reg32_t const ETHERNET_MAC_MACAxLR[4] = {
  [0] = (rw_reg32_t)0x40028304U,
  [1] = (rw_reg32_t)0x4002830CU,
  [2] = (rw_reg32_t)0x40028314U,
  [3] = (rw_reg32_t)0x4002831CU,
};

/** @subsection ETHERNET_MAC Register Field Definitions */

const field32_t ETHERNET_MAC_DMAMR_SWR                         = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMAMR_DA                          = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_DMAMR_TXPR                        = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_DMAMR_PR                          = {.msk = 0x00007000U, .pos = 12};
const field32_t ETHERNET_MAC_DMAMR_INTM                        = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_DMASBMR_FB                        = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMASBMR_AAL                       = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_DMASBMR_MB                        = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_DMASBMR_RB                        = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_DMAISR_DC0IS                      = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMAISR_MTLIS                      = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_DMAISR_MACIS                      = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_DMADSR_AXWHSTS                    = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMADSR_RPS0                       = {.msk = 0x00000F00U, .pos = 8};
const field32_t ETHERNET_MAC_DMADSR_TPS0                       = {.msk = 0x0000F000U, .pos = 12};
const field32_t ETHERNET_MAC_DMACCR_MSS                        = {.msk = 0x00003FFFU, .pos = 0};
const field32_t ETHERNET_MAC_DMACCR_PBLX8                      = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_DMACCR_DSL                        = {.msk = 0x001C0000U, .pos = 18};
const field32_t ETHERNET_MAC_DMACTXCR_ST                       = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMACTXCR_OSF                      = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_DMACTXCR_TSE                      = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_DMACTXCR_TXPBL                    = {.msk = 0x003F0000U, .pos = 16};
const field32_t ETHERNET_MAC_DMACRXCR_SR                       = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMACRXCR_RBSZ                     = {.msk = 0x00007FFEU, .pos = 1};
const field32_t ETHERNET_MAC_DMACRXCR_RXPBL                    = {.msk = 0x003F0000U, .pos = 16};
const field32_t ETHERNET_MAC_DMACRXCR_RPF                      = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_DMACTXDLAR_TDESLA                 = {.msk = 0xFFFFFFFCU, .pos = 2};
const field32_t ETHERNET_MAC_DMACRXDLAR_RDESLA                 = {.msk = 0xFFFFFFFCU, .pos = 2};
const field32_t ETHERNET_MAC_DMACTXDTPR_TDT                    = {.msk = 0xFFFFFFFCU, .pos = 2};
const field32_t ETHERNET_MAC_DMACRXDTPR_RDT                    = {.msk = 0xFFFFFFFCU, .pos = 2};
const field32_t ETHERNET_MAC_DMACTXRLR_TDRL                    = {.msk = 0x000003FFU, .pos = 0};
const field32_t ETHERNET_MAC_DMACRXRLR_RDRL                    = {.msk = 0x000003FFU, .pos = 0};
const field32_t ETHERNET_MAC_DMACIER_TIE                       = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMACIER_TXSE                      = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_DMACIER_TBUE                      = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_DMACIER_RIE                       = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_DMACIER_RBUE                      = {.msk = 0x00000080U, .pos = 7};
const field32_t ETHERNET_MAC_DMACIER_RSE                       = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_DMACIER_RWTE                      = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_DMACIER_ETIE                      = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_DMACIER_ERIE                      = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_DMACIER_FBEE                      = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_DMACIER_CDEE                      = {.msk = 0x00002000U, .pos = 13};
const field32_t ETHERNET_MAC_DMACIER_AIE                       = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_DMACIER_NIE                       = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_DMACRXIWTR_RWT                    = {.msk = 0x000000FFU, .pos = 0};
const field32_t ETHERNET_MAC_DMACSR_TI                         = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_DMACSR_TPS                        = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_DMACSR_TBU                        = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_DMACSR_RI                         = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_DMACSR_RBU                        = {.msk = 0x00000080U, .pos = 7};
const field32_t ETHERNET_MAC_DMACSR_RPS                        = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_DMACSR_RWT                        = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_DMACSR_ET                         = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_DMACSR_ER                         = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_DMACSR_FBE                        = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_DMACSR_CDE                        = {.msk = 0x00002000U, .pos = 13};
const field32_t ETHERNET_MAC_DMACSR_AIS                        = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_DMACSR_NIS                        = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_DMACSR_TEB                        = {.msk = 0x00070000U, .pos = 16};
const field32_t ETHERNET_MAC_DMACSR_REB                        = {.msk = 0x00380000U, .pos = 19};
const field32_t ETHERNET_MAC_DMACMFCR_MFC                      = {.msk = 0x000007FFU, .pos = 0};
const field32_t ETHERNET_MAC_DMACMFCR_MFCO                     = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_MTLOMR_DTXSTS                     = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MTLOMR_CNTPRST                    = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MTLOMR_CNTCLR                     = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MTLISR_Q0IS                       = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MTLTXQOMR_FTQ                     = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MTLTXQOMR_TSF                     = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MTLTXQOMR_TXQEN                   = {.msk = 0x0000000CU, .pos = 2};
const field32_t ETHERNET_MAC_MTLTXQOMR_TTC                     = {.msk = 0x00000070U, .pos = 4};
const field32_t ETHERNET_MAC_MTLTXQOMR_TQS                     = {.msk = 0x01FF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MTLTXQUR_UFFRMCNT                 = {.msk = 0x000007FFU, .pos = 0};
const field32_t ETHERNET_MAC_MTLTXQUR_UFCNTOVF                 = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_MTLTXQDR_TXQPAUSED                = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MTLTXQDR_TRCSTS                   = {.msk = 0x00000006U, .pos = 1};
const field32_t ETHERNET_MAC_MTLTXQDR_TWCSTS                   = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MTLTXQDR_TXQSTS                   = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MTLTXQDR_TXSTSFSTS                = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MTLTXQDR_PTXQ                     = {.msk = 0x00070000U, .pos = 16};
const field32_t ETHERNET_MAC_MTLTXQDR_STXSTSF                  = {.msk = 0x00700000U, .pos = 20};
const field32_t ETHERNET_MAC_MTLQICSR_TXUNFIS                  = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MTLQICSR_TXUIE                    = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MTLQICSR_RXOVFIS                  = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MTLQICSR_RXOIE                    = {.msk = 0x01000000U, .pos = 24};
const field32_t ETHERNET_MAC_MTLRXQOMR_RTC                     = {.msk = 0x00000003U, .pos = 0};
const field32_t ETHERNET_MAC_MTLRXQOMR_FUP                     = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MTLRXQOMR_FEP                     = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MTLRXQOMR_RSF                     = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MTLRXQOMR_DIS_TCP_EF              = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_MTLRXQOMR_EHFC                    = {.msk = 0x00000080U, .pos = 7};
const field32_t ETHERNET_MAC_MTLRXQOMR_RFA                     = {.msk = 0x00000700U, .pos = 8};
const field32_t ETHERNET_MAC_MTLRXQOMR_RFD                     = {.msk = 0x0001C000U, .pos = 14};
const field32_t ETHERNET_MAC_MTLRXQOMR_RQS                     = {.msk = 0x00700000U, .pos = 20};
const field32_t ETHERNET_MAC_MTLRXQMPOCR_OVFPKTCNT             = {.msk = 0x000007FFU, .pos = 0};
const field32_t ETHERNET_MAC_MTLRXQMPOCR_OVFCNTOVF             = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_MTLRXQMPOCR_MISPKTCNT             = {.msk = 0x07FF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MTLRXQMPOCR_MISCNTOVF             = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MTLRXQDR_RWCSTS                   = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MTLRXQDR_RRCSTS                   = {.msk = 0x00000006U, .pos = 1};
const field32_t ETHERNET_MAC_MTLRXQDR_RXQSTS                   = {.msk = 0x00000030U, .pos = 4};
const field32_t ETHERNET_MAC_MTLRXQDR_PRXQ                     = {.msk = 0x3FFF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACCR_RE                          = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACCR_TE                          = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACCR_PRELEN                      = {.msk = 0x0000000CU, .pos = 2};
const field32_t ETHERNET_MAC_MACCR_DC                          = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACCR_BL                          = {.msk = 0x00000060U, .pos = 5};
const field32_t ETHERNET_MAC_MACCR_DR                          = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACCR_DCRS                        = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MACCR_DO                          = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_MACCR_ECRSFD                      = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_MACCR_LM                          = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_MACCR_DM                          = {.msk = 0x00002000U, .pos = 13};
const field32_t ETHERNET_MAC_MACCR_FES                         = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_MACCR_JE                          = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACCR_JD                          = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MACCR_WD                          = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACCR_ACS                         = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACCR_CST                         = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MACCR_S2KP                        = {.msk = 0x00400000U, .pos = 22};
const field32_t ETHERNET_MAC_MACCR_GPSLCE                      = {.msk = 0x00800000U, .pos = 23};
const field32_t ETHERNET_MAC_MACCR_IPG                         = {.msk = 0x07000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACCR_IPC                         = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MACCR_SARC                        = {.msk = 0x70000000U, .pos = 28};
const field32_t ETHERNET_MAC_MACCR_ARPEN                       = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACECR_GPSL                       = {.msk = 0x00003FFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACECR_DCRCC                      = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACECR_SPEN                       = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MACECR_USP                        = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACECR_EIPGEN                     = {.msk = 0x01000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACECR_EIPG                       = {.msk = 0x3E000000U, .pos = 25};
const field32_t ETHERNET_MAC_MACPFR_PR                         = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACPFR_HUC                        = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACPFR_HMC                        = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACPFR_DAIF                       = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACPFR_PM                         = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACPFR_DBF                        = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACPFR_PCF                        = {.msk = 0x000000C0U, .pos = 6};
const field32_t ETHERNET_MAC_MACPFR_SAIF                       = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACPFR_SAF                        = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MACPFR_HPF                        = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_MACPFR_VTFE                       = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACPFR_IPFE                       = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACPFR_DNTU                       = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MACPFR_RA                         = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACWTR_WTO                        = {.msk = 0x0000000FU, .pos = 0};
const field32_t ETHERNET_MAC_MACWTR_PWE                        = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACVTR_VL                         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACVTR_ETV                        = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACVTR_VTIM                       = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MACVTR_ESVL                       = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACVTR_ERSVLM                     = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACVTR_DOVLTC                     = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACVTR_EVLS                       = {.msk = 0x00600000U, .pos = 21};
const field32_t ETHERNET_MAC_MACVTR_EVLRXS                     = {.msk = 0x01000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACVTR_VTHM                       = {.msk = 0x02000000U, .pos = 25};
const field32_t ETHERNET_MAC_MACVTR_EDVLP                      = {.msk = 0x04000000U, .pos = 26};
const field32_t ETHERNET_MAC_MACVTR_ERIVLT                     = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MACVTR_EIVLS                      = {.msk = 0x30000000U, .pos = 28};
const field32_t ETHERNET_MAC_MACVTR_EIVLRXS                    = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACVHTR_VLHT                      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACVIR_VLT                        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACVIR_VLC                        = {.msk = 0x00030000U, .pos = 16};
const field32_t ETHERNET_MAC_MACVIR_VLP                        = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACVIR_CSVL                       = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACVIR_VLTI                       = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACIVIR_VLT                       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACIVIR_VLC                       = {.msk = 0x00030000U, .pos = 16};
const field32_t ETHERNET_MAC_MACIVIR_VLP                       = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACIVIR_CSVL                      = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACIVIR_VLTI                      = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACQTXFCR_FCB_BPA                 = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACQTXFCR_TFE                     = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACQTXFCR_PLT                     = {.msk = 0x00000070U, .pos = 4};
const field32_t ETHERNET_MAC_MACQTXFCR_DZPQ                    = {.msk = 0x00000080U, .pos = 7};
const field32_t ETHERNET_MAC_MACQTXFCR_PT                      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACRXFCR_RFE                      = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACRXFCR_UP                       = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACISR_PHYIS                      = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACISR_PMTIS                      = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACISR_LPIIS                      = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACISR_MMCIS                      = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACISR_MMCRXIS                    = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MACISR_MMCTXIS                    = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_MACISR_TSIS                       = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_MACISR_TXSTSIS                    = {.msk = 0x00002000U, .pos = 13};
const field32_t ETHERNET_MAC_MACISR_RXSTSIS                    = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_MACIER_PHYIE                      = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACIER_PMTIE                      = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACIER_LPIIE                      = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACIER_TSIE                       = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_MACIER_TXSTSIE                    = {.msk = 0x00002000U, .pos = 13};
const field32_t ETHERNET_MAC_MACIER_RXSTSIE                    = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_MACRXTXSR_TJT                     = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACRXTXSR_NCARR                   = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACRXTXSR_LCARR                   = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACRXTXSR_EXDEF                   = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACRXTXSR_LCOL                    = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACRXTXSR_EXCOL                   = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACRXTXSR_RWT                     = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACPCSR_PWRDWN                    = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACPCSR_MGKPKTEN                  = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACPCSR_RWKPKTEN                  = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACPCSR_MGKPRCVD                  = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACPCSR_RWKPRCVD                  = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_MACPCSR_GLBLUCAST                 = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MACPCSR_RWKPFE                    = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_MACPCSR_RWKPTR                    = {.msk = 0x1F000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACPCSR_RWKFILTRST                = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACLCSR_TLPIEN                    = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACLCSR_TLPIEX                    = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACLCSR_RLPIEN                    = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACLCSR_RLPIEX                    = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACLCSR_TLPIST                    = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACLCSR_RLPIST                    = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MACLCSR_LPIEN                     = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACLCSR_PLS                       = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MACLCSR_PLSEN                     = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACLCSR_LPITXA                    = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACLCSR_LPITE                     = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACLCSR_LPITCSE                   = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MACLTCR_TWT                       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACLTCR_LST                       = {.msk = 0x03FF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACLETR_LPIET                     = {.msk = 0x0001FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MAC1USTCR_TIC_1US_CNTR            = {.msk = 0x00000FFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACVR_SNPSVER                     = {.msk = 0x000000FFU, .pos = 0};
const field32_t ETHERNET_MAC_MACVR_USERVER                     = {.msk = 0x0000FF00U, .pos = 8};
const field32_t ETHERNET_MAC_MACHWF1R_RXFIFOSIZE               = {.msk = 0x0000001FU, .pos = 0};
const field32_t ETHERNET_MAC_MACHWF1R_TXFIFOSIZE               = {.msk = 0x000007C0U, .pos = 6};
const field32_t ETHERNET_MAC_MACHWF1R_OSTEN                    = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_MACHWF1R_PTOEN                    = {.msk = 0x00001000U, .pos = 12};
const field32_t ETHERNET_MAC_MACHWF1R_ADVTHWORD                = {.msk = 0x00002000U, .pos = 13};
const field32_t ETHERNET_MAC_MACHWF1R_DCBEN                    = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACHWF1R_SPHEN                    = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MACHWF1R_TSOEN                    = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACHWF1R_DBGMEMA                  = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACHWF1R_AVSEL                    = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACHWF1R_HASHTBLSZ                = {.msk = 0x03000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACHWF1R_L3L4FNUM                 = {.msk = 0x78000000U, .pos = 27};
const field32_t ETHERNET_MAC_MACHWF2R_RXQCNT                   = {.msk = 0x0000000FU, .pos = 0};
const field32_t ETHERNET_MAC_MACHWF2R_TXQCNT                   = {.msk = 0x000003C0U, .pos = 6};
const field32_t ETHERNET_MAC_MACHWF2R_RXCHCNT                  = {.msk = 0x0000F000U, .pos = 12};
const field32_t ETHERNET_MAC_MACHWF2R_TXCHCNT                  = {.msk = 0x003C0000U, .pos = 18};
const field32_t ETHERNET_MAC_MACHWF2R_PPSOUTNUM                = {.msk = 0x07000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACHWF2R_AUXSNAPNUM               = {.msk = 0x70000000U, .pos = 28};
const field32_t ETHERNET_MAC_MACMDIOAR_MB                      = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACMDIOAR_C45E                    = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACMDIOAR_GOC                     = {.msk = 0x0000000CU, .pos = 2};
const field32_t ETHERNET_MAC_MACMDIOAR_SKAP                    = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACMDIOAR_CR                      = {.msk = 0x00000F00U, .pos = 8};
const field32_t ETHERNET_MAC_MACMDIOAR_NTC                     = {.msk = 0x00007000U, .pos = 12};
const field32_t ETHERNET_MAC_MACMDIOAR_RDA                     = {.msk = 0x001F0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACMDIOAR_PA                      = {.msk = 0x03E00000U, .pos = 21};
const field32_t ETHERNET_MAC_MACMDIOAR_BTB                     = {.msk = 0x04000000U, .pos = 26};
const field32_t ETHERNET_MAC_MACMDIOAR_PSE                     = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MACMDIODR_MD                      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACMDIODR_RA                      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACA0HR_ADDRHI                    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACA0HR_AE                        = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACA1HR_ADDRHI                    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACA1HR_MBC                       = {.msk = 0x3F000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACA1HR_SA                        = {.msk = 0x40000000U, .pos = 30};
const field32_t ETHERNET_MAC_MACA1HR_AE                        = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACA2HR_ADDRHI                    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACA2HR_MBC                       = {.msk = 0x3F000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACA2HR_SA                        = {.msk = 0x40000000U, .pos = 30};
const field32_t ETHERNET_MAC_MACA2HR_AE                        = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACA3HR_ADDRHI                    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACA3HR_MBC                       = {.msk = 0x3F000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACA3HR_SA                        = {.msk = 0x40000000U, .pos = 30};
const field32_t ETHERNET_MAC_MACA3HR_AE                        = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MMC_CONTROL_CNTRST                = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MMC_CONTROL_CNTSTOPRO             = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MMC_CONTROL_RSTONRD               = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MMC_CONTROL_CNTFREEZ              = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MMC_CONTROL_CNTPRST               = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MMC_CONTROL_CNTPRSTLVL            = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MMC_CONTROL_UCDBC                 = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXCRCERPIS       = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXALGNERPIS      = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXUCGPIS         = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXLPIUSCIS       = {.msk = 0x04000000U, .pos = 26};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_RXLPITRCIS       = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXSCOLGPIS       = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXMCOLGPIS       = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXGPKTIS         = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXLPIUSCIS       = {.msk = 0x04000000U, .pos = 26};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_TXLPITRCIS       = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXCRCERPIM  = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXALGNERPIM = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXUCGPIM    = {.msk = 0x00020000U, .pos = 17};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXLPIUSCIM  = {.msk = 0x04000000U, .pos = 26};
const field32_t ETHERNET_MAC_MMC_RX_INTERRUPT_MASK_RXLPITRCIM  = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXSCOLGPIM  = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXMCOLGPIM  = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXGPKTIM    = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXLPIUSCIM  = {.msk = 0x04000000U, .pos = 26};
const field32_t ETHERNET_MAC_MMC_TX_INTERRUPT_MASK_TXLPITRCIM  = {.msk = 0x08000000U, .pos = 27};
const field32_t ETHERNET_MAC_MACL3L4C0R_L3SAM0                 = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACL3L4C0R_L3SAIM0                = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACL3L4C0R_L3DAM0                 = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACL3L4C0R_L3DAIM0                = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACL3L4C0R_L3HSBM0                = {.msk = 0x000007C0U, .pos = 6};
const field32_t ETHERNET_MAC_MACL3L4C0R_L3HDBM0                = {.msk = 0x0000F800U, .pos = 11};
const field32_t ETHERNET_MAC_MACL3L4C0R_L4SPM0                 = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACL3L4C0R_L4SPIM0                = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACL3L4C0R_L4DPM0                 = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACL3L4C0R_L4DPIM0                = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MACL4A0R_L4SP0                    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACL4A0R_L4DP0                    = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACDR_RPESTS                      = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACDR_RFCFCSTS                    = {.msk = 0x00000006U, .pos = 1};
const field32_t ETHERNET_MAC_MACDR_TPESTS                      = {.msk = 0x00010000U, .pos = 16};
const field32_t ETHERNET_MAC_MACDR_TFCSTS                      = {.msk = 0x00060000U, .pos = 17};
const field32_t ETHERNET_MAC_MACL3L4C1R_L3SAM1                 = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACL3L4C1R_L3SAIM1                = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACL3L4C1R_L3DAM1                 = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACL3L4C1R_L3DAIM1                = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACL3L4C1R_L3HSBM1                = {.msk = 0x000007C0U, .pos = 6};
const field32_t ETHERNET_MAC_MACL3L4C1R_L3HDBM1                = {.msk = 0x0000F800U, .pos = 11};
const field32_t ETHERNET_MAC_MACL3L4C1R_L4SPM1                 = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACL3L4C1R_L4SPIM1                = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACL3L4C1R_L4DPM1                 = {.msk = 0x00100000U, .pos = 20};
const field32_t ETHERNET_MAC_MACL3L4C1R_L4DPIM1                = {.msk = 0x00200000U, .pos = 21};
const field32_t ETHERNET_MAC_MACL4A1R_L4SP1                    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACL4A1R_L4DP1                    = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACTSCR_TSENA                     = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACTSCR_TSCFUPDT                  = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACTSCR_TSINIT                    = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACTSCR_TSUPDT                    = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACTSCR_TSADDREG                  = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACTSCR_TSENALL                   = {.msk = 0x00000100U, .pos = 8};
const field32_t ETHERNET_MAC_MACTSCR_TSCTRLSSR                 = {.msk = 0x00000200U, .pos = 9};
const field32_t ETHERNET_MAC_MACTSCR_TSVER2ENA                 = {.msk = 0x00000400U, .pos = 10};
const field32_t ETHERNET_MAC_MACTSCR_TSIPENA                   = {.msk = 0x00000800U, .pos = 11};
const field32_t ETHERNET_MAC_MACTSCR_TSEVNTENA                 = {.msk = 0x00004000U, .pos = 14};
const field32_t ETHERNET_MAC_MACTSCR_TSMSTRENA                 = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_MACTSCR_SNAPTYPSEL                = {.msk = 0x00030000U, .pos = 16};
const field32_t ETHERNET_MAC_MACTSCR_TSENMACADDR               = {.msk = 0x00040000U, .pos = 18};
const field32_t ETHERNET_MAC_MACTSCR_CSC                       = {.msk = 0x00080000U, .pos = 19};
const field32_t ETHERNET_MAC_MACTSCR_TXTSSTSM                  = {.msk = 0x01000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACSSIR_SNSINC                    = {.msk = 0x0000FF00U, .pos = 8};
const field32_t ETHERNET_MAC_MACSSIR_SSINC                     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACSTNR_TSSS                      = {.msk = 0x7FFFFFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACSTNUR_TSSS                     = {.msk = 0x7FFFFFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACSTNUR_ADDSUB                   = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACTSSR_TSSOVF                    = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACTSSR_TSTARGT0                  = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACTSSR_AUXTSTRIG                 = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACTSSR_TSTRGTERR0                = {.msk = 0x00000008U, .pos = 3};
const field32_t ETHERNET_MAC_MACTSSR_TXTSSIS                   = {.msk = 0x00008000U, .pos = 15};
const field32_t ETHERNET_MAC_MACTSSR_ATSSTN                    = {.msk = 0x000F0000U, .pos = 16};
const field32_t ETHERNET_MAC_MACTSSR_ATSSTM                    = {.msk = 0x01000000U, .pos = 24};
const field32_t ETHERNET_MAC_MACTSSR_ATSNS                     = {.msk = 0x3E000000U, .pos = 25};
const field32_t ETHERNET_MAC_MACTXTSSNR_TXTSSLO                = {.msk = 0x7FFFFFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACTXTSSNR_TXTSSMIS               = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACACR_ATSFC                      = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACATSNR_AUXTSLO                  = {.msk = 0x7FFFFFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACPPSCR_PPSCTRL                  = {.msk = 0x0000000FU, .pos = 0};
const field32_t ETHERNET_MAC_MACPPSCR_PPSEN0                   = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACPPSCR_TRGTMODSEL0              = {.msk = 0x00000060U, .pos = 5};
const field32_t ETHERNET_MAC_MACPPSTTSR_TSTRH0                 = {.msk = 0x7FFFFFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACPPSTTNR_TTSL0                  = {.msk = 0x7FFFFFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACPPSTTNR_TRGTBUSY0              = {.msk = 0x80000000U, .pos = 31};
const field32_t ETHERNET_MAC_MACPOCR_PTOEN                     = {.msk = 0x00000001U, .pos = 0};
const field32_t ETHERNET_MAC_MACPOCR_ASYNCEN                   = {.msk = 0x00000002U, .pos = 1};
const field32_t ETHERNET_MAC_MACPOCR_APDREQEN                  = {.msk = 0x00000004U, .pos = 2};
const field32_t ETHERNET_MAC_MACPOCR_ASYNCTRIG                 = {.msk = 0x00000010U, .pos = 4};
const field32_t ETHERNET_MAC_MACPOCR_APDREQTRIG                = {.msk = 0x00000020U, .pos = 5};
const field32_t ETHERNET_MAC_MACPOCR_DRRDIS                    = {.msk = 0x00000040U, .pos = 6};
const field32_t ETHERNET_MAC_MACPOCR_DN                        = {.msk = 0x0000FF00U, .pos = 8};
const field32_t ETHERNET_MAC_MACSPI2R_SPI2                     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t ETHERNET_MAC_MACLMIR_LSI                       = {.msk = 0x000000FFU, .pos = 0};
const field32_t ETHERNET_MAC_MACLMIR_DRSYNCR                   = {.msk = 0x00000700U, .pos = 8};
const field32_t ETHERNET_MAC_MACLMIR_LMPDRI                    = {.msk = 0xFF000000U, .pos = 24};

/** @subsection Enumerated ETHERNET_MAC Register Field Definitions */

field32_t const ETHERNET_MAC_MACL3L4C0R_LxPEN0[5] = {
  [3] = {.msk = 0x00000001U, .pos = 0},
  [4] = {.msk = 0x00010000U, .pos = 16},
};

field32_t const ETHERNET_MAC_MACL3L4C1R_LxPEN1[5] = {
  [3] = {.msk = 0x00000001U, .pos = 0},
  [4] = {.msk = 0x00010000U, .pos = 16},
};

field32_t const ETHERNET_MAC_MACTSCR_TSIPVxENA[7] = {
  [4] = {.msk = 0x00002000U, .pos = 13},
  [6] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const ETHERNET_MAC_MACACR_ATSENx[4] = {
  [0] = {.msk = 0x00000010U, .pos = 4},
  [1] = {.msk = 0x00000020U, .pos = 5},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00000080U, .pos = 7},
};

/**************************************************************************************************
 * @section DMAx Definitions
 **************************************************************************************************/

/** @subsection Enumerated DMAx Register Definitions */

ro_reg32_t const DMAx_LISR[3] = {
  [1] = (ro_reg32_t)0x40020000U,
  [2] = (ro_reg32_t)0x40020400U,
};

ro_reg32_t const DMAx_HISR[3] = {
  [1] = (ro_reg32_t)0x40020004U,
  [2] = (ro_reg32_t)0x40020404U,
};

rw_reg32_t const DMAx_LIFCR[3] = {
  [1] = (rw_reg32_t)0x40020008U,
  [2] = (rw_reg32_t)0x40020408U,
};

rw_reg32_t const DMAx_HIFCR[3] = {
  [1] = (rw_reg32_t)0x4002000CU,
  [2] = (rw_reg32_t)0x4002040CU,
};

rw_reg32_t const DMAx_S0CR[3] = {
  [1] = (rw_reg32_t)0x40020010U,
  [2] = (rw_reg32_t)0x40020410U,
};

rw_reg32_t const DMAx_SxNDTR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40020014U,
    [1] = (rw_reg32_t)0x4002002CU,
    [2] = (rw_reg32_t)0x40020044U,
    [3] = (rw_reg32_t)0x4002005CU,
    [4] = (rw_reg32_t)0x40020074U,
    [5] = (rw_reg32_t)0x4002008CU,
    [6] = (rw_reg32_t)0x400200A4U,
    [7] = (rw_reg32_t)0x400200BCU,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40020414U,
    [1] = (rw_reg32_t)0x4002042CU,
    [2] = (rw_reg32_t)0x40020444U,
    [3] = (rw_reg32_t)0x4002045CU,
    [4] = (rw_reg32_t)0x40020474U,
    [5] = (rw_reg32_t)0x4002048CU,
    [6] = (rw_reg32_t)0x400204A4U,
    [7] = (rw_reg32_t)0x400204BCU,
  },
};

rw_reg32_t const DMAx_SxPAR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40020018U,
    [1] = (rw_reg32_t)0x40020030U,
    [2] = (rw_reg32_t)0x40020048U,
    [3] = (rw_reg32_t)0x40020060U,
    [4] = (rw_reg32_t)0x40020078U,
    [5] = (rw_reg32_t)0x40020090U,
    [6] = (rw_reg32_t)0x400200A8U,
    [7] = (rw_reg32_t)0x400200C0U,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40020418U,
    [1] = (rw_reg32_t)0x40020430U,
    [2] = (rw_reg32_t)0x40020448U,
    [3] = (rw_reg32_t)0x40020460U,
    [4] = (rw_reg32_t)0x40020478U,
    [5] = (rw_reg32_t)0x40020490U,
    [6] = (rw_reg32_t)0x400204A8U,
    [7] = (rw_reg32_t)0x400204C0U,
  },
};

rw_reg32_t const DMAx_SxM0AR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x4002001CU,
    [1] = (rw_reg32_t)0x40020034U,
    [2] = (rw_reg32_t)0x4002004CU,
    [3] = (rw_reg32_t)0x40020064U,
    [4] = (rw_reg32_t)0x4002007CU,
    [5] = (rw_reg32_t)0x40020094U,
    [6] = (rw_reg32_t)0x400200ACU,
    [7] = (rw_reg32_t)0x400200C4U,
  },
  [2] = {
    [0] = (rw_reg32_t)0x4002041CU,
    [1] = (rw_reg32_t)0x40020434U,
    [2] = (rw_reg32_t)0x4002044CU,
    [3] = (rw_reg32_t)0x40020464U,
    [4] = (rw_reg32_t)0x4002047CU,
    [5] = (rw_reg32_t)0x40020494U,
    [6] = (rw_reg32_t)0x400204ACU,
    [7] = (rw_reg32_t)0x400204C4U,
  },
};

rw_reg32_t const DMAx_SxM1AR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40020020U,
    [1] = (rw_reg32_t)0x40020038U,
    [2] = (rw_reg32_t)0x40020050U,
    [3] = (rw_reg32_t)0x40020068U,
    [4] = (rw_reg32_t)0x40020080U,
    [5] = (rw_reg32_t)0x40020098U,
    [6] = (rw_reg32_t)0x400200B0U,
    [7] = (rw_reg32_t)0x400200C8U,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40020420U,
    [1] = (rw_reg32_t)0x40020438U,
    [2] = (rw_reg32_t)0x40020450U,
    [3] = (rw_reg32_t)0x40020468U,
    [4] = (rw_reg32_t)0x40020480U,
    [5] = (rw_reg32_t)0x40020498U,
    [6] = (rw_reg32_t)0x400204B0U,
    [7] = (rw_reg32_t)0x400204C8U,
  },
};

rw_reg32_t const DMAx_SxFCR[3][8] = {
  [1] = {
    [0] = (rw_reg32_t)0x40020024U,
    [1] = (rw_reg32_t)0x4002003CU,
    [2] = (rw_reg32_t)0x40020054U,
    [3] = (rw_reg32_t)0x4002006CU,
    [4] = (rw_reg32_t)0x40020084U,
    [5] = (rw_reg32_t)0x4002009CU,
    [6] = (rw_reg32_t)0x400200B4U,
    [7] = (rw_reg32_t)0x400200CCU,
  },
  [2] = {
    [0] = (rw_reg32_t)0x40020424U,
    [1] = (rw_reg32_t)0x4002043CU,
    [2] = (rw_reg32_t)0x40020454U,
    [3] = (rw_reg32_t)0x4002046CU,
    [4] = (rw_reg32_t)0x40020484U,
    [5] = (rw_reg32_t)0x4002049CU,
    [6] = (rw_reg32_t)0x400204B4U,
    [7] = (rw_reg32_t)0x400204CCU,
  },
};

rw_reg32_t const DMAx_S1CR[3] = {
  [1] = (rw_reg32_t)0x40020028U,
  [2] = (rw_reg32_t)0x40020428U,
};

rw_reg32_t const DMAx_S2CR[3] = {
  [1] = (rw_reg32_t)0x40020040U,
  [2] = (rw_reg32_t)0x40020440U,
};

rw_reg32_t const DMAx_S3CR[3] = {
  [1] = (rw_reg32_t)0x40020058U,
  [2] = (rw_reg32_t)0x40020458U,
};

rw_reg32_t const DMAx_S4CR[3] = {
  [1] = (rw_reg32_t)0x40020070U,
  [2] = (rw_reg32_t)0x40020470U,
};

rw_reg32_t const DMAx_S5CR[3] = {
  [1] = (rw_reg32_t)0x40020088U,
  [2] = (rw_reg32_t)0x40020488U,
};

rw_reg32_t const DMAx_S6CR[3] = {
  [1] = (rw_reg32_t)0x400200A0U,
  [2] = (rw_reg32_t)0x400204A0U,
};

rw_reg32_t const DMAx_S7CR[3] = {
  [1] = (rw_reg32_t)0x400200B8U,
  [2] = (rw_reg32_t)0x400204B8U,
};

/** @subsection DMAx Register Field Definitions */

const field32_t DMAx_S0CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S0CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S0CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S0CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S0CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S0CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S0CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S0CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S0CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S0CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S0CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S0CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S0CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S0CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S0CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S0CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S0CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S0CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_SxNDTR_NDT  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t DMAx_SxFCR_FEIE  = {.msk = 0x00000080U, .pos = 7};
const field32_t DMAx_SxFCR_FS    = {.msk = 0x00000038U, .pos = 3};
const field32_t DMAx_SxFCR_DMDIS = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_SxFCR_FTH   = {.msk = 0x00000003U, .pos = 0};
const field32_t DMAx_S1CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S1CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S1CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S1CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S1CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S1CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S1CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S1CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S1CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S1CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S1CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S1CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S1CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S1CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S1CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S1CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S1CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S1CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S1CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_S2CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S2CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S2CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S2CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S2CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S2CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S2CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S2CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S2CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S2CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S2CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S2CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S2CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S2CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S2CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S2CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S2CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S2CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S2CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_S3CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S3CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S3CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S3CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S3CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S3CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S3CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S3CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S3CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S3CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S3CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S3CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S3CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S3CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S3CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S3CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S3CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S3CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S3CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_S4CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S4CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S4CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S4CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S4CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S4CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S4CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S4CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S4CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S4CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S4CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S4CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S4CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S4CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S4CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S4CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S4CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S4CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S4CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_S5CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S5CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S5CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S5CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S5CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S5CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S5CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S5CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S5CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S5CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S5CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S5CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S5CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S5CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S5CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S5CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S5CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S5CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S5CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_S6CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S6CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S6CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S6CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S6CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S6CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S6CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S6CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S6CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S6CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S6CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S6CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S6CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S6CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S6CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S6CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S6CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S6CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S6CR_EN     = {.msk = 0x00000001U, .pos = 0};
const field32_t DMAx_S7CR_MBURST = {.msk = 0x01800000U, .pos = 23};
const field32_t DMAx_S7CR_PBURST = {.msk = 0x00600000U, .pos = 21};
const field32_t DMAx_S7CR_ACK    = {.msk = 0x00100000U, .pos = 20};
const field32_t DMAx_S7CR_CT     = {.msk = 0x00080000U, .pos = 19};
const field32_t DMAx_S7CR_DBM    = {.msk = 0x00040000U, .pos = 18};
const field32_t DMAx_S7CR_PL     = {.msk = 0x00030000U, .pos = 16};
const field32_t DMAx_S7CR_PINCOS = {.msk = 0x00008000U, .pos = 15};
const field32_t DMAx_S7CR_MSIZE  = {.msk = 0x00006000U, .pos = 13};
const field32_t DMAx_S7CR_PSIZE  = {.msk = 0x00001800U, .pos = 11};
const field32_t DMAx_S7CR_MINC   = {.msk = 0x00000400U, .pos = 10};
const field32_t DMAx_S7CR_PINC   = {.msk = 0x00000200U, .pos = 9};
const field32_t DMAx_S7CR_CIRC   = {.msk = 0x00000100U, .pos = 8};
const field32_t DMAx_S7CR_DIR    = {.msk = 0x000000C0U, .pos = 6};
const field32_t DMAx_S7CR_PFCTRL = {.msk = 0x00000020U, .pos = 5};
const field32_t DMAx_S7CR_TCIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t DMAx_S7CR_HTIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t DMAx_S7CR_TEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t DMAx_S7CR_DMEIE  = {.msk = 0x00000002U, .pos = 1};
const field32_t DMAx_S7CR_EN     = {.msk = 0x00000001U, .pos = 0};

/** @subsection Enumerated DMAx Register Field Definitions */

field32_t const DMAx_LISR_TCIFx[4] = {
  [0] = {.msk = 0x00000020U, .pos = 5},
  [1] = {.msk = 0x00000800U, .pos = 11},
  [2] = {.msk = 0x00200000U, .pos = 21},
  [3] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const DMAx_LISR_HTIFx[4] = {
  [0] = {.msk = 0x00000010U, .pos = 4},
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [3] = {.msk = 0x04000000U, .pos = 26},
};

field32_t const DMAx_LISR_TEIFx[4] = {
  [0] = {.msk = 0x00000008U, .pos = 3},
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00080000U, .pos = 19},
  [3] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const DMAx_LISR_DMEIFx[4] = {
  [0] = {.msk = 0x00000004U, .pos = 2},
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00040000U, .pos = 18},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const DMAx_LISR_FEIFx[4] = {
  [0] = {.msk = 0x00000001U, .pos = 0},
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00010000U, .pos = 16},
  [3] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const DMAx_LIFCR_CTCIFx[4] = {
  [0] = {.msk = 0x00000020U, .pos = 5},
  [1] = {.msk = 0x00000800U, .pos = 11},
  [2] = {.msk = 0x00200000U, .pos = 21},
  [3] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const DMAx_LIFCR_CHTIFx[4] = {
  [0] = {.msk = 0x00000010U, .pos = 4},
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [3] = {.msk = 0x04000000U, .pos = 26},
};

field32_t const DMAx_LIFCR_CTEIFx[4] = {
  [0] = {.msk = 0x00000008U, .pos = 3},
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00080000U, .pos = 19},
  [3] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const DMAx_LIFCR_CDMEIFx[4] = {
  [0] = {.msk = 0x00000004U, .pos = 2},
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00040000U, .pos = 18},
  [3] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const DMAx_LIFCR_CFEIFx[4] = {
  [0] = {.msk = 0x00000001U, .pos = 0},
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00010000U, .pos = 16},
  [3] = {.msk = 0x00400000U, .pos = 22},
};

/**************************************************************************************************
 * @section HRTIM_MASTER Definitions
 **************************************************************************************************/

/** @subsection HRTIM_MASTER Register Definitions */

rw_reg32_t const HRTIM_MASTER_MCR    = (rw_reg32_t)0x40017400U;
ro_reg32_t const HRTIM_MASTER_MISR   = (ro_reg32_t)0x40017404U;
rw_reg32_t const HRTIM_MASTER_MICR   = (rw_reg32_t)0x40017408U;
rw_reg32_t const HRTIM_MASTER_MDIER4 = (rw_reg32_t)0x4001740CU;
rw_reg32_t const HRTIM_MASTER_MCNTR  = (rw_reg32_t)0x40017410U;
rw_reg32_t const HRTIM_MASTER_MPER   = (rw_reg32_t)0x40017414U;
rw_reg32_t const HRTIM_MASTER_MREP   = (rw_reg32_t)0x40017418U;
rw_reg32_t const HRTIM_MASTER_MCMP1R = (rw_reg32_t)0x4001741CU;
rw_reg32_t const HRTIM_MASTER_MCMP2R = (rw_reg32_t)0x40017424U;
rw_reg32_t const HRTIM_MASTER_MCMP3R = (rw_reg32_t)0x40017428U;
rw_reg32_t const HRTIM_MASTER_MCMP4R = (rw_reg32_t)0x4001742CU;

/** @subsection HRTIM_MASTER Register Field Definitions */

const field32_t HRTIM_MASTER_MCR_BRSTDMA   = {.msk = 0xC0000000U, .pos = 30};
const field32_t HRTIM_MASTER_MCR_MREPU     = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_MASTER_MCR_PREEN     = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_MASTER_MCR_DACSYNC   = {.msk = 0x06000000U, .pos = 25};
const field32_t HRTIM_MASTER_MCR_TECEN     = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_MASTER_MCR_TDCEN     = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_MASTER_MCR_TCCEN     = {.msk = 0x00080000U, .pos = 19};
const field32_t HRTIM_MASTER_MCR_TBCEN     = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_MASTER_MCR_TACEN     = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_MASTER_MCR_MCEN      = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_MASTER_MCR_SYNC_SRC  = {.msk = 0x0000C000U, .pos = 14};
const field32_t HRTIM_MASTER_MCR_SYNC_OUT  = {.msk = 0x00003000U, .pos = 12};
const field32_t HRTIM_MASTER_MCR_SYNCSTRTM = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_MASTER_MCR_SYNCRSTM  = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_MASTER_MCR_SYNC_IN   = {.msk = 0x00000300U, .pos = 8};
const field32_t HRTIM_MASTER_MCR_HALF      = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_MASTER_MCR_RETRIG    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_MASTER_MCR_CONT      = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_MASTER_MCR_CK_PSC    = {.msk = 0x00000007U, .pos = 0};
const field32_t HRTIM_MASTER_MISR_MUPD     = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_MASTER_MISR_SYNC     = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_MASTER_MISR_MREP     = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_MASTER_MICR_MUPDC    = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_MASTER_MICR_SYNCC    = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_MASTER_MICR_MREPC    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_MASTER_MDIER4_MUPDDE = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_MASTER_MDIER4_SYNCDE = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_MASTER_MDIER4_MREPDE = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_MASTER_MDIER4_MUPDIE = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_MASTER_MDIER4_SYNCIE = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_MASTER_MDIER4_MREPIE = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_MASTER_MCNTR_MCNT    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_MASTER_MPER_MPER     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_MASTER_MREP_MREP     = {.msk = 0x000000FFU, .pos = 0};
const field32_t HRTIM_MASTER_MCMP1R_MCMP1  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_MASTER_MCMP2R_MCMP2  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_MASTER_MCMP3R_MCMP3  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_MASTER_MCMP4R_MCMP4  = {.msk = 0x0000FFFFU, .pos = 0};

/** @subsection Enumerated HRTIM_MASTER Register Field Definitions */

field32_t const HRTIM_MASTER_MISR_MCMPx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_MASTER_MICR_MCMPxC[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_MASTER_MDIER4_MCMPxDE[5] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_MASTER_MDIER4_MCMPxIE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

/**************************************************************************************************
 * @section HRTIM_TIMA Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMA Register Definitions */

rw_reg32_t const HRTIM_TIMA_TIMACR    = (rw_reg32_t)0x40017480U;
ro_reg32_t const HRTIM_TIMA_TIMAISR   = (ro_reg32_t)0x40017484U;
rw_reg32_t const HRTIM_TIMA_TIMAICR   = (rw_reg32_t)0x40017488U;
rw_reg32_t const HRTIM_TIMA_TIMADIER5 = (rw_reg32_t)0x4001748CU;
rw_reg32_t const HRTIM_TIMA_CNTAR     = (rw_reg32_t)0x40017490U;
rw_reg32_t const HRTIM_TIMA_PERAR     = (rw_reg32_t)0x40017494U;
rw_reg32_t const HRTIM_TIMA_REPAR     = (rw_reg32_t)0x40017498U;
rw_reg32_t const HRTIM_TIMA_CMP1AR    = (rw_reg32_t)0x4001749CU;
rw_reg32_t const HRTIM_TIMA_CMP1CAR   = (rw_reg32_t)0x400174A0U;
rw_reg32_t const HRTIM_TIMA_CMP2AR    = (rw_reg32_t)0x400174A4U;
rw_reg32_t const HRTIM_TIMA_CMP3AR    = (rw_reg32_t)0x400174A8U;
rw_reg32_t const HRTIM_TIMA_CMP4AR    = (rw_reg32_t)0x400174ACU;
ro_reg32_t const HRTIM_TIMA_CPT1AR    = (ro_reg32_t)0x400174B0U;
ro_reg32_t const HRTIM_TIMA_CPT2AR    = (ro_reg32_t)0x400174B4U;
rw_reg32_t const HRTIM_TIMA_DTAR      = (rw_reg32_t)0x400174B8U;
rw_reg32_t const HRTIM_TIMA_EEFAR1    = (rw_reg32_t)0x400174CCU;
rw_reg32_t const HRTIM_TIMA_EEFAR2    = (rw_reg32_t)0x400174D0U;
rw_reg32_t const HRTIM_TIMA_RSTAR     = (rw_reg32_t)0x400174D4U;
rw_reg32_t const HRTIM_TIMA_CHPAR     = (rw_reg32_t)0x400174D8U;
rw_reg32_t const HRTIM_TIMA_OUTAR     = (rw_reg32_t)0x400174E4U;
rw_reg32_t const HRTIM_TIMA_FLTAR     = (rw_reg32_t)0x400174E8U;

/** @subsection Enumerated HRTIM_TIMA Register Definitions */

rw_reg32_t const HRTIM_TIMA_SETAxR[3] = {
  [1] = (rw_reg32_t)0x400174BCU,
  [2] = (rw_reg32_t)0x400174C4U,
};

rw_reg32_t const HRTIM_TIMA_RSTAxR[3] = {
  [1] = (rw_reg32_t)0x400174C0U,
  [2] = (rw_reg32_t)0x400174C8U,
};

rw_reg32_t const HRTIM_TIMA_CPTxACR[3] = {
  [1] = (rw_reg32_t)0x400174DCU,
  [2] = (rw_reg32_t)0x400174E0U,
};

/** @subsection HRTIM_TIMA Register Field Definitions */

const field32_t HRTIM_TIMA_TIMACR_UPDGAT      = {.msk = 0xF0000000U, .pos = 28};
const field32_t HRTIM_TIMA_TIMACR_PREEN       = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMA_TIMACR_DACSYNC     = {.msk = 0x06000000U, .pos = 25};
const field32_t HRTIM_TIMA_TIMACR_MSTU        = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMA_TIMACR_TEU         = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_TIMA_TIMACR_TDU         = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMA_TIMACR_TCU         = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMA_TIMACR_TBU         = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMA_TIMACR_TXRSTU      = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_TIMA_TIMACR_TXREPU      = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMA_TIMACR_SYNCSTRTX   = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMA_TIMACR_SYNCRSTX    = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_TIMA_TIMACR_PSHPLL      = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMA_TIMACR_HALF        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_TIMA_TIMACR_RETRIG      = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMA_TIMACR_CONT        = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_TIMA_TIMACR_CK_PSCX     = {.msk = 0x00000007U, .pos = 0};
const field32_t HRTIM_TIMA_TIMAISR_IPPSTAT    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMA_TIMAISR_CPPSTAT    = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMA_TIMAISR_DLYPRT     = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMA_TIMAISR_RST        = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMA_TIMAISR_UPD        = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMA_TIMAISR_REP        = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMA_TIMAICR_DLYPRTC    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMA_TIMAICR_RSTC       = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMA_TIMAICR_UPDC       = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMA_TIMAICR_REPC       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMA_TIMADIER5_DLYPRTDE = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMA_TIMADIER5_RSTDE    = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMA_TIMADIER5_SETX2DE  = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMA_TIMADIER5_SET1XDE  = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMA_TIMADIER5_UPDDE    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMA_TIMADIER5_REPDE    = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMA_TIMADIER5_DLYPRTIE = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMA_TIMADIER5_RSTIE    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMA_TIMADIER5_SETX2IE  = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMA_TIMADIER5_SET1XIE  = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMA_TIMADIER5_UPDIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMA_TIMADIER5_REPIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMA_CNTAR_CNTX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_PERAR_PERX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_REPAR_REPX         = {.msk = 0x000000FFU, .pos = 0};
const field32_t HRTIM_TIMA_CMP1AR_CMP1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_CMP1CAR_REPX       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t HRTIM_TIMA_CMP1CAR_CMP1X      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_CMP2AR_CMP2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_CMP3AR_CMP3X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_CMP4AR_CMP4X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_CPT1AR_CPT1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_CPT2AR_CPT2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMA_DTAR_DTFLKX        = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMA_DTAR_DTFSLKX       = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMA_DTAR_SDTFX         = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMA_DTAR_DTFX          = {.msk = 0x01FF0000U, .pos = 16};
const field32_t HRTIM_TIMA_DTAR_DTRLKX        = {.msk = 0x00008000U, .pos = 15};
const field32_t HRTIM_TIMA_DTAR_DTRSLKX       = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMA_DTAR_DTPRSC        = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMA_DTAR_SDTRX         = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMA_DTAR_DTRX          = {.msk = 0x000001FFU, .pos = 0};
const field32_t HRTIM_TIMA_SETAxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMA_SETAxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMA_SETAxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMA_SETAxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMA_SETAxR_SST         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMA_RSTAxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMA_RSTAxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMA_RSTAxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMA_RSTAxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMA_RSTAxR_SRT         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMA_RSTAR_MSTPER       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMA_RSTAR_UPDT         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMA_CHPAR_STRTPW       = {.msk = 0x00000780U, .pos = 7};
const field32_t HRTIM_TIMA_CHPAR_CHPDTY       = {.msk = 0x00000070U, .pos = 4};
const field32_t HRTIM_TIMA_CHPAR_CHPFRQ       = {.msk = 0x0000000FU, .pos = 0};
const field32_t HRTIM_TIMA_CPTxACR_TE1RST     = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMA_CPTxACR_TE1SET     = {.msk = 0x10000000U, .pos = 28};
const field32_t HRTIM_TIMA_CPTxACR_TD1RST     = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMA_CPTxACR_TD1SET     = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMA_CPTxACR_TC1RST     = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMA_CPTxACR_TC1SET     = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMA_CPTxACR_TB1RST     = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMA_CPTxACR_TB1SET     = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMA_CPTxACR_UDPCPT     = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMA_CPTxACR_SWCPT      = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMA_OUTAR_DLYPRT       = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMA_OUTAR_DLYPRTEN     = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMA_OUTAR_DTEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_TIMA_FLTAR_FLTLCK       = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated HRTIM_TIMA Register Field Definitions */

field32_t const HRTIM_TIMA_TIMACR_DELCMPx[5] = {
  [2] = {.msk = 0x00003000U, .pos = 12},
  [4] = {.msk = 0x0000C000U, .pos = 14},
};

field32_t const HRTIM_TIMA_TIMAISR_OxSTAT[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMA_TIMAISR_RSTXx[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMA_TIMAISR_SETXx[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMA_TIMAISR_CPTx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMA_TIMAISR_CMPx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMA_TIMAICR_RSTXxC[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMA_TIMAICR_SETxXC[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMA_TIMAICR_CPTxC[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMA_TIMAICR_CMPxC[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMA_TIMADIER5_RSTXxDE[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const HRTIM_TIMA_TIMADIER5_CPTxDE[3] = {
  [1] = {.msk = 0x00800000U, .pos = 23},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMA_TIMADIER5_CMPxDE[5] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMA_TIMADIER5_RSTXxIE[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMA_TIMADIER5_CPTxIE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMA_TIMADIER5_CMPxIE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMA_SETAxR_EXTEVNTx[11] = {
  [1]  = {.msk = 0x00200000U, .pos = 21},
  [2]  = {.msk = 0x00400000U, .pos = 22},
  [3]  = {.msk = 0x00800000U, .pos = 23},
  [4]  = {.msk = 0x01000000U, .pos = 24},
  [5]  = {.msk = 0x02000000U, .pos = 25},
  [6]  = {.msk = 0x04000000U, .pos = 26},
  [7]  = {.msk = 0x08000000U, .pos = 27},
  [8]  = {.msk = 0x10000000U, .pos = 28},
  [9]  = {.msk = 0x20000000U, .pos = 29},
  [10] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMA_SETAxR_TIMEVNTx[10] = {
  [1]  = {.msk = 0x00001000U, .pos = 12},
  [2]  = {.msk = 0x00002000U, .pos = 13},
  [3]  = {.msk = 0x00004000U, .pos = 14},
  [4]  = {.msk = 0x00008000U, .pos = 15},
  [5]  = {.msk = 0x00010000U, .pos = 16},
  [6]  = {.msk = 0x00020000U, .pos = 17},
  [7]  = {.msk = 0x00040000U, .pos = 18},
  [8]  = {.msk = 0x00080000U, .pos = 19},
  [9]  = {.msk = 0x00100000U, .pos = 20},
};

field32_t const HRTIM_TIMA_SETAxR_MSTCMPx[5] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMA_EEFAR1_EExFLTR[6] = {
  [1] = {.msk = 0x0000001EU, .pos = 1},
  [2] = {.msk = 0x00000780U, .pos = 7},
  [3] = {.msk = 0x0001E000U, .pos = 13},
  [4] = {.msk = 0x00780000U, .pos = 19},
  [5] = {.msk = 0x1E000000U, .pos = 25},
};

field32_t const HRTIM_TIMA_EEFAR1_EExLTCH[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00001000U, .pos = 12},
  [4] = {.msk = 0x00040000U, .pos = 18},
  [5] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMA_RSTAR_TIMECMPx[5] = {
  [1] = {.msk = 0x10000000U, .pos = 28},
  [2] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMA_RSTAR_TIMDCMPx[5] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x04000000U, .pos = 26},
  [4] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMA_RSTAR_TIMCCMPx[5] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMA_RSTAR_TIMBCMPx[5] = {
  [1] = {.msk = 0x00080000U, .pos = 19},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [4] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const HRTIM_TIMA_CPTxACR_TECMPx[3] = {
  [1] = {.msk = 0x40000000U, .pos = 30},
  [2] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const HRTIM_TIMA_CPTxACR_TDCMPx[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMA_CPTxACR_TCCMPx[3] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMA_CPTxACR_TBCMPx[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMA_CPTxACR_EXEVxCPT[11] = {
  [1]  = {.msk = 0x00000004U, .pos = 2},
  [2]  = {.msk = 0x00000008U, .pos = 3},
  [3]  = {.msk = 0x00000010U, .pos = 4},
  [4]  = {.msk = 0x00000020U, .pos = 5},
  [5]  = {.msk = 0x00000040U, .pos = 6},
  [6]  = {.msk = 0x00000080U, .pos = 7},
  [7]  = {.msk = 0x00000100U, .pos = 8},
  [8]  = {.msk = 0x00000200U, .pos = 9},
  [9]  = {.msk = 0x00000400U, .pos = 10},
  [10] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMA_OUTAR_DIDLx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMA_OUTAR_CHPx[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_TIMA_OUTAR_FAULTx[3] = {
  [1] = {.msk = 0x00000030U, .pos = 4},
  [2] = {.msk = 0x00300000U, .pos = 20},
};

field32_t const HRTIM_TIMA_OUTAR_IDLESx[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMA_OUTAR_IDLEMx[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const HRTIM_TIMA_OUTAR_POLx[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const HRTIM_TIMA_FLTAR_FLTxEN[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

/**************************************************************************************************
 * @section HRTIM_TIMB Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMB Register Definitions */

rw_reg32_t const HRTIM_TIMB_TIMBCR    = (rw_reg32_t)0x40017500U;
ro_reg32_t const HRTIM_TIMB_TIMBISR   = (ro_reg32_t)0x40017504U;
rw_reg32_t const HRTIM_TIMB_TIMBICR   = (rw_reg32_t)0x40017508U;
rw_reg32_t const HRTIM_TIMB_TIMBDIER5 = (rw_reg32_t)0x4001750CU;
rw_reg32_t const HRTIM_TIMB_CNTR      = (rw_reg32_t)0x40017510U;
rw_reg32_t const HRTIM_TIMB_PERBR     = (rw_reg32_t)0x40017514U;
rw_reg32_t const HRTIM_TIMB_REPBR     = (rw_reg32_t)0x40017518U;
rw_reg32_t const HRTIM_TIMB_CMP1BR    = (rw_reg32_t)0x4001751CU;
rw_reg32_t const HRTIM_TIMB_CMP1CBR   = (rw_reg32_t)0x40017520U;
rw_reg32_t const HRTIM_TIMB_CMP2BR    = (rw_reg32_t)0x40017524U;
rw_reg32_t const HRTIM_TIMB_CMP3BR    = (rw_reg32_t)0x40017528U;
rw_reg32_t const HRTIM_TIMB_CMP4BR    = (rw_reg32_t)0x4001752CU;
ro_reg32_t const HRTIM_TIMB_CPT1BR    = (ro_reg32_t)0x40017530U;
ro_reg32_t const HRTIM_TIMB_CPT2BR    = (ro_reg32_t)0x40017534U;
rw_reg32_t const HRTIM_TIMB_DTBR      = (rw_reg32_t)0x40017538U;
rw_reg32_t const HRTIM_TIMB_EEFBR1    = (rw_reg32_t)0x4001754CU;
rw_reg32_t const HRTIM_TIMB_EEFBR2    = (rw_reg32_t)0x40017550U;
rw_reg32_t const HRTIM_TIMB_RSTBR     = (rw_reg32_t)0x40017554U;
rw_reg32_t const HRTIM_TIMB_CHPBR     = (rw_reg32_t)0x40017558U;
rw_reg32_t const HRTIM_TIMB_OUTBR     = (rw_reg32_t)0x40017564U;
rw_reg32_t const HRTIM_TIMB_FLTBR     = (rw_reg32_t)0x40017568U;

/** @subsection Enumerated HRTIM_TIMB Register Definitions */

rw_reg32_t const HRTIM_TIMB_SETBxR[3] = {
  [1] = (rw_reg32_t)0x4001753CU,
  [2] = (rw_reg32_t)0x40017544U,
};

rw_reg32_t const HRTIM_TIMB_RSTBxR[3] = {
  [1] = (rw_reg32_t)0x40017540U,
  [2] = (rw_reg32_t)0x40017548U,
};

rw_reg32_t const HRTIM_TIMB_CPTxBCR[3] = {
  [1] = (rw_reg32_t)0x4001755CU,
  [2] = (rw_reg32_t)0x40017560U,
};

/** @subsection HRTIM_TIMB Register Field Definitions */

const field32_t HRTIM_TIMB_TIMBCR_UPDGAT      = {.msk = 0xF0000000U, .pos = 28};
const field32_t HRTIM_TIMB_TIMBCR_PREEN       = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMB_TIMBCR_DACSYNC     = {.msk = 0x06000000U, .pos = 25};
const field32_t HRTIM_TIMB_TIMBCR_MSTU        = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMB_TIMBCR_TEU         = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_TIMB_TIMBCR_TDU         = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMB_TIMBCR_TCU         = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMB_TIMBCR_TBU         = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMB_TIMBCR_TXRSTU      = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_TIMB_TIMBCR_TXREPU      = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMB_TIMBCR_SYNCSTRTX   = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMB_TIMBCR_SYNCRSTX    = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_TIMB_TIMBCR_PSHPLL      = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMB_TIMBCR_HALF        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_TIMB_TIMBCR_RETRIG      = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMB_TIMBCR_CONT        = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_TIMB_TIMBCR_CK_PSCX     = {.msk = 0x00000007U, .pos = 0};
const field32_t HRTIM_TIMB_TIMBISR_IPPSTAT    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMB_TIMBISR_CPPSTAT    = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMB_TIMBISR_DLYPRT     = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMB_TIMBISR_RST        = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMB_TIMBISR_UPD        = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMB_TIMBISR_REP        = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMB_TIMBICR_DLYPRTC    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMB_TIMBICR_RSTC       = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMB_TIMBICR_UPDC       = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMB_TIMBICR_REPC       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMB_TIMBDIER5_DLYPRTDE = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMB_TIMBDIER5_RSTDE    = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMB_TIMBDIER5_SETX2DE  = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMB_TIMBDIER5_SET1XDE  = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMB_TIMBDIER5_UPDDE    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMB_TIMBDIER5_REPDE    = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMB_TIMBDIER5_DLYPRTIE = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMB_TIMBDIER5_RSTIE    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMB_TIMBDIER5_SETX2IE  = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMB_TIMBDIER5_SET1XIE  = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMB_TIMBDIER5_UPDIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMB_TIMBDIER5_REPIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMB_CNTR_CNTX          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_PERBR_PERX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_REPBR_REPX         = {.msk = 0x000000FFU, .pos = 0};
const field32_t HRTIM_TIMB_CMP1BR_CMP1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_CMP1CBR_REPX       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t HRTIM_TIMB_CMP1CBR_CMP1X      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_CMP2BR_CMP2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_CMP3BR_CMP3X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_CMP4BR_CMP4X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_CPT1BR_CPT1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_CPT2BR_CPT2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMB_DTBR_DTFLKX        = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMB_DTBR_DTFSLKX       = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMB_DTBR_SDTFX         = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMB_DTBR_DTFX          = {.msk = 0x01FF0000U, .pos = 16};
const field32_t HRTIM_TIMB_DTBR_DTRLKX        = {.msk = 0x00008000U, .pos = 15};
const field32_t HRTIM_TIMB_DTBR_DTRSLKX       = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMB_DTBR_DTPRSC        = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMB_DTBR_SDTRX         = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMB_DTBR_DTRX          = {.msk = 0x000001FFU, .pos = 0};
const field32_t HRTIM_TIMB_SETBxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMB_SETBxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMB_SETBxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMB_SETBxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMB_SETBxR_SST         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMB_RSTBxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMB_RSTBxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMB_RSTBxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMB_RSTBxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMB_RSTBxR_SRT         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMB_RSTBR_MSTPER       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMB_RSTBR_UPDT         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMB_CHPBR_STRTPW       = {.msk = 0x00000780U, .pos = 7};
const field32_t HRTIM_TIMB_CHPBR_CHPDTY       = {.msk = 0x00000070U, .pos = 4};
const field32_t HRTIM_TIMB_CHPBR_CHPFRQ       = {.msk = 0x0000000FU, .pos = 0};
const field32_t HRTIM_TIMB_CPTxBCR_TE1RST     = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMB_CPTxBCR_TE1SET     = {.msk = 0x10000000U, .pos = 28};
const field32_t HRTIM_TIMB_CPTxBCR_TD1RST     = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMB_CPTxBCR_TD1SET     = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMB_CPTxBCR_TC1RST     = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMB_CPTxBCR_TC1SET     = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMB_CPTxBCR_TA1RST     = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMB_CPTxBCR_TA1SET     = {.msk = 0x00001000U, .pos = 12};
const field32_t HRTIM_TIMB_CPTxBCR_UDPCPT     = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMB_CPTxBCR_SWCPT      = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMB_OUTBR_DLYPRT       = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMB_OUTBR_DLYPRTEN     = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMB_OUTBR_DTEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_TIMB_FLTBR_FLTLCK       = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated HRTIM_TIMB Register Field Definitions */

field32_t const HRTIM_TIMB_TIMBCR_DELCMPx[5] = {
  [2] = {.msk = 0x00003000U, .pos = 12},
  [4] = {.msk = 0x0000C000U, .pos = 14},
};

field32_t const HRTIM_TIMB_TIMBISR_OxSTAT[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMB_TIMBISR_RSTXx[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMB_TIMBISR_SETXx[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMB_TIMBISR_CPTx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMB_TIMBISR_CMPx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMB_TIMBICR_RSTXxC[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMB_TIMBICR_SETxXC[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMB_TIMBICR_CPTxC[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMB_TIMBICR_CMPxC[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMB_TIMBDIER5_RSTXxDE[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const HRTIM_TIMB_TIMBDIER5_CPTxDE[3] = {
  [1] = {.msk = 0x00800000U, .pos = 23},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMB_TIMBDIER5_CMPxDE[5] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMB_TIMBDIER5_RSTXxIE[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMB_TIMBDIER5_CPTxIE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMB_TIMBDIER5_CMPxIE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMB_SETBxR_EXTEVNTx[11] = {
  [1]  = {.msk = 0x00200000U, .pos = 21},
  [2]  = {.msk = 0x00400000U, .pos = 22},
  [3]  = {.msk = 0x00800000U, .pos = 23},
  [4]  = {.msk = 0x01000000U, .pos = 24},
  [5]  = {.msk = 0x02000000U, .pos = 25},
  [6]  = {.msk = 0x04000000U, .pos = 26},
  [7]  = {.msk = 0x08000000U, .pos = 27},
  [8]  = {.msk = 0x10000000U, .pos = 28},
  [9]  = {.msk = 0x20000000U, .pos = 29},
  [10] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMB_SETBxR_TIMEVNTx[10] = {
  [1]  = {.msk = 0x00001000U, .pos = 12},
  [2]  = {.msk = 0x00002000U, .pos = 13},
  [3]  = {.msk = 0x00004000U, .pos = 14},
  [4]  = {.msk = 0x00008000U, .pos = 15},
  [5]  = {.msk = 0x00010000U, .pos = 16},
  [6]  = {.msk = 0x00020000U, .pos = 17},
  [7]  = {.msk = 0x00040000U, .pos = 18},
  [8]  = {.msk = 0x00080000U, .pos = 19},
  [9]  = {.msk = 0x00100000U, .pos = 20},
};

field32_t const HRTIM_TIMB_SETBxR_MSTCMPx[5] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMB_EEFBR1_EExFLTR[6] = {
  [1] = {.msk = 0x0000001EU, .pos = 1},
  [2] = {.msk = 0x00000780U, .pos = 7},
  [3] = {.msk = 0x0001E000U, .pos = 13},
  [4] = {.msk = 0x00780000U, .pos = 19},
  [5] = {.msk = 0x1E000000U, .pos = 25},
};

field32_t const HRTIM_TIMB_EEFBR1_EExLTCH[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00001000U, .pos = 12},
  [4] = {.msk = 0x00040000U, .pos = 18},
  [5] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMB_RSTBR_TIMECMPx[5] = {
  [1] = {.msk = 0x10000000U, .pos = 28},
  [2] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMB_RSTBR_TIMDCMPx[5] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x04000000U, .pos = 26},
  [4] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMB_RSTBR_TIMCCMPx[5] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMB_RSTBR_TIMACMPx[5] = {
  [1] = {.msk = 0x00080000U, .pos = 19},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [4] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const HRTIM_TIMB_CPTxBCR_TECMPx[3] = {
  [1] = {.msk = 0x40000000U, .pos = 30},
  [2] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const HRTIM_TIMB_CPTxBCR_TDCMPx[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMB_CPTxBCR_TCCMPx[3] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMB_CPTxBCR_TACMPx[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const HRTIM_TIMB_CPTxBCR_EXEVxCPT[11] = {
  [1]  = {.msk = 0x00000004U, .pos = 2},
  [2]  = {.msk = 0x00000008U, .pos = 3},
  [3]  = {.msk = 0x00000010U, .pos = 4},
  [4]  = {.msk = 0x00000020U, .pos = 5},
  [5]  = {.msk = 0x00000040U, .pos = 6},
  [6]  = {.msk = 0x00000080U, .pos = 7},
  [7]  = {.msk = 0x00000100U, .pos = 8},
  [8]  = {.msk = 0x00000200U, .pos = 9},
  [9]  = {.msk = 0x00000400U, .pos = 10},
  [10] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMB_OUTBR_DIDLx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMB_OUTBR_CHPx[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_TIMB_OUTBR_FAULTx[3] = {
  [1] = {.msk = 0x00000030U, .pos = 4},
  [2] = {.msk = 0x00300000U, .pos = 20},
};

field32_t const HRTIM_TIMB_OUTBR_IDLESx[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMB_OUTBR_IDLEMx[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const HRTIM_TIMB_OUTBR_POLx[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const HRTIM_TIMB_FLTBR_FLTxEN[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

/**************************************************************************************************
 * @section HRTIM_TIMC Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMC Register Definitions */

rw_reg32_t const HRTIM_TIMC_TIMCCR    = (rw_reg32_t)0x40017580U;
ro_reg32_t const HRTIM_TIMC_TIMCISR   = (ro_reg32_t)0x40017584U;
rw_reg32_t const HRTIM_TIMC_TIMCICR   = (rw_reg32_t)0x40017588U;
rw_reg32_t const HRTIM_TIMC_TIMCDIER5 = (rw_reg32_t)0x4001758CU;
rw_reg32_t const HRTIM_TIMC_CNTCR     = (rw_reg32_t)0x40017590U;
rw_reg32_t const HRTIM_TIMC_PERCR     = (rw_reg32_t)0x40017594U;
rw_reg32_t const HRTIM_TIMC_REPCR     = (rw_reg32_t)0x40017598U;
rw_reg32_t const HRTIM_TIMC_CMP1CR    = (rw_reg32_t)0x4001759CU;
rw_reg32_t const HRTIM_TIMC_CMP1CCR   = (rw_reg32_t)0x400175A0U;
rw_reg32_t const HRTIM_TIMC_CMP2CR    = (rw_reg32_t)0x400175A4U;
rw_reg32_t const HRTIM_TIMC_CMP3CR    = (rw_reg32_t)0x400175A8U;
rw_reg32_t const HRTIM_TIMC_CMP4CR    = (rw_reg32_t)0x400175ACU;
ro_reg32_t const HRTIM_TIMC_CPT1CR    = (ro_reg32_t)0x400175B0U;
ro_reg32_t const HRTIM_TIMC_CPT2CR    = (ro_reg32_t)0x400175B4U;
rw_reg32_t const HRTIM_TIMC_DTCR      = (rw_reg32_t)0x400175B8U;
rw_reg32_t const HRTIM_TIMC_EEFCR1    = (rw_reg32_t)0x400175CCU;
rw_reg32_t const HRTIM_TIMC_EEFCR2    = (rw_reg32_t)0x400175D0U;
rw_reg32_t const HRTIM_TIMC_RSTCR     = (rw_reg32_t)0x400175D4U;
rw_reg32_t const HRTIM_TIMC_CHPCR     = (rw_reg32_t)0x400175D8U;
rw_reg32_t const HRTIM_TIMC_OUTCR     = (rw_reg32_t)0x400175E4U;
rw_reg32_t const HRTIM_TIMC_FLTCR     = (rw_reg32_t)0x400175E8U;

/** @subsection Enumerated HRTIM_TIMC Register Definitions */

rw_reg32_t const HRTIM_TIMC_SETCxR[3] = {
  [1] = (rw_reg32_t)0x400175BCU,
  [2] = (rw_reg32_t)0x400175C4U,
};

rw_reg32_t const HRTIM_TIMC_RSTCxR[3] = {
  [1] = (rw_reg32_t)0x400175C0U,
  [2] = (rw_reg32_t)0x400175C8U,
};

rw_reg32_t const HRTIM_TIMC_CPTxCCR[3] = {
  [1] = (rw_reg32_t)0x400175DCU,
  [2] = (rw_reg32_t)0x400175E0U,
};

/** @subsection HRTIM_TIMC Register Field Definitions */

const field32_t HRTIM_TIMC_TIMCCR_UPDGAT      = {.msk = 0xF0000000U, .pos = 28};
const field32_t HRTIM_TIMC_TIMCCR_PREEN       = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMC_TIMCCR_DACSYNC     = {.msk = 0x06000000U, .pos = 25};
const field32_t HRTIM_TIMC_TIMCCR_MSTU        = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMC_TIMCCR_TEU         = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_TIMC_TIMCCR_TDU         = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMC_TIMCCR_TCU         = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMC_TIMCCR_TBU         = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMC_TIMCCR_TXRSTU      = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_TIMC_TIMCCR_TXREPU      = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMC_TIMCCR_SYNCSTRTX   = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMC_TIMCCR_SYNCRSTX    = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_TIMC_TIMCCR_PSHPLL      = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMC_TIMCCR_HALF        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_TIMC_TIMCCR_RETRIG      = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMC_TIMCCR_CONT        = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_TIMC_TIMCCR_CK_PSCX     = {.msk = 0x00000007U, .pos = 0};
const field32_t HRTIM_TIMC_TIMCISR_IPPSTAT    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMC_TIMCISR_CPPSTAT    = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMC_TIMCISR_DLYPRT     = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMC_TIMCISR_RST        = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMC_TIMCISR_UPD        = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMC_TIMCISR_REP        = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMC_TIMCICR_DLYPRTC    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMC_TIMCICR_RSTC       = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMC_TIMCICR_UPDC       = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMC_TIMCICR_REPC       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMC_TIMCDIER5_DLYPRTDE = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMC_TIMCDIER5_RSTDE    = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMC_TIMCDIER5_SETX2DE  = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMC_TIMCDIER5_SET1XDE  = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMC_TIMCDIER5_UPDDE    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMC_TIMCDIER5_REPDE    = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMC_TIMCDIER5_DLYPRTIE = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMC_TIMCDIER5_RSTIE    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMC_TIMCDIER5_SETX2IE  = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMC_TIMCDIER5_SET1XIE  = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMC_TIMCDIER5_UPDIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMC_TIMCDIER5_REPIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMC_CNTCR_CNTX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_PERCR_PERX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_REPCR_REPX         = {.msk = 0x000000FFU, .pos = 0};
const field32_t HRTIM_TIMC_CMP1CR_CMP1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_CMP1CCR_REPX       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t HRTIM_TIMC_CMP1CCR_CMP1X      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_CMP2CR_CMP2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_CMP3CR_CMP3X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_CMP4CR_CMP4X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_CPT1CR_CPT1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_CPT2CR_CPT2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMC_DTCR_DTFLKX        = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMC_DTCR_DTFSLKX       = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMC_DTCR_SDTFX         = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMC_DTCR_DTFX          = {.msk = 0x01FF0000U, .pos = 16};
const field32_t HRTIM_TIMC_DTCR_DTRLKX        = {.msk = 0x00008000U, .pos = 15};
const field32_t HRTIM_TIMC_DTCR_DTRSLKX       = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMC_DTCR_DTPRSC        = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMC_DTCR_SDTRX         = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMC_DTCR_DTRX          = {.msk = 0x000001FFU, .pos = 0};
const field32_t HRTIM_TIMC_SETCxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMC_SETCxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMC_SETCxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMC_SETCxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMC_SETCxR_SST         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMC_RSTCxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMC_RSTCxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMC_RSTCxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMC_RSTCxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMC_RSTCxR_SRT         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMC_RSTCR_MSTPER       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMC_RSTCR_UPDT         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMC_CHPCR_STRTPW       = {.msk = 0x00000780U, .pos = 7};
const field32_t HRTIM_TIMC_CHPCR_CHPDTY       = {.msk = 0x00000070U, .pos = 4};
const field32_t HRTIM_TIMC_CHPCR_CHPFRQ       = {.msk = 0x0000000FU, .pos = 0};
const field32_t HRTIM_TIMC_CPTxCCR_TE1RST     = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMC_CPTxCCR_TE1SET     = {.msk = 0x10000000U, .pos = 28};
const field32_t HRTIM_TIMC_CPTxCCR_TD1RST     = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMC_CPTxCCR_TD1SET     = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMC_CPTxCCR_TB1RST     = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMC_CPTxCCR_TB1SET     = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMC_CPTxCCR_TA1RST     = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMC_CPTxCCR_TA1SET     = {.msk = 0x00001000U, .pos = 12};
const field32_t HRTIM_TIMC_CPTxCCR_UDPCPT     = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMC_CPTxCCR_SWCPT      = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMC_OUTCR_DLYPRT       = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMC_OUTCR_DLYPRTEN     = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMC_OUTCR_DTEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_TIMC_FLTCR_FLTLCK       = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated HRTIM_TIMC Register Field Definitions */

field32_t const HRTIM_TIMC_TIMCCR_DELCMPx[5] = {
  [2] = {.msk = 0x00003000U, .pos = 12},
  [4] = {.msk = 0x0000C000U, .pos = 14},
};

field32_t const HRTIM_TIMC_TIMCISR_OxSTAT[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMC_TIMCISR_RSTXx[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMC_TIMCISR_SETXx[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMC_TIMCISR_CPTx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMC_TIMCISR_CMPx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMC_TIMCICR_RSTXxC[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMC_TIMCICR_SETxXC[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMC_TIMCICR_CPTxC[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMC_TIMCICR_CMPxC[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMC_TIMCDIER5_RSTXxDE[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const HRTIM_TIMC_TIMCDIER5_CPTxDE[3] = {
  [1] = {.msk = 0x00800000U, .pos = 23},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMC_TIMCDIER5_CMPxDE[5] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMC_TIMCDIER5_RSTXxIE[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMC_TIMCDIER5_CPTxIE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMC_TIMCDIER5_CMPxIE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMC_SETCxR_EXTEVNTx[11] = {
  [1]  = {.msk = 0x00200000U, .pos = 21},
  [2]  = {.msk = 0x00400000U, .pos = 22},
  [3]  = {.msk = 0x00800000U, .pos = 23},
  [4]  = {.msk = 0x01000000U, .pos = 24},
  [5]  = {.msk = 0x02000000U, .pos = 25},
  [6]  = {.msk = 0x04000000U, .pos = 26},
  [7]  = {.msk = 0x08000000U, .pos = 27},
  [8]  = {.msk = 0x10000000U, .pos = 28},
  [9]  = {.msk = 0x20000000U, .pos = 29},
  [10] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMC_SETCxR_TIMEVNTx[10] = {
  [1]  = {.msk = 0x00001000U, .pos = 12},
  [2]  = {.msk = 0x00002000U, .pos = 13},
  [3]  = {.msk = 0x00004000U, .pos = 14},
  [4]  = {.msk = 0x00008000U, .pos = 15},
  [5]  = {.msk = 0x00010000U, .pos = 16},
  [6]  = {.msk = 0x00020000U, .pos = 17},
  [7]  = {.msk = 0x00040000U, .pos = 18},
  [8]  = {.msk = 0x00080000U, .pos = 19},
  [9]  = {.msk = 0x00100000U, .pos = 20},
};

field32_t const HRTIM_TIMC_SETCxR_MSTCMPx[5] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMC_EEFCR1_EExFLTR[6] = {
  [1] = {.msk = 0x0000001EU, .pos = 1},
  [2] = {.msk = 0x00000780U, .pos = 7},
  [3] = {.msk = 0x0001E000U, .pos = 13},
  [4] = {.msk = 0x00780000U, .pos = 19},
  [5] = {.msk = 0x1E000000U, .pos = 25},
};

field32_t const HRTIM_TIMC_EEFCR1_EExLTCH[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00001000U, .pos = 12},
  [4] = {.msk = 0x00040000U, .pos = 18},
  [5] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMC_RSTCR_TIMECMPx[5] = {
  [1] = {.msk = 0x10000000U, .pos = 28},
  [2] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMC_RSTCR_TIMDCMPx[5] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x04000000U, .pos = 26},
  [4] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMC_RSTCR_TIMBCMPx[5] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMC_RSTCR_TIMACMPx[5] = {
  [1] = {.msk = 0x00080000U, .pos = 19},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [4] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const HRTIM_TIMC_CPTxCCR_TECMPx[3] = {
  [1] = {.msk = 0x40000000U, .pos = 30},
  [2] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const HRTIM_TIMC_CPTxCCR_TDCMPx[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMC_CPTxCCR_TBCMPx[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMC_CPTxCCR_TACMPx[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const HRTIM_TIMC_CPTxCCR_EXEVxCPT[11] = {
  [1]  = {.msk = 0x00000004U, .pos = 2},
  [2]  = {.msk = 0x00000008U, .pos = 3},
  [3]  = {.msk = 0x00000010U, .pos = 4},
  [4]  = {.msk = 0x00000020U, .pos = 5},
  [5]  = {.msk = 0x00000040U, .pos = 6},
  [6]  = {.msk = 0x00000080U, .pos = 7},
  [7]  = {.msk = 0x00000100U, .pos = 8},
  [8]  = {.msk = 0x00000200U, .pos = 9},
  [9]  = {.msk = 0x00000400U, .pos = 10},
  [10] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMC_OUTCR_DIDLx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMC_OUTCR_CHPx[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_TIMC_OUTCR_FAULTx[3] = {
  [1] = {.msk = 0x00000030U, .pos = 4},
  [2] = {.msk = 0x00300000U, .pos = 20},
};

field32_t const HRTIM_TIMC_OUTCR_IDLESx[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMC_OUTCR_IDLEMx[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const HRTIM_TIMC_OUTCR_POLx[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const HRTIM_TIMC_FLTCR_FLTxEN[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

/**************************************************************************************************
 * @section HRTIM_TIMD Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIMD Register Definitions */

rw_reg32_t const HRTIM_TIMD_TIMDCR    = (rw_reg32_t)0x40017600U;
ro_reg32_t const HRTIM_TIMD_TIMDISR   = (ro_reg32_t)0x40017604U;
rw_reg32_t const HRTIM_TIMD_TIMDICR   = (rw_reg32_t)0x40017608U;
rw_reg32_t const HRTIM_TIMD_TIMDDIER5 = (rw_reg32_t)0x4001760CU;
rw_reg32_t const HRTIM_TIMD_CNTDR     = (rw_reg32_t)0x40017610U;
rw_reg32_t const HRTIM_TIMD_PERDR     = (rw_reg32_t)0x40017614U;
rw_reg32_t const HRTIM_TIMD_REPDR     = (rw_reg32_t)0x40017618U;
rw_reg32_t const HRTIM_TIMD_CMP1DR    = (rw_reg32_t)0x4001761CU;
rw_reg32_t const HRTIM_TIMD_CMP1CDR   = (rw_reg32_t)0x40017620U;
rw_reg32_t const HRTIM_TIMD_CMP2DR    = (rw_reg32_t)0x40017624U;
rw_reg32_t const HRTIM_TIMD_CMP3DR    = (rw_reg32_t)0x40017628U;
rw_reg32_t const HRTIM_TIMD_CMP4DR    = (rw_reg32_t)0x4001762CU;
ro_reg32_t const HRTIM_TIMD_CPT1DR    = (ro_reg32_t)0x40017630U;
ro_reg32_t const HRTIM_TIMD_CPT2DR    = (ro_reg32_t)0x40017634U;
rw_reg32_t const HRTIM_TIMD_DTDR      = (rw_reg32_t)0x40017638U;
rw_reg32_t const HRTIM_TIMD_EEFDR1    = (rw_reg32_t)0x4001764CU;
rw_reg32_t const HRTIM_TIMD_EEFDR2    = (rw_reg32_t)0x40017650U;
rw_reg32_t const HRTIM_TIMD_RSTDR     = (rw_reg32_t)0x40017654U;
rw_reg32_t const HRTIM_TIMD_CHPDR     = (rw_reg32_t)0x40017658U;
rw_reg32_t const HRTIM_TIMD_OUTDR     = (rw_reg32_t)0x40017664U;
rw_reg32_t const HRTIM_TIMD_FLTDR     = (rw_reg32_t)0x40017668U;

/** @subsection Enumerated HRTIM_TIMD Register Definitions */

rw_reg32_t const HRTIM_TIMD_SETDxR[3] = {
  [1] = (rw_reg32_t)0x4001763CU,
  [2] = (rw_reg32_t)0x40017644U,
};

rw_reg32_t const HRTIM_TIMD_RSTDxR[3] = {
  [1] = (rw_reg32_t)0x40017640U,
  [2] = (rw_reg32_t)0x40017648U,
};

rw_reg32_t const HRTIM_TIMD_CPTxDCR[3] = {
  [1] = (rw_reg32_t)0x4001765CU,
  [2] = (rw_reg32_t)0x40017660U,
};

/** @subsection HRTIM_TIMD Register Field Definitions */

const field32_t HRTIM_TIMD_TIMDCR_UPDGAT      = {.msk = 0xF0000000U, .pos = 28};
const field32_t HRTIM_TIMD_TIMDCR_PREEN       = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMD_TIMDCR_DACSYNC     = {.msk = 0x06000000U, .pos = 25};
const field32_t HRTIM_TIMD_TIMDCR_MSTU        = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIMD_TIMDCR_TEU         = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_TIMD_TIMDCR_TDU         = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMD_TIMDCR_TCU         = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMD_TIMDCR_TBU         = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMD_TIMDCR_TXRSTU      = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_TIMD_TIMDCR_TXREPU      = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMD_TIMDCR_SYNCSTRTX   = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMD_TIMDCR_SYNCRSTX    = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_TIMD_TIMDCR_PSHPLL      = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMD_TIMDCR_HALF        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_TIMD_TIMDCR_RETRIG      = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMD_TIMDCR_CONT        = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_TIMD_TIMDCR_CK_PSCX     = {.msk = 0x00000007U, .pos = 0};
const field32_t HRTIM_TIMD_TIMDISR_IPPSTAT    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMD_TIMDISR_CPPSTAT    = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMD_TIMDISR_DLYPRT     = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMD_TIMDISR_RST        = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMD_TIMDISR_UPD        = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMD_TIMDISR_REP        = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMD_TIMDICR_DLYPRTC    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMD_TIMDICR_RSTC       = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMD_TIMDICR_UPDC       = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMD_TIMDICR_REPC       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMD_TIMDDIER5_DLYPRTDE = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMD_TIMDDIER5_RSTDE    = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMD_TIMDDIER5_SETX2DE  = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIMD_TIMDDIER5_SET1XDE  = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMD_TIMDDIER5_UPDDE    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIMD_TIMDDIER5_REPDE    = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMD_TIMDDIER5_DLYPRTIE = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMD_TIMDDIER5_RSTIE    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMD_TIMDDIER5_SETX2IE  = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIMD_TIMDDIER5_SET1XIE  = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMD_TIMDDIER5_UPDIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIMD_TIMDDIER5_REPIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMD_CNTDR_CNTX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_PERDR_PERX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_REPDR_REPX         = {.msk = 0x000000FFU, .pos = 0};
const field32_t HRTIM_TIMD_CMP1DR_CMP1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_CMP1CDR_REPX       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t HRTIM_TIMD_CMP1CDR_CMP1X      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_CMP2DR_CMP2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_CMP3DR_CMP3X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_CMP4DR_CMP4X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_CPT1DR_CPT1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_CPT2DR_CPT2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIMD_DTDR_DTFLKX        = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMD_DTDR_DTFSLKX       = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIMD_DTDR_SDTFX         = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIMD_DTDR_DTFX          = {.msk = 0x01FF0000U, .pos = 16};
const field32_t HRTIM_TIMD_DTDR_DTRLKX        = {.msk = 0x00008000U, .pos = 15};
const field32_t HRTIM_TIMD_DTDR_DTRSLKX       = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIMD_DTDR_DTPRSC        = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMD_DTDR_SDTRX         = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMD_DTDR_DTRX          = {.msk = 0x000001FFU, .pos = 0};
const field32_t HRTIM_TIMD_SETDxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMD_SETDxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMD_SETDxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMD_SETDxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMD_SETDxR_SST         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMD_RSTDxR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIMD_RSTDxR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIMD_RSTDxR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIMD_RSTDxR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMD_RSTDxR_SRT         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMD_RSTDR_MSTPER       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIMD_RSTDR_UPDT         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMD_CHPDR_STRTPW       = {.msk = 0x00000780U, .pos = 7};
const field32_t HRTIM_TIMD_CHPDR_CHPDTY       = {.msk = 0x00000070U, .pos = 4};
const field32_t HRTIM_TIMD_CHPDR_CHPFRQ       = {.msk = 0x0000000FU, .pos = 0};
const field32_t HRTIM_TIMD_CPTxDCR_TE1RST     = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIMD_CPTxDCR_TE1SET     = {.msk = 0x10000000U, .pos = 28};
const field32_t HRTIM_TIMD_CPTxDCR_TC1RST     = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIMD_CPTxDCR_TC1SET     = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIMD_CPTxDCR_TB1RST     = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIMD_CPTxDCR_TB1SET     = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIMD_CPTxDCR_TA1RST     = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIMD_CPTxDCR_TA1SET     = {.msk = 0x00001000U, .pos = 12};
const field32_t HRTIM_TIMD_CPTxDCR_UDPCPT     = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIMD_CPTxDCR_SWCPT      = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIMD_OUTDR_DLYPRT       = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIMD_OUTDR_DLYPRTEN     = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIMD_OUTDR_DTEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_TIMD_FLTDR_FLTLCK       = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated HRTIM_TIMD Register Field Definitions */

field32_t const HRTIM_TIMD_TIMDCR_DELCMPx[5] = {
  [2] = {.msk = 0x00003000U, .pos = 12},
  [4] = {.msk = 0x0000C000U, .pos = 14},
};

field32_t const HRTIM_TIMD_TIMDISR_OxSTAT[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMD_TIMDISR_RSTXx[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMD_TIMDISR_SETXx[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMD_TIMDISR_CPTx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMD_TIMDISR_CMPx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMD_TIMDICR_RSTXxC[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMD_TIMDICR_SETxXC[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMD_TIMDICR_CPTxC[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMD_TIMDICR_CMPxC[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMD_TIMDDIER5_RSTXxDE[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const HRTIM_TIMD_TIMDDIER5_CPTxDE[3] = {
  [1] = {.msk = 0x00800000U, .pos = 23},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMD_TIMDDIER5_CMPxDE[5] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMD_TIMDDIER5_RSTXxIE[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIMD_TIMDDIER5_CPTxIE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIMD_TIMDDIER5_CMPxIE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIMD_SETDxR_EXTEVNTx[11] = {
  [1]  = {.msk = 0x00200000U, .pos = 21},
  [2]  = {.msk = 0x00400000U, .pos = 22},
  [3]  = {.msk = 0x00800000U, .pos = 23},
  [4]  = {.msk = 0x01000000U, .pos = 24},
  [5]  = {.msk = 0x02000000U, .pos = 25},
  [6]  = {.msk = 0x04000000U, .pos = 26},
  [7]  = {.msk = 0x08000000U, .pos = 27},
  [8]  = {.msk = 0x10000000U, .pos = 28},
  [9]  = {.msk = 0x20000000U, .pos = 29},
  [10] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMD_SETDxR_TIMEVNTx[10] = {
  [1]  = {.msk = 0x00001000U, .pos = 12},
  [2]  = {.msk = 0x00002000U, .pos = 13},
  [3]  = {.msk = 0x00004000U, .pos = 14},
  [4]  = {.msk = 0x00008000U, .pos = 15},
  [5]  = {.msk = 0x00010000U, .pos = 16},
  [6]  = {.msk = 0x00020000U, .pos = 17},
  [7]  = {.msk = 0x00040000U, .pos = 18},
  [8]  = {.msk = 0x00080000U, .pos = 19},
  [9]  = {.msk = 0x00100000U, .pos = 20},
};

field32_t const HRTIM_TIMD_SETDxR_MSTCMPx[5] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMD_EEFDR1_EExFLTR[6] = {
  [1] = {.msk = 0x0000001EU, .pos = 1},
  [2] = {.msk = 0x00000780U, .pos = 7},
  [3] = {.msk = 0x0001E000U, .pos = 13},
  [4] = {.msk = 0x00780000U, .pos = 19},
  [5] = {.msk = 0x1E000000U, .pos = 25},
};

field32_t const HRTIM_TIMD_EEFDR1_EExLTCH[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00001000U, .pos = 12},
  [4] = {.msk = 0x00040000U, .pos = 18},
  [5] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMD_RSTDR_TIMECMPx[5] = {
  [1] = {.msk = 0x10000000U, .pos = 28},
  [2] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIMD_RSTDR_TIMCCMPx[5] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x04000000U, .pos = 26},
  [4] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIMD_RSTDR_TIMBCMPx[5] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIMD_RSTDR_TIMACMPx[5] = {
  [1] = {.msk = 0x00080000U, .pos = 19},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [4] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const HRTIM_TIMD_CPTxDCR_TECMPx[3] = {
  [1] = {.msk = 0x40000000U, .pos = 30},
  [2] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const HRTIM_TIMD_CPTxDCR_TCCMPx[3] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMD_CPTxDCR_TBCMPx[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMD_CPTxDCR_TACMPx[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const HRTIM_TIMD_CPTxDCR_EXEVxCPT[11] = {
  [1]  = {.msk = 0x00000004U, .pos = 2},
  [2]  = {.msk = 0x00000008U, .pos = 3},
  [3]  = {.msk = 0x00000010U, .pos = 4},
  [4]  = {.msk = 0x00000020U, .pos = 5},
  [5]  = {.msk = 0x00000040U, .pos = 6},
  [6]  = {.msk = 0x00000080U, .pos = 7},
  [7]  = {.msk = 0x00000100U, .pos = 8},
  [8]  = {.msk = 0x00000200U, .pos = 9},
  [9]  = {.msk = 0x00000400U, .pos = 10},
  [10] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIMD_OUTDR_DIDLx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIMD_OUTDR_CHPx[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_TIMD_OUTDR_FAULTx[3] = {
  [1] = {.msk = 0x00000030U, .pos = 4},
  [2] = {.msk = 0x00300000U, .pos = 20},
};

field32_t const HRTIM_TIMD_OUTDR_IDLESx[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIMD_OUTDR_IDLEMx[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const HRTIM_TIMD_OUTDR_POLx[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const HRTIM_TIMD_FLTDR_FLTxEN[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

/**************************************************************************************************
 * @section HRTIM_TIME Definitions
 **************************************************************************************************/

/** @subsection HRTIM_TIME Register Definitions */

rw_reg32_t const HRTIM_TIME_TIMECR    = (rw_reg32_t)0x40017680U;
ro_reg32_t const HRTIM_TIME_TIMEISR   = (ro_reg32_t)0x40017684U;
rw_reg32_t const HRTIM_TIME_TIMEICR   = (rw_reg32_t)0x40017688U;
rw_reg32_t const HRTIM_TIME_TIMEDIER5 = (rw_reg32_t)0x4001768CU;
rw_reg32_t const HRTIM_TIME_CNTER     = (rw_reg32_t)0x40017690U;
rw_reg32_t const HRTIM_TIME_PERER     = (rw_reg32_t)0x40017694U;
rw_reg32_t const HRTIM_TIME_REPER     = (rw_reg32_t)0x40017698U;
rw_reg32_t const HRTIM_TIME_CMP1ER    = (rw_reg32_t)0x4001769CU;
rw_reg32_t const HRTIM_TIME_CMP1CER   = (rw_reg32_t)0x400176A0U;
rw_reg32_t const HRTIM_TIME_CMP2ER    = (rw_reg32_t)0x400176A4U;
rw_reg32_t const HRTIM_TIME_CMP3ER    = (rw_reg32_t)0x400176A8U;
rw_reg32_t const HRTIM_TIME_CMP4ER    = (rw_reg32_t)0x400176ACU;
ro_reg32_t const HRTIM_TIME_CPT1ER    = (ro_reg32_t)0x400176B0U;
ro_reg32_t const HRTIM_TIME_CPT2ER    = (ro_reg32_t)0x400176B4U;
rw_reg32_t const HRTIM_TIME_DTER      = (rw_reg32_t)0x400176B8U;
rw_reg32_t const HRTIM_TIME_EEFER1    = (rw_reg32_t)0x400176CCU;
rw_reg32_t const HRTIM_TIME_EEFER2    = (rw_reg32_t)0x400176D0U;
rw_reg32_t const HRTIM_TIME_RSTER     = (rw_reg32_t)0x400176D4U;
rw_reg32_t const HRTIM_TIME_CHPER     = (rw_reg32_t)0x400176D8U;
rw_reg32_t const HRTIM_TIME_OUTER     = (rw_reg32_t)0x400176E4U;
rw_reg32_t const HRTIM_TIME_FLTER     = (rw_reg32_t)0x400176E8U;

/** @subsection Enumerated HRTIM_TIME Register Definitions */

rw_reg32_t const HRTIM_TIME_SETExR[3] = {
  [1] = (rw_reg32_t)0x400176BCU,
  [2] = (rw_reg32_t)0x400176C4U,
};

rw_reg32_t const HRTIM_TIME_RSTExR[3] = {
  [1] = (rw_reg32_t)0x400176C0U,
  [2] = (rw_reg32_t)0x400176C8U,
};

rw_reg32_t const HRTIM_TIME_CPTxECR[3] = {
  [1] = (rw_reg32_t)0x400176DCU,
  [2] = (rw_reg32_t)0x400176E0U,
};

/** @subsection HRTIM_TIME Register Field Definitions */

const field32_t HRTIM_TIME_TIMECR_UPDGAT      = {.msk = 0xF0000000U, .pos = 28};
const field32_t HRTIM_TIME_TIMECR_PREEN       = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIME_TIMECR_DACSYNC     = {.msk = 0x06000000U, .pos = 25};
const field32_t HRTIM_TIME_TIMECR_MSTU        = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIME_TIMECR_TEU         = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_TIME_TIMECR_TDU         = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIME_TIMECR_TCU         = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIME_TIMECR_TBU         = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIME_TIMECR_TXRSTU      = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_TIME_TIMECR_TXREPU      = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIME_TIMECR_SYNCSTRTX   = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIME_TIMECR_SYNCRSTX    = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_TIME_TIMECR_PSHPLL      = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIME_TIMECR_HALF        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_TIME_TIMECR_RETRIG      = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIME_TIMECR_CONT        = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_TIME_TIMECR_CK_PSCX     = {.msk = 0x00000007U, .pos = 0};
const field32_t HRTIM_TIME_TIMEISR_IPPSTAT    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIME_TIMEISR_CPPSTAT    = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIME_TIMEISR_DLYPRT     = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIME_TIMEISR_RST        = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIME_TIMEISR_UPD        = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIME_TIMEISR_REP        = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIME_TIMEICR_DLYPRTC    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIME_TIMEICR_RSTC       = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIME_TIMEICR_UPDC       = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIME_TIMEICR_REPC       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIME_TIMEDIER5_DLYPRTDE = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIME_TIMEDIER5_RSTDE    = {.msk = 0x20000000U, .pos = 29};
const field32_t HRTIM_TIME_TIMEDIER5_SETX2DE  = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_TIME_TIMEDIER5_SET1XDE  = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIME_TIMEDIER5_UPDDE    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_TIME_TIMEDIER5_REPDE    = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIME_TIMEDIER5_DLYPRTIE = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIME_TIMEDIER5_RSTIE    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIME_TIMEDIER5_SETX2IE  = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_TIME_TIMEDIER5_SET1XIE  = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIME_TIMEDIER5_UPDIE    = {.msk = 0x00000040U, .pos = 6};
const field32_t HRTIM_TIME_TIMEDIER5_REPIE    = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIME_CNTER_CNTX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_PERER_PERX         = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_REPER_REPX         = {.msk = 0x000000FFU, .pos = 0};
const field32_t HRTIM_TIME_CMP1ER_CMP1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_CMP1CER_REPX       = {.msk = 0x00FF0000U, .pos = 16};
const field32_t HRTIM_TIME_CMP1CER_CMP1X      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_CMP2ER_CMP2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_CMP3ER_CMP3X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_CMP4ER_CMP4X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_CPT1ER_CPT1X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_CPT2ER_CPT2X       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_TIME_DTER_DTFLKX        = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIME_DTER_DTFSLKX       = {.msk = 0x40000000U, .pos = 30};
const field32_t HRTIM_TIME_DTER_SDTFX         = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIME_DTER_DTFX          = {.msk = 0x01FF0000U, .pos = 16};
const field32_t HRTIM_TIME_DTER_DTRLKX        = {.msk = 0x00008000U, .pos = 15};
const field32_t HRTIM_TIME_DTER_DTRSLKX       = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_TIME_DTER_DTPRSC        = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIME_DTER_SDTRX         = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIME_DTER_DTRX          = {.msk = 0x000001FFU, .pos = 0};
const field32_t HRTIM_TIME_SETExR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIME_SETExR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIME_SETExR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIME_SETExR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIME_SETExR_SST         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIME_RSTExR_UPDATE      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_TIME_RSTExR_MSTPER      = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_TIME_RSTExR_PER         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_TIME_RSTExR_RESYNC      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIME_RSTExR_SRT         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIME_RSTER_MSTPER       = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_TIME_RSTER_UPDT         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIME_CHPER_STRTPW       = {.msk = 0x00000780U, .pos = 7};
const field32_t HRTIM_TIME_CHPER_CHPDTY       = {.msk = 0x00000070U, .pos = 4};
const field32_t HRTIM_TIME_CHPER_CHPFRQ       = {.msk = 0x0000000FU, .pos = 0};
const field32_t HRTIM_TIME_CPTxECR_TD1RST     = {.msk = 0x02000000U, .pos = 25};
const field32_t HRTIM_TIME_CPTxECR_TD1SET     = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_TIME_CPTxECR_TC1RST     = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_TIME_CPTxECR_TC1SET     = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_TIME_CPTxECR_TB1RST     = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_TIME_CPTxECR_TB1SET     = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_TIME_CPTxECR_TA1RST     = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_TIME_CPTxECR_TA1SET     = {.msk = 0x00001000U, .pos = 12};
const field32_t HRTIM_TIME_CPTxECR_UDPCPT     = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_TIME_CPTxECR_SWCPT      = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_TIME_OUTER_DLYPRT       = {.msk = 0x00001C00U, .pos = 10};
const field32_t HRTIM_TIME_OUTER_DLYPRTEN     = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_TIME_OUTER_DTEN         = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_TIME_FLTER_FLTLCK       = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated HRTIM_TIME Register Field Definitions */

field32_t const HRTIM_TIME_TIMECR_DELCMPx[5] = {
  [2] = {.msk = 0x00003000U, .pos = 12},
  [4] = {.msk = 0x0000C000U, .pos = 14},
};

field32_t const HRTIM_TIME_TIMEISR_OxSTAT[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIME_TIMEISR_RSTXx[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIME_TIMEISR_SETXx[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIME_TIMEISR_CPTx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIME_TIMEISR_CMPx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIME_TIMEICR_RSTXxC[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIME_TIMEICR_SETxXC[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIME_TIMEICR_CPTxC[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIME_TIMEICR_CMPxC[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIME_TIMEDIER5_RSTXxDE[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const HRTIM_TIME_TIMEDIER5_CPTxDE[3] = {
  [1] = {.msk = 0x00800000U, .pos = 23},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIME_TIMEDIER5_CMPxDE[5] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x00020000U, .pos = 17},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIME_TIMEDIER5_RSTXxIE[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_TIME_TIMEDIER5_CPTxIE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00000100U, .pos = 8},
};

field32_t const HRTIM_TIME_TIMEDIER5_CMPxIE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_TIME_SETExR_EXTEVNTx[11] = {
  [1]  = {.msk = 0x00200000U, .pos = 21},
  [2]  = {.msk = 0x00400000U, .pos = 22},
  [3]  = {.msk = 0x00800000U, .pos = 23},
  [4]  = {.msk = 0x01000000U, .pos = 24},
  [5]  = {.msk = 0x02000000U, .pos = 25},
  [6]  = {.msk = 0x04000000U, .pos = 26},
  [7]  = {.msk = 0x08000000U, .pos = 27},
  [8]  = {.msk = 0x10000000U, .pos = 28},
  [9]  = {.msk = 0x20000000U, .pos = 29},
  [10] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIME_SETExR_TIMEVNTx[10] = {
  [1]  = {.msk = 0x00001000U, .pos = 12},
  [2]  = {.msk = 0x00002000U, .pos = 13},
  [3]  = {.msk = 0x00004000U, .pos = 14},
  [4]  = {.msk = 0x00008000U, .pos = 15},
  [5]  = {.msk = 0x00010000U, .pos = 16},
  [6]  = {.msk = 0x00020000U, .pos = 17},
  [7]  = {.msk = 0x00040000U, .pos = 18},
  [8]  = {.msk = 0x00080000U, .pos = 19},
  [9]  = {.msk = 0x00100000U, .pos = 20},
};

field32_t const HRTIM_TIME_SETExR_MSTCMPx[5] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00000400U, .pos = 10},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIME_EEFER1_EExFLTR[6] = {
  [1] = {.msk = 0x0000001EU, .pos = 1},
  [2] = {.msk = 0x00000780U, .pos = 7},
  [3] = {.msk = 0x0001E000U, .pos = 13},
  [4] = {.msk = 0x00780000U, .pos = 19},
  [5] = {.msk = 0x1E000000U, .pos = 25},
};

field32_t const HRTIM_TIME_EEFER1_EExLTCH[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00001000U, .pos = 12},
  [4] = {.msk = 0x00040000U, .pos = 18},
  [5] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIME_RSTER_TIMDCMPx[5] = {
  [1] = {.msk = 0x10000000U, .pos = 28},
  [2] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_TIME_RSTER_TIMCCMPx[5] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x04000000U, .pos = 26},
  [4] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIME_RSTER_TIMBCMPx[5] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_TIME_RSTER_TIMACMPx[5] = {
  [1] = {.msk = 0x00080000U, .pos = 19},
  [2] = {.msk = 0x00100000U, .pos = 20},
  [4] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const HRTIM_TIME_CPTxECR_TDCMPx[3] = {
  [1] = {.msk = 0x04000000U, .pos = 26},
  [2] = {.msk = 0x08000000U, .pos = 27},
};

field32_t const HRTIM_TIME_CPTxECR_TCCMPx[3] = {
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIME_CPTxECR_TBCMPx[3] = {
  [1] = {.msk = 0x00040000U, .pos = 18},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIME_CPTxECR_TACMPx[3] = {
  [1] = {.msk = 0x00004000U, .pos = 14},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const HRTIM_TIME_CPTxECR_EXEVxCPT[11] = {
  [1]  = {.msk = 0x00000004U, .pos = 2},
  [2]  = {.msk = 0x00000008U, .pos = 3},
  [3]  = {.msk = 0x00000010U, .pos = 4},
  [4]  = {.msk = 0x00000020U, .pos = 5},
  [5]  = {.msk = 0x00000040U, .pos = 6},
  [6]  = {.msk = 0x00000080U, .pos = 7},
  [7]  = {.msk = 0x00000100U, .pos = 8},
  [8]  = {.msk = 0x00000200U, .pos = 9},
  [9]  = {.msk = 0x00000400U, .pos = 10},
  [10] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const HRTIM_TIME_OUTER_DIDLx[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00800000U, .pos = 23},
};

field32_t const HRTIM_TIME_OUTER_CHPx[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_TIME_OUTER_FAULTx[3] = {
  [1] = {.msk = 0x00000030U, .pos = 4},
  [2] = {.msk = 0x00300000U, .pos = 20},
};

field32_t const HRTIM_TIME_OUTER_IDLESx[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00080000U, .pos = 19},
};

field32_t const HRTIM_TIME_OUTER_IDLEMx[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const HRTIM_TIME_OUTER_POLx[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const HRTIM_TIME_FLTER_FLTxEN[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

/**************************************************************************************************
 * @section HRTIM_COMMON Definitions
 **************************************************************************************************/

/** @subsection HRTIM_COMMON Register Definitions */

rw_reg32_t const HRTIM_COMMON_CR1     = (rw_reg32_t)0x40017780U;
rw_reg32_t const HRTIM_COMMON_CR2     = (rw_reg32_t)0x40017784U;
rw_reg32_t const HRTIM_COMMON_ISR     = (rw_reg32_t)0x40017788U;
rw_reg32_t const HRTIM_COMMON_ICR     = (rw_reg32_t)0x4001778CU;
rw_reg32_t const HRTIM_COMMON_IER     = (rw_reg32_t)0x40017790U;
rw_reg32_t const HRTIM_COMMON_OENR    = (rw_reg32_t)0x40017794U;
rw_reg32_t const HRTIM_COMMON_DISR    = (rw_reg32_t)0x40017798U;
ro_reg32_t const HRTIM_COMMON_ODSR    = (ro_reg32_t)0x4001779CU;
rw_reg32_t const HRTIM_COMMON_BMCR    = (rw_reg32_t)0x400177A0U;
rw_reg32_t const HRTIM_COMMON_BMTRG   = (rw_reg32_t)0x400177A4U;
rw_reg32_t const HRTIM_COMMON_BMCMPR6 = (rw_reg32_t)0x400177A8U;
rw_reg32_t const HRTIM_COMMON_BMPER   = (rw_reg32_t)0x400177ACU;
rw_reg32_t const HRTIM_COMMON_EECR1   = (rw_reg32_t)0x400177B0U;
rw_reg32_t const HRTIM_COMMON_EECR2   = (rw_reg32_t)0x400177B4U;
rw_reg32_t const HRTIM_COMMON_EECR3   = (rw_reg32_t)0x400177B8U;
rw_reg32_t const HRTIM_COMMON_ADC1R   = (rw_reg32_t)0x400177BCU;
rw_reg32_t const HRTIM_COMMON_ADC2R   = (rw_reg32_t)0x400177C0U;
rw_reg32_t const HRTIM_COMMON_ADC3R   = (rw_reg32_t)0x400177C4U;
rw_reg32_t const HRTIM_COMMON_ADC4R   = (rw_reg32_t)0x400177C8U;
rw_reg32_t const HRTIM_COMMON_DLLCR   = (rw_reg32_t)0x400177CCU;
rw_reg32_t const HRTIM_COMMON_FLTINR1 = (rw_reg32_t)0x400177D0U;
rw_reg32_t const HRTIM_COMMON_FLTINR2 = (rw_reg32_t)0x400177D4U;
rw_reg32_t const HRTIM_COMMON_BDMUPDR = (rw_reg32_t)0x400177D8U;
rw_reg32_t const HRTIM_COMMON_BDTXUPR = (rw_reg32_t)0x400177DCU;
rw_reg32_t const HRTIM_COMMON_BDMADR  = (rw_reg32_t)0x400177E0U;

/** @subsection HRTIM_COMMON Register Field Definitions */

const field32_t HRTIM_COMMON_CR1_TEUDIS        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_CR1_TDUDIS        = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_CR1_TCUDIS        = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_COMMON_CR1_TBUDIS        = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_COMMON_CR1_TAUDIS        = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_CR1_MUDIS         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_CR2_TERST         = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_COMMON_CR2_TDRST         = {.msk = 0x00001000U, .pos = 12};
const field32_t HRTIM_COMMON_CR2_TCRST         = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_COMMON_CR2_TBRST         = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_COMMON_CR2_TARST         = {.msk = 0x00000200U, .pos = 9};
const field32_t HRTIM_COMMON_CR2_MRST          = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_COMMON_CR2_TESWU         = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_CR2_TDSWU         = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_CR2_TCSWU         = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_COMMON_CR2_TBSWU         = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_COMMON_CR2_TASWU         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_CR2_MSWU          = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_ISR_BMPER         = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_ISR_DLLRDY        = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_COMMON_ISR_SYSFLT        = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_ICR_BMPERC        = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_ICR_DLLRDYC       = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_COMMON_ICR_SYSFLTC       = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_IER_BMPERIE       = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_IER_DLLRDYIE      = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_COMMON_IER_SYSFLTE       = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_BMCR_BMSTAT       = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_COMMON_BMCR_TEBM         = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_COMMON_BMCR_TDBM         = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_COMMON_BMCR_TCBM         = {.msk = 0x00080000U, .pos = 19};
const field32_t HRTIM_COMMON_BMCR_TBBM         = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_COMMON_BMCR_TABM         = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_BMCR_MTBM         = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_COMMON_BMCR_BMPREN       = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_COMMON_BMCR_BMPRSC       = {.msk = 0x000003C0U, .pos = 6};
const field32_t HRTIM_COMMON_BMCR_BMCLK        = {.msk = 0x0000003CU, .pos = 2};
const field32_t HRTIM_COMMON_BMCR_BMOM         = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_BMCR_BME          = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_BMTRG_OCHPEV      = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_COMMON_BMTRG_TEREP       = {.msk = 0x01000000U, .pos = 24};
const field32_t HRTIM_COMMON_BMTRG_TERST       = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_COMMON_BMTRG_TDREP       = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_COMMON_BMTRG_TDRST       = {.msk = 0x00080000U, .pos = 19};
const field32_t HRTIM_COMMON_BMTRG_TCREP       = {.msk = 0x00010000U, .pos = 16};
const field32_t HRTIM_COMMON_BMTRG_TCRST       = {.msk = 0x00008000U, .pos = 15};
const field32_t HRTIM_COMMON_BMTRG_TBREP       = {.msk = 0x00001000U, .pos = 12};
const field32_t HRTIM_COMMON_BMTRG_TBRST       = {.msk = 0x00000800U, .pos = 11};
const field32_t HRTIM_COMMON_BMTRG_TAREP       = {.msk = 0x00000100U, .pos = 8};
const field32_t HRTIM_COMMON_BMTRG_TARST       = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_COMMON_BMTRG_MSTREP      = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_COMMON_BMTRG_MSTRST      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_BMTRG_SW          = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_BMCMPR6_BMCMP     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_COMMON_BMPER_BMPER       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t HRTIM_COMMON_ADC1R_AD1TEPER    = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_COMMON_ADC1R_AD1TDPER    = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_COMMON_ADC1R_AD1TCPER    = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_COMMON_ADC1R_AD1TBRST    = {.msk = 0x00080000U, .pos = 19};
const field32_t HRTIM_COMMON_ADC1R_AD1TBPER    = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_COMMON_ADC1R_AD1TARST    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_COMMON_ADC1R_AD1TAPER    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_COMMON_ADC1R_AD1MPER     = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_ADC2R_AD2TERST    = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_COMMON_ADC2R_AD2TDRST    = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_COMMON_ADC2R_AD2TDPER    = {.msk = 0x04000000U, .pos = 26};
const field32_t HRTIM_COMMON_ADC2R_AD2TCRST    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_COMMON_ADC2R_AD2TCPER    = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_COMMON_ADC2R_AD2TBPER    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_ADC2R_AD2TAPER    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_COMMON_ADC2R_AD2MPER     = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_ADC3R_AD1TEPER    = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_COMMON_ADC3R_AD1TDPER    = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_COMMON_ADC3R_AD1TCPER    = {.msk = 0x00800000U, .pos = 23};
const field32_t HRTIM_COMMON_ADC3R_AD1TBRST    = {.msk = 0x00080000U, .pos = 19};
const field32_t HRTIM_COMMON_ADC3R_AD1TBPER    = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_COMMON_ADC3R_AD1TARST    = {.msk = 0x00004000U, .pos = 14};
const field32_t HRTIM_COMMON_ADC3R_AD1TAPER    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_COMMON_ADC3R_AD1MPER     = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_ADC4R_AD2TERST    = {.msk = 0x80000000U, .pos = 31};
const field32_t HRTIM_COMMON_ADC4R_AD2TDRST    = {.msk = 0x08000000U, .pos = 27};
const field32_t HRTIM_COMMON_ADC4R_AD2TDPER    = {.msk = 0x04000000U, .pos = 26};
const field32_t HRTIM_COMMON_ADC4R_AD2TCRST    = {.msk = 0x00400000U, .pos = 22};
const field32_t HRTIM_COMMON_ADC4R_AD2TCPER    = {.msk = 0x00200000U, .pos = 21};
const field32_t HRTIM_COMMON_ADC4R_AD2TBPER    = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_ADC4R_AD2TAPER    = {.msk = 0x00002000U, .pos = 13};
const field32_t HRTIM_COMMON_ADC4R_AD2MPER     = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_DLLCR_CALRTE      = {.msk = 0x0000000CU, .pos = 2};
const field32_t HRTIM_COMMON_DLLCR_CALEN       = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_DLLCR_CAL         = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_FLTINR2_FLTSD     = {.msk = 0x03000000U, .pos = 24};
const field32_t HRTIM_COMMON_FLTINR2_FLT5LCK   = {.msk = 0x00000080U, .pos = 7};
const field32_t HRTIM_COMMON_FLTINR2_FLT5F     = {.msk = 0x00000078U, .pos = 3};
const field32_t HRTIM_COMMON_FLTINR2_FLT5SRC   = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_COMMON_FLTINR2_FLT5P     = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_FLTINR2_FLT5E     = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_BDMUPDR_MREP      = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_BDMUPDR_MPER      = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_BDMUPDR_MCNT      = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_COMMON_BDMUPDR_MDIER     = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_COMMON_BDMUPDR_MICR      = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_BDMUPDR_MCR       = {.msk = 0x00000001U, .pos = 0};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXFLTR  = {.msk = 0x00100000U, .pos = 20};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXOUTR  = {.msk = 0x00080000U, .pos = 19};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXCHPR  = {.msk = 0x00040000U, .pos = 18};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXRSTR  = {.msk = 0x00020000U, .pos = 17};
const field32_t HRTIM_COMMON_BDTXUPR_TIMX_DTXR = {.msk = 0x00000400U, .pos = 10};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXREP   = {.msk = 0x00000020U, .pos = 5};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXPER   = {.msk = 0x00000010U, .pos = 4};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXCNT   = {.msk = 0x00000008U, .pos = 3};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXDIER  = {.msk = 0x00000004U, .pos = 2};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXICR   = {.msk = 0x00000002U, .pos = 1};
const field32_t HRTIM_COMMON_BDTXUPR_TIMXCR    = {.msk = 0x00000001U, .pos = 0};

/** @subsection Enumerated HRTIM_COMMON Register Field Definitions */

field32_t const HRTIM_COMMON_CR1_ADxUSRC[5] = {
  [1] = {.msk = 0x00070000U, .pos = 16},
  [2] = {.msk = 0x00380000U, .pos = 19},
  [3] = {.msk = 0x01C00000U, .pos = 22},
  [4] = {.msk = 0x0E000000U, .pos = 25},
};

field32_t const HRTIM_COMMON_ISR_FLTx[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const HRTIM_COMMON_ICR_FLTxC[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const HRTIM_COMMON_IER_FLTxIE[6] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
  [5] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const HRTIM_COMMON_OENR_TExOEN[3] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const HRTIM_COMMON_OENR_TDxOEN[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const HRTIM_COMMON_OENR_TCxOEN[3] = {
  [1] = {.msk = 0x00000010U, .pos = 4},
  [2] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const HRTIM_COMMON_OENR_TBxOEN[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_COMMON_OENR_TAxOEN[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const HRTIM_COMMON_DISR_TExODIS[3] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const HRTIM_COMMON_DISR_TDxODIS[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const HRTIM_COMMON_DISR_TCxODIS[3] = {
  [1] = {.msk = 0x00000010U, .pos = 4},
  [2] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const HRTIM_COMMON_DISR_TBxODIS[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_COMMON_DISR_TAxODIS[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const HRTIM_COMMON_ODSR_TExODS[3] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const HRTIM_COMMON_ODSR_TDxODS[3] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const HRTIM_COMMON_ODSR_TCxODS[3] = {
  [1] = {.msk = 0x00000010U, .pos = 4},
  [2] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const HRTIM_COMMON_ODSR_TBxODS[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_COMMON_ODSR_TAxODS[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const HRTIM_COMMON_BMTRG_TECMPx[3] = {
  [1] = {.msk = 0x02000000U, .pos = 25},
  [2] = {.msk = 0x04000000U, .pos = 26},
};

field32_t const HRTIM_COMMON_BMTRG_TDCMPx[3] = {
  [1] = {.msk = 0x00200000U, .pos = 21},
  [2] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_COMMON_BMTRG_TCCMPx[3] = {
  [1] = {.msk = 0x00020000U, .pos = 17},
  [2] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const HRTIM_COMMON_BMTRG_TBCMPx[3] = {
  [1] = {.msk = 0x00002000U, .pos = 13},
  [2] = {.msk = 0x00004000U, .pos = 14},
};

field32_t const HRTIM_COMMON_BMTRG_TACMPx[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const HRTIM_COMMON_BMTRG_MSTCMPx[5] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000020U, .pos = 5},
  [4] = {.msk = 0x00000040U, .pos = 6},
};

field32_t const HRTIM_COMMON_EECR1_EExFAST[6] = {
  [1] = {.msk = 0x00000020U, .pos = 5},
  [2] = {.msk = 0x00000800U, .pos = 11},
  [3] = {.msk = 0x00020000U, .pos = 17},
  [4] = {.msk = 0x00800000U, .pos = 23},
  [5] = {.msk = 0x20000000U, .pos = 29},
};

field32_t const HRTIM_COMMON_EECR1_EExSNS[6] = {
  [1] = {.msk = 0x00000018U, .pos = 3},
  [2] = {.msk = 0x00000600U, .pos = 9},
  [3] = {.msk = 0x00018000U, .pos = 15},
  [4] = {.msk = 0x00600000U, .pos = 21},
  [5] = {.msk = 0x18000000U, .pos = 27},
};

field32_t const HRTIM_COMMON_EECR1_EExPOL[6] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000100U, .pos = 8},
  [3] = {.msk = 0x00004000U, .pos = 14},
  [4] = {.msk = 0x00100000U, .pos = 20},
  [5] = {.msk = 0x04000000U, .pos = 26},
};

field32_t const HRTIM_COMMON_EECR1_EExSRC[6] = {
  [1] = {.msk = 0x00000003U, .pos = 0},
  [2] = {.msk = 0x000000C0U, .pos = 6},
  [3] = {.msk = 0x00003000U, .pos = 12},
  [4] = {.msk = 0x000C0000U, .pos = 18},
  [5] = {.msk = 0x03000000U, .pos = 24},
};

field32_t const HRTIM_COMMON_ADC1R_AD1TECx[5] = {
  [2] = {.msk = 0x10000000U, .pos = 28},
  [3] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_COMMON_ADC1R_AD1TDCx[5] = {
  [2] = {.msk = 0x01000000U, .pos = 24},
  [3] = {.msk = 0x02000000U, .pos = 25},
  [4] = {.msk = 0x04000000U, .pos = 26},
};

field32_t const HRTIM_COMMON_ADC1R_AD1TCCx[5] = {
  [2] = {.msk = 0x00100000U, .pos = 20},
  [3] = {.msk = 0x00200000U, .pos = 21},
  [4] = {.msk = 0x00400000U, .pos = 22},
};

field32_t const HRTIM_COMMON_ADC1R_AD1TBCx[5] = {
  [2] = {.msk = 0x00008000U, .pos = 15},
  [3] = {.msk = 0x00010000U, .pos = 16},
  [4] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const HRTIM_COMMON_ADC1R_AD1TACx[5] = {
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_COMMON_ADC1R_AD1EEVx[6] = {
  [1] = {.msk = 0x00000020U, .pos = 5},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00000080U, .pos = 7},
  [4] = {.msk = 0x00000100U, .pos = 8},
  [5] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const HRTIM_COMMON_ADC1R_AD1MCx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_COMMON_ADC2R_AD2TECx[5] = {
  [2] = {.msk = 0x10000000U, .pos = 28},
  [3] = {.msk = 0x20000000U, .pos = 29},
  [4] = {.msk = 0x40000000U, .pos = 30},
};

field32_t const HRTIM_COMMON_ADC2R_AD2TDCx[5] = {
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
  [4] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const HRTIM_COMMON_ADC2R_AD2TCCx[5] = {
  [2] = {.msk = 0x00040000U, .pos = 18},
  [3] = {.msk = 0x00080000U, .pos = 19},
  [4] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const HRTIM_COMMON_ADC2R_AD2TBCx[5] = {
  [2] = {.msk = 0x00004000U, .pos = 14},
  [3] = {.msk = 0x00008000U, .pos = 15},
  [4] = {.msk = 0x00010000U, .pos = 16},
};

field32_t const HRTIM_COMMON_ADC2R_AD2TACx[5] = {
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const HRTIM_COMMON_ADC2R_AD2EEVx[11] = {
  [6]  = {.msk = 0x00000020U, .pos = 5},
  [7]  = {.msk = 0x00000040U, .pos = 6},
  [8]  = {.msk = 0x00000080U, .pos = 7},
  [9]  = {.msk = 0x00000100U, .pos = 8},
  [10] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const HRTIM_COMMON_ADC2R_AD2MCx[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000002U, .pos = 1},
  [3] = {.msk = 0x00000004U, .pos = 2},
  [4] = {.msk = 0x00000008U, .pos = 3},
};

field32_t const HRTIM_COMMON_FLTINR1_FLTxLCK[5] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00008000U, .pos = 15},
  [3] = {.msk = 0x00800000U, .pos = 23},
  [4] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const HRTIM_COMMON_FLTINR1_FLTxF[5] = {
  [1] = {.msk = 0x00000078U, .pos = 3},
  [2] = {.msk = 0x00007800U, .pos = 11},
  [3] = {.msk = 0x00780000U, .pos = 19},
  [4] = {.msk = 0x78000000U, .pos = 27},
};

field32_t const HRTIM_COMMON_FLTINR1_FLTxSRC[5] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00040000U, .pos = 18},
  [4] = {.msk = 0x04000000U, .pos = 26},
};

field32_t const HRTIM_COMMON_FLTINR1_FLTxP[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000200U, .pos = 9},
  [3] = {.msk = 0x00020000U, .pos = 17},
  [4] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const HRTIM_COMMON_FLTINR1_FLTxE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000100U, .pos = 8},
  [3] = {.msk = 0x00010000U, .pos = 16},
  [4] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const HRTIM_COMMON_BDMUPDR_MCMPx[5] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000100U, .pos = 8},
  [4] = {.msk = 0x00000200U, .pos = 9},
};

field32_t const HRTIM_COMMON_BDTXUPR_TIMXEEFRx[3] = {
  [1] = {.msk = 0x00008000U, .pos = 15},
  [2] = {.msk = 0x00010000U, .pos = 16},
};

field32_t const HRTIM_COMMON_BDTXUPR_TIMXRSTxR[3] = {
  [1] = {.msk = 0x00001000U, .pos = 12},
  [2] = {.msk = 0x00004000U, .pos = 14},
};

field32_t const HRTIM_COMMON_BDTXUPR_TIMXSETxR[3] = {
  [1] = {.msk = 0x00000800U, .pos = 11},
  [2] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const HRTIM_COMMON_BDTXUPR_TIMXCMPx[5] = {
  [1] = {.msk = 0x00000040U, .pos = 6},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000100U, .pos = 8},
  [4] = {.msk = 0x00000200U, .pos = 9},
};

/**************************************************************************************************
 * @section DFSDM Definitions
 **************************************************************************************************/

/** @subsection Enumerated DFSDM Register Definitions */

rw_reg32_t const DFSDM_DFSDM_CHCFGxR1[8] = {
  [0] = (rw_reg32_t)0x40017000U,
  [1] = (rw_reg32_t)0x40017004U,
  [2] = (rw_reg32_t)0x40017008U,
  [3] = (rw_reg32_t)0x4001700CU,
  [4] = (rw_reg32_t)0x40017010U,
  [5] = (rw_reg32_t)0x40017014U,
  [6] = (rw_reg32_t)0x40017018U,
  [7] = (rw_reg32_t)0x4001701CU,
};

rw_reg32_t const DFSDM_DFSDM_CHCFGxR2[8] = {
  [0] = (rw_reg32_t)0x40017020U,
  [1] = (rw_reg32_t)0x40017024U,
  [2] = (rw_reg32_t)0x40017028U,
  [3] = (rw_reg32_t)0x4001702CU,
  [4] = (rw_reg32_t)0x40017030U,
  [5] = (rw_reg32_t)0x40017034U,
  [6] = (rw_reg32_t)0x40017038U,
  [7] = (rw_reg32_t)0x4001703CU,
};

rw_reg32_t const DFSDM_DFSDM_AWSCDxR[8] = {
  [0] = (rw_reg32_t)0x40017040U,
  [1] = (rw_reg32_t)0x40017044U,
  [2] = (rw_reg32_t)0x40017048U,
  [3] = (rw_reg32_t)0x4001704CU,
  [4] = (rw_reg32_t)0x40017050U,
  [5] = (rw_reg32_t)0x40017054U,
  [6] = (rw_reg32_t)0x40017058U,
  [7] = (rw_reg32_t)0x4001705CU,
};

ro_reg32_t const DFSDM_DFSDM_CHWDATxR[8] = {
  [0] = (ro_reg32_t)0x40017060U,
  [1] = (ro_reg32_t)0x40017064U,
  [2] = (ro_reg32_t)0x40017068U,
  [3] = (ro_reg32_t)0x4001706CU,
  [4] = (ro_reg32_t)0x40017070U,
  [5] = (ro_reg32_t)0x40017074U,
  [6] = (ro_reg32_t)0x40017078U,
  [7] = (ro_reg32_t)0x4001707CU,
};

rw_reg32_t const DFSDM_DFSDM_CHDATINxR[8] = {
  [0] = (rw_reg32_t)0x40017080U,
  [1] = (rw_reg32_t)0x40017084U,
  [2] = (rw_reg32_t)0x40017088U,
  [3] = (rw_reg32_t)0x4001708CU,
  [4] = (rw_reg32_t)0x40017090U,
  [5] = (rw_reg32_t)0x40017094U,
  [6] = (rw_reg32_t)0x40017098U,
  [7] = (rw_reg32_t)0x4001709CU,
};

rw_reg32_t const DFSDM_DFSDMx_CR1[4] = {
  [0] = (rw_reg32_t)0x400170A0U,
  [1] = (rw_reg32_t)0x400170A4U,
  [2] = (rw_reg32_t)0x400170A8U,
  [3] = (rw_reg32_t)0x400170ACU,
};

rw_reg32_t const DFSDM_DFSDMx_CR2[4] = {
  [0] = (rw_reg32_t)0x400170B0U,
  [1] = (rw_reg32_t)0x400170B4U,
  [2] = (rw_reg32_t)0x400170B8U,
  [3] = (rw_reg32_t)0x400170BCU,
};

ro_reg32_t const DFSDM_DFSDMx_ISR[4] = {
  [0] = (ro_reg32_t)0x400170C0U,
  [1] = (ro_reg32_t)0x400170C4U,
  [2] = (ro_reg32_t)0x400170C8U,
  [3] = (ro_reg32_t)0x400170CCU,
};

rw_reg32_t const DFSDM_DFSDMx_ICR[4] = {
  [0] = (rw_reg32_t)0x400170D0U,
  [1] = (rw_reg32_t)0x400170D4U,
  [2] = (rw_reg32_t)0x400170D8U,
  [3] = (rw_reg32_t)0x400170DCU,
};

rw_reg32_t const DFSDM_DFSDMx_JCHGR[4] = {
  [0] = (rw_reg32_t)0x400170E0U,
  [1] = (rw_reg32_t)0x400170E4U,
  [2] = (rw_reg32_t)0x400170E8U,
  [3] = (rw_reg32_t)0x400170ECU,
};

rw_reg32_t const DFSDM_DFSDMx_FCR[4] = {
  [0] = (rw_reg32_t)0x400170F0U,
  [1] = (rw_reg32_t)0x400170F4U,
  [2] = (rw_reg32_t)0x400170F8U,
  [3] = (rw_reg32_t)0x400170FCU,
};

ro_reg32_t const DFSDM_DFSDMx_JDATAR[4] = {
  [0] = (ro_reg32_t)0x40017100U,
  [1] = (ro_reg32_t)0x40017104U,
  [2] = (ro_reg32_t)0x40017108U,
  [3] = (ro_reg32_t)0x4001710CU,
};

ro_reg32_t const DFSDM_DFSDMx_RDATAR[4] = {
  [0] = (ro_reg32_t)0x40017110U,
  [1] = (ro_reg32_t)0x40017114U,
  [2] = (ro_reg32_t)0x40017118U,
  [3] = (ro_reg32_t)0x4001711CU,
};

rw_reg32_t const DFSDM_DFSDMx_AWHTR[4] = {
  [0] = (rw_reg32_t)0x40017120U,
  [1] = (rw_reg32_t)0x40017124U,
  [2] = (rw_reg32_t)0x40017128U,
  [3] = (rw_reg32_t)0x4001712CU,
};

rw_reg32_t const DFSDM_DFSDMx_AWLTR[4] = {
  [0] = (rw_reg32_t)0x40017130U,
  [1] = (rw_reg32_t)0x40017134U,
  [2] = (rw_reg32_t)0x40017138U,
  [3] = (rw_reg32_t)0x4001713CU,
};

ro_reg32_t const DFSDM_DFSDMx_AWSR[4] = {
  [0] = (ro_reg32_t)0x40017140U,
  [1] = (ro_reg32_t)0x40017144U,
  [2] = (ro_reg32_t)0x40017148U,
  [3] = (ro_reg32_t)0x4001714CU,
};

rw_reg32_t const DFSDM_DFSDMx_AWCFR[4] = {
  [0] = (rw_reg32_t)0x40017150U,
  [1] = (rw_reg32_t)0x40017154U,
  [2] = (rw_reg32_t)0x40017158U,
  [3] = (rw_reg32_t)0x4001715CU,
};

ro_reg32_t const DFSDM_DFSDMx_EXMAX[4] = {
  [0] = (ro_reg32_t)0x40017160U,
  [1] = (ro_reg32_t)0x40017164U,
  [2] = (ro_reg32_t)0x40017168U,
  [3] = (ro_reg32_t)0x4001716CU,
};

ro_reg32_t const DFSDM_DFSDMx_EXMIN[4] = {
  [0] = (ro_reg32_t)0x40017170U,
  [1] = (ro_reg32_t)0x40017174U,
  [2] = (ro_reg32_t)0x40017178U,
  [3] = (ro_reg32_t)0x4001717CU,
};

ro_reg32_t const DFSDM_DFSDMx_CNVTIMR[4] = {
  [0] = (ro_reg32_t)0x40017180U,
  [1] = (ro_reg32_t)0x40017184U,
  [2] = (ro_reg32_t)0x40017188U,
  [3] = (ro_reg32_t)0x4001718CU,
};

/** @subsection DFSDM Register Field Definitions */

const field32_t DFSDM_DFSDM_CHCFGxR1_SITP     = {.msk = 0x00000003U, .pos = 0};
const field32_t DFSDM_DFSDM_CHCFGxR1_SPICKSEL = {.msk = 0x0000000CU, .pos = 2};
const field32_t DFSDM_DFSDM_CHCFGxR1_SCDEN    = {.msk = 0x00000020U, .pos = 5};
const field32_t DFSDM_DFSDM_CHCFGxR1_CKABEN   = {.msk = 0x00000040U, .pos = 6};
const field32_t DFSDM_DFSDM_CHCFGxR1_CHEN     = {.msk = 0x00000080U, .pos = 7};
const field32_t DFSDM_DFSDM_CHCFGxR1_CHINSEL  = {.msk = 0x00000100U, .pos = 8};
const field32_t DFSDM_DFSDM_CHCFGxR1_DATMPX   = {.msk = 0x00003000U, .pos = 12};
const field32_t DFSDM_DFSDM_CHCFGxR1_DATPACK  = {.msk = 0x0000C000U, .pos = 14};
const field32_t DFSDM_DFSDM_CHCFGxR1_CKOUTDIV = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DFSDM_DFSDM_CHCFGxR1_CKOUTSRC = {.msk = 0x40000000U, .pos = 30};
const field32_t DFSDM_DFSDM_CHCFGxR1_DFSDMEN  = {.msk = 0x80000000U, .pos = 31};
const field32_t DFSDM_DFSDM_CHCFGxR2_DTRBS    = {.msk = 0x000000F8U, .pos = 3};
const field32_t DFSDM_DFSDM_CHCFGxR2_OFFSET   = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDM_AWSCDxR_SCDT      = {.msk = 0x000000FFU, .pos = 0};
const field32_t DFSDM_DFSDM_AWSCDxR_BKSCD     = {.msk = 0x0000F000U, .pos = 12};
const field32_t DFSDM_DFSDM_AWSCDxR_AWFOSR    = {.msk = 0x001F0000U, .pos = 16};
const field32_t DFSDM_DFSDM_AWSCDxR_AWFORD    = {.msk = 0x00C00000U, .pos = 22};
const field32_t DFSDM_DFSDM_CHWDATxR_WDATA    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t DFSDM_DFSDMx_CR1_DFEN         = {.msk = 0x00000001U, .pos = 0};
const field32_t DFSDM_DFSDMx_CR1_JSWSTART     = {.msk = 0x00000002U, .pos = 1};
const field32_t DFSDM_DFSDMx_CR1_JSYNC        = {.msk = 0x00000008U, .pos = 3};
const field32_t DFSDM_DFSDMx_CR1_JSCAN        = {.msk = 0x00000010U, .pos = 4};
const field32_t DFSDM_DFSDMx_CR1_JDMAEN       = {.msk = 0x00000020U, .pos = 5};
const field32_t DFSDM_DFSDMx_CR1_JEXTSEL      = {.msk = 0x00001F00U, .pos = 8};
const field32_t DFSDM_DFSDMx_CR1_JEXTEN       = {.msk = 0x00006000U, .pos = 13};
const field32_t DFSDM_DFSDMx_CR1_RSWSTART     = {.msk = 0x00020000U, .pos = 17};
const field32_t DFSDM_DFSDMx_CR1_RCONT        = {.msk = 0x00040000U, .pos = 18};
const field32_t DFSDM_DFSDMx_CR1_RSYNC        = {.msk = 0x00080000U, .pos = 19};
const field32_t DFSDM_DFSDMx_CR1_RDMAEN       = {.msk = 0x00200000U, .pos = 21};
const field32_t DFSDM_DFSDMx_CR1_RCH          = {.msk = 0x07000000U, .pos = 24};
const field32_t DFSDM_DFSDMx_CR1_FAST         = {.msk = 0x20000000U, .pos = 29};
const field32_t DFSDM_DFSDMx_CR1_AWFSEL       = {.msk = 0x40000000U, .pos = 30};
const field32_t DFSDM_DFSDMx_CR2_JEOCIE       = {.msk = 0x00000001U, .pos = 0};
const field32_t DFSDM_DFSDMx_CR2_REOCIE       = {.msk = 0x00000002U, .pos = 1};
const field32_t DFSDM_DFSDMx_CR2_JOVRIE       = {.msk = 0x00000004U, .pos = 2};
const field32_t DFSDM_DFSDMx_CR2_ROVRIE       = {.msk = 0x00000008U, .pos = 3};
const field32_t DFSDM_DFSDMx_CR2_AWDIE        = {.msk = 0x00000010U, .pos = 4};
const field32_t DFSDM_DFSDMx_CR2_SCDIE        = {.msk = 0x00000020U, .pos = 5};
const field32_t DFSDM_DFSDMx_CR2_CKABIE       = {.msk = 0x00000040U, .pos = 6};
const field32_t DFSDM_DFSDMx_CR2_EXCH         = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_CR2_AWDCH        = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DFSDM_DFSDMx_ISR_JEOCF        = {.msk = 0x00000001U, .pos = 0};
const field32_t DFSDM_DFSDMx_ISR_REOCF        = {.msk = 0x00000002U, .pos = 1};
const field32_t DFSDM_DFSDMx_ISR_JOVRF        = {.msk = 0x00000004U, .pos = 2};
const field32_t DFSDM_DFSDMx_ISR_ROVRF        = {.msk = 0x00000008U, .pos = 3};
const field32_t DFSDM_DFSDMx_ISR_AWDF         = {.msk = 0x00000010U, .pos = 4};
const field32_t DFSDM_DFSDMx_ISR_JCIP         = {.msk = 0x00002000U, .pos = 13};
const field32_t DFSDM_DFSDMx_ISR_RCIP         = {.msk = 0x00004000U, .pos = 14};
const field32_t DFSDM_DFSDMx_ISR_CKABF        = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DFSDM_DFSDMx_ISR_SCDF         = {.msk = 0xFF000000U, .pos = 24};
const field32_t DFSDM_DFSDMx_ICR_CLRJOVRF     = {.msk = 0x00000004U, .pos = 2};
const field32_t DFSDM_DFSDMx_ICR_CLRROVRF     = {.msk = 0x00000008U, .pos = 3};
const field32_t DFSDM_DFSDMx_ICR_CLRCKABF     = {.msk = 0x00FF0000U, .pos = 16};
const field32_t DFSDM_DFSDMx_ICR_CLRSCDF      = {.msk = 0xFF000000U, .pos = 24};
const field32_t DFSDM_DFSDMx_JCHGR_JCHG       = {.msk = 0x000000FFU, .pos = 0};
const field32_t DFSDM_DFSDMx_FCR_IOSR         = {.msk = 0x000000FFU, .pos = 0};
const field32_t DFSDM_DFSDMx_FCR_FOSR         = {.msk = 0x03FF0000U, .pos = 16};
const field32_t DFSDM_DFSDMx_FCR_FORD         = {.msk = 0xE0000000U, .pos = 29};
const field32_t DFSDM_DFSDMx_JDATAR_JDATACH   = {.msk = 0x00000007U, .pos = 0};
const field32_t DFSDM_DFSDMx_JDATAR_JDATA     = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_RDATAR_RDATACH   = {.msk = 0x00000007U, .pos = 0};
const field32_t DFSDM_DFSDMx_RDATAR_RPEND     = {.msk = 0x00000010U, .pos = 4};
const field32_t DFSDM_DFSDMx_RDATAR_RDATA     = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_AWHTR_BKAWH      = {.msk = 0x0000000FU, .pos = 0};
const field32_t DFSDM_DFSDMx_AWHTR_AWHT       = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_AWLTR_BKAWL      = {.msk = 0x0000000FU, .pos = 0};
const field32_t DFSDM_DFSDMx_AWLTR_AWLT       = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_AWSR_AWLTF       = {.msk = 0x000000FFU, .pos = 0};
const field32_t DFSDM_DFSDMx_AWSR_AWHTF       = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_AWCFR_CLRAWLTF   = {.msk = 0x000000FFU, .pos = 0};
const field32_t DFSDM_DFSDMx_AWCFR_CLRAWHTF   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_EXMAX_EXMAXCH    = {.msk = 0x00000007U, .pos = 0};
const field32_t DFSDM_DFSDMx_EXMAX_EXMAX      = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_EXMIN_EXMINCH    = {.msk = 0x00000007U, .pos = 0};
const field32_t DFSDM_DFSDMx_EXMIN_EXMIN      = {.msk = 0xFFFFFF00U, .pos = 8};
const field32_t DFSDM_DFSDMx_CNVTIMR_CNVCNT   = {.msk = 0xFFFFFFF0U, .pos = 4};

/** @subsection Enumerated DFSDM Register Field Definitions */

field32_t const DFSDM_DFSDM_CHDATINxR_INDATx[2] = {
  [0] = {.msk = 0x0000FFFFU, .pos = 0},
  [1] = {.msk = 0xFFFF0000U, .pos = 16},
};

/**************************************************************************************************
 * @section G2_TIM1x Definitions
 **************************************************************************************************/

/** @subsection Enumerated G2_TIM1x Register Definitions */

rw_reg32_t const G2_TIM1x_CR1[8] = {
  [6] = (rw_reg32_t)0x40014400U,
  [7] = (rw_reg32_t)0x40014800U,
};

rw_reg32_t const G2_TIM1x_CR2[8] = {
  [6] = (rw_reg32_t)0x40014404U,
  [7] = (rw_reg32_t)0x40014804U,
};

rw_reg32_t const G2_TIM1x_DIER[8] = {
  [6] = (rw_reg32_t)0x4001440CU,
  [7] = (rw_reg32_t)0x4001480CU,
};

rw_reg32_t const G2_TIM1x_SR[8] = {
  [6] = (rw_reg32_t)0x40014410U,
  [7] = (rw_reg32_t)0x40014810U,
};

rw_reg32_t const G2_TIM1x_EGR[8] = {
  [6] = (rw_reg32_t)0x40014414U,
  [7] = (rw_reg32_t)0x40014814U,
};

rw_reg32_t const G2_TIM1x_CCMR1_OUTPUT[8] = {
  [6] = (rw_reg32_t)0x40014418U,
  [7] = (rw_reg32_t)0x40014818U,
};

rw_reg32_t const G2_TIM1x_CCMR1_INPUT[8] = {
  [6] = (rw_reg32_t)0x40014418U,
  [7] = (rw_reg32_t)0x40014818U,
};

rw_reg32_t const G2_TIM1x_CCER[8] = {
  [6] = (rw_reg32_t)0x40014420U,
  [7] = (rw_reg32_t)0x40014820U,
};

rw_reg32_t const G2_TIM1x_CNT[8] = {
  [6] = (rw_reg32_t)0x40014424U,
  [7] = (rw_reg32_t)0x40014824U,
};

rw_reg32_t const G2_TIM1x_PSC[8] = {
  [6] = (rw_reg32_t)0x40014428U,
  [7] = (rw_reg32_t)0x40014828U,
};

rw_reg32_t const G2_TIM1x_ARR[8] = {
  [6] = (rw_reg32_t)0x4001442CU,
  [7] = (rw_reg32_t)0x4001482CU,
};

rw_reg32_t const G2_TIM1x_RCR[8] = {
  [6] = (rw_reg32_t)0x40014430U,
  [7] = (rw_reg32_t)0x40014830U,
};

rw_reg32_t const G2_TIM1x_CCR1[8] = {
  [6] = (rw_reg32_t)0x40014434U,
  [7] = (rw_reg32_t)0x40014834U,
};

rw_reg32_t const G2_TIM1x_BDTR[8] = {
  [6] = (rw_reg32_t)0x40014444U,
  [7] = (rw_reg32_t)0x40014844U,
};

rw_reg32_t const G2_TIM1x_DCR[8] = {
  [6] = (rw_reg32_t)0x40014448U,
  [7] = (rw_reg32_t)0x40014848U,
};

rw_reg32_t const G2_TIM1x_DMAR[8] = {
  [6] = (rw_reg32_t)0x4001444CU,
  [7] = (rw_reg32_t)0x4001484CU,
};

rw_reg32_t const G2_TIM1x_TIM16_AF1[8] = {
  [6] = (rw_reg32_t)0x40014460U,
  [7] = (rw_reg32_t)0x40014860U,
};

rw_reg32_t const G2_TIM1x_TIM16_TISEL[8] = {
  [6] = (rw_reg32_t)0x40014468U,
  [7] = (rw_reg32_t)0x40014868U,
};

/** @subsection G2_TIM1x Register Field Definitions */

const field32_t G2_TIM1x_CR1_CEN             = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_CR1_UDIS            = {.msk = 0x00000002U, .pos = 1};
const field32_t G2_TIM1x_CR1_URS             = {.msk = 0x00000004U, .pos = 2};
const field32_t G2_TIM1x_CR1_OPM             = {.msk = 0x00000008U, .pos = 3};
const field32_t G2_TIM1x_CR1_ARPE            = {.msk = 0x00000080U, .pos = 7};
const field32_t G2_TIM1x_CR1_CKD             = {.msk = 0x00000300U, .pos = 8};
const field32_t G2_TIM1x_CR1_UIFREMAP        = {.msk = 0x00000800U, .pos = 11};
const field32_t G2_TIM1x_CR2_OIS1N           = {.msk = 0x00000200U, .pos = 9};
const field32_t G2_TIM1x_CR2_OIS1            = {.msk = 0x00000100U, .pos = 8};
const field32_t G2_TIM1x_CR2_CCDS            = {.msk = 0x00000008U, .pos = 3};
const field32_t G2_TIM1x_CR2_CCUS            = {.msk = 0x00000004U, .pos = 2};
const field32_t G2_TIM1x_CR2_CCPC            = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_DIER_UIE            = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_DIER_CC1IE          = {.msk = 0x00000002U, .pos = 1};
const field32_t G2_TIM1x_DIER_COMIE          = {.msk = 0x00000020U, .pos = 5};
const field32_t G2_TIM1x_DIER_BIE            = {.msk = 0x00000080U, .pos = 7};
const field32_t G2_TIM1x_DIER_UDE            = {.msk = 0x00000100U, .pos = 8};
const field32_t G2_TIM1x_DIER_CC1DE          = {.msk = 0x00000200U, .pos = 9};
const field32_t G2_TIM1x_DIER_COMDE          = {.msk = 0x00002000U, .pos = 13};
const field32_t G2_TIM1x_SR_CC1OF            = {.msk = 0x00000200U, .pos = 9};
const field32_t G2_TIM1x_SR_BIF              = {.msk = 0x00000080U, .pos = 7};
const field32_t G2_TIM1x_SR_COMIF            = {.msk = 0x00000020U, .pos = 5};
const field32_t G2_TIM1x_SR_CC1IF            = {.msk = 0x00000002U, .pos = 1};
const field32_t G2_TIM1x_SR_UIF              = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_EGR_BG              = {.msk = 0x00000080U, .pos = 7};
const field32_t G2_TIM1x_EGR_COMG            = {.msk = 0x00000020U, .pos = 5};
const field32_t G2_TIM1x_EGR_CC1G            = {.msk = 0x00000002U, .pos = 1};
const field32_t G2_TIM1x_EGR_UG              = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_CCMR1_OUTPUT_CC1S   = {.msk = 0x00000003U, .pos = 0};
const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1FE  = {.msk = 0x00000004U, .pos = 2};
const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1PE  = {.msk = 0x00000008U, .pos = 3};
const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1M   = {.msk = 0x00000070U, .pos = 4};
const field32_t G2_TIM1x_CCMR1_OUTPUT_OC1M_3 = {.msk = 0x00010000U, .pos = 16};
const field32_t G2_TIM1x_CCMR1_INPUT_IC1F    = {.msk = 0x000000F0U, .pos = 4};
const field32_t G2_TIM1x_CCMR1_INPUT_IC1PSC  = {.msk = 0x0000000CU, .pos = 2};
const field32_t G2_TIM1x_CCMR1_INPUT_CC1S    = {.msk = 0x00000003U, .pos = 0};
const field32_t G2_TIM1x_CCER_CC1NP          = {.msk = 0x00000008U, .pos = 3};
const field32_t G2_TIM1x_CCER_CC1NE          = {.msk = 0x00000004U, .pos = 2};
const field32_t G2_TIM1x_CCER_CC1P           = {.msk = 0x00000002U, .pos = 1};
const field32_t G2_TIM1x_CCER_CC1E           = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_CNT_CNT             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G2_TIM1x_CNT_UIFCPY          = {.msk = 0x80000000U, .pos = 31};
const field32_t G2_TIM1x_PSC_PSC             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G2_TIM1x_ARR_ARR             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G2_TIM1x_RCR_REP             = {.msk = 0x000000FFU, .pos = 0};
const field32_t G2_TIM1x_CCR1_CCR1           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G2_TIM1x_BDTR_DTG            = {.msk = 0x000000FFU, .pos = 0};
const field32_t G2_TIM1x_BDTR_LOCK           = {.msk = 0x00000300U, .pos = 8};
const field32_t G2_TIM1x_BDTR_OSSI           = {.msk = 0x00000400U, .pos = 10};
const field32_t G2_TIM1x_BDTR_OSSR           = {.msk = 0x00000800U, .pos = 11};
const field32_t G2_TIM1x_BDTR_BKE            = {.msk = 0x00001000U, .pos = 12};
const field32_t G2_TIM1x_BDTR_BKP            = {.msk = 0x00002000U, .pos = 13};
const field32_t G2_TIM1x_BDTR_AOE            = {.msk = 0x00004000U, .pos = 14};
const field32_t G2_TIM1x_BDTR_MOE            = {.msk = 0x00008000U, .pos = 15};
const field32_t G2_TIM1x_BDTR_BKF            = {.msk = 0x000F0000U, .pos = 16};
const field32_t G2_TIM1x_DCR_DBL             = {.msk = 0x00001F00U, .pos = 8};
const field32_t G2_TIM1x_DCR_DBA             = {.msk = 0x0000001FU, .pos = 0};
const field32_t G2_TIM1x_DMAR_DMAB           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G2_TIM1x_TIM16_AF1_BKINE     = {.msk = 0x00000001U, .pos = 0};
const field32_t G2_TIM1x_TIM16_AF1_BKDFBK1E  = {.msk = 0x00000100U, .pos = 8};
const field32_t G2_TIM1x_TIM16_AF1_BKINP     = {.msk = 0x00000200U, .pos = 9};
const field32_t G2_TIM1x_TIM16_TISEL_TI1SEL  = {.msk = 0x0000000FU, .pos = 0};

/** @subsection Enumerated G2_TIM1x Register Field Definitions */

field32_t const G2_TIM1x_TIM16_AF1_BKCMPxE[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const G2_TIM1x_TIM16_AF1_BKCMPxP[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

/**************************************************************************************************
 * @section TIM15 Definitions
 **************************************************************************************************/

/** @subsection TIM15 Register Definitions */

rw_reg32_t const TIM15_CR1          = (rw_reg32_t)0x40014000U;
rw_reg32_t const TIM15_CR2          = (rw_reg32_t)0x40014004U;
rw_reg32_t const TIM15_SMCR         = (rw_reg32_t)0x40014008U;
rw_reg32_t const TIM15_DIER         = (rw_reg32_t)0x4001400CU;
rw_reg32_t const TIM15_SR           = (rw_reg32_t)0x40014010U;
rw_reg32_t const TIM15_EGR          = (rw_reg32_t)0x40014014U;
rw_reg32_t const TIM15_CCMR1_OUTPUT = (rw_reg32_t)0x40014018U;
rw_reg32_t const TIM15_CCMR1_INPUT  = (rw_reg32_t)0x40014018U;
rw_reg32_t const TIM15_CCER         = (rw_reg32_t)0x40014020U;
rw_reg32_t const TIM15_CNT          = (rw_reg32_t)0x40014024U;
rw_reg32_t const TIM15_PSC          = (rw_reg32_t)0x40014028U;
rw_reg32_t const TIM15_ARR          = (rw_reg32_t)0x4001402CU;
rw_reg32_t const TIM15_RCR          = (rw_reg32_t)0x40014030U;
rw_reg32_t const TIM15_CCR1         = (rw_reg32_t)0x40014034U;
rw_reg32_t const TIM15_CCR2         = (rw_reg32_t)0x40014038U;
rw_reg32_t const TIM15_BDTR         = (rw_reg32_t)0x40014044U;
rw_reg32_t const TIM15_DCR          = (rw_reg32_t)0x40014048U;
rw_reg32_t const TIM15_DMAR         = (rw_reg32_t)0x4001404CU;
rw_reg32_t const TIM15_AF1          = (rw_reg32_t)0x40014060U;
rw_reg32_t const TIM15_TISEL        = (rw_reg32_t)0x40014068U;

/** @subsection TIM15 Register Field Definitions */

const field32_t TIM15_CR1_CEN       = {.msk = 0x00000001U, .pos = 0};
const field32_t TIM15_CR1_UDIS      = {.msk = 0x00000002U, .pos = 1};
const field32_t TIM15_CR1_URS       = {.msk = 0x00000004U, .pos = 2};
const field32_t TIM15_CR1_OPM       = {.msk = 0x00000008U, .pos = 3};
const field32_t TIM15_CR1_ARPE      = {.msk = 0x00000080U, .pos = 7};
const field32_t TIM15_CR1_CKD       = {.msk = 0x00000300U, .pos = 8};
const field32_t TIM15_CR1_UIFREMAP  = {.msk = 0x00000800U, .pos = 11};
const field32_t TIM15_CR2_CCPC      = {.msk = 0x00000001U, .pos = 0};
const field32_t TIM15_CR2_CCUS      = {.msk = 0x00000004U, .pos = 2};
const field32_t TIM15_CR2_CCDS      = {.msk = 0x00000008U, .pos = 3};
const field32_t TIM15_CR2_MMS       = {.msk = 0x00000070U, .pos = 4};
const field32_t TIM15_CR2_TI1S      = {.msk = 0x00000080U, .pos = 7};
const field32_t TIM15_CR2_OIS1N     = {.msk = 0x00000200U, .pos = 9};
const field32_t TIM15_SMCR_SMS      = {.msk = 0x00000007U, .pos = 0};
const field32_t TIM15_SMCR_TS_2_0   = {.msk = 0x00000070U, .pos = 4};
const field32_t TIM15_SMCR_MSM      = {.msk = 0x00000080U, .pos = 7};
const field32_t TIM15_SMCR_SMS_3    = {.msk = 0x00010000U, .pos = 16};
const field32_t TIM15_SMCR_TS_4_3   = {.msk = 0x00300000U, .pos = 20};
const field32_t TIM15_DIER_UIE      = {.msk = 0x00000001U, .pos = 0};
const field32_t TIM15_DIER_COMIE    = {.msk = 0x00000020U, .pos = 5};
const field32_t TIM15_DIER_TIE      = {.msk = 0x00000040U, .pos = 6};
const field32_t TIM15_DIER_BIE      = {.msk = 0x00000080U, .pos = 7};
const field32_t TIM15_DIER_UDE      = {.msk = 0x00000100U, .pos = 8};
const field32_t TIM15_DIER_COMDE    = {.msk = 0x00002000U, .pos = 13};
const field32_t TIM15_DIER_TDE      = {.msk = 0x00004000U, .pos = 14};
const field32_t TIM15_SR_BIF        = {.msk = 0x00000080U, .pos = 7};
const field32_t TIM15_SR_TIF        = {.msk = 0x00000040U, .pos = 6};
const field32_t TIM15_SR_COMIF      = {.msk = 0x00000020U, .pos = 5};
const field32_t TIM15_SR_UIF        = {.msk = 0x00000001U, .pos = 0};
const field32_t TIM15_EGR_BG        = {.msk = 0x00000080U, .pos = 7};
const field32_t TIM15_EGR_TG        = {.msk = 0x00000040U, .pos = 6};
const field32_t TIM15_EGR_COMG      = {.msk = 0x00000020U, .pos = 5};
const field32_t TIM15_EGR_UG        = {.msk = 0x00000001U, .pos = 0};
const field32_t TIM15_CCER_CC1NE    = {.msk = 0x00000004U, .pos = 2};
const field32_t TIM15_CNT_CNT       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t TIM15_CNT_UIFCPY    = {.msk = 0x80000000U, .pos = 31};
const field32_t TIM15_PSC_PSC       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t TIM15_ARR_ARR       = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t TIM15_RCR_REP       = {.msk = 0x000000FFU, .pos = 0};
const field32_t TIM15_CCR1_CCR1     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t TIM15_CCR2_CCR2     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t TIM15_BDTR_MOE      = {.msk = 0x00008000U, .pos = 15};
const field32_t TIM15_BDTR_AOE      = {.msk = 0x00004000U, .pos = 14};
const field32_t TIM15_BDTR_BKP      = {.msk = 0x00002000U, .pos = 13};
const field32_t TIM15_BDTR_BKE      = {.msk = 0x00001000U, .pos = 12};
const field32_t TIM15_BDTR_OSSR     = {.msk = 0x00000800U, .pos = 11};
const field32_t TIM15_BDTR_OSSI     = {.msk = 0x00000400U, .pos = 10};
const field32_t TIM15_BDTR_LOCK     = {.msk = 0x00000300U, .pos = 8};
const field32_t TIM15_BDTR_DTG      = {.msk = 0x000000FFU, .pos = 0};
const field32_t TIM15_BDTR_BKF      = {.msk = 0x000F0000U, .pos = 16};
const field32_t TIM15_DCR_DBL       = {.msk = 0x00001F00U, .pos = 8};
const field32_t TIM15_DCR_DBA       = {.msk = 0x0000001FU, .pos = 0};
const field32_t TIM15_DMAR_DMAB     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t TIM15_AF1_BKINE     = {.msk = 0x00000001U, .pos = 0};
const field32_t TIM15_AF1_BKDF1BK0E = {.msk = 0x00000100U, .pos = 8};
const field32_t TIM15_AF1_BKINP     = {.msk = 0x00000200U, .pos = 9};

/** @subsection Enumerated TIM15 Register Field Definitions */

field32_t const TIM15_CR2_OISx[3] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const TIM15_DIER_CCxIE[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const TIM15_DIER_CCxDE[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const TIM15_SR_CCxOF[3] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const TIM15_SR_CCxIF[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const TIM15_EGR_CCxG[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const TIM15_CCMR1_OUTPUT_CCxS[3] = {
  [1] = {.msk = 0x00000003U, .pos = 0},
  [2] = {.msk = 0x00000300U, .pos = 8},
};

field32_t const TIM15_CCMR1_OUTPUT_OCxFE[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const TIM15_CCMR1_OUTPUT_OCxPE[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const TIM15_CCMR1_OUTPUT_OCxM[3] = {
  [1] = {.msk = 0x00000070U, .pos = 4},
  [2] = {.msk = 0x00007000U, .pos = 12},
};

field32_t const TIM15_CCMR1_OUTPUT_OCxM_3[3] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const TIM15_CCMR1_INPUT_ICxF[3] = {
  [1] = {.msk = 0x000000F0U, .pos = 4},
  [2] = {.msk = 0x0000F000U, .pos = 12},
};

field32_t const TIM15_CCMR1_INPUT_ICxPSC[3] = {
  [1] = {.msk = 0x0000000CU, .pos = 2},
  [2] = {.msk = 0x00000C00U, .pos = 10},
};

field32_t const TIM15_CCER_CCxNP[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const TIM15_CCER_CCxP[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const TIM15_CCER_CCxE[3] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const TIM15_AF1_BKCMPxE[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const TIM15_AF1_BKCMPxP[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const TIM15_TISEL_TIxSEL[3] = {
  [1] = {.msk = 0x0000000FU, .pos = 0},
  [2] = {.msk = 0x00000F00U, .pos = 8},
};

/**************************************************************************************************
 * @section USARTx Definitions
 **************************************************************************************************/

/** @subsection Enumerated USARTx Register Definitions */

rw_reg32_t const USARTx_CR1[7] = {
  [1] = (rw_reg32_t)0x40011000U,
  [2] = (rw_reg32_t)0x40004400U,
  [3] = (rw_reg32_t)0x40004800U,
  [6] = (rw_reg32_t)0x40011400U,
};

rw_reg32_t const USARTx_CR2[7] = {
  [1] = (rw_reg32_t)0x40011004U,
  [2] = (rw_reg32_t)0x40004404U,
  [3] = (rw_reg32_t)0x40004804U,
  [6] = (rw_reg32_t)0x40011404U,
};

rw_reg32_t const USARTx_CR3[7] = {
  [1] = (rw_reg32_t)0x40011008U,
  [2] = (rw_reg32_t)0x40004408U,
  [3] = (rw_reg32_t)0x40004808U,
  [6] = (rw_reg32_t)0x40011408U,
};

rw_reg32_t const USARTx_BRR[7] = {
  [1] = (rw_reg32_t)0x4001100CU,
  [2] = (rw_reg32_t)0x4000440CU,
  [3] = (rw_reg32_t)0x4000480CU,
  [6] = (rw_reg32_t)0x4001140CU,
};

rw_reg32_t const USARTx_GTPR[7] = {
  [1] = (rw_reg32_t)0x40011010U,
  [2] = (rw_reg32_t)0x40004410U,
  [3] = (rw_reg32_t)0x40004810U,
  [6] = (rw_reg32_t)0x40011410U,
};

rw_reg32_t const USARTx_RTOR[7] = {
  [1] = (rw_reg32_t)0x40011014U,
  [2] = (rw_reg32_t)0x40004414U,
  [3] = (rw_reg32_t)0x40004814U,
  [6] = (rw_reg32_t)0x40011414U,
};

rw_reg32_t const USARTx_RQR[7] = {
  [1] = (rw_reg32_t)0x40011018U,
  [2] = (rw_reg32_t)0x40004418U,
  [3] = (rw_reg32_t)0x40004818U,
  [6] = (rw_reg32_t)0x40011418U,
};

ro_reg32_t const USARTx_ISR[7] = {
  [1] = (ro_reg32_t)0x4001101CU,
  [2] = (ro_reg32_t)0x4000441CU,
  [3] = (ro_reg32_t)0x4000481CU,
  [6] = (ro_reg32_t)0x4001141CU,
};

rw_reg32_t const USARTx_ICR[7] = {
  [1] = (rw_reg32_t)0x40011020U,
  [2] = (rw_reg32_t)0x40004420U,
  [3] = (rw_reg32_t)0x40004820U,
  [6] = (rw_reg32_t)0x40011420U,
};

ro_reg32_t const USARTx_RDR[7] = {
  [1] = (ro_reg32_t)0x40011024U,
  [2] = (ro_reg32_t)0x40004424U,
  [3] = (ro_reg32_t)0x40004824U,
  [6] = (ro_reg32_t)0x40011424U,
};

rw_reg32_t const USARTx_TDR[7] = {
  [1] = (rw_reg32_t)0x40011028U,
  [2] = (rw_reg32_t)0x40004428U,
  [3] = (rw_reg32_t)0x40004828U,
  [6] = (rw_reg32_t)0x40011428U,
};

rw_reg32_t const USARTx_PRESC[7] = {
  [1] = (rw_reg32_t)0x4001102CU,
  [2] = (rw_reg32_t)0x4000442CU,
  [3] = (rw_reg32_t)0x4000482CU,
  [6] = (rw_reg32_t)0x4001142CU,
};

/** @subsection USARTx Register Field Definitions */

const field32_t USARTx_CR1_RXFFIE      = {.msk = 0x80000000U, .pos = 31};
const field32_t USARTx_CR1_TXFEIE      = {.msk = 0x40000000U, .pos = 30};
const field32_t USARTx_CR1_FIFOEN      = {.msk = 0x20000000U, .pos = 29};
const field32_t USARTx_CR1_EOBIE       = {.msk = 0x08000000U, .pos = 27};
const field32_t USARTx_CR1_RTOIE       = {.msk = 0x04000000U, .pos = 26};
const field32_t USARTx_CR1_OVER8       = {.msk = 0x00008000U, .pos = 15};
const field32_t USARTx_CR1_CMIE        = {.msk = 0x00004000U, .pos = 14};
const field32_t USARTx_CR1_MME         = {.msk = 0x00002000U, .pos = 13};
const field32_t USARTx_CR1_WAKE        = {.msk = 0x00000800U, .pos = 11};
const field32_t USARTx_CR1_PCE         = {.msk = 0x00000400U, .pos = 10};
const field32_t USARTx_CR1_PS          = {.msk = 0x00000200U, .pos = 9};
const field32_t USARTx_CR1_PEIE        = {.msk = 0x00000100U, .pos = 8};
const field32_t USARTx_CR1_TXEIE       = {.msk = 0x00000080U, .pos = 7};
const field32_t USARTx_CR1_TCIE        = {.msk = 0x00000040U, .pos = 6};
const field32_t USARTx_CR1_RXNEIE      = {.msk = 0x00000020U, .pos = 5};
const field32_t USARTx_CR1_IDLEIE      = {.msk = 0x00000010U, .pos = 4};
const field32_t USARTx_CR1_TE          = {.msk = 0x00000008U, .pos = 3};
const field32_t USARTx_CR1_RE          = {.msk = 0x00000004U, .pos = 2};
const field32_t USARTx_CR1_UESM        = {.msk = 0x00000002U, .pos = 1};
const field32_t USARTx_CR1_UE          = {.msk = 0x00000001U, .pos = 0};
const field32_t USARTx_CR2_ADD4_7      = {.msk = 0xF0000000U, .pos = 28};
const field32_t USARTx_CR2_ADD0_3      = {.msk = 0x0F000000U, .pos = 24};
const field32_t USARTx_CR2_RTOEN       = {.msk = 0x00800000U, .pos = 23};
const field32_t USARTx_CR2_ABREN       = {.msk = 0x00100000U, .pos = 20};
const field32_t USARTx_CR2_MSBFIRST    = {.msk = 0x00080000U, .pos = 19};
const field32_t USARTx_CR2_TAINV       = {.msk = 0x00040000U, .pos = 18};
const field32_t USARTx_CR2_TXINV       = {.msk = 0x00020000U, .pos = 17};
const field32_t USARTx_CR2_RXINV       = {.msk = 0x00010000U, .pos = 16};
const field32_t USARTx_CR2_SWAP        = {.msk = 0x00008000U, .pos = 15};
const field32_t USARTx_CR2_LINEN       = {.msk = 0x00004000U, .pos = 14};
const field32_t USARTx_CR2_STOP        = {.msk = 0x00003000U, .pos = 12};
const field32_t USARTx_CR2_CLKEN       = {.msk = 0x00000800U, .pos = 11};
const field32_t USARTx_CR2_CPOL        = {.msk = 0x00000400U, .pos = 10};
const field32_t USARTx_CR2_CPHA        = {.msk = 0x00000200U, .pos = 9};
const field32_t USARTx_CR2_LBCL        = {.msk = 0x00000100U, .pos = 8};
const field32_t USARTx_CR2_LBDIE       = {.msk = 0x00000040U, .pos = 6};
const field32_t USARTx_CR2_LBDL        = {.msk = 0x00000020U, .pos = 5};
const field32_t USARTx_CR2_ADDM7       = {.msk = 0x00000010U, .pos = 4};
const field32_t USARTx_CR2_DIS_NSS     = {.msk = 0x00000008U, .pos = 3};
const field32_t USARTx_CR2_SLVEN       = {.msk = 0x00000001U, .pos = 0};
const field32_t USARTx_CR3_TXFTCFG     = {.msk = 0xE0000000U, .pos = 29};
const field32_t USARTx_CR3_RXFTIE      = {.msk = 0x10000000U, .pos = 28};
const field32_t USARTx_CR3_RXFTCFG     = {.msk = 0x0E000000U, .pos = 25};
const field32_t USARTx_CR3_TCBGTIE     = {.msk = 0x01000000U, .pos = 24};
const field32_t USARTx_CR3_TXFTIE      = {.msk = 0x00800000U, .pos = 23};
const field32_t USARTx_CR3_WUFIE       = {.msk = 0x00400000U, .pos = 22};
const field32_t USARTx_CR3_WUS         = {.msk = 0x00300000U, .pos = 20};
const field32_t USARTx_CR3_SCARCNT     = {.msk = 0x000E0000U, .pos = 17};
const field32_t USARTx_CR3_DEP         = {.msk = 0x00008000U, .pos = 15};
const field32_t USARTx_CR3_DEM         = {.msk = 0x00004000U, .pos = 14};
const field32_t USARTx_CR3_DDRE        = {.msk = 0x00002000U, .pos = 13};
const field32_t USARTx_CR3_OVRDIS      = {.msk = 0x00001000U, .pos = 12};
const field32_t USARTx_CR3_ONEBIT      = {.msk = 0x00000800U, .pos = 11};
const field32_t USARTx_CR3_CTSIE       = {.msk = 0x00000400U, .pos = 10};
const field32_t USARTx_CR3_CTSE        = {.msk = 0x00000200U, .pos = 9};
const field32_t USARTx_CR3_RTSE        = {.msk = 0x00000100U, .pos = 8};
const field32_t USARTx_CR3_DMAT        = {.msk = 0x00000080U, .pos = 7};
const field32_t USARTx_CR3_DMAR        = {.msk = 0x00000040U, .pos = 6};
const field32_t USARTx_CR3_SCEN        = {.msk = 0x00000020U, .pos = 5};
const field32_t USARTx_CR3_NACK        = {.msk = 0x00000010U, .pos = 4};
const field32_t USARTx_CR3_HDSEL       = {.msk = 0x00000008U, .pos = 3};
const field32_t USARTx_CR3_IRLP        = {.msk = 0x00000004U, .pos = 2};
const field32_t USARTx_CR3_IREN        = {.msk = 0x00000002U, .pos = 1};
const field32_t USARTx_CR3_EIE         = {.msk = 0x00000001U, .pos = 0};
const field32_t USARTx_BRR_BRR_4_15    = {.msk = 0x0000FFF0U, .pos = 4};
const field32_t USARTx_BRR_BRR_0_3     = {.msk = 0x0000000FU, .pos = 0};
const field32_t USARTx_GTPR_GT         = {.msk = 0x0000FF00U, .pos = 8};
const field32_t USARTx_GTPR_PSC        = {.msk = 0x000000FFU, .pos = 0};
const field32_t USARTx_RTOR_BLEN       = {.msk = 0xFF000000U, .pos = 24};
const field32_t USARTx_RTOR_RTO        = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t USARTx_RQR_TXFRQ       = {.msk = 0x00000010U, .pos = 4};
const field32_t USARTx_RQR_RXFRQ       = {.msk = 0x00000008U, .pos = 3};
const field32_t USARTx_RQR_MMRQ        = {.msk = 0x00000004U, .pos = 2};
const field32_t USARTx_RQR_SBKRQ       = {.msk = 0x00000002U, .pos = 1};
const field32_t USARTx_RQR_ABRRQ       = {.msk = 0x00000001U, .pos = 0};
const field32_t USARTx_ISR_TXFT        = {.msk = 0x08000000U, .pos = 27};
const field32_t USARTx_ISR_RXFT        = {.msk = 0x04000000U, .pos = 26};
const field32_t USARTx_ISR_TCBGT       = {.msk = 0x02000000U, .pos = 25};
const field32_t USARTx_ISR_RXFF        = {.msk = 0x01000000U, .pos = 24};
const field32_t USARTx_ISR_TXFE        = {.msk = 0x00800000U, .pos = 23};
const field32_t USARTx_ISR_REACK       = {.msk = 0x00400000U, .pos = 22};
const field32_t USARTx_ISR_TEACK       = {.msk = 0x00200000U, .pos = 21};
const field32_t USARTx_ISR_WUF         = {.msk = 0x00100000U, .pos = 20};
const field32_t USARTx_ISR_RWU         = {.msk = 0x00080000U, .pos = 19};
const field32_t USARTx_ISR_SBKF        = {.msk = 0x00040000U, .pos = 18};
const field32_t USARTx_ISR_CMF         = {.msk = 0x00020000U, .pos = 17};
const field32_t USARTx_ISR_BUSY        = {.msk = 0x00010000U, .pos = 16};
const field32_t USARTx_ISR_ABRF        = {.msk = 0x00008000U, .pos = 15};
const field32_t USARTx_ISR_ABRE        = {.msk = 0x00004000U, .pos = 14};
const field32_t USARTx_ISR_UDR         = {.msk = 0x00002000U, .pos = 13};
const field32_t USARTx_ISR_EOBF        = {.msk = 0x00001000U, .pos = 12};
const field32_t USARTx_ISR_RTOF        = {.msk = 0x00000800U, .pos = 11};
const field32_t USARTx_ISR_CTS         = {.msk = 0x00000400U, .pos = 10};
const field32_t USARTx_ISR_CTSIF       = {.msk = 0x00000200U, .pos = 9};
const field32_t USARTx_ISR_LBDF        = {.msk = 0x00000100U, .pos = 8};
const field32_t USARTx_ISR_TXE         = {.msk = 0x00000080U, .pos = 7};
const field32_t USARTx_ISR_TC          = {.msk = 0x00000040U, .pos = 6};
const field32_t USARTx_ISR_RXNE        = {.msk = 0x00000020U, .pos = 5};
const field32_t USARTx_ISR_IDLE        = {.msk = 0x00000010U, .pos = 4};
const field32_t USARTx_ISR_ORE         = {.msk = 0x00000008U, .pos = 3};
const field32_t USARTx_ISR_NF          = {.msk = 0x00000004U, .pos = 2};
const field32_t USARTx_ISR_FE          = {.msk = 0x00000002U, .pos = 1};
const field32_t USARTx_ISR_PE          = {.msk = 0x00000001U, .pos = 0};
const field32_t USARTx_ICR_WUCF        = {.msk = 0x00100000U, .pos = 20};
const field32_t USARTx_ICR_CMCF        = {.msk = 0x00020000U, .pos = 17};
const field32_t USARTx_ICR_UDRCF       = {.msk = 0x00002000U, .pos = 13};
const field32_t USARTx_ICR_EOBCF       = {.msk = 0x00001000U, .pos = 12};
const field32_t USARTx_ICR_RTOCF       = {.msk = 0x00000800U, .pos = 11};
const field32_t USARTx_ICR_CTSCF       = {.msk = 0x00000200U, .pos = 9};
const field32_t USARTx_ICR_LBDCF       = {.msk = 0x00000100U, .pos = 8};
const field32_t USARTx_ICR_TCBGTC      = {.msk = 0x00000080U, .pos = 7};
const field32_t USARTx_ICR_TCCF        = {.msk = 0x00000040U, .pos = 6};
const field32_t USARTx_ICR_TXFECF      = {.msk = 0x00000020U, .pos = 5};
const field32_t USARTx_ICR_IDLECF      = {.msk = 0x00000010U, .pos = 4};
const field32_t USARTx_ICR_ORECF       = {.msk = 0x00000008U, .pos = 3};
const field32_t USARTx_ICR_NCF         = {.msk = 0x00000004U, .pos = 2};
const field32_t USARTx_ICR_FECF        = {.msk = 0x00000002U, .pos = 1};
const field32_t USARTx_ICR_PECF        = {.msk = 0x00000001U, .pos = 0};
const field32_t USARTx_RDR_RDR         = {.msk = 0x000001FFU, .pos = 0};
const field32_t USARTx_TDR_TDR         = {.msk = 0x000001FFU, .pos = 0};
const field32_t USARTx_PRESC_PRESCALER = {.msk = 0x0000000FU, .pos = 0};

/** @subsection Enumerated USARTx Register Field Definitions */

field32_t const USARTx_CR1_Mx[2] = {
  [0] = {.msk = 0x00001000U, .pos = 12},
  [1] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const USARTx_CR1_DEATx[5] = {
  [0] = {.msk = 0x00200000U, .pos = 21},
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
  [4] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const USARTx_CR1_DEDTx[5] = {
  [0] = {.msk = 0x00010000U, .pos = 16},
  [1] = {.msk = 0x00020000U, .pos = 17},
  [2] = {.msk = 0x00040000U, .pos = 18},
  [3] = {.msk = 0x00080000U, .pos = 19},
  [4] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const USARTx_CR2_ABRMODx[2] = {
  [0] = {.msk = 0x00200000U, .pos = 21},
  [1] = {.msk = 0x00400000U, .pos = 22},
};

/**************************************************************************************************
 * @section UARTx Definitions
 **************************************************************************************************/

/** @subsection Enumerated UARTx Register Definitions */

rw_reg32_t const UARTx_CR1[9] = {
  [4] = (rw_reg32_t)0x40004C00U,
  [5] = (rw_reg32_t)0x40005000U,
  [7] = (rw_reg32_t)0x40007800U,
  [8] = (rw_reg32_t)0x40007C00U,
};

rw_reg32_t const UARTx_CR2[9] = {
  [4] = (rw_reg32_t)0x40004C04U,
  [5] = (rw_reg32_t)0x40005004U,
  [7] = (rw_reg32_t)0x40007804U,
  [8] = (rw_reg32_t)0x40007C04U,
};

rw_reg32_t const UARTx_CR3[9] = {
  [4] = (rw_reg32_t)0x40004C08U,
  [5] = (rw_reg32_t)0x40005008U,
  [7] = (rw_reg32_t)0x40007808U,
  [8] = (rw_reg32_t)0x40007C08U,
};

rw_reg32_t const UARTx_BRR[9] = {
  [4] = (rw_reg32_t)0x40004C0CU,
  [5] = (rw_reg32_t)0x4000500CU,
  [7] = (rw_reg32_t)0x4000780CU,
  [8] = (rw_reg32_t)0x40007C0CU,
};

rw_reg32_t const UARTx_GTPR[9] = {
  [4] = (rw_reg32_t)0x40004C10U,
  [5] = (rw_reg32_t)0x40005010U,
  [7] = (rw_reg32_t)0x40007810U,
  [8] = (rw_reg32_t)0x40007C10U,
};

rw_reg32_t const UARTx_RTOR[9] = {
  [4] = (rw_reg32_t)0x40004C14U,
  [5] = (rw_reg32_t)0x40005014U,
  [7] = (rw_reg32_t)0x40007814U,
  [8] = (rw_reg32_t)0x40007C14U,
};

rw_reg32_t const UARTx_RQR[9] = {
  [4] = (rw_reg32_t)0x40004C18U,
  [5] = (rw_reg32_t)0x40005018U,
  [7] = (rw_reg32_t)0x40007818U,
  [8] = (rw_reg32_t)0x40007C18U,
};

ro_reg32_t const UARTx_ISR[9] = {
  [4] = (ro_reg32_t)0x40004C1CU,
  [5] = (ro_reg32_t)0x4000501CU,
  [7] = (ro_reg32_t)0x4000781CU,
  [8] = (ro_reg32_t)0x40007C1CU,
};

rw_reg32_t const UARTx_ICR[9] = {
  [4] = (rw_reg32_t)0x40004C20U,
  [5] = (rw_reg32_t)0x40005020U,
  [7] = (rw_reg32_t)0x40007820U,
  [8] = (rw_reg32_t)0x40007C20U,
};

ro_reg32_t const UARTx_RDR[9] = {
  [4] = (ro_reg32_t)0x40004C24U,
  [5] = (ro_reg32_t)0x40005024U,
  [7] = (ro_reg32_t)0x40007824U,
  [8] = (ro_reg32_t)0x40007C24U,
};

rw_reg32_t const UARTx_TDR[9] = {
  [4] = (rw_reg32_t)0x40004C28U,
  [5] = (rw_reg32_t)0x40005028U,
  [7] = (rw_reg32_t)0x40007828U,
  [8] = (rw_reg32_t)0x40007C28U,
};

rw_reg32_t const UARTx_PRESC[9] = {
  [4] = (rw_reg32_t)0x40004C2CU,
  [5] = (rw_reg32_t)0x4000502CU,
  [7] = (rw_reg32_t)0x4000782CU,
  [8] = (rw_reg32_t)0x40007C2CU,
};

/** @subsection UARTx Register Field Definitions */

const field32_t UARTx_CR1_RXFFIE      = {.msk = 0x80000000U, .pos = 31};
const field32_t UARTx_CR1_TXFEIE      = {.msk = 0x40000000U, .pos = 30};
const field32_t UARTx_CR1_FIFOEN      = {.msk = 0x20000000U, .pos = 29};
const field32_t UARTx_CR1_EOBIE       = {.msk = 0x08000000U, .pos = 27};
const field32_t UARTx_CR1_RTOIE       = {.msk = 0x04000000U, .pos = 26};
const field32_t UARTx_CR1_OVER8       = {.msk = 0x00008000U, .pos = 15};
const field32_t UARTx_CR1_CMIE        = {.msk = 0x00004000U, .pos = 14};
const field32_t UARTx_CR1_MME         = {.msk = 0x00002000U, .pos = 13};
const field32_t UARTx_CR1_WAKE        = {.msk = 0x00000800U, .pos = 11};
const field32_t UARTx_CR1_PCE         = {.msk = 0x00000400U, .pos = 10};
const field32_t UARTx_CR1_PS          = {.msk = 0x00000200U, .pos = 9};
const field32_t UARTx_CR1_PEIE        = {.msk = 0x00000100U, .pos = 8};
const field32_t UARTx_CR1_TXEIE       = {.msk = 0x00000080U, .pos = 7};
const field32_t UARTx_CR1_TCIE        = {.msk = 0x00000040U, .pos = 6};
const field32_t UARTx_CR1_RXNEIE      = {.msk = 0x00000020U, .pos = 5};
const field32_t UARTx_CR1_IDLEIE      = {.msk = 0x00000010U, .pos = 4};
const field32_t UARTx_CR1_TE          = {.msk = 0x00000008U, .pos = 3};
const field32_t UARTx_CR1_RE          = {.msk = 0x00000004U, .pos = 2};
const field32_t UARTx_CR1_UESM        = {.msk = 0x00000002U, .pos = 1};
const field32_t UARTx_CR1_UE          = {.msk = 0x00000001U, .pos = 0};
const field32_t UARTx_CR2_ADD4_7      = {.msk = 0xF0000000U, .pos = 28};
const field32_t UARTx_CR2_ADD0_3      = {.msk = 0x0F000000U, .pos = 24};
const field32_t UARTx_CR2_RTOEN       = {.msk = 0x00800000U, .pos = 23};
const field32_t UARTx_CR2_ABREN       = {.msk = 0x00100000U, .pos = 20};
const field32_t UARTx_CR2_MSBFIRST    = {.msk = 0x00080000U, .pos = 19};
const field32_t UARTx_CR2_TAINV       = {.msk = 0x00040000U, .pos = 18};
const field32_t UARTx_CR2_TXINV       = {.msk = 0x00020000U, .pos = 17};
const field32_t UARTx_CR2_RXINV       = {.msk = 0x00010000U, .pos = 16};
const field32_t UARTx_CR2_SWAP        = {.msk = 0x00008000U, .pos = 15};
const field32_t UARTx_CR2_LINEN       = {.msk = 0x00004000U, .pos = 14};
const field32_t UARTx_CR2_STOP        = {.msk = 0x00003000U, .pos = 12};
const field32_t UARTx_CR2_CLKEN       = {.msk = 0x00000800U, .pos = 11};
const field32_t UARTx_CR2_CPOL        = {.msk = 0x00000400U, .pos = 10};
const field32_t UARTx_CR2_CPHA        = {.msk = 0x00000200U, .pos = 9};
const field32_t UARTx_CR2_LBCL        = {.msk = 0x00000100U, .pos = 8};
const field32_t UARTx_CR2_LBDIE       = {.msk = 0x00000040U, .pos = 6};
const field32_t UARTx_CR2_LBDL        = {.msk = 0x00000020U, .pos = 5};
const field32_t UARTx_CR2_ADDM7       = {.msk = 0x00000010U, .pos = 4};
const field32_t UARTx_CR2_DIS_NSS     = {.msk = 0x00000008U, .pos = 3};
const field32_t UARTx_CR2_SLVEN       = {.msk = 0x00000001U, .pos = 0};
const field32_t UARTx_CR3_TXFTCFG     = {.msk = 0xE0000000U, .pos = 29};
const field32_t UARTx_CR3_RXFTIE      = {.msk = 0x10000000U, .pos = 28};
const field32_t UARTx_CR3_RXFTCFG     = {.msk = 0x0E000000U, .pos = 25};
const field32_t UARTx_CR3_TCBGTIE     = {.msk = 0x01000000U, .pos = 24};
const field32_t UARTx_CR3_TXFTIE      = {.msk = 0x00800000U, .pos = 23};
const field32_t UARTx_CR3_WUFIE       = {.msk = 0x00400000U, .pos = 22};
const field32_t UARTx_CR3_WUS         = {.msk = 0x00300000U, .pos = 20};
const field32_t UARTx_CR3_SCARCNT     = {.msk = 0x000E0000U, .pos = 17};
const field32_t UARTx_CR3_DEP         = {.msk = 0x00008000U, .pos = 15};
const field32_t UARTx_CR3_DEM         = {.msk = 0x00004000U, .pos = 14};
const field32_t UARTx_CR3_DDRE        = {.msk = 0x00002000U, .pos = 13};
const field32_t UARTx_CR3_OVRDIS      = {.msk = 0x00001000U, .pos = 12};
const field32_t UARTx_CR3_ONEBIT      = {.msk = 0x00000800U, .pos = 11};
const field32_t UARTx_CR3_CTSIE       = {.msk = 0x00000400U, .pos = 10};
const field32_t UARTx_CR3_CTSE        = {.msk = 0x00000200U, .pos = 9};
const field32_t UARTx_CR3_RTSE        = {.msk = 0x00000100U, .pos = 8};
const field32_t UARTx_CR3_DMAT        = {.msk = 0x00000080U, .pos = 7};
const field32_t UARTx_CR3_DMAR        = {.msk = 0x00000040U, .pos = 6};
const field32_t UARTx_CR3_SCEN        = {.msk = 0x00000020U, .pos = 5};
const field32_t UARTx_CR3_NACK        = {.msk = 0x00000010U, .pos = 4};
const field32_t UARTx_CR3_HDSEL       = {.msk = 0x00000008U, .pos = 3};
const field32_t UARTx_CR3_IRLP        = {.msk = 0x00000004U, .pos = 2};
const field32_t UARTx_CR3_IREN        = {.msk = 0x00000002U, .pos = 1};
const field32_t UARTx_CR3_EIE         = {.msk = 0x00000001U, .pos = 0};
const field32_t UARTx_BRR_BRR_4_15    = {.msk = 0x0000FFF0U, .pos = 4};
const field32_t UARTx_BRR_BRR_0_3     = {.msk = 0x0000000FU, .pos = 0};
const field32_t UARTx_GTPR_GT         = {.msk = 0x0000FF00U, .pos = 8};
const field32_t UARTx_GTPR_PSC        = {.msk = 0x000000FFU, .pos = 0};
const field32_t UARTx_RTOR_BLEN       = {.msk = 0xFF000000U, .pos = 24};
const field32_t UARTx_RTOR_RTO        = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t UARTx_RQR_TXFRQ       = {.msk = 0x00000010U, .pos = 4};
const field32_t UARTx_RQR_RXFRQ       = {.msk = 0x00000008U, .pos = 3};
const field32_t UARTx_RQR_MMRQ        = {.msk = 0x00000004U, .pos = 2};
const field32_t UARTx_RQR_SBKRQ       = {.msk = 0x00000002U, .pos = 1};
const field32_t UARTx_RQR_ABRRQ       = {.msk = 0x00000001U, .pos = 0};
const field32_t UARTx_ISR_TXFT        = {.msk = 0x08000000U, .pos = 27};
const field32_t UARTx_ISR_RXFT        = {.msk = 0x04000000U, .pos = 26};
const field32_t UARTx_ISR_TCBGT       = {.msk = 0x02000000U, .pos = 25};
const field32_t UARTx_ISR_RXFF        = {.msk = 0x01000000U, .pos = 24};
const field32_t UARTx_ISR_TXFE        = {.msk = 0x00800000U, .pos = 23};
const field32_t UARTx_ISR_REACK       = {.msk = 0x00400000U, .pos = 22};
const field32_t UARTx_ISR_TEACK       = {.msk = 0x00200000U, .pos = 21};
const field32_t UARTx_ISR_WUF         = {.msk = 0x00100000U, .pos = 20};
const field32_t UARTx_ISR_RWU         = {.msk = 0x00080000U, .pos = 19};
const field32_t UARTx_ISR_SBKF        = {.msk = 0x00040000U, .pos = 18};
const field32_t UARTx_ISR_CMF         = {.msk = 0x00020000U, .pos = 17};
const field32_t UARTx_ISR_BUSY        = {.msk = 0x00010000U, .pos = 16};
const field32_t UARTx_ISR_ABRF        = {.msk = 0x00008000U, .pos = 15};
const field32_t UARTx_ISR_ABRE        = {.msk = 0x00004000U, .pos = 14};
const field32_t UARTx_ISR_UDR         = {.msk = 0x00002000U, .pos = 13};
const field32_t UARTx_ISR_EOBF        = {.msk = 0x00001000U, .pos = 12};
const field32_t UARTx_ISR_RTOF        = {.msk = 0x00000800U, .pos = 11};
const field32_t UARTx_ISR_CTS         = {.msk = 0x00000400U, .pos = 10};
const field32_t UARTx_ISR_CTSIF       = {.msk = 0x00000200U, .pos = 9};
const field32_t UARTx_ISR_LBDF        = {.msk = 0x00000100U, .pos = 8};
const field32_t UARTx_ISR_TXE         = {.msk = 0x00000080U, .pos = 7};
const field32_t UARTx_ISR_TC          = {.msk = 0x00000040U, .pos = 6};
const field32_t UARTx_ISR_RXNE        = {.msk = 0x00000020U, .pos = 5};
const field32_t UARTx_ISR_IDLE        = {.msk = 0x00000010U, .pos = 4};
const field32_t UARTx_ISR_ORE         = {.msk = 0x00000008U, .pos = 3};
const field32_t UARTx_ISR_NF          = {.msk = 0x00000004U, .pos = 2};
const field32_t UARTx_ISR_FE          = {.msk = 0x00000002U, .pos = 1};
const field32_t UARTx_ISR_PE          = {.msk = 0x00000001U, .pos = 0};
const field32_t UARTx_ICR_WUCF        = {.msk = 0x00100000U, .pos = 20};
const field32_t UARTx_ICR_CMCF        = {.msk = 0x00020000U, .pos = 17};
const field32_t UARTx_ICR_UDRCF       = {.msk = 0x00002000U, .pos = 13};
const field32_t UARTx_ICR_EOBCF       = {.msk = 0x00001000U, .pos = 12};
const field32_t UARTx_ICR_RTOCF       = {.msk = 0x00000800U, .pos = 11};
const field32_t UARTx_ICR_CTSCF       = {.msk = 0x00000200U, .pos = 9};
const field32_t UARTx_ICR_LBDCF       = {.msk = 0x00000100U, .pos = 8};
const field32_t UARTx_ICR_TCBGTC      = {.msk = 0x00000080U, .pos = 7};
const field32_t UARTx_ICR_TCCF        = {.msk = 0x00000040U, .pos = 6};
const field32_t UARTx_ICR_TXFECF      = {.msk = 0x00000020U, .pos = 5};
const field32_t UARTx_ICR_IDLECF      = {.msk = 0x00000010U, .pos = 4};
const field32_t UARTx_ICR_ORECF       = {.msk = 0x00000008U, .pos = 3};
const field32_t UARTx_ICR_NCF         = {.msk = 0x00000004U, .pos = 2};
const field32_t UARTx_ICR_FECF        = {.msk = 0x00000002U, .pos = 1};
const field32_t UARTx_ICR_PECF        = {.msk = 0x00000001U, .pos = 0};
const field32_t UARTx_RDR_RDR         = {.msk = 0x000001FFU, .pos = 0};
const field32_t UARTx_TDR_TDR         = {.msk = 0x000001FFU, .pos = 0};
const field32_t UARTx_PRESC_PRESCALER = {.msk = 0x0000000FU, .pos = 0};

/** @subsection Enumerated UARTx Register Field Definitions */

field32_t const UARTx_CR1_Mx[2] = {
  [0] = {.msk = 0x00001000U, .pos = 12},
  [1] = {.msk = 0x10000000U, .pos = 28},
};

field32_t const UARTx_CR1_DEATx[5] = {
  [0] = {.msk = 0x00200000U, .pos = 21},
  [1] = {.msk = 0x00400000U, .pos = 22},
  [2] = {.msk = 0x00800000U, .pos = 23},
  [3] = {.msk = 0x01000000U, .pos = 24},
  [4] = {.msk = 0x02000000U, .pos = 25},
};

field32_t const UARTx_CR1_DEDTx[5] = {
  [0] = {.msk = 0x00010000U, .pos = 16},
  [1] = {.msk = 0x00020000U, .pos = 17},
  [2] = {.msk = 0x00040000U, .pos = 18},
  [3] = {.msk = 0x00080000U, .pos = 19},
  [4] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const UARTx_CR2_ABRMODx[2] = {
  [0] = {.msk = 0x00200000U, .pos = 21},
  [1] = {.msk = 0x00400000U, .pos = 22},
};

/**************************************************************************************************
 * @section A_TIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated A_TIMx Register Definitions */

rw_reg32_t const A_TIMx_CR1[9] = {
  [1] = (rw_reg32_t)0x40010000U,
  [8] = (rw_reg32_t)0x40010400U,
};

rw_reg32_t const A_TIMx_CR2[9] = {
  [1] = (rw_reg32_t)0x40010004U,
  [8] = (rw_reg32_t)0x40010404U,
};

rw_reg32_t const A_TIMx_SMCR[9] = {
  [1] = (rw_reg32_t)0x40010008U,
  [8] = (rw_reg32_t)0x40010408U,
};

rw_reg32_t const A_TIMx_DIER[9] = {
  [1] = (rw_reg32_t)0x4001000CU,
  [8] = (rw_reg32_t)0x4001040CU,
};

rw_reg32_t const A_TIMx_SR[9] = {
  [1] = (rw_reg32_t)0x40010010U,
  [8] = (rw_reg32_t)0x40010410U,
};

rw_reg32_t const A_TIMx_EGR[9] = {
  [1] = (rw_reg32_t)0x40010014U,
  [8] = (rw_reg32_t)0x40010414U,
};

rw_reg32_t const A_TIMx_CCMR1_OUTPUT[9] = {
  [1] = (rw_reg32_t)0x40010018U,
  [8] = (rw_reg32_t)0x40010418U,
};

rw_reg32_t const A_TIMx_CCMR1_INPUT[9] = {
  [1] = (rw_reg32_t)0x40010018U,
  [8] = (rw_reg32_t)0x40010418U,
};

rw_reg32_t const A_TIMx_CCMR2_OUTPUT[9] = {
  [1] = (rw_reg32_t)0x4001001CU,
  [8] = (rw_reg32_t)0x4001041CU,
};

rw_reg32_t const A_TIMx_CCMR2_INPUT[9] = {
  [1] = (rw_reg32_t)0x4001001CU,
  [8] = (rw_reg32_t)0x4001041CU,
};

rw_reg32_t const A_TIMx_CCER[9] = {
  [1] = (rw_reg32_t)0x40010020U,
  [8] = (rw_reg32_t)0x40010420U,
};

rw_reg32_t const A_TIMx_CNT[9] = {
  [1] = (rw_reg32_t)0x40010024U,
  [8] = (rw_reg32_t)0x40010424U,
};

rw_reg32_t const A_TIMx_PSC[9] = {
  [1] = (rw_reg32_t)0x40010028U,
  [8] = (rw_reg32_t)0x40010428U,
};

rw_reg32_t const A_TIMx_ARR[9] = {
  [1] = (rw_reg32_t)0x4001002CU,
  [8] = (rw_reg32_t)0x4001042CU,
};

rw_reg32_t const A_TIMx_CCR1[9] = {
  [1] = (rw_reg32_t)0x40010034U,
  [8] = (rw_reg32_t)0x40010434U,
};

rw_reg32_t const A_TIMx_CCR2[9] = {
  [1] = (rw_reg32_t)0x40010038U,
  [8] = (rw_reg32_t)0x40010438U,
};

rw_reg32_t const A_TIMx_CCR3[9] = {
  [1] = (rw_reg32_t)0x4001003CU,
  [8] = (rw_reg32_t)0x4001043CU,
};

rw_reg32_t const A_TIMx_CCR4[9] = {
  [1] = (rw_reg32_t)0x40010040U,
  [8] = (rw_reg32_t)0x40010440U,
};

rw_reg32_t const A_TIMx_DCR[9] = {
  [1] = (rw_reg32_t)0x40010048U,
  [8] = (rw_reg32_t)0x40010448U,
};

rw_reg32_t const A_TIMx_DMAR[9] = {
  [1] = (rw_reg32_t)0x4001004CU,
  [8] = (rw_reg32_t)0x4001044CU,
};

rw_reg32_t const A_TIMx_RCR[9] = {
  [1] = (rw_reg32_t)0x40010030U,
  [8] = (rw_reg32_t)0x40010430U,
};

rw_reg32_t const A_TIMx_BDTR[9] = {
  [1] = (rw_reg32_t)0x40010044U,
  [8] = (rw_reg32_t)0x40010444U,
};

rw_reg32_t const A_TIMx_CCMR3_OUTPUT[9] = {
  [1] = (rw_reg32_t)0x40010054U,
  [8] = (rw_reg32_t)0x40010454U,
};

rw_reg32_t const A_TIMx_CCR5[9] = {
  [1] = (rw_reg32_t)0x40010058U,
  [8] = (rw_reg32_t)0x40010458U,
};

rw_reg32_t const A_TIMx_CRR6[9] = {
  [1] = (rw_reg32_t)0x4001005CU,
  [8] = (rw_reg32_t)0x4001045CU,
};

rw_reg32_t const A_TIMx_AF1[9] = {
  [1] = (rw_reg32_t)0x40010060U,
  [8] = (rw_reg32_t)0x40010460U,
};

rw_reg32_t const A_TIMx_AF2[9] = {
  [1] = (rw_reg32_t)0x40010064U,
  [8] = (rw_reg32_t)0x40010464U,
};

rw_reg32_t const A_TIMx_TISEL[9] = {
  [1] = (rw_reg32_t)0x40010068U,
  [8] = (rw_reg32_t)0x40010468U,
};

/** @subsection A_TIMx Register Field Definitions */

const field32_t A_TIMx_CR1_CEN             = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_CR1_UDIS            = {.msk = 0x00000002U, .pos = 1};
const field32_t A_TIMx_CR1_URS             = {.msk = 0x00000004U, .pos = 2};
const field32_t A_TIMx_CR1_OPM             = {.msk = 0x00000008U, .pos = 3};
const field32_t A_TIMx_CR1_DIR             = {.msk = 0x00000010U, .pos = 4};
const field32_t A_TIMx_CR1_CMS             = {.msk = 0x00000060U, .pos = 5};
const field32_t A_TIMx_CR1_ARPE            = {.msk = 0x00000080U, .pos = 7};
const field32_t A_TIMx_CR1_CKD             = {.msk = 0x00000300U, .pos = 8};
const field32_t A_TIMx_CR1_UIFREMAP        = {.msk = 0x00000800U, .pos = 11};
const field32_t A_TIMx_CR2_MMS2            = {.msk = 0x00F00000U, .pos = 20};
const field32_t A_TIMx_CR2_TI1S            = {.msk = 0x00000080U, .pos = 7};
const field32_t A_TIMx_CR2_MMS             = {.msk = 0x00000070U, .pos = 4};
const field32_t A_TIMx_CR2_CCDS            = {.msk = 0x00000008U, .pos = 3};
const field32_t A_TIMx_CR2_CCUS            = {.msk = 0x00000004U, .pos = 2};
const field32_t A_TIMx_CR2_CCPC            = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_SMCR_SMS            = {.msk = 0x00000007U, .pos = 0};
const field32_t A_TIMx_SMCR_TS             = {.msk = 0x00000070U, .pos = 4};
const field32_t A_TIMx_SMCR_MSM            = {.msk = 0x00000080U, .pos = 7};
const field32_t A_TIMx_SMCR_ETF            = {.msk = 0x00000F00U, .pos = 8};
const field32_t A_TIMx_SMCR_ETPS           = {.msk = 0x00003000U, .pos = 12};
const field32_t A_TIMx_SMCR_ECE            = {.msk = 0x00004000U, .pos = 14};
const field32_t A_TIMx_SMCR_ETP            = {.msk = 0x00008000U, .pos = 15};
const field32_t A_TIMx_SMCR_SMS_3          = {.msk = 0x00010000U, .pos = 16};
const field32_t A_TIMx_SMCR_TS_4_3         = {.msk = 0x00300000U, .pos = 20};
const field32_t A_TIMx_DIER_TDE            = {.msk = 0x00004000U, .pos = 14};
const field32_t A_TIMx_DIER_COMDE          = {.msk = 0x00002000U, .pos = 13};
const field32_t A_TIMx_DIER_UDE            = {.msk = 0x00000100U, .pos = 8};
const field32_t A_TIMx_DIER_TIE            = {.msk = 0x00000040U, .pos = 6};
const field32_t A_TIMx_DIER_UIE            = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_DIER_BIE            = {.msk = 0x00000080U, .pos = 7};
const field32_t A_TIMx_DIER_COMIE          = {.msk = 0x00000020U, .pos = 5};
const field32_t A_TIMx_SR_SBIF             = {.msk = 0x00002000U, .pos = 13};
const field32_t A_TIMx_SR_B2IF             = {.msk = 0x00000100U, .pos = 8};
const field32_t A_TIMx_SR_BIF              = {.msk = 0x00000080U, .pos = 7};
const field32_t A_TIMx_SR_TIF              = {.msk = 0x00000040U, .pos = 6};
const field32_t A_TIMx_SR_COMIF            = {.msk = 0x00000020U, .pos = 5};
const field32_t A_TIMx_SR_UIF              = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_EGR_UG              = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_EGR_COMG            = {.msk = 0x00000020U, .pos = 5};
const field32_t A_TIMx_EGR_TG              = {.msk = 0x00000040U, .pos = 6};
const field32_t A_TIMx_EGR_BG              = {.msk = 0x00000080U, .pos = 7};
const field32_t A_TIMx_EGR_B2G             = {.msk = 0x00000100U, .pos = 8};
const field32_t A_TIMx_CCMR1_INPUT_IC2PCS  = {.msk = 0x00000C00U, .pos = 10};
const field32_t A_TIMx_CCMR1_INPUT_ICPCS   = {.msk = 0x0000000CU, .pos = 2};
const field32_t A_TIMx_CCMR2_OUTPUT_OC3M_3 = {.msk = 0x00010000U, .pos = 16};
const field32_t A_TIMx_CCMR2_OUTPUT_OC4M_4 = {.msk = 0x01000000U, .pos = 24};
const field32_t A_TIMx_CNT_CNT             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_CNT_UIFCPY          = {.msk = 0x80000000U, .pos = 31};
const field32_t A_TIMx_PSC_PSC             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_ARR_ARR             = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_CCR1_CCR1           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_CCR2_CCR2           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_CCR3_CCR3           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_CCR4_CCR4           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_DCR_DBL             = {.msk = 0x00001F00U, .pos = 8};
const field32_t A_TIMx_DCR_DBA             = {.msk = 0x0000001FU, .pos = 0};
const field32_t A_TIMx_DMAR_DMAB           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_RCR_REP             = {.msk = 0x000000FFU, .pos = 0};
const field32_t A_TIMx_BDTR_DTG            = {.msk = 0x000000FFU, .pos = 0};
const field32_t A_TIMx_BDTR_LOCK           = {.msk = 0x00000300U, .pos = 8};
const field32_t A_TIMx_BDTR_OSSI           = {.msk = 0x00000400U, .pos = 10};
const field32_t A_TIMx_BDTR_OSSR           = {.msk = 0x00000800U, .pos = 11};
const field32_t A_TIMx_BDTR_BKE            = {.msk = 0x00001000U, .pos = 12};
const field32_t A_TIMx_BDTR_BKP            = {.msk = 0x00002000U, .pos = 13};
const field32_t A_TIMx_BDTR_AOE            = {.msk = 0x00004000U, .pos = 14};
const field32_t A_TIMx_BDTR_MOE            = {.msk = 0x00008000U, .pos = 15};
const field32_t A_TIMx_BDTR_BKF            = {.msk = 0x000F0000U, .pos = 16};
const field32_t A_TIMx_BDTR_BK2F           = {.msk = 0x00F00000U, .pos = 20};
const field32_t A_TIMx_BDTR_BK2E           = {.msk = 0x01000000U, .pos = 24};
const field32_t A_TIMx_BDTR_BK2P           = {.msk = 0x02000000U, .pos = 25};
const field32_t A_TIMx_CCR5_CCR5           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_CRR6_CCR6           = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t A_TIMx_AF1_BKINE           = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_AF1_BKDF1BK0E       = {.msk = 0x00000100U, .pos = 8};
const field32_t A_TIMx_AF1_BKINP           = {.msk = 0x00000200U, .pos = 9};
const field32_t A_TIMx_AF1_ETRSEL          = {.msk = 0x0003C000U, .pos = 14};
const field32_t A_TIMx_AF2_BK2INE          = {.msk = 0x00000001U, .pos = 0};
const field32_t A_TIMx_AF2_BK2DF1BK1E      = {.msk = 0x00000100U, .pos = 8};
const field32_t A_TIMx_AF2_BK2INP          = {.msk = 0x00000200U, .pos = 9};

/** @subsection Enumerated A_TIMx Register Field Definitions */

field32_t const A_TIMx_CR2_OISx[7] = {
  [1] = {.msk = 0x00000100U, .pos = 8},
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00001000U, .pos = 12},
  [4] = {.msk = 0x00004000U, .pos = 14},
  [5] = {.msk = 0x00010000U, .pos = 16},
  [6] = {.msk = 0x00040000U, .pos = 18},
};

field32_t const A_TIMx_CR2_OISxN[4] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000800U, .pos = 11},
  [3] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const A_TIMx_DIER_CCxDE[5] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const A_TIMx_DIER_CCxIE[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const A_TIMx_SR_CCxIF[7] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000010U, .pos = 4},
  [5] = {.msk = 0x00010000U, .pos = 16},
  [6] = {.msk = 0x00020000U, .pos = 17},
};

field32_t const A_TIMx_SR_CCxOF[5] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const A_TIMx_EGR_CCxG[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const A_TIMx_CCMR1_OUTPUT_CCxS[3] = {
  [1] = {.msk = 0x00000003U, .pos = 0},
  [2] = {.msk = 0x00000300U, .pos = 8},
};

field32_t const A_TIMx_CCMR1_OUTPUT_OCxFE[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const A_TIMx_CCMR1_OUTPUT_OCxPE[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const A_TIMx_CCMR1_OUTPUT_OCxM[3] = {
  [1] = {.msk = 0x00000070U, .pos = 4},
  [2] = {.msk = 0x00007000U, .pos = 12},
};

field32_t const A_TIMx_CCMR1_OUTPUT_OCxCE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const A_TIMx_CCMR1_OUTPUT_OCxM_3[3] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const A_TIMx_CCMR1_INPUT_ICxF[3] = {
  [1] = {.msk = 0x000000F0U, .pos = 4},
  [2] = {.msk = 0x0000F000U, .pos = 12},
};

field32_t const A_TIMx_CCMR2_INPUT_ICxPSC[5] = {
  [3] = {.msk = 0x0000000CU, .pos = 2},
  [4] = {.msk = 0x00000C00U, .pos = 10},
};

field32_t const A_TIMx_CCER_CCxE[7] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000100U, .pos = 8},
  [4] = {.msk = 0x00001000U, .pos = 12},
  [5] = {.msk = 0x00010000U, .pos = 16},
  [6] = {.msk = 0x00100000U, .pos = 20},
};

field32_t const A_TIMx_CCER_CCxP[7] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000020U, .pos = 5},
  [3] = {.msk = 0x00000200U, .pos = 9},
  [4] = {.msk = 0x00002000U, .pos = 13},
  [5] = {.msk = 0x00020000U, .pos = 17},
  [6] = {.msk = 0x00200000U, .pos = 21},
};

field32_t const A_TIMx_CCER_CCxNE[4] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000040U, .pos = 6},
  [3] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const A_TIMx_CCER_CCxNP[5] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const A_TIMx_CCMR3_OUTPUT_OCxM3[7] = {
  [5] = {.msk = 0x00010000U, .pos = 16},
  [6] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const A_TIMx_CCR5_GC5Cx[4] = {
  [1] = {.msk = 0x20000000U, .pos = 29},
  [2] = {.msk = 0x40000000U, .pos = 30},
  [3] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const A_TIMx_AF1_BKCMPxE[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const A_TIMx_AF1_BKCMPxP[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const A_TIMx_AF2_BK2CMPxE[3] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
};

field32_t const A_TIMx_AF2_BK2CMPxP[3] = {
  [1] = {.msk = 0x00000400U, .pos = 10},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const A_TIMx_TISEL_TIxSEL[5] = {
  [1] = {.msk = 0x0000000FU, .pos = 0},
  [2] = {.msk = 0x00000F00U, .pos = 8},
  [3] = {.msk = 0x000F0000U, .pos = 16},
  [4] = {.msk = 0x0F000000U, .pos = 24},
};

/**************************************************************************************************
 * @section FDCANx Definitions
 **************************************************************************************************/

/** @subsection Enumerated FDCANx Register Definitions */

ro_reg32_t const FDCANx_FDCAN_CREL[3] = {
  [1] = (ro_reg32_t)0x4000A000U,
  [2] = (ro_reg32_t)0x4000A400U,
};

ro_reg32_t const FDCANx_FDCAN_ENDN[3] = {
  [1] = (ro_reg32_t)0x4000A004U,
  [2] = (ro_reg32_t)0x4000A404U,
};

ro_reg32_t const FDCANx_FDCAN_DBTP[3] = {
  [1] = (ro_reg32_t)0x4000A00CU,
  [2] = (ro_reg32_t)0x4000A40CU,
};

ro_reg32_t const FDCANx_FDCAN_TEST[3] = {
  [1] = (ro_reg32_t)0x4000A010U,
  [2] = (ro_reg32_t)0x4000A410U,
};

ro_reg32_t const FDCANx_FDCAN_RWD[3] = {
  [1] = (ro_reg32_t)0x4000A014U,
  [2] = (ro_reg32_t)0x4000A414U,
};

rw_reg32_t const FDCANx_FDCAN_CCCR[3] = {
  [1] = (rw_reg32_t)0x4000A018U,
  [2] = (rw_reg32_t)0x4000A418U,
};

rw_reg32_t const FDCANx_FDCAN_NBTP[3] = {
  [1] = (rw_reg32_t)0x4000A01CU,
  [2] = (rw_reg32_t)0x4000A41CU,
};

rw_reg32_t const FDCANx_FDCAN_TSCC[3] = {
  [1] = (rw_reg32_t)0x4000A020U,
  [2] = (rw_reg32_t)0x4000A420U,
};

rw_reg32_t const FDCANx_FDCAN_TSCV[3] = {
  [1] = (rw_reg32_t)0x4000A024U,
  [2] = (rw_reg32_t)0x4000A424U,
};

rw_reg32_t const FDCANx_FDCAN_TOCC[3] = {
  [1] = (rw_reg32_t)0x4000A028U,
  [2] = (rw_reg32_t)0x4000A428U,
};

rw_reg32_t const FDCANx_FDCAN_TOCV[3] = {
  [1] = (rw_reg32_t)0x4000A02CU,
  [2] = (rw_reg32_t)0x4000A42CU,
};

rw_reg32_t const FDCANx_FDCAN_ECR[3] = {
  [1] = (rw_reg32_t)0x4000A040U,
  [2] = (rw_reg32_t)0x4000A440U,
};

rw_reg32_t const FDCANx_FDCAN_PSR[3] = {
  [1] = (rw_reg32_t)0x4000A044U,
  [2] = (rw_reg32_t)0x4000A444U,
};

ro_reg32_t const FDCANx_FDCAN_TDCR[3] = {
  [1] = (ro_reg32_t)0x4000A048U,
  [2] = (ro_reg32_t)0x4000A448U,
};

ro_reg32_t const FDCANx_FDCAN_IR[3] = {
  [1] = (ro_reg32_t)0x4000A050U,
  [2] = (ro_reg32_t)0x4000A450U,
};

ro_reg32_t const FDCANx_FDCAN_IE[3] = {
  [1] = (ro_reg32_t)0x4000A054U,
  [2] = (ro_reg32_t)0x4000A454U,
};

ro_reg32_t const FDCANx_FDCAN_ILS[3] = {
  [1] = (ro_reg32_t)0x4000A058U,
  [2] = (ro_reg32_t)0x4000A458U,
};

rw_reg32_t const FDCANx_FDCAN_ILE[3] = {
  [1] = (rw_reg32_t)0x4000A05CU,
  [2] = (rw_reg32_t)0x4000A45CU,
};

rw_reg32_t const FDCANx_FDCAN_GFC[3] = {
  [1] = (rw_reg32_t)0x4000A080U,
  [2] = (rw_reg32_t)0x4000A480U,
};

rw_reg32_t const FDCANx_FDCAN_SIDFC[3] = {
  [1] = (rw_reg32_t)0x4000A084U,
  [2] = (rw_reg32_t)0x4000A484U,
};

rw_reg32_t const FDCANx_FDCAN_XIDFC[3] = {
  [1] = (rw_reg32_t)0x4000A088U,
  [2] = (rw_reg32_t)0x4000A488U,
};

rw_reg32_t const FDCANx_FDCAN_XIDAM[3] = {
  [1] = (rw_reg32_t)0x4000A090U,
  [2] = (rw_reg32_t)0x4000A490U,
};

ro_reg32_t const FDCANx_FDCAN_HPMS[3] = {
  [1] = (ro_reg32_t)0x4000A094U,
  [2] = (ro_reg32_t)0x4000A494U,
};

ro_reg32_t const FDCANx_FDCAN_NDAT1[3] = {
  [1] = (ro_reg32_t)0x4000A098U,
  [2] = (ro_reg32_t)0x4000A498U,
};

ro_reg32_t const FDCANx_FDCAN_NDAT2[3] = {
  [1] = (ro_reg32_t)0x4000A09CU,
  [2] = (ro_reg32_t)0x4000A49CU,
};

rw_reg32_t const FDCANx_FDCAN_RXF0C[3] = {
  [1] = (rw_reg32_t)0x4000A0A0U,
  [2] = (rw_reg32_t)0x4000A4A0U,
};

rw_reg32_t const FDCANx_FDCAN_RXF0S[3] = {
  [1] = (rw_reg32_t)0x4000A0A4U,
  [2] = (rw_reg32_t)0x4000A4A4U,
};

rw_reg32_t const FDCANx_FDCAN_RXF0A[3] = {
  [1] = (rw_reg32_t)0x4000A0A8U,
  [2] = (rw_reg32_t)0x4000A4A8U,
};

rw_reg32_t const FDCANx_FDCAN_RXBC[3] = {
  [1] = (rw_reg32_t)0x4000A0ACU,
  [2] = (rw_reg32_t)0x4000A4ACU,
};

rw_reg32_t const FDCANx_FDCAN_RXF1C[3] = {
  [1] = (rw_reg32_t)0x4000A0B0U,
  [2] = (rw_reg32_t)0x4000A4B0U,
};

rw_reg32_t const FDCANx_FDCAN_RXF1S[3] = {
  [1] = (rw_reg32_t)0x4000A0B4U,
  [2] = (rw_reg32_t)0x4000A4B4U,
};

rw_reg32_t const FDCANx_FDCAN_RXF1A[3] = {
  [1] = (rw_reg32_t)0x4000A0B8U,
  [2] = (rw_reg32_t)0x4000A4B8U,
};

rw_reg32_t const FDCANx_FDCAN_RXESC[3] = {
  [1] = (rw_reg32_t)0x4000A0BCU,
  [2] = (rw_reg32_t)0x4000A4BCU,
};

rw_reg32_t const FDCANx_FDCAN_TXBC[3] = {
  [1] = (rw_reg32_t)0x4000A0C0U,
  [2] = (rw_reg32_t)0x4000A4C0U,
};

ro_reg32_t const FDCANx_FDCAN_TXFQS[3] = {
  [1] = (ro_reg32_t)0x4000A0C4U,
  [2] = (ro_reg32_t)0x4000A4C4U,
};

rw_reg32_t const FDCANx_FDCAN_TXESC[3] = {
  [1] = (rw_reg32_t)0x4000A0C8U,
  [2] = (rw_reg32_t)0x4000A4C8U,
};

ro_reg32_t const FDCANx_FDCAN_TXBRP[3] = {
  [1] = (ro_reg32_t)0x4000A0CCU,
  [2] = (ro_reg32_t)0x4000A4CCU,
};

rw_reg32_t const FDCANx_FDCAN_TXBAR[3] = {
  [1] = (rw_reg32_t)0x4000A0D0U,
  [2] = (rw_reg32_t)0x4000A4D0U,
};

rw_reg32_t const FDCANx_FDCAN_TXBCR[3] = {
  [1] = (rw_reg32_t)0x4000A0D4U,
  [2] = (rw_reg32_t)0x4000A4D4U,
};

rw_reg32_t const FDCANx_FDCAN_TXBTO[3] = {
  [1] = (rw_reg32_t)0x4000A0D8U,
  [2] = (rw_reg32_t)0x4000A4D8U,
};

ro_reg32_t const FDCANx_FDCAN_TXBCF[3] = {
  [1] = (ro_reg32_t)0x4000A0DCU,
  [2] = (ro_reg32_t)0x4000A4DCU,
};

rw_reg32_t const FDCANx_FDCAN_TXBTIE[3] = {
  [1] = (rw_reg32_t)0x4000A0E0U,
  [2] = (rw_reg32_t)0x4000A4E0U,
};

rw_reg32_t const FDCANx_FDCAN_TXBCIE[3] = {
  [1] = (rw_reg32_t)0x4000A0E4U,
  [2] = (rw_reg32_t)0x4000A4E4U,
};

rw_reg32_t const FDCANx_FDCAN_TXEFC[3] = {
  [1] = (rw_reg32_t)0x4000A0F0U,
  [2] = (rw_reg32_t)0x4000A4F0U,
};

rw_reg32_t const FDCANx_FDCAN_TXEFS[3] = {
  [1] = (rw_reg32_t)0x4000A0F4U,
  [2] = (rw_reg32_t)0x4000A4F4U,
};

rw_reg32_t const FDCANx_FDCAN_TXEFA[3] = {
  [1] = (rw_reg32_t)0x4000A0F8U,
  [2] = (rw_reg32_t)0x4000A4F8U,
};

rw_reg32_t const FDCANx_FDCAN_TTTMC[3] = {
  [1] = (rw_reg32_t)0x4000A100U,
  [2] = (rw_reg32_t)0x4000A500U,
};

rw_reg32_t const FDCANx_FDCAN_TTRMC[3] = {
  [1] = (rw_reg32_t)0x4000A104U,
  [2] = (rw_reg32_t)0x4000A504U,
};

rw_reg32_t const FDCANx_FDCAN_TTOCF[3] = {
  [1] = (rw_reg32_t)0x4000A108U,
  [2] = (rw_reg32_t)0x4000A508U,
};

rw_reg32_t const FDCANx_FDCAN_TTMLM[3] = {
  [1] = (rw_reg32_t)0x4000A10CU,
  [2] = (rw_reg32_t)0x4000A50CU,
};

rw_reg32_t const FDCANx_FDCAN_TURCF[3] = {
  [1] = (rw_reg32_t)0x4000A110U,
  [2] = (rw_reg32_t)0x4000A510U,
};

rw_reg32_t const FDCANx_FDCAN_TTOCN[3] = {
  [1] = (rw_reg32_t)0x4000A114U,
  [2] = (rw_reg32_t)0x4000A514U,
};

rw_reg32_t const FDCANx_CAN_TTGTP[3] = {
  [1] = (rw_reg32_t)0x4000A118U,
  [2] = (rw_reg32_t)0x4000A518U,
};

rw_reg32_t const FDCANx_FDCAN_TTTMK[3] = {
  [1] = (rw_reg32_t)0x4000A11CU,
  [2] = (rw_reg32_t)0x4000A51CU,
};

rw_reg32_t const FDCANx_FDCAN_TTIR[3] = {
  [1] = (rw_reg32_t)0x4000A120U,
  [2] = (rw_reg32_t)0x4000A520U,
};

rw_reg32_t const FDCANx_FDCAN_TTIE[3] = {
  [1] = (rw_reg32_t)0x4000A124U,
  [2] = (rw_reg32_t)0x4000A524U,
};

rw_reg32_t const FDCANx_FDCAN_TTILS[3] = {
  [1] = (rw_reg32_t)0x4000A128U,
  [2] = (rw_reg32_t)0x4000A528U,
};

rw_reg32_t const FDCANx_FDCAN_TTOST[3] = {
  [1] = (rw_reg32_t)0x4000A12CU,
  [2] = (rw_reg32_t)0x4000A52CU,
};

ro_reg32_t const FDCANx_FDCAN_TURNA[3] = {
  [1] = (ro_reg32_t)0x4000A130U,
  [2] = (ro_reg32_t)0x4000A530U,
};

ro_reg32_t const FDCANx_FDCAN_TTLGT[3] = {
  [1] = (ro_reg32_t)0x4000A134U,
  [2] = (ro_reg32_t)0x4000A534U,
};

ro_reg32_t const FDCANx_FDCAN_TTCTC[3] = {
  [1] = (ro_reg32_t)0x4000A138U,
  [2] = (ro_reg32_t)0x4000A538U,
};

ro_reg32_t const FDCANx_FDCAN_TTCPT[3] = {
  [1] = (ro_reg32_t)0x4000A13CU,
  [2] = (ro_reg32_t)0x4000A53CU,
};

ro_reg32_t const FDCANx_FDCAN_TTCSM[3] = {
  [1] = (ro_reg32_t)0x4000A140U,
  [2] = (ro_reg32_t)0x4000A540U,
};

rw_reg32_t const FDCANx_FDCAN_TTTS[3] = {
  [1] = (rw_reg32_t)0x4000A300U,
  [2] = (rw_reg32_t)0x4000A700U,
};

/** @subsection FDCANx Register Field Definitions */

const field32_t FDCANx_FDCAN_CREL_REL     = {.msk = 0xF0000000U, .pos = 28};
const field32_t FDCANx_FDCAN_CREL_STEP    = {.msk = 0x0F000000U, .pos = 24};
const field32_t FDCANx_FDCAN_CREL_SUBSTEP = {.msk = 0x00F00000U, .pos = 20};
const field32_t FDCANx_FDCAN_CREL_YEAR    = {.msk = 0x000F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_CREL_MON     = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FDCANx_FDCAN_CREL_DAY     = {.msk = 0x000000FFU, .pos = 0};
const field32_t FDCANx_FDCAN_DBTP_DSJW    = {.msk = 0x0000000FU, .pos = 0};
const field32_t FDCANx_FDCAN_DBTP_DBRP    = {.msk = 0x001F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_DBTP_TDC     = {.msk = 0x00800000U, .pos = 23};
const field32_t FDCANx_FDCAN_TEST_LBCK    = {.msk = 0x00000010U, .pos = 4};
const field32_t FDCANx_FDCAN_TEST_TX      = {.msk = 0x00000060U, .pos = 5};
const field32_t FDCANx_FDCAN_TEST_RX      = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_RWD_WDV      = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FDCANx_FDCAN_RWD_WDC      = {.msk = 0x000000FFU, .pos = 0};
const field32_t FDCANx_FDCAN_CCCR_INIT    = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_CCCR_CCE     = {.msk = 0x00000002U, .pos = 1};
const field32_t FDCANx_FDCAN_CCCR_ASM     = {.msk = 0x00000004U, .pos = 2};
const field32_t FDCANx_FDCAN_CCCR_CSA     = {.msk = 0x00000008U, .pos = 3};
const field32_t FDCANx_FDCAN_CCCR_CSR     = {.msk = 0x00000010U, .pos = 4};
const field32_t FDCANx_FDCAN_CCCR_MON     = {.msk = 0x00000020U, .pos = 5};
const field32_t FDCANx_FDCAN_CCCR_DAR     = {.msk = 0x00000040U, .pos = 6};
const field32_t FDCANx_FDCAN_CCCR_TEST    = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_CCCR_FDOE    = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_CCCR_BSE     = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_CCCR_PXHD    = {.msk = 0x00001000U, .pos = 12};
const field32_t FDCANx_FDCAN_CCCR_EFBI    = {.msk = 0x00002000U, .pos = 13};
const field32_t FDCANx_FDCAN_CCCR_TXP     = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_CCCR_NISO    = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_NBTP_NSJW    = {.msk = 0xFE000000U, .pos = 25};
const field32_t FDCANx_FDCAN_NBTP_NBRP    = {.msk = 0x01FF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_NBTP_NTSEG1  = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FDCANx_FDCAN_NBTP_TSEG2   = {.msk = 0x0000007FU, .pos = 0};
const field32_t FDCANx_FDCAN_TSCC_TCP     = {.msk = 0x000F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TSCC_TSS     = {.msk = 0x00000003U, .pos = 0};
const field32_t FDCANx_FDCAN_TSCV_TSC     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TOCC_ETOC    = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_TOCC_TOS     = {.msk = 0x00000006U, .pos = 1};
const field32_t FDCANx_FDCAN_TOCC_TOP     = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TOCV_TOC     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_ECR_CEL      = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_ECR_RP       = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_ECR_TREC     = {.msk = 0x00007F00U, .pos = 8};
const field32_t FDCANx_FDCAN_ECR_TEC      = {.msk = 0x000000FFU, .pos = 0};
const field32_t FDCANx_FDCAN_PSR_LEC      = {.msk = 0x00000007U, .pos = 0};
const field32_t FDCANx_FDCAN_PSR_ACT      = {.msk = 0x00000018U, .pos = 3};
const field32_t FDCANx_FDCAN_PSR_EP       = {.msk = 0x00000020U, .pos = 5};
const field32_t FDCANx_FDCAN_PSR_EW       = {.msk = 0x00000040U, .pos = 6};
const field32_t FDCANx_FDCAN_PSR_BO       = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_PSR_DLEC     = {.msk = 0x00000700U, .pos = 8};
const field32_t FDCANx_FDCAN_PSR_RESI     = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_PSR_RBRS     = {.msk = 0x00001000U, .pos = 12};
const field32_t FDCANx_FDCAN_PSR_REDL     = {.msk = 0x00002000U, .pos = 13};
const field32_t FDCANx_FDCAN_PSR_PXE      = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_PSR_TDCV     = {.msk = 0x007F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TDCR_TDCF    = {.msk = 0x0000007FU, .pos = 0};
const field32_t FDCANx_FDCAN_TDCR_TDCO    = {.msk = 0x00007F00U, .pos = 8};
const field32_t FDCANx_FDCAN_IR_HPM       = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_IR_TC        = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_IR_TCF       = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_IR_TEF       = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_IR_TEFN      = {.msk = 0x00001000U, .pos = 12};
const field32_t FDCANx_FDCAN_IR_TEFW      = {.msk = 0x00002000U, .pos = 13};
const field32_t FDCANx_FDCAN_IR_TEFF      = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_IR_TEFL      = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_IR_TSW       = {.msk = 0x00010000U, .pos = 16};
const field32_t FDCANx_FDCAN_IR_MRAF      = {.msk = 0x00020000U, .pos = 17};
const field32_t FDCANx_FDCAN_IR_TOO       = {.msk = 0x00040000U, .pos = 18};
const field32_t FDCANx_FDCAN_IR_DRX       = {.msk = 0x00080000U, .pos = 19};
const field32_t FDCANx_FDCAN_IR_ELO       = {.msk = 0x00400000U, .pos = 22};
const field32_t FDCANx_FDCAN_IR_EP        = {.msk = 0x00800000U, .pos = 23};
const field32_t FDCANx_FDCAN_IR_EW        = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_IR_BO        = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_IR_WDI       = {.msk = 0x04000000U, .pos = 26};
const field32_t FDCANx_FDCAN_IR_PEA       = {.msk = 0x08000000U, .pos = 27};
const field32_t FDCANx_FDCAN_IR_PED       = {.msk = 0x10000000U, .pos = 28};
const field32_t FDCANx_FDCAN_IR_ARA       = {.msk = 0x20000000U, .pos = 29};
const field32_t FDCANx_FDCAN_IE_HPME      = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_IE_TCE       = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_IE_TCFE      = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_IE_TEFE      = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_IE_TEFNE     = {.msk = 0x00001000U, .pos = 12};
const field32_t FDCANx_FDCAN_IE_TEFWE     = {.msk = 0x00002000U, .pos = 13};
const field32_t FDCANx_FDCAN_IE_TEFFE     = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_IE_TEFLE     = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_IE_TSWE      = {.msk = 0x00010000U, .pos = 16};
const field32_t FDCANx_FDCAN_IE_MRAFE     = {.msk = 0x00020000U, .pos = 17};
const field32_t FDCANx_FDCAN_IE_TOOE      = {.msk = 0x00040000U, .pos = 18};
const field32_t FDCANx_FDCAN_IE_DRXE      = {.msk = 0x00080000U, .pos = 19};
const field32_t FDCANx_FDCAN_IE_BECE      = {.msk = 0x00100000U, .pos = 20};
const field32_t FDCANx_FDCAN_IE_BEUE      = {.msk = 0x00200000U, .pos = 21};
const field32_t FDCANx_FDCAN_IE_ELOE      = {.msk = 0x00400000U, .pos = 22};
const field32_t FDCANx_FDCAN_IE_EPE       = {.msk = 0x00800000U, .pos = 23};
const field32_t FDCANx_FDCAN_IE_EWE       = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_IE_BOE       = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_IE_WDIE      = {.msk = 0x04000000U, .pos = 26};
const field32_t FDCANx_FDCAN_IE_PEAE      = {.msk = 0x08000000U, .pos = 27};
const field32_t FDCANx_FDCAN_IE_PEDE      = {.msk = 0x10000000U, .pos = 28};
const field32_t FDCANx_FDCAN_IE_ARAE      = {.msk = 0x20000000U, .pos = 29};
const field32_t FDCANx_FDCAN_ILS_HPML     = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_ILS_TCL      = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_ILS_TCFL     = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_ILS_TEFL     = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_ILS_TEFNL    = {.msk = 0x00001000U, .pos = 12};
const field32_t FDCANx_FDCAN_ILS_TEFWL    = {.msk = 0x00002000U, .pos = 13};
const field32_t FDCANx_FDCAN_ILS_TEFFL    = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_ILS_TEFLL    = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_ILS_TSWL     = {.msk = 0x00010000U, .pos = 16};
const field32_t FDCANx_FDCAN_ILS_MRAFL    = {.msk = 0x00020000U, .pos = 17};
const field32_t FDCANx_FDCAN_ILS_TOOL     = {.msk = 0x00040000U, .pos = 18};
const field32_t FDCANx_FDCAN_ILS_DRXL     = {.msk = 0x00080000U, .pos = 19};
const field32_t FDCANx_FDCAN_ILS_BECL     = {.msk = 0x00100000U, .pos = 20};
const field32_t FDCANx_FDCAN_ILS_BEUL     = {.msk = 0x00200000U, .pos = 21};
const field32_t FDCANx_FDCAN_ILS_ELOL     = {.msk = 0x00400000U, .pos = 22};
const field32_t FDCANx_FDCAN_ILS_EPL      = {.msk = 0x00800000U, .pos = 23};
const field32_t FDCANx_FDCAN_ILS_EWL      = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_ILS_BOL      = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_ILS_WDIL     = {.msk = 0x04000000U, .pos = 26};
const field32_t FDCANx_FDCAN_ILS_PEAL     = {.msk = 0x08000000U, .pos = 27};
const field32_t FDCANx_FDCAN_ILS_PEDL     = {.msk = 0x10000000U, .pos = 28};
const field32_t FDCANx_FDCAN_ILS_ARAL     = {.msk = 0x20000000U, .pos = 29};
const field32_t FDCANx_FDCAN_GFC_RRFE     = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_GFC_RRFS     = {.msk = 0x00000002U, .pos = 1};
const field32_t FDCANx_FDCAN_GFC_ANFE     = {.msk = 0x0000000CU, .pos = 2};
const field32_t FDCANx_FDCAN_GFC_ANFS     = {.msk = 0x00000030U, .pos = 4};
const field32_t FDCANx_FDCAN_SIDFC_FLSSA  = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_SIDFC_LSS    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_XIDFC_FLESA  = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_XIDFC_LSE    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_XIDAM_EIDM   = {.msk = 0x1FFFFFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_HPMS_BIDX    = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_HPMS_MSI     = {.msk = 0x000000C0U, .pos = 6};
const field32_t FDCANx_FDCAN_HPMS_FIDX    = {.msk = 0x00007F00U, .pos = 8};
const field32_t FDCANx_FDCAN_HPMS_FLST    = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_RXF0C_F0SA   = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_RXF0C_F0S    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_RXF0C_F0WM   = {.msk = 0xFF000000U, .pos = 24};
const field32_t FDCANx_FDCAN_RXF0S_F0FL   = {.msk = 0x0000007FU, .pos = 0};
const field32_t FDCANx_FDCAN_RXF0S_F0G    = {.msk = 0x00003F00U, .pos = 8};
const field32_t FDCANx_FDCAN_RXF0S_F0P    = {.msk = 0x003F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_RXF0S_F0F    = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_RXF0S_RF0L   = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_RXF0A_FA01   = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_RXBC_RBSA    = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_RXF1C_F1SA   = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_RXF1C_F1S    = {.msk = 0x007F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_RXF1C_F1WM   = {.msk = 0x7F000000U, .pos = 24};
const field32_t FDCANx_FDCAN_RXF1S_F1FL   = {.msk = 0x0000007FU, .pos = 0};
const field32_t FDCANx_FDCAN_RXF1S_F1GI   = {.msk = 0x00007F00U, .pos = 8};
const field32_t FDCANx_FDCAN_RXF1S_F1PI   = {.msk = 0x007F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_RXF1S_F1F    = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_RXF1S_RF1L   = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_RXF1S_DMS    = {.msk = 0xC0000000U, .pos = 30};
const field32_t FDCANx_FDCAN_RXF1A_F1AI   = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_RXESC_RBDS   = {.msk = 0x00000700U, .pos = 8};
const field32_t FDCANx_FDCAN_TXBC_TBSA    = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_TXBC_NDTB    = {.msk = 0x003F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TXBC_TFQS    = {.msk = 0x3F000000U, .pos = 24};
const field32_t FDCANx_FDCAN_TXBC_TFQM    = {.msk = 0x40000000U, .pos = 30};
const field32_t FDCANx_FDCAN_TXFQS_TFFL   = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_TXFQS_TFGI   = {.msk = 0x00001F00U, .pos = 8};
const field32_t FDCANx_FDCAN_TXFQS_TFQPI  = {.msk = 0x001F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TXFQS_TFQF   = {.msk = 0x00200000U, .pos = 21};
const field32_t FDCANx_FDCAN_TXESC_TBDS   = {.msk = 0x00000007U, .pos = 0};
const field32_t FDCANx_FDCAN_TXEFC_EFSA   = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_TXEFC_EFS    = {.msk = 0x003F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TXEFC_EFWM   = {.msk = 0x3F000000U, .pos = 24};
const field32_t FDCANx_FDCAN_TXEFS_EFFL   = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_TXEFS_EFGI   = {.msk = 0x00001F00U, .pos = 8};
const field32_t FDCANx_FDCAN_TXEFS_EFF    = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_TXEFS_TEFL   = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_TXEFA_EFAI   = {.msk = 0x0000001FU, .pos = 0};
const field32_t FDCANx_FDCAN_TTTMC_TMSA   = {.msk = 0x0000FFFCU, .pos = 2};
const field32_t FDCANx_FDCAN_TTTMC_TME    = {.msk = 0x007F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTRMC_RID    = {.msk = 0x1FFFFFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TTRMC_XTD    = {.msk = 0x40000000U, .pos = 30};
const field32_t FDCANx_FDCAN_TTRMC_RMPS   = {.msk = 0x80000000U, .pos = 31};
const field32_t FDCANx_FDCAN_TTOCF_OM     = {.msk = 0x00000003U, .pos = 0};
const field32_t FDCANx_FDCAN_TTOCF_GEN    = {.msk = 0x00000008U, .pos = 3};
const field32_t FDCANx_FDCAN_TTOCF_TM     = {.msk = 0x00000010U, .pos = 4};
const field32_t FDCANx_FDCAN_TTOCF_LDSDL  = {.msk = 0x000000E0U, .pos = 5};
const field32_t FDCANx_FDCAN_TTOCF_IRTO   = {.msk = 0x00007F00U, .pos = 8};
const field32_t FDCANx_FDCAN_TTOCF_EECS   = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_TTOCF_AWL    = {.msk = 0x00FF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTOCF_EGTF   = {.msk = 0x01000000U, .pos = 24};
const field32_t FDCANx_FDCAN_TTOCF_ECC    = {.msk = 0x02000000U, .pos = 25};
const field32_t FDCANx_FDCAN_TTOCF_EVTP   = {.msk = 0x04000000U, .pos = 26};
const field32_t FDCANx_FDCAN_TTMLM_CCM    = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_TTMLM_CSS    = {.msk = 0x000000C0U, .pos = 6};
const field32_t FDCANx_FDCAN_TTMLM_TXEW   = {.msk = 0x00000F00U, .pos = 8};
const field32_t FDCANx_FDCAN_TTMLM_ENTT   = {.msk = 0x0FFF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TURCF_NCL    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TURCF_DC     = {.msk = 0x3FFF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TURCF_ELT    = {.msk = 0x80000000U, .pos = 31};
const field32_t FDCANx_FDCAN_TTOCN_SGT    = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_TTOCN_ECS    = {.msk = 0x00000002U, .pos = 1};
const field32_t FDCANx_FDCAN_TTOCN_SWP    = {.msk = 0x00000004U, .pos = 2};
const field32_t FDCANx_FDCAN_TTOCN_SWS    = {.msk = 0x00000018U, .pos = 3};
const field32_t FDCANx_FDCAN_TTOCN_RTIE   = {.msk = 0x00000020U, .pos = 5};
const field32_t FDCANx_FDCAN_TTOCN_TMC    = {.msk = 0x000000C0U, .pos = 6};
const field32_t FDCANx_FDCAN_TTOCN_TTIE   = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_TTOCN_GCS    = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_TTOCN_FGP    = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_TTOCN_TMG    = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_TTOCN_NIG    = {.msk = 0x00001000U, .pos = 12};
const field32_t FDCANx_FDCAN_TTOCN_ESCN   = {.msk = 0x00002000U, .pos = 13};
const field32_t FDCANx_FDCAN_TTOCN_LCKC   = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_CAN_TTGTP_NCL      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_CAN_TTGTP_CTP      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTTMK_TM     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TTTMK_TICC   = {.msk = 0x007F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTTMK_LCKM   = {.msk = 0x80000000U, .pos = 31};
const field32_t FDCANx_FDCAN_TTIR_SBC     = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_TTIR_SMC     = {.msk = 0x00000002U, .pos = 1};
const field32_t FDCANx_FDCAN_TTIR_CSM     = {.msk = 0x00000004U, .pos = 2};
const field32_t FDCANx_FDCAN_TTIR_SOG     = {.msk = 0x00000008U, .pos = 3};
const field32_t FDCANx_FDCAN_TTIR_RTMI    = {.msk = 0x00000010U, .pos = 4};
const field32_t FDCANx_FDCAN_TTIR_TTMI    = {.msk = 0x00000020U, .pos = 5};
const field32_t FDCANx_FDCAN_TTIR_SWE     = {.msk = 0x00000040U, .pos = 6};
const field32_t FDCANx_FDCAN_TTIR_GTW     = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_TTIR_GTD     = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_TTIR_GTE     = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_TTIR_TXU     = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_TTIR_TXO     = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_TTIR_ELC     = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_TTIR_IWTG    = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_TTIR_WT      = {.msk = 0x00010000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTIR_AW      = {.msk = 0x00020000U, .pos = 17};
const field32_t FDCANx_FDCAN_TTIR_CER     = {.msk = 0x00040000U, .pos = 18};
const field32_t FDCANx_FDCAN_TTIE_SBCE    = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_TTIE_SMCE    = {.msk = 0x00000002U, .pos = 1};
const field32_t FDCANx_FDCAN_TTIE_CSME    = {.msk = 0x00000004U, .pos = 2};
const field32_t FDCANx_FDCAN_TTIE_SOGE    = {.msk = 0x00000008U, .pos = 3};
const field32_t FDCANx_FDCAN_TTIE_RTMIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t FDCANx_FDCAN_TTIE_TTMIE   = {.msk = 0x00000020U, .pos = 5};
const field32_t FDCANx_FDCAN_TTIE_SWEE    = {.msk = 0x00000040U, .pos = 6};
const field32_t FDCANx_FDCAN_TTIE_GTWE    = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_TTIE_GTDE    = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_TTIE_GTEE    = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_TTIE_TXUE    = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_TTIE_TXOE    = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_TTIE_ELCE    = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_TTIE_IWTGE   = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_TTIE_WTE     = {.msk = 0x00010000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTIE_AWE     = {.msk = 0x00020000U, .pos = 17};
const field32_t FDCANx_FDCAN_TTIE_CERE    = {.msk = 0x00040000U, .pos = 18};
const field32_t FDCANx_FDCAN_TTILS_SBCL   = {.msk = 0x00000001U, .pos = 0};
const field32_t FDCANx_FDCAN_TTILS_SMCL   = {.msk = 0x00000002U, .pos = 1};
const field32_t FDCANx_FDCAN_TTILS_CSML   = {.msk = 0x00000004U, .pos = 2};
const field32_t FDCANx_FDCAN_TTILS_SOGL   = {.msk = 0x00000008U, .pos = 3};
const field32_t FDCANx_FDCAN_TTILS_RTMIL  = {.msk = 0x00000010U, .pos = 4};
const field32_t FDCANx_FDCAN_TTILS_TTMIL  = {.msk = 0x00000020U, .pos = 5};
const field32_t FDCANx_FDCAN_TTILS_SWEL   = {.msk = 0x00000040U, .pos = 6};
const field32_t FDCANx_FDCAN_TTILS_GTWL   = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_TTILS_GTDL   = {.msk = 0x00000100U, .pos = 8};
const field32_t FDCANx_FDCAN_TTILS_GTEL   = {.msk = 0x00000200U, .pos = 9};
const field32_t FDCANx_FDCAN_TTILS_TXUL   = {.msk = 0x00000400U, .pos = 10};
const field32_t FDCANx_FDCAN_TTILS_TXOL   = {.msk = 0x00000800U, .pos = 11};
const field32_t FDCANx_FDCAN_TTILS_ELCL   = {.msk = 0x00004000U, .pos = 14};
const field32_t FDCANx_FDCAN_TTILS_IWTGL  = {.msk = 0x00008000U, .pos = 15};
const field32_t FDCANx_FDCAN_TTILS_WTL    = {.msk = 0x00010000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTILS_AWL    = {.msk = 0x00020000U, .pos = 17};
const field32_t FDCANx_FDCAN_TTILS_CERL   = {.msk = 0x00040000U, .pos = 18};
const field32_t FDCANx_FDCAN_TTOST_EL     = {.msk = 0x00000003U, .pos = 0};
const field32_t FDCANx_FDCAN_TTOST_MS     = {.msk = 0x0000000CU, .pos = 2};
const field32_t FDCANx_FDCAN_TTOST_SYS    = {.msk = 0x00000030U, .pos = 4};
const field32_t FDCANx_FDCAN_TTOST_GTP    = {.msk = 0x00000040U, .pos = 6};
const field32_t FDCANx_FDCAN_TTOST_QCS    = {.msk = 0x00000080U, .pos = 7};
const field32_t FDCANx_FDCAN_TTOST_RTO    = {.msk = 0x0000FF00U, .pos = 8};
const field32_t FDCANx_FDCAN_TTOST_WGTD   = {.msk = 0x00400000U, .pos = 22};
const field32_t FDCANx_FDCAN_TTOST_GFI    = {.msk = 0x00800000U, .pos = 23};
const field32_t FDCANx_FDCAN_TTOST_TMP    = {.msk = 0x07000000U, .pos = 24};
const field32_t FDCANx_FDCAN_TTOST_GSI    = {.msk = 0x08000000U, .pos = 27};
const field32_t FDCANx_FDCAN_TTOST_WFE    = {.msk = 0x10000000U, .pos = 28};
const field32_t FDCANx_FDCAN_TTOST_AWE    = {.msk = 0x20000000U, .pos = 29};
const field32_t FDCANx_FDCAN_TTOST_WECS   = {.msk = 0x40000000U, .pos = 30};
const field32_t FDCANx_FDCAN_TTOST_SPL    = {.msk = 0x80000000U, .pos = 31};
const field32_t FDCANx_FDCAN_TURNA_NAV    = {.msk = 0x0003FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TTLGT_LT     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TTLGT_GT     = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTCTC_CT     = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TTCTC_CC     = {.msk = 0x003F0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTCPT_CT     = {.msk = 0x0000003FU, .pos = 0};
const field32_t FDCANx_FDCAN_TTCPT_SWV    = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t FDCANx_FDCAN_TTCSM_CSM    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t FDCANx_FDCAN_TTTS_SWTDEL  = {.msk = 0x00000003U, .pos = 0};
const field32_t FDCANx_FDCAN_TTTS_EVTSEL  = {.msk = 0x00000030U, .pos = 4};

/** @subsection Enumerated FDCANx Register Field Definitions */

field32_t const FDCANx_FDCAN_DBTP_DTSEGx[3] = {
  [1] = {.msk = 0x00001F00U, .pos = 8},
  [2] = {.msk = 0x000000F0U, .pos = 4},
};

field32_t const FDCANx_FDCAN_IR_RFxN[2] = {
  [0] = {.msk = 0x00000001U, .pos = 0},
  [1] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const FDCANx_FDCAN_IR_RFxW[2] = {
  [0] = {.msk = 0x00000002U, .pos = 1},
  [1] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const FDCANx_FDCAN_IR_RFxF[2] = {
  [0] = {.msk = 0x00000004U, .pos = 2},
  [1] = {.msk = 0x00000040U, .pos = 6},
};

field32_t const FDCANx_FDCAN_IR_RFxL[2] = {
  [0] = {.msk = 0x00000008U, .pos = 3},
  [1] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const FDCANx_FDCAN_IE_RFxNE[2] = {
  [0] = {.msk = 0x00000001U, .pos = 0},
  [1] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const FDCANx_FDCAN_IE_RFxWE[2] = {
  [0] = {.msk = 0x00000002U, .pos = 1},
  [1] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const FDCANx_FDCAN_IE_RFxFE[2] = {
  [0] = {.msk = 0x00000004U, .pos = 2},
  [1] = {.msk = 0x00000040U, .pos = 6},
};

field32_t const FDCANx_FDCAN_IE_RFxLE[2] = {
  [0] = {.msk = 0x00000008U, .pos = 3},
  [1] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const FDCANx_FDCAN_ILS_RFxNL[2] = {
  [0] = {.msk = 0x00000001U, .pos = 0},
  [1] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const FDCANx_FDCAN_ILS_RFxWL[2] = {
  [0] = {.msk = 0x00000002U, .pos = 1},
  [1] = {.msk = 0x00000020U, .pos = 5},
};

field32_t const FDCANx_FDCAN_ILS_RFxFL[2] = {
  [0] = {.msk = 0x00000004U, .pos = 2},
  [1] = {.msk = 0x00000040U, .pos = 6},
};

field32_t const FDCANx_FDCAN_ILS_RFxLL[2] = {
  [0] = {.msk = 0x00000008U, .pos = 3},
  [1] = {.msk = 0x00000080U, .pos = 7},
};

field32_t const FDCANx_FDCAN_ILE_EINTx[2] = {
  [0] = {.msk = 0x00000001U, .pos = 0},
  [1] = {.msk = 0x00000002U, .pos = 1},
};

field32_t const FDCANx_FDCAN_NDAT1_NDx[32] = {
  [0]  = {.msk = 0x00000001U, .pos = 0},
  [1]  = {.msk = 0x00000002U, .pos = 1},
  [2]  = {.msk = 0x00000004U, .pos = 2},
  [3]  = {.msk = 0x00000008U, .pos = 3},
  [4]  = {.msk = 0x00000010U, .pos = 4},
  [5]  = {.msk = 0x00000020U, .pos = 5},
  [6]  = {.msk = 0x00000040U, .pos = 6},
  [7]  = {.msk = 0x00000080U, .pos = 7},
  [8]  = {.msk = 0x00000100U, .pos = 8},
  [9]  = {.msk = 0x00000200U, .pos = 9},
  [10] = {.msk = 0x00000400U, .pos = 10},
  [11] = {.msk = 0x00000800U, .pos = 11},
  [12] = {.msk = 0x00001000U, .pos = 12},
  [13] = {.msk = 0x00002000U, .pos = 13},
  [14] = {.msk = 0x00004000U, .pos = 14},
  [15] = {.msk = 0x00008000U, .pos = 15},
  [16] = {.msk = 0x00010000U, .pos = 16},
  [17] = {.msk = 0x00020000U, .pos = 17},
  [18] = {.msk = 0x00040000U, .pos = 18},
  [19] = {.msk = 0x00080000U, .pos = 19},
  [20] = {.msk = 0x00100000U, .pos = 20},
  [21] = {.msk = 0x00200000U, .pos = 21},
  [22] = {.msk = 0x00400000U, .pos = 22},
  [23] = {.msk = 0x00800000U, .pos = 23},
  [24] = {.msk = 0x01000000U, .pos = 24},
  [25] = {.msk = 0x02000000U, .pos = 25},
  [26] = {.msk = 0x04000000U, .pos = 26},
  [27] = {.msk = 0x08000000U, .pos = 27},
  [28] = {.msk = 0x10000000U, .pos = 28},
  [29] = {.msk = 0x20000000U, .pos = 29},
  [30] = {.msk = 0x40000000U, .pos = 30},
  [31] = {.msk = 0x80000000U, .pos = 31},
};

field32_t const FDCANx_FDCAN_NDAT2_ND3x[10] = {
  [3]  = {.msk = 0x00000002U, .pos = 1},
  [4]  = {.msk = 0x00000004U, .pos = 2},
  [5]  = {.msk = 0x00000008U, .pos = 3},
  [6]  = {.msk = 0x00000010U, .pos = 4},
  [7]  = {.msk = 0x00000020U, .pos = 5},
  [8]  = {.msk = 0x00000040U, .pos = 6},
  [9]  = {.msk = 0x00000080U, .pos = 7},
};

field32_t const FDCANx_FDCAN_RXESC_FxDS[2] = {
  [0] = {.msk = 0x00000007U, .pos = 0},
  [1] = {.msk = 0x00000070U, .pos = 4},
};

field32_t const FDCANx_FDCAN_TTIR_SEx[3] = {
  [1] = {.msk = 0x00001000U, .pos = 12},
  [2] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const FDCANx_FDCAN_TTIE_SExE[3] = {
  [1] = {.msk = 0x00001000U, .pos = 12},
  [2] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const FDCANx_FDCAN_TTILS_SExL[3] = {
  [1] = {.msk = 0x00001000U, .pos = 12},
  [2] = {.msk = 0x00002000U, .pos = 13},
};

/**************************************************************************************************
 * @section CAN_CCU Definitions
 **************************************************************************************************/

/** @subsection CAN_CCU Register Definitions */

rw_reg32_t const CAN_CCU_CREL  = (rw_reg32_t)0x4000A800U;
rw_reg32_t const CAN_CCU_CCFG  = (rw_reg32_t)0x4000A804U;
rw_reg32_t const CAN_CCU_CSTAT = (rw_reg32_t)0x4000A808U;
rw_reg32_t const CAN_CCU_CWD   = (rw_reg32_t)0x4000A80CU;
rw_reg32_t const CAN_CCU_IR    = (rw_reg32_t)0x4000A810U;
rw_reg32_t const CAN_CCU_IE    = (rw_reg32_t)0x4000A814U;

/** @subsection CAN_CCU Register Field Definitions */

const field32_t CAN_CCU_CREL_DAY     = {.msk = 0x000000FFU, .pos = 0};
const field32_t CAN_CCU_CREL_MON     = {.msk = 0x0000FF00U, .pos = 8};
const field32_t CAN_CCU_CREL_YEAR    = {.msk = 0x000F0000U, .pos = 16};
const field32_t CAN_CCU_CREL_SUBSTEP = {.msk = 0x00F00000U, .pos = 20};
const field32_t CAN_CCU_CREL_STEP    = {.msk = 0x0F000000U, .pos = 24};
const field32_t CAN_CCU_CREL_REL     = {.msk = 0xF0000000U, .pos = 28};
const field32_t CAN_CCU_CCFG_TQBT    = {.msk = 0x0000001FU, .pos = 0};
const field32_t CAN_CCU_CCFG_BCC     = {.msk = 0x00000040U, .pos = 6};
const field32_t CAN_CCU_CCFG_CFL     = {.msk = 0x00000080U, .pos = 7};
const field32_t CAN_CCU_CCFG_OCPM    = {.msk = 0x0000FF00U, .pos = 8};
const field32_t CAN_CCU_CCFG_CDIV    = {.msk = 0x000F0000U, .pos = 16};
const field32_t CAN_CCU_CCFG_SWR     = {.msk = 0x80000000U, .pos = 31};
const field32_t CAN_CCU_CSTAT_OCPC   = {.msk = 0x0003FFFFU, .pos = 0};
const field32_t CAN_CCU_CSTAT_TQC    = {.msk = 0x1FFC0000U, .pos = 18};
const field32_t CAN_CCU_CSTAT_CALS   = {.msk = 0xC0000000U, .pos = 30};
const field32_t CAN_CCU_CWD_WDC      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t CAN_CCU_CWD_WDV      = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t CAN_CCU_IR_CWE       = {.msk = 0x00000001U, .pos = 0};
const field32_t CAN_CCU_IR_CSC       = {.msk = 0x00000002U, .pos = 1};
const field32_t CAN_CCU_IE_CWEE      = {.msk = 0x00000001U, .pos = 0};
const field32_t CAN_CCU_IE_CSCE      = {.msk = 0x00000002U, .pos = 1};

/**************************************************************************************************
 * @section MDIOS Definitions
 **************************************************************************************************/

/** @subsection MDIOS Register Definitions */

rw_reg32_t const MDIOS_MDIOS_CR      = (rw_reg32_t)0x40009400U;
ro_reg32_t const MDIOS_MDIOS_WRFR    = (ro_reg32_t)0x40009404U;
rw_reg32_t const MDIOS_MDIOS_CWRFR   = (rw_reg32_t)0x40009408U;
ro_reg32_t const MDIOS_MDIOS_RDFR    = (ro_reg32_t)0x4000940CU;
rw_reg32_t const MDIOS_MDIOS_CRDFR   = (rw_reg32_t)0x40009410U;
ro_reg32_t const MDIOS_MDIOS_SR      = (ro_reg32_t)0x40009414U;
rw_reg32_t const MDIOS_MDIOS_CLRFR   = (rw_reg32_t)0x40009418U;
ro_reg32_t const MDIOS_MDIOS_DINR0   = (ro_reg32_t)0x4000941CU;
ro_reg32_t const MDIOS_MDIOS_DINR1   = (ro_reg32_t)0x40009420U;
ro_reg32_t const MDIOS_MDIOS_DINR2   = (ro_reg32_t)0x40009424U;
ro_reg32_t const MDIOS_MDIOS_DINR3   = (ro_reg32_t)0x40009428U;
ro_reg32_t const MDIOS_MDIOS_DINR4   = (ro_reg32_t)0x4000942CU;
ro_reg32_t const MDIOS_MDIOS_DINR5   = (ro_reg32_t)0x40009430U;
ro_reg32_t const MDIOS_MDIOS_DINR6   = (ro_reg32_t)0x40009434U;
ro_reg32_t const MDIOS_MDIOS_DINR7   = (ro_reg32_t)0x40009438U;
ro_reg32_t const MDIOS_MDIOS_DINR8   = (ro_reg32_t)0x4000943CU;
ro_reg32_t const MDIOS_MDIOS_DINR9   = (ro_reg32_t)0x40009440U;
ro_reg32_t const MDIOS_MDIOS_DINR10  = (ro_reg32_t)0x40009444U;
ro_reg32_t const MDIOS_MDIOS_DINR11  = (ro_reg32_t)0x40009448U;
ro_reg32_t const MDIOS_MDIOS_DINR12  = (ro_reg32_t)0x4000944CU;
ro_reg32_t const MDIOS_MDIOS_DINR13  = (ro_reg32_t)0x40009450U;
ro_reg32_t const MDIOS_MDIOS_DINR14  = (ro_reg32_t)0x40009454U;
ro_reg32_t const MDIOS_MDIOS_DINR15  = (ro_reg32_t)0x40009458U;
ro_reg32_t const MDIOS_MDIOS_DINR16  = (ro_reg32_t)0x4000945CU;
ro_reg32_t const MDIOS_MDIOS_DINR17  = (ro_reg32_t)0x40009460U;
ro_reg32_t const MDIOS_MDIOS_DINR18  = (ro_reg32_t)0x40009464U;
ro_reg32_t const MDIOS_MDIOS_DINR19  = (ro_reg32_t)0x40009468U;
ro_reg32_t const MDIOS_MDIOS_DINR20  = (ro_reg32_t)0x4000946CU;
ro_reg32_t const MDIOS_MDIOS_DINR21  = (ro_reg32_t)0x40009470U;
ro_reg32_t const MDIOS_MDIOS_DINR22  = (ro_reg32_t)0x40009474U;
ro_reg32_t const MDIOS_MDIOS_DINR23  = (ro_reg32_t)0x40009478U;
ro_reg32_t const MDIOS_MDIOS_DINR24  = (ro_reg32_t)0x4000947CU;
ro_reg32_t const MDIOS_MDIOS_DINR25  = (ro_reg32_t)0x40009480U;
ro_reg32_t const MDIOS_MDIOS_DINR26  = (ro_reg32_t)0x40009484U;
ro_reg32_t const MDIOS_MDIOS_DINR27  = (ro_reg32_t)0x40009488U;
ro_reg32_t const MDIOS_MDIOS_DINR28  = (ro_reg32_t)0x4000948CU;
ro_reg32_t const MDIOS_MDIOS_DINR29  = (ro_reg32_t)0x40009490U;
ro_reg32_t const MDIOS_MDIOS_DINR30  = (ro_reg32_t)0x40009494U;
ro_reg32_t const MDIOS_MDIOS_DINR31  = (ro_reg32_t)0x40009498U;
rw_reg32_t const MDIOS_MDIOS_DOUTR0  = (rw_reg32_t)0x4000949CU;
rw_reg32_t const MDIOS_MDIOS_DOUTR1  = (rw_reg32_t)0x400094A0U;
rw_reg32_t const MDIOS_MDIOS_DOUTR2  = (rw_reg32_t)0x400094A4U;
rw_reg32_t const MDIOS_MDIOS_DOUTR3  = (rw_reg32_t)0x400094A8U;
rw_reg32_t const MDIOS_MDIOS_DOUTR4  = (rw_reg32_t)0x400094ACU;
rw_reg32_t const MDIOS_MDIOS_DOUTR5  = (rw_reg32_t)0x400094B0U;
rw_reg32_t const MDIOS_MDIOS_DOUTR6  = (rw_reg32_t)0x400094B4U;
rw_reg32_t const MDIOS_MDIOS_DOUTR7  = (rw_reg32_t)0x400094B8U;
rw_reg32_t const MDIOS_MDIOS_DOUTR8  = (rw_reg32_t)0x400094BCU;
rw_reg32_t const MDIOS_MDIOS_DOUTR9  = (rw_reg32_t)0x400094C0U;
rw_reg32_t const MDIOS_MDIOS_DOUTR10 = (rw_reg32_t)0x400094C4U;
rw_reg32_t const MDIOS_MDIOS_DOUTR11 = (rw_reg32_t)0x400094C8U;
rw_reg32_t const MDIOS_MDIOS_DOUTR12 = (rw_reg32_t)0x400094CCU;
rw_reg32_t const MDIOS_MDIOS_DOUTR13 = (rw_reg32_t)0x400094D0U;
rw_reg32_t const MDIOS_MDIOS_DOUTR14 = (rw_reg32_t)0x400094D4U;
rw_reg32_t const MDIOS_MDIOS_DOUTR15 = (rw_reg32_t)0x400094D8U;
rw_reg32_t const MDIOS_MDIOS_DOUTR16 = (rw_reg32_t)0x400094DCU;
rw_reg32_t const MDIOS_MDIOS_DOUTR17 = (rw_reg32_t)0x400094E0U;
rw_reg32_t const MDIOS_MDIOS_DOUTR18 = (rw_reg32_t)0x400094E4U;
rw_reg32_t const MDIOS_MDIOS_DOUTR19 = (rw_reg32_t)0x400094E8U;
rw_reg32_t const MDIOS_MDIOS_DOUTR20 = (rw_reg32_t)0x400094ECU;
rw_reg32_t const MDIOS_MDIOS_DOUTR21 = (rw_reg32_t)0x400094F0U;
rw_reg32_t const MDIOS_MDIOS_DOUTR22 = (rw_reg32_t)0x400094F4U;
rw_reg32_t const MDIOS_MDIOS_DOUTR23 = (rw_reg32_t)0x400094F8U;
rw_reg32_t const MDIOS_MDIOS_DOUTR24 = (rw_reg32_t)0x400094FCU;
rw_reg32_t const MDIOS_MDIOS_DOUTR25 = (rw_reg32_t)0x40009500U;
rw_reg32_t const MDIOS_MDIOS_DOUTR26 = (rw_reg32_t)0x40009504U;
rw_reg32_t const MDIOS_MDIOS_DOUTR27 = (rw_reg32_t)0x40009508U;
rw_reg32_t const MDIOS_MDIOS_DOUTR28 = (rw_reg32_t)0x4000950CU;
rw_reg32_t const MDIOS_MDIOS_DOUTR29 = (rw_reg32_t)0x40009510U;
rw_reg32_t const MDIOS_MDIOS_DOUTR30 = (rw_reg32_t)0x40009514U;
rw_reg32_t const MDIOS_MDIOS_DOUTR31 = (rw_reg32_t)0x40009518U;

/** @subsection MDIOS Register Field Definitions */

const field32_t MDIOS_MDIOS_CR_EN           = {.msk = 0x00000001U, .pos = 0};
const field32_t MDIOS_MDIOS_CR_WRIE         = {.msk = 0x00000002U, .pos = 1};
const field32_t MDIOS_MDIOS_CR_RDIE         = {.msk = 0x00000004U, .pos = 2};
const field32_t MDIOS_MDIOS_CR_EIE          = {.msk = 0x00000008U, .pos = 3};
const field32_t MDIOS_MDIOS_CR_DPC          = {.msk = 0x00000080U, .pos = 7};
const field32_t MDIOS_MDIOS_CR_PORT_ADDRESS = {.msk = 0x00001F00U, .pos = 8};
const field32_t MDIOS_MDIOS_SR_PERF         = {.msk = 0x00000001U, .pos = 0};
const field32_t MDIOS_MDIOS_SR_SERF         = {.msk = 0x00000002U, .pos = 1};
const field32_t MDIOS_MDIOS_SR_TERF         = {.msk = 0x00000004U, .pos = 2};
const field32_t MDIOS_MDIOS_CLRFR_CPERF     = {.msk = 0x00000001U, .pos = 0};
const field32_t MDIOS_MDIOS_CLRFR_CSERF     = {.msk = 0x00000002U, .pos = 1};
const field32_t MDIOS_MDIOS_CLRFR_CTERF     = {.msk = 0x00000004U, .pos = 2};
const field32_t MDIOS_MDIOS_DINR0_DIN0      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR1_DIN1      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR2_DIN2      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR3_DIN3      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR4_DIN4      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR5_DIN5      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR6_DIN6      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR7_DIN7      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR8_DIN8      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR9_DIN9      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR10_DIN10    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR11_DIN11    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR12_DIN12    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR13_DIN13    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR14_DIN14    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR15_DIN15    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR16_DIN16    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR17_DIN17    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR18_DIN18    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR19_DIN19    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR20_DIN20    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR21_DIN21    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR22_DIN22    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR23_DIN23    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR24_DIN24    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR25_DIN25    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR26_DIN26    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR27_DIN27    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR28_DIN28    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR29_DIN29    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR30_DIN30    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DINR31_DIN31    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR0_DOUT0    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR1_DOUT1    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR2_DOUT2    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR3_DOUT3    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR4_DOUT4    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR5_DOUT5    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR6_DOUT6    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR7_DOUT7    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR8_DOUT8    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR9_DOUT9    = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR10_DOUT10  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR11_DOUT11  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR12_DOUT12  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR13_DOUT13  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR14_DOUT14  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR15_DOUT15  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR16_DOUT16  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR17_DOUT17  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR18_DOUT18  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR19_DOUT19  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR20_DOUT20  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR21_DOUT21  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR22_DOUT22  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR23_DOUT23  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR24_DOUT24  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR25_DOUT25  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR26_DOUT26  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR27_DOUT27  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR28_DOUT28  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR29_DOUT29  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR30_DOUT30  = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t MDIOS_MDIOS_DOUTR31_DOUT31  = {.msk = 0x0000FFFFU, .pos = 0};

/**************************************************************************************************
 * @section OPAMP Definitions
 **************************************************************************************************/

/** @subsection OPAMP Register Definitions */

rw_reg32_t const OPAMP_OPAMP1_CSR = (rw_reg32_t)0x40009000U;
rw_reg32_t const OPAMP_OPAMP2_CSR = (rw_reg32_t)0x40009010U;

/** @subsection Enumerated OPAMP Register Definitions */

rw_reg32_t const OPAMP_OPAMPx_OTR[3] = {
  [1] = (rw_reg32_t)0x40009004U,
  [2] = (rw_reg32_t)0x40009014U,
};

rw_reg32_t const OPAMP_OPAMPx_HSOTR[3] = {
  [1] = (rw_reg32_t)0x40009008U,
  [2] = (rw_reg32_t)0x40009018U,
};

/** @subsection OPAMP Register Field Definitions */

const field32_t OPAMP_OPAMP1_CSR_OPAEN           = {.msk = 0x00000001U, .pos = 0};
const field32_t OPAMP_OPAMP1_CSR_FORCE_VP        = {.msk = 0x00000002U, .pos = 1};
const field32_t OPAMP_OPAMP1_CSR_VP_SEL          = {.msk = 0x0000000CU, .pos = 2};
const field32_t OPAMP_OPAMP1_CSR_VM_SEL          = {.msk = 0x00000060U, .pos = 5};
const field32_t OPAMP_OPAMP1_CSR_OPAHSM          = {.msk = 0x00000100U, .pos = 8};
const field32_t OPAMP_OPAMP1_CSR_CALON           = {.msk = 0x00000800U, .pos = 11};
const field32_t OPAMP_OPAMP1_CSR_CALSEL          = {.msk = 0x00003000U, .pos = 12};
const field32_t OPAMP_OPAMP1_CSR_PGA_GAIN        = {.msk = 0x0003C000U, .pos = 14};
const field32_t OPAMP_OPAMP1_CSR_USERTRIM        = {.msk = 0x00040000U, .pos = 18};
const field32_t OPAMP_OPAMP1_CSR_TSTREF          = {.msk = 0x20000000U, .pos = 29};
const field32_t OPAMP_OPAMP1_CSR_CALOUT          = {.msk = 0x40000000U, .pos = 30};
const field32_t OPAMP_OPAMPx_OTR_TRIMOFFSETN     = {.msk = 0x0000001FU, .pos = 0};
const field32_t OPAMP_OPAMPx_OTR_TRIMOFFSETP     = {.msk = 0x00001F00U, .pos = 8};
const field32_t OPAMP_OPAMPx_HSOTR_TRIMLPOFFSETN = {.msk = 0x0000001FU, .pos = 0};
const field32_t OPAMP_OPAMPx_HSOTR_TRIMLPOFFSETP = {.msk = 0x00001F00U, .pos = 8};
const field32_t OPAMP_OPAMP2_CSR_OPAEN           = {.msk = 0x00000001U, .pos = 0};
const field32_t OPAMP_OPAMP2_CSR_FORCE_VP        = {.msk = 0x00000002U, .pos = 1};
const field32_t OPAMP_OPAMP2_CSR_VM_SEL          = {.msk = 0x00000060U, .pos = 5};
const field32_t OPAMP_OPAMP2_CSR_OPAHSM          = {.msk = 0x00000100U, .pos = 8};
const field32_t OPAMP_OPAMP2_CSR_CALON           = {.msk = 0x00000800U, .pos = 11};
const field32_t OPAMP_OPAMP2_CSR_CALSEL          = {.msk = 0x00003000U, .pos = 12};
const field32_t OPAMP_OPAMP2_CSR_PGA_GAIN        = {.msk = 0x0003C000U, .pos = 14};
const field32_t OPAMP_OPAMP2_CSR_USERTRIM        = {.msk = 0x00040000U, .pos = 18};
const field32_t OPAMP_OPAMP2_CSR_TSTREF          = {.msk = 0x20000000U, .pos = 29};
const field32_t OPAMP_OPAMP2_CSR_CALOUT          = {.msk = 0x40000000U, .pos = 30};

/**************************************************************************************************
 * @section SWPMI Definitions
 **************************************************************************************************/

/** @subsection SWPMI Register Definitions */

rw_reg32_t const SWPMI_CR  = (rw_reg32_t)0x40008800U;
rw_reg32_t const SWPMI_BRR = (rw_reg32_t)0x40008804U;
ro_reg32_t const SWPMI_ISR = (ro_reg32_t)0x4000880CU;
rw_reg32_t const SWPMI_ICR = (rw_reg32_t)0x40008810U;
rw_reg32_t const SWPMI_IER = (rw_reg32_t)0x40008814U;
ro_reg32_t const SWPMI_RFL = (ro_reg32_t)0x40008818U;
rw_reg32_t const SWPMI_TDR = (rw_reg32_t)0x4000881CU;
ro_reg32_t const SWPMI_RDR = (ro_reg32_t)0x40008820U;
rw_reg32_t const SWPMI_OR  = (rw_reg32_t)0x40008824U;

/** @subsection SWPMI Register Field Definitions */

const field32_t SWPMI_CR_RXDMA     = {.msk = 0x00000001U, .pos = 0};
const field32_t SWPMI_CR_TXDMA     = {.msk = 0x00000002U, .pos = 1};
const field32_t SWPMI_CR_RXMODE    = {.msk = 0x00000004U, .pos = 2};
const field32_t SWPMI_CR_TXMODE    = {.msk = 0x00000008U, .pos = 3};
const field32_t SWPMI_CR_LPBK      = {.msk = 0x00000010U, .pos = 4};
const field32_t SWPMI_CR_SWPACT    = {.msk = 0x00000020U, .pos = 5};
const field32_t SWPMI_CR_DEACT     = {.msk = 0x00000400U, .pos = 10};
const field32_t SWPMI_CR_SWPTEN    = {.msk = 0x00000800U, .pos = 11};
const field32_t SWPMI_BRR_BR       = {.msk = 0x000000FFU, .pos = 0};
const field32_t SWPMI_ISR_RXBFF    = {.msk = 0x00000001U, .pos = 0};
const field32_t SWPMI_ISR_TXBEF    = {.msk = 0x00000002U, .pos = 1};
const field32_t SWPMI_ISR_RXBERF   = {.msk = 0x00000004U, .pos = 2};
const field32_t SWPMI_ISR_RXOVRF   = {.msk = 0x00000008U, .pos = 3};
const field32_t SWPMI_ISR_TXUNRF   = {.msk = 0x00000010U, .pos = 4};
const field32_t SWPMI_ISR_RXNE     = {.msk = 0x00000020U, .pos = 5};
const field32_t SWPMI_ISR_TXE      = {.msk = 0x00000040U, .pos = 6};
const field32_t SWPMI_ISR_TCF      = {.msk = 0x00000080U, .pos = 7};
const field32_t SWPMI_ISR_SRF      = {.msk = 0x00000100U, .pos = 8};
const field32_t SWPMI_ISR_SUSP     = {.msk = 0x00000200U, .pos = 9};
const field32_t SWPMI_ISR_DEACTF   = {.msk = 0x00000400U, .pos = 10};
const field32_t SWPMI_ISR_RDYF     = {.msk = 0x00000800U, .pos = 11};
const field32_t SWPMI_ICR_CRXBFF   = {.msk = 0x00000001U, .pos = 0};
const field32_t SWPMI_ICR_CTXBEF   = {.msk = 0x00000002U, .pos = 1};
const field32_t SWPMI_ICR_CRXBERF  = {.msk = 0x00000004U, .pos = 2};
const field32_t SWPMI_ICR_CRXOVRF  = {.msk = 0x00000008U, .pos = 3};
const field32_t SWPMI_ICR_CTXUNRF  = {.msk = 0x00000010U, .pos = 4};
const field32_t SWPMI_ICR_CTCF     = {.msk = 0x00000080U, .pos = 7};
const field32_t SWPMI_ICR_CSRF     = {.msk = 0x00000100U, .pos = 8};
const field32_t SWPMI_ICR_CRDYF    = {.msk = 0x00000800U, .pos = 11};
const field32_t SWPMI_IER_RXBFIE   = {.msk = 0x00000001U, .pos = 0};
const field32_t SWPMI_IER_TXBEIE   = {.msk = 0x00000002U, .pos = 1};
const field32_t SWPMI_IER_RXBERIE  = {.msk = 0x00000004U, .pos = 2};
const field32_t SWPMI_IER_RXOVRIE  = {.msk = 0x00000008U, .pos = 3};
const field32_t SWPMI_IER_TXUNRIE  = {.msk = 0x00000010U, .pos = 4};
const field32_t SWPMI_IER_RIE      = {.msk = 0x00000020U, .pos = 5};
const field32_t SWPMI_IER_TIE      = {.msk = 0x00000040U, .pos = 6};
const field32_t SWPMI_IER_TCIE     = {.msk = 0x00000080U, .pos = 7};
const field32_t SWPMI_IER_SRIE     = {.msk = 0x00000100U, .pos = 8};
const field32_t SWPMI_IER_RDYIE    = {.msk = 0x00000800U, .pos = 11};
const field32_t SWPMI_RFL_RFL      = {.msk = 0x0000001FU, .pos = 0};
const field32_t SWPMI_OR_SWP_TBYP  = {.msk = 0x00000001U, .pos = 0};
const field32_t SWPMI_OR_SWP_CLASS = {.msk = 0x00000002U, .pos = 1};

/**************************************************************************************************
 * @section G_TIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated G_TIMx Register Definitions */

rw_reg32_t const G_TIMx_CR1[15] = {
  [2]  = (rw_reg32_t)0x40000000U,
  [3]  = (rw_reg32_t)0x40000400U,
  [4]  = (rw_reg32_t)0x40000800U,
  [5]  = (rw_reg32_t)0x40000C00U,
  [12] = (rw_reg32_t)0x40001800U,
  [13] = (rw_reg32_t)0x40001C00U,
  [14] = (rw_reg32_t)0x40002000U,
};

rw_reg32_t const G_TIMx_CR2[15] = {
  [2]  = (rw_reg32_t)0x40000004U,
  [3]  = (rw_reg32_t)0x40000404U,
  [4]  = (rw_reg32_t)0x40000804U,
  [5]  = (rw_reg32_t)0x40000C04U,
  [12] = (rw_reg32_t)0x40001804U,
  [13] = (rw_reg32_t)0x40001C04U,
  [14] = (rw_reg32_t)0x40002004U,
};

rw_reg32_t const G_TIMx_SMCR[15] = {
  [2]  = (rw_reg32_t)0x40000008U,
  [3]  = (rw_reg32_t)0x40000408U,
  [4]  = (rw_reg32_t)0x40000808U,
  [5]  = (rw_reg32_t)0x40000C08U,
  [12] = (rw_reg32_t)0x40001808U,
  [13] = (rw_reg32_t)0x40001C08U,
  [14] = (rw_reg32_t)0x40002008U,
};

rw_reg32_t const G_TIMx_DIER[15] = {
  [2]  = (rw_reg32_t)0x4000000CU,
  [3]  = (rw_reg32_t)0x4000040CU,
  [4]  = (rw_reg32_t)0x4000080CU,
  [5]  = (rw_reg32_t)0x40000C0CU,
  [12] = (rw_reg32_t)0x4000180CU,
  [13] = (rw_reg32_t)0x40001C0CU,
  [14] = (rw_reg32_t)0x4000200CU,
};

rw_reg32_t const G_TIMx_SR[15] = {
  [2]  = (rw_reg32_t)0x40000010U,
  [3]  = (rw_reg32_t)0x40000410U,
  [4]  = (rw_reg32_t)0x40000810U,
  [5]  = (rw_reg32_t)0x40000C10U,
  [12] = (rw_reg32_t)0x40001810U,
  [13] = (rw_reg32_t)0x40001C10U,
  [14] = (rw_reg32_t)0x40002010U,
};

rw_reg32_t const G_TIMx_EGR[15] = {
  [2]  = (rw_reg32_t)0x40000014U,
  [3]  = (rw_reg32_t)0x40000414U,
  [4]  = (rw_reg32_t)0x40000814U,
  [5]  = (rw_reg32_t)0x40000C14U,
  [12] = (rw_reg32_t)0x40001814U,
  [13] = (rw_reg32_t)0x40001C14U,
  [14] = (rw_reg32_t)0x40002014U,
};

rw_reg32_t const G_TIMx_CCMR1_OUTPUT[15] = {
  [2]  = (rw_reg32_t)0x40000018U,
  [3]  = (rw_reg32_t)0x40000418U,
  [4]  = (rw_reg32_t)0x40000818U,
  [5]  = (rw_reg32_t)0x40000C18U,
  [12] = (rw_reg32_t)0x40001818U,
  [13] = (rw_reg32_t)0x40001C18U,
  [14] = (rw_reg32_t)0x40002018U,
};

rw_reg32_t const G_TIMx_CCMR1_INPUT[15] = {
  [2]  = (rw_reg32_t)0x40000018U,
  [3]  = (rw_reg32_t)0x40000418U,
  [4]  = (rw_reg32_t)0x40000818U,
  [5]  = (rw_reg32_t)0x40000C18U,
  [12] = (rw_reg32_t)0x40001818U,
  [13] = (rw_reg32_t)0x40001C18U,
  [14] = (rw_reg32_t)0x40002018U,
};

rw_reg32_t const G_TIMx_CCMR2_OUTPUT[15] = {
  [2]  = (rw_reg32_t)0x4000001CU,
  [3]  = (rw_reg32_t)0x4000041CU,
  [4]  = (rw_reg32_t)0x4000081CU,
  [5]  = (rw_reg32_t)0x40000C1CU,
  [12] = (rw_reg32_t)0x4000181CU,
  [13] = (rw_reg32_t)0x40001C1CU,
  [14] = (rw_reg32_t)0x4000201CU,
};

rw_reg32_t const G_TIMx_CCMR2_INPUT[15] = {
  [2]  = (rw_reg32_t)0x4000001CU,
  [3]  = (rw_reg32_t)0x4000041CU,
  [4]  = (rw_reg32_t)0x4000081CU,
  [5]  = (rw_reg32_t)0x40000C1CU,
  [12] = (rw_reg32_t)0x4000181CU,
  [13] = (rw_reg32_t)0x40001C1CU,
  [14] = (rw_reg32_t)0x4000201CU,
};

rw_reg32_t const G_TIMx_CCER[15] = {
  [2]  = (rw_reg32_t)0x40000020U,
  [3]  = (rw_reg32_t)0x40000420U,
  [4]  = (rw_reg32_t)0x40000820U,
  [5]  = (rw_reg32_t)0x40000C20U,
  [12] = (rw_reg32_t)0x40001820U,
  [13] = (rw_reg32_t)0x40001C20U,
  [14] = (rw_reg32_t)0x40002020U,
};

rw_reg32_t const G_TIMx_CNT[15] = {
  [2]  = (rw_reg32_t)0x40000024U,
  [3]  = (rw_reg32_t)0x40000424U,
  [4]  = (rw_reg32_t)0x40000824U,
  [5]  = (rw_reg32_t)0x40000C24U,
  [12] = (rw_reg32_t)0x40001824U,
  [13] = (rw_reg32_t)0x40001C24U,
  [14] = (rw_reg32_t)0x40002024U,
};

rw_reg32_t const G_TIMx_PSC[15] = {
  [2]  = (rw_reg32_t)0x40000028U,
  [3]  = (rw_reg32_t)0x40000428U,
  [4]  = (rw_reg32_t)0x40000828U,
  [5]  = (rw_reg32_t)0x40000C28U,
  [12] = (rw_reg32_t)0x40001828U,
  [13] = (rw_reg32_t)0x40001C28U,
  [14] = (rw_reg32_t)0x40002028U,
};

rw_reg32_t const G_TIMx_ARR[15] = {
  [2]  = (rw_reg32_t)0x4000002CU,
  [3]  = (rw_reg32_t)0x4000042CU,
  [4]  = (rw_reg32_t)0x4000082CU,
  [5]  = (rw_reg32_t)0x40000C2CU,
  [12] = (rw_reg32_t)0x4000182CU,
  [13] = (rw_reg32_t)0x40001C2CU,
  [14] = (rw_reg32_t)0x4000202CU,
};

rw_reg32_t const G_TIMx_CCR1[15] = {
  [2]  = (rw_reg32_t)0x40000034U,
  [3]  = (rw_reg32_t)0x40000434U,
  [4]  = (rw_reg32_t)0x40000834U,
  [5]  = (rw_reg32_t)0x40000C34U,
  [12] = (rw_reg32_t)0x40001834U,
  [13] = (rw_reg32_t)0x40001C34U,
  [14] = (rw_reg32_t)0x40002034U,
};

rw_reg32_t const G_TIMx_CCR2[15] = {
  [2]  = (rw_reg32_t)0x40000038U,
  [3]  = (rw_reg32_t)0x40000438U,
  [4]  = (rw_reg32_t)0x40000838U,
  [5]  = (rw_reg32_t)0x40000C38U,
  [12] = (rw_reg32_t)0x40001838U,
  [13] = (rw_reg32_t)0x40001C38U,
  [14] = (rw_reg32_t)0x40002038U,
};

rw_reg32_t const G_TIMx_CCR3[15] = {
  [2]  = (rw_reg32_t)0x4000003CU,
  [3]  = (rw_reg32_t)0x4000043CU,
  [4]  = (rw_reg32_t)0x4000083CU,
  [5]  = (rw_reg32_t)0x40000C3CU,
  [12] = (rw_reg32_t)0x4000183CU,
  [13] = (rw_reg32_t)0x40001C3CU,
  [14] = (rw_reg32_t)0x4000203CU,
};

rw_reg32_t const G_TIMx_CCR4[15] = {
  [2]  = (rw_reg32_t)0x40000040U,
  [3]  = (rw_reg32_t)0x40000440U,
  [4]  = (rw_reg32_t)0x40000840U,
  [5]  = (rw_reg32_t)0x40000C40U,
  [12] = (rw_reg32_t)0x40001840U,
  [13] = (rw_reg32_t)0x40001C40U,
  [14] = (rw_reg32_t)0x40002040U,
};

rw_reg32_t const G_TIMx_DCR[15] = {
  [2]  = (rw_reg32_t)0x40000048U,
  [3]  = (rw_reg32_t)0x40000448U,
  [4]  = (rw_reg32_t)0x40000848U,
  [5]  = (rw_reg32_t)0x40000C48U,
  [12] = (rw_reg32_t)0x40001848U,
  [13] = (rw_reg32_t)0x40001C48U,
  [14] = (rw_reg32_t)0x40002048U,
};

rw_reg32_t const G_TIMx_DMAR[15] = {
  [2]  = (rw_reg32_t)0x4000004CU,
  [3]  = (rw_reg32_t)0x4000044CU,
  [4]  = (rw_reg32_t)0x4000084CU,
  [5]  = (rw_reg32_t)0x40000C4CU,
  [12] = (rw_reg32_t)0x4000184CU,
  [13] = (rw_reg32_t)0x40001C4CU,
  [14] = (rw_reg32_t)0x4000204CU,
};

rw_reg32_t const G_TIMx_AF1[15] = {
  [2]  = (rw_reg32_t)0x40000060U,
  [3]  = (rw_reg32_t)0x40000460U,
  [4]  = (rw_reg32_t)0x40000860U,
  [5]  = (rw_reg32_t)0x40000C60U,
  [12] = (rw_reg32_t)0x40001860U,
  [13] = (rw_reg32_t)0x40001C60U,
  [14] = (rw_reg32_t)0x40002060U,
};

rw_reg32_t const G_TIMx_TISEL[15] = {
  [2]  = (rw_reg32_t)0x40000068U,
  [3]  = (rw_reg32_t)0x40000468U,
  [4]  = (rw_reg32_t)0x40000868U,
  [5]  = (rw_reg32_t)0x40000C68U,
  [12] = (rw_reg32_t)0x40001868U,
  [13] = (rw_reg32_t)0x40001C68U,
  [14] = (rw_reg32_t)0x40002068U,
};

/** @subsection G_TIMx Register Field Definitions */

const field32_t G_TIMx_CR1_UIFREMAP       = {.msk = 0x00000800U, .pos = 11};
const field32_t G_TIMx_CR1_CKD            = {.msk = 0x00000300U, .pos = 8};
const field32_t G_TIMx_CR1_ARPE           = {.msk = 0x00000080U, .pos = 7};
const field32_t G_TIMx_CR1_CMS            = {.msk = 0x00000060U, .pos = 5};
const field32_t G_TIMx_CR1_DIR            = {.msk = 0x00000010U, .pos = 4};
const field32_t G_TIMx_CR1_OPM            = {.msk = 0x00000008U, .pos = 3};
const field32_t G_TIMx_CR1_URS            = {.msk = 0x00000004U, .pos = 2};
const field32_t G_TIMx_CR1_UDIS           = {.msk = 0x00000002U, .pos = 1};
const field32_t G_TIMx_CR1_CEN            = {.msk = 0x00000001U, .pos = 0};
const field32_t G_TIMx_CR2_TI1S           = {.msk = 0x00000080U, .pos = 7};
const field32_t G_TIMx_CR2_MMS            = {.msk = 0x00000070U, .pos = 4};
const field32_t G_TIMx_CR2_CCDS           = {.msk = 0x00000008U, .pos = 3};
const field32_t G_TIMx_SMCR_TS_4_3        = {.msk = 0x00300000U, .pos = 20};
const field32_t G_TIMx_SMCR_SMS_3         = {.msk = 0x00010000U, .pos = 16};
const field32_t G_TIMx_SMCR_ETP           = {.msk = 0x00008000U, .pos = 15};
const field32_t G_TIMx_SMCR_ECE           = {.msk = 0x00004000U, .pos = 14};
const field32_t G_TIMx_SMCR_ETPS          = {.msk = 0x00003000U, .pos = 12};
const field32_t G_TIMx_SMCR_ETF           = {.msk = 0x00000F00U, .pos = 8};
const field32_t G_TIMx_SMCR_MSM           = {.msk = 0x00000080U, .pos = 7};
const field32_t G_TIMx_SMCR_TS            = {.msk = 0x00000070U, .pos = 4};
const field32_t G_TIMx_SMCR_SMS           = {.msk = 0x00000007U, .pos = 0};
const field32_t G_TIMx_DIER_TDE           = {.msk = 0x00004000U, .pos = 14};
const field32_t G_TIMx_DIER_UDE           = {.msk = 0x00000100U, .pos = 8};
const field32_t G_TIMx_DIER_TIE           = {.msk = 0x00000040U, .pos = 6};
const field32_t G_TIMx_DIER_UIE           = {.msk = 0x00000001U, .pos = 0};
const field32_t G_TIMx_SR_TIF             = {.msk = 0x00000040U, .pos = 6};
const field32_t G_TIMx_SR_UIF             = {.msk = 0x00000001U, .pos = 0};
const field32_t G_TIMx_EGR_TG             = {.msk = 0x00000040U, .pos = 6};
const field32_t G_TIMx_EGR_UG             = {.msk = 0x00000001U, .pos = 0};
const field32_t G_TIMx_CCMR1_INPUT_IC2PCS = {.msk = 0x00000C00U, .pos = 10};
const field32_t G_TIMx_CCMR1_INPUT_ICPCS  = {.msk = 0x0000000CU, .pos = 2};
const field32_t G_TIMx_CCMR2_OUTPUT_O24CE = {.msk = 0x00008000U, .pos = 15};
const field32_t G_TIMx_CCMR2_OUTPUT_OC3CE = {.msk = 0x00000080U, .pos = 7};
const field32_t G_TIMx_CNT_CNT_L          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_CNT_CNT_H          = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t G_TIMx_PSC_PSC            = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_ARR_ARR_H          = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t G_TIMx_ARR_ARR_L          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_CCR1_CCR1_H        = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t G_TIMx_CCR1_CCR1_L        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_CCR2_CCR2_H        = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t G_TIMx_CCR2_CCR2_L        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_CCR3_CCR3_H        = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t G_TIMx_CCR3_CCR3_L        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_CCR4_CCR4_H        = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t G_TIMx_CCR4_CCR4_L        = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_DCR_DBL            = {.msk = 0x00001F00U, .pos = 8};
const field32_t G_TIMx_DCR_DBA            = {.msk = 0x0000001FU, .pos = 0};
const field32_t G_TIMx_DMAR_DMAB          = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t G_TIMx_AF1_ETRSEL         = {.msk = 0x0003C000U, .pos = 14};

/** @subsection Enumerated G_TIMx Register Field Definitions */

field32_t const G_TIMx_DIER_CCxDE[5] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const G_TIMx_DIER_CCxIE[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const G_TIMx_SR_CCxOF[5] = {
  [1] = {.msk = 0x00000200U, .pos = 9},
  [2] = {.msk = 0x00000400U, .pos = 10},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const G_TIMx_SR_CCxIF[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const G_TIMx_EGR_CCxG[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000004U, .pos = 2},
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000010U, .pos = 4},
};

field32_t const G_TIMx_CCMR1_OUTPUT_CCxS[3] = {
  [1] = {.msk = 0x00000003U, .pos = 0},
  [2] = {.msk = 0x00000300U, .pos = 8},
};

field32_t const G_TIMx_CCMR1_OUTPUT_OCxFE[3] = {
  [1] = {.msk = 0x00000004U, .pos = 2},
  [2] = {.msk = 0x00000400U, .pos = 10},
};

field32_t const G_TIMx_CCMR1_OUTPUT_OCxPE[3] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const G_TIMx_CCMR2_OUTPUT_OCxPE[5] = {
  [3] = {.msk = 0x00000008U, .pos = 3},
  [4] = {.msk = 0x00000800U, .pos = 11},
};

field32_t const G_TIMx_CCMR1_OUTPUT_OCxM[3] = {
  [1] = {.msk = 0x00000070U, .pos = 4},
  [2] = {.msk = 0x00007000U, .pos = 12},
};

field32_t const G_TIMx_CCMR2_OUTPUT_OCxM[5] = {
  [3] = {.msk = 0x00000070U, .pos = 4},
  [4] = {.msk = 0x00007000U, .pos = 12},
};

field32_t const G_TIMx_CCMR1_OUTPUT_OCxCE[3] = {
  [1] = {.msk = 0x00000080U, .pos = 7},
  [2] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const G_TIMx_CCMR1_OUTPUT_OCxM_3[3] = {
  [1] = {.msk = 0x00010000U, .pos = 16},
  [2] = {.msk = 0x01000000U, .pos = 24},
};

field32_t const G_TIMx_CCMR1_INPUT_ICxF[3] = {
  [1] = {.msk = 0x000000F0U, .pos = 4},
  [2] = {.msk = 0x0000F000U, .pos = 12},
};

field32_t const G_TIMx_CCMR2_INPUT_ICxPSC[5] = {
  [3] = {.msk = 0x0000000CU, .pos = 2},
  [4] = {.msk = 0x00000C00U, .pos = 10},
};

field32_t const G_TIMx_CCER_CCxNP[5] = {
  [1] = {.msk = 0x00000008U, .pos = 3},
  [2] = {.msk = 0x00000080U, .pos = 7},
  [3] = {.msk = 0x00000800U, .pos = 11},
  [4] = {.msk = 0x00008000U, .pos = 15},
};

field32_t const G_TIMx_CCER_CCxP[5] = {
  [1] = {.msk = 0x00000002U, .pos = 1},
  [2] = {.msk = 0x00000020U, .pos = 5},
  [3] = {.msk = 0x00000200U, .pos = 9},
  [4] = {.msk = 0x00002000U, .pos = 13},
};

field32_t const G_TIMx_CCER_CCxE[5] = {
  [1] = {.msk = 0x00000001U, .pos = 0},
  [2] = {.msk = 0x00000010U, .pos = 4},
  [3] = {.msk = 0x00000100U, .pos = 8},
  [4] = {.msk = 0x00001000U, .pos = 12},
};

field32_t const G_TIMx_TISEL_TIxSEL[5] = {
  [1] = {.msk = 0x0000000FU, .pos = 0},
  [2] = {.msk = 0x00000F00U, .pos = 8},
  [3] = {.msk = 0x000F0000U, .pos = 16},
  [4] = {.msk = 0x0F000000U, .pos = 24},
};

/**************************************************************************************************
 * @section B_TIMx Definitions
 **************************************************************************************************/

/** @subsection Enumerated B_TIMx Register Definitions */

rw_reg32_t const B_TIMx_CR1[8] = {
  [6] = (rw_reg32_t)0x40001000U,
  [7] = (rw_reg32_t)0x40001400U,
};

rw_reg32_t const B_TIMx_CR2[8] = {
  [6] = (rw_reg32_t)0x40001004U,
  [7] = (rw_reg32_t)0x40001404U,
};

rw_reg32_t const B_TIMx_DIER[8] = {
  [6] = (rw_reg32_t)0x4000100CU,
  [7] = (rw_reg32_t)0x4000140CU,
};

rw_reg32_t const B_TIMx_SR[8] = {
  [6] = (rw_reg32_t)0x40001010U,
  [7] = (rw_reg32_t)0x40001410U,
};

rw_reg32_t const B_TIMx_EGR[8] = {
  [6] = (rw_reg32_t)0x40001014U,
  [7] = (rw_reg32_t)0x40001414U,
};

rw_reg32_t const B_TIMx_CNT[8] = {
  [6] = (rw_reg32_t)0x40001024U,
  [7] = (rw_reg32_t)0x40001424U,
};

rw_reg32_t const B_TIMx_PSC[8] = {
  [6] = (rw_reg32_t)0x40001028U,
  [7] = (rw_reg32_t)0x40001428U,
};

rw_reg32_t const B_TIMx_ARR[8] = {
  [6] = (rw_reg32_t)0x4000102CU,
  [7] = (rw_reg32_t)0x4000142CU,
};

/** @subsection B_TIMx Register Field Definitions */

const field32_t B_TIMx_CR1_UIFREMAP = {.msk = 0x00000800U, .pos = 11};
const field32_t B_TIMx_CR1_ARPE     = {.msk = 0x00000080U, .pos = 7};
const field32_t B_TIMx_CR1_OPM      = {.msk = 0x00000008U, .pos = 3};
const field32_t B_TIMx_CR1_URS      = {.msk = 0x00000004U, .pos = 2};
const field32_t B_TIMx_CR1_UDIS     = {.msk = 0x00000002U, .pos = 1};
const field32_t B_TIMx_CR1_CEN      = {.msk = 0x00000001U, .pos = 0};
const field32_t B_TIMx_CR2_MMS      = {.msk = 0x00000070U, .pos = 4};
const field32_t B_TIMx_DIER_UDE     = {.msk = 0x00000100U, .pos = 8};
const field32_t B_TIMx_DIER_UIE     = {.msk = 0x00000001U, .pos = 0};
const field32_t B_TIMx_SR_UIF       = {.msk = 0x00000001U, .pos = 0};
const field32_t B_TIMx_EGR_UG       = {.msk = 0x00000001U, .pos = 0};
const field32_t B_TIMx_CNT_CNT      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t B_TIMx_CNT_UIFCPY   = {.msk = 0x80000000U, .pos = 31};
const field32_t B_TIMx_PSC_PSC      = {.msk = 0x0000FFFFU, .pos = 0};
const field32_t B_TIMx_ARR_ARR      = {.msk = 0x0000FFFFU, .pos = 0};

/**************************************************************************************************
 * @section NVIC Definitions
 **************************************************************************************************/

/** @subsection NVIC Register Definitions */

rw_reg32_t const NVIC_STIR = (rw_reg32_t)0xE000EF00U;

/** @subsection Enumerated NVIC Register Definitions */

rw_reg32_t const NVIC_ISERx[4] = {
  [0] = (rw_reg32_t)0xE000E100U,
  [1] = (rw_reg32_t)0xE000E104U,
  [2] = (rw_reg32_t)0xE000E108U,
  [3] = (rw_reg32_t)0xE000E10CU,
};

rw_reg32_t const NVIC_ICERx[4] = {
  [0] = (rw_reg32_t)0xE000E180U,
  [1] = (rw_reg32_t)0xE000E184U,
  [2] = (rw_reg32_t)0xE000E188U,
  [3] = (rw_reg32_t)0xE000E18CU,
};

rw_reg32_t const NVIC_ISPRx[4] = {
  [0] = (rw_reg32_t)0xE000E200U,
  [1] = (rw_reg32_t)0xE000E204U,
  [2] = (rw_reg32_t)0xE000E208U,
  [3] = (rw_reg32_t)0xE000E20CU,
};

rw_reg32_t const NVIC_ICPRx[4] = {
  [0] = (rw_reg32_t)0xE000E280U,
  [1] = (rw_reg32_t)0xE000E284U,
  [2] = (rw_reg32_t)0xE000E288U,
  [3] = (rw_reg32_t)0xE000E2C0U,
};

ro_reg32_t const NVIC_IABRx[4] = {
  [0] = (ro_reg32_t)0xE000E300U,
  [1] = (ro_reg32_t)0xE000E304U,
  [2] = (ro_reg32_t)0xE000E308U,
  [3] = (ro_reg32_t)0xE000E30CU,
};

rw_reg32_t const NVIC_IPRx[39] = {
  [0]  = (rw_reg32_t)0xE000E400U,
  [1]  = (rw_reg32_t)0xE000E404U,
  [2]  = (rw_reg32_t)0xE000E408U,
  [3]  = (rw_reg32_t)0xE000E40CU,
  [4]  = (rw_reg32_t)0xE000E410U,
  [5]  = (rw_reg32_t)0xE000E414U,
  [6]  = (rw_reg32_t)0xE000E418U,
  [7]  = (rw_reg32_t)0xE000E41CU,
  [8]  = (rw_reg32_t)0xE000E420U,
  [9]  = (rw_reg32_t)0xE000E424U,
  [10] = (rw_reg32_t)0xE000E428U,
  [11] = (rw_reg32_t)0xE000E42CU,
  [12] = (rw_reg32_t)0xE000E430U,
  [13] = (rw_reg32_t)0xE000E434U,
  [14] = (rw_reg32_t)0xE000E438U,
  [15] = (rw_reg32_t)0xE000E43CU,
  [16] = (rw_reg32_t)0xE000E440U,
  [17] = (rw_reg32_t)0xE000E444U,
  [18] = (rw_reg32_t)0xE000E448U,
  [19] = (rw_reg32_t)0xE000E44CU,
  [20] = (rw_reg32_t)0xE000E450U,
  [21] = (rw_reg32_t)0xE000E454U,
  [22] = (rw_reg32_t)0xE000E458U,
  [23] = (rw_reg32_t)0xE000E45CU,
  [24] = (rw_reg32_t)0xE000E460U,
  [25] = (rw_reg32_t)0xE000E464U,
  [26] = (rw_reg32_t)0xE000E468U,
  [27] = (rw_reg32_t)0xE000E46CU,
  [28] = (rw_reg32_t)0xE000E470U,
  [29] = (rw_reg32_t)0xE000E474U,
  [30] = (rw_reg32_t)0xE000E478U,
  [31] = (rw_reg32_t)0xE000E47CU,
  [32] = (rw_reg32_t)0xE000E480U,
  [33] = (rw_reg32_t)0xE000E484U,
  [34] = (rw_reg32_t)0xE000E488U,
  [35] = (rw_reg32_t)0xE000E48CU,
  [36] = (rw_reg32_t)0xE000E490U,
  [37] = (rw_reg32_t)0xE000E494U,
  [38] = (rw_reg32_t)0xE000E498U,
};

/** @subsection NVIC_STIR Register Field Definitions */

const field32_t NVIC_STIR_INTID = {.msk = 0x000001FFU, .pos = 0};

/** @subsection Enumerated NVIC Register Field Definitions */

field32_t const NVIC_IPRx_IPR_Nx[4] = {
  [0] = {.msk = 0x000000FFU, .pos = 0},
  [1] = {.msk = 0x0000FF00U, .pos = 8},
  [2] = {.msk = 0x00FF0000U, .pos = 16},
  [3] = {.msk = 0xFF000000U, .pos = 24},
};

/**************************************************************************************************
 * @section MPU Definitions
 **************************************************************************************************/

/** @subsection MPU Register Definitions */

ro_reg32_t const MPU_MPU_TYPER = (ro_reg32_t)0xE000ED90U;
ro_reg32_t const MPU_MPU_CTRL  = (ro_reg32_t)0xE000ED94U;
rw_reg32_t const MPU_MPU_RNR   = (rw_reg32_t)0xE000ED98U;
rw_reg32_t const MPU_MPU_RBAR  = (rw_reg32_t)0xE000ED9CU;
rw_reg32_t const MPU_MPU_RASR  = (rw_reg32_t)0xE000EDA0U;

/** @subsection MPU Register Field Definitions */

const field32_t MPU_MPU_TYPER_SEPARATE  = {.msk = 0x00000001U, .pos = 0};
const field32_t MPU_MPU_TYPER_DREGION   = {.msk = 0x0000FF00U, .pos = 8};
const field32_t MPU_MPU_TYPER_IREGION   = {.msk = 0x00FF0000U, .pos = 16};
const field32_t MPU_MPU_CTRL_ENABLE     = {.msk = 0x00000001U, .pos = 0};
const field32_t MPU_MPU_CTRL_HFNMIENA   = {.msk = 0x00000002U, .pos = 1};
const field32_t MPU_MPU_CTRL_PRIVDEFENA = {.msk = 0x00000004U, .pos = 2};
const field32_t MPU_MPU_RNR_REGION      = {.msk = 0x000000FFU, .pos = 0};
const field32_t MPU_MPU_RBAR_REGION     = {.msk = 0x0000000FU, .pos = 0};
const field32_t MPU_MPU_RBAR_VALID      = {.msk = 0x00000010U, .pos = 4};
const field32_t MPU_MPU_RBAR_ADDR       = {.msk = 0xFFFFFFE0U, .pos = 5};
const field32_t MPU_MPU_RASR_ENABLE     = {.msk = 0x00000001U, .pos = 0};
const field32_t MPU_MPU_RASR_SIZE       = {.msk = 0x0000003EU, .pos = 1};
const field32_t MPU_MPU_RASR_SRD        = {.msk = 0x0000FF00U, .pos = 8};
const field32_t MPU_MPU_RASR_B          = {.msk = 0x00010000U, .pos = 16};
const field32_t MPU_MPU_RASR_C          = {.msk = 0x00020000U, .pos = 17};
const field32_t MPU_MPU_RASR_S          = {.msk = 0x00040000U, .pos = 18};
const field32_t MPU_MPU_RASR_TEX        = {.msk = 0x00380000U, .pos = 19};
const field32_t MPU_MPU_RASR_AP         = {.msk = 0x07000000U, .pos = 24};
const field32_t MPU_MPU_RASR_XN         = {.msk = 0x10000000U, .pos = 28};

/**************************************************************************************************
 * @section STK Definitions
 **************************************************************************************************/

/** @subsection STK Register Definitions */

rw_reg32_t const STK_CSR   = (rw_reg32_t)0xE000E010U;
rw_reg32_t const STK_RVR   = (rw_reg32_t)0xE000E014U;
rw_reg32_t const STK_CVR   = (rw_reg32_t)0xE000E018U;
rw_reg32_t const STK_CALIB = (rw_reg32_t)0xE000E01CU;

/** @subsection STK Register Field Definitions */

const field32_t STK_CSR_ENABLE    = {.msk = 0x00000001U, .pos = 0};
const field32_t STK_CSR_TICKINT   = {.msk = 0x00000002U, .pos = 1};
const field32_t STK_CSR_CLKSOURCE = {.msk = 0x00000004U, .pos = 2};
const field32_t STK_CSR_COUNTFLAG = {.msk = 0x00010000U, .pos = 16};
const field32_t STK_RVR_RELOAD    = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t STK_CVR_CURRENT   = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t STK_CALIB_TENMS   = {.msk = 0x00FFFFFFU, .pos = 0};
const field32_t STK_CALIB_SKEW    = {.msk = 0x40000000U, .pos = 30};
const field32_t STK_CALIB_NOREF   = {.msk = 0x80000000U, .pos = 31};

/**************************************************************************************************
 * @section FPU Definitions
 **************************************************************************************************/

/** @subsection FPU Register Definitions */

rw_reg32_t const FPU_FPCCR = (rw_reg32_t)0xE000EF34U;
rw_reg32_t const FPU_FPCAR = (rw_reg32_t)0xE000EF38U;
rw_reg32_t const FPU_FPSCR = (rw_reg32_t)0xE000EF3CU;
rw_reg32_t const FPU_CPACR = (rw_reg32_t)0xE000ED88U;

/** @subsection FPU Register Field Definitions */

const field32_t FPU_FPCCR_LSPACT  = {.msk = 0x00000001U, .pos = 0};
const field32_t FPU_FPCCR_USER    = {.msk = 0x00000002U, .pos = 1};
const field32_t FPU_FPCCR_THREAD  = {.msk = 0x00000008U, .pos = 3};
const field32_t FPU_FPCCR_HFRDY   = {.msk = 0x00000010U, .pos = 4};
const field32_t FPU_FPCCR_MMRDY   = {.msk = 0x00000020U, .pos = 5};
const field32_t FPU_FPCCR_BFRDY   = {.msk = 0x00000040U, .pos = 6};
const field32_t FPU_FPCCR_MONRDY  = {.msk = 0x00000100U, .pos = 8};
const field32_t FPU_FPCCR_LSPEN   = {.msk = 0x40000000U, .pos = 30};
const field32_t FPU_FPCCR_ASPEN   = {.msk = 0x80000000U, .pos = 31};
const field32_t FPU_FPCAR_ADDRESS = {.msk = 0xFFFFFFF8U, .pos = 3};
const field32_t FPU_FPSCR_IOC     = {.msk = 0x00000001U, .pos = 0};
const field32_t FPU_FPSCR_DZC     = {.msk = 0x00000002U, .pos = 1};
const field32_t FPU_FPSCR_OFC     = {.msk = 0x00000004U, .pos = 2};
const field32_t FPU_FPSCR_UFC     = {.msk = 0x00000008U, .pos = 3};
const field32_t FPU_FPSCR_IXC     = {.msk = 0x00000010U, .pos = 4};
const field32_t FPU_FPSCR_IDC     = {.msk = 0x00000080U, .pos = 7};
const field32_t FPU_FPSCR_RMODE   = {.msk = 0x00C00000U, .pos = 22};
const field32_t FPU_FPSCR_FZ      = {.msk = 0x01000000U, .pos = 24};
const field32_t FPU_FPSCR_DN      = {.msk = 0x02000000U, .pos = 25};
const field32_t FPU_FPSCR_AHP     = {.msk = 0x04000000U, .pos = 26};
const field32_t FPU_FPSCR_V       = {.msk = 0x10000000U, .pos = 28};
const field32_t FPU_FPSCR_C       = {.msk = 0x20000000U, .pos = 29};
const field32_t FPU_FPSCR_Z       = {.msk = 0x40000000U, .pos = 30};
const field32_t FPU_FPSCR_N       = {.msk = 0x80000000U, .pos = 31};
const field32_t FPU_CPACR_CP      = {.msk = 0x00F00000U, .pos = 20};

/**************************************************************************************************
 * @section SCB Definitions
 **************************************************************************************************/

/** @subsection SCB Register Definitions */

ro_reg32_t const SCB_CPUID       = (ro_reg32_t)0xE000ED00U;
rw_reg32_t const SCB_ICSR        = (rw_reg32_t)0xE000ED04U;
rw_reg32_t const SCB_VTOR        = (rw_reg32_t)0xE000ED08U;
rw_reg32_t const SCB_AIRCR       = (rw_reg32_t)0xE000ED0CU;
rw_reg32_t const SCB_SCR         = (rw_reg32_t)0xE000ED10U;
rw_reg32_t const SCB_CCR         = (rw_reg32_t)0xE000ED14U;
rw_reg32_t const SCB_SHPR1       = (rw_reg32_t)0xE000ED18U;
rw_reg32_t const SCB_SHPR2       = (rw_reg32_t)0xE000ED1CU;
rw_reg32_t const SCB_SHPR3       = (rw_reg32_t)0xE000ED20U;
rw_reg32_t const SCB_SHCRS       = (rw_reg32_t)0xE000ED24U;
rw_reg32_t const SCB_CFSR        = (rw_reg32_t)0xE000ED28U;
rw_reg32_t const SCB_HFSR        = (rw_reg32_t)0xE000ED2CU;
rw_reg32_t const SCB_MMFAR       = (rw_reg32_t)0xE000ED34U;
rw_reg32_t const SCB_BFAR        = (rw_reg32_t)0xE000ED38U;
rw_reg32_t const SCB_ACTRL_ACTRL = (rw_reg32_t)0xE000E008U;

/** @subsection SCB Register Field Definitions */

const field32_t SCB_CPUID_REVISION             = {.msk = 0x0000000FU, .pos = 0};
const field32_t SCB_CPUID_PARTNO               = {.msk = 0x0000FFF0U, .pos = 4};
const field32_t SCB_CPUID_CONSTANT             = {.msk = 0x000F0000U, .pos = 16};
const field32_t SCB_CPUID_VARIANT              = {.msk = 0x00F00000U, .pos = 20};
const field32_t SCB_CPUID_IMPLEMENTER          = {.msk = 0xFF000000U, .pos = 24};
const field32_t SCB_ICSR_VECTACTIVE            = {.msk = 0x000001FFU, .pos = 0};
const field32_t SCB_ICSR_RETTOBASE             = {.msk = 0x00000800U, .pos = 11};
const field32_t SCB_ICSR_VECTPENDING           = {.msk = 0x0007F000U, .pos = 12};
const field32_t SCB_ICSR_ISRPENDING            = {.msk = 0x00400000U, .pos = 22};
const field32_t SCB_ICSR_PENDSTCLR             = {.msk = 0x02000000U, .pos = 25};
const field32_t SCB_ICSR_PENDSTSET             = {.msk = 0x04000000U, .pos = 26};
const field32_t SCB_ICSR_PENDSVCLR             = {.msk = 0x08000000U, .pos = 27};
const field32_t SCB_ICSR_PENDSVSET             = {.msk = 0x10000000U, .pos = 28};
const field32_t SCB_ICSR_NMIPENDSET            = {.msk = 0x80000000U, .pos = 31};
const field32_t SCB_VTOR_TBLOFF                = {.msk = 0x3FFFFE00U, .pos = 9};
const field32_t SCB_AIRCR_VECTRESET            = {.msk = 0x00000001U, .pos = 0};
const field32_t SCB_AIRCR_VECTCLRACTIVE        = {.msk = 0x00000002U, .pos = 1};
const field32_t SCB_AIRCR_SYSRESETREQ          = {.msk = 0x00000004U, .pos = 2};
const field32_t SCB_AIRCR_PRIGROUP             = {.msk = 0x00000700U, .pos = 8};
const field32_t SCB_AIRCR_ENDIANESS            = {.msk = 0x00008000U, .pos = 15};
const field32_t SCB_AIRCR_VECTKEYSTAT          = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t SCB_SCR_SLEEPONEXIT            = {.msk = 0x00000002U, .pos = 1};
const field32_t SCB_SCR_SLEEPDEEP              = {.msk = 0x00000004U, .pos = 2};
const field32_t SCB_SCR_SEVEONPEND             = {.msk = 0x00000010U, .pos = 4};
const field32_t SCB_CCR_NONBASETHRDENA         = {.msk = 0x00000001U, .pos = 0};
const field32_t SCB_CCR_USERSETMPEND           = {.msk = 0x00000002U, .pos = 1};
const field32_t SCB_CCR_UNALIGN__TRP           = {.msk = 0x00000008U, .pos = 3};
const field32_t SCB_CCR_DIV_0_TRP              = {.msk = 0x00000010U, .pos = 4};
const field32_t SCB_CCR_BFHFNMIGN              = {.msk = 0x00000100U, .pos = 8};
const field32_t SCB_CCR_STKALIGN               = {.msk = 0x00000200U, .pos = 9};
const field32_t SCB_CCR_DC                     = {.msk = 0x00010000U, .pos = 16};
const field32_t SCB_CCR_IC                     = {.msk = 0x00020000U, .pos = 17};
const field32_t SCB_CCR_BP                     = {.msk = 0x00040000U, .pos = 18};
const field32_t SCB_SHPR2_PRI_11               = {.msk = 0xFF000000U, .pos = 24};
const field32_t SCB_SHCRS_MEMFAULTACT          = {.msk = 0x00000001U, .pos = 0};
const field32_t SCB_SHCRS_BUSFAULTACT          = {.msk = 0x00000002U, .pos = 1};
const field32_t SCB_SHCRS_USGFAULTACT          = {.msk = 0x00000008U, .pos = 3};
const field32_t SCB_SHCRS_SVCALLACT            = {.msk = 0x00000080U, .pos = 7};
const field32_t SCB_SHCRS_MONITORACT           = {.msk = 0x00000100U, .pos = 8};
const field32_t SCB_SHCRS_PENDSVACT            = {.msk = 0x00000400U, .pos = 10};
const field32_t SCB_SHCRS_SYSTICKACT           = {.msk = 0x00000800U, .pos = 11};
const field32_t SCB_SHCRS_USGFAULTPENDED       = {.msk = 0x00001000U, .pos = 12};
const field32_t SCB_SHCRS_MEMFAULTPENDED       = {.msk = 0x00002000U, .pos = 13};
const field32_t SCB_SHCRS_BUSFAULTPENDED       = {.msk = 0x00004000U, .pos = 14};
const field32_t SCB_SHCRS_SVCALLPENDED         = {.msk = 0x00008000U, .pos = 15};
const field32_t SCB_SHCRS_MEMFAULTENA          = {.msk = 0x00010000U, .pos = 16};
const field32_t SCB_SHCRS_BUSFAULTENA          = {.msk = 0x00020000U, .pos = 17};
const field32_t SCB_SHCRS_USGFAULTENA          = {.msk = 0x00040000U, .pos = 18};
const field32_t SCB_CFSR_IACCVIOL              = {.msk = 0x00000001U, .pos = 0};
const field32_t SCB_CFSR_DACCVIOL              = {.msk = 0x00000002U, .pos = 1};
const field32_t SCB_CFSR_MUNSTKERR             = {.msk = 0x00000008U, .pos = 3};
const field32_t SCB_CFSR_MSTKERR               = {.msk = 0x00000010U, .pos = 4};
const field32_t SCB_CFSR_MLSPERR               = {.msk = 0x00000020U, .pos = 5};
const field32_t SCB_CFSR_MMARVALID             = {.msk = 0x00000080U, .pos = 7};
const field32_t SCB_CFSR_IBUSERR               = {.msk = 0x00000100U, .pos = 8};
const field32_t SCB_CFSR_PRECISERR             = {.msk = 0x00000200U, .pos = 9};
const field32_t SCB_CFSR_IMPRECISERR           = {.msk = 0x00000400U, .pos = 10};
const field32_t SCB_CFSR_UNSTKERR              = {.msk = 0x00000800U, .pos = 11};
const field32_t SCB_CFSR_STKERR                = {.msk = 0x00001000U, .pos = 12};
const field32_t SCB_CFSR_LSPERR                = {.msk = 0x00002000U, .pos = 13};
const field32_t SCB_CFSR_BFARVALID             = {.msk = 0x00008000U, .pos = 15};
const field32_t SCB_CFSR_UNDEFINSTR            = {.msk = 0x00010000U, .pos = 16};
const field32_t SCB_CFSR_INVSTATE              = {.msk = 0x00020000U, .pos = 17};
const field32_t SCB_CFSR_INVPC                 = {.msk = 0x00040000U, .pos = 18};
const field32_t SCB_CFSR_NOCP                  = {.msk = 0x00080000U, .pos = 19};
const field32_t SCB_CFSR_UNALIGNED             = {.msk = 0x01000000U, .pos = 24};
const field32_t SCB_CFSR_DIVBYZERO             = {.msk = 0x02000000U, .pos = 25};
const field32_t SCB_HFSR_VECTTBL               = {.msk = 0x00000002U, .pos = 1};
const field32_t SCB_HFSR_FORCED                = {.msk = 0x40000000U, .pos = 30};
const field32_t SCB_HFSR_DEBUG_VT              = {.msk = 0x80000000U, .pos = 31};
const field32_t SCB_ACTRL_ACTRL_DISFOLD        = {.msk = 0x00000004U, .pos = 2};
const field32_t SCB_ACTRL_ACTRL_FPEXCODIS      = {.msk = 0x00000400U, .pos = 10};
const field32_t SCB_ACTRL_ACTRL_DISRAMODE      = {.msk = 0x00000800U, .pos = 11};
const field32_t SCB_ACTRL_ACTRL_DISITMATBFLUSH = {.msk = 0x00001000U, .pos = 12};

/** @subsection Enumerated SCB Register Field Definitions */

field32_t const SCB_SHPR1_PRI_x[7] = {
  [4] = {.msk = 0x000000FFU, .pos = 0},
  [5] = {.msk = 0x0000FF00U, .pos = 8},
  [6] = {.msk = 0x00FF0000U, .pos = 16},
};

field32_t const SCB_SHPR3_PRI_1x[6] = {
  [4] = {.msk = 0x00FF0000U, .pos = 16},
  [5] = {.msk = 0xFF000000U, .pos = 24},
};

/**************************************************************************************************
 * @section Debug Definitions
 **************************************************************************************************/

/** @subsection Debug Register Definitions */

rw_reg32_t const DBG_DHCSR   = (rw_reg32_t)0xE000EDF0U;
rw_reg32_t const DBG_DCRSR   = (rw_reg32_t)0xE000EDF4U;
rw_reg32_t const DBG_DCRDR   = (rw_reg32_t)0xE000EDF8U;
rw_reg32_t const DBG_DEMCR   = (rw_reg32_t)0xE000EDFCU;

/** @subsection Debug Register Field Definitions */

const field32_t DBG_DHCSR_C_DEBUGEN    = {.msk = 0x00000001U, .pos = 0};
const field32_t DBG_DHCSR_C_HALT       = {.msk = 0x00000002U, .pos = 1};
const field32_t DBG_DHCSR_C_STEP       = {.msk = 0x00000004U, .pos = 2};
const field32_t DBG_DHCSR_C_MASKINTS   = {.msk = 0x00000008U, .pos = 3};
const field32_t DBG_DHCSR_C_SNAPSTALL  = {.msk = 0x00000020U, .pos = 5};
const field32_t DBG_DHCSR_S_REGRDY     = {.msk = 0x00010000U, .pos = 16};
const field32_t DBG_DHCSR_S_HALT       = {.msk = 0x00020000U, .pos = 17};
const field32_t DBG_DHCSR_S_SLEEP      = {.msk = 0x00040000U, .pos = 18};
const field32_t DBG_DHCSR_S_LOCKUP     = {.msk = 0x00080000U, .pos = 19};
const field32_t DBG_DHCSR_S_RETIRE_ST  = {.msk = 0x01000000U, .pos = 24};
const field32_t DBG_DHCSR_S_RESET_ST   = {.msk = 0x02000000U, .pos = 25};
const field32_t DBG_DHCSR_DBGKEY       = {.msk = 0xFFFF0000U, .pos = 16};
const field32_t DBG_DCRSR_REGSEL       = {.msk = 0x0000001FU, .pos = 0};
const field32_t DBG_DCRSR_REGWnR       = {.msk = 0x00010000U, .pos = 16};
const field32_t DBG_DCRDR_DBGTMP       = {.msk = 0xFFFFFFFFU, .pos = 0};
const field32_t DBG_DEMCR_VC_CORERESET = {.msk = 0x00000001U, .pos = 0};
const field32_t DBG_DEMCR_VC_MMERR     = {.msk = 0x00000010U, .pos = 4};
const field32_t DBG_DEMCR_VC_NOCPERR   = {.msk = 0x00000020U, .pos = 5};
const field32_t DBG_DEMCR_VC_CHKERR    = {.msk = 0x00000040U, .pos = 6};
const field32_t DBG_DEMCR_VC_STATERR   = {.msk = 0x00000080U, .pos = 7};
const field32_t DBG_DEMCR_VC_BUSERR    = {.msk = 0x00000100U, .pos = 8};
const field32_t DBG_DEMCR_VC_INTERR    = {.msk = 0x00000200U, .pos = 9};
const field32_t DBG_DEMCR_VC_HARDERR   = {.msk = 0x00000400U, .pos = 10};
const field32_t DBG_DEMCR_MON_EN       = {.msk = 0x00010000U, .pos = 16};
const field32_t DBG_DEMCR_MON_PEND     = {.msk = 0x00020000U, .pos = 17};
const field32_t DBG_DEMCR_MON_STEP     = {.msk = 0x00040000U, .pos = 18};
const field32_t DBG_DEMCR_MON_REQ      = {.msk = 0x00080000U, .pos = 19};
const field32_t DBG_DEMCR_TRCENA       = {.msk = 0x01000000U, .pos = 24};

/**************************************************************************************************
 * @section PF Definitions
 **************************************************************************************************/

/** @subsection PF Register Definitions */

ro_reg32_t const PF_CLIDR  = (ro_reg32_t)0xE000ED78U;
ro_reg32_t const PF_CTR    = (ro_reg32_t)0xE000ED7CU;
ro_reg32_t const PF_CCSIDR = (ro_reg32_t)0xE000ED80U;

/** @subsection PF Register Field Definitions */

const field32_t PF_CLIDR_LOUIS          = {.msk = 0x00E00000U, .pos = 21};
const field32_t PF_CLIDR_LOC            = {.msk = 0x07000000U, .pos = 24};
const field32_t PF_CLIDR_LOU            = {.msk = 0x38000000U, .pos = 27};
const field32_t PF_CTR_IMINLINE         = {.msk = 0x0000000FU, .pos = 0};
const field32_t PF_CTR_DMINLINE         = {.msk = 0x000F0000U, .pos = 16};
const field32_t PF_CTR_ERG              = {.msk = 0x00F00000U, .pos = 20};
const field32_t PF_CTR_CWG              = {.msk = 0x0F000000U, .pos = 24};
const field32_t PF_CTR_FORMAT           = {.msk = 0xE0000000U, .pos = 29};
const field32_t PF_CCSIDR_LINESIZE      = {.msk = 0x00000007U, .pos = 0};
const field32_t PF_CCSIDR_ASSOCIATIVITY = {.msk = 0x00001FF8U, .pos = 3};
const field32_t PF_CCSIDR_NUMSETS       = {.msk = 0x0FFFE000U, .pos = 13};
const field32_t PF_CCSIDR_WA            = {.msk = 0x10000000U, .pos = 28};
const field32_t PF_CCSIDR_RA            = {.msk = 0x20000000U, .pos = 29};
const field32_t PF_CCSIDR_WB            = {.msk = 0x40000000U, .pos = 30};
const field32_t PF_CCSIDR_WT            = {.msk = 0x80000000U, .pos = 31};

/** @subsection Enumerated PF Register Field Definitions */

field32_t const PF_CLIDR_CLx[8] = {
  [1] = {.msk = 0x00000007U, .pos = 0},
  [2] = {.msk = 0x00000038U, .pos = 3},
  [3] = {.msk = 0x000001C0U, .pos = 6},
  [4] = {.msk = 0x00000E00U, .pos = 9},
  [5] = {.msk = 0x00007000U, .pos = 12},
  [6] = {.msk = 0x00038000U, .pos = 15},
  [7] = {.msk = 0x001C0000U, .pos = 18},
};

/**************************************************************************************************
 * @section AC Definitions
 **************************************************************************************************/

/** @subsection AC Register Definitions */

rw_reg32_t const AC_ITCMCR = (rw_reg32_t)0xE000EF90U;
rw_reg32_t const AC_DTCMCR = (rw_reg32_t)0xE000EF94U;
rw_reg32_t const AC_AHBPCR = (rw_reg32_t)0xE000EF98U;
rw_reg32_t const AC_CACR   = (rw_reg32_t)0xE000EF9CU;
rw_reg32_t const AC_AHBSCR = (rw_reg32_t)0xE000EFA0U;
rw_reg32_t const AC_ABFSR  = (rw_reg32_t)0xE000EFA8U;

/** @subsection AC Register Field Definitions */

const field32_t AC_ITCMCR_EN        = {.msk = 0x00000001U, .pos = 0};
const field32_t AC_ITCMCR_RMW       = {.msk = 0x00000002U, .pos = 1};
const field32_t AC_ITCMCR_RETEN     = {.msk = 0x00000004U, .pos = 2};
const field32_t AC_ITCMCR_SZ        = {.msk = 0x00000078U, .pos = 3};
const field32_t AC_DTCMCR_EN        = {.msk = 0x00000001U, .pos = 0};
const field32_t AC_DTCMCR_RMW       = {.msk = 0x00000002U, .pos = 1};
const field32_t AC_DTCMCR_RETEN     = {.msk = 0x00000004U, .pos = 2};
const field32_t AC_DTCMCR_SZ        = {.msk = 0x00000078U, .pos = 3};
const field32_t AC_AHBPCR_EN        = {.msk = 0x00000001U, .pos = 0};
const field32_t AC_AHBPCR_SZ        = {.msk = 0x0000000EU, .pos = 1};
const field32_t AC_CACR_SIWT        = {.msk = 0x00000001U, .pos = 0};
const field32_t AC_CACR_ECCEN       = {.msk = 0x00000002U, .pos = 1};
const field32_t AC_CACR_FORCEWT     = {.msk = 0x00000004U, .pos = 2};
const field32_t AC_AHBSCR_CTL       = {.msk = 0x00000003U, .pos = 0};
const field32_t AC_AHBSCR_TPRI      = {.msk = 0x000007FCU, .pos = 2};
const field32_t AC_AHBSCR_INITCOUNT = {.msk = 0x0000F800U, .pos = 11};
const field32_t AC_ABFSR_ITCM       = {.msk = 0x00000001U, .pos = 0};
const field32_t AC_ABFSR_DTCM       = {.msk = 0x00000002U, .pos = 1};
const field32_t AC_ABFSR_AHBP       = {.msk = 0x00000004U, .pos = 2};
const field32_t AC_ABFSR_AXIM       = {.msk = 0x00000008U, .pos = 3};
const field32_t AC_ABFSR_EPPB       = {.msk = 0x00000010U, .pos = 4};
const field32_t AC_ABFSR_AXIMTYPE   = {.msk = 0x00000300U, .pos = 8};

/**************************************************************************************************
 * @section RAMECCx Definitions
 **************************************************************************************************/

/** @subsection Enumerated RAMECCx Register Definitions */

rw_reg32_t const RAMECCx_IER[4] = {
  [1] = (rw_reg32_t)0x52009000U,
  [2] = (rw_reg32_t)0x48023000U,
  [3] = (rw_reg32_t)0x58027000U,
};

rw_reg32_t const RAMECCx_M1CR[4] = {
  [1] = (rw_reg32_t)0x52009020U,
  [2] = (rw_reg32_t)0x48023020U,
  [3] = (rw_reg32_t)0x58027020U,
};

rw_reg32_t const RAMECCx_M1SR[4] = {
  [1] = (rw_reg32_t)0x52009024U,
  [2] = (rw_reg32_t)0x48023024U,
  [3] = (rw_reg32_t)0x58027024U,
};

rw_reg32_t const RAMECCx_M1FAR[4] = {
  [1] = (rw_reg32_t)0x52009028U,
  [2] = (rw_reg32_t)0x48023028U,
  [3] = (rw_reg32_t)0x58027028U,
};

rw_reg32_t const RAMECCx_M1FDRL[4] = {
  [1] = (rw_reg32_t)0x5200902CU,
  [2] = (rw_reg32_t)0x4802302CU,
  [3] = (rw_reg32_t)0x5802702CU,
};

rw_reg32_t const RAMECCx_M1FDRH[4] = {
  [1] = (rw_reg32_t)0x52009030U,
  [2] = (rw_reg32_t)0x48023030U,
  [3] = (rw_reg32_t)0x58027030U,
};

rw_reg32_t const RAMECCx_M1FECR[4] = {
  [1] = (rw_reg32_t)0x52009034U,
  [2] = (rw_reg32_t)0x48023034U,
  [3] = (rw_reg32_t)0x58027034U,
};

rw_reg32_t const RAMECCx_M2CR[4] = {
  [1] = (rw_reg32_t)0x52009040U,
  [2] = (rw_reg32_t)0x48023040U,
  [3] = (rw_reg32_t)0x58027040U,
};

rw_reg32_t const RAMECCx_M2SR[4] = {
  [1] = (rw_reg32_t)0x52009044U,
  [2] = (rw_reg32_t)0x48023044U,
  [3] = (rw_reg32_t)0x58027044U,
};

rw_reg32_t const RAMECCx_M2FAR[4] = {
  [1] = (rw_reg32_t)0x52009048U,
  [2] = (rw_reg32_t)0x48023048U,
  [3] = (rw_reg32_t)0x58027048U,
};

rw_reg32_t const RAMECCx_M2FDRL[4] = {
  [1] = (rw_reg32_t)0x5200904CU,
  [2] = (rw_reg32_t)0x4802304CU,
  [3] = (rw_reg32_t)0x5802704CU,
};

ro_reg32_t const RAMECCx_M2FDRH[4] = {
  [1] = (ro_reg32_t)0x52009050U,
  [2] = (ro_reg32_t)0x48023050U,
  [3] = (ro_reg32_t)0x58027050U,
};

ro_reg32_t const RAMECCx_M2FECR[4] = {
  [1] = (ro_reg32_t)0x52009058U,
  [2] = (ro_reg32_t)0x48023058U,
  [3] = (ro_reg32_t)0x58027058U,
};

ro_reg32_t const RAMECCx_M3CR[4] = {
  [1] = (ro_reg32_t)0x52009060U,
  [2] = (ro_reg32_t)0x48023060U,
};

ro_reg32_t const RAMECCx_M3SR[4] = {
  [1] = (ro_reg32_t)0x52009064U,
  [2] = (ro_reg32_t)0x48023064U,
};

rw_reg32_t const RAMECCx_M3FAR[4] = {
  [1] = (rw_reg32_t)0x52009068U,
  [2] = (rw_reg32_t)0x48023068U,
};

ro_reg32_t const RAMECCx_M3FDRL[4] = {
  [1] = (ro_reg32_t)0x5200906CU,
  [2] = (ro_reg32_t)0x4802306CU,
};

ro_reg32_t const RAMECCx_M3FDRH[4] = {
  [1] = (ro_reg32_t)0x52009070U,
  [2] = (ro_reg32_t)0x48023070U,
};

ro_reg32_t const RAMECCx_M3FECR[4] = {
  [1] = (ro_reg32_t)0x5200907CU,
  [2] = (ro_reg32_t)0x4802307CU,
};

ro_reg32_t const RAMECCx_M4CR[4] = {
  [1] = (ro_reg32_t)0x52009080U,
  [2] = (ro_reg32_t)0x48023080U,
};

ro_reg32_t const RAMECCx_M4SR[4] = {
  [1] = (ro_reg32_t)0x52009084U,
  [2] = (ro_reg32_t)0x48023084U,
};

ro_reg32_t const RAMECCx_M4FAR[4] = {
  [1] = (ro_reg32_t)0x52009088U,
  [2] = (ro_reg32_t)0x48023088U,
};

rw_reg32_t const RAMECCx_M4FDRL[4] = {
  [1] = (rw_reg32_t)0x5200908CU,
  [2] = (rw_reg32_t)0x4802308CU,
};

ro_reg32_t const RAMECCx_M4FDRH[4] = {
  [1] = (ro_reg32_t)0x52009090U,
  [2] = (ro_reg32_t)0x48023090U,
};

ro_reg32_t const RAMECCx_M4FECR[4] = {
  [1] = (ro_reg32_t)0x52009090U,
  [2] = (ro_reg32_t)0x48023090U,
};

ro_reg32_t const RAMECCx_M5CR[4] = {
  [1] = (ro_reg32_t)0x520090A0U,
  [2] = (ro_reg32_t)0x480230A0U,
};

rw_reg32_t const RAMECCx_M5SR[4] = {
  [1] = (rw_reg32_t)0x520090A4U,
  [2] = (rw_reg32_t)0x480230A4U,
};

rw_reg32_t const RAMECCx_M5FAR[4] = {
  [1] = (rw_reg32_t)0x520090A8U,
  [2] = (rw_reg32_t)0x480230A8U,
};

ro_reg32_t const RAMECCx_M5FDRL[4] = {
  [1] = (ro_reg32_t)0x520090ACU,
  [2] = (ro_reg32_t)0x480230ACU,
};

ro_reg32_t const RAMECCx_M5FDRH[4] = {
  [1] = (ro_reg32_t)0x520090B0U,
  [2] = (ro_reg32_t)0x480230B0U,
};

ro_reg32_t const RAMECCx_M5FECR[4] = {
  [1] = (ro_reg32_t)0x520090B4U,
  [2] = (ro_reg32_t)0x480230B4U,
};

/** @subsection RAMECCx Register Field Definitions */

const field32_t RAMECCx_IER_GIE          = {.msk = 0x00000001U, .pos = 0};
const field32_t RAMECCx_IER_GECCSEIE_    = {.msk = 0x00000002U, .pos = 1};
const field32_t RAMECCx_IER_GECCDEIE     = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_IER_GECCDEBWIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t RAMECCx_M1CR_ECCSEIE     = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M1CR_ECCDEIE     = {.msk = 0x00000008U, .pos = 3};
const field32_t RAMECCx_M1CR_ECCDEBWIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t RAMECCx_M1CR_ECCELEN     = {.msk = 0x00000020U, .pos = 5};
const field32_t RAMECCx_M1SR_ECCSEIE     = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M1SR_ECCDEIE     = {.msk = 0x00000008U, .pos = 3};
const field32_t RAMECCx_M1SR_ECCDEBWIE   = {.msk = 0x00000010U, .pos = 4};
const field32_t RAMECCx_M1SR_ECCELEN     = {.msk = 0x00000020U, .pos = 5};
const field32_t RAMECCx_M1FAR_ECCSEIE    = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M1FAR_ECCDEIE    = {.msk = 0x00000008U, .pos = 3};
const field32_t RAMECCx_M1FAR_ECCDEBWIE  = {.msk = 0x00000010U, .pos = 4};
const field32_t RAMECCx_M1FAR_ECCELEN    = {.msk = 0x00000020U, .pos = 5};
const field32_t RAMECCx_M1FDRL_ECCSEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M1FDRL_ECCDEIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t RAMECCx_M1FDRL_ECCDEBWIE = {.msk = 0x00000010U, .pos = 4};
const field32_t RAMECCx_M1FDRL_ECCELEN   = {.msk = 0x00000020U, .pos = 5};
const field32_t RAMECCx_M1FDRH_ECCSEIE   = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M1FDRH_ECCDEIE   = {.msk = 0x00000008U, .pos = 3};
const field32_t RAMECCx_M1FDRH_ECCDEBWIE = {.msk = 0x00000010U, .pos = 4};
const field32_t RAMECCx_M1FDRH_ECCELEN   = {.msk = 0x00000020U, .pos = 5};
const field32_t RAMECCx_M1FECR_SEDCF     = {.msk = 0x00000001U, .pos = 0};
const field32_t RAMECCx_M1FECR_DEDF      = {.msk = 0x00000002U, .pos = 1};
const field32_t RAMECCx_M1FECR_DEBWDF    = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M2CR_SEDCF       = {.msk = 0x00000001U, .pos = 0};
const field32_t RAMECCx_M2CR_DEDF        = {.msk = 0x00000002U, .pos = 1};
const field32_t RAMECCx_M2CR_DEBWDF      = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M2SR_SEDCF       = {.msk = 0x00000001U, .pos = 0};
const field32_t RAMECCx_M2SR_DEDF        = {.msk = 0x00000002U, .pos = 1};
const field32_t RAMECCx_M2SR_DEBWDF      = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M2FAR_SEDCF      = {.msk = 0x00000001U, .pos = 0};
const field32_t RAMECCx_M2FAR_DEDF       = {.msk = 0x00000002U, .pos = 1};
const field32_t RAMECCx_M2FAR_DEBWDF     = {.msk = 0x00000004U, .pos = 2};
const field32_t RAMECCx_M2FDRL_SEDCF     = {.msk = 0x00000001U, .pos = 0};
const field32_t RAMECCx_M2FDRL_DEDF      = {.msk = 0x00000002U, .pos = 1};
const field32_t RAMECCx_M2FDRL_DEBWDF    = {.msk = 0x00000004U, .pos = 2};

/**************************************************************************************************
 * @section ART Definitions
 **************************************************************************************************/

/** @subsection ART Register Definitions */

rw_reg32_t const ART_CTR = (rw_reg32_t)0x40024400U;

/** @subsection ART Register Field Definitions */

const field32_t ART_CTR_EN         = {.msk = 0x00000001U, .pos = 0};
const field32_t ART_CTR_PCACHEADDR = {.msk = 0x000FFF00U, .pos = 8};