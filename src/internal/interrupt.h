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
 * @file internal/interrupt.h
 * @authors Aaron McBride
 * @brief Reference information and prototypes for system interrupts.
 */

#pragma once
#include <stdint.h>

/**************************************************************************************************
 * @section Exception Handler Prototypes
 **************************************************************************************************/

/** @subsection CM7 exception handlers */

__attribute__((weak)) void cm7_reset_exc_handler();       /** @brief CM7_Reset exception. */
__attribute__((weak)) void cm7_nmi_exc_handler();         /** @brief CM7_Non-maskable interrupt. */
__attribute__((weak)) void cm7_hard_fault_exc_handler();  /** @brief CM7_Hard fault. */
__attribute__((weak)) void cm7_mem_manage_exc_handler();  /** @brief CM7_Memory management fault (MPU). */
__attribute__((weak)) void cm7_bus_fault_exc_handler();   /** @brief CM7_Bus fault. */
__attribute__((weak)) void cm7_usage_fault_exc_handler(); /** @brief CM7_Usage fault. */
__attribute__((weak)) void cm7_svc_exc_handler();         /** @brief CM7_System service call exception. */
__attribute__((weak)) void cm7_debug_mon_exc_handler();   /** @brief CM7_Debug monitor fault/exception. */
__attribute__((weak)) void cm7_pendsv_exc_handler();      /** @brief CM7_Pended system service call exception. */
__attribute__((weak)) void cm7_systick_exc_handler();     /** @brief CM7_SysTick interrupt. */

/** @subsection CM4 exception handlers */

__attribute__((weak)) void cm4_reset_exc_handler();       /** @brief CM4_Reset exception. */
__attribute__((weak)) void cm4_nmi_exc_handler();         /** @brief CM4_Non-maskable interrupt. */
__attribute__((weak)) void cm4_hard_fault_exc_handler();  /** @brief CM4_Hard fault. */
__attribute__((weak)) void cm4_mem_manage_exc_handler();  /** @brief CM4_Memory management fault (MPU). */
__attribute__((weak)) void cm4_bus_fault_exc_handler();   /** @brief CM4_Bus fault. */
__attribute__((weak)) void cm4_usage_fault_exc_handler(); /** @brief CM4_Usage fault. */
__attribute__((weak)) void cm4_svc_exc_handler();         /** @brief CM4_System service call exception. */
__attribute__((weak)) void cm4_debug_mon_exc_handler();   /** @brief CM4_Debug monitor fault/exception. */
__attribute__((weak)) void cm4_pendsv_exc_handler();      /** @brief CM4_Pended system service call exception. */
__attribute__((weak)) void cm4_systick_exc_handler();     /** @brief CM4_SysTick interrupt. */

/**************************************************************************************************
 * @section IRQ Handler Prototypes
 **************************************************************************************************/

__attribute__((weak)) void wwdg1_irq_handler();                  /** @brief Window watchdog interrupt (CM7 core). */
__attribute__((weak)) void wwdg2_irq_handler();                  /** @brief Window watchdog interrupt (CM4 core). */
__attribute__((weak)) void pvd_pvm_irq_handler();                /** @brief PVD through EXTI line. */
__attribute__((weak)) void rtc_tamp_stamp_css_lse_irq_handler(); /** @brief RTC tamper, timestamp. */
__attribute__((weak)) void rtc_wkup_irq_handler();               /** @brief RTC wakeup interrupt. */
__attribute__((weak)) void flash_irq_handler();                  /** @brief Flash memory. */
__attribute__((weak)) void rcc_irq_handler();                    /** @brief RCC global interrupt. */
__attribute__((weak)) void exti0_irq_handler();                  /** @brief EXTI line 0 interrupt. */
__attribute__((weak)) void exti1_irq_handler();                  /** @brief EXTI line 1 interrupt. */
__attribute__((weak)) void exti2_irq_handler();                  /** @brief EXTI line 2 interrupt. */
__attribute__((weak)) void exti3_irq_handler();                  /** @brief EXTI line 3 interrupt. */
__attribute__((weak)) void exti4_irq_handler();                  /** @brief EXTI line 4 interrupt. */
__attribute__((weak)) void dma_str0_irq_handler();               /** @brief DMA1 stream0. */
__attribute__((weak)) void dma_str1_irq_handler();               /** @brief DMA1 stream1. */
__attribute__((weak)) void dma_str2_irq_handler();               /** @brief DMA1 stream2. */
__attribute__((weak)) void dma_str3_irq_handler();               /** @brief DMA1 stream3. */
__attribute__((weak)) void dma_str4_irq_handler();               /** @brief DMA1 stream4. */
__attribute__((weak)) void dma_str5_irq_handler();               /** @brief DMA1 stream5. */
__attribute__((weak)) void dma_str6_irq_handler();               /** @brief DMA1 stream6. */
__attribute__((weak)) void adc1_2_irq_handler();                 /** @brief ADC1 and ADC2. */
__attribute__((weak)) void fdcan1_it0_irq_handler();             /** @brief FDCAN1 interrupt 0. */
__attribute__((weak)) void fdcan2_it0_irq_handler();             /** @brief FDCAN2 interrupt 0. */
__attribute__((weak)) void fdcan1_it1_irq_handler();             /** @brief FDCAN1 interrupt 1. */
__attribute__((weak)) void fdcan2_it1_irq_handler();             /** @brief FDCAN2 interrupt 1. */
__attribute__((weak)) void exti9_5_irq_handler();                /** @brief EXTI line[9:5] interrupts. */
__attribute__((weak)) void tim1_brk_irq_handler();               /** @brief TIM1 break interrupt. */
__attribute__((weak)) void tim1_up_irq_handler();                /** @brief TIM1 update interrupt. */
__attribute__((weak)) void tim1_trg_com_irq_handler();           /** @brief TIM1 trigger and commutation. */
__attribute__((weak)) void tim1_cc_irq_handler();                /** @brief TIM1 capture / compare. */
__attribute__((weak)) void tim2_irq_handler();                   /** @brief TIM2 global interrupt. */
__attribute__((weak)) void tim3_irq_handler();                   /** @brief TIM3 global interrupt. */
__attribute__((weak)) void tim4_irq_handler();                   /** @brief TIM4 global interrupt. */
__attribute__((weak)) void i2c1_ev_irq_handler();                /** @brief I2C1 event interrupt. */
__attribute__((weak)) void i2c1_er_irq_handler();                /** @brief I2C1 error interrupt. */
__attribute__((weak)) void i2c2_ev_irq_handler();                /** @brief I2C2 event interrupt. */
__attribute__((weak)) void i2c2_er_irq_handler();                /** @brief I2C2 error interrupt. */
__attribute__((weak)) void spi1_irq_handler();                   /** @brief SPI1 global interrupt. */
__attribute__((weak)) void spi2_irq_handler();                   /** @brief SPI2 global interrupt. */
__attribute__((weak)) void usart1_irq_handler();                 /** @brief USART1 global interrupt. */
__attribute__((weak)) void usart2_irq_handler();                 /** @brief USART2 global interrupt. */
__attribute__((weak)) void usart3_irq_handler();                 /** @brief USART3 global interrupt. */
__attribute__((weak)) void exti15_10_irq_handler();              /** @brief EXTI line[15:10] interrupts. */
__attribute__((weak)) void rtc_alarm_irq_handler();              /** @brief RTC alarms (A and B). */
__attribute__((weak)) void tim8_brk_tim12_irq_handler();         /** @brief TIM8 and 12 break global. */
__attribute__((weak)) void tim8_up_tim13_irq_handler();          /** @brief TIM8 and 13 update global. */
__attribute__((weak)) void tim8_trg_com_tim14_irq_handler();     /** @brief TIM8 and 14 trigger /commutation and global. */
__attribute__((weak)) void tim8_cc_irq_handler();                /** @brief TIM8 capture / compare. */
__attribute__((weak)) void dma1_str7_irq_handler();              /** @brief DMA1 stream7. */
__attribute__((weak)) void fmc_irq_handler();                    /** @brief FMC global interrupt. */
__attribute__((weak)) void sdmmc1_irq_handler();                 /** @brief SDMMC global interrupt. */
__attribute__((weak)) void tim5_irq_handler();                   /** @brief TIM5 global interrupt. */
__attribute__((weak)) void spi3_irq_handler();                   /** @brief SPI3 global interrupt. */
__attribute__((weak)) void uart4_irq_handler();                  /** @brief UART4 global interrupt. */
__attribute__((weak)) void uart5_irq_handler();                  /** @brief UART5 global interrupt. */
__attribute__((weak)) void tim6_dac_irq_handler();               /** @brief TIM6 global interrupt. */
__attribute__((weak)) void tim7_irq_handler();                   /** @brief TIM7 global interrupt. */
__attribute__((weak)) void dma2_str0_irq_handler();              /** @brief DMA2 stream0 interrupt. */
__attribute__((weak)) void dma2_str1_irq_handler();              /** @brief DMA2 stream1 interrupt. */
__attribute__((weak)) void dma2_str2_irq_handler();              /** @brief DMA2 stream2 interrupt. */
__attribute__((weak)) void dma2_str3_irq_handler();              /** @brief DMA2 stream3 interrupt. */
__attribute__((weak)) void dma2_str4_irq_handler();              /** @brief DMA2 stream4 interrupt. */
__attribute__((weak)) void eth_irq_handler();                    /** @brief Ethernet global interrupt. */
__attribute__((weak)) void eth_wkup_irq_handler();               /** @brief Ethernet wakeup through EXTI. */
__attribute__((weak)) void fdcan_cal_irq_handler();              /** @brief CAN2TX interrupts. */
__attribute__((weak)) void cpu1_sev_irq_handler();               /** @brief Arm cortex-m7 (cpu 1) send event interrupt (used by CM4 core). */
__attribute__((weak)) void cpu2_sev_irq_handler();               /** @brief Arm cortex-m4 (cpu 2) send event interrupt (used by CM7 core). */
__attribute__((weak)) void dma2_str5_irq_handler();              /** @brief DMA2 stream5 interrupt. */
__attribute__((weak)) void dma2_str6_irq_handler();              /** @brief DMA2 stream6 interrupt. */
__attribute__((weak)) void dma2_str7_irq_handler();              /** @brief DMA2 stream7 interrupt. */
__attribute__((weak)) void usart6_irq_handler();                 /** @brief USART6 global interrupt. */
__attribute__((weak)) void i2c3_ev_irq_handler();                /** @brief I2C3 event interrupt. */
__attribute__((weak)) void i2c3_er_irq_handler();                /** @brief I2C3 error interrupt. */
__attribute__((weak)) void otg_hs_ep1_out_irq_handler();         /** @brief OTG_HS out global interrupt. */
__attribute__((weak)) void otg_hs_ep1_in_irq_handler();          /** @brief OTG_HS in global interrupt. */
__attribute__((weak)) void otg_hs_wkup_irq_handler();            /** @brief OTG_HS wakeup interrupt. */
__attribute__((weak)) void otg_hs_irq_handler();                 /** @brief OTG_HS global interrupt. */
__attribute__((weak)) void dcmi_irq_handler();                   /** @brief DCMI global interrupt. */
__attribute__((weak)) void cryp_irq_handler();                   /** @brief CRYP global interrupt. */
__attribute__((weak)) void hash_rng_irq_handler();               /** @brief HASH and RNG. */
__attribute__((weak)) void cpu1_fpu_irq_handler();               /** @brief Floating point unit interrupt (CM7 core). */
__attribute__((weak)) void cpu2_fpu_irq_handler();               /** @brief Floating point unit interrupt (CM4 core). */
__attribute__((weak)) void uart7_irq_handler();                  /** @brief UART7 global interrupt. */
__attribute__((weak)) void uart8_irq_handler();                  /** @brief UART8 global interrupt. */
__attribute__((weak)) void spi4_irq_handler();                   /** @brief SPI4 global interrupt. */
__attribute__((weak)) void spi5_irq_handler();                   /** @brief SPI5 global interrupt. */
__attribute__((weak)) void spi6_irq_handler();                   /** @brief SPI6 global interrupt. */
__attribute__((weak)) void sai1_irq_handler();                   /** @brief SAI1 global interrupt. */
__attribute__((weak)) void ltdc_irq_handler();                   /** @brief LCD-TFT global interrupt. */
__attribute__((weak)) void ltdc_er_irq_handler();                /** @brief LCD-TFT error interrupt. */
__attribute__((weak)) void dma2d_irq_handler();                  /** @brief DMA2D global interrupt. */
__attribute__((weak)) void sai2_irq_handler();                   /** @brief SAI2 global interrupt. */
__attribute__((weak)) void quadspi_irq_handler();                /** @brief QuadSPI global interrupt. */
__attribute__((weak)) void lptim1_irq_handler();                 /** @brief LPTIM1 global interrupt. */
__attribute__((weak)) void cec_irq_handler();                    /** @brief HDMI-CEC global interrupt. */
__attribute__((weak)) void i2c4_ev_irq_handler();                /** @brief I2C4 event interrupt. */
__attribute__((weak)) void i2c4_er_irq_handler();                /** @brief I2C4 error interrupt. */
__attribute__((weak)) void spdif_irq_handler();                  /** @brief SPDIFRX global interrupt. */
__attribute__((weak)) void otg_fs_ep1_out_irq_handler();         /** @brief OTG_FS out global interrupt. */
__attribute__((weak)) void otg_fs_ep1_in_irq_handler();          /** @brief OTG_FS in global interrupt. */
__attribute__((weak)) void otg_fs_wkup_irq_handler();            /** @brief OTG_FS wakeup. */
__attribute__((weak)) void otg_fs_irq_handler();                 /** @brief OTG_FS global interrupt. */
__attribute__((weak)) void dmamux1_ov_irq_handler();             /** @brief DMAMUX1 overrun interrupt. */
__attribute__((weak)) void hrtim_mst_irq_handler();              /** @brief HRTIM master timer interrupt. */
__attribute__((weak)) void hrtim_tima_irq_handler();             /** @brief HRTIM timer A interrupt. */
__attribute__((weak)) void hrtim_timb_irq_handler();             /** @brief HRTIM timer B interrupt. */
__attribute__((weak)) void hrtim_timc_irq_handler();             /** @brief HRTIM timer C interrupt. */
__attribute__((weak)) void hrtim_timd_irq_handler();             /** @brief HRTIM timer D interrupt. */
__attribute__((weak)) void hrtim_time_irq_handler();             /** @brief HRTIM timer E interrupt. */
__attribute__((weak)) void hrtim_flt_irq_handler();              /** @brief HRTIM fault interrupt. */
__attribute__((weak)) void dfsdm1_flt0_irq_handler();            /** @brief DFSDM1 filter 0 interrupt. */
__attribute__((weak)) void dfsdm1_flt1_irq_handler();            /** @brief DFSDM1 filter 1 interrupt. */
__attribute__((weak)) void dfsdm1_flt2_irq_handler();            /** @brief DFSDM1 filter 2 interrupt. */
__attribute__((weak)) void dfsdm1_flt3_irq_handler();            /** @brief DFSDM1 filter 3 interrupt. */
__attribute__((weak)) void sai3_irq_handler();                   /** @brief SAI3 global interrupt. */
__attribute__((weak)) void swpmi_irq_handler();                  /** @brief SWPMI global interrupt. */
__attribute__((weak)) void tim15_irq_handler();                  /** @brief TIM15 global interrupt. */
__attribute__((weak)) void tim16_irq_handler();                  /** @brief TIM16 global interrupt. */
__attribute__((weak)) void tim17_irq_handler();                  /** @brief TIM17 global interrupt. */
__attribute__((weak)) void mdios_wkup_irq_handler();             /** @brief MDIOS wakeup. */
__attribute__((weak)) void mdios_irq_handler();                  /** @brief MDIOS global interrupt. */
__attribute__((weak)) void jpeg_irq_handler();                   /** @brief JPEG global interrupt. */
__attribute__((weak)) void mdma_irq_handler();                   /** @brief MDMA. */
__attribute__((weak)) void sdmmc2_irq_handler();                 /** @brief SDMMC global interrupt. */
__attribute__((weak)) void hsem0_irq_handler();                  /** @brief HSEM global interrupt 0 (CM7 core). */
__attribute__((weak)) void hsem1_irq_handler();                  /** @brief HSEM global interrupt 1 (CM4 core). */
__attribute__((weak)) void adc3_irq_handler();                   /** @brief ADC3 global interrupt. */
__attribute__((weak)) void dmamux2_ovr_irq_handler();            /** @brief DMAMUX2 overrun interrupt. */
__attribute__((weak)) void bdma_ch1_irq_handler();               /** @brief BDMA channel 1 interrupt. */
__attribute__((weak)) void bdma_ch2_irq_handler();               /** @brief BDMA channel 2 interrupt. */
__attribute__((weak)) void bdma_ch3_irq_handler();               /** @brief BDMA channel 3 interrupt. */
__attribute__((weak)) void bdma_ch4_irq_handler();               /** @brief BDMA channel 4 interrupt. */
__attribute__((weak)) void bdma_ch5_irq_handler();               /** @brief BDMA channel 5 interrupt. */
__attribute__((weak)) void bdma_ch6_irq_handler();               /** @brief BDMA channel 6 interrupt. */
__attribute__((weak)) void bdma_ch7_irq_handler();               /** @brief BDMA channel 7 interrupt. */
__attribute__((weak)) void bdma_ch8_irq_handler();               /** @brief BDMA channel 8 interrupt. */
__attribute__((weak)) void comp_irq_handler();                   /** @brief COMP1 and COMP2. */
__attribute__((weak)) void lptim2_irq_handler();                 /** @brief LPTIM2 timer interrupt. */
__attribute__((weak)) void lptim3_irq_handler();                 /** @brief LPTIM3 timer interrupt. */
__attribute__((weak)) void lptim4_irq_handler();                 /** @brief LPTIM4 timer interrupt. */
__attribute__((weak)) void lptim5_irq_handler();                 /** @brief LPTIM5 timer interrupt. */
__attribute__((weak)) void lpuart_irq_handler();                 /** @brief LPUART global interrupt. */
__attribute__((weak)) void wwdg1_rst_irq_handler();              /** @brief Window watchdog reset interrupt (used by CM4 core). */
__attribute__((weak)) void wwdg2_rst_irq_handler();              /** @brief Window watchdog reset interrupt (used by CM7 core). */
__attribute__((weak)) void crs_irq_handler();                    /** @brief Clock recovery system global interrupt. */
__attribute__((weak)) void sai4_irq_handler();                   /** @brief SAI4 global interrupt. */
__attribute__((weak)) void cpu1_hold_core_irq_handler();         /** @brief Hold core interrupt (CM7 core). */
__attribute__((weak)) void cpu2_hold_core_irq_handler();         /** @brief Hold core interrupt (CM4 core). */
__attribute__((weak)) void wkup_irq_handler();                   /** @brief WKUP1 to WKUP6 pins. */

/**************************************************************************************************
 * @section Miscellaneous Constants
 **************************************************************************************************/

/** @brief The number of implement IRQs */
extern const int32_t IRQ_COUNT;

/** @brief Number of priority bits supported the NVIC. */
extern const int32_t NVIC_PRIO_BITS;

/** @brief Maximum supported exception priority value. */
extern const int32_t NVIC_MAX_PRIO;

/** @brief Offset between exception and IRQ numbers. */
extern const int32_t IRQ_EXC_OFFSET;

/**************************************************************************************************
 * @section Exception Numbers
 **************************************************************************************************/

extern const int32_t RST_EXC_NUM;         /** @brief Reset exception (both cores). */
extern const int32_t NMI_EXC_NUM;         /** @brief Non-maskable interrupt (both cores). */
extern const int32_t HARD_FAULT_EXC_NUM;  /** @brief Hard fault (both cores). */
extern const int32_t MEM_MANAGE_EXC_NUM;  /** @brief Memory management fault (MPU (both cores)). */
extern const int32_t BUS_FAULT_EXC_NUM;   /** @brief Bus fault (both cores). */
extern const int32_t USAGE_FAULT_EXC_NUM; /** @brief Usage fault (both cores). */
extern const int32_t SVC_EXC_NUM;         /** @brief System service call exception (both cores). */
extern const int32_t DEBUG_MON_EXC_NUM;   /** @brief Debug monitor fault/exception (both cores). */
extern const int32_t PENDSV_EXC_NUM;      /** @brief Pended system service call exception (both cores). */
extern const int32_t SYSTICK_EXC_NUM;     /** @brief SysTick interrupt (both cores). */

/**************************************************************************************************
 * @section IRQ Numbers
 **************************************************************************************************/

/** @subsection Non-enumerated IRQ number constants. */

extern const int32_t PVD_PVM_IRQ_NUM;                /** @brief PVD through EXTI line. */
extern const int32_t RTC_TAMP_STAMP_CSS_LSE_IRQ_NUM; /** @brief RTC tamper, timestamp. */
extern const int32_t RTC_WKUP_IRQ_NUM;               /** @brief RTC wakeup interrupt. */
extern const int32_t FLASH_IRQ_NUM;                  /** @brief Flash memory. */
extern const int32_t RCC_IRQ_NUM;                    /** @brief RCC global interrupt. */
extern const int32_t ADC1_2_IRQ_NUM;                 /** @brief ADC1 and ADC2. */
extern const int32_t EXTI9_5_IRQ_NUM;                /** @brief EXTI line[9:5] interrupts. */
extern const int32_t TIM1_BRK_IRQ_NUM;               /** @brief TIM1 break interrupt. */
extern const int32_t TIM1_UP_IRQ_NUM;                /** @brief TIM1 update interrupt. */
extern const int32_t TIM1_TRG_COM_IRQ_NUM;           /** @brief TIM1 trigger and commutation. */
extern const int32_t EXTI15_10_IRQ_NUM;              /** @brief EXTI line[15:10] interrupts. */
extern const int32_t RTC_ALARM_IRQ_NUM;              /** @brief RTC alarms (A and B). */
extern const int32_t TIM8_BRK_TIM12_IRQ_NUM;         /** @brief TIM8 and 12 break global. */
extern const int32_t TIM8_UP_TIM13_IRQ_NUM;          /** @brief TIM8 and 13 update global. */
extern const int32_t TIM8_TRG_COM_TIM14_IRQ_NUM;     /** @brief TIM8 and 14 trigger /commutation and global. */
extern const int32_t FMC_IRQ_NUM;                    /** @brief FMC global interrupt. */
extern const int32_t TIM6_DAC_IRQ_NUM;               /** @brief TIM6 global interrupt. */
extern const int32_t ETH_IRQ_NUM;                    /** @brief Ethernet global interrupt. */
extern const int32_t ETH_WKUP_IRQ_NUM;               /** @brief Ethernet wakeup through EXTI. */
extern const int32_t FDCAN_CAL_IRQ_NUM;              /** @brief CAN2TX interrupts. */  
extern const int32_t OTG_HS_EP1_OUT_IRQ_NUM;         /** @brief OTG_HS out global interrupt. */
extern const int32_t OTG_HS_EP1_IN_IRQ_NUM;          /** @brief OTG_HS in global interrupt. */
extern const int32_t OTG_HS_WKUP_IRQ_NUM;            /** @brief OTG_HS wakeup interrupt. */
extern const int32_t OTG_HS_IRQ_NUM;                 /** @brief OTG_HS global interrupt. */
extern const int32_t DCMI_IRQ_NUM;                   /** @brief DCMI global interrupt. */
extern const int32_t CRYP_IRQ_NUM;                   /** @brief CRYP global interrupt. */
extern const int32_t HASH_RNG_IRQ_NUM;               /** @brief HASH and RNG. */
extern const int32_t LTDC_IRQ_NUM;                   /** @brief LCD-TFT global interrupt. */
extern const int32_t LTDC_ER_IRQ_NUM;                /** @brief LCD-TFT error interrupt. */
extern const int32_t DMA2D_IRQ_NUM;                  /** @brief DMA2D global interrupt. */
extern const int32_t QUADSPI_IRQ_NUM;                /** @brief QuadSPI global interrupt. */
extern const int32_t CEC_IRQ_NUM;                    /** @brief HDMI-CEC global interrupt. */
extern const int32_t SPDIF_IRQ_NUM;                  /** @brief SPDIFRX global interrupt. */
extern const int32_t OTG_FS_EP1_OUT_IRQ_NUM;         /** @brief OTG_FS out global interrupt. */
extern const int32_t OTG_FS_EP1_IN_IRQ_NUM;          /** @brief OTG_FS in global interrupt. */
extern const int32_t OTG_FS_WKUP_IRQ_NUM;            /** @brief OTG_FS wakeup. */
extern const int32_t OTG_FS_IRQ_NUM;                 /** @brief OTG_FS global interrupt. */
extern const int32_t DMAMUX1_OV_IRQ_NUM;             /** @brief DMAMUX1 overrun interrupt. */
extern const int32_t HRTIM_MST_IRQ_NUM;              /** @brief HRTIM master timer interrupt. */
extern const int32_t HRTIM_FLT_IRQ_NUM;              /** @brief HRTIM fault interrupt. */
extern const int32_t SWPMI_IRQ_NUM;                  /** @brief SWPMI global interrupt. */
extern const int32_t MDIOS_WKUP_IRQ_NUM;             /** @brief MDIOS wakeup. */
extern const int32_t MDIOS_IRQ_NUM;                  /** @brief MDIOS global interrupt. */
extern const int32_t JPEG_IRQ_NUM;                   /** @brief JPEG global interrupt. */
extern const int32_t MDMA_IRQ_NUM;                   /** @brief MDMA global interrupt. */
extern const int32_t ADC3_IRQ_NUM;                   /** @brief ADC3 global interrupt. */
extern const int32_t DMAMUX2_OVR_IRQ_NUM;            /** @brief DMAMUX2 overrun interrupt. */
extern const int32_t COMP_IRQ_NUM;                   /** @brief COMP1 and COMP2. */
extern const int32_t LPUART_IRQ_NUM;                 /** @brief LPUART global interrupt. */
extern const int32_t CRS_IRQ_NUM;                    /** @brief Clock recovery system globa. */
extern const int32_t WKUP_IRQ_NUM;                   /** @brief WKUP1 to WKUP6 pins. */

/** @subsection Enumerated IRQ number constants. */

extern const int32_t WWDGx_IRQ_NUM[3];          /** @brief Window watchdog interrupt (CM7/CM4 core). */
extern const int32_t CPUx_SEV_IRQ_NUM[3];       /** @brief Arm cortex-m7/m4 send event interrupt (used by CM4/CM7). */
extern const int32_t HSEMx_IRQ_NUM[2];          /** @brief HSEM global interrupt x. */
extern const int32_t WWDGx_RST_IRQ_NUM[3];      /** @brief Window watchdog x reset interrupt (used by CM7/CM4) */
extern const int32_t CPUx_FPU_IRQ_NUM[3];       /** @brief Floating point unit interrupt (used by CM4/CM7). */
extern const int32_t CPUx_HOLD_CORE_IRQ_NUM[3]; /** @brief Hold core interrupt (CM7/CM4 core). */
extern const int32_t HRTIM_TIMx_IRQ_NUM[6];     /** @brief HRTIM timer x interrupt (CM7/CM4 core). */
extern const int32_t BDMA_CHx_IRQ_NUM[9];       /** @brief BDMA channel x interrupt. */
extern const int32_t I2Cx_EV_IRQ_NUM[5];        /** @brief I2C event interrupt. */
extern const int32_t I2Cx_ER_IRQ_NUM[5];        /** @brief I2C error interrupt. */
extern const int32_t SAIx_IRQ_NUM[5];           /** @brief SAI global interrupt. */
extern const int32_t SDMMCx_IRQ_NUM[3];         /** @brief SDMMC global interrupt. */
extern const int32_t SPIx_IRQ_NUM[7];           /** @brief SPI global interrupt. */
extern const int32_t LPTIMx_IRQ_NUM[6];         /** @brief LPTIM global interrupt. */
extern const int32_t EXTIx_IRQ_NUM[5];          /** @brief EXTI global interrupt. */
extern const int32_t DFSDM1_FLTx_IRQ_NUM[4];    /** @brief DFSDM1 filter global interrupt. */
extern const int32_t TIMx_IRQ_NUM[18];          /** @brief TIM global interrupt. */
extern const int32_t USARTx_IRQ_NUM[7];         /** @brief USART global interrupt. */
extern const int32_t UARTx_IRQ_NUM[9];          /** @brief UART global interrupt. */
extern const int32_t TIMx_CC_IRQ_NUM[9];        /** @brief TIM capture/compare global interrupt. */
extern const int32_t DMAx_STRx_IRQ_NUM[3][8];   /** @brief DMA1 stream x interrupt. */
extern const int32_t FDCANx_ITx_IRQ_NUM[3][2];  /** @brief FDCAN1 interrupt x. */