/**
* This file is part of the Titan Flight Computer Project
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
 * @file internal/systick.c
 * @authors Jude Merritt
 * @brief SysTick delay utility
 */

#include <stdint.h>
#include "include/mmio.h"
#include "include/errc.h"
#include "myWork/systick.h"

// ((480 * 1,000,000) * 0.001 - 1)
// 1ms countdown
#define DEFAULT_RELOAD_VAL 0x752FF
#define CLOCK_FREQ 480000000

void systick_init() {
    //Program reload value
    WRITE_FIELD(STK_RVR, STK_RVR_RELOAD, DEFAULT_RELOAD_VAL);

    //Set clock source
    SET_FIELD(STK_CSR, STK_CSR_CLKSOURCE);

    //Enable SysTick
    SET_FIELD(STK_CSR, STK_CSR_ENABLE);
}

void systick_delay(uint32_t delay) {
    if (delay == 0) return;

    //Clear current value
    WRITE_FIELD(STK_CVR, STK_CVR_CURRENT, 0U);

    //Run delay loop
    for (int i = 0; i < delay; i++) {
        while (IS_FIELD_CLR(STK_CSR, STK_CSR_COUNTFLAG)){
            asm("NOP");
        }
    }
}