
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
 * @file peripheral/watchdog.h
 * @authors Joshua Beard
 * @brief Driver for the watchdog timer
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "internal/mmio.h"

#if defined(__cplusplus)
extern "C" {
#endif

int32_t START_KEY = 0xCCCC;
int32_t REG_ACCESS = 0x5555;
int32_t RESET_RLR = 0xAAAA;
int32_t PRESCALER_DIVIDER = 1; // doc page 2051, can be other values to increase
                               // frequency of countdown clock

int32_t RESET_COUNT = 4000; // any 12 bit number

/**
 * Clock is 32 kHz (32,000 clk per second).  Lets say we want a one second delay
 * before reset
 *
 * 32,000 c/second * (1/8) {PRESCALER_DIVIDER = 1} = 4000 watchdog time
 * decrements Set RESET_COUNT = 4000 to make this exactly one second
 *
 * Note that with PRESCALER_DIVIDER = 0 it is not possible to make one second
 * delay, because 8000 > 4095
 */

bool ti_IWDG_enable() {
  WRITE_FIELD(RCC_APB3ENR, RCC_APB3ENR_WWDG1EN, 1);

  WRITE_FIELD(IWDGx_KR[1], IWDGx_KR_KEY, START_KEY);

  WRITE_FIELD(IWDGx_KR[1], IWDGx_KR_KEY, REG_ACCESS);

  WRITE_FIELD(IWDGx_PR[1], IWDGx_PR_PR, PRESCALER_DIVIDER);

  WRITE_FIELD(IWDGx_RLR[1], IWDGx_RLR_RL, RESET_COUNT);
  // Wait for SR to be zeroed? says in doc but idk

  return 1;
}

void ti_IWDG_reset_timer() {
  WRITE_FIELD(IWDGx_KR[1], IWDGx_KR_KEY, RESET_RLR);
}

#if defined(__cplusplus)
}
#endif
