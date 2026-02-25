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
 * @file internal/delay.h
 * @authors Jude Merritt
 * @brief SysTick delay utility
 */

#pragma once
#include <stdint.h>
#include "include/errc.h"

/**
 * @brief Initializes the systick timer.
 */
void systick_init();

/**
 * @brief Provides an exact time delay using the systick timer.
 *
 * @param delay The duration of the delay in milleseconds (ms)
 */
void systick_delay(uint32_t delay);