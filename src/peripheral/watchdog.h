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

#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Enables the Independent Watchdog
 * Doc Page 2047 for an overview
 * @return if enable succeeded.
 */
bool ti_IWDG_enable();

/**
 * Resets the countdown so the board doesn't reset
 */
void ti_IWDG_reset_timer();

#if defined(__cplusplus)
}
#endif
