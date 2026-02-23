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
 * @file peripheral/mutex.h
 * @authors Aaron McBride
 * @brief Mutex synchronization primitives.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "thread.h"

struct ti_mutex_t {
  const int32_t id;
  const void* const handle;
};

#define TI_MUTEX_MEM_SIZE 0

struct ti_mutex_t ti_create_mutex(void* mem);

void ti_destroy_mutex(struct ti_mutex_t mutex);

bool ti_acquire_mutex(struct ti_mutex_t mutex, int64_t timeout);

bool ti_release_mutex(struct ti_mutex_t mutex, int64_t timeout);

bool ti_is_mutex_locked(struct ti_mutex_t mutex);

struct ti_thread_t ti_get_mutex_owner(struct ti_mutex_t mutex);

bool ti_is_valid_mutex(struct ti_mutex_t mutex);

bool ti_is_mutex_equal(struct ti_mutex_t mutex1, struct ti_mutex_t mutex2);