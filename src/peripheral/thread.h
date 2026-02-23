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
 * @file peripheral/thread.h
 * @authors Aaron McBride
 * @brief Thread management and control facilities.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

enum ti_thread_state_t {
  TI_THREAD_STATE_EXCLUSIVE,
  TI_THREAD_STATE_CRITICAL,
  TI_THREAD_STATE_RUNNING,
  TI_THREAD_STATE_READY,
  TI_THREAD_STATE_SUSPENDED,
  TI_THREAD_STATE_STOPPED,
  TI_THREAD_STATE_OVERFLOW,
};

struct ti_thread_t {
  const int32_t id;
  const void* const handle;
};

extern const int32_t TI_MAX_THREAD_PRIORITY;
extern const int32_t TI_MIN_THREAD_PRIORITY;

#define TI_THREAD_MEM_SIZE(stack_size) 0

struct ti_thread_t ti_create_thread(void* mem, void (*entry_fn)(void*), void* arg, int32_t stack_size, int32_t priority);

void ti_destroy_thread(struct ti_thread_t thread);

void ti_suspend_thread(struct ti_thread_t thread);

void ti_resume_thread(struct ti_thread_t thread);

void ti_enter_critical(void);

void ti_exit_critical(void);

void ti_enter_exclusive(void);

void ti_exit_exclusive(void);

void ti_exit(void);

void ti_yield(void);

void ti_set_thread_priority(struct ti_thread_t thread, int32_t priority);

int32_t ti_get_thread_priority(struct ti_thread_t thread);

enum thread_state_t ti_get_thread_state(struct ti_thread_t thread);

void* ti_get_thread_arg(struct ti_thread_t thread);

int32_t ti_get_thread_stack_size(struct ti_thread_t thread);

int32_t ti_get_thread_stack_usage(struct ti_thread_t thread);

bool ti_is_valid_thread(struct ti_thread_t thread);

struct ti_thread_t ti_get_this_thread(void);

bool ti_is_thread_equal(struct ti_thread_t thread_1, struct ti_thread_t thread_2);