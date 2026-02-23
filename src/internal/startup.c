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
 * @file internal/startup.c
 * @authors Aaron McBride
 * @brief Reset handlers and system initialization logic.
 */

#include "interrupt.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/************************************************************************************************
 * @section Program Initialization Routines
 ************************************************************************************************/

// Loads required sections of memory from flash into RAM.
static void _load_prog_mem(void) {
  typedef struct {
    const uint32_t *start; // Start of section to load
    const uint32_t *end;   // End of section to load
    uint32_t *dst;         // Start of location to load section to.
  } load_entry_t;
  // Extern symbols from linker script
  extern load_entry_t __load_table_start;
  extern load_entry_t __load_table_end;
  load_entry_t *cur_entry = &__load_table_start;
  while (cur_entry < &__load_table_end) {
    const uint32_t *src = cur_entry->start;
    uint32_t *dst = cur_entry->dst;
    while (src < cur_entry->end) {
      *dst++ = *src++;
    }
    cur_entry++;
  }
}

// Clears required sections of memory.
static void _clear_prog_mem(void) {
  typedef struct {
    uint32_t *start; // Start of section to clear
    uint32_t *end;   // End of section to clear
  } clear_entry_t;
  // Extern symbols from linker script
  extern clear_entry_t __clear_table_start;
  extern clear_entry_t __clear_table_end;
  clear_entry_t *cur_entry = &__clear_table_start;
  while (cur_entry < &__clear_table_end) {
    uint32_t *dst = cur_entry->start;
    while (dst < cur_entry->end) {
      *dst++ = 0U;
    }
    cur_entry++;
  }
}

// Invokes constructor functions
static void _invoke_init_fn(void) {
  typedef void (*init_fn_t)(void);
  extern init_fn_t __init_array_start;
  extern init_fn_t __init_array_end;
  init_fn_t *cur_fn = &__init_array_start;
  while (cur_fn < &__init_array_end) {
    (*cur_fn)();
    cur_fn++;
  }
}

// Invokes destructor functions
static void _invoke_fini_fn(void) {
  typedef void (*fini_fn_t)(void);
  extern fini_fn_t __fini_array_start;
  extern fini_fn_t __fini_array_end;
  fini_fn_t *cur_fn = &__fini_array_start;
  while (cur_fn < &__fini_array_end) {
    (*cur_fn)();
    cur_fn++;
  }
}

/************************************************************************************************
 * @section Reset Handlers
 ************************************************************************************************/

extern void _start(void);

// Reset handler for the CM7 core.
void cm7_reset_exc_handler(void) {
  _load_prog_mem();
  _clear_prog_mem();
  _invoke_init_fn();
  // TODO - Program startup/entry here
  _start();
  _invoke_fini_fn();
  while (true) {
    asm("wfi");
  }
}

// Reset handler for the CM4 core.
void cm4_reset_exc_handler(void) {
  while (true) {
    asm("wfi");
  }
}
