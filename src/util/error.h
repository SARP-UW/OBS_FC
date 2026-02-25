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
 * @file util/error.h
 * @authors Joshua Beard & Aaron McBride & Charles Faisandier
 * @brief Error handling utilities public interface.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**************************************************************************************************
 * @section Constants
 **************************************************************************************************/

// The maximum number of errors to log in the error list.
// This defines the size of the static append-only error log.
#define tal_max_err_cnt 10

/**************************************************************************************************
 * @section Types
 **************************************************************************************************/

/**
 * @brief A struct containing rich context for a single error.
 *
 * @note This data is stored in read-only memory by the tal_raise macro.
 */
typedef struct
{
    const char *msg;
    const char *func;
    const char *file;
    int32_t line;
    bool is_error;
} tal_err_t;

/**
 * @brief The container for managing errors within a module.
 *
 * @note err_count is the total number of errors that have occurred.
 * err_list is an append-only log of the first tal_max_err_cnt errors.
 */
typedef struct
{
    const tal_err_t *err_list[tal_max_err_cnt];
    int32_t err_count;
} tal_flag_t;

/**************************************************************************************************
 * @section Macros and Functions
 **************************************************************************************************/

/**
 * @brief Logs a new error to the given tal_flag_t.
 *
 * @param flag The error flag to log the error to.
 * @param msg The human-readable error message.
 */
#define tal_raise(flag, msg)                                                                             \
do {                                                                                                     \
    static const tal_err_t err_##__LINE__ = {msg, __PRETTY_FUNCTION__, __FILE__, __LINE__, true}; \
    tal_raise_impl__(flag, &err_##__LINE__);                                                             \
} while (0)
//  #define tal_raise(flag, msg)                                                                                                                 \
//      static const tal_err_t err_##__LINE__ = {.msg = msg, .func = __PRETTY_FUNCTION__, .file = __FILE__, .line = __LINE__, .is_error = true}; \
//      tal_raise_impl__(flag, &err_##__LINE__);

/**
 * @brief Creates a new error and returns it.
 * This is used for critical errors, which should be logged and transmitted. 
 *
 * @param flag The error flag to log the error to.
 * @param msg The human-readable error message.
 */
#define tal_raise_ret(flag, msg, ret)                                                                                                        \
    static const tal_err_t err_##__LINE__ = {.msg = msg, .func = __PRETTY_FUNCTION__, .file = __FILE__, .line = __LINE__, .is_error = true}; \
    return err_##__LINE__;  

/**
 * @brief Checks if the given error flag indicates an error.
 *
 * @param err The error to check.
 * @return true if the error indicates an error, false otherwise.
 */
bool tal_is_err(const tal_err_t *err);

// Private implementation function.
void tal_raise_impl__(tal_flag_t* flag, const tal_err_t* error);