# This file is part of the titan project.
# Copyright (c) 2025 UW SARP
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
# 
# @file CMakeLists.txt
# @authors Aaron McBride
# @brief Top level cmake file for the titan project.

cmake_minimum_required(VERSION 3.19)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX "arm-none-eabi-")

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON) 

# proj dirs



set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(titan C ASM)

set(EXECUTABLE ${PROJECT_NAME}.elf)
enable_testing()

add_executable(${EXECUTABLE} 
  ${CMAKE_SOURCE_DIR}/main.c
  ${CMAKE_SOURCE_DIR}/internal/startup.c
  ${CMAKE_SOURCE_DIR}/internal/interrupt.c
  ${CMAKE_SOURCE_DIR}/internal/vtable.c
  ${CMAKE_SOURCE_DIR}/internal/mmio.c
  ${CMAKE_SOURCE_DIR}/peripheral/gpio.c
  ${CMAKE_SOURCE_DIR}/peripheral/watchdog.c
  ${CMAKE_SOURCE_DIR}/peripheral/pwm.c
  ${CMAKE_SOURCE_DIR}/internal/alloc.c
  ${CMAKE_SOURCE_DIR}/peripheral/uart.c
  ${CMAKE_SOURCE_DIR}/internal/dma.c
  ${CMAKE_SOURCE_DIR}/peripheral/spi.c
  ${CMAKE_SOURCE_DIR}/peripheral/actuator.c
  ${CMAKE_SOURCE_DIR}/peripheral/radio.c
 
)

# add_executable(test_allocator.elf
#   ${CMAKE_TEST_DIR}/test_alloc.c
#   ${CMAKE_SOURCE_DIR}/internal/alloc.c
# )

# add_test(NAME TEST_ALLOC COMMAND test_allocator.elf)


target_include_directories(${EXECUTABLE} PRIVATE
  ${CMAKE_SOURCE_DIR}
)

target_compile_options(${EXECUTABLE} PRIVATE
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -fdata-sections
  -ffreestanding
  -T devboard.ld
  # -O2
  -g3
)

target_link_options(${EXECUTABLE} PRIVATE
  -nostdlib
  -T ${CMAKE_SOURCE_DIR}/internal/linker.ld
  # -mtext-section-literals # added for linker issues
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -specs=nano.specs
  -lnosys
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--gc-sections
  -Xlinker
  -print-memory-usage
)
