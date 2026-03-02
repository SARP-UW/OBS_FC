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

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(titan C ASM)

enable_testing()

set(COMMON_SOURCES
  ${CMAKE_SOURCE_DIR}/src/internal/startup.c
  ${CMAKE_SOURCE_DIR}/src/internal/interrupt.c
  ${CMAKE_SOURCE_DIR}/src/internal/vtable.c
  ${CMAKE_SOURCE_DIR}/src/internal/mmio.c
  ${CMAKE_SOURCE_DIR}/src/internal/alloc.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/gpio.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/watchdog.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/pwm.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/uart.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/dma.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/spi.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/qspi.c
  ${CMAKE_SOURCE_DIR}/src/peripheral/systick.c
  ${CMAKE_SOURCE_DIR}/src/devices/actuator.c
  ${CMAKE_SOURCE_DIR}/src/devices/radio.c
  ${CMAKE_SOURCE_DIR}/src/devices/barometer.c
)

function(add_firmware_target target_name entry_file)
  add_executable(${target_name}.elf
    ${entry_file}
    ${COMMON_SOURCES}
  )

  target_include_directories(${target_name}.elf PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
  )

  target_compile_options(${target_name}.elf PRIVATE
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

  target_link_options(${target_name}.elf PRIVATE
    -nostdlib
    -T ${CMAKE_SOURCE_DIR}/src/internal/linker.ld
    # -mtext-section-literals # added for linker issues
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -lnosys
    -Wl,-Map=${target_name}.map,--cref
    -Wl,--gc-sections
    -Xlinker
    -print-memory-usage
  )
endfunction()

add_firmware_target(titan ${CMAKE_SOURCE_DIR}/src/main.c)
add_firmware_target(test_pwm ${CMAKE_SOURCE_DIR}/test/test_pwm.c)
add_firmware_target(test_spi ${CMAKE_SOURCE_DIR}/test/test_spi.c)
add_firmware_target(test_usart ${CMAKE_SOURCE_DIR}/test/test_usart.c)
add_firmware_target(test_oscilloscope ${CMAKE_SOURCE_DIR}/test/test_oscilloscope.c)

# Native host unit test: test_alloc (compiled with system gcc, not the ARM cross-compiler)
add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/test_alloc
  COMMAND gcc -std=c18 -Wall -Wextra
    ${CMAKE_SOURCE_DIR}/src/internal/alloc.c
    ${CMAKE_SOURCE_DIR}/test/test_alloc.c
    -o ${CMAKE_BINARY_DIR}/test_alloc
  DEPENDS
    ${CMAKE_SOURCE_DIR}/src/internal/alloc.c
    ${CMAKE_SOURCE_DIR}/test/test_alloc.c
    ${CMAKE_SOURCE_DIR}/src/internal/alloc.h
  COMMENT "Building native host test: test_alloc"
)
add_custom_target(test_alloc_target ALL DEPENDS ${CMAKE_BINARY_DIR}/test_alloc)
add_test(NAME test_alloc COMMAND ${CMAKE_BINARY_DIR}/test_alloc)

# Doxygen documentation (optional, run with: cmake --build build --target docs)
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
  add_custom_target(docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating Doxygen documentation"
  )
endif()
