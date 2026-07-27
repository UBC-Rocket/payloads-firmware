cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Payloads)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

set(RN2483_RADIO_FREQ_HZ "" CACHE STRING
    "RN2483 raw-LoRa frequency in Hz")
set(RN2483_RADIO_SF "" CACHE STRING
    "RN2483 spreading factor (7-12)")
set(RN2483_RADIO_BW_KHZ "" CACHE STRING
    "RN2483 bandwidth in kHz (125, 250, or 500)")
set(RN2483_RADIO_CR "" CACHE STRING
    "RN2483 coding-rate denominator (5-8 for 4/5 through 4/8)")
set(RN2483_RADIO_SYNC_WORD "" CACHE STRING
    "RN2483 one-byte hexadecimal or decimal sync word")

foreach(required_radio_setting
        RN2483_RADIO_FREQ_HZ
        RN2483_RADIO_SF
        RN2483_RADIO_BW_KHZ
        RN2483_RADIO_CR
        RN2483_RADIO_SYNC_WORD)
    if("${${required_radio_setting}}" STREQUAL "")
        message(FATAL_ERROR
            "${required_radio_setting} is required for flashable firmware")
    endif()
endforeach()

if(NOT ((RN2483_RADIO_FREQ_HZ GREATER_EQUAL 433050000 AND
         RN2483_RADIO_FREQ_HZ LESS_EQUAL 434790000) OR
        (RN2483_RADIO_FREQ_HZ GREATER_EQUAL 863000000 AND
         RN2483_RADIO_FREQ_HZ LESS_EQUAL 870000000)))
    message(FATAL_ERROR "RN2483_RADIO_FREQ_HZ is outside RN2483 bands")
endif()
if(RN2483_RADIO_SF LESS 7 OR RN2483_RADIO_SF GREATER 12)
    message(FATAL_ERROR "RN2483_RADIO_SF must be from 7 through 12")
endif()
if(NOT (RN2483_RADIO_BW_KHZ EQUAL 125 OR
        RN2483_RADIO_BW_KHZ EQUAL 250 OR
        RN2483_RADIO_BW_KHZ EQUAL 500))
    message(FATAL_ERROR "RN2483_RADIO_BW_KHZ must be 125, 250, or 500")
endif()
if(RN2483_RADIO_CR LESS 5 OR RN2483_RADIO_CR GREATER 8)
    message(FATAL_ERROR "RN2483_RADIO_CR must be from 5 through 8")
endif()
math(EXPR RN2483_RADIO_SYNC_WORD_VALUE "${RN2483_RADIO_SYNC_WORD}")
if(RN2483_RADIO_SYNC_WORD_VALUE LESS 0 OR
   RN2483_RADIO_SYNC_WORD_VALUE GREATER 255)
    message(FATAL_ERROR "RN2483_RADIO_SYNC_WORD must fit in one byte")
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/payload_app.c
    Core/Src/rn2483.c
    Core/Src/rn2483_stm32.c
    Core/Src/accel/bmi088_accel.c
    Core/Src/accel/bmi088_accel_stm32.c
    Core/Src/storage/payload_log.c
    Core/Src/storage/sd_logger.c
    Core/Src/storage/sd_spi_diskio.c
    Core/Src/uv/ltr390.c
    Core/Src/uv/ltr390_stm32.c
    Middlewares/Third_Party/FatFs/src/ff.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/Third_Party/FatFs/src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    RN2483_RADIO_FREQ_HZ=${RN2483_RADIO_FREQ_HZ}
    RN2483_RADIO_SF=${RN2483_RADIO_SF}
    RN2483_RADIO_BW_KHZ=${RN2483_RADIO_BW_KHZ}
    RN2483_RADIO_CR=${RN2483_RADIO_CR}
    RN2483_RADIO_SYNC_WORD=${RN2483_RADIO_SYNC_WORD_VALUE}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
