cmake_minimum_required(VERSION 3.22)

project(payloads_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(test_bmi088_accel
    test_bmi088_accel.c
    ../Core/Src/accel/bmi088_accel.c
    ../Core/Src/accel/bmi088_accel_stm32.c
)

target_include_directories(test_bmi088_accel PRIVATE
    stubs
    ../Core/Inc
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_bmi088_accel PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
    )
endif()

add_test(NAME bmi088_accel COMMAND test_bmi088_accel)
