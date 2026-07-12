# Payloads firmware

This is the firmware for UBC Rocket's Cloudburst payload, to be used in Launch Canada 2026. The architecture is fairly simple: we continuously read accelerometer and UV data, write the data to an SD card, while monitoring for commands from the LoRa radio module. In all honesty I don't know exactly what we're meant to be testing, but that means that you don't have to either to understand this codebase.   

STM32G0B1CCT6 firmware generated with STM32CubeMX and built with CMake, Ninja,
and the Arm GNU toolchain (and codex).

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
```

The firmware image is written to `build/Debug/Payloads.elf`.

## Host tests

The bus-independent BMI088 and LTR390 register drivers can be tested without
target hardware:

```sh
cmake -S tests -B build/host-tests
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

## BMI088 accelerometer

The accelerometer uses SPI2 in mode 0 with 8-bit frames at 8 MHz. The STM32
adapter handles the extra dummy response byte required by BMI088 accelerometer
SPI reads. `main.c` initializes the device with the datasheet reset defaults
(±6 g, 100 Hz, normal bandwidth) and polls the data-ready flag.

The current polling loop is intended for that 100 Hz default. Higher ODRs need
an interrupt/FIFO acquisition path to avoid dropping samples, and any future
device sharing SPI2 must serialize access around complete driver operations.

These globals are useful during board bring-up in a debugger:

- `accel_last_status`
- `accel_latest_sample`
- `accel_sample_count`
- `accel_error_count`

The software path is covered by host tests and the cross-compiled firmware
build. Hardware validation should confirm chip ID `0x1E` and approximately
1 g on the gravity-aligned axis while the board is stationary.

Protocol details are defined in the
[Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf).

## LTR-390UV-01 light/UV sensor

The LTR390 driver supports both ambient-light and UV modes over I2C at the
device's fixed 7-bit address `0x53`. It checks the part ID, performs a software
reset, configures resolution/rate/gain, verifies register readback, polls the
new-data flag, and returns coherent 20-bit readings with lux or UVI conversion.

`ltr390_default_uvs_config` selects UV mode, 20-bit resolution, a 500 ms
measurement period, and 18x gain. This matches the conditions for the
datasheet's 2300-count/UVI sensitivity. Use a window factor of `1.0f` for a
clear/no-window setup and calibrate it for tinted optical windows.

The board has one LTR390 on each 100 kHz I2C controller: I2C1 on PB8/PB7
(SCL/SDA), I2C2 on PB10/PB11, and I2C3 on PB3/PB4. `main.c` initializes all
three sensors and polls their new-data flags every 10 ms. The latest readings
and diagnostics are available as indexed debugger globals:

- `uv_latest_sample[0..2]`
- `uv_last_status[0..2]`
- `uv_sample_count[0..2]`
- `uv_error_count[0..2]`

The indices map to I2C1, I2C2, and I2C3 respectively. The adapter is blocking
and shared-bus calls must be serialized. Protocol and conversion details are in
`Docs/LTR-390UV_Final_ DS_V1 1.pdf`.
