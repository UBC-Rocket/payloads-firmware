# Payloads firmware

STM32G0B1CCT6 firmware generated with STM32CubeMX and built with CMake, Ninja,
and the Arm GNU toolchain (and codex).

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
```

The firmware image is written to `build/Debug/Payloads.elf`.

## Host tests

The bus-independent BMI088 register driver can be tested without target
hardware:

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
