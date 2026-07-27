# Payloads firmware

Firmware for UBC Rocket's Cloudburst payload on the STM32G0B1CCT6. It reads from the accel and the UV sensor(s; should be three, but currently only has 1). UV values are set using pwm in main.c, we should probably figure out exactly what values we want for that at some point. LORA might work, might not, who even knows. The rest of this readme is AI generated :smile:


## Build

The receiver must use the same raw-LoRa profile as the transmitter. CMake
requires those values rather than silently building a receiver with guessed RF
settings. For example:

```sh
cmake --preset Debug \
  -DRN2483_RADIO_FREQ_HZ=868000000 \
  -DRN2483_RADIO_SF=7 \
  -DRN2483_RADIO_BW_KHZ=125 \
  -DRN2483_RADIO_CR=5 \
  -DRN2483_RADIO_SYNC_WORD=0x12
cmake --build --preset Debug
```

Replace the example values with the transmitter's frequency, spreading factor,
bandwidth, coding-rate denominator, and sync byte. The ELF is written to
`build/Debug/Payloads.elf`.

## Runtime behavior

- The BMI088 accelerometer on SPI2 runs at ±6 g, 100 Hz, and normal bandwidth.
  Its 1024-byte FIFO is drained every 20 ms so short SD-card stalls do not lose
  acquisition timing.
- The single LTR390 is detected on the configured 100 kHz I2C1, I2C2, and I2C3
  headers in that order. Once found, only that instance is sampled. It runs in
  UV mode at 20-bit resolution, a 500 ms measurement period, and 18x gain.
- PA1 is `UVLED_CTRL`/TIM2 channel 2 in the `.ioc`. A runtime override in the
  protected user-code section runs it at 1 kHz with a 10% active-high LED-on
  interval on the shared UV-emitter control net.
- A preformatted FAT16 or FAT32 SD card on SPI1 is mounted without formatting.
  Each boot creates the first free `LOG0000.CSV` through `LOG9999.CSV`, buffers
  records in RAM, writes in batches, and synchronizes once per second. Failed
  cards are retried every five seconds while acquisition continues.
- USART2 uses the `.ioc` value of 115200 baud. Startup sends the RN2483 break
  and `0x55` auto-baud sequence, pauses the MAC, applies and reads back the
  required raw-LoRa profile, then enters continuous receive mode.
- `PUMP_ON` sets PD2 high. `PUMP_OFF` sets PD2 low. These are the only accepted
  radio payloads; malformed or unknown packets leave the pump unchanged. The
  pump starts low and is also forced low by `Error_Handler`.

The generated SPI1 setup in the `.ioc` is four-bit with hardware NSS. SD cards
require eight-bit SPI mode 0 with software-controlled chip select, so the SD
transport reinitializes SPI1 after `MX_SPI1_Init`: 250 kHz for card startup and
8 MHz afterward, with PA4 driven as the card chip select. The peripheral and
pin assignment still come directly from the `.ioc`.

## Log format

`time_ms` is monotonic milliseconds since MCU startup; there is no real-time
clock in the `.ioc`. A row is emitted for every 100 Hz accelerometer sample:

```text
time_ms,accel_x_raw,accel_y_raw,accel_z_raw,uv_raw,accel_valid,uv_valid,uv_new,sensor_error_mask,pump_on,accel_fifo_skipped_total,log_dropped_total
```

`uv_valid` identifies whether the cached reading is valid and `uv_new`
identifies the row that first records a new conversion. `uv_i2c_bus_number` is
1, 2, or 3 for the detected connector and zero while the sensor is unavailable.
Error and dropped-record counters make degraded operation visible instead of
silently producing plausible-looking data.

## Host tests

```sh
cmake -S tests -B build/host-tests -G Ninja
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

The tests cover BMI088 FIFO parsing and SPI framing, the LTR390 driver,
fragmented RN2483 replies and pump commands, buffered FatFs logging and
recovery, and byte-level SDHC initialization/read/write behavior.

Useful debugger globals include `accel_last_status`, `accel_latest_sample`,
`accel_sample_count`, `accel_error_count`, `accel_fifo_skipped_total`,
`uv_latest_sample`, `uv_last_status`, `uv_sample_count`, `uv_error_count`,
`uv_i2c_bus_number`,
`payload_sd_status`, `payload_log_dropped_count`, `payload_radio_ready`, and
`payload_pump_on`.

Protocol references are the local copies in `Docs/` and the
[Microchip RN2483 command reference](https://ww1.microchip.com/downloads/en/DeviceDoc/RN2483-LoRa-Technology-Module-Command-Reference-User-Guide-DS40001784G.pdf).
