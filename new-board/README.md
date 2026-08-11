# New-board range bridge

This is the `range-test-rx` raw-LoRa bridge port for the custom
STM32F103C8T6 board. It keeps the original 433.575 MHz,
SF12/BW125/CR4/5, CRC-on, sync-`0x34` radio profile and accepts
`PUMP_ON`, `PUMP_OFF`, `BUMP 1-3600`, `LED_ON`, `LED_OFF`, and `PING`.

## Wiring

Both UARTs use 3.3 V logic and require a common ground.

- Host command/debug UART: USART1 at 115200 baud
  - PA9 / USART1_TX -> USB-to-serial RX
  - PA10 / USART1_RX <- USB-to-serial TX
- RN2483 UART: USART2 at 57600 baud
  - PA2 / USART2_TX -> RN2483 RX
  - PA3 / USART2_RX <- RN2483 TX

The custom board configuration has no Nucleo LD2 or B1, so the port reports
activity through USART1 instead of toggling a status LED.

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
```

The ELF, Intel HEX, and binary images are written to `build/Debug/`. The GNU
linker and IAR configurations both limit the image to the STM32F103C8's 64 KiB
flash and 20 KiB SRAM.
