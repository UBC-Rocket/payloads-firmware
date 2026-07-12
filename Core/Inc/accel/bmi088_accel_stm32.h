/**
 * @file bmi088_accel_stm32.h
 * @brief STM32 HAL SPI transport for the BMI088 accelerometer.
 */

#ifndef BMI088_ACCEL_STM32_H
#define BMI088_ACCEL_STM32_H

#include "accel/bmi088_accel.h"
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *chip_select_port;
    uint16_t chip_select_pin;
    uint32_t timeout_ms;
} bmi088_accel_stm32_bus_t;

/**
 * @brief Configure an STM32 SPI transport and bind it to a BMI088 instance.
 *
 * SPI must already be initialized for 8-bit, MSB-first mode 0 or mode 3 at
 * no more than 10 MHz, with software NSS, TI mode disabled, and CRC disabled.
 * Chip select is driven inactive before returning.
 *
 * This blocking adapter is not thread- or ISR-safe. Callers sharing the SPI
 * peripheral must serialize each complete driver operation.
 */
bmi088_accel_status_t bmi088_accel_stm32_bind(
    bmi088_accel_t *device,
    bmi088_accel_stm32_bus_t *bus,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *chip_select_port,
    uint16_t chip_select_pin,
    uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_ACCEL_STM32_H */
