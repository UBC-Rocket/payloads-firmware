/**
 * @file ltr390_stm32.h
 * @brief STM32 HAL I2C transport for the LTR-390UV-01.
 */

#ifndef LTR390_STM32_H
#define LTR390_STM32_H

#include "stm32g0xx_hal.h"
#include "uv/ltr390.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint32_t timeout_ms;
} ltr390_stm32_bus_t;

/**
 * Bind a blocking STM32 HAL I2C transport to an LTR390 device.
 *
 * The caller selects the physical bus by passing hi2c1, hi2c2, or hi2c3.
 * Complete driver calls must be serialized if that bus is shared. Do not call
 * this blocking adapter from interrupt context.
 */
ltr390_status_t ltr390_stm32_bind(ltr390_t *device,
                                  ltr390_stm32_bus_t *bus,
                                  I2C_HandleTypeDef *i2c,
                                  uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* LTR390_STM32_H */
