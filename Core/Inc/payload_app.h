/**
 * @file payload_app.h
 * @brief Cooperative payload acquisition, logging, and command application.
 */

#ifndef PAYLOAD_APP_H
#define PAYLOAD_APP_H

#include "stm32g0xx_hal.h"

void payload_app_init(I2C_HandleTypeDef *i2c3,
                      SPI_HandleTypeDef *sd_spi,
                      SPI_HandleTypeDef *accel_spi,
                      UART_HandleTypeDef *radio_uart,
                      UART_HandleTypeDef *debug_uart);

void payload_app_process(void);

/**
 * @brief Enforce real-time output deadlines from the 1 ms system tick.
 *
 * This must stay non-blocking because it runs in interrupt context.
 */
void payload_app_1ms_tick(uint32_t now_ms);

#endif /* PAYLOAD_APP_H */
