/**
 * @file sd_spi_diskio.h
 * @brief FatFs disk I/O implementation for an SD card on STM32 SPI2.
 */

#ifndef SD_SPI_DISKIO_H
#define SD_SPI_DISKIO_H

#include "stm32g0xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SD_SPI_ERROR_NONE = 0,
    SD_SPI_ERROR_NOT_BOUND,
    SD_SPI_ERROR_HAL,
    SD_SPI_ERROR_TIMEOUT,
    SD_SPI_ERROR_PROTOCOL
} sd_spi_error_t;

bool sd_spi_diskio_bind(SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *chip_select_port,
                        uint16_t chip_select_pin);

/**
 * Reapply the card's safe startup SPI and chip-select configuration.
 */
bool sd_spi_configure_slow(void);
bool sd_spi_set_data_speed(void);

sd_spi_error_t sd_spi_last_error(void);

#endif /* SD_SPI_DISKIO_H */
