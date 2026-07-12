#ifndef TEST_STM32G0XX_HAL_H
#define TEST_STM32G0XX_HAL_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct {
    uint32_t Mode;
    uint32_t Direction;
    uint32_t DataSize;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t NSS;
    uint32_t BaudRatePrescaler;
    uint32_t FirstBit;
    uint32_t TIMode;
    uint32_t CRCCalculation;
} SPI_InitTypeDef;

typedef struct {
    SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct {
    uint32_t AddressingMode;
} I2C_InitTypeDef;

typedef struct {
    I2C_InitTypeDef Init;
} I2C_HandleTypeDef;

typedef struct {
    uint32_t unused;
} GPIO_TypeDef;

#define SPI_MODE_MASTER 0x01U
#define SPI_DIRECTION_2LINES 0x02U
#define SPI_DATASIZE_8BIT 0x08U
#define SPI_POLARITY_LOW 0x00U
#define SPI_POLARITY_HIGH 0x01U
#define SPI_PHASE_1EDGE 0x00U
#define SPI_PHASE_2EDGE 0x01U
#define SPI_NSS_SOFT 0x01U
#define SPI_FIRSTBIT_MSB 0x00U
#define SPI_TIMODE_DISABLE 0x00U
#define SPI_CRCCALCULATION_DISABLE 0x00U

#define SPI_BAUDRATEPRESCALER_2 2U
#define SPI_BAUDRATEPRESCALER_4 4U
#define SPI_BAUDRATEPRESCALER_8 8U
#define SPI_BAUDRATEPRESCALER_16 16U
#define SPI_BAUDRATEPRESCALER_32 32U
#define SPI_BAUDRATEPRESCALER_64 64U
#define SPI_BAUDRATEPRESCALER_128 128U
#define SPI_BAUDRATEPRESCALER_256 256U

#define I2C_ADDRESSINGMODE_7BIT 0x01U
#define I2C_ADDRESSINGMODE_10BIT 0x02U
#define I2C_MEMADD_SIZE_8BIT 0x01U

void HAL_GPIO_WritePin(GPIO_TypeDef *port,
                       uint16_t pin,
                       GPIO_PinState state);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout);

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
                                          const uint8_t *tx_data,
                                          uint8_t *rx_data,
                                          uint16_t size,
                                          uint32_t timeout);

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c,
                                   uint16_t device_address,
                                   uint16_t memory_address,
                                   uint16_t memory_address_size,
                                   uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout);

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c,
                                    uint16_t device_address,
                                    uint16_t memory_address,
                                    uint16_t memory_address_size,
                                    uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout);

void HAL_Delay(uint32_t milliseconds);
uint32_t HAL_RCC_GetPCLK1Freq(void);

#endif /* TEST_STM32G0XX_HAL_H */
