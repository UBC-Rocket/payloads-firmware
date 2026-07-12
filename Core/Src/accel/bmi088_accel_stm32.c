#include "accel/bmi088_accel_stm32.h"

#include <string.h>

#define BMI088_ACCEL_SPI_READ_BIT 0x80U
#define BMI088_ACCEL_SPI_MAX_READ_LENGTH 6U
#define BMI088_ACCEL_SPI_MAX_CLOCK_HZ 10000000U

static uint32_t spi_prescaler_divisor(uint32_t prescaler)
{
    switch (prescaler) {
    case SPI_BAUDRATEPRESCALER_2:
        return 2U;
    case SPI_BAUDRATEPRESCALER_4:
        return 4U;
    case SPI_BAUDRATEPRESCALER_8:
        return 8U;
    case SPI_BAUDRATEPRESCALER_16:
        return 16U;
    case SPI_BAUDRATEPRESCALER_32:
        return 32U;
    case SPI_BAUDRATEPRESCALER_64:
        return 64U;
    case SPI_BAUDRATEPRESCALER_128:
        return 128U;
    case SPI_BAUDRATEPRESCALER_256:
        return 256U;
    default:
        return 0U;
    }
}

static bool stm32_read(void *context,
                       uint8_t first_register,
                       uint8_t *data,
                       size_t length)
{
    bmi088_accel_stm32_bus_t *bus =
        (bmi088_accel_stm32_bus_t *)context;

    if (bus == NULL || data == NULL || length == 0U ||
        length > BMI088_ACCEL_SPI_MAX_READ_LENGTH) {
        return false;
    }

    /* BMI088 accelerometer reads return one protocol dummy byte after the
       address. The requested register bytes start at rx[2]. */
    uint8_t tx[BMI088_ACCEL_SPI_MAX_READ_LENGTH + 2U] = {0U};
    uint8_t rx[BMI088_ACCEL_SPI_MAX_READ_LENGTH + 2U] = {0U};
    tx[0] = (uint8_t)(first_register | BMI088_ACCEL_SPI_READ_BIT);

    HAL_GPIO_WritePin(bus->chip_select_port,
                      bus->chip_select_pin,
                      GPIO_PIN_RESET);
    const HAL_StatusTypeDef hal_status =
        HAL_SPI_TransmitReceive(bus->spi,
                               tx,
                               rx,
                               (uint16_t)(length + 2U),
                               bus->timeout_ms);
    HAL_GPIO_WritePin(bus->chip_select_port,
                      bus->chip_select_pin,
                      GPIO_PIN_SET);

    if (hal_status != HAL_OK) {
        return false;
    }

    memcpy(data, &rx[2], length);
    return true;
}

static bool stm32_write(void *context,
                        uint8_t register_address,
                        uint8_t value)
{
    bmi088_accel_stm32_bus_t *bus =
        (bmi088_accel_stm32_bus_t *)context;
    if (bus == NULL) {
        return false;
    }

    const uint8_t tx[2] = {
        (uint8_t)(register_address & (uint8_t)~BMI088_ACCEL_SPI_READ_BIT),
        value,
    };

    HAL_GPIO_WritePin(bus->chip_select_port,
                      bus->chip_select_pin,
                      GPIO_PIN_RESET);
    const HAL_StatusTypeDef hal_status =
        HAL_SPI_Transmit(bus->spi,
                         (const uint8_t *)tx,
                         (uint16_t)sizeof(tx),
                         bus->timeout_ms);
    HAL_GPIO_WritePin(bus->chip_select_port,
                      bus->chip_select_pin,
                      GPIO_PIN_SET);

    return hal_status == HAL_OK;
}

static void stm32_delay_ms(void *context, uint32_t milliseconds)
{
    (void)context;
    HAL_Delay(milliseconds);
}

bmi088_accel_status_t bmi088_accel_stm32_bind(
    bmi088_accel_t *device,
    bmi088_accel_stm32_bus_t *bus,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *chip_select_port,
    uint16_t chip_select_pin,
    uint32_t timeout_ms)
{
    if (device == NULL || bus == NULL || spi == NULL ||
        chip_select_port == NULL || chip_select_pin == 0U || timeout_ms == 0U) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }

    const bool mode_0 = spi->Init.CLKPolarity == SPI_POLARITY_LOW &&
                        spi->Init.CLKPhase == SPI_PHASE_1EDGE;
    const bool mode_3 = spi->Init.CLKPolarity == SPI_POLARITY_HIGH &&
                        spi->Init.CLKPhase == SPI_PHASE_2EDGE;
    const uint32_t prescaler_divisor =
        spi_prescaler_divisor(spi->Init.BaudRatePrescaler);
    const uint32_t spi_clock_hz = prescaler_divisor == 0U
        ? 0U
        : HAL_RCC_GetPCLK1Freq() / prescaler_divisor;
    if (spi->Init.Mode != SPI_MODE_MASTER ||
        spi->Init.Direction != SPI_DIRECTION_2LINES ||
        spi->Init.DataSize != SPI_DATASIZE_8BIT ||
        spi->Init.NSS != SPI_NSS_SOFT ||
        spi->Init.FirstBit != SPI_FIRSTBIT_MSB ||
        spi->Init.TIMode != SPI_TIMODE_DISABLE ||
        spi->Init.CRCCalculation != SPI_CRCCALCULATION_DISABLE ||
        (!mode_0 && !mode_3) || spi_clock_hz == 0U ||
        spi_clock_hz > BMI088_ACCEL_SPI_MAX_CLOCK_HZ) {
        return BMI088_ACCEL_ERROR_CONFIGURATION;
    }

    bus->spi = spi;
    bus->chip_select_port = chip_select_port;
    bus->chip_select_pin = chip_select_pin;
    bus->timeout_ms = timeout_ms;

    HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_SET);

    const bmi088_accel_transport_t transport = {
        .read = stm32_read,
        .write = stm32_write,
        .delay_ms = stm32_delay_ms,
        .context = bus,
    };

    return bmi088_accel_bind(device, &transport);
}
