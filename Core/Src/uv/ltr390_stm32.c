#include "uv/ltr390_stm32.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

static bool stm32_read(void *context,
                       uint8_t first_register,
                       uint8_t *data,
                       size_t length)
{
    ltr390_stm32_bus_t *bus = (ltr390_stm32_bus_t *)context;
    if (bus == NULL || data == NULL || length == 0U || length > UINT16_MAX) {
        return false;
    }

    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(bus->i2c,
                         (uint16_t)(LTR390_I2C_ADDRESS << 1U),
                         (uint16_t)first_register,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         (uint16_t)length,
                         bus->timeout_ms);
    return status == HAL_OK;
}

static bool stm32_write(void *context,
                        uint8_t register_address,
                        uint8_t value)
{
    ltr390_stm32_bus_t *bus = (ltr390_stm32_bus_t *)context;
    if (bus == NULL) {
        return false;
    }

    return HAL_I2C_Mem_Write(bus->i2c,
                             (uint16_t)(LTR390_I2C_ADDRESS << 1U),
                             (uint16_t)register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             bus->timeout_ms) == HAL_OK;
}

static void stm32_delay_ms(void *context, uint32_t milliseconds)
{
    (void)context;
    HAL_Delay(milliseconds);
}

ltr390_status_t ltr390_stm32_bind(ltr390_t *device,
                                  ltr390_stm32_bus_t *bus,
                                  I2C_HandleTypeDef *i2c,
                                  uint32_t timeout_ms)
{
    if (device == NULL || bus == NULL || i2c == NULL || timeout_ms == 0U) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (i2c->Init.AddressingMode != I2C_ADDRESSINGMODE_7BIT) {
        return LTR390_ERROR_CONFIGURATION;
    }

    bus->i2c = i2c;
    bus->timeout_ms = timeout_ms;

    const ltr390_transport_t transport = {
        .read = stm32_read,
        .write = stm32_write,
        .delay_ms = stm32_delay_ms,
        .context = bus,
    };
    return ltr390_bind(device, &transport);
}
