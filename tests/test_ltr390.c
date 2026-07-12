#include "uv/ltr390.h"
#include "uv/ltr390_stm32.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define REG_MAIN_CTRL 0x00U
#define REG_MEAS_RATE 0x04U
#define REG_GAIN 0x05U
#define REG_PART_ID 0x06U
#define REG_MAIN_STATUS 0x07U
#define REG_ALS_DATA 0x0DU
#define REG_UVS_DATA 0x10U

typedef struct {
    uint8_t registers[0x27];
    bool fail_reads;
    bool fail_writes;
    uint32_t delay_total_ms;
} fake_bus_t;

static void reset_registers(uint8_t *registers)
{
    memset(registers, 0, 0x27U);
    registers[REG_MEAS_RATE] = 0x22U;
    registers[REG_GAIN] = 0x01U;
    registers[REG_PART_ID] = 0xB2U;
    registers[REG_MAIN_STATUS] = 0x20U;
}

static void reset_fake_bus(fake_bus_t *bus)
{
    memset(bus, 0, sizeof(*bus));
    reset_registers(bus->registers);
}

static bool fake_read(void *context,
                      uint8_t first_register,
                      uint8_t *data,
                      size_t length)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    if (bus->fail_reads || (size_t)first_register + length > 0x27U) {
        return false;
    }
    memcpy(data, &bus->registers[first_register], length);
    return true;
}

static bool fake_write(void *context,
                       uint8_t register_address,
                       uint8_t value)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    if (bus->fail_writes || register_address >= 0x27U) {
        return false;
    }
    if (register_address == REG_MAIN_CTRL && (value & 0x10U) != 0U) {
        reset_registers(bus->registers);
    } else {
        bus->registers[register_address] = value;
    }
    return true;
}

static void fake_delay(void *context, uint32_t milliseconds)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    bus->delay_total_ms += milliseconds;
}

static ltr390_transport_t fake_transport(fake_bus_t *bus)
{
    const ltr390_transport_t transport = {
        .read = fake_read,
        .write = fake_write,
        .delay_ms = fake_delay,
        .context = bus,
    };
    return transport;
}

static bool nearly_equal(float left, float right, float tolerance)
{
    const float difference = left > right ? left - right : right - left;
    return difference <= tolerance;
}

static void test_uvs_initialization_and_read(void)
{
    fake_bus_t bus;
    reset_fake_bus(&bus);
    const ltr390_transport_t transport = fake_transport(&bus);
    ltr390_t device;

    assert(ltr390_bind(&device, &transport) == LTR390_OK);
    assert(ltr390_init(&device, &ltr390_default_uvs_config) == LTR390_OK);
    assert(device.initialized);
    assert(bus.registers[REG_MAIN_CTRL] == 0x0AU);
    assert(bus.registers[REG_MEAS_RATE] == 0x04U);
    assert(bus.registers[REG_GAIN] == 0x04U);
    assert(bus.delay_total_ms == 20U);

    bus.registers[REG_MAIN_STATUS] = 0x08U;
    bool ready = false;
    assert(ltr390_data_ready(&device, &ready) == LTR390_OK);
    assert(ready);

    bus.registers[REG_UVS_DATA] = 0xF8U;
    bus.registers[REG_UVS_DATA + 1U] = 0x59U;
    bus.registers[REG_UVS_DATA + 2U] = 0x00U;

    ltr390_uvs_sample_t sample;
    assert(ltr390_read_uvs(&device, 1.0f, &sample) == LTR390_OK);
    assert(sample.raw == 23032U);
    assert(nearly_equal(sample.uvi, 10.0139f, 0.001f));
    assert(ltr390_read_als(&device, 1.0f, NULL) == LTR390_ERROR_ARGUMENT);
}

static void test_als_mode_and_conversion(void)
{
    fake_bus_t bus;
    reset_fake_bus(&bus);
    const ltr390_transport_t transport = fake_transport(&bus);
    ltr390_t device;

    assert(ltr390_bind(&device, &transport) == LTR390_OK);
    assert(ltr390_init(&device, &ltr390_default_uvs_config) == LTR390_OK);
    assert(ltr390_set_mode(&device, LTR390_MODE_ALS) == LTR390_OK);
    assert(bus.registers[REG_MAIN_CTRL] == 0x02U);

    bus.registers[REG_ALS_DATA] = 0xB0U;
    bus.registers[REG_ALS_DATA + 1U] = 0x04U;
    bus.registers[REG_ALS_DATA + 2U] = 0x00U;

    ltr390_als_sample_t sample;
    assert(ltr390_read_als(&device, 1.0f, &sample) == LTR390_OK);
    assert(sample.raw == 1200U);
    assert(nearly_equal(sample.lux, 10.0f, 0.0001f));

    ltr390_uvs_sample_t uvs_sample;
    assert(ltr390_read_uvs(&device, 1.0f, &uvs_sample) ==
           LTR390_ERROR_MODE);
}

static void test_errors(void)
{
    fake_bus_t bus;
    reset_fake_bus(&bus);
    const ltr390_transport_t transport = fake_transport(&bus);
    ltr390_t device;

    assert(ltr390_bind(&device, &transport) == LTR390_OK);
    bus.registers[REG_PART_ID] = 0xA2U;
    assert(ltr390_init(&device, &ltr390_default_uvs_config) ==
           LTR390_ERROR_PART_ID);

    reset_fake_bus(&bus);
    ltr390_config_t invalid = ltr390_default_uvs_config;
    invalid.resolution = (ltr390_resolution_t)0x60;
    assert(ltr390_init(&device, &invalid) == LTR390_ERROR_CONFIGURATION);

    bus.fail_reads = true;
    assert(ltr390_init(&device, &ltr390_default_uvs_config) ==
           LTR390_ERROR_COMMUNICATION);
    assert(ltr390_set_mode(&device, LTR390_MODE_ALS) ==
           LTR390_ERROR_NOT_INITIALIZED);
}

static uint8_t hal_registers[0x27];
static uint16_t hal_last_device_address;
static uint16_t hal_last_memory_address_size;
static uint32_t hal_last_timeout;
static uint32_t hal_delay_total;

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c,
                                   uint16_t device_address,
                                   uint16_t memory_address,
                                   uint16_t memory_address_size,
                                   uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout)
{
    (void)i2c;
    if ((uint32_t)memory_address + size > sizeof(hal_registers)) {
        return HAL_ERROR;
    }
    hal_last_device_address = device_address;
    hal_last_memory_address_size = memory_address_size;
    hal_last_timeout = timeout;
    memcpy(data, &hal_registers[memory_address], size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c,
                                    uint16_t device_address,
                                    uint16_t memory_address,
                                    uint16_t memory_address_size,
                                    uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout)
{
    (void)i2c;
    if ((uint32_t)memory_address + size > sizeof(hal_registers)) {
        return HAL_ERROR;
    }
    hal_last_device_address = device_address;
    hal_last_memory_address_size = memory_address_size;
    hal_last_timeout = timeout;
    if (memory_address == REG_MAIN_CTRL && size == 1U &&
        (data[0] & 0x10U) != 0U) {
        reset_registers(hal_registers);
    } else {
        memcpy(&hal_registers[memory_address], data, size);
    }
    return HAL_OK;
}

void HAL_Delay(uint32_t milliseconds)
{
    hal_delay_total += milliseconds;
}

static void test_stm32_adapter(void)
{
    reset_registers(hal_registers);
    hal_last_device_address = 0U;
    hal_last_memory_address_size = 0U;
    hal_last_timeout = 0U;
    hal_delay_total = 0U;

    I2C_HandleTypeDef i2c = {
        .Init = {
            .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
        },
    };
    ltr390_t device;
    ltr390_stm32_bus_t bus;

    assert(ltr390_stm32_bind(&device, &bus, &i2c, 25U) == LTR390_OK);
    assert(ltr390_init(&device, &ltr390_default_uvs_config) == LTR390_OK);
    assert(hal_last_device_address == (uint16_t)(LTR390_I2C_ADDRESS << 1U));
    assert(hal_last_memory_address_size == I2C_MEMADD_SIZE_8BIT);
    assert(hal_last_timeout == 25U);
    assert(hal_delay_total == 20U);

    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    assert(ltr390_stm32_bind(&device, &bus, &i2c, 25U) ==
           LTR390_ERROR_CONFIGURATION);
}

int main(void)
{
    test_uvs_initialization_and_read();
    test_als_mode_and_conversion();
    test_errors();
    test_stm32_adapter();
    puts("LTR390 tests passed");
    return 0;
}
