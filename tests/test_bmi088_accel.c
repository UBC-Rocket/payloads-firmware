#include "accel/bmi088_accel.h"
#include "accel/bmi088_accel_stm32.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define REG_CHIP_ID 0x00U
#define REG_ERROR 0x02U
#define REG_STATUS 0x03U
#define REG_DATA 0x12U
#define REG_TEMPERATURE 0x22U
#define REG_CONFIG 0x40U
#define REG_RANGE 0x41U
#define REG_POWER_CONFIG 0x7CU
#define REG_POWER_CONTROL 0x7DU
#define REG_SOFT_RESET 0x7EU

typedef struct {
    uint8_t registers[128];
    uint8_t chip_id;
    uint32_t delay_total_ms;
    size_t chip_id_reads;
    size_t writes;
    bool fail_reads;
    bool fail_writes;
} fake_bus_t;

static int failures;

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            fprintf(stderr, "check failed at %s:%d: %s\n",                  \
                    __FILE__, __LINE__, #condition);                          \
            failures++;                                                       \
        }                                                                     \
    } while (0)

typedef struct {
    uint8_t registers[128];
    uint8_t chip_id;
    uint8_t last_tx[8];
    uint16_t last_transfer_size;
    size_t chip_id_reads;
    size_t cs_low_count;
    size_t cs_high_count;
    uint32_t delay_total_ms;
    uint32_t peripheral_clock_hz;
    GPIO_PinState chip_select_state;
    bool protocol_error;
    bool force_error;
} fake_hal_t;

static fake_hal_t fake_hal;

static void fake_hal_reset(void)
{
    memset(&fake_hal, 0, sizeof(fake_hal));
    fake_hal.chip_id = BMI088_ACCEL_CHIP_ID;
    fake_hal.registers[REG_CHIP_ID] = fake_hal.chip_id;
    fake_hal.registers[REG_CONFIG] = 0xA8U;
    fake_hal.registers[REG_RANGE] = 0x01U;
    fake_hal.registers[REG_POWER_CONFIG] = 0x03U;
    fake_hal.peripheral_clock_hz = 64000000U;
    fake_hal.chip_select_state = GPIO_PIN_SET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port,
                       uint16_t pin,
                       GPIO_PinState state)
{
    if (port == NULL || pin == 0U) {
        fake_hal.protocol_error = true;
    }

    fake_hal.chip_select_state = state;
    if (state == GPIO_PIN_RESET) {
        fake_hal.cs_low_count++;
    } else {
        fake_hal.cs_high_count++;
    }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout)
{
    (void)spi;
    (void)timeout;
    if (fake_hal.force_error) {
        return HAL_ERROR;
    }
    if (data == NULL || size != 2U ||
        fake_hal.chip_select_state != GPIO_PIN_RESET) {
        fake_hal.protocol_error = true;
        return HAL_ERROR;
    }

    memcpy(fake_hal.last_tx, data, size);
    fake_hal.last_transfer_size = size;
    const uint8_t register_address = (uint8_t)(data[0] & 0x7FU);
    if (register_address == REG_SOFT_RESET && data[1] == 0xB6U) {
        memset(fake_hal.registers, 0, sizeof(fake_hal.registers));
        fake_hal.registers[REG_CHIP_ID] = fake_hal.chip_id;
        fake_hal.registers[REG_CONFIG] = 0xA8U;
        fake_hal.registers[REG_RANGE] = 0x01U;
        fake_hal.registers[REG_POWER_CONFIG] = 0x03U;
    } else {
        fake_hal.registers[register_address] = data[1];
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
                                          const uint8_t *tx_data,
                                          uint8_t *rx_data,
                                          uint16_t size,
                                          uint32_t timeout)
{
    (void)spi;
    (void)timeout;
    if (fake_hal.force_error) {
        return HAL_ERROR;
    }
    if (tx_data == NULL || rx_data == NULL || size < 3U ||
        size > sizeof(fake_hal.last_tx) ||
        fake_hal.chip_select_state != GPIO_PIN_RESET) {
        fake_hal.protocol_error = true;
        return HAL_ERROR;
    }

    memcpy(fake_hal.last_tx, tx_data, size);
    fake_hal.last_transfer_size = size;
    memset(rx_data, 0, size);

    if ((tx_data[0] & 0x80U) == 0U) {
        fake_hal.protocol_error = true;
    }
    for (uint16_t index = 1U; index < size; ++index) {
        if (tx_data[index] != 0U) {
            fake_hal.protocol_error = true;
        }
    }

    const uint8_t first_register = (uint8_t)(tx_data[0] & 0x7FU);
    for (uint16_t index = 0U; index < size - 2U; ++index) {
        uint8_t value = fake_hal.registers[first_register + index];
        if (first_register == REG_CHIP_ID && index == 0U) {
            fake_hal.chip_id_reads++;
            if (fake_hal.chip_id_reads == 1U ||
                fake_hal.chip_id_reads == 3U) {
                value = 0U;
            }
        }
        rx_data[index + 2U] = value;
    }

    return HAL_OK;
}

void HAL_Delay(uint32_t milliseconds)
{
    fake_hal.delay_total_ms += milliseconds;
}

uint32_t HAL_RCC_GetPCLK1Freq(void)
{
    return fake_hal.peripheral_clock_hz;
}

static void check_close(float actual, float expected, float tolerance)
{
    const float difference = actual > expected
        ? actual - expected
        : expected - actual;
    CHECK(difference <= tolerance);
}

static void fake_bus_reset(fake_bus_t *bus, uint8_t chip_id)
{
    memset(bus, 0, sizeof(*bus));
    bus->chip_id = chip_id;
    bus->registers[REG_CHIP_ID] = chip_id;
    bus->registers[REG_CONFIG] = 0xA8U;
    bus->registers[REG_RANGE] = 0x01U;
    bus->registers[REG_POWER_CONFIG] = 0x03U;
}

static bool fake_read(void *context,
                      uint8_t first_register,
                      uint8_t *data,
                      size_t length)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    if (bus->fail_reads || data == NULL || length == 0U ||
        (size_t)first_register + length > sizeof(bus->registers)) {
        return false;
    }

    if (first_register == REG_CHIP_ID && length == 1U) {
        bus->chip_id_reads++;
        /* Reads 1 and 3 model the discarded SPI reads before and after reset. */
        data[0] = (bus->chip_id_reads == 1U || bus->chip_id_reads == 3U)
            ? 0U
            : bus->chip_id;
        return true;
    }

    memcpy(data, &bus->registers[first_register], length);
    return true;
}

static bool fake_write(void *context, uint8_t register_address, uint8_t value)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    if (bus->fail_writes || register_address >= sizeof(bus->registers)) {
        return false;
    }

    bus->writes++;
    if (register_address == REG_SOFT_RESET && value == 0xB6U) {
        const uint8_t chip_id = bus->chip_id;
        memset(bus->registers, 0, sizeof(bus->registers));
        bus->registers[REG_CHIP_ID] = chip_id;
        bus->registers[REG_CONFIG] = 0xA8U;
        bus->registers[REG_RANGE] = 0x01U;
        bus->registers[REG_POWER_CONFIG] = 0x03U;
        return true;
    }

    bus->registers[register_address] = value;
    return true;
}

static void fake_delay_ms(void *context, uint32_t milliseconds)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    bus->delay_total_ms += milliseconds;
}

static bmi088_accel_status_t bind_fake_device(bmi088_accel_t *device,
                                               fake_bus_t *bus)
{
    const bmi088_accel_transport_t transport = {
        .read = fake_read,
        .write = fake_write,
        .delay_ms = fake_delay_ms,
        .context = bus,
    };
    return bmi088_accel_bind(device, &transport);
}

static void test_initialization(void)
{
    fake_bus_t bus;
    bmi088_accel_t device;
    fake_bus_reset(&bus, BMI088_ACCEL_CHIP_ID);

    CHECK(bind_fake_device(&device, &bus) == BMI088_ACCEL_OK);
    CHECK(bmi088_accel_init(&device, &bmi088_accel_default_config) ==
          BMI088_ACCEL_OK);
    CHECK(device.initialized);
    CHECK(bus.chip_id_reads == 4U);
    CHECK(bus.writes == 5U);
    CHECK(bus.delay_total_ms == 56U);
    CHECK(bus.registers[REG_POWER_CONFIG] == 0x00U);
    CHECK(bus.registers[REG_POWER_CONTROL] == 0x04U);
    CHECK(bus.registers[REG_RANGE] == 0x01U);
    CHECK(bus.registers[REG_CONFIG] == 0xA8U);
    CHECK(bus.registers[REG_ERROR] == 0x00U);
}

static void test_sample_and_temperature(void)
{
    fake_bus_t bus;
    bmi088_accel_t device;
    fake_bus_reset(&bus, BMI088_ACCEL_CHIP_ID);
    CHECK(bind_fake_device(&device, &bus) == BMI088_ACCEL_OK);
    CHECK(bmi088_accel_init(&device, &bmi088_accel_default_config) ==
          BMI088_ACCEL_OK);

    bus.registers[REG_STATUS] = 0x80U;
    bool ready = false;
    CHECK(bmi088_accel_data_ready(&device, &ready) == BMI088_ACCEL_OK);
    CHECK(ready);

    /* +16384, -16384, +32767 in little-endian register order. */
    const uint8_t raw_sample[6] = {0x00U, 0x40U, 0x00U, 0xC0U, 0xFFU, 0x7FU};
    memcpy(&bus.registers[REG_DATA], raw_sample, sizeof(raw_sample));

    bmi088_accel_sample_t sample;
    CHECK(bmi088_accel_read_sample(&device, &sample) == BMI088_ACCEL_OK);
    CHECK(sample.x == 16384);
    CHECK(sample.y == -16384);
    CHECK(sample.z == 32767);
    check_close(sample.x_mps2, 3.0f * 9.80665f, 0.001f);
    check_close(sample.y_mps2, -3.0f * 9.80665f, 0.001f);
    check_close(sample.z_mps2, 6.0f * 9.80665f, 0.003f);

    bus.registers[REG_TEMPERATURE] = 0x00U;
    bus.registers[REG_TEMPERATURE + 1U] = 0x60U;
    float temperature_c = 0.0f;
    CHECK(bmi088_accel_read_temperature(&device, &temperature_c) ==
          BMI088_ACCEL_OK);
    check_close(temperature_c, 23.375f, 0.0001f);

    bus.registers[REG_TEMPERATURE] = 0x80U;
    CHECK(bmi088_accel_read_temperature(&device, &temperature_c) ==
          BMI088_ACCEL_ERROR_DATA);
}

static void test_failures(void)
{
    fake_bus_t bus;
    bmi088_accel_t device;
    fake_bus_reset(&bus, BMI088_ACCEL_CHIP_ID);

    CHECK(bmi088_accel_bind(NULL, NULL) == BMI088_ACCEL_ERROR_ARGUMENT);
    CHECK(bind_fake_device(&device, &bus) == BMI088_ACCEL_OK);

    bmi088_accel_sample_t sample;
    CHECK(bmi088_accel_read_sample(&device, &sample) ==
          BMI088_ACCEL_ERROR_NOT_INITIALIZED);

    bmi088_accel_config_t invalid_config = bmi088_accel_default_config;
    invalid_config.bandwidth = (bmi088_accel_bandwidth_t)0x00;
    CHECK(bmi088_accel_init(&device, &invalid_config) ==
          BMI088_ACCEL_ERROR_CONFIGURATION);

    fake_bus_reset(&bus, 0x00U);
    CHECK(bind_fake_device(&device, &bus) == BMI088_ACCEL_OK);
    CHECK(bmi088_accel_init(&device, &bmi088_accel_default_config) ==
          BMI088_ACCEL_ERROR_CHIP_ID);

    fake_bus_reset(&bus, BMI088_ACCEL_CHIP_ID);
    bus.fail_reads = true;
    CHECK(bind_fake_device(&device, &bus) == BMI088_ACCEL_OK);
    CHECK(bmi088_accel_init(&device, &bmi088_accel_default_config) ==
          BMI088_ACCEL_ERROR_COMMUNICATION);

    fake_bus_reset(&bus, BMI088_ACCEL_CHIP_ID);
    bus.fail_writes = true;
    CHECK(bind_fake_device(&device, &bus) == BMI088_ACCEL_OK);
    CHECK(bmi088_accel_init(&device, &bmi088_accel_default_config) ==
          BMI088_ACCEL_ERROR_COMMUNICATION);
}

static void test_stm32_transport(void)
{
    fake_hal_reset();

    GPIO_TypeDef chip_select_port = {0};
    SPI_HandleTypeDef spi = {
        .Init = {
            .Mode = SPI_MODE_MASTER,
            .Direction = SPI_DIRECTION_2LINES,
            .DataSize = SPI_DATASIZE_8BIT,
            .CLKPolarity = SPI_POLARITY_LOW,
            .CLKPhase = SPI_PHASE_1EDGE,
            .NSS = SPI_NSS_SOFT,
            .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8,
            .FirstBit = SPI_FIRSTBIT_MSB,
            .TIMode = SPI_TIMODE_DISABLE,
            .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        },
    };
    bmi088_accel_t device;
    bmi088_accel_stm32_bus_t bus;

    CHECK(bmi088_accel_stm32_bind(&device,
                                  &bus,
                                  &spi,
                                  &chip_select_port,
                                  0x1000U,
                                  10U) == BMI088_ACCEL_OK);
    CHECK(fake_hal.chip_select_state == GPIO_PIN_SET);
    CHECK(bmi088_accel_init(&device, &bmi088_accel_default_config) ==
          BMI088_ACCEL_OK);
    CHECK(!fake_hal.protocol_error);
    CHECK(fake_hal.cs_high_count == fake_hal.cs_low_count + 1U);
    CHECK(fake_hal.delay_total_ms == 56U);

    fake_hal.registers[REG_STATUS] = 0x80U;
    bool ready = false;
    CHECK(bmi088_accel_data_ready(&device, &ready) == BMI088_ACCEL_OK);
    CHECK(ready);
    CHECK(fake_hal.last_transfer_size == 3U);
    CHECK(fake_hal.last_tx[0] == (uint8_t)(REG_STATUS | 0x80U));
    CHECK(fake_hal.last_tx[1] == 0U);

    const uint8_t raw_sample[6] = {0x00U, 0x40U, 0x00U, 0xC0U, 0xFFU, 0x7FU};
    memcpy(&fake_hal.registers[REG_DATA], raw_sample, sizeof(raw_sample));
    bmi088_accel_sample_t sample;
    CHECK(bmi088_accel_read_sample(&device, &sample) == BMI088_ACCEL_OK);
    CHECK(fake_hal.last_transfer_size == 8U);
    CHECK(fake_hal.last_tx[0] == (uint8_t)(REG_DATA | 0x80U));
    for (size_t index = 1U; index < fake_hal.last_transfer_size; ++index) {
        CHECK(fake_hal.last_tx[index] == 0U);
    }
    CHECK(sample.x == 16384);
    CHECK(sample.y == -16384);
    CHECK(sample.z == 32767);

    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    CHECK(bmi088_accel_stm32_bind(&device,
                                  &bus,
                                  &spi,
                                  &chip_select_port,
                                  0x1000U,
                                  10U) == BMI088_ACCEL_ERROR_CONFIGURATION);
}

int main(void)
{
    test_initialization();
    test_sample_and_temperature();
    test_failures();
    test_stm32_transport();

    if (failures != 0) {
        fprintf(stderr, "%d BMI088 test(s) failed\n", failures);
        return 1;
    }

    puts("BMI088 accelerometer tests passed");
    return 0;
}
