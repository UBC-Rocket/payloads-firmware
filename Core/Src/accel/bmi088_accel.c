#include "accel/bmi088_accel.h"

#include <string.h>

#define BMI088_ACCEL_REG_CHIP_ID 0x00U
#define BMI088_ACCEL_REG_ERROR 0x02U
#define BMI088_ACCEL_REG_STATUS 0x03U
#define BMI088_ACCEL_REG_DATA 0x12U
#define BMI088_ACCEL_REG_TEMPERATURE 0x22U
#define BMI088_ACCEL_REG_FIFO_LENGTH 0x24U
#define BMI088_ACCEL_REG_FIFO_DATA 0x26U
#define BMI088_ACCEL_REG_CONFIG 0x40U
#define BMI088_ACCEL_REG_RANGE 0x41U
#define BMI088_ACCEL_REG_FIFO_DOWNS 0x45U
#define BMI088_ACCEL_REG_FIFO_CONFIG_0 0x48U
#define BMI088_ACCEL_REG_FIFO_CONFIG_1 0x49U
#define BMI088_ACCEL_REG_POWER_CONFIG 0x7CU
#define BMI088_ACCEL_REG_POWER_CONTROL 0x7DU
#define BMI088_ACCEL_REG_SOFT_RESET 0x7EU

#define BMI088_ACCEL_SOFT_RESET_COMMAND 0xB6U
#define BMI088_ACCEL_POWER_ACTIVE 0x00U
#define BMI088_ACCEL_POWER_ENABLED 0x04U
#define BMI088_ACCEL_DATA_READY_MASK 0x80U
#define BMI088_ACCEL_ERROR_MASK 0x1DU
#define BMI088_ACCEL_GRAVITY_MPS2 9.80665f
#define BMI088_ACCEL_FIFO_DOWNS_NONE 0x80U
#define BMI088_ACCEL_FIFO_STREAM_MODE 0x02U
#define BMI088_ACCEL_FIFO_ACCEL_ENABLE 0x50U
#define BMI088_ACCEL_FIFO_DISABLE 0x10U
#define BMI088_ACCEL_FIFO_DATA_HEADER 0x84U
#define BMI088_ACCEL_FIFO_SKIP_HEADER 0x40U
#define BMI088_ACCEL_FIFO_SENSOR_TIME_HEADER 0x44U
#define BMI088_ACCEL_FIFO_CONFIG_HEADER 0x48U
#define BMI088_ACCEL_FIFO_SAMPLE_DROP_HEADER 0x50U
#define BMI088_ACCEL_FIFO_HEADER_MASK 0xFCU
#define BMI088_ACCEL_FIFO_OVERREAD_MSB 0x80U

const bmi088_accel_config_t bmi088_accel_default_config = {
    .range = BMI088_ACCEL_RANGE_6G,
    .odr = BMI088_ACCEL_ODR_100_HZ,
    .bandwidth = BMI088_ACCEL_BANDWIDTH_NORMAL,
};

static bool transport_is_valid(const bmi088_accel_t *device)
{
    return (device != NULL) &&
           (device->transport.read != NULL) &&
           (device->transport.write != NULL) &&
           (device->transport.delay_ms != NULL);
}

static bool config_is_valid(const bmi088_accel_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    const bool range_valid = config->range == BMI088_ACCEL_RANGE_3G ||
                             config->range == BMI088_ACCEL_RANGE_6G ||
                             config->range == BMI088_ACCEL_RANGE_12G ||
                             config->range == BMI088_ACCEL_RANGE_24G;
    const bool odr_valid = config->odr >= BMI088_ACCEL_ODR_12_5_HZ &&
                           config->odr <= BMI088_ACCEL_ODR_1600_HZ;
    const bool bandwidth_valid =
        config->bandwidth == BMI088_ACCEL_BANDWIDTH_OSR4 ||
        config->bandwidth == BMI088_ACCEL_BANDWIDTH_OSR2 ||
        config->bandwidth == BMI088_ACCEL_BANDWIDTH_NORMAL;

    return range_valid && odr_valid && bandwidth_valid;
}

static float scale_for_range(bmi088_accel_range_t range)
{
    float full_scale_g = 0.0f;

    switch (range) {
    case BMI088_ACCEL_RANGE_3G:
        full_scale_g = 3.0f;
        break;
    case BMI088_ACCEL_RANGE_6G:
        full_scale_g = 6.0f;
        break;
    case BMI088_ACCEL_RANGE_12G:
        full_scale_g = 12.0f;
        break;
    case BMI088_ACCEL_RANGE_24G:
        full_scale_g = 24.0f;
        break;
    default:
        return 0.0f;
    }

    return (full_scale_g * BMI088_ACCEL_GRAVITY_MPS2) / 32768.0f;
}

static void decode_sample(const bmi088_accel_t *device,
                          const uint8_t *raw,
                          bmi088_accel_sample_t *sample)
{
    sample->x = (int16_t)((uint16_t)raw[0] |
                          ((uint16_t)raw[1] << 8U));
    sample->y = (int16_t)((uint16_t)raw[2] |
                          ((uint16_t)raw[3] << 8U));
    sample->z = (int16_t)((uint16_t)raw[4] |
                          ((uint16_t)raw[5] << 8U));
    sample->x_mps2 = (float)sample->x * device->scale_mps2_per_lsb;
    sample->y_mps2 = (float)sample->y * device->scale_mps2_per_lsb;
    sample->z_mps2 = (float)sample->z * device->scale_mps2_per_lsb;
}

static bmi088_accel_status_t read_registers(bmi088_accel_t *device,
                                             uint8_t first_register,
                                             uint8_t *data,
                                             size_t length)
{
    if (!device->transport.read(device->transport.context,
                                first_register,
                                data,
                                length)) {
        return BMI088_ACCEL_ERROR_COMMUNICATION;
    }

    return BMI088_ACCEL_OK;
}

static bmi088_accel_status_t write_register(bmi088_accel_t *device,
                                             uint8_t register_address,
                                             uint8_t value)
{
    if (!device->transport.write(device->transport.context,
                                 register_address,
                                 value)) {
        return BMI088_ACCEL_ERROR_COMMUNICATION;
    }

    return BMI088_ACCEL_OK;
}

static void delay_ms(bmi088_accel_t *device, uint32_t milliseconds)
{
    device->transport.delay_ms(device->transport.context, milliseconds);
}

bmi088_accel_status_t bmi088_accel_bind(
    bmi088_accel_t *device,
    const bmi088_accel_transport_t *transport)
{
    if (device == NULL || transport == NULL || transport->read == NULL ||
        transport->write == NULL || transport->delay_ms == NULL) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }

    const bmi088_accel_transport_t transport_copy = *transport;
    memset(device, 0, sizeof(*device));
    device->transport = transport_copy;

    return BMI088_ACCEL_OK;
}

bmi088_accel_status_t bmi088_accel_init(
    bmi088_accel_t *device,
    const bmi088_accel_config_t *config)
{
    if (!transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }
    if (!config_is_valid(config)) {
        return BMI088_ACCEL_ERROR_CONFIGURATION;
    }

    device->initialized = false;
    uint8_t value = 0U;

    /* The accelerometer powers up in I2C mode. Discarding one SPI read causes
       the CS rising edge that selects SPI until the next reset. */
    delay_ms(device, 1U);
    (void)read_registers(device, BMI088_ACCEL_REG_CHIP_ID, &value, 1U);
    delay_ms(device, 1U);

    bmi088_accel_status_t status =
        read_registers(device, BMI088_ACCEL_REG_CHIP_ID, &value, 1U);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    if (value != BMI088_ACCEL_CHIP_ID) {
        return BMI088_ACCEL_ERROR_CHIP_ID;
    }

    status = write_register(device,
                            BMI088_ACCEL_REG_SOFT_RESET,
                            BMI088_ACCEL_SOFT_RESET_COMMAND);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    delay_ms(device, 2U);

    /* Soft reset also resets the serial interface selection. */
    (void)read_registers(device, BMI088_ACCEL_REG_CHIP_ID, &value, 1U);
    delay_ms(device, 1U);
    status = read_registers(device, BMI088_ACCEL_REG_CHIP_ID, &value, 1U);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    if (value != BMI088_ACCEL_CHIP_ID) {
        return BMI088_ACCEL_ERROR_CHIP_ID;
    }

    status = write_register(device,
                            BMI088_ACCEL_REG_POWER_CONFIG,
                            BMI088_ACCEL_POWER_ACTIVE);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    delay_ms(device, 5U);

    status = write_register(device,
                            BMI088_ACCEL_REG_POWER_CONTROL,
                            BMI088_ACCEL_POWER_ENABLED);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    delay_ms(device, 5U);

    const uint8_t range_value = (uint8_t)config->range;
    const uint8_t config_value =
        (uint8_t)(((uint8_t)config->bandwidth << 4U) |
                  (uint8_t)config->odr);

    status = write_register(device, BMI088_ACCEL_REG_RANGE, range_value);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    delay_ms(device, 1U);

    status = write_register(device, BMI088_ACCEL_REG_CONFIG, config_value);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    /* Allow the digital filter and ODR change to settle before readback/use. */
    delay_ms(device, 40U);

    uint8_t range_readback = 0U;
    uint8_t config_readback = 0U;
    uint8_t power_config_readback = 0U;
    uint8_t power_control_readback = 0U;
    uint8_t device_error = 0U;

    status = read_registers(device,
                            BMI088_ACCEL_REG_RANGE,
                            &range_readback,
                            1U);
    if (status == BMI088_ACCEL_OK) {
        status = read_registers(device,
                                BMI088_ACCEL_REG_CONFIG,
                                &config_readback,
                                1U);
    }
    if (status == BMI088_ACCEL_OK) {
        status = read_registers(device,
                                BMI088_ACCEL_REG_POWER_CONFIG,
                                &power_config_readback,
                                1U);
    }
    if (status == BMI088_ACCEL_OK) {
        status = read_registers(device,
                                BMI088_ACCEL_REG_POWER_CONTROL,
                                &power_control_readback,
                                1U);
    }
    if (status == BMI088_ACCEL_OK) {
        status = read_registers(device,
                                BMI088_ACCEL_REG_ERROR,
                                &device_error,
                                1U);
    }
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    if (range_readback != range_value || config_readback != config_value ||
        power_config_readback != BMI088_ACCEL_POWER_ACTIVE ||
        power_control_readback != BMI088_ACCEL_POWER_ENABLED) {
        return BMI088_ACCEL_ERROR_CONFIGURATION;
    }
    if ((device_error & BMI088_ACCEL_ERROR_MASK) != 0U) {
        return BMI088_ACCEL_ERROR_DEVICE;
    }

    device->config = *config;
    device->scale_mps2_per_lsb = scale_for_range(config->range);
    device->initialized = true;

    return BMI088_ACCEL_OK;
}

bmi088_accel_status_t bmi088_accel_data_ready(
    bmi088_accel_t *device,
    bool *ready)
{
    if (device == NULL || ready == NULL) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_NOT_INITIALIZED;
    }

    uint8_t status_register = 0U;
    const bmi088_accel_status_t status =
        read_registers(device,
                       BMI088_ACCEL_REG_STATUS,
                       &status_register,
                       1U);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    *ready = (status_register & BMI088_ACCEL_DATA_READY_MASK) != 0U;
    return BMI088_ACCEL_OK;
}

bmi088_accel_status_t bmi088_accel_read_sample(
    bmi088_accel_t *device,
    bmi088_accel_sample_t *sample)
{
    if (device == NULL || sample == NULL) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_NOT_INITIALIZED;
    }

    uint8_t raw[6] = {0U};
    const bmi088_accel_status_t status =
        read_registers(device, BMI088_ACCEL_REG_DATA, raw, sizeof(raw));
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    decode_sample(device, raw, sample);

    return BMI088_ACCEL_OK;
}

bmi088_accel_status_t bmi088_accel_read_temperature(
    bmi088_accel_t *device,
    float *temperature_c)
{
    if (device == NULL || temperature_c == NULL) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_NOT_INITIALIZED;
    }

    uint8_t raw[2] = {0U};
    const bmi088_accel_status_t status =
        read_registers(device,
                       BMI088_ACCEL_REG_TEMPERATURE,
                       raw,
                       sizeof(raw));
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    if (raw[0] == 0x80U) {
        return BMI088_ACCEL_ERROR_DATA;
    }

    const uint16_t unsigned_value =
        (uint16_t)(((uint16_t)raw[0] << 3U) |
                   ((uint16_t)raw[1] >> 5U));
    const int16_t signed_value = unsigned_value > 1023U
        ? (int16_t)((int32_t)unsigned_value - 2048)
        : (int16_t)unsigned_value;

    *temperature_c = ((float)signed_value * 0.125f) + 23.0f;
    return BMI088_ACCEL_OK;
}

bmi088_accel_status_t bmi088_accel_fifo_enable(bmi088_accel_t *device)
{
    if (device == NULL || !device->initialized ||
        !transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_NOT_INITIALIZED;
    }

    bmi088_accel_status_t status =
        write_register(device,
                       BMI088_ACCEL_REG_FIFO_DOWNS,
                       BMI088_ACCEL_FIFO_DOWNS_NONE);
    if (status == BMI088_ACCEL_OK) {
        status = write_register(device,
                                BMI088_ACCEL_REG_FIFO_CONFIG_0,
                                BMI088_ACCEL_FIFO_STREAM_MODE);
    }
    if (status == BMI088_ACCEL_OK) {
        status = write_register(device,
                                BMI088_ACCEL_REG_FIFO_CONFIG_1,
                                BMI088_ACCEL_FIFO_ACCEL_ENABLE);
    }
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    uint8_t readback[3] = {0U};
    status = read_registers(device,
                            BMI088_ACCEL_REG_FIFO_DOWNS,
                            &readback[0],
                            1U);
    if (status == BMI088_ACCEL_OK) {
        status = read_registers(device,
                                BMI088_ACCEL_REG_FIFO_CONFIG_0,
                                &readback[1],
                                1U);
    }
    if (status == BMI088_ACCEL_OK) {
        status = read_registers(device,
                                BMI088_ACCEL_REG_FIFO_CONFIG_1,
                                &readback[2],
                                1U);
    }
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    if (readback[0] != BMI088_ACCEL_FIFO_DOWNS_NONE ||
        readback[1] != BMI088_ACCEL_FIFO_STREAM_MODE ||
        readback[2] != BMI088_ACCEL_FIFO_ACCEL_ENABLE) {
        return BMI088_ACCEL_ERROR_CONFIGURATION;
    }

    return BMI088_ACCEL_OK;
}

bmi088_accel_status_t bmi088_accel_fifo_disable(bmi088_accel_t *device)
{
    if (device == NULL || !device->initialized ||
        !transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_NOT_INITIALIZED;
    }

    return write_register(device,
                          BMI088_ACCEL_REG_FIFO_CONFIG_1,
                          BMI088_ACCEL_FIFO_DISABLE);
}

bmi088_accel_status_t bmi088_accel_fifo_read(
    bmi088_accel_t *device,
    bmi088_accel_fifo_batch_t *batch)
{
    if (device == NULL || batch == NULL) {
        return BMI088_ACCEL_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return BMI088_ACCEL_ERROR_NOT_INITIALIZED;
    }

    memset(batch, 0, sizeof(*batch));

    uint8_t length_bytes[2] = {0U};
    bmi088_accel_status_t status =
        read_registers(device,
                       BMI088_ACCEL_REG_FIFO_LENGTH,
                       length_bytes,
                       sizeof(length_bytes));
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    uint16_t fifo_length =
        (uint16_t)((uint16_t)length_bytes[0] |
                   (((uint16_t)length_bytes[1] & 0x3FU) << 8U));
    if (fifo_length > BMI088_ACCEL_FIFO_CAPACITY_BYTES) {
        fifo_length = BMI088_ACCEL_FIFO_CAPACITY_BYTES;
        batch->more_data = true;
    }
    batch->fifo_bytes = fifo_length;
    if (fifo_length == 0U) {
        return BMI088_ACCEL_OK;
    }

    const size_t requested = (size_t)fifo_length + 4U;
    status = read_registers(device,
                            BMI088_ACCEL_REG_FIFO_DATA,
                            device->fifo_buffer,
                            requested);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    size_t offset = 0U;
    while (offset < requested) {
        const uint8_t header = device->fifo_buffer[offset];
        const uint8_t frame_type =
            (uint8_t)(header & BMI088_ACCEL_FIFO_HEADER_MASK);

        if (frame_type == BMI088_ACCEL_FIFO_DATA_HEADER) {
            if ((requested - offset) < 7U) {
                batch->more_data = true;
                break;
            }
            if (batch->sample_count >= BMI088_ACCEL_FIFO_MAX_SAMPLES) {
                batch->more_data = true;
                break;
            }
            decode_sample(device,
                          &device->fifo_buffer[offset + 1U],
                          &batch->samples[batch->sample_count]);
            batch->sample_count++;
            offset += 7U;
        } else if (frame_type == BMI088_ACCEL_FIFO_SKIP_HEADER) {
            if ((requested - offset) < 2U) {
                batch->more_data = true;
                break;
            }
            batch->skipped_frames =
                (uint16_t)(batch->skipped_frames +
                           device->fifo_buffer[offset + 1U]);
            offset += 2U;
        } else if (frame_type == BMI088_ACCEL_FIFO_SENSOR_TIME_HEADER) {
            if ((requested - offset) < 4U) {
                batch->more_data = true;
                break;
            }
            batch->sensor_time_ticks =
                (uint32_t)device->fifo_buffer[offset + 1U] |
                ((uint32_t)device->fifo_buffer[offset + 2U] << 8U) |
                ((uint32_t)device->fifo_buffer[offset + 3U] << 16U);
            batch->sensor_time_valid = true;
            offset += 4U;
        } else if (frame_type == BMI088_ACCEL_FIFO_CONFIG_HEADER) {
            if ((requested - offset) < 2U) {
                batch->more_data = true;
                break;
            }
            batch->configuration_frames++;
            offset += 2U;
        } else if (frame_type == BMI088_ACCEL_FIFO_SAMPLE_DROP_HEADER) {
            if ((requested - offset) < 2U) {
                batch->more_data = true;
                break;
            }
            batch->sample_drop_frames++;
            offset += 2U;
        } else if (header == BMI088_ACCEL_FIFO_OVERREAD_MSB &&
                   (requested - offset) >= 2U &&
                   device->fifo_buffer[offset + 1U] == 0U) {
            break;
        } else {
            batch->malformed_frames++;
            offset++;
        }
    }

    return batch->malformed_frames == 0U
        ? BMI088_ACCEL_OK
        : BMI088_ACCEL_ERROR_DATA;
}
