#include "uv/ltr390.h"

#include <stddef.h>
#include <stdint.h>

#define LTR390_REG_MAIN_CTRL 0x00U
#define LTR390_REG_MEAS_RATE 0x04U
#define LTR390_REG_GAIN 0x05U
#define LTR390_REG_PART_ID 0x06U
#define LTR390_REG_MAIN_STATUS 0x07U
#define LTR390_REG_ALS_DATA 0x0DU
#define LTR390_REG_UVS_DATA 0x10U

#define LTR390_MAIN_CTRL_RESET 0x10U
#define LTR390_MAIN_CTRL_ENABLE 0x02U
#define LTR390_MAIN_CTRL_WRITABLE_MASK 0x1AU
#define LTR390_STATUS_DATA_READY 0x08U
#define LTR390_MEAS_RATE_MASK 0x77U
#define LTR390_GAIN_MASK 0x07U
#define LTR390_RESET_DELAY_MS 10U
#define LTR390_WAKE_DELAY_MS 10U
#define LTR390_UVS_SENSITIVITY_COUNTS_PER_UVI 2300.0f

const ltr390_config_t ltr390_default_als_config = {
    .mode = LTR390_MODE_ALS,
    .resolution = LTR390_RESOLUTION_18_BIT,
    .measurement_rate = LTR390_RATE_100_MS,
    .gain = LTR390_GAIN_3X,
};

const ltr390_config_t ltr390_default_uvs_config = {
    .mode = LTR390_MODE_UVS,
    .resolution = LTR390_RESOLUTION_20_BIT,
    .measurement_rate = LTR390_RATE_500_MS,
    .gain = LTR390_GAIN_18X,
};

static bool transport_is_valid(const ltr390_t *device)
{
    return device != NULL &&
           device->transport.read != NULL &&
           device->transport.write != NULL &&
           device->transport.delay_ms != NULL;
}

static bool mode_is_valid(ltr390_mode_t mode)
{
    return mode == LTR390_MODE_ALS || mode == LTR390_MODE_UVS;
}

static bool resolution_is_valid(ltr390_resolution_t resolution)
{
    return resolution == LTR390_RESOLUTION_20_BIT ||
           resolution == LTR390_RESOLUTION_19_BIT ||
           resolution == LTR390_RESOLUTION_18_BIT ||
           resolution == LTR390_RESOLUTION_17_BIT ||
           resolution == LTR390_RESOLUTION_16_BIT ||
           resolution == LTR390_RESOLUTION_13_BIT;
}

static bool rate_is_valid(ltr390_measurement_rate_t rate)
{
    return rate == LTR390_RATE_25_MS ||
           rate == LTR390_RATE_50_MS ||
           rate == LTR390_RATE_100_MS ||
           rate == LTR390_RATE_200_MS ||
           rate == LTR390_RATE_500_MS ||
           rate == LTR390_RATE_1000_MS ||
           rate == LTR390_RATE_2000_MS;
}

static bool gain_is_valid(ltr390_gain_t gain)
{
    return gain == LTR390_GAIN_1X ||
           gain == LTR390_GAIN_3X ||
           gain == LTR390_GAIN_6X ||
           gain == LTR390_GAIN_9X ||
           gain == LTR390_GAIN_18X;
}

static bool config_is_valid(const ltr390_config_t *config)
{
    return config != NULL && mode_is_valid(config->mode) &&
           resolution_is_valid(config->resolution) &&
           rate_is_valid(config->measurement_rate) &&
           gain_is_valid(config->gain);
}

static ltr390_status_t read_registers(ltr390_t *device,
                                      uint8_t first_register,
                                      uint8_t *data,
                                      size_t length)
{
    if (!device->transport.read(device->transport.context,
                                first_register,
                                data,
                                length)) {
        return LTR390_ERROR_COMMUNICATION;
    }
    return LTR390_OK;
}

static ltr390_status_t write_register(ltr390_t *device,
                                      uint8_t register_address,
                                      uint8_t value)
{
    if (!device->transport.write(device->transport.context,
                                 register_address,
                                 value)) {
        return LTR390_ERROR_COMMUNICATION;
    }
    return LTR390_OK;
}

static ltr390_status_t verify_part_id(ltr390_t *device)
{
    uint8_t part_id = 0U;
    const ltr390_status_t status =
        read_registers(device, LTR390_REG_PART_ID, &part_id, 1U);
    if (status != LTR390_OK) {
        return status;
    }
    if ((part_id & LTR390_PART_ID_MASK) != LTR390_PART_ID_VALUE) {
        return LTR390_ERROR_PART_ID;
    }
    return LTR390_OK;
}

static float gain_factor(ltr390_gain_t gain)
{
    switch (gain) {
    case LTR390_GAIN_1X:
        return 1.0f;
    case LTR390_GAIN_3X:
        return 3.0f;
    case LTR390_GAIN_6X:
        return 6.0f;
    case LTR390_GAIN_9X:
        return 9.0f;
    case LTR390_GAIN_18X:
        return 18.0f;
    default:
        return 0.0f;
    }
}

static float integration_factor(ltr390_resolution_t resolution)
{
    switch (resolution) {
    case LTR390_RESOLUTION_20_BIT:
        return 4.0f;
    case LTR390_RESOLUTION_19_BIT:
        return 2.0f;
    case LTR390_RESOLUTION_18_BIT:
        return 1.0f;
    case LTR390_RESOLUTION_17_BIT:
        return 0.5f;
    case LTR390_RESOLUTION_16_BIT:
        return 0.25f;
    case LTR390_RESOLUTION_13_BIT:
        return 0.125f;
    default:
        return 0.0f;
    }
}

ltr390_status_t ltr390_bind(ltr390_t *device,
                            const ltr390_transport_t *transport)
{
    if (device == NULL || transport == NULL || transport->read == NULL ||
        transport->write == NULL || transport->delay_ms == NULL) {
        return LTR390_ERROR_ARGUMENT;
    }

    device->transport = *transport;
    device->config = ltr390_default_uvs_config;
    device->initialized = false;
    return LTR390_OK;
}

ltr390_status_t ltr390_init(ltr390_t *device,
                            const ltr390_config_t *config)
{
    if (device == NULL || !transport_is_valid(device)) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (!config_is_valid(config)) {
        return LTR390_ERROR_CONFIGURATION;
    }

    device->initialized = false;

    ltr390_status_t status = verify_part_id(device);
    if (status != LTR390_OK) {
        return status;
    }

    status = write_register(device,
                            LTR390_REG_MAIN_CTRL,
                            LTR390_MAIN_CTRL_RESET);
    if (status != LTR390_OK) {
        return status;
    }
    device->transport.delay_ms(device->transport.context,
                               LTR390_RESET_DELAY_MS);

    status = verify_part_id(device);
    if (status != LTR390_OK) {
        return status;
    }

    const uint8_t measurement_config =
        (uint8_t)config->resolution | (uint8_t)config->measurement_rate;
    status = write_register(device, LTR390_REG_MEAS_RATE, measurement_config);
    if (status == LTR390_OK) {
        status = write_register(device, LTR390_REG_GAIN, (uint8_t)config->gain);
    }
    if (status == LTR390_OK) {
        status = write_register(device,
                                LTR390_REG_MAIN_CTRL,
                                (uint8_t)config->mode |
                                    LTR390_MAIN_CTRL_ENABLE);
    }
    if (status != LTR390_OK) {
        return status;
    }

    device->transport.delay_ms(device->transport.context,
                               LTR390_WAKE_DELAY_MS);

    uint8_t main_control = 0U;
    uint8_t measurement_readback = 0U;
    uint8_t gain_readback = 0U;
    status = read_registers(device,
                            LTR390_REG_MAIN_CTRL,
                            &main_control,
                            1U);
    if (status == LTR390_OK) {
        status = read_registers(device,
                                LTR390_REG_MEAS_RATE,
                                &measurement_readback,
                                1U);
    }
    if (status == LTR390_OK) {
        status = read_registers(device,
                                LTR390_REG_GAIN,
                                &gain_readback,
                                1U);
    }
    if (status != LTR390_OK) {
        return status;
    }

    const uint8_t expected_control =
        (uint8_t)config->mode | LTR390_MAIN_CTRL_ENABLE;
    if ((main_control & LTR390_MAIN_CTRL_WRITABLE_MASK) != expected_control ||
        (measurement_readback & LTR390_MEAS_RATE_MASK) != measurement_config ||
        (gain_readback & LTR390_GAIN_MASK) != (uint8_t)config->gain) {
        return LTR390_ERROR_CONFIGURATION;
    }

    device->config = *config;
    device->initialized = true;
    return LTR390_OK;
}

ltr390_status_t ltr390_set_mode(ltr390_t *device, ltr390_mode_t mode)
{
    if (device == NULL || !mode_is_valid(mode)) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return LTR390_ERROR_NOT_INITIALIZED;
    }

    const ltr390_status_t status =
        write_register(device,
                       LTR390_REG_MAIN_CTRL,
                       (uint8_t)mode | LTR390_MAIN_CTRL_ENABLE);
    if (status != LTR390_OK) {
        return status;
    }

    device->transport.delay_ms(device->transport.context,
                               LTR390_WAKE_DELAY_MS);

    uint8_t main_control = 0U;
    ltr390_status_t readback_status =
        read_registers(device, LTR390_REG_MAIN_CTRL, &main_control, 1U);
    if (readback_status != LTR390_OK) {
        return readback_status;
    }
    const uint8_t expected_control =
        (uint8_t)mode | LTR390_MAIN_CTRL_ENABLE;
    if ((main_control & LTR390_MAIN_CTRL_WRITABLE_MASK) != expected_control) {
        return LTR390_ERROR_CONFIGURATION;
    }

    device->config.mode = mode;
    return LTR390_OK;
}

ltr390_status_t ltr390_data_ready(ltr390_t *device, bool *ready)
{
    if (device == NULL || ready == NULL) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return LTR390_ERROR_NOT_INITIALIZED;
    }

    uint8_t main_status = 0U;
    const ltr390_status_t status =
        read_registers(device, LTR390_REG_MAIN_STATUS, &main_status, 1U);
    if (status != LTR390_OK) {
        return status;
    }

    *ready = (main_status & LTR390_STATUS_DATA_READY) != 0U;
    return LTR390_OK;
}

ltr390_status_t ltr390_read_raw(ltr390_t *device, uint32_t *raw)
{
    if (device == NULL || raw == NULL) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return LTR390_ERROR_NOT_INITIALIZED;
    }

    uint8_t bytes[3] = {0U};
    const uint8_t first_register = device->config.mode == LTR390_MODE_UVS
        ? LTR390_REG_UVS_DATA
        : LTR390_REG_ALS_DATA;
    const ltr390_status_t status =
        read_registers(device, first_register, bytes, sizeof(bytes));
    if (status != LTR390_OK) {
        return status;
    }

    *raw = (uint32_t)bytes[0] |
           ((uint32_t)bytes[1] << 8U) |
           (((uint32_t)bytes[2] & 0x0FU) << 16U);
    return LTR390_OK;
}

ltr390_status_t ltr390_read_als(ltr390_t *device,
                                float window_factor,
                                ltr390_als_sample_t *sample)
{
    if (device == NULL || sample == NULL || !(window_factor > 0.0f)) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return LTR390_ERROR_NOT_INITIALIZED;
    }
    if (device->config.mode != LTR390_MODE_ALS) {
        return LTR390_ERROR_MODE;
    }

    const ltr390_status_t status = ltr390_read_raw(device, &sample->raw);
    if (status != LTR390_OK) {
        return status;
    }

    const float divisor = gain_factor(device->config.gain) *
                          integration_factor(device->config.resolution);
    sample->lux = (0.6f * (float)sample->raw * window_factor) / divisor;
    return LTR390_OK;
}

ltr390_status_t ltr390_read_uvs(ltr390_t *device,
                                float window_factor,
                                ltr390_uvs_sample_t *sample)
{
    if (device == NULL || sample == NULL || !(window_factor > 0.0f)) {
        return LTR390_ERROR_ARGUMENT;
    }
    if (!device->initialized || !transport_is_valid(device)) {
        return LTR390_ERROR_NOT_INITIALIZED;
    }
    if (device->config.mode != LTR390_MODE_UVS) {
        return LTR390_ERROR_MODE;
    }

    const ltr390_status_t status = ltr390_read_raw(device, &sample->raw);
    if (status != LTR390_OK) {
        return status;
    }

    const float sensitivity = LTR390_UVS_SENSITIVITY_COUNTS_PER_UVI *
        (gain_factor(device->config.gain) / 18.0f) *
        (integration_factor(device->config.resolution) / 4.0f);
    sample->uvi = ((float)sample->raw * window_factor) / sensitivity;
    return LTR390_OK;
}
