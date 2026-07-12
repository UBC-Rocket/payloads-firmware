/**
 * @file ltr390.h
 * @brief Bus-independent LTR-390UV-01 ambient-light and UV sensor driver.
 */

#ifndef LTR390_H
#define LTR390_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LTR390_I2C_ADDRESS 0x53U
#define LTR390_PART_ID_MASK 0xF0U
#define LTR390_PART_ID_VALUE 0xB0U

typedef enum {
    LTR390_OK = 0,
    LTR390_ERROR_ARGUMENT,
    LTR390_ERROR_COMMUNICATION,
    LTR390_ERROR_PART_ID,
    LTR390_ERROR_CONFIGURATION,
    LTR390_ERROR_NOT_INITIALIZED,
    LTR390_ERROR_MODE
} ltr390_status_t;

typedef enum {
    LTR390_MODE_ALS = 0x00,
    LTR390_MODE_UVS = 0x08
} ltr390_mode_t;

typedef enum {
    LTR390_RESOLUTION_20_BIT = 0x00,
    LTR390_RESOLUTION_19_BIT = 0x10,
    LTR390_RESOLUTION_18_BIT = 0x20,
    LTR390_RESOLUTION_17_BIT = 0x30,
    LTR390_RESOLUTION_16_BIT = 0x40,
    LTR390_RESOLUTION_13_BIT = 0x50
} ltr390_resolution_t;

typedef enum {
    LTR390_RATE_25_MS = 0x00,
    LTR390_RATE_50_MS = 0x01,
    LTR390_RATE_100_MS = 0x02,
    LTR390_RATE_200_MS = 0x03,
    LTR390_RATE_500_MS = 0x04,
    LTR390_RATE_1000_MS = 0x05,
    LTR390_RATE_2000_MS = 0x06
} ltr390_measurement_rate_t;

typedef enum {
    LTR390_GAIN_1X = 0x00,
    LTR390_GAIN_3X = 0x01,
    LTR390_GAIN_6X = 0x02,
    LTR390_GAIN_9X = 0x03,
    LTR390_GAIN_18X = 0x04
} ltr390_gain_t;

typedef struct {
    ltr390_mode_t mode;
    ltr390_resolution_t resolution;
    ltr390_measurement_rate_t measurement_rate;
    ltr390_gain_t gain;
} ltr390_config_t;

/** Datasheet reset configuration, activated in ALS mode. */
extern const ltr390_config_t ltr390_default_als_config;

/** UV configuration matching the datasheet's 2300-count/UVI sensitivity. */
extern const ltr390_config_t ltr390_default_uvs_config;

typedef bool (*ltr390_read_fn)(void *context,
                               uint8_t first_register,
                               uint8_t *data,
                               size_t length);

typedef bool (*ltr390_write_fn)(void *context,
                                uint8_t register_address,
                                uint8_t value);

typedef void (*ltr390_delay_ms_fn)(void *context, uint32_t milliseconds);

typedef struct {
    ltr390_read_fn read;
    ltr390_write_fn write;
    ltr390_delay_ms_fn delay_ms;
    void *context;
} ltr390_transport_t;

typedef struct {
    uint32_t raw;
    float lux;
} ltr390_als_sample_t;

typedef struct {
    uint32_t raw;
    float uvi;
} ltr390_uvs_sample_t;

typedef struct {
    ltr390_transport_t transport;
    ltr390_config_t config;
    bool initialized;
} ltr390_t;

/** Attach a register transport to a device instance. */
ltr390_status_t ltr390_bind(ltr390_t *device,
                            const ltr390_transport_t *transport);

/** Identify, reset, configure, activate, and verify the sensor. */
ltr390_status_t ltr390_init(ltr390_t *device,
                            const ltr390_config_t *config);

/** Change between ALS and UVS operation while retaining rate/gain settings. */
ltr390_status_t ltr390_set_mode(ltr390_t *device, ltr390_mode_t mode);

/** Query the new-data flag in MAIN_STATUS. */
ltr390_status_t ltr390_data_ready(ltr390_t *device, bool *ready);

/** Read the active channel as one coherent 20-bit raw value. */
ltr390_status_t ltr390_read_raw(ltr390_t *device, uint32_t *raw);

/** Read ALS data and convert it to lux using the supplied window factor. */
ltr390_status_t ltr390_read_als(ltr390_t *device,
                                float window_factor,
                                ltr390_als_sample_t *sample);

/**
 * Read UVS data and convert it to UV index using the supplied window factor.
 * The datasheet's 2300-count/UVI point (18x, 20-bit) is scaled linearly for
 * other supported gain and integration-time settings.
 */
ltr390_status_t ltr390_read_uvs(ltr390_t *device,
                                float window_factor,
                                ltr390_uvs_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* LTR390_H */
