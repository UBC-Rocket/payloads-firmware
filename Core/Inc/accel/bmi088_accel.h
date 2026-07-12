/**
 * @file bmi088_accel.h
 * @brief Bus-independent BMI088 accelerometer driver.
 *
 * The transport callbacks hide the MCU-specific SPI implementation. The
 * driver performs device identification, reset, power-up, configuration,
 * readback verification, and conversion of raw samples to m/s^2.
 */

#ifndef BMI088_ACCEL_H
#define BMI088_ACCEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_ACCEL_CHIP_ID 0x1EU

typedef enum {
    BMI088_ACCEL_OK = 0,
    BMI088_ACCEL_ERROR_ARGUMENT,
    BMI088_ACCEL_ERROR_COMMUNICATION,
    BMI088_ACCEL_ERROR_CHIP_ID,
    BMI088_ACCEL_ERROR_CONFIGURATION,
    BMI088_ACCEL_ERROR_DEVICE,
    BMI088_ACCEL_ERROR_NOT_INITIALIZED,
    BMI088_ACCEL_ERROR_DATA
} bmi088_accel_status_t;

typedef enum {
    BMI088_ACCEL_RANGE_3G = 0x00,
    BMI088_ACCEL_RANGE_6G = 0x01,
    BMI088_ACCEL_RANGE_12G = 0x02,
    BMI088_ACCEL_RANGE_24G = 0x03
} bmi088_accel_range_t;

typedef enum {
    BMI088_ACCEL_ODR_12_5_HZ = 0x05,
    BMI088_ACCEL_ODR_25_HZ = 0x06,
    BMI088_ACCEL_ODR_50_HZ = 0x07,
    BMI088_ACCEL_ODR_100_HZ = 0x08,
    BMI088_ACCEL_ODR_200_HZ = 0x09,
    BMI088_ACCEL_ODR_400_HZ = 0x0A,
    BMI088_ACCEL_ODR_800_HZ = 0x0B,
    BMI088_ACCEL_ODR_1600_HZ = 0x0C
} bmi088_accel_odr_t;

/** Values written to the ACC_CONF acc_bwp field (bits 7:4). */
typedef enum {
    BMI088_ACCEL_BANDWIDTH_OSR4 = 0x08,
    BMI088_ACCEL_BANDWIDTH_OSR2 = 0x09,
    BMI088_ACCEL_BANDWIDTH_NORMAL = 0x0A
} bmi088_accel_bandwidth_t;

typedef struct {
    bmi088_accel_range_t range;
    bmi088_accel_odr_t odr;
    bmi088_accel_bandwidth_t bandwidth;
} bmi088_accel_config_t;

/** Datasheet reset settings, with the accelerometer subsequently powered on. */
extern const bmi088_accel_config_t bmi088_accel_default_config;

typedef bool (*bmi088_accel_read_fn)(void *context,
                                     uint8_t first_register,
                                     uint8_t *data,
                                     size_t length);

typedef bool (*bmi088_accel_write_fn)(void *context,
                                      uint8_t register_address,
                                      uint8_t value);

typedef void (*bmi088_accel_delay_ms_fn)(void *context, uint32_t milliseconds);

typedef struct {
    bmi088_accel_read_fn read;
    bmi088_accel_write_fn write;
    bmi088_accel_delay_ms_fn delay_ms;
    void *context;
} bmi088_accel_transport_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    float x_mps2;
    float y_mps2;
    float z_mps2;
} bmi088_accel_sample_t;

typedef struct {
    bmi088_accel_transport_t transport;
    bmi088_accel_config_t config;
    float scale_mps2_per_lsb;
    bool initialized;
} bmi088_accel_t;

/**
 * @brief Attach a register transport to a device instance.
 */
bmi088_accel_status_t bmi088_accel_bind(
    bmi088_accel_t *device,
    const bmi088_accel_transport_t *transport);

/**
 * @brief Reset, identify, power on, configure, and verify the accelerometer.
 */
bmi088_accel_status_t bmi088_accel_init(
    bmi088_accel_t *device,
    const bmi088_accel_config_t *config);

/**
 * @brief Query the accelerometer data-ready flag.
 */
bmi088_accel_status_t bmi088_accel_data_ready(
    bmi088_accel_t *device,
    bool *ready);

/**
 * @brief Read one coherent XYZ sample and convert it to m/s^2.
 */
bmi088_accel_status_t bmi088_accel_read_sample(
    bmi088_accel_t *device,
    bmi088_accel_sample_t *sample);

/**
 * @brief Read the on-die temperature in degrees Celsius.
 */
bmi088_accel_status_t bmi088_accel_read_temperature(
    bmi088_accel_t *device,
    float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_ACCEL_H */

