/**
 * @file payload_log.h
 * @brief Stable CSV record format for payload flight data.
 */

#ifndef PAYLOAD_LOG_H
#define PAYLOAD_LOG_H

#include <stddef.h>
#include <stdint.h>

#define PAYLOAD_LOG_CSV_HEADER                                             \
    "time_ms,accel_x_raw,accel_y_raw,accel_z_raw,uv_raw,"                 \
    "accel_valid,uv_valid,uv_new,sensor_error_mask,"                      \
    "pump_on,accel_fifo_skipped_total,log_dropped_total\r\n"

typedef struct {
    uint64_t time_ms;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    uint32_t uv_raw;
    uint32_t accel_fifo_skipped_total;
    uint32_t log_dropped_total;
    uint8_t accel_valid;
    uint8_t uv_valid;
    uint8_t uv_new;
    uint8_t sensor_error_mask;
    uint8_t pump_on;
} payload_log_record_t;

/**
 * Return the number of characters written, or 0 when the output does not fit.
 */
size_t payload_log_format_record(char *output,
                                 size_t output_size,
                                 const payload_log_record_t *record);

#endif /* PAYLOAD_LOG_H */
