#include "storage/payload_log.h"

#include <stdio.h>

static size_t format_uint64_decimal(char *output,
                                    size_t output_size,
                                    uint64_t value)
{
    char reversed[20];
    size_t length = 0U;

    do {
        reversed[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    if (length >= output_size) {
        return 0U;
    }
    for (size_t index = 0U; index < length; index++) {
        output[index] = reversed[length - index - 1U];
    }
    output[length] = '\0';
    return length;
}

size_t payload_log_format_record(char *output,
                                 size_t output_size,
                                 const payload_log_record_t *record)
{
    if (output == NULL || output_size == 0U || record == NULL) {
        return 0U;
    }

    const size_t timestamp_length =
        format_uint64_decimal(output, output_size, record->time_ms);
    if (timestamp_length == 0U) {
        output[0] = '\0';
        return 0U;
    }

    const int length = snprintf(
        &output[timestamp_length],
        output_size - timestamp_length,
        ",%d,%d,%d,%lu,%u,%u,%u,%u,%u,%lu,%lu\r\n",
        (int)record->accel_x_raw,
        (int)record->accel_y_raw,
        (int)record->accel_z_raw,
        (unsigned long)record->uv_raw,
        (unsigned int)record->accel_valid,
        (unsigned int)record->uv_valid,
        (unsigned int)record->uv_new,
        (unsigned int)record->sensor_error_mask,
        (unsigned int)record->pump_on,
        (unsigned long)record->accel_fifo_skipped_total,
        (unsigned long)record->log_dropped_total);
    if (length <= 0 ||
        (size_t)length >= output_size - timestamp_length) {
        output[0] = '\0';
        return 0U;
    }
    return timestamp_length + (size_t)length;
}
