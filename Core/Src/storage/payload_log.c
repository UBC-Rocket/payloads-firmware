#include "storage/payload_log.h"

#include <stdio.h>

size_t payload_log_format_record(char *output,
                                 size_t output_size,
                                 const payload_log_record_t *record)
{
    if (output == NULL || output_size == 0U || record == NULL) {
        return 0U;
    }

    const int length = snprintf(
        output,
        output_size,
        "%llu,%d,%d,%d,%lu,%u,%u,%u,%u,%u,%lu,%lu\r\n",
        (unsigned long long)record->time_ms,
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
    if (length <= 0 || (size_t)length >= output_size) {
        output[0] = '\0';
        return 0U;
    }
    return (size_t)length;
}
