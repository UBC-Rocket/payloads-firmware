#include "storage/sd_logger.h"

#include <stdio.h>
#include <string.h>

static int failures;
static unsigned int mount_calls;
static unsigned int open_calls;
static unsigned int write_calls;
static unsigned int sync_calls;
static unsigned int close_calls;
static FRESULT forced_write_result;
static char last_write[SD_LOGGER_STAGING_SIZE + 1U];
static char last_open_path[20];

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            fprintf(stderr, "check failed at %s:%d: %s\n",                  \
                    __FILE__, __LINE__, #condition);                          \
            failures++;                                                       \
        }                                                                     \
    } while (0)

FRESULT f_mount(FATFS *filesystem, const TCHAR *path, BYTE option)
{
    (void)filesystem;
    (void)path;
    (void)option;
    mount_calls++;
    return FR_OK;
}

FRESULT f_open(FIL *file, const TCHAR *path, BYTE mode)
{
    (void)file;
    (void)mode;
    open_calls++;
    snprintf(last_open_path, sizeof(last_open_path), "%s", path);
    return FR_OK;
}

FRESULT f_write(FIL *file,
                const void *buffer,
                UINT bytes_to_write,
                UINT *bytes_written)
{
    (void)file;
    write_calls++;
    if (forced_write_result != FR_OK) {
        *bytes_written = 0U;
        return forced_write_result;
    }
    CHECK(bytes_to_write <= SD_LOGGER_STAGING_SIZE);
    memcpy(last_write, buffer, bytes_to_write);
    last_write[bytes_to_write] = '\0';
    *bytes_written = bytes_to_write;
    return FR_OK;
}

FRESULT f_sync(FIL *file)
{
    (void)file;
    sync_calls++;
    return FR_OK;
}

FRESULT f_close(FIL *file)
{
    (void)file;
    close_calls++;
    return FR_OK;
}

static payload_log_record_t make_record(uint64_t time_ms)
{
    const payload_log_record_t record = {
        .time_ms = time_ms,
        .accel_x_raw = -1,
        .accel_y_raw = 2,
        .accel_z_raw = 3,
        .uv_raw = 4U,
        .accel_fifo_skipped_total = 7U,
        .log_dropped_total = 0U,
        .accel_valid = 1U,
        .uv_valid = 1U,
        .uv_new = 1U,
        .sensor_error_mask = 0U,
        .pump_on = 1U,
    };
    return record;
}

static void reset_fakes(void)
{
    mount_calls = 0U;
    open_calls = 0U;
    write_calls = 0U;
    sync_calls = 0U;
    close_calls = 0U;
    forced_write_result = FR_OK;
    last_write[0] = '\0';
    last_open_path[0] = '\0';
}

static void test_format(void)
{
    const payload_log_record_t record = make_record(1234U);
    char line[160];
    const size_t length =
        payload_log_format_record(line, sizeof(line), &record);
    CHECK(length == strlen(line));
    CHECK(strcmp(line, "1234,-1,2,3,4,1,1,1,0,1,7,0\r\n") == 0);

    const payload_log_record_t wide_time =
        make_record(UINT64_C(18446744073709551615));
    const size_t wide_length =
        payload_log_format_record(line, sizeof(line), &wide_time);
    CHECK(wide_length == strlen(line));
    CHECK(strcmp(line,
                 "18446744073709551615,-1,2,3,4,1,1,1,0,1,7,0\r\n") == 0);

    char too_small[8];
    CHECK(payload_log_format_record(too_small,
                                    sizeof(too_small),
                                    &record) == 0U);
}

static void test_queue_and_service(void)
{
    reset_fakes();
    sd_logger_t logger;
    sd_logger_init(&logger, 0U);
    sd_logger_service(&logger, 0U);
    CHECK(sd_logger_status(&logger) == SD_LOGGER_READY);
    CHECK(mount_calls == 1U);
    CHECK(open_calls == 1U);
    CHECK(strcmp(last_open_path, "0:/LOG0000.CSV") == 0);
    CHECK(write_calls == 1U);
    CHECK(strcmp(last_write, PAYLOAD_LOG_CSV_HEADER) == 0);

    for (uint64_t index = 0U; index < 8U; index++) {
        const payload_log_record_t record = make_record(index);
        sd_logger_push(&logger, &record);
    }
    sd_logger_service(&logger, 10U);
    CHECK(logger.queue_count == 0U);
    CHECK(write_calls == 2U);
    CHECK(strncmp(last_write, "0,-1,2,3", 8U) == 0);

    for (uint64_t index = 0U;
         index < SD_LOGGER_QUEUE_CAPACITY + 1U;
         index++) {
        const payload_log_record_t record = make_record(index);
        sd_logger_push(&logger, &record);
    }
    CHECK(logger.queue_count == SD_LOGGER_QUEUE_CAPACITY);
    CHECK(sd_logger_dropped_records(&logger) == 1U);

    forced_write_result = FR_DISK_ERR;
    sd_logger_service(&logger, 20U);
    CHECK(sd_logger_status(&logger) == SD_LOGGER_ERROR);
    CHECK(!logger.file_open);
    CHECK(close_calls == 1U);
}

static void test_experiment_file_rollover(void)
{
    reset_fakes();
    sd_logger_t logger;
    sd_logger_init(&logger, 100U);
    sd_logger_service(&logger, 100U);
    CHECK(strcmp(last_open_path, "0:/LOG0000.CSV") == 0);

    const payload_log_record_t before_pump = make_record(110U);
    sd_logger_push(&logger, &before_pump);
    sd_logger_begin_experiment(&logger, 120U);
    CHECK(logger.queue_count == 0U);
    CHECK(logger.experiment_file);
    CHECK(!logger.file_open);
    CHECK(close_calls == 1U);

    sd_logger_service(&logger, 120U);
    CHECK(sd_logger_status(&logger) == SD_LOGGER_READY);
    CHECK(strcmp(last_open_path, "0:/EXP0000.CSV") == 0);
    CHECK(open_calls == 2U);
}

int main(void)
{
    test_format();
    test_queue_and_service();
    test_experiment_file_rollover();

    if (failures != 0) {
        fprintf(stderr, "%d SD logger test(s) failed\n", failures);
        return 1;
    }
    puts("SD logger tests passed");
    return 0;
}
