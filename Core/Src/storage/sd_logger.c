#include "storage/sd_logger.h"

#include <stdio.h>
#include <string.h>

#define SD_LOGGER_RETRY_MS 5000U
#define SD_LOGGER_SYNC_MS 1000U
#define SD_LOGGER_MAX_FILES 10000U
#define SD_LOGGER_MIN_BATCH_RECORDS 8U

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void mark_offline(sd_logger_t *logger,
                         uint32_t now_ms,
                         FRESULT result)
{
    logger->last_fatfs_result = result;
    logger->status = SD_LOGGER_ERROR;
    logger->write_errors++;
    if (logger->file_open) {
        (void)f_close(&logger->file);
        logger->file_open = false;
    }
    if (logger->mounted) {
        (void)f_mount(NULL, "0:", 1U);
        logger->mounted = false;
    }
    logger->retry_at_ms = now_ms + SD_LOGGER_RETRY_MS;
}

static bool mount_and_open(sd_logger_t *logger, uint32_t now_ms)
{
    logger->last_fatfs_result =
        f_mount(&logger->filesystem, "0:", 1U);
    if (logger->last_fatfs_result != FR_OK) {
        logger->mount_errors++;
        logger->status = SD_LOGGER_ERROR;
        logger->retry_at_ms = now_ms + SD_LOGGER_RETRY_MS;
        return false;
    }
    logger->mounted = true;

    char filename[20];
    bool opened = false;
    for (uint16_t index = 0U; index < SD_LOGGER_MAX_FILES; index++) {
        const int length =
            snprintf(filename, sizeof(filename), "0:/LOG%04u.CSV", index);
        if (length <= 0 || (size_t)length >= sizeof(filename)) {
            break;
        }
        logger->last_fatfs_result =
            f_open(&logger->file,
                   filename,
                   (BYTE)(FA_WRITE | FA_CREATE_NEW));
        if (logger->last_fatfs_result == FR_EXIST) {
            continue;
        }
        if (logger->last_fatfs_result == FR_OK) {
            logger->file_index = index;
            opened = true;
        }
        break;
    }

    if (!opened) {
        mark_offline(logger, now_ms, logger->last_fatfs_result);
        return false;
    }
    logger->file_open = true;

    UINT written = 0U;
    const size_t header_length = strlen(PAYLOAD_LOG_CSV_HEADER);
    logger->last_fatfs_result =
        f_write(&logger->file,
                PAYLOAD_LOG_CSV_HEADER,
                (UINT)header_length,
                &written);
    if (logger->last_fatfs_result != FR_OK ||
        written != (UINT)header_length) {
        mark_offline(logger, now_ms,
                     logger->last_fatfs_result == FR_OK
                         ? FR_DISK_ERR
                         : logger->last_fatfs_result);
        return false;
    }
    logger->last_fatfs_result = f_sync(&logger->file);
    if (logger->last_fatfs_result != FR_OK) {
        mark_offline(logger, now_ms, logger->last_fatfs_result);
        return false;
    }

    logger->next_sync_ms = now_ms + SD_LOGGER_SYNC_MS;
    logger->status = SD_LOGGER_READY;
    return true;
}

void sd_logger_init(sd_logger_t *logger, uint32_t now_ms)
{
    if (logger == NULL) {
        return;
    }
    memset(logger, 0, sizeof(*logger));
    logger->status = SD_LOGGER_OFFLINE;
    logger->retry_at_ms = now_ms;
}

void sd_logger_push(sd_logger_t *logger,
                    const payload_log_record_t *record)
{
    if (logger == NULL || record == NULL) {
        return;
    }

    if (logger->queue_count == SD_LOGGER_QUEUE_CAPACITY) {
        logger->queue_tail =
            (logger->queue_tail + 1U) % SD_LOGGER_QUEUE_CAPACITY;
        logger->queue_count--;
        logger->dropped_records++;
    }

    logger->queue[logger->queue_head] = *record;
    logger->queue[logger->queue_head].log_dropped_total =
        logger->dropped_records;
    logger->queue_head =
        (logger->queue_head + 1U) % SD_LOGGER_QUEUE_CAPACITY;
    logger->queue_count++;
}

static void write_queued_records(sd_logger_t *logger, uint32_t now_ms)
{
    size_t staging_length = 0U;
    size_t staged_records = 0U;
    while (staged_records < logger->queue_count) {
        const size_t queue_index =
            (logger->queue_tail + staged_records) %
            SD_LOGGER_QUEUE_CAPACITY;
        const size_t length =
            payload_log_format_record(
                &logger->staging[staging_length],
                sizeof(logger->staging) - staging_length,
                &logger->queue[queue_index]);
        if (length == 0U) {
            if (staged_records == 0U) {
                logger->queue_tail =
                    (logger->queue_tail + 1U) %
                    SD_LOGGER_QUEUE_CAPACITY;
                logger->queue_count--;
                logger->dropped_records++;
            }
            break;
        }
        staging_length += length;
        staged_records++;
    }
    if (staged_records == 0U) {
        return;
    }

    UINT written = 0U;
    logger->last_fatfs_result =
        f_write(&logger->file,
                logger->staging,
                (UINT)staging_length,
                &written);
    if (logger->last_fatfs_result != FR_OK ||
        written != (UINT)staging_length) {
        mark_offline(logger, now_ms,
                     logger->last_fatfs_result == FR_OK
                         ? FR_DISK_ERR
                         : logger->last_fatfs_result);
        return;
    }

    logger->queue_tail =
        (logger->queue_tail + staged_records) %
        SD_LOGGER_QUEUE_CAPACITY;
    logger->queue_count -= staged_records;
}

void sd_logger_service(sd_logger_t *logger, uint32_t now_ms)
{
    if (logger == NULL) {
        return;
    }
    if (!logger->file_open) {
        if (deadline_reached(now_ms, logger->retry_at_ms)) {
            (void)mount_and_open(logger, now_ms);
        }
        return;
    }

    const bool sync_due =
        deadline_reached(now_ms, logger->next_sync_ms);
    if (logger->queue_count >= SD_LOGGER_MIN_BATCH_RECORDS ||
        (logger->queue_count > 0U && sync_due)) {
        write_queued_records(logger, now_ms);
        if (!logger->file_open) {
            return;
        }
    }

    if (sync_due) {
        logger->last_fatfs_result = f_sync(&logger->file);
        if (logger->last_fatfs_result != FR_OK) {
            mark_offline(logger, now_ms, logger->last_fatfs_result);
            return;
        }
        logger->next_sync_ms = now_ms + SD_LOGGER_SYNC_MS;
    }
}

void sd_logger_close(sd_logger_t *logger)
{
    if (logger == NULL) {
        return;
    }
    if (logger->file_open) {
        (void)f_sync(&logger->file);
        (void)f_close(&logger->file);
        logger->file_open = false;
    }
    if (logger->mounted) {
        (void)f_mount(NULL, "0:", 1U);
        logger->mounted = false;
    }
    logger->status = SD_LOGGER_OFFLINE;
}

sd_logger_status_t sd_logger_status(const sd_logger_t *logger)
{
    return logger == NULL ? SD_LOGGER_ERROR : logger->status;
}

uint32_t sd_logger_dropped_records(const sd_logger_t *logger)
{
    return logger == NULL ? 0U : logger->dropped_records;
}
