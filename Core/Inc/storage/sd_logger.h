/**
 * @file sd_logger.h
 * @brief Buffered, recoverable CSV logger backed by FatFs.
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "ff.h"
#include "storage/payload_log.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SD_LOGGER_QUEUE_CAPACITY 256U
#define SD_LOGGER_STAGING_SIZE 1024U

typedef enum {
    SD_LOGGER_OFFLINE = 0,
    SD_LOGGER_READY,
    SD_LOGGER_ERROR
} sd_logger_status_t;

typedef struct {
    FATFS filesystem;
    FIL file;
    payload_log_record_t queue[SD_LOGGER_QUEUE_CAPACITY];
    char staging[SD_LOGGER_STAGING_SIZE];
    size_t queue_head;
    size_t queue_tail;
    size_t queue_count;
    uint32_t dropped_records;
    uint32_t write_errors;
    uint32_t mount_errors;
    uint32_t retry_at_ms;
    uint32_t next_sync_ms;
    uint16_t file_index;
    FRESULT last_fatfs_result;
    sd_logger_status_t status;
    bool mounted;
    bool file_open;
    bool experiment_file;
} sd_logger_t;

void sd_logger_init(sd_logger_t *logger, uint32_t now_ms);
void sd_logger_push(sd_logger_t *logger,
                    const payload_log_record_t *record);
void sd_logger_begin_experiment(sd_logger_t *logger, uint32_t now_ms);
void sd_logger_service(sd_logger_t *logger, uint32_t now_ms);
void sd_logger_close(sd_logger_t *logger);

sd_logger_status_t sd_logger_status(const sd_logger_t *logger);
uint32_t sd_logger_dropped_records(const sd_logger_t *logger);

#endif /* SD_LOGGER_H */
