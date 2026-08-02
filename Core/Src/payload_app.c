#include "payload_app.h"

#include "accel/bmi088_accel.h"
#include "accel/bmi088_accel_stm32.h"
#include "main.h"
#include "rn2483.h"
#include "rn2483_stm32.h"
#include "storage/payload_log.h"
#include "storage/sd_logger.h"
#include "storage/sd_spi_diskio.h"
#include "uv/ltr390.h"
#include "uv/ltr390_stm32.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef RN2483_RADIO_FREQ_HZ
#error RN2483_RADIO_FREQ_HZ must be supplied by CMake
#endif
#ifndef RN2483_RADIO_SF
#error RN2483_RADIO_SF must be supplied by CMake
#endif
#ifndef RN2483_RADIO_BW_KHZ
#error RN2483_RADIO_BW_KHZ must be supplied by CMake
#endif
#ifndef RN2483_RADIO_CR
#error RN2483_RADIO_CR must be supplied by CMake
#endif
#ifndef RN2483_RADIO_SYNC_WORD
#error RN2483_RADIO_SYNC_WORD must be supplied by CMake
#endif

#define UV_I2C_BUS_COUNT 3U
#define UV_POLL_INTERVAL_MS 10U
#define UV_RETRY_INTERVAL_MS 1000U
#define UV_WINDOW_FACTOR 1.0f
#define ACCEL_POLL_INTERVAL_MS 20U
#define ACCEL_RETRY_INTERVAL_MS 1000U
#define ACCEL_SAMPLE_INTERVAL_MS 10U
#define SENSOR_MAX_CONSECUTIVE_ERRORS 3U

#define SENSOR_ERROR_ACCEL 0x01U
#define SENSOR_ERROR_UV 0x02U

bmi088_accel_t haccel;
bmi088_accel_stm32_bus_t haccel_bus;
volatile bmi088_accel_sample_t accel_latest_sample;
volatile bmi088_accel_status_t accel_last_status =
    BMI088_ACCEL_ERROR_NOT_INITIALIZED;
volatile uint32_t accel_sample_count;
volatile uint32_t accel_error_count;
volatile uint32_t accel_fifo_skipped_total;

ltr390_t huv;
ltr390_stm32_bus_t huv_bus;
volatile ltr390_uvs_sample_t uv_latest_sample;
volatile ltr390_status_t uv_last_status = LTR390_ERROR_NOT_INITIALIZED;
volatile uint32_t uv_sample_count;
volatile uint32_t uv_error_count;
volatile uint8_t uv_i2c_bus_number;

volatile sd_logger_status_t payload_sd_status = SD_LOGGER_OFFLINE;
volatile uint32_t payload_log_dropped_count;
volatile bool payload_pump_on;
volatile bool payload_radio_ready;

static I2C_HandleTypeDef *uv_i2c_handles[UV_I2C_BUS_COUNT];
static SPI_HandleTypeDef *payload_sd_spi;
static SPI_HandleTypeDef *payload_accel_spi;
static UART_HandleTypeDef *payload_radio_uart;
static bmi088_accel_fifo_batch_t accel_batch;
static sd_logger_t logger;
static rn2483_t radio;
static rn2483_stm32_bus_t radio_bus;
static bool radio_initialized;

static uint32_t uv_next_poll_ms;
static uint32_t uv_retry_at_ms;
static uint8_t uv_consecutive_errors;
static bool uv_valid;
static bool uv_new;
static uint32_t accel_next_poll_ms;
static uint32_t accel_retry_at_ms;
static uint8_t accel_consecutive_errors;
static bool accel_fifo_ready;
static uint32_t last_hal_tick;
static uint64_t tick_epoch;
static uint64_t last_accel_timestamp_ms;
static bool accel_timeline_valid;
static uint64_t synthetic_next_ms;

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static uint64_t monotonic_ms(void)
{
    const uint32_t tick = HAL_GetTick();
    if (tick < last_hal_tick) {
        tick_epoch += (UINT64_C(1) << 32U);
    }
    last_hal_tick = tick;
    return tick_epoch + tick;
}

static void set_pump(bool on)
{
    HAL_GPIO_WritePin(PUMP_CTRL_GPIO_Port,
                      PUMP_CTRL_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    payload_pump_on = on;
}

static uint8_t sensor_error_mask(void)
{
    uint8_t mask = accel_fifo_ready ? 0U : SENSOR_ERROR_ACCEL;
    if (!huv.initialized) {
        mask |= SENSOR_ERROR_UV;
    }
    return mask;
}

static void push_record(uint64_t timestamp_ms,
                        const bmi088_accel_sample_t *sample,
                        bool accel_valid,
                        bool new_uv)
{
    payload_log_record_t record;
    memset(&record, 0, sizeof(record));
    record.time_ms = timestamp_ms;
    if (sample != NULL) {
        record.accel_x_raw = sample->x;
        record.accel_y_raw = sample->y;
        record.accel_z_raw = sample->z;
    }
    record.uv_raw = uv_latest_sample.raw;
    record.accel_fifo_skipped_total = accel_fifo_skipped_total;
    record.accel_valid = accel_valid ? 1U : 0U;
    record.uv_valid = uv_valid ? 1U : 0U;
    record.uv_new = new_uv ? 1U : 0U;
    record.sensor_error_mask = sensor_error_mask();
    record.pump_on = payload_pump_on ? 1U : 0U;
    sd_logger_push(&logger, &record);
}

static void initialize_accelerometer(uint32_t now_ms)
{
    accel_last_status =
        bmi088_accel_stm32_bind(&haccel,
                                &haccel_bus,
                                payload_accel_spi,
                                ACC_CS_GPIO_Port,
                                ACC_CS_Pin,
                                10U);
    if (accel_last_status == BMI088_ACCEL_OK) {
        accel_last_status =
            bmi088_accel_init(&haccel, &bmi088_accel_default_config);
    }
    if (accel_last_status == BMI088_ACCEL_OK) {
        accel_last_status = bmi088_accel_fifo_enable(&haccel);
    }

    accel_fifo_ready = accel_last_status == BMI088_ACCEL_OK;
    accel_consecutive_errors = 0U;
    if (!accel_fifo_ready) {
        accel_error_count++;
        accel_retry_at_ms = now_ms + ACCEL_RETRY_INTERVAL_MS;
    } else {
        accel_next_poll_ms = now_ms + ACCEL_POLL_INTERVAL_MS;
        accel_timeline_valid = false;
    }
}

static void initialize_uv_sensor(uint32_t now_ms)
{
    ltr390_status_t status = LTR390_ERROR_COMMUNICATION;
    huv.initialized = false;
    uv_i2c_bus_number = 0U;
    uv_valid = false;

    for (size_t index = 0U; index < UV_I2C_BUS_COUNT; index++) {
        status = ltr390_stm32_bind(&huv,
                                   &huv_bus,
                                   uv_i2c_handles[index],
                                   10U);
        if (status == LTR390_OK) {
            status = ltr390_init(&huv, &ltr390_default_uvs_config);
        }
        if (status == LTR390_OK) {
            uv_i2c_bus_number = (uint8_t)(index + 1U);
            break;
        }
    }

    uv_last_status = status;
    uv_consecutive_errors = 0U;
    if (status != LTR390_OK) {
        uv_error_count++;
        uv_retry_at_ms = now_ms + UV_RETRY_INTERVAL_MS;
    }
}

static void process_radio(uint32_t now_ms)
{
    if (!radio_initialized) {
        payload_radio_ready = false;
        return;
    }

    rn2483_process(&radio, now_ms);
    rn2483_event_t event;
    while ((event = rn2483_take_event(&radio)) != RN2483_EVENT_NONE) {
        if (event == RN2483_EVENT_PUMP_ON) {
            set_pump(true);
        } else if (event == RN2483_EVENT_PUMP_OFF) {
            set_pump(false);
        }
    }
    payload_radio_ready = rn2483_is_ready(&radio);
}

static void process_uv(uint32_t now_ms)
{
    if (!huv.initialized &&
        deadline_reached(now_ms, uv_retry_at_ms)) {
        initialize_uv_sensor(now_ms);
    }

    if (!huv.initialized ||
        !deadline_reached(now_ms, uv_next_poll_ms)) {
        return;
    }
    uv_next_poll_ms = now_ms + UV_POLL_INTERVAL_MS;

    bool ready = false;
    ltr390_status_t status = ltr390_data_ready(&huv, &ready);
    if (status == LTR390_OK && ready) {
        ltr390_uvs_sample_t sample;
        status = ltr390_read_uvs(&huv, UV_WINDOW_FACTOR, &sample);
        if (status == LTR390_OK) {
            uv_latest_sample = sample;
            uv_sample_count++;
            uv_valid = true;
            uv_new = true;
        }
    }

    uv_last_status = status;
    if (status == LTR390_OK) {
        uv_consecutive_errors = 0U;
    } else {
        uv_error_count++;
        if (++uv_consecutive_errors >= SENSOR_MAX_CONSECUTIVE_ERRORS) {
            huv.initialized = false;
            uv_i2c_bus_number = 0U;
            uv_valid = false;
            uv_retry_at_ms = now_ms + UV_RETRY_INTERVAL_MS;
        }
    }
}

static void log_accelerometer_batch(uint64_t now_ms)
{
    if (accel_batch.sample_count == 0U) {
        return;
    }

    const uint64_t batch_span_ms =
        (accel_batch.sample_count - 1U) * ACCEL_SAMPLE_INTERVAL_MS;
    uint64_t first_timestamp =
        now_ms >= batch_span_ms ? now_ms - batch_span_ms : 0U;
    if (accel_timeline_valid &&
        first_timestamp <= last_accel_timestamp_ms) {
        first_timestamp =
            last_accel_timestamp_ms + ACCEL_SAMPLE_INTERVAL_MS;
    }

    for (size_t index = 0U;
         index < accel_batch.sample_count;
         index++) {
        const uint64_t timestamp =
            first_timestamp + (index * ACCEL_SAMPLE_INTERVAL_MS);
        const bool record_has_new_uv =
            index == (accel_batch.sample_count - 1U) && uv_new;
        push_record(timestamp,
                    &accel_batch.samples[index],
                    true,
                    record_has_new_uv);
        accel_latest_sample = accel_batch.samples[index];
        accel_sample_count++;
        last_accel_timestamp_ms = timestamp;
    }
    uv_new = false;
    accel_timeline_valid = true;
    synthetic_next_ms =
        last_accel_timestamp_ms + ACCEL_SAMPLE_INTERVAL_MS;
}

static void process_accelerometer(uint32_t now_ms, uint64_t now_extended_ms)
{
    if (!accel_fifo_ready) {
        if (deadline_reached(now_ms, accel_retry_at_ms)) {
            initialize_accelerometer(now_ms);
        }
        unsigned int emitted = 0U;
        while (!accel_fifo_ready &&
               now_extended_ms >= synthetic_next_ms &&
               emitted < 4U) {
            push_record(synthetic_next_ms, NULL, false, uv_new);
            uv_new = false;
            synthetic_next_ms += ACCEL_SAMPLE_INTERVAL_MS;
            emitted++;
        }
        return;
    }

    if (!deadline_reached(now_ms, accel_next_poll_ms)) {
        return;
    }
    accel_next_poll_ms = now_ms + ACCEL_POLL_INTERVAL_MS;

    accel_last_status = bmi088_accel_fifo_read(&haccel, &accel_batch);
    accel_fifo_skipped_total +=
        (uint32_t)accel_batch.skipped_frames +
        (uint32_t)accel_batch.sample_drop_frames;
    if (accel_last_status == BMI088_ACCEL_OK ||
        accel_last_status == BMI088_ACCEL_ERROR_DATA) {
        if (accel_last_status == BMI088_ACCEL_ERROR_DATA) {
            accel_error_count++;
        }
        accel_consecutive_errors = 0U;
        log_accelerometer_batch(now_extended_ms);
    } else {
        accel_error_count++;
        if (++accel_consecutive_errors >=
            SENSOR_MAX_CONSECUTIVE_ERRORS) {
            accel_fifo_ready = false;
            haccel.initialized = false;
            accel_retry_at_ms = now_ms + ACCEL_RETRY_INTERVAL_MS;
            synthetic_next_ms = now_extended_ms;
        }
    }
}

void payload_app_init(I2C_HandleTypeDef *i2c1,
                      I2C_HandleTypeDef *i2c2,
                      I2C_HandleTypeDef *i2c3,
                      SPI_HandleTypeDef *sd_spi,
                      SPI_HandleTypeDef *accel_spi,
                      UART_HandleTypeDef *radio_uart)
{
    uv_i2c_handles[0] = i2c1;
    uv_i2c_handles[1] = i2c2;
    uv_i2c_handles[2] = i2c3;
    payload_sd_spi = sd_spi;
    payload_accel_spi = accel_spi;
    payload_radio_uart = radio_uart;

    const uint32_t now_ms = HAL_GetTick();
    last_hal_tick = now_ms;
    tick_epoch = 0U;
    synthetic_next_ms = now_ms;
    uv_next_poll_ms = now_ms;
    set_pump(false);

    initialize_accelerometer(now_ms);
    initialize_uv_sensor(now_ms);

    (void)sd_spi_diskio_bind(payload_sd_spi,
                             SD_CS_GPIO_Port,
                             SD_CS_Pin);
    sd_logger_init(&logger, now_ms);

    const rn2483_raw_config_t radio_config = {
        .frequency_hz = RN2483_RADIO_FREQ_HZ,
        .spreading_factor = RN2483_RADIO_SF,
        .bandwidth_khz = RN2483_RADIO_BW_KHZ,
        .coding_rate_denominator = RN2483_RADIO_CR,
        .sync_word = RN2483_RADIO_SYNC_WORD,
    };
    HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    const rn2483_status_t radio_status =
        rn2483_stm32_bind(&radio,
                          &radio_bus,
                          payload_radio_uart,
                          &radio_config,
                          now_ms);
    radio_initialized =
        radio_status == RN2483_OK ||
        radio_status == RN2483_ERROR_TRANSPORT;
    if (radio_initialized) {
        if (!rn2483_stm32_autobaud(&radio_bus,
                                   GPIOA,
                                   GPIO_PIN_2,
                                   GPIO_AF1_USART2)) {
            radio.stats.transport_errors++;
            (void)rn2483_stm32_rearm_receive(&radio_bus);
        }
    }
}

void payload_app_process(void)
{
    const uint64_t now_extended_ms = monotonic_ms();
    const uint32_t now_ms = (uint32_t)now_extended_ms;

    process_radio(now_ms);
    process_uv(now_ms);
    process_accelerometer(now_ms, now_extended_ms);
    sd_logger_service(&logger, now_ms);

    payload_sd_status = sd_logger_status(&logger);
    payload_log_dropped_count = sd_logger_dropped_records(&logger);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    rn2483_stm32_rx_complete(&radio_bus, uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (radio_initialized && uart == payload_radio_uart) {
        radio.stats.transport_errors++;
        (void)rn2483_stm32_rearm_receive(&radio_bus);
    }
}
