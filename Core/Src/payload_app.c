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
#include <stdio.h>
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

#define UV_I2C_BUS_NUMBER 3U
#define UV_POLL_INTERVAL_MS 10U
#define UV_RETRY_INTERVAL_MS 1000U
#define ACCEL_POLL_INTERVAL_MS 20U
#define ACCEL_RETRY_INTERVAL_MS 1000U
#define ACCEL_SAMPLE_INTERVAL_MS 10U
#define SENSOR_MAX_CONSECUTIVE_ERRORS 3U
#define DEBUG_STATUS_INTERVAL_MS 1000U
#define DEBUG_UART_TIMEOUT_MS 50U
#define DEBUG_LINE_SIZE 320U
#define PING_REPLY_DELAY_MS 250U
#define UV_I2C_TIMEOUT_MS 50U

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
volatile uint8_t payload_led_pwm_percent;
volatile bool payload_radio_ready;

static I2C_HandleTypeDef *uv_i2c_handle;
static SPI_HandleTypeDef *payload_sd_spi;
static SPI_HandleTypeDef *payload_accel_spi;
static UART_HandleTypeDef *payload_radio_uart;
static UART_HandleTypeDef *payload_debug_uart;
static bmi088_accel_fifo_batch_t accel_batch;
static sd_logger_t logger;
static rn2483_t radio;
static rn2483_stm32_bus_t radio_bus;
static bool radio_initialized;
static uint32_t debug_next_status_ms;
static uint32_t radio_reported_invalid_packets;
static uint32_t ping_reply_at_ms;
static bool ping_reply_pending;
static uint32_t pump_bump_ends_at_ms;
static bool pump_bump_active;

static uint32_t uv_next_poll_ms;
static uint32_t uv_retry_at_ms;
static uint8_t uv_consecutive_errors;
static bool uv_address_ack;
static uint32_t uv_address_error;
static uint32_t uv_init_hal_error;
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

typedef struct {
    GPIO_TypeDef *port;
    uint16_t scl_pin;
    uint16_t sda_pin;
} uv_i2c_pins_t;

static const uv_i2c_pins_t uv_i2c_pins = {
    .port = GPIOB,
    .scl_pin = GPIO_PIN_3,
    .sda_pin = GPIO_PIN_4,
};

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void debug_transmit(const char *text)
{
    if (payload_debug_uart == NULL || text == NULL) {
        return;
    }

    const size_t length = strlen(text);
    if (length == 0U || length > UINT16_MAX) {
        return;
    }
    (void)HAL_UART_Transmit(payload_debug_uart,
                            (uint8_t *)text,
                            (uint16_t)length,
                            DEBUG_UART_TIMEOUT_MS);
}

static bool recover_uv_i2c_bus(void)
{
    if (uv_i2c_handle == NULL) {
        return false;
    }

    I2C_HandleTypeDef *i2c = uv_i2c_handle;
    const uv_i2c_pins_t *pins = &uv_i2c_pins;
    if (HAL_I2C_DeInit(i2c) != HAL_OK) {
        return false;
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pins->scl_pin | pins->sda_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(pins->port, gpio.Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(pins->port, &gpio);
    HAL_Delay(1U);

    /* A slave can hold SDA low if the MCU reset in the middle of a byte.
       Nine SCL pulses finish that byte, then the transitions below issue a
       STOP and return both lines to their idle-high state. */
    for (uint32_t pulse = 0U; pulse < 9U; pulse++) {
        HAL_GPIO_WritePin(pins->port, pins->scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(pins->port, pins->scl_pin, GPIO_PIN_SET);
        HAL_Delay(1U);
    }
    HAL_GPIO_WritePin(pins->port, pins->scl_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pins->port, pins->sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(pins->port, pins->scl_pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(pins->port, pins->sda_pin, GPIO_PIN_SET);
    HAL_Delay(1U);

    const bool lines_released =
        HAL_GPIO_ReadPin(pins->port, pins->scl_pin) == GPIO_PIN_SET &&
        HAL_GPIO_ReadPin(pins->port, pins->sda_pin) == GPIO_PIN_SET;

    if (HAL_I2C_Init(i2c) != HAL_OK ||
        HAL_I2CEx_ConfigAnalogFilter(i2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK ||
        HAL_I2CEx_ConfigDigitalFilter(i2c, 0U) != HAL_OK) {
        return false;
    }
    return lines_released;
}

static void debug_radio_status(uint32_t now_ms, bool force)
{
    if (!force && !deadline_reached(now_ms, debug_next_status_ms)) {
        return;
    }
    debug_next_status_ms = now_ms + DEBUG_STATUS_INTERVAL_MS;

    char line[DEBUG_LINE_SIZE];
    const int length = snprintf(
        line,
        sizeof(line),
        "RADIO ready=%u phase=%u wait=%u lines=%lu valid=%lu "
        "badpkt=%lu badline=%lu timeout=%lu uart=%lu tx=%lu txerr=%lu "
        "pump=%u led=%u last=\"%s\"\r\n",
        payload_radio_ready ? 1U : 0U,
        (unsigned int)radio.phase,
        radio.waiting_for_reply ? 1U : 0U,
        (unsigned long)radio.stats.received_lines,
        (unsigned long)radio.stats.valid_commands,
        (unsigned long)radio.stats.invalid_packets,
        (unsigned long)radio.stats.invalid_lines,
        (unsigned long)radio.stats.response_timeouts,
        (unsigned long)radio.stats.transport_errors,
        (unsigned long)radio.stats.transmitted_packets,
        (unsigned long)radio.stats.transmit_failures,
        payload_pump_on ? 1U : 0U,
        payload_led_pwm_percent,
        radio.stats.received_lines == 0U ? "-" : radio.last_line);
    if (length > 0 && (size_t)length < sizeof(line)) {
        debug_transmit(line);
    }
}

static void debug_uv_sample(uint32_t now_ms, uint32_t raw)
{
    char line[DEBUG_LINE_SIZE];
    const int length = snprintf(
        line,
        sizeof(line),
        "UV sample time_ms=%lu raw=%lu raw_valid=1 uvi_valid=0 "
        "count=%lu errors=%lu bus=%u\r\n",
        (unsigned long)now_ms,
        (unsigned long)raw,
        (unsigned long)uv_sample_count,
        (unsigned long)uv_error_count,
        (unsigned int)uv_i2c_bus_number);
    if (length > 0 && (size_t)length < sizeof(line)) {
        debug_transmit(line);
    }
}

static void debug_uv_read_error(uint32_t now_ms,
                                ltr390_status_t status,
                                uint8_t consecutive_errors)
{
    const uint32_t hal_error = uv_i2c_handle == NULL
                                   ? HAL_I2C_ERROR_NONE
                                   : HAL_I2C_GetError(uv_i2c_handle);
    char line[DEBUG_LINE_SIZE];
    const int length = snprintf(
        line,
        sizeof(line),
        "UV read time_ms=%lu status=%u hal=%lu consecutive=%u "
        "errors=%lu bus=%u\r\n",
        (unsigned long)now_ms,
        (unsigned int)status,
        (unsigned long)hal_error,
        (unsigned int)consecutive_errors,
        (unsigned long)uv_error_count,
        (unsigned int)uv_i2c_bus_number);
    if (length > 0 && (size_t)length < sizeof(line)) {
        debug_transmit(line);
    }
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

static void set_led_pwm(uint8_t percent)
{
    UVLED_SetDutyPercent(percent);
    payload_led_pwm_percent = UVLED_GetDutyPercent();
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

    uv_address_ack = false;
    uv_address_error = HAL_I2C_ERROR_NONE;
    uv_init_hal_error = HAL_I2C_ERROR_NONE;

    if (uv_i2c_handle != NULL) {
        HAL_StatusTypeDef address_status =
            HAL_I2C_IsDeviceReady(
                uv_i2c_handle,
                (uint16_t)(LTR390_I2C_ADDRESS << 1U),
                1U,
                UV_I2C_TIMEOUT_MS);
        uv_address_error = HAL_I2C_GetError(uv_i2c_handle);

        if (address_status != HAL_OK &&
            (uv_address_error &
             (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_TIMEOUT)) != 0U &&
            recover_uv_i2c_bus()) {
            address_status =
                HAL_I2C_IsDeviceReady(
                    uv_i2c_handle,
                    (uint16_t)(LTR390_I2C_ADDRESS << 1U),
                    1U,
                    UV_I2C_TIMEOUT_MS);
            uv_address_error = HAL_I2C_GetError(uv_i2c_handle);
        }

        uv_address_ack = address_status == HAL_OK;
        if (uv_address_ack) {
            status = ltr390_stm32_bind(&huv,
                                       &huv_bus,
                                       uv_i2c_handle,
                                       UV_I2C_TIMEOUT_MS);
            if (status == LTR390_OK) {
                status = ltr390_init(&huv, &ltr390_default_uvs_config);
            }
            uv_init_hal_error = HAL_I2C_GetError(uv_i2c_handle);
        }
    }

    if (status == LTR390_OK) {
        uv_i2c_bus_number = UV_I2C_BUS_NUMBER;
    }

    uv_last_status = status;
    uv_consecutive_errors = 0U;
    if (status != LTR390_OK) {
        uv_error_count++;
        uv_retry_at_ms = now_ms + UV_RETRY_INTERVAL_MS;
    } else {
        uv_next_poll_ms = now_ms;
    }

    char line[DEBUG_LINE_SIZE];
    const int length = snprintf(
        line,
        sizeof(line),
        "UV init status=%u bus=%u ack=%u:%lu init_hal=%lu lines=%u%u\r\n",
        (unsigned int)status,
        (unsigned int)uv_i2c_bus_number,
        uv_address_ack ? 1U : 0U,
        (unsigned long)uv_address_error,
        (unsigned long)uv_init_hal_error,
        HAL_GPIO_ReadPin(uv_i2c_pins.port,
                         uv_i2c_pins.scl_pin) == GPIO_PIN_SET ? 1U : 0U,
        HAL_GPIO_ReadPin(uv_i2c_pins.port,
                         uv_i2c_pins.sda_pin) == GPIO_PIN_SET ? 1U : 0U);
    if (length > 0 && (size_t)length < sizeof(line)) {
        debug_transmit(line);
    }
}

static void process_radio(uint32_t now_ms)
{
    if (!radio_initialized) {
        payload_radio_ready = false;
        return;
    }

    if (ping_reply_pending) {
        if (!deadline_reached(now_ms, ping_reply_at_ms)) {
            payload_radio_ready = false;
            return;
        }
        ping_reply_pending = false;
        if (rn2483_send_text(&radio, "PONG")) {
            debug_transmit("EVENT PING reply=PONG queued\r\n");
        } else {
            debug_transmit("EVENT PING reply=PONG failed\r\n");
        }
    }

    rn2483_process(&radio, now_ms);
    if (radio.stats.invalid_packets != radio_reported_invalid_packets) {
        char line[DEBUG_LINE_SIZE];
        const int length = snprintf(
            line,
            sizeof(line),
            "BADPKT count=%lu raw=\"%s\"\r\n",
            (unsigned long)radio.stats.invalid_packets,
            radio.last_line);
        if (length > 0 && (size_t)length < sizeof(line)) {
            debug_transmit(line);
        }
        radio_reported_invalid_packets = radio.stats.invalid_packets;
    }

    rn2483_event_t event;
    while ((event = rn2483_take_event(&radio)) != RN2483_EVENT_NONE) {
        if (event == RN2483_EVENT_PUMP_ON) {
            pump_bump_active = false;
            const bool pump_was_on = payload_pump_on;
            set_pump(true);
            if (!pump_was_on) {
                sd_logger_begin_experiment(&logger, now_ms);
                debug_transmit(
                    "EVENT PUMP_ON applied pump=1 log=EXP\r\n");
            } else {
                debug_transmit(
                    "EVENT PUMP_ON applied pump=1 log=unchanged\r\n");
            }
        } else if (event == RN2483_EVENT_PUMP_OFF) {
            pump_bump_active = false;
            set_pump(false);
            debug_transmit("EVENT PUMP_OFF applied pump=0\r\n");
        } else if (event == RN2483_EVENT_LED_ON) {
            set_led_pwm(100U);
            debug_transmit("EVENT LED_ON applied led=100\r\n");
        } else if (event == RN2483_EVENT_LED_OFF) {
            set_led_pwm(0U);
            debug_transmit("EVENT LED_OFF applied led=0\r\n");
        } else if (event == RN2483_EVENT_BUMP) {
            const uint32_t seconds = rn2483_bump_seconds(&radio);
            const bool pump_was_on = payload_pump_on;
            set_pump(true);
            pump_bump_ends_at_ms = now_ms + (seconds * 1000U);
            pump_bump_active = true;
            if (!pump_was_on) {
                sd_logger_begin_experiment(&logger, now_ms);
            }

            char line[DEBUG_LINE_SIZE];
            const int length = snprintf(
                line,
                sizeof(line),
                "EVENT BUMP applied pump=1 seconds=%lu log=%s\r\n",
                (unsigned long)seconds,
                pump_was_on ? "unchanged" : "EXP");
            if (length > 0 && (size_t)length < sizeof(line)) {
                debug_transmit(line);
            }
        } else if (event == RN2483_EVENT_PING) {
            /* Hold the RN2483 idle briefly so the bridge has time to switch
               from transmit completion into receive mode before PONG starts. */
            ping_reply_at_ms = now_ms + PING_REPLY_DELAY_MS;
            ping_reply_pending = true;
            debug_transmit("EVENT PING received reply=PONG pending\r\n");
        }
    }

    payload_radio_ready = rn2483_is_ready(&radio);
}

static void process_pump_bump(uint32_t now_ms)
{
    if (!pump_bump_active ||
        !deadline_reached(now_ms, pump_bump_ends_at_ms)) {
        return;
    }

    pump_bump_active = false;
    set_pump(false);
    debug_transmit("EVENT BUMP complete pump=0\r\n");
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
        ltr390_uvs_sample_t sample = {
            .raw = 0U,
            .uvi = 0.0f,
            .uvi_valid = false,
        };
        /* Log the sensor count without inventing an optical-window factor.
           UVI can only be produced after the assembled payload is calibrated. */
        status = ltr390_read_raw(&huv, &sample.raw);
        if (status == LTR390_OK) {
            uv_latest_sample = sample;
            uv_sample_count++;
            uv_valid = true;
            uv_new = true;
            debug_uv_sample(now_ms, sample.raw);
        }
    }

    uv_last_status = status;
    if (status == LTR390_OK) {
        uv_consecutive_errors = 0U;
    } else {
        uv_error_count++;
        uv_consecutive_errors++;
        debug_uv_read_error(now_ms, status, uv_consecutive_errors);
        if (uv_consecutive_errors >= SENSOR_MAX_CONSECUTIVE_ERRORS) {
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

void payload_app_init(I2C_HandleTypeDef *i2c3,
                      SPI_HandleTypeDef *sd_spi,
                      SPI_HandleTypeDef *accel_spi,
                      UART_HandleTypeDef *radio_uart,
                      UART_HandleTypeDef *debug_uart)
{
    uv_i2c_handle = i2c3;
    payload_sd_spi = sd_spi;
    payload_accel_spi = accel_spi;
    payload_radio_uart = radio_uart;
    payload_debug_uart = debug_uart;

    const uint32_t now_ms = HAL_GetTick();
    last_hal_tick = now_ms;
    tick_epoch = 0U;
    synthetic_next_ms = now_ms;
    uv_next_poll_ms = now_ms;
    radio_reported_invalid_packets = 0U;
    ping_reply_at_ms = 0U;
    ping_reply_pending = false;
    pump_bump_ends_at_ms = 0U;
    pump_bump_active = false;
    set_pump(false);
    set_led_pwm(0U);

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
    bool autobaud_ok = false;
    if (radio_initialized) {
        autobaud_ok = rn2483_stm32_autobaud(&radio_bus,
                                            GPIOA,
                                            GPIO_PIN_2,
                                            GPIO_AF1_USART2);
        if (!autobaud_ok) {
            radio.stats.transport_errors++;
            (void)rn2483_stm32_rearm_receive(&radio_bus);
        }
    }

    char boot_line[DEBUG_LINE_SIZE];
    const int boot_length = snprintf(
        boot_line,
        sizeof(boot_line),
        "BOOT freq=%lu sf=%u bw=%u cr=4/%u sync=%02X bind=%u autobaud=%u\r\n",
        (unsigned long)radio_config.frequency_hz,
        radio_config.spreading_factor,
        radio_config.bandwidth_khz,
        radio_config.coding_rate_denominator,
        radio_config.sync_word,
        (unsigned int)radio_status,
        autobaud_ok ? 1U : 0U);
    if (boot_length > 0 && (size_t)boot_length < sizeof(boot_line)) {
        debug_transmit(boot_line);
    }
    debug_radio_status(now_ms, true);
}

void payload_app_process(void)
{
    const uint64_t now_extended_ms = monotonic_ms();
    const uint32_t now_ms = (uint32_t)now_extended_ms;

    process_radio(now_ms);
    process_pump_bump(now_ms);
    debug_radio_status(now_ms, false);
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
