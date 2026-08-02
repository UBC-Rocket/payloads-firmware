#include "storage/sd_spi_diskio.h"

#include "diskio.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RESPONSE_CAPACITY 1024U

static int failures;
static uint8_t response_queue[RESPONSE_CAPACITY];
static size_t response_head;
static size_t response_tail;
static uint8_t command_bytes[6];
static size_t command_length;
static bool card_selected;
static bool write_active;
static size_t write_length;
static uint8_t written_sector[512];
static uint32_t fake_tick;
static uint32_t init_calls;
static uint32_t gpio_init_calls;
static uint32_t cmd0_count;
static uint32_t cmd0_failures_remaining;
static bool configured_mode3;
static bool require_mode3;
static bool spi_initialized_while_selected;
static uint32_t select_count;

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            fprintf(stderr, "check failed at %s:%d: %s\n",                  \
                    __FILE__, __LINE__, #condition);                          \
            failures++;                                                       \
        }                                                                     \
    } while (0)

static void queue_byte(uint8_t byte)
{
    CHECK(response_head < RESPONSE_CAPACITY);
    response_queue[response_head++] = byte;
}

static uint8_t dequeue_byte(void)
{
    return response_tail < response_head
        ? response_queue[response_tail++]
        : 0xFFU;
}

static void queue_data_block(const uint8_t *data, size_t length)
{
    queue_byte(0xFEU);
    for (size_t index = 0U; index < length; index++) {
        queue_byte(data[index]);
    }
    queue_byte(0xFFU);
    queue_byte(0xFFU);
}

static void handle_command(void)
{
    const uint8_t command = (uint8_t)(command_bytes[0] & 0x3FU);
    switch (command) {
    case 0U:
        cmd0_count++;
        if (require_mode3 && !configured_mode3) {
            queue_byte(0xFFU);
        } else if (cmd0_failures_remaining > 0U) {
            cmd0_failures_remaining--;
            queue_byte(0xFFU);
        } else {
            queue_byte(1U);
        }
        break;
    case 8U:
        queue_byte(1U);
        queue_byte(0U);
        queue_byte(0U);
        queue_byte(1U);
        queue_byte(0xAAU);
        break;
    case 9U: {
        uint8_t csd[16] = {0};
        csd[0] = 0x40U;
        csd[9] = 3U;
        queue_byte(0U);
        queue_data_block(csd, sizeof(csd));
        break;
    }
    case 17U: {
        uint8_t sector[512];
        for (size_t index = 0U; index < sizeof(sector); index++) {
            sector[index] = (uint8_t)index;
        }
        queue_byte(0U);
        queue_data_block(sector, sizeof(sector));
        break;
    }
    case 24U:
        queue_byte(0U);
        break;
    case 41U:
        queue_byte(0U);
        break;
    case 55U:
        queue_byte(1U);
        break;
    case 58U:
        queue_byte(0U);
        queue_byte(0x40U);
        queue_byte(0U);
        queue_byte(0U);
        queue_byte(0U);
        break;
    default:
        queue_byte(0U);
        break;
    }
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *spi)
{
    init_calls++;
    spi_initialized_while_selected |= card_selected;
    configured_mode3 =
        spi->Init.CLKPolarity == SPI_POLARITY_HIGH &&
        spi->Init.CLKPhase == SPI_PHASE_2EDGE;
    return HAL_OK;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port;
    (void)init;
    gpio_init_calls++;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port,
                       uint16_t pin,
                       GPIO_PinState state)
{
    (void)port;
    (void)pin;
    card_selected = state == GPIO_PIN_RESET;
    if (card_selected) {
        select_count++;
    }
    if (!card_selected) {
        command_length = 0U;
    }
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
                                          const uint8_t *tx_data,
                                          uint8_t *rx_data,
                                          uint16_t size,
                                          uint32_t timeout)
{
    (void)spi;
    (void)timeout;
    CHECK(size == 1U);

    uint8_t response = dequeue_byte();
    if (card_selected) {
        const uint8_t transmitted = tx_data[0];
        if (write_active) {
            if (write_length < sizeof(written_sector)) {
                written_sector[write_length] = transmitted;
            }
            write_length++;
            if (write_length == sizeof(written_sector) + 2U) {
                write_active = false;
                queue_byte(0x05U);
                queue_byte(0xFFU);
            }
        } else if (transmitted == 0xFEU && response_head == response_tail) {
            write_active = true;
            write_length = 0U;
        } else if (command_length > 0U ||
                   (transmitted & 0xC0U) == 0x40U) {
            command_bytes[command_length++] = transmitted;
            if (command_length == sizeof(command_bytes)) {
                command_length = 0U;
                handle_command();
            }
        }
    }
    *rx_data = response;
    return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
    return fake_tick++;
}

void HAL_Delay(uint32_t milliseconds)
{
    fake_tick += milliseconds;
}

static void reset_fake(void)
{
    response_head = 0U;
    response_tail = 0U;
    command_length = 0U;
    card_selected = false;
    write_active = false;
    write_length = 0U;
    fake_tick = 0U;
    init_calls = 0U;
    gpio_init_calls = 0U;
    cmd0_count = 0U;
    cmd0_failures_remaining = 0U;
    configured_mode3 = false;
    require_mode3 = false;
    spi_initialized_while_selected = false;
    select_count = 0U;
    memset(written_sector, 0, sizeof(written_sector));
}

int main(void)
{
    reset_fake();
    CHECK(!sd_spi_configure_slow());
    CHECK(sd_spi_last_error() == SD_SPI_ERROR_NOT_BOUND);

    SPI_HandleTypeDef spi = {0};
    GPIO_TypeDef port = {0};
    CHECK(sd_spi_diskio_bind(&spi, &port, 1U));
    card_selected = true;
    CHECK(sd_spi_configure_slow());
    CHECK(!spi_initialized_while_selected);
    CHECK(spi.Init.DataSize == SPI_DATASIZE_8BIT);
    CHECK(spi.Init.NSS == SPI_NSS_SOFT);
    CHECK(spi.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256);
    CHECK(gpio_init_calls == 1U);

    cmd0_failures_remaining = 2U;
    CHECK(disk_initialize(0U) == 0U);
    CHECK(cmd0_count == 3U);
    CHECK(select_count == 1U);
    CHECK(spi.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);
    CHECK(init_calls >= 3U);

    uint8_t read_buffer[512];
    CHECK(disk_read(0U, read_buffer, 2U, 1U) == RES_OK);
    for (size_t index = 0U; index < sizeof(read_buffer); index++) {
        CHECK(read_buffer[index] == (uint8_t)index);
    }

    uint8_t write_buffer[512];
    for (size_t index = 0U; index < sizeof(write_buffer); index++) {
        write_buffer[index] = (uint8_t)(255U - index);
    }
    CHECK(disk_write(0U, write_buffer, 3U, 1U) == RES_OK);
    CHECK(memcmp(write_buffer, written_sector, sizeof(write_buffer)) == 0);

    WORD sector_size = 0U;
    CHECK(disk_ioctl(0U, GET_SECTOR_SIZE, &sector_size) == RES_OK);
    CHECK(sector_size == 512U);
    DWORD sector_count = 0U;
    CHECK(disk_ioctl(0U, GET_SECTOR_COUNT, &sector_count) == RES_OK);
    CHECK(sector_count == 4096U);

    reset_fake();
    SPI_HandleTypeDef mode3_spi = {0};
    require_mode3 = true;
    CHECK(sd_spi_diskio_bind(&mode3_spi, &port, 1U));
    CHECK(disk_initialize(0U) == 0U);
    CHECK(cmd0_count > 1U);
    CHECK(configured_mode3);
    CHECK(mode3_spi.Init.CLKPolarity == SPI_POLARITY_HIGH);
    CHECK(mode3_spi.Init.CLKPhase == SPI_PHASE_2EDGE);
    CHECK(mode3_spi.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);

    if (failures != 0) {
        fprintf(stderr, "%d SD SPI test(s) failed\n", failures);
        return 1;
    }
    puts("SD SPI disk I/O tests passed");
    return 0;
}
