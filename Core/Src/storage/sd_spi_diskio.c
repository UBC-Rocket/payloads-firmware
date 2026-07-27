#include "storage/sd_spi_diskio.h"

#include "diskio.h"

#include <stddef.h>

#define SD_SPI_DRIVE 0U
#define SD_SPI_SECTOR_SIZE 512U
#define SD_SPI_INITIALIZATION_TIMEOUT_MS 1500U
#define SD_SPI_READY_TIMEOUT_MS 500U
#define SD_SPI_TOKEN_TIMEOUT_MS 300U
#define SD_SPI_HAL_TIMEOUT_MS 20U

#define SD_CARD_TYPE_MMC 0x01U
#define SD_CARD_TYPE_SD1 0x02U
#define SD_CARD_TYPE_SD2 0x04U
#define SD_CARD_TYPE_BLOCK 0x08U

#define SD_CMD0 0U
#define SD_CMD1 1U
#define SD_CMD8 8U
#define SD_CMD9 9U
#define SD_CMD10 10U
#define SD_CMD12 12U
#define SD_CMD16 16U
#define SD_CMD17 17U
#define SD_CMD24 24U
#define SD_CMD41 (0x80U + 41U)
#define SD_CMD55 55U
#define SD_CMD58 58U

#define SD_DATA_TOKEN 0xFEU
#define SD_WRITE_ACCEPTED 0x05U

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *chip_select_port;
    uint16_t chip_select_pin;
    DSTATUS status;
    uint8_t card_type;
    sd_spi_error_t last_error;
    bool bound;
} sd_spi_context_t;

static sd_spi_context_t sd_context = {
    .status = STA_NOINIT,
    .last_error = SD_SPI_ERROR_NOT_BOUND,
};

static bool elapsed(uint32_t started_ms, uint32_t timeout_ms)
{
    return (uint32_t)(HAL_GetTick() - started_ms) >= timeout_ms;
}

static bool configure_spi(uint32_t prescaler)
{
    if (!sd_context.bound || sd_context.spi == NULL) {
        sd_context.last_error = SD_SPI_ERROR_NOT_BOUND;
        return false;
    }

    SPI_HandleTypeDef *spi = sd_context.spi;
    spi->Init.Mode = SPI_MODE_MASTER;
    spi->Init.Direction = SPI_DIRECTION_2LINES;
    spi->Init.DataSize = SPI_DATASIZE_8BIT;
    spi->Init.CLKPolarity = SPI_POLARITY_LOW;
    spi->Init.CLKPhase = SPI_PHASE_1EDGE;
    spi->Init.NSS = SPI_NSS_SOFT;
    spi->Init.BaudRatePrescaler = prescaler;
    spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi->Init.TIMode = SPI_TIMODE_DISABLE;
    spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi->Init.CRCPolynomial = 7U;
    spi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    spi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(spi) != HAL_OK) {
        sd_context.last_error = SD_SPI_ERROR_HAL;
        return false;
    }
    return true;
}

bool sd_spi_diskio_bind(SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *chip_select_port,
                        uint16_t chip_select_pin)
{
    if (spi == NULL || chip_select_port == NULL || chip_select_pin == 0U) {
        return false;
    }

    sd_context.spi = spi;
    sd_context.chip_select_port = chip_select_port;
    sd_context.chip_select_pin = chip_select_pin;
    sd_context.status = STA_NOINIT;
    sd_context.card_type = 0U;
    sd_context.last_error = SD_SPI_ERROR_NONE;
    sd_context.bound = true;
    return true;
}

bool sd_spi_configure_slow(void)
{
    if (!configure_spi(SPI_BAUDRATEPRESCALER_256)) {
        return false;
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = sd_context.chip_select_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(sd_context.chip_select_port, &gpio);
    HAL_GPIO_WritePin(sd_context.chip_select_port,
                      sd_context.chip_select_pin,
                      GPIO_PIN_SET);
    return true;
}

bool sd_spi_set_data_speed(void)
{
    return configure_spi(SPI_BAUDRATEPRESCALER_8);
}

sd_spi_error_t sd_spi_last_error(void)
{
    return sd_context.last_error;
}

static uint8_t exchange_byte(uint8_t transmitted)
{
    uint8_t received = 0xFFU;
    if (HAL_SPI_TransmitReceive(sd_context.spi,
                               &transmitted,
                               &received,
                               1U,
                               SD_SPI_HAL_TIMEOUT_MS) != HAL_OK) {
        sd_context.last_error = SD_SPI_ERROR_HAL;
        return 0xFFU;
    }
    return received;
}

static void deselect_card(void)
{
    HAL_GPIO_WritePin(sd_context.chip_select_port,
                      sd_context.chip_select_pin,
                      GPIO_PIN_SET);
    (void)exchange_byte(0xFFU);
}

static bool wait_ready(uint32_t timeout_ms)
{
    const uint32_t started_ms = HAL_GetTick();
    do {
        if (exchange_byte(0xFFU) == 0xFFU) {
            return true;
        }
    } while (!elapsed(started_ms, timeout_ms));

    sd_context.last_error = SD_SPI_ERROR_TIMEOUT;
    return false;
}

static bool select_card(void)
{
    HAL_GPIO_WritePin(sd_context.chip_select_port,
                      sd_context.chip_select_pin,
                      GPIO_PIN_RESET);
    (void)exchange_byte(0xFFU);
    if (wait_ready(SD_SPI_READY_TIMEOUT_MS)) {
        return true;
    }
    deselect_card();
    return false;
}

static uint8_t send_command(uint8_t command, uint32_t argument)
{
    if ((command & 0x80U) != 0U) {
        command &= 0x7FU;
        const uint8_t response = send_command(SD_CMD55, 0U);
        if (response > 1U) {
            return response;
        }
    }

    if (command != SD_CMD12) {
        deselect_card();
        if (!select_card()) {
            return 0xFFU;
        }
    }

    (void)exchange_byte((uint8_t)(0x40U | command));
    (void)exchange_byte((uint8_t)(argument >> 24U));
    (void)exchange_byte((uint8_t)(argument >> 16U));
    (void)exchange_byte((uint8_t)(argument >> 8U));
    (void)exchange_byte((uint8_t)argument);

    uint8_t crc = 0x01U;
    if (command == SD_CMD0) {
        crc = 0x95U;
    } else if (command == SD_CMD8) {
        crc = 0x87U;
    }
    (void)exchange_byte(crc);
    if (command == SD_CMD12) {
        (void)exchange_byte(0xFFU);
    }

    for (uint8_t attempts = 0U; attempts < 10U; attempts++) {
        const uint8_t response = exchange_byte(0xFFU);
        if ((response & 0x80U) == 0U) {
            return response;
        }
    }
    sd_context.last_error = SD_SPI_ERROR_PROTOCOL;
    return 0xFFU;
}

static bool receive_data_block(uint8_t *buffer, size_t length)
{
    const uint32_t started_ms = HAL_GetTick();
    uint8_t token;
    do {
        token = exchange_byte(0xFFU);
    } while (token == 0xFFU &&
             !elapsed(started_ms, SD_SPI_TOKEN_TIMEOUT_MS));

    if (token != SD_DATA_TOKEN) {
        sd_context.last_error = token == 0xFFU
            ? SD_SPI_ERROR_TIMEOUT
            : SD_SPI_ERROR_PROTOCOL;
        return false;
    }

    for (size_t index = 0U; index < length; index++) {
        buffer[index] = exchange_byte(0xFFU);
    }
    (void)exchange_byte(0xFFU);
    (void)exchange_byte(0xFFU);
    return true;
}

static bool transmit_data_block(const uint8_t *buffer, uint8_t token)
{
    if (!wait_ready(SD_SPI_READY_TIMEOUT_MS)) {
        return false;
    }
    (void)exchange_byte(token);
    if (token == 0xFDU) {
        return true;
    }

    for (size_t index = 0U; index < SD_SPI_SECTOR_SIZE; index++) {
        (void)exchange_byte(buffer[index]);
    }
    (void)exchange_byte(0xFFU);
    (void)exchange_byte(0xFFU);
    if ((exchange_byte(0xFFU) & 0x1FU) != SD_WRITE_ACCEPTED) {
        sd_context.last_error = SD_SPI_ERROR_PROTOCOL;
        return false;
    }
    return wait_ready(SD_SPI_READY_TIMEOUT_MS);
}

DSTATUS disk_initialize(BYTE physical_drive)
{
    if (physical_drive != SD_SPI_DRIVE || !sd_context.bound) {
        return STA_NOINIT;
    }
    sd_context.status = STA_NOINIT;
    sd_context.card_type = 0U;
    sd_context.last_error = SD_SPI_ERROR_NONE;
    if (!sd_spi_configure_slow()) {
        return sd_context.status;
    }

    deselect_card();
    for (uint8_t clocks = 0U; clocks < 10U; clocks++) {
        (void)exchange_byte(0xFFU);
    }

    uint8_t type = 0U;
    if (send_command(SD_CMD0, 0U) == 1U) {
        const uint32_t started_ms = HAL_GetTick();
        if (send_command(SD_CMD8, 0x1AAU) == 1U) {
            uint8_t ocr[4];
            for (size_t index = 0U; index < sizeof(ocr); index++) {
                ocr[index] = exchange_byte(0xFFU);
            }
            if (ocr[2] == 0x01U && ocr[3] == 0xAAU) {
                while (!elapsed(started_ms,
                                SD_SPI_INITIALIZATION_TIMEOUT_MS) &&
                       send_command(SD_CMD41, 1UL << 30U) != 0U) {
                }
                if (!elapsed(started_ms,
                             SD_SPI_INITIALIZATION_TIMEOUT_MS) &&
                    send_command(SD_CMD58, 0U) == 0U) {
                    for (size_t index = 0U; index < sizeof(ocr); index++) {
                        ocr[index] = exchange_byte(0xFFU);
                    }
                    type = (uint8_t)(SD_CARD_TYPE_SD2 |
                        ((ocr[0] & 0x40U) != 0U
                            ? SD_CARD_TYPE_BLOCK
                            : 0U));
                }
            }
        } else {
            uint8_t initialization_command;
            if (send_command(SD_CMD41, 0U) <= 1U) {
                type = SD_CARD_TYPE_SD1;
                initialization_command = SD_CMD41;
            } else {
                type = SD_CARD_TYPE_MMC;
                initialization_command = SD_CMD1;
            }
            while (!elapsed(started_ms,
                            SD_SPI_INITIALIZATION_TIMEOUT_MS) &&
                   send_command(initialization_command, 0U) != 0U) {
            }
            if (elapsed(started_ms, SD_SPI_INITIALIZATION_TIMEOUT_MS) ||
                send_command(SD_CMD16, SD_SPI_SECTOR_SIZE) != 0U) {
                type = 0U;
            }
        }
    }

    sd_context.card_type = type;
    deselect_card();
    if (type != 0U && sd_spi_set_data_speed()) {
        sd_context.status &= (DSTATUS)~STA_NOINIT;
    } else if (sd_context.last_error == SD_SPI_ERROR_NONE) {
        sd_context.last_error = SD_SPI_ERROR_PROTOCOL;
    }
    return sd_context.status;
}

DSTATUS disk_status(BYTE physical_drive)
{
    return physical_drive == SD_SPI_DRIVE
        ? sd_context.status
        : STA_NOINIT;
}

DRESULT disk_read(BYTE physical_drive,
                  BYTE *buffer,
                  DWORD sector,
                  UINT count)
{
    if (physical_drive != SD_SPI_DRIVE || buffer == NULL || count == 0U) {
        return RES_PARERR;
    }
    if ((sd_context.status & STA_NOINIT) != 0U) {
        return RES_NOTRDY;
    }
    if ((sd_context.card_type & SD_CARD_TYPE_BLOCK) == 0U) {
        sector *= SD_SPI_SECTOR_SIZE;
    }

    while (count > 0U) {
        if (send_command(SD_CMD17, sector) != 0U ||
            !receive_data_block(buffer, SD_SPI_SECTOR_SIZE)) {
            deselect_card();
            return RES_ERROR;
        }
        deselect_card();
        buffer += SD_SPI_SECTOR_SIZE;
        sector += (sd_context.card_type & SD_CARD_TYPE_BLOCK) != 0U
            ? 1U
            : SD_SPI_SECTOR_SIZE;
        count--;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE physical_drive,
                   const BYTE *buffer,
                   DWORD sector,
                   UINT count)
{
    if (physical_drive != SD_SPI_DRIVE || buffer == NULL || count == 0U) {
        return RES_PARERR;
    }
    if ((sd_context.status & STA_NOINIT) != 0U) {
        return RES_NOTRDY;
    }
    if ((sd_context.card_type & SD_CARD_TYPE_BLOCK) == 0U) {
        sector *= SD_SPI_SECTOR_SIZE;
    }

    while (count > 0U) {
        if (send_command(SD_CMD24, sector) != 0U ||
            !transmit_data_block(buffer, SD_DATA_TOKEN)) {
            deselect_card();
            return RES_ERROR;
        }
        deselect_card();
        buffer += SD_SPI_SECTOR_SIZE;
        sector += (sd_context.card_type & SD_CARD_TYPE_BLOCK) != 0U
            ? 1U
            : SD_SPI_SECTOR_SIZE;
        count--;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE physical_drive, BYTE command, void *buffer)
{
    if (physical_drive != SD_SPI_DRIVE) {
        return RES_PARERR;
    }
    if ((sd_context.status & STA_NOINIT) != 0U) {
        return RES_NOTRDY;
    }

    DRESULT result = RES_ERROR;
    uint8_t csd[16];
    switch (command) {
    case CTRL_SYNC:
        if (select_card() && wait_ready(SD_SPI_READY_TIMEOUT_MS)) {
            result = RES_OK;
        }
        break;
    case GET_SECTOR_COUNT:
        if (buffer != NULL && send_command(SD_CMD9, 0U) == 0U &&
            receive_data_block(csd, sizeof(csd))) {
            if ((csd[0] >> 6U) == 1U) {
                const DWORD size =
                    (DWORD)csd[9] |
                    ((DWORD)csd[8] << 8U) |
                    ((DWORD)(csd[7] & 0x3FU) << 16U);
                *(DWORD *)buffer = (size + 1U) << 10U;
            } else {
                const uint8_t shift =
                    (uint8_t)((csd[5] & 0x0FU) +
                              ((csd[10] & 0x80U) >> 7U) +
                              ((csd[9] & 0x03U) << 1U) + 2U);
                const DWORD size =
                    (DWORD)((csd[8] >> 6U) |
                            ((WORD)csd[7] << 2U) |
                            ((WORD)(csd[6] & 0x03U) << 10U)) + 1U;
                *(DWORD *)buffer = size << (shift - 9U);
            }
            result = RES_OK;
        }
        break;
    case GET_SECTOR_SIZE:
        if (buffer != NULL) {
            *(WORD *)buffer = SD_SPI_SECTOR_SIZE;
            result = RES_OK;
        }
        break;
    case GET_BLOCK_SIZE:
        if (buffer != NULL) {
            *(DWORD *)buffer = 1U;
            result = RES_OK;
        }
        break;
    case MMC_GET_TYPE:
        if (buffer != NULL) {
            *(BYTE *)buffer = sd_context.card_type;
            result = RES_OK;
        }
        break;
    case MMC_GET_CSD:
        if (buffer != NULL && send_command(SD_CMD9, 0U) == 0U &&
            receive_data_block((uint8_t *)buffer, 16U)) {
            result = RES_OK;
        }
        break;
    case MMC_GET_CID:
        if (buffer != NULL && send_command(SD_CMD10, 0U) == 0U &&
            receive_data_block((uint8_t *)buffer, 16U)) {
            result = RES_OK;
        }
        break;
    case MMC_GET_OCR:
        if (buffer != NULL && send_command(SD_CMD58, 0U) == 0U) {
            for (size_t index = 0U; index < 4U; index++) {
                ((uint8_t *)buffer)[index] = exchange_byte(0xFFU);
            }
            result = RES_OK;
        }
        break;
    default:
        result = RES_PARERR;
        break;
    }

    deselect_card();
    return result;
}

DWORD get_fattime(void)
{
    return 0U;
}
