/*! @file rn2483.c
 *  @brief RN2483 UART driver for the STM32G0B1.
 */

#include "main.h"
#include "rn2483.h"

#include <stdio.h>
#include <string.h>

#define RN2483_RESPONSE_TIMEOUT_MS 2000U
#define RN2483_ASYNC_RESPONSE_TIMEOUT_MS 30000U


/* USART2 (PA2/PA3) is wired to the RN2483.  USART1 is reserved for debugging. */
static UART_HandleTypeDef *rn2483_uart = NULL;

void RN2483_Init(UART_HandleTypeDef *uart)
{
    rn2483_uart = uart;
}

static int read_byte(uint8_t *byte, uint32_t timeout_ms)
{
    if (rn2483_uart == NULL)
    {
        return RN2483_ERR_PARAM;
    }
    HAL_StatusTypeDef result;

    result = HAL_UART_Receive(rn2483_uart, byte, 1U, timeout_ms);

    switch (result)
    {
        case HAL_OK:
            return RN2483_SUCCESS;

        case HAL_TIMEOUT:
            return RN2483_ERR_TIMEOUT;

        case HAL_BUSY:
            return RN2483_ERR_BUSY;

        case HAL_ERROR:
        default:
            return RN2483_ERR_UART;
    }
}

static int write_command(const char *command)
{
    if (rn2483_uart == NULL)
    {
        return RN2483_ERR_PARAM;
    }

    size_t length = strlen(command);

    if (length == 0U || length >= RN2483_MAX_COMMAND)
    {
        return RN2483_ERR_PARAM;
    }

    HAL_StatusTypeDef result;

    result = HAL_UART_Transmit(rn2483_uart,
                               (uint8_t *)command,
                               (uint16_t)length,
                               RN2483_RESPONSE_TIMEOUT_MS);

    switch (result)
    {
        case HAL_OK:
            return RN2483_SUCCESS;

        case HAL_TIMEOUT:
            return RN2483_ERR_TIMEOUT;

        case HAL_BUSY:
            return RN2483_ERR_BUSY;

        case HAL_ERROR:
        default:
            return RN2483_ERR_UART;
    }
}

static int RN2483_response(char *response, uint32_t first_byte_timeout_ms)
{
    size_t index = 0U;
    uint8_t byte;
    uint32_t timeout_ms = first_byte_timeout_ms;

    if (response == NULL) {
        return RN2483_ERR_PARAM;
    }

    while (index < (RN2483_MAX_BUFF - 1U)) {
        
        int status = read_byte(&byte, timeout_ms);

        if (status != RN2483_SUCCESS)
        {
            response[index] = '\0';
            return status;
        }

        response[index++] = (char)byte;
        response[index] = '\0';
        if (byte == '\n') {
            return RN2483_SUCCESS;
        }
        timeout_ms = RN2483_RESPONSE_TIMEOUT_MS;
    }

    response[index] = '\0';
    return RN2483_EOB;
}

static int command_expect_ok(const char *command)
{
    char response[RN2483_MAX_BUFF];
    int status = RN2483_command(command, response);

    return (status == RN2483_SUCCESS && strcmp(response, "ok\r\n") == 0)
               ? RN2483_SUCCESS
               : (status == RN2483_SUCCESS ? RN2483_ERR_PANIC : status);
}

static bool configured(const char *value)
{
    return value != NULL && value[0] != '\0';
}

static int max_payload_length(void)
{
    if (strcmp(RN2483_DATA_RATE, "0") == 0 || strcmp(RN2483_DATA_RATE, "1") == 0 ||
        strcmp(RN2483_DATA_RATE, "2") == 0) {
        return 59;
    }
    if (strcmp(RN2483_DATA_RATE, "3") == 0) {
        return 123;
    }
    if (strcmp(RN2483_DATA_RATE, "4") == 0 || strcmp(RN2483_DATA_RATE, "5") == 0 ||
        strcmp(RN2483_DATA_RATE, "6") == 0 || strcmp(RN2483_DATA_RATE, "7") == 0) {
        return 230;
    }
    return 0;
}

int RN2483_reset(void)
{
    char response[RN2483_MAX_BUFF];
    return RN2483_command("sys reset\r\n", response);
}

int RN2483_autobaud(int baud)
{
    uint8_t sync = 0x55U;
    HAL_StatusTypeDef result;

    if (rn2483_uart == NULL || baud <= 0) {
        return RN2483_ERR_PARAM;
    }

    /* The RN2483 detects the new baud rate from a UART break followed by 0x55. */
    if (HAL_UART_DeInit(rn2483_uart) != HAL_OK) {
        return RN2483_ERR_UART;
    }
    rn2483_uart->Init.BaudRate = (uint32_t)baud;
    if (HAL_UART_Init(rn2483_uart) != HAL_OK) {
        return RN2483_ERR_UART;
    }
    if (HAL_LIN_SendBreak(rn2483_uart) != HAL_OK) {
        return RN2483_ERR_UART;
    }
    HAL_Delay(2U);
    result = HAL_UART_Transmit(rn2483_uart, &sync, 1U, RN2483_RESPONSE_TIMEOUT_MS);
    if (result == HAL_TIMEOUT) {
        return RN2483_ERR_TIMEOUT;
    }
    if (result != HAL_OK) {
        return result == HAL_BUSY ? RN2483_ERR_BUSY : RN2483_ERR_UART;
    }
    return RN2483_firmware((char[RN2483_MAX_BUFF]){0});
}

int RN2483_command(const char *command, char *response)
{
    size_t length;
    int status;

    if (command == NULL || response == NULL) {
        return RN2483_ERR_PARAM;
    }
    length = strlen(command);
    if (length < 2U || command[length - 2U] != '\r' || command[length - 1U] != '\n') {
        return RN2483_ERR_PARAM;
    }

    status = write_command(command);
    if (status != RN2483_SUCCESS) {
        return status;
    }
    return RN2483_response(response, RN2483_RESPONSE_TIMEOUT_MS);
}

int RN2483_firmware(char *buff)
{
    return RN2483_command("sys get ver\r\n", buff);
}

int RN2483_initMAC(void)
{
    int status;

    if (!configured(RN2483_FREQUENCY) || !configured(RN2483_DEV_ADDR) ||
        !configured(RN2483_DEV_EUI) || !configured(RN2483_APP_EUI) ||
        !configured(RN2483_APP_KEY) || !configured(RN2483_DATA_RATE)) {
        return RN2483_ERR_KIDS;
    }

    const char * const commands[] = {
        "mac reset " RN2483_FREQUENCY "\r\n",
        "mac set devaddr " RN2483_DEV_ADDR "\r\n",
        "mac set deveui " RN2483_DEV_EUI "\r\n",
        "mac set appeui " RN2483_APP_EUI "\r\n",
        "mac set appkey " RN2483_APP_KEY "\r\n",
        "mac set dr " RN2483_DATA_RATE "\r\n",
        "mac save\r\n",
    };

    for (size_t index = 0U; index < (sizeof(commands) / sizeof(commands[0])); ++index) {
        status = command_expect_ok(commands[index]);
        if (status != RN2483_SUCCESS) {
            return status;
        }
    }
    return RN2483_SUCCESS;
}

int RN2483_join(RN2483_JoinMode mode)
{
    char response[RN2483_MAX_BUFF];
    int status;

    if (mode == RN2483_OTAA) {
        status = RN2483_command("mac join otaa\r\n", response);
    } else if (mode == RN2483_ABP) {
        status = RN2483_command("mac join abp\r\n", response);
    } else {
        return RN2483_ERR_PARAM;
    }
    if (status != RN2483_SUCCESS) {
        return status;
    }
    if (strcmp(response, "ok\r\n") == 0) {
        status = RN2483_response(response, RN2483_ASYNC_RESPONSE_TIMEOUT_MS);
        if (status != RN2483_SUCCESS) {
            return status;
        }
        return strcmp(response, "accepted\r\n") == 0 ? RN2483_SUCCESS : RN2483_DENIED;
    }
    if (strcmp(response, "keys_not_init\r\n") == 0) {
        return RN2483_ERR_KIDS;
    }
    if (strcmp(response, "no_free_ch\r\n") == 0) {
        return RN2483_ERR_BUSY;
    }
    if (strcmp(response, "silent\r\n") == 0 || strcmp(response, "busy\r\n") == 0 ||
        strcmp(response, "mac_paused\r\n") == 0) {
        return RN2483_ERR_STATE;
    }
    return RN2483_ERR_PANIC;
}

int RN2483_tx(const char *buff, bool confirm, char *downlink)
{
    char response[RN2483_MAX_BUFF];
    char command[RN2483_MAX_COMMAND];
    size_t length;
    int max_length;
    int written;
    int status;

    if (buff == NULL || !configured(RN2483_PORT)) {
        return RN2483_ERR_PARAM;
    }
    length = strlen(buff);
    max_length = max_payload_length();
    if (max_length == 0 || length > (size_t)max_length) {
        return RN2483_ERR_PARAM;
    }

    written = snprintf(command, sizeof(command), "mac tx %s %s ",
                       confirm ? "cnf" : "uncnf", RN2483_PORT);
    if (written < 0 || (size_t)written + (length * 2U) + 3U > sizeof(command)) {
        return RN2483_ERR_PARAM;
    }
    for (size_t index = 0U; index < length; ++index) {
        (void)snprintf(&command[written + (index * 2U)], 3U, "%02X", (unsigned char)buff[index]);
    }
    (void)snprintf(&command[written + (length * 2U)], 3U, "\r\n");

    status = RN2483_command(command, response);
    if (status != RN2483_SUCCESS) {
        return status;
    }
    if (strcmp(response, "ok\r\n") != 0) {
        if (strcmp(response, "invalid_param\r\n") == 0) return RN2483_ERR_PARAM;
        if (strcmp(response, "no_free_ch\r\n") == 0) return RN2483_ERR_BUSY;
        if (strcmp(response, "not_joined\r\n") == 0 ||
            strcmp(response, "frame_counter_err_rejoin_needed\r\n") == 0) return RN2483_ERR_JOIN;
        if (strcmp(response, "silent\r\n") == 0 || strcmp(response, "busy\r\n") == 0 ||
            strcmp(response, "mac_paused\r\n") == 0) return RN2483_ERR_STATE;
        return RN2483_ERR_PANIC;
    }

    status = RN2483_response(response, RN2483_ASYNC_RESPONSE_TIMEOUT_MS);
    if (status != RN2483_SUCCESS) {
        return status;
    }
    if (strcmp(response, "mac_tx_ok\r\n") == 0) return RN2483_NODOWN;
    if (strcmp(response, "mac_err\r\n") == 0 || strcmp(response, "invalid_data_len\r\n") == 0) {
        return RN2483_ERR_PANIC;
    }
    if (downlink != NULL)
    {
        memcpy(downlink, response, strlen(response) + 1U);
    }

    return RN2483_SUCCESS;
}
