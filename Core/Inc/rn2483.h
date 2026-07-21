/**
 * @file rn2483.h
 * @brief STM32 HAL driver interface for the Microchip RN2483 LoRaWAN module.
 */

#ifndef RN2483_H
#define RN2483_H

#include "stm32g0xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
/*
 * Buffer size includes space for the terminating '\0'
 * added by the driver after receiving an RN2483 response.
 */

#define RN2483_MAX_BUFF 512U
#define RN2483_MAX_COMMAND 512U

/*
 * Credentials are empty by default, preventing secrets from being committed.
 * Override these macros through private build definitions before calling
 * RN2483_initMAC().  All values must be C string literals.
 */
#ifndef RN2483_FREQUENCY
#define RN2483_FREQUENCY ""
#endif
#ifndef RN2483_DEV_ADDR
#define RN2483_DEV_ADDR ""
#endif
#ifndef RN2483_DEV_EUI
#define RN2483_DEV_EUI ""
#endif
#ifndef RN2483_APP_EUI
#define RN2483_APP_EUI ""
#endif
#ifndef RN2483_APP_KEY
#define RN2483_APP_KEY ""
#endif
#ifndef RN2483_DATA_RATE
#define RN2483_DATA_RATE ""
#endif
#ifndef RN2483_PORT
#define RN2483_PORT ""
#endif

typedef enum {
    RN2483_SUCCESS = 0,
    RN2483_ERR_PARAM,
    RN2483_EOB,
    RN2483_ERR_KIDS,
    RN2483_ERR_BUSY,
    RN2483_ERR_STATE,
    RN2483_DENIED,
    RN2483_ERR_JOIN,
    RN2483_NODOWN,
    RN2483_ERR_PANIC,
    RN2483_ERR_TIMEOUT = -10,
    RN2483_ERR_UART = -11,
} RN2483_ReturnCode;


/** Bind the driver to the UART wired to the RN2483 (USART2 on this board). */
void RN2483_Init(UART_HandleTypeDef *uart);

/** Issue `sys reset`; the board has no dedicated RN2483 reset GPIO. */
int RN2483_reset(void);

/**
 * Change the UART speed using the RN2483 break + 0x55 autobaud sequence.
 * Reconfigures the UART passed to RN2483_Init().
 */
int RN2483_autobaud(int baud);

/**
 * Send a CRLF-terminated RN2483 command and read its first response line.
 * `response` must point to an array of at least RN2483_MAX_BUFF characters.
 */
int RN2483_command(const char *command, char *response);

/** Read the module firmware string into a RN2483_MAX_BUFF-character buffer. */
int RN2483_firmware(char *response);

/** Apply the configured LoRaWAN parameters and save them to the module. */
int RN2483_initMAC(void);
typedef enum {
    RN2483_OTAA = 0,
    RN2483_ABP,
} RN2483_JoinMode;

/** Join the configured network through OTAA or ABP. */
int RN2483_join(RN2483_JoinMode mode);

/**
 * Send an application payload.  `downlink`, when non-NULL, must point to an
 * array of at least RN2483_MAX_BUFF characters; it receives a `mac_rx` line.
 */
int RN2483_tx(const char *payload, bool confirm, char *downlink);

#endif /* RN2483_H */
