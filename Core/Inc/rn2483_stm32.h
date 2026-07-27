/**
 * @file rn2483_stm32.h
 * @brief STM32 HAL UART adapter for the RN2483 raw-radio driver.
 */

#ifndef RN2483_STM32_H
#define RN2483_STM32_H

#include "rn2483.h"
#include "stm32g0xx_hal.h"

typedef struct {
    rn2483_t *device;
    UART_HandleTypeDef *uart;
    uint8_t rx_byte;
} rn2483_stm32_bus_t;

rn2483_status_t rn2483_stm32_bind(rn2483_t *device,
                                  rn2483_stm32_bus_t *bus,
                                  UART_HandleTypeDef *uart,
                                  const rn2483_raw_config_t *config,
                                  uint32_t now_ms);

/**
 * Hold the module RX line low long enough to force auto-baud, then send 0x55
 * at the UART rate configured by Payloads.ioc.
 */
bool rn2483_stm32_autobaud(rn2483_stm32_bus_t *bus,
                           GPIO_TypeDef *tx_port,
                           uint16_t tx_pin,
                           uint32_t tx_alternate);

void rn2483_stm32_rx_complete(rn2483_stm32_bus_t *bus,
                              UART_HandleTypeDef *uart);
bool rn2483_stm32_rearm_receive(rn2483_stm32_bus_t *bus);

#endif /* RN2483_STM32_H */
