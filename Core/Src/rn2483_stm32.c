#include "rn2483_stm32.h"

#define RN2483_UART_TIMEOUT_MS 50U
#define RN2483_BREAK_MS 2U

static bool stm32_transmit(void *context,
                           const uint8_t *data,
                           size_t length)
{
    rn2483_stm32_bus_t *bus = (rn2483_stm32_bus_t *)context;
    if (bus == NULL || bus->uart == NULL || data == NULL ||
        length == 0U || length > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(bus->uart,
                             (uint8_t *)data,
                             (uint16_t)length,
                             RN2483_UART_TIMEOUT_MS) == HAL_OK;
}

static bool arm_receive(rn2483_stm32_bus_t *bus)
{
    return HAL_UART_Receive_IT(bus->uart, &bus->rx_byte, 1U) == HAL_OK;
}

bool rn2483_stm32_rearm_receive(rn2483_stm32_bus_t *bus)
{
    if (bus == NULL || bus->uart == NULL) {
        return false;
    }
    (void)HAL_UART_AbortReceive(bus->uart);
    return arm_receive(bus);
}

rn2483_status_t rn2483_stm32_bind(rn2483_t *device,
                                  rn2483_stm32_bus_t *bus,
                                  UART_HandleTypeDef *uart,
                                  const rn2483_raw_config_t *config,
                                  uint32_t now_ms)
{
    if (device == NULL || bus == NULL || uart == NULL) {
        return RN2483_ERROR_ARGUMENT;
    }

    bus->device = device;
    bus->uart = uart;
    bus->rx_byte = 0U;
    const rn2483_transport_t transport = {
        .transmit = stm32_transmit,
        .context = bus,
    };
    const rn2483_status_t status =
        rn2483_init(device, &transport, config, now_ms);
    if (status != RN2483_OK) {
        return status;
    }

    return arm_receive(bus) ? RN2483_OK : RN2483_ERROR_TRANSPORT;
}

bool rn2483_stm32_autobaud(rn2483_stm32_bus_t *bus,
                           GPIO_TypeDef *tx_port,
                           uint16_t tx_pin,
                           uint32_t tx_alternate)
{
    if (bus == NULL || bus->uart == NULL || tx_port == NULL ||
        tx_pin == 0U) {
        return false;
    }

    (void)HAL_UART_AbortReceive(bus->uart);
    __HAL_UART_DISABLE(bus->uart);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = tx_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(tx_port, &gpio);
    HAL_GPIO_WritePin(tx_port, tx_pin, GPIO_PIN_RESET);
    HAL_Delay(RN2483_BREAK_MS);
    HAL_GPIO_WritePin(tx_port, tx_pin, GPIO_PIN_SET);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = tx_alternate;
    HAL_GPIO_Init(tx_port, &gpio);
    __HAL_UART_ENABLE(bus->uart);
    HAL_Delay(1U);

    uint8_t sync = 0x55U;
    const bool synchronized =
        HAL_UART_Transmit(bus->uart,
                          &sync,
                          1U,
                          RN2483_UART_TIMEOUT_MS) == HAL_OK;
    return synchronized && arm_receive(bus);
}

void rn2483_stm32_rx_complete(rn2483_stm32_bus_t *bus,
                              UART_HandleTypeDef *uart)
{
    if (bus == NULL || uart != bus->uart || bus->device == NULL) {
        return;
    }

    rn2483_on_rx_byte(bus->device, bus->rx_byte);
    if (!arm_receive(bus)) {
        bus->device->stats.transport_errors++;
    }
}
