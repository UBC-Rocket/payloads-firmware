#include "rn2483.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RN2483_RESPONSE_TIMEOUT_MS 1000U
#define RN2483_RETRY_DELAY_MS 1000U
#define RN2483_CONFIGURATION_STEP_COUNT 22U

typedef enum {
    EXPECT_EXACT = 0,
    EXPECT_PREFIX,
    EXPECT_NONZERO_NUMBER
} expected_reply_t;

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool config_valid(const rn2483_raw_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    const bool frequency_valid =
        (config->frequency_hz >= 433050000U &&
         config->frequency_hz <= 434790000U) ||
        (config->frequency_hz >= 863000000U &&
         config->frequency_hz <= 870000000U);
    const bool bandwidth_valid =
        config->bandwidth_khz == 125U ||
        config->bandwidth_khz == 250U ||
        config->bandwidth_khz == 500U;

    return frequency_valid &&
           config->spreading_factor >= 7U &&
           config->spreading_factor <= 12U &&
           bandwidth_valid &&
           config->coding_rate_denominator >= 5U &&
           config->coding_rate_denominator <= 8U;
}

static void enter_backoff(rn2483_t *device, uint32_t now_ms)
{
    device->phase = RN2483_PHASE_BACKOFF;
    device->waiting_for_reply = false;
    device->ready = false;
    device->retry_at_ms = now_ms + RN2483_RETRY_DELAY_MS;
    device->configuration_step = 0U;
    device->stats.restarts++;
}

static bool transmit_command(rn2483_t *device,
                             const char *command,
                             uint32_t now_ms)
{
    const size_t length = strlen(command);
    if (length == 0U || length >= RN2483_COMMAND_SIZE ||
        !device->transport.transmit(device->transport.context,
                                    (const uint8_t *)command,
                                    length)) {
        device->stats.transport_errors++;
        enter_backoff(device, now_ms);
        return false;
    }

    device->waiting_for_reply = true;
    device->deadline_ms = now_ms + RN2483_RESPONSE_TIMEOUT_MS;
    return true;
}

static bool build_configuration_command(const rn2483_t *device,
                                        uint8_t step,
                                        char *command,
                                        size_t command_size,
                                        char *expected,
                                        size_t expected_size,
                                        expected_reply_t *expected_type)
{
    int command_length = -1;
    int expected_length = -1;
    *expected_type = EXPECT_EXACT;

    switch (step) {
    case 0U:
        command_length = snprintf(command, command_size, "sys get ver\r\n");
        expected_length = snprintf(expected, expected_size, "RN2483 ");
        *expected_type = EXPECT_PREFIX;
        break;
    case 1U:
        command_length = snprintf(command, command_size, "mac pause\r\n");
        expected[0] = '\0';
        expected_length = 0;
        *expected_type = EXPECT_NONZERO_NUMBER;
        break;
    case 2U:
        command_length = snprintf(command, command_size,
                                  "radio set mod lora\r\n");
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 3U:
        command_length = snprintf(command, command_size,
                                  "radio set freq %lu\r\n",
                                  (unsigned long)device->config.frequency_hz);
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 4U:
        command_length = snprintf(command, command_size,
                                  "radio set sf sf%u\r\n",
                                  device->config.spreading_factor);
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 5U:
        command_length = snprintf(command, command_size,
                                  "radio set bw %u\r\n",
                                  device->config.bandwidth_khz);
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 6U:
        command_length = snprintf(command, command_size,
                                  "radio set cr 4/%u\r\n",
                                  device->config.coding_rate_denominator);
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 7U:
        command_length = snprintf(command, command_size,
                                  "radio set crc on\r\n");
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 8U:
        command_length = snprintf(command, command_size,
                                  "radio set iqi off\r\n");
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 9U:
        command_length = snprintf(command, command_size,
                                  "radio set prlen 8\r\n");
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 10U:
        command_length = snprintf(command, command_size,
                                  "radio set sync %02X\r\n",
                                  device->config.sync_word);
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 11U:
        command_length = snprintf(command, command_size,
                                  "radio set wdt 0\r\n");
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 12U:
        command_length = snprintf(command, command_size, "radio get mod\r\n");
        expected_length = snprintf(expected, expected_size, "lora");
        break;
    case 13U:
        command_length = snprintf(command, command_size, "radio get freq\r\n");
        expected_length = snprintf(expected, expected_size, "%lu",
                                   (unsigned long)device->config.frequency_hz);
        break;
    case 14U:
        command_length = snprintf(command, command_size, "radio get sf\r\n");
        expected_length = snprintf(expected, expected_size, "sf%u",
                                   device->config.spreading_factor);
        break;
    case 15U:
        command_length = snprintf(command, command_size, "radio get bw\r\n");
        expected_length = snprintf(expected, expected_size, "%u",
                                   device->config.bandwidth_khz);
        break;
    case 16U:
        command_length = snprintf(command, command_size, "radio get cr\r\n");
        expected_length = snprintf(expected, expected_size, "4/%u",
                                   device->config.coding_rate_denominator);
        break;
    case 17U:
        command_length = snprintf(command, command_size, "radio get crc\r\n");
        expected_length = snprintf(expected, expected_size, "on");
        break;
    case 18U:
        command_length = snprintf(command, command_size, "radio get iqi\r\n");
        expected_length = snprintf(expected, expected_size, "off");
        break;
    case 19U:
        command_length = snprintf(command, command_size, "radio get prlen\r\n");
        expected_length = snprintf(expected, expected_size, "8");
        break;
    case 20U:
        command_length = snprintf(command, command_size, "radio get sync\r\n");
        expected_length = snprintf(expected, expected_size, "%02X",
                                   device->config.sync_word);
        break;
    case 21U:
        command_length = snprintf(command, command_size, "radio get wdt\r\n");
        expected_length = snprintf(expected, expected_size, "0");
        break;
    default:
        return false;
    }

    return command_length >= 0 &&
           (size_t)command_length < command_size &&
           expected_length >= 0 &&
           (size_t)expected_length < expected_size;
}

static bool strings_equal_case_insensitive(const char *left,
                                           const char *right)
{
    while (*left != '\0' && *right != '\0') {
        char left_char = *left;
        char right_char = *right;
        if (left_char >= 'a' && left_char <= 'z') {
            left_char = (char)(left_char - ('a' - 'A'));
        }
        if (right_char >= 'a' && right_char <= 'z') {
            right_char = (char)(right_char - ('a' - 'A'));
        }
        if (left_char != right_char) {
            return false;
        }
        left++;
        right++;
    }
    return *left == '\0' && *right == '\0';
}

static bool configuration_reply_matches(const rn2483_t *device,
                                        const char *line)
{
    char command[RN2483_COMMAND_SIZE];
    char expected[RN2483_LINE_SIZE];
    expected_reply_t expected_type;
    if (!build_configuration_command(device,
                                     device->configuration_step,
                                     command,
                                     sizeof(command),
                                     expected,
                                     sizeof(expected),
                                     &expected_type)) {
        return false;
    }

    if (expected_type == EXPECT_PREFIX) {
        return strncmp(line, expected, strlen(expected)) == 0;
    }
    if (expected_type == EXPECT_NONZERO_NUMBER) {
        char *end = NULL;
        const unsigned long value = strtoul(line, &end, 10);
        return end != line && *end == '\0' && value != 0UL;
    }
    return strings_equal_case_insensitive(line, expected);
}

static void handle_complete_line(rn2483_t *device,
                                 const char *line,
                                 uint32_t now_ms)
{
    device->stats.received_lines++;

    if (device->phase == RN2483_PHASE_LISTENING) {
        if (strcmp(line, "radio_rx 50554D505F4F4E") == 0) {
            device->pending_event = RN2483_EVENT_PUMP_ON;
            device->stats.valid_commands++;
        } else if (strcmp(line, "radio_rx 50554D505F4F4646") == 0) {
            device->pending_event = RN2483_EVENT_PUMP_OFF;
            device->stats.valid_commands++;
        } else if (strncmp(line, "radio_rx ", 9U) == 0) {
            device->stats.invalid_packets++;
        } else if (strcmp(line, "radio_err") != 0) {
            device->stats.invalid_lines++;
            return;
        }

        device->ready = false;
        device->phase = RN2483_PHASE_ARM_RECEIVER;
        device->waiting_for_reply = false;
        return;
    }

    if (!device->waiting_for_reply) {
        device->stats.invalid_lines++;
        return;
    }

    if (device->phase == RN2483_PHASE_CONFIGURE) {
        if (!configuration_reply_matches(device, line)) {
            device->stats.invalid_lines++;
            enter_backoff(device, now_ms);
            return;
        }
        device->configuration_step++;
        device->waiting_for_reply = false;
        if (device->configuration_step >=
            RN2483_CONFIGURATION_STEP_COUNT) {
            device->phase = RN2483_PHASE_ARM_RECEIVER;
        }
    } else if (device->phase == RN2483_PHASE_ARM_RECEIVER) {
        if (strcmp(line, "ok") != 0) {
            device->stats.invalid_lines++;
            enter_backoff(device, now_ms);
            return;
        }
        device->waiting_for_reply = false;
        device->phase = RN2483_PHASE_LISTENING;
        device->ready = true;
    }
}

static void consume_received_bytes(rn2483_t *device, uint32_t now_ms)
{
    while (device->rx_tail != device->rx_head) {
        const uint8_t byte = device->rx_ring[device->rx_tail];
        device->rx_tail =
            (uint16_t)((device->rx_tail + 1U) % RN2483_RX_RING_SIZE);

        if (byte == '\n') {
            if (!device->discarding_line && device->line_length > 0U) {
                if (device->line[device->line_length - 1U] == '\r') {
                    device->line_length--;
                }
                device->line[device->line_length] = '\0';
                handle_complete_line(device, device->line, now_ms);
            } else if (device->discarding_line) {
                device->stats.invalid_lines++;
            }
            device->line_length = 0U;
            device->discarding_line = false;
        } else if (!device->discarding_line) {
            if (device->line_length < (RN2483_LINE_SIZE - 1U)) {
                device->line[device->line_length++] = (char)byte;
            } else {
                device->line_length = 0U;
                device->discarding_line = true;
            }
        }
    }
}

rn2483_status_t rn2483_init(rn2483_t *device,
                            const rn2483_transport_t *transport,
                            const rn2483_raw_config_t *config,
                            uint32_t now_ms)
{
    if (device == NULL || transport == NULL || transport->transmit == NULL) {
        return RN2483_ERROR_ARGUMENT;
    }
    if (!config_valid(config)) {
        return RN2483_ERROR_CONFIGURATION;
    }

    memset(device, 0, sizeof(*device));
    device->transport = *transport;
    device->config = *config;
    device->phase = RN2483_PHASE_CONFIGURE;
    device->retry_at_ms = now_ms;
    return RN2483_OK;
}

void rn2483_on_rx_byte(rn2483_t *device, uint8_t byte)
{
    if (device == NULL) {
        return;
    }

    const uint16_t next =
        (uint16_t)((device->rx_head + 1U) % RN2483_RX_RING_SIZE);
    if (next == device->rx_tail) {
        device->stats.receive_overflows++;
        return;
    }
    device->rx_ring[device->rx_head] = byte;
    device->rx_head = next;
}

void rn2483_process(rn2483_t *device, uint32_t now_ms)
{
    if (device == NULL || device->transport.transmit == NULL) {
        return;
    }

    consume_received_bytes(device, now_ms);

    if (device->phase == RN2483_PHASE_BACKOFF) {
        if (!deadline_reached(now_ms, device->retry_at_ms)) {
            return;
        }
        device->phase = RN2483_PHASE_CONFIGURE;
        device->configuration_step = 0U;
    }

    if (device->waiting_for_reply) {
        if (deadline_reached(now_ms, device->deadline_ms)) {
            device->stats.response_timeouts++;
            enter_backoff(device, now_ms);
        }
        return;
    }

    if (device->phase == RN2483_PHASE_CONFIGURE) {
        char command[RN2483_COMMAND_SIZE];
        char expected[RN2483_LINE_SIZE];
        expected_reply_t expected_type;
        if (!build_configuration_command(device,
                                         device->configuration_step,
                                         command,
                                         sizeof(command),
                                         expected,
                                         sizeof(expected),
                                         &expected_type)) {
            enter_backoff(device, now_ms);
            return;
        }
        (void)transmit_command(device, command, now_ms);
    } else if (device->phase == RN2483_PHASE_ARM_RECEIVER) {
        (void)transmit_command(device, "radio rx 0\r\n", now_ms);
    }
}

rn2483_event_t rn2483_take_event(rn2483_t *device)
{
    if (device == NULL) {
        return RN2483_EVENT_NONE;
    }
    const rn2483_event_t event = device->pending_event;
    device->pending_event = RN2483_EVENT_NONE;
    return event;
}

bool rn2483_is_ready(const rn2483_t *device)
{
    return device != NULL && device->ready;
}
