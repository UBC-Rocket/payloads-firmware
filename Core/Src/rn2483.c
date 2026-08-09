#include "rn2483.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RN2483_RESPONSE_TIMEOUT_MS 1000U
#define RN2483_TRANSMIT_TIMEOUT_MS 10000U
#define RN2483_RECEIVE_WATCHDOG_MS 4000U
#define RN2483_RETRY_DELAY_MS 1000U
#define RN2483_CONFIGURATION_STEP_COUNT 24U
#define RN2483_PAYLOAD_TEXT_SIZE 16U

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

static bool is_transmit_phase(rn2483_phase_t phase)
{
    return phase == RN2483_PHASE_TRANSMIT_COMMAND ||
           phase == RN2483_PHASE_WAIT_TRANSMIT;
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
        if (is_transmit_phase(device->phase)) {
            device->stats.transmit_failures++;
        }
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
        command_length = snprintf(command, command_size, "sys reset\r\n");
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
                                  "radio set pwr 14\r\n");
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 12U:
        command_length = snprintf(command, command_size,
                                  "radio set wdt %u\r\n",
                                  RN2483_RECEIVE_WATCHDOG_MS);
        expected_length = snprintf(expected, expected_size, "ok");
        break;
    case 13U:
        command_length = snprintf(command, command_size, "radio get mod\r\n");
        expected_length = snprintf(expected, expected_size, "lora");
        break;
    case 14U:
        command_length = snprintf(command, command_size, "radio get freq\r\n");
        expected_length = snprintf(expected, expected_size, "%lu",
                                   (unsigned long)device->config.frequency_hz);
        break;
    case 15U:
        command_length = snprintf(command, command_size, "radio get sf\r\n");
        expected_length = snprintf(expected, expected_size, "sf%u",
                                   device->config.spreading_factor);
        break;
    case 16U:
        command_length = snprintf(command, command_size, "radio get bw\r\n");
        expected_length = snprintf(expected, expected_size, "%u",
                                   device->config.bandwidth_khz);
        break;
    case 17U:
        command_length = snprintf(command, command_size, "radio get cr\r\n");
        expected_length = snprintf(expected, expected_size, "4/%u",
                                   device->config.coding_rate_denominator);
        break;
    case 18U:
        command_length = snprintf(command, command_size, "radio get crc\r\n");
        expected_length = snprintf(expected, expected_size, "on");
        break;
    case 19U:
        command_length = snprintf(command, command_size, "radio get iqi\r\n");
        expected_length = snprintf(expected, expected_size, "off");
        break;
    case 20U:
        command_length = snprintf(command, command_size, "radio get prlen\r\n");
        expected_length = snprintf(expected, expected_size, "8");
        break;
    case 21U:
        command_length = snprintf(command, command_size, "radio get sync\r\n");
        expected_length = snprintf(expected, expected_size, "%02X",
                                   device->config.sync_word);
        break;
    case 22U:
        command_length = snprintf(command, command_size, "radio get wdt\r\n");
        expected_length = snprintf(expected, expected_size, "%u",
                                   RN2483_RECEIVE_WATCHDOG_MS);
        break;
    case 23U:
        command_length = snprintf(command, command_size, "radio get pwr\r\n");
        expected_length = snprintf(expected, expected_size, "14");
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

static int hex_nibble(char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    if (character >= 'A' && character <= 'F') {
        return character - 'A' + 10;
    }
    if (character >= 'a' && character <= 'f') {
        return character - 'a' + 10;
    }
    return -1;
}

static bool decode_radio_payload(const char *line,
                                 char *text,
                                 size_t text_size)
{
    static const char prefix[] = "radio_rx";
    if (line == NULL || text == NULL || text_size < 2U ||
        strncmp(line, prefix, sizeof(prefix) - 1U) != 0) {
        return false;
    }

    const char *hex = line + (sizeof(prefix) - 1U);
    if (*hex != ' ') {
        return false;
    }
    while (*hex == ' ') {
        hex++;
    }

    uint8_t decoded[RN2483_PAYLOAD_TEXT_SIZE + 2U];
    size_t decoded_length = 0U;
    while (*hex != '\0') {
        if (hex[1] == '\0' ||
            decoded_length >= sizeof(decoded)) {
            return false;
        }
        const int high = hex_nibble(hex[0]);
        const int low = hex_nibble(hex[1]);
        if (high < 0 || low < 0) {
            return false;
        }
        decoded[decoded_length++] =
            (uint8_t)(((uint8_t)high << 4U) | (uint8_t)low);
        hex += 2;
    }

    size_t text_length = decoded_length;
    if (text_length >= 2U &&
        decoded[text_length - 2U] == (uint8_t)'\r' &&
        decoded[text_length - 1U] == (uint8_t)'\n') {
        text_length -= 2U;
    } else if (text_length >= 1U &&
               (decoded[text_length - 1U] == (uint8_t)'\r' ||
                decoded[text_length - 1U] == (uint8_t)'\n')) {
        text_length--;
    }

    if (text_length == 0U || text_length >= text_size) {
        return false;
    }
    for (size_t index = 0U; index < text_length; index++) {
        if (decoded[index] < 0x20U || decoded[index] > 0x7EU) {
            return false;
        }
        text[index] = (char)decoded[index];
    }
    text[text_length] = '\0';
    return true;
}

static bool parse_led_pwm(const char *text, uint8_t *percent)
{
    static const char prefix[] = "LED_PWM ";
    if (strncmp(text, prefix, sizeof(prefix) - 1U) != 0) {
        return false;
    }

    const char *digits = text + (sizeof(prefix) - 1U);
    if (*digits == '\0') {
        return false;
    }

    uint16_t value = 0U;
    for (size_t index = 0U; digits[index] != '\0'; index++) {
        if (index >= 3U ||
            digits[index] < '0' || digits[index] > '9') {
            return false;
        }
        value = (uint16_t)((value * 10U) +
                           (uint16_t)(digits[index] - '0'));
        if (value > 100U) {
            return false;
        }
    }

    *percent = (uint8_t)value;
    return true;
}

static bool parse_radio_event(const char *line, rn2483_event_t *event)
{
    char text[RN2483_PAYLOAD_TEXT_SIZE];
    if (!decode_radio_payload(line, text, sizeof(text))) {
        return false;
    }

    event->led_pwm_percent = 0U;
    if (strcmp(text, "PUMP_ON") == 0) {
        event->type = RN2483_EVENT_PUMP_ON;
    } else if (strcmp(text, "PUMP_OFF") == 0) {
        event->type = RN2483_EVENT_PUMP_OFF;
    } else if (strcmp(text, "LED_ON") == 0) {
        event->type = RN2483_EVENT_LED_ON;
        event->led_pwm_percent = 100U;
    } else if (strcmp(text, "LED_OFF") == 0) {
        event->type = RN2483_EVENT_LED_OFF;
    } else if (parse_led_pwm(text, &event->led_pwm_percent)) {
        event->type = RN2483_EVENT_LED_PWM;
    } else {
        return false;
    }
    return true;
}

static void handle_complete_line(rn2483_t *device,
                                 const char *line,
                                 uint32_t now_ms)
{
    device->stats.received_lines++;
    const size_t line_length = strlen(line);
    memcpy(device->last_line, line, line_length + 1U);

    if (device->phase == RN2483_PHASE_LISTENING) {
        rn2483_event_t event;
        if (parse_radio_event(line, &event)) {
            device->pending_event = event;
            device->stats.valid_commands++;
        } else if (strncmp(line, "radio_rx ", 9U) == 0) {
            device->stats.invalid_packets++;
        } else if (strcmp(line, "radio_err") != 0) {
            device->stats.invalid_lines++;
            return;
        }

        device->ready = false;
        device->phase = device->transmit_queued
                            ? RN2483_PHASE_TRANSMIT_COMMAND
                            : RN2483_PHASE_ARM_RECEIVER;
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
            device->phase = device->transmit_queued
                                ? RN2483_PHASE_TRANSMIT_COMMAND
                                : RN2483_PHASE_ARM_RECEIVER;
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
    } else if (device->phase == RN2483_PHASE_TRANSMIT_COMMAND) {
        if (strcmp(line, "ok") != 0) {
            device->stats.invalid_lines++;
            device->stats.transmit_failures++;
            enter_backoff(device, now_ms);
            return;
        }
        device->phase = RN2483_PHASE_WAIT_TRANSMIT;
        device->deadline_ms = now_ms + RN2483_TRANSMIT_TIMEOUT_MS;
    } else if (device->phase == RN2483_PHASE_WAIT_TRANSMIT) {
        if (strcmp(line, "radio_tx_ok") != 0) {
            device->stats.invalid_lines++;
            device->stats.transmit_failures++;
            enter_backoff(device, now_ms);
            return;
        }
        device->stats.transmitted_packets++;
        device->transmit_command[0] = '\0';
        device->transmit_queued = false;
        device->waiting_for_reply = false;
        device->phase = RN2483_PHASE_ARM_RECEIVER;
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
            if (is_transmit_phase(device->phase)) {
                device->stats.transmit_failures++;
            }
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
    } else if (device->phase == RN2483_PHASE_TRANSMIT_COMMAND) {
        (void)transmit_command(device, device->transmit_command, now_ms);
    }
}

bool rn2483_take_event(rn2483_t *device, rn2483_event_t *event)
{
    if (device == NULL || event == NULL ||
        device->pending_event.type == RN2483_EVENT_NONE) {
        return false;
    }
    *event = device->pending_event;
    device->pending_event.type = RN2483_EVENT_NONE;
    return true;
}

bool rn2483_is_ready(const rn2483_t *device)
{
    return device != NULL && device->ready;
}

bool rn2483_send_text(rn2483_t *device, const char *payload)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    static const char prefix[] = "radio tx ";

    if (device == NULL || payload == NULL || payload[0] == '\0' ||
        device->transmit_queued) {
        return false;
    }

    const bool can_queue =
        device->phase == RN2483_PHASE_CONFIGURE ||
        device->phase == RN2483_PHASE_BACKOFF ||
        device->phase == RN2483_PHASE_LISTENING ||
        (device->phase == RN2483_PHASE_ARM_RECEIVER &&
         !device->waiting_for_reply);
    if (!can_queue) {
        return false;
    }

    const size_t payload_length = strlen(payload);
    const size_t command_length =
        (sizeof(prefix) - 1U) + (payload_length * 2U) + 2U;
    if (command_length >= sizeof(device->transmit_command)) {
        return false;
    }

    size_t position = 0U;
    memcpy(device->transmit_command, prefix, sizeof(prefix) - 1U);
    position += sizeof(prefix) - 1U;
    for (size_t index = 0U; index < payload_length; index++) {
        const uint8_t byte = (uint8_t)payload[index];
        device->transmit_command[position++] = hex_digits[byte >> 4U];
        device->transmit_command[position++] = hex_digits[byte & 0x0FU];
    }
    device->transmit_command[position++] = '\r';
    device->transmit_command[position++] = '\n';
    device->transmit_command[position] = '\0';
    device->transmit_queued = true;

    if (device->phase == RN2483_PHASE_ARM_RECEIVER) {
        device->phase = RN2483_PHASE_TRANSMIT_COMMAND;
    }
    return true;
}
