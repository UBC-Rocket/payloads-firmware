#include "rn2483.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    char last_command[RN2483_COMMAND_SIZE];
    uint32_t command_count;
    bool fail_transmit;
} fake_transport_t;

static int failures;

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            fprintf(stderr, "check failed at %s:%d: %s\n",                  \
                    __FILE__, __LINE__, #condition);                          \
            failures++;                                                       \
        }                                                                     \
    } while (0)

static bool fake_transmit(void *context,
                          const uint8_t *data,
                          size_t length)
{
    fake_transport_t *transport = (fake_transport_t *)context;
    if (transport->fail_transmit ||
        length >= sizeof(transport->last_command)) {
        return false;
    }
    memcpy(transport->last_command, data, length);
    transport->last_command[length] = '\0';
    transport->command_count++;
    return true;
}

static void feed_line(rn2483_t *device, const char *line)
{
    for (size_t index = 0U; line[index] != '\0'; index++) {
        rn2483_on_rx_byte(device, (uint8_t)line[index]);
    }
    rn2483_on_rx_byte(device, (uint8_t)'\r');
    rn2483_on_rx_byte(device, (uint8_t)'\n');
}

static void check_command(rn2483_t *device,
                          fake_transport_t *transport,
                          uint32_t *now_ms,
                          const char *command,
                          const char *reply)
{
    rn2483_process(device, *now_ms);
    CHECK(strcmp(transport->last_command, command) == 0);
    feed_line(device, reply);
    (*now_ms)++;
    rn2483_process(device, *now_ms);
}

static void configure_to_listening(rn2483_t *device,
                                   fake_transport_t *transport,
                                   uint32_t *now_ms)
{
    check_command(device, transport, now_ms,
                  "sys get ver\r\n",
                  "RN2483 1.0.5 Dec 15 2015 09:38:09");
    check_command(device, transport, now_ms, "mac pause\r\n", "4294967295");
    check_command(device, transport, now_ms,
                  "radio set mod lora\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set freq 868000000\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set sf sf7\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set bw 125\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set cr 4/5\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set crc on\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set iqi off\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set prlen 8\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set sync 12\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set wdt 0\r\n", "ok");
    check_command(device, transport, now_ms, "radio get mod\r\n", "lora");
    check_command(device, transport, now_ms,
                  "radio get freq\r\n", "868000000");
    check_command(device, transport, now_ms, "radio get sf\r\n", "sf7");
    check_command(device, transport, now_ms, "radio get bw\r\n", "125");
    check_command(device, transport, now_ms, "radio get cr\r\n", "4/5");
    check_command(device, transport, now_ms, "radio get crc\r\n", "on");
    check_command(device, transport, now_ms, "radio get iqi\r\n", "off");
    check_command(device, transport, now_ms, "radio get prlen\r\n", "8");
    check_command(device, transport, now_ms, "radio get sync\r\n", "12");
    check_command(device, transport, now_ms, "radio get wdt\r\n", "0");

    rn2483_process(device, *now_ms);
    CHECK(strcmp(transport->last_command, "radio rx 0\r\n") == 0);
    feed_line(device, "ok");
    (*now_ms)++;
    rn2483_process(device, *now_ms);
    CHECK(rn2483_is_ready(device));
}

static void test_configuration_and_commands(void)
{
    fake_transport_t fake = {0};
    const rn2483_transport_t transport = {
        .transmit = fake_transmit,
        .context = &fake,
    };
    const rn2483_raw_config_t config = {
        .frequency_hz = 868000000U,
        .spreading_factor = 7U,
        .bandwidth_khz = 125U,
        .coding_rate_denominator = 5U,
        .sync_word = 0x12U,
    };
    rn2483_t device;
    uint32_t now_ms = 0U;
    CHECK(rn2483_init(&device, &transport, &config, now_ms) == RN2483_OK);
    configure_to_listening(&device, &fake, &now_ms);

    feed_line(&device, "radio_rx 50554D505F4F4E");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(rn2483_take_event(&device) == RN2483_EVENT_PUMP_ON);
    CHECK(rn2483_take_event(&device) == RN2483_EVENT_NONE);
    CHECK(strcmp(fake.last_command, "radio rx 0\r\n") == 0);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(rn2483_is_ready(&device));

    feed_line(&device, "radio_rx 50554D505F4F4646");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(rn2483_take_event(&device) == RN2483_EVENT_PUMP_OFF);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 50554D505F4F4E00");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(rn2483_take_event(&device) == RN2483_EVENT_NONE);
    CHECK(device.stats.invalid_packets == 1U);
    CHECK(strcmp(fake.last_command, "radio rx 0\r\n") == 0);
}

static void test_validation_and_recovery(void)
{
    fake_transport_t fake = {0};
    const rn2483_transport_t transport = {
        .transmit = fake_transmit,
        .context = &fake,
    };
    rn2483_raw_config_t config = {
        .frequency_hz = 868000000U,
        .spreading_factor = 7U,
        .bandwidth_khz = 125U,
        .coding_rate_denominator = 5U,
        .sync_word = 0x12U,
    };
    rn2483_t device;
    CHECK(rn2483_init(NULL, &transport, &config, 0U) ==
          RN2483_ERROR_ARGUMENT);
    config.frequency_hz = 915000000U;
    CHECK(rn2483_init(&device, &transport, &config, 0U) ==
          RN2483_ERROR_CONFIGURATION);

    config.frequency_hz = 868000000U;
    CHECK(rn2483_init(&device, &transport, &config, 0U) == RN2483_OK);
    rn2483_process(&device, 0U);
    CHECK(strcmp(fake.last_command, "sys get ver\r\n") == 0);
    rn2483_process(&device, 1000U);
    CHECK(device.stats.response_timeouts == 1U);
    CHECK(device.phase == RN2483_PHASE_BACKOFF);
    rn2483_process(&device, 1999U);
    CHECK(fake.command_count == 1U);
    rn2483_process(&device, 2000U);
    CHECK(fake.command_count == 2U);
    CHECK(strcmp(fake.last_command, "sys get ver\r\n") == 0);

    feed_line(&device, "not an rn2483");
    rn2483_process(&device, 2001U);
    CHECK(device.phase == RN2483_PHASE_BACKOFF);
    CHECK(device.stats.invalid_lines == 1U);
}

int main(void)
{
    test_configuration_and_commands();
    test_validation_and_recovery();

    if (failures != 0) {
        fprintf(stderr, "%d RN2483 test(s) failed\n", failures);
        return 1;
    }

    puts("RN2483 tests passed");
    return 0;
}
