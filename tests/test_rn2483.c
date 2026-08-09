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

static void check_event(rn2483_t *device,
                        rn2483_event_type_t expected_type,
                        uint8_t expected_led_pwm_percent)
{
    rn2483_event_t event = {0};
    CHECK(rn2483_take_event(device, &event));
    CHECK(event.type == expected_type);
    CHECK(event.led_pwm_percent == expected_led_pwm_percent);
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
    char command[RN2483_COMMAND_SIZE];
    char reply[RN2483_LINE_SIZE];

    check_command(device, transport, now_ms,
                  "sys reset\r\n",
                  "RN2483 1.0.5 Dec 15 2015 09:38:09");
    check_command(device, transport, now_ms, "mac pause\r\n", "4294967295");
    check_command(device, transport, now_ms,
                  "radio set mod lora\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set freq 433575000\r\n", "ok");
    snprintf(command, sizeof(command),
             "radio set sf sf%u\r\n", device->config.spreading_factor);
    check_command(device, transport, now_ms, command, "ok");
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
    snprintf(command, sizeof(command),
             "radio set sync %02X\r\n", device->config.sync_word);
    check_command(device, transport, now_ms, command, "ok");
    check_command(device, transport, now_ms,
                  "radio set pwr 14\r\n", "ok");
    check_command(device, transport, now_ms,
                  "radio set wdt 4000\r\n", "ok");
    check_command(device, transport, now_ms, "radio get mod\r\n", "lora");
    check_command(device, transport, now_ms,
                  "radio get freq\r\n", "433575000");
    snprintf(reply, sizeof(reply), "sf%u", device->config.spreading_factor);
    check_command(device, transport, now_ms, "radio get sf\r\n", reply);
    check_command(device, transport, now_ms, "radio get bw\r\n", "125");
    check_command(device, transport, now_ms, "radio get cr\r\n", "4/5");
    check_command(device, transport, now_ms, "radio get crc\r\n", "on");
    check_command(device, transport, now_ms, "radio get iqi\r\n", "off");
    check_command(device, transport, now_ms, "radio get prlen\r\n", "8");
    snprintf(reply, sizeof(reply), "%02X", device->config.sync_word);
    check_command(device, transport, now_ms, "radio get sync\r\n", reply);
    check_command(device, transport, now_ms, "radio get wdt\r\n", "4000");
    check_command(device, transport, now_ms, "radio get pwr\r\n", "14");

    if (device->transmit_queued) {
        CHECK(strcmp(transport->last_command,
                     device->transmit_command) == 0);
        return;
    }
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
        .frequency_hz = 433575000U,
        .spreading_factor = 7U,
        .bandwidth_khz = 125U,
        .coding_rate_denominator = 5U,
        .sync_word = 0x12U,
    };
    rn2483_t device;
    uint32_t now_ms = 0U;
    CHECK(rn2483_init(&device, &transport, &config, now_ms) == RN2483_OK);
    configure_to_listening(&device, &fake, &now_ms);

    /* RN2483 1.0.4 places two spaces between radio_rx and the payload. */
    feed_line(&device, "radio_rx  50554D505F4F4E");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_PUMP_ON, 0U);
    rn2483_event_t event = {0};
    CHECK(!rn2483_take_event(&device, &event));
    CHECK(strcmp(device.last_line, "radio_rx  50554D505F4F4E") == 0);
    CHECK(strcmp(fake.last_command, "radio rx 0\r\n") == 0);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(rn2483_is_ready(&device));

    feed_line(&device, "radio_rx   50554D505F4F4646");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_PUMP_OFF, 0U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 50554D505F4F4E0D0A");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_PUMP_ON, 0U);
    CHECK(device.stats.valid_commands == 3U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 50554D505F4F4E00");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(!rn2483_take_event(&device, &event));
    CHECK(device.stats.invalid_packets == 1U);
    CHECK(strcmp(fake.last_command, "radio rx 0\r\n") == 0);

    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 4C45445F4F4E");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_LED_ON, 100U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 4C45445F4F46460D");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_LED_OFF, 0U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 4C45445F50574D203432");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_LED_PWM, 42U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 4C45445F50574D2030");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_LED_PWM, 0U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 4C45445F50574D20313030");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_event(&device, RN2483_EVENT_LED_PWM, 100U);
    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);

    feed_line(&device, "radio_rx 4C45445F50574D20313031");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(!rn2483_take_event(&device, &event));
    CHECK(device.stats.invalid_packets == 2U);
}

static void test_validation_and_recovery(void)
{
    fake_transport_t fake = {0};
    const rn2483_transport_t transport = {
        .transmit = fake_transmit,
        .context = &fake,
    };
    rn2483_raw_config_t config = {
        .frequency_hz = 433575000U,
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

    config.frequency_hz = 433575000U;
    CHECK(rn2483_init(&device, &transport, &config, 0U) == RN2483_OK);
    rn2483_process(&device, 0U);
    CHECK(strcmp(fake.last_command, "sys reset\r\n") == 0);
    rn2483_process(&device, 1000U);
    CHECK(device.stats.response_timeouts == 1U);
    CHECK(device.phase == RN2483_PHASE_BACKOFF);
    rn2483_process(&device, 1999U);
    CHECK(fake.command_count == 1U);
    rn2483_process(&device, 2000U);
    CHECK(fake.command_count == 2U);
    CHECK(strcmp(fake.last_command, "sys reset\r\n") == 0);

    feed_line(&device, "not an rn2483");
    rn2483_process(&device, 2001U);
    CHECK(device.phase == RN2483_PHASE_BACKOFF);
    CHECK(device.stats.invalid_lines == 1U);
}

static void test_text_transmit_returns_to_receive(void)
{
    fake_transport_t fake = {0};
    const rn2483_transport_t transport = {
        .transmit = fake_transmit,
        .context = &fake,
    };
    const rn2483_raw_config_t config = {
        .frequency_hz = 433575000U,
        .spreading_factor = 12U,
        .bandwidth_khz = 125U,
        .coding_rate_denominator = 5U,
        .sync_word = 0x34U,
    };
    rn2483_t device;
    uint32_t now_ms = 0U;
    CHECK(rn2483_init(&device, &transport, &config, now_ms) == RN2483_OK);
    configure_to_listening(&device, &fake, &now_ms);

    CHECK(rn2483_send_text(&device, "PING 0"));
    CHECK(rn2483_is_ready(&device));
    CHECK(!rn2483_send_text(&device, "PING 1"));

    /* The bounded receive watchdog makes the module definitively idle before
       the queued packet is sent. */
    feed_line(&device, "radio_err");
    now_ms++;
    rn2483_process(&device, now_ms);
    check_command(&device, &fake, &now_ms,
                  "radio tx 50494E472030\r\n", "ok");
    CHECK(device.phase == RN2483_PHASE_WAIT_TRANSMIT);

    feed_line(&device, "radio_tx_ok");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(device.stats.transmitted_packets == 1U);
    CHECK(strcmp(fake.last_command, "radio rx 0\r\n") == 0);

    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(rn2483_is_ready(&device));
}

static void test_startup_text_transmit_precedes_receive(void)
{
    fake_transport_t fake = {0};
    const rn2483_transport_t transport = {
        .transmit = fake_transmit,
        .context = &fake,
    };
    const rn2483_raw_config_t config = {
        .frequency_hz = 433575000U,
        .spreading_factor = 12U,
        .bandwidth_khz = 125U,
        .coding_rate_denominator = 5U,
        .sync_word = 0x34U,
    };
    rn2483_t device;
    uint32_t now_ms = 0U;
    CHECK(rn2483_init(&device, &transport, &config, now_ms) == RN2483_OK);
    CHECK(rn2483_send_text(&device, "PING 0"));

    configure_to_listening(&device, &fake, &now_ms);
    CHECK(device.phase == RN2483_PHASE_TRANSMIT_COMMAND);
    CHECK(strcmp(fake.last_command,
                 "radio tx 50494E472030\r\n") == 0);

    feed_line(&device, "ok");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(device.phase == RN2483_PHASE_WAIT_TRANSMIT);

    feed_line(&device, "radio_tx_ok");
    now_ms++;
    rn2483_process(&device, now_ms);
    CHECK(device.stats.transmitted_packets == 1U);
    CHECK(strcmp(fake.last_command, "radio rx 0\r\n") == 0);
}

int main(void)
{
    test_configuration_and_commands();
    test_validation_and_recovery();
    test_text_transmit_returns_to_receive();
    test_startup_text_transmit_precedes_receive();

    if (failures != 0) {
        fprintf(stderr, "%d RN2483 test(s) failed\n", failures);
        return 1;
    }

    puts("RN2483 tests passed");
    return 0;
}
