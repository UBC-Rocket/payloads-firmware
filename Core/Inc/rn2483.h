/**
 * @file rn2483.h
 * @brief Nonblocking raw-LoRa command driver for the Microchip RN2483.
 */

#ifndef RN2483_H
#define RN2483_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RN2483_RX_RING_SIZE 256U
#define RN2483_LINE_SIZE 96U
#define RN2483_COMMAND_SIZE 96U

typedef enum {
    RN2483_OK = 0,
    RN2483_ERROR_ARGUMENT,
    RN2483_ERROR_CONFIGURATION,
    RN2483_ERROR_TRANSPORT
} rn2483_status_t;

typedef enum {
    RN2483_EVENT_NONE = 0,
    RN2483_EVENT_PUMP_ON,
    RN2483_EVENT_PUMP_OFF
} rn2483_event_t;

typedef struct {
    uint32_t frequency_hz;
    uint8_t spreading_factor;
    uint16_t bandwidth_khz;
    uint8_t coding_rate_denominator;
    uint8_t sync_word;
} rn2483_raw_config_t;

typedef bool (*rn2483_transmit_fn)(void *context,
                                   const uint8_t *data,
                                   size_t length);

typedef struct {
    rn2483_transmit_fn transmit;
    void *context;
} rn2483_transport_t;

typedef struct {
    uint32_t received_lines;
    uint32_t invalid_lines;
    uint32_t invalid_packets;
    uint32_t transport_errors;
    uint32_t response_timeouts;
    uint32_t receive_overflows;
    uint32_t restarts;
    uint32_t valid_commands;
} rn2483_stats_t;

typedef enum {
    RN2483_PHASE_CONFIGURE = 0,
    RN2483_PHASE_ARM_RECEIVER,
    RN2483_PHASE_LISTENING,
    RN2483_PHASE_BACKOFF
} rn2483_phase_t;

typedef struct {
    rn2483_transport_t transport;
    rn2483_raw_config_t config;
    rn2483_stats_t stats;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_ring[RN2483_RX_RING_SIZE];
    char line[RN2483_LINE_SIZE];
    size_t line_length;
    uint8_t configuration_step;
    uint32_t deadline_ms;
    uint32_t retry_at_ms;
    rn2483_event_t pending_event;
    rn2483_phase_t phase;
    bool waiting_for_reply;
    bool discarding_line;
    bool ready;
} rn2483_t;

rn2483_status_t rn2483_init(rn2483_t *device,
                            const rn2483_transport_t *transport,
                            const rn2483_raw_config_t *config,
                            uint32_t now_ms);

/**
 * @brief Feed one UART byte from interrupt context.
 */
void rn2483_on_rx_byte(rn2483_t *device, uint8_t byte);

/**
 * @brief Advance configuration, reply parsing, timeout, and receive state.
 */
void rn2483_process(rn2483_t *device, uint32_t now_ms);

/**
 * @brief Return and clear the next validated pump command.
 */
rn2483_event_t rn2483_take_event(rn2483_t *device);

bool rn2483_is_ready(const rn2483_t *device);

#ifdef __cplusplus
}
#endif

#endif /* RN2483_H */
