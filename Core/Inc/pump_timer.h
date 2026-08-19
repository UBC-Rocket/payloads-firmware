/**
 * @file pump_timer.h
 * @brief Wrap-safe timer state for the timed pump command.
 */

#ifndef PUMP_TIMER_H
#define PUMP_TIMER_H

#include <stdbool.h>
#include <stdint.h>

/* Change this value to set the fixed pump-run duration.  The existing
 * PUMP_RUN_6_5 command name is intentionally independent of this setting. */
#ifndef PUMP_RUN_DURATION_MS
#define PUMP_RUN_DURATION_MS 6500U
#endif

typedef struct {
    volatile uint32_t deadline_ms;
    volatile bool active;
} pump_timer_t;

void pump_timer_start(pump_timer_t *timer,
                      uint32_t now_ms,
                      uint32_t duration_ms);
void pump_timer_cancel(pump_timer_t *timer);

/**
 * @brief Expire an active timer at or after its deadline.
 * @return true exactly once for each timer that reaches its deadline.
 */
bool pump_timer_expire(pump_timer_t *timer, uint32_t now_ms);

bool pump_timer_is_active(const pump_timer_t *timer);

#endif /* PUMP_TIMER_H */
