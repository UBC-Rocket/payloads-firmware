#include "pump_timer.h"

#include <stddef.h>

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

void pump_timer_start(pump_timer_t *timer,
                      uint32_t now_ms,
                      uint32_t duration_ms)
{
    if (timer == NULL) {
        return;
    }

    timer->deadline_ms = now_ms + duration_ms;
    timer->active = true;
}

void pump_timer_cancel(pump_timer_t *timer)
{
    if (timer != NULL) {
        timer->active = false;
    }
}

bool pump_timer_expire(pump_timer_t *timer, uint32_t now_ms)
{
    if (timer == NULL || !timer->active ||
        !deadline_reached(now_ms, timer->deadline_ms)) {
        return false;
    }

    timer->active = false;
    return true;
}

bool pump_timer_is_active(const pump_timer_t *timer)
{
    return timer != NULL && timer->active;
}
