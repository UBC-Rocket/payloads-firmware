#include "pump_timer.h"

#include <stdint.h>
#include <stdio.h>

static int failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        if (!(condition)) {                                                \
            fprintf(stderr, "check failed at %s:%d: %s\n",               \
                    __FILE__, __LINE__, #condition);                       \
            failures++;                                                    \
        }                                                                  \
    } while (0)

int main(void)
{
    pump_timer_t timer = {0};

    CHECK(!pump_timer_is_active(&timer));
    CHECK(!pump_timer_expire(&timer, 1000U));

    pump_timer_start(&timer, 1000U, 1500U);
    CHECK(pump_timer_is_active(&timer));
    CHECK(!pump_timer_expire(&timer, 2499U));
    CHECK(pump_timer_expire(&timer, 2500U));
    CHECK(!pump_timer_expire(&timer, 2501U));

    pump_timer_start(&timer, 5000U, 5500U);
    pump_timer_start(&timer, 6000U, 1250U);
    CHECK(!pump_timer_expire(&timer, 7249U));
    CHECK(pump_timer_expire(&timer, 7250U));

    pump_timer_start(&timer, 100U, 100U);
    pump_timer_cancel(&timer);
    CHECK(!pump_timer_expire(&timer, 1000U));

    pump_timer_start(&timer, UINT32_MAX - 49U, 100U);
    CHECK(!pump_timer_expire(&timer, 49U));
    CHECK(pump_timer_expire(&timer, 50U));

    pump_timer_start(NULL, 0U, 100U);
    pump_timer_cancel(NULL);
    CHECK(!pump_timer_expire(NULL, 100U));
    CHECK(!pump_timer_is_active(NULL));

    if (failures != 0) {
        fprintf(stderr, "%d pump timer test(s) failed\n", failures);
        return 1;
    }
    return 0;
}
