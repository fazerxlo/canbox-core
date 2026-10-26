#define _POSIX_C_SOURCE 199309L
#include "hal/hal_system.h"
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

static struct timespec s_start_time;
static uint32_t s_virtual_offset_ms = 0;

hal_status_t hal_system_init(void) {
    s_virtual_offset_ms = 0;
    if (clock_gettime(CLOCK_MONOTONIC, &s_start_time) != 0) {
        return HAL_STATUS_ERROR;
    }
    return HAL_STATUS_OK;
}

uint32_t hal_get_tick_ms(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return s_virtual_offset_ms;
    }
    if (s_start_time.tv_sec == 0 && s_start_time.tv_nsec == 0) {
        s_start_time = now;
    }
    int64_t sec = (int64_t)now.tv_sec - (int64_t)s_start_time.tv_sec;
    int64_t nsec = (int64_t)now.tv_nsec - (int64_t)s_start_time.tv_nsec;
    int64_t ms = sec * 1000LL + nsec / 1000000LL;
    return (uint32_t)ms + s_virtual_offset_ms;
}

void hal_delay_ms(uint32_t ms) {
#if defined(FAST_VIRTUAL_TIME)
    s_virtual_offset_ms += ms;
#else
    const char *fast_env = getenv("CANBOX_FAST_TIME");
    if (fast_env && fast_env[0] == '1') {
        s_virtual_offset_ms += ms;
        return;
    }
    struct timespec req = {
        .tv_sec = ms / 1000,
        .tv_nsec = (long)(ms % 1000) * 1000000L
    };
    nanosleep(&req, NULL);
#endif
}

void hal_system_reboot(void) {
    printf("[SYS] Reboot invoked on desktop host. Exiting...\n");
    exit(0);
}

void hal_system_enter_deep_sleep(void) {
    hal_delay_ms(10);
}

void hal_watchdog_feed(void) {
}
