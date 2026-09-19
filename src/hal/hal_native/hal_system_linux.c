#define _POSIX_C_SOURCE 199309L
#include "hal/hal_system.h"
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

static struct timespec s_start_time;

hal_status_t hal_system_init(void) {
    if (clock_gettime(CLOCK_MONOTONIC, &s_start_time) != 0) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint32_t hal_get_tick_ms(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }
    uint64_t ms = (uint64_t)(now.tv_sec - s_start_time.tv_sec) * 1000ULL +
                  (uint64_t)(now.tv_nsec - s_start_time.tv_nsec) / 1000000ULL;
    return (uint32_t)ms;
}

void hal_delay_ms(uint32_t ms) {
    struct timespec req = {
        .tv_sec = ms / 1000,
        .tv_nsec = (long)(ms % 1000) * 1000000L
    };
    nanosleep(&req, NULL);
}

void hal_system_reboot(void) {
    printf("[SYS] Reboot invoked on desktop host. Exiting...\n");
    exit(0);
}
