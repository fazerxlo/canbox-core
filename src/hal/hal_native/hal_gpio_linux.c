#include "hal/hal_gpio.h"
#include <stdio.h>

static bool s_gpio_states[8] = {0};

hal_status_t hal_gpio_init(void) {
    for (int i = 0; i < 8; i++) {
        s_gpio_states[i] = false;
    }
    return HAL_OK;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool state) {
    if ((int)pin >= 0 && pin < 8) {
        s_gpio_states[pin] = state;
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin) {
    if ((int)pin >= 0 && pin < 8) {
        return s_gpio_states[pin];
    }
    return false;
}

void hal_gpio_toggle(hal_gpio_pin_t pin) {
    if ((int)pin >= 0 && pin < 8) {
        s_gpio_states[pin] = !s_gpio_states[pin];
    }
}

