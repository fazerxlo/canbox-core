#include "hal/hal_gpio.h"
#include <stdio.h>

static bool s_gpio_states[16] = {0};

hal_status_t hal_gpio_init(void) {
    for (int i = 0; i < 16; i++) {
        s_gpio_states[i] = false;
    }
    printf("[GPIO] ACC:0 ILL:0 REV:0\n");
    fflush(stdout);
    return HAL_STATUS_OK;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool state) {
    if ((int)pin >= 0 && pin < 16) {
        bool changed = (s_gpio_states[pin] != state);
        s_gpio_states[pin] = state;
        if (changed && (pin == GPIO_PIN_HEADUNIT_POWER || pin == GPIO_PIN_ILL_OUT || pin == GPIO_PIN_REVERSE_OUT)) {
            printf("[GPIO] ACC:%d ILL:%d REV:%d\n",
                   s_gpio_states[GPIO_PIN_HEADUNIT_POWER] ? 1 : 0,
                   s_gpio_states[GPIO_PIN_ILL_OUT] ? 1 : 0,
                   s_gpio_states[GPIO_PIN_REVERSE_OUT] ? 1 : 0);
            fflush(stdout);
        }
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin) {
    if ((int)pin >= 0 && pin < 16) {
        return s_gpio_states[pin];
    }
    return false;
}

void hal_gpio_toggle(hal_gpio_pin_t pin) {
    if ((int)pin >= 0 && pin < 16) {
        hal_gpio_write(pin, !s_gpio_states[pin]);
    }
}


