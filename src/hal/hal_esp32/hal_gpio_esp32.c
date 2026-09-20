#include "hal/hal_gpio.h"
#include "driver/gpio.h"

#define PIN_LED_STATUS   GPIO_NUM_2
#define PIN_IGNITION_IN  GPIO_NUM_34 // Input-only pin

hal_status_t hal_gpio_init(void) {
    // Configure LED
    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << PIN_LED_STATUS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_out);

    // Configure Ignition Detection (optocoupler / resistor divider)
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << PIN_IGNITION_IN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_in);

    return HAL_OK;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool state) {
    if (pin == GPIO_PIN_LED_STATUS) {
        gpio_set_level(PIN_LED_STATUS, state ? 1 : 0);
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin) {
    if (pin == GPIO_PIN_IGNITION_IN) {
        return gpio_get_level(PIN_IGNITION_IN) != 0;
    }
    return false;
}

void hal_gpio_toggle(hal_gpio_pin_t pin) {
    if (pin == GPIO_PIN_LED_STATUS) {
        static bool s_led = false;
        s_led = !s_led;
        gpio_set_level(PIN_LED_STATUS, s_led ? 1 : 0);
    }
}
