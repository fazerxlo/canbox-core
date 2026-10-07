#include "hal/hal_gpio.h"
#include "NUC131.h"

#define NUC_GPIO_LED_PORT      PA
#define NUC_GPIO_LED_PIN       9u
#define NUC_GPIO_CAN_STBY_PORT PA
#define NUC_GPIO_CAN_STBY_PIN  12u
#define NUC_GPIO_HU_PWR_PORT   PA
#define NUC_GPIO_HU_PWR_PIN    8u
#define NUC_GPIO_REVERSE_PORT  PA
#define NUC_GPIO_REVERSE_PIN   13u
#define NUC_GPIO_IGN_PORT      PA
#define NUC_GPIO_IGN_PIN       0u

static void gpio_write_port(GPIO_T *port, uint32_t pin, bool state) {
    if (state) {
        port->DOUT |= (1u << pin);
    } else {
        port->DOUT &= ~(1u << pin);
    }
}

static bool gpio_read_output(GPIO_T *port, uint32_t pin) {
    return (port->DOUT & (1u << pin)) != 0u;
}

static bool gpio_read_input(GPIO_T *port, uint32_t pin) {
    return (port->PIN & (1u << pin)) != 0u;
}

hal_status_t hal_gpio_init(void) {
    GPIO_SetMode(NUC_GPIO_LED_PORT, NUC_GPIO_LED_PIN, GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN, GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_HU_PWR_PORT, NUC_GPIO_HU_PWR_PIN, GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_REVERSE_PORT, NUC_GPIO_REVERSE_PIN, GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_IGN_PORT, NUC_GPIO_IGN_PIN, GPIO_PMD_INPUT);

    gpio_write_port(NUC_GPIO_LED_PORT, NUC_GPIO_LED_PIN, false);
    gpio_write_port(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN, false);
    gpio_write_port(NUC_GPIO_HU_PWR_PORT, NUC_GPIO_HU_PWR_PIN, false);
    gpio_write_port(NUC_GPIO_REVERSE_PORT, NUC_GPIO_REVERSE_PIN, false);

    return HAL_STATUS_OK;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool state) {
    switch (pin) {
        case GPIO_PIN_LED_STATUS:
            gpio_write_port(NUC_GPIO_LED_PORT, NUC_GPIO_LED_PIN, state);
            break;
        case GPIO_PIN_CAN_STBY:
            gpio_write_port(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN, state);
            break;
        case GPIO_PIN_HEADUNIT_POWER:
            gpio_write_port(NUC_GPIO_HU_PWR_PORT, NUC_GPIO_HU_PWR_PIN, state);
            break;
        case GPIO_PIN_REVERSE_OUT:
            gpio_write_port(NUC_GPIO_REVERSE_PORT, NUC_GPIO_REVERSE_PIN, state);
            break;
        case GPIO_PIN_IGNITION_IN:
        default:
            break;
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin) {
    switch (pin) {
        case GPIO_PIN_LED_STATUS:
            return gpio_read_output(NUC_GPIO_LED_PORT, NUC_GPIO_LED_PIN);
        case GPIO_PIN_CAN_STBY:
            return gpio_read_output(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN);
        case GPIO_PIN_HEADUNIT_POWER:
            return gpio_read_output(NUC_GPIO_HU_PWR_PORT, NUC_GPIO_HU_PWR_PIN);
        case GPIO_PIN_REVERSE_OUT:
            return gpio_read_output(NUC_GPIO_REVERSE_PORT, NUC_GPIO_REVERSE_PIN);
        case GPIO_PIN_IGNITION_IN:
            return gpio_read_input(NUC_GPIO_IGN_PORT, NUC_GPIO_IGN_PIN);
        default:
            return false;
    }
}

void hal_gpio_toggle(hal_gpio_pin_t pin) {
    hal_gpio_write(pin, !hal_gpio_read(pin));
}
