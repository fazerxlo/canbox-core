#include "hal/hal_gpio.h"
#include "NUC131.h"

#define NUC_GPIO_CAN_STBY_PORT PC
#define NUC_GPIO_CAN_STBY_PIN  3u
#define NUC_GPIO_HU_PWR_PORT   PA
#define NUC_GPIO_HU_PWR_PIN    8u
#define NUC_GPIO_ILL_PORT      PA
#define NUC_GPIO_ILL_PIN       9u
#define NUC_GPIO_BRAKE_PORT    PA
#define NUC_GPIO_BRAKE_PIN     12u
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
    // Transceiver Standby: PC3 in Open-Drain mode (0 = active, 1 = standby/silent)
    GPIO_SetMode(NUC_GPIO_CAN_STBY_PORT, (1u << NUC_GPIO_CAN_STBY_PIN), GPIO_PMD_OPEN_DRAIN);

    // Outputs in Push-Pull mode: PA8 (ACC), PA9 (ILL), PA12 (BRAKE), PA13 (REVERSE)
    GPIO_SetMode(NUC_GPIO_HU_PWR_PORT,  (1u << NUC_GPIO_HU_PWR_PIN),  GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_ILL_PORT,     (1u << NUC_GPIO_ILL_PIN),     GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_BRAKE_PORT,   (1u << NUC_GPIO_BRAKE_PIN),   GPIO_PMD_OUTPUT);
    GPIO_SetMode(NUC_GPIO_REVERSE_PORT, (1u << NUC_GPIO_REVERSE_PIN), GPIO_PMD_OUTPUT);

    // Input: PA0 (Ignition sense)
    GPIO_SetMode(NUC_GPIO_IGN_PORT,     (1u << NUC_GPIO_IGN_PIN),     GPIO_PMD_INPUT);

    // Initial output states
    gpio_write_port(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN, false); // 0 = Normal / Active
    gpio_write_port(NUC_GPIO_HU_PWR_PORT,   NUC_GPIO_HU_PWR_PIN,   false); // 0 = OFF
    gpio_write_port(NUC_GPIO_ILL_PORT,      NUC_GPIO_ILL_PIN,      false); // 0 = OFF
    gpio_write_port(NUC_GPIO_BRAKE_PORT,    NUC_GPIO_BRAKE_PIN,    false); // 0 = OFF (permanently)
    gpio_write_port(NUC_GPIO_REVERSE_PORT,  NUC_GPIO_REVERSE_PIN,  false); // 0 = OFF

    return HAL_STATUS_OK;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool state) {
    switch (pin) {
        case GPIO_PIN_CAN_STBY:
            gpio_write_port(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN, state);
            break;
        case GPIO_PIN_HEADUNIT_POWER:
            gpio_write_port(NUC_GPIO_HU_PWR_PORT, NUC_GPIO_HU_PWR_PIN, state);
            break;
        case GPIO_PIN_ILL_OUT:
            gpio_write_port(NUC_GPIO_ILL_PORT, NUC_GPIO_ILL_PIN, state);
            break;
        case GPIO_PIN_BRAKE_OUT:
            // Permanently OFF
            gpio_write_port(NUC_GPIO_BRAKE_PORT, NUC_GPIO_BRAKE_PIN, false);
            break;
        case GPIO_PIN_REVERSE_OUT:
            gpio_write_port(NUC_GPIO_REVERSE_PORT, NUC_GPIO_REVERSE_PIN, state);
            break;
        case GPIO_PIN_LED_STATUS:
        case GPIO_PIN_IGNITION_IN:
        default:
            break;
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin) {
    switch (pin) {
        case GPIO_PIN_CAN_STBY:
            return gpio_read_output(NUC_GPIO_CAN_STBY_PORT, NUC_GPIO_CAN_STBY_PIN);
        case GPIO_PIN_HEADUNIT_POWER:
            return gpio_read_output(NUC_GPIO_HU_PWR_PORT, NUC_GPIO_HU_PWR_PIN);
        case GPIO_PIN_ILL_OUT:
            return gpio_read_output(NUC_GPIO_ILL_PORT, NUC_GPIO_ILL_PIN);
        case GPIO_PIN_BRAKE_OUT:
            return gpio_read_output(NUC_GPIO_BRAKE_PORT, NUC_GPIO_BRAKE_PIN);
        case GPIO_PIN_REVERSE_OUT:
            return gpio_read_output(NUC_GPIO_REVERSE_PORT, NUC_GPIO_REVERSE_PIN);
        case GPIO_PIN_IGNITION_IN:
            return gpio_read_input(NUC_GPIO_IGN_PORT, NUC_GPIO_IGN_PIN);
        case GPIO_PIN_LED_STATUS:
        default:
            return false;
    }
}

void hal_gpio_toggle(hal_gpio_pin_t pin) {
    if (pin != GPIO_PIN_LED_STATUS && pin != GPIO_PIN_BRAKE_OUT) {
        hal_gpio_write(pin, !hal_gpio_read(pin));
    }
}
