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
    /* Set GPIO modes and initial outputs matching factory board:
     * Port A: PA_PMD = 0x75f5ff0f, PA_DOUT = 0x4cff (PA15 push-pull 0, PA8/9/12/13 push-pull 0, PA0 input)
     * Port B: PB_PMD = 0x43030ff4, PB_DOUT = 0x113f (PB15 push-pull 0, PB12/PB3 driven to factory 1)
     * Port C: PC_PMD = 0x0000f045, PC_DOUT = 0x00c7 (PC0=1, PC1=1, PC2=1, PC3=0 CAN active, PC6=1, PC7=1)
     */
    PA->PMD  = 0x75F5FF0FUL;
    PA->DOUT = 0x00004CFFUL;

    PB->PMD  = 0x43030FF4UL;
    PB->DOUT = 0x0000113FUL;

    PC->PMD  = 0x0000F045UL;
    PC->DOUT = 0x000000C7UL;

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
