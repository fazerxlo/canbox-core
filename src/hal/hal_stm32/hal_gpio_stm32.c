#include "hal/hal_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

hal_status_t hal_gpio_init(void) {
    // Enable GPIO Port clocks
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

    LL_GPIO_InitTypeDef gpio_init;
    LL_GPIO_StructInit(&gpio_init);

    // Status LED: PC13 (Output Push-Pull)
    gpio_init.Pin = LL_GPIO_PIN_13;
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOC, &gpio_init);
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13); // Default off (active low on BluePill)

    // CAN STBY: PB0 (Output Push-Pull, default LOW -> normal mode)
    gpio_init.Pin = LL_GPIO_PIN_0;
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &gpio_init);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

    // Head Unit Power Enable: PB1 (Output Push-Pull, default HIGH)
    gpio_init.Pin = LL_GPIO_PIN_1;
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &gpio_init);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);

    // Ignition IN: PB12 (Input Pull-Down)
    gpio_init.Pin = LL_GPIO_PIN_12;
    gpio_init.Mode = LL_GPIO_MODE_INPUT;
    gpio_init.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOB, &gpio_init);

    return HAL_STATUS_OK;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool state) {
    switch (pin) {
        case GPIO_PIN_LED_STATUS:
            if (state) {
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13); // Active LOW on BluePill
            } else {
                LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
            }
            break;
        case GPIO_PIN_CAN_STBY:
            if (state) {
                LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
            } else {
                LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
            }
            break;
        case GPIO_PIN_HEADUNIT_POWER:
            if (state) {
                LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
            } else {
                LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
            }
            break;
        case GPIO_PIN_IGNITION_IN:
        default:
            break;
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin) {
    switch (pin) {
        case GPIO_PIN_LED_STATUS:
            return LL_GPIO_IsOutputPinSet(GPIOC, LL_GPIO_PIN_13) == 0;
        case GPIO_PIN_CAN_STBY:
            return LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_0) != 0;
        case GPIO_PIN_HEADUNIT_POWER:
            return LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_1) != 0;
        case GPIO_PIN_IGNITION_IN:
            return LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_12) != 0;
        default:
            return false;
    }
}

void hal_gpio_toggle(hal_gpio_pin_t pin) {
    switch (pin) {
        case GPIO_PIN_LED_STATUS:
            LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
            break;
        case GPIO_PIN_CAN_STBY:
            LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
            break;
        case GPIO_PIN_HEADUNIT_POWER:
            LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_1);
            break;
        case GPIO_PIN_IGNITION_IN:
        default:
            break;
    }
}
