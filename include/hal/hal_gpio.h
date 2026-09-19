#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>
#include "hal_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPIO_PIN_LED_STATUS = 0,
    GPIO_PIN_CAN_STBY,       // Transceiver standby/silent control
    GPIO_PIN_IGNITION_IN,    // Accessory/Ignition 12V detect (via opto/divider)
    GPIO_PIN_HEADUNIT_POWER  // Optional switched 12V supply enable
} hal_gpio_pin_t;

hal_status_t hal_gpio_init(void);
void hal_gpio_write(hal_gpio_pin_t pin, bool state);
bool hal_gpio_read(hal_gpio_pin_t pin);
void hal_gpio_toggle(hal_gpio_pin_t pin);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
