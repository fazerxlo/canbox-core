#ifndef HAL_SYSTEM_H
#define HAL_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_OK       =  0,
    HAL_ERROR    = -1,
    HAL_BUSY     = -2,
    HAL_TIMEOUT  = -3,
    HAL_OVERFLOW = -4
} hal_status_t;

/**
 * @brief Initialize system clocks, NVIC, and low-level peripherals.
 */
hal_status_t hal_system_init(void);

/**
 * @brief Returns elapsed monotonic milliseconds since boot.
 * Must be non-blocking and safe to call from any context.
 */
uint32_t hal_get_tick_ms(void);

/**
 * @brief Busy delay for early peripheral setup (avoid in main loop).
 */
void hal_delay_ms(uint32_t ms);

/**
 * @brief Software reset trigger.
 */
void hal_system_reboot(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SYSTEM_H */
