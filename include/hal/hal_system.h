#ifndef HAL_SYSTEM_H
#define HAL_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_STATUS_OK       =  0,
    HAL_STATUS_ERROR    = -1,
    HAL_STATUS_BUSY     = -2,
    HAL_STATUS_TIMEOUT  = -3,
    HAL_STATUS_OVERFLOW = -4
} hal_status_t;

#define HAL_OK       HAL_STATUS_OK
#define HAL_ERROR    HAL_STATUS_ERROR
#define HAL_BUSY     HAL_STATUS_BUSY
#define HAL_TIMEOUT  HAL_STATUS_TIMEOUT
#define HAL_OVERFLOW HAL_STATUS_OVERFLOW

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
