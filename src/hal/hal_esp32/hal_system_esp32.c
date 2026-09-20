#include "hal/hal_system.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

hal_status_t hal_system_init(void) {
    // Clocks and core peripherals are brought up by ESP-IDF secondary bootloader
    return HAL_STATUS_OK;
}

uint32_t hal_get_tick_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void hal_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void hal_system_reboot(void) {
    esp_restart();
}
