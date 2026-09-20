#include "hal/hal_system.h"
#include "hal/hal_can.h"
#include "hal/hal_uart.h"
#include "hal/hal_gpio.h"
#include "core/can_router.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    hal_system_init();
    hal_gpio_init();
    hal_can_init(CAN_BAUD_500K);
    hal_uart_init(UART_BAUD_38400);

    can_router_init();

    uint32_t last_heartbeat = 0;

    while (1) {
        can_frame_t rx_can_frame;
        if (hal_can_receive(&rx_can_frame) == HAL_OK) {
            can_router_process_can(&rx_can_frame);
        }

        uint8_t uart_byte;
        while (hal_uart_read_byte(&uart_byte) == HAL_OK) {
            can_router_process_uart_byte(uart_byte);
        }

        uint32_t now = hal_get_tick_ms();
        if ((now - last_heartbeat) >= 100) {
            last_heartbeat = now;
            can_router_periodic_100ms();
            hal_gpio_toggle(GPIO_PIN_LED_STATUS);
        }

        // Relinquish remaining slice to prevent IDLE task watchdog trigger
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
