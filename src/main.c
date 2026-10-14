#include "hal/hal_system.h"
#include "hal/hal_can.h"
#include "hal/hal_uart.h"
#include "hal/hal_gpio.h"
#include "core/can_router.h"

int main(void) {
    hal_system_init();
    hal_gpio_init();
    hal_can_init(CAN_BAUD_125K); // 125 kbps standard for PSA CAN2004 Comfort bus
    hal_uart_init(UART_BAUD_38400); // 38400 baud standard for Raise/PSA protocol

    can_router_init();

    uint32_t last_heartbeat = 0;

    while (1) {
        bool activity = false;
        can_frame_t rx_can_frame;
        if (hal_can_receive(&rx_can_frame) == HAL_STATUS_OK) {
            can_router_process_can(&rx_can_frame);
            activity = true;
        }

        uint8_t uart_byte;
        while (hal_uart_read_byte(&uart_byte) == HAL_STATUS_OK) {
            can_router_process_uart_byte(uart_byte);
            activity = true;
        }

        uint32_t now = hal_get_tick_ms();
        if ((now - last_heartbeat) >= 100) {
            last_heartbeat = now;
            can_router_periodic_100ms();
            hal_gpio_toggle(GPIO_PIN_LED_STATUS);
            activity = true;
        }

        if (can_router_is_bus_sleeping()) {
            hal_system_enter_deep_sleep();
            can_router_notify_wake();
            last_heartbeat = hal_get_tick_ms();
        }

#if defined(PLATFORM_LINUX)
        if (!activity) {
            hal_delay_ms(1);
        }
#else
        (void)activity;
#endif
    }

    return 0;
}
