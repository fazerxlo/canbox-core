#include "hal/hal_system.h"
#include "hal/hal_can.h"
#include "hal/hal_gpio.h"
#include "core/can_router.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_iwdg.h"
#include "stm32f1xx_ll_gpio.h"

#define STM32_HW_TIMEOUT 200000UL

/* 72 MHz HCLK SysTick down-counter: 72 ticks per 1 us */
#define STM32_TICKS_PER_US             72UL
#define STM32_WAKE_INITIAL_WINDOW      (8UL * STM32_TICKS_PER_US)
#define STM32_WAKE_MIN_INITIAL_LOW     (5UL * STM32_TICKS_PER_US)
#define STM32_WAKE_HEADER_WINDOW       (80UL * STM32_TICKS_PER_US)
#define STM32_WAKE_MIN_BIT_LOW         (6UL * STM32_TICKS_PER_US)
#define STM32_WAKE_MAX_BIT_LOW         (48UL * STM32_TICKS_PER_US)

static volatile uint32_t s_ticks_ms = 0;

void SysTick_Handler(void) {
    s_ticks_ms++;
}

hal_status_t hal_system_init(void) {
    // 72 MHz Clock Configuration (HSE 8MHz Crystal -> PLL x9, fallback to HSI 64MHz if HSE absent/faulty)
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    LL_RCC_HSE_Enable();
    uint32_t timeout = STM32_HW_TIMEOUT;
    while (!LL_RCC_HSE_IsReady() && --timeout);

    if (LL_RCC_HSE_IsReady()) {
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    } else {
        LL_RCC_HSI_Enable();
        timeout = STM32_HW_TIMEOUT;
        while (!LL_RCC_HSI_IsReady() && --timeout);
#if defined(LL_RCC_PLL_MUL_16)
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
#else
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_9);
#endif
    }

    LL_RCC_PLL_Enable();
    timeout = STM32_HW_TIMEOUT;
    while (!LL_RCC_PLL_IsReady() && --timeout);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2); // APB1 = 36 MHz (CAN1, USART2)
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1); // APB2 = 72 MHz (USART1, GPIO)
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    timeout = STM32_HW_TIMEOUT;
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL && --timeout);

    // Configure Independent Watchdog (IWDG): 40 kHz LSI / 64 = 625 Hz -> 2000 reload = ~3.2s timeout
    LL_RCC_LSI_Enable();
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
    LL_IWDG_SetReloadCounter(IWDG, 2000U);
    LL_IWDG_ReloadCounter(IWDG);
    LL_IWDG_Enable(IWDG);

    // Configure SysTick for 1ms interrupts
    LL_Init1msTick(72000000);
    LL_SYSTICK_EnableIT();

    return HAL_STATUS_OK;
}

uint32_t hal_get_tick_ms(void) {
    return s_ticks_ms;
}

void hal_watchdog_feed(void) {
    LL_IWDG_ReloadCounter(IWDG);
}

void hal_delay_ms(uint32_t ms) {
    uint32_t start = s_ticks_ms;
    while ((s_ticks_ms - start) < ms) {
        hal_watchdog_feed();
    }
}

void hal_system_reboot(void) {
    hal_gpio_write(GPIO_PIN_CAN_STBY, true);
    hal_gpio_write(GPIO_PIN_ILL_OUT, false);
    hal_gpio_write(GPIO_PIN_REVERSE_OUT, false);
    NVIC_SystemReset();
}

static uint32_t systick_elapsed_ticks(uint32_t start_val, uint32_t current_val) {
    if (start_val >= current_val) {
        return start_val - current_val;
    }
    return start_val + (SysTick->LOAD + 1UL) - current_val;
}

static bool qualify_can_rx_wakeup_pulse(void) {
    uint32_t t_origin = SysTick->VAL;
    uint32_t consecutive_low_us = 0U;
    uint32_t t_low_start = t_origin;

    /* Step A: Sample PA11 (CAN_RX) at 1 us intervals over an 8 us window */
    for (uint32_t sample = 0U; sample < 8U; sample++) {
        if ((GPIOA->IDR & LL_GPIO_PIN_11) == 0U) {
            if (consecutive_low_us == 0U) {
                t_low_start = SysTick->VAL;
            }
            consecutive_low_us++;
            if (systick_elapsed_ticks(t_low_start, SysTick->VAL) >= STM32_WAKE_MIN_INITIAL_LOW) {
                return true;
            }
        } else if (consecutive_low_us > 0U) {
            break;
        } else if (systick_elapsed_ticks(t_origin, SysTick->VAL) >= STM32_WAKE_INITIAL_WINDOW) {
            break;
        }

        uint32_t t_step = SysTick->VAL;
        uint32_t step_guard = 300U;
        while ((systick_elapsed_ticks(t_step, SysTick->VAL) < STM32_TICKS_PER_US) && --step_guard) {
            if ((GPIOA->IDR & LL_GPIO_PIN_11) != 0U && consecutive_low_us > 0U) {
                break;
            }
        }
        if ((GPIOA->IDR & LL_GPIO_PIN_11) == 0U && consecutive_low_us > 0U) {
            if (systick_elapsed_ticks(t_low_start, SysTick->VAL) >= STM32_WAKE_MIN_INITIAL_LOW) {
                return true;
            }
        }
    }

    /* Step B: Monitor PA11 for up to 80 us for a second 6..48 us dominant pulse */
    uint32_t wait_high_guard = 2000U;
    while (((GPIOA->IDR & LL_GPIO_PIN_11) == 0U) &&
           (systick_elapsed_ticks(t_origin, SysTick->VAL) < STM32_WAKE_HEADER_WINDOW) &&
           --wait_high_guard);

    uint32_t window_guard = 12000U;
    while ((systick_elapsed_ticks(t_origin, SysTick->VAL) < STM32_WAKE_HEADER_WINDOW) && --window_guard) {
        if ((GPIOA->IDR & LL_GPIO_PIN_11) == 0U) {
            uint32_t edge_start = SysTick->VAL;
            uint32_t pulse_guard = 8000U;
            while (((GPIOA->IDR & LL_GPIO_PIN_11) == 0U) && --pulse_guard) {
                uint32_t low_ticks = systick_elapsed_ticks(edge_start, SysTick->VAL);
                if (low_ticks > (STM32_WAKE_MAX_BIT_LOW + STM32_TICKS_PER_US)) {
                    break;
                }
            }
            uint32_t low_ticks = systick_elapsed_ticks(edge_start, SysTick->VAL);
            if (low_ticks >= STM32_WAKE_MIN_BIT_LOW && low_ticks <= STM32_WAKE_MAX_BIT_LOW) {
                return true;
            }
            if (low_ticks > STM32_WAKE_MAX_BIT_LOW) {
                break;
            }
        }
    }

    return false;
}

void hal_system_enter_deep_sleep(void) {
    /* Place CAN transceiver in standby before sleeping */
    hal_gpio_write(GPIO_PIN_CAN_STBY, true);

    /* Suspend 1ms SysTick interrupt so CPU sleeps until CAN RX or UART RX interrupt */
    LL_SYSTICK_DisableIT();
    hal_watchdog_feed();
    __WFI();
    hal_watchdog_feed();

    if (hal_gpio_read(GPIO_PIN_IGNITION_IN) ||
        ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U) ||
        (((GPIOA->IDR & LL_GPIO_PIN_11) == 0U) && qualify_can_rx_wakeup_pulse())) {
        hal_gpio_write(GPIO_PIN_CAN_STBY, false);
        can_router_notify_wake();
    }

    LL_SYSTICK_EnableIT();
}
