#include "hal/hal_system.h"
#include "hal/hal_can.h"
#include "hal/hal_gpio.h"
#include "core/can_router.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_iwdg.h"

#define STM32_HW_TIMEOUT 200000UL

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

void hal_system_enter_deep_sleep(void) {
    /* Suspend 1ms SysTick interrupt so CPU sleeps until CAN RX or UART RX interrupt */
    LL_SYSTICK_DisableIT();
    hal_watchdog_feed();
    __WFI();
    hal_watchdog_feed();
    LL_SYSTICK_EnableIT();
}
