#include "hal/hal_system.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_cortex.h"

static volatile uint32_t s_ticks_ms = 0;

void SysTick_Handler(void) {
    s_ticks_ms++;
}

hal_status_t hal_system_init(void) {
    // 72 MHz Clock Configuration (HSE 8MHz Crystal -> PLL x9)
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady());

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();
    while (!LL_RCC_PLL_IsReady());

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2); // APB1 = 36 MHz (CAN1, USART2)
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1); // APB2 = 72 MHz (USART1, GPIO)
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    // Configure SysTick for 1ms interrupts
    LL_Init1msTick(72000000);
    LL_SYSTICK_EnableIT();

    return HAL_STATUS_OK;
}

uint32_t hal_get_tick_ms(void) {
    return s_ticks_ms;
}

void hal_delay_ms(uint32_t ms) {
    uint32_t start = s_ticks_ms;
    while ((s_ticks_ms - start) < ms);
}

void hal_system_reboot(void) {
    NVIC_SystemReset();
}
