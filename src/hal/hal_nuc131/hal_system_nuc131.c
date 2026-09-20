#include "hal/hal_system.h"
#include "NUC131.h"

static volatile uint32_t s_ticks_ms = 0;

void SysTick_Handler(void) {
    s_ticks_ms++;
}

hal_status_t hal_system_init(void) {
    // Unlock protected registers
    SYS_UnlockReg();

    // Enable internal 22.1184 MHz high-speed oscillator (HIRC)
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    // Configure PLL to 50 MHz: FIN = HIRC, FOUT = 50 MHz
    // PLLCON settings: OUT_DV = 0, IN_DV = 1, FB_DV = 31 (standard BSP macro)
    CLK->PLLCON = CLK_PLLCON_50MHz_HIRC;
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    // Switch HCLK to PLL
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_PLL;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;

    // Update SystemCoreClock CMSIS global
    SystemCoreClock = 50000000;

    // Relock registers
    SYS_LockReg();

    // Configure SysTick for 1ms
    SysTick_Config(SystemCoreClock / 1000);

    return HAL_OK;
}

uint32_t hal_get_tick_ms(void) {
    return s_ticks_ms;
}

void hal_delay_ms(uint32_t ms) {
    uint32_t start = s_ticks_ms;
    while ((s_ticks_ms - start) < ms);
}

void hal_system_reboot(void) {
    SYS_UnlockReg();
    SYS->IPRSTC1 |= SYS_IPRSTC1_CHIP_RST_Msk;
}
