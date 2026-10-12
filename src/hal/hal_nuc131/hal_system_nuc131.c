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
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    // Set HCLK source to HIRC (22.1184 MHz) with divider 1
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    // Configure PLL to 48 MHz: FIN = HIRC, FOUT = 48 MHz
    // Uses CLK_SetCoreClock() which internally sets up PLL
    CLK_SetCoreClock(48000000);

    // Update SystemCoreClock CMSIS global
    SystemCoreClock = 48000000;

    // Relock registers
    SYS_LockReg();

    // Configure SysTick for exact 1ms ticks:
    // Core Clock (HCLK) = 48 MHz -> 48,000 counts per 1 ms
    CLK_EnableSysTick(CLK_CLKSEL0_STCLK_S_HCLK, SystemCoreClock / 1000);

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
    SYS_UnlockReg();
    SYS_ResetChip();
}
