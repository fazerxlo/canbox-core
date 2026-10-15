#include "hal/hal_system.h"
#include "NUC131.h"

static volatile uint32_t s_ticks_ms = 0;

void SysTick_Handler(void) {
    s_ticks_ms++;
}

/* ISR Handlers for wake-up sources to clear flags and avoid Default_Handler hang */
void PWRWU_IRQHandler(void) {
    CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk; /* Clear power-down wake-up status */
}

void GPAB_IRQHandler(void) {
    PA->ISRC = PA->ISRC; /* Clear Port A interrupt flags (e.g. PA0 ignition) */
    PB->ISRC = PB->ISRC;
}

void GPCDEF_IRQHandler(void) {
    PC->ISRC = PC->ISRC;
    PD->ISRC = PD->ISRC; /* Clear Port D interrupt flags (e.g. PD6 CAN_RX) */
    PE->ISRC = PE->ISRC;
    PF->ISRC = PF->ISRC;
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
    /* Safe Hardware Isolation Sequence prior to Reset:
     * 1. Assert ACC ON (PA8 = HIGH) so Head Unit remains awake during flash/reboot
     * 2. De-assert other outputs (PA9 ILL, PA12 BRAKE, PA13 REV = LOW)
     * 3. Silence buzzer drive lines (PB3, PB12 driven HIGH)
     * 4. Assert CAN transceiver Standby (PC3 = HIGH) to isolate differential CAN bus
     * 5. Disconnect CAN0 peripheral from PD6/PD7 and drive PD7 push-pull recessive HIGH
     * 6. Wait for UART0 TX FIFO to drain completely
     */
    PA->DOUT = (PA->DOUT & ~((1U << 9) | (1U << 12) | (1U << 13))) | (1U << 8);
    PB->DOUT |= (1U << 3) | (1U << 12);
    PC->DOUT |= (1U << 3);

    SYS_UnlockReg();
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    PD->DOUT |= (1U << 7);
    PD->PMD = (PD->PMD & ~GPIO_PMD_PMD7_Msk) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD7_Pos);

    /* Wait for UART0 TX FIFO and shift register to drain */
    while (!(UART0->FSR & UART_FSR_TE_FLAG_Msk));

    /* Brief settling delay before asserting chip reset */
    for (volatile int i = 0; i < 50000; i++);

    SYS_ResetChip();
    while (1);
}

static void hal_system_on_wakeup(void) {
    SYS_UnlockReg();

    // Clear Power-Down flag
    CLK->PWRCON &= ~CLK_PWRCON_PWR_DOWN_EN_Msk;

    // Restore PLL to 48 MHz and wait for oscillator stability
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    SYS_LockReg();

    // Restore CAN transceiver active drive (PC3 = LOW)
    PC->DOUT &= ~(1U << 3);

    // Re-arm SysTick for exact 1ms task scheduler
    CLK_EnableSysTick(CLK_CLKSEL0_STCLK_S_HCLK, SystemCoreClock / 1000);

    // Disable wakeup interrupts to prevent re-triggering during active run
    GPIO_DisableInt(PD, 6);
    GPIO_DisableInt(PA, 0);
    GPIO_DisableInt(PB, 0);
    CAN_DISABLE_WAKEUP(CAN0);

    NVIC_ClearPendingIRQ(GPCDEF_IRQn);
    NVIC_ClearPendingIRQ(GPAB_IRQn);
    NVIC_ClearPendingIRQ(PWRWU_IRQn);
    NVIC_ClearPendingIRQ(CAN0_IRQn);
}

void hal_system_enter_deep_sleep(void) {
    /* 1. Turn off all discrete vehicle outputs */
    PA->DOUT &= ~((1U << 8) | (1U << 9) | (1U << 12) | (1U << 13));

    /* 2. Put CAN transceiver in Standby (PC3 = HIGH) */
    PC->DOUT |= (1U << 3);

    /* 3. Disable SysTick timer to prevent 1ms wakeups */
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;

    /* 4. Configure Wakeup Interrupt Sources */
    SYS_UnlockReg();

    // Enable hardware CAN Wake-up on CAN0 Rx edge
    CAN_ENABLE_WAKEUP(CAN0);
    NVIC_ClearPendingIRQ(CAN0_IRQn);
    NVIC_EnableIRQ(CAN0_IRQn);

    // Enable EXTI on PD6 (CAN_RX) to wake on dominant CAN bus pulse
    GPIO_EnableInt(PD, 6, GPIO_INT_FALLING);
    NVIC_ClearPendingIRQ(GPCDEF_IRQn);
    NVIC_EnableIRQ(GPCDEF_IRQn);

    // Enable EXTI on PA0 (Hardware Ignition Sense line, if wired)
    GPIO_EnableInt(PA, 0, GPIO_INT_RISING);
    NVIC_ClearPendingIRQ(GPAB_IRQn);
    NVIC_EnableIRQ(GPAB_IRQn);

    // Enable EXTI on PB0 (UART0_RXD) to wake on incoming Head Unit / Flasher serial commands
    GPIO_EnableInt(PB, 0, GPIO_INT_FALLING);
    NVIC_ClearPendingIRQ(GPAB_IRQn);
    NVIC_EnableIRQ(GPAB_IRQn);

    // Enable Power-down Wake-up Interrupt in Clock Controller
    CLK->PWRCON |= CLK_PWRCON_PD_WU_INT_EN_Msk;
    NVIC_ClearPendingIRQ(PWRWU_IRQn);
    NVIC_EnableIRQ(PWRWU_IRQn);

    /* 5. Enable Cortex-M0 Deep Sleep in SCB */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* 6. Enable Chip Power-Down mode and wait for CPU WFI */
    CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk);

    SYS_LockReg();

    /* 7. Stop CPU core until external interrupt occurs */
    __asm volatile("wfi");

    /* ========================================================= */
    /*   MCU WAKES UP HERE IMMEDIATELY UPON BUS/KEY TRANSITION   */
    /* ========================================================= */

    hal_system_on_wakeup();
}
