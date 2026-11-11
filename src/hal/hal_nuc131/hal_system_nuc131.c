#include "hal/hal_system.h"
#include "hal/hal_can.h"
#include "hal/hal_uart.h"
#include "core/can_router.h"
#include "NUC131.h"

#define HW_POLL_TIMEOUT 200000UL

/* 22.1184 MHz HIRC: ~22.12 ticks per 1 us (TIMER1 with prescale = 0) */
#define NUC_HIRC_TICKS_PER_US          22UL
#define CAN_WAKE_INITIAL_WINDOW_TICKS  (8UL * NUC_HIRC_TICKS_PER_US)
#define CAN_WAKE_MIN_INITIAL_LOW_TICKS (5UL * NUC_HIRC_TICKS_PER_US)
#define CAN_WAKE_HEADER_WINDOW_TICKS   (80UL * NUC_HIRC_TICKS_PER_US)
#define CAN_WAKE_MIN_BIT_LOW_TICKS     (6UL * NUC_HIRC_TICKS_PER_US)
#define CAN_WAKE_MAX_BIT_LOW_TICKS     (48UL * NUC_HIRC_TICKS_PER_US)

static volatile uint32_t s_ticks_ms = 0;

/* Shared with hal_can_nuc131.c and latched by ISRs if an interrupt fires during the post-wake listen window */
extern volatile bool g_nuc131_can_woke_flag;
static volatile bool s_irq_woke_by_can = false;

void SysTick_Handler(void) {
    s_ticks_ms++;
}

void WDT_IRQHandler(void) {
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
}

void BOD_IRQHandler(void) {
    SYS_CLEAR_BOD_INT_FLAG();
}

/* ISR Handlers for wake-up sources to latch wake reason and clear flags */
void PWRWU_IRQHandler(void) {
    CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk; /* Clear power-down wake-up status */
}

void GPAB_IRQHandler(void) {
    uint32_t pa_isrc = PA->ISRC;
    uint32_t pb_isrc = PB->ISRC;

    /* Clear interrupt sources immediately */
    PA->ISRC = pa_isrc;
    PB->ISRC = pb_isrc;

    /* Only latch CAN/Ignition wake if PA.0 triggered AND PA.0 is actively HIGH */
    if ((pa_isrc & (1U << 0)) && ((PA->PIN & (1U << 0)) != 0U)) {
        s_irq_woke_by_can = true;
    }
}

void GPCDEF_IRQHandler(void) {
    uint32_t pd_isrc = PD->ISRC;

    PC->ISRC = PC->ISRC;
    PD->ISRC = pd_isrc;
    PE->ISRC = PE->ISRC;
    PF->ISRC = PF->ISRC;

    /* Latch CAN wake if PD.6 (CAN_RX) triggered a falling-edge interrupt */
    if (pd_isrc & (1U << 6)) {
        s_irq_woke_by_can = true;
    }
}

hal_status_t hal_system_init(void) {
    // Unlock protected registers
    SYS_UnlockReg();

    // Enable Brown-Out Detector (BOD) at 2.7V with automatic hardware Reset
    SYS_EnableBOD(SYS_BODCR_BOD_RST_EN, SYS_BODCR_BOD_VL_2_7V);

    // Enable internal 22.1184 MHz high-speed oscillator (HIRC) and 10 kHz LIRC (for WDT)
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk);
    uint32_t timeout = HW_POLL_TIMEOUT;
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk) && --timeout);

    // Set HCLK source to HIRC (22.1184 MHz) with divider 1
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    // Configure PLL to 48 MHz: FIN = HIRC, FOUT = 48 MHz
    CLK_SetCoreClock(48000000);

    // Update SystemCoreClock CMSIS global
    SystemCoreClock = 48000000;

    // Enable WDT peripheral clock (10 kHz LIRC) and open WDT with 2^14 (~1.6s) timeout
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, 0);
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_130CLK, TRUE, FALSE);

    // Enable TIMER1 clocked directly from 22.1184 MHz HIRC for sub-microsecond CAN wake qualification
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, 0);

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

void hal_watchdog_feed(void) {
    SYS_UnlockReg();
    WDT_RESET_COUNTER();
    SYS_LockReg();
}

void hal_delay_ms(uint32_t ms) {
    uint32_t start = s_ticks_ms;
    while ((s_ticks_ms - start) < ms) {
        hal_watchdog_feed();
    }
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

    /* Wait for UART0 TX FIFO and shift register to drain (bounded timeout) */
    uint32_t timeout = HW_POLL_TIMEOUT;
    while (!(UART0->FSR & UART_FSR_TE_FLAG_Msk) && --timeout);

    /* Brief settling delay before asserting chip reset */
    for (volatile int i = 0; i < 50000; i++);

    SYS_ResetChip();
    while (1);
}

static bool qualify_can_rx_wakeup_pulse(void) {
    /* Ensure TIMER1 is running in 24-bit continuous mode clocked directly from 22.1184 MHz HIRC (~22.12 ticks/us) */
    if ((TIMER1->TCSR & (TIMER_TCSR_CEN_Msk | TIMER_TCSR_TDR_EN_Msk)) !=
        (TIMER_TCSR_CEN_Msk | TIMER_TCSR_TDR_EN_Msk)) {
        TIMER1->TCMPR = 0xFFFFFFUL;
        TIMER1->TCSR  = TIMER_CONTINUOUS_MODE | TIMER_TCSR_TDR_EN_Msk | TIMER_TCSR_CEN_Msk;
    }

    uint32_t t_origin = TIMER1->TDR & 0xFFFFFFUL;

    /* Step A (Initial Glitch Check):
     * Sample PD.6 (CAN_RX) at 1 us intervals over a bounded 8 us window.
     * Accounting for ~2-3 us HIRC wakeup latency after WFI, a genuine 8.0 us
     * 125 kbps dominant bit holds PD.6 LOW for >= 5.0 us. */
    uint32_t consecutive_low_us = 0U;
    uint32_t t_low_start = t_origin;

    for (uint32_t sample = 0U; sample < 8U; sample++) {
        if ((PD->PIN & (1U << 6)) == 0U) {
            if (consecutive_low_us == 0U) {
                t_low_start = TIMER1->TDR & 0xFFFFFFUL;
            }
            consecutive_low_us++;
            uint32_t low_ticks = (TIMER1->TDR - t_low_start) & 0xFFFFFFUL;
            if (low_ticks >= CAN_WAKE_MIN_INITIAL_LOW_TICKS) {
                return true;
            }
        } else if (consecutive_low_us > 0U) {
            /* Initial LOW pulse ended before reaching 5.0 us; proceed to Step B */
            break;
        } else if (((TIMER1->TDR - t_origin) & 0xFFFFFFUL) >= CAN_WAKE_INITIAL_WINDOW_TICKS) {
            break;
        }

        /* Wait 1 us (~22 HIRC ticks) while monitoring for premature rising edge */
        uint32_t t_step = TIMER1->TDR & 0xFFFFFFUL;
        uint32_t step_guard = 200U;
        while ((((TIMER1->TDR - t_step) & 0xFFFFFFUL) < NUC_HIRC_TICKS_PER_US) && --step_guard) {
            if ((PD->PIN & (1U << 6)) != 0U && consecutive_low_us > 0U) {
                break;
            }
        }
        if ((PD->PIN & (1U << 6)) == 0U && consecutive_low_us > 0U) {
            uint32_t low_ticks = (TIMER1->TDR - t_low_start) & 0xFFFFFFUL;
            if (low_ticks >= CAN_WAKE_MIN_INITIAL_LOW_TICKS) {
                return true;
            }
        }
    }

    /* Step B (Second Dominant Bit Confirmation within 1 Frame Header):
     * If the first pulse was already ending as the CPU woke up (e.g. a single 8 us SOF
     * bit followed by recessive ID bits), monitor PD.6 for up to 80 us (10 bit times
     * at 125 kbps) for the next falling edge (1 -> 0) and verify its LOW duration is
     * between 6.0 us and 48.0 us (1 to 6 consecutive dominant bits). */
    uint32_t wait_high_guard = 1000U;
    while (((PD->PIN & (1U << 6)) == 0U) &&
           (((TIMER1->TDR - t_origin) & 0xFFFFFFUL) < CAN_WAKE_HEADER_WINDOW_TICKS) &&
           --wait_high_guard);

    uint32_t window_guard = 8000U;
    while ((((TIMER1->TDR - t_origin) & 0xFFFFFFUL) < CAN_WAKE_HEADER_WINDOW_TICKS) && --window_guard) {
        if ((PD->PIN & (1U << 6)) == 0U) {
            uint32_t edge_start = TIMER1->TDR & 0xFFFFFFUL;
            uint32_t pulse_guard = 4000U;
            while (((PD->PIN & (1U << 6)) == 0U) && --pulse_guard) {
                uint32_t low_ticks = (TIMER1->TDR - edge_start) & 0xFFFFFFUL;
                if (low_ticks > (CAN_WAKE_MAX_BIT_LOW_TICKS + NUC_HIRC_TICKS_PER_US)) {
                    break; /* Reject stuck-dominant bus fault (> 6 dominant bits) */
                }
            }
            uint32_t low_ticks = (TIMER1->TDR - edge_start) & 0xFFFFFFUL;
            if (low_ticks >= CAN_WAKE_MIN_BIT_LOW_TICKS &&
                low_ticks <= CAN_WAKE_MAX_BIT_LOW_TICKS) {
                return true;
            }
            if (low_ticks > CAN_WAKE_MAX_BIT_LOW_TICKS) {
                break;
            }
        }
    }

    /* Step C (Noise Rejection): No valid >= 5-6 us dominant bit observed */
    return false;
}

static void hal_system_on_wakeup(bool woke_by_can) {
    SYS_UnlockReg();

    // Clear Power-Down flag
    CLK->PWRCON &= ~CLK_PWRCON_PWR_DOWN_EN_Msk;

    // Restore PLL to 48 MHz and wait for oscillator stability (bounded timeout)
    uint32_t timeout = HW_POLL_TIMEOUT;
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk) && --timeout);

    // Re-enable WDT after waking from deep sleep
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_130CLK, TRUE, FALSE);

    SYS_LockReg();

    // Only wake CAN transceiver immediately if wakeup was triggered by CAN / Ignition;
    // if woken by UART noise/traffic while car is asleep, keep CAN transceiver in standby
    // until an actual CAN frame arrives.
    if (woke_by_can) {
        PC->DOUT &= ~(1U << 3);
        can_router_notify_wake();
    }

    // Re-arm SysTick for exact 1ms task scheduler
    CLK_EnableSysTick(CLK_CLKSEL0_STCLK_S_HCLK, SystemCoreClock / 1000);
}

void hal_system_enter_deep_sleep(void) {
    /* 1. Turn off all discrete vehicle outputs (ACC PA8, ILL PA9, BRAKE PA12, REV PA13) */
    PA->DOUT &= ~((1U << 8) | (1U << 9) | (1U << 12) | (1U << 13));

    /* 2. Put CAN transceiver in Standby (PC3 = HIGH) and abort any stuck CAN TX */
    PC->DOUT |= (1U << 3);
    CAN0->CON |= CAN_CON_DAR_Msk;

    /* Allow brief settling time after cutting PA8 (ACC) so Head Unit TX voltage decay
     * on PB0 (UART0_RXD) does not immediately trigger a false falling-edge wake interrupt */
    for (volatile int i = 0; i < 10000; i++);

    /* 3. Disable SysTick timer to prevent 1ms wakeups */
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;

    /* 4. Mask global interrupts via PRIMASK before arming wake sources.
     * On Cortex-M0, enabled NVIC interrupts still wake the core from WFI when PRIMASK=1,
     * allowing deterministic inspection of raw ISRC/WU_STATUS flags before any ISR runs. */
    __disable_irq();

    SYS_UnlockReg();

    // Pause WDT during deep sleep so it does not reset the sleeping MCU every 1.6s
    WDT_Close();

    // Enable Brown-Out Detector Low-Power Mode during deep sleep
    SYS_SET_BOD_LPM();

    // Enable hardware CAN Wake-up on CAN0 Rx edge and clear any stale wake status
    CAN0->WU_STATUS = 0;
    CAN_ENABLE_WAKEUP(CAN0);

    // Enable EXTI on PD6 (CAN_RX), PA0 (Ignition), and PB0 (UART0_RXD)
    GPIO_EnableInt(PD, 6, GPIO_INT_FALLING);
    GPIO_EnableInt(PA, 0, GPIO_INT_RISING);
    GPIO_EnableInt(PB, 0, GPIO_INT_FALLING);

    // Clear all GPIO interrupt status flags and NVIC pending bits AFTER enabling EXTI
    // so no stale edge prior to sleep can falsely classify the wake source
    PA->ISRC = PA->ISRC;
    PB->ISRC = PB->ISRC;
    PC->ISRC = PC->ISRC;
    PD->ISRC = PD->ISRC;
    PE->ISRC = PE->ISRC;
    PF->ISRC = PF->ISRC;

    s_irq_woke_by_can = false;
    g_nuc131_can_woke_flag = false;

    NVIC_ClearPendingIRQ(CAN0_IRQn);
    NVIC_ClearPendingIRQ(GPCDEF_IRQn);
    NVIC_ClearPendingIRQ(GPAB_IRQn);
    NVIC_ClearPendingIRQ(PWRWU_IRQn);

    NVIC_EnableIRQ(CAN0_IRQn);
    NVIC_EnableIRQ(GPCDEF_IRQn);
    NVIC_EnableIRQ(GPAB_IRQn);

    // Enable Power-down Wake-up Interrupt in Clock Controller
    CLK->PWRCON |= CLK_PWRCON_PD_WU_INT_EN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);

    // Ensure TIMER1 is armed on 22.1184 MHz HIRC for immediate post-WFI pulse qualification
    CLK->APBCLK |= CLK_APBCLK_TMR1_EN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR1_S_Msk) | CLK_CLKSEL1_TMR1_S_HIRC;
    TIMER1->TCMPR = 0xFFFFFFUL;
    TIMER1->TCSR  = TIMER_CONTINUOUS_MODE | TIMER_TCSR_TDR_EN_Msk | TIMER_TCSR_CEN_Msk;

    /* 5. Enable Cortex-M0 Deep Sleep in SCB */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    bool woke_by_can = false;

    for (;;) {
        // Clear all GPIO interrupt status flags, CAN wake status, and NVIC pending bits
        // immediately prior to entering Power-Down WFI
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
        PC->ISRC = PC->ISRC;
        PD->ISRC = PD->ISRC;
        PE->ISRC = PE->ISRC;
        PF->ISRC = PF->ISRC;
        CAN0->WU_STATUS = 0;
        CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk;

        NVIC_ClearPendingIRQ(CAN0_IRQn);
        NVIC_ClearPendingIRQ(GPCDEF_IRQn);
        NVIC_ClearPendingIRQ(GPAB_IRQn);
        NVIC_ClearPendingIRQ(PWRWU_IRQn);

        /* 6. Enable Chip Power-Down mode and wait for CPU WFI */
        SYS_UnlockReg();
        CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk);
        SYS_LockReg();

        /* 7. Stop CPU core until external interrupt occurs */
        __asm volatile("wfi");

        /* ========================================================= */
        /*   MCU WAKES UP HERE IMMEDIATELY UPON BUS/KEY TRANSITION   */
        /* ========================================================= */

        /* Ensure Power-Down bit is cleared and 22.1184 MHz HIRC is stable so GPIO
         * and TIMER1 registers are clocked before sampling pins or pulse width */
        SYS_UnlockReg();
        CLK->PWRCON &= ~CLK_PWRCON_PWR_DOWN_EN_Msk;
        SYS_LockReg();
        uint32_t hirc_wait = 1000U;
        while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk) && --hirc_wait);

        /* Sample raw hardware wake flags while PRIMASK=1 before any ISR clears them:
         * 1. PA.0 rising edge (Ignition sense) verified with PA.0 pin actively HIGH
         * 2. PB.0 falling edge (UART0_RXD start bit)
         * 3. PD.6 falling edge (CAN_RX) or CAN0 hardware wake-up status bit */
        bool pa_wake  = ((PA->ISRC & (1U << 0)) != 0U) && ((PA->PIN & (1U << 0)) != 0U);
        bool pb_wake  = (PB->ISRC & (1U << 0)) != 0U;
        bool can_edge = ((PD->ISRC & (1U << 6)) != 0U) ||
                        ((CAN0->WU_STATUS & CAN_WUSTATUS_WAKUP_STS_Msk) != 0U) ||
                        ((PD->PIN & (1U << 6)) == 0U);

        if (pa_wake) {
            woke_by_can = true;
            break;
        }

        if (can_edge) {
            bool valid_can_bit = qualify_can_rx_wakeup_pulse();
            if (valid_can_bit) {
                woke_by_can = true;
                break;
            }
            if (!pb_wake) {
                /* Sub-bit electrical noise rejected (< 50 us):
                 * Clear wake flags and immediately re-enter deep sleep without waking
                 * CAN transceiver (PC.3 stays HIGH), PLL/SysTick, or can_router. */
                PD->ISRC = (1U << 6);
                CAN0->WU_STATUS = 0;
                CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk;
                NVIC_ClearPendingIRQ(CAN0_IRQn);
                NVIC_ClearPendingIRQ(GPCDEF_IRQn);
                NVIC_ClearPendingIRQ(PWRWU_IRQn);
                continue;
            }
        }

        if (pb_wake) {
            woke_by_can = false;
            break;
        }
    }

    /* Disarm wake-only GPIO interrupts and clear pending flags before re-enabling IRQs */
    GPIO_DisableInt(PD, 6);
    GPIO_DisableInt(PA, 0);
    GPIO_DisableInt(PB, 0);
    CAN_DISABLE_WAKEUP(CAN0);

    PA->ISRC = PA->ISRC;
    PB->ISRC = PB->ISRC;
    PD->ISRC = PD->ISRC;
    CAN0->WU_STATUS = 0;
    CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk;

    NVIC_ClearPendingIRQ(CAN0_IRQn);
    NVIC_ClearPendingIRQ(GPCDEF_IRQn);
    NVIC_ClearPendingIRQ(GPAB_IRQn);
    NVIC_ClearPendingIRQ(PWRWU_IRQn);

    CAN0->CON &= ~CAN_CON_DAR_Msk;
    SYS_UnlockReg();
    SYS_CLEAR_BOD_LPM();
    SYS_LockReg();

    hal_system_on_wakeup(woke_by_can);

    /* Re-enable global interrupts so SysTick, UART0, and CAN0 ISRs run normally */
    __enable_irq();

    /* If woken by UART RX (PB0), wait up to 50 ms for any incoming UART command
     * (such as 0xE0 firmware upgrade trigger) before re-entering deep sleep */
    if (!woke_by_can) {
        uint32_t wake_listen_start = s_ticks_ms;
        while ((s_ticks_ms - wake_listen_start) < 50) {
            uint8_t b;
            while (hal_uart_read_byte(&b) == HAL_STATUS_OK) {
                can_router_process_uart_byte(b);
                wake_listen_start = s_ticks_ms; /* Extend window while active packet streams */
            }
            if (s_irq_woke_by_can || g_nuc131_can_woke_flag ||
                (((PD->PIN & (1U << 6)) == 0U) && qualify_can_rx_wakeup_pulse())) {
                PC->DOUT &= ~(1U << 3);
                can_router_notify_wake();
                break;
            }
            can_frame_t rx_can;
            if (hal_can_receive(&rx_can) == HAL_STATUS_OK) {
                PC->DOUT &= ~(1U << 3);
                can_router_notify_wake();
                can_router_process_can(&rx_can);
                break;
            }
            hal_watchdog_feed();
        }
    }

    TIMER1->TCSR = 0;
}
