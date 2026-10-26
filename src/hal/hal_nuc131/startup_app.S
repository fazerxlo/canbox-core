    .syntax unified
    .arch armv6-m

    .section .vectors, "a"
    .align 2
    .globl __Vectors
__Vectors:
    .long   _estack            /* Top of Stack */
    .long   Reset_Handler      /* Reset Handler */
    .long   NMI_Handler        /* NMI Handler */
    .long   HardFault_Handler  /* Hard Fault Handler */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   SVC_Handler        /* SVCall Handler */
    .long   0                  /* Reserved */
    .long   0                  /* Reserved */
    .long   PendSV_Handler     /* PendSV Handler */
    .long   SysTick_Handler    /* SysTick Handler */

    /* External Interrupts (32 IRQs matching NUC131) */
    .long   BOD_IRQHandler     /* 0: Brown-out detector */
    .long   WDT_IRQHandler     /* 1: Watchdog timer */
    .long   EINT0_IRQHandler   /* 2: External interrupt 0 */
    .long   EINT1_IRQHandler   /* 3: External interrupt 1 */
    .long   GPAB_IRQHandler    /* 4: External interrupt from GPA / GPB */
    .long   GPCDEF_IRQHandler  /* 5: External interrupt from GPC / GPD / GPE / GPF */
    .long   Default_Handler    /* 6: Reserved */
    .long   Default_Handler    /* 7: Reserved */
    .long   TMR0_IRQHandler    /* 8: Timer 0 */
    .long   TMR1_IRQHandler    /* 9: Timer 1 */
    .long   TMR2_IRQHandler    /* 10: Timer 2 */
    .long   TMR3_IRQHandler    /* 11: Timer 3 */
    .long   UART02_IRQHandler  /* 12: UART0 and UART2 */
    .long   UART1_IRQHandler   /* 13: UART1 */
    .long   SPI0_IRQHandler    /* 14: SPI0 */
    .long   UART3_IRQHandler   /* 15: UART3 */
    .long   UART4_IRQHandler   /* 16: UART4 */
    .long   UART5_IRQHandler   /* 17: UART5 */
    .long   I2C0_IRQHandler    /* 18: I2C0 */
    .long   I2C1_IRQHandler    /* 19: I2C1 */
    .long   CAN0_IRQHandler    /* 20: CAN0 */
    .long   CAN1_IRQHandler    /* 21: CAN1 */
    .long   PWM0_IRQHandler    /* 22: PWM0 */
    .long   PWM1_IRQHandler    /* 23: PWM1 */
    .long   BPWM0_IRQHandler   /* 24: BPWM0 */
    .long   BPWM1_IRQHandler   /* 25: BPWM1 */
    .long   BRAKE0_IRQHandler  /* 26: BRAKE0 */
    .long   BRAKE1_IRQHandler  /* 27: BRAKE1 */
    .long   PWRWU_IRQHandler   /* 28: Power down wake up */
    .long   ADC_IRQHandler     /* 29: ADC */
    .long   CKD_IRQHandler     /* 30: Clock detection */
    .long   RTC_IRQHandler     /* 31: RTC */

    .text
    .thumb
    .align 2
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Disable interrupts */
    cpsid i

    /* Set Stack Pointer */
    ldr r0, =_estack
    msr msp, r0

    /* Silence buzzer, keep CAN transceiver in Standby (PC3=1), apply factory GPIO outputs:
     * PC_DOUT = 0x00cf, PC_PMD = 0x0000f045 (PC0=1, PC1=1, PC2=1, PC3=1 Standby, PC6=1, PC7=1)
     * PB_DOUT = 0x113f, PB_PMD = 0x43030ff4 (PB1=1 TX, PB3=1, PB12=1, PB15=0)
     * PA_DOUT = 0x4cff, PA_PMD = 0x75f5ff0f (PA15=0, PA8/9/12/13=0)
     * PD_DOUT = 0x00c0, PD_PMD = 0x00007000 (PD7=1 CAN_TX Recessive Push-Pull, PD6=Input CAN_RX)
     */
    ldr r2, =0x50004088   /* GPIOC_DOUT */
    ldr r1, =0x000000CF
    str r1, [r2]
    ldr r2, =0x50004080   /* GPIOC_PMD */
    ldr r1, =0x0000F045
    str r1, [r2]

    /* Immediately isolate CAN_TX (PD7) pin: drive Push-Pull HIGH (Recessive) */
    ldr r2, =0x500040C8   /* GPIOD_DOUT */
    ldr r1, =0x000000C0   /* PD7 = 1, PD6 = 1 */
    str r1, [r2]
    ldr r2, =0x500040C0   /* GPIOD_PMD */
    ldr r1, =0x00007000   /* PD7 Push-Pull (01b), PD6 Input (00b) */
    str r1, [r2]

    ldr r2, =0x50004048   /* GPIOB_DOUT */
    ldr r1, =0x0000113F
    str r1, [r2]
    ldr r2, =0x50004040   /* GPIOB_PMD */
    ldr r1, =0x43030FF4
    str r1, [r2]

    ldr r2, =0x50004008   /* GPIOA_DOUT */
    ldr r1, =0x00004CFF
    str r1, [r2]
    ldr r2, =0x50004000   /* GPIOA_PMD */
    ldr r1, =0x75F5FF0F
    str r1, [r2]

    /* Copy .data from flash to SRAM */
    ldr r1, =_sidata
    ldr r2, =_sdata
    ldr r3, =_edata
.L_copy_data:
    cmp r2, r3
    bhs .L_zero_bss
    ldr r0, [r1]
    adds r1, #4
    str r0, [r2]
    adds r2, #4
    b .L_copy_data

.L_zero_bss:
    /* Zero .bss in SRAM */
    ldr r2, =_sbss
    ldr r3, =_ebss
    movs r0, #0
.L_clear_bss:
    cmp r2, r3
    bhs .L_call_inits
    str r0, [r2]
    adds r2, #4
    b .L_clear_bss

.L_call_inits:
    /* Call constructors / init array */
    ldr r4, =__init_array_start
    ldr r5, =__init_array_end
.L_init_loop:
    cmp r4, r5
    bhs .L_jump_main
    ldr r3, [r4]
    adds r4, #4
    blx r3
    b .L_init_loop

.L_jump_main:
    cpsie i
    bl main

.L_hang:
    b .L_hang

    .macro def_irq handler
    .globl \handler
    .weak \handler
    .set \handler, Default_Handler
    .endm

    .globl NMI_Handler
    .globl HardFault_Handler
    .globl SVC_Handler
    .globl PendSV_Handler
    .globl Default_Handler

    .type NMI_Handler, %function
    .type HardFault_Handler, %function
    .type SVC_Handler, %function
    .type PendSV_Handler, %function
    .type Default_Handler, %function

NMI_Handler:
HardFault_Handler:
SVC_Handler:
PendSV_Handler:
Default_Handler:
    /* Fault Recovery: Immediately isolate CAN bus (PC3=1 Standby, PD7=1 Recessive),
     * turn off all vehicle outputs (PA8/9/12/13=0), and trigger Cortex-M0 System Reset */
    cpsid i
    ldr r2, =0x50004088   /* GPIOC_DOUT */
    ldr r1, =0x000000CF   /* PC3 = 1 (CAN Standby) */
    str r1, [r2]
    ldr r2, =0x500040C8   /* GPIOD_DOUT */
    ldr r1, =0x000000C0   /* PD7 = 1 (CAN_TX Recessive) */
    str r1, [r2]
    ldr r2, =0x50004008   /* GPIOA_DOUT */
    ldr r1, =0x00004CFF   /* PA8/9/12/13 = 0 (ACC/ILL/BRAKE/REV OFF) */
    str r1, [r2]
    ldr r2, =0xE000ED0C   /* SCB->AIRCR */
    ldr r1, =0x05FA0004   /* VECTKEY (0x05FA) | SYSRESETREQ (bit 2) */
    str r1, [r2]
    b .

    def_irq SysTick_Handler

    def_irq BOD_IRQHandler
    def_irq WDT_IRQHandler
    def_irq EINT0_IRQHandler
    def_irq EINT1_IRQHandler
    def_irq GPAB_IRQHandler
    def_irq GPCDEF_IRQHandler
    def_irq PWM0_IRQHandler
    def_irq PWM1_IRQHandler
    def_irq BPWM0_IRQHandler
    def_irq BPWM1_IRQHandler
    def_irq BRAKE0_IRQHandler
    def_irq BRAKE1_IRQHandler
    def_irq TMR0_IRQHandler
    def_irq TMR1_IRQHandler
    def_irq TMR2_IRQHandler
    def_irq TMR3_IRQHandler
    def_irq UART02_IRQHandler
    def_irq UART1_IRQHandler
    def_irq SPI0_IRQHandler
    def_irq UART3_IRQHandler
    def_irq UART4_IRQHandler
    def_irq UART5_IRQHandler
    def_irq I2C0_IRQHandler
    def_irq I2C1_IRQHandler
    def_irq CAN0_IRQHandler
    def_irq CAN1_IRQHandler
    def_irq PWRWU_IRQHandler
    def_irq ADC_IRQHandler
    def_irq CKD_IRQHandler
    def_irq RTC_IRQHandler
