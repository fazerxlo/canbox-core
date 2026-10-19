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

    /* External Interrupts (32 IRQs forwarding to application at 0x00002000) */
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

    /* 0. Disconnect PD6 and PD7 from CAN peripheral in SYS_GPD_MFP */
    ldr r2, =0x50000100   /* SYS_REGWRPROT */
    ldr r1, =0x59
    str r1, [r2]
    ldr r1, =0x16
    str r1, [r2]
    ldr r1, =0x88
    str r1, [r2]
    ldr r2, =0x5000003C   /* SYS_GPD_MFP */
    ldr r1, [r2]
    ldr r3, =0x000000C0
    bics r1, r3
    str r1, [r2]
    ldr r2, =0x50000100
    movs r1, #0
    str r1, [r2]

    /* 1. Silence buzzer, isolate CAN transceiver in Standby (PC3=1), apply factory GPIO outputs:
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

    /* 2. Immediately isolate CAN_TX (PD7) pin: drive Push-Pull HIGH (Recessive) */
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
    ldr r1, =0x00004DFF   /* PA8 (ACC) = 1 (ON to prevent Head Unit sleep), PA15=0, PA9/12/13=0 */
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
    bhs .L_jump_main
    str r0, [r2]
    adds r2, #4
    b .L_clear_bss

.L_jump_main:
    bl main

.L_hang:
    b Default_Handler

    /* Forwarding trampolines to application vector table (0x00002000) */
    .macro forward_vec name offset
    .globl \name
    .type \name, %function
\name:
    ldr r0, =(\offset)
    ldr r0, [r0]
    adds r1, r0, #1
    bne 1f
    b   Default_Handler
1:  bx  r0
    .endm

    forward_vec NMI_Handler        0x00002008
    forward_vec HardFault_Handler  0x0000200C
    forward_vec SVC_Handler        0x0000202C
    forward_vec PendSV_Handler     0x00002038
    forward_vec SysTick_Handler    0x0000203C

    forward_vec BOD_IRQHandler     0x00002040
    forward_vec WDT_IRQHandler     0x00002044
    forward_vec EINT0_IRQHandler   0x00002048
    forward_vec EINT1_IRQHandler   0x0000204C
    forward_vec GPAB_IRQHandler    0x00002050
    forward_vec GPCDEF_IRQHandler  0x00002054
    forward_vec TMR0_IRQHandler    0x00002060
    forward_vec TMR1_IRQHandler    0x00002064
    forward_vec TMR2_IRQHandler    0x00002068
    forward_vec TMR3_IRQHandler    0x0000206C
    forward_vec UART02_IRQHandler  0x00002070
    forward_vec UART1_IRQHandler   0x00002074
    forward_vec SPI0_IRQHandler    0x00002078
    forward_vec UART3_IRQHandler   0x0000207C
    forward_vec UART4_IRQHandler   0x00002080
    forward_vec UART5_IRQHandler   0x00002084
    forward_vec I2C0_IRQHandler    0x00002088
    forward_vec I2C1_IRQHandler    0x0000208C
    forward_vec CAN0_IRQHandler    0x00002090
    forward_vec CAN1_IRQHandler    0x00002094
    forward_vec PWM0_IRQHandler    0x00002098
    forward_vec PWM1_IRQHandler    0x0000209C
    forward_vec BPWM0_IRQHandler   0x000020A0
    forward_vec BPWM1_IRQHandler   0x000020A4
    forward_vec BRAKE0_IRQHandler  0x000020A8
    forward_vec BRAKE1_IRQHandler  0x000020AC
    forward_vec PWRWU_IRQHandler   0x000020B0
    forward_vec ADC_IRQHandler     0x000020B4
    forward_vec CKD_IRQHandler     0x000020B8
    forward_vec RTC_IRQHandler     0x000020BC

    .globl Default_Handler
    .type Default_Handler, %function
Default_Handler:
    cpsid i
    ldr r2, =0x50004088   /* GPIOC_DOUT: PC3=1 CAN Standby */
    ldr r1, =0x000000CF
    str r1, [r2]
    ldr r2, =0x50004008   /* GPIOA_DOUT: PA8/9/12/13=0 OFF */
    ldr r1, =0x00004CFF
    str r1, [r2]
    ldr r2, =0xE000ED0C   /* SCB->AIRCR: System Reset */
    ldr r1, =0x05FA0004
    str r1, [r2]
    b .
