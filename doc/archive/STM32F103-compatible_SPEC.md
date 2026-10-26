# Custom Firmware Implementation Specification
**Target Hardware:** SimpleSoft / XP Toyota CAN Bus Interface Box  
**Target Silicon:** STM32F103-compatible Cortex-M3 (ARMv7-M)  
**Document Version:** 1.0  
**Date:** 2026-09-21  

---

## 1. Hardware Architecture & System Clock

### 1.1 Microcontroller Resources
- **Core:** ARM 32-bit Cortex-M3 CPU
- **Flash Memory:** 128 KB (`0x08000000 - 0x0801FFFF`)
- **SRAM:** 20 KB (`0x20000000 - 0x20004FFF`)
- **Transceiver:** NXP TJA1050 or pin-compatible High-Speed CAN Transceiver
- **Host Interface:** 3.3V / 5V-tolerant UART to Android Head Unit

### 1.2 Recommended Clock Tree
```text
  8.000 MHz Crystal (HSE) / Internal RC (HSI)
          │
          ▼
   [PLL Multiplier x9] ──► SYSCLK = 72 MHz
                              │
               ┌──────────────┴──────────────┐
               ▼                             ▼
       AHB Prescaler /1              APB1 Prescaler /2
       HCLK = 72 MHz                 PCLK1 = 36 MHz (CAN1, TIM2..4)
       APB2 Prescaler /1
       PCLK2 = 72 MHz (USART1, GPIOA..D)
```

---

## 2. GPIO Pinout & Configuration Specification

All GPIO ports must have their clocks enabled in `RCC_APB2ENR` (`IOPAEN`, `IOPBEN`, `AFIOEN`).

| Pin | Direction | Mode / Config | Register Setting | Hardware Function & Circuit Behavior |
| :--- | :--- | :--- | :--- | :--- |
| **PB2** | Output | General Purpose Output (Push-Pull, 50MHz) | `GPIOB->CRL: MODE2=11b, CNF2=00b` | **ACC 12V Trigger Output:** Active HIGH (drives NPN/MOSFET switch to head unit ACC line). |
| **PB5** | Output | General Purpose Output (Push-Pull, 50MHz) | `GPIOB->CRL: MODE5=11b, CNF5=00b` | **Reverse Camera Trigger:** Active HIGH (+12V switched trigger when reverse gear engaged). |
| **PB7** | Output | General Purpose Output (Push-Pull, 50MHz) | `GPIOB->CRL: MODE7=11b, CNF7=00b` | **Illumination (ILL) Trigger:** Active HIGH when headlights/parking lights are active. |
| **PB8** | Input | Input with Pull-Up (IPU) | `GPIOB->CRH: MODE8=00b, CNF8=10b, ODR8=1` | **CAN1_RX:** Connects to TJA1050 RXD (AFIO Remap = 10b / PB8/PB9). |
| **PB9** | Output | Alternate Function Push-Pull (50MHz) | `GPIOB->CRH: MODE9=11b, CNF9=10b` | **CAN1_TX:** Connects to TJA1050 TXD. |
| **PB10** | Output | General Purpose Output (Push-Pull, 50MHz) | `GPIOB->CRH: MODE10=11b, CNF10=00b` | **CAN_STB (Transceiver Standby):** LOW = Active / Normal; HIGH = Low-power Standby. |
| **PB12** | Output | General Purpose Output (Push-Pull, 50MHz) | `GPIOB->CRH: MODE12=11b, CNF12=00b` | **Piezo Buzzer Drive:** Toggled at 2.4 kHz to 4.0 kHz for parking radar warning beeps. |
| **PA9** | Output | Alternate Function Push-Pull (50MHz) | `GPIOA->CRH: MODE9=11b, CNF9=10b` | **USART1_TX:** Serial TX line to Android Head Unit. |
| **PA10** | Input | Floating Input / Pull-Up | `GPIOA->CRH: MODE10=00b, CNF10=01b` | **USART1_RX:** Serial RX line from Android Head Unit. |
| **PA0** | Input | Analog Input / Sense | `GPIOA->CRL: MODE0=00b, CNF0=00b` | **BATT / ACC Voltage Sense:** Hardware voltage divider monitoring 12V car battery line. |

### 2.1 GPIO Initialization C Code Example
```c
void hardware_gpio_init(void)
{
    /* Enable GPIOA, GPIOB, and AFIO peripheral clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    /* Remap CAN1 to PB8/PB9 */
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2; /* 10: PB8, PB9 */

    /* Configure Outputs: PB2 (ACC), PB5 (REV), PB7 (ILL), PB10 (CAN_STB), PB12 (BUZZER) */
    /* Push-Pull Output, max speed 50 MHz -> CNF=00b, MODE=11b (0x3) */
    GPIOB->CRL &= ~(0x000F0F00u | (0x0F << (7 * 4)));
    GPIOB->CRL |=  (0x00030300u | (0x03 << (7 * 4))); /* PB2, PB5, PB7 */

    GPIOB->CRH &= ~(0x000F0F00u);
    GPIOB->CRH |=  (0x00030300u); /* PB10, PB12 */

    /* Set default output states */
    GPIOB->BRR = (1u << 2) | (1u << 5) | (1u << 7) | (1u << 12); /* All outputs LOW */
    GPIOB->BRR = (1u << 10); /* CAN_STB LOW = Transceiver Active */

    /* Configure CAN Pins: PB8 (Input Pull-up), PB9 (AF Push-Pull) */
    GPIOB->CRH &= ~(0x000000FFu);
    GPIOB->CRH |=  (0x000000B8u); /* PB9: CNF=10b, MODE=11b; PB8: CNF=10b, MODE=00b */
    GPIOB->BSRR = (1u << 8);      /* Pull-up on PB8 */

    /* Configure USART1 Pins: PA9 (TX, AF PP), PA10 (RX, Input Floating/Pull-up) */
    GPIOA->CRH &= ~(0x00000FF0u);
    GPIOA->CRH |=  (0x000004B0u); /* PA9: AF PP 50MHz (0xB), PA10: Input Floating (0x4) */
}
```

---

## 3. CAN Subsystem Configuration (125 kbps)

### 3.1 Bit-Timing Mathematics for 125 kbps
With **$f_{PCLK1} = 36\text{ MHz}$** (APB1 clock):
$$\text{Bit Rate} = \frac{f_{PCLK1}}{\text{Prescaler} \times (1 + \text{BS1} + \text{BS2})}$$

For standard automotive CAN at **125 kbps** with **16 Time Quanta (Tq)** per bit and **87.5% sample point**:
- **$\text{Sync\_Seg} = 1\text{ Tq}$** (fixed by CAN standard)
- **$\text{BS1} = 13\text{ Tq}$** (Time Segment 1, propag + phase 1) &rarr; Register value = $13 - 1 = 12$ (`0x0C`)
- **$\text{BS2} = 2\text{ Tq}$** (Time Segment 2, phase 2) &rarr; Register value = $2 - 1 = 1$ (`0x01`)
- **$\text{SJW} = 1\text{ Tq}$** (Resynchronization Jump Width) &rarr; Register value = $1 - 1 = 0$ (`0x00`)
- **$\text{Nominal Bit Time} = 1 + 13 + 2 = 16\text{ Tq}$**
- **$\text{Sample Point} = \frac{1 + 13}{16} = 87.5\%$**
- **$\text{Prescaler (BRP)} = \frac{36\,000\,000\text{ Hz}}{125\,000\text{ bps} \times 16} = 18$** &rarr; Register value = $18 - 1 = 17$ (`0x0011`)

$$\text{CAN\_BTR Value} = (0 \ll 24) \mid (1 \ll 20) \mid (12 \ll 16) \mid 17 = \mathbf{0x001C0011}$$

*(Note: If running at 500 kbps, $\text{Prescaler} = \frac{36\,\text{MHz}}{500\,\text{kbps} \times 16} = 4.5 \implies$ use 18 Tq with Prescaler 4: $\text{BS1}=14, \text{BS2}=3 \implies \mathbf{0x002D0003}$)*.

### 3.2 CAN Driver Initialization Code
```c
uint32_t can_init_125k(void)
{
    /* Enable CAN1 clock in APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    /* Exit Sleep and Request Initialization Mode */
    CAN1->MCR = CAN_MCR_INRQ;
    uint32_t timeout = 2000u;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0) {
        if (--timeout == 0) return 0; /* Hardware error */
    }

    /* Configure Operating Parameters:
     * - Automatic Bus-Off Management (ABOM = 1)
     * - Automatic Wake-Up from sleep on CAN bus activity (AWUM = 1)
     * - Transmit FIFO Priority (TXFP = 0)
     */
    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM;

    /* Set 125 kbps Bit Timing: BRP=18, BS1=13, BS2=2, SJW=1 */
    CAN1->BTR = (0u << 24)        /* SJW = 1 Tq */
              | (1u << 20)        /* BS2 = 2 Tq */
              | (12u << 16)       /* BS1 = 13 Tq */
              | (17u << 0);       /* BRP = 18 (prescaler) */

    /* Leave Initialization Mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    timeout = 2000u;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0) {
        if (--timeout == 0) return 0;
    }

    /* Configure Filter 0: Accept All Frames into FIFO 0 */
    CAN1->FMR |= CAN_FMR_FINIT;   /* Enter filter init */
    CAN1->FA1R &= ~CAN_FA1R_FACT0;/* Deactivate filter 0 */
    CAN1->FS1R |= CAN_FS1R_FSC0;  /* 32-bit single scale */
    CAN1->FM1R &= ~CAN_FM1R_FBM0; /* Mask mode */
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;/* Assign to FIFO 0 */
    
    CAN1->sFilterRegister[0].FR1 = 0x00000000u; /* ID = 0 */
    CAN1->sFilterRegister[0].FR2 = 0x00000000u; /* Mask = 0 (Match All) */
    
    CAN1->FA1R |= CAN_FA1R_FACT0; /* Activate filter 0 */
    CAN1->FMR &= ~CAN_FMR_FINIT;  /* Exit filter init */

    /* Enable FIFO 0 Message Pending Interrupt */
    CAN1->IER |= CAN_IER_FMPIE0;
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    return 1;
}
```

---

## 4. Host UART Protocol Configuration

### 4.1 Baud Rate Calculation (USART1 on APB2 @ 72 MHz)
$$\text{USARTDIV} = \frac{f_{PCLK2}}{16 \times \text{BaudRate}}$$

| Target Baud Rate | Application Mode | $\text{USARTDIV}$ | Mantissa (`DIV_Mantissa`) | Fraction (`DIV_Fraction`) | `USART1->BRR` Register |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **38,400 bps** *(Default SimpleSoft)* | Android Head Unit link | $117.1875$ | $117$ (`0x75`) | $0.1875 \times 16 = 3$ (`0x3`) | **`0x0753`** |
| **115,200 bps** *(High Speed)* | Fast debugging / logs | $39.0625$ | $39$ (`0x27`) | $0.0625 \times 16 = 1$ (`0x1`) | **`0x0271`** |

### 4.2 UART Driver Initialization Code
```c
void uart_hu_init(uint32_t baudrate)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Calculate BRR divider for 72 MHz APB2 clock */
    uint32_t integer_div = (25u * 72000000u) / (4u * baudrate);
    uint32_t mantissa = integer_div / 100u;
    uint32_t fraction = (((integer_div - (mantissa * 100u)) * 16u) + 50u) / 100u;
    USART1->BRR = (mantissa << 4) | (fraction & 0x0Fu);

    /* Enable Transmitter, Receiver, and RX Interrupt */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
}
```

### 4.3 SimpleSoft Head Unit Packet Protocol
Every frame sent to or received from the Android head unit follows the packet structure:
```text
[ 0x2E ] [ CMD ] [ LENGTH ] [ PAYLOAD 0 .. N-1 ] [ CHECKSUM ]
```
- **Sync Byte:** Always `0x2E` (`'.'`)
- **Checksum Calculation:**
  $$\text{Checksum} = \left(\sum_{i=1}^{\text{Length}} \text{Payload}[i] + \text{CMD} + \text{LENGTH}\right) \text{ XOR } 0\text{xFF}$$

#### Packet Types:
- **`0x01` Steering Wheel Key:** `[0x2E, 0x01, 0x02, KeyCode, KeyState, CS]`
- **`0x02` Reverse Status:** `[0x2E, 0x02, 0x02, IsReverse (0/1), 0x00, CS]`
- **`0x03` Doors & Power Status:** `[0x2E, 0x03, 0x04, DoorFlags, PowerFlags, 0x00, 0x00, CS]`
- **`0x04` Vehicle Speed:** `[0x2E, 0x04, 0x02, Speed_kmh, 0x00, CS]`

---

## 5. Low-Power Sleep & Wakeup Architecture

### 5.1 Power-Down Sequence
1. **Detect Ignition / ACC OFF:** When CAN frame `0x280` indicates vehicle power is off or voltage on `PA0` drops below 11.5V.
2. **Debounce / Hold Timer:** Keep the module awake for 15 seconds to allow head-unit shutdown.
3. **De-assert Output Pins:** Set `PB2 = 0` (ACC), `PB5 = 0` (REV), `PB7 = 0` (ILL), `PB12 = 0` (Buzzer).
4. **Place CAN Transceiver into Standby:** Set `PB10 = 1` (`CAN_STB = HIGH`).
5. **Put bxCAN Controller into Sleep:**
   ```c
   CAN1->MCR |= CAN_MCR_SLEEP;
   while ((CAN1->MSR & CAN_MSR_SLAK) == 0); /* Wait for sleep acknowledge */
   ```
6. **Enter Cortex-M3 Stop / Low-Power Mode:**
   ```c
   void enter_system_sleep(void)
   {
       /* Configure EXTI on PB8 (CAN_RX) to wake on falling edge */
       AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
       EXTI->IMR |= EXTI_IMR_MR8;
       EXTI->FTSR |= EXTI_FTSR_TR8; /* Trigger on falling edge */
       NVIC_EnableIRQ(EXTI9_5_IRQn);

       /* Set Sleep-Deep bit in System Control Register */
       SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
       PWR->CR |= PWR_CR_LPDS; /* Low-power regulator in STOP mode */

       /* Execute Wait For Interrupt */
       __WFI();

       /* === WAKE UP OCCURS HERE === */
       system_clock_reinit_72mhz();
       GPIOB->BRR = (1u << 10); /* CAN_STB LOW = Transceiver Active */
       CAN1->MCR &= ~CAN_MCR_SLEEP; /* Wake bxCAN */
       GPIOB->BSRR = (1u << 2); /* Turn ACC output ON */
   }
   ```

---

## 6. Flash Memory Layout & Vector Table

```text
0x08000000 ┌────────────────────────────────────────────────────────┐
           │ Vector Table (SCB->VTOR = 0x08000000)                  │
0x08000100 ├────────────────────────────────────────────────────────┤
           │ Application Firmware Code (.text)                      │
           │ Static Tables & Strings (.rodata)                      │
0x0801E000 ├────────────────────────────────────────────────────────┤
           │ User Calibration & CAN Baud Configuration (Pages 60-62)│
0x0801FFF0 ├────────────────────────────────────────────────────────┤
           │ 16-Byte Boot / Hardware Metadata Block                 │
0x08020000 └────────────────────────────────────────────────────────┘
```

---

## 7. Build and Verification Guidelines

1. **Compiler Flags:**
   ```bash
   arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -ffunction-sections -fdata-sections -Wall -Wextra -DSTM32F10X_MD -T stm32f103_reconstructed.ld
   ```
2. **Test Strategy:**
   - Verify 125 kbps CAN reception using a CAN-USB analyzer (e.g. CANable / PCAN / ValueCAN).
   - Verify UART frame integrity at 38,400 baud using logic analyzer / FTDI converter.
   - Measure sleep quiescent current (< 1.5 mA total board consumption in Stop mode).

