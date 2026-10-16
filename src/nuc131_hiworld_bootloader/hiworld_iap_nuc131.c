#include "bootloader/hiworld_iap_nuc131.h"
#include <string.h>

/* NUC131 MMIO Register Bases */
#define SYS_BASE                (0x50000000UL)
#define CLK_BASE                (0x50000200UL)
#define FMC_BASE                (0x5000C000UL)
#define GPIOA_BASE              (0x50004000UL)
#define GPIOB_BASE              (0x50004040UL)
#define GPIOC_BASE              (0x50004080UL)
#define GPIOD_BASE              (0x500040C0UL)
#define UART0_BASE              (0x40050000UL)

/* System & Protection */
#define SYS_REGWRPROT           (*(volatile uint32_t *)(SYS_BASE + 0x100))
#define SYS_IPRSTC2             (*(volatile uint32_t *)(SYS_BASE + 0x00C))
#define SYS_GPB_MFP             (*(volatile uint32_t *)(SYS_BASE + 0x034))
#define SYS_GPD_MFP             (*(volatile uint32_t *)(SYS_BASE + 0x03C))

/* Clock Control */
#define CLK_PWRCON              (*(volatile uint32_t *)(CLK_BASE + 0x000))
#define CLK_APBCLK              (*(volatile uint32_t *)(CLK_BASE + 0x008))
#define CLK_CLKSTATUS           (*(volatile uint32_t *)(CLK_BASE + 0x00C))
#define CLK_CLKSEL1             (*(volatile uint32_t *)(CLK_BASE + 0x014))
#define CLK_CLKDIV              (*(volatile uint32_t *)(CLK_BASE + 0x018))

/* Flash Memory Controller */
#define FMC_ISPCON              (*(volatile uint32_t *)(FMC_BASE + 0x00))
#define FMC_ISPADR              (*(volatile uint32_t *)(FMC_BASE + 0x04))
#define FMC_ISPDAT              (*(volatile uint32_t *)(FMC_BASE + 0x08))
#define FMC_ISPCMD              (*(volatile uint32_t *)(FMC_BASE + 0x0C))
#define FMC_ISPTRG              (*(volatile uint32_t *)(FMC_BASE + 0x10))

/* GPIO Registers */
#define GPIOA_PMD               (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_DOUT              (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOB_PMD               (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_DOUT              (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOC_PMD               (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_DOUT              (*(volatile uint32_t *)(GPIOC_BASE + 0x08))
#define GPIOD_PMD               (*(volatile uint32_t *)(GPIOD_BASE + 0x00))
#define GPIOD_DOUT              (*(volatile uint32_t *)(GPIOD_BASE + 0x08))

/* UART0 Registers (NUC131 TRM: BAUD is at offset 0x24) */
#define UART0_RBR               (*(volatile uint32_t *)(UART0_BASE + 0x00))
#define UART0_THR               (*(volatile uint32_t *)(UART0_BASE + 0x00))
#define UART0_LCR               (*(volatile uint32_t *)(UART0_BASE + 0x0C))
#define UART0_BAUD              (*(volatile uint32_t *)(UART0_BASE + 0x24))
#define UART0_FSR               (*(volatile uint32_t *)(UART0_BASE + 0x18))

/* Protocol Constants */
static const uint8_t HIWORLD_MAGIC[12] = {
    0x01, 0x09, 0x07, 0x08, 0x00, 0x02, 0x01, 0x07, 0x91, 0x32, 0x47, 0xA4
};
static const uint8_t EXPECTED_PASSWORD[4] = {0x19, 0x78, 0x02, 0x17};

/* Forward Declarations */
static void fmc_unlock(void);
static void fmc_lock(void);
static bool fmc_erase_page(uint32_t addr);
static bool fmc_program_word(uint32_t addr, uint32_t data);
static void uart_send_byte(uint8_t b);
static bool uart_recv_byte(uint8_t *b, uint32_t timeout_loops);
static bool uart_recv_exact(uint8_t *buf, uint32_t len, uint32_t timeout_ms);
static void jump_to_app(void);

/* --- Hardware Setup --- */
void iap_init_hardware(void) {
    SYS_REGWRPROT = 0x59;
    SYS_REGWRPROT = 0x16;
    SYS_REGWRPROT = 0x88;

    /* 1. Ensure PD6 and PD7 are standard GPIO, NOT CAN peripheral */
    SYS_GPD_MFP &= ~((1U << 6) | (1U << 7));

    /* 2. Configure UART0 clock:
     * - Ensure internal 22.1184 MHz high-speed RC (HIRC) is enabled and stable
     * - Select HIRC as UART0 clock source in CLKSEL1 (bits [25:24] = 11b)
     * - Set UART0 clock divider to 1 (bits [11:8] = 0000b)
     * - Enable UART0 peripheral clock in APBCLK (bit 16)
     */
    CLK_PWRCON |= (1U << 2);
    while (!(CLK_CLKSTATUS & (1U << 4)));

    CLK_CLKSEL1 = (CLK_CLKSEL1 & ~(3U << 24)) | (3U << 24);
    CLK_CLKDIV  &= ~(0xFU << 8);
    CLK_APBCLK  |= (1U << 16);

    /* 3. Configure PB0 as UART0_RXD and PB1 as UART0_TXD in GPB_MFP */
    SYS_GPB_MFP &= ~((1U << 0) | (1U << 1));
    SYS_GPB_MFP |=  ((1U << 0) | (1U << 1));

    SYS_REGWRPROT = 0x00;

    /* 4. Configure pin directions, isolate CAN in Standby (PC3=1), drive CAN_TX (PD7) HIGH (Recessive), and silence buzzer:
     * PC_DOUT = 0x00cf, PC_PMD = 0x0000f045 (PC3=1 Standby)
     * PB_DOUT = 0x113f, PB_PMD = 0x43030ff4 (PB1=1 TX, PB3=1, PB12=1 silence buzzer)
     * PD_DOUT = 0x00c0, PD_PMD = 0x00007000 (PD7=1 CAN_TX Recessive Push-Pull, PD6=Input)
     * PA_DOUT = 0x00004cff, PA_PMD = 0x75f5ff0f (All vehicle outputs off: PA8, PA9, PA12, PA13 = 0)
     */
    GPIOC_DOUT = 0x000000CF;
    GPIOC_PMD  = 0x0000F045;

    GPIOD_DOUT = 0x000000C0;
    GPIOD_PMD  = 0x00007000;

    GPIOB_DOUT = 0x0000113F;
    GPIOB_PMD  = 0x43030FF4;

    GPIOA_DOUT = 0x00004DFF; /* PA8 (ACC) = 1 (ON to prevent Head Unit sleep), PA9/12/13 = 0 (OFF) */
    GPIOA_PMD  = 0x75F5FF0F;

    /* 5. Configure UART0 for 38,400 bps 8-N-1 on default reset clock (22.1184 MHz HIRC) */
    /* Mode 0 (16x oversampling): BRD = (22,118,400 / (16 * 38,400)) - 2 = 34 (0.00% error) */
    UART0_BAUD = 34;
    UART0_LCR  = 0x03;

    /* 6. Clear UART0 RX and TX FIFOs */
    UART0_FSR |= (1U << 1) | (1U << 2);
}

/* --- FMC Flash Drivers --- */
static void fmc_unlock(void) {
    SYS_REGWRPROT = 0x59;
    SYS_REGWRPROT = 0x16;
    SYS_REGWRPROT = 0x88;
    FMC_ISPCON |= (1U << 0) /* ISPEN */ | (1U << 3) /* APUEN */;
}

static void fmc_lock(void) {
    FMC_ISPCON &= ~(1U << 0);
    SYS_REGWRPROT = 0x00;
}

static bool fmc_erase_page(uint32_t addr) {
    FMC_ISPADR = addr;
    FMC_ISPCMD = 0x22; /* Page Erase */
    FMC_ISPTRG = 0x01;
    __asm volatile("isb");
    while (FMC_ISPTRG & 0x01);
    return true;
}

static bool fmc_program_word(uint32_t addr, uint32_t data) {
    FMC_ISPADR = addr;
    FMC_ISPDAT = data;
    FMC_ISPCMD = 0x21; /* Word Program */
    FMC_ISPTRG = 0x01;
    __asm volatile("isb");
    while (FMC_ISPTRG & 0x01);
    return (*(volatile uint32_t *)addr == data);
}

/* --- UART0 Communication --- */
static void uart_send_byte(uint8_t b) {
    while (UART0_FSR & (1U << 23)); /* Wait if TX FIFO full */
    UART0_THR = b;
}

static bool uart_recv_byte(uint8_t *b, uint32_t timeout_loops) {
    while (timeout_loops--) {
        if (!(UART0_FSR & (1U << 14))) { /* RX FIFO not empty */
            *b = (uint8_t)(UART0_RBR & 0xFF);
            return true;
        }
        for (volatile int d = 0; d < 12; d++); /* ~1 us delay */
    }
    return false;
}

static bool uart_recv_exact(uint8_t *buf, uint32_t len, uint32_t timeout_ms) {
    for (uint32_t i = 0; i < len; i++) {
        uint32_t loops = (i == 0) ? (timeout_ms * 1000) : (100 * 1000);
        if (!uart_recv_byte(&buf[i], loops)) {
            return false;
        }
    }
    return true;
}

/* --- Checksum & De-Obfuscation --- */
static uint32_t calc_record_csum(const uint8_t *rec) {
    uint32_t sum = 0;
    for (int j = 0; j < 33; j++) {
        uint32_t w = (uint32_t)rec[j * 4 + 0] |
                     ((uint32_t)rec[j * 4 + 1] << 8) |
                     ((uint32_t)rec[j * 4 + 2] << 16) |
                     ((uint32_t)rec[j * 4 + 3] << 24);
        sum += w;
    }
    return sum;
}

static void deobfuscate(uint8_t *chunk, const uint8_t *key) {
    for (uint16_t i = 0; i < IAP_DATA_SIZE; i++) {
        chunk[i] = (uint8_t)((chunk[i] - i - key[i & 3]) & 0xFF);
    }
}

/* --- Main Bootloader Engine --- */
void iap_run(void) {
    /* Ensure hardware, UART0 clock, pin muxing, and CAN standby are initialized */
    iap_init_hardware();

    volatile uint32_t *p_token = (volatile uint32_t *)BOOT_TOKEN_ADDR;
    bool force_iap = (*p_token == BOOT_TOKEN_MAGIC);

    /* Integrity check application at 0x00002000 */
    uint32_t app_sp    = *(volatile uint32_t *)(APP_BASE_ADDR + 0);
    uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4);
    bool app_valid = (app_sp >= 0x20000400 && app_sp <= 0x20002000 &&
                      (app_reset & 1) && app_reset >= APP_BASE_ADDR && app_reset < APP_MAX_ADDR);

    if (!force_iap && app_valid) {
        /* Cold-boot rescue window: Listen briefly (~50 ms) for 0xE0 upgrade trigger from host */
        uint8_t peek = 0;
        if (uart_recv_byte(&peek, 20000) && peek == 0x5A) {
            uint8_t hdr[3] = {0};
            if (uart_recv_byte(&hdr[0], 20000) && hdr[0] == 0xA5 &&
                uart_recv_byte(&hdr[1], 20000) &&
                uart_recv_byte(&hdr[2], 20000) && hdr[2] == 0xE0) {
                force_iap = true;
            } else {
                jump_to_app();
            }
        } else {
            jump_to_app();
        }
    }

    /* Clear token once consumed */
    *p_token = 0;

    /* Persistent connection loop */
    while (1) {
        /* Phase 2: Send 'U' prompt with 100 ms retry cadence */
        uint8_t rx_pw[4];
        bool pw_matched = false;

        while (!pw_matched) {
            uart_send_byte(0x55); /* 'U' */

            /* Wait up to 100 ms for first byte of password */
            uint8_t b = 0;
            if (uart_recv_byte(&b, 37000)) {
                if (b == EXPECTED_PASSWORD[0]) {
                    rx_pw[0] = b;
                    if (uart_recv_byte(&rx_pw[1], 80000) &&
                        uart_recv_byte(&rx_pw[2], 80000) &&
                        uart_recv_byte(&rx_pw[3], 80000)) {
                        if (memcmp(rx_pw, EXPECTED_PASSWORD, 4) == 0) {
                            pw_matched = true;
                            break;
                        }
                    }
                } else if (b == 0x5A) {
                    /* Drain any retransmitted 0xE0 trigger from HU */
                    uint8_t dummy;
                    for (int k = 0; k < 6; k++) {
                        uart_recv_byte(&dummy, 5000);
                    }
                }
            }
        }

        /* Password matched: Unlock Flash and immediately acknowledge with 'B', 'S' */
        fmc_unlock();
        uart_send_byte(0x42); /* 'B' */
        uart_send_byte(0x53); /* 'S' */

        uint8_t record[IAP_REC_SIZE];
        uint8_t key[4] = {0};

        /* Phase 3: Receive 136-byte records */
        while (1) {
            if (!uart_recv_exact(record, IAP_REC_SIZE, 5000)) {
                uart_send_byte(0x54); /* 'T' = Timeout receiving 136 bytes */
                break;
            }

            /* Record 0: Header with Obfuscation Key */
            if (memcmp(record, HIWORLD_MAGIC, 12) == 0) {
                memcpy(key, &record[132], 4);
                uart_send_byte(0x53); /* 'S' = ACK */
                continue;
            }

            /* Data Record: Validate 32-bit Additive Checksum */
            uint32_t expected_csum = calc_record_csum(record);
            uint32_t received_csum = (uint32_t)record[132] |
                                     ((uint32_t)record[133] << 8) |
                                     ((uint32_t)record[134] << 16) |
                                     ((uint32_t)record[135] << 24);

            if (expected_csum != received_csum) {
                uart_send_byte(0x43); /* 'C' = Checksum mismatch */
                break;
            }

            /* De-obfuscate payload chunk */
            deobfuscate(record, key);

            /* Derive target address strictly from 16-bit sequence number (Bytes 128..129) */
            uint16_t seq = (uint16_t)record[128] | ((uint16_t)record[129] << 8);
            if (seq == 0) {
                uart_send_byte(0x5A); /* 'Z' = Zero sequence */
                break;
            }
            uint32_t rec_target_addr = APP_BASE_ADDR + ((uint32_t)(seq - 1) * IAP_DATA_SIZE);

            if (rec_target_addr < APP_BASE_ADDR || (rec_target_addr + IAP_DATA_SIZE) > APP_MAX_ADDR) {
                uart_send_byte(0x46); /* 'F' = Flash boundary overflow */
                break;
            }

            /* Erase new 512-byte flash page on demand at page boundary */
            if ((rec_target_addr % FMC_PAGE_SIZE) == 0) {
                fmc_erase_page(rec_target_addr);
            }

            /* Program 128 bytes (32 words) with write verification */
            bool write_ok = true;
            for (int w = 0; w < 32; w++) {
                uint32_t word = (uint32_t)record[w * 4 + 0] |
                                ((uint32_t)record[w * 4 + 1] << 8) |
                                ((uint32_t)record[w * 4 + 2] << 16) |
                                ((uint32_t)record[w * 4 + 3] << 24);
                if (!fmc_program_word(rec_target_addr + (w * 4), word)) {
                    write_ok = false;
                    break;
                }
            }

            if (!write_ok) {
                uart_send_byte(0x50); /* 'P' = Flash write verify failure */
                break;
            }

            /* Check EOF flag (Byte 131) */
            if (record[131] == 0x01) {
                fmc_lock();
                uart_send_byte(0x45); /* 'E' = Success */

                /* Wait for UART0 TX FIFO to empty completely */
                while (!(UART0_FSR & (1U << 28)));

                jump_to_app();
                return;
            }

            /* ACK record */
            uart_send_byte(0x53); /* 'S' */
        }

        fmc_lock();
        if (app_valid) {
            jump_to_app();
        }
    }
}

/* --- Hardware Vector Remap & Application Jump --- */
static void jump_to_app(void) {
    uint32_t app_sp    = *(volatile uint32_t *)(APP_BASE_ADDR + 0);
    uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4);

    if (app_sp < 0x20000400 || app_sp > 0x20002000 || !(app_reset & 1)) {
        return;
    }

    /* Wait for UART0 TX FIFO to fully empty */
    while (!(UART0_FSR & (1U << 28)));

    /* Disable interrupts and clear pending flags on Cortex-M0 (32 IRQs max) */
    *(volatile uint32_t *)0xE000E180 = 0xFFFFFFFF; /* NVIC->ICER[0] */
    *(volatile uint32_t *)0xE000E280 = 0xFFFFFFFF; /* NVIC->ICPR[0] */

    /* Keep CAN transceiver standby (PC3 = 1) held until application initializes CAN */

    /* FMC Hardware Vector Remap: ISPCMD = 0x2E */
    SYS_REGWRPROT = 0x59;
    SYS_REGWRPROT = 0x16;
    SYS_REGWRPROT = 0x88;
    FMC_ISPCON |= (1U << 0) | (1U << 24); /* ISPEN | VECMAP Enable */
    FMC_ISPCMD  = 0x2E;                   /* Vector Remap Command (0x2E) */
    FMC_ISPADR  = APP_BASE_ADDR;          /* 0x00002000 */
    FMC_ISPTRG  = 0x01;
    __asm volatile("isb");
    while (FMC_ISPTRG & 0x01);            /* Wait for hardware completion */
    FMC_ISPCON &= ~(1U << 0);
    SYS_REGWRPROT = 0x00;

    /* Set Stack Pointer & Branch to Application */
    __asm volatile(
        "msr msp, %0 \n"
        "bx  %1      \n"
        :
        : "r" (app_sp), "r" (app_reset)
        : "memory"
    );

    while (1);
}

int main(void) {
    iap_run();
    return 0;
}

