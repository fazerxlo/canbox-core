#include "hal/hal_uart.h"
#include "core/ring_buffer.h"
#include "NUC131.h"

#define NUC_UART_RX_RING_SIZE 128

static uint8_t s_uart_storage[NUC_UART_RX_RING_SIZE];
static ring_buffer_t s_uart_rx_rb;

hal_status_t hal_uart_init(uart_baudrate_t baudrate) {
    ring_buffer_init(&s_uart_rx_rb, s_uart_storage, sizeof(uint8_t), NUC_UART_RX_RING_SIZE);

    SYS_UnlockReg();

    // Enable UART0 peripheral clock
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_PLL;
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_UART_N_Msk);

    // Multi-function pins: PB.0 = RXD0, PB.1 = TXD0
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    SYS_LockReg();

    // Line control: 8 data bits, 1 stop bit, no parity
    UART0->LCR = UART_LCR_WLS_8BITS;

    // Mode 2 Baud Rate Divisor: Baud = F_CLK / (BAUD_DIV + 2)
    // BAUD_DIV = (F_CLK / Baud) - 2
    uint32_t baud_div = (50000000 / baudrate) - 2;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(50000000, baudrate);

    // Reset and enable RX FIFO with 1-byte trigger threshold
    UART0->FCR = UART_FCR_RFR_Msk | UART_FCR_TFR_Msk | UART_FCR_RFITL_1BYTE;

    // Enable RX Interrupt
    UART0->IER |= UART_IER_RDA_IEN_Msk;
    NVIC_EnableIRQ(UART02_IRQn);

    return HAL_OK;
}

hal_status_t hal_uart_read_byte(uint8_t *byte) {
    if (!byte) return HAL_ERROR;
    return ring_buffer_pop(&s_uart_rx_rb, byte) ? HAL_OK : HAL_TIMEOUT;
}

size_t hal_uart_read(uint8_t *buffer, size_t max_len) {
    if (!buffer || max_len == 0) return 0;
    size_t count = 0;
    while (count < max_len && ring_buffer_pop(&s_uart_rx_rb, &buffer[count])) {
        count++;
    }
    return count;
}

hal_status_t hal_uart_write(const uint8_t *data, size_t len) {
    if (!data || len == 0) return HAL_ERROR;

    for (size_t i = 0; i < len; i++) {
        while (UART0->FSR & UART_FSR_TX_FULL_Msk); // Wait until FIFO is not full
        UART0->DATA = data[i];
    }
    return HAL_OK;
}

hal_status_t hal_uart_flush_tx(void) {
    while (!(UART0->FSR & UART_FSR_TE_FLAG_Msk)); // Wait for transmitter empty
    return HAL_OK;
}

void UART02_IRQHandler(void) {
    if (UART0->ISR & UART_ISR_RDA_INT_Msk) {
        while (!(UART0->FSR & UART_FSR_RX_EMPTY_Msk)) {
            uint8_t byte = (uint8_t)(UART0->DATA);
            ring_buffer_push(&s_uart_rx_rb, &byte);
        }
    }
}
