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
    CLK_EnableModuleClock(UART0_MODULE);

    // Select UART module clock source as PLL and set divider
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    // Multi-function pins: PB.0 = RXD0, PB.1 = TXD0
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    SYS_LockReg();

    // UART_Open() configures the UART with standard settings:
    // - Data bits: 8
    // - Stop bits: 1
    // - Parity: none
    // - Baudrate: as specified
    UART_Open(UART0, baudrate);

    // Reset and enable RX FIFO with 1-byte trigger threshold
    UART0->FCR = UART_FCR_RFR_Msk | UART_FCR_TFR_Msk | UART_FCR_RFITL_1BYTE;

    // Enable RX Data Available + Buffer Error / Line Status Interrupts
    UART_EnableInt(UART0, UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_BUF_ERR_IEN_Msk);
    NVIC_SetPriority(UART02_IRQn, 1);
    NVIC_EnableIRQ(UART02_IRQn);

    return HAL_STATUS_OK;
}

hal_status_t hal_uart_read_byte(uint8_t *byte) {
    if (!byte) return HAL_STATUS_ERROR;
    return ring_buffer_pop(&s_uart_rx_rb, byte) ? HAL_STATUS_OK : HAL_STATUS_TIMEOUT;
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
    if (!data || len == 0) return HAL_STATUS_ERROR;

    for (size_t i = 0; i < len; i++) {
        uint32_t timeout = 100000UL;
        while (UART_IS_TX_FULL(UART0) && --timeout); // Wait until TX FIFO is not full
        UART_WRITE(UART0, data[i]);
    }
    return HAL_STATUS_OK;
}

hal_status_t hal_uart_flush_tx(void) {
    uint32_t timeout = 100000UL;
    while (!UART_IS_TX_EMPTY(UART0) && --timeout); // Wait for transmitter empty
    return HAL_STATUS_OK;
}

void UART02_IRQHandler(void) {
    uint32_t isr = UART0->ISR;

    if (isr & (UART_ISR_RDA_INT_Msk | UART_ISR_TOUT_IF_Msk)) {
        while (!UART_GET_RX_EMPTY(UART0)) {
            uint8_t byte = (uint8_t)UART_READ(UART0);
            ring_buffer_push(&s_uart_rx_rb, &byte);
        }
    }

    /* Clear Break, Framing, Parity, and FIFO Overrun errors caused by Head Unit power transitions */
    if (UART0->FSR & (UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk | UART_FSR_RX_OVER_IF_Msk)) {
        UART0->FSR = (UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk | UART_FSR_RX_OVER_IF_Msk);
    }
}
