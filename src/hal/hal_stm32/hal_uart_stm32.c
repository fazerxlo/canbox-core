#include "hal/hal_uart.h"
#include "core/ring_buffer.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"

#define UART_RX_RING_SIZE 128

static uint8_t s_uart_rx_storage[UART_RX_RING_SIZE];
static ring_buffer_t s_uart_rx_rb;

hal_status_t hal_uart_init(uart_baudrate_t baudrate) {
    ring_buffer_init(&s_uart_rx_rb, s_uart_rx_storage, sizeof(uint8_t), UART_RX_RING_SIZE);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    // PA9 (TX) Alternate Function Push-Pull
    LL_GPIO_InitTypeDef gpio_init;
    LL_GPIO_StructInit(&gpio_init);
    gpio_init.Pin = LL_GPIO_PIN_9;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &gpio_init);

    // PA10 (RX) Floating Input
    gpio_init.Pin = LL_GPIO_PIN_10;
    gpio_init.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &gpio_init);

    LL_USART_InitTypeDef usart_init;
    LL_USART_StructInit(&usart_init);
    usart_init.BaudRate = baudrate;
    usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART1, &usart_init);

    LL_USART_EnableIT_RXNE(USART1);
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);

    LL_USART_Enable(USART1);

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
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1, data[i]);
    }
    while (!LL_USART_IsActiveFlag_TC(USART1));
    return HAL_STATUS_OK;
}

hal_status_t hal_uart_flush_tx(void) {
    while (!LL_USART_IsActiveFlag_TC(USART1));
    return HAL_STATUS_OK;
}

void USART1_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        uint8_t byte = LL_USART_ReceiveData8(USART1);
        ring_buffer_push(&s_uart_rx_rb, &byte);
    }
    if (LL_USART_IsActiveFlag_ORE(USART1)) {
        LL_USART_ClearFlag_ORE(USART1);
    }
}
