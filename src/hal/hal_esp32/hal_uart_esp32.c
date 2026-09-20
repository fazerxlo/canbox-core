#include "hal/hal_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#define HU_UART_PORT      UART_NUM_1
#define HU_UART_TX_PIN    GPIO_NUM_17
#define HU_UART_RX_PIN    GPIO_NUM_16
#define HU_UART_BUF_SIZE  512

static bool s_uart_installed = false;

hal_status_t hal_uart_init(uart_baudrate_t baudrate) {
    if (s_uart_installed) {
        uart_driver_delete(HU_UART_PORT);
        s_uart_installed = false;
    }

    uart_config_t uart_config = {
        .baud_rate  = (int)baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(HU_UART_PORT, &uart_config) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }

    if (uart_set_pin(HU_UART_PORT, HU_UART_TX_PIN, HU_UART_RX_PIN, 
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }

    // Allocate ring buffer in ESP-IDF UART driver ISR
    if (uart_driver_install(HU_UART_PORT, HU_UART_BUF_SIZE, HU_UART_BUF_SIZE, 0, NULL, 0) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }

    s_uart_installed = true;
    return HAL_STATUS_OK;
}

hal_status_t hal_uart_read_byte(uint8_t *byte) {
    if (!s_uart_installed || !byte) {
        return HAL_STATUS_ERROR;
    }

    int len = uart_read_bytes(HU_UART_PORT, byte, 1, 0); // Non-blocking
    if (len == 1) {
        return HAL_STATUS_OK;
    }

    return HAL_STATUS_TIMEOUT;
}

size_t hal_uart_read(uint8_t *buffer, size_t max_len) {
    if (!s_uart_installed || !buffer || max_len == 0) {
        return 0;
    }

    int bytes_read = uart_read_bytes(HU_UART_PORT, buffer, (uint32_t)max_len, 0);
    return (bytes_read > 0) ? (size_t)bytes_read : 0;
}

hal_status_t hal_uart_write(const uint8_t *data, size_t len) {
    if (!s_uart_installed || !data || len == 0) {
        return HAL_STATUS_ERROR;
    }

    int bytes_written = uart_write_bytes(HU_UART_PORT, (const char *)data, len);
    return (bytes_written == (int)len) ? HAL_STATUS_OK : HAL_STATUS_ERROR;
}

hal_status_t hal_uart_flush_tx(void) {
    if (!s_uart_installed) {
        return HAL_STATUS_ERROR;
    }

    esp_err_t err = uart_wait_tx_done(HU_UART_PORT, pdMS_TO_TICKS(50));
    return (err == ESP_OK) ? HAL_STATUS_OK : HAL_STATUS_TIMEOUT;
}
