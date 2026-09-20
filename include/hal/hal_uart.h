#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "hal_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_BAUD_9600   = 9600,
    UART_BAUD_19200  = 19200,
    UART_BAUD_38400  = 38400,
    UART_BAUD_115200 = 115200
} uart_baudrate_t;

/**
 * @brief Initialize the UART port connected to the Android Head Unit.
 */
hal_status_t hal_uart_init(uart_baudrate_t baudrate);

/**
 * @brief Non-blocking check and read of a single byte from RX buffer.
 * @param[out] byte Storage for retrieved byte.
 * @return HAL_STATUS_OK if byte was read, HAL_STATUS_TIMEOUT if RX buffer is empty.
 */
hal_status_t hal_uart_read_byte(uint8_t *byte);

/**
 * @brief Read up to max_len bytes from the RX buffer.
 * @return Number of bytes actually read.
 */
size_t hal_uart_read(uint8_t *buffer, size_t max_len);

/**
 * @brief Non-blocking or DMA/buffered write of raw bytes out to Head Unit.
 * @param data Byte array to send.
 * @param len Byte count.
 */
hal_status_t hal_uart_write(const uint8_t *data, size_t len);

/**
 * @brief Flush hardware/software transmit queues.
 */
hal_status_t hal_uart_flush_tx(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
