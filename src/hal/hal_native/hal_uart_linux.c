#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include "hal/hal_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pty.h>
#include <termios.h>

#define PTY_SYMLINK_PATH "/tmp/ttyCanbox"

static int s_master_fd = -1;

hal_status_t hal_uart_init(uart_baudrate_t baudrate) {
    (void)baudrate; // Ignored for virtual pseudo-terminals

    if (s_master_fd >= 0) {
        close(s_master_fd);
        s_master_fd = -1;
    }

    char slave_name[64];
    struct termios raw_opts;
    int slave_fd = -1;

    // Build raw terminal options (no echo, 8N1, raw binary stream)
    cfmakeraw(&raw_opts);

    if (openpty(&s_master_fd, &slave_fd, slave_name, &raw_opts, NULL) < 0) {
        perror("[UART] openpty failed");
        return HAL_STATUS_ERROR;
    }

    if (slave_fd >= 0) {
        close(slave_fd);
    }

    // Set master non-blocking
    int flags = fcntl(s_master_fd, F_GETFL, 0);
    if (flags < 0 || fcntl(s_master_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("[UART] Setting non-blocking failed");
        close(s_master_fd);
        s_master_fd = -1;
        return HAL_STATUS_ERROR;
    }

    // Expose convenient symlink for host applications
    unlink(PTY_SYMLINK_PATH);
    if (symlink(slave_name, PTY_SYMLINK_PATH) < 0) {
        perror("[UART] Failed to create symlink");
    }

    printf("[UART] Head unit UART simulated on: %s\n", slave_name);
    printf("[UART] Access port symlink available at: %s\n", PTY_SYMLINK_PATH);

    return HAL_STATUS_OK;
}

hal_status_t hal_uart_read_byte(uint8_t *byte) {
    if (s_master_fd < 0 || !byte) {
        return HAL_STATUS_ERROR;
    }

    ssize_t bytes_read = read(s_master_fd, byte, 1);
    if (bytes_read == 1) {
        return HAL_STATUS_OK;
    }

    if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return HAL_STATUS_TIMEOUT; // Buffer empty
    }

    return HAL_STATUS_ERROR;
}

size_t hal_uart_read(uint8_t *buffer, size_t max_len) {
    if (s_master_fd < 0 || !buffer || max_len == 0) {
        return 0;
    }

    ssize_t bytes_read = read(s_master_fd, buffer, max_len);
    if (bytes_read > 0) {
        return (size_t)bytes_read;
    }

    return 0;
}

hal_status_t hal_uart_write(const uint8_t *data, size_t len) {
    if (s_master_fd < 0 || !data || len == 0) {
        return HAL_STATUS_ERROR;
    }

    size_t total_written = 0;
    while (total_written < len) {
        ssize_t written = write(s_master_fd, data + total_written, len - total_written);
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return HAL_STATUS_BUSY;
            }
            return HAL_STATUS_ERROR;
        }
        total_written += (size_t)written;
    }

    return HAL_STATUS_OK;
}

hal_status_t hal_uart_flush_tx(void) {
    if (s_master_fd < 0) {
        return HAL_STATUS_ERROR;
    }
    tcflush(s_master_fd, TCOFLUSH);
    return HAL_STATUS_OK;
}
