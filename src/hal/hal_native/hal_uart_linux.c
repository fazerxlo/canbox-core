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

static speed_t baudrate_to_speed(uart_baudrate_t baudrate) {
    switch (baudrate) {
        case UART_BAUD_9600:   return B9600;
        case UART_BAUD_19200:  return B19200;
        case UART_BAUD_38400:  return B38400;
        case UART_BAUD_115200: return B115200;
        default:               return B38400;
    }
}

hal_status_t hal_uart_init(uart_baudrate_t baudrate) {
    if (s_master_fd >= 0) {
        close(s_master_fd);
        s_master_fd = -1;
    }

    const char *baud_env = getenv("CANBOX_UART_BAUD");
    if (baud_env && baud_env[0] != '\0') {
        int b = atoi(baud_env);
        if (b == 9600) baudrate = UART_BAUD_9600;
        else if (b == 19200) baudrate = UART_BAUD_19200;
        else if (b == 38400) baudrate = UART_BAUD_38400;
        else if (b == 115200) baudrate = UART_BAUD_115200;
    }

    const char *uart_dev = getenv("CANBOX_UART_DEVICE");
    if (uart_dev && uart_dev[0] != '\0') {
        s_master_fd = open(uart_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (s_master_fd < 0) {
            perror("[UART] Failed to open physical serial device");
            return HAL_STATUS_ERROR;
        }

        struct termios tty;
        if (tcgetattr(s_master_fd, &tty) != 0) {
            perror("[UART] tcgetattr failed");
            close(s_master_fd);
            s_master_fd = -1;
            return HAL_STATUS_ERROR;
        }

        cfmakeraw(&tty);
        speed_t speed = baudrate_to_speed(baudrate);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag &= ~PARENB;

        if (tcsetattr(s_master_fd, TCSANOW, &tty) != 0) {
            perror("[UART] tcsetattr failed");
            close(s_master_fd);
            s_master_fd = -1;
            return HAL_STATUS_ERROR;
        }

        printf("[UART] Physical serial device opened: %s (Baud: %d)\n", uart_dev, (int)baudrate);
        return HAL_STATUS_OK;
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
