#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include "hal/hal_can.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_INTERFACE_NAME "vcan0"

static int s_can_fd = -1;

hal_status_t hal_can_init(can_baudrate_t baudrate) {
    (void)baudrate; // Baudrate is determined by the virtual link setup on Linux host

    s_can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s_can_fd < 0) {
        perror("[CAN] Socket creation failed");
        return HAL_STATUS_ERROR;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_INTERFACE_NAME, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(s_can_fd, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "[CAN] Interface %s not found. Run:\n"
                        "  sudo ip link add dev %s type vcan\n"
                        "  sudo ip link set up %s\n",
                CAN_INTERFACE_NAME, CAN_INTERFACE_NAME, CAN_INTERFACE_NAME);
        close(s_can_fd);
        s_can_fd = -1;
        return HAL_STATUS_ERROR;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s_can_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[CAN] Socket bind failed");
        close(s_can_fd);
        s_can_fd = -1;
        return HAL_STATUS_ERROR;
    }

    // Set non-blocking operation
    int flags = fcntl(s_can_fd, F_GETFL, 0);
    if (flags < 0 || fcntl(s_can_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("[CAN] Setting non-blocking mode failed");
        close(s_can_fd);
        s_can_fd = -1;
        return HAL_STATUS_ERROR;
    }

    printf("[CAN] Initialized on %s\n", CAN_INTERFACE_NAME);
    return HAL_STATUS_OK;
}

hal_status_t hal_can_set_filters(const can_filter_t *filters, uint8_t count) {
    if (s_can_fd < 0 || count == 0 || !filters) {
        return HAL_STATUS_ERROR;
    }

    struct can_filter rfilter[count];
    for (uint8_t i = 0; i < count; i++) {
        rfilter[i].can_id = filters[i].id;
        if (filters[i].is_extended) {
            rfilter[i].can_id |= CAN_EFF_FLAG;
            rfilter[i].can_mask = (filters[i].mask & CAN_EFF_MASK) | CAN_EFF_FLAG;
        } else {
            rfilter[i].can_mask = (filters[i].mask & CAN_SFF_MASK);
        }
    }

    if (setsockopt(s_can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        perror("[CAN] Failed to apply filters");
        return HAL_STATUS_ERROR;
    }

    return HAL_STATUS_OK;
}

hal_status_t hal_can_send(const can_frame_t *frame) {
    if (s_can_fd < 0 || !frame) {
        return HAL_STATUS_ERROR;
    }

    struct can_frame linux_frame;
    memset(&linux_frame, 0, sizeof(linux_frame));

    linux_frame.can_id = frame->id;
    if (frame->is_extended) {
        linux_frame.can_id |= CAN_EFF_FLAG;
    }
    if (frame->is_remote) {
        linux_frame.can_id |= CAN_RTR_FLAG;
    }

    linux_frame.can_dlc = (frame->dlc > CAN_MAX_DLC) ? CAN_MAX_DLC : frame->dlc;
    memcpy(linux_frame.data, frame->data, linux_frame.can_dlc);

    ssize_t bytes_written = write(s_can_fd, &linux_frame, sizeof(linux_frame));
    if (bytes_written != sizeof(linux_frame)) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HAL_STATUS_BUSY;
        }
        return HAL_STATUS_ERROR;
    }

    return HAL_STATUS_OK;
}

hal_status_t hal_can_receive(can_frame_t *frame) {
    if (s_can_fd < 0 || !frame) {
        return HAL_STATUS_ERROR;
    }

    struct can_frame linux_frame;
    ssize_t bytes_read = read(s_can_fd, &linux_frame, sizeof(linux_frame));

    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HAL_STATUS_TIMEOUT; // Queue empty
        }
        return HAL_STATUS_ERROR;
    }

    if ((size_t)bytes_read < sizeof(linux_frame)) {
        return HAL_STATUS_ERROR;
    }

    frame->is_extended = (linux_frame.can_id & CAN_EFF_FLAG) != 0;
    frame->is_remote   = (linux_frame.can_id & CAN_RTR_FLAG) != 0;
    frame->id          = frame->is_extended ? (linux_frame.can_id & CAN_EFF_MASK)
                                            : (linux_frame.can_id & CAN_SFF_MASK);
    frame->dlc         = linux_frame.can_dlc;
    memcpy(frame->data, linux_frame.data, linux_frame.can_dlc);
    frame->timestamp_ms = hal_get_tick_ms();

    return HAL_STATUS_OK;
}
