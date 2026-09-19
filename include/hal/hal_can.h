#ifndef HAL_CAN_H
#define HAL_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_system.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_MAX_DLC 8

typedef enum {
    CAN_BAUD_125K = 125000,
    CAN_BAUD_250K = 250000,
    CAN_BAUD_500K = 500000,
    CAN_BAUD_1M   = 1000000
} can_baudrate_t;

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[CAN_MAX_DLC];
    bool     is_extended;
    bool     is_remote;
    uint32_t timestamp_ms;
} can_frame_t;

typedef struct {
    uint32_t id;
    uint32_t mask;
    bool     is_extended;
} can_filter_t;

/**
 * @brief Initialize the CAN controller and physical transceiver.
 */
hal_status_t hal_can_init(can_baudrate_t baudrate);

/**
 * @brief Configure hardware accept filters.
 * @param filters Array of filter definitions.
 * @param count Number of filters in the array.
 */
hal_status_t hal_can_set_filters(const can_filter_t *filters, uint8_t count);

/**
 * @brief Push a CAN frame to the hardware TX mailbox/buffer.
 * @return HAL_OK on enqueue, HAL_BUSY or HAL_OVERFLOW if mailbox/ringbuffer is full.
 */
hal_status_t hal_can_send(const can_frame_t *frame);

/**
 * @brief Non-blocking read of the next received CAN frame.
 * @param[out] frame Destination pointer for retrieved frame.
 * @return HAL_OK if frame retrieved, HAL_TIMEOUT/HAL_EMPTY if no frames waiting.
 */
hal_status_t hal_can_receive(can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_H */
