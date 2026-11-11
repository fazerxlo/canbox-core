#ifndef CAN_SNIFFER_H
#define CAN_SNIFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SNIFFER_MAX_IDS             128
#define SNIFFER_EVENT_QUEUE_SIZE    32
#define SNIFFER_MAX_BATCH_PAYLOAD   52
#define SNIFFER_MAX_DURATION_S      30

#define SNIFFER_MODE_STOP           0x00
#define SNIFFER_MODE_DELTA          0x01

#define SNIFFER_SUBTYPE_LIVE        0x01
#define SNIFFER_SUBTYPE_SUMMARY     0x02
#define SNIFFER_SUBTYPE_FINISHED    0xFF

/**
 * @brief Initialize the CAN delta sniffer subsystem.
 */
void can_sniffer_init(void);

/**
 * @brief Start sniffer session.
 * @param mode 0x01: Start Delta + Period sniffer.
 * @param duration_s Duration in seconds (1..30, hard-capped to max 30s).
 * @param now_ms Current system timestamp in milliseconds.
 */
void can_sniffer_start(uint8_t mode, uint8_t duration_s, uint32_t now_ms);

/**
 * @brief Stop sniffer session and begin flushing summary.
 * @param now_ms Current system timestamp in milliseconds.
 */
void can_sniffer_stop(uint32_t now_ms);

/**
 * @brief Returns true if sniffer is currently active or flushing summary.
 */
bool can_sniffer_is_active(void);

/**
 * @brief Process incoming or outgoing CAN frame for sniffer tracking.
 * @param frame Pointer to CAN frame.
 * @param is_tx true if transmitted by canbox-core, false if received from bus.
 * @param now_ms Current system timestamp in milliseconds.
 */
void can_sniffer_on_can_frame(const can_frame_t *frame, bool is_tx, uint32_t now_ms);

/**
 * @brief Paced periodic worker called at 10 ms intervals.
 * Flushes at most ONE 0xD1 frame (max payload <= 52 bytes) per call.
 * @param now_ms Current system timestamp in milliseconds.
 */
void can_sniffer_poll_10ms(uint32_t now_ms);

/**
 * @brief Get total count of dropped events due to queue/table overflow.
 */
uint16_t can_sniffer_get_dropped_count(void);

/**
 * @brief Get number of unique CAN IDs tracked in the current session.
 */
uint8_t can_sniffer_get_unique_id_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_SNIFFER_H */

