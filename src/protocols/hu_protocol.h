#ifndef HU_PROTOCOL_H
#define HU_PROTOCOL_H

#include <stdint.h>
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

void hu_protocol_feed_byte(uint8_t byte);
void hu_protocol_send_heartbeat(void);
void hu_protocol_send_wheel_key(const vehicle_wheel_t *wheel);
void hu_protocol_send_doors(const vehicle_doors_t *doors);
void hu_protocol_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* HU_PROTOCOL_H */
