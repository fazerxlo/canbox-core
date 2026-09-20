#ifndef HU_PROTOCOL_DRIVER_H
#define HU_PROTOCOL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HU_PROTOCOL_RAISE = 0,
    HU_PROTOCOL_HIWORLD,
    HU_PROTOCOL_BAGOO,
    HU_PROTOCOL_COUNT
} hu_protocol_id_t;

typedef struct {
    hu_protocol_id_t id;
    const char      *name;
    void           (*init)(void);
    void           (*feed_byte)(uint8_t byte);
    void           (*send_wheel_key)(const vehicle_wheel_t *wheel);
    void           (*send_doors)(const vehicle_doors_t *doors);
    void           (*send_telemetry)(uint16_t speed, uint16_t rpm, int16_t angle);
    void           (*send_heartbeat)(void);
} hu_protocol_driver_t;

bool hu_protocol_set_active(hu_protocol_id_t protocol_id);
const hu_protocol_driver_t *hu_protocol_get_active(void);
void hu_protocol_init(void);

#ifdef __cplusplus
}
#endif

#endif /* HU_PROTOCOL_DRIVER_H */

