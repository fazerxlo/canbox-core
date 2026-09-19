#ifndef VEHICLE_PROFILE_H
#define VEHICLE_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include "hal/hal_can.h"
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VEHICLE_PROFILE_PSA_2004 = 0,
    VEHICLE_PROFILE_VAG_PQ35,
    VEHICLE_PROFILE_RENAULT,
    VEHICLE_PROFILE_TOYOTA_TNGA,
    VEHICLE_PROFILE_COUNT
} vehicle_profile_id_t;

typedef void (*profile_can_decoder_t)(const can_frame_t *frame, vehicle_state_t *state);

typedef struct {
    uint32_t              can_id;
    profile_can_decoder_t handler;
} profile_can_rule_t;

typedef struct {
    vehicle_profile_id_t       id;
    const char                *name;
    can_baudrate_t             default_baud;
    const profile_can_rule_t  *rules;
    uint8_t                    rule_count;
    void                     (*init)(void);
} vehicle_profile_t;

bool vehicle_profile_set_active(vehicle_profile_id_t profile_id);
const vehicle_profile_t *vehicle_profile_get_active(void);
void vehicle_profile_process_frame(const can_frame_t *frame, vehicle_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_PROFILE_H */
