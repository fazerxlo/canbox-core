#ifndef HIWORLD_CAR_MAPPING_H
#define HIWORLD_CAR_MAPPING_H

#include <stdint.h>
#include <stdbool.h>
#include "core/vehicle_profile.h"
#include "protocols/hiworld_connection.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t              model_id;
    vehicle_profile_id_t profile_id;
    uint32_t             baud_rate;
    const char          *description;
} hiworld_car_entry_t;

bool hiworld_car_mapping_get_profile(uint8_t model_id, vehicle_profile_id_t *out_profile_id);
bool hiworld_car_mapping_get_model_id(vehicle_profile_id_t profile_id, uint8_t *out_model_id);
uint32_t hiworld_car_mapping_get_baud_rate(uint8_t model_id);

#ifdef __cplusplus
}
#endif

#endif /* HIWORLD_CAR_MAPPING_H */

