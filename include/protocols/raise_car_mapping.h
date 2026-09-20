#ifndef RAISE_CAR_MAPPING_H
#define RAISE_CAR_MAPPING_H

#include <stdint.h>
#include <stdbool.h>
#include "core/vehicle_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Raise Brand Identifiers */
#define RAISE_BRAND_PSA      0x01
#define RAISE_BRAND_VAG      0x02
#define RAISE_BRAND_RENAULT  0x03
#define RAISE_BRAND_TOYOTA   0x04

/* Raise Model Identifiers */
#define RAISE_MODEL_PSA_2004 0x01
#define RAISE_MODEL_VAG_PQ35 0x01

typedef struct {
    uint8_t              brand;
    uint8_t              model;
    vehicle_profile_id_t profile_id;
    const char          *description;
} raise_car_entry_t;

bool raise_car_mapping_get_profile(uint8_t brand, uint8_t model, vehicle_profile_id_t *out_profile_id);
bool raise_car_mapping_get_codes(vehicle_profile_id_t profile_id, uint8_t *out_brand, uint8_t *out_model);

#ifdef __cplusplus
}
#endif

#endif /* RAISE_CAR_MAPPING_H */

