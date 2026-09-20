#include "protocols/raise_car_mapping.h"
#include <stddef.h>

static const raise_car_entry_t s_raise_car_table[] = {
    {
        .brand       = RAISE_BRAND_PSA,
        .model       = RAISE_MODEL_PSA_2004,
        .profile_id  = VEHICLE_PROFILE_PSA_2004,
        .description = "PSA CAN2004/CAN2010 (307/308/407/408/508/C4/C5)"
    },
    {
        .brand       = RAISE_BRAND_VAG,
        .model       = RAISE_MODEL_VAG_PQ35,
        .profile_id  = VEHICLE_PROFILE_VAG_PQ35,
        .description = "VAG PQ35/PQ46 (Golf 5/6, Passat B6/B7, Octavia 2)"
    }
};

#define RAISE_CAR_TABLE_SIZE (sizeof(s_raise_car_table) / sizeof(s_raise_car_table[0]))

bool raise_car_mapping_get_profile(uint8_t brand, uint8_t model, vehicle_profile_id_t *out_profile_id) {
    if (!out_profile_id) {
        return false;
    }

    for (size_t i = 0; i < RAISE_CAR_TABLE_SIZE; i++) {
        if (s_raise_car_table[i].brand == brand && s_raise_car_table[i].model == model) {
            *out_profile_id = s_raise_car_table[i].profile_id;
            return true;
        }
    }

    return false;
}

bool raise_car_mapping_get_codes(vehicle_profile_id_t profile_id, uint8_t *out_brand, uint8_t *out_model) {
    if (!out_brand || !out_model) {
        return false;
    }

    for (size_t i = 0; i < RAISE_CAR_TABLE_SIZE; i++) {
        if (s_raise_car_table[i].profile_id == profile_id) {
            *out_brand = s_raise_car_table[i].brand;
            *out_model = s_raise_car_table[i].model;
            return true;
        }
    }

    return false;
}

