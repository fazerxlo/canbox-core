#include "protocols/hiworld_car_mapping.h"
#include <stddef.h>

static const hiworld_car_entry_t s_hiworld_car_table[] = {
    { HIWORLD_CAR_MODEL_CITROEN_C_QUATRE_08,  VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C-Quatre (2008+)" },
    { HIWORLD_CAR_MODEL_CITROEN_C4_16,        VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C4 / Quatre (2016+)" },
    { HIWORLD_CAR_MODEL_CITROEN_C4L_13,       VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C4L (2013+)" },
    { HIWORLD_CAR_MODEL_CITROEN_C5_10,        VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C5 (2010+)" },
    { HIWORLD_CAR_MODEL_CITROEN_C5_13,        VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C5 (2013+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_307_04,       VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 307 (2004-2011)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_308_12,       VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 308 (2012-2016)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_408_10,       VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 408 (2010+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_508_LOW_11,   VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 508 Low-Trim (2011+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_508_HIGH_11,  VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 508 High-Trim (2011+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_3008_13,      VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 3008 (2013+)" },
    { HIWORLD_CAR_MODEL_CITROEN_DS5_12,       VEHICLE_PROFILE_PSA_2004, 500000, "Citroen DS5 (2012+)" },
    { HIWORLD_CAR_MODEL_CITROEN_DS5LS_12,     VEHICLE_PROFILE_PSA_2004, 500000, "Citroen DS5LS (2012+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_2008_14,      VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot 2008 (2014+)" },
    { HIWORLD_CAR_MODEL_CITROEN_DS4_12,       VEHICLE_PROFILE_PSA_2004, 500000, "Citroen DS4 (2012+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_308S_408_14,  VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot 308S / 408 (2014-2019)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_3008_KEEP_13, VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 3008 Keep-Screen (2013)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_301_12,       VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 301 / Citroen C-Elysee" },
    { HIWORLD_CAR_MODEL_CITROEN_C3_XR_15,     VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C3-XR (2015+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_4008_5008_17, VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot 4008 / 5008 (2017+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_508_FL_15,    VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot 508 Facelift (2015+)" },
    { HIWORLD_CAR_MODEL_CITROEN_DS6_16,       VEHICLE_PROFILE_PSA_2004, 500000, "Citroen DS6 (2016+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_301_19,       VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot 301 (2019+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_RIFTER_HI_19, VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot Rifter High (2019+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_RIFTER_LO_19, VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot Rifter Low (2019+)" },
    { HIWORLD_CAR_MODEL_CITROEN_TIANYI_C5_17, VEHICLE_PROFILE_PSA_2004, 500000, "Citroen Tianyi C5 Aircross (2017+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_308_CC_11,    VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 308 CC (2011+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_407_06,       VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot 407 (2004-2011)" },
    { HIWORLD_CAR_MODEL_OPEL_COMBO_CORSA_19,  VEHICLE_PROFILE_PSA_2004, 500000, "Opel Combo / Corsa / Grandland" },
    { HIWORLD_CAR_MODEL_CITROEN_C3_23,        VEHICLE_PROFILE_PSA_2004, 500000, "Citroen C3 (2023+)" },
    { HIWORLD_CAR_MODEL_CITROEN_C4_09,        VEHICLE_PROFILE_PSA_2004, 125000, "Citroen C4 (2009+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_3008_22,      VEHICLE_PROFILE_PSA_2004, 500000, "Peugeot 3008 (2022+)" },
    { HIWORLD_CAR_MODEL_PEUGEOT_PARTNER_09,   VEHICLE_PROFILE_PSA_2004, 125000, "Peugeot Partner (2009+)" },
    { HIWORLD_CAR_MODEL_CITROEN_BERLINGO_17,  VEHICLE_PROFILE_PSA_2004, 500000, "Citroen Berlingo (2017+)" }
};

#define HIWORLD_CAR_TABLE_SIZE (sizeof(s_hiworld_car_table) / sizeof(s_hiworld_car_table[0]))

bool hiworld_car_mapping_get_profile(uint8_t model_id, vehicle_profile_id_t *out_profile_id) {
    if (!out_profile_id) {
        return false;
    }

    for (size_t i = 0; i < HIWORLD_CAR_TABLE_SIZE; i++) {
        if (s_hiworld_car_table[i].model_id == model_id) {
            *out_profile_id = s_hiworld_car_table[i].profile_id;
            return true;
        }
    }

    return false;
}

bool hiworld_car_mapping_get_model_id(vehicle_profile_id_t profile_id, uint8_t *out_model_id) {
    if (!out_model_id) {
        return false;
    }

    for (size_t i = 0; i < HIWORLD_CAR_TABLE_SIZE; i++) {
        if (s_hiworld_car_table[i].profile_id == profile_id) {
            *out_model_id = s_hiworld_car_table[i].model_id;
            return true;
        }
    }

    return false;
}

uint32_t hiworld_car_mapping_get_baud_rate(uint8_t model_id) {
    for (size_t i = 0; i < HIWORLD_CAR_TABLE_SIZE; i++) {
        if (s_hiworld_car_table[i].model_id == model_id) {
            return s_hiworld_car_table[i].baud_rate;
        }
    }
    return 125000;
}

