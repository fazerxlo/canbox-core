#include "core/vehicle_profile.h"
#include <stddef.h>

extern const vehicle_profile_t g_profile_psa;
extern const vehicle_profile_t g_profile_vag;

static const vehicle_profile_t *s_available_profiles[] = {
    [VEHICLE_PROFILE_PSA_2004]   = &g_profile_psa,
    [VEHICLE_PROFILE_VAG_PQ35]   = &g_profile_vag,
    [VEHICLE_PROFILE_RENAULT]    = NULL,
    [VEHICLE_PROFILE_TOYOTA_TNGA] = NULL,
};

#if defined(PLATFORM_LINUX)
#include <stdlib.h>
#include <strings.h>
#endif

static const vehicle_profile_t *s_active_profile = &g_profile_psa;

void vehicle_profile_init(void) {
#if defined(PLATFORM_LINUX)
    const char *prof_env = getenv("CANBOX_VEHICLE_PROFILE");
    if (prof_env && prof_env[0] != '\0') {
        if (strcasecmp(prof_env, "psa") == 0 || strcasecmp(prof_env, "psa_2004") == 0 || strcasecmp(prof_env, "peugeot_407") == 0) {
            vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);
        } else if (strcasecmp(prof_env, "vag") == 0 || strcasecmp(prof_env, "vag_pq35") == 0) {
            vehicle_profile_set_active(VEHICLE_PROFILE_VAG_PQ35);
        }
    }
#endif
    if (s_active_profile && s_active_profile->init) {
        s_active_profile->init();
    }
}

bool vehicle_profile_set_active(vehicle_profile_id_t profile_id) {
    if (profile_id >= VEHICLE_PROFILE_COUNT) {
        return false;
    }
    if (s_available_profiles[profile_id] == NULL) {
        return false;
    }
    s_active_profile = s_available_profiles[profile_id];
    if (s_active_profile->init) {
        s_active_profile->init();
    }
    return true;
}

const vehicle_profile_t *vehicle_profile_get_active(void) {
    return s_active_profile;
}

void vehicle_profile_process_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!s_active_profile || !frame || !state) {
        return;
    }

    for (uint8_t i = 0; i < s_active_profile->rule_count; i++) {
        if (s_active_profile->rules[i].can_id == frame->id) {
            if (s_active_profile->rules[i].handler) {
                s_active_profile->rules[i].handler(frame, state);
            }
            break;
        }
    }
}

