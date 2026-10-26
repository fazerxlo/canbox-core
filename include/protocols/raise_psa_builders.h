#ifndef RAISE_PSA_BUILDERS_H
#define RAISE_PSA_BUILDERS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "profiles/peugeot_407.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t build_raise_stalk_key(uint8_t key_id, uint8_t state, uint8_t *out_buf, size_t max_len);
size_t build_raise_hvac_packet(const hvac_state_t *st, uint8_t *out_buf, size_t max_len);
size_t build_raise_rear_radar(uint8_t rl, uint8_t rc, uint8_t rr,
                              uint8_t fl, uint8_t fc, uint8_t fr,
                              uint8_t *out_buf, size_t max_len);
size_t build_raise_front_radar(uint8_t fl, uint8_t fc, uint8_t fr,
                               uint8_t *out_buf, size_t max_len);
size_t build_raise_instant_fuel(uint16_t instant_fuel_dkl, uint16_t dte_range_km,
                                uint16_t dest_dist_km, uint8_t *out, size_t max_len);
size_t build_raise_trip1(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out);
size_t build_raise_trip2(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out);
size_t build_raise_outside_temp(uint8_t temp_raw, uint8_t *out, size_t max_len);
size_t build_raise_reverse_state(bool reverse_active, uint8_t *out, size_t max_len);
size_t build_raise_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len);
size_t build_raise_steering_angle(int16_t angle_deci_deg, uint8_t *out);
size_t build_raise_amplifier(const jbl_amplifier_state_t *st, uint8_t *out, size_t max_len);
size_t build_raise_cd_changer(const cd_changer_state_t *st, uint8_t *out, size_t max_len);
size_t build_raise_rds_name(const char *name, uint8_t *out, size_t max_len);
size_t build_raise_tpms_numeric(const canbox_tpms_state_t *tpms, uint8_t *out);
size_t build_raise_tpms_temp_alarms(const canbox_tpms_state_t *tpms, uint8_t *out);
size_t build_raise_tpms_discrete(const uint8_t alarms[4], uint8_t *out);
size_t build_raise_start_stop(bool is_active, uint32_t stop_time_sec, uint8_t *out);
size_t build_raise_cruise_memory(bool active, uint8_t target_spd, const uint8_t presets[5], uint8_t *out);
size_t build_raise_adas(const canbox_adas_state_t *adas, uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* RAISE_PSA_BUILDERS_H */
