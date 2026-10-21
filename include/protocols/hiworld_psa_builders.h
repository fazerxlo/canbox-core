#ifndef HIWORLD_PSA_BUILDERS_H
#define HIWORLD_PSA_BUILDERS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "core/can_router.h"
#include "profiles/peugeot_407.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t build_hiworld_radar(const vehicle_radar_t *radar, uint8_t *out, size_t max_len);
size_t build_hiworld_trip_instant(const vehicle_trip_t *trip, uint8_t *out, size_t max_len);
size_t build_hiworld_trip1(const vehicle_trip_t *trip, uint8_t *out, size_t max_len);
size_t build_hiworld_trip2(const vehicle_trip_t *trip, uint8_t *out, size_t max_len);
size_t build_hiworld_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len);
size_t build_hiworld_doors_state(const vehicle_doors_t *doors, uint8_t *out, size_t max_len);
size_t build_hiworld_audio_settings(const vehicle_audio_settings_t *audio, uint8_t *out, size_t max_len);
size_t build_hiworld_radio_state(const vehicle_radio_t *radio, uint8_t *out_buf, size_t max_out);
size_t build_hiworld_radio_text(const char *text, uint8_t text_len, uint8_t *out_buf, size_t max_out);
size_t build_hiworld_media_state(const vehicle_cdc_t *cdc, uint8_t *out_buf, size_t max_out);
size_t build_hiworld_radio_preset_freqs(const vehicle_radio_t *radio, uint8_t *out_buf, size_t max_out);
size_t build_hiworld_radio_preset_names(const vehicle_radio_t *radio, uint8_t *out_buf, size_t max_out);
size_t build_hiworld_tpms_numeric(const vehicle_tpms_t *tpms, uint8_t *out, size_t max_len);
size_t build_hiworld_tpms_discrete(const vehicle_tpms_t *tpms, uint8_t *out, size_t max_len);
size_t build_hiworld_alert_single(const vehicle_alert_item_t *alert, uint8_t *out, size_t max_len);
size_t build_hiworld_alerts_summary(const vehicle_alert_item_t *items, uint8_t count, uint8_t *out, size_t max_len);
size_t build_hiworld_central_state1(const vehicle_config_t *cfg, uint8_t *out, size_t max_len);
size_t build_hiworld_central_state2(const vehicle_config_t *cfg, const vehicle_state_t *st, uint8_t *out, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* HIWORLD_PSA_BUILDERS_H */
