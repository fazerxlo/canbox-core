#include "protocols/hiworld_psa_builders.h"
#include "proto_hiworld.h"
#include "core/can_router.h"
#include <string.h>

size_t build_hiworld_radar(const vehicle_radar_t *radar, uint8_t *out, size_t max_len) {
    if (!radar || !out || max_len < 17) {
        return 0;
    }

    out[0] = HIWORLD_SYNC_1;           /* 0x5A */
    out[1] = HIWORLD_SYNC_2;           /* 0xA5 */
    out[2] = 0x0C;                     /* Length: 12 Payload bytes */
    out[3] = HIWORLD_CMD_RADAR;        /* Cmd: 0x41 */

    out[4] = radar->rear_left_outer;
    out[5] = radar->rear_left_center;
    out[6] = radar->rear_right_center;
    out[7] = radar->rear_right_outer;

    out[8]  = radar->front_left_outer;
    out[9]  = radar->front_left_center;
    out[10] = radar->front_right_center;
    out[11] = radar->front_right_outer;

    out[12] = 0x01;                    /* Radar active / enabled flag */
    out[13] = 0x00;                    /* Reserved */
    out[14] = 0x3F;                    /* 6-sensor configuration mask */
    out[15] = 0x05;                    /* Distance scale / max zone steps */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 15; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[16] = (uint8_t)((sum - 1) & 0xFF);

    return 17;
}

size_t build_hiworld_telemetry_1a(const vehicle_state_t *state, uint8_t *out, size_t max_len) {
    if (!state || !out || max_len < 17) {
        return 0;
    }

    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x0C;                       /* Length: 12 payload bytes */
    out[3] = HIWORLD_CMD_POWERTRAIN_TELEMETRY; /* Opcode: 0x1A */

    /* Byte 0: Status Flags */
    uint8_t flags = 0;
    if (state->ignition_on)      flags |= 0x01;
    if (state->rpm > 400)         flags |= 0x02;
    if (state->reverse_gear)      flags |= 0x04;
    if (state->handbrake)         flags |= 0x08;
    if (state->fuel_low_warning)  flags |= 0x10;
    out[4] = flags;

    /* Byte 1: Gear Position (0=P, 1=R, 2=N, 3=D, 4=M1, etc.) */
    out[5] = state->gear;

    /* Byte 2..3: Vehicle Speed in km/h */
    out[6] = (uint8_t)(state->speed_kmh >> 8);
    out[7] = (uint8_t)(state->speed_kmh & 0xFF);

    /* Byte 4..5: Engine RPM */
    out[8] = (uint8_t)(state->rpm >> 8);
    out[9] = (uint8_t)(state->rpm & 0xFF);

    /* Byte 6: Coolant Temperature (°C) */
    out[10] = (uint8_t)(state->coolant_temp_c);

    /* Byte 7: Oil Temperature (°C) */
    out[11] = (uint8_t)(state->oil_temp_c);

    /* Byte 8: Fuel Level in Liters */
    out[12] = state->fuel_liters;

    /* Byte 9: Fuel Percentage (0..100%) */
    out[13] = state->fuel_percentage;

    /* Byte 10..11: Instantaneous Fuel Consumption (0.1 L/100km, or 0xFFFF) */
    uint16_t instant_fuel = state->trip.instant_fuel_deci;
    out[14] = (uint8_t)(instant_fuel >> 8);
    out[15] = (uint8_t)(instant_fuel & 0xFF);

    /* Checksum: (Length + Opcode + sum(payload) - 1) & 0xFF */
    uint8_t sum = 0;
    for (size_t i = 2; i <= 15; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[16] = (uint8_t)((sum - 1) & 0xFF);

    return 17;
}

size_t build_hiworld_trip_instant(const vehicle_trip_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 15) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x0A;               /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P0; /* Cmd 0x13 */
    out[4] = (uint8_t)(trip->instant_fuel_deci >> 8);
    out[5] = (uint8_t)(trip->instant_fuel_deci & 0xFF);
    out[6] = (uint8_t)(trip->range_km >> 8);
    out[7] = (uint8_t)(trip->range_km & 0xFF);
    out[8] = (uint8_t)(trip->dest_dist_km >> 8);
    out[9] = (uint8_t)(trip->dest_dist_km & 0xFF);
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x00;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);
    return 15;
}

size_t build_hiworld_trip1(const vehicle_trip_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 15) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x0A;               /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P1; /* Cmd 0x14 */
    out[4] = (uint8_t)(trip->trip1_avg_fuel >> 8);
    out[5] = (uint8_t)(trip->trip1_avg_fuel & 0xFF);
    out[6] = 0x00;               /* Reserved */
    out[7] = trip->trip1_avg_speed;
    out[8] = (uint8_t)(trip->trip1_distance_km >> 8);
    out[9] = (uint8_t)(trip->trip1_distance_km & 0xFF);
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x00;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);
    return 15;
}

size_t build_hiworld_trip2(const vehicle_trip_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 15) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x0A;               /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P2; /* Cmd 0x15 */
    out[4] = (uint8_t)(trip->trip2_avg_fuel >> 8);
    out[5] = (uint8_t)(trip->trip2_avg_fuel & 0xFF);
    out[6] = 0x00;               /* Reserved */
    out[7] = trip->trip2_avg_speed;
    out[8] = (uint8_t)(trip->trip2_distance_km >> 8);
    out[9] = (uint8_t)(trip->trip2_distance_km & 0xFF);
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x00;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);
    return 15;
}

size_t build_hiworld_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len) {
    if (!doors || !out || max_len < 15) {
        return 0;
    }

    out[0] = HIWORLD_SYNC_1;        /* 0x5A */
    out[1] = HIWORLD_SYNC_2;        /* 0xA5 */
    out[2] = 0x0A;                  /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_DOORS;     /* Cmd: 0x12 */
    out[4] = 0x00;                  /* Byte 0 */
    out[5] = 0x04;                  /* Byte 1 */

    uint8_t b2 = 0x04;              /* Base active flag */
    if (doors->driver_door)     b2 |= 0x80;
    if (doors->pass_door)       b2 |= 0x40;
    if (doors->rear_left_door)  b2 |= 0x20;
    if (doors->rear_right_door) b2 |= 0x10;
    if (doors->trunk)           b2 |= 0x08;
    if (doors->hood)            b2 |= 0x04;
    out[6] = b2;

    out[7]  = 0x00;
    out[8]  = 0x00;
    out[9]  = 0x00;
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x03;                 /* Byte 9: Sub-type */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);

    return 15;
}

size_t build_hiworld_doors_state(const vehicle_doors_t *doors, uint8_t *out, size_t max_len) {
    if (!doors || !out || max_len < 15) {
        return 0;
    }

    out[0] = HIWORLD_SYNC_1;        /* 0x5A */
    out[1] = HIWORLD_SYNC_2;        /* 0xA5 */
    out[2] = 0x0A;                  /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_DOORS;     /* Cmd: 0x12 */
    out[4] = 0x00;                  /* Byte 0 */
    out[5] = 0x04;                  /* Byte 1 */

    uint8_t b2 = 0x04;              /* Base active flag */
    if (doors->door_driver)     b2 |= 0x80;
    if (doors->door_passenger)  b2 |= 0x40;
    if (doors->door_rear_left)  b2 |= 0x20;
    if (doors->door_rear_right) b2 |= 0x10;
    if (doors->trunk)           b2 |= 0x08;
    if (doors->hood)            b2 |= 0x04;
    out[6] = b2;

    out[7]  = 0x00;
    out[8]  = 0x00;
    out[9]  = 0x00;
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x03;                 /* Byte 9: Sub-type */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);

    return 15;
}

size_t build_hiworld_audio_settings(const vehicle_audio_settings_t *audio, uint8_t *out, size_t max_len) {
    if (!audio || !out || max_len < 13) {
        return 0;
    }

    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x08; /* Length */
    out[3] = HIWORLD_CMD_AUDIO_SETTINGS; /* Opcode Cmd 0x56 */

    out[4] = (uint8_t)((audio->is_muted ? 0x80 : 0x00) | (audio->active_mode & 0x0F));
    out[5] = audio->bass;
    out[6] = audio->treble;
    out[7] = audio->balance;
    out[8] = audio->fader;
    out[9] = audio->eq_preset;
    out[10] = (uint8_t)((audio->loudness ? 0x10 : 0x00) | (audio->speed_volume ? 0x01 : 0x00));
    out[11] = audio->master_volume;

    uint16_t sum = (uint16_t)(out[2] + out[3]);
    for (size_t i = 4; i < 12; i++) {
        sum = (uint16_t)(sum + out[i]);
    }
    out[12] = (uint8_t)((sum - 1) & 0xFF);

    return 13;
}

size_t build_hiworld_radio_state(const vehicle_radio_t *radio, uint8_t *out_buf, size_t max_out) {
    if (!radio || !out_buf || max_out < 19) return 0;

    uint8_t len = 14;
    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = len;
    out_buf[3] = HIWORLD_CMD_CAR_RADIO_STATE; /* 0x84 */

    /* If in external source, send source code; otherwise send radio band */
    if (radio->source_mode >= 0x20) {
        if (radio->source_mode == 0x30) {
            out_buf[4] = 0x20; /* Internal CD */
        } else {
            out_buf[4] = radio->source_mode;
        }
    } else {
        out_buf[4] = radio->band;
    }

    if (radio->source_mode < 0x20 && radio->power_status != 0) {
        out_buf[5] = (uint8_t)(radio->freq_0_1mhz & 0xFF);
        out_buf[6] = (uint8_t)((radio->freq_0_1mhz >> 8) & 0xFF);
        out_buf[7] = radio->preset_slot;
        out_buf[8] = radio->indicators;
    } else {
        out_buf[5] = 0x00;
        out_buf[6] = 0x00;
        out_buf[7] = 0x00;
        out_buf[8] = 0x00;
    }

    out_buf[9] = radio->power_status;

    /* 8-Byte ASCII Station Name (space padded, blank for external sources) */
    for (size_t i = 0; i < 8; i++) {
        out_buf[10 + i] = (radio->source_mode < 0x20 && radio->station_name[i] != '\0') ? (uint8_t)radio->station_name[i] : (uint8_t)' ';
    }

    /* Checksum: ((LEN + CMD + sum(DATA)) - 1) & 0xFF */
    uint16_t sum = len + HIWORLD_CMD_CAR_RADIO_STATE;
    for (size_t i = 4; i < (size_t)(4 + len); i++) {
        sum += out_buf[i];
    }
    out_buf[4 + len] = (uint8_t)((sum - 1) & 0xFF);

    return 5 + len;
}

size_t build_hiworld_radio_text(const char *text, uint8_t text_len, uint8_t *out_buf, size_t max_out) {
    if (!text || text_len == 0 || !out_buf) return 0;

    uint8_t len = text_len;
    if (len > PSA_RD4_MAX_RADIO_TEXT_LEN) len = PSA_RD4_MAX_RADIO_TEXT_LEN;
    if (max_out < (size_t)(5 + len)) return 0;

    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = len;
    out_buf[3] = HIWORLD_CMD_RADIO_TEXT; /* 0x86 */

    memcpy(&out_buf[4], text, len);

    uint16_t sum = len + HIWORLD_CMD_RADIO_TEXT;
    for (size_t i = 0; i < len; i++) {
        sum += (uint8_t)text[i];
    }
    out_buf[4 + len] = (uint8_t)((sum - 1) & 0xFF);

    return 5 + len;
}

size_t build_hiworld_media_state(const vehicle_cdc_t *cdc, uint8_t *out_buf, size_t max_out) {
    if (!cdc || !out_buf || max_out < 16) return 0;

    uint8_t len = 0x0B; /* 11 payload bytes */
    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = len;
    out_buf[3] = HIWORLD_CMD_CAR_MEDIA_STATE; /* 0x97 */

    out_buf[4]  = cdc->active_disc & 0x0F;
    out_buf[5]  = cdc->discs_loaded_mask & 0x3F;
    out_buf[6]  = cdc->disc_format & 0x3F;
    /* Track Number uint16_le (Data[3..4] in spec, out_buf[7..8]) */
    out_buf[7]  = (uint8_t)(cdc->track_num & 0xFF);
    out_buf[8]  = (uint8_t)((cdc->track_num >> 8) & 0xFF);
    out_buf[9]  = cdc->elapsed_min;
    out_buf[10] = cdc->elapsed_sec;
    out_buf[11] = cdc->play_modes;
    out_buf[12] = cdc->play_status;
    /* Total Tracks uint16_le (Data[9..10] in spec, out_buf[13..14]) */
    out_buf[13] = (uint8_t)(cdc->total_tracks & 0xFF);
    out_buf[14] = (uint8_t)((cdc->total_tracks >> 8) & 0xFF);

    uint16_t sum = len + HIWORLD_CMD_CAR_MEDIA_STATE;
    for (size_t i = 4; i <= 14; i++) {
        sum += out_buf[i];
    }
    out_buf[15] = (uint8_t)((sum - 1) & 0xFF);

    return 16;
}

size_t build_hiworld_radio_preset_freqs(const vehicle_radio_t *radio, uint8_t *out_buf, size_t max_out) {
    if (!radio || !out_buf || max_out < 18) return 0;

    uint8_t len = 13; /* 1 byte band + 6 * 2 bytes freq uint16_be */
    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = len;
    out_buf[3] = HIWORLD_CMD_CAR_RADIO_PRESET_FREQ; /* 0x85 */

    out_buf[4] = radio->band;

    for (size_t i = 0; i < 6; i++) {
        uint16_t freq = radio->preset_freqs[i];
        out_buf[5 + (i * 2)]     = (uint8_t)((freq >> 8) & 0xFF);
        out_buf[5 + (i * 2) + 1] = (uint8_t)(freq & 0xFF);
    }

    uint16_t sum = len + HIWORLD_CMD_CAR_RADIO_PRESET_FREQ;
    for (size_t i = 4; i < (size_t)(4 + len); i++) {
        sum += out_buf[i];
    }
    out_buf[4 + len] = (uint8_t)((sum - 1) & 0xFF);

    return 5 + len;
}

size_t build_hiworld_radio_preset_names(const vehicle_radio_t *radio, uint8_t *out_buf, size_t max_out) {
    if (!radio || !out_buf || max_out < 54) return 0;

    uint8_t len = 49; /* 1 byte band + 6 * 8 bytes names */
    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = len;
    out_buf[3] = HIWORLD_CMD_CAR_RADIO_PRESET_FREQ; /* 0x85 */

    out_buf[4] = radio->band;

    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < 8; j++) {
            char c = radio->preset_names[i][j];
            out_buf[5 + (i * 8) + j] = (c != '\0') ? (uint8_t)c : (uint8_t)' ';
        }
    }

    uint16_t sum = len + HIWORLD_CMD_CAR_RADIO_PRESET_FREQ;
    for (size_t i = 4; i < (size_t)(4 + len); i++) {
        sum += out_buf[i];
    }
    out_buf[4 + len] = (uint8_t)((sum - 1) & 0xFF);

    return 5 + len;
}

size_t build_hiworld_tpms_numeric(const vehicle_tpms_t *tpms, uint8_t *out, size_t max_len) {
    if (!tpms || !out || max_len < 11) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x06;                     /* Length: 6 Payload bytes */
    out[3] = HIWORLD_CMD_TPMS_NUMERIC; /* Cmd 0x66 */
    out[4] = 0x01;                     /* Mode: Live */
    out[5] = tpms->pressure_bar_deci[0];
    out[6] = tpms->pressure_bar_deci[1];
    out[7] = tpms->pressure_bar_deci[2];
    out[8] = tpms->pressure_bar_deci[3];
    out[9] = 0x00;                     /* Unit: Bar */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[10] = (uint8_t)((sum - 1) & 0xFF);
    return 11;
}

size_t build_hiworld_tpms_discrete(const vehicle_tpms_t *tpms, uint8_t *out, size_t max_len) {
    if (!tpms || !out || max_len < 9) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x04;                      /* Length: 4 Payload bytes */
    out[3] = HIWORLD_CMD_TPMS_DISCRETE; /* Cmd 0x18 */
    out[4] = tpms->alarm_state[0];
    out[5] = tpms->alarm_state[1];
    out[6] = tpms->alarm_state[2];
    out[7] = tpms->alarm_state[3];

    uint8_t sum = 0;
    for (size_t i = 2; i <= 7; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[8] = (uint8_t)((sum - 1) & 0xFF);
    return 9;
}

size_t build_hiworld_alert_single(const vehicle_alert_item_t *alert, uint8_t *out, size_t max_len) {
    if (!alert || !out || max_len < 10) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x05; /* Len = 5 */
    out[3] = HIWORLD_CMD_EXTENDED_ALERT; /* 0xEA */
    out[4] = (uint8_t)(alert->can_alarm_id >> 8);
    out[5] = (uint8_t)(alert->can_alarm_id & 0xFF);
    out[6] = alert->is_active ? (uint8_t)(0xC0 | ((alert->severity & 0x03) << 4) | (alert->chime_id & 0x0F)) : 0x00;
    out[7] = alert->door_mask;
    out[8] = alert->param_detail;
    
    uint16_t sum = out[2] + out[3] + out[4] + out[5] + out[6] + out[7] + out[8];
    out[9] = (uint8_t)((sum - 1) & 0xFF);
    return 10;
}

size_t build_hiworld_alerts_summary(const vehicle_alert_item_t *items, uint8_t count, uint8_t *out, size_t max_len) {
    if (!out || max_len < 70) {
        return 0;
    }
    if (count > CANBOX_MAX_ACTIVE_ALERTS) {
        count = CANBOX_MAX_ACTIVE_ALERTS;
    }
    
    uint8_t payload_len = (uint8_t)(4 + (count * 5));
    memset(out, 0, (size_t)(payload_len + 5));
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = payload_len;
    out[3] = HIWORLD_CMD_EXTENDED_ALERT; /* 0xEA */
    out[4] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = count;
    
    for (uint8_t i = 0; i < count; i++) {
        uint8_t offset = 8 + (i * 5);
        out[offset] = (uint8_t)(items[i].can_alarm_id >> 8);
        out[offset + 1] = (uint8_t)(items[i].can_alarm_id & 0xFF);
        out[offset + 2] = (uint8_t)(0xC0 | ((items[i].severity & 0x03) << 4) | (items[i].chime_id & 0x0F));
        out[offset + 3] = items[i].door_mask;
        out[offset + 4] = items[i].param_detail;
    }
    
    uint16_t sum = (uint16_t)(out[2] + out[3]);
    for (uint8_t i = 0; i < payload_len; i++) {
        sum = (uint16_t)(sum + out[4 + i]);
    }
    out[4 + payload_len] = (uint8_t)((sum - 1) & 0xFF);
    return (size_t)(5 + payload_len);
}

size_t build_hiworld_central_state1(const vehicle_config_t *cfg, uint8_t *out, size_t max_len) {
    if (!cfg || !out || max_len < 7) return 0;

    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x02; /* Length: 2 payload bytes */
    out[3] = 0x76; /* Cmd ID: CentralState1 */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (cfg->parking_radar_enabled) b0 |= 0x40;
    if (cfg->auto_lighting_enabled) b0 |= 0x10; /* Automatic headlights (0x260 B2.4) -> 0x50 with radar */
    if (cfg->ambient_lighting_level > 0) {
        b0 |= 0x08; /* Ambient light active */
        b0 |= ((cfg->ambient_lighting_level >> 1) & 0x07);
    }
    out[4] = b0;

    /* Byte 1 */
    uint8_t b1 = 0;
    if (cfg->rear_wiper_reverse)    b1 |= 0x80;
    if (cfg->auto_door_lock)        b1 |= 0x20;
    if (cfg->selective_boot_unlock) b1 |= 0x08;
    if (cfg->drl_enabled)           b1 |= 0x04;
    if (cfg->follow_me_home) {
        if (cfg->follow_me_home_sec >= 60)      b1 |= 0x03;
        else if (cfg->follow_me_home_sec >= 30) b1 |= 0x02;
        else                                    b1 |= 0x01;
    }
    out[5] = b1;

    /* Checksum: ((len + cmd + sum) - 1) & 0xFF */
    uint8_t sum = (uint8_t)(out[2] + out[3] + out[4] + out[5]);
    out[6] = (uint8_t)((sum - 1) & 0xFF);

    return 7;
}

size_t build_hiworld_central_state2(const vehicle_config_t *cfg, const vehicle_state_t *st, uint8_t *out, size_t max_len) {
    if (!cfg || !out || max_len < 13) return 0;

    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x08; /* Length: 8 payload bytes (matches OEM PA12.20) */
    out[3] = 0x79; /* Cmd ID: CentralState2 */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (cfg->motorized_tailgate)     b0 |= 0x80;
    if (cfg->directional_headlights) b0 |= 0x40;
    out[4] = b0;

    /* Byte 1: Drive profile */
    out[5] = (cfg->active_profile_id > 1) ? 0x20 : 0x00;

    /* Byte 2: Cluster dimmer (0x036 B3[3:0] << 2, e.g. 0x0F << 2 = 0x3C) */
    out[6] = (uint8_t)((cfg->cluster_dimmer & 0x0F) << 2);

    /* Byte 3: Door / Window / ESP status (matches PA12.20 0x10) */
    out[7] = 0x10;

    /* Bytes 4 & 5: Unused ADAS */
    out[8] = 0x00;
    out[9] = 0x00;

    /* Byte 6: Climate Dual/Mono Sync (0x02=Mono, 0x00=Dual) */
    out[10] = (st && !st->climate.dual_mode) ? 0x02 : 0x00;

    /* Byte 7: Reserved */
    out[11] = 0x00;

    /* Checksum: ((len + cmd + sum) - 1) & 0xFF */
    uint8_t sum = 0;
    for (size_t i = 2; i < 12; i++) sum = (uint8_t)(sum + out[i]);
    out[12] = (uint8_t)((sum - 1) & 0xFF);

    return 13;
}

