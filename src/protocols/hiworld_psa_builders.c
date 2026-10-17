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
    if (!trip || !out || max_len < 11) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x06;               /* Length: 6 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P1; /* Cmd 0x14 */
    out[4] = (uint8_t)(trip->trip1_avg_fuel >> 8);
    out[5] = (uint8_t)(trip->trip1_avg_fuel & 0xFF);
    out[6] = 0x00;               /* Reserved */
    out[7] = trip->trip1_avg_speed;
    out[8] = (uint8_t)(trip->trip1_distance_km >> 8);
    out[9] = (uint8_t)(trip->trip1_distance_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[10] = (uint8_t)((sum - 1) & 0xFF);
    return 11;
}

size_t build_hiworld_trip2(const vehicle_trip_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 11) {
        return 0;
    }
    out[0] = HIWORLD_SYNC_1;
    out[1] = HIWORLD_SYNC_2;
    out[2] = 0x06;               /* Length: 6 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P2; /* Cmd 0x15 */
    out[4] = (uint8_t)(trip->trip2_avg_fuel >> 8);
    out[5] = (uint8_t)(trip->trip2_avg_fuel & 0xFF);
    out[6] = 0x00;               /* Reserved */
    out[7] = trip->trip2_avg_speed;
    out[8] = (uint8_t)(trip->trip2_distance_km >> 8);
    out[9] = (uint8_t)(trip->trip2_distance_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[10] = (uint8_t)((sum - 1) & 0xFF);
    return 11;
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

