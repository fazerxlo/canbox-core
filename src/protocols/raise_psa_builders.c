#include "protocols/raise_psa_builders.h"
#include "proto_raise.h"
#include <string.h>

size_t build_raise_stalk_key(uint8_t key_id, uint8_t state, uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < 6) {
        return 0;
    }

    out_buf[0] = RAISE_SYNC_BYTE;
    out_buf[1] = RAISE_CMD_STALK_KEY;
    out_buf[2] = 0x02;
    out_buf[3] = key_id;
    out_buf[4] = state;
    out_buf[5] = (uint8_t)((out_buf[1] + out_buf[2] + out_buf[3] + out_buf[4]) ^ 0xFF);
    return 6;
}

size_t build_raise_hvac_packet(const hvac_state_t *st, uint8_t *out_buf, size_t max_len) {
    if (!st || !out_buf || max_len < 11) {
        return 0;
    }

    out_buf[0] = RAISE_SYNC_BYTE;
    out_buf[1] = RAISE_CMD_HVAC;
    out_buf[2] = 0x07; /* Length */

    uint8_t d0 = 0;
    if (st->power)         d0 |= 0x80;
    if (st->ac_compressor) d0 |= 0x40;
    if (st->recirculation) d0 |= 0x20;
    if (st->aqs_auto)      d0 |= 0x10;
    if (st->auto_mode)     d0 |= 0x08;
    if (st->dual_mode)     d0 |= 0x04;
    if (st->rear_defrost)  d0 |= 0x01;
    out_buf[3] = d0;

    uint8_t d1 = (uint8_t)(st->fan_speed & 0x0F);
    if (st->driver_wind_up)   d1 |= 0x80;
    if (st->driver_wind_face) d1 |= 0x40;
    if (st->driver_wind_down) d1 |= 0x20;
    out_buf[4] = d1;

    out_buf[5] = st->driver_temp_raw;
    out_buf[6] = st->pass_temp_raw;

    uint8_t d4 = 0;
    if (st->front_max_defrost) d4 |= 0x80;
    if (st->ac_max)            d4 |= 0x08;
    out_buf[7] = d4;

    out_buf[8] = 0x00; /* Rear Power / Flags */

    uint8_t d6 = 0;
    if (st->pass_wind_up)   d6 |= 0x80;
    if (st->pass_wind_face) d6 |= 0x40;
    if (st->pass_wind_down) d6 |= 0x20;
    out_buf[9] = d6;

    /* Checksum: (Sum ^ 0xFF) & 0xFF */
    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out_buf[i];
    }
    out_buf[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

size_t build_raise_rear_radar(uint8_t rl, uint8_t rc, uint8_t rr,
                              uint8_t fl, uint8_t fc, uint8_t fr,
                              uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < 11) return 0;
    out_buf[0] = RAISE_SYNC_BYTE;
    out_buf[1] = RAISE_CMD_RADAR_REAR;
    out_buf[2] = 0x07; /* Length */
    out_buf[3] = 0x00; /* Flag */
    out_buf[4] = rl;
    out_buf[5] = rc;
    out_buf[6] = rr;
    out_buf[7] = fl;
    out_buf[8] = fc;
    out_buf[9] = fr;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out_buf[i];
    }
    out_buf[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

size_t build_raise_front_radar(uint8_t fl, uint8_t fc, uint8_t fr,
                               uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < 8) return 0;
    out_buf[0] = RAISE_SYNC_BYTE;
    out_buf[1] = RAISE_CMD_RADAR_FRONT;
    out_buf[2] = 0x04; /* Length */
    out_buf[3] = 0x00; /* Flag */
    out_buf[4] = fl;
    out_buf[5] = fc;
    out_buf[6] = fr;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 6; i++) {
        sum += out_buf[i];
    }
    out_buf[7] = (uint8_t)(sum ^ 0xFF);
    return 8;
}

size_t build_raise_instant_fuel(uint16_t instant_fuel_dkl, uint16_t dte_range_km,
                                uint16_t dest_dist_km, uint8_t *out, size_t max_len) {
    if (!out || max_len < 10) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_INSTANT_FUEL;
    out[2] = 0x06;
    out[3] = (uint8_t)(instant_fuel_dkl >> 8);
    out[4] = (uint8_t)(instant_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(dte_range_km >> 8);
    out[6] = (uint8_t)(dte_range_km & 0xFF);
    out[7] = (uint8_t)(dest_dist_km >> 8);
    out[8] = (uint8_t)(dest_dist_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_trip1(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out) {
    if (!out) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_TRIP1;
    out[2] = 0x06;
    out[3] = (uint8_t)(avg_fuel_dkl >> 8);
    out[4] = (uint8_t)(avg_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(avg_spd_kmh >> 8);
    out[6] = (uint8_t)(avg_spd_kmh & 0xFF);
    out[7] = (uint8_t)(dist_dkm >> 8);
    out[8] = (uint8_t)(dist_dkm & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_trip2(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out) {
    if (!out) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_TRIP2;
    out[2] = 0x06;
    out[3] = (uint8_t)(avg_fuel_dkl >> 8);
    out[4] = (uint8_t)(avg_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(avg_spd_kmh >> 8);
    out[6] = (uint8_t)(avg_spd_kmh & 0xFF);
    out[7] = (uint8_t)(dist_dkm >> 8);
    out[8] = (uint8_t)(dist_dkm & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_outside_temp(uint8_t temp_raw, uint8_t *out, size_t max_len) {
    if (!out || max_len < 5) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_OUTSIDE_TEMP;
    out[2] = 0x01;
    out[3] = temp_raw;
    out[4] = (uint8_t)((out[1] + out[2] + out[3]) ^ 0xFF);
    return 5;
}

size_t build_raise_reverse_state(bool reverse_active, uint8_t *out, size_t max_len) {
    if (!out || max_len < 5) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_REVERSE;
    out[2] = 0x01;
    out[3] = reverse_active ? 0x80 : 0x00;
    out[4] = (uint8_t)((out[1] + out[2] + out[3]) ^ 0xFF);
    return 5;
}

size_t build_raise_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len) {
    if (!doors || !out || max_len < 12) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_DOORS_BODY;
    out[2] = 0x08; /* Length */

    uint8_t d0 = 0;
    if (doors->driver_door)     d0 |= 0x80;
    if (doors->pass_door)       d0 |= 0x40;
    if (doors->rear_left_door)  d0 |= 0x20;
    if (doors->rear_right_door) d0 |= 0x10;
    if (doors->trunk)           d0 |= 0x08;
    if (doors->hood)            d0 |= 0x04;
    out[3] = d0;

    out[4] = doors->handbrake ? 0x01 : 0x00;
    out[5] = 0x00;
    out[6] = 0x00;
    out[7] = 0x00;
    out[8] = 0x00;
    out[9] = 0x00;
    out[10] = 0x00;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_raise_steering_angle(int16_t angle_deci_deg, uint8_t *out) {
    if (!out) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_STEERING_ANGLE;
    out[2] = 0x02;
    out[3] = (uint8_t)(angle_deci_deg & 0xFF);        /* Little Endian */
    out[4] = (uint8_t)((angle_deci_deg >> 8) & 0xFF);
    out[5] = (uint8_t)((out[1] + out[2] + out[3] + out[4]) ^ 0xFF);
    return 6;
}

size_t build_raise_amplifier(const jbl_amplifier_state_t *st, uint8_t *out, size_t max_len) {
    if (!st || !out || max_len < 12) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_AMPLIFIER;
    out[2] = 0x08;
    out[3] = 0x00; /* Fixed padding */
    out[4] = (uint8_t)(st->bass & 0x0F);
    out[5] = (uint8_t)(st->treble & 0x0F);
    out[6] = (uint8_t)(st->balance & 0x0F);
    out[7] = (uint8_t)(st->fader & 0x0F);
    out[8] = (uint8_t)(st->eq_preset & 0x07);
    out[9] = (uint8_t)((st->loudness ? 0x10 : 0x00) | (st->speed_vol_comp & 0x0F));
    out[10] = (uint8_t)(st->master_volume & 0x1F);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_raise_cd_changer(const cd_changer_state_t *st, uint8_t *out, size_t max_len) {
    if (!st || !out || max_len < 11) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_CD_CHANGER;
    out[2] = 0x07;
    out[3] = 0x02; /* CD Player Mode */
    out[4] = st->disc_slot;
    out[5] = st->track_num;
    out[6] = st->total_tracks;
    out[7] = st->elapsed_min;
    out[8] = st->elapsed_sec;
    out[9] = st->play_flags;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out[i];
    }
    out[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

size_t build_raise_rds_name(const char *name, uint8_t *out, size_t max_len) {
    if (!out || max_len < 12) return 0;
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_RDS_NAME;
    out[2] = 0x08;

    size_t nlen = (name != NULL) ? strlen(name) : 0;
    for (size_t i = 0; i < 8; i++) {
        out[3 + i] = (i < nlen && name[i] != '\0') ? (uint8_t)name[i] : (uint8_t)' ';
    }

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_raise_tpms_numeric(const canbox_tpms_state_t *tpms, uint8_t *out) {
    if (!tpms || !out) {
        return 0;
    }
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_TPMS_NUMERIC;
    out[2] = 0x06;
    out[3] = 0x01; /* Real-time mode */
    out[4] = (uint8_t)tpms->pressure_dbar[0];
    out[5] = (uint8_t)tpms->pressure_dbar[1];
    out[6] = (uint8_t)tpms->pressure_dbar[2];
    out[7] = (uint8_t)tpms->pressure_dbar[3];
    out[8] = 0x00; /* Unit: Bar * 0.1 */

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_tpms_temp_alarms(const canbox_tpms_state_t *tpms, uint8_t *out) {
    if (!tpms || !out) {
        return 0;
    }
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_TPMS_TEMPS;
    out[2] = 0x08;
    for (int i = 0; i < 4; i++) {
        int16_t t = tpms->temperature_c[i] + 40;
        out[3 + i] = (t < 0) ? 0 : ((t > 255) ? 255 : (uint8_t)t);
    }
    for (int i = 0; i < 4; i++) {
        out[7 + i] = tpms->alarm_code[i];
    }
    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_raise_tpms_discrete(const uint8_t alarms[4], uint8_t *out) {
    if (!alarms || !out) {
        return 0;
    }
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_TPMS_DISCRETE;
    out[2] = 0x04;
    out[3] = alarms[0];
    out[4] = alarms[1];
    out[5] = alarms[2];
    out[6] = alarms[3];

    uint8_t sum = 0;
    for (size_t i = 1; i <= 6; i++) {
        sum += out[i];
    }
    out[7] = (uint8_t)(sum ^ 0xFF);
    return 8;
}

size_t build_raise_start_stop(bool is_active, uint32_t stop_time_sec, uint8_t *out) {
    if (!out) {
        return 0;
    }
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_START_STOP;
    out[2] = 0x05;
    out[3] = is_active ? 0x01 : 0x00;
    out[4] = (uint8_t)((stop_time_sec >> 24) & 0xFF);
    out[5] = (uint8_t)((stop_time_sec >> 16) & 0xFF);
    out[6] = (uint8_t)((stop_time_sec >> 8) & 0xFF);
    out[7] = (uint8_t)(stop_time_sec & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 7; i++) {
        sum += out[i];
    }
    out[8] = (uint8_t)(sum ^ 0xFF);
    return 9;
}

size_t build_raise_cruise_memory(bool active, uint8_t target_spd, const uint8_t presets[5], uint8_t *out) {
    if (!out) {
        return 0;
    }
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_CRUISE_MEMORY;
    out[2] = 0x07;
    out[3] = active ? 0x01 : 0x00;
    out[4] = target_spd;
    if (presets) {
        memcpy(&out[5], presets, 5);
    } else {
        memset(&out[5], 0, 5);
    }

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out[i];
    }
    out[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

size_t build_raise_adas(const canbox_adas_state_t *adas, uint8_t *out) {
    if (!adas || !out) {
        return 0;
    }
    out[0] = RAISE_SYNC_BYTE;
    out[1] = RAISE_CMD_ADAS;
    out[2] = 0x06;
    out[3] = adas->blind_spot_warning ? 0x80 : 0x00;
    out[4] = adas->fatigue_coffee_cup ? 0x80 : 0x00;
    out[5] = adas->lane_departure_state;
    out[6] = adas->speed_limit_tsr;
    out[7] = adas->esp_active ? 0x01 : 0x00;
    out[8] = adas->aeb_risk_level;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

