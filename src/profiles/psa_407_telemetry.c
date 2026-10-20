#include "profiles/peugeot_407.h"
#include "hal/hal_gpio.h"
#include <string.h>

static inline uint16_t read_be16(const uint8_t *d) {
    return ((uint16_t)d[0] << 8) | (uint16_t)d[1];
}

/* --------------------------------------------------------------------------
 * 1.4 Trip Computer & Engine Telemetry
 * -------------------------------------------------------------------------- */
void psa_decode_trip_0x165(const uint8_t *data, uint8_t dlc,
                           uint16_t *instant_fuel, uint16_t *range_dte,
                           uint16_t *dest_dist, uint8_t *ext_temp_raw) {
    if (!data) return;
    if (dlc >= 2 && instant_fuel) *instant_fuel = read_be16(&data[0]);
    if (dlc >= 4 && range_dte)    *range_dte = read_be16(&data[2]);
    if (dlc >= 6 && dest_dist)    *dest_dist = read_be16(&data[4]);
    if (dlc >= 7 && ext_temp_raw) *ext_temp_raw = data[6];
}

void psa_decode_trip1_0x1a5(const uint8_t *data, uint8_t dlc,
                            uint16_t *dist, uint16_t *avg_fuel, uint16_t *avg_speed) {
    if (!data) return;
    if (dlc >= 2 && dist)      *dist = read_be16(&data[0]);
    if (dlc >= 4 && avg_fuel)  *avg_fuel = read_be16(&data[2]);
    if (dlc >= 5 && avg_speed) *avg_speed = (uint16_t)data[4];
}

void psa_decode_trip2_0x2a5(const uint8_t *data, uint8_t dlc,
                            uint16_t *dist, uint16_t *avg_fuel, uint16_t *avg_speed) {
    if (!data) return;
    if (dlc >= 2 && dist)      *dist = read_be16(&data[0]);
    if (dlc >= 4 && avg_fuel)  *avg_fuel = read_be16(&data[2]);
    if (dlc >= 5 && avg_speed) *avg_speed = (uint16_t)data[4];
}

void psa_decode_reverse_0x036(const uint8_t *data, uint8_t dlc, bool *reverse_active) {
    if (!data || !reverse_active || dlc < 2) return;
    *reverse_active = (data[1] & 0x80) != 0;
}

void psa_decode_reverse_0x0f6(const uint8_t *data, uint8_t dlc, bool *reverse_active) {
    if (!data || !reverse_active || dlc < 8) return;
    *reverse_active = (data[7] & 0x80) != 0;
}

void psa_reverse_set_hardware_trigger(bool reverse_active) {
    hal_gpio_write(GPIO_PIN_REVERSE_OUT, reverse_active);
}

bool psa_trip_process_can_0x221(vehicle_trip_t *trip, const uint8_t *data, uint8_t dlc) {
    if (!trip || !data || dlc < 7) return false;

    /* Byte 0 Bit 7: Fuel info hidden/invalid.
     * Also guard against uninitialized/BSI status frames where all data is 0xFF.
     */
    if ((data[0] & 0x80) != 0) return false;
    uint16_t fuel = read_be16(&data[1]);
    uint16_t range = read_be16(&data[3]);
    if (fuel == 0xFFFF && range == 0xFFFF) return false;

    trip->instant_fuel_deci = fuel;
    trip->range_km          = range;
    trip->dest_dist_km      = read_be16(&data[5]);
    trip->instant_valid     = true;
    return true;
}

bool psa_trip_process_can_0x2a1(vehicle_trip_t *trip, const uint8_t *data, uint8_t dlc) {
    if (!trip || !data || dlc < 5) return false;

    trip->trip1_distance_km = read_be16(&data[1]);
    trip->trip1_avg_fuel    = read_be16(&data[3]);
    trip->trip1_avg_speed   = data[0];
    trip->trip1_valid       = true;
    return true;
}

bool psa_trip_process_can_0x261(vehicle_trip_t *trip, const uint8_t *data, uint8_t dlc) {
    if (!trip || !data || dlc < 5) return false;

    trip->trip2_distance_km = read_be16(&data[1]);
    trip->trip2_avg_fuel    = read_be16(&data[3]);
    trip->trip2_avg_speed   = data[0];
    trip->trip2_valid       = true;
    return true;
}

void psa_engine_process_can_0x0b6(vehicle_state_t *state, const uint8_t *data, uint8_t dlc) {
    if (!state || !data || dlc < 4) return;
    uint16_t raw_rpm = read_be16(&data[0]);
    uint16_t raw_spd = read_be16(&data[2]);

    state->rpm = (raw_rpm == 0xFFFF) ? 0 : (raw_rpm >> 3);
    state->speed_kmh = (raw_spd == 0xFFFF) ? 0 : (raw_spd / 100);
}

void psa_engine_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    if (frame->id == 0x0B6) {
        psa_engine_process_can_0x0b6(state, frame->data, frame->dlc);
    }
}

void psa_bsi_process_can_0x0f6(vehicle_state_t *state, const uint8_t *data, uint8_t dlc) {
    if (!state || !data || dlc < 8) return;
    state->reverse_gear = (data[7] & 0x80) != 0;
    if (data[5] != 0xFF) {
        state->climate.outdoor_temp_raw = data[5];
    }
}


void psa_trip_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    switch (frame->id) {
        case PSA_CAN_ID_TRIP_INSTANT: /* 0x221 */
            psa_stalk_process_tip_0x221(&state->wheel, frame->data, frame->dlc);
            if (psa_trip_process_can_0x221(&state->trip, frame->data, frame->dlc)) {
                state->dirty.trip_page = 1;
            }
            break;
        case PSA_CAN_ID_TRIP1_ODB: /* 0x2A1 */
            if (psa_trip_process_can_0x2a1(&state->trip, frame->data, frame->dlc)) {
                state->dirty.trip_page = 2;
            }
            break;
        case PSA_CAN_ID_TRIP2_ODB: /* 0x261 */
            if (psa_trip_process_can_0x261(&state->trip, frame->data, frame->dlc)) {
                state->dirty.trip_page = 3;
            }
            break;
        default:
            break;
    }
}

bool build_psa_trip_reset_frame(uint8_t trip_index, can_frame_t *out_frame) {
    if (!out_frame || (trip_index != 1 && trip_index != 2)) {
        return false;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->id          = PSA_CAN_ID_TRIP_INSTANT; /* 0x221 (MSG_DEMANDES_EMF) */
    out_frame->dlc         = 8;
    out_frame->is_extended = false;
    out_frame->is_remote   = false;

    if (trip_index == 1) {
        out_frame->data[0] = 0x80; /* Bit 7: Trip 1 Reset */
    } else {
        out_frame->data[0] = 0x40; /* Bit 6: Trip 2 Reset */
    }

    return true;
}

hal_status_t psa_trip_send_reset(uint8_t trip_index) {
    can_frame_t frame;
    if (!build_psa_trip_reset_frame(trip_index, &frame)) {
        return HAL_STATUS_ERROR;
    }

    return hal_can_send(&frame);
}


/* --------------------------------------------------------------------------
 * 2.1 Direct TPMS Numeric Readings & Fault Classification
 * -------------------------------------------------------------------------- */
void psa_tpms_init(vehicle_tpms_t *tpms) {
    if (!tpms) return;
    memset(tpms, 0, sizeof(vehicle_tpms_t));
    tpms->pressure_bar_deci[0] = 24; /* 2.4 Bar default */
    tpms->pressure_bar_deci[1] = 24;
    tpms->pressure_bar_deci[2] = 22; /* 2.2 Bar default */
    tpms->pressure_bar_deci[3] = 22;
}

void psa_tpms_process_can_0x361(vehicle_tpms_t *tpms, const uint8_t *data, uint8_t dlc) {
    if (!tpms || !data || dlc < 8) return;

    uint16_t fl_raw = read_be16(&data[0]);
    uint16_t fr_raw = read_be16(&data[2]);
    uint16_t rr_raw = read_be16(&data[4]);
    uint16_t rl_raw = read_be16(&data[6]);

    tpms->alarm_state[0] = (uint8_t)((fl_raw >> 14) & 0x03);
    tpms->alarm_state[1] = (uint8_t)((fr_raw >> 14) & 0x03);
    tpms->alarm_state[2] = (uint8_t)((rl_raw >> 14) & 0x03);
    tpms->alarm_state[3] = (uint8_t)((rr_raw >> 14) & 0x03);

    if (tpms->alarm_state[0] != 3 && (fl_raw & 0x3FFF) != 0x3FFF) {
        tpms->pressure_bar_deci[0] = (uint8_t)(fl_raw & 0x3FFF);
    } else {
        tpms->pressure_bar_deci[0] = 0;
    }
    if (tpms->alarm_state[1] != 3 && (fr_raw & 0x3FFF) != 0x3FFF) {
        tpms->pressure_bar_deci[1] = (uint8_t)(fr_raw & 0x3FFF);
    } else {
        tpms->pressure_bar_deci[1] = 0;
    }
    if (tpms->alarm_state[2] != 3 && (rl_raw & 0x3FFF) != 0x3FFF) {
        tpms->pressure_bar_deci[2] = (uint8_t)(rl_raw & 0x3FFF);
    } else {
        tpms->pressure_bar_deci[2] = 0;
    }
    if (tpms->alarm_state[3] != 3 && (rr_raw & 0x3FFF) != 0x3FFF) {
        tpms->pressure_bar_deci[3] = (uint8_t)(rr_raw & 0x3FFF);
    } else {
        tpms->pressure_bar_deci[3] = 0;
    }

    tpms->valid = true;
}

void psa_tpms_process_can_0x3a1(vehicle_tpms_t *tpms, const uint8_t *data, uint8_t dlc) {
    if (!tpms || !data || dlc < 4) return;

    tpms->pressure_bar_deci[0] = (uint8_t)((data[0] * 5 + 5) / 10);
    tpms->pressure_bar_deci[1] = (uint8_t)((data[1] * 5 + 5) / 10);
    tpms->pressure_bar_deci[2] = (uint8_t)((data[3] * 5 + 5) / 10); /* RL */
    tpms->pressure_bar_deci[3] = (uint8_t)((data[2] * 5 + 5) / 10); /* RR */
    tpms->valid = true;
}

void psa_tpms_process_can_0x1e1(vehicle_tpms_t *tpms, const uint8_t *data, uint8_t dlc) {
    if (!tpms || !data || dlc < 4) return;

    /* PSA 0x1E1 bits 5..3 carry RT4 tyre state enum:
       0=OK, 1=LOW, 2=FLAT, 3=NO_DATA, 4=BATTERY_LOW */
    uint8_t fl_st = (uint8_t)((data[0] >> 3) & 0x07);
    uint8_t fr_st = (uint8_t)((data[1] >> 3) & 0x07);
    uint8_t rr_st = (uint8_t)((data[2] >> 3) & 0x07);
    uint8_t rl_st = (uint8_t)((data[3] >> 3) & 0x07);

    /* Map PSA enum to Hiworld discrete alarm:
       0=Normal, 1=Low, 2=Puncture, 3=Fault/Offline */
    tpms->alarm_state[0] = (fl_st >= 3) ? 3 : fl_st;
    tpms->alarm_state[1] = (fr_st >= 3) ? 3 : fr_st;
    tpms->alarm_state[2] = (rl_st >= 3) ? 3 : rl_st;
    tpms->alarm_state[3] = (rr_st >= 3) ? 3 : rr_st;
    tpms->valid = true;
}

void psa_tpms_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    switch (frame->id) {
        case PSA_CAN_ID_TPMS_DIRECT_361:
            psa_tpms_process_can_0x361(&state->tpms, frame->data, frame->dlc);
            break;
        case PSA_CAN_ID_TPMS_PRESSURES_3A1:
            psa_tpms_process_can_0x3a1(&state->tpms, frame->data, frame->dlc);
            break;
        case PSA_CAN_ID_TPMS_STATUS_1E1:
            psa_tpms_process_can_0x1e1(&state->tpms, frame->data, frame->dlc);
            break;
        default:
            break;
    }
}

/* --------------------------------------------------------------------------
 * 2.2 Stop & Start (S&S) Telemetry & Timer
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * 2.3 Cruise Control & Speed Memory Presets
 * -------------------------------------------------------------------------- */
void psa_decode_cruise_0x1a8(const uint8_t *data, uint8_t dlc, bool *active, uint8_t *set_speed_kmh, uint32_t *partial_odo_m) {
    if (!data || dlc < 3) {
        return;
    }
    if (active) {
        uint8_t status = (data[0] >> 3) & 0x07;
        *active = (status == 1 || status == 2);
    }
    if (set_speed_kmh) {
        uint16_t spd_raw = read_be16(&data[1]);
        if (spd_raw == 0xFFFF) {
            *set_speed_kmh = 0;
        } else {
            *set_speed_kmh = (uint8_t)(spd_raw / 100);
        }
    }
    if (partial_odo_m && dlc >= 8) {
        uint32_t odo_raw = ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | (uint32_t)data[7];
        *partial_odo_m = (odo_raw == 0xFFFFFF) ? 0 : odo_raw;
    }
}

