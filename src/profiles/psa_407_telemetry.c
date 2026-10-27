#include "profiles/peugeot_407.h"
#include "hal/hal_gpio.h"
#include <string.h>

static inline uint16_t read_be16(const uint8_t *d) {
    return ((uint16_t)d[0] << 8) | (uint16_t)d[1];
}

/* --------------------------------------------------------------------------
 * 1.4 Trip Computer & Engine Telemetry
 *
 * NOTE ON LEGACY FUNCTIONS (psa_decode_trip_0x165 / 0x1a5 / 0x2a5):
 * These legacy functions are preserved for backwards compatibility with
 * external test harnesses. In genuine PSA CAN2004 architecture:
 *   - CAN 0x165 is MSG_ETAT_AUTORADIO (radio status)
 *   - CAN 0x1A5 is MSG_VOLUME_RADIO_OR_CD_STATUS (RD4 volume or RT4 CD)
 *   - CAN 0x2A5 is MSG_NOM_STATION (RDS station name)
 * Real PSA trip computer telemetry is decoded below via:
 *   - CAN 0x221 (MSG_INFO_GEN_ODB): psa_trip_process_can_0x221
 *   - CAN 0x2A1 (MSG_INFOS_TRAJET1_ODB): psa_trip_process_can_0x2a1
 *   - CAN 0x261 (MSG_INFOS_TRAJET2_ODB): psa_trip_process_can_0x261
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

    /* Note: 0xFFFF sentinels in fuel/range represent valid PSA stationary/undefined
     * indicators ('--.-') and must NOT be suppressed so the HU displays '--.-'.
     *
     * In PSA CAN2004 (RT4 8.31 OEM ground truth get_trip_data_inst offset 0x012454):
     * - Bytes 1-2: CRUISING_RANGE (km)
     * - Bytes 3-4: INST_CONSUMPTION (0.1 L/100km)
     * - Bytes 5-6: DIST_TO_DEST (km)
     */
    trip->range_km          = read_be16(&data[1]);
    trip->instant_fuel_deci = read_be16(&data[3]);
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


static bool     s_prev_trip_btn           = false;
static uint32_t s_trip_btn_press_ms       = 0;
static bool     s_trip_long_press_handled = false;

void psa_trip_init(void) {
    s_prev_trip_btn           = false;
    s_trip_btn_press_ms       = 0;
    s_trip_long_press_handled = false;
}

void psa_trip_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    switch (frame->id) {
        case PSA_CAN_ID_TRIP_INSTANT: /* 0x221 */
            if (frame->dlc >= 1) {
                bool trip_btn_pressed = (frame->data[0] & 0x08) != 0;
                uint32_t now = hal_get_tick_ms();

                if (trip_btn_pressed && !s_prev_trip_btn) {
                    /* Rising Edge */
                    s_trip_btn_press_ms = now;
                    s_trip_long_press_handled = false;
                } else if (trip_btn_pressed && s_prev_trip_btn) {
                    /* Held State */
                    if (!s_trip_long_press_handled && (now - s_trip_btn_press_ms) >= 2000) {
                        s_trip_long_press_handled = true;
                        if (!psa_car_config_is_mdf_present()) {
                            uint8_t page_mask = psa_car_config_get_trip_page_mask();
                            if (page_mask == PSA_MDF_TRIP_PAGE_TRIP1) {
                                psa_trip_send_reset(1);
                            } else if (page_mask == PSA_MDF_TRIP_PAGE_TRIP2) {
                                psa_trip_send_reset(2);
                            }
                        }
                    }
                } else if (!trip_btn_pressed && s_prev_trip_btn) {
                    /* Falling Edge */
                    if (!s_trip_long_press_handled && !psa_car_config_is_mdf_present()) {
                        psa_car_config_cycle_trip_page();
                        uint8_t page_mask = psa_car_config_get_trip_page_mask();
                        if (page_mask == PSA_MDF_TRIP_PAGE_INSTANT) {
                            state->dirty.trip_page = 1;
                        } else if (page_mask == PSA_MDF_TRIP_PAGE_TRIP1) {
                            state->dirty.trip_page = 2;
                        } else if (page_mask == PSA_MDF_TRIP_PAGE_TRIP2) {
                            state->dirty.trip_page = 3;
                        }
                    }
                }
                s_prev_trip_btn = trip_btn_pressed;
            }
            psa_stalk_process_tip_0x221(&state->wheel, frame->data, frame->dlc);
            if (psa_trip_process_can_0x221(&state->trip, frame->data, frame->dlc)) {
                if (state->dirty.trip_page == 0) {
                    state->dirty.trip_page = 1;
                }
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
    out_frame->id          = PSA_CAN_ID_MDF_TRIP_CMD_167; /* 0x167 */
    out_frame->dlc         = 8;
    out_frame->is_extended = false;
    out_frame->is_remote   = false;

    uint8_t page = (trip_index == 1) ? PSA_MDF_TRIP_PAGE_TRIP1 : PSA_MDF_TRIP_PAGE_TRIP2;
    uint8_t reset_bit = (trip_index == 1) ? 0x80 : 0x40;

    out_frame->data[0] = (uint8_t)(page | reset_bit);
    out_frame->data[1] = 0x06;
    out_frame->data[2] = 0xFF;
    out_frame->data[3] = 0xFF;

    return true;
}

hal_status_t psa_trip_send_reset(uint8_t trip_index) {
    if (psa_car_config_is_mdf_present()) {
        return HAL_STATUS_ERROR;
    }

    can_frame_t frame;
    if (!build_psa_trip_reset_frame(trip_index, &frame)) {
        return HAL_STATUS_ERROR;
    }

    psa_car_config_set_trip_reset(trip_index);
    return hal_can_send(&frame);
}


/* --------------------------------------------------------------------------
 * 2.1 Direct TPMS Numeric Readings & Fault Classification
 * -------------------------------------------------------------------------- */
static bool s_has_direct_pressures = false;

void psa_tpms_init(vehicle_tpms_t *tpms) {
    s_has_direct_pressures = false;
    if (!tpms) return;
    memset(tpms, 0, sizeof(vehicle_tpms_t));
    tpms->pressure_bar_deci[0] = 24; /* FL: 2.4 Bar default */
    tpms->pressure_bar_deci[1] = 24; /* FR: 2.4 Bar default */
    tpms->pressure_bar_deci[2] = 22; /* RL: 2.2 Bar default */
    tpms->pressure_bar_deci[3] = 22; /* RR: 2.2 Bar default */
}

static uint8_t psa_tpms_decode_0x1e1_wheel(uint8_t raw_byte, uint8_t nominal_deci_bar, uint8_t *out_synth_bar) {
    /* RT4 / PSA CAN 0x1E1 Bits 7:3 (5-bit enum, (raw_byte >> 3) & 0x1F):
     * 0x00 (raw 0x00) = Normal / OK
     * 0x01 (raw 0x08) = Under-inflated (Low pressure)
     * 0x02 (raw 0x10) = Punctured (Flat)
     * 0x03 (raw 0x18) = Sensor Missing / Not Detected
     * 0x04 (raw 0x20) = Sensor Battery Low / Hardware Fault
     * 0x08 (raw 0x40) = RT4 Default / Invalid: Unmonitored / Standby
     * 0x10 (raw 0x80) = Sensor Offline / Unequipped (e.g. spare wheel)
     */
    uint8_t state_enum = (uint8_t)((raw_byte >> 3) & 0x1F);
    switch (state_enum) {
        case 0x00:
            if (out_synth_bar) *out_synth_bar = nominal_deci_bar;
            return 0; /* Hiworld: Normal / OK */
        case 0x01:
            if (out_synth_bar) *out_synth_bar = 17; /* 1.7 Bar synthetic low */
            return 1; /* Hiworld: Low Pressure */
        case 0x02:
            if (out_synth_bar) *out_synth_bar = 8;  /* 0.8 Bar synthetic puncture */
            return 2; /* Hiworld: Puncture / Flat */
        case 0x03:
        case 0x04:
        case 0x08: /* RT4 Default / Invalid: Unmonitored (raw 0x40) */
        case 0x10: /* Sensor Offline / Unequipped (raw 0x80) */
        default:
            if (out_synth_bar) *out_synth_bar = 0;  /* 0.0 Bar / Offline */
            return 3; /* Hiworld: Sensor Fault / Offline / Missing */
    }
}

static inline uint8_t psa_tpms_scale_0x3a1_raw(uint8_t raw) {
    if (raw == 0xFF) {
        return 0; /* Invalid / Offline / Default */
    }
    return (uint8_t)((raw * 5 + 5) / 10);
}

void psa_tpms_process_can_0x3a1(vehicle_tpms_t *tpms, const uint8_t *data, uint8_t dlc) {
    if (!tpms || !data || dlc < 4) return;

    s_has_direct_pressures = true;

    tpms->pressure_bar_deci[0] = psa_tpms_scale_0x3a1_raw(data[0]);        /* FL */
    tpms->pressure_bar_deci[1] = psa_tpms_scale_0x3a1_raw(data[1]);        /* FR */
    tpms->pressure_bar_deci[2] = psa_tpms_scale_0x3a1_raw(data[3]);        /* RL (CAN Byte 3) */
    tpms->pressure_bar_deci[3] = psa_tpms_scale_0x3a1_raw(data[2]);        /* RR (CAN Byte 2) */
    tpms->valid = true;
}

void psa_tpms_process_can_0x1e1(vehicle_tpms_t *tpms, const uint8_t *data, uint8_t dlc) {
    if (!tpms || !data || dlc < 4) return;

    /* Guard against standby / unequipped / unmonitored frames before TPMS wakes up
     * (Bit 7 set = unequipped 0x80, or Bit 6 set = unmonitored 0x40, i.e. (data & 0xC0) != 0).
     * Bench frames (40 40 40 40) or unequipped frames (80 80 80 80) must not latch synthetic
     * pressures or assert false normal tire states. */
    if (!tpms->valid &&
        ((data[0] & 0xC0) != 0) && ((data[1] & 0xC0) != 0) &&
        ((data[2] & 0xC0) != 0) && ((data[3] & 0xC0) != 0)) {
        return;
    }

    uint8_t synth_fl, synth_fr, synth_rl, synth_rr;

    /* PSA 0x1E1 Byte order: Byte 0=FL, Byte 1=FR, Byte 2=RR, Byte 3=RL
     * Hiworld array order:  [0]=FL,    [1]=FR,    [2]=RL,    [3]=RR */
    tpms->alarm_state[0] = psa_tpms_decode_0x1e1_wheel(data[0], 24, &synth_fl); /* FL */
    tpms->alarm_state[1] = psa_tpms_decode_0x1e1_wheel(data[1], 24, &synth_fr); /* FR */
    tpms->alarm_state[2] = psa_tpms_decode_0x1e1_wheel(data[3], 22, &synth_rl); /* RL */
    tpms->alarm_state[3] = psa_tpms_decode_0x1e1_wheel(data[2], 22, &synth_rr); /* RR */

    /* If no direct numeric pressures (0x3A1) have been received,
     * populate synthetic pressures so Hiworld Cmd 0x66 matches the discrete states. */
    if (!s_has_direct_pressures) {
        tpms->pressure_bar_deci[0] = synth_fl;
        tpms->pressure_bar_deci[1] = synth_fr;
        tpms->pressure_bar_deci[2] = synth_rl;
        tpms->pressure_bar_deci[3] = synth_rr;
    }

    tpms->valid = true;
}

void psa_tpms_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    switch (frame->id) {
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
