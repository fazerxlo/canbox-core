#include "core/vehicle_profile.h"
#include "profiles/peugeot_407.h"
#include <string.h>

static inline uint16_t read_be16_local(const uint8_t *d) {
    return ((uint16_t)d[0] << 8) | (uint16_t)d[1];
}

static vehicle_state_t *s_active_state_for_stalk = 0;
static vehicle_state_t *s_active_state_for_console = 0;

static void stalk_key_cb(uint8_t key_id, uint8_t state) {
    if (!s_active_state_for_stalk) return;

    if (state == 0) {
        if (key_id != PSA_STALK_KEY_SCROLL_UP && key_id != PSA_STALK_KEY_SCROLL_DOWN) {
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_NONE;
            s_active_state_for_stalk->wheel.press_state = 0;
        }
        return;
    }

    switch (key_id) {
        case PSA_STALK_KEY_VOL_UP:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_VOL_UP;
            break;
        case PSA_STALK_KEY_VOL_DOWN:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_VOL_DOWN;
            break;
        case PSA_STALK_KEY_NEXT:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_NEXT;
            break;
        case PSA_STALK_KEY_PREV:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_PREV;
            break;
        case PSA_STALK_KEY_SCROLL_UP:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_SCROLL_UP;
            break;
        case PSA_STALK_KEY_SCROLL_DOWN:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_SCROLL_DOWN;
            break;
        case PSA_STALK_KEY_SRC:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_SRC;
            break;
        case PSA_STALK_KEY_MUTE:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_MUTE;
            break;
        case PSA_STALK_KEY_TEL_ANSWER:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_PHONE_ACCEPT;
            break;
        case PSA_STALK_KEY_TEL_HANGUP:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_PHONE_HANGUP;
            break;
        case PSA_STALK_KEY_TRIP:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_TRIP;
            break;
        default:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_NONE;
            break;
    }
    if (s_active_state_for_stalk->wheel.active_key == WHEEL_KEY_NONE) {
        s_active_state_for_stalk->wheel.press_state = 0;
    } else {
        s_active_state_for_stalk->wheel.press_state = (state != 0) ? 1 : 0;
    }
}

static void console_key_cb(uint8_t key_id, uint8_t state) {
    if (!s_active_state_for_console) return;

    s_active_state_for_console->panel_key.key_code = (state != 0) ? key_id : 0;
    s_active_state_for_console->panel_key.press_state = state;
}

static void psa_decode_stalk_0x21f_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;
    s_active_state_for_stalk = state;
    psa_decode_stalk_0x21f(frame->data, frame->dlc, stalk_key_cb);
}

static void psa_decode_console_0x3e5_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 6) return;
    s_active_state_for_console = state;
    psa_decode_console_0x3e5(frame->data, frame->dlc, console_key_cb);
}

static void psa_decode_bsi_slow_0x0f6(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc >= 8) {
        state->reverse_gear = (frame->data[7] & (1 << 7)) != 0;
    }
}

static void psa_decode_wheel_keys_0x128(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 1) return;

    uint8_t btn = frame->data[0];
    switch (btn) {
        case 0x01: state->wheel.active_key = WHEEL_KEY_VOL_UP; break;
        case 0x02: state->wheel.active_key = WHEEL_KEY_VOL_DOWN; break;
        case 0x04: state->wheel.active_key = WHEEL_KEY_NEXT; break;
        case 0x08: state->wheel.active_key = WHEEL_KEY_PREV; break;
        case 0x10: state->wheel.active_key = WHEEL_KEY_SRC; break;
        case 0x20: state->wheel.active_key = WHEEL_KEY_MUTE; break;
        default:   state->wheel.active_key = WHEEL_KEY_NONE; break;
    }
    state->wheel.press_state = (state->wheel.active_key != WHEEL_KEY_NONE) ? 1 : 0;

    if (frame->dlc >= 5) {
        uint8_t light_flags = frame->data[4];
        state->lights.side_light = (light_flags & (1 << 7)) != 0;
        state->lights.headlights = (light_flags & (1 << 6)) != 0;
        state->lights.high_beam  = (light_flags & (1 << 5)) != 0;
        state->lights.front_fog  = (light_flags & (1 << 4)) != 0;
        state->lights.rear_fog   = (light_flags & (1 << 3)) != 0;
    }
}

static inline void psa_update_derived_power_state(vehicle_state_t *state) {
    if (state->economy_mode) {
        state->ignition_state = VEHICLE_IGNITION_OFF;
    } else if (state->radio_present) {
        /* When factory radio is present:
         * If radio is in sleep / standby mode ((data[0] & 0xC0) == 0), ACC is OFF.
         * If radio is active (knob turned on, bit 7), ACC is ON.
         * Otherwise (shutting down / transition), follow ignition status. */
        if (state->radio_sleep) {
            state->ignition_state = VEHICLE_IGNITION_OFF;
        } else if (state->radio_on) {
            state->ignition_state = state->ignition_on ? VEHICLE_IGNITION_ON : VEHICLE_IGNITION_ACC;
        } else {
            state->ignition_state = state->ignition_on ? VEHICLE_IGNITION_ON : VEHICLE_IGNITION_OFF;
        }
    } else {
        /* No factory radio detected on bus: ACC follows ignition switch */
        if (state->ignition_on) {
            state->ignition_state = VEHICLE_IGNITION_ON;
        } else {
            state->ignition_state = VEHICLE_IGNITION_OFF;
        }
    }
}

static void psa_decode_ignition_reverse_0x036(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;

    state->reverse_gear = (frame->data[1] & (1 << 7)) != 0;
    state->handbrake    = (frame->data[1] & (1 << 0)) != 0;

    if (frame->dlc >= 3) {
        state->economy_mode = (frame->data[2] & 0x80) != 0;
    }

    if (frame->dlc >= 4) {
        state->lights.illumination = (frame->data[3] & 0x20) != 0;
        state->lights.brightness   = (frame->data[3] & 0x0F);
    }

    if (frame->dlc >= 5) {
        uint8_t phase_vie = frame->data[4] & 0x07;
        /* 0x01: Run (+APC), 0x03: Wakeup / Key turn pulse */
        state->ignition_on = (phase_vie == 0x01 || phase_vie == 0x03);
    }

    psa_update_derived_power_state(state);
}

static void psa_decode_radio_power_0x165(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 1) return;
    state->radio_present = true;
    /* Bit 7 = 1 indicates factory RD4 radio audio is powered ON */
    state->radio_on = (frame->data[0] & 0x80) != 0;
    /* (Bits [7:6] == 0) indicates radio is in deep sleep / standby (e.g. 0x08) */
    state->radio_sleep = ((frame->data[0] & 0xC0) == 0);
    psa_update_derived_power_state(state);
}

static void psa_decode_emf_heartbeat_0x0df(const can_frame_t *frame, vehicle_state_t *state) {
    (void)frame;
    /* Periodic display state broadcast from OEM Multi-Function Display (EMF) */
    state->mfd_present = true;
}

static void psa_decode_emf_commands_0x167(const can_frame_t *frame, vehicle_state_t *state) {
    (void)frame;
    /* Periodic display commands / active page broadcast from OEM Multi-Function Display (EMF) */
    state->mfd_present = true;
}

static void psa_decode_doors_0x220_profile(const can_frame_t *frame, vehicle_state_t *state) {
    psa_doors_body_t doors;
    psa_decode_doors_0x220(frame->data, frame->dlc, &doors);
    state->doors.door_driver     = doors.driver_door;
    state->doors.door_passenger  = doors.pass_door;
    state->doors.door_rear_left  = doors.rear_left_door;
    state->doors.door_rear_right = doors.rear_right_door;
    state->doors.trunk           = doors.trunk;
    state->doors.hood            = doors.hood;
    state->handbrake             = doors.handbrake;
}

static void psa_decode_engine_speed_0x0b6(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 8) return;

    state->rpm = read_be16_local(&frame->data[0]) >> 3;
    state->speed_kmh = read_be16_local(&frame->data[2]) >> 7;
}

static void psa_decode_hvac_0x1d0_profile(const can_frame_t *frame, vehicle_state_t *state) {
    psa_hvac_process_can_0x1d0(&state->climate, frame->data, frame->dlc);
}

static void psa_decode_hvac_0x1e3_profile(const can_frame_t *frame, vehicle_state_t *state) {
    psa_hvac_process_can_0x1e3(&state->climate, frame->data, frame->dlc);
}

static void psa_decode_cruise_0x1a8_profile(const can_frame_t *frame, vehicle_state_t *state) {
    (void)state;
    bool active = false;
    uint8_t set_spd = 0;
    uint32_t odo = 0;
    psa_decode_cruise_0x1a8(frame->data, frame->dlc, &active, &set_spd, &odo);
}

static void psa_decode_alerts_0x168_profile(const can_frame_t *frame, vehicle_state_t *state) {
    (void)state;
    bool tpms_fault = false, tpms_under = false, tpms_punc = false, esp_fault = false;
    psa_decode_alerts_0x168(frame->data, frame->dlc, &tpms_fault, &tpms_under, &tpms_punc, &esp_fault);
}

static void psa_decode_tpms_0x361_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 8) return;

    uint16_t fl_raw = read_be16_local(&frame->data[0]);
    uint16_t fr_raw = read_be16_local(&frame->data[2]);
    uint16_t rr_raw = read_be16_local(&frame->data[4]);
    uint16_t rl_raw = read_be16_local(&frame->data[6]);

    state->tpms.alarm_state[0] = (uint8_t)((fl_raw >> 14) & 0x03);
    state->tpms.alarm_state[1] = (uint8_t)((fr_raw >> 14) & 0x03);
    state->tpms.alarm_state[2] = (uint8_t)((rl_raw >> 14) & 0x03);
    state->tpms.alarm_state[3] = (uint8_t)((rr_raw >> 14) & 0x03);

    if (state->tpms.alarm_state[0] != 3 && (fl_raw & 0x3FFF) != 0x3FFF) {
        state->tpms.pressure_bar_deci[0] = (uint8_t)(fl_raw & 0x3FFF);
    } else {
        state->tpms.pressure_bar_deci[0] = 0;
    }
    if (state->tpms.alarm_state[1] != 3 && (fr_raw & 0x3FFF) != 0x3FFF) {
        state->tpms.pressure_bar_deci[1] = (uint8_t)(fr_raw & 0x3FFF);
    } else {
        state->tpms.pressure_bar_deci[1] = 0;
    }
    if (state->tpms.alarm_state[2] != 3 && (rl_raw & 0x3FFF) != 0x3FFF) {
        state->tpms.pressure_bar_deci[2] = (uint8_t)(rl_raw & 0x3FFF);
    } else {
        state->tpms.pressure_bar_deci[2] = 0;
    }
    if (state->tpms.alarm_state[3] != 3 && (rr_raw & 0x3FFF) != 0x3FFF) {
        state->tpms.pressure_bar_deci[3] = (uint8_t)(rr_raw & 0x3FFF);
    } else {
        state->tpms.pressure_bar_deci[3] = 0;
    }

    state->tpms.valid = true;
}

static void psa_decode_tpms_0x3a1_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;

    state->tpms.pressure_bar_deci[0] = (uint8_t)((frame->data[0] * 5 + 5) / 10);
    state->tpms.pressure_bar_deci[1] = (uint8_t)((frame->data[1] * 5 + 5) / 10);
    state->tpms.pressure_bar_deci[2] = (uint8_t)((frame->data[3] * 5 + 5) / 10); /* RL */
    state->tpms.pressure_bar_deci[3] = (uint8_t)((frame->data[2] * 5 + 5) / 10); /* RR */
    state->tpms.valid = true;
}

static void psa_decode_tpms_0x1e1_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;

    uint8_t fl_st = (uint8_t)((frame->data[0] >> 3) & 0x07);
    uint8_t fr_st = (uint8_t)((frame->data[1] >> 3) & 0x07);
    uint8_t rr_st = (uint8_t)((frame->data[2] >> 3) & 0x07);
    uint8_t rl_st = (uint8_t)((frame->data[3] >> 3) & 0x07);

    state->tpms.alarm_state[0] = (fl_st >= 3) ? 3 : fl_st;
    state->tpms.alarm_state[1] = (fr_st >= 3) ? 3 : fr_st;
    state->tpms.alarm_state[2] = (rl_st >= 3) ? 3 : rl_st;
    state->tpms.alarm_state[3] = (rr_st >= 3) ? 3 : rr_st;
    state->tpms.valid = true;
}

static void psa_decode_trip_0x221_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 1) return;

    /* Stalk Tip Trip button on Byte 0 Bit 3 (0x08) */
    s_active_state_for_stalk = state;
    psa_decode_stalk_tip_0x221(frame->data, frame->dlc, stalk_key_cb);

    if (frame->dlc < 7) return;

    /* Byte 0 Bit 7: Fuel info hidden/invalid.
     * Also guard against uninitialized/BSI status frames where all data is 0xFF.
     */
    if ((frame->data[0] & 0x80) != 0) return;
    uint16_t fuel = read_be16_local(&frame->data[1]);
    uint16_t range = read_be16_local(&frame->data[3]);
    if (fuel == 0xFFFF && range == 0xFFFF) return;

    state->trip.instant_fuel_deci = fuel;
    state->trip.range_km = range;
    state->trip.dest_dist_km = read_be16_local(&frame->data[5]);
    state->trip.instant_valid = true;
    state->trip.updated_page = 1;
}

static void psa_decode_trip1_0x2a1_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 5) return;

    state->trip.trip1_distance_km = read_be16_local(&frame->data[1]);
    state->trip.trip1_avg_fuel    = read_be16_local(&frame->data[3]);
    state->trip.trip1_avg_speed   = (frame->dlc >= 7 && (frame->data[5] || frame->data[6])) ?
                                     (uint8_t)read_be16_local(&frame->data[5]) : frame->data[0];
    state->trip.trip1_valid = true;
    state->trip.updated_page = 2;
}

static void psa_decode_trip2_0x261_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 5) return;

    state->trip.trip2_distance_km = read_be16_local(&frame->data[1]);
    state->trip.trip2_avg_fuel    = read_be16_local(&frame->data[3]);
    state->trip.trip2_avg_speed   = (frame->dlc >= 7 && (frame->data[5] || frame->data[6])) ?
                                     (uint8_t)read_be16_local(&frame->data[5]) : frame->data[0];
    state->trip.trip2_valid = true;
    state->trip.updated_page = 3;
}

static void psa_decode_radar_0x0e1_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 6) return;

    state->radar.rear_active    = (frame->data[1] >> 6) & 1;
    state->radar.front_active   = (frame->data[1] >> 4) & 1;
    state->radar.display_active = (frame->data[5] & 0x02) ? true : false;

    if (!state->radar.display_active && !state->radar.rear_active && !state->radar.front_active) {
        state->radar.rear_left_outer    = 0xFF;
        state->radar.rear_left_center   = 0xFF;
        state->radar.rear_right_center  = 0xFF;
        state->radar.rear_right_outer   = 0xFF;
        state->radar.front_left_outer   = 0xFF;
        state->radar.front_left_center  = 0xFF;
        state->radar.front_right_center = 0xFF;
        state->radar.front_right_outer  = 0xFF;
    } else {
        uint8_t rl = (frame->data[3] >> 5) & 0x07;
        uint8_t rc = (frame->data[3] >> 2) & 0x07;
        uint8_t rr = (frame->data[4] >> 5) & 0x07;
        uint8_t fl = (frame->data[4] >> 2) & 0x07;
        uint8_t fc = (frame->data[5] >> 5) & 0x07;
        uint8_t fr = (frame->data[5] >> 2) & 0x07;

        state->radar.rear_left_outer    = psa_radar_map_zone(rl);
        state->radar.rear_left_center   = psa_radar_map_zone(rc);
        state->radar.rear_right_center  = psa_radar_map_zone(rc);
        state->radar.rear_right_outer   = psa_radar_map_zone(rr);

        state->radar.front_left_outer   = psa_radar_map_zone(fl);
        state->radar.front_left_center  = psa_radar_map_zone(fc);
        state->radar.front_right_center = psa_radar_map_zone(fc);
        state->radar.front_right_outer  = psa_radar_map_zone(fr);
    }
    state->radar.valid = true;
    state->radar.updated = true;
}

static psa_journal_iso_tp_t s_journal_ctx;

static void psa_decode_alert_message_0x1a1_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    psa_decode_alert_message_0x1a1(frame->data, frame->dlc, &state->alerts.realtime_alert);
    state->alerts.realtime_updated = true;
}

static void psa_decode_alert_journal_0x120_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    vehicle_alert_item_t items[CANBOX_MAX_ACTIVE_ALERTS] = {0};
    uint8_t count = 0;
    if (psa_process_journal_0x120(&s_journal_ctx, frame->data, frame->dlc, items, &count)) {
        memcpy(state->alerts.active_items, items, sizeof(items));
        state->alerts.active_count = count;
        state->alerts.journal_updated = true;
    }
}

static const profile_can_rule_t s_psa_rules[] = {
    { PSA_CAN_ID_REVERSE_IGNITION,   psa_decode_ignition_reverse_0x036 },
    { PSA_CAN_ID_RADAR_0E1,          psa_decode_radar_0x0e1_profile },
    { 0x0F6,                         psa_decode_bsi_slow_0x0f6 },
    { PSA_CAN_ID_STALK_21F,          psa_decode_stalk_0x21f_profile },
    { PSA_CAN_ID_CONSOLE_3E5,        psa_decode_console_0x3e5_profile },
    { PSA_CAN_ID_ALERT_JOURNAL,      psa_decode_alert_journal_0x120_profile },
    { 0x128,                         psa_decode_wheel_keys_0x128 },
    { PSA_CAN_ID_ALERTS_INDICATORS,  psa_decode_alerts_0x168_profile },
    { PSA_CAN_ID_ALERT_MESSAGE,      psa_decode_alert_message_0x1a1_profile },
    { PSA_CAN_ID_CRUISE_CONTROL,     psa_decode_cruise_0x1a8_profile },
    { 0x0B6,                         psa_decode_engine_speed_0x0b6 },
    { PSA_CAN_ID_CLIMATE_HVAC,       psa_decode_hvac_0x1d0_profile },
    { 0x1E3,                         psa_decode_hvac_0x1e3_profile },
    { PSA_CAN_ID_DOORS_BODY_220,     psa_decode_doors_0x220_profile },
    { PSA_CAN_ID_TRIP_INSTANT,       psa_decode_trip_0x221_profile },
    { PSA_CAN_ID_TRIP1_ODB,          psa_decode_trip1_0x2a1_profile },
    { PSA_CAN_ID_TRIP2_ODB,          psa_decode_trip2_0x261_profile },
    { PSA_CAN_ID_TPMS_STATUS_1E1,    psa_decode_tpms_0x1e1_profile },
    { PSA_CAN_ID_TPMS_DIRECT_361,    psa_decode_tpms_0x361_profile },
    { PSA_CAN_ID_TPMS_PRESSURES_3A1, psa_decode_tpms_0x3a1_profile },
    { 0x165,                         psa_decode_radio_power_0x165 },
    { PSA_CAN_ID_EMF_STATUS_0DF,     psa_decode_emf_heartbeat_0x0df },
    { PSA_CAN_ID_EMF_COMMANDS_167,   psa_decode_emf_commands_0x167 },
};

static void psa_init(void) {
    psa_stalk_init(NULL);
    psa_console_init(NULL);
    psa_journal_iso_tp_init(&s_journal_ctx);
}

static bool psa_reset_trip(uint8_t trip_index) {
    return psa_trip_send_reset(trip_index) == HAL_STATUS_OK;
}

static bool psa_query_alerts(void) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = PSA_CAN_ID_ALERT_QUERY;
    frame.dlc = 8;
    frame.data[0] = 0x01;
    frame.data[1] = 0x01;
    return hal_can_send(&frame) == HAL_STATUS_OK;
}

const vehicle_profile_t g_profile_psa = {
    .id           = VEHICLE_PROFILE_PSA_2004,
    .name         = "PSA CAN2004/CAN2010 (Peugeot 407)",
    .default_baud = CAN_BAUD_125K,
    .rules        = s_psa_rules,
    .rule_count   = sizeof(s_psa_rules) / sizeof(s_psa_rules[0]),
    .init         = psa_init,
    .reset_trip   = psa_reset_trip,
    .query_alerts = psa_query_alerts
};
