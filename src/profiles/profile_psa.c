#include "core/vehicle_profile.h"
#include "core/can_router.h"
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

static bool s_reverse_from_0f6 = false;
static bool s_reverse_from_036 = false;
static bool s_have_0f6 = false;
static bool s_ignition_from_0f6 = false;

static inline void psa_update_derived_power_state(vehicle_state_t *state);

static void psa_decode_bsi_slow_0x0f6(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc >= 8) {
        s_have_0f6 = true;
        s_ignition_from_0f6 = (frame->data[0] & (1 << 3)) != 0;
        state->ignition_on = s_ignition_from_0f6;

        s_reverse_from_0f6 = (frame->data[7] & (1 << 7)) != 0;
        state->reverse_gear = s_reverse_from_0f6 || s_reverse_from_036;

        /* Outside temperature: Byte 5, Raw * 0.5 - 40 C (0xFF = invalid; keep last value) */
        if (frame->data[5] != 0xFF) {
            state->climate.outdoor_temp_raw = frame->data[5];
        }

        psa_update_derived_power_state(state);
    }
}

/* PSA Comfort CAN 0x10B: BSI-routed steering wheel angle (0.1 deg, signed BE, +Left / -Right) */
static void psa_decode_steering_angle_0x10b_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;
    if (frame->data[3] != 0x00) return; /* Sensor error flags set: discard */

    int16_t raw_deci_deg = (int16_t)read_be16_local(&frame->data[0]);
    if (raw_deci_deg == (int16_t)0x7FFF) return; /* Invalid sentinel */

    state->steering_angle_deg = (int16_t)(raw_deci_deg / 10);
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

static bool s_prev_ignition_on = false;
static bool s_rd4_prev_powered = false;
static bool s_rd4_manual_acc   = false;

static inline void psa_update_derived_power_state(vehicle_state_t *state) {
    bool rd4_present = psa_rd4_is_connected(&state->media.radio);
    bool rd4_powered = rd4_present && psa_rd4_is_powered(&state->media.radio) && !state->media.radio.sleep;

    // 1. Economy Mode has absolute authority -> Force OFF immediately
    if (state->economy_mode) {
        s_rd4_manual_acc = false;
        state->ignition_state = VEHICLE_IGNITION_OFF;
    }
    // 2. Ignition is ON (+APC) -> Force ON immediately
    // RD4 sleep, restart, silence, or absence is completely ignored.
    else if (state->ignition_on) {
        s_rd4_manual_acc = false;
        state->ignition_state = VEHICLE_IGNITION_ON;
    }
    // 3. Ignition is OFF
    else {
        // Falling edge: If ignition just turned OFF, cut HU power immediately
        if (s_prev_ignition_on) {
            s_rd4_manual_acc = false;
        }

        // RD4 accessory handling while parked
        if (rd4_present) {
            if (!s_rd4_prev_powered && rd4_powered) {
                // Rising edge: User explicitly pushed RD4 power knob while key is OFF
                s_rd4_manual_acc = true;
            } else if (!rd4_powered) {
                // User pushed RD4 power knob to turn off, or RD4 entered sleep
                s_rd4_manual_acc = false;
            }
        } else {
            s_rd4_manual_acc = false;
        }

        state->ignition_state = s_rd4_manual_acc ? VEHICLE_IGNITION_ACC : VEHICLE_IGNITION_OFF;
    }

    s_prev_ignition_on = state->ignition_on;
    s_rd4_prev_powered = rd4_powered;
}

static void psa_decode_ignition_reverse_0x036(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;

    s_reverse_from_036 = (frame->data[1] & (1 << 7)) != 0;
    state->reverse_gear = s_reverse_from_0f6 || s_reverse_from_036;
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
        if (phase_vie == 0x00 || phase_vie == 0x02) {
            /* 0x00: Deep sleep, 0x02: Shutdown countdown -> Ignition is definitively OFF */
            state->ignition_on = false;
            s_ignition_from_0f6 = false;
        } else if (s_have_0f6) {
            /* 0x0F6 Byte 0 Bit 3 is the authoritative vehicle ground truth for +APC */
            state->ignition_on = s_ignition_from_0f6;
        } else {
            /* Fallback for standalone/synthetic test environments without 0x0F6 */
            state->ignition_on = (phase_vie == 0x01 || phase_vie == 0x03);
        }
    }

    psa_update_derived_power_state(state);
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

static psa_journal_iso_tp_t s_journal_ctx;

static void psa_decode_alert_message_0x1a1_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    psa_decode_alert_message_0x1a1(frame->data, frame->dlc, &state->alerts.realtime_alert);
    state->dirty.alert_realtime = true;
}

static void psa_decode_alert_journal_0x120_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    vehicle_alert_item_t items[CANBOX_MAX_ACTIVE_ALERTS] = {0};
    uint8_t count = 0;
    if (psa_process_journal_0x120(&s_journal_ctx, frame->data, frame->dlc, items, &count)) {
        memcpy(state->alerts.active_items, items, sizeof(items));
        state->alerts.active_count = count;
        state->dirty.alert_journal = true;
    }
}

static void psa_decode_etat_autoradio_0x165_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    psa_rd4_decode_frame(frame, state);
    psa_update_derived_power_state(state);
}

static const profile_can_rule_t s_psa_rules[] = {
    { PSA_CAN_ID_REVERSE_IGNITION,   psa_decode_ignition_reverse_0x036 },
    { PSA_CAN_ID_TEXTE_RADIO,        psa_rd4_decode_frame },
    { PSA_CAN_ID_RADAR_0E1,          psa_radar_decode_frame },
    { 0x0F6,                         psa_decode_bsi_slow_0x0f6 },
    { PSA_CAN_ID_STEERING_SAS_10B,   psa_decode_steering_angle_0x10b_profile },
    { PSA_CAN_ID_STALK_21F,          psa_decode_stalk_0x21f_profile },
    { PSA_CAN_ID_CONSOLE_3E5,        psa_decode_console_0x3e5_profile },
    { PSA_CAN_ID_ALERT_JOURNAL,      psa_decode_alert_journal_0x120_profile },
    { PSA_CAN_ID_STATION_LIST_125,   psa_rd4_decode_frame },
    { 0x128,                         psa_decode_wheel_keys_0x128 },
    { PSA_CAN_ID_ETAT_AUTORADIO,     psa_decode_etat_autoradio_0x165_profile },
    { PSA_CAN_ID_ALERTS_INDICATORS,  psa_decode_alerts_0x168_profile },
    { PSA_CAN_ID_ALERT_MESSAGE,      psa_decode_alert_message_0x1a1_profile },
    { PSA_CAN_ID_RD4_VOLUME,         psa_rd4_decode_frame },
    { PSA_CAN_ID_CRUISE_CONTROL,     psa_decode_cruise_0x1a8_profile },
    { 0x0B6,                         psa_engine_decode_frame },
    { PSA_CAN_ID_CLIMATE_HVAC,       psa_hvac_decode_frame },
    { 0x1E3,                         psa_hvac_decode_frame },
    { PSA_CAN_ID_RD4_AUDIO_SETTINGS, psa_rd4_decode_frame },
    { PSA_CAN_ID_DOORS_BODY_220,     psa_doors_decode_frame },
    { PSA_CAN_ID_TRIP_INSTANT,       psa_trip_decode_frame },
    { PSA_CAN_ID_ETAT_TUNER,         psa_rd4_decode_frame },
    { PSA_CAN_ID_INFO_TUNER,         psa_rd4_decode_frame },
    { PSA_CAN_ID_TRIP2_ODB,          psa_trip_decode_frame },
    { PSA_CAN_ID_TRIP1_ODB,          psa_trip_decode_frame },
    { PSA_CAN_ID_NOM_STATION,        psa_rd4_decode_frame },
    { PSA_CAN_ID_TPMS_STATUS_1E1,    psa_tpms_decode_frame },
    { PSA_CAN_ID_TPMS_DIRECT_361,    psa_tpms_decode_frame },
    { PSA_CAN_ID_TPMS_PRESSURES_3A1, psa_tpms_decode_frame },
    { PSA_CAN_ID_CD_TRAY_325,        psa_rd4_decode_frame },
    { PSA_CAN_ID_CD_DISC_INFO_365,   psa_rd4_decode_frame },
    { PSA_CAN_ID_CD_PLAY_3A5,        psa_rd4_decode_frame },
    { PSA_CAN_ID_CD_CHANGER,         psa_rd4_decode_frame },
};

static void psa_init(void) {
    s_reverse_from_0f6 = false;
    s_reverse_from_036 = false;
    s_have_0f6         = false;
    s_ignition_from_0f6 = false;
    s_prev_ignition_on = false;
    s_rd4_prev_powered = false;
    s_rd4_manual_acc   = false;
    psa_stalk_init(NULL);
    psa_console_init(NULL);
    psa_journal_iso_tp_init(&s_journal_ctx);
    psa_rd4_media_init(NULL, NULL);
}

static bool psa_reset_trip(uint8_t trip_index) {
    if (can_router_is_bus_sleeping()) {
        return false;
    }
    return psa_trip_send_reset(trip_index) == HAL_STATUS_OK;
}

static bool psa_query_alerts(void) {
    if (can_router_is_bus_sleeping()) {
        return false;
    }
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
