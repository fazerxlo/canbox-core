#include "core/vehicle_profile.h"
#include "profiles/peugeot_407.h"

static inline uint16_t read_be16_local(const uint8_t *d) {
    return ((uint16_t)d[0] << 8) | (uint16_t)d[1];
}

static vehicle_state_t *s_active_state_for_stalk = 0;

static void stalk_key_cb(uint8_t key_id, uint8_t state) {
    if (!s_active_state_for_stalk) return;

    if (state == 0) {
        s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_NONE;
        s_active_state_for_stalk->wheel.press_state = 0;
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
        case PSA_STALK_KEY_SCROLL_UP:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_NEXT;
            break;
        case PSA_STALK_KEY_PREV:
        case PSA_STALK_KEY_SCROLL_DOWN:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_PREV;
            break;
        case PSA_STALK_KEY_SRC:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_SRC;
            break;
        case PSA_STALK_KEY_TEL_ANSWER:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_PHONE_ACCEPT;
            break;
        case PSA_STALK_KEY_TEL_HANGUP:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_PHONE_HANGUP;
            break;
        default:
            s_active_state_for_stalk->wheel.active_key = WHEEL_KEY_NONE;
            break;
    }
    s_active_state_for_stalk->wheel.press_state = (state != 0) ? 1 : 0;
}

static void psa_decode_stalk_0x0f6(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;
    s_active_state_for_stalk = state;
    psa_stalk_process_can(frame->data, frame->dlc, stalk_key_cb);
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
    state->wheel.press_state = (btn != 0) ? 1 : 0;

    if (frame->dlc >= 5) {
        uint8_t light_flags = frame->data[4];
        state->lights.side_light = (light_flags & (1 << 7)) != 0;
        state->lights.headlights = (light_flags & (1 << 6)) != 0;
        state->lights.high_beam  = (light_flags & (1 << 5)) != 0;
        state->lights.front_fog  = (light_flags & (1 << 4)) != 0;
        state->lights.rear_fog   = (light_flags & (1 << 3)) != 0;
    }
}

static void psa_decode_doors_0x036(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 3) return;

    state->doors.door_driver     = (frame->data[0] & (1 << 0)) != 0;
    state->doors.door_passenger  = (frame->data[0] & (1 << 1)) != 0;
    state->doors.door_rear_left  = (frame->data[0] & (1 << 2)) != 0;
    state->doors.door_rear_right = (frame->data[0] & (1 << 3)) != 0;
    state->doors.trunk           = (frame->data[0] & (1 << 4)) != 0;
    state->doors.hood            = (frame->data[0] & (1 << 5)) != 0;

    state->reverse_gear          = (frame->data[1] & (1 << 7)) != 0;
    state->handbrake             = (frame->data[1] & (1 << 0)) != 0;

    if (frame->dlc >= 5) {
        state->ignition_state    = (vehicle_ignition_state_t)frame->data[4];
    }
}

static void psa_decode_doors_0x221_profile(const can_frame_t *frame, vehicle_state_t *state) {
    psa_doors_body_t doors;
    psa_decode_doors_0x221(frame->data, frame->dlc, &doors);
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

static void psa_decode_steering_angle_0x0e8(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;
    int16_t raw_angle = (int16_t)read_be16_local(&frame->data[0]);
    state->steering_angle_deg = raw_angle / 10;
}

static void psa_decode_steering_angle_0x0e6_profile(const can_frame_t *frame, vehicle_state_t *state) {
    int16_t angle = 0;
    psa_decode_steering_angle_0x0e6(frame->data, frame->dlc, &angle);
    state->steering_angle_deg = angle / 10;
}

static void psa_decode_hvac_0x1d0_profile(const can_frame_t *frame, vehicle_state_t *state) {
    hvac_state_t hvac;
    psa_decode_hvac_0x1d0(frame->data, frame->dlc, &hvac);
    state->climate.power_on        = hvac.power;
    state->climate.ac_on          = hvac.ac_compressor;
    state->climate.auto_mode       = hvac.auto_mode;
    state->climate.recirculate     = hvac.recirculation;
    state->climate.fan_speed       = hvac.fan_speed;
    state->climate.temp_driver     = hvac.driver_temp_raw;
    state->climate.temp_passenger  = hvac.pass_temp_raw;
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

static const profile_can_rule_t s_psa_rules[] = {
    { PSA_CAN_ID_REVERSE_IGNITION,  psa_decode_doors_0x036 },
    { PSA_CAN_ID_STEERING_ANGLE,    psa_decode_steering_angle_0x0e6_profile },
    { PSA_CAN_ID_STALK_BUTTONS,     psa_decode_stalk_0x0f6 },
    { 0x128,                        psa_decode_wheel_keys_0x128 },
    { PSA_CAN_ID_ALERTS_INDICATORS, psa_decode_alerts_0x168_profile },
    { PSA_CAN_ID_CRUISE_CONTROL,    psa_decode_cruise_0x1a8_profile },
    { 0x0B6,                        psa_decode_engine_speed_0x0b6 },
    { 0x0E8,                        psa_decode_steering_angle_0x0e8 },
    { PSA_CAN_ID_CLIMATE_HVAC,      psa_decode_hvac_0x1d0_profile },
    { PSA_CAN_ID_DOORS_BODY,        psa_decode_doors_0x221_profile },
};

static void psa_init(void) {
    psa_stalk_init(NULL);
}

const vehicle_profile_t g_profile_psa = {
    .id           = VEHICLE_PROFILE_PSA_2004,
    .name         = "PSA CAN2004/CAN2010 (Peugeot 407)",
    .default_baud = CAN_BAUD_125K,
    .rules        = s_psa_rules,
    .rule_count   = sizeof(s_psa_rules) / sizeof(s_psa_rules[0]),
    .init         = psa_init
};
