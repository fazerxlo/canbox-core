#include "core/vehicle_profile.h"

static inline uint16_t read_be16(const uint8_t *d) {
    return ((uint16_t)d[0] << 8) | (uint16_t)d[1];
}

static void psa_decode_wheel_keys(const can_frame_t *frame, vehicle_state_t *state) {
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
}

static void psa_decode_doors(const can_frame_t *frame, vehicle_state_t *state) {
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

static void psa_decode_engine_speed(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 8) return;

    state->rpm = read_be16(&frame->data[0]) >> 3;
    state->speed_kmh = read_be16(&frame->data[2]) >> 7;
}

static void psa_decode_steering_angle(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;
    int16_t raw_angle = (int16_t)read_be16(&frame->data[0]);
    state->steering_angle_deg = raw_angle / 10;
}

static const profile_can_rule_t s_psa_rules[] = {
    { 0x128, psa_decode_wheel_keys },
    { 0x036, psa_decode_doors },
    { 0x0B6, psa_decode_engine_speed },
    { 0x0E8, psa_decode_steering_angle }
};

static void psa_init(void) {}

const vehicle_profile_t g_profile_psa = {
    .id           = VEHICLE_PROFILE_PSA_2004,
    .name         = "PSA CAN2004/CAN2010",
    .default_baud = CAN_BAUD_125K,
    .rules        = s_psa_rules,
    .rule_count   = sizeof(s_psa_rules) / sizeof(s_psa_rules[0]),
    .init         = psa_init
};
