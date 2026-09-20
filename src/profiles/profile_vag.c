#include "core/vehicle_profile.h"

static inline uint16_t read_le16(const uint8_t *d) {
    return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}

static void vag_decode_wheel_keys(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 1) return;

    uint8_t btn = frame->data[0];
    switch (btn) {
        case 0x06: state->wheel.active_key = WHEEL_KEY_VOL_UP; break;
        case 0x07: state->wheel.active_key = WHEEL_KEY_VOL_DOWN; break;
        case 0x08: state->wheel.active_key = WHEEL_KEY_NEXT; break;
        case 0x09: state->wheel.active_key = WHEEL_KEY_PREV; break;
        case 0x0A: state->wheel.active_key = WHEEL_KEY_MUTE; break;
        case 0x0B: state->wheel.active_key = WHEEL_KEY_PHONE_ACCEPT; break;
        case 0x1A: state->wheel.active_key = WHEEL_KEY_SRC; break;
        default:   state->wheel.active_key = WHEEL_KEY_NONE; break;
    }
    state->wheel.press_state = (btn != 0) ? 1 : 0;
}

static void vag_decode_doors(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;

    state->doors.door_driver     = (frame->data[0] & (1 << 0)) != 0;
    state->doors.door_passenger  = (frame->data[0] & (1 << 1)) != 0;
    state->doors.door_rear_left  = (frame->data[0] & (1 << 2)) != 0;
    state->doors.door_rear_right = (frame->data[0] & (1 << 3)) != 0;
    state->doors.trunk           = (frame->data[0] & (1 << 4)) != 0;
    state->doors.hood            = (frame->data[0] & (1 << 5)) != 0;

    state->reverse_gear          = (frame->data[1] & (1 << 7)) != 0;
    state->handbrake             = (frame->data[1] & (1 << 0)) != 0;
}

static void vag_decode_engine_speed(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;

    state->rpm = read_le16(&frame->data[2]) / 4;
}

static void vag_decode_steering_angle(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;

    int16_t raw_angle = (int16_t)read_le16(&frame->data[0]);
    state->steering_angle_deg = raw_angle / 10;
}

static const profile_can_rule_t s_vag_rules[] = {
    { 0x5C0, vag_decode_wheel_keys },
    { 0x470, vag_decode_doors },
    { 0x280, vag_decode_engine_speed },
    { 0x0C2, vag_decode_steering_angle }
};

static void vag_init(void) {}

const vehicle_profile_t g_profile_vag = {
    .id           = VEHICLE_PROFILE_VAG_PQ35,
    .name         = "VAG PQ35/PQ46",
    .default_baud = CAN_BAUD_500K,
    .rules        = s_vag_rules,
    .rule_count   = sizeof(s_vag_rules) / sizeof(s_vag_rules[0]),
    .init         = vag_init
};

