#include "core/can_router.h"
#include "protocols/hu_protocol.h"
#include <string.h>

static vehicle_state_t s_current_state;
static vehicle_state_t s_last_sent_state;

/* Signal Extraction Helpers (Big/Little Endian bit unpacking) */
static inline uint16_t unpack_be16(const uint8_t *data) {
    return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

/* Example Vehicle CAN Handlers (Standard PSA/VW/General CAN layout) */

// ID 0x128: Steering wheel keys & stalks
static void handle_can_0x128(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 2) return;

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

// ID 0x036: Door contact sensors & reverse gear
static void handle_can_0x036(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;

    state->doors.door_driver     = (frame->data[0] & (1 << 0)) != 0;
    state->doors.door_passenger  = (frame->data[0] & (1 << 1)) != 0;
    state->doors.door_rear_left  = (frame->data[0] & (1 << 2)) != 0;
    state->doors.door_rear_right = (frame->data[0] & (1 << 3)) != 0;
    state->doors.trunk           = (frame->data[0] & (1 << 4)) != 0;
    state->doors.hood            = (frame->data[0] & (1 << 5)) != 0;

    state->reverse_gear          = (frame->data[1] & (1 << 7)) != 0;
    state->handbrake             = (frame->data[1] & (1 << 0)) != 0;
}

// ID 0x0B6: Engine speed and vehicle road speed
static void handle_can_0x0B6(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 8) return;

    state->rpm       = (unpack_be16(&frame->data[0]) >> 3); // Raw RPM / 8
    state->speed_kmh = (unpack_be16(&frame->data[2]) >> 7); // Speed scale
}

// ID 0x0E8: Steering Wheel Angle (SAS)
static void handle_can_0x0E8(const can_frame_t *frame, vehicle_state_t *state) {
    if (frame->dlc < 4) return;

    int16_t raw_angle = (int16_t)unpack_be16(&frame->data[0]);
    state->steering_angle_deg = raw_angle / 10;
}

/* Dispatch Routing Table */
static const can_router_rule_t ROUTER_TABLE[] = {
    { 0x128, handle_can_0x128 },
    { 0x036, handle_can_0x036 },
    { 0x0B6, handle_can_0x0B6 },
    { 0x0E8, handle_can_0x0E8 }
};

#define ROUTER_TABLE_SIZE (sizeof(ROUTER_TABLE) / sizeof(ROUTER_TABLE[0]))

void can_router_init(void) {
    memset(&s_current_state, 0, sizeof(s_current_state));
    memset(&s_last_sent_state, 0, sizeof(s_last_sent_state));
    hu_protocol_init();
}

void can_router_process_can(const can_frame_t *frame) {
    if (!frame) return;

    for (size_t i = 0; i < ROUTER_TABLE_SIZE; i++) {
        if (ROUTER_TABLE[i].can_id == frame->id) {
            ROUTER_TABLE[i].handler(frame, &s_current_state);
            break;
        }
    }

    // Immediately push high-priority event-driven changes (steering keys)
    if (s_current_state.wheel.active_key != s_last_sent_state.wheel.active_key ||
        s_current_state.wheel.press_state != s_last_sent_state.wheel.press_state) {
        
        hu_protocol_send_wheel_key(&s_current_state.wheel);
        s_last_sent_state.wheel = s_current_state.wheel;
    }

    // Immediately push door status updates on change
    if (memcmp(&s_current_state.doors, &s_last_sent_state.doors, sizeof(vehicle_doors_t)) != 0) {
        hu_protocol_send_doors(&s_current_state.doors);
        s_last_sent_state.doors = s_current_state.doors;
    }
}

void can_router_process_uart_byte(uint8_t byte) {
    hu_protocol_feed_byte(byte);
}

void can_router_periodic_100ms(void) {
    // Broadcast periodic states (Vehicle speed, RPM, Radar/Parking)
    if (s_current_state.speed_kmh != s_last_sent_state.speed_kmh ||
        s_current_state.rpm != s_last_sent_state.rpm ||
        s_current_state.steering_angle_deg != s_last_sent_state.steering_angle_deg) {

        hu_protocol_send_telemetry(s_current_state.speed_kmh,
                                   s_current_state.rpm,
                                   s_current_state.steering_angle_deg);

        s_last_sent_state.speed_kmh = s_current_state.speed_kmh;
        s_last_sent_state.rpm = s_current_state.rpm;
        s_last_sent_state.steering_angle_deg = s_current_state.steering_angle_deg;
    }

    // Send keep-alive packet to keep Android HU comms link alive
    hu_protocol_send_heartbeat();
}

const vehicle_state_t *can_router_get_state(void) {
    return &s_current_state;
}
