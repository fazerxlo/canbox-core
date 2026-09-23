#include "core/can_router.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include <string.h>

static vehicle_state_t s_current_state;
static vehicle_state_t s_last_sent_state;

void can_router_init(void) {
    memset(&s_current_state, 0, sizeof(s_current_state));
    memset(&s_last_sent_state, 0, sizeof(s_last_sent_state));
    vehicle_profile_init();
    hu_protocol_init();
}

void can_router_process_can(const can_frame_t *frame) {
    if (!frame) return;

    vehicle_profile_process_frame(frame, &s_current_state);

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

    // Continuously repeat door status frame while ANY door/trunk/hood is open.
    // Stops repeating as soon as all doors are closed.
    bool any_door_open = s_current_state.doors.door_driver ||
                         s_current_state.doors.door_passenger ||
                         s_current_state.doors.door_rear_left ||
                         s_current_state.doors.door_rear_right ||
                         s_current_state.doors.trunk ||
                         s_current_state.doors.hood;

    if (any_door_open) {
        hu_protocol_send_doors(&s_current_state.doors);
    }

    // Send keep-alive packet to keep Android HU comms link alive
    hu_protocol_send_heartbeat();
}

const vehicle_state_t *can_router_get_state(void) {
    return &s_current_state;
}
