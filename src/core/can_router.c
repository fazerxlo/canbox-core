#include "core/can_router.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_gpio.h"
#include <string.h>

static vehicle_state_t s_current_state;
static vehicle_state_t s_last_sent_state;
static uint16_t        s_tpms_periodic_timer = 0;
static uint8_t         s_doors_periodic_timer = 0;

void can_router_init(void) {
    memset(&s_current_state, 0, sizeof(s_current_state));
    memset(&s_last_sent_state, 0, sizeof(s_last_sent_state));
    s_tpms_periodic_timer = 0;
    s_doors_periodic_timer = 0;
    hal_gpio_write(GPIO_PIN_REVERSE_OUT, false);
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

    // Immediately push climate updates on change
    if (memcmp(&s_current_state.climate, &s_last_sent_state.climate, sizeof(vehicle_climate_t)) != 0) {
        hu_protocol_send_climate(&s_current_state.climate);
        s_last_sent_state.climate = s_current_state.climate;
    }

    // Immediately push TPMS updates on change (decoupling numeric pressures and discrete alarms)
    if (s_current_state.tpms.valid) {
        bool first_tpms = !s_last_sent_state.tpms.valid;
        bool pressures_changed = first_tpms || (memcmp(s_current_state.tpms.pressure_bar_deci,
                                                       s_last_sent_state.tpms.pressure_bar_deci,
                                                       sizeof(s_current_state.tpms.pressure_bar_deci)) != 0);
        bool alarms_changed = first_tpms || (memcmp(s_current_state.tpms.alarm_state,
                                                    s_last_sent_state.tpms.alarm_state,
                                                    sizeof(s_current_state.tpms.alarm_state)) != 0);

        if (pressures_changed && alarms_changed) {
            hu_protocol_send_tpms(&s_current_state.tpms);
            s_last_sent_state.tpms = s_current_state.tpms;
        } else if (pressures_changed) {
            hu_protocol_send_tpms_numeric(&s_current_state.tpms);
            memcpy(s_last_sent_state.tpms.pressure_bar_deci,
                   s_current_state.tpms.pressure_bar_deci,
                   sizeof(s_current_state.tpms.pressure_bar_deci));
            s_last_sent_state.tpms.valid = true;
        } else if (alarms_changed) {
            hu_protocol_send_tpms_discrete(&s_current_state.tpms);
            memcpy(s_last_sent_state.tpms.alarm_state,
                   s_current_state.tpms.alarm_state,
                   sizeof(s_current_state.tpms.alarm_state));
            s_last_sent_state.tpms.valid = true;
        }
    }

    // Immediately push trip computer updates on new valid trip frames
    if (s_current_state.trip.updated_page != 0) {
        uint8_t page = s_current_state.trip.updated_page;
        s_current_state.trip.updated_page = 0;
        if (page == 1 && s_current_state.trip.instant_valid) {
            hu_protocol_send_trip_instant(&s_current_state.trip);
            s_last_sent_state.trip.instant_fuel_deci = s_current_state.trip.instant_fuel_deci;
            s_last_sent_state.trip.range_km = s_current_state.trip.range_km;
            s_last_sent_state.trip.dest_dist_km = s_current_state.trip.dest_dist_km;
            s_last_sent_state.trip.instant_valid = true;
        } else if (page == 2 && s_current_state.trip.trip1_valid) {
            hu_protocol_send_trip1(&s_current_state.trip);
            s_last_sent_state.trip.trip1_avg_fuel = s_current_state.trip.trip1_avg_fuel;
            s_last_sent_state.trip.trip1_avg_speed = s_current_state.trip.trip1_avg_speed;
            s_last_sent_state.trip.trip1_distance_km = s_current_state.trip.trip1_distance_km;
            s_last_sent_state.trip.trip1_valid = true;
        } else if (page == 3 && s_current_state.trip.trip2_valid) {
            hu_protocol_send_trip2(&s_current_state.trip);
            s_last_sent_state.trip.trip2_avg_fuel = s_current_state.trip.trip2_avg_fuel;
            s_last_sent_state.trip.trip2_avg_speed = s_current_state.trip.trip2_avg_speed;
            s_last_sent_state.trip.trip2_distance_km = s_current_state.trip.trip2_distance_km;
            s_last_sent_state.trip.trip2_valid = true;
        }
    }

    // Immediately push reverse status updates (pull physical BACK wire + notify HU protocol)
    if (s_current_state.reverse_gear != s_last_sent_state.reverse_gear) {
        hal_gpio_write(GPIO_PIN_REVERSE_OUT, s_current_state.reverse_gear);
        hu_protocol_send_reverse(s_current_state.reverse_gear);
        s_last_sent_state.reverse_gear = s_current_state.reverse_gear;
    }

    // Immediately push radar telemetry on updates
    if (s_current_state.radar.updated) {
        s_current_state.radar.updated = false;
        hu_protocol_send_radar(&s_current_state.radar);
        s_last_sent_state.radar = s_current_state.radar;
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

    // Repeat door status frame at 1 Hz (every 1000ms / 10 ticks) while ANY door/trunk/hood is open.
    // Stops repeating as soon as all doors are closed.
    bool any_door_open = s_current_state.doors.door_driver ||
                         s_current_state.doors.door_passenger ||
                         s_current_state.doors.door_rear_left ||
                         s_current_state.doors.door_rear_right ||
                         s_current_state.doors.trunk ||
                         s_current_state.doors.hood;

    if (any_door_open) {
        if (++s_doors_periodic_timer >= 10) {
            s_doors_periodic_timer = 0;
            hu_protocol_send_doors(&s_current_state.doors);
        }
    } else {
        s_doors_periodic_timer = 0;
    }

    // Periodically refresh TPMS telemetry every 30 seconds (300 * 100ms ticks)
    if (++s_tpms_periodic_timer >= 300) {
        s_tpms_periodic_timer = 0;
        if (s_current_state.tpms.valid) {
            hu_protocol_send_tpms(&s_current_state.tpms);
        }
    }

    // Send keep-alive packet to keep Android HU comms link alive
    hu_protocol_send_heartbeat();
}

const vehicle_state_t *can_router_get_state(void) {
    return &s_current_state;
}

bool can_router_reset_trip(uint8_t trip_index) {
    return vehicle_profile_reset_trip(trip_index);
}
