#include "core/can_router.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_gpio.h"
#include <string.h>

static vehicle_state_t s_current_state;
static vehicle_state_t s_last_sent_state;
static uint16_t        s_tpms_periodic_timer = 0;
static uint8_t         s_doors_periodic_timer = 0;

static bool            s_initial_alerts_scanned = false;
static bool            s_bsi_alert_query_pending = false;
static uint8_t         s_bsi_alert_query_timeout_ticks = 0;
static vehicle_alert_item_t s_prev_alert_items[CANBOX_MAX_ACTIVE_ALERTS];
static uint8_t         s_prev_alert_count = 0;

void can_router_init(void) {
    memset(&s_current_state, 0, sizeof(s_current_state));
    memset(&s_last_sent_state, 0, sizeof(s_last_sent_state));
    s_tpms_periodic_timer = 0;
    s_doors_periodic_timer = 0;
    s_initial_alerts_scanned = false;
    s_bsi_alert_query_pending = false;
    s_bsi_alert_query_timeout_ticks = 0;
    s_prev_alert_count = 0;
    memset(s_prev_alert_items, 0, sizeof(s_prev_alert_items));
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
        
        /* Only emit if either current or previous state had an active key */
        if (s_current_state.wheel.active_key != WHEEL_KEY_NONE ||
            s_last_sent_state.wheel.active_key != WHEEL_KEY_NONE) {
            hu_protocol_send_wheel_key(&s_current_state.wheel);
            s_last_sent_state.wheel = s_current_state.wheel;

            /* If this was a momentary pulse key (rotary scroll), emit immediate release */
            if (s_current_state.wheel.press_state != 0 &&
                (s_current_state.wheel.active_key == WHEEL_KEY_SCROLL_UP ||
                 s_current_state.wheel.active_key == WHEEL_KEY_SCROLL_DOWN)) {
                s_current_state.wheel.active_key = WHEEL_KEY_NONE;
                s_current_state.wheel.press_state = 0;
                hu_protocol_send_wheel_key(&s_current_state.wheel);
                s_last_sent_state.wheel = s_current_state.wheel;
            }
        } else {
            s_last_sent_state.wheel = s_current_state.wheel;
        }
    }

    // Immediately push panel/fascia key events on change
    if (s_current_state.panel_key.key_code != s_last_sent_state.panel_key.key_code ||
        s_current_state.panel_key.press_state != s_last_sent_state.panel_key.press_state) {

        hu_protocol_send_panel_key(&s_current_state.panel_key);
        s_last_sent_state.panel_key = s_current_state.panel_key;
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
        if (memcmp(&s_current_state.radar, &s_last_sent_state.radar, sizeof(vehicle_radar_t)) != 0) {
            hu_protocol_send_radar(&s_current_state.radar);
            s_last_sent_state.radar = s_current_state.radar;
        }
    }

    // Immediately push single alert popup or clear
    if (s_current_state.alerts.realtime_updated) {
        s_current_state.alerts.realtime_updated = false;
        if (s_current_state.alerts.realtime_alert.is_active) {
            if (!s_last_sent_state.alerts.realtime_alert.is_active ||
                s_current_state.alerts.realtime_alert.can_alarm_id != s_last_sent_state.alerts.realtime_alert.can_alarm_id ||
                s_current_state.alerts.realtime_alert.door_mask != s_last_sent_state.alerts.realtime_alert.door_mask ||
                s_current_state.alerts.realtime_alert.param_detail != s_last_sent_state.alerts.realtime_alert.param_detail) {
                hu_protocol_send_alert_single(&s_current_state.alerts.realtime_alert);
            }
            s_last_sent_state.alerts.realtime_alert = s_current_state.alerts.realtime_alert;
        } else if (s_last_sent_state.alerts.realtime_alert.is_active) {
            /* Alert was previously active and is now cleared: send dismissal single frame */
            vehicle_alert_item_t dismiss_alert = s_last_sent_state.alerts.realtime_alert;
            dismiss_alert.is_active = false;
            dismiss_alert.display_req = false;
            hu_protocol_send_alert_single(&dismiss_alert);

            s_last_sent_state.alerts.realtime_alert = s_current_state.alerts.realtime_alert;
            hu_protocol_send_alerts_summary(s_current_state.alerts.active_items, s_current_state.alerts.active_count);
        }
    }

    // Handle alert journal updates (edge-triggered single alerts & table synchronization)
    if (s_current_state.alerts.journal_updated) {
        s_current_state.alerts.journal_updated = false;

        if (!s_initial_alerts_scanned) {
            /* Startup / first scan: transmit full 24-byte table once; no popup toasts */
            s_initial_alerts_scanned = true;
            s_prev_alert_count = s_current_state.alerts.active_count;
            memcpy(s_prev_alert_items, s_current_state.alerts.active_items, sizeof(s_prev_alert_items));
            hu_protocol_send_alerts_summary(s_current_state.alerts.active_items, s_current_state.alerts.active_count);
            s_bsi_alert_query_pending = false;
        } else {
            bool has_new = false;
            bool has_cleared = false;

            /* Check for newly active alerts (0 -> 1 transition) */
            for (uint8_t i = 0; i < s_current_state.alerts.active_count; i++) {
                const vehicle_alert_item_t *item = &s_current_state.alerts.active_items[i];
                bool found = false;
                for (uint8_t j = 0; j < s_prev_alert_count; j++) {
                    if (s_prev_alert_items[j].can_alarm_id == item->can_alarm_id) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    /* Edge-triggered new alert event: emit single alert frame (Len = 0x02) */
                    hu_protocol_send_alert_single(item);
                    has_new = true;
                }
            }

            /* Check for cleared alerts (1 -> 0 transition) */
            for (uint8_t i = 0; i < s_prev_alert_count; i++) {
                const vehicle_alert_item_t *item = &s_prev_alert_items[i];
                bool found = false;
                for (uint8_t j = 0; j < s_current_state.alerts.active_count; j++) {
                    if (s_current_state.alerts.active_items[j].can_alarm_id == item->can_alarm_id) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    has_cleared = true;
                }
            }

            bool table_changed = has_new || has_cleared || (s_prev_alert_count != s_current_state.alerts.active_count);

            /* Transmit full 24-byte table ONLY on table change or active query pending */
            if (table_changed || s_bsi_alert_query_pending) {
                hu_protocol_send_alerts_summary(s_current_state.alerts.active_items, s_current_state.alerts.active_count);
                s_bsi_alert_query_pending = false;
            }

            s_prev_alert_count = s_current_state.alerts.active_count;
            memcpy(s_prev_alert_items, s_current_state.alerts.active_items, sizeof(s_prev_alert_items));
        }

        s_last_sent_state.alerts = s_current_state.alerts;
    }
}

void can_router_process_uart_byte(uint8_t byte) {
    hu_protocol_feed_byte(byte);
}

void can_router_periodic_100ms(void) {
    // Broadcast periodic states (Vehicle speed, RPM)
    if (s_current_state.speed_kmh != s_last_sent_state.speed_kmh ||
        s_current_state.rpm != s_last_sent_state.rpm) {

        hu_protocol_send_telemetry(s_current_state.speed_kmh,
                                   s_current_state.rpm,
                                   0);

        s_last_sent_state.speed_kmh = s_current_state.speed_kmh;
        s_last_sent_state.rpm = s_current_state.rpm;
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

    // BSI alert query timeout check (500ms / 5 ticks)
    if (s_bsi_alert_query_pending) {
        if (s_bsi_alert_query_timeout_ticks > 0) {
            s_bsi_alert_query_timeout_ticks--;
        }
        if (s_bsi_alert_query_timeout_ticks == 0) {
            s_bsi_alert_query_pending = false;
            hu_protocol_send_alerts_summary(s_current_state.alerts.active_items, s_current_state.alerts.active_count);
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

bool can_router_query_alert_journal(void) {
    /* Immediately respond with cached summary / empty clearance frame per Section 8 */
    hu_protocol_send_alerts_summary(s_current_state.alerts.active_items, s_current_state.alerts.active_count);
    s_bsi_alert_query_pending = true;
    s_bsi_alert_query_timeout_ticks = 5; /* 5 * 100ms = 500ms */
    return vehicle_profile_query_alerts();
}
