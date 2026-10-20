#include "core/can_router.h"
#include "core/vehicle_profile.h"
#include "profiles/psa_source_arbiter.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_gpio.h"
#include <string.h>

static vehicle_state_t s_current_state;
static vehicle_state_t s_last_sent_state;
static uint16_t        s_tpms_periodic_timer = 0;
static uint8_t         s_doors_periodic_timer = 0;
static uint8_t         s_audio_sync_timer = 0;
static uint16_t        s_bulk_resync_timer = 0;
static bool            s_angle_reverse_synced = false;
static uint16_t        s_media_resync_timer = 0;

static bool            s_initial_alerts_scanned = false;
static bool            s_bsi_alert_query_pending = false;
static uint8_t         s_bsi_alert_query_timeout_ticks = 0;
static vehicle_alert_item_t s_prev_alert_items[CANBOX_MAX_ACTIVE_ALERTS];
static uint8_t         s_prev_alert_count = 0;
static uint16_t        s_can_inactivity_ticks = 0;
static bool            s_can_bus_sleeping = false;

/* 256 bytes bitmask for 11-bit standard CAN IDs (0..2047): O(1) early rejection */
static uint8_t         s_allowed_ids_bitmask[256];

void can_router_rebuild_filter(void) {
    memset(s_allowed_ids_bitmask, 0, sizeof(s_allowed_ids_bitmask));
    const vehicle_profile_t *prof = vehicle_profile_get_active();
    if (!prof || !prof->rules) {
        return;
    }
    for (uint8_t i = 0; i < prof->rule_count; i++) {
        uint32_t id = prof->rules[i].can_id;
        if (id <= 0x7FF) {
            s_allowed_ids_bitmask[id >> 3] |= (uint8_t)(1U << (id & 7));
        }
    }
}

bool can_router_is_id_allowed(uint32_t can_id) {
    if (can_id > 0x7FF) {
        return false;
    }
    return (s_allowed_ids_bitmask[can_id >> 3] & (uint8_t)(1U << (can_id & 7))) != 0;
}

void can_router_init(void) {
    memset(&s_current_state, 0, sizeof(s_current_state));
    memset(&s_last_sent_state, 0, sizeof(s_last_sent_state));
    s_current_state.audio.master_volume = 15;
    s_current_state.audio.bass = 9;
    s_current_state.audio.treble = 9;
    s_current_state.audio.balance = 9;
    s_current_state.audio.fader = 9;
    s_current_state.audio.eq_preset = 0;
    s_current_state.audio.loudness = false;
    s_current_state.audio.speed_volume = false;
    s_current_state.audio.active_mode = 0;
    s_last_sent_state.audio = s_current_state.audio;
    s_tpms_periodic_timer = 0;
    s_doors_periodic_timer = 0;
    s_audio_sync_timer = 0;
    s_bulk_resync_timer = 0;
    s_angle_reverse_synced = false;
    s_media_resync_timer = 0;
    s_initial_alerts_scanned = false;
    s_bsi_alert_query_pending = false;
    s_bsi_alert_query_timeout_ticks = 0;
    s_prev_alert_count = 0;
    memset(s_prev_alert_items, 0, sizeof(s_prev_alert_items));
    s_can_inactivity_ticks = 0;
    s_can_bus_sleeping = false;
    s_current_state.media.radio.source_mode = 0xFF;
    memset(s_current_state.media.radio.station_name, ' ', 8);
    s_current_state.media.radio.station_name[8] = '\0';
    s_last_sent_state.media.radio.source_mode = 0xFF;
    memset(s_last_sent_state.media.radio.station_name, ' ', 8);
    s_last_sent_state.media.radio.station_name[8] = '\0';
    hal_gpio_write(GPIO_PIN_REVERSE_OUT, false);
    hal_gpio_write(GPIO_PIN_ILL_OUT, false);
    hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, false);
    hal_gpio_write(GPIO_PIN_CAN_STBY, false);
    vehicle_profile_init();
    hu_protocol_init();
    psa_source_arbiter_init();
    can_router_rebuild_filter();
}

void can_router_process_can(const can_frame_t *frame) {
    if (!frame) return;

    // Wake transceiver and reset inactivity counter on any CAN activity
    if (s_can_bus_sleeping) {
        hal_gpio_write(GPIO_PIN_CAN_STBY, false);
        s_can_bus_sleeping = false;
    }
    s_can_inactivity_ticks = 0;

    // Reject extended (29-bit) or unmapped standard CAN IDs early (O(1) static bitmask)
    if (frame->is_extended || !can_router_is_id_allowed(frame->id)) {
        return;
    }

    vehicle_profile_process_frame(frame, &s_current_state);

    // Immediately push illumination hardware trigger on lighting change
    bool ill_active = s_current_state.lights.side_light ||
                      s_current_state.lights.headlights ||
                      s_current_state.lights.illumination;
    bool prev_ill   = s_last_sent_state.lights.side_light ||
                      s_last_sent_state.lights.headlights ||
                      s_last_sent_state.lights.illumination;
    if (ill_active != prev_ill) {
        hal_gpio_write(GPIO_PIN_ILL_OUT, ill_active);
    }
    s_last_sent_state.lights = s_current_state.lights;

    // Immediately push switched ACC / HU power on ignition transition
    bool acc_active = (s_current_state.ignition_state == VEHICLE_IGNITION_ON ||
                       s_current_state.ignition_state == VEHICLE_IGNITION_ACC);
    bool prev_acc   = (s_last_sent_state.ignition_state == VEHICLE_IGNITION_ON ||
                       s_last_sent_state.ignition_state == VEHICLE_IGNITION_ACC);
    if (acc_active != prev_acc) {
        hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, acc_active);
        s_last_sent_state.ignition_state = s_current_state.ignition_state;
    }

    // Immediately push reverse hardware trigger on reverse gear change (zero latency)
    if (s_current_state.reverse_gear != s_last_sent_state.reverse_gear) {
        hal_gpio_write(GPIO_PIN_REVERSE_OUT, s_current_state.reverse_gear);
        hu_protocol_send_reverse(s_current_state.reverse_gear);
        s_last_sent_state.reverse_gear = s_current_state.reverse_gear;
    }

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
    if (s_current_state.dirty.trip_page != 0) {
        uint8_t page = s_current_state.dirty.trip_page;
        s_current_state.dirty.trip_page = 0;
        if (page == 1 && s_current_state.trip.instant_valid) {
            if (!s_last_sent_state.trip.instant_valid ||
                s_current_state.trip.instant_fuel_deci != s_last_sent_state.trip.instant_fuel_deci ||
                s_current_state.trip.range_km != s_last_sent_state.trip.range_km ||
                s_current_state.trip.dest_dist_km != s_last_sent_state.trip.dest_dist_km) {
                hu_protocol_send_trip_instant(&s_current_state.trip);
                s_last_sent_state.trip.instant_fuel_deci = s_current_state.trip.instant_fuel_deci;
                s_last_sent_state.trip.range_km = s_current_state.trip.range_km;
                s_last_sent_state.trip.dest_dist_km = s_current_state.trip.dest_dist_km;
                s_last_sent_state.trip.instant_valid = true;
            }
        } else if (page == 2 && s_current_state.trip.trip1_valid) {
            if (!s_last_sent_state.trip.trip1_valid ||
                s_current_state.trip.trip1_avg_fuel != s_last_sent_state.trip.trip1_avg_fuel ||
                s_current_state.trip.trip1_avg_speed != s_last_sent_state.trip.trip1_avg_speed ||
                s_current_state.trip.trip1_distance_km != s_last_sent_state.trip.trip1_distance_km) {
                hu_protocol_send_trip1(&s_current_state.trip);
                s_last_sent_state.trip.trip1_avg_fuel = s_current_state.trip.trip1_avg_fuel;
                s_last_sent_state.trip.trip1_avg_speed = s_current_state.trip.trip1_avg_speed;
                s_last_sent_state.trip.trip1_distance_km = s_current_state.trip.trip1_distance_km;
                s_last_sent_state.trip.trip1_valid = true;
            }
        } else if (page == 3 && s_current_state.trip.trip2_valid) {
            if (!s_last_sent_state.trip.trip2_valid ||
                s_current_state.trip.trip2_avg_fuel != s_last_sent_state.trip.trip2_avg_fuel ||
                s_current_state.trip.trip2_avg_speed != s_last_sent_state.trip.trip2_avg_speed ||
                s_current_state.trip.trip2_distance_km != s_last_sent_state.trip.trip2_distance_km) {
                hu_protocol_send_trip2(&s_current_state.trip);
                s_last_sent_state.trip.trip2_avg_fuel = s_current_state.trip.trip2_avg_fuel;
                s_last_sent_state.trip.trip2_avg_speed = s_current_state.trip.trip2_avg_speed;
                s_last_sent_state.trip.trip2_distance_km = s_current_state.trip.trip2_distance_km;
                s_last_sent_state.trip.trip2_valid = true;
            }
        }
    }

    // Immediately push reverse status updates (pull physical BACK wire + notify HU protocol)
    if (s_current_state.reverse_gear != s_last_sent_state.reverse_gear) {
        hal_gpio_write(GPIO_PIN_REVERSE_OUT, s_current_state.reverse_gear);
        hu_protocol_send_reverse(s_current_state.reverse_gear);
        s_last_sent_state.reverse_gear = s_current_state.reverse_gear;
    }

    // Push dynamic trajectory steering angle on change, only while reverse gear is engaged
    if (s_current_state.reverse_gear) {
        if (s_current_state.steering_angle_deg != s_last_sent_state.steering_angle_deg ||
            !s_angle_reverse_synced) {
            hu_protocol_send_steering_angle(s_current_state.steering_angle_deg);
            s_last_sent_state.steering_angle_deg = s_current_state.steering_angle_deg;
            s_angle_reverse_synced = true;
        }
    } else {
        s_angle_reverse_synced = false;
    }

    // Immediately push radar telemetry on updates
    if (s_current_state.dirty.radar) {
        s_current_state.dirty.radar = false;
        if (memcmp(&s_current_state.radar, &s_last_sent_state.radar, sizeof(vehicle_radar_t)) != 0) {
            hu_protocol_send_radar(&s_current_state.radar);
            s_last_sent_state.radar = s_current_state.radar;
        }
    }

    // Immediately push single alert popup or clear
    if (s_current_state.dirty.alert_realtime) {
        s_current_state.dirty.alert_realtime = false;
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
    if (s_current_state.dirty.alert_journal) {
        s_current_state.dirty.alert_journal = false;

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

    // Immediately push audio settings updates on change
    if (s_current_state.dirty.audio) {
        s_current_state.dirty.audio = false;
        if (memcmp(&s_current_state.audio, &s_last_sent_state.audio, sizeof(vehicle_audio_settings_t)) != 0) {
            hu_protocol_send_audio_settings(&s_current_state.audio);
            s_last_sent_state.audio = s_current_state.audio;
        }
    }

    // Immediately push radio tuner & source updates on change
    if (s_current_state.dirty.radio) {
        s_current_state.dirty.radio = false;

        bool radio_changed = false;
        if (s_current_state.media.radio.source_mode >= 0x20) {
            /* External sources (AUX, CD, CDC) or OFF (0xFF):
             * Only present (first 0x165 arrival), source_mode, or power_status changes trigger 0x84.
             * Background tuner changes (0x225/0x2A5) are cached without emitting 0x84. */
            if (s_current_state.media.radio.present != s_last_sent_state.media.radio.present ||
                s_current_state.media.radio.source_mode != s_last_sent_state.media.radio.source_mode ||
                s_current_state.media.radio.power_status != s_last_sent_state.media.radio.power_status) {
                radio_changed = true;
            }
        } else {
            /* Radio tuner mode (< 0x20): any change in frequency, band, preset, indicators, name, power, or source */
            if (memcmp(&s_current_state.media.radio, &s_last_sent_state.media.radio, sizeof(vehicle_radio_t)) != 0) {
                radio_changed = true;
            }
        }

        if (radio_changed) {
            if (psa_source_arbiter_is_source_filtered() || s_current_state.media.radio.power_status == 3) {
                /* Suppress intermediate 0x84 broadcast while RD4 source switch, debounce, or mute/transition is in progress */
            } else {
                uint8_t old_src = s_last_sent_state.media.radio.source_mode;
                uint8_t new_src = s_current_state.media.radio.source_mode;

                /* If transitioning away from CD (0x20/0x30), notify HU that CD playback has stopped */
                bool old_is_cd = (old_src == 0x20 || old_src == 0x30);
                bool new_is_cd = (new_src == 0x20 || new_src == 0x30);
                if (old_is_cd && !new_is_cd) {
                    s_current_state.media.cdc.play_status = 0x01; /* Pause/Stop */
                    hu_protocol_send_media_state(&s_current_state.media.cdc);
                    s_last_sent_state.media.cdc = s_current_state.media.cdc;
                }

                hu_protocol_send_radio_state(&s_current_state.media.radio);

                /* If transitioning to radio mode, also refresh presets and radio text */
                if (old_src >= 0x20 && new_src < 0x20) {
                    hu_protocol_send_radio_presets(&s_current_state.media.radio);
                    s_current_state.dirty.radio_preset = false;
                    if (s_current_state.media.radio.radio_text_len > 0) {
                        hu_protocol_send_radio_text(s_current_state.media.radio.radio_text, s_current_state.media.radio.radio_text_len);
                        s_current_state.dirty.radio_text = false;
                    }
                }

                s_last_sent_state.media.radio = s_current_state.media.radio;
            }
        } else if (s_current_state.media.radio.source_mode >= 0x20) {
            /* Keep cached radio state aligned without sending serial frames */
            s_last_sent_state.media.radio = s_current_state.media.radio;
        }
    }

    // Immediately push radio preset list updates on change (only if currently in radio tuner mode)
    if (s_current_state.dirty.radio_preset) {
        s_current_state.dirty.radio_preset = false;
        if (!psa_source_arbiter_is_source_filtered() &&
            s_current_state.media.radio.source_mode < 0x20 && s_current_state.media.radio.power_status != 0) {
            hu_protocol_send_radio_presets(&s_current_state.media.radio);
        }
        memcpy(s_last_sent_state.media.radio.preset_freqs, s_current_state.media.radio.preset_freqs, sizeof(s_last_sent_state.media.radio.preset_freqs));
        memcpy(s_last_sent_state.media.radio.preset_names, s_current_state.media.radio.preset_names, sizeof(s_last_sent_state.media.radio.preset_names));
    }

    // Immediately push dynamic RDS RadioText on change (only if currently in radio tuner mode)
    if (s_current_state.dirty.radio_text) {
        s_current_state.dirty.radio_text = false;
        if (!psa_source_arbiter_is_source_filtered() &&
            s_current_state.media.radio.source_mode < 0x20 && s_current_state.media.radio.power_status != 0) {
            hu_protocol_send_radio_text(s_current_state.media.radio.radio_text, s_current_state.media.radio.radio_text_len);
        }
        s_last_sent_state.media.radio.radio_text_len = s_current_state.media.radio.radio_text_len;
        memcpy(s_last_sent_state.media.radio.radio_text, s_current_state.media.radio.radio_text, sizeof(s_last_sent_state.media.radio.radio_text));
    }

    // Immediately push CD / CD Changer (CDC) updates on change (only when in CD mode)
    if (s_current_state.dirty.cdc) {
        s_current_state.dirty.cdc = false;
        uint8_t src = s_current_state.media.radio.source_mode;
        bool is_cd_source = (src == 0x20 || src == 0x30 ||
                             (!s_current_state.media.radio.present && s_current_state.media.cdc.active_disc != 0));
        if (is_cd_source && memcmp(&s_current_state.media.cdc, &s_last_sent_state.media.cdc, sizeof(vehicle_cdc_t)) != 0) {
            if (psa_source_arbiter_is_source_filtered()) {
                /* Suppress intermediate 0x97 broadcast while RD4 source switch or debounce is in progress */
            } else {
                hu_protocol_send_media_state(&s_current_state.media.cdc);
                s_last_sent_state.media.cdc = s_current_state.media.cdc;
            }
        }
    }
}

void can_router_push_radio_and_media_state(void) {
    if (!s_current_state.media.radio.present) {
        if (psa_source_arbiter_get_reported_source() == PSA_RD4_SRC_AUX) {
            vehicle_radio_t aux_radio;
            memset(&aux_radio, 0, sizeof(aux_radio));
            aux_radio.source_mode = 0x40;
            aux_radio.power_status = 0x01;
            hu_protocol_send_radio_state(&aux_radio);
            return;
        }
    }

    uint8_t old_src = s_last_sent_state.media.radio.source_mode;
    uint8_t new_src = s_current_state.media.radio.source_mode;

    /* If transitioning away from CD (0x20/0x30), notify HU that CD playback has stopped */
    bool old_is_cd = (old_src == 0x20 || old_src == 0x30);
    bool new_is_cd = (new_src == 0x20 || new_src == 0x30);
    if (old_is_cd && !new_is_cd) {
        s_current_state.media.cdc.play_status = 0x01; /* Pause/Stop */
        hu_protocol_send_media_state(&s_current_state.media.cdc);
        s_last_sent_state.media.cdc = s_current_state.media.cdc;
    }

    hu_protocol_send_radio_state(&s_current_state.media.radio);
    s_last_sent_state.media.radio = s_current_state.media.radio;

    if (s_current_state.media.radio.source_mode < 0x20 && s_current_state.media.radio.power_status != 0) {
        hu_protocol_send_radio_presets(&s_current_state.media.radio);
        s_current_state.dirty.radio_preset = false;
        if (s_current_state.media.radio.radio_text_len > 0) {
            hu_protocol_send_radio_text(s_current_state.media.radio.radio_text, s_current_state.media.radio.radio_text_len);
            s_current_state.dirty.radio_text = false;
        }
    } else if (s_current_state.media.radio.source_mode == 0x20 ||
               s_current_state.media.radio.source_mode == 0x30) {
        hu_protocol_send_media_state(&s_current_state.media.cdc);
        s_last_sent_state.media.cdc = s_current_state.media.cdc;
    }
}

void can_router_process_uart_byte(uint8_t byte) {
    hu_protocol_feed_byte(byte);
}

void can_router_periodic_100ms(void) {
    // Bus Inactivity & Safe Sleep Watchdog (3.0s / 30 ticks of total CAN silence)
    if (++s_can_inactivity_ticks >= 30) {
        s_can_inactivity_ticks = 30; /* Clamp */
        if (!s_can_bus_sleeping) {
            s_current_state.ignition_on = false;
            s_current_state.media.radio.power_status = 0x00;
            s_current_state.media.radio.source_mode = 0xFF;
            s_current_state.media.radio.sleep = false;
            s_current_state.economy_mode = false;
            s_current_state.ignition_state = VEHICLE_IGNITION_OFF;
            s_last_sent_state.ignition_state = VEHICLE_IGNITION_OFF;
            hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, false);
            hal_gpio_write(GPIO_PIN_ILL_OUT, false);
            hal_gpio_write(GPIO_PIN_REVERSE_OUT, false);
            hal_gpio_write(GPIO_PIN_CAN_STBY, true); /* Silent/Standby mode */
            s_audio_sync_timer = 0;
            s_can_bus_sleeping = true;
        }
    }

    // When bus is sleeping, halt all periodic transmissions and heartbeats
    if (s_can_bus_sleeping) {
        return;
    }

    // Periodic RD4 Source Arbiter tick (100ms)
    psa_source_arbiter_tick_100ms();

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

    // Periodic audio settings transmission:
    // While inside an active DSP menu (active_mode >= 2), transmit every 1000ms (10 ticks)
    // with the current active mode so the Head Unit display stays active and refreshed.
    // When no menu is open (active_mode == 0), suppress 1 Hz periodic transmission to prevent
    // unnecessary serial traffic (passive sync is handled via 60s bulk resync).
    if (s_current_state.audio.valid) {
        if (s_current_state.audio.active_mode >= 2) {
            if (++s_audio_sync_timer >= 10) {
                s_audio_sync_timer = 0;
                hu_protocol_send_audio_settings(&s_current_state.audio);
            }
        } else {
            s_audio_sync_timer = 0;
        }
    }

    // Periodic bulk state resync at 60 seconds (600 * 100ms = 60s)
    // Ensures Android Head Unit recovers state after a silent reboot or app restart
    if (++s_bulk_resync_timer >= 600) {
        s_bulk_resync_timer = 0;
        hu_protocol_send_doors(&s_current_state.doors);
        if (s_current_state.audio.valid && s_current_state.audio.active_mode == 0) {
            vehicle_audio_settings_t sync_audio = s_current_state.audio;
            sync_audio.active_mode = 0;
            hu_protocol_send_audio_settings(&sync_audio);
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

    // Periodic slow resync for Radio & Media states (60 seconds / 600 ticks)
    if (++s_media_resync_timer >= 600) {
        s_media_resync_timer = 0;
        if (!psa_source_arbiter_is_source_filtered() && s_current_state.media.radio.power_status != 0) {
            hu_protocol_send_radio_state(&s_current_state.media.radio);
            if (s_current_state.media.radio.source_mode < 0x20) {
                hu_protocol_send_radio_presets(&s_current_state.media.radio);
                if (s_current_state.media.radio.radio_text_len > 0) {
                    hu_protocol_send_radio_text(s_current_state.media.radio.radio_text, s_current_state.media.radio.radio_text_len);
                }
            } else if (s_current_state.media.radio.source_mode == 0x20 ||
                       s_current_state.media.radio.source_mode == 0x30) {
                hu_protocol_send_media_state(&s_current_state.media.cdc);
            }
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

bool can_router_is_bus_sleeping(void) {
    return s_can_bus_sleeping;
}

void can_router_notify_wake(void) {
    if (s_can_bus_sleeping) {
        hal_gpio_write(GPIO_PIN_CAN_STBY, false);
        s_can_bus_sleeping = false;
    }
    s_can_inactivity_ticks = 0;
}
