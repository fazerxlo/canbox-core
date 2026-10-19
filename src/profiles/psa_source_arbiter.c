#include "profiles/psa_source_arbiter.h"
#include "profiles/peugeot_407.h"
#include "core/can_router.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_can.h"
#include "hal/hal_uart.h"
#include "hal/hal_system.h"
#include <string.h>

static psa_source_arbiter_t s_arbiter;
static uint8_t s_stalk_rotary_cnt = 0;
static uint32_t s_target_reached_tick_ms = 0;
static uint32_t s_last_station_list_scroll_ms = 0;

void psa_source_arbiter_init(void) {
    memset(&s_arbiter, 0, sizeof(s_arbiter));
    s_arbiter.sync_state = SRC_SYNC_IDLE;
    s_stalk_rotary_cnt = 0;
    s_target_reached_tick_ms = 0;
    s_last_station_list_scroll_ms = 0;
}

const psa_source_arbiter_t *psa_source_arbiter_get_context(void) {
    return &s_arbiter;
}

uint8_t psa_source_mode_to_arbiter(uint8_t source_mode) {
    if (source_mode < 0x20) return PSA_RD4_SRC_TUNER;
    if (source_mode == 0x20 || source_mode == 0x30) return PSA_RD4_SRC_CD;
    if (source_mode == 0x31) return PSA_RD4_SRC_CDC;
    if (source_mode == 0x40 || source_mode == 0x41 || source_mode == 0x33 || source_mode == 0x34) return PSA_RD4_SRC_AUX;
    return 0;
}

#define PSA_RD4_MAX_SWITCH_RETRIES      8
#define PSA_RD4_WAIT_CONFIRM_TIMEOUT_MS 650
#define PSA_RD4_POST_CHANGE_SETTLE_MS   150
#define PSA_RD4_POST_TARGET_SETTLE_MS   400

void psa_source_arbiter_update_stalk_rotary(uint8_t rotary_cnt) {
    s_stalk_rotary_cnt = rotary_cnt;
}

uint8_t psa_source_arbiter_get_stalk_rotary(void) {
    return s_stalk_rotary_cnt;
}

static bool psa_rd4_is_source(uint8_t target_source, const vehicle_radio_t *radio) {
    if (!radio) return false;
    switch (target_source) {
        case PSA_RD4_SRC_TUNER:
            return (radio->source_mode < 0x20 && radio->power_status != 0);
        case PSA_RD4_SRC_CD:
            return ((radio->source_mode == 0x20 || radio->source_mode == 0x30) && radio->power_status != 0);
        case PSA_RD4_SRC_CDC:
            return (radio->source_mode == 0x31 && radio->power_status != 0);
        case PSA_RD4_SRC_AUX:
            return ((radio->source_mode == 0x40 || radio->source_mode == 0x41 ||
                     radio->source_mode == 0x33 || radio->source_mode == 0x34) && radio->power_status != 0);
        default:
            return false;
    }
}

static void psa_source_arbiter_pulse_source(void) {
    if (can_router_is_bus_sleeping()) {
        return;
    }
    can_frame_t frame_press;
    memset(&frame_press, 0, sizeof(frame_press));
    frame_press.id = PSA_CAN_ID_STALK_21F;
    frame_press.dlc = 3;
    frame_press.data[0] = 0x02;
    frame_press.data[1] = s_stalk_rotary_cnt;
    frame_press.data[2] = 0x00;
    hal_can_send(&frame_press);

    hal_delay_ms(STALK_PRESS_DURATION_MS);

    can_frame_t frame_release;
    memset(&frame_release, 0, sizeof(frame_release));
    frame_release.id = PSA_CAN_ID_STALK_21F;
    frame_release.dlc = 3;
    frame_release.data[0] = 0x00;
    frame_release.data[1] = s_stalk_rotary_cnt;
    frame_release.data[2] = 0x00;
    hal_can_send(&frame_release);
}

static void psa_source_arbiter_execute_station_list_scroll(void) {
    if (can_router_is_bus_sleeping()) {
        return;
    }
    uint32_t now = hal_get_tick_ms();
    if (s_last_station_list_scroll_ms != 0 && (now - s_last_station_list_scroll_ms) < 2000) {
        return;
    }
    s_last_station_list_scroll_ms = (now != 0) ? now : 1;

    s_stalk_rotary_cnt = (uint8_t)(s_stalk_rotary_cnt + 1);
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = PSA_CAN_ID_STALK_21F;
    frame.dlc = 3;
    frame.data[0] = 0x00;
    frame.data[1] = s_stalk_rotary_cnt;
    frame.data[2] = 0x00;
    hal_can_send(&frame);

    hal_delay_ms(80);

    s_stalk_rotary_cnt = (uint8_t)(s_stalk_rotary_cnt - 1);
    frame.data[1] = s_stalk_rotary_cnt;
    hal_can_send(&frame);

    hal_delay_ms(80);
}

static void psa_source_arbiter_check_pending_station_list(void) {
    if (s_arbiter.pending_station_list_trigger) {
        s_arbiter.pending_station_list_trigger = false;
        if (s_arbiter.actual_rd4_source == PSA_RD4_SRC_TUNER) {
            psa_source_arbiter_execute_station_list_scroll();
        }
    }
}

void psa_source_arbiter_process_can_0x165(uint8_t source_mode, uint8_t power_status) {
    if (power_status == 0 || source_mode == 0xFF) {
        s_arbiter.actual_rd4_source = 0;
        if (s_arbiter.sync_state == SRC_SYNC_WAIT_CONFIRM || s_arbiter.sync_state == SRC_SYNC_SEEKING) {
            s_arbiter.last_pulse_source = 0;
            s_arbiter.state_timer_ms = hal_get_tick_ms();
        } else {
            s_arbiter.sync_state = SRC_SYNC_IDLE;
            s_arbiter.target_source = 0;
            s_arbiter.retry_count = 0;
            s_arbiter.reported_hu_source = 0;
        }
        s_arbiter.pending_station_list_trigger = false;
        s_arbiter.user_override_non_hu = false;
        s_target_reached_tick_ms = 0;
        return;
    }

    uint8_t new_source = psa_source_mode_to_arbiter(source_mode);
    if (new_source == 0) return;

    if (s_arbiter.reported_hu_source == 0 && s_arbiter.sync_state == SRC_SYNC_IDLE) {
        s_arbiter.actual_rd4_source = new_source;
        s_arbiter.reported_hu_source = new_source;
        s_arbiter.sync_state = SRC_SYNC_IDLE;
        s_target_reached_tick_ms = 0;
        psa_source_arbiter_check_pending_station_list();
        return;
    }

    uint8_t prev_rd4_source = s_arbiter.actual_rd4_source;
    s_arbiter.actual_rd4_source = new_source;

    switch (s_arbiter.sync_state) {
        case SRC_SYNC_IDLE:
            if (new_source != s_arbiter.reported_hu_source) {
                if (s_target_reached_tick_ms != 0) {
                    if ((hal_get_tick_ms() - s_target_reached_tick_ms) < PSA_RD4_POST_TARGET_SETTLE_MS) {
                        break;
                    }
                    s_target_reached_tick_ms = 0;
                }
                s_arbiter.sync_state = SRC_SYNC_PHYSICAL_DEBOUNCE;
                s_arbiter.state_timer_ms = hal_get_tick_ms();
                s_arbiter.user_override_non_hu = (new_source != PSA_RD4_SRC_AUX);
            }
            break;

        case SRC_SYNC_PHYSICAL_DEBOUNCE:
            if (new_source != prev_rd4_source) {
                s_arbiter.state_timer_ms = hal_get_tick_ms();
            }
            s_arbiter.user_override_non_hu = (new_source != PSA_RD4_SRC_AUX);
            break;

        case SRC_SYNC_WAIT_CONFIRM:
        case SRC_SYNC_SEEKING:
            if (new_source == s_arbiter.last_pulse_source) {
                break;
            }

            if (new_source == s_arbiter.target_source) {
                s_arbiter.reported_hu_source = new_source;
                s_arbiter.target_source = 0;
                s_arbiter.retry_count = 0;
                s_arbiter.sync_state = SRC_SYNC_IDLE;
                can_router_push_radio_and_media_state();
                psa_source_arbiter_check_pending_station_list();
                s_target_reached_tick_ms = hal_get_tick_ms();
            } else {
                s_arbiter.retry_count++;
                if (s_arbiter.retry_count >= PSA_RD4_MAX_SWITCH_RETRIES) {
                    s_arbiter.sync_state = SRC_SYNC_IDLE;
                    s_arbiter.reported_hu_source = new_source;
                    s_arbiter.target_source = 0;
                    s_target_reached_tick_ms = 0;
                    can_router_push_radio_and_media_state();
                    psa_source_arbiter_check_pending_station_list();
                } else {
                    hal_delay_ms(PSA_RD4_POST_CHANGE_SETTLE_MS);
                    s_arbiter.last_pulse_source = new_source;
                    psa_source_arbiter_pulse_source();
                    s_arbiter.state_timer_ms = hal_get_tick_ms();
                    s_arbiter.sync_state = SRC_SYNC_WAIT_CONFIRM;
                }
            }
            break;
    }
}

void psa_source_arbiter_switch_source(uint8_t target_source) {
    if (s_arbiter.actual_rd4_source == target_source && s_arbiter.reported_hu_source == target_source) {
        s_arbiter.sync_state = SRC_SYNC_IDLE;
        s_arbiter.target_source = 0;
        can_router_push_radio_and_media_state();
        psa_source_arbiter_check_pending_station_list();
        return;
    }

    if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
        if (s_arbiter.actual_rd4_source == target_source) {
            s_arbiter.reported_hu_source = target_source;
            s_arbiter.target_source = 0;
            s_arbiter.retry_count = 0;
            s_arbiter.sync_state = SRC_SYNC_IDLE;
            can_router_push_radio_and_media_state();
            psa_source_arbiter_check_pending_station_list();
            return;
        }
        if (s_arbiter.sync_state == SRC_SYNC_WAIT_CONFIRM || s_arbiter.sync_state == SRC_SYNC_SEEKING) {
            s_arbiter.target_source = target_source;
            return;
        }
    }

    s_arbiter.target_source = target_source;
    s_arbiter.retry_count = 0;
    s_arbiter.sync_state = SRC_SYNC_WAIT_CONFIRM;
    s_arbiter.last_pulse_source = s_arbiter.actual_rd4_source;
    s_arbiter.state_timer_ms = hal_get_tick_ms();

    psa_source_arbiter_pulse_source();

    const vehicle_state_t *state = can_router_get_state();
    if (state == NULL || !state->media.radio.present) {
        s_arbiter.sync_state = SRC_SYNC_IDLE;
        s_arbiter.reported_hu_source = target_source;
        can_router_push_radio_and_media_state();
        psa_source_arbiter_check_pending_station_list();
        return;
    }

    s_arbiter.state_timer_ms = hal_get_tick_ms();
}

void psa_source_arbiter_trigger_station_list(void) {
    if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
        s_arbiter.pending_station_list_trigger = true;
        return;
    }

    const vehicle_state_t *state = can_router_get_state();
    if (state != NULL && state->media.radio.source_mode < 0x20 && state->media.radio.power_status != 0) {
        psa_source_arbiter_execute_station_list_scroll();
    }
}

void psa_source_arbiter_handle_hu_cmd(uint8_t subcmd, uint8_t param) {
    switch (subcmd) {
        case 0x02: /* Audio Start / Stop */
            if (param == 0x01) {
                if (!s_arbiter.user_override_non_hu && s_arbiter.sync_state != SRC_SYNC_PHYSICAL_DEBOUNCE) {
                    psa_source_arbiter_switch_source(PSA_RD4_SRC_AUX);
                }
            } else if (param == 0x00) {
                s_arbiter.user_override_non_hu = false;
            }
            break;

        case 0x03: /* Switch RD4 Source (0x01=Tuner, 0x02=Internal CD, 0x03/0x04=Line In 1 / AUX 1) */
            s_arbiter.user_override_non_hu = false;
            if (param == 0x01) {
                psa_source_arbiter_switch_source(PSA_RD4_SRC_TUNER);
            } else if (param == 0x02) {
                psa_source_arbiter_switch_source(PSA_RD4_SRC_CD);
            } else if (param == 0x03 || param == 0x04) {
                psa_source_arbiter_switch_source(PSA_RD4_SRC_AUX);
            }
            break;

        default:
            break;
    }
}

void psa_source_arbiter_tick_100ms(void) {
    uint32_t now = hal_get_tick_ms();
    if (s_arbiter.sync_state == SRC_SYNC_PHYSICAL_DEBOUNCE) {
        if ((now - s_arbiter.state_timer_ms) >= 800) {
            s_arbiter.reported_hu_source = s_arbiter.actual_rd4_source;
            s_arbiter.sync_state = SRC_SYNC_IDLE;
            can_router_push_radio_and_media_state();
            psa_source_arbiter_check_pending_station_list();
        }
    } else if (s_arbiter.sync_state == SRC_SYNC_WAIT_CONFIRM) {
        if ((now - s_arbiter.state_timer_ms) >= PSA_RD4_WAIT_CONFIRM_TIMEOUT_MS) {
            const vehicle_state_t *st = can_router_get_state();
            if (st != NULL && st->media.radio.present && s_arbiter.actual_rd4_source == 0) {
                s_arbiter.state_timer_ms = now;
            } else {
                s_arbiter.retry_count++;
                if (s_arbiter.retry_count >= PSA_RD4_MAX_SWITCH_RETRIES) {
                    s_arbiter.sync_state = SRC_SYNC_IDLE;
                    s_arbiter.reported_hu_source = s_arbiter.actual_rd4_source;
                    s_arbiter.target_source = 0;
                    can_router_push_radio_and_media_state();
                    psa_source_arbiter_check_pending_station_list();
                } else {
                    s_arbiter.last_pulse_source = s_arbiter.actual_rd4_source;
                    psa_source_arbiter_pulse_source();
                    s_arbiter.state_timer_ms = hal_get_tick_ms();
                }
            }
        }
    }
}

bool psa_source_arbiter_is_switching_in_progress(void) {
    return (s_arbiter.sync_state != SRC_SYNC_IDLE);
}

bool psa_source_arbiter_is_source_filtered(void) {
    if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
        return true;
    }
    if (s_arbiter.reported_hu_source != 0 &&
        s_arbiter.actual_rd4_source != s_arbiter.reported_hu_source) {
        return true;
    }
    return false;
}

uint8_t psa_source_arbiter_get_target_source(void) {
    return s_arbiter.target_source;
}

uint8_t psa_source_arbiter_get_reported_source(void) {
    return s_arbiter.reported_hu_source;
}

bool psa_source_arbiter_is_target_source_reached(const struct vehicle_radio *radio) {
    if (s_arbiter.sync_state == SRC_SYNC_IDLE) {
        return true;
    }
    return psa_rd4_is_source(s_arbiter.target_source, (const vehicle_radio_t *)radio);
}

