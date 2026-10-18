#include "profiles/psa_virtual_cdc.h"
#include "profiles/peugeot_407.h"
#include "core/can_router.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_can.h"
#include "hal/hal_uart.h"
#include "hal/hal_system.h"
#include <string.h>

static psa_virtual_cdc_t s_cdc;
static psa_source_arbiter_t s_arbiter;
static uint8_t s_stalk_rotary_cnt = 0;
static uint32_t s_target_reached_tick_ms = 0;
static uint32_t s_last_station_list_scroll_ms = 0;

void psa_virtual_cdc_init(void) {
    memset(&s_cdc, 0, sizeof(s_cdc));
    s_cdc.enabled = true;              /* Continuously broadcast CDC presence by default */
    s_cdc.playback_active = false;      /* Standby initially (no audio streaming) */
    s_cdc.radio_enabled = true;        /* Assume radio enabled by default */
    s_cdc.active_disc = 1;              /* Disc 1 default */
    s_cdc.track_num = 1;
    s_cdc.total_tracks = 99;            /* TRACK_MAX = 99 in CDCEmu */
    s_cdc.elapsed_min = 0;
    s_cdc.elapsed_sec = 0;
    s_cdc.status_icon = 3;              /* STATUS_ICON_PLAY1 = 3 */
    s_cdc.intro_mode = false;
    s_cdc.random_mode = false;
    s_cdc.repeat_mode = false;
    s_cdc.timer_ticks_100ms = 0;
    s_stalk_rotary_cnt = 0;
    s_target_reached_tick_ms = 0;
    s_last_station_list_scroll_ms = 0;

    memset(&s_arbiter, 0, sizeof(s_arbiter));
    s_arbiter.sync_state = SRC_SYNC_IDLE;
}

void psa_virtual_cdc_set_enabled(bool enabled) {
    s_cdc.enabled = enabled;
    if (!enabled) {
        s_cdc.playback_active = false;
    }
}

bool psa_virtual_cdc_is_active(void) {
    return s_cdc.playback_active;
}

bool psa_virtual_cdc_is_presence_enabled(void) {
    return s_cdc.enabled;
}

const psa_virtual_cdc_t *psa_virtual_cdc_get_context(void) {
    return &s_cdc;
}

const psa_source_arbiter_t *psa_virtual_cdc_get_arbiter(void) {
    return &s_arbiter;
}

uint8_t psa_source_mode_to_arbiter(uint8_t source_mode) {
    if (source_mode < 0x20) return PSA_RD4_SRC_TUNER;
    if (source_mode == 0x20 || source_mode == 0x30) return PSA_RD4_SRC_CD;
    if (source_mode == 0x31) return PSA_RD4_SRC_CDC;
    if (source_mode == 0x40 || source_mode == 0x41 || source_mode == 0x33 || source_mode == 0x34) return PSA_RD4_SRC_AUX;
    return 0;
}

void psa_virtual_cdc_process_can_0x131(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 5) return;

    /* Byte 0 Bit 7: radio_enabled */
    s_cdc.radio_enabled = (data[0] & 0x80) != 0;

    /* Byte 0 Bit 1: radio_playing (RD4 currently selected CDC as source) */
    bool rd4_playing_cdc = (data[0] & 0x02) != 0;
    if (rd4_playing_cdc && !s_cdc.playback_active && s_cdc.status_icon != 2) {
        s_cdc.playback_active = true;
        s_cdc.status_icon = 3; /* STATUS_ICON_PLAY1 */
    }

    /* Byte 2: Go to disk (if non-zero) */
    if (data[2] >= 1 && data[2] <= 6) {
        s_cdc.active_disc = data[2];
        s_cdc.track_num = 1;
        s_cdc.elapsed_min = 0;
        s_cdc.elapsed_sec = 0;
    }

    /* Byte 4: Go to track (if non-zero) */
    if (data[4] >= 1 && data[4] <= 99) {
        s_cdc.track_num = data[4];
        s_cdc.elapsed_min = 0;
        s_cdc.elapsed_sec = 0;
    }

    /* Byte 0 Bit 5: Next track */
    if (data[0] & 0x20) {
        if (s_cdc.track_num < s_cdc.total_tracks) {
            s_cdc.track_num++;
        }
        s_cdc.elapsed_min = 0;
        s_cdc.elapsed_sec = 0;
    }

    /* Byte 0 Bit 6: Previous track */
    if (data[0] & 0x40) {
        if (s_cdc.track_num > 1) {
            s_cdc.track_num--;
        }
        s_cdc.elapsed_min = 0;
        s_cdc.elapsed_sec = 0;
    }

    /* Byte 1: Repeat (Bit 7), Random (Bit 0) */
    s_cdc.repeat_mode = (data[1] & 0x80) != 0;
    s_cdc.random_mode = (data[1] & 0x01) != 0;
}

#define PSA_RD4_MAX_SWITCH_RETRIES      8
#define PSA_RD4_WAIT_CONFIRM_TIMEOUT_MS 650
#define PSA_RD4_POST_CHANGE_SETTLE_MS   150
#define PSA_RD4_POST_TARGET_SETTLE_MS   400

void psa_virtual_cdc_update_stalk_rotary(uint8_t rotary_cnt) {
    s_stalk_rotary_cnt = rotary_cnt;
}

uint8_t psa_virtual_cdc_get_stalk_rotary(void) {
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

static void psa_virtual_cdc_pulse_source(void) {
    /* Press Frame: Transmit 21F#02<cnt>00 strictly ONCE for STALK_PRESS_DURATION_MS (50 ms).
     * Never transmit multiple press frames within one pulse, as any intervening
     * COM2000 idle frame (21F#00<cnt>00) would cause RD4 to register a double-click! */
    can_frame_t frame_press;
    memset(&frame_press, 0, sizeof(frame_press));
    frame_press.id = PSA_CAN_ID_STALK_21F;
    frame_press.dlc = 3;
    frame_press.data[0] = 0x02;
    frame_press.data[1] = s_stalk_rotary_cnt;
    frame_press.data[2] = 0x00;
    hal_can_send(&frame_press);

    hal_delay_ms(STALK_PRESS_DURATION_MS);

    /* Release Frame: Transmit 21F#00<cnt>00 (Byte 0 = 0x00 Idle) */
    can_frame_t frame_release;
    memset(&frame_release, 0, sizeof(frame_release));
    frame_release.id = PSA_CAN_ID_STALK_21F;
    frame_release.dlc = 3;
    frame_release.data[0] = 0x00;
    frame_release.data[1] = s_stalk_rotary_cnt;
    frame_release.data[2] = 0x00;
    hal_can_send(&frame_release);
}

static void psa_virtual_cdc_execute_station_list_scroll(void) {
    uint32_t now = hal_get_tick_ms();
    if (s_last_station_list_scroll_ms != 0 && (now - s_last_station_list_scroll_ms) < 2000) {
        return; /* Debounce repeated 0xA1 bursts from HU */
    }
    s_last_station_list_scroll_ms = (now != 0) ? now : 1;

    /* Step 1 (Scroll UP): <cnt> = (<cnt> + 1) & 0xFF */
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

    /* Step 2 (Scroll DOWN - Restore): <cnt> = (<cnt> - 1) & 0xFF */
    s_stalk_rotary_cnt = (uint8_t)(s_stalk_rotary_cnt - 1);
    frame.data[1] = s_stalk_rotary_cnt;
    hal_can_send(&frame);

    hal_delay_ms(80);
}

static void psa_virtual_cdc_check_pending_station_list(void) {
    if (s_arbiter.pending_station_list_trigger) {
        s_arbiter.pending_station_list_trigger = false;
        if (s_arbiter.actual_rd4_source == PSA_RD4_SRC_TUNER) {
            psa_virtual_cdc_execute_station_list_scroll();
        }
    }
}

void psa_virtual_cdc_process_can_0x165(uint8_t source_mode, uint8_t power_status) {
    if (power_status == 0 || source_mode == 0xFF) {
        s_arbiter.actual_rd4_source = 0;
        if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
            s_arbiter.sync_state = SRC_SYNC_IDLE;
            s_arbiter.target_source = 0;
            s_arbiter.retry_count = 0;
        }
        s_arbiter.reported_hu_source = 0;
        s_arbiter.pending_station_list_trigger = false;
        s_target_reached_tick_ms = 0;
        s_cdc.radio_enabled = false;
        return;
    }

    /* RD4 is ON: ensure CDC emulation is always enabled and present on CAN */
    s_cdc.radio_enabled = true;
    s_cdc.enabled = true;

    uint8_t new_source = psa_source_mode_to_arbiter(source_mode);
    if (new_source == 0) return;

    /* When RD4 is active on another source, CDC remains enabled in PAUSE state */
    if (new_source != PSA_RD4_SRC_CDC && s_arbiter.sync_state == SRC_SYNC_IDLE) {
        s_cdc.playback_active = false;
        s_cdc.status_icon = 2; /* STATUS_ICON_PAUSE */
    }

    /* First arrival after boot: sync immediately without debounce delay */
    if (s_arbiter.reported_hu_source == 0) {
        s_arbiter.actual_rd4_source = new_source;
        s_arbiter.reported_hu_source = new_source;
        s_arbiter.sync_state = SRC_SYNC_IDLE;
        s_target_reached_tick_ms = 0;
        psa_virtual_cdc_check_pending_station_list();
        return;
    }

    uint8_t prev_rd4_source = s_arbiter.actual_rd4_source;
    s_arbiter.actual_rd4_source = new_source;

    switch (s_arbiter.sync_state) {
        case SRC_SYNC_IDLE:
            if (new_source != s_arbiter.reported_hu_source) {
                /* Post-target settle guard (400 ms): in-flight RD4 step cannot trigger debounce */
                if (s_target_reached_tick_ms != 0) {
                    if ((hal_get_tick_ms() - s_target_reached_tick_ms) < PSA_RD4_POST_TARGET_SETTLE_MS) {
                        break;
                    }
                    s_target_reached_tick_ms = 0;
                }
                /* Rule F3: Physical Button Settle Filter - enter 800ms debounce */
                s_arbiter.sync_state = SRC_SYNC_PHYSICAL_DEBOUNCE;
                s_arbiter.state_timer_ms = hal_get_tick_ms();
                if (new_source == PSA_RD4_SRC_TUNER) {
                    s_arbiter.pending_station_list_trigger = true;
                }
            }
            break;

        case SRC_SYNC_PHYSICAL_DEBOUNCE:
            /* Reset 800ms debounce timer ONLY if the source actually changed again */
            if (new_source != prev_rd4_source) {
                s_arbiter.state_timer_ms = hal_get_tick_ms();
                if (new_source == PSA_RD4_SRC_TUNER) {
                    s_arbiter.pending_station_list_trigger = true;
                }
            }
            break;

        case SRC_SYNC_WAIT_CONFIRM:
        case SRC_SYNC_SEEKING:
            /* Ignore periodic 0x165 frame if RD4 hasn't moved from the source where we pulsed */
            if (new_source == s_arbiter.last_pulse_source) {
                break;
            }

            /* Rule F2: Closed-Loop Pulse Stepping */
            if (new_source == s_arbiter.target_source) {
                /* Target reached! */
                s_arbiter.reported_hu_source = new_source;
                s_arbiter.target_source = 0;
                s_arbiter.retry_count = 0;
                s_arbiter.sync_state = SRC_SYNC_IDLE;
                can_router_push_radio_and_media_state();
                psa_virtual_cdc_check_pending_station_list();
                s_target_reached_tick_ms = hal_get_tick_ms();
            } else {
                /* Intermediate source reached: wait settle time, then send ONE pulse for the next step */
                s_arbiter.retry_count++;
                if (s_arbiter.retry_count >= PSA_RD4_MAX_SWITCH_RETRIES) {
                    /* Safety Fallback: abort seeking, accept current source */
                    s_arbiter.sync_state = SRC_SYNC_IDLE;
                    s_arbiter.reported_hu_source = new_source;
                    s_arbiter.target_source = 0;
                    s_target_reached_tick_ms = 0;
                    can_router_push_radio_and_media_state();
                    psa_virtual_cdc_check_pending_station_list();
                } else {
                    hal_delay_ms(PSA_RD4_POST_CHANGE_SETTLE_MS);
                    s_arbiter.last_pulse_source = new_source;
                    psa_virtual_cdc_pulse_source();
                    /* Record timestamp AFTER pulse completes so tick_100ms waits a full 450 ms */
                    s_arbiter.state_timer_ms = hal_get_tick_ms();
                    s_arbiter.sync_state = SRC_SYNC_WAIT_CONFIRM;
                }
            }
            break;
    }
}

void psa_virtual_cdc_switch_source(uint8_t target_source) {
    if (target_source == PSA_RD4_SRC_TUNER) {
        s_arbiter.pending_station_list_trigger = true;
    }

    /* Rule F1: If already on target source, confirm immediately */
    if (s_arbiter.actual_rd4_source == target_source && s_arbiter.reported_hu_source == target_source) {
        s_arbiter.sync_state = SRC_SYNC_IDLE;
        s_arbiter.target_source = 0;
        can_router_push_radio_and_media_state();
        psa_virtual_cdc_check_pending_station_list();
        return;
    }

    if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
        if (s_arbiter.actual_rd4_source == target_source) {
            s_arbiter.reported_hu_source = target_source;
            s_arbiter.target_source = 0;
            s_arbiter.retry_count = 0;
            s_arbiter.sync_state = SRC_SYNC_IDLE;
            can_router_push_radio_and_media_state();
            psa_virtual_cdc_check_pending_station_list();
            return;
        }
        s_arbiter.target_source = target_source;
        return;
    }

    /* Rule F1 & F4: Target acquisition & Initiate pulse sequence */
    s_arbiter.target_source = target_source;
    s_arbiter.retry_count = 0;
    s_arbiter.sync_state = SRC_SYNC_WAIT_CONFIRM;
    s_arbiter.last_pulse_source = s_arbiter.actual_rd4_source;
    s_arbiter.state_timer_ms = hal_get_tick_ms();

    /* Fire first 0x21F pulse */
    psa_virtual_cdc_pulse_source();

    const vehicle_state_t *state = can_router_get_state();
    if (state == NULL || !state->media.radio.present) {
        s_arbiter.sync_state = SRC_SYNC_IDLE;
        s_arbiter.reported_hu_source = target_source;
        can_router_push_radio_and_media_state();
        psa_virtual_cdc_check_pending_station_list();
        return;
    }

    /* Record timer AFTER pulse finishes */
    s_arbiter.state_timer_ms = hal_get_tick_ms();
}

void psa_virtual_cdc_trigger_station_list(void) {
    if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
        s_arbiter.pending_station_list_trigger = true;
        return;
    }

    const vehicle_state_t *state = can_router_get_state();
    if (state != NULL && state->media.radio.source_mode < 0x20 && state->media.radio.power_status != 0) {
        psa_virtual_cdc_execute_station_list_scroll();
    }
}

void psa_virtual_cdc_set_state(bool active) {
    if (active) {
        s_cdc.playback_active = true;
        s_cdc.status_icon = 3;         /* STATUS_ICON_PLAY1 = 3 */
        psa_virtual_cdc_switch_source(PSA_RD4_SRC_CDC);
    } else {
        /* Pause / Standby: keep CDC presence alive */
        s_cdc.playback_active = false;
        s_cdc.status_icon = 2;         /* STATUS_ICON_PAUSE = 2 */
    }
}

void psa_virtual_cdc_handle_hu_cmd(uint8_t subcmd, uint8_t param) {
    switch (subcmd) {
        case 0x01: /* Enable / Disable Virtual CDC Emulation */
            psa_virtual_cdc_set_enabled(param != 0);
            break;

        case 0x02: /* Audio Start / Stop */
            if (param == 0x01) {
                psa_virtual_cdc_set_state(true);
            } else if (param == 0x00) {
                psa_virtual_cdc_set_state(false);
            }
            break;

        case 0x03: /* Switch RD4 Source (0x01 = Tuner, 0x02 = Internal CD, 0x03 = CDC, 0x04 = AUX) */
            if (param == 0x01) {
                s_cdc.playback_active = false;
                s_cdc.status_icon = 2; /* PAUSE */
                psa_virtual_cdc_switch_source(PSA_RD4_SRC_TUNER);
            } else if (param == 0x02) {
                s_cdc.playback_active = false;
                s_cdc.status_icon = 2; /* PAUSE */
                psa_virtual_cdc_switch_source(PSA_RD4_SRC_CD);
            } else if (param == 0x03) {
                psa_virtual_cdc_switch_source(PSA_RD4_SRC_CDC);
            } else if (param == 0x04) {
                s_cdc.playback_active = false;
                s_cdc.status_icon = 2; /* PAUSE */
                psa_virtual_cdc_switch_source(PSA_RD4_SRC_AUX);
            }
            break;

        case 0x0E: /* Select Disc Slot (1..6) */
            if (param >= 1 && param <= 6) {
                s_cdc.active_disc = param;
            } else if (param == 0) {
                s_cdc.active_disc = 1;
            }
            break;

        case 0x0F: /* Set Elapsed Seconds */
            if (param <= 59) {
                s_cdc.elapsed_sec = param;
            }
            break;

        case 0x10: /* Set Track Number */
            if (param >= 1 && param <= 99) {
                s_cdc.track_num = param;
            }
            break;

        default:
            break;
    }
}

void psa_virtual_cdc_tick_100ms(void) {
    /* --------------------------------------------------------------------------
     * 0. State Arbiter Timeouts & Settle Checking
     * -------------------------------------------------------------------------- */
    uint32_t now = hal_get_tick_ms();
    if (s_arbiter.sync_state == SRC_SYNC_PHYSICAL_DEBOUNCE) {
        if ((now - s_arbiter.state_timer_ms) >= 800) {
            /* Rule F3: 800 ms continuously settled on actual_rd4_source */
            s_arbiter.reported_hu_source = s_arbiter.actual_rd4_source;
            s_arbiter.sync_state = SRC_SYNC_IDLE;
            can_router_push_radio_and_media_state();
            psa_virtual_cdc_check_pending_station_list();
        }
    } else if (s_arbiter.sync_state == SRC_SYNC_WAIT_CONFIRM) {
        if ((now - s_arbiter.state_timer_ms) >= PSA_RD4_WAIT_CONFIRM_TIMEOUT_MS) {
            /* Timeout: RD4 missed the pulse (e.g. bus collision) - retry */
            s_arbiter.retry_count++;
            if (s_arbiter.retry_count >= PSA_RD4_MAX_SWITCH_RETRIES) {
                s_arbiter.sync_state = SRC_SYNC_IDLE;
                s_arbiter.reported_hu_source = s_arbiter.actual_rd4_source;
                s_arbiter.target_source = 0;
                can_router_push_radio_and_media_state();
                psa_virtual_cdc_check_pending_station_list();
            } else {
                s_arbiter.last_pulse_source = s_arbiter.actual_rd4_source;
                psa_virtual_cdc_pulse_source();
                s_arbiter.state_timer_ms = hal_get_tick_ms();
            }
        }
    }

    if (!s_cdc.enabled) {
        return;
    }

    s_cdc.timer_ticks_100ms++;

    /* --------------------------------------------------------------------------
     * 1. CAN 0x162: CDC Status & Source Availability Heartbeat (Every 100 ms)
     * Payload: [ Byte0, Byte1, 0x06, DiscNum, 0x00, 0x06, 0x00 ]
     * Byte 0: 0xA0 (Radio enabled bit 7 = 1, bit 5 = 1)
     * Byte 1: Status icon in bits 2:0 (3 = Play, 2 = Pause, 1 = Stop, 6 = None)
     *         Bit 1 set (0x02) if playing
     * Byte 2: 0x06
     * Byte 3: DiscNum (1..6)
     * Byte 4: 0x00
     * Byte 5: 0x06 (Non-zero value required by RD4 to register CDC in source list!)
     * Byte 6: 0x00
     * -------------------------------------------------------------------------- */
    can_frame_t frame_162;
    memset(&frame_162, 0, sizeof(frame_162));
    frame_162.id = PSA_CAN_ID_CDC_STATUS_162;
    frame_162.dlc = 7;
    frame_162.data[0] = s_cdc.radio_enabled ? 0xA0 : 0x20;
    frame_162.data[1] = (uint8_t)((s_cdc.playback_active ? 0x02 : 0x00) | (s_cdc.status_icon & 0x07));
    frame_162.data[2] = 0x06;
    frame_162.data[3] = s_cdc.active_disc;
    frame_162.data[4] = 0x00;
    frame_162.data[5] = 0x06;
    frame_162.data[6] = 0x00;
    hal_can_send(&frame_162);

    /* --------------------------------------------------------------------------
     * 2. Periodic 500 ms Group (Every 5 ticks of 100ms)
     * Frames: 0x1A2 (Track Count), 0x1E2 (Current Track & Time), 0x1A0 (Play Modes)
     * -------------------------------------------------------------------------- */
    if (s_cdc.timer_ticks_100ms % 5 == 0) {
        /* CAN 0x1A2: Track Count on Current Disc (DLC 5) */
        can_frame_t frame_1a2;
        memset(&frame_1a2, 0, sizeof(frame_1a2));
        frame_1a2.id = PSA_CAN_ID_CDC_TRACKS_1A2;
        frame_1a2.dlc = 5;
        frame_1a2.data[0] = s_cdc.total_tracks; /* 99 / TRACK_MAX */
        frame_1a2.data[1] = 0x58;
        frame_1a2.data[2] = 0x00;
        frame_1a2.data[3] = 0x00;
        frame_1a2.data[4] = 0x00;
        hal_can_send(&frame_1a2);

        /* CAN 0x1E2: Current Track Number, Minutes, Seconds (DLC 7) */
        can_frame_t frame_1e2;
        memset(&frame_1e2, 0, sizeof(frame_1e2));
        frame_1e2.id = PSA_CAN_ID_CDC_TRACK_1E2;
        frame_1e2.dlc = 7;
        frame_1e2.data[0] = s_cdc.track_num;
        frame_1e2.data[1] = 0x05;
        frame_1e2.data[2] = 0x00;
        frame_1e2.data[3] = s_cdc.elapsed_min;
        frame_1e2.data[4] = s_cdc.elapsed_sec;
        frame_1e2.data[5] = 0x00;
        frame_1e2.data[6] = 0x00;
        hal_can_send(&frame_1e2);

        /* CAN 0x1A0: CDC Play Modes (Intro, Random, Repeat) (DLC 2) */
        can_frame_t frame_1a0;
        memset(&frame_1a0, 0, sizeof(frame_1a0));
        frame_1a0.id = PSA_CAN_ID_CDC_PLAYMODES_1A0;
        frame_1a0.dlc = 2;
        frame_1a0.data[0] = (uint8_t)(0x92 | (s_cdc.intro_mode ? 0x20 : 0x00) | (s_cdc.random_mode ? 0x04 : 0x00));
        frame_1a0.data[1] = (uint8_t)(s_cdc.repeat_mode ? 0x80 : 0x00);
        hal_can_send(&frame_1a0);
    }

    /* --------------------------------------------------------------------------
     * 3. Periodic 1000 ms Group (Every 10 ticks of 100ms)
     * Frames: 0x531 (CDC Wakeup/Heartbeat), 0x0E2 (Disc Slot Indicators)
     * -------------------------------------------------------------------------- */
    if (s_cdc.timer_ticks_100ms % 10 == 0) {
        /* CAN 0x531: CDC Device Alive Heartbeat (DLC 8) */
        can_frame_t frame_531;
        memset(&frame_531, 0, sizeof(frame_531));
        frame_531.id = PSA_CAN_ID_CDC_HEARTBEAT_531;
        frame_531.dlc = 8;
        frame_531.data[0] = 0x09;
        hal_can_send(&frame_531);

        /* CAN 0x0E2: Magazine Disc Loaded Map (DLC 8) */
        can_frame_t frame_0e2;
        memset(&frame_0e2, 0, sizeof(frame_0e2));
        frame_0e2.id = PSA_CAN_ID_CDC_DISCNUM_0E2;
        frame_0e2.dlc = 8;
        frame_0e2.data[0] = 0x00;
        frame_0e2.data[1] = 0x00;
        frame_0e2.data[2] = 0x00;
        frame_0e2.data[3] = (uint8_t)(s_cdc.active_disc << 4);
        frame_0e2.data[4] = 0x00;
        frame_0e2.data[5] = 0x03;
        frame_0e2.data[6] = 0x33;
        frame_0e2.data[7] = 0x33;
        hal_can_send(&frame_0e2);
    }
}

bool psa_virtual_cdc_is_switching_in_progress(void) {
    return (s_arbiter.sync_state != SRC_SYNC_IDLE);
}

bool psa_virtual_cdc_is_source_filtered(void) {
    if (s_arbiter.sync_state != SRC_SYNC_IDLE) {
        return true;
    }
    if (s_arbiter.reported_hu_source != 0 &&
        s_arbiter.actual_rd4_source != s_arbiter.reported_hu_source) {
        return true;
    }
    return false;
}

uint8_t psa_virtual_cdc_get_target_source(void) {
    return s_arbiter.target_source;
}

uint8_t psa_virtual_cdc_get_reported_source(void) {
    return s_arbiter.reported_hu_source;
}

bool psa_virtual_cdc_is_target_source_reached(const struct vehicle_radio *radio) {
    if (s_arbiter.sync_state == SRC_SYNC_IDLE) {
        return true;
    }
    return psa_rd4_is_source(s_arbiter.target_source, (const vehicle_radio_t *)radio);
}
