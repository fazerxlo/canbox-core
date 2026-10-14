#include "profiles/psa_virtual_cdc.h"
#include "profiles/peugeot_407.h"
#include "core/can_router.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_can.h"
#include "hal/hal_system.h"
#include <string.h>

static psa_virtual_cdc_t s_cdc;

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

void psa_virtual_cdc_process_can_0x131(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 5) return;

    /* Byte 0 Bit 7: radio_enabled */
    s_cdc.radio_enabled = (data[0] & 0x80) != 0;

    /* Byte 0 Bit 1: radio_playing (RD4 currently selected CDC as source) */
    bool rd4_playing_cdc = (data[0] & 0x02) != 0;
    if (rd4_playing_cdc) {
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

void psa_virtual_cdc_set_state(bool active) {
    if (active) {
        s_cdc.playback_active = true;
        s_cdc.status_icon = 3;         /* STATUS_ICON_PLAY1 = 3 */

        /* Check current RD4 radio source mode */
        const vehicle_state_t *state = can_router_get_state();
        if (state != NULL && state->media.radio.source_mode != 0x31) {
            /* RD4 is not on CDC (currently on Tuner < 0x20, AUX 0x20/0x21, etc.):
             * inject momentary SOURCE key press on virtual COM2000 stalk (0x21F) */
            can_frame_t frame_press;
            memset(&frame_press, 0, sizeof(frame_press));
            frame_press.id = PSA_CAN_ID_STALK_21F;
            frame_press.dlc = 3;
            frame_press.data[0] = 0x04;
            frame_press.data[1] = 0x00;
            frame_press.data[2] = 0x00;
            hal_can_send(&frame_press);

            hal_delay_ms(60);

            can_frame_t frame_release;
            memset(&frame_release, 0, sizeof(frame_release));
            frame_release.id = PSA_CAN_ID_STALK_21F;
            frame_release.dlc = 3;
            frame_release.data[0] = 0x00;
            frame_release.data[1] = 0x00;
            frame_release.data[2] = 0x00;
            hal_can_send(&frame_release);
        }

        /* Uplink Source Notification: emit Cmd 0x84 (0x31 = CDC) to notify HU */
        vehicle_radio_t cdc_radio;
        if (state != NULL) {
            cdc_radio = state->media.radio;
        } else {
            memset(&cdc_radio, 0, sizeof(cdc_radio));
        }
        cdc_radio.source_mode = 0x31;
        cdc_radio.power_status = 0x01;
        hu_protocol_send_radio_state(&cdc_radio);
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
