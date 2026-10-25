#include "profiles/peugeot_407.h"
#include "profiles/psa_source_arbiter.h"
#include "hal/hal_gpio.h"
#include <string.h>

static inline uint16_t read_be16(const uint8_t *d) {
    return ((uint16_t)d[0] << 8) | (uint16_t)d[1];
}

/* --------------------------------------------------------------------------
 * 1.1 Steering Column Stalk & Buttons
 * -------------------------------------------------------------------------- */
static psa_stalk_state_t g_stalk_state;

void psa_stalk_init(psa_stalk_state_t *st) {
    if (st) {
        memset(st, 0, sizeof(*st));
    } else {
        memset(&g_stalk_state, 0, sizeof(g_stalk_state));
    }
}

void psa_stalk_process_can_ex(psa_stalk_state_t *st, const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key) {
    if (!st || !data || !send_key || dlc < 2) {
        return;
    }

    uint8_t b0 = data[0];
    uint8_t b1 = data[1];
    uint8_t b2 = (dlc >= 3) ? data[2] : 0;

    #define DISPATCH_KEY(curr, prev, mask, key_code) do { \
        if (((curr) & (mask)) && !((prev) & (mask))) send_key((key_code), 1); \
        if (!((curr) & (mask)) && ((prev) & (mask))) send_key((key_code), 0); \
    } while (0)

    DISPATCH_KEY(b0, st->prev_b0, 0x08, PSA_STALK_KEY_VOL_UP);     /* Vol + */
    DISPATCH_KEY(b0, st->prev_b0, 0x04, PSA_STALK_KEY_VOL_DOWN);   /* Vol - */
    DISPATCH_KEY(b0, st->prev_b0, 0x02, PSA_STALK_KEY_NEXT);       /* Next */
    DISPATCH_KEY(b0, st->prev_b0, 0x01, PSA_STALK_KEY_PREV);       /* Prev */
    DISPATCH_KEY(b0, st->prev_b0, 0x40, PSA_STALK_KEY_SRC);        /* Source */
    DISPATCH_KEY(b0, st->prev_b0, 0x10, PSA_STALK_KEY_OK);         /* OK / Confirm */
    DISPATCH_KEY(b0, st->prev_b0, 0x80, PSA_STALK_KEY_DARK);       /* Dark */
    DISPATCH_KEY(b0, st->prev_b0, 0x20, PSA_STALK_KEY_ESC);        /* ESC */
    DISPATCH_KEY(b1, st->prev_b1, 0x40, PSA_STALK_KEY_MENU);       /* Menu */
    DISPATCH_KEY(b2, st->prev_b2, 0x01, PSA_STALK_KEY_TEL_ANSWER); /* Tel Answer */
    DISPATCH_KEY(b2, st->prev_b2, 0x02, PSA_STALK_KEY_TEL_HANGUP); /* Tel Hangup */

    #undef DISPATCH_KEY

    /* Rotary Encoder Scroll delta */
    int8_t scroll_curr = (int8_t)(b1 & 0x0F);
    int8_t scroll_prev = (int8_t)(st->prev_b1 & 0x0F);
    int8_t delta = (int8_t)(scroll_curr - scroll_prev);
    if ((delta > 0 && delta <= 7) || delta < -7) {
        send_key(PSA_STALK_KEY_SCROLL_UP, 1);
        send_key(PSA_STALK_KEY_SCROLL_UP, 0);
    } else if ((delta < 0 && delta >= -7) || delta > 7) {
        send_key(PSA_STALK_KEY_SCROLL_DOWN, 1);
        send_key(PSA_STALK_KEY_SCROLL_DOWN, 0);
    }

    st->prev_b0 = b0;
    st->prev_b1 = b1;
    st->prev_b2 = b2;
}

void psa_stalk_process_can(const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key) {
    psa_stalk_process_can_ex(&g_stalk_state, data, dlc, send_key);
}

void psa_decode_stalk_0x21f_ex(psa_stalk_state_t *st, const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key) {
    if (!st || !data || !send_key || dlc < 1) {
        return;
    }

    uint8_t b0 = data[0];
    uint8_t prev0 = st->prev_b0;

    /* Check chorded Mute first (Vol+ and Vol- simultaneously: 0x08 | 0x04 = 0x0C) */
    bool curr_mute = ((b0 & 0x0C) == 0x0C);
    bool prev_mute = ((prev0 & 0x0C) == 0x0C);

    if (curr_mute && !prev_mute) {
        send_key(PSA_STALK_KEY_MUTE, 1);
    } else if (!curr_mute && prev_mute) {
        send_key(PSA_STALK_KEY_MUTE, 0);
    }

    /* If not mute, check Vol+ and Vol- independently */
    if (!curr_mute && !prev_mute) {
        if ((b0 & 0x08) && !(prev0 & 0x08)) send_key(PSA_STALK_KEY_VOL_UP, 1);
        if (!(b0 & 0x08) && (prev0 & 0x08)) send_key(PSA_STALK_KEY_VOL_UP, 0);

        if ((b0 & 0x04) && !(prev0 & 0x04)) send_key(PSA_STALK_KEY_VOL_DOWN, 1);
        if (!(b0 & 0x04) && (prev0 & 0x04)) send_key(PSA_STALK_KEY_VOL_DOWN, 0);
    }

    /* Next Track (Seek+) on bit 7 (0x80) */
    if ((b0 & 0x80) && !(prev0 & 0x80)) send_key(PSA_STALK_KEY_NEXT, 1);
    if (!(b0 & 0x80) && (prev0 & 0x80)) send_key(PSA_STALK_KEY_NEXT, 0);

    /* Prev Track (Seek-) on bit 6 (0x40) */
    if ((b0 & 0x40) && !(prev0 & 0x40)) send_key(PSA_STALK_KEY_PREV, 1);
    if (!(b0 & 0x40) && (prev0 & 0x40)) send_key(PSA_STALK_KEY_PREV, 0);

    /* Source (SRC) on bit 1 (0x02) */
    if ((b0 & 0x02) && !(prev0 & 0x02)) send_key(PSA_STALK_KEY_SRC, 1);
    if (!(b0 & 0x02) && (prev0 & 0x02)) send_key(PSA_STALK_KEY_SRC, 0);

    /* Rotary Encoder (molette) counter on Byte 1 */
    if (dlc >= 2) {
        uint8_t b1 = data[1];
        psa_source_arbiter_update_stalk_rotary(b1);
        if (st->rotary_init) {
            int8_t delta = (int8_t)(b1 - st->prev_rotary);
            if (delta > 0) {
                send_key(PSA_STALK_KEY_SCROLL_UP, 1);
                send_key(PSA_STALK_KEY_SCROLL_UP, 0);
            } else if (delta < 0) {
                send_key(PSA_STALK_KEY_SCROLL_DOWN, 1);
                send_key(PSA_STALK_KEY_SCROLL_DOWN, 0);
            }
        }
        st->prev_rotary = b1;
        st->rotary_init = true;
    }

    st->prev_b0 = b0;
}

void psa_decode_stalk_0x21f(const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key) {
    psa_decode_stalk_0x21f_ex(&g_stalk_state, data, dlc, send_key);
}

void psa_decode_stalk_tip_0x221_ex(psa_stalk_state_t *st, const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key) {
    if (!st || !data || !send_key || dlc < 1) {
        return;
    }

    /* Stalk Tip Trip button on Byte 0 Bit 3 (0x08).
     * Idle: 0xC0 (bit 3 = 0), Pressed: 0xC8 (bit 3 = 1).
     * Proven in vehicle dump dump_2026-10-06_20-27-16.log and dump_2026-10-06_20-45-34.log.
     * Left Stalk Tip button on Byte 0 Bit 0 (0x01) -> Hiworld Key 0x40 (H1H2PAF23A 0x080081EE).
     */
    bool curr_trip = (data[0] & 0x08) != 0;
    bool prev_trip = (st->prev_trip_tip != 0);

    if (curr_trip && !prev_trip) {
        send_key(PSA_STALK_KEY_TRIP, 1);
    } else if (!curr_trip && prev_trip) {
        send_key(PSA_STALK_KEY_TRIP, 0);
    }

    st->prev_trip_tip = curr_trip ? 1 : 0;

    bool curr_left = (data[0] & 0x01) != 0;
    bool prev_left = (st->prev_left_tip != 0);

    if (curr_left && !prev_left) {
        send_key(PSA_STALK_KEY_LEFT_TIP, 1);
    } else if (!curr_left && prev_left) {
        send_key(PSA_STALK_KEY_LEFT_TIP, 0);
    }

    st->prev_left_tip = curr_left ? 1 : 0;
}

void psa_decode_stalk_tip_0x221(const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key) {
    psa_decode_stalk_tip_0x221_ex(&g_stalk_state, data, dlc, send_key);
}

/* --------------------------------------------------------------------------
 * 1.1b RD4 Center Console & Fascia Buttons (0x3E5 only)
 * -------------------------------------------------------------------------- */
static psa_console_state_t g_console_state;

void psa_console_init(psa_console_state_t *st) {
    if (st) {
        memset(st, 0, sizeof(*st));
    } else {
        memset(&g_console_state, 0, sizeof(g_console_state));
    }
}

/* RD4 / RD5 Center Console Physical Buttons (CAN ID 0x3E5, DLC 6).
 * Every button is a 2-bit field; a non-zero field means pressed (observed value 01).
 *   Byte 0: MENU[7:6]  TEL[5:4] (bit position from simulator doc, not yet captured)  CLIM[1:0]
 *   Byte 1: TRIP[7:6]  MODE[5:4] (pending, not decoded) AUDIO[1:0]
 *   Byte 2: OK[7:6]    ESC[5:4]  DARK[3:2]
 *   Byte 5: UP[7:6]    DOWN[5:4] RIGHT[3:2] LEFT[1:0]
 * Verified in can_log_buttons.log / can_log_buttons1.log / can_log_buttons2.log
 * and doc/CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md section 3.3.
 * 0x167 and 0x0DF are NOT key sources.
 */
typedef struct {
    uint8_t byte_idx;
    uint8_t mask;
    uint8_t key;
} psa_console_map_t;

static const psa_console_map_t k_console_map[] = {
    { 0, 0xC0, PSA_PANEL_KEY_MENU  },
    { 0, 0x30, PSA_PANEL_KEY_TEL   },
    { 0, 0x03, PSA_PANEL_KEY_CLIM  },
    { 1, 0xC0, PSA_PANEL_KEY_TRIP  },
    { 1, 0x03, PSA_PANEL_KEY_AUDIO },
    { 2, 0xC0, PSA_PANEL_KEY_OK    },
    { 2, 0x30, PSA_PANEL_KEY_ESC   },
    { 2, 0x0C, PSA_PANEL_KEY_DARK  },
    { 5, 0xC0, PSA_PANEL_KEY_UP    },
    { 5, 0x30, PSA_PANEL_KEY_DOWN  },
    { 5, 0x0C, PSA_PANEL_KEY_RIGHT },
    { 5, 0x03, PSA_PANEL_KEY_LEFT  },
};

void psa_decode_console_0x3e5_ex(psa_console_state_t *st, const uint8_t *data, uint8_t dlc, psa_panel_key_callback_t send_key) {
    if (!st || !data || !send_key || dlc < 6) {
        return;
    }

    uint8_t key = PSA_PANEL_KEY_NONE;
    bool active_still_held = false;

    for (size_t i = 0; i < sizeof(k_console_map) / sizeof(k_console_map[0]); i++) {
        const psa_console_map_t *m = &k_console_map[i];
        if ((data[m->byte_idx] & m->mask) == 0) {
            continue;
        }
        if (m->key == st->active_key) {
            active_still_held = true;
        } else if (key == PSA_PANEL_KEY_NONE) {
            key = m->key;
        }
    }

    /* Keep the currently reported key while it is still held */
    if (active_still_held) {
        return;
    }

    if (key != st->active_key) {
        if (st->active_key != PSA_PANEL_KEY_NONE) {
            send_key(st->active_key, 0);
        }
        if (key != PSA_PANEL_KEY_NONE) {
            send_key(key, 1);
        }
        st->active_key = key;
    }
}

void psa_decode_console_0x3e5(const uint8_t *data, uint8_t dlc, psa_panel_key_callback_t send_key) {
    psa_decode_console_0x3e5_ex(&g_console_state, data, dlc, send_key);
}

/* --------------------------------------------------------------------------
 * 1.2 Dual-Zone Climate Control (HVAC)
 * -------------------------------------------------------------------------- */
void psa_decode_hvac_0x1d0(const uint8_t *data, uint8_t dlc, hvac_state_t *st) {
    if (!data || !st || dlc < 4) {
        return;
    }

    memset(st, 0, sizeof(*st));

    st->power         = (data[0] & 0x80) != 0;
    st->ac_compressor = (data[0] & 0x40) != 0;
    st->recirculation = (data[0] & 0x20) != 0;
    st->aqs_auto      = (data[0] & 0x10) != 0;
    st->auto_mode     = (data[0] & 0x08) != 0;
    st->dual_mode     = (data[0] & 0x04) != 0;
    st->rear_defrost  = (data[0] & 0x01) != 0;

    st->driver_wind_up   = (data[1] & 0x80) != 0;
    st->driver_wind_face = (data[1] & 0x40) != 0;
    st->driver_wind_down = (data[1] & 0x20) != 0;
    st->fan_speed        = (data[1] & 0x0F);

    st->driver_temp_raw  = data[2];
    st->pass_temp_raw    = data[3];

    if (dlc >= 5) {
        st->front_max_defrost = (data[4] & 0x80) != 0;
        st->ac_max            = (data[4] & 0x08) != 0;
    }

    if (dlc >= 7) {
        st->pass_wind_up   = (data[6] & 0x80) != 0;
        st->pass_wind_face = (data[6] & 0x40) != 0;
        st->pass_wind_down = (data[6] & 0x20) != 0;
    }
}

static const uint8_t peugeot_temp_map[23] = {
    0xFE, 28, 30, 32, 34, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 52, 54, 56, 0xFF
};



static const uint8_t psa_wind_to_hiworld[16] = {
    [0] = 0,   // Auto
    [1] = 11,  // Windshield / Screen (0x0B)
    [2] = 3,   // Floor / Feet (0x03)
    [3] = 6,   // Face / Center (0x06)
    [4] = 11,  // Windshield / Screen (0x0B)
    [5] = 5,   // Face + Floor (0x05)
    [6] = 12,  // Windshield + Floor (0x0C)
    [7] = 13,  // Windshield + Face (0x0D)
    [8] = 14,  // Floor / Down (0x0E)
};

void psa_hvac_process_can_0x1d0(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc) {
    if (!climate || !data) return;

    /* Rear defrost demand: Byte 4 Bit 6 (0x40) */
    if (dlc >= 5) climate->rear_defrost = ((data[4] >> 6) & 0x01) != 0;

    if (dlc < 7) return;

    climate->power_on = (data[0] != 0xA8);
    climate->ac_on = climate->power_on;
    climate->front_max_defrost = (data[0] == 0x19);

    uint8_t fan = data[2] & 0x0F;
    if (fan == 0x0F) {
        climate->fan_speed = 0;
    } else if (fan <= 8) {
        climate->fan_speed = fan + 1;
    }

    /* Air distribution: Byte 3 upper nibble is Left (Driver), lower nibble is Right (Passenger) */
    uint8_t left_code = (data[3] >> 4) & 0x0F;
    uint8_t right_code = data[3] & 0x0F;

    climate->driver_wind_mode = psa_wind_to_hiworld[left_code];
    climate->pass_wind_mode = psa_wind_to_hiworld[right_code];

    if (data[0] == 0x19) {
        /* When front defrost is active, Byte 4 (0x20) reflects physical forced fresh air flaps.
         * Do not overwrite aqs_auto from Byte 4; preserve aqs_auto as decoded by 0x1E3. */
        climate->recirculate = false;
    } else if ((data[4] & 0x10) != 0) {
        climate->recirculate = true;
        climate->aqs_auto = false;
    } else if ((data[4] & 0x20) != 0) {
        climate->recirculate = false;
        climate->aqs_auto = false;
    } else {
        climate->recirculate = false;
        climate->aqs_auto = true;
    }

    uint8_t l_temp = data[5];
    uint8_t r_temp = data[6];
    if (l_temp < 23) climate->temp_driver = peugeot_temp_map[l_temp];
    if (r_temp < 23) climate->temp_passenger = peugeot_temp_map[r_temp];
}

void psa_hvac_process_can_0x1e3(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc) {
    if (!climate || !data || dlc < 2) return;

    climate->auto_mode = ((data[0] & 0x0C) == 0x0C);
    climate->dual_mode = (data[0] & 0x01) != 0;
    climate->front_max_defrost = (data[1] & 0x80) != 0;

    if (climate->front_max_defrost) {
        climate->recirculate = false;
        climate->aqs_auto = ((data[0] & 0x10) != 0);
    } else if ((data[0] & 0x80) != 0) {
        climate->recirculate = true;
        climate->aqs_auto = false;
    } else if ((data[0] & 0x10) != 0) {
        climate->recirculate = false;
        climate->aqs_auto = true;
    } else {
        climate->recirculate = false;
        climate->aqs_auto = false;
    }

    if (dlc >= 6) {
        /* Air distribution: Byte 4 upper nibble is Left (Driver), Byte 5 upper nibble is Right (Passenger) */
        uint8_t left_code = (data[4] >> 4) & 0x0F;
        uint8_t right_code = (data[5] >> 4) & 0x0F;

        climate->driver_wind_mode = psa_wind_to_hiworld[left_code];
        climate->pass_wind_mode = psa_wind_to_hiworld[right_code];
    }
}

void psa_hvac_process_can_0x12d(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc) {
    (void)climate;
    (void)data;
    (void)dlc;
}

void psa_hvac_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    if (frame->id == PSA_CAN_ID_CLIMATE_HVAC) {
        psa_hvac_process_can_0x1d0(&state->climate, frame->data, frame->dlc);
    } else if (frame->id == 0x1E3) {
        psa_hvac_process_can_0x1e3(&state->climate, frame->data, frame->dlc);
    }
}

/* --------------------------------------------------------------------------
 * 1.3 Ultrasonic Parking Sensors (Front & Rear AAS)
 * -------------------------------------------------------------------------- */
void psa_decode_aas_rear_0x260(const uint8_t *data, uint8_t dlc, uint8_t *rl, uint8_t *rc, uint8_t *rr) {
    if (!data || dlc < 3) return;
    if (rl) *rl = data[0];
    if (rc) *rc = data[1];
    if (rr) *rr = data[2];
}

void psa_decode_aas_front_0x270(const uint8_t *data, uint8_t dlc, uint8_t *fl, uint8_t *fc, uint8_t *fr) {
    if (!data || dlc < 3) return;
    if (fl) *fl = data[0];
    if (fc) *fc = data[1];
    if (fr) *fr = data[2];
}

uint8_t psa_radar_map_zone(uint8_t raw3bit) {
    if (raw3bit >= 7) return PSA_RADAR_DIST_INACTIVE;
    return raw3bit;
}

void psa_radar_process_can_0x0e1(vehicle_radar_t *radar, const uint8_t *data, uint8_t dlc) {
    if (!radar || !data || dlc < 6) return;

    radar->rear_active    = (data[1] >> 6) & 1;
    radar->front_active   = (data[1] >> 4) & 1;
    radar->display_active = (data[5] & 0x02) ? true : false;

    if (!radar->display_active && !radar->rear_active && !radar->front_active) {
        radar->rear_left_outer    = PSA_RADAR_DIST_INACTIVE;
        radar->rear_left_center   = PSA_RADAR_DIST_INACTIVE;
        radar->rear_right_center  = PSA_RADAR_DIST_INACTIVE;
        radar->rear_right_outer   = PSA_RADAR_DIST_INACTIVE;
        radar->front_left_outer   = PSA_RADAR_DIST_INACTIVE;
        radar->front_left_center  = PSA_RADAR_DIST_INACTIVE;
        radar->front_right_center = PSA_RADAR_DIST_INACTIVE;
        radar->front_right_outer  = PSA_RADAR_DIST_INACTIVE;
    } else {
        uint8_t rl = (data[3] >> 5) & 0x07;
        uint8_t rc = (data[3] >> 2) & 0x07;
        uint8_t rr = (data[4] >> 5) & 0x07;
        uint8_t fl = (data[4] >> 2) & 0x07;
        uint8_t fc = (data[5] >> 5) & 0x07;
        uint8_t fr = (data[5] >> 2) & 0x07;

        radar->rear_left_outer    = psa_radar_map_zone(rl);
        radar->rear_left_center   = psa_radar_map_zone(rc);
        radar->rear_right_center  = psa_radar_map_zone(rc);
        radar->rear_right_outer   = psa_radar_map_zone(rr);

        radar->front_left_outer   = psa_radar_map_zone(fl);
        radar->front_left_center  = psa_radar_map_zone(fc);
        radar->front_right_center = psa_radar_map_zone(fc);
        radar->front_right_outer  = psa_radar_map_zone(fr);
    }
    radar->valid = true;
}

void psa_radar_process_can_0x260(vehicle_radar_t *radar, const uint8_t *data, uint8_t dlc) {
    if (!radar || !data || dlc < 3) return;

    radar->rear_left_outer   = data[0];
    radar->rear_left_center  = data[1];
    radar->rear_right_center = data[1];
    radar->rear_right_outer  = data[2];

    if (dlc >= 4) {
        radar->rear_active  = (data[3] & 0x80) ? true : false;
        radar->system_fault = (data[3] & 0x01) ? true : false;
    }
    radar->valid = true;
}

void psa_radar_process_can_0x270(vehicle_radar_t *radar, const uint8_t *data, uint8_t dlc) {
    if (!radar || !data || dlc < 3) return;

    radar->front_left_outer   = data[0];
    radar->front_left_center  = data[1];
    radar->front_right_center = data[1];
    radar->front_right_outer  = data[2];

    radar->valid = true;
}

void psa_radar_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    if (frame->id == PSA_CAN_ID_RADAR_0E1) {
        psa_radar_process_can_0x0e1(&state->radar, frame->data, frame->dlc);
        state->dirty.radar = true;
    }
}


void psa_stalk_process_tip_0x221(vehicle_wheel_t *wheel, const uint8_t *data, uint8_t dlc) {
    if (!wheel || !data || dlc < 1) return;
    bool curr_trip = (data[0] & 0x08) != 0;
    bool curr_left = (data[0] & 0x01) != 0;

    if (curr_trip) {
        wheel->active_key = WHEEL_KEY_TRIP;
        wheel->press_state = 1;
    } else if (curr_left) {
        wheel->active_key = WHEEL_KEY_STALK_LEFT;
        wheel->press_state = 1;
    } else if (g_stalk_state.prev_trip_tip != 0 || g_stalk_state.prev_left_tip != 0 ||
               wheel->active_key == WHEEL_KEY_TRIP || wheel->active_key == WHEEL_KEY_STALK_LEFT) {
        wheel->active_key = WHEEL_KEY_NONE;
        wheel->press_state = 0;
    }
    g_stalk_state.prev_trip_tip = curr_trip ? 1 : 0;
    g_stalk_state.prev_left_tip = curr_left ? 1 : 0;
}


/* --------------------------------------------------------------------------
 * 1.5 Doors & Body Status
 * -------------------------------------------------------------------------- */
void psa_decode_doors_0x220(const uint8_t *data, uint8_t dlc, psa_doors_body_t *doors) {
    if (!data || !doors || dlc < 1) return;
    memset(doors, 0, sizeof(*doors));

    doors->driver_door     = (data[0] & 0x80) != 0;
    doors->pass_door       = (data[0] & 0x40) != 0;
    doors->rear_left_door  = (data[0] & 0x20) != 0;
    doors->rear_right_door = (data[0] & 0x10) != 0;
    doors->trunk           = (data[0] & 0x08) != 0;
    doors->hood            = false; /* PSA CAN 0x220 bit 0x04 is body bus idle active flag, NOT hood */
    doors->rear_window     = (data[0] & 0x02) != 0;
    doors->fuel_flap       = (data[0] & 0x01) != 0;

    if (dlc >= 2) {
        doors->auto_rear_wiper       = (data[1] & 0x80) != 0;
        doors->auto_locking          = (data[1] & 0x10) != 0;
        doors->parking_radar_enabled = (data[1] & 0x08) != 0;
        doors->handbrake             = (data[1] & 0x01) != 0;
    }
}

void psa_decode_doors_0x221(const uint8_t *data, uint8_t dlc, psa_doors_body_t *doors) {
    psa_decode_doors_0x220(data, dlc, doors);
}

void psa_doors_process_can_0x220(vehicle_doors_t *doors, bool *handbrake, const uint8_t *data, uint8_t dlc) {
    if (!doors || !data || dlc < 1) return;

    uint8_t b0 = data[0];
    doors->door_driver     = (b0 & 0x80) ? true : false;
    doors->door_passenger  = (b0 & 0x40) ? true : false;
    doors->door_rear_left  = (b0 & 0x20) ? true : false;
    doors->door_rear_right = (b0 & 0x10) ? true : false;
    doors->trunk           = (b0 & 0x08) ? true : false;
    doors->hood            = false; /* PSA CAN 0x220 bit 0x04 is body bus idle active flag, NOT hood */

    if (dlc >= 2 && handbrake) {
        *handbrake = (data[1] & 0x01) ? true : false;
    }
}

void psa_doors_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;
    if (frame->id == PSA_CAN_ID_DOORS_BODY_220) {
        psa_doors_process_can_0x220(&state->doors, &state->handbrake, frame->data, frame->dlc);
    }
}

/* --------------------------------------------------------------------------
 * 1.6 Steering Wheel Angle & Dynamic Trajectory
 * -------------------------------------------------------------------------- */
void psa_decode_steering_angle_0x0e6(const uint8_t *data, uint8_t dlc, int16_t *angle_deci_deg) {
    if (!data || !angle_deci_deg || dlc < 2) return;
    *angle_deci_deg = (int16_t)read_be16(&data[0]);
}

