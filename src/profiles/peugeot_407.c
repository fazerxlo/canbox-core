#include "profiles/peugeot_407.h"
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

size_t build_raise_stalk_key(uint8_t key_id, uint8_t state, uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < 6) {
        return 0;
    }

    out_buf[0] = 0x2E;
    out_buf[1] = 0x02;
    out_buf[2] = 0x02;
    out_buf[3] = key_id;
    out_buf[4] = state;
    out_buf[5] = (uint8_t)((out_buf[1] + out_buf[2] + out_buf[3] + out_buf[4]) ^ 0xFF);
    return 6;
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

size_t build_raise_hvac_packet(const hvac_state_t *st, uint8_t *out_buf, size_t max_len) {
    if (!st || !out_buf || max_len < 11) {
        return 0;
    }

    out_buf[0] = 0x2E;
    out_buf[1] = 0x21;
    out_buf[2] = 0x07; /* Length */

    uint8_t d0 = 0;
    if (st->power)         d0 |= 0x80;
    if (st->ac_compressor) d0 |= 0x40;
    if (st->recirculation) d0 |= 0x20;
    if (st->aqs_auto)      d0 |= 0x10;
    if (st->auto_mode)     d0 |= 0x08;
    if (st->dual_mode)     d0 |= 0x04;
    if (st->rear_defrost)  d0 |= 0x01;
    out_buf[3] = d0;

    uint8_t d1 = (uint8_t)(st->fan_speed & 0x0F);
    if (st->driver_wind_up)   d1 |= 0x80;
    if (st->driver_wind_face) d1 |= 0x40;
    if (st->driver_wind_down) d1 |= 0x20;
    out_buf[4] = d1;

    out_buf[5] = st->driver_temp_raw;
    out_buf[6] = st->pass_temp_raw;

    uint8_t d4 = 0;
    if (st->front_max_defrost) d4 |= 0x80;
    if (st->ac_max)            d4 |= 0x08;
    out_buf[7] = d4;

    out_buf[8] = 0x00; /* Rear Power / Flags */

    uint8_t d6 = 0;
    if (st->pass_wind_up)   d6 |= 0x80;
    if (st->pass_wind_face) d6 |= 0x40;
    if (st->pass_wind_down) d6 |= 0x20;
    out_buf[9] = d6;

    /* Checksum: (Sum ^ 0xFF) & 0xFF */
    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out_buf[i];
    }
    out_buf[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
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

size_t build_raise_rear_radar(uint8_t rl, uint8_t rc, uint8_t rr,
                              uint8_t fl, uint8_t fc, uint8_t fr,
                              uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < 11) return 0;
    out_buf[0] = 0x2E;
    out_buf[1] = 0x32;
    out_buf[2] = 0x07; /* Length */
    out_buf[3] = 0x00; /* Flag */
    out_buf[4] = rl;
    out_buf[5] = rc;
    out_buf[6] = rr;
    out_buf[7] = fl;
    out_buf[8] = fc;
    out_buf[9] = fr;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out_buf[i];
    }
    out_buf[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

size_t build_raise_front_radar(uint8_t fl, uint8_t fc, uint8_t fr,
                               uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < 8) return 0;
    out_buf[0] = 0x2E;
    out_buf[1] = 0x30;
    out_buf[2] = 0x04; /* Length */
    out_buf[3] = 0x00; /* Flag */
    out_buf[4] = fl;
    out_buf[5] = fc;
    out_buf[6] = fr;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 6; i++) {
        sum += out_buf[i];
    }
    out_buf[7] = (uint8_t)(sum ^ 0xFF);
    return 8;
}

/* --------------------------------------------------------------------------
 * 1.4 Trip Computer & Engine Telemetry
 * -------------------------------------------------------------------------- */
void psa_decode_trip_0x165(const uint8_t *data, uint8_t dlc,
                           uint16_t *instant_fuel, uint16_t *range_dte,
                           uint16_t *dest_dist, uint8_t *ext_temp_raw) {
    if (!data) return;
    if (dlc >= 2 && instant_fuel) *instant_fuel = read_be16(&data[0]);
    if (dlc >= 4 && range_dte)    *range_dte = read_be16(&data[2]);
    if (dlc >= 6 && dest_dist)    *dest_dist = read_be16(&data[4]);
    if (dlc >= 7 && ext_temp_raw) *ext_temp_raw = data[6];
}

void psa_decode_trip1_0x1a5(const uint8_t *data, uint8_t dlc,
                            uint16_t *dist, uint16_t *avg_fuel, uint16_t *avg_speed) {
    if (!data) return;
    if (dlc >= 2 && dist)      *dist = read_be16(&data[0]);
    if (dlc >= 4 && avg_fuel)  *avg_fuel = read_be16(&data[2]);
    if (dlc >= 5 && avg_speed) *avg_speed = (uint16_t)data[4];
}

void psa_decode_trip2_0x2a5(const uint8_t *data, uint8_t dlc,
                            uint16_t *dist, uint16_t *avg_fuel, uint16_t *avg_speed) {
    if (!data) return;
    if (dlc >= 2 && dist)      *dist = read_be16(&data[0]);
    if (dlc >= 4 && avg_fuel)  *avg_fuel = read_be16(&data[2]);
    if (dlc >= 5 && avg_speed) *avg_speed = (uint16_t)data[4];
}

size_t build_raise_instant_fuel(uint16_t instant_fuel_dkl, uint16_t dte_range_km,
                                uint16_t dest_dist_km, uint8_t *out, size_t max_len) {
    if (!out || max_len < 10) return 0;
    out[0] = 0x2E;
    out[1] = 0x33;
    out[2] = 0x06;
    out[3] = (uint8_t)(instant_fuel_dkl >> 8);
    out[4] = (uint8_t)(instant_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(dte_range_km >> 8);
    out[6] = (uint8_t)(dte_range_km & 0xFF);
    out[7] = (uint8_t)(dest_dist_km >> 8);
    out[8] = (uint8_t)(dest_dist_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_trip1(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out) {
    if (!out) return 0;
    out[0] = 0x2E;
    out[1] = 0x34;
    out[2] = 0x06;
    out[3] = (uint8_t)(avg_fuel_dkl >> 8);
    out[4] = (uint8_t)(avg_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(avg_spd_kmh >> 8);
    out[6] = (uint8_t)(avg_spd_kmh & 0xFF);
    out[7] = (uint8_t)(dist_dkm >> 8);
    out[8] = (uint8_t)(dist_dkm & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_trip2(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out) {
    if (!out) return 0;
    out[0] = 0x2E;
    out[1] = 0x35;
    out[2] = 0x06;
    out[3] = (uint8_t)(avg_fuel_dkl >> 8);
    out[4] = (uint8_t)(avg_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(avg_spd_kmh >> 8);
    out[6] = (uint8_t)(avg_spd_kmh & 0xFF);
    out[7] = (uint8_t)(dist_dkm >> 8);
    out[8] = (uint8_t)(dist_dkm & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_outside_temp(uint8_t temp_raw, uint8_t *out, size_t max_len) {
    if (!out || max_len < 5) return 0;
    out[0] = 0x2E;
    out[1] = 0x36;
    out[2] = 0x01;
    out[3] = temp_raw;
    out[4] = (uint8_t)((out[1] + out[2] + out[3]) ^ 0xFF);
    return 5;
}

size_t build_raise_reverse_state(bool reverse_active, uint8_t *out, size_t max_len) {
    if (!out || max_len < 5) return 0;
    out[0] = 0x2E;
    out[1] = 0x40;
    out[2] = 0x01;
    out[3] = reverse_active ? 0x80 : 0x00;
    out[4] = (uint8_t)((out[1] + out[2] + out[3]) ^ 0xFF);
    return 5;
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
    doors->hood            = (data[0] & 0x04) != 0;
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

size_t build_raise_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len) {
    if (!doors || !out || max_len < 12) return 0;
    out[0] = 0x2E;
    out[1] = 0x38;
    out[2] = 0x08; /* Length */

    uint8_t d0 = 0;
    if (doors->driver_door)     d0 |= 0x80;
    if (doors->pass_door)       d0 |= 0x40;
    if (doors->rear_left_door)  d0 |= 0x20;
    if (doors->rear_right_door) d0 |= 0x10;
    if (doors->trunk)           d0 |= 0x08;
    if (doors->hood)            d0 |= 0x04;
    out[3] = d0;

    out[4] = doors->handbrake ? 0x01 : 0x00;
    out[5] = 0x00;
    out[6] = 0x00;
    out[7] = 0x00;
    out[8] = 0x00;
    out[9] = 0x00;
    out[10] = 0x00;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_hiworld_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len) {
    if (!doors || !out || max_len < 15) {
        return 0;
    }

    out[0] = HIWORLD_SOF1;        /* 0x5A */
    out[1] = HIWORLD_SOF2;        /* 0xA5 */
    out[2] = 0x0A;                /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_DOOR;    /* Cmd: 0x12 */
    out[4] = 0x00;                /* Byte 0 */
    out[5] = 0x04;                /* Byte 1 */

    uint8_t b2 = 0x04;          /* Base active flag */
    if (doors->driver_door)     b2 |= 0x80;
    if (doors->pass_door)       b2 |= 0x40;
    if (doors->rear_left_door)  b2 |= 0x20;
    if (doors->rear_right_door) b2 |= 0x10;
    if (doors->trunk)           b2 |= 0x08;
    if (doors->hood)            b2 |= 0x04;
    out[6] = b2;

    out[7]  = 0x00;
    out[8]  = 0x00;
    out[9]  = 0x00;
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x03;               /* Byte 9: Sub-type */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);

    return 15;
}

size_t build_hiworld_doors_state(const psa_doors_state_t *state, uint8_t *out, size_t max_len) {
    if (!state || !out || max_len < 15) {
        return 0;
    }

    out[0] = HIWORLD_SOF1;        /* 0x5A */
    out[1] = HIWORLD_SOF2;        /* 0xA5 */
    out[2] = 0x0A;                /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_DOOR;    /* Cmd: 0x12 */
    out[4] = 0x00;                /* Byte 0 */
    out[5] = 0x04;                /* Byte 1 */

    uint8_t b2 = 0x04;          /* Base active flag */
    if (state->door_front_left)  b2 |= 0x80;
    if (state->door_front_right) b2 |= 0x40;
    if (state->door_rear_left)   b2 |= 0x20;
    if (state->door_rear_right)  b2 |= 0x10;
    if (state->trunk_open)       b2 |= 0x08;
    if (state->hood_open)        b2 |= 0x04;
    out[6] = b2;

    out[7]  = 0x00;
    out[8]  = 0x00;
    out[9]  = 0x00;
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x03;               /* Byte 9: Sub-type */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);

    return 15;
}

void psa_doors_init(psa_doors_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(psa_doors_ctx_t));
    ctx->uart_tx = uart_tx;
}

void psa_doors_send_hiworld(psa_doors_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[16];
    size_t len = build_hiworld_doors_state(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

void psa_doors_process_can_0x220(psa_doors_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 1) return;

    uint8_t b0 = data[0];
    ctx->state.door_front_left   = (b0 & 0x80) ? true : false;
    ctx->state.door_front_right  = (b0 & 0x40) ? true : false;
    ctx->state.door_rear_left    = (b0 & 0x20) ? true : false;
    ctx->state.door_rear_right   = (b0 & 0x10) ? true : false;
    ctx->state.trunk_open        = (b0 & 0x08) ? true : false;
    ctx->state.hood_open         = (b0 & 0x04) ? true : false;
    ctx->state.rear_window_open  = (b0 & 0x02) ? true : false;
    ctx->state.fuel_flap_open    = (b0 & 0x01) ? true : false;

    if (dlc >= 2) {
        ctx->state.auto_rear_wiper_active = (data[1] & 0x80) ? true : false;
        ctx->state.auto_locking_active    = (data[1] & 0x10) ? true : false;
        ctx->state.parking_radar_enabled  = (data[1] & 0x08) ? true : false;
        ctx->state.handbrake_pulled       = (data[1] & 0x01) ? true : false;
    }

    psa_doors_send_hiworld(ctx);
}

/* --------------------------------------------------------------------------
 * 1.6 Steering Wheel Angle & Dynamic Trajectory
 * -------------------------------------------------------------------------- */
void psa_decode_steering_angle_0x0e6(const uint8_t *data, uint8_t dlc, int16_t *angle_deci_deg) {
    if (!data || !angle_deci_deg || dlc < 2) return;
    *angle_deci_deg = (int16_t)read_be16(&data[0]);
}

size_t build_raise_steering_angle(int16_t angle_deci_deg, uint8_t *out) {
    if (!out) return 0;
    out[0] = 0x2E;
    out[1] = 0x29;
    out[2] = 0x02;
    out[3] = (uint8_t)(angle_deci_deg & 0xFF);        /* Little Endian */
    out[4] = (uint8_t)((angle_deci_deg >> 8) & 0xFF);
    out[5] = (uint8_t)((out[1] + out[2] + out[3] + out[4]) ^ 0xFF);
    return 6;
}

/* --------------------------------------------------------------------------
 * 1.7 OEM JBL Sound Amplifier (DSP)
 * -------------------------------------------------------------------------- */
void psa_decode_amplifier_0x1a0(const uint8_t *data, uint8_t dlc, jbl_amplifier_state_t *amp) {
    if (!data || !amp || dlc < 8) return;
    memset(amp, 0, sizeof(*amp));

    amp->bass           = data[1] & 0x0F;
    amp->treble         = data[2] & 0x0F;
    amp->balance        = data[3] & 0x0F;
    amp->fader          = data[4] & 0x0F;
    amp->eq_preset      = data[5] & 0x07;
    amp->loudness       = (data[6] & 0x10) != 0;
    amp->speed_vol_comp = data[6] & 0x0F;
    amp->master_volume  = data[7] & 0x1F;
}

size_t build_raise_amplifier(const jbl_amplifier_state_t *st, uint8_t *out, size_t max_len) {
    if (!st || !out || max_len < 12) return 0;
    out[0] = 0x2E;
    out[1] = 0x56;
    out[2] = 0x08;
    out[3] = 0x00; /* Fixed padding */
    out[4] = (uint8_t)(st->bass & 0x0F);
    out[5] = (uint8_t)(st->treble & 0x0F);
    out[6] = (uint8_t)(st->balance & 0x0F);
    out[7] = (uint8_t)(st->fader & 0x0F);
    out[8] = (uint8_t)(st->eq_preset & 0x07);
    out[9] = (uint8_t)((st->loudness ? 0x10 : 0x00) | (st->speed_vol_comp & 0x0F));
    out[10] = (uint8_t)(st->master_volume & 0x1F);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

/* --------------------------------------------------------------------------
 * 1.8 RD4 Radio & CD Changer Media Data
 * -------------------------------------------------------------------------- */
void psa_decode_cd_changer_0x3a6(const uint8_t *data, uint8_t dlc, cd_changer_state_t *cdc) {
    if (!data || !cdc || dlc < 7) return;
    memset(cdc, 0, sizeof(*cdc));

    cdc->disc_slot    = data[1];
    cdc->track_num    = data[2];
    cdc->total_tracks = data[3];
    cdc->elapsed_min  = data[4];
    cdc->elapsed_sec  = data[5];
    cdc->play_flags   = data[6];
}

size_t build_raise_cd_changer(const cd_changer_state_t *st, uint8_t *out, size_t max_len) {
    if (!st || !out || max_len < 11) return 0;
    out[0] = 0x2E;
    out[1] = 0x54;
    out[2] = 0x07;
    out[3] = 0x02; /* CD Player Mode */
    out[4] = st->disc_slot;
    out[5] = st->track_num;
    out[6] = st->total_tracks;
    out[7] = st->elapsed_min;
    out[8] = st->elapsed_sec;
    out[9] = st->play_flags;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out[i];
    }
    out[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

void psa_decode_rds_name_0x396(const uint8_t *data, uint8_t dlc, char out_name[9]) {
    if (!data || !out_name) return;
    size_t len = (dlc > 8) ? 8 : (size_t)dlc;
    memcpy(out_name, data, len);
    out_name[len] = '\0';
}

size_t build_raise_rds_name(const char *name, uint8_t *out, size_t max_len) {
    if (!out || max_len < 12) return 0;
    out[0] = 0x2E;
    out[1] = 0x55;
    out[2] = 0x08;

    size_t nlen = (name != NULL) ? strlen(name) : 0;
    for (size_t i = 0; i < 8; i++) {
        out[3 + i] = (i < nlen && name[i] != '\0') ? (uint8_t)name[i] : (uint8_t)' ';
    }

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

/* --------------------------------------------------------------------------
 * 2.1 Direct TPMS Numeric Readings & Fault Classification
 * -------------------------------------------------------------------------- */
size_t build_raise_tpms_numeric(const canbox_tpms_state_t *tpms, uint8_t *out) {
    if (!tpms || !out) {
        return 0;
    }
    out[0] = 0x2E;
    out[1] = 0x66;
    out[2] = 0x06;
    out[3] = 0x01; /* Real-time mode */
    out[4] = (uint8_t)tpms->pressure_dbar[0];
    out[5] = (uint8_t)tpms->pressure_dbar[1];
    out[6] = (uint8_t)tpms->pressure_dbar[2];
    out[7] = (uint8_t)tpms->pressure_dbar[3];
    out[8] = 0x00; /* Unit: Bar * 0.1 */

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_tpms_temp_alarms(const canbox_tpms_state_t *tpms, uint8_t *out) {
    if (!tpms || !out) {
        return 0;
    }
    out[0] = 0x2E;
    out[1] = 0x68;
    out[2] = 0x08;
    for (int i = 0; i < 4; i++) {
        int16_t t = tpms->temperature_c[i] + 40;
        out[3 + i] = (t < 0) ? 0 : ((t > 255) ? 255 : (uint8_t)t);
    }
    for (int i = 0; i < 4; i++) {
        out[7 + i] = tpms->alarm_code[i];
    }
    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) {
        sum += out[i];
    }
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_raise_tpms_discrete(const uint8_t alarms[4], uint8_t *out) {
    if (!alarms || !out) {
        return 0;
    }
    out[0] = 0x2E;
    out[1] = 0x18;
    out[2] = 0x04;
    out[3] = alarms[0];
    out[4] = alarms[1];
    out[5] = alarms[2];
    out[6] = alarms[3];

    uint8_t sum = 0;
    for (size_t i = 1; i <= 6; i++) {
        sum += out[i];
    }
    out[7] = (uint8_t)(sum ^ 0xFF);
    return 8;
}

/* --------------------------------------------------------------------------
 * 2.2 Stop & Start (S&S) Telemetry & Timer
 * -------------------------------------------------------------------------- */
size_t build_raise_start_stop(bool is_active, uint32_t stop_time_sec, uint8_t *out) {
    if (!out) {
        return 0;
    }
    out[0] = 0x2E;
    out[1] = 0x71;
    out[2] = 0x05;
    out[3] = is_active ? 0x01 : 0x00;
    out[4] = (uint8_t)((stop_time_sec >> 24) & 0xFF);
    out[5] = (uint8_t)((stop_time_sec >> 16) & 0xFF);
    out[6] = (uint8_t)((stop_time_sec >> 8) & 0xFF);
    out[7] = (uint8_t)(stop_time_sec & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 7; i++) {
        sum += out[i];
    }
    out[8] = (uint8_t)(sum ^ 0xFF);
    return 9;
}

/* --------------------------------------------------------------------------
 * 2.3 Cruise Control & Speed Memory Presets
 * -------------------------------------------------------------------------- */
size_t build_raise_cruise_memory(bool active, uint8_t target_spd, const uint8_t presets[5], uint8_t *out) {
    if (!out) {
        return 0;
    }
    out[0] = 0x2E;
    out[1] = 0x72;
    out[2] = 0x07;
    out[3] = active ? 0x01 : 0x00;
    out[4] = target_spd;
    if (presets) {
        memcpy(&out[5], presets, 5);
    } else {
        memset(&out[5], 0, 5);
    }

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out[i];
    }
    out[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}

void psa_decode_cruise_0x1a8(const uint8_t *data, uint8_t dlc, bool *active, uint8_t *set_speed_kmh, uint32_t *partial_odo_m) {
    if (!data || dlc < 3) {
        return;
    }
    if (active) {
        uint8_t status = (data[0] >> 3) & 0x07;
        *active = (status == 1 || status == 2);
    }
    if (set_speed_kmh) {
        uint16_t spd_raw = read_be16(&data[1]);
        if (spd_raw == 0xFFFF) {
            *set_speed_kmh = 0;
        } else {
            *set_speed_kmh = (uint8_t)(spd_raw / 100);
        }
    }
    if (partial_odo_m && dlc >= 8) {
        uint32_t odo_raw = ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | (uint32_t)data[7];
        *partial_odo_m = (odo_raw == 0xFFFFFF) ? 0 : odo_raw;
    }
}

/* --------------------------------------------------------------------------
 * 2.4 Driver Assistance & ADAS Features
 * -------------------------------------------------------------------------- */
size_t build_raise_adas(const canbox_adas_state_t *adas, uint8_t *out) {
    if (!adas || !out) {
        return 0;
    }
    out[0] = 0x2E;
    out[1] = 0x70;
    out[2] = 0x06;
    out[3] = adas->blind_spot_warning ? 0x80 : 0x00;
    out[4] = adas->fatigue_coffee_cup ? 0x80 : 0x00;
    out[5] = adas->lane_departure_state;
    out[6] = adas->speed_limit_tsr;
    out[7] = adas->esp_active ? 0x01 : 0x00;
    out[8] = adas->aeb_risk_level;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) {
        sum += out[i];
    }
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

void psa_decode_alerts_0x168(const uint8_t *data, uint8_t dlc,
                             bool *tpms_fault, bool *tpms_underinflation,
                             bool *tpms_puncture, bool *esp_fault) {
    if (!data) return;
    if (tpms_fault && dlc >= 1) {
        *tpms_fault = (data[0] & 0x01) != 0;
    }
    if (tpms_underinflation && dlc >= 2) {
        *tpms_underinflation = (data[1] & 0x80) != 0;
    }
    if (tpms_puncture && dlc >= 2) {
        *tpms_puncture = (data[1] & 0x40) != 0;
    }
    if (esp_fault && dlc >= 4) {
        *esp_fault = (data[3] & 0x10) != 0;
    }
}

