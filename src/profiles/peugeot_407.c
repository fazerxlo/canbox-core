#include "profiles/peugeot_407.h"
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
    if (!climate || !data || dlc < 7) return;

    climate->power_on = (data[0] != 0xA8);
    climate->ac_on = climate->power_on;

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

    if ((data[4] & 0x10) != 0) {
        climate->recirculate = true;
        climate->aqs_auto = false;
    } else if ((data[4] & 0x20) != 0) {
        climate->recirculate = false;
        climate->aqs_auto = false;
    } else {
        climate->recirculate = false;
        climate->aqs_auto = true;
    }
    climate->rear_defrost = (data[4] & 0x01) != 0;

    uint8_t l_temp = data[5];
    uint8_t r_temp = data[6];
    if (l_temp < 23) climate->temp_driver = peugeot_temp_map[l_temp];
    if (r_temp < 23) climate->temp_passenger = peugeot_temp_map[r_temp];
}

void psa_hvac_process_can_0x1e3(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc) {
    if (!climate || !data || dlc < 2) return;

    climate->auto_mode = ((data[0] & 0x0C) == 0x0C);
    climate->dual_mode = (data[0] & 0x01) != 0;

    if ((data[0] & 0x80) != 0) {
        climate->recirculate = true;
        climate->aqs_auto = false;
    } else if ((data[0] & 0x10) != 0) {
        climate->recirculate = false;
        climate->aqs_auto = true;
    } else {
        climate->recirculate = false;
        climate->aqs_auto = false;
    }

    climate->front_max_defrost = (data[1] & 0x80) != 0;

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

size_t build_hiworld_radar(const psa_radar_state_t *radar, uint8_t *out, size_t max_len) {
    if (!radar || !out || max_len < 17) {
        return 0;
    }

    out[0] = HIWORLD_SOF1;             /* 0x5A */
    out[1] = HIWORLD_SOF2;             /* 0xA5 */
    out[2] = 0x0C;                     /* Length: 12 Payload bytes */
    out[3] = HIWORLD_CMD_RADAR_STATE;  /* Cmd: 0x41 */

    out[4] = radar->rear_left_outer;
    out[5] = radar->rear_left_center;
    out[6] = radar->rear_right_center;
    out[7] = radar->rear_right_outer;

    out[8]  = radar->front_left_outer;
    out[9]  = radar->front_left_center;
    out[10] = radar->front_right_center;
    out[11] = radar->front_right_outer;

    out[12] = 0x01;                    /* Radar active / enabled flag */
    out[13] = 0x00;                    /* Reserved */
    out[14] = 0x3F;                    /* 6-sensor configuration mask */
    out[15] = 0x05;                    /* Distance scale / max zone steps */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 15; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[16] = (uint8_t)((sum - 1) & 0xFF);

    return 17;
}

void psa_radar_init(psa_radar_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->state.rear_left_outer    = PSA_RADAR_DIST_INACTIVE;
    ctx->state.rear_left_center   = PSA_RADAR_DIST_INACTIVE;
    ctx->state.rear_right_center  = PSA_RADAR_DIST_INACTIVE;
    ctx->state.rear_right_outer   = PSA_RADAR_DIST_INACTIVE;
    ctx->state.front_left_outer   = PSA_RADAR_DIST_INACTIVE;
    ctx->state.front_left_center  = PSA_RADAR_DIST_INACTIVE;
    ctx->state.front_right_center = PSA_RADAR_DIST_INACTIVE;
    ctx->state.front_right_outer  = PSA_RADAR_DIST_INACTIVE;
    ctx->uart_tx = uart_tx;
}

void psa_radar_send_hiworld(psa_radar_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[17];
    size_t len = build_hiworld_radar(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

uint8_t psa_radar_map_zone(uint8_t raw3bit) {
    if (raw3bit >= 7) return PSA_RADAR_DIST_INACTIVE;
    return raw3bit;
}

void psa_radar_process_can_0x0e1(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 6) return;

    ctx->state.rear_active    = (data[1] >> 6) & 1;
    ctx->state.front_active   = (data[1] >> 4) & 1;
    ctx->state.display_active = (data[5] & 0x02) ? true : false;

    if (!ctx->state.display_active && !ctx->state.rear_active && !ctx->state.front_active) {
        ctx->state.rear_left_outer    = PSA_RADAR_DIST_INACTIVE;
        ctx->state.rear_left_center   = PSA_RADAR_DIST_INACTIVE;
        ctx->state.rear_right_center  = PSA_RADAR_DIST_INACTIVE;
        ctx->state.rear_right_outer   = PSA_RADAR_DIST_INACTIVE;
        ctx->state.front_left_outer   = PSA_RADAR_DIST_INACTIVE;
        ctx->state.front_left_center  = PSA_RADAR_DIST_INACTIVE;
        ctx->state.front_right_center = PSA_RADAR_DIST_INACTIVE;
        ctx->state.front_right_outer  = PSA_RADAR_DIST_INACTIVE;
    } else {
        uint8_t rl = (data[3] >> 5) & 0x07;
        uint8_t rc = (data[3] >> 2) & 0x07;
        uint8_t rr = (data[4] >> 5) & 0x07;
        uint8_t fl = (data[4] >> 2) & 0x07;
        uint8_t fc = (data[5] >> 5) & 0x07;
        uint8_t fr = (data[5] >> 2) & 0x07;

        ctx->state.rear_left_outer    = psa_radar_map_zone(rl);
        ctx->state.rear_left_center   = psa_radar_map_zone(rc);
        ctx->state.rear_right_center  = psa_radar_map_zone(rc);
        ctx->state.rear_right_outer   = psa_radar_map_zone(rr);

        ctx->state.front_left_outer   = psa_radar_map_zone(fl);
        ctx->state.front_left_center  = psa_radar_map_zone(fc);
        ctx->state.front_right_center = psa_radar_map_zone(fc);
        ctx->state.front_right_outer  = psa_radar_map_zone(fr);
    }

    psa_radar_send_hiworld(ctx);
}

void psa_radar_process_can_0x260(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 3) return;

    ctx->state.rear_left_outer   = data[0];
    ctx->state.rear_left_center  = data[1];
    ctx->state.rear_right_center = data[1];
    ctx->state.rear_right_outer  = data[2];

    if (dlc >= 4) {
        ctx->state.rear_active  = (data[3] & 0x80) ? true : false;
        ctx->state.system_fault = (data[3] & 0x01) ? true : false;
    }

    psa_radar_send_hiworld(ctx);
}

void psa_radar_process_can_0x270(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 3) return;

    ctx->state.front_left_outer   = data[0];
    ctx->state.front_left_center  = data[1];
    ctx->state.front_right_center = data[1];
    ctx->state.front_right_outer  = data[2];

    psa_radar_send_hiworld(ctx);
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

void psa_decode_reverse_0x036(const uint8_t *data, uint8_t dlc, bool *reverse_active) {
    if (!data || !reverse_active || dlc < 2) return;
    *reverse_active = (data[1] & 0x80) != 0;
}

void psa_decode_reverse_0x0f6(const uint8_t *data, uint8_t dlc, bool *reverse_active) {
    if (!data || !reverse_active || dlc < 8) return;
    *reverse_active = (data[7] & 0x80) != 0;
}

void psa_reverse_set_hardware_trigger(bool reverse_active) {
    hal_gpio_write(GPIO_PIN_REVERSE_OUT, reverse_active);
}

size_t build_hiworld_trip_instant(const psa_trip_state_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 15) {
        return 0;
    }
    out[0] = HIWORLD_SOF1;
    out[1] = HIWORLD_SOF2;
    out[2] = 0x0A;               /* Length: 10 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P0; /* Cmd 0x13 */
    out[4] = (uint8_t)(trip->instant_fuel_deci >> 8);
    out[5] = (uint8_t)(trip->instant_fuel_deci & 0xFF);
    out[6] = (uint8_t)(trip->range_km >> 8);
    out[7] = (uint8_t)(trip->range_km & 0xFF);
    out[8] = (uint8_t)(trip->dest_dist_km >> 8);
    out[9] = (uint8_t)(trip->dest_dist_km & 0xFF);
    out[10] = 0x00;
    out[11] = 0x00;
    out[12] = 0x00;
    out[13] = 0x00;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 13; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[14] = (uint8_t)((sum - 1) & 0xFF);
    return 15;
}

size_t build_hiworld_trip1(const psa_trip_state_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 11) {
        return 0;
    }
    out[0] = HIWORLD_SOF1;
    out[1] = HIWORLD_SOF2;
    out[2] = 0x06;               /* Length: 6 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P1; /* Cmd 0x14 */
    out[4] = (uint8_t)(trip->trip1_avg_fuel >> 8);
    out[5] = (uint8_t)(trip->trip1_avg_fuel & 0xFF);
    out[6] = 0x00;               /* Reserved */
    out[7] = trip->trip1_avg_speed;
    out[8] = (uint8_t)(trip->trip1_distance_km >> 8);
    out[9] = (uint8_t)(trip->trip1_distance_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[10] = (uint8_t)((sum - 1) & 0xFF);
    return 11;
}

size_t build_hiworld_trip2(const psa_trip_state_t *trip, uint8_t *out, size_t max_len) {
    if (!trip || !out || max_len < 11) {
        return 0;
    }
    out[0] = HIWORLD_SOF1;
    out[1] = HIWORLD_SOF2;
    out[2] = 0x06;               /* Length: 6 Payload bytes */
    out[3] = HIWORLD_CMD_ECU_P2; /* Cmd 0x15 */
    out[4] = (uint8_t)(trip->trip2_avg_fuel >> 8);
    out[5] = (uint8_t)(trip->trip2_avg_fuel & 0xFF);
    out[6] = 0x00;               /* Reserved */
    out[7] = trip->trip2_avg_speed;
    out[8] = (uint8_t)(trip->trip2_distance_km >> 8);
    out[9] = (uint8_t)(trip->trip2_distance_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[10] = (uint8_t)((sum - 1) & 0xFF);
    return 11;
}

void psa_trip_init(psa_trip_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(psa_trip_ctx_t));
    ctx->uart_tx = uart_tx;
}

void psa_trip_send_instant(psa_trip_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[16];
    size_t len = build_hiworld_trip_instant(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

void psa_trip_send_trip1(psa_trip_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[11];
    size_t len = build_hiworld_trip1(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

void psa_trip_send_trip2(psa_trip_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[11];
    size_t len = build_hiworld_trip2(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

void psa_trip_process_can_0x0b6(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;
    uint16_t raw_rpm = read_be16(&data[0]);
    uint16_t raw_spd = read_be16(&data[2]);

    ctx->state.rpm = (raw_rpm == 0xFFFF) ? 0 : (raw_rpm >> 3);
    ctx->state.speed_kmh = (raw_spd == 0xFFFF) ? 0 : (raw_spd / 100);
}

void psa_trip_process_can_0x221(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 7) return;

    ctx->state.instant_fuel_deci = read_be16(&data[1]);
    ctx->state.range_km          = read_be16(&data[3]);
    ctx->state.dest_dist_km      = read_be16(&data[5]);

    psa_trip_send_instant(ctx);
}

void psa_trip_process_can_0x2a1(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 5) return;

    ctx->state.trip1_distance_km = read_be16(&data[1]);
    ctx->state.trip1_avg_fuel    = read_be16(&data[3]);
    ctx->state.trip1_avg_speed   = (dlc >= 7 && (data[5] || data[6])) ? (uint8_t)read_be16(&data[5]) : data[0];

    psa_trip_send_trip1(ctx);
}

void psa_trip_process_can_0x261(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 5) return;

    ctx->state.trip2_distance_km = read_be16(&data[1]);
    ctx->state.trip2_avg_fuel    = read_be16(&data[3]);
    ctx->state.trip2_avg_speed   = (dlc >= 7 && (data[5] || data[6])) ? (uint8_t)read_be16(&data[5]) : data[0];

    psa_trip_send_trip2(ctx);
}

void psa_trip_process_can_0x0f6(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 8) return;

    ctx->state.coolant_c = (int8_t)((int16_t)data[1] - 40);
    if (data[5] != 0xFF) {
        ctx->state.ambient_c = (int8_t)(((int16_t)data[5] * 5 - 400) / 10);
    }
    ctx->state.reverse_active = (data[7] & 0x80) != 0;
}

bool build_psa_trip_reset_frame(uint8_t trip_index, can_frame_t *out_frame) {
    if (!out_frame || (trip_index != 1 && trip_index != 2)) {
        return false;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->id          = PSA_CAN_ID_TRIP_INSTANT; /* 0x221 (MSG_DEMANDES_EMF) */
    out_frame->dlc         = 8;
    out_frame->is_extended = false;
    out_frame->is_remote   = false;

    if (trip_index == 1) {
        out_frame->data[0] = 0x80; /* Bit 7: Trip 1 Reset */
    } else {
        out_frame->data[0] = 0x40; /* Bit 6: Trip 2 Reset */
    }

    return true;
}

hal_status_t psa_trip_send_reset(uint8_t trip_index) {
    can_frame_t frame;
    if (!build_psa_trip_reset_frame(trip_index, &frame)) {
        return HAL_STATUS_ERROR;
    }

    return hal_can_send(&frame);
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
void psa_tpms_init(psa_tpms_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(psa_tpms_ctx_t));
    ctx->state.fl_press_bar_deci = 24; /* 2.4 Bar default */
    ctx->state.fr_press_bar_deci = 24;
    ctx->state.rr_press_bar_deci = 22; /* 2.2 Bar default */
    ctx->state.rl_press_bar_deci = 22;
    ctx->uart_tx = uart_tx;
}

size_t build_hiworld_tpms_numeric(const psa_tpms_state_t *tpms, uint8_t *out, size_t max_len) {
    if (!tpms || !out || max_len < 11) {
        return 0;
    }
    out[0] = HIWORLD_SOF1;
    out[1] = HIWORLD_SOF2;
    out[2] = 0x06;                     /* Length: 6 Payload bytes */
    out[3] = HIWORLD_CMD_TPMS_NUMERIC; /* Cmd 0x66 */
    out[4] = 0x01;                     /* Mode: Live */
    out[5] = tpms->fl_press_bar_deci;
    out[6] = tpms->fr_press_bar_deci;
    out[7] = tpms->rl_press_bar_deci;
    out[8] = tpms->rr_press_bar_deci;
    out[9] = 0x00;                     /* Unit: Bar */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[10] = (uint8_t)((sum - 1) & 0xFF);
    return 11;
}

size_t build_hiworld_tpms_discrete(const psa_tpms_state_t *tpms, uint8_t *out, size_t max_len) {
    if (!tpms || !out || max_len < 9) {
        return 0;
    }
    out[0] = HIWORLD_SOF1;
    out[1] = HIWORLD_SOF2;
    out[2] = 0x04;                      /* Length: 4 Payload bytes */
    out[3] = HIWORLD_CMD_TPMS_DISCRETE; /* Cmd 0x18 */
    out[4] = tpms->fl_state;
    out[5] = tpms->fr_state;
    out[6] = tpms->rl_state;
    out[7] = tpms->rr_state;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 7; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[8] = (uint8_t)((sum - 1) & 0xFF);
    return 9;
}

void psa_tpms_send_numeric(psa_tpms_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[11];
    size_t len = build_hiworld_tpms_numeric(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

void psa_tpms_send_discrete(psa_tpms_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t p[9];
    size_t len = build_hiworld_tpms_discrete(&ctx->state, p, sizeof(p));
    if (len > 0) {
        ctx->uart_tx(p, len);
    }
}

void psa_tpms_process_can_0x361(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 8) return;

    uint16_t fl_raw = read_be16(&data[0]);
    uint16_t fr_raw = read_be16(&data[2]);
    uint16_t rr_raw = read_be16(&data[4]);
    uint16_t rl_raw = read_be16(&data[6]);

    ctx->state.fl_state = (uint8_t)((fl_raw >> 14) & 0x03);
    ctx->state.fr_state = (uint8_t)((fr_raw >> 14) & 0x03);
    ctx->state.rr_state = (uint8_t)((rr_raw >> 14) & 0x03);
    ctx->state.rl_state = (uint8_t)((rl_raw >> 14) & 0x03);

    if (ctx->state.fl_state != 3 && (fl_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.fl_press_bar_deci = (uint8_t)(fl_raw & 0x3FFF);
    } else {
        ctx->state.fl_press_bar_deci = 0;
    }
    if (ctx->state.fr_state != 3 && (fr_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.fr_press_bar_deci = (uint8_t)(fr_raw & 0x3FFF);
    } else {
        ctx->state.fr_press_bar_deci = 0;
    }
    if (ctx->state.rr_state != 3 && (rr_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.rr_press_bar_deci = (uint8_t)(rr_raw & 0x3FFF);
    } else {
        ctx->state.rr_press_bar_deci = 0;
    }
    if (ctx->state.rl_state != 3 && (rl_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.rl_press_bar_deci = (uint8_t)(rl_raw & 0x3FFF);
    } else {
        ctx->state.rl_press_bar_deci = 0;
    }

    psa_tpms_send_numeric(ctx);
    psa_tpms_send_discrete(ctx);
}

void psa_tpms_process_can_0x3a1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;

    ctx->state.fl_press_bar_deci = (uint8_t)((data[0] * 5 + 5) / 10);
    ctx->state.fr_press_bar_deci = (uint8_t)((data[1] * 5 + 5) / 10);
    ctx->state.rr_press_bar_deci = (uint8_t)((data[2] * 5 + 5) / 10);
    ctx->state.rl_press_bar_deci = (uint8_t)((data[3] * 5 + 5) / 10);

    psa_tpms_send_numeric(ctx);
}

void psa_tpms_process_can_0x1e1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;

    /* PSA 0x1E1 bits 5..3 carry RT4 tyre state enum:
       0=OK, 1=LOW, 2=FLAT, 3=NO_DATA, 4=BATTERY_LOW */
    uint8_t fl_st = (uint8_t)((data[0] >> 3) & 0x07);
    uint8_t fr_st = (uint8_t)((data[1] >> 3) & 0x07);
    uint8_t rr_st = (uint8_t)((data[2] >> 3) & 0x07);
    uint8_t rl_st = (uint8_t)((data[3] >> 3) & 0x07);

    /* Map PSA enum to Hiworld discrete alarm:
       0=Normal, 1=Low, 2=Puncture, 3=Fault/Offline */
    ctx->state.fl_state = (fl_st >= 3) ? 3 : fl_st;
    ctx->state.fr_state = (fr_st >= 3) ? 3 : fr_st;
    ctx->state.rr_state = (rr_st >= 3) ? 3 : rr_st;
    ctx->state.rl_state = (rl_st >= 3) ? 3 : rl_st;

    psa_tpms_send_discrete(ctx);
}

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

/* --------------------------------------------------------------------------
 * 2.5 Vehicle Alerts & Diagnostic Journal (Hiworld Command 0x42)
 * -------------------------------------------------------------------------- */
#define TOTAL_ALARM_ENTRIES 208

static const uint16_t Alarm_IndexToPointer_Tab[TOTAL_ALARM_ENTRIES] = {
    [0x00] = 0x000C, [0x01] = 0x000B, [0x02] = 0x019C, [0x03] = 0x0008,
    [0x04] = 0x019B, [0x05] = 0x019A, [0x06] = 0x0002, [0x07] = 0x0001,
    [0x08] = 0x000F, [0x09] = 0x000A, [0x0A] = 0x000E, [0x0B] = 0xFFFF,
    [0x0C] = 0x000D, [0x0D] = 0x0003, [0x0E] = 0x0005, [0x0F] = 0x0004,
    [0x10] = 0x007F, [0x11] = 0x0067, [0x12] = 0xFFFF, [0x13] = 0x006E,
    [0x14] = 0x006B, [0x15] = 0x006A, [0x16] = 0x006C, [0x17] = 0x0066,
    [0x18] = 0xFFFF, [0x19] = 0xFFFF, [0x1A] = 0x006F, [0x1B] = 0x007E,
    [0x1C] = 0xFFFF, [0x1D] = 0x0073, [0x1E] = 0x0072, [0x1F] = 0x007D,
    [0x20] = 0x0087, [0x21] = 0x0075, [0x22] = 0x0086, [0x23] = 0x0074,
    [0x24] = 0x0085, [0x25] = 0x0084, [0x26] = 0x0081, [0x27] = 0x0080,
    [0x28] = 0x00D9, [0x29] = 0x00D2, [0x2A] = 0x00D0, [0x2B] = 0x00DF,
    [0x2C] = 0x00CB, [0x2D] = 0x00CA, [0x2E] = 0x00C9, [0x2F] = 0xFFFF,
    [0x30] = 0xFFFF, [0x31] = 0xFFFF, [0x32] = 0x00E4, [0x33] = 0x00E3,
    [0x34] = 0xFFFF, [0x35] = 0x00D7, [0x36] = 0xFFFF, [0x37] = 0x00E0,
    [0x38] = 0xFFFF, [0x39] = 0xFFFF, [0x3A] = 0xFFFF, [0x3B] = 0xFFFF,
    [0x3C] = 0xFFFF, [0x3D] = 0x00E5, [0x3E] = 0xFFFF, [0x3F] = 0xFFFF,
    [0x40] = 0x0135, [0x41] = 0x013A, [0x42] = 0x0137, [0x43] = 0x0133,
    [0x44] = 0x0131, [0x45] = 0x012F, [0x46] = 0x0083, [0x47] = 0x012E,
    [0x48] = 0xFFFF, [0x49] = 0xFFFF, [0x4A] = 0xFFFF, [0x4B] = 0xFFFF,
    [0x4C] = 0x00E2, [0x4D] = 0x0082, [0x4E] = 0xFFFF, [0x4F] = 0xFFFF,
    [0x50] = 0x0198, [0x51] = 0x0195, [0x52] = 0x0194, [0x53] = 0xFFFF,
    [0x54] = 0xFFFF, [0x55] = 0x0193, [0x56] = 0x0192, [0x57] = 0x0191,
    [0x58] = 0xFFFF, [0x59] = 0xFFFF, [0x5A] = 0xFFFF, [0x5B] = 0xFFFF,
    [0x5C] = 0x0202, [0x5D] = 0x020B, [0x5E] = 0x020A, [0x5F] = 0xFFFF,
    [0x60] = 0x0209, [0x61] = 0x0206, [0x62] = 0x0205, [0x63] = 0x0204,
    [0x64] = 0x0203, [0x65] = 0xFFFF, [0x66] = 0xFFFF, [0x67] = 0xFFFF,
    [0x68] = 0x01FB, [0x69] = 0x01FA, [0x6A] = 0x01F9, [0x6B] = 0x01F8,
    [0x6C] = 0x01F7, [0x6D] = 0x01F6, [0x6E] = 0x01FE, [0x6F] = 0x01F5,
    [0x70] = 0xFFFF, [0x71] = 0xFFFF, [0x72] = 0xFFFF, [0x73] = 0xFFFF,
    [0x74] = 0x007C, [0x75] = 0x01FD, [0x76] = 0xFFFF, [0x77] = 0x01FC,
    [0x78] = 0x0012, [0x79] = 0x0068, [0x7A] = 0x0069, [0x7B] = 0x006D,
    [0x7C] = 0x0013, [0x7D] = 0x0078, [0x7E] = 0x0088, [0x7F] = 0x0089,
    [0x80] = 0x008A, [0x81] = 0x0076, [0x82] = 0x008C, [0x83] = 0x008D,
    [0x84] = 0x0222, [0x85] = 0x0221, [0x86] = 0x0220, [0x87] = 0x0091,
    [0x88] = 0x00CD, [0x89] = 0x00D1, [0x8A] = 0x00D4, [0x8B] = 0x00D8,
    [0x8C] = 0x00DE, [0x8D] = 0x00E1, [0x8E] = 0x00E6, [0x8F] = 0x00EC,
    [0x90] = 0x00ED, [0x91] = 0x00EE, [0x92] = 0x00EF, [0x93] = 0x012D,
    [0x94] = 0x0130, [0x95] = 0x0132, [0x96] = 0x0136, [0x97] = 0x0138,
    [0x98] = 0x0139, [0x99] = 0x013B, [0x9A] = 0x013C, [0x9B] = 0x00DA,
    [0x9C] = 0x0079, [0x9D] = 0x00E7, [0x9E] = 0x00E9, [0x9F] = 0x00EA,
    [0xA0] = 0x00EB, [0xA1] = 0x00D3, [0xA2] = 0x013D, [0xA3] = 0x013E,
    [0xA4] = 0x00CE, [0xA5] = 0x0134, [0xA6] = 0x0064, [0xA7] = 0x007A,
    [0xA8] = 0x0092, [0xA9] = 0x0095, [0xAA] = 0x0096, [0xAB] = 0x0097,
    [0xAC] = 0x009A, [0xAD] = 0x009B, [0xAE] = 0x009C, [0xAF] = 0x009D,
    [0xB0] = 0x009E, [0xB1] = 0x009F, [0xB2] = 0x00A0, [0xB3] = 0x00D5,
    [0xB4] = 0x00D6, [0xB5] = 0x00E8, [0xB6] = 0x013F, [0xB7] = 0x0140,
    [0xB8] = 0x0196, [0xB9] = 0x0197, [0xBA] = 0x0199, [0xBB] = 0x0011,
    [0xBC] = 0x0063, [0xBD] = 0x00A1, [0xBE] = 0x00F0, [0xBF] = 0x00F1,
    [0xC0] = 0x00F2, [0xC1] = 0x00F3, [0xC2] = 0x00F4, [0xC3] = 0x00F5,
    [0xC4] = 0x00F6, [0xC5] = 0x00F7, [0xC6] = 0x00F8, [0xC7] = 0x00F9,
    [0xC8] = 0x01FF, [0xC9] = 0x0200, [0xCA] = 0x0201, [0xCB] = 0x0061,
    [0xCC] = 0x0062, [0xCD] = 0x007B, [0xCE] = 0xFFFF, [0xCF] = 0xFFFF
};

static const uint16_t Alarm_BitToIndex_Tab[PSA_JOURNAL_TOTAL_BITS] = {
    0x0E, 0x07, 0xFFFF, 0x03, 0xFFFF, 0x7B, 0x0D, 0xFFFF, 0x0F, 0x28, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x14, 0x80, 0xFFFF, 0x1F, 0x11, 0x37, 0x7D,
    0x27, 0x10, 0x15, 0x08, 0xFFFF, 0x29, 0x26, 0x13, 0x16, 0x1A, 0x81, 0x1B,
    0x1D, 0x1E, 0x2C, 0x20, 0x21, 0x22, 0x8C, 0x8C, 0x8C, 0x82, 0x8C, 0x8C,
    0x8C, 0x8C, 0x8C, 0x84, 0x8C, 0x8C, 0x8C, 0x2D, 0x2E, 0x2E, 0x2E, 0x2E,
    0x32, 0x33, 0x35, 0xB2, 0xB2, 0xB2, 0xB2, 0xAC, 0xAC, 0xAD, 0xAD, 0xAE,
    0xAE, 0xAF, 0xAF, 0xAF, 0xAF, 0xB0, 0xB0, 0xB0, 0xB0, 0xB1, 0xB1, 0x47,
    0x4C, 0x4D, 0x50, 0x46, 0x51, 0x52, 0x9C, 0x7E, 0x7F, 0x55, 0x83, 0x83,
    0x83, 0x83, 0x56, 0x57, 0x89, 0x5C, 0x5D, 0xA1, 0xA1, 0xA1, 0x5E, 0x8B,
    0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x60, 0x8E, 0x9D, 0x91,
    0x99, 0x9A, 0x61, 0x79, 0x7A, 0x62, 0x63, 0x9E, 0x9F, 0xA0, 0x8F, 0x92,
    0x64, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0x6B, 0xB5, 0xA6, 0x68, 0xBB, 0x69,
    0x6A, 0xC5, 0xC6, 0xC7, 0x6C, 0x6D, 0x6E, 0x6F, 0x78, 0x7C, 0x74, 0x75,
    0x77, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

uint16_t psa_can_alarm_id_to_hiworld_code(uint16_t can_alarm_id, uint8_t door_mask, uint8_t param) {
    switch (can_alarm_id) {
        case 0x000D: return PSA_HIWORLD_ALERT_LOW_FUEL;           /* Low fuel */
        case 0x0001: return PSA_HIWORLD_ALERT_ENGINE_TEMP;        /* Engine temp too high */
        case 0x0002: return PSA_HIWORLD_ALERT_OIL_PRESSURE;       /* Oil pressure low */
        case 0x0003: return PSA_HIWORLD_ALERT_OIL_PRESSURE;       /* Check engine oil level */
        case 0x000F: return PSA_HIWORLD_ALERT_BRAKE_FLUID;        /* Brake system faulty */
        case 0x000C: return PSA_HIWORLD_ALERT_HANDBRAKE;          /* Handbrake on */
        case 0x00DF: return PSA_HIWORLD_ALERT_KEY_BATTERY;        /* Remote key battery flat */
        case 0x0195: return PSA_HIWORLD_ALERT_DIRECTIONAL_LIGHTS; /* Directional headlamps faulty */
        case 0x006B: return PSA_HIWORLD_ALERT_BATTERY_CHARGE;     /* Battery charge fault */
        case 0x006A: return PSA_HIWORLD_ALERT_ESP;                /* ESP / ASR faulty */
        case 0x0069: return PSA_HIWORLD_ALERT_ABS;                /* ABS direct code */
        case 0x006C: return PSA_HIWORLD_ALERT_ABS;                /* ABS braking system faulty */
        case 0x006F: return PSA_HIWORLD_ALERT_DPF;                /* DPF risk of clogging */
        case 0x0064: return PSA_HIWORLD_ALERT_DPF;                /* DPF direct code */
        case 0x006E: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution system faulty */
        case 0x0068: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution direct code */
        case 0x007E: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution system error */
        case 0x0073:
        case 0x0202: return PSA_HIWORLD_ALERT_GEARBOX;            /* Gearbox faulty */
        case 0x0067: return PSA_HIWORLD_ALERT_GEARBOX;            /* Gearbox direct code */
        case 0x0061: return PSA_HIWORLD_ALERT_SERVICE_DUE;        /* Service due */
        case 0x009E: return PSA_HIWORLD_ALERT_SUSPENSION_90KMH;   /* Suspension max 90 km/h */
        case 0x0072:
        case 0x00D8:
        case 0x00E2: return PSA_HIWORLD_ALERT_SUSPENSION_SYSTEM;  /* Suspension system error */
        case 0x007F: return PSA_HIWORLD_ALERT_AUTO_LIGHTS;        /* Automatic headlights */
        case 0x0081: return PSA_HIWORLD_ALERT_AUTO_LIGHTS;        /* Auto lights code */
        case 0x00CB: return PSA_HIWORLD_ALERT_AUTO_WIPERS;        /* Automatic wipers */
        case 0x0083: return PSA_HIWORLD_ALERT_AUTO_WIPERS;        /* Auto wipers code */
        case 0x0139: return PSA_HIWORLD_ALERT_AUTO_WIPERS;        /* Auto wipers deactivated */
        case 0x0074: return PSA_HIWORLD_ALERT_DOOR_FL;            /* FL door open */
        case 0x0085: return PSA_HIWORLD_ALERT_DOOR_FR;            /* FR door open */
        case 0x0084: return PSA_HIWORLD_ALERT_DOOR_REAR;          /* Rear door open */
        case 0x0080: return PSA_HIWORLD_ALERT_BOOT;               /* Boot open */
        case 0x0008:                                              /* Door open / Braking system faulty */
            if (door_mask != 0 && door_mask != 0xFF) {
                if (door_mask & 0x01) return PSA_HIWORLD_ALERT_DOOR_FL;
                if (door_mask & 0x02) return PSA_HIWORLD_ALERT_DOOR_FR;
                if (door_mask & 0x0C) return PSA_HIWORLD_ALERT_DOOR_REAR;
                if (door_mask & 0x10) return PSA_HIWORLD_ALERT_BOOT;
            }
            return PSA_HIWORLD_ALERT_HANDBRAKE;                   /* 0x0008: Braking system faulty in Hiworld HU */
        case 0x000B:                                              /* Door open general / Driver seatbelt */
            if (door_mask != 0 && door_mask != 0xFF) {
                if (door_mask & 0x01) return PSA_HIWORLD_ALERT_DOOR_FL;
                if (door_mask & 0x02) return PSA_HIWORLD_ALERT_DOOR_FR;
                if (door_mask & 0x0C) return PSA_HIWORLD_ALERT_DOOR_REAR;
                if (door_mask & 0x10) return PSA_HIWORLD_ALERT_BOOT;
            }
            return PSA_HIWORLD_ALERT_SEATBELT_FL;                 /* Driver seatbelt */
        case 0x0004:                                              /* Tyre pressure low */
            if (param == 1) return PSA_HIWORLD_ALERT_TPMS_UNDER_FR;
            if (param == 2) return PSA_HIWORLD_ALERT_TPMS_UNDER_RR;
            if (param == 3) return PSA_HIWORLD_ALERT_TPMS_UNDER_RL;
            return PSA_HIWORLD_ALERT_TPMS_UNDER_FL;
        case 0x0005:                                              /* Tyre puncture */
            if (param == 1) return PSA_HIWORLD_ALERT_TPMS_PUNCTURE_FR;
            if (param == 2 || param == 3) return PSA_HIWORLD_ALERT_TPMS_PUNCTURE_REAR;
            return PSA_HIWORLD_ALERT_TPMS_PUNCTURE_FL;
        case 0x00C9: return PSA_HIWORLD_ALERT_TPMS_UNDER_FL;      /* Tyre pressure(s) not monitored */
        case 0x00D3:
        case 0x00E3: return PSA_HIWORLD_ALERT_BULB_SIDELIGHT;     /* Sidelight bulb */
        case 0x013D:
        case 0x00E5: return PSA_HIWORLD_ALERT_BULB_DIPPED;        /* Dipped beam bulb */
        case 0x00CE:
        case 0x0134:
        case 0x0097:
        case 0x00E7: return PSA_HIWORLD_ALERT_BULB_BRAKE;         /* Brake light bulb */
        case 0x0092:
        case 0x0095:
        case 0x007A:
        case 0x00E8: return PSA_HIWORLD_ALERT_BULB_REVERSE;       /* Reversing / indicator */
        case 0x0078:
        case 0x00F0: return PSA_HIWORLD_ALERT_AIRBAG;             /* Airbag / pretensioner */
        case 0x012F: return PSA_HIWORLD_ALERT_SEATBELT_FL;        /* Driver seatbelt */
        case 0x013A:
        case 0x0130: return PSA_HIWORLD_ALERT_SEATBELT_FR;        /* Front passenger seatbelt */
        case 0x0086:
        case 0x01FA: return PSA_HIWORLD_ALERT_IMMOBILIZER;        /* Electronic immobilizer */
        default:
            return can_alarm_id;
    }
}

void psa_decode_alert_message_0x1a1(const uint8_t *data, uint8_t dlc, vehicle_alert_item_t *alert) {
    if (!data || !alert || dlc < 4) {
        return;
    }
    bool popup_active = (data[0] & 0x80) != 0;
    uint16_t can_alarm_id = ((uint16_t)(data[0] & 0x7F) << 8) | (uint16_t)data[1];
    bool display_req = (data[2] & 0x80) != 0;
    uint8_t priority = (data[2] >> 4) & 0x07;
    uint8_t sound_id = data[2] & 0x0F;
    uint8_t door_mask = data[3];
    uint8_t param = (dlc >= 5) ? data[4] : 0;

    if (display_req && can_alarm_id != 0) {
        alert->is_active = true;
        alert->can_alarm_id = can_alarm_id;
        alert->display_req = true;
        alert->severity = priority;
        alert->chime_id = sound_id;
        alert->door_mask = door_mask;
        alert->param_detail = param;
        alert->alert_code = psa_can_alarm_id_to_hiworld_code(can_alarm_id, door_mask, param);
    } else if (!display_req || !popup_active || can_alarm_id == 0) {
        alert->is_active = false;
        alert->display_req = false;
        alert->can_alarm_id = 0;
        alert->alert_code = 0;
    }
}

void psa_journal_iso_tp_init(psa_journal_iso_tp_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

static uint8_t psa_journal_extract_codes(const uint8_t *buffer, uint16_t *out_codes, uint8_t max_out) {
    uint8_t count = 0;
    for (uint16_t bit = 0; bit < PSA_JOURNAL_TOTAL_BITS && count < max_out; bit++) {
        uint8_t b_idx = (uint8_t)(bit >> 3);
        uint8_t mask = (uint8_t)(1 << (7 - (bit & 7)));
        if (buffer[b_idx] & mask) {
            uint16_t a_idx = Alarm_BitToIndex_Tab[bit];
            if (a_idx < TOTAL_ALARM_ENTRIES && Alarm_IndexToPointer_Tab[a_idx] != 0xFFFF) {
                uint16_t can_id = Alarm_IndexToPointer_Tab[a_idx];
                uint16_t hw_code = psa_can_alarm_id_to_hiworld_code(can_id, 0, 0);

                bool already_present = false;
                for (uint8_t i = 0; i < count; i++) {
                    if (out_codes[i] == hw_code) {
                        already_present = true;
                        break;
                    }
                }
                if (!already_present) {
                    out_codes[count++] = hw_code;
                }
            }
        }
    }
    return count;
}

bool psa_process_journal_0x120(psa_journal_iso_tp_t *ctx, const uint8_t *data, uint8_t dlc,
                               uint16_t *out_codes, uint8_t *out_count) {
    if (!ctx || !data || !out_codes || !out_count || dlc < 2) {
        return false;
    }

    /* Mode A: Real PSA Block Multiplexing (2-bit block header in Byte 0 [7:6])
     * 0x7C (01b = 1): Block 1 -> buffer[0..6]
     * 0xBC (10b = 2): Block 2 -> buffer[7..13]
     * 0xFC (11b = 3): Block 3 -> buffer[14..20]
     */
    uint8_t block_id = (data[0] >> 6) & 0x03;
    if (dlc >= 8 && block_id >= 1 && block_id <= 3) {
        uint8_t offset = (uint8_t)((block_id - 1) * 7);
        memcpy(&ctx->buffer[offset], &data[1], 7);
        ctx->blocks_received |= (uint8_t)(1 << (block_id - 1));

        if ((ctx->blocks_received & 0x07) == 0x07) {
            *out_count = psa_journal_extract_codes(ctx->buffer, out_codes, CANBOX_MAX_ACTIVE_ALERTS);
            return true;
        }
        return false;
    }

    /* Mode B: ISO-TP Multi-frame Fallback (PCI nibbles 0x10 and 0x20) */
    uint8_t pci = data[0];

    /* First Frame: high nibble 0x1 */
    if ((pci & 0xF0) == 0x10) {
        uint16_t total_len = ((uint16_t)(pci & 0x0F) << 8) | (uint16_t)data[1];
        if (total_len == PSA_JOURNAL_PAYLOAD_BYTES) {
            ctx->in_progress = true;
            ctx->bytes_rx = 0;
            ctx->expected_seq = 1;
            uint8_t chunk = (dlc > 2) ? (dlc - 2) : 0;
            if (chunk > 6) chunk = 6;
            memcpy(&ctx->buffer[0], &data[2], chunk);
            ctx->bytes_rx = chunk;
        }
        return false;
    }

    /* Consecutive Frame: high nibble 0x2 */
    if ((pci & 0xF0) == 0x20 && ctx->in_progress) {
        uint8_t seq = pci & 0x0F;
        if (seq == ctx->expected_seq) {
            ctx->expected_seq = (ctx->expected_seq + 1) & 0x0F;
            uint8_t chunk = (uint8_t)(dlc - 1);
            if (ctx->bytes_rx + chunk > PSA_JOURNAL_PAYLOAD_BYTES) {
                chunk = (uint8_t)(PSA_JOURNAL_PAYLOAD_BYTES - ctx->bytes_rx);
            }
            memcpy(&ctx->buffer[ctx->bytes_rx], &data[1], chunk);
            ctx->bytes_rx += chunk;

            if (ctx->bytes_rx >= PSA_JOURNAL_PAYLOAD_BYTES) {
                ctx->in_progress = false;
                *out_count = psa_journal_extract_codes(ctx->buffer, out_codes, CANBOX_MAX_ACTIVE_ALERTS);
                return true;
            }
        } else {
            /* Out-of-order sequence: abort */
            ctx->in_progress = false;
        }
    }

    return false;
}

size_t build_hiworld_alert_single(uint16_t alert_code, uint8_t *out, size_t max_len) {
    if (!out || max_len < 7) {
        return 0;
    }
    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x02; /* Len = 2 */
    out[3] = HIWORLD_CMD_WARNING_INFO; /* 0x42 */
    out[4] = (uint8_t)(alert_code >> 8);
    out[5] = (uint8_t)(alert_code & 0xFF);

    uint8_t sum = (uint8_t)(out[2] + out[3] + out[4] + out[5]);
    out[6] = (uint8_t)((sum - 1) & 0xFF);
    return 7;
}

size_t build_hiworld_alerts_summary(const uint16_t *codes, uint8_t count, uint8_t *out, size_t max_len) {
    if (!out || max_len < 29) {
        return 0;
    }
    if (count > CANBOX_MAX_ACTIVE_ALERTS) {
        count = CANBOX_MAX_ACTIVE_ALERTS;
    }

    memset(out, 0, 29);
    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x18; /* Len = 24 (0x18) */
    out[3] = HIWORLD_CMD_WARNING_INFO; /* 0x42 */
    /* D0..D2: Category flags / reserved = 0x00 */
    out[4] = 0x00;
    out[5] = 0x00;
    out[6] = 0x00;
    /* D3: mNumber = count */
    out[7] = count;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t offset = (uint8_t)(8 + (i * 2));
        out[offset]     = (uint8_t)(codes[i] >> 8);
        out[offset + 1] = (uint8_t)(codes[i] & 0xFF);
    }

    uint8_t sum = (uint8_t)(out[2] + out[3]);
    for (size_t i = 4; i < 28; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[28] = (uint8_t)((sum - 1) & 0xFF);
    return 29;
}

