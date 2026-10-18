#include "unity.h"
#include "profiles/peugeot_407.h"
#include "profiles/psa_virtual_cdc.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "protocols/hiworld_psa_builders.h"
#include "protocols/raise_psa_builders.h"
#include "hal/hal_gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

bool hal_can_native_get_last_sent_frame(can_frame_t *out_frame);
void hal_can_native_clear_sent_frame(void);

static psa_stalk_state_t s_stalk_state;
static uint8_t s_last_stalk_key_id = 0;
static uint8_t s_last_stalk_key_state = 0;
static int s_stalk_cb_count = 0;

static void test_stalk_key_callback(uint8_t key_id, uint8_t state) {
    s_last_stalk_key_id = key_id;
    s_last_stalk_key_state = state;
    s_stalk_cb_count++;
}

static uint8_t s_last_panel_key_id = 0;
static uint8_t s_last_panel_key_state = 0;
static int s_panel_cb_count = 0;

static void test_panel_key_callback(uint8_t key_id, uint8_t state) {
    s_last_panel_key_id = key_id;
    s_last_panel_key_state = state;
    s_panel_cb_count++;
}

#define MAX_TPMS_TX_PACKETS 4
static uint8_t s_tpms_uart_buf[MAX_TPMS_TX_PACKETS][32];
static size_t  s_tpms_uart_len[MAX_TPMS_TX_PACKETS];
static int     s_tpms_uart_tx_calls = 0;

static void __attribute__((unused)) test_tpms_uart_tx(const uint8_t *buf, size_t len) {
    if (buf && s_tpms_uart_tx_calls < MAX_TPMS_TX_PACKETS && len <= sizeof(s_tpms_uart_buf[0])) {
        memcpy(s_tpms_uart_buf[s_tpms_uart_tx_calls], buf, len);
        s_tpms_uart_len[s_tpms_uart_tx_calls] = len;
        s_tpms_uart_tx_calls++;
    }
}

#define MAX_TRIP_TX_PACKETS 4
static uint8_t s_trip_uart_buf[MAX_TRIP_TX_PACKETS][32];
static size_t  s_trip_uart_len[MAX_TRIP_TX_PACKETS];
static int     s_trip_uart_tx_calls = 0;

static void __attribute__((unused)) test_trip_uart_tx(const uint8_t *buf, size_t len) {
    if (buf && s_trip_uart_tx_calls < MAX_TRIP_TX_PACKETS && len <= sizeof(s_trip_uart_buf[0])) {
        memcpy(s_trip_uart_buf[s_trip_uart_tx_calls], buf, len);
        s_trip_uart_len[s_trip_uart_tx_calls] = len;
        s_trip_uart_tx_calls++;
    }
}

#define MAX_RADAR_TX_PACKETS 8
static uint8_t s_radar_uart_buf[MAX_RADAR_TX_PACKETS][32];
static size_t  s_radar_uart_len[MAX_RADAR_TX_PACKETS];
static int     s_radar_uart_tx_calls = 0;

static void __attribute__((unused)) test_radar_uart_tx(const uint8_t *buf, size_t len) {
    if (buf && s_radar_uart_tx_calls < MAX_RADAR_TX_PACKETS && len <= sizeof(s_radar_uart_buf[0])) {
        memcpy(s_radar_uart_buf[s_radar_uart_tx_calls], buf, len);
        s_radar_uart_len[s_radar_uart_tx_calls] = len;
        s_radar_uart_tx_calls++;
    }
}

void setUp_peugeot_407(void) {
    psa_stalk_init(&s_stalk_state);
    s_last_stalk_key_id = 0;
    s_last_stalk_key_state = 0;
    s_stalk_cb_count = 0;
    s_last_panel_key_id = 0;
    s_last_panel_key_state = 0;
    s_panel_cb_count = 0;
    s_tpms_uart_tx_calls = 0;
    memset(s_tpms_uart_len, 0, sizeof(s_tpms_uart_len));
    memset(s_tpms_uart_buf, 0, sizeof(s_tpms_uart_buf));
    s_trip_uart_tx_calls = 0;
    memset(s_trip_uart_len, 0, sizeof(s_trip_uart_len));
    memset(s_trip_uart_buf, 0, sizeof(s_trip_uart_buf));
    s_radar_uart_tx_calls = 0;
    memset(s_radar_uart_len, 0, sizeof(s_radar_uart_len));
    memset(s_radar_uart_buf, 0, sizeof(s_radar_uart_buf));
}

/* --------------------------------------------------------------------------
 * 1.1 Steering Column Stalk & Buttons Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_stalk_buttons_press_and_release(void) {
    // 1. Volume Up Press (Byte 0 = 0x08)
    uint8_t can_data[8] = { 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    TEST_ASSERT_EQUAL_INT(1, s_stalk_cb_count);

    // Release
    can_data[0] = 0x00;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);

    // 2. Volume Down (Byte 0 = 0x04)
    can_data[0] = 0x04;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 3. Next Track (Byte 0 = 0x02)
    can_data[0] = 0x02;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 4. Prev Track (Byte 0 = 0x01)
    can_data[0] = 0x01;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 5. Source (Byte 0 = 0x40)
    can_data[0] = 0x40;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 6. OK / Click (Byte 0 = 0x10)
    can_data[0] = 0x10;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 7. Dark (Byte 0 = 0x80)
    can_data[0] = 0x80;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 8. ESC (Byte 0 = 0x20)
    can_data[0] = 0x20;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 9. Menu (Byte 1 = 0x40)
    can_data[0] = 0x00;
    can_data[1] = 0x40;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 10. Tel Answer (Byte 2 = 0x01)
    can_data[1] = 0x00;
    can_data[2] = 0x01;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // 11. Tel Hangup (Byte 2 = 0x02)
    can_data[2] = 0x02;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
}

void test_peugeot_407_stalk_rotary_encoder(void) {
    uint8_t can_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // Scroll Up (+1 step: 0x00 -> 0x01)
    can_data[1] = 0x01;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count); // pulse press (1) and release (0)

    // Scroll Down (-1 step: 0x01 -> 0x00)
    can_data[1] = 0x00;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);

    // Rollover Down (0x00 -> 0x0F is delta -1 with modulo 16)
    can_data[1] = 0x0F;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);

    // Rollover Up (0x0F -> 0x00 is delta +1 with modulo 16)
    can_data[1] = 0x00;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);
}

void test_peugeot_407_verification_vector_1_vol_up(void) {
    // Vector 1: cansend vcan0 0F6#0800000000000000 -> Expected: 2E 02 02 14 01 E6
    // Checksum = ~(0x02 + 0x02 + 0x14 + 0x01) = ~0x19 = 0xE6
    uint8_t out[16];
    size_t len = build_raise_stalk_key(PSA_STALK_KEY_VOL_UP, 1, out, sizeof(out));

    const uint8_t expected[] = { 0x2E, 0x02, 0x02, 0x14, 0x01, 0xE6 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
}

void test_peugeot_407_stalk_0x21f_buttons_and_rotary(void) {
    psa_stalk_state_t st;
    psa_stalk_init(&st);

    uint8_t data[3] = { 0x00, 0x00, 0x00 };

    // 1. Volume Up: 0x08
    data[0] = 0x08;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_UP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    data[0] = 0x00;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_UP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);

    // 2. Volume Down: 0x04
    data[0] = 0x04;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_DOWN, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    data[0] = 0x00;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_DOWN, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);

    // 3. Chorded MUTE: 0x0C (0x08 | 0x04)
    data[0] = 0x0C;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_MUTE, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    data[0] = 0x00;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_MUTE, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);

    // 4. Next Seek: 0x80
    data[0] = 0x80;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_NEXT, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    data[0] = 0x00;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_NEXT, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);

    // 5. Prev Seek: 0x40
    data[0] = 0x40;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_PREV, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    data[0] = 0x00;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_PREV, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);

    // 6. Source Toggle: 0x02
    data[0] = 0x02;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SRC, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    data[0] = 0x00;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SRC, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);

    // 7. Rotary Scroll Up (+1 step: counter 0 -> 1)
    data[1] = 0x01;
    s_stalk_cb_count = 0;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count); // pulse press (1) and release (0)
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SCROLL_UP, s_last_stalk_key_id);

    // 8. Rotary Scroll Down (-1 step: counter 1 -> 0)
    data[1] = 0x00;
    s_stalk_cb_count = 0;
    psa_decode_stalk_0x21f_ex(&st, data, sizeof(data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SCROLL_DOWN, s_last_stalk_key_id);
}

/* Helper: feed one 0x3E5 frame, return callback count */
static void feed_3e5(psa_console_state_t *st, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b5) {
    uint8_t data[6] = { b0, b1, b2, 0x00, 0x00, b5 };
    psa_decode_console_0x3e5_ex(st, data, sizeof(data), test_panel_key_callback);
}

static void check_3e5_key(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b5, uint8_t expected_key) {
    psa_console_state_t st;
    psa_console_init(&st);

    s_last_panel_key_id = PSA_PANEL_KEY_NONE;
    s_last_panel_key_state = 0;
    s_panel_cb_count = 0;
    feed_3e5(&st, b0, b1, b2, b5);
    TEST_ASSERT_EQUAL_HEX8(expected_key, s_last_panel_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_panel_key_state);
    TEST_ASSERT_EQUAL_INT(1, s_panel_cb_count);

    /* Release (idle frame) */
    feed_3e5(&st, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL_HEX8(expected_key, s_last_panel_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_panel_key_state);
    TEST_ASSERT_EQUAL_INT(2, s_panel_cb_count);
}

/* Proven frames from can_log_buttons.log (AUDIO, TRIP, CLIM, DARK) */
void test_peugeot_407_console_0x3e5_buttons(void) {
    check_3e5_key(0x00, 0x01, 0x00, 0x00, PSA_PANEL_KEY_AUDIO); /* 000100000000 */
    check_3e5_key(0x00, 0x40, 0x00, 0x00, PSA_PANEL_KEY_TRIP);  /* 004000000000 */
    check_3e5_key(0x01, 0x00, 0x00, 0x00, PSA_PANEL_KEY_CLIM);  /* 010000000000 */
    check_3e5_key(0x00, 0x00, 0x04, 0x00, PSA_PANEL_KEY_DARK);  /* 000004000000 */
    /* can_log_buttons1.log (MENU, OK, ESC) */
    check_3e5_key(0x40, 0x00, 0x00, 0x00, PSA_PANEL_KEY_MENU);  /* 400000000000 */
    check_3e5_key(0x00, 0x00, 0x40, 0x00, PSA_PANEL_KEY_OK);    /* 000040000000 */
    check_3e5_key(0x00, 0x00, 0x10, 0x00, PSA_PANEL_KEY_ESC);   /* 000010000000 */
    /* can_log_buttons2.log (UP) + simulator-doc layout for the other arrows */
    check_3e5_key(0x00, 0x00, 0x00, 0x40, PSA_PANEL_KEY_UP);    /* 000000000040 */
    check_3e5_key(0x00, 0x00, 0x00, 0x10, PSA_PANEL_KEY_DOWN);
    check_3e5_key(0x00, 0x00, 0x00, 0x04, PSA_PANEL_KEY_RIGHT);
    check_3e5_key(0x00, 0x00, 0x00, 0x01, PSA_PANEL_KEY_LEFT);
    /* TEL: B0[5:4] per simulator doc; Hiworld code 0x05 PHONE */
    check_3e5_key(0x10, 0x00, 0x00, 0x00, PSA_PANEL_KEY_TEL);
}

void test_peugeot_407_console_0x3e5_edge_cases(void) {
    psa_console_state_t st;
    psa_console_init(&st);

    /* Repeated identical press frames do not retrigger */
    s_panel_cb_count = 0;
    feed_3e5(&st, 0, 0, 0x04, 0);
    feed_3e5(&st, 0, 0, 0x04, 0);
    feed_3e5(&st, 0, 0, 0x04, 0);
    TEST_ASSERT_EQUAL_INT(1, s_panel_cb_count);
    feed_3e5(&st, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL_INT(2, s_panel_cb_count);

    /* Same key pressed twice in a row gives two full press/release cycles (DARK x2 in log) */
    s_panel_cb_count = 0;
    feed_3e5(&st, 0, 0, 0x04, 0);
    feed_3e5(&st, 0, 0, 0, 0);
    feed_3e5(&st, 0, 0, 0x04, 0);
    feed_3e5(&st, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL_INT(4, s_panel_cb_count);

    /* Idle frame with no key held produces nothing */
    s_panel_cb_count = 0;
    feed_3e5(&st, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL_INT(0, s_panel_cb_count);

    /* Short DLC is rejected (no out-of-bounds read) */
    const uint8_t short_frame[5] = { 0x40, 0, 0, 0, 0 };
    psa_decode_console_0x3e5_ex(&st, short_frame, sizeof(short_frame), test_panel_key_callback);
    TEST_ASSERT_EQUAL_INT(0, s_panel_cb_count);

    /* Pending MODE (B1[5:4]) is NOT decoded */
    feed_3e5(&st, 0x00, 0x10, 0, 0);
    TEST_ASSERT_EQUAL_INT(0, s_panel_cb_count);
}

void test_peugeot_407_stalk_tip_0x221_trip_button(void) {
    psa_stalk_state_t st;
    psa_stalk_init(&st);

    // 1. Press: CAN ID 0x221 Byte 0 Bit 3 = 0x08 (0xC8 nominal from vehicle dump)
    const uint8_t can_221_press[] = { 0xC8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    s_last_stalk_key_id = 0;
    s_last_stalk_key_state = 0;
    psa_decode_stalk_tip_0x221_ex(&st, can_221_press, sizeof(can_221_press), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_TRIP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(1, s_last_stalk_key_state);

    // 2. Release: CAN ID 0x221 Byte 0 Bit 3 cleared (0xC0 nominal from vehicle dump)
    const uint8_t can_221_release[] = { 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    psa_decode_stalk_tip_0x221_ex(&st, can_221_release, sizeof(can_221_release), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_TRIP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_UINT8(0, s_last_stalk_key_state);
}

/* --------------------------------------------------------------------------
 * 1.2 Dual-Zone Climate Control (HVAC) Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_verification_vector_2_climate(void) {
    // Vector 2: cansend vcan0 1D0#C8452B2C00004000 -> Expected: 2E 21 07 C8 45 2B 2C 00 00 40 33
    // Checksum = ~(0x21 + 0x07 + 0xC8 + 0x45 + 0x2B + 0x2C + 0x00 + 0x00 + 0x40) = ~0xCC = 0x33
    const uint8_t can_1d0[] = { 0xC8, 0x45, 0x2B, 0x2C, 0x00, 0x00, 0x40, 0x00 };
    hvac_state_t st;
    psa_decode_hvac_0x1d0(can_1d0, sizeof(can_1d0), &st);

    TEST_ASSERT_TRUE(st.power);
    TEST_ASSERT_TRUE(st.ac_compressor);
    TEST_ASSERT_TRUE(st.auto_mode);
    TEST_ASSERT_FALSE(st.dual_mode);
    TEST_ASSERT_EQUAL_UINT8(5, st.fan_speed);
    TEST_ASSERT_TRUE(st.driver_wind_face);
    TEST_ASSERT_TRUE(st.pass_wind_face);

    uint8_t out[16];
    size_t len = build_raise_hvac_packet(&st, out, sizeof(out));

    const uint8_t expected[] = { 0x2E, 0x21, 0x07, 0xC8, 0x45, 0x2B, 0x2C, 0x00, 0x00, 0x40, 0x33 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
}

void test_peugeot_407_hvac_defrost_and_recirc(void) {
    hvac_state_t st;
    memset(&st, 0, sizeof(st));
    st.power = true;
    st.recirculation = true;
    st.rear_defrost = true;
    st.front_max_defrost = true;
    st.ac_max = true;
    st.driver_wind_up = true;
    st.fan_speed = 8;
    st.driver_temp_raw = 0xFF; // HI
    st.pass_temp_raw = 0x00;   // LO

    uint8_t out[16];
    size_t len = build_raise_hvac_packet(&st, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
}

static bool rd_feed(uint32_t id, uint8_t dlc, const uint8_t *d) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    memcpy(f.data, d, dlc);
    can_router_process_can(&f);
    return can_router_get_state()->climate.rear_defrost;
}

void test_peugeot_407_rear_defrost_detection(void) {
    can_router_init();

    /* 0x1D0: Byte 4 Bit 6 */
    const uint8_t h_on[]  = { 0x00, 0x00, 0x00, 0x00, 0x40, 0x0A, 0x0A };
    const uint8_t h_off[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x0A };
    const uint8_t h_old[] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x0A, 0x0A };
    TEST_ASSERT_TRUE(rd_feed(0x1D0, 7, h_on));
    TEST_ASSERT_FALSE(rd_feed(0x1D0, 7, h_off));
    TEST_ASSERT_FALSE(rd_feed(0x1D0, 7, h_old)); /* bit 0 is no longer rear defrost */

    /* Short frames do not touch state */
    const uint8_t s1[] = { 0x00, 0x00, 0x00, 0x00 };
    TEST_ASSERT_TRUE(rd_feed(0x1D0, 7, h_on));
    TEST_ASSERT_TRUE(rd_feed(0x1D0, 4, s1));
}

/* --------------------------------------------------------------------------
 * 1.3 Ultrasonic Parking Sensors (Front & Rear AAS) Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_verification_vector_3_parking_radar(void) {
    // Vector 3: cansend vcan0 260#0303030000000000 -> Expected: 2E 32 07 00 03 03 03 FF FF FF C0
    // Checksum = ~(0x32 + 0x07 + 0x00 + 0x03 + 0x03 + 0x03 + 0xFF + 0xFF + 0xFF) = ~0x3F = 0xC0
    const uint8_t can_260[] = { 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rl = 0xFF, rc = 0xFF, rr = 0xFF;
    psa_decode_aas_rear_0x260(can_260, sizeof(can_260), &rl, &rc, &rr);


    uint8_t out[16];
    size_t len = build_raise_rear_radar(rl, rc, rr, 0xFF, 0xFF, 0xFF, out, sizeof(out));

    const uint8_t expected[] = { 0x2E, 0x32, 0x07, 0x00, 0x03, 0x03, 0x03, 0xFF, 0xFF, 0xFF, 0xC0 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
}

void test_peugeot_407_front_parking_radar(void) {
    const uint8_t can_270[] = { 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t fl = 0xFF, fc = 0xFF, fr = 0xFF;
    psa_decode_aas_front_0x270(can_270, sizeof(can_270), &fl, &fc, &fr);


    uint8_t out[16];
    size_t len = build_raise_front_radar(fl, fc, fr, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(8, len);
}

void test_peugeot_407_radar_hiworld_vector_1_obstacle_rear_center(void) {
    vehicle_radar_t radar;
    memset(&radar, 0, sizeof(radar));

    // Vector 1: Native CAN 0x0E1 Obstacle Rear Center
    // cansend vcan0 0E1#24403F04202600
    const uint8_t can_0e1[] = { 0x24, 0x40, 0x3F, 0x04, 0x20, 0x26, 0x00 };
    psa_radar_process_can_0x0e1(&radar, can_0e1, sizeof(can_0e1));

    TEST_ASSERT_TRUE(radar.rear_active);
    TEST_ASSERT_FALSE(radar.front_active);
    TEST_ASSERT_TRUE(radar.display_active);

    // Verify builder directly
    uint8_t out[20];
    size_t len = build_hiworld_radar(&radar, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(17, len);

    // Expected UART Output (Hiworld 0x41):
    // 5A A5 0C 41 00 01 01 01 00 01 01 01 01 00 3F 05 97
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x3F, 0x05, 0x97
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_radar_hiworld_vector_2_quiescent_all_clear(void) {
    vehicle_radar_t radar;
    memset(&radar, 0, sizeof(radar));

    // Vector 2: Quiescent / All Clear
    // cansend vcan0 0E1#24003FFCFCFC00
    const uint8_t can_0e1[] = { 0x24, 0x00, 0x3F, 0xFC, 0xFC, 0xFC, 0x00 };
    psa_radar_process_can_0x0e1(&radar, can_0e1, sizeof(can_0e1));

    TEST_ASSERT_FALSE(radar.rear_active);
    TEST_ASSERT_FALSE(radar.front_active);
    TEST_ASSERT_FALSE(radar.display_active);

    uint8_t out[20];
    size_t len = build_hiworld_radar(&radar, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(17, len);

    // Expected UART Output (Hiworld 0x41):
    // 5A A5 0C 41 FF FF FF FF FF FF FF FF 01 00 3F 05 89
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x3F, 0x05, 0x89
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_radar_aee2010_0x260_and_0x270(void) {
    vehicle_radar_t radar;
    memset(&radar, 0, sizeof(radar));

    // Test 0x260 rear radar: RL=0x01, RC=0x02, RR=0x03, byte 3: Active(0x80) | Fault(0x01)
    const uint8_t can_260[] = { 0x01, 0x02, 0x03, 0x81 };
    psa_radar_process_can_0x260(&radar, can_260, sizeof(can_260));

    TEST_ASSERT_TRUE(radar.rear_active);
    TEST_ASSERT_TRUE(radar.system_fault);

    // Test 0x270 front radar: FL=0x00, FC=0x01, FR=0x02
    const uint8_t can_270[] = { 0x00, 0x01, 0x02 };
    psa_radar_process_can_0x270(&radar, can_270, sizeof(can_270));

    TEST_ASSERT_EQUAL_UINT8(0x00, radar.front_left_outer);
    TEST_ASSERT_EQUAL_UINT8(0x01, radar.front_left_center);
    TEST_ASSERT_EQUAL_UINT8(0x02, radar.front_right_outer);
}

void test_peugeot_407_radar_zone_mapping_and_boundaries(void) {
    // 0 -> Zone 0 (closest / critical / touching)
    // 1..6 -> Zones 1..6
    // 7, 8 -> Inactive (0xFF)

    // Null safety & DLC boundaries
    vehicle_radar_t radar;
    memset(&radar, 0, sizeof(radar));

    psa_radar_process_can_0x0e1(NULL, (const uint8_t[]){ 0x24 }, 1);
    psa_radar_process_can_0x0e1(&radar, NULL, 7);
    psa_radar_process_can_0x0e1(&radar, (const uint8_t[]){ 0x24, 0x00 }, 2); // dlc < 6

    psa_radar_process_can_0x260(&radar, (const uint8_t[]){ 0x01 }, 1); // dlc < 3
    psa_radar_process_can_0x270(&radar, (const uint8_t[]){ 0x01 }, 1); // dlc < 3

    uint8_t out[20];
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_radar(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_radar(&radar, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_radar(&radar, out, 16)); // too small
}

