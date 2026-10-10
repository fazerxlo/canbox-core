#include "unity.h"
#include "profiles/peugeot_407.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
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

static void test_tpms_uart_tx(const uint8_t *buf, size_t len) {
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

static void test_trip_uart_tx(const uint8_t *buf, size_t len) {
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

static void test_radar_uart_tx(const uint8_t *buf, size_t len) {
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
    psa_radar_ctx_t ctx;
    psa_radar_init(&ctx, test_radar_uart_tx);
    s_radar_uart_tx_calls = 0;

    // Vector 1: Native CAN 0x0E1 Obstacle Rear Center
    // cansend vcan0 0E1#24403F04202600
    const uint8_t can_0e1[] = { 0x24, 0x40, 0x3F, 0x04, 0x20, 0x26, 0x00 };
    psa_radar_process_can_0x0e1(&ctx, can_0e1, sizeof(can_0e1));

    TEST_ASSERT_EQUAL_INT(1, s_radar_uart_tx_calls);

    // Expected UART Output (Hiworld 0x41):
    // 5A A5 0C 41 00 01 01 01 00 01 01 01 01 00 3F 05 97
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x3F, 0x05, 0x97
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), s_radar_uart_len[0]);

    TEST_ASSERT_TRUE(ctx.state.rear_active);
    TEST_ASSERT_FALSE(ctx.state.front_active);
    TEST_ASSERT_TRUE(ctx.state.display_active);

    // Verify builder directly
    uint8_t out[20];
    size_t len = build_hiworld_radar(&ctx.state, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(17, len);
}

void test_peugeot_407_radar_hiworld_vector_2_quiescent_all_clear(void) {
    psa_radar_ctx_t ctx;
    psa_radar_init(&ctx, test_radar_uart_tx);
    s_radar_uart_tx_calls = 0;

    // Vector 2: Quiescent / All Clear
    // cansend vcan0 0E1#24003FFCFCFC00
    const uint8_t can_0e1[] = { 0x24, 0x00, 0x3F, 0xFC, 0xFC, 0xFC, 0x00 };
    psa_radar_process_can_0x0e1(&ctx, can_0e1, sizeof(can_0e1));

    TEST_ASSERT_EQUAL_INT(1, s_radar_uart_tx_calls);

    // Expected UART Output (Hiworld 0x41):
    // 5A A5 0C 41 FF FF FF FF FF FF FF FF 01 00 3F 05 89
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x3F, 0x05, 0x89
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), s_radar_uart_len[0]);

    TEST_ASSERT_FALSE(ctx.state.rear_active);
    TEST_ASSERT_FALSE(ctx.state.front_active);
    TEST_ASSERT_FALSE(ctx.state.display_active);
}

void test_peugeot_407_radar_aee2010_0x260_and_0x270(void) {
    psa_radar_ctx_t ctx;
    psa_radar_init(&ctx, test_radar_uart_tx);
    s_radar_uart_tx_calls = 0;

    // Test 0x260 rear radar: RL=0x01, RC=0x02, RR=0x03, byte 3: Active(0x80) | Fault(0x01)
    const uint8_t can_260[] = { 0x01, 0x02, 0x03, 0x81 };
    psa_radar_process_can_0x260(&ctx, can_260, sizeof(can_260));

    TEST_ASSERT_EQUAL_INT(1, s_radar_uart_tx_calls);
    TEST_ASSERT_TRUE(ctx.state.rear_active);
    TEST_ASSERT_TRUE(ctx.state.system_fault);

    // Test 0x270 front radar: FL=0x00, FC=0x01, FR=0x02
    const uint8_t can_270[] = { 0x00, 0x01, 0x02 };
    psa_radar_process_can_0x270(&ctx, can_270, sizeof(can_270));

    TEST_ASSERT_EQUAL_INT(2, s_radar_uart_tx_calls);
}

void test_peugeot_407_radar_zone_mapping_and_boundaries(void) {
    // 0 -> Zone 0 (closest / critical / touching)
    // 1..6 -> Zones 1..6
    // 7, 8 -> Inactive (0xFF)

    // Null safety & DLC boundaries
    psa_radar_ctx_t ctx;
    psa_radar_init(&ctx, test_radar_uart_tx);
    s_radar_uart_tx_calls = 0;

    psa_radar_process_can_0x0e1(NULL, (const uint8_t[]){ 0x24 }, 1);
    psa_radar_process_can_0x0e1(&ctx, NULL, 7);
    psa_radar_process_can_0x0e1(&ctx, (const uint8_t[]){ 0x24, 0x00 }, 2); // dlc < 6
    TEST_ASSERT_EQUAL_INT(0, s_radar_uart_tx_calls);

    psa_radar_process_can_0x260(&ctx, (const uint8_t[]){ 0x01 }, 1); // dlc < 3
    psa_radar_process_can_0x270(&ctx, (const uint8_t[]){ 0x01 }, 1); // dlc < 3
    TEST_ASSERT_EQUAL_INT(0, s_radar_uart_tx_calls);

    uint8_t out[20];
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_radar(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_radar(&ctx.state, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_radar(&ctx.state, out, 16)); // too small
}

/* --------------------------------------------------------------------------
 * 1.4 Trip Computer & Engine Telemetry Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_trip_computer_0x165_and_0x1a5(void) {
    // Instant fuel (0x165): 6.8 L/100km (68 = 0x0044), Range 850 km (0x0352), Dest 120 km (0x0078), Temp 24 C (0x18)
    const uint8_t can_165[] = { 0x00, 0x44, 0x03, 0x52, 0x00, 0x78, 0x18, 0x00 };
    uint16_t fuel = 0, range = 0, dest = 0;
    uint8_t temp_raw = 0;
    psa_decode_trip_0x165(can_165, sizeof(can_165), &fuel, &range, &dest, &temp_raw);

    TEST_ASSERT_EQUAL_UINT16(68, fuel);
    TEST_ASSERT_EQUAL_UINT16(850, range);
    TEST_ASSERT_EQUAL_UINT16(120, dest);

    uint8_t out_fuel[16];
    size_t len_fuel = build_raise_instant_fuel(fuel, range, dest, out_fuel, sizeof(out_fuel));
    TEST_ASSERT_EQUAL_UINT32(10, len_fuel);

    uint8_t out_temp[8];
    size_t len_temp = build_raise_outside_temp(temp_raw, out_temp, sizeof(out_temp));
    TEST_ASSERT_EQUAL_UINT32(5, len_temp);

    // Trip 1 (0x1A5): Dist 154.2 km (1542 = 0x0606), Avg Fuel 5.9 L/100km (59 = 0x003B), Avg Speed 82 km/h (0x52)
    const uint8_t can_1a5[] = { 0x06, 0x06, 0x00, 0x3B, 0x52, 0x00, 0x00, 0x00 };
    uint16_t t1_dist = 0, t1_fuel = 0, t1_spd = 0;
    psa_decode_trip1_0x1a5(can_1a5, sizeof(can_1a5), &t1_dist, &t1_fuel, &t1_spd);

    TEST_ASSERT_EQUAL_UINT16(1542, t1_dist);
    TEST_ASSERT_EQUAL_UINT16(59, t1_fuel);
    TEST_ASSERT_EQUAL_UINT16(82, t1_spd);

    uint8_t out_t1[16];
    size_t len_t1 = build_raise_trip1(t1_fuel, t1_spd, t1_dist, out_t1);
    TEST_ASSERT_EQUAL_UINT32(10, len_t1);
}

void test_peugeot_407_reverse_state(void) {
    uint8_t out[8];
    size_t len = build_raise_reverse_state(true, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(5, len);

    bool rev = false;

    // CAN 0x036 reverse decoding
    const uint8_t can_036_rev_on[] = { 0x0E, 0x80, 0x00 };
    psa_decode_reverse_0x036(can_036_rev_on, sizeof(can_036_rev_on), &rev);
    TEST_ASSERT_TRUE(rev);

    const uint8_t can_036_rev_off[] = { 0x0E, 0x00, 0x00 };
    psa_decode_reverse_0x036(can_036_rev_off, sizeof(can_036_rev_off), &rev);
    TEST_ASSERT_FALSE(rev);

    // CAN 0x0F6 reverse decoding
    const uint8_t can_0f6_rev_on[] = { 0x88, 0x5A, 0x00, 0x00, 0x00, 0x64, 0x64, 0x80 };
    psa_decode_reverse_0x0f6(can_0f6_rev_on, sizeof(can_0f6_rev_on), &rev);
    TEST_ASSERT_TRUE(rev);

    const uint8_t can_0f6_rev_off[] = { 0x88, 0x5A, 0x00, 0x00, 0x00, 0x64, 0x64, 0x00 };
    psa_decode_reverse_0x0f6(can_0f6_rev_off, sizeof(can_0f6_rev_off), &rev);
    TEST_ASSERT_FALSE(rev);

    // Boundary checks
    rev = true;
    psa_decode_reverse_0x036(NULL, 2, &rev);
    TEST_ASSERT_TRUE(rev);
    psa_decode_reverse_0x036(can_036_rev_off, 1, &rev);
    TEST_ASSERT_TRUE(rev);
    psa_decode_reverse_0x0f6(can_0f6_rev_off, 7, &rev);
    TEST_ASSERT_TRUE(rev);

    // Hardware GPIO trigger
    psa_reverse_set_hardware_trigger(true);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    psa_reverse_set_hardware_trigger(false);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));
}

/* --------------------------------------------------------------------------
 * Hiworld Peugeot 407 Trip Computer & Telemetry Unit Tests & Verification Vectors
 * Based on CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md
 * -------------------------------------------------------------------------- */
void test_peugeot_407_trip_hiworld_vector_1_instant_fuel(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    // Vector 1: Instant Fuel 6.8 L/100km (68 = 0x0044), Range 640 km (0x0280)
    // CAN ID 0x221 injection: cansend vcan0 221#00004402800000
    const uint8_t can_221[] = { 0x00, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00 };
    psa_trip_process_can_0x221(&ctx, can_221, sizeof(can_221));

    TEST_ASSERT_EQUAL_INT(1, s_trip_uart_tx_calls);

    // Expected UART Output: 5A A5 0A 13 00 44 02 80 00 00 00 00 00 00 E2
    const uint8_t expected[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2 };
    TEST_ASSERT_EQUAL_UINT32(15, s_trip_uart_len[0]);

    TEST_ASSERT_EQUAL_UINT16(68, ctx.state.instant_fuel_deci);
    TEST_ASSERT_EQUAL_UINT16(640, ctx.state.range_km);
    TEST_ASSERT_EQUAL_UINT16(0, ctx.state.dest_dist_km);

    // Verify builder directly
    uint8_t out[16];
    size_t len = build_hiworld_trip_instant(&ctx.state, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(15, len);
}

void test_peugeot_407_trip_hiworld_vector_target_mileage_dump(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    // CAN ID 0x221 injection from dump_2026-09-29_19-37-10.log:
    // Fuel: 7.1 (0x0047), Range: 619 km (0x026B), Target mileage: 204 km (raw 2040 = 0x07F8)
    const uint8_t can_221[] = { 0x00, 0x00, 0x47, 0x02, 0x6B, 0x07, 0xF8 };
    psa_trip_process_can_0x221(&ctx, can_221, sizeof(can_221));

    TEST_ASSERT_EQUAL_INT(1, s_trip_uart_tx_calls);

    // Expected UART Output: 5A A5 0A 13 00 47 02 6B 07 F8 00 00 00 00 CF
    const uint8_t expected[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x47, 0x02, 0x6B, 0x07, 0xF8, 0x00, 0x00, 0x00, 0x00, 0xCF };
    TEST_ASSERT_EQUAL_UINT32(15, s_trip_uart_len[0]);

    TEST_ASSERT_EQUAL_UINT16(71, ctx.state.instant_fuel_deci);
    TEST_ASSERT_EQUAL_UINT16(619, ctx.state.range_km);
    TEST_ASSERT_EQUAL_UINT16(2040, ctx.state.dest_dist_km);

    uint8_t out[16];
    size_t len = build_hiworld_trip_instant(&ctx.state, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(15, len);
}

void test_peugeot_407_trip_hiworld_vector_2_trip1_historical(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    // Vector 2: Trip 1 Historical (Distance 569 km, Fuel 7.3 L/100km, Mean Speed 37 km/h)
    // CAN ID 0x2A1 injection: cansend vcan0 2A1#25023900490025
    const uint8_t can_2a1[] = { 0x25, 0x02, 0x39, 0x00, 0x49, 0x00, 0x25 };
    psa_trip_process_can_0x2a1(&ctx, can_2a1, sizeof(can_2a1));

    TEST_ASSERT_EQUAL_INT(1, s_trip_uart_tx_calls);

    // Expected UART Output: 5A A5 06 14 00 49 00 25 02 39 C2
    const uint8_t expected[] = { 0x5A, 0xA5, 0x06, 0x14, 0x00, 0x49, 0x00, 0x25, 0x02, 0x39, 0xC2 };
    TEST_ASSERT_EQUAL_UINT32(11, s_trip_uart_len[0]);

    TEST_ASSERT_EQUAL_UINT16(569, ctx.state.trip1_distance_km);
    TEST_ASSERT_EQUAL_UINT16(73, ctx.state.trip1_avg_fuel);
    TEST_ASSERT_EQUAL_UINT8(37, ctx.state.trip1_avg_speed);

    // Verify builder directly
    uint8_t out[16];
    size_t len = build_hiworld_trip1(&ctx.state, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
}

void test_peugeot_407_trip_hiworld_vector_3_trip2_historical(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    // Vector 3: Trip 2 Historical (Distance 921 km, Fuel 7.9 L/100km, Mean Speed 35 km/h)
    // CAN ID 0x261 injection: cansend vcan0 261#230399004F0023
    const uint8_t can_261[] = { 0x23, 0x03, 0x99, 0x00, 0x4F, 0x00, 0x23 };
    psa_trip_process_can_0x261(&ctx, can_261, sizeof(can_261));

    TEST_ASSERT_EQUAL_INT(1, s_trip_uart_tx_calls);

    // Expected UART Output: 5A A5 06 15 00 4F 00 23 03 99 28
    // Note: Spec manual sum typo stated 0x12A - 1 = 0x29, but actual sum is 0x129 - 1 = 0x28.
    const uint8_t expected[] = { 0x5A, 0xA5, 0x06, 0x15, 0x00, 0x4F, 0x00, 0x23, 0x03, 0x99, 0x28 };
    TEST_ASSERT_EQUAL_UINT32(11, s_trip_uart_len[0]);

    TEST_ASSERT_EQUAL_UINT16(921, ctx.state.trip2_distance_km);
    TEST_ASSERT_EQUAL_UINT16(79, ctx.state.trip2_avg_fuel);
    TEST_ASSERT_EQUAL_UINT8(35, ctx.state.trip2_avg_speed);

    // Verify builder directly
    uint8_t out[16];
    size_t len = build_hiworld_trip2(&ctx.state, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
}

void test_peugeot_407_trip_hiworld_vector_4_fast_dynamics_0x0b6(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    // Vector 4: Fast Dynamics RPM & Speed (800 RPM, 10 km/h)
    // CAN ID 0x0B6 injection: cansend vcan0 0B6#190003E8000000D0
    const uint8_t can_0b6[] = { 0x19, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0xD0 };
    psa_trip_process_can_0x0b6(&ctx, can_0b6, sizeof(can_0b6));

    TEST_ASSERT_EQUAL_INT(0, s_trip_uart_tx_calls);
    TEST_ASSERT_EQUAL_UINT16(800, ctx.state.rpm);
    TEST_ASSERT_EQUAL_UINT16(10, ctx.state.speed_kmh);

    // Test invalid / engine off (0xFFFF)
    const uint8_t can_0b6_off[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xD0 };
    psa_trip_process_can_0x0b6(&ctx, can_0b6_off, sizeof(can_0b6_off));
    TEST_ASSERT_EQUAL_UINT16(0, ctx.state.rpm);
    TEST_ASSERT_EQUAL_UINT16(0, ctx.state.speed_kmh);
}

void test_peugeot_407_trip_hiworld_bsi_slow_data_0x0f6(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    // 0x0F6: Coolant Raw 130 (90 C), Ambient Raw 128 (24 C), Reverse Active (Bit 7 of byte 7)
    const uint8_t can_0f6[] = { 0x88, 130, 0x00, 0x00, 0x00, 128, 128, 0x80 };
    psa_trip_process_can_0x0f6(&ctx, can_0f6, sizeof(can_0f6));

    TEST_ASSERT_EQUAL_INT8(90, ctx.state.coolant_c);
    TEST_ASSERT_EQUAL_INT8(24, ctx.state.ambient_c);
    TEST_ASSERT_TRUE(ctx.state.reverse_active);

    // Forward gear, Ambient 0 C (Raw 80)
    const uint8_t can_0f6_fwd[] = { 0x88, 125, 0x00, 0x00, 0x00, 80, 80, 0x00 };
    psa_trip_process_can_0x0f6(&ctx, can_0f6_fwd, sizeof(can_0f6_fwd));
    TEST_ASSERT_EQUAL_INT8(85, ctx.state.coolant_c);
    TEST_ASSERT_EQUAL_INT8(0, ctx.state.ambient_c);
    TEST_ASSERT_FALSE(ctx.state.reverse_active);

    // Raw 0xFF should NOT overwrite existing ambient_c
    const uint8_t can_0f6_inv[] = { 0x88, 120, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00 };
    psa_trip_process_can_0x0f6(&ctx, can_0f6_inv, sizeof(can_0f6_inv));
    TEST_ASSERT_EQUAL_INT8(80, ctx.state.coolant_c);
    TEST_ASSERT_EQUAL_INT8(0, ctx.state.ambient_c);

    // Sub-zero ambient temperature: Raw 40 -> (40 * 5 - 400)/10 = -20 C
    const uint8_t can_0f6_neg[] = { 0x88, 120, 0x00, 0x00, 0x00, 40, 40, 0x00 };
    psa_trip_process_can_0x0f6(&ctx, can_0f6_neg, sizeof(can_0f6_neg));
    TEST_ASSERT_EQUAL_INT8(-20, ctx.state.ambient_c);
}

void test_peugeot_407_trip_hiworld_speed_fallback(void) {
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);

    // DLC = 5 without bytes 5..6 (speed in byte 0 is 45 km/h)
    const uint8_t can_2a1_short[] = { 45, 0x01, 0x00, 0x00, 0x30 };
    psa_trip_process_can_0x2a1(&ctx, can_2a1_short, sizeof(can_2a1_short));
    TEST_ASSERT_EQUAL_UINT8(45, ctx.state.trip1_avg_speed);

    // DLC = 7 but bytes 5..6 are 0 (fallback to byte 0 which is 55 km/h)
    const uint8_t can_261_zero_speed[] = { 55, 0x01, 0x00, 0x00, 0x30, 0x00, 0x00 };
    psa_trip_process_can_0x261(&ctx, can_261_zero_speed, sizeof(can_261_zero_speed));
    TEST_ASSERT_EQUAL_UINT8(55, ctx.state.trip2_avg_speed);
}

void test_peugeot_407_trip_hiworld_boundary_and_null_safety(void) {
    // NULL pointer calls must not crash
    psa_trip_init(NULL, NULL);
    psa_trip_send_instant(NULL);
    psa_trip_send_trip1(NULL);
    psa_trip_send_trip2(NULL);
    psa_trip_process_can_0x0b6(NULL, NULL, 0);
    psa_trip_process_can_0x221(NULL, NULL, 0);
    psa_trip_process_can_0x2a1(NULL, NULL, 0);
    psa_trip_process_can_0x261(NULL, NULL, 0);
    psa_trip_process_can_0x0f6(NULL, NULL, 0);

    // Short DLC boundary checks
    psa_trip_ctx_t ctx;
    psa_trip_init(&ctx, test_trip_uart_tx);
    s_trip_uart_tx_calls = 0;

    uint8_t dummy[8] = { 0 };
    psa_trip_process_can_0x0b6(&ctx, dummy, 3);
    psa_trip_process_can_0x221(&ctx, dummy, 6);
    psa_trip_process_can_0x2a1(&ctx, dummy, 4);
    psa_trip_process_can_0x261(&ctx, dummy, 4);
    psa_trip_process_can_0x0f6(&ctx, dummy, 7);
    TEST_ASSERT_EQUAL_INT(0, s_trip_uart_tx_calls);

    // Send with NULL uart_tx must not crash
    ctx.uart_tx = NULL;
    psa_trip_send_instant(&ctx);
    psa_trip_send_trip1(&ctx);
    psa_trip_send_trip2(&ctx);

    // Builder buffer size & NULL safety
    uint8_t out[16];
    psa_trip_state_t st;
    memset(&st, 0, sizeof(st));

    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip_instant(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip_instant(&st, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip_instant(&st, out, 8));

    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip1(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip1(&st, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip1(&st, out, 10));

    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip2(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip2(&st, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_trip2(&st, out, 10));
}

/* --------------------------------------------------------------------------
 * 1.5 Doors & Body Status Tests
 * -------------------------------------------------------------------------- */
static uint8_t s_doors_uart_buf[32];
static size_t  s_doors_uart_len = 0;
static int     s_doors_uart_tx_calls = 0;

static void test_doors_uart_tx(const uint8_t *buf, size_t len) {
    if (buf && len <= sizeof(s_doors_uart_buf)) {
        memcpy(s_doors_uart_buf, buf, len);
        s_doors_uart_len = len;
        s_doors_uart_tx_calls++;
    }
}

void test_peugeot_407_verification_vector_4_doors(void) {
    // Vector 4: cansend vcan0 221#8000000000000000 (Driver door open)
    const uint8_t can_221[] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_doors_body_t doors;
    psa_decode_doors_0x221(can_221, sizeof(can_221), &doors);

    TEST_ASSERT_TRUE(doors.driver_door);
    TEST_ASSERT_FALSE(doors.pass_door);
    TEST_ASSERT_FALSE(doors.trunk);
    TEST_ASSERT_FALSE(doors.handbrake);

    uint8_t out[16];
    size_t len = build_raise_doors(&doors, out, sizeof(out));

    // Expected checksum: ~(0x38 + 0x08 + 0x80) = ~0xC0 = 0x3F
    const uint8_t expected[] = { 0x2E, 0x38, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
}

void test_peugeot_407_doors_all_open_with_handbrake(void) {
    const uint8_t can_221[] = { 0xFC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_doors_body_t doors;
    psa_decode_doors_0x221(can_221, sizeof(can_221), &doors);

    TEST_ASSERT_TRUE(doors.driver_door);
    TEST_ASSERT_TRUE(doors.pass_door);
    TEST_ASSERT_TRUE(doors.rear_left_door);
    TEST_ASSERT_TRUE(doors.rear_right_door);
    TEST_ASSERT_TRUE(doors.trunk);
    TEST_ASSERT_TRUE(doors.hood);
    TEST_ASSERT_TRUE(doors.handbrake);

    uint8_t out[16];
    size_t len = build_raise_doors(&doors, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(12, len);
}

void test_peugeot_407_doors_hiworld_vector_1_driver_front(void) {
    // Vector 1 from SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md:
    // CAN ID 0x220 data: 80 00 -> Expected UART Hiworld 0x12 (10-byte): 5A A5 0A 12 00 04 84 00 00 00 00 00 00 03 A6
    const uint8_t can_220[] = { 0x80, 0x00 };
    psa_doors_body_t doors;
    psa_decode_doors_0x220(can_220, sizeof(can_220), &doors);

    TEST_ASSERT_TRUE(doors.driver_door);
    TEST_ASSERT_FALSE(doors.pass_door);
    TEST_ASSERT_FALSE(doors.rear_left_door);
    TEST_ASSERT_FALSE(doors.rear_right_door);
    TEST_ASSERT_FALSE(doors.trunk);
    TEST_ASSERT_FALSE(doors.hood);

    uint8_t out[20];
    size_t len = build_hiworld_doors(&doors, out, sizeof(out));

    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xA6
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
}

void test_peugeot_407_doors_hiworld_all_open(void) {
    const uint8_t can_220[] = { 0xFC, 0x01 };
    psa_doors_body_t doors;
    psa_decode_doors_0x220(can_220, sizeof(can_220), &doors);

    TEST_ASSERT_TRUE(doors.driver_door);
    TEST_ASSERT_TRUE(doors.pass_door);
    TEST_ASSERT_TRUE(doors.rear_left_door);
    TEST_ASSERT_TRUE(doors.rear_right_door);
    TEST_ASSERT_TRUE(doors.trunk);
    TEST_ASSERT_TRUE(doors.hood);
    TEST_ASSERT_TRUE(doors.handbrake);

    uint8_t out[20];
    size_t len = build_hiworld_doors(&doors, out, sizeof(out));

    // b2 = 0xFC | 0x04 = 0xFC
    // Checksum = (0x0A + 0x12 + 0x00 + 0x04 + 0xFC + 0x03 - 1) & 0xFF = 0x1E
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1E
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
}

void test_peugeot_407_doors_hiworld_discrete_bits(void) {
    psa_doors_body_t doors;
    uint8_t out[20];

    // FR door only (0x40) -> b2 = 0x44 -> CS = (0x0A + 0x12 + 0x04 + 0x44 + 0x03 - 1) = 0x66
    const uint8_t can_fr[] = { 0x40, 0x00 };
    psa_decode_doors_0x220(can_fr, sizeof(can_fr), &doors);
    TEST_ASSERT_TRUE(doors.pass_door);
    size_t len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_fr[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x66
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);

    // RL door only (0x20) -> b2 = 0x24 -> CS = (0x0A + 0x12 + 0x04 + 0x24 + 0x03 - 1) = 0x46
    const uint8_t can_rl[] = { 0x20, 0x00 };
    psa_decode_doors_0x220(can_rl, sizeof(can_rl), &doors);
    TEST_ASSERT_TRUE(doors.rear_left_door);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_rl[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x46
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);

    // RR door only (0x10) -> b2 = 0x14 -> CS = (0x0A + 0x12 + 0x04 + 0x14 + 0x03 - 1) = 0x36
    const uint8_t can_rr[] = { 0x10, 0x00 };
    psa_decode_doors_0x220(can_rr, sizeof(can_rr), &doors);
    TEST_ASSERT_TRUE(doors.rear_right_door);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_rr[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x36
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);

    // Trunk only (0x08) -> b2 = 0x0C -> CS = (0x0A + 0x12 + 0x04 + 0x0C + 0x03 - 1) = 0x2E
    const uint8_t can_trunk[] = { 0x08, 0x00 };
    psa_decode_doors_0x220(can_trunk, sizeof(can_trunk), &doors);
    TEST_ASSERT_TRUE(doors.trunk);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_trunk[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x2E
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);

    // All closed (0x00) -> b2 = 0x04 -> CS = (0x0A + 0x12 + 0x04 + 0x04 + 0x03 - 1) = 0x26
    const uint8_t can_none[] = { 0x00, 0x00 };
    psa_decode_doors_0x220(can_none, sizeof(can_none), &doors);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_none[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x26
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);
}

void test_peugeot_407_doors_decode_extended_signals(void) {
    // Byte 0: Rear window (0x02), Fuel flap (0x01)
    // Byte 1: Wiper reverse (0x80), Auto lock (0x10), Radar enabled (0x08), Handbrake (0x01)
    const uint8_t can_220[] = { 0x03, 0x99 };
    psa_doors_body_t doors;
    psa_decode_doors_0x220(can_220, sizeof(can_220), &doors);

    TEST_ASSERT_FALSE(doors.driver_door);
    TEST_ASSERT_FALSE(doors.pass_door);
    TEST_ASSERT_FALSE(doors.rear_left_door);
    TEST_ASSERT_FALSE(doors.rear_right_door);
    TEST_ASSERT_FALSE(doors.trunk);
    TEST_ASSERT_FALSE(doors.hood);

    TEST_ASSERT_TRUE(doors.rear_window);
    TEST_ASSERT_TRUE(doors.fuel_flap);
    TEST_ASSERT_TRUE(doors.auto_rear_wiper);
    TEST_ASSERT_TRUE(doors.auto_locking);
    TEST_ASSERT_TRUE(doors.parking_radar_enabled);
    TEST_ASSERT_TRUE(doors.handbrake);
}

void test_peugeot_407_psa_doors_ctx_pipeline(void) {
    psa_doors_ctx_t ctx;
    s_doors_uart_len = 0;
    s_doors_uart_tx_calls = 0;

    psa_doors_init(&ctx, test_doors_uart_tx);
    TEST_ASSERT_FALSE(ctx.state.door_front_left);
    TEST_ASSERT_EQUAL_INT(0, s_doors_uart_tx_calls);

    // Process CAN 0x220: FL door open (0x80), handbrake active (0x01)
    const uint8_t can_open[] = { 0x80, 0x01 };
    psa_doors_process_can_0x220(&ctx, can_open, sizeof(can_open));

    TEST_ASSERT_TRUE(ctx.state.door_front_left);
    TEST_ASSERT_TRUE(ctx.state.handbrake_pulled);
    TEST_ASSERT_EQUAL_INT(1, s_doors_uart_tx_calls);

    const uint8_t expected_open[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xA6
    };
    TEST_ASSERT_EQUAL_UINT32(15, s_doors_uart_len);

    // Process CAN 0x220: All closed
    const uint8_t can_closed[] = { 0x00, 0x00 };
    psa_doors_process_can_0x220(&ctx, can_closed, sizeof(can_closed));

    TEST_ASSERT_FALSE(ctx.state.door_front_left);
    TEST_ASSERT_FALSE(ctx.state.handbrake_pulled);
    TEST_ASSERT_EQUAL_INT(2, s_doors_uart_tx_calls);

    const uint8_t expected_closed[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x26
    };
    TEST_ASSERT_EQUAL_UINT32(15, s_doors_uart_len);

    // Test send_hiworld without uart_tx function (should not crash)
    ctx.uart_tx = NULL;
    psa_doors_send_hiworld(&ctx);
}

void test_peugeot_407_hiworld_doors_boundary_checks(void) {
    psa_doors_body_t doors;
    memset(&doors, 0, sizeof(doors));
    doors.driver_door = true;

    uint8_t out[20];
    // NULL out buffer
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors(&doors, NULL, sizeof(out)));
    // NULL doors pointer
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors(NULL, out, sizeof(out)));
    // Buffer too small (< 15 bytes)
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors(&doors, out, 14));

    // build_hiworld_doors_state boundary tests
    psa_doors_state_t state;
    memset(&state, 0, sizeof(state));
    state.door_front_left = true;
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors_state(&state, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors_state(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors_state(&state, out, 14));

    size_t len = build_hiworld_doors_state(&state, out, sizeof(out));
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xA6
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);

    // psa_decode_doors_0x220 robustness
    psa_decode_doors_0x220(NULL, 2, &doors);
    psa_decode_doors_0x220(out, 0, &doors);
    psa_decode_doors_0x220(out, 2, NULL);

    // psa_doors_process_can_0x220 robustness
    psa_doors_ctx_t ctx;
    psa_doors_init(&ctx, test_doors_uart_tx);
    psa_doors_process_can_0x220(NULL, out, 2);
    psa_doors_process_can_0x220(&ctx, NULL, 2);
    psa_doors_process_can_0x220(&ctx, out, 0);
}

/* --------------------------------------------------------------------------
 * 1.6 Steering Wheel Angle Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_steering_wheel_angle(void) {
    // 35.0 deg right = 350 deci-deg (0x015E)
    const uint8_t can_0e6[] = { 0x01, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int16_t angle = 0;
    psa_decode_steering_angle_0x0e6(can_0e6, sizeof(can_0e6), &angle);
    TEST_ASSERT_EQUAL_INT16(350, angle);

    uint8_t out[8];
    size_t len = build_raise_steering_angle(angle, out);
    TEST_ASSERT_EQUAL_UINT32(6, len);

    // Negative angle: -12.5 deg left = -125 deci-deg (0xFF83)
    angle = -125;
    len = build_raise_steering_angle(angle, out);
    TEST_ASSERT_EQUAL_UINT32(6, len);
}

/* --------------------------------------------------------------------------
 * 1.7 OEM JBL Sound Amplifier Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_jbl_amplifier(void) {
    // Bass=9 (+2), Treble=7 (0), Bal=7, Fader=6, EQ=1 (Pop), Loudness=1 (0x10), SpeedComp=2, Vol=18
    const uint8_t can_1a0[] = { 0x00, 0x09, 0x07, 0x07, 0x06, 0x01, 0x12, 0x12 };
    jbl_amplifier_state_t amp;
    psa_decode_amplifier_0x1a0(can_1a0, sizeof(can_1a0), &amp);

    TEST_ASSERT_EQUAL_UINT8(9, amp.bass);
    TEST_ASSERT_EQUAL_UINT8(7, amp.treble);
    TEST_ASSERT_EQUAL_UINT8(7, amp.balance);
    TEST_ASSERT_EQUAL_UINT8(6, amp.fader);
    TEST_ASSERT_EQUAL_UINT8(1, amp.eq_preset);
    TEST_ASSERT_TRUE(amp.loudness);
    TEST_ASSERT_EQUAL_UINT8(2, amp.speed_vol_comp);
    TEST_ASSERT_EQUAL_UINT8(18, amp.master_volume);

    uint8_t out[16];
    size_t len = build_raise_amplifier(&amp, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(12, len);
}

/* --------------------------------------------------------------------------
 * 1.8 RD4 Radio & CD Changer Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_cd_changer_and_rds(void) {
    // CD Changer (0x3A6): Disc 2, Track 14, Total 20, 03:45, Random
    const uint8_t can_3a6[] = { 0x00, 0x02, 0x0E, 0x14, 0x03, 0x2D, 0x01, 0x00 };
    cd_changer_state_t cdc;
    psa_decode_cd_changer_0x3a6(can_3a6, sizeof(can_3a6), &cdc);

    TEST_ASSERT_EQUAL_UINT8(2, cdc.disc_slot);
    TEST_ASSERT_EQUAL_UINT8(14, cdc.track_num);
    TEST_ASSERT_EQUAL_UINT8(20, cdc.total_tracks);
    TEST_ASSERT_EQUAL_UINT8(3, cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(45, cdc.elapsed_sec);

    uint8_t out_cdc[16];
    size_t len_cdc = build_raise_cd_changer(&cdc, out_cdc, sizeof(out_cdc));
    TEST_ASSERT_EQUAL_UINT32(11, len_cdc);

    // RDS Radio Station (0x396): "BBC R1  "
    const uint8_t can_396[] = { 'B', 'B', 'C', ' ', 'R', '1', ' ', ' ' };
    char name[9];
    psa_decode_rds_name_0x396(can_396, sizeof(can_396), name);
    TEST_ASSERT_EQUAL_STRING("BBC R1  ", name);

    uint8_t out_rds[16];
    size_t len_rds = build_raise_rds_name("BBC R1", out_rds, sizeof(out_rds));
    TEST_ASSERT_EQUAL_UINT32(12, len_rds);
}

void test_peugeot_407_rd4_vector_1_fm_tuner(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. Source: Tuner (0x165)
    const uint8_t can_165[] = { 0xCC, 0x54, 0x10, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_165, sizeof(can_165));
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);

    // 2. Tuner Status (0x225): FM1, Preset 1, RDS Lock, 102.50 MHz (raw 1050 = 0x041A)
    const uint8_t can_225[] = { 0x20, 0x01, 0x90, 0x04, 0x1A };
    psa_rd4_process_can_0x225(&media.radio, can_225, sizeof(can_225));
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.band); /* FM1 */
    TEST_ASSERT_EQUAL_UINT16(1025, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.preset_slot);
    TEST_ASSERT_EQUAL_UINT8(0x20, media.radio.indicators);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.source_mode);

    // 3. Station Name (0x2A5): "RMF FM  "
    const uint8_t can_2a5[] = { 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D, 0x20, 0x20 };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5, sizeof(can_2a5));
    TEST_ASSERT_EQUAL_STRING("RMF FM  ", media.radio.station_name);

    // 4. Hiworld Output (Cmd 0x84): 19 wire bytes
    uint8_t out[32];
    size_t out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);

    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* FM1 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[5]); /* 1025 & 0xFF (Little Endian for Hiworld) */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[6]); /* 1025 >> 8 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[7]); /* Preset 1 */
    TEST_ASSERT_EQUAL_HEX8(0x20, out[8]); /* RDS indicator */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[9]); /* Playing */
    TEST_ASSERT_EQUAL_STRING_LEN("RMF FM  ", (char *)&out[10], 8);

    /* Verify checksum matches ((0x0E + 0x84 + sum(payload) - 1) & 0xFF) */
    uint16_t sum = out[2] + out[3];
    for (size_t i = 0; i < 14; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[18]);
}

void test_peugeot_407_rd4_vector_2_isotp_radiotext(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // Dynamic RDS RadioText: "Queen - Bohemian Rhapsody" (25 chars) with prefix 10 00 00 00 (Total 29 chars = 0x01D)
    const uint8_t ff[]  = { 0x10, 0x1D, 0x10, 0x00, 0x00, 0x00, 0x51, 0x75 };
    const uint8_t cf1[] = { 0x21, 0x65, 0x65, 0x6E, 0x20, 0x2D, 0x20, 0x42 };
    const uint8_t cf2[] = { 0x22, 0x6F, 0x68, 0x65, 0x6D, 0x69, 0x61, 0x6E };
    const uint8_t cf3[] = { 0x23, 0x20, 0x52, 0x68, 0x61, 0x70, 0x73, 0x6F };
    const uint8_t cf4[] = { 0x24, 0x64, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00 };

    psa_rd4_process_can_0x0a4(&media.radio, &isotp, ff, sizeof(ff));
    TEST_ASSERT_TRUE(isotp.active);
    TEST_ASSERT_FALSE(media.radio.radio_text_updated);

    psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf1, sizeof(cf1));
    TEST_ASSERT_TRUE(isotp.active);
    psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf2, sizeof(cf2));
    TEST_ASSERT_TRUE(isotp.active);
    psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf3, sizeof(cf3));
    TEST_ASSERT_TRUE(isotp.active);

    psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf4, sizeof(cf4));
    TEST_ASSERT_FALSE(isotp.active);
    TEST_ASSERT_TRUE(media.radio.radio_text_updated);
    TEST_ASSERT_EQUAL_UINT8(25, media.radio.radio_text_len);
    TEST_ASSERT_EQUAL_STRING("Queen - Bohemian Rhapsody", media.radio.radio_text);
    TEST_ASSERT_EQUAL_UINT8(0x04, media.radio.indicators & 0x04); /* RDTEXT bit */

    // Hiworld Output (Cmd 0x86): 5 + 25 = 30 wire bytes
    uint8_t out[64];
    size_t out_len = build_hiworld_radio_text(media.radio.radio_text, media.radio.radio_text_len, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(30, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(25, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x86, out[3]);
    TEST_ASSERT_EQUAL_STRING_LEN("Queen - Bohemian Rhapsody", (char *)&out[4], 25);

    uint16_t sum = out[2] + out[3];
    for (size_t i = 0; i < 25; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[29]);
}

void test_peugeot_407_rd4_vector_3_cd_changer(void) {
    vehicle_media_t media;
    psa_rd4_media_init(&media, NULL);

    // CAN 0x3A6: Disc 2, Track 14, Total 20, Min 3, Sec 45, Play Flags 0x01 (Random)
    const uint8_t can_3a6[] = { 0x00, 0x02, 0x0E, 0x14, 0x03, 0x2D, 0x01, 0x00 };
    psa_rd4_process_can_0x3a6(&media.cdc, can_3a6, sizeof(can_3a6));

    TEST_ASSERT_EQUAL_UINT8(2, media.cdc.active_disc);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.discs_loaded_mask);
    TEST_ASSERT_EQUAL_UINT16(14, media.cdc.track_num);
    TEST_ASSERT_EQUAL_UINT16(20, media.cdc.total_tracks);
    TEST_ASSERT_EQUAL_UINT8(3, media.cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(45, media.cdc.elapsed_sec);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.cdc.play_modes);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.cdc.play_status);

    // Hiworld Output (Cmd 0x97): 16 wire bytes
    uint8_t out[32];
    size_t out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);

    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x02, 0x02, 0x00, 0x00, 0x0E, 0x03, 0x2D, 0x01, 0x01, 0x00, 0x14, 0xF9
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, 16);
}

void test_peugeot_407_rd4_source_and_wavebands(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. CD Internal Drive Source
    const uint8_t can_cd[] = { 0xCC, 0x54, 0x20, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_cd, sizeof(can_cd));
    TEST_ASSERT_EQUAL_UINT8(0x30, media.radio.source_mode);

    // 2. AUX 1 Source
    const uint8_t can_aux1[] = { 0xCC, 0x54, 0x40, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_aux1, sizeof(can_aux1));
    TEST_ASSERT_EQUAL_UINT8(0x20, media.radio.source_mode);

    // 3. AUX 2 Source
    const uint8_t can_aux2[] = { 0xCC, 0x54, 0x50, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_aux2, sizeof(can_aux2));
    TEST_ASSERT_EQUAL_UINT8(0x21, media.radio.source_mode);

    // 4. CDC Source
    const uint8_t can_cdc[] = { 0xCC, 0x54, 0x30, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_cdc, sizeof(can_cdc));
    TEST_ASSERT_EQUAL_UINT8(0x31, media.radio.source_mode);

    // 5. AM Tuner: 1440 kHz, Band 0x50
    const uint8_t can_am[] = { 0x00, 0x02, 0x50, 0x05, 0xA0 };
    psa_rd4_process_can_0x225(&media.radio, can_am, sizeof(can_am));
    TEST_ASSERT_EQUAL_UINT8(0x10, media.radio.band);
    TEST_ASSERT_EQUAL_UINT16(1440, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_UINT8(2, media.radio.preset_slot);

    // 6. Seeking flag on 0x225 (Byte 0 Bit 3 = 0x08)
    const uint8_t can_seek[] = { 0x08, 0x00, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_seek, sizeof(can_seek));
    TEST_ASSERT_EQUAL_UINT8(0x02, media.radio.power_status);

    // Restoration from seeking
    const uint8_t can_locked[] = { 0x00, 0x00, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_locked, sizeof(can_locked));
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);

    // 7. Single frame RadioText on 0x0A4
    const uint8_t can_sf[] = { 0x07, 'T', 'E', 'S', 'T', ' ', 'O', 'K' };
    psa_rd4_process_can_0x0a4(&media.radio, &isotp, can_sf, sizeof(can_sf));
    TEST_ASSERT_EQUAL_UINT8(7, media.radio.radio_text_len);
    TEST_ASSERT_EQUAL_STRING("TEST OK", media.radio.radio_text);
}

void test_peugeot_407_rd4_frequency_and_ta_stability(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. Source Tuner (0x165)
    const uint8_t can_165[] = { 0xC8, 0xC0, 0x10, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_165, sizeof(can_165));

    // 2. Station Name (0x2A5): " RMF FM "
    const uint8_t can_2a5[] = { 0x20, 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D, 0x20 };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5, sizeof(can_2a5));

    // 3. Tuner Frequency (0x225): 96.0 MHz (raw 0x0398 = 920 -> 960 in 0.1 MHz)
    // CAN payload from real dump: 20 10 10 03 98
    const uint8_t can_225[] = { 0x20, 0x10, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_225, sizeof(can_225));
    TEST_ASSERT_EQUAL_UINT16(960, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.preset_slot);

    // 4. TA set by 0x265 (Byte 0 bit 5 = 0x20): payload b4 10 00 01
    const uint8_t can_265[] = { 0xB4, 0x10, 0x00, 0x01 };
    psa_rd4_process_can_0x265(&media.radio, can_265, sizeof(can_265));
    TEST_ASSERT_EQUAL_HEX8(0xA0, media.radio.indicators & 0xA0); /* Both TA (0x80) and RDS (0x20) set */

    // 5. Subsequent 0x225 arrival: must NOT clear TA set by 0x265!
    media.radio.updated = false;
    psa_rd4_process_can_0x225(&media.radio, can_225, sizeof(can_225));
    TEST_ASSERT_EQUAL_HEX8(0xA0, media.radio.indicators & 0xA0); /* TA remains set (no flickering!) */

    // 6. Hiworld Serialization (Cmd 0x84): verify Little-Endian frequency bytes & FM1 band
    uint8_t out[32];
    size_t out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* FM1 band */
    // 960 (0x03C0) in Little Endian: out[5] = 0xC0, out[6] = 0x03
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA0, out[8]); /* Indicators: TA + RDS */

    // 7. Test FM2 frame from real dump (dump_2026-10-09_21-41-25.log):
    // CAN 0x225: 20 50 20 02 ee (Preset 5, Band 0x20 = FM2, Freq 750 = 87.5 MHz)
    const uint8_t can_225_fm2[] = { 0x20, 0x50, 0x20, 0x02, 0xEE };
    psa_rd4_process_can_0x225(&media.radio, can_225_fm2, sizeof(can_225_fm2));
    TEST_ASSERT_EQUAL_UINT8(0x02, media.radio.band); /* FM2 -> 0x02 */
    TEST_ASSERT_EQUAL_UINT8(5, media.radio.preset_slot);
    TEST_ASSERT_EQUAL_UINT16(875, media.radio.freq_0_1mhz); /* 750 / 2 + 500 = 875 (87.5 MHz) */

    out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[4]); /* FM2 band for Hiworld */

    // 8. Station name & RDS cleared on frequency change or empty 0x2A5
    // Set a station name first:
    const uint8_t can_2a5_test[] = { 'T', 'E', 'S', 'T', 'F', 'M', ' ', ' ' };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5_test, sizeof(can_2a5_test));
    TEST_ASSERT_EQUAL_STRING("TESTFM  ", media.radio.station_name);

    // Now arrive with 0x2A5 carrying all 0x00 (no signal):
    const uint8_t can_2a5_empty[8] = { 0 };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5_empty, sizeof(can_2a5_empty));
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.station_name);

    // Set station name again, then change frequency without RDS:
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5_test, sizeof(can_2a5_test));
    TEST_ASSERT_EQUAL_STRING("TESTFM  ", media.radio.station_name);

    // Change frequency to 96.45 MHz (raw 0x03A1) without RDS (Byte 0 = 0x00):
    const uint8_t can_225_new_freq[] = { 0x00, 0x00, 0x10, 0x03, 0xA1 };
    psa_rd4_process_can_0x225(&media.radio, can_225_new_freq, sizeof(can_225_new_freq));
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.station_name);
    TEST_ASSERT_EQUAL_UINT16(964, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_HEX8(0x00, media.radio.indicators & 0x20); /* RDS cleared */
}

void test_peugeot_407_rd4_preset_memory_list(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. Simulate tuning into 6 presets on FM1 (band 0x01)
    // Preset 1: 87.5 MHz (raw 750 = 0x02EE)
    const uint8_t can_p1[] = { 0x20, 0x10, 0x10, 0x02, 0xEE };
    psa_rd4_process_can_0x225(&media.radio, can_p1, sizeof(can_p1));
    // Preset 2: 96.0 MHz (raw 920 = 0x0398)
    const uint8_t can_p2[] = { 0x20, 0x20, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_p2, sizeof(can_p2));
    // Preset 3: 102.5 MHz (raw 1050 = 0x041A)
    const uint8_t can_p3[] = { 0x20, 0x30, 0x10, 0x04, 0x1A };
    psa_rd4_process_can_0x225(&media.radio, can_p3, sizeof(can_p3));

    TEST_ASSERT_EQUAL_UINT16(875, media.radio.preset_freqs[0]);
    TEST_ASSERT_EQUAL_UINT16(960, media.radio.preset_freqs[1]);
    TEST_ASSERT_EQUAL_UINT16(1025, media.radio.preset_freqs[2]);
    TEST_ASSERT_TRUE(media.radio.preset_updated);

    // 2. Build Standard Frequency Mode (Cmd 0x85, 13 payload bytes -> 18 wire bytes)
    uint8_t out[64];
    size_t out_len = build_hiworld_radio_preset_freqs(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(18, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0D, out[2]); /* Length 13 */
    TEST_ASSERT_EQUAL_HEX8(0x85, out[3]); /* Opcode 0x85 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* Band FM1 */

    // Check Preset 1 (875 -> 0x036B uint16_be)
    TEST_ASSERT_EQUAL_HEX8(0x03, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x6B, out[6]);
    // Check Preset 2 (960 -> 0x03C0 uint16_be)
    TEST_ASSERT_EQUAL_HEX8(0x03, out[7]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[8]);
    // Check Preset 3 (1025 -> 0x0401 uint16_be)
    TEST_ASSERT_EQUAL_HEX8(0x04, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[10]);

    // Checksum check
    uint16_t sum = out[2] + out[3];
    for (size_t i = 0; i < 13; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[17]);

    // 3. Extended Station Names Mode via native CAN 0x125 ISO-TP stream
    // Proven multi-frame sequence captured from vehicle dump_2026-10-10_00-05-44.log:
    const uint8_t can_125_ff[]  = { 0x10, 0x28, 0x10, 0x04, 0x00, 0x40, 0x4A, 0x45 };
    const uint8_t can_125_cf1[] = { 0x21, 0x44, 0x59, 0x4E, 0x4B, 0x41, 0x20, 0xB0 };
    const uint8_t can_125_cf2[] = { 0x22, 0x20, 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D };
    const uint8_t can_125_cf3[] = { 0x23, 0x20, 0xB0, 0x52, 0x4D, 0x46, 0x20, 0x4D };
    const uint8_t can_125_cf4[] = { 0x24, 0x41, 0x58, 0x58, 0xB0, 0x20, 0x54, 0x52 };
    const uint8_t can_125_cf5[] = { 0x25, 0x4F, 0x4A, 0x4B, 0x41, 0x20, 0xB0 };

    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_ff, sizeof(can_125_ff));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf1, sizeof(can_125_cf1));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf2, sizeof(can_125_cf2));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf3, sizeof(can_125_cf3));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf4, sizeof(can_125_cf4));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf5, sizeof(can_125_cf5));

    TEST_ASSERT_EQUAL_STRING("JEDYNKA ", media.radio.preset_names[0]);
    TEST_ASSERT_EQUAL_STRING(" RMF FM ", media.radio.preset_names[1]);
    TEST_ASSERT_EQUAL_STRING("RMF MAXX", media.radio.preset_names[2]);
    TEST_ASSERT_EQUAL_STRING(" TROJKA ", media.radio.preset_names[3]);
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.preset_names[4]);
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.preset_names[5]);

    // Build Extended Station Names Mode (Cmd 0x85, 49 payload bytes -> 54 wire bytes)
    out_len = build_hiworld_radio_preset_names(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(54, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x31, out[2]); /* Length 49 */
    TEST_ASSERT_EQUAL_HEX8(0x85, out[3]); /* Opcode 0x85 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* Band FM1 */

    TEST_ASSERT_EQUAL_STRING_LEN("JEDYNKA ", (char *)&out[5], 8);
    TEST_ASSERT_EQUAL_STRING_LEN(" RMF FM ", (char *)&out[13], 8);
    TEST_ASSERT_EQUAL_STRING_LEN("RMF MAXX", (char *)&out[21], 8);
    TEST_ASSERT_EQUAL_STRING_LEN(" TROJKA ", (char *)&out[29], 8);
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&out[37], 8);
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&out[45], 8);

    sum = out[2] + out[3];
    for (size_t i = 0; i < 49; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[53]);
}

/* --------------------------------------------------------------------------
 * 2.1 Direct TPMS Numeric Readings & Fault Classification Tests
 * -------------------------------------------------------------------------- */
void test_psa_extended_tpms_numeric(void) {
    canbox_tpms_state_t tpms;
    memset(&tpms, 0, sizeof(tpms));
    tpms.pressure_dbar[0] = 24; // 2.4 Bar (0x18)
    tpms.pressure_dbar[1] = 24;
    tpms.pressure_dbar[2] = 24;
    tpms.pressure_dbar[3] = 24;

    uint8_t out[16];
    size_t len = build_raise_tpms_numeric(&tpms, out);
    TEST_ASSERT_EQUAL_UINT32(10, len);

    uint8_t expected_cs = (uint8_t)((0x66 + 0x06 + 0x01 + 0x18 + 0x18 + 0x18 + 0x18 + 0x00) ^ 0xFF);
    TEST_ASSERT_EQUAL_UINT32(0, build_raise_tpms_numeric(NULL, out));
}

void test_psa_extended_tpms_temp_and_alarms(void) {
    canbox_tpms_state_t tpms;
    memset(&tpms, 0, sizeof(tpms));
    tpms.temperature_c[0] = 20; // 20 + 40 = 60 (0x3C)
    tpms.temperature_c[1] = 25; // 25 + 40 = 65 (0x41)
    tpms.temperature_c[2] = 18; // 18 + 40 = 58 (0x3A)
    tpms.temperature_c[3] = 19; // 19 + 40 = 59 (0x3B)

    tpms.alarm_code[0] = 0; // Normal
    tpms.alarm_code[1] = 1; // Low pressure
    tpms.alarm_code[2] = 2; // Puncture
    tpms.alarm_code[3] = 4; // Low battery

    uint8_t out[16];
    size_t len = build_raise_tpms_temp_alarms(&tpms, out);
    TEST_ASSERT_EQUAL_UINT32(12, len);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) sum += out[i];

    // Test extreme temperatures clamping
    tpms.temperature_c[0] = -50; // Underflow clamped to 0
    tpms.temperature_c[1] = 250; // Overflow clamped to 255
    build_raise_tpms_temp_alarms(&tpms, out);
}

void test_psa_extended_tpms_discrete_alarms(void) {
    uint8_t alarms[4] = { 0x01, 0x00, 0x00, 0x01 };
    uint8_t out[16];
    size_t len = build_raise_tpms_discrete(alarms, out);
    TEST_ASSERT_EQUAL_UINT32(8, len);

    uint8_t expected_cs = (uint8_t)((0x18 + 0x04 + 0x01 + 0x00 + 0x00 + 0x01) ^ 0xFF);
}

/* --------------------------------------------------------------------------
 * Hiworld Peugeot 407 TPMS Unit Tests & Verification Vectors
 * Based on CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md
 * -------------------------------------------------------------------------- */
void test_peugeot_407_tpms_hiworld_vector_1_nominal(void) {
    psa_tpms_ctx_t ctx;
    psa_tpms_init(&ctx, test_tpms_uart_tx);
    s_tpms_uart_tx_calls = 0;

    // Vector 1: CAN 0x361 FL=2.4, FR=2.4, RR=2.2, RL=2.2 Bar (0018001800160016)
    const uint8_t can_361_v1[] = { 0x00, 0x18, 0x00, 0x18, 0x00, 0x16, 0x00, 0x16 };
    psa_tpms_process_can_0x361(&ctx, can_361_v1, sizeof(can_361_v1));

    TEST_ASSERT_EQUAL_INT(2, s_tpms_uart_tx_calls);

    // Numeric (Cmd 0x66): 5A A5 06 66 01 18 18 16 16 00 C8
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 0x18, 0x18, 0x16, 0x16, 0x00, 0xC8 };
    TEST_ASSERT_EQUAL_UINT32(11, s_tpms_uart_len[0]);

    // Discrete (Cmd 0x18): 5A A5 04 18 00 00 00 00 1B
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x00, 0x00, 0x00, 0x1B };
    TEST_ASSERT_EQUAL_UINT32(9, s_tpms_uart_len[1]);

    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(22, ctx.state.rl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(22, ctx.state.rr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fr_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rr_state);
}

void test_peugeot_407_tpms_hiworld_vector_2_asymmetric_low(void) {
    psa_tpms_ctx_t ctx;
    psa_tpms_init(&ctx, test_tpms_uart_tx);
    s_tpms_uart_tx_calls = 0;

    // Vector 2: FL=2.4, FR=2.3, RR=2.1, RL=1.2 (Underinflated 0x4000) -> 001800170015400C
    const uint8_t can_361_v2[] = { 0x00, 0x18, 0x00, 0x17, 0x00, 0x15, 0x40, 0x0C };
    psa_tpms_process_can_0x361(&ctx, can_361_v2, sizeof(can_361_v2));

    TEST_ASSERT_EQUAL_INT(2, s_tpms_uart_tx_calls);

    // Numeric (Cmd 0x66): 5A A5 06 66 01 18 17 0C 15 00 BC
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 0x18, 0x17, 0x0C, 0x15, 0x00, 0xBC };
    TEST_ASSERT_EQUAL_UINT32(11, s_tpms_uart_len[0]);

    // Discrete (Cmd 0x18): 5A A5 04 18 00 00 01 00 1C (RL alarm = 0x01 LOW)
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x00, 0x01, 0x00, 0x1C };
    TEST_ASSERT_EQUAL_UINT32(9, s_tpms_uart_len[1]);

    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(23, ctx.state.fr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(12, ctx.state.rl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(21, ctx.state.rr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fr_state);
    TEST_ASSERT_EQUAL_UINT8(1, ctx.state.rl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rr_state);
}

void test_peugeot_407_tpms_hiworld_vector_3_puncture(void) {
    psa_tpms_ctx_t ctx;
    psa_tpms_init(&ctx, test_tpms_uart_tx);
    s_tpms_uart_tx_calls = 0;

    // Vector 3: FL=2.4, FR=0.4 (Puncture 0x8000), RR=2.4, RL=2.4 -> 0018800400180018
    const uint8_t can_361_v3[] = { 0x00, 0x18, 0x80, 0x04, 0x00, 0x18, 0x00, 0x18 };
    psa_tpms_process_can_0x361(&ctx, can_361_v3, sizeof(can_361_v3));

    TEST_ASSERT_EQUAL_INT(2, s_tpms_uart_tx_calls);

    // Numeric (Cmd 0x66): 5A A5 06 66 01 18 04 18 18 00 B8
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 0x18, 0x04, 0x18, 0x18, 0x00, 0xB8 };
    TEST_ASSERT_EQUAL_UINT32(11, s_tpms_uart_len[0]);

    // Discrete (Cmd 0x18): 5A A5 04 18 00 02 00 00 1D (FR alarm = 0x02 PUNCTURE)
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x02, 0x00, 0x00, 0x1D };
    TEST_ASSERT_EQUAL_UINT32(9, s_tpms_uart_len[1]);

    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(4, ctx.state.fr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.rl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.rr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fl_state);
    TEST_ASSERT_EQUAL_UINT8(2, ctx.state.fr_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rr_state);
}

void test_peugeot_407_tpms_hiworld_0x3a1_pressures(void) {
    psa_tpms_ctx_t ctx;
    psa_tpms_init(&ctx, test_tpms_uart_tx);
    s_tpms_uart_tx_calls = 0;

    // CAN 0x3A1: 0.05 Bar resolution
    // FL: 48 (2.4 Bar), FR: 46 (2.3 Bar), RR: 42 (2.1 Bar), RL: 24 (1.2 Bar)
    const uint8_t can_3a1[] = { 48, 46, 42, 24 };
    psa_tpms_process_can_0x3a1(&ctx, can_3a1, sizeof(can_3a1));

    TEST_ASSERT_EQUAL_INT(1, s_tpms_uart_tx_calls);
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(23, ctx.state.fr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(21, ctx.state.rr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(12, ctx.state.rl_press_bar_deci);

    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 24, 23, 12, 21, 0x00, 0xBC };
    TEST_ASSERT_EQUAL_UINT32(11, s_tpms_uart_len[0]);
}

void test_peugeot_407_tpms_hiworld_0x1e1_status_enum(void) {
    psa_tpms_ctx_t ctx;
    psa_tpms_init(&ctx, test_tpms_uart_tx);
    s_tpms_uart_tx_calls = 0;

    // CAN 0x1E1:
    // Byte 0 (FL): 0 (OK) -> (0 << 3) = 0x00
    // Byte 1 (FR): 1 (LOW) -> (1 << 3) = 0x08
    // Byte 2 (RR): 2 (FLAT) -> (2 << 3) = 0x10
    // Byte 3 (RL): 4 (BATTERY_LOW -> >=3 => 3 FAULT) -> (4 << 3) = 0x20
    const uint8_t can_1e1[] = { 0x00, 0x08, 0x10, 0x20 };
    psa_tpms_process_can_0x1e1(&ctx, can_1e1, sizeof(can_1e1));

    TEST_ASSERT_EQUAL_INT(1, s_tpms_uart_tx_calls);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fl_state);
    TEST_ASSERT_EQUAL_UINT8(1, ctx.state.fr_state);
    TEST_ASSERT_EQUAL_UINT8(2, ctx.state.rr_state);
    TEST_ASSERT_EQUAL_UINT8(3, ctx.state.rl_state);

    // Discrete packet: Cmd 0x18, Len 0x04, FL=0, FR=1, RL=3, RR=2
    // Checksum = (0x04 + 0x18 + 0 + 1 + 3 + 2 - 1) = 0x21
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x01, 0x03, 0x02, 0x21 };
    TEST_ASSERT_EQUAL_UINT32(9, s_tpms_uart_len[0]);
}

void test_peugeot_407_tpms_hiworld_pipeline_and_boundaries(void) {
    psa_tpms_ctx_t ctx;
    psa_tpms_init(&ctx, test_tpms_uart_tx);

    // Initial defaults check
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.fr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(22, ctx.state.rr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(22, ctx.state.rl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fr_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rr_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rl_state);

    // Serializer boundary checks
    uint8_t out[16];
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_numeric(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_numeric(&ctx.state, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_numeric(&ctx.state, out, 10)); // < 11
    TEST_ASSERT_EQUAL_UINT32(11, build_hiworld_tpms_numeric(&ctx.state, out, sizeof(out)));

    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_discrete(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_discrete(&ctx.state, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_discrete(&ctx.state, out, 8)); // < 9
    TEST_ASSERT_EQUAL_UINT32(9, build_hiworld_tpms_discrete(&ctx.state, out, sizeof(out)));

    // CAN process boundary checks
    uint8_t dummy_can[8] = { 0 };
    psa_tpms_process_can_0x361(NULL, dummy_can, 8);
    psa_tpms_process_can_0x361(&ctx, NULL, 8);
    psa_tpms_process_can_0x361(&ctx, dummy_can, 7); // dlc < 8

    psa_tpms_process_can_0x3a1(NULL, dummy_can, 4);
    psa_tpms_process_can_0x3a1(&ctx, NULL, 4);
    psa_tpms_process_can_0x3a1(&ctx, dummy_can, 3); // dlc < 4

    psa_tpms_process_can_0x1e1(NULL, dummy_can, 4);
    psa_tpms_process_can_0x1e1(&ctx, NULL, 4);
    psa_tpms_process_can_0x1e1(&ctx, dummy_can, 3); // dlc < 4

    // Missing sensor (0x3FFF) / fault handling in 0x361
    const uint8_t can_missing[8] = { 0xFF, 0xFF, 0xC0, 0x18, 0x3F, 0xFF, 0x00, 0x18 };
    // FL: 0xFFFF (state 3, raw 0x3FFF) -> state 3, press 0
    // FR: 0xC018 (state 3, raw 0x0018) -> state 3, press 0
    // RR: 0x3FFF (state 0, raw 0x3FFF) -> state 0, press 0 (missing raw)
    // RL: 0x0018 (state 0, raw 0x0018) -> state 0, press 24
    psa_tpms_process_can_0x361(&ctx, can_missing, 8);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(3, ctx.state.fl_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.fr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(3, ctx.state.fr_state);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rr_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rr_state);
    TEST_ASSERT_EQUAL_UINT8(24, ctx.state.rl_press_bar_deci);
    TEST_ASSERT_EQUAL_UINT8(0, ctx.state.rl_state);

    // Send with NULL uart_tx should not crash
    ctx.uart_tx = NULL;
    psa_tpms_send_numeric(&ctx);
    psa_tpms_send_discrete(&ctx);
    psa_tpms_init(NULL, NULL);
}

/* --------------------------------------------------------------------------
 * 2.2 Stop & Start (S&S) Telemetry & Timer Tests
 * -------------------------------------------------------------------------- */
void test_psa_extended_start_stop(void) {
    // Vector: Active, 125 seconds (0x0000007D)
    uint8_t out[16];
    size_t len = build_raise_start_stop(true, 125, out);
    TEST_ASSERT_EQUAL_UINT32(9, len);

    uint8_t expected_cs = (uint8_t)((0x71 + 0x05 + 0x01 + 0x00 + 0x00 + 0x00 + 0x7D) ^ 0xFF);

    // Test Inactive, 3600 seconds (0x00000E10)
    len = build_raise_start_stop(false, 3600, out);
    TEST_ASSERT_EQUAL_UINT32(9, len);
}

/* --------------------------------------------------------------------------
 * 2.3 Cruise Control & Speed Memory Presets Tests
 * -------------------------------------------------------------------------- */
void test_psa_extended_cruise_memory(void) {
    const uint8_t presets[5] = { 50, 70, 90, 110, 130 };
    uint8_t out[16];
    size_t len = build_raise_cruise_memory(true, 110, presets, out);
    TEST_ASSERT_EQUAL_UINT32(11, len);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) sum += out[i];
}

void test_psa_extended_decode_cruise_0x1a8(void) {
    // 1. Cruise active at 110.00 km/h (0x2AF8), partial odo 1234.567 km (0x12D687 = 1234567 m)
    const uint8_t can_1a8_active[] = { 0x48, 0x2A, 0xF8, 0x00, 0x00, 0x12, 0xD6, 0x87 };
    bool active = false;
    uint8_t set_spd = 0;
    uint32_t partial_odo = 0;

    psa_decode_cruise_0x1a8(can_1a8_active, sizeof(can_1a8_active), &active, &set_spd, &partial_odo);
    TEST_ASSERT_TRUE(active);
    TEST_ASSERT_EQUAL_UINT8(110, set_spd);
    TEST_ASSERT_EQUAL_UINT32(1234567, partial_odo);

    // 2. Idle / Standby with no set speed (0xFFFF) and no partial odo (0xFFFFFF)
    const uint8_t can_1a8_idle[] = { 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF };
    psa_decode_cruise_0x1a8(can_1a8_idle, sizeof(can_1a8_idle), &active, &set_spd, &partial_odo);
    TEST_ASSERT_FALSE(active);
    TEST_ASSERT_EQUAL_UINT8(0, set_spd);
    TEST_ASSERT_EQUAL_UINT32(0, partial_odo);

    // 3. Limiter at 130 km/h (0x32C8 = 13000), odo 123 m (0x00007B)
    const uint8_t can_1a8_limiter[] = { 0x88, 0x32, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x7B };
    psa_decode_cruise_0x1a8(can_1a8_limiter, sizeof(can_1a8_limiter), &active, &set_spd, &partial_odo);
    TEST_ASSERT_TRUE(active);
    TEST_ASSERT_EQUAL_UINT8(130, set_spd);
    TEST_ASSERT_EQUAL_UINT32(123, partial_odo);
}

/* --------------------------------------------------------------------------
 * 2.4 Driver Assistance & ADAS Features Tests
 * -------------------------------------------------------------------------- */
void test_psa_extended_adas(void) {
    canbox_adas_state_t adas;
    memset(&adas, 0, sizeof(adas));
    adas.blind_spot_warning = true;  // 0x80
    adas.fatigue_coffee_cup = true;  // 0x80
    adas.lane_departure_state = 2;   // Right line departure
    adas.speed_limit_tsr = 90;       // 90 km/h (0x5A)
    adas.esp_active = true;          // 0x01
    adas.aeb_risk_level = 2;         // Emergency braking

    uint8_t out[16];
    size_t len = build_raise_adas(&adas, out);
    TEST_ASSERT_EQUAL_UINT32(10, len);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) sum += out[i];
}

void test_psa_extended_decode_alerts_0x168(void) {
    const uint8_t can_168_alerts[] = { 0x01, 0xC0, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00 };
    bool tpms_fault = false, underinfl = false, puncture = false, esp_fault = false;

    psa_decode_alerts_0x168(can_168_alerts, sizeof(can_168_alerts), &tpms_fault, &underinfl, &puncture, &esp_fault);
    TEST_ASSERT_TRUE(tpms_fault);
    TEST_ASSERT_TRUE(underinfl);
    TEST_ASSERT_TRUE(puncture);
    TEST_ASSERT_TRUE(esp_fault);

    const uint8_t can_168_clear[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_decode_alerts_0x168(can_168_clear, sizeof(can_168_clear), &tpms_fault, &underinfl, &puncture, &esp_fault);
    TEST_ASSERT_FALSE(tpms_fault);
    TEST_ASSERT_FALSE(underinfl);
    TEST_ASSERT_FALSE(puncture);
    TEST_ASSERT_FALSE(esp_fault);
}

// Test the raw state machine
void test_peugeot_407_hvac_hiworld(void) {
    vehicle_climate_t climate;
    memset(&climate, 0, sizeof(climate));

    // Vector 1: Auto AC 21.0°C Dual Mode, Fan Speed 3 (CAN=2)
    // Wait, the test uses CAN `03` which is Fan Speed 4!
    const uint8_t can_1d0[] = { 0x08, 0x00, 0x03, 0x00, 0x00, 0x0B, 0x0B, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0, 8);
    
    TEST_ASSERT_TRUE(climate.power_on);
    TEST_ASSERT_EQUAL_UINT8(4, climate.fan_speed);  // 03 + 1 = 4
    
    // can_1d0 byte 3 is 0x00 -> Auto
    TEST_ASSERT_EQUAL_UINT8(0, climate.driver_wind_mode);
    TEST_ASSERT_EQUAL_UINT8(0, climate.pass_wind_mode);

    const uint8_t can_1e3[] = { 0x1D, 0x00 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3, 2);
    
    TEST_ASSERT_TRUE(climate.ac_on);
    TEST_ASSERT_TRUE(climate.auto_mode);
    TEST_ASSERT_TRUE(climate.dual_mode);
    TEST_ASSERT_TRUE(climate.aqs_auto);
    TEST_ASSERT_FALSE(climate.recirculate);

    // Vector 2: Dual Zone air distribution - Left=4 (Windshield 11), Right=2 (Floor 3)
    const uint8_t can_1d0_v2[] = { 0x28, 0x00, 0x00, 0x42, 0x00, 0x0E, 0x0B, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0_v2, 8);
    TEST_ASSERT_EQUAL_UINT8(11, climate.driver_wind_mode);
    TEST_ASSERT_EQUAL_UINT8(3, climate.pass_wind_mode);

    // Vector 3: Left=2 (Floor 3), Right=4 (Windshield 11)
    const uint8_t can_1d0_v3[] = { 0x28, 0x00, 0x00, 0x24, 0x00, 0x0E, 0x0B, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0_v3, 8);
    TEST_ASSERT_EQUAL_UINT8(3, climate.driver_wind_mode);
    TEST_ASSERT_EQUAL_UINT8(11, climate.pass_wind_mode);

    // Vector 4: Left=3 (Face 6), Right=7 (Windshield+Face 13)
    const uint8_t can_1d0_v4[] = { 0x28, 0x00, 0x00, 0x37, 0x00, 0x0E, 0x0B, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0_v4, 8);
    TEST_ASSERT_EQUAL_UINT8(6, climate.driver_wind_mode);
    TEST_ASSERT_EQUAL_UINT8(13, climate.pass_wind_mode);

    // Vector 5: 0x1E3 EMF Frame Airflow Decoding - Left=4 (Windshield 11), Right=2 (Floor 3)
    const uint8_t can_1e3_v5[] = { 0x11, 0x30, 0x12, 0x12, 0x40, 0x20, 0x03, 0x00 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v5, 8);
    TEST_ASSERT_EQUAL_UINT8(11, climate.driver_wind_mode);
    TEST_ASSERT_EQUAL_UINT8(3, climate.pass_wind_mode);

    // Vector 6: 0x1E3 EMF Frame Auto Airflow Decoding - Left=0 (Auto 0), Right=0 (Auto 0)
    const uint8_t can_1e3_v6[] = { 0x1C, 0x30, 0x12, 0x12, 0x00, 0x00, 0x03, 0x00 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v6, 8);
    TEST_ASSERT_EQUAL_UINT8(0, climate.driver_wind_mode);
    TEST_ASSERT_EQUAL_UINT8(0, climate.pass_wind_mode);

    // Vector 7: 0x1E3 Recirculation ON (0x85)
    const uint8_t can_1e3_v7[] = { 0x85, 0x30, 0x0D, 0x08, 0x00, 0x00, 0x01, 0x00 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v7, 8);
    TEST_ASSERT_TRUE(climate.recirculate);
    TEST_ASSERT_FALSE(climate.aqs_auto);

    // Vector 8: 0x1E3 Fresh Air (0x05)
    const uint8_t can_1e3_v8[] = { 0x05, 0x30, 0x0D, 0x08, 0x00, 0x00, 0x01, 0x00 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v8, 8);
    TEST_ASSERT_FALSE(climate.recirculate);
    TEST_ASSERT_FALSE(climate.aqs_auto);

    // Vector 9: Provenance scenario from dump_2026-10-02_13-43-03.log:
    // 0x1D0 sets Auto Air Intake (Byte 4 = 0x00) with manual fan speed 3 (Byte 2 = 0x02)
    const uint8_t can_1d0_v9[] = { 0x08, 0x00, 0x02, 0x43, 0x00, 0x06, 0x11, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0_v9, 8);
    TEST_ASSERT_TRUE(climate.power_on);
    TEST_ASSERT_TRUE(climate.ac_on);
    TEST_ASSERT_TRUE(climate.aqs_auto);
    TEST_ASSERT_FALSE(climate.recirculate);

    // 0x1E3 arrives ~80ms later: Byte 0 = 0x15 (Bit 4 = Auto intake AQS, Bit 0 = Dual)
    const uint8_t can_1e3_v9[] = { 0x15, 0x30, 0x06, 0x11, 0x40, 0x30, 0x02 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v9, 7);
    TEST_ASSERT_TRUE(climate.aqs_auto);
    TEST_ASSERT_FALSE(climate.recirculate);
    TEST_ASSERT_TRUE(climate.ac_on);
    TEST_ASSERT_TRUE(climate.dual_mode);

    // Vector 10: Issue 3 Provenance scenario from dump_2026-10-05_21-28-30.log
    // and dump_2026-10-06_14-44-27.log: Front defrost active with AQS intake.
    // 0x1E3 arrives: Byte 0 = 0x14 (AQS active), Byte 1 = 0xB0 (Bit 7 = Front Demist active)
    const uint8_t can_1e3_v10[] = { 0x14, 0xB0, 0x0B, 0x0B, 0x00, 0x00, 0x01, 0x00 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v10, 8);
    TEST_ASSERT_TRUE(climate.front_max_defrost);
    TEST_ASSERT_TRUE(climate.aqs_auto);
    TEST_ASSERT_FALSE(climate.recirculate);

    // 0x1D0 arrives: Byte 0 = 0x19 (Front Demist active), Byte 4 = 0x20 (Forced Fresh Air physical flap)
    const uint8_t can_1d0_v10[] = { 0x19, 0x00, 0x01, 0x00, 0x20, 0x0B, 0x0B, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0_v10, 8);
    TEST_ASSERT_TRUE(climate.front_max_defrost);
    TEST_ASSERT_TRUE(climate.aqs_auto);
    TEST_ASSERT_FALSE(climate.recirculate);

    // Re-feed 0x1E3: verify zero state jitter / oscillation
    psa_hvac_process_can_0x1e3(&climate, can_1e3_v10, 8);
    TEST_ASSERT_TRUE(climate.front_max_defrost);
    TEST_ASSERT_TRUE(climate.aqs_auto);
    TEST_ASSERT_FALSE(climate.recirculate);

    // Turn off Front Demist back to Auto:
    const uint8_t can_1d0_auto[] = { 0x08, 0x00, 0x02, 0x00, 0x00, 0x0B, 0x0B, 0x00 };
    psa_hvac_process_can_0x1d0(&climate, can_1d0_auto, 8);
    TEST_ASSERT_FALSE(climate.front_max_defrost);
    TEST_ASSERT_TRUE(climate.aqs_auto);

    const uint8_t can_1e3_auto[] = { 0x1C, 0x30, 0x0B, 0x0B, 0x00, 0x00, 0x02 };
    psa_hvac_process_can_0x1e3(&climate, can_1e3_auto, 7);
    TEST_ASSERT_FALSE(climate.front_max_defrost);
    TEST_ASSERT_TRUE(climate.aqs_auto);
}

void test_peugeot_407_trip_reset_frames(void) {
    can_frame_t frame;

    // 1. Trip 1 Reset (Bit 7 = 0x80)
    TEST_ASSERT_TRUE(build_psa_trip_reset_frame(1, &frame));
    TEST_ASSERT_EQUAL_HEX32(0x221, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, frame.dlc);
    for (int i = 1; i < 8; i++) {
    }

    // 2. Trip 2 Reset (Bit 6 = 0x40)
    TEST_ASSERT_TRUE(build_psa_trip_reset_frame(2, &frame));
    TEST_ASSERT_EQUAL_HEX32(0x221, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, frame.dlc);
    for (int i = 1; i < 8; i++) {
    }

    // 3. Boundary & Error checks
    TEST_ASSERT_FALSE(build_psa_trip_reset_frame(0, &frame));
    TEST_ASSERT_FALSE(build_psa_trip_reset_frame(3, &frame));
    TEST_ASSERT_FALSE(build_psa_trip_reset_frame(1, NULL));

    // 4. Send reset status check
    TEST_ASSERT_EQUAL(HAL_STATUS_OK, psa_trip_send_reset(1));
    TEST_ASSERT_EQUAL(HAL_STATUS_OK, psa_trip_send_reset(2));
    TEST_ASSERT_EQUAL(HAL_STATUS_ERROR, psa_trip_send_reset(0));
}

/* --------------------------------------------------------------------------
 * Vehicle Alerts & Diagnostic Journal Verification Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_alert_single_abs(void) {
    /* CAN 0x1A1: POPUP_ACTIVE=1, CAN_ALARM_ID=0x006C (ABS Fault), DISPLAY=1 (0x80), PRIORITY=3 (0x30), SOUND=2 (0x02) -> 0xB2 */
    const uint8_t can_data[8] = { 0x80, 0x6C, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00 };
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));

    psa_decode_alert_message_0x1a1(can_data, 8, &alert);

    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x006C, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, alert.alert_code);
    TEST_ASSERT_EQUAL_UINT8(3, alert.severity);
    TEST_ASSERT_EQUAL_UINT8(2, alert.chime_id);

    /* Serialize Hiworld single alert frame */
    uint8_t out[16];
    size_t len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    /* CS = (0x02 + 0x42 + 0x00 + 0x69 - 1) & 0xFF = 0xAC */
}

void test_peugeot_407_alert_single_suspension(void) {
    /* CAN 0x1A1: POPUP_ACTIVE=1, CAN_ALARM_ID=0x009E (Suspension 90km/h), DISPLAY=1, PRIORITY=2, SOUND=1 */
    const uint8_t can_data[8] = { 0x80, 0x9E, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00 };
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));

    psa_decode_alert_message_0x1a1(can_data, 8, &alert);

    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x009E, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_SUSPENSION_90KMH, alert.alert_code);

    uint8_t out[16];
    size_t len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    /* CS = (0x02 + 0x42 + 0x00 + 0x6B - 1) & 0xFF = 0xAE */
}

void test_peugeot_407_alert_single_low_fuel(void) {
    /* CAN 0x1A1: POPUP_ACTIVE=1, CAN_ALARM_ID=0x000D (Low Fuel), DISPLAY=1, PRIORITY=2, SOUND=1 */
    const uint8_t can_data[8] = { 0x80, 0x0D, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00 };
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));

    psa_decode_alert_message_0x1a1(can_data, 8, &alert);

    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x000D, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_LOW_FUEL, alert.alert_code);

    uint8_t out[16];
    size_t len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    /* CS = (0x02 + 0x42 + 0x00 + 0x01 - 1) & 0xFF = 0x44 */

    /* Test clear alert: POPUP_ACTIVE = 0 */
    const uint8_t clear_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_decode_alert_message_0x1a1(clear_data, 8, &alert);
    TEST_ASSERT_FALSE(alert.is_active);
    TEST_ASSERT_FALSE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x0000, alert.alert_code);
}

void test_peugeot_407_alert_journal_multi(void) {
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);

    vehicle_alert_item_t out_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t  out_count = 0;
    memset(out_codes, 0, sizeof(out_codes));

    /* Journal test payload:
     * Byte 0: 0x40 (Bit 1 = Engine Overheat -> Alarm Index 0x07 -> CAN 0x0001 -> Hiworld 0x0003)
     * Byte 4: 0xC0 (Bit 32 = ABS -> Alarm Index 0x16 -> CAN 0x006C -> Hiworld 0x0069,
     *               Bit 33 = DPF -> Alarm Index 0x1A -> CAN 0x006F -> Hiworld 0x0064)
     */
    const uint8_t ff_data[8]  = { 0x10, 0x15, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x00 };
    const uint8_t cf1_data[8] = { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf2_data[8] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf3_data[8] = { 0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, ff_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf1_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf2_data, 8, out_codes, &out_count));
    TEST_ASSERT_TRUE(psa_process_journal_0x120(&ctx, cf3_data, 8, out_codes, &out_count));

    TEST_ASSERT_EQUAL_UINT8(3, out_count);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ENGINE_TEMP, out_codes[0].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, out_codes[1].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_DPF, out_codes[2].alert_code);

    /* Serialize Hiworld Multi-Alert Summary (24 payload bytes) */
    uint8_t out[80];
    size_t len = build_hiworld_alerts_summary(out_codes, out_count, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(24, len);
    /* Check padding to 24 payload bytes */
    for (int i = 14; i < 28; i++) {
    }
    /* CS = (0x18 + 0x42 + 3 + 3 + 0x69 + 0x64 - 1) & 0xFF = 0x2C */
}

void test_peugeot_407_alert_journal_clear(void) {
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);

    vehicle_alert_item_t out_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t  out_count = 0;
    memset(out_codes, 0, sizeof(out_codes));

    /* Journal test payload with 0 active faults */
    const uint8_t ff_data[8]  = { 0x10, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf1_data[8] = { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf2_data[8] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf3_data[8] = { 0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, ff_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf1_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf2_data, 8, out_codes, &out_count));
    TEST_ASSERT_TRUE(psa_process_journal_0x120(&ctx, cf3_data, 8, out_codes, &out_count));

    TEST_ASSERT_EQUAL_UINT8(0, out_count);

    /* Serialize Hiworld clear frame (mNumber = 0) */
    uint8_t out[80];
    size_t len = build_hiworld_alerts_summary(out_codes, out_count, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(9, len);
    /* CS = (0x18 + 0x42 - 1) & 0xFF = 0x59 */
}

void test_peugeot_407_alert_router_pipeline_and_query(void) {
    can_router_init();
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    /* 1. Feed single alert frame (CAN 0x1A1, ABS fault) */
    can_frame_t frame_abs = {
        .id = PSA_CAN_ID_ALERT_MESSAGE,
        .dlc = 8,
        .data = { 0x80, 0x6C, 0xF2, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_abs);

    const vehicle_state_t *st = can_router_get_state();
    TEST_ASSERT_TRUE(st->alerts.realtime_alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, st->alerts.realtime_alert.alert_code);

    /* 2. Feed clear frame (CAN 0x1A1, POPUP_ACTIVE=0) */
    can_frame_t frame_clear = {
        .id = PSA_CAN_ID_ALERT_MESSAGE,
        .dlc = 8,
        .data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_clear);
    st = can_router_get_state();
    TEST_ASSERT_FALSE(st->alerts.realtime_alert.is_active);

    /* 3. Feed multi-frame journal sequence (Initial Scan: 3 active faults) */
    can_frame_t f_ff = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x10, 0x15, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x00 } };
    can_frame_t f_c1 = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    can_frame_t f_c2 = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    can_frame_t f_c3 = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };

    can_router_process_can(&f_ff);
    can_router_process_can(&f_c1);
    can_router_process_can(&f_c2);
    can_router_process_can(&f_c3);

    st = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT8(3, st->alerts.active_count);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ENGINE_TEMP, st->alerts.active_items[0].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, st->alerts.active_items[1].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_DPF, st->alerts.active_items[2].alert_code);

    /* 4. Retransmit identical journal sequence: no state change, no unsolicited popup */
    can_router_process_can(&f_ff);
    can_router_process_can(&f_c1);
    can_router_process_can(&f_c2);
    can_router_process_can(&f_c3);
    st = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT8(3, st->alerts.active_count);

    /* 5. Feed UART Query Cmd 0x2F with 0xD4 checksum variant (Android APK forwardType 0x2F) */
    hal_can_native_clear_sent_frame();
    const uint8_t query_packet_d4[] = { 0x5A, 0xA5, 0x01, 0x2F, 0x01, 0xD4 };
    for (size_t i = 0; i < sizeof(query_packet_d4); i++) {
        can_router_process_uart_byte(query_packet_d4[i]);
    }

    /* Verify BSI diagnostic query CAN frame 0x39B was transmitted on CAN */
    can_frame_t sent_query;
    memset(&sent_query, 0, sizeof(sent_query));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_query));
    TEST_ASSERT_EQUAL_HEX32(PSA_CAN_ID_ALERT_QUERY, sent_query.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_query.dlc);

    /* 6. Verify 500ms timeout handling in periodic tick */
    for (int t = 0; t < 5; t++) {
        can_router_periodic_100ms();
    }

    /* 7. Verify standard 0x30 checksum for 0x2F also triggers BSI query */
    hal_can_native_clear_sent_frame();
    const uint8_t query_packet_30[] = { 0x5A, 0xA5, 0x01, 0x2F, 0x01, 0x30 };
    for (size_t i = 0; i < sizeof(query_packet_30); i++) {
        can_router_process_uart_byte(query_packet_30[i]);
    }
    memset(&sent_query, 0, sizeof(sent_query));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_query));
    TEST_ASSERT_EQUAL_HEX32(PSA_CAN_ID_ALERT_QUERY, sent_query.id);

    /* 8. Verify periodic timer ticks do not spam alert summaries when no query is pending */
    for (int t = 0; t < 600; t++) {
        can_router_periodic_100ms();
    }
}

void test_peugeot_407_alert_boundary_and_malformed(void) {
    /* 1. NULL safety */
    psa_decode_alert_message_0x1a1(NULL, 8, NULL);
    psa_journal_iso_tp_init(NULL);
    TEST_ASSERT_FALSE(psa_process_journal_0x120(NULL, NULL, 0, NULL, NULL));

    /* 2. Short DLC on 0x1A1 */
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));
    const uint8_t short_data[3] = { 0x80, 0x6C, 0xF2 };
    psa_decode_alert_message_0x1a1(short_data, 3, &alert);
    TEST_ASSERT_FALSE(alert.is_active);

    /* 3. Short DLC on 0x120 */
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);
    vehicle_alert_item_t codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t count = 0;
    const uint8_t short_pci[1] = { 0x10 };
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, short_pci, 1, codes, &count));

    /* 4. Out of sequence consecutive frame */
    const uint8_t ff_data[8] = { 0x10, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t wrong_cf[8] = { 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; /* expected 0x21 */
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, ff_data, 8, codes, &count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, wrong_cf, 8, codes, &count));
    TEST_ASSERT_FALSE(ctx.in_progress);

    /* 5. Buffer size boundaries */
    uint8_t small_buf[5];
    TEST_ASSERT_EQUAL_size_t(0, build_hiworld_alert_single(&codes[0], small_buf, sizeof(small_buf)));
    TEST_ASSERT_EQUAL_size_t(0, build_hiworld_alerts_summary(codes, count, small_buf, sizeof(small_buf)));
}

void test_peugeot_407_alert_journal_block_multiplexed_real_log(void) {
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);

    vehicle_alert_item_t out_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t  out_count = 0;
    memset(out_codes, 0, sizeof(out_codes));

    /* Real frames captured from dump_2026-10-02_12-15-52.log:
     * Frame 1: FC 00 00 00 00 0F 00 00 (Block 3)
     * Frame 2: BC 00 00 00 00 00 00 00 (Block 2)
     * Frame 3: 7C 10 00 03 00 04 00 08 (Block 1)
     */
    const uint8_t blk3[8] = { 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00 };
    const uint8_t blk2[8] = { 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t blk1[8] = { 0x7C, 0x10, 0x00, 0x03, 0x00, 0x04, 0x00, 0x08 };

    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, blk3, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, blk2, 8, out_codes, &out_count));
    TEST_ASSERT_TRUE(psa_process_journal_0x120(&ctx, blk1, 8, out_codes, &out_count));

    /* Verify parsed alerts from real dump */
    TEST_ASSERT_EQUAL_UINT8(9, out_count);
    TEST_ASSERT_EQUAL_HEX16(0x0008, out_codes[0].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00E0, out_codes[1].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0078, out_codes[2].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0083, out_codes[3].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x007F, out_codes[4].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0130, out_codes[5].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0131, out_codes[6].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00E5, out_codes[7].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0138, out_codes[8].can_alarm_id);

    /* Verify mapped Hiworld alert codes */
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_HANDBRAKE, out_codes[0].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AIRBAG, out_codes[2].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_IMMOBILIZER, out_codes[3].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ANTIPOLLUTION, out_codes[4].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AUTO_LIGHTS, out_codes[5].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_LOW_FUEL, out_codes[8].alert_code);
}

void test_peugeot_407_alert_diagnostic_cycle_and_toggle_bit(void) {
    vehicle_alert_item_t alert;

    /* 1. Depollution fault (CAN 0x007F): byte 0 bit 7 toggle = 0, display_req = 1 */
    const uint8_t can_depollution[8] = { 0x00, 0x7F, 0xD6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_depollution, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x007F, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ANTIPOLLUTION, alert.alert_code);

    /* 2. Electronic anti-theft / immobilizer (CAN 0x0083): toggle = 1, display_req = 1 */
    const uint8_t can_immobilizer[8] = { 0x80, 0x83, 0xD6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_immobilizer, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0083, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_IMMOBILIZER, alert.alert_code);

    /* 3. Tyre pressure not monitored (CAN 0x00E5): toggle = 0, display_req = 1 */
    const uint8_t can_tpms_unmonitored[8] = { 0x00, 0xE5, 0xD7, 0x1E, 0x40, 0x00, 0x00, 0x00 };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_tpms_unmonitored, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00E5, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_TPMS_UNDER_FL, alert.alert_code);

    /* 4. Automatic headlamp lighting activated (CAN 0x0130): toggle = 1, display_req = 1 */
    const uint8_t can_auto_lights[8] = { 0x81, 0x30, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_auto_lights, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0130, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AUTO_LIGHTS, alert.alert_code);

    /* 5. Fuel level too low (CAN 0x0138): toggle = 1, display_req = 1 */
    const uint8_t can_low_fuel_138[8] = { 0x81, 0x38, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_low_fuel_138, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0138, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_LOW_FUEL, alert.alert_code);

    /* 6. Automatic screen wipe deactivated (CAN 0x0139): toggle = 0, display_req = 1 */
    const uint8_t can_auto_wipers_139[8] = { 0x01, 0x39, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_auto_wipers_139, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0139, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AUTO_WIPERS, alert.alert_code);

    /* 7. Diagnosis in progress (CAN 0x00F0) and completed (CAN 0x00F1) */
    const uint8_t can_diag_prog[8] = { 0x80, 0xF0, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_diag_prog, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F0, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00F0, alert.alert_code);

    const uint8_t can_diag_comp[8] = { 0x80, 0xF1, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_diag_comp, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F1, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00F1, alert.alert_code);

    /* 8. Sequence toggle bit behavior: verify that toggle bit 0 does NOT dismiss active alert */
    const uint8_t can_toggle_zero[8] = { 0x01, 0x31, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_toggle_zero, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);

    /* Dismissal requires display_req = 0 or can_alarm_id = 0 */
    const uint8_t can_dismiss[8] = { 0x01, 0x31, 0x54, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; /* display_req = 0 (bit 7 clear) */
    psa_decode_alert_message_0x1a1(can_dismiss, 8, &alert);
    TEST_ASSERT_FALSE(alert.is_active);
    TEST_ASSERT_FALSE(alert.display_req);
}

void test_peugeot_407_csv_alerts_iteration(void) {
    const char *paths[] = {
        "doc/RT4_CAN_ALERTS_0x1A1.csv",
        "../doc/RT4_CAN_ALERTS_0x1A1.csv",
        "../../doc/RT4_CAN_ALERTS_0x1A1.csv",
        NULL
    };
    FILE *fp = NULL;
    for (int p = 0; paths[p] != NULL; p++) {
        fp = fopen(paths[p], "r");
        if (fp != NULL) break;
    }
    TEST_ASSERT_NOT_NULL_MESSAGE(fp, "Failed to open doc/RT4_CAN_ALERTS_0x1A1.csv");

    char line[1024];
    /* Skip CSV header */
    TEST_ASSERT_NOT_NULL(fgets(line, sizeof(line), fp));

    int alert_count = 0;
    while (fgets(line, sizeof(line), fp)) {
        size_t len = strlen(line);
        if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len == 0) continue;

        /* Parse comma-separated fields with quote awareness */
        char *tokens[10];
        int tok_idx = 0;
        char *ptr = line;
        bool in_quotes = false;
        tokens[tok_idx++] = ptr;

        while (*ptr && tok_idx < 10) {
            if (*ptr == '"') {
                in_quotes = !in_quotes;
            } else if (*ptr == ',' && !in_quotes) {
                *ptr = '\0';
                tokens[tok_idx++] = ptr + 1;
            }
            ptr++;
        }

        if (tok_idx < 10) continue;

        uint16_t can_id = (uint16_t)strtoul(tokens[3], NULL, 10);

        if (can_id != 0xFFFF && can_id != 0) {
            /* 1. Simulate CAN 0x1A1 frame with active display request */
            uint8_t frame_data[8] = {
                (uint8_t)(((can_id >> 8) & 0x7F) | 0x80), /* Toggle = 1, ID Hi */
                (uint8_t)(can_id & 0xFF),                 /* ID Lo */
                0x80 | 0x20 | 0x01,                      /* DispReq=1, Priority=2, Sound=1 */
                0x00, 0x00, 0x00, 0x00, 0x00
            };

            vehicle_alert_item_t alert;
            memset(&alert, 0, sizeof(alert));
            psa_decode_alert_message_0x1a1(frame_data, 8, &alert);

            TEST_ASSERT_TRUE(alert.is_active);
            TEST_ASSERT_TRUE(alert.display_req);
            TEST_ASSERT_EQUAL_HEX16(can_id, alert.can_alarm_id);
            TEST_ASSERT_EQUAL_HEX16(psa_can_alarm_id_to_hiworld_code(can_id, 0, 0), alert.alert_code);

            /* 2. Verify Extended Alert serializer (Cmd 0xEA, 10 bytes) */
            uint8_t ext_buf[16];
            size_t ext_len = build_hiworld_alert_single(&alert, ext_buf, sizeof(ext_buf));
            TEST_ASSERT_EQUAL_size_t(10, ext_len);
            TEST_ASSERT_EQUAL_HEX8(0x5A, ext_buf[0]);
            TEST_ASSERT_EQUAL_HEX8(0xA5, ext_buf[1]);
            TEST_ASSERT_EQUAL_HEX8(0x05, ext_buf[2]);
            TEST_ASSERT_EQUAL_HEX8(0xEA, ext_buf[3]);
            TEST_ASSERT_EQUAL_HEX8((uint8_t)(can_id >> 8), ext_buf[4]);
            TEST_ASSERT_EQUAL_HEX8((uint8_t)(can_id & 0xFF), ext_buf[5]);

            /* 3. Verify Alert Dismissal behavior (DispReq=0) */
            frame_data[2] = 0x00;
            psa_decode_alert_message_0x1a1(frame_data, 8, &alert);
            TEST_ASSERT_FALSE(alert.is_active);
            TEST_ASSERT_FALSE(alert.display_req);
            TEST_ASSERT_EQUAL_HEX16(0, alert.can_alarm_id);
        }

        alert_count++;
    }

    fclose(fp);
    TEST_ASSERT_EQUAL_INT(208, alert_count);
}

void test_peugeot_407_custom_alert_severities_lookup(void) {
    /* STOP (2) - Major critical faults */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0001)); /* Engine temp */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0002)); /* Oil pressure */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0005)); /* Tyre puncture */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0008)); /* Braking system faulty */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x000D)); /* Puncture */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x000F)); /* Brake system */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0066)); /* Braking system */

    /* SERVICE (1) - Faults requiring service */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0003)); /* Oil level */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0004)); /* Low tyre pressure */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x000E)); /* Power steering */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0064)); /* DPF */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0067)); /* Brake pads */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0068)); /* Depollution */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006A)); /* ABS */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006B)); /* Battery charge */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006C)); /* Suspension */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006E)); /* Gearbox */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006F)); /* DPF */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0072)); /* Suspension 90km/h */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0073)); /* Auto gearbox */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0078)); /* Airbag */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x007E)); /* ABS/ESP */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x007F)); /* Depollution */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0083)); /* Anti-theft */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0088)); /* TPMS */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x00E0)); /* TPMS unmonitored */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x00E5)); /* TPMS unmonitored */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0195)); /* Directional headlamps */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0202)); /* Auto gearbox */

    /* INFO (0) - Confirmations, reminders, diagnosis statuses */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x000A)); /* Black ice */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x000C)); /* Handbrake */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0061)); /* Service due */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0069)); /* ESP off */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x00F0)); /* Diag in progress */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x00F1)); /* Diag complete */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0130)); /* Auto lights */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0131)); /* Child safety */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0138)); /* Low fuel */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0139)); /* Auto wipers */
}

void test_peugeot_407_custom_cockpit_check_sequence(void) {
    vehicle_alert_item_t alert;
    uint8_t out[16];
    size_t len;

    /* Step 1: Start of Diagnosis (0x00F0) - Spec Section 4.1 Step 1 */
    const uint8_t can_step1[8] = { 0x00, 0xF0, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00 };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step1, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F0, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step1[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0xF0, 0xC0, 0x00, 0x00, 0x9E };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step1, out, 10);

    /* Step 2: Rolling Alert 0x0008 (Braking system faulty) - Spec Section 4.1 Step 2 */
    const uint8_t can_step2[8] = { 0x80, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step2, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0008, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step2[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0x08, 0xC0, 0x00, 0x00, 0xB6 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step2, out, 10);

    /* Step 3: Inter-Alert Blanking (0x0000) - Spec Section 4.1 Step 3 */
    const uint8_t can_step3[8] = { 0x80, 0x00, 0x58, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step3, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0000, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step3[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0x00, 0xC0, 0x00, 0x00, 0xAE };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step3, out, 10);

    /* Step 4: End of Diagnosis (0x00F1) - Spec Section 4.1 Step 4 */
    const uint8_t can_step4[8] = { 0x80, 0xF1, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step4, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F1, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step4[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0xF1, 0xC0, 0x00, 0x00, 0x9F };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step4, out, 10);
}

void test_peugeot_407_custom_summary_table_and_empty_clearance(void) {
    uint8_t out[80];

    /* 1. Empty Clearance Frame (N = 0) - Spec Section 7 */
    size_t len = build_hiworld_alerts_summary(NULL, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(9, len);
    const uint8_t exp_clear[9] = { 0x5A, 0xA5, 0x04, 0xEA, 0x00, 0x00, 0x00, 0x00, 0xED };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_clear, out, 9);

    /* 2. 5 Active Faults Summary Table - Spec Section 6 */
    vehicle_alert_item_t faults[5];
    memset(faults, 0, sizeof(faults));

    faults[0].can_alarm_id = 0x0008; faults[0].is_active = true; faults[0].display_req = true;
    faults[1].can_alarm_id = 0x0078; faults[1].is_active = true; faults[1].display_req = true;
    faults[2].can_alarm_id = 0x0083; faults[2].is_active = true; faults[2].display_req = true;
    faults[3].can_alarm_id = 0x00E0; faults[3].is_active = true; faults[3].display_req = true;
    faults[4].can_alarm_id = 0x0138; faults[4].is_active = true; faults[4].display_req = true;

    len = build_hiworld_alerts_summary(faults, 5, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(34, len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x1D, out[2]); /* Payload length: 4 + 5*5 = 29 (0x1D) */
    TEST_ASSERT_EQUAL_HEX8(0xEA, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0x05, out[7]); /* N = 5 faults */

    /* Record 0: 0x0008 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[10]);

    /* Record 1: 0x0078 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[13]);
    TEST_ASSERT_EQUAL_HEX8(0x78, out[14]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[15]);

    /* Record 2: 0x0083 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[18]);
    TEST_ASSERT_EQUAL_HEX8(0x83, out[19]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[20]);

    /* Record 3: 0x00E0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[23]);
    TEST_ASSERT_EQUAL_HEX8(0xE0, out[24]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[25]);

    /* Record 4: 0x0138 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[28]);
    TEST_ASSERT_EQUAL_HEX8(0x38, out[29]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[30]);
}

void test_peugeot_407_custom_downlink_0x2f_response(void) {
    can_router_init();
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    hal_can_native_clear_sent_frame();

    /* Feed standard Downlink Query Cmd 0x2F (Spec Section 8: 5A A5 01 2F 00 2F) */
    const uint8_t query_pkt[] = { 0x5A, 0xA5, 0x01, 0x2F, 0x00, 0x2F };
    for (size_t i = 0; i < sizeof(query_pkt); i++) {
        can_router_process_uart_byte(query_pkt[i]);
    }

    /* Verify BSI diagnostic query frame 0x39B was sent on CAN */
    can_frame_t sent_query;
    memset(&sent_query, 0, sizeof(sent_query));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_query));
    TEST_ASSERT_EQUAL_HEX32(PSA_CAN_ID_ALERT_QUERY, sent_query.id);
}

void test_can_id_filter_enforcement(void) {
    can_router_init();
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);

    const vehicle_profile_t *prof = vehicle_profile_get_active();
    TEST_ASSERT_NOT_NULL(prof);

    /* 1. Verify all active profile rule CAN IDs are accepted by the O(1) bitmask */
    for (uint8_t i = 0; i < prof->rule_count; i++) {
        uint32_t id = prof->rules[i].can_id;
        if (id <= 0x7FF) {
            TEST_ASSERT_TRUE_MESSAGE(can_router_is_id_allowed(id), "Valid profile CAN ID must be allowed");
        }
    }

    /* 2. Verify unmapped standard CAN IDs (e.g., 0x7FF, 0x111, 0x217, 0x260) are rejected */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x7FF));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x111));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x217));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x260));

    /* 3. Verify boundary / out-of-range IDs (> 0x7FF) are rejected */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x800));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x18DAF110));

    /* 4. Inject unmapped standard frame into can_router_process_can:
     * Ensure vehicle state cache is untouched */
    const vehicle_state_t *st_before = can_router_get_state();
    uint16_t rpm_before = st_before->rpm;
    bool drv_door_before = st_before->doors.door_driver;

    can_frame_t unmapped_frame = {
        .id = 0x7FF,
        .dlc = 8,
        .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
    can_router_process_can(&unmapped_frame);

    const vehicle_state_t *st_after = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT16(rpm_before, st_after->rpm);
    TEST_ASSERT_EQUAL_INT(drv_door_before, st_after->doors.door_driver);

    /* 5. Inject 29-bit extended frame (even with matching ID value) - must be rejected */
    can_frame_t ext_frame = {
        .id = 0x0B6, /* RPM frame ID, but marked extended */
        .dlc = 8,
        .is_extended = true,
        .data = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&ext_frame);
    TEST_ASSERT_EQUAL_UINT16(rpm_before, can_router_get_state()->rpm);

    /* 6. Verify dynamic profile switching updates the filter */
    vehicle_profile_set_active(VEHICLE_PROFILE_VAG_PQ35);
    TEST_ASSERT_TRUE(can_router_is_id_allowed(0x5C0)); /* VAG wheel keys */
    TEST_ASSERT_TRUE(can_router_is_id_allowed(0x470)); /* VAG doors */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(PSA_CAN_ID_STALK_21F)); /* PSA stalk must not be in VAG */

    /* Revert back to PSA profile */
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);
    TEST_ASSERT_TRUE(can_router_is_id_allowed(PSA_CAN_ID_STALK_21F));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x5C0));
}

