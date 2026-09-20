#include "unity.h"
#include "profiles/peugeot_407.h"
#include <string.h>

static psa_stalk_state_t s_stalk_state;
static uint8_t s_last_stalk_key_id = 0;
static uint8_t s_last_stalk_key_state = 0;
static int s_stalk_cb_count = 0;

static void test_stalk_key_callback(uint8_t key_id, uint8_t state) {
    s_last_stalk_key_id = key_id;
    s_last_stalk_key_state = state;
    s_stalk_cb_count++;
}

void setUp_peugeot_407(void) {
    psa_stalk_init(&s_stalk_state);
    s_last_stalk_key_id = 0;
    s_last_stalk_key_state = 0;
    s_stalk_cb_count = 0;
}

/* --------------------------------------------------------------------------
 * 1.1 Steering Column Stalk & Buttons Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_stalk_buttons_press_and_release(void) {
    // 1. Volume Up Press (Byte 0 = 0x08)
    uint8_t can_data[8] = { 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    TEST_ASSERT_EQUAL_INT(1, s_stalk_cb_count);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_UP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // Release
    can_data[0] = 0x00;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_UP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(0, s_last_stalk_key_state);

    // 2. Volume Down (Byte 0 = 0x04)
    can_data[0] = 0x04;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_VOL_DOWN, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 3. Next Track (Byte 0 = 0x02)
    can_data[0] = 0x02;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_NEXT, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 4. Prev Track (Byte 0 = 0x01)
    can_data[0] = 0x01;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_PREV, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 5. Source (Byte 0 = 0x40)
    can_data[0] = 0x40;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SRC, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 6. OK / Click (Byte 0 = 0x10)
    can_data[0] = 0x10;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_OK, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 7. Dark (Byte 0 = 0x80)
    can_data[0] = 0x80;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_DARK, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 8. ESC (Byte 0 = 0x20)
    can_data[0] = 0x20;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_ESC, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 9. Menu (Byte 1 = 0x40)
    can_data[0] = 0x00;
    can_data[1] = 0x40;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_MENU, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 10. Tel Answer (Byte 2 = 0x01)
    can_data[1] = 0x00;
    can_data[2] = 0x01;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_TEL_ANSWER, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);

    // 11. Tel Hangup (Byte 2 = 0x02)
    can_data[2] = 0x02;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_TEL_HANGUP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_stalk_key_state);
}

void test_peugeot_407_stalk_rotary_encoder(void) {
    uint8_t can_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);

    // Scroll Up (+1 step: 0x00 -> 0x01)
    can_data[1] = 0x01;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count); // pulse press (1) and release (0)
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SCROLL_UP, s_last_stalk_key_id);
    TEST_ASSERT_EQUAL_HEX8(0, s_last_stalk_key_state);

    // Scroll Down (-1 step: 0x01 -> 0x00)
    can_data[1] = 0x00;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SCROLL_DOWN, s_last_stalk_key_id);

    // Rollover Down (0x00 -> 0x0F is delta -1 with modulo 16)
    can_data[1] = 0x0F;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SCROLL_DOWN, s_last_stalk_key_id);

    // Rollover Up (0x0F -> 0x00 is delta +1 with modulo 16)
    can_data[1] = 0x00;
    s_stalk_cb_count = 0;
    psa_stalk_process_can_ex(&s_stalk_state, can_data, sizeof(can_data), test_stalk_key_callback);
    TEST_ASSERT_EQUAL_INT(2, s_stalk_cb_count);
    TEST_ASSERT_EQUAL_HEX8(PSA_STALK_KEY_SCROLL_UP, s_last_stalk_key_id);
}

void test_peugeot_407_verification_vector_1_vol_up(void) {
    // Vector 1: cansend vcan0 0F6#0800000000000000 -> Expected: 2E 02 02 14 01 E6
    // Checksum = ~(0x02 + 0x02 + 0x14 + 0x01) = ~0x19 = 0xE6
    uint8_t out[16];
    size_t len = build_raise_stalk_key(PSA_STALK_KEY_VOL_UP, 1, out, sizeof(out));

    const uint8_t expected[] = { 0x2E, 0x02, 0x02, 0x14, 0x01, 0xE6 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
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
    TEST_ASSERT_EQUAL_HEX8(0x2B, st.driver_temp_raw); /* 21.5 C */
    TEST_ASSERT_EQUAL_HEX8(0x2C, st.pass_temp_raw);   /* 22.0 C */
    TEST_ASSERT_TRUE(st.pass_wind_face);

    uint8_t out[16];
    size_t len = build_raise_hvac_packet(&st, out, sizeof(out));

    const uint8_t expected[] = { 0x2E, 0x21, 0x07, 0xC8, 0x45, 0x2B, 0x2C, 0x00, 0x00, 0x40, 0x33 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
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
    TEST_ASSERT_EQUAL_HEX8(0x2E, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x21, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x07, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0xA1, out[3]); // Power(0x80) | Recirc(0x20) | RearDefrost(0x01)
    TEST_ASSERT_EQUAL_HEX8(0x88, out[4]); // DriverWindUp(0x80) | FanSpeed(8)
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0x88, out[7]); // FrontMax(0x80) | ACMax(0x08)
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

    TEST_ASSERT_EQUAL_HEX8(0x03, rl);
    TEST_ASSERT_EQUAL_HEX8(0x03, rc);
    TEST_ASSERT_EQUAL_HEX8(0x03, rr);

    uint8_t out[16];
    size_t len = build_raise_rear_radar(rl, rc, rr, 0xFF, 0xFF, 0xFF, out, sizeof(out));

    const uint8_t expected[] = { 0x2E, 0x32, 0x07, 0x00, 0x03, 0x03, 0x03, 0xFF, 0xFF, 0xFF, 0xC0 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_front_parking_radar(void) {
    const uint8_t can_270[] = { 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t fl = 0xFF, fc = 0xFF, fr = 0xFF;
    psa_decode_aas_front_0x270(can_270, sizeof(can_270), &fl, &fc, &fr);

    TEST_ASSERT_EQUAL_HEX8(0x01, fl);
    TEST_ASSERT_EQUAL_HEX8(0x02, fc);
    TEST_ASSERT_EQUAL_HEX8(0x04, fr);

    uint8_t out[16];
    size_t len = build_raise_front_radar(fl, fc, fr, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(8, len);
    TEST_ASSERT_EQUAL_HEX8(0x2E, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x30, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[6]);
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
    TEST_ASSERT_EQUAL_HEX8(0x18, temp_raw);

    uint8_t out_fuel[16];
    size_t len_fuel = build_raise_instant_fuel(fuel, range, dest, out_fuel, sizeof(out_fuel));
    TEST_ASSERT_EQUAL_UINT32(10, len_fuel);
    TEST_ASSERT_EQUAL_HEX8(0x2E, out_fuel[0]);
    TEST_ASSERT_EQUAL_HEX8(0x33, out_fuel[1]);
    TEST_ASSERT_EQUAL_HEX8(0x06, out_fuel[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out_fuel[3]);
    TEST_ASSERT_EQUAL_HEX8(0x44, out_fuel[4]);

    uint8_t out_temp[8];
    size_t len_temp = build_raise_outside_temp(temp_raw, out_temp, sizeof(out_temp));
    TEST_ASSERT_EQUAL_UINT32(5, len_temp);
    TEST_ASSERT_EQUAL_HEX8(0x2E, out_temp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x36, out_temp[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out_temp[2]);
    TEST_ASSERT_EQUAL_HEX8(0x18, out_temp[3]);

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
    TEST_ASSERT_EQUAL_HEX8(0x2E, out_t1[0]);
    TEST_ASSERT_EQUAL_HEX8(0x34, out_t1[1]);
    TEST_ASSERT_EQUAL_HEX8(0x06, out_t1[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out_t1[3]);
    TEST_ASSERT_EQUAL_HEX8(0x3B, out_t1[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out_t1[5]);
    TEST_ASSERT_EQUAL_HEX8(0x52, out_t1[6]);
    TEST_ASSERT_EQUAL_HEX8(0x06, out_t1[7]);
    TEST_ASSERT_EQUAL_HEX8(0x06, out_t1[8]);
}

void test_peugeot_407_reverse_state(void) {
    uint8_t out[8];
    size_t len = build_raise_reverse_state(true, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(5, len);
    TEST_ASSERT_EQUAL_HEX8(0x2E, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x40, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x80, out[3]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((0x40 + 0x01 + 0x80) ^ 0xFF), out[4]);
}

/* --------------------------------------------------------------------------
 * 1.5 Doors & Body Status Tests
 * -------------------------------------------------------------------------- */
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
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
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
    TEST_ASSERT_EQUAL_HEX8(0xFC, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]);
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
    TEST_ASSERT_EQUAL_HEX8(0x2E, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x29, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x5E, out[3]); // Little-Endian low
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); // Little-Endian high

    // Negative angle: -12.5 deg left = -125 deci-deg (0xFF83)
    angle = -125;
    len = build_raise_steering_angle(angle, out);
    TEST_ASSERT_EQUAL_UINT32(6, len);
    TEST_ASSERT_EQUAL_HEX8(0x83, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[4]);
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
    TEST_ASSERT_EQUAL_HEX8(0x2E, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x56, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x09, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x07, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x07, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0x06, out[7]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x12, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0x12, out[10]);
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
    TEST_ASSERT_EQUAL_HEX8(0x01, cdc.play_flags);

    uint8_t out_cdc[16];
    size_t len_cdc = build_raise_cd_changer(&cdc, out_cdc, sizeof(out_cdc));
    TEST_ASSERT_EQUAL_UINT32(11, len_cdc);
    TEST_ASSERT_EQUAL_HEX8(0x2E, out_cdc[0]);
    TEST_ASSERT_EQUAL_HEX8(0x54, out_cdc[1]);
    TEST_ASSERT_EQUAL_HEX8(0x07, out_cdc[2]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out_cdc[3]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out_cdc[4]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, out_cdc[5]);
    TEST_ASSERT_EQUAL_HEX8(0x14, out_cdc[6]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out_cdc[7]);
    TEST_ASSERT_EQUAL_HEX8(0x2D, out_cdc[8]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out_cdc[9]);

    // RDS Radio Station (0x396): "BBC R1  "
    const uint8_t can_396[] = { 'B', 'B', 'C', ' ', 'R', '1', ' ', ' ' };
    char name[9];
    psa_decode_rds_name_0x396(can_396, sizeof(can_396), name);
    TEST_ASSERT_EQUAL_STRING("BBC R1  ", name);

    uint8_t out_rds[16];
    size_t len_rds = build_raise_rds_name("BBC R1", out_rds, sizeof(out_rds));
    TEST_ASSERT_EQUAL_UINT32(12, len_rds);
    TEST_ASSERT_EQUAL_HEX8(0x2E, out_rds[0]);
    TEST_ASSERT_EQUAL_HEX8(0x55, out_rds[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out_rds[2]);
    TEST_ASSERT_EQUAL_HEX8('B', out_rds[3]);
    TEST_ASSERT_EQUAL_HEX8('B', out_rds[4]);
    TEST_ASSERT_EQUAL_HEX8('C', out_rds[5]);
    TEST_ASSERT_EQUAL_HEX8(' ', out_rds[6]);
    TEST_ASSERT_EQUAL_HEX8('R', out_rds[7]);
    TEST_ASSERT_EQUAL_HEX8('1', out_rds[8]);
    TEST_ASSERT_EQUAL_HEX8(' ', out_rds[9]);
    TEST_ASSERT_EQUAL_HEX8(' ', out_rds[10]);
}
