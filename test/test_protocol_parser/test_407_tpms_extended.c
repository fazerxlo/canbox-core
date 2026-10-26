#include "unity.h"
#include "profiles/peugeot_407.h"
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
    TEST_ASSERT_EQUAL_HEX8(expected_cs, out[9]);
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
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(sum ^ 0xFF), out[11]);

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
    TEST_ASSERT_EQUAL_HEX8(expected_cs, out[7]);
}

/* --------------------------------------------------------------------------
 * Hiworld Peugeot 407 TPMS Unit Tests & Verification Vectors
 * Based on CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md
 * -------------------------------------------------------------------------- */
void test_peugeot_407_tpms_hiworld_vector_1_nominal(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // Vector 1: CAN 0x3A1 FL=2.4, FR=2.4, RR=2.2, RL=2.2 Bar (48, 48, 44, 44)
    const uint8_t can_3a1_v1[] = { 48, 48, 44, 44 };
    psa_tpms_process_can_0x3a1(&tpms, can_3a1_v1, sizeof(can_3a1_v1));
    const uint8_t can_1e1_v1[] = { 0x00, 0x00, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_v1, sizeof(can_1e1_v1));

    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[3]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[2]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[3]);

    uint8_t out[16];
    // Numeric (Cmd 0x66): 5A A5 06 66 01 18 18 16 16 00 C8
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 0x18, 0x18, 0x16, 0x16, 0x00, 0xC8 };
    size_t len_num = build_hiworld_tpms_numeric(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len_num);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_numeric, out, sizeof(exp_numeric));

    // Discrete (Cmd 0x18): 5A A5 04 18 00 00 00 00 1B
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x00, 0x00, 0x00, 0x1B };
    size_t len_disc = build_hiworld_tpms_discrete(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(9, len_disc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_discrete, out, sizeof(exp_discrete));
}

void test_peugeot_407_tpms_hiworld_vector_2_asymmetric_low(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // Vector 2: FL=2.4, FR=2.3, RR=2.1, RL=1.2 -> 0x3A1: [48, 46, 42, 24], 0x1E1: RL Low (0x08)
    const uint8_t can_3a1_v2[] = { 48, 46, 42, 24 };
    psa_tpms_process_can_0x3a1(&tpms, can_3a1_v2, sizeof(can_3a1_v2));
    const uint8_t can_1e1_v2[] = { 0x00, 0x00, 0x00, 0x08 };
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_v2, sizeof(can_1e1_v2));

    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(23, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(12, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(21, tpms.pressure_bar_deci[3]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(1, tpms.alarm_state[2]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[3]);

    uint8_t out[16];
    // Numeric (Cmd 0x66): 5A A5 06 66 01 18 17 0C 15 00 BC
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 0x18, 0x17, 0x0C, 0x15, 0x00, 0xBC };
    size_t len_num = build_hiworld_tpms_numeric(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len_num);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_numeric, out, sizeof(exp_numeric));

    // Discrete (Cmd 0x18): 5A A5 04 18 00 00 01 00 1C (RL alarm = 0x01 LOW)
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x00, 0x01, 0x00, 0x1C };
    size_t len_disc = build_hiworld_tpms_discrete(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(9, len_disc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_discrete, out, sizeof(exp_discrete));
}

void test_peugeot_407_tpms_hiworld_vector_3_puncture(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // Vector 3: FL=2.4, FR=0.4 (Puncture), RR=2.4, RL=2.4 -> 0x3A1: [48, 8, 48, 48], 0x1E1: FR Puncture (0x10)
    const uint8_t can_3a1_v3[] = { 48, 8, 48, 48 };
    psa_tpms_process_can_0x3a1(&tpms, can_3a1_v3, sizeof(can_3a1_v3));
    const uint8_t can_1e1_v3[] = { 0x00, 0x10, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_v3, sizeof(can_1e1_v3));

    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(4, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[3]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(2, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[2]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[3]);

    uint8_t out[16];
    // Numeric (Cmd 0x66): 5A A5 06 66 01 18 04 18 18 00 B8
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 0x18, 0x04, 0x18, 0x18, 0x00, 0xB8 };
    size_t len_num = build_hiworld_tpms_numeric(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len_num);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_numeric, out, sizeof(exp_numeric));

    // Discrete (Cmd 0x18): 5A A5 04 18 00 02 00 00 1D (FR alarm = 0x02 PUNCTURE)
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x02, 0x00, 0x00, 0x1D };
    size_t len_disc = build_hiworld_tpms_discrete(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(9, len_disc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_discrete, out, sizeof(exp_discrete));
}

void test_peugeot_407_tpms_hiworld_0x3a1_pressures(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // CAN 0x3A1: 0.05 Bar resolution
    // FL: 48 (2.4 Bar), FR: 46 (2.3 Bar), RR: 42 (2.1 Bar), RL: 24 (1.2 Bar)
    const uint8_t can_3a1[] = { 48, 46, 42, 24, 0x00, 0x00, 0x00, 0x00 };
    psa_tpms_process_can_0x3a1(&tpms, can_3a1, sizeof(can_3a1));

    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(23, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(12, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(21, tpms.pressure_bar_deci[3]);

    uint8_t out[16];
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 24, 23, 12, 21, 0x00, 0xBC };
    size_t len = build_hiworld_tpms_numeric(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_numeric, out, sizeof(exp_numeric));

    // Test 0xFF sentinel (invalid / offline sensor): RL = 0xFF -> 0 deci-bar
    const uint8_t can_3a1_invalid[] = { 48, 48, 44, 0xFF, 0x00, 0x00, 0x00, 0x00 };
    psa_tpms_process_can_0x3a1(&tpms, can_3a1_invalid, sizeof(can_3a1_invalid));
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(0,  tpms.pressure_bar_deci[2]); // RL 0xFF -> 0
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[3]); // RR 44 -> 22
}

void test_peugeot_407_tpms_hiworld_0x1e1_status_enum(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // CAN 0x1E1 bits 5..3 ((raw >> 3) & 0x07):
    // Byte 0 (FL): 0 (OK)          -> (0 << 3) = 0x00
    // Byte 1 (FR): 1 (LOW)         -> (1 << 3) = 0x08
    // Byte 2 (RR): 2 (FLAT)        -> (2 << 3) = 0x10
    // Byte 3 (RL): 4 (BATTERY_LOW) -> (4 << 3) = 0x20
    const uint8_t can_1e1[] = { 0x00, 0x08, 0x10, 0x20 };
    psa_tpms_process_can_0x1e1(&tpms, can_1e1, sizeof(can_1e1));

    TEST_ASSERT_TRUE(tpms.valid);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[0]); // FL OK
    TEST_ASSERT_EQUAL_UINT8(1, tpms.alarm_state[1]); // FR LOW
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[2]); // RL FAULT (from Byte 3)
    TEST_ASSERT_EQUAL_UINT8(2, tpms.alarm_state[3]); // RR FLAT (from Byte 2)

    // In Basic Mode (no direct numeric frames yet), synthetic pressures are populated:
    // FL (OK): 24 (2.4 Bar)
    // FR (LOW): 17 (1.7 Bar)
    // RL (FAULT): 0 (0.0 Bar)
    // RR (FLAT): 8 (0.8 Bar)
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(17, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(8, tpms.pressure_bar_deci[3]);

    // Also verify enum 3 (0x18 = Sensor Missing) and Bit 7 (0x80 = Unmonitored) map to Fault (3)
    const uint8_t can_1e1_missing[] = { 0x18, 0x80, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_missing, sizeof(can_1e1_missing));
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[0]); // FL Missing -> 3
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[1]); // FR Unmonitored (0x80) -> 3
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[1]);

    // Also verify 0x40 (Bit 6 / enum 0x08 = RT4 Invalid/Default unmonitored) maps to Fault/Offline (3)
    const uint8_t can_1e1_unmonitored[] = { 0x40, 0x40, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_unmonitored, sizeof(can_1e1_unmonitored));
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[1]);

    // Re-apply first test vector to verify packet builders
    psa_tpms_process_can_0x1e1(&tpms, can_1e1, sizeof(can_1e1));

    // Discrete packet: Cmd 0x18, Len 0x04, FL=0, FR=1, RL=3, RR=2
    // Checksum = (0x04 + 0x18 + 0 + 1 + 3 + 2 - 1) = 0x21
    const uint8_t exp_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x00, 0x01, 0x03, 0x02, 0x21 };
    uint8_t out[16];
    size_t len = build_hiworld_tpms_discrete(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(9, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_discrete, out, sizeof(exp_discrete));

    // Numeric packet: Cmd 0x66, Len 0x06, Mode=1, FL=24, FR=17, RL=0, RR=8, Unit=0
    // Checksum = (0x06 + 0x66 + 1 + 24 + 17 + 0 + 8 + 0 - 1) & 0xFF = 0x9D
    const uint8_t exp_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 24, 17, 0, 8, 0x00, 0x9D };
    len = build_hiworld_tpms_numeric(&tpms, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_numeric, out, sizeof(exp_numeric));
}

void test_peugeot_407_tpms_standby_guard(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);
    TEST_ASSERT_FALSE(tpms.valid);

    // Initial standby/uninitialized frame before TPMS wakes up (Bit 7 set on all wheels): should be ignored
    const uint8_t standby_frame[] = { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, standby_frame, sizeof(standby_frame));
    TEST_ASSERT_FALSE(tpms.valid);

    const uint8_t standby_masked_frame[] = { 0x88, 0x90, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, standby_masked_frame, sizeof(standby_masked_frame));
    TEST_ASSERT_FALSE(tpms.valid);

    // Bench unmonitored frame (0x40 on road wheels, 0x80 on spare): should be ignored before TPMS wakes up
    const uint8_t bench_unmonitored_frame[] = { 0x40, 0x40, 0x40, 0x40, 0x80, 0x20, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, bench_unmonitored_frame, sizeof(bench_unmonitored_frame));
    TEST_ASSERT_FALSE(tpms.valid);

    // When normal frame arrives, TPMS activates
    const uint8_t normal_frame[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00 };
    psa_tpms_process_can_0x1e1(&tpms, normal_frame, sizeof(normal_frame));
    TEST_ASSERT_TRUE(tpms.valid);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[2]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[3]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[3]);

    // Once active, a 80 80 80 80 frame is processed as sensor offline/fault
    psa_tpms_process_can_0x1e1(&tpms, standby_frame, sizeof(standby_frame));
    TEST_ASSERT_TRUE(tpms.valid);
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[2]);
    TEST_ASSERT_EQUAL_UINT8(3, tpms.alarm_state[3]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.pressure_bar_deci[3]);
}

void test_peugeot_407_tpms_two_tier_hybrid(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // 1. Basic Mode: only 0x1E1 frames received (0x08 = 1 << 3 = Under-inflated / Low)
    const uint8_t can_1e1_low[] = { 0x08, 0x00, 0x00, 0x00 }; // FL Low
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_low, sizeof(can_1e1_low));
    TEST_ASSERT_TRUE(tpms.valid);
    TEST_ASSERT_EQUAL_UINT8(1, tpms.alarm_state[0]); // FL Low
    TEST_ASSERT_EQUAL_UINT8(17, tpms.pressure_bar_deci[0]); // FL 1.7 Bar synthetic
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[1]); // FR 2.4 Bar nominal
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[2]); // RL 2.2 Bar nominal
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[3]); // RR 2.2 Bar nominal

    // Verify Vector 4 serialization against Hiworld packet builders
    uint8_t out[32];
    uint32_t len = build_hiworld_tpms_numeric(&tpms, out, sizeof(out));
    const uint8_t exp_v4_numeric[] = { 0x5A, 0xA5, 0x06, 0x66, 0x01, 17, 24, 22, 22, 0x00, 0xC1 };
    TEST_ASSERT_EQUAL_UINT32(11, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_v4_numeric, out, sizeof(exp_v4_numeric));

    len = build_hiworld_tpms_discrete(&tpms, out, sizeof(out));
    const uint8_t exp_v4_discrete[] = { 0x5A, 0xA5, 0x04, 0x18, 0x01, 0x00, 0x00, 0x00, 0x1C };
    TEST_ASSERT_EQUAL_UINT32(9, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_v4_discrete, out, sizeof(exp_v4_discrete));

    // 2. Direct Numeric Frame arrives (0x3A1)
    // FL: 46 (2.3 Bar), FR: 48 (2.4 Bar), RR: 44 (2.2 Bar), RL: 44 (2.2 Bar)
    const uint8_t can_3a1[] = { 46, 48, 44, 44 };
    psa_tpms_process_can_0x3a1(&tpms, can_3a1, sizeof(can_3a1));
    TEST_ASSERT_EQUAL_UINT8(23, tpms.pressure_bar_deci[0]); // Real 2.3 Bar

    // 3. Subsequent 0x1E1 frames must NOT overwrite real pressure readings!
    // (0x10 = 2 << 3 = Punctured / Flat)
    const uint8_t can_1e1_punct[] = { 0x10, 0x00, 0x00, 0x00 }; // FL Puncture (alarm 2)
    psa_tpms_process_can_0x1e1(&tpms, can_1e1_punct, sizeof(can_1e1_punct));
    TEST_ASSERT_EQUAL_UINT8(2, tpms.alarm_state[0]); // Alarm state updated to puncture
    TEST_ASSERT_EQUAL_UINT8(23, tpms.pressure_bar_deci[0]); // Preserves direct reading 2.3 Bar (not overwritten by 8)
}

void test_peugeot_407_tpms_hiworld_pipeline_and_boundaries(void) {
    vehicle_tpms_t tpms;
    psa_tpms_init(&tpms);

    // Initial defaults check
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[0]);
    TEST_ASSERT_EQUAL_UINT8(24, tpms.pressure_bar_deci[1]);
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[2]);
    TEST_ASSERT_EQUAL_UINT8(22, tpms.pressure_bar_deci[3]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[0]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[1]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[2]);
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[3]);

    // Serializer boundary checks
    uint8_t out[16];
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_numeric(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_numeric(&tpms, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_numeric(&tpms, out, 10)); // < 11
    TEST_ASSERT_EQUAL_UINT32(11, build_hiworld_tpms_numeric(&tpms, out, sizeof(out)));

    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_discrete(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_discrete(&tpms, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_tpms_discrete(&tpms, out, 8)); // < 9
    TEST_ASSERT_EQUAL_UINT32(9, build_hiworld_tpms_discrete(&tpms, out, sizeof(out)));

    // CAN process boundary checks
    uint8_t dummy_can[8] = { 0 };
    psa_tpms_process_can_0x3a1(NULL, dummy_can, 4);
    psa_tpms_process_can_0x3a1(&tpms, NULL, 4);
    psa_tpms_process_can_0x3a1(&tpms, dummy_can, 3); // dlc < 4

    psa_tpms_process_can_0x1e1(NULL, dummy_can, 4);
    psa_tpms_process_can_0x1e1(&tpms, NULL, 4);
    psa_tpms_process_can_0x1e1(&tpms, dummy_can, 3); // dlc < 4
    TEST_ASSERT_EQUAL_UINT8(0, tpms.alarm_state[3]);

    psa_tpms_init(NULL);
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
    TEST_ASSERT_EQUAL_HEX8(expected_cs, out[8]);

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
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(sum ^ 0xFF), out[10]);
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
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(sum ^ 0xFF), out[9]);
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

    // 1. Trip 1 Reset (CAN ID 0x167, 0x8A = 0x80 | 0x0A on Byte 0, 0x06 on Byte 1)
    TEST_ASSERT_TRUE(build_psa_trip_reset_frame(1, &frame));
    TEST_ASSERT_EQUAL_HEX32(0x167, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x8A, frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x06, frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame.data[3]);

    // 2. Trip 2 Reset (CAN ID 0x167, 0x4C = 0x40 | 0x0C on Byte 0, 0x06 on Byte 1)
    TEST_ASSERT_TRUE(build_psa_trip_reset_frame(2, &frame));
    TEST_ASSERT_EQUAL_HEX32(0x167, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x4C, frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x06, frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame.data[3]);

    // 3. Boundary & Error checks
    TEST_ASSERT_FALSE(build_psa_trip_reset_frame(0, &frame));
    TEST_ASSERT_FALSE(build_psa_trip_reset_frame(3, &frame));
    TEST_ASSERT_FALSE(build_psa_trip_reset_frame(1, NULL));

    // 4. Send reset status check
    TEST_ASSERT_EQUAL(HAL_STATUS_OK, psa_trip_send_reset(1));
    TEST_ASSERT_EQUAL(HAL_STATUS_OK, psa_trip_send_reset(2));
    TEST_ASSERT_EQUAL(HAL_STATUS_ERROR, psa_trip_send_reset(0));
}

