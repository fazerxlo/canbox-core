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
    vehicle_trip_t trip;
    memset(&trip, 0, sizeof(trip));

    // Vector 1: Instant Fuel 6.8 L/100km (68 = 0x0044), Range 640 km (0x0280)
    // CAN ID 0x221 injection: cansend vcan0 221#00004402800000
    const uint8_t can_221[] = { 0x00, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00 };
    psa_trip_process_can_0x221(&trip, can_221, sizeof(can_221));

    TEST_ASSERT_EQUAL_UINT16(68, trip.instant_fuel_deci);
    TEST_ASSERT_EQUAL_UINT16(640, trip.range_km);
    TEST_ASSERT_EQUAL_UINT16(0, trip.dest_dist_km);

    // Verify builder directly
    // Expected UART Output: 5A A5 0A 13 00 44 02 80 00 00 00 00 00 00 E2
    const uint8_t expected[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2 };
    uint8_t out[16];
    size_t len = build_hiworld_trip_instant(&trip, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_trip_hiworld_vector_target_mileage_dump(void) {
    vehicle_trip_t trip;
    memset(&trip, 0, sizeof(trip));

    // CAN ID 0x221 injection from dump_2026-09-29_19-37-10.log:
    // Fuel: 7.1 (0x0047), Range: 619 km (0x026B), Target mileage: 204 km (raw 2040 = 0x07F8)
    const uint8_t can_221[] = { 0x00, 0x00, 0x47, 0x02, 0x6B, 0x07, 0xF8 };
    psa_trip_process_can_0x221(&trip, can_221, sizeof(can_221));

    TEST_ASSERT_EQUAL_UINT16(71, trip.instant_fuel_deci);
    TEST_ASSERT_EQUAL_UINT16(619, trip.range_km);
    TEST_ASSERT_EQUAL_UINT16(2040, trip.dest_dist_km);

    const uint8_t expected[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x47, 0x02, 0x6B, 0x07, 0xF8, 0x00, 0x00, 0x00, 0x00, 0xCF };
    uint8_t out[16];
    size_t len = build_hiworld_trip_instant(&trip, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_trip_hiworld_vector_2_trip1_historical(void) {
    vehicle_trip_t trip;
    memset(&trip, 0, sizeof(trip));

    // Vector 2: Trip 1 Historical (Distance 569 km, Fuel 7.3 L/100km, Mean Speed 37 km/h)
    // CAN ID 0x2A1 injection: cansend vcan0 2A1#25023900490025
    const uint8_t can_2a1[] = { 0x25, 0x02, 0x39, 0x00, 0x49, 0x00, 0x25 };
    psa_trip_process_can_0x2a1(&trip, can_2a1, sizeof(can_2a1));

    TEST_ASSERT_EQUAL_UINT16(569, trip.trip1_distance_km);
    TEST_ASSERT_EQUAL_UINT16(73, trip.trip1_avg_fuel);
    TEST_ASSERT_EQUAL_UINT8(37, trip.trip1_avg_speed);

    // Expected UART Output: 5A A5 06 14 00 49 00 25 02 39 C2
    const uint8_t expected[] = { 0x5A, 0xA5, 0x06, 0x14, 0x00, 0x49, 0x00, 0x25, 0x02, 0x39, 0xC2 };
    uint8_t out[16];
    size_t len = build_hiworld_trip1(&trip, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_trip_hiworld_vector_3_trip2_historical(void) {
    vehicle_trip_t trip;
    memset(&trip, 0, sizeof(trip));

    // Vector 3: Trip 2 Historical (Distance 921 km, Fuel 7.9 L/100km, Mean Speed 35 km/h)
    // CAN ID 0x261 injection: cansend vcan0 261#230399004F0023
    const uint8_t can_261[] = { 0x23, 0x03, 0x99, 0x00, 0x4F, 0x00, 0x23 };
    psa_trip_process_can_0x261(&trip, can_261, sizeof(can_261));

    TEST_ASSERT_EQUAL_UINT16(921, trip.trip2_distance_km);
    TEST_ASSERT_EQUAL_UINT16(79, trip.trip2_avg_fuel);
    TEST_ASSERT_EQUAL_UINT8(35, trip.trip2_avg_speed);

    // Expected UART Output: 5A A5 06 15 00 4F 00 23 03 99 28
    const uint8_t expected[] = { 0x5A, 0xA5, 0x06, 0x15, 0x00, 0x4F, 0x00, 0x23, 0x03, 0x99, 0x28 };
    uint8_t out[16];
    size_t len = build_hiworld_trip2(&trip, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(11, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_peugeot_407_trip_hiworld_vector_4_fast_dynamics_0x0b6(void) {
    vehicle_state_t state;
    memset(&state, 0, sizeof(state));

    // Vector 4: Fast Dynamics RPM & Speed (800 RPM, 10 km/h)
    // CAN ID 0x0B6 injection: cansend vcan0 0B6#190003E8000000D0
    const uint8_t can_0b6[] = { 0x19, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0xD0 };
    psa_engine_process_can_0x0b6(&state, can_0b6, sizeof(can_0b6));

    TEST_ASSERT_EQUAL_UINT16(800, state.rpm);
    TEST_ASSERT_EQUAL_UINT16(10, state.speed_kmh);

    // Test invalid / engine off (0xFFFF)
    const uint8_t can_0b6_off[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xD0 };
    psa_engine_process_can_0x0b6(&state, can_0b6_off, sizeof(can_0b6_off));
    TEST_ASSERT_EQUAL_UINT16(0, state.rpm);
    TEST_ASSERT_EQUAL_UINT16(0, state.speed_kmh);
}

void test_peugeot_407_trip_hiworld_bsi_slow_data_0x0f6(void) {
    vehicle_state_t state;
    memset(&state, 0, sizeof(state));

    // 0x0F6: Reverse Active (Bit 7 of byte 7), Ambient Raw 128 (24 C)
    const uint8_t can_0f6[] = { 0x88, 130, 0x00, 0x00, 0x00, 128, 128, 0x80 };
    psa_bsi_process_can_0x0f6(&state, can_0f6, sizeof(can_0f6));

    TEST_ASSERT_EQUAL_UINT8(128, state.climate.outdoor_temp_raw);
    TEST_ASSERT_TRUE(state.reverse_gear);

    // Forward gear, Ambient 0 C (Raw 80)
    const uint8_t can_0f6_fwd[] = { 0x88, 125, 0x00, 0x00, 0x00, 80, 80, 0x00 };
    psa_bsi_process_can_0x0f6(&state, can_0f6_fwd, sizeof(can_0f6_fwd));
    TEST_ASSERT_EQUAL_UINT8(80, state.climate.outdoor_temp_raw);
    TEST_ASSERT_FALSE(state.reverse_gear);

    // Raw 0xFF should NOT overwrite existing outdoor_temp_raw
    const uint8_t can_0f6_inv[] = { 0x88, 120, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00 };
    psa_bsi_process_can_0x0f6(&state, can_0f6_inv, sizeof(can_0f6_inv));
    TEST_ASSERT_EQUAL_UINT8(80, state.climate.outdoor_temp_raw);

    // Sub-zero ambient temperature: Raw 40 -> (40 * 5 - 400)/10 = -20 C
    const uint8_t can_0f6_neg[] = { 0x88, 120, 0x00, 0x00, 0x00, 40, 40, 0x00 };
    psa_bsi_process_can_0x0f6(&state, can_0f6_neg, sizeof(can_0f6_neg));
    TEST_ASSERT_EQUAL_UINT8(40, state.climate.outdoor_temp_raw);
}

void test_peugeot_407_trip_hiworld_speed_fallback(void) {
    vehicle_trip_t trip;
    memset(&trip, 0, sizeof(trip));

    // DLC = 5 without bytes 5..6 (speed in byte 0 is 45 km/h)
    const uint8_t can_2a1_short[] = { 45, 0x01, 0x00, 0x00, 0x30 };
    psa_trip_process_can_0x2a1(&trip, can_2a1_short, sizeof(can_2a1_short));
    TEST_ASSERT_EQUAL_UINT8(45, trip.trip1_avg_speed);

    // DLC = 7 but bytes 5..6 are 0 (fallback to byte 0 which is 55 km/h)
    const uint8_t can_261_zero_speed[] = { 55, 0x01, 0x00, 0x00, 0x30, 0x00, 0x00 };
    psa_trip_process_can_0x261(&trip, can_261_zero_speed, sizeof(can_261_zero_speed));
    TEST_ASSERT_EQUAL_UINT8(55, trip.trip2_avg_speed);
}

void test_peugeot_407_trip_hiworld_boundary_and_null_safety(void) {
    // NULL pointer calls must not crash
    psa_engine_process_can_0x0b6(NULL, NULL, 0);
    psa_trip_process_can_0x221(NULL, NULL, 0);
    psa_trip_process_can_0x2a1(NULL, NULL, 0);
    psa_trip_process_can_0x261(NULL, NULL, 0);
    psa_bsi_process_can_0x0f6(NULL, NULL, 0);

    // Short DLC boundary checks
    vehicle_state_t state;
    memset(&state, 0, sizeof(state));
    vehicle_trip_t trip;
    memset(&trip, 0, sizeof(trip));

    uint8_t dummy[8] = { 0 };
    psa_engine_process_can_0x0b6(&state, dummy, 3);
    psa_trip_process_can_0x221(&trip, dummy, 6);
    psa_trip_process_can_0x2a1(&trip, dummy, 4);
    psa_trip_process_can_0x261(&trip, dummy, 4);
    psa_bsi_process_can_0x0f6(&state, dummy, 7);

    // Builder buffer size & NULL safety
    uint8_t out[16];
    vehicle_trip_t st;
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

static void __attribute__((unused)) test_doors_uart_tx(const uint8_t *buf, size_t len) {
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
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_fr, out, 15);

    // RL door only (0x20) -> b2 = 0x24 -> CS = (0x0A + 0x12 + 0x04 + 0x24 + 0x03 - 1) = 0x46
    const uint8_t can_rl[] = { 0x20, 0x00 };
    psa_decode_doors_0x220(can_rl, sizeof(can_rl), &doors);
    TEST_ASSERT_TRUE(doors.rear_left_door);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_rl[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x46
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_rl, out, 15);

    // RR door only (0x10) -> b2 = 0x14 -> CS = (0x0A + 0x12 + 0x04 + 0x14 + 0x03 - 1) = 0x36
    const uint8_t can_rr[] = { 0x10, 0x00 };
    psa_decode_doors_0x220(can_rr, sizeof(can_rr), &doors);
    TEST_ASSERT_TRUE(doors.rear_right_door);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_rr[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x36
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_rr, out, 15);

    // Trunk only (0x08) -> b2 = 0x0C -> CS = (0x0A + 0x12 + 0x04 + 0x0C + 0x03 - 1) = 0x2E
    const uint8_t can_trunk[] = { 0x08, 0x00 };
    psa_decode_doors_0x220(can_trunk, sizeof(can_trunk), &doors);
    TEST_ASSERT_TRUE(doors.trunk);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_trunk[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x2E
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_trunk, out, 15);

    // All closed (0x00) -> b2 = 0x04 -> CS = (0x0A + 0x12 + 0x04 + 0x04 + 0x03 - 1) = 0x26
    const uint8_t can_none[] = { 0x00, 0x00 };
    psa_decode_doors_0x220(can_none, sizeof(can_none), &doors);
    len = build_hiworld_doors(&doors, out, sizeof(out));
    const uint8_t exp_none[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x26
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_none, out, 15);
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
    vehicle_doors_t doors;
    bool handbrake = false;
    memset(&doors, 0, sizeof(doors));

    TEST_ASSERT_FALSE(doors.door_driver);
    TEST_ASSERT_FALSE(handbrake);

    // Process CAN 0x220: FL door open (0x80), handbrake active (0x01)
    const uint8_t can_open[] = { 0x80, 0x01 };
    psa_doors_process_can_0x220(&doors, &handbrake, can_open, sizeof(can_open));

    TEST_ASSERT_TRUE(doors.door_driver);
    TEST_ASSERT_TRUE(handbrake);

    const uint8_t expected_open[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xA6
    };
    uint8_t out[20];
    size_t len = build_hiworld_doors_state(&doors, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_open, out, sizeof(expected_open));

    // Process CAN 0x220: All closed
    const uint8_t can_closed[] = { 0x00, 0x00 };
    psa_doors_process_can_0x220(&doors, &handbrake, can_closed, sizeof(can_closed));

    TEST_ASSERT_FALSE(doors.door_driver);
    TEST_ASSERT_FALSE(handbrake);

    const uint8_t expected_closed[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x26
    };
    len = build_hiworld_doors_state(&doors, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_closed, out, sizeof(expected_closed));
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
    vehicle_doors_t vdoors;
    memset(&vdoors, 0, sizeof(vdoors));
    vdoors.door_driver = true;
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors_state(&vdoors, NULL, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors_state(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_doors_state(&vdoors, out, 14));

    size_t len = build_hiworld_doors_state(&vdoors, out, sizeof(out));
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xA6
    };
    TEST_ASSERT_EQUAL_UINT32(15, len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, 15);

    // psa_decode_doors_0x220 robustness
    psa_decode_doors_0x220(NULL, 2, &doors);
    psa_decode_doors_0x220(out, 0, &doors);
    psa_decode_doors_0x220(out, 2, NULL);

    // psa_doors_process_can_0x220 robustness
    bool handbrake = false;
    psa_doors_process_can_0x220(NULL, &handbrake, out, 2);
    psa_doors_process_can_0x220(&vdoors, NULL, out, 2);
    psa_doors_process_can_0x220(&vdoors, &handbrake, NULL, 2);
    psa_doors_process_can_0x220(&vdoors, &handbrake, out, 0);
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

