#include "unity.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"

void setUp_raise(void);
void setUp_hiworld(void);
void setUp_bagoo(void);
void setUp_peugeot_407(void);

void setUp(void) {
    setUp_raise();
    setUp_hiworld();
    setUp_bagoo();
    setUp_peugeot_407();
}

void tearDown(void) {
}

// Raise protocol test declarations
void test_raise_serialize_valid_packet(void);
void test_raise_serialize_buffer_too_small(void);
void test_raise_parse_valid_frame(void);
void test_raise_parse_zero_length_payload(void);
void test_raise_reject_corrupted_checksum(void);
void test_raise_ignore_preceding_noise(void);
void test_raise_resync_on_invalid_length(void);
void test_raise_car_mapping_lookup(void);
void test_raise_parse_car_model_select_packet(void);

// Bagoo protocol test declarations
void test_bagoo_serialize_valid_packet(void);
void test_bagoo_serialize_buffer_too_small(void);
void test_bagoo_parse_valid_dump_frames(void);
void test_bagoo_reject_corrupted_checksum(void);
void test_bagoo_ignore_preceding_noise(void);

// Peugeot 407 SPEC_01 test declarations
void test_peugeot_407_stalk_buttons_press_and_release(void);
void test_peugeot_407_stalk_rotary_encoder(void);
void test_peugeot_407_verification_vector_1_vol_up(void);
void test_peugeot_407_verification_vector_2_climate(void);
void test_peugeot_407_hvac_defrost_and_recirc(void);
void test_peugeot_407_verification_vector_3_parking_radar(void);
void test_peugeot_407_front_parking_radar(void);
void test_peugeot_407_trip_computer_0x165_and_0x1a5(void);
void test_peugeot_407_reverse_state(void);
void test_peugeot_407_verification_vector_4_doors(void);
void test_peugeot_407_doors_all_open_with_handbrake(void);
void test_peugeot_407_steering_wheel_angle(void);
void test_peugeot_407_jbl_amplifier(void);
void test_peugeot_407_cd_changer_and_rds(void);

// Peugeot 407 SPEC_02 test declarations
void test_psa_extended_tpms_numeric(void);
void test_psa_extended_tpms_temp_and_alarms(void);
void test_psa_extended_tpms_discrete_alarms(void);
void test_psa_extended_start_stop(void);
void test_psa_extended_cruise_memory(void);
void test_psa_extended_decode_cruise_0x1a8(void);
void test_psa_extended_adas(void);
void test_psa_extended_decode_alerts_0x168(void);

// Hiworld protocol test declarations
void test_hiworld_serialize_valid_packet(void);
void test_hiworld_parse_valid_stream(void);
void test_hiworld_reject_bad_checksum(void);
void test_hiworld_resync_after_false_sync1(void);

// Driver manager tests
void test_hu_protocol_driver_switching(void) {
    TEST_ASSERT_TRUE(hu_protocol_set_active(HU_PROTOCOL_RAISE));
    TEST_ASSERT_EQUAL_STRING("Raise", hu_protocol_get_active()->name);

    TEST_ASSERT_TRUE(hu_protocol_set_active(HU_PROTOCOL_HIWORLD));
    TEST_ASSERT_EQUAL_STRING("Hiworld", hu_protocol_get_active()->name);

    TEST_ASSERT_TRUE(hu_protocol_set_active(HU_PROTOCOL_BAGOO));
    TEST_ASSERT_EQUAL_STRING("Bagoo", hu_protocol_get_active()->name);

    // Invalid protocol ID
    TEST_ASSERT_FALSE(hu_protocol_set_active(HU_PROTOCOL_COUNT));

    // Reset back to Raise for default
    TEST_ASSERT_TRUE(hu_protocol_set_active(HU_PROTOCOL_RAISE));
}

int main(void) {
    UNITY_BEGIN();

    // Protocol Driver Switching
    RUN_TEST(test_hu_protocol_driver_switching);

    // Raise Protocol Unit Tests
    RUN_TEST(test_raise_serialize_valid_packet);
    RUN_TEST(test_raise_serialize_buffer_too_small);
    RUN_TEST(test_raise_parse_valid_frame);
    RUN_TEST(test_raise_parse_zero_length_payload);
    RUN_TEST(test_raise_reject_corrupted_checksum);
    RUN_TEST(test_raise_ignore_preceding_noise);
    RUN_TEST(test_raise_resync_on_invalid_length);
    RUN_TEST(test_raise_car_mapping_lookup);
    RUN_TEST(test_raise_parse_car_model_select_packet);

    // Bagoo Protocol Unit Tests
    RUN_TEST(test_bagoo_serialize_valid_packet);
    RUN_TEST(test_bagoo_serialize_buffer_too_small);
    RUN_TEST(test_bagoo_parse_valid_dump_frames);
    RUN_TEST(test_bagoo_reject_corrupted_checksum);
    RUN_TEST(test_bagoo_ignore_preceding_noise);

    // Peugeot 407 SPEC_01 Unit Tests & Verification Vectors
    RUN_TEST(test_peugeot_407_stalk_buttons_press_and_release);
    RUN_TEST(test_peugeot_407_stalk_rotary_encoder);
    RUN_TEST(test_peugeot_407_verification_vector_1_vol_up);
    RUN_TEST(test_peugeot_407_verification_vector_2_climate);
    RUN_TEST(test_peugeot_407_hvac_defrost_and_recirc);
    RUN_TEST(test_peugeot_407_verification_vector_3_parking_radar);
    RUN_TEST(test_peugeot_407_front_parking_radar);
    RUN_TEST(test_peugeot_407_trip_computer_0x165_and_0x1a5);
    RUN_TEST(test_peugeot_407_reverse_state);
    RUN_TEST(test_peugeot_407_verification_vector_4_doors);
    RUN_TEST(test_peugeot_407_doors_all_open_with_handbrake);
    RUN_TEST(test_peugeot_407_steering_wheel_angle);
    RUN_TEST(test_peugeot_407_jbl_amplifier);
    RUN_TEST(test_peugeot_407_cd_changer_and_rds);

    // Peugeot 407 SPEC_02 Extended Unit Tests & Verification Vectors
    RUN_TEST(test_psa_extended_tpms_numeric);
    RUN_TEST(test_psa_extended_tpms_temp_and_alarms);
    RUN_TEST(test_psa_extended_tpms_discrete_alarms);
    RUN_TEST(test_psa_extended_start_stop);
    RUN_TEST(test_psa_extended_cruise_memory);
    RUN_TEST(test_psa_extended_decode_cruise_0x1a8);
    RUN_TEST(test_psa_extended_adas);
    RUN_TEST(test_psa_extended_decode_alerts_0x168);

    // Hiworld Protocol Unit Tests
    RUN_TEST(test_hiworld_serialize_valid_packet);
    RUN_TEST(test_hiworld_parse_valid_stream);
    RUN_TEST(test_hiworld_reject_bad_checksum);
    RUN_TEST(test_hiworld_resync_after_false_sync1);

    return UNITY_END();
}

