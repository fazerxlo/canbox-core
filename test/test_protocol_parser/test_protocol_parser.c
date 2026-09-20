#include "unity.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"

void setUp_raise(void);
void setUp_hiworld(void);

void setUp(void) {
    setUp_raise();
    setUp_hiworld();
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

    // Invalid protocol ID
    TEST_ASSERT_FALSE(hu_protocol_set_active(HU_PROTOCOL_COUNT));
    // Unimplemented Bagoo
    TEST_ASSERT_FALSE(hu_protocol_set_active(HU_PROTOCOL_BAGOO));

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

    // Hiworld Protocol Unit Tests
    RUN_TEST(test_hiworld_serialize_valid_packet);
    RUN_TEST(test_hiworld_parse_valid_stream);
    RUN_TEST(test_hiworld_reject_bad_checksum);
    RUN_TEST(test_hiworld_resync_after_false_sync1);

    return UNITY_END();
}

