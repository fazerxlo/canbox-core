#include "unity.h"
#include "protocols/proto_raise.h"
#include <string.h>

static raise_packet_t s_last_packet;
static int s_rx_call_count = 0;

static void test_raise_rx_callback(const raise_packet_t *packet) {
    s_rx_call_count++;
    s_last_packet = *packet;
}

void setUp_raise(void) {
    s_rx_call_count = 0;
    memset(&s_last_packet, 0, sizeof(s_last_packet));
    proto_raise_init(test_raise_rx_callback);
}

// 1. Verify standard packet serialization and checksum calculation
void test_raise_serialize_valid_packet(void) {
    const uint8_t payload[] = { 0x01, 0x02, 0x03 };
    uint8_t out[16];

    // Cmd: 0x24, Len: 3, Payload: 0x01, 0x02, 0x03
    // Sum = 0x24 + 0x03 + 0x01 + 0x02 + 0x03 = 0x2D
    // Checksum = ~0x2D = 0xD2
    size_t written = proto_raise_serialize(0x24, payload, sizeof(payload), out, sizeof(out));

    TEST_ASSERT_EQUAL_UINT32(7, written);
    TEST_ASSERT_EQUAL_HEX8(RAISE_SYNC_BYTE, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x24, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[2]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &out[3], 3);
    TEST_ASSERT_EQUAL_HEX8(0xD2, out[6]);
}

// 2. Verify buffer safety on serialize overflow
void test_raise_serialize_buffer_too_small(void) {
    const uint8_t payload[] = { 0xAA, 0xBB };
    uint8_t out[4]; // Requires at least 6 bytes

    size_t written = proto_raise_serialize(0x01, payload, sizeof(payload), out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(0, written);
}

// 3. Feed a valid byte stream and check decoded output
void test_raise_parse_valid_frame(void) {
    // Sync(0x2E), Cmd(0x01), Len(0x02), Payload(0x05, 0x00)
    // Sum = 0x01 + 0x02 + 0x05 + 0x00 = 0x08 -> CS = ~0x08 = 0xF7
    const uint8_t stream[] = { 0x2E, 0x01, 0x02, 0x05, 0x00, 0xF7 };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_raise_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0x02, s_last_packet.len);
    TEST_ASSERT_EQUAL_HEX8(0x05, s_last_packet.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, s_last_packet.payload[1]);
}

// 4. Verify zero-length payload packet handling
void test_raise_parse_zero_length_payload(void) {
    // Cmd: 0x7F, Len: 0x00 -> Sum = 0x7F -> CS = ~0x7F = 0x80
    const uint8_t stream[] = { 0x2E, 0x7F, 0x00, 0x80 };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_raise_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x7F, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0x00, s_last_packet.len);
}

// 5. Corrupted checksum rejection
void test_raise_reject_corrupted_checksum(void) {
    // Checksum intentionally corrupted (0x00 instead of 0xF7)
    const uint8_t stream[] = { 0x2E, 0x01, 0x02, 0x05, 0x00, 0x00 };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_raise_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(0, s_rx_call_count);
}

// 6. Garbage noise leading into a valid frame
void test_raise_ignore_preceding_noise(void) {
    const uint8_t stream[] = {
        0xFF, 0x00, 0xAA, 0x55, 0x12, // Random bus noise
        0x2E, 0x01, 0x01, 0x42, 0xBB  // Valid: Sum = 0x01 + 0x01 + 0x42 = 0x44 -> CS = ~0x44 = 0xBB
    };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_raise_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0x42, s_last_packet.payload[0]);
}

// 7. Resync when fake sync occurs inside corrupted stream
void test_raise_resync_on_invalid_length(void) {
    const uint8_t stream[] = {
        0x2E, 0x01, 0xFF,             // 0xFF exceeds RAISE_MAX_PAYLOAD_LEN -> Drops frame
        0x2E, 0x02, 0x01, 0x10, 0xEC  // Valid: Sum = 0x02 + 0x01 + 0x10 = 0x13 -> CS = 0xEC
    };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_raise_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x02, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0x10, s_last_packet.payload[0]);
}

// 8. Verify Raise car mapping lookup
#include "protocols/raise_car_mapping.h"
void test_raise_car_mapping_lookup(void) {
    vehicle_profile_id_t profile;

    // Test PSA mapping
    TEST_ASSERT_TRUE(raise_car_mapping_get_profile(RAISE_BRAND_PSA, RAISE_MODEL_PSA_2004, &profile));
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_PSA_2004, profile);

    // Test VAG mapping
    TEST_ASSERT_TRUE(raise_car_mapping_get_profile(RAISE_BRAND_VAG, RAISE_MODEL_VAG_PQ35, &profile));
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_VAG_PQ35, profile);

    // Test Unknown brand / model
    TEST_ASSERT_FALSE(raise_car_mapping_get_profile(0xFF, 0x01, &profile));
    TEST_ASSERT_FALSE(raise_car_mapping_get_profile(RAISE_BRAND_PSA, 0xFF, &profile));

    // Test reverse mapping
    uint8_t brand = 0, model = 0;
    TEST_ASSERT_TRUE(raise_car_mapping_get_codes(VEHICLE_PROFILE_PSA_2004, &brand, &model));
    TEST_ASSERT_EQUAL_HEX8(RAISE_BRAND_PSA, brand);
    TEST_ASSERT_EQUAL_HEX8(RAISE_MODEL_PSA_2004, model);

    TEST_ASSERT_TRUE(raise_car_mapping_get_codes(VEHICLE_PROFILE_VAG_PQ35, &brand, &model));
    TEST_ASSERT_EQUAL_HEX8(RAISE_BRAND_VAG, brand);
    TEST_ASSERT_EQUAL_HEX8(RAISE_MODEL_VAG_PQ35, model);
}

// 9. Verify parsing of Car Model Select frame (0xCA)
void test_raise_parse_car_model_select_packet(void) {
    // Cmd: 0xCA, Len: 0x02, Payload: Brand 0x01, Model 0x01
    // Sum = 0xCA + 0x02 + 0x01 + 0x01 = 0xCE -> CS = ~0xCE = 0x31
    const uint8_t stream[] = { 0x2E, 0xCA, 0x02, 0x01, 0x01, 0x31 };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_raise_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(RAISE_CMD_CAR_MODEL_SELECT, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0x02, s_last_packet.len);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_last_packet.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_last_packet.payload[1]);
}
