#include "unity.h"
#include "protocols/proto_bagoo.h"
#include <string.h>

static bagoo_packet_t s_last_bagoo_packet;
static int s_bagoo_rx_call_count = 0;

static void test_bagoo_rx_callback(const bagoo_packet_t *packet) {
    s_bagoo_rx_call_count++;
    s_last_bagoo_packet = *packet;
}

void setUp_bagoo(void) {
    s_bagoo_rx_call_count = 0;
    memset(&s_last_bagoo_packet, 0, sizeof(s_last_bagoo_packet));
    proto_bagoo_init(test_bagoo_rx_callback);
}

// 1. Verify Bagoo packet serialization (Len + Cmd + Payload + CS)
void test_bagoo_serialize_valid_packet(void) {
    const uint8_t payload[] = { 0x00, 0x00, 0x08 };
    uint8_t out[16];

    // Cmd: 0x01, Len: 3, Payload: 0x00, 0x00, 0x08
    // Total len field = 3 + 3 = 6
    // Sum = 6 + 1 + 0 + 0 + 8 = 0x0F
    size_t written = proto_bagoo_serialize(0x01, payload, sizeof(payload), out, sizeof(out));

    TEST_ASSERT_EQUAL_UINT32(7, written);
    TEST_ASSERT_EQUAL_HEX8(BAGOO_SYNC_BYTE, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x06, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[2]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &out[3], 3);
    TEST_ASSERT_EQUAL_HEX8(0x0F, out[6]);
}

// 2. Verify buffer safety on serialize overflow
void test_bagoo_serialize_buffer_too_small(void) {
    const uint8_t payload[] = { 0xAA, 0xBB };
    uint8_t out[4]; // Requires at least 6 bytes (FD + Len + Cmd + 2 bytes + CS)

    size_t written = proto_bagoo_serialize(0x01, payload, sizeof(payload), out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(0, written);
}

// 3. Feed a valid byte stream from Peugeot 407 dump and check decoded output
void test_bagoo_parse_valid_dump_frames(void) {
    // Frame 1: FD 06 01 00 00 08 0F
    const uint8_t stream1[] = { 0xFD, 0x06, 0x01, 0x00, 0x00, 0x08, 0x0F };

    for (size_t i = 0; i < sizeof(stream1); i++) {
        proto_bagoo_feed_byte(stream1[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_bagoo_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_last_bagoo_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(3, s_last_bagoo_packet.len);
    TEST_ASSERT_EQUAL_HEX8(0x08, s_last_bagoo_packet.payload[2]);

    // Frame 2: FD 04 36 58 92 (Outside temp raw 0x58, CS = 4 + 0x36 + 0x58 = 0x92)
    const uint8_t stream2[] = { 0xFD, 0x04, 0x36, 0x58, 0x92 };
    for (size_t i = 0; i < sizeof(stream2); i++) {
        proto_bagoo_feed_byte(stream2[i]);
    }

    TEST_ASSERT_EQUAL_INT(2, s_bagoo_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x36, s_last_bagoo_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_bagoo_packet.len);
    TEST_ASSERT_EQUAL_HEX8(0x58, s_last_bagoo_packet.payload[0]);
}

// 4. Corrupted checksum rejection
void test_bagoo_reject_corrupted_checksum(void) {
    const uint8_t stream[] = { 0xFD, 0x06, 0x01, 0x00, 0x00, 0x08, 0xAA };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_bagoo_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(0, s_bagoo_rx_call_count);
}

// 5. Preceding garbage noise before a valid Bagoo frame
void test_bagoo_ignore_preceding_noise(void) {
    const uint8_t stream[] = {
        0x55, 0xAA, 0x00, 0xFF,
        0xFD, 0x05, 0x7D, 0x06, 0x04, 0x8C // Sum = 5 + 0x7D + 6 + 4 = 0x8C
    };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_bagoo_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_bagoo_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x7D, s_last_bagoo_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(2, s_last_bagoo_packet.len);
    TEST_ASSERT_EQUAL_HEX8(0x06, s_last_bagoo_packet.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x04, s_last_bagoo_packet.payload[1]);
}

