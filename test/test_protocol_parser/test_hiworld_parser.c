#include "unity.h"
#include "protocols/proto_hiworld.h"
#include <string.h>

static hiworld_packet_t s_last_packet;
static int s_rx_call_count = 0;

static void test_hiworld_rx_callback(const hiworld_packet_t *packet) {
    s_rx_call_count++;
    s_last_packet = *packet;
}

void setUp_hiworld(void) {
    s_rx_call_count = 0;
    memset(&s_last_packet, 0, sizeof(s_last_packet));
    proto_hiworld_init(test_hiworld_rx_callback);
}

void test_hiworld_serialize_valid_packet(void) {
    const uint8_t payload[] = { 0x01, 0x01 };
    uint8_t out[16];

    // Payload len = 2 -> len field = 3 (cmd + 2 bytes)
    // Cmd = 0x11
    // CS = (3 + 0x11 + 0x01 + 0x01) & 0xFF = 0x16
    size_t written = proto_hiworld_serialize(0x11, payload, sizeof(payload), out, sizeof(out));

    TEST_ASSERT_EQUAL_UINT32(7, written);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SYNC_1, out[0]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SYNC_2, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x11, out[3]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &out[4], 2);
    TEST_ASSERT_EQUAL_HEX8(0x16, out[6]);
}

void test_hiworld_parse_valid_stream(void) {
    // 0x5A 0xA5, Len: 2 (Cmd + 1 byte), Cmd: 0x21, Payload: 0x05, CS: 2 + 0x21 + 0x05 = 0x28
    const uint8_t stream[] = { 0x5A, 0xA5, 0x02, 0x21, 0x05, 0x28 };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_hiworld_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x21, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_packet.payload_len);
    TEST_ASSERT_EQUAL_HEX8(0x05, s_last_packet.payload[0]);
}

void test_hiworld_reject_bad_checksum(void) {
    const uint8_t stream[] = { 0x5A, 0xA5, 0x02, 0x21, 0x05, 0xFF };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_hiworld_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(0, s_rx_call_count);
}

void test_hiworld_resync_after_false_sync1(void) {
    // Extra 0x5A byte before real header
    const uint8_t stream[] = { 0x5A, 0x5A, 0xA5, 0x01, 0xFF, 0x00 };
    // Len: 1, Cmd: 0xFF, Payload: 0 -> CS = (1 + 0xFF) & 0xFF = 0x00

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_hiworld_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0xFF, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0, s_last_packet.payload_len);
}
