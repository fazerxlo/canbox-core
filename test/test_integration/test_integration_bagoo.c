#include "test_integration_common.h"
#include "protocols/proto_bagoo.h"

void test_integration_bagoo_steering_wheel_volume_up_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_BAGOO);

    // Inject PSA CAN frame: Volume Up press (CAN ID 0x128)
    can_frame_t frame = {
        .id = 0x128,
        .dlc = 2,
        .data = { 0x01, 0x00 }
    };
    can_router_process_can(&frame);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Bagoo Wheel Key (Cmd 0x01, Len 3, Payload: 0x00, 0x00, 0x08):
    // Wire: Sync(0xFD), TotalLen(6), Cmd(0x01), Payload(0x00, 0x00, 0x08), CS(0x0F)
    const uint8_t expected[] = { 0xFD, 0x06, 0x01, 0x00, 0x00, 0x08, 0x0F };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));

    // Release button
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    can_router_process_can(&frame);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Release Frame: Payload(0x00, 0x00, 0x00), CS(0x07)
    const uint8_t expected_release[] = { 0xFD, 0x06, 0x01, 0x00, 0x00, 0x00, 0x07 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));
}

void test_integration_bagoo_door_status_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_BAGOO);

    can_frame_t frame = {
        .id = 0x036,
        .dlc = 4,
        .data = { 0x11, 0x00, 0x00, 0x00 } // Driver door (bit 0) + Trunk (bit 4)
    };
    can_router_process_can(&frame);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Bagoo Door Status (Cmd 0x38, 8 bytes payload):
    // d0 = 0x88 (Driver door bit7: 0x80 | Trunk bit3: 0x08)
    // Payload: { 0x88, 0x88, 0x81, 0x01, 0x00, 0x00, 0x00, 0x68 }
    // TotalLen = 11 (0x0B), CS = 0x3D
    const uint8_t expected[] = {
        0xFD, 0x0B, 0x38, 0x88, 0x88, 0x81, 0x01, 0x00, 0x00, 0x00, 0x68, 0x3D
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));
}

void test_integration_bagoo_hu_uart_to_canbox_version_query(void) {
    hu_protocol_set_active(HU_PROTOCOL_BAGOO);

    // Head Unit sends Bagoo version query request:
    // Sync(0xFD), TotalLen(0x03), Cmd(0x7F), CS(3 + 0x7F = 0x82)
    const uint8_t req[] = { 0xFD, 0x03, 0x7F, 0x82 };

    for (size_t i = 0; i < sizeof(req); i++) {
        can_router_process_uart_byte(req[i]);
    }

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Firmware responds with "RZC-PSA":
    // Sync(0xFD), TotalLen(0x0A), Cmd(0x7F), Payload("RZC-PSA"), CS(0x89)
    const uint8_t expected_resp[] = {
        0xFD, 0x0A, 0x7F, 'R', 'Z', 'C', '-', 'P', 'S', 'A', 0x89
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_resp), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_resp, rx_buf, sizeof(expected_resp));
}

