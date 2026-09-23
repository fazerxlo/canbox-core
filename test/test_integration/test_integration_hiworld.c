#include "test_integration_common.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"

void test_integration_hiworld_steering_wheel_volume_up_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    // Inject PSA CAN frame: Volume Up press (CAN ID 0x128)
    can_frame_t frame = {
        .id = 0x128,
        .dlc = 2,
        .data = { 0x01, 0x00 }
    };
    can_router_process_can(&frame);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Hiworld SWC Frame: Sync(0x5A, 0xA5), Len(0x02), Cmd(0x11), Key(0x01), Pressed(0x01), CS(0x14)
    const uint8_t expected[] = { 0x5A, 0xA5, 0x02, 0x11, 0x01, 0x01, 0x14 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));

    // Release button
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    can_router_process_can(&frame);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Release Frame: Sync(0x5A, 0xA5), Len(0x02), Cmd(0x11), Key(0x00), Pressed(0x00), CS(0x12)
    const uint8_t expected_release[] = { 0x5A, 0xA5, 0x02, 0x11, 0x00, 0x00, 0x12 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));
}

void test_integration_hiworld_door_status_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    can_frame_t frame = {
        .id = 0x036,
        .dlc = 4,
        .data = { 0x11, 0x00, 0x00, 0x00 } // Driver door (bit 0) + Trunk (bit 4)
    };
    can_router_process_can(&frame);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Hiworld Door Status Cmd 0x12:
    // Payload byte 0: driver door (bit 0 = 0x01) | trunk (bit 4 = 0x10) -> 0x11
    // Len = 1 (1 byte payload), CS = (0x01 + 0x12 + 0x11 - 1) = 0x23
    const uint8_t expected[] = { 0x5A, 0xA5, 0x01, 0x12, 0x11, 0x23 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));
}

void test_integration_hiworld_telemetry_periodic_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    // Steering angle CAN frame: 150 -> 15 deg (0x000F)
    can_frame_t e8_frame = {
        .id = 0x0E8,
        .dlc = 4,
        .data = { 0x00, 0x96, 0x00, 0x00 }
    };
    can_router_process_can(&e8_frame);

    // Trigger periodic 100ms update
    can_router_periodic_100ms();

    uint8_t rx_buf[64];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Hiworld telemetry dispatches:
    // 1. Steering Track Angle (Cmd 0x11, Len 0x02, Payload: angle_le16(0x0F, 0x00), CS = (0x02 + 0x11 + 0x0F + 0x00 - 1) = 0x21)
    // 2. Heartbeat (Cmd 0xFF, Len 0x01, Payload: 0x01, CS = (0x01 + 0xFF + 0x01 - 1) = 0x00)
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x02, 0x11, 0x0F, 0x00, 0x21,
        0x5A, 0xA5, 0x01, 0xFF, 0x01, 0x00
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));
}

void test_integration_hiworld_runtime_car_selection_and_handshake(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    // Feed downlink car model selection packet: 5A A5 02 24 22 00 47 (Peugeot 407)
    const uint8_t stream[] = { 0x5A, 0xA5, 0x02, 0x24, 0x22, 0x00, 0x47 };
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }

    uint8_t rx_buf[128];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expect response with Version (0xF0) + Feature Enable 1 (0x71) + Feature Enable 2 (0x72)
    TEST_ASSERT_TRUE(rx_len > 0);

    // Verify first response is 0xF0
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF1, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF2, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_CMD_VERSION_REPORT, rx_buf[3]);
}
