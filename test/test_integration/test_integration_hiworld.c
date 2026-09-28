#include "test_integration_common.h"
#include "core/vehicle_profile.h"
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

    // 1. Test PSA 0x220 authoritative frame: Front Left (0x80)
    can_frame_t frame_220 = {
        .id = 0x220,
        .dlc = 2,
        .data = { 0x80, 0x00 }
    };
    can_router_process_can(&frame_220);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Expected real Hiworld 0x12 frame (10-byte payload): 5A A5 0A 12 00 04 84 00 00 00 00 00 00 03 A6
    const uint8_t expected_220[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xA6
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_220), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_220, rx_buf, sizeof(expected_220));

    // 2. Test PSA 0x220 frame: RL (0x20) + RR (0x10)
    can_frame_t frame_220_rl_rr = {
        .id = 0x220,
        .dlc = 2,
        .data = { 0x30, 0x00 }
    };
    can_router_process_can(&frame_220_rl_rr);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Checksum = (0x0A + 0x12 + 0x00 + 0x04 + 0x34 + 0x03 - 1) & 0xFF = 0x56
    const uint8_t expected_220_rl_rr[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x56
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_220_rl_rr), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_220_rl_rr, rx_buf, sizeof(expected_220_rl_rr));

    // 3. Verify that receiving non-door CAN ID 0x221 (BSI status) does NOT overwrite door state
    can_frame_t bsi_frame_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
    can_router_process_can(&bsi_frame_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len); // No bogus door status frame emitted
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

void test_integration_hiworld_tpms_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);

    // Drain any leftover UART bytes
    uint8_t drain[128];
    while (read_uart_output(drain, sizeof(drain)) > 0);

    // Inject CAN ID 0x361 from real vehicle log dump_2026-09-28_16-32-16.log:
    // DATA: 80 00 40 0F 00 1E 00 16
    // FL: 0.0 Bar, puncture (state 2)
    // FR: 1.5 Bar, low (state 1)
    // RR: 3.0 Bar, ok (state 0)
    // RL: 2.2 Bar, ok (state 0)
    can_frame_t frame_361 = {
        .id = 0x361,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x40, 0x0F, 0x00, 0x1E, 0x00, 0x16 }
    };
    can_router_process_can(&frame_361);

    uint8_t rx_buf[64];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expected UART Output:
    // 1. Numeric TPMS (Cmd 0x66): Mode(0x01), FL(0=0x00), FR(15=0x0F), RL(22=0x16), RR(30=0x1E), Unit(0x00), CS(0xAF)
    //    Wire: 5A A5 06 66 01 00 0F 16 1E 00 AF
    // 2. Discrete TPMS (Cmd 0x18): FL(2), FR(1), RL(0), RR(0), CS(0x1E)
    //    Wire: 5A A5 04 18 02 01 00 00 1E
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x06, 0x66, 0x01, 0x00, 0x0F, 0x16, 0x1E, 0x00, 0xAF,
        0x5A, 0xA5, 0x04, 0x18, 0x02, 0x01, 0x00, 0x00, 0x1E
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));

    // 1. Re-inject identical frame -> No UART TPMS frames should be sent
    can_router_process_can(&frame_361);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    // 2. Inject frame with pressure change ONLY (alarms unchanged):
    // Change RL pressure from 2.2 Bar (0x16) to 2.4 Bar (0x18). Alarms remain identical.
    frame_361.data[7] = 0x18;
    can_router_process_can(&frame_361);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Only Numeric TPMS (Cmd 0x66) should be emitted! Discrete 0x18 must NOT be emitted.
    // CS = (6 + 0x66 + 1 + 0 + 0x0F + 0x18 + 0x1E + 0 - 1) = 0xB1
    const uint8_t expected_numeric_only[] = {
        0x5A, 0xA5, 0x06, 0x66, 0x01, 0x00, 0x0F, 0x18, 0x1E, 0x00, 0xB1
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_numeric_only), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_numeric_only, rx_buf, sizeof(expected_numeric_only));

    // 3. Periodic refresh: 299 ticks do NOT emit TPMS
    for (int t = 0; t < 299; t++) {
        can_router_periodic_100ms();
        // Drain heartbeats (Cmd 0xFF)
        read_uart_output(rx_buf, sizeof(rx_buf));
    }

    // Tick 300 (30 seconds) MUST emit both Numeric (0x66) and Discrete (0x18)
    can_router_periodic_100ms();
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Expect: Numeric (0x66, 11 bytes) + Discrete (0x18, 9 bytes) + Heartbeat (0xFF, 6 bytes) = 26 bytes
    TEST_ASSERT_EQUAL_UINT32(26, rx_len);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_CMD_TPMS_NUMERIC, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_CMD_TPMS_DISCRETE, rx_buf[14]);
}

void test_integration_hiworld_trip_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    size_t rx_len = 0;

    // 1. Vector 1: PSA CAN 0x221 (Instantaneous Trip)
    // Instant fuel: 6.8 L/100km (68 = 0x0044), Range: 640 km (0x0280)
    can_frame_t frame_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x00, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00 }
    };
    can_router_process_can(&frame_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_221[] = { 0x5A, 0xA5, 0x04, 0x13, 0x00, 0x44, 0x02, 0x80, 0xDC };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_221), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_221, rx_buf, sizeof(expected_221));

    // 2. Vector 2: PSA CAN 0x2A1 (Trip 1 Historical)
    // Distance: 569 km (0x0239), Fuel: 7.3 L/100km (73 = 0x0049), Mean Speed: 37 km/h (0x25)
    can_frame_t frame_2a1 = {
        .id = 0x2A1,
        .dlc = 7,
        .data = { 0x25, 0x02, 0x39, 0x00, 0x49, 0x00, 0x25 }
    };
    can_router_process_can(&frame_2a1);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_2a1[] = { 0x5A, 0xA5, 0x06, 0x14, 0x00, 0x49, 0x00, 0x25, 0x02, 0x39, 0xC2 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_2a1), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_2a1, rx_buf, sizeof(expected_2a1));

    // 3. Vector 3: PSA CAN 0x261 (Trip 2 Historical)
    // Distance: 921 km (0x0399), Fuel: 7.9 L/100km (79 = 0x004F), Mean Speed: 35 km/h (0x23)
    can_frame_t frame_261 = {
        .id = 0x261,
        .dlc = 7,
        .data = { 0x23, 0x03, 0x99, 0x00, 0x4F, 0x00, 0x23 }
    };
    can_router_process_can(&frame_261);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_261[] = { 0x5A, 0xA5, 0x06, 0x15, 0x00, 0x4F, 0x00, 0x23, 0x03, 0x99, 0x28 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_261), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_261, rx_buf, sizeof(expected_261));

    // 4. Test frames from dump_2026-09-28_20-24-29.log:
    // CAN 0x221: DATA: 00 00 47 02 18 04 b0 -> Instant fuel: 7.1 L/100km (0x0047), Range: 536 km (0x0218)
    can_frame_t dump_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x00, 0x00, 0x47, 0x02, 0x18, 0x04, 0xB0 }
    };
    can_router_process_can(&dump_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_dump_221[] = { 0x5A, 0xA5, 0x04, 0x13, 0x00, 0x47, 0x02, 0x18, 0x77 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_dump_221), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_221, rx_buf, sizeof(expected_dump_221));

    // CAN 0x2A1: DATA: 47 02 39 00 49 00 47 -> Dist: 569 km (0x0239), Fuel: 7.3 (0x0049), Speed: 71 km/h (0x47)
    can_frame_t dump_2a1 = {
        .id = 0x2A1,
        .dlc = 7,
        .data = { 0x47, 0x02, 0x39, 0x00, 0x49, 0x00, 0x47 }
    };
    can_router_process_can(&dump_2a1);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_dump_2a1[] = { 0x5A, 0xA5, 0x06, 0x14, 0x00, 0x49, 0x00, 0x47, 0x02, 0x39, 0xE4 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_dump_2a1), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_2a1, rx_buf, sizeof(expected_dump_2a1));

    // CAN 0x261: DATA: 23 03 99 00 4f 00 23 -> Dist: 921 km (0x0399), Fuel: 7.9 (0x004F), Speed: 35 km/h (0x23)
    can_frame_t dump_261 = {
        .id = 0x261,
        .dlc = 7,
        .data = { 0x23, 0x03, 0x99, 0x00, 0x4F, 0x00, 0x23 }
    };
    can_router_process_can(&dump_261);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_dump_261[] = { 0x5A, 0xA5, 0x06, 0x15, 0x00, 0x4F, 0x00, 0x23, 0x03, 0x99, 0x28 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_dump_261), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_261, rx_buf, sizeof(expected_dump_261));

    // 5. Verify masked / uninitialized frames produce no output
    can_frame_t masked_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x80, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00 }
    };
    can_router_process_can(&masked_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    can_frame_t uninit_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00 }
    };
    can_router_process_can(&uninit_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);
}
