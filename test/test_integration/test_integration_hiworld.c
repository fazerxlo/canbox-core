#include <string.h>
#include "test_integration_common.h"
#include "core/vehicle_profile.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"
#include "profiles/psa_virtual_cdc.h"
#include "hal/hal_gpio.h"

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

    // Hiworld SWC Frame: Sync(0x5A, 0xA5), Len(0x0A), Cmd(0x11), Payload[10], CS(0xC9)
    const uint8_t expected[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x01, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC9 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));

    // Release button
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    can_router_process_can(&frame);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Release Frame: Sync(0x5A, 0xA5), Len(0x0A), Cmd(0x11), Key(0x00), Pressed(0x00), CS(0xC7)
    const uint8_t expected_release[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC7 };
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

    uint8_t rx_buf[64];
    size_t rx_len;

    // Ticks 1 to 9 (100ms - 900ms) must NOT emit heartbeat (throttled to 1 Hz)
    for (int tick = 1; tick <= 9; tick++) {
        can_router_periodic_100ms();
        rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
        TEST_ASSERT_EQUAL_UINT32(0, rx_len);
    }

    // Tick 10 (1000ms / 1s) MUST emit the 1 Hz Hiworld periodic heartbeat (Cmd 0xFF)
    can_router_periodic_100ms();
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Hiworld periodic heartbeat (Cmd 0xFF, Len 0x01, Payload: 0x01, CS = (0x01 + 0xFF + 0x01 - 1) = 0x00)
    const uint8_t expected[] = {
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

    // Expect response with ACK (0xFF, payload 0x24) + Version (0xF0) + Feature Enables
    TEST_ASSERT_TRUE(rx_len >= 6);

    // Verify first response is ACK frame: 5A A5 01 FF 24 23
    const uint8_t expected_ack[] = { 0x5A, 0xA5, 0x01, 0xFF, 0x24, 0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    // Verify subsequent frame is Version Report (0xF0)
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF1, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF2, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_CMD_VERSION_REPORT, rx_buf[9]);

    // Feed downlink car model selection packet again (repeated handshake with unchanged model)
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // MUST ONLY emit ACK frame (6 bytes), completely suppressing the 70-byte configuration burst
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_ack), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));
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

    // 3. Periodic refresh: 299 ticks do NOT emit TPMS (while bus is active)
    for (int t = 0; t < 299; t++) {
        can_frame_t keepalive = { .id = 0x7FF, .dlc = 0 };
        can_router_process_can(&keepalive);
        can_router_periodic_100ms();
        // Drain heartbeats (Cmd 0xFF)
        read_uart_output(rx_buf, sizeof(rx_buf));
    }

    // Tick 300 (30 seconds) MUST emit both Numeric (0x66) and Discrete (0x18)
    can_frame_t keepalive = { .id = 0x7FF, .dlc = 0 };
    can_router_process_can(&keepalive);
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
    const uint8_t expected_221[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2 };
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
    // CAN 0x221: DATA: 00 00 47 02 18 04 b0 -> Instant fuel: 7.1 L/100km (0x0047), Range: 536 km (0x0218), Dest: 1200 raw (0x04B0)
    can_frame_t dump_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x00, 0x00, 0x47, 0x02, 0x18, 0x04, 0xB0 }
    };
    can_router_process_can(&dump_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_dump_221[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x47, 0x02, 0x18, 0x04, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x31 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_dump_221), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_221, rx_buf, sizeof(expected_dump_221));

    // 5. Test frames from user dump_2026-09-29_19-37-10.log:
    // CAN 0x221: DATA: 00 00 47 02 6b 07 f8 -> Instant fuel: 7.1 (0x0047), Range: 619 km (0x026B), Target mileage: 204 km (raw 2040 = 0x07F8)
    can_frame_t log_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x00, 0x00, 0x47, 0x02, 0x6B, 0x07, 0xF8 }
    };
    can_router_process_can(&log_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_log_221[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x47, 0x02, 0x6B, 0x07, 0xF8, 0x00, 0x00, 0x00, 0x00, 0xCF };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_log_221), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_log_221, rx_buf, sizeof(expected_log_221));

    const vehicle_state_t *st = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT16(71, st->trip.instant_fuel_deci);
    TEST_ASSERT_EQUAL_UINT16(619, st->trip.range_km);
    TEST_ASSERT_EQUAL_UINT16(2040, st->trip.dest_dist_km);

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

void test_integration_hiworld_downlink_trip_reset_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);

    hal_can_native_clear_sent_frame();
    can_frame_t sent_frame;

    // 1. Android sends Trip 1 Reset (fallback 1-indexed): 5A A5 02 1B 01 01 1E
    const uint8_t reset_trip1_cmd[] = { 0x5A, 0xA5, 0x02, 0x1B, 0x01, 0x01, 0x1E };
    for (size_t i = 0; i < sizeof(reset_trip1_cmd); i++) {
        can_router_process_uart_byte(reset_trip1_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x221, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent_frame.data[0]); // Bit 7: Trip 1 Reset
    for (int i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, sent_frame.data[i]);
    }

    // 2. Android sends Trip 1 Reset (2-byte format, page 2 = EcuInfoPage2): 5A A5 02 1B 02 01 1F
    hal_can_native_clear_sent_frame();
    const uint8_t reset_trip1_p2_cmd[] = { 0x5A, 0xA5, 0x02, 0x1B, 0x02, 0x01, 0x1F };
    for (size_t i = 0; i < sizeof(reset_trip1_p2_cmd); i++) {
        can_router_process_uart_byte(reset_trip1_p2_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x221, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent_frame.data[0]); // Bit 7: Trip 1 Reset
    for (int i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, sent_frame.data[i]);
    }

    // 3. Android sends Trip 1 Reset (4-byte format from bench log dump_2026-09-29_17-13-36.log): 5A A5 04 1B 02 02 01 FF 22
    hal_can_native_clear_sent_frame();
    const uint8_t reset_trip1_4byte_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x02, 0x02, 0x01, 0xFF, 0x22 };
    for (size_t i = 0; i < sizeof(reset_trip1_4byte_cmd); i++) {
        can_router_process_uart_byte(reset_trip1_4byte_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x221, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent_frame.data[0]); // Bit 7: Trip 1 Reset
    for (int i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, sent_frame.data[i]);
    }

    // 4. Android sends Trip 2 Reset (4-byte format from bench log dump_2026-09-29_17-17-52.log): 5A A5 04 1B 03 03 01 FF 24
    hal_can_native_clear_sent_frame();
    const uint8_t reset_trip2_4byte_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x03, 0x03, 0x01, 0xFF, 0x24 };
    for (size_t i = 0; i < sizeof(reset_trip2_4byte_cmd); i++) {
        can_router_process_uart_byte(reset_trip2_4byte_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x221, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x40, sent_frame.data[0]); // Bit 6: Trip 2 Reset
    for (int i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, sent_frame.data[i]);
    }

    // 5. Android sends Tab Navigation frames (dump_2026-09-29_17-22-10.log): MUST NOT trigger any CAN reset
    hal_can_native_clear_sent_frame();
    const uint8_t nav_p1_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x01, 0x00, 0x01, 0xFF, 0x1F };
    for (size_t i = 0; i < sizeof(nav_p1_cmd); i++) {
        can_router_process_uart_byte(nav_p1_cmd[i]);
    }
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&sent_frame));

    const uint8_t nav_p2_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x02, 0x00, 0x01, 0xFF, 0x20 };
    for (size_t i = 0; i < sizeof(nav_p2_cmd); i++) {
        can_router_process_uart_byte(nav_p2_cmd[i]);
    }
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&sent_frame));

    const uint8_t nav_p3_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x03, 0x00, 0x01, 0xFF, 0x21 };
    for (size_t i = 0; i < sizeof(nav_p3_cmd); i++) {
        can_router_process_uart_byte(nav_p3_cmd[i]);
    }
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&sent_frame));
}

void test_integration_hiworld_radar_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    // 1. Inject Vector 1: Native CAN 0x0E1 Obstacle
    // cansend vcan0 0E1#24403F04202600
    can_frame_t frame_0e1 = {
        .id = 0x0E1,
        .dlc = 7,
        .data = { 0x24, 0x40, 0x3F, 0x04, 0x20, 0x26, 0x00 }
    };
    can_router_process_can(&frame_0e1);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expected UART Output (Hiworld 0x41):
    // 5A A5 0C 41 00 01 01 01 00 01 01 01 01 00 3F 05 97
    const uint8_t expected_v1[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x3F, 0x05, 0x97
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_v1), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_v1, rx_buf, sizeof(expected_v1));

    // 2. Inject Vector 2: Quiescent / All Clear
    // cansend vcan0 0E1#24003FFCFCFC00
    can_frame_t frame_quiescent = {
        .id = 0x0E1,
        .dlc = 7,
        .data = { 0x24, 0x00, 0x3F, 0xFC, 0xFC, 0xFC, 0x00 }
    };
    can_router_process_can(&frame_quiescent);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Expected UART Output: 5A A5 0C 41 FF FF FF FF FF FF FF FF 01 00 3F 05 89
    const uint8_t expected_v2[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x3F, 0x05, 0x89
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_v2), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_v2, rx_buf, sizeof(expected_v2));
}

void test_integration_hiworld_reverse_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    // Initial state: Reverse off
    TEST_ASSERT_FALSE(can_router_get_state()->reverse_gear);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    // 1. Engage reverse via 0x036: byte 1 bit 7 = 1
    can_frame_t rev_on = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&rev_on);

    TEST_ASSERT_TRUE(can_router_get_state()->reverse_gear);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    // 2. Disengage reverse via 0x036
    can_frame_t rev_off = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&rev_off);

    TEST_ASSERT_FALSE(can_router_get_state()->reverse_gear);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    // 3. Engage reverse via 0x0F6: byte 7 bit 7 = 1
    can_frame_t f6_on = {
        .id = 0x0F6,
        .dlc = 8,
        .data = { 0x88, 0x5A, 0x00, 0x00, 0x00, 0x64, 0x64, 0x80 }
    };
    can_router_process_can(&f6_on);

    TEST_ASSERT_TRUE(can_router_get_state()->reverse_gear);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    // 4. Disengage reverse via 0x0F6
    can_frame_t f6_off = {
        .id = 0x0F6,
        .dlc = 8,
        .data = { 0x88, 0x5A, 0x00, 0x00, 0x00, 0x64, 0x64, 0x00 }
    };
    can_router_process_can(&f6_off);

    TEST_ASSERT_FALSE(can_router_get_state()->reverse_gear);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));
}

void test_integration_hiworld_reverse_quiescent_no_blinking_or_trajectory(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);

    // Drain any leftover UART bytes
    uint8_t drain[128];
    while (read_uart_output(drain, sizeof(drain)) > 0);

    // 1. Engage reverse via 0x036 (byte 1 bit 7 = 1)
    can_frame_t rev_frame = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&rev_frame);
    TEST_ASSERT_TRUE(can_router_get_state()->reverse_gear);
    while (read_uart_output(drain, sizeof(drain)) > 0);

    // 2. Inject CAN 0x128 with cluster lighting telemetry (data[0] = 0x91)
    // Must NOT trigger fake SWC key press or trajectory angle jump (Cmd 0x11 with 00 01)
    can_frame_t frame_128 = {
        .id = 0x128,
        .dlc = 8,
        .data = { 0x91, 0xE0, 0x00, 0x00, 0x00, 0x80, 0xB0, 0x01 }
    };
    can_router_process_can(&frame_128);

    TEST_ASSERT_EQUAL_INT(WHEEL_KEY_NONE, can_router_get_state()->wheel.active_key);
    TEST_ASSERT_EQUAL_UINT8(0, can_router_get_state()->wheel.press_state);

    uint8_t rx_buf[64];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    // 3. Inject native CAN2004 radar frame 0x0E1: All clear (FC FC FE)
    can_frame_t frame_0e1 = {
        .id = 0x0E1,
        .dlc = 7,
        .data = { 0x24, 0x40, 0x3F, 0xFC, 0xFC, 0xFE, 0x00 }
    };
    can_router_process_can(&frame_0e1);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_radar_clear[] = {
        0x5A, 0xA5, 0x0C, 0x41, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x3F, 0x05, 0x89
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_radar_clear), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_radar_clear, rx_buf, sizeof(expected_radar_clear));

    const vehicle_state_t *st = can_router_get_state();
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_left_outer);
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_left_center);
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_right_center);
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_right_outer);

    // 4. Inject CAN 0x260 (MSG_BSI_INF_PROFILS on CAN2004)
    // Must NOT be treated as parking radar and must NOT overwrite rear sensors with 0x00
    can_frame_t frame_260 = {
        .id = 0x260,
        .dlc = 8,
        .data = { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_260);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    st = can_router_get_state();
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_left_outer);
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_left_center);
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_right_center);
    TEST_ASSERT_EQUAL_HEX8(0xFF, st->radar.rear_right_outer);

    // 5. Re-inject 0x0E1: Delta detection prevents duplicate serial frames
    can_router_process_can(&frame_0e1);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);
}

void test_integration_hiworld_native_stalk_0x21f_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    size_t rx_len;

    // 1. Volume Up Press (21F: 0x08, counter: 0x01)
    can_frame_t frame_21f = {
        .id = 0x21F,
        .dlc = 3,
        .data = { 0x08, 0x01, 0x00 }
    };
    can_router_process_can(&frame_21f);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_vol_up[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x01, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC9
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_vol_up), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_vol_up, rx_buf, sizeof(expected_vol_up));

    // Release Vol Up
    frame_21f.data[0] = 0x00;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_release[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC7
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));

    // 2. Chorded MUTE Press (21F: 0x0C = Vol+ | Vol-)
    frame_21f.data[0] = 0x0C;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_mute[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x03, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xCB
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_mute), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_mute, rx_buf, sizeof(expected_mute));

    // Release MUTE
    frame_21f.data[0] = 0x00;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));

    // 3. Next Track / Seek Up (21F: 0x80)
    frame_21f.data[0] = 0x80;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_next[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x08, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xD0
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_next), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_next, rx_buf, sizeof(expected_next));

    // Release Seek Up
    frame_21f.data[0] = 0x00;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));

    // 4. Source Toggle (21F: 0x02)
    frame_21f.data[0] = 0x02;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_src[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x0B, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xD3
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_src), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_src, rx_buf, sizeof(expected_src));

    // Release Source
    frame_21f.data[0] = 0x00;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));

    // 5. Rotary Scroll Up (+1 step: counter 0x01 -> 0x02)
    // Emits press (0x12, 1) and immediate auto-release (0x00, 0)
    frame_21f.data[0] = 0x00;
    frame_21f.data[1] = 0x02;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_scroll_up_pulse[] = {
        // Press:
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x12, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xDA,
        // Release:
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC7
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_scroll_up_pulse), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_scroll_up_pulse, rx_buf, sizeof(expected_scroll_up_pulse));

    // 6. Rotary Scroll Down (-1 step: counter 0x02 -> 0x01)
    // Emits press (0x11, 1) and immediate auto-release (0x00, 0)
    frame_21f.data[1] = 0x01;
    can_router_process_can(&frame_21f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_scroll_down_pulse[] = {
        // Press:
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x11, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xD9,
        // Release:
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC7
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_scroll_down_pulse), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_scroll_down_pulse, rx_buf, sizeof(expected_scroll_down_pulse));
}

void test_integration_hiworld_stalk_tip_0x221_trip_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[32];
    size_t rx_len;

    // Stalk Tip Trip Button Press (CAN ID 0x221: Byte 0 Bit 3 = 0x08, 0xC8 nominal from vehicle dumps)
    // Verified in dump_2026-10-06_20-27-16.log and dump_2026-10-06_20-45-34.log
    can_frame_t frame_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0xC8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
    can_router_process_can(&frame_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_stalk_trip_press[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x14, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xDC
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_stalk_trip_press), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_stalk_trip_press, rx_buf, sizeof(expected_stalk_trip_press));

    // Release Stalk Tip Trip Button (0xC0 nominal from vehicle dumps)
    frame_221.data[0] = 0xC0;
    can_router_process_can(&frame_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_stalk_trip_release[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC7
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_stalk_trip_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_stalk_trip_release, rx_buf, sizeof(expected_stalk_trip_release));
}

static void check_3e5_pipeline(const uint8_t *can6, const uint8_t *expected_press) {
    uint8_t rx_buf[32];
    size_t rx_len;
    const uint8_t idle[6] = { 0 };
    const uint8_t expected_release[] = { 0x5A, 0xA5, 0x02, 0x21, 0x00, 0x00, 0x22 };
    can_frame_t f = { .id = 0x3E5, .dlc = 6 };

    memcpy(f.data, can6, 6);
    can_router_process_can(&f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(7, rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_press, rx_buf, 7);

    memcpy(f.data, idle, 6);
    can_router_process_can(&f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));
}

/* CAN frames from can_log_buttons*.log -> wire frames from the original OEM Hiworld Canbox */
void test_integration_hiworld_console_0x3e5_full_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    static const uint8_t audio[6] = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t trip[6]  = { 0x00, 0x40, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t clim[6]  = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t dark[6]  = { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00 };
    static const uint8_t menu[6]  = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t esc[6]   = { 0x00, 0x00, 0x10, 0x00, 0x00, 0x00 };
    static const uint8_t up[6]    = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x40 };
    static const uint8_t down[6]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };
    static const uint8_t right[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };
    static const uint8_t tel[6]   = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t left[6]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };

    static const uint8_t w_audio[] = { 0x5A, 0xA5, 0x02, 0x21, 0x31, 0x01, 0x54 };
    static const uint8_t w_trip[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x40, 0x01, 0x63 };
    static const uint8_t w_clim[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x28, 0x01, 0x4B };
    static const uint8_t w_dark[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x07, 0x01, 0x2A };
    static const uint8_t w_menu[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x2E, 0x01, 0x51 };
    static const uint8_t w_esc[]   = { 0x5A, 0xA5, 0x02, 0x21, 0x25, 0x01, 0x48 };
    static const uint8_t w_up[]    = { 0x5A, 0xA5, 0x02, 0x21, 0x17, 0x01, 0x3A };
    static const uint8_t w_down[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x18, 0x01, 0x3B };
    static const uint8_t w_tel[]   = { 0x5A, 0xA5, 0x02, 0x21, 0x05, 0x01, 0x28 };
    static const uint8_t w_left[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x19, 0x01, 0x3C };
    static const uint8_t w_right[] = { 0x5A, 0xA5, 0x02, 0x21, 0x1A, 0x01, 0x3D };

    check_3e5_pipeline(audio, w_audio);
    check_3e5_pipeline(trip, w_trip);
    check_3e5_pipeline(clim, w_clim);
    check_3e5_pipeline(dark, w_dark);
    check_3e5_pipeline(dark, w_dark); /* DARK pressed twice in the log */
    check_3e5_pipeline(menu, w_menu);
    check_3e5_pipeline(esc, w_esc);
    check_3e5_pipeline(up, w_up);
    check_3e5_pipeline(down, w_down);
    check_3e5_pipeline(tel, w_tel);
    check_3e5_pipeline(left, w_left);
    check_3e5_pipeline(right, w_right);
}

void test_integration_hiworld_console_0x3e5_ok_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    /* OK: can_log_buttons1.log frame 000040000000 */
    static const uint8_t ok[6] = { 0x00, 0x00, 0x40, 0x00, 0x00, 0x00 };
    static const uint8_t w_ok[] = { 0x5A, 0xA5, 0x02, 0x21, 0x24, 0x01, 0x47 };
    check_3e5_pipeline(ok, w_ok);
}

void test_integration_hiworld_steering_angle_trajectory_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    uint8_t rx_buf[64];
    size_t rx_len;

    /* Steering frames outside reverse gear must not emit trajectory data */
    can_frame_t sas = { .id = 0x10B, .dlc = 8, .data = { 0x00, 0xD2, 0, 0, 0, 0, 0, 0 } };
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* Engage reverse (0x0F6 Byte 7 bit 7): current angle (+21 deg Left) is pushed */
    can_frame_t rev = { .id = 0x0F6, .dlc = 8, .data = { 0x8E, 0x58, 0, 0, 0, 0, 0, 0x80 } };
    can_router_process_can(&rev);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_21l[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xEB, 0x5E, 0x22, 0xAB };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_21l), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_21l, rx_buf, sizeof(exp_21l));

    /* Centered */
    memset(sas.data, 0, sizeof(sas.data));
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_c[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x22, 0xC1 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_c), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_c, rx_buf, sizeof(exp_c));

    /* Unchanged angle: no retransmission (delta detection) */
    can_router_process_can(&sas);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));

    /* Left +60.0 deg */
    sas.data[0] = 0x02; sas.data[1] = 0x58;
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_60l[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC4, 0x5E, 0x22, 0x84 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_60l), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_60l, rx_buf, sizeof(exp_60l));

    /* Right -90.0 deg */
    sas.data[0] = 0xFC; sas.data[1] = 0x7C;
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_90r[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0x5E, 0x22, 0x1B };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_90r), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_90r, rx_buf, sizeof(exp_90r));

    /* Sensor error flag in Byte 3: frame discarded */
    sas.data[0] = 0x00; sas.data[1] = 0x00; sas.data[3] = 0x01;
    can_router_process_can(&sas);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));
}

void test_integration_hiworld_outside_temp_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[256];
    (void)read_uart_output(rx_buf, sizeof(rx_buf)); /* Drain pending output */

    /* Vehicle log: 0x0F6 byte 5 = 0x7C -> 0x7C * 0.5 - 40 = 22 C */
    can_frame_t bsi_0f6 = {
        .id = 0x0F6,
        .dlc = 8,
        .data = { 0x88, 0x64, 0xFE, 0xFE, 0xFF, 0x7C, 0x7C, 0x00 }
    };
    can_router_process_can(&bsi_0f6);

    TEST_ASSERT_EQUAL_HEX8(0x7C, can_router_get_state()->climate.outdoor_temp_raw);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found = false;
    for (size_t i = 0; i + 16 <= rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i + 1] == 0xA5 && rx_buf[i + 3] == 0x31) {
            TEST_ASSERT_EQUAL_HEX8(0x0C, rx_buf[i + 2]);
            TEST_ASSERT_EQUAL_HEX8(0x7C, rx_buf[i + 4 + 11]); /* Payload byte 11 */
            found = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "Hiworld 0x31 climate frame with outside temp not sent");

    /* 0xFF is invalid: previous value is retained */
    bsi_0f6.data[5] = 0xFF;
    can_router_process_can(&bsi_0f6);
    TEST_ASSERT_EQUAL_HEX8(0x7C, can_router_get_state()->climate.outdoor_temp_raw);
}

void test_integration_hiworld_rd4_volume_and_audio_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);
    can_router_rebuild_filter();

    uint8_t rx_buf[256];
    (void)read_uart_output(rx_buf, sizeof(rx_buf)); /* Drain pending UART */

    // 1. Inject Volume Frame 0x1A5: Vol=18 (0x12), Unmuted
    can_frame_t frame_vol = {
        .id = 0x1A5,
        .dlc = 1,
        .data = { 0x12 }
    };
    can_router_process_can(&frame_vol);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x56, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[4]); // Mode 1 (Volume active), unmuted
    TEST_ASSERT_EQUAL_UINT8(18, rx_buf[11]); // Master Volume

    // 2. Inject Sound Settings 0x1E5: Bass active (0xBF = 0x3F | 0x80), Treble=9, Bal=9, Fad=9, Loudness=ON, SpdVol=ON
    can_frame_t frame_audio = {
        .id = 0x1E5,
        .dlc = 7,
        .data = { 0x3F, 0x3F, 0xBF, 0x00, 0x3F, 0x47, 0x00 }
    };
    can_router_process_can(&frame_audio);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, rx_buf[4]); // Mode 2 (Bass active), unmuted
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[5]);  // Bass = 9
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[6]);  // Treble = 9
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[7]);  // Balance = 9
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[8]);  // Fader = 9
    TEST_ASSERT_EQUAL_HEX8(0x11, rx_buf[10]); // Loudness ON (0x10) | SpdVol ON (0x01)
    TEST_ASSERT_EQUAL_UINT8(18, rx_buf[11]); // Master volume retained

    // 3. Menu Dismissal: 0x1E5 arrives with all active mode bits = 0
    frame_audio.data[2] = 0x3F; // Bass active bit cleared
    can_router_process_can(&frame_audio);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[4]); // Mode 0 (Menu dismissal HUD dismiss)

    // 4. Repeated unchanged 0x1E5 frame: zero jitter / no UART traffic
    can_router_process_can(&frame_audio);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    // 5. Idle Menu Closed Suppression: 10 ticks of can_router_periodic_100ms() MUST NOT emit 0x56 when idle
    for (int i = 0; i < 10; i++) {
        can_router_periodic_100ms();
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool idle_audio_sync_sent = false;
    for (size_t i = 0; i + 13 <= rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i + 1] == 0xA5 && rx_buf[i + 2] == 0x08 && rx_buf[i + 3] == 0x56) {
            idle_audio_sync_sent = true;
            break;
        }
    }
    TEST_ASSERT_FALSE_MESSAGE(idle_audio_sync_sent, "Audio Cmd 0x56 was unexpectedly sent periodically while menu was closed!");

    // 6. Active Menu Refresh: while active_mode >= 2 (e.g. Bass Mode 2 or Balance Mode 4),
    // periodic timer MUST transmit Cmd 0x56 every 1000ms with the active mode to keep HU menu open!
    frame_audio.data[2] = 0xBF; // Re-enter Bass active mode (Mode 2)
    can_router_process_can(&frame_audio);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, rx_buf[4]); // Mode 2

    // Run 10 ticks (1000ms) of periodic timer while in active mode
    for (int i = 0; i < 9; i++) {
        can_router_periodic_100ms();
    }
    (void)read_uart_output(rx_buf, sizeof(rx_buf)); // Drain heartbeats

    // 10th tick must fire active menu refresh
    can_router_periodic_100ms();
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool active_refresh_sent = false;
    for (size_t i = 0; i + 13 <= rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i + 1] == 0xA5 && rx_buf[i + 2] == 0x08 && rx_buf[i + 3] == 0x56) {
            TEST_ASSERT_EQUAL_HEX8(0x02, rx_buf[i + 4]); // Must be Mode 2 (NOT 0!)
            TEST_ASSERT_EQUAL_UINT8(18, rx_buf[i + 11]); // Master Vol
            active_refresh_sent = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(active_refresh_sent, "Periodic timer failed to refresh active DSP menu after 1000ms!");
    TEST_ASSERT_EQUAL_UINT8(2, can_router_get_state()->audio.active_mode); // State must retain mode 2!

    // 7. Volume knob stopped / idle frame (0xEE = 0xE0 | 14 from vehicle log):
    // Must dismiss volume mode 1 -> 0 WITHOUT sending mute bit (D0 must be 0x00, NOT 0x80)
    can_frame_t frame_vol_turning = {
        .id = 0x1A5,
        .dlc = 1,
        .data = { 0x0F } // Vol 15, rotating
    };
    can_router_process_can(&frame_vol_turning);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[4]); // Mode 1, unmuted
    TEST_ASSERT_EQUAL_UINT8(15, rx_buf[11]);

    can_frame_t frame_vol_stopped = {
        .id = 0x1A5,
        .dlc = 1,
        .data = { 0xEE } // Vol 14, stopped / idle (0xE0 | 14)
    };
    can_router_process_can(&frame_vol_stopped);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[4]); // Dismissal: Mode 0, unmuted (0x00, NOT 0x80!)
    TEST_ASSERT_EQUAL_UINT8(14, rx_buf[11]); // Master Vol = 14

    // Repeated identical idle frame: zero UART traffic
    can_router_process_can(&frame_vol_stopped);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);
}

void test_integration_hiworld_rd4_radio_tuner_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Source Tuner (0x165)
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_165);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 2. Tuner Frequency (0x225): FM1, Preset 1, RDS Lock, 102.50 MHz
    can_frame_t frame_225 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x01, 0x90, 0x04, 0x1A }
    };
    can_router_process_can(&frame_225);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 3. Station Name (0x2A5): "RMF FM  "
    can_frame_t frame_2a5 = {
        .id = 0x2A5,
        .dlc = 8,
        .data = { 'R', 'M', 'F', ' ', 'F', 'M', ' ', ' ' }
    };
    can_router_process_can(&frame_2a5);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= 19);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[4]); /* FM1 -> 0x01 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[5]); /* 1025 & 0xFF (Little Endian for Hiworld) */
    TEST_ASSERT_EQUAL_HEX8(0x04, rx_buf[6]); /* 1025 >> 8 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x20, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[9]);
    TEST_ASSERT_EQUAL_STRING_LEN("RMF FM  ", (char *)&rx_buf[10], 8);
}

void test_integration_hiworld_rd4_radiotext_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Source Tuner ON via 0x165
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_165);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 2. Inject ISO-TP multi-frame stream for "Queen - Bohemian Rhapsody" (29 bytes with 4-byte prefix)
    const uint8_t can_ff[]  = { 0x10, 0x1D, 0x10, 0x00, 0x00, 0x00, 0x51, 0x75 };
    const uint8_t can_cf1[] = { 0x21, 0x65, 0x65, 0x6E, 0x20, 0x2D, 0x20, 0x42 };
    const uint8_t can_cf2[] = { 0x22, 0x6F, 0x68, 0x65, 0x6D, 0x69, 0x61, 0x6E };
    const uint8_t can_cf3[] = { 0x23, 0x20, 0x52, 0x68, 0x61, 0x70, 0x73, 0x6F };
    const uint8_t can_cf4[] = { 0x24, 0x64, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00 };

    can_frame_t frame = { .id = 0x0A4, .dlc = 8 };

    memcpy(frame.data, can_ff, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf1, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf2, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf3, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf4, 8);
    can_router_process_can(&frame);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(30, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(25, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x86, rx_buf[3]);
    TEST_ASSERT_EQUAL_STRING_LEN("Queen - Bohemian Rhapsody", (char *)&rx_buf[4], 25);
}

void test_integration_hiworld_rd4_cd_changer_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // CAN 0x3A6: Disc 2, Track 14, Total 20, 03:45, Random
    can_frame_t frame = {
        .id = 0x3A6,
        .dlc = 8,
        .data = { 0x00, 0x02, 0x0E, 0x14, 0x03, 0x2D, 0x01, 0x00 }
    };
    can_router_process_can(&frame);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(16, rx_len);

    /* 5A A5 0B 97 02 02 00 0E 00 03 2D 01 02 14 00 FA */
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x02, 0x02, 0x00, 0x0E, 0x00, 0x03, 0x2D, 0x01, 0x02, 0x14, 0x00, 0xFA
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, 16);
}

void test_integration_hiworld_rd4_cd_in_dash_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Audio Source switch to CD (CAN 0x165 -> 0x20 = CD)
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x20, 0x00 }
    };
    can_router_process_can(&frame_165);

    // Emits Cmd 0x84 (Audio Source = 0x20 CD)
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= 19);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x20, rx_buf[4]); /* CD Source */

    // 2. CD Disc Info (CAN 0x365: Total Tracks = 18 = 0x12)
    can_frame_t frame_365 = {
        .id = 0x365,
        .dlc = 5,
        .data = { 0x12, 0x26, 0x02, 0x00, 0x00 }
    };
    can_router_process_can(&frame_365);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 3. CD Play Telemetry (CAN 0x3A5: Track 5, Playing, 02 min 35 sec)
    can_frame_t frame_3a5 = {
        .id = 0x3A5,
        .dlc = 6,
        .data = { 0x05, 0x03, 0x19, 0x02, 0x23, 0x00 }
    };
    can_router_process_can(&frame_3a5);

    // Emits Cmd 0x97 (Single in-dash CD: 5A A5 0B 97 00 00 00 05 00 02 23 00 02 12 00 DF)
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(16, rx_len);

    const uint8_t expected_cd_ex1[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x05, 0x00, 0x02, 0x23, 0x00, 0x02, 0x12, 0x00, 0xDF
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_cd_ex1, rx_buf, 16);

    // 4. Switch back to Radio (CAN 0x165 -> 0x10 = FM1):
    // MUST send Cmd 0x97 stopping CD playback first, then Cmd 0x84 for radio tuner
    can_frame_t frame_radio = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_radio);

    /* Allow 800ms physical button debounce to settle */
    hal_delay_ms(850);
    can_router_periodic_100ms();

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Buffer contains Cmd 0x97 (16 bytes) followed by Cmd 0x84 (19 bytes) = 35 bytes */
    TEST_ASSERT_TRUE(rx_len >= 35);

    // Verify first packet is Cmd 0x97 with play_status = 0x01 (Stop/Pause)
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x97, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[12]); /* play_status = 0x01 (Pause/Stop) */

    // Verify second packet is Cmd 0x84 with Tuner mode
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[16]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[17]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[18]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[19]);

    // Inject uninitialized sentinels while on Radio (must produce zero UART output)
    can_frame_t frame_365_sentinel = {
        .id = 0x365,
        .dlc = 5,
        .data = { 0xFF, 0xFF, 0xFF, 0x00, 0x00 }
    };
    can_router_process_can(&frame_365_sentinel);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    can_frame_t frame_3a5_sentinel = {
        .id = 0x3A5,
        .dlc = 6,
        .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00 }
    };
    can_router_process_can(&frame_3a5_sentinel);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);
}

void test_integration_hiworld_rd4_downlink_resume_queries(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Send Tuner Resume Query: forwardType(0x0F) -> 5A A5 00 0F 0E
    const uint8_t q_tuner[] = { 0x5A, 0xA5, 0x00, 0x0F, 0x0E };
    for (size_t i = 0; i < sizeof(q_tuner); i++) {
        can_router_process_uart_byte(q_tuner[i]);
    }

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= (6 + 19));
    /* Universal ACK frame: 5A A5 01 FF 0F 0E */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, rx_buf[4]);
    /* Radio State response: 5A A5 0E 84 ... */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[9]);

    // 2. Send Media Resume Query: forwardType(0x11) -> 5A A5 00 11 10
    const uint8_t q_media[] = { 0x5A, 0xA5, 0x00, 0x11, 0x10 };
    for (size_t i = 0; i < sizeof(q_media); i++) {
        can_router_process_uart_byte(q_media[i]);
    }

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(6 + 16, rx_len);
    /* Universal ACK frame: 5A A5 01 FF 11 10 */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x11, rx_buf[4]);
    /* Media State response: 5A A5 0B 97 ... */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x97, rx_buf[9]);

    // 3. Send Tuner UI Open Query: Cmd 0xA1 -> 5A A5 03 A1 00 01 FF A3
    const uint8_t q_tuner_a1[] = { 0x5A, 0xA5, 0x03, 0xA1, 0x00, 0x01, 0xFF, 0xA3 };
    for (size_t i = 0; i < sizeof(q_tuner_a1); i++) {
        can_router_process_uart_byte(q_tuner_a1[i]);
    }

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= (6 + 19));
    /* Universal ACK frame: 5A A5 01 FF A1 A0 */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0xA1, rx_buf[4]);
    /* Radio State response: 5A A5 0E 84 ... */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[9]);
}

void test_integration_hiworld_rd4_preset_list_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 0. Source Tuner ON via 0x165
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_165);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Inject CAN 0x225 with Preset 1 active (87.5 MHz)
    can_frame_t frame_225_p1 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x10, 0x10, 0x02, 0xEE }
    };
    can_router_process_can(&frame_225_p1);

    // Should output Cmd 0x84 (Radio State) followed by Cmd 0x85 (Preset list freqs - 18 bytes)
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= (19 + 18));

    // Find Cmd 0x85 in the stream
    bool found_0x85 = false;
    for (size_t i = 0; i < rx_len - 4; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == 0x85) {
            found_0x85 = true;
            TEST_ASSERT_EQUAL_HEX8(0x0D, rx_buf[i+2]); /* Length 13 */
            TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[i+4]); /* Band FM1 */
            // Preset 1 frequency: 875 -> 0x036B
            TEST_ASSERT_EQUAL_HEX8(0x03, rx_buf[i+5]);
            TEST_ASSERT_EQUAL_HEX8(0x6B, rx_buf[i+6]);
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x85);

    // 2. Inject native CAN 0x125 ISO-TP frames broadcasting station list
    can_frame_t frame_125_ff  = { .id = 0x125, .dlc = 8, .data = { 0x10, 0x28, 0x10, 0x04, 0x00, 0x40, 0x4A, 0x45 } };
    can_frame_t frame_125_cf1 = { .id = 0x125, .dlc = 8, .data = { 0x21, 0x44, 0x59, 0x4E, 0x4B, 0x41, 0x20, 0xB0 } };
    can_frame_t frame_125_cf2 = { .id = 0x125, .dlc = 8, .data = { 0x22, 0x20, 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D } };
    can_frame_t frame_125_cf3 = { .id = 0x125, .dlc = 8, .data = { 0x23, 0x20, 0xB0, 0x52, 0x4D, 0x46, 0x20, 0x4D } };
    can_frame_t frame_125_cf4 = { .id = 0x125, .dlc = 8, .data = { 0x24, 0x41, 0x58, 0x58, 0xB0, 0x20, 0x54, 0x52 } };
    can_frame_t frame_125_cf5 = { .id = 0x125, .dlc = 7, .data = { 0x25, 0x4F, 0x4A, 0x4B, 0x41, 0x20, 0xB0 } };

    can_router_process_can(&frame_125_ff);
    can_router_process_can(&frame_125_cf1);
    can_router_process_can(&frame_125_cf2);
    can_router_process_can(&frame_125_cf3);
    can_router_process_can(&frame_125_cf4);
    can_router_process_can(&frame_125_cf5);

    // Should output Cmd 0x85 (Extended Station Names Mode - 54 bytes)
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= 54);

    found_0x85 = false;
    for (size_t i = 0; i < rx_len - 4; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == 0x85) {
            found_0x85 = true;
            TEST_ASSERT_EQUAL_HEX8(0x31, rx_buf[i+2]); /* Length 49 */
            TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[i+4]); /* Band FM1 */
            TEST_ASSERT_EQUAL_STRING_LEN("JEDYNKA ", (char *)&rx_buf[i+5], 8);
            TEST_ASSERT_EQUAL_STRING_LEN(" RMF FM ", (char *)&rx_buf[i+13], 8);
            TEST_ASSERT_EQUAL_STRING_LEN("RMF MAXX", (char *)&rx_buf[i+21], 8);
            TEST_ASSERT_EQUAL_STRING_LEN(" TROJKA ", (char *)&rx_buf[i+29], 8);
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x85);
}

void test_integration_hiworld_rd4_aux_mode_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    /* 1. Inject CAN 0x165 selecting AUX Input 1 (src = 0x4 -> source_mode 0x40) */
    can_frame_t frame_aux = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_aux);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Must output Cmd 0x84 (CarRadioState) indicating AUX 1 (0x40) */
    TEST_ASSERT_EQUAL_UINT32(19, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x40, rx_buf[4]); /* Source = AUX 1 (0x40) */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[5]); /* Frequency LSB = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[6]); /* Frequency MSB = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[7]); /* Preset slot = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[8]); /* Indicators = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[9]); /* Power status = 1 (Playing) */
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&rx_buf[10], 8); /* Blank station name */

    /* Verify NO Cmd 0x85 (presets) was transmitted */
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x85) {
            TEST_FAIL_MESSAGE("Cmd 0x85 should NOT be sent when audio source is AUX");
        }
    }

    /* 2. Background CAN tuner frame 0x225 and station name 0x2A5 arrive from vehicle */
    can_frame_t frame_225 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x60, 0x90, 0x03, 0x14 } /* Preset 6, FM1, 89.4 MHz */
    };
    can_router_process_can(&frame_225);

    can_frame_t frame_2a5 = {
        .id = 0x2A5,
        .dlc = 8,
        .data = { 'J', 'E', 'D', 'Y', 'N', 'K', 'A', ' ' }
    };
    can_router_process_can(&frame_2a5);

    /* Must produce ZERO UART output: background tuner changes while in AUX must not spam serial or trigger Radio UI */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 3. Simulate periodic 60s resync ticks (600 ticks) */
    for (int t = 0; t < 599; t++) {
        if (t % 10 == 0) {
            can_router_process_can(&frame_aux);
        }
        can_router_periodic_100ms();
    }
    /* Drain heartbeats accumulated during the 599 ticks */
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 600th tick triggers the 60s periodic bulk resync */
    can_router_process_can(&frame_aux);
    can_router_periodic_100ms();

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Resync in AUX mode must emit Cmd 0x84 (AUX 1), NEVER Cmd 0x85 (presets) or Cmd 0x86 (radiotext) */
    bool found_0x84 = false;
    bool found_0x85 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            if (rx_buf[i + 3] == 0x84) found_0x84 = true;
            if (rx_buf[i + 3] == 0x85) found_0x85 = true;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);
    TEST_ASSERT_FALSE(found_0x85);

    /* 4. Transition back to Radio (CAN 0x165 -> Tuner 0x10 = FM1) */
    can_frame_t frame_radio = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_radio);

    /* Allow 800ms physical button debounce to settle */
    hal_delay_ms(850);
    can_router_periodic_100ms();

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Transition to radio MUST output Cmd 0x84 (Radio State) followed by Cmd 0x85 (Preset list) */
    found_0x84 = false;
    found_0x85 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            if (rx_buf[i + 3] == 0x84) found_0x84 = true;
            if (rx_buf[i + 3] == 0x85) found_0x85 = true;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);
    TEST_ASSERT_TRUE(found_0x85);
}

void test_integration_hiworld_rd4_radio_off_suppresses_cd_tuner_frames(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 0. Boot sequence from dump_2026-10-15_18-37-15.log:
     * Background 0x2A5 and 0x225 arrive BEFORE the first 0x165 OFF frame.
     * They must be cached silently without emitting any UART frames. */
    can_frame_t frame_2a5_boot = {
        .id = 0x2A5,
        .dlc = 8,
        .data = { 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D, 0x20, 0x20 }
    };
    can_router_process_can(&frame_2a5_boot);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));

    /* First 0x165 arrives with RD4 OFF (0x48, 0xC8, 0x00, 0x00):
     * Emits one-shot Cmd 0x84 with source_mode = 0xFF, power_status = 0x00 */
    can_frame_t frame_off = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x48, 0xC8, 0x00, 0x00 }
    };
    can_router_process_can(&frame_off);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_radio_off[] = {
        0x5A, 0xA5, 0x0E, 0x84, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 0x90
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_radio_off), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_radio_off, rx_buf, sizeof(expected_radio_off));

    /* Subsequent 0x165 OFF frames must NOT re-emit 0x84 */
    can_router_process_can(&frame_off);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));

    /* 1. Power ON RD4 radio in Tuner mode (CAN 0x165: 0xCC, 0x54, 0x10, 0x02) */
    can_frame_t frame_on = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_on);

    can_frame_t frame_225 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x01, 0x90, 0x04, 0x1A }
    };
    can_router_process_can(&frame_225);

    /* Verify 0x84 received while radio is ON */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_0x84 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_0x84 = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);

    /* 2. RD4 Radio turned OFF (CAN 0x165: 0x48, 0xC8, 0x00, 0x00):
     * Must emit one-shot 0x84 0xFF (Radio OFF) cleanly */
    can_router_process_can(&frame_off);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_radio_off), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_radio_off, rx_buf, sizeof(expected_radio_off));

    /* 3. Inject background CAN tuner status (CAN 0x225 from dump log) while OFF:
     * Cached in state, but ZERO UART output (no flip-flopping to 0x01 / 0x85!) */
    can_frame_t frame_225_bg = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x30, 0x00, 0x10, 0x03, 0xDC }
    };
    can_router_process_can(&frame_225_bg);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 4. Inject background CAN CD track telemetry (CAN 0x3A5 from dump log) while OFF:
     * Cached in state, but ZERO UART output */
    can_frame_t frame_3a5_bg = {
        .id = 0x3A5,
        .dlc = 6,
        .data = { 0x0B, 0x02, 0x24, 0x02, 0x0E, 0x00 }
    };
    can_router_process_can(&frame_3a5_bg);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 5. Inject background CAN Station Name (0x2A5) and another 0x165 OFF frame:
     * ZERO UART output */
    can_router_process_can(&frame_2a5_boot);
    can_router_process_can(&frame_off);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 6. Verify 60s periodic resync does NOT emit any radio or CD packets when radio is off */
    for (int t = 0; t < 600; t++) {
        can_router_periodic_100ms();
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            uint8_t cmd = rx_buf[i + 3];
            TEST_ASSERT_NOT_EQUAL_HEX8(0x84, cmd);
            TEST_ASSERT_NOT_EQUAL_HEX8(0x85, cmd);
            TEST_ASSERT_NOT_EQUAL_HEX8(0x86, cmd);
            TEST_ASSERT_NOT_EQUAL_HEX8(0x97, cmd);
        }
    }

    /* 7. Turn RD4 radio back ON: telemetry resumes immediately with the cached 0x225 frequency! */
    can_router_process_can(&frame_on);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    found_0x84 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_0x84 = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);
}

void test_integration_hiworld_virtual_cdc_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();

    /* Make sure bus is awake by feeding an active CAN frame */
    can_frame_t frame_ign = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_ign);

    /* Drain initial UART output */
    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 1. Downlink: Send Hiworld Cmd 0xAD SubCmd 0x02, Param 0x01 (Audio Start / Play) */
    /* Wire packet: [ 0x5A, 0xA5, 0x03, 0xAD, 0x02, 0x01, 0xB2 ] */
    const uint8_t hu_play_packet[] = { 0x5A, 0xA5, 0x03, 0xAD, 0x02, 0x01, 0xB2 };
    for (size_t i = 0; i < sizeof(hu_play_packet); i++) {
        can_router_process_uart_byte(hu_play_packet[i]);
    }

    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());

    /* Check UART output: ACK for 0xAD (0x5A, 0xA5, 0x01, 0xFF, 0xAD, 0xAC) and Cmd 0x84 (CDC 0x31) */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_ack_0xad = false;
    bool found_0x84_cdc = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            if (rx_buf[i + 3] == 0xFF && (i + 4) < rx_len && rx_buf[i + 4] == 0xAD) {
                found_ack_0xad = true;
            }
            if (rx_buf[i + 3] == 0x84 && (i + 4) < rx_len && rx_buf[i + 4] == 0x31) {
                found_0x84_cdc = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_ack_0xad);
    TEST_ASSERT_TRUE(found_0x84_cdc);

    /* 2. Run periodic 100ms ticks: verifies CAN 0x162 is transmitted every 100ms */
    hal_can_native_clear_sent_frame();
    can_router_periodic_100ms();
    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x162, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(7, frame_sent.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x06, frame_sent.data[5]); /* Source registration magic byte */

    /* 3. Downlink: Send Hiworld Cmd 0xAD SubCmd 0x02, Param 0x00 (Audio Stop / Standby) */
    /* Wire packet: [ 0x5A, 0xA5, 0x03, 0xAD, 0x02, 0x00, 0xB1 ] */
    hal_can_native_clear_sent_frame();
    const uint8_t hu_stop_packet[] = { 0x5A, 0xA5, 0x03, 0xAD, 0x02, 0x00, 0xB1 };
    for (size_t i = 0; i < sizeof(hu_stop_packet); i++) {
        can_router_process_uart_byte(hu_stop_packet[i]);
    }

    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());

    /* Verify 100ms tick continues broadcasting 0x162 presence in standby */
    hal_can_native_clear_sent_frame();
    can_router_periodic_100ms();
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x162, frame_sent.id);
    TEST_ASSERT_EQUAL_HEX8(0x02, frame_sent.data[1] & 0x07); /* Status icon: 2 (Pause) */

    /* 4. Downlink: Send Hiworld Cmd 0xAD SubCmd 0x01, Param 0x00 (Disable Virtual CDC) */
    /* Wire packet: [ 0x5A, 0xA5, 0x03, 0xAD, 0x01, 0x00, 0xB0 ] */
    hal_can_native_clear_sent_frame();
    const uint8_t hu_disable_packet[] = { 0x5A, 0xA5, 0x03, 0xAD, 0x01, 0x00, 0xB0 };
    for (size_t i = 0; i < sizeof(hu_disable_packet); i++) {
        can_router_process_uart_byte(hu_disable_packet[i]);
    }

    TEST_ASSERT_FALSE(psa_virtual_cdc_is_presence_enabled());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());

    /* Verify 100ms tick halts all transmission when disabled */
    hal_can_native_clear_sent_frame();
    can_router_periodic_100ms();
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* 5. Downlink: Send Hiworld Cmd 0xAD SubCmd 0x01, Param 0x01 (Re-enable Virtual CDC) */
    /* Wire packet: [ 0x5A, 0xA5, 0x03, 0xAD, 0x01, 0x01, 0xB1 ] */
    const uint8_t hu_enable_packet[] = { 0x5A, 0xA5, 0x03, 0xAD, 0x01, 0x01, 0xB1 };
    for (size_t i = 0; i < sizeof(hu_enable_packet); i++) {
        can_router_process_uart_byte(hu_enable_packet[i]);
    }

    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());
    hal_can_native_clear_sent_frame();
    can_router_periodic_100ms();
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x162, frame_sent.id);
}

void test_integration_hiworld_rd4_audio_routing_spec(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_virtual_cdc_init();

    /* Wake bus */
    can_frame_t frame_ign = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_ign);

    /* Drain initial UART output */
    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* --------------------------------------------------------------------------
     * 1. Downlink: Switch RD4 to CDC (5A A5 02 AD 02 01 B1)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_cdc_active[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x02, 0x01, 0xB1 };
    for (size_t i = 0; i < sizeof(pkt_cdc_active); i++) {
        can_router_process_uart_byte(pkt_cdc_active[i]);
    }

    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());
    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Verify UART responses: Universal ACK (0xFF, 0xAD) and Cmd 0x84 (CDC 0x31) */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_ack_ad = false;
    bool found_84_cdc = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len) {
            if (rx_buf[i + 3] == 0xFF && rx_buf[i + 4] == 0xAD) {
                found_ack_ad = true;
            }
            if (rx_buf[i + 3] == 0x84 && rx_buf[i + 4] == 0x31) {
                found_84_cdc = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_ack_ad);
    TEST_ASSERT_TRUE(found_84_cdc);

    /* --------------------------------------------------------------------------
     * 2. Downlink: CDC Standby (5A A5 02 AD 02 00 B0)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_cdc_standby[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x02, 0x00, 0xB0 };
    for (size_t i = 0; i < sizeof(pkt_cdc_standby); i++) {
        can_router_process_uart_byte(pkt_cdc_standby[i]);
    }

    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());
    /* CDC Standby must NOT inject stalk pulse */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* --------------------------------------------------------------------------
     * 3. Downlink: Switch RD4 to Tuner (5A A5 02 AD 03 01 B3)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_tuner[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x01, 0xB3 };
    for (size_t i = 0; i < sizeof(pkt_switch_tuner); i++) {
        can_router_process_uart_byte(pkt_switch_tuner[i]);
    }

    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* --------------------------------------------------------------------------
     * 4. Downlink: Switch RD4 to CD (5A A5 02 AD 03 02 B4)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_cd[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x02, 0xB4 };
    for (size_t i = 0; i < sizeof(pkt_switch_cd); i++) {
        can_router_process_uart_byte(pkt_switch_cd[i]);
    }

    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Confirm CD settled via CAN 0x165 */
    can_frame_t frame_cd = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_cd);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());

    /* --------------------------------------------------------------------------
     * 5. Downlink: Switch to Tuner & Query Radio State / Station List (5A A5 01 0F 0F)
     * -------------------------------------------------------------------------- */
    /* HU sends Switch to Tuner, then immediately queries 0x0F */
    for (size_t i = 0; i < sizeof(pkt_switch_tuner); i++) {
        can_router_process_uart_byte(pkt_switch_tuner[i]);
    }
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    hal_can_native_clear_sent_frame();
    const uint8_t pkt_query_0f[] = { 0x5A, 0xA5, 0x01, 0x0F, 0x0F };
    for (size_t i = 0; i < sizeof(pkt_query_0f); i++) {
        can_router_process_uart_byte(pkt_query_0f[i]);
    }

    /* While switching in progress, rotary scroll must be deferred */
    const psa_source_arbiter_t *arb = psa_virtual_cdc_get_arbiter();
    TEST_ASSERT_TRUE(arb->pending_station_list_trigger);

    /* Confirm Tuner arrival via CAN 0x165 -> triggers deferred rotary scroll */
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 } /* 0x10 = Tuner */
    };
    can_router_process_can(&frame_165);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);

    /* Verify CAN 0x21F station list trigger was transmitted upon settling */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Verify UART responses: Universal ACK (0xFF, 0x0F) and Cmd 0x84 */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_ack_0f = false;
    bool found_84_radio = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len) {
            if (rx_buf[i + 3] == 0xFF && rx_buf[i + 4] == 0x0F) {
                found_ack_0f = true;
            }
            if (rx_buf[i + 3] == 0x84) {
                found_84_radio = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_ack_0f);
    TEST_ASSERT_TRUE(found_84_radio);
}

void test_integration_hiworld_source_switch_suppression(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_virtual_cdc_init();

    /* Set radio initially to Tuner via CAN 0x165 */
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165);

    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* Downlink: Switch RD4 to CD (5A A5 02 AD 03 02 B4) */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_cd[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x02, 0xB4 };
    for (size_t i = 0; i < sizeof(pkt_switch_cd); i++) {
        can_router_process_uart_byte(pkt_switch_cd[i]);
    }

    /* 1. Verify Stalk 0x21F pulse was injected */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_source_filtered());

    /* 2. Before RD4 acknowledges, 0x84 must NOT be sent to HU */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84_premature = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_84_premature = true;
        }
    }
    TEST_ASSERT_FALSE(found_84_premature);

    /* 3. RD4 acknowledges by sending CAN 0x165 (0x20 in high nibble = CD) */
    can_frame_t frame_cd = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_cd);

    /* 4. Target reached! Verify final confirmed state pushed to HU: Cmd 0x84 */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_84 = true;
        }
    }
    TEST_ASSERT_TRUE(found_84);
}

void test_integration_hiworld_multi_hop_source_switch(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. Initialize RD4 on CDC (0x30 in CAN 0x165) */
    can_frame_t frame_cdc = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x30, 0x00 }
    };
    can_router_process_can(&frame_cdc);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_reported_source());

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    /* 2. HU requests CD: 5A A5 02 AD 03 02 B3 */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_cd[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x02, 0xB3 };
    for (size_t i = 0; i < sizeof(pkt_switch_cd); i++) {
        can_router_process_uart_byte(pkt_switch_cd[i]);
    }

    /* Pulse 1: Injected immediately */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    /* 3. RD4 reacts to Pulse 1: transitions from CDC to AUX 1 (0x40) */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_aux = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x40, 0x00 }
    };
    can_router_process_can(&frame_aux);

    /* Pulse 2: Injected after AUX 1 received */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    /* 4. RD4 reacts to Pulse 2: transitions from AUX 1 to Tuner (0x10) */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x10, 0x00 }
    };
    can_router_process_can(&frame_tuner);

    /* Pulse 3: Injected after Tuner received */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    /* 5. RD4 reacts to Pulse 3: transitions from Tuner to CD (0x20) */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_cd = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x20, 0x00 }
    };
    can_router_process_can(&frame_cd);

    /* 6. Target reached! No Pulse 4 must be emitted */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_virtual_cdc_get_reported_source());

    /* Verify final confirmed state pushed to HU: Cmd 0x84 with 0x20 (CD) */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84_cd = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len && rx_buf[i + 3] == 0x84) {
            if (rx_buf[i + 4] == 0x20) {
                found_84_cd = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_84_cd);
}

