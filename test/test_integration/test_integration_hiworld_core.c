#include <string.h>
#include "test_integration_common.h"
#include "core/vehicle_profile.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"
#include "hal/hal_gpio.h"
#include "hal/hal_system.h"

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

    // 3. Verify that receiving non-door CAN ID 0x221 (BSI status/trip) does NOT overwrite door state
    can_frame_t bsi_frame_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
    can_router_process_can(&bsi_frame_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    if (rx_len > 0) {
        TEST_ASSERT_NOT_EQUAL(0x12, rx_buf[3]); // Emitted trip 0x13, NOT bogus door status frame 0x12
    }

    // 4. Test PSA 0x220 all closed frame: DATA: 04 00 (bit 0x04 must NOT be treated as hood open)
    can_frame_t frame_220_closed = {
        .id = 0x220,
        .dlc = 2,
        .data = { 0x04, 0x00 }
    };
    can_router_process_can(&frame_220_closed);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_220_closed[] = {
        0x5A, 0xA5, 0x0A, 0x12, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x26
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_220_closed), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_220_closed, rx_buf, sizeof(expected_220_closed));

    // Verify doors are all closed in router state
    const vehicle_state_t *st = can_router_get_state();
    TEST_ASSERT_FALSE(st->doors.door_driver);
    TEST_ASSERT_FALSE(st->doors.door_passenger);
    TEST_ASSERT_FALSE(st->doors.door_rear_left);
    TEST_ASSERT_FALSE(st->doors.door_rear_right);
    TEST_ASSERT_FALSE(st->doors.trunk);
    TEST_ASSERT_FALSE(st->doors.hood);

    // Verify that periodic 100ms ticks do NOT retransmit 0x12 when all doors are closed
    for (int i = 0; i < 15; i++) {
        can_router_periodic_100ms();
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Must only have heartbeat 0xFF frames, ZERO 0x12 door frames
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && i + 3 < rx_len) {
            TEST_ASSERT_NOT_EQUAL(HIWORLD_CMD_DOOR_WINDOW_STATE, rx_buf[i + 3]);
        }
    }
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
    hiworld_conn_reset_024_state(proto_hiworld_get_connection_ctx());

    uint8_t rx_buf[256];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 1. Initial 0x24 (Case A - Initial Boot / Model Configuration):
     * 5A A5 02 24 22 00 47 (Peugeot 407) */
    const uint8_t stream[] = { 0x5A, 0xA5, 0x02, 0x24, 0x22, 0x00, 0x47 };
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Expect response with ACK (0xFF, payload 0x24) + Version (0xF0) + Feature Enables + Full State */
    const uint8_t expected_ack[] = { 0x5A, 0xA5, 0x01, 0xFF, 0x24, 0x23 };
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_ack));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    /* Verify subsequent frame is Version Report (0xF0) */
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF1, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF2, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_CMD_VERSION_REPORT, rx_buf[9]);

    /* Verify full state push includes doors (0x12) */
    bool found_doors = false;
    for (size_t i = 0; i + 3 < rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_DOOR_WINDOW_STATE) {
            found_doors = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found_doors);

    /* Drain all remaining bytes from initial handshake burst */
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 2. Steady-State Ping 1 (Case B, Step 0 @ t+3s):
     * Emits ACK + static discovery (0xF0, 0x71, 0x72, 0xC1).
     * Must NOT perform full state push (no doors 0x12). */
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_ack));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    bool found_version = false;
    found_doors = false;
    for (size_t i = 0; i + 3 < rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_VERSION_REPORT) {
            found_version = true;
        }
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_DOOR_WINDOW_STATE) {
            found_doors = true;
        }
    }
    TEST_ASSERT_TRUE(found_version);
    TEST_ASSERT_FALSE(found_doors);

    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 3. Steady-State Ping 2 (Case B, Step 1 @ t+6s):
     * Emits ACK + Central Vehicle Settings (0x76 & 0x79).
     * Must NOT send 0xF0, 0x12, or 0x31. */
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_ack));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    bool found_central = false;
    found_version = false;
    for (size_t i = 0; i + 3 < rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_CENTRAL_STATE1) {
            found_central = true;
        }
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_VERSION_REPORT) {
            found_version = true;
        }
    }
    TEST_ASSERT_TRUE(found_central);
    TEST_ASSERT_FALSE(found_version);

    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 4. Steady-State Ping 3 (Case B, Step 2 @ t+9s):
     * Emits ACK + Climate (0x31) + Doors (0x12).
     * Must NOT send 0xF0 or Central State 0x76. */
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_ack));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    bool found_climate = false;
    found_central = false;
    for (size_t i = 0; i + 3 < rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_AIR_CON) {
            found_climate = true;
        }
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_CENTRAL_STATE1) {
            found_central = true;
        }
    }
    TEST_ASSERT_TRUE(found_climate);
    TEST_ASSERT_FALSE(found_central);

    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 5. Steady-State Ping 4 (Case B, Step 3 @ t+12s):
     * Emits ACK + Radio State (0x84).
     * Must NOT send 0x85 (presets), 0xF0, or 0x31. */
    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_ack));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    bool found_radio = false;
    bool found_presets = false;
    for (size_t i = 0; i + 3 < rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_CAR_RADIO_STATE) {
            found_radio = true;
        }
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_CAR_RADIO_PRESET_FREQ) {
            found_presets = true;
        }
    }
    TEST_ASSERT_TRUE(found_radio);
    TEST_ASSERT_FALSE(found_presets);

    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 6. Simulated HU Reboot Gap (> 4500 ms):
     * Simulate that Android restarted and no 0x24 was seen for 5000 ms.
     * Must trigger Case A again (Instant Full State Push). */
    hiworld_connection_ctx_t *ctx = proto_hiworld_get_connection_ctx();
    TEST_ASSERT_NOT_NULL(ctx);
    ctx->last_024_rx_ms = hal_get_tick_ms() - 5000;

    for (size_t i = 0; i < sizeof(stream); i++) {
        hu_protocol_get_active()->feed_byte(stream[i]);
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_ack));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    found_version = false;
    found_doors = false;
    for (size_t i = 0; i + 3 < rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_VERSION_REPORT) {
            found_version = true;
        }
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == HIWORLD_CMD_DOOR_WINDOW_STATE) {
            found_doors = true;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_version, "Expected full discovery burst (0xF0) after reboot gap");
    TEST_ASSERT_TRUE_MESSAGE(found_doors, "Expected full state push (doors) after reboot gap");
}

void test_integration_hiworld_tpms_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);

    // Drain any leftover UART bytes
    uint8_t drain[128];
    while (read_uart_output(drain, sizeof(drain)) > 0);

    // Inject CAN ID 0x1E1 and 0x3A1 for TPMS:
    // FL: 0.0 Bar, puncture (state 2)
    // FR: 1.5 Bar, low (state 1)
    // RR: 3.0 Bar, ok (state 0)
    // RL: 2.2 Bar, ok (state 0)
    // Inject CAN ID 0x3A1 for direct numeric TPMS:
    // FL: 0.0 Bar
    // FR: 1.5 Bar
    // RR: 3.0 Bar
    // RL: 2.2 Bar
    can_frame_t frame_3a1 = {
        .id = 0x3A1,
        .dlc = 4,
        .data = { 0, 30, 60, 44 }
    };
    can_router_process_can(&frame_3a1);

    uint8_t rx_buf[64];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expected UART Output:
    // 1. Numeric TPMS (Cmd 0x66): Mode(0x01), FL(0=0x00), FR(15=0x0F), RL(22=0x16), RR(30=0x1E), Unit(0x00), CS(0xAF)
    //    Wire: 5A A5 06 66 01 00 0F 16 1E 00 AF
    // 2. Discrete TPMS (Cmd 0x18): FL(0), FR(0), RL(0), RR(0), CS(0x1B)
    //    Wire: 5A A5 04 18 00 00 00 00 1B
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x06, 0x66, 0x01, 0x00, 0x0F, 0x16, 0x1E, 0x00, 0xAF,
        0x5A, 0xA5, 0x04, 0x18, 0x00, 0x00, 0x00, 0x00, 0x1B
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));

    // 1. Re-inject identical frame -> No UART TPMS frames should be sent
    can_router_process_can(&frame_3a1);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    // 2. Inject frame with pressure change ONLY (alarms unchanged):
    // Change RL pressure from 2.2 Bar (44) to 2.4 Bar (48). Alarms remain identical.
    frame_3a1.data[3] = 48;
    can_router_process_can(&frame_3a1);
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
        .data = { 0x25, 0x02, 0x39, 0x00, 0x49, 0x03, 0x99 }
    };
    can_router_process_can(&frame_2a1);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_2a1[] = { 0x5A, 0xA5, 0x0A, 0x14, 0x00, 0x49, 0x00, 0x25, 0x02, 0x39, 0x00, 0x00, 0x00, 0x00, 0xC6 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_2a1), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_2a1, rx_buf, sizeof(expected_2a1));

    // 3. Vector 3: PSA CAN 0x261 (Trip 2 Historical)
    // Distance: 921 km (0x0399), Fuel: 7.9 L/100km (79 = 0x004F), Mean Speed: 35 km/h (0x23)
    can_frame_t frame_261 = {
        .id = 0x261,
        .dlc = 7,
        .data = { 0x23, 0x03, 0x99, 0x00, 0x4F, 0x03, 0x99 }
    };
    can_router_process_can(&frame_261);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_261[] = { 0x5A, 0xA5, 0x0A, 0x15, 0x00, 0x4F, 0x00, 0x23, 0x03, 0x99, 0x00, 0x00, 0x00, 0x00, 0x2C };
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

    // CAN 0x2A1: DATA: 47 02 39 00 49 03 99 -> Dist: 569 km (0x0239), Fuel: 7.3 (0x0049), Speed: 71 km/h (0x47)
    can_frame_t dump_2a1 = {
        .id = 0x2A1,
        .dlc = 7,
        .data = { 0x47, 0x02, 0x39, 0x00, 0x49, 0x03, 0x99 }
    };
    can_router_process_can(&dump_2a1);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_dump_2a1[] = { 0x5A, 0xA5, 0x0A, 0x14, 0x00, 0x49, 0x00, 0x47, 0x02, 0x39, 0x00, 0x00, 0x00, 0x00, 0xE8 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_dump_2a1), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_2a1, rx_buf, sizeof(expected_dump_2a1));

    // CAN 0x261: DATA: 24 03 99 00 4f 03 99 -> Dist: 921 km (0x0399), Fuel: 7.9 (0x004F), Speed: 36 km/h (0x24)
    can_frame_t dump_261 = {
        .id = 0x261,
        .dlc = 7,
        .data = { 0x24, 0x03, 0x99, 0x00, 0x4F, 0x03, 0x99 }
    };
    can_router_process_can(&dump_261);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_dump_261[] = { 0x5A, 0xA5, 0x0A, 0x15, 0x00, 0x4F, 0x00, 0x24, 0x03, 0x99, 0x00, 0x00, 0x00, 0x00, 0x2D };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_dump_261), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_261, rx_buf, sizeof(expected_dump_261));

    // 5. Verify 0x221 with bit 7 set (nominal PSA 407 C0/C8/80) emits valid trip 0x13
    can_frame_t masked_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x80, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00 }
    };
    can_router_process_can(&masked_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_221_80[] = { 0x5A, 0xA5, 0x0A, 0x13, 0x00, 0x44, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_221_80), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_221_80, rx_buf, sizeof(expected_221_80));

    // 6. Stationary sentinel frame with 0xFFFF (dump_2026-10-20_13-33-42.log) must emit 0x13
    can_frame_t stationary_221 = {
        .id = 0x221,
        .dlc = 7,
        .data = { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
    can_router_process_can(&stationary_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_stationary_221[] = { 0x5A, 0xA5, 0x0A, 0x13, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x16 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_stationary_221), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_stationary_221, rx_buf, sizeof(expected_stationary_221));

    // 7. Verify rate limiting / deduplication: re-injecting unchanged trip frames produces ZERO UART transmissions
    can_router_process_can(&stationary_221);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    can_router_process_can(&dump_2a1);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    can_router_process_can(&dump_261);
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
    TEST_ASSERT_EQUAL_HEX32(0x167, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent_frame.data[1]); // Bit 7: Trip 1 Reset
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[3]);

    // 2. Android sends Trip 1 Reset (2-byte format, page 2 = EcuInfoPage2): 5A A5 02 1B 02 01 1F
    hal_can_native_clear_sent_frame();
    const uint8_t reset_trip1_p2_cmd[] = { 0x5A, 0xA5, 0x02, 0x1B, 0x02, 0x01, 0x1F };
    for (size_t i = 0; i < sizeof(reset_trip1_p2_cmd); i++) {
        can_router_process_uart_byte(reset_trip1_p2_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent_frame.data[1]); // Bit 7: Trip 1 Reset
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[3]);

    // 3. Android sends Trip 1 Reset (4-byte format from bench log dump_2026-09-29_17-13-36.log): 5A A5 04 1B 02 02 01 FF 22
    hal_can_native_clear_sent_frame();
    const uint8_t reset_trip1_4byte_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x02, 0x02, 0x01, 0xFF, 0x22 };
    for (size_t i = 0; i < sizeof(reset_trip1_4byte_cmd); i++) {
        can_router_process_uart_byte(reset_trip1_4byte_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent_frame.data[1]); // Bit 7: Trip 1 Reset
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[3]);

    // 4. Android sends Trip 2 Reset (4-byte format from bench log dump_2026-09-29_17-17-52.log): 5A A5 04 1B 03 03 01 FF 24
    hal_can_native_clear_sent_frame();
    const uint8_t reset_trip2_4byte_cmd[] = { 0x5A, 0xA5, 0x04, 0x1B, 0x03, 0x03, 0x01, 0xFF, 0x24 };
    for (size_t i = 0; i < sizeof(reset_trip2_4byte_cmd); i++) {
        can_router_process_uart_byte(reset_trip2_4byte_cmd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x40, sent_frame.data[1]); // Bit 6: Trip 2 Reset
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[3]);

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

    uint8_t rx_buf[64];
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
    const uint8_t expected_trip_instant[] = {
        0x5A, 0xA5, 0x0A, 0x13, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x16
    };
    TEST_ASSERT_TRUE(rx_len >= sizeof(expected_stalk_trip_press));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_stalk_trip_press, rx_buf, sizeof(expected_stalk_trip_press));
    if (rx_len >= sizeof(expected_stalk_trip_press) + sizeof(expected_trip_instant)) {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_trip_instant, rx_buf + sizeof(expected_stalk_trip_press), sizeof(expected_trip_instant));
    }

    // Release Stalk Tip Trip Button (0xC0 nominal from vehicle dumps)
    hal_can_native_clear_sent_frame();
    frame_221.data[0] = 0xC0;
    can_router_process_can(&frame_221);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_stalk_trip_release[] = {
        0x5A, 0xA5, 0x0A, 0x11, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22, 0xC7
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_stalk_trip_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_stalk_trip_release, rx_buf, sizeof(expected_stalk_trip_release));

    // Verify 0x167 is sent on release advancing trip page
    can_frame_t sent_frame;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_frame));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x02, sent_frame.data[0]); // Advance to Trip 1
    TEST_ASSERT_EQUAL_HEX8(0x00, sent_frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent_frame.data[3]);
}

