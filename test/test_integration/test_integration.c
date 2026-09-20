#include "unity.h"
#include "core/can_router.h"
#include "hal/hal_uart.h"
#include "protocols/proto_raise.h"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define PTY_SYMLINK "/tmp/ttyCanbox"

static int s_slave_fd = -1;

void setUp(void) {
    hal_uart_init(UART_BAUD_38400);
    can_router_init();

    if (s_slave_fd >= 0) {
        close(s_slave_fd);
        s_slave_fd = -1;
    }

    s_slave_fd = open(PTY_SYMLINK, O_RDWR | O_NOCTTY | O_NONBLOCK);
}

void tearDown(void) {
    if (s_slave_fd >= 0) {
        close(s_slave_fd);
        s_slave_fd = -1;
    }
}

static size_t read_uart_output(uint8_t *buf, size_t max_len) {
    if (s_slave_fd < 0) {
        return 0;
    }
    // Give a tiny moment for PTY buffers if needed
    usleep(5000);
    ssize_t n = read(s_slave_fd, buf, max_len);
    return (n > 0) ? (size_t)n : 0;
}

void test_integration_steering_wheel_volume_up_pipeline(void) {
    can_frame_t frame = {
        .id = 0x128,
        .dlc = 2,
        .data = { 0x01, 0x00 } // Vol Up press
    };

    can_router_process_can(&frame);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expected Raise frame: Sync(0x2E), Cmd(0x01), Len(0x02), Payload(0x01, 0x00), Checksum(0xFB)
    // Checksum = ~(0x01 + 0x02 + 0x01 + 0x00) = ~0x04 = 0xFB
    const uint8_t expected[] = { 0x2E, 0x01, 0x02, 0x01, 0x00, 0xFB };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));

    // Now release the button
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    can_router_process_can(&frame);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expected Raise release frame: Sync(0x2E), Cmd(0x01), Len(0x02), Payload(0x00, 0x00), Checksum(0xFC)
    // Checksum = ~(0x01 + 0x02 + 0x00 + 0x00) = ~0x03 = 0xFC
    const uint8_t expected_release[] = { 0x2E, 0x01, 0x02, 0x00, 0x00, 0xFC };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));
}

void test_integration_door_status_pipeline(void) {
    can_frame_t frame = {
        .id = 0x036,
        .dlc = 4,
        .data = { 0x11, 0x00, 0x00, 0x00 } // Driver door (bit 0) + Trunk (bit 4)
    };

    can_router_process_can(&frame);

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // In Raise protocol Cmd 0x24:
    // Driver door is bit 0 (0x01), Trunk is bit 6 (0x40) -> Payload = 0x41
    // Checksum = ~(0x24 + 0x01 + 0x41) = ~0x66 = 0x99
    const uint8_t expected[] = { 0x2E, 0x24, 0x01, 0x41, 0x99 };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, sizeof(expected));
}

void test_integration_telemetry_periodic_pipeline(void) {
    // Engine RPM & Speed CAN frame
    // RPM raw = 8000 -> / 8 = 1000 RPM (0x1F40 -> data[0]=0x1F, data[1]=0x40)
    // Speed raw = 6400 -> >> 7 = 50 km/h (0x1900 -> data[2]=0x19, data[3]=0x00)
    can_frame_t b6_frame = {
        .id = 0x0B6,
        .dlc = 8,
        .data = { 0x1F, 0x40, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&b6_frame);

    // Steering angle CAN frame: 150 -> 15 deg
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

    // Telemetry: Cmd 0x29, Len 0x06, Payload: speed_be16(0x00, 0x32), rpm_be16(0x03, 0xE8), angle_be16(0x00, 0x0F)
    // Sum = 0x29 + 0x06 + 0x00 + 0x32 + 0x03 + 0xE8 + 0x00 + 0x0F = 0x15B -> 0x5B -> ~0x5B = 0xA4
    // Heartbeat: Cmd 0x20, Len 0x01, Payload: 0x01 -> Sum = 0x20 + 0x01 + 0x01 = 0x22 -> ~0x22 = 0xDD
    const uint8_t expected_telemetry[] = {
        0x2E, 0x29, 0x06, 0x00, 0x32, 0x03, 0xE8, 0x00, 0x0F, 0xA4,
        0x2E, 0x20, 0x01, 0x01, 0xDD
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_telemetry), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_telemetry, rx_buf, sizeof(expected_telemetry));
}

void test_integration_hu_uart_to_canbox_version_query(void) {
    // Android Head Unit sends version query packet:
    // Sync(0x2E), Cmd(0x7F), Len(0x00), Checksum(0x80)
    const uint8_t req[] = { 0x2E, 0x7F, 0x00, 0x80 };

    for (size_t i = 0; i < sizeof(req); i++) {
        can_router_process_uart_byte(req[i]);
    }

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Firmware responds with "SW001":
    // Cmd: 0x7F, Len: 0x05, Payload: 'S'(0x53), 'W'(0x57), '0'(0x30), '0'(0x30), '1'(0x31)
    // Sum = 0x7F + 0x05 + 0x53 + 0x57 + 0x30 + 0x30 + 0x31 = 0x1BF -> 0xBF -> ~0xBF = 0x40
    const uint8_t expected_resp[] = {
        0x2E, 0x7F, 0x05, 'S', 'W', '0', '0', '1', 0x40
    };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_resp), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_resp, rx_buf, sizeof(expected_resp));
}

#include "core/vehicle_profile.h"
#include "protocols/raise_car_mapping.h"

void test_integration_raise_runtime_car_selection(void) {
    // 1. Initially active profile is PSA
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_PSA_2004, vehicle_profile_get_active()->id);

    // 2. Android Head Unit sends Car Selection frame for VAG PQ35 (Brand: 0x02, Model: 0x01):
    // Sync(0x2E), Cmd(0xCA), Len(0x02), Brand(0x02), Model(0x01), Checksum(0x30)
    // Sum = 0xCA + 0x02 + 0x02 + 0x01 = 0xCF -> CS = ~0xCF = 0x30
    const uint8_t select_vag[] = { 0x2E, 0xCA, 0x02, 0x02, 0x01, 0x30 };
    for (size_t i = 0; i < sizeof(select_vag); i++) {
        can_router_process_uart_byte(select_vag[i]);
    }

    uint8_t rx_buf[32];
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));

    // Expected ACK frame: Sync(0x2E), Cmd(0xCA), Len(0x03), Brand(0x02), Model(0x01), Status(0x01), Checksum(0x2E)
    // Sum = 0xCA + 0x03 + 0x02 + 0x01 + 0x01 = 0xD1 -> CS = ~0xD1 = 0x2E
    const uint8_t expected_ack[] = { 0x2E, 0xCA, 0x03, 0x02, 0x01, 0x01, 0x2E };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_ack), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_ack, rx_buf, sizeof(expected_ack));

    // Verify active profile switched to VAG
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_VAG_PQ35, vehicle_profile_get_active()->id);

    // 3. Inject VAG CAN frame on CAN ID 0x5C0 (Vol Up = 0x06)
    can_frame_t vag_can = {
        .id = 0x5C0,
        .dlc = 1,
        .data = { 0x06 }
    };
    can_router_process_can(&vag_can);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    // Expected Raise wheel key Vol Up: Sync(0x2E), Cmd(0x01), Len(0x02), Key(0x01, 0x00), CS(0xFB)
    const uint8_t expected_key[] = { 0x2E, 0x01, 0x02, 0x01, 0x00, 0xFB };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_key), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, rx_buf, sizeof(expected_key));

    // 4. Switch back to PSA (Brand: 0x01, Model: 0x01)
    // Sum = 0xCA + 0x02 + 0x01 + 0x01 = 0xCE -> CS = ~0xCE = 0x31
    const uint8_t select_psa[] = { 0x2E, 0xCA, 0x02, 0x01, 0x01, 0x31 };
    for (size_t i = 0; i < sizeof(select_psa); i++) {
        can_router_process_uart_byte(select_psa[i]);
    }

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_PSA_2004, vehicle_profile_get_active()->id);
}

// Scenario tests
void test_scenario_ignition_off_after_power_on(void);
void test_scenario_lights_off_side_light_on_headlights_on(void);

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_integration_steering_wheel_volume_up_pipeline);
    RUN_TEST(test_integration_door_status_pipeline);
    RUN_TEST(test_integration_telemetry_periodic_pipeline);
    RUN_TEST(test_integration_hu_uart_to_canbox_version_query);
    RUN_TEST(test_integration_raise_runtime_car_selection);
    RUN_TEST(test_scenario_ignition_off_after_power_on);
    RUN_TEST(test_scenario_lights_off_side_light_on_headlights_on);
    return UNITY_END();
}
