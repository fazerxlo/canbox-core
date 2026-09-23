#include "unity.h"
#include "test_integration_common.h"
#include "hal/hal_uart.h"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define PTY_SYMLINK "/tmp/ttyCanbox"

static int s_slave_fd = -1;

void setUp(void) {
    hal_uart_init(UART_BAUD_38400);
    can_router_init();
    // Default to Raise protocol for base tests
    hu_protocol_set_active(HU_PROTOCOL_RAISE);

    if (s_slave_fd >= 0) {
        close(s_slave_fd);
        s_slave_fd = -1;
    }

    s_slave_fd = open(PTY_SYMLINK, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (s_slave_fd >= 0) {
        uint8_t drain[256];
        while (read(s_slave_fd, drain, sizeof(drain)) > 0) {}
    }
}

void tearDown(void) {
    if (s_slave_fd >= 0) {
        close(s_slave_fd);
        s_slave_fd = -1;
    }
}

size_t read_uart_output(uint8_t *buf, size_t max_len) {
    if (s_slave_fd < 0) {
        return 0;
    }
    // Give a tiny moment for PTY buffers if needed
    usleep(5000);
    ssize_t n = read(s_slave_fd, buf, max_len);
    return (n > 0) ? (size_t)n : 0;
}

// 1. Raise Protocol Integration Tests (test_integration_raise.c)
void test_integration_raise_steering_wheel_volume_up_pipeline(void);
void test_integration_raise_door_status_pipeline(void);
void test_integration_raise_telemetry_periodic_pipeline(void);
void test_integration_raise_hu_uart_to_canbox_version_query(void);
void test_integration_raise_runtime_car_selection(void);

// 2. Hiworld Protocol Integration Tests (test_integration_hiworld.c)
void test_integration_hiworld_steering_wheel_volume_up_pipeline(void);
void test_integration_hiworld_door_status_pipeline(void);
void test_integration_hiworld_telemetry_periodic_pipeline(void);
void test_integration_hiworld_runtime_car_selection_and_handshake(void);

// 3. Bagoo Protocol Integration Tests (test_integration_bagoo.c)
void test_integration_bagoo_steering_wheel_volume_up_pipeline(void);
void test_integration_bagoo_door_status_pipeline(void);
void test_integration_bagoo_hu_uart_to_canbox_version_query(void);

// 4. Real CAN Log Scenario Tests (test_scenario_*.c)
void test_scenario_ignition_off_after_power_on(void);
void test_scenario_lights_off_side_light_on_headlights_on(void);

int main(void) {
    UNITY_BEGIN();

    // 1. Raise Protocol Integration Tests
    RUN_TEST(test_integration_raise_steering_wheel_volume_up_pipeline);
    RUN_TEST(test_integration_raise_door_status_pipeline);
    RUN_TEST(test_integration_raise_telemetry_periodic_pipeline);
    RUN_TEST(test_integration_raise_hu_uart_to_canbox_version_query);
    RUN_TEST(test_integration_raise_runtime_car_selection);

    // 2. Hiworld Protocol Integration Tests
    RUN_TEST(test_integration_hiworld_steering_wheel_volume_up_pipeline);
    RUN_TEST(test_integration_hiworld_door_status_pipeline);
    RUN_TEST(test_integration_hiworld_telemetry_periodic_pipeline);
    RUN_TEST(test_integration_hiworld_runtime_car_selection_and_handshake);

    // 3. Bagoo Protocol Integration Tests
    RUN_TEST(test_integration_bagoo_steering_wheel_volume_up_pipeline);
    RUN_TEST(test_integration_bagoo_door_status_pipeline);
    RUN_TEST(test_integration_bagoo_hu_uart_to_canbox_version_query);

    // 4. Real CAN Log Scenarios
    RUN_TEST(test_scenario_ignition_off_after_power_on);
    RUN_TEST(test_scenario_lights_off_side_light_on_headlights_on);

    return UNITY_END();
}

