#include "unity.h"
#include "can_log_player.h"
#include "core/can_router.h"

static const char *LOG_PATH = "test/test_integration/data/ignition_off_after_power_on.csv";

void test_scenario_ignition_off_after_power_on(void) {
    can_log_player_t player;
    bool opened = can_log_player_open(&player, LOG_PATH);
    TEST_ASSERT_TRUE_MESSAGE(opened, "Failed to open CAN log CSV file");

    can_router_init();

    // -------------------------------------------------------------
    // Scenario Step 1: Baseline / Pre-Ignition Sparse Bus
    // Replay initial frames before the first 0x036 frame (t < 49.36s)
    // -------------------------------------------------------------
    uint32_t count = can_log_player_replay_until_timestamp(&player, 49360000ULL);
    TEST_ASSERT_EQUAL_UINT32(14, count);
    TEST_ASSERT_EQUAL_UINT32(14, player.frames_processed);

    const vehicle_state_t *state = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT16(0, state->speed_kmh);
    TEST_ASSERT_EQUAL_UINT16(0, state->rpm);
    TEST_ASSERT_EQUAL_INT16(0, state->steering_angle_deg);
    TEST_ASSERT_FALSE(state->reverse_gear);
    TEST_ASSERT_FALSE(state->handbrake);
    TEST_ASSERT_FALSE(state->doors.door_driver);

    // -------------------------------------------------------------
    // Scenario Step 2: ACC / Power-On Announcement
    // Replay the first 0x036 message at t = 49.363170s
    // -------------------------------------------------------------
    count = can_log_player_replay_until_can_id(&player, 0x036);
    TEST_ASSERT_EQUAL_UINT32(1, count); // First 0x036 frame (Frame 15)
    TEST_ASSERT_EQUAL_UINT32(15, player.frames_processed);

    // Verify 0x036 decoding
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ACC, state->ignition_state);
    TEST_ASSERT_FALSE(state->doors.door_driver);
    TEST_ASSERT_TRUE(state->doors.door_passenger);
    TEST_ASSERT_TRUE(state->doors.door_rear_left);
    TEST_ASSERT_TRUE(state->doors.door_rear_right);
    TEST_ASSERT_FALSE(state->doors.trunk);
    TEST_ASSERT_FALSE(state->doors.hood);
    TEST_ASSERT_FALSE(state->reverse_gear);

    // -------------------------------------------------------------
    // Scenario Step 3: Sustained ACC Phase & Virtual Time Progression
    // Advance virtual time through the ~100ms periodic 0x036 stream to t = 60.0s
    // -------------------------------------------------------------
    count = can_log_player_replay_until_timestamp(&player, 60000000ULL);
    TEST_ASSERT_GREATER_THAN(50, count);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ACC, state->ignition_state);
    // Ensure virtual time drove periodic 100ms ticks (~10.6s elapsed = ~106 ticks)
    TEST_ASSERT_GREATER_THAN(100, player.periodic_ticks_fired);

    // -------------------------------------------------------------
    // Scenario Step 4: Settled Ignition OFF Transition
    // Replay the remainder of the log to the final frame 156 (t = 63.263189s)
    // -------------------------------------------------------------
    can_log_player_replay_all(&player);

    TEST_ASSERT_EQUAL_UINT32(156, player.frames_processed);
    TEST_ASSERT_GREATER_THAN(130, player.periodic_ticks_fired);
    TEST_ASSERT_GREATER_THAN_UINT64(14000000ULL, player.simulated_elapsed_us);

    // In frame 156, 0x036 D5 dropped from 0x02 (ACC) to 0x00 (Settled OFF)
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);

    can_log_player_close(&player);
}
