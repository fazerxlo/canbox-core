#include "unity.h"
#include "can_log_player.h"
#include "core/can_router.h"

static const char *LOG_PATH = "test/test_integration/data/lights_off_side_light_on_headlights_on.csv";

void test_scenario_lights_off_side_light_on_headlights_on(void) {
    can_log_player_t player;
    bool opened = can_log_player_open(&player, LOG_PATH);
    TEST_ASSERT_TRUE_MESSAGE(opened, "Failed to open CAN log CSV file");

    can_router_init();

    // -------------------------------------------------------------------------
    // Scenario Step 1: Initial Baseline & Lights OFF State (lights_off)
    // Replay initial frames before side lights are switched ON (t < 10.35s).
    // First 0x128 frame arrives at Frame 42 (t = 8.060354s) with D5 = 0x00.
    // -------------------------------------------------------------------------
    uint32_t count = can_log_player_replay_until_timestamp(&player, 10000000ULL); // 10.0s mark
    TEST_ASSERT_EQUAL_UINT32(604, count);
    TEST_ASSERT_EQUAL_UINT32(604, player.frames_processed);

    const vehicle_state_t *state = can_router_get_state();

    // Verify lights_off: all lighting indicators remain false (OFF)
    TEST_ASSERT_FALSE_MESSAGE(state->lights.side_light, "Expected side_light OFF in lights_off phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.headlights, "Expected headlights OFF in lights_off phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.high_beam,  "Expected high_beam OFF in lights_off phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.front_fog,  "Expected front_fog OFF in lights_off phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.rear_fog,   "Expected rear_fog OFF in lights_off phase");

    // Ignition and periodic virtual time assertions
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_EQUAL_UINT32(20, player.periodic_ticks_fired);

    // -------------------------------------------------------------------------
    // Scenario Step 2: Side Lights Activation (side_light_on)
    // Replay through frame 711 (t = 10.354113s) where 0x128 D5 changes to 0x80.
    // -------------------------------------------------------------------------
    count = can_log_player_replay_until_timestamp(&player, 10354113ULL);
    TEST_ASSERT_GREATER_THAN(0, count);
    TEST_ASSERT_EQUAL_UINT32(711, player.frames_processed);

    // Verify side_light_on: side_light is ON, headlights remain OFF
    TEST_ASSERT_TRUE_MESSAGE(state->lights.side_light,  "Expected side_light ON in side_light_on phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.headlights, "Expected headlights OFF in side_light_on phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.high_beam,  "Expected high_beam OFF in side_light_on phase");

    // -------------------------------------------------------------------------
    // Scenario Step 3: Dipped Headlights Activation (headlights_on)
    // Replay through frame 1323 (t = 12.455485s) where 0x128 D5 changes to 0xC0
    // (0x80 side lights | 0x40 headlights).
    // -------------------------------------------------------------------------
    count = can_log_player_replay_until_timestamp(&player, 12455485ULL);
    TEST_ASSERT_GREATER_THAN(0, count);
    TEST_ASSERT_EQUAL_UINT32(1323, player.frames_processed);

    // Verify headlights_on: both side_light and headlights are ON
    TEST_ASSERT_TRUE_MESSAGE(state->lights.side_light,  "Expected side_light ON in headlights_on phase");
    TEST_ASSERT_TRUE_MESSAGE(state->lights.headlights,  "Expected headlights ON in headlights_on phase");
    TEST_ASSERT_FALSE_MESSAGE(state->lights.high_beam,  "Expected high_beam OFF in headlights_on phase");

    // -------------------------------------------------------------------------
    // Scenario Step 4: High Beam Transition & Full Scenario Completion
    // Replay remaining frames through end of log (t = 18.223402s).
    // -------------------------------------------------------------------------
    can_log_player_replay_all(&player);

    TEST_ASSERT_EQUAL_UINT32(2992, player.frames_processed);
    TEST_ASSERT_EQUAL_UINT32(103, player.periodic_ticks_fired);
    TEST_ASSERT_EQUAL_UINT64(10300920ULL, player.simulated_elapsed_us);

    // Final settled lighting state check (in frame 1945 D5=0xA0: side_light + high_beam)
    TEST_ASSERT_TRUE(state->lights.side_light);
    TEST_ASSERT_TRUE(state->lights.high_beam);

    can_log_player_close(&player);
}
