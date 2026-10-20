#include "unity.h"
#include "can_log_player.h"
#include "core/can_router.h"
#include "profiles/peugeot_407.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "hal/hal_gpio.h"

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
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power should be OFF prior to ignition");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_CAN_STBY), "Transceiver should be active on CAN traffic");

    // -------------------------------------------------------------
    // Scenario Step 2: Battery Connect / Key OFF Announcement (0x036 with D5=0x02)
    // Replay the first 0x036 message at t = 49.363170s
    // Per Peugeot CAN2004 spec: 0x02 = Ignition OFF (Going to sleep)
    // -------------------------------------------------------------
    count = can_log_player_replay_until_can_id(&player, 0x036);
    TEST_ASSERT_EQUAL_UINT32(1, count); // First 0x036 frame (Frame 15)
    TEST_ASSERT_EQUAL_UINT32(15, player.frames_processed);

    // Verify 0x036 decoding: 0x02 is Ignition OFF, ACC power must remain OFF
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power must remain OFF when key is OFF (0x02)");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_CAN_STBY));
    TEST_ASSERT_FALSE(state->doors.door_driver);
    TEST_ASSERT_FALSE(state->doors.door_passenger);
    TEST_ASSERT_FALSE(state->doors.door_rear_left);
    TEST_ASSERT_FALSE(state->doors.door_rear_right);
    TEST_ASSERT_FALSE(state->doors.trunk);
    TEST_ASSERT_FALSE(state->doors.hood);
    TEST_ASSERT_FALSE(state->reverse_gear);

    // -------------------------------------------------------------
    // Scenario Step 3: Sustained Key OFF Phase & Virtual Time Progression
    // Advance virtual time through the ~100ms periodic 0x036 stream to t = 60.0s
    // -------------------------------------------------------------
    count = can_log_player_replay_until_timestamp(&player, 60000000ULL);
    TEST_ASSERT_GREATER_THAN(50, count);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power must remain OFF throughout 0x02 phase");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_CAN_STBY));
    // Ensure virtual time drove periodic 100ms ticks (~10.6s elapsed = ~106 ticks)
    TEST_ASSERT_GREATER_THAN(100, player.periodic_ticks_fired);

    // -------------------------------------------------------------
    // Scenario Step 4: Settled Deep Sleep (0x00) Transition
    // Replay the remainder of the log to the final frame 156 (t = 63.263189s)
    // -------------------------------------------------------------
    can_log_player_replay_all(&player);

    TEST_ASSERT_EQUAL_UINT32(156, player.frames_processed);
    TEST_ASSERT_GREATER_THAN(130, player.periodic_ticks_fired);
    TEST_ASSERT_GREATER_THAN_UINT64(14000000ULL, player.simulated_elapsed_us);

    // In frame 156, 0x036 D5 dropped from 0x02 (Ignition OFF) to 0x00 (Deep Sleep / Standby)
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power must remain OFF in deep sleep (0x00)");

    can_log_player_close(&player);

    // -------------------------------------------------------------
    // Scenario Step 5: Bus Inactivity & Standby Sleep Watchdog
    // Simulate 3.0s (30 ticks) of CAN bus silence after log completion
    // -------------------------------------------------------------
    TEST_ASSERT_FALSE_MESSAGE(can_router_is_bus_sleeping(), "Bus should not be sleeping immediately after traffic");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_CAN_STBY), "CAN transceiver should still be active");

    for (int i = 0; i < 29; i++) {
        can_router_periodic_100ms();
    }
    // At 29 ticks (2.9s), watchdog has not expired yet
    TEST_ASSERT_FALSE(can_router_is_bus_sleeping());
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_CAN_STBY));

    // 30th tick (3.0s total silence) triggers sleep transition
    can_router_periodic_100ms();
    TEST_ASSERT_TRUE_MESSAGE(can_router_is_bus_sleeping(), "Bus should enter sleep mode after 3.0s silence");
    TEST_ASSERT_TRUE_MESSAGE(hal_gpio_read(GPIO_PIN_CAN_STBY), "CAN transceiver should be put into standby mode");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power must remain OFF in sleep");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_ILL_OUT), "Illumination output must be OFF in sleep");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_REVERSE_OUT), "Reverse output must be OFF in sleep");

    // Receiving any CAN frame wakes up bus
    can_frame_t wake_frame = { .id = 0x217, .dlc = 8, .data = {0} };
    can_router_process_can(&wake_frame);
    TEST_ASSERT_FALSE_MESSAGE(can_router_is_bus_sleeping(), "Bus should wake up upon receiving CAN frame");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_CAN_STBY), "Transceiver should be active upon wake up");
}

void test_scenario_ignition_silence_watchdog_forces_sleep_from_on(void) {
    can_router_init();

    // Setup: Bus is active with Ignition ON
    can_frame_t frame_ign_on = {
        .id = 0x036,
        .dlc = 8,
        .data = {0x0E, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xA0}
    };
    can_router_process_can(&frame_ign_on);

    const vehicle_state_t *state = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power should be ON");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_CAN_STBY));
    TEST_ASSERT_FALSE(can_router_is_bus_sleeping());

    // Advance 29 ticks without CAN traffic (< 3.0s)
    for (int i = 0; i < 29; i++) {
        can_router_periodic_100ms();
    }
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));
    TEST_ASSERT_FALSE(can_router_is_bus_sleeping());
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_CAN_STBY));

    // 30th tick reaches 3.0s bus silence watchdog threshold
    can_router_periodic_100ms();

    // Watchdog must unconditionally force sleep, kill ACC power, and enter standby
    TEST_ASSERT_TRUE_MESSAGE(can_router_is_bus_sleeping(), "Watchdog must force sleep mode on CAN silence");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_OFF, state->ignition_state, "Ignition state must be forced OFF");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power must be cut on CAN silence");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_ILL_OUT), "ILL power must be cut on CAN silence");
    TEST_ASSERT_TRUE_MESSAGE(hal_gpio_read(GPIO_PIN_CAN_STBY), "CAN transceiver standby must be asserted");

    // CAN activity wakes up the system
    can_frame_t frame_wake = {
        .id = 0x036,
        .dlc = 8,
        .data = {0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0x50}
    };
    can_router_process_can(&frame_wake);
    TEST_ASSERT_FALSE_MESSAGE(can_router_is_bus_sleeping(), "CAN frame must wake bus from sleep");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_CAN_STBY), "Transceiver standby must be deasserted on wake");
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));
}

void test_scenario_ignition_economy_mode_forces_radio_off(void) {
    can_router_init();
    const vehicle_state_t *state = can_router_get_state();

    // 1. Ignition ON reference frame: 0E 00 00 0F 01 00 00 A0
    can_frame_t frame_on = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x0F, 0x01, 0x00, 0x00, 0xA0 }
    };
    can_router_process_can(&frame_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));
    TEST_ASSERT_FALSE(state->economy_mode);

    // 2. Economy mode active on 0x036 with key OFF (Byte 2 Bit 7 = 0x80, Byte 4 = 0x02): 0E 00 80 0F 02 00 00 A0
    can_frame_t frame_eco = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x80, 0x0F, 0x02, 0x00, 0x00, 0xA0 }
    };
    can_router_process_can(&frame_eco);
    TEST_ASSERT_TRUE(state->economy_mode);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_OFF, state->ignition_state, "Economy mode must force ignition OFF");
    TEST_ASSERT_FALSE_MESSAGE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER), "ACC power must be cut when Economy Mode is active");

    // 3. Attempting to turn RD4 radio ON via 0x165 while in economy mode must keep ACC OFF
    can_frame_t frame_radio_on = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xA0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_radio_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Turn radio back OFF
    can_frame_t frame_radio_off = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x48, 0xA0, 0x00, 0x00 }
    };
    can_router_process_can(&frame_radio_off);

    // 4. Return to Normal mode with Ignition OFF (0x02) and Radio OFF: 0E 00 00 0F 02 00 00 A0
    can_frame_t frame_norm_off = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x0F, 0x02, 0x00, 0x00, 0xA0 }
    };
    can_router_process_can(&frame_norm_off);
    TEST_ASSERT_FALSE(state->economy_mode);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // 5. In Normal mode, RD4 radio ON with key OFF promotes state to VEHICLE_IGNITION_ACC
    can_router_process_can(&frame_radio_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ACC, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // 6. If Economy mode subsequently triggers, ACC is immediately cut
    can_router_process_can(&frame_eco);
    TEST_ASSERT_TRUE(state->economy_mode);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // 7. Normal mode with RD4 radio turned OFF via 0x165 (0x48 transition, then 0x08 sleep)
    can_router_process_can(&frame_norm_off); // Return to normal mode
    can_router_process_can(&frame_radio_off); // 0x48: Audio turning off, screen goodbye
    TEST_ASSERT_FALSE(psa_rd4_is_powered(&state->media.radio));
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // 8. RD4 radio entering deep sleep / standby (0x08 80 00 00)
    can_frame_t frame_radio_sleep = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x08, 0x80, 0x00, 0x00 }
    };
    can_router_process_can(&frame_radio_sleep);
    TEST_ASSERT_TRUE(state->media.radio.sleep);
    TEST_ASSERT_FALSE(psa_rd4_is_powered(&state->media.radio));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_OFF, state->ignition_state, "Radio in sleep mode must keep ACC OFF");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Ignition ON (+APC) overrides sleeping radio: Head Unit turns ON
    can_frame_t frame_ign_with_radio_sleep = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x0F, 0x01, 0x00, 0x00, 0xA0 }
    };
    can_router_process_can(&frame_ign_with_radio_sleep);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_ON, state->ignition_state, "Ignition ON must power HU even if radio is sleeping");
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // 9. When radio powers back ON (0xC8), ACC turns ON
    can_router_process_can(&frame_radio_on);
    TEST_ASSERT_TRUE(psa_rd4_is_powered(&state->media.radio));
    TEST_ASSERT_FALSE(state->media.radio.sleep);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));
}

bool hal_can_native_get_last_sent_frame(can_frame_t *out_frame);
void hal_can_native_clear_sent_frame(void);

void test_scenario_sleeping_bus_blocks_can_tx_on_uart_activity(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();

    // Advance 30 ticks (3.0s) of CAN silence so bus enters sleep mode
    for (int i = 0; i < 30; i++) {
        can_router_periodic_100ms();
    }
    TEST_ASSERT_TRUE(can_router_is_bus_sleeping());
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_CAN_STBY));

    hal_can_native_clear_sent_frame();

    // 1. Late HU sends Date/Time Set (0xC6): 5A A5 06 C6 1A 0A 12 14 15 01 1A
    const uint8_t pkt_time[] = { 0x5A, 0xA5, 0x06, 0xC6, 0x1A, 0x0A, 0x12, 0x14, 0x15, 0x01, 0x2B };
    for (size_t i = 0; i < sizeof(pkt_time); i++) {
        can_router_process_uart_byte(pkt_time[i]);
    }

    // 2. Late HU sends Virtual CDC switch command (0xAD 02 01): 5A A5 02 AD 02 01 B1
    const uint8_t pkt_cdc[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x02, 0x01, 0xB1 };
    for (size_t i = 0; i < sizeof(pkt_cdc); i++) {
        can_router_process_uart_byte(pkt_cdc[i]);
    }

    // 3. Late HU sends Diagnostic Query (0x2F): 5A A5 02 2F 00 00 D4
    const uint8_t pkt_diag[] = { 0x5A, 0xA5, 0x02, 0x2F, 0x00, 0x00, 0xD4 };
    for (size_t i = 0; i < sizeof(pkt_diag); i++) {
        can_router_process_uart_byte(pkt_diag[i]);
    }

    // 4. Periodic 100ms ticks while sleeping
    can_router_periodic_100ms();

    // Verify CAN bus remains asleep, transceiver stays in standby, and ZERO CAN frames were sent
    can_frame_t sent_frame;
    TEST_ASSERT_TRUE(can_router_is_bus_sleeping());
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_CAN_STBY));
    TEST_ASSERT_FALSE_MESSAGE(hal_can_native_get_last_sent_frame(&sent_frame),
                              "No CAN frames may be transmitted while Comfort CAN bus is sleeping");
}

void test_scenario_acc_power_lifecycle(void) {
    can_router_init();
    vehicle_state_t *state = can_router_get_state();

    const can_frame_t f_ign_off = {
        .id = 0x036, .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x0F, 0x02, 0x00, 0x00, 0xA0 }
    };
    const can_frame_t f_ign_on = {
        .id = 0x036, .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x0F, 0x01, 0x00, 0x00, 0xA0 }
    };
    const can_frame_t f_eco_on = {
        .id = 0x036, .dlc = 8,
        .data = { 0x0E, 0x00, 0x80, 0x0F, 0x02, 0x00, 0x00, 0xA0 }
    };
    const can_frame_t f_rd4_on = {
        .id = 0x165, .dlc = 4,
        .data = { 0xC8, 0x00, 0x11, 0x00 }
    };
    const can_frame_t f_rd4_off = {
        .id = 0x165, .dlc = 4,
        .data = { 0x48, 0xA0, 0x00, 0x00 }
    };
    const can_frame_t f_rd4_sleep = {
        .id = 0x165, .dlc = 4,
        .data = { 0x08, 0x80, 0x00, 0x00 }
    };
    const can_frame_t f_rd4_reboot_00 = {
        .id = 0x165, .dlc = 4,
        .data = { 0x00, 0x00, 0x00, 0x00 }
    };

    const can_frame_t f_0f6_ign_on = {
        .id = 0x0F6, .dlc = 8,
        .data = { 0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20 }
    };
    const can_frame_t f_0f6_ign_off = {
        .id = 0x0F6, .dlc = 8,
        .data = { 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20 }
    };

    // --- Scenario 1: Initial state without RD4 on bus ---
    can_router_process_can(&f_ign_off);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Turn ON ignition without RD4: HU powers ON
    can_router_process_can(&f_0f6_ign_on);
    can_router_process_can(&f_ign_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Turn OFF ignition without RD4: HU powers OFF
    can_router_process_can(&f_0f6_ign_off);
    can_router_process_can(&f_ign_off);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // --- Scenario 2: RD4 present in car ---
    // Start vehicle: IGN ON + RD4 ON
    can_router_process_can(&f_0f6_ign_on);
    can_router_process_can(&f_ign_on);
    can_router_process_can(&f_rd4_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Mid-drive RD4 restart/silence resilience:
    // RD4 sends 00 00 00 00 (reboot) or 08 80 (standby glitch)
    can_router_process_can(&f_rd4_reboot_00);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    can_router_process_can(&f_rd4_sleep);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Radio recovers
    can_router_process_can(&f_rd4_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ON, state->ignition_state);

    // --- Scenario 3: Key turned OFF while driving (IGN goes OFF) ---
    // HU MUST turn OFF immediately even if RD4 has not powered down yet
    can_router_process_can(&f_0f6_ign_off);
    can_router_process_can(&f_ign_off);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_OFF, state->ignition_state, "HU must turn OFF when IGN goes OFF");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Subsequent late RD4 frames from shutdown do NOT re-trigger ACC
    can_router_process_can(&f_rd4_on); // Stale/trailing frame
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // RD4 finishes shutting down
    can_router_process_can(&f_rd4_off);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // --- Scenario 4: Parked, IGN is OFF, user explicitly switches ON RD4 ---
    // Vehicle ground truth: BSI wakes up comfort network (0x036 Phase=0x01) but 0x0F6 is IGN OFF (0x80)
    can_router_process_can(&f_0f6_ign_off);
    can_router_process_can(&f_ign_on); // 0x036 awake
    can_router_process_can(&f_rd4_on);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_ACC, state->ignition_state, "HU must turn ON in ACC when RD4 switched ON with key OFF");
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // User switches OFF RD4 while parked (BSI 0x036 still broadcasting awake Phase=0x01!)
    can_router_process_can(&f_rd4_off);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_OFF, state->ignition_state, "HU must turn OFF when RD4 switched OFF even if 0x036 is awake");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // Periodic 0x036 (Phase=0x01) keeper frames must NOT turn ACC back ON while RD4 is OFF
    can_router_process_can(&f_ign_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_OFF, state->ignition_state);
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // User turns RD4 back ON while parked
    can_router_process_can(&f_rd4_on);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IGNITION_ACC, state->ignition_state);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    // --- Scenario 5: Economy mode triggers while in ACC ---
    can_router_process_can(&f_eco_on);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IGNITION_OFF, state->ignition_state, "Economy mode must cut HU power");
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));
}
