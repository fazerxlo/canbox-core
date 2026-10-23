#include "unity.h"
#include "profiles/peugeot_407.h"
#include "core/can_router.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "protocols/hiworld_psa_builders.h"
#include "protocols/hiworld_connection.h"
#include <string.h>

bool hal_can_native_get_last_sent_frame(can_frame_t *out_frame);
void hal_can_native_clear_sent_frame(void);
size_t hal_can_native_get_sent_history(can_frame_t *out_frames, size_t max_count);

void setUp_peugeot_407_car_config(void) {
    psa_car_config_init();
    hal_can_native_clear_sent_frame();
}

/* Vector 1: Headunit Configures Follow-Me-Home Headlamps to 30 Seconds */
void test_peugeot_407_car_config_vector1_follow_me_home_downlink(void) {
    setUp_peugeot_407_car_config();

    /* 5A A5 02 7B 06 02 84 */
    const uint8_t hu_packet[] = { 0x5A, 0xA5, 0x02, 0x7B, 0x06, 0x02, 0x84 };
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    for (size_t i = 0; i < sizeof(hu_packet); i++) {
        hu_protocol_feed_byte(hu_packet[i]);
    }

    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent.dlc);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]); /* Profile 1 with Bit 2 = 0 */
    TEST_ASSERT_TRUE((sent.data[2] & 0x20) != 0); /* Follow-Me-Home ON */
    TEST_ASSERT_EQUAL_UINT8(0x02, sent.data[2] & 0x03); /* 30 seconds */
}

/* Vector 2: Headunit Enables Daytime Running Lights (DRL = ON) */
void test_peugeot_407_car_config_vector2_drl_downlink(void) {
    setUp_peugeot_407_car_config();

    /* 5A A5 02 7B 05 01 82 */
    const uint8_t hu_packet[] = { 0x5A, 0xA5, 0x02, 0x7B, 0x05, 0x01, 0x82 };
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    for (size_t i = 0; i < sizeof(hu_packet); i++) {
        hu_protocol_feed_byte(hu_packet[i]);
    }

    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent.dlc);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);
    TEST_ASSERT_TRUE((sent.data[3] & 0x40) != 0); /* DRL bit 6 ON */
}

/* Vector 3: Headunit Enables Rear Wiper in Reverse Gear */
void test_peugeot_407_car_config_vector3_rear_wiper_downlink(void) {
    setUp_peugeot_407_car_config();

    /* 5A A5 02 7B 01 01 7E */
    const uint8_t hu_packet[] = { 0x5A, 0xA5, 0x02, 0x7B, 0x01, 0x01, 0x7E };
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    for (size_t i = 0; i < sizeof(hu_packet); i++) {
        hu_protocol_feed_byte(hu_packet[i]);
    }

    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent.dlc);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);
    TEST_ASSERT_TRUE((sent.data[5] & 0x10) != 0); /* Rear wiper bit 4 ON */
}

/* Vector 4: BSI Broadcasts Configuration State (0x260 Settings & 0x361 Availability) */
void test_peugeot_407_car_config_vector4_bsi_to_uplink_0x76(void) {
    setUp_peugeot_407_car_config();

    /* 1. CAN frame from BSI: ID: 0x260, DLC: 8, Data: 01 03 B1 40 00 D0 00 10 (Profile 1 - State A) */
    const uint8_t can_data_260[8] = { 0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10 };
    vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    psa_decode_config_0x260(&cfg, can_data_260, sizeof(can_data_260));

    /* Verify decoded fields */
    TEST_ASSERT_TRUE(cfg.directional_headlights);
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(15, cfg.follow_me_home_sec);
    TEST_ASSERT_TRUE(cfg.auto_lighting_enabled);
    TEST_ASSERT_TRUE(cfg.drl_enabled);
    TEST_ASSERT_FALSE(cfg.auto_door_lock);
    TEST_ASSERT_FALSE(cfg.selective_boot_unlock);
    TEST_ASSERT_TRUE(cfg.motorized_tailgate);
    TEST_ASSERT_TRUE(cfg.parking_radar_enabled);
    TEST_ASSERT_TRUE(cfg.rear_wiper_reverse);
    TEST_ASSERT_EQUAL_UINT8(1, cfg.active_profile_id);

    /* Serialize to Hiworld Cmd 0x76 CentralState1:
     * Byte 0: 0x50 (0x40 Radar Enabled | 0x10 Auto Headlights)
     * Byte 1: 0x85 (0x80 Wiper | 0x04 DRL | 0x01 Follow 15s)
     * CS: (0x02 + 0x76 + 0x50 + 0x85 - 1) & 0xFF = 0x4C */
    uint8_t out[16];
    size_t out_len = build_hiworld_central_state1(&cfg, out, sizeof(out));

    const uint8_t expected_on[7] = { 0x5A, 0xA5, 0x02, 0x76, 0x50, 0x85, 0x4C };
    TEST_ASSERT_EQUAL_UINT32(7, out_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_on, out, 7);

    /* Test 60s Follow-Me-Home encoding format: 01 03 B4 40 00 D0 00 10 (bit 2 = 1 -> 60s) */
    const uint8_t can_data_260_60s[8] = { 0x01, 0x03, 0xB4, 0x40, 0x00, 0xD0, 0x00, 0x10 };
    memset(&cfg, 0, sizeof(cfg));
    psa_decode_config_0x260(&cfg, can_data_260_60s, sizeof(can_data_260_60s));
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(60, cfg.follow_me_home_sec);
    TEST_ASSERT_TRUE(cfg.auto_lighting_enabled);
    out_len = build_hiworld_central_state1(&cfg, out, sizeof(out));
    const uint8_t expected_60s[7] = { 0x5A, 0xA5, 0x02, 0x76, 0x50, 0x87, 0x4E };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_60s, out, 7);

    /* Test 30s Follow-Me-Home encoding format: 01 03 B2 40 00 D0 00 10 (bit 1 = 1 -> 30s) */
    const uint8_t can_data_260_30s[8] = { 0x01, 0x03, 0xB2, 0x40, 0x00, 0xD0, 0x00, 0x10 };
    memset(&cfg, 0, sizeof(cfg));
    psa_decode_config_0x260(&cfg, can_data_260_30s, sizeof(can_data_260_30s));
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(30, cfg.follow_me_home_sec);
    TEST_ASSERT_TRUE(cfg.auto_lighting_enabled);
    out_len = build_hiworld_central_state1(&cfg, out, sizeof(out));
    const uint8_t expected_30s[7] = { 0x5A, 0xA5, 0x02, 0x76, 0x50, 0x86, 0x4D };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_30s, out, 7);

    /* State B: Follow-Me-Home & DRL OFF: 01 03 91 00 00 D0 00 10 */
    const uint8_t can_data_260_off[8] = { 0x01, 0x03, 0x91, 0x00, 0x00, 0xD0, 0x00, 0x10 };
    memset(&cfg, 0, sizeof(cfg));
    psa_decode_config_0x260(&cfg, can_data_260_off, sizeof(can_data_260_off));
    TEST_ASSERT_FALSE(cfg.follow_me_home);
    TEST_ASSERT_FALSE(cfg.drl_enabled);
    TEST_ASSERT_TRUE(cfg.auto_lighting_enabled);
    out_len = build_hiworld_central_state1(&cfg, out, sizeof(out));
    const uint8_t expected_off[7] = { 0x5A, 0xA5, 0x02, 0x76, 0x50, 0x80, 0x47 };
    TEST_ASSERT_EQUAL_UINT32(7, out_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_off, out, 7);

    /* 2. CAN frame from BSI: ID: 0x361, DLC: 6, Data: 01 01 91 40 30 10 (Menu Availability) */
    const uint8_t can_data_361[6] = { 0x01, 0x01, 0x91, 0x40, 0x30, 0x10 };
    psa_decode_config_0x361(can_data_361, sizeof(can_data_361));

    uint8_t feat1[2], feat2[8];
    psa_car_config_get_feature_enables(feat1, feat2);

    /* Cmd 0x71: 0xA7 (0x80 Ambient | 0x20 AAS | 0x04 DRL | 0x03 Follow), 0xA8 (Wiper, AutoLock, SelectiveBoot) */
    TEST_ASSERT_EQUAL_HEX8(0xA7, feat1[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA8, feat1[1]);

    /* Cmd 0x72: 0xC3 (0x80 Hinge | 0x40 Directional | 0x02 TPMS | 0x01 Profile) */
    TEST_ASSERT_EQUAL_HEX8(0xC3, feat2[0]);
    TEST_ASSERT_EQUAL_HEX8(0x04, feat2[1]);
    TEST_ASSERT_EQUAL_HEX8(0x02, feat2[5]);
}

/* Extended Convenience & Personalization Settings Downlink */
void test_peugeot_407_car_config_extended_settings_downlink(void) {
    setUp_peugeot_407_car_config();

    /* Selective Door Unlocking (0x7B setting 0x04) */
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x04, 1));
    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent.dlc);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);
    TEST_ASSERT_TRUE((sent.data[4] & 0x04) != 0);

    /* Ambient Lighting Brightness (0x7B setting 0x0A, level 12 with switch on) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x0A, 0x8C));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x0C, sent.data[2] & 0x0F);

    /* Parking Radar AAS Toggle (0x7B setting 0x0B, mute = 0) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x0B, 0x00));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x00, sent.data[5] & 0x40);

    /* Auto Door Lock while Driving (0x7B setting 0x0C, on = 1) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x0C, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);
    TEST_ASSERT_TRUE((sent.data[4] & 0x08) != 0);

    /* Remote Boot Mode (0x7D setting 0x01) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x01, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_TRUE((sent.data[5] & 0x80) != 0);

    /* Directional Headlights (0x7D setting 0x02) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x02, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_TRUE((sent.data[2] & 0x80) != 0);

    /* TPMS Calibration Pulse (0x7D setting 0x03) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x03, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x221, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x10, sent.data[3]);

    /* Driver Profile Select (0x7D setting 0x0B, profile 2) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x0B, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x02, sent.data[0]);

    /* Zone Temp Sync (0x7D setting 0x24) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x24, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x1D0, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);

    /* Climate Dual/Mono Sync (0x3B setting 0x0F) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x3B, 0x0F, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x1D0, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);
}

/* CentralState2 Hiworld Serial Builder */
void test_peugeot_407_car_config_central_state2_builder(void) {
    vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.motorized_tailgate = true;
    cfg.directional_headlights = true;
    cfg.active_profile_id = 2;
    cfg.cluster_dimmer = 10;

    vehicle_state_t st;
    memset(&st, 0, sizeof(st));
    st.climate.dual_mode = false; /* Mono mode active */

    uint8_t out[16];
    size_t len = build_hiworld_central_state2(&cfg, &st, out, sizeof(out));

    TEST_ASSERT_EQUAL_UINT32(13, len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x79, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[4]); /* 0x80 (tailgate) | 0x40 (directional) */
    TEST_ASSERT_EQUAL_HEX8(0x20, out[5]); /* Profile > 1 -> 0x20 */
    TEST_ASSERT_EQUAL_HEX8(0x28, out[6]); /* Dimmer 10 << 2 = 40 = 0x28 */
    TEST_ASSERT_EQUAL_HEX8(0x10, out[7]); /* PA12.20 status byte 0x10 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[10]); /* Mono mode -> 0x02 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[11]); /* Reserved */

    /* Verify checksum */
    uint8_t sum = 0;
    for (size_t i = 2; i < 12; i++) sum = (uint8_t)(sum + out[i]);
    uint8_t expected_cs = (uint8_t)((sum - 1) & 0xFF);
    TEST_ASSERT_EQUAL_HEX8(expected_cs, out[12]);
}

/* Boundary & Safe Guard Tests */
void test_peugeot_407_car_config_boundary_and_guards(void) {
    setUp_peugeot_407_car_config();

    vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* NULL pointers and short DLC */
    psa_decode_config_0x361(NULL, 6);
    psa_decode_config_0x361((const uint8_t *)"12345", 5);

    psa_decode_config_0x260(NULL, (const uint8_t *)"12345678", 8);
    psa_decode_config_0x260(&cfg, NULL, 8);
    psa_decode_config_0x260(&cfg, (const uint8_t *)"1234567", 7);
    TEST_ASSERT_FALSE(cfg.follow_me_home);

    /* Non-profile 1 frame in 0x260 should be ignored */
    const uint8_t frame_prof2[8] = { 0x02, 0x03, 0x92, 0xC0, 0x00, 0xD0, 0x00, 0x10 };
    psa_decode_config_0x260(&cfg, frame_prof2, 8);
    TEST_ASSERT_FALSE(cfg.follow_me_home);

    /* Invalid setting ID in apply downlink */
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_ERROR, psa_car_config_apply_downlink(0x7B, 0xFF, 1));
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_ERROR, psa_car_config_apply_downlink(0x7D, 0xFF, 1));
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_ERROR, psa_car_config_apply_downlink(0x99, 0x01, 1));

    /* Small buffer for builders */
    uint8_t small_buf[4];
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_central_state1(&cfg, small_buf, sizeof(small_buf)));
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_central_state2(&cfg, NULL, small_buf, sizeof(small_buf)));
}

void test_peugeot_407_car_config_disconnected_mdf_authority(void) {
    setUp_peugeot_407_car_config();
    can_router_init();

    /* 1. Initial boot: receive 0x260 from BSI with Follow-Me-Home 15s (0xB1) */
    can_frame_t frame_bsi;
    memset(&frame_bsi, 0, sizeof(frame_bsi));
    frame_bsi.id  = 0x260;
    frame_bsi.dlc = 8;
    const uint8_t bsi_data_15s[8] = { 0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10 };
    memcpy(frame_bsi.data, bsi_data_15s, 8);

    can_router_process_can(&frame_bsi);
    TEST_ASSERT_TRUE(psa_car_config_is_cached_valid());
    TEST_ASSERT_FALSE(psa_car_config_is_mdf_present());

    vehicle_config_t cfg;
    psa_car_config_get_cached(&cfg);
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(15, cfg.follow_me_home_sec);

    /* 2. HU Downlink: User selects 30 seconds (Cmd 0x7B setting 0x06 val 2) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_TRUE(can_router_set_car_config(0x7B, 0x06, 0x02));

    psa_car_config_get_cached(&cfg);
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(30, cfg.follow_me_home_sec);

    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x02, sent.data[2] & 0x07); /* One-hot bit 1 = 30s */

    /* 3. Static BSI 0x260 EEPROM frame arrives (still containing 0xB1 / 15s) while MDF is disconnected */
    can_router_process_can(&frame_bsi);

    /* Assert that standalone authority preserved the user's 30s setting without reverting */
    psa_car_config_get_cached(&cfg);
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(30, cfg.follow_me_home_sec);

    /* 4. HU Downlink: User selects 60 seconds (val 3) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_TRUE(can_router_set_car_config(0x7B, 0x06, 0x03));

    psa_car_config_get_cached(&cfg);
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(60, cfg.follow_me_home_sec);

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x15B, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x04, sent.data[2] & 0x07); /* One-hot bit 2 = 60s */

    /* 5. Simulate MDF reconnect on workbench (0x525 arrives) */
    can_frame_t frame_mdf;
    memset(&frame_mdf, 0, sizeof(frame_mdf));
    frame_mdf.id  = PSA_CAN_ID_MDF_DIAG_525;
    frame_mdf.dlc = 8;
    frame_mdf.data[0] = 0x00;
    frame_mdf.data[1] = 0x88;
    frame_mdf.data[2] = 0x40;
    frame_mdf.data[5] = 0x75;
    frame_mdf.data[6] = 0x28;
    frame_mdf.data[7] = 0x33;
    can_router_process_can(&frame_mdf);

    TEST_ASSERT_TRUE(psa_car_config_is_mdf_present());

    /* With physical MDF present, BSI 0x260 update is authoritative */
    can_router_process_can(&frame_bsi);
    psa_car_config_get_cached(&cfg);
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(15, cfg.follow_me_home_sec);
}

void test_peugeot_407_virtual_mdf_periodic_broadcast(void) {
    setUp_peugeot_407_car_config();

    TEST_ASSERT_FALSE(psa_car_config_is_mdf_present());

    /* Call periodic 100ms 9 times: no broadcast yet */
    for (int i = 0; i < 9; i++) {
        psa_car_config_periodic_100ms();
    }
    can_frame_t sent;
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&sent));

    /* 10th tick (1000 ms): 0x3F6, 0x0DF emitted (0x167 is strictly event-driven) */
    hal_can_native_clear_sent_frame();
    psa_car_config_periodic_100ms();

    can_frame_t history[4];
    size_t count = hal_can_native_get_sent_history(history, 4);
    TEST_ASSERT_EQUAL_UINT32(2, count);

    /* Frame 1: 0x3F6 (DLC 7): 00 00 00 00 00 C0 01 */
    TEST_ASSERT_EQUAL_HEX32(0x3F6, history[0].id);
    TEST_ASSERT_EQUAL_UINT8(7, history[0].dlc);
    const uint8_t exp_3f6[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_3f6, history[0].data, 7);

    /* Frame 2: 0x0DF (DLC 3): 10 00 50 */
    TEST_ASSERT_EQUAL_HEX32(0x0DF, history[1].id);
    TEST_ASSERT_EQUAL_UINT8(3, history[1].dlc);
    const uint8_t exp_0df[] = { 0x10, 0x00, 0x50 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_0df, history[1].data, 3);
}

void test_peugeot_407_mdf_coexistence_suppression(void) {
    setUp_peugeot_407_car_config();

    /* Physical OEM MDF transmits 0x525 */
    can_frame_t frame_525;
    memset(&frame_525, 0, sizeof(frame_525));
    frame_525.id  = PSA_CAN_ID_MDF_DIAG_525;
    frame_525.dlc = 8;
    frame_525.data[5] = 0x75;
    frame_525.data[6] = 0x28;
    frame_525.data[7] = 0x33;
    can_router_process_can(&frame_525);

    TEST_ASSERT_TRUE(psa_car_config_is_mdf_present());

    /* 10 periodic ticks: verify NOTHING is transmitted on CAN */
    hal_can_native_clear_sent_frame();
    for (int i = 0; i < 10; i++) {
        psa_car_config_periodic_100ms();
    }
    can_frame_t sent;
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&sent));
}

void test_peugeot_407_matrix_cluster_page_cycling(void) {
    setUp_peugeot_407_car_config();
    can_router_init();

    can_frame_t sent;

    /* 1. Inject 0x221 press: C8 FF FF FF FF FF FF */
    hal_can_native_clear_sent_frame();
    can_frame_t f_press = { .id = 0x221, .dlc = 7, .data = { 0xC8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };
    can_router_process_can(&f_press);

    /* 2. Inject 0x221 release: C0 FF FF FF FF FF FF */
    hal_can_native_clear_sent_frame();
    can_frame_t f_rel = { .id = 0x221, .dlc = 7, .data = { 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };
    can_router_process_can(&f_rel);

    /* Verify 0x167 is immediately transmitted with 02 00 FF FF 00 00 00 00 (Trip 1) */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent.dlc);
    const uint8_t exp_p1[] = { 0x02, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_p1, sent.data, 8);
    TEST_ASSERT_EQUAL_UINT8(0x02, psa_car_config_get_trip_page_mask());

    /* 3. Repeat press/release -> advances to 04 00 FF FF 00 00 00 00 (Trip 2) */
    can_router_process_can(&f_press);
    hal_can_native_clear_sent_frame();
    can_router_process_can(&f_rel);

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent.id);
    const uint8_t exp_p2[] = { 0x04, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_p2, sent.data, 8);
    TEST_ASSERT_EQUAL_UINT8(0x04, psa_car_config_get_trip_page_mask());

    /* 4. Repeat press/release -> wraps to 01 00 FF FF 00 00 00 00 (Instant) */
    can_router_process_can(&f_press);
    hal_can_native_clear_sent_frame();
    can_router_process_can(&f_rel);

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent.id);
    const uint8_t exp_p0[] = { 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_p0, sent.data, 8);
    TEST_ASSERT_EQUAL_UINT8(0x01, psa_car_config_get_trip_page_mask());
}

void test_peugeot_407_trip_reset_0x167(void) {
    setUp_peugeot_407_car_config();

    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL(HAL_STATUS_OK, psa_trip_send_reset(1));

    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x167, sent.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x80, sent.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, sent.data[3]);
}
