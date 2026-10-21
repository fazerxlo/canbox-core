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
    TEST_ASSERT_EQUAL_HEX32(0x361, sent.id);
    TEST_ASSERT_EQUAL_UINT8(6, sent.dlc);

    /* Expected: 80 02 00 00 01 00 (Byte 0: 0x80, Byte 1: 0x02) */
    const uint8_t expected[6] = { 0x80, 0x02, 0x00, 0x00, 0x01, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, sent.data, 6);
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
    TEST_ASSERT_EQUAL_HEX32(0x361, sent.id);
    TEST_ASSERT_EQUAL_UINT8(6, sent.dlc);

    /* Expected: 40 00 00 00 01 00 (Byte 0: 0x40) */
    const uint8_t expected[6] = { 0x40, 0x00, 0x00, 0x00, 0x01, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, sent.data, 6);
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
    TEST_ASSERT_EQUAL_HEX32(0x361, sent.id);
    TEST_ASSERT_EQUAL_UINT8(6, sent.dlc);

    /* Expected: 20 00 00 00 01 00 (Byte 0: 0x20) */
    const uint8_t expected[6] = { 0x20, 0x00, 0x00, 0x00, 0x01, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, sent.data, 6);
}

/* Vector 4: BSI Broadcasts Configuration State -> CAN Box Uplinks Telemetry */
void test_peugeot_407_car_config_vector4_bsi_to_uplink_0x76(void) {
    setUp_peugeot_407_car_config();

    /* CAN frame from BSI: ID: 0x361, DLC: 6, Data: E8 02 0A 00 01 00 */
    const uint8_t can_data[6] = { 0xE8, 0x02, 0x0A, 0x00, 0x01, 0x00 };
    vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    psa_decode_config_0x361(&cfg, can_data, sizeof(can_data));

    /* Verify decoded fields */
    TEST_ASSERT_TRUE(cfg.follow_me_home);
    TEST_ASSERT_EQUAL_UINT8(30, cfg.follow_me_home_sec);
    TEST_ASSERT_TRUE(cfg.drl_enabled);
    TEST_ASSERT_TRUE(cfg.rear_wiper_reverse);
    TEST_ASSERT_TRUE(cfg.auto_door_lock);
    TEST_ASSERT_FALSE(cfg.selective_boot_unlock);
    TEST_ASSERT_EQUAL_UINT8(10, cfg.ambient_lighting_level);
    TEST_ASSERT_TRUE(cfg.parking_radar_enabled);

    /* Serialize to Hiworld Cmd 0x76 CentralState1:
     * Byte 0: 0x4D (0x40 Radar | 0x08 Ambient On | 0x05 Brightness 10/2)
     * Byte 1: 0xA6 (0x80 Wiper | 0x20 Auto Lock | 0x04 DRL | 0x02 Follow 30s)
     * CS: (0x02 + 0x76 + 0x4D + 0xA6 - 1) & 0xFF = 0x6A */
    uint8_t out[16];
    size_t out_len = build_hiworld_central_state1(&cfg, out, sizeof(out));

    const uint8_t expected[7] = { 0x5A, 0xA5, 0x02, 0x76, 0x4D, 0xA6, 0x6A };
    TEST_ASSERT_EQUAL_UINT32(7, out_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, 7);
}

/* Extended Convenience & Personalization Settings Downlink */
void test_peugeot_407_car_config_extended_settings_downlink(void) {
    setUp_peugeot_407_car_config();

    /* Selective Door Unlocking (0x7B setting 0x04) */
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x04, 1));
    can_frame_t sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_HEX32(0x361, sent.id);
    TEST_ASSERT_TRUE((sent.data[0] & 0x04) != 0);

    /* Ambient Lighting Brightness (0x7B setting 0x0A, level 12 with switch on) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x0A, 0x8C));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_UINT8(0x0C, sent.data[2] & 0x0F);

    /* Parking Radar AAS Toggle (0x7B setting 0x0B, mute = 0) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x0B, 0x00));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_EQUAL_UINT8(0x00, sent.data[4]);

    /* Auto Door Lock while Driving (0x7B setting 0x0C, on = 1) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7B, 0x0C, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_TRUE((sent.data[0] & 0x08) != 0);

    /* Remote Boot Mode (0x7D setting 0x01) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x01, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_TRUE((sent.data[0] & 0x01) != 0);

    /* Directional Headlights (0x7D setting 0x02) */
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_EQUAL_INT(HAL_STATUS_OK, psa_car_config_apply_downlink(0x7D, 0x02, 0x01));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent));
    TEST_ASSERT_TRUE((sent.data[0] & 0x02) != 0);

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
    TEST_ASSERT_EQUAL_HEX32(0x260, sent.id);
    TEST_ASSERT_EQUAL_UINT8(0x02, sent.data[3]);

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

    TEST_ASSERT_EQUAL_UINT32(12, len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x07, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x79, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[4]); /* 0x80 (tailgate) | 0x40 (directional) */
    TEST_ASSERT_EQUAL_HEX8(0x20, out[5]); /* Profile > 1 -> 0x20 */
    TEST_ASSERT_EQUAL_HEX8(0x28, out[6]); /* Dimmer 10 << 2 = 40 = 0x28 */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[7]); /* ESP active */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[10]); /* Mono mode -> 0x02 */

    /* Verify checksum */
    uint8_t sum = 0;
    for (size_t i = 2; i < 11; i++) sum += out[i];
    uint8_t expected_cs = (uint8_t)((sum - 1) & 0xFF);
    TEST_ASSERT_EQUAL_HEX8(expected_cs, out[11]);
}

/* Boundary & Safe Guard Tests */
void test_peugeot_407_car_config_boundary_and_guards(void) {
    setUp_peugeot_407_car_config();

    vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* NULL pointers and short DLC */
    psa_decode_config_0x361(NULL, (const uint8_t *)"123456", 6);
    psa_decode_config_0x361(&cfg, NULL, 6);
    psa_decode_config_0x361(&cfg, (const uint8_t *)"12345", 5);
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
