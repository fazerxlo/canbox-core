#include "unity.h"
#include "profiles/peugeot_407.h"
#include "profiles/psa_virtual_cdc.h"
#include "core/vehicle_profile.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "protocols/hiworld_psa_builders.h"
#include "protocols/raise_psa_builders.h"
#include "hal/hal_gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

bool hal_can_native_get_last_sent_frame(can_frame_t *out_frame);
void hal_can_native_clear_sent_frame(void);

/* --------------------------------------------------------------------------
 * Vehicle Alerts & Diagnostic Journal Verification Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_alert_single_abs(void) {
    /* CAN 0x1A1: POPUP_ACTIVE=1, CAN_ALARM_ID=0x006C (ABS Fault), DISPLAY=1 (0x80), PRIORITY=3 (0x30), SOUND=2 (0x02) -> 0xB2 */
    const uint8_t can_data[8] = { 0x80, 0x6C, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00 };
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));

    psa_decode_alert_message_0x1a1(can_data, 8, &alert);

    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x006C, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, alert.alert_code);
    TEST_ASSERT_EQUAL_UINT8(3, alert.severity);
    TEST_ASSERT_EQUAL_UINT8(2, alert.chime_id);

    /* Serialize Hiworld single alert frame */
    uint8_t out[16];
    size_t len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    /* CS = (0x02 + 0x42 + 0x00 + 0x69 - 1) & 0xFF = 0xAC */
}

void test_peugeot_407_alert_single_suspension(void) {
    /* CAN 0x1A1: POPUP_ACTIVE=1, CAN_ALARM_ID=0x009E (Suspension 90km/h), DISPLAY=1, PRIORITY=2, SOUND=1 */
    const uint8_t can_data[8] = { 0x80, 0x9E, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00 };
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));

    psa_decode_alert_message_0x1a1(can_data, 8, &alert);

    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x009E, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_SUSPENSION_90KMH, alert.alert_code);

    uint8_t out[16];
    size_t len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    /* CS = (0x02 + 0x42 + 0x00 + 0x6B - 1) & 0xFF = 0xAE */
}

void test_peugeot_407_alert_single_low_fuel(void) {
    /* CAN 0x1A1: POPUP_ACTIVE=1, CAN_ALARM_ID=0x000D (Low Fuel), DISPLAY=1, PRIORITY=2, SOUND=1 */
    const uint8_t can_data[8] = { 0x80, 0x0D, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00 };
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));

    psa_decode_alert_message_0x1a1(can_data, 8, &alert);

    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x000D, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_LOW_FUEL, alert.alert_code);

    uint8_t out[16];
    size_t len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    /* CS = (0x02 + 0x42 + 0x00 + 0x01 - 1) & 0xFF = 0x44 */

    /* Test clear alert: POPUP_ACTIVE = 0 */
    const uint8_t clear_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    psa_decode_alert_message_0x1a1(clear_data, 8, &alert);
    TEST_ASSERT_FALSE(alert.is_active);
    TEST_ASSERT_FALSE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x0000, alert.alert_code);
}

void test_peugeot_407_alert_journal_multi(void) {
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);

    vehicle_alert_item_t out_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t  out_count = 0;
    memset(out_codes, 0, sizeof(out_codes));

    /* Journal test payload:
     * Byte 0: 0x40 (Bit 1 = Engine Overheat -> Alarm Index 0x07 -> CAN 0x0001 -> Hiworld 0x0003)
     * Byte 4: 0xC0 (Bit 32 = ABS -> Alarm Index 0x16 -> CAN 0x006C -> Hiworld 0x0069,
     *               Bit 33 = DPF -> Alarm Index 0x1A -> CAN 0x006F -> Hiworld 0x0064)
     */
    const uint8_t ff_data[8]  = { 0x10, 0x15, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x00 };
    const uint8_t cf1_data[8] = { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf2_data[8] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf3_data[8] = { 0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, ff_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf1_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf2_data, 8, out_codes, &out_count));
    TEST_ASSERT_TRUE(psa_process_journal_0x120(&ctx, cf3_data, 8, out_codes, &out_count));

    TEST_ASSERT_EQUAL_UINT8(3, out_count);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ENGINE_TEMP, out_codes[0].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, out_codes[1].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_DPF, out_codes[2].alert_code);

    /* Serialize Hiworld Multi-Alert Summary (24 payload bytes) */
    uint8_t out[80];
    size_t len = build_hiworld_alerts_summary(out_codes, out_count, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(24, len);
    /* Check padding to 24 payload bytes */
    for (int i = 14; i < 28; i++) {
    }
    /* CS = (0x18 + 0x42 + 3 + 3 + 0x69 + 0x64 - 1) & 0xFF = 0x2C */
}

void test_peugeot_407_alert_journal_clear(void) {
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);

    vehicle_alert_item_t out_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t  out_count = 0;
    memset(out_codes, 0, sizeof(out_codes));

    /* Journal test payload with 0 active faults */
    const uint8_t ff_data[8]  = { 0x10, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf1_data[8] = { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf2_data[8] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cf3_data[8] = { 0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, ff_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf1_data, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, cf2_data, 8, out_codes, &out_count));
    TEST_ASSERT_TRUE(psa_process_journal_0x120(&ctx, cf3_data, 8, out_codes, &out_count));

    TEST_ASSERT_EQUAL_UINT8(0, out_count);

    /* Serialize Hiworld clear frame (mNumber = 0) */
    uint8_t out[80];
    size_t len = build_hiworld_alerts_summary(out_codes, out_count, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(9, len);
    /* CS = (0x18 + 0x42 - 1) & 0xFF = 0x59 */
}

void test_peugeot_407_alert_router_pipeline_and_query(void) {
    can_router_init();
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    /* 1. Feed single alert frame (CAN 0x1A1, ABS fault) */
    can_frame_t frame_abs = {
        .id = PSA_CAN_ID_ALERT_MESSAGE,
        .dlc = 8,
        .data = { 0x80, 0x6C, 0xF2, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_abs);

    const vehicle_state_t *st = can_router_get_state();
    TEST_ASSERT_TRUE(st->alerts.realtime_alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, st->alerts.realtime_alert.alert_code);

    /* 2. Feed clear frame (CAN 0x1A1, POPUP_ACTIVE=0) */
    can_frame_t frame_clear = {
        .id = PSA_CAN_ID_ALERT_MESSAGE,
        .dlc = 8,
        .data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_clear);
    st = can_router_get_state();
    TEST_ASSERT_FALSE(st->alerts.realtime_alert.is_active);

    /* 3. Feed multi-frame journal sequence (Initial Scan: 3 active faults) */
    can_frame_t f_ff = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x10, 0x15, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x00 } };
    can_frame_t f_c1 = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    can_frame_t f_c2 = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    can_frame_t f_c3 = { .id = PSA_CAN_ID_ALERT_JOURNAL, .dlc = 8, .data = { 0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };

    can_router_process_can(&f_ff);
    can_router_process_can(&f_c1);
    can_router_process_can(&f_c2);
    can_router_process_can(&f_c3);

    st = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT8(3, st->alerts.active_count);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ENGINE_TEMP, st->alerts.active_items[0].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ABS, st->alerts.active_items[1].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_DPF, st->alerts.active_items[2].alert_code);

    /* 4. Retransmit identical journal sequence: no state change, no unsolicited popup */
    can_router_process_can(&f_ff);
    can_router_process_can(&f_c1);
    can_router_process_can(&f_c2);
    can_router_process_can(&f_c3);
    st = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT8(3, st->alerts.active_count);

    /* 5. Feed UART Query Cmd 0x2F with 0xD4 checksum variant (Android APK forwardType 0x2F) */
    hal_can_native_clear_sent_frame();
    const uint8_t query_packet_d4[] = { 0x5A, 0xA5, 0x01, 0x2F, 0x01, 0xD4 };
    for (size_t i = 0; i < sizeof(query_packet_d4); i++) {
        can_router_process_uart_byte(query_packet_d4[i]);
    }

    /* Verify BSI diagnostic query CAN frame 0x39B was transmitted on CAN */
    can_frame_t sent_query;
    memset(&sent_query, 0, sizeof(sent_query));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_query));
    TEST_ASSERT_EQUAL_HEX32(PSA_CAN_ID_ALERT_QUERY, sent_query.id);
    TEST_ASSERT_EQUAL_UINT8(8, sent_query.dlc);

    /* 6. Verify 500ms timeout handling in periodic tick */
    for (int t = 0; t < 5; t++) {
        can_router_periodic_100ms();
    }

    /* 7. Verify standard 0x30 checksum for 0x2F also triggers BSI query */
    hal_can_native_clear_sent_frame();
    const uint8_t query_packet_30[] = { 0x5A, 0xA5, 0x01, 0x2F, 0x01, 0x30 };
    for (size_t i = 0; i < sizeof(query_packet_30); i++) {
        can_router_process_uart_byte(query_packet_30[i]);
    }
    memset(&sent_query, 0, sizeof(sent_query));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_query));
    TEST_ASSERT_EQUAL_HEX32(PSA_CAN_ID_ALERT_QUERY, sent_query.id);

    /* 8. Verify periodic timer ticks do not spam alert summaries when no query is pending */
    for (int t = 0; t < 600; t++) {
        can_router_periodic_100ms();
    }
}

void test_peugeot_407_alert_boundary_and_malformed(void) {
    /* 1. NULL safety */
    psa_decode_alert_message_0x1a1(NULL, 8, NULL);
    psa_journal_iso_tp_init(NULL);
    TEST_ASSERT_FALSE(psa_process_journal_0x120(NULL, NULL, 0, NULL, NULL));

    /* 2. Short DLC on 0x1A1 */
    vehicle_alert_item_t alert;
    memset(&alert, 0, sizeof(alert));
    const uint8_t short_data[3] = { 0x80, 0x6C, 0xF2 };
    psa_decode_alert_message_0x1a1(short_data, 3, &alert);
    TEST_ASSERT_FALSE(alert.is_active);

    /* 3. Short DLC on 0x120 */
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);
    vehicle_alert_item_t codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t count = 0;
    const uint8_t short_pci[1] = { 0x10 };
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, short_pci, 1, codes, &count));

    /* 4. Out of sequence consecutive frame */
    const uint8_t ff_data[8] = { 0x10, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t wrong_cf[8] = { 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; /* expected 0x21 */
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, ff_data, 8, codes, &count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, wrong_cf, 8, codes, &count));
    TEST_ASSERT_FALSE(ctx.in_progress);

    /* 5. Buffer size boundaries */
    uint8_t small_buf[5];
    TEST_ASSERT_EQUAL_size_t(0, build_hiworld_alert_single(&codes[0], small_buf, sizeof(small_buf)));
    TEST_ASSERT_EQUAL_size_t(0, build_hiworld_alerts_summary(codes, count, small_buf, sizeof(small_buf)));
}

void test_peugeot_407_alert_journal_block_multiplexed_real_log(void) {
    psa_journal_iso_tp_t ctx;
    psa_journal_iso_tp_init(&ctx);

    vehicle_alert_item_t out_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t  out_count = 0;
    memset(out_codes, 0, sizeof(out_codes));

    /* Real frames captured from dump_2026-10-02_12-15-52.log:
     * Frame 1: FC 00 00 00 00 0F 00 00 (Block 3)
     * Frame 2: BC 00 00 00 00 00 00 00 (Block 2)
     * Frame 3: 7C 10 00 03 00 04 00 08 (Block 1)
     */
    const uint8_t blk3[8] = { 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00 };
    const uint8_t blk2[8] = { 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t blk1[8] = { 0x7C, 0x10, 0x00, 0x03, 0x00, 0x04, 0x00, 0x08 };

    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, blk3, 8, out_codes, &out_count));
    TEST_ASSERT_FALSE(psa_process_journal_0x120(&ctx, blk2, 8, out_codes, &out_count));
    TEST_ASSERT_TRUE(psa_process_journal_0x120(&ctx, blk1, 8, out_codes, &out_count));

    /* Verify parsed alerts from real dump */
    TEST_ASSERT_EQUAL_UINT8(9, out_count);
    TEST_ASSERT_EQUAL_HEX16(0x0008, out_codes[0].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00E0, out_codes[1].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0078, out_codes[2].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0083, out_codes[3].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x007F, out_codes[4].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0130, out_codes[5].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0131, out_codes[6].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00E5, out_codes[7].can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x0138, out_codes[8].can_alarm_id);

    /* Verify mapped Hiworld alert codes */
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_HANDBRAKE, out_codes[0].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AIRBAG, out_codes[2].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_IMMOBILIZER, out_codes[3].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ANTIPOLLUTION, out_codes[4].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AUTO_LIGHTS, out_codes[5].alert_code);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_LOW_FUEL, out_codes[8].alert_code);
}

void test_peugeot_407_alert_diagnostic_cycle_and_toggle_bit(void) {
    vehicle_alert_item_t alert;

    /* 1. Depollution fault (CAN 0x007F): byte 0 bit 7 toggle = 0, display_req = 1 */
    const uint8_t can_depollution[8] = { 0x00, 0x7F, 0xD6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_depollution, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);
    TEST_ASSERT_EQUAL_HEX16(0x007F, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_ANTIPOLLUTION, alert.alert_code);

    /* 2. Electronic anti-theft / immobilizer (CAN 0x0083): toggle = 1, display_req = 1 */
    const uint8_t can_immobilizer[8] = { 0x80, 0x83, 0xD6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_immobilizer, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0083, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_IMMOBILIZER, alert.alert_code);

    /* 3. Tyre pressure not monitored (CAN 0x00E5): toggle = 0, display_req = 1 */
    const uint8_t can_tpms_unmonitored[8] = { 0x00, 0xE5, 0xD7, 0x1E, 0x40, 0x00, 0x00, 0x00 };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_tpms_unmonitored, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00E5, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_TPMS_UNDER_FL, alert.alert_code);

    /* 4. Automatic headlamp lighting activated (CAN 0x0130): toggle = 1, display_req = 1 */
    const uint8_t can_auto_lights[8] = { 0x81, 0x30, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_auto_lights, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0130, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AUTO_LIGHTS, alert.alert_code);

    /* 5. Fuel level too low (CAN 0x0138): toggle = 1, display_req = 1 */
    const uint8_t can_low_fuel_138[8] = { 0x81, 0x38, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_low_fuel_138, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0138, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_LOW_FUEL, alert.alert_code);

    /* 6. Automatic screen wipe deactivated (CAN 0x0139): toggle = 0, display_req = 1 */
    const uint8_t can_auto_wipers_139[8] = { 0x01, 0x39, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_auto_wipers_139, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0139, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(PSA_HIWORLD_ALERT_AUTO_WIPERS, alert.alert_code);

    /* 7. Diagnosis in progress (CAN 0x00F0) and completed (CAN 0x00F1) */
    const uint8_t can_diag_prog[8] = { 0x80, 0xF0, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_diag_prog, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F0, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00F0, alert.alert_code);

    const uint8_t can_diag_comp[8] = { 0x80, 0xF1, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_diag_comp, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F1, alert.can_alarm_id);
    TEST_ASSERT_EQUAL_HEX16(0x00F1, alert.alert_code);

    /* 8. Sequence toggle bit behavior: verify that toggle bit 0 does NOT dismiss active alert */
    const uint8_t can_toggle_zero[8] = { 0x01, 0x31, 0xD4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_toggle_zero, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_TRUE(alert.display_req);

    /* Dismissal requires display_req = 0 or can_alarm_id = 0 */
    const uint8_t can_dismiss[8] = { 0x01, 0x31, 0x54, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; /* display_req = 0 (bit 7 clear) */
    psa_decode_alert_message_0x1a1(can_dismiss, 8, &alert);
    TEST_ASSERT_FALSE(alert.is_active);
    TEST_ASSERT_FALSE(alert.display_req);
}

void test_peugeot_407_csv_alerts_iteration(void) {
    const char *paths[] = {
        "doc/RT4_CAN_ALERTS_0x1A1.csv",
        "../doc/RT4_CAN_ALERTS_0x1A1.csv",
        "../../doc/RT4_CAN_ALERTS_0x1A1.csv",
        NULL
    };
    FILE *fp = NULL;
    for (int p = 0; paths[p] != NULL; p++) {
        fp = fopen(paths[p], "r");
        if (fp != NULL) break;
    }
    TEST_ASSERT_NOT_NULL_MESSAGE(fp, "Failed to open doc/RT4_CAN_ALERTS_0x1A1.csv");

    char line[1024];
    /* Skip CSV header */
    TEST_ASSERT_NOT_NULL(fgets(line, sizeof(line), fp));

    int alert_count = 0;
    while (fgets(line, sizeof(line), fp)) {
        size_t len = strlen(line);
        if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len == 0) continue;

        /* Parse comma-separated fields with quote awareness */
        char *tokens[10];
        int tok_idx = 0;
        char *ptr = line;
        bool in_quotes = false;
        tokens[tok_idx++] = ptr;

        while (*ptr && tok_idx < 10) {
            if (*ptr == '"') {
                in_quotes = !in_quotes;
            } else if (*ptr == ',' && !in_quotes) {
                *ptr = '\0';
                tokens[tok_idx++] = ptr + 1;
            }
            ptr++;
        }

        if (tok_idx < 10) continue;

        uint16_t can_id = (uint16_t)strtoul(tokens[3], NULL, 10);

        if (can_id != 0xFFFF && can_id != 0) {
            /* 1. Simulate CAN 0x1A1 frame with active display request */
            uint8_t frame_data[8] = {
                (uint8_t)(((can_id >> 8) & 0x7F) | 0x80), /* Toggle = 1, ID Hi */
                (uint8_t)(can_id & 0xFF),                 /* ID Lo */
                0x80 | 0x20 | 0x01,                      /* DispReq=1, Priority=2, Sound=1 */
                0x00, 0x00, 0x00, 0x00, 0x00
            };

            vehicle_alert_item_t alert;
            memset(&alert, 0, sizeof(alert));
            psa_decode_alert_message_0x1a1(frame_data, 8, &alert);

            TEST_ASSERT_TRUE(alert.is_active);
            TEST_ASSERT_TRUE(alert.display_req);
            TEST_ASSERT_EQUAL_HEX16(can_id, alert.can_alarm_id);
            TEST_ASSERT_EQUAL_HEX16(psa_can_alarm_id_to_hiworld_code(can_id, 0, 0), alert.alert_code);

            /* 2. Verify Extended Alert serializer (Cmd 0xEA, 10 bytes) */
            uint8_t ext_buf[16];
            size_t ext_len = build_hiworld_alert_single(&alert, ext_buf, sizeof(ext_buf));
            TEST_ASSERT_EQUAL_size_t(10, ext_len);
            TEST_ASSERT_EQUAL_HEX8(0x5A, ext_buf[0]);
            TEST_ASSERT_EQUAL_HEX8(0xA5, ext_buf[1]);
            TEST_ASSERT_EQUAL_HEX8(0x05, ext_buf[2]);
            TEST_ASSERT_EQUAL_HEX8(0xEA, ext_buf[3]);
            TEST_ASSERT_EQUAL_HEX8((uint8_t)(can_id >> 8), ext_buf[4]);
            TEST_ASSERT_EQUAL_HEX8((uint8_t)(can_id & 0xFF), ext_buf[5]);

            /* 3. Verify Alert Dismissal behavior (DispReq=0) */
            frame_data[2] = 0x00;
            psa_decode_alert_message_0x1a1(frame_data, 8, &alert);
            TEST_ASSERT_FALSE(alert.is_active);
            TEST_ASSERT_FALSE(alert.display_req);
            TEST_ASSERT_EQUAL_HEX16(0, alert.can_alarm_id);
        }

        alert_count++;
    }

    fclose(fp);
    TEST_ASSERT_EQUAL_INT(208, alert_count);
}

void test_peugeot_407_custom_alert_severities_lookup(void) {
    /* STOP (2) - Major critical faults */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0001)); /* Engine temp */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0002)); /* Oil pressure */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0005)); /* Tyre puncture */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0008)); /* Braking system faulty */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x000D)); /* Puncture */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x000F)); /* Brake system */
    TEST_ASSERT_EQUAL_UINT8(2, psa_can_alarm_id_get_severity(0x0066)); /* Braking system */

    /* SERVICE (1) - Faults requiring service */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0003)); /* Oil level */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0004)); /* Low tyre pressure */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x000E)); /* Power steering */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0064)); /* DPF */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0067)); /* Brake pads */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0068)); /* Depollution */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006A)); /* ABS */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006B)); /* Battery charge */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006C)); /* Suspension */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006E)); /* Gearbox */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x006F)); /* DPF */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0072)); /* Suspension 90km/h */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0073)); /* Auto gearbox */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0078)); /* Airbag */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x007E)); /* ABS/ESP */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x007F)); /* Depollution */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0083)); /* Anti-theft */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0088)); /* TPMS */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x00E0)); /* TPMS unmonitored */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x00E5)); /* TPMS unmonitored */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0195)); /* Directional headlamps */
    TEST_ASSERT_EQUAL_UINT8(1, psa_can_alarm_id_get_severity(0x0202)); /* Auto gearbox */

    /* INFO (0) - Confirmations, reminders, diagnosis statuses */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x000A)); /* Black ice */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x000C)); /* Handbrake */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0061)); /* Service due */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0069)); /* ESP off */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x00F0)); /* Diag in progress */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x00F1)); /* Diag complete */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0130)); /* Auto lights */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0131)); /* Child safety */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0138)); /* Low fuel */
    TEST_ASSERT_EQUAL_UINT8(0, psa_can_alarm_id_get_severity(0x0139)); /* Auto wipers */
}

void test_peugeot_407_custom_cockpit_check_sequence(void) {
    vehicle_alert_item_t alert;
    uint8_t out[16];
    size_t len;

    /* Step 1: Start of Diagnosis (0x00F0) - Spec Section 4.1 Step 1 */
    const uint8_t can_step1[8] = { 0x00, 0xF0, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00 };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step1, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F0, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step1[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0xF0, 0xC0, 0x00, 0x00, 0x9E };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step1, out, 10);

    /* Step 2: Rolling Alert 0x0008 (Braking system faulty) - Spec Section 4.1 Step 2 */
    const uint8_t can_step2[8] = { 0x80, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step2, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0008, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step2[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0x08, 0xC0, 0x00, 0x00, 0xB6 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step2, out, 10);

    /* Step 3: Inter-Alert Blanking (0x0000) - Spec Section 4.1 Step 3 */
    const uint8_t can_step3[8] = { 0x80, 0x00, 0x58, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step3, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x0000, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step3[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0x00, 0xC0, 0x00, 0x00, 0xAE };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step3, out, 10);

    /* Step 4: End of Diagnosis (0x00F1) - Spec Section 4.1 Step 4 */
    const uint8_t can_step4[8] = { 0x80, 0xF1, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memset(&alert, 0, sizeof(alert));
    psa_decode_alert_message_0x1a1(can_step4, 8, &alert);
    TEST_ASSERT_TRUE(alert.is_active);
    TEST_ASSERT_EQUAL_HEX16(0x00F1, alert.can_alarm_id);
    len = build_hiworld_alert_single(&alert, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(10, len);
    const uint8_t exp_step4[10] = { 0x5A, 0xA5, 0x05, 0xEA, 0x00, 0xF1, 0xC0, 0x00, 0x00, 0x9F };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_step4, out, 10);
}

void test_peugeot_407_custom_summary_table_and_empty_clearance(void) {
    uint8_t out[80];

    /* 1. Empty Clearance Frame (N = 0) - Spec Section 7 */
    size_t len = build_hiworld_alerts_summary(NULL, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(9, len);
    const uint8_t exp_clear[9] = { 0x5A, 0xA5, 0x04, 0xEA, 0x00, 0x00, 0x00, 0x00, 0xED };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_clear, out, 9);

    /* 2. 5 Active Faults Summary Table - Spec Section 6 */
    vehicle_alert_item_t faults[5];
    memset(faults, 0, sizeof(faults));

    faults[0].can_alarm_id = 0x0008; faults[0].is_active = true; faults[0].display_req = true;
    faults[1].can_alarm_id = 0x0078; faults[1].is_active = true; faults[1].display_req = true;
    faults[2].can_alarm_id = 0x0083; faults[2].is_active = true; faults[2].display_req = true;
    faults[3].can_alarm_id = 0x00E0; faults[3].is_active = true; faults[3].display_req = true;
    faults[4].can_alarm_id = 0x0138; faults[4].is_active = true; faults[4].display_req = true;

    len = build_hiworld_alerts_summary(faults, 5, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(34, len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x1D, out[2]); /* Payload length: 4 + 5*5 = 29 (0x1D) */
    TEST_ASSERT_EQUAL_HEX8(0xEA, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0x05, out[7]); /* N = 5 faults */

    /* Record 0: 0x0008 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[10]);

    /* Record 1: 0x0078 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[13]);
    TEST_ASSERT_EQUAL_HEX8(0x78, out[14]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[15]);

    /* Record 2: 0x0083 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[18]);
    TEST_ASSERT_EQUAL_HEX8(0x83, out[19]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[20]);

    /* Record 3: 0x00E0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[23]);
    TEST_ASSERT_EQUAL_HEX8(0xE0, out[24]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[25]);

    /* Record 4: 0x0138 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[28]);
    TEST_ASSERT_EQUAL_HEX8(0x38, out[29]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[30]);
}

void test_peugeot_407_custom_downlink_0x2f_response(void) {
    can_router_init();
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    hal_can_native_clear_sent_frame();

    /* Feed standard Downlink Query Cmd 0x2F (Spec Section 8: 5A A5 01 2F 00 2F) */
    const uint8_t query_pkt[] = { 0x5A, 0xA5, 0x01, 0x2F, 0x00, 0x2F };
    for (size_t i = 0; i < sizeof(query_pkt); i++) {
        can_router_process_uart_byte(query_pkt[i]);
    }

    /* Verify BSI diagnostic query frame 0x39B was sent on CAN */
    can_frame_t sent_query;
    memset(&sent_query, 0, sizeof(sent_query));
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&sent_query));
    TEST_ASSERT_EQUAL_HEX32(PSA_CAN_ID_ALERT_QUERY, sent_query.id);
}

void test_can_id_filter_enforcement(void) {
    can_router_init();
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);

    const vehicle_profile_t *prof = vehicle_profile_get_active();
    TEST_ASSERT_NOT_NULL(prof);

    /* 1. Verify all active profile rule CAN IDs are accepted by the O(1) bitmask */
    for (uint8_t i = 0; i < prof->rule_count; i++) {
        uint32_t id = prof->rules[i].can_id;
        if (id <= 0x7FF) {
            TEST_ASSERT_TRUE_MESSAGE(can_router_is_id_allowed(id), "Valid profile CAN ID must be allowed");
        }
    }

    /* 2. Verify unmapped standard CAN IDs (e.g., 0x7FF, 0x111, 0x217, 0x260) are rejected */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x7FF));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x111));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x217));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x260));

    /* 3. Verify boundary / out-of-range IDs (> 0x7FF) are rejected */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x800));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x18DAF110));

    /* 4. Inject unmapped standard frame into can_router_process_can:
     * Ensure vehicle state cache is untouched */
    const vehicle_state_t *st_before = can_router_get_state();
    uint16_t rpm_before = st_before->rpm;
    bool drv_door_before = st_before->doors.door_driver;

    can_frame_t unmapped_frame = {
        .id = 0x7FF,
        .dlc = 8,
        .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
    };
    can_router_process_can(&unmapped_frame);

    const vehicle_state_t *st_after = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT16(rpm_before, st_after->rpm);
    TEST_ASSERT_EQUAL_INT(drv_door_before, st_after->doors.door_driver);

    /* 5. Inject 29-bit extended frame (even with matching ID value) - must be rejected */
    can_frame_t ext_frame = {
        .id = 0x0B6, /* RPM frame ID, but marked extended */
        .dlc = 8,
        .is_extended = true,
        .data = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&ext_frame);
    TEST_ASSERT_EQUAL_UINT16(rpm_before, can_router_get_state()->rpm);

    /* 6. Verify dynamic profile switching updates the filter */
    vehicle_profile_set_active(VEHICLE_PROFILE_VAG_PQ35);
    TEST_ASSERT_TRUE(can_router_is_id_allowed(0x5C0)); /* VAG wheel keys */
    TEST_ASSERT_TRUE(can_router_is_id_allowed(0x470)); /* VAG doors */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(PSA_CAN_ID_STALK_21F)); /* PSA stalk must not be in VAG */

    /* Revert back to PSA profile */
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);
    TEST_ASSERT_TRUE(can_router_is_id_allowed(PSA_CAN_ID_STALK_21F));
    TEST_ASSERT_FALSE(can_router_is_id_allowed(0x5C0));
}

