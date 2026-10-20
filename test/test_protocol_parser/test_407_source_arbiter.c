#include "unity.h"
#include "profiles/peugeot_407.h"
#include "profiles/psa_source_arbiter.h"
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

void test_peugeot_407_aux_mode_suppression(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    /* 1. Set source to AUX 1 via CAN 0x165 (0xC8, 0xC0, 0x40, 0x00) */
    const uint8_t can_165_aux1[] = { 0xC8, 0xC0, 0x40, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_165_aux1, sizeof(can_165_aux1));

    TEST_ASSERT_EQUAL_HEX8(0x40, media.radio.source_mode);
    TEST_ASSERT_EQUAL_HEX8(0x01, media.radio.power_status);

    /* 2. Process background CAN tuner frame 0x225 (preset 6, 89.4 MHz, FM1) */
    const uint8_t can_225_bg[] = { 0x20, 0x60, 0x90, 0x03, 0x14 };
    psa_rd4_process_can_0x225(&media.radio, can_225_bg, sizeof(can_225_bg));

    /* Source mode MUST remain AUX 1 (0x40), NOT overwritten by tuner band */
    TEST_ASSERT_EQUAL_HEX8(0x40, media.radio.source_mode);

    /* 3. Build Hiworld Cmd 0x84: radio tuner parameters must be blanked in AUX mode */
    uint8_t out[32];
    size_t out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x40, out[4]); /* Source: AUX 1 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]); /* Frequency LSB = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[6]); /* Frequency MSB = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[7]); /* Preset slot = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]); /* Indicators = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[9]); /* Power status = 1 */
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&out[10], 8); /* Blank station name */

    /* 4. Test AUX 2 mode (src = 0x5 -> 0x41) */
    const uint8_t can_165_aux2[] = { 0xC8, 0xC0, 0x50, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_165_aux2, sizeof(can_165_aux2));
    TEST_ASSERT_EQUAL_HEX8(0x41, media.radio.source_mode);

    out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_HEX8(0x41, out[4]); /* Source: AUX 2 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[7]); /* Preset slot = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]); /* Indicators = 0 */
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&out[10], 8);
}

void test_peugeot_407_rd4_audio_source_switching(void) {
    can_router_init();
    psa_source_arbiter_init();
    psa_source_arbiter_update_stalk_rotary(0x42);
    TEST_ASSERT_EQUAL_UINT8(0x42, psa_source_arbiter_get_stalk_rotary());

    /* 1. Downlink: Switch RD4 to Tuner (SubCmd 0x03, Param 0x01) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x01);
    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(3, frame_sent.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]); /* Release frame is last sent */
    TEST_ASSERT_EQUAL_HEX8(0x42, frame_sent.data[1]); /* Rolling rotary counter preserved */

    /* 2. Downlink: Switch RD4 to Internal CD (SubCmd 0x03, Param 0x02) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x02);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x42, frame_sent.data[1]);

    /* 3. Downlink: Switch RD4 to Line In 1 / AUX 1 (SubCmd 0x03, Param 0x03) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x03);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());

    /* 4. Downlink: Switch RD4 to Line In 1 / AUX 1 via Audio Start (SubCmd 0x02, Param 0x01) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());

    /* 5. Downlink: Audio Stop / Standby (SubCmd 0x02, Param 0x00) -> NO 0x21F frame */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x00);
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));
}

void test_peugeot_407_rd4_station_list_trigger(void) {
    can_router_init();
    psa_source_arbiter_init();
    psa_source_arbiter_update_stalk_rotary(0x10);

    /* 1. Put radio into Tuner mode: source_mode < 0x20 and power_status == 1 */
    vehicle_state_t *st = (vehicle_state_t *)can_router_get_state();
    st->media.radio.source_mode = 0x01; /* FM1 */
    st->media.radio.power_status = 0x01;
    st->media.radio.present = true;

    /* 2. Trigger station list rotary encoder scroll */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_trigger_station_list();

    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(3, frame_sent.dlc);
    /* Forward rotary change is +1 */
    TEST_ASSERT_EQUAL_UINT8(0x11, psa_source_arbiter_get_stalk_rotary());

    /* Complete closed-loop sequence: RD4 confirms station 2, settle, return pulse */
    psa_source_arbiter_process_can_0x225(2, 960);
    hal_delay_ms(70);
    psa_source_arbiter_tick_100ms();
    /* Net rotary change is 0 after Up then Down */
    TEST_ASSERT_EQUAL_UINT8(0x10, psa_source_arbiter_get_stalk_rotary());
    psa_source_arbiter_process_can_0x225(1, 960);

    /* 3. Non-tuner mode (CD mode 0x20): trigger must be suppressed to avoid disturbing CD playback */
    st->media.radio.source_mode = 0x20;
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_trigger_station_list();
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* 4. Radio OFF mode: trigger must also be suppressed */
    st->media.radio.source_mode = 0x01;
    st->media.radio.power_status = 0x00;
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_trigger_station_list();
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));
}

void test_peugeot_407_rd4_source_switch_lockout_and_suppression(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio into initial state: Tuner (0x10) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());

    /* Target is CD (0x02). Trigger switch to CD */
    psa_source_arbiter_handle_hu_cmd(0x03, 0x02);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_TRUE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_source_arbiter_get_target_source());

    /* Direct query helper verification while switching */
    vehicle_radio_t radio_aux = { .source_mode = 0x40, .power_status = 1 };
    vehicle_radio_t radio_cd = { .source_mode = 0x20, .power_status = 1 };
    vehicle_radio_t radio_tuner = { .source_mode = 0x01, .power_status = 1 };

    TEST_ASSERT_FALSE(psa_source_arbiter_is_target_source_reached(&radio_aux));
    TEST_ASSERT_TRUE(psa_source_arbiter_is_target_source_reached(&radio_cd));
    TEST_ASSERT_FALSE(psa_source_arbiter_is_target_source_reached(&radio_tuner));

    /* Rule F4: Preemption - switch target in-flight to Line In 1 / AUX 1 (SubCmd 0x02, Param 0x01) */
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());

    /* CAN 0x165 arrives confirming AUX 1 (0x40 in nibble = AUX 1) */
    can_frame_t frame_165_aux = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_aux);

    /* Target reached! Transition back to IDLE */
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());
}

void test_peugeot_407_rd4_physical_button_debounce(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio into initial state: Tuner (0x10) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* User physically presses SOURCE on the radio: transitions directly to CD (0x20) */
    can_frame_t frame_165_cd = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_cd);

    /* Debounce state active: reported source remains TUNER while debounce settles */
    TEST_ASSERT_TRUE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* User presses again 200ms later to cycle to AUX 1 (0x40) */
    hal_delay_ms(200);
    can_frame_t frame_165_aux = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_aux);

    /* Still in debounce, reported source is STILL Tuner */
    TEST_ASSERT_TRUE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* Advance time past 800ms debounce settle time (e.g. 850 ms) */
    hal_delay_ms(850);
    psa_source_arbiter_tick_100ms();

    /* Debounce resolved! Reported source is now AUX 1 */
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());
}

void test_peugeot_407_rd4_cd_switch_intermediate_aux1_stepping(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* 1. Initial State: Radio is in Tuner mode (CAN 0x165: 0xC8, 0xD0, 0x10, 0x00) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* 2. HU requests switch to CD (SubCmd 0x03, Param 0x02) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x02);

    /* Verify Pulse #1 injected */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_TRUE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_source_arbiter_get_target_source());

    /* 3. RD4 responds to Pulse #1 by stepping to AUX 1: 165#c8d04000 (intermediate source) */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_165_aux1 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux1);

    /* Verify Pulse #2 injected automatically after 150 ms settle time */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_TRUE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_source_arbiter_get_target_source());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* 4. RD4 responds to Pulse #2 by reaching CD: 165#c8d02000 */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_165_cd = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x20, 0x00 }
    };
    can_router_process_can(&frame_165_cd);

    /* Target reached! */
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_source_arbiter_get_reported_source());
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
}

void test_peugeot_407_rd4_no_premature_pulse_when_new_cmd_arrives(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* 1. Initial state: Radio is on Tuner (0x10) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* 2. HU sends Cmd 0xAD to switch to CD (SubCmd 0x03, Param 0x02) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x02);

    /* Verify Pulse #1 injected */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());
    const psa_source_arbiter_t *arbiter = psa_source_arbiter_get_context();
    uint32_t initial_timer = arbiter->state_timer_ms;

    /* 3. Before RD4 responds with 0x165, HU sends new Cmd 0xAD (SubCmd 0x02 Param 0x01 = Line In 1 / AUX 1) */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);

    /* Verification:
     * - NO new 0x21F pulse injected directly!
     * - Timer must NOT be reset!
     * - Target is updated to AUX 1! */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_UINT32(initial_timer, arbiter->state_timer_ms);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());

    /* 4. RD4 responds to Pulse #1 by switching to AUX 1: 165#c8d04000 */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_165_aux1 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux1);

    /* Target AUX 1 reached! Transition to IDLE, no extra pulses */
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_source_arbiter_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
}

void test_peugeot_407_rd4_confirm_timeout_350ms(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio on Tuner */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);

    /* HU requests switch to CD */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x02);
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));

    /* Advance time by 300 ms (before timeout): tick_100ms should NOT retransmit pulse */
    hal_delay_ms(300);
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_tick_100ms();
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));

    /* Advance time past 650 ms timeout: tick_100ms re-pulses */
    hal_delay_ms(400);
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_tick_100ms();
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
}

void test_peugeot_407_rd4_post_target_settle_guard(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Initial on Tuner */
    can_frame_t frame_165_tuner = {
        .id = 0x165, .dlc = 4, .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);

    /* Switch to CD */
    psa_source_arbiter_handle_hu_cmd(0x03, 0x02);

    /* CD arrives: target reached! */
    can_frame_t frame_165_cd = {
        .id = 0x165, .dlc = 4, .data = { 0xC8, 0xD0, 0x20, 0x00 }
    };
    can_router_process_can(&frame_165_cd);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_source_arbiter_get_reported_source());

    /* A stray intermediate 0x165 frame arrives 100 ms later: must NOT enter debounce */
    hal_delay_ms(100);
    can_frame_t frame_165_stray = {
        .id = 0x165, .dlc = 4, .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_stray);
    const psa_source_arbiter_t *arb = psa_source_arbiter_get_context();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arb->sync_state);
}

void test_peugeot_407_rd4_physical_override_blocks_periodic_0xad_02_01(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* 1. RD4 starts on AUX 1 (165#c8d04000) */
    can_frame_t frame_165_aux = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux);
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());

    /* 2. User presses physical RD4 SOURCE button -> switches to Tuner (165#c8d01000) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    const psa_source_arbiter_t *arbiter = psa_source_arbiter_get_context();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);
    TEST_ASSERT_TRUE(arbiter->user_override_non_hu);

    /* 3. During physical debounce (e.g. 200ms later), HU sends periodic 0xAD 02 01 */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    can_frame_t frame_pulse;
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_UINT8(0, arbiter->target_source);
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);

    /* 4. Physical debounce settles after 850ms on Tuner */
    hal_delay_ms(850);
    psa_source_arbiter_tick_100ms();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arbiter->sync_state);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, arbiter->reported_hu_source);
    TEST_ASSERT_TRUE(arbiter->user_override_non_hu);

    /* 5. HU sends periodic 0xAD 02 01 again while settled on Tuner */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, arbiter->reported_hu_source);

    /* 6. Once HU stops audio (0xAD 02 00) and later starts fresh playback (0xAD 02 01),
     * switching to AUX 1 is allowed again */
    psa_source_arbiter_handle_hu_cmd(0x02, 0x00);
    TEST_ASSERT_FALSE(arbiter->user_override_non_hu);

    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());
}

void test_peugeot_407_rd4_reset_zero_volume_and_0xa1_loop_suppression(void) {
    vehicle_audio_settings_t audio;
    memset(&audio, 0, sizeof(audio));

    /* 1. Test 0x1A5 transient zero volume rejection:
     * When master_volume was established > 2, a solitary 0x1A5#00 frame must be rejected */
    audio.master_volume = 15;
    audio.valid = true;

    const uint8_t can_1a5_zero[] = { 0x00 };
    bool changed = psa_decode_rd4_volume_0x1a5(can_1a5_zero, sizeof(can_1a5_zero), &audio);
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_EQUAL_UINT8(15, audio.master_volume);

    /* Valid non-zero volume update */
    const uint8_t can_1a5_vol12[] = { 0x0C };
    changed = psa_decode_rd4_volume_0x1a5(can_1a5_vol12, sizeof(can_1a5_vol12), &audio);
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL_UINT8(12, audio.master_volume);
}

void test_peugeot_407_rd4_cmd_0xad_switches_to_line_in_1(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio on Tuner */
    can_frame_t frame_165_tuner = {
        .id = 0x165, .dlc = 4, .data = { 0x80, 0x00, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* 1. Downlink: Cmd 0xAD SubCmd 0x03 Param 0x03 (HU Audio) -> switches to Line In 1 / AUX 1 */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x03, 0x03);
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());

    /* Settle on AUX 1 */
    can_frame_t frame_165_aux = {
        .id = 0x165, .dlc = 4, .data = { 0x80, 0x00, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());

    /* 2. Switch back to Tuner */
    psa_source_arbiter_handle_hu_cmd(0x03, 0x01);
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    /* 3. Downlink: Cmd 0xAD SubCmd 0x02 Param 0x01 (Audio Play) -> switches to Line In 1 / AUX 1 */
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());
}

void test_peugeot_407_station_list_scroll_closed_loop_success(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio on Tuner with Preset 1 */
    can_frame_t frame_165_tuner = {
        .id = 0x165, .dlc = 4, .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    can_frame_t frame_225_p1 = {
        .id = 0x225, .dlc = 5, .data = { 0x20, 0x10, 0x10, 0x03, 0x98 }
    };
    can_router_process_can(&frame_225_p1);

    hal_can_native_clear_sent_frame();
    psa_source_arbiter_trigger_station_list();
    TEST_ASSERT_TRUE(psa_source_arbiter_is_station_scroll_in_progress());

    /* Verify first forward pulse (+1) sent */
    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    uint8_t forward_cnt = frame_sent.data[1];

    /* Advance time without RD4 station change: should remain in WAIT_NEXT */
    hal_delay_ms(200);
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_tick_100ms();
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_TRUE(psa_source_arbiter_is_station_scroll_in_progress());

    /* RD4 confirms move to Station / Preset 2 via CAN 0x225 */
    can_frame_t frame_225_p2 = {
        .id = 0x225, .dlc = 5, .data = { 0x29, 0x20, 0x10, 0x03, 0x98 }
    };
    can_router_process_can(&frame_225_p2);

    /* Settle for 70 ms (>= PSA_STATION_SCROLL_SETTLE_MS) */
    hal_delay_ms(70);
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_tick_100ms();

    /* Verify return pulse (-1) sent back to station 1 */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(forward_cnt - 1), frame_sent.data[1]);

    /* RD4 confirms return to Station / Preset 1 via CAN 0x225 */
    can_router_process_can(&frame_225_p1);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_station_scroll_in_progress());
}

void test_peugeot_407_station_list_scroll_timeout_retry_and_abort(void) {
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio on Tuner with Preset 1 */
    can_frame_t frame_165_tuner = {
        .id = 0x165, .dlc = 4, .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    can_frame_t frame_225_p1 = {
        .id = 0x225, .dlc = 5, .data = { 0x20, 0x10, 0x10, 0x03, 0x98 }
    };
    can_router_process_can(&frame_225_p1);

    hal_can_native_clear_sent_frame();
    psa_source_arbiter_trigger_station_list();
    TEST_ASSERT_TRUE(psa_source_arbiter_is_station_scroll_in_progress());

    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Timeout attempt 1: wait 510 ms without RD4 moving to station 2 */
    hal_delay_ms(510);
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_tick_100ms();

    /* Requirement: repeated once upon timeout! */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_station_scroll_in_progress());

    /* Timeout attempt 2: wait another 510 ms without RD4 moving */
    hal_delay_ms(510);
    hal_can_native_clear_sent_frame();
    psa_source_arbiter_tick_100ms();

    /* Requirement: abort and do NOT send return pulse (-1) to avoid wrap to station 6! */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_FALSE(psa_source_arbiter_is_station_scroll_in_progress());
}
