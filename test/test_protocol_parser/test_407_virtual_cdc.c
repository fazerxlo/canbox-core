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

void test_peugeot_407_virtual_cdc_initial_state(void) {
    psa_virtual_cdc_init();
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());

    const psa_virtual_cdc_t *ctx = psa_virtual_cdc_get_context();
    TEST_ASSERT_NOT_NULL(ctx);
    TEST_ASSERT_TRUE(ctx->enabled);
    TEST_ASSERT_EQUAL_UINT8(1, ctx->active_disc);
    TEST_ASSERT_EQUAL_UINT8(1, ctx->track_num);
    TEST_ASSERT_EQUAL_UINT8(99, ctx->total_tracks);
    TEST_ASSERT_EQUAL_UINT8(0, ctx->elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(0, ctx->elapsed_sec);

    /* 100ms tick: transmits 0x162 status frame */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_tick_100ms();
    can_frame_t frame_162;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_162));
    TEST_ASSERT_EQUAL_HEX32(0x162, frame_162.id);
    TEST_ASSERT_EQUAL_UINT8(7, frame_162.dlc);
    TEST_ASSERT_EQUAL_HEX8(0xA0, frame_162.data[0]); /* radio_enabled (0xA0) */
    TEST_ASSERT_EQUAL_HEX8(0x03, frame_162.data[1]); /* status_icon (3) */
    TEST_ASSERT_EQUAL_HEX8(0x06, frame_162.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x01, frame_162.data[3]); /* disc 1 */
    TEST_ASSERT_EQUAL_HEX8(0x06, frame_162.data[5]); /* source registration magic byte 0x06 */
}

void test_peugeot_407_virtual_cdc_downlink_controls(void) {
    psa_virtual_cdc_init();

    /* SubCmd 0x0E: Select Disc Slot 2 */
    psa_virtual_cdc_handle_hu_cmd(0x0E, 0x02);
    /* SubCmd 0x0F: Set Elapsed Sec 20 (0x14) */
    psa_virtual_cdc_handle_hu_cmd(0x0F, 0x14);
    /* SubCmd 0x10: Set Track Number 5 (0x05) */
    psa_virtual_cdc_handle_hu_cmd(0x10, 0x05);

    /* SubCmd 0x02: Audio Start (0x01) */
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());

    const psa_virtual_cdc_t *ctx = psa_virtual_cdc_get_context();
    TEST_ASSERT_EQUAL_UINT8(2, ctx->active_disc);
    TEST_ASSERT_EQUAL_UINT8(5, ctx->track_num);
    TEST_ASSERT_EQUAL_UINT8(20, ctx->elapsed_sec);

    /* 100ms tick: 0x162 reflects active disc and playing bit */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_tick_100ms();
    can_frame_t frame_162;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_162));
    TEST_ASSERT_EQUAL_HEX32(0x162, frame_162.id);
    TEST_ASSERT_EQUAL_HEX8(0x02 | 0x03, frame_162.data[1]); /* playing (0x02) | status_icon (0x03) */
    TEST_ASSERT_EQUAL_HEX8(0x02, frame_162.data[3]); /* disc 2 */

    /* SubCmd 0x02: Audio Stop (0x00) -> returns to standby */
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x00);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());

    /* SubCmd 0x01: Disable Virtual CDC (0x00) -> halts all periodic frames */
    psa_virtual_cdc_handle_hu_cmd(0x01, 0x00);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_presence_enabled());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_tick_100ms();
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_162));

    /* SubCmd 0x01: Enable Virtual CDC (0x01) -> resumes periodic presence */
    psa_virtual_cdc_handle_hu_cmd(0x01, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_tick_100ms();
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_162));
    TEST_ASSERT_EQUAL_HEX32(0x162, frame_162.id);
}

void test_peugeot_407_virtual_cdc_periodic_transmission(void) {
    psa_virtual_cdc_init();
    psa_virtual_cdc_set_state(true);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());

    /* Test 0x131 reception from RD4 radio: Next Track command (0x20) */
    const uint8_t can_131_next[] = { 0xA0, 0x00, 0x00, 0x00, 0x00 }; /* Bit 7: radio on, Bit 5: next track */
    psa_virtual_cdc_process_can_0x131(can_131_next, sizeof(can_131_next));
    const psa_virtual_cdc_t *ctx = psa_virtual_cdc_get_context();
    TEST_ASSERT_EQUAL_UINT8(2, ctx->track_num); /* Track incremented from 1 to 2 */

    /* Run 5 ticks (500ms): 0x1A0 (Play Modes) is sent in the 500ms group */
    hal_can_native_clear_sent_frame();
    for (int i = 0; i < 5; i++) {
        psa_virtual_cdc_tick_100ms();
    }
    can_frame_t frame;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame));
    TEST_ASSERT_EQUAL_HEX32(0x1A0, frame.id);
    TEST_ASSERT_EQUAL_UINT8(2, frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x92, frame.data[0]);

    /* Run 10 ticks (1000ms): 0x0E2 (Disc slot indicator) is sent in the 1000ms group */
    hal_can_native_clear_sent_frame();
    for (int i = 0; i < 5; i++) {
        psa_virtual_cdc_tick_100ms();
    }
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame));
    TEST_ASSERT_EQUAL_HEX32(0x0E2, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, frame.dlc);
}

void test_peugeot_407_rd4_audio_source_switching(void) {
    can_router_init();
    psa_virtual_cdc_init();
    psa_virtual_cdc_update_stalk_rotary(0x42);
    TEST_ASSERT_EQUAL_UINT8(0x42, psa_virtual_cdc_get_stalk_rotary());

    /* 1. Downlink: Switch RD4 to Tuner (SubCmd 0x03, Param 0x01) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x01);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(3, frame_sent.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]); /* Release frame is last sent */
    TEST_ASSERT_EQUAL_HEX8(0x42, frame_sent.data[1]); /* Rolling rotary counter preserved */

    /* 2. Downlink: Switch RD4 to Internal CD (SubCmd 0x03, Param 0x02) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x02);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x42, frame_sent.data[1]);

    /* 3. Downlink: Switch RD4 to CDC (SubCmd 0x02, Param 0x01) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x42, frame_sent.data[1]);

    /* 4. Downlink: CDC Standby (SubCmd 0x02, Param 0x00) -> NO 0x21F frame */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x00);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));
}

void test_peugeot_407_rd4_station_list_trigger(void) {
    can_router_init();
    psa_virtual_cdc_init();
    psa_virtual_cdc_update_stalk_rotary(0x10);

    /* Case A: Radio is inactive (power_status == 0) -> no scroll frames */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_trigger_station_list();
    can_frame_t frame_sent;
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* Case B: Radio is active on Tuner -> triggers scroll UP and DOWN on 0x21F */
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 } /* 0x10 = Tuner */
    };
    can_router_process_can(&frame_165);

    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_trigger_station_list();
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_UINT8(3, frame_sent.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x10, frame_sent.data[1]); /* Restored to original counter */

    /* Case C: Arbiter is busy seeking -> defers scroll trigger until settled on Tuner */
    psa_virtual_cdc_init();
    psa_virtual_cdc_update_stalk_rotary(0x10);
    /* Target is CDC (0x03). Trigger switch to CDC and settle */
    can_frame_t frame_165_cdc = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_cdc);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_reported_source());

    /* Now command switch to Tuner (0x01) -> actively seeking */
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    const psa_source_arbiter_t *arb = psa_virtual_cdc_get_arbiter();
    /* Switching to Tuner must NOT automatically arm station list trigger */
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);

    /* While busy seeking, trigger station list (Cmd 0x0F arrived) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_trigger_station_list();
    TEST_ASSERT_TRUE(arb->pending_station_list_trigger);
    /* No scroll frame should have been sent right now */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* RD4 settles on Tuner via 0x165 */
    can_router_process_can(&frame_165);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);
    /* Now scroll frame was executed */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);
    TEST_ASSERT_EQUAL_HEX8(0x10, frame_sent.data[1]);

    /* Case D: 2-second cooldown debounces rapid consecutive station list triggers */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_trigger_station_list();
    /* Immediately triggering again within 2 seconds must be debounced (no CAN transmission) */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* Case E: Normal switch to Tuner without Cmd 0x0F does NOT trigger rotary pulse */
    psa_virtual_cdc_init();
    psa_virtual_cdc_update_stalk_rotary(0x10);
    can_router_process_can(&frame_165_cdc); /* On CDC */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x01); /* Switch to Tuner */
    can_router_process_can(&frame_165); /* Settles on Tuner */
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);
    /* Last sent frame must NOT be a rotary scroll pulse (i.e. not 0x21F with restored cnt) */
    if (hal_can_native_get_last_sent_frame(&frame_sent)) {
        /* If 0x21F was sent, it should only be SOURCE button pulse, never idle rotary scroll */
        if (frame_sent.id == 0x21F) {
            TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[0]);
            TEST_ASSERT_EQUAL_HEX8(0x10, frame_sent.data[1]);
        }
    }

    /* Case F: Physical button transition to Tuner does NOT trigger rotary pulse */
    psa_virtual_cdc_init();
    psa_virtual_cdc_update_stalk_rotary(0x10);
    can_router_process_can(&frame_165_cdc); /* On CDC */
    can_router_process_can(&frame_165);     /* Driver clicked SRC to Tuner -> enters physical debounce */
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);
    hal_delay_ms(850);
    psa_virtual_cdc_tick_100ms();           /* Settles in IDLE on Tuner */
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);
}

void test_peugeot_407_rd4_source_switch_lockout_and_suppression(void) {
    can_router_init();
    psa_virtual_cdc_init();

    /* Put radio into initial state: Tuner (0x10) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());

    /* Target is CD (0x02). Trigger switch to CD */
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x02);
    /* Now switching is actively in progress waiting for CAN 0x165 confirm */
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_virtual_cdc_get_target_source());

    /* Direct query helper verification while switching */
    vehicle_radio_t radio_aux = { .source_mode = 0x40, .power_status = 1 };
    vehicle_radio_t radio_cd = { .source_mode = 0x20, .power_status = 1 };
    vehicle_radio_t radio_tuner = { .source_mode = 0x01, .power_status = 1 };

    TEST_ASSERT_FALSE(psa_virtual_cdc_is_target_source_reached(&radio_aux));
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_target_source_reached(&radio_cd));
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_target_source_reached(&radio_tuner));

    /* Rule F4: Preemption - switch target in-flight to CDC (SubCmd 0x02, Param 0x01) */
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_target_source());

    /* CAN 0x165 arrives confirming CDC (0x30 in nibble = CDC) */
    can_frame_t frame_165_cdc = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_cdc);

    /* Target reached! Transition back to IDLE */
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_reported_source());
}

void test_peugeot_407_rd4_physical_button_debounce(void) {
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. Initial boot arrival: Tuner (0x10) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_virtual_cdc_get_reported_source());

    /* 2. Driver clicks physical SRC button -> 0x165 reports CD (0x20) */
    can_frame_t frame_165_cd = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_cd);

    /* Enters SRC_SYNC_PHYSICAL_DEBOUNCE -> 0x84 suppressed! */
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_source_filtered());
    const psa_source_arbiter_t *arbiter = psa_virtual_cdc_get_arbiter();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);
    /* Reported source is still the old Tuner until debounce settles */
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, arbiter->reported_hu_source);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, arbiter->actual_rd4_source);

    /* 3. Driver clicks again within 200 ms -> 0x165 reports CDC (0x30) */
    can_frame_t frame_165_cdc = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_cdc);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, arbiter->actual_rd4_source);

    /* 4. Advance time past 800 ms debounce settle timer */
    hal_delay_ms(850);
    psa_virtual_cdc_tick_100ms();

    /* Settled! Returns to IDLE and updates reported source */
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arbiter->sync_state);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, arbiter->reported_hu_source);
}

void test_peugeot_407_rd4_cd_switch_intermediate_aux1_stepping(void) {
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. Radio initial state: Tuner mode */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_virtual_cdc_get_reported_source());

    /* 2. HU requests switch to Internal CD (SubCmd 0x03, Param 0x02) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x02);

    /* Verify initial pulse injected and arbiter is seeking */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_virtual_cdc_get_target_source());

    /* 3. In the vehicle, RD4 cycles through Input 1 / AUX 1 first: 165#c8d04000 */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_165_aux1 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux1);

    const vehicle_state_t *state = can_router_get_state();
    /* Must be decoded as AUX 1 (0x40), NEVER misdecoded as CD (0x20) */
    TEST_ASSERT_EQUAL_HEX8(0x40, state->media.radio.source_mode);

    /* Arbiter must NOT stop on AUX 1 when target is CD: must keep pulsing 0x21F */
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_source_filtered());
    const psa_source_arbiter_t *arbiter = psa_virtual_cdc_get_arbiter();
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, arbiter->actual_rd4_source);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, arbiter->target_source);
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_WAIT_CONFIRM, arbiter->sync_state);
    /* Next pulse was injected */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);

    /* 4. RD4 cycles to Internal CD: 165#c8d02000 */
    can_frame_t frame_165_cd = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x20, 0x00 }
    };
    can_router_process_can(&frame_165_cd);

    /* Target CD reached! */
    TEST_ASSERT_EQUAL_HEX8(0x20, state->media.radio.source_mode);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_virtual_cdc_get_reported_source());
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arbiter->sync_state);
}

void test_peugeot_407_rd4_no_premature_pulse_when_new_cmd_arrives(void) {
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. Initial state: Radio is on Tuner (0x10) */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_virtual_cdc_get_reported_source());

    /* 2. HU sends Cmd 0xAD to switch to CD (SubCmd 0x03, Param 0x02) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x02);

    /* Verify Pulse #1 injected */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    const psa_source_arbiter_t *arbiter = psa_virtual_cdc_get_arbiter();
    uint32_t initial_timer = arbiter->state_timer_ms;

    /* 3. Before RD4 responds with 0x165, HU sends new Cmd 0xAD (e.g. SubCmd 0x02 Param 0x01 = CDC) */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);

    /* Bug 3 verification:
     * - NO new 0x21F pulse injected directly!
     * - Timer must NOT be reset!
     * - Target is updated to CDC! */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_UINT32(initial_timer, arbiter->state_timer_ms);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_target_source());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    /* 4. RD4 responds to Pulse #1 by switching to AUX 1: 165#c8d04000 */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_165_aux1 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux1);

    /* Settle delay observed, then Pulse #2 injected towards new target CDC */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, arbiter->target_source);

    /* 5. RD4 responds to Pulse #2 by switching to CDC: 165#c8d03000 */
    can_frame_t frame_165_cdc = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x30, 0x00 }
    };
    can_router_process_can(&frame_165_cdc);

    /* Target CDC reached without overshooting */
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_source_filtered());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_reported_source());
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arbiter->sync_state);
}

void test_peugeot_407_rd4_confirm_timeout_350ms(void) {
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. Radio on Tuner */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);

    /* 2. Switch to CD */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x02);
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));

    /* 3. Advance 300 ms (under 650ms threshold) */
    hal_can_native_clear_sent_frame();
    hal_delay_ms(300);
    psa_virtual_cdc_tick_100ms();

    /* Must NOT fire a retry pulse yet! Waiting up to 650ms for RD4 0x165 */
    const psa_source_arbiter_t *arbiter = psa_virtual_cdc_get_arbiter();
    TEST_ASSERT_EQUAL_UINT8(0, arbiter->retry_count);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    /* 4. Advance past 650ms (total 300 + 400 = 700ms > 650ms) */
    hal_delay_ms(400);
    psa_virtual_cdc_tick_100ms();

    /* Now timeout expired, retry pulse was fired and retry_count incremented */
    TEST_ASSERT_EQUAL_UINT8(1, arbiter->retry_count);
}

void test_peugeot_407_rd4_post_target_settle_guard(void) {
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. Radio on CD */
    can_frame_t frame_165_cd = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x20, 0x00 }
    };
    can_router_process_can(&frame_165_cd);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_virtual_cdc_get_reported_source());

    /* 2. Switch to Tuner */
    psa_virtual_cdc_handle_hu_cmd(0x03, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());

    /* 3. RD4 confirms Tuner: Target reached! */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_virtual_cdc_get_reported_source());

    /* 4. An in-flight RD4 step arrives 100 ms later (e.g. residual step to CD) */
    hal_delay_ms(100);
    can_router_process_can(&frame_165_cd);

    /* Post-target settle guard (400ms) MUST ignore this in-flight step:
     * Arbiter must remain in SRC_SYNC_IDLE and NOT enter SRC_SYNC_PHYSICAL_DEBOUNCE! */
    const psa_source_arbiter_t *arbiter = psa_virtual_cdc_get_arbiter();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arbiter->sync_state);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, arbiter->reported_hu_source);

    /* 5. Advance past 400ms settle guard window */
    hal_delay_ms(350);

    /* 6. Now a physical source change to CD arrives (driver click) */
    can_router_process_can(&frame_165_cd);
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);
}

void test_peugeot_407_virtual_cdc_always_enabled_and_subcmd_0x02(void) {
    psa_virtual_cdc_init();

    /* 1. RD4 powers on in Tuner mode: 165#c8d01000 */
    can_frame_t frame_165_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_165_tuner);

    /* CDC must be enabled on CAN, but in PAUSE state (not active playback) */
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    const psa_virtual_cdc_t *ctx = psa_virtual_cdc_get_context();
    TEST_ASSERT_TRUE(ctx->radio_enabled);
    TEST_ASSERT_EQUAL_UINT8(2, ctx->status_icon); /* STATUS_ICON_PAUSE */

    /* 2. HU requests playback / switch to CDC: SubCmd 0x02, Param 0x01 */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);

    /* Verify CDC playback active and source switch sequence initiated */
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_target_source());

    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);

    /* 3. HU requests standby: SubCmd 0x02, Param 0x00 */
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x00);
    /* CDC playback stopped, in PAUSE state, but remains ENABLED on bus */
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_presence_enabled());
    TEST_ASSERT_EQUAL_UINT8(2, ctx->status_icon); /* STATUS_ICON_PAUSE */
}

void test_peugeot_407_rd4_physical_override_blocks_periodic_0xad_02_01(void) {
    /* Regression test for dump_2026-10-18_15-20-27.log:
     * When user manually presses RD4 SOURCE button to switch away from CDC,
     * repeated/periodic 5A A5 02 AD 02 01 B1 from HU must NOT hijack RD4 back to CDC. */
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. RD4 starts on CDC (165#c8d03000) */
    can_frame_t frame_165_cdc = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x30, 0x00 }
    };
    can_router_process_can(&frame_165_cdc);
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_active());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, psa_virtual_cdc_get_reported_source());

    /* 2. User presses physical RD4 SOURCE button -> switches to AUX 1 (165#c8d04000) */
    can_frame_t frame_165_aux = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xD0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_165_aux);
    const psa_source_arbiter_t *arbiter = psa_virtual_cdc_get_arbiter();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);
    TEST_ASSERT_TRUE(arbiter->user_override_non_cdc);
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());

    /* 3. During physical debounce (e.g. 200ms later), HU sends periodic 0xAD 02 01 */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    can_frame_t frame_pulse;
    /* Must NOT inject 0x21F pulse or change target_source! */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_UINT8(0, arbiter->target_source);
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_PHYSICAL_DEBOUNCE, arbiter->sync_state);

    /* 4. Physical debounce settles after 850ms on AUX 1 */
    hal_delay_ms(850);
    psa_virtual_cdc_tick_100ms();
    TEST_ASSERT_EQUAL_INT(SRC_SYNC_IDLE, arbiter->sync_state);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, arbiter->reported_hu_source);
    TEST_ASSERT_TRUE(arbiter->user_override_non_cdc);

    /* 5. 447ms later, HU sends periodic 0xAD 02 01 again while settled on AUX 1 */
    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    /* Must NOT hijack RD4 back to CDC! */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_FALSE(psa_virtual_cdc_is_active());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, arbiter->reported_hu_source);

    /* 6. Once HU stops audio (0xAD 02 00) and later starts fresh playback (0xAD 02 01),
     * or sends explicit SubCmd 0x03, switching to CDC is allowed again */
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x00);
    TEST_ASSERT_FALSE(arbiter->user_override_non_cdc);

    hal_can_native_clear_sent_frame();
    psa_virtual_cdc_handle_hu_cmd(0x02, 0x01);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_virtual_cdc_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CDC, arbiter->target_source);
}

void test_peugeot_407_rd4_reset_zero_volume_and_0xa1_loop_suppression(void) {
    /* Regression test for dump_2026-10-18_15-21-33.log:
     * 1) When RD4 powers off / resets (165#00000000), transient 1A5#00 and 325#000000
     *    must NOT set master_volume = 0 or latch play_status = 0x0C (Eject).
     * 2) Subsequent 1A5#E9 restores volume = 9 cleanly. */
    can_router_init();
    psa_virtual_cdc_init();

    /* 1. RD4 active on Internal CD with Volume = 9 (1A5#E9) and Track 3 playing */
    can_frame_t frame_165_cd = {
        .id = 0x165, .dlc = 4, .data = { 0xC8, 0xD0, 0x20, 0x00 }
    };
    can_frame_t frame_1a5_vol9 = {
        .id = 0x1A5, .dlc = 1, .data = { 0xE9 }
    };
    can_frame_t frame_365 = {
        .id = 0x365, .dlc = 5, .data = { 0x0C, 0x26, 0x02, 0x00, 0x00 }
    };
    can_frame_t frame_3a5 = {
        .id = 0x3A5, .dlc = 6, .data = { 0x02, 0x01, 0x19, 0x00, 0x2E, 0x00 }
    };
    can_router_process_can(&frame_165_cd);
    can_router_process_can(&frame_1a5_vol9);
    can_router_process_can(&frame_365);
    can_router_process_can(&frame_3a5);

    const vehicle_state_t *st = can_router_get_state();
    TEST_ASSERT_EQUAL_UINT8(9, st->audio.master_volume);
    TEST_ASSERT_EQUAL_UINT8(0, st->audio.active_mode);
    TEST_ASSERT_EQUAL_UINT8(0x02, st->media.cdc.play_status);

    /* 2. RD4 powers off / warm resets (165#00000000) and emits zeroed burst (1A5#00, 325#000000) */
    can_frame_t frame_165_off = {
        .id = 0x165, .dlc = 4, .data = { 0x00, 0x00, 0x00, 0x00 }
    };
    can_frame_t frame_1a5_zero = {
        .id = 0x1A5, .dlc = 1, .data = { 0x00 }
    };
    can_frame_t frame_325_zero = {
        .id = 0x325, .dlc = 3, .data = { 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165_off);
    can_router_process_can(&frame_1a5_zero);
    can_router_process_can(&frame_325_zero);

    /* Volume MUST remain 9 (NOT overwritten by 0) and play_status MUST be 0x01 Pause (NOT 0x0C Eject) */
    TEST_ASSERT_EQUAL_UINT8(9, st->audio.master_volume);
    TEST_ASSERT_EQUAL_UINT8(0, st->audio.active_mode);
    TEST_ASSERT_EQUAL_UINT8(0x01, st->media.cdc.play_status);
}




