#include <string.h>
#include "test_integration_common.h"
#include "core/vehicle_profile.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"
#include "profiles/psa_source_arbiter.h"
#include "hal/hal_gpio.h"

void test_integration_hiworld_rd4_radio_off_suppresses_cd_tuner_frames(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 0. Boot sequence from dump_2026-10-15_18-37-15.log:
     * Background 0x2A5 and 0x225 arrive BEFORE the first 0x165 OFF frame.
     * They must be cached silently without emitting any UART frames. */
    can_frame_t frame_2a5_boot = {
        .id = 0x2A5,
        .dlc = 8,
        .data = { 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D, 0x20, 0x20 }
    };
    can_router_process_can(&frame_2a5_boot);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));

    /* First 0x165 arrives with RD4 OFF (0x48, 0xC8, 0x00, 0x00):
     * Emits one-shot Cmd 0x84 with source_mode = 0xFF, power_status = 0x00 */
    can_frame_t frame_off = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x48, 0xC8, 0x00, 0x00 }
    };
    can_router_process_can(&frame_off);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t expected_radio_off[] = {
        0x5A, 0xA5, 0x0E, 0x84, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 0x90
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_radio_off), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_radio_off, rx_buf, sizeof(expected_radio_off));

    /* Subsequent 0x165 OFF frames must NOT re-emit 0x84 */
    can_router_process_can(&frame_off);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));

    /* 1. Power ON RD4 radio in Tuner mode (CAN 0x165: 0xCC, 0x54, 0x10, 0x02) */
    can_frame_t frame_on = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_on);

    can_frame_t frame_225 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x01, 0x90, 0x04, 0x1A }
    };
    can_router_process_can(&frame_225);

    /* Verify 0x84 received while radio is ON */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_0x84 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_0x84 = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);

    /* 2. RD4 Radio turned OFF (CAN 0x165: 0x48, 0xC8, 0x00, 0x00):
     * Must emit one-shot 0x84 0xFF (Radio OFF) cleanly */
    can_router_process_can(&frame_off);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_radio_off), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_radio_off, rx_buf, sizeof(expected_radio_off));

    /* 3. Inject background CAN tuner status (CAN 0x225 from dump log) while OFF:
     * Cached in state, but ZERO UART output (no flip-flopping to 0x01 / 0x85!) */
    can_frame_t frame_225_bg = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x30, 0x00, 0x10, 0x03, 0xDC }
    };
    can_router_process_can(&frame_225_bg);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 4. Inject background CAN CD track telemetry (CAN 0x3A5 from dump log) while OFF:
     * Cached in state, but ZERO UART output */
    can_frame_t frame_3a5_bg = {
        .id = 0x3A5,
        .dlc = 6,
        .data = { 0x0B, 0x02, 0x24, 0x02, 0x0E, 0x00 }
    };
    can_router_process_can(&frame_3a5_bg);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 5. Inject background CAN Station Name (0x2A5) and another 0x165 OFF frame:
     * ZERO UART output */
    can_router_process_can(&frame_2a5_boot);
    can_router_process_can(&frame_off);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 6. Verify 60s periodic resync does NOT emit any radio or CD packets when radio is off */
    for (int t = 0; t < 600; t++) {
        can_router_periodic_100ms();
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            uint8_t cmd = rx_buf[i + 3];
            TEST_ASSERT_NOT_EQUAL_HEX8(0x84, cmd);
            TEST_ASSERT_NOT_EQUAL_HEX8(0x85, cmd);
            TEST_ASSERT_NOT_EQUAL_HEX8(0x86, cmd);
            TEST_ASSERT_NOT_EQUAL_HEX8(0x97, cmd);
        }
    }

    /* 7. Turn RD4 radio back ON: telemetry resumes immediately with the cached 0x225 frequency! */
    can_router_process_can(&frame_on);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    found_0x84 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_0x84 = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);
}

void test_integration_hiworld_rd4_audio_routing_spec(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_source_arbiter_init();

    /* Wake bus */
    can_frame_t frame_ign = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_ign);

    /* Drain initial UART output */
    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* --------------------------------------------------------------------------
     * 1. Downlink: Switch RD4 to Line In 1 / AUX 1 (5A A5 02 AD 02 01 B1)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_aux_active[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x02, 0x01, 0xB1 };
    for (size_t i = 0; i < sizeof(pkt_aux_active); i++) {
        can_router_process_uart_byte(pkt_aux_active[i]);
    }

    can_frame_t frame_sent;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Verify UART responses: Universal ACK (0xFF, 0xAD) and Cmd 0x84 (AUX 1 0x40) */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_ack_ad = false;
    bool found_84_aux = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len) {
            if (rx_buf[i + 3] == 0xFF && rx_buf[i + 4] == 0xAD) {
                found_ack_ad = true;
            }
            if (rx_buf[i + 3] == 0x84 && rx_buf[i + 4] == 0x40) {
                found_84_aux = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_ack_ad);
    TEST_ASSERT_TRUE(found_84_aux);

    /* --------------------------------------------------------------------------
     * 2. Downlink: Standby (5A A5 02 AD 02 00 B0)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_standby[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x02, 0x00, 0xB0 };
    for (size_t i = 0; i < sizeof(pkt_standby); i++) {
        can_router_process_uart_byte(pkt_standby[i]);
    }

    /* Standby must NOT inject stalk pulse */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_sent));

    /* --------------------------------------------------------------------------
     * 3. Downlink: Switch RD4 to Tuner (5A A5 02 AD 03 01 B3)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_tuner[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x01, 0xB3 };
    for (size_t i = 0; i < sizeof(pkt_switch_tuner); i++) {
        can_router_process_uart_byte(pkt_switch_tuner[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* --------------------------------------------------------------------------
     * 4. Downlink: Switch RD4 to CD (5A A5 02 AD 03 02 B4)
     * -------------------------------------------------------------------------- */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_cd[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x02, 0xB4 };
    for (size_t i = 0; i < sizeof(pkt_switch_cd); i++) {
        can_router_process_uart_byte(pkt_switch_cd[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Confirm CD settled via CAN 0x165 */
    can_frame_t frame_cd = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_cd);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());

    /* --------------------------------------------------------------------------
     * 5. Downlink: Switch to Tuner & Query Radio State / Station List (5A A5 01 0F 0F)
     * -------------------------------------------------------------------------- */
    /* HU sends Switch to Tuner, then immediately queries 0x0F */
    for (size_t i = 0; i < sizeof(pkt_switch_tuner); i++) {
        can_router_process_uart_byte(pkt_switch_tuner[i]);
    }
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());

    hal_can_native_clear_sent_frame();
    const uint8_t pkt_query_0f[] = { 0x5A, 0xA5, 0x01, 0x0F, 0x0F };
    for (size_t i = 0; i < sizeof(pkt_query_0f); i++) {
        can_router_process_uart_byte(pkt_query_0f[i]);
    }

    /* While switching in progress, rotary scroll must be deferred */
    const psa_source_arbiter_t *arb = psa_source_arbiter_get_context();
    TEST_ASSERT_TRUE(arb->pending_station_list_trigger);

    /* Confirm Tuner arrival via CAN 0x165 -> triggers deferred rotary scroll */
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 } /* 0x10 = Tuner */
    };
    can_router_process_can(&frame_165);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_FALSE(arb->pending_station_list_trigger);

    /* Verify CAN 0x21F station list trigger was transmitted upon settling */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_sent));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_sent.id);

    /* Verify UART responses: Universal ACK (0xFF, 0x0F) and Cmd 0x84 */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_ack_0f = false;
    bool found_84_radio = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len) {
            if (rx_buf[i + 3] == 0xFF && rx_buf[i + 4] == 0x0F) {
                found_ack_0f = true;
            }
            if (rx_buf[i + 3] == 0x84) {
                found_84_radio = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_ack_0f);
    TEST_ASSERT_TRUE(found_84_radio);
}

void test_integration_hiworld_source_switch_suppression(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_source_arbiter_init();

    /* Set radio initially to Tuner via CAN 0x165 */
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_165);

    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* Downlink: Switch RD4 to CD (5A A5 02 AD 03 02 B4) */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_cd[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x02, 0xB4 };
    for (size_t i = 0; i < sizeof(pkt_switch_cd); i++) {
        can_router_process_uart_byte(pkt_switch_cd[i]);
    }

    /* 1. Verify Stalk 0x21F pulse was injected */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_source_filtered());

    /* 2. Before RD4 acknowledges, 0x84 must NOT be sent to HU */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84_premature = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_84_premature = true;
        }
    }
    TEST_ASSERT_FALSE(found_84_premature);

    /* 3. RD4 acknowledges by sending CAN 0x165 (0x20 in high nibble = CD) */
    can_frame_t frame_cd = {
        .id = 0x165,
        .dlc = 8,
        .data = { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&frame_cd);

    /* 4. Target reached! Verify final confirmed state pushed to HU: Cmd 0x84 */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len && rx_buf[i + 3] == 0x84) {
            found_84 = true;
        }
    }
    TEST_ASSERT_TRUE(found_84);
}

void test_integration_hiworld_multi_hop_source_switch(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_source_arbiter_init();

    /* 1. Initialize RD4 on AUX 1 (0x40 in CAN 0x165) */
    can_frame_t frame_aux = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x40, 0x00 }
    };
    can_router_process_can(&frame_aux);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    /* 2. HU requests CD: 5A A5 02 AD 03 02 B3 */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_switch_cd[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x02, 0xB3 };
    for (size_t i = 0; i < sizeof(pkt_switch_cd); i++) {
        can_router_process_uart_byte(pkt_switch_cd[i]);
    }

    /* Pulse 1: Injected immediately */
    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());

    /* 3. RD4 reacts to Pulse 1: transitions from AUX 1 to Tuner (0x10) */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_tuner = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x10, 0x00 }
    };
    can_router_process_can(&frame_tuner);

    /* Pulse 2: Injected after Tuner received */
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_TRUE(psa_source_arbiter_is_switching_in_progress());

    /* 4. RD4 reacts to Pulse 2: transitions from Tuner to CD (0x20) */
    hal_can_native_clear_sent_frame();
    can_frame_t frame_cd = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0x80, 0x00, 0x20, 0x00 }
    };
    can_router_process_can(&frame_cd);

    /* 5. Target reached! No Pulse 3 must be emitted */
    TEST_ASSERT_FALSE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_CD, psa_source_arbiter_get_reported_source());

    /* Verify final confirmed state pushed to HU: Cmd 0x84 with 0x20 (CD) */
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84_cd = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len && rx_buf[i + 3] == 0x84) {
            if (rx_buf[i + 4] == 0x20) {
                found_84_cd = true;
            }
        }
    }
    TEST_ASSERT_TRUE(found_84_cd);
}

void test_integration_hiworld_cmd_0xad_03_03_and_02_01_switch_to_line_in_1(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_router_init();
    psa_source_arbiter_init();

    /* Put radio on Tuner */
    can_frame_t frame_tuner = {
        .id = 0x165, .dlc = 4, .data = { 0x80, 0x00, 0x10, 0x00 }
    };
    can_router_process_can(&frame_tuner);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());

    uint8_t rx_buf[128];
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 1. Command 5A A5 02 AD 03 03 B5 (SubCmd 0x03 Param 0x03) switches RD4 from Tuner to Line In 1 (AUX 1) */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_ad_03_03[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x03, 0x03, 0xB5 };
    for (size_t i = 0; i < sizeof(pkt_ad_03_03); i++) {
        can_router_process_uart_byte(pkt_ad_03_03[i]);
    }

    can_frame_t frame_pulse;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());

    /* RD4 confirms Line In 1 / AUX 1 (0x40) */
    can_frame_t frame_aux1 = {
        .id = 0x165, .dlc = 4, .data = { 0x80, 0x00, 0x40, 0x00 }
    };
    can_router_process_can(&frame_aux1);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found_84_aux1 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len && rx_buf[i + 3] == 0x84 && rx_buf[i + 4] == 0x40) {
            found_84_aux1 = true;
        }
    }
    TEST_ASSERT_TRUE(found_84_aux1);

    /* 2. Switch back to Tuner */
    psa_source_arbiter_switch_source(PSA_RD4_SRC_TUNER);
    can_router_process_can(&frame_tuner);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_TUNER, psa_source_arbiter_get_reported_source());
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 3. Command 5A A5 02 AD 02 01 B1 (SubCmd 0x02 Param 0x01) also switches to Line In 1 (AUX 1) */
    hal_can_native_clear_sent_frame();
    const uint8_t pkt_ad_02_01[] = { 0x5A, 0xA5, 0x02, 0xAD, 0x02, 0x01, 0xB1 };
    for (size_t i = 0; i < sizeof(pkt_ad_02_01); i++) {
        can_router_process_uart_byte(pkt_ad_02_01[i]);
    }

    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&frame_pulse));
    TEST_ASSERT_EQUAL_HEX32(0x21F, frame_pulse.id);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_target_source());

    can_router_process_can(&frame_aux1);
    TEST_ASSERT_FALSE(psa_source_arbiter_is_switching_in_progress());
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_SRC_AUX, psa_source_arbiter_get_reported_source());

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    found_84_aux1 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 4) < rx_len && rx_buf[i + 3] == 0x84 && rx_buf[i + 4] == 0x40) {
            found_84_aux1 = true;
        }
    }
    TEST_ASSERT_TRUE(found_84_aux1);
}

