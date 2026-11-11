#include "unity.h"
#include "core/can_sniffer.h"
#include "core/can_router.h"
#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"
#include "hal/hal_can.h"
#include "hal/hal_uart.h"
#include "hal/hal_gpio.h"
#include <string.h>

extern void hal_uart_native_clear_tx_history(void);
extern size_t hal_uart_native_get_tx_history(uint8_t *out_buf, size_t max_len);

void setUp_can_sniffer(void) {
    can_router_init();
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    can_sniffer_init();
    hal_uart_native_clear_tx_history();
}

/**
 * @brief Test 1: 0xD0 Downlink control (Start / Stop) and standard 0xFF ACK.
 */
void test_can_sniffer_0xd0_start_stop_and_ack(void) {
    setUp_can_sniffer();

    /* 1. Downlink 0xD0 Start Sniffer: 5A A5 02 D0 01 1E F0 (Mode 1, 30s) */
    const uint8_t start_cmd[] = { 0x5A, 0xA5, 0x02, 0xD0, 0x01, 0x1E, 0xF0 };
    hal_uart_native_clear_tx_history();

    for (size_t i = 0; i < sizeof(start_cmd); i++) {
        hu_protocol_feed_byte(start_cmd[i]);
    }

    TEST_ASSERT_TRUE(can_sniffer_is_active());

    /* Check ACK 0xFF frame: 5A A5 01 FF D0 CF */
    uint8_t tx_buf[64];
    size_t tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(6, tx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, tx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, tx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, tx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, tx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0xD0, tx_buf[4]);
    TEST_ASSERT_EQUAL_HEX8(0xCF, tx_buf[5]);

    /* 2. Downlink 0xD0 Stop Sniffer: 5A A5 02 D0 00 1E EF (Mode 0) */
    hal_uart_native_clear_tx_history();
    const uint8_t stop_cmd[] = { 0x5A, 0xA5, 0x02, 0xD0, 0x00, 0x1E, 0xEF };
    for (size_t i = 0; i < sizeof(stop_cmd); i++) {
        hu_protocol_feed_byte(stop_cmd[i]);
    }

    tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(6, tx_len);
    TEST_ASSERT_EQUAL_HEX8(0xFF, tx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0xD0, tx_buf[4]);

    /* Drain summary poll */
    can_sniffer_poll_10ms(31000);
    TEST_ASSERT_FALSE(can_sniffer_is_active());
}

/**
 * @brief Test 2: Deduplication of unchanged periodic frames vs immediate delta trigger on 1-bit payload change.
 */
void test_can_sniffer_dedup_and_delta_trigger(void) {
    setUp_can_sniffer();
    can_sniffer_start(SNIFFER_MODE_DELTA, 30, 0);
    TEST_ASSERT_TRUE(can_sniffer_is_active());

    /* Initial discovery of CAN ID 0x1E3 (HVAC) */
    can_frame_t f1;
    memset(&f1, 0, sizeof(f1));
    f1.id = 0x1E3;
    f1.dlc = 7;
    const uint8_t payload_a[7] = { 0x1D, 0x30, 0x0B, 0x0B, 0x00, 0x00, 0x02 };
    memcpy(f1.data, payload_a, 7);

    can_sniffer_on_can_frame(&f1, false, 100);

    /* Poll at t=100ms: Should emit SUBTYPE 0x01 with initial discovery record */
    hal_uart_native_clear_tx_history();
    can_sniffer_poll_10ms(100);

    uint8_t tx_buf[128];
    size_t tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    /* Frame: 5A A5 <len> D1 <subtype 0x01> <record (7+7=14 bytes)> <cs> -> total 20 bytes */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(20, tx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, tx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, tx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(15, tx_buf[2]); /* 1 subtype + 14 record bytes */
    TEST_ASSERT_EQUAL_HEX8(0xD1, tx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, tx_buf[4]); /* Subtype 0x01: Live */

    /* Verify record fields */
    uint16_t dt = tx_buf[5] | (tx_buf[6] << 8);
    TEST_ASSERT_EQUAL_UINT16(100, dt);
    uint16_t id_flags = tx_buf[7] | (tx_buf[8] << 8);
    TEST_ASSERT_EQUAL_HEX16(0x1E3 | (7 << 11), id_flags); /* CAN ID 0x1E3, DLC 7, RX (bit15=0) */
    uint16_t repeats = tx_buf[9] | (tx_buf[10] << 8);
    TEST_ASSERT_EQUAL_UINT16(0, repeats); /* 0 repeats on discovery */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload_a, &tx_buf[12], 7);

    /* Repeat 5 identical frames at 200, 300, 400, 500, 600 ms */
    for (uint32_t t = 200; t <= 600; t += 100) {
        can_sniffer_on_can_frame(&f1, false, t);
        hal_uart_native_clear_tx_history();
        can_sniffer_poll_10ms(t);
        /* Must NOT emit anything for unchanged frames! */
        TEST_ASSERT_EQUAL_UINT32(0, hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf)));
    }

    /* Transition: 1 bit flipped in byte 2 (0x0B -> 0x0C) at t=700ms */
    f1.data[2] = 0x0C;
    can_sniffer_on_can_frame(&f1, false, 700);

    /* Poll at t=700ms: Should emit SUBTYPE 0x01 with repeat_count=5 and period_10ms=10 (100ms) */
    hal_uart_native_clear_tx_history();
    can_sniffer_poll_10ms(700);

    tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(20, tx_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, tx_buf[4]); /* Subtype 0x01 */

    dt = tx_buf[5] | (tx_buf[6] << 8);
    TEST_ASSERT_EQUAL_UINT16(700, dt);
    repeats = tx_buf[9] | (tx_buf[10] << 8);
    TEST_ASSERT_EQUAL_UINT16(5, repeats); /* Exactly 5 prior repetitions */
    uint8_t period_10ms = tx_buf[11];
    TEST_ASSERT_EQUAL_UINT8(10, period_10ms); /* 10 * 10ms = 100ms interval */
    TEST_ASSERT_EQUAL_HEX8(0x0C, tx_buf[14]); /* Flipped byte verified */
}

/**
 * @brief Test 3: CAN_TX logging and software ID filter bypass during sniffer mode.
 */
void test_can_sniffer_tx_logging_and_filter_bypass(void) {
    setUp_can_sniffer();
    can_sniffer_start(SNIFFER_MODE_DELTA, 30, 0);

    /* Frame 1: Unmapped standard CAN ID (0x7AA) sent to can_router_process_can() */
    can_frame_t unmapped;
    memset(&unmapped, 0, sizeof(unmapped));
    unmapped.id = 0x7AA;
    unmapped.dlc = 4;
    unmapped.data[0] = 0xDE;
    unmapped.data[1] = 0xAD;
    unmapped.data[2] = 0xBE;
    unmapped.data[3] = 0xEF;

    /* Verify router filter normally rejects it */
    TEST_ASSERT_FALSE(can_router_is_id_allowed(unmapped.id));

    /* But router process_can feeds it to sniffer BEFORE filter check! */
    can_router_process_can(&unmapped);

    hal_uart_native_clear_tx_history();
    can_sniffer_poll_10ms(50);

    uint8_t tx_buf[64];
    size_t tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(16, tx_len);
    uint16_t id_flags = tx_buf[7] | (tx_buf[8] << 8);
    TEST_ASSERT_EQUAL_HEX16(0x7AA | (4 << 11), id_flags);

    /* Frame 2: Outgoing CAN_TX frame injected by canbox-core */
    can_frame_t tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.id = 0x15B;
    tx_frame.dlc = 8;
    tx_frame.data[0] = 0x01;
    tx_frame.data[1] = 0x03;

    hal_can_send(&tx_frame);

    hal_uart_native_clear_tx_history();
    can_sniffer_poll_10ms(60);

    tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(20, tx_len);
    id_flags = tx_buf[7] | (tx_buf[8] << 8);
    /* Bit 15 (0x8000) must be SET for CAN_TX! */
    TEST_ASSERT_TRUE((id_flags & 0x8000) != 0);
    TEST_ASSERT_EQUAL_HEX16(0x15B, id_flags & 0x07FF);
    TEST_ASSERT_EQUAL_UINT8(8, (id_flags >> 11) & 0x0F);
}

/**
 * @brief Test 4: Paced flushing (<= 52 bytes payload / <= 57 bytes wire) and End-of-Capture summary + 0xFF finished marker.
 */
void test_can_sniffer_pacing_and_summary_flush(void) {
    setUp_can_sniffer();
    can_sniffer_start(SNIFFER_MODE_DELTA, 30, 0);

    /* Burst 10 distinct CAN IDs into the sniffer to generate multiple live batch records */
    for (uint16_t id = 0x100; id < 0x10A; id++) {
        can_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id = id;
        f.dlc = 8;
        memset(f.data, (uint8_t)id, 8);
        can_sniffer_on_can_frame(&f, false, 100);
    }

    /* Verify each poll_10ms emits AT MOST ONE frame <= 57 bytes */
    for (int tick = 0; tick < 4; tick++) {
        hal_uart_native_clear_tx_history();
        can_sniffer_poll_10ms(100 + tick * 10);
        uint8_t tx_buf[128];
        size_t tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
        if (tx_len > 0) {
            TEST_ASSERT_LESS_OR_EQUAL_UINT32(57, tx_len);
            /* Check wire structure: 5A A5 <len> D1 <0x01> ... <cs> */
            TEST_ASSERT_EQUAL_HEX8(0x5A, tx_buf[0]);
            TEST_ASSERT_EQUAL_HEX8(0xA5, tx_buf[1]);
            TEST_ASSERT_EQUAL_HEX8(0xD1, tx_buf[3]);
            TEST_ASSERT_EQUAL_HEX8(0x01, tx_buf[4]);
        }
    }

    /* Stop sniffer at 30 seconds */
    can_sniffer_stop(30000);
    TEST_ASSERT_TRUE(can_sniffer_is_active()); /* Still flushing summary */

    /* Poll summary: Should emit 0x02 summary batches then 0xFF finished marker */
    bool saw_summary = false;
    bool saw_finished = false;

    for (int tick = 0; tick < 10; tick++) {
        hal_uart_native_clear_tx_history();
        can_sniffer_poll_10ms(30000 + (tick + 1) * 10);
        uint8_t tx_buf[128];
        size_t tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
        if (tx_len > 0) {
            TEST_ASSERT_LESS_OR_EQUAL_UINT32(57, tx_len);
            if (tx_buf[4] == 0x02) {
                saw_summary = true;
            } else if (tx_buf[4] == 0xFF) {
                saw_finished = true;
                /* Finished packet: 5A A5 04 D1 FF <TOTAL_UNIQUE_IDS> <DROPPED_EVENTS_LE16> <CS> */
                TEST_ASSERT_EQUAL_HEX8(0x04, tx_buf[2]);
                TEST_ASSERT_EQUAL_UINT8(10, tx_buf[5]); /* 10 unique IDs seen */
                uint16_t dropped = tx_buf[6] | (tx_buf[7] << 8);
                TEST_ASSERT_EQUAL_UINT16(0, dropped);
                break;
            }
        }
    }

    TEST_ASSERT_TRUE(saw_summary);
    TEST_ASSERT_TRUE(saw_finished);
    TEST_ASSERT_FALSE(can_sniffer_is_active()); /* Closed and idle */
}

/**
 * @brief Test 5: Verify HU action command suppression, lighting GPIO lock, and ACC status lock during sniffing.
 */
void test_can_sniffer_suppresses_hu_actions_locks_acc_and_lights(void) {
    setUp_can_sniffer();
    hal_gpio_init();

    /* Start sniffer */
    can_sniffer_start(SNIFFER_MODE_DELTA, 30, 0);
    TEST_ASSERT_TRUE(can_sniffer_is_active());

    /* Requirement 1: ACC must be ON during sniffing */
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    /* Lighting initially off */
    bool initial_ill = hal_gpio_read(GPIO_PIN_ILL_OUT);

    /* Clear any UART frames */
    hal_uart_native_clear_tx_history();

    /* 1. Simulate steering wheel button CAN frame (0x21F) */
    can_frame_t swc_frame;
    memset(&swc_frame, 0, sizeof(swc_frame));
    swc_frame.id = 0x21F;
    swc_frame.dlc = 3;
    swc_frame.data[0] = 0x04; /* Volume Up */
    can_router_process_can(&swc_frame);

    /* 2. Simulate reverse gear CAN frame (0x0F6) */
    can_frame_t rev_frame;
    memset(&rev_frame, 0, sizeof(rev_frame));
    rev_frame.id = 0x0F6;
    rev_frame.dlc = 8;
    rev_frame.data[7] = 0x80; /* Reverse gear active */
    can_router_process_can(&rev_frame);

    /* 3. Simulate climate CAN frame (0x1E3) */
    can_frame_t clim_frame;
    memset(&clim_frame, 0, sizeof(clim_frame));
    clim_frame.id = 0x1E3;
    clim_frame.dlc = 7;
    clim_frame.data[0] = 0x5D; /* Climate blower changed */
    can_router_process_can(&clim_frame);

    /* 4. Simulate door open CAN frame (0x220) */
    can_frame_t door_frame;
    memset(&door_frame, 0, sizeof(door_frame));
    door_frame.id = 0x220;
    door_frame.dlc = 5;
    door_frame.data[0] = 0x01; /* Driver door open */
    can_router_process_can(&door_frame);

    /* 5. Simulate lighting change CAN frame (0x128) */
    can_frame_t light_frame;
    memset(&light_frame, 0, sizeof(light_frame));
    light_frame.id = 0x128;
    light_frame.dlc = 7;
    light_frame.data[4] = 0x07; /* Headlights & sidelights ON */
    can_router_process_can(&light_frame);

    /* Verify no action frames sent to HU over UART (only sniffer records if drained via poll) */
    uint8_t tx_buf[256];
    size_t tx_len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
    /* Router must not have written ANY standard UI command (0x11, 0x12, 0x21, 0x31, 0x41) */
    TEST_ASSERT_EQUAL_UINT32(0, tx_len);

    /* Requirement 2: Lighting GPIO must NOT have changed */
    TEST_ASSERT_EQUAL(initial_ill, hal_gpio_read(GPIO_PIN_ILL_OUT));

    /* Reverse GPIO must NOT have triggered */
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    /* Requirement 3: ACC status must stay strictly ON */
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));

    /* 6. Run 35 ticks (3.5 seconds) of silence to test bus inactivity watchdog */
    for (int i = 0; i < 35; i++) {
        can_router_periodic_100ms();
    }

    /* Bus must NOT sleep during sniffing and ACC must remain ON */
    TEST_ASSERT_FALSE(can_router_is_bus_sleeping());
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_HEADUNIT_POWER));
}

/**
 * @brief Test 6: Verify hard 30s ceiling and automatic stop without HU stop command.
 */
void test_can_sniffer_hard_30s_timeout_without_hu_stop(void) {
    setUp_can_sniffer();

    /* Start with requested duration = 60s (should be clamped to 30s) */
    can_sniffer_start(SNIFFER_MODE_DELTA, 60, 0);
    TEST_ASSERT_TRUE(can_sniffer_is_active());

    /* Feed one CAN frame */
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = 0x1E3;
    f.dlc = 7;
    f.data[0] = 0xAA;
    can_sniffer_on_can_frame(&f, false, 500);

    /* Drain initial live event */
    can_sniffer_poll_10ms(500);

    /* At t = 29.9s, sniffer is still active */
    can_sniffer_poll_10ms(29900);
    TEST_ASSERT_TRUE(can_sniffer_is_active());

    /* At t = 30.0s, duration expires -> automatically transitions to summary flush */
    can_sniffer_poll_10ms(30000);
    TEST_ASSERT_TRUE(can_sniffer_is_active());

    /* Drain summary packets */
    bool saw_finished = false;
    for (int i = 0; i < 10; i++) {
        hal_uart_native_clear_tx_history();
        can_sniffer_poll_10ms(30000 + (i + 1) * 10);
        uint8_t tx_buf[64];
        size_t len = hal_uart_native_get_tx_history(tx_buf, sizeof(tx_buf));
        if (len >= 5 && tx_buf[4] == 0xFF) {
            saw_finished = true;
            break;
        }
    }

    TEST_ASSERT_TRUE(saw_finished);
    TEST_ASSERT_FALSE(can_sniffer_is_active());

    /* At t = 31.0s, sniffer is completely idle and drops any further frames */
    hal_uart_native_clear_tx_history();
    can_frame_t f2;
    memset(&f2, 0, sizeof(f2));
    f2.id = 0x21F;
    f2.dlc = 3;
    can_sniffer_on_can_frame(&f2, false, 31000);
    can_sniffer_poll_10ms(31000);

    uint8_t tx_buf2[64];
    size_t len2 = hal_uart_native_get_tx_history(tx_buf2, sizeof(tx_buf2));
    TEST_ASSERT_EQUAL_UINT32(0, len2);
}
