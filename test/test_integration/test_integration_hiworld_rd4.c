#include <string.h>
#include "test_integration_common.h"
#include "core/vehicle_profile.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"
#include "profiles/psa_virtual_cdc.h"
#include "hal/hal_gpio.h"


static void check_3e5_pipeline(const uint8_t *can6, const uint8_t *expected_press) {
    uint8_t rx_buf[32];
    size_t rx_len;
    const uint8_t idle[6] = { 0 };
    const uint8_t expected_release[] = { 0x5A, 0xA5, 0x02, 0x21, 0x00, 0x00, 0x22 };
    can_frame_t f = { .id = 0x3E5, .dlc = 6 };

    memcpy(f.data, can6, 6);
    can_router_process_can(&f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(7, rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_press, rx_buf, 7);

    memcpy(f.data, idle, 6);
    can_router_process_can(&f);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_release), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_release, rx_buf, sizeof(expected_release));
}

/* CAN frames from can_log_buttons*.log -> wire frames from the original OEM Hiworld Canbox */
void test_integration_hiworld_console_0x3e5_full_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    static const uint8_t audio[6] = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t trip[6]  = { 0x00, 0x40, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t clim[6]  = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t dark[6]  = { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00 };
    static const uint8_t menu[6]  = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t esc[6]   = { 0x00, 0x00, 0x10, 0x00, 0x00, 0x00 };
    static const uint8_t up[6]    = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x40 };
    static const uint8_t down[6]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };
    static const uint8_t right[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };
    static const uint8_t tel[6]   = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t left[6]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };

    static const uint8_t w_audio[] = { 0x5A, 0xA5, 0x02, 0x21, 0x31, 0x01, 0x54 };
    static const uint8_t w_trip[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x40, 0x01, 0x63 };
    static const uint8_t w_clim[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x28, 0x01, 0x4B };
    static const uint8_t w_dark[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x07, 0x01, 0x2A };
    static const uint8_t w_menu[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x2E, 0x01, 0x51 };
    static const uint8_t w_esc[]   = { 0x5A, 0xA5, 0x02, 0x21, 0x25, 0x01, 0x48 };
    static const uint8_t w_up[]    = { 0x5A, 0xA5, 0x02, 0x21, 0x17, 0x01, 0x3A };
    static const uint8_t w_down[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x18, 0x01, 0x3B };
    static const uint8_t w_tel[]   = { 0x5A, 0xA5, 0x02, 0x21, 0x05, 0x01, 0x28 };
    static const uint8_t w_left[]  = { 0x5A, 0xA5, 0x02, 0x21, 0x19, 0x01, 0x3C };
    static const uint8_t w_right[] = { 0x5A, 0xA5, 0x02, 0x21, 0x1A, 0x01, 0x3D };

    check_3e5_pipeline(audio, w_audio);
    check_3e5_pipeline(trip, w_trip);
    check_3e5_pipeline(clim, w_clim);
    check_3e5_pipeline(dark, w_dark);
    check_3e5_pipeline(dark, w_dark); /* DARK pressed twice in the log */
    check_3e5_pipeline(menu, w_menu);
    check_3e5_pipeline(esc, w_esc);
    check_3e5_pipeline(up, w_up);
    check_3e5_pipeline(down, w_down);
    check_3e5_pipeline(tel, w_tel);
    check_3e5_pipeline(left, w_left);
    check_3e5_pipeline(right, w_right);
}

void test_integration_hiworld_console_0x3e5_ok_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    /* OK: can_log_buttons1.log frame 000040000000 */
    static const uint8_t ok[6] = { 0x00, 0x00, 0x40, 0x00, 0x00, 0x00 };
    static const uint8_t w_ok[] = { 0x5A, 0xA5, 0x02, 0x21, 0x24, 0x01, 0x47 };
    check_3e5_pipeline(ok, w_ok);
}

void test_integration_hiworld_steering_angle_trajectory_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    uint8_t rx_buf[64];
    size_t rx_len;

    /* Steering frames outside reverse gear must not emit trajectory data */
    can_frame_t sas = { .id = 0x10B, .dlc = 8, .data = { 0x00, 0xD2, 0, 0, 0, 0, 0, 0 } };
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* Engage reverse (0x0F6 Byte 7 bit 7): current angle (+21 deg Left) is pushed */
    can_frame_t rev = { .id = 0x0F6, .dlc = 8, .data = { 0x8E, 0x58, 0, 0, 0, 0, 0, 0x80 } };
    can_router_process_can(&rev);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_21l[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xEB, 0x5E, 0x22, 0xAB };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_21l), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_21l, rx_buf, sizeof(exp_21l));

    /* Centered */
    memset(sas.data, 0, sizeof(sas.data));
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_c[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x22, 0xC1 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_c), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_c, rx_buf, sizeof(exp_c));

    /* Unchanged angle: no retransmission (delta detection) */
    can_router_process_can(&sas);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));

    /* Left +60.0 deg */
    sas.data[0] = 0x02; sas.data[1] = 0x58;
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_60l[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC4, 0x5E, 0x22, 0x84 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_60l), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_60l, rx_buf, sizeof(exp_60l));

    /* Right -90.0 deg */
    sas.data[0] = 0xFC; sas.data[1] = 0x7C;
    can_router_process_can(&sas);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    const uint8_t exp_90r[] = { 0x5A, 0xA5, 0x0A, 0x11, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0x5E, 0x22, 0x1B };
    TEST_ASSERT_EQUAL_UINT32(sizeof(exp_90r), rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp_90r, rx_buf, sizeof(exp_90r));

    /* Sensor error flag in Byte 3: frame discarded */
    sas.data[0] = 0x00; sas.data[1] = 0x00; sas.data[3] = 0x01;
    can_router_process_can(&sas);
    TEST_ASSERT_EQUAL_UINT32(0, read_uart_output(rx_buf, sizeof(rx_buf)));
}

void test_integration_hiworld_outside_temp_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[256];
    (void)read_uart_output(rx_buf, sizeof(rx_buf)); /* Drain pending output */

    /* Vehicle log: 0x0F6 byte 5 = 0x7C -> 0x7C * 0.5 - 40 = 22 C */
    can_frame_t bsi_0f6 = {
        .id = 0x0F6,
        .dlc = 8,
        .data = { 0x88, 0x64, 0xFE, 0xFE, 0xFF, 0x7C, 0x7C, 0x00 }
    };
    can_router_process_can(&bsi_0f6);

    TEST_ASSERT_EQUAL_HEX8(0x7C, can_router_get_state()->climate.outdoor_temp_raw);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool found = false;
    for (size_t i = 0; i + 16 <= rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i + 1] == 0xA5 && rx_buf[i + 3] == 0x31) {
            TEST_ASSERT_EQUAL_HEX8(0x0C, rx_buf[i + 2]);
            TEST_ASSERT_EQUAL_HEX8(0x7C, rx_buf[i + 4 + 11]); /* Payload byte 11 */
            found = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "Hiworld 0x31 climate frame with outside temp not sent");

    /* 0xFF is invalid: previous value is retained */
    bsi_0f6.data[5] = 0xFF;
    can_router_process_can(&bsi_0f6);
    TEST_ASSERT_EQUAL_HEX8(0x7C, can_router_get_state()->climate.outdoor_temp_raw);
}

void test_integration_hiworld_rd4_volume_and_audio_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);
    can_router_rebuild_filter();

    uint8_t rx_buf[256];
    (void)read_uart_output(rx_buf, sizeof(rx_buf)); /* Drain pending UART */

    // 1. Inject Volume Frame 0x1A5: Vol=18 (0x12), Unmuted
    can_frame_t frame_vol = {
        .id = 0x1A5,
        .dlc = 1,
        .data = { 0x12 }
    };
    can_router_process_can(&frame_vol);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x56, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[4]); // Mode 1 (Volume active), unmuted
    TEST_ASSERT_EQUAL_UINT8(18, rx_buf[11]); // Master Volume

    // 2. Inject Sound Settings 0x1E5: Bass active (0xBF = 0x3F | 0x80), Treble=9, Bal=9, Fad=9, Loudness=ON, SpdVol=ON
    can_frame_t frame_audio = {
        .id = 0x1E5,
        .dlc = 7,
        .data = { 0x3F, 0x3F, 0xBF, 0x00, 0x3F, 0x47, 0x00 }
    };
    can_router_process_can(&frame_audio);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, rx_buf[4]); // Mode 2 (Bass active), unmuted
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[5]);  // Bass = 9
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[6]);  // Treble = 9
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[7]);  // Balance = 9
    TEST_ASSERT_EQUAL_UINT8(9, rx_buf[8]);  // Fader = 9
    TEST_ASSERT_EQUAL_HEX8(0x11, rx_buf[10]); // Loudness ON (0x10) | SpdVol ON (0x01)
    TEST_ASSERT_EQUAL_UINT8(18, rx_buf[11]); // Master volume retained

    // 3. Menu Dismissal: 0x1E5 arrives with all active mode bits = 0
    frame_audio.data[2] = 0x3F; // Bass active bit cleared
    can_router_process_can(&frame_audio);

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[4]); // Mode 0 (Menu dismissal HUD dismiss)

    // 4. Repeated unchanged 0x1E5 frame: zero jitter / no UART traffic
    can_router_process_can(&frame_audio);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    // 5. Idle Menu Closed Suppression: 10 ticks of can_router_periodic_100ms() MUST NOT emit 0x56 when idle
    for (int i = 0; i < 10; i++) {
        can_router_periodic_100ms();
    }
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool idle_audio_sync_sent = false;
    for (size_t i = 0; i + 13 <= rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i + 1] == 0xA5 && rx_buf[i + 2] == 0x08 && rx_buf[i + 3] == 0x56) {
            idle_audio_sync_sent = true;
            break;
        }
    }
    TEST_ASSERT_FALSE_MESSAGE(idle_audio_sync_sent, "Audio Cmd 0x56 was unexpectedly sent periodically while menu was closed!");

    // 6. Active Menu Refresh: while active_mode >= 2 (e.g. Bass Mode 2 or Balance Mode 4),
    // periodic timer MUST transmit Cmd 0x56 every 1000ms with the active mode to keep HU menu open!
    frame_audio.data[2] = 0xBF; // Re-enter Bass active mode (Mode 2)
    can_router_process_can(&frame_audio);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, rx_buf[4]); // Mode 2

    // Run 10 ticks (1000ms) of periodic timer while in active mode
    for (int i = 0; i < 9; i++) {
        can_router_periodic_100ms();
    }
    (void)read_uart_output(rx_buf, sizeof(rx_buf)); // Drain heartbeats

    // 10th tick must fire active menu refresh
    can_router_periodic_100ms();
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    bool active_refresh_sent = false;
    for (size_t i = 0; i + 13 <= rx_len; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i + 1] == 0xA5 && rx_buf[i + 2] == 0x08 && rx_buf[i + 3] == 0x56) {
            TEST_ASSERT_EQUAL_HEX8(0x02, rx_buf[i + 4]); // Must be Mode 2 (NOT 0!)
            TEST_ASSERT_EQUAL_UINT8(18, rx_buf[i + 11]); // Master Vol
            active_refresh_sent = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(active_refresh_sent, "Periodic timer failed to refresh active DSP menu after 1000ms!");
    TEST_ASSERT_EQUAL_UINT8(2, can_router_get_state()->audio.active_mode); // State must retain mode 2!

    // 7. Volume knob stopped / idle frame (0xEE = 0xE0 | 14 from vehicle log):
    // Must dismiss volume mode 1 -> 0 WITHOUT sending mute bit (D0 must be 0x00, NOT 0x80)
    can_frame_t frame_vol_turning = {
        .id = 0x1A5,
        .dlc = 1,
        .data = { 0x0F } // Vol 15, rotating
    };
    can_router_process_can(&frame_vol_turning);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[4]); // Mode 1, unmuted
    TEST_ASSERT_EQUAL_UINT8(15, rx_buf[11]);

    can_frame_t frame_vol_stopped = {
        .id = 0x1A5,
        .dlc = 1,
        .data = { 0xEE } // Vol 14, stopped / idle (0xE0 | 14)
    };
    can_router_process_can(&frame_vol_stopped);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(13, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[4]); // Dismissal: Mode 0, unmuted (0x00, NOT 0x80!)
    TEST_ASSERT_EQUAL_UINT8(14, rx_buf[11]); // Master Vol = 14

    // Repeated identical idle frame: zero UART traffic
    can_router_process_can(&frame_vol_stopped);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);
}

void test_integration_hiworld_rd4_radio_tuner_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Source Tuner (0x165)
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_165);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 2. Tuner Frequency (0x225): FM1, Preset 1, RDS Lock, 102.50 MHz
    can_frame_t frame_225 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x01, 0x90, 0x04, 0x1A }
    };
    can_router_process_can(&frame_225);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 3. Station Name (0x2A5): "RMF FM  "
    can_frame_t frame_2a5 = {
        .id = 0x2A5,
        .dlc = 8,
        .data = { 'R', 'M', 'F', ' ', 'F', 'M', ' ', ' ' }
    };
    can_router_process_can(&frame_2a5);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= 19);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[4]); /* FM1 -> 0x01 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[5]); /* 1025 & 0xFF (Little Endian for Hiworld) */
    TEST_ASSERT_EQUAL_HEX8(0x04, rx_buf[6]); /* 1025 >> 8 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x20, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[9]);
    TEST_ASSERT_EQUAL_STRING_LEN("RMF FM  ", (char *)&rx_buf[10], 8);
}

void test_integration_hiworld_rd4_radiotext_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Source Tuner ON via 0x165
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_165);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 2. Inject ISO-TP multi-frame stream for "Queen - Bohemian Rhapsody" (29 bytes with 4-byte prefix)
    const uint8_t can_ff[]  = { 0x10, 0x1D, 0x10, 0x00, 0x00, 0x00, 0x51, 0x75 };
    const uint8_t can_cf1[] = { 0x21, 0x65, 0x65, 0x6E, 0x20, 0x2D, 0x20, 0x42 };
    const uint8_t can_cf2[] = { 0x22, 0x6F, 0x68, 0x65, 0x6D, 0x69, 0x61, 0x6E };
    const uint8_t can_cf3[] = { 0x23, 0x20, 0x52, 0x68, 0x61, 0x70, 0x73, 0x6F };
    const uint8_t can_cf4[] = { 0x24, 0x64, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00 };

    can_frame_t frame = { .id = 0x0A4, .dlc = 8 };

    memcpy(frame.data, can_ff, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf1, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf2, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf3, 8);
    can_router_process_can(&frame);
    memcpy(frame.data, can_cf4, 8);
    can_router_process_can(&frame);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(30, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(25, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x86, rx_buf[3]);
    TEST_ASSERT_EQUAL_STRING_LEN("Queen - Bohemian Rhapsody", (char *)&rx_buf[4], 25);
}

void test_integration_hiworld_rd4_cd_changer_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // CAN 0x3A6: Disc 2, Track 14, Total 20, 03:45, Random
    can_frame_t frame = {
        .id = 0x3A6,
        .dlc = 8,
        .data = { 0x00, 0x02, 0x0E, 0x14, 0x03, 0x2D, 0x01, 0x00 }
    };
    can_router_process_can(&frame);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(16, rx_len);

    /* 5A A5 0B 97 02 02 00 0E 00 03 2D 01 02 14 00 FA */
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x02, 0x02, 0x00, 0x0E, 0x00, 0x03, 0x2D, 0x01, 0x02, 0x14, 0x00, 0xFA
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx_buf, 16);
}

void test_integration_hiworld_rd4_cd_in_dash_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[64];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Audio Source switch to CD (CAN 0x165 -> 0x20 = CD)
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x20, 0x00 }
    };
    can_router_process_can(&frame_165);

    // Emits Cmd 0x84 (Audio Source = 0x20 CD)
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= 19);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x20, rx_buf[4]); /* CD Source */

    // 2. CD Disc Info (CAN 0x365: Total Tracks = 18 = 0x12)
    can_frame_t frame_365 = {
        .id = 0x365,
        .dlc = 5,
        .data = { 0x12, 0x26, 0x02, 0x00, 0x00 }
    };
    can_router_process_can(&frame_365);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 3. CD Play Telemetry (CAN 0x3A5: Track 5, Playing, 02 min 35 sec)
    can_frame_t frame_3a5 = {
        .id = 0x3A5,
        .dlc = 6,
        .data = { 0x05, 0x03, 0x19, 0x02, 0x23, 0x00 }
    };
    can_router_process_can(&frame_3a5);

    // Emits Cmd 0x97 (Single in-dash CD: 5A A5 0B 97 00 00 00 05 00 02 23 00 02 12 00 DF)
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(16, rx_len);

    const uint8_t expected_cd_ex1[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x05, 0x00, 0x02, 0x23, 0x00, 0x02, 0x12, 0x00, 0xDF
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_cd_ex1, rx_buf, 16);

    // 4. Switch back to Radio (CAN 0x165 -> 0x10 = FM1):
    // MUST send Cmd 0x97 stopping CD playback first, then Cmd 0x84 for radio tuner
    can_frame_t frame_radio = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_radio);

    /* Allow 800ms physical button debounce to settle */
    hal_delay_ms(850);
    can_router_periodic_100ms();

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Buffer contains Cmd 0x97 (16 bytes) followed by Cmd 0x84 (19 bytes) = 35 bytes */
    TEST_ASSERT_TRUE(rx_len >= 35);

    // Verify first packet is Cmd 0x97 with play_status = 0x01 (Stop/Pause)
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x97, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[12]); /* play_status = 0x01 (Pause/Stop) */

    // Verify second packet is Cmd 0x84 with Tuner mode
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[16]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[17]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[18]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[19]);

    // Inject uninitialized sentinels while on Radio (must produce zero UART output)
    can_frame_t frame_365_sentinel = {
        .id = 0x365,
        .dlc = 5,
        .data = { 0xFF, 0xFF, 0xFF, 0x00, 0x00 }
    };
    can_router_process_can(&frame_365_sentinel);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    can_frame_t frame_3a5_sentinel = {
        .id = 0x3A5,
        .dlc = 6,
        .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00 }
    };
    can_router_process_can(&frame_3a5_sentinel);
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);
}

void test_integration_hiworld_rd4_downlink_resume_queries(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Send Tuner Resume Query: forwardType(0x0F) -> 5A A5 00 0F 0E
    const uint8_t q_tuner[] = { 0x5A, 0xA5, 0x00, 0x0F, 0x0E };
    for (size_t i = 0; i < sizeof(q_tuner); i++) {
        can_router_process_uart_byte(q_tuner[i]);
    }

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= (6 + 19));
    /* Universal ACK frame: 5A A5 01 FF 0F 0E */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, rx_buf[4]);
    /* Radio State response: 5A A5 0E 84 ... */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[9]);

    // 2. Send Media Resume Query: forwardType(0x11) -> 5A A5 00 11 10
    const uint8_t q_media[] = { 0x5A, 0xA5, 0x00, 0x11, 0x10 };
    for (size_t i = 0; i < sizeof(q_media); i++) {
        can_router_process_uart_byte(q_media[i]);
    }

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(6 + 16, rx_len);
    /* Universal ACK frame: 5A A5 01 FF 11 10 */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x11, rx_buf[4]);
    /* Media State response: 5A A5 0B 97 ... */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x97, rx_buf[9]);

    // 3. Send Tuner UI Open Query: Cmd 0xA1 -> 5A A5 03 A1 00 01 FF A3
    const uint8_t q_tuner_a1[] = { 0x5A, 0xA5, 0x03, 0xA1, 0x00, 0x01, 0xFF, 0xA3 };
    for (size_t i = 0; i < sizeof(q_tuner_a1); i++) {
        can_router_process_uart_byte(q_tuner_a1[i]);
    }

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= (6 + 19));
    /* Universal ACK frame: 5A A5 01 FF A1 A0 */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0xA1, rx_buf[4]);
    /* Radio State response: 5A A5 0E 84 ... */
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[9]);

    /* CRITICAL: Verify that passive 0xA1 volume/source query does NOT trigger CAN 0x21F rotary scroll */
    can_frame_t frame_sent;
    if (hal_can_native_get_last_sent_frame(&frame_sent) && frame_sent.id == 0x21F) {
        TEST_ASSERT_EQUAL_HEX8(0x00, frame_sent.data[1]);
    }
}

void test_integration_hiworld_rd4_preset_list_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 0. Source Tuner ON via 0x165
    can_frame_t frame_165 = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xCC, 0x54, 0x10, 0x02 }
    };
    can_router_process_can(&frame_165);
    read_uart_output(rx_buf, sizeof(rx_buf));

    // 1. Inject CAN 0x225 with Preset 1 active (87.5 MHz)
    can_frame_t frame_225_p1 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x10, 0x10, 0x02, 0xEE }
    };
    can_router_process_can(&frame_225_p1);

    // Should output Cmd 0x84 (Radio State) followed by Cmd 0x85 (Preset list freqs - 18 bytes)
    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= (19 + 18));

    // Find Cmd 0x85 in the stream
    bool found_0x85 = false;
    for (size_t i = 0; i < rx_len - 4; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == 0x85) {
            found_0x85 = true;
            TEST_ASSERT_EQUAL_HEX8(0x0D, rx_buf[i+2]); /* Length 13 */
            TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[i+4]); /* Band FM1 */
            // Preset 1 frequency: 875 -> 0x036B
            TEST_ASSERT_EQUAL_HEX8(0x03, rx_buf[i+5]);
            TEST_ASSERT_EQUAL_HEX8(0x6B, rx_buf[i+6]);
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x85);

    // 2. Inject native CAN 0x125 ISO-TP frames broadcasting station list
    can_frame_t frame_125_ff  = { .id = 0x125, .dlc = 8, .data = { 0x10, 0x28, 0x10, 0x04, 0x00, 0x40, 0x4A, 0x45 } };
    can_frame_t frame_125_cf1 = { .id = 0x125, .dlc = 8, .data = { 0x21, 0x44, 0x59, 0x4E, 0x4B, 0x41, 0x20, 0xB0 } };
    can_frame_t frame_125_cf2 = { .id = 0x125, .dlc = 8, .data = { 0x22, 0x20, 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D } };
    can_frame_t frame_125_cf3 = { .id = 0x125, .dlc = 8, .data = { 0x23, 0x20, 0xB0, 0x52, 0x4D, 0x46, 0x20, 0x4D } };
    can_frame_t frame_125_cf4 = { .id = 0x125, .dlc = 8, .data = { 0x24, 0x41, 0x58, 0x58, 0xB0, 0x20, 0x54, 0x52 } };
    can_frame_t frame_125_cf5 = { .id = 0x125, .dlc = 7, .data = { 0x25, 0x4F, 0x4A, 0x4B, 0x41, 0x20, 0xB0 } };

    can_router_process_can(&frame_125_ff);
    can_router_process_can(&frame_125_cf1);
    can_router_process_can(&frame_125_cf2);
    can_router_process_can(&frame_125_cf3);
    can_router_process_can(&frame_125_cf4);
    can_router_process_can(&frame_125_cf5);

    // Should output Cmd 0x85 (Extended Station Names Mode - 54 bytes)
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_TRUE(rx_len >= 54);

    found_0x85 = false;
    for (size_t i = 0; i < rx_len - 4; i++) {
        if (rx_buf[i] == 0x5A && rx_buf[i+1] == 0xA5 && rx_buf[i+3] == 0x85) {
            found_0x85 = true;
            TEST_ASSERT_EQUAL_HEX8(0x31, rx_buf[i+2]); /* Length 49 */
            TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[i+4]); /* Band FM1 */
            TEST_ASSERT_EQUAL_STRING_LEN("JEDYNKA ", (char *)&rx_buf[i+5], 8);
            TEST_ASSERT_EQUAL_STRING_LEN(" RMF FM ", (char *)&rx_buf[i+13], 8);
            TEST_ASSERT_EQUAL_STRING_LEN("RMF MAXX", (char *)&rx_buf[i+21], 8);
            TEST_ASSERT_EQUAL_STRING_LEN(" TROJKA ", (char *)&rx_buf[i+29], 8);
            break;
        }
    }
    TEST_ASSERT_TRUE(found_0x85);
}

void test_integration_hiworld_rd4_aux_mode_pipeline(void) {
    hu_protocol_set_active(HU_PROTOCOL_HIWORLD);

    uint8_t rx_buf[128];
    read_uart_output(rx_buf, sizeof(rx_buf));

    /* 1. Inject CAN 0x165 selecting AUX Input 1 (src = 0x4 -> source_mode 0x40) */
    can_frame_t frame_aux = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x40, 0x00 }
    };
    can_router_process_can(&frame_aux);

    size_t rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Must output Cmd 0x84 (CarRadioState) indicating AUX 1 (0x40) */
    TEST_ASSERT_EQUAL_UINT32(19, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, rx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, rx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, rx_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, rx_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x40, rx_buf[4]); /* Source = AUX 1 (0x40) */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[5]); /* Frequency LSB = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[6]); /* Frequency MSB = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[7]); /* Preset slot = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_buf[8]); /* Indicators = 0 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rx_buf[9]); /* Power status = 1 (Playing) */
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&rx_buf[10], 8); /* Blank station name */

    /* Verify NO Cmd 0x85 (presets) was transmitted */
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x85) {
            TEST_FAIL_MESSAGE("Cmd 0x85 should NOT be sent when audio source is AUX");
        }
    }

    /* 2. Background CAN tuner frame 0x225 and station name 0x2A5 arrive from vehicle */
    can_frame_t frame_225 = {
        .id = 0x225,
        .dlc = 5,
        .data = { 0x20, 0x60, 0x90, 0x03, 0x14 } /* Preset 6, FM1, 89.4 MHz */
    };
    can_router_process_can(&frame_225);

    can_frame_t frame_2a5 = {
        .id = 0x2A5,
        .dlc = 8,
        .data = { 'J', 'E', 'D', 'Y', 'N', 'K', 'A', ' ' }
    };
    can_router_process_can(&frame_2a5);

    /* Must produce ZERO UART output: background tuner changes while in AUX must not spam serial or trigger Radio UI */
    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    TEST_ASSERT_EQUAL_UINT32(0, rx_len);

    /* 3. Simulate periodic 60s resync ticks (600 ticks) */
    for (int t = 0; t < 599; t++) {
        if (t % 10 == 0) {
            can_router_process_can(&frame_aux);
        }
        can_router_periodic_100ms();
    }
    /* Drain heartbeats accumulated during the 599 ticks */
    while (read_uart_output(rx_buf, sizeof(rx_buf)) > 0) {}

    /* 600th tick triggers the 60s periodic bulk resync */
    can_router_process_can(&frame_aux);
    can_router_periodic_100ms();

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Resync in AUX mode must emit Cmd 0x84 (AUX 1), NEVER Cmd 0x85 (presets) or Cmd 0x86 (radiotext) */
    bool found_0x84 = false;
    bool found_0x85 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            if (rx_buf[i + 3] == 0x84) found_0x84 = true;
            if (rx_buf[i + 3] == 0x85) found_0x85 = true;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);
    TEST_ASSERT_FALSE(found_0x85);

    /* 4. Transition back to Radio (CAN 0x165 -> Tuner 0x10 = FM1) */
    can_frame_t frame_radio = {
        .id = 0x165,
        .dlc = 4,
        .data = { 0xC8, 0xC0, 0x10, 0x00 }
    };
    can_router_process_can(&frame_radio);

    /* Allow 800ms physical button debounce to settle */
    hal_delay_ms(850);
    can_router_periodic_100ms();

    rx_len = read_uart_output(rx_buf, sizeof(rx_buf));
    /* Transition to radio MUST output Cmd 0x84 (Radio State) followed by Cmd 0x85 (Preset list) */
    found_0x84 = false;
    found_0x85 = false;
    for (size_t i = 0; i < rx_len; i++) {
        if (rx_buf[i] == 0x5A && (i + 3) < rx_len) {
            if (rx_buf[i + 3] == 0x84) found_0x84 = true;
            if (rx_buf[i + 3] == 0x85) found_0x85 = true;
        }
    }
    TEST_ASSERT_TRUE(found_0x84);
    TEST_ASSERT_TRUE(found_0x85);
}

