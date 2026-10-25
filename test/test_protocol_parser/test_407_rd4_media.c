#include "unity.h"
#include "profiles/peugeot_407.h"
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
 * 1.7 OEM JBL Sound Amplifier Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_jbl_amplifier(void) {
    // Bass=9 (+2), Treble=7 (0), Bal=7, Fader=6, EQ=1 (Pop), Loudness=1 (0x10), SpeedComp=2, Vol=18
    const uint8_t can_1a0[] = { 0x00, 0x09, 0x07, 0x07, 0x06, 0x01, 0x12, 0x12 };
    jbl_amplifier_state_t amp;
    psa_decode_amplifier_0x1a0(can_1a0, sizeof(can_1a0), &amp);

    TEST_ASSERT_EQUAL_UINT8(9, amp.bass);
    TEST_ASSERT_EQUAL_UINT8(7, amp.treble);
    TEST_ASSERT_EQUAL_UINT8(7, amp.balance);
    TEST_ASSERT_EQUAL_UINT8(6, amp.fader);
    TEST_ASSERT_EQUAL_UINT8(1, amp.eq_preset);
    TEST_ASSERT_TRUE(amp.loudness);
    TEST_ASSERT_EQUAL_UINT8(2, amp.speed_vol_comp);
    TEST_ASSERT_EQUAL_UINT8(18, amp.master_volume);

    uint8_t out[16];
    size_t len = build_raise_amplifier(&amp, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(12, len);
}

/* --------------------------------------------------------------------------
 * 1.7b RD4 Radio Volume & Audio Settings (DSP) Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_rd4_volume(void) {
    vehicle_audio_settings_t audio;
    memset(&audio, 0, sizeof(audio));
    audio.master_volume = 15;

    // 1. Invalid DLC (0 bytes) -> ignored
    const uint8_t can_empty[] = { 0x00 };
    psa_decode_rd4_volume_0x1a5(can_empty, 0, &audio);
    TEST_ASSERT_FALSE(audio.valid);

    // 2. Normal Volume Change: Vol=18, Unmuted
    const uint8_t can_vol18[] = { 0x12 }; // 18 = 0x12
    TEST_ASSERT_TRUE(psa_decode_rd4_volume_0x1a5(can_vol18, sizeof(can_vol18), &audio));
    TEST_ASSERT_TRUE(audio.valid);
    TEST_ASSERT_EQUAL_UINT8(18, audio.master_volume);
    TEST_ASSERT_FALSE(audio.is_muted);
    TEST_ASSERT_EQUAL_UINT8(1, audio.active_mode);

    // 3. Mute Toggle: Vol=18, Muted (0x80 | 18 = 0x92)
    const uint8_t can_mute[] = { 0x92 };
    TEST_ASSERT_TRUE(psa_decode_rd4_volume_0x1a5(can_mute, sizeof(can_mute), &audio));
    TEST_ASSERT_EQUAL_UINT8(18, audio.master_volume);
    TEST_ASSERT_TRUE(audio.is_muted);
    TEST_ASSERT_EQUAL_UINT8(1, audio.active_mode);

    // 4. Volume Clamping: 31 clamped to 30
    const uint8_t can_clamp[] = { 0x1F }; // 31
    TEST_ASSERT_TRUE(psa_decode_rd4_volume_0x1a5(can_clamp, sizeof(can_clamp), &audio));
    TEST_ASSERT_EQUAL_UINT8(30, audio.master_volume);
    TEST_ASSERT_FALSE(audio.is_muted);
    TEST_ASSERT_EQUAL_UINT8(1, audio.active_mode);

    // 5. Volume Stopped / Idle Frame (0xEE = 0xE0 | 14 from vehicle log):
    // Bits [7:5] = 0xE0 means stopped, NOT muted! Dismisses volume mode 1 -> 0.
    const uint8_t can_idle[] = { 0xEE }; // Vol 14, stopped
    TEST_ASSERT_TRUE(psa_decode_rd4_volume_0x1a5(can_idle, sizeof(can_idle), &audio));
    TEST_ASSERT_EQUAL_UINT8(14, audio.master_volume);
    TEST_ASSERT_FALSE(audio.is_muted);
    TEST_ASSERT_EQUAL_UINT8(0, audio.active_mode);

    // 6. Repeated Unchanged Idle Frame (0xEE): zero jitter / no UART spam
    TEST_ASSERT_FALSE(psa_decode_rd4_volume_0x1a5(can_idle, sizeof(can_idle), &audio));
    TEST_ASSERT_EQUAL_UINT8(14, audio.master_volume);
    TEST_ASSERT_FALSE(audio.is_muted);
    TEST_ASSERT_EQUAL_UINT8(0, audio.active_mode);

    // 7. Idle Frame with Volume Change (0xF0 = 0xE0 | 16): Vol 16, unmuted, mode 0
    const uint8_t can_idle16[] = { 0xF0 }; // Vol 16, stopped
    TEST_ASSERT_TRUE(psa_decode_rd4_volume_0x1a5(can_idle16, sizeof(can_idle16), &audio));
    TEST_ASSERT_EQUAL_UINT8(16, audio.master_volume);
    TEST_ASSERT_FALSE(audio.is_muted);
    TEST_ASSERT_EQUAL_UINT8(0, audio.active_mode);
}

void test_peugeot_407_rd4_audio_settings(void) {
    vehicle_audio_settings_t audio;
    memset(&audio, 0, sizeof(audio));

    // 1. Invalid DLC (< 7 bytes) -> ignored
    const uint8_t can_short[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_short, 6, &audio);
    TEST_ASSERT_FALSE(audio.valid);

    // 2. All Neutral / Center (bias 63 -> 9): Bal=9, Fad=9, Bass=9, Treb=9, Loud=OFF, SpdVol=OFF, EQ=None, Mode=0
    const uint8_t can_neutral[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x00, 0x00 };
    TEST_ASSERT_TRUE(psa_decode_rd4_audio_settings_0x1e5(can_neutral, sizeof(can_neutral), &audio));
    TEST_ASSERT_TRUE(audio.valid);
    TEST_ASSERT_EQUAL_UINT8(9, audio.balance);
    TEST_ASSERT_EQUAL_UINT8(9, audio.fader);
    TEST_ASSERT_EQUAL_UINT8(9, audio.bass);
    TEST_ASSERT_EQUAL_UINT8(9, audio.treble);
    TEST_ASSERT_FALSE(audio.loudness);
    TEST_ASSERT_FALSE(audio.speed_volume);
    TEST_ASSERT_EQUAL_UINT8(0, audio.eq_preset);
    TEST_ASSERT_EQUAL_UINT8(0, audio.active_mode);

    // 3. Minimums (-9 -> 0): raw 54 (0x36)
    const uint8_t can_min[] = { 0x36, 0x36, 0x36, 0x00, 0x36, 0x00, 0x00 };
    TEST_ASSERT_TRUE(psa_decode_rd4_audio_settings_0x1e5(can_min, sizeof(can_min), &audio));
    TEST_ASSERT_EQUAL_UINT8(0, audio.balance);
    TEST_ASSERT_EQUAL_UINT8(0, audio.fader);
    TEST_ASSERT_EQUAL_UINT8(0, audio.bass);
    TEST_ASSERT_EQUAL_UINT8(0, audio.treble);

    // 4. Maximums (+9 -> 18): raw 72 (0x48)
    const uint8_t can_max[] = { 0x48, 0x48, 0x48, 0x00, 0x48, 0x00, 0x00 };
    TEST_ASSERT_TRUE(psa_decode_rd4_audio_settings_0x1e5(can_max, sizeof(can_max), &audio));
    TEST_ASSERT_EQUAL_UINT8(18, audio.balance);
    TEST_ASSERT_EQUAL_UINT8(18, audio.fader);
    TEST_ASSERT_EQUAL_UINT8(18, audio.bass);
    TEST_ASSERT_EQUAL_UINT8(18, audio.treble);

    // 5. Clamping out-of-bounds (< 54 and > 72)
    const uint8_t can_oob[] = { 0x20, 0x70, 0x3F, 0x00, 0x3F, 0x00, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_oob, sizeof(can_oob), &audio);
    TEST_ASSERT_EQUAL_UINT8(0, audio.balance);  // 32 (< 54) clamped to 0
    TEST_ASSERT_EQUAL_UINT8(18, audio.fader);   // 112 (> 72) clamped to 18

    // 6. Switches: Loudness ON (0x40), Speed Volume ON (0x07)
    const uint8_t can_switches[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x47, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_switches, sizeof(can_switches), &audio);
    TEST_ASSERT_TRUE(audio.loudness);
    TEST_ASSERT_TRUE(audio.speed_volume);

    // 7. EQ Presets
    const uint8_t eq_tests[5][2] = {
        { 0x07, 1 }, // Classical
        { 0x0B, 2 }, // Jazz-Blues
        { 0x0F, 3 }, // Pop-Rock
        { 0x13, 4 }, // Vocals
        { 0x17, 5 }, // Techno
    };
    for (int i = 0; i < 5; i++) {
        uint8_t can_eq[7] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x00, eq_tests[i][0] };
        psa_decode_rd4_audio_settings_0x1e5(can_eq, sizeof(can_eq), &audio);
        TEST_ASSERT_EQUAL_UINT8(eq_tests[i][1], audio.eq_preset);
    }

    // 8. Active Mode Detection:
    // Bass active (Byte 2 Bit 7) -> Mode 2
    uint8_t can_mode2[] = { 0x3F, 0x3F, 0xBF, 0x00, 0x3F, 0x00, 0x00 }; // 0x3F | 0x80 = 0xBF
    TEST_ASSERT_TRUE(psa_decode_rd4_audio_settings_0x1e5(can_mode2, sizeof(can_mode2), &audio));
    TEST_ASSERT_EQUAL_UINT8(2, audio.active_mode);

    // Treble active (Byte 4 Bit 7) -> Mode 3
    uint8_t can_mode3[] = { 0x3F, 0x3F, 0x3F, 0x00, 0xBF, 0x00, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_mode3, sizeof(can_mode3), &audio);
    TEST_ASSERT_EQUAL_UINT8(3, audio.active_mode);

    // Balance active (Byte 0 Bit 7) -> Mode 4
    uint8_t can_mode4[] = { 0xBF, 0x3F, 0x3F, 0x00, 0x3F, 0x00, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_mode4, sizeof(can_mode4), &audio);
    TEST_ASSERT_EQUAL_UINT8(4, audio.active_mode);

    // Fader active (Byte 1 Bit 7) -> Mode 5
    uint8_t can_mode5[] = { 0x3F, 0xBF, 0x3F, 0x00, 0x3F, 0x00, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_mode5, sizeof(can_mode5), &audio);
    TEST_ASSERT_EQUAL_UINT8(5, audio.active_mode);

    // Loudness active (Byte 5 Bit 7) -> Mode 6
    uint8_t can_mode6[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x80, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_mode6, sizeof(can_mode6), &audio);
    TEST_ASSERT_EQUAL_UINT8(6, audio.active_mode);

    // Speed Volume active (Byte 5 Bit 4) -> Mode 7
    uint8_t can_mode7[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x10, 0x00 };
    psa_decode_rd4_audio_settings_0x1e5(can_mode7, sizeof(can_mode7), &audio);
    TEST_ASSERT_EQUAL_UINT8(7, audio.active_mode);

    // Equalizer active (Byte 6 Bit 6) -> Mode 8
    uint8_t can_mode8[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x00, 0x40 };
    psa_decode_rd4_audio_settings_0x1e5(can_mode8, sizeof(can_mode8), &audio);
    TEST_ASSERT_EQUAL_UINT8(8, audio.active_mode);

    // 9. Menu Dismissal: transition from active (Mode 8) to closed (Mode 0)
    const uint8_t can_dismiss[] = { 0x3F, 0x3F, 0x3F, 0x00, 0x3F, 0x00, 0x00 };
    TEST_ASSERT_TRUE(psa_decode_rd4_audio_settings_0x1e5(can_dismiss, sizeof(can_dismiss), &audio));
    TEST_ASSERT_EQUAL_UINT8(0, audio.active_mode);

    // 10. Menu Closed / Inactive Repeated Frame: zero jitter / no update spam
    TEST_ASSERT_FALSE(psa_decode_rd4_audio_settings_0x1e5(can_dismiss, sizeof(can_dismiss), &audio));
    TEST_ASSERT_EQUAL_UINT8(0, audio.active_mode);

    // 11. Active Mode Repeated Unchanged Frame: zero flicker / no redundant update spam while in menu
    TEST_ASSERT_TRUE(psa_decode_rd4_audio_settings_0x1e5(can_mode2, sizeof(can_mode2), &audio)); // Re-enter Bass mode
    TEST_ASSERT_EQUAL_UINT8(2, audio.active_mode);
    TEST_ASSERT_FALSE(psa_decode_rd4_audio_settings_0x1e5(can_mode2, sizeof(can_mode2), &audio)); // Repeated unchanged frame in mode 2
    TEST_ASSERT_EQUAL_UINT8(2, audio.active_mode);
}

void test_peugeot_407_rd4_hiworld_frame_build(void) {
    vehicle_audio_settings_t audio;
    memset(&audio, 0, sizeof(audio));
    audio.master_volume = 18;
    audio.is_muted = false;
    audio.bass = 11;
    audio.treble = 9;
    audio.balance = 9;
    audio.fader = 8;
    audio.eq_preset = 3;
    audio.loudness = true;
    audio.speed_volume = true;
    audio.active_mode = 2; // Bass active

    uint8_t out[16];
    size_t len = build_hiworld_audio_settings(&audio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(13, len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x56, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[4]);  // Unmuted (0x00) | Mode 2 = 0x02
    TEST_ASSERT_EQUAL_UINT8(11, out[5]);  // Bass
    TEST_ASSERT_EQUAL_UINT8(9, out[6]);   // Treble
    TEST_ASSERT_EQUAL_UINT8(9, out[7]);   // Balance
    TEST_ASSERT_EQUAL_UINT8(8, out[8]);   // Fader
    TEST_ASSERT_EQUAL_UINT8(3, out[9]);   // EQ Preset
    TEST_ASSERT_EQUAL_HEX8(0x11, out[10]); // Loudness (0x10) | SpeedVol (0x01)
    TEST_ASSERT_EQUAL_UINT8(18, out[11]); // Master Volume
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[12]); // Checksum

    // Muted test:
    audio.is_muted = true;
    len = build_hiworld_audio_settings(&audio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(13, len);
    TEST_ASSERT_EQUAL_HEX8(0x82, out[4]); // Muted (0x80) | Mode 2 = 0x82
    TEST_ASSERT_EQUAL_HEX8(0x2A, out[12]); // Checksum: 0xAA + 0x80 = 0x12A & 0xFF = 0x2A

    // Buffer too small -> return 0
    TEST_ASSERT_EQUAL_UINT32(0, build_hiworld_audio_settings(&audio, out, 12));
}

/* --------------------------------------------------------------------------
 * 1.8 RD4 Radio & CD Changer Tests
 * -------------------------------------------------------------------------- */
void test_peugeot_407_cd_changer_and_rds(void) {
    // CD Changer (0x3A6): Disc 2, Track 14, Total 20, 03:45, Random
    const uint8_t can_3a6[] = { 0x00, 0x02, 0x0E, 0x14, 0x03, 0x2D, 0x01, 0x00 };
    cd_changer_state_t cdc;
    psa_decode_cd_changer_0x3a6(can_3a6, sizeof(can_3a6), &cdc);

    TEST_ASSERT_EQUAL_UINT8(2, cdc.disc_slot);
    TEST_ASSERT_EQUAL_UINT8(14, cdc.track_num);
    TEST_ASSERT_EQUAL_UINT8(20, cdc.total_tracks);
    TEST_ASSERT_EQUAL_UINT8(3, cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(45, cdc.elapsed_sec);

    uint8_t out_cdc[16];
    size_t len_cdc = build_raise_cd_changer(&cdc, out_cdc, sizeof(out_cdc));
    TEST_ASSERT_EQUAL_UINT32(11, len_cdc);

    // RDS Radio Station (0x396): "BBC R1  "
    const uint8_t can_396[] = { 'B', 'B', 'C', ' ', 'R', '1', ' ', ' ' };
    char name[9];
    psa_decode_rds_name_0x396(can_396, sizeof(can_396), name);
    TEST_ASSERT_EQUAL_STRING("BBC R1  ", name);

    uint8_t out_rds[16];
    size_t len_rds = build_raise_rds_name("BBC R1", out_rds, sizeof(out_rds));
    TEST_ASSERT_EQUAL_UINT32(12, len_rds);
}

void test_peugeot_407_rd4_vector_1_fm_tuner(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. Source: Tuner (0x165)
    const uint8_t can_165[] = { 0xCC, 0x54, 0x10, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_165, sizeof(can_165));
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);

    // 2. Tuner Status (0x225): FM1, Preset 1, RDS Lock, 102.50 MHz (raw 1050 = 0x041A)
    const uint8_t can_225[] = { 0x20, 0x01, 0x90, 0x04, 0x1A };
    psa_rd4_process_can_0x225(&media.radio, can_225, sizeof(can_225));
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.band); /* FM1 */
    TEST_ASSERT_EQUAL_UINT16(1025, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.preset_slot);
    TEST_ASSERT_EQUAL_UINT8(0x20, media.radio.indicators);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.source_mode);

    // 3. Station Name (0x2A5): "RMF FM  "
    const uint8_t can_2a5[] = { 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D, 0x20, 0x20 };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5, sizeof(can_2a5));
    TEST_ASSERT_EQUAL_STRING("RMF FM  ", media.radio.station_name);

    // 4. Hiworld Output (Cmd 0x84): 19 wire bytes
    uint8_t out[32];
    size_t out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);

    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0E, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x84, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* FM1 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[5]); /* 1025 & 0xFF (Little Endian for Hiworld) */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[6]); /* 1025 >> 8 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[7]); /* Preset 1 */
    TEST_ASSERT_EQUAL_HEX8(0x20, out[8]); /* RDS indicator */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[9]); /* Playing */
    TEST_ASSERT_EQUAL_STRING_LEN("RMF FM  ", (char *)&out[10], 8);

    /* Verify checksum matches ((0x0E + 0x84 + sum(payload) - 1) & 0xFF) */
    uint16_t sum = out[2] + out[3];
    for (size_t i = 0; i < 14; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[18]);
}

void test_peugeot_407_rd4_vector_2_isotp_radiotext(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // Dynamic RDS RadioText: "Queen - Bohemian Rhapsody" (25 chars) with prefix 10 00 00 00 (Total 29 chars = 0x01D)
    const uint8_t ff[]  = { 0x10, 0x1D, 0x10, 0x00, 0x00, 0x00, 0x51, 0x75 };
    const uint8_t cf1[] = { 0x21, 0x65, 0x65, 0x6E, 0x20, 0x2D, 0x20, 0x42 };
    const uint8_t cf2[] = { 0x22, 0x6F, 0x68, 0x65, 0x6D, 0x69, 0x61, 0x6E };
    const uint8_t cf3[] = { 0x23, 0x20, 0x52, 0x68, 0x61, 0x70, 0x73, 0x6F };
    const uint8_t cf4[] = { 0x24, 0x64, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00 };

    hal_can_native_clear_sent_frame();
    TEST_ASSERT_FALSE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, ff, sizeof(ff)));
    TEST_ASSERT_TRUE(isotp.active);
    can_frame_t fc_frame;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&fc_frame));
    TEST_ASSERT_EQUAL_HEX32(0x09F, fc_frame.id);
    TEST_ASSERT_EQUAL_UINT8(3, fc_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x30, fc_frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_frame.data[2]);

    TEST_ASSERT_FALSE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf1, sizeof(cf1)));
    TEST_ASSERT_TRUE(isotp.active);
    TEST_ASSERT_FALSE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf2, sizeof(cf2)));
    TEST_ASSERT_TRUE(isotp.active);
    TEST_ASSERT_FALSE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf3, sizeof(cf3)));
    TEST_ASSERT_TRUE(isotp.active);

    TEST_ASSERT_TRUE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf4, sizeof(cf4)));
    TEST_ASSERT_FALSE(isotp.active);
    TEST_ASSERT_EQUAL_UINT8(25, media.radio.radio_text_len);
    TEST_ASSERT_EQUAL_STRING("Queen - Bohemian Rhapsody", media.radio.radio_text);
    TEST_ASSERT_EQUAL_UINT8(0x04, media.radio.indicators & 0x04); /* RDTEXT bit */

    // Hiworld Output (Cmd 0x86): 5 + 25 = 30 wire bytes
    uint8_t out[64];
    size_t out_len = build_hiworld_radio_text(media.radio.radio_text, media.radio.radio_text_len, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(30, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(25, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x86, out[3]);
    TEST_ASSERT_EQUAL_STRING_LEN("Queen - Bohemian Rhapsody", (char *)&out[4], 25);

    uint16_t sum = out[2] + out[3];
    for (size_t i = 0; i < 25; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[29]);

    // Test 68-byte (0x44) First Frame with 4-byte header (non-zero RDS flags) and full 64-character text
    // FF: 10 44 10 01 02 00 'T' 'e' (total_len = 0x44 = 68 bytes)
    const uint8_t ff_68[] = { 0x10, 0x44, 0x10, 0x01, 0x02, 0x00, 'T', 'e' };
    hal_can_native_clear_sent_frame();
    TEST_ASSERT_FALSE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, ff_68, sizeof(ff_68)));
    TEST_ASSERT_TRUE(isotp.active);
    TEST_ASSERT_EQUAL_UINT16(68, isotp.total_length);
    TEST_ASSERT_EQUAL_UINT16(6, isotp.received_length);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&fc_frame));
    TEST_ASSERT_EQUAL_HEX32(0x09F, fc_frame.id);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_frame.data[2]);

    // Feed 9 Consecutive Frames (21..29) of 7 bytes each: 6 + (9 * 7) = 69 >= 68
    uint8_t sn = 1;
    for (int i = 0; i < 8; i++) {
        uint8_t cf[8];
        cf[0] = 0x20 | (sn & 0x0F);
        memset(&cf[1], 'A' + i, 7);
        sn = (sn + 1) & 0x0F;
        TEST_ASSERT_FALSE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf, sizeof(cf)));
        TEST_ASSERT_TRUE(isotp.active);
    }
    // Final CF to complete reassembly (6 + 56 = 62; need 6 more bytes)
    uint8_t cf_last[8];
    cf_last[0] = 0x20 | (sn & 0x0F);
    memset(&cf_last[1], 'Z', 7);
    TEST_ASSERT_TRUE(psa_rd4_process_can_0x0a4(&media.radio, &isotp, cf_last, sizeof(cf_last)));
    TEST_ASSERT_FALSE(isotp.active);
    TEST_ASSERT_EQUAL_UINT8(PSA_RD4_MAX_RADIO_TEXT_LEN, media.radio.radio_text_len);
    TEST_ASSERT_EQUAL_UINT8('T', media.radio.radio_text[0]);
    TEST_ASSERT_EQUAL_UINT8('e', media.radio.radio_text[1]);
    TEST_ASSERT_EQUAL_UINT8('\0', media.radio.radio_text[64]);
}

void test_peugeot_407_rd4_vector_3_cd_changer(void) {
    vehicle_media_t media;
    psa_rd4_media_init(&media, NULL);

    // CAN 0x3A6: Disc 2, Track 14, Total 20, Min 3, Sec 45, Play Flags 0x01 (Random)
    const uint8_t can_3a6[] = { 0x00, 0x02, 0x0E, 0x14, 0x03, 0x2D, 0x01, 0x00 };
    psa_rd4_process_can_0x3a6(&media.cdc, can_3a6, sizeof(can_3a6));

    TEST_ASSERT_EQUAL_UINT8(2, media.cdc.active_disc);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.discs_loaded_mask);
    TEST_ASSERT_EQUAL_UINT16(14, media.cdc.track_num);
    TEST_ASSERT_EQUAL_UINT16(20, media.cdc.total_tracks);
    TEST_ASSERT_EQUAL_UINT8(3, media.cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(45, media.cdc.elapsed_sec);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.cdc.play_modes);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.play_status);

    // Hiworld Output (Cmd 0x97): 16 wire bytes (Little-Endian uint16_le tracks, Status 0x02 = Play)
    uint8_t out[32];
    size_t out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);

    /* 5A A5 0B 97 02 02 00 0E 00 03 2D 01 02 14 00 FA
     * Checksum: (0x0B + 0x97 + 0x02 + 0x02 + 0x00 + 0x0E + 0x00 + 0x03 + 0x2D + 0x01 + 0x02 + 0x14 + 0x00) - 1 = 0xFB - 1 = 0xFA
     */
    const uint8_t expected[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x02, 0x02, 0x00, 0x0E, 0x00, 0x03, 0x2D, 0x01, 0x02, 0x14, 0x00, 0xFA
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, 16);
}

void test_peugeot_407_rd4_cd_in_dash_playback(void) {
    vehicle_media_t media;
    psa_rd4_media_init(&media, NULL);

    /* Single In-Dash CD:
     * In OEM captures (dump_2026-10-10_11-12-22.log, dump_2026-10-10_12-07-47.log, dump_2026-10-10_12-09-48.log):
     * active_disc = 0x00, discs_loaded_mask = 0x00.
     * Track: 5 (0x05 0x00), Elapsed: 02:35 (0x02 0x23), Mode: 0x00, Status: Play (0x02), Total tracks: 18 (0x12 0x00)
     * Expected wire frame: 5A A5 0B 97 00 00 00 05 00 02 23 00 02 12 00 DF
     */

    // 1. Audio Source CD: 0x165 -> 0x2 (Internal CD -> 0x20)
    const uint8_t can_165[] = { 0xC8, 0xC0, 0x20, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_165, sizeof(can_165));
    TEST_ASSERT_EQUAL_UINT8(0x20, media.radio.source_mode);
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);

    // 2. CD Disc Info: 0x365 -> Total 18 tracks (0x12)
    const uint8_t can_365[] = { 0x12, 0x26, 0x02, 0x00, 0x00 };
    psa_rd4_process_can_0x365(&media.cdc, can_365, sizeof(can_365));
    TEST_ASSERT_EQUAL_UINT16(18, media.cdc.total_tracks);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.disc_format);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.active_disc);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.discs_loaded_mask);

    // 3. CD Play Telemetry: 0x3A5 -> Track 5 in data[0], 0x19 (Playing), 02 min 35 sec
    const uint8_t can_3a5[] = { 0x05, 0x03, 0x19, 0x02, 0x23, 0x00 };
    psa_rd4_process_can_0x3a5(&media.cdc, can_3a5, sizeof(can_3a5));
    TEST_ASSERT_EQUAL_UINT16(5, media.cdc.track_num);
    TEST_ASSERT_EQUAL_UINT8(2, media.cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(35, media.cdc.elapsed_sec);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.play_status);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.play_modes);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.active_disc);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.discs_loaded_mask);

    // Build Hiworld Frame Cmd 0x97
    uint8_t out[32];
    size_t out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);

    const uint8_t expected_spec_in_dash[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x05, 0x00, 0x02, 0x23, 0x00, 0x02, 0x12, 0x00, 0xDF
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_spec_in_dash, out, 16);
}

void test_peugeot_407_rd4_cd_tray_mechanism(void) {
    vehicle_media_t media;
    psa_rd4_media_init(&media, NULL);

    // 1. CD Inserted: CAN 0x325 -> data[1] = 0x03
    const uint8_t can_insert[] = { 0x80, 0x03, 0x00 };
    psa_rd4_process_can_0x325(&media.cdc, can_insert, sizeof(can_insert));
    // For single in-dash CD, active_disc and magazine mask stay 0x00
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.discs_loaded_mask);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.active_disc);

    // 2. CD Ejected: CAN 0x325 -> data[1] = 0x00
    const uint8_t can_eject[] = { 0x80, 0x00, 0x00 };
    psa_rd4_process_can_0x325(&media.cdc, can_eject, sizeof(can_eject));
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.discs_loaded_mask);
    TEST_ASSERT_EQUAL_UINT8(0x0C, media.cdc.play_status); /* Eject status code */

    // Check Hiworld wire output
    uint8_t out[32];
    size_t out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);

    /* Disc=0, Loaded=0, Format=0, Track=0, Min=0, Sec=0, Mode=0, Status=0x0C, Total=0
     * Checksum: (0x0B + 0x97 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 + 0x0C + 0x00 + 0x00) - 1 = 0xAE - 1 = 0xAD
     */
    const uint8_t expected_eject[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0xAD
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_eject, out, 16);
}

void test_peugeot_407_rd4_cd_ground_truth_dump(void) {
    vehicle_media_t media;
    psa_rd4_media_init(&media, NULL);

    /* Section 7 Example 6: Ground truth from dump_2026-10-10_11-12-22.log
     * CAN 0x165 (dlc 4): C8 C0 20 00 -> Source 0x2 (Internal CD)
     * CAN 0x365 (dlc 5): 0C 26 02 00 00 -> Total Tracks = 12 (0x0C)
     * CAN 0x3A5 (dlc 6): 03 03 19 03 0C 00 -> Track 3, Playing, 03 min 12 sec
     * Single drive (disc_slot = 0, loaded_mask = 0)
     * Wire output: 5A A5 0B 97 00 00 00 03 00 03 0C 00 02 0C 00 C1
     */
    const uint8_t can_165[] = { 0xC8, 0xC0, 0x20, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_165, sizeof(can_165));
    TEST_ASSERT_EQUAL_UINT8(0x20, media.radio.source_mode);

    const uint8_t can_365[] = { 0x0C, 0x26, 0x02, 0x00, 0x00 };
    psa_rd4_process_can_0x365(&media.cdc, can_365, sizeof(can_365));
    TEST_ASSERT_EQUAL_UINT16(12, media.cdc.total_tracks);

    const uint8_t can_3a5[] = { 0x03, 0x03, 0x19, 0x03, 0x0C, 0x00 };
    psa_rd4_process_can_0x3a5(&media.cdc, can_3a5, sizeof(can_3a5));
    TEST_ASSERT_EQUAL_UINT16(3, media.cdc.track_num);
    TEST_ASSERT_EQUAL_UINT8(3, media.cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(12, media.cdc.elapsed_sec);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.play_status);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.play_modes);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.active_disc);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.discs_loaded_mask);

    uint8_t out[32];
    size_t out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);

    const uint8_t expected_dump[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x0C, 0x00, 0x02, 0x0C, 0x00, 0xC1
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump, out, 16);

    /* Verification for dump_2026-10-10_12-07-47.log:
     * CAN 0x3A5: 05 03 1E 00 07 00 -> Track 5, raw status 0x1E, 00 min 07 sec
     * OEM CAN box: 5A A5 0B 97 00 00 00 05 00 00 07 00 02 0C 00 BB
     */
    const uint8_t can_3a5_log1[] = { 0x05, 0x03, 0x1E, 0x00, 0x07, 0x00 };
    psa_rd4_process_can_0x3a5(&media.cdc, can_3a5_log1, sizeof(can_3a5_log1));
    TEST_ASSERT_EQUAL_UINT16(5, media.cdc.track_num);
    TEST_ASSERT_EQUAL_UINT8(0, media.cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(7, media.cdc.elapsed_sec);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.play_status);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.play_modes);

    out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);
    const uint8_t expected_dump_log1[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x07, 0x00, 0x02, 0x0C, 0x00, 0xBB
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_log1, out, 16);

    /* Verification for dump_2026-10-10_12-09-48.log:
     * CAN 0x3A5: 03 03 19 00 04 00 -> Track 3, raw status 0x19, 00 min 04 sec
     * OEM CAN box: 5A A5 0B 97 00 00 00 03 00 00 04 00 02 0C 00 B6
     */
    const uint8_t can_3a5_log2[] = { 0x03, 0x03, 0x19, 0x00, 0x04, 0x00 };
    psa_rd4_process_can_0x3a5(&media.cdc, can_3a5_log2, sizeof(can_3a5_log2));
    TEST_ASSERT_EQUAL_UINT16(3, media.cdc.track_num);
    TEST_ASSERT_EQUAL_UINT8(0, media.cdc.elapsed_min);
    TEST_ASSERT_EQUAL_UINT8(4, media.cdc.elapsed_sec);
    TEST_ASSERT_EQUAL_UINT8(0x02, media.cdc.play_status);
    TEST_ASSERT_EQUAL_UINT8(0x00, media.cdc.play_modes);

    out_len = build_hiworld_media_state(&media.cdc, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(16, out_len);
    const uint8_t expected_dump_log2[] = {
        0x5A, 0xA5, 0x0B, 0x97, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x02, 0x0C, 0x00, 0xB6
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_dump_log2, out, 16);

    /* Verification for dump_2026-10-11_19-22-47.log:
     * When radio is active, car broadcasts uninitialized sentinels:
     * CAN 0x365: FF FF FF 00 00
     * CAN 0x3A5: FF FF FF FF 7F 00
     * Decoders MUST ignore these frames and NOT mark cdc as updated or playing!
     */
    const uint8_t can_365_sentinel[] = { 0xFF, 0xFF, 0xFF, 0x00, 0x00 };
    TEST_ASSERT_FALSE(psa_rd4_process_can_0x365(&media.cdc, can_365_sentinel, sizeof(can_365_sentinel)));

    const uint8_t can_3a5_sentinel[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00 };
    TEST_ASSERT_FALSE(psa_rd4_process_can_0x3a5(&media.cdc, can_3a5_sentinel, sizeof(can_3a5_sentinel)));
}

void test_peugeot_407_rd4_source_and_wavebands(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. CD Internal Drive Source
    const uint8_t can_cd[] = { 0xCC, 0x54, 0x20, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_cd, sizeof(can_cd));
    TEST_ASSERT_EQUAL_UINT8(0x20, media.radio.source_mode);

    // 2. AUX 1 Source
    const uint8_t can_aux1[] = { 0xCC, 0x54, 0x40, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_aux1, sizeof(can_aux1));
    TEST_ASSERT_EQUAL_UINT8(0x40, media.radio.source_mode);

    // 3. AUX 2 Source
    const uint8_t can_aux2[] = { 0xCC, 0x54, 0x50, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_aux2, sizeof(can_aux2));
    TEST_ASSERT_EQUAL_UINT8(0x41, media.radio.source_mode);

    // 4. CDC Source
    const uint8_t can_cdc[] = { 0xCC, 0x54, 0x30, 0x02 };
    psa_rd4_process_can_0x165(&media.radio, can_cdc, sizeof(can_cdc));
    TEST_ASSERT_EQUAL_UINT8(0x31, media.radio.source_mode);

    // 5. AM Tuner: 1440 kHz, Band 0x50
    const uint8_t can_am[] = { 0x00, 0x02, 0x50, 0x05, 0xA0 };
    psa_rd4_process_can_0x225(&media.radio, can_am, sizeof(can_am));
    TEST_ASSERT_EQUAL_UINT8(0x10, media.radio.band);
    TEST_ASSERT_EQUAL_UINT16(1440, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_UINT8(2, media.radio.preset_slot);

    // 6. Seeking flag on 0x225 (Byte 0 Bit 3 = 0x08)
    const uint8_t can_seek[] = { 0x08, 0x00, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_seek, sizeof(can_seek));
    TEST_ASSERT_EQUAL_UINT8(0x02, media.radio.power_status);

    // Restoration from seeking
    const uint8_t can_locked[] = { 0x00, 0x00, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_locked, sizeof(can_locked));
    TEST_ASSERT_EQUAL_UINT8(0x01, media.radio.power_status);

    // 7. Single frame RadioText on 0x0A4
    const uint8_t can_sf[] = { 0x07, 'T', 'E', 'S', 'T', ' ', 'O', 'K' };
    psa_rd4_process_can_0x0a4(&media.radio, &isotp, can_sf, sizeof(can_sf));
    TEST_ASSERT_EQUAL_UINT8(7, media.radio.radio_text_len);
    TEST_ASSERT_EQUAL_STRING("TEST OK", media.radio.radio_text);
}

void test_peugeot_407_rd4_frequency_and_ta_stability(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. Source Tuner (0x165)
    const uint8_t can_165[] = { 0xC8, 0xC0, 0x10, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_165, sizeof(can_165));

    // 2. Station Name (0x2A5): " RMF FM "
    const uint8_t can_2a5[] = { 0x20, 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D, 0x20 };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5, sizeof(can_2a5));

    // 3. Tuner Frequency (0x225): 96.0 MHz (raw 0x0398 = 920 -> 960 in 0.1 MHz)
    // CAN payload from real dump: 20 10 10 03 98
    const uint8_t can_225[] = { 0x20, 0x10, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_225, sizeof(can_225));
    TEST_ASSERT_EQUAL_UINT16(960, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.preset_slot);

    // 4. TA set by 0x265 (Byte 0 bit 4 = 0x10, bit 5 = 0x20 Stereo): payload b4 10 00 01
    const uint8_t can_265[] = { 0xB4, 0x10, 0x00, 0x01 };
    psa_rd4_process_can_0x265(&media.radio, can_265, sizeof(can_265));
    TEST_ASSERT_EQUAL_HEX8(0xE0, media.radio.indicators & 0xE0); /* TA (0x80), Stereo (0x40), RDS (0x20) */

    // 5. Subsequent 0x225 arrival: must NOT clear TA or Stereo set by 0x265!
    psa_rd4_process_can_0x225(&media.radio, can_225, sizeof(can_225));
    TEST_ASSERT_EQUAL_HEX8(0xE0, media.radio.indicators & 0xE0); /* TA & ST remain set (no flickering!) */

    // 5b. TA toggle OFF by 0x265 (A4 10 00 01): clears TA (0x80) while preserving Stereo (0x40)
    const uint8_t can_265_off[] = { 0xA4, 0x10, 0x00, 0x01 };
    psa_rd4_process_can_0x265(&media.radio, can_265_off, sizeof(can_265_off));
    TEST_ASSERT_EQUAL_HEX8(0x00, media.radio.indicators & 0x80); /* TA cleared */
    TEST_ASSERT_EQUAL_HEX8(0x60, media.radio.indicators & 0x60); /* Stereo (0x40) and RDS (0x20) active */

    // Turn TA back on for subsequent checks
    psa_rd4_process_can_0x265(&media.radio, can_265, sizeof(can_265));

    // 6. Hiworld Serialization (Cmd 0x84): verify Little-Endian frequency bytes & FM1 band
    uint8_t out[32];
    size_t out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* FM1 band */
    // 960 (0x03C0) in Little Endian: out[5] = 0xC0, out[6] = 0x03
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0xE0, out[8]); /* Indicators: TA (0x80) + ST (0x40) + RDS (0x20) */

    // 7. Test FM2 frame from real dump (dump_2026-10-09_21-41-25.log):
    // CAN 0x225: 20 50 20 02 ee (Preset 5, Band 0x20 = FM2, Freq 750 = 87.5 MHz)
    const uint8_t can_225_fm2[] = { 0x20, 0x50, 0x20, 0x02, 0xEE };
    psa_rd4_process_can_0x225(&media.radio, can_225_fm2, sizeof(can_225_fm2));
    TEST_ASSERT_EQUAL_UINT8(0x02, media.radio.band); /* FM2 -> 0x02 */
    TEST_ASSERT_EQUAL_UINT8(5, media.radio.preset_slot);
    TEST_ASSERT_EQUAL_UINT16(875, media.radio.freq_0_1mhz); /* 750 / 2 + 500 = 875 (87.5 MHz) */

    out_len = build_hiworld_radio_state(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(19, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[4]); /* FM2 band for Hiworld */

    // 8. Station name & RDS cleared on frequency change or empty 0x2A5
    // Set a station name first:
    const uint8_t can_2a5_test[] = { 'T', 'E', 'S', 'T', 'F', 'M', ' ', ' ' };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5_test, sizeof(can_2a5_test));
    TEST_ASSERT_EQUAL_STRING("TESTFM  ", media.radio.station_name);

    // Now arrive with 0x2A5 carrying all 0x00 (no signal):
    const uint8_t can_2a5_empty[8] = { 0 };
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5_empty, sizeof(can_2a5_empty));
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.station_name);

    // Set station name again, then change frequency without RDS:
    psa_rd4_process_can_0x2a5(&media.radio, can_2a5_test, sizeof(can_2a5_test));
    TEST_ASSERT_EQUAL_STRING("TESTFM  ", media.radio.station_name);

    // Change frequency to 96.45 MHz (raw 0x03A1) without RDS (Byte 0 = 0x00):
    const uint8_t can_225_new_freq[] = { 0x00, 0x00, 0x10, 0x03, 0xA1 };
    psa_rd4_process_can_0x225(&media.radio, can_225_new_freq, sizeof(can_225_new_freq));
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.station_name);
    TEST_ASSERT_EQUAL_UINT16(964, media.radio.freq_0_1mhz);
    TEST_ASSERT_EQUAL_HEX8(0x00, media.radio.indicators & 0x20); /* RDS cleared */
}

void test_peugeot_407_rd4_preset_memory_list(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    // 1. Simulate tuning into 6 presets on FM1 (band 0x01)
    // Preset 1: 87.5 MHz (raw 750 = 0x02EE)
    const uint8_t can_p1[] = { 0x20, 0x10, 0x10, 0x02, 0xEE };
    psa_rd4_process_can_0x225(&media.radio, can_p1, sizeof(can_p1));
    // Preset 2: 96.0 MHz (raw 920 = 0x0398)
    const uint8_t can_p2[] = { 0x20, 0x20, 0x10, 0x03, 0x98 };
    psa_rd4_process_can_0x225(&media.radio, can_p2, sizeof(can_p2));
    // Preset 3: 102.5 MHz (raw 1050 = 0x041A)
    const uint8_t can_p3[] = { 0x20, 0x30, 0x10, 0x04, 0x1A };
    psa_rd4_process_can_0x225(&media.radio, can_p3, sizeof(can_p3));

    TEST_ASSERT_EQUAL_UINT16(875, media.radio.preset_freqs[0]);
    TEST_ASSERT_EQUAL_UINT16(960, media.radio.preset_freqs[1]);
    TEST_ASSERT_EQUAL_UINT16(1025, media.radio.preset_freqs[2]);

    // 2. Build Standard Frequency Mode (Cmd 0x85, 13 payload bytes -> 18 wire bytes)
    uint8_t out[64];
    size_t out_len = build_hiworld_radio_preset_freqs(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(18, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0D, out[2]); /* Length 13 */
    TEST_ASSERT_EQUAL_HEX8(0x85, out[3]); /* Opcode 0x85 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* Band FM1 */

    // Check Preset 1 (875 -> 0x036B uint16_be)
    TEST_ASSERT_EQUAL_HEX8(0x03, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x6B, out[6]);
    // Check Preset 2 (960 -> 0x03C0 uint16_be)
    TEST_ASSERT_EQUAL_HEX8(0x03, out[7]);
    TEST_ASSERT_EQUAL_HEX8(0xC0, out[8]);
    // Check Preset 3 (1025 -> 0x0401 uint16_be)
    TEST_ASSERT_EQUAL_HEX8(0x04, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[10]);

    // Checksum check
    uint16_t sum = out[2] + out[3];
    for (size_t i = 0; i < 13; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[17]);

    // 3. Extended Station Names Mode via native CAN 0x125 ISO-TP stream
    // Proven multi-frame sequence captured from vehicle dump_2026-10-10_00-05-44.log:
    const uint8_t can_125_ff[]  = { 0x10, 0x28, 0x10, 0x04, 0x00, 0x40, 0x4A, 0x45 };
    const uint8_t can_125_cf1[] = { 0x21, 0x44, 0x59, 0x4E, 0x4B, 0x41, 0x20, 0xB0 };
    const uint8_t can_125_cf2[] = { 0x22, 0x20, 0x52, 0x4D, 0x46, 0x20, 0x46, 0x4D };
    const uint8_t can_125_cf3[] = { 0x23, 0x20, 0xB0, 0x52, 0x4D, 0x46, 0x20, 0x4D };
    const uint8_t can_125_cf4[] = { 0x24, 0x41, 0x58, 0x58, 0xB0, 0x20, 0x54, 0x52 };
    const uint8_t can_125_cf5[] = { 0x25, 0x4F, 0x4A, 0x4B, 0x41, 0x20, 0xB0 };

    media.radio.band = 0x01; /* Initial band: FM1 */
    hal_can_native_clear_sent_frame();
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_ff, sizeof(can_125_ff));
    TEST_ASSERT_TRUE(isotp.active);
    can_frame_t fc_125;
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&fc_125));
    TEST_ASSERT_EQUAL_HEX32(0x11F, fc_125.id);
    TEST_ASSERT_EQUAL_UINT8(3, fc_125.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x30, fc_125.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_125.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_125.data[2]);

    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf1, sizeof(can_125_cf1));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf2, sizeof(can_125_cf2));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf3, sizeof(can_125_cf3));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf4, sizeof(can_125_cf4));
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_cf5, sizeof(can_125_cf5));

    TEST_ASSERT_EQUAL_STRING("JEDYNKA ", media.radio.preset_names[0]);
    TEST_ASSERT_EQUAL_STRING(" RMF FM ", media.radio.preset_names[1]);
    TEST_ASSERT_EQUAL_STRING("RMF MAXX", media.radio.preset_names[2]);
    TEST_ASSERT_EQUAL_STRING(" TROJKA ", media.radio.preset_names[3]);
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.preset_names[4]);
    TEST_ASSERT_EQUAL_STRING("        ", media.radio.preset_names[5]);
    TEST_ASSERT_EQUAL_HEX8(0x01, media.radio.band); /* Band must not be overwritten by 0x125 */

    // dump_mdf.log vector at 1792763612.148: ID:125 DLC:8 DATA:103a406000002052 (First Frame)
    const uint8_t can_125_dump_mdf_ff[] = { 0x10, 0x3A, 0x40, 0x60, 0x00, 0x00, 0x20, 0x52 };
    hal_can_native_clear_sent_frame();
    psa_rd4_process_can_0x125(&media.radio, &isotp, can_125_dump_mdf_ff, sizeof(can_125_dump_mdf_ff));
    TEST_ASSERT_TRUE(isotp.active);
    TEST_ASSERT_TRUE(hal_can_native_get_last_sent_frame(&fc_125));
    TEST_ASSERT_EQUAL_HEX32(0x11F, fc_125.id);
    TEST_ASSERT_EQUAL_UINT8(3, fc_125.dlc);
    TEST_ASSERT_EQUAL_HEX8(0x30, fc_125.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_125.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, fc_125.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x01, media.radio.band); /* 0x40 in payload must not corrupt band to FM-AST (0x04) */

    // Build Extended Station Names Mode (Cmd 0x85, 49 payload bytes -> 54 wire bytes)
    out_len = build_hiworld_radio_preset_names(&media.radio, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(54, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x5A, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x31, out[2]); /* Length 49 */
    TEST_ASSERT_EQUAL_HEX8(0x85, out[3]); /* Opcode 0x85 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[4]); /* Band FM1 */

    TEST_ASSERT_EQUAL_STRING_LEN("JEDYNKA ", (char *)&out[5], 8);
    TEST_ASSERT_EQUAL_STRING_LEN(" RMF FM ", (char *)&out[13], 8);
    TEST_ASSERT_EQUAL_STRING_LEN("RMF MAXX", (char *)&out[21], 8);
    TEST_ASSERT_EQUAL_STRING_LEN(" TROJKA ", (char *)&out[29], 8);
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&out[37], 8);
    TEST_ASSERT_EQUAL_STRING_LEN("        ", (char *)&out[45], 8);

    sum = out[2] + out[3];
    for (size_t i = 0; i < 49; i++) {
        sum += out[4 + i];
    }
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((sum - 1) & 0xFF), out[53]);
}

void test_peugeot_407_rd4_source_transition_mute(void) {
    vehicle_media_t media;
    psa_isotp_rx_ctx_t isotp;
    psa_rd4_media_init(&media, &isotp);

    /* 1. CD active (C8 C0 20 00) */
    const uint8_t can_cd_active[] = { 0xC8, 0xC0, 0x20, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_cd_active, sizeof(can_cd_active));
    TEST_ASSERT_EQUAL_HEX8(0x20, media.radio.source_mode);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.power_status);
    TEST_ASSERT_FALSE(media.radio.sleep);

    /* 2. Source transition mute initiates (48 C0 00 00) */
    const uint8_t can_mute_trans[] = { 0x48, 0xC0, 0x00, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_mute_trans, sizeof(can_mute_trans));
    TEST_ASSERT_EQUAL_HEX8(0x20, media.radio.source_mode); /* Retained */
    TEST_ASSERT_EQUAL_UINT8(3, media.radio.power_status);  /* Muted */
    TEST_ASSERT_FALSE(media.radio.sleep);

    /* 3. Waiting handshake / deck spinup (48 D0 00 00) */
    const uint8_t can_mute_handshake[] = { 0x48, 0xD0, 0x00, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_mute_handshake, sizeof(can_mute_handshake));
    TEST_ASSERT_EQUAL_HEX8(0x20, media.radio.source_mode); /* Retained */
    TEST_ASSERT_EQUAL_UINT8(3, media.radio.power_status);  /* Muted */
    TEST_ASSERT_FALSE(media.radio.sleep);

    /* 4. Transition completes into Tuner (C8 C0 10 00) */
    const uint8_t can_tuner_active[] = { 0xC8, 0xC0, 0x10, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_tuner_active, sizeof(can_tuner_active));
    TEST_ASSERT_EQUAL_HEX8(media.radio.band, media.radio.source_mode);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.power_status);
    TEST_ASSERT_FALSE(media.radio.sleep);

    /* 5. Radio turns off (48 C8 00 00) */
    const uint8_t can_turn_off[] = { 0x48, 0xC8, 0x00, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_turn_off, sizeof(can_turn_off));
    TEST_ASSERT_EQUAL_HEX8(0xFF, media.radio.source_mode);
    TEST_ASSERT_EQUAL_UINT8(0, media.radio.power_status);

    /* 6. Radio enters deep sleep (08 00 00 00) */
    const uint8_t can_sleep[] = { 0x08, 0x00, 0x00, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_sleep, sizeof(can_sleep));
    TEST_ASSERT_TRUE(media.radio.sleep);
    TEST_ASSERT_EQUAL_HEX8(0xFF, media.radio.source_mode);
    TEST_ASSERT_EQUAL_UINT8(0, media.radio.power_status);

    /* 7. RD4 transient reboot frame test:
     * Radio wakes up into CD, then receives 0x00 0x00 0x00 0x00 reset frame:
     * State must NOT drop to sleep or unpowered during 0x00 frames */
    psa_rd4_process_can_0x165(&media.radio, can_cd_active, sizeof(can_cd_active));
    TEST_ASSERT_EQUAL_HEX8(0x20, media.radio.source_mode);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.power_status);

    const uint8_t can_watchdog_reset[] = { 0x00, 0x00, 0x00, 0x00 };
    for (int i = 0; i < 5; i++) {
        psa_rd4_process_can_0x165(&media.radio, can_watchdog_reset, sizeof(can_watchdog_reset));
        TEST_ASSERT_EQUAL_HEX8(0x20, media.radio.source_mode);
        TEST_ASSERT_EQUAL_UINT8(1, media.radio.power_status); /* Preserved during 0x00 reboot */
    }

    /* 8. Cold wake from standby with source nibble 0 (C8 80 00 00):
     * Must immediately set power_status = 1 and psa_rd4_is_powered() = true */
    psa_rd4_process_can_0x165(&media.radio, can_sleep, sizeof(can_sleep));
    TEST_ASSERT_TRUE(media.radio.sleep);
    TEST_ASSERT_EQUAL_UINT8(0, media.radio.power_status);

    const uint8_t can_wake_initializing[] = { 0xC8, 0x80, 0x00, 0x00 };
    psa_rd4_process_can_0x165(&media.radio, can_wake_initializing, sizeof(can_wake_initializing));
    TEST_ASSERT_FALSE(media.radio.sleep);
    TEST_ASSERT_TRUE(media.radio.present);
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.power_status);
    TEST_ASSERT_TRUE(psa_rd4_is_powered(&media.radio));

    /* Source initializes to Tuner shortly after */
    psa_rd4_process_can_0x165(&media.radio, can_tuner_active, sizeof(can_tuner_active));
    TEST_ASSERT_EQUAL_UINT8(1, media.radio.power_status);
    TEST_ASSERT_EQUAL_HEX8(media.radio.band, media.radio.source_mode);
}


