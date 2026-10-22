#include "profiles/peugeot_407.h"
#include "hal/hal_can.h"
#include <string.h>

static uint8_t s_cached_0x361_raw[6] = { 0x01, 0x01, 0x91, 0x40, 0x30, 0x10 };
static bool    s_cached_0x361_valid  = false;

/* Live cache of active Profile 1 status frame (0x260 / 0x15B) */
static uint8_t s_cached_profile1_raw[8] = { 0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10 };
static bool    s_cached_profile1_valid  = false;

/* Feature enables (Cmd 0x71 and Cmd 0x72) telecoded in BSI */
static uint8_t s_feature_enable_1[2] = { 0x60, 0x12 };
static uint8_t s_feature_enable_2[8] = { 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* Convert Byte 2 delay code to seconds (supports both B2[1:0] and EMF-C B2[3:2] e.g. 0xB4) */
static inline uint8_t delay_code_to_sec(uint8_t byte2) {
    uint8_t code = byte2 & 0x03;
    if (code == 0) {
        code = (byte2 >> 2) & 0x03;
    }
    switch (code) {
        case 1:  return 15;
        case 2:  return 30;
        case 3:  return 60;
        default: return 15; /* Any active Follow-Me-Home (B2.5 = 1) maps to at least 15s */
    }
}

void psa_car_config_init(void) {
    s_cached_0x361_raw[0] = 0x01;
    s_cached_0x361_raw[1] = 0x01;
    s_cached_0x361_raw[2] = 0x91;
    s_cached_0x361_raw[3] = 0x40;
    s_cached_0x361_raw[4] = 0x30;
    s_cached_0x361_raw[5] = 0x10;
    s_cached_0x361_valid  = false;

    s_cached_profile1_raw[0] = 0x01;
    s_cached_profile1_raw[1] = 0x03;
    s_cached_profile1_raw[2] = 0xB1;
    s_cached_profile1_raw[3] = 0x40;
    s_cached_profile1_raw[4] = 0x00;
    s_cached_profile1_raw[5] = 0xD0;
    s_cached_profile1_raw[6] = 0x00;
    s_cached_profile1_raw[7] = 0x10;
    s_cached_profile1_valid  = false;

    s_feature_enable_1[0] = 0x60;
    s_feature_enable_1[1] = 0x12;

    memset(s_feature_enable_2, 0, sizeof(s_feature_enable_2));
    s_feature_enable_2[0] = 0x00;
    s_feature_enable_2[1] = 0x01;
    s_feature_enable_2[2] = 0x80;
}

void psa_decode_config_0x361(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 6) return;

    memcpy(s_cached_0x361_raw, data, 6);
    s_cached_0x361_valid = true;

    /*
     * 0x361 is Menu Availability (Equipment Mask), NOT Current Settings.
     * Byte 0: profile_number
     * Byte 2: auto_lighting (bit 0), follow_me_home (bit 4), hinge_panel (bit 7)
     * Byte 3: drl_present (bit 6)
     * Byte 4: emf_present (bit 4), aas_disable (bit 5)
     * Byte 5: tpms_present (bits 4..6)
     */

    /* Cmd 0x71: CentralStateEnable1 (2 bytes) */
    uint8_t feat1_b0 = 0x80; /* Ambient mood lighting menu enabled */
    if (data[2] & 0x10) {
        feat1_b0 |= 0x03; /* Follow-Me-Home delay supported */
    }
    if (data[3] & 0x40) {
        feat1_b0 |= 0x04; /* Daytime running lights (DRL) supported */
    }
    if (data[4] & 0x20) {
        feat1_b0 |= 0x20; /* Parking Radar (AAS) toggle supported */
    }
    s_feature_enable_1[0] = feat1_b0;
    s_feature_enable_1[1] = 0x80 | 0x20 | 0x08; /* Rear Wiper, Auto Door Lock, Selective Boot */

    /* Cmd 0x72: CentralStateEnable2 (8 bytes) */
    memset(s_feature_enable_2, 0, sizeof(s_feature_enable_2));
    uint8_t feat2_b0 = 0x01; /* Drive Mode Profile supported */
    if (data[2] & 0x80) {
        feat2_b0 |= 0x80; /* Hinge panel / remote trunk */
    }
    if (data[2] & 0x01) {
        feat2_b0 |= 0x40; /* Directional / adaptive lighting */
    }
    if (data[5] & 0x70) {
        feat2_b0 |= 0x02; /* TPMS calibration reset */
    }
    s_feature_enable_2[0] = feat2_b0;
    s_feature_enable_2[1] = 0x04; /* Cluster dimmer */
    s_feature_enable_2[5] = 0x02; /* Zone temp sync */
}

void psa_car_config_get_feature_enables(uint8_t feat1[2], uint8_t feat2[8]) {
    if (feat1) {
        memcpy(feat1, s_feature_enable_1, 2);
    }
    if (feat2) {
        memcpy(feat2, s_feature_enable_2, 8);
    }
}

void psa_decode_config_0x260(vehicle_config_t *cfg, const uint8_t *data, uint8_t dlc) {
    if (!cfg || !data || dlc < 8) return;

    /* Profile 1 is the active user personalization profile on Peugeot 407 */
    if (data[0] != 0x01) {
        return;
    }

    memcpy(s_cached_profile1_raw, data, 8);
    s_cached_profile1_valid = true;

    cfg->active_profile_id      = 1;
    cfg->directional_headlights = (data[2] & 0x80) != 0;
    cfg->follow_me_home         = (data[2] & 0x20) != 0;
    cfg->follow_me_home_sec     = cfg->follow_me_home ? delay_code_to_sec(data[2]) : 0;
    cfg->auto_lighting_enabled  = (data[2] & 0x10) != 0;
    cfg->drl_enabled            = (data[3] & 0x40) != 0;
    cfg->auto_door_lock         = (data[4] & 0x08) != 0;
    cfg->selective_boot_unlock  = (data[4] & 0x04) != 0;
    cfg->motorized_tailgate     = (data[5] & 0x80) != 0;
    cfg->parking_radar_enabled  = (data[5] & 0x40) != 0;
    cfg->rear_wiper_reverse     = (data[5] & 0x10) != 0;
}

hal_status_t psa_car_config_apply_downlink(uint8_t cmd_id, uint8_t setting_id, uint8_t val) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    if (cmd_id == 0x7B) {
        frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B; /* 0x15B */
        frame.dlc = 8;
        memcpy(frame.data, s_cached_profile1_raw, 8);
        frame.data[0] = 0x01; /* Profile 1, Bit 2 cleared = pending configuration write request */

        switch (setting_id) {
            case 0x01: /* Rear Wiper in Reverse */
                if (val) frame.data[5] |= 0x10;
                else     frame.data[5] &= ~0x10;
                break;
            case 0x04: /* Selective Door Unlocking */
                if (val) frame.data[4] |= 0x04;
                else     frame.data[4] &= ~0x04;
                break;
            case 0x05: /* Daytime Running Lights */
                if (val) frame.data[3] |= 0x40;
                else     frame.data[3] &= ~0x40;
                break;
            case 0x06: /* Follow-Me-Home Headlamp Delay */
                if (val == 0) {
                    frame.data[2] &= ~0x20;
                    frame.data[2] &= ~0x03;
                } else {
                    frame.data[2] |= 0x20;
                    frame.data[2] = (uint8_t)((frame.data[2] & ~0x03) | (val & 0x03));
                }
                break;
            case 0x0A: /* Ambient Lighting Brightness */
                if (val & 0x80) {
                    frame.data[2] = (uint8_t)((frame.data[2] & 0xF0) | (val & 0x0F));
                } else {
                    frame.data[2] &= 0xF0;
                }
                break;
            case 0x0B: /* Parking Radar (AAS) Master Toggle */
                if (val) frame.data[5] |= 0x40;
                else     frame.data[5] &= ~0x40;
                break;
            case 0x0C: /* Auto Door Lock while Driving */
                if (val) frame.data[4] |= 0x08;
                else     frame.data[4] &= ~0x08;
                break;
            default:
                return HAL_STATUS_ERROR;
        }

        /* Update local cache optimistically and inject to BSI */
        memcpy(s_cached_profile1_raw, frame.data, 8);
        s_cached_profile1_valid = true;
        return hal_can_send(&frame);
    }
    else if (cmd_id == 0x7D) {
        if (setting_id == 0x01) { /* Remote Boot Mode */
            frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B;
            frame.dlc = 8;
            memcpy(frame.data, s_cached_profile1_raw, 8);
            frame.data[0] = 0x01;
            if (val) frame.data[5] |= 0x80;
            else     frame.data[5] &= ~0x80;
            memcpy(s_cached_profile1_raw, frame.data, 8);
            s_cached_profile1_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x02) { /* Directional Headlights */
            frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B;
            frame.dlc = 8;
            memcpy(frame.data, s_cached_profile1_raw, 8);
            frame.data[0] = 0x01;
            if (val) frame.data[2] |= 0x80;
            else     frame.data[2] &= ~0x80;
            memcpy(s_cached_profile1_raw, frame.data, 8);
            s_cached_profile1_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x03) { /* TPMS Calibration Pulse */
            frame.id = 0x221;
            frame.dlc = 8;
            frame.data[3] = 0x10;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x0B) { /* Driver Profile Select */
            frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B;
            frame.dlc = 8;
            memcpy(frame.data, s_cached_profile1_raw, 8);
            frame.data[0] = (uint8_t)((val & 0x03) + 1); /* Profile ID with bit 2 cleared */
            memcpy(s_cached_profile1_raw, frame.data, 8);
            s_cached_profile1_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x24) { /* Zone Temp Sync (Mono/Dual) */
            frame.id = 0x1D0;
            frame.dlc = 7;
            frame.data[0] = 0x01; /* DUAL_REQ */
            return hal_can_send(&frame);
        }
    }
    else if (cmd_id == 0x3B) {
        if (setting_id == 0x0F) { /* Climate Dual/Mono Sync (Section 3.3) */
            frame.id = 0x1D0;
            frame.dlc = 7;
            frame.data[0] = 0x01; /* DUAL_REQ */
            return hal_can_send(&frame);
        }
    }

    return HAL_STATUS_ERROR;
}

void psa_car_config_get_cached(vehicle_config_t *cfg) {
    if (cfg && s_cached_profile1_valid) {
        psa_decode_config_0x260(cfg, s_cached_profile1_raw, 8);
    }
}
